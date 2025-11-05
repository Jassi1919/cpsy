#!/usr/bin/env python3
from gpiozero import Robot, DigitalOutputDevice
import time
import board, busio
import adafruit_tcs34725
from collections import deque

# ---------------- Hardware ----------------
SLP_GPIO = 'GPIO26'
robot = Robot(left=('GPIO12','GPIO18'), right=('GPIO13','GPIO19'))
slp = DigitalOutputDevice(SLP_GPIO); slp.on()

i2c = busio.I2C(board.SCL, board.SDA)
tcs = adafruit_tcs34725.TCS34725(i2c)
tcs.integration_time = 50    # ms
tcs.gain = 4                 # 1,4,16,60

# ---------------- PID & speeds ----------------
Kp, Ki, Kd = 10, 2, 90

BASE_SPEED_PWM = 90          # cruise baseline
MIN_PWM, MAX_PWM = 0, 200
I_MIN, I_MAX = -4, 4
INTEGRAL_DEADZONE = 4.0
CORR_CLAMP = 100

# Anti-stall helpers
MIN_RUN_PWM = 70             # floor when non-zero
KICK_PWM = 140
KICK_MS = 120
KICK_COOLDOWN_MS = 600
_last_kick_ms = 0

# --- Lost-line detection / recovery ---
C_HISTORY = 18               # samples for variance check
C_EPS = 25                   # "not changing" window
SAT_LIMIT = 20               # cycles of |corr|==CORR_CLAMP to trigger recovery

# Auto-cal (optional). Hold over white, then over black at startup if you want
AUTO_CAL = False

# Targets (can be overridden by auto-cal)
black_value, white_value = 600, 6000
middle_value = 3300

# ---------------- Motor balance (your low-speed data) ----------------
def motor_balance(speed):
    """Return (L_scale, R_scale) for low-speed bias."""
    low_s, low_r = 0.10, 1.58
    mid_s, mid_r = 0.25, 1.57
    s = max(min(speed, mid_s), low_s)
    t = (s - low_s) / (mid_s - low_s) if mid_s != low_s else 0.0
    right_mult = low_r + (mid_r - low_r) * t
    return 1.00, right_mult

# ---------------- Helpers ----------------
def clamp(x, lo, hi): return lo if x < lo else hi if x > hi else x

def pwm_to_robot_speed(pwm_value):
    pwm_value = clamp(pwm_value, 0, 255)
    return (pwm_value / 255.0) * 0.17

def _now_ms(): return int(time.monotonic() * 1000)

def _apply_floor_and_kick(L_cmd, R_cmd):
    global _last_kick_ms
    absL, absR = abs(L_cmd), abs(R_cmd)

    def lift(cmd):
        if cmd == 0: return 0
        return (MIN_RUN_PWM if cmd > 0 else -MIN_RUN_PWM) if abs(cmd) < MIN_RUN_PWM else cmd

    L2, R2 = lift(L_cmd), lift(R_cmd)

    now = _now_ms()
    need_kick = (0 < absL < MIN_RUN_PWM) and (0 < absR < MIN_RUN_PWM)
    if need_kick and (now - _last_kick_ms) > KICK_COOLDOWN_MS:
        Lk = KICK_PWM if L_cmd >= 0 else -KICK_PWM
        Rk = KICK_PWM if R_cmd >= 0 else -KICK_PWM
        robot.value = (
            (1 if Lk > 0 else -1) * pwm_to_robot_speed(abs(Lk)),
            (1 if Rk > 0 else -1) * pwm_to_robot_speed(abs(Rk)),
        )
        time.sleep(KICK_MS/1000.0)
        _last_kick_ms = now
        L2, R2 = lift(L_cmd), lift(R_cmd)

    return L2, R2

def set_motors(left_pwm_signed, right_pwm_signed):
    left_pwm_signed, right_pwm_signed = _apply_floor_and_kick(left_pwm_signed, right_pwm_signed)

    left_sign  = 1 if left_pwm_signed  >= 0 else -1
    right_sign = 1 if right_pwm_signed >= 0 else -1
    left_mag   = pwm_to_robot_speed(abs(int(left_pwm_signed)))
    right_mag  = pwm_to_robot_speed(abs(int(right_pwm_signed)))

    # Dynamic balance from current magnitude
    ref_speed = max(left_mag, right_mag, 0.10)
    Ls, Rs = motor_balance(ref_speed)
    left_mag  = clamp(left_mag  * Ls, 0, 1)
    right_mag = clamp(right_mag * Rs, 0, 1)

    robot.value = (left_sign * left_mag, right_sign * right_mag)

def stop(): robot.stop()

def read_clear_channel():
    r, g, b, c = tcs.color_raw
    return c

# ---------------- Auto-cal (optional) ----------------
def quick_autocal():
    global black_value, white_value, middle_value
    print("\nAuto-cal: place sensor over WHITE, press Enter…"); input()
    ws = [read_clear_channel() for _ in range(20)]; time.sleep(0.05)
    white_value = sum(ws)//len(ws)
    print(f"  white ≈ {white_value}")

    print("Auto-cal: place sensor over BLACK, press Enter…"); input()
    bs = [read_clear_channel() for _ in range(20)]; time.sleep(0.05)
    black_value = sum(bs)//len(bs)
    print(f"  black ≈ {black_value}")

    middle_value = (white_value + black_value)//2
    print(f"  middle set to {middle_value}\n")

# ---------------- Recovery logic ----------------
def recover_search(last_corr_sign):
    """
    Actively search for the line:
    - If we were saturated +100 (turn-left), pivot RIGHT a bit first, then sweep left.
    - Opposite if saturated -100.
    """
    print(">>> LOST LINE: starting recovery sweep")
    # Stronger, reliable pivot speeds
    SW_PWM_FAST = 170
    SW_PWM_SOFT = 120
    SW_T1 = 0.25
    SW_T2 = 0.35

    # Turn opposite of last saturation first
    if last_corr_sign > 0:   # we were turning left → try right first
        # right pivot (left backward, right forward) small
        set_motors(-SW_PWM_SOFT, +SW_PWM_SOFT); time.sleep(SW_T1)
        # then sweep the other way a bit longer
        set_motors(+SW_PWM_FAST, -SW_PWM_FAST); time.sleep(SW_T2)
    else:                    # we were turning right → try left first
        set_motors(+SW_PWM_SOFT, -SW_PWM_SOFT); time.sleep(SW_T1)
        set_motors(-SW_PWM_FAST, +SW_PWM_FAST); time.sleep(SW_T2)

    stop()
    print(">>> RECOVERY DONE")

# ---------------- Main loop ----------------
def main():
    global middle_value

    print("Starting line following with recovery…")
    try:
        _ = tcs.color_raw
        print("TCS34725 detected.")
    except Exception as e:
        print(f"Sensor error: {e}")
        return

    if AUTO_CAL:
        quick_autocal()

    # PID state
    error = 0.0
    previous_error = 0.0
    sum_error = 0.0

    c_hist = deque(maxlen=C_HISTORY)
    sat_count = 0
    last_corr_sign = 0

    try:
        while True:
            c = read_clear_channel()
            c_hist.append(c)

            # PID
            error = float(middle_value) - float(c)
            if abs(error) < INTEGRAL_DEADZONE:
                sum_error = 0.0
            else:
                sum_error += error
                sum_error = clamp(sum_error, I_MIN, I_MAX)

            differential = error - previous_error
            correction = Kp * error + Ki * sum_error + Kd * differential
            correction = clamp(correction, -CORR_CLAMP, CORR_CLAMP)
            previous_error = error

            # Detect saturation
            if abs(correction) >= CORR_CLAMP:
                sat_count += 1
                last_corr_sign = 1 if correction > 0 else -1
            else:
                sat_count = 0

            # Detect "no change" in sensor
            if len(c_hist) == C_HISTORY:
                c_range = max(c_hist) - min(c_hist)
            else:
                c_range = C_EPS + 1  # don't trigger early

            # Recovery condition
            if sat_count >= SAT_LIMIT and c_range <= C_EPS:
                recover_search(last_corr_sign)
                # reset states after recovery
                sat_count = 0
                sum_error = 0.0
                previous_error = 0.0
                c_hist.clear()
                continue

            # Speeds
            L = clamp(int(BASE_SPEED_PWM + correction), MIN_PWM, MAX_PWM)
            R = clamp(int(BASE_SPEED_PWM - correction), MIN_PWM, MAX_PWM)

            # Debug (trim if too chatty)
            print(f"C={c:4d}  Err={error:+6.1f}  Corr={correction:+6.1f}  L={L:3d}  R={R:3d}")

            set_motors(L, R)
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        stop()
        slp.off()

if __name__ == "__main__":
    main()
