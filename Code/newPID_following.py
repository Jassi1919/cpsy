from gpiozero import Robot, Motor, DigitalOutputDevice
import time
import board
import busio
import adafruit_tcs34725
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306

# -----------------------
# Hardware setup
# -----------------------
slp = DigitalOutputDevice('GPIO26')
slp.on()  # Enable motor driver
robot = Robot(left=('GPIO12','GPIO18'), right=('GPIO13','GPIO19'))

i2c = busio.I2C(board.SCL, board.SDA)
tcs = adafruit_tcs34725.TCS34725(i2c)
tcs.integration_time = 50   # ms (2.4, 24, 50, 101, 154, 700)
tcs.gain = 4                # 1, 4, 16, 60

# OLED setup
oled = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c)
oled.fill(0); oled.show()
def display_status(happy=True, text=""):
    oled.fill(0)
    image = Image.new("1", (oled.width, oled.height))
    draw = ImageDraw.Draw(image)
    if text:
        font = ImageFont.load_default()
        draw.text((10, 28), text, font=font, fill=255)
    else:
        cx, cy, r = 64, 32, 25
        draw.ellipse((cx - r, cy - r, cx + r, cy + r), outline=255)
        draw.ellipse((cx - 10, cy - 10, cx - 5, cy - 5), fill=255)
        draw.ellipse((cx + 5, cy - 10, cx + 10, cy - 5), fill=255)
        if happy:
            draw.arc((cx - 10, cy - 5, cx + 10, cy + 15), start=0, end=180, fill=255)
        else:
            draw.arc((cx - 10, cy + 5, cx + 10, cy + 15), start=180, end=360, fill=255)
    oled.image(image); oled.show()

display_status(text="Jason"); time.sleep(0.8); display_status(happy=True)

# -----------------------
# PID + control params
# -----------------------
# Faster mapping to motor speed than before
SPEED_SCALE = 0.32   # was ~0.17 → increase overall speed
BASE_SPEED_PWM = 310 # was 255; center speed in "PWM-like" domain

MIN_PWM = 0
MAX_PWM = 500

# Slew limiting (prevents sudden big jumps → fewer line dropouts at speed)
MAX_PWM_STEP = 40     # max change per loop per wheel in PWM-like units

# PID (retuned for higher speed: more D, a bit less P/I)
Kp = 8.0
Ki = 1.5
Kd = 110.0

# Integral bounds (units: error * seconds)
I_MIN = -8.0
I_MAX =  8.0

# If error tiny, zero integral (reduces slow drift)
INTEGRAL_DEADZONE = 6.0

# Target brightness (tune this to your line/background)
middle_value = 3300
black_value  =  600
white_value  = 6000

# PID state
error = 0.0
previous_error = 0.0
sum_error = 0.0

# Filter state (EMA on clear channel)
ema_c = None
EMA_ALPHA = 0.25  # 0..1 (higher = more responsive, lower = smoother)

# Stuck detection
last_sensor_readings = []
STUCK_THRESHOLD = 120
STUCK_COUNT = 0
last_correction = 0.0

# Slew state
prev_left_pwm = BASE_SPEED_PWM
prev_right_pwm = BASE_SPEED_PWM

# -----------------------
# Utility helpers
# -----------------------
def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def pwm_to_robot_speed(pwm_value):
    pwm_value = clamp(pwm_value, 0, 255)
    # scale up for higher top speed
    return (pwm_value / 255.0) * SPEED_SCALE

def limit_slew(prev_val, target_val, max_step):
    if target_val > prev_val + max_step:
        return prev_val + max_step
    if target_val < prev_val - max_step:
        return prev_val - max_step
    return target_val

def set_motors(left_pwm_signed, right_pwm_signed):
    # Direction + magnitude split, with mapping to Robot.value
    left_sign  = 1 if left_pwm_signed  >= 0 else -1
    right_sign = 1 if right_pwm_signed >= 0 else -1
    left_mag   = pwm_to_robot_speed(abs(int(left_pwm_signed)))
    right_mag  = pwm_to_robot_speed(abs(int(right_pwm_signed)))
    robot.value = (left_sign * left_mag, right_sign * right_mag)

def stop():
    robot.stop()

def read_clear_channel():
    r, g, b, c = tcs.color_raw
    return r, g, b, c

def read_clear_filtered():
    global ema_c
    _, _, _, c = read_clear_channel()
    if ema_c is None:
        ema_c = float(c)
    else:
        ema_c = EMA_ALPHA * float(c) + (1.0 - EMA_ALPHA) * ema_c
    return int(ema_c)

# -----------------------
# Main control loop
# -----------------------
def loop(dt):
    global error, previous_error, sum_error
    global last_sensor_readings, STUCK_COUNT, last_correction
    global prev_left_pwm, prev_right_pwm

    # Read sensor (filtered)
    c = read_clear_filtered()

    # Debug print (throttled)
    # print(f"Sensor: C={c}, Target={middle_value}, Err={middle_value - c:.1f}")

    # Keep last readings for stuck detection
    last_sensor_readings.append(c)
    if len(last_sensor_readings) > 4:
        last_sensor_readings.pop(0)
    if len(last_sensor_readings) == 4:
        sensor_range = max(last_sensor_readings) - min(last_sensor_readings)
        if sensor_range < STUCK_THRESHOLD:
            STUCK_COUNT += 1
        else:
            STUCK_COUNT = 0

    # If stuck for 10 readings, do directional wiggle
    if STUCK_COUNT >= 10:
        display_status(happy=False)
        print(f"STUCK: wiggle (last corr={last_correction:.1f})")
        for i in range(4):  # 4 * 75ms = 300ms wiggle
            if last_correction > 0:   # was steering left
                # mostly left, brief right
                set_motors(-250, 250) if i < 2 else set_motors(120, -120)
            else:                     # was steering right/straight
                set_motors(250, -250) if i < 2 else set_motors(-120, 120)
            time.sleep(0.075)
        last_sensor_readings = []
        STUCK_COUNT = 0
        return

    # PID with dt
    error = float(middle_value) - float(c)
    if abs(error) < INTEGRAL_DEADZONE:
        sum_error = 0.0
    else:
        sum_error += error * dt
        sum_error = clamp(sum_error, I_MIN, I_MAX)

    derivative = (error - previous_error) / dt if dt > 0 else 0.0
    correction = Kp * error + Ki * sum_error + Kd * derivative
    last_correction = correction

    # Clamp correction
    correction = clamp(correction, -220, 220)
    previous_error = error

    # Compute wheel commands
    target_left  = BASE_SPEED_PWM + int(correction)
    target_right = BASE_SPEED_PWM - int(correction)

    # Clamp
    target_left  = clamp(target_left,  MIN_PWM, MAX_PWM)
    target_right = clamp(target_right, MIN_PWM, MAX_PWM)

    # Slew limit
    target_left  = int(limit_slew(prev_left_pwm,  target_left,  MAX_PWM_STEP))
    target_right = int(limit_slew(prev_right_pwm, target_right, MAX_PWM_STEP))

    prev_left_pwm, prev_right_pwm = target_left, target_right

    # Debug
    # print(f"dt={dt:.3f} corr={correction:.1f} | L={target_left} R={target_right}")

    # Apply + happy face
    set_motors(target_left, target_right)
    display_status(happy=True)

def main():
    print("Starting up...")
    # Quick sensor presence sanity check
    try:
        _ = tcs.color_raw
        print("TCS34725 detected.")
    except Exception as e:
        print(f"ERROR: TCS34725 not detected or I2C issue: {e}")
        return

    print("Control loop. Ctrl+C to stop.")
    try:
        last = time.monotonic()
        while True:
            now = time.monotonic()
            dt = now - last
            last = now
            # Safety clamp on dt (protects derivative on occasional hiccups)
            if dt <= 0 or dt > 0.2:
                dt = 0.02
            loop(dt)
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        stop()
        try:
            oled.fill(0); oled.show()
            print("OLED cleared.")
        except Exception:
            pass

if __name__ == "__main__":
    main()