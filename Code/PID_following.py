from gpiozero import Robot, Motor, DigitalOutputDevice
import time
import board
import busio
import adafruit_tcs34725
import math

# -----------------------
# Hardware setup (yours)
# -----------------------
slp = DigitalOutputDevice('GPIO26')
slp.on()  # Enable motor driver
robot = Robot(left=('GPIO12','GPIO18'), right=('GPIO13','GPIO19'))

i2c = busio.I2C(board.SCL, board.SDA)
tcs = adafruit_tcs34725.TCS34725(i2c)
tcs.integration_time = 50   # ms (valid options in this lib include 2.4, 24, 50, 101, 154, 700)
tcs.gain = 4                # valid gains: 1, 4, 16, 60

# -----------------------
# PID + control params
# -----------------------
Kp = 10
Ki = 2
Kd = 90

# The original code used "PWM" in [0..255]. We'll compute in that domain,
# then map to Robot's [-1..1].
BASE_SPEED_PWM = 255

# Clamp speeds to keep some torque but avoid slamming max

MIN_PWM = 0
MAX_PWM = 500

# Integral windup guard (same spirit as original)
I_MIN = -4
I_MAX = 4

# If error is tiny, zero the integral to avoid drift
INTEGRAL_DEADZONE = 4.0

# The target "middle" clear channel value (overwritten by calibration)
middle_value = 3300
black_value = 600
white_value = 6000

# PID state
error = 0.0
previous_error = 0.0
sum_error = 0.0

# -----------------------
# Utility helpers
# -----------------------
def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def pwm_to_robot_speed(pwm_value):
    pwm_value = clamp(pwm_value, 0, 255)
    pwm_value = (pwm_value / 255.0) * 0.170
    return pwm_value

def set_motors(left_pwm_signed, right_pwm_signed):
    """
    Accept signed "PWM-like" speeds in range [-255..255],
    map to Robot.value which is tuple in [-1..1].
    """
    # Direction + magnitude split
    left_sign = 1 if left_pwm_signed >= 0 else -1
    right_sign = 1 if right_pwm_signed >= 0 else -1

    left_mag = pwm_to_robot_speed(abs(int(left_pwm_signed)))
    right_mag = pwm_to_robot_speed(abs(int(right_pwm_signed)))

    # Robot.value expects (-1..1) per side
    robot.value = (left_sign * left_mag, right_sign * right_mag)

def stop():
    robot.stop()

def read_clear_channel():
    # Returns (r,g,b,c); we use c
    r, g, b, c = tcs.color_raw
    return r, g, b, c

# -----------------------
# Main control loop
# -----------------------
# Add these global variables at the top with other globals
last_sensor_readings = []
STUCK_THRESHOLD = 120  # If sensor varies less than this, we're stuck
STUCK_COUNT = 0
last_correction = 0

def loop():
    global error, previous_error, sum_error, last_sensor_readings, STUCK_COUNT, last_correction

    # Read sensor
    _, _, _, c = read_clear_channel()
   
    # DEBUG: Print sensor reading and calculated values
    print(f"Sensor: C={c}, Target={middle_value}, Error={float(middle_value) - float(c):.1f}")
   
    # Keep last 5 sensor readings
    last_sensor_readings.append(c)
    if len(last_sensor_readings) > 4:
        last_sensor_readings.pop(0)
   
    # Check if we're stuck (sensor reading not changing much)
    if len(last_sensor_readings) == 4:
        sensor_range = max(last_sensor_readings) - min(last_sensor_readings)
        if sensor_range < STUCK_THRESHOLD:
            STUCK_COUNT += 1
        else:
            STUCK_COUNT = 0
   
    # If stuck for 10 readings, do directional wiggle
    if STUCK_COUNT >= 10:
        print(f"ROVER STUCK! Doing directional wiggle (correction was: {last_correction:.1f})...")
       
        # Determine wiggle direction based on last correction
        if last_correction > 0:  # Was trying to go left
            print("Wiggling LEFT")
            # Wiggle pattern: mostly left with some right
            for i in range(4):  # 4 cycles of 0.075s each
                if i % 4 == 0 or i % 4 == 1:  # First two cycles: hard left
                    set_motors(-250, 250)  # Hard left
                else:  # Last two cycles: slight right
                    set_motors(100, -100)  # Gentle right
                time.sleep(0.075)
        else:  # Was trying to go right (or straight)
            print("Wiggling RIGHT")  
            # Wiggle pattern: mostly right with some left
            for i in range(4):  # 4 cycles of 0.075s each
                if i % 4 == 0 or i % 4 == 1:  # First two cycles: hard right
                    set_motors(250, -250)  # Hard right
                else:  # Last two cycles: slight left
                    set_motors(-100, 100)  # Gentle left
                time.sleep(0.075)
       
        # Reset
        last_sensor_readings = []
        STUCK_COUNT = 0
        return

    # PID
    error = float(middle_value) - float(c)
    sum_error += error
    sum_error = clamp(sum_error, I_MIN, I_MAX)

    if abs(error) < INTEGRAL_DEADZONE:
        sum_error = 0.0

    differential = error - previous_error
    correction = Kp * error + Ki * sum_error + Kd * differential
   
    # Store the correction direction for stuck recovery
    last_correction = correction
   
    # Clamp correction to reasonable range
    correction = clamp(correction, -200, 200)
   
    previous_error = error

    # Compute wheel speeds in "PWM-like" domain
    speed_left = BASE_SPEED_PWM + int(correction)
    speed_right = BASE_SPEED_PWM - int(correction)

    # Clamp
    speed_left = clamp(speed_left, MIN_PWM, MAX_PWM)
    speed_right = clamp(speed_right, MIN_PWM, MAX_PWM)

    # DEBUG: Print motor speeds and correction
    print(f"Correction: {correction:.1f}, Left: {speed_left}, Right: {speed_right}")

    # Apply
    set_motors(speed_left, speed_right)

def main():
    print("Starting up...")
    # Quick sensor presence sanity check
    try:
        _ = tcs.color_raw
        print("TCS34725 detected.")
    except Exception as e:
        print(f"ERROR: TCS34725 not detected or I2C issue: {e}")
        return

    #calibrate_sensor()
    print("Entering control loop. Press Ctrl+C to stop.")

    try:
        while True:
            loop()
            # Match typical Arduino loop cadence (integration time is 50ms; keep a modest pace)
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        stop()

if __name__ == "__main__":
    main()