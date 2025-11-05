#!/usr/bin/env python3
"""
motor_bursts_straight.py — straight-line burst testing only (forward/backward).

- Longer duration to observe drift
- Per-motor speed multipliers to compensate weak side
- Optional per-side stop offsets to test end-of-burst drift
- 3s pre-action delay before each run
"""

import time
from gpiozero import DigitalOutputDevice, Robot

# ---------------- USER SETTINGS ----------------
SPEED_BOTH    = 0.25   # base speed for both-motor bursts (0..1)
DUR_BOTH      = 3.0    # duration for each forward/backward run (seconds)
REPEATS       = 2      # how many forward/backward cycles

PRE_DELAY     = 2.0    # seconds before each run (for clear observation)
PAUSE         = 0.8    # pause between runs (seconds)
COUNTDOWN     = 3      # startup countdown (seconds)

# Per-motor scaling to balance strength (1.00 = no change)
LEFT_MULTIPLIER  = 1.00
RIGHT_MULTIPLIER = 1.57   # e.g., right wheel slightly weaker → bump it up

# OPTIONAL: stop timing offsets (milliseconds)
# Positive value = stop that side LATER; Negative = stop EARLIER
STOP_OFFSET_LEFT_MS  = 0
STOP_OFFSET_RIGHT_MS = 60   # try delaying the right stop a touch to counter early cutout
# ------------------------------------------------

# GPIO mapping
SLP_GPIO   = 'GPIO26'
LEFT_PINS  = ('GPIO12', 'GPIO18')  # (forward, backward)
RIGHT_PINS = ('GPIO13', 'GPIO19')  # (forward, backward)

def wait_and_log(label):
    print(f"\n→ Preparing for {label} in {int(PRE_DELAY)}s…")
    time.sleep(PRE_DELAY)

def drive_both(robot: Robot, direction: str, base_speed: float, dur: float,
               left_mult: float, right_mult: float):
    # compute signed speeds
    l = base_speed * left_mult
    r = base_speed * right_mult
    if direction == "forward":
        cmd = (l, r)
    else:
        cmd = (-l, -r)

    print(f"[{time.strftime('%H:%M:%S')}] BOTH {direction:<8} "
          f"L={abs(cmd[0]):.2f} (x{left_mult:.2f})  R={abs(cmd[1]):.2f} (x{right_mult:.2f})  for {dur:.2f}s")

    wait_and_log(f"both motors {direction}")
    robot.value = cmd
    t_start = time.monotonic()
    while time.monotonic() - t_start < dur:
        time.sleep(0.01)

    # coordinated stop with optional per-side offsets
    # apply offset by briefly driving only the other side to zero first
    # Note: offsets are small (ms); this is just for testing end-drift
    if STOP_OFFSET_LEFT_MS == STOP_OFFSET_RIGHT_MS == 0:
        robot.stop()
    else:
        # determine which to stop first
        l_delay = max(0, STOP_OFFSET_LEFT_MS) / 1000.0
        r_delay = max(0, STOP_OFFSET_RIGHT_MS) / 1000.0
        # stop the side with smaller (or negative) offset first
        # stop order: earliest offset side → latest offset side
        if l_delay <= r_delay:
            # stop left first
            robot.value = (0.0, cmd[1])  # left to 0, keep right
            time.sleep(r_delay - l_delay)
            robot.value = (0.0, 0.0)
        else:
            # stop right first
            robot.value = (cmd[0], 0.0)  # right to 0, keep left
            time.sleep(l_delay - r_delay)
            robot.value = (0.0, 0.0)

    time.sleep(PAUSE)

def main():
    slp = DigitalOutputDevice(SLP_GPIO)
    robot = Robot(left=LEFT_PINS, right=RIGHT_PINS)

    try:
        print("=== STRAIGHT-LINE MOTOR TEST ===")
        for i in range(COUNTDOWN, 0, -1):
            print(f"Starting in {i}…")
            time.sleep(1)

        slp.on()
        print("Motor driver (SLP) enabled.\n")

        for k in range(1, REPEATS + 1):
            print(f"\n--- Cycle {k}/{REPEATS} ---")
            drive_both(robot, "forward",  SPEED_BOTH, DUR_BOTH, LEFT_MULTIPLIER, RIGHT_MULTIPLIER)
            drive_both(robot, "backward", SPEED_BOTH, DUR_BOTH, LEFT_MULTIPLIER, RIGHT_MULTIPLIER)

        print("\n✅ Test finished.")
        print("Notes:")
        print("- If it still drifts right, slightly increase RIGHT_MULTIPLIER (e.g., +0.02).")
        print("- If drift happens only at the end of a run, adjust STOP_OFFSET_*_MS.")
        print("- Re-run to iterate until straight and symmetric.")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        robot.stop()
        slp.off()
        print("Motors stopped and SLP disabled.")

if __name__ == "__main__":
    main()
