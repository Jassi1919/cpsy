#!/usr/bin/env python3
# ICM-20948 compass sanity + calibration + tilt-compensated heading
# Requires: pip3 install adafruit-circuitpython-icm20x

import time, math, sys
import board, busio
import adafruit_icm20x

# ---- USER KNOBS ----
I2C_ADDR_HINT = 0x68     # 0x68 if AD→GND, 0x69 if AD→3V3
SETTLE_S = 0.2           # loop delay
CAL_SWEEP_SEC = 25       # how long to rotate for min/max capture
DECLINATION_DEG = 0.0    # set your local magnetic declination (east +, west -)
# ---------------------

def heading_naive(mx, my):
    """Plain 2D atan2 heading (no tilt comp, no calibration)."""
    return (math.degrees(math.atan2(my, mx)) + 360.0) % 360.0

def tilt_compensated_heading(mx, my, mz, ax, ay, az):
    """Tilt-compensated heading using accel to remove roll & pitch."""
    # Normalize accelerometer vector
    an = math.sqrt(ax*ax + ay*ay + az*az)
    if an == 0:
        return float('nan')
    axn, ayn, azn = ax/an, ay/an, az/an

    # Roll & pitch (right-hand system, Z up)
    roll  = math.atan2(ayn, azn)
    pitch = math.atan2(-axn, math.sqrt(ayn*ayn + azn*azn))

    # Rotate mag vector into horizontal plane
    mx_h = mx*math.cos(pitch) + mz*math.sin(pitch)
    my_h = mx*math.sin(roll)*math.sin(pitch) + my*math.cos(roll) - mz*math.sin(roll)*math.cos(pitch)

    hdg = (math.degrees(math.atan2(my_h, mx_h)) + 360.0) % 360.0
    return hdg

def main():
    # I2C bringup
    i2c = busio.I2C(board.SCL, board.SDA)
    imu = adafruit_icm20x.ICM20948(i2c)  # addr is auto-detected by lib

    print("ICM-20948 ready. Press Ctrl+C to skip any step.")

    # ---- Step 1: sanity loop (raw + naive heading) ----
    print("\n[Step 1] Sanity: raw mag/acc + naive heading (flat board).")
    print("Rotate the board slowly; headings should change.")
    for _ in range(10):
        mx, my, mz = imu.magnetic        # microtesla
        ax, ay, az = imu.acceleration    # m/s^2
        hdg = heading_naive(mx, my)
        print(f"Mag(uT)=({mx:6.2f},{my:6.2f},{mz:6.2f})  Acc(m/s^2)=({ax:5.2f},{ay:5.2f},{az:5.2f})  "
              f"Naive={hdg:6.1f}°")
        time.sleep(SETTLE_S)

    # ---- Step 2: quick hard-iron calibration (min/max) ----
    print("\n[Step 2] Hard-iron calibration sweep.")
    print("Rotate the sensor through all orientations (figure 8 / full turns) for ~{} s...".format(CAL_SWEEP_SEC))
    t0 = time.time()
    mx_min = my_min = mz_min =  1e9
    mx_max = my_max = mz_max = -1e9
    while time.time() - t0 < CAL_SWEEP_SEC:
        mx, my, mz = imu.magnetic
        mx_min, my_min, mz_min = min(mx_min, mx), min(my_min, my), min(mz_min, mz)
        mx_max, my_max, mz_max = max(mx_max, mx), max(my_max, my), max(mz_max, mz)
        time.sleep(0.02)

    # Offsets (hard-iron)
    mx_off = (mx_min + mx_max) / 2.0
    my_off = (my_min + my_max) / 2.0
    mz_off = (mz_min + mz_max) / 2.0

    # Soft-iron scale (simple axis-wise scale; optional)
    mx_scale = (mx_max - mx_min) / 2.0
    my_scale = (my_max - my_min) / 2.0
    mz_scale = (mz_max - mz_min) / 2.0
    avg_scale = (mx_scale + my_scale + mz_scale) / 3.0
    sx, sy, sz = avg_scale/mx_scale if mx_scale else 1, avg_scale/my_scale if my_scale else 1, avg_scale/mz_scale if mz_scale else 1

    print("Calibration results:")
    print(f"  Offsets: mx_off={mx_off:.3f}, my_off={my_off:.3f}, mz_off={mz_off:.3f}")
    print(f"  Scales : sx={sx:.3f}, sy={sy:.3f}, sz={sz:.3f}")
    print("Apply these, then compute tilt-compensated + declination-corrected heading.\n")

    # ---- Step 3: calibrated, tilt-compensated heading ----
    print("[Step 3] Calibrated, tilt-compensated heading (with declination).")
    print("Press Ctrl+C to stop.\n")
    while True:
        mx, my, mz = imu.magnetic
        ax, ay, az = imu.acceleration

        # Apply hard-iron offset and (optional) soft-iron scale
        mx_c = (mx - mx_off) * sx
        my_c = (my - my_off) * sy
        mz_c = (mz - mz_off) * sz

        hdg_tc = tilt_compensated_heading(mx_c, my_c, mz_c, ax, ay, az)
        if math.isnan(hdg_tc):
            print("Accel norm zero; skipping sample.")
            time.sleep(SETTLE_S)
            continue

        # Declination (east positive, west negative)
        hdg_true = (hdg_tc + DECLINATION_DEG + 360.0) % 360.0

        print(f"MagC(uT)=({mx_c:6.2f},{my_c:6.2f},{mz_c:6.2f})  "
              f"TC={hdg_tc:6.1f}°  True={hdg_true:6.1f}° (decl={DECLINATION_DEG:+.1f}°)")
        time.sleep(SETTLE_S)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nDone.")