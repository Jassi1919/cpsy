import time, board, busio
import adafruit_icm20x

# I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# IMU device (autodetect addr 0x68 or 0x69)
imu = adafruit_icm20x.ICM20948(i2c)

print("ICM-20948 Quick Test: Ctrl+C to exit")

while True:
    mag_x, mag_y, mag_z = imu.magnetic       # in microteslas
    acc_x, acc_y, acc_z = imu.acceleration   # in m/s^2
    print(f"Mag(uT): {mag_x:6.2f}, {mag_y:6.2f}, {mag_z:6.2f} | "
          f"Acc(m/s²): {acc_x:6.2f}, {acc_y:6.2f}, {acc_z:6.2f}")
    time.sleep(0.5)