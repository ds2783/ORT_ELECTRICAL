#!/usr/bin/env python

import time
from icm20948 import ICM20948

print("""
read-all-calibrated.py

Reads calibrated accelerometer, gyroscope, and magnetometer data.
Ensure the IMU is stationary for a few seconds during calibration.

Press Ctrl+C to exit!
""")

imu = ICM20948()

def calibrate_imu(samples=100, delay=0.01):
    print("Calibrating... Please keep the IMU stationary.")
    accel_offset = [0.0, 0.0, 0.0]
    gyro_offset = [0.0, 0.0, 0.0]

    for _ in range(samples):
        ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro_data()
        accel_offset[0] += ax
        accel_offset[1] += ay
        accel_offset[2] += az
        gyro_offset[0] += gx
        gyro_offset[1] += gy
        gyro_offset[2] += gz
        time.sleep(delay)

    accel_offset = [x / samples for x in accel_offset]
    gyro_offset = [x / samples for x in gyro_offset]

    # Gravity compensation on Z-axis
    accel_offset[2] -= 1.0  # assuming Z points up and stationary IMU reads ~1g

    print("Calibration complete.")
    print(f"Accel Offset: {accel_offset}")
    print(f"Gyro Offset:  {gyro_offset}")
    return accel_offset, gyro_offset

accel_offset, gyro_offset = calibrate_imu()

while True:
    x, y, z = imu.read_magnetometer_data()
    ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro_data()

    ax -= accel_offset[0]
    ay -= accel_offset[1]
    az -= accel_offset[2]

    gx -= gyro_offset[0]
    gy -= gyro_offset[1]
    gz -= gyro_offset[2]

    print(f"""
Accel: {ax:05.2f} {ay:05.2f} {az:05.2f}
Gyro:  {gx:05.2f} {gy:05.2f} {gz:05.2f}
Mag:   {x:05.2f} {y:05.2f} {z:05.2f}""")

    time.sleep(0.25)
