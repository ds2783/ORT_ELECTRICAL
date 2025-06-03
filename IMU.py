#!/usr/bin/env python

import time
from icm20948 import ICM20948

print("""
read-all-calibrated.py

Calibrates accelerometer, gyroscope, and magnetometer.
Ensure the IMU is:
- Still during accel/gyro calibration
- Moved in all directions during magnetometer calibration (figure-8)

Press Ctrl+C to exit!
""")

imu = ICM20948()

def calibrate_accel_gyro(samples=100, delay=0.01):
    print("Calibrating accelerometer and gyroscope... Keep IMU still.")
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

    # Subtract gravity (~1g) from Z
    accel_offset[2] -= 1.0

    print("Accel/Gyro calibration done.")
    return accel_offset, gyro_offset

def calibrate_magnetometer(duration=20, delay=0.05):
    print("\nCalibrating magnetometer. Slowly rotate the IMU in all directions...")
    print("Start moving now.")

    mag_min = [float('inf')] * 3
    mag_max = [float('-inf')] * 3

    start_time = time.time()
    while time.time() - start_time < duration:
        x, y, z = imu.read_magnetometer_data()
        mag_min[0] = min(mag_min[0], x)
        mag_min[1] = min(mag_min[1], y)
        mag_min[2] = min(mag_min[2], z)
        mag_max[0] = max(mag_max[0], x)
        mag_max[1] = max(mag_max[1], y)
        mag_max[2] = max(mag_max[2], z)
        time.sleep(delay)

    mag_offset = [(max_ + min_) / 2 for max_, min_ in zip(mag_max, mag_min)]

    print("Magnetometer calibration done.")
    print(f"Offsets: X={mag_offset[0]:.2f}, Y={mag_offset[1]:.2f}, Z={mag_offset[2]:.2f}")
    return mag_offset

# Calibrate sensors
accel_offset, gyro_offset = calibrate_accel_gyro()
mag_offset = calibrate_magnetometer()

print("\nCalibration complete. Streaming calibrated data...\n")

while True:
    # Read raw data
    mx, my, mz = imu.read_magnetometer_data()
    ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro_data()

    # Apply calibration
    ax -= accel_offset[0]
    ay -= accel_offset[1]
    az -= accel_offset[2]

    gx -= gyro_offset[0]
    gy -= gyro_offset[1]
    gz -= gyro_offset[2]

    mx -= mag_offset[0]
    my -= mag_offset[1]
    mz -= mag_offset[2]

    print(f"""
Accel: {ax:05.2f} {ay:05.2f} {az:05.2f}
Gyro:  {gx:05.2f} {gy:05.2f} {gz:05.2f}
Mag:   {mx:05.2f} {my:05.2f} {mz:05.2f}""")

    time.sleep(0.25)
