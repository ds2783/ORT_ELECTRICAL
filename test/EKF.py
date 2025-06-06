#!/usr/bin/env python

import numpy as np
from ahrs.filters import EKF
from scipy.spatial.transform import Rotation as R
import time

def get_sensor_data():
    # Simulate a small rotation around Z
    accel = np.array([0.0, 0.0, 1.0])
    gyro = np.array([0.0, 0.0, 5.0])  # degrees/sec
    mag = np.array([30.0, 0.0, 40.0])
    return accel, gyro, mag

# Initialize EKF filter and orientation state
ekf = EKF()
q = np.array([1.0, 0.0, 0.0, 0.0])  # Initial quaternion

print("Starting EKF sensor fusion...\nPress Ctrl+C to stop.")

try:
    while True:
        accel, gyro, mag = get_sensor_data()

        # Convert gyro from deg/s to rad/s
        gyro_rad = np.deg2rad(gyro)

        # Update EKF and get updated quaternion
        q = ekf.update(q, gyr=gyro_rad, acc=accel, mag=mag)

        # Convert quaternion to Euler angles (ZYX order = yaw, pitch, roll)
        euler = R.from_quat([q[1], q[2], q[3], q[0]]).as_euler('zyx', degrees=True)
        yaw, pitch, roll = euler

        # Output quaternion and Euler angles
        print(f"Quaternion: [{q[0]: .4f}, {q[1]: .4f}, {q[2]: .4f}, {q[3]: .4f}]")
        print(f"Euler Angles (deg): Yaw: {yaw: .2f}, Pitch: {pitch: .2f}, Roll: {roll: .2f}")
        print("-" * 60)

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nStopped.")
