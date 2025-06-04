import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import Quaternion

import time
import numpy as np
from ahrs.filters import EKF
from scipy.spatial.transform import Rotation as R

from elysium.hardware.icm20948 import ICM20948


class imu_sensor(Node):
    def __init__(self):
        super().__init__("imu_sensor")
        self.imu = ICM20948(0x68)

        self.ekf = EKF()

        self.configure_sub_ = self.create_subscription(
            Bool, "/calibrate_imu", self.calibrateCB_, 10
        )
        self.quaternion_pub_ = self.create_publisher(Quaternion, "/imu_quat", 10)

    def calibrate_accel_gyro(self, samples=100, delay=0.01):
        self.get_logger().info(
            "Calibrating accelerometer and gyroscope... Keep IMU still."
        )
        accel_offset = [0.0, 0.0, 0.0]
        gyro_offset = [0.0, 0.0, 0.0]

        for _ in range(samples):
            ax, ay, az, gx, gy, gz = self.imu.read_accelerometer_gyro_data()
            accel_offset[0] += ax
            accel_offset[1] += ay
            accel_offset[2] += az
            gyro_offset[0] += gx
            gyro_offset[1] += gy
            gyro_offset[2] += gz
            time.sleep(delay)

        accel_offset = np.array([x / samples for x in accel_offset])
        gyro_offset = np.array([x / samples for x in gyro_offset])

        # Subtract gravity (~1g) from Z
        accel_offset[2] -= 1.0

        self.get_logger().info("Accel/Gyro calibration done.")
        return accel_offset, gyro_offset

    def calibrate_magnetometer(self, duration=20, delay=0.05):
        self.get_logger().info(
            "Calibrating magnetometer. Slowly rotate the IMU in all directions..."
        )
        self.get_logger().info("Start moving now.")

        mag_min = [float("inf")] * 3
        mag_max = [float("-inf")] * 3

        start_time = time.time()
        while time.time() - start_time < duration:
            x, y, z = self.imu.read_magnetometer_data()
            mag_min[0] = min(mag_min[0], x)
            mag_min[1] = min(mag_min[1], y)
            mag_min[2] = min(mag_min[2], z)
            mag_max[0] = max(mag_max[0], x)
            mag_max[1] = max(mag_max[1], y)
            mag_max[2] = max(mag_max[2], z)
            time.sleep(delay)

        mag_offset = np.array(
            [(max_ + min_) / 2 for max_, min_ in zip(mag_max, mag_min)]
        )

        self.get_logger().info("Magnetometer calibration done.")
        self.get_logger().info(
            f"Offsets: X={mag_offset[0]:.2f}, Y={mag_offset[1]:.2f}, Z={mag_offset[2]:.2f}"
        )
        return mag_offset

    def calibrateCB_(self, msg: Bool):
        self.accel_offset, self.gyro_offset = self.calibrate_accel_gyro()
        self.mag_offset = self.calibrate_magnetometer()
        self.get_logger().info("Calibration complte. Streaming calibration data...")

    def sendDataCB_(self):
        mx, my, mz = self.imu.read_magnetometer_data()
        mag_temp = (mx, my, mz)
        ax, ay, az, gx, gy, gz = self.imu.read_accelerometer_gyro_data()
        accel_temp = (ax, ay, az)
        gyro_temp = (gx, gy, gz)

        mag, accel, gyro = (
            mag_temp - self.mag_offset,
            accel_temp - self.accel_offset,
            gyro_temp - self.gyro_offset,
        )
        gyro_rad = np.deg2rad(gyro)

        q = np.array([1.0, 0.0, 0.0, 0.0])  # Initial quaternion
        # Update EKF and get updated quaternion
        q = self.ekf.update(q, gyr=gyro_rad, acc=accel, mag=mag)

        # Create message
        quat = Quaternion()
        quat.x = q.x
        quat.y = q.y
        quat.z = q.z
        quat.w = q.w
        self.quaternion_pub_.publish(quat)

        # Convert quaternion to Euler angles (ZYX order = yaw, pitch, roll)
        # euler = R.from_quat([q[1], q[2], q[3], q[0]]).as_euler('zyx', degrees=True)
        # yaw, pitch, roll = euler

def main(args=None):
    rclpy.init(args=args)

    # has to be initialised this way round!
    imu = imu_sensor()

    rclpy.spin(imu)
    # Cleanup After Shutdown
    rclpy.shutdown() 
