import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.action.server import ActionServer

from geometry_msgs.msg import Quaternion

import time
import numpy as np
from threading import Thread
from ahrs.filters import EKF

from elysium.hardware.icm20948 import ICM20948
from ort_interfaces.action import CalibrateImu


class Imu(Node):
    def __init__(self, sleep_node, i2c_addr=0x68):
        super().__init__("imu_sensor")
        self.imu = ICM20948(i2c_addr)

        self.ekf = EKF()

        # Topics -------------
        self.quaternion_pub_ = self.create_publisher(Quaternion, "/imu/quat", 10)
        # -------------------

        # Action Server ------
        self.calibrate_action_server_ = ActionServer(self, CalibrateImu, "/imu/calibrate", self.actionServerCB_) 
        # --------------------
        
        # Timer -----------
        self.imu_data_timer_ = self.create_timer(0.2, self.sendDataCB_)
        # -----------------

        # Variables ---------
        self.mag_offset = self.gyro_offset = self.accel_offset = 0
        self.sleep_node = sleep_node
        self.rate = self.sleep_node.create_rate(1)

    def calibrate_accel_gyro(self, samples=100, delay=0.01):
        self.get_logger().info(
            "Calibrating accelerometer and gyroscope... Keep IMU still."
        )
        accel_offset = [0.0, 0.0, 0.0]
        gyro_offset = [0.0, 0.0, 0.0]

        self.rate = self.sleep_node.create_rate(1/delay)
        
        for _ in range(samples):
            ax, ay, az, gx, gy, gz = self.imu.read_accelerometer_gyro_data()
            accel_offset[0] += ax
            accel_offset[1] += ay
            accel_offset[2] += az
            gyro_offset[0] += gx
            gyro_offset[1] += gy
            gyro_offset[2] += gz
            self.rate.sleep()

        accel_offset = np.array([x / samples for x in accel_offset])
        gyro_offset = np.array([x / samples for x in gyro_offset])

        # Subtract gravity (~1g) from Z
        accel_offset[2] -= 1.0

        self.get_logger().info("Accel/Gyro calibration done.")
        return accel_offset, gyro_offset

    def calibrate_magnetometer(self, goal_handle, feedback_msg, duration=20, delay=0.05):
        self.get_logger().info(
            "Calibrating magnetometer. Slowly rotate the IMU in all directions..."
        )
        self.get_logger().info("Start moving now.")

        mag_min = [float("inf")] * 3
        mag_max = [float("-inf")] * 3

        self.rate = self.sleep_node.create_rate(1/delay)

        start_time = time.time()
        while time.time() - start_time < duration:
            x, y, z = self.imu.read_magnetometer_data()
            mag_min[0] = min(mag_min[0], x)
            mag_min[1] = min(mag_min[1], y)
            mag_min[2] = min(mag_min[2], z)
            mag_max[0] = max(mag_max[0], x)
            mag_max[1] = max(mag_max[1], y)
            mag_max[2] = max(mag_max[2], z)
            current_duration = duration - (time.time() - start_time)
            feedback_msg.seconds = int(current_duration)
            goal_handle.publish_feedback(feedback_msg)
            self.rate.sleep()

        mag_offset = np.array(
            [(max_ + min_) / 2 for max_, min_ in zip(mag_max, mag_min)]
        )

        self.get_logger().info("Magnetometer calibration done.")
        self.get_logger().info(
            f"Offsets: X={mag_offset[0]:.2f}, Y={mag_offset[1]:.2f}, Z={mag_offset[2]:.2f}"
        )
        return mag_offset


    def actionServerCB_(self, goal_handle):
        self.get_logger().info("Executing goal.")
        
        feedback_msg = CalibrateImu.Feedback()
        # Accelerometer + Gyrometer calibration.
        if goal_handle.request.code == 0:
            self.accel_offset, self.gyro_offset = self.calibrate_accel_gyro()
        if goal_handle.request.code == 1:
            self.mag_offset = self.calibrate_magnetometer(goal_handle, feedback_msg)
        if goal_handle.request.code == 0 or goal_handle.request.code == 1:
            goal_handle.succeed()
            result = CalibrateImu.Result()
            result.result = 0
            return result

        goal_handle.fail()
        result = CalibrateImu.Result()
        result.result = 2 
        return result


    def sendDataCB_(self):
        mx, my, mz = self.imu.read_magnetometer_data()
        mag_temp = np.array([mx, my, mz])
        ax, ay, az, gx, gy, gz = self.imu.read_accelerometer_gyro_data()
        accel_temp = np.array([ax, ay, az])
        gyro_temp = np.array([gx, gy, gz])
        
        mag = mag_temp - self.mag_offset
        accel = accel_temp - self.accel_offset
        gyro = gyro_temp - self.gyro_offset
        
        gyro_rad = np.deg2rad(gyro)

        q = np.array([1.0, 0.0, 0.0, 0.0])  # Initial quaternion
        # Update EKF and get updated quaternion
        # w,x,y,z
        q = self.ekf.update(q, gyr=gyro_rad, acc=accel, mag=mag)

        # Create message
        quat = Quaternion()
        quat.x = q[1]
        quat.y = q[2]
        quat.z = q[3]
        quat.w = q[0]
        self.quaternion_pub_.publish(quat)

        # Convert quaternion to Euler angles (ZYX order = yaw, pitch, roll)
        # euler = R.from_quat([q[1], q[2], q[3], q[0]]).as_euler('zyx', degrees=True)
        # yaw, pitch, roll = euler

def main(args=None):
    rclpy.init(args=args)

    sleep_node = rclpy.create_node("control_sleep_node")  
    imu = Imu(sleep_node=sleep_node)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(imu)
    executor.add_node(sleep_node)

    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    rclpy.spin(imu)
    # Cleanup After Shutdown
    rclpy.shutdown() 
