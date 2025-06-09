import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.action.server import ActionServer

from geometry_msgs.msg import Quaternion
from ort_interfaces.action import CalibrateImu

import time
import numpy as np
from pyrr import quaternion
from threading import Thread
from ahrs.filters import EKF

from elysium.hardware.icm20948 import ICM20948

from elysium.config.sensors import IMU_SENSOR_PERIOD


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
        self.imu_data_timer_ = self.create_timer(IMU_SENSOR_PERIOD, self.sendDataCB_)
        # -----------------

        # Variables ---------
        self.mag_offset = self.gyro_offset = self.accel_offset = 0
        self.sleep_node = sleep_node
        self.rate = self.sleep_node.create_rate(1)

        self.q = np.array([0.0, 0.0, 0.0, 1.0])  # Initial quaternion
        self.inverse = np.array([0.0, 0.0, 0.0, 1.0])

        self.imu_process = True
        
        self.imu_thread = Thread(target=self.update_ekf)
        self.imu_thread.start()


    def calibrate_accel_gyro(self, samples=100, delay=0.01):
        self.imu_process = False
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
        self.imu_process = True
        self.imu_thread = Thread(target=self.update_ekf)
        self.imu_thread.start()
        return accel_offset, gyro_offset

    def calibrate_magnetometer(self, goal_handle, feedback_msg, duration=20, delay=0.05):
        self.imu_process = False
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
        self.imu_process = True
        self.imu_thread = Thread(target=self.update_ekf)
        self.imu_thread.start()
        return mag_offset

    def zero_axis(self):
        self.inverse = quaternion.inverse(self.q)
        

    def actionServerCB_(self, goal_handle):
        self.get_logger().info("Executing goal.")
        
        feedback_msg = CalibrateImu.Feedback()
        # Accelerometer + Gyrometer calibration.
        if goal_handle.request.code == 0:
            self.accel_offset, self.gyro_offset = self.calibrate_accel_gyro()
        if goal_handle.request.code == 1:
            self.mag_offset = self.calibrate_magnetometer(goal_handle, feedback_msg)
        if goal_handle.request.code == 2:
            self.zero_axis()
        if goal_handle.request.code in [0, 1, 2]:
            goal_handle.succeed()
            result = CalibrateImu.Result()
            result.result = 0
            return result

        goal_handle.fail()
        result = CalibrateImu.Result()
        result.result = 2 
        return result


    def sendDataCB_(self):
        corrected_q = quaternion.cross(self.q , self.inverse)
        # Create message
        quat = Quaternion()
        quat.x = corrected_q[1]
        quat.y = corrected_q[2]
        quat.z = corrected_q[3]
        quat.w = corrected_q[0]
        self.quaternion_pub_.publish(quat)

        # Convert quaternion to Euler angles (ZYX order = yaw, pitch, roll)
        # euler = R.from_quat([q[1], q[2], q[3], q[0]]).as_euler('zyx', degrees=True)
        # yaw, pitch, roll = euler
    
    def update_ekf(self):
        while self.imu_process:
            try:
                mx, my, mz = self.imu.read_magnetometer_data()
                mag_temp = np.array([mx, my, mz])
                ax, ay, az, gx, gy, gz = self.imu.read_accelerometer_gyro_data()
                accel_temp = np.array([ax, ay, az])
                gyro_temp = np.array([gx, gy, gz])
        
                mag = mag_temp - self.mag_offset
                accel = accel_temp - self.accel_offset
                gyro = gyro_temp - self.gyro_offset
        
                gyro_rad = np.deg2rad(gyro)

                # magnetometer data comes in micro Tesla ->
                # EKF take nano Tesla
                mag_nano = mag * 1e3

                # Update EKF and get updated quaternion
                # w,x,y,z
                self.q = self.ekf.update(self.q, gyr=gyro_rad, acc=accel, mag=mag_nano)
            except:
                self.get_logger().warn("Timout waiting for IMU")
        

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
