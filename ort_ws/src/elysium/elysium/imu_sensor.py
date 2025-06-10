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

from elysium.hardware.icm20948 import ICM20948

from elysium.config.sensors import IMU_SENSOR_PERIOD

import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ACCELEROMETER


class Imu(Node):
    def __init__(self, sleep_node, i2c_addr=0x4b):
        super().__init__("imu_sensor")
        i2c = busio.I2C(board.SCL, board.SDA)
        self.bno = BNO08X_I2C(i2c, address=i2c_addr)
        self.bno.initialize()
        
        # self.ekf = EKF()

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
        self.sleep_node = sleep_node
        self.rate = self.sleep_node.create_rate(1)

        self.q = np.array([0.0, 0.0, 0.0, 1.0])  # Initial quaternion
        self.inverse = np.array([0.0, 0.0, 0.0, 1.0])


    def calibrate_imu(self):
        self.get_logger().info(str(self.bno.calibration_status))
        self.bno.begin_calibration()
        self.get_logger().info(str(self.bno.calibration_status))


    def sendDataCB_(self):
        self.q = np.array(self.bno.quaternion)

        corrected_q = quaternion.cross(self.q , self.inverse)
        # Create message
        quat = Quaternion()
        quat.x = corrected_q[1]
        quat.y = corrected_q[2]
        quat.z = corrected_q[3]
        quat.w = corrected_q[0]
        self.quaternion_pub_.publish(quat)


    def actionServerCB_(self, goal_handle):
        self.get_logger().info("Executing goal.")
        
        feedback_msg = CalibrateImu.Feedback()
        # Accelerometer + Gyrometer calibration.
        if goal_handle.request.code == 0:
            self.calibrate_imu()
        if goal_handle.request.code == 1:
            self.zero_axis()
        if goal_handle.request.code in [0, 1]:
            goal_handle.succeed()
            result = CalibrateImu.Result()
            result.result = 0
            return result

        goal_handle.fail()
        result = CalibrateImu.Result()
        result.result = 2 
        return result


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
