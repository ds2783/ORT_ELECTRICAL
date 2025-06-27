import rclpy
from rclpy.node import Node
from rclpy.action.server import ActionServer
from rclpy.qos import qos_profile_sensor_data
import rclpy.utilities

from geometry_msgs.msg import Quaternion
from ort_interfaces.action import Calibrate

import numpy as np
from pyrr import quaternion

from elysium.config.sensors import IMU_SENSOR_PERIOD

import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR


class Imu(Node):
    def __init__(self, i2c_addr=0x4B):
        super().__init__("imu_sensor")
        i2c = busio.I2C(board.SCL, board.SDA)
        self.bno = BNO08X_I2C(i2c, address=i2c_addr)
        self.bno.initialize()
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

        # Topics -------------
        self.quaternion_pub_ = self.create_publisher(
            Quaternion, "/imu/quat", qos_profile=qos_profile_sensor_data
        )
        # -------------------

        # Action Server ------
        self.calibrate_action_server_ = ActionServer(
            self, CalibrateImu, "/imu/calibrate", self.actionServerCB_
        )
        # --------------------

        # Timer -----------
        self.imu_data_timer_ = self.create_timer(IMU_SENSOR_PERIOD, self.sendDataCB_)
        # -----------------

        # Variables ---------

        self.q = np.array([0.0, 0.0, 0.0, 1.0])  # Initial quaternion
        self.inverse = np.array([0.0, 0.0, 0.0, 1.0])

    def zero_axis(self):
        self.inverse = quaternion.inverse(self.q)

    def calibrate_imu(self):
        self.bno.begin_calibration()
        self.bno.save_calibration_data()

    def sendDataCB_(self):
        self.q = np.array(self.bno.quaternion)

        corrected_q = quaternion.cross(self.q, self.inverse)
        # Create message
        quat = Quaternion()
        quat.x = corrected_q[1]
        quat.y = corrected_q[2]
        quat.z = corrected_q[3]
        quat.w = corrected_q[0]
        self.quaternion_pub_.publish(quat)

    def actionServerCB_(self, goal_handle):
        self.get_logger().info("Executing goal.")

        # Accelerometer + Gyrometer calibration.
        if goal_handle.request.code == 0:
            self.calibrate_imu()
        if goal_handle.request.code == 1:
            self.zero_axis()
        if goal_handle.request.code in [0, 1]:
            goal_handle.succeed()
            result = Calibrate.Result()
            result.result = 0
            return result

        goal_handle.fail()
        result.result = 2
        return result


def main(args=None):
    rclpy.init(args=args)
    imu = Imu()

    # Cleanup After Shutdown
    try:
        while rclpy.utilities.ok():
            rclpy.spin(imu)
    except KeyboardInterrupt:
        imu.get_logger().warn(f"KeyboardInterrupt triggered.")
    finally:
        imu.destroy_node()
        rclpy.utilities.try_shutdown()  

