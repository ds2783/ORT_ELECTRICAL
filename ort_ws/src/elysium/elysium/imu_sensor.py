from numpy.lib import average
import rclpy
from rclpy.node import Node
from rclpy.action.server import ActionServer
from rclpy.qos import qos_profile_sensor_data
import rclpy.utilities

from geometry_msgs.msg import Quaternion, Vector3
from ort_interfaces.action import Calibrate

import numpy as np
from pyrr import quaternion

from elysium.config.sensors import IMU_SENSOR_PERIOD, IMU_UPDATE_FREQUENCY
from elysium.utils import RollingAverage

import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR, BNO_REPORT_LINEAR_ACCELERATION


class Imu(Node):
    def __init__(self, i2c_addr=0x4B):
        super().__init__("imu_sensor")
        i2c = busio.I2C(board.SCL, board.SDA)
        self.bno = BNO08X_I2C(i2c, address=i2c_addr)
        self.bno.initialize()
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        self.bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)

        # Topics -------------
        self.quaternion_pub_ = self.create_publisher(
            Quaternion, "/imu/quat", qos_profile=qos_profile_sensor_data
        )
        self.accelerometer_pub_ = self.create_publisher(
            Vector3, "/imu/linear_accel", qos_profile=qos_profile_sensor_data
        )
        # -------------------

        # Action Server ------
        self.calibrate_action_server_ = ActionServer(
            self, Calibrate, "/imu/calibrate", self.actionServerCB_
        )
        # --------------------

        # Timer -----------
        self.imu_data_timer_ = self.create_timer(IMU_SENSOR_PERIOD, self.sendDataCB_)
        self.imu_update_timer_ = self.create_timer(
            1 / IMU_UPDATE_FREQUENCY, self.updateAccelCB_
        )
        # -----------------

        # Variables ---------
        self.acceleration_x = RollingAverage(50)
        self.acceleration_y = RollingAverage(50)
        self.acceleration_z = RollingAverage(50)

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

        accel = Vector3(
            x=self.acceleration_x.average,
            y=self.acceleration_y.average,
            z=self.acceleration_z.average,
        )
        self.accelerometer_pub_.publish(accel)

    def updateAccelCB_(self):
        self.acceleration_x.add(self.bno.linear_acceleration[0])
        self.acceleration_y.add(self.bno.linear_acceleration[1])
        self.acceleration_z.add(self.bno.linear_acceleration[2])

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
        result = Calibrate.Result()
        result.result = 2
        return result


def main(args=None):
    rclpy.init(args=args)
    imu = Imu()

    # Cleanup After Shutdown
    try:
        rclpy.spin(imu)
    except KeyboardInterrupt:
        imu.get_logger().warn(f"KeyboardInterrupt triggered.")
    finally:
        imu.destroy_node()
        rclpy.utilities.try_shutdown()
