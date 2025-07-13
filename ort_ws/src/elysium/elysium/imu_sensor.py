import rclpy
from rclpy.node import Node
from rclpy.action.server import ActionServer
from rclpy.qos import qos_profile_sensor_data
import rclpy.utilities
import rclpy.executors

from geometry_msgs.msg import Quaternion, Vector3
from ort_interfaces.action import Calibrate

import numpy as np
import time

from elysium.config.sensors import IMU_SENSOR_PERIOD, IMU_UPDATE_FREQUENCY, IMU_ACCELEROMETER_CALIBRATION_PERIOD
from elysium.config.services import CALIBRATE_IMU, ZERO_AXIS, SUCCESS, FAIL_UNRECOGNISED_OP_CODE
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
        
        self.linear_offset_x = 0 
        self.linear_offset_y = 0
        self.linear_offset_z = 0

        self.q = np.array([0.0, 0.0, 0.0, 1.0])  # Initial quaternion

        self.calibration_rate = self.create_rate(IMU_UPDATE_FREQUENCY)

    def calibrate_imu(self, goal_handle):
        feedback = Calibrate.Feedback()
        feedback_time = int(IMU_ACCELEROMETER_CALIBRATION_PERIOD)
        feedback.msg.seconds = feedback_time
        goal_handle.publish_feedback(feedback)

        start = time.monotonic()
        now = time.monotonic()
        
        accel_x = []
        accel_y = []
        accel_z = []
        while (now - start) < IMU_ACCELEROMETER_CALIBRATION_PERIOD:
            now = time.monotonic()
            accel_x.append(self.bno.linear_acceleration[0]) 
            accel_y.append(self.bno.linear_acceleration[1]) 
            accel_z.append(self.bno.linear_acceleration[2])
            
            prev_feedback_time = feedback_time
            # int() rounds up when higher than 0.5 and rounds when when lower than 0.5.
            # This means it should always be publishing seconds remaining.
            feedback_time = int(now - start + 0.5)
            if prev_feedback_time != feedback_time:
                feedback.msg.seconds = feedback_time
                goal_handle.publish_feedback(feedback)

            self.calibration_rate.sleep()

        self.linear_offset_x = sum(accel_x)/len(accel_x)
        self.linear_offset_y = sum(accel_y)/len(accel_y)
        self.linear_offset_z = sum(accel_z)/len(accel_z)

    def sendDataCB_(self):
        self.q = np.array(self.bno.quaternion)
        # Create message
        quat = Quaternion()
        quat.x = self.q[1]
        quat.y = self.q[2]
        quat.z = self.q[3]
        quat.w = self.q[0]
        self.quaternion_pub_.publish(quat)

        if (
            self.acceleration_x.average is not None
            and self.acceleration_y.average is not None
            and self.acceleration_z.average is not None
        ):
            accel = Vector3(
                x=float(self.acceleration_x.average - self.linear_offset_x),
                y=float(self.acceleration_y.average - self.linear_offset_y),
                z=float(self.acceleration_z.average - self.linear_offset_z),
            )
            self.accelerometer_pub_.publish(accel)

    def updateAccelCB_(self):
        self.take_accel_reading()

    def take_accel_reading(self):
        self.acceleration_x.add(self.bno.linear_acceleration[0])
        self.acceleration_y.add(self.bno.linear_acceleration[1])
        self.acceleration_z.add(self.bno.linear_acceleration[2])

    def actionServerCB_(self, goal_handle):
        self.get_logger().info("Executing goal.")

        # Accelerometer + Gyrometer calibration.
        if goal_handle.request.code == CALIBRATE_IMU:
            self.calibrate_imu(goal_handle)
        if goal_handle.request.code in [CALIBRATE_IMU]:
            goal_handle.succeed()
            result = Calibrate.Result()
            result.result = SUCCESS
            return result

        goal_handle.succeed()
        result = Calibrate.Result()
        result.result = FAIL_UNRECOGNISED_OP_CODE
        return result


def main(args=None):
    rclpy.init(args=args)
    imu = Imu()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(imu)

    # Cleanup After Shutdown
    try:
        executor.spin()
    except KeyboardInterrupt:
        imu.get_logger().warn(f"KeyboardInterrupt triggered.")
    finally:
        imu.destroy_node()
        rclpy.utilities.try_shutdown()
