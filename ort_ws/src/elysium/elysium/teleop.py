from os import wait
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from ort_interfaces.msg import CameraRotation

from elysium.config.mappings import AXES
from elysium.config.controls import (
    CAMERA_SENSITIVITY,
    CAMERA_SERVO_X,
    CAMERA_SERVO_Z,
    OFFSET,
)

import time
from dataclasses import dataclass
from adafruit_servokit import ServoKit


@dataclass
class twist:
    linear: float
    rotation: float


@dataclass
class rotation2D:
    z_axis: float
    x_axis: float


class TelepresenceOperations(Node):
    def __init__(self):
        super().__init__("teleop")
        self.controller_commands_sub_ = self.create_subscription(
            Joy, "joy", self.teleopCB_, 10
        )
        self.base_poing_sub_ = self.create_subscription(
            Bool, "ping", self.confirmConnectionCB_, 10
        )
        self.cam_angles__pub_ = self.create_publisher(
            CameraRotation, "/elysium/cam_angles", 10
        )

        # State -
        self.state = twist(0, 0)
        self.target = twist(0, 0)

        self.z_increment = 0
        self.x_increment = 0
        
        # Setting Zeroed rotation for camera servos
        self.cam_angles_ = rotation2D(90.0, 90.0)

        # Conncection timer
        self.last_connection_ = time.time_ns()
        self.connection_timer_ = self.create_timer(0.2, self.shutdownCB_)
        self.driver_timer_ = self.create_timer(0.01, self.driveCB_)

        # Servo Offset control
        self.offset_ = OFFSET

        self.kit_ = ServoKit(channels=16)

    def confirmConnectionCB_(self, msg: Bool):
        self.last_connection_ = time.time_ns()

    def shutdownCB_(self):
        if time.time_ns() > self.last_connection_ + 5e8:
            self.target.linear = 0
            self.target.rotation = 0

            self.cam_angles_.x_axis = 90
            self.cam_angles_.z_axis = 90

            self.drive()
            self.camera_rotate()

    def teleopCB_(self, msg: Joy):
        # DRIVE -----------------
        self.target.linear = msg.axes[AXES["TRIGGERRIGHT"]]
        self.target.linear -= msg.axes[AXES["TRIGGERLEFT"]]
        # goes from 1 to -1, therefore difference between the two
        # should be halved.
        self.target.linear /= 2
        self.target.rotation = msg.axes[AXES["LEFTX"]]
        # ------------------------

        self.z_increment = msg.axes[AXES["RIGHTX"]] * CAMERA_SENSITIVITY
        self.x_increment = msg.axes[AXES["RIGHTY"]] * CAMERA_SENSITIVITY

        # publish camera rotation, note 90degrees servo rotation -> 0degrees around the axis
        camera_rotation_msg = CameraRotation(
            z_axis=float(self.cam_angles_.z_axis - 90), x_axis=float(self.cam_angles_.x_axis - 90)
        )
        self.cam_angles__pub_.publish(camera_rotation_msg)

    def driveCB_(self):
        self.drive()
        self.camera_rotate()

    def bound_range(self, value):
        if value > 1:
            value = 1
        elif value < -1:
            value = -1
        return value

    def bound_180(self, value):
        if value > 180:
            value = 180
        elif value < 0:
            value = 0
        return value

    def drive(self):
        left_side = self.bound_range(self.target.linear + 0.5 * self.target.rotation)
        right_side = self.bound_range(-self.target.linear + 0.5 * self.target.rotation)

        # TESTING TO MAKE WHEELS MORE SENSITIVE
        left_side = self.bound_180(90.0 + 60 * left_side + self.offset_)
        right_side = self.bound_180(90.0 + 60 * right_side + self.offset_)

        for i in range(0, 3):
            self.kit_.servo[i].angle = right_side
        for i in range(3, 6):
            self.kit_.servo[i].angle = left_side

        # self.get_logger().info(
        #     "left_side: " + str(left_side) + " right_side: " + str(right_side)
        # )

    def camera_rotate(self):
        self.cam_angles_.z_axis = self.bound_180(float(self.z_increment + self.cam_angles_.z_axis))
        self.cam_angles_.x_axis = self.bound_180(float(self.x_increment + self.cam_angles_.x_axis))
        # POSITIONAL
        self.kit_.servo[CAMERA_SERVO_Z].angle = self.cam_angles_.z_axis
        # CONTIOUS
        self.kit_.servo[CAMERA_SERVO_X].angle = self.cam_angles_.x_axis

        # self.get_logger().info(
        #     "z_axis: " + str(self.cam_angles_.z_axis) + " x_axis: " + str(self.cam_angles_.x_axis)
        # )


def main(args=None):
    rclpy.init(args=args)
    node = TelepresenceOperations()
    rclpy.spin(node)
    rclpy.shutdown()
