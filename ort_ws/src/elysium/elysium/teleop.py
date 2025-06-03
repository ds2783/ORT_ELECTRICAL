import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import Joy

from elysium.config.mappings import AXES, BUTTONS

import time
from dataclasses import dataclass
from adafruit_servokit import ServoKit

CAMERA_SERVO_Z = 6
CAMERA_SERVO_X = 7

OFFSET = 10.


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

        # State -
        self.state = twist(0, 0)
        self.target = twist(0, 0)

        self.cam_angles = rotation2D(0, 0)

        self.last_connection_ = time.time_ns()
        self.connection_timer_ = self.create_timer(0.2, self.shutdownCB_)

        self.offset_ = OFFSET

        self.kit = ServoKit(channels=16)

    def confirmConnectionCB_(self, msg: Bool):
        self.last_connection_ = time.time_ns()

    def shutdownCB_(self):
        if time.time_ns() > self.last_connection_ + 5e8:
            self.target.linear = 0
            self.target.rotation = 0

            self.cam_angles.x_axis = 0

            self.drive()
            self.camera_rotate()

    def teleopCB_(self, msg: Joy):
        # DRIVE -----------------
        self.target.linear = msg.axes[AXES["TRIGGERRIGHT"]] + self.offset_
        self.target.linear -= msg.axes[AXES["TRIGGERLEFT"]] + self.offset_
        # goes from 1 to -1, therefore difference between the two
        # should be halved.
        self.target.linear /= 2
        self.target.rotation = msg.axes[AXES["LEFTX"]] + self.offset_

        self.drive()
        # ------------------------
        self.cam_angles.z_axis = 90 + msg.axes[AXES["RIGHTX"]] * 90 + self.offset_
        self.cam_angles.x_axis = 90 + msg.axes[AXES["LEFTX"]] * 90 + self.offset_

        self.camera_rotate()

    def bound_range(self, value):
        if value > 1:
            value = 1
        elif value < -1:
            value = -1
        return value

    def drive(self):
        left_side = self.bound_range(self.target.linear + 0.5 * self.target.rotation)
        right_side = self.bound_range(-self.target.linear + 0.5 * self.target.rotation)

        # TESTING TO MAKE WHEELS MORE SENSITIVE
        left_side = 90.0 + 60 * left_side
        right_side = 90.0 + 60 * right_side

        for i in range(0, 3):
            self.kit.servo[i].angle = right_side
        for i in range(3, 6):
            self.kit.servo[i].angle = left_side

        self.get_logger().info(
            "left_side: " + str(left_side) + " right_side: " + str(right_side)
        )

    def camera_rotate(self):
        # POSITIONAL
        self.kit.servo[CAMERA_SERVO_Z].angle = self.cam_angles.z_axis
        # CONTIOUS
        self.kit.servo[CAMERA_SERVO_X].angle = self.cam_angles.x_axis


def main(args=None):
    rclpy.init(args=args)
    node = TelepresenceOperations()
    rclpy.spin(node)
    rclpy.shutdown()
