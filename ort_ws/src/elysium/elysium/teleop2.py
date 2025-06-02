import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from elysium.config.mappings import AXES, BUTTONS

from dataclasses import dataclass

from adafruit_servokit import ServoKit

CAMERA_SERVO_Z = 6
CAMERA_SERVO_X = 7

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
            Joy, "joy", self.teleopCB, 10
        )

        # State -
        self.state = twist(0, 0)
        self.target = twist(0, 0)

        self.cam_angles = rotation2D(0, 0)

        self.kit = ServoKit(channels=16)

    def teleopCB(self, msg: Joy):
        # DRIVE -----------------
        self.target.linear = msg.axes[AXES["TRIGGERRIGHT"]]
        self.target.linear -= msg.axes[AXES["TRIGGERLEFT"]]
        # goes from 1 to -1, therefore difference between the two
        # should be halved.
        self.target.linear /= 2
        self.target.rotation = msg.axes[AXES["LEFTX"]]

        self.drive()
        # ------------------------
        self.cam_angles.z_axis = msg.axes[AXES["RIGHTX"]] * 90
        self.cam_angles.x_axis = msg.axes[AXES["LEFTX"]] * 90

        self.camera_rotate()

    def drive(self):
        left_side = self.target.linear + 0.5 * self.target.rotation
        if left_side > 1:
            left_side = 1
        if left_side < -1:
            left_side = -1

        # UGLY MUST FIX
        right_side = -self.target.linear + 0.5 * self.target.rotation
        if right_side > 1:
            right_side = 1
        if right_side < -1:
            right_side = -1

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
        self.kit.servo[CAMERA_SERVO_Z].angle = self.cam_angles.z_axis
        self.kit.servo[CAMERA_SERVO_X].angle = self.cam_angles.x_axis


def main(args=None):
    rclpy.init(args=args)
    node = TelepresenceOperations()
    rclpy.spin(node)
    rclpy.shutdown()
