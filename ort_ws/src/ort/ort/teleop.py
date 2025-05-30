import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Joy
from dataclasses import dataclass

from adafruit_servokit import ServoKit

AXES = {
    "LEFTX": 0,
    "LEFTY": 1,
    "RIGHTX": 2,
    "RIGHTY": 3,
    "TRIGGERLEFT": 4,
    "TRIGGERRIGHT": 5,
}

BUTTONS = {
    "CROSS": 0,
    "CIRCLE": 1,
    "SQUARE": 2,
    "TRIANGLE": 3,
    "SELECT": 4,
    "MIDDLE": 5,
    "START": 6,
    "LEFTSTICK": 7,
    "RIGHTSTICK": 8,
    "LEFTSHOULDER": 9,
    "RIGHTSHOULDER": 10,
    "DPAD_UP": 11,
    "DPAD_DOWN": 12,
    "DPAD_LEFT": 13,
    "DPAD_RIGHT": 14,
    "MISC1": 15,
    "TOUCHPAD": 20,
}


class TelepresenceOperations(Node):
    def __init__(self):
        super().__init__("teleop")
        self.controller_commands_sub_ = self.create_subscription(
            Joy, "joy", self.teleopCB, 10
        )

        self.kit = ServoKit(channels=16)

    def teleopCB(self, msg: Joy):
        forward = msg.buttons[BUTTONS["DPAD_UP"]]
        backward = msg.buttons[BUTTONS["DPAD_DOWN"]]
        turn_right = msg.buttons[BUTTONS["DPAD_RIGHT"]]
        turn_left = msg.buttons[BUTTONS["DPAD_LEFT"]]

        self.runMotors(forward, backward, turn_right, turn_left)

    def runMotors(self, forward, backward, turn_right, turn_left):
        if (forward + backward + turn_right + turn_left) == 1:
            if forward == True:
                right_side = 1
                left_side = -1
            elif backward == True:
                right_side = -1
                left_side = 1
            elif turn_right == True:
                right_side = 1
                left_side = 1
            else:
                right_side = -1
                left_side = -1

            for i in range(0, 3):
                self.kit.continuous_servo[i].throttle = right_side
            for i in range(3, 6):
                self.kit.continuous_servo[i].throttle = left_side

        else:
            for i in range(0, 6):
                self.kit.continuous_servo[i].throttle = 0


def main(args=None):
    rclpy.init(args=args)
    node = TelepresenceOperations()
    rclpy.spin(node)
    rclpy.shutdown()
