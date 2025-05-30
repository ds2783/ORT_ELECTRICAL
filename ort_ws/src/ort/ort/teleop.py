import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Joy
from dataclasses import dataclass

from adafruit_servokit import ServoKit

@dataclass
class twist:
    linear : float
    rotation : float

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
        self.controller_commands_sub_ = self.create_subscription(Joy, "joy", self.teleopCB, 10)
        
        # State - 
        self.state = twist(0, 0)
        self.target = twist(0, 0)

        self.kit = ServoKit(channels=16)



    def teleopCB(self, msg: Joy):
        self.target.linear = msg.axes[AXES["TRIGGERRIGHT"]]
        self.target.linear -= msg.axes[AXES["TRIGGERLEFT"]]
        # goes from 1 to -1, therefore difference between the two
        # should be halved.
        self.target.linear /= 2
        self.target.rotation = msg.axes[AXES["LEFTX"]]

        self.runMotors()

        self.get_logger().info(str(self.target.linear) + " " + str(self.target.rotation))

    def runMotors(self):
        left_side = self.target.linear + 0.5 * self.target.rotation
        if left_side > 1:
            left_side = 1
        if left_side < -1:
            left_side = -1
        
        # UGLY MUST FIX
        right_side = - self.target.linear + 0.5 * self.target.rotation
        if right_side > 1:
            right_side = 1
        if right_side < -1:
            right_side = -1
 
        for i in range(0, 3):
            self.kit.continuous_servo[i].throttle = left_side
        for i in range(3, 6):
            self.kit.continuous_servo[i].throttle = right_side



def main(args=None):

    rclpy.init(args=args)
    node = TelepresenceOperations()
    rclpy.spin(node)
    rclpy.shutdown()
