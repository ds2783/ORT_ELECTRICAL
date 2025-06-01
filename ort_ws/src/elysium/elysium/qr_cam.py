import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import String

from elysium.streaming.StreamClient import StreamClient
from elysium.Comms.Output import Output
from elysium.config.mappings import AXES, BUTTONS

import cv2
from qreader import QReader
from signal import signal, SIGINT
import sys


class QRCam(Node):
    def __init__(self):
        super().__init__("qr_cam")
        # STREAM ---------------------
        # Initialise stream
        self.client = StreamClient("IRCam", "192.168.0.101", 'udp', 5008, 1920, 1080, stereo=True) 

        # ----------------------------

        # OBJECTS --------------------
        self.qreader_ = QReader()
        # ----------------------------

        # Topics ---------------------
        self.qr_pub_ = self.create_publisher(String, "qr", 10)
        self.controller_sub_ = self.create_subscription(Joy, "joy", self.controlCB, 10)
        # ----------------------------

        # Variables ------------------
        self.qr_button_ = False
        # ----------------------------

    def controlCB(self, msg: Joy):
        trigger_pressed = msg.buttons[BUTTONS["CIRCLE"]]
        if trigger_pressed and not self.qr_button_:
            # CAPTURE QR-CODE
            self.qr_button_ = True
            image = self.client.fetch_img()
            if image is not None:
                qreader_out = self.qreader_.detect_and_decode(image=image)
                # create Ros2 message 
                message = String()
                message.data = qreader_out
                self.qr_pub_.publish(message)
        elif not trigger_pressed and self.qr_button_:
            # Ensure only one capture even per press
            self.qr_button_ = False

    def handler(self, signal_received, frame):
        self.client.stop()
        self.get_logger().warn(
            "Stream has stoped due to error: "
            + str(signal_received)
            + " on frame: "
            + str(frame)
        )


def main(args=None):
    rclpy.init(args=args)
    node = QRCam()
    rclpy.spin(node)
    rclpy.shutdown()
