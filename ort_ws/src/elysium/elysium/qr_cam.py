import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import String

from elysium.streaming.StreamServer import StreamServer
from elysium.Comms.Output import Output
from elysium.config.mappings import AXES, BUTTONS

import cv2
from qreader import QReader
from picamera2 import Picamera2
from libcamera import controls
from signal import signal, SIGINT
import sys

# config
MULTICAST = False
IP = "192.168.0.103"  # UDP multicast IP
PORT = 5008
MODEL = "imx708_noir"  # camera model name (find using libcamera-vid --list-cameras)
WIDTH = 1296
HEIGHT = 972
NAME = "QR-Cam"  # stream name for display in console

out = Output("None")  # console output, with optional TCP forwarding

# Initialise stream
stream = StreamServer(
    out, MODEL, NAME
)  # system finds the camera based upon the model number (assumes no duplicates)
stream.configure(WIDTH, HEIGHT)
stream.start(IP, PORT, MULTICAST)  # using a multicast address 224.1.1.1:5008
stream.set_bitrate(5000000)

# stream.set_controls({"AfMode": controls.AfModeEnum.Continuous}) # set a libcamera control


class QRCam(Node):
    def __init__(self):
        super().__init__("qr_cam")
        # STREAM ---------------------
        out = Output("None")  # console output, with optional TCP forwarding
        # Initialise stream
        self.stream = StreamServer(
            out, MODEL, NAME
        )  # system finds the camera based upon the model number (assumes no duplicates)
        self.stream.configure(WIDTH, HEIGHT)
        self.stream.start(IP, PORT, MULTICAST)  # using a multicast address 224.1.1.1:5008
        self.stream.set_bitrate(5000000)
        signal(SIGINT, self.handler)
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
            image = self.stream.capture_img()
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
        stream.stop()
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
