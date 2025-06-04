#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


from mission_control.stream.stream_client import StreamClient
from mission_control.config.mappings import AXES, BUTTONS
from mission_control.config.network import COMM_PORT, PORT_MAIN_BASE, PI_IP

import glfw
from multiprocessing import Process
from qreader import QReader
from multiprocessing.connection import Client



class BaseNode(Node):
    def __init__(self, port):
        super().__init__("base")
        # CONSTANTS -- DO NOT REASSIGN
        # ---

        # STATE OBJECTS
        self.main_cam = StreamClient(
            "Stereo", "PI_IP", "udp", PORT_MAIN_BASE, 640, 480, stereo=False
        )
        self.qreader_ = QReader()

        address = ('localhost', port)
        self.comms_ = Client(address, authkey=b'123')
        # ---

        # TIMERS
        self.ping_timer_ = self.create_timer(0.2, self.pingCB_)
        # ----

        # Could possibly intercept the joy-commands and process what gets sent onto the Gorgon but leave for now
        self.get_logger().info("The Base Station has been initialised.")
        # ----------------------------

        # Topics ---------------------
        self.connection_pub_ = self.create_publisher(Bool, "ping", 10)
        self.controller_sub_ = self.create_subscription(Joy, "joy", self.controlCB_, 10)
        # ----------------------------

        # Variables ------------------
        self.qr_button_ = False
        self.last_qr = "None"
        # ----------------------------

    def controlCB_(self, msg: Joy):
        trigger_pressed = msg.buttons[BUTTONS["CIRCLE"]]
        if trigger_pressed and not self.qr_button_:
            # CAPTURE QR-CODE
            self.qr_button_ = True
            image = self.main_cam.fetch_frame()
            if image is not None:
                qreader_out = self.qreader_.detect_and_decode(image=image)
                # record position +
                self.last_qr = str(qreader_out)
                self.sendComms(self.last_qr)

        elif not trigger_pressed and self.qr_button_:
            # Ensure only one capture even per press
            self.qr_button_ = False

    def pingCB_(self):
        msg = Bool()
        msg.data = True
        self.connection_pub_.publish(msg)

    def handler(self, signal_received, frame):
        self.main_cam.stop()
        self.get_logger().warn(
            "Stream has stoped due to error: "
            + str(signal_received)
            + " on frame: "
            + str(frame)
        )

    def sendComms(self, msg):
        self.comms_.send(msg)

def main(args=None):
    rclpy.init(args=args)

    # has to be initialised this way round!
    base = BaseNode(COMM_PORT)

    rclpy.spin(base)
    # Cleanup After Shutdown
    rclpy.shutdown()
