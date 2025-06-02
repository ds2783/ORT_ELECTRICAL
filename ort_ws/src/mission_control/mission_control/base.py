#!/usr/bin/env python3
import rclpy
import rclpy.executors
from rclpy.node import Node

from sensor_msgs.msg import Joy
# from std_msgs.msg import String


from mission_control.gui import control_gui
from mission_control.streaming.stream_client import StreamClient 
from mission_control.config.mappings import AXES, BUTTONS

import glfw
from threading import Thread
from functools import partial
from qreader import QReader


class BaseNode(Node):
    def __init__(self):
        super().__init__("base")
        # CONSTANTS -- DO NOT REASSIGN
        # --- 

        # STATE VARIABLES
        # ---
        
        # STATE OBJECTS
        self.main_cam = StreamClient("Stereo","192.168.0.101","udp",5008, 640, 480, stereo=False)
        # ---
            
        # TESTS
        # ---

        # Could possibly intercept the joy-commands and process what gets sent onto the Gorgon but leave for now
        self.get_logger().info("The Base Station has been initialised.")

        # OBJECTS --------------------
        self.qreader_ = QReader()
        # ----------------------------

        # Topics ---------------------
        # self.qr_pub_ = self.create_publisher(String, "qr", 10)
        self.controller_sub_ = self.create_subscription(Joy, "joy", self.controlCB, 10)
        # ----------------------------

        # Variables ------------------
        self.qr_button_ = False
        self.last_qr = "None"
        # ----------------------------

    def controlCB(self, msg: Joy):
        trigger_pressed = msg.buttons[BUTTONS["CIRCLE"]]
        if trigger_pressed and not self.qr_button_:
            # CAPTURE QR-CODE
            self.qr_button_ = True

            image = self.main_cam.fetch_frame()
            if image is not None:
                qreader_out = self.qreader_.detect_and_decode(image=image)
                # create Ros2 message 
                # message = String()
                # message.data = str(qreader_out)
                # self.qr_pub_.publish(message)
                self.last_qr = str(qreader_out)
        elif not trigger_pressed and self.qr_button_:
            # Ensure only one capture even per press
            self.qr_button_ = False

    def handler(self, signal_received, frame):
        self.main_cam.stop()
        self.get_logger().warn(
            "Stream has stoped due to error: "
            + str(signal_received)
            + " on frame: "
            + str(frame)
        )

def main(args=None):
    rclpy.init(args=args)
    base = BaseNode()
    cams = [StreamClient("Stereo","192.168.0.101","udp",5009,640,480,stereo=False),
            StreamClient("Stereo","192.168.0.101","udp",5010,640,480,stereo=False)]
    gui = control_gui.GUI(cams)

    # Run GUI
    while not glfw.window_should_close(gui.window):
        gui.run(base.last_qr)
        rclpy.spin_once(base)

    # Cleanup After Shutdown
    gui.impl.shutdown()
    glfw.terminate()
    rclpy.shutdown()

