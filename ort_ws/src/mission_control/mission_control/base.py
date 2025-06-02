#!/usr/bin/env python3
import rclpy
import rclpy.executors
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
# from std_msgs.msg import String


from mission_control.gui import control_gui
from mission_control.streaming.stream_client import StreamClient 
from mission_control.config.mappings import AXES, BUTTONS

import glfw
from threading import Thread
from functools import partial
from qreader import QReader
import socket


class BaseNode(Node):
    def __init__(self, comms):
        super().__init__("base")
        # CONSTANTS -- DO NOT REASSIGN
        # --- 

        # STATE OBJECTS
        self.main_cam = StreamClient("Stereo","192.168.0.101","udp",5008, 640, 480, stereo=False)
        self.qreader_ = QReader()
        self.comms_ = comms
        # ---
            
        # TIMERS
        self.ping_timer_ = self.create_timer(0.2, self.pingCB_)
        # ----

        # Could possibly intercept the joy-commands and process what gets sent onto the Gorgon but leave for now
        self.get_logger().info("The Base Station has been initialised.")
        # ----------------------------

        # Topics ---------------------
        self.connection_pub_ = self.create_publisher(Bool, "ping", 10)
        # self.qr_pub_ = self.create_publisher(String, "qr", 10)
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
                # create Ros2 message 
                # message = String()
                # message.data = str(qreader_out)
                # self.qr_pub_.publish(message)
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
        self.comms_.send(msg.encode())

def main(args=None):
    rclpy.init(args=args)
    comms = socket.socketpair(socket.AF_UNIX)
    base = BaseNode(comms[0])

    cams = [StreamClient("Stereo","192.168.0.101","udp",5009,640,480,stereo=False),
            StreamClient("Stereo","192.168.0.101","udp",5010,640,480,stereo=False)]
    gui = control_gui.GUI(cams, comms[1])

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(base)

    node_thread = Thread(target=executor.spin)
    node_thread.start()

    # Run GUI
    while not glfw.window_should_close(gui.window):
        gui.run()

    # Cleanup After Shutdown
    gui.impl.shutdown()
    glfw.terminate()
    rclpy.shutdown()

