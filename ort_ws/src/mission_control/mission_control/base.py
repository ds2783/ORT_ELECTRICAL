#!/usr/bin/env python3
import rclpy
import rclpy.executors
from rclpy.node import Node

from mission_control.gui import control_gui
from mission_control.streaming.stream_client import StreamClient 

import glfw
from threading import Thread
from functools import partial


class BaseNode(Node):
    def __init__(self):
        super().__init__("base")
        # CONSTANTS -- DO NOT REASSIGN
        # --- 

        # STATE VARIABLES
        # ---
        
        # STATE OBJECTS
        self.main_cam = StreamClient("Stereo","192.168.0.101","udp",5008, 640, 480, stereo=False)
        self.client_streams=[
        # StreamClient("Stereo","stereocam","tcp",8081,720,640,stereo=True),
        # StreamClient("Stereo","192.168.0.101","udp",5008, 640, 480, stereo=False),
        # StreamClient("USB","stereocam","tcp",8082,640,480),
        # StreamClient("USB","127.0.0.1","udp",8082,640,480),
        # StreamClient("Stereo","stereocam","rtsp","8554/stream1",640,480,stereo=True),
        ]
        # ---
            
        # TESTS
        # ---

        # Could possibly intercept the joy-commands and process what gets sent onto the Gorgon but leave for now
        self.get_logger().info("The Base Station has been initialised.")



def main(args=None):
    rclpy.init(args=args)
    base = BaseNode()
    gui = control_gui.GUI(base)

    # Multithreading so that Base node can be run concurrently with the GUI
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

