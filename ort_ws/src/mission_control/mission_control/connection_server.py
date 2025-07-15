#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import rclpy.utilities

from std_msgs.msg import Bool

from mission_control.config.network import baseQoS

class ConnectionServer(Node):
    def __init__(self):
        super().__init__("connection_service")
        # TIMERS
        self.ping_timer_ = self.create_timer(
            0.2, self.pingCB_
        )
        # Connection Timer
        self.connection_pub_ = self.create_publisher(Bool, "/ping", qos_profile=baseQoS)

    def pingCB_(self):
        msg = Bool()
        msg.data = True
        self.connection_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    service = ConnectionServer()

    try:
        rclpy.spin(service)
    except:
        rclpy.utilities.try_shutdown()
