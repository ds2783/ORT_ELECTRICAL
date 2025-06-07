#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3

from mission_control.stream.stream_client import StreamClient
from mission_control.config.mappings import AXES, BUTTONS
from mission_control.config.network import COMM_PORT, PORT_MAIN_BASE, PI_IP

import glfw
from multiprocessing import Process
from qreader import QReader
from multiprocessing.connection import Client

import numpy as np
import csv


class BaseNode(Node):
    def __init__(self, port):
        super().__init__("base")
        # CONSTANTS -- DO NOT REASSIGN
        # ---

        # STATE OBJECTS
        self.main_cam = StreamClient(
            "Stereo", PI_IP, "udp", PORT_MAIN_BASE, 640, 480, stereo=False
        )
        self.qreader_ = QReader()

        address = ("localhost", port)
        self.comms_ = Client(address, authkey=b"123")
        # ---

        # TIMERS
        self.ping_timer_ = self.create_timer(0.2, self.pingCB_)
        # ----

        # Topics ---------------------
        self.connection_pub_ = self.create_publisher(Bool, "ping", 10)
        self.controller_sub_ = self.create_subscription(Joy, "joy", self.controlCB_, 10)

        self.euler_angles_pub_ = self.create_subscription(
            Vector3, "/elysium/euler_angles", self.eulerCB_, 10
        )
        self.odom_pub_ = self.create_subscription(
            Odometry, "/elysium/odom", self.odomCB_, 10
        )
        # ----------------------------

        # Variables ------------------
        self.qr_button_ = False
        self.last_qr = "None"

        self.eulerAngles = Vector3()
        self.odom = Odometry()

        # Elysium Position
        self.elysium_x = 0
        self.elysium_y = 0
        self.elysium_z = 0

        # Recover Old Codes
        self.scanned_codes = {}
        try:
            with open("qr_data.csv", newline="") as csvfile:
                reader = csv.DictReader(csvfile, delimiter="&", quotechar="|")
                headers = reader.fieldnames
                if headers is not None:
                    for index, row in enumerate(reader):
                        self.scanned_codes[headers[index]] = row[headers[index]]
            self.get_logger().info(
                "This data was recovered: " + str(self.scanned_codes)
            )
        except:
            self.get_logger().info("No CSV to recover from.")
        # ----------------------------
        self.get_logger().info("The Base Station has been initialised.")

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
                self.sendComms("qr---:" + self.last_qr)

                for code in qreader_out:
                    if code != None:
                        self.scanned_codes[code] = (
                            self.elysium_x,
                            self.elysium_y,
                            self.elysium_z,
                        )

                with open("qr_data.csv", "w", newline="") as csvfile:
                    writer = csv.DictWriter(
                        csvfile, self.scanned_codes.keys(), delimiter="&", quotechar="|"
                    )
                    writer.writeheader()
                    writer.writerow(self.scanned_codes)

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

    def eulerCB_(self, msg: Vector3):
        self.eulerAngles = msg
        self.sendComms("yaw--:" + f"{rad_degrees(msg.x):2f}")
        self.sendComms("pitch:" + f"{rad_degrees(msg.y):2f}")
        self.sendComms("roll-:" + f"{rad_degrees(msg.z):2f}")

    def odomCB_(self, msg: Odometry):
        self.odom = msg
        self.elysium_x = msg.pose.pose.position.x
        self.elysium_y = msg.pose.pose.position.y
        self.elysium_z = msg.pose.pose.position.z

        self.sendComms("x----:" + f"{self.elysium_x:2f}")
        self.sendComms("y----:" + f"{self.elysium_y:2f}")
        self.sendComms("z----:" + f"{self.elysium_z:2f}")

        self.sendComms("x_vel:" + f"{msg.twist.twist.linear.x:2f}")
        self.sendComms("y_vel:" + f"{msg.twist.twist.linear.y:2f}")
        self.sendComms("z_vel:" + f"{msg.twist.twist.linear.z:2f}")

    def sendComms(self, msg):
        self.comms_.send(msg)


def rad_degrees(num):
    return (180 / np.pi) * num


def main(args=None):
    rclpy.init(args=args)

    # has to be initialised this way round!
    base = BaseNode(COMM_PORT)

    rclpy.spin(base)
    # Cleanup After Shutdown
    rclpy.shutdown()
