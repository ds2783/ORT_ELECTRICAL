#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import rclpy.utilities

from sensor_msgs.msg import BatteryState, Joy
from std_msgs.msg import Bool, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from ort_interfaces.msg import CameraRotation, BatteryInfo

from mission_control.stream.stream_client import ServerClient
from mission_control.config.mappings import BUTTONS
from mission_control.config.network import COMM_PORT, PORT_MAIN_BASE, PI_IP, tofQoS
from mission_control.config.gui import QR_DIRECTORY

from qreader import QReader
from multiprocessing.connection import Client

import numpy as np
import datetime
from PIL import Image
import json
import time

class BaseNode(Node):
    def __init__(self, port):
        super().__init__("base")
        # STATE OBJECTS
        self.main_cam = ServerClient(PI_IP, PORT_MAIN_BASE)
        self.qreader_ = QReader()

        self.address = ("localhost", port)
        try:
            self.comms_ = Client(self.address, authkey=b"123")
            self.get_logger().info("Connection Successful.")
            self.try_again = False
        except:
            self.try_again = True
        # ---

        # TIMERS
        self.ping_timer_ = self.create_timer(0.2, self.pingCB_)
        # ----

        # Topics ---------------------
        # Subscriptions
        self.controller_sub_ = self.create_subscription(Joy, "joy", self.controlCB_, 10)

        self.qr_tof_sub_ = self.create_subscription(
            Float32, "/distance_sensor/qr_code", self.qTofCB_, qos_profile=tofQoS
        )

        self.bottom_tof_sub_ = self.create_subscription(
            Float32, "/distance_sensor/optical_flow", self.oTofCB_, qos_profile=tofQoS
        )

        self.euler_angles_sub_ = self.create_subscription(
            Vector3, "/elysium/euler_angles", self.eulerCB_, 10
        )
        self.odom_sub_ = self.create_subscription(
            Odometry, "/elysium/odom", self.odomCB_, 10
        )

        self.camera_angles_sub_ = self.create_subscription(
            CameraRotation, "/elysium/cam_angles", self.camCB_, 10
        )

        self.battery_sub_ = self.create_subscription(
            BatteryInfo, "/battery_monitor", self.battCB_, qos_profile=tofQoS
        )

        # Publishers
        self.connection_pub_ = self.create_publisher(Bool, "ping", 10)
        # ----------------------------

        # Variables ------------------
        self.qr_button_ = False
        self.last_qr = "None"

        self.eulerAngles = Vector3(x=0.0, y=0.0, z=0.0)
        self.odom = Odometry()
        self.qr_tof_dist = Float32(data=0.0)
        self.op_tof_dist = Float32(data=0.0)
        self.cam_rotation = CameraRotation(z_axis=0.0, x_axis=0.0)

        # Elysium Position
        self.elysium_x = 0.0
        self.elysium_y = 0.0
        self.elysium_z = 0.0

        # Elysium Battery
        self.soc = 0.0

        # Recover Old Codes
        self.scanned_codes = {}
        try:
            with open('qr_data.json', 'r') as fp:
                self.scanned_codes = json.load(fp)
            self.get_logger().info(
                "This data was recovered: " + str(self.scanned_codes)
            )
        except:
            self.get_logger().info("No CSV to recover from.")
        self.start_time = time.monotonic()
        # Added this state variable in case Python interpreter is too dump to 
        # figure out that (now - self.start_time) < 1.0 will never be true
        # again after 1 second.
        self.qr_ping = True
        # ----------------------------
        self.get_logger().info("The Base Station has been initialised.")

    def controlCB_(self, msg: Joy):
        trigger_pressed = msg.buttons[BUTTONS["CIRCLE"]]
        if trigger_pressed and not self.qr_button_:
            # CAPTURE QR-CODE
            self.qr_button_ = True
            image = self.main_cam.get_picture(self.get_logger)
            if image is not None:
                self.get_logger().info("Image sent for processing.")
                qreader_out = self.qreader_.detect_and_decode(image=image)

                self.last_qr = str(qreader_out)
                self.sendComms("qr---:" + self.last_qr)

                # assuming y is the forward coordinate
                # (still to be tested)
                # raycasted vector ->
                relative_elysium_pos = np.array([[0.0], [self.qr_tof_dist.data]])
                # x -> yaw, which is rotation around the z_axis
                xy_plane_rotation = self.eulerAngles.x + self.cam_rotation.z_axis

                displacement = rotate_vector2D(xy_plane_rotation, relative_elysium_pos)

                # Calculate the distance from the plane the rover inhabits
                # rotation x_axis -> pitch
                offset = displacement * np.cos(self.cam_rotation.x_axis)

                x_dist = self.elysium_x + offset[0][0]
                y_dist = self.elysium_y + offset[1][0]

                # list comprension to filter out None's
                qr_final = [x for x in qreader_out if x is not None]

                for code in qr_final:
                    date = datetime.datetime.now()
                    fname = f"qr_{len(self.scanned_codes)}_{date.year}_{date.month}_{date.day}.jpeg"
                    self.scanned_codes[code] = {
                            "x": x_dist,
                            "y": y_dist,
                            "distance": np.sqrt(x_dist ** 2 + y_dist ** 2),
                            "filename": fname,
                            }

                    im = Image.fromarray(image)
                    im.save(f"{QR_DIRECTORY}{fname}")
               
                self.get_logger().info(str(self.scanned_codes))
                if len(qr_final) >= 1:
                    self.get_logger().info("Sending JSON object.")
                    self.sendComms("qrdic:" + json.dumps(self.scanned_codes))

                with open('qr_data.json', 'w') as fp:
                    json.dump(self.scanned_codes, fp)
                
                self.get_logger().info("Image processed and CSV updated.")
            else:
                self.get_logger().warn("No image available for processing.")

        elif not trigger_pressed and self.qr_button_:
            # Ensure only one capture even per press
            self.qr_button_ = False

    def pingCB_(self):
        msg = Bool()
        msg.data = True
        self.connection_pub_.publish(msg)

        if self.try_again == True:
            try:
                self.comms_ = Client(self.address, authkey=b"123")
                self.try_again = False
            except:
                pass
        now = time.monotonic()

        # Allow time for GUI to fully load.
        if (now - self.start_time) < 1.0 and self.qr_ping:
            self.sendComms("qrdic:" + json.dumps(self.scanned_codes))
        else:
            self.qr_ping = False

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

        self.sendComms("q-tof:" + f"{self.qr_tof_dist.data:2f}")
        self.sendComms("o-tof:" + f"{self.op_tof_dist.data:2f}")

    def qTofCB_(self, msg: Float32):
        self.qr_tof_dist = msg

    def oTofCB_(self, msg: Float32):
        self.op_tof_dist = msg

    def camCB_(self, msg: CameraRotation):
        self.cam_rotation = msg
        self.sendComms("cam_y:" + f"{self.cam_rotation.z_axis:2f}")
        self.sendComms("cam_p:" + f"{self.cam_rotation.x_axis:2f}")

    def battCB_(self, msg: BatteryInfo):
        self.soc = msg.soc
        self.sendComms("soc--:" + f"{self.soc:2f}")

    def sendComms(self, msg):
        if self.try_again == False:
            try:
                self.comms_.send(msg)
            except:
                self.get_logger().warn("Non comms link found.")


def rad_degrees(num):
    return (180 / np.pi) * num


# Euler angle in rads
# vector2D -> np.array([[x], [y]])
def rotate_vector2D(euler_angle, vector2D):
    rotation = np.array(
        [
            [np.cos(euler_angle), -np.sin(euler_angle)],
            [np.sin(euler_angle), np.cos(euler_angle)],
        ]
    )
    return np.matmul(rotation, vector2D)


def main(args=None):
    rclpy.init(args=args)

    # has to be initialised this way round!
    base = BaseNode(COMM_PORT)
    try:
        rclpy.spin(base)
    # Cleanup After Shutdown
    except:
        base.destroy_node()
        rclpy.utilities.try_shutdown()
