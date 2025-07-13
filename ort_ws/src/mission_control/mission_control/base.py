#!/usr/bin/env python3
import rclpy
import rclpy.utilities
import rclpy.executors
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from ort_interfaces.msg import CameraRotation, BatteryInfo, OpticalFlowCalibration
from ort_interfaces.srv import DistanceData

from mission_control.stream.stream_client import ServerClient
from mission_control.config.mappings import BUTTONS
from mission_control.config.network import COMM_PORT, PORT_MAIN_BASE, PI_IP, tofQoS
from mission_control.config.gui import QR_DIRECTORY

from qreader import QReader
from multiprocessing.connection import Client

import numpy as np
import datetime
from PIL import Image
from pathlib import Path
import json
import time
from functools import partial


class BaseNode(Node):
    def __init__(self, port):
        super().__init__("base")
        # CB groups 
        ping_group = MutuallyExclusiveCallbackGroup()
        # ---------------
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
        self.ping_timer_ = self.create_timer(0.2, self.pingCB_, callback_group=ping_group)
        self.cam_timer_ = self.create_timer(0.3, self.get_cam_distCB_)
        # ----

        # Topics ---------------------
        # Subscriptions
        self.controller_sub_ = self.create_subscription(Joy, "joy", self.controlCB_, 10)

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

        self.gps_dist_sub_ = self.create_subscription(
            Float32, "/elysium/gps_dist", self.gpsCB_, 10
        )
        self.optical_calibration_ = self.create_subscription(
            OpticalFlowCalibration, "/elysium/optical_factor", self.ofs_calCB_, 10
        )

        # Publishers
        self.connection_pub_ = self.create_publisher(Bool, "ping", 10)
        # ----------------------------

        # Services
        self.cam_tof_client_ = self.create_client(
            DistanceData,
            "/elysium/cam/distance_service",
        )
        # -----------------------

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
        self.elysium_x_raw = 0.0
        self.elysium_y_raw = 0.0
        self.elysium_z_raw = 0.0

        # initial value is stating uncalibrated
        self.optical_factor = 1.0
        self.optical_calibration_points = []
        self.number_of_calibration_samples = 0
        
        # Elysium Battery
        self.soc = 0.0

        # GPS
        self.gps_dist_ = 0
        self.last_gps_ = None

        # Recover Old Codes
        self.scanned_codes = {}
        try:
            with open("qr_data.json", "r") as fp:
                self.scanned_codes = json.load(fp)
            self.get_logger().info(
                "This data was recovered: " + str(self.scanned_codes)
            )
        except:
            self.get_logger().info("No JSON to recover from.")
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
                # raycasted vector  ->
                relative_elysium_pos = np.array([[0.0], [self.qr_tof_dist.data]])
                # x -> yaw, which is rotation around the z_axis
                xy_plane_rotation = self.eulerAngles.x + self.cam_rotation.z_axis

                displacement = rotate_vector2D(xy_plane_rotation, relative_elysium_pos)

                # Calculate the distance from the plane the rover inhabits
                # rotation x_axis -> pitch
                offset = displacement * np.cos(self.cam_rotation.x_axis)
                offset_x = offset[0][0]
                offset_y = offset[1][0]

                x_ofs = self.elysium_x_raw
                y_ofs = self.elysium_y_raw
                
                # make sure the ofs values are up to date
                self.elysium_x = self.elysium_x_raw * self.optical_factor
                self.elysium_y = self.elysium_y_raw * self.optical_factor

                x_dist = self.elysium_x + offset[0][0]
                y_dist = self.elysium_y + offset[1][0]

                dist = np.sqrt(x_dist**2 + y_dist**2)

                elysium_dist = np.sqrt(self.elysium_x**2 + self.elysium_y**2)
                # list comprension to filter out None's
                qr_final = [x for x in qreader_out if x is not None]

                now = time.monotonic()
                # (if no reasonable fix is aquired, the gps_dist sub won't publish)
                if self.last_gps_:
                    if (now - self.last_gps_) > 2.0:
                        gps_reliability = False
                    else:
                        gps_reliability = True
                else:
                    gps_reliability = False
                
                reliable_sensor = self.get_more_reliable_sensor(elysium_dist, self.gps_dist_, gps_reliability)

                for code in qr_final:
                    date = datetime.datetime.now()
                    fname = f"qr_{len(self.scanned_codes)}_{date.year}_{date.month}_{date.day}.jpeg"
                    self.scanned_codes[code] = {
                        "x": x_dist,
                        "y": y_dist,
                        "x-ofs": x_ofs,
                        "y-ofs": y_ofs,
                        "offset-x": offset_x,
                        "offset-y": offset_y,
                        "distance-ofs-imu": dist,
                        "distance-gps": self.gps_dist_,
                        "filename": fname,
                        "more-reliable": reliable_sensor,
                        "gps-reliability": gps_reliability,
                        "ofs-sample-num": self.number_of_calibration_samples,
                    }

                    # Rotate camera from mount point.
                    im = Image.fromarray(image).rotate(270)

                    if not Path(QR_DIRECTORY).is_dir():
                        Path(QR_DIRECTORY).mkdir()

                    im.save(QR_DIRECTORY+fname)
                
                if len(qr_final) >= 1:
                    self.get_logger().info("Sending JSON object.")
                    self.sendComms("qrdic:" + json.dumps(self.scanned_codes))

                with open("qr_data.json", "w") as fp:
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

    def ofs_calCB_(self, msg: OpticalFlowCalibration):
        prev_val = self.optical_factor
        self.optical_factor = msg.optical_factor
        self.optical_calibration_points = msg.samples
        self.number_of_calibration_samples = msg.num_samples 
        
        if prev_val != self.optical_factor:
            self.get_logger().info(
                "Optical calibration factor successfully set to: "
                + str(self.optical_factor)
            )

            self.update_qr_codes()
            self.sendComms("qrdic:" + json.dumps(self.scanned_codes))

            with open("qr_data.json", "w") as fp:
                json.dump(self.scanned_codes, fp)


    def update_qr_codes(self):
        for key in self.scanned_codes.keys():
            entry = self.scanned_codes[key]
            elysium_x = entry["x-ofs"] * self.optical_factor
            elysium_y = entry["y-ofs"] * self.optical_factor

            x_dist = elysium_x + entry["offset-x"]
            y_dist = elysium_y + entry["offset-y"]
            entry["x"] = x_dist
            entry["y"] = y_dist

            dist = np.sqrt(x_dist**2 + y_dist**2)
            entry["distance-ofs-imu"] = dist

            entry["ofs-sample-num"] = self.number_of_calibration_samples

            elysium_dist = np.sqrt(elysium_x**2 + elysium_y**2)
            entry["more-reliable"] = self.get_more_reliable_sensor(
                elysium_dist, entry["distance-gps"], entry["gps-reliability"]
            )
            self.scanned_codes[key] = entry

    def get_more_reliable_sensor(self, elysium_ofs_dist, gps_dist, gps_reliability):
        if abs(gps_dist - elysium_ofs_dist) > 3.0 and gps_reliability:
            self.get_logger().warn(
                "GPS and calculated coordinates of the Rover disagree by more than 3m."
            )
            self.get_logger().warn(
                "Defaulting to GPS as it is deemed to be more reliable."
            )
            reliable_sensor = "gps"
        else:
            reliable_sensor = "op-imu"
        return reliable_sensor

    def get_cam_distCB_(self):
        timeout = time.monotonic()
        now = time.monotonic()
        while not self.cam_tof_client_.wait_for_service(0.1) and (now - timeout) < 0.1 :
            now = time.monotonic()
        if (now - timeout) < 0.1:
            request = DistanceData.Request()
            request.request = True
            future = self.cam_tof_client_.call_async(request)
            future.add_done_callback(partial(self.complete_tof_requestCB_))
        else:
            self.get_logger().warn("Distance service is unavailable.")

    def complete_tof_requestCB_(self, future):
        response = future.result()
        if response.data_retrieved:
            self.tof_dist = response.distance
            self.sendComms("q-tof:" + f"{self.qr_tof_dist.data:2f}")
        else:
            self.get_logger().error("ToF could not obtain a reading.")

    def eulerCB_(self, msg: Vector3):
        self.eulerAngles = msg
        self.sendComms("yaw--:" + f"{rad_degrees(msg.x):2f}")
        self.sendComms("pitch:" + f"{rad_degrees(msg.y):2f}")
        self.sendComms("roll-:" + f"{rad_degrees(msg.z):2f}")

    def odomCB_(self, msg: Odometry):
        self.odom = msg
        self.elysium_x_raw = msg.pose.pose.position.x
        self.elysium_y_raw = msg.pose.pose.position.y
        self.elysium_z_raw = msg.pose.pose.position.z

        self.elysium_x = self.elysium_x_raw * self.optical_factor
        self.elysium_y = self.elysium_y_raw * self.optical_factor
        self.elysium_z = self.elysium_z_raw * self.optical_factor

        self.sendComms("x----:" + f"{self.elysium_x:2f}")
        self.sendComms("y----:" + f"{self.elysium_y:2f}")
        self.sendComms("z----:" + f"{self.elysium_z:2f}")

        self.sendComms("x_vel:" + f"{msg.twist.twist.linear.x:2f}")
        self.sendComms("y_vel:" + f"{msg.twist.twist.linear.y:2f}")
        self.sendComms("z_vel:" + f"{msg.twist.twist.linear.z:2f}")

    def oTofCB_(self, msg: Float32):
        self.op_tof_dist = msg
        self.sendComms("o-tof:" + f"{self.op_tof_dist.data:2f}")

    def camCB_(self, msg: CameraRotation):
        self.cam_rotation = msg
        self.sendComms("cam_y:" + f"{self.cam_rotation.z_axis:2f}")
        self.sendComms("cam_p:" + f"{self.cam_rotation.x_axis:2f}")

    def battCB_(self, msg: BatteryInfo):
        self.soc = msg.soc
        self.sendComms("soc--:" + f"{self.soc:2f}")

    def gpsCB_(self, msg: Float32):
        self.gps_dist_ = msg.data
        self.last_gps_ = time.monotonic()
        self.sendComms("gps-d:" + f"{self.gps_dist_:2f}")

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

    base = BaseNode(COMM_PORT)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(base)
    try:
        executor.spin()
    except:
        base.destroy_node()
        rclpy.utilities.try_shutdown()
