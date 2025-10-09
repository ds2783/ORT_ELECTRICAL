#!/usr/bin/env python3
import rclpy
import rclpy.utilities
import rclpy.executors
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float32, Int8
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from ort_interfaces.msg import CameraRotation, BatteryInfo, OpticalFlowCalibration
from ort_interfaces.srv import DistanceData

from mission_control.stream.stream_client import ServerClient
from mission_control.config.mappings import BUTTONS
from mission_control.config.network import (
    COMM_PORT,
    PI_IP,
    CODE_CONTINUE,
    CODE_TERMINATE,
    PORT_MAIN_SERVER,
    PORT_SECONDARY_SERVER,
    flagQoS,
    queueToS,
    periodicQoS,
)
from mission_control.config.gui import QR_DIRECTORY

from qreader import QReader
from multiprocessing.connection import Client
from threading import Thread

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
        comms_group = MutuallyExclusiveCallbackGroup()
        service_group = MutuallyExclusiveCallbackGroup()
        # ---------------
        # STATE OBJECTS
        self.main_cam = ServerClient(PI_IP, PORT_MAIN_SERVER)
        self.secondary_cam = ServerClient(PI_IP, PORT_SECONDARY_SERVER)

        self.stream_thread = Thread(target=self.start_stream)
        self.stream_thread.start() 
        
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
        self.comms_timer_ = self.create_timer(
            0.5, self.commsCB_, callback_group=comms_group
        )
        # ----

        # Topics ---------------------
        # Subscriptions
        self.controller_sub_ = self.create_subscription(
            Joy, "joy", self.controlCB_, qos_profile=qos_profile_sensor_data
        )

        self.bottom_tof_sub_ = self.create_subscription(
            Float32, "/distance_sensor/optical_flow", self.oTofCB_, qos_profile=queueToS
        )

        self.euler_angles_sub_ = self.create_subscription(
            Vector3, "/elysium/euler_angles", self.eulerCB_, qos_profile=queueToS
        )
        self.odom_sub_ = self.create_subscription(
            Odometry, "/elysium/odom", self.odomCB_, qos_profile=queueToS
        )

        self.camera_angles_sub_ = self.create_subscription(
            CameraRotation, "/elysium/cam_angles", self.camCB_, 10
        )

        self.battery_sub_ = self.create_subscription(
            BatteryInfo, "/battery_monitor", self.battCB_, qos_profile=queueToS
        )

        self.gps_dist_sub_ = self.create_subscription(
            Float32, "/elysium/gps_dist", self.gpsCB_, qos_profile=queueToS
        )
        self.optical_calibration_ = self.create_subscription(
            OpticalFlowCalibration, "/elysium/optical_factor", self.ofs_calCB_, qos_profile=periodicQoS
        )

        # Control GUI Subscriptions
        self.rock_num_sub_ = self.create_subscription(
            Int8, "/rock_num", self.num_rock_CB_, qos_profile=flagQoS
        )

        # Services
        self.tof_client_ = self.create_client(
            DistanceData, "/elysium/cam/distance_service", callback_group=service_group
        )
        # -----------------------

        # Variables ------------------
        self.qr_button_ = False
        self.last_qr = "None"
        self.capture_num_ = 0

        self.eulerAngles = Vector3(x=0.0, y=0.0, z=0.0)
        self.odom = Odometry()
        self.op_tof_dist = Float32(data=0.0)
        self.qr_tof_dist = 0.0
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
        self.optical_angle = 0.0
        self.optical_calibration_factors = []
        self.optical_calibration_angles = []
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

        # 80Hz rate
        TIGHT_LOOP_RATE = 80
        self.rate = self.create_rate(TIGHT_LOOP_RATE)
        # ----------------------------

        # GUI
        self.rock_num_ = 0

        self.get_logger().info("The Base Station has been initialised.")

    def controlCB_(self, msg: Joy):
        circle = msg.buttons[BUTTONS["CIRCLE"]]
        if circle and not self.qr_button_:
            # CAPTURE QR-CODE
            resp = CODE_TERMINATE
            if resp == CODE_TERMINATE:
                self.get_logger().error(
                    "Distance measured from ToF not available. Measurement not updated."
                )

            self.qr_button_ = True
            image = self.main_cam.get_picture(self.get_logger)
            if image is not None:
                self.scan_qr(image, self.rock_num_)
            else:
                self.get_logger().warn("No image available for processing.")

        elif not circle and self.qr_button_:
            # Ensure only one capture even per press
            self.qr_button_ = False

        square = msg.buttons[BUTTONS["SQUARE"]]
        if square:
            self.get_logger().info("Attempting to read cam distance.")
            self.get_request(self.request_tof_dist)

    def scan_qr(self, image, rock_num):
        self.get_logger().info("Image sent for processing.")
        qreader_out = self.qreader_.detect_and_decode(image=image)

        self.last_qr = str(qreader_out)
        self.capture_num_ += 1
        self.sendComms("qr---:" + self.last_qr + " : " + str(self.capture_num_))

        # assuming y is the forward coordinate
        # (still to be tested)
        # raycasted vector  ->
        relative_elysium_pos = np.array([[0.0], [self.qr_tof_dist]])
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
        elysium_pos_pre_correction = np.array(
            [[self.elysium_x_raw], [self.elysium_y_raw]]
        )
        elysium_pos = (
            rotate_vector2D(-self.optical_angle, elysium_pos_pre_correction)
            * self.optical_factor
        )

        self.elysium_x = elysium_pos[0][0]
        self.elysium_y = elysium_pos[1][0]

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

        reliable_sensor = self.get_more_reliable_sensor(
            elysium_dist, self.gps_dist_, gps_reliability
        )

        for code in qr_final:
            date = datetime.datetime.now()
            fname = (
                f"{code}_rock_{rock_num}_date_{date.year}_{date.month}_{date.day}.jpeg"
            )
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
                "rock-num": rock_num,
                "distance-from-cam": self.qr_tof_dist,
            }

            # Rotate camera from mount point.
            im = Image.fromarray(image).rotate(270)

            if not Path(QR_DIRECTORY).is_dir():
                Path(QR_DIRECTORY).mkdir()

            im.save(QR_DIRECTORY + fname)

        if len(qr_final) >= 1:
            self.get_logger().info("Sending JSON object.")
            self.sendComms("qrdic")
            self.sendComms(self.scanned_codes)

        with open("qr_data.json", "w") as fp:
            json.dump(self.scanned_codes, fp)

        self.get_logger().info("Image processed and CSV updated.")

    def commsCB_(self):
        if self.try_again == True:
            try:
                self.comms_ = Client(self.address, authkey=b"123")
                self.try_again = False
            except:
                pass
        now = time.monotonic()

        # Allow time for GUI to fully load.
        if (now - self.start_time) < 1.0 and self.qr_ping:
            self.sendComms("qrdic")
            self.sendComms(self.scanned_codes)

        else:
            self.qr_ping = False

        data_dump = {
            "x": self.elysium_x,
            "y": self.elysium_y,
            "z": self.elysium_z,
            "x_vel": self.odom.twist.twist.linear.x,
            "y_vel": self.odom.twist.twist.linear.y,
            "z_vel": self.odom.twist.twist.linear.z,
            "yaw": rad_degrees(self.eulerAngles.x),
            "pitch": rad_degrees(self.eulerAngles.y),
            "roll": rad_degrees(self.eulerAngles.z),
            "cam_y": self.cam_rotation.z_axis,
            "cam_p": self.cam_rotation.x_axis,
            "o-tof": self.op_tof_dist.data,
            "gps-d": self.gps_dist_,
            "soc": self.soc,
        }

        self.sendComms("dump")
        self.sendComms(data_dump)

    def ofs_calCB_(self, msg: OpticalFlowCalibration):
        prev_val = self.optical_factor
        self.optical_factor = msg.optical_factor
        self.optical_angle = msg.angle
        self.optical_calibration_factors = msg.sample_factors
        self.optical_calibration_angles = msg.sample_angles
        self.number_of_calibration_samples = msg.num_samples

        if prev_val != self.optical_factor:
            self.get_logger().info(
                "Optical calibration factor successfully set to: "
                + str(self.optical_factor)
            )

            self.get_logger().info(
                "Optical calibration angle successfully set to: "
                + str(self.optical_angle)
            )

            self.update_qr_codes()
            self.sendComms("qrdic")
            self.sendComms(self.scanned_codes)

            with open("qr_data.json", "w") as fp:
                json.dump(self.scanned_codes, fp)

    def update_qr_codes(self):
        for key in self.scanned_codes.keys():
            entry = self.scanned_codes[key]

            # make sure the ofs values are up to date
            elysium_pos_pre_correction = np.array([[entry["x-ofs"]], [entry["y-ofs"]]])
            elysium_pos = (
                rotate_vector2D(-self.optical_angle, elysium_pos_pre_correction)
                * self.optical_factor
            )

            elysium_x = elysium_pos[0][0]
            elysium_y = elysium_pos[1][0]

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

    def eulerCB_(self, msg: Vector3):
        self.eulerAngles = msg

    def odomCB_(self, msg: Odometry):
        self.odom = msg
        self.elysium_x_raw = msg.pose.pose.position.x
        self.elysium_y_raw = msg.pose.pose.position.y
        self.elysium_z_raw = msg.pose.pose.position.z

        elysium_pos_pre_correction = np.array(
            [[self.elysium_x_raw], [self.elysium_y_raw]]
        )
        elysium_pos = (
            rotate_vector2D(-self.optical_angle, elysium_pos_pre_correction)
            * self.optical_factor
        )

        self.elysium_x = float(elysium_pos[0][0])
        self.elysium_y = float(elysium_pos[1][0])
        self.elysium_z = self.elysium_z_raw * self.optical_factor

    def oTofCB_(self, msg: Float32):
        self.op_tof_dist = msg

    def camCB_(self, msg: CameraRotation):
        self.cam_rotation = msg

    def battCB_(self, msg: BatteryInfo):
        self.soc = msg.soc

    def gpsCB_(self, msg: Float32):
        self.gps_dist_ = msg.data
        self.last_gps_ = time.monotonic()

    def sendComms(self, msg):
        if self.try_again == False:
            try:
                self.comms_.send(msg)
            except:
                self.get_logger().warn("Non comms link found.")

    def get_request(self, server_func):
        self.request_complete = False
        self.server_unavailable = False
        server_func()

        now = time.monotonic()
        timeout = time.monotonic()
        while not self.request_complete and (now - timeout) < 2.0:
            self.rate.sleep()
            now = time.monotonic()
            if self.server_unavailable:
                return CODE_TERMINATE

        if self.request_complete:
            return CODE_CONTINUE
        else:
            return CODE_TERMINATE

    def request_tof_dist(self):
        timeout = time.monotonic()
        now = time.monotonic()
        while not self.tof_client_.wait_for_service(1.5):
            now = time.monotonic()
            self.rate.sleep()
            self.get_logger().warn("Waiting for connection to distance service.")
        if (now - timeout) < 1.5:
            request = DistanceData.Request()
            request.request = True
            future = self.tof_client_.call_async(request)
            future.add_done_callback(partial(self.complete_tof_requestCB_))
        else:
            self.get_logger().warn("Distance calibration service is unavailable.")
            self.server_unavailable = True

    def complete_tof_requestCB_(self, future):
        response = future.result()
        if response.data_retrieved:
            self.qr_tof_dist = response.distance
            self.get_logger().info("Camera distance set.")
            self.sendComms("q-tof:" + f"{self.qr_tof_dist:2f}")
        else:
            self.get_logger().error("ToF could not obtain a reading.")
            self.server_unavailable = True
        self.request_complete = True

    def num_rock_CB_(self, msg: Int8):
        self.rock_num_ = msg.data

    def start_stream(self):
        self.main_cam.start_stream()
        self.secondar_cam.start_stream()


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
