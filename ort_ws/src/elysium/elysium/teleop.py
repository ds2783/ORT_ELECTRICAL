import rclpy
from rclpy.node import Node
import rclpy.utilities
import rclpy.executors
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.action.server import ActionServer

from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import Joy
from ort_interfaces.msg import CameraRotation
from ort_interfaces.srv import Vec2Pos
from ort_interfaces.action import Calibrate

from elysium.config.mappings import AXES
from elysium.config.controls import (
    CAMERA_SENSITIVITY,
    CAMERA_SERVO_X,
    CAMERA_SERVO_Z,
    OFFSET,
)

import time
from functools import partial
from numpy import pi
from dataclasses import dataclass
from adafruit_servokit import ServoKit

CODE_TERMINATE = 0
CODE_CONTINUE = 1


@dataclass
class twist:
    linear: float
    rotation: float


@dataclass
class rotation2D:
    z_axis: float
    x_axis: float


class TelepresenceOperations(Node):
    def __init__(self):
        super().__init__("teleop")

        node_cb_group = MutuallyExclusiveCallbackGroup()
        service_cb_group = MutuallyExclusiveCallbackGroup()
        connection_cb_group = MutuallyExclusiveCallbackGroup()

        # Topics
        self.controller_commands_sub_ = self.create_subscription(
            Joy, "joy", self.teleopCB_, 10, callback_group=node_cb_group
        )
        self.base_ping_sub_ = self.create_subscription(
            Bool,
            "ping",
            self.confirmConnectionCB_,
            10,
            callback_group=connection_cb_group,
        )

        # Publishers
        self.cam_angles__pub_ = self.create_publisher(
            CameraRotation, "/elysium/cam_angles", 10
        )
        self.optical_factor_pub_ = self.create_publisher(
            Float32, "/elysium/ofs_calibration", 10
        )

        # Services
        self.optical_client_ = self.create_client(
            Vec2Pos, "/elysium/srv/position", callback_group=service_cb_group
        )

        # Action Server
        self.calibrate_optical_ = ActionServer(
            self,
            Calibrate,
            "/optical_flow/calibrate",
            self.actionServerCB_,
            callback_group=node_cb_group,
        )

        # State -
        self.state = twist(0, 0)
        self.target = twist(0, 0)

        self.z_increment = 0
        self.x_increment = 0

        # Setting Zeroed rotation for camera servos
        self.cam_angles_ = rotation2D(90.0, 90.0)

        # Connection timer
        self.last_connection_ = time.time_ns()
        self.connection_timer_ = self.create_timer(0.4, self.shutdownCB_, node_cb_group)
        self.driver_timer_ = self.create_timer(0.02, self.driveCB_, node_cb_group)

        # Servo Offset control
        self.offset_ = OFFSET
        self.kit_ = ServoKit(channels=16)

        # Optical Calibration
        self.opt_x = 0
        self.opt_y = 0

        sleep_seconds = 2
        self.rate = self.create_rate(sleep_seconds)

    def actionServerCB_(self, goal_handle):
        self.get_logger().info("Executing goal.")

        CALIBRATE_OFS = 2
        if goal_handle.request.code == CALIBRATE_OFS:
            self.target.linear = 0
            self.target.rotation = 0
            self.drive()

            resp1 = self.get_optical_pos()
            resp2 = None
            self.get_logger().info("x: " + str(self.opt_x) + " y: " + str(self.opt_y))
            self.get_logger().info("First response is: " + str(resp1))

            if resp1 == CODE_CONTINUE:
                x1, y1 = self.opt_x, self.opt_y
                self.target.linear = 1
                self.drive()
                
                # sleep for $(sleep_seconds) while rover drives
                self.rate.sleep()

                self.target.linear = 0
                self.drive()

                resp2 = self.get_optical_pos()
                self.get_logger().info(
                    "x: " + str(self.opt_x) + " y: " + str(self.opt_y)
                )
                self.get_logger().info("Second response is: " + str(resp2))
                if resp2 == CODE_CONTINUE:
                    x2, y2 = self.opt_x, self.opt_y

                    y_dist = y2 - y1

                    factor = Float32(data=1 / y_dist)
                    self.optical_factor_pub_.publish(factor)

                    goal_handle.succeed()
                    result = Calibrate.Result()
                    result.result = 0
                    return result

            if resp1 == CODE_TERMINATE or resp2 == CODE_TERMINATE:
                self.get_logger().warn(
                    "Initial and final positions could not be obtained. No calibration was aquired."
                )
                goal_handle.succeed()
                result = Calibrate.Result()
                result.result = 1
                return result

        else:
            goal_handle.succeed()
            result = Calibrate.Result()
            result.result = 2
            self.get_logger().warn(
                "Unrecognised OP-Code recieved from Calibration Client."
            )
            return result

    def get_optical_pos(self):
        self.request_complete = False
        self.server_unavailable = False
        self.request_optical_pos()

        now = time.monotonic()
        timeout = time.monotonic()
        while not self.request_complete and (now - timeout) < 5.0:
            now = time.monotonic()
            if self.server_unavailable:
                return CODE_TERMINATE

        if self.request_complete:
            return CODE_CONTINUE
        else:
            return CODE_TERMINATE

    def request_optical_pos(self):
        timeout = time.monotonic()
        now = time.monotonic()
        while not self.optical_client_.wait_for_service(1.0) and (now - timeout) < 2.0:
            now = time.monotonic()
            self.get_logger().warn("Waiting for connection to position service.")

        if (now - timeout) < 2.0:
            request = Vec2Pos.Request()
            request.request = True
            future = self.optical_client_.call_async(request)
            future.add_done_callback(partial(self.complete_requestCB_))
        else:
            self.get_logger().warn(
                "Optical position calibration service is unavailable."
            )
            self.server_unavailable = True

    def complete_requestCB_(self, future):
        response = future.result()
        self.opt_x = response.x
        self.opt_y = response.y
        self.request_complete = True
        self.get_logger().info("Opt distances set.")

    def confirmConnectionCB_(self, msg: Bool):
        self.last_connection_ = time.time_ns()

    def shutdownCB_(self):
        if (time.time_ns() > self.last_connection_ + 1e9):
            self.get_logger().warn("Lost connection, setting movement to zero.")
            self.target.linear = 0
            self.target.rotation = 0
            self.drive()

    def teleopCB_(self, msg: Joy):
        # DRIVE -----------------
        self.target.linear = msg.axes[AXES["TRIGGERRIGHT"]]
        self.target.linear -= msg.axes[AXES["TRIGGERLEFT"]]
        # goes from 1 to -1, therefore difference between the two
        # should be halved.
        self.target.linear /= 2
        self.target.rotation = -msg.axes[AXES["LEFTX"]]
        # ------------------------

        self.z_increment = msg.axes[AXES["RIGHTX"]] * CAMERA_SENSITIVITY
        self.x_increment = msg.axes[AXES["RIGHTY"]] * CAMERA_SENSITIVITY

        # publish camera rotation, note 90degrees servo rotation -> 0degrees around the axis
        camera_rotation_msg = CameraRotation(
            z_axis=float(float_to_rad(self.cam_angles_.z_axis - 90)),
            x_axis=float(float_to_rad(self.cam_angles_.x_axis - 90)),
        )
        self.cam_angles__pub_.publish(camera_rotation_msg)

    def driveCB_(self):
        self.drive()
        self.camera_rotate()

    def bound_range(self, value):
        if value > 1:
            value = 1
        elif value < -1:
            value = -1
        return value

    def bound_180(self, value):
        if value > 180:
            value = 180
        elif value < 0:
            value = 0
        return value

    def drive(self):
        left_side = self.bound_range(self.target.linear + 0.5 * self.target.rotation)
        right_side = self.bound_range(-self.target.linear + 0.5 * self.target.rotation)

        # TESTING TO MAKE WHEELS MORE SENSITIVE
        left_side = self.bound_180(90.0 + 60 * left_side + self.offset_)
        right_side = self.bound_180(90.0 + 60 * right_side + self.offset_)

        for i in range(0, 3):
            self.kit_.servo[i].angle = right_side
        for i in range(3, 6):
            self.kit_.servo[i].angle = left_side

        # self.get_logger().info(
        #     "left_side: " + str(left_side) + " right_side: " + str(right_side)
        # )

    def camera_rotate(self):
        self.cam_angles_.z_axis = self.bound_180(
            float(self.z_increment + self.cam_angles_.z_axis)
        )
        self.cam_angles_.x_axis = self.bound_180(
            float(-self.x_increment + self.cam_angles_.x_axis)
        )
        # POSITIONAL
        self.kit_.servo[CAMERA_SERVO_Z].angle = self.cam_angles_.z_axis
        # CONTIOUS
        self.kit_.servo[CAMERA_SERVO_X].angle = self.cam_angles_.x_axis

        # self.get_logger().info(
        #     "z_axis: "
        #     + str(self.cam_angles_.z_axis)
        #     + " x_axis: "
        #     + str(self.cam_angles_.x_axis)
        # )


def float_to_rad(val):
    return (pi / 180) * val


def main(args=None):
    rclpy.init(args=args)

    tele = TelepresenceOperations()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(tele)
    try:
        executor.spin()
    except KeyboardInterrupt:
        tele.get_logger().warn(f"KeyboardInterrupt triggered.")
    finally:
        tele.destroy_node()
        rclpy.utilities.try_shutdown()
