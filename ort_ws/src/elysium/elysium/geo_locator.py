from nav_msgs import msg
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    Quaternion,
    Vector3,
    Twist,
    Point,
    Pose,
    PoseWithCovariance,
    TwistWithCovariance,
)
from std_msgs.msg import Float32, Header
from ort_interfaces.msg import OpticalFlow

from elysium.config.sensors import DISTANCE_SENSOR_REFRESH_PERIOD, OPTICAL_CALIBRATION
from elysium.config.network import DIAGNOSTIC_PERIOD

import numpy as np
from scipy.spatial.transform import Rotation as R


class GeoLocator(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # Subscriptions ---------
        self.tof_sub_ = self.create_subscription(
            Float32,
            "/distance_sensor/optical_flow",
            self.tofCB_,
            qos_profile=qos_profile_sensor_data,
        )
        self.quaternion_sub_ = self.create_subscription(
            Quaternion, "/imu/quat", self.imuCB_, qos_profile_sensor_data
        )
        self.optical_sub_ = self.create_subscription(
            OpticalFlow,
            "/optical_flow/increment",
            self.opticalCB_,
            qos_profile_sensor_data,
        )
        self.reset_pos_ = self.create_subscription(Bool, "/elysium/reset_pos", self.resetCB_, 10)
        # ----------------------

        # Publishers -----------
        self.euler_angles_pub_ = self.create_publisher(
            Vector3, "/elysium/euler_angles", 10
        )
        self.odom_pub_ = self.create_publisher(
            Odometry, "/elysium/odom", 10
        )
        # ----------------------

        # Timers ----------------
        self.create_timer(DIAGNOSTIC_PERIOD, self.publish_)
        # ------------------------

        self.euler_angles = Vector3()
        self.rotation_ = Quaternion()
        self.quat_ = np.array([1.0, 0.0, 0.0, 0.0])
        self.distance_sensor_dt_ = DISTANCE_SENSOR_REFRESH_PERIOD

        # Cartesian Displacement - Initiale Values
        # TO DO:
        # Add csv file to load previous displacements incase of crash
        self.z_prev_ = 0.0
        self.z_pos = 0.0
        self.x_pos = 0.0
        self.y_pos = 0.0

        self.dx = 0
        self.dy = 0
        # avoids division by zero error
        self.dt = 0.0001

    def resetCB_(self, msg: Bool):
        if msg.data == True:
            self.x_pos = 0
            self.y_pos = 0

    def tofCB_(self, msg: Float32):
        self.z_prev_ = self.z_pos
        self.z_axis = msg.data

    def imuCB_(self, msg: Quaternion):
        self.rotation_ = msg
        # x,y,z,w
        self.quat_ = np.array(
            [self.rotation_.x, self.rotation_.y, self.rotation_.z, self.rotation_.w]
        )

    def opticalCB_(self, msg: OpticalFlow):
        euler = R.from_quat(
            [self.rotation_.x, self.rotation_.y, self.rotation_.z, self.rotation_.w]
        ).as_euler("zyx", degrees=False)
        roll, pitch, yaw = euler

        self.euler_angles = Vector3(x=yaw, y=pitch, z=roll)

        # rotate the dx and dy increments around the yaw
        increment = np.array([[msg.dx], [msg.dy]])
        rotated_increment = rotate_vector2D(yaw, increment)

        self.dx = rotated_increment[0][0]
        self.dy = rotated_increment[1][0]
        self.x_pos += self.dx * OPTICAL_CALIBRATION
        self.y_pos += self.dy * OPTICAL_CALIBRATION

        self.dt = msg.dt

    def publish_(self):
        self.euler_angles_pub_.publish(self.euler_angles)

        odom_msg = Odometry(
            header=Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id="odom",
            ),
            child_frame_id="base_link",
            pose=PoseWithCovariance(
                pose=Pose(position=Point(x=self.x_pos, y=self.y_pos, z=self.z_pos))
            ),
            twist=TwistWithCovariance(
                twist=Twist(
                    linear=Vector3(
                        x=self.dx / self.dt,
                        y=self.dy / self.dt,
                        z=(self.z_pos - self.z_prev_) / self.distance_sensor_dt_,
                    )
                )
            ),
        )

        self.odom_pub_.publish(odom_msg)

# Euler angle in rads
# vector2D -> np.array([[x], [y]])
def rotate_vector2D(euler_angle, vector2D):
    rotation = np.array([[np.cos(euler_angle), -np.sin(euler_angle)], [np.sin(euler_angle), np.cos(euler_angle)]])
    return np.matmul(rotation, vector2D)    


def main(args=None):
    rclpy.init(args=args)

    location_node = GeoLocator("location_service")
    rclpy.spin(location_node)
    rclpy.shutdown()
