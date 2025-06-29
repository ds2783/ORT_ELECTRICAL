import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import rclpy.utilities

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
from ort_interfaces.msg import OpticalFlow, GPSStatus
from ort_interfaces.srv import Vec2Pos

from elysium.config.sensors import (
    DISTANCE_SENSOR_REFRESH_PERIOD,
    OPTICAL_CALIBRATION,
    tofQoS,
)
from elysium.config.network import DIAGNOSTIC_PERIOD

import numpy as np
from scipy.spatial.transform import Rotation as R


# TO DO: Integrate GPS, OpticalFlow and IMU to all use the same coordinate system.


class GeoLocator(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # Subscriptions ---------
        self.tof_sub_ = self.create_subscription(
            Float32,
            "/distance_sensor/optical_flow",
            self.tofCB_,
            qos_profile=tofQoS,
        )
        self.quaternion_sub_ = self.create_subscription(
            Quaternion, "/imu/quat", self.imuCB_, qos_profile=qos_profile_sensor_data
        )
        self.optical_sub_ = self.create_subscription(
            OpticalFlow,
            "/optical_flow/increment",
            self.opticalCB_,
            qos_profile=qos_profile_sensor_data,
        )
        self.reset_pos_ = self.create_subscription(
            Bool, "/elysium/reset_pos", self.resetCB_, 10
        )

        self.gps_sub_ = self.create_subscription(
            GPSStatus, "/elysium/gps_data", self.gpsCB_, 10
        )

        self.optical_calibration_ = self.create_subscription(
            Float32, "/elysium/ofs_calibration", self.ofs_calCB_, 10
        )
        # ----------------------

        # Publishers -----------
        self.euler_angles_pub_ = self.create_publisher(
            Vector3, "/elysium/euler_angles", 10
        )
        self.odom_pub_ = self.create_publisher(Odometry, "/elysium/odom", 10)

        self.gps_dist_pub_ = self.create_publisher(Float32, "/elysium/gps_dist", 10)
        # ----------------------

        self.position_service_ = self.create_service(
            Vec2Pos, "/elysium/srv/position", self.positionCB_
        )

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

        # Calibration
        self.calibration_y_move = 0
        self.calibration_x_move = 0

        self.optical_factor = OPTICAL_CALIBRATION

        # GPS Vars
        self.start_lat = None
        self.start_lon = None

        self.lat = None
        self.long = None

    def resetCB_(self, msg: Bool):
        if msg.data == True:
            self.x_pos = 0.0
            self.y_pos = 0.0

            self.start_lat = self.lat
            self.start_lon = self.long
            self.get_logger().info(
                f"Captured start coordinates: {self.start_lat}, {self.start_lon}"
            )

    def positionCB_(self, req, response):
        response.x = self.calibration_x_move
        response.y = self.calibration_y_move
        return response

    def ofs_calCB_(self, msg: Float32):
        self.optical_factor = msg.data
        self.get_logger().info(
            "Optical calibration factor successfuly set to: " + str(self.optical_factor)
        )

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

        self.calibration_y_move += msg.dy
        self.calibration_x_move += msg.dx
        # rotate the dx and dy increments around the yaw
        increment = np.array([[msg.dx], [msg.dy]])
        rotated_increment = rotate_vector2D(yaw, increment)

        self.dx = rotated_increment[0][0]
        self.dy = rotated_increment[1][0]
        self.x_pos += self.dx * self.optical_factor
        self.y_pos += self.dy * self.optical_factor

        self.dt = msg.dt

    def gpsCB_(self, msg):
        # conditions to define a reasonable fix
        if (
            msg.fix_quality > 0
            and msg.latitude
            and msg.longitude
            and msg.pdop < 10
            and msg.hdop < 10
            and msg.vdop < 10
        ):
            self.lat = msg.latitude
            self.long = msg.longitude

            if not self.start_lon or not self.start_lat:
                self.start_lat = self.lat
                self.start_lon = self.long
            else:
                dist = haversine(self.start_lat, self.start_lon, self.lat, self.long)
                msg = Float32(data=float(dist))
                self.gps_dist_pub_.publish(msg)

        else:
            self.get_logger().info("GPS data is ignored as there is no solid fix.")

    def publish_(self):
        self.euler_angles_pub_.publish(self.euler_angles)

        odom_msg = Odometry(
            header=Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id="odom",
            ),
            child_frame_id="base_link",
            pose=PoseWithCovariance(
                pose=Pose(
                    position=Point(
                        x=float(self.x_pos), y=float(self.y_pos), z=float(self.z_pos)
                    )
                )
            ),
            twist=TwistWithCovariance(
                twist=Twist(
                    linear=Vector3(
                        x=float(self.dx / self.dt),
                        y=float(self.dy / self.dt),
                        z=float((self.z_pos - self.z_prev_) / self.distance_sensor_dt_),
                    )
                )
            ),
        )

        self.odom_pub_.publish(odom_msg)


def haversine(lat1, lon1, lat2, lon2):
    R = 6371000  # Earth radius in meters
    phi1 = np.radians(lat1)
    phi2 = np.radians(lat2)
    delta_phi = np.radians(lat2 - lat1)
    delta_lambda = np.radians(lon2 - lon1)

    a = (
        np.sin(delta_phi / 2.0) ** 2
        + np.cos(phi1) * np.cos(phi2) * np.sin(delta_lambda / 2.0) ** 2
    )

    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))

    return R * c


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

    location_node = GeoLocator("location_service")

    try:
        while rclpy.utilities.ok():
            rclpy.spin(location_node)
    except KeyboardInterrupt:
        location_node.get_logger().warn(f"KeyboardInterrupt triggered.")
    finally:
        location_node.destroy_node()
        rclpy.utilities.try_shutdown()
