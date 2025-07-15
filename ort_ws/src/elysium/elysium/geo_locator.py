import rclpy
import rclpy.utilities
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Bool, Float32, Header
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
from ort_interfaces.msg import OpticalFlow, GPSStatus, OpticalFlowCalibration
from ort_interfaces.srv import Vec2Pos, CullCalibration

from elysium.config.sensors import (
    DISTANCE_SENSOR_REFRESH_PERIOD,
    OPTICAL_CALIBRATION,
    tofQoS,
)
from elysium.config.network import DIAGNOSTIC_PERIOD
from elysium.config.services import GEO_BACKUP_PERIOD, SUCCESS, FAIL

import numpy as np
import json
from scipy.spatial.transform import Rotation as R
from pyrr import quaternion


# ADD STATE SAVING, from previous position etc...
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
        self.reset_pos_sub_ = self.create_subscription(
            Bool, "/elysium/reset_pos", self.zero_resetCB_, 10
        )

        self.gps_sub_ = self.create_subscription(
            GPSStatus, "/elysium/gps_data", self.gpsCB_, 10
        )
        self.optical_calibration_sub_ = self.create_subscription(
            Float32, "/elysium/ofs_calibration", self.ofs_calCB_, 10
        )
        # ----------------------

        # Publishers -----------
        self.euler_angles_pub_ = self.create_publisher(
            Vector3, "/elysium/euler_angles", 10
        )
        self.odom_pub_ = self.create_publisher(Odometry, "/elysium/odom", 10)

        self.gps_dist_pub_ = self.create_publisher(Float32, "/elysium/gps_dist", 10)

        self.optical_factor_pub_ = self.create_publisher(
            OpticalFlowCalibration, "/elysium/optical_factor", 10
        )
        # ----------------------

        self.position_service_ = self.create_service(
            Vec2Pos, "/elysium/srv/position", self.positionCB_
        )

        self.cull_optical_flow_calibration_service_ = self.create_service(
            CullCalibration, "/elysium/srv/cull_calibration", self.cullCB_
        )

        # Timers ----------------
        self.diagnostic_timer_ = self.create_timer(DIAGNOSTIC_PERIOD, self.publish_)
        self.backup_timer = self.create_timer(GEO_BACKUP_PERIOD, self.backup_)
        # ------------------------

        self.dx = 0
        self.dy = 0
        # avoids division by zero error
        self.dt = 0.0001

        self.distance_sensor_dt_ = DISTANCE_SENSOR_REFRESH_PERIOD

        self.z_pos = 0.0
        self.z_prev_ = 0.0

        # Calibration
        self.calibration_y_move = 0
        self.calibration_x_move = 0

        # GPS Vars
        self.lat = None
        self.long = None

        # IMU Vars
        self.euler_angles = Vector3()
        self.rotation_ = Quaternion()
        self.quat_ = np.array([0.0, 0.0, 0.0, 1.0])
        self.raw_quat_ = np.array([0.0, 0.0, 0.0, 1.0])

        # raw signifies that the OFS sensor has not been calibrated,
        # this allows retrodiction of position at base station.
        try:
            with open("geo_data.json", "r") as fp:
                recovery = json.load(fp)

            self.raw_x_pos = recovery["raw-x-pos"]
            self.raw_y_pos = recovery["raw-y-pos"]

            self.optical_calibration_points = np.array(
                recovery["optical-calibration-points"]
            )
            if self.optical_calibration_points.size >= 1:
                self.optical_factor = np.mean(self.optical_calibration_points)
            else:
                self.optical_factor = 1.0
            self.optical_calibration_points = list(self.optical_calibration_points)

            self.start_lat = recovery["start-lat"]
            self.start_lon = recovery["start-lon"]

            self.inverse_offset = np.array(recovery["quat-offset"])

            self.get_logger().info("This data was recovered: " + str(recovery))
        except:
            self.get_logger().info("No JSON to recover from.")

            self.raw_x_pos = 0.0
            self.raw_y_pos = 0.0

            self.optical_calibration_points = []
            self.optical_factor = OPTICAL_CALIBRATION

            self.start_lat = None
            self.start_lon = None

            self.inverse_offset = np.array([0.0, 0.0, 0.0, 1.0])

    def backup_(self):
        # Periodically sync base with Geo_locator
        calibration = OpticalFlowCalibration()
        calibration.samples = np.array(self.optical_calibration_points)
        calibration.optical_factor = float(self.optical_factor)
        calibration.num_samples = len(self.optical_calibration_points)
        self.optical_factor_pub_.publish(calibration)

        backup = {
            "raw-x-pos": self.raw_x_pos,
            "raw-y-pos": self.raw_y_pos,
            "optical-calibration-points": np.array(self.optical_calibration_points),
            "start-lat": self.start_lat,
            "start-lon": self.start_lon,
            "quat-offset": np.array(self.inverse_offset),
        }
        with open("geo_data.json", "w") as fp:
            json.dump(backup, fp, cls=NPEncoder)

    def zero_resetCB_(self, msg: Bool):
        self.inverse_offset = quaternion.inverse(self.raw_quat_)
        if msg.data == True:
            self.raw_x_pos = 0.0
            self.raw_y_pos = 0.0

            self.start_lat = self.lat
            self.start_lon = self.long
            self.get_logger().info(
                f"Captured start coordinates: {self.start_lat}, {self.start_lon}"
            )

    def positionCB_(self, req, response):
        response.x = self.calibration_x_move
        response.y = self.calibration_y_move
        return response

    def cullCB_(self, req, response):
        if req.clear_all:
            self.optical_calibration_points = []
            self.optical_factor = 1.0
            response.ret_code = SUCCESS
        else:
            self.optical_calibration_points = list(self.optical_calibration_points)
            try:
                self.optical_calibration_points.pop(req.index)
                response.ret_code = SUCCESS
            except IndexError:
                response.ret_code = FAIL
            self.optical_factor = np.mean(self.optical_calibration_points)

            self.publish_ofs_calibration()

    def ofs_calCB_(self, msg: Float32):
        # Forcing back to list incase type mutation
        self.optical_calibration_points = list(self.optical_calibration_points)
        self.optical_calibration_points.append(msg.data)
        self.optical_factor = np.mean(self.optical_calibration_points)
        self.get_logger().info(
            "Optical calibration factor successfully set to: "
            + str(self.optical_factor)
        )

        self.publish_ofs_calibration()

    def publish_ofs_calibration(self):
        calibration = OpticalFlowCalibration()
        calibration.samples = np.array(self.optical_calibration_points)
        calibration.optical_factor = float(self.optical_factor)
        calibration.num_samples = len(self.optical_calibration_points)
        self.optical_factor_pub_.publish(calibration)

    def tofCB_(self, msg: Float32):
        self.z_prev_ = self.z_pos
        self.z_axis = msg.data

    def imuCB_(self, msg: Quaternion):
        self.rotation_ = msg
        # x,y,z,w
        rec_quat = np.array(
            [self.rotation_.w, self.rotation_.x, self.rotation_.y, self.rotation_.z]
        )
        self.raw_quat_ = rec_quat
        self.quat_ = quaternion.cross(rec_quat, self.inverse_offset)

    def opticalCB_(self, msg: OpticalFlow):
        euler = R.from_quat(
            [self.quat_[1], self.quat_[2], self.quat_[3], self.quat_[0]]
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
        self.raw_x_pos += self.dx
        self.raw_y_pos += self.dy
        self.dt = msg.dt

    def gpsCB_(self, msg):
        # conditions to define a reasonable fix
        if msg.fix_quality > 0 and msg.latitude and msg.longitude and msg.hdop < 5:
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
                        # using raw optical flow values allow retrodiction
                        # of true distances
                        x=float(self.raw_x_pos),
                        y=float(self.raw_y_pos),
                        z=float(self.z_pos),
                    )
                )
            ),
            twist=TwistWithCovariance(
                twist=Twist(
                    linear=Vector3(
                        x=float(self.dx * self.optical_factor / self.dt),
                        y=float(self.dy * self.optical_factor / self.dt),
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


class NPEncoder(json.JSONEncoder):
    def default(self, o):
        try:
            return o.tolist()  # works with any object that has .tolist() method
        except AttributeError:
            pass
        # Let the base class default method raise the TypeError
        return json.JSONEncoder.default(self, o)


def main(args=None):
    rclpy.init(args=args)

    location_node = GeoLocator("location_service")

    try:
        rclpy.spin(location_node)
    except KeyboardInterrupt:
        location_node.get_logger().warn(f"KeyboardInterrupt triggered.")
    finally:
        location_node.destroy_node()
        rclpy.utilities.try_shutdown()
