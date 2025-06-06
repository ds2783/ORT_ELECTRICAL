import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

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

from elysium.config.sensors import DISTANCE_SENSOR_REFRESH_PERIOD

import threading
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
        # ----------------------

        # Publishers -----------
        self.euler_angles_pub_ = self.create_publisher(
            Vector3, "/imu/euler_angles", qos_profile_sensor_data
        )
        self.odom_pub_ = self.create_publisher(
            Odometry, "/elysium/odom", qos_profile_sensor_data
        )
        # ----------------------

        self.rotation_ = Quaternion()
        self.distance_sensor_dt_ = DISTANCE_SENSOR_REFRESH_PERIOD

        # Cartesian Displacement - Initiale Values
        # TO DO:
        # Add csv file to load previous displacements incase of crash
        self.z_prev_ = 0
        self.z_pos = 0
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
        ).as_euler("zyx", degrees=True)
        yaw, pitch, roll = euler

        euler_angles = Vector3(x=yaw, y=pitch, z=roll)
        self.euler_angles_pub_.publish(euler_angles)

        # rotate the dx and dy increments around the yaw
        rotation = np.array([np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)])
        increment = np.array([msg.dx, msg.dy])

        rotated_increment = np.multiply(rotation, increment)
        dx = rotated_increment[0]
        dy = rotated_increment[1]
        self.x_pos += dx
        self.y_pos += dy

        odom_msg = Odometry(
            header=Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id=self.get_parameter("parent_frame").value,
            ),
            child_frame_id=self.get_parameter("child_frame").value,
            pose=PoseWithCovariance(
                pose=Pose(position=Point(x=self.x_pos, y=self.y_pos, z=self.z_pos))
            ),
            twist=TwistWithCovariance(
                twist=Twist(
                    linear=Vector3(
                        x=dx / msg.dt,
                        y=dy / msg.dt,
                        z=(self.z_pos - self.z_prev_) / self.distance_sensor_dt_,
                    )
                )
            ),
        )

        self.odom_pub_.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)

    location_node = GeoLocator("location_service")
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(location_node)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)

    try:
        executor_thread.start()
    except KeyboardInterrupt:
        location_node.get_logger().warn(f"KeyboardInterrupt triggered.")
    finally:
        location_node.destroy_node()
        rclpy.shutdown()
        executor_thread.join()

