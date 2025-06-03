
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="elysium",
                executable="teleop",
                namespace="",
                name="teleop",
            ),
            Node(
                package="elysium",
                executable="imu_sensor",
                namespace="",
                name="imu_sensor",
            ),
            Node(
                package="elysium",
                executable="distance_sensor",
                namespace="",
                name="distance_sensor",
            ),
        ]
    )
