from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="joy",
                executable="joy_node",
                namespace="",
                name="joy_node",
                parameters=[{"autorepeat_rate": 0.0, "coalesce_interval_ms": 1}],
            ),
            Node(
                package="mission_control",
                executable="base",
                namespace="",
                name="base",
            ),
            Node(
                package="mission_control",
                executable="control_gui",
                namespace="",
                name="gui",
            ),
        ]
    )
