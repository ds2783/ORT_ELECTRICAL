from sys import executable
from rclpy.node import Node
from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import RegisterEventHandler
from launch.events.process import ProcessStarted
from launch.event_handlers.on_process_start import OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():
    node1 = Node(package="mission_control", executable="control_gui")
    node2 = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[{"autorepeat_rate": 0.0, "coalesce_interval_ms": 5, "deadzone": 0.1}],
    )
    node3 = Node(package="mission_control", executable="base")
    node4 = Node(package="mission_control", executable="connection_server")

    already_started_nodes = set()

    def start_next_node(event: ProcessStarted, context: LaunchContext):
        print(f"node {event.process_name} started.")
        already_started_nodes.update([event.process_name])
        if len(already_started_nodes) == 3:
            print(f"all required nodes are up, time to start node1")
            return node1

    return LaunchDescription(
        [
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=node2, on_start=start_next_node
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=node3, on_start=start_next_node
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=node4, on_start=start_next_node
                )
            ),
            node2,
            node3,
            node4,
        ]
    )

if __name__ == "__main__":
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
