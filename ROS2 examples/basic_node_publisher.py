import rclpy
import rclpy.utilities
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node 
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import Float32

import random

QoS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, # Keep only up to the last 10 samples
    depth=10,  # Queue size of 10
    reliability=ReliabilityPolicy.BEST_EFFORT,  # attempt to deliver samples, 
    # but lose them if the network isn't robust
    durability=DurabilityPolicy.VOLATILE, # no attempt to persist samples. 
    # deadline=
    # lifespan=
    # liveliness=
    # liveliness_lease_duration=

    # refer to QoS ros documentation and 
    # QoSProfile source code for kwargs and what they do
)


class BasicPublisherNode(Node):
    def __init__(self, node_name, topic_name):
        """
        Basic publisher node for demonstation purposess. 
        
        :param node_name: node name
        :type node_name: str
        :param topic_name: topic name
        :type topic_name: str
        """

        super().__init__(node_name)  # init the Node parent class

        msg_type = Float32  # define the topic's data type

        self.publisher_1 = self.create_subscription(msg_type=msg_type, 
                                                       topic=topic_name,  
                                                       qos_profile=QoS)

        period = 1  # period interval for the timer_callback method to be called.

        self.timer = self.create_timer(period, self.timer_callback)

    def timer_callback(self):
        msg = Float32()  # initialise the ROS2 message

        msg.data = random.uniform(0, 1)  # assign its parameters using the dot operator.
        # The standard message formats (of foxy) are here: https://docs.ros2.org/foxy/api/std_msgs/msg/ 
        # The whole included message formats are here: https://docs.ros2.org/foxy/api/

        self.get_logger().info(f"Sending a float32 value of {msg.data:.3f}.")  # log the outgoing value
        
        self.publisher_1.publish(msg)  # publish the message to the topic

def main(args=None):
    rclpy.init(args=args)  # initialisation of the rclpy library

    topic_name = "/topic_example"
    node_name  = "basic_publishing_node"

    basic_node = BasicPublisherNode(node_name, topic_name)  # create the node object

    try:
        rclpy.spin(basic_node)  # start the node's event loop by 'spinning' the node.
    except KeyboardInterrupt:
        basic_node.get_logger().warn(f"KeyboardInterrupt triggered.")  # if you Ctrl+C in terminal it will catch here.
    except ExternalShutdownException as err:
        basic_node.get_logger().warn(f"ExternalShutdownException triggered: {err}.")
    finally:
        basic_node.destroy_node()  # Don't need to do this but it is good to be explicit about destroying a node.
        rclpy.utilities.try_shutdown()  # Shutdown the rclpy thread. 
        # Don't use just 'shutdown' as another node may have already called it. It's better to just try to shutdown. 

