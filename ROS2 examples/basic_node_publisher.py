import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy
import rclpy.utilities
from std_msgs.msg import Float32


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


class BasicNode(Node):
    def __init__(self, node_name, topic_name):
        """
        Basic node for demonstation purposess. 
        
        :param node_name: node name
        :type node_name: str
        :param topic_name: topic name
        :type topic_name: str
        :param factory: gpiozero pin factory for the pin 
        :type factory: gpiozero pin factory
        """

        super().__init__(node_name)

        msg_type = Float32
        self.subscriber_1 = self.create_subscription(msg_type=msg_type, 
                                                       topic=topic_name, 
                                                       callback=self.callback, 
                                                       qos_profile=QoS)
    
    def callback(self, msg):
        data = msg.value
        self.get_logger().info(f"Received a float32 value of {data:.3f}.")
        

def main(args=None):
    rclpy.init(args=args)  # initialisation of the rclpy library

    topic_name = "/topic_example"
    node_name  = "basic_node"

    basic_node = BasicNode(node_name, topic_name)  # create the node object

    try:
        rclpy.spin(basic_node)  # start the node's event loop by 'spinning' the node.
    except KeyboardInterrupt:
        basic_node.get_logger().warn(f"KeyboardInterrupt triggered.")  # if you Ctrl+C in terminal it will catch here.
    finally:
        basic_node.destroy_node()  # Don't need to do this but it is good to be explicit about destroying a node.
        rclpy.utilities.try_shutdown()  # Shutdown the rclpy thread. 
        # Don't use just 'shutdown' as another node may have already called it. It's better to just try to shutdown. 

