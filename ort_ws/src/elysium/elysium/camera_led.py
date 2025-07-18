import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy
import rclpy.utilities

import gpiozero as gpio
import lgpio
from gpiozero.pins.lgpio import LGPIOFactory
import RPI.GPIO as GPIO

from std_msgs.msg import Bool


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


class LEDNode(Node):
    def __init__(self, node_name, topic_name):
        """
        Node that handles the infrared LED panel for the upper camera on the rover. 
        
        :param node_name: node name
        :type node_name: str
        :param topic_name: topic name
        :type topic_name: str
        :param factory: gpiozero pin factory for the pin 
        :type factory: gpiozero pin factory
        """

        super().__init__(node_name)

        msg_type = Bool
        self.led_subscriber = self.create_subscription(msg_type=msg_type, 
                                                       topic=topic_name, 
                                                       callback=self._led_callback, 
                                                       qos_profile=QoS)
        GPIO.setmode(GPIO.BOARD)
        self.led_pin = 12
    
        GPIO.output(self.led_pin, GPIO.LOW)

    def _led_callback(self, msg):
        on = msg.data  # This always will be a float since ROS2 typechecks (and complains a lot if it isn't) it before sending data
        if on:
            GPIO.output(self.led_pin, GPIO.HIGH)
        elif not on:
            GPIO.output(self.led_pin, GPIO.LOW)
            
def main(args=None):
    rclpy.init(args=args)

    topic_name = "/led"
    node_name  = "led_camera"
    
    _led_node = LEDNode(node_name, topic_name)

    try:
        while rclpy.utilities.ok():
            rclpy.spin(_led_node)
    except KeyboardInterrupt:
        _led_node.get_logger().warn(f"KeyboardInterrupt triggered.")
    finally:
        _led_node.destroy_node()
        rclpy.utilities.try_shutdown()  

