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
    def __init__(self, node_name, topic_name, factory):
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
        self.led_pin = 12
        self.pin = gpio.DigitalOutputDevice(self.led_pin, initial_value=False, pin_factory=factory)  # active low to turn off ToF
    
    def _led_callback(self, msg):
        on = msg.data  # This always will be a float since ROS2 typechecks (and complains a lot if it isn't) it before sending data
        if on:
            self.pin.on()
        elif not on:
            self.pin.off()

def __patched_init(self, chip=None):  
    
    # This is necessary because LGPIO assumes the Pi5 uses gpiochip4, but in a shadow kernel
    # update, Team RPi has refactored it so that gpiochip0 is used like the rest of them. The issue is that LGPIO has not caught on
    # and locks you into gpiochip4 even if you pass a param to say its 0. 

    gpio.pins.lgpio.LGPIOFactory.__bases__[0].__init__(self)
    chip = 0
    self._handle = lgpio.gpiochip_open(chip)
    self._chip = chip
    self.pin_class = gpio.pins.lgpio.LGPIOPin
            
def main(args=None):
    rclpy.init(args=args)

    topic_name = "/led"
    node_name  = "led_camera"
    
    gpio.pins.lgpio.LGPIOFactory.__init__ = __patched_init   # setup the XSHUT pin and the green LED pins. 
    factory = LGPIOFactory()
    _led_node = LEDNode(node_name, topic_name, factory)

    try:
        while rclpy.utilities.ok():
            rclpy.spin(_led_node)
    except KeyboardInterrupt:
        _led_node.get_logger().warn(f"KeyboardInterrupt triggered.")
    finally:
        _led_node.destroy_node()
        rclpy.utilities.try_shutdown()  

