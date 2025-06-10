import rclpy
import rclpy.logging
from rclpy.node import Node 
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy

import gpiozero as gpio
import lgpio
from gpiozero.pins.lgpio import LGPIOFactory

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


class LEDNode(Node):
    def __init__(self, node_name, topic_name):
        super().__init__(node_name)

        msg_type = Float32
        self.led_subscriber = self.create_subscriber(msg_type=msg_type, topic=topic_name, callback=self._led_callback, qos_profile=QoS)
        gpio.pins.lgpio.LGPIOFactory.__init__ = __patched_init
        factory = LGPIOFactory()    
        self.led_pwm_pin = gpio.PWMOutputDevice(pin=12, initial_value=0, frequency=1000, pin_factory=factory)
        


    def _led_callback(self, msg):
        float_val = msg.data

        if float_val < 0 or float_val > 1:
            raise ValueError(f"The LED brightness value must be between 0 and 1.")
        
        self.set_brightness(float_val)
    

    def set_brightness(self, value: float):
        """Set the brightness of the LED board, value between 0 and 1. 
        """
        self.led_pwm_pin.value = value

    def set_frequency(self, value: int):
        """Set the frequency of the LED board, value between 1 and 10kHz. 
        """
        self.led_pwm_pin.frequency = value


def __patched_init(self, chip=None):
    gpio.pins.lgpio.LGPIOFactory.__bases__[0].__init__(self)
    chip = 0
    self._handle = lgpio.gpiochip_open(chip)
    self._chip = chip
    self.pin_class = gpio.pins.lgpio.LGPIOPin

def main(args=None):
    rclpy.init(args=args)

    topic_name = "/led"
    node_name  = "led_camera"
    _led_node = LEDNode(node_name, topic_name)

    try:
        while rclpy.ok():
            rclpy.spin(_led_node)
    except KeyboardInterrupt:
        _led_node.get_logger().warn(f"KeyboardInterrupt triggered.")
    finally:
        _led_node.destroy_node()
        rclpy.try_shutdown()  

