import rclpy
from rclpy.node import Node
import rclpy.utilities

import smbus3 as smbus
import gpiozero as gpio
import lgpio
from gpiozero.pins.lgpio import LGPIOFactory

from std_msgs.msg import Float32
import threading
import elysium.hardware.adafruit_vl53l4cx as tof
from elysium.config.sensors import DISTANCE_SENSOR_REFRESH_PERIOD, tofQoS


class DistanceNode(Node):
    def __init__(self, node_name, topic_name, i2c_addr, sleep_node):
        """Time of Flight Node using the VL53L4CX. 
        Measures the distance off the sensor and returns the value.

        :param node_name: node name
        :type node_name: str
        :param topic_name: topic name
        :type topic_name: str
        :param i2c_addr: i2c address on the i2c bus
        :type i2c_addr: hexadecimal, optional
        :param sleep_node: sleep node for Rates
        :type sleep_node: Node
        """

        super().__init__(node_name)

        self.sleep_node = sleep_node

        msg_type = Float32
        self.distance_publisher = self.create_publisher(
            msg_type=msg_type, topic=topic_name, qos_profile=tofQoS
        )

        refresh_period = DISTANCE_SENSOR_REFRESH_PERIOD  # 200ms data retrieval rate
        self.poll_data = self.create_timer(
            refresh_period, self.get_data, autostart=True
        )

        try:
            self.bus = smbus.SMBus("/dev/i2c-1")
            self.i2c_addr = i2c_addr
            self.sensor = tof.VL53L4CX(self.bus, self.i2c_addr, sleep_node=sleep_node)

        except Exception:
            self.get_logger().error(
                f"[{self.get_name()}] Could not open i2c bus/initialise TOF. Falling back onto the original firmware library."
            )
            import elysium.hardware.adafruit_vl53l4cd as tof_fallback

            self.sensor = tof_fallback.VL53L4CD(address=i2c_addr)

    def test_i2c(self):
        try:
            tmp = self.sensor._read_register(0x010F, 2)
        except OSError:
            self.get_logger().error(
                f"Node {self.get_name()} I2C address is not accessible."
            )

    def get_data(self):
        """Get the data from the ToF sensors.
        """
        
        self.sensor.start_sensor()  # we can't have the TOF sensor initialise fully in the __init__ because the sleep node hasn't spun up yet (it uses rate.sleep)

        try:
            self.sensor.start_ranging()  # so we start the sensor here proper. It only actually inits once
        except OSError as err:
            self.get_logger().warn(
                f"[{self.get_name()}] OSError, probably due to the TOF updating internal addresses. Error: {err}"
            )
            _wait(0.2)

        if self.sensor.data_ready:
            self.sensor.clear_interrupt()

        msg = Float32()
        msg.data = self.sensor.distance
        self.distance_publisher.publish(msg)

    def _wait(self, seconds):
        rate = self.sleep_node.create_rate(1/seconds)
        rate.sleep()
        self.sleep_node.destroy_rate(rate)

def __patched_init(self, chip=None):
    gpio.pins.lgpio.LGPIOFactory.__bases__[0].__init__(self)
    chip = 0
    self._handle = lgpio.gpiochip_open(chip)
    self._chip = chip
    self.pin_class = gpio.pins.lgpio.LGPIOPin


def main(args=None):
    rclpy.init(args=args)

    topic_name_1 = "/distance_sensor/qr_code"
    node_name_1 = "distance_node_qr"
    topic_name_2 = "/distance_sensor/optical_flow"
    node_name_2 = "distance_node_optical_flow"

    sleep_node = rclpy.create_node("dis_sleep_node")

    gpio.pins.lgpio.LGPIOFactory.__init__ = __patched_init
    factory = LGPIOFactory()

    xshut_pin = gpio.DigitalOutputDevice(17, pin_factory=factory)
    xshut_pin.off()

    green_led = gpio.DigitalOutputDevice(26, pin_factory=factory)
    green_led.on()

    import time

    _distance_sensor_1 = DistanceNode(
        node_name_1, topic_name_1, i2c_addr=0x29, sleep_node=sleep_node
    )
    # we accept the cursed for what it is. It's 100ms each on startup only anyway it's fineeee.
    time.sleep(0.3) 

    _distance_sensor_1.sensor.set_address(0x2A)
    time.sleep(0.3)
    xshut_pin.on()

    _distance_sensor_2 = DistanceNode(
        node_name_2, topic_name_2, i2c_addr=0x29, sleep_node=sleep_node
    )

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(_distance_sensor_1)
    executor.add_node(_distance_sensor_2)
    executor.add_node(sleep_node)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        while rclpy.utilities.ok():
            pass
    except KeyboardInterrupt:
        _distance_sensor_1.get_logger().warn(f"KeyboardInterrupt triggered.")
        _distance_sensor_2.get_logger().warn(f"KeyboardInterrupt triggered.")
    finally:
        _distance_sensor_1.destroy_node()
        _distance_sensor_2.destroy_node()
        rclpy.utilities.try_shutdown()
        executor_thread.join()
