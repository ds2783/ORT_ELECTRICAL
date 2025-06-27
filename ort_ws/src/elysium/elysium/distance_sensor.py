import rclpy
from rclpy.node import Node
import rclpy.utilities

import smbus3 as smbus
import gpiozero as gpio
import lgpio
from gpiozero.pins.lgpio import LGPIOFactory

from std_msgs.msg import Float32

import time
import threading
import elysium.hardware.adafruit_vl53l4cx as tof
from elysium.config.sensors import DISTANCE_SENSOR_REFRESH_PERIOD, tofQoS


class DistanceNode(Node):
    def __init__(self, node_name, topic_name, i2c_bus, i2c_addr, sleep_node):
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
        self.send_data = self.create_timer(
            refresh_period, self.get_data, autostart=True
        )

        # self.poll_data = self.create_

        try:
            self.bus = i2c_bus
            self.i2c_addr = i2c_addr
            self.sensor = tof.VL53L4CX(self.bus, self.i2c_addr, sleep_node=self)

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

    # def start_ranging(self):
    #     """Starts ranging operation."""
    #     # start ranging depending inter-measurement setting
    #     if self.sensor.inter_measurement == 0:
    #         # continuous mode
    #         self.sensor._write_register(tof._VL53L4CX_SYSTEM_START, b"\x21")
    #     else:
    #         # autonomous mode
    #         self.sensor._write_register(tof._VL53L4CX_SYSTEM_START, b"\x40")

    #     # wait for data ready
    #     timed_out = True
    #     for _ in range(1000):
    #         if self.data_ready:
    #             timed_out = False
    #             break
    #         self._wait()
    #     if timed_out:
    #         raise TimeoutError("Time out waiting for data ready.")

    #     self.clear_interrupt()
    #     self._ranging = True

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
            self._wait(0.2)

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

    sleep_node = rclpy.create_node("dist_sleep_node")

    i2c_bus = smbus.SMBus("/dev/i2c-1")

    gpio.pins.lgpio.LGPIOFactory.__init__ = __patched_init   # setup the XSHUT pin and the green LED pins. 
    factory = LGPIOFactory()
    xshut_pin = gpio.DigitalOutputDevice(17, initial_value=True, pin_factory=factory)  # active low to turn off ToF
    green_led = gpio.DigitalOutputDevice(26, pin_factory=factory)
    green_led.on()  # indicate ROS2 is running. 

    try:
        test_tof_1 = tof.VL53L4CX(i2c_bus, i2c_address=0x29, sleep_node=sleep_node)
        test_tof_2 = tof.VL53L4CX(i2c_bus, i2c_address=0x2A, sleep_node=sleep_node)
        both_on = True  # The i2c addresses have already been set properly and are returning correct model id 
        # values. 
    except OSError as err:
        sleep_node.get_logger().info(f"""[distance node] the i2c addresses of the sleep node have not been set yet, 
                                      (to be expected after a reboot) and will be set accordingly.""")
        both_on = False  # They are not both set to the correct addresses, and have to be set accordingly. 

    if both_on:
        _distance_sensor_1 = DistanceNode(
        node_name_1, topic_name_1, i2c_bus, i2c_addr=0x2A, sleep_node=sleep_node
        )
        _distance_sensor_2 = DistanceNode(
        node_name_2, topic_name_2, i2c_bus, i2c_addr=0x29, sleep_node=sleep_node
        )
    else:
        xshut_pin.off()

        _distance_sensor_1 = DistanceNode(
            node_name_1, topic_name_1, i2c_bus, i2c_addr=0x29, sleep_node=sleep_node
        )
        # we accept the cursed for what it is. It's 100ms each on startup only anyway it's fineeee.
        time.sleep(0.3) 

        _distance_sensor_1.sensor.set_address(0x2A)
        time.sleep(0.3)
        xshut_pin.on()

        _distance_sensor_2 = DistanceNode(
            node_name_2, topic_name_2, i2c_bus, i2c_addr=0x29, sleep_node=sleep_node
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
