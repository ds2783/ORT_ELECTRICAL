import rclpy
from rclpy.node import Node
import rclpy.utilities
import rclpy.executors

import smbus3 as smbus
import gpiozero as gpio
import lgpio
from gpiozero.pins.lgpio import LGPIOFactory

from std_msgs.msg import Float32
from ort_interfaces.srv import DistanceData

import time
import elysium.hardware.adafruit_vl53l4cd as tof
from elysium.config.sensors import DISTANCE_SENSOR_START_DELAY, DISTANCE_SENSOR_POLL_PERIOD, DISTANCE_SENSOR_REFRESH_PERIOD, tofQoS


class DistanceNode(Node):
    def __init__(self, 
                 node_name: str, 
                 topic_name: str,
                 i2c_addr: int, 
                 srv: bool = False):
        """Time of Flight Node using the VL53L4CX. 
        Measures the distance off the sensor and returns the value.

        :param node_name: node name
        :type node_name: str
        :param topic_name: topic name
        :type topic_name: str
        :param i2c_addr: i2c address on the i2c bus
        :type i2c_addr: hexadecimal, optional
        :param srv: if the node is a service or not, default False
        :type srv: bool
        """

        super().__init__(node_name)


        if not srv:  # if the node is a service or not. 
            msg_type = Float32
            self.distance_publisher = self.create_publisher(
                msg_type=msg_type, topic=topic_name, qos_profile=tofQoS
            )

            refresh_period = DISTANCE_SENSOR_REFRESH_PERIOD  # 200ms data retrieval rate
            self.send_data = self.create_timer(
                refresh_period, self.get_data, autostart=True
            )

            poll_period = DISTANCE_SENSOR_POLL_PERIOD
            self.poll_data = self.create_timer(poll_period, self._poll_data, autostart=False)

            self.data = .0
        else:
            self.srv = self.create_service(DistanceData, "/elysium/cam/distance_service", self.data_srv_callback)


        try:
            self.i2c_addr = i2c_addr
            self.sensor = tof.VL53L4CD(self.i2c_addr)

        except Exception as err:
            self.get_logger().error(
                f"[{self.get_name()}] Could not open i2c bus/initialise TOF. Error: {err}"
            )
        
        if not srv:
            self.sensor.start_ranging()
            self.poll_data.reset()

    def test_i2c(self):
        try:
            tmp = self.sensor._read_register(0x010F, 2)
        except OSError:
            self.get_logger().error(
                f"Node {self.get_name()} I2C address is not accessible."
            )

    def data_srv_callback(self, request, response):
        self.sensor.start_ranging() 
        # Request isn't considered in this interaction.  
        if self.sensor.data_ready:  # checks if data is ready from the ToF
            response.distance = self.sensor.distance
            response.data_retrieved = True
            self.sensor.clear_interrupt()

        else: 
            response.distance = -1.0  # otherwise sets the dist to -1
            response.data_retrieved = False 
        self.sensor.stop_ranging()

        return response

    def _poll_data(self):
        if self.sensor.data_ready:
            self.data = self.sensor.distance
            self.sensor.clear_interrupt()

    def get_data(self):
        """Get the data from the ToF sensors.
        """
        msg = Float32()
        msg.data = self.data
        self.distance_publisher.publish(msg)


def __patched_init(self, chip=None):
    gpio.pins.lgpio.LGPIOFactory.__bases__[0].__init__(self)
    chip = 0
    self._handle = lgpio.gpiochip_open(chip)
    self._chip = chip
    self.pin_class = gpio.pins.lgpio.LGPIOPin


def main(args=None):
    rclpy.init(args=args)

    logger_node = rclpy.create_node("dist_logging_node")

    topic_name_1 = "/distance_sensor/qr_code"
    node_name_1 = "distance_node_qr"
    topic_name_2 = "/distance_sensor/optical_flow"
    node_name_2 = "distance_node_optical_flow"

    gpio.pins.lgpio.LGPIOFactory.__init__ = __patched_init   # setup the XSHUT pin and the green LED pins. 
    factory = LGPIOFactory()
    xshut_pin = gpio.DigitalOutputDevice(17, initial_value=True, pin_factory=factory)  # active low to turn off ToF
    green_led = gpio.DigitalOutputDevice(26, pin_factory=factory)
    green_led.on()  # indicate ROS2 is running. 

    try:
        test_tof_1 = tof.VL53L4CD(0x29)
        test_tof_2 = tof.VL53L4CD(0x2A)
        del test_tof_1  # delete them after so they don't interfere with the initialisation later 
        del test_tof_2
        both_on = True  # The i2c addresses have already been set properly and are returning correct model id 
        # values. 
    except OSError as err:
        logger_node.get_logger().info(f"""[distance node] the i2c addresses of the sleep node have not been set yet, 
                                      (to be expected after a reboot) and will be set accordingly.""")
        both_on = False  # They are not both set to the correct addresses, and have to be set accordingly. 

    if both_on:
        _distance_sensor_1 = DistanceNode(
        node_name_1, topic_name_1, i2c_addr=0x2A, srv=True
        )
        _distance_sensor_2 = DistanceNode(
        node_name_2, topic_name_2, i2c_addr=0x29 
        )

    else:
        xshut_pin.off()

        _distance_sensor_1 = DistanceNode(
            node_name_1, topic_name_1, i2c_addr=0x29, srv=True)
        # we accept the cursed for what it is. It's 300ms each on startup only anyway it's fineeee.
        time.sleep(0.3) 

        _distance_sensor_1.sensor.set_address(0x2A)
        time.sleep(0.3)
        xshut_pin.on()

        _distance_sensor_2 = DistanceNode(
            node_name_2, topic_name_2, i2c_addr=0x29
            )

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(_distance_sensor_1)
    executor.add_node(_distance_sensor_2)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        _distance_sensor_1.get_logger().warn(f"KeyboardInterrupt triggered.")
        _distance_sensor_2.get_logger().warn(f"KeyboardInterrupt triggered.")
    finally:
        _distance_sensor_1.destroy_node()
        _distance_sensor_2.destroy_node()
        rclpy.utilities.try_shutdown()
