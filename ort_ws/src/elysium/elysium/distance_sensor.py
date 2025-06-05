import rclpy
import rclpy.logging
from rclpy.node import Node 
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy
import smbus3 as smbus

from std_msgs.msg import Float32
import threading
import elysium.hardware.adafruit_vl53l4cd as tof


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


class DistanceNode(Node):
    def __init__(self, node_name, topic_name, i2c_addr):
        super().__init__(node_name)

        msg_type = Float32
        self.distance_publisher = self.create_publisher(msg_type=msg_type, topic=topic_name, qos_profile=QoS)
        
        refresh_period = 200e-3  # 200ms data retrieval rate
        self.poll_data = self.create_timer(refresh_period, self.get_data, autostart=False)

        self.bus = smbus.SMBus("/dev/i2c-1")
        self.i2c_addr = i2c_addr
        self.sensor = tof.VL53L4CD(self.bus, self.i2c_addr)

    def test_i2c(self):
        try:
            tmp = self._read_register(0x010F, 2)
        except Exception:
            self.get_logger().error(f"Node {self.get_name()} I2C address is not accessible.")

    def get_data(self):
        data = self.sensor.distance
        self.distance_publisher.publish(data)
        self.get_logger().info(f"Distance published: {data} cm")

def main(args=None):
    rclpy.init(args=args)

    topic_name_1 = "/distance_sensor/qr_code"
    node_name_1 = "distance_node_qr"
    topic_name_2 = "/distance_sensor/optical_flow"
    node_name_2 = "distance_node_optical_flow"

    _distance_sensor_1 = DistanceNode(node_name_1, topic_name_1, i2c_addr=0x29)
    #_distance_sensor_2 = DistanceNode(topic_name_2)
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(_distance_sensor_1)
    # executor.add_node(_distance_sensor_2)
    
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        while rclpy.ok():
            pass
    except KeyboardInterrupt:
        _distance_sensor_1.get_logger().warn(f"KeyboardInterrupt triggered.")
        # _distance_sensor_2.get_logger().warn(f"KeyboardInterrupt triggered.")
    finally:
        _distance_sensor_1.destroy_node()
        # _distance_sensor_2.destroy_node()
        rclpy.try_shutdown()  
        executor_thread.join()