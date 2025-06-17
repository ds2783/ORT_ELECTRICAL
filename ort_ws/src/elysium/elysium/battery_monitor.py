import rclpy
import rclpy.logging
from rclpy.node import Node 
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy

import elysium.hardware.adafruit_ina260 as _ina260
from elysium.config.sensors import BMS_REFRESH_PERIOD

import busio, board

from ort_interfaces.msg import BatteryInfo


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

)


class BatteryMonitorNode(Node):
    def __init__(self, node_name, topic_name, i2c_addr=0x40):
        super().__init__(node_name)

        msg_type = BatteryInfo
        self.bms_publisher = self.create_publisher(msg_type=msg_type, topic=topic_name, qos_profile=QoS)
        self.update_timer = self.create_timer(BMS_REFRESH_PERIOD, self.send_data)
    
        i2c = busio.I2C(board.SCL, board.SDA)
        self.bms = _ina260.INA260(i2c, address=i2c_addr)

        self.bms.averaging_count = _ina260.AveragingCount.COUNT_4   # averaging out on 4 samples


    def send_data(self):
        msg = BatteryInfo()

        msg.voltage = self.bms.voltage
        msg.current = self.bms.current
        msg.power = self.bms.power
        
        self.bms_publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)

    topic_name = "/battery_monitor"
    node_name  = "battery_monitor"
    
    _bms_node = BatteryMonitorNode(node_name, topic_name, i2c_addr=0x44)

    try:
        while rclpy.ok():
            rclpy.spin(_bms_node)
    except KeyboardInterrupt:
        _bms_node.get_logger().warn(f"KeyboardInterrupt triggered.")
    finally:
        _bms_node.destroy_node()
        rclpy.try_shutdown()  

