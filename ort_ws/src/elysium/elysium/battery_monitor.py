import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy

import pandas
from pathlib import Path
import busio, board

import elysium.hardware.adafruit_ina260 as _ina260
from elysium.config.sensors import BMS_REFRESH_PERIOD, BMS_DELTA_T, BMS_BATTERY_CAPACITY, BMS_LOOKUP_TABLE_PATH
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


# BMS CAPACITY LOGIC DERIVED HERE: https://www.youtube.com/watch?v=rOwcxFErcvQ


class BatteryMonitorNode(Node):
    def __init__(self, node_name, topic_name, i2c_addr=0x40, lookup_recording=True):
        super().__init__(node_name)

        msg_type = BatteryInfo
        self.bms_publisher = self.create_publisher(msg_type=msg_type, topic=topic_name, qos_profile=QoS)
        self.publisher_timer = self.create_timer(BMS_REFRESH_PERIOD, self.send_data)
        self.save_timer = self.create_timer(10, self._save_lookup_data)
    
        i2c = busio.I2C(board.SCL, board.SDA)
        self.bms = _ina260.INA260(i2c, address=i2c_addr)

        self.bms.averaging_count = _ina260.AveragingCount.COUNT_4   # averaging out on 4 samples

        self.delta_t = self.create_timer(BMS_DELTA_T, self.get_data)

        self.measured_voltage, self.measured_current, self.measured_power = .0, .0, .0

        self.soc = 1  # state of charge = capacity remaining / total capacity
        self.current_capacity = BMS_BATTERY_CAPACITY
        self.total_capacity = BMS_BATTERY_CAPACITY

        # We are going to assume we are entering with a full battery. This needs to be backed up by a voltage lookup table I'll generate by running a few 
        # discharges while tracking the current integrated SOC. 

        # 4.18 max on a 12.6V 1.5A full charge
        # 2.2 min 

        self.lookup_table = self._read_lookup_data(BMS_LOOKUP_TABLE_PATH)

        if lookup_recording:
            self.lookup = True

    def _read_lookup_data(self, path):
        dataframe = pandas.read_csv(path, sep=",")
        self.get_logger().info(f"{dataframe}")
        return dataframe

    def _save_lookup_data(self, path=BMS_BATTERY_CAPACITY):
        
        # Make sure this path is valid!

        if not Path(path).is_file():
            try:
                with open(path, "w") as fs:
                    ...
            except OSError:
                self.get_logger().error(f"[{self.get_name()}] - OSError: probably given a bad path for the ocv_lookup.csv file.")

        self.lookup_table.to_csv(path, sep=",", na_rep=0)
        
    def get_data(self):
        self.measured_voltage = self.bms.voltage  # V
        self.measured_current = self.bms.current  # mA 
        self.measured_power = self.bms.power # mW

        charge_expended = self.measured_current * 1e-3 * BMS_DELTA_T  # mW * 0.0001 * dt 
        self.current_capacity -= charge_expended
        self.soc = self.current_capacity / self.total_capacity

        # if self.lookup:
        #     self._save_lookup_data(BMS_LOOKUP_TABLE_PATH)

    def send_data(self):
        msg = BatteryInfo()

        msg.voltage = float(self.measured_voltage)
        msg.current = float(self.measured_current)
        msg.power = float(self.measured_power)
        msg.soc = float(self.soc)
        msg.remaining_capacity = float(self.current_capacity)
        msg.total_capacity = float(self.total_capacity)
        
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

