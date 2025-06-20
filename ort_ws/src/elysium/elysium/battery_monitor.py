import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy

import pandas
from pathlib import Path
import busio, board

import elysium.hardware.adafruit_ina260 as _ina260
from elysium.config.sensors import BMS_REFRESH_PERIOD, BMS_DELTA_T, BMS_BATTERY_CAPACITY, BMS_SAVE_PATH, BMS_LOOKUP_TABLE_PATH
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
    
        i2c = busio.I2C(board.SCL, board.SDA)
        self.bms = _ina260.INA260(i2c, address=i2c_addr)

        self.bms.averaging_count = _ina260.AveragingCount.COUNT_4   # averaging out on 4 samples

        self.delta_t = self.create_timer(BMS_DELTA_T, self.get_data)

        self.measured_voltage, self.measured_current, self.measured_power = .0, .0, .0

        
        self.current_capacity = BMS_BATTERY_CAPACITY
        self.total_capacity = BMS_BATTERY_CAPACITY

        # We are going to assume we are entering with a full battery. This needs to be backed up by a voltage lookup table I'll generate by running a few 
        # discharges while tracking the current integrated SOC. 

        # 4.18 max on a 12.6V 1.5A full charge
        # 2.2 min 

        self.lookup_table = self._read_lookup_data()
        self.soc, self.prev_soc = self._read_battery_file()  # state of charge = capacity remaining / total capacity

        self._compare_ocv_soc()  # sanity check the stored SOC values against the 'OCV' values in the lookup table. 
        
        self.lookup = lookup_recording  # if this should be a full-discharge lookup table calibration.

    def _read_battery_file(self, path=BMS_SAVE_PATH):
        if not Path(path).is_file():  # Under the assumption that there isn't a SOC to be had, refer to the lookup table. 
            tmp = self.bms.voltage
            self.soc = self._find_ocv_soc(tmp)
            self._save_battery_file()

        with open(path, "r") as fs:
            data = fs.readline()
            
        soc = float(data)
        return soc, soc
    
    def _save_battery_file(self, path=BMS_SAVE_PATH):
        with open(path, "w") as fs:
            fs.write(f"{self.soc}")

    def _read_lookup_data(self, path=BMS_LOOKUP_TABLE_PATH):
        try:
            dataframe = pandas.read_csv(path, sep=",")
        except FileNotFoundError:
            self.get_logger().warn("OCV Lookup table does not exists, making a new one...")
            
        _row_range = pandas.array(range(0, 1001)) # 0 to 1000, 0 inclusive which is why we use 1001. 
        _soc_values = pandas.array(range(0, 1001)) / 1000

        dataframe.fillna(0)  # fills empty NAN values with 0. 

        if dataframe.shape != (1001, 4):
            file_exists = False
            new_dataframe = pandas.DataFrame(index=_row_range, columns=["soc", "charge", "current", "ocv"])
            dataframe.fillna(0)  # fills empty NAN values with 0. 
            new_dataframe.iloc[:, 0] = _soc_values
            dataframe = new_dataframe
        
        self.get_logger().info(f"{dataframe}")
        return dataframe

    def _save_lookup_data(self, path=BMS_LOOKUP_TABLE_PATH):
        
        # Make sure this path is valid!

        if not Path(path).is_file():
            try:
                with open(path, "w") as fs:
                    ...
            except OSError:
                self.get_logger().error(f"[{self.get_name()}] - OSError: probably given a bad path for the ocv_lookup.csv file.")

        self.lookup_table.to_csv(path, sep=",", na_rep=0)
        self.prev_soc = self.soc

        self._save_battery_file()

    def _compare_ocv_soc(self):
        experimental_ocv_value = self.lookup_table.iloc[self.soc, 3]
        tmp_voltage = self.bms.voltage
        if tmp_voltage / experimental_ocv_value >= 0.1:
            self.get_logger().warn(f"[{self.get_name()}] The expected OCV value is more than 10% different to what we are expecting of the battery voltage, rebasing SOC.")
            self.soc = self._find_ocv_soc(tmp_voltage)

    def _find_ocv_soc(self, ocv):
        for i in range(1, 1000):
            if self.lookup_table.iloc[i, 3] > ocv and self.lookup_table.iloc[i+1, 3] < ocv:
                return self.lookup_table.iloc[i, 3]
                    

    def get_data(self):
        self.measured_voltage = self.bms.voltage  # V
        self.measured_current = self.bms.current  # mA 
        self.measured_power = self.bms.power # mW

        charge_expended = self.measured_current * 1e-3 * BMS_DELTA_T  # mW * 0.0001 * dt 
        self.current_capacity -= charge_expended
        self.soc = round(self.current_capacity / self.total_capacity, ndigits=3)
        
        if self.lookup and (self.prev_soc - self.soc) >= 0.001:
            self.lookup_table.iloc[int(round(1000*self.soc)), 1:] = [charge_expended, self.measured_current, self.measured_voltage]
            self._save_lookup_data()

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

