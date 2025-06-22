import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy

import pandas
import pandas.errors
from pathlib import Path
import busio, board

import elysium.hardware.adafruit_ina260 as _ina260
from elysium.config.sensors import (BMS_REFRESH_PERIOD, 
                                    BMS_DELTA_T, 
                                    BMS_UNDERVOLT_WARN, 
                                    BMS_UNDERVOLT_SHUTDOWN,
                                    BMS_BATTERY_CAPACITY, 
                                    BMS_SAVE_PATH, 
                                    BMS_LOOKUP_TABLE_PATH)
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

# USING A COMBINATION OF SIMPLISTIC COLOUMB COUNTING AND AN 'OCV' LOOKUP TABLE. 
# SUBSEQUENT YEARS SHOULD LOOK AT KALMANN FILTERS FOR A MORE OPTIMISED ESTIMATION OF THE SOC OF THE BATTERY.  


class BatteryMonitorNode(Node):
    def __init__(self, node_name, topic_name, i2c_addr=0x40, lookup_recording=False):
        """Battery Monitoring Node

        :param node_name: node name
        :type node_name: str
        :param topic_name: topic name
        :type topic_name: str
        :param i2c_addr: i2c address on the i2c bus, defaults to 0x40
        :type i2c_addr: hexadecimal, optional
        :param lookup_recording: set to True if you want to create a new OCV lookup table, defaults to False
        :type lookup_recording: bool, optional
        """
        super().__init__(node_name)

        msg_type = BatteryInfo
        self.bms_publisher = self.create_publisher(msg_type=msg_type, topic=topic_name, qos_profile=QoS)
        self.publisher_timer = self.create_timer(BMS_REFRESH_PERIOD, self.send_data)
    
        i2c = busio.I2C(board.SCL, board.SDA)
        self.bms = _ina260.INA260(i2c, address=i2c_addr)

        self.bms.averaging_count = _ina260.AveragingCount.COUNT_4   # averaging out on 4 samples
        self.bms.voltage_conversion_time = _ina260.ConversionTime.TIME_2_116_ms

        self.delta_t = self.create_timer(BMS_DELTA_T, self.get_data)

        self.measured_voltage, self.measured_current, self.measured_power = .0, .0, .0

        
        self.current_capacity = BMS_BATTERY_CAPACITY
        self.total_capacity = BMS_BATTERY_CAPACITY

        # We are going to assume we are entering with a full battery. This needs to be backed up by a voltage lookup table I'll generate by running a few 
        # discharges while tracking the current integrated SOC. 

        # 4.18 max on a 12.6V 1.5A full charge
        # 2.2 min
        
        self.lookup = lookup_recording  # if this should be a full-discharge lookup table calibration.
 
        self.lookup_table = self._read_lookup_data()
        self.soc, self.prev_soc = self._read_battery_file()  # state of charge = capacity remaining / total capacity

        self.current_capacity = self.soc * self.total_capacity

        if not self.lookup:
            self._compare_ocv_soc()  # sanity check the stored SOC values against the 'OCV' values in the lookup table. 
        

    def _read_battery_file(self, path=BMS_SAVE_PATH):
        """If the BMS_SAVE_PATH file exists, read and typecast into float. 
        
        If it doesn't, and the lookup table is not complete, just assume we are recording a new lookup table and set SOC to full.
        
        Otherwise, use the lookup table to return a SOC value.  

        :param path: path, found in the config.sensors file, defaults to BMS_SAVE_PATH
        :type path: PathLike, optional
        :return: SOC value 
        :rtype: float
        """
        if Path(path).is_file():  # Under the assumption that there isn't a SOC to be had, refer to the lookup table. 
            with open(path, "r") as fs:
                data = fs.readline()  # for now we are just keeping the data as a string
            self.soc = float(data)  # typecast the chars into a float
        
        else:
            tmp = self.bms.voltage
            
            if (self.lookup_table == 0).sum().sum() > 2000:  # if the number of zeroes is above 2000, assume that 
                # the table has NOT been populated yet. 
                return 1, 1
            else:
                self.soc = self._find_ocv_soc(tmp)  # Get the OCV lookup value
            
        soc = float(self.soc)
        return soc, soc
    
    def _save_battery_file(self, path=BMS_SAVE_PATH):
        """Save the SOC as a string to the BMS_SAVE_PATH

        :param path: path, found in the config.sensors file, defaults to BMS_SAVE_PATH
        :type path: PathLike, optional
        """

        with open(path, "w") as fs:
            fs.write(f"{self.soc}")

    def _read_lookup_data(self, path=BMS_LOOKUP_TABLE_PATH):
        """Read the lookup table. Generates an empty formatted dataframe if the file does not exists. 

        Should continue recording if there is more than ((1001*4 - 2500) / 4) = ~375 entries in the table. 

        :param path: path, found in the config.sensors file, defaults to BMS_LOOKUP_TABLE_PATH
        :type path: PathLike, optional
        :return: Pandas dataframe of the lookup table
        :rtype: Pandas.DataFrame
        """

        dataframe = pandas.DataFrame(dtype=pandas.Float64Dtype())

        if Path(path).is_file(): 
            try:
                dataframe = pandas.read_csv(path, sep=",", dtype=pandas.Float64Dtype())
            except pandas.errors.EmptyDataError:
                pass
        
        dataframe = dataframe.fillna(0)  # fills empty NAN values with 0. 

        cond_1 = dataframe.shape != (1001, 4)  # if the dataframe is malformed
        cond_2 = (dataframe == 0).sum().sum() > 2500  # if there is more than 625 entries empty
        cond_3 = self.lookup  # if the lookup bool is true

        if cond_1 or (cond_2 and cond_3):  # create a new dataframe if there doesn't exist one. 
            _row_range = pandas.array(range(0, 1001)) # 0 to 1000, 0 inclusive which is why we use 1001. 
            _soc_values = pandas.array(range(0, 1001)) / 1000
            new_dataframe = pandas.DataFrame(index=_row_range, columns=["soc", "charge", "current", "ocv"], dtype=pandas.Float64Dtype())
            new_dataframe = new_dataframe.fillna(0)  # fills empty NAN values with 0. 
            new_dataframe.iloc[:, 0] = _soc_values
            dataframe = new_dataframe
        
        return dataframe

    def _save_lookup_data(self, path=BMS_LOOKUP_TABLE_PATH):
        """Save the lookup table.

        :param path: path, found in the config.sensors file, defaults to BMS_LOOKUP_TABLE_PATH
        :type path: PathLike, optional
        """
        # Make sure this path is valid!

        if not Path(path).is_file():
            self._create_blank_file(path)

        self.lookup_table.to_csv(path, sep=",", na_rep=0)
        self.prev_soc = self.soc

        self._save_battery_file()

    def _create_blank_file(self, path):
        """Create a blank file.

        :param path: path
        :type path: PathLike
        """

        try:
            with open(path, "w") as fs:
                ...
        except OSError:
            self.get_logger().error(f"[{self.get_name()}] - OSError: probably given a bad path for the file ({path}).")

    def _compare_ocv_soc(self, deviation=0.1):
        """Compare the OCV lookup value with the immediate measured voltage. 
        
        If there is more than a 10% deviation (default value), assume we have drifted too far and set the SOC to the OCV lookup value.
        
        :param deviation: voltage deviation, defaults to 0.1
        :type deviation: float, optional
        """

        experimental_ocv_value = self.lookup_table.iloc[int(round(1000*self.soc)), 3]
        tmp_voltage = self.bms.voltage

        if tmp_voltage / experimental_ocv_value >= deviation:
            self.get_logger().warn(f"[{self.get_name()}] The expected OCV value is more than 10% different to what we are expecting of the battery voltage, rebasing SOC.")
            self.soc = self._find_ocv_soc(tmp_voltage)
            self.current_capacity = self.soc * self.total_capacity

    def _find_ocv_soc(self, ocv: float):
        """Find the SOC from the OCV table voltages, needs to iterate through the table from the smallest value upwards. 

        :param ocv: ocv voltage
        :type ocv: float
        :return: soc
        :rtype: float
        """
        for i in range(1, 1000):
            if self.lookup_table.iloc[i, 3] > ocv and self.lookup_table.iloc[i+1, 3] < ocv:
                return self.lookup_table.iloc[i, 0]

    def _shutdown(self, grace_time=1):
        """Shutdown the system due to critically low voltages. The grace time is measured in minutes. 

        :param grace_time: shutdown grace time in minutes, defaults to 1
        :type grace_time: int, optional
        """

        import os
        os.system(f"shutdown --halt +{grace_time}")
    
        for _ in range(3):
            self.get_logger().warn(f"Queued shutdown for {grace_time} minute(s).")


    def get_data(self):
        """Gets the data from the INA260 and stores it in members of the node. 

        Calculates the remaining SOC using fairly precise couloumb counting. 

        If the difference between the current SOC and previous SOC is larger or equal to 0.1%, 
        record the new values in the lookup table if enabled.

        If the voltage is less than the BMS_UNDERVOLT_WARN threshold, warn the user to charge the batteries.

        If the voltage is less than the BMS_UNDERVOLT_SHUTDOWN threshold, shut down the rover to preserve onboard data. 
        """

        self.measured_voltage = self.bms.voltage  # V
        self.measured_current = self.bms.current  # mA 
        self.measured_power = self.bms.power # mW

        charge_expended = self.measured_current * 1e-3 * BMS_DELTA_T  # mA * 0.0001 * dt 
        self.current_capacity -= 0.27778 * charge_expended # This is a conversion from couloumbs to mAh 
        self.soc = round(self.current_capacity / self.total_capacity, ndigits=3)
        
        if self.lookup and abs(self.prev_soc - self.soc) >= 0.001:  # if the soc value has dropped 0.1%, save the data to the lookup table. 
            self.lookup_table.iloc[int(round(1000*self.soc)), 1:] = [charge_expended, self.measured_current, self.measured_voltage]
            self._save_lookup_data()
        
        if self.measured_voltage <= BMS_UNDERVOLT_WARN and self.measured_voltage > 1:  # checking if the voltage isn't around 0, since the Pi could be connected 
            # to external power supplies, leading to near-zero reading on the INA260.  
            self.get_logger().warn(f"[{self.get_name()}] The battery is providing {BMS_UNDERVOLT_WARN}V or lower, please charge the battery. \
                                   Lowest recorded voltage while (barely) still in operation was about 6.7V.")
        
        elif self.measured_voltage <= BMS_UNDERVOLT_SHUTDOWN and self.measured_voltage > 1:  # will try to gracefully shutdown the entire system. 
            for _ in range(3):
                self.get_logger().error(f"[{self.get_name()}] The battery is providing {BMS_UNDERVOLT_SHUTDOWN}V or lower, \
                                        the system will summarily power down in 1 minute to preserve data.")
            
            self._shutdown(grace_time=1)

    def send_data(self):
        """Send the data to the ROS2 topic.
        """
        
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
    
    _bms_node = BatteryMonitorNode(node_name, topic_name, i2c_addr=0x44, lookup_recording=True)

    try:
        while rclpy.ok():
            rclpy.spin(_bms_node)
    except KeyboardInterrupt:
        _bms_node.get_logger().warn(f"KeyboardInterrupt triggered.")
    finally:
        _bms_node.destroy_node()
        rclpy.try_shutdown()  

