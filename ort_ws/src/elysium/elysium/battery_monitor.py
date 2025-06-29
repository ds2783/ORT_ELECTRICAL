import rclpy
import rclpy.utilities
from rclpy.node import Node

import pandas
import pandas.errors
import numpy as np
from pathlib import Path
import busio, board

import elysium.hardware.adafruit_ina260 as _ina260
from elysium.config.sensors import (
    BMS_REFRESH_PERIOD,
    BMS_DELTA_T,
    BMS_ROLLING_AVERAGE_SECONDS,
    BMS_UNDERVOLT_WARN,
    BMS_UNDERVOLT_SHUTDOWN,
    BMS_BATTERY_CAPACITY,
    BMS_SAVE_PATH,
    BMS_LOOKUP_TABLE_PATH,
    OCV_ARRAY_SIZE,
    tofQoS,
)
from ort_interfaces.msg import BatteryInfo


# BMS CAPACITY LOGIC DERIVED HERE: https://www.youtube.com/watch?v=rOwcxFErcvQ

# USING A COMBINATION OF SIMPLISTIC COLOUMB COUNTING AND AN 'OCV' LOOKUP TABLE.
# SUBSEQUENT TBRO MEMBERS SHOULD LOOK AT KALMANN FILTERS FOR A MORE OPTIMISED ESTIMATION OF THE SOC OF THE BATTERY.


class RollingAverage:
    def __init__(self, length):
        self.items = 0
        self.length = length
        self._queue = np.zeros(length, dtype=np.float32)
    
    def add(self, value):
        if self.items < self.length:
            self.items += 1
    
        self._queue[:-1] = self._queue[1:]
        self._queue[-1] = value

    @property
    def average(self):
        _items = self.items if self.items else 1  # Avoid dividing by zero.
        
        return np.sum(self._queue)/_items
    

class BatteryMonitorNode(Node):
    def __init__(self, node_name, topic_name, i2c_addr=0x40, recording_lookup=False):
        """Battery Monitoring Node using the INA260.
        Measures the voltage, current and power going throught INA260, and estimates
        the SOC (State of Charge) of the battery LiPo cells.

        :param node_name: node name
        :type node_name: str
        :param topic_name: topic name
        :type topic_name: str
        :param i2c_addr: i2c address on the i2c bus, defaults to 0x40
        :type i2c_addr: hexadecimal, optional
        :param recording_lookup: set to True if you want to create a new OCV lookup table, defaults to False
        :type recording_lookup: bool, optional
        """

        super().__init__(node_name)

        msg_type = BatteryInfo
        self.bms_publisher = self.create_publisher(
            msg_type=msg_type, topic=topic_name, qos_profile=tofQoS
        )
        self.publisher_timer = self.create_timer(BMS_REFRESH_PERIOD, self.send_data)
        self._tmp_timer = self.create_timer(BMS_ROLLING_AVERAGE_SECONDS, self._rebase_voltages)    

        i2c = busio.I2C(board.SCL, board.SDA)
        self.bms = _ina260.INA260(i2c, address=i2c_addr)

        self.bms.averaging_count = (
            _ina260.AveragingCount.COUNT_16
        )  # averaging out on 16 samples
        self.bms.voltage_conversion_time = _ina260.ConversionTime.TIME_2_116_ms

        self.delta_t = self.create_timer(BMS_DELTA_T, self.get_data)

        self.measured_voltage, self.measured_current, self.measured_power, self.est_time_remaining = (
            0.0,
            0.0,
            0.0,
            0.0
        )
        
        self.charge_avg = RollingAverage(length=round(BMS_ROLLING_AVERAGE_SECONDS/BMS_DELTA_T))
        self.voltage_avg = RollingAverage(length=round(BMS_ROLLING_AVERAGE_SECONDS/BMS_DELTA_T))

        self.current_capacity = BMS_BATTERY_CAPACITY
        self.total_capacity = BMS_BATTERY_CAPACITY

        # We are going to assume we are entering with a full battery. This needs to be backed up by a voltage lookup table I'll generate by running a few
        # discharges while tracking the current integrated SOC.

        # 4.2 max per cell voltage on a 12.6V 1.5A full charge.
        # ~8.8/3 min per cell voltage before the RPi5 succumbed to low voltage.

        self.lookup = recording_lookup  # if this should be a full-discharge lookup table calibration run.

        self.lookup_table = self._read_lookup_data()
        self.soc, self.prev_soc = (
            self._read_battery_file()
        )  # state of charge = capacity remaining / total capacity

    @property
    def soc(self):
        return round(self._soc, ndigits=3)

    @soc.setter
    def soc(self, value):

        if value < 0:
            value = 0

        self._soc = value
        self.current_capacity = self._soc * self.total_capacity

    def _rebase_voltages(self):
        """Run X seconds after rclpy spins the node. Destroys the timer afterwards. 
        """
        
        if not self.lookup:
            self._compare_ocv_soc()  # sanity check the stored SOC values against the 'OCV' values in the lookup table.
        self.destroy_timer(self._tmp_timer)

    def _read_battery_file(self, path=BMS_SAVE_PATH):
        """If the BMS_SAVE_PATH file exists, read and typecast into float.

        If it doesn't, and the lookup table is not complete, just assume we are recording a new lookup table and set SOC to full.

        Otherwise, use the lookup table to return a SOC value.

        :param path: path, found in the config.sensors file, defaults to BMS_SAVE_PATH
        :type path: PathLike, optional
        :return: SOC value
        :rtype: float
        """

        if Path(
            path
        ).is_file():  # Under the assumption that there isn't a SOC to be had, refer to the lookup table.
            with open(path, "r") as fs:
                data = fs.readline()  # for now we are just keeping the data as a string
            self.soc = float(data)  # typecast the chars into a float

        else:
            tmp = self.bms.voltage

            if (
                self.lookup_table == 0
            ).sum().sum() > 2000:  # if the number of zeroes is above 2000, assume that
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
                dataframe = pandas.read_csv(path, sep=",", index_col=0, dtype=pandas.Float64Dtype())
            except pandas.errors.EmptyDataError:
                pass

        dataframe = dataframe.fillna(0)  # fills empty NAN values with 0.

        cond_1 = dataframe.shape != (OCV_ARRAY_SIZE + 1, 4)  # if the dataframe is malformed
        cond_2 = (
            dataframe == 0
        ).sum().sum() > 2500  # if there is more than 625 entries empty
        cond_3 = self.lookup  # if the lookup bool is true

        if cond_1 or (
            cond_2 and cond_3
        ):  # create a new dataframe if there doesn't exist one.
            _row_range = pandas.array(
                range(0, OCV_ARRAY_SIZE + 1)
            )  # 0 to OCV_ARRAY_SIZE, 0 inclusive which is why we use OCV_ARRAY_SIZE + 1.
            _soc_values = pandas.array(range(0, OCV_ARRAY_SIZE + 1)) / OCV_ARRAY_SIZE
            new_dataframe = pandas.DataFrame(
                index=_row_range,
                columns=["soc", "charge", "current", "ocv"],
                dtype=pandas.Float64Dtype(),
            )
            new_dataframe = new_dataframe.fillna(0)  # fills empty NAN values with 0.
            new_dataframe.iloc[:, 0] = _soc_values
            dataframe = new_dataframe

        return dataframe

    def _save_lookup_data(self, path=BMS_LOOKUP_TABLE_PATH):
        """Save the lookup table.

        :param path: path, found in the config.sensors file, defaults to BMS_LOOKUP_TABLE_PATH
        :type path: PathLike, optional
        """

        if not Path(path).is_file():  # validate whether or not the file exists already.
            self._create_blank_file(path)

        self.lookup_table.to_csv(path, sep=",", na_rep=0)

    def _create_blank_file(self, path):
        """Create a blank file.

        :param path: path
        :type path: PathLike
        """

        try:
            with open(path, "w") as fs:
                ...
        except OSError:
            self.get_logger().error(
                f"[{self.get_name()}] - OSError: probably given a bad path for the file ({path})."
            )

    def _compare_ocv_soc(self, deviation=0.05):
        """Compare the OCV lookup value with the immediate measured voltage.

        If there is more than a 5% deviation (default value), 
        assume we have drifted too far and set the SOC to the OCV lookup value.

        :param deviation: soc deviation, defaults to 0.05
        :type deviation: float, optional
        """

        tmp_voltage = self.voltage_avg.average
        lookup_soc = self._find_ocv_soc(tmp_voltage)
        
        if lookup_soc:
            if abs(lookup_soc - self.soc) >= deviation:
                self.get_logger().warn(
                        f"[{self.get_name()}] The expected SOC value is more than {self.soc * 100:1f}% different to what we are expecting of the battery voltage, rebasing SOC."
                )
                tmp = self.soc
                self.soc = lookup_soc

                self.get_logger().warn(f"Old value: {tmp:.3f}, new value: {self.soc:.3f}, measured voltage: {tmp_voltage:.3f}")
        else:
            self.get_logger().warn("Failed to check rebase, as no valid soc interpolation found.")

    def _find_ocv_soc(self, ocv: float):
        """Find the SOC from the OCV table voltages, needs to iterate through the table from the smallest value upwards.

        :param ocv: ocv voltage
        :type ocv: float
        :return: soc
        :rtype: float
        """

        voltage = self.lookup_table["ocv"]
        soc = self.lookup_table["soc"]

        try:
            lookupval = np.interp(ocv, voltage, soc)
            if lookupval is None:
                self.get_logger().warn(
                    "Interpolation returned None, invalid Interpolation."
                )
            else:
                return float(lookupval)
        except:
            self.get_logger().warn("Could not interpolate lookup OCV.")

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
        self.measured_power = self.bms.power  # mW

        charge_expended = (
            self.measured_current * 1e-3 * BMS_DELTA_T
        * (1 / 3.6))  # mA * 0.0001 * dt, giving charge in mAh.
        self.soc = (
            self.current_capacity - charge_expended 
        ) / self.total_capacity 

        self.charge_avg.add(charge_expended)
        self.est_time_remaining = self.current_capacity / (self.charge_avg.average / BMS_DELTA_T)  # estimated time remaining in seconds.
        
        self.voltage_avg.add(self.measured_voltage)

        if (abs(self.prev_soc - self.soc) >= 0.001
        ):  # if the soc value has dropped 0.1%, check and save the data to the lookup table.
            self.prev_soc = self.soc
            self._save_battery_file()

            if self.lookup:
                self.lookup_table.iloc[int(round(OCV_ARRAY_SIZE * self.soc)), 1:] = [
                    charge_expended,
                    self.measured_current,
                    self.measured_voltage,
                ]
                self._save_lookup_data()

        if (
            self.measured_voltage <= BMS_UNDERVOLT_WARN and self.measured_voltage > 1
        ):  # checking if the voltage isn't around 0, since the Pi could be connected
            # to external power supplies, leading to near-zero reading on the INA260.
            self.get_logger().warn(
                f"[{self.get_name()}] The battery is providing {BMS_UNDERVOLT_WARN}V or lower, please charge the battery. \
                                   Lowest recorded voltage while (barely) still in operation was about 6.7V."
            )

        elif (
            self.measured_voltage <= BMS_UNDERVOLT_SHUTDOWN
            and self.measured_voltage > 1
        ):  # will try to gracefully shutdown the entire system.
            for _ in range(3):
                self.get_logger().error(
                    f"[{self.get_name()}] The battery is providing {BMS_UNDERVOLT_SHUTDOWN}V or lower, \
                                        the system will summarily power down in 1 minute to preserve data."
                )

            self._shutdown(grace_time=1)

    def send_data(self):
        """Send the data to the ROS2 topic."""

        msg = BatteryInfo()

        msg.voltage = float(self.measured_voltage)
        msg.current = float(self.measured_current)
        msg.power = float(self.measured_power)
        msg.soc = float(self.soc)
        msg.remaining_capacity = float(self.current_capacity)
        msg.total_capacity = float(self.total_capacity)
        msg.estimated_time_remaining = float(self.est_time_remaining)

        self.bms_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    topic_name = "/battery_monitor"
    node_name = "battery_monitor"

    _bms_node = BatteryMonitorNode(
        node_name, topic_name, i2c_addr=0x44, recording_lookup=False
    )

    try:
        while rclpy.utilities.ok():
            rclpy.spin(_bms_node)
    except KeyboardInterrupt:
        _bms_node.get_logger().warn(f"KeyboardInterrupt triggered.")
    finally:
        _bms_node.destroy_node()
        rclpy.utilities.try_shutdown()
