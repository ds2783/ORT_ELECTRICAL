import time
import adafruit_gps
import serial

import rclpy
from rclpy.node import Node

from elysium.config.sensors import GPS_DEV_PORT, GPS_REFRESH_PERIOD

from ort_interfaces.msg import GPSStatus, SatelliteInfo

class GPS(Node):
    def __init__(self, baudrate, timeout):
        super().__init__('gps_publisher')

        # RX = board.RX
        # TX = board.TX

        uart = serial.Serial(GPS_DEV_PORT, baudrate, timeout=timeout)

        self.gps = adafruit_gps.GPS(uart, debug=False)
        # self.gps = adafruit_gps.GPS_GtopI2C(i2c, debug=False)

        # Enable all sentences (including GSA/GSV)
        self.gps.send_command(b"PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0")
        self.gps.send_command(b"PMTK220,1000")  # 1 Hz update rate

        self.publisher_ = self.create_publisher(GPSStatus, '/elysium/gps_data', 10)
        self.timer = self.create_timer(GPS_REFRESH_PERIOD, self.gpsCB_)
        self.last_print = time.monotonic()

        self.talkers = {
            "GA": "Galileo",
            "GB": "BeiDou",
            "GI": "NavIC",
            "GL": "GLONASS",
            "GP": "GPS",
            "GQ": "QZSS",
            "GN": "GNSS",
        }

    def format_dop(self, dop):
        if dop is None:
            return "Unknown"
        if dop > 20:
            return f"{dop:.1f} - Poor"
        elif dop > 10:
            return f"{dop:.1f} - Fair"
        elif dop > 5:
            return f"{dop:.1f} - Moderate"
        elif dop > 2:
            return f"{dop:.1f} - Good"
        elif dop > 1:
            return f"{dop:.1f} - Excellent"
        else:
            return f"{dop:.1f} - Ideal"

    def gpsCB_(self):
        self.gps.update()

        if not self.gps.has_fix:
            self.get_logger().info('Waiting for GPS fix...')
            return

        msg = GPSStatus()
        msg.latitude = float(self.gps.latitude)
        msg.longitude = float(self.gps.longitude)
        msg.fix_quality = int(self.gps.fix_quality) or 0
        msg.satellites = int(self.gps.satellites) or 0
        msg.pdop = float(self.gps.pdop)
        msg.hdop = float(self.gps.hdop)
        msg.vdop = float(self.gps.vdop)

        # Add satellite info
        msg.satellites_info = []
        if self.gps.sat_prns:
            for s in self.gps.sat_prns:
                sat_info = SatelliteInfo()
                sat_info.talker = self.talkers.get(s[0:2], "Unknown")
                sat_info.prn = s[2:]
                if self.gps.sats and s in self.gps.sats and self.gps.sats[s] is not None:
                    sat = self.gps.sats[s]
                    sat_info.elevation = int(sat[1]) if sat[1] is not None else -1
                    sat_info.azimuth = int(sat[2]) if sat[2] is not None else -1
                    sat_info.snr = int(sat[3]) if sat[3] is not None else -1
                else:
                    sat_info.elevation = -1
                    sat_info.azimuth = -1
                    sat_info.snr = -1
                msg.satellites_info.append(sat_info)

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {msg.latitude}, {msg.longitude} ({msg.satellites} sats)")

def main(args=None):
    rclpy.init(args=args)

    node = GPS(baudrate=9600, timeout=5)
    try:
        rclpy.spin(node)
    except:
        node.destroy_node()
        rclpy.shutdown()

