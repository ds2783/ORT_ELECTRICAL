import math
import rclpy
from rclpy.node import Node

from gps_msgs.msg import GPSStatus
from gps_msgs.msg import Coordinate

class DistanceCalculatorNode(Node):
    def __init__(self):
        super().__init__('distance_calculator_node')

        self.start_lat = None
        self.start_lon = None

        self.create_subscription(Coordinate, 'start_coordinates', self.start_coordinates_callback, 10)
        self.create_subscription(GPSStatus, 'gps_data', self.gps_callback, 10)

    def start_coordinates_callback(self, msg):
        try:
            self.start_lat = float(msg.latitude)
            self.start_lon = float(msg.longitude)
            self.get_logger().info(f"Updated start coordinates: {self.start_lat}, {self.start_lon}")
        except ValueError:
            self.get_logger().warn("Invalid start coordinates received")

    def haversine(self, lat1, lon1, lat2, lon2):
        R = 6371000  # Earth radius in meters
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        a = math.sin(delta_phi / 2.0) ** 2 + \
            math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2

        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return R * c

    def gps_callback(self, msg):
        if self.start_lat is None or self.start_lon is None:
            self.get_logger().info("Waiting for start coordinates...")
            return

        try:
            current_lat = float(msg.latitude)
            current_lon = float(msg.longitude)
        except ValueError:
            self.get_logger().warn("Invalid GPS data")
            return

        distance = self.haversine(self.start_lat, self.start_lon, current_lat, current_lon)
        self.get_logger().info(f"Distance from start: {distance:.2f} meters")


def main(args=None):
    rclpy.init(args=args)
    node = DistanceCalculatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
