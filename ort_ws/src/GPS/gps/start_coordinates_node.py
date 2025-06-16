import rclpy
from rclpy.node import Node

from ort_interfaces.msg import GPSStatus, Coordinate

class StartCoordinatesNode(Node):
    def __init__(self):
        super().__init__('start_coordinates_node')

        self.publisher_ = self.create_publisher(Coordinate, '/start_coordinates', 10)
        self.subscription = self.create_subscription(
            GPSStatus,
            '/gps_data',
            self.gps_callback,
            10)

        self.start_lat = None
        self.start_lon = None
        self.timer = None

    def gps_callback(self, msg):
        if self.start_lat is None and msg.fix_quality > 0 and msg.latitude and msg.longitude:
            # Save the first fix coordinates
            self.start_lat = msg.latitude
            self.start_lon = msg.longitude
            self.get_logger().info(f"Captured start coordinates: {self.start_lat}, {self.start_lon}")

            # Start timer to publish these coordinates repeatedly every 1 second
            self.timer = self.create_timer(1.0, self.publish_start_coordinates)

    def publish_start_coordinates(self):
        if self.start_lat is None or self.start_lon is None:
            return  # Safety check

        start_msg = Coordinate()
        start_msg.latitude = self.start_lat
        start_msg.longitude = self.start_lon
        self.publisher_.publish(start_msg)
        self.get_logger().info(f"Published start coordinates: {start_msg.latitude}, {start_msg.longitude}")


def main(args=None):
    rclpy.init(args=args)
    node = StartCoordinatesNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
