import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class FilterScan(Node):
    def __init__(self):
        super().__init__("filter_scan_node")
    
        self.declare_parameter("max_distance", 2.0)
        self.declare_parameter("min_distance", 0.0)          
        self.min_distance = self.get_parameter("min_distance").value
        self.max_distance = self.get_parameter("max_distance").value

        self.publishers_ = self.create_publisher(LaserScan, "scan2", 10)
        self.subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        self.get_logger().info("min distance: %.2f"%self.min_distance)
        self.get_logger().info("max distance: %.2f"%self.max_distance)

        self.get_logger().info("Filter Distance Started...")

    def scan_callback(self, msg):
        self.filter_msg = LaserScan()

        self.filter_msg.header = msg.header
        self.filter_msg.scan_time = msg.scan_time
        self.filter_msg.range_min = msg.range_min
        self.filter_msg.range_max = msg.range_max
        self.filter_msg.angle_increment = msg.angle_increment
        self.filter_msg.time_increment = msg.time_increment
        self.filter_msg.angle_min = msg.angle_min
        self.filter_msg.angle_max = msg.angle_max

        for distance in msg.ranges:
            if self.min_distance <= distance and distance <= self.max_distance:
                self.filter_msg.ranges.append(distance)
            else:
                if np.isnan(distance) or np.isinf(distance):
                    self.filter_msg.ranges.append(distance)
                else:
                    self.filter_msg.ranges.append(np.nan)
        self.publishers_.publish(self.filter_msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = FilterScan()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()