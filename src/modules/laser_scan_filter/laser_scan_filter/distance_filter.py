import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class DistanceFilter(Node):
    def __init__(self):
        super().__init__("distance_filter_node")
        self.init()
        self.info()
        self.pub = self.create_publisher(LaserScan, '/laser_scan_distance_filter', 10)
        self.sub = self.create_subscription(LaserScan, '/laser_scan', self.laseScancallback, 10)
        
    def init(self):
        self.declare_parameter("max_distance", 3.0)
        self.declare_parameter("min_distance", 1.0)
        self.declare_parameter("frame_id", "world")
        self.max_distance = self.get_parameter("max_distance").value
        self.min_distance = self.get_parameter("min_distance").value
        self.frame_id = self.get_parameter("frame_id").value

    def info(self):
        self.get_logger().info("filter started to scan...")
        self.get_logger().info("max_distance: %.2f" % self.max_distance)
        self.get_logger().info("min_distance: %.2f" % self.min_distance)
        
    def laseScancallback(self, msg):      
        self.new_msg = LaserScan()
        self.new_msg.header = msg.header
        self.new_msg.range_max = msg.range_max
        self.new_msg.time_increment = msg.time_increment
        self.new_msg.angle_increment = msg.angle_increment
        self.new_msg.intensities = msg.intensities
        self.new_msg.angle_max = msg.angle_max
        self.new_msg.angle_min = msg.angle_min    

        self.new_msg.header.frame_id = self.frame_id
        self.new_msg.scan_time = self.max_distance
        self.new_msg.range_min = self.min_distance

        self.filtered_ranges = []
        for distance in msg.ranges:
            if distance > self.min_distance and distance < self.max_distance:
                self.filtered_ranges.append(distance)
            else:
                self.filtered_ranges.append(float('inf'))

        self.new_msg.ranges = self.filtered_ranges
        self.pub.publish(self.new_msg)

def main(args=None):
    rclpy.init(args=args)
    distance_node = DistanceFilter()
    rclpy.spin(distance_node)
    distance_node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()