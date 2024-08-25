import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class AngleFilter(Node):
    def __init__(self):
        super().__init__("angle_filter_node")
        self.init()
        self.info()
        self.pub = self.create_publisher(LaserScan, '/laser_scan_angle_filter', 10)
        self.sub = self.create_subscription(LaserScan, '/laser_scan', self.laseScancallback, 10)

    def init(self):
        self.declare_parameter("max_angle" ,180)
        self.declare_parameter("min_angle" ,0)
        self.declare_parameter("frame_id", "world")
        self.max_angle = self.get_parameter("max_angle").value
        self.min_angle = self.get_parameter("min_angle").value
        self.frame_id = self.get_parameter("frame_id").value

    def info(self):
        self.get_logger().info("filter started to scan...")
        self.get_logger().info("max_angle: %.2f"%self.max_angle)
        self.get_logger().info("min_angle: %.2f"%self.min_angle)

    def laseScancallback(self, msg):
        self.new_msg = LaserScan()
        self.max_angle = self.get_parameter("max_angle").value
        self.min_angle = self.get_parameter("min_angle").value
        self.deg2rad(self.min_angle, self.max_angle)

        self.new_msg.header = msg.header
        self.new_msg.header.frame_id = self.frame_id
        self.new_msg.scan_time = msg.scan_time
        self.new_msg.range_max = msg.range_max
        self.new_msg.range_min = msg.range_min
        self.new_msg.time_increment = msg.time_increment
        self.new_msg.angle_increment = msg.angle_increment

        self.radian_angle_max = msg.angle_max
        self.radian_angle_min = msg.angle_min
        self.calculateIndex(self.desired_min_angle, self.desired_max_angle)
        self.new_msg.ranges = msg.ranges[self.start_index:self.end_index]

        self.pub.publish(self.new_msg)

    def calculateIndex(self, min_angle_rad, max_angle_rad):
        if max_angle_rad > self.radian_angle_max:
            self.desired_max_angle = self.radian_angle_max

        if min_angle_rad < self.radian_angle_min:
            self.desired_min_angle = self.radian_angle_min

        self.new_msg.angle_max = self.desired_max_angle
        self.new_msg.angle_min = self.desired_min_angle

        self.start_index = int((self.desired_min_angle - self.radian_angle_min)/self.new_msg.angle_increment)
        self.end_index = int((self.desired_max_angle - self.radian_angle_min)/self.new_msg.angle_increment)
        
    def deg2rad(self, min_angle, max_angle):    
        self.desired_min_angle = math.radians(min_angle)
        self.desired_max_angle = math.radians(max_angle)

def main(args=None):
    rclpy.init(args=args)
    filter_scan_node = AngleFilter()
    rclpy.spin(filter_scan_node)
    filter_scan_node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()