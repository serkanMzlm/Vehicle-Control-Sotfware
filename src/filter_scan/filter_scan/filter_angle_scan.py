import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class FilterScan(Node):
    def __init__(self):
        super().__init__("filter_angle_node")
        
        self.declare_parameter("max_angle", 30)
        self.declare_parameter("min_angle", 10)          
        self.min_angle = self.get_parameter("min_angle").value
        self.max_angle = self.get_parameter("max_angle").value

        self.deg2rad(self.min_angle, self.max_angle)

        self.publishers_ = self.create_publisher(LaserScan, "scan2", 10)
        self.subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.get_logger().info("Filter Started...")

    def scan_callback(self, msg):
        self.filter_msg = LaserScan()

        # İki scan verisinde de ortak veriler
        self.filter_msg.header = msg.header
        self.filter_msg.scan_time = msg.scan_time
        self.filter_msg.range_min = msg.range_min
        self.filter_msg.range_max = msg.range_max
        self.filter_msg.angle_increment = msg.angle_increment
        self.filter_msg.time_increment = msg.time_increment

        # Scan verisinde farklı olucak veriler
        self.rad_angle_max = msg.angle_max
        self.rad_angle_min = msg.angle_min
        
        self.calculateIndex(self.desired_min_agnle, self.desired_max_agnle)

        self.filter_msg.ranges = msg.ranges[self.start_index:self.end_index]
        self.publishers_.publish(self.filter_msg)
    
    def deg2rad(self, min_degree, max_degree):
        self.desired_min_agnle = math.radians(min_degree)
        self.desired_max_agnle = math.radians(max_degree)
        self.get_logger().info("%.2f"%self.desired_min_agnle)
        self.get_logger().info("%.2f"%self.desired_max_agnle)
    
    def calculateIndex(self, min_angle, max_angle):
        if max_angle > self.rad_angle_max:
            self.desired_max_agnle = self.rad_angle_max
        if min_angle < self.rad_angle_min:
            self.desired_min_agnle = self.rad_angle_min
        
        self.filter_msg.angle_min = self.desired_min_agnle 
        self.filter_msg.angle_max = self.desired_max_agnle 
        self.start_index =  int((self.desired_min_agnle  - self.rad_angle_min) / self.filter_msg.angle_increment)
        self.end_index = int((self.desired_max_agnle  - self.rad_angle_min) / self.filter_msg.angle_increment)

def main(args=None):
    rclpy.init(args=args)
    node = FilterScan()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()