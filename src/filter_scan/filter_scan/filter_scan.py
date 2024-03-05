import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import math

class MyNode(Node):
    def __init__(self):
        super().__init__("filter_scan_node")
        self.declare_parameter("max_angle", 5)
        self.declare_parameter("min_angle", -5)          
        self.min_angle = self.get_parameter("min_angle").value
        self.max_angle = self.get_parameter("max_angle").value
        self.publishers_ = self.create_publisher(LaserScan, "scan2", 10)
        self.subscription = self.create_subscription(LaserScan, 'scan',
            self.scan_callback, 10)
        self.get_logger().info("started...")

    def scan_callback(self, msg):
        new_scan_msg = LaserScan()
        new_scan_msg.header = msg.header
        new_scan_msg.angle_min = msg.angle_min
        new_scan_msg.angle_max = msg.angle_max
        new_scan_msg.angle_increment = msg.angle_increment
        new_scan_msg.time_increment = msg.time_increment
        new_scan_msg.scan_time = msg.scan_time
        new_scan_msg.range_min = msg.range_min
        new_scan_msg.range_max = msg.range_max
        start_angle = math.radians(10) 
        end_angle = math.radians(15)  
        new_scan_msg.ranges = msg.ranges[int(start_angle / msg.angle_increment):int(end_angle / msg.angle_increment)]
       
        self.publishers_.publish(new_scan_msg)
    
    def deg2rad(self, max_degree, min_degree):
        self.desired_min_agnle = math.radians(min_degree)
        self.desired_max_agnle = math.radians(max_degree)
    
    def calculateIndex(self, max_degree, min_degree):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()