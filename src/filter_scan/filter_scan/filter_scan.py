import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import math

class MyNode(Node):
    def __init__(self):
        super().__init__("filter_scan_node")
        self.declare_parameter("max_angle", 30)
        self.declare_parameter("min_angle", 10)          
        self.min_angle = self.get_parameter("min_angle").value
        self.max_angle = self.get_parameter("max_angle").value
        self.deg2rad(self.max_angle, self.min_angle)

        self.publishers_ = self.create_publisher(LaserScan, "scan2", 10)
        self.subscription = self.create_subscription(LaserScan, 'scan',
            self.scan_callback, 10)
        self.get_logger().info("started...")

    def scan_callback(self, msg):
        self.new_scan_msg = LaserScan()
        self.new_scan_msg.header = msg.header
        self.prev_max = msg.angle_max
        self.prev_min = msg.angle_min
        self.new_scan_msg.angle_increment = msg.angle_increment
        self.new_scan_msg.time_increment = msg.time_increment
        self.new_scan_msg.scan_time = msg.scan_time
        self.new_scan_msg.range_min = msg.range_min
        self.new_scan_msg.range_max = msg.range_max
        # self.get_logger().info("start:::: %.2f"%self.desired_min_agnle)
        self.calculateIndex(self.desired_max_agnle, self.desired_min_agnle)
        start_angle = self.desired_min_agnle 
        end_angle = self.desired_max_agnle 
        start_index =  int((start_angle - self.prev_min) / msg.angle_increment)
        end_index = int((end_angle - self.prev_min) / msg.angle_increment)
        
        self.new_scan_msg.angle_max = end_angle
        self.new_scan_msg.angle_min = start_angle
        self.get_logger().info("start index: %d"%start_index)
        self.get_logger().info("stop index: %d"%end_index)
        self.get_logger().info("start: %.2f"%start_angle)
        self.get_logger().info("stop: %.2f"%end_angle)
        self.new_scan_msg.ranges = msg.ranges[start_index:end_index]
       
        self.publishers_.publish(self.new_scan_msg)
    
    def deg2rad(self, max_degree, min_degree):
        self.desired_min_agnle = math.radians(min_degree)
        self.desired_max_agnle = math.radians(max_degree)
        self.get_logger().info("%.2f"%self.desired_min_agnle)
        self.get_logger().info("%.2f"%self.desired_max_agnle)
    
    def calculateIndex(self, max_angle, min_angle):
        if max_angle > self.prev_max:
            self.desired_max_agnle = self.prev_max
        if min_angle < self.prev_min:
            self.get_logger().info("True")
            self.desired_min_agnle = self.prev_min
        

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()