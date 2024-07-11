#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import time
from pmw3901 import PMW3901

class MyNode(Node):
    def __init__(self):
        super().__init__('pmw3901_node')
        self.publisher_ = self.create_publisher(Twist, 'optical_flow', 10)

        self.sensor = PMW3901(spi_port=0, spi_cs_gpio=8)
        self.sensor.set_rotation(0)

        self.tx = 0
        self.ty = 0

        self.timer_ = self.create_timer(0.1, self.publish_sensor_data)
        self.get_logger().info("pmw3901_node started...")

    def publish_sensor_data(self):
        try:
            x, y = self.sensor.get_motion()
        except RuntimeError:
            return

        # self.tx += x
        # self.ty += y

        msg = Twist()
        msg.linear.x = float(x)
        msg.linear.y = float(y)

        self.publisher_.publish(msg)
        # self.get_logger().info("Relative: x {:03d} y {:03d}".format(x, y))

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
