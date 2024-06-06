#!/usr/bin/python3

import os
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ament_index_python.packages import get_package_share_directory


class Player(Node):
    def __init__(self):
        super().__init__("player_node") 
        self.get_logger().info("q(exit)")
        self.parameters_path =  get_package_share_directory('parameters')
        self.video_path = os.path.join(self.parameters_path + "/video", "front_camera.avi")

        self.camera_pub = self.create_publisher(Image, 'camera', 10)
        # self.timer = self.create_timer(1.0 / 15, self.cameraCallback)

        self.bridge = CvBridge()
        self.cameraCallback()

    def cameraCallback(self):
        self.get_logger().info(self.video_path)
        if os.path.exists(self.video_path):
            self.cap = cv2.VideoCapture(self.video_path)
            while self.cap.isOpened():
                ret, frame = self.cap.read()
                if not ret:
                    break
                
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.camera_pub.publish(img_msg)
                cv2.imshow("Video", frame)
                if cv2.waitKey(10) & 0xFF == ord('q'):
                        break
                    
            self.cap.release()
            cv2.destroyAllWindows()

        else: 
            self.get_logger().error("File not opened")

def main(args = None):
	rclpy.init(args=args)
	player = Player()
	# rclpy.spin(player)    
	rclpy.shutdown()

if __name__ == '__main__':
  	main()