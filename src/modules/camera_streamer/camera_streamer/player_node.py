#!/usr/bin/python3

import os
import cv2

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from ament_index_python.packages import get_package_share_directory


class Player(Node):
    def __init__(self):
        super().__init__("player_node") 
        self.get_logger().info("Press 'q' to exit.")
        
        self.parameters_path =  get_package_share_directory('vehicle_control_software')
        self.video_path = os.path.join(self.parameters_path + "/video", "front_camera.avi")
        self.camera_pub = self.create_publisher(Image, 'camera', 10)
        self.bridge = CvBridge()

        self.startVideoStream()

    def startVideoStream(self):
        self.get_logger().info("Starting video stream...")
        if os.path.exists(self.video_path):
            self.cap = cv2.VideoCapture(self.video_path)
            self.processVideoStream()
        else:
            self.get_logger().error("Video file not found.")

    def processVideoStream(self):
        while self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                break
                
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.camera_pub.publish(img_msg)

            cv2.imshow("Video", frame)
            if cv2.waitKey(10) & 0xFF == ord('q'):
                    break
            
        self.releaseVideoStream()
                    
    def releaseVideoStream(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        cv2.destroyAllWindows()
        self.get_logger().info("Video stream stopped.")

def main(args = None):
    rclpy.init(args=args)
    player = Player()
    player.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  	main()