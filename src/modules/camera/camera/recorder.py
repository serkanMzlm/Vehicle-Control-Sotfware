#!/usr/bin/python3

import os
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory

class VideoRecorderNode(Node):
    def __init__(self):
        super().__init__("video_recorder_node")
        
        # Initialize video capture subscription
        self.camera_subscription = self.create_subscription(
            Image,
            "camera",
            self.handleCameraImage,
            10
        )
        
        # Setup video file path and VideoWriter
        package_path = get_package_share_directory('vehicle_control_software')
        self.video_file_path = os.path.join(package_path, "video", "front_camera.avi")
        
        # VideoWriter settings
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.video_writer = cv2.VideoWriter(self.video_file_path, fourcc, 10, (480, 480))
        if not self.video_writer.isOpened():
            self.get_logger().error(f"Failed to open VideoWriter at {self.video_file_path}")
        
        # Initialize CvBridge and frame storage
        self.bridge = CvBridge()
        self.latest_frame = None
        
        # Timer for periodically updating the display
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.updateDisplay)
    
    def handleCameraImage(self, image_msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        self.get_logger().debug(f"Received image with size: {self.latest_frame.shape}, type: {self.latest_frame.dtype}")
        
        if self.video_writer.isOpened():
            self.video_writer.write(self.latest_frame)
    
    def updateDisplay(self):
        if self.latest_frame is not None:
            cv2.imshow("Front Camera", self.latest_frame)
            cv2.waitKey(1)  # Process GUI events
    
    def cleanupResources(self):
        if self.video_writer.isOpened():
            self.video_writer.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    video_recorder_node = VideoRecorderNode()

    try:
        rclpy.spin(video_recorder_node)
    except KeyboardInterrupt:
        pass
    finally:
        video_recorder_node.cleanupResources()
        video_recorder_node.destroy_node()
        if rclpy.ok():  
            rclpy.shutdown()

if __name__ == '__main__':
    main()
