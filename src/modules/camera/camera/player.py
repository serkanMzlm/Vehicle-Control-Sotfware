#!/usr/bin/python3

import os
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory

class VideoPlayer(Node):
    def __init__(self):
        super().__init__("video_player_node")
        
        # Log an info message about exiting
        self.get_logger().info("Press 'q' to exit.")
        
        # Set up paths and publishers
        self.package_path = get_package_share_directory('vehicle_control_software')
        self.video_file_path = os.path.join(self.package_path, "video", "front_camera.avi")
        self.camera_publisher = self.create_publisher(Image, "camera", 10)
        self.cv_bridge = CvBridge()
        
        # Start video streaming
        self.initializeVideoStream()
    
    def initializeVideoStream(self):
        self.get_logger().info("Initializing video stream...")
        
        if os.path.exists(self.video_file_path):
            self.video_capture = cv2.VideoCapture(self.video_file_path)
            self.processVideoStream()
        else:
            self.get_logger().error("Video file not found.")
    
    def processVideoStream(self):
        while self.video_capture.isOpened():
            ret, frame = self.video_capture.read()
            if not ret:
                self.get_logger().info("End of video file reached.")
                break

            # Convert the frame to ROS Image message and publish
            image_message = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.camera_publisher.publish(image_message)

            # Display the frame
            cv2.imshow("Video Playback", frame)
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break

        # Release resources
        self.cleanupVideoStream()
    
    def cleanupVideoStream(self):
        """Release video capture and close any OpenCV windows."""
        if hasattr(self, 'video_capture'):
            self.video_capture.release()
        cv2.destroyAllWindows()
        self.get_logger().info("Video stream stopped.")

def main(args=None):
    rclpy.init(args=args)
    video_player = VideoPlayer()
    video_player.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
