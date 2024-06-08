#!/usr/bin/python3

import os
import cv2

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from ament_index_python.packages import get_package_share_directory


class Recorder(Node):
    def __init__(self):
        super().__init__("recorder_node")
        self.camera_sub = self.create_subscription(Image, "camera", self.cameraCallback, 10)
        self.parameters_path =  get_package_share_directory('secure_drive_vehicle')
        self.video_path = os.path.join(self.parameters_path + "/video", "front_camera.avi")
        
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.out_video = cv2.VideoWriter(self.video_path, fourcc, 15, (360, 360))
        self.bridge = CvBridge()
    
    def cameraCallback(self, img):
        frame = self.bridge.imgmsg_to_cv2(img, 'bgr8')
        self.out_video.write(frame)
        cv2.imshow("Front Camera", frame)
        cv2.waitKey(1)
    
    def releaseVideoStream(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        cv2.destroyAllWindows()
        self.get_logger().info("Video stream stopped.")
       
def main(args = None):
    rclpy.init(args=args)
    recorder = Recorder()

    try:
        rclpy.spin(recorder)    
    except KeyboardInterrupt:
        pass
    
    recorder.out_video.release()
    cv2.destroyAllWindows()

    rclpy.shutdown()

if __name__ == '__main__':
  	main()