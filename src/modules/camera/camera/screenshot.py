import cv2
import os
import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory

class ScreenshotNode(Node):
    def __init__(self):
        super().__init__('screenshot_node')

        # Declare and get the 'source' parameter to determine the input source
        self.declare_parameter('input_source', 'camera')
        self.input_source = self.get_parameter('input_source').get_parameter_value().string_value

        # Initialize the capture mechanism based on the input source
        if self.input_source == 'camera':
            self.get_logger().info("Using camera input.")
            self.camera_capture = cv2.VideoCapture(0)
            self.timer = self.create_timer(0.05, self.captureFromCamera)
        elif self.input_source == 'simulation':
            self.get_logger().info("Using simulation input.")
            self.subscription = self.create_subscription(
                Image, "camera", self.captureFromSimulation, qos_profile_sensor_data
            )
            self.bridge = CvBridge()
        else:
            self.get_logger().error(f"Invalid input source: {self.input_source}")
            raise SystemExit
        
        # Log the instructions for the user
        self.get_logger().info("Press 's' to save the image.")
        self.get_logger().info("Press 'q' to quit the program.")
        self.get_logger().debug(f"Input Source: {self.input_source}")

        self.image_save_path = os.path.join(get_package_share_directory('vehicle_control_software'), 'images')
        os.makedirs(self.image_save_path, exist_ok=True)
        self.image_count = 0

    def captureFromCamera(self):
        if self.camera_capture.isOpened():
            ret, frame = self.camera_capture.read()
            if ret:
                cv2.imshow("Camera Feed", frame)
                self.handleUserInput(frame)
            else:
                self.get_logger().error("Failed to capture frame from camera.")
    
    def captureFromSimulation(self, image_msg):
        frame = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        cv2.imshow("Simulation Feed", frame)
        self.handleUserInput(frame)

    def handleUserInput(self, frame):
        key = cv2.waitKey(1)
        if key == ord('s'):
            self.saveImage(frame)
        elif key == ord('q'):
            raise SystemExit

    def saveImage(self, frame):
        """Save the captured frame to the specified directory."""
        image_filename = os.path.join(self.image_save_path, f'img_{self.image_count}.png')
        cv2.imwrite(image_filename, frame)
        self.get_logger().info(f"Image saved as: img_{self.image_count}.png")
        self.get_logger().debug(f"Save path: {self.image_save_path}")
        self.image_count += 1

    def releaseResources(self):
        """Release camera resources and destroy all OpenCV windows."""
        if self.input_source == 'camera' and hasattr(self, 'camera_capture'):
            self.camera_capture.release()
        cv2.destroyAllWindows()
        self.get_logger().info("Resources released and program terminated.")

def main(args=None):
    rclpy.init(args=args)
    screenshot_node = ScreenshotNode()

    try:
        rclpy.spin(screenshot_node)
    except SystemExit:
        rclpy.logging.get_logger("Shutdown").info("Exiting the program.")
    finally:
        screenshot_node.releaseResources()
        screenshot_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
