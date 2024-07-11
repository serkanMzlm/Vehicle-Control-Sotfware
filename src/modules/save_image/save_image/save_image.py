import cv2
import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data

from ament_index_python.packages import get_package_share_directory

vio_sim_path = get_package_share_directory("vehicle_control_software")

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.declare_parameter('select_input', 'camera')
        self.select_input = self.get_parameter('select_input').get_parameter_value().string_value

        if self.select_input == 'camera':
            self.get_logger().info("camera")
            self.cap = cv2.VideoCapture(0)
            self.timer = self.create_timer(0.05, self.camera_callback)
        elif self.select_input == 'simulation':
            self.get_logger().info("simulation")
            self.get_logger().info("s: Saves the image")
            self.get_logger().info("q: Terminates the program")
            self.get_logger().info(vio_sim_path)
            self.subscription_ = self.create_subscription(
                Image, "camera", self.sim_callback, qos_profile_sensor_data
            )
            self.br = CvBridge()
        else:
            self.get_logger().error('Invalid input mode: %s' % self.select_input)
            raise SystemExit
        
        self.get_logger().debug('Input Mode: %s' %self.select_input)
        self.i = 0

    def camera_callback(self):
        if self.cap.isOpened():
            ret, img = self.cap.read()
            if ret:
                cv2.imshow("Image", img)
                k = cv2.waitKey(1)
                if k == ord('s'):
                    cv2.imwrite(vio_sim_path + '/images/img' + str(self.i) + '.png', img)
                    self.get_logger().info('image saved: img%d' %self.i)
                    self.get_logger().debug('Save path: %s' %vio_sim_path)
                    self.i += 1
                elif k == ord('q'):
                    raise SystemExit

    def sim_callback(self, data):
        frame = self.br.imgmsg_to_cv2(data, "bgr8")
        cv2.imshow("Camera", frame)
        k = cv2.waitKey(1)
        if k == ord('s'):
            cv2.imwrite(vio_sim_path + '/images/img' + str(self.i) + '.png', frame)
            self.get_logger().info('image saved: img%d' %self.i)
            self.get_logger().debug('Save path: %s' %vio_sim_path)
            self.i += 1
        elif k == ord('q'):
            raise SystemExit

    def release_camera(self):
        if self.select_input == 'camera':
            self.get_logger().info('Release camera called.')
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    try:
        rclpy.spin(camera_node)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
        camera_node.release_camera()

    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
