import glob
import pickle
import cv2 as cv
import numpy as np

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory

save_path = get_package_share_directory("vehicle_control_software")
calibration_path = save_path + "/calibration_data/"

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        self.get_logger().info("Calibration started")
        self.chessboard_size = (7, 7)
        self.frame_size = (720, 720)
        self.size_of_chessboard_squares_mm = 250
        self.criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        self.objp = self.createObjectPoints()
        self.objpoints, self.imgpoints = self.extractCornersFromImages()
        self.safeData()
    
    def createObjectPoints(self):
        objp = np.zeros((self.chessboard_size[0] * self.chessboard_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.chessboard_size[0], 0:self.chessboard_size[1]].T.reshape(-1, 2)
        return objp * self.size_of_chessboard_squares_mm
    
    def extractCornersFromImages(self):
        objpoints = []
        imgpoints = []

        images = glob.glob(save_path + '/images/*.png')
        if not images:
            self.get_logger().warn('No images found in the images folder.')
            raise SystemExit
        
        for image_path in images:
            img = cv.imread(image_path)
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

            ret, corners = cv.findChessboardCorners(gray, self.chessboard_size, None)
            if ret:
                objpoints.append(self.objp)

                corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
                imgpoints.append(corners2)

                cv.drawChessboardCorners(img, self.chessboard_size, corners2, ret)
                cv.imshow('img', img)
                cv.waitKey(500)

        return objpoints, imgpoints
    
    def safeData(self):
        ret, cameraMatrix, dist, rvecs, tvecs = cv.calibrateCamera(
            self.objpoints, self.imgpoints, self.frame_size, None, None
        )
        print(cameraMatrix)
        pickle.dump((cameraMatrix, dist), open(calibration_path + "calibration.pkl", "wb"))
        pickle.dump(cameraMatrix, open(calibration_path + "cameraMatrix.pkl", "wb"))
        pickle.dump(dist, open(calibration_path + "dist.pkl", "wb"))


def main(args=None):
    rclpy.init(args=args)
    calibrate_node = CalibrationNode()
    cv.destroyAllWindows()
    rclpy.shutdown() 

if __name__ == '__main__':
    main()