import numpy as np
import cv2
import cv2.aruco as aruco
import yaml
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from .constants import ARUCO_DICT

class SensorCamera:
    def __init__(self, node: Node, aruco_detected_callback, angle=90, image_width=640):
        self.node = node
        self.aruco_detected_callback = aruco_detected_callback
        self.camera_calibration_yaml = f'support/dados_calibracao{angle}.yaml'
        self.load_calibration()
        self.aruco_dict = aruco.getPredefinedDictionary(ARUCO_DICT)
        self.parameters = aruco.DetectorParameters_create()
        self.bridge = CvBridge()
        self.image_sub = self.node.create_subscription(Image, '/camera', self.image_callback, 10)
        self.fov_width = np.degrees(1.5708 if angle == 90 else 1.047)
        self.image_width = image_width
        self.id_aruco_target = None

    def load_calibration(self):
        with open(self.camera_calibration_yaml, 'r') as file:
            data = yaml.safe_load(file)
            self.camera_matrix = np.array(data['camera_matrix'])
            self.dist_coeffs = np.array(data['dist_coeffs'])
            self.marker_size = float(data['marker_size'][0])

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            self.detect_and_decorate_arucos(cv_image)
        except CvBridgeError as error:
            self.node.get_logger().error(f'Could not convert image: {error}')

    def fix_target(self, marker_id):
        self.id_aruco_target = marker_id

    def detect_and_decorate_arucos(self, image):
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray_image, self.aruco_dict, parameters=self.parameters, cameraMatrix=self.camera_matrix, distCoeff=self.dist_coeffs)
        if ids is not None:
            for corner, marker_id in zip(corners, ids.flatten()):
                if self.is_target_marker(marker_id):
                    distance, angle = self.calculate_marker_position(corner)
                    self.aruco_detected_callback(distance, round(angle, 2))
                self.highlight_target_aruco(image, corner, marker_id)
        cv2.imshow('Aruco Detector', image)
        cv2.waitKey(1)

    def is_target_marker(self, marker_id):
        return self.id_aruco_target is not None and self.id_aruco_target == marker_id

    def calculate_marker_position(self, corner):
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, self.marker_size, self.camera_matrix, self.dist_coeffs)
        distance = np.linalg.norm(tvec[0])
        pixel_error = int(np.mean(corner[0], axis=0)[0]) - (self.image_width // 2)
        rotation_angle = pixel_error * (self.fov_width / self.image_width)
        return distance, rotation_angle

    def highlight_target_aruco(self, image, corner, marker_id):
        line_color = (0, 255, 0) if self.is_target_marker(marker_id) else (0, 0, 255)
        cv2.polylines(image, [np.int32(corner)], True, line_color, 1)
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, self.marker_size, self.camera_matrix, self.dist_coeffs)
        cv2.aruco.drawAxis(image, self.camera_matrix, self.dist_coeffs, rvec[0], tvec[0], self.marker_size * 0.5)

