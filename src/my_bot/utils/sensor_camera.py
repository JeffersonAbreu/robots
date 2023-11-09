# As importações permanecem as mesmas.
import numpy as np
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
import yaml

class SensorCamera:
    def __init__(self, node: Node, aruco_detected_callback):
        self.camera_calibration_yaml ='/path/to/camera_calibration.yaml'
        self.node = node
        self.aruco_detected_callback = aruco_detected_callback
        self.marker_size = 0
        self.camera_matrix = []
        self.dist_coeffs = []
        self.id_aruco_target = None
        self.lock = False
        self.aruco_dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
        self.parameters = aruco.DetectorParameters_create()

        self.bridge = CvBridge()
        self.image_sub = self.node.create_subscription(Image, '/camera', self.__image_callback, 10)

    def __image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.node.get_logger().error('Could not convert image: %s' % e)
            return
        
        # O código de detecção e visualização dos marcadores ArUco permanece o mesmo.

    def load_calibration(self):
        with open(self.camera_calibration_yaml, 'r') as infile:
            calibration_data = yaml.safe_load(infile)
            self.camera_matrix = np.array(calibration_data['camera_matrix'])
            self.dist_coeffs = np.array(calibration_data['dist_coeffs'])
            self.marker_size = calibration_data['marker_size']

    def fix_target(self, id: int = None) -> None:
        self.id_aruco_target = id
        self.lock = False if id is None else self.lock

    def lock_target(self) -> None:
        self.lock = True

# O resto do código para iniciar o nó e a lógica ROS permanece inalterado.
