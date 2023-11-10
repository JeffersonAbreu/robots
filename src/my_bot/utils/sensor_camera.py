import numpy as np
import cv2.aruco as aruco #version 4.8
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image

class SensorCamera:
    def __init__(self, node: Node, aruco_detected_callback):
        # Inicializador da classe, define variáveis e inicializa a subscrição da imagem
        self.node = node
        self.aruco_detected_callback = aruco_detected_callback
        self.id_aruco_target = None
        self.lock = False
        self.aruco_dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
        self.parameters = aruco.DetectorParameters_create()

        self.bridge = CvBridge()
        self.image_sub = self.node.create_subscription(
            Image, '/camera', self.__image_callback, 10)

    def __image_callback(self, data):
        # Callback para quando uma imagem é recebida
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.detect_and_decorate_arucos(cv_image)
        except CvBridgeError as e:
            self.node.get_logger().error(f'Could not convert image: {e}')
            return

    def detect_and_decorate_arucos(self, cv_image):
        # Detecta e decora os marcadores ArUco na imagem
        corners, ids, rejectedImgPoints = aruco.detectMarkers(
            cv_image, self.aruco_dictionary, parameters=self.parameters)
        
        if ids is not None:
            if self.id_aruco_target in ids:
                index = list(ids).index(self.id_aruco_target)
                target_corner = corners[index]
                self.mark_target(cv_image, target_corner)
                if not self.lock:
                    self.aruco_detected_callback(self.id_aruco_target)

            aruco.drawDetectedMarkers(cv_image, corners, ids)

    def mark_target(self, cv_image, target_corner):
        # Marca o ArUco alvo com um círculo vermelho
        center = target_corner.mean(axis=1).astype(int).flatten()
        cv2.circle(cv_image, tuple(center), 30, (0, 0, 255), 3)

    def fix_target(self, id):
        pass

# A continuação do código depende de como o callback aruco_detected_callback é definido e como o processamento de imagem é integrado ao nó ROS2.
# No entanto, com base na descrição, esta é a estrutura básica de uma classe que atende aos requisitos mencionados.
