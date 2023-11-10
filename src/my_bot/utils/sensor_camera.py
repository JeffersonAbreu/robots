import numpy as np
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from .constants import ARUCO_DICT, MARKER_SIZE

class SensorCamera:
    def __init__(self, node: Node, aruco_detected_callback):
        self.node = node
        self.aruco_detected_callback = aruco_detected_callback
        self.id_aruco_target = None
        self.lock = False
        self.aruco_dictionary = aruco.getPredefinedDictionary(ARUCO_DICT)
        self.parameters = aruco.DetectorParameters_create()
        self.bridge = CvBridge()
        self.image_sub = self.node.create_subscription(
            Image, '/camera', self.__image_callback, 10)

    def __image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.detect_and_decorate_arucos(cv_image)
            cv2.imshow("Camera View", cv_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            self.node.get_logger().error(f'Could not convert image: {e}')

    def detect_and_decorate_arucos(self, cv_image):
        corners, ids, rejectedImgPoints = aruco.detectMarkers(
            cv_image, self.aruco_dictionary, parameters=self.parameters)

        if ids is not None:
            for i, corner in enumerate(corners):
                id = ids[i][0]
                if id == self.id_aruco_target:
                    # Garantindo que os cantos são uma matriz NumPy válida
                    corner = np.array(corner, dtype=np.int32)
                    # Calcula o centro do marcador ArUco
                    center_x, center_y, error_x, error_y, distance = self.calculate_marker_position(cv_image, corner)
                    if distance <= 5:
                        # Verifica se o contorno tem pontos válidos antes de desenhar
                        if corner.size > 0:
                            # Colore de verde se estiver dentro de 5 metros
                            cv2.drawContours(cv_image, [corner], -1, (0, 255, 0), 5)
                        if abs(error_x) < 0.05 * cv_image.shape[1] and abs(error_y) < 0.05 * cv_image.shape[0]:
                            # Circula de vermelho se estiver quase no centro
                            cv2.circle(cv_image, (center_x, center_y), 30, (0, 0, 255), 3)
                        self.aruco_detected_callback(error_x, error_y, distance)

            aruco.drawDetectedMarkers(cv_image, corners, ids)



    def calculate_marker_position(self, cv_image, corner):
        # Calcula o centro do marcador ArUco
        center_x = int(sum(point[0] for point in corner[0]) / 4)
        center_y = int(sum(point[1] for point in corner[0]) / 4)
        center = (center_x, center_y)
        
        # Calcula o erro baseado no centro da imagem
        error_x = center_x - cv_image.shape[1] / 2
        error_y = center_y - cv_image.shape[0] / 2
        
        # Estima a distância até o marcador ArUco
        distance = self.estimate_distance_to_aruco(corner)
        return center_x, center_y, error_x, error_y, distance

    def estimate_distance_to_aruco(self, corner):
        # Estima a distância usando o tamanho conhecido do marcador e a área da imagem detectada.
        # Isso é uma simplificação e pode precisar ser ajustado para a sua configuração específica.
        marker_length_pixels = np.linalg.norm(corner[0][0] - corner[0][1])
        pixels_per_meter = marker_length_pixels / MARKER_SIZE
        distance = 1 / pixels_per_meter  # MARKER_SIZE é em metros
        return distance

    def fix_target(self, id):
        self.id_aruco_target = id

    def __del__(self):
        cv2.destroyAllWindows()
