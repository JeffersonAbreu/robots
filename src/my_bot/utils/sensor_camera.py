import numpy as np
import yaml
import os
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from .constants import CAMERA_MATRIX, DIST_COEFFS, ARUCO_DICT, MARKER_SIZE # Tamanho real do marcador em metros (20cm)

# Parâmetros da câmera
#FOV_WIDTH = np.degrees(1.047)  # Campo de visão horizontal em graus, convertido de radianos
ANGLE = 90
'''
Opções do angulo da camera:
- 90°
- 60°
'''
RAD_90 = 1.5708
RAD_60 = 1.047
RAD = RAD_60 if ANGLE == 60 else RAD_90
FOV_WIDTH = np.degrees(RAD)
IMAGE_WIDTH = 640  # Largura da imagem em pixels
text_color = (0, 0, 0)  # Preto para texto
blue_color = (255, 0, 0)  # Azul para resultados positivos
red_color  = (0, 0, 255)  # Vermelho para resultados negativos

def calculate_rotation_angle(pixel_error, image_width, fov_width):
    pixels_to_degrees = fov_width / image_width
    rotation_angle = pixel_error * pixels_to_degrees
    return rotation_angle

class SensorCamera:
    def __init__(self, node: Node, aruco_detected_callback):
        """
        Inicializa a câmera.
        :param marker_size: Tamanho do marcador em metros.
        :param camera_matrix: Matriz de câmera para a calibração da câmera.
        :param dist_coeffs: Coeficientes de distorção da câmera.
        """
        self.camera_calibration_yaml =f'support/dados_calibracao{ANGLE}.yaml'
        self.node = node
        self.aruco_detected_callback = aruco_detected_callback
        self.marker_size    = None
        self.camera_matrix  = None
        self.dist_coeffs    = None
        self.load_calibration()
        self.id_aruco_target = None
        self.track_aruco_target = False
        self.aruco_dict = aruco.getPredefinedDictionary(ARUCO_DICT)
        self.parameters = aruco.DetectorParameters_create()
        self.bridge = CvBridge()
        self.image_sub = self.node.create_subscription(Image, '/camera', self.__image_callback, 10)

    def __image_callback(self, data):
        """
        Callback da imagem da camera
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            cv_image, distance_to_aruco, rotation_angle = self.detect_and_decorate_arucos2(cv_image)
            self.aruco_detected_callback(distance_to_aruco, round(rotation_angle, 2))
            # Exibe informações na imagem
            
            cv_image = self.display_info(cv_image)       
            cv_image = self.draw_line_center_x(cv_image)
            # Exibe a imagem com o ArUco detectado e a linha central
            cv2.imshow('Aruco Detector', cv_image)
            cv2.waitKey(1)  # Atualiza a janela de exibição
        except CvBridgeError as e:
            self.node.get_logger().error('Could not convert image: %s' % e)

    
    def is_target_marker(self, marker_id):
        if self.id_aruco_target is None:
            return False
        return self.id_aruco_target == marker_id

    def fix_target(self, marker_id):
        self.id_aruco_target = marker_id
    
    def load_calibration(self):
        with open(self.camera_calibration_yaml, 'r') as infile:
            calibration_data = yaml.safe_load(infile)
            self.camera_matrix = np.array(calibration_data['camera_matrix'])
            self.dist_coeffs = np.array(calibration_data['dist_coeffs'])
            self.marker_size = float(calibration_data['marker_size'][0])

    def draw_line_center_x(self, cv_image):
        """
        Desenha o contorno dos marcadores ArUco detectados e uma linha vertical no centro da tela.
        """
        # Desenha uma linha vertical no centro da tela
        image_center_x = cv_image.shape[1] // 2
        cv2.line(cv_image, (image_center_x, 0), (image_center_x, cv_image.shape[0]), blue_color, 1)
        return cv_image

    def calculate_pixel_error(self, corners, image_width):
        """
        Calcula a diferença em pixels entre o centro do marcador ArUco e o centro da tela.
        """
        marker_center_x = int(np.mean(corners[0], axis=0)[0])
        image_center_x = image_width // 2
        pixel_error = marker_center_x - image_center_x
        return pixel_error

    def calculate_distance(self, corners, cv_image):
        """
        Estima a distância da câmera até o marcador ArUco.
        """
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
        # Desenha o eixo para o marcador ArUco
        cv2.aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec[0], tvec[0], self.marker_size * 0.5)
        distance_to_aruco = np.linalg.norm(tvec[0])
        return distance_to_aruco
    
    def display_info(self, img):
        """
        Escreve na tela as informações sobre a distância, o erro de pixel e o ângulo de rotação.
        """
        img = cv2.putText(img, f"{'ID: ':>8}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)
        if self.id_aruco_target is not None:
            img = cv2.putText(img, f"{str(self.id_aruco_target):>2}", (80, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, blue_color, 2)
        return img
    


    def highlight_target_aruco(self, cv_image, corner, target_id):
        """
        Destaca o marcador ArUco com o ID alvo.
        """
        if target_id == self.id_aruco_target:
            # Destaca o marcador ArUco com uma borda mais espessa
            if self.track_aruco_target:
                cv2.polylines(cv_image, [np.int32(corner)], True, (0, 255, 0), 1)
            else:
                cv2.polylines(cv_image, [np.int32(corner)], True, (0, 0, 255), 1)
        else:
            cv2.polylines(cv_image, [np.int32(corner)], True, (0, 255, 255), 1)
        return cv_image

    
    def detect_and_decorate_arucos2(self, cv_image):
        """
        Detecta marcadores ArUco, calcula a distância até eles e o erro de pixel, e desenha na imagem.
        """
        distance_to_aruco=0
        rotation_angle=0
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray_image, self.aruco_dict, parameters=self.parameters, cameraMatrix=self.camera_matrix, distCoeff=self.dist_coeffs)
        # Isto deve ser um array 3D de cantos detectados
        corners = np.array(corners).astype(float)

        if ids is not None and len(corners) > 0:
            ids = ids.flatten()
            
            for i, corner in enumerate(corners):
                if self.is_target_marker(ids[i]):
                    # Calcula a distância até o marcador ArUco
                    distance_to_aruco = self.calculate_distance(corner, cv_image)
                    # Calcula o erro em pixels
                    image_width = cv_image.shape[1]
                    pixel_error = self.calculate_pixel_error(corner, image_width)
                    rotation_angle = calculate_rotation_angle(pixel_error, IMAGE_WIDTH, FOV_WIDTH)
                cv_image = self.highlight_target_aruco(cv_image, corner, ids[i])
        return cv_image, distance_to_aruco, rotation_angle
