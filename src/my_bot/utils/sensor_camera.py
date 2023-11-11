import numpy as np
import yaml
import os
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from .constants import ARUCO_DICT, MARKER_SIZE

class SensorCamera:
    def __init__(self, node: Node, aruco_detected_callback):
        self.node = node
        self.calibration_file_path = 'dados_calibracao.yaml'
        self.camera_matrix = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]], dtype=float)
        self.dist_coeffs = np.zeros(5)  # Supondo que não há distorção
        self.load_config_initial()
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
    
    def load_config_initial(self):
        # Verifica se o arquivo de calibração existe
        if os.path.isfile(self.calibration_file_path):
            try:
                # Abre o arquivo de calibração e carrega os dados
                with open(self.calibration_file_path, 'r') as arquivo:
                    calib_data = yaml.safe_load(arquivo)
                    # Verifica se as chaves esperadas existem no dicionário
                    if 'camera_matrix' in calib_data:
                        # Carrega os dados de calibração na matriz da câmera e nos coeficientes de distorção
                        self.camera_matrix = np.array(calib_data['camera_matrix'], dtype=float)
                    else:
                        raise KeyError("As chaves 'camera_matrix' não estão presentes no arquivo de calibração.")
                    if 'dist_coeffs' in calib_data:
                        self.dist_coeffs = np.array(calib_data['dist_coeffs'], dtype=float)
                    else:
                        raise KeyError("As chaves 'dist_coeffs' não estão presentes no arquivo de calibração.")
                    print(f"Dados de calibração carregados de {self.calibration_file_path}")
                    
            except yaml.YAMLError as e:
                self.get_logger().error(f"Erro ao ler o arquivo YAML: {e}")
            except KeyError as e:
                self.get_logger().error(f"{e}")
            except Exception as e:
                self.get_logger().error(f"Erro ao carregar o arquivo de configuração da câmera: {e}")
        else:
            self.get_logger().error(f"Arquivo de configuração da câmera não encontrado: {self.calibration_file_path}")
