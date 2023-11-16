import numpy as np
import yaml
import os
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from .constants import ARUCO_DICT, MARKER_SIZE # Tamanho real do marcador em metros (20cm)

class SensorCamera:
    def __init__(self, node: Node, aruco_detected_callback):
        self.node = node
        self.calibration_file_path = 'dados_calibracao.yaml'
        self.camera_matrix = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]], dtype=float)
        self.dist_coeffs = np.zeros(5)  # Supondo que não há distorção
        self.load_config_initial()
        self.aruco_detected_callback = aruco_detected_callback
        self.id_aruco_target = None
        self.id_aruco_target_lock = False
        self.aruco_dictionary = aruco.getPredefinedDictionary(ARUCO_DICT)
        self.parameters = aruco.DetectorParameters_create()
        self.bridge = CvBridge()
        self.image_sub = self.node.create_subscription(Image, '/camera', self.__image_callback, 10)

    def __image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.detect_and_decorate_arucos(cv_image)
        except CvBridgeError as e:
            self.node.get_logger().error(f'Could not convert image: {e}')

    def detect_and_decorate_arucos(self, cv_image):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dictionary, parameters=self.parameters)

        if ids is not None:
            for i, corner in enumerate(corners):
                id = ids[i][0]
                if id == self.id_aruco_target:
                    # Calcula o centro do marcador ArUco
                    center_aruco_x = int(np.mean(corner[0][:, 0]))  # Média das coordenadas X
                    center_aruco_y = int(np.mean(corner[0][:, 1]))  # Média das coordenadas Y
                    center_aruco = (center_aruco_x, center_aruco_y)

                    # Calcula o erro apenas no eixo X
                    error_x = center_aruco_x - cv_image.shape[1] // 2
                    # Se o ArUco estiver à esquerda do centro, a distância será negativa; se estiver à direita, será positiva
                    error_x = -error_x if center_aruco_x < cv_image.shape[1] // 2 else error_x

                    # Visualiza a informação
                    cv2.circle(cv_image, center_aruco, 5, (0, 0, 255), -1)
                    cv2.line(cv_image, center_aruco, (cv_image.shape[1] // 2, center_aruco_y), (0, 255, 0), 2)
                    cv2.putText(cv_image, f"Erro X: {error_x}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                    # Desenha os marcadores ArUco detectados
                    aruco.drawDetectedMarkers(cv_image, corners, ids)

                    self.aruco_detected_callback(error_x)

        # Exibe a imagem
        cv2.imshow('Visualização da Câmera', cv_image)




    def fix_target(self, id):
        self.id_aruco_target = id
        self.id_aruco_target_lock = False

    def set_lock(self):
        self.id_aruco_target_lock = not self.id_aruco_target_lock

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

    
    def print_in_imagem(self, img, msg):
        cv2.putText(img, msg, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)