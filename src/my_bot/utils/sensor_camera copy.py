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
        self.camera_matrix = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]])
        self.dist_coeffs = np.zeros((5, 1))
        # Definir a distância focal da câmera. Substitua 1000.0 pelo seu valor calibrado
        self.focal_length = 1000.0
        self.marker_size = MARKER_SIZE / 10
        self.load_config_initial()
        self.aruco_detected_callback = aruco_detected_callback
        self.id_aruco_target = None
        self.id_aruco_target_lock = False
        self.aruco_dictionary = aruco.getPredefinedDictionary(ARUCO_DICT)
        self.parameters = aruco.DetectorParameters_create()
        self.bridge = CvBridge()
        self.image_sub = self.node.create_subscription(Image, '/camera', self.__image_callback, 10)

    def __image_callback(self, data):
        """
        Callback da imagem da camera
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.node.get_logger().error('Could not convert image: %s' % e)
            return

         # Detecção das tags ArUco
        corners, ids, _ = aruco.detectMarkers(cv_image, self.aruco_dictionary, parameters=self.parameters)
        error_x = 0
        # Se tags forem detectadas, desenhe-as na imagem e calcule a pose
        if ids is not None:
            ids = ids.flatten()  # Isso garante que ids seja um array 1D
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
            for i in range(len(ids)):
                id = ids[i]
                # Adiciona o ID e a distância na imagem
                aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec[i], tvec[i], self.marker_size * 0.5)  # Draw Axis for each marker

                if self.id_aruco_target is not None and id == self.id_aruco_target:
                    aruco_marker = corners[i]
                    marker_center = np.mean(aruco_marker[0], axis=0)
                    image_center = np.array([cv_image.shape[1]/2, cv_image.shape[0]/2])

                    # Calcula o erro
                    error_x = (marker_center[0] - image_center[0]) * 0.01
                    
                    color = (255,0,255) # rosa pink
                    if self.lock:
                        color = (0, 0, 255) # vermelho
                    cv2.polylines(cv_image, [corners[i].astype(np.int32)], True, color, 2)
                else:
                    # Para todos os outros marcadores, desenhe com a cor padrão (verde)
                    aruco.drawDetectedMarkers(cv_image, [corners[i]], [ids[i]])



        # Exibir a imagem com tags detectadas
        cv2.namedWindow('Aruco Detector', cv2.WINDOW_NORMAL)  # Cria uma janela que pode ser redimensionada
        cv2.resizeWindow('Aruco Detector', 800, 600)  # Define o novo tamanho da janela para 800x600

        cv2.imshow('Aruco Detector', cv_image)
        cv2.waitKey(1)
        self.aruco_detected_callback(error_x)

    def detect_and_decorate_arucos(self, cv_image):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dictionary, parameters=self.parameters)
        error_x = 0

        if ids is not None and len(corners) > 0:
            for i, corner in enumerate(corners):
                id = ids[i][0]
                corner = np.array(corner).reshape((-1, 2))
                if id == self.id_aruco_target:
                    # Desenha uma mira no ArUco alvo
                    cv2.drawContours(cv_image, [corner], -1, (0, 0, 255), 2)
                    # Calcula e exibe o erro
                    center_aruco = tuple(corner.mean(axis=0).astype(int))
                    error_x = center_aruco[0] - cv_image.shape[1] // 2
                    self.aruco_detected_callback(error_x)
                else:
                    # Desenha todos os outros ArUcos de verde limão
                    cv2.drawContours(cv_image, [corner], -1, (50, 205, 50), 2)
                
                # Mostra o ID de todos os marcadores à direita deles na cor rosa
                text_position = (corner[0][0] + 10, corner[0][1])
                cv2.putText(cv_image, f"ID: {id}", text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 105, 180), 2)

        # Exibe a imagem
        cv2.imshow('Visualização da Câmera', cv_image)
        cv2.waitKey(1)  # Atualiza a imagem a cada milissegundo

        return error_x  # Retorna a distância do erro em pixels



    def fix_target(self, id):
        self.id_aruco_target = id
        self.id_aruco_target_lock = False

    def set_lock(self):
        self.id_aruco_target_lock = True

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