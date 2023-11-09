import numpy as np
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
import yaml
from .constants import ARUCO_DICT

class SensorCamera:
    '''
    matplotlib
    pandas
    numpy
    scikit-learn
    '''
    def __init__(self, node: Node, aruco_detected_callback):
        """
        Inicializa a câmera.
        :param marker_size: Tamanho do marcador em metros.
        :param camera_matrix: Matriz de câmera para a calibração da câmera.
        :param dist_coeffs: Coeficientes de distorção da câmera.
        """
        
        self.camera_calibration_yaml ='/home/jeff/tcc/robots/src/my_bot/utils/camera_config/camera_calibration.yaml'
        self.node = node
        self.aruco_detected_callback = aruco_detected_callback
        self.marker_size = 0
        self.camera_matrix = []
        self.dist_coeffs = []
        self.id_aruco_target = None
        self.lock = False
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        self.parameters = aruco.DetectorParameters_create()

        self.bridge = CvBridge()
        self.image_sub = self.node.create_subscription(Image, '/camera',  self.__image_callback, 10)

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

        # Se tags forem detectadas, desenhe-as na imagem e calcule a pose
        if ids is not None:
            ids = ids.flatten()  # Isso garante que ids seja um array 1D
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
            for i in range(len(ids)):
                id = ids[i]
                distance = np.linalg.norm(tvec[i][0])
                # Adiciona o ID e a distância na imagem
                aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec[i], tvec[i], self.marker_size * 0.5)  # Draw Axis for each marker

                if self.id_aruco_target is not None and id == self.id_aruco_target:
                    aruco_marker = corners[i]
                    marker_center = np.mean(aruco_marker[0], axis=0)
                    image_center = np.array([cv_image.shape[1]/2, cv_image.shape[0]/2])

                    color = (0, 255, 0)
                    cv2.putText(cv_image, f"Dist: {distance:.2f}m", (int(corners[i][0][0][0]), int(corners[i][0][0][1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    # Calcula o erro
                    error_x = (marker_center[0] - image_center[0]) * 0.01
                    error_y = (marker_center[1] - image_center[1]) * 0.01

                    self.aruco_detected_callback(error_x, error_y, distance)
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

    def load_calibration(self):
        with open(self.camera_calibration_yaml, 'r') as infile:
            calibration_data = yaml.safe_load(infile)
            self.camera_matrix = np.array(calibration_data['camera_matrix'])
            self.dist_coeffs = np.array(calibration_data['dist_coeffs'])
            self.marker_size = calibration_data['marker_size']

    def fix_target(self, id:int = None) -> None:
        self.id_aruco_target = id
        if id == None:
            self.lock = False


    def lock_target(self) -> None:
        self.lock = True