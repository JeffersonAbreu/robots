import numpy as np
import yaml
import os
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from .constants import ARUCO_DICT, MARKER_SIZE # Tamanho real do marcador em metros (20cm)
PIXELS_TO_DEGREES = 0.1  # exemplo: 10 pixels equivalem a 1 grau
FOV_WIDTH = 60.0  # Campo de visão horizontal de 60 graus

class SensorCamera:
    def __init__(self, node: Node, aruco_detected_callback):
        """
        Inicializa a câmera.
        :param marker_size: Tamanho do marcador em metros.
        :param camera_matrix: Matriz de câmera para a calibração da câmera.
        :param dist_coeffs: Coeficientes de distorção da câmera.
        """
        
        self.camera_calibration_yaml ='support/dados_calibracao.yaml'
        self.node = node
        self.aruco_detected_callback = aruco_detected_callback
        self.marker_size = MARKER_SIZE
        self.camera_matrix = []
        self.dist_coeffs = []
        self.load_calibration()
        self.id_aruco_target = None
        self.id_aruco_target_lock = False
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
            cv_image = self.track_aruco(cv_image)
        except CvBridgeError as e:
            self.node.get_logger().error('Could not convert image: %s' % e)
            return

    def detect_and_decorate_arucos(self, cv_image):
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        #corners, ids, _ = aruco.detectMarkers(gray_image, self.aruco_dict, parameters=self.parameters)
        corners, ids, _ = cv2.aruco.detectMarkers(gray_image, self.aruco_dict, parameters=self.parameters, cameraMatrix=self.camera_matrix, distCoeff=self.dist_coeffs)

        if ids is not None and len(corners) > 0:
            cv2.aruco.drawDetectedMarkers(cv_image, corners)
            ids = ids.flatten()
            for i, corner in enumerate(corners):
                if self.is_target_marker(ids[i]):
                    marker_center = self.calculate_center(corner)
                    frame_center = cv_image.shape[1] / 2.0
                    erro = marker_center - frame_center
                    return erro, cv_image
        return None, cv_image


    def is_target_marker(self, marker_id):
        if self.id_aruco_target is None:
            return False
        return self.id_aruco_target == marker_id

    def calculate_center(self, corners):
        return (np.mean(corners, axis=0).astype(int).flatten())[0]

    def fix_target(self, marker_id):
        self.id_aruco_target = marker_id
        self.id_aruco_target_lock = False
        self.node.get_logger().error(f'Novo alvo marcado ID: {marker_id:>2}')
    
    def set_lock(self):
        self.id_aruco_target_lock = True

    def load_calibration(self):
        with open(self.camera_calibration_yaml, 'r') as infile:
            calibration_data = yaml.safe_load(infile)
            self.camera_matrix = np.array(calibration_data['camera_matrix'])
            self.dist_coeffs = np.array(calibration_data['dist_coeffs'])
            self.marker_size = calibration_data['marker_size']
    
    def pose_esitmation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
        '''
        frame - Frame from the video stream
        matrix_coefficients - Intrinsic matrix of the calibrated camera
        distortion_coefficients - Distortion coefficients associated with your camera

        return:-
        frame - The frame with the axis drawn on it
        '''

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
        parameters = cv2.aruco.DetectorParameters_create()


        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters,
            cameraMatrix=matrix_coefficients,
            distCoeff=distortion_coefficients)

            # If markers are detected
        if len(corners) > 0:
            for i in range(0, len(ids)):
                # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,
                                                                        distortion_coefficients)
                # Draw a square around the markers
                cv2.aruco.drawDetectedMarkers(frame, corners) 

                # Draw Axis
                cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)  

        return frame
    
    def track_aruco(self, cv_image):
        """
        Rastreia o marcador ArUco e envia comandos para o robô girar em direção a ele.
        """
        pixel_difference, cv_image = self.detect_and_decorate_arucos(cv_image)

        if pixel_difference is not None:
            # Converte a diferença de pixels para graus
            degrees_per_pixel = FOV_WIDTH / cv_image.shape[1]
            # Determina o ângulo para o comando baseado na direção
            angle_difference = pixel_difference * degrees_per_pixel            
            # Emite o comando para girar o robô em direção ao ArUco
            self.aruco_detected_callback(angle_difference)

        return cv_image