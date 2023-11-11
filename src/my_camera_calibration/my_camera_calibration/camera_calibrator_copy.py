import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import yaml

class CameraCalibrator(Node):
    def __init__(self, camera_matrix, dist_coeffs):
        super().__init__('camera_calibrator')
        self.bridge = CvBridge()
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
        self.parameters = aruco.DetectorParameters_create()
        self.marker_length = 0.2  # size of the marker side in meters

        self.subscription = self.create_subscription(Image, '/camera', self.image_callback, 10)
        self.calibration_data = []
        self.last_tvecs = None
        self.last_rvecs = None
        self.last_ids = None

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Redimensionar a imagem para 480p se necessário
        cv_image = cv2.resize(cv_image, (640, 480))

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            # Estima a pose do(s) marcador(es) ArUco
            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
            self.last_tvecs = tvecs
            self.last_rvecs = rvecs
            self.last_ids = ids
            for i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
                # Desenhar o sistema de coordenadas do marcador ArUco na imagem
                aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
                
                # A distância será a norma do vetor de translação
                distance = np.linalg.norm(tvec)

                # Supondo que o marcador ArUco é um quadrado com o tamanho conhecido
                size_estimate = self.marker_length / np.linalg.norm(corners[i][0][0] - corners[i][0][1])
                
                # Preparar o texto para exibição
                info_text = f"ID: {ids[i][0]} Distance: {distance:.2f}m Size: {size_estimate:.2f}m"

                # Exibir distância e tamanho estimado na janela do OpenCV
                cv2.putText(cv_image, info_text,
                            (10, 30 * (i + 1)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2, cv2.LINE_AA)


        cv2.imshow('Camera Calibration', cv_image)
        key = cv2.waitKey(1)
        if key == ord('g') and self.last_ids is not None:
            self.save_calibration_data()

    def run_calibration(self):
        print("Press 'C' to capture calibration data, 'G' to save and exit.")
        while rclpy.ok():
            rclpy.spin_once(self)
            key = cv2.waitKey(1)

            if key == ord('c'):
                print("Calibration data captured.")
                # Aqui você pode implementar a lógica para capturar os dados de calibração
                # ...

            if key == ord('g'):
                self.save_calibration_data()
                print("Calibration data saved. Exiting.")
                break

        cv2.destroyAllWindows()

    def save_calibration_data(self):
        # Salva os dados de calibração em um arquivo YAML
        calibration_data = {
            'tvecs': self.last_tvecs.tolist(),
            'rvecs': self.last_rvecs.tolist(),
            'ids': self.last_ids.flatten().tolist()
        }
        with open('camera_calibration.yaml', 'w') as f:
            yaml.dump(calibration_data, f, default_flow_style=False)
        print("Calibration data saved.")
        # Não esqueça de zerar os dados após salvar
        self.last_tvecs = None
        self.last_rvecs = None
        self.last_ids = None

def main(args=None):
    rclpy.init(args=args)

    # Valores de exemplo para os parâmetros intrínsecos da câmera
    # Estes valores devem ser substituídos pelos resultados da sua calibração da câmera
    camera_matrix = np.array([[1000, 0, 320],
                              [0, 1000, 240],
                              [0,    0,   1]], dtype=float)
    dist_coeffs = np.zeros((5, 1))  # Supondo que não há distorção

    calibrator = CameraCalibrator(camera_matrix, dist_coeffs)
    calibrator.run_calibration()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
