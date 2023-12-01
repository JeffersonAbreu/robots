import cv2
import cv2.aruco as aruco
import numpy as np
import glob
import yaml

# Classe para calibrar a câmera usando marcadores ArUco
class CameraCalibrador():
    def __init__(self):
        # Listas para armazenar os cantos dos marcadores e seus IDs
        self.all_corners = []
        self.all_ids = []
        self.obj_points = []
        self.calibration_flags = cv2.CALIB_RATIONAL_MODEL
        self.calibration_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
        self.marker_length = 0.2  # Tamanho do marcador ArUco em metros

        # Preparar pontos de objeto 3D
        self.objp = np.zeros((4, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.marker_length:2j, 0:self.marker_length:2j].reshape(-1, 2)

    def calibration(self):
        # Tamanho da imagem capturada
        image_size = None

        # Iterar sobre as imagens para encontrar marcadores ArUco
        for image_path in glob.glob('/home/jeff/tcc/robots/images/*.png'):
            img = cv2.imread(image_path)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict)
            
            if ids is not None:
                # Refinamento subpixel dos cantos
                corners = aruco.refineDetectedMarkers(gray, None, corners, ids, rejectedImgPoints)
                self.all_corners.append(corners)
                self.all_ids.append(ids)
                self.obj_points.append(self.objp)
                image_size = gray.shape[::-1]

        # Verifica se foi possível capturar algum ponto de imagem
        if image_size is None or len(self.all_corners) == 0:
            raise Exception("Não foi possível calibrar porque nenhuma imagem contém marcadores ArUco.")

        # Calibrar a câmera com os pontos coletados
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraAruco(
            self.all_corners, self.all_ids, self.obj_points, self.aruco_dict, image_size, None, None)

        # Salvar os dados de calibração em um arquivo
        calibration_data = {'camera_matrix': camera_matrix.tolist(), 'dist_coeffs': dist_coeffs.tolist()}
        with open('calibration_data.yaml', 'w') as f:
            yaml.dump(calibration_data, f)

        # Exibe os resultados da calibração
        print("Calibração concluída")
        print("Matriz da câmera:", camera_matrix)
        print("Coeficientes de distorção:", dist_coeffs)

# Ponto de entrada do script
if __name__ == '__main__':
    calibrador = CameraCalibrador()
    calibrador.calibration()
