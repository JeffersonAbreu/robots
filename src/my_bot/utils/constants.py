import numpy as np
import cv2.aruco as aruco

# Constantes

DISTANCE_TO_WALL = 0.5  # Distância da area de interece da parede
WALL_LARG = 0.05

# Configurações do ArUco
ARUCO_DICT = aruco.DICT_4X4_100  # Substitua pelo dicionário desejado
MARKER_SIZE = 20  # Tamanho do marcador em centímetros

'''
Tamanho está para folha de papel A4 quadrada
'''

# Configurações de simulação da câmera (valores fictícios, ajuste conforme necessário)
CAMERA_MATRIX = np.array([[1000, 0, 320],
                          [0, 1000, 240],
                          [0, 0, 1]], dtype=float)
DIST_COEFFS = np.zeros((5, 1))  # Coeficientes de distorção da câmera