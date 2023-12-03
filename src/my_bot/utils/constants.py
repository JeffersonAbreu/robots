import numpy as np
import cv2.aruco as aruco

DISTANCE_TO_WALL = 0.5  
'''Distância da area de interece da parede'''
WALL_LARG = 0.05
'''Metade da largura da parede'''
TAG_WIDTH = 0.2
'''Largura da tag em metros'''
ANGLE_TAG_DOBLE = 30
'''Inclinação das TAGs que serão feitas dobradas'''


'''
Configurações do ArUco
'''
# Tamanho está para folha de papel A4 quadrada
MARKER_SIZE = TAG_WIDTH  # Tamanho do marcador em centímetros
ARUCO_DICT = aruco.DICT_4X4_100
'''
Dicionário: DICT_4X4_50, onde ?x? é a matriz e o numero apos é a quantidade de arucos que são possiveis de gerar
'''

'''
Configurações de simulação da câmera (valores fictícios, ajuste conforme necessário)
'''
CAMERA_MATRIX = np.array([[1000, 0, 320],
                          [0, 1000, 240],
                          [0, 0, 1]], dtype=float)
DIST_COEFFS = np.zeros((5, 1))  # Coeficientes de distorção da câmera

CALLBACK_INTERVAL = 0.5
'''Timer callback 0.5'''
CALLBACK_INTERVAL_TARGET = CALLBACK_INTERVAL / 2
CALLBACK_INTERVAL_TURN = 0.001
'''Verifica a cada 0.001 segundos'''
CALLBACK_INTERVAL_ACCELERATION = 0.2

# ROBO
# Curva
MIN_SPEED_TURN = 0.001
TOP_SPEED_TURN = 1.0
# Defina os limites de velocidade
TOP_SPEED = 1.0
MIN_SPEED = 0.1
ZERO = 0.0
WALL_COLID = 0.12
WALL_MIN_SECURITY = 0.3
'''Distancia minima já é uma colisão'''
 
LIDAR_RANGE = 13
LIDAR_RANGE_=LIDAR_RANGE-1


'''
TRACKING
'''
FACTOR_CORRECTION_TURN = 0.1
'''Usado para ajustar a velocidade da rotação'''
FACTOR_ACCELERATION = 0.001
FACTOR_BRAKING = -0.03