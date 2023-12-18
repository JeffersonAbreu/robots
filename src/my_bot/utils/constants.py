import numpy as np
import cv2.aruco as aruco
# ================================================================================
# TAG 
# ================================================================================
DISTANCE_TO_WALL = 0.5  
'''Distância da area de interece da parede'''
WALL_LARG = 0.05
'''Metade da largura da parede'''
TAG_WIDTH = 0.2
'''Largura da tag em metros'''
ANGLE_TAG_DOBLE = 15
'''Inclinação das TAGs que serão feitas dobradas'''


'''
================================================================
Configurações do ArUco 
================================================================
'''
# Tamanho está para folha de papel A4 quadrada
MARKER_SIZE = 0.2  # Tamanho do marcador em centímetros
ARUCO_DICT = aruco.DICT_4X4_100
'''
Dicionário:  DICT_4X4_100
- Onde ?x? é a matriz
- E o numero apos é a quantidade de arucos que são possiveis de gerar
'''

'''
==========================================================================================
Configurações de simulação da câmera (valores fictícios, ajuste conforme necessário) 
==========================================================================================
'''
CAMERA_ANGLE_VISION = 60 # 60 ou 90, lembre de trocar a camera no modelo para a específica

CALLBACK_INTERVAL = 0.5
'''Timer callback 0.5'''
CALLBACK_INTERVAL_TARGET = 2.5
CALLBACK_INTERVAL_TURN = 0.001
'''Verifica a cada 0.001 segundos'''
CALLBACK_INTERVAL_ACCELERATION = 0.1

# ================================================================================
# ROBO 
# ================================================================================
# Curva
MIN_SPEED_TURN = 0.001
''' Defina os limites de MIN velocidade da ginada'''
TOP_SPEED_TURN = 1.0
''' Defina os limites de MAX velocidade da ginada'''
TOP_SPEED = 1.0
''' Defina os limites de MAX velocidade'''
MIN_SPEED = 0.1
''' Defina os limites de MIN velocidade'''
ZERO = 0.0
WALL_COLID = 0.12
WALL_MIN_SECURITY = 0.3
'''Distancia minima já é uma colisão'''
 
LIDAR_RANGE = 13
LIDAR_RANGE_=LIDAR_RANGE-1


# ================================================================================
#TRACKING - RESTREAMENTO 
# ================================================================================
FACTOR_CORRECTION_TURN = 0.1
'''Usado para ajustar a velocidade da rotação'''
FACTOR_ACCELERATION = 0.0025
FACTOR_BRAKING = -0.03

LOG_LINE = False

DISTANCE_AREA_OF_INTEREST = 0.9