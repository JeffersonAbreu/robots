
import cv2.aruco as aruco

ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
MARKER_SIZE = 500
SCALE = 0.5
DISTANCE_TO_WALL = 0.5  # Distância da area de interece da parede
WALL_LARG = 0.05