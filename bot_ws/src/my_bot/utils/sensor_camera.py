from rclpy.node import Node
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class SensorCamera:

    def __init__(self, node: Node, aruco_detected_callback=None):
        """
        Inicializa o Lidar.
        """
        self.node = node
        self.aruco_detected_callback = aruco_detected_callback
        self.ids = None

        self.bridge = CvBridge()
        self.image_sub = self.node.create_subscription(Image, '/camera',  self.__image_callback, 10)
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        self.parameters = aruco.DetectorParameters_create()

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
        corners, ids, rejectedImgPoints = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)

        # Se tags forem detectadas, desenhe-as na imagem
        if ids is not None:
            aruco.drawDetectedMarkers(cv_image, corners, ids)
            self.ids = ids
            self.aruco_detected_callback()

        # Exibir a imagem com tags detectadas
        cv2.imshow('Aruco Detector', cv_image)
        cv2.waitKey(1)

    def get_aruco_dectected(self):
        return self.ids