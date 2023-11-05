from sensor_msgs.msg import LaserScan
from rclpy.node import Node

class SensorLidar:

    def __init__(self, node: Node, obstacle_detected_callback):
        """
        Inicializa o Lidar.
        """
        self.node = node
        self.lidar_sub = self.node.create_subscription(LaserScan, '/lidar', self.__lidar_callback, 10)
        self.obstacle_detected_callback = obstacle_detected_callback

    def __lidar_callback(self, msg: LaserScan):
        """
        Callback para dados do LiDAR.
        """
        self.obstacle_detected_callback(msg.ranges)
        
