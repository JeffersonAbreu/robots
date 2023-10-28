from sensor_msgs.msg import LaserScan
from rclpy.node import Node

class SensorLidar:

    def __init__(self, node: Node, obstacle_detected_callback=None):
        """
        Inicializa o Lidar.
        """
        self.node = node
        self.lidar_data: LaserScan = None
        self.lidar_sub = self.node.create_subscription(LaserScan, '/lidar', self.__lidar_callback, 10)
        self.obstacle_detected_callback = obstacle_detected_callback

    def __lidar_callback(self, msg: LaserScan):
        """
        Callback para dados do LiDAR.
        """
        self.lidar_data = msg
        if self.obstacle_detected_callback:
            self.obstacle_detected_callback(msg.ranges)

    def get_distance_to_obstacle(self) -> float:
        """
        Retorna a distância ao obstáculo mais próximo diretamente à frente do robô.
        """
        # caso o lidar for de 360º segue um exemplo:
        front_index = round(len(self.lidar_data.ranges)/2)
        return self.lidar_data.ranges[front_index]
        
