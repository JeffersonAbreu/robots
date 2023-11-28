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
        self.lidar_data_ranges = [10.0 for _ in range(271)] # range lidar, preencha com 10 para não ter erro

    def __lidar_callback(self, msg: LaserScan):
        """
        Callback para dados do LiDAR.
        """
        self.lidar_data_ranges = msg.ranges
        self.obstacle_detected_callback()
        

    def get_data_range(self, grau = 0):
        """
        Retorna a distância ao obstáculo mais próximo diretamente à frente do robô.
        """
        index = 135 + grau
        index = max(0, min(index, 270))
        value = self.lidar_data_ranges[index]
        try:
            value = float(value)
        except:
            print("Não é float. Ajustado para 10.0")
            value = 10.0
        return value
        
    def find_closest_wall_angle(self, direction=0):
        '''
        Retorna a menor distancia e o angulo
        Se for passado uma direcao negativa pega só a esqueda e vise-versa
        '''
        min_distance = float('inf')
        angle_of_min_distance = 0
        angles_range:[int] = range(len(self.lidar_data_ranges))
        if direction != 0:
            # Define o intervalo de ângulos para a esquerda ou direita 
            angles_range:[int] = range(134, 0) if direction > 0 else range(136, 270)

        # Percorre todos os ângulos e distâncias
        for angle in angles_range:
            distance = self.lidar_data_ranges[angle]
            if distance < min_distance:
                min_distance = distance
                angle_of_min_distance = angle
            else:
                break

        return angle_of_min_distance, min_distance

