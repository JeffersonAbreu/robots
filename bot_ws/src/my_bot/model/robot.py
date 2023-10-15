from geometry_msgs.msg import Twist
from utils import CommandType, SensorIMU, SensorLidar, SensorOdom
from utils.bib import degrees_to_radians, normalize_angle
from rclpy.node import Node

# Constantes de Velocidade
MIN_SPEED = 0.1
TOP_SPEED = 2.0
SPEED = 0.0

# Funções Auxiliares

def get_angular_speed(error: float) -> float:
    """
    Calcula a velocidade angular com base no erro.
    """
    Kp = 1.0
    angular_speed = Kp * error

    # Limitando a velocidade angular
    if abs(angular_speed) < MIN_SPEED:
        angular_speed = MIN_SPEED if angular_speed > 0 else -MIN_SPEED
    elif abs(angular_speed) > TOP_SPEED:
        angular_speed = TOP_SPEED if angular_speed > 0 else -TOP_SPEED

    return angular_speed

# Classe Principal

class Robot:

    def __init__(self, node: Node, handle_obstacle_detection):
        """
        Inicializa o robô.
        """
        self.node = node
        self.state = None
        self.initial_orientation = 0.0
        self.current_orientation = 0.0
        self.destiny_orientation = 0.0
        self.initial_position = 0.0
        self.move_destiny = 0.0

        # Publicadores e Subscritores
        self.twist_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.sensor_imu: SensorIMU = SensorIMU(node)
        self.sensor_odom: SensorOdom = SensorOdom(node)
        self.sensor_lidar: SensorLidar = SensorLidar(node, handle_obstacle_detection)

    def get_time_now(self):
        return self.node.get_clock().now().seconds_nanoseconds()[0]
    
    def get_distance_to_obstacle(self) -> float:
        """
        Retorna a distância ao obstáculo mais próximo diretamente à frente do robô.
        """
        return self.sensor_lidar.get_distance_to_obstacle()
    
    def get_orientation(self) -> float:
        """
        Calcule e retorne a orientação do sensor IMU. (angle z)
        """
        return self.sensor_imu.get_orientation()
    # Movimentação

    def move_forward(self, speed=0.3):
        """
        Move o robô para frente. Se a velocidade não for especificada, use 0.3 como padrão.
        """
        twist = Twist()
        twist.linear.x = speed
        self.twist_pub.publish(twist)
        self.state = CommandType.MOVE_FORWARD

    def move_backward(self, distance: float):
        """
        Move o robô para trás por uma distância específica.
        """
        self.move_distance(distance=distance, speed=-0.3)
        
    def move_distance(self, distance: float, speed: float = 0.3):
        """
        Move o robô por uma distância específica.
        """
        SPEED = speed
        initial_position = self.sensor_odom.get_position()
        distance = distance if SPEED > 0 else -distance
        self.move_destiny = distance + initial_position 
        self.move_timer = self.node.create_timer(0.01, self.__move_timer_callback)

    def __move_timer_callback(self):
        current_position = self.sensor_odom.get_position()
        if current_position - self.move_destiny <= 0.001:
            self.node.get_logger().info(f'position: x: {self.sensor_odom.get_position()}')
            self.move_timer.cancel()
            self.stop()
        else:
            self.move_forward(SPEED)


    def curve(self, angle: float):
        """
        Começa a curvar o robô por um ângulo específico.
        """
        # Definir velocidade para a frente
        speed = 0.3
        self.move_forward(speed)

        # Começar a curvar
        self.turn_by_angle(angle)
        self.state = CommandType.CURVE

    def stop(self):
        """
        Para o robô.
        """
        twist = Twist()
        self.twist_pub.publish(twist)
        self.state = CommandType.STOP

    # Rotação

    def __turn(self, speed):
        """
        Define a velocidade de rotação.
        """
        twist = Twist()
        twist.angular.z = speed
        self.twist_pub.publish(twist)

    def turn_by_angle(self, angle):
        """
        Gira o robô por um ângulo específico.
        """
        self.state = CommandType.TURN
        self.initial_orientation = self.current_orientation = self.sensor_imu.get_orientation()
        self.destiny_orientation = self.current_orientation + degrees_to_radians(angle)
        self.turn_timer = self.node.create_timer(0.1, self.__turn_timer_callback)  # Verifica a cada 0.1 segundos
    
    # Outros
    def get_state(self):
        """
        Retorna o estado atual do robô.
        """
        return self.state
    
    def __turn_timer_callback(self):
        self.current_orientation = self.sensor_imu.get_orientation()
        
        if abs(self.current_orientation - self.destiny_orientation) <= 0.001:
            self.stop()
            self.turn_timer.cancel()
        else:
            difference = self.destiny_orientation - self.current_orientation
            # Limitar a velocidade angular para evitar viradas muito rápidas
            angular_speed = max(min(difference, 2.0), -2.0)
            diff = abs(angular_speed)
            if diff < 0.05:
                if diff > 0.00:
                    angular_speed =  angular_speed * 0.75 # speed: 75%
                else:
                    if diff > 0.005:
                        angular_speed = angular_speed * 0.5 # speed: 50%
                    else:
                        if diff > 0.0005:
                            angular_speed = angular_speed * 0.25 # speed: 25%
                        else:
                            angular_speed = angular_speed * 0.1 # speed: 10%
    
            self.__turn(angular_speed)    
            