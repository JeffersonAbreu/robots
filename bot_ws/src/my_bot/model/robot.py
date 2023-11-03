from geometry_msgs.msg import Twist
from utils import CommandType, Command, SensorIMU, SensorLidar, SensorOdom, SensorCamera
from utils.bib import degrees_to_radians, normalize_angle2, radians_to_degrees, normalize_angle_degrees
from rclpy.node import Node

# Constantes de Velocidade
MIN_SPEED = 0.001
TOP_SPEED = 1.0

# Funções Auxiliares
def get_angular_speed(error: float) -> float:
    """
    Calcula a velocidade angular com base no erro.
    """
    Kp = 1.0  # Este é o coeficiente proporcional. Ajuste conforme necessário.
    angular_speed = Kp * error

    # Limitando a velocidade angular
    if abs(angular_speed) < MIN_SPEED:
        angular_speed = MIN_SPEED if angular_speed > 0 else -MIN_SPEED
    elif abs(angular_speed) > TOP_SPEED:
        angular_speed = TOP_SPEED if angular_speed > 0 else -TOP_SPEED

    # Desaceleração conforme se aproxima do destino
    DECELERATION_THRESHOLD = degrees_to_radians(0.25)  # Ajuste conforme necessário. Representa a diferença de ângulo em radianos.
    if abs(error) < DECELERATION_THRESHOLD:
        angular_speed *= (abs(error) / DECELERATION_THRESHOLD)

    return angular_speed

def get_angular_speed2(error: float) -> float:
        '''
        Limitar a velocidade angular para evitar viradas muito rápidas
        '''
        angular_speed = max(min(degrees_to_radians(error), 2.0), -2.0)
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

        return angular_speed
            


# Classe Principal

class Robot:

    def __init__(self, node: Node, handle_obstacle_detection, handle_aruco_detected):
        """
        Inicializa o robô.
        """
        self.node = node
        self.state = None
        self.orientation:int = 0
        self.initial_orientation = 0.0
        self.current_orientation = 0.0
        self.destiny_orientation = 0.0
        self.distance = 0.0
        self.running_the_command = False
        self.twist = Twist()
        # Publicadores e Subscritores
        self.twist_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.sensor_imu: SensorIMU = SensorIMU(node)
        self.sensor_odom: SensorOdom = SensorOdom(node)
        self.sensor_lidar: SensorLidar = SensorLidar(node, handle_obstacle_detection)
        self.sensor_camera: SensorCamera = SensorCamera(node, handle_aruco_detected)

    def get_time_now(self):
        return self.node.get_clock().now().seconds_nanoseconds()[0]
    
    def get_distance_to_obstacle(self) -> float:
        """
        Retorna a distância ao obstáculo mais próximo diretamente à frente do robô.
        """
        return self.sensor_lidar.get_distance_to_obstacle()
    
    def get_aruco_dectected(self):
        """
        Retorna o que foi detectado pela camera
        """
        return self.sensor_camera.get_aruco_dectected()
    
    def get_orientation(self) -> float:
        """
        Calcule e retorne a orientação do sensor IMU. (angle z) em graus.
        """
        return self.sensor_imu.get_orientation()
    
    # Movimentação
    def move_forward(self, speed=0.5):
        """
        Move o robô para frente. Se a velocidade não for especificada, use 0.3 como padrão.
        """
        self.twist.linear.x = speed
        self.twist_pub.publish(self.twist)
        self.state = CommandType.MOVE_FORWARD
        self.running_the_command = False

    def move_backward(self, distance: float):
        """
        Move o robô para trás por uma distância específica.
        """
        self.move_distance(distance=distance, speed=-0.3)
        
    def move_distance(self, distance: float = 0.0, speed: float = 0.5):
        """
        Move o robô por uma distância específica.
        """
        self.sensor_odom.reset_odom()
        self.distance = distance #if speed > 0 else -distance
        self.state = CommandType.MOVE_FORWARD if speed > 0 else CommandType.MOVE_BACKWARD
        self.twist.linear.x = speed
        self.twist_pub.publish(self.twist)
        self.move_timer = self.node.create_timer(0.01, self.__move_timer_callback)

    def __move_timer_callback(self):
        travelled_distance = self.sensor_odom.get_travelled_distance()
        if self.distance - travelled_distance <= 0.001:
            self.node.get_logger().info(f'distance: {self.distance} | travelled: {round(travelled_distance, 3)}')
            self.stop()
            self.move_timer.cancel()


    def stop(self):
        """
        Para o robô.
        """
        self.twist = Twist()
        self.twist_pub.publish(self.twist)
        self.state = CommandType.STOP
        self.running_the_command = False
        self.node.get_logger().warning(f'STOP')

    # Outros
    def get_state(self):
        """
        Retorna o estado atual do robô.
        """
        return self.state
    
    # Rotação

    def __turn(self, speed):
        """
        Define a velocidade de rotação.
        """
        self.twist.angular.z = speed
        self.twist_pub.publish(self.twist)

    def turn_by_angle(self, angle:int):
        """
        Gira o robô por um ângulo específico em graus.
        """
        self.orientation += angle

        self.state = CommandType.TURN
        self.initial_orientation = round(normalize_angle_degrees(self.get_orientation()))
        self.destiny_orientation = normalize_angle2(self.orientation)
        #self.destiny_orientation = normalize_angle2(self.initial_orientation + angle)
        self.turn_timer = self.node.create_timer(0.001, self.__turn_timer_callback)  # Verifica a cada 0.001 segundos
        
        self.log_info = False
    
    
    def __turn_timer_callback(self):
        self.current_orientation = normalize_angle2(self.get_orientation())
        # Calcula a diferença entre a orientação atual e a desejada
        difference = normalize_angle2(self.destiny_orientation - self.current_orientation)
        
        # Se a diferença for pequena o suficiente, pare o robô
        if abs(difference) <= 0.15:
            self.stop()
            self.turn_timer.cancel()
        else:
            # Caso contrário, continue girando na direção mais curta para alcançar a orientação desejada
            angular_speed = get_angular_speed(degrees_to_radians(difference))
            self.__turn(angular_speed)
            if abs(difference) <= 5.0 and self.log_info:
                self.node.get_logger().info(f'Current: {round(self.current_orientation, 3)} | Distance: {round(abs(difference), 3)} | Destiny: {self.destiny_orientation}')

   
    def execute(self, command: Command) -> None:
        self.running_the_command = True
        if command.type == CommandType.MOVE_FORWARD:
            if command.value == None:
                self.move_forward()
            else:
                self.move_distance(distance=command.value)
        elif command.type == CommandType.MOVE_BACKWARD:
            self.move_backward(command.value)
        elif command.type == CommandType.TURN:
            self.turn_by_angle(command.value)
        elif command.type == CommandType.STOP:
            self.stop()
        
        self.node.get_logger().info(f'Command: {command.type}')
    
    def is_command_running(self) -> bool:
        return self.running_the_command