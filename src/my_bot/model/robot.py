from geometry_msgs.msg import Twist
from utils import CommandType, Orientation, Command, SensorIMU, SensorLidar, SensorOdom, SensorCamera
from utils.bib import degrees_to_radians, normalize_angle2, normalize_angle_degrees, is_ON
from rclpy.node import Node
from utils.constants import CALLBACK_INTERVAL_TURN, CALLBACK_INTERVAL_ACCELERATION
from utils.constants import MIN_SPEED_TURN, TOP_SPEED_TURN, TOP_SPEED
from utils.constants import FACTOR_ACCELERATION, FACTOR_BRAKING

# Funções Auxiliares
def get_angular_speed(error: float) -> float:
    """
    Calcula a velocidade angular com base no erro.
    """
    Kp = 1.0  # Este é o coeficiente proporcional. Ajuste conforme necessário.
    angular_speed = Kp * error

    # Limitando a velocidade angular
    if abs(angular_speed) < MIN_SPEED_TURN:
        angular_speed = MIN_SPEED_TURN if angular_speed > 0 else -MIN_SPEED_TURN
    elif abs(angular_speed) > TOP_SPEED_TURN:
        angular_speed = TOP_SPEED_TURN if angular_speed > 0 else -TOP_SPEED_TURN

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
        self.turn_diff = 0.0
        self.orientation:int = 0
        self.initial_orientation = 0.0
        self.current_orientation = 0.0
        self.destiny_orientation = 0.0
        self.old_speed          = 0
        self.get_old_turn_direction = 0
        self.distance = 0.0
        self.speed_ = 0.0
        self.twist = Twist()
        # callbacks
        self.turn_timer = None
        self.speed_timer =  None
        # Publicadores e Subscritores
        self.twist_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.sensor_imu: SensorIMU = SensorIMU(node)
        self.sensor_odom: SensorOdom = SensorOdom(node)
        self.sensor_lidar: SensorLidar = SensorLidar(node, handle_obstacle_detection)
        self.sensor_camera: SensorCamera = SensorCamera(node, handle_aruco_detected)


    def get_distance_to_wall(self, grau = 0):
        """
        Retorna a distância ao obstáculo mais próximo diretamente à frente do robô.
        """
        return self.sensor_lidar.get_data_range(grau)
    
    def get_time_now(self):
        return self.node.get_clock().now().seconds_nanoseconds()[0]
    
   
    def get_orientation(self) -> float:
        """
        Calcule e retorne a orientação do sensor IMU. (angle z) em graus.
        """
        return self.sensor_imu.get_orientation()
    
    def get_acceleration(self):
        '''
        Retorna a ostado da aceleracao
         0 : estável
        -1 : desacelerando
         1 : acelerando
        '''
        new = self.get_speed()
        old = self.old_speed
        self.old_speed = self.get_speed()
        if round(new, 2) == round(old, 2) or self.state == CommandType.STOP:
            return 0
        return 1 if new > old else -1
    
    def get_speed(self) -> float:
        return self.twist.linear.x
    
    # Movimentação
    def move_forward(self, speed=0.5):
        """
        Move o robô para frente. Se a velocidade não for especificada, use 0.3 como padrão.
        """
        self.old_speed      = self.twist.linear.x
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
        self.old_speed      = self.twist.linear.x
        self.twist.linear.x = speed
        self.twist_pub.publish(self.twist)
        self.move_timer = self.node.create_timer(CALLBACK_INTERVAL_ACCELERATION, self.__move_timer_callback)

    def __move_timer_callback(self):
        travelled_distance = self.sensor_odom.get_travelled_distance()
        if self.distance - travelled_distance <= 0.001:
            self.old_speed      = self.twist.linear.x
            self.twist.linear.x = 0.0
            self.twist_pub.publish(self.twist)
            self.move_timer.cancel()
            self.running_the_command = False


    def stop(self):
        """
        Para o robô.
        """
        if is_ON(self.turn_timer):
            self.turn_timer.cancel()
        if is_ON(self.speed_timer):
            self.speed_timer.cancel()

        self.twist = Twist()
        self.twist_pub.publish(self.twist)
        self.state = CommandType.STOP
        self.node.get_logger().warning(f'STOP')

    def stop_turn(self):
        if self.is_turnning():
            self.turn_timer.cancel()
            self.twist.angular.z = 0.0
            self.twist_pub.publish(self.twist)


    # Outros
    def get_state(self):
        """
        Retorna o estado atual do robô.
        """
        return self.state
    
    # Rotação
    def __speed_timer_callback(self):
        factor = FACTOR_ACCELERATION if self.speed_ >= self.get_speed() else FACTOR_BRAKING
        print(f"   FACTOR: {factor}")
        speed = round(self.get_speed() + factor, 3)
        print(f"new speed: {speed}")
        speed = max(self.speed_, min(speed, self.speed_))
        print(f"   Ajuste: {speed} : ajustado com min e max" )
        if speed == self.speed_:
            self.speed_timer.cancel()
            self.node.destroy_node()
        else:
            self.move_forward(speed)
        self.old_speed = self.get_speed()

    
    def set_speed(self, speed):
        self.speed_ = round(max(0, min(speed, TOP_SPEED)), 3)
        print(f"SET_SPEED: {self.speed_}")
        if self.speed_timer is None:
            self.speed_timer = self.node.create_timer(CALLBACK_INTERVAL_ACCELERATION, self.__speed_timer_callback)  # Verifica a cada 0.001 segundos
        else:
            self.speed_timer.reset()



    def __turn(self, speed):
        """
        Define a velocidade de rotação.
        """
        self.get_old_turn_direction = 1 if speed > 0 else -1
        self.twist.angular.z = speed
        self.twist_pub.publish(self.twist)

    def turn_by_angle(self, angle:int):
        """
        Gira o robô por um ângulo específico em graus.
        """
        self.orientation += angle
        self.running_the_command = True
        self.state = CommandType.TURN
        self.initial_orientation = round(normalize_angle_degrees(self.get_orientation()))
        self.destiny_orientation = normalize_angle2(self.orientation)
        if self.turn_timer is None:
            self.turn_timer = self.node.create_timer(CALLBACK_INTERVAL_TURN, self.__turn_timer_callback) 
        else:
            self.turn_timer.reset()
        self.log_info = False
    
    def turn_by_orientation(self, orientation) -> float:
        orientation_target = Orientation.format_degrees(orientation)
        orientation_actual = round(normalize_angle_degrees(self.get_orientation()))
        angle = normalize_angle2(orientation_target - orientation_actual)
        self.turn_by_angle(angle)
        return angle
    
    
    def __turn_timer_callback(self):
        self.current_orientation = normalize_angle2(self.get_orientation())
        # Calcula a diferença entre a orientação atual e a desejada
        difference = normalize_angle2(self.destiny_orientation - self.current_orientation)
        self.turn_diff = difference
        
        # Se a diferença for pequena o suficiente, pare o robô
        if abs(difference) <= 0.15:
            self.turn_timer.cancel()
            self.twist.angular.z = 0.0
            self.twist_pub.publish(self.twist)
        else:
            # Caso contrário, continue girando na direção mais curta para alcançar a orientação desejada
            angular_speed = get_angular_speed(degrees_to_radians(difference))
            self.__turn(angular_speed)
            if abs(difference) <= 5.0 and self.log_info:
                self.node.get_logger().info(f'Current: {round(self.current_orientation, 3)} | Distance: {round(abs(difference), 3)} | Destiny: {self.destiny_orientation}')

    def is_turnning(self) -> bool:
        return is_ON(self.turn_timer)