import math
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import LaserScan
from utils.state import State
from utils.robot_estimator import RobotEstimator
from utils.command import Command, CommandType, CommandQueue
from utils.bib import degrees_to_radians, normalize_angle

# Constantes de Velocidade
VEL_MIN = 0.1
TOP_SPEED = 2.0

# Funções Auxiliares

def get_angular_speed(error: float) -> float:
    """
    Calcula a velocidade angular com base no erro.
    """
    Kp = 1.0
    angular_speed = Kp * error

    # Limitando a velocidade angular
    if abs(angular_speed) < VEL_MIN:
        angular_speed = VEL_MIN if angular_speed > 0 else -VEL_MIN
    elif abs(angular_speed) > TOP_SPEED:
        angular_speed = TOP_SPEED if angular_speed > 0 else -TOP_SPEED

    return angular_speed

# Classe Principal

class Robot:

    def __init__(self, node):
        """
        Inicializa o robô.
        """
        self.node = node
        self.robot_estimator = RobotEstimator(node)
        self.state = State.STOP
        self.current_orientation = 0.0
        self.destiny_orientation = 0.0

        # Publicadores e Subscritores
        self.twist_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.lidar_sub = self.node.create_subscription(LaserScan, '/lidar', self.__lidar_callback, 10)

        # Comandos e Timers
        self.command_queue = CommandQueue()
        self.command_timer = self.node.create_timer(0.1, self.__execute_commands)

    # Movimentação

    def move_forward(self):
        """
        Move o robô para frente.
        """
        twist = Twist()
        twist.linear.x = 0.3
        self.twist_pub.publish(twist)
        self.state = State.FORWARD

    def move_distance(self, distance: float):
        """
        Move o robô por uma distância específica.
        """
        start_position = self.robot_estimator.get_position()
        current_position = start_position
        speed = 0.3  # Velocidade padrão em m/s

        while abs(current_position - start_position) < distance:
            self.move_forward(speed)
            current_position = self.robot_estimator.get_position()

        self.stop()

    def start_curve(self, angle: float):
        """
        Começa a curvar o robô por um ângulo específico.
        """
        # Definir velocidade para a frente
        speed = 0.3
        self.move_forward(speed)

        # Começar a curvar
        self.turn_by_angle(angle)

    def move_backward(self, distance: float):
        """
        Move o robô para trás por uma distância específica.
        """
        start_position = self.robot_estimator.get_position()
        current_position = start_position
        speed = -0.3  # Velocidade negativa para mover para trás

        while abs(current_position - start_position) < distance:
            self.move_forward(speed)  # Reutilizando a função move_forward com velocidade negativa
            current_position = self.robot_estimator.get_position()

        self.stop()

    def stop(self):
        """
        Para o robô.
        """
        twist = Twist()
        self.twist_pub.publish(twist)
        self.state = State.STOP

    # Rotação

    def __turn_speed_left(self, angle_speed):
        """
        Gira o robô para a esquerda.
        """
        self.__turn_speed(angle_speed * -1)

    def __turn_speed_right(self, angle_speed):
        """
        Gira o robô para a direita.
        """
        self.__turn_speed(angle_speed)

    def __turn_speed(self, angle_speed):
        """
        Define a velocidade de rotação.
        """
        twist = Twist()
        twist.angular.z = angle_speed
        self.twist_pub.publish(twist)

    def turn_by_angle(self, angle):
        """
        Gira o robô por um ângulo específico.
        """
        self.state = State.TURN
        self.current_orientation = self.robot_estimator.get_orientation()
        self.destiny_orientation = normalize_angle(self.current_orientation + degrees_to_radians(angle))
        error = self.destiny_orientation - self.current_orientation
        angular_speed = get_angular_speed(error)
        if angular_speed > 0:
            self.__turn_speed_right(angular_speed)
        else:
            self.__turn_speed_left(-angular_speed)

    # Comandos

    def __execute_commands(self):
        """
        Executa os comandos na fila.
        """
        command = self.command_queue.get_next_command()
        if command:
            if command.type == CommandType.MOVE_FORWARD:
                self.move_distance(command.value)
            elif command.type == CommandType.MOVE_BACKWARD:
                self.move_backward(command.value)
            elif command.type == CommandType.TURN:
                self.turn_by_angle(command.value)
            elif command.type == CommandType.STOP:
                self.stop()
            elif command.type == CommandType.START_CURVE:
                self.start_curve(command.value)

    # Callbacks

    def __lidar_callback(self, msg):
        """
        Callback para dados do LiDAR.
        """
        if self.get_state() == State.FORWARD:
            distance_to_wall = msg.ranges[0]
            if distance_to_wall < 0.9:
                self.command_queue.add_command(CommandType.TURN, 90)
            elif distance_to_wall < 0.3:
                self.command_queue.add_command(CommandType.STOP)
                self.command_queue.cancel_all()

    # Outros

    def get_state(self):
        """
        Retorna o estado atual do robô.
        """
        return self.state
