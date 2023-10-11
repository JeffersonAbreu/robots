from model import Robot
from utils import Command, CommandType
from .qrcode_detector import QRCodeDetector
from .robot_navigator import RobotNavigator

class RobotController:
    '''
    O controlador, ou "controller", é responsável por tomar decisões com base nas informações\n
    dos sensores e no planejamento de caminho. O algoritmo de "seguir parede" é uma técnica\n
    simples de navegação que faz o robô seguir uma parede à sua direita ou esquerda até \n
    encontrar um objetivo ou evitar obstáculos.
    '''

    def __init__(self, robot: Robot):
        self.robot = robot
        self.execute_task_list(robot)
        self.navigator = RobotNavigator(robot)
        self.qr_detector = QRCodeDetector()

    def execute_task_list(self, robot: Robot):
        """
        Exemplo de lista de tarefas
        """
        tasks = [
            Command(CommandType.MOVE_FORWARD, 0.4),  # Mover para frente 40cm
            Command(CommandType.START_CURVE, 45),    # Começar a curvar 45º
            Command(CommandType.STOP),               # Parar
            Command(CommandType.MOVE_BACKWARD, 1.0), # Dar ré 1m
            Command(CommandType.TURN, -90),          # Girar 90º esquerda
            Command(CommandType.MOVE_FORWARD, 0.5),  # Pra frente 50cm
            Command(CommandType.STOP)                # Parar
        ]

        for task in tasks:
            robot.command_queue.add_command(task)

    def follow_wall(self):
        while True:
            # Se detectar uma parede à direita, vire à esquerda
            if self.robot.lidar_detected_wall_on_right():
                self.robot.turn_left()
            # Se detectar um obstáculo à frente, pare e vire à esquerda
            elif self.robot.lidar_detected_obstacle_ahead():
                self.robot.stop()
                self.robot.turn_left()
            # Se não houver obstáculos, continue movendo-se para a frente
            else:
                self.robot.move_forward()

            # Verifique se há um QR Code visível e leia as informações dele
            image = self.robot.get_camera_image()
            qr_code_data = self.qr_detector.detect_qrcode(image)
            if qr_code_data:
                self.handle_qr_code_data(qr_code_data)

    def handle_qr_code_data(self, data):
        # Aqui, vou poder lidar com os dados do QR Code. Por exemplo, se o QR Code indicar
        # que o robô está próximo a uma porta, você pode instruir o robô a entrar.
        pass