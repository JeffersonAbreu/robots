from model import Robot
from utils import Command, CommandQueue, CommandType
from rclpy.node import Node
import pdb # pdb.set_trace() # serve para debugar
from utils.bib import degrees_to_radians, radians_to_degrees
import os
MIN_DISTANCE = 0.25

class RobotController:
    '''
    O controlador, ou "controller", é responsável por tomar decisões com base nas informações\n
    dos sensores e no planejamento de caminho. O algoritmo de "seguir parede" é uma técnica\n
    simples de navegação que faz o robô seguir uma parede à sua direita ou esquerda até \n
    encontrar um objetivo ou evitar obstáculos.
    '''

    def __init__(self, node: Node):
        self.node = node
        self.robot = Robot(self.node, self.handle_obstacle_detection, self.handle_aruco_detected)
        # self.navigator = RobotNavigator(self.robot)
        # self.qr_detector = QRCodeDetector()

        self.lidar_data_ranges: list[float] = []
        
        self.current_orientation = 0.0
        self.current_command = 0.0

        self.commands_queue: CommandQueue = CommandQueue()
        self.bkp_commands_queue: CommandQueue = self.commands_queue
        self.execute_task_list()
        self.command_timer = self.node.create_timer(0.1, self.__execute_commands)

        alvo = 26
        self.robot.sensor_camera.fix_target(alvo)

    def execute_task_list(self):
        """
        Exemplo de lista de tarefas
        """
        tasks = [ 
            #Command(CommandType.MOVE_FORWARD, 0.5),  
            #Command(CommandType.MOVE_FORWARD, 4.5),  # Mover para frente
            Command(CommandType.TURN, -90),
            #Command(CommandType.STOP)               # Parar
        ]

        for task in tasks:
            self.commands_queue.add_command(task)
    
    # Lidar
    def handle_obstacle_detection(self, ranges: list[float]):
        """
        Lógica para lidar com a detecção de obstáculos
        """
        self.lidar_data_ranges = ranges

        if self.get_distance_to_obstacle() < MIN_DISTANCE:
            pass

    def get_distance_to_obstacle(self, grau: int = 0) -> float:
        """
        Retorna a distância ao obstáculo mais próximo diretamente à frente do robô.
        """
        if grau == 0:
            front_index = 135 # index do meio ( frente: ponto zero do robo )
        else:
            front_index = 135 + grau
        return self.lidar_data_ranges[front_index]
    
    # Camera
    def handle_aruco_detected(self, error):
        """
        ids detectados
        """
        self.commands_queue.clear()
        self.robot.move_forward()
        if not self.robot.sensor_camera.id_aruco_target_lock:
            if abs(error) < 1:
                self.robot.sensor_camera.set_lock()
        else:
            pass
        distance = self.get_distance_to_obstacle()
        print(f'erro X:[ {error:.2f} ], distance: {distance}')

    # Comandos

    def __execute_commands(self):
        """
        Executa os comandos na fila.
        """
        if not self.robot.is_command_running(): # Se não estiver rodando um comando
            if self.commands_queue.is_next(): # existir próximo
                command: Command = self.commands_queue.get_next_command()
                self.current_command = command
                self.robot.execute(command)