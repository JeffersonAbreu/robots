from model import Robot
from utils import Command, CommandQueue, CommandType
from rclpy.node import Node
import pdb # pdb.set_trace() # serve para debugar
from utils.bib import radians_to_degrees
import os
# acesse ->     zzzcode.ai
MIN_DISTANCE = 0.5

class RobotController:
    '''
    O controlador, ou "controller", é responsável por tomar decisões com base nas informações\n
    dos sensores e no planejamento de caminho. O algoritmo de "seguir parede" é uma técnica\n
    simples de navegação que faz o robô seguir uma parede à sua direita ou esquerda até \n
    encontrar um objetivo ou evitar obstáculos.
    '''

    def __init__(self, node: Node):
        self.node = node
        self.robot = Robot(self.node, self.handle_obstacle_detection)
        # self.navigator = RobotNavigator(self.robot)
        # self.qr_detector = QRCodeDetector()

        self.is_paused = False
        self.avoiding_obstacle = False
        
        self.current_orientation = 0.0
        self.current_command = 0.0

        self.command_queue: CommandQueue = CommandQueue()
        self.bkp_command_queue: CommandQueue = self.command_queue
        self.execute_task_list()
        self.command_timer = self.node.create_timer(0.1, self.__execute_commands)

    def execute_task_list(self):
        """
        Exemplo de lista de tarefas
        """
        tasks = [
            #Command(CommandType.MOVE_BACKWARD, 1.5), # Dar ré 1m
            #Command(CommandType.TURN, -15), 
            Command(CommandType.MOVE_FORWARD, 1.5),  Command(CommandType.TURN, -40),
            Command(CommandType.MOVE_FORWARD),  # Mover para frente
            #Command(CommandType.STOP)                # Parar
        ]



        for task in tasks:
            self.command_queue.add_command(task)

        #self.node.get_logger().warning(f'tem {self.command_queue.size()} comandos')
    
    # Lidar
    def handle_obstacle_detection(self, ranges):
        """
        Lógica para lidar com a detecção de obstáculos
        """
        os.system('clear')
        for index, range in enumerate(ranges):
            self.node.get_logger().info(f'{index}: {range}')

        if self.robot.get_state() == CommandType.MOVE_FORWARD and self.robot.get_distance_to_obstacle() < MIN_DISTANCE:
            self.pause_task_queue()
            self.is_avoiding_obstacle = True
            self.avoiding_obstacle = self.node.create_timer(0.1, self.avoid_obstacle)

    # Comandos

    def __execute_commands(self):
        """
        Executa os comandos na fila.
        """
        if self.is_paused:
            return
        if not self.robot.is_command_running(): # Se não estiver rodando um comando
            if self.command_queue.is_next(): # existir próximo
                command: Command = self.command_queue.get_next_command()
                self.current_command = command
                self.robot.execute(command)

    def pause_task_queue(self):
        """
        Pausa a fila de tarefas.
        """
        self.is_paused = True
        self.robot.stop()
        if self.current_command.value != None:
            if self.current_command.type == CommandType.TURN:
                self.current_command.value = self.current_command.value - self.robot.get_orientation()
            if self.current_command.type == CommandType.MOVE_FORWARD or self.current_command.type == CommandType.MOVE_BACKWARD:
                    self.current_command.value = self.current_command.value - self.robot.sensor_odom.get_travelled_distance()
        self.command_queue.add_command_to_init(self.current_command)
        self.current_orientation = self.robot.get_orientation()
        self.bkp_command_queue = self.command_queue

    def resume_task_queue(self):
        """
        Retoma a fila de tarefas.
        """
        self.is_paused = True
        self.avoiding_obstacle.cancel()
        self.robot.stop()
        self.command_queue = self.bkp_command_queue
        self.bkp_command_queue.clear()
        self.is_paused = False

    def avoid_obstacle(self):
        """
        contornar o obstáculo.
        """
        if self.command_queue.size() == 0:
            if self.robot.get_distance_to_obstacle() < MIN_DISTANCE:
                tasks = [
                    Command(CommandType.MOVE_BACKWARD, 0.2), # Mover para tras
                    Command(CommandType.TURN, -45),          # Girar 90º esquerda
                    Command(CommandType.MOVE_FORWARD, 0.4),  # Pra frente 50cm
                ]
                self.command_queue.clear()
                for task in tasks:
                    self.command_queue.add_command(task)
            else:
                # Depois de contornar o obstáculo, retome a fila de tarefas.
                self.resume_task_queue()




