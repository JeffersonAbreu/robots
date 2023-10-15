from model import Robot
from utils import Command, CommandQueue, CommandType
from rclpy.node import Node

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
        
        self.current_orientation = 0.0
        self.current_command = 0.0

        self.command_queue: CommandQueue = CommandQueue()
        self.bkp_command_queue: CommandQueue = self.command_queue
        self.command_timer = self.node.create_timer(0.1, self.__execute_commands)
        self.avoiding_obstacle = None
        self.execute_task_list()

    def execute_task_list(self):
        """
        Exemplo de lista de tarefas
        """
        tasks = [
            Command(CommandType.MOVE_FORWARD, 1.0),  # Mover para frente
            #Command(CommandType.CURVE, 45),          # Começar a curvar 45º
            #Command(CommandType.STOP),               # Parar
            #Command(CommandType.MOVE_BACKWARD, 1.0), # Dar ré 1m
            #Command(CommandType.TURN, -90),          # Girar 90º esquerda
            #Command(CommandType.MOVE_FORWARD, 0.5),  # Pra frente 50cm
            #Command(CommandType.STOP)                # Parar
        ]



        for task in tasks:
            self.command_queue.add_command(task)

    def follow_wall(self):
        '''
        Seguir parede
        '''
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
    '''
    def handle_qr_code_data(self, data):
        # Aqui, vou poder lidar com os dados do QR Code. Por exemplo, se o QR Code indicar
        # que o robô está próximo a uma porta, você pode instruir o robô a entrar.
        pass
    '''
    
        # Lidar
    def handle_obstacle_detection(self):
        """
        Lógica para lidar com a detecção de obstáculos
        """
        
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
        
        command: Command = self.command_queue.get_next_command()
        self.current_command = command
        if command:
            if   command.type == CommandType.MOVE_FORWARD:
                if command.value == None:
                    self.robot.move_forward()
                else:
                    self.robot.move_distance(distance=command.value)
            elif command.type == CommandType.MOVE_BACKWARD:
                self.robot.move_backward(command.value)
            elif command.type == CommandType.TURN:
                self.robot.turn_by_angle(command.value)
            elif command.type == CommandType.STOP:
                self.robot.stop()
            elif command.type == CommandType.CURVE:
                self.robot.curve(command.value)

    def pause_task_queue(self):
        """
        Pausa a fila de tarefas.
        """
        self.is_paused = True
        self.robot.stop()
        self.command_queue.add_command_to_init(self.current_command)
        self.current_orientation = self.robot.get_orientation()
        self.bkp_command_queue = self.command_queue
        self.command_queue.clear()
        self.is_paused = False

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
        tasks = []
        if self.command_queue.size() == 0:
            if self.robot.get_distance_to_obstacle() < MIN_DISTANCE:
                tasks = [
                    Command(CommandType.MOVE_BACKWARD, 0.2),  # Mover para tras
                    Command(CommandType.STOP),               # Parar
                    Command(CommandType.TURN, -90),          # Girar 90º esquerda
                    Command(CommandType.MOVE_FORWARD, 0.5),  # Pra frente 50cm
                    Command(CommandType.STOP)                # Parar
                ]
            else:
                # Depois de contornar o obstáculo, retome a fila de tarefas.
                self.resume_task_queue()


        if tasks:
            for task in tasks:
                self.command_queue.add_command(task)


