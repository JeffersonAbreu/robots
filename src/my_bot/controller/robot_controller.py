from model import Robot
from utils import Command, CommandQueue, CommandType, Graph
from rclpy.node import Node
import pdb # pdb.set_trace() # serve para debugar
from utils.bib import degrees_to_radians, radians_to_degrees
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
        self.lidar_data_ranges=[]
        self.old_angle = 0
        self.command_timer = self.node.create_timer(0.1, self.__execute_commands)
        graph = Graph()
        graph.load_from_file('/home/jeff/tcc/robots/support/graph_data.txt')
        self.paths = graph.find_paths(25, 4)
        #self.robot.sensor_camera.fix_target(alvo)

    def clear_task_list(self):
        if self.commands_queue.size() > 0:
            self.commands_queue.clear()
        self.robot.stop_turn()
        self.running_the_command = False

    def add_task(self, comm: Command):
        self.commands_queue.add_command(comm)

    # Lidar
    def handle_obstacle_detection(self, ranges):
        """
        Lógica para lidar com a detecção de obstáculos
        """
        try:
            self.lidar_data_ranges = ranges
        except Exception as e:
            print(f'robot_controller:handle_obstacle_detection {e}')

    def get_distance_to_obstacle(self, grau = 0):
        """
        Retorna a distância ao obstáculo mais próximo diretamente à frente do robô.
        """
        if grau == 0:
            front_index = 135 # index do meio ( frente: ponto zero do robo )
        else:
            front_index = 135 + grau
        value = self.lidar_data_ranges[front_index]
        #return value
    
    # Camera
    def handle_aruco_detected(self, angle):
        """
        ids detectados
        """
        distance_target = self.get_distance_to_obstacle()
        if distance_target < 1.0:
            new_angle, new_target = self.get_next_target()
            if new_target is not None:
                self.robot.stop_turn()
                self.robot.turn_by_angle(new_angle)
                self.robot.sensor_camera.fix_target(new_target)
            else:
                self.robot.stop()
            return
        print(f'direction:[ {angle} ], distance target: {distance_target} : ref: {abs(angle) // distance_target}')
        if self.robot.sensor_camera.id_aruco_target_lock:
            if self.old_angle < angle or angle > self.old_angle:
                self.robot.stop_turn()
                self.robot.turn_by_angle(angle)
                self.ajuste_speed(angle, distance_target)
            self.old_angle = angle
            
        elif (abs(angle) // distance_target) < 5:
            self.robot.sensor_camera.set_lock()

    def ajuste_speed(self, angle, dist):
        speed = 0.5
        if dist > 2.0:
            if abs(angle) > 60:
                speed = 0.6
            elif abs(angle) > 30:
                speed = 0.7
            elif abs(angle) > 15:
                speed = 0.8
            else:
                speed = 1.0
        elif dist > 1.0:
            speed = 0.7
        self.robot.move_forward(speed)
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

