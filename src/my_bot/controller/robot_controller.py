from model import Robot
from utils import Command, CommandQueue, CommandType, Graph, Path
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
        #self.command_timer = self.node.create_timer(0.1, self.__execute_commands)
        self.graph = Graph()
        self.graph.load_from_file('support/graph_data.txt')
        # Busca todos os caminhos do vértice id_origen para o vértice id_destiny
        self.id_origen = 25
        self.id_destiny = 1
        self.target_:Path = None
        self.all_paths_from_destiny = self.graph.load_paths_if_exist(self.id_origen, self.id_destiny)
        self.targets_list:list[Path] = None
        self.robot.move_forward(speed=1.0)
        self.get_next_target()

    def get_next_target(self):
        if self.targets_list is None or len(self.targets_list) == 0:
           if self.all_paths_from_destiny is None or len(self.all_paths_from_destiny) == 0:
              self.target_ = None
           else:
               self.targets_list = self.all_paths_from_destiny.pop(0)
               self.target_ = self.targets_list.pop(0)
        elif self.target_ is not None and self.graph.has_next(self.target_):
             self.target_ = self.targets_list.pop(0)
        if self.target_ is None:
           print("Sem caminhos disponiveis")
           return
        else:
            if self.target_.id_destiny == self.id_destiny:
               print("Chegamos no destino")
               self.robot.stop()
               self.all_paths_from_destiny.clear()
               return
            else:
               self.robot.sensor_camera.fix_target(self.target_.id_destiny)
               self.robot.turn_by_orientation(self.target_.orientation)
               self.id_origen = self.target_.id_origen

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
        return float(value)
    
    # Camera
    def handle_aruco_detected(self, angle):
        """
        ids detectados
        """
        angle = round(angle, 1)
        distance_target = round(self.get_distance_to_obstacle(), 2)

        print(f'direction:[ {angle:>3} ], distance target: {distance_target:>4} : ref: {abs(angle) / distance_target:>4}')
        
        if self.robot.sensor_camera.id_aruco_target_lock:
            #if abs(angle) <= 0.5:
            #    self.robot.stop_turn()
            if abs(angle) > self.old_angle:
                direction = "DIREITA" if angle > 0 else "ESQUERDA"
                self.node.get_logger().warning(f'Virando para {direction}: {abs(angle):>3}º')
                self.robot.turn_by_angle(angle)
                self.ajuste_speed(int(angle), round(distance_target, 1))
            self.old_angle = abs(angle)
            distance_target = round(self.get_distance_to_obstacle(), 2)
            print(f"distancia: {distance_target:>4} {distance_target <= 0.20}")
            if distance_target <= 0.20:
                self.robot.stop_turn()
                self.robot.stop()
                self.get_next_target()
            
        elif (abs(angle) // distance_target) < 5.0:
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
                speed = 1.2
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