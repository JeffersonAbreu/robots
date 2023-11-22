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
        graph = Graph()
        graph.load_from_file('support/graph_data.txt')
        # Busca todos os caminhos do vértice id_origen para o vértice id_destiny
        all_paths_from_destiny = graph.load_paths_if_exist( 25, 1 )
        if len(all_paths_from_destiny) == 0:
            self.node.get_logger().error(f'Não foi localizado nenhuma rota!')
        else:
            self.lidar_data_ranges=[]
            self.robot = Robot(self.node, self.handle_obstacle_detection, self.handle_aruco_detected)
            self.new_angle = 0
            self.old_angle = 0
            self.target_actual:Path = None
            self.targets_list:list[Path] = all_paths_from_destiny[0] # pega a rota menor / primeira rota
            self.go_next_target()
            self.robot.speed_up()
            #self.command_timer = self.node.create_timer(0.001, self.__execute_commands)


    def go_next_target(self):
        if len(self.targets_list) > 0:
            self.target_actual = self.targets_list.pop(0)
            self.robot.sensor_camera.fix_target(self.target_actual.id_destiny)
            angle = self.robot.turn_by_orientation(self.target_actual.orientation)

            self.node.get_logger().warning(f"    ID destiny: {self.target_actual.id_destiny:>2}")
            self.node.get_logger().warning(f"ID orientation: {self.target_actual.orientation.name}")
            print("Proximos:")
            self.node.get_logger().warning(" -> ".join(f"{vertex.id_destiny} {vertex.orientation.name}" for vertex in self.targets_list))
            return angle
        elif self.robot.sensor_camera.id_aruco_target == self.target_actual.id_destiny:
            self.node.get_logger().warning("Chegamos no destino!")
            self.robot.stop()
            return None

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
        index = 135 + grau
        print(index)
        return float(self.lidar_data_ranges[index])
    
    # Camera
    def handle_aruco_detected(self, angle):
        """
        ids detectados
        """
        self.new_angle = round(angle, 1)
        distance_target = round(self.get_distance_to_obstacle(), 2)
        ref = round(abs(angle) / distance_target, 2)

        print(f'erro aruco:[ {self.new_angle:>3} ], distance target front: {distance_target:>4} : ref: {ref:>4}, speed: {round(self.robot.get_speed(), 4):>4}')
        
        
        if not self.robot.sensor_camera.id_aruco_target_lock:
            if ref < 4.0:
                self.robot.sensor_camera.set_lock()
                self.robot.stop_turn()
            self.robot.speed_up(0.2)
        elif abs(self.new_angle) > self.old_angle:
            direction = "DIREITA" if self.new_angle > 0 else "ESQUERDA"
            self.node.get_logger().warning(f'Virando para {direction}: {abs(self.new_angle):>3}º')
            self.robot.turn_by_angle(self.new_angle)
        elif self.robot.sensor_camera.id_aruco_target_lock:
            self.ajuste_speed(ref, distance_target)
        
        if distance_target < 1.0 and self.robot.sensor_camera.id_aruco_target_lock:
            self.node.get_logger().error(f'Novo alvo marcado ID: {distance_target:>2}')
            self.robot.stop_turn()
            self.robot.speed_down(pare=True)
            angle = self.go_next_target()
        self.old_angle = abs(self.new_angle)


    def ajuste_speed(self, ref, dist):
        if dist > 1.00:
            if   ref < 0.1:
                self.robot.speed_up()
            elif ref < 0.2:
                self.robot.speed_up(speed=1.25)
            elif ref < 0.3:
                self.robot.speed_up(speed=1.1)
            elif ref < 0.4:
                self.robot.speed_up(speed=0.9)
            elif ref < 0.5:
                self.robot.speed_up(speed=0.8)
            elif ref < 0.6:
                self.robot.speed_up(speed=0.7)
            elif ref < 0.7:
                self.robot.speed_up(speed=0.6)
            elif ref < 0.8:
                self.robot.speed_up(speed=0.5)
            elif ref < 0.9:
                self.robot.speed_up(speed=0.4)
            else:
                self.robot.speed_up(speed=0.3)
        else:
            self.robot.speed_down()
    # Comandos

    def __execute_commands(self):
        """
        Executa os comandos na fila.
        """