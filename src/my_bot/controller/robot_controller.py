from model import Robot
from utils import Command, CommandQueue, CommandType, Graph, Path
from rclpy.node import Node
import pdb # pdb.set_trace() # serve para debugar
import inspect
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
            self.lidar_data_ranges = [10.0 for _ in range(271)] # range lidar, preencha com 10 para não ter erro
            self.robot = Robot(self.node, self.handle_obstacle_detection, self.handle_aruco_detected)
            self.distance_to_aruco = 0
            self.rotation_angle = 0
            self.old_angle = 0
            self.new_action = False
            self.target_actual:Path = None
            self.targets_list:list[Path] = all_paths_from_destiny[0] # pega a rota menor / primeira rota
            self.go_next_target()
            self.go = False
            self.detected = False
            self.detected_count = 0
            self.loop_timer     = self.node.create_timer(1.0, self.__loop_not_detected)
            self.command_timer  = self.node.create_timer(0.05, self.__execute_commands)


    def go_next_target(self):
        if len(self.targets_list) > 0:
            self.target_actual = self.targets_list.pop(0)
            self.robot.sensor_camera.fix_target(self.target_actual.id_destiny)
            angle_new_orientation = self.robot.turn_by_orientation(self.target_actual.orientation)
            speed = 0.5
            if angle_new_orientation > 90 :
                if self.get_distance_to_wall() < 0.50:
                    speed = 0.1
                else:
                    speed = 0.2
            elif angle_new_orientation > 60 :
                if self.get_distance_to_wall() < 0.50:
                    speed = 0.2
                else:
                    speed = 0.3
            elif angle_new_orientation > 45 :
                speed = 0.4
            self.robot.set_speed(speed)
            self.node.get_logger().error(f"            SPEED: {speed:>2}")
            self.node.get_logger().warning(f"    ID destiny: {self.target_actual.id_destiny:>2}")
            self.node.get_logger().warning(f"ID orientation: {self.target_actual.orientation.name}")
            print("Proximos:")
            self.node.get_logger().warning(" -> ".join(f"{vertex.id_destiny} {vertex.orientation.name}" for vertex in self.targets_list))
        elif self.robot.sensor_camera.id_aruco_target == self.target_actual.id_destiny:
            self.node.get_logger().warning("Chegamos no destino!")
            self.robot.stop()

    # Lidar
    def handle_obstacle_detection(self, ranges):
        """
        Lógica para lidar com a detecção de obstáculos
        """
        try:
            self.lidar_data_ranges = ranges
        except Exception as e:
            print(f'robot_controller:handle_obstacle_detection {e}')

    def get_distance_to_wall(self, grau = 0):
        """
        Retorna a distância ao obstáculo mais próximo diretamente à frente do robô.
        """
        index = 135 + grau
        value = self.lidar_data_ranges[index]
        try:
            value = float(value)
        except:
            print("Não é float. Ajustado para 10.0")
            value = 10.0
        return value
    
    # Camera
    def handle_aruco_detected(self, distance_to_aruco=0.1, pixel_error=0.1):
        """
        ids detectados
        """
        self.rotation_angle        = pixel_error
        self.distance_to_aruco  = distance_to_aruco
        self.new_action         = True
        self.detected           = True

    def ajuste_speed(self, alfa):
        alfa = abs(alfa)
        speed = 0
        if self.get_distance_to_wall() > 3.0:
            if   alfa < 0.1:
                speed=1.5
            elif alfa < 0.2:
                speed=1.2
            elif alfa < 0.5:
                speed=1.0
            elif alfa < 1.0:
                speed=0.9
            elif alfa < 1.5:
                speed=0.8
            elif alfa < 3:
                speed=0.7
            elif alfa < 5:
                speed=0.6
            elif alfa < 8:
                speed=0.5
            elif alfa < 10:
                speed=0.4
            else:
                speed=0.3
        elif alfa < 5:
            speed=1.0
            if self.get_distance_to_wall() > 1.0:
                speed = 0.7
        else:
            speed = 0.5
        return speed
    # Comandos

    def __execute_commands(self):
        """
        Executa os comandos na fila.
        """
        dist_wall = f'{round(self.get_distance_to_wall(), 2):.2f}m'
        dist_aruc = "-"
        my_speed_ = f'{round(self.robot.get_speed(), 2):.2f}'
        angle_err = "-"

        if self.new_action:
            dist_aruc = f'{round(self.distance_to_aruco, 2):.2f}m'
            angle_err = self.rotation_angle
            dist_wall = f'{round(self.get_distance_to_wall(int(angle_err)), 2):.2f}m'
        if self.detected_count <= 10:            
            if self.new_action and self.distance_to_aruco >= 1.0:
                if self.robot.sensor_camera.track_aruco:
                    angle  = self.rotation_angle
                    angle_err = angle
                    if abs(angle) == 0.0:
                        self.robot.stop_turn()
                    elif abs(angle) > self.old_angle:
                        direction = "DIREITA" if angle > 0 else "ESQUERDA"
                        self.node.get_logger().warning(f'Virando para {direction:>8}: {abs(angle):>3}°')
                        self.robot.turn_by_angle( angle )
                    speed = self.ajuste_speed( angle )
                    self.robot.set_speed(speed)
                    self.old_angle = abs(angle)
                    if self.robot.get_speed() != speed:
                        my_speed_ = green(my_speed_) if self.robot.get_speed() < speed else red(my_speed_)
                self.new_action = False  
            elif  self.robot.sensor_camera.track_aruco:
                if self.distance_to_aruco <= 1.0 and self.distance_to_aruco > 0.9:
                    print(green("Entrei aqui: "), get_current_line_number())
                    self.robot.stop_turn()
                    self.robot.set_speed(0.1)
                    print(red("cheguei aqui: "), get_current_line_number())
                elif self.distance_to_aruco > 0 and self.distance_to_aruco < 1.0 and self.robot.get_speed() < 0.3: # espera quase parar para ter outro comando/alvo
                    self.go = False
                    self.distance_to_aruco = 0
                    self.rotation_angle = 0
                    self.go_next_target()
                else:
                    self.robot.set_speed(0.3)
            elif not self.robot.is_turnning():
                    if not self.go:
                        self.robot.set_speed(0.5)
                        self.go = True
                    elif self.distance_to_aruco > 0.1 and abs(self.rotation_angle // self.distance_to_aruco) <= 5:
                        print(red(abs(self.rotation_angle // self.distance_to_aruco)))
                        self.robot.sensor_camera.track_aruco = True
                        self.node.get_logger().error(f'Mira travada no aruco: {self.robot.sensor_camera.id_aruco_target}!')
                        self.robot.stop_turn()

        print(f"dist_wall: {dist_wall:>6}, dist_aruc: {dist_aruc:>6}, angle: {angle_err:>9}, speed: {my_speed_:>4}m/s")

    def __loop_not_detected(self):
        if not self.detected:
            self.detected_count += 1
            self.detected = False
        elif self.detected_count > 10:
                self.command_timer.cancel()
                print(f"Cancelado: {self.command_timer.is_canceled()}")
                print(red("Dez repetições sem achar o aruco"))
                self.detected_count = 0
        else:
            self.detected_count = 0
            if self.command_timer.is_canceled():
                print(f"Cancelado: {self.command_timer.is_canceled()}")
                self.command_timer  = self.node.create_timer(0.005, self.__execute_commands)
                print(f"Cancelado: {self.command_timer.is_canceled()}")

def green(text):
    return f"\033[92m{text}\033[0m"
def red(text):
    return f"\033[91m{text}\033[0m"

# Função para obter o número da linha atual
def get_current_line_number():
    return inspect.currentframe().f_back.f_lineno