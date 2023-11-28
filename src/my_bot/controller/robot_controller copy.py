from model import Robot
from utils import Command, CommandQueue, CommandType, Graph, Path
from utils.bib import Color as cor
from rclpy.node import Node
import pdb # pdb.set_trace() # serve para debugar
import inspect
import time

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
            self.robo = Robot(self.node, self.handle_obstacle_detection, self.handle_aruco_detected)
            self.new_distance_aruco = 0
            self.old_distance_aruco = 0
            self.new_rotation_angle = 0
            self.old_rotation_angle = 0
            self.new_action         = False
            self.area_of_interest   = True
            self.count_not_detected = 0
            self.target_actual:Path = None
            self.targets_list:list[Path] = all_paths_from_destiny[0] # pega a rota menor / primeira rota

            self._timer_turn     = None
            self._timer_target   = None
            self._timer_move     = None
            self._timer_controll = self.node.create_timer(0.0001, self.__execute_controll)
            self._timer_walker   = None
            self._timer_show_inf = self.node.create_timer(0.0001, self.show_infos)

    def start_turn(self, timer=0.0001):
        self._timer_turn    = self.node.create_timer(timer, self.__turn__callback)
    def start_target(self, timer=0.0001):
        self._timer_target  = self.node.create_timer(timer, self.__next_target_callback)
    def start_move(self, timer=0.0001):
        self._timer_move    = self.node.create_timer(timer, self.__move__callback)
    
    def __execute_controll(self):
        if not self.robo.sensor_camera.track_aruco_target:
            if self.robo.sensor_camera.id_aruco_target is None:
                if self.robo.get_speed() <= 0.2:
                    self._timer_controll.cancel()
                    self.robo.set_speed(0.2)
                    while self.robo.get_speed() > 0.2:
                        time.sleep(0.5)
                        print(cor.green(f"Aguardando a velocidade diminuir <= 2: {self.robo.get_speed()}"), get_current_line_number())
                    self.new_action = True
                    if self.area_of_interest:
                        self.start_target()
                    time.sleep(2)
                    return
        else:
            if self.new_distance_aruco < 1 and not self.robo.sensor_camera.track_aruco_target:
                self.robo.sensor_camera.track_aruco_target = False
                self.robo.sensor_camera.id_aruco_target = None
            else:
                if self.is_update_info():
                    if self._timer_turn is None or self._timer_turn.is_canceled():
                        self.start_turn()
                    if self._timer_move is None or self._timer_move.is_canceled():
                        self.start_move()
                elif self.robo.sensor_camera.track_aruco_target:
                    self.count_not_detected  += 1
                    print(cor.red(f"Contando as vezes que não detectou o aruco! {self.count_not_detected} vezes."))
                    if(self.count_not_detected > 20):
                        self._timer_controll.cancel()
                        self.robo.set_speed(0)
                        self._timer_walker   = self.node.create_timer(0.01, self.__walker__callback)
        if self.robo.sensor_camera.id_aruco_target is None:
            min_wall, _ = self.robo.sensor_lidar.find_closest_wall_angle()
            if min_wall == float('-inf'):
                self.robo.stop()
                self._timer_controll.cancel()
                if not self._timer_turn.is_canceled():
                    self._timer_turn.cancel()
                if not self._timer_move.is_canceled():
                    self._timer_move.cancel()
                self._timer_walker   = self.node.create_timer(0.01, self.__walker__callback)
                            
                    
                
    def __walker__callback(self):
        angle, min_dist = self.robo.sensor_lidar.find_closest_wall_angle(self.old_rotation_angle)
        if min_dist < 0.5:
            esq = self.robo.get_distance_to_wall(angle - 1)
            dir = self.robo.get_distance_to_wall(angle + 1)
            self.robo.move_backward(0.6)
            angle = abs(dir) - abs(esq)
            self.robo.turn_by_angle(angle)
        if self.is_update_info():
            self.robo.stop()
            self._timer_controll.reset()
            self._timer_walker.cancel()
        else:
            # se chegou aqui está predido
            self.robo.turn_by_angle(self.old_rotation_angle)



    
    def __move__callback(self):
        """
        Executa os comandos na fila.
        """
        if self.is_update_info():
            speed = ajuste_speed( self.new_distance_aruco, self.new_rotation_angle, 0.3 ) # SPEED_Z
            self.robo.set_speed(speed)
            self.old_rotation_angle = self.new_rotation_angle
            #self._timer_move.cancel()
        


    def __turn__callback(self):
        if self.is_update_info():
            if abs(self.new_rotation_angle) < abs(self.old_rotation_angle):
                if abs(self.robo.turn_diff) < 5:
                    self.robo.turn_by_angle( self.new_rotation_angle / 2 )    
                else:
                    self.robo.turn_by_angle( self.new_rotation_angle )
        elif abs(self.new_rotation_angle) <= 0.1:
                self.robo.stop_turn()
                self._timer_turn.cancel()
                

    def __next_target_callback(self):
        if len(self.targets_list) > 0: 
            if self.new_action:
                self.new_action = False
                self.area_of_interest = False
                self.target_actual = self.targets_list.pop(0)
                self.robo.sensor_camera.fix_target(self.target_actual.id_destiny)
                angle_new_orientation = abs(self.robo.turn_by_orientation(self.target_actual.orientation))
                speed = 0.5
                if angle_new_orientation > 90 :
                    if self.robo.get_distance_to_wall() < 0.50:
                        speed = 0.1
                    else:
                        speed = 0.2
                elif angle_new_orientation > 60 :
                    if self.robo.get_distance_to_wall() < 0.50:
                        speed = 0.2
                    else:
                        speed = 0.3
                elif angle_new_orientation > 45 :
                    speed = 0.4
                self.robo.set_speed(speed)
            elif not self.robo.is_turnning(): # or (abs(self.robo.turn_diff) <= 5 and abs(self.robo.turn_diff) > 0.0):
                '''
                g = round(self.robo.turn_diff, 2)
                g = cor.green(g) if g > 0 else cor.red(g)
                print(cor.green("Agora já pode ir!!!!"), f'diferença de {g}°')
                '''
                self.robo.sensor_camera.track_aruco_target = True
                self.robo.set_speed(0.5)
                self._timer_target.cancel()
                self._timer_controll.reset()
        else:
            self._timer_target.cancel()
            self.node.get_logger().warning("Chegamos no destino!")
            self.robo.stop()
        

    def show_infos(self):
        a  = f'{self.new_rotation_angle:>7.2f}'
        if self.is_update_info():
            a  = cor.green(a) if self.new_rotation_angle > self.old_rotation_angle else cor.blue(a)
        else:
            a  = cor.yellow(a) 
        b  = cor.blue(f'{self.new_distance_aruco:>5.2f}')
        t  = f'{self.robo.get_speed():>5.2f}'
        x = self.robo.get_acceleration()
        if x == 0 :
            t = cor.blue(t)
        else:
            t = cor.green(t) if x > 0 else cor.red(t)
        x  = self.robo.get_distance_to_wall()
        x = cor.yellow(f'{x:>5.2f}') if x > 1 else cor.red(f'{x:>5.2f}')
        dy, dx = self.robo.sensor_lidar.find_closest_wall_angle()
        dx = cor.cyan(f'{dx:>5.2f}')
        dy = cor.magenta(f'{dy:>3}')
        print(f"ANGLE: {a}, DISTANCIA: {b}, SPEED: {t}, WALL: {x}, [ Wall: {dx},  angle: {dy} ]")

        id = f'{self.robo.sensor_camera.id_aruco_target}'
        r  = f"{cor.green('YES') if self.robo.sensor_camera.track_aruco_target else cor.red('NO')}, ID:  {cor.cyan(f'{id:>2}')}"
        turn = cor.red("OFF") if self._timer_turn is None or self._timer_turn.is_canceled() else cor.green(" ON")
        target = cor.red("OFF") if self._timer_target is None or self._timer_target.is_canceled() else cor.green(" ON")
        move = cor.red("OFF") if self._timer_move is None or self._timer_move.is_canceled() else cor.green(" ON")
        contr = cor.red("OFF") if self._timer_controll is None or self._timer_controll.is_canceled() else cor.green(" ON")
        walker = cor.red("OFF") if self._timer_walker is None or self._timer_walker.is_canceled() else cor.green(" ON")
        print(f' TURN: {turn}, TARGET: {target}, MOVE: {move}, CONTROLL: {contr}, WALKER: {walker}, FIXED_ALVO: {r}')
        print(f" ERRO TURN: {self.robo.turn_diff:>4}")
    # Lidar
    def handle_obstacle_detection(self):
        """
        Lógica para lidar com a detecção de obstáculos
        """
        pass
        #print("Algo detectado!")

    # Camera
    def handle_aruco_detected(self, distance_to_aruco, pixel_error):
        """
        ids detectados
        """
        self.old_rotation_angle  = self.new_rotation_angle
        self.old_distance_aruco  = self.new_distance_aruco
        self.new_rotation_angle  = pixel_error
        self.new_distance_aruco  = distance_to_aruco
        self.count_not_detected  = 0

    def is_update_info(self):
        return self.old_rotation_angle  == self.new_rotation_angle and self.old_distance_aruco  == self.new_distance_aruco
    
def ajuste_speed(distance, angle_rotation, speed_z):
    # Defina os limites de velocidade
    MAX_SPEED = 1.0
    MIN_SPEED = 0.1
    # Defina os limiares de distância e ângulo
    DISTANCE_THRESHOLD = 3.0  # Distância a partir da qual a velocidade começa a diminuir
    ANGLE_THRESHOLD = 5  # Ângulo a partir do qual a velocidade de rotação começa a diminuir

    # Ajusta a velocidade com base na distância
    if distance > DISTANCE_THRESHOLD:
        speed = MAX_SPEED
    else:
        # Reduz a velocidade à medida que se aproxima do ArUco
        speed = MIN_SPEED + (distance / DISTANCE_THRESHOLD) * (MAX_SPEED - MIN_SPEED)

    # Ajusta a velocidade com base no ângulo de rotação
    if abs(angle_rotation) > ANGLE_THRESHOLD:
        # Se o ângulo de rotação for muito grande, reduz ainda mais a velocidade
        speed *= 0.5

    # Ajusta a velocidade com base na velocidade atual (para evitar paradas bruscas)
    if speed_z > 0 and angle_rotation != 0:
        # Se estiver girando, ajuste a velocidade de parada com base na velocidade e ângulo
        stopping_time = abs(angle_rotation) / speed_z
        if stopping_time < 1:  # Se precisar parar em menos de 1 segundo, reduza a velocidade
            speed *= stopping_time

    return max(MIN_SPEED, min(speed, MAX_SPEED))  # Garante que a velocidade esteja dentro dos limites
        

# Função para obter o número da linha atual
def get_current_line_number():
    return inspect.currentframe().f_back.f_lineno

