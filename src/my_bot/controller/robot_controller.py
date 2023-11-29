from model import Robot, Navigation, Tracking
from utils import Command, CommandQueue, CommandType, Graph, Path
from utils.constants import CALLBACK_INTERVAL
from utils.bib import normalize_angle2
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
        self.nav = Navigation(25, 1)
        if self.nav.is_exist_rote():
            self.robo = Robot(self.node, self.handle_obstacle_detection, self.handle_aruco_detected)
            self.tracking = Tracking(node, self.robo)
            self._not_get_id = True
            self.count_not_detected = 0
            self._timer_target   = None
            self._timer_move     = None
            self._timer_controll = None
            self._timer_walker   = None
            self._timer_area_of_interest = None
            self._timers = [self._timer_target, self._timer_move, self._timer_controll, self._timer_walker]
            
            self.nav.select_a_route()
            self.go_next()
            self._timer_show_inf = self.node.create_timer(CALLBACK_INTERVAL, self.show_infos)
        else:
            self.node.get_logger().error(f'Não foi localizado nenhuma rota!')

    def start_target(self, timer=CALLBACK_INTERVAL):
        self._timer_target  = self.node.create_timer(timer, self.__next_target_callback)
    def start_move(self, timer=CALLBACK_INTERVAL):
        self._timer_move    = self.node.create_timer(timer, self.__move__callback)
    def start_walker(self, timer=CALLBACK_INTERVAL):
        self._timer_walker       = self.node.create_timer(timer, self.__walker__callback)

    def stop_all_timers(self):
        for _timer in self._timers:
            if is_ON(_timer):
                _timer.cancel()

    def area_of_interest(self):
        if self.robo.get_distance_to_wall() < 0.3:
            self.robo.stop()
        elif (self.robo.get_distance_to_wall() < 0.8 and self.robo.get_distance_to_wall() > 0.5 ) or ( self.tracking.new_distance_aruco > 0.5 and self.tracking.new_distance_aruco < 0.8 ):
            self.robo.set_speed(0.2)
        elif self.robo.get_speed() > 0.1 and (self.robo.get_distance_to_wall() < 0.5 or (self.tracking.new_distance_aruco > 0 and self.tracking.new_distance_aruco < 0.5)):
            self.robo.set_speed(0.1)
        elif self.tracking.new_distance_aruco > 0.8:
            self.robo.set_speed(0.3)
        if self._timer_show_inf.is_ready():
            self._timer_show_inf.cancel()
        print(cor.red("area_of_interest!!!"), get_current_line_number())
        #self.tracking.stop_tracking()
        #self.robo.stop_turn()
        self._not_get_id = True
        self._timer_area_of_interest    = self.node.create_timer(CALLBACK_INTERVAL, self.__area_of_interest__callback)

    def __area_of_interest__callback(self):
        print(cor.blue(f"ACELERAÇÂO: {self.robo.get_acceleration()}"), get_current_line_number())
        if self.robo.get_distance_to_wall() > 0.7 and self.tracking.new_distance_aruco > 0.7:
            print(cor.red("__area_of_interest__callback!!!"), get_current_line_number())
            self.show_infos()
            return
        elif self.robo.get_speed() > 0.3:
            print(cor.red("__area_of_interest__callback!!!"), get_current_line_number())
            self.show_infos()
            return
        if self.tracking.new_distance_aruco < 0.3 or self.robo.get_distance_to_wall() < 0.3:
            print(cor.red("__area_of_interest__callback!!!"), get_current_line_number())
            self.show_infos()
            self.robo.stop()
        
        self._timer_area_of_interest.cancel()
        
        print(cor.blue(f"timer_area_of_interest.cancel()"), get_current_line_number())
        if self._timer_show_inf.is_canceled():
            self._timer_show_inf.reset()
            print(cor.blue(f"self._timer_show_inf.reset()"), get_current_line_number())
        print(cor.blue(f"self._timer_show_inf.reset()"), get_current_line_number())
        if self._not_get_id:
            self.go_next()

    
    def __execute_controll(self):
        if self.robo.sensor_camera.track_aruco_target and self.tracking.is_tracking():
            if self.tracking.new_distance_aruco < 1 and self.robo.get_distance_to_wall() < 1:
                self.stop_all_timers()
                self.area_of_interest()
            elif not self.is_update_info():
                self.count_not_detected  += 1
                if self.count_not_detected > 100 and not self.is_update_info():
                    print(cor.red("PARAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA!!!"), get_current_line_number())
                    self.stop_all_timers()
                    self.robo.set_speed(0)
                    self.start_walker()
        if self.robo.get_speed() > 0 and self.robo.get_distance_to_wall() < 0.3:
            print(cor.red("PARAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA!!!"), get_current_line_number())
            self.robo.stop()
            self.stop_all_timers()
        angle, min_wall = self.robo.sensor_lidar.find_closest_wall_angle()

        if min_wall < 0.3 and (angle >= -45 or angle <= 45):
            print(cor.red("PARAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA!!!"), get_current_line_number())
            print(f"angle: {angle} e min_wall: {min_wall}")
            self.robo.stop()
            self.stop_all_timers()
        if self.robo.get_state() == CommandType.STOP and is_OFF(self._timer_controll):
            print(cor.red("Waaaaaalllllkkkkkkkeeeeeerrrrrrrrrrrrrr!!!"), get_current_line_number())
            self.start_walker()
                           
                    
                
    def __walker__callback(self):
        angle, min_dist = self.robo.sensor_lidar.find_closest_wall_angle(self.tracking.old_rotation_angle)
        if angle == 0 :
            self.robo.stop_turn()
        if min_dist < 0.5:
            if angle != 0:
                esq = self.robo.get_distance_to_wall(angle - 1)
                dir = self.robo.get_distance_to_wall(angle + 1)
                angle = abs(dir) - abs(esq)
                self.robo.turn_by_angle(angle)
            self.robo.move_backward(0.1)
            return 
        self.robo.set_speed(0)
        if self.is_update_info():
            self._timer_walker.cancel()
            time.sleep(1.5)
            self.robo.stop()
            self.tracking.start_tracking()
            self._timer_controll.reset()
            return
        else:
            # se chegou aqui está predido
            self.robo.turn_by_angle(self.tracking.old_rotation_angle)


    def __move__callback(self):
        """
        Executa os comandos na fila.
        """
        if self.is_update_info_direction():
            if abs(self.tracking.new_rotation_angle) > abs(self.tracking.old_rotation_angle):
                speed = 0.2
            else:
                speed = ajuste_speed( self.tracking.new_distance_aruco, self.tracking.new_rotation_angle, 0.3 ) # SPEED_Z
            self.robo.set_speed(speed)            
            #self._timer_move.cancel()
     

    def __next_target_callback(self):
        '''
        Aguarda o alinhamento para o alvo ou quase
        '''
        def go():
            self._timer_target.cancel()
            self.robo.set_speed(0.7)
            if self._timer_controll is None:
                self._timer_controll = self.node.create_timer(CALLBACK_INTERVAL, self.__execute_controll)
            else:
                self._timer_controll.reset()
            self.tracking.start_tracking()
            return
            
        if not self.robo.is_turnning():
            print(cor.red("__next_target_callback CANSELADO!!!"), get_current_line_number())
            go()
        elif self.is_update_info_direction():
            dist = self.tracking.new_distance_aruco
            diff = abs(self.robo.turn_diff)
            if diff <= 0.5 and dist < 2:
                print(cor.red("__next_target_callback CANSELADO!!!"), get_current_line_number())
                go()
            elif dist < 5.00:
                if dist >= 4.50 and diff <= 1:
                    print(cor.red("__next_target_callback CANSELADO!!!"), get_current_line_number())
                    go()
                elif dist >= 4.00 and diff <= 2:
                    print(cor.red("__next_target_callback CANSELADO!!!"), get_current_line_number())
                    go()
                elif dist >= 3.00 and diff <= 6:
                    print(cor.red("__next_target_callback CANSELADO!!!"), get_current_line_number())
                    go()
                elif dist >= 2.00 and diff <= 10:
                    print(cor.red("__next_target_callback CANSELADO!!!"), get_current_line_number())
                    go()
                elif dist < 2.00 and diff <= 15:
                    print(cor.red("__next_target_callback CANSELADO!!!"), get_current_line_number())
                    go()
        

    def show_infos(self):
        a  = f'{self.tracking.new_rotation_angle:>7.2f}'
        if self.is_update_info():
            a  = cor.green(a) if self.tracking.new_rotation_angle > self.tracking.old_rotation_angle else cor.blue(a)
        else:
            a  = cor.yellow(a) 
        b  = cor.blue(f'{self.tracking.new_distance_aruco:>5.2f}')
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
        r  = f"{cor.green('YES') if self.robo.sensor_camera.track_aruco_target else cor.red(' NO')}, ID:  {cor.cyan(f'{id:>2}')}"
        turn = cor.red("OFF") if not is_ON( self.tracking._timer_turn ) else cor.green(" ON")
        target = cor.red("OFF") if not is_ON( self._timer_target ) else cor.green(" ON")
        move = cor.red("OFF") if not is_ON( self._timer_move ) else cor.green(" ON")
        contr = cor.red("OFF") if not is_ON( self._timer_controll ) else cor.green(" ON")
        walker = cor.red("OFF") if not is_ON( self._timer_walker ) else cor.green(" ON")
        print(f' TURN: {turn}, TARGET: {target}, MOVE: {move}, CONTROLL: {contr}, WALKER: {walker}, FIXED_ALVO: {r}')

        w = f"TRACKING ALVO? {cor.green('YES') if self.tracking.is_tracking() else cor.red(' NO')}"
        z = f'{self.robo.turn_diff:>5.2f}'
        h = f'QTD not detected: {self.count_not_detected:>3}'
        print(f" ERRO TURN: {cor.yellow(z)}° {w}")
    
    # handle Lidar
    def handle_obstacle_detection(self):
        """
        Lógica para lidar com a detecção de obstáculos
        """
        pass
        #print("Algo detectado!")

    # handle Camera
    def handle_aruco_detected(self, distance_to_aruco, angle_error):
        """
        ids detectados
        """
        self.count_not_detected  = 0
        self.tracking.handle_aruco_detected(distance_to_aruco, angle_error)
        


    def is_update_info(self):
        return self.tracking.old_rotation_angle  != self.tracking.new_rotation_angle and self.tracking.old_distance_aruco  != self.tracking.new_distance_aruco
    def is_update_info_direction(self):
        return self.tracking.old_rotation_angle  != self.tracking.new_rotation_angle


    def go_next(self):
        print(cor.blue(f"def go_next(self):"), get_current_line_number())
        if self.nav.is_next() and self._not_get_id:
            self._not_get_id = False
            print(cor.blue(f"Debug: "), get_current_line_number())
            target = self.nav.get_next()
            self.tracking.fix_target(target.id_destiny)
            angle_new_orientation = abs(self.robo.turn_by_orientation(target.orientation))
            speed = 0.5
            if angle_new_orientation > 90 :
                print(cor.blue(f"Debug: "), get_current_line_number())
                if self.robo.get_distance_to_wall() < 0.50:
                    print(cor.blue(f"Debug: "), get_current_line_number())
                    speed = 0.1
                else:
                    print(cor.blue(f"Debug: "), get_current_line_number())
                    speed = 0.2
            elif angle_new_orientation > 60 :
                print(cor.blue(f"Debug: "), get_current_line_number())
                if self.robo.get_distance_to_wall() < 0.50:
                    print(cor.blue(f"Debug: "), get_current_line_number())
                    speed = 0.2
                else:
                    print(cor.blue(f"Debug: "), get_current_line_number())
                    speed = 0.3
            elif angle_new_orientation > 45 :
                    print(cor.blue(f"Debug: "), get_current_line_number())
                    speed = 0.4
            self.robo.set_speed(speed)
            print(cor.blue(f"Debug: "), get_current_line_number())
            self.start_target()
            print(cor.blue(f"Debug: "), get_current_line_number())
        else:
            print(cor.blue(f"Debug: "), get_current_line_number())
            self.node.get_logger().warning("Chegamos no destino!")
            self.robo.stop()


def is_ON(thread):
    return False if thread is None or thread.is_canceled() else True

def is_OFF(thread):
    return not ( is_ON(thread) )


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
        

def get_current_line_number():
    '''
    Função para obter o número da linha atual de onde ela é chamada
    '''
    return f"Line: {cor.yellow(inspect.currentframe().f_back.f_lineno)}"