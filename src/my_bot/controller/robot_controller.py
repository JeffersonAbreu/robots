from model import Robot, Navigation, Tracking
from utils import Command, CommandQueue, CommandType, Graph, Path
from utils.constants import CALLBACK_INTERVAL, CALLBACK_INTERVAL_TARGET, MIN_SPEED, TOP_SPEED, ZERO
from utils.bib import get_current_line_number as ondeTO, is_ON, is_OFF, on_or_off, yes_or_no
from utils.bib import Color as cor
from rclpy.node import Node
import pdb # pdb.set_trace() # serve para debugar
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
        self.nav = Navigation(25, 30)
        if self.nav.is_exist_rote():
            self.robo = Robot(self.node, self.handle_obstacle_detection, self.handle_aruco_detected)
            self.tracking = Tracking(node, self.robo)
            self._timer_target   = None
            self._timer_controll = None
            self._timer_walker   = None
            self._timer_show_inf = None
            self._timer_area_of_interest = None
            self._timers = [self._timer_target, self._timer_controll, self._timer_walker]
            
            self.nav.select_a_route()
            self.robo.set_speed(MIN_SPEED)
            self.go_next()
            #self.tracking.fix_target(26)
            #self.tracking.start_tracking()
            self.start_show()        
        else:
            self.node.get_logger().error(f'Não foi localizado nenhuma rota!')
    
    def start_show(self):
        ondeTO()
        self._timer_show_inf = self.node.create_timer(CALLBACK_INTERVAL, self.show_infos)

    def start_target(self):
        ondeTO()
        self._timer_target = self.node.create_timer(CALLBACK_INTERVAL_TARGET, self.__target_callback)
        
    def start_walker(self):
        ondeTO()
        self._timer_walker = self.node.create_timer(CALLBACK_INTERVAL, self.__walker__callback)
        
    def start_controll(self):
        ondeTO()
        self._timer_controll = self.node.create_timer(CALLBACK_INTERVAL, self.__execute_controll)
        
    def stop_all_timers(self):
        ondeTO()
        for _timer in self._timers:
            if is_ON(_timer):
                _timer.cancel()

    
    def __execute_controll(self):
        lidar_detected_front = self.robo.get_distance_to_wall()
        if lidar_detected_front > 1 and not self.robo.is_turnning():
            self.robo.set_speed(TOP_SPEED)
            if self.tracking.not_is_discovered:
                # se não achou nada cotinue
                return

        if self.tracking.are_you_tracking():
            ondeTO()
            if self.is_area_of_interest():
                ondeTO()
                self.stop_all_timers()
                self.area_of_interest()
                return
            if self.robo.get_speed() < MIN_SPEED:
                self.robo.set_speed(MIN_SPEED)
            
            elif self.tracking.count_not_detected > 100:
                    ondeTO()
                    print(cor.red(f"NOT DETECTED!!! {self.tracking.count_not_detected:>2}"))
                    self.stop_all_timers()
                    self.robo.set_speed(ZERO)
                    #self.start_walker()
        if self.robo.get_speed() > 0 and self.robo.get_distance_to_wall() < 0.3:
            self.show_infos()
            print(cor.red("PARAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA!!!"))
            self.robo.stop()
            ondeTO()
            self.stop_all_timers()

        angle, min_wall = self.robo.sensor_lidar.find_closest_wall_angle()

        if min_wall < 0.3 and abs(angle) in (0, 1):
            self.show_infos()
            print(cor.red("PARAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA!!!"), ondeTO())
            print(f"angle: {angle} e min_wall: {min_wall}")
            self.robo.stop()
            ondeTO()
            self.stop_all_timers()
        if self.robo.get_state() == CommandType.STOP and is_OFF(self._timer_controll):
            print(cor.red("Waaaaaalllllkkkkkkkeeeeeerrrrrrrrrrrrrr!!!"), ondeTO())
            self.start_walker()
    
    def is_area_of_interest(self) -> bool:
        '''
        Verifique se estou na aréa do aruco
        '''
        return self.tracking.new_distance_aruco < 1 and self.robo.get_distance_to_wall()
                           

    def area_of_interest(self):
        ondeTO()
        self.robo.set_speed(MIN_SPEED)
        if is_ON(self._timer_show_inf):
            self._timer_show_inf.cancel()
        print(cor.red("area_of_interest!!!"), ondeTO())
        self.tracking.stop_tracking()
        self._timer_area_of_interest    = self.node.create_timer(CALLBACK_INTERVAL, self.__area_of_interest__callback)

    def __area_of_interest__callback(self):
        speed = self.robo.get_speed()
        a = self.robo.get_acceleration()
        print("ACELERAÇÂO: ", cor.blue(a), "VELOCIDADE ATUAL: ", cor.yellow(speed), ondeTO())
        if a == -1:
            ondeTO()
            '''Aguarde até parar a fenagem'''
            return
        self._timer_area_of_interest.cancel()        
        time.sleep(CALLBACK_INTERVAL)
        ondeTO()
        self.go_next()
    
    def go_next(self):
        ondeTO()
        if self.nav.is_next():
            ondeTO()
            target = self.nav.get_next()
            self.tracking.fix_target(target.id_destiny)
            self.robo.turn_by_orientation(target.orientation)
            self.start_show()
            self.start_target()
        else:
            self.node.get_logger().warning("Chegamos no destino!")
            self.robo.stop()
            ondeTO()
            self.stop_all_timers()
            self.tracking.stop_tracking()
    
    def __target_callback(self):
        '''
        Aguarda o alinhamento para o alvo ou quase
        '''
        def go():
            ondeTO()
            self._timer_target.cancel()
            self.robo.stop_turn()
            self.robo.set_speed(TOP_SPEED) # velocidade média
            self.tracking.start_tracking()
            self.start_controll()
            return
        
        def should_follow_aruco():
            """
            Determina se é hora de cancelar o giro e seguir o ArUco de forma proporcional.

            :param distancia: Distância até o ArUco em metros.
            :param angulo: Diferença angular em graus entre o ArUco e o centro da imagem da câmera.
            :return: Retorna True se for hora de seguir o ArUco, False caso contrário.
            """
            if not self.is_update_info_direction():
                ondeTO()
                return False
            distance = round(self.tracking.new_distance_aruco, 2)
            angle = round(abs(self.tracking.new_rotation_angle), 2)
            # Define os limites máximo e mínimo para a distância.
            _MAX_DISTANCE = 7.5  # metros
            _MIN_DISTANCE = 1.0  # metros

            # Define os limites máximo e mínimo para o ângulo.
            _MAX_ANGLE = 45.0  # graus
            _MIN_ANGLE = 0.0   # graus

            # Calcula um fator de escala proporcional à distância
            scale_factor = round( (distance - _MIN_DISTANCE) / (_MAX_DISTANCE - _MIN_DISTANCE), 5 )

            # Calcula o limite de ângulo com base no fator de escala
            angle_limit = _MIN_ANGLE + (_MAX_ANGLE - _MIN_ANGLE) * (1 - scale_factor)

            # Verifica se o ângulo atual está dentro do limite calculado
            return angle <= angle_limit

            
        if not self.robo.is_turnning() or should_follow_aruco():
            ondeTO()
            print(cor.red("__next_target_callback CANSELADO!!!"))
            go()
        else:
            if self.is_update_info_direction():
                if abs(self.tracking.new_rotation_angle) > abs(self.tracking.old_rotation_angle):
                    go()
        if abs(int(self.robo.turn_diff)) <= 10 and self.robo.get_speed() == MIN_SPEED:
            self.robo.set_speed(0.2)
        if self.robo.turn_diff < 0 and self.robo.get_distance_to_wall(-2) > 0.3 and self.robo.get_speed() == MIN_SPEED:
            self.robo.set_speed(0.2)
        if self.robo.turn_diff > 0 and self.robo.get_distance_to_wall(2) > 0.3 and self.robo.get_speed() == MIN_SPEED:
            self.robo.set_speed(0.2)
        if self.robo.get_speed() < MIN_SPEED:
            self.robo.set_speed(MIN_SPEED)
        

        
    def __walker__callback(self):
        ondeTO()
        angle, min_dist = self.robo.sensor_lidar.find_closest_wall_angle(self.tracking.old_rotation_angle)
        if angle == 0 :
            ondeTO()
            self.robo.stop_turn()
        if min_dist < 0.5:
            ondeTO()
            if angle != 0:
                esq = self.robo.get_distance_to_wall(angle - 1)
                dir = self.robo.get_distance_to_wall(angle + 1)
                angle = abs(dir) - abs(esq)
                self.robo.turn_by_angle(angle)
            self.robo.move_backward(0.1)
            return 
        #self.robo.set_speed(0)
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


    def show_infos(self):

        a  = '-'
        b  = '-'
        if self.is_update_info():
            a  = f'{self.tracking.new_rotation_angle:>7.2f}'
            a  = cor.green(a) if abs(self.tracking.new_rotation_angle) < abs(self.tracking.old_rotation_angle) else cor.red(a)
            b  = f'{self.tracking.new_distance_aruco:>6.2f}'
            b  = cor.green(b) if self.tracking.new_distance_aruco < self.tracking.old_distance_aruco else cor.blue(b)
        else:
            a  = cor.yellow(f'{a:>7}')
            b  = cor.yellow(f'{b:>6}')
        t  = f'{self.robo.get_speed():>7.2f}'
        x = self.robo.get_acceleration()
        t = cor.blue(t) if x == 0 else (cor.green(t) if x > 0 else cor.red(t))
        x  = self.robo.get_distance_to_wall()
        x = cor.yellow(f'{x:>7.2f}') if x > 1 else cor.red(f'{x:>7.2f}')
        dy, dx = self.robo.sensor_lidar.find_closest_wall_angle()
        dx = cor.cyan(f'{dx:>6.2f}')
        dy = cor.magenta(f'{dy:>5}')

        id = f"{cor.black('[')} {cor.cyan(f'{self.robo.sensor_camera.id_aruco_target:>3}')} {cor.black(']')}"
        turn     = on_or_off( is_ON( self.tracking._timer_turn ))
        move     = on_or_off( is_ON( self.tracking._timer_move ))
        target   = on_or_off( is_ON( self._timer_target ))
        controll = on_or_off( is_ON( self._timer_controll ))
        walker   = on_or_off( is_ON( self._timer_walker ))
        z = cor.yellow(f'{self.robo.turn_diff:>6.2f}°')
        w = f"{yes_or_no(self.tracking.are_you_tracking()):>6}"
        n = f'{self.tracking.count_not_detected:>2}x' if self.tracking.are_you_tracking() else '-'
        h = f'   não detectou: {cor.red(n) if self.tracking.count_not_detected > 0 else cor.black(n)}'
        
        print(f" CONTROLL: {controll}    ID: {id}   RASTREAR? {w }")
        print(f"     TURN: {turn    } ANGLE: {a }  ERRO TURN: {z }")
        print(f"     MOVE: {move    } SPEED: {t }   DISTANCE: {b }")
        print(f"   WALKER: {walker  }  WALL: {x }   MIN Wall: {dx}  angle: {dy} ]")
        print(f"   TARGET: {target  }{h } 1ª Detecção: {yes_or_no(not self.tracking.not_is_discovered)}")
        print("")
    
    # handle Lidar
    def handle_obstacle_detection(self):
        """
        Lógica para lidar com a detecção de obstáculos
        """

    # handle Camera
    def handle_aruco_detected(self, distance_to_aruco, angle_error):
        """
        ids detectados
        """
        self.tracking.handle_aruco_detected(distance_to_aruco, angle_error)

    def is_update_info(self):
        return self.tracking.is_update_info()
    
    def is_update_info_direction(self):
        return self.tracking.is_update_info_direction()

    def destroy_app(self):
        self.stop_all_timers()
        self.node.destroy_node()