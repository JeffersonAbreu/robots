from model import Robot, Navigation, Tracking
from utils import Command, CommandQueue, CommandType, Graph, Path
from utils.constants import CALLBACK_INTERVAL, CALLBACK_INTERVAL_TARGET, MIN_SPEED, TOP_SPEED
from utils.bib import normalize_angle2, get_current_line_number, is_ON, is_OFF, on_or_off
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
            self._not_get_id = True
            self.count_not_detected = 0
            self._timer_target   = None
            self._timer_controll = None
            self._timer_walker   = None
            self._timer_area_of_interest = None
            self._timers = [self._timer_target, self._timer_controll, self._timer_walker]
            
            self.nav.select_a_route()
            self.go_next()
            self._timer_show_inf = None
            self.robo.set_speed(MIN_SPEED)
        else:
            self.node.get_logger().error(f'Não foi localizado nenhuma rota!')

    def start_show(self):
        if self._timer_show_inf is None:
            self._timer_show_inf = self.node.create_timer(CALLBACK_INTERVAL, self.show_infos)
        else:
            self._timer_show_inf.reset()

    def start_target(self):
        if self._timer_target is None:
            self._timer_target = self.node.create_timer(CALLBACK_INTERVAL_TARGET, self.__target_callback)
        else:
            self._timer_target.reset()

    def start_walker(self):
        if self._timer_walker is None:
            self._timer_walker = self.node.create_timer(CALLBACK_INTERVAL, self.__walker__callback)
        else:
            self._timer_walker.reset()

    def start_controll(self):
        if self._timer_controll is None:
            self._timer_controll = self.node.create_timer(CALLBACK_INTERVAL, self.__execute_controll)
        else:
            self._timer_controll.reset()

    def stop_all_timers(self):
        for _timer in self._timers:
            if is_ON(_timer):
                _timer.cancel()

    
    def __execute_controll(self):
        if self.tracking.is_tracking():
            if self.is_area_of_interest():
                self.stop_all_timers()
                self.area_of_interest()
                return
            
            elif not self.is_update_info():
                self.count_not_detected  += 1
                if self.count_not_detected > 100 and not self.is_update_info():
                    print(cor.red(f"NOT DETECTED!!! {self.count_not_detected:>2}"), get_current_line_number())
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
    
    def is_area_of_interest(self) -> bool:
        '''
        Verifique se estou na aréa do aruco
        '''
        return self.tracking.new_distance_aruco < 1 and self.robo.get_distance_to_wall() < 1
                           

    def area_of_interest(self):
        self.robo.set_speed(MIN_SPEED*2)
        if is_ON(self._timer_show_inf):
            self._timer_show_inf.cancel()
        print(cor.red("area_of_interest!!!"), get_current_line_number())
        self.tracking.stop_tracking()
        self._not_get_id = True
        self._timer_area_of_interest    = self.node.create_timer(CALLBACK_INTERVAL, self.__area_of_interest__callback)

    def __area_of_interest__callback(self):
        speed = self.robo.get_speed()
        a = self.robo.get_acceleration()
        print("ACELERAÇÂO: ", cor.blue(a), "VELOCIDADE ATUAL: ", cor.yellow(speed), get_current_line_number())
        if a == -1:
            '''
            Aguarde até parar a fenagem
            '''
            return

        if self._not_get_id:
            self._timer_area_of_interest.cancel()        
            time.sleep(CALLBACK_INTERVAL)
            self.go_next()
    
    def go_next(self):
        if self.nav.is_next() and self._not_get_id:
            self.count_not_detected = 0
            self.start_controll()
            self._not_get_id = False
            target = self.nav.get_next()
            self.tracking.fix_target(target.id_destiny)
            self.robo.turn_by_orientation(target.orientation)
            self.start_target()
        else:
            self.node.get_logger().warning("Chegamos no destino!")
            self.robo.stop()
            self.stop_all_timers()
            self.tracking.stop_tracking()
     

    def __target_callback(self):
        '''
        Aguarda o alinhamento para o alvo ou quase
        '''
        def go():
            self.robo.stop_turn()
            self._timer_target.cancel()
            self.robo.set_speed(TOP_SPEED)
            self.tracking.start_tracking()
            #self.start_show()
            return
        
        def should_follow_aruco():
            """
            Determina se é hora de cancelar o giro e seguir o ArUco de forma proporcional.

            :param distancia: Distância até o ArUco em metros.
            :param angulo: Diferença angular em graus entre o ArUco e o centro da imagem da câmera.
            :return: Retorna True se for hora de seguir o ArUco, False caso contrário.
            """
            distance = round(self.tracking.new_distance_aruco, 2)
            angle = round(abs(self.tracking.new_rotation_angle), 2)
            # Define os limites máximo e mínimo para a distância.
            _MAX_DISTANCE = 7.5  # metros
            _MIN_DISTANCE = 1.0  # metros

            # Define os limites máximo e mínimo para o ângulo.
            _MAX_ANGLE = 45.0  # graus
            _MIN_ANGLE = 5.0   # graus

            # Calcula um fator de escala proporcional à distância
            scale_factor = round( (distance - _MIN_DISTANCE) / (_MAX_DISTANCE - _MIN_DISTANCE), 5 )

            # Calcula o limite de ângulo com base no fator de escala
            angle_limit = round( _MIN_ANGLE + (_MAX_ANGLE - _MIN_ANGLE) * (1 - scale_factor), 2 )

            print("scale_factor: ", cor.yellow(scale_factor), f" angle[ {cor.cyan(angle)} ] <= angle_limit[ {cor.blue(angle_limit)} ] = ", on_or_off(angle <= angle_limit))
            # Verifica se o ângulo atual está dentro do limite calculado
            return angle <= angle_limit

            
        if not self.robo.is_turnning():
            print(cor.red("__next_target_callback CANSELADO!!!"), get_current_line_number(-1))
            go()

        elif self.is_update_info_direction():
            if should_follow_aruco():
                print(cor.red("__next_target_callback CANSELADO!!!"), get_current_line_number(-1))
                go()
    

                        
                
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


    def show_infos(self):

        a  = '-'
        b  = '-'
        if self.is_update_info():
            a  = f'{self.tracking.new_rotation_angle:>7.2f}'
            a  = cor.green(a) if abs(self.tracking.new_rotation_angle) < abs(self.tracking.old_rotation_angle) else cor.red(a)

            b  = f'{self.tracking.new_distance_aruco:>7.2f}'
            b  = cor.green(a) if self.tracking.new_distance_aruco < self.tracking.old_distance_aruco else cor.blue(a)
        else:
            a  = cor.yellow(f'{a:>7}')
            b  = cor.yellow(f'{b:>7}')
        t  = f'{self.robo.get_speed():>7.2f}'
        x = self.robo.get_acceleration()
        if x == 0 :
            t = cor.blue(t)
        else:
            t = cor.green(t) if x > 0 else cor.red(t)
        x  = self.robo.get_distance_to_wall()
        x = cor.yellow(f'{x:>7.2f}') if x > 1 else cor.red(f'{x:>7.2f}')
        dy, dx = self.robo.sensor_lidar.find_closest_wall_angle()
        dx = cor.cyan(f'{dx:>5.2f}')
        dy = cor.magenta(f'{dy:>3}')

        i = f'{self.robo.sensor_camera.id_aruco_target}'
        i = cor.cyan(f'{i}')
        r  = f"{on_or_off(self.robo.sensor_camera.track_aruco_target)}"
        turn____ = on_or_off( is_ON( self.tracking._timer_turn ))
        target__ = on_or_off( is_ON( self._timer_target ))
        move____ = on_or_off( is_ON( self.tracking._timer_move ))
        controll = on_or_off( is_ON( self._timer_controll ))
        walker__ = on_or_off( is_ON( self._timer_walker ))
        z = f'{self.robo.turn_diff:>6.2f}'
        w = f"{cor.green('YES') if self.tracking.is_tracking() else cor.red(' NO')}"
        print(f" CONTROLL: {controll}    ID: {i:>7} FIXED_ALVO: {r} TRACKING ALVO? {w}")
        print(f"     TURN: {turn____} ANGLE: {a}  ERRO TURN: {cor.yellow(z)}°")
        print(f"     MOVE: {move____} SPEED: {t}   DISTANCE: {b}")
        print(f"   WALKER: {walker__}  WALL: {x}   MIN Wall: {dx},  angle: {dy} ]")
        print(f"   TARGET: {target__}")
        

        
        n = f'{self.count_not_detected:>3}x'
        h = f'not detected: {cor.red(n) if self.count_not_detected > 0 else cor.black(n)}'
        print("")
    
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
        return self.tracking.is_update_info()
    
    def is_update_info_direction(self):
        return self.tracking.is_update_info_direction()

    def destroy_app(self):
        self.node.destroy_node()