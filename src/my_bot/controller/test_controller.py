from model import Robot
from utils import Command, CommandQueue, CommandType, Graph, Path
from utils.constants import CALLBACK_INTERVAL, WALL_COLID, LIDAR_RANGE_, WALL_MIN_SECURITY, MIN_SPEED
from utils.bib import Color as cor, get_current_line_number, is_OFF
from rclpy.node import Node
import math
import time

class TestController:
    '''
    Controlador responsável pela navegação do robô, utilizando a técnica de seguir parede.
    '''

    def __init__(self, node: Node):
        self.node = node
        self.robo = Robot(node, self.handle_obstacle_detection, self.handle_aruco_detected)
        self.robo.set_speed(0.5)
        self.robo.turn_by_angle(45)
        self._timer_walker = None
        self._timer_controll = None
        self._timer_colid_wall = None
        self.start_controll()
        #self.info = self.node.create_timer(CALLBACK_INTERVAL, self.info_lidar)

    def check_risk_of_collision(self, angle: int):
        """
        Verifica se há risco de colisão com base no ângulo fornecido.
        """
        return self.robo.sensor_lidar.check_collision_router(direction=angle)

    def suggest_turn_direction(self):
        """
        Analisa os dados do LiDAR para sugerir a melhor direção para virar.
        Retorna 'left', 'right' ou 'none'.
        """
        left_distances = self.robo.sensor_lidar.lidar_data__left
        right_distances = self.robo.sensor_lidar.lidar_data__right

        # Calcula a média das distâncias para a esquerda e direita
        average_left = sum(left_distances) / len(left_distances)
        average_right = sum(right_distances) / len(right_distances)

        if average_left > average_right:
            return 'left'
        elif average_right > average_left:
            return 'right'
        else:
            return 'none'

    def start_walker(self):
        ''' Inicia o modo de navegação para seguir a parede mais próxima. '''
        def __walker__callback():
            angle, min_wall = self.robo.sensor_lidar.find_closest_wall_angle()
            self.adjust_robot_orientation(angle,min_wall)

        if not self._timer_walker:
            self._timer_walker = self.node.create_timer(CALLBACK_INTERVAL, __walker__callback)

    def adjust_robot_orientation(self, angle, min_wall):
        ''' Ajusta a orientação do robô para manter uma distância segura da parede. '''
        if min_wall < ( WALL_MIN_SECURITY + WALL_COLID ):
            self.robo.set_speed(MIN_SPEED)
        else:
            self.robo.set_speed(MIN_SPEED*2)

        if abs(angle) >= 3 and min_wall <= WALL_MIN_SECURITY:
            self.robo.stop_turn()
            self.stop_walker()
            print(f"angle: {angle:>4} | min_wall: {min_wall:.3f} | SPEED: {self.robo.get_speed():.3f}", "adjust    ", cor.red(f'TURN: STOP'), get_current_line_number())
            self.start_controll()
            return

        turn_angle = _ajust_for_turn_angle(angle)
        print(f"angle: {angle:>4} | min_wall: {min_wall:.3f} | SPEED: {self.robo.get_speed():.3f}", "walker    ", cor.red(f'TURN: {turn_angle:>4}'), get_current_line_number())
        self.robo.turn_by_angle(turn_angle)

    def stop_walker(self):
        ''' Para o modo de navegação de seguir parede e retorna ao controle principal. '''
        if self._timer_walker:
            self._timer_walker.cancel()
        self.start_controll()

    def start_controll(self):
        ''' Inicia o controle principal do robô. '''
        def __execute_controll():
            angle, min_wall = self.robo.sensor_lidar.find_closest_wall_angle()
            #self.evaluate_wall_proximity(angle, min_wall)
            # Verifica o risco de colisão
            if self.check_risk_of_collision(0):  # Verifica para a frente
                # Sugerir uma direção para virar
                turn_direction = self.suggest_turn_direction()
                print(f"angle: {'1':>4} | min_wall: {min_wall:.3f} | SPEED: {self.robo.get_speed():.3f}", "proximity", cor.red(f'TURN: {turn_direction:>5}'), get_current_line_number())
                if turn_direction == 'left':
                    self.robo.turn_by_angle(-1)
                elif turn_direction == 'right':
                    self.robo.turn_by_angle(1)
                else:
                    # Nenhuma direção segura encontrada
                    if self.robo.get_state() == CommandType.STOP:
                        print("SOOOOCOOORROOOOO!")
                        self.start_walker()
                    else:
                        self.robo.stop()
            else:
                # Continua movendo-se para frente
                print("# Continua movendo-se para frente!")
                self.robo.set_speed(MIN_SPEED*3)

        if not self._timer_controll:
            self._timer_controll = self.node.create_timer(CALLBACK_INTERVAL, __execute_controll)

    def evaluate_wall_proximity(self, angle, min_wall):
        ''' Avalia a proximidade da parede e ajusta a velocidade e a orientação do robô. '''
        if min_wall < ( WALL_MIN_SECURITY + WALL_COLID ) and min_wall > WALL_MIN_SECURITY and abs(angle) >= 2:
            self.robo.stop_turn()
            self.robo.set_speed(MIN_SPEED)
            turn_angle = _ajust_for_turn_angle(angle)
            self.robo.turn_by_angle(turn_angle)
            print(f"angle: {angle:>4} | min_wall: {min_wall:.3f} | SPEED: {self.robo.get_speed():.3f}", "proximity", cor.red(f'TURN: {turn_angle:>4}'), get_current_line_number())
        elif min_wall < WALL_MIN_SECURITY and abs(angle) > 3:
            self.robo.stop_turn()
            self.switch_to_walker_mode()
        else:
            self.robo.set_speed(MIN_SPEED*3)

    def switch_to_walker_mode(self):
        ''' Alterna para o modo de navegação de seguir parede. '''
        if self._timer_controll:
            self._timer_controll.cancel()
        self.robo.set_speed(MIN_SPEED)
        self.start_walker()

    def handle_obstacle_detection(self):
        ''' Lida com a detecção de obstáculos. '''
        
        

    def start_colid_wall(self, angle: int):
        ''' Inicia o modo de navegação para evitar colisões com paredes. '''
        self.robo.stop_move()
        self.cancel_all_timers()
        turn_angle = _ajust_for_turn_angle(angle)
        self.robo.turn_by_angle(turn_angle)

        def __colid_wall__callback():
            if self.robo.sensor_lidar.check_collision_router(angle):
                self.handle_collision_detected()
            else:
                self.handle_no_collision_detected()

        self._timer_colid_wall = self.node.create_timer(CALLBACK_INTERVAL, __colid_wall__callback)

    def cancel_all_timers(self):
        ''' Cancela todos os timers ativos. '''
        if self._timer_walker:
            self._timer_walker.cancel()
        if self._timer_controll:
            self._timer_controll.cancel()

    def handle_collision_detected(self):
        ''' Lida com a detecção de colisão. '''
        self.robo.stop_turn()
        self.start_controll()

    def handle_no_collision_detected(self):
        ''' Lida com a ausência de detecção de colisão. '''
        self.continue_moving_forward()


    def handle_aruco_detected(self, distance_to_aruco, pixel_error):
        ''' Lida com a detecção de marcadores ArUco. '''
        # Implemente a lógica de processamento de marcadores ArUco aqui

    def info_lidar(self):
        left = 0
        right= 0
        zero = self.robo.get_distance_to_wall(0)
        print(f'left: {zero:>7.5f} : {0:>3} : {zero:>7.5f} :right')
        for i in range(1, LIDAR_RANGE_):
            left = -i
            right = i
            left = self.robo.get_distance_to_wall(left)
            right = self.robo.get_distance_to_wall(right)
            print(f'left: {left:>7.5f} : {i:>3} : {right:>7.5f} :right')
        
        al, dl = self.robo.sensor_lidar.find_closest_wall_angle(-6)
        ar, dr = self.robo.sensor_lidar.find_closest_wall_angle(6)
        print(f'angulo esq: {al:>2} direcao: {dl}')
        print(f'angulo esq: -6 direcao: {self.robo.get_distance_to_wall(-6)}')

        print(f'angulo dir: {ar:>3} direcao: {dr}')
        print(f'angulo dir:  6 direcao: {self.robo.get_distance_to_wall(6)}')

def _ajust_for_turn_angle(angle:int):
    new_angle = abs(angle) - LIDAR_RANGE_
    return new_angle if angle < 0 else -new_angle