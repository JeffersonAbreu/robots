from model import Robot
import math
import time
from utils.bib import is_ON, is_OFF, ajuste_speed
from utils.constants import CALLBACK_INTERVAL, FACTOR_CORRECTION_TURN, TOP_SPEED, MIN_SPEED

class Tracking:
    
    def __init__(self, node, robo: Robot):
        self.node = node
        self.robo:Robot = robo
        self.new_rotation_angle  = 0
        self.new_distance_aruco  = 0
        self.old_rotation_angle  = 0
        self.old_distance_aruco  = 0
        self.count_not_detected  = 0
        self.tracking            = False
        self.update              = False
        self._timer_turn = None
        self.old_diff = 0
        self.not_is_discovered = True
        self.ajust_turn_value = 0 
        '''se ainda não foi detectado pela primeira vez'''
    
    def start_turn(self, timer=CALLBACK_INTERVAL): #0.0001
        def __turn__callback():
            if self.tracking:
                self.ajust_turn(self.new_rotation_angle, self.robo.get_speed(), self.new_distance_aruco)
                '''
                if self.robo.get_speed() > TOP_SPEED*0.5:
                    turn = self.ajust_turn(self.new_rotation_angle, self.robo.get_speed(), self.new_distance_aruco)
                    self.robo.turn_by_angle(turn * FACTOR_CORRECTION_TURN)
                elif self.robo.get_speed() > TOP_SPEED*0.25:
                    self.robo.turn_by_angle( self.new_rotation_angle * FACTOR_CORRECTION_TURN)
                else:
                    self.robo.turn_by_angle( self.new_rotation_angle * FACTOR_CORRECTION_TURN * 2)
                '''
                self.old_diff = self.robo.turn_diff
            self._timer_turn.cancel()
            self._timer_turn = None
    
        self._timer_turn = self.node.create_timer(timer, __turn__callback)

    def ajust_turn(self, angle_error, speed_robo, distance_to_tag):
        # Constantes para ajustar a sensibilidade
        MAX_SPEED = 1.0  # Velocidade máxima
        MAX_DISTANCE = 10.0  # Distância máxima considerada
        MIN_DISTANCE = 1.0   # Distância mínima considerada

        # Dinamicamente ajustar o MAX_ANGLE_ERROR com base na distância
        if distance_to_tag < MIN_DISTANCE:
            MAX_ANGLE_ERROR = 10.0  # Erro angular máximo para distâncias muito próximas
        elif distance_to_tag > MAX_DISTANCE:
            MAX_ANGLE_ERROR = 45.0  # Erro angular máximo para distâncias maiores
        else:
            # Escala linear entre os valores mínimo e máximo com base na distância
            MAX_ANGLE_ERROR = 10.0 + (45.0 - 10.0) * ((distance_to_tag - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE))

        # Calcula fatores de ajuste com base na velocidade, distância e erro angular
        speed_factor = 1 - (speed_robo / MAX_SPEED)
        distance_factor = 1 - (distance_to_tag / MAX_DISTANCE)
        angle_error_factor = 1 - (abs(angle_error) / MAX_ANGLE_ERROR)

        # Combina os fatores para calcular o ângulo ajustado
        # Quanto maior a velocidade, distância ou erro angular, menor será o ângulo ajustado
        adjusted_angle = angle_error * speed_factor * distance_factor * angle_error_factor

        # Aplica o giro com o ângulo ajustado
        self.robo.turn_by_angle(adjusted_angle)
        if self.new_distance_aruco < 5:
            self.robo.set_speed(max(MIN_SPEED*2, TOP_SPEED*2 - abs(adjusted_angle)))
        else:
            self.robo.set_speed(max(MIN_SPEED*2, TOP_SPEED - abs(adjusted_angle)))
        
        self.ajust_turn_value = adjusted_angle

    
    def reset(self) -> None:
        self.new_rotation_angle  = 0
        self.new_distance_aruco  = 0
        self.old_rotation_angle  = 0
        self.old_distance_aruco  = 0
        self.old_diff            = 0
        self.tracking            = False
        self.update              = False
        self.not_is_discovered   = True
        self.update_tracking_camera()

    def start_tracking(self):
        self.tracking = True
        self.not_is_discovered = True
        self.update_tracking_camera()

    def update_tracking_camera(self):
        '''Colorir a detecção do aruco'''
        self.robo.sensor_camera.track_aruco_target = self.tracking

    def stop_tracking(self):
        self.reset()

    def are_you_tracking(self) -> bool:
        return self.tracking

    def fix_target(self, id:int) -> None:
        self.stop_tracking()
        self.robo.sensor_camera.fix_target(id)

    def handle_aruco_detected(self, distance, angle_error):
        """
        detectado
        """
        if distance == 0 and angle_error == 0:
            self.update = False
            if self.are_you_tracking():
                self.count_not_detected  += 1
        else:
            self.old_rotation_angle  = self.new_rotation_angle
            self.old_distance_aruco  = self.new_distance_aruco
            self.new_rotation_angle  = angle_error
            self.new_distance_aruco  = distance
            self.update = True
            self.count_not_detected  = 0
            if self.are_you_tracking() and is_OFF(self._timer_turn):
                self.start_turn()
                if self.not_is_discovered: # and abs(self.new_distance_aruco) < 2:
                #if self.not_is_discovered and self.robo.get_distance_to_wall() != float('inf'):
                    self.not_is_discovered = False

    
        

    

    def terorema_de_pitagoras(self, direction=0):
        def calc_side_and_angle(hypotenuse, side_a):
            """
            Calcula o lado e o ângulo em um triângulo retângulo dado a hipotenusa e o outro lado.

            :param hipotenusa: O comprimento da hipotenusa do triângulo retângulo.
            :param side_a    : O comprimento de um dos lados do triângulo retângulo.
            :return: Um tuple contendo o comprimento do outro lado e o ângulo oposto a ele em graus.
            """

            # Calcula o lado menor usando o Teorema de Pitágoras
            side_b = math.sqrt(hypotenuse**2 - side_a**2)

            # Calcula o ângulo em radianos usando a função arco cosseno (acos)
            angulo_radianos = math.acos(side_a / hypotenuse)

            # Converte o ângulo de radianos para graus
            angulo_graus = math.degrees(angulo_radianos)

            return side_b, angulo_graus

        hypotenuse = self.robo.get_distance_to_wall(direction)
        left  = self.robo.get_distance_to_wall(-1)
        right = self.robo.get_distance_to_wall(+1)
        direction = -1 if left < right else +1
        _, side_a = self.robo.sensor_lidar.find_closest_wall_angle(direction)
        side_b, angle = calc_side_and_angle(hypotenuse, side_a)
        return side_b, angle
    
    def is_update_info(self):
        return self.update
    
    def is_update_info_direction(self):
        return self.update and round(self.old_rotation_angle, 2)  != round(self.new_rotation_angle, 2)

