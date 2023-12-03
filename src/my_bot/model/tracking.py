from model import Robot
import math
import time
from utils.bib import is_ON, is_OFF, ajuste_speed
from utils.constants import CALLBACK_INTERVAL, FACTOR_CORRECTION_TURN

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
        self._timer_move = None
        self.old_diff = 0
    
    def start_turn(self, timer=CALLBACK_INTERVAL): #0.0001
        def __turn__callback():
            if self.tracking:  
                self.robo.turn_by_angle( self.new_rotation_angle * FACTOR_CORRECTION_TURN)
                self.old_diff = self.robo.turn_diff
            self._timer_turn.cancel()
    
        if self._timer_turn is not None:
            self._timer_turn.reset()
        else:
            self._timer_turn = self.node.create_timer(timer, __turn__callback)
    
    def start_move(self):
        def __move__callback():
            """
            Executa os comandos na fila.
            """
            if self.is_update_info_direction():
                speed = ajuste_speed( self.new_distance_aruco, self.new_rotation_angle, self.robo.get_speed() ) # SPEED_Z
                self.robo.set_speed(speed)
            self._timer_move.cancel()


        if self._timer_move is not None:
            self._timer_move.reset()
        else:
            self._timer_move = self.node.create_timer(CALLBACK_INTERVAL, __move__callback)

    def reset(self) -> None:
        self.new_rotation_angle  = 0
        self.new_distance_aruco  = 0
        self.old_rotation_angle  = 0
        self.old_distance_aruco  = 0
        self.old_diff            = 0
        self.tracking            = False
        self.update              = False
        self.update_tracking_camera()
        if is_ON(self._timer_turn):
            self._timer_turn.cancel()
        if is_ON(self._timer_move):
            self._timer_move.cansel()

    def start_tracking(self):
        self.tracking = True
        self.update_tracking_camera()

    def update_tracking_camera(self):
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

