from model import Robot
import math
import time
from utils.bib import is_ON, is_OFF, ajuste_speed
from utils.constants import CALLBACK_INTERVAL, FACTOR_CORRECTION_TURN

class Tracking:
    
    def __init__(self, node, robo: Robot):
        self._id = None
        self.node = node
        self.robo:Robot = robo
        self.new_rotation_angle  = 0
        self.new_distance_aruco  = 0
        self.old_rotation_angle  = self.new_rotation_angle
        self.old_distance_aruco  = self.new_distance_aruco
        self.tracking = False
        self._timer_turn = None
        self._timer_move = None
        self.old_diff = 0
    
    def start_turn(self, timer=CALLBACK_INTERVAL): #0.0001
        def __turn__callback():
            if self.robo.sensor_camera.track_aruco_target:  
                '''
                if abs(self.robo.turn_diff) > abs(self.old_diff):
                    self.robo.stop_turn()
                '''
                #if not self.robo.is_turnning() or ( self.new_distance_aruco < 2 and abs(self.robo.turn_diff) < 5 ):
                   # if abs(self.new_rotation_angle) < abs(self.old_rotation_angle):
                self.robo.turn_by_angle( self.new_rotation_angle * FACTOR_CORRECTION_TURN)
                       # self.robo.set_speed( self.robo.get_speed() + 0.05 )
                    #else:
                       # self.robo.turn_by_angle( self.old_rotation_angle )
                        #self.robo.set_speed( self.robo.get_speed() - 0.1 )
                self.old_diff = self.robo.turn_diff
                self._timer_turn.cancel()
            else:
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
                if abs(self.new_rotation_angle) > abs(self.old_rotation_angle):
                    speed = 0.2
                else:
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
        self._id                 = None
        if is_ON(self._timer_turn):
            self._timer_turn.cancel()
        if is_ON(self._timer_move):
            self._timer_move.cansel()

    def start_tracking(self):
        self.tracking = True

    def stop_tracking(self):
        self.reset()

    def is_tracking(self) -> bool:
        return self.tracking

    def fix_target(self, id:int) -> None:
        self.stop_tracking()
        self._id = id
        self.robo.sensor_camera.fix_target(id)

    def handle_aruco_detected(self, distance, angle_error):
        """
        detectado
        """
        self.old_rotation_angle  = self.new_rotation_angle
        self.old_distance_aruco  = self.new_distance_aruco
        self.new_rotation_angle  = angle_error
        self.new_distance_aruco  = distance
        if self.tracking and is_OFF(self._timer_turn):
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
        return self.is_update_info_direction() and round(self.old_distance_aruco, 2)  != round(self.new_distance_aruco, 2)
    
    def is_update_info_direction(self):
        return round(self.old_rotation_angle, 2)  != round(self.new_rotation_angle, 2)

