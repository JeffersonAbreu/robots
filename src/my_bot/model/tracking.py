from model import Robot
import math
import time
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

def is_ON(thread):
    return False if thread is None or thread.is_canceled() else True

def is_OFF(thread):
    return not ( is_ON(thread) )


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
        self.old_diff = 0
    
    def start_turn(self, timer=0.0001): #0.0001
        def __turn__callback():
            if self.robo.sensor_camera.track_aruco_target:  
                if abs(self.robo.turn_diff) > abs(self.old_diff):
                    self.robo.stop_turn()
                if not self.robo.is_turnning() or ( self.new_distance_aruco < 2 and abs(self.robo.turn_diff) < 5 ):
                    if abs(self.new_rotation_angle) < abs(self.old_rotation_angle):
                        self.robo.turn_by_angle( self.new_rotation_angle )
                        self.robo.set_speed( self.robo.get_speed() + 0.05 )
                    else:
                        self.robo.turn_by_angle( self.old_rotation_angle )
                        self.robo.set_speed( self.robo.get_speed() - 0.1 )
                self.old_diff = self.robo.turn_diff
                self._timer_turn.cancel()
            else:
                self._timer_turn.cancel()
    
        self._timer_turn    = self.node.create_timer(timer, __turn__callback)

    def set_id_target(self, id:int):
        self._id = id

    def start_tracking(self):
        self.tracking = True

    def stop_tracking(self):
        self.tracking = False

    def is_tracking(self) -> bool:
        return self.tracking

    def fix_target(self, id:int) -> None:
        self.stop_tracking()
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
        hypotenuse = self.robo.get_distance_to_wall(direction)
        left  = self.robo.get_distance_to_wall(-1)
        right = self.robo.get_distance_to_wall(+1)
        direction = -1 if left < right else +1
        _, side_a = self.robo.sensor_lidar.find_closest_wall_angle(direction)
        side_b, angle = calc_side_and_angle(hypotenuse, side_a)
        return side_b, angle
    

