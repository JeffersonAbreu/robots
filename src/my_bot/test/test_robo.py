from bib import *
from bib import get_current_line_number as gcln
from constants import *
from orientation import Orientation

def turn_by_orientation(orientation):
        if orientation == Orientation.NORTH:
            orientation = 0
        elif orientation == Orientation.EAST:
            orientation = 90
        elif orientation == Orientation.SOUTH:
            orientation = 180
        elif orientation == Orientation.WEST:
            orientation = 270

        orientation_robo = round(normalize_angle_degrees(45))
        angle = normalize_angle2(orientation - orientation_robo)

        print(f'Orientation    Robo: {orientation_robo:>4}')
        print(f'Orientation Destiny: {orientation:>4}')
        print(f'turn angle: {angle:>4}')

#turn_by_orientation(Orientation.SOUTH)
FACTOR_ACCELERATION = 0.01
FACTOR_BRAKING = -0.03
TOP_SPEED = 1.0
MIN_SPEED = 0.1
ZERO = 0.0

class Robot:
    def __init__(self):
        self.__speed = MIN_SPEED*2
        self.speed_ = 0
        self.speed_timer = False

    def get_speed(self):
        return self.__speed

    def move_forward(self, speed):
        print(f"self_move_forward({speed})")
        self.__speed = speed

    def speed_timer_cancel(self):
        print("self.speed_timer.cancel()")
        self.speed_timer = True

    def __speed_timer_callback(self):
        self.speed_timer = False
        while not self.speed_timer:
            factor = FACTOR_ACCELERATION if self.speed_ >= self.get_speed() else FACTOR_BRAKING
            print(f"   FACTOR: {factor}")
            speed = round(self.get_speed() + factor, 3)
            print(f"new speed: {speed}")
            speed = max(MIN_SPEED, min(speed, TOP_SPEED))
            print(f"   Ajuste: {speed} : ajustado com min e max" )
            self.move_forward(speed)
            if speed == self.speed_ or speed == TOP_SPEED or speed == MIN_SPEED:
                if speed == MIN_SPEED and self.STOP:
                    self.move_forward(ZERO)
                self.speed_timer_cancel()
        
    def set_speed(self, speed):
        if speed < MIN_SPEED:
            speed = ZERO
        if self.get_speed() < MIN_SPEED and speed >= MIN_SPEED:
            self.move_forward(MIN_SPEED)
        if self.get_speed() == speed:
            return        
        self.STOP = False
        if speed == ZERO:
            self.STOP = True
        self.speed_ = round(max(MIN_SPEED, min(speed, TOP_SPEED)), 3)
        print(f"SET_SPEED: {self.speed_}")
        self.__speed_timer_callback()  # Verifica a cada 0.001 segundos


# Exemplo de uso
robot = Robot()
robot.set_speed(TOP_SPEED/2)  # Define a velocidade inicial
