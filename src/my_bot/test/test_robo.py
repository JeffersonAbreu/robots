from bib import *
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

turn_by_orientation(Orientation.SOUTH)