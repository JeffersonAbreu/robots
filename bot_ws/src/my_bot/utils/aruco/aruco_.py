from enum import Enum, auto
import os
import math
from tag import Tag
from constants import WALL_LARG

class Orientation(Enum):
    NORTH = auto()
    SOUTH = auto()
    EAST = auto()
    WEST = auto()

class Aruco_:
    """
    Representação do ARUCO

    Parâmetros:\n
    key : ID da tag aruco\n
    orientacion : Sentido em que a tag está apontada\n
    relative_to : É a parede onde a tag será fixada\n
    xy : deslocamento da tag em relação a parede
    """
    def __init__(self, key: int, orientation: Orientation, relative_to: str, xy: float = 0.0):
        self.key = key 
        self.orientation = orientation
        self.relative_to = relative_to
        self.xy = xy
    
    def __lt__(self, other):
        """Defina aqui como você quer comparar os objetos Aruco_"""
        return self.key < other.key

    def get_pose(self):
        _x = 0
        _y = 0
        _z = 0.25
        _roll = 0
        _pitch = 0
        _yaw = 0

        if self.orientation == Orientation.SOUTH:
            _x = -WALL_LARG
            _y = self.xy
            _yaw = math.pi
        elif self.orientation == Orientation.NORTH:
            _x = WALL_LARG
            _y = self.xy
        elif self.orientation == Orientation.WEST:
            _x = self.xy
            _y = WALL_LARG
            _yaw = math.pi / 2
        elif self.orientation == Orientation.EAST:
            _x = self.xy
            _y = -WALL_LARG
            _yaw = -math.pi / 2


        return [_x, _y, _z, _roll, _pitch, _yaw]
        
    def create_tag(self) -> Tag:
      name = f"tag_{self.key:02}"
      pose = self.get_pose()
      uri = "model://" + os.path.join('ar_tags', name)
      return Tag(name, pose, self.relative_to, uri)