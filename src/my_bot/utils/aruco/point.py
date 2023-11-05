from tag import Tag
from aruco_ import Orientation
from constants import DISTANCE_TO_WALL

class Point:
    """
    Representação do Ponto de Interesse

    Parâmetros:\n
    key : ID\n
    orientacion : Sentido em que o ponto será criado\n
    x e y : localização
    """
    def __init__(self, key: int, orientation: Orientation, x: float = 0.0, y: float = 0.0):
        self.key = key 
        self.orientation = orientation
        self.x = x
        self.y = y

    def get_pose(self):
        _x = self.x
        _y = self.y
        _z = 0
        _roll = 0
        _pitch = 0
        _yaw = 0

        if   self.orientation == Orientation.SOUTH:
            _x -= DISTANCE_TO_WALL
        elif self.orientation == Orientation.NORTH:
            _x += DISTANCE_TO_WALL
        elif self.orientation == Orientation.WEST:
            _y += DISTANCE_TO_WALL
        elif self.orientation == Orientation.EAST:
            _y -= DISTANCE_TO_WALL

        return [_x, _y, _z, _roll, _pitch, _yaw]
        
    def create_tag(self) -> Tag:
      name = f"point_{self.key:02}"
      pose = self.get_pose()
      uri = "model://point"
      return Tag(name, pose, None, uri)