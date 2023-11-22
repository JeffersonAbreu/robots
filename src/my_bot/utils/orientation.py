from enum import Enum, auto
class Orientation(Enum):
    NORTH = auto()
    SOUTH = auto()
    EAST = auto()
    WEST = auto()
    NE = auto() # Nordeste: Northeast (NE)
    SE = auto() # Sudeste: Southeast (SE)
    NW = auto() # Noroeste: Northwest (NW)
    SW = auto() # Sudoeste: Southwest (SW)

    def format_degrees(orientation):
        if orientation == Orientation.NORTH:
            degrees = 0
        elif orientation == Orientation.NE:
            degrees = 45
        elif orientation == Orientation.EAST:
            degrees = 90
        elif orientation == Orientation.SE:
            degrees = 135
        elif orientation == Orientation.SOUTH:
            degrees = 180
        elif orientation == Orientation.SW:
            degrees = 225
        elif orientation == Orientation.WEST:
            degrees = 270
        elif orientation == Orientation.NW:
            degrees = 315
        return degrees