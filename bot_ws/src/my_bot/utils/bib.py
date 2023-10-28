import math
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Quaternion

def radians_to_degrees(radians) -> float:
    return radians * (180 / math.pi)

def degrees_to_radians(degrees) -> float:
    return degrees * (math.pi / 180)

def transfor_quat_for_euler(orientation: Quaternion):
    if not all(hasattr(orientation, attr) for attr in ['x', 'y', 'z', 'w']):
        raise TypeError("Expected an object with x, y, z, and w attributes!")
    else:
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        rotation = R.from_quat([x, y, z, w])
        return rotation.as_euler('xyz', degrees=True) # degrees or radians
    
def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))

def normalize_angle_degrees(angle: int) -> int:
    """Normaliza o ângulo para o intervalo [0, 360) graus."""
    return angle % 360

def normalize_angle2(angle: int) -> int:
    """Normaliza um ângulo para o intervalo [-180º, 180º)."""
    while angle >= 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle