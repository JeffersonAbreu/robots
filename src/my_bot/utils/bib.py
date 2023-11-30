import math
import inspect
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Quaternion
from utils.constants import MIN_SPEED, TOP_SPEED

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

def is_ON(thread):
    return False if thread is None or thread.is_canceled() else True

def is_OFF(thread):
    return not ( is_ON(thread) )


def ajuste_speed(distance, angle_rotation, speed_z):
    # Defina os limiares de distância e ângulo
    DISTANCE_THRESHOLD = 3.0  # Distância a partir da qual a velocidade começa a diminuir
    ANGLE_THRESHOLD = 5  # Ângulo a partir do qual a velocidade de rotação começa a diminuir

    # Ajusta a velocidade com base na distância
    if distance > DISTANCE_THRESHOLD:
        speed = TOP_SPEED
    else:
        # Reduz a velocidade à medida que se aproxima do ArUco
        speed = MIN_SPEED + (distance / DISTANCE_THRESHOLD) * (TOP_SPEED - MIN_SPEED)

    # Ajusta a velocidade com base no ângulo de rotação
    if abs(angle_rotation) > ANGLE_THRESHOLD:
        # Se o ângulo de rotação for muito grande, reduz ainda mais a velocidade
        speed *= 0.5

    # Ajusta a velocidade com base na velocidade atual (para evitar paradas bruscas)
    if speed_z > 0 and angle_rotation != 0:
        # Se estiver girando, ajuste a velocidade de parada com base na velocidade e ângulo
        stopping_time = abs(angle_rotation) / speed_z
        if stopping_time < 1:  # Se precisar parar em menos de 1 segundo, reduza a velocidade
            speed *= stopping_time

    return max(MIN_SPEED, min(speed, TOP_SPEED))  # Garante que a velocidade esteja dentro dos limites
        

def get_current_line_number(ajust_line:int=0):
    '''
    Função para obter o número da linha atual de onde ela é chamada
    :ajust_line , pode ser usado se quiser ajustar linhas para ou para cima.
    exemplo: estou na linha 100 e quero que mostre a linha 99, passo ajust_line = -1
    '''
    return f"Line: {Color.yellow(inspect.currentframe().f_back.f_lineno + ajust_line)}"

def on_or_off(condition:bool):
    return Color.green('YES') if condition else Color.red(' NO')

class Color:
    def black(text):
        return f"\033[90m{text}\033[0m"

    def red(text):
        return f"\033[91m{text}\033[0m"

    def green(text):
        return f"\033[92m{text}\033[0m"

    def yellow(text):
        return f"\033[93m{text}\033[0m"

    def blue(text):
        return f"\033[94m{text}\033[0m"

    def magenta(text):
        return f"\033[95m{text}\033[0m"

    def cyan(text):
        return f"\033[96m{text}\033[0m"

    def white(text):
        return f"\033[97m{text}\033[0m"