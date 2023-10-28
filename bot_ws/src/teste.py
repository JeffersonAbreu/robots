import math
# Constantes de Velocidade
MIN_SPEED = 0.001
TOP_SPEED = 1.0
SPEED = 0.0
# Funções Auxiliares
def get_angular_speed(error: float) -> float:
    """
    Calcula a velocidade angular com base no erro.
    """
    Kp = 1.0  # Este é o coeficiente proporcional. Ajuste conforme necessário.
    angular_speed = Kp * error

    # Limitando a velocidade angular
    if abs(angular_speed) < MIN_SPEED:
        angular_speed = MIN_SPEED if angular_speed > 0 else -MIN_SPEED
    elif abs(angular_speed) > TOP_SPEED:
        angular_speed = TOP_SPEED if angular_speed > 0 else -TOP_SPEED

    # Desaceleração conforme se aproxima do destino
    DECELERATION_THRESHOLD = 0.1  # Ajuste conforme necessário. Representa a diferença de ângulo em radianos.
    if abs(error) < DECELERATION_THRESHOLD:
        angular_speed *= (abs(error) / DECELERATION_THRESHOLD)

    return angular_speed

def radians_to_degrees(radians) -> float:
    return radians * (180 / math.pi)

def degrees_to_radians(degrees) -> float:
    return degrees * (math.pi / 180)

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

def get_orientation() -> float:
    """Calcule e retorne a orientação do sensor IMU. (angle z) em graus."""
    return orientation

def __turn(speed):
        """
        Define a velocidade de rotação.
        """
        print(speed)

def turn_by_angle(angle:int):
    """
    Gira o robô por um ângulo específico em graus.
    """
    initial_orientation = get_orientation()
    destiny_orientation = normalize_angle2(initial_orientation + angle)
    __turn_timer_callback(initial_orientation, destiny_orientation)


def __turn_timer_callback(current_orientation, destiny_orientation):
    ori = get_orientation()
    while True:
        current_orientation = ori
        # Calcula a diferença entre a orientação atual e a desejada
        difference = normalize_angle2(destiny_orientation - current_orientation)
        print(f'Current: {(current_orientation)} | Distance: {abs(difference)} | Destiny: {(destiny_orientation)}')
        
        # Se a diferença for pequena o suficiente, pare o robô
        if abs(difference) <= 0.001:
            print('stop()')
            break
        else:
            # Caso contrário, continue girando na direção mais curta para alcançar a orientação desejada
            angular_speed = get_angular_speed(difference)
            __turn(angular_speed)
            ori += angular_speed
'''
print(normalize_angle_degrees(normalize_angle_degrees(-90) + (0)))
print('')
print(normalize_angle2(normalize_angle2(-90) + (0)))
print('')
print(normalize_angle(normalize_angle(-10) + (0)))
'''
orientation = 100
turn_by_angle(-35)
