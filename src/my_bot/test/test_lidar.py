from sensor_msgs.msg import LaserScan
from rclpy.node import Node

class SensorLidar:

    def __init__(self):
        """
        Inicializa o Lidar.
        """
        self.lidar_data__front = 0.0
        self.lidar_data__left  = []
        self.lidar_data__right = []
        
        '''Lateral espaça minimo para um caminho'''


    def collision_router(self, direction=0) -> bool:
        # Definindo os limiares de distância e a matriz de colisão
        thresholds = [0, 2.5, 5, 7.5, 10, 15]
        matrix_collision = [
            [0.902103, 0.404213, 0.265292, 0.209872, 0.184844, 0.174666],
            [0.481064, 0.275839, 0.198622, 0.165227, 0.150698, 0.147149],
            [0.435427, 0.259953, 0.190666, 0.160348, 0.147420, 0.145077],
            [0.368469, 0.235040, 0.178146, 0.152872, 0.142697, 0.142582],
            [0.330757, 0.219343, 0.169745, 0.147628, 0.139241, 0.140622],
            [0.266498, 0.190136, 0.153578, 0.137499, 0.132752, 0.137443]
        ]

        # Seleciona o índice apropriado com base na direção
        index = next((i for i, t in enumerate(thresholds) if t >= abs(direction)), len(thresholds) - 1)
        collision_thresholds = matrix_collision[index]

        direction = 0 if abs(direction) < 2.5 else (-1 if direction < 0 else 1 )
        # Verifica se há risco de colisão
        return self.check_collision(direction, collision_thresholds)

    def check_collision(self, direction, collision_thresholds):
        if direction == 0:
            if self.lidar_data__front < collision_thresholds[0]:
               return True
            for i in range(6):
               distance  = self.lidar_data__right[i]
               threshold = collision_thresholds[i]
               if distance < threshold:
                    return True
            for i in range(6):
               distance  = self.lidar_data__left[i]
               threshold = collision_thresholds[i]
               if distance < threshold:
                    return True
        elif direction > 0:
            for i in range(6):
               distance  = self.lidar_data__right[i]
               threshold = collision_thresholds[i]
               if distance < threshold:
                    return True
        elif direction < 0:
            for i in range(6):
               distance  = self.lidar_data__left[i]
               threshold = collision_thresholds[i]
               if distance < threshold:
                    return True
        return False

    
        
    def get_data_range(self, grau:int = 0):
        """
        Retorna a distância ao obstáculo mais próximo diretamente à frente do robô.
        """
        if abs(grau) == 0:
            return self.lidar_data__front

        index = abs(grau) - 1
        index = max(0, min(index, 5))

        if grau > 0:
            return self.lidar_data__right[index]
        return self.lidar_data__left[index]
          

    def find_closest_wall_angle(self, direction: int = 0):
        """
        Retorna a menor distância e o ângulo relativo ao LiDAR. 
        - Se direction for 0, verifica a frente.
        - Se direction for positivo, verifica a direita.
        - Se direction for negativo, verifica a esquerda.
        - O ângulo varia de -6 a 6, correspondendo a uma variação de 15° por índice.
        """
        min_distance = float('inf')
        angle_of_min_distance = 0

        # Verifica a direção específica ou todas as direções
        if direction >= 0:
            for angle, distance in enumerate(self.lidar_data__right):
                if distance < min_distance:
                    min_distance = distance
                    angle_of_min_distance = angle + 1

        if direction <= 0:
            for angle, distance in enumerate(self.lidar_data__left):
                if distance < min_distance:
                    min_distance = distance
                    angle_of_min_distance = -(angle + 1)

        # Verifica a frente apenas se direction for 0
        if direction == 0 and self.lidar_data__front < min_distance:
            min_distance = self.lidar_data__front

        return angle_of_min_distance, min_distance
    
    def print_range(self, direction=0):
        if direction > 0:
            for angle, distance in enumerate(self.lidar_data__right):
                my_print(angle + 1, distance)
        if direction == 0:
            for i in range(len(self.lidar_data__right)-1, -1, -1):
                my_print(i + 1, self.lidar_data__right[i])
            my_print(0, self.lidar_data__front)
        if direction <= 0:
            for angle, distance in enumerate(self.lidar_data__left):
                my_print(-(angle + 1), distance)
                

def my_print(i, value):
    print(f'[{i:>2}] = {value:>8.6f}')


# Vamos simular um teste para a função check_collision da classe SensorLidar

# Criar uma instância da classe de teste
sensor_test = SensorLidar()

# Simular diferentes cenários de teste
test_results = []

# Cenário 1: Obstáculos próximos em todas as direções
sensor_test.lidar_data__front = 1
sensor_test.lidar_data__left = [0.902103, 0.404213, 0.265292, 0.209872, 0.184844, 0.174666]
sensor_test.lidar_data__right = [0.902103, 0.404213, 0.265292, 0.209872, 0.184844, 0.174666]
test_results.append(sensor_test.collision_router())

# Cenário 2: Obstáculos apenas à frente
sensor_test.lidar_data__front = 0.6
sensor_test.lidar_data__left = [0.481064, 0.275839, 0.198622, 0.165227, 0.150698, 0.147149]
sensor_test.lidar_data__right = [0.481064, 0.275839, 0.198622, 0.165227, 0.150698, 0.147149]
test_results.append(sensor_test.collision_router())
test_results.append(sensor_test.collision_router(-6))

# Cenário 3: Sem obstáculos próximos
sensor_test.lidar_data__front = 0.50
sensor_test.lidar_data__left = [0.435427, 0.259953, 0.190666, 0.160348, 0.147420, 0.145077]
sensor_test.lidar_data__right = [0.435427, 0.259953, 0.190666, 0.160348, 0.147420, 0.145077]
test_results.append(sensor_test.collision_router())
test_results.append(sensor_test.collision_router(3))