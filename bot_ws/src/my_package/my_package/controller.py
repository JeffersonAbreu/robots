import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Inicializando o publicador para o tópico '/cmd_vel'
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Definindo o período do timer para chamar a função de movimento
        timer_period = 0.1  # segundos
        self.timer = self.create_timer(timer_period, self.move_robot)
        
        # Definindo o estado inicial para mover para a frente
        self.state = "FORWARD"
        
        # Capturando o tempo atual para controlar a duração dos movimentos
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]

    def move_robot(self):
        """
        Função para controlar o movimento do robô.
        O robô se move para a frente por 3 segundos e depois gira por 3 segundos.
        """
        msg = Twist()

        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        elapsed_time = current_time - self.start_time

        # Movendo para a frente
        if self.state == "FORWARD":
            msg.linear.x = 0.5
            if elapsed_time > 3:
                self.state = "TURN"
                self.start_time = current_time
        
        # Girando para a esquerda
        elif self.state == "TURN":
            msg.angular.z = 0.5
            if elapsed_time > 3:
                self.state = "FORWARD"
                self.start_time = current_time

        # Publicando a mensagem de movimento
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    # Inicializando o nó do controlador
    robot_controller = RobotController()

    # Mantendo o nó em execução
    rclpy.spin(robot_controller)

    # Limpeza após a execução
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
