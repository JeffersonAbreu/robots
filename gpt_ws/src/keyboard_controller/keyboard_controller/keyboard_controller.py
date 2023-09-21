import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from keyboard import Keyboard # pip install keyboard

class KeyboardController(Node):

    def __init__(self):
        super().__init__('keyboard_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.keyboard = Keyboard()

    def run(self):
        while rclpy.ok():
            key = self.keyboard.get_key()
            twist_msg = Twist()

            # Mapeie as teclas do teclado para comandos de velocidade linear e angular
            if key == 'up':
                twist_msg.linear.x = 0.5  # Velocidade linear para a frente
            elif key == 'down':
                twist_msg.linear.x = -0.5  # Velocidade linear para trás
            elif key == 'left':
                twist_msg.angular.z = 1.0  # Velocidade angular para a esquerda
            elif key == 'right':
                twist_msg.angular.z = -1.0  # Velocidade angular para a direita

            # Publicar o comando de controle
            self.publisher.publish(twist_msg)
            self.get_logger().info(f'Command sent: linear={twist_msg.linear.x}, angular={twist_msg.angular.z}')

            rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardController()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
