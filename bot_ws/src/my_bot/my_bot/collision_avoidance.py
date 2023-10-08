import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import time
from utils.robo import Robo
from utils.state import State

class CollisionAvoidance(Node):

    def __init__(self):
        super().__init__('collision_avoidance')
        self.robo = Robo(self)
        self.lidar_sub = self.create_subscription(LaserScan, '/lidar/scan', self.lidar_callback, 10)
        self.get_logger().info("Collision avoidance node has been started.")
        self.move_forward()


    def lidar_callback(self, msg):
        if self.robo.get_state() == State.FORWARD:
            # Considerando que o LiDAR 1D tem apenas uma amostra
            distance_to_wall = msg.ranges[0]

            if distance_to_wall < 0.3:  # Se a distância for menor que 30cm
                self.stop_and_turn()


    def move_forward(self):
        self.robo.move_forward()
        
    def stop_and_turn(self):
        self.stop()

        # Virar 90 graus para a direita
        self.robo.turn_right(90)
        time.sleep(1)

        # Seguir em frente
        self.move_forward()

    def stop(self):
        # Parar o robô
        self.robo.stop()
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidance()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()