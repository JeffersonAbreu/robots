#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MyCollision(Node):

    def __init__(self):
        super().__init__('my_collision')
        self.get_logger().info('My Collision node is up and running.')

    def detect_collision(self):
        # Coloque sua lógica de detecção de colisão aqui
        self.get_logger().info('Detecting collisions.')

def main(args=None):
    rclpy.init(args=args)
    node = MyCollision()
    try:
        node.detect_collision()  # Execute a lógica de detecção de colisão
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
