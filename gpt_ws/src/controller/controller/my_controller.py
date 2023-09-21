#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MyController(Node):

    def __init__(self):
        super().__init__('my_controller')
        self.get_logger().info('My Controller node is up and running.')

    def run_controller(self):
        # Coloque sua lógica de controle aqui
        self.get_logger().info('Running controller logic.')

def main(args=None):
    rclpy.init(args=args)
    node = MyController()
    try:
        node.run_controller()  # Execute a lógica de controle
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
