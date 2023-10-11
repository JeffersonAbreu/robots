import rclpy
from rclpy.node import Node
from model import Robot
from controller import RobotController

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        robot = Robot(self)
        robot_controller = RobotController(robot)

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("\n<Ctrl> + C     Cancelado o processo!!!\n")
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
