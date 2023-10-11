import rclpy
from rclpy.node import Node
from utils.robot4 import Robot

class CollisionAvoidance(Node):

    def __init__(self):
        super().__init__('collision_avoidance')
        self.robo = Robot(self)
        self.get_logger().info("Collision avoidance node has been started.")

        #self.robo.move_forward()
        self.robo.turn_by_angle(180)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidance()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()