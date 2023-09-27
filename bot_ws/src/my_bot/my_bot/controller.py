import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.move_robot)

    def move_robot(self):
        msg = Twist()
        msg.linear.x = 0.5  # Move forward
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        print("CTRL + C     Cancelado o processo")
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
