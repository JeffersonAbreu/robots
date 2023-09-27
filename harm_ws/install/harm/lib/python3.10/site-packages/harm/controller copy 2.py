import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import signal
import sys


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state = "FORWARD"
        self.start_time = time.time()

    def move_robot(self):
        msg = Twist()

        if self.state == "FORWARD":
            msg.linear.x = 0.5
            if time.time() - self.start_time > 30:
                self.state = "TURN"
                self.start_time = time.time()

        elif self.state == "TURN":
            msg.angular.z = 0.5
            if time.time() - self.start_time > 30:
                self.state = "FORWARD"
                self.start_time = time.time()

        self.publisher_.publish(msg)

    def run(self):
        while rclpy.ok():
            self.move_robot()
            rclpy.spin_once(self, timeout_sec=0.01)

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    robot_controller.run()
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
