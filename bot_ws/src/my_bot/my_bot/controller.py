import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.01
        self.detectou = False
        self.timer = self.create_timer(timer_period, self.move_robot)

    def move_robot(self):
        if self.detectou == False:
            msg = Twist()
            msg.linear.x = 0.2  # Move forward
            #msg.angular.z = 0.2
            self.cmd_pub.publish(msg)

    def lidar_callback(self, msg):
        # Considerando que o LiDAR 1D tem apenas uma amostra
        distance_to_wall = msg.ranges[0]

        if distance_to_wall < 0.3:  # Se a distância for menor que 30cm
            self.detectou = True
            self.stop_and_turn()
            self.get_logger().info("Colisão.")

    def stop_and_turn(self):
        # Parar o robô
        twist = Twist()
        self.cmd_pub.publish(twist)
        rclpy.sleep(1)

        # Virar 90 graus para a direita
        twist.angular.z = -1.57  # Aproximadamente 90 graus em radianos
        self.cmd_pub.publish(twist)
        rclpy.sleep(1)

        # Seguir em frente
        twist.linear.x = 0.3
        twist.angular.z = 0
        self.cmd_pub.publish(twist)
        self.detectou = False

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        print("\n<Ctrl> + C     Cancelado o processo!!!\n")
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
