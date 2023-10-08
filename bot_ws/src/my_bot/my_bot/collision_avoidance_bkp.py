import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time
from nav_msgs.msg import Odometry

class CollisionAvoidance(Node):

    def __init__(self):
        super().__init__('collision_avoidance')
        self.lidar_sub = self.create_subscription(LaserScan, '/lidar/scan', self.lidar_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.get_logger().info("Collision avoidance node has been started.")
        
        # Estados:
        self.FORWARD = 'FORWARD' # 'AVANÇAR'
        self.STOP = 'STOP'       # 'PARAR'
        self.TURN = 'TURN'       # 'GIRAR'
        self.state = self.FORWARD
        self.move_forward()

    def lidar_callback(self, msg):
        if self.state == self.FORWARD:
            # Considerando que o LiDAR 1D tem apenas uma amostra
            distance_to_wall = msg.ranges[0]

            if distance_to_wall < 0.3:  # Se a distância for menor que 30cm
                self.stop_and_turn()

    def odom_callback(self, msg):
        # A posição do robô está no campo pose.pose.position
        position = msg.pose.pose.position
        #self.get_logger().info('Robot position: x: %f, y: %f, z: %f' % (position.x, position.y, position.z))

    def move_forward(self):
        twist = Twist()
        twist.linear.x = 0.3
        self.cmd_pub.publish(twist)
        
    def stop_and_turn(self):
        self.state = self.STOP

        # Parar o robô
        twist = Twist()
        self.cmd_pub.publish(twist)
        time.sleep(1)

        # Virar 90 graus para a direita
        self.state = self.TURN
        twist.angular.z = -1.57
        self.cmd_pub.publish(twist)
        time.sleep(1)

        # Seguir em frente
        self.state = self.FORWARD
        self.move_forward()

def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidance()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()