import rclpy
from geometry_msgs.msg import Twist

def main():
    rclpy.init()
    node = rclpy.create_node('car_controller')

    cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)

    while rclpy.ok():
        twist = Twist()
        # Defina os valores de velocidade linear e angular desejados aqui
        cmd_vel_pub.publish(twist)
        node.spin_once()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

