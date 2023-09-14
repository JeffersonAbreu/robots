import rclpy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    rclpy.init()
    node = rclpy.create_node('keyboard_control_node')
    cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
    
    linear_speed = 0.2  # Velocidade linear (m/s)
    angular_speed = 0.2  # Velocidade angular (rad/s)
    
    try:
        while rclpy.ok():
            key = getKey(0.1)
            
            twist = Twist()
            
            if key == 'w':
                twist.linear.x = linear_speed
            elif key == 's':
                twist.linear.x = -linear_speed
            elif key == 'a':
                twist.angular.z = angular_speed
            elif key == 'd':
                twist.angular.z = -angular_speed
            
            cmd_vel_pub.publish(twist)
            
            if key == '\x03':
                break
    
    except Exception as e:
        print(e)
    
    finally:
        twist = Twist()
        cmd_vel_pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    main()

