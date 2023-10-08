from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from utils.state import State
from utils.util import degrees_to_radians

class Robo:

    def __init__(self, node):
        self.node = node
        self.state = ''
        self.old_state = ''
        self.twist_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.node.create_subscription(Odometry,'/odom',self.__odom_callback,10)


    def move_forward(self):
        twist = Twist()
        twist.linear.x = 0.3
        self.twist_pub.publish(twist)


    def stop(self):
        twist = Twist()
        self.twist_pub.publish(twist)
    

    def turn_left(self, angle):
        self.__turn(angle * -1)
    

    def turn_right(self, angle):
        self.__turn(angle)
        

    def __turn(self, angle):
        twist = Twist()
        twist.angular.z = degrees_to_radians(angle)
        self.twist_pub.publish(twist)


    def __odom_callback(self, msg):
        # A posição do robô está no campo pose.pose.position
        position = msg.pose.pose.position
        self.node.get_logger().info('Robot position: x: %f, y: %f, z: %f' % (position.x, position.y, position.z))
        if position.x == 0:
            self.state = State.STOP
        elif position.x > 0:
            self.state = State.FORWARD
        else:
            self.state = State.TURN
        
        if self.old_state != self.state:
            self.old_state = self.state
            self.node.get_logger().info(f'Robot position: {self.state}')


    def get_state(self):
        return self.state            