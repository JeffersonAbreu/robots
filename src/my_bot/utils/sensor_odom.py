from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from math import sqrt
class SensorOdom:

    def __init__(self, node: Node):
        self.node = node
        # Subscribers
        self.odom_sub = self.node.create_subscription(Odometry, '/odom', self.__odom_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # Variables
        self.last_x = 0
        self.last_y = 0
        self.current_odom_x = 0.0
        self.current_odom_y = 0.0
        self.traveled_distance = 0.0


    def __odom_callback(self, msg):
        # Save the current odometry of position and orientation
        self.current_odom_x = msg.pose.pose.position.x
        self.current_odom_y = msg.pose.pose.position.y

        # Calculate current distance traveled: (Calculated as straight lines)
        dx = (self.current_odom_x-self.last_x)
        dy = (self.current_odom_y-self.last_y)
        distance = sqrt((dx) ** 2 + (dy) ** 2)
        '''
        hipotenusa (distância euclidiana)
        teorema de Pitágoras para calcular a distância entre dois pontos.
        '''
        # Update last_x and last_y to current values to avoid error accumulation
        self.last_x = self.current_odom_x
        self.last_y = self.current_odom_y

        self.traveled_distance += abs(distance)

    def reset_odom(self) -> None:
        self.last_x = self.current_odom_x
        self.last_y = self.current_odom_y
        self.traveled_distance = 0.0

    
    def get_travelled_distance(self) -> float:
        return self.traveled_distance