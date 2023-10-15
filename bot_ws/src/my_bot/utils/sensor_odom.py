from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from utils.bib import transfor_quat_for_euler
from rclpy.node import Node

class SensorOdom:

    def __init__(self, node: Node):
        self.node = node
        # Subscribers
        self.odom_sub = self.node.create_subscription(Odometry, '/odom', self.__odom_callback, 10)
        # Variables
        self.orienation = Quaternion()
        self.position = None

    def __odom_callback(self, msg):
        self.position = msg.pose.pose.position
        self.orienation = msg.pose.pose.orientation

    def get_orientation(self) -> float:
        _, _, yaw__odom = transfor_quat_for_euler(self.orienation)
        return yaw__odom * (-1)
    
    def get_position(self) -> float:
        return self.position.x