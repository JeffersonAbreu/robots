from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

from utils.bib import transfor_quat_for_euler

class RobotEstimator:

    def __init__(self, node):
        self.node = node
        # Subscribers
        self.imu_sub = self.node.create_subscription(Imu, '/imu', self.__imu_callback, 10)
        self.odom_sub = self.node.create_subscription(Odometry, '/odom', self.__odom_callback, 10)
        # Variables
        self.odom_position = 0.0
        self.orientation_odom = Quaternion()
        self.orientation__imu = Quaternion()

    def __imu_callback(self, msg):
        self.orientation__imu = msg.orientation

    def __odom_callback(self, msg):
        self.orientation_odom = msg.pose.pose.orientation
        self.odom_position = msg.twist.twist.linear.x

    def get_position(self) -> float:
        return self.odom_position
    
    def get_orientation(self) -> float:
        """Calcule e retorne a orientação combinada de IMU e odometria."""
        _, _, yaw__imu = transfor_quat_for_euler(self.orientation__imu)
        _, _, yaw_odom = transfor_quat_for_euler(self.orientation_odom)
        return yaw__imu * (-1)