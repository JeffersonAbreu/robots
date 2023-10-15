from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from utils.bib import transfor_quat_for_euler
from rclpy.node import Node

class SensorIMU:

    def __init__(self, node: Node):
        self.node = node
        # Subscribers
        self.imu_sub = self.node.create_subscription(Imu, '/imu', self.__imu_callback, 10)
        # Variables
        self.orientation__imu = Quaternion()

    def __imu_callback(self, msg):
        self.orientation__imu = msg.orientation

    def get_orientation(self) -> float:
        """Calcule e retorne a orientação do sensor IMU. (angle z)"""
        _, _, yaw__imu = transfor_quat_for_euler(self.orientation__imu)
        return yaw__imu * (-1)