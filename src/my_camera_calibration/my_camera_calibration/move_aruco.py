# move_aruco.py
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose

class MoveAruco(Node):
    def __init__(self):
        super().__init__('move_aruco')
        self.state_service = self.create_client(SetEntityState, '/set_entity_state')
        while not self.state_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_entity_state service...')
        self.move_aruco()

    def move_aruco(self):
        # ... Add your logic here to move the ArUco marker ...
        pass

def main(args=None):
    rclpy.init(args=args)
    node = MoveAruco()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
