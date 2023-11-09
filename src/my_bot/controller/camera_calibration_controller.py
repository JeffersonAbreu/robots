from utils import SensorCamera
from rclpy.node import Node
# acesse ->     zzzcode.ai
MIN_DISTANCE = 0.5

class CameraCalibrationController:
    def __init__(self, node: Node):
        self.node = node
        self.sensor_camera = SensorCamera(self.node, self.handle_aruco_detected)
        self.sensor_camera.fix_target(1)

    # Camera
    def handle_aruco_detected(self, error_x, error_y, distance):
        """
        ids detectados
        """
        if self.robot.sensor_camera.lock is None:
            if abs(error_y) < 1:
                self.robot.sensor_camera.lock_target()
                self.commands_queue.clear()
                self.robot.stop()
                self.robot.move_forward()
        else:
            print(f'erro[{error_x:.2f}, {error_y}  distancia: {distance:.2f}')