import rclpy
from rclpy.node import Node
from controller import CameraCalibrationController

class Calibration(Node):
    def __init__(self):
        super().__init__('calibration')
        my_controller = CameraCalibrationController(self)

def main(args=None):
    rclpy.init(args=args)

    # Inicializando o nó do controlador
    controller = Calibration()
    try:
        # Mantendo o nó em execução
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("\n<Ctrl> + C     Cancelado o processo!!!\n")

    # Limpeza após a execução
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
