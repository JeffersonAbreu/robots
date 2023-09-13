import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_callback(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")  # Converter para formato OpenCV (BGR)

    # Aqui, você pode usar a biblioteca OpenCV para processar a imagem e ler QR codes.

def main():
    rclpy.init()
    node = rclpy.create_node('qr_code_reader')
    image_sub = node.create_subscription(Image, '/camera/image', image_callback, 10)
    # Adicione qualquer outra configuração necessária para ler QR codes aqui

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
