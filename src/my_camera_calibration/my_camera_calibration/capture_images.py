# capture_images.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class CaptureImages(Node):
    def __init__(self):
        super().__init__('capture_images')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera', self.image_callback, 10)
        self.count = 0
        os.makedirs('images', exist_ok=True)
        self.cv_image = None

    def image_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def capture_image(self):
        if self.cv_image is not None:
            cv2.imwrite(f'images/aruco_{self.count}.png', self.cv_image)
            self.count += 1
            print(f'Captured image {self.count}')

def main(args=None):
    rclpy.init(args=args)
    node = CaptureImages()
    
    # Thread for spinning in the background
    def spin():
        rclpy.spin(node)
    import threading
    spin_thread = threading.Thread(target=spin)
    spin_thread.start()
    
    while True:
        # Display the image to capture
        if node.cv_image is not None:
            cv2.imshow('Camera', node.cv_image)
            key = cv2.waitKey(1) & 0xFF
        
            # Press 'c' to capture the image
            if key == ord('c'):
                node.capture_image()
            
            # Press 'q' to quit
            if key == ord('q'):
                break
    
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    main()
