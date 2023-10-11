import cv2
import numpy as np

class QRCodeDetector:
    '''
    instalar qrcode e opencv-python
    '''
    def __init__(self):
        self.detector = cv2.QRCodeDetector()

    def detect_qrcode(self, image):
        # Detecta QR Code na imagem
        retval, decoded_info, _ = self.detector.detectAndDecode(image)
        return decoded_info
