import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class LaneFollowing(Node):

    def __init__(self):
        super().__init__('lanefollowing')
        
        # Inicializa a ponte entre ROS e OpenCV
        self.bridge = CvBridge()
        
        # Cria um publicador para o tópico cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Cria uma assinatura para o tópico camera_sensor/image_raw
        self.img_sub = self.create_subscription(Image, '/camera_sensor/image_raw', self.subs_callback, 10)
        
        # Cria um temporizador para chamar o método update_callback a cada 10ms
        self.timer = self.create_timer(0.01, self.update_callback)
        
        # Inicializa as matrizes de imagem
        self.frame = None
        self.gray = None
        self.dst = None
        
        # Inicializa pontos e variáveis relacionadas
        self.prevpt1 = cv2.Point(110, 60)
        self.prevpt2 = cv2.Point(620, 60)
        self.cpt = [cv2.Point(0, 0), cv2.Point(0, 0)]
        self.fpt = cv2.Point(0, 0)
        
        # Inicializa outras variáveis
        self.minlb = [0, 0]
        self.ptdistance = [0.0, 0.0]
        self.threshdistance = [0.0, 0.0]
        self.mindistance1 = []
        self.mindistance2 = []
        self.error = 0

    def subs_callback(self, msg):
        try:
            # Converte a mensagem ROS Image para uma imagem OpenCV BGR
            self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Converte a mensagem ROS Image para uma imagem OpenCV em escala de cinza
            self.gray = self.bridge.imgmsg_to_cv2(msg, "mono8")
            
            # Etapas de processamento de imagem
            gray = gray + 100 - np.mean(gray)
            gray = cv2.threshold(gray, 160, 255, cv2.THRESH_BINARY)
            dst = gray[int(gray.shape[0] * 2 / 3):, :]
        
            cnt, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)
            
            mindistance1 = []
            mindistance2 = []
            
            if cnt > 1:
                for i in range(1, cnt):
                    p = centroids[i]
                    ptdistance = [abs(p[0] - self.prevpt1.x), abs(p[0] - self.prevpt2.x)]
                    mindistance1.append(ptdistance[0])
                    mindistance2.append(ptdistance[1])

                threshdistance = [min(mindistance1), min(mindistance2)]
                minlb = [mindistance1.index(min(mindistance1)), mindistance2.index(min(mindistance2))]

                cpt = [cv2.Point2d(centroids[minlb[0] + 1][0], centroids[minlb[0] + 1][1]),
                      cv2.Point2d(centroids[minlb[1] + 1][0], centroids[minlb[1] + 1][1])]

                if threshdistance[0] > 100:
                    cpt[0] = self.prevpt1
                if threshdistance[1] > 100:
                    cpt[1] = self.prevpt2
            else:
                cpt = [self.prevpt1, self.prevpt2]

            self.prevpt1 = cpt[0]
            self.prevpt2 = cpt[1]

            fpt = cv2.Point2d((cpt[0].x + cpt[1].x) / 2, (cpt[0].y + cpt[1].y) / 2 + gray.shape[0] * 2 / 3)
            dst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)

            cv2.circle(self.frame, (int(fpt.x), int(fpt.y)), 2, (0, 0, 255), 2)
            cv2.circle(dst, (int(cpt[0].x), int(cpt[0].y)), 2, (0, 0, 255), 2)
            cv2.circle(dst, (int(cpt[1].x), int(cpt[1].y)), 2, (255, 0, 0), 2)

            self.error = dst.shape[1] / 2 - fpt.x

            cv2.imshow("camera", self.frame)
            cv2.imshow("gray", dst)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)
            
    def update_callback(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.5
        cmd_vel.angular.z = (self.error * 90.0 / 400) / 15
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    lane_following_node = LaneFollowing()
    rclpy.spin(lane_following_node)
    lane_following_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
