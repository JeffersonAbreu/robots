import rclpy
import os
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import yaml
import time

class CameraCalibrador(Node):
    def __init__(self):
        super().__init__('calibrador_camera')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera', self.callback_imagem, 10)
        self.marker_length_m = 0.2  # Tamanho real do marcador em metros (20cm)
        self.distance_to_marker_m = 2.5  # Distância desejada do marcador em metros (2,5m)
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
        self.parametros = aruco.DetectorParameters_create()
        self.dados_calibracao = {}

        # Parâmetros de calibração fictícios iniciais
        self.camera_matrix = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]], dtype=float)
        self.dist_coeffs = np.zeros(5)  # Supondo que não há distorção
        self.contagem_regressiva = 20

        self.calibration_file_path = 'dados_calibracao.yaml'
        self.load_config_initial()

    def callback_imagem(self, msg):
        imagem_cv = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        #imagem_cv = cv2.resize(imagem_cv, (640, 480))

        if self.contagem_regressiva > 0:
            self.print_in_imagem(imagem_cv, f"Calibrando em {self.contagem_regressiva} segundos...")
            cv2.imshow('Calibracao da Camera', imagem_cv)
            cv2.waitKey(1)
            self.contagem_regressiva -= 1
        elif self.contagem_regressiva == 0:
            self.print_in_imagem(imagem_cv, "Initializy calibration...")
            cv2.imshow('Calibracao da Camera', imagem_cv)
            cv2.waitKey(1)
            self.contagem_regressiva -= 1
            time.sleep(1)
        else:
            self.processar_imagem(imagem_cv)

    def print_in_imagem(self, img, msg):
        cv2.putText(img, msg, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)

    def load_config_initial(self):
        # Verifica se o arquivo de calibração existe
        if os.path.isfile(self.calibration_file_path):
            try:
                # Abre o arquivo de calibração e carrega os dados
                with open(self.calibration_file_path, 'r') as arquivo:
                    calib_data = yaml.safe_load(arquivo)
                    # Verifica se as chaves esperadas existem no dicionário
                    if 'camera_matrix' in calib_data:
                        # Carrega os dados de calibração na matriz da câmera e nos coeficientes de distorção
                        self.camera_matrix = np.array(calib_data['camera_matrix'], dtype=float)
                    else:
                        raise KeyError("As chaves 'camera_matrix' não estão presentes no arquivo de calibração.")
                    if 'dist_coeffs' in calib_data:
                        self.dist_coeffs = np.array(calib_data['dist_coeffs'], dtype=float)
                    else:
                        raise KeyError("As chaves 'dist_coeffs' não estão presentes no arquivo de calibração.")
                    print(f"Dados de calibração carregados de {self.calibration_file_path}")
                    
            except yaml.YAMLError as e:
                self.get_logger().error(f"Erro ao ler o arquivo YAML: {e}")
            except KeyError as e:
                self.get_logger().error(f"{e}")
            except Exception as e:
                self.get_logger().error(f"Erro ao carregar o arquivo de configuração da câmera: {e}")
        else:
            self.get_logger().error(f"Arquivo de configuração da câmera não encontrado: {self.calibration_file_path}")
            
    def processar_imagem(self, imagem):
        # Converte a imagem para escala de cinza para detecção de marcadores
        cinza = cv2.cvtColor(imagem, cv2.COLOR_BGR2GRAY)
        cantos, ids, _ = aruco.detectMarkers(cinza, self.aruco_dict, parameters=self.parametros)

        # Se algum marcador ArUco foi detectado
        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(cantos, self.marker_length_m, self.camera_matrix, self.dist_coeffs)

            for i in range(len(ids)):
                # Desenha os contornos e os eixos dos marcadores ArUco
                aruco.drawDetectedMarkers(imagem, cantos, ids)
                # Exibir a pose estimada para o primeiro marcador
                aruco.drawAxis(imagem, self.camera_matrix, self.dist_coeffs, rvecs[0], tvecs[0], self.marker_length_m)
                distance = np.linalg.norm(tvecs[0][0])

                # Calcula o tamanho percebido do marcador
                largura_percebida_cm, altura_percebida_cm = self.calcular_tamanho_percebido(cantos[i][0], tvecs[i])

                    
                # Exibe o tamanho estimado do marcador na imagem
                cv2.putText(imagem, f"Tamanho estimado: {largura_percebida_cm:.2f}cm x {altura_percebida_cm:.2f}cm", (10, 30 * (i+1)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

                # Atualiza os parâmetros da câmera ficticiamente para aproximar a distância desejada
                self.ajustar_parametros_camera(distance)

                # Escreve na imagem a distância estimada até o marcador
                self.print_in_imagem(imagem, f"Distancia estimada: {distance:.2f} m")

                if abs(distance - self.distance_to_marker_m) < 0.001:
                    self.salvar_dados_calibracao(imagem)
                    return  # Sai da função para não processar mais imagens


        # Exibe a imagem
        cv2.imshow('Calibracao da Camera', imagem)
        cv2.waitKey(1)

    def calcular_tamanho_percebido(self, corner, tvec):
        # Considera a distância entre os cantos superior esquerdo e superior direito para largura
        # e os cantos superior esquerdo e inferior esquerdo para altura.
        largura_pixel = np.linalg.norm(corner[0] - corner[1])
        altura_pixel = np.linalg.norm(corner[0] - corner[3])

        # Convertendo pixels em metros usando a distância até o marcador (tvec) e a matriz da câmera.
        largura_m = (largura_pixel / self.camera_matrix[0, 0]) * tvec[0, 2]
        altura_m = (altura_pixel / self.camera_matrix[1, 1]) * tvec[0, 2]

        # Convertendo metros em centímetros para exibição.
        largura_cm = largura_m * 100
        altura_cm = altura_m * 100

        return largura_cm, altura_cm

    def ajustar_parametros_camera(self, distance):
        # Simula o ajuste dos parâmetros da câmera para alinhar a distância percebida com a real
        # Este código apenas ajusta o valor da matriz da câmera para simular uma aproximação da distância desejada
        # Em um cenário real, isso seria feito com base em múltiplas imagens e um processo de otimização
        if distance < self.distance_to_marker_m:
            self.camera_matrix[0][0] += 1  # Ajusta a focal length para simular um afastamento da câmera
            self.camera_matrix[1][1] += 1
        elif distance > self.distance_to_marker_m:
            self.camera_matrix[0][0] -= 1  # Ajusta a focal length para simular uma aproximação da câmera
            self.camera_matrix[1][1] -= 1

        # Atualiza os dados de calibração
        self.dados_calibracao['camera_matrix'] = self.camera_matrix.tolist()

    def salvar_dados_calibracao(self, img):
        # Verifica se a matriz da câmera e os coeficientes de distorção foram definidos
        if hasattr(self, 'camera_matrix') and hasattr(self, 'dist_coeffs'):
            dados_calibracao = {
                'camera_matrix': self.camera_matrix.tolist(),
                'dist_coeffs': self.dist_coeffs.tolist(),
            }
            # Salva os dados da calibração usando o caminho do arquivo fornecido pelo parâmetro
            try:
                with open(self.calibration_file_path, 'w') as arquivo:
                    yaml.dump(dados_calibracao, arquivo, default_flow_style=False)
                self.get_logger().info(f"Dados de calibração salvos em {self.calibration_file_path}")
                self.print_in_imagem(img, "Sucesso...")
                cv2.imshow('Calibracao da Camera', img)
                cv2.waitKey(1)
                time.sleep(3)
            except Exception as e:
                self.get_logger().error(f"Erro ao salvar o arquivo de configuração da câmera: {e}")
        else:
            self.get_logger().error("Matriz da câmera e coeficientes de distorção não estão definidos.")
        
        # Após salvar, fechar o programa
        self.encerrar_calibracao()

    def encerrar_calibracao(self):
        self.subscription.destroy()
        cv2.destroyAllWindows()
        self.destroy_node()  # Destruir o nó
        rclpy.shutdown()  # Encerrar o rclpy

def main(args=None):
    rclpy.init(args=args)
    calibrador = CameraCalibrador()
    while rclpy.ok():
        rclpy.spin_once(calibrador)

if __name__ == '__main__':
    main()
