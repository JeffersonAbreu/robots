import numpy as np
import cv2
import yaml

# Parâmetros padrão da câmera (substitua pelos seus valores após a calibração)
default_camera_matrix = np.array([[1000, 0, 320],
                                  [0, 1000, 240],
                                  [0, 0, 1]], dtype=float)
default_dist_coeffs = np.zeros((5, 1))  # Substitua pelos seus coeficientes após a calibração

# Configurações padrão do marcador ArUco
default_marker_size = 0.1  # Tamanho do marcador em metros

# Caminho para o arquivo de configuração (pode ser alterado se necessário)
config_file_path = 'camera_calibration.yaml'

def save_camera_config(camera_matrix, dist_coeffs, marker_size):
    """ Salva a matriz da câmera, os coeficientes de distorção e o tamanho do marcador em um arquivo YAML. """
    with open(config_file_path, 'w') as f:
        yaml.dump({
            'camera_matrix': camera_matrix.tolist(),
            'dist_coeffs': dist_coeffs.tolist(),
            'marker_size': marker_size
        }, f)

def load_camera_config():
    """ Carrega a matriz da câmera, os coeficientes de distorção e o tamanho do marcador de um arquivo YAML. """
    try:
        with open(config_file_path, 'r') as f:
            config = yaml.safe_load(f)
            camera_matrix = np.array(config['camera_matrix'])
            dist_coeffs = np.array(config['dist_coeffs'])
            marker_size = config['marker_size']
            return camera_matrix, dist_coeffs, marker_size
    except FileNotFoundError:
        print(f"Arquivo de configuração {config_file_path} não encontrado. Usando valores padrão.")
        return default_camera_matrix, default_dist_coeffs, default_marker_size
