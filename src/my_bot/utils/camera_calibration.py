import yaml
from my_aruco.constants import CAMERA_MATRIX, DIST_COEFFS, MARKER_SIZE

import subprocess

# Função para executar um comando e esperar por uma entrada do usuário
def run_command_and_wait(command):
    # Execute o comando no terminal
    process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    # Aguarde o comando terminar e obtenha a saída
    stdout, stderr = process.communicate()
    
    # Imprima a saída do comando
    print(stdout.decode())
    
    # Imprima quaisquer erros se ocorrerem
    if stderr:
        print(stderr.decode())
    

def save_calibration():
    calibration_data = {
        'camera_matrix': CAMERA_MATRIX.tolist(),
        'dist_coeffs': DIST_COEFFS.tolist(),
        'marker_size': MARKER_SIZE
    }
    
    with open('camera_calibration.yaml', 'w') as outfile:
        yaml.dump(calibration_data, outfile, default_flow_style=False)
    
    print("Calibração simulada salva com sucesso.")

if __name__ == "__main__":
    input("Pressione Enter para continuar...")
    run_command_and_wait('python3 my_bot/utils/aruco/main.py')
    # Aguarde o usuário pressionar uma tecla
    save_calibration()

 ###########################################
##  EM UM OUTRO MOMENTO TERA O CALIBRADOR  ##
 ###########################################