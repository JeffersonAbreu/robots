import numpy as np
def calculate_new_angle(pixel_error, distance_to_aruco, image_width, fov_width):
    # Convertendo o erro em pixels para graus
    pixels_to_degrees = 0.1 # exemplo: 10 pixels equivalem a 1 grau
    angle_error = pixel_error * pixels_to_degrees
    
    # Ajustando o ângulo com base na distância
    # Quanto maior a distância, maior será o ajuste necessário
    if distance_to_aruco != 0:
        angle_adjustment = np.arctan(pixel_error / distance_to_aruco)
        new_angle = np.degrees(angle_adjustment)
    else:
        new_angle = angle_error  # Se a distância for 0, o ajuste é baseado apenas no erro de pixel
    
    return new_angle

# Exemplo de uso
image_width = 640  # Largura da imagem em pixels
fov_width = 60.0   # Campo de visão horizontal em graus
pixel_error = 44  # Exemplo de erro em pixels
distance_to_aruco = 4.8  # Exemplo de distância em metros

new_angle = calculate_new_angle(pixel_error, distance_to_aruco, image_width, fov_width)
print(f"O robô deve girar {new_angle:.2f} graus para alinhar com o ArUco.")

index = 361
index = max(0, min(index, 270))
print(f"index: {index}")