import numpy as np

class AdjacencyMatrix:
    pass
  # Completando a lista de arestas com os dados fornecidos

arestas = [
    ( 1,  2,  8.5), ( 1, 28,  3.0)
    ( 2,  1,  8.5), ( 2,  3,  8.0)
    ( 3,  2,  8.0), ( 3,  4,  1.5), ( 3,  5,  2.0)
    ( 4,  3,  1.5), ( 4,  6, 13.0)
    ( 5,  3,  2.0)
    ( 6,  4, 13.0), ( 6,  7,  5.0), ( 6,  8,  2.0)
    ( 7,  6,  5.0)
    ( 8,  6,  2.0), ( 8,  9,  2.0), ( 8, 10,  6.0)
    ( 9,  8,  2.0)
    (10,  8,  6.0), (10, 11,  2.0), (10, 12,  6.0)
    (11, 10,  2.0)
    (12, 10,  6.0), (12, 13,  2.0), (12, 18,  5.0)
    (13, 12,  2.0), (13, 14,  6.0), (13, 15,  1.5)
    (14, 13,  6.0)
    (15, 13,  1.5), (15, 16,  5.0)
    (16, 15,  5.0), (16, 17,  7.5), (16, 25,  1.0)
    (17, 16,  7.5), (17, 18,  5.5)
    (18, 17,  5.5), (18, 19,  4.5)
    (19, 18,  4.5), (19, 20,  9.0)
    (20, 19,  9.0), (20, 21,  5.5), (20, 23,  4.0)
    (21, 20,  5.5), (21, 22,  4.0)
    (22, 21,  4.0), (22, 23,  5.5)
    (23, 20,  4.0), (23, 22,  5.5), (23, 24,  5.5)
    (24, 18,  5.0), (24, 23,  5.5)
    (25, 16,  1.0), (25, 26,  7.5)
    (26, 25,  7.5), (26, 27,  3.0)
    (27, 26,  3.0), (27, 28,  4.5), (27, 29, 10.5)
    (28,  1,  3.0), (28, 27,  4.5)
    (29, 27, 10.5), (29, 30,  2.0)
    (30, 29,  2.0)
]


def criar_matriz_adjacencia(arestas, num_vertices):
    """
    Cria uma matriz de adjacência a partir de uma lista de arestas.
    
    :param arestas: Lista de tuplas (origem, destino, peso)
    :param num_vertices: Número total de vértices no grafo
    :return: Matriz de adjacência numpy
    """
    matriz = np.zeros((num_vertices, num_vertices))
    for origem, destino, peso in arestas:
        matriz[origem-1][destino-1] = peso  # Ajustando para índice base-0
    return matriz

# Lista de arestas e pesos (assumindo peso 1 para cada conexão)

# Número de vértices (assumindo que o maior número é o total de vértices)
num_vertices = 30

# Criando a matriz de adjacência
matriz_adjacencia = criar_matriz_adjacencia(arestas, num_vertices)

# Exibindo a matriz de adjacência
print(matriz_adjacencia)


