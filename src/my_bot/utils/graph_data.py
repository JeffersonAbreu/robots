# Exemplo de uso:
from graph import Graph

graph = Graph()
graph.load_from_file('/home/jeff/tcc/robots/support/graph_data.txt')  # Substitua pelo caminho do seu arquivo

# Substitua 1 e 5 pelos vértices de início e fim desejados
start_vertex = 25
end_vertex = 4
paths_with_orientation = graph.get_paths_with_orientation(start_vertex, end_vertex)

# Imprime os caminhos possíveis com as orientações
for path in paths_with_orientation:
    print(" -> ".join(f"{id} {orientation.name}" for id, orientation in path))

# busca de profundidade

paths = graph.find_paths(start_vertex, end_vertex)

# Imprime os caminhos possíveis entre os vértices start_vertex e end_vertex
for path in paths:
    print(f"De {start_vertex} para {end_vertex} = Caminho:", " -> ".join(f"{id} {orientation.name}" for id, orientation in path))