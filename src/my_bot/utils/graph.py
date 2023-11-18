from orientation import Orientation

class Graph:
    def __init__(self):
        self.adjacency_list = {}  # Dicionário para armazenar a lista de adjacências

    def load_from_file(self, file_path):
        # Carrega o grafo a partir de um arquivo
        with open(file_path, 'r') as file:
            for line in file:
                parts = line.split(':')
                if len(parts) == 2:
                    node = int(parts[0].strip())
                    connections = parts[1].strip().split(',')
                    for connection in connections:
                        target_node, orientation = connection.strip().split()
                        self.add_edge(node, int(target_node), Orientation[orientation])

    def add_edge(self, from_node, to_node, orientation):
        # Adiciona uma aresta ao grafo
        if from_node not in self.adjacency_list:
            self.adjacency_list[from_node] = []
        self.adjacency_list[from_node].append((to_node, orientation))

    def find_all_paths(self, start_vertex, end_vertex, path=[]):
        # Encontra todos os caminhos entre dois vértices usando DFS
        path = path + [start_vertex]
        if start_vertex == end_vertex:
            return [path]
        if start_vertex not in self.adjacency_list:
            return []
        paths = []
        for (vertex, _) in self.adjacency_list[start_vertex]:
            if vertex not in path:
                extended_paths = self.find_all_paths(vertex, end_vertex, path)
                for p in extended_paths:
                    paths.append(p)
        return paths

    def get_next_node_orientation(self, current_node, destination_node):
        if current_node in self.adjacency_list:
            for node, orientation in self.adjacency_list[current_node]:
                if node == destination_node:
                    return node, orientation
        return None, None

    def save_to_file(self, file_path):
        with open(file_path, 'w') as file:
            for node, edges in self.adjacency_list.items():
                connections = ', '.join(f"{target_node} {orientation.name}" for target_node, orientation in edges)
                file.write(f"{node}: {connections}\n")

    def get_paths_with_orientation(self, start_vertex, end_vertex):
        # Obtém os caminhos com as orientações para cada passo
        all_paths = self.find_all_paths(start_vertex, end_vertex)
        paths_with_orientation = []
        for path in all_paths:
            path_with_orientation = []
            for i in range(len(path)-1):
                for (vertex, orientation) in self.adjacency_list[path[i]]:
                    if vertex == path[i+1]:
                        path_with_orientation.append((vertex, orientation))
                        break
            paths_with_orientation.append(path_with_orientation)
        return paths_with_orientation
    
    def find_paths(self, start_vertex, end_vertex):
        """
        Encontra todos os caminhos possíveis do vértice de início ao vértice de destino.

        :param start_vertex: O vértice de início.
        :param end_vertex: O vértice de destino.
        :return: Uma lista de caminhos, onde cada caminho é uma lista de tuplas (nó, orientação).
        """
        def find_paths_util(current_vertex, end_vertex, visited, path):
            # Marca o vértice atual como visitado e adiciona ao caminho
            visited[current_vertex] = True
            path.append(current_vertex)

            # Se o vértice atual é o de destino, então armazena o caminho atual
            if current_vertex == end_vertex:
                all_paths.append(path.copy())
            else:
                # Se o vértice atual não é o de destino, então recursivamente
                # busca por todos os vértices adjacentes que ainda não foram visitados
                if current_vertex in self.adjacency_list:
                    for (next_vertex, orientation) in self.adjacency_list[current_vertex]:
                        if not visited[next_vertex]:
                            find_paths_util(next_vertex, end_vertex, visited, path)
            
            # Remove o vértice atual do caminho e marca como não visitado
            path.pop()
            visited[current_vertex] = False

        # Inicializa os caminhos, uma variável para armazenar todos os caminhos encontrados
        all_paths = []
        # Inicializa o dicionário de visitados com False para todos os vértices
        visited = {vertex: False for vertex in self.adjacency_list}
        # Utilitário para encontrar caminhos
        find_paths_util(start_vertex, end_vertex, visited, [])

        # Converte os caminhos para incluir as orientações
        paths_with_orientations = []
        for path in all_paths:
            path_with_orientations = []
            for i in range(len(path) - 1):
                for (vertex, orientation) in self.adjacency_list[path[i]]:
                    if vertex == path[i + 1]:
                        path_with_orientations.append((vertex, orientation))
                        break
            # Não esqueça de adicionar o vértice final com sua orientação
            last_vertex = path[-1]
            for (vertex, orientation) in self.adjacency_list[last_vertex]:
                if vertex == end_vertex:
                    path_with_orientations.append((end_vertex, orientation))
                    break
            paths_with_orientations.append(path_with_orientations)

        return paths_with_orientations