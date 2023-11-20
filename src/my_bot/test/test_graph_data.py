from typing import List
from enum import Enum, auto
class Orientation(Enum):
    NORTH = auto()
    SOUTH = auto()
    EAST = auto()
    WEST = auto()

class Path:
    def __init__(self, id_origen: int, id_destiny: int, orientation: Orientation):
        self.id_origen: int = id_origen
        self.id_destiny: int = id_destiny
        self.orientation: Orientation = orientation

    def __repr__(self) -> str:
        return f"{self.id_origen} -> {self.id_destiny} {self.orientation.name}"

class Graph:
    def __init__(self):
        self.paths: List[Path] = []

    def load_from_file(self, file_path: str) -> None:
        with open(file_path, 'r') as file:
            for line in file:
                id_origen, rest = line.strip().split(':')
                connections = rest.split(',')
                for connection in connections:
                    id_destiny, orientation = connection.strip().split()
                    self.paths.append(Path(int(id_origen), int(id_destiny), Orientation[orientation]))
        self.paths.sort(key=lambda path: (path.id_origen, path.id_destiny))


    def has_next(self, current_path):
        # Verifica se existe um próximo caminho a partir do vértice de destino do caminho atual
        return any(path.id_origen == current_path.id_destiny for path in self.paths)

      
    def load_paths_if_exist(self, origin, destiny):
        # Carrega todos os caminhos possíveis entre a origem e o destino
        def dfs(current_vertex, destination_vertex, current_path, all_paths):
            # Verifica se o último vértice do caminho atual é o destino
            if current_path[-1].id_destiny == destination_vertex:
                all_paths.append(current_path[:])
                return
            # Continua a busca pelos caminhos possíveis
            for p in self.paths:
                if p.id_origen == current_vertex and not any(cp.id_destiny == p.id_destiny for cp in current_path):
                    dfs(p.id_destiny, destination_vertex, current_path + [p], all_paths)

        all_paths = []  # Armazena todos os caminhos encontrados
        initial_paths = [p for p in self.paths if p.id_origen == origin]
        for p in initial_paths:
            dfs(p.id_destiny, destiny, [p], all_paths)

        # Ordena os caminhos encontrados pelo tamanho
        all_paths.sort(key=len)
        return all_paths

graph = Graph()
graph.load_from_file('/home/jeff/tcc/robots/support/graph_data.txt')  # Substitua pelo caminho do seu arquivo

id_destiny = 30
# Busca todos os caminhos do vértice 1 para o vértice 30 e os imprime
all_paths_from_destiny = graph.load_paths_if_exist(25, id_destiny)
'''
for path in all_paths_from_1_to_30:
    print(" -> ".join(f"{vertex.id_destiny} {vertex.orientation.name}" for vertex in path))
'''

target_ = None
targets_list = []

while True:
    if len(targets_list) == 0:
        if len(all_paths_from_destiny) == 0:
            target_ = None
        else:
            targets_list = all_paths_from_destiny.pop(0)
            target_ = targets_list.pop(0)
    elif target_ is not None and graph.has_next(target_):
            target_ = targets_list.pop(0)
    if target_ is None:
        print("Sem caminhos disponiveis")
        break
    else:
        if target_.id_destiny == id_destiny:
            print("Chegamos no destino")
            print("robot.stop()")
            all_paths_from_destiny.clear()
            break
        else:
            print(f"robot.sensor_camera.fix_target({target_.id_destiny})")
            print(f"robot.turn_by_orientation({target_.orientation})")

