
from utils import Command, CommandQueue, CommandType, Graph, Path
class Navigation:
    def __init__(self, origin: int, destiny: int) -> None:
        self.origin = origin
        self.destiny = destiny
        self.graph = Graph()
        self.graph.load_from_file('support/graph_data.txt')
        self.target_actual:Path = None
        self.targets_list:list[Path] = None

    

    def is_exist_rote(self, origin=None, destiny=None) -> bool:    
        orig = self.origin
        dest = self.destiny
        if origin is not None:
            orig = origin
        if destiny is not None:
            dest = destiny
        # Busca todos os caminhos do vértice id_origen para o vértice id_destiny
        all_paths_from_destiny = self.graph.load_paths_if_exist( orig, dest )
        return len(all_paths_from_destiny) > 0
    
    def select_a_route(self, origin=None, destiny=None):    
        orig = self.origin
        dest = self.destiny
        if origin is not None:
            orig = origin
        if destiny is not None:
            dest = destiny
        # Busca todos os caminhos do vértice id_origen para o vértice id_destiny
        all_paths_from_destiny = self.graph.load_paths_if_exist( orig, dest )
        self.targets_list:list[Path] = all_paths_from_destiny[0] # pega a rota menor / primeira rota

    def set_origin(self, origin: int) -> None:
        self.origin = origin

    def set_destiny(self, destiny: int) -> None:
        self.destiny = destiny

    def is_next(self) -> bool:
        return len(self.targets_list) > 0
    
    def get_next(self) -> Path:
        self.target_actual = self.targets_list.pop(0)
        self.origin = self.target_actual.id_origen
        return self.target_actual