
import os
import shutil
import math
import cv2
import numpy as np
from xml.dom.minidom import parse
import xml.etree.ElementTree as ET

# Configurações do ArUco
from constants import DISTANCE_TO_WALL, WALL_LARG, ARUCO_DICT, MARKER_SIZE, TAG_WIDTH, ANGLE_TAG_DOBLE

HEIGHT_FROM_THE_FLOOR = 0.1
CREATE_POINTS = False # criar a placa no chão a frente da area do aruco
from enum import Enum, auto
class Orientation(Enum):
    NORTH = auto()
    SOUTH = auto()
    EAST = auto()
    WEST = auto()
    NE = auto() # Nordeste: Northeast (NE)
    SE = auto() # Sudeste: Southeast (SE)
    NW = auto() # Noroeste: Northwest (NW)
    SW = auto() # Sudoeste: Southwest (SW)

    def format_degrees(orientation):
        if orientation == Orientation.NORTH:
            degrees = 0
        elif orientation == Orientation.NE:
            degrees = 45
        elif orientation == Orientation.EAST:
            degrees = 90
        elif orientation == Orientation.SE:
            degrees = 135
        elif orientation == Orientation.SOUTH:
            degrees = 180
        elif orientation == Orientation.SW:
            degrees = 225
        elif orientation == Orientation.WEST:
            degrees = 270
        elif orientation == Orientation.NW:
            degrees = 315
        return degrees


class Tag:
    def __init__(self, name: str, pose: list[float], parent: str = None, uri: str = None):
        self.name = name
        self.pose = pose
        self.parent = parent
        self.uri = uri
    
    def getPoseValues(self) -> str:
        return "{} {} {} {} {} {}".format(*self.pose)
    
    def getKey(self):
        return int(self.name.split('_')[1]) if self.name.startswith('tag_') else None
    
    def copy(self):
        return Tag(self.name, self.pose.copy(), self.parent, self.uri)
    
class TagModel:
    def __init__(self, origen, destiny) -> None:
        self.origen = origen
        self.destiny = destiny
        remove_dir(destiny)
        create_dir(destiny)

    def create_model(self, key, name):
        path_destiny = os.path.join(self.destiny, name)
        copy(self.origen, path_destiny)
        modify_model_config(path_destiny, name)
        modify_model_sdf(path_destiny, name)
        name_image = maker(path_destiny, key, name)
        modify_tag_dae(path_destiny, name_image)



def create_dir(_path):
    if not os.path.isdir(_path):
       os.makedirs(_path)

def remove_dir(_path):
    if os.path.isdir(_path):
       shutil.rmtree(_path)

def copy(origen, destiny):
    # Copie a pasta e todos os seus conteúdos
    if os.path.isdir(origen):
       shutil.copytree(origen, destiny)
    else:
       print(f"'{origen}' não é um diretório válido.")

def modify_model_config(path_destiny, name):
    # modify model.config
    model_config_path = os.path.join(path_destiny, "model.config")
    dom = parse(model_config_path)
    for node in dom.getElementsByTagName('name'):
        node.firstChild.nodeValue = name
        break
    f = open(model_config_path, 'w+')
    f.write(dom.toxml())
    f.close()

def modify_model_sdf(path_destiny, name):
    """
    Modifica o arquivo SDF do modelo para incluir o marcador ArUco com a escala correta.

    :param path_destiny: Caminho do diretório onde o arquivo SDF será salvo.
    :param name: Nome do modelo.
    """
    # Caminho para o arquivo SDF do modelo sem a versão
    model_noversion_sdf_path = os.path.join(path_destiny, "model.sdf")
    
    # Carrega o arquivo SDF como um DOM XML
    dom = parse(model_noversion_sdf_path)

    # Altera o nome do modelo no arquivo SDF
    for node in dom.getElementsByTagName('model'):
        node.attributes["name"].value = name
        break

    # Abre o arquivo SDF para escrita
    with open(model_noversion_sdf_path, 'w') as f:
        # Escreve o XML modificado no arquivo
        f.write(dom.toxml())

def modify_tag_dae(path_destiny, image_name):
    dom = parse(os.path.join(path_destiny, "tag.dae"))
    for node in dom.getElementsByTagName('init_from'):
        node.firstChild.nodeValue = image_name
        break

    f = open(os.path.join(path_destiny, "tag.dae"), 'w+')
    # Write the modified xml file
    f.write(dom.toxml())
    f.close()

class Point:
    """
    Representação do Ponto de Interesse

    Parâmetros:\n
    key : ID\n
    orientacion : Sentido em que o ponto será criado\n
    x e y : localização
    """
    def __init__(self, key: int, orientation: Orientation, x: float = 0.0, y: float = 0.0):
        self.key = key 
        self.orientation = orientation
        self.x = x
        self.y = y

    def get_pose(self):
        _x = self.x
        _y = self.y
        _z = 0
        _roll = 0
        _pitch = 0
        _yaw = 0

        if   self.orientation == Orientation.SOUTH:
            _x -= DISTANCE_TO_WALL
        elif self.orientation == Orientation.NORTH:
            _x += DISTANCE_TO_WALL
        elif self.orientation == Orientation.WEST:
            _y += DISTANCE_TO_WALL
        elif self.orientation == Orientation.EAST:
            _y -= DISTANCE_TO_WALL

        return [_x, _y, _z, _roll, _pitch, _yaw]
        
    def create_tag(self) -> Tag:
      name = f"point_{self.key:02}"
      pose = self.get_pose()
      uri = "model://point"
      return Tag(name, pose, None, uri)

class PlantBase:
  def __init__(self, path) -> None:
    self.file = path

  def __removeAllTag(self, tag_name) -> None:
    dom = parse(self.file)
    includes = dom.getElementsByTagName('include')
    for include in includes:
        names = include.getElementsByTagName('name')
        if names:
            name = names[0]
            if name.firstChild.data.startswith(tag_name):
                include.parentNode.removeChild(include)

    with open(self.file, 'w') as file:
        dom.writexml(file)

  def removeAllTagsAruco(self) -> None:
     self.__removeAllTag('tag_')
    
  def removeAllTagsPoint(self) -> None:
     self.__removeAllTag('point_')
     

  
  def add_tag(self, tag: Tag) -> None:
    # modify model.sdf
    
    dom = parse(self.file)
    model = dom.getElementsByTagName('model')[0]
    
    # add include
    include = dom.createElement("include")

    # name
    name = dom.createElement("name")
    name.appendChild(dom.createTextNode(tag.name))
    include.appendChild(name)
    # pose
    values = dom.createTextNode(tag.getPoseValues())
    pose = dom.createElement("pose")
    pose.setAttribute("relative_to", tag.parent)
    pose.appendChild(values)
    include.appendChild(pose)
    # URI
    uri = dom.createElement("uri")
    uri.appendChild(dom.createTextNode(tag.uri))
    include.appendChild(uri)

    model.appendChild(include)

   
    with open(self.file, 'w') as file:
        dom.writexml(file)


  def ler_includes(self) -> list[Tag]:
    tree = ET.parse(self.file)
    root = tree.getroot()

    includes = {}

    # Ler todas as tags 'include' e armazenar as informações necessárias
    for include in root.findall('model/include'):
        name = include.find('name').text if include.find('name') is not None else None
        pose = list(map(float, include.find('pose').text.split())) if include.find('pose') is not None else [0, 0, 0, 0, 0, 0]
        parent = include.find('pose').get('relative_to') if include.find('pose').get('relative_to') is not None else None
        includes[name] = Tag(name, pose, parent)

    return includes


def maker(destiny, marker_id:int, name):
    """
    Gera um marcador ArUco e salva como uma imagem PNG.

    :param destiny: Caminho do diretório onde a imagem será salva.
    :param marker_id: ID do marcador ArUco a ser gerado.
    :param name: Nome base para o arquivo de imagem.
    :return: O nome do arquivo da imagem gerada.
    """
    # Defina o tamanho do marcador em centímetros e a resolução de impressão em DPI
    tamanho_cm = MARKER_SIZE
    dpi = 300

    # Converta o tamanho do marcador para pixels
    tamanho_pixels = int((tamanho_cm / 2.54) * dpi)  # 2.54 cm por polegada

    # Escolha o dicionário ArUco
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)

    # Especifique o ID do marcador ArUco que você deseja gerar

    # Crie uma imagem em branco onde o marcador será desenhado
    marker_image = np.zeros((tamanho_pixels, tamanho_pixels), dtype=np.uint8)

    # Gere o marcador ArUco
    marker_image = cv2.aruco.drawMarker(aruco_dict, marker_id, tamanho_pixels, marker_image, borderBits=1)

    name = f"aruco_{name}.png"
    destiny = os.path.join(destiny, name)
    # Salve a imagem do marcador
    cv2.imwrite(destiny, marker_image)

    return name



class Aruco_:
    """
    Representação do ARUCO

    Parâmetros:\n
    key : ID da tag aruco\n
    orientacion : Sentido em que a tag está apontada\n
    relative_to : É a parede onde a tag será fixada\n
    xy : deslocamento da tag em relação a parede
    """
    def __init__(self, key: int, orientation: Orientation, relative_to: str, xy: float = 0.0, doble: bool = False):
        self.key = key 
        self.orientation = orientation
        self.relative_to = relative_to
        self.xy = xy
        self.doble = doble
    
    def __lt__(self, other):
        """Defina aqui como você quer comparar os objetos Aruco_"""
        return self.key < other.key

    def get_pose(self):
        _x = 0
        _y = 0
        _z = HEIGHT_FROM_THE_FLOOR
        _roll = 0
        _pitch = 0
        _yaw = 0

        if self.orientation == Orientation.SOUTH:
            _x = -WALL_LARG
            _y = self.xy
            _yaw = math.pi
        elif self.orientation == Orientation.NORTH:
            _x = WALL_LARG
            _y = self.xy
        elif self.orientation == Orientation.WEST:
            _x = self.xy
            _y = WALL_LARG
            _yaw = math.pi / 2
        elif self.orientation == Orientation.EAST:
            _x = self.xy
            _y = -WALL_LARG
            _yaw = -math.pi / 2


        return [_x, _y, _z, _roll, _pitch, _yaw]
        
    def create_tag(self) -> Tag:
      name = f"tag_{self.key:02}"
      pose = self.get_pose()
      uri = "model://" + os.path.join('ar_tags', name)
      return Tag(name, pose, self.relative_to, uri)

def calcular_coordenadas(includes):
    # Criar um dicionário para mapear os nomes aos objetos
    def get_parent(nome):
        return includes[nome]
    
    def get_coords(tag):
        x, y, _, _, _, _ = tag.pose
        return x, y
    

    for tag_name in includes:
        # Somar as coordenadas com as do parente, se houver
        tag : Tag = includes[tag_name]
        tag_x, tag_y = get_coords(tag)
        parent = tag.parent
        if parent != None:
            parent = get_parent(tag.parent)
            parent_x, parent_y = get_coords(parent)
            tag_x += parent_x
            tag_y += parent_y
            includes[tag_name].pose[0] = tag_x
            includes[tag_name].pose[1] = tag_y

    return includes

def filtrar_tags(includes):
    '''Filtrar apenas as tags que começam com "tag"'''
    tags : list[Tag] = []
    for name in includes:
        if name.startswith('tag_'):
            tags.append(includes[name])

    return tags

def check(arucos: list[Aruco_]) -> bool:
    arucos.sort()
    old = arucos.pop(0)
    for ar in arucos:
        if ar.key == old.key:
            return True
        old = ar
    
    return False

def calc_side_triangle_retangle(hypotenuse, angle):
    # Converte o ângulo de graus para radianos
    angulo_rad = math.radians(angle)

    # Calcula o lado oposto usando seno
    opposite_side = hypotenuse * math.sin(angulo_rad)

    # Calcula o lado adjacente usando cosseno
    adjacency_side = hypotenuse * math.cos(angulo_rad)

    return opposite_side, adjacency_side


def create_doble_arucos_(tag: Tag, orientation: Orientation):
    opposite_side, adjacency_side = calc_side_triangle_retangle(TAG_WIDTH, ANGLE_TAG_DOBLE)
    INCLINATION_ANGLE = math.radians(ANGLE_TAG_DOBLE)#Angulo de inclinação em radianos
    # Calcula o lado oposto (wall_offset) usando a fórmula para um triângulo retângulo
    WALL_OFFSET = opposite_side / 2  # Lado oposto calculado
    SLIDE_TO_SIDE = adjacency_side / 1.5

    def tag_duplicate(is_tag_left):
        _tag = tag.copy()
        _tag.name = tag.name + '_LEFT' if is_tag_left else tag.name + '_RIGHT'
        _x, _y, _z, _roll, _pitch, _yaw = _tag.pose
        if orientation == Orientation.SOUTH:
            _x   -= WALL_OFFSET
            _y   += SLIDE_TO_SIDE if is_tag_left else -SLIDE_TO_SIDE
            _yaw += -INCLINATION_ANGLE if is_tag_left else INCLINATION_ANGLE
        elif orientation == Orientation.NORTH:
            _x   += WALL_OFFSET
            _y   += -SLIDE_TO_SIDE if is_tag_left else SLIDE_TO_SIDE
            _yaw += -INCLINATION_ANGLE if is_tag_left else INCLINATION_ANGLE
        elif orientation == Orientation.WEST:
            _x   += SLIDE_TO_SIDE if is_tag_left else -SLIDE_TO_SIDE
            _y   += WALL_OFFSET
            _yaw += -INCLINATION_ANGLE if is_tag_left else INCLINATION_ANGLE
        elif orientation == Orientation.EAST:
            _x   += -SLIDE_TO_SIDE if is_tag_left else SLIDE_TO_SIDE
            _y   -= WALL_OFFSET
            _yaw += -INCLINATION_ANGLE if is_tag_left else INCLINATION_ANGLE

        _tag.pose = [_x, _y, _z, _roll, _pitch, _yaw]
        return _tag

    tag_left = tag_duplicate(True)
    tag_right = tag_duplicate(False)

    return tag_left, tag_right




def main(arucos: list[Aruco_]):
    dir_origen = os.path.dirname(os.path.realpath(__file__))
    dir_tag_model_origen = os.path.join(dir_origen, 'tag_model')
    dir_base = os.path.realpath(os.path.join(dir_origen, '..', 'models'))
    maze_sdf = os.path.join(dir_base, 'maze', 'model.sdf')
    dir_tags_model_destiny = os.path.join(dir_base, 'ar_tags')

    tag_model = TagModel(dir_tag_model_origen, dir_tags_model_destiny)
    maze = PlantBase(maze_sdf)
    maze.removeAllTagsAruco()
    maze.removeAllTagsPoint()
    
    area_of_interest = {}
    key_multiple = check(arucos.copy())


    for ar in arucos:
        tag = ar.create_tag()
        tag_name = tag.name
        if key_multiple:
            tag.name = tag_name + '_' + ar.orientation.name
        if ar.doble:
            tag_left, tag_right = create_doble_arucos_(tag, ar.orientation)
            maze.add_tag(tag_left)
            maze.add_tag(tag_right)
        else:
            maze.add_tag(tag)
        
        if not os.path.isdir(os.path.join(dir_tags_model_destiny, tag_name)):
            """Cria o model somente se não exestir"""
            tag_model.create_model(ar.key, tag_name)
            area_of_interest[tag.name] = {'key': ar.key, 'ori': ar.orientation}

    # Ler e calcular as coordenadas
    includes = maze.ler_includes()
    includes_com_coordenadas = calcular_coordenadas(includes)

    # Filtrar as tags
    tags = filtrar_tags(includes_com_coordenadas)
    if key_multiple:
        """Somente se houver tags multiplas da mesma key"""
        tags = [t for t in tags if t.name in area_of_interest]
    
    list_points: list[Point] = []
    for tag in tags:
        x, y, _, _, _, _ = tag.pose
        key = area_of_interest[tag.name]['key']
        ori = area_of_interest[tag.name]['ori']
        point = Point(key, ori, x, y)
        list_points.append(point)

    if CREATE_POINTS:
      for tag in [t.create_tag() for t in list_points]:
          maze.add_tag(tag)

     

if __name__ == "__main__":
    list_simples = [
      Aruco_( 1, Orientation.SOUTH,"c15", 1.5),
      Aruco_( 2, Orientation.SOUTH,"c14",-2.5),
      Aruco_( 1, Orientation.EAST , "c0", 1.5),
      Aruco_( 3, Orientation.SOUTH,"c12", 0.0),
      Aruco_( 4, Orientation.SOUTH,"c12",-1.5),
      Aruco_( 4, Orientation.WEST ,"c11", 1.5),
      Aruco_( 5, Orientation.WEST , "a7", 2.0),
      Aruco_( 6, Orientation.WEST , "c9",-1.5),
      Aruco_( 7, Orientation.NORTH, "c7",-1.5),
      Aruco_( 8, Orientation.SOUTH, "d7",-2.0),
      Aruco_( 9, Orientation.WEST , "d2", 2.0),
      Aruco_(10, Orientation.SOUTH, "d8", 2.0),
      Aruco_(11, Orientation.EAST , "d4", 2.0),
      Aruco_(12, Orientation.EAST , "d1",-1.5),
      Aruco_(13, Orientation.EAST , "d1", 0.0),
      Aruco_(14, Orientation.NORTH, "d8", 2.0),
      Aruco_(15, Orientation.EAST , "d1", 2.0),
      Aruco_(15, Orientation.SOUTH, "b3", 2.0),
      Aruco_(16, Orientation.WEST , "d9",-2.0),
      Aruco_(17, Orientation.NORTH, "c5",-2.5),
      Aruco_(17, Orientation.WEST , "d4",-1.5),
      Aruco_(18, Orientation.NORTH, "c4",-2.5),
      Aruco_(19, Orientation.NORTH, "c4", 1.5),
      Aruco_(19, Orientation.EAST , "c3",-1.5),
      Aruco_(20, Orientation.EAST , "c2", 2.0),
      Aruco_(21, Orientation.SOUTH, "b2", 2.0),
      Aruco_(21, Orientation.EAST,  "c1", 2.0),
      Aruco_(22, Orientation.SOUTH, "b2",-2.0),
      Aruco_(22, Orientation.WEST,  "b1", 2.0),
      Aruco_(23, Orientation.WEST , "d1", 2.0),
      Aruco_(24, Orientation.WEST , "d1",-1.5),
      Aruco_(25, Orientation.WEST , "a1",-2.0),
      Aruco_(26, Orientation.EAST , "b1", 2.0),
      Aruco_(27, Orientation.SOUTH, "b4",-1.0),
      Aruco_(28, Orientation.NORTH, "b2", 1.5),
      Aruco_(28, Orientation.EAST , "c0",-1.5),
      Aruco_(29, Orientation.SOUTH, "b6",-1.0),
      Aruco_(29, Orientation.WEST,  "a6", 2.0),
      Aruco_(30, Orientation.WEST , "a6",-0.5)
    ]
    list_completa = [
      Aruco_( 1, Orientation.SOUTH,"c15", 1.5),
      Aruco_( 2, Orientation.SOUTH,"c14",-2.5, doble=True),
      Aruco_( 1, Orientation.EAST , "c0", 1.5),
      Aruco_( 3, Orientation.SOUTH,"c12", 0.0, doble=True),
      Aruco_( 4, Orientation.SOUTH,"c12",-1.5),
      Aruco_( 4, Orientation.WEST ,"c11", 1.5),
      Aruco_( 5, Orientation.WEST , "a7", 2.0, doble=True),
      Aruco_( 6, Orientation.WEST , "c9",-1.5, doble=True),
      Aruco_( 7, Orientation.NORTH, "c7",-1.5),
      Aruco_( 8, Orientation.SOUTH, "d7",-2.0, doble=True),
      Aruco_( 9, Orientation.WEST , "d2", 2.0, doble=True),
      Aruco_(10, Orientation.SOUTH, "d8", 2.0, doble=True),
      Aruco_(11, Orientation.EAST , "d4", 2.0, doble=True),
      Aruco_(12, Orientation.EAST , "d1",-1.5, doble=True),
      Aruco_(13, Orientation.EAST , "d1", 0.0, doble=True),
      Aruco_(14, Orientation.NORTH, "d8", 2.0, doble=True),
      Aruco_(15, Orientation.EAST , "d1", 2.0),
      Aruco_(15, Orientation.SOUTH, "b3", 2.0),
      Aruco_(16, Orientation.WEST , "d9",-2.0, doble=True),
      Aruco_(17, Orientation.NORTH, "c5",-2.5),
      Aruco_(17, Orientation.WEST , "d4",-1.5),
      Aruco_(18, Orientation.NORTH, "c4",-2.5, doble=True),
      Aruco_(19, Orientation.NORTH, "c4", 1.5),
      Aruco_(19, Orientation.EAST , "c3",-1.5),
      Aruco_(20, Orientation.EAST , "c2", 2.0, doble=True),
      Aruco_(21, Orientation.SOUTH, "b2", 2.0),
      Aruco_(21, Orientation.EAST,  "c1", 2.0),
      Aruco_(22, Orientation.SOUTH, "b2",-2.0),
      Aruco_(22, Orientation.WEST,  "b1", 2.0),
      Aruco_(23, Orientation.WEST , "d1", 2.0, doble=True),
      Aruco_(24, Orientation.WEST , "d1",-1.5, doble=True),
      Aruco_(25, Orientation.WEST , "a1",-2.0, doble=True),
      Aruco_(26, Orientation.EAST , "b1", 2.0, doble=True),
      Aruco_(27, Orientation.SOUTH, "b4",-1.0, doble=True),
      Aruco_(28, Orientation.NORTH, "b2", 1.5),
      Aruco_(28, Orientation.EAST , "c0",-1.5),
      Aruco_(29, Orientation.SOUTH, "b6",-1.0),
      Aruco_(29, Orientation.WEST,  "a6", 2.0),
      Aruco_(30, Orientation.WEST , "a6",-0.5, doble=True)
    ]


    main(list_completa)