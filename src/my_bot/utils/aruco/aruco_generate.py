#! /usr/bin/env python
from xml.dom.minidom import parse
import os

from tag_model import TagModel
from plant_base import PlantBase
from tag import Tag
from aruco_ import Aruco_, Orientation
from point import Point


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




def main(arucos: list[Aruco_]):
    dir_origen = os.path.dirname(os.path.realpath(__file__))
    dir_tag_model_origen = os.path.join(dir_origen, 'tag_model')
    dir_base = os.path.realpath(os.path.join(dir_origen, '..', '..', 'models'))
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

    for tag in [t.create_tag() for t in list_points]:
        maze.add_tag(tag)
        


     

if __name__ == "__main__":
    main([
      Aruco_( 1, Orientation.SOUTH,"c15",1.5),
      Aruco_( 2, Orientation.SOUTH,"c14", -2.5),
      #Aruco_( 1, Orientation.EAST , "c0", 1.5),
      Aruco_( 3, Orientation.SOUTH,"c12"),
      Aruco_( 4, Orientation.SOUTH,"c12", -1.5),
      #Aruco_( 4, Orientation.WEST ,"c11", 1.5),
      Aruco_( 5, Orientation.WEST , "a7", 2  ),
      Aruco_( 6, Orientation.WEST , "c9",-1.5),
      Aruco_( 7, Orientation.NORTH, "c7", -1.5),
      Aruco_( 8, Orientation.SOUTH, "d7", -2  ),
      Aruco_( 9, Orientation.WEST , "d2", 2  ),
      Aruco_(10, Orientation.SOUTH, "d8",2  ),
      Aruco_(11, Orientation.EAST , "d4", 2  ),
      Aruco_(12, Orientation.EAST , "d1",-1.5),
      Aruco_(13, Orientation.EAST , "d1"),
      Aruco_(14, Orientation.NORTH, "d8",2  ),
      Aruco_(15, Orientation.EAST , "d1", 2  ),
      Aruco_(16, Orientation.WEST , "d9",-2  ),
      Aruco_(17, Orientation.NORTH, "c5", -2.5),
      #Aruco_(17, Orientation.WEST , "d4",-1.5),
      Aruco_(18, Orientation.NORTH, "c4", -2.5),
      Aruco_(19, Orientation.NORTH, "c4",1.5),
      #Aruco_(19, Orientation.EAST , "c3",-1.5),
      Aruco_(20, Orientation.EAST , "c2", 2  ),
      Aruco_(21, Orientation.SOUTH, "b2",2  ),
      Aruco_(22, Orientation.SOUTH, "b2", -2  ),
      Aruco_(23, Orientation.WEST , "d1", 2  ),
      Aruco_(24, Orientation.WEST , "d1",-1.5),
      Aruco_(25, Orientation.WEST , "a1",-2  ),
      Aruco_(26, Orientation.EAST , "b1", 2  ),
      Aruco_(27, Orientation.SOUTH, "b4", -1  ),
      Aruco_(28, Orientation.NORTH, "b2",1.5),
      #Aruco_(28, Orientation.EAST , "c0",-1.5),
      Aruco_(29, Orientation.SOUTH, "b6", -1  ),
      Aruco_(30, Orientation.WEST , "a6",-0.5)
    ])

