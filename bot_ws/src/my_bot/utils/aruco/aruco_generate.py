#! /usr/bin/env python
import argparse
from xml.dom.minidom import parse
import cv2
import cv2.aruco as aruco
from xml.dom.minidom import parse
import sys
import os
import shutil
from enum import Enum, auto
import math

#sys.setrecursionlimit(2000) 
marker_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
MARKER_SIZE = 500
SCALE = 0.5


class Orientation(Enum):
    NORTH = auto()
    SOUTH = auto()
    EAST = auto()
    WEST = auto()

def name_img(num: int) -> str:
   return 'tag_aruco_' + format_number(num) +'.png'

def format_number(num) -> str:
    """Formata o número para ter dois dígitos, preenchendo com zero se necessário."""
    return f"{num:02}"

def create_diretory_ar_tags(_path):
  if not os.path.isdir(_path):
      print(f"O diretório '{_path}' não existe. Criando...")
      os.makedirs(_path)

def remove_diretory_ar_tags(_path):
   if os.path.isdir(_path):
      print(f"O diretório '{_path}' existe. Removendo...")
      shutil.rmtree(_path)

def copy(origen, destiny):
   # Copie a pasta e todos os seus conteúdos
   if os.path.isdir(origen):
        shutil.copytree(origen, destiny)
   else:
        print(f"'{origen}' não é um diretório válido.")

def make_aruco(num:int, tamanho, destiny):
    marker_image = aruco.generateImageMarker(marker_dict, num, tamanho)
    #cv2.imshow("img", marker_image)
    name_image = name_img(num)
    path_and_file = os.path.join(destiny, name_image)
    cv2.imwrite(path_and_file, marker_image)
    return name_image

def modify_model_config(path_destiny, name):
    # modify model.config
    model_config_path = os.path.join(path_destiny, "model.config")
    dom = parse(model_config_path)
    for node in dom.getElementsByTagName('name'):
        node.firstChild.nodeValue = name
        print(node.firstChild.nodeValue)
        break
    f = open(model_config_path, 'w+')
    # Write the modified xml file
    f.write(dom.toxml())
    f.close()

def modify_model_sdf(path_destiny, name, pose_values):
    # modify model.sdf
    model_noversion_sdf_path = os.path.join(path_destiny, "model.sdf")
    dom = parse(model_noversion_sdf_path)
    for node in dom.getElementsByTagName('model'):
        node.attributes["name"].value = name
        break

    for node in dom.getElementsByTagName('link'):
        # add pose to link
        pose = dom.createElement("pose")
        values = dom.createTextNode("{} {} {} {} {} {}".format(*pose_values))
        pose.appendChild(values)
        node.appendChild(pose)
        break

    scaleModified = False
    for node in dom.getElementsByTagName('mesh'):
        for child in node.childNodes:
            if child.nodeName == "scale":
                child.firstChild.nodeValue = \
                    "{} {} {}".format(SCALE, SCALE, SCALE)
                scaleModified = True
            if child.nodeName == "uri":
                child.firstChild.nodeValue = "model://" + os.path.join('ar_tags', name, "tag.dae")
        if not scaleModified:
            pose = dom.createElement("scale")
            values = dom.createTextNode("{} {} {}".format(SCALE, SCALE, SCALE))
            pose.appendChild(values)
            node.appendChild(pose)

    f = open(model_noversion_sdf_path, 'w+')
    # Write the modified xml file
    f.write(dom.toxml())
    f.close()

def modify_tag_dae(path_destiny, image_name):
    dom = parse(os.path.join(path_destiny, "tag.dae"))
    for node in dom.getElementsByTagName('init_from'):
        node.firstChild.nodeValue = image_name
        break

    f = open(os.path.join(path_destiny, "tag.dae"), 'w+')
    # Write the modified xml file
    f.write(dom.toxml())
    f.close()

def get_pose(orientation):
    _x = 0
    _y = 0
    _z = 0.25
    _roll = 0
    _pitch = 0
    _yaw = 0

    if orientation == Orientation.SOUTH:
        _x = -0.05
        _yaw = math.pi
    elif orientation == Orientation.NORTH:
        _x = 0.05
    elif orientation == Orientation.WEST:
        _y = -0.05
        _yaw = (math.pi / 2) + math.pi
    elif orientation == Orientation.EAST:
        _y = 0.05
        _yaw = math.pi / 2


    return [_x, _y, _z, _roll, _pitch, _yaw]
def main():
  dir_origen = os.path.dirname(os.path.realpath(__file__))
  dir_model = os.path.join(dir_origen, 'model')
  caminho_base = os.path.join(dir_origen, '..', '..', 'models', 'ar_tags')
  diretorio_gazebo = os.path.relpath(caminho_base)
  print(diretorio_gazebo)

  remove_diretory_ar_tags(diretorio_gazebo)
  create_diretory_ar_tags(diretorio_gazebo)

  list = [
      [1, Orientation.SOUTH],
      [2, Orientation.WEST],
      [3, Orientation.EAST],
      [4, Orientation.NORTH]
  ]

  for key, orientation in list:
      name_dir = 'tag_'+ format_number(key)
      path_destiny = os.path.join(diretorio_gazebo, name_dir)
      copy(dir_model, path_destiny)
      name_image = make_aruco(key, MARKER_SIZE, path_destiny)
      modify_model_config(path_destiny, name_dir)
      modify_model_sdf(path_destiny, name_dir, get_pose(orientation))
      modify_tag_dae(path_destiny, name_image)

if __name__ == "__main__":
    main()