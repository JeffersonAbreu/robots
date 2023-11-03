import os
import shutil
import cv2.aruco as aruco
import cv2
from xml.dom.minidom import parse
from constants import ARUCO_DICT, MARKER_SIZE, SCALE

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
        name_image = make_aruco(path_destiny, key, name)
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
    model_noversion_sdf_path = os.path.join(path_destiny, "model.sdf")
    dom = parse(model_noversion_sdf_path)
    for node in dom.getElementsByTagName('model'):
        node.attributes["name"].value = name
        break

    for node in dom.getElementsByTagName('mesh'):
        '''
        for child in node.childNodes:
            if child.nodeName == "uri":
                child.firstChild.nodeValue = "model://" + os.path.join('ar_tags', name, "tag.dae")
                break
        '''

        scale = dom.createElement("scale")
        values = dom.createTextNode("{} {} {}".format(SCALE, SCALE, SCALE))
        scale.appendChild(values)
        node.appendChild(scale)

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

def make_aruco(destiny, num:int, name):
    marker_image = aruco.generateImageMarker(ARUCO_DICT, num, MARKER_SIZE)
    #cv2.imshow("img", marker_image)
    name_image = f"aruco_{name}.png"
    path_and_file = os.path.join(destiny, name_image)
    cv2.imwrite(path_and_file, marker_image)
    return name_image