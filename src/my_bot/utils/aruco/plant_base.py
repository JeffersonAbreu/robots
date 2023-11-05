from xml.dom.minidom import parse
import xml.etree.ElementTree as ET

from tag import Tag

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

