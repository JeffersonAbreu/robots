### Tutorial: Criando um Robô de Duas Rodas no ROS 2 e Gazebo
Vamos criar o diretório onde terá os senários e modelos

#### 1. Criar a Pasta:

Dentro do pacode `my_package`, crie as pastas `worlds, models`:

```bash
cd ~/ros2_ws/src/my_package
mkdir worlds models
```

#### 2. Criação do Mundo:

Dentro da pasta `worlds`, crie o arquivo `my_world.world`:

#### 3. Conteúdo do arquivo Mundo:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <!-- Um chão simples -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <!-- Uma luz solar -->
    <include>
      <uri>model://sun</uri>
    </include>
  </world>
</sdf>
```



## Vamos criar modelos simples para o sol (`sun`) e um plano (`ground_plane`).

### 1. Modelo do Sol (`sun`):

Dentro da pasta `models`, crie uma subpasta chamada `sun`. Dentro dela, crie os seguintes arquivos:

#### `model.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <light type="directional" name="sun">
    <cast_shadows>true</cast_shadows>
    <pose>0 0 10 0 0 0</pose>
    <diffuse>0.8 0.8 0.8 1</diffuse>
    <specular>0.2 0.2 0.2 1</specular>
    <attenuation>
      <range>1000</range>
      <constant>0.9</constant>
      <linear>0.01</linear>
      <quadratic>0.001</quadratic>
    </attenuation>
    <direction>-0.5 0.1 -0.9</direction>
  </light>
</sdf>
```

#### `model.config`:

```xml
<?xml version="1.0"?>
<model>
  <name>Sun</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>Your Name</name>
    <email>your.email@example.com</email>
  </author>
  <description>A simple sun model.</description>
</model>
```

### 2. Modelo do Plano (`ground_plane`):

Dentro da pasta `models`, crie uma subpasta chamada `ground_plane`. Dentro dela, crie os seguintes arquivos:

#### `model.sdf`:

```xml
<sdf version="1.6">
  <model name="ground_plane">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>50 50</size>
          </plane>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>50 50</size>
          </plane>
        </geometry>
        <material>
          <ambient>0.8 0.8 0.8 1</ambient>
          <diffuse>0.8 0.8 0.8 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
```

#### `model.config`:

```xml
<?xml version="1.0"?>
<model>
  <name>Ground Plane</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>Your Name</name>
    <email>your.email@example.com</email>
  </author>
  <description>A simple ground plane model.</description>
</model>
```


### 3. Atualização do `setup.py`:

Abra o arquivo `setup.py` no diretório `my_package` e adicione:

```python
data_files=[
    ('share/ament_index/resource

_index/packages',
            ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/main.launch.py']),
    ('share/' + package_name + '/world', ['world/my_world.world']),
    ('share/' + package_name + '/models', ['models/*'])
],
```
###  Variável ambiente ([Variável_Ambiene](variavel_ambiente.md))