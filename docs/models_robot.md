
#### 4. Criação dos Modelos do Robô:

Dentro da pasta `models`, crie subpastas para cada componente do robô: `base_link`, `caster_wheel`, `left_wheel`, e `right_wheel`.

##### 4.1. Modelo do Corpo do Robô (`base_link`):

Dentro de `models/base_link/`, crie:

`model.sdf`:
```xml
<sdf version="1.6">
  <model name="base_link">
    <link name="link">
      <visual name="visual">
        <geometry>
          <box>
            <size>0.5 0.3 0.2</size>
          </box>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
```

`model.config`:
```xml
<?xml version="1.0"?>
<model>
  <name>Base Link</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>Your Name</name>
    <email>your.email@example.com</email>
  </author>
  <description>Base link for the two-wheeled robot.</description>
</model>
```

##### 4.2. Modelo da Roda Esquerda (`left_wheel`):

Dentro de `models/left_wheel/`, crie:

`model.sdf`:
```xml
<sdf version="1.6">
  <model name="left_wheel">
    <link name="link">
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
```

`model.config`:
```xml
<?xml version="1.0"?>
<model>
  <name>Left Wheel</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>Your Name</name>
    <email>your.email@example.com</email>
  </author>
  <description>Left wheel for the two-wheeled robot.</description>
</model>
```

##### 4.3. Modelo da Roda Direita (`right_wheel`):

Dentro de `models/right_wheel/`, crie:

`model.sdf`:
```xml
<sdf version="1.6">
  <model name="right_wheel">
    <link name="link">
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
```

`model.config`:
```xml
<?xml version="1.0"?>
<model>
  <name>Right Wheel</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>Your Name</name>
    <email>your.email@example.com</email>
  </author>
  <description>Right wheel for the two-wheeled robot.</description>
</model>
```

#### 5. Robô Completo:

Dentro da pasta `models`, crie uma subpasta chamada `two_wheeled_robot`. Dentro dela, crie o arquivo `model.sdf`:

```xml
<sdf version="1.6">
  <model name="two_wheeled_robot">
    <!-- Base do robô -->
    <include>
      <uri>model://base_link</uri>
    </include>

    <!-- Roda esquerda -->
    <include>
      <uri>model://left_wheel</uri>
      <pose>0.1 0.15 0 0 0 0</pose>
    </include>
    <joint name="left_wheel_joint" type="revolute">
      <parent>base_link::link</parent>
      <child>left_wheel::link</child>
      <axis>
        <xyz>0 1 0</xyz>
      </axis>
    </joint>

    <!-- Roda direita -->
    <include>
      <uri>model://right_wheel</uri>
      <pose>-0.1 0.15 0 0 0 0</pose>
    </include>
    <joint name="right_wheel_joint" type="revolute">
      <parent>base_link::link</parent>
      <child>right_wheel::link</child>
      <axis>
        <xyz>0 1 0</xyz>
      </axis>
    </joint>
  </model>
</sdf>
```

#### 6. Atualização do `setup.py`:

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

#### 7. Compilação:

Compile o workspace:

```bash
cd ~/ros2_ws
colcon build
```

#### 8. Execução:

Após a compilação, carregue o ambiente:

```bash
source install/setup.bash
```

Execute o arquivo de lançamento:

```bash
ros2 launch my_package main.launch.py
```

Isso deve lançar o Gazebo com o mundo e o robô de duas rodas que você definiu.

Entendido! Vamos criar um modelo separado para a roda caster e depois incluí-lo no `robot.sdf` usando a tag `<include>`.

### 1. Criação do Modelo da Roda Caster

Dentro da pasta `models`, crie uma subpasta chamada `caster_wheel`:

```
my_package/
│
├── models/
│   ├── caster_wheel/
│   │   ├── model.config
│   │   └── model.sdf
│
└── ...
```

**model.config** (dentro de `caster_wheel`):

```xml
<?xml version="1.0"?>
<model>
  <name>caster_wheel</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
  <author>
    <name>Your Name</name>
    <email>your.email@example.com</email>
  </author>
  <description>A simple caster wheel.</description>
</model>
```

**model.sdf** (dentro de `caster_wheel`):

```xml
<?xml version='1.0'?>
<sdf version='1.7'>
  <model name='caster_wheel'>
    <link name='link'>
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.001</iyy>
          <iyz>0</iyz>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <geometry>
          <sphere>
            <radius>0.05</radius>
          </sphere>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <sphere>
            <radius>0.05</radius>
          </sphere>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
```

### 2. Incluindo a Roda Caster no `robot.sdf`

No seu arquivo `robot.sdf`, você pode incluir o modelo da roda caster usando a tag `<include>`:

```xml
<?xml version='1.0'?>
<sdf version='1.7'>
  <model name='two_wheeled_robot'>
    ...
    <!-- Incluindo a roda caster -->
    <include>
      <uri>model://caster_wheel</uri>
      <pose>0 0 -0.2 0 0 0</pose>
      <name>caster</name>
    </include>
    ...
  </model>
</sdf>
```

