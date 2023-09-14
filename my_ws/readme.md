Para criar pacotes no ROS 2 com Ignition Gazebo (também conhecido como Ignition Robotics), você pode seguir os passos abaixo. Certifique-se de que você já tenha o ROS 2 e o Ignition Gazebo instalados e configurados no seu sistema antes de começar.

**1. Crie um espaço de trabalho do ROS 2** (se você ainda não tiver um):

Primeiro, crie um espaço de trabalho do ROS 2 para organizar seus pacotes. Substitua `my_ws` pelo nome que você deseja dar ao seu espaço de trabalho.

```bash
mkdir -p ~/my_ws/src
cd ~/my_ws/src
```

**2. Crie um pacote do ROS 2 com Ignition Gazebo:**

Para criar um pacote com suporte ao Ignition Gazebo, você pode usar a ferramenta `ros2 pkg create` com a opção `--build-type ament_cmake` para criar um pacote com o sistema de compilação CMake do ROS 2. Substitua `my_package` pelo nome que você deseja dar ao seu pacote.

```bash
ros2 pkg create --build-type ament_cmake my_package
```

**3. Configure o pacote para usar o Ignition Gazebo:**

Dentro do diretório do seu pacote (`~/my_ws/src/my_package`), você precisará configurar seu pacote para usar o Ignition Gazebo em vez do Gazebo padrão. Para fazer isso, crie um arquivo `ament.ignore` com o seguinte conteúdo:

```plaintext
# Informa ao ROS 2 que estamos usando Ignition Gazebo
gazebo_ros_pkgs
```

Este arquivo informa ao sistema de compilação do ROS 2 (Ament) para ignorar qualquer dependência relacionada ao pacote `gazebo_ros_pkgs`, que é usado com o Gazebo padrão. Em vez disso, o Ignition Gazebo será usado.

**4. Crie um arquivo SDF para o seu modelo:**

Crie um arquivo `.sdf` que descreve o modelo que você deseja simular no Ignition Gazebo. Coloque esse arquivo em algum lugar dentro do seu pacote, por exemplo, em `~/my_ws/src/my_package/models/my_model/my_model.sdf`.

**5. Crie um arquivo de lançamento para iniciar o Ignition Gazebo com seu modelo:**

Crie um arquivo de lançamento (por exemplo, `my_simulation.launch.py`) dentro do diretório `launch` do seu pacote. Este arquivo será usado para iniciar o Ignition Gazebo com o seu modelo. Certifique-se de que o arquivo de lançamento especifique o caminho correto para o arquivo `.sdf` que você criou.

**6. Configure o ambiente de simulação:**

Certifique-se de configurar o ambiente de simulação corretamente, definindo variáveis de ambiente como `GAZEBO_MODEL_PATH` para apontar para o diretório onde seus modelos estão localizados. Você também pode definir outras variáveis de ambiente relevantes, como `GAZEBO_RESOURCE_PATH`, conforme necessário.

**7. Execute a simulação:**

Agora, você pode executar a simulação usando o arquivo de lançamento que você criou:

```bash
ros2 launch my_package my_simulation.launch.py
```

Certifique-se de substituir `my_package` pelo nome do seu pacote e `my_simulation.launch.py` pelo nome do seu arquivo de lançamento.

Com esses passos, você criou um pacote no ROS 2 com suporte ao Ignition Gazebo e configurou a simulação do seu modelo. Certifique-se de ajustar o arquivo SDF e o arquivo de lançamento de acordo com as especificações do seu modelo e as necessidades do seu projeto.

export GAZEBO_MODEL_PATH=~/my_ws/src/my_ignition_gazebo_package/models:$GAZEBO_MODEL_PATH
export GAZEBO_RESOURCE_PATH=~/my_ws/src/my_ignition_gazebo_package/worlds:$GAZEBO_RESOURCE_PATH

ros2 pkg create --build-type ament_python my_car_control_package

Para rodar o projeto completo, incluindo o Ignition Gazebo e o nó de controle do teclado, siga estas etapas:

**1. Abra um terminal e navegue para o diretório do seu espaço de trabalho do ROS 2:**

```bash
cd ~/my_ws
```

**2. Compile o pacote do nó de controle do teclado:**

Certifique-se de que o seu pacote `my_car_control_package` esteja compilado. Use o seguinte comando:

```bash
colcon build --packages-select my_car_control_package
```

Isso compilará apenas o pacote `my_car_control_package`.

**3. Configure o ambiente do ROS 2:**

Se você ainda não configurou o ambiente do ROS 2 neste terminal, execute o seguinte comando:

```bash
source /opt/ros/foxy/setup.bash
```

Substitua `/opt/ros/foxy/setup.bash` pelo caminho correto para o seu ambiente ROS 2, se necessário.

**4. Execute o Ignition Gazebo com o mundo e modelo do seu carro:**

Certifique-se de ter um arquivo SDF para o mundo e o modelo do seu carro, como mencionado anteriormente. Em seguida, execute o Ignition Gazebo com o seguinte comando:

```bash
ros2 launch my_ignition_gazebo_package my_simulation.launch.py
```

Substitua `my_ignition_gazebo_package` e `my_simulation.launch.py` pelos nomes do seu pacote e arquivo de lançamento.

**5. Execute o nó de controle do teclado:**

Abra um novo terminal e navegue para o diretório do seu espaço de trabalho do ROS 2 novamente:

```bash
cd ~/my_ws
```

Agora, execute o nó de controle do teclado com o seguinte comando:

```bash
ros2 run my_car_control_package keyboard_control_node.py
```

Isso iniciará o nó de controle do teclado, que deve estar ouvindo as teclas pressionadas e publicando comandos de velocidade no tópico `/cmd_vel`.

Com esses passos, você deve ter o Ignition Gazebo rodando com seu modelo de carro e o nó de controle do teclado em execução. Você pode usar as teclas "w", "s", "a" e "d" no terminal em que você iniciou o nó de controle do teclado para controlar o movimento do carro na simulação. Certifique-se de que os nomes dos pacotes e arquivos correspondam aos nomes reais do seu projeto.
