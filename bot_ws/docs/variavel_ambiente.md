A variável de ambiente `GAZEBO_MODEL_PATH` informa ao Gazebo onde procurar modelos personalizados. 

Para adicionar o diretório `models` do seu pacote ao `GAZEBO_MODEL_PATH`, siga os passos abaixo:

### 1. Descubra o caminho completo para o diretório `models`:

Suponha que o caminho completo para o diretório `models` seja `/home/usuario/ros2_ws/src/my_package/models`.

### 2. Adicione este caminho à variável `GAZEBO_MODEL_PATH`:

#### Temporariamente (para a sessão atual do terminal):

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/usuario/ros2_ws/src/my_package/models
```

#### Permanentemente (para todas as futuras sessões):

Adicione a linha acima ao seu arquivo `.bashrc` (ou equivalente, dependendo do shell que você estiver usando):

```bash
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/usuario/ros2_ws/src/my_package/models" >> ~/.bashrc
```

Depois disso, recarregue o `.bashrc`:

```bash
source ~/.bashrc
```

Agora, sempre que você iniciar um novo terminal, o `GAZEBO_MODEL_PATH` incluirá o caminho para o diretório `models` do seu pacote, permitindo que o Gazebo encontre e carregue seus modelos personalizados.



#### 3. Compilação:

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