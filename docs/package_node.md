### 1. Criando o Pacote:
Dentro diretório `src` do seu workspace ROS 2.
Crie o pacote usando o comando `ros2 pkg create`. Como você quer um pacote Python, use o tipo de construção `ament_python`:

```bash
ros2 pkg create --build-type ament_python --node-name controller my_package
```

O comando `ros2 pkg create --build-type ament_python --node-name controller my_package`, o ROS 2 irá criar automaticamente um pacote Python chamado `my_package` e também gerará um nó Python chamado `controller`.

Aqui está o que acontece:

### Estrutura de Diretórios:

Após executar o comando, a estrutura de diretórios será algo assim:

```
my_package/
│
├── my_package/
│   ├── __init__.py
│   └── controller.py
|
├── resource/
│
├── test/
│   └── test_copyright.py
│
├── package.xml
├── setup.cfg
└── setup.py
```

### Conteúdo do Nó `controller`:

O arquivo `controller.py` será preenchido com um esqueleto básico para um nó ROS 2, atualize com:

```python
import rclpy
from rclpy.node import Node

class Controller(Node):

    def __init__(self):
        super().__init__('controller')
        # Seu código de inicialização aqui
        self.get_logger().info('Hello world: my_package')

    def listener_callback(self, msg):
        # Callback para seu subscriber
        pass

def main(args=None):
    rclpy.init(args=args)

    controller_node = Controller()

    rclpy.spin(controller_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Executando o Nó:

Depois de criar o pacote e o nó usando o comando acima, você pode compilar e executar o nó da mesma maneira que mencionado anteriormente:

1. Compile o workspace:

```bash
cd ~/ros2_ws
colcon build
```

2. Carregue o ambiente:

```bash
source install/setup.bash
```

3. Execute o nó:

```bash
ros2 run my_package controller
```

Ao executar o nó, você verá a mensagem "Hello world: my_package" no terminal, indicando que o nó está funcionando corretamente.


[Criando uma launch](launch.md)