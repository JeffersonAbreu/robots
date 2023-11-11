>[!IMPORTANT]
> Importante seguir essa construção, se possível na ordem.

### Check list for construct project

[ x ] Modelar Robo:   
  - [ x ] Estrutura   
    - [ ] Sensores:   
    - [ ] Camera   
    - [ ] Bússola   
    - [ ] IMU   
    - [ ] LIDAR 1D   
  - [ ] Atuador ( Motor )   
  - [ ] Modelar ambiente:   
  - [ ] Criar o mapa   
  - [ ] Adicionar QRCode   
  - [ ] criar mapeamento para o robo   
  - [ ] Compreender Pub/Sub do ROS/Gazebo   
  - [ ] Controlar o robo através de um código Python   
  - [ ] Fazer leitura do QRCode via Python enquanto o robo navega   
  - [ ] Ler dados do robo e agir de acordo   

# o que vou fazer agora:
    - calibrar a camera no SensorCamera
        asas

[Exemplo para ver](https://automaticaddison.com/how-to-simulate-a-robot-using-gazebo-and-ros-2/)
[markdown tutorial](https://markdown.net.br/sintaxe-basica/#code)
```sh
ros2 run rqt_image_view rqt_image_view /rgbd_camera/image # vizualizar camera
. world.sh # inicia o gazebo - simulador 3D
. controller.sh # inicia o robo

```