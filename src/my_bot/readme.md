>[!IMPORTANT]
> Importante seguir essa construção, se possível na ordem.

### Check list for construct project

1 - [x] Modelar Robo:
  [x] Estrutura
  [] Sensores:
    [] Camera
    [] Bússola
    [] IMU
    [] LIDAR 1D
  [] Atuador ( Motor )
1 - [] Modelar ambiente:
    [] Criar o mapa
    [] Adicionar QRCode
    [] criar mapeamento para o robo
1 - [] Compreender Pub/Sub do ROS/Gazebo
1 - [] Controlar o robo através de um código Python
1 - [] Fazer leitura do QRCode via Python enquanto o robo navega
1 - [] Ler dados do robo e agir de acordo



Config server para camera
`https://gazebosim.org/api/sim/8/server_config.html`

ros2 run rqt_image_view rqt_image_view /rgbd_camera/image