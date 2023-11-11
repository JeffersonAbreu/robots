import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    GZ_WORLD = "mini.world"
    world_file_path = get_package_share_directory('my_bot')
    models_path = os.path.join(world_file_path, 'models')
    worlds_path = os.path.join(world_file_path, 'worlds')
    os.environ['GZ_SIM_RESOURCE_PATH'] = models_path + ":" + worlds_path
    GZ_WORLD = os.path.join(world_file_path, 'worlds', GZ_WORLD)

    # Define o caminho onde o arquivo de calibração será salvo
    calibration_file_path = os.path.join(get_package_share_directory('my_camera_calibration'), 'config', 'calibration_data.yaml')

    # Registra o evento de shutdown
    return LaunchDescription([
        # Inicia o Gazebo com o mundo desejado
        ExecuteProcess(
            cmd=['gz', 'sim', GZ_WORLD, '-r'],
            output='screen'
        ),

        # Cria uma ponte para o tópico /camera
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gazebo_bridge_camera',
            arguments=[
                '/camera@sensor_msgs/msg/Image[gz.msgs.Image'
            ],
            parameters=[{'use_sim_time': True}],
        ),

        # Carrega o Node de calibração da câmera
        Node(
            package='my_camera_calibration',
            executable='camera_calibrator',
            name='camera_calibrator',
            output='screen',
            parameters=[
                {'camera_info_url': 'package://my_camera_calibration/config/camera_info.yaml'},
                {'calibration_file_path': calibration_file_path}  # Adiciona o caminho do arquivo de calibração como um parâmetro
            ]
        ),
    ])
