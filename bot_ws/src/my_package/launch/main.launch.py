from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Caminho para o arquivo do mundo
    path = get_package_share_directory('my_package')
    file = 'teste.sdf'
    #file = 'my_world.world'
    world_file_path = os.path.join(path, 'worlds', file)

    return LaunchDescription([
        # Iniciar o Gazebo com o arquivo do mundo
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', world_file_path],
            output='screen'
        ),
        
        # Iniciar o nó do controlador
        Node(
            package='my_package',
            executable='controller',
            name='robot_controller',
            output='screen'
        )
    ])
