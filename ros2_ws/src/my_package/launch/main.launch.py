from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Caminho para o arquivo do mundo
    world_file_path = os.path.join(get_package_share_directory('my_package'), 'worlds', 'my_world.world')

    return LaunchDescription([
        # Iniciar o Ignition Fortress com o arquivo do mundo
        ExecuteProcess(
            cmd=['gz', 'sim', '--verbose', world_file_path],
            output='screen'
        ),
        
        # Iniciar o nó do controlador
        Node(
            package='my_package',
            executable='controller',
            name='controller_node',
            output='screen'
        )
    ])
