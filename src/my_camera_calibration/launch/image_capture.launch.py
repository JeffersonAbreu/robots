import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Registra o evento de shutdown
    return LaunchDescription([
        # Carrega o Node de calibração da câmera
        Node(
            package='my_camera_calibration',
            executable='capture_images',
            name='capture_images',
            output='screen'
        ),
    ])
