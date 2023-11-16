#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    GZ_WORLD = "mini.world"
    # Set gazebo environment variables
    pkg_share_dir = get_package_share_directory('my_bot')
    models_path = pkg_share_dir + "/models"
    worlds_path = pkg_share_dir + "/worlds"
    os.environ['GZ_SIM_RESOURCE_PATH'] = models_path + ":" + worlds_path
    
    # Execute o Gazebo com o arquivo de mundo
    gazebo_process = ExecuteProcess(cmd=['gz', 'sim', GZ_WORLD, '-r'], output='screen')

    bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gazebo_bridge_camera',
            arguments=[
                '/camera@sensor_msgs/msg/Image[gz.msgs.Image'
            ],
            parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        gazebo_process,
                # Cria uma ponte para o tópico /camera
        bridge
    ])