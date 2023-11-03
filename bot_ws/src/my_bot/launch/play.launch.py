#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from utils import Bridge

GZ_WORLD = "world.world"

def generate_launch_description():
    # Set gazebo environment variables
    pkg_share_dir = get_package_share_directory('my_bot')
    models_path = pkg_share_dir + "/models"
    worlds_path = pkg_share_dir + "/worlds"
    os.environ['GZ_SIM_RESOURCE_PATH'] = models_path + ":" + worlds_path
    
    # Execute o Gazebo com o arquivo de mundo
    gazebo_process = ExecuteProcess(cmd=['gz', 'sim', GZ_WORLD], output='screen')

    return LaunchDescription([
        gazebo_process,
    ])