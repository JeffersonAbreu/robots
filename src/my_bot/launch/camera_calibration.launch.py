#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

GZ_WORLD = "camera_calibration.world"

def generate_launch_description():
    # Set gazebo environment variables
    pkg_share_dir = get_package_share_directory('my_bot')
    models_path = pkg_share_dir + "/models"
    worlds_path = pkg_share_dir + "/worlds"
    os.environ['GZ_SIM_RESOURCE_PATH'] = models_path + ":" + worlds_path
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image'
        ],
        parameters=[{'use_sim_time': True}],
        output='both' #both
    )
    # Execute o Gazebo com o arquivo de mundo
    gazebo_process = ExecuteProcess(cmd=['gz', 'sim', GZ_WORLD, '-r'], output='screen')

    controller = ExecuteProcess(
        cmd=['ros2', 'run', 'my_bot', 'calibration'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_process,
        bridge,
        #controller
    ])