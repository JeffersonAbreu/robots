from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ignition_gazebo_ros',
            executable='ignition_gazebo',
            name='ignition_gazebo',
            output='screen',
            prefix=['/usr/bin/ign gazebo'],
            parameters=[{'ign_args': '-r ~/tcc/robot/my_ws/src/my_robot/worlds/my_world.sdf'}], # Substitua com o caminho correto para o seu arquivo SDF
        ),
    ])
