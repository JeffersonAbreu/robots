import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot')
    _keyboard = os.path.join(pkg_dir,'launch','keyboard.launch.py')
    _controller = os.path.join(pkg_dir,'launch','keyboard.launch.py')
    _collision = os.path.join(pkg_dir,'launch', 'collision.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_keyboard_control',
            default_value='true',
            description='Whether to use keyboard control for the robot'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            name='ros_gz_sim',
            output='screen',
            arguments=['worlds/world.world']
        ),
"""
        IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(_keyboard),
            condition=IfCondition(LaunchConfiguration('use_keyboard_control'))
        ),

        IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(_controller)
        ),

        IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(_collision)
        ),
"""
    ])
