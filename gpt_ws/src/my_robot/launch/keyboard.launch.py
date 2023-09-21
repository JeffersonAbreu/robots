import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='keyboard_controller',
            executable='keyboard_controller',
            name='keyboard_controller',
            output='screen',
        ),
    ])
