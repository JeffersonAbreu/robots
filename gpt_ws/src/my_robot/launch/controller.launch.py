import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller',
            executable='my_controller',
            name='my_controller',
            output='screen',
        ),
    ])
