import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='collision',
            executable='my_collision',
            name='my_collision',
            output='screen',
        ),
    ])
