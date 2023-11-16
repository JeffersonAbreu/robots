from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='gazebo',
            arguments=['worlds/aruco_world.sdf'],
            output='screen'
        ),
        Node(
            package='my_aruco_package',
            executable='move_aruco',
            name='move_aruco',
            output='screen'
        ),
        Node(
            package='my_aruco_package',
            executable='capture_images',
            name='capture_images',
            output='screen'
        ),
    ])
