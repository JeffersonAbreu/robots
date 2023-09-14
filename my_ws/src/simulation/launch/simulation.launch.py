# my_simulation.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            description='Escolha o mundo a ser carregado no Ignition Gazebo',
            default_value='~/tcc/robot/my_ws/src/my_car_simulation/worlds/my_world.sdf'
        ),
        Node(
            package='ignition_gazebo_ros',
            executable='ignition_gazebo',
            name='ignition_gazebo',
            namespace='my_car_simulation',
            output='screen',
            prefix=['/usr/bin/ign gazebo'],
            parameters=[{'ign_args': '-r $(arg world)'}]
        ),
    ])
