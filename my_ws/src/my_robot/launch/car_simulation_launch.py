from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_file',
                               default_value=os.path.join(
                                   get_package_share_directory('my_robot'), 'models', 'car.sdf'),
                               description='Path to the car model SDF file'),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                LaunchDescription([
                    LogInfo(
                        condition=IfCondition(LaunchConfiguration('verbose')),
                        msg='Launching the car model...'
                    ),
                    Node(
                        package='gazebo_ros',
                        executable='spawn_entity.py',
                        namespace='',
                        name='spawn_entity',
                        output='screen',
                        parameters=[{'verbose': LaunchConfiguration('verbose')}],
                        arguments=['-entity', 'car', '-topic', 'model', '-file', LaunchConfiguration('model_file')],
                    ),
                    # Adicione um nó para iniciar o sensor LiDAR
                    Node(
                        package='gazebo_ros',
                        executable='spawn_entity.py',
                        namespace='',
                        name='spawn_lidar',
                        output='screen',
                        arguments=['-entity', 'lidar', '-topic', 'lidar', '-file', 'path_to_lidar_model.sdf'],
                    ),
                    # Adicione um nó para iniciar a câmera
                    Node(
                        package='gazebo_ros',
                        executable='spawn_entity.py',
                        namespace='',
                        name='spawn_camera',
                        output='screen',
                        arguments=['-entity', 'camera', '-topic', 'camera', '-file', 'path_to_camera_model.sdf'],
                    ),
                ]),
                # Add more nodes as needed
            ]),
        ),
    ])

