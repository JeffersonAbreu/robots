import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    current_dir = os.path.dirname(__file__)
    world_file_path = os.path.join(current_dir, '..', 'worlds', 'mapa3.sdf')

# Definindo a ação para o controlador do robô
    robot_controller_action = Node(
        package='harm',
        executable='controller',
        name='robot_controller',
        output='screen'
    )

    # Definindo a ação para lançar o Gazebo Harmonic
    gazebo_action = ExecuteProcess(
        cmd=['gz', 'sim', world_file_path],
        output='screen'
    )

    # Estabelece uma ponte entre os tópicos ROS 2 e Gazebo para o robo.
    # Esta ponte permite a comunicação bidirecional entre o ROS 2 e o Gazebo, permitindo que comandos sejam enviados
    # para o o robo e que a odometria seja recebida de volta. A confiabilidade QoS é configurada como "reliable"
    # para garantir a entrega de todas as mensagens.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                   '/model/vehicle_blue/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
        parameters=[{'qos_overrides./model/vehicle_blue.subscriber.reliability': 'reliable'}],
        output='screen'
    )

    # Retornando a descrição de lançamento com as ações definidas
    return LaunchDescription([
        robot_controller_action,
        gazebo_action,
        bridge
    ])
