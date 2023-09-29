#!/usr/bin/env python3
import os
import signal

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

#https://gazebosim.org/docs/harmonic/ros2_integration
#The ROS message type is followed by an @, [, or ] symbol where:
BIDIRECIONAL = '@'
GAZEBO_TO_ROS2 = '['
ROS2_TO_GAZEBO = ']'
# Lista para armazenar os PIDs dos processos
pids = []
# Settings
GZ_WORLD = "2world_my.world"

def ros_gz_bridge(topic, ros_type, gz_type, direction):
    param = f'{topic}@{ros_type}{direction}{gz_type}'
    cmd = ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', param]
    return ExecuteProcess(cmd=cmd, output='screen')

def ros_gz_twist_bridge(topic):
    ros_type = "geometry_msgs/msg/Twist"
    gz_type = "gz.msgs.Twist"
    direction = BIDIRECIONAL
    return ros_gz_bridge(topic, ros_type, gz_type, direction)

def ros_gz_image_bridge(topic):
    ros_type = "sensor_msgs/msg/Image"
    gz_type = "gz.msgs.Image"
    direction = GAZEBO_TO_ROS2
    return ros_gz_bridge(topic, ros_type, gz_type, direction)

def ros_gz_pose_bridge(topic):
    ros_type = "geometry_msgs/msg/PoseStamped"
    gz_type = "gz.msgs.Pose"
    direction = GAZEBO_TO_ROS2
    return ros_gz_bridge(topic, ros_type, gz_type, direction)



def generate_launch_description():
    # Set gazebo environment variables
    pkg_share_dir = get_package_share_directory('my_bot')
    models_path = pkg_share_dir + "/models"
    worlds_path = pkg_share_dir + "/worlds"
    os.environ['GZ_SIM_RESOURCE_PATH'] = models_path + ":" + worlds_path
    
    # Execute o Gazebo com o arquivo de mundo
    gazebo_process = ExecuteProcess(cmd=['gz', 'sim', GZ_WORLD], output='screen')
    
    # Gazebo -> ROS2 bridges
    # cam0_bridge = ros_gz_image_bridge("/gimbal/camera0")
    twist_bridge = ros_gz_twist_bridge("/cmd_vel")

    # Execute o controlador do robô
    controller_process = ExecuteProcess(
        cmd=['ros2', 'run', 'my_bot', 'controller'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_process,
        twist_bridge,
        controller_process
    ])