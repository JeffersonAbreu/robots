import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('my_camera_calibration')
    file = os.path.join(pkg_share_dir, 'my_camera_calibration', 'camera_calibrator.py')
    # Execute o Gazebo com o arquivo de mundo
    gazebo_process = ExecuteProcess(cmd=['python3', file], output='screen')

    return LaunchDescription([
        gazebo_process,
    ])