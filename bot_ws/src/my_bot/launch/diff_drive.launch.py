# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Configure ROS nodes for launch
    pkg_share_dir = get_package_share_directory('my_bot')
    models_path = pkg_share_dir + "/models"
    worlds_path = pkg_share_dir + "/worlds"
    os.environ['GZ_SIM_RESOURCE_PATH'] = models_path + ":" + worlds_path

    # Setup project paths
    #pkg_project_bringup = get_package_share_directory('ros_gz_example_bringup')
    pkg_project_gazebo = get_package_share_directory('my_bot')
    pkg_project_description = get_package_share_directory('my_bot')
    pkg_ros_gz_sim = get_package_share_directory('my_bot')

    # Load the SDF file from "description" package
    sdf_file  =  os.path.join(pkg_project_description, 'models', 'car_bot', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Setup to launch the simulator and Gazebo world
    gz_sim = ExecuteProcess(cmd=['gz', 'sim', 'world.sdf'], output='screen')

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    '''
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )
    
    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_description, 'config', 'diff_drive.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )
    '''
    
    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )


    '''
    controller_process = ExecuteProcess(
        cmd=['ros2', 'run', 'my_bot', 'collision_avoidance'],
        output='screen'
    )
    '''
        
    return LaunchDescription([
        gz_sim,
        #DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),
        bridge,
        #robot_state_publisher,
        #rviz
        #controller_process
    ])
