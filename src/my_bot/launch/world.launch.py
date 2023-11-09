import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from utils import Bridge

def generate_launch_description():
    # Configure ROS nodes for launch
    pkg_share_dir = get_package_share_directory('my_bot')
    models_path = pkg_share_dir + "/models"
    worlds_path = pkg_share_dir + "/worlds"

    os.environ['GZ_SIM_RESOURCE_PATH'] = models_path + ":" + worlds_path
    #os.environ['QT_QPA_PLATFORM'] = 'wayland'
    # Setup to launch the simulator and Gazebo world

    gz_sim = ExecuteProcess(cmd=['gz', 'sim', 'world.sdf', '-r'], output='screen')
    '''
    gz_sim = ExecuteProcess(cmd=['gz', 'sim', '-v', '8','world.sdf', '-r'], output='screen')
    gz sim <path to>/fuel_preview.sdf --gui-config <path to>saved.config

    controller_process = ExecuteProcess(
        cmd=['ros2', 'run', 'my_bot', 'collision_avoidance'],
        output='screen'
    )ros2 run rqt_image_view rqt_image_view /camera
    rqt_image_view = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view', '/camera'],
        output='screen'
    )https://automaticaddison.com/how-to-simulate-a-robot-using-gazebo-and-ros-2/
    '''

    # Bridge ROS topics and Gazebo messages for establishing communication
    '''
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image'
        ],
        parameters=[{'use_sim_time': True}],
        output='both' #both
    )

    bridge = Bridge()
    bridge.topic_twist('/cmd_vel')
    bridge.topic_laser_scan('/lidar')
    bridge.topic_odometry('/odom')
    bridge.topic_imu('/imu')
    bridge.topic_camera('/camera')

    bridge = bridge.create_node()
    '''
    return LaunchDescription([
        gz_sim,
        #bridge,
        #controller_process
        #rqt_image_view
    ])
