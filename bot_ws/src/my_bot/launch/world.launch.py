import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from utils import Bridge

def generate_launch_description():
    # Configure ROS nodes for launch
    pkg_share_dir = get_package_share_directory('my_bot')
    models_path = pkg_share_dir + "/models"
    worlds_path = pkg_share_dir + "/worlds"

    os.environ['GZ_SIM_RESOURCE_PATH'] = models_path + ":" + worlds_path

    # Setup to launch the simulator and Gazebo world
    gz_sim = ExecuteProcess(cmd=['gz', 'sim', 'world.sdf', '-r'], output='screen')
    '''
    controller_process = ExecuteProcess(
        cmd=['ros2', 'run', 'my_bot', 'collision_avoidance'],
        output='screen'
    )ros2 run rqt_image_view rqt_image_view /camera
    '''
    rqt_image_view = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view', '/camera'],
        output='screen'
    )
    # Bridge ROS topics and Gazebo messages for establishing communication
     
    bridge = Bridge()
    bridge.topic_twist('/cmd_vel')
    bridge.topic_laser_scan('/lidar')
    bridge.topic_odometry('/odom')
    bridge.topic_imu('/imu')
    bridge.topic_camera('/camera')

    bridge = bridge.create_node()
        
    return LaunchDescription([
        gz_sim,
        bridge,
        #controller_process
        #rqt_image_view
    ])
