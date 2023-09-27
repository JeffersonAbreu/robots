    # Bridge
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                    '/lidar2@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan --ros-args -r /lidar2:=/laser_scan',
                    '/lidar2/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked --ros-args -r /lidar2/points:=/point_cloud2'],
        output='screen'
    )


    return LaunchDescription([
        ign_gazebo,
        # DeclareLaunchArgument('rviz', default_value='true',
        #                       description='Open RViz.'),
        
        
        rviz2,
        TimerAction(period=10.0,actions=[bridge])
    ])