#!/usr/bin/env python3
from launch_ros.actions import Node

#https://gazebosim.org/docs/harmonic/ros2_integration
#The ROS message type is followed by an @, [, or ] symbol where:

class Bridge:
    def __init__(self):
       self.BIDIRECIONAL = '@'
       self.GZ_TO_ROS = '['
       self.ROS_TO_GZ = ']'
       self.arguments = []


    def __add_ros_gz_bridge(self, topic, ros_type, direction, gz_type) -> None:
      param = f'{topic}@{ros_type}{direction}{gz_type}'
      self.arguments.append(param)

    
    def create_node(self):
        return Node(
           package='ros_gz_bridge',
           executable='parameter_bridge',
           arguments=self.arguments,
           parameters=[{'use_sim_time': True}],
           # tipos de saida
           # 'log': Redireciona a saída para um arquivo de log. Esse arquivo geralmente é encontrado no diretório de log do ROS 2, que é tipicamente ~/.ros/log/.
           # 'screen': Exibe a saída diretamente no terminal ou console de onde o lançamento foi iniciado.
           # None: Suprime a saída, de modo que nada é mostrado no terminal e nenhum arquivo de log é criado.
           # 'both': Combina 'log' e 'screen'. A saída é mostrada no terminal e também é gravada em um arquivo de log.
           output='screen')


    def topic_twist(self, topic):
      ros_type = "geometry_msgs/msg/Twist"
      gz_type = "gz.msgs.Twist"
      direction = self.BIDIRECIONAL
      self.__add_ros_gz_bridge(topic, ros_type, direction, gz_type)


    def topic_laser_scan(self, topic):
      ros_type = "sensor_msgs/msg/LaserScan"
      gz_type = "gz.msgs.LaserScan"
      direction = self.GZ_TO_ROS
      self.__add_ros_gz_bridge(topic, ros_type, direction, gz_type)


    def topic_odometry(self, topic):
      ros_type = "nav_msgs/msg/Odometry"
      gz_type = "gz.msgs.Odometry"
      direction = self.GZ_TO_ROS
      self.__add_ros_gz_bridge(topic, ros_type, direction, gz_type)

    
    def topic_imu(self, topic):
      ros_type = "sensor_msgs/msg/Imu"
      gz_type = "gz.msgs.IMU"
      direction = self.GZ_TO_ROS
      self.__add_ros_gz_bridge(topic, ros_type, direction, gz_type)
