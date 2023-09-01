cp -r /home/ti/.gazebo/models/modelo_robot/*.STL /home/ti/.gazebo/models/modelo_robot/meshes/
sudo apt install ros-humble-joint-state-publisher-gui
cp -r /home/ti/.gazebo/models/modelo_robot/meshes/Base_castor_link.STL /home/ti/.gazebo/models/modelo_robot/meshes/base_link.STL /home/ti/.gazebo/models/modelo_robot/meshes/Rueda_castor_link.STL /home/ti/.gazebo/models/modelo_robot/meshes/Rueda_derecha_link.STL /home/ti/.gazebo/models/modelo_robot/meshes/Rueda_izquierda_link.STL /home/ti/.gazebo/models/modelo_robot/meshes/
ros2 launch modelo_robot display.launch.py
https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html
sudo apt install ros-humble-desktop-full
sudo apt -y install libignition-common-dev
sudo apt install -f ros-humble-gazebo-ros-pkgs
