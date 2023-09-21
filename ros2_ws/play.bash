colcon build
source install/setup.bash
## export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/jeff/tcc/robot/ros2_ws/src/my_package/models
ros2 launch my_package main.launch.py
