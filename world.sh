colcon build --symlink-install --packages-select my_bot
source install/setup.bash
ros2 launch my_bot world.launch.py
