>[!NOTE]
> Highlights information that users should take into account, even when skimming.

>[!IMPORTANT]
> Crucial information necessary for users to succeed.

>[!WARNING]
> Critical content demanding immediate user attention due to potential risks.

```
gedit ~/.bashrc
```
Cole ao final do arquivo:
```
######################################
########       MY ROS2        ########
######################################
source /opt/ros/iron/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
export ROS_DISTRO=iron

######################################
########      MY GAZEBO       ########
######################################
export GZ_VERSION=harmonic
export GZ_SIM_RESOURCE_PATH=$HOME/tcc/robot/ros2_ws/src/my_package/models:$HOME/tcc/robot/harm_ws/src/harm/models
```
>[!IMPORTANT]
> Obs.: Caso tenha mais de um path de models separe com `:`




https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html
https://gazebosim.org/docs/harmonic/install_ubuntu
sudo apt-get install ros-${ROS_DISTRO}-ros-gz

```
sudo apt install python3-colcon-common-extensions
sudo apt install python3-pip
pip3 install setuptools==58.2.0
pip install numpy==1.24.3
pip install scipy

```

