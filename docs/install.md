[!NOTE]
Highlights information that users should take into account, even when skimming.

[!IMPORTANT]
Crucial information necessary for users to succeed.

[!WARNING]
Critical content demanding immediate user attention due to potential risks.

```sh
gedit ~/.bashrc
```
Cole ao final do arquivo:
```sh
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
> [!IMPORTANT]
> Obs.: Caso tenha mais de um path de models separe com `:`   



### Visite e instale o ROS2 IRON
VIsite Link: [docs.ros.org](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html)

### install gazebo harmonic

**Primeiro instale algumas ferramentas necessárias:**
```sh
sudo apt-get update
sudo apt-get install lsb-release wget gnupg
```   


**Em seguida, instale o Gazebo Harmonic:**
```sh
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
```
Esses comando foram extraidos de: [gazebosim.org](https://gazebosim.org/docs/harmonic/install_ubuntu)   




### Para integrar o ROS2 ao GAZEBO
```sh
sudo apt-get install ros-${ROS_DISTRO}-ros-gz${GZ_VERSION}
```


**__Pronto, agora instale estes pacotes para autocompletar a digitação com `TAB`__**
```sh
sudo apt install python3-colcon-common-extensions
```

Outras recomendações caso precise:
```sh
sudo apt install python3-pip
pip3 install setuptools==58.2.0
```
### Para verificar as versões dos pacotes instalados, especialmente em um sistema baseado em Ubuntu com ROS Iron, você pode usar os seguintes comandos no terminal:

Para verificar a versão do cv_bridge:
```sh
apt list --installed | grep cv-bridge
```

Para verificar a versão do OpenCV:
```sh
python3 -c "import cv2; print(cv2.__version__)"
```
Este último comando executa uma linha de código Python que importa o OpenCV e imprime a versão instalada. Esses comandos fornecerão as versões exatas dos pacotes instalados no seu sistema.





## Obs:
Esse `readme.md`, foi feito com os exemplos de [markdown.net.br](https://markdown.net.br/sintaxe-basica/)