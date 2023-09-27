import os
import signal
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Caminho para o arquivo de mundo
    current_dir = os.path.dirname(__file__)
    world_file_path = os.path.join(current_dir, '..', 'worlds', 'mapa3.sdf')

    # Lista para armazenar os PIDs dos processos
    pids = []

    def kill_processes(event, context):
        """Hook para matar processos quando a launch file for encerrada."""
        for pid in pids:
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

    # Crie o LaunchService e adicione o hook
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.register_on_shutdown(kill_processes)

    # Execute o Gazebo com o arquivo de mundo
    gazebo_process = ExecuteProcess(
        cmd=['gz', 'sim', world_file_path, '-r'],
        output='screen'
    )
    pids.append(gazebo_process.pid)

    # Execute o controlador do robô
    """
    controller_process = ExecuteProcess(
        cmd=['ros2', 'run', 'harm', 'controller'],
        output='screen'
    )
    pids.append(controller_process.pid)
    """
    

    return LaunchDescription([
        gazebo_process,
        #controller_process
    ])
