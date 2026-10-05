import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("lslidar_driver"), 'launch', 'lsn10_launch.py')
        )
    )

    origincar_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('origincar_base'), 'launch', 'origincar_bringup.launch.py')
        )
    )

    nav_pkg_share = get_package_share_directory("nav_pkg")
    map_file      = os.path.join(nav_pkg_share, "maps", "666.yaml")
    nav_param_file= os.path.join(nav_pkg_share, "config", "nav2_params.yaml")

    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'false',
            'params_file': nav_param_file,
            'autostart': 'true',
            # ? ?   use_amcl       ??     
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(origincar_bringup)
    ld.add_action(lidar_launch)
    ld.add_action(TimerAction(period=3.0, actions=[navigation_cmd]))  #   ?  ?3s    

    # 键盘遥控（标准 teleop_twist_keyboard，按键 u/i/o/j/k/l，k 停止）
    # teleop 读 stdin，launch 默认不给终端，所以把 stdin 指到 /dev/tty
    teleop = ExecuteProcess(
        cmd=['ros2 run teleop_twist_keyboard teleop_twist_keyboard </dev/tty'],
        shell=True,
    )
    ld.add_action(teleop)

    return ld
