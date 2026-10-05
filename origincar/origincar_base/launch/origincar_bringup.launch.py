import os
from pathlib import Path
import launch
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
import launch_ros.actions
from launch.conditions import UnlessCondition


def generate_launch_description():
    bringup_dir = get_package_share_directory('origincar_base')
    launch_dir = os.path.join(bringup_dir, 'launch')
    ekf_config = Path(get_package_share_directory('origincar_base'), 'config', 'ekf.yaml')
    imu_config = Path(get_package_share_directory('origincar_base'), 'config', 'imu.yaml')
    imu660ra_launch = Path(get_package_share_directory('imu660ra_ros2'), 'launch', 'imu660ra.launch.py')

    carto_slam = LaunchConfiguration('carto_slam', default='false')
    carto_slam_dec = DeclareLaunchArgument('carto_slam', default_value='false')

    origincar_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'base_serial.launch.py')),
        launch_arguments={'akmcar': 'true'}.items(),
    )

    choose_car = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'robot_mode_description.launch.py')),
    )

    base_to_link = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_link',
        arguments=['0.0', '0.0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
    )
    base_to_gyro = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_gyro',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'gyro_link'],
    )
    base_to_imu = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu',
        arguments=['0.04', '0.04', '0.0', '0.0', '0.0', '0.0', 'base_link', 'imu_link'],
    )


    imu660ra_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(imu660ra_launch)),
    )

    robot_ekf = launch_ros.actions.Node(
        condition=UnlessCondition(carto_slam),
        package='robot_localization',
        executable='ekf_node',
        parameters=[ekf_config],
        remappings=[("odometry/filtered", "odom")]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    base_to_laser = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser',
        arguments=['0.1', '0.0', '0.10', '0', '0', '0', 'base_link', 'laser'],
    )

    ld = LaunchDescription()

    ld.add_action(carto_slam_dec)
    ld.add_action(origincar_base)
    ld.add_action(base_to_link)
    ld.add_action(base_to_gyro)
    ld.add_action(base_to_imu)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(choose_car)
    ld.add_action(imu660ra_node)
    ld.add_action(robot_ekf)
    ld.add_action(base_to_laser)

    return ld
