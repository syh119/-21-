from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('laser_frame', default_value='laser'),
        DeclareLaunchArgument('laser_topic', default_value='/scan'),
        DeclareLaunchArgument('initial_pose_topic', default_value='/initialpose'),
        DeclareLaunchArgument('map_topic', default_value='/map'),
        DeclareLaunchArgument(
            'local_clear_service',
            default_value='/local_costmap/clear_entirely_local_costmap',
        ),
        DeclareLaunchArgument(
            'global_clear_service',
            default_value='/global_costmap/clear_entirely_global_costmap',
        ),
        Node(
            package='jie_ware_humble',
            executable='lidar_loc',
            name='lidar_loc',
            output='screen',
            parameters=[{
                'base_frame': LaunchConfiguration('base_frame'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'laser_frame': LaunchConfiguration('laser_frame'),
                'laser_topic': LaunchConfiguration('laser_topic'),
                'initial_pose_topic': LaunchConfiguration('initial_pose_topic'),
                'map_topic': LaunchConfiguration('map_topic'),
                'local_clear_service': LaunchConfiguration('local_clear_service'),
                'global_clear_service': LaunchConfiguration('global_clear_service'),
            }],
        ),
    ])
