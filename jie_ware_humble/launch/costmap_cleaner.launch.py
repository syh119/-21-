from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('initial_pose_topic', default_value='/initialpose'),
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
            executable='costmap_cleaner',
            name='costmap_cleaner',
            output='screen',
            parameters=[{
                'initial_pose_topic': LaunchConfiguration('initial_pose_topic'),
                'local_clear_service': LaunchConfiguration('local_clear_service'),
                'global_clear_service': LaunchConfiguration('global_clear_service'),
            }],
        ),
    ])
