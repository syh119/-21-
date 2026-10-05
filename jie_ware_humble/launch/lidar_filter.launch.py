from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('source_topic', default_value='/scan'),
        DeclareLaunchArgument('pub_topic', default_value='/scan_filtered'),
        DeclareLaunchArgument('outlier_threshold', default_value='0.1'),
        Node(
            package='jie_ware_humble',
            executable='lidar_filter_node',
            name='lidar_filter_node',
            output='screen',
            parameters=[{
                'source_topic': LaunchConfiguration('source_topic'),
                'pub_topic': LaunchConfiguration('pub_topic'),
                'outlier_threshold': LaunchConfiguration('outlier_threshold'),
            }],
        ),
    ])
