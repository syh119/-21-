from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="imu660ra_ros2",
                executable="imu660ra_node",
                name="imu660ra_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("imu660ra_ros2"), "config", "imu660ra.yaml"]
                    )
                ],
            )
        ]
    )
