from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import launch_ros.actions

def generate_launch_description():
    akmcar = LaunchConfiguration('akmcar', default='false')

    robot_parameters = [
        {'usart_port_name': '/dev/ttyACM0',
         'serial_baud_rate': 115200,
         'robot_frame_id': 'base_footprint',
         'odom_frame_id': 'odom_raw',
         'cmd_vel': 'cmd_vel',
         'product_number': 0}
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'akmcar',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        # 阿克曼模式底盘节点（修复重复remappings）
        launch_ros.actions.Node(
            condition=IfCondition(akmcar),
            package='origincar_base',
            executable='origincar_base_node',
            parameters=robot_parameters + [{'akm_cmd_vel': 'ackermann_cmd'}],
            remappings=[('/odom', '/odom_raw')],  # 只保留一个，冗余的cmd_vel映射删掉
        ),

        launch_ros.actions.Node(
            condition=IfCondition(akmcar),
            package='origincar_base',
            executable='cmd_vel_to_ackermann_drive.py',
            name='cmd_vel_to_ackermann_drive',
        ),

        # 差速模式底盘节点（正确）
        launch_ros.actions.Node(
            condition=UnlessCondition(akmcar),
            package='origincar_base',
            executable='origincar_base_node',
            parameters=robot_parameters + [{'akm_cmd_vel': 'none'}],
            remappings=[('/odom', '/odom_raw')], 
        )
    ])