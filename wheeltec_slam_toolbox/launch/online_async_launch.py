import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ==================== 1. 硬件启动 ====================
    origincar_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('origincar_base'), 'launch', 'origincar_bringup.launch.py')
        )
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("lslidar_driver"), 'launch', 'lsn10_launch.py')
        )
    )

    # ==================== 2. 雷达静态 TF ====================
   

    # ==================== 3. 激光时间戳修复 ====================
    scan_timestamp_fix = Node(
        package='topic_tools',
        executable='relay',
        name='scan_timestamp_fix',
        arguments=['/scan', '/scan_deskewed'],
        parameters=[{'use_system_time': True}]
    )

    # ==================== 4. SLAM 参数配置（核心优化） ====================
    slam_params = {
        "use_sim_time": False,
        "base_frame": "base_footprint",
        "odom_frame": "odom",
        "map_frame": "map",
        "resolution": 0.05,
        
        # 【关键修复】匹配镭神N10量程
        "max_laser_range": 12.0,
        
        # 【关键修复】时间补偿：激光数据大约延迟0.2秒，我们让SLAM往未来看一点
        "transform_time_offset": 0.2, 
        
        # 【关键修复】超大TF缓存，专治时间不同步
        "tf_buffer_duration": 30.0,
        "tf_listener_timeout": 1.0,
        "acceptable_timestamp_delay": 2.0,
        
        # 队列加大
        "odom_queue_size": 2000,
        "scan_queue_size": 2000,
        
        # 建图逻辑优化
        "do_loop_closing": True,
        "use_odometry": True,
        "link_scan_maximum_distance": 3.0,
        "minimum_travel_distance": 0.1,
        "minimum_travel_heading": 0.1,
    }

    # ==================== 5. SlamToolbox 节点 ====================
    slam_toolbox_node = Node(
        parameters=[slam_params],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        remappings=[
            ('odom', '/odom'),
            ('scan', '/scan_deskewed')
        ]
    )

    # ==================== 启动顺序 ====================
    ld = LaunchDescription()
    ld.add_action(origincar_bringup)
    ld.add_action(lidar_launch)
    ld.add_action(scan_timestamp_fix)
    # 延时增加到 5 秒，确保硬件完全就绪
    ld.add_action(TimerAction(period=2.0, actions=[slam_toolbox_node]))

    return ld