import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 雷达节点
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("lslidar_driver"), 'launch', 'lsn10_launch.py')
        )
    )

    # 2. 底盘节点
    origincar_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('origincar_base'), 'launch', 'origincar_bringup.launch.py')
        )
    )

    # 3. 雷达TF
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf',
        output='screen',
        arguments=[
            '--x', '0.2', '--y', '0.0', '--z', '0.1',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'laser'
        ]
    )

    # 4. 激光时间戳修复（只定义一次！）
    scan_restamp = Node(
    package='origincar_base',        # 放到你自己的包里
    executable='scan_restamp',
    name='scan_restamp',
    output='screen'
    )

    # 【核心优化】SLAM参数，适配你的真实硬件
    slam_params = {
        "use_sim_time": False,
        "base_frame": "base_footprint",
        "odom_frame": "odom_combined",
        "map_frame": "map",
        "resolution": 0.05,
        "minimum_travel_distance": 0.0,
        "minimum_travel_heading": 0.0,
        # 【修正1】匹配你日志里的真实硬件量程：0.2m-10.0m
        "max_laser_range": 10.0,
        "minimum_laser_range": 0.2,
        "do_loop_closing": True,
        "use_odometry": True,
        "tf_listener_timeout": 0.5,
        "tf_buffer_duration": 30.0,
        "acceptable_timestamp_delay": 1.0,
        "odom_queue_size": 2000,
        "scan_queue_size": 2000,
        # 适配阿克曼小车的里程计噪声
        "odom_alpha1": 2.0,
        "odom_alpha2": 2.0,
        "odom_alpha3": 1.5,
        "odom_alpha4": 1.5,
        # 【修正2】既然用了时间戳修复节点，这里设成0.0，避免双重补偿
        "transform_time_offset": 0.0,
        # 【修正3】匹配真实激光量程
        "scan_buffer_maximum_scan_distance": 10.0,
        # 【新增】消掉激光点数不匹配的警告
        "ignore_laser_timestamp_mismatch": True,
        # 建图优化参数
        "link_scan_maximum_distance": 3.0,
        "loop_search_maximum_distance": 4.0,
        "minimum_travel_distance": 0.1,
        "minimum_travel_heading": 0.1,
        "scan_buffer_size": 10,
        "correlation_search_space_dimension": 0.5,
        "correlation_search_space_resolution": 0.05,
        "correlation_search_space_smear_deviation": 0.1,
        "distance_map_search_space_dimension": 5.0,
        "distance_map_search_space_resolution": 0.05,
        "distance_map_search_space_smear_deviation": 0.1,
        "loop_search_space_dimension": 6.0,
        "loop_search_space_resolution": 0.05,
        "loop_search_space_smear_deviation": 0.1,
        "loop_search_maximum_distance": 4.0,
        "loop_search_minimum_chain_size": 10,
        "loop_search_maximum_variance": 0.5,
        "loop_search_maximum_correlation_score": 0.8,
        "loop_search_maximum_distance_between_scans": 3.0,
    }

    # SLAM节点
    slam_cmd = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="async_slam_toolbox_node",
        parameters=[slam_params],
        remappings=[
            ("scan", "/scan_deskewed"),  # 用修复后的时间戳话题
            ("odom", "/odom")
        ],
        output='screen'
    )

    # 启动顺序
    ld = LaunchDescription()
    ld.add_action(origincar_bringup)
    ld.add_action(lidar_launch)
    ld.add_action(lidar_tf)
    ld.add_action(scan_timestamp_fix)
    ld.add_action(TimerAction(period=5.0, actions=[slam_cmd]))

    return ld