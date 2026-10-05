from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scan_topic = LaunchConfiguration("scan_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    output_topic = LaunchConfiguration("output_topic")
    output_cloud_topic = LaunchConfiguration("output_cloud_topic")
    world_cloud_topic = LaunchConfiguration("world_cloud_topic")
    world_cloud_frame = LaunchConfiguration("world_cloud_frame")
    raw_cloud_topic = LaunchConfiguration("raw_cloud_topic")
    oriented_imu_topic = LaunchConfiguration("oriented_imu_topic")
    default_scan_rate_hz = LaunchConfiguration("default_scan_rate_hz")
    lidar_msg_delay_ms = LaunchConfiguration("lidar_msg_delay_ms")
    imu_buffer_sec = LaunchConfiguration("imu_buffer_sec")
    odom_buffer_sec = LaunchConfiguration("odom_buffer_sec")
    laser_offset_x = LaunchConfiguration("laser_offset_x")
    laser_offset_y = LaunchConfiguration("laser_offset_y")
    scan_direction_clockwise = LaunchConfiguration("scan_direction_clockwise")
    publish_raw_cloud = LaunchConfiguration("publish_raw_cloud")
    publish_world_cloud = LaunchConfiguration("publish_world_cloud")
    invert_imu_yaw = LaunchConfiguration("invert_imu_yaw")
    use_odom_translation = LaunchConfiguration("use_odom_translation")
    estimate_orientation_from_raw_imu = LaunchConfiguration("estimate_orientation_from_raw_imu")
    madgwick_beta = LaunchConfiguration("madgwick_beta")

    return LaunchDescription([
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("imu_topic", default_value="/imu/data_raw"),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("output_topic", default_value="/scan_deskewed"),
        DeclareLaunchArgument("output_cloud_topic", default_value="/scan_deskewed_cloud"),
        DeclareLaunchArgument("world_cloud_topic", default_value="/scan_deskewed_world_cloud"),
        DeclareLaunchArgument("world_cloud_frame", default_value="odom"),
        DeclareLaunchArgument("raw_cloud_topic", default_value="/scan_origin_cloud"),
        DeclareLaunchArgument("oriented_imu_topic", default_value="/imu/oriented"),
        DeclareLaunchArgument("default_scan_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("lidar_msg_delay_ms", default_value="10.0"),
        DeclareLaunchArgument("imu_buffer_sec", default_value="3.0"),
        DeclareLaunchArgument("odom_buffer_sec", default_value="3.0"),
        DeclareLaunchArgument("laser_offset_x", default_value="0.0"),
        DeclareLaunchArgument("laser_offset_y", default_value="0.0"),
        DeclareLaunchArgument("scan_direction_clockwise", default_value="false"),
        DeclareLaunchArgument("publish_raw_cloud", default_value="true"),
        DeclareLaunchArgument("publish_world_cloud", default_value="true"),
        DeclareLaunchArgument("invert_imu_yaw", default_value="false"),
        DeclareLaunchArgument("use_odom_translation", default_value="true"),
        DeclareLaunchArgument("estimate_orientation_from_raw_imu", default_value="true"),
        DeclareLaunchArgument("madgwick_beta", default_value="0.05"),
        Node(
            package="scan_undistortion_2d",
            executable="scan_undistortion_node",
            name="scan_undistortion_node",
            output="screen",
            parameters=[{
                "scan_topic": scan_topic,
                "imu_topic": imu_topic,
                "odom_topic": odom_topic,
                "output_topic": output_topic,
                "output_cloud_topic": output_cloud_topic,
                "world_cloud_topic": world_cloud_topic,
                "world_cloud_frame": world_cloud_frame,
                "raw_cloud_topic": raw_cloud_topic,
                "oriented_imu_topic": oriented_imu_topic,
                "default_scan_rate_hz": default_scan_rate_hz,
                "lidar_msg_delay_ms": lidar_msg_delay_ms,
                "imu_buffer_sec": imu_buffer_sec,
                "odom_buffer_sec": odom_buffer_sec,
                "laser_offset_x": laser_offset_x,
                "laser_offset_y": laser_offset_y,
                "scan_direction_clockwise": scan_direction_clockwise,
                "publish_raw_cloud": publish_raw_cloud,
                "publish_world_cloud": publish_world_cloud,
                "invert_imu_yaw": invert_imu_yaw,
                "use_odom_translation": use_odom_translation,
                "estimate_orientation_from_raw_imu": estimate_orientation_from_raw_imu,
                "madgwick_beta": madgwick_beta,
            }],
        ),
    ])
