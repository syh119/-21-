import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python import get_package_share_directory

def generate_launch_description():
    nv12_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_decode.launch.py'
            )
        ),
        launch_arguments={
            'codec_in_mode': 'ros',
            'codec_out_mode': 'shared_mem',
            'codec_sub_topic': '/image',
            'codec_pub_topic': '/hbmem_img'
        }.items()
    )

    # WebSocket调试 - 专门用于障碍物检测可视化
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('websocket') + '/launch/websocket.launch.py'
        ),
        launch_arguments={
            'websocket_image_topic': '/image',
            'websocket_image_type': 'mjpeg',
            'websocket_smart_topic': '/racing_obstacle_detection'  # 障碍物检测话题
        }.items()
    )

    usb_cam_node = Node(
        package='hobot_usb_cam',
        executable='hobot_usb_cam',
        name='hobot_usb_cam',
        parameters=[
            {"camera_calibration_file_path": "/opt/tros/lib/hobot_usb_cam/config/usb_camera_calibration.yaml"},
            {"frame_id": "default_usb_cam"},
            {"framerate": 30},
            {"image_width": 640},
            {"image_height": 480},
            {"io_method": "mmap"},
            {"pixel_format": "mjpeg"},
            {"video_device": "/dev/video1"},
            {"zero_copy": False}
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )

    return LaunchDescription([
        usb_cam_node,
        nv12_codec_node,
        web_node,
    ])