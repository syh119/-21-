import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python import get_package_share_directory

def generate_launch_description():
    # 1. USB摄像头节点（发布原始MJPEG图像到/image话题）
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
            {"video_device": "/dev/video8"},
            {"zero_copy": False}
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )

    # 2. NV12编解码节点（ROS话题转共享内存，供感知模型使用）
    nv12_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('hobot_codec'), 'launch/hobot_codec_decode.launch.py')
        ),
        launch_arguments={
            'codec_in_mode': 'ros',
            'codec_out_mode': 'shared_mem',
            'codec_sub_topic': '/image',
            'codec_pub_topic': '/hbmem_img'
        }.items()
    )

    # 3. 赛道检测模型节点
    racing_track_detection_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('racing_track_detection_resnet'), 'launch', 'racing_track_detection_resnet.launch.py')
        )
    )

    # 4. 障碍物/二维码检测模型节点
    racing_obstacle_detection_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('racing_obstacle_detection_yolo'), 'launch', 'racing_obstacle_detection_yolo.launch.py')
        )
    )

    # 5. WebSocket可视化节点（可选，用于网页查看图像和检测结果）
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('websocket') + '/launch/websocket.launch.py'
        ),
        launch_arguments={
            'websocket_image_topic': '/image',
            'websocket_image_type': 'mjpeg',
            'websocket_smart_topic': '/racing_obstacle_detection'
        }.items()
    )

    # 6. 新增：Python二维码解析节点（仅保留这一个新增节点）
    qr_code_scanner_node = Node(
        package='racing_control',  # 替换为Python脚本所在的实际包名
        executable='qr_code_scanner_node.py',  # Python脚本文件名
        name='qr_code_scanner_node',
        output='screen',  # 日志打印到终端，方便调试
        # 强制使用Python3解释器，避免环境问题
        prefix='python3 ',
        arguments=['--ros-args', '--log-level', 'info']  # 日志级别设为info
    )

    # 组装所有节点（仅保留原有节点 + 二维码解析节点）
    return LaunchDescription([
        usb_cam_node,
        nv12_codec_node,
        racing_track_detection_node,
        racing_obstacle_detection_node,
        qr_code_scanner_node,  # 仅新增二维码解析节点
        web_node  # WebSocket可选保留
    ])