import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python import get_package_share_directory

def generate_launch_description():
    # NV12编解码节点 - 将MJPEG转换为共享内存格式
    nv12_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_decode.launch.py'
            )
        ),
        launch_arguments={
            'codec_in_mode': 'ros',           # 输入模式：ROS话题
            'codec_out_mode': 'shared_mem',   # 输出模式：共享内存
            'codec_sub_topic': '/image',      # 订阅原始图像
            'codec_pub_topic': '/hbmem_img'   # 发布共享内存图像
        }.items()
    )

    # WebSocket调试节点 - 用于实时预览
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py'
            )
        ),
        launch_arguments={
            'websocket_image_topic': '/image',
            'websocket_smart_topic': '/racing_track_center_detection'
        }.items()
    )

    # USB摄像头节点配置
    usb_cam_node = Node(
        package='hobot_usb_cam',
        executable='hobot_usb_cam',
        name='hobot_usb_cam',
        parameters=[
            {"camera_calibration_file_path": "/opt/tros/lib/hobot_usb_cam/config/usb_camera_calibration.yaml"},
            {"frame_id": "default_usb_cam"},
            {"framerate": 30},                # 帧率：30fps
            {"image_width": 640},             # 图像宽度
            {"image_height": 480},            # 图像高度
            {"io_method": "mmap"},            # 内存映射方式
            {"pixel_format": "mjpeg"},        # 像素格式
            {"video_device": "/dev/video1"},  # 设备节点（需根据实际调整）
            {"zero_copy": False}
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )

    return LaunchDescription([
        usb_cam_node,
        nv12_codec_node,
        web_node,
    ])