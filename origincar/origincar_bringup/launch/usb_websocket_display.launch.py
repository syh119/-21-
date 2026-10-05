import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory, get_package_prefix

def generate_launch_description():

    # 仅保留USB摄像头参数
    launch_args = [
        DeclareLaunchArgument('device', default_value='/dev/video0', description='usb camera device'),
    ]

    # 启动USB摄像头
    usb_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('hobot_usb_cam') + '/launch/hobot_usb_cam.launch.py'),
                                       launch_arguments={'usb_image_width': '640', 'usb_image_height': '480',
                                                         'usb_video_device': LaunchConfiguration('device')}.items())

    # 视频编解码（必需）
    nv12_codec_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('hobot_codec') + '/launch/hobot_codec_decode.launch.py'),
                                               launch_arguments={'codec_in_mode': 'ros', 'codec_out_mode': 'shared_mem',
                                                                 'codec_sub_topic': '/image', 'codec_pub_topic': '/hbmem_img'}.items())

    jpeg_codec_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('hobot_codec') + '/launch/hobot_codec_encode.launch.py'),
                                               launch_arguments={'codec_in_mode': 'shared_mem', 'codec_out_mode': 'ros',
                                                                 'codec_sub_topic': '/hbmem_img', 'codec_pub_topic': '/image'}.items())

    # 网页显示（必需）
    web_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('websocket') + '/launch/websocket.launch.py'),
                                        launch_arguments={'websocket_image_topic': '/image', 'websocket_image_type': 'mjpeg',
                                                          'websocket_smart_topic': '/racing_obstacle_detection'}.items())

    # 图像传输节点
    image_transport_node = Node(
        package='utils',
        executable='image_transport_node',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    # 【核心】删除了所有 dnn_node_example 相关代码！
    return LaunchDescription(launch_args + [
        usb_node,
        nv12_codec_node,
        jpeg_codec_node,
        web_node,
        image_transport_node
    ])