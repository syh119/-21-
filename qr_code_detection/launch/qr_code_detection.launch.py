#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2022, www.guyuehome.com

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    usb_cam_node = Node(
        package='hobot_usb_cam',
        executable='hobot_usb_cam',
        name='hobot_usb_cam',
        parameters=[
            {"camera_calibration_file_path": "/opt/tros/lib/hobot_usb_cam/config/usb_camera_calibration.yaml"},
            {"frame_id": "default_usb_cam"},
            {"framerate": 40},                
            {"image_width": 1920},             
            {"image_height": 1080},             
            {"io_method": "mmap"},            
            {"pixel_format": "mjpeg"},         
            {"video_device": "/dev/video8"},  
            {"zero_copy": False}
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )

    qr_detection_node = Node(
        package='qr_code_detection',
        executable='qr_detection_node',
        output='screen'
    )

    oled_text_renderer_node = Node(
        package='qr_code_detection',
        executable='oled_text_renderer',
        output='screen'
    )

    return LaunchDescription([
        usb_cam_node,
        nv12_codec_node,
        qr_detection_node,
        oled_text_renderer_node
    ])
