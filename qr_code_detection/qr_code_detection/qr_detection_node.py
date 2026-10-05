#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2022, www.guyuehome.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import cv2
import cv_bridge
import os
import time
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import numpy as np
from origincar_msg.msg import Sign
from ament_index_python.packages import get_package_share_directory

color = (0, 0, 255)
thick = 3
font_scale = 0.5
font_thickness = 2


class QrCodeDetection(Node):
    def __init__(self):
        super().__init__('qrcode_detect')
        self.get_logger().info("Start qrcode_detect.")
        self.pub_qrcode_info = self.create_publisher(Sign, '/sign_switch', 10)
        self.qr_display_pub = self.create_publisher(String, '/qr_display_text', 10)
        self.image_sub = self.create_subscription(CompressedImage, "/image", self.image_callback, 10)
        
        self.bridge = cv_bridge.CvBridge()
        self.Signmsg = Sign()

        modelPath = os.path.join(get_package_share_directory('qr_code_detection'), 'model/')

        self.detect_obj = cv2.wechat_qrcode_WeChatQRCode(
            modelPath+'detect.prototxt', modelPath+'detect.caffemodel',
            modelPath+'sr.prototxt', modelPath+'sr.caffemodel')

    def image_callback(self, msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)

        qrInfo, qrPoints = self.detect_obj.detectAndDecode(cv_image)
        emptyList = ()
        if qrInfo != emptyList:
            self.get_logger().info('qrInfo: "{0}"'.format(qrInfo))
            self.get_logger().info('qrPoints: "{0}"'.format(qrPoints))

            qrInfo_str = qrInfo[0].strip()
            
            # Core logic: Judge number parity, odd=clockwise, even=counterclockwise
            if qrInfo_str.isdigit():
                number = int(qrInfo_str)
                # Odd number → Clockwise (publish 3)
                if number % 2 == 1:
                    self.Signmsg.sign_data = 3
                    self.get_logger().info(f'Detected odd number: {number}, command: Clockwise')
                    self.publish_and_shutdown(f'{number} CW')
                # Even number → Counterclockwise (publish 4)
                else:
                    self.Signmsg.sign_data = 4
                    self.get_logger().info(f'Detected even number: {number}, command: Counterclockwise')
                    self.publish_and_shutdown(f'{number} CCW')
            else:
                self.get_logger().warn(f'Invalid QR content: {qrInfo_str}, only numbers are supported')
                return
    
    def publish_and_shutdown(self, qr_display_text):
        try:
            self.get_logger().info(f'Publishing to /sign_switch: {self.Signmsg.sign_data}')
            display_msg = String()
            display_msg.data = qr_display_text
            self.get_logger().info(f'Publishing to /qr_display_text: {display_msg.data}')
            for i in range(3):
                self.pub_qrcode_info.publish(self.Signmsg)
                self.qr_display_pub.publish(display_msg)
                time.sleep(0.1)
            self.get_logger().info('QR code parsing completed, closing current node')
            self.destroy_node()
        except Exception as e:
            self.get_logger().error(f'Failed to publish message: {e}')


def main(args=None):
    rclpy.init(args=args)
    qrCodeDetection = QrCodeDetection()
    rclpy.spin(qrCodeDetection)
    qrCodeDetection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
