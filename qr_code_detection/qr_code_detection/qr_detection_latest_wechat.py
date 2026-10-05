#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""使用 WeChat 全图识别，并始终丢弃过期相机帧的二维码节点。"""

import os
import time

import cv2
import cv_bridge
import rclpy
from ament_index_python.packages import get_package_share_directory
from origincar_msg.msg import Sign
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class QrCodeDetectionLatestWechat(Node):
    """维持原版 WeChat 全图识别率，并只解码最新一帧。"""

    def __init__(self):
        super().__init__('qrcode_detect_latest_wechat')
        self.get_logger().info('Start latest-frame WeChat QR detector.')

        self.pub_qrcode_info = self.create_publisher(Sign, '/sign_switch', 10)
        self.qr_display_pub = self.create_publisher(String, '/qr_display_text', 10)

        # 相机图像只保留最新一帧，处理慢时自动丢弃过期帧。
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT)
        self.image_sub = self.create_subscription(
            CompressedImage, '/image', self.image_callback, image_qos)

        self.declare_parameter('decode_interval_sec', 0.12)
        self.decode_interval_sec = self.get_parameter(
            'decode_interval_sec').value
        self.latest_image_msg = None
        self.last_decode_time = 0.0
        self.completed = False
        self.publish_count = 0
        self.publish_timer = None

        self.bridge = cv_bridge.CvBridge()
        self.sign_msg = Sign()
        model_path = os.path.join(
            get_package_share_directory('qr_code_detection'), 'model')
        self.detector = cv2.wechat_qrcode_WeChatQRCode(
            os.path.join(model_path, 'detect.prototxt'),
            os.path.join(model_path, 'detect.caffemodel'),
            os.path.join(model_path, 'sr.prototxt'),
            os.path.join(model_path, 'sr.caffemodel'))

        # 订阅回调只缓存图像；解码在定时器中进行，避免处理旧帧。
        self.decode_timer = self.create_timer(0.02, self.process_latest_image)

    def image_callback(self, msg):
        if not self.completed:
            self.latest_image_msg = msg

    def process_latest_image(self):
        if self.completed or self.latest_image_msg is None:
            return

        now = time.monotonic()
        if now - self.last_decode_time < self.decode_interval_sec:
            return

        # 取走当前最新帧；解码期间新到的帧会覆盖缓存中的旧帧。
        image_msg = self.latest_image_msg
        self.latest_image_msg = None
        self.last_decode_time = now

        try:
            image = self.bridge.compressed_imgmsg_to_cv2(image_msg)
            qr_info, _ = self.detector.detectAndDecode(image)
        except Exception as exc:
            self.get_logger().warn(f'QR decoding failed: {exc}')
            return

        if not qr_info:
            return

        qr_text = qr_info[0].strip()
        if not qr_text:
            return

        self.get_logger().info(f'Detected QR code: {qr_text}')
        if not qr_text.isdigit():
            self.get_logger().warn(
                f'Invalid QR content: {qr_text}, only numbers are supported')
            return

        number = int(qr_text)
        self.sign_msg.sign_data = 3 if number % 2 else 4
        direction = 'CW' if number % 2 else 'CCW'
        self.start_publish(f'{number} {direction}')

    def start_publish(self, qr_display_text):
        """立即发布第一条结果，后续两条非阻塞重发。"""
        self.completed = True
        self.decode_timer.cancel()
        self.display_msg = String()
        self.display_msg.data = qr_display_text
        self.publish_result()
        self.publish_timer = self.create_timer(0.1, self.publish_result)

    def publish_result(self):
        self.pub_qrcode_info.publish(self.sign_msg)
        self.qr_display_pub.publish(self.display_msg)
        self.publish_count += 1
        if self.publish_count < 3:
            return

        self.publish_timer.cancel()
        self.get_logger().info('QR parsing completed, closing current node')
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QrCodeDetectionLatestWechat()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
