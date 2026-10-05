#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""全图快速定位二维码候选区域，再对候选区域进行精解码。"""

import os
import time

import cv2
import cv_bridge
import rclpy
from ament_index_python.packages import get_package_share_directory
from origincar_msg.msg import Sign
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class QrCodeDetectionFastCandidate(Node):
    """完整画面快速找码，只把候选框交给 WeChat QR 解码器。"""

    def __init__(self):
        super().__init__('qrcode_detect_fast_candidate')
        self.get_logger().info('Start fast-candidate QR detector.')

        self.pub_qrcode_info = self.create_publisher(Sign, '/sign_switch', 10)
        self.qr_display_pub = self.create_publisher(String, '/qr_display_text', 10)
        self.image_sub = self.create_subscription(
            CompressedImage, '/image', self.image_callback, 10)
        self.bridge = cv_bridge.CvBridge()
        self.sign_msg = Sign()

        # 候选框外额外保留的比例，避免切边影响精解码。
        self.declare_parameter('candidate_padding_ratio', 0.5)
        self.candidate_padding_ratio = self.get_parameter(
            'candidate_padding_ratio').value

        # 第一级：只定位，不在全图做内容解码。
        self.fast_detector = cv2.QRCodeDetector()

        # 第二级：仅对第一级定位出的较小候选区域进行鲁棒解码。
        model_path = os.path.join(
            get_package_share_directory('qr_code_detection'), 'model')
        self.wechat_detector = cv2.wechat_qrcode_WeChatQRCode(
            os.path.join(model_path, 'detect.prototxt'),
            os.path.join(model_path, 'detect.caffemodel'),
            os.path.join(model_path, 'sr.prototxt'),
            os.path.join(model_path, 'sr.caffemodel'))

    def image_callback(self, msg):
        try:
            image = self.bridge.compressed_imgmsg_to_cv2(msg)
            found, points = self.fast_detector.detect(image)
        except Exception as exc:
            self.get_logger().warn(f'Fast QR candidate detection failed: {exc}')
            return

        if not found or points is None:
            return

        candidate = self.extract_candidate(image, points)
        if candidate is None:
            return

        try:
            qr_info, _ = self.wechat_detector.detectAndDecode(candidate)
        except cv2.error as exc:
            self.get_logger().warn(f'Candidate QR decoding failed: {exc}')
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
        self.publish_and_shutdown(f'{number} {direction}')

    def extract_candidate(self, image, points):
        """从完整图像中按候选框切出带安全边缘的解码区域。"""
        height, width = image.shape[:2]
        corners = points.reshape(-1, 2).astype(float)
        x_min, y_min = corners.min(axis=0)
        x_max, y_max = corners.max(axis=0)

        side = max(x_max - x_min, y_max - y_min)
        padding = max(16, int(side * self.candidate_padding_ratio))
        left = max(0, int(x_min) - padding)
        top = max(0, int(y_min) - padding)
        right = min(width, int(x_max) + padding)
        bottom = min(height, int(y_max) + padding)
        if right <= left or bottom <= top:
            return None
        return image[top:bottom, left:right]

    def publish_and_shutdown(self, qr_display_text):
        try:
            display_msg = String()
            display_msg.data = qr_display_text
            for _ in range(3):
                self.pub_qrcode_info.publish(self.sign_msg)
                self.qr_display_pub.publish(display_msg)
                time.sleep(0.1)
            self.get_logger().info('QR parsing completed, closing current node')
            self.destroy_node()
        except Exception as exc:
            self.get_logger().error(f'Failed to publish message: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = QrCodeDetectionFastCandidate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
