#!/usr/bin/env python3

import rclpy
import time  # 新增：用于发布重发延时，避免消息丢失
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from origincar_msg.msg import Sign  # 导入自定义消息
from ament_index_python.packages import get_package_share_directory

class QRCodeDetector(Node):
    def __init__(self):
        super().__init__('qr_code_detector')
        # 修复1：订阅实际摄像头话题 /image（原/image_raw是错的，和你的环境匹配）
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10)
        # 保留原发布器：/sign_switch + Sign消息类型，无需修改
        self.pub_qrcode_info = self.create_publisher(Sign, '/sign_switch', 10)
        self.Signmsg = Sign()
        self.bridge = CvBridge()
        self.get_logger().info('QR Code Detector Node has been started.')

    def listener_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.detect_qr_code(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def detect_qr_code(self, image):
        detector = cv2.QRCodeDetector()
        data, bbox, _ = detector.detectAndDecode(image)
        if bbox is not None and data.strip() != "":  # 增加空内容过滤，避免无效识别
            self.get_logger().info(f'Detected QR code: {data.strip()}')
            # 初始化消息（避免重复发布旧值）
            self.Signmsg = Sign()
            if data.strip() == "ClockWise":
                self.Signmsg.sign_data = 3
                self.publish_and_cleanup()  # 匹配成功，发布+销毁自身
            elif data.strip() == "AntiClockWise":
                self.Signmsg.sign_data = 4
                self.publish_and_cleanup()  # 匹配成功，发布+销毁自身
            else:
                # 修复2：识别其他二维码（如8888）不return，打印日志方便测试，不发布
                self.get_logger().warn(f'QR code not match (ClockWise/AntiClockWise): {data.strip()}')
        # 识别不到二维码，无任何操作，避免日志刷屏

    # 新增核心函数：发布+重发+安全销毁自身（替代原全局shutdown）
    def publish_and_cleanup(self):
        try:
            self.get_logger().info(f'Publishing to /sign_switch: {self.Signmsg.sign_data}')
            # 修复3：重发3次+短延时，确保ROS2消息不丢失，导航节点必收到
            for i in range(3):
                self.pub_qrcode_info.publish(self.Signmsg)
                time.sleep(0.1)  # 100ms延时，避免消息被缓冲区覆盖
            # 修复4：仅销毁当前解析节点，不全局关闭ROS2（关键！联合启动不影响其他节点）
            self.get_logger().info('Publish success, shutting down QR detector node only.')
            self.destroy_node()
        except Exception as e:
            self.get_logger().error(f'Publish failed: {e}')
        
def main(args=None):
    rclpy.init(args=args)
    qr_code_detector = QRCodeDetector()
    rclpy.spin(qr_code_detector)
    # 原销毁逻辑保留，实际会被publish_and_cleanup里的destroy_node替代
    qr_code_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()