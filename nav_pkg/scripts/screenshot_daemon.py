#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import numpy as np
import time
import os
import socket
import base64
from volcenginesdkarkruntime import Ark
import requests

requests.packages.urllib3.disable_warnings()

# ===================== 所有配置都在这里 =====================
ROS_IMAGE_TOPIC = "/image"
RECOGNITION_RESULT_TOPIC = "/object_recognition_result"
SAVE_DIR = "./screenshots"
SOCKET_PATH = "/tmp/screenshot.sock"

ARK_API_KEY = os.environ.get("ARK_API_KEY", "")
VOLC_MODEL = "doubao-seed-2-0-mini-260428"   # 你的专属模型ID
TIMEOUT = 20
IMAGE_QUALITY = 100
IMAGE_WIDTH = 1280
MAX_WAIT_FOR_IMAGE = 1.0  # 最多等待1秒获取图像
# ==========================================================

os.makedirs(SAVE_DIR, exist_ok=True)

class TriggeredCaptureDaemon(Node):
    def __init__(self):
        super().__init__('triggered_capture_daemon')
        
        # 提前初始化好所有耗时的对象（只初始化一次）
        self.result_pub = self.create_publisher(String, RECOGNITION_RESULT_TOPIC, 10)
        self.client = Ark(
            base_url="https://ark.cn-beijing.volces.com/api/v3",
            api_key=ARK_API_KEY,
            timeout=TIMEOUT
        )
        
        # 最激进的QoS配置
        self.image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 创建Unix域套接字
        if os.path.exists(SOCKET_PATH):
            os.unlink(SOCKET_PATH)
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.bind(SOCKET_PATH)
        self.sock.listen(1)
        self.sock.setblocking(False)
        
        self.get_logger().info("✅ 守护进程启动完成，所有初始化已完成")
        self.get_logger().info("⏳ 等待客户端触发命令...")

    def capture_single_image(self):
        """【核心】触发时才临时订阅，只取第一帧图像"""
        self.captured_image = None
        self.image_received = False
        self.capture_start_time = time.time() * 1000
        
        # 临时订阅图像话题
        self.sub = self.create_subscription(
            CompressedImage,
            ROS_IMAGE_TOPIC,
            self.single_image_callback,
            self.image_qos
        )
        
        self.get_logger().info("📸 开始捕获客户端触发后的第一帧图像...")
        
        # 等待第一帧图像
        wait_start = time.time()
        while rclpy.ok() and not self.image_received:
            rclpy.spin_once(self, timeout_sec=0.0001)
            if time.time() - wait_start > MAX_WAIT_FOR_IMAGE:
                self.get_logger().error("❌ 超时：1秒内未获取到图像！")
                self.sub.destroy()
                return None
        
        # 立即取消订阅，不再接收新图像
        self.sub.destroy()
        return self.captured_image

    def single_image_callback(self, msg):
        """只处理第一帧图像"""
        if self.image_received:
            return
            
        capture_time = time.time() * 1000
        time_diff = capture_time - self.capture_start_time
        
        self.get_logger().info(f"✅ 捕获到图像！耗时：{time_diff:.1f}ms")
        
        # 解码图像
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.captured_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.image_received = True

    def preprocess_image(self, img):
        """图像预处理"""
        h, w = img.shape[:2]
        new_w = IMAGE_WIDTH
        new_h = int(h * new_w / w)
        img_resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_CUBIC)
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), IMAGE_QUALITY]
        _, img_encoded = cv2.imencode('.jpg', img_resized, encode_param)
        return img_encoded

    def recognize_image(self, img_encoded):
        """AI识别"""
        base64_str = base64.b64encode(img_encoded).decode('utf-8')
        img_base64 = f"data:image/jpeg;base64,{base64_str}"
        
        input_data = [
            {
                "role": "user",
                "content": [
                    {"type": "input_image", "image_url": img_base64},
                    {"type": "input_text", "text": "用最快的速度说明图片中的卡通人物：穿着什么衣服，在做什么，在哪里。人物大概率在医院"}
                ],
            }
        ]

        response = self.client.responses.create(
            model=VOLC_MODEL,
            input=input_data,
            timeout=TIMEOUT,
            stream=False
        )

        # 解析结果
        full_result = ""
        if hasattr(response, 'output') and len(response.output) > 0:
            for output_item in response.output:
                if hasattr(output_item, 'content') and len(output_item.content) > 0:
                    for content_item in output_item.content:
                        if hasattr(content_item, 'text'):
                            full_result += content_item.text
        
        return full_result.strip() if full_result else "未检测到有效内容"

    def process_request(self):
        """处理客户端请求：触发取图→识别→发布结果"""
        # 1. 触发捕获客户端启动后的第一帧图像
        img = self.capture_single_image()
        if img is None:
            return "ERROR:Failed to capture image"
        
        # 2. 保存图像
        timestamp = int(time.time() * 1000)
        filename = f"{SAVE_DIR}/shot_{timestamp}.jpg"
        cv2.imwrite(filename, img)
        self.get_logger().info(f"📂 图像已保存：{filename}")
        
        # 3. AI识别
        ai_start = time.time()
        img_encoded = self.preprocess_image(img)
        result = self.recognize_image(img_encoded)
        ai_time = int((time.time() - ai_start) * 1000)
        
        # 4. 发布结果到ROS话题
        result_msg = String()
        result_msg.data = result
        self.result_pub.publish(result_msg)
        
        self.get_logger().info(f"✅ 识别完成 | AI耗时：{ai_time}ms")
        self.get_logger().info(f"📝 结果：{result}")
        
        return f"OK:{result}:{filename}:{ai_time}"

    def run(self):
        """主循环"""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.001)
            
            try:
                conn, addr = self.sock.accept()
                conn.setblocking(False)
                conn.recv(1024)  # 收到客户端命令
                
                # 立即开始处理
                response = self.process_request()
                conn.send(response.encode())
                conn.close()
                
            except BlockingIOError:
                continue
            except Exception as e:
                self.get_logger().warning(f"⚠️  请求处理错误：{str(e)}")
                continue

def main():
    if not ARK_API_KEY or not VOLC_MODEL:
        print("❌ 错误：请先配置ARK_API_KEY和VOLC_MODEL！")
        return
    
    rclpy.init(args=None)
    daemon = TriggeredCaptureDaemon()
    try:
        daemon.run()
    except KeyboardInterrupt:
        print("\n🛑 守护进程被手动终止")
    finally:
        os.unlink(SOCKET_PATH)
        daemon.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()