#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import requests
import base64
import json
import time
import os

# ===================== 配置项（替换为你的百度密钥）=====================
API_KEY = os.environ.get("BAIDU_API_KEY", "")
SECRET_KEY = os.environ.get("BAIDU_SECRET_KEY", "")
# ======================================================================

class ROS2ImageToTextNode(Node):
    def __init__(self):
        super().__init__('rdk_cloud_img2text_node')
        
        # 订阅ROS2摄像头话题（和你的hobot_usb_cam节点发布的话题一致）
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image',  # 固定订阅/hobot_usb_cam发布的/image话题
            self.image_callback,
            10  # 消息队列大小
        )
        
        # 控制是否只处理一帧（避免重复调用API）
        self.processed = False
        self.get_logger().info("✅ RDK X5 独立版云端图生文节点已启动")
        self.get_logger().info(f"🔍 正在订阅 /image 话题（摄像头设备：/dev/video8）")
        self.get_logger().info("💡 提示：仅处理第一帧图像，按Ctrl+C终止节点")

    def get_baidu_access_token(self):
        """获取百度API访问令牌"""
        url = f"https://aip.baidubce.com/oauth/2.0/token?grant_type=client_credentials&client_id={API_KEY}&client_secret={SECRET_KEY}"
        try:
            response = requests.post(url, timeout=10)
            if response.status_code == 200:
                token_data = response.json()
                self.get_logger().info("✅ 百度API Token获取成功")
                return token_data.get("access_token")
            else:
                self.get_logger().error(f"❌ Token获取失败：{response.text}")
                return None
        except requests.exceptions.Timeout:
            self.get_logger().error("❌ 网络超时！请检查RDK X5外网连接")
            return None
        except Exception as e:
            self.get_logger().error(f"❌ Token请求异常：{str(e)}")
            return None

    def img_to_base64(self, img_path):
        """将本地图像转为Base64编码（适配百度API）"""
        try:
            with open(img_path, "rb") as f:
                base64_data = base64.b64encode(f.read()).decode('utf-8')
            return base64_data
        except Exception as e:
            self.get_logger().error(f"❌ 图像转Base64失败：{str(e)}")
            return None

    def cloud_image_to_text(self, img_path):
        """调用百度云端API实现图生文"""
        # 1. 获取Token
        access_token = self.get_baidu_access_token()
        if not access_token:
            return "Token获取失败，无法调用云端API"
        
        # 2. 图像编码
        img_base64 = self.img_to_base64(img_path)
        if not img_base64:
            return "图像编码失败"
        
        # 3. 构造API请求
        api_url = f"https://aip.baidubce.com/rpc/2.0/ai_custom/v1/wenxinworkshop/chat/completions?access_token={access_token}"
        messages = [
            {
                "role": "user",
                "content": [
                    {"type": "image", "image": img_base64},
                    {"type": "text", "text": "请详细描述这张图片的内容，重点说明赛道、障碍物、二维码等元素（如果有）"}
                ]
            }
        ]
        payload = {
            "model": "ernie-vilg-v2",  # 百度支持图像理解的轻量模型
            "messages": messages,
            "temperature": 0.5,        # 生成文本更稳定
            "max_tokens": 200          # 最大生成字数
        }
        
        # 4. 发送请求到云端
        try:
            self.get_logger().info("🔄 正在请求百度云端API...")
            response = requests.post(api_url, json=payload, timeout=20)
            if response.status_code == 200:
                result = response.json()
                return result.get("result", "云端未返回有效描述")
            else:
                return f"❌ API调用失败：{response.status_code} - {response.text}"
        except requests.exceptions.Timeout:
            return "❌ 云端请求超时，请检查网络或稍后重试"
        except Exception as e:
            return f"❌ 云端调用异常：{str(e)}"

    def image_callback(self, msg):
        """图像话题回调函数（接收到摄像头图像后执行）"""
        # 仅处理第一帧图像，避免重复调用API
        if self.processed:
            return
        
        try:
            # 1. 解析ROS2的MJPEG格式图像消息
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # 2. 保存图像到临时目录（避免权限问题）
            img_path = "/tmp/rdk_ros2_img.jpg"
            cv2.imwrite(img_path, img)
            self.get_logger().info(f"📸 摄像头图像采集成功，保存路径：{img_path}")
            
            # 3. 调用云端图生文接口
            start_time = time.time()
            result_text = self.cloud_image_to_text(img_path)
            end_time = time.time()
            
            # 4. 打印最终结果
            self.get_logger().info("\n==================== 图生文结果 ====================")
            self.get_logger().info(f"⏱️  总耗时：{end_time - start_time:.2f} 秒")
            self.get_logger().info(f"📝 图像描述：\n{result_text}")
            self.get_logger().info("=====================================================")
            
            # 标记为已处理，避免重复执行
            self.processed = True
            
        except Exception as e:
            self.get_logger().error(f"❌ 图像处理失败：{str(e)}")

def main(args=None):
    # 初始化ROS2上下文
    rclpy.init(args=args)
    
    # 创建并启动图生文节点
    node = ROS2ImageToTextNode()
    
    # 保持节点运行（直到Ctrl+C终止）
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 图生文节点被手动终止")
    finally:
        # 清理资源
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()