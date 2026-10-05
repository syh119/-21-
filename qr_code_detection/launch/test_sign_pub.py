#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from origincar_msg.msg import Sign

class TestSignPub(Node):
    def __init__(self):
        super().__init__('test_sign_pub')
        # 1. 创建发布者
        self.pub = self.create_publisher(Sign, '/sign_switch', 10)
        # 2. 每0.5秒执行一次发布逻辑
        self.timer = self.create_timer(0.5, self.pub_msg)
        # 3. 初始化Sign消息对象（全局复用）
        self.sign_msg = Sign()
        # 4. 测试用：手动指定二维码内容（后续替换为实际检测结果）
        # 可改成 "AntiClockWise" 测试发布4，或改成其他内容测试不发布
        self.test_qr_data = "ClockWise"  
        self.get_logger().info(f"测试节点启动！模拟二维码内容：{self.test_qr_data}")

    def pub_msg(self):
        # 5. 根据二维码内容赋值（修复缩进+变量定义）
        qr_data = self.test_qr_data  # 读取测试用的二维码内容
        if qr_data == "ClockWise":
            self.sign_msg.sign_data = 3
            self.get_logger().info(f"匹配到ClockWise，发布sign_data=3")
        elif qr_data == "AntiClockWise":
            self.sign_msg.sign_data = 4
            self.get_logger().info(f"匹配到AntiClockWise，发布sign_data=4")
        else:
            self.get_logger().info(f"二维码内容不匹配：{qr_data}，不发布")
            return  # 不发布，直接返回
        
        # 6. 发布赋值后的消息（修复：发布的是赋值后的self.sign_msg，不是空的msg）
        self.pub.publish(self.sign_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TestSignPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()