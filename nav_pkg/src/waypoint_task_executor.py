#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
import subprocess
import threading
import time
import math
import os
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

try:
    from scipy.spatial.transform import Rotation as R
    SCIPY = True
except:
    SCIPY = False

class NavCarNode(Node):
    def __init__(self):
        super().__init__("nav_car_task")
        self.get_logger().info("✅ 导航任务节点已启动")

        # ===================== 【你只需要改这里的坐标/路径】 =====================
        self.qr_pos          = (0.858919, -0.186457, 0.0)        # 二维码点
        self.track_start     = (-0.0497308, 0.117271, 2.54)     # 赛道起点
        # 配置图生文脚本路径（必填：替换为你的实际路径）
        self.img2text_script = "/home/ubuntu/your_ws/src/your_package/rdk_volc_ark_img2text.py"  # 示例路径
        self.img2text_pid    = None  # 记录图生文进程ID，用于后续清理

        # 规定路线：顺时针 / 逆时针 去 A/B/C 的完整路线（中间航点 + 终点）
        self.route = {
            "A": {
                "顺时针": [
                    (0.1, 0.2, 1.57),
                    (0.3, 0.4, 1.57),
                    (0.5, 0.5, 1.57),
                    (0.6, 0.6, 1.57),   # 最后到A点
                ],
                "逆时针": [
                    (1.0, 0.2, 1.57),
                    (0.9, 0.4, 1.57),
                    (0.7, 0.6, 1.57),
                    (0.6, 0.6, 1.57),   # 最后到A点
                ]
            },
            "B": {
                "顺时针": [
                    (0.2, 1.0, 1.57),
                    (0.4, 1.1, 1.57),
                    (0.6, 1.2, 1.57),
                ],
                "逆时针": [
                    (1.2, 1.0, 1.57),
                    (1.0, 1.1, 1.57),
                    (0.6, 1.2, 1.57),
                ]
            },
            "C": {
                "顺时针": [
                    (0.2, 1.5, 1.57),
                    (0.4, 1.6, 1.57),
                    (0.6, 1.7, 1.57),
                ],
                "逆时针": [
                    (1.2, 1.5, 1.57),
                    (1.0, 1.6, 1.57),
                    (0.6, 1.7, 1.57),
                ]
            }
        }
        # =================================================================

        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        # 状态
        self.arrived_qr       = False
        self.arrived_track    = False
        self.recog_result     = ""  # A/B/C
        self.direction        = ""  # 顺时针/逆时针
        self.task_running     = False

        # 订阅
        self.create_subscription(String, "/object_recognition_result", self.recog_cb, 10)
        self.create_subscription(String, "/qr_direction",        self.dir_cb,  10)

        # 启动主流程
        threading.Thread(target=self.main_flow, daemon=True).start()

    # ===================== 主流程（新增启动图生文逻辑） =====================
    def main_flow(self):
        self.get_logger().info("1. 导航到二维码点")
        self.go_to(self.qr_pos)
        self.arrived_qr = True

        self.get_logger().info("2. 到达二维码位置，启动二维码解析 + 图生文程序...")
        # 启动二维码解析
        subprocess.Popen("ros2 launch qr_code_detection qr_code_detection.launch.py", shell=True)
        # 启动图生文程序（核心新增逻辑）
        self._start_img2text_script()
        
        self.get_logger().info("3. 等待图生文结果（A/B/C）...")
        while not self.recog_result: time.sleep(0.5)

        self.get_logger().info(f"4. 收到识别结果：{self.recog_result}，前往赛道起点")
        self.go_to(self.track_start)
        self.arrived_track = True

        self.get_logger().info("5. 等待方向（顺时针/逆时针）...")
        while not self.direction: time.sleep(0.5)

        self.get_logger().info(f"6. 方向：{self.direction} → 开始跑规定路线去 {self.recog_result}")
        self.run_route()

    # ===================== 新增：启动图生文脚本 =====================
    def _start_img2text_script(self):
        """启动图生文程序，包含路径校验、进程管理、异常处理"""
        # 1. 校验脚本路径是否存在
        if not os.path.exists(self.img2text_script):
            self.get_logger().error(f"❌ 图生文脚本不存在：{self.img2text_script}，请检查路径！")
            return
        
        # 2. 先清理可能残留的同名进程（避免重复启动）
        try:
            subprocess.run(
                f"pkill -9 -f rdk_volc_ark_img2text.py",
                shell=True,
                capture_output=True,
                timeout=3
            )
            self.get_logger().info("✅ 清理残留的图生文进程完成")
        except Exception as e:
            self.get_logger().warn(f"⚠️ 清理残留进程失败（无残留）：{e}")
        
        # 3. 启动图生文脚本（后台运行，记录PID）
        try:
            # 用python3启动，指定脚本路径，输出日志到临时文件（方便调试）
            cmd = f"python3 {self.img2text_script} > /tmp/img2text.log 2>&1 &"
            self.img2text_pid = subprocess.Popen(cmd, shell=True).pid
            self.get_logger().info(f"✅ 图生文程序启动成功，PID={self.img2text_pid}")
            # 等待1秒确保程序加载完成
            time.sleep(1)
        except Exception as e:
            self.get_logger().error(f"❌ 启动图生文程序失败：{e}")

    # ===================== 话题回调 =====================
    def recog_cb(self, msg):
        if not self.arrived_qr: return
        res = msg.data.strip().upper()
        if res in ["A","B","C"]:
            self.recog_result = res
            self.get_logger().info(f"📌 图生文结果：{res}")

    def dir_cb(self, msg):
        if not self.arrived_track: return
        d = msg.data.strip()
        if "顺时针" in d:
            self.direction = "顺时针"
            self.get_logger().info("📌 方向：顺时针")
        elif "逆时针" in d:
            self.direction = "逆时针"
            self.get_logger().info("📌 方向：逆时针")

    # ===================== 导航 =====================
    def go_to(self, point):
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("等待nav2...")

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = point[0]
        goal.pose.pose.position.y = point[1]
        q = self.yaw2quat(point[2])
        goal.pose.pose.orientation.x = q[0]
        goal.pose.pose.orientation.y = q[1]
        goal.pose.pose.orientation.z = q[2]
        goal.pose.pose.orientation.w = q[3]

        future = self.nav_client.send_goal_async(goal)
        while rclpy.ok() and not future.done(): time.sleep(0.1)
        res = future.result().get_result_async()
        while rclpy.ok() and not res.done(): time.sleep(0.1)

    def run_route(self):
        if self.task_running: return
        self.task_running = True
        waypoints = self.route[self.recog_result][self.direction]
        for i, pt in enumerate(waypoints):
            self.get_logger().info(f"➡️ 航点 {i+1}/{len(waypoints)}")
            self.go_to(pt)
        self.get_logger().info(f"🎉 到达终点：{self.recog_result}")
        self.task_running = False

    def yaw2quat(self, yaw):
        if SCIPY:
            return R.from_euler('z', yaw).as_quat()
        else:
            cy = math.cos(yaw*0.5)
            sy = math.sin(yaw*0.5)
            return [0.0, 0.0, sy, cy]

    # ===================== 新增：清理进程（程序退出时） =====================
    def _cleanup_processes(self):
        """清理启动的二维码解析、图生文进程"""
        # 清理图生文进程
        if self.img2text_pid:
            try:
                subprocess.run(f"kill -9 {self.img2text_pid}", shell=True, timeout=3)
                self.get_logger().info(f"✅ 已终止图生文进程（PID={self.img2text_pid}）")
            except Exception as e:
                self.get_logger().warn(f"⚠️ 终止图生文进程失败：{e}")
        # 清理二维码解析进程
        try:
            subprocess.run("pkill -9 -f qr_code_detection", shell=True, timeout=3)
            self.get_logger().info("✅ 已终止二维码解析进程")
        except Exception as e:
            self.get_logger().warn(f"⚠️ 终止二维码解析进程失败：{e}")

# ===================== 主函数（新增进程清理） =====================
def main(args=None):
    rclpy.init(args=args)
    node = NavCarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 手动终止节点，清理进程...")
        node._cleanup_processes()
    except Exception as e:
        node.get_logger().error(f"❌ 节点异常退出：{e}")
        node._cleanup_processes()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()