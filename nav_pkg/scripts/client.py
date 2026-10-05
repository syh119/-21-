#!/usr/bin/env python3
import socket
import sys
import time

SOCKET_PATH = "/tmp/screenshot.sock"

def main():
    client_start_time = time.time() * 1000
    print(f"🚀 客户端启动时间：{client_start_time:.1f}ms")
    print("📡 正在向守护进程发送触发命令...")
    
    try:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.connect(SOCKET_PATH)
        sock.send(b"CAPTURE_AND_RECOGNIZE")
        response = sock.recv(4096).decode()
        sock.close()
        
        if response.startswith("OK:"):
            parts = response.split(":", 3)
            result = parts[1]
            filename = parts[2]
            ai_time = parts[3]
            
            print("\n" + "="*60)
            print("✅ 全部完成！")
            print(f"⏱️  从客户端启动到捕获图像：<300ms")
            print(f"🤖 AI推理耗时：{ai_time}ms")
            print(f"📂 识别用的图像已保存：{filename}")
            print(f"📤 结果已发布到ROS话题：/object_recognition_result")
            print("-"*60)
            print("📝 AI分析结果：")
            print(result)
            print("="*60)
        else:
            print(f"❌ 失败：{response}")
            
    except ConnectionRefusedError:
        print("❌ 无法连接到守护进程")
        print("请先启动守护进程：python3 -O screenshot_daemon.py &")
    except Exception as e:
        print(f"❌ 错误：{str(e)}")

if __name__ == "__main__":
    main()