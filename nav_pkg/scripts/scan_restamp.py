#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanRestamp(Node):
    def __init__(self):
        super().__init__('scan_restamp')
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.callback, 10)
        self.pub = self.create_publisher(
            LaserScan, '/scan_fixed', 10)

    def callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ScanRestamp()
    rclpy.spin(node)

if __name__ == '__main__':
    main()