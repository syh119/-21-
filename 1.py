#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2022, www.guyuehome.com
# Licensed under the Apache License, Version 2.0

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from origincar_msg.msg import Sign

class SingleFileMultiNav(Node):
    def __init__(self):
        super().__init__('single_file_multi_nav')
        
        # Nav2标准目标点发布者
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # 全程监听扫码节点输出
        self.create_subscription(
            Sign,
            '/sign_switch',
            self.qr_trigger_callback,
            10
        )

        # 已填入你的实际坐标
        self.waypoints = [
            # 第1个点：二维码识别位置
            [2.418 , 0.647, 41.31],
            # 第2个点：扫码完成后自动前往的目标点
            [2.629, 1.511, 98.92]
        ]

        self.current_index = 0
        self.navigation_active = False
        self.qr_processed = False  # 全局单次触发标记

        self.get_logger().info('='*60)
        self.get_logger().info('✅ 全程扫码触发导航已启动')
        self.get_logger().info('📋 运行逻辑：')
        self.get_logger().info('   1. 按回车开始导航到第1个点')
        self.get_logger().info('   2. 途中扫到二维码 → 立即转向第2个点')
        self.get_logger().info('   3. 到达后扫到二维码 → 正常前往第2个点')
        self.get_logger().info('='*60)

        # 等待用户确认初始定位
        input('\n请先在RViz2中设置初始定位，然后按回车键开始导航 >>> ')
        self.navigate_to_current()

    def yaw_to_quaternion(self, yaw_deg):
        """将yaw偏航角(度)转换为ROS2四元数"""
        yaw_rad = math.radians(yaw_deg)
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)
        return [qx, qy, qz, qw]

    def navigate_to_current(self):
        """导航到当前索引对应的目标点"""
        if self.current_index >= len(self.waypoints):
            self.get_logger().info('🎉 所有导航任务已完成！')
            return

        point = self.waypoints[self.current_index]
        x, y, yaw_deg = point
        quat = self.yaw_to_quaternion(yaw_deg)

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.x = quat[0]
        goal_msg.pose.orientation.y = quat[1]
        goal_msg.pose.orientation.z = quat[2]
        goal_msg.pose.orientation.w = quat[3]

        # Nav2发布新目标会自动取消之前的导航
        self.goal_pub.publish(goal_msg)
        
        if self.current_index == 0:
            self.get_logger().info(f'🚀 正在导航到第1个点: ({x:.2f}, {y:.2f}), 朝向: {yaw_deg:.1f}°')
            self.get_logger().info('📡 全程监听二维码信号，扫到立即跳转...')
        else:
            self.get_logger().info(f'🚀 正在导航到第2个点: ({x:.2f}, {y:.2f}), 朝向: {yaw_deg:.1f}°')
        
        self.navigation_active = True

    def qr_trigger_callback(self, msg):
        """收到扫码信号立即触发（全程有效）"""
        # 只触发一次，且仅在第1个点阶段响应
        if self.qr_processed or not self.navigation_active or self.current_index != 0:
            return

        self.qr_processed = True
        self.get_logger().info(f'📱 收到二维码识别信号，指令码: {msg.sign_data}')
        self.get_logger().info('⚡ 立即取消当前导航，直接前往第2个点...')
        
        # 直接切换到第二个点并发布目标（无等待延迟）
        self.current_index += 1
        self.navigate_to_current()

def main(args=None):
    rclpy.init(args=args)
    node = SingleFileMultiNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('👋 导航已手动停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()