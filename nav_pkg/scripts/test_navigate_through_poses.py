#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_to_quat(yaw: float):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class ThroughPosesTester(Node):
    def __init__(self):
        super().__init__("through_poses_tester")
        self.client = ActionClient(self, NavigateThroughPoses, "/navigate_through_poses")

    def make_pose(self, x: float, y: float, yaw: float = 0.0) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quat(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def run(self):
        poses = [
            self.make_pose(1.970198392868042, 0.015225127339363098),
            self.make_pose(0.8648350834846497, -0.4261478781700134),
            self.make_pose(0.17267444729804993, 0.044627413153648376),
        ]

        self.get_logger().info("Waiting for /navigate_through_poses action server...")
        while not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("navigate_through_poses server not available yet")

        goal = NavigateThroughPoses.Goal()
        goal.poses = poses

        self.get_logger().info("Sending NavigateThroughPoses test goal with 3 poses")
        send_future = self.client.send_goal_async(goal)
        while rclpy.ok() and not send_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("NavigateThroughPoses goal rejected")
            return 1

        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.02)

        result = result_future.result()
        if result is None:
            self.get_logger().error("No result returned")
            return 2

        self.get_logger().info(f"Finished with status={result.status}")
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = ThroughPosesTester()
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
