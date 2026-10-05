#!/usr/bin/env python3
import math
import time
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


Point2D = Tuple[float, float]


def yaw_to_quat(yaw: float):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class ClockwiseRollingGoalTester(Node):
    def __init__(self):
        super().__init__("clockwise_rolling_goal_tester")
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.pose_callback,
            10,
        )

        # Clockwise ordered route samples. The script keeps chasing a point ahead
        # on this loop instead of treating the nearby final point as the whole task.
        self.route: List[Point2D] = [
            (0.17267444729804993, 0.044627413153648376),
            (1.970198392868042, 0.015225127339363098),
            (0.8648350834846497, -0.4261478781700134),
        ]

        self.lookahead_steps = 1
        self.switch_radius = 0.35
        self.resend_distance = 0.25
        self.current_pose: Point2D | None = None
        self.last_goal_index: int | None = None
        self.goal_handle = None

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )

    def distance(self, a: Point2D, b: Point2D) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def nearest_route_index(self, pos: Point2D) -> int:
        return min(
            range(len(self.route)),
            key=lambda i: self.distance(pos, self.route[i]),
        )

    def choose_goal_index(self, pos: Point2D) -> int:
        nearest = self.nearest_route_index(pos)
        if self.distance(pos, self.route[nearest]) < self.switch_radius:
            nearest = (nearest + 1) % len(self.route)
        return (nearest + self.lookahead_steps - 1) % len(self.route)

    def make_goal_pose(self, x: float, y: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y

        nearest = self.nearest_route_index((x, y))
        next_idx = (nearest + 1) % len(self.route)
        yaw = math.atan2(
            self.route[next_idx][1] - y,
            self.route[next_idx][0] - x,
        )
        qx, qy, qz, qw = yaw_to_quat(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def send_goal(self, index: int):
        target = self.route[index]
        goal = NavigateToPose.Goal()
        goal.pose = self.make_goal_pose(target[0], target[1])

        self.get_logger().info(
            f"Sending rolling goal index={index} x={target[0]:.3f} y={target[1]:.3f}"
        )
        future = self.client.send_goal_async(goal)
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        self.goal_handle = future.result()
        if self.goal_handle is None or not self.goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return False

        self.last_goal_index = index
        return True

    def cancel_goal(self):
        if self.goal_handle is None:
            return
        cancel_future = self.goal_handle.cancel_goal_async()
        while rclpy.ok() and not cancel_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        self.goal_handle = None

    def run(self):
        self.get_logger().info("Waiting for /navigate_to_pose action server...")
        while not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("navigate_to_pose server not available yet")

        self.get_logger().info("Waiting for /amcl_pose...")
        while rclpy.ok() and self.current_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("Rolling goal mode started")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_pose is None:
                continue

            next_index = self.choose_goal_index(self.current_pose)
            target = self.route[next_index]

            if self.last_goal_index is None:
                if not self.send_goal(next_index):
                    return 1
            else:
                current_target = self.route[self.last_goal_index]
                target_shift = self.distance(current_target, target)
                if next_index != self.last_goal_index and target_shift > self.resend_distance:
                    self.get_logger().info(
                        f"Advance rolling target {self.last_goal_index} -> {next_index}"
                    )
                    self.cancel_goal()
                    if not self.send_goal(next_index):
                        return 1

            time.sleep(0.05)

        return 0


def main(args=None):
    rclpy.init(args=args)
    node = ClockwiseRollingGoalTester()
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
