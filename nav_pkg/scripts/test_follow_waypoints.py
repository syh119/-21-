#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_to_quat(yaw: float):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class FollowWaypointsTester(Node):
    def __init__(self):
        super().__init__("follow_waypoints_tester")
        self.client = ActionClient(self, FollowWaypoints, "/follow_waypoints")
        self.feedback_index = -1

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

    def feedback_cb(self, msg):
        idx = msg.feedback.current_waypoint
        if idx != self.feedback_index:
            self.feedback_index = idx
            self.get_logger().info(f"Current waypoint index={idx}")

    def run(self):
        poses = [
            self.make_pose(0.8724091649055481, 0.7930363416671753),
            self.make_pose(1.9957119226455688, -0.26822590827941895),
            self.make_pose(0.873784065246582, -1.0756381750106812),
            self.make_pose(0.3687317371368408, -0.04304753616452217),
        ]

        self.get_logger().info("Waiting for /follow_waypoints action server...")
        while not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("follow_waypoints server not available yet")

        goal = FollowWaypoints.Goal()
        goal.poses = poses

        self.get_logger().info("Sending FollowWaypoints test goal with 4 poses")
        send_future = self.client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        while rclpy.ok() and not send_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("FollowWaypoints goal rejected")
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

        missed = list(result.result.missed_waypoints)
        self.get_logger().info(
            f"Finished with status={result.status}, missed_waypoints={missed}"
        )
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = FollowWaypointsTester()
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
