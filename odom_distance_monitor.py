#!/usr/bin/env python3

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomDistanceMonitor(Node):
    def __init__(self) -> None:
        super().__init__("odom_distance_monitor")

        self.declare_parameter("odom_topic", "/odom_raw")
        self.declare_parameter("target_distance", 1.0)
        self.declare_parameter("report_interval", 0.1)

        self.odom_topic = self.get_parameter("odom_topic").value
        self.target_distance = float(self.get_parameter("target_distance").value)
        self.report_interval = float(self.get_parameter("report_interval").value)

        self.last_x = None
        self.last_y = None
        self.total_distance = 0.0
        self.last_report_mark = 0.0
        self.reached_target = False

        self.subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )

        self.get_logger().info(
            f"Monitoring {self.odom_topic}, target distance = {self.target_distance:.3f} m"
        )

    def odom_callback(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.last_x is None or self.last_y is None:
            self.last_x = x
            self.last_y = y
            self.get_logger().info(f"Start point set at x={x:.3f}, y={y:.3f}")
            return

        step_distance = math.hypot(x - self.last_x, y - self.last_y)
        self.total_distance += step_distance
        self.last_x = x
        self.last_y = y

        if self.total_distance - self.last_report_mark >= self.report_interval:
            self.last_report_mark = self.total_distance
            self.get_logger().info(
                f"Current distance: {self.total_distance:.3f} m / {self.target_distance:.3f} m"
            )

        if not self.reached_target and self.total_distance >= self.target_distance:
            self.reached_target = True
            self.get_logger().info(
                f"Reached target distance: {self.total_distance:.3f} m"
            )
            raise SystemExit


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomDistanceMonitor()

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
