#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_from_quat(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class YawConsistencyChecker(Node):
    def __init__(self):
        super().__init__("yaw_consistency_checker")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("amcl_topic", "/amcl_pose")
        self.declare_parameter("warn_threshold_deg", 10.0)
        self.declare_parameter("print_hz", 2.0)

        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.amcl_topic = str(self.get_parameter("amcl_topic").value)
        self.warn_threshold_deg = abs(
            float(self.get_parameter("warn_threshold_deg").value)
        )
        print_hz = max(float(self.get_parameter("print_hz").value), 0.1)

        self.odom_yaw: Optional[float] = None
        self.amcl_yaw: Optional[float] = None
        self.last_odom_stamp = "none"
        self.last_amcl_stamp = "none"

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.create_subscription(
            PoseWithCovarianceStamped, self.amcl_topic, self.amcl_callback, 20
        )
        self.create_timer(1.0 / print_hz, self.report)

        self.get_logger().info(
            "Checking yaw consistency: "
            f"odom={self.odom_topic}, amcl={self.amcl_topic}, "
            f"warn_threshold={self.warn_threshold_deg:.1f} deg"
        )

    def stamp_to_text(self, stamp) -> str:
        return f"{stamp.sec}.{stamp.nanosec:09d}"

    def odom_callback(self, msg: Odometry) -> None:
        self.odom_yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.last_odom_stamp = self.stamp_to_text(msg.header.stamp)

    def amcl_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self.amcl_yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.last_amcl_stamp = self.stamp_to_text(msg.header.stamp)

    def report(self) -> None:
        if self.odom_yaw is None or self.amcl_yaw is None:
            self.get_logger().warn(
                "Waiting for both topics... "
                f"odom_received={self.odom_yaw is not None}, "
                f"amcl_received={self.amcl_yaw is not None}"
            )
            return

        diff = normalize_angle(self.amcl_yaw - self.odom_yaw)
        odom_deg = math.degrees(self.odom_yaw)
        amcl_deg = math.degrees(self.amcl_yaw)
        diff_deg = math.degrees(diff)
        text = (
            f"odom_yaw={odom_deg:7.2f} deg, "
            f"amcl_yaw={amcl_deg:7.2f} deg, "
            f"amcl-odom={diff_deg:7.2f} deg, "
            f"odom_stamp={self.last_odom_stamp}, "
            f"amcl_stamp={self.last_amcl_stamp}"
        )

        if abs(diff_deg) > self.warn_threshold_deg:
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = YawConsistencyChecker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
