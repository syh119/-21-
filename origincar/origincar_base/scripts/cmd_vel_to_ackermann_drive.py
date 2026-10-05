#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile


class CmdVel2AckermannDriveNode(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_ackermann_drive")
        self.declare_parameter("wheelbase", 0.15)
        self.declare_parameter("frame_id", "odom_combined")
        self.declare_parameter("cmd_angle_instead_rotvel", False)
        self.declare_parameter("max_steering_angle_rate", 3.0)

        self.publisher = self.create_publisher(
            AckermannDriveStamped, "/ackermann_cmd", QoSProfile(depth=10)
        )
        self.subscription = self.create_subscription(
            Twist, "cmd_vel", self.cmd_callback, QoSProfile(depth=10)
        )
        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.cmd_angle_instead_rotvel = bool(
            self.get_parameter("cmd_angle_instead_rotvel").value
        )
        self.max_steering_angle_rate = abs(
            float(self.get_parameter("max_steering_angle_rate").value)
        )
        self.last_steering_angle = 0.0
        self.last_cmd_time: Optional[rclpy.time.Time] = None

        self.get_logger().info(
            "Ackermann limiter enabled: "
            f"wheelbase={self.wheelbase:.3f} m, "
            f"max_rate={self.max_steering_angle_rate:.3f} rad/s"
        )

    def convert_trans_rot_vel_to_steering_angle(self, vel, omega):
        if omega == 0 or vel == 0:
            return 0.0
        radius = vel / omega
        return math.atan(self.wheelbase / radius)

    def limit_steering_rate(self, steering: float, now: rclpy.time.Time) -> float:
        if self.max_steering_angle_rate <= 1e-6 or self.last_cmd_time is None:
            self.last_cmd_time = now
            self.last_steering_angle = steering
            return steering

        dt = (now - self.last_cmd_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return self.last_steering_angle

        max_delta = self.max_steering_angle_rate * dt
        delta = steering - self.last_steering_angle
        if delta > max_delta:
            steering = self.last_steering_angle + max_delta
        elif delta < -max_delta:
            steering = self.last_steering_angle - max_delta

        self.last_cmd_time = now
        self.last_steering_angle = steering
        return steering

    def cmd_callback(self, data):
        vel = data.linear.x
        if self.cmd_angle_instead_rotvel:
            steering = data.angular.z
        else:
            steering = self.convert_trans_rot_vel_to_steering_angle(vel, data.angular.z)

        now = self.get_clock().now()
        steering = self.limit_steering_rate(steering, now)

        msg = AckermannDriveStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.drive.steering_angle = steering
        msg.drive.speed = vel
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = CmdVel2AckermannDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
