#!/usr/bin/env python3
import json
import math
import os
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, String


TRAINING_DIR = Path.home() / ".nav_pkg_training" / "clockwise_gate_goal"
PROFILE_PATH = TRAINING_DIR / "track_profile.json"
SEGMENT_NAMES = {
    0: "segment_1_pre_gate",
    1: "segment_2_post_gate",
}


def default_profile() -> Dict[str, Any]:
    return {
        "version": 1,
        "segments": {
            "segment_1_pre_gate": {
                "id": 0,
                "base_speed_mps": 1.0,
                "min_speed_mps": 0.12,
                "max_speed_mps": 1.1,
            },
            "segment_2_post_gate": {
                "id": 1,
                "base_speed_mps": 1.0,
                "min_speed_mps": 0.12,
                "max_speed_mps": 1.1,
            },
        },
        "safety": {
            "slowdown_distance_m": 1.2,
            "crawl_distance_m": 0.8,
            "stop_distance_m": 0.35,
            "crawl_speed_mps": 0.22,
            "collision_distance_m": 0.18,
        },
    }


def merge_missing_dict(data: Dict[str, Any], defaults: Dict[str, Any]) -> Dict[str, Any]:
    merged = dict(data)
    for key, value in defaults.items():
        if key not in merged:
            merged[key] = value
        elif isinstance(value, dict) and isinstance(merged[key], dict):
            merged[key] = merge_missing_dict(merged[key], value)
    return merged


def load_profile() -> Dict[str, Any]:
    TRAINING_DIR.mkdir(parents=True, exist_ok=True)
    if not PROFILE_PATH.exists():
        profile = default_profile()
        PROFILE_PATH.write_text(
            json.dumps(profile, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        return profile
    try:
        loaded = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
        if isinstance(loaded, dict):
            return merge_missing_dict(loaded, default_profile())
        return default_profile()
    except (OSError, json.JSONDecodeError):
        return default_profile()


class SpeedManagerNode(Node):
    def __init__(self):
        super().__init__("speed_manager_node")
        self.profile = load_profile()
        self.profile_mtime: Optional[float] = self.get_profile_mtime()
        self.current_segment_id = -1
        self.current_mode = "idle"
        self.front_distance_m: Optional[float] = None

        self.create_subscription(Twist, "/cmd_vel_nav", self.cmd_callback, 20)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 20)
        self.create_subscription(
            Int32, "/training/current_segment", self.segment_callback, 10
        )
        self.create_subscription(String, "/training/lap_mode", self.mode_callback, 10)
        self.create_subscription(LaserScan, "/scan_deskewed", self.scan_callback, 20)
        self.create_subscription(LaserScan, "/scan", self.scan_callback, 20)
        self.create_timer(2.0, self.reload_profile_if_needed)

        self.get_logger().info(
            f"Speed manager active, profile path: {PROFILE_PATH}"
        )

    def get_profile_mtime(self) -> Optional[float]:
        try:
            return PROFILE_PATH.stat().st_mtime
        except OSError:
            return None

    def reload_profile_if_needed(self) -> None:
        current_mtime = self.get_profile_mtime()
        if current_mtime is None or current_mtime == self.profile_mtime:
            return
        self.profile = load_profile()
        self.profile_mtime = current_mtime
        self.get_logger().info("Reloaded speed profile from disk")

    def segment_callback(self, msg: Int32) -> None:
        self.current_segment_id = msg.data

    def mode_callback(self, msg: String) -> None:
        self.current_mode = msg.data

    def scan_callback(self, msg: LaserScan) -> None:
        self.front_distance_m = self.compute_front_distance(msg)

    def compute_front_distance(self, scan: LaserScan) -> Optional[float]:
        if not scan.ranges:
            return None

        min_distance = None
        front_half_angle_rad = math.radians(20.0)
        for index, distance in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            if abs(angle) > front_half_angle_rad:
                continue
            if math.isinf(distance) or math.isnan(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue
            if min_distance is None or distance < min_distance:
                min_distance = distance
        return min_distance

    def get_segment_cap(self) -> float:
        segment_name = SEGMENT_NAMES.get(self.current_segment_id)
        if segment_name is None:
            return max(
                segment.get("max_speed_mps", 1.0)
                for segment in self.profile.get("segments", {}).values()
            )

        segment_profile = self.profile.get("segments", {}).get(segment_name, {})
        base_speed = float(segment_profile.get("base_speed_mps", 1.0))
        min_speed = float(segment_profile.get("min_speed_mps", 0.0))
        max_speed = float(segment_profile.get("max_speed_mps", base_speed))
        return max(min(base_speed, max_speed), min_speed)

    def get_risk_cap(self, segment_cap: float) -> float:
        safety = self.profile.get("safety", {})
        slowdown_distance = float(safety.get("slowdown_distance_m", 1.2))
        crawl_distance = float(safety.get("crawl_distance_m", 0.8))
        stop_distance = float(safety.get("stop_distance_m", 0.35))
        crawl_speed = float(safety.get("crawl_speed_mps", 0.22))

        if self.front_distance_m is None:
            return segment_cap
        if self.front_distance_m <= stop_distance:
            return 0.0
        if self.front_distance_m <= crawl_distance:
            return min(segment_cap, crawl_speed)
        if self.front_distance_m >= slowdown_distance:
            return segment_cap

        span = max(slowdown_distance - crawl_distance, 1e-6)
        ratio = (self.front_distance_m - crawl_distance) / span
        return crawl_speed + ratio * (segment_cap - crawl_speed)

    def cmd_callback(self, msg: Twist) -> None:
        limited = Twist()
        limited.linear.x = msg.linear.x
        limited.linear.y = msg.linear.y
        limited.linear.z = msg.linear.z
        limited.angular.x = msg.angular.x
        limited.angular.y = msg.angular.y
        limited.angular.z = msg.angular.z

        segment_cap = self.get_segment_cap()
        risk_cap = self.get_risk_cap(segment_cap)
        final_cap = min(segment_cap, risk_cap)

        original_linear_x = msg.linear.x
        sign = 1.0 if msg.linear.x >= 0.0 else -1.0
        limited.linear.x = sign * min(abs(msg.linear.x), final_cap)

        # Preserve curvature when capping speed; otherwise the car turns
        # tighter than the planner intended after we reduce only linear speed.
        if abs(original_linear_x) > 1e-6 and abs(limited.linear.x) < abs(original_linear_x):
            scale = limited.linear.x / original_linear_x
            limited.angular.z = msg.angular.z * scale

        self.cmd_pub.publish(limited)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpeedManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
