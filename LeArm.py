#!/usr/bin/env python3

from dataclasses import dataclass
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


@dataclass
class TopicState:
    name: str
    stamp_ns: Optional[int] = None
    arrival_ns: Optional[int] = None
    arrival_hz: float = 0.0
    stamp_hz: float = 0.0
    min_gap_ms: Optional[float] = None
    max_gap_ms: Optional[float] = None

    def update(self, stamp_ns: int, arrival_ns: int) -> None:
        if self.arrival_ns is not None:
            arrival_gap_ns = arrival_ns - self.arrival_ns
            if arrival_gap_ns > 0:
                arrival_gap_ms = arrival_gap_ns / 1_000_000.0
                inst_hz = 1_000_000_000.0 / arrival_gap_ns
                self.arrival_hz = inst_hz if self.arrival_hz == 0.0 else (
                    0.8 * self.arrival_hz + 0.2 * inst_hz
                )
                self.min_gap_ms = arrival_gap_ms if self.min_gap_ms is None else (
                    min(self.min_gap_ms, arrival_gap_ms)
                )
                self.max_gap_ms = arrival_gap_ms if self.max_gap_ms is None else (
                    max(self.max_gap_ms, arrival_gap_ms)
                )

        if self.stamp_ns is not None:
            stamp_gap_ns = stamp_ns - self.stamp_ns
            if stamp_gap_ns > 0:
                inst_hz = 1_000_000_000.0 / stamp_gap_ns
                self.stamp_hz = inst_hz if self.stamp_hz == 0.0 else (
                    0.8 * self.stamp_hz + 0.2 * inst_hz
                )

        self.stamp_ns = stamp_ns
        self.arrival_ns = arrival_ns


class EkfInputTimeProbe(Node):
    def __init__(self) -> None:
        super().__init__("ekf_input_time_probe")

        self.odom_raw_topic = "/odom_raw"
        self.imu_topic = "/imu660ra/data_raw"
        self.odom_topic = "/odom"

        self.odom_raw = TopicState(self.odom_raw_topic)
        self.imu = TopicState(self.imu_topic)
        self.odom = TopicState(self.odom_topic)

        self.create_subscription(
            Odometry, self.odom_raw_topic, self.odom_raw_callback, qos_profile_sensor_data
        )
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos_profile_sensor_data)
        self.create_timer(0.5, self.report)

        self.get_logger().info(
            "Watching /odom_raw, /imu660ra/data_raw, /odom. "
            "No package install or colcon build is required."
        )

    def odom_raw_callback(self, msg: Odometry) -> None:
        self.odom_raw.update(stamp_to_ns(msg.header.stamp), self.now_ns())

    def imu_callback(self, msg: Imu) -> None:
        self.imu.update(stamp_to_ns(msg.header.stamp), self.now_ns())

    def odom_callback(self, msg: Odometry) -> None:
        self.odom.update(stamp_to_ns(msg.header.stamp), self.now_ns())

    def report(self) -> None:
        self.get_logger().info(format_state(self.odom_raw, self.now_ns(), 20.0))
        self.get_logger().info(format_state(self.imu, self.now_ns(), 100.0))
        self.get_logger().info(format_state(self.odom, self.now_ns(), 15.0))
        self.get_logger().info(
            "stamp delta: "
            f"imu - odom_raw = {format_delta(self.imu, self.odom_raw)}, "
            f"odom - odom_raw = {format_delta(self.odom, self.odom_raw)}, "
            f"odom - imu = {format_delta(self.odom, self.imu)}"
        )
        self.get_logger().info("-" * 90)

    def now_ns(self) -> int:
        return self.get_clock().now().nanoseconds


def stamp_to_ns(stamp) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def ns_to_ms(delta_ns: int) -> float:
    return delta_ns / 1_000_000.0


def format_state(state: TopicState, now_ns: int, expected_hz: float) -> str:
    if state.stamp_ns is None:
        return f"{state.name}: no data"

    age_ms = ns_to_ms(now_ns - state.stamp_ns)
    age_text = f"lag={age_ms:7.2f}ms" if age_ms >= 0.0 else f"future={-age_ms:6.2f}ms"

    gap_text = "gap=n/a"
    if state.min_gap_ms is not None and state.max_gap_ms is not None:
        gap_text = f"gap_min={state.min_gap_ms:6.1f}ms gap_max={state.max_gap_ms:6.1f}ms"

    warnings = []
    if age_ms > 120.0:
        warnings.append("OLD_STAMP")
    if expected_hz > 0.0 and state.arrival_hz > 0.0 and state.arrival_hz < expected_hz * 0.75:
        warnings.append("LOW_HZ")

    warn_text = "" if not warnings else " WARN:" + ",".join(warnings)
    return (
        f"{state.name}: {age_text} arrival_hz={state.arrival_hz:6.2f} "
        f"stamp_hz={state.stamp_hz:6.2f} expected={expected_hz:5.1f} "
        f"{gap_text}{warn_text}"
    )


def format_delta(newer: TopicState, older: TopicState) -> str:
    if newer.stamp_ns is None or older.stamp_ns is None:
        return "no data"

    delta_ms = ns_to_ms(newer.stamp_ns - older.stamp_ns)
    if abs(delta_ms) < 2.0:
        return f"{delta_ms:+7.2f}ms aligned"
    if delta_ms > 0.0:
        return f"{delta_ms:+7.2f}ms first_newer"
    return f"{delta_ms:+7.2f}ms first_older"


def main() -> None:
    rclpy.init()
    node = EkfInputTimeProbe()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
