#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import os
import re
import subprocess
import threading
import time
from typing import Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_virtual_layer.srv import AddLine, RemoveShape
from origincar_msg.msg import Sign
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


Waypoint = Tuple[float, float, float]
Point2D = Tuple[float, float]

SCRIPT_FOLDER = os.path.dirname(os.path.abspath(__file__))
DAEMON_SCRIPT_PATH = os.path.join(SCRIPT_FOLDER, "screenshot_daemon.py")
CLIENT_SCRIPT_PATH = os.path.join(SCRIPT_FOLDER, "client.py")

SOCKET_PATH = "/tmp/screenshot.sock"
DAEMON_STARTUP_WAIT = 1.0
CLIENT_TIMEOUT = 15.0
ROBOT_STABILIZE_DELAY = 0.2
SECOND_POINT_PHOTO_DELAY = 0.2
CLOCKWISE_TEXT = "\u987a\u65f6\u9488"
COUNTERCLOCKWISE_TEXT = "\u9006\u65f6\u9488"
FIRST_WALL_IDENTIFIER = "9101"
SECOND_WALL_IDENTIFIER = "9102"
PARALLEL_FIRST_WALL_IDENTIFIER = "9103"


def yaw_to_quat(yaw: float):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def read_output(process, logger, prefix):
    try:
        for line in iter(process.stdout.readline, ""):
            if line:
                logger.info(f"[{prefix}] {line.strip()}")
        for line in iter(process.stderr.readline, ""):
            if line:
                logger.error(f"[{prefix} ERROR] {line.strip()}")
        process.wait()
        logger.info(f"[{prefix}] Process exited with code {process.returncode}")
    except Exception as exc:
        logger.error(f"[{prefix} THREAD] Error: {exc}")


class ClockwiseGateGoalTester(Node):
    def __init__(self):
        super().__init__("clockwise_gate_goal_tester")

        self.start_time = time.time()
        self.first_pose_time = 0.0

        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.add_line_client = self.create_client(
            AddLine, "/global_costmap/virtual_layer/add_line"
        )
        self.remove_shape_client = self.create_client(
            RemoveShape, "/global_costmap/virtual_layer/remove_shape"
        )

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.pose_callback, 10
        )
        self.sign_sub = self.create_subscription(
            Sign, "/sign_switch", self.sign_callback, 10
        )
        self.qr_text_sub = self.create_subscription(
            String, "/qr_display_text", self.qr_text_callback, 10
        )

        self.current_pose: Point2D | None = None
        self.goal_reach_radius = 0.35
        self.final_goal_reach_radius = 0.1
        self.virtual_wall_settle_sec = 0.0
        self.post_qr_first_wall_delay_sec = 0.1
        self.second_wall_settle_sec = 0.2

        self.first_wall_uuid = ""
        self.first_wall_shape_id = -1
        self.parallel_first_wall_uuid = ""
        self.parallel_first_wall_shape_id = -1
        self.second_wall_uuid = ""
        self.second_wall_shape_id = -1

        self.qr_wait_enabled = False
        self.qr_received = False
        self.qr_sign_data: int | None = None
        self.qr_display_text = ""
        self.route_direction = "clockwise"
        self.route_direction_label = CLOCKWISE_TEXT

        self.daemon_process: subprocess.Popen | None = None
        self.daemon_thread: threading.Thread | None = None
        self.client_process: subprocess.Popen | None = None
        self.client_thread: threading.Thread | None = None
        self.image_recognition_async = False

        # 如果你希望脚本自动拉起二维码识别，把这里改成 True。
        # 默认 False，表示外部自己启动：
        # ros2 launch qr_code_detection qr_code_detection.launch.py
        self.auto_start_qr_detection = False
        self.qr_process: subprocess.Popen | None = None
        self.qr_thread: threading.Thread | None = None

        # 三个导航点，坐标由你自己填
        # 第一段虚拟墙：第一个点扫码成功后创建，再发第二个点
        # 与第一段虚拟墙同时出现/消失的附加虚拟墙，坐标后续自行填写
        # 第二段虚拟墙：第二个点拍照完成后创建，再发第三个点

        self.clockwise_first_goal: Waypoint = (2.412,-1.093,-0.029)
        self.clockwise_second_goal: Waypoint = (4.925, 1.219, -0.785)
        self.shared_third_goal: Waypoint = (0.392,-0.078,3.092)
        self.clockwise_first_wall_p1_xy: Point2D = (2.902, 2.335)
        self.clockwise_first_wall_p2_xy: Point2D = (5.880, -0.108)
        self.clockwise_parallel_first_wall_p1_xy: Point2D = (2.902, 2.335)
        self.clockwise_parallel_first_wall_p2_xy: Point2D = (5.880, -0.108)
        self.clockwise_second_wall_p1_xy: Point2D = (4.935, 0.616)
        self.clockwise_second_wall_p2_xy: Point2D = (1.753, 3.079)

        self.counterclockwise_first_goal: Waypoint = (2.412,-1.093,-0.029)
        self.counterclockwise_second_goal: Waypoint = (3.393, 2.510, 2.451)
        self.counterclockwise_first_wall_p1_xy: Point2D = (4.935, 0.616)
        self.counterclockwise_first_wall_p2_xy: Point2D = (1.753, 3.079)
        self.counterclockwise_parallel_first_wall_p1_xy: Point2D = (4.935, 0.616)
        self.counterclockwise_parallel_first_wall_p2_xy: Point2D = (1.753, 3.079)
        self.counterclockwise_second_wall_p1_xy: Point2D = (2.902, 2.335)
        self.counterclockwise_second_wall_p2_xy: Point2D = (5.880, -0.108)

        self.route_configs = {
            "clockwise": {
                "label": CLOCKWISE_TEXT,
                "goals": (
                    self.clockwise_first_goal,
                    self.clockwise_second_goal,
                    self.shared_third_goal,
                ),
                "first_wall": (
                    self.clockwise_first_wall_p1_xy,
                    self.clockwise_first_wall_p2_xy,
                ),
                "parallel_first_wall": (
                    self.clockwise_parallel_first_wall_p1_xy,
                    self.clockwise_parallel_first_wall_p2_xy,
                ),
                "second_wall": (
                    self.clockwise_second_wall_p1_xy,
                    self.clockwise_second_wall_p2_xy,
                ),
            },
            "counterclockwise": {
                "label": COUNTERCLOCKWISE_TEXT,
                "goals": (
                    self.counterclockwise_first_goal,
                    self.counterclockwise_second_goal,
                    self.shared_third_goal,
                ),
                "first_wall": (
                    self.counterclockwise_first_wall_p1_xy,
                    self.counterclockwise_first_wall_p2_xy,
                ),
                "parallel_first_wall": (
                    self.counterclockwise_parallel_first_wall_p1_xy,
                    self.counterclockwise_parallel_first_wall_p2_xy,
                ),
                "second_wall": (
                    self.counterclockwise_second_wall_p1_xy,
                    self.counterclockwise_second_wall_p2_xy,
                ),
            },
        }
        self.active_goals = self.route_configs["clockwise"]["goals"]
        self.active_first_wall = self.route_configs["clockwise"]["first_wall"]
        self.active_parallel_first_wall = self.route_configs["clockwise"][
            "parallel_first_wall"
        ]
        self.active_second_wall = self.route_configs["clockwise"]["second_wall"]

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        if self.current_pose is None:
            self.first_pose_time = time.time()
            elapsed = self.first_pose_time - self.start_time
            self.get_logger().info(f"First AMCL pose received after {elapsed:.2f}s")

        self.current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )

    def sign_callback(self, msg: Sign):
        if not self.qr_wait_enabled:
            return

        self.qr_sign_data = msg.sign_data
        self.qr_received = True
        self.get_logger().info(f"Received QR result on /sign_switch: {msg.sign_data}")

    def qr_text_callback(self, msg: String):
        if not self.qr_wait_enabled:
            return

        self.qr_display_text = msg.data.strip()
        if self.qr_display_text:
            self.get_logger().info(
                f"Received QR result on /qr_display_text: {self.qr_display_text}"
            )
            self.get_logger().info(
                "QR text is recorded for reference only, waiting for numeric result from /sign_switch..."
            )

    def distance(self, a: Point2D, b: Point2D) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def wait_for_pose(self):
        self.get_logger().info("Waiting for /amcl_pose...")
        while rclpy.ok() and self.current_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_for_services(self):
        self.get_logger().info("Waiting for virtual layer services...")
        while not self.add_line_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("add_line service not available yet")
        while not self.remove_shape_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("remove_shape service not available yet")

    def wait_for_nav_server(self):
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Waiting for /navigate_to_pose...")

    def reset_qr_state(self):
        self.qr_received = False
        self.qr_sign_data = None
        self.qr_display_text = ""

    def start_qr_wait(self, message: str):
        self.reset_qr_state()
        self.qr_wait_enabled = True
        self.get_logger().info(message)

    def finish_qr_wait(self):
        self.qr_wait_enabled = False
        self.get_logger().info(
            f"QR completed, sign_data={self.qr_sign_data}, text='{self.qr_display_text}'"
        )

    def extract_qr_number_from_text(self) -> int | None:
        text = self.qr_display_text.strip()
        if not text:
            return None

        match = re.search(r"-?\d+", text)
        if not match:
            return None

        return int(match.group())

    def resolve_route_direction_from_qr(self) -> str:
        qr_number = self.qr_sign_data
        if qr_number is not None:
            direction = "clockwise" if (qr_number % 2) == 1 else "counterclockwise"
            direction_label = (
                CLOCKWISE_TEXT if direction == "clockwise" else COUNTERCLOCKWISE_TEXT
            )
            self.get_logger().info(
                f"QR numeric payload {qr_number} -> {direction_label} route"
            )
            return direction

        text = self.qr_display_text.strip()
        if CLOCKWISE_TEXT in text:
            self.get_logger().info("QR text indicates clockwise route")
            return "clockwise"
        if COUNTERCLOCKWISE_TEXT in text:
            self.get_logger().info("QR text indicates counterclockwise route")
            return "counterclockwise"

        qr_number = self.extract_qr_number_from_text()
        if qr_number is not None:
            self.get_logger().info(
                f"Parsed QR numeric payload from text: {qr_number}"
            )

            direction = "clockwise" if (qr_number % 2) == 1 else "counterclockwise"
            direction_label = (
                CLOCKWISE_TEXT if direction == "clockwise" else COUNTERCLOCKWISE_TEXT
            )
            self.get_logger().info(
                f"QR text numeric payload {qr_number} -> {direction_label} route"
            )
            return direction

        self.get_logger().warn(
            "Unable to determine route direction from QR result, defaulting to clockwise"
        )
        return "clockwise"

    def activate_route_from_qr(self):
        self.route_direction = self.resolve_route_direction_from_qr()
        config = self.route_configs[self.route_direction]
        self.route_direction_label = config["label"]
        self.active_goals = config["goals"]
        self.active_first_wall = config["first_wall"]
        self.active_parallel_first_wall = config["parallel_first_wall"]
        self.active_second_wall = config["second_wall"]

        first_goal, second_goal, third_goal = self.active_goals
        self.get_logger().info(
            "Activated route %s: P1=(%.3f, %.3f) P2=(%.3f, %.3f) P3=(%.3f, %.3f)"
            % (
                self.route_direction_label,
                first_goal[0],
                first_goal[1],
                second_goal[0],
                second_goal[1],
                third_goal[0],
                third_goal[1],
            )
        )

    def wait_for_enter_to_start(self):
        if self.current_pose is not None:
            x, y = self.current_pose
            self.get_logger().info(
                f"AMCL current pose: x={x:.3f}, y={y:.3f}"
            )

        self.get_logger().info(
            "All prerequisites ready. Adjust AMCL initial pose if needed, then press Enter to start navigation."
        )

        start_event = threading.Event()

        def wait_for_input():
            try:
                input()
            except EOFError:
                self.get_logger().warn(
                    "stdin is not available, starting navigation immediately"
                )
            finally:
                start_event.set()

        threading.Thread(target=wait_for_input, daemon=True).start()

        while rclpy.ok() and not start_event.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)

    def start_qr_detection_if_needed(self):
        if not self.auto_start_qr_detection:
            self.get_logger().info(
                "QR detection is expected to be started externally: "
                "ros2 launch qr_code_detection qr_code_detection.launch.py"
            )
            return True

        if self.qr_process and self.qr_process.poll() is None:
            return True

        self.get_logger().info("Starting qr_code_detection launch...")
        try:
            self.qr_process = subprocess.Popen(
                ["ros2", "launch", "qr_code_detection", "qr_code_detection.launch.py"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                universal_newlines=True,
            )
            self.qr_thread = threading.Thread(
                target=read_output,
                args=(self.qr_process, self.get_logger(), "QR"),
                daemon=True,
            )
            self.qr_thread.start()
            time.sleep(2.0)
            if self.qr_process.poll() is not None:
                self.get_logger().error("qr_code_detection launch exited immediately")
                return False
            return True
        except Exception as exc:
            self.get_logger().error(f"Failed to start qr_code_detection launch: {exc}")
            return False

    def add_virtual_line(self, p1: Point2D, p2: Point2D, identifier: str, label: str):
        req = AddLine.Request()
        req.x1 = p1[0]
        req.y1 = p1[1]
        req.x2 = p2[0]
        req.y2 = p2[1]
        req.thickness = 0.08
        req.frame_id = "map"
        req.cost_level = 254
        req.duration = -1.0
        req.identifier = identifier

        self.get_logger().info(f"Adding {label}...")
        future = self.add_line_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        resp = future.result()
        if resp is None or not resp.success:
            self.get_logger().error(f"Failed to add {label}")
            return None, None

        self.get_logger().info(f"{label} added successfully")
        if self.virtual_wall_settle_sec > 0:
            time.sleep(self.virtual_wall_settle_sec)
        return resp.uuid, resp.shape_id

    def remove_virtual_line(self, uuid: str, shape_id: int, label: str):
        candidates = []
        if uuid:
            candidates.append(uuid)
        if shape_id >= 0:
            candidates.append(str(shape_id))
        if not candidates:
            return True

        for identifier in candidates:
            req = RemoveShape.Request()
            req.identifier = identifier
            future = self.remove_shape_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            resp = future.result()
            if resp is not None and resp.success:
                self.get_logger().info(f"{label} removed successfully")
                if self.virtual_wall_settle_sec > 0:
                    time.sleep(self.virtual_wall_settle_sec)
                return True
            self.get_logger().warn(f"Failed to remove {label} by identifier {identifier}")
        return False

    def clear_virtual_line_by_identifier(self, identifier: str, label: str):
        req = RemoveShape.Request()
        req.identifier = identifier
        future = self.remove_shape_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        resp = future.result()
        if resp is not None and resp.success:
            self.get_logger().info(f"Cleared stale {label} by identifier {identifier}")
            if self.virtual_wall_settle_sec > 0:
                time.sleep(self.virtual_wall_settle_sec)
            return True
        self.get_logger().info(f"No stale {label} found for identifier {identifier}")
        return False

    def clear_first_wall_slots(self):
        self.clear_virtual_line_by_identifier(
            FIRST_WALL_IDENTIFIER, "first virtual wall"
        )
        self.clear_virtual_line_by_identifier(
            PARALLEL_FIRST_WALL_IDENTIFIER, "parallel first virtual wall"
        )
        self.first_wall_uuid = ""
        self.first_wall_shape_id = -1
        self.parallel_first_wall_uuid = ""
        self.parallel_first_wall_shape_id = -1

    def clear_second_wall_slot(self):
        self.clear_virtual_line_by_identifier(
            SECOND_WALL_IDENTIFIER, "second virtual wall"
        )
        self.second_wall_uuid = ""
        self.second_wall_shape_id = -1

    def clear_all_wall_slots(self):
        self.clear_first_wall_slots()
        self.clear_second_wall_slot()

    def add_first_wall(self):
        self.clear_all_wall_slots()

        first_wall_p1_xy, first_wall_p2_xy = self.active_first_wall
        self.first_wall_uuid, self.first_wall_shape_id = self.add_virtual_line(
            first_wall_p1_xy,
            first_wall_p2_xy,
            FIRST_WALL_IDENTIFIER,
            f"{self.route_direction_label} first virtual wall",
        )
        if self.first_wall_uuid is None:
            return False

        parallel_first_wall_p1_xy, parallel_first_wall_p2_xy = (
            self.active_parallel_first_wall
        )
        (
            self.parallel_first_wall_uuid,
            self.parallel_first_wall_shape_id,
        ) = self.add_virtual_line(
            parallel_first_wall_p1_xy,
            parallel_first_wall_p2_xy,
            PARALLEL_FIRST_WALL_IDENTIFIER,
            f"{self.route_direction_label} parallel first virtual wall",
        )
        return self.parallel_first_wall_uuid is not None

    def remove_first_wall(self):
        primary_success = self.remove_virtual_line(
            self.first_wall_uuid, self.first_wall_shape_id, "first virtual wall"
        )
        parallel_success = self.remove_virtual_line(
            self.parallel_first_wall_uuid,
            self.parallel_first_wall_shape_id,
            "parallel first virtual wall",
        )

        if primary_success:
            self.first_wall_uuid = ""
            self.first_wall_shape_id = -1
        if parallel_success:
            self.parallel_first_wall_uuid = ""
            self.parallel_first_wall_shape_id = -1
        return primary_success and parallel_success

    def add_second_wall(self):
        self.clear_first_wall_slots()
        self.clear_second_wall_slot()

        second_wall_p1_xy, second_wall_p2_xy = self.active_second_wall
        self.second_wall_uuid, self.second_wall_shape_id = self.add_virtual_line(
            second_wall_p1_xy,
            second_wall_p2_xy,
            SECOND_WALL_IDENTIFIER,
            f"{self.route_direction_label} second virtual wall",
        )
        return self.second_wall_uuid is not None

    def remove_second_wall(self):
        success = self.remove_virtual_line(
            self.second_wall_uuid, self.second_wall_shape_id, "second virtual wall"
        )
        if success:
            self.second_wall_uuid = ""
            self.second_wall_shape_id = -1
        return success

    def make_pose(self, goal: Waypoint) -> PoseStamped:
        x, y, yaw = goal
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

    def send_goal(self, goal: Waypoint, label: str):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_pose(goal)
        x, y, _ = goal
        self.get_logger().info(f"Sending {label}: x={x:.3f}, y={y:.3f}")
        future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f"{label} rejected")
            return None
        return goal_handle

    def wait_for_goal_result(
        self,
        goal_handle,
        goal_xy: Point2D,
        label: str,
        stop_on_position_only: bool = False,
        reach_radius: float | None = None,
    ):
        result_future = goal_handle.get_result_async()
        reached_logged = False
        effective_radius = (
            self.goal_reach_radius if reach_radius is None else reach_radius
        )
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.current_pose and not reached_logged:
                dist = self.distance(self.current_pose, goal_xy)
                if dist <= effective_radius:
                    self.get_logger().info(f"{label} reached within {dist:.3f}m")
                    reached_logged = True
                    if stop_on_position_only:
                        self.get_logger().info(
                            f"{label} position reached, ignoring final yaw and stopping navigation"
                        )
                        self.cancel_goal(goal_handle, label)
                        rclpy.spin_until_future_complete(
                            self, result_future, timeout_sec=2.0
                        )
                        return "position_only"

        result = result_future.result()
        if not result:
            self.get_logger().error(f"{label} finished without result")
            return None

        self.get_logger().info(f"{label} finished")
        return result

    def cancel_goal(self, goal_handle, label: str):
        self.get_logger().info(f"Cancelling {label}...")
        future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        response = future.result()
        if response is None:
            self.get_logger().warn(f"Did not receive cancel response for {label}")
            return False
        if response.goals_canceling:
            self.get_logger().info(f"{label} cancel accepted")
            return True
        self.get_logger().warn(f"{label} cancel rejected or goal already finished")
        return False

    def wait_for_goal_or_qr(self, goal_handle, goal_xy: Point2D, label: str):
        result_future = goal_handle.get_result_async()
        reached_logged = False
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            if self.qr_received:
                self.finish_qr_wait()
                if not result_future.done():
                    self.cancel_goal(goal_handle, label)
                    rclpy.spin_until_future_complete(
                        self, result_future, timeout_sec=2.0
                    )
                self.get_logger().info(
                    f"QR detected, skipping the rest of {label} and proceeding to POINT 2"
                )
                return "qr"

            if result_future.done():
                break

            if self.current_pose and not reached_logged:
                dist = self.distance(self.current_pose, goal_xy)
                if dist <= self.goal_reach_radius:
                    self.get_logger().info(f"{label} reached within {dist:.3f}m")
                    reached_logged = True

        if not rclpy.ok():
            return None

        result = result_future.result()
        if not result:
            self.get_logger().error(f"{label} finished without result")
            return None

        self.get_logger().info(f"{label} finished before QR was detected")
        return "goal"

    def wait_for_qr_result(
        self, timeout_sec: float | None = None, reset_state: bool = True
    ):
        if reset_state:
            self.start_qr_wait(
                "Waiting for numeric QR result from /sign_switch before selecting route and virtual walls..."
            )
        elif not self.qr_wait_enabled:
            self.qr_wait_enabled = True
            self.get_logger().info(
                "Continuing to wait for numeric QR result from /sign_switch..."
            )

        start_time = time.time()

        while rclpy.ok() and not self.qr_received:
            rclpy.spin_once(self, timeout_sec=0.1)
            if timeout_sec is not None and (time.time() - start_time) > timeout_sec:
                self.qr_wait_enabled = False
                self.get_logger().error("Timed out waiting for QR detection result")
                return False

        self.finish_qr_wait()
        return True

    def start_daemon(self):
        self.get_logger().info(f"Daemon path: {DAEMON_SCRIPT_PATH}")
        if not os.path.exists(DAEMON_SCRIPT_PATH):
            self.get_logger().error("Screenshot daemon not found")
            return False

        if os.path.exists(SOCKET_PATH):
            os.unlink(SOCKET_PATH)

        self.daemon_process = subprocess.Popen(
            ["python3", "-O", DAEMON_SCRIPT_PATH],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            universal_newlines=True,
        )

        self.daemon_thread = threading.Thread(
            target=read_output,
            args=(self.daemon_process, self.get_logger(), "DAEMON"),
            daemon=True,
        )
        self.daemon_thread.start()

        self.get_logger().info(f"Daemon started, PID: {self.daemon_process.pid}")
        time.sleep(DAEMON_STARTUP_WAIT)

        if self.daemon_process.poll() is not None:
            self.get_logger().error("Daemon exited immediately after startup")
            return False
        return True

    def trigger_image_recognition(self, wait_for_result: bool = True):
        if not os.path.exists(CLIENT_SCRIPT_PATH):
            self.get_logger().error("Image recognition client not found")
            return False

        if self.client_process and self.client_process.poll() is None:
            self.get_logger().warn("Image recognition client is already running")
            return False

        try:
            mode_text = "waiting for completion" if wait_for_result else "continuing immediately"
            self.get_logger().info(
                f"Triggering image recognition and {mode_text}..."
            )
            self.client_process = subprocess.Popen(
                ["python3", CLIENT_SCRIPT_PATH],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                universal_newlines=True,
            )

            self.client_thread = threading.Thread(
                target=read_output,
                args=(self.client_process, self.get_logger(), "CLIENT"),
                daemon=True,
            )
            self.client_thread.start()

            self.image_recognition_async = not wait_for_result
            if not wait_for_result:
                self.get_logger().info(
                    "Image recognition triggered, navigation will continue without waiting"
                )
                return True

            self.client_process.wait(timeout=CLIENT_TIMEOUT)
            if self.client_thread and self.client_thread.is_alive():
                self.client_thread.join(timeout=1.0)

            if self.client_process.returncode != 0:
                self.get_logger().error(
                    f"Image recognition client exited with code {self.client_process.returncode}"
                )
                return False

            self.get_logger().info("Image recognition finished")
            self.image_recognition_async = False
            return True
        except subprocess.TimeoutExpired:
            self.get_logger().error("Image recognition timed out")
            if self.client_process and self.client_process.poll() is None:
                self.client_process.kill()
            self.image_recognition_async = False
            return False
        except Exception as exc:
            self.get_logger().error(f"Failed to trigger image recognition: {exc}")
            self.image_recognition_async = False
            return False

    def wait_for_async_image_recognition(self, timeout_sec: float):
        if not self.image_recognition_async or not self.client_process:
            return

        if self.client_process.poll() is not None:
            self.image_recognition_async = False
            return

        self.get_logger().info(
            f"Waiting up to {timeout_sec:.1f}s for background image recognition to finish..."
        )
        try:
            self.client_process.wait(timeout=timeout_sec)
            if self.client_thread and self.client_thread.is_alive():
                self.client_thread.join(timeout=1.0)
            self.get_logger().info("Background image recognition finished")
        except subprocess.TimeoutExpired:
            self.get_logger().warn(
                "Background image recognition is still running during cleanup"
            )
        finally:
            if self.client_process.poll() is not None:
                self.image_recognition_async = False

    def cleanup_all(self):
        self.get_logger().info("Cleaning up resources...")

        if self.client_process and self.client_process.poll() is None:
            if self.image_recognition_async:
                self.wait_for_async_image_recognition(CLIENT_TIMEOUT)

            if self.client_process.poll() is None:
                self.get_logger().info(
                    f"Terminating image recognition client (PID: {self.client_process.pid})"
                )
                self.client_process.terminate()
                try:
                    self.client_process.wait(timeout=2)
                except Exception:
                    self.client_process.kill()

        self.remove_first_wall()
        self.remove_second_wall()

        if self.qr_process and self.qr_process.poll() is None:
            self.get_logger().info(f"Terminating QR launch (PID: {self.qr_process.pid})")
            self.qr_process.terminate()
            try:
                self.qr_process.wait(timeout=2)
            except Exception:
                self.qr_process.kill()

        if self.daemon_process and self.daemon_process.poll() is None:
            self.get_logger().info(
                f"Terminating screenshot daemon (PID: {self.daemon_process.pid})"
            )
            self.daemon_process.terminate()
            try:
                self.daemon_process.wait(timeout=2)
            except Exception:
                self.daemon_process.kill()

        if os.path.exists(SOCKET_PATH):
            os.unlink(SOCKET_PATH)

    def run(self):
        if not self.start_daemon():
            self.cleanup_all()
            return 10

        self.wait_for_pose()
        self.wait_for_services()
        self.wait_for_nav_server()

        if not self.start_qr_detection_if_needed():
            self.cleanup_all()
            return 11

        self.wait_for_enter_to_start()
        self.start_qr_wait(
            "Monitoring QR while navigating to POINT 1. If QR is detected early, navigation will switch to POINT 2 immediately."
        )

        first_goal, _, _ = self.active_goals
        first_xy = (first_goal[0], first_goal[1])

        g1 = self.send_goal(first_goal, "POINT 1")
        if not g1:
            self.cleanup_all()
            return 12
        point1_status = self.wait_for_goal_or_qr(g1, first_xy, "POINT 1")
        if point1_status is None:
            self.cleanup_all()
            return 13

        if point1_status == "goal":
            self.get_logger().info(
                f"Waiting {ROBOT_STABILIZE_DELAY}s at POINT 1 before continuing QR wait..."
            )
            time.sleep(ROBOT_STABILIZE_DELAY)
            if not self.wait_for_qr_result(reset_state=False):
                self.cleanup_all()
                return 13

        self.activate_route_from_qr()
        _, second_goal, third_goal = self.active_goals
        second_xy = (second_goal[0], second_goal[1])
        third_xy = (third_goal[0], third_goal[1])

        if not self.add_first_wall():
            self.cleanup_all()
            return 14
        if self.post_qr_first_wall_delay_sec > 0:
            self.get_logger().info(
                f"QR completed and first virtual wall is ready. Waiting {self.post_qr_first_wall_delay_sec:.2f}s before sending POINT 2..."
            )
            time.sleep(self.post_qr_first_wall_delay_sec)

        g2 = self.send_goal(second_goal, f"POINT 2 ({self.route_direction_label})")
        if not g2:
            self.cleanup_all()
            return 15
        self.wait_for_goal_result(
            g2, second_xy, f"POINT 2 ({self.route_direction_label})"
        )

        self.get_logger().info(
            f"POINT 2 reached, waiting {SECOND_POINT_PHOTO_DELAY:.1f}s before triggering photo..."
        )
        time.sleep(SECOND_POINT_PHOTO_DELAY)
        self.get_logger().info("POINT 2 photo delay complete, triggering photo and continuing immediately...")
        if not self.trigger_image_recognition(wait_for_result=False):
            self.cleanup_all()
            return 16

        if not self.remove_first_wall():
            self.get_logger().warn("Failed to remove first virtual wall, continuing anyway")

        if not self.add_second_wall():
            self.cleanup_all()
            return 17
        if self.second_wall_settle_sec > 0:
            self.get_logger().info(
                f"Waiting {self.second_wall_settle_sec:.2f}s for second virtual wall to settle before sending POINT 3..."
            )
            time.sleep(self.second_wall_settle_sec)

        g3 = self.send_goal(third_goal, f"POINT 3 ({self.route_direction_label})")
        if not g3:
            self.cleanup_all()
            return 18
        self.wait_for_goal_result(
            g3,
            third_xy,
            f"POINT 3 ({self.route_direction_label})",
        )

        total_time = time.time() - self.start_time
        self.get_logger().info(f"All navigation tasks completed in {total_time:.2f}s")
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = ClockwiseGateGoalTester()
    try:
        code = node.run()
    finally:
        node.cleanup_all()
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
