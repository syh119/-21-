#!/usr/bin/env python3
"""
快速多点航点导航 - 修复版
修复：防止刚发出就触发提前切换的连锁崩溃
"""

import rclpy
import math
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener, TransformException

# ══════════════════════════════════════════════
#  配置区
# ══════════════════════════════════════════════
WAYPOINTS = [
    (-0.038, -0.62,   3.85882),
    ( 0.146, -0.878,  4.59882),
    ( 0.20,  -1.8,    3.65328),
    ( 0.62,  -0.1,    3.05847),
    ( 1.2,   -0.2,    1.94751),
    ( 1.28,  -1.92,   1.51190),
    ( 1.22,  -1.99,   1.55252),
    ( 1.1,   -3.45,   1.55252),
    ( 0.62,  -3.57,   6.13214),
    ( 0.22,  -3.47,   5.29518),
    ( 0.07,  -2.8,    4.54452),
    (-0.25,  -1.78,   5.49303),
]

LOOP_FOREVER     = False  # True = 循环巡逻
SKIP_ON_FAIL     = True   # True = 失败跳过

# 提前切换：剩余距离 < 此值时切换（米）
LOOKAHEAD_DIST   = 0.35

# 提前切换保护：goal发出后至少等这么多秒才允许提前切换
# 防止"机器人已经在目标附近"导致立刻切换
SWITCH_MIN_SEC   = 2.0

# 单点导航超时（秒）
NAV_TIMEOUT_SEC  = 90.0

# AMCL预热参数
WARMUP_SPIN_VEL  = 0.3   # rad/s
WARMUP_SPIN_SEC  = 2.0   # 秒
# ══════════════════════════════════════════════

class FastWaypointNavigator(Node):

    def __init__(self):
        super().__init__('fast_waypoint_navigator')

        self._client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)

        self._waypoints  = WAYPOINTS
        self._total      = len(WAYPOINTS)
        self._index      = 0
        self._loop_count = 0
        self._fail_count = 0

        self._current_goal_handle = None
        self._nav_start_time      = None
        self._timeout_timer       = None

        # ★ 核心修复：防连锁切换的两个锁
        self._switching    = False   # 正在切换中，忽略后续feedback
        self._goal_send_time = None  # 记录goal发出的时间

        self.get_logger().info('=' * 55)
        self.get_logger().info('  快速多点导航节点启动（修复版）')
        self.get_logger().info(f'  航点数量    : {self._total}')
        self.get_logger().info(f'  提前切换距离 : {LOOKAHEAD_DIST}m')
        self.get_logger().info(f'  切换保护时间 : {SWITCH_MIN_SEC}s')
        self.get_logger().info(f'  循环巡逻    : {LOOP_FOREVER}')
        self.get_logger().info('=' * 55)

    # ──────────────────────────────────────────
    #  AMCL预热
    # ──────────────────────────────────────────
    def _warmup_spin(self):
        self.get_logger().info(
            f'🔄 AMCL预热：原地微转 {WARMUP_SPIN_SEC}s...')
        twist = Twist()
        twist.angular.z = WARMUP_SPIN_VEL
        steps    = int(WARMUP_SPIN_SEC * 20)
        interval = 1.0 / 20.0
        for _ in range(steps):
            self._cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=interval)
        twist.angular.z = 0.0
        self._cmd_vel_pub.publish(twist)
        self.get_logger().info('✅ 预热完成')
        time.sleep(0.5)

    # ──────────────────────────────────────────
    #  等待TF
    # ──────────────────────────────────────────
    def _wait_for_tf(self, timeout=15.0):
        self.get_logger().info('⏳ 等待 map → base_footprint TF...')
        deadline = time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            try:
                self._tf_buffer.lookup_transform(
                    'map', 'base_footprint',
                    rclpy.time.Time(),
                    timeout=Duration(seconds=1.0))
                self.get_logger().info('✅ TF 就绪')
                return True
            except TransformException:
                rclpy.spin_once(self, timeout_sec=0.5)

        # 超时就预热
        self.get_logger().warn('⚠️  TF超时，启动预热...')
        self._warmup_spin()

        deadline2 = time.time() + 8.0
        while rclpy.ok() and time.time() < deadline2:
            try:
                self._tf_buffer.lookup_transform(
                    'map', 'base_footprint',
                    rclpy.time.Time(),
                    timeout=Duration(seconds=1.0))
                self.get_logger().info('✅ TF 预热后就绪')
                return True
            except TransformException:
                rclpy.spin_once(self, timeout_sec=0.5)

        self.get_logger().error('❌ TF彻底失败，检查AMCL和激光')
        return False

    # ──────────────────────────────────────────
    #  构造PoseStamped
    # ──────────────────────────────────────────
    def _make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id    = 'map'
        pose.header.stamp       = self.get_clock().now().to_msg()
        pose.pose.position.x    = float(x)
        pose.pose.position.y    = float(y)
        pose.pose.position.z    = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    # ──────────────────────────────────────────
    #  启动
    # ──────────────────────────────────────────
    def start(self):
        self.get_logger().info('⏳ 等待导航服务器...')
        if not self._client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('❌ 导航服务器未启动')
            return
        self.get_logger().info('✅ 导航服务器已连接')

        if not self._wait_for_tf(timeout=10.0):
            return

        self._send_waypoint(0)

    # ──────────────────────────────────────────
    #  发送指定航点
    # ──────────────────────────────────────────
    def _send_waypoint(self, idx):
        if idx >= self._total:
            self._on_all_done()
            return

        # 重置所有状态
        self._switching      = False
        self._index          = idx
        self._goal_send_time = time.time()  # ★ 记录发送时间

        x, y, yaw = self._waypoints[idx]
        self.get_logger().info(
            f'\n{"─"*50}\n'
            f'  📍 航点 {idx+1}/{self._total}  '
            f'({x:.2f}, {y:.2f})  '
            f'{math.degrees(yaw):.1f}°\n'
            f'{"─"*50}'
        )

        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose(x, y, yaw)

        send_future = self._client.send_goal_async(
            goal,
            feedback_callback=self._feedback_cb)
        send_future.add_done_callback(self._goal_response_cb)

    # ──────────────────────────────────────────
    #  goal接受回调
    # ──────────────────────────────────────────
    def _goal_response_cb(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(
                f'❌ 航点 {self._index+1} 被拒绝，跳过')
            self._fail_count += 1
            self._send_waypoint(self._index + 1)
            return

        self.get_logger().info(f'✅ 航点 {self._index+1} 出发！')
        self._current_goal_handle = goal_handle
        self._nav_start_time      = time.time()

        self._cancel_timeout_timer()
        self._timeout_timer = self.create_timer(
            1.0, self._check_timeout)

        goal_handle.get_result_async().add_done_callback(
            self._result_cb)

    # ──────────────────────────────────────────
    #  实时反馈（含提前切换保护）
    # ──────────────────────────────────────────
    def _feedback_cb(self, feedback_msg):
        # 正在切换中，忽略旧goal的feedback
        if self._switching:
            return

        dist    = feedback_msg.feedback.distance_remaining
        elapsed = time.time() - (self._nav_start_time or time.time())

        self.get_logger().info(
            f'📊 [{self._index+1}/{self._total}] '
            f'剩余: {dist:.2f}m  已用: {elapsed:.0f}s',
            throttle_duration_sec=1.5
        )

        # ★ 提前切换：必须同时满足三个条件
        time_since_send = time.time() - (self._goal_send_time or 0)
        is_last = (self._index + 1 >= self._total)

        if (dist < LOOKAHEAD_DIST          # 条件1：距离够近
                and time_since_send > SWITCH_MIN_SEC  # 条件2：发出超过2秒
                and not is_last):          # 条件3：不是最后一个点
            self._switching = True
            self.get_logger().info(
                f'⚡ [{elapsed:.1f}s] 距离{dist:.2f}m，'
                f'提前切换到航点 {self._index+2}')
            self._cancel_timeout_timer()
            self._send_waypoint(self._index + 1)

    # ──────────────────────────────────────────
    #  超时检查
    # ──────────────────────────────────────────
    def _check_timeout(self):
        if self._switching or self._nav_start_time is None:
            self._cancel_timeout_timer()
            return
        elapsed = time.time() - self._nav_start_time
        if elapsed > NAV_TIMEOUT_SEC:
            self.get_logger().warn(
                f'⏰ 航点 {self._index+1} 超时，强制取消')
            self._cancel_timeout_timer()
            if self._current_goal_handle:
                self._current_goal_handle.cancel_goal_async()

    def _cancel_timeout_timer(self):
        if self._timeout_timer:
            self._timeout_timer.cancel()
            self._timeout_timer = None

    # ──────────────────────────────────────────
    #  导航结果回调
    # ──────────────────────────────────────────
    def _result_cb(self, future):
        # 提前切换后，旧goal结果直接忽略
        if self._switching:
            return

        self._cancel_timeout_timer()
        self._current_goal_handle = None
        self._nav_start_time      = None

        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'🎉 航点 {self._index+1} 精确到达')
            self._send_waypoint(self._index + 1)

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(
                f'⏰ 航点 {self._index+1} 超时取消，跳过')
            self._fail_count += 1
            self._send_waypoint(self._index + 1)

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(
                f'❌ 航点 {self._index+1} 失败(ABORTED)')
            self._fail_count += 1
            if SKIP_ON_FAIL:
                self._send_waypoint(self._index + 1)
            else:
                self.get_logger().error('🛑 任务终止')
                rclpy.shutdown()

        else:
            self.get_logger().warn(
                f'❓ 未知状态{status}，跳过')
            self._fail_count += 1
            self._send_waypoint(self._index + 1)

    # ──────────────────────────────────────────
    #  全部完成
    # ──────────────────────────────────────────
    def _on_all_done(self):
        self._loop_count += 1
        self.get_logger().info(
            f'\n{"═"*55}\n'
            f'  🏆 第{self._loop_count}圈完成！\n'
            f'  成功: {self._total - self._fail_count}/{self._total}\n'
            f'  失败: {self._fail_count}/{self._total}\n'
            f'{"═"*55}'
        )
        if LOOP_FOREVER:
            self.get_logger().info('🔄 开始下一圈...')
            self._index      = 0
            self._fail_count = 0
            self._send_waypoint(0)
        else:
            rclpy.shutdown()

# ══════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = FastWaypointNavigator()
    node.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('⚠️  用户中断')
    finally:
        # 安全停车：context可能已经关闭，要判断
        try:
            if node._cmd_vel_pub and rclpy.ok():
                node._cmd_vel_pub.publish(Twist())
        except Exception:
            pass

if __name__ == '__main__':
    main()