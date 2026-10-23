#!/usr/bin/env python3
"""ミッション実行ノード (状態機械)。

LLM (llm_commander) が生成した JSON コマンドを /mission_cmd で受け取り、
LiDAR / IMU / オドメトリを使って /cmd_vel を 20 Hz で出力する。
LLM は「何をするか」だけを決め、制御ループには一切関与しない (安全設計)。

対応アクション (JSON):
  {"action":"follow_panel","side":"left","distance":0.9,"speed":0.5,"length":10}
  {"action":"go_straight","distance":10,"speed":0.5,"heading":"north"}   # heading 省略で現在方位を維持
  {"action":"turn","angle_deg":90}        # 相対角, 左回り正
  {"action":"stop"}
  {"action":"mow_lane","length":11,"swath":0.6,"margin":0.6,"start_side":"left"}
                                          # レーン (列の間) を左端から右端まで往復して刈る (planner.py)
  {"action":"sequence","steps":[ ... ]}   # 上記を順番に実行
"""
import json
import math
from collections import deque
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String

from .planner import plan_mow_lane
from .controllers import (HeadingController, LineFitter, PanelFollowController,
                          quat_to_euler, wrap_angle)

HEADINGS = {"east": 0.0, "north": math.pi / 2, "west": math.pi, "south": -math.pi / 2,
            "東": 0.0, "北": math.pi / 2, "西": math.pi, "南": -math.pi / 2}


class MissionExecutor(Node):
    def __init__(self) -> None:
        super().__init__("mission_executor")
        # ---- パラメータ ----
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("default_speed", 0.5)
        self.declare_parameter("default_panel_distance", 0.9)
        self.declare_parameter("turn_speed", 0.6)
        self.rate = self.get_parameter("control_rate").value
        self.default_speed = self.get_parameter("default_speed").value
        self.default_dist = self.get_parameter("default_panel_distance").value
        self.turn_speed = self.get_parameter("turn_speed").value

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(String, "mission_cmd", self.on_cmd, 10)
        self.create_subscription(LaserScan, "scan", self.on_scan, sensor_qos)
        self.create_subscription(Imu, "imu", self.on_imu, sensor_qos)
        self.create_subscription(Odometry, "odom", self.on_odom, sensor_qos)
        self.pub_vel = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub_status = self.create_publisher(String, "mission_status", 10)

        # ---- センサ状態 ----
        self.scan: Optional[LaserScan] = None
        self.yaw = 0.0
        self.pitch = 0.0
        self.odom_xy = (0.0, 0.0)
        self.have_imu = False

        # ---- 制御器 ----
        self.fitter = LineFitter()
        self.panel_ctrl = PanelFollowController(target_dist=self.default_dist)
        self.heading_ctrl = HeadingController()

        # ---- 状態機械 ----
        self.queue: deque = deque()
        self.current: Optional[Dict[str, Any]] = None
        self.state = "idle"
        self.step_start_xy = (0.0, 0.0)
        self.step_start_yaw = 0.0
        self.step_start_t = 0.0
        self.last_t = self.now()
        self.create_timer(1.0 / self.rate, self.control_loop)
        self.get_logger().info("mission_executor ready. waiting for /mission_cmd")

    def now(self) -> float:
        """ROS 時刻 [s]。use_sim_time ならシミュレーション時刻なので、RTF < 1 でも PID の dt や
        duration が実時間でずれない (WSL2 では RTF ≈ 0.25〜0.33)"""
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------ callbacks
    def on_scan(self, msg: LaserScan) -> None:
        self.scan = msg

    def on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        _, self.pitch, self.yaw = quat_to_euler(q.x, q.y, q.z, q.w)
        self.have_imu = True

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.odom_xy = (p.x, p.y)

    def on_cmd(self, msg: String) -> None:
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"invalid JSON: {e}: {msg.data}")
            return
        self.get_logger().info(f"received: {cmd}")
        if cmd.get("action") == "stop":
            self.queue.clear()
            self.finish_step("stopped by command")
            return
        steps = cmd["steps"] if cmd.get("action") == "sequence" else [cmd]
        self.queue.extend(steps)
        if self.current is None:
            self.next_step()

    # ------------------------------------------------------------ state machine
    def next_step(self) -> None:
        if not self.queue:
            self.current = None
            self.state = "idle"
            self.publish_status("idle")
            self.stop_robot()
            return
        self.current = self.queue.popleft()
        self.state = self.current.get("action", "stop")
        if self.state == "mow_lane":
            self.expand_mow_lane(self.current)
            self.next_step()
            return
        self.step_start_xy = self.odom_xy
        self.step_start_yaw = self.yaw
        self.step_start_t = self.now()
        spd = float(self.current.get("speed", self.default_speed))
        if self.state == "follow_panel":
            side = 1 if str(self.current.get("side", "left")).lower() in ("left", "左") else -1
            self.panel_ctrl = PanelFollowController(
                target_dist=float(self.current.get("distance", self.default_dist)),
                side=side, v_nominal=spd)
        elif self.state == "go_straight":
            self.heading_ctrl = HeadingController(v_nominal=spd)
            hd = self.current.get("heading")
            if isinstance(hd, str) and hd.lower() in HEADINGS:
                self.heading_ctrl.set_target(HEADINGS[hd.lower()])
            elif isinstance(hd, (int, float)):
                self.heading_ctrl.set_target(math.radians(float(hd)))
            else:
                self.heading_ctrl.set_target(self.yaw)
        elif self.state == "turn":
            pass
        else:
            self.get_logger().warn(f"unknown action {self.state}, skipping")
            self.next_step()
            return
        self.publish_status(f"start {self.state}: {self.current}")

    def expand_mow_lane(self, cmd: Dict[str, Any]) -> None:
        """LiDAR で左右の列までの距離を測り、往復の手順に展開してキューの先頭に入れる"""
        fits = {}
        if self.scan is not None:
            for side in (1, -1):
                pts = LineFitter.scan_to_points(
                    self.scan.ranges, self.scan.angle_min, self.scan.angle_increment,
                    self.scan.range_min, self.scan.range_max, side=side)
                fits[side] = self.fitter.fit(pts)
        if not fits.get(1) or not fits.get(-1):
            self.publish_status("done mow_lane: panel rows not detected on both sides")
            return
        steps = plan_mow_lane(
            fits[1].distance, fits[-1].distance,
            length=float(cmd.get("length", 11.0)), swath=float(cmd.get("swath", 0.6)),
            margin=float(cmd.get("margin", 0.6)), speed=float(cmd.get("speed", self.default_speed)),
            start_side="right" if str(cmd.get("start_side", "left")).lower() in ("right", "右") else "left")
        n_pass = sum(1 for s in steps if s["action"] == "follow_panel")
        self.publish_status(f"start mow_lane: width {fits[1].distance + fits[-1].distance:.2f} m, "
                            f"{n_pass} passes, {len(steps)} steps")
        self.queue.extendleft(reversed(steps))

    def finish_step(self, reason: str) -> None:
        self.publish_status(f"done {self.state}: {reason}")
        self.stop_robot()
        self.next_step()

    # ------------------------------------------------------------ control
    def control_loop(self) -> None:
        now = self.now()
        dt = now - self.last_t
        self.last_t = now
        if not 0.0 < dt < 0.5:  # /clock 受信前・シミュレーション一時停止・リセット直後
            dt = 1.0 / self.rate
        if self.current is None:
            return
        traveled = math.hypot(self.odom_xy[0] - self.step_start_xy[0],
                              self.odom_xy[1] - self.step_start_xy[1])
        elapsed = now - self.step_start_t
        v, w = 0.0, 0.0

        if self.state == "follow_panel":
            fit = None
            if self.scan is not None:
                pts = LineFitter.scan_to_points(
                    self.scan.ranges, self.scan.angle_min, self.scan.angle_increment,
                    self.scan.range_min, self.scan.range_max, side=self.panel_ctrl.side)
                fit = self.fitter.fit(pts)
            v, w, st = self.panel_ctrl.step(fit, dt)
            if st == "stopped":
                self.finish_step("panel lost")
                return
            length = self.current.get("length")
            if length is not None and traveled >= float(length):
                self.finish_step(f"length {length} m reached")
                return
            dur = self.current.get("duration")
            if dur is not None and elapsed >= float(dur):
                self.finish_step(f"duration {dur} s reached")
                return

        elif self.state == "go_straight":
            v, w = self.heading_ctrl.step(self.yaw, dt, self.pitch)
            dist = self.current.get("distance")
            if dist is not None and traveled >= float(dist):
                self.finish_step(f"distance {dist} m reached")
                return
            dur = self.current.get("duration")
            if dur is not None and elapsed >= float(dur):
                self.finish_step(f"duration {dur} s reached")
                return

        elif self.state == "turn":
            target = math.radians(float(self.current.get("angle_deg", 90.0)))
            turned = wrap_angle(self.yaw - self.step_start_yaw)
            # 180° 付近の wrap 対策: 進捗は符号付きで単調増加とみなす
            remaining = target - turned
            if abs(target) > math.pi * 0.95 and abs(turned) < 0.1 and elapsed > 1.0:
                remaining = target  # 一周に近い場合の保険
            if abs(remaining) < math.radians(2.0):
                self.finish_step("turn complete")
                return
            w = math.copysign(min(self.turn_speed, 0.3 + 1.5 * abs(remaining)), remaining)
            v = 0.0

        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub_vel.publish(msg)

    def stop_robot(self) -> None:
        self.pub_vel.publish(Twist())

    def publish_status(self, text: str) -> None:
        self.get_logger().info(text)
        self.pub_status.publish(String(data=text))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
