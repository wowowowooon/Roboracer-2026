#!/usr/bin/env python3
"""
Stanley controller waypoint follower — CSV 슬라이딩 + /local_path override.

Pure Pursuit 버전(waypoint_follow_node)과 별도 executable.
기본 CSV·TF·속도 스케일·회피 게이트 구조는 동일, 조향만 Stanley.
"""
from __future__ import annotations

import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float64, String
from tf2_ros import Buffer, TransformException, TransformListener

from path_following.track_sliding import (
    LoopTrackSliding,
    apply_track_direction,
    load_csv_xy,
    param_bool,
    resolve_csv_path,
)


# ============================================================
# USER TUNING — Stanley 경로 추종 (여기만 수정)
# ============================================================
CFG = {
    "csv_path": "",
    "reverse_track_direction": True,  # 200005 raceline: False (True면 hdg_err ~140° 반대)
    "path_window_size": 140,
    "path_anchor_half_width": 120,
    "map_frame": "map",
    "base_frame": "base_link",
    "tf_lookup_timeout_sec": 0.2,
    "local_path_topic": "/local_path",
    "planner_path_override_topic": "/planner_path_override_active",
    "planner_speed_scale_topic": "/planner/speed_scale",
    "measured_speed_topic": "/vehicle/speed_mps",
    "measured_speed_stale_sec": 0.3,
    "measured_speed_filter_alpha": 0.25,
    "drive_topic": "/drive",
    "tracked_path_topic": "/waypoint_tracked_path",
    "timer_period_ms": 30,
    # /drive.speed 는 발행하지 않음(0). 실제 속도는 control_node auto_cruise.
    "publish_drive_speed": False,
    "nominal_speed": 1.6,
    "use_planner_speed_scale": True,
    "planner_speed_stale_sec": 0.75,
    "max_drive_speed": 1.6,
    "speed_smooth_alpha": 0.2,
    "speed_slew_mps": 1.0,
    "max_steering_angle": 0.6981,  # ±40° — control_node / ESP S±1.0 과 동일
    "steering_smooth_alpha": 0.35,
    "wheelbase": 0.33,
    "stanley_k": 1.5,
    "stanley_softening": 0.12,
    # |cte|가 클수록 heading_error 가중치↓ (직선 평행주행 시 상쇄 방지)
    "stanley_heading_cte_blend_m": 0.08,
    "stanley_heading_min_weight": 0.25,
    "stanley_debug_log_hz": 2.0,
    "status_log_hz": 2.0,
    "planner_gate_stale_sec": 0.15,  # override False 미수신 시 빠르게 CSV 복귀
    "steering_rate_limit_radps": 5.0,
    "publish_tracked_path": True,
}


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def quat_to_yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def closest_point_on_segment(
    px: float,
    py: float,
    ax: float,
    ay: float,
    bx: float,
    by: float,
) -> Tuple[float, float, float]:
    abx = bx - ax
    aby = by - ay
    apx = px - ax
    apy = py - ay

    ab2 = abx * abx + aby * aby
    if ab2 < 1e-12:
        return ax, ay, 0.0

    t = (apx * abx + apy * aby) / ab2
    t = max(0.0, min(1.0, t))

    qx = ax + t * abx
    qy = ay + t * aby
    return qx, qy, t


class StanleyWaypointFollowNode(Node):
    def __init__(self):
        super().__init__("stanley_waypoint_follow_node")

        for key, value in CFG.items():
            self.declare_parameter(key, value)

        self.csv_path = resolve_csv_path(
            self.get_parameter("csv_path").get_parameter_value().string_value
        )
        if not self.csv_path:
            raise RuntimeError("stanley_waypoint_follow_node: csv_path is required.")

        self.path_window_size = int(self.get_parameter("path_window_size").value)
        self.path_anchor_half_width = int(
            self.get_parameter("path_anchor_half_width").value
        )

        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.tf_timeout = float(self.get_parameter("tf_lookup_timeout_sec").value)

        self.local_path_topic = self.get_parameter("local_path_topic").value
        self.gate_topic = self.get_parameter("planner_path_override_topic").value
        self.speed_scale_topic = self.get_parameter("planner_speed_scale_topic").value
        self.measured_speed_topic = self.get_parameter("measured_speed_topic").value
        self.drive_topic = self.get_parameter("drive_topic").value
        self.tracked_path_topic = self.get_parameter("tracked_path_topic").value

        self.timer_period = float(self.get_parameter("timer_period_ms").value) / 1000.0

        self.publish_drive_speed = param_bool(
            self.get_parameter("publish_drive_speed").value
        )
        self.nominal_speed = float(self.get_parameter("nominal_speed").value)
        _ups = self.get_parameter("use_planner_speed_scale").value
        self.use_planner_speed_scale = param_bool(_ups)
        self.planner_speed_stale_ns = int(
            float(self.get_parameter("planner_speed_stale_sec").value) * 1e9
        )
        self.max_drive_speed = float(self.get_parameter("max_drive_speed").value)
        self.speed_smooth_alpha = float(self.get_parameter("speed_smooth_alpha").value)
        self.speed_slew_mps = float(self.get_parameter("speed_slew_mps").value)
        self.measured_speed_stale_ns = int(
            float(self.get_parameter("measured_speed_stale_sec").value) * 1e9
        )
        self.measured_speed_filter_alpha = float(
            self.get_parameter("measured_speed_filter_alpha").value
        )

        self.max_steering = float(self.get_parameter("max_steering_angle").value)
        self.steering_smooth_alpha = float(
            self.get_parameter("steering_smooth_alpha").value
        )
        self.wheelbase = max(1e-3, float(self.get_parameter("wheelbase").value))

        self.stanley_k = float(self.get_parameter("stanley_k").value)
        self.stanley_softening = float(self.get_parameter("stanley_softening").value)
        self.stanley_heading_cte_blend_m = max(
            1e-3, float(self.get_parameter("stanley_heading_cte_blend_m").value)
        )
        self.stanley_heading_min_weight = max(
            0.0,
            min(1.0, float(self.get_parameter("stanley_heading_min_weight").value)),
        )

        self.gate_stale_ns = int(
            float(self.get_parameter("planner_gate_stale_sec").value) * 1e9
        )
        self.steering_rate_limit_radps = float(
            self.get_parameter("steering_rate_limit_radps").value
        )

        _ptp = self.get_parameter("publish_tracked_path").value
        self.publish_tracked_path = param_bool(_ptp)

        csv_points = load_csv_xy(self.csv_path)
        reverse_track = param_bool(
            self.get_parameter("reverse_track_direction").value
        )
        csv_points = apply_track_direction(csv_points, reverse_track)
        if len(csv_points) < 2:
            raise RuntimeError(f"CSV needs at least 2 points: {self.csv_path}")

        self.track = LoopTrackSliding(
            csv_points,
            self.path_window_size,
            self.path_anchor_half_width,
        )

        self._local_path: List[Tuple[float, float]] = []
        self._path_poses: List[Tuple[float, float]] = []

        self._planner_override_active = False
        self._planner_gate_recv_ns = 0

        self._planner_speed_scale = 1.0
        self._planner_speed_recv_ns = 0

        self._measured_speed_mps = 0.0
        self._filtered_speed_mps = 0.0
        self._measured_speed_recv_ns = 0
        self._measured_speed_initialized = False

        self._last_speed_cmd: float | None = None
        self._last_steering_cmd = 0.0
        self._last_heading_err = 0.0
        self._last_cte_term = 0.0
        dbg_hz = max(0.0, float(self.get_parameter("stanley_debug_log_hz").value))
        self._stanley_debug_period = 1.0 / dbg_hz if dbg_hz > 0.0 else 0.0
        self._stanley_debug_accum = 0.0
        status_hz = max(0.0, float(self.get_parameter("status_log_hz").value))
        self._status_log_period = 1.0 / status_hz if status_hz > 0.0 else 0.0
        self._status_log_accum = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Path, self.local_path_topic, self._cb_local_path, 10)
        self.create_subscription(Bool, self.gate_topic, self._cb_planner_gate, 10)
        self.create_subscription(
            Float64,
            self.measured_speed_topic,
            self._cb_measured_speed,
            10,
        )

        if self.use_planner_speed_scale:
            self.create_subscription(
                Float64, self.speed_scale_topic, self._cb_speed_scale, 10
            )

        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)

        # New publishers for CSV logging
        self.raw_steer_cmd_pub = self.create_publisher(Float64, "/control/raw_steer_cmd", 10)
        self.filtered_steer_cmd_pub = self.create_publisher(Float64, "/control/filtered_steer_cmd", 10)
        self.cte_pub = self.create_publisher(Float64, "/control/cross_track_error", 10)
        self.heading_error_pub = self.create_publisher(Float64, "/control/heading_error", 10)
        self.path_curvature_pub = self.create_publisher(Float64, "/control/path_curvature", 10)

        self.tracked_path_pub = None
        if self.publish_tracked_path:
            self.tracked_path_pub = self.create_publisher(
                Path, self.tracked_path_topic, 10
            )

        self.timer = self.create_timer(self.timer_period, self._timer_cb)

        self.get_logger().info(
            f"Stanley waypoint follower | CSV={self.csv_path}, "
            f"points={len(csv_points)}, reverse_track={reverse_track}, "
            f"drive={self.drive_topic}, "
            f"measured_speed={self.measured_speed_topic}, "
            f"stanley_k={self.stanley_k}, soft={self.stanley_softening}, "
            f"hdg_blend={self.stanley_heading_cte_blend_m:.2f}m, "
            f"steering_rate_limit={self.steering_rate_limit_radps:.2f} rad/s"
        )

    def _cb_local_path(self, msg: Path) -> None:
        self._local_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _cb_planner_gate(self, msg: Bool) -> None:
        self._planner_override_active = bool(msg.data)
        self._planner_gate_recv_ns = self.get_clock().now().nanoseconds

    def _cb_speed_scale(self, msg: Float64) -> None:
        self._planner_speed_scale = float(msg.data)
        self._planner_speed_recv_ns = self.get_clock().now().nanoseconds

    def _cb_measured_speed(self, msg: Float64) -> None:
        speed = float(msg.data)
        if not math.isfinite(speed):
            return

        # Stanley 분모에는 진행 방향과 무관한 속력 크기만 사용한다.
        speed = abs(speed)
        if not self._measured_speed_initialized:
            self._filtered_speed_mps = speed
            self._measured_speed_initialized = True
        else:
            alpha = max(0.0, min(1.0, self.measured_speed_filter_alpha))
            self._filtered_speed_mps += alpha * (
                speed - self._filtered_speed_mps
            )

        self._measured_speed_mps = speed
        self._measured_speed_recv_ns = self.get_clock().now().nanoseconds

    def _get_control_speed(self, fallback_speed: float) -> Tuple[float, bool]:
        now_ns = self.get_clock().now().nanoseconds
        speed_alive = (
            self._measured_speed_recv_ns > 0
            and now_ns - self._measured_speed_recv_ns
            < self.measured_speed_stale_ns
        )
        if speed_alive and self._measured_speed_initialized:
            return abs(self._filtered_speed_mps), True
        return abs(fallback_speed), False

    def _get_pose_map(self) -> Tuple[float, float, float] | None:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout),
            )
        except TransformException:
            return None

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        yaw = quat_to_yaw(tf.transform.rotation)
        return x, y, yaw

    def _select_mode_and_path(self, x: float, y: float) -> str:
        now_ns = self.get_clock().now().nanoseconds

        gate_alive = (
            self._planner_gate_recv_ns > 0
            and now_ns - self._planner_gate_recv_ns < self.gate_stale_ns
        )

        if gate_alive and self._planner_override_active:
            if len(self._local_path) >= 2:
                self._path_poses = list(self._local_path)
                return "LOCAL_PATH"

            self._path_poses = []
            return "STOP"

        self._path_poses = self.track.sliding_xy(x, y)
        return "CSV_TRACKING"

    def _target_speed(self, mode: str) -> float:
        scale = 1.0
        now_ns = self.get_clock().now().nanoseconds

        if self.use_planner_speed_scale:
            speed_alive = (
                self._planner_speed_recv_ns > 0
                and now_ns - self._planner_speed_recv_ns < self.planner_speed_stale_ns
            )

            if speed_alive:
                scale = max(0.05, min(4.0, self._planner_speed_scale))

        v = self.nominal_speed * scale
        v = min(v, self.max_drive_speed)
        return max(0.0, v)

    def _smooth_speed(self, target: float) -> float:
        if self._last_speed_cmd is None:
            self._last_speed_cmd = target
            return target

        alpha = max(0.0, min(1.0, self.speed_smooth_alpha))
        filtered = self._last_speed_cmd + alpha * (target - self._last_speed_cmd)

        max_step = max(0.0, self.speed_slew_mps) * self.timer_period
        dv = filtered - self._last_speed_cmd
        dv = max(-max_step, min(max_step, dv))

        self._last_speed_cmd += dv
        return max(0.0, self._last_speed_cmd)

    def _stanley_control(
        self,
        path: List[Tuple[float, float]],
        x: float,
        y: float,
        yaw: float,
        speed: float,
        mode: str,
    ) -> Tuple[float, float, float, float]:
        if len(path) < 2:
            return 0.0, 0.0, x, y, 0.0, 0.0

        # Ackermann 조향은 전륜 기준 — PP와 동일 wheelbase
        px = x + self.wheelbase * math.cos(yaw)
        py = y + self.wheelbase * math.sin(yaw)

        best_d2 = float("inf")
        best_i = 0
        best_qx = 0.0
        best_qy = 0.0

        for i in range(len(path) - 1):
            ax, ay = path[i]
            bx, by = path[i + 1]

            qx, qy, _ = closest_point_on_segment(px, py, ax, ay, bx, by)
            d2 = (px - qx) ** 2 + (py - qy) ** 2

            if d2 < best_d2:
                best_d2 = d2
                best_i = i
                best_qx = qx
                best_qy = qy

        ax, ay = path[best_i]
        bx, by = path[min(best_i + 1, len(path) - 1)]

        path_yaw = math.atan2(by - ay, bx - ax)

        heading_error = wrap_pi(path_yaw - yaw)

        dx = px - best_qx
        dy = py - best_qy

        # CTE>0: 경로 기준 오른쪽 (ROS map +y=좌)
        right_x = math.sin(path_yaw)
        right_y = -math.cos(path_yaw)
        cte = dx * right_x + dy * right_y

        cte_term = math.atan2(
            self.stanley_k * cte,
            abs(speed) + self.stanley_softening,
        )

        cte_abs = abs(cte)
        hdg_w = max(
            self.stanley_heading_min_weight,
            1.0 - cte_abs / self.stanley_heading_cte_blend_m,
        )
        # 조향 부호 (실차 ESP 서보 실측과 동일, 추가 반전 없음):
        #   +steering = 좌, -steering = 우
        # cte>0(경로 오른쪽) → 좌회전(+)으로 복귀
        steering = hdg_w * heading_error + cte_term

        # Stanley 조향값은 원형 각도가 아니라 bounded control input 이므로
        # wrap_pi()로 다시 감싸면 반응이 과하게 휘어질 수 있다.
        steering = max(-self.max_steering, min(self.max_steering, steering))

        return steering, cte, best_qx, best_qy, heading_error, cte_term

    def _maybe_log_stanley_debug(
        self,
        mode: str,
        cte: float,
        heading_error: float,
        cte_term: float,
        steering: float,
        speed_cmd: float,
        control_speed: float,
        measured_speed_alive: bool,
    ) -> None:
        if self._stanley_debug_period <= 0.0:
            return
        self._stanley_debug_accum += self.timer_period
        if self._stanley_debug_accum < self._stanley_debug_period:
            return
        self._stanley_debug_accum = 0.0
        speed_source = "MEASURED" if measured_speed_alive else "COMMAND_FALLBACK"
        self.get_logger().info(
            f"stanley dbg [{mode}]: cte={cte:+.3f}m "
            f"hdg_err={math.degrees(heading_error):+.1f}deg "
            f"cte_term={math.degrees(cte_term):+.1f}deg "
            f"steer={math.degrees(steering):+.1f}deg "
            f"speed_cmd={speed_cmd:.2f} measured={self._measured_speed_mps:.2f} "
            f"filtered={self._filtered_speed_mps:.2f} "
            f"control_speed={control_speed:.2f} speed_source={speed_source}"
        )

    def _maybe_log_status(
        self,
        *,
        pose_ok: bool,
        x: float,
        y: float,
        yaw: float,
        csv_x: float,
        csv_y: float,
        cte: float,
        speed_cmd: float,
        measured_speed: float,
        control_speed: float,
        steering: float,
        mode: str,
        path_x: float | None = None,
        path_y: float | None = None,
        steering_raw: float | None = None,
    ) -> None:
        if self._status_log_period <= 0.0:
            return

        self._status_log_accum += self.timer_period
        if self._status_log_accum < self._status_log_period:
            return
        self._status_log_accum = 0.0

        if not pose_ok:
            self.get_logger().info("STATUS | TF 없음 (map -> base_link)")
            return

        lat = math.hypot(x - csv_x, y - csv_y)
        steer_deg = math.degrees(steering)
        raw_part = ""
        if steering_raw is not None:
            raw_part = f" steer_raw={math.degrees(steering_raw):+.1f}°"

        if mode == "LOCAL_PATH":
            px = path_x if path_x is not None else csv_x
            py = path_y if path_y is not None else csv_y
            self.get_logger().info(
                f"STATUS | LOCAL_PATH | "
                f"veh=({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):+.1f}°) "
                f"path=({px:.2f}, {py:.2f}) "
                f"cte={cte:+.2f}m "
                f"hdg_err={math.degrees(self._last_heading_err):+.1f}° "
                f"cte_term={math.degrees(self._last_cte_term):+.1f}° "
                f"v_cmd={speed_cmd:.2f}m/s v_meas={measured_speed:.2f}m/s "
                f"v_ctrl={control_speed:.2f}m/s "
                f"steer={steer_deg:+.1f}°{raw_part}"
            )
            return

        self.get_logger().info(
            f"STATUS | veh=({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):+.1f}°) "
            f"csv=({csv_x:.2f}, {csv_y:.2f}) lat={lat:.2f}m cte={cte:+.2f}m "
            f"hdg_err={math.degrees(self._last_heading_err):+.1f}° "
            f"cte_term={math.degrees(self._last_cte_term):+.1f}° "
            f"v_cmd={speed_cmd:.2f}m/s v_meas={measured_speed:.2f}m/s "
            f"v_ctrl={control_speed:.2f}m/s "
            f"steer={steer_deg:+.1f}° mode={mode}"
        )

    def _smooth_steering(self, target: float) -> float:
        alpha = max(0.0, min(1.0, self.steering_smooth_alpha))
        if alpha <= 0.0:
            return target
        return self._last_steering_cmd + alpha * (target - self._last_steering_cmd)

    def _rate_limit_steering(self, target: float) -> float:
        max_step = self.steering_rate_limit_radps * self.timer_period
        diff = target - self._last_steering_cmd
        diff = max(-max_step, min(max_step, diff))
        out = self._last_steering_cmd + diff
        out = max(-self.max_steering, min(self.max_steering, out))
        self._last_steering_cmd = out
        return out

    def _compute_path_curvature(self, path: List[Tuple[float, float]], nearest_idx: int) -> float:
        """
        경로의 곡률을 계산한다. nearest_idx 근처 3개 점을 사용.

        부호: 양수 = 좌회전, 음수 = 우회전 (전진 방향 기준)
        """
        if len(path) < 3 or nearest_idx < 0 or nearest_idx >= len(path) - 2:
            return 0.0  # 곡률 계산 불가능

        # nearest_idx, nearest_idx+1, nearest_idx+2 사용
        p0 = path[nearest_idx]
        p1 = path[nearest_idx + 1]
        p2 = path[nearest_idx + 2]

        x0, y0 = p0
        x1, y1 = p1
        x2, y2 = p2

        # 점들 사이 거리
        dx1 = x1 - x0
        dy1 = y1 - y0
        d1 = math.sqrt(dx1 * dx1 + dy1 * dy1)

        dx2 = x2 - x1
        dy2 = y2 - y1
        d2 = math.sqrt(dx2 * dx2 + dy2 * dy2)

        if d1 < 1e-6 or d2 < 1e-6:
            return 0.0

        # yaw 각도
        yaw1 = math.atan2(dy1, dx1)
        yaw2 = math.atan2(dy2, dx2)

        # yaw 변화
        dyaw = wrap_pi(yaw2 - yaw1)
        avg_dist = (d1 + d2) / 2.0

        if avg_dist < 1e-6:
            return 0.0

        curvature = dyaw / avg_dist  # 1/m
        return curvature

    def _publish_control_diagnostics(
        self,
        raw_steer: float,
        filtered_steer: float,
        cte: float,
        heading_err: float,
        path: List[Tuple[float, float]],
        nearest_idx: int,
    ) -> None:
        """현재 Stanley 제어 진단 값들을 topic으로 publish."""
        # Normalize steering to [-1, 1] based on max_steering
        if abs(self.max_steering) > 1e-6:
            raw_norm = raw_steer / self.max_steering
            filtered_norm = filtered_steer / self.max_steering
        else:
            raw_norm = 0.0
            filtered_norm = 0.0

        msg = Float64()
        msg.data = float(raw_norm)
        self.raw_steer_cmd_pub.publish(msg)

        msg.data = float(filtered_norm)
        self.filtered_steer_cmd_pub.publish(msg)

        msg.data = float(cte)
        self.cte_pub.publish(msg)

        msg.data = float(heading_err)
        self.heading_error_pub.publish(msg)

        curvature = self._compute_path_curvature(path, nearest_idx)
        msg.data = float(curvature)
        self.path_curvature_pub.publish(msg)

    def _publish_drive(self, speed: float, steering: float) -> None:
        # 실제 속도는 control_node. /drive 에는 조향만 (publish_drive_speed=False).
        if self.publish_drive_speed and abs(speed) < 1e-6:
            self._last_speed_cmd = None

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed) if self.publish_drive_speed else 0.0
        msg.drive.steering_angle = float(steering)
        self.drive_pub.publish(msg)

    def _publish_tracked_path(self) -> None:
        if self.tracked_path_pub is None or len(self._path_poses) < 2:
            return

        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        for px, py in self._path_poses:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(px)
            ps.pose.position.y = float(py)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)

        self.tracked_path_pub.publish(msg)

    def _timer_cb(self) -> None:
        pose = self._get_pose_map()

        if pose is None:
            self._publish_drive(0.0, 0.0)
            self._maybe_log_status(
                pose_ok=False,
                x=0.0,
                y=0.0,
                yaw=0.0,
                csv_x=0.0,
                csv_y=0.0,
                cte=0.0,
                speed_cmd=0.0,
                measured_speed=self._filtered_speed_mps,
                control_speed=0.0,
                steering=0.0,
                mode="NO_TF",
            )
            return

        x, y, yaw = pose
        csv_x, csv_y, _ = self.track.closest_projection_on_loop(x, y)

        mode = self._select_mode_and_path(x, y)

        if mode == "STOP" or len(self._path_poses) < 2:
            self._publish_drive(0.0, 0.0)
            self._maybe_log_status(
                pose_ok=True,
                x=x,
                y=y,
                yaw=yaw,
                csv_x=csv_x,
                csv_y=csv_y,
                cte=0.0,
                speed_cmd=0.0,
                measured_speed=self._filtered_speed_mps,
                control_speed=0.0,
                steering=0.0,
                mode=mode,
            )
            return

        self._publish_tracked_path()

        target_speed = self._target_speed(mode)
        speed_cmd = self._smooth_speed(target_speed)
        control_speed, measured_speed_alive = self._get_control_speed(speed_cmd)

        steering_raw, cte, path_x, path_y, heading_err, cte_term = self._stanley_control(
            self._path_poses,
            x,
            y,
            yaw,
            control_speed,
            mode,
        )
        self._maybe_log_stanley_debug(
            mode,
            cte,
            heading_err,
            cte_term,
            steering_raw,
            speed_cmd,
            control_speed,
            measured_speed_alive,
        )
        self._last_heading_err = heading_err
        self._last_cte_term = cte_term

        steering_smoothed = self._smooth_steering(steering_raw)
        steering_cmd = self._rate_limit_steering(steering_smoothed)

        # Publish control diagnostics for CSV logging
        # Find nearest path index for curvature calculation
        nearest_idx = 0
        if len(self._path_poses) >= 2:
            best_d2 = float("inf")
            px = x + self.wheelbase * math.cos(yaw)
            py = y + self.wheelbase * math.sin(yaw)
            for i in range(len(self._path_poses) - 1):
                ax, ay = self._path_poses[i]
                bx, by = self._path_poses[i + 1]
                qx, qy, _ = closest_point_on_segment(px, py, ax, ay, bx, by)
                d2 = (px - qx) ** 2 + (py - qy) ** 2
                if d2 < best_d2:
                    best_d2 = d2
                    nearest_idx = i

        self._publish_control_diagnostics(
            steering_raw,
            steering_cmd,
            cte,
            heading_err,
            self._path_poses,
            nearest_idx,
        )

        self._publish_drive(speed_cmd, steering_cmd)

        self._maybe_log_status(
            pose_ok=True,
            x=x,
            y=y,
            yaw=yaw,
            csv_x=csv_x,
            csv_y=csv_y,
            cte=cte,
            speed_cmd=speed_cmd,
            measured_speed=self._filtered_speed_mps,
            control_speed=control_speed,
            steering=steering_cmd,
            mode=mode,
            path_x=path_x,
            path_y=path_y,
            steering_raw=steering_raw,
        )


def main(args=None):
    rclpy.init(args=args)
    node = StanleyWaypointFollowNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
