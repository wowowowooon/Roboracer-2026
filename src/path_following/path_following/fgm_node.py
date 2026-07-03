#!/usr/bin/env python3
"""
FGM (Follow the Gap Method) 노드.

/scan + /static_obstacles → FOV, Safety Bubble, Max Gap → /fgm_target.

**타이밍·게이트(언제 LOCAL_PATH 쓸지)는 local_planner_node CFG 에서만 조정.**
여기는 갭 추종 알고리즘·스무딩 파라미터만.
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker


# ============================================================
# USER TUNING — FGM 파라미터 (여기만 수정)
# launch에서 같은 이름으로 넣으면 launch 값이 우선.
# ============================================================
CFG = {
    # Topics
    "scan_topic": "/scan",
    "laser_frame": "laser",
    "obstacle_topic": "/static_obstacles",
    "target_topic": "/fgm_target",
    "publish_debug_scan": False,
    # 스캔 전처리·갭 (알고리즘)
    "fov_half_deg": 60.0,
    "preprocess_max_range_m": 2.0,
    "bubble_radius_m": 0.20,
    "obstacle_bubble_trigger_dist_m": 0.5,
    "gap_threshold_primary_m": 1.5,
    "gap_threshold_fallback_m": 0.5,
    "min_gap_width_bins": 4,
    "gap_hysteresis_len_ratio": 0.78,
    # 목표점 (레이저 프레임, 갭 방향 거리 [m])
    "target_distance_m": 0.5,
    "gap_lateral_gain": 0.8,
    "max_avoid_heading_deg": 45.0,
    # 목표 스무딩
    "target_smooth_alpha": 0.36,
    "target_max_step_m": 0.17,
    "max_raw_target_step_m": 0.6,
    "target_smooth_beta": 0.46,
    "target_output_damping": 0.14,
    "target_out_max_step_m": 0.13,
    # RViz V자 갭 마커 (주행과 무관, 표시만)
    "gap_marker_arm_scale": 1.5,
    "gap_marker_max_arm_m": 2.0,
}


def _param_bool(val) -> bool:
    if isinstance(val, bool):
        return val
    return str(val).lower() in ("1", "true", "yes")


class FGMNode(Node):
    def __init__(self):
        super().__init__("fgm_node")

        for key, value in CFG.items():
            self.declare_parameter(key, value)

        scan_t = self.get_parameter("scan_topic").value
        obs_t = self.get_parameter("obstacle_topic").value
        tgt_t = self.get_parameter("target_topic").value
        self._laser_frame = str(self.get_parameter("laser_frame").value)

        self.scan_sub = self.create_subscription(LaserScan, scan_t, self.scan_callback, 10)
        self.obstacle_sub = self.create_subscription(
            Float32MultiArray, obs_t, self.obstacle_callback, 10
        )

        self.target_pub = self.create_publisher(PointStamped, tgt_t, 10)
        self.debug_scan_pub = self.create_publisher(LaserScan, "/fgm_debug_scan", 10)
        self.gap_marker_pub = self.create_publisher(Marker, "/fgm_gap_marker", 10)

        self.preprocess_dist = float(self.get_parameter("preprocess_max_range_m").value)
        self.bubble_radius = float(self.get_parameter("bubble_radius_m").value)
        self.obstacle_bubble_trigger_dist_m = float(
            self.get_parameter("obstacle_bubble_trigger_dist_m").value
        )
        self.publish_debug_scan = _param_bool(self.get_parameter("publish_debug_scan").value)

        self.fov_angle = math.radians(float(self.get_parameter("fov_half_deg").value))
        self.gap_thr_primary = float(self.get_parameter("gap_threshold_primary_m").value)
        self.gap_thr_fallback = float(self.get_parameter("gap_threshold_fallback_m").value)
        self.target_dist_default = float(self.get_parameter("target_distance_m").value)
        self.min_gap_bins = max(2, int(self.get_parameter("min_gap_width_bins").value))
        self.hyst_ratio = min(
            0.999,
            max(0.3, float(self.get_parameter("gap_hysteresis_len_ratio").value)),
        )
        self.smooth_alpha = min(
            1.0, max(0.0, float(self.get_parameter("target_smooth_alpha").value))
        )
        self.max_step_m = max(0.0, float(self.get_parameter("target_max_step_m").value))
        self.max_raw_step_m = max(0.0, float(self.get_parameter("max_raw_target_step_m").value))
        self.smooth_beta = min(
            1.0, max(0.0, float(self.get_parameter("target_smooth_beta").value))
        )
        self.output_damping = min(
            0.95, max(0.0, float(self.get_parameter("target_output_damping").value))
        )
        self.out_max_step_m = max(0.0, float(self.get_parameter("target_out_max_step_m").value))

        self.gap_lateral_gain = min(
            1.0, max(0.05, float(self.get_parameter("gap_lateral_gain").value))
        )
        self.max_avoid_heading_rad = math.radians(
            max(5.0, min(85.0, float(self.get_parameter("max_avoid_heading_deg").value)))
        )

        self.gap_marker_arm_scale = max(
            0.0, float(self.get_parameter("gap_marker_arm_scale").value)
        )
        _gmax = float(self.get_parameter("gap_marker_max_arm_m").value)
        self.gap_marker_max_arm_m = _gmax if _gmax > 0.0 else None

        self.latest_obstacles: list = []
        self._last_gap_center_idx: int | None = None
        self._filt_x: float | None = None
        self._filt_y: float | None = None
        self._out_x: float | None = None
        self._out_y: float | None = None
        self._prev_pub_x: float | None = None
        self._prev_pub_y: float | None = None
        self._last_raw_x: float | None = None
        self._last_raw_y: float | None = None

        self.get_logger().info(
            f"FGM started | target={self.target_dist_default}m, "
            f"preprocess={self.preprocess_dist}m, "
            f"bubble≤{self.obstacle_bubble_trigger_dist_m}m, "
            f"marker scale={self.gap_marker_arm_scale} max={_gmax}m"
        )

    def obstacle_callback(self, msg: Float32MultiArray) -> None:
        self.latest_obstacles = list(msg.data)

    def _reset_fgm_filter_state(self) -> None:
        self._last_gap_center_idx = None
        self._filt_x = self._filt_y = None
        self._out_x = self._out_y = None
        self._prev_pub_x = self._prev_pub_y = None
        self._last_raw_x = self._last_raw_y = None

    def _publish_gap_marker_delete(self) -> None:
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self._laser_frame
        m.ns = "fgm_gap"
        m.id = 0
        m.action = Marker.DELETE
        self.gap_marker_pub.publish(m)

    def _clamp_raw_target(self, tx: float, ty: float) -> tuple[float, float]:
        if self.max_raw_step_m <= 0.0 or self._last_raw_x is None:
            return tx, ty
        dx, dy = tx - self._last_raw_x, ty - self._last_raw_y
        d = math.hypot(dx, dy)
        if d > self.max_raw_step_m and d > 1e-9:
            s = self.max_raw_step_m / d
            tx = self._last_raw_x + dx * s
            ty = self._last_raw_y + dy * s
        return tx, ty

    def _select_gap(self, gaps: list, max_len: int) -> np.ndarray | None:
        if not gaps:
            return None
        wide = [g for g in gaps if len(g) >= self.min_gap_bins]
        if not wide:
            wide = list(gaps)
        thresh_len = max(self.min_gap_bins, int(math.ceil(self.hyst_ratio * max_len)))

        def center_idx(g: np.ndarray) -> int:
            return int(g[len(g) // 2])

        if self._last_gap_center_idx is not None:
            candidates = [g for g in wide if len(g) >= thresh_len]
            if not candidates:
                candidates = wide
            best = min(
                candidates,
                key=lambda g: abs(center_idx(g) - self._last_gap_center_idx),
            )
            return best

        return max(wide, key=lambda g: len(g))

    def _first_stage_smooth(self, tx: float, ty: float) -> tuple[float, float]:
        """1차 EMA + 스텝 제한 (내부 상태 _filt_*)."""
        if self._filt_x is None:
            self._filt_x, self._filt_y = tx, ty
            return float(tx), float(ty)

        px, py = self._filt_x, self._filt_y
        a = self.smooth_alpha
        if a <= 0.0:
            nx, ny = tx, ty
        else:
            nx = px + a * (tx - px)
            ny = py + a * (ty - py)

        dx, dy = nx - px, ny - py
        dist = math.hypot(dx, dy)
        if self.max_step_m > 0.0 and dist > self.max_step_m and dist > 1e-9:
            s = self.max_step_m / dist
            nx = px + dx * s
            ny = py + dy * s

        self._filt_x, self._filt_y = nx, ny
        return float(nx), float(ny)

    def _second_stage_and_damp(self, fx: float, fy: float) -> tuple[float, float]:
        """2차 EMA + 출력 감쇠 + 출력 스텝 제한. 발행 좌표는 여기서 확정."""
        b = self.smooth_beta
        if b <= 0.0 or self._out_x is None:
            ox, oy = fx, fy
            self._out_x, self._out_y = ox, oy
        else:
            self._out_x += b * (fx - self._out_x)
            self._out_y += b * (fy - self._out_y)
            ox, oy = self._out_x, self._out_y

        dmp = self.output_damping
        if dmp > 0.0 and self._prev_pub_x is not None and self._prev_pub_y is not None:
            ox -= dmp * (ox - self._prev_pub_x)
            oy -= dmp * (oy - self._prev_pub_y)

        if self.out_max_step_m > 0.0 and self._prev_pub_x is not None and self._prev_pub_y is not None:
            dx, dy = ox - self._prev_pub_x, oy - self._prev_pub_y
            dist = math.hypot(dx, dy)
            if dist > self.out_max_step_m and dist > 1e-9:
                s = self.out_max_step_m / dist
                ox = self._prev_pub_x + dx * s
                oy = self._prev_pub_y + dy * s

        self._out_x, self._out_y = ox, oy
        self._prev_pub_x, self._prev_pub_y = ox, oy
        return float(ox), float(oy)

    def scan_callback(self, scan_msg: LaserScan) -> None:
        ranges = np.array(scan_msg.ranges, dtype=np.float64)
        ranges = np.where(np.isinf(ranges), self.preprocess_dist, ranges)
        ranges = np.where(np.isnan(ranges), 0.0, ranges)
        ranges[ranges > self.preprocess_dist] = self.preprocess_dist

        angle_min = scan_msg.angle_min
        angle_inc = scan_msg.angle_increment
        if angle_inc <= 1e-12:
            self.get_logger().warn("LaserScan angle_increment too small.")
            return

        start_fov_idx = int((-self.fov_angle - angle_min) / angle_inc)
        end_fov_idx = int((self.fov_angle - angle_min) / angle_inc)
        start_fov_idx = max(0, start_fov_idx)
        end_fov_idx = min(len(ranges), end_fov_idx)

        ranges[:start_fov_idx] = 0.0
        ranges[end_fov_idx:] = 0.0

        valid_indices = np.where(ranges > 0.0)[0]
        if len(valid_indices) > 0:
            min_dist_idx = int(valid_indices[np.argmin(ranges[valid_indices])])
            min_dist = float(ranges[min_dist_idx])
            if min_dist < self.preprocess_dist:
                self.create_bubble(ranges, min_dist_idx, min_dist, angle_inc)

        if len(self.latest_obstacles) > 0:
            num_obs = len(self.latest_obstacles) // 4
            for i in range(num_obs):
                obs_x = self.latest_obstacles[4 * i + 1]
                obs_y = self.latest_obstacles[4 * i + 2]
                obs_r = self.latest_obstacles[4 * i + 3]
                obs_dist = math.sqrt(obs_x**2 + obs_y**2)
                if obs_dist > self.obstacle_bubble_trigger_dist_m:
                    continue
                obs_angle = math.atan2(obs_y, obs_x)
                obs_idx = int((obs_angle - angle_min) / angle_inc)
                if 0 <= obs_idx < len(ranges):
                    effective_radius = self.bubble_radius + obs_r
                    self.create_bubble(
                        ranges,
                        obs_idx,
                        obs_dist,
                        angle_inc,
                        radius_override=effective_radius,
                    )

        gap_threshold = self.gap_thr_primary
        threshold_indices = np.where(ranges > gap_threshold)[0]
        if len(threshold_indices) == 0:
            gap_threshold = self.gap_thr_fallback
            threshold_indices = np.where(ranges > gap_threshold)[0]
            if len(threshold_indices) == 0:
                # 갭이 없을 땐 이전 출력 반복 발행 금지(게이트 켠 상태에서만 여기 도달)
                return

        splits = np.where(np.diff(threshold_indices) > 1)[0] + 1
        gaps = [g for g in np.split(threshold_indices, splits) if len(g) > 0]
        if not gaps:
            return

        max_len = max(len(g) for g in gaps)
        chosen = self._select_gap(gaps, max_len)
        if chosen is None or len(chosen) == 0:
            return

        self._last_gap_center_idx = int(chosen[len(chosen) // 2])

        gap_start_idx = int(chosen[0])
        gap_end_idx = int(chosen[-1])
        best_idx = int(chosen[len(chosen) // 2])
        raw_angle = angle_min + best_idx * angle_inc
        eff_angle = raw_angle * self.gap_lateral_gain
        ma = self.max_avoid_heading_rad
        if abs(eff_angle) > ma:
            eff_angle = math.copysign(ma, eff_angle)

        target_dist = self.target_dist_default
        target_x = target_dist * math.cos(eff_angle)
        target_y = target_dist * math.sin(eff_angle)

        rx, ry = self._clamp_raw_target(target_x, target_y)
        self._last_raw_x, self._last_raw_y = rx, ry

        fx, fy = self._first_stage_smooth(rx, ry)
        ox, oy = self._second_stage_and_damp(fx, fy)

        viz_stamp = self.get_clock().now().to_msg()

        point_msg = PointStamped()
        point_msg.header.stamp = viz_stamp
        point_msg.header.frame_id = self._laser_frame
        point_msg.point.x = float(ox)
        point_msg.point.y = float(oy)
        point_msg.point.z = 0.0
        self.target_pub.publish(point_msg)

        debug_msg = LaserScan()
        debug_msg.header = scan_msg.header
        debug_msg.angle_min = scan_msg.angle_min
        debug_msg.angle_max = scan_msg.angle_max
        debug_msg.angle_increment = scan_msg.angle_increment
        debug_msg.range_min = scan_msg.range_min
        debug_msg.range_max = scan_msg.range_max
        debug_msg.time_increment = scan_msg.time_increment
        debug_msg.scan_time = scan_msg.scan_time
        debug_msg.ranges = [float(r) for r in ranges]

        debug_msg.header.stamp = viz_stamp
        if self.publish_debug_scan:
            self.debug_scan_pub.publish(debug_msg)

        self.publish_gap_marker(
            gap_start_idx,
            gap_end_idx,
            ranges,
            angle_min,
            angle_inc,
            viz_stamp,
        )

    def publish_gap_marker(
        self,
        start_idx: int,
        end_idx: int,
        ranges: np.ndarray,
        angle_min: float,
        angle_inc: float,
        stamp_msg,
    ) -> None:
        marker = Marker()
        marker.header.stamp = stamp_msg
        marker.header.frame_id = self._laser_frame
        marker.ns = "fgm_gap"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.05

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        p_origin = Point()
        p_origin.x = 0.0
        p_origin.y = 0.0
        p_origin.z = 0.0

        start_angle = angle_min + start_idx * angle_inc
        end_angle = angle_min + end_idx * angle_inc

        r_s = max(float(ranges[start_idx]), 1e-6)
        r_e = max(float(ranges[end_idx]), 1e-6)
        scale = self.gap_marker_arm_scale if self.gap_marker_arm_scale > 0.0 else 1.0
        len_s = r_s * scale
        len_e = r_e * scale
        if self.gap_marker_max_arm_m is not None:
            cap_hi = self.gap_marker_max_arm_m
        else:
            cap_hi = self.preprocess_dist
        len_s = min(len_s, cap_hi)
        len_e = min(len_e, cap_hi)

        p_start = Point()
        p_start.x = float(len_s * math.cos(start_angle))
        p_start.y = float(len_s * math.sin(start_angle))
        p_start.z = 0.0

        p_end = Point()
        p_end.x = float(len_e * math.cos(end_angle))
        p_end.y = float(len_e * math.sin(end_angle))
        p_end.z = 0.0

        marker.points.append(p_origin)
        marker.points.append(p_start)

        marker.points.append(p_origin)
        marker.points.append(p_end)

        self.gap_marker_pub.publish(marker)

    def create_bubble(
        self,
        ranges: np.ndarray,
        center_idx: int,
        dist: float,
        angle_inc: float,
        radius_override: float | None = None,
    ) -> None:
        radius = radius_override if radius_override is not None else self.bubble_radius
        safe_theta = math.atan(radius / (dist + 0.001))
        idx_radius = int(safe_theta / angle_inc)

        start_idx = max(0, center_idx - idx_radius)
        end_idx = min(len(ranges), center_idx + idx_radius)

        ranges[start_idx:end_idx] = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = FGMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
