#!/usr/bin/env python3
"""
주행 전략 노드 — 직선·곡선·장애물에 따른 목표 속도 제안.

- CSV 레이스라인으로 곡선 구간(1m 간격 샘플에서 방향 변화가 큰 구간)을 전처리.
- TF로 map 상 차 위치 → 트랙 누적 거리 s 투영.
- /static_obstacles(Float32MultiArray)로 전방 장애물 거리 판단.
- 발행: /strategy/target_speed, /strategy/speed_multiplier, /strategy/speed_condition(UInt8).

speed_condition 코드(로컬플래너·웨이포인트 연동용):
  0=HOLD(TF실패·안전), 1=곡선내, 2=곡선직전(룩어헤드), 3=장애물≤3m, 4=장애물≤10m, 5=직선가속

속도 스케일(기본):
  직선(곡선 아님 + 조건 충족): medium_speed * 2
  곡선 구간 내부: medium_speed * speed_curve_mul (기본 0.5)
  그 외/감속 구간: medium_speed
"""
from __future__ import annotations

import math
from typing import List, Sequence, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64, UInt8
from tf2_ros import Buffer, TransformException, TransformListener

from path_following.track_sliding import (
    apply_track_direction,
    load_csv_xy,
    param_bool,
    resolve_csv_path,
)


# ============================================================
# USER TUNING — drive_strategy (여기만 수정)
# ============================================================
CFG = {
    "csv_path": "",
    "reverse_track_direction": False,
    "map_frame": "map",
    "base_frame": "base_link",
    "tf_lookup_timeout_sec": 0.2,
    "medium_speed": 1.6,
    "speed_straight_mul": 2.0,
    "speed_curve_mul": 0.5,
    "obstacle_topic": "/static_obstacles",
    "obstacle_clear_m": 10.0,
    "obstacle_slow_m": 3.0,
    "curve_lookahead_m": 3.0,
    "curve_sample_step_m": 1.0,
    "curve_deg_per_m": 4.0,
    "timer_hz": 10.0,
    "publish_debug": False,
    "topic_target_speed": "/strategy/target_speed",
    "topic_speed_multiplier": "/strategy/speed_multiplier",
    "topic_speed_condition": "/strategy/speed_condition",
}


def _wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _closest_on_loop(
    xp: float,
    yp: float,
    xs: Sequence[float],
    ys: Sequence[float],
) -> Tuple[float, float, int, float]:
    """폐곡선 상 최근접점, 세그먼트 인덱스, 해당 세그먼트 위 파라미터 t∈[0,1]."""
    n = len(xs)
    best_d2 = float("inf")
    best_qx = best_qy = 0.0
    best_i = 0
    best_t = 0.0
    for i in range(n):
        ax, ay = xs[i], ys[i]
        bx, by = xs[(i + 1) % n], ys[(i + 1) % n]
        abx, aby = bx - ax, by - ay
        apx, apy = xp - ax, yp - ay
        ab2 = abx * abx + aby * aby
        if ab2 < 1e-14:
            continue
        t = max(0.0, min(1.0, (apx * abx + apy * aby) / ab2))
        qx = ax + t * abx
        qy = ay + t * aby
        d2 = (xp - qx) ** 2 + (yp - qy) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_qx, best_qy = qx, qy
            best_i = i
            best_t = t
    return best_qx, best_qy, best_i, best_t


def _build_loop_geometry(
    pts: List[Tuple[float, float]],
) -> Tuple[List[float], List[float], List[float], float]:
    n = len(pts)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    seg_len: List[float] = []
    for i in range(n):
        ax, ay = xs[i], ys[i]
        bx, by = xs[(i + 1) % n], ys[(i + 1) % n]
        seg_len.append(math.hypot(bx - ax, by - ay))
    cum0 = [0.0]
    for i in range(n):
        cum0.append(cum0[-1] + seg_len[i])
    total_l = cum0[-1]
    seg_start = cum0[:-1]
    return xs, ys, seg_start, total_l


def _polyline_xy_at_s(
    s: float,
    total_l: float,
    xs: List[float],
    ys: List[float],
    seg_start: List[float],
    seg_len: List[float],
    n: int,
) -> Tuple[float, float, int]:
    if total_l < 1e-6:
        return xs[0], ys[0], 0
    s = s % total_l
    for i in range(n):
        if seg_start[i] + seg_len[i] >= s - 1e-9:
            tloc = (s - seg_start[i]) / max(seg_len[i], 1e-9)
            tloc = max(0.0, min(1.0, tloc))
            x = xs[i] + tloc * (xs[(i + 1) % n] - xs[i])
            y = ys[i] + tloc * (ys[(i + 1) % n] - ys[i])
            return x, y, i
    return xs[-1], ys[-1], n - 1


def _forward_dist_on_loop(s_from: float, s_to: float, total_l: float) -> float:
    d = s_to - s_from
    while d < 0:
        d += total_l
    while d >= total_l:
        d -= total_l
    return d


def _merge_intervals(
    raw: List[Tuple[float, float]], total_l: float
) -> List[Tuple[float, float]]:
    if not raw:
        return []
    raw = sorted(raw, key=lambda x: x[0])
    out: List[Tuple[float, float]] = []
    a, b = raw[0]
    for c, d in raw[1:]:
        if c <= b + 0.05:
            b = max(b, d)
        else:
            out.append((a, min(b, total_l)))
            a, b = c, d
    out.append((a, min(b, total_l)))
    return out


def compute_curve_intervals_1m(
    xs: List[float],
    ys: List[float],
    seg_start: List[float],
    seg_len: List[float],
    total_l: float,
    n: int,
    *,
    sample_step_m: float,
    curve_deg_per_m: float,
) -> List[Tuple[float, float]]:
    """1m 보간으로 이웃 샘플 간 |Δheading|이 임계 이상이면 곡선으로 묶인 구간."""
    if total_l < sample_step_m * 2:
        return []
    hs: List[float] = []
    ss: List[float] = []
    u = 0.0
    uf = min(sample_step_m * 0.5, total_l * 0.5)
    while u < total_l - 1e-6:
        x0, y0, _ = _polyline_xy_at_s(u, total_l, xs, ys, seg_start, seg_len, n)
        x1, y1, _ = _polyline_xy_at_s(
            (u + uf) % total_l, total_l, xs, ys, seg_start, seg_len, n
        )
        hs.append(math.atan2(y1 - y0, x1 - x0))
        ss.append(u + uf * 0.5)
        u += sample_step_m
    curve_flag = []
    thresh = math.radians(curve_deg_per_m)
    for i in range(len(hs) - 1):
        curve_flag.append(abs(_wrap_pi(hs[i + 1] - hs[i])) >= thresh * 0.8)
    raw_iv: List[Tuple[float, float]] = []
    i = 0
    while i < len(curve_flag):
        if not curve_flag[i]:
            i += 1
            continue
        j = i
        while j < len(curve_flag) and curve_flag[j]:
            j += 1
        s0 = ss[i]
        s1 = ss[j - 1] + sample_step_m
        raw_iv.append((max(0.0, s0 - sample_step_m), min(total_l, s1)))
        i = j if j > i else i + 1
    return _merge_intervals(raw_iv, total_l)


def _point_in_curve_zones(s: float, zones: List[Tuple[float, float]], total_l: float) -> bool:
    for a, b in zones:
        if a <= s <= b:
            return True
        if a > b and (s >= a or s <= b):
            return True
    return False


def _next_curve_entry_distance(
    s: float, zones: List[Tuple[float, float]], total_l: float
) -> float | None:
    """순방향으로 가장 가까운 곡선 구간 시작점까지의 호 거리 (래핑 포함)."""
    if not zones:
        return None
    best: float | None = None
    for a, b in zones:
        fd = _forward_dist_on_loop(s, a, total_l)
        if fd < 1e-4:
            continue
        if best is None or fd < best:
            best = fd
    return best


class DriveStrategyNode(Node):
    def __init__(self) -> None:
        super().__init__("drive_strategy_node")

        for key, value in CFG.items():
            self.declare_parameter(key, value)

        path = resolve_csv_path(
            self.get_parameter("csv_path").get_parameter_value().string_value
        )
        reverse_track = param_bool(
            self.get_parameter("reverse_track_direction").value
        )
        self._pts = apply_track_direction(load_csv_xy(path), reverse_track)
        if len(self._pts) < 3:
            raise RuntimeError(f"CSV 점 개수 부족: {path}")
        self._xs, self._ys, self._seg_start, self._total_l = _build_loop_geometry(self._pts)
        self._n = len(self._xs)
        self._seg_len = [
            math.hypot(
                self._xs[(i + 1) % self._n] - self._xs[i],
                self._ys[(i + 1) % self._n] - self._ys[i],
            )
            for i in range(self._n)
        ]

        step = float(self.get_parameter("curve_sample_step_m").value)
        deg_pm = float(self.get_parameter("curve_deg_per_m").value)
        self._curve_zones = compute_curve_intervals_1m(
            self._xs,
            self._ys,
            self._seg_start,
            self._seg_len,
            self._total_l,
            self._n,
            sample_step_m=step,
            curve_deg_per_m=deg_pm,
        )

        self._medium = float(self.get_parameter("medium_speed").value)
        self._mul_str = float(self.get_parameter("speed_straight_mul").value)
        self._mul_curve = float(self.get_parameter("speed_curve_mul").value)
        self._obs_clear_m = float(self.get_parameter("obstacle_clear_m").value)
        self._obs_slow_m = float(self.get_parameter("obstacle_slow_m").value)
        self._curve_lookahead_m = float(self.get_parameter("curve_lookahead_m").value)
        self._map_frame = self.get_parameter("map_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._tf_timeout = float(self.get_parameter("tf_lookup_timeout_sec").value)
        self._publish_debug = param_bool(self.get_parameter("publish_debug").value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        obs_topic = self.get_parameter("obstacle_topic").value
        self.create_subscription(Float32MultiArray, obs_topic, self._cb_obs, 10)

        self._obs_msg_data: List[float] = []
        ts = self.get_parameter("topic_target_speed").value
        tm = self.get_parameter("topic_speed_multiplier").value
        tc = self.get_parameter("topic_speed_condition").value
        self._pub_speed = self.create_publisher(Float64, ts, 10)
        self._pub_mul = self.create_publisher(Float64, tm, 10)
        self._pub_cond = self.create_publisher(UInt8, tc, 10)

        hz = max(1.0, float(self.get_parameter("timer_hz").value))
        self.create_timer(1.0 / hz, self._tick)

        self.get_logger().info(
            f"drive_strategy: csv={path} L={self._total_l:.1f}m, "
            f"곡선구간 {len(self._curve_zones)}개, medium={self._medium} "
            f"(직선×{self._mul_str}, 곡선×{self._mul_curve})"
        )

    def _cb_obs(self, msg: Float32MultiArray) -> None:
        self._obs_msg_data = list(msg.data)

    def _nearest_obstacle_m(self) -> float | None:
        """Float32MultiArray [id,x,y,r,...] laser/base 기준, 전방만 필터."""
        data = self._obs_msg_data
        best: float | None = None
        i = 0
        while i + 2 < len(data):
            x = float(data[i + 1])
            y = float(data[i + 2])
            if x > 0.0 and x <= self._obs_clear_m + 2.0:
                d = math.hypot(x, y)
                if best is None or d < best:
                    best = d
            i += 4
        return best

    def _pose_map(self) -> Tuple[float, float] | None:
        try:
            t = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self._tf_timeout),
            )
        except TransformException:
            return None
        return (t.transform.translation.x, t.transform.translation.y)

    def _tick(self) -> None:
        # 조건 코드: 항상 하나 (0..5). TF 실패 시 0 + 배율 1.0
        COND_HOLD = 0
        COND_CURVE = 1
        COND_APPROACH = 2
        COND_OBS_3M = 3
        COND_OBS_10M = 4
        COND_STRAIGHT = 5

        pose = self._pose_map()
        if pose is None:
            self._pub_speed.publish(Float64(data=max(0.0, self._medium * 1.0)))
            self._pub_mul.publish(Float64(data=1.0))
            self._pub_cond.publish(UInt8(data=COND_HOLD))
            return

        mx, my = pose
        _, _, seg_i, tpar = _closest_on_loop(mx, my, self._xs, self._ys)
        s = self._seg_start[seg_i] + tpar * self._seg_len[seg_i]

        in_curve = _point_in_curve_zones(s, self._curve_zones, self._total_l)
        dist_to_curve_entry = _next_curve_entry_distance(
            s, self._curve_zones, self._total_l
        )
        approaching_curve = (
            not in_curve
            and dist_to_curve_entry is not None
            and dist_to_curve_entry <= self._curve_lookahead_m + 1e-6
        )

        # 장애 거리는 한 번만 계산 (디버그/분기에서 중복 호출 방지)
        obs_nearest_d = self._nearest_obstacle_m()
        blocked = obs_nearest_d is not None and obs_nearest_d <= self._obs_clear_m + 1e-6
        obs_medium = obs_nearest_d is not None and obs_nearest_d <= self._obs_slow_m + 1e-6

        mul: float
        cond_u8: int
        reason = ""
        if in_curve:
            mul = self._mul_curve
            cond_u8 = COND_CURVE
            reason = "curve"
        elif approaching_curve:
            mul = 1.0
            cond_u8 = COND_APPROACH
            reason = "approach_curve"
        elif obs_medium:
            mul = 1.0
            cond_u8 = COND_OBS_3M
            reason = "obstacle_3m"
        elif blocked:
            mul = 1.0
            cond_u8 = COND_OBS_10M
            reason = "obstacle_10m"
        else:
            mul = self._mul_str
            cond_u8 = COND_STRAIGHT
            reason = "straight"

        v = max(0.0, self._medium * mul)
        self._pub_speed.publish(Float64(data=v))
        self._pub_mul.publish(Float64(data=mul))
        self._pub_cond.publish(UInt8(data=cond_u8))

        if self._publish_debug:
            self.get_logger().info(
                f"s={s:.1f} m mul={mul:.2f} cond={cond_u8} ({reason}) in_curve={in_curve} "
                f"dist_next_curve={dist_to_curve_entry} obs_d={obs_nearest_d}"
            )


def main(args: object = None) -> None:
    rclpy.init(args=args)
    node = DriveStrategyNode()
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
