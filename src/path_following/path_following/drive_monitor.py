#!/usr/bin/env python3
"""
실차 주행 디버그 모니터 — 별도 터미널에서 2Hz 갱신.

실행:
  source install/setup.bash && ros2 run path_following drive_monitor
  python3 src/path_following/scripts/drive_monitor.py
"""
from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass, field

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray, Float64, Float64MultiArray, String, UInt8
from tf2_ros import Buffer, TransformListener


def _rad2deg(r: float) -> float:
    return math.degrees(r)


def _esp_steer_to_servo_deg(norm: float) -> float:
    return 90.0 + float(norm) * 40.0


def _age_str(last_mono: float | None, *, stale: float = 0.5) -> str:
    if last_mono is None:
        return "없음"
    age = time.monotonic() - last_mono
    flag = " STALE" if age > stale else ""
    return f"{age:.2f}s{flag}"


@dataclass
class TopicStamp:
    last_mono: float | None = None
    hz: float = 0.0
    _times: list[float] = field(default_factory=list)

    def mark(self) -> None:
        now = time.monotonic()
        self.last_mono = now
        self._times.append(now)
        cutoff = now - 2.0
        self._times = [t for t in self._times if t >= cutoff]
        if len(self._times) >= 2:
            self.hz = (len(self._times) - 1) / (self._times[-1] - self._times[0])
        else:
            self.hz = 0.0


class DriveMonitor(Node):
    def __init__(self) -> None:
        super().__init__("drive_monitor")
        self.declare_parameter("refresh_hz", 2.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        self._map_frame = self.get_parameter("map_frame").value
        self._base_frame = self.get_parameter("base_frame").value

        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        self._st_drive = TopicStamp()
        self._drive_speed = 0.0
        self._drive_steer = 0.0
        self.create_subscription(
            AckermannDriveStamped, "/drive", self._cb_drive, 10
        )

        self._st_odom = TopicStamp()
        self._odom_v = 0.0
        self.create_subscription(Odometry, "/odom", self._cb_odom, 10)

        self._st_tel = TopicStamp()
        self._tel: list[float] = []
        self.create_subscription(
            Float64MultiArray, "/vehicle/telemetry", self._cb_telemetry, 10
        )

        self._st_scan = TopicStamp()
        self._scan_min_m = float("inf")
        self._scan_min_deg = 0.0
        self._scan_n = 0
        self.create_subscription(LaserScan, "/scan", self._cb_scan, 10)

        self._st_obs = TopicStamp()
        self._obs_count = 0
        self._obs_nearest_m = float("inf")
        self._obs_nearest_xy = (0.0, 0.0)
        self.create_subscription(
            Float32MultiArray, "/static_obstacles", self._cb_obs, 10
        )

        self._st_fgm = TopicStamp()
        self._fgm_x = 0.0
        self._fgm_y = 0.0
        self._fgm_dist = float("inf")
        self._fgm_heading_deg = 0.0
        self.create_subscription(
            PointStamped, "/fgm_target", self._cb_fgm, 10
        )

        self._st_override = TopicStamp()
        self._override = False
        self.create_subscription(
            Bool, "/planner_path_override_active", self._cb_override, 10
        )

        self._st_planner_mode = TopicStamp()
        self._planner_mode = "?"
        self.create_subscription(String, "/planner/mode", self._cb_planner_mode, 10)

        self._st_local_path = TopicStamp()
        self._local_path_n = 0
        self.create_subscription(Path, "/local_path", self._cb_local_path, 10)

        self._st_speed_scale = TopicStamp()
        self._planner_speed_scale = 1.0
        self.create_subscription(
            Float64, "/planner/speed_scale", self._cb_speed_scale, 10
        )

        self._st_strategy = TopicStamp()
        self._strategy_mul = 1.0
        self.create_subscription(
            Float64, "/strategy/speed_multiplier", self._cb_strategy, 10
        )

        self._st_speed_cond = TopicStamp()
        self._speed_cond = 0
        self.create_subscription(
            UInt8, "/planner/speed_condition", self._cb_speed_cond, 10
        )

        self._last_tf_xy: tuple[float, float] | None = None
        self._last_tf_mono: float | None = None
        self._tf_speed = 0.0
        self._pose_x = 0.0
        self._pose_y = 0.0
        self._pose_yaw_deg = 0.0
        self._tf_ok = False

        hz = float(self.get_parameter("refresh_hz").value)
        self.create_timer(1.0 / max(0.5, hz), self._refresh)

    def _cb_drive(self, msg: AckermannDriveStamped) -> None:
        self._st_drive.mark()
        self._drive_speed = float(msg.drive.speed)
        self._drive_steer = float(msg.drive.steering_angle)

    def _cb_odom(self, msg: Odometry) -> None:
        self._st_odom.mark()
        self._odom_v = float(msg.twist.twist.linear.x)

    def _cb_telemetry(self, msg: Float64MultiArray) -> None:
        self._st_tel.mark()
        self._tel = list(msg.data)

    def _cb_scan(self, msg: LaserScan) -> None:
        self._st_scan.mark()
        self._scan_n = len(msg.ranges)
        min_r = float("inf")
        min_deg = 0.0
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            if r < msg.range_min or r > msg.range_max:
                continue
            ang = _rad2deg(msg.angle_min + i * msg.angle_increment)
            if abs(ang) > 60.0:
                continue
            if r < min_r:
                min_r = r
                min_deg = ang
        self._scan_min_m = min_r
        self._scan_min_deg = min_deg

    def _cb_obs(self, msg: Float32MultiArray) -> None:
        self._st_obs.mark()
        data = msg.data
        n = len(data) // 4
        self._obs_count = n
        nearest = float("inf")
        nxy = (0.0, 0.0)
        for i in range(n):
            base = i * 4
            x = float(data[base + 1])
            y = float(data[base + 2])
            d = math.hypot(x, y)
            if d < nearest:
                nearest = d
                nxy = (x, y)
        self._obs_nearest_m = nearest
        self._obs_nearest_xy = nxy

    def _cb_fgm(self, msg: PointStamped) -> None:
        self._st_fgm.mark()
        self._fgm_x = float(msg.point.x)
        self._fgm_y = float(msg.point.y)
        self._fgm_dist = math.hypot(self._fgm_x, self._fgm_y)
        self._fgm_heading_deg = _rad2deg(math.atan2(self._fgm_y, self._fgm_x))

    def _cb_override(self, msg: Bool) -> None:
        self._st_override.mark()
        self._override = bool(msg.data)

    def _cb_planner_mode(self, msg: String) -> None:
        self._st_planner_mode.mark()
        self._planner_mode = msg.data.strip() or "?"

    def _cb_local_path(self, msg: Path) -> None:
        self._st_local_path.mark()
        self._local_path_n = len(msg.poses)

    def _cb_speed_scale(self, msg: Float64) -> None:
        self._st_speed_scale.mark()
        self._planner_speed_scale = float(msg.data)

    def _cb_strategy(self, msg: Float64) -> None:
        self._st_strategy.mark()
        self._strategy_mul = float(msg.data)

    def _cb_speed_cond(self, msg: UInt8) -> None:
        self._st_speed_cond.mark()
        self._speed_cond = int(msg.data)

    def _update_tf(self) -> None:
        try:
            tf = self._tf_buf.lookup_transform(
                self._map_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except Exception:
            self._tf_ok = False
            return

        self._tf_ok = True
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._pose_x = x
        self._pose_y = y
        self._pose_yaw_deg = _rad2deg(yaw)

        now = time.monotonic()
        if self._last_tf_xy is not None and self._last_tf_mono is not None:
            dt = now - self._last_tf_mono
            if dt > 1e-3:
                dx = x - self._last_tf_xy[0]
                dy = y - self._last_tf_xy[1]
                self._tf_speed = math.hypot(dx, dy) / dt
        self._last_tf_xy = (x, y)
        self._last_tf_mono = now

    def _stanley_follow_label(self) -> str:
        if not self._tf_ok:
            return "NO_TF"
        if self._override and self._local_path_n >= 2:
            return "LOCAL_PATH"
        if self._override:
            return "STOP(override, no path)"
        return "CSV_TRACKING"

    def _planner_mode_ko(self) -> str:
        m = self._planner_mode.upper()
        return {
            "GLOBAL": "CSV 직진 (GLOBAL)",
            "AVOID": "회피 (AVOID)",
            "REJOIN": "CSV 복귀 (REJOIN)",
        }.get(m, m)

    def _control_mode_ko(self) -> str:
        if len(self._tel) >= 7:
            if self._tel[7] >= 0.5:
                return "ESTOP"
            if self._tel[6] >= 0.5:
                return "AUTO (CH5 자율)"
            return "MANUAL (CH5 수동)"
        return "control_node 없음"

    def _refresh(self) -> None:
        self._update_tf()
        lines: list[str] = []
        w = 72
        lines.append("=" * w)
        lines.append(" F1TENTH 실차 주행 모니터  (Ctrl+C 종료)")
        lines.append("=" * w)

        lines.append("[ 모드 ]")
        lines.append(f"  차량 제어     : {self._control_mode_ko()}")
        if len(self._tel) >= 10:
            lines.append(
                f"  RC CH5        : {self._tel[5]:.0f} us  "
                f"(CH1={self._tel[8]:.0f} CH2={self._tel[9]:.0f})"
            )
        elif len(self._tel) >= 6:
            lines.append(f"  RC CH5        : {self._tel[5]:.0f} us")
        lines.append(
            f"  Planner       : {self._planner_mode_ko()}  (override={self._override})"
        )
        lines.append(f"  Stanley 추종  : {self._stanley_follow_label()}")
        lines.append(
            f"  local_path    : {self._local_path_n} pts  "
            f"(age {_age_str(self._st_local_path.last_mono)})"
        )

        lines.append("")
        lines.append("[ 속도 ]")
        lines.append(
            f"  /drive 명령   : {self._drive_speed:+.2f} m/s  "
            f"(age {_age_str(self._st_drive.last_mono, stale=0.3)})"
        )
        if self._st_odom.last_mono is not None:
            lines.append(
                f"  /odom 실측    : {self._odom_v:+.2f} m/s  "
                f"(age {_age_str(self._st_odom.last_mono, stale=0.5)})"
            )
        else:
            lines.append(
                f"  TF 추정속도   : {self._tf_speed:.2f} m/s  "
                f"({'TF OK' if self._tf_ok else 'TF 없음'})"
            )
        if len(self._tel) >= 4:
            lines.append(
                f"  VESC duty     : {self._tel[2]:+.3f}  (목표 {self._tel[3]:+.3f})  "
                f"[telemetry age {_age_str(self._st_tel.last_mono)}]"
            )
        else:
            lines.append("  VESC duty     : — (control_node /vehicle/telemetry 없음)")
        if len(self._tel) >= 19:
            lines.append(
                f"  VESC 속도     : {self._tel[10]:+.2f} m/s  "
                f"(target {self._tel[11]:.2f}, err {self._tel[12]:+.2f})"
            )
            lines.append(
                f"  Speed PI      : ff {self._tel[13]:+.3f}, cmd {self._tel[14]:+.3f}, "
                f"erpm {self._tel[15]:+.0f}"
            )
            lines.append(
                f"  VESC 전원     : motor {self._tel[17]:+.2f} A, "
                f"in {self._tel[16]:+.2f} A, {self._tel[18]:.1f} V"
            )
        lines.append(
            f"  속도 배율     : strategy×{self._strategy_mul:.2f}  "
            f"planner×{self._planner_speed_scale:.2f}  cond={self._speed_cond}"
        )

        lines.append("")
        lines.append("[ 조향 ]")
        lines.append(
            f"  /drive 명령   : {_rad2deg(self._drive_steer):+.1f}°  "
            f"({self._drive_steer:+.3f} rad)"
        )
        if len(self._tel) >= 5:
            esp = self._tel[4]
            lines.append(
                f"  ESP S: 전송    : {esp:+.3f}  "
                f"(서보 약 {_esp_steer_to_servo_deg(esp):.0f}°)"
            )
        else:
            lines.append("  ESP S: 전송    : — (control_node 필요)")

        lines.append("")
        lines.append("[ 위치 (map) ]")
        if self._tf_ok:
            lines.append(
                f"  pose          : x={self._pose_x:.2f} y={self._pose_y:.2f}  "
                f"yaw={self._pose_yaw_deg:+.1f}°"
            )
        else:
            lines.append("  pose          : TF map→base_link 없음")

        lines.append("")
        lines.append("[ LiDAR / 장애물 ]")
        scan_hz = f"{self._st_scan.hz:.1f} Hz" if self._st_scan.hz > 0 else "—"
        lines.append(
            f"  /scan         : {scan_hz}  n={self._scan_n}  "
            f"age {_age_str(self._st_scan.last_mono, stale=0.3)}"
        )
        if math.isfinite(self._scan_min_m):
            lines.append(
                f"  전방 최소거리 : {self._scan_min_m:.2f} m @ {self._scan_min_deg:+.0f}°"
            )
        else:
            lines.append("  전방 최소거리 : —")
        if self._obs_count > 0:
            ox, oy = self._obs_nearest_xy
            lines.append(
                f"  /static_obs   : {self._obs_count}개  "
                f"최근접 {self._obs_nearest_m:.2f} m  laser({ox:+.2f},{oy:+.2f})  "
                f"age {_age_str(self._st_obs.last_mono)}"
            )
        else:
            lines.append(
                f"  /static_obs   : 0 (인식 없음)  age {_age_str(self._st_obs.last_mono)}"
            )
        if math.isfinite(self._fgm_dist):
            lines.append(
                f"  /fgm_target   : {self._fgm_dist:.2f} m  "
                f"heading {self._fgm_heading_deg:+.0f}°  "
                f"laser({self._fgm_x:.2f},{self._fgm_y:.2f})  "
                f"age {_age_str(self._st_fgm.last_mono)}"
            )
        else:
            lines.append(f"  /fgm_target   : —  age {_age_str(self._st_fgm.last_mono)}")

        lines.append("")
        lines.append("=" * w)

        if sys.stdout.isatty():
            sys.stdout.write("\033[2J\033[H")
        sys.stdout.write("\n".join(lines) + "\n")
        sys.stdout.flush()


def main() -> None:
    rclpy.init()
    node = DriveMonitor()
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
