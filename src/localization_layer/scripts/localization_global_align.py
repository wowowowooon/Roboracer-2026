#!/usr/bin/env python3
"""Auto localise on saved map: move slightly -> scan-match pose -> restart Cartographer."""

from __future__ import annotations

import math
import os
import threading
import time

import numpy as np
import rclpy
import yaml
from cartographer_ros_msgs.srv import FinishTrajectory, GetTrajectoryStates, StartTrajectory
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Quaternion, TransformStamped
from nav_msgs.msg import OccupancyGrid
from PIL import Image
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

_TRAJECTORY_ACTIVE = 0
_LIDAR_X_IN_BASE = 0.31
_LIDAR_Y_IN_BASE = 0.0


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _load_map_grid(yaml_path: str) -> OccupancyGrid:
    with open(yaml_path, 'r', encoding='utf-8') as f:
        meta = yaml.safe_load(f) or {}

    image_path = meta.get('image', '')
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(yaml_path), image_path)
    if not os.path.isfile(image_path):
        stem, _ = os.path.splitext(image_path)
        for ext in ('.pgm', '.png'):
            alt = stem + ext
            if os.path.isfile(alt):
                image_path = alt
                break
    if not os.path.isfile(image_path):
        raise FileNotFoundError(image_path)

    resolution = float(meta.get('resolution', 0.05))
    origin = meta.get('origin', [0.0, 0.0, 0.0])
    negate = int(meta.get('negate', 0))
    occ_t = float(meta.get('occupied_thresh', 0.65))
    free_t = float(meta.get('free_thresh', 0.196))

    img = np.array(Image.open(image_path).convert('L'), dtype=np.float32)
    occ_prob = img / 255.0 if negate else 1.0 - (img / 255.0)
    grid = np.full(img.shape, -1, dtype=np.int8)
    grid[occ_prob > occ_t] = 100
    grid[occ_prob < free_t] = 0

    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.resolution = resolution
    msg.info.width = int(img.shape[1])
    msg.info.height = int(img.shape[0])
    msg.info.origin.position.x = float(origin[0])
    msg.info.origin.position.y = float(origin[1])
    msg.info.origin.position.z = float(origin[2] if len(origin) > 2 else 0.0)
    msg.data = grid[::-1, :].reshape(-1).tolist()
    lookup = np.array(msg.data, dtype=np.int16).reshape(
        (msg.info.height, msg.info.width)
    )
    return msg, lookup


def _grid_lookup(grid: np.ndarray, msg: OccupancyGrid, wx: float, wy: float) -> int:
    gx = int((wx - msg.info.origin.position.x) / msg.info.resolution)
    gy = int((wy - msg.info.origin.position.y) / msg.info.resolution)
    if gx < 0 or gy < 0 or gx >= msg.info.width or gy >= msg.info.height:
        return -1
    return int(grid[gy, gx])


def _score_pose(
    scan: LaserScan,
    grid: np.ndarray,
    map_msg: OccupancyGrid,
    x: float,
    y: float,
    yaw: float,
    *,
    ray_stride: int = 1,
) -> float:
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    score = 0.0
    used = 0
    for i in range(0, len(scan.ranges), ray_stride):
        rng = scan.ranges[i]
        if not math.isfinite(rng) or rng < scan.range_min or rng > scan.range_max:
            continue
        angle = scan.angle_min + i * scan.angle_increment
        lx = _LIDAR_X_IN_BASE + rng * math.cos(angle)
        ly = _LIDAR_Y_IN_BASE + rng * math.sin(angle)
        wx = x + cos_y * lx - sin_y * ly
        wy = y + sin_y * lx + cos_y * ly
        occ = _grid_lookup(grid, map_msg, wx, wy)
        if occ < 0:
            continue
        used += 1
        if occ >= 50:
            score += 2.0
        elif occ == 0:
            score -= 0.4
    return float('-inf') if used < 8 else score / used


def _on_track_cell(grid: np.ndarray, gx: int, gy: int) -> bool:
    """Free cell adjacent to a wall — search only on drivable corridor."""
    h, w = grid.shape
    if grid[gy, gx] != 0:
        return False
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            ny, nx = gy + dy, gx + dx
            if 0 <= ny < h and 0 <= nx < w and grid[ny, nx] >= 50:
                return True
    return False


def _search_pose(
    scan: LaserScan,
    grid: np.ndarray,
    map_msg: OccupancyGrid,
    *,
    xy_step: float,
    yaw_step_deg: float,
    ray_stride: int,
) -> tuple[float, float, float, float, float, float]:
    res = map_msg.info.resolution
    ox = map_msg.info.origin.position.x
    oy = map_msg.info.origin.position.y
    step_cells = max(1, int(round(xy_step / res)))
    yaw_step = math.radians(yaw_step_deg)

    best = (float('-inf'), 0.0, 0.0, 0.0)
    second = (float('-inf'), 0.0, 0.0, 0.0)
    h, w = grid.shape
    for gy in range(0, h, step_cells):
        for gx in range(0, w, step_cells):
            if not _on_track_cell(grid, gx, gy):
                continue
            wx = ox + (gx + 0.5) * res
            wy = oy + (gy + 0.5) * res
            yaw = 0.0
            while yaw < 2.0 * math.pi - 1e-9:
                s = _score_pose(
                    scan, grid, map_msg, wx, wy, yaw, ray_stride=ray_stride
                )
                if s > best[0]:
                    second = best
                    best = (s, wx, wy, yaw)
                elif s > second[0]:
                    second = (s, wx, wy, yaw)
                yaw += yaw_step

    return best[1], best[2], best[3], best[0], second[0]


def _refine_pose(
    scan: LaserScan,
    grid: np.ndarray,
    map_msg: OccupancyGrid,
    x: float,
    y: float,
    yaw: float,
    *,
    xy_window_m: float,
    xy_step_m: float,
    yaw_window_deg: float,
    yaw_step_deg: float,
) -> tuple[float, float, float, float]:
    best = (float('-inf'), x, y, yaw)
    yaw_step = math.radians(yaw_step_deg)
    yaw_window = math.radians(yaw_window_deg)
    dyaw = -yaw_window
    while dyaw <= yaw_window + 1e-9:
        cyaw = yaw + dyaw
        dy = -xy_window_m
        while dy <= xy_window_m + 1e-9:
            dx = -xy_window_m
            while dx <= xy_window_m + 1e-9:
                s = _score_pose(scan, grid, map_msg, x + dx, y + dy, cyaw)
                if s > best[0]:
                    best = (s, x + dx, y + dy, cyaw)
                dx += xy_step_m
            dy += xy_step_m
        dyaw += yaw_step
    return best[1], best[2], best[3], best[0]


class LocalizationGlobalAlign(Node):
    def __init__(self):
        super().__init__('localization_global_align')

        self.declare_parameter('pbstream_filename', '')
        self.declare_parameter('configuration_directory', '')
        self.declare_parameter('configuration_basename', 'cartographer_2d_localization.lua')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('min_scan_count', 5)
        self.declare_parameter('motion_scan_delta', 0.8)
        self.declare_parameter('min_align_score', 0.45)
        self.declare_parameter('min_score_margin', 0.12)
        self.declare_parameter('coarse_xy_step_m', 0.35)
        self.declare_parameter('coarse_yaw_step_deg', 20.0)
        self.declare_parameter('fine_xy_window_m', 0.55)
        self.declare_parameter('fine_xy_step_m', 0.07)
        self.declare_parameter('fine_yaw_window_deg', 22.0)
        self.declare_parameter('fine_yaw_step_deg', 3.0)

        pbstream = self.get_parameter('pbstream_filename').value
        self._config_dir = self.get_parameter('configuration_directory').value
        self._config_base = self.get_parameter('configuration_basename').value
        self._min_scans = int(self.get_parameter('min_scan_count').value)
        self._motion_delta = float(self.get_parameter('motion_scan_delta').value)
        self._min_score = float(self.get_parameter('min_align_score').value)
        self._min_margin = float(self.get_parameter('min_score_margin').value)

        map_yaml = self._resolve_map_yaml(pbstream)
        self._map_msg, self._grid = _load_map_grid(map_yaml)

        self._service_cb = MutuallyExclusiveCallbackGroup()
        self._finish = self.create_client(
            FinishTrajectory, '/finish_trajectory', callback_group=self._service_cb
        )
        self._start = self.create_client(
            StartTrajectory, '/start_trajectory', callback_group=self._service_cb
        )
        self._states = self.create_client(
            GetTrajectoryStates, '/get_trajectory_states', callback_group=self._service_cb
        )

        scan_topic = self.get_parameter('scan_topic').value
        self._latest_scan: LaserScan | None = None
        self._prev_ranges: list[float] | None = None
        self._scan_count = 0
        self._motion_accum = 0.0
        self._aligned = False
        self._cartographer_active = False
        self._busy = False
        self._pending_pose: tuple[float, float, float] | None = None
        self._hold_x = 0.0
        self._hold_y = 0.0
        self._hold_yaw = 0.0
        self._lock = threading.Lock()
        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(LaserScan, scan_topic, self._on_scan, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self._on_initialpose, 10
        )
        self.create_timer(0.2, self._on_tick)
        self.create_timer(0.35, self._poll_trajectory_states)
        self.create_timer(1.0 / 30.0, self._publish_hold_tf)

        self.get_logger().info(
            f'Auto localise ready ({map_yaml}). '
            '조금 움직이면 맵에서 위치를 자동으로 찾습니다.'
        )

    def _resolve_map_yaml(self, pbstream: str) -> str:
        stem = os.path.splitext(pbstream)[0]
        origin = f'{stem}_origin.yaml'
        if os.path.isfile(origin):
            with open(origin, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            ros_map = data.get('ros_map_yaml')
            if isinstance(ros_map, str) and os.path.isfile(ros_map):
                return ros_map
        for cand in (f'{stem}_rosmap.yaml', f'{stem}.yaml'):
            if os.path.isfile(cand):
                return cand
        raise RuntimeError(f'No map yaml for {pbstream}')

    def _on_scan(self, msg: LaserScan) -> None:
        self._latest_scan = msg
        self._scan_count += 1
        if self._prev_ranges is None or len(self._prev_ranges) != len(msg.ranges):
            self._prev_ranges = list(msg.ranges)
            return
        delta = 0.0
        for a, b in zip(self._prev_ranges, msg.ranges):
            if math.isfinite(a) and math.isfinite(b):
                delta += abs(a - b)
        self._motion_accum += delta
        self._prev_ranges = list(msg.ranges)

    def _on_initialpose(self, msg: PoseWithCovarianceStamped) -> None:
        if msg.header.frame_id and msg.header.frame_id not in ('map', ''):
            return
        p = msg.pose.pose
        yaw = math.atan2(
            2.0 * (p.orientation.w * p.orientation.z + p.orientation.x * p.orientation.y),
            1.0 - 2.0 * (p.orientation.y ** 2 + p.orientation.z ** 2),
        )
        with self._lock:
            self._pending_pose = (p.position.x, p.position.y, yaw)
            self._hold_x, self._hold_y, self._hold_yaw = p.position.x, p.position.y, yaw
            self._aligned = False
        self.get_logger().info('RViz pose queued — 위치 재설정')

    def _call_service(self, client, request, timeout: float = 8.0):
        future = client.call_async(request)
        done = threading.Event()
        result = {'resp': None}

        def _done(fut):
            try:
                result['resp'] = fut.result()
            except Exception as exc:
                self.get_logger().error(f'service error: {exc}')
            finally:
                done.set()

        future.add_done_callback(_done)
        if not done.wait(timeout):
            return None
        return result['resp']

    def _poll_trajectory_states(self) -> None:
        if not self._states.service_is_ready():
            return
        resp = self._call_service(self._states, GetTrajectoryStates.Request(), 2.0)
        if resp is None or resp.status.code != 0:
            return
        self._cartographer_active = any(
            s == _TRAJECTORY_ACTIVE for s in resp.trajectory_states.trajectory_state
        )

    def _publish_hold_tf(self) -> None:
        if self._cartographer_active:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self._hold_x
        t.transform.translation.y = self._hold_y
        t.transform.rotation.z = math.sin(self._hold_yaw * 0.5)
        t.transform.rotation.w = math.cos(self._hold_yaw * 0.5)
        self._tf_broadcaster.sendTransform(t)

    def _finish_all(self) -> bool:
        resp = self._call_service(self._states, GetTrajectoryStates.Request(), 3.0)
        if resp is None or resp.status.code != 0:
            return False
        ids = [
            int(i)
            for i, s in zip(resp.trajectory_states.trajectory_id,
                            resp.trajectory_states.trajectory_state)
            if s == _TRAJECTORY_ACTIVE
        ]
        if not ids:
            return True
        for tid in sorted(ids, reverse=True):
            req = FinishTrajectory.Request()
            req.trajectory_id = tid
            fr = self._call_service(self._finish, req)
            if fr is None or fr.status.code != 0:
                return False
        time.sleep(0.25)
        return True

    def _start_trajectory(self, x: float, y: float, yaw: float) -> bool:
        req = StartTrajectory.Request()
        req.configuration_directory = self._config_dir
        req.configuration_basename = self._config_base
        req.use_initial_pose = True
        req.relative_to_trajectory_id = 0
        req.initial_pose = Pose()
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.orientation = _yaw_to_quat(yaw)
        resp = self._call_service(self._start, req)
        if resp is None or resp.status.code != 0:
            msg = resp.status.message if resp is not None else 'timeout'
            self.get_logger().error(f'start_trajectory failed: {msg}')
            return False
        self.get_logger().info(
            f'위치 찾음 ✓  x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.0f}°'
        )
        return True

    def _align_from_scan(self, scan: LaserScan) -> tuple[float, float, float] | None:
        cx, cy, cyaw, cs, s2 = _search_pose(
            scan,
            self._grid,
            self._map_msg,
            xy_step=float(self.get_parameter('coarse_xy_step_m').value),
            yaw_step_deg=float(self.get_parameter('coarse_yaw_step_deg').value),
            ray_stride=3,
        )
        if not math.isfinite(cs):
            return None
        if cs - s2 < self._min_margin:
            self.get_logger().warn(
                f'유사 구간 여러 개 (score {cs:.2f} vs {s2:.2f}) — '
                'RViz 2D Pose Estimate 로 대략 위치 찍어주세요'
            )
            return None
        fx, fy, fyaw, fs = _refine_pose(
            scan,
            self._grid,
            self._map_msg,
            cx,
            cy,
            cyaw,
            xy_window_m=float(self.get_parameter('fine_xy_window_m').value),
            xy_step_m=float(self.get_parameter('fine_xy_step_m').value),
            yaw_window_deg=float(self.get_parameter('fine_yaw_window_deg').value),
            yaw_step_deg=float(self.get_parameter('fine_yaw_step_deg').value),
        )
        if fs < self._min_score:
            self.get_logger().warn(f'매칭 점수 낮음 ({fs:.2f}) — 더 움직여 보세요')
            return None
        self.get_logger().info(f'스캔 매칭 score {cs:.2f} -> {fs:.2f}')
        return fx, fy, fyaw

    def _apply_pose(self, pose: tuple[float, float, float], source: str) -> bool:
        resp = self._call_service(self._states, GetTrajectoryStates.Request(), 3.0)
        has_active = False
        if resp is not None and resp.status.code == 0:
            has_active = any(
                s == _TRAJECTORY_ACTIVE for s in resp.trajectory_states.trajectory_state
            )
        if has_active and not self._finish_all():
            self.get_logger().error('finish_trajectory 실패')
            return False
        ok = self._start_trajectory(*pose)
        if ok:
            self._aligned = True
            self._hold_x, self._hold_y, self._hold_yaw = pose
            self.get_logger().info(f'Localization OK ({source})')
        return ok

    def _on_tick(self):
        if self._busy:
            return

        with self._lock:
            manual = self._pending_pose
            self._pending_pose = None

        if manual is not None:
            self._aligned = False
            self._busy = True
            try:
                self._apply_pose(manual, 'RViz')
            finally:
                self._busy = False
            return

        if self._aligned:
            return

        scan = self._latest_scan
        if scan is None or self._scan_count < self._min_scans:
            return
        if self._motion_accum < self._motion_delta:
            return
        if not (
            self._finish.service_is_ready()
            and self._start.service_is_ready()
            and self._states.service_is_ready()
        ):
            return

        self._busy = True
        try:
            self.get_logger().info('움직임 감지 — 맵에서 위치 검색 중...')
            pose = self._align_from_scan(scan)
            if pose is not None:
                self._apply_pose(pose, 'auto scan match')
        finally:
            self._busy = False


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationGlobalAlign()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.remove_node(node)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
