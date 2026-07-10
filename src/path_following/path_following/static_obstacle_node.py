#!/usr/bin/env python3
"""
정적 장애물 노드: Map Residual (Static Map Subtraction).

대표 아이디어 (자율주행/실내 로봇에서 흔함):
  - 사전 occupancy map 의 벽은 고정으로 간주
  - LiDAR 엔드포인트를 map 프레임으로 변환
  - 맵 점유(벽) 근처 히트 → 벽으로 버리고
  - 맵 자유공간에 찍힌 히트 → 예상 밖 = 장애물
  - 장애물 히트만 클러스터링 → /static_obstacles

시뮬: gym 맵에는 장애물을 그려도 되고, 이 노드의 wall prior 는
**장애물 없는 벽 맵**(`*_walls.yaml` → `*_no_obstacle.png`)을 써야
그린 박스가 벽으로 흡수되지 않는다.

**회피 타이밍·코리도는 local_planner_node CFG.**
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration as MsgDuration
from PIL import Image
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from path_following.track_sliding import param_bool


# ============================================================
# USER TUNING — 맵 잔차 장애 검출 (게이트는 local_planner)
# ============================================================
CFG = {
    "laser_frame": "laser",
    "map_frame": "map",
    "scan_topic": "/scan",
    "obstacles_topic": "/static_obstacles",
    "markers_topic": "/visualization_marker_array",
    # 벽 prior (장애물 없는 맵 YAML). 실차: cartographer rosmap.
    "map_yaml": (
        "/home/nvidia/f1tenth_ajou/maps/"
        "cartographer_map_20260710_210821_rosmap.yaml"
    ),
    # 맵 벽 셀과의 매칭 반경 [m] — 로컬라이즈 오차 시 벽이 장애로 새는 것 억제
    "wall_match_radius_m": 0.35,
    "tf_timeout_sec": 0.10,
    # --- clustering (맵에 없는 히트만) ---
    "cluster_gap_threshold_m": 0.28,
    "min_cluster_points": 8,
    "max_obstacle_size_m": 0.85,
    "min_obstacle_size_m": 0.12,
    "log_detections": True,
    "log_throttle_sec": 2.0,
}


class StaticMap:
    """ROS map YAML + PNG → 팽창된 벽 occupancy."""

    def __init__(self, yaml_path: str, wall_match_radius_m: float):
        path = Path(yaml_path).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f"map yaml not found: {path}")

        with path.open("r", encoding="utf-8") as f:
            meta = yaml.safe_load(f)

        img_name = str(meta["image"])
        img_path = Path(img_name)
        if not img_path.is_absolute():
            img_path = path.parent / img_path
        if not img_path.is_file():
            raise FileNotFoundError(f"map image not found: {img_path}")

        self.resolution = float(meta["resolution"])
        origin = meta["origin"]
        self.origin_x = float(origin[0])
        self.origin_y = float(origin[1])
        self.negate = int(meta.get("negate", 0))
        self.occupied_thresh = float(meta.get("occupied_thresh", 0.65))

        gray = np.asarray(Image.open(img_path).convert("L"), dtype=np.float64)
        # ROS map_server: negate=0 → black(0) = occupied
        if self.negate:
            occ_prob = gray / 255.0
        else:
            occ_prob = (255.0 - gray) / 255.0
        occupied = occ_prob >= self.occupied_thresh

        r_cells = max(0, int(math.ceil(wall_match_radius_m / self.resolution)))
        self.wall = self._dilate(occupied, r_cells)
        self.height, self.width = self.wall.shape
        self.image_path = str(img_path)
        self.yaml_path = str(path)
        self.wall_match_radius_m = wall_match_radius_m
        self.dilate_cells = r_cells

    @staticmethod
    def _dilate(mask: np.ndarray, radius: int) -> np.ndarray:
        if radius <= 0:
            return mask.astype(bool, copy=True)
        ys, xs = np.where(mask)
        out = np.zeros_like(mask, dtype=bool)
        h, w = mask.shape
        r2 = radius * radius
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dx * dx + dy * dy > r2:
                    continue
                yy = ys + dy
                xx = xs + dx
                valid = (yy >= 0) & (yy < h) & (xx >= 0) & (xx < w)
                out[yy[valid], xx[valid]] = True
        return out

    def world_to_cell(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        # ROS: row 0 = top of image = origin_y + height*res
        col = np.floor((x - self.origin_x) / self.resolution).astype(np.int64)
        row = np.floor(
            (self.origin_y + self.height * self.resolution - y) / self.resolution
        ).astype(np.int64)
        return row, col

    def is_wall(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """맵 프레임 (x,y)가 팽창 벽이면 True."""
        row, col = self.world_to_cell(x, y)
        inside = (
            (row >= 0)
            & (row < self.height)
            & (col >= 0)
            & (col < self.width)
        )
        out = np.ones(x.shape, dtype=bool)  # 맵 밖 = 벽으로 취급(오검출 억제)
        out[inside] = self.wall[row[inside], col[inside]]
        return out


class StaticObstacleNode(Node):
    def __init__(self):
        super().__init__("static_obstacle_node")
        for key, value in CFG.items():
            self.declare_parameter(key, value)

        self._laser_frame = str(self.get_parameter("laser_frame").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self.cluster_gap_threshold_m = float(
            self.get_parameter("cluster_gap_threshold_m").value
        )
        self.min_cluster_points = max(
            3, int(self.get_parameter("min_cluster_points").value)
        )
        self.max_obstacle_size_m = float(
            self.get_parameter("max_obstacle_size_m").value
        )
        self.min_obstacle_size_m = float(
            self.get_parameter("min_obstacle_size_m").value
        )
        self.tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        self.log_throttle_ns = int(
            max(0.1, float(self.get_parameter("log_throttle_sec").value)) * 1e9
        )
        self._log_detections = param_bool(self.get_parameter("log_detections").value)
        self._last_detect_log_ns = 0
        self._last_tf_warn_ns = 0

        map_yaml = str(self.get_parameter("map_yaml").value)
        wall_r = max(0.0, float(self.get_parameter("wall_match_radius_m").value))
        self.static_map = StaticMap(map_yaml, wall_r)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        scan_topic = self.get_parameter("scan_topic").value
        markers_topic = self.get_parameter("markers_topic").value
        obstacles_topic = self.get_parameter("obstacles_topic").value

        self.subscription = self.create_subscription(
            LaserScan, scan_topic, self.listener_callback, 10
        )
        self.marker_pub = self.create_publisher(MarkerArray, markers_topic, 10)
        self.obstacle_pub = self.create_publisher(Float32MultiArray, obstacles_topic, 10)

        self.get_logger().info(
            "static_obstacle: Map Residual (static map subtraction) | "
            f"walls={self.static_map.yaml_path} "
            f"img={Path(self.static_map.image_path).name} "
            f"match_r={self.static_map.wall_match_radius_m:.2f}m "
            f"({self.static_map.dilate_cells} cells) "
            f"frame={self._map_frame}←{self._laser_frame}"
        )

    def _publish_empty_obstacles(self) -> None:
        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        self.marker_pub.publish(marker_array)
        obs_msg = Float32MultiArray()
        obs_msg.data = []
        self.obstacle_pub.publish(obs_msg)

    def _valid_range(self, r: float) -> bool:
        return math.isfinite(r) and r > 0.05

    def _lookup_laser_to_map(self):
        try:
            return self.tf_buffer.lookup_transform(
                self._map_frame,
                self._laser_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout),
            )
        except TransformException:
            return None

    @staticmethod
    def _transform_xy(
        t, lx: np.ndarray, ly: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        q = t.transform.rotation
        # yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        c, s = math.cos(yaw), math.sin(yaw)
        tx = t.transform.translation.x
        ty = t.transform.translation.y
        mx = c * lx - s * ly + tx
        my = s * lx + c * ly + ty
        return mx, my

    def _cluster_xy(self, px: np.ndarray, py: np.ndarray) -> list[tuple[np.ndarray, np.ndarray]]:
        """레이저 프레임 점열을 인접 거리로 클러스터."""
        n = int(px.size)
        if n == 0:
            return []
        order = np.argsort(np.arctan2(py, px))
        px = px[order]
        py = py[order]
        clusters: list[tuple[np.ndarray, np.ndarray]] = []
        start = 0
        for i in range(1, n):
            if math.hypot(px[i] - px[i - 1], py[i] - py[i - 1]) > self.cluster_gap_threshold_m:
                if i - start >= self.min_cluster_points:
                    clusters.append((px[start:i].copy(), py[start:i].copy()))
                start = i
        if n - start >= self.min_cluster_points:
            clusters.append((px[start:].copy(), py[start:].copy()))
        return clusters

    def listener_callback(self, msg: LaserScan) -> None:
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        if ranges.size == 0:
            self._publish_empty_obstacles()
            return

        tf = self._lookup_laser_to_map()
        if tf is None:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_tf_warn_ns > 2_000_000_000:
                self.get_logger().warn(
                    f"TF {self._map_frame}←{self._laser_frame} 없음 — 장애 미발행"
                )
                self._last_tf_warn_ns = now_ns
            self._publish_empty_obstacles()
            return

        angle_min = float(msg.angle_min)
        angle_inc = float(msg.angle_increment)
        idx = np.arange(ranges.size, dtype=np.float64)
        valid = np.isfinite(ranges) & (ranges > 0.05) & (ranges < float(msg.range_max))
        if not np.any(valid):
            self._publish_empty_obstacles()
            return

        r = ranges[valid]
        th = angle_min + idx[valid] * angle_inc
        lx = r * np.cos(th)
        ly = r * np.sin(th)
        mx, my = self._transform_xy(tf, lx, ly)

        # 맵 벽(팽창)에 매칭되면 버리고, 나머지가 장애 후보
        wall_hit = self.static_map.is_wall(mx, my)
        obs_mask = ~wall_hit
        if not np.any(obs_mask):
            self._publish_empty_obstacles()
            return

        ox = lx[obs_mask]
        oy = ly[obs_mask]
        clusters = self._cluster_xy(ox, oy)
        if not clusters:
            self._publish_empty_obstacles()
            return

        now_msg = self.get_clock().now().to_msg()
        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        obstacle_data_list: list[float] = []
        final_obstacle_count = 0
        nearest_logic = None

        for cidx, (px_arr, py_arr) in enumerate(clusters):
            d2 = px_arr * px_arr + py_arr * py_arr
            kmin = int(np.argmin(d2))
            logic_x = float(px_arr[kmin])
            logic_y = float(py_arr[kmin])

            min_x, max_x = float(np.min(px_arr)), float(np.max(px_arr))
            min_y, max_y = float(np.min(py_arr)), float(np.max(py_arr))
            size_x = max_x - min_x
            size_y = max_y - min_y
            if size_x > self.max_obstacle_size_m or size_y > self.max_obstacle_size_m:
                continue
            span_m = max(size_x, size_y)
            if span_m < self.min_obstacle_size_m:
                continue

            radius = span_m / 2.0
            obstacle_data_list.extend([float(cidx), logic_x, logic_y, radius])
            d = math.hypot(logic_x, logic_y)
            if nearest_logic is None or d < nearest_logic[2]:
                nearest_logic = (logic_x, logic_y, d)

            marker = Marker()
            marker.header.frame_id = self._laser_frame
            marker.header.stamp = now_msg
            marker.ns = "obstacles"
            marker.id = cidx
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = (min_x + max_x) / 2.0
            marker.pose.position.y = (min_y + max_y) / 2.0
            marker.pose.position.z = 0.0
            marker.scale.x = max(size_x, 0.1)
            marker.scale.y = max(size_y, 0.1)
            marker.scale.z = 0.2
            marker.color.a = 0.8
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.lifetime = MsgDuration(sec=0, nanosec=200000000)
            marker_array.markers.append(marker)
            final_obstacle_count += 1

        self.marker_pub.publish(marker_array)
        obs_msg = Float32MultiArray()
        obs_msg.data = obstacle_data_list
        self.obstacle_pub.publish(obs_msg)

        if final_obstacle_count > 0 and self._log_detections:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_detect_log_ns >= self.log_throttle_ns:
                if nearest_logic is not None:
                    nx, ny, nd = nearest_logic
                    self.get_logger().info(
                        "맵잔차 장애 "
                        f"{final_obstacle_count}개 "
                        f"(최근접: x={nx:.2f}m, y={ny:.2f}m, d={nd:.2f}m) "
                        "→ /static_obstacles"
                    )
                else:
                    self.get_logger().info(
                        f"맵잔차 장애 {final_obstacle_count}개 → /static_obstacles"
                    )
                self._last_detect_log_ns = now_ns


def main(args=None):
    rclpy.init(args=args)
    node = StaticObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
