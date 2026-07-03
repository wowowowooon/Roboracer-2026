#!/usr/bin/env python3
"""
정적 장애물 노드: /scan 클러스터링 → /static_obstacles 발행.

**게이트·코리도·전방 거리 등 사용 타이밍은 local_planner_node CFG 에서만 조정.**
여기는 클러스터링·크기/형상 필터만.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration as MsgDuration

from path_following.track_sliding import param_bool


# ============================================================
# USER TUNING — 클러스터링·형상 (게이트는 local_planner)
# ============================================================
CFG = {
    "laser_frame": "laser",
    "scan_topic": "/scan",
    "obstacles_topic": "/static_obstacles",
    "markers_topic": "/visualization_marker_array",
    # 인접 빔 사이 공간 gap — 같은 물체로 묶을 최대 거리
    "cluster_gap_threshold_m": 0.28,
    # Hokuyo ~0.25°/beam: 1m에서 빔 간격 ≈4mm → 작은 박스도 6~8점 가능
    "min_cluster_points": 6,
    "max_obstacle_size_m": 0.85,
    # span(긴 변) 기준 — 기둥·다리 등 한쪽이 얇은 클러스터도 통과
    "min_obstacle_size_m": 0.05,
    "elongated_cluster_max_ratio": 0.0,
    "flat_range_std_max_m": 0.0,
    "flat_wall_min_span_m": 0.42,
    "log_detections": True,
    "log_throttle_sec": 2.0,
}


class StaticObstacleNode(Node):
    def __init__(self):
        super().__init__("static_obstacle_node")
        for key, value in CFG.items():
            self.declare_parameter(key, value)

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
        self.log_throttle_ns = int(
            max(0.1, float(self.get_parameter("log_throttle_sec").value)) * 1e9
        )
        self._log_detections = param_bool(self.get_parameter("log_detections").value)
        self._elong_ratio = float(
            self.get_parameter("elongated_cluster_max_ratio").value
        )
        self._flat_std_max = float(self.get_parameter("flat_range_std_max_m").value)
        self._flat_min_span = float(self.get_parameter("flat_wall_min_span_m").value)
        self._laser_frame = str(self.get_parameter("laser_frame").value)
        self._last_detect_log_ns = 0

        scan_topic = self.get_parameter("scan_topic").value
        markers_topic = self.get_parameter("markers_topic").value
        obstacles_topic = self.get_parameter("obstacles_topic").value

        self.subscription = self.create_subscription(
            LaserScan,
            scan_topic,
            self.listener_callback,
            10,
        )
        self.marker_pub = self.create_publisher(MarkerArray, markers_topic, 10)
        self.obstacle_pub = self.create_publisher(Float32MultiArray, obstacles_topic, 10)
        self.get_logger().info(
            "static_obstacle: scan clustering only — gate/timing → local_planner_node"
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
        return math.isfinite(r) and r > 0.0

    @staticmethod
    def _beam_xy(r: float, angle_min: float, angle_increment: float, index: int) -> tuple[float, float]:
        th = angle_min + index * angle_increment
        return r * math.cos(th), r * math.sin(th)

    def _cluster_scan(self, ranges: np.ndarray, angle_min: float, angle_increment: float) -> list:
        """
        원본 스캔 인덱스 연속 구간 기준 클러스터링.
        (valid 포인트만 flatten 하면 invalid 구간에서 같은 물체가 쪼개짐)
        """
        n = len(ranges)
        clusters: list = []
        i = 0
        while i < n:
            while i < n and not self._valid_range(ranges[i]):
                i += 1
            if i >= n:
                break
            seg_start = i
            while i < n and self._valid_range(ranges[i]):
                i += 1
            seg_end = i

            sub_start = seg_start
            for j in range(seg_start + 1, seg_end):
                px0, py0 = self._beam_xy(
                    float(ranges[j - 1]), angle_min, angle_increment, j - 1
                )
                px1, py1 = self._beam_xy(
                    float(ranges[j]), angle_min, angle_increment, j
                )
                if math.hypot(px1 - px0, py1 - py0) > self.cluster_gap_threshold_m:
                    self._append_cluster_if_valid(
                        ranges, angle_min, angle_increment, sub_start, j, clusters
                    )
                    sub_start = j
            self._append_cluster_if_valid(
                ranges, angle_min, angle_increment, sub_start, seg_end, clusters
            )
        return clusters

    def _append_cluster_if_valid(
        self,
        ranges: np.ndarray,
        angle_min: float,
        angle_increment: float,
        start: int,
        end: int,
        clusters: list,
    ) -> None:
        if end - start < self.min_cluster_points:
            return
        idx_arr = np.arange(start, end, dtype=np.int64)
        r_arr = ranges[start:end].astype(np.float64)
        theta = angle_min + idx_arr.astype(np.float64) * angle_increment
        px = r_arr * np.cos(theta)
        py = r_arr * np.sin(theta)
        clusters.append((idx_arr, r_arr, px, py))

    def listener_callback(self, msg):
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        now_msg = self.get_clock().now().to_msg()

        if ranges.size == 0:
            self._publish_empty_obstacles()
            return

        valid_clusters = self._cluster_scan(ranges, angle_min, angle_increment)
        if not valid_clusters:
            self._publish_empty_obstacles()
            return

        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        obstacle_data_list = []
        final_obstacle_count = 0
        nearest_logic = None

        for cidx, cluster in enumerate(valid_clusters):
            _i_arr, _r_arr, px_arr, py_arr = cluster
            d2 = px_arr * px_arr + py_arr * py_arr
            kmin = int(np.argmin(d2))
            closest_point = (float(px_arr[kmin]), float(py_arr[kmin]))

            min_x, max_x = float(np.min(px_arr)), float(np.max(px_arr))
            min_y, max_y = float(np.min(py_arr)), float(np.max(py_arr))
            size_x = max_x - min_x
            size_y = max_y - min_y
            vis_center_x = (min_x + max_x) / 2.0
            vis_center_y = (min_y + max_y) / 2.0
            logic_x = closest_point[0]
            logic_y = closest_point[1]

            if size_x > self.max_obstacle_size_m or size_y > self.max_obstacle_size_m:
                continue
            span_m = max(size_x, size_y)
            if span_m < self.min_obstacle_size_m:
                continue

            if self._elong_ratio > 0.0:
                mn_dim = max(min(size_x, size_y), 1e-6)
                if span_m / mn_dim >= self._elong_ratio:
                    continue
            if self._flat_std_max > 0.0:
                std_r = float(np.std(_r_arr))
                if std_r < self._flat_std_max and span_m >= self._flat_min_span:
                    continue

            radius = max(size_x, size_y) / 2.0
            obstacle_data_list.extend([float(cidx), logic_x, logic_y, radius])
            d = math.hypot(logic_x, logic_y)
            if nearest_logic is None or d < nearest_logic[2]:
                nearest_logic = (logic_x, logic_y, d)

            vis_size_x = size_x if size_x >= 0.1 else 0.1
            vis_size_y = size_y if size_y >= 0.1 else 0.1

            marker = Marker()
            marker.header.frame_id = self._laser_frame
            marker.header.stamp = now_msg
            marker.ns = "obstacles"
            marker.id = cidx
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = vis_center_x
            marker.pose.position.y = vis_center_y
            marker.pose.position.z = 0.0
            marker.scale.x = vis_size_x
            marker.scale.y = vis_size_y
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
                        "장애물 "
                        f"{final_obstacle_count}개 클러스터 "
                        f"(최근접: x={nx:.2f}m, y={ny:.2f}m, d={nd:.2f}m) "
                        "→ /static_obstacles"
                    )
                else:
                    self.get_logger().info(
                        f"장애물 {final_obstacle_count}개 → /static_obstacles"
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
