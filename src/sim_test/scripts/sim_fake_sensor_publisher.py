#!/usr/bin/env python3

import math
import os
import random
import re
import threading
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from PIL import Image
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


class FakeSensorPublisher(Node):
    def __init__(self):
        super().__init__("fake_sensor_publisher")
        # Keep defaults conservative so missing params do not overload CPU.
        self.declare_parameter("lidar_rate_hz", 8.0)
        self.declare_parameter("imu_rate_hz", 8.0)
        self.declare_parameter("odom_rate_hz", 8.0)
        self.declare_parameter("state_rate_hz", 50.0)
        self.declare_parameter("scan_range_min", 0.12)
        self.declare_parameter("scan_range_max", 8.0)
        self.declare_parameter("scan_angle_min_deg", -80.0)
        self.declare_parameter("scan_angle_max_deg", 80.0)
        self.declare_parameter("scan_angle_increment_deg", 2.0)
        self.declare_parameter("laser_offset_x_m", 0.2)
        self.declare_parameter("laser_offset_y_m", 0.0)
        self.declare_parameter("laser_offset_z_m", 0.0)
        self.declare_parameter("laser_yaw_offset_deg", 0.0)
        self.declare_parameter("scan_noise_std_m", 0.0)
        self.declare_parameter("scan_dropout_ratio", 0.0)
        self.declare_parameter("scan_outlier_ratio", 0.0)
        self.declare_parameter("scan_outlier_max_m", 0.3)
        self.declare_parameter("publish_static_tf", False)
        self.declare_parameter("world_type", "racing")
        self.declare_parameter("map_yaml_path", "")
        self.declare_parameter("centerline_csv_path", "")
        self.declare_parameter("centerline_auto_align_to_map", True)
        self.declare_parameter("centerline_offset_x_m", 0.0)
        self.declare_parameter("centerline_offset_y_m", 0.0)
        self.declare_parameter("random_seed", 42)
        self.declare_parameter("odom_velocity_scale_error", 0.0)
        self.declare_parameter("odom_yaw_rate_bias_deg", 0.0)
        self.declare_parameter("odom_yaw_rate_noise_std_deg", 0.0)
        self.declare_parameter("publish_odom_tf", True)
        self.declare_parameter("publish_imu", True)
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("odom_follow_ground_truth", False)
        self.declare_parameter("imu_yaw_bias_deg", 0.0)
        self.declare_parameter("imu_ang_vel_bias_deg", 0.0)
        self.declare_parameter("imu_ang_vel_noise_std_deg", 0.0)
        self.declare_parameter("speed_modulation_amp", 0.0)
        self.declare_parameter("slip_event_rate", 0.0)
        self.declare_parameter("slip_duration_min_sec", 0.2)
        self.declare_parameter("slip_duration_max_sec", 0.6)
        self.declare_parameter("slip_scale", 0.85)
        self.declare_parameter("map_path_speed_mps", 30.0)
        self.declare_parameter("map_path_points", 720)
        self.declare_parameter("map_min_speed_mps", 3.0)
        self.declare_parameter("map_corner_preview_m", 1.0)
        self.declare_parameter("map_corner_slowdown_gain", 15.0)
        self.declare_parameter("map_max_lateral_accel_mps2", 7.0)
        self.declare_parameter("map_yaw_preview_m", 0.8)
        self.declare_parameter("map_use_pure_pursuit", True)
        self.declare_parameter("map_use_waypoint_heading", True)
        self.declare_parameter("map_follow_waypoint_yaw", False)
        self.declare_parameter("map_waypoint_heading_gain", 4.0)
        self.declare_parameter("map_waypoint_heading_auto_offset", True)
        self.declare_parameter("map_waypoint_heading_offset_rad", 0.0)
        self.declare_parameter("pp_lookahead_m", 2.2)
        self.declare_parameter("pp_lookahead_gain", 0.12)
        self.declare_parameter("pp_max_yaw_rate_radps", 2.2)
        self.declare_parameter("pp_relock_distance_m", 12.0)
        self.declare_parameter("map_raycast_step_m", 0.05)
        self.declare_parameter("map_single_wall_inset_m", 1.2)
        self.declare_parameter("use_threaded_scan_publisher", True)
        self.declare_parameter("use_threaded_odom_publisher", True)
        self.declare_parameter("motion_source", "internal_path")
        self.declare_parameter("external_path_topic", "/local_path")
        self.declare_parameter("external_path_fallback_topic", "/recommended_path")
        self.declare_parameter("external_path_target_index", 6)
        self.declare_parameter("external_search_ahead", 120)
        self.declare_parameter("external_search_behind", 20)
        self.declare_parameter("external_relock_distance_m", 2.5)

        self.lidar_rate_hz = max(
            1.0, self.get_parameter("lidar_rate_hz").get_parameter_value().double_value
        )
        self.imu_rate_hz = max(
            1.0, self.get_parameter("imu_rate_hz").get_parameter_value().double_value
        )
        self.odom_rate_hz = max(
            1.0, self.get_parameter("odom_rate_hz").get_parameter_value().double_value
        )
        self.state_rate_hz = max(
            1.0, self.get_parameter("state_rate_hz").get_parameter_value().double_value
        )
        self.scan_range_min = self.get_parameter("scan_range_min").get_parameter_value().double_value
        self.scan_range_max = self.get_parameter("scan_range_max").get_parameter_value().double_value
        self.scan_angle_min = math.radians(
            self.get_parameter("scan_angle_min_deg").get_parameter_value().double_value
        )
        self.scan_angle_max = math.radians(
            self.get_parameter("scan_angle_max_deg").get_parameter_value().double_value
        )
        self.scan_angle_increment = math.radians(
            self.get_parameter("scan_angle_increment_deg").get_parameter_value().double_value
        )
        self.laser_offset_x_m = self.get_parameter(
            "laser_offset_x_m"
        ).get_parameter_value().double_value
        self.laser_offset_y_m = self.get_parameter(
            "laser_offset_y_m"
        ).get_parameter_value().double_value
        self.laser_offset_z_m = self.get_parameter(
            "laser_offset_z_m"
        ).get_parameter_value().double_value
        self.laser_yaw_offset = math.radians(
            self.get_parameter("laser_yaw_offset_deg").get_parameter_value().double_value
        )
        self.scan_noise_std_m = self.get_parameter("scan_noise_std_m").get_parameter_value().double_value
        self.scan_dropout_ratio = self.get_parameter("scan_dropout_ratio").get_parameter_value().double_value
        self.scan_outlier_ratio = self.get_parameter("scan_outlier_ratio").get_parameter_value().double_value
        self.scan_outlier_max_m = self.get_parameter("scan_outlier_max_m").get_parameter_value().double_value
        self.publish_static_tf = self.get_parameter(
            "publish_static_tf"
        ).get_parameter_value().bool_value
        self.world_type = self.get_parameter("world_type").get_parameter_value().string_value
        self.map_yaml_path = self.get_parameter("map_yaml_path").get_parameter_value().string_value
        self.centerline_csv_path = self.get_parameter(
            "centerline_csv_path"
        ).get_parameter_value().string_value
        self.centerline_auto_align_to_map = self.get_parameter(
            "centerline_auto_align_to_map"
        ).get_parameter_value().bool_value
        self.centerline_offset_x_m = self.get_parameter(
            "centerline_offset_x_m"
        ).get_parameter_value().double_value
        self.centerline_offset_y_m = self.get_parameter(
            "centerline_offset_y_m"
        ).get_parameter_value().double_value
        self.random_seed = self.get_parameter("random_seed").get_parameter_value().integer_value
        self.odom_velocity_scale_error = self.get_parameter(
            "odom_velocity_scale_error"
        ).get_parameter_value().double_value
        self.odom_yaw_rate_bias = math.radians(
            self.get_parameter("odom_yaw_rate_bias_deg").get_parameter_value().double_value
        )
        self.odom_yaw_rate_noise_std = math.radians(
            self.get_parameter("odom_yaw_rate_noise_std_deg").get_parameter_value().double_value
        )
        self.publish_odom_tf = self.get_parameter(
            "publish_odom_tf"
        ).get_parameter_value().bool_value
        self.publish_imu = self.get_parameter("publish_imu").get_parameter_value().bool_value
        self.publish_odom = self.get_parameter("publish_odom").get_parameter_value().bool_value
        self.odom_follow_ground_truth = self.get_parameter(
            "odom_follow_ground_truth"
        ).get_parameter_value().bool_value
        self.imu_yaw_bias = math.radians(
            self.get_parameter("imu_yaw_bias_deg").get_parameter_value().double_value
        )
        self.imu_ang_vel_bias = math.radians(
            self.get_parameter("imu_ang_vel_bias_deg").get_parameter_value().double_value
        )
        self.imu_ang_vel_noise_std = math.radians(
            self.get_parameter("imu_ang_vel_noise_std_deg").get_parameter_value().double_value
        )
        self.speed_modulation_amp = self.get_parameter(
            "speed_modulation_amp"
        ).get_parameter_value().double_value
        self.slip_event_rate = self.get_parameter("slip_event_rate").get_parameter_value().double_value
        self.slip_duration_min_sec = self.get_parameter(
            "slip_duration_min_sec"
        ).get_parameter_value().double_value
        self.slip_duration_max_sec = self.get_parameter(
            "slip_duration_max_sec"
        ).get_parameter_value().double_value
        self.slip_scale = self.get_parameter("slip_scale").get_parameter_value().double_value
        self.map_path_speed_mps = self.get_parameter("map_path_speed_mps").get_parameter_value().double_value
        if self.map_path_speed_mps > 35.0:
            raw_speed = self.map_path_speed_mps
            self.map_path_speed_mps = raw_speed / 3.6
            self.get_logger().warn(
                f"map_path_speed_mps={raw_speed:.1f} is too high for m/s. "
                f"Interpreting as km/h -> {self.map_path_speed_mps:.2f} m/s."
            )
        self.map_path_points = max(
            180, self.get_parameter("map_path_points").get_parameter_value().integer_value
        )
        self.map_min_speed_mps = self.get_parameter("map_min_speed_mps").get_parameter_value().double_value
        self.map_corner_preview_m = self.get_parameter(
            "map_corner_preview_m"
        ).get_parameter_value().double_value
        self.map_corner_slowdown_gain = self.get_parameter(
            "map_corner_slowdown_gain"
        ).get_parameter_value().double_value
        self.map_max_lateral_accel_mps2 = self.get_parameter(
            "map_max_lateral_accel_mps2"
        ).get_parameter_value().double_value
        self.map_yaw_preview_m = self.get_parameter("map_yaw_preview_m").get_parameter_value().double_value
        self.map_use_pure_pursuit = self.get_parameter(
            "map_use_pure_pursuit"
        ).get_parameter_value().bool_value
        self.map_use_waypoint_heading = self.get_parameter(
            "map_use_waypoint_heading"
        ).get_parameter_value().bool_value
        self.map_follow_waypoint_yaw = self.get_parameter(
            "map_follow_waypoint_yaw"
        ).get_parameter_value().bool_value
        # Mapping stability mode: follow centerline by monotonic s-progress only.
        # This removes controller relock/branch behavior from the pose source.
        self.map_use_pure_pursuit = False
        self.map_follow_waypoint_yaw = False
        self.map_waypoint_heading_gain = self.get_parameter(
            "map_waypoint_heading_gain"
        ).get_parameter_value().double_value
        self.map_waypoint_heading_auto_offset = self.get_parameter(
            "map_waypoint_heading_auto_offset"
        ).get_parameter_value().bool_value
        self.map_waypoint_heading_offset_rad = self.get_parameter(
            "map_waypoint_heading_offset_rad"
        ).get_parameter_value().double_value
        self.pp_lookahead_m = self.get_parameter("pp_lookahead_m").get_parameter_value().double_value
        self.pp_lookahead_gain = self.get_parameter("pp_lookahead_gain").get_parameter_value().double_value
        self.pp_max_yaw_rate_radps = self.get_parameter(
            "pp_max_yaw_rate_radps"
        ).get_parameter_value().double_value
        self.pp_relock_distance_m = self.get_parameter(
            "pp_relock_distance_m"
        ).get_parameter_value().double_value
        self.map_raycast_step_m = max(
            0.02, self.get_parameter("map_raycast_step_m").get_parameter_value().double_value
        )
        self.map_single_wall_inset_m = max(
            0.2, self.get_parameter("map_single_wall_inset_m").get_parameter_value().double_value
        )
        self.use_threaded_scan_publisher = self.get_parameter(
            "use_threaded_scan_publisher"
        ).get_parameter_value().bool_value
        self.use_threaded_odom_publisher = self.get_parameter(
            "use_threaded_odom_publisher"
        ).get_parameter_value().bool_value
        self.motion_source = (
            self.get_parameter("motion_source").get_parameter_value().string_value.strip().lower()
        )
        if self.motion_source not in ("internal_path", "local_path"):
            self.get_logger().warn(
                f"Unknown motion_source='{self.motion_source}', fallback to 'internal_path'."
            )
            self.motion_source = "internal_path"
        self.external_path_topic = self.get_parameter(
            "external_path_topic"
        ).get_parameter_value().string_value
        self.external_path_fallback_topic = self.get_parameter(
            "external_path_fallback_topic"
        ).get_parameter_value().string_value
        self.external_path_target_index = max(
            1,
            self.get_parameter("external_path_target_index").get_parameter_value().integer_value,
        )
        self.external_search_ahead = max(
            10,
            self.get_parameter("external_search_ahead").get_parameter_value().integer_value,
        )
        self.external_search_behind = max(
            0,
            self.get_parameter("external_search_behind").get_parameter_value().integer_value,
        )
        self.external_relock_distance_m = max(
            0.5,
            self.get_parameter("external_relock_distance_m").get_parameter_value().double_value,
        )
        # On this environment, ROS timers periodically stall and collapse
        # effective scan/odom rates to ~1 Hz, which looks like TF/map teleport.
        # Force dedicated publisher threads for deterministic cadence.
        if not self.use_threaded_scan_publisher:
            self.get_logger().warn(
                "use_threaded_scan_publisher=false requested, but forcing true for stability."
            )
        if not self.use_threaded_odom_publisher:
            self.get_logger().warn(
                "use_threaded_odom_publisher=false requested, but forcing true for stability."
            )
        self.use_threaded_scan_publisher = True
        self.use_threaded_odom_publisher = True
        self.state_dt = 1.0 / self.state_rate_hz
        self.scan_period_sec = 1.0 / self.lidar_rate_hz
        self.t = 0.0
        self.rng = random.Random(self.random_seed)
        self.odom_x = None
        self.odom_y = None
        self.odom_yaw = None
        self.last_odom_pub_t = None
        self.last_odom_pub_wall_ns = None
        self.gt_x = 0.0
        self.gt_y = 0.0
        self.gt_yaw = 0.0
        self.gt_v = 0.0
        self.gt_yaw_rate = 0.0
        self.gt_ax_body = 0.0
        self.gt_ay_body = 0.0
        self._slip_until_t = 0.0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_resolution = 0.05
        self.map_width = 0
        self.map_height = 0
        self.map_occ = None
        self.map_cx = 0.0
        self.map_cy = 0.0
        self.map_radius = 2.0
        self.map_omega = 0.05
        self.map_path_xy = []
        self.map_path_heading = []
        self.map_heading_offset_rad = 0.0
        self.map_path_s = []
        self.map_path_total_length = 0.0
        self.map_path_progress = 0.0
        self.map_path_is_loop = True
        self._prev_gt_yaw_for_rate = 0.0
        self._warned_no_map_path = False
        self._warned_no_external_path = False
        self.external_local_path_xy = []
        self.external_recommended_path_xy = []
        self._last_external_best_i = None
        self._last_external_path_len = 0
        self._last_scan_perf_warn_t = -1e9
        self._last_pose_jump_warn_t = -1e9
        self._scan_pub_count = 0
        self._scan_pub_elapsed_accum = 0.0
        self._timer_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._stamp_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self._scan_diag_last_wall_ns = time.monotonic_ns()
        self._last_scan_wall_ns = None
        self._timer_cb_group = ReentrantCallbackGroup()
        self._state_lock = threading.Lock()
        self._scan_thread_stop = threading.Event()
        self._scan_thread = None
        self._odom_thread_stop = threading.Event()
        self._odom_thread = None
        self._stamp_lock = threading.Lock()
        self._last_header_stamp_ns = 0

        if self.centerline_csv_path:
            self._load_centerline_csv(self.centerline_csv_path)

        if self.map_yaml_path:
            self._load_occupancy_map(self.map_yaml_path)
            self.world_type = "map"

        if self.map_occ is not None and self.map_path_total_length > 0.0:
            self._align_centerline_to_map()
        if self.world_type == "map":
            if self.map_path_total_length > 0.0:
                self.get_logger().info(
                    f"Map path ready. length={self.map_path_total_length:.2f} m, points={len(self.map_path_xy)}"
                )
            else:
                self.get_logger().error(
                    "Map path is empty. Check map_yaml_path/centerline_csv_path. "
                    "Vehicle motion will be held to avoid circular fallback."
                )

        # Cartographer's scan subscriber uses RELIABLE QoS by default.
        # Publishing BEST_EFFORT here causes QoS mismatch and drops all scan data.
        scan_qos = QoSProfile(depth=10)
        scan_qos.reliability = ReliabilityPolicy.RELIABLE
        scan_qos.durability = DurabilityPolicy.VOLATILE
        self.scan_pub = self.create_publisher(LaserScan, "/scan", scan_qos)
        self.imu_pub = None
        self.odom_pub = None
        if self.publish_imu:
            self.imu_pub = self.create_publisher(Imu, "/ebimu/imu", 10)
        if self.publish_odom:
            self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        tf_qos = QoSProfile(depth=100)
        tf_qos.reliability = ReliabilityPolicy.RELIABLE

        tf_static_qos = QoSProfile(depth=1)
        tf_static_qos.reliability = ReliabilityPolicy.RELIABLE
        tf_static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.tf_pub = self.create_publisher(TFMessage, "/tf", tf_qos)
        self.tf_static_pub = self.create_publisher(TFMessage, "/tf_static", tf_static_qos)
        self.local_path_sub = None
        self.recommended_path_sub = None
        if self.motion_source == "local_path":
            self.local_path_sub = self.create_subscription(
                Path,
                self.external_path_topic,
                self._on_local_path,
                10,
                callback_group=self._timer_cb_group,
            )
            self.recommended_path_sub = self.create_subscription(
                Path,
                self.external_path_fallback_topic,
                self._on_recommended_path,
                10,
                callback_group=self._timer_cb_group,
            )

        if self.publish_static_tf:
            self._publish_static_tfs()

        self.scan_angles = []
        current = self.scan_angle_min
        while current <= self.scan_angle_max + 1e-9:
            self.scan_angles.append(current)
            current += self.scan_angle_increment
        self.scan_angles_np = np.array(self.scan_angles, dtype=np.float32)
        self.scan_dist_samples = np.arange(
            0.0,
            self.scan_range_max + self.map_raycast_step_m,
            self.map_raycast_step_m,
            dtype=np.float32,
        )
        if self.scan_dist_samples.size == 0:
            self.scan_dist_samples = np.array([0.0], dtype=np.float32)

        self.state_timer = self.create_timer(
            self.state_dt,
            self._on_state_timer,
            callback_group=self._timer_cb_group,
            clock=self._timer_clock,
        )
        self.scan_timer = None
        self._scan_thread = threading.Thread(
            target=self._scan_publish_loop,
            name="scan_publish_loop",
            daemon=True,
        )
        self._scan_thread.start()
        self.imu_timer = None
        self.odom_timer = None
        if self.publish_imu:
            self.imu_timer = self.create_timer(
                1.0 / self.imu_rate_hz,
                self._on_imu_timer,
                callback_group=self._timer_cb_group,
                clock=self._timer_clock,
            )
        if self.publish_odom:
            self._odom_thread = threading.Thread(
                target=self._odom_publish_loop,
                name="odom_publish_loop",
                daemon=True,
            )
            self._odom_thread.start()
        self.get_logger().info(
            "Fake sensor publisher started. "
            f"world_type={self.world_type}, rates(lidar/imu/odom/state)="
            f"{self.lidar_rate_hz:.1f}/{self.imu_rate_hz:.1f}/{self.odom_rate_hz:.1f}/{self.state_rate_hz:.1f} Hz"
        )
        self.get_logger().info(
            f"Publish flags: scan=True, imu={self.publish_imu}, odom={self.publish_odom}, odom_tf={self.publish_odom_tf}"
        )
        self.get_logger().info(
            f"Scan scheduler: {'threaded' if self.use_threaded_scan_publisher else 'ros_timer'}"
        )
        if self.publish_odom:
            self.get_logger().info(
                f"Odom scheduler: {'threaded' if self.use_threaded_odom_publisher else 'ros_timer'}"
            )
        self.get_logger().info(
            "Active scan params: "
            f"range_max={self.scan_range_max:.2f} m, "
            f"angle_min/max={math.degrees(self.scan_angle_min):.1f}/{math.degrees(self.scan_angle_max):.1f} deg, "
            f"increment={math.degrees(self.scan_angle_increment):.3f} deg"
        )
        self.get_logger().info(
            "Control mode: centerline s-progress (pure_pursuit forced OFF, waypoint_yaw forced OFF)"
        )
        self.get_logger().info(
            f"Motion source: {self.motion_source}"
            + (
                f" (topic={self.external_path_topic}, fallback={self.external_path_fallback_topic}, "
                f"target_index={self.external_path_target_index}, "
                f"search=+{self.external_search_ahead}/-{self.external_search_behind})"
                if self.motion_source == "local_path"
                else ""
            )
        )

    def _yaw_to_quat(self, yaw: float):
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)
        return 0.0, 0.0, qz, qw

    def _next_header_stamp(self):
        # Keep outgoing message stamps strictly increasing to avoid Cartographer
        # rejecting out-of-order LaserScan subdivisions.
        with self._stamp_lock:
            now_ns = self._stamp_clock.now().nanoseconds
            if now_ns <= self._last_header_stamp_ns:
                now_ns = self._last_header_stamp_ns + 1_000_000  # +1 ms
            self._last_header_stamp_ns = now_ns
        return rclpy.time.Time(nanoseconds=now_ns).to_msg()

    def _publish_static_tfs(self):
        now = self._next_header_stamp()

        tf_laser = TransformStamped()
        tf_laser.header.stamp = now
        tf_laser.header.frame_id = "base_link"
        tf_laser.child_frame_id = "laser"
        tf_laser.transform.translation.x = self.laser_offset_x_m
        tf_laser.transform.translation.y = self.laser_offset_y_m
        tf_laser.transform.translation.z = self.laser_offset_z_m
        _, _, qz, qw = self._yaw_to_quat(self.laser_yaw_offset)
        tf_laser.transform.rotation.z = qz
        tf_laser.transform.rotation.w = qw

        tf_imu = TransformStamped()
        tf_imu.header.stamp = now
        tf_imu.header.frame_id = "base_link"
        tf_imu.child_frame_id = "imu_link"
        tf_imu.transform.translation.x = 0.6
        tf_imu.transform.translation.y = 0.0
        tf_imu.transform.translation.z = 0.5
        tf_imu.transform.rotation.w = 1.0

        self.tf_static_pub.publish(TFMessage(transforms=[tf_laser, tf_imu]))

    def _load_occupancy_map(self, yaml_path: str):
        with open(yaml_path, "r", encoding="utf-8") as f:
            meta = yaml.safe_load(f)

        image_path = meta.get("image", "")
        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(yaml_path), image_path)

        img = Image.open(image_path).convert("L")
        data = np.array(img, dtype=np.uint8)
        self.map_height, self.map_width = data.shape
        self.map_resolution = float(meta.get("resolution", 0.05))
        origin = meta.get("origin", [0.0, 0.0, 0.0])
        self.map_origin_x = float(origin[0])
        self.map_origin_y = float(origin[1])
        negate = int(meta.get("negate", 0))
        occ_th = float(meta.get("occupied_thresh", 0.65))

        if negate == 0:
            occ_prob = (255.0 - data.astype(np.float32)) / 255.0
        else:
            occ_prob = data.astype(np.float32) / 255.0
        self.map_occ = occ_prob >= occ_th

        width_m = self.map_width * self.map_resolution
        height_m = self.map_height * self.map_resolution

        ys, xs = np.where(self.map_occ)
        if len(xs) > 0:
            min_x = int(xs.min())
            max_x = int(xs.max())
            min_y = int(ys.min())
            max_y = int(ys.max())

            # Convert occupied bbox from image coords to world coords.
            world_x_min = self.map_origin_x + min_x * self.map_resolution
            world_x_max = self.map_origin_x + max_x * self.map_resolution
            map_y_min = self.map_height - 1 - max_y
            map_y_max = self.map_height - 1 - min_y
            world_y_min = self.map_origin_y + map_y_min * self.map_resolution
            world_y_max = self.map_origin_y + map_y_max * self.map_resolution

            span_x = max(1.0, world_x_max - world_x_min)
            span_y = max(1.0, world_y_max - world_y_min)
            self.map_cx = 0.5 * (world_x_min + world_x_max)
            self.map_cy = 0.5 * (world_y_min + world_y_max)
            self.map_radius = 0.32 * min(span_x, span_y)
            self.map_omega = 0.06
            if self.map_path_total_length <= 0.0:
                self._build_map_path_from_polar()
        else:
            self.map_cx = self.map_origin_x + 0.5 * width_m
            self.map_cy = self.map_origin_y + 0.5 * height_m
            self.map_radius = 0.25 * min(width_m, height_m)
            self.map_omega = 0.05

    def _load_centerline_csv(self, csv_path: str):
        if not os.path.exists(csv_path):
            self.get_logger().warn(f"Centerline CSV not found: {csv_path}")
            return

        points = []
        headings = []
        s_values = []
        csv_mode = "xy"
        with open(csv_path, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                if line.startswith("#"):
                    lower = line.lower()
                    # Waypoint format: s_m; x_m; y_m; ...
                    if "s_m" in lower and "x_m" in lower and "y_m" in lower:
                        csv_mode = "sxy"
                    continue
                parts = [p.strip() for p in re.split(r"[,;\s]+", line) if p.strip()]
                if len(parts) < 2:
                    continue
                try:
                    if csv_mode == "sxy" and len(parts) >= 3:
                        s_val = float(parts[0])
                        x = float(parts[1])
                        y = float(parts[2])
                        psi = float(parts[3]) if len(parts) >= 4 else float("nan")
                    else:
                        s_val = float("nan")
                        x = float(parts[0])
                        y = float(parts[1])
                        psi = float("nan")
                except ValueError:
                    continue
                points.append((x, y))
                headings.append(psi)
                s_values.append(s_val)

        # Waypoint CSV can contain unsorted rows. If s_m exists, enforce
        # monotonic ordering to prevent multi-meter path jumps.
        if csv_mode == "sxy" and len(points) >= 3:
            rows = list(zip(s_values, points, headings))
            rows = [r for r in rows if not math.isnan(r[0])]
            if len(rows) >= 3:
                rows.sort(key=lambda r: r[0])
                points = [r[1] for r in rows]
                headings = [r[2] for r in rows]

        if len(points) < 3:
            self.get_logger().warn(
                f"Centerline CSV parse failed or too short ({len(points)} pts): {csv_path}"
            )
            return

        close_gap = math.hypot(points[0][0] - points[-1][0], points[0][1] - points[-1][1])
        # Only close the path when endpoints are already close. Forcing closure
        # on open/unsorted centerlines creates multi-meter teleport jumps.
        if close_gap <= 1.0:
            if close_gap > 0.1:
                points.append(points[0])
                headings.append(headings[0] if headings else float("nan"))
            self.map_path_is_loop = True
        else:
            self.map_path_is_loop = False
            self.get_logger().warn(
                f"Centerline appears open (endpoints gap={close_gap:.2f} m). "
                "Treating path as non-loop to avoid wrap-around teleports."
            )

        filtered = [points[0]]
        filtered_heading = [headings[0]]
        s = [0.0]
        total = 0.0
        for i in range(1, len(points)):
            seg = math.hypot(points[i][0] - filtered[-1][0], points[i][1] - filtered[-1][1])
            if seg < 1e-4:
                continue
            filtered.append(points[i])
            filtered_heading.append(headings[i])
            total += seg
            s.append(total)

        if total < 2.0:
            self.get_logger().warn(f"Centerline length too short: {total:.3f} m ({csv_path})")
            return

        # Estimate heading offset between waypoint psi and geometric path tangent.
        if self.map_waypoint_heading_auto_offset and len(filtered_heading) == len(filtered):
            diffs = []
            for i in range(1, len(filtered)):
                h = filtered_heading[i]
                if math.isnan(h):
                    continue
                dx = filtered[i][0] - filtered[i - 1][0]
                dy = filtered[i][1] - filtered[i - 1][1]
                if dx * dx + dy * dy < 1e-8:
                    continue
                tangent = math.atan2(dy, dx)
                diffs.append(math.atan2(math.sin(tangent - h), math.cos(tangent - h)))
            if diffs:
                s_sin = sum(math.sin(d) for d in diffs)
                s_cos = sum(math.cos(d) for d in diffs)
                self.map_heading_offset_rad = math.atan2(s_sin, s_cos)
            else:
                self.map_heading_offset_rad = 0.0
        else:
            self.map_heading_offset_rad = self.map_waypoint_heading_offset_rad

        self.map_path_xy = filtered
        self.map_path_heading = filtered_heading
        self.map_path_s = s
        self.map_path_total_length = total
        self.map_path_progress = 0.0
        self._prev_gt_yaw_for_rate = 0.0
        self.world_type = "map"
        self.get_logger().info(
            f"Loaded centerline CSV path with {len(filtered)} points, length={total:.2f} m"
        )
        self.get_logger().info(
            f"Waypoint heading offset={self.map_heading_offset_rad:.3f} rad"
        )

    def _align_centerline_to_map(self):
        if not self.centerline_auto_align_to_map and abs(self.centerline_offset_x_m) < 1e-9 and abs(self.centerline_offset_y_m) < 1e-9:
            return

        xs = [p[0] for p in self.map_path_xy]
        ys = [p[1] for p in self.map_path_xy]
        if not xs:
            return
        csv_cx = 0.5 * (min(xs) + max(xs))
        csv_cy = 0.5 * (min(ys) + max(ys))

        dx = self.centerline_offset_x_m
        dy = self.centerline_offset_y_m
        if self.centerline_auto_align_to_map:
            dx += (self.map_cx - csv_cx)
            dy += (self.map_cy - csv_cy)

        self.map_path_xy = [(x + dx, y + dy) for (x, y) in self.map_path_xy]
        self.get_logger().info(
            f"Centerline aligned to map. shift=({dx:.3f}, {dy:.3f}) m"
        )

    def _occupied_along_ray(self, theta: float, max_range: float):
        """Returns consolidated occupied hit distances from map center along ray."""
        if self.map_occ is None:
            return []
        step = max(0.03, 0.6 * self.map_resolution)
        c = math.cos(theta)
        s = math.sin(theta)
        d = 0.0
        prev_occ = False
        runs = []
        run_start = 0.0
        while d <= max_range:
            x = self.map_cx + d * c
            y = self.map_cy + d * s
            row, col = self._world_to_map(x, y)
            if row < 0 or row >= self.map_height or col < 0 or col >= self.map_width:
                # Do not merge out-of-map area into occupied runs.
                # For outer-wall-only maps, treating OOB as occupied makes
                # "wall hit" appear far outside the track.
                if prev_occ:
                    runs.append((run_start, d))
                break
            else:
                occ = bool(self.map_occ[row, col])
            if occ and not prev_occ:
                run_start = d
            if prev_occ and not occ:
                runs.append((run_start, d))
            prev_occ = occ
            d += step
        if prev_occ:
            runs.append((run_start, max_range))

        centers = []
        min_run = max(2.0 * self.map_resolution, 0.04)
        for start, end in runs:
            if end - start >= min_run:
                centers.append(0.5 * (start + end))
        return centers

    def _build_map_path_from_polar(self):
        max_range = max(self.map_width, self.map_height) * self.map_resolution
        path = []
        for i in range(self.map_path_points):
            theta = (2.0 * math.pi * i) / self.map_path_points
            hits = self._occupied_along_ray(theta, max_range)
            if len(hits) >= 2:
                r = 0.5 * (hits[0] + hits[1])
                x = self.map_cx + r * math.cos(theta)
                y = self.map_cy + r * math.sin(theta)
                path.append((x, y))
            elif len(hits) == 1:
                # Single-wall maps (outer border only): follow the wall with
                # a fixed inward offset so mapping can still progress.
                r = max(0.0, hits[0] - self.map_single_wall_inset_m)
                x = self.map_cx + r * math.cos(theta)
                y = self.map_cy + r * math.sin(theta)
                path.append((x, y))

        if len(path) < 60:
            self.get_logger().warn(
                "Map centerline extraction failed. Falling back to circular path."
            )
            self.map_path_xy = []
            self.map_path_heading = []
            self.map_path_s = []
            self.map_path_total_length = 0.0
            return

        # Close path only when endpoints are close enough.
        close_gap = math.hypot(path[0][0] - path[-1][0], path[0][1] - path[-1][1])
        if close_gap <= 1.0:
            if close_gap > 0.1:
                path.append(path[0])
            self.map_path_is_loop = True
        else:
            self.map_path_is_loop = False
            self.get_logger().warn(
                f"Extracted path appears open (endpoints gap={close_gap:.2f} m). "
                "Treating path as non-loop to avoid wrap-around teleports."
            )

        filtered = [path[0]]
        s = [0.0]
        total = 0.0
        for i in range(1, len(path)):
            seg = math.hypot(path[i][0] - filtered[-1][0], path[i][1] - filtered[-1][1])
            if seg < 1e-4:
                continue
            filtered.append(path[i])
            total += seg
            s.append(total)

        if total < 2.0:
            self.map_path_xy = []
            self.map_path_heading = []
            self.map_path_s = []
            self.map_path_total_length = 0.0
            return

        self.map_path_xy = filtered
        self.map_path_heading = []
        self.map_path_s = s
        self.map_path_total_length = total
        self.map_path_progress = 0.0

    def _interpolate_map_path(self, s_query: float):
        if self.map_path_total_length <= 0.0 or len(self.map_path_xy) < 2:
            return None
        if self.map_path_is_loop:
            s_mod = s_query % self.map_path_total_length
        else:
            s_mod = max(0.0, min(self.map_path_total_length, s_query))
        idx = 1
        while idx < len(self.map_path_s) and self.map_path_s[idx] < s_mod:
            idx += 1
        if idx >= len(self.map_path_s):
            idx = len(self.map_path_s) - 1
        s0 = self.map_path_s[idx - 1]
        s1 = self.map_path_s[idx]
        p0 = self.map_path_xy[idx - 1]
        p1 = self.map_path_xy[idx]
        if s1 - s0 < 1e-6:
            t = 0.0
        else:
            t = (s_mod - s0) / (s1 - s0)
        x = p0[0] + t * (p1[0] - p0[0])
        y = p0[1] + t * (p1[1] - p0[1])
        yaw = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        return x, y, yaw

    def _interpolate_waypoint_heading(self, s_query: float):
        if self.map_path_total_length <= 0.0:
            return None
        if len(self.map_path_heading) != len(self.map_path_xy) or len(self.map_path_heading) < 2:
            return None
        if any(math.isnan(h) for h in self.map_path_heading[: min(20, len(self.map_path_heading))]):
            return None
        if self.map_path_is_loop:
            s_mod = s_query % self.map_path_total_length
        else:
            s_mod = max(0.0, min(self.map_path_total_length, s_query))
        idx = 1
        while idx < len(self.map_path_s) and self.map_path_s[idx] < s_mod:
            idx += 1
        if idx >= len(self.map_path_s):
            idx = len(self.map_path_s) - 1
        s0 = self.map_path_s[idx - 1]
        s1 = self.map_path_s[idx]
        h0 = self.map_path_heading[idx - 1]
        h1 = self.map_path_heading[idx]
        if math.isnan(h0) or math.isnan(h1):
            return None
        if s1 - s0 < 1e-6:
            return h0
        t = (s_mod - s0) / (s1 - s0)
        dh = math.atan2(math.sin(h1 - h0), math.cos(h1 - h0))
        h = math.atan2(math.sin(h0 + t * dh), math.cos(h0 + t * dh))
        return math.atan2(
            math.sin(h + self.map_heading_offset_rad),
            math.cos(h + self.map_heading_offset_rad),
        )

    def _compute_smoothed_yaw_and_rate(self, s_query: float, v: float):
        d = max(0.2, self.map_yaw_preview_m)
        p_prev = self._interpolate_map_path(s_query - d)
        p_curr = self._interpolate_map_path(s_query)
        p_next = self._interpolate_map_path(s_query + d)
        if p_prev is None or p_curr is None or p_next is None:
            if p_curr is None:
                return 0.0, 0.0
            return p_curr[2], 0.0

        yaw = math.atan2(p_next[1] - p_prev[1], p_next[0] - p_prev[0])
        yaw_prev = math.atan2(p_curr[1] - p_prev[1], p_curr[0] - p_prev[0])
        yaw_next = math.atan2(p_next[1] - p_curr[1], p_next[0] - p_curr[0])
        signed_curvature = math.atan2(
            math.sin(yaw_next - yaw_prev), math.cos(yaw_next - yaw_prev)
        ) / (2.0 * d)
        yaw_rate = v * signed_curvature
        return yaw, yaw_rate

    def _closest_path_index(self, x: float, y: float):
        if len(self.map_path_xy) < 2:
            return 0
        best_i = 0
        best_d2 = float("inf")
        for i, (px, py) in enumerate(self.map_path_xy):
            d2 = (x - px) * (x - px) + (y - py) * (y - py)
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
        return best_i

    def _on_local_path(self, msg: Path):
        if not msg.poses:
            return
        pts = []
        for pose in msg.poses:
            pts.append((pose.pose.position.x, pose.pose.position.y))
        with self._state_lock:
            self.external_local_path_xy = pts

    def _on_recommended_path(self, msg: Path):
        if not msg.poses:
            return
        pts = []
        for pose in msg.poses:
            pts.append((pose.pose.position.x, pose.pose.position.y))
        with self._state_lock:
            self.external_recommended_path_xy = pts

    def _step_from_local_path(self):
        path_xy = self.external_local_path_xy
        path_source = "local_path"
        if len(path_xy) < 2 and len(self.external_recommended_path_xy) >= 2:
            path_xy = self.external_recommended_path_xy
            path_source = "recommended_path"

        if len(path_xy) < 2:
            if not self._warned_no_external_path:
                self.get_logger().warn(
                    f"No external path yet (local='{self.external_path_topic}', "
                    f"fallback='{self.external_path_fallback_topic}'); holding pose."
                )
                self._warned_no_external_path = True
            self.gt_v = 0.0
            self.gt_yaw_rate = 0.0
            self.gt_ax_body = 0.0
            self.gt_ay_body = 0.0
            return

        self._warned_no_external_path = False
        n = len(path_xy)
        if self._last_external_path_len != n:
            self._last_external_best_i = None
            self._last_external_path_len = n

        if self._last_external_best_i is None:
            best_i = 0
            best_d2 = float("inf")
            for i, (px, py) in enumerate(path_xy):
                d2 = (px - self.gt_x) * (px - self.gt_x) + (py - self.gt_y) * (py - self.gt_y)
                if d2 < best_d2:
                    best_d2 = d2
                    best_i = i
        else:
            best_i = self._last_external_best_i
            best_d2 = float("inf")
            k0 = -self.external_search_behind
            k1 = self.external_search_ahead
            for k in range(k0, k1 + 1):
                i = (self._last_external_best_i + k) % n
                px, py = path_xy[i]
                d2 = (px - self.gt_x) * (px - self.gt_x) + (py - self.gt_y) * (py - self.gt_y)
                if d2 < best_d2:
                    best_d2 = d2
                    best_i = i
            # If we're clearly off this local neighborhood, relock globally.
            if math.sqrt(best_d2) > self.external_relock_distance_m:
                best_i = 0
                best_d2 = float("inf")
                for i, (px, py) in enumerate(path_xy):
                    d2 = (px - self.gt_x) * (px - self.gt_x) + (py - self.gt_y) * (py - self.gt_y)
                    if d2 < best_d2:
                        best_d2 = d2
                        best_i = i
        self._last_external_best_i = best_i

        target_i = min(n - 1, best_i + self.external_path_target_index)
        tx, ty = path_xy[target_i]
        desired_yaw = math.atan2(ty - self.gt_y, tx - self.gt_x)
        yaw_err = math.atan2(math.sin(desired_yaw - self.gt_yaw), math.cos(desired_yaw - self.gt_yaw))
        max_step = max(0.05, self.pp_max_yaw_rate_radps) * self.state_dt
        yaw_step = max(-max_step, min(max_step, yaw_err))
        self.gt_yaw += yaw_step
        self.gt_yaw = math.atan2(math.sin(self.gt_yaw), math.cos(self.gt_yaw))

        v = max(0.1, self.map_path_speed_mps)
        self.gt_x += v * math.cos(self.gt_yaw) * self.state_dt
        self.gt_y += v * math.sin(self.gt_yaw) * self.state_dt
        self.gt_v = v
        self.gt_yaw_rate = yaw_step / max(1e-4, self.state_dt)
        self.gt_ax_body = 0.0
        self.gt_ay_body = self.gt_v * self.gt_yaw_rate
        if path_source == "recommended_path" and int(self.t * 2.0) % 10 == 0:
            self.get_logger().info("Driving from /recommended_path fallback (no /local_path).")

    def _pure_pursuit_step(self):
        if len(self.map_path_xy) < 2:
            return

        # Initialize pose on the first segment once path becomes available.
        if self.t <= self.state_dt and self.gt_v == 0.0 and abs(self.gt_x) < 1e-6 and abs(self.gt_y) < 1e-6:
            x0, y0 = self.map_path_xy[0]
            x1, y1 = self.map_path_xy[1]
            self.gt_x = x0
            self.gt_y = y0
            self.gt_yaw = math.atan2(y1 - y0, x1 - x0)

        # Keep progress monotonic. Only relock when pose drifts far from current
        # reference to avoid index-flip oscillations near curves.
        ref = self._interpolate_map_path(self.map_path_progress)
        if ref is not None:
            ref_dx = self.gt_x - ref[0]
            ref_dy = self.gt_y - ref[1]
            if math.hypot(ref_dx, ref_dy) > max(2.0, self.pp_relock_distance_m):
                nearest_idx = self._closest_path_index(self.gt_x, self.gt_y)
                if nearest_idx < len(self.map_path_s):
                    self.map_path_progress = self.map_path_s[nearest_idx]

        v = self._compute_map_path_speed(self.map_path_progress)

        lookahead = max(0.8, self.pp_lookahead_m + self.pp_lookahead_gain * v)
        target_s = self.map_path_progress + lookahead
        target = self._interpolate_map_path(target_s)
        if target is None:
            return
        tx, ty, _ = target

        alpha = math.atan2(ty - self.gt_y, tx - self.gt_x) - self.gt_yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        yaw_rate = (2.0 * v * math.sin(alpha)) / max(0.2, lookahead)
        if self.map_use_waypoint_heading:
            desired_yaw = self._interpolate_waypoint_heading(target_s)
            if desired_yaw is not None:
                yaw_err = math.atan2(
                    math.sin(desired_yaw - self.gt_yaw),
                    math.cos(desired_yaw - self.gt_yaw),
                )
                yaw_rate += self.map_waypoint_heading_gain * yaw_err
        yaw_rate = max(-self.pp_max_yaw_rate_radps, min(self.pp_max_yaw_rate_radps, yaw_rate))

        self.gt_yaw += yaw_rate * self.state_dt
        self.gt_yaw = math.atan2(math.sin(self.gt_yaw), math.cos(self.gt_yaw))
        self.gt_x += v * math.cos(self.gt_yaw) * self.state_dt
        self.gt_y += v * math.sin(self.gt_yaw) * self.state_dt

        self.gt_v = v
        self.gt_yaw_rate = yaw_rate
        self.gt_ax_body = 0.0
        self.gt_ay_body = v * yaw_rate

        # Progress must advance even in pure-pursuit mode. Without this,
        # the controller keeps chasing almost the same local segment and can
        # oscillate/spin in place on curved tracks.
        ds = max(0.0, v) * self.state_dt
        if self.map_path_is_loop:
            self.map_path_progress = (
                self.map_path_progress + ds
            ) % max(1e-6, self.map_path_total_length)
        else:
            self.map_path_progress = min(
                max(0.0, self.map_path_total_length - 1e-3),
                self.map_path_progress + ds,
            )

    def _compute_map_path_speed(self, s_query: float):
        v_base = max(0.3, self.map_path_speed_mps)
        if self.map_path_total_length <= 0.0:
            return v_base

        preview = max(0.2, self.map_corner_preview_m)
        interp_prev = self._interpolate_map_path(s_query - preview)
        interp_next = self._interpolate_map_path(s_query + preview)
        if interp_prev is None or interp_next is None:
            return v_base

        dyaw = math.atan2(
            math.sin(interp_next[2] - interp_prev[2]),
            math.cos(interp_next[2] - interp_prev[2]),
        )
        curvature = abs(dyaw) / (2.0 * preview)

        v_curve = v_base
        if self.map_corner_slowdown_gain > 0.0:
            v_curve = min(v_curve, v_base / (1.0 + self.map_corner_slowdown_gain * curvature))

        if self.map_max_lateral_accel_mps2 > 0.0 and curvature > 1e-4:
            v_lat = math.sqrt(self.map_max_lateral_accel_mps2 / curvature)
            v_curve = min(v_curve, v_lat)

        min_v = max(0.3, self.map_min_speed_mps)
        return min(v_base, max(min_v, v_curve))

    def _world_to_map(self, x: float, y: float):
        mx = int((x - self.map_origin_x) / self.map_resolution)
        my = int((y - self.map_origin_y) / self.map_resolution)
        row = self.map_height - 1 - my
        col = mx
        return row, col

    def _ray_to_map_world(self, x0: float, y0: float, theta: float, max_range: float):
        if self.map_occ is None:
            return max_range
        step = max(self.map_raycast_step_m, self.map_resolution)
        c = math.cos(theta)
        s = math.sin(theta)
        d = 0.0
        while d <= max_range:
            x = x0 + d * c
            y = y0 + d * s
            row, col = self._world_to_map(x, y)
            if row < 0 or row >= self.map_height or col < 0 or col >= self.map_width:
                return d
            if self.map_occ[row, col]:
                return d
            d += step
        return max_range

    def _ray_circle_hits(
        self, x0: float, y0: float, c: float, s: float, cx: float, cy: float, r: float
    ):
        fx = x0 - cx
        fy = y0 - cy
        b = 2.0 * (fx * c + fy * s)
        cterm = fx * fx + fy * fy - r * r
        disc = b * b - 4.0 * cterm
        if disc < 0.0:
            return []
        sq = math.sqrt(disc)
        t1 = (-b - sq) / 2.0
        t2 = (-b + sq) / 2.0
        return [t for t in (t1, t2) if t > 0.02]

    def _ray_to_room_world(self, x0: float, y0: float, theta: float, max_range: float):
        c = math.cos(theta)
        s = math.sin(theta)
        eps = 1e-9
        hits = []

        # Room bounds: x,y in [-8, 8]
        for xw in (-8.0, 8.0):
            if abs(c) > eps:
                t = (xw - x0) / c
                y = y0 + t * s
                if t > 0.0 and -8.0 <= y <= 8.0:
                    hits.append(t)
        for yw in (-8.0, 8.0):
            if abs(s) > eps:
                t = (yw - y0) / s
                x = x0 + t * c
                if t > 0.0 and -8.0 <= x <= 8.0:
                    hits.append(t)

        hits.extend(self._ray_circle_hits(x0, y0, c, s, 2.0, 1.0, 1.0))

        if not hits:
            return max_range
        return min(max_range, min(hits))

    def _ray_to_racing_world(self, x0: float, y0: float, theta: float, max_range: float):
        c = math.cos(theta)
        s = math.sin(theta)
        hits = []

        # Simple circular race track: annulus between inner/outer walls.
        hits.extend(self._ray_circle_hits(x0, y0, c, s, 0.0, 0.0, 8.0))
        hits.extend(self._ray_circle_hits(x0, y0, c, s, 0.0, 0.0, 4.0))

        # Add two landmark obstacles to reduce symmetry.
        hits.extend(self._ray_circle_hits(x0, y0, c, s, 0.0, 6.2, 0.45))
        hits.extend(self._ray_circle_hits(x0, y0, c, s, -5.8, 0.0, 0.45))

        if not hits:
            return max_range
        return min(max_range, min(hits))

    def _publish_scan(self, x: float, y: float, yaw: float):
        scan_started = time.monotonic_ns()
        if self._last_scan_wall_ns is not None:
            wall_gap = (scan_started - self._last_scan_wall_ns) * 1e-9
            if wall_gap > max(0.5, 3.0 * self.scan_period_sec):
                self.get_logger().warn(
                    f"scan callback gap detected: {wall_gap:.3f}s (target period {self.scan_period_sec:.3f}s)"
                )
        self._last_scan_wall_ns = scan_started
        msg = LaserScan()
        msg.header.stamp = self._next_header_stamp()
        msg.header.frame_id = "laser"
        msg.angle_min = self.scan_angle_min
        msg.angle_max = self.scan_angle_min + self.scan_angle_increment * (len(self.scan_angles) - 1)
        msg.angle_increment = self.scan_angle_increment
        msg.time_increment = 0.0
        msg.scan_time = self.scan_period_sec
        msg.range_min = self.scan_range_min
        msg.range_max = self.scan_range_max

        c = math.cos(yaw)
        s = math.sin(yaw)
        laser_x = x + c * self.laser_offset_x_m - s * self.laser_offset_y_m
        laser_y = y + s * self.laser_offset_x_m + c * self.laser_offset_y_m
        laser_yaw = yaw + self.laser_yaw_offset

        if (
            self.world_type == "map"
            and self.map_occ is not None
            and self.scan_dropout_ratio <= 0.0
            and self.scan_outlier_ratio <= 0.0
            and self.scan_noise_std_m <= 0.0
        ):
            msg.ranges = self._scan_ranges_from_map_vectorized(
                laser_x, laser_y, laser_yaw, msg.range_max
            )
            self.scan_pub.publish(msg)
            elapsed_sec = (time.monotonic_ns() - scan_started) * 1e-9
            self._update_scan_diagnostics(elapsed_sec)
            if elapsed_sec > self.scan_period_sec and (self.t - self._last_scan_perf_warn_t) > 5.0:
                self._last_scan_perf_warn_t = self.t
                self.get_logger().warn(
                    f"scan generation slow: {elapsed_sec:.3f}s > period {self.scan_period_sec:.3f}s"
                )
            return

        ranges = []
        for angle in self.scan_angles:
            world_theta = laser_yaw + angle
            if self.world_type == "room":
                dist = self._ray_to_room_world(laser_x, laser_y, world_theta, msg.range_max)
            elif self.world_type == "map":
                dist = self._ray_to_map_world(laser_x, laser_y, world_theta, msg.range_max)
            else:
                dist = self._ray_to_racing_world(laser_x, laser_y, world_theta, msg.range_max)
            if self.scan_dropout_ratio > 0.0 and self.rng.random() < self.scan_dropout_ratio:
                ranges.append(float("inf"))
                continue

            noisy_dist = dist
            if self.scan_noise_std_m > 0.0:
                noisy_dist += self.rng.gauss(0.0, self.scan_noise_std_m)

            if self.scan_outlier_ratio > 0.0 and self.rng.random() < self.scan_outlier_ratio:
                noisy_dist += self.rng.uniform(-self.scan_outlier_max_m, self.scan_outlier_max_m)

            noisy_dist = max(msg.range_min, min(msg.range_max, noisy_dist))
            ranges.append(noisy_dist)

        msg.ranges = ranges
        self.scan_pub.publish(msg)
        elapsed_sec = (time.monotonic_ns() - scan_started) * 1e-9
        self._update_scan_diagnostics(elapsed_sec)
        if elapsed_sec > self.scan_period_sec and (self.t - self._last_scan_perf_warn_t) > 5.0:
            self._last_scan_perf_warn_t = self.t
            self.get_logger().warn(
                f"scan generation slow: {elapsed_sec:.3f}s > period {self.scan_period_sec:.3f}s"
            )

    def _update_scan_diagnostics(self, elapsed_sec: float):
        self._scan_pub_count += 1
        self._scan_pub_elapsed_accum += elapsed_sec
        now_ns = time.monotonic_ns()
        window_sec = (now_ns - self._scan_diag_last_wall_ns) * 1e-9
        if window_sec < 5.0:
            return

        pub_hz = self._scan_pub_count / max(1e-6, window_sec)
        avg_gen_ms = (self._scan_pub_elapsed_accum / max(1, self._scan_pub_count)) * 1000.0
        self.get_logger().info(
            f"scan publisher diag: pub_hz={pub_hz:.2f}, avg_gen={avg_gen_ms:.1f}ms, target={self.lidar_rate_hz:.2f}Hz"
        )
        self._scan_diag_last_wall_ns = now_ns
        self._scan_pub_count = 0
        self._scan_pub_elapsed_accum = 0.0

    def _scan_ranges_from_map_vectorized(
        self,
        laser_x: float,
        laser_y: float,
        laser_yaw: float,
        range_max: float,
    ):
        angles = self.scan_angles_np + np.float32(laser_yaw)
        cos_t = np.cos(angles)[:, None]
        sin_t = np.sin(angles)[:, None]
        dists = self.scan_dist_samples

        x = laser_x + cos_t * dists[None, :]
        y = laser_y + sin_t * dists[None, :]

        mx = ((x - self.map_origin_x) / self.map_resolution).astype(np.int32)
        my = ((y - self.map_origin_y) / self.map_resolution).astype(np.int32)
        row = self.map_height - 1 - my
        col = mx

        valid = (
            (row >= 0)
            & (row < self.map_height)
            & (col >= 0)
            & (col < self.map_width)
        )

        hit = ~valid
        if np.any(valid):
            occ = np.zeros_like(hit, dtype=bool)
            occ[valid] = self.map_occ[row[valid], col[valid]]
            hit |= occ

        first_idx = np.argmax(hit, axis=1)
        ray_indices = np.arange(hit.shape[0])
        has_hit = hit[ray_indices, first_idx]

        ranges = np.full(hit.shape[0], range_max, dtype=np.float32)
        ranges[has_hit] = dists[first_idx[has_hit]]
        return ranges.tolist()

    def _publish_imu(self, yaw: float, yaw_rate: float):
        if self.imu_pub is None:
            return
        msg = Imu()
        msg.header.stamp = self._next_header_stamp()
        msg.header.frame_id = "imu_link"
        imu_yaw = yaw + self.imu_yaw_bias
        qx, qy, qz, qw = self._yaw_to_quat(imu_yaw)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.angular_velocity.z = (
            yaw_rate
            + self.imu_ang_vel_bias
            + self.rng.gauss(0.0, self.imu_ang_vel_noise_std)
        )
        msg.linear_acceleration.x = self.gt_ax_body
        msg.linear_acceleration.y = self.gt_ay_body
        msg.linear_acceleration.z = 9.81
        self.imu_pub.publish(msg)

    def _publish_odom_and_tf(self, x: float, y: float, yaw: float, v: float, yaw_rate: float):
        if self.odom_pub is None:
            return
        if self.odom_follow_ground_truth:
            self.odom_x = x
            self.odom_y = y
            self.odom_yaw = yaw
            v_meas = v
            yaw_rate_meas = yaw_rate
        else:
            if self.odom_x is None:
                self.odom_x = x
                self.odom_y = y
                self.odom_yaw = yaw
                self.last_odom_pub_t = self.t
                self.last_odom_pub_wall_ns = time.monotonic_ns()

            odom_dt = 1.0 / self.odom_rate_hz
            now_wall_ns = time.monotonic_ns()
            if self.last_odom_pub_wall_ns is not None:
                odom_dt = max(1e-3, (now_wall_ns - self.last_odom_pub_wall_ns) * 1e-9)
            self.last_odom_pub_wall_ns = now_wall_ns
            self.last_odom_pub_t = self.t

            v_meas = v * (1.0 + self.odom_velocity_scale_error)
            yaw_rate_meas = (
                yaw_rate
                + self.odom_yaw_rate_bias
                + self.rng.gauss(0.0, self.odom_yaw_rate_noise_std)
            )

            # Short slip events emulate tire slip at aggressive corner entries.
            if self.slip_event_rate > 0.0 and self.t >= self._slip_until_t:
                if self.rng.random() < self.slip_event_rate:
                    self._slip_until_t = self.t + self.rng.uniform(
                        self.slip_duration_min_sec, self.slip_duration_max_sec
                    )
            slip_scale = self.slip_scale if self.t < self._slip_until_t else 1.0

            v_meas *= slip_scale
            yaw_rate_meas *= (2.0 - slip_scale)

            self.odom_yaw += yaw_rate_meas * odom_dt
            self.odom_x += v_meas * math.cos(self.odom_yaw) * odom_dt
            self.odom_y += v_meas * math.sin(self.odom_yaw) * odom_dt

        now = self._next_header_stamp()
        qx, qy, qz, qw = self._yaw_to_quat(self.odom_yaw)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v_meas
        odom.twist.twist.angular.z = yaw_rate_meas
        self.odom_pub.publish(odom)

        if self.publish_odom_tf:
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = "odom"
            tf.child_frame_id = "base_link"
            tf.transform.translation.x = self.odom_x
            tf.transform.translation.y = self.odom_y
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_pub.publish(TFMessage(transforms=[tf]))

    def _update_ground_truth(self):
        prev_x = self.gt_x
        prev_y = self.gt_y
        prev_yaw = self.gt_yaw

        # Use different trajectories by world type.
        if self.world_type == "room":
            radius = 2.5
            omega = 0.12
        elif self.world_type == "map":
            if self.motion_source == "local_path":
                self._step_from_local_path()
            elif self.map_path_total_length > 0.0:
                if self.map_use_pure_pursuit:
                    self._pure_pursuit_step()
                else:
                    v = self._compute_map_path_speed(self.map_path_progress)
                    self.map_path_progress += v * self.state_dt
                    if not self.map_path_is_loop:
                        self.map_path_progress = min(
                            self.map_path_progress, max(0.0, self.map_path_total_length - 1e-3)
                        )
                    interp = self._interpolate_map_path(self.map_path_progress)
                    if interp is None:
                        x = self.map_cx + self.map_radius * math.cos(self.map_omega * self.t)
                        y = self.map_cy + self.map_radius * math.sin(self.map_omega * self.t)
                        yaw = self.map_omega * self.t + math.pi / 2.0
                        yaw_rate = self.map_omega
                    else:
                        x, y, _ = interp
                        if self.map_follow_waypoint_yaw:
                            desired_yaw = self._interpolate_waypoint_heading(self.map_path_progress)
                            if desired_yaw is None:
                                desired_yaw, _ = self._compute_smoothed_yaw_and_rate(
                                    self.map_path_progress, v
                                )
                        else:
                            desired_yaw, _ = self._compute_smoothed_yaw_and_rate(
                                self.map_path_progress, v
                            )

                        # Limit heading slew in mapping mode to avoid corner-triggered
                        # pose jumps that can destabilize scan matching.
                        yaw_err = math.atan2(
                            math.sin(desired_yaw - self.gt_yaw),
                            math.cos(desired_yaw - self.gt_yaw),
                        )
                        max_step = max(0.05, self.pp_max_yaw_rate_radps) * self.state_dt
                        yaw_step = max(-max_step, min(max_step, yaw_err))
                        yaw = self.gt_yaw + yaw_step
                        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
                        yaw_rate = yaw_step / max(1e-4, self.state_dt)
                    ax_body = 0.0
                    ay_body = v * yaw_rate
                    self.gt_x = x
                    self.gt_y = y
                    self.gt_yaw = yaw
                    self.gt_v = v
                    self.gt_yaw_rate = yaw_rate
                    self.gt_ax_body = ax_body
                    self.gt_ay_body = ay_body
            elif self.motion_source == "internal_path":
                if not self._warned_no_map_path:
                    self.get_logger().error(
                        "No valid map path available; holding pose (no circular fallback)."
                    )
                    self._warned_no_map_path = True
                self.gt_v = 0.0
                self.gt_yaw_rate = 0.0
                self.gt_ax_body = 0.0
                self.gt_ay_body = 0.0

            # Guard against discontinuous centerline/path samples that can
            # teleport TF and destabilize Cartographer. Skip guard in startup
            # warm-up to allow initial pose/yaw alignment to settle.
            if self.t > 2.0:
                xy_jump = math.hypot(self.gt_x - prev_x, self.gt_y - prev_y)
                yaw_jump = abs(
                    math.atan2(
                        math.sin(self.gt_yaw - prev_yaw),
                        math.cos(self.gt_yaw - prev_yaw),
                    )
                )
                # External-path following can include sharper local replans, so
                # use relaxed suppression thresholds to avoid false stop loops.
                if self.motion_source == "local_path":
                    xy_jump_limit = 1.20
                    yaw_jump_limit = math.radians(90.0)
                else:
                    xy_jump_limit = 0.35
                    yaw_jump_limit = math.radians(25.0)

                if xy_jump > xy_jump_limit or yaw_jump > yaw_jump_limit:
                    if (self.t - self._last_pose_jump_warn_t) > 1.0:
                        self._last_pose_jump_warn_t = self.t
                        self.get_logger().warn(
                            f"Pose jump suppressed: dxy={xy_jump:.3f} m, dyaw={math.degrees(yaw_jump):.1f} deg"
                        )
                    # Relock progress to current vicinity so the next interpolation
                    # does not keep hitting the same discontinuous segment.
                    nearest_idx = self._closest_path_index(prev_x, prev_y)
                    if nearest_idx < len(self.map_path_s):
                        self.map_path_progress = self.map_path_s[nearest_idx]
                    self.gt_x = prev_x
                    self.gt_y = prev_y
                    self.gt_yaw = prev_yaw
                    self.gt_v = 0.0
                    self.gt_yaw_rate = 0.0
                    self.gt_ax_body = 0.0
                    self.gt_ay_body = 0.0
            self.t += self.state_dt
            return
        else:
            radius = 6.0
            omega = 0.16
        # Add mild speed modulation to better emulate real driving.
        omega_mod = omega * (1.0 + self.speed_modulation_amp * math.sin(0.25 * self.t))
        x = radius * math.cos(omega_mod * self.t)
        y = radius * math.sin(omega_mod * self.t)
        yaw = omega_mod * self.t + math.pi / 2.0
        v = radius * omega_mod
        yaw_rate = omega_mod
        ax_body = 0.0
        ay_body = v * yaw_rate

        self.gt_x = x
        self.gt_y = y
        self.gt_yaw = yaw
        self.gt_v = v
        self.gt_yaw_rate = yaw_rate
        self.gt_ax_body = ax_body
        self.gt_ay_body = ay_body
        self.t += self.state_dt

    def _on_state_timer(self):
        with self._state_lock:
            self._update_ground_truth()

    def _on_scan_timer(self):
        with self._state_lock:
            x = self.gt_x
            y = self.gt_y
            yaw = self.gt_yaw
        self._publish_scan(x, y, yaw)

    def _on_imu_timer(self):
        with self._state_lock:
            yaw = self.gt_yaw
            yaw_rate = self.gt_yaw_rate
        self._publish_imu(yaw, yaw_rate)

    def _on_odom_timer(self):
        with self._state_lock:
            x = self.gt_x
            y = self.gt_y
            yaw = self.gt_yaw
            v = self.gt_v
            yaw_rate = self.gt_yaw_rate
        self._publish_odom_and_tf(x, y, yaw, v, yaw_rate)

    def _scan_publish_loop(self):
        next_ts = time.monotonic()
        while not self._scan_thread_stop.is_set():
            with self._state_lock:
                x = self.gt_x
                y = self.gt_y
                yaw = self.gt_yaw
            self._publish_scan(x, y, yaw)
            next_ts += self.scan_period_sec
            sleep_sec = next_ts - time.monotonic()
            if sleep_sec > 0.0:
                time.sleep(sleep_sec)
            else:
                next_ts = time.monotonic()

    def _odom_publish_loop(self):
        odom_period_sec = 1.0 / max(1.0, self.odom_rate_hz)
        next_ts = time.monotonic()
        while not self._odom_thread_stop.is_set():
            with self._state_lock:
                x = self.gt_x
                y = self.gt_y
                yaw = self.gt_yaw
                v = self.gt_v
                yaw_rate = self.gt_yaw_rate
            self._publish_odom_and_tf(x, y, yaw, v, yaw_rate)
            next_ts += odom_period_sec
            sleep_sec = next_ts - time.monotonic()
            if sleep_sec > 0.0:
                time.sleep(sleep_sec)
            else:
                next_ts = time.monotonic()

    def destroy_node(self):
        self._scan_thread_stop.set()
        if self._scan_thread is not None and self._scan_thread.is_alive():
            self._scan_thread.join(timeout=1.0)
        self._odom_thread_stop.set()
        if self._odom_thread is not None and self._odom_thread.is_alive():
            self._odom_thread.join(timeout=1.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FakeSensorPublisher()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
