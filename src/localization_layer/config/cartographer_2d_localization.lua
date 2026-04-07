include "cartographer_2d_mapping.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

-- Keep localization frame model consistent with fake-sim lidar pipeline.
options.tracking_frame = "base_link"
options.published_frame = "base_link"
options.odom_frame = "odom"
options.map_frame = "map"
options.provide_odom_frame = false
-- In this sim pipeline odom can diverge from scan-matching near sharp turns.
-- Prefer lidar-only localization stability.
options.use_odometry = false

-- Lidar-only localization (IMU noise/frame mismatch can destabilize scan matching).
TRAJECTORY_BUILDER_2D.use_imu_data = false
-- Online correlative matching can snap to wrong places on repetitive tracks.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false

-- Stronger local smoothness to avoid sudden yaw/position flips.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 30.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 120.

-- Reduce aggressive global corrections during online localization.
POSE_GRAPH.optimize_every_n_nodes = 30
POSE_GRAPH.global_sampling_ratio = 0.0
POSE_GRAPH.constraint_builder.min_score = 0.78
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.90

return options
