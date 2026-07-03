include "cartographer_2d_mapping_imu_lidar_no_odom.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

-- LiDAR-primary / IMU-auxiliary pure localization. No wheel odom input or odom TF output.
options.tracking_frame = "base_link"
options.published_frame = "base_link"
options.map_frame = "map"
options.provide_odom_frame = false
options.use_odometry = false
options.use_pose_extrapolator = true
options.pose_publish_period_sec = 0.02
options.submap_publish_period_sec = 0.5
options.odometry_sampling_ratio = 0.0
options.imu_sampling_ratio = 1.0
options.rangefinder_sampling_ratio = 1.0
options.landmarks_sampling_ratio = 0.0
options.fixed_frame_pose_sampling_ratio = 0.0

MAP_BUILDER.num_background_threads = 2

-- LiDAR: scan matching (primary). IMU: extrapolator only (auxiliary, between scans).
-- use_online_correlative_scan_matching 은 include 파일에서 이미 설정됨
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.50
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(30.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 5.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10
-- Match every scan (40 Hz LiDAR); do not skip scans while extrapolator drifts.
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.0
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.0
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.0
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 20

POSE_GRAPH.optimize_every_n_nodes = 90
-- 0.0 이면 finish_trajectory 시 FixedRatioSampler 경고가 반복됨 (루프 클로저 미사용).
POSE_GRAPH.global_sampling_ratio = 0.01
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 4
POSE_GRAPH.max_num_final_iterations = 20

return options
