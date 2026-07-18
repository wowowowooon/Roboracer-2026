include "cartographer_2d_mapping_imu_lidar_no_odom.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

-- Higher speed: match a bit more often + slightly wider search so small slip
-- corrects every scan instead of "drift then snap". Keep CPU below prior 0.5m/30° blowup.
options.tracking_frame = "base_link"
options.published_frame = "base_link"
options.map_frame = "map"
options.provide_odom_frame = false
options.use_odometry = false
options.use_pose_extrapolator = true
options.pose_publish_period_sec = 0.02
options.submap_publish_period_sec = 1.0
options.odometry_sampling_ratio = 0.0
options.imu_sampling_ratio = 1.0
options.rangefinder_sampling_ratio = 1.0
options.landmarks_sampling_ratio = 0.0
options.fixed_frame_pose_sampling_ratio = 0.0

MAP_BUILDER.num_background_threads = 2

TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.08
TRAJECTORY_BUILDER_2D.max_range = 16.0
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.07
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- ~50 km/h @ 40 Hz ≈ 0.35 m/scan; 0.48 m leaves margin if one match is late.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.48
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(22.)
-- Softer prior: correct small error each frame (less "틀어졌다 확 잡힘").
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 5.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 26.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 3.5
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 16.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10
-- Moving: match every ~3 cm. Idle: ~20 Hz cap.
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.03
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.8)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 20

POSE_GRAPH.optimize_every_n_nodes = 120
POSE_GRAPH.global_sampling_ratio = 0.01
POSE_GRAPH.constraint_builder.sampling_ratio = 0.02
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 8
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 2
POSE_GRAPH.max_num_final_iterations = 10

return options
