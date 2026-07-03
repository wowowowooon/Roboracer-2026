include "pose_graph_mapping.lua"

MAP_BUILDER = {
  use_trajectory_builder_2d = false,
  use_trajectory_builder_3d = false,
  num_background_threads = 1,
  pose_graph = POSE_GRAPH,
  collate_by_trajectory = false,
}

return MAP_BUILDER
