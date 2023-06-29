include "carto_2d_slam_testbed04.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 100,
}
POSE_GRAPH.optimize_every_n_nodes = 5

return options
