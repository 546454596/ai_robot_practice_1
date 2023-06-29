include "carto_slam_huawei.lua"

options.rangefinder_sampling_ratio = 0.5

TRAJECTORY_BUILDER.pure_localization_trimmer = {
    max_submaps_to_keep = 1000,
}

POSE_GRAPH.constraint_builder.global_localization_min_score = 0.45
POSE_GRAPH.constraint_builder.sampling_ratio = 0.01
POSE_GRAPH.global_sampling_ratio = 0.005
POSE_GRAPH.optimize_every_n_nodes = 10

return options
