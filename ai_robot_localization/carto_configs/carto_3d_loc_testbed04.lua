include "carto_3d_slam_testbed04.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
    max_submaps_to_keep = 1000,
}
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.4
POSE_GRAPH.constraint_builder.sampling_ratio = 0.015
POSE_GRAPH.global_sampling_ratio = 1e-6
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.optimize_every_n_nodes = 50

return options
