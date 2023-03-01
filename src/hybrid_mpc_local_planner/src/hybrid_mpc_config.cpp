//
// Created by ytz on 2021/1/8.
//

#include "hybrid_mpc_local_planner/hybrid_mpc_config.h"
namespace hybrid_mpc_local_planner
{
    void HybridMPCConfig::loadRosParamFromNodeHandle(const ros::NodeHandle &nh)
    {
        nh.param("obstacle_weight", path.obstacle_weight, path.obstacle_weight);
        nh.param("astar_heuristic_weight", path.astar_heuristic_weight, path.astar_heuristic_weight);
        nh.param("roadmap_proximity_weight", path.roadmap_proximity_weight, path.roadmap_proximity_weight);
        nh.param("viapoint_threshold_distance", path.viapoint_threshold_distance, path.viapoint_threshold_distance);
        nh.param("goal_x_threshold", planner.goal_x_threshold, planner.goal_x_threshold);
        nh.param("goal_y_threshold", planner.goal_y_threshold, planner.goal_y_threshold);
        nh.param("goal_yaw_threshold", planner.goal_yaw_threshold, planner.goal_yaw_threshold);
        nh.param("yaw_pid_kp", planner.yaw_pid_kp, planner.yaw_pid_kp);
        nh.param("yaw_pid_kd", planner.yaw_pid_kd, planner.yaw_pid_kd);
        nh.param("yaw_pid_output_max", planner.yaw_pid_output_max, planner.yaw_pid_output_max);
        nh.param("unknown_space_valid", planner.unknown_space_valid, planner.unknown_space_valid);
    }
}