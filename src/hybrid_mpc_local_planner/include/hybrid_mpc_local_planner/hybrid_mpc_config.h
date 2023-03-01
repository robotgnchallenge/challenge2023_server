//
// Created by ytz on 2021/1/8.
//

#ifndef HYBRID_MPC_LOCAL_PLANNER_HYBRID_MPC_CONFIG_H
#define HYBRID_MPC_LOCAL_PLANNER_HYBRID_MPC_CONFIG_H

#include <ros/ros.h>
#include <ros/console.h>

namespace hybrid_mpc_local_planner
{

    class HybridMPCConfig
    {
    public:
        // path finder related parameters
        struct Path
        {
            double obstacle_weight;
            double astar_heuristic_weight;
            double roadmap_proximity_weight;
            double viapoint_threshold_distance;
        } path;

        // local planner related parameters
        struct Planner
        {
            float yaw_pid_kp;
            float yaw_pid_kd;
            float yaw_pid_output_max;
            float yaw_pid_ctrl_dead_th;
            double goal_x_threshold;
            double goal_y_threshold;
            double goal_yaw_threshold;
            bool unknown_space_valid;
        } planner;

        HybridMPCConfig()
        {
            path.obstacle_weight = 0.1;
            path.astar_heuristic_weight = 0.3;
            path.roadmap_proximity_weight = 0.3;
            path.viapoint_threshold_distance = 0.3;

            planner.goal_x_threshold = 0.1;
            planner.goal_y_threshold = 0.1;
            planner.goal_yaw_threshold = 0.1;
            planner.yaw_pid_kp = 1;
            planner.yaw_pid_kd = 0;
            planner.yaw_pid_output_max = 1.5;
            planner.yaw_pid_ctrl_dead_th = 0.025;
            planner.unknown_space_valid = false;
        }

        void loadRosParamFromNodeHandle(const ros::NodeHandle &nh);
    };
}

#endif // HYBRID_MPC_LOCAL_PLANNER_HYBRID_MPC_CONFIG_H
