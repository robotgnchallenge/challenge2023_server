#ifndef HYBRID_MPC_LOCAL_PLANNER_HYBRID_MPC_PLANNER_H_
#define HYBRID_MPC_LOCAL_PLANNER_HYBRID_MPC_PLANNER_H_

#include <vector>
#include <stack>
#include <Eigen/Core>

#include <base_local_planner/trajectory.h>
#include <base_local_planner/map_grid_visualizer.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/cost_values.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

#include <hybrid_mpc_local_planner/hybrid_mpc_config.h>
#include "hybrid_mpc_local_planner/MPCTrajPlanner.h"
#include "hybrid_mpc_local_planner/MPCTrajPlannerRequest.h"
#include <hybrid_mpc_local_planner/pid.hpp>

namespace hybrid_mpc_local_planner
{
    typedef std::pair<int, int> pose_idx;
    typedef std::pair<double, pose_idx> f_pose_pair;

    class MPCPlanner
    {
    public:
        // a struct to store a grid's information
        struct grid_info
        {
            int parent_x;
            int parent_y;
            double f;
        };
        MPCPlanner(const HybridMPCConfig &cfg, std::string name, costmap_2d::Costmap2DROS *costmap);
        ~MPCPlanner();

        // map related
        void mapToWorld(double map_x, double map_y, double &world_x, double &world_y);
        /**
         * @brief transfer world coordinate to map coordinate
         *
         * @param wx
         * @param wy
         * @param mx
         * @param my
         * @return true
         * @return false
         */
        bool worldToMap(double world_x, double world_y, double &map_x, double &map_y);

        // check if the cell with index x and y is valid or not
        bool is_valid(pose_idx pose);

        // planning related
        bool make_plan(const geometry_msgs::PoseStamped &source, const geometry_msgs::PoseStamped &goal,
                       std::vector<geometry_msgs::PoseStamped> &plan, const bool terminate, std::vector<geometry_msgs::PoseStamped> &local_ref_path,
                       geometry_msgs::Twist &cmd_vel);
        bool make_plan(const pose_idx source, const pose_idx goal, std::vector<geometry_msgs::PoseStamped> &plan, const bool terminate);
        bool switch_mode(bool mode);

    private:
        // generate the astar_path_
        void generate_path(const pose_idx source, const pose_idx goal, std::vector<geometry_msgs::PoseStamped> &plan);

        // calculate distance from parent to child
        double calculate_g(const pose_idx parent, const pose_idx child);

        // calculate heuristic for the current grid
        double calculate_h(const pose_idx curr, const pose_idx goal);

        // calculate obstacle value for the grid, bigger value for closer obstacle distance
        double calculate_obstacle_weight(const pose_idx curr);

        // calculate the distance between the curr cell and the closest cell within the vicinity of the roadmap
        double calculate_roadmap_proximity_weight(const pose_idx curr);

        // visualization utilities
        void light_up_grid(const pose_idx curr, const int idx, const float r, const float g, const float b, const double size);

        // check are two grid very close
        bool within_vicinity(const pose_idx curr, const pose_idx goal, double &distance);

        bool update_local_topo_potential_field(std::vector<geometry_msgs::PoseStamped> &local_ref_path);
        // inflate a single cell with the specified number of waves
        void inflate_cell(int core_potential, int num_wave, int x, int y);

        double quat_2_euler(const geometry_msgs::PoseStamped &msg);
        costmap_2d::Costmap2DROS *lcr_;
        costmap_2d::Costmap2D *lc_; // local costmap
        // astar search related
        std::vector<std::vector<bool>> visited_;
        std::vector<std::vector<grid_info>> grids_info_;
        std::set<f_pose_pair> to_be_visited_;
        bool astar_success_ = false;
        std::vector<pose_idx> astar_path_;
        double obstacle_weight_;
        double astar_heuristic_weight_;
        double roadmap_proximity_weight_;
        double viapoint_threshold_distance_;
        std::vector<std::vector<int>> roadmap_potential_field_;
        bool unknown_space_valid_;

        // mpc planning related
        ros::ServiceClient mpc_client_;

        // robot state related
        bool is_rotating_ = false;
        float yaw_pid_kp_, yaw_pid_kd_, yaw_pid_output_max_, yaw_pid_ctrl_dead_th_;
        PID yaw_pid;

        // visualization related
        ros::Publisher grid_light_pub_, vel_pub_;
        nav_msgs::Path local_nav_path_;

        base_local_planner::Trajectory result_traj_;
        std::vector<geometry_msgs::PoseStamped> global_plan_;
    };

}

#endif