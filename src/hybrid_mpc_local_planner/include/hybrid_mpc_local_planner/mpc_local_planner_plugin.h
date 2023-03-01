#ifndef MPC_LOCAL_PLANNER_PLUGIN_H_
#define MPC_LOCAL_PLANNER_PLUGIN_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <tf2_ros/buffer.h>

#include <dynamic_reconfigure/server.h>

#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <ros/ros.h>
#include <hybrid_mpc_local_planner/hybrid_mpc_planner.h>
#include <hybrid_mpc_local_planner/hybrid_mpc_config.h>

namespace hybrid_mpc_local_planner
{
    class MPCLocalPlannerPlugin : public nav_core::BaseLocalPlanner
    {
    public:
        MPCLocalPlannerPlugin();
        /**
         * @brief  Constructs the ros wrapper
         * @param name The name to give this instance of the trajectory planner
         * @param tf A pointer to a transform listener
         * @param costmap The cost map to use for assigning costs to trajectories
         */
        void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros);
        /**
         * @brief  Destructor for the wrapper
         */
        ~MPCLocalPlannerPlugin();
        /**
         * @brief  Given the current position, orientation, and velocity of the robot,
         * compute velocity commands to send to the base
         * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
         * @return True if a valid trajectory was found, false otherwise
         */
        bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);
        /**
         * @brief  Set the plan that the controller is following
         * @param orig_global_plan The plan to pass to the controller
         * @return True if the plan was updated successfully, false otherwise
         */
        bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);
        /**
         * @brief  Check if the goal pose has been achieved
         * @return True if achieved, false otherwise
         */
        bool isGoalReached();
        bool isInitialized() { return initialized_; }
        bool switchRotationMode(bool mode);

    private:
        void publishLocalPlan(std::vector<geometry_msgs::PoseStamped> &path);

        void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped> &path);

        void marker_properties(visualization_msgs::Marker &marker, int id, double r, double g, double b);

        double quat_2_euler(const geometry_msgs::PoseStamped &msg);

        // returns true if the local_goal is the global goal
        bool update_local_goal(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);
        // returns true if the local_goal is the global goal
        bool update_local_ref_path(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);
        // move_base version
        bool update_local_ref_path_move_base(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);

        tf2_ros::Buffer *tf_;

        // for parameter conigurations
        HybridMPCConfig cfg_;

        // for visualisation, publishers of global and local plan
        ros::Publisher g_plan_pub_, l_plan_pub_, local_goal_pub_, robot_pose_pub_, astar_path_pub_, astar_ref_path_pub_;
        boost::shared_ptr<MPCPlanner> dp_; ///< @brief The trajectory controller

        // costmap related
        costmap_2d::Costmap2DROS *costmap_ros_;

        // path planning related
        std::vector<geometry_msgs::PoseStamped> global_path_;
        std::vector<geometry_msgs::PoseStamped> local_ref_path_;
        std::vector<geometry_msgs::PoseStamped> local_path_;
        nav_msgs::Path local_ref_path_ros_;
        nav_msgs::Path local_nav_path_;
        geometry_msgs::PoseStamped local_goal_;
        double goal_x_threshold_, goal_y_threshold_, goal_yaw_threshold_;

        // visualization related
        visualization_msgs::Marker local_goal_marker_;
        visualization_msgs::Marker robot_pose_marker_;

        geometry_msgs::PoseStamped current_pose_;
        bool initialized_;
    };
}
#endif