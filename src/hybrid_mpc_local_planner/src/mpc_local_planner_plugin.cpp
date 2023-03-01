#include <hybrid_mpc_local_planner/mpc_local_planner_plugin.h>
#include <Eigen/Core>
#include <cmath>
#include <ros/console.h>
#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>
#include <pluginlib/class_list_macros.h>
#include <math.h>

PLUGINLIB_EXPORT_CLASS(hybrid_mpc_local_planner::MPCLocalPlannerPlugin, nav_core::BaseLocalPlanner)

namespace hybrid_mpc_local_planner
{
    MPCLocalPlannerPlugin::MPCLocalPlannerPlugin() : initialized_(false) {}
    MPCLocalPlannerPlugin::~MPCLocalPlannerPlugin()
    {
    }

    void MPCLocalPlannerPlugin::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!isInitialized())
        {
            ros::NodeHandle private_nh("~/" + name);

            cfg_.loadRosParamFromNodeHandle(private_nh);

            g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
            l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
            local_goal_pub_ = private_nh.advertise<visualization_msgs::Marker>("local_goal", 0);
            robot_pose_pub_ = private_nh.advertise<visualization_msgs::Marker>("robot_pose", 0);
            astar_path_pub_ = private_nh.advertise<nav_msgs::Path>("astar_path", 0);
            astar_ref_path_pub_ = private_nh.advertise<nav_msgs::Path>("astar_ref_path", 0);

            tf_ = tf;
            costmap_ros_ = costmap_ros;
            costmap_ros_->getRobotPose(current_pose_);

            dp_ = boost::shared_ptr<MPCPlanner>(new MPCPlanner(cfg_, name, costmap_ros_));
            initialized_ = true;

            goal_x_threshold_ = cfg_.planner.goal_x_threshold;
            goal_y_threshold_ = cfg_.planner.goal_y_threshold;
            goal_yaw_threshold_ = cfg_.planner.goal_yaw_threshold;
        }
        else
        {
            ROS_WARN("This planner has already been initialized");
        }
    }

    bool MPCLocalPlannerPlugin::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
    {
        if (!isInitialized())
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        else
        {
            ROS_INFO("Got new plan");
            global_path_.clear();
            global_path_ = orig_global_plan;
            switchRotationMode(false);
            return true;
        }
    }

    bool MPCLocalPlannerPlugin::isGoalReached()
    {
        if (!isInitialized())
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        if (!costmap_ros_->getRobotPose(current_pose_))
        {
            ROS_ERROR("Could not get robot pose");
            return false;
        }
        double current_yaw = quat_2_euler(current_pose_);
        geometry_msgs::PoseStamped global_goal = global_path_.back();
        double goal_yaw = quat_2_euler(global_goal);

        if(goal_yaw >= -M_PI &&  goal_yaw < 0){
            goal_yaw += 2*M_PI;
        }
        if(current_yaw >= -M_PI && current_yaw < 0){
            current_yaw += 2*M_PI;
        }
        
        auto yaw_offset =fabs( std::fmod( ( (current_yaw - goal_yaw) + M_PI ) , 2 * M_PI) - M_PI) ;

        if (pow(current_pose_.pose.position.x - global_goal.pose.position.x,2) + pow(current_pose_.pose.position.y - global_goal.pose.position.y,2) <= pow(0.15,2) && yaw_offset < 0.15)
        {
            // disable the rotating in place mode, switch to mpc planning mode
            std::cout<<"[MPC_PLANNER] Reach target position.]"<<std::endl;
            dp_->switch_mode(false);
            return true;
        }
        else{
            std::cout<<"[MPC_PLANNER] Failed to reach target position.]"<<std::endl;
            return false;
        }
            
    }

    bool MPCLocalPlannerPlugin::switchRotationMode(bool mode)
    {
        dp_->switch_mode(mode);
        return true;
    }

    bool MPCLocalPlannerPlugin::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {
        bool res = false;
        if (!costmap_ros_->getRobotPose(current_pose_))
        {
            ROS_ERROR("Could not get robot pose");
            return false;
        }
        else
        {
            ROS_INFO("local planner is computing");
            // visualize the current robot pose
            marker_properties(robot_pose_marker_, 0, 1.0, 0.0, 0.0);
            robot_pose_marker_.pose.position.x = current_pose_.pose.position.x;
            robot_pose_marker_.pose.position.y = current_pose_.pose.position.y;
            robot_pose_marker_.pose.orientation = current_pose_.pose.orientation;
            robot_pose_pub_.publish(robot_pose_marker_);

            // find the waypoint in the orig_global_plan that is within the border of the local costmap, and set it as the goal for local planner
            bool terminate = false;
            terminate = update_local_ref_path(global_path_);
            //  visualize the local goal in rviz
            marker_properties(local_goal_marker_, 1, 1.0, 0.0, 0.0);
            local_goal_marker_.pose.position.x = local_goal_.pose.position.x;
            local_goal_marker_.pose.position.y = local_goal_.pose.position.y;
            local_goal_marker_.pose.orientation = local_goal_.pose.orientation;
            local_goal_pub_.publish(local_goal_marker_);

            // visualize the portion of global path as the local planner's reference
            local_ref_path_ros_.poses.clear();
            local_ref_path_ros_.header.stamp = ros::Time::now();
            local_ref_path_ros_.header.frame_id = costmap_ros_->getGlobalFrameID();
            local_ref_path_ros_.poses = local_ref_path_;
            astar_ref_path_pub_.publish(local_ref_path_ros_);

            if (dp_->make_plan(current_pose_, local_goal_, local_path_, terminate, local_ref_path_, cmd_vel))
            {
                std::cout<<"[MPC_PLANNER] Found a valid path."<<std::endl;
                ROS_INFO_THROTTLE(1.0, "Found a valid path");
                res = true;
            }
            else
            {
                std::cout<<"[MPC_PLANNER] Can not found a valid path."<<std::endl;
                ROS_ERROR("Cannot find a valid path");
                res = false;
            }

            // visualize the local path
            local_nav_path_.poses.clear();
            local_nav_path_.header.stamp = ros::Time::now();
            local_nav_path_.header.frame_id = costmap_ros_->getGlobalFrameID();
            local_nav_path_.poses = local_path_;
            astar_path_pub_.publish(local_nav_path_);
            return res;
        }
    }

    void MPCLocalPlannerPlugin::marker_properties(visualization_msgs::Marker &marker, int id, double r, double g, double b)
    {
        marker.header.frame_id = costmap_ros_->getGlobalFrameID();
        marker.header.stamp = ros::Time::now();
        marker.ns = "hybrid_mpc_local_planner";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.id = id;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.scale.x = 0.1;
        marker.scale.y = marker.scale.z = 0.05;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0f;
    }

    double MPCLocalPlannerPlugin::quat_2_euler(const geometry_msgs::PoseStamped &msg)
    {
        tf2::Quaternion q(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    bool MPCLocalPlannerPlugin::update_local_goal(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
    {
        int goal_idx = -1;
        for (int i = 0; i < orig_global_plan.size(); i++)
        {
            if ((orig_global_plan[i].pose.position.x - costmap_ros_->getCostmap()->getOriginX() >= costmap_ros_->getCostmap()->getSizeInMetersX()) || (orig_global_plan[i].pose.position.y - costmap_ros_->getCostmap()->getOriginY() >= costmap_ros_->getCostmap()->getSizeInMetersY()) || (orig_global_plan[i].pose.position.x <= costmap_ros_->getCostmap()->getOriginX()) || (orig_global_plan[i].pose.position.y <= costmap_ros_->getCostmap()->getOriginY()))
            {
                break;
            }
            goal_idx++;
        }
        if (goal_idx == orig_global_plan.size() - 1)
        {
            local_goal_ = orig_global_plan.back();
            return true;
        }
        else
        {
            local_goal_ = orig_global_plan[goal_idx];
            return false;
        }
    }

    bool MPCLocalPlannerPlugin::update_local_ref_path(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
    {
        local_ref_path_.clear();
        int start_idx = 0;
        for (int i = 0; i < orig_global_plan.size(); i++)
        {
            if ((orig_global_plan[i].pose.position.x - costmap_ros_->getCostmap()->getOriginX() >= costmap_ros_->getCostmap()->getSizeInMetersX()) ||
                (orig_global_plan[i].pose.position.y - costmap_ros_->getCostmap()->getOriginY() >= costmap_ros_->getCostmap()->getSizeInMetersY()) ||
                (orig_global_plan[i].pose.position.x <= costmap_ros_->getCostmap()->getOriginX()) ||
                (orig_global_plan[i].pose.position.y <= costmap_ros_->getCostmap()->getOriginY()))
            {
                start_idx++;
            }
            else
            {
                break;
            }
        }
        ROS_INFO("start index: %d", start_idx);
        int goal_idx = start_idx - 1;
        for (int i = start_idx; i < orig_global_plan.size(); i++)
        {
            if ((orig_global_plan[i].pose.position.x - costmap_ros_->getCostmap()->getOriginX() >= costmap_ros_->getCostmap()->getSizeInMetersX()) ||
                (orig_global_plan[i].pose.position.y - costmap_ros_->getCostmap()->getOriginY() >= costmap_ros_->getCostmap()->getSizeInMetersY()) ||
                (orig_global_plan[i].pose.position.x <= costmap_ros_->getCostmap()->getOriginX()) ||
                (orig_global_plan[i].pose.position.y <= costmap_ros_->getCostmap()->getOriginY()) )
            {
                ROS_INFO("current goal idx: %d ", goal_idx);
                break;
            }
            goal_idx++;
        }
        ROS_INFO("goal index: %d", goal_idx);
        ROS_INFO("global plan length: %d", (int)orig_global_plan.size());
        if ((goal_idx == orig_global_plan.size() - 1) ) //add by lsc
        {
            local_ref_path_ = std::vector<geometry_msgs::PoseStamped>(orig_global_plan.begin() + start_idx, orig_global_plan.end());
            local_goal_ = orig_global_plan.back();
            ROS_INFO("global goal inside local costmap");
            return true;
        }
        else
        {
            local_ref_path_ = std::vector<geometry_msgs::PoseStamped>(orig_global_plan.begin() + start_idx, orig_global_plan.begin() + goal_idx);
            local_goal_ = orig_global_plan[goal_idx];
            ROS_INFO("global goal outside local costmap");
            return false;
        }
        std::cout<<"==================================="<<std::endl;
    }

    bool MPCLocalPlannerPlugin::update_local_ref_path_move_base(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
    {
        local_ref_path_ = orig_global_plan;
        local_goal_ = orig_global_plan.back();
        ROS_INFO("local path got updated");
        return true;
    }

}
