#include <hybrid_mpc_local_planner/hybrid_mpc_planner_ros.h>
#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>

namespace hybrid_mpc_local_planner
{
    MPCPlannerROS::MPCPlannerROS() : initialized_(false) {}
    MPCPlannerROS::~MPCPlannerROS()
    {
    }

    void MPCPlannerROS::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
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

    bool MPCPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
    {
        if (!isInitialized())
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        else
        {
            ROS_INFO_THROTTLE(1.0, "Got new plan");
            global_path_ = orig_global_plan;
            return true;
        }
    }

    bool MPCPlannerROS::isGoalReached()
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
        if (fabs(current_pose_.pose.position.x - global_goal.pose.position.x) < goal_x_threshold_ && fabs(current_pose_.pose.position.y - global_goal.pose.position.y) < goal_y_threshold_ && fabs(current_yaw - goal_yaw) < goal_yaw_threshold_)
        {
            // disable the rotating in place mode, switch to mpc planning mode
            dp_->switch_mode(false);
            return true;
        }
        else
            return false;
    }

    bool MPCPlannerROS::switchRotationMode(bool mode)
    {
        dp_->switch_mode(mode);
        return true;
    }

    bool MPCPlannerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {
        bool res = false;
        if (!costmap_ros_->getRobotPose(current_pose_))
        {
            ROS_ERROR("Could not get robot pose");
            return false;
        }
        else
        {
            ROS_INFO_THROTTLE(1.0, "local planner is computing");
            // visualize the current robot pose
            marker_properties(robot_pose_marker_, 0, 1.0, 0.0, 0.0);
            robot_pose_marker_.pose.position.x = current_pose_.pose.position.x;
            robot_pose_marker_.pose.position.y = current_pose_.pose.position.y;
            robot_pose_marker_.pose.orientation = current_pose_.pose.orientation;
            robot_pose_pub_.publish(robot_pose_marker_);

            // find the waypoint in the orig_global_plan that is within the border of the local costmap, and set it as the goal for local planner
            // bool terminate = update_local_goal(global_path_);
            bool terminate = update_local_ref_path(global_path_);
            // visualize the local goal in rviz
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
                ROS_INFO_THROTTLE(1.0, "Found a valid path");
                res = true;
            }
            else
            {
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

    void MPCPlannerROS::marker_properties(visualization_msgs::Marker &marker, int id, double r, double g, double b)
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

    double MPCPlannerROS::quat_2_euler(const geometry_msgs::PoseStamped &msg)
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

    bool MPCPlannerROS::update_local_goal(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
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

    bool MPCPlannerROS::update_local_ref_path(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
    {
        int goal_idx = -1;
        // assign the global plan waypoint within the costmap as the reference goal for the local plan
        for (int i = 0; i < orig_global_plan.size(); i++)
        {
            if ((orig_global_plan[i].pose.position.x - costmap_ros_->getCostmap()->getOriginX() >= costmap_ros_->getCostmap()->getSizeInMetersX()) || (orig_global_plan[i].pose.position.y - costmap_ros_->getCostmap()->getOriginY() >= costmap_ros_->getCostmap()->getSizeInMetersY()) || (orig_global_plan[i].pose.position.x <= costmap_ros_->getCostmap()->getOriginX()) || (orig_global_plan[i].pose.position.y <= costmap_ros_->getCostmap()->getOriginY()))
            {
                break;
            }
            goal_idx++;
        }
        // keep pulling local goal closer to the robot until the goal is collision-free
        double origin_x = costmap_ros_->getCostmap()->getOriginX(), origin_y = costmap_ros_->getCostmap()->getOriginY();
        double resolution = costmap_ros_->getCostmap()->getResolution();
        double mx = (orig_global_plan[goal_idx].pose.position.x - origin_x) / resolution;
        double my = (orig_global_plan[goal_idx].pose.position.y - origin_y) / resolution;
        unsigned char cost = costmap_ros_->getCostmap()->getCost(mx, my);
        while (goal_idx > 0 && (cost == costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE || cost == costmap_2d::NO_INFORMATION))
        {
            ROS_INFO("******************************************");
            ROS_INFO("Pulling local goal closer to get away from collision");
            std::cout << "local goal cost: " << static_cast<int>(cost) << std::endl;
            std::cout << "goal idx: " << goal_idx << std::endl;
            goal_idx--;
            mx = (orig_global_plan[goal_idx].pose.position.x - origin_x) / resolution;
            my = (orig_global_plan[goal_idx].pose.position.y - origin_y) / resolution;
            cost = costmap_ros_->getCostmap()->getCost(mx, my);
        }
        if (goal_idx == orig_global_plan.size() - 1)
        {
            local_ref_path_ = orig_global_plan;
            local_goal_ = orig_global_plan.back();
            return true;
        }
        else
        {
            local_ref_path_ = std::vector<geometry_msgs::PoseStamped>(orig_global_plan.begin(), orig_global_plan.begin() + goal_idx);
            local_goal_ = orig_global_plan[goal_idx];
            return false;
        }
    }

}