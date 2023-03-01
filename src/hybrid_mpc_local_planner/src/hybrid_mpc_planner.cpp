
#include <hybrid_mpc_local_planner/hybrid_mpc_planner.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <hybrid_mpc_local_planner/utils.hpp>

namespace hybrid_mpc_local_planner
{
    MPCPlanner::MPCPlanner(const HybridMPCConfig &cfg, std::string name, costmap_2d::Costmap2DROS *costmap)
    {
        ros::NodeHandle private_nh("~/" + name);
        mpc_client_ = private_nh.serviceClient<hybrid_mpc_local_planner::MPCTrajPlanner>("/mpc_traj_planner");
        lcr_ = costmap;
        lc_ = costmap->getCostmap();
        grid_light_pub_ = private_nh.advertise<visualization_msgs::Marker>("grid_light", 10);
        vel_pub_ = private_nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        obstacle_weight_ = cfg.path.obstacle_weight;
        astar_heuristic_weight_ = cfg.path.astar_heuristic_weight;
        roadmap_proximity_weight_ = cfg.path.roadmap_proximity_weight;
        viapoint_threshold_distance_ = cfg.path.viapoint_threshold_distance;
        yaw_pid_kp_ = cfg.planner.yaw_pid_kp;
        yaw_pid_kd_ = cfg.planner.yaw_pid_kd;
        yaw_pid_output_max_ = cfg.planner.yaw_pid_output_max;
        yaw_pid_ctrl_dead_th_ = cfg.planner.yaw_pid_ctrl_dead_th;
        unknown_space_valid_ = cfg.planner.unknown_space_valid;
        ROS_INFO("Yaw PID|kp:%.2f kd:%.2f", yaw_pid_kp_, yaw_pid_kd_);
        yaw_pid = PID(yaw_pid_kp_, 0, yaw_pid_kd_, 1, yaw_pid_output_max_, 1, yaw_pid_ctrl_dead_th_);
    }

    MPCPlanner::~MPCPlanner() {}

    void MPCPlanner::mapToWorld(double map_x, double map_y, double &world_x, double &world_y)
    {
        world_x = lc_->getOriginX() + map_x * lc_->getResolution();
        world_y = lc_->getOriginY() + map_y * lc_->getResolution();
    }

    bool MPCPlanner::worldToMap(double world_x, double world_y, double &map_x, double &map_y)
    {
        double origin_x = lc_->getOriginX(), origin_y = lc_->getOriginY();
        double resolution = lc_->getResolution();
        // ROS_INFO("local costmap origin: ", origin_x, origin_y);
        if (world_x < origin_x || world_y < origin_y)
        {
            ROS_ERROR("out of local costmap boundary");
            ROS_INFO("world_x, world_y, origin_x, origin_y: %f,%f,%f,%f", world_x, world_y, origin_x, origin_y);
            return false;
        }

        map_x = (world_x - origin_x) / resolution;
        map_y = (world_y - origin_y) / resolution;
        if (map_x < lc_->getSizeInCellsX() && map_y < lc_->getSizeInCellsY())
            return true;

        return false;
    }

    bool MPCPlanner::is_valid(pose_idx pose)
    {
        // check is the pose within the boundary
        if (pose.first < 0 || pose.first >= lc_->getSizeInCellsX() || pose.second < 0 || pose.second >= lc_->getSizeInCellsY())
        {
            return false;
        }
        unsigned char cost = lc_->getCost(pose.first, pose.second);
        // std::cout<<"cost is: "<<cost<<std::endl;
        // refactor: handle unknown_space_valid option
        if (cost == costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {
            return false;
        }
        if (!unknown_space_valid_ && cost == costmap_2d::NO_INFORMATION)
        {
            return false;
        }

        return true;
    }

    bool MPCPlanner::make_plan(const geometry_msgs::PoseStamped &source, const geometry_msgs::PoseStamped &goal,
                               std::vector<geometry_msgs::PoseStamped> &plan, const bool terminate,
                               std::vector<geometry_msgs::PoseStamped> &local_ref_path, geometry_msgs::Twist &cmd_vel)
    {
        double start_mx, start_my, goal_mx, goal_my;
        if (!worldToMap(source.pose.position.x, source.pose.position.y, start_mx, start_my))
        {
            ROS_ERROR("wrong source pose");
            return false;
        }
        if (!worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my))
        {
            ROS_ERROR("wrong goal pose");
            return false;
        }
        plan.clear();
        pose_idx start_idx = std::make_pair(start_mx, start_my);
        pose_idx goal_idx = std::make_pair(goal_mx, goal_my);
        bool follow_global_path = update_local_topo_potential_field(local_ref_path);
        // ROS_ERROR("Start Astar path planning");
        // std::cout << "start pose: " << source.pose.position.x << " " << source.pose.position.y << " goal pose: " << goal.pose.position.x << " " << goal.pose.position<< std::endl;

        //  chek if the start and goal are the same?
        if (abs(start_idx.first - goal_idx.first) + abs(start_idx.second - goal_idx.second) <= 2 ||
            is_rotating_)
        {
            if (abs(start_idx.first - goal_idx.first) + abs(start_idx.second - goal_idx.second) <= 2 &&
                !is_rotating_)
            {
                ROS_INFO_THROTTLE(1.0, "Near the goal, moving");
            }
            else if (abs(start_idx.first - goal_idx.first) + abs(start_idx.second - goal_idx.second) > 2 &&
                     is_rotating_)
            {
                ROS_INFO_THROTTLE(1.0, "Keep on rotating in place");
            }
            else
            {
                ROS_INFO_THROTTLE(1.0, "Near the goal and keep on rotating in place");
            }
            if (terminate)
            {
                // > adjust to target yaw
                ROS_INFO_THROTTLE(1.0, "Reached the x,y of goal");
                double goal_yaw = quat_2_euler(goal);
                double curr_yaw = quat_2_euler(source);

                float delta_angle = goal_yaw - curr_yaw;
                delta_angle = AngleLimitPI(delta_angle);
                goal_yaw = curr_yaw + delta_angle;
                float yaw_ctrl = yaw_pid.calcuOutput(curr_yaw, goal_yaw);
                // int signal = 1;
                // if(yaw_ctrl < 0) signal = -1;
                
                // if(fabs(yaw_ctrl) <= 0.1 ) yaw_ctrl = signal * 0.1;


                if (fabs(delta_angle) > yaw_pid.getCTRL_DEAD_TH())
                {
                    std::cout<<"d_angule_roatation" << " "<<delta_angle << " "<<yaw_pid.getCTRL_DEAD_TH() <<" "  <<yaw_ctrl<<std::endl;
                    ROS_INFO_THROTTLE(1.0, "Rotate in place");
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.linear.y = 0.0;
                    cmd_vel.angular.z = yaw_ctrl;
                    is_rotating_ = true;
                }
                else
                {
                    std::cout<<"d_angule" << " "<<delta_angle << " "<<yaw_pid.getCTRL_DEAD_TH() <<" "  <<yaw_ctrl<<std::endl;
                    ROS_INFO_THROTTLE(1.0, "No need rotation, stopping the robot");
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.linear.y = 0.0;
                    cmd_vel.angular.z = 0.0;
                    is_rotating_ = false;
                }
            }
            return true;
        }
        else
        {
            ROS_INFO_THROTTLE(1.0, "Far away from goal");
            // A* path planning
            if (!make_plan(start_idx, goal_idx, plan, terminate))
            {
                ROS_ERROR("Unable to find a collision free path");
                return false;
            }
            local_nav_path_.poses.clear();
            local_nav_path_.header.stamp = ros::Time::now();
            local_nav_path_.header.frame_id = lcr_->getGlobalFrameID();
            local_nav_path_.poses = plan;
            // MPC trajectory planning
            hybrid_mpc_local_planner::MPCTrajPlanner srv;
            ros::service::waitForService("/mpc_traj_planner", 2);
            srv.request.ref_path = local_nav_path_;
            srv.request.terminate = terminate;
            if (mpc_client_.call(srv))
            {
                // ROS_INFO("send request to mpc service server");
                int signal = 1;
                int signal_vel_x = 1;
                auto yaw_vel = srv.response.steer_angle;
                auto x_vel = srv.response.linear_acc;
                if(yaw_vel < 0) signal = -1;
                if(x_vel < 0) signal_vel_x = -1;
                
                yaw_vel = signal * std::max(0.1,fabs(yaw_vel));
                x_vel = signal_vel_x * std::max(0.1,fabs(x_vel));
                cmd_vel.linear.x = x_vel;
                cmd_vel.linear.y = 0.0;
                cmd_vel.angular.z = yaw_vel;
            }
            else
            {
                ROS_ERROR("Failed to pass ref path to MPC service server");
            }
            // std::cout<<"path length is: "<<plan.size()<<std::endl;
            return true;
        }
    }

    bool MPCPlanner::make_plan(const pose_idx source, const pose_idx goal, std::vector<geometry_msgs::PoseStamped> &plan, const bool terminate)
    {
        // std::cout << "source pose: " << source.first << " " << source.second << std::endl;
        // std::cout << "goal pose: " << goal.first << " " << goal.second << std::endl;
        if (!is_valid(source) || !is_valid(goal))
        {
            ROS_ERROR("Not valid request");
            return false;
        }

        visited_ = std::vector<std::vector<bool>>(lc_->getSizeInCellsX(), std::vector<bool>(lc_->getSizeInCellsY(), false));
        grids_info_ = std::vector<std::vector<grid_info>>(lc_->getSizeInCellsX(), std::vector<grid_info>(lc_->getSizeInCellsY()));
        for (int i = 0; i < lc_->getSizeInCellsX(); i++)
        {
            for (int j = 0; j < lc_->getSizeInCellsY(); j++)
            {
                grids_info_[i][j].parent_x = -1;
                grids_info_[i][j].parent_y = -1;
                grids_info_[i][j].f = double(INT32_MAX);
            }
        }

        grids_info_[source.first][source.second].parent_x = source.first;
        grids_info_[source.first][source.second].parent_y = source.second;
        grids_info_[source.first][source.second].f = 0.0;
        to_be_visited_.clear();
        astar_success_ = false;
        to_be_visited_.insert(std::make_pair(0.0, source));
        int idx = 0;

        while (!to_be_visited_.empty() && !astar_success_)
        {
            pose_idx curr = to_be_visited_.begin()->second;
            to_be_visited_.erase(to_be_visited_.begin());
            // std::cout<<"visiting grid: "<<curr.first<<" "<<curr.second<<std::endl;
            visited_[curr.first][curr.second] = true;

            for (int i = curr.first - 1; i <= curr.first + 1; i++)
            {
                for (int j = curr.second - 1; j <= curr.second + 1; j++)
                {
                    pose_idx neibor = std::make_pair(i, j);
                    if (neibor == curr)
                        continue;
                    if (is_valid(neibor) && !visited_[i][j])
                    {
                        if (terminate)
                        {
                            if (neibor == goal)
                            {
                                astar_success_ = true;
                                grids_info_[goal.first][goal.second].parent_x = curr.first;
                                grids_info_[goal.first][goal.second].parent_y = curr.second;
                                generate_path(source, goal, plan);
                                return true;
                            }
                        }
                        else
                        {
                            double dist;
                            if (within_vicinity(neibor, goal, dist))
                            {
                                astar_success_ = true;
                                grids_info_[neibor.first][neibor.second].parent_x = curr.first;
                                grids_info_[neibor.first][neibor.second].parent_y = curr.second;
                                generate_path(source, neibor, plan);
                                return true;
                            }
                        }

                        double curr_g = grids_info_[curr.first][curr.second].f + calculate_g(curr, neibor);
                        double curr_h = calculate_h(neibor, goal);
                        double curr_ob = calculate_obstacle_weight(neibor);
                        double curr_roadmap_proximity = calculate_roadmap_proximity_weight(neibor);
                        double curr_f = curr_g + curr_h + curr_ob + curr_roadmap_proximity;
                        if (curr_f < grids_info_[neibor.first][neibor.second].f)
                        {

                            light_up_grid(neibor, idx, 0.0, 0.0, 1.0, 0.02);
                            grids_info_[neibor.first][neibor.second].parent_x = curr.first;
                            grids_info_[neibor.first][neibor.second].parent_y = curr.second;
                            grids_info_[neibor.first][neibor.second].f = curr_f;
                            to_be_visited_.insert(std::make_pair(curr_f, neibor));
                        }
                        idx++;
                    }
                }
            }
        }
        return astar_success_;
    }

    bool MPCPlanner::switch_mode(bool mode)
    {
        is_rotating_ = mode;
        return true;
    }

    void MPCPlanner::generate_path(const pose_idx source, const pose_idx goal, std::vector<geometry_msgs::PoseStamped> &plan)
    {
        // clear the current path
        astar_path_.clear();
        // get the reverse path by tracing the parents from the goal pose
        std::stack<pose_idx> reverse_path;
        pose_idx curr = goal;
        reverse_path.push(curr);
        while (curr != source)
        {
            pose_idx curr_parent = (std::make_pair(grids_info_[curr.first][curr.second].parent_x,
                                                   grids_info_[curr.first][curr.second].parent_y));
            reverse_path.push(curr_parent);
            curr = curr_parent;
        }
        // get the path from the reverse path
        while (!reverse_path.empty())
        {
            astar_path_.push_back(reverse_path.top());
            reverse_path.pop();
        }
        // convert the astar_path_ to the desired format
        for (auto waypoint : astar_path_)
        {
            geometry_msgs::PoseStamped curr_pose;
            curr_pose.header.stamp = ros::Time::now();
            curr_pose.header.frame_id = lcr_->getGlobalFrameID();
            double wx, wy;
            mapToWorld(waypoint.first, waypoint.second, wx, wy);
            curr_pose.pose.position.x = wx;
            curr_pose.pose.position.y = wy;
            // std::cout<<"x: "<<wx<<" y: "<<wy<<std::endl;
            plan.push_back(curr_pose);
        }
    }

    double MPCPlanner::calculate_g(const pose_idx parent, const pose_idx child)
    {
        double distance = sqrt((parent.first - child.first) * (parent.first - child.first) + (parent.second - child.second) * (parent.second - child.second));
        return distance;
    }

    double MPCPlanner::calculate_h(const pose_idx curr, const pose_idx goal)
    {
        double euclidean_heuristic = double(sqrt((curr.first - goal.first) * (curr.first - goal.first) + (curr.second - goal.second) * (curr.second - goal.second)));
        return astar_heuristic_weight_ * euclidean_heuristic;
    }

    double MPCPlanner::calculate_obstacle_weight(const pose_idx curr)
    {
        int cost = static_cast<int>(lc_->getCost(curr.first, curr.second));
        if (cost < 150)
            return 0;
        else
        {
            return obstacle_weight_ * cost;
        }
    }

    double MPCPlanner::calculate_roadmap_proximity_weight(const pose_idx curr)
    {
        int curr_dist = 100 - roadmap_potential_field_[curr.first][curr.second];
        return roadmap_proximity_weight_ * curr_dist;
    }

    void MPCPlanner::light_up_grid(const pose_idx curr, const int idx, const float r, const float g, const float b, double size)
    {
        visualization_msgs::Marker curr_grid;
        curr_grid.header.frame_id = lcr_->getGlobalFrameID();
        curr_grid.header.stamp = ros::Time::now();
        curr_grid.ns = "hybrid_mpc_local_planner";
        curr_grid.id = idx;
        curr_grid.type = visualization_msgs::Marker::SPHERE;
        curr_grid.action = visualization_msgs::Marker::ADD;
        curr_grid.scale.x = curr_grid.scale.y = curr_grid.scale.z = size;
        curr_grid.color.r = r;
        curr_grid.color.g = g;
        curr_grid.color.b = b;
        curr_grid.color.a = 1.0f;
        curr_grid.lifetime = ros::Duration(4);
        double wx, wy;
        mapToWorld(curr.first, curr.second, wx, wy);
        curr_grid.pose.position.x = wx;
        curr_grid.pose.position.y = wy;
        curr_grid.pose.orientation.w = 1.0f;
        grid_light_pub_.publish(curr_grid);
    }

    bool MPCPlanner::within_vicinity(const pose_idx curr, const pose_idx goal, double &distance)
    {
        distance = lc_->getResolution() * double(sqrt((curr.first - goal.first) * (curr.first - goal.first) + (curr.second - goal.second) * (curr.second - goal.second)));
        return (distance <= viapoint_threshold_distance_);
    }

    bool MPCPlanner::update_local_topo_potential_field(std::vector<geometry_msgs::PoseStamped> &local_ref_path)
    {
        // re-initialize the potential field 2d vector values to 0
        // roadmap_potential_field_.resize(lc_->getSizeInCellsX(), std::vector<int>(0, lc_->getSizeInCellsY()));
        roadmap_potential_field_ = std::vector<std::vector<int>>(lc_->getSizeInCellsX(), std::vector<int>(lc_->getSizeInCellsY()));
        for (int i = 0; i < lc_->getSizeInCellsX(); i++)
        {
            for (int j = 0; j < lc_->getSizeInCellsY(); j++)
            {
                roadmap_potential_field_[i][j] = 0;
            }
        }
        if (local_ref_path.empty())
            return false;

        for (auto point : local_ref_path)
        {
            double mx, my;
            worldToMap(point.pose.position.x, point.pose.position.y, mx, my);
            roadmap_potential_field_[int(mx)][int(my)] = 100;
        }
        // inflate the roadmap_potential_field_
        int num_wave = 5;
        for (int i = lc_->getSizeInCellsY() - 1; i >= 0; i--)
        {
            for (int j = 0; j < lc_->getSizeInCellsX(); j++)
            {
                if (roadmap_potential_field_[j][i] == 100)
                {
                    inflate_cell(100, num_wave, j, i);
                }
            }
        }
        return true;
    }

    void MPCPlanner::inflate_cell(int core_potential, int num_wave, int x, int y)
    {
        int curr_potential = core_potential;
        for (int wave = 1; wave < num_wave + 1; wave++)
        {
            curr_potential *= pow(0.8, wave);
            // update the wave_front cells
            // top and bottom rows:
            for (int i = x - wave; i <= x + wave; i++)
            {
                if (i >= 0 && i < lc_->getSizeInCellsX())
                {
                    if ((y + wave) >= 0 && (y + wave) < lc_->getSizeInCellsY())
                        roadmap_potential_field_[i][y + wave] = std::max(roadmap_potential_field_[i][y + wave], curr_potential);
                    if ((y - wave) >= 0 && (y - wave) < lc_->getSizeInCellsY())
                        roadmap_potential_field_[i][y - wave] = std::max(roadmap_potential_field_[i][y - wave], curr_potential);
                }
            }
            // left and right columns:
            for (int j = y - wave; j <= y + wave; j++)
            {
                if (j >= 0 && j < lc_->getSizeInCellsY())
                {
                    if ((x - wave) >= 0 && (x - wave) < lc_->getSizeInCellsX())
                        roadmap_potential_field_[x - wave][j] = std::max(roadmap_potential_field_[x - wave][j], curr_potential);
                    if ((x + wave) >= 0 && (x + wave) < lc_->getSizeInCellsX())
                        roadmap_potential_field_[x + wave][j] = std::max(roadmap_potential_field_[x + wave][j], curr_potential);
                }
            }
        }
    }

    double MPCPlanner::quat_2_euler(const geometry_msgs::PoseStamped &msg)
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

}
