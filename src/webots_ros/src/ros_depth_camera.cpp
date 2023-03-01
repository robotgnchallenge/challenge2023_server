#include "ros/ros.h"

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

ros::Subscriber camera_info_sub_;
ros::Publisher camera_info_pub_;
ros::Publisher camera_pose_pub_;


sensor_msgs::CameraInfo info;

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    info.header = msg->header;
    camera_info_pub_.publish(info);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_depth_camera", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    if (argc < 2) {
        throw std::runtime_error("No robot_name. Aborting.");
    }
    const std::string robot_name = std::string(argv[1]);

    info.width = 640;
    info.height = 360;
    info.distortion_model = "plumb_bob";
    double cx = info.width / 2.0;
    double cy = info.height / 2.0;
    double fx = info.width / (2.0 * tan(2.05949 / 2));
    double fy = fx;
    info.D = {0,0,0,0,0};
    info.K = {fx, 0, cx, 0, fy, cy, 0, 0, 1.0};
    info.R = {1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0};
    info.P = {fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1.0, 0};
    camera_info_sub_ = nh.subscribe("image_rect", 100, imageCallback);
    camera_info_pub_ = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 100);
    camera_pose_pub_ = nh.advertise<geometry_msgs::TransformStamped>("transform", 100);

    ros::WallRate rosRate(50);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    // wait for the webots ready
    if (!tfBuffer.canTransform("world", robot_name + "/depth_camera", ros::Time(0), ros::Duration(60.0))) {
        ROS_ERROR("Cannot Transform from world to %s/depth_camera", robot_name.c_str());
        return 1;
    }
    geometry_msgs::TransformStamped transform_msg;
    while (::ros::ok() && ::ros::master::check()) {
        transform_msg = tfBuffer.lookupTransform("world", robot_name + "/depth_camera", ros::Time::now(), ros::Duration(1.0));
        camera_pose_pub_.publish(transform_msg);
        ros::spinOnce();
        rosRate.sleep();
    }
    return 0;
}
