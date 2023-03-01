#include "ros/ros.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <string>
#include <iomanip>

ros::Subscriber camera_info_sub_;
ros::Publisher camera_info_pub_;

sensor_msgs::CameraInfo info;

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    info.header = msg->header;
    camera_info_pub_.publish(info);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_camera_info", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    if (argc < 5) {
        throw std::runtime_error("Param should be [robot_name width height fov]");
    }
    const std::string robot_name = std::string(argv[1]);
    const int width = std::stoi(std::string(argv[2]));
    const int height = std::stoi(std::string(argv[3]));
    const double fov = std::stod(std::string(argv[4]));

    info.width = width;
    info.height = height;
    info.distortion_model = "plumb_bob";
    double cx = info.width / 2.0;
    double cy = info.height / 2.0;
    double fx = info.width / (2.0 * tan(fov / 2));
    double fy = fx;
    info.D = {0,0,0,0,0};
    info.K = {fx, 0, cx, 0, fy, cy, 0, 0, 1.0};
    info.R = {1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0};
    info.P = {fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1.0, 0};
    camera_info_sub_ = nh.subscribe("image_rect", 100, imageCallback);
    camera_info_pub_ = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 100);

    ros::spin();
    return 0;
}
