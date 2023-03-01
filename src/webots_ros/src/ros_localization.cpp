#include <ros/init.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>

#include <webots_ros/get_uint64.h>
#include <webots_ros/supervisor_get_from_def.h>
#include <webots_ros/node_get_position.h>
#include <webots_ros/node_get_orientation.h>
#include <webots_ros/node_get_pose.h>
#include <webots_ros/node_enable_pose_tracking.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_localization", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    if (argc < 2) {
        throw std::runtime_error("No robot_name. Aborting.");
    }
    const std::string robot_name = std::string(argv[1]);

    ros::ServiceClient supervisor_get_from_def_client;
    webots_ros::supervisor_get_from_def supervisor_get_from_def_srv;
    supervisor_get_from_def_client = nh.serviceClient<webots_ros::supervisor_get_from_def>(robot_name + "/supervisor/get_from_def");
    // wait for the webots ready
    if (!supervisor_get_from_def_client.waitForExistence(ros::Duration(60.0))) {
        ROS_ERROR("WEBOTS supervisor get_from_def srv is not exist");
        return 0;
    }
    // wait for Origin and MobileArm to be updated
    ros::Duration(0.1).sleep();
    supervisor_get_from_def_srv.request.name = "Origin";
    supervisor_get_from_def_client.call(supervisor_get_from_def_srv);
    uint64_t origin_node = 0;
    if (supervisor_get_from_def_srv.response.node != 0) {
        ROS_INFO("Got node %ld from DEF Origin.", supervisor_get_from_def_srv.response.node);
        origin_node = supervisor_get_from_def_srv.response.node;
    } else {
        ROS_ERROR("Could not get node from DEF Origin.");
    }

    supervisor_get_from_def_srv.request.name = "MobileArm";
    supervisor_get_from_def_client.call(supervisor_get_from_def_srv);
    uint64_t mobile_arm_node = 0;
    if (supervisor_get_from_def_srv.response.node != 0) {
        ROS_INFO("Got node %ld from DEF MobileArm.", supervisor_get_from_def_srv.response.node);
        mobile_arm_node = supervisor_get_from_def_srv.response.node;
    } else {
        ROS_ERROR("Could not get node from DEF MobileArm.");
    }
    supervisor_get_from_def_client.shutdown();

    // supervisor_node_get_pose
    ros::ServiceClient supervisor_node_get_pose_client;
    webots_ros::node_get_pose supervisor_node_get_pose_srv;
    supervisor_node_get_pose_client = nh.serviceClient<webots_ros::node_get_pose>(robot_name + "/supervisor/node/get_pose");
    if (!supervisor_node_get_pose_client.waitForExistence(ros::Duration(1.0))) {
        ROS_ERROR("WEBOTS supervisor node_get_pose srv is not exist");
        return 0;
    }
    supervisor_node_get_pose_srv.request.from_node = mobile_arm_node;
    supervisor_node_get_pose_srv.request.node = origin_node;

    ros::WallRate rosRate(50);
    tf::TransformBroadcaster tfBroadcaster;
    while (::ros::ok() && ::ros::master::check()) {
        ROS_INFO("Get pose");
        supervisor_node_get_pose_client.call(supervisor_node_get_pose_srv);
        ROS_INFO("From_def get_pose rotation is:\nw=%f x=%f y=%f z=%f.", supervisor_node_get_pose_srv.response.pose.rotation.w,
                supervisor_node_get_pose_srv.response.pose.rotation.x, supervisor_node_get_pose_srv.response.pose.rotation.y,
                supervisor_node_get_pose_srv.response.pose.rotation.z);
        ROS_INFO("From_def get_pose translation is:\nx=%f y=%f z=%f.", supervisor_node_get_pose_srv.response.pose.translation.x,
                supervisor_node_get_pose_srv.response.pose.translation.y, supervisor_node_get_pose_srv.response.pose.translation.z);

        geometry_msgs::TransformStamped base_tf;
        base_tf.header.stamp = ros::Time::now();
        base_tf.header.frame_id = "world";
        base_tf.child_frame_id = "base_link";
        base_tf.transform.translation = supervisor_node_get_pose_srv.response.pose.translation;
        base_tf.transform.rotation = supervisor_node_get_pose_srv.response.pose.rotation;
        tfBroadcaster.sendTransform(base_tf);

        rosRate.sleep();
    }
    supervisor_node_get_pose_client.shutdown();
    ROS_INFO("Supervisor exit");

    return 0;
}
