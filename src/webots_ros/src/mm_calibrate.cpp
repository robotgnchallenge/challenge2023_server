//
// Created by robot on 2021/10/28.
//
#include "ros/ros.h"
#include <webots_ros/get_uint64.h>
#include <webots_ros/set_int.h>
#include <webots_ros/set_float.h>
#include <webots_ros/node_get_position.h>
#include <webots_ros/node_get_orientation.h>
#include <webots_ros/Float64Stamped.h>
#include <signal.h>
#include <std_msgs/String.h>
#include "tf/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

#define TIME_STEP 8

static uint64_t supervisorNodeID = 0;
static std::string baseModelName;
static std::string armModelName;
webots_ros::set_int timeStepSrv;
webots_ros::set_float wheelSrv;
ros::ServiceClient timeStepClient;
ros::ServiceClient middleLeftWheelVelocityClient;
ros::ServiceClient middleRightWheelVelocityClient;
ros::ServiceClient mirSupervisorClient;
ros::ServiceClient mirPositionClient;
ros::ServiceClient mirOrientationClient;
webots_ros::node_get_position supervisorPositionSrv;
webots_ros::node_get_orientation supervisorOrientationSrv;


void resetZero(){
    ROS_INFO("Reset all the control commands");
    wheelSrv.request.value = 0;
    middleLeftWheelVelocityClient.call(wheelSrv);
    middleRightWheelVelocityClient.call(wheelSrv);
}

void quit(int sig) {
    ROS_ERROR("shut down the ros node and reset all the service requests");
    timeStepSrv.request.value = 0;
    timeStepClient.call(timeStepSrv);
    resetZero();
    ros::shutdown();
    return;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mm_calibrate", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    // handling the shutdown request
    signal(SIGINT, quit);
    baseModelName = "MirKinova";
    armModelName = "jaco1";

    timeStepClient = n.serviceClient<webots_ros::set_int>(baseModelName + "/robot/time_step");
    timeStepSrv.request.value = TIME_STEP;
    if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success){
        ROS_ERROR("Failed to call service time_step to update robot's time step.");
    }
    resetZero();
    // define the mir base supervisor related service clients
    mirSupervisorClient = n.serviceClient<webots_ros::get_uint64>(baseModelName + "/supervisor/get_self", true);
    webots_ros::get_uint64 supervisorIDSrv;
    supervisorIDSrv.request.ask = true;
    mirSupervisorClient.waitForExistence(ros::Duration(0.5));
    mirSupervisorClient.call(supervisorIDSrv);
    std::cout<<"mir supervisor node id found: "<<supervisorIDSrv.response.value;
    supervisorNodeID = supervisorIDSrv.response.value;
    mirPositionClient = n.serviceClient<webots_ros::node_get_position>(baseModelName + "/supervisor/node/get_position", true);
    mirOrientationClient = n.serviceClient<webots_ros::node_get_orientation>(baseModelName + "/supervisor/node/get_orientation", true);
    // Main loop
    while (ros::ok()) {
        if(supervisorNodeID != 0){
            supervisorPositionSrv.request.node = supervisorNodeID;
            supervisorOrientationSrv.request.node = supervisorNodeID;
            mirPositionClient.call(supervisorPositionSrv);
            mirOrientationClient.call(supervisorOrientationSrv);
            // convert the quaternion orientation to RPY expression, since in webots the upper direction axis is y axis, so the pitch angle is the mobile base heading angle
            geometry_msgs::Quaternion orientation = supervisorOrientationSrv.response.orientation;
            tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
            double yaw, pitch, roll;
            mat.getEulerYPR(yaw, pitch, roll);
            double x, y;
            x = supervisorPositionSrv.response.position.x;
            y = supervisorPositionSrv.response.position.y;
            // print out the current base pose
            std::cout<<"quaternion x y z w "<<orientation.x<<", "<<orientation.y<<", "<<orientation.z<<", "<<orientation.w<<std::endl;
            std::cout<<"translation x: "<<x<<" y: "<<y<<std::endl;
            std::cout<<"yaw: "<<yaw<<" roll: "<<roll<<" pitch: "<<pitch<<std::endl;
            // if(fabs(yaw+3.141593)<0.1 && fabs(roll+1.57)<0.1){
            //     // second quater
            //     pitch = 3.141593 - pitch;
            // }
            // if((fabs(yaw+3.141593)<0.1 || fabs(yaw-3.141593)<0.1) && fabs(roll-1.57)<0.1){
            //     // second & third quater 
            //     yaw = 3.141593 - yaw;
            //     std::cout<<"in sencond or third quater"<<std::endl;
            // }
            // else if(fabs(yaw)<0.1 && fabs(roll+1.57)<0.1){
            //     // first & fourth quater
            //     //pitch = pitch;
            //     std::cout<<"in first or fourth quater"<<std::endl;
            // }
            std::cout<<"Current heading angle is "<<yaw*180/3.1415926<<std::endl;
        }else{
            std::cout<<"waiting for supervisor ID to be found";
        }
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    ros::spin();

    return 0;
}


