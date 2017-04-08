#include <iostream>
#include <ros/ros.h>

#include <create_node/TurtlebotSensorState.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include "miltbot_system/RunSystem.h"
#include "miltbot_system/AddTarget.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int voltage_limit;
ros::ServiceClient run_system_client;
ros::ServiceClient add_target_client;

long generateTargetId() {
    return rand();
}

void loadBaseStationWaypoints() {
    // callGetBaseStationWaypoints();
}

void callRunSystemService(bool status) {
    miltbot_system::RunSystem srv;
    srv.request.status = status;
    if(run_system_client.call(srv)) {
        bool flag = srv.response.success;
    }
    else {
        ROS_ERROR("Failed to call service run_system");
    }
}

bool callAddTargetService() {
    miltbot_system::AddTarget srv;
    long id = generateTargetId();
    // srv.request.waypoint = data;
    srv.request.waypoint.id = id;
    if(add_target_client.call(srv)) {
        bool flag = srv.response.success;
    }
    else {
        ROS_ERROR("Failed to call service add_target");
    }
    return true;
} 

void runSystemRecovery() {
    callRunSystemService(false);
    callAddTargetService();
}

void turtlebotSensorCallback(const create_node::TurtlebotSensorState &msg) {
    int data = msg.voltage;
    ROS_INFO("Voltage : %d",data);
    if(data < voltage_limit) {
        runSystemRecovery();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "power_node");
    ros::NodeHandle nh;

    std::string turtlebot_state_sub_topic_name("icreate_node/sensor_state");
    std::string move_base_cancel_pub_topic_name("move_base/cancel");
    std::string run_system_service_name("run_system");
    std::string add_target_service_name("add_target");

    // nh.param("bumper_node/base_frame_id", base_frame_id, base_frame_id);
    // nh.param("bumper_node/robot_frame_id", robot_frame_id, robot_frame_id);
    nh.param("power_node/voltage_limit", voltage_limit, voltage_limit);
    nh.param("turtlebot_state_sub_topic", turtlebot_state_sub_topic_name, turtlebot_state_sub_topic_name);
    nh.param("move_base_cancel_pub_topic", move_base_cancel_pub_topic_name, move_base_cancel_pub_topic_name);
    nh.param("run_system_service", run_system_service_name, run_system_service_name);

    loadBaseStationWaypoints();

    ros::Subscriber turtlebot_sensor_sub = nh.subscribe(turtlebot_state_sub_topic_name, 10, turtlebotSensorCallback);
    // move_base_cancel_pub = nh.advertise<actionlib_msgs::GoalID>(move_base_cancel_pub_topic_name, 1);
    run_system_client = nh.serviceClient<miltbot_system::RunSystem>(run_system_service_name);
    add_target_client = nh.serviceClient<miltbot_system::AddTarget>(add_target_service_name);
    ros::spin();

    return 0;
}