#include <iostream>
#include <ros/ros.h>

#include <create_node/TurtlebotSensorState.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include "miltbot_common/Waypoint.h"
#include "miltbot_map/GetBaseStationList.h"
#include "miltbot_navigation/NavigationState.h"
#include "miltbot_system/RunSystem.h"
#include "miltbot_system/AddTarget.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int voltage_limit;
bool isRunChargingNavigation;
ros::ServiceClient run_system_client;
ros::ServiceClient add_target_client;
ros::ServiceClient get_base_station_client;

std::vector<miltbot_common::Waypoint> waypoints;
miltbot_navigation::NavigationState navigation_state;

long generateTargetId() {
    return rand();
}

miltbot_common::Waypoint selectBaseStation() {
    miltbot_common::Waypoint res;
    for(int i = 0; i < waypoints.size(); i++) {
        if(waypoints[i].building == navigation_state.building) {
            if(waypoints[i].building_floor == navigation_state.building_floor) {
                res = waypoints[i];
            }
        }
    }
    return res;
}

void callGetBaseStationWaypointService() {
    miltbot_map::GetBaseStationList srv;
    if(get_base_station_client.call(srv)) {
        waypoints = srv.response.waypoints;
    }
    else {
        ROS_ERROR("Failed to call service get_base_station");
    }
}

void callRunChargingNavigationService(bool status) {
    miltbot_system::RunSystem srv;
    srv.request.status = status;
    if(run_system_client.call(srv)) {
        bool flag = srv.response.success;
    }
    else {
        ROS_ERROR("Failed to call service run_charging_navigation");
    }
}

bool callAddTargetService() {
    miltbot_system::AddTarget srv;
    long id = generateTargetId();
    srv.request.waypoint = selectBaseStation();
    srv.request.waypoint.id = id;
    srv.request.waypoint.task = "BACKTOCHARGE";
    if(add_target_client.call(srv)) {
        bool flag = srv.response.success;
    }
    else {
        ROS_ERROR("Failed to call service add_target");
    }
    return true;
} 

void loadBaseStationWaypoints() {
    callGetBaseStationWaypointService();
}

void runChargingNavigation() {
    callRunChargingNavigationService(false);
    callAddTargetService();
}

void turtlebotSensorCallback(const create_node::TurtlebotSensorState &msg) {
    int voltage = msg.voltage;
    // bool is_finish_back_to_charge = msg.is_finish_back_to_charge;
    ROS_INFO("Voltage : %d",voltage);
    if(voltage < voltage_limit && !isRunChargingNavigation) {
        runChargingNavigation();
        isRunChargingNavigation = true;
    }
    //ถ้าชาร์ตเต็มทำอะไรต่อ
    // else if(voltage > voltage_limit && isRunChargingNavigation) {

    // }
}

void navigationStateCallback(const miltbot_navigation::NavigationState &msg) {
    navigation_state = msg;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "power_node");
    ros::NodeHandle nh;

    std::string turtlebot_state_sub_topic_name("icreate_node/sensor_state");
    std::string navigation_state_sub_topic_name("navigation_state");
    std::string move_base_cancel_pub_topic_name("move_base/cancel");
    std::string run_system_service_name("run_system");
    std::string add_target_service_name("add_target");
    std::string get_base_station_service_name("get_base_station");
    voltage_limit = 15510;
    isRunChargingNavigation = false;

    // nh.param("bumper_node/base_frame_id", base_frame_id, base_frame_id);
    // nh.param("bumper_node/robot_frame_id", robot_frame_id, robot_frame_id);
    nh.param("power_node/voltage_limit", voltage_limit, voltage_limit);
    nh.param("turtlebot_state_sub_topic", turtlebot_state_sub_topic_name, turtlebot_state_sub_topic_name);
    nh.param("navigation_state_sub_topic", navigation_state_sub_topic_name, navigation_state_sub_topic_name);
    nh.param("move_base_cancel_pub_topic", move_base_cancel_pub_topic_name, move_base_cancel_pub_topic_name);
    nh.param("run_system_service", run_system_service_name, run_system_service_name);
    nh.param("add_target_service", add_target_service_name, add_target_service_name);
    nh.param("get_base_station_service", get_base_station_service_name, get_base_station_service_name);

    ros::Subscriber turtlebot_sensor_sub = nh.subscribe(turtlebot_state_sub_topic_name, 10, turtlebotSensorCallback);
    ros::Subscriber navigation_state_sub = nh.subscribe(navigation_state_sub_topic_name, 10, navigationStateCallback);
    // move_base_cancel_pub = nh.advertise<actionlib_msgs::GoalID>(move_base_cancel_pub_topic_name, 1);
    run_system_client = nh.serviceClient<miltbot_system::RunSystem>(run_system_service_name);
    add_target_client = nh.serviceClient<miltbot_system::AddTarget>(add_target_service_name);
    get_base_station_client = nh.serviceClient<miltbot_map::GetBaseStationList>(get_base_station_service_name);

    loadBaseStationWaypoints();
    ros::spin();

    return 0;
}