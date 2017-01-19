#include <ros/ros.h>
#include <iostream>

#include "icreate_transportation/single_transportation.h"
#include "icreate_navigation/single_navigation.h"

//Client Service of move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


int main(int argc, char** argv) {
    ros::init(argc, argv, "single_test_new");
    ros::NodeHandle nh;

    //Initialize Class
    icreate::SingleTransportation single_transportation;
    icreate::SingleNavigation single_navigation("Building 4", "Floor 20");

    std::string move_base_topic_name("/move_base");
    // std::string base_frame_id("/map");
    std::string base_frame_id("map");
    std::string robot_frame_id("base_footprint");
    std::string package_name("icreate_navigation");
	int polling_rate(30);
    int timer_duration(10);
    nh.param("/waypoint_navigation/move_base_topic", move_base_topic_name, move_base_topic_name);
    nh.param("/waypoint_navigation/base_frame_id", base_frame_id, base_frame_id);
    nh.param("/waypoint_navigation/package_name", package_name, package_name); 
	nh.param("/waypoint_navigation/polling_rate", polling_rate, polling_rate);
    nh.param("/waypoint_navigation/timer_duration", timer_duration, timer_duration);
    MoveBaseClient ac(move_base_topic_name, true);

    // Callback polling Rate 
    ros::Rate r(polling_rate);

    single_navigation.sendWaypointRequest("Building 4", "Floor 20");
    // single_navigation.showNa


    return 0;
}