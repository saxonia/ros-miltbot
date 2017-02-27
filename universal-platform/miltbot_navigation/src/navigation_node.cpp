#include <ros/ros.h>
// #include "miltbot_navigation/RunMoveBase.h"

#include "miltbot_navigation/navigation.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigation_node");
    ros::NodeHandle nh;

    std::string move_base_topic_name("move_base");
    std::string base_frame_id("map");
    std::string robot_frame_id("base_footprint");
    std::string package_name("icreate_navigation");
    std::string building_name("Building 4");
    std::string building_floor_name("Floor 20");
	int polling_rate(30);
    int timer_duration(10);
    nh.param("/waypoint_navigation/move_base_topic", move_base_topic_name, move_base_topic_name);
    nh.param("/waypoint_navigation/base_frame_id", base_frame_id, base_frame_id);
    nh.param("/waypoint_navigation/robot_frame_id", robot_frame_id, robot_frame_id);
    nh.param("/waypoint_navigation/package_name", package_name, package_name); 
	nh.param("/waypoint_navigation/polling_rate", polling_rate, polling_rate);
    nh.param("/waypoint_navigation/timer_duration", timer_duration, timer_duration);
    nh.param("/waypoint_navigation/building", building_name, building_name);
    nh.param("/waypoint_navigation/building_floor", building_floor_name, building_floor_name);

    miltbot::Navigation navigation(base_frame_id, robot_frame_id, building_name, building_floor_name);

    // Callback polling Rate 
    ros::Rate r(polling_rate);

    // start();
    navigation.createTimer(timer_duration);
    // navigation.update();
    // ros::spin();

    while(ros::ok()) {
        ros::spinOnce();
        r.sleep();  
        navigation.update();
    }

}