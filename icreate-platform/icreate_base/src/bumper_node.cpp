#include <ros/ros.h>
#include <iostream>
#include "create_node/TurtlebotSensorState.h"


void turtlebotSensorCallback(const create_node::TurtlebotSensorState &msg) {
    ROS_DEBUG_STREAM(msg);
    ROS_DEBUG("Bump Wheel Drops : %d",msg.bumps_wheeldrops);
}

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "bumper_node");
    ros::NodeHandle nh;
    ros::Subscriber turtlebot_sensor_sub = nh.subscribe("icreate_node/sensor_state", 10, turtlebotSensorCallback);
    ros::spin();
    return 0;
}