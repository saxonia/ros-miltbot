#include <ros/ros.h>
#include <iostream>
#include "sensor_msgs/LaserScan.h"

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    float val = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    int idx = val/2;
    // ROS_INFO("%f",val);
    std::vector<float> store;
    store = msg->ranges;
    // ROS_WARN("%ld",store.size());
    ROS_INFO("Middle Range: %f",store[idx]);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_test");
    ros::NodeHandle nh;

    ros::Subscriber scan_sub = nh.subscribe("scan", 1000, scanCallback);
    while(ros::ok()) {
        ros::spinOnce();
        // break;
    }
    return 0;
}