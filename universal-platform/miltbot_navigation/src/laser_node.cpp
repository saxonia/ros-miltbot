#include <ros/ros.h>
#include <iostream>
#include "sensor_msgs/LaserScan.h"
#include "icreate_lift_navigation/GetMiddleRange.h"

float mid_range;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    float val = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    int idx = val/2;
    std::vector<float> store;
    store = msg->ranges;
    mid_range = store[idx];
    // ROS_WARN("%ld",store.size());
    ROS_INFO("Middle Range: %f",store[idx]);
}

bool getMiddleRangeService(icreate_lift_navigation::GetMiddleRange::Request &req,
                            icreate_lift_navigation::GetMiddleRange::Response &res) {
    res.mid_range = mid_range;
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_node");
    ros::NodeHandle nh;

    std::string laser_scan_sub_topic_name("scan");
    std::string get_middle_range_service_name("get_middle_range");

    nh.param("laser_scan_sub_topic",laser_scan_sub_topic_name,laser_scan_sub_topic_name);
    nh.param("get_middle_range_service",get_middle_range_service_name,get_middle_range_service_name);

    ros::Subscriber scan_sub = nh.subscribe(laser_scan_sub_topic_name, 1, scanCallback);
    ros::ServiceServer service = nh.advertiseService(get_middle_range_service_name, getMiddleRangeService);
    ros::spin();
    return 0;
}