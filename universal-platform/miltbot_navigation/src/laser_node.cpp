#include <ros/ros.h>
#include <iostream>
#include "sensor_msgs/LaserScan.h"

#include "miltbot_navigation/GetMiddleRange.h"
#include "miltbot_navigation/IsLiftOpen.h"


float mid_range;
float old_mid_range;
float mid_range_threshold;
bool is_lift_open;
// std::vector<float> range_queue(10);

bool verifyLiftDoor(std::vector<float> store, int mid_idx) {
    // int lower = mid_idx-15;
    // int upper = mid_idx+15;
    // float sum;
    // for(int i = lower; i <= upper;i++) {
    //     sum += store[i];
    // }
    // float avg = sum / (upper-lower);
    // ROS_INFO("Average Range %lf",avg);
    ROS_INFO("Diff Range %lf",fabs(store[mid_idx] - old_mid_range));
    // if(fabs(avg - old_mid_range) > mid_range_threshold) {
    if(fabs(store[mid_idx] - old_mid_range) > mid_range_threshold) {
        is_lift_open = true;
    }
    else {
        is_lift_open = false;
    }
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    float val = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    int idx = val/2;
    std::vector<float> store;
    store = msg->ranges;
    mid_range = store[idx];
    verifyLiftDoor(store, idx);
    old_mid_range = mid_range;
}

void timerCallback(const ros::TimerEvent &event) {
    ROS_INFO("Middle Range: %f", mid_range);
}

bool getMiddleRangeService(miltbot_navigation::GetMiddleRange::Request &req,
                            miltbot_navigation::GetMiddleRange::Response &res) {
    res.mid_range = mid_range;
    return true;
}

bool isLiftOpenService(miltbot_navigation::IsLiftOpen::Request &req,
                       miltbot_navigation::IsLiftOpen::Response &res) {
    res.is_lift_open = is_lift_open;
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_node");
    ros::NodeHandle nh;

    std::string laser_scan_sub_topic_name("scan");
    std::string get_middle_range_service_name("get_middle_range");
    std::string is_lift_open_service_name("is_lift_open");

    // nh.param("laser_scan_sub_topic",laser_scan_sub_topic_name,laser_scan_sub_topic_name);
    // nh.param("get_middle_range_service",get_middle_range_service_name,get_middle_range_service_name);
    nh.param("laser_node/mid_range_threshold", mid_range_threshold, mid_range_threshold);

    ros::Subscriber scan_sub = nh.subscribe(laser_scan_sub_topic_name, 1, scanCallback);
    ros::ServiceServer get_middle_range_service = nh.advertiseService(get_middle_range_service_name, getMiddleRangeService);
    ros::ServiceServer is_lift_open_service = nh.advertiseService(is_lift_open_service_name, isLiftOpenService);
    ros::Timer info_timer = nh.createTimer(ros::Duration(5), timerCallback);
    ros::spin();
    return 0;
}