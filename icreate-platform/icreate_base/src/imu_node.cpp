#include <ros/ros.h>
#include <iostream>
#include <serial/serial.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle nh;

    serial::Serial serial("/dev/ttyACM0", 9600);
    while(ros::ok()) {
        std::string res = serial.read(100);
        ROS_INFO("Get data: %s",res.c_str());
        sleep(1);
    }
    return 0;
}