#include <ros/ros.h>
#include <iostream>

#include "icreate_navigation/multi_navigation.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "single_navigation_test");
    icreate::SingleNavigation single("Building 4", "Floor 1");
    std::cout << single.building_floor_num << std::endl;
    single.sendWaypointRequest("Building 4", "Floor 20");
    std::cout << single.building_floor_num << std::endl;

    std::vector<int> num = {1,2,3,4};

    std::cout << num.back() << std::endl;
    return 0;
}
