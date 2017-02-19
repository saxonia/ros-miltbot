#include <ros/ros.h>
#include <iostream>

#include "icreate_navigation/multi_navigation.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "single_navigation_test");
    icreate::MultiNavigation multi;
    icreate::Robot robot("Building 4", "Floor 20", "map", "base_footprint");
    multi.addSingleNavigation("Building 4", "Floor 20");
    std::cout << multi.getCountNavigation() << std::endl;
    multi.addSingleNavigation("Building 4", "Floor 17");
    std::cout << multi.getCountNavigation() << std::endl;
    ROS_INFO("Sax : %ld",multi.navigations_.size());
    for(int i = 0; i < multi.navigations_.size(); i++) {
        ROS_INFO("%s %s",multi.navigations_[i].building.c_str(), robot.building.c_str());
        ROS_INFO("%s %s",multi.navigations_[i].building_floor.c_str(), robot.building_floor.c_str());
        if(multi.navigations_[i].building == robot.building 
            && multi.navigations_[i].building_floor == robot.building_floor) {
            multi.nav_idx = i;
            break;
        }
    }
    ROS_INFO("%d",multi.nav_idx);
    multi.lift_navigation_step = 2;
    multi.runLiftNavigation(robot);

    
    return 0;
}
