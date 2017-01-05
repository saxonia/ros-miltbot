#include <ros/ros.h>
#include <iostream>

bool startNavigation() {
    std::cout << "Start Lift Navigation" << std::endl;
    std::cout << "Please press \"y\" to start navigation " << std::endl;
    std::string input;
    std::cin >> input;
    if(input == "y" || input == "Y")
        return true;
    else
        return false;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lift_navigation");
    ros::NodeHandle nh;
    if(!startNavigation()) {
        std::cout << std::endl;
        std::cout << "Stop Lift Navigation" << std::endl;
        return -1;
    } 

    
    

    return 0;
}