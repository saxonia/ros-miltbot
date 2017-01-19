#include <ros/ros.h>
#include <iostream>
#include <algorithm>    // std::find
#include <string>
#include <exception>

#include <miltbot_map/GetWaypointName.h>

struct Robot {
    int id_;
    std::string position_;

    Robot() { }

    Robot(int id, std::string position) {
        id_ = id;
        position_ = position;
    }
};

// std::vector<Robot> robot_list;
std::vector<std::string> waypoint_name;
std::vector<std::string> floor_list;
std::vector<std::string> robot_list;
int navigation_mode;
int selected_waypoint;
int selected_robot;


std::pair<std::vector<std::string>, std::vector<std::string> > getWaypointNameClient(ros::NodeHandle &nh) {
    ros::ServiceClient client = nh.serviceClient<miltbot_map::GetWaypointName>("get_waypoint_name");
    miltbot_map::GetWaypointName srv;
    srv.request.building = "Building 3";
    // srv.request.floor = "Floor 20";
    std::pair<std::vector<std::string>, std::vector<std::string> > res;
    if(client.call(srv)) {
        res.first = srv.response.data;
        res.second = srv.response.floor;
    }
    else {
        ROS_ERROR("Failed to call service get_waypoint_name");
    }
    return res;
}

std::vector<std::string> getRobotListClient(ros::NodeHandle &nh) {
    std::vector<std::string> res;
    res.push_back("icreate1");
    return res;
}

void init(ros::NodeHandle &nh) {
    // Robot a(1,"a");
    // robot_list.push_back(a);
    // a = Robot(2,"b");
    // robot_list.push_back(a);
    std::pair<std::vector<std::string>, std::vector<std::string> > tmp;
    tmp = getWaypointNameClient(nh);
    waypoint_name = tmp.first;
    floor_list = tmp.second;
    robot_list = getRobotListClient(nh);
}

void showSelectMode() {
    navigation_mode = -1;
    // Select Mode for Transportation 
	std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;
	std::cout << "[AGENT] Select Mode" <<std::endl;
	std::cout << "[ 0 ] Go to Specific Point." <<std::endl;
	std::cout << "[ 1 ] Delivery and Come Back to This Place." <<std::endl;
	std::cout << "[ 2 ] Delivery and Come Back to Base Station" <<std::endl;
	std::cout << "[ 3 ] Execute The Memorized Sequence" <<std::endl;
	std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;
	std::cout << "Select Mode[0,1,2] : " ;
    // int input; 
	std::cin  >> navigation_mode;
    std::cout << "You Selected : " << navigation_mode <<std::endl;
	std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;

}

void showSelectWaypoint() {
    selected_waypoint = -1;
    while(ros::ok()) {
        //Show Waypoint List
        for(int i = 0 ; i < waypoint_name.size() ; i++) {
            std::cout <<"["<<i<<"] " << waypoint_name[i] << " : " << floor_list[i] << std::endl; 
        } 
        //Get select waypoint input
        std::cout << "[AGENT] Input Target Waypoints ID : ";
        std::string input;
        std::cin >> input;
        try {
            selected_waypoint = std::atoi(input.c_str());
        }
        catch(std::exception& e) {
            std::cout << "Standard exception: " << e.what() << std::endl;
            continue;
        } 
        
        if(selected_waypoint > -1 && selected_waypoint < waypoint_name.size()) {
            showSelectMode();
            break;
        }
        else {
            std::cout << "[AGENT] Wrong Waypoint ID" << std::endl;
        }
    }
}

void showSelectRobot() {
    selected_robot = -1;
    while(ros::ok()) {
        //Show Waypoint List
        for(int i = 0 ; i < robot_list.size() ; i++) {
            std::cout <<"["<<i<<"] " << robot_list[i] << std::endl; 
        }
        std::cout << "Please select robot : " ;
        std::cin >> selected_robot;
        break;
    } 
}

void setNavigation() {
    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "fleet_manager_fake");
    ros::NodeHandle nh;

    init(nh);

    while(ros::ok()) {
        std::cout << "Welcome to Sending Supplies program" << std::endl;
        std::cout << "Please press \"y\" to continue or \"n\" to cancel" << std::endl;
        std::cout << "Your command: " ;
        std::string command;
        std::cin >> command;
        if(command == "n") {
            break;
        }
        else if(command != "y") {
            continue;
        }
        showSelectWaypoint();
        showSelectRobot();
        setNavigation();
    }

    return 0;
}