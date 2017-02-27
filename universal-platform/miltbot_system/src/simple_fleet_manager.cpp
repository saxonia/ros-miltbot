#include <ros/ros.h>
#include <iostream>
#include <map>
// #include <icreate_system/AddRobot.h>
// #include <icreate_system/AddTarget.h>

#include "miltbot_navigation/move_base_data.h"
#include "miltbot_map/Waypoint.h"
#include "miltbot_map/GetWaypointList.h"
#include "miltbot_system/AddTarget.h"

using namespace miltbot;

// std::vector<MoveBaseGoalData> lifts;
typedef std::pair<std::string, std::string> Key;

std::map<Key, std::vector<MoveBaseGoalData> > targets;
// MoveBaseGoalData target;

void addTargetData(std::string building, std::string building_floor, std::vector<miltbot_map::Waypoint> waypoints) {
    Key key = std::make_pair(building, building_floor);
    std::vector<MoveBaseGoalData> data;
    for(int i = 0; i < waypoints.size(); i++) {
           MoveBaseGoalData one_data;
           one_data.setGoalName(waypoints[i].name);
           one_data.setBuilding(waypoints[i].building);
           one_data.setBuildingFloor(waypoints[i].floor);
           one_data.setGoal(waypoints[i].goal);
           data.push_back(one_data);
    }
    targets[key] = data;
}

bool callGetWaypointService(ros::NodeHandle &nh, std::string building, std::string building_floor) {
    ros::ServiceClient client = nh.serviceClient<miltbot_map::GetWaypointList>("get_waypoint_list");
    miltbot_map::GetWaypointList srv;
    srv.request.building = building;
    srv.request.floor = building_floor;
    std::vector<miltbot_map::Waypoint> waypoints;
    if(client.call(srv)) {
        waypoints = srv.response.waypoints;
        if(waypoints.size() > 0) {
            addTargetData(building, building_floor, waypoints);
        }
        else {
            ROS_WARN("Failed to receive waypoints");
        }
        return true;
    }
    else {
        ROS_ERROR("Failed to call service get_waypoint_list");
        return false;
    }
}

void loadWaypoint(ros::NodeHandle &nh) {
    std::string building = "Building 4";
    std::string building_floor = "Floor 20";
    callGetWaypointService(nh, building, building_floor);
    building = "Building 4";
    building_floor = "Floor 17";
    callGetWaypointService(nh, building, building_floor);
}

void addRobot(ros::NodeHandle &nh) {
    // ros::ServiceClient client = nh.serviceClient<icreate_system::AddRobot>("add_robot");
    // icreate_system::AddRobot srv;
    // srv.request.building = "";
    // srv.request.building_floor = "";
    // srv.request.base_frame_id = "";
    // srv.request.robot_frame_id = "";
    // if(client.call(srv)) {
        
    //     // mid_range = srv.response.mid_range;
    //     // ROS_INFO("Get Mid Range data: %f",mid_range);
    // }
    // else {
    //     ROS_ERROR("Fail to call Service add_robot");
    // }
}

void addTarget(ros::NodeHandle &nh, MoveBaseGoalData data) {
    ros::ServiceClient client = nh.serviceClient<miltbot_system::AddTarget>("add_target");
    miltbot_system::AddTarget srv;
    miltbot_map::Waypoint waypoint;
    waypoint.name = data.getGoalName();
    waypoint.goal = data.getGoal();
    waypoint.building = data.getBuilding();
    waypoint.floor = data.getBuildingFloor();
    srv.request.waypoint = waypoint;
    srv.request.task = "SINGLERUN";
    srv.request.priority = data.getPriority();
    if(client.call(srv)) {
        bool flag = srv.response.success;
    }
    else {
        ROS_ERROR("Failed to call service add_target");
    }
}

void setTargetPriority(MoveBaseGoalData &data) {
    std::cout << "Please insert target priority";
    std::cout << "Your Input: " << std::endl;
    int priority;
    std::cin >> priority;
    data.priority = priority;
}

void displayWaypoints(std::vector<MoveBaseGoalData> data) {
    std::cout << "Please Select Waypoint" << std::endl;
    for(int i = 0 ; i < data.size() ; i++) {
      std::cout <<"["<<i<<"] " << data[i].getGoalName() <<std::endl; 
    }
    std::cout <<"["<<99<<"] " << "Cancel" << " " <<std::endl; 
}

void displayBuildingFloor() {
    int i = 0;
    std::cout << "Please Select Floor" << std::endl;
    for(std::map<Key, std::vector<MoveBaseGoalData> >::iterator it = targets.begin(); it != targets.end(); it++, i++) {
        std::cout <<"["<<i<<"] " << (it->first).second <<std::endl; 
    }
    std::cout <<"["<<99<<"] " << "Cancel" <<std::endl;
}

bool showWaypointMenu(MoveBaseGoalData &target) {
    int selected_floor;
    while(ros::ok()) {
        //แสดงเรียงลำดับตาม อาคาร ชั้น จุดหมาย
        displayBuildingFloor();
        // Ask for ID and wait user input
        std::cout << "[AGENT] Input Target Floor : ";
        std::cin >> selected_floor;
        std::cout << std::endl;
        if((selected_floor < 0 || selected_floor >= targets.size()) && selected_floor != 99) {
            ROS_WARN("Wrong Select Floor To Navigate Try Again");
            continue;
        }
        else if(selected_floor == 99) {
            std::cout << std::endl;
            std::cout << std::endl;
            return false;
        }
        break;
        // return std::make_pair(selected_floor, selected_point);
    }
    std::map<Key, std::vector<MoveBaseGoalData> >::iterator it = targets.begin();
    std::advance(it, selected_floor);
    std::vector<MoveBaseGoalData> data = it->second;
    while(ros::ok()) {
        displayWaypoints(data);
	    // Ask for ID and wait user input
	    std::cout << "[AGENT] Input Target Waypoints ID : " ;
        int selected_point;
	    std::cin >> selected_point;
        std::cout << std::endl;
        if((selected_point < 0 || selected_point >= data.size()) && selected_point != 99) {
            ROS_WARN("Wrong Select Point To Navigate Try Again");
            continue;
        }
        else if(selected_point == 99) {
            return false;
        }
        else {
            target = data[selected_point];
            setTargetPriority(target);
            std::cout << "You Finish Add Waypoint" << std::endl;
            std::cout << std::endl;
            return true;
        }
    }
}

void runAddTarget(ros::NodeHandle &nh) {
    MoveBaseGoalData data;
    showWaypointMenu(data);
    addTarget(nh,data);
}

void showFleetManagerMenu() {
    std::cout << "Start Simple Fleet Manager System" << std::endl;
    std::cout << "Please Select Your Command" << std::endl;
    // std::cout << "[0] Add Robot To System" << std::endl;
    // std::cout << "[1] Delete Robot From System" << std::endl;
    std::cout << "[2] Add Target To Robot" << std::endl;
    std::cout << "[3] Delete Target From Robot" << std::endl;
    std::cout << "[4] Send Robot To Receive Supplies" << std::endl;
    std::cout << "[5] Send Robot To Send Suoolies" << std::endl;
    std::cout << "[10] Exit Program" << std::endl;
    std::cout << "[99] Stop System" << std::endl;
}

bool runFleetManagerMenu(ros::NodeHandle &nh) {
    showFleetManagerMenu();
    std::cout << "Your Command: ";
    int command;
    std::cin >> command;
    // std::cout << "Your Command is " << command << std::endl;
    std::cout << std::endl;
    switch(command) {
        // case 0:
        //     std::cout << "0" << std::endl;
        //     addRobot(nh);
        //     break;
        // case 1:
        //     std::cout << "1" << std::endl;
        //     break;
        case 2:
            runAddTarget(nh);
            break;
        case 3:
            std::cout << "3" << std::endl;
            break;
        case 10:
            std::cout << "10" << std::endl;
            return false;
        default: 
            std::cout << "Wrong Command Please Try Again" << std::endl;
    }
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_fleet_manager");
    ros::NodeHandle nh;

    loadWaypoint(nh);

    while(ros::ok()) {
        if(!runFleetManagerMenu(nh)) {
            break;
        }
    }
    return 1;
}