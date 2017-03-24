#include <ros/ros.h>
#include <iostream>
#include <map>
#include <stdlib.h>

#include "miltbot_common/Waypoint.h"
#include "miltbot_common/WaypointList.h"
#include "miltbot_map/GetWaypointList.h"
#include "miltbot_system/ViewTargetQueue.h"
#include "miltbot_system/GetRobotList.h"
#include "miltbot_system/AddTarget.h"
#include "miltbot_system/DeleteTarget.h"
#include "miltbot_system/RunSystem.h"

typedef std::pair<std::string, std::string> Key;

std::map<Key, std::vector<miltbot_common::Waypoint> > targets;
std::vector<miltbot_common::Waypoint> target_queue;

std::string view_target_queue_service_name("view_target_queue");
std::string get_waypoint_list_service_name("get_waypoint_list");
std::string add_target_service_name("add_target");
std::string delete_target_service_name("delete_target");
std::string add_default_target_service_name("add_default_target");
std::string run_system_service_name("run_system");

long generateTargetId() {
    return rand();
}

bool callGetWaypointService(ros::NodeHandle &nh, std::string building, std::string building_floor) {
    ros::ServiceClient client = nh.serviceClient<miltbot_map::GetWaypointList>("get_waypoint_list");
    miltbot_map::GetWaypointList srv;
    srv.request.building = building;
    srv.request.floor = building_floor;
    std::vector<miltbot_common::Waypoint> waypoints;
    if(client.call(srv)) {
        waypoints = srv.response.waypoints;
        if(waypoints.size() > 0) {
            // addTargetData(building, building_floor, waypoints);
            Key key = std::make_pair(building, building_floor);
            targets[key] = waypoints;
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

void callRunSystemService(ros::NodeHandle &nh, bool status, std::string service_namespace) {
    ros::ServiceClient client = nh.serviceClient<miltbot_system::RunSystem>(service_namespace + run_system_service_name);
    miltbot_system::RunSystem srv;
    srv.request.status = status;
    if(client.call(srv)) {
        bool flag = srv.response.success;
    }
    else {
        ROS_ERROR("Failed to call service run_system");
    }
}

void addRobot(ros::NodeHandle &nh) {
}

void callAddTargetService(ros::NodeHandle &nh, miltbot_common::Waypoint data, std::string service_namespace) {
    ROS_ERROR("%s",(service_namespace + add_target_service_name).c_str());
    ros::ServiceClient client = nh.serviceClient<miltbot_system::AddTarget>(service_namespace + add_target_service_name);
    miltbot_system::AddTarget srv;
    long id = generateTargetId();
    srv.request.waypoint = data;
    srv.request.waypoint.id = id;
    if(client.call(srv)) {
        bool flag = srv.response.success;
    }
    else {
        ROS_ERROR("Failed to call service add_target");
    }
}

void callDeleteTargetService(ros::NodeHandle &nh, long id, std::string service_namespace) {
    ROS_ERROR("ID: %ld",id);
    ros::ServiceClient client = nh.serviceClient<miltbot_system::DeleteTarget>(service_namespace + delete_target_service_name);
    miltbot_system::DeleteTarget srv;
    srv.request.id = id;
    if(client.call(srv)) {
        bool flag = srv.response.success;
    }
    else {
        ROS_ERROR("Failed to call service delete_target");
    }
}

void callAddDefaultTargetService(ros::NodeHandle &nh, miltbot_common::Waypoint data, std::string service_namespace) {
    ROS_WARN("%s",(service_namespace + add_default_target_service_name).c_str());
    ros::ServiceClient client = nh.serviceClient<miltbot_system::AddTarget>(service_namespace + add_default_target_service_name);
    miltbot_system::AddTarget srv;
    long id = generateTargetId();
    srv.request.waypoint = data;
    srv.request.waypoint.id = id;
    if(client.call(srv)) {
        bool flag = srv.response.success;
    }
    else {
        ROS_ERROR("Failed to call service add_default_target");
    }
}

void callViewTargetQueue(ros::NodeHandle &nh, std::string service_namespace) {
    ros::ServiceClient client = nh.serviceClient<miltbot_system::ViewTargetQueue>(service_namespace + view_target_queue_service_name);
    miltbot_system::ViewTargetQueue srv;
    if(client.call(srv)) {
        target_queue = srv.response.target_queue;
    }
    else {
        ROS_ERROR("Failed to call service view_target_queue");
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

void setTargetPriority(miltbot_common::Waypoint &data) {
    std::cout << "Please insert target priority" << std::endl;
    std::cout << "Your Input: ";
    int priority;
    std::cin >> priority;
    data.priority = priority;
}

void displayWaypoints(std::vector<miltbot_common::Waypoint> data) {
    std::cout << "Please Select Waypoint" << std::endl;
    for(int i = 0 ; i < data.size() ; i++) {
      std::cout <<"["<<i<<"] " << data[i].name <<std::endl; 
    }
    std::cout <<"["<<99<<"] " << "Cancel" << " " <<std::endl; 
}

void displayBuildingFloor() {
    int i = 0;
    std::cout << "Please Select Floor" << std::endl;
    for(std::map<Key, std::vector<miltbot_common::Waypoint> >::iterator it = targets.begin(); it != targets.end(); it++, i++) {
        std::cout <<"["<<i<<"] " << (it->first).second <<std::endl; 
    }
    std::cout <<"["<<99<<"] " << "Cancel" <<std::endl;
}

void displayRobot() {
    int i = 0;
    std::cout << "Please Select Robot" << std::endl;
    std::cout <<"["<<i<<"] " << "bot1" <<std::endl; 
    // for(std::map<Key, std::vector<miltbot_common::Waypoint> >::iterator it = targets.begin(); it != targets.end(); it++, i++) {
        // std::cout <<"["<<i<<"] " << (it->first).second <<std::endl; 
    // }
    std::cout <<"["<<99<<"] " << "Cancel" <<std::endl;
}

void displayTargetQueue() {
    for(int i = 0; i < target_queue.size(); i++) {
        std::cout <<"["<<i<<"] " << target_queue[i].name <<std::endl; 
    }
}

bool showWaypointMenu(miltbot_common::Waypoint &target) {
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
    std::map<Key, std::vector<miltbot_common::Waypoint> >::iterator it = targets.begin();
    std::advance(it, selected_floor);
    std::vector<miltbot_common::Waypoint> data = it->second;
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

bool showRobotMenu(std::string &service_namespace) {
    int selected_robot;
    while(ros::ok()) {
        displayRobot();
        std::cout << "[AGENT] Input Robot : ";
        std::cin >> selected_robot;
        std::cout << std::endl;
        if((selected_robot < 0 || selected_robot >= 1) && selected_robot != 99) {
            ROS_WARN("Wrong Select Point To Navigate Try Again");
            continue;
        }
        else if(selected_robot == 99) {
            return false;
        }
        else {
            service_namespace = "bot1/";
            return true;
        }
    }
}

bool showQueueMenu(ros::NodeHandle &nh, std::string service_namespace, long &id) {
    int selected_target;
    while(ros::ok()) {
        callViewTargetQueue(nh, service_namespace);
        std::cout << "Please Select Target" << std::endl;
        displayTargetQueue();
        std::cout <<"["<<99<<"] " << "Cancel" <<std::endl;
        std::cout << "[AGENT] Input Target : ";
        std::cin >> selected_target;
        if((selected_target < 0 || selected_target >= target_queue.size()) && selected_target != 99) {
            ROS_WARN("Wrong Select Point To Navigate Try Again");
            continue;
        }
        else if(selected_target == 99) {
            return false;
        }
        else {
            id = target_queue[selected_target].id;
            return true;
        }
    }
}

void runViewQueue(ros::NodeHandle &nh) {
    std::string service_namespace;
    if(!showRobotMenu(service_namespace)) return;
    callViewTargetQueue(nh, service_namespace);
    std::cout << "Current Target Queue" << std::endl;
    displayTargetQueue();
    std::cout << std::endl;
    std::cout << std::endl;
}

void runAddTarget(ros::NodeHandle &nh) {
    std::string service_namespace;
    if(!showRobotMenu(service_namespace)) return;
    miltbot_common::Waypoint data;
    if(!showWaypointMenu(data)) return;
    data.task = "SINGLERUN";
    callAddTargetService(nh, data, service_namespace);
}

void runDeleteTarget(ros::NodeHandle &nh) {
    std::string service_namespace;
    if(!showRobotMenu(service_namespace)) return;
    long id;
    if(!showQueueMenu(nh, service_namespace, id)) return;
    callDeleteTargetService(nh, id, service_namespace);
}

void runReceiveSupplies(ros::NodeHandle &nh) {
    std::string service_namespace;
    if(!showRobotMenu(service_namespace)) return;
    miltbot_common::Waypoint data;
    if(!showWaypointMenu(data)) return;
    data.task = "GOING";
    callAddTargetService(nh, data, service_namespace);
}

void runSendSupplies(ros::NodeHandle &nh) {
    std::string service_namespace;
    if(!showRobotMenu(service_namespace)) return;
    miltbot_common::Waypoint data;
    if(!showWaypointMenu(data)) return;
    data.task = "SENDSUPPLIES";
    callAddTargetService(nh, data, service_namespace);
}

void runAddDefaultTarget(ros::NodeHandle &nh) {
    std::string service_namespace;
    if(!showRobotMenu(service_namespace)) return;
    miltbot_common::Waypoint data;
    showWaypointMenu(data);
    data.task = "SINGLERUN";
    callAddDefaultTargetService(nh,data, service_namespace);
}

void runStartSystem(ros::NodeHandle &nh) {
    std::string service_namespace;
    if(!showRobotMenu(service_namespace)) return;
    callRunSystemService(nh, true, service_namespace);
}

void runShutdownSystem(ros::NodeHandle &nh) {
    std::string service_namespace;
    if(!showRobotMenu(service_namespace)) return;
    callRunSystemService(nh, false, service_namespace);
}

void showFleetManagerMenu() {
    std::cout << "Start Simple Fleet Manager System" << std::endl;
    std::cout << "Please Select Your Command" << std::endl;
    std::cout << "[0] View Current Queue of Robot" << std::endl;
    // std::cout << "[1] Add Robot To System" << std::endl;
    // std::cout << "[2] Delete Robot From System" << std::endl;
    std::cout << "[3] Add Target To Robot" << std::endl;
    std::cout << "[4] Delete Target From Robot" << std::endl;
    std::cout << "[5] Send Robot To Receive Supplies" << std::endl;
    std::cout << "[6] Send Robot To Send Supplies" << std::endl;
    std::cout << "[7] Add Default Target To Robot" << std::endl;
    std::cout << "[10] Exit Program" << std::endl;
    std::cout << "[98] Start System" << std::endl;
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
        case 0:
            runViewQueue(nh);
            break;
        // case 1:
        //     std::cout << "0" << std::endl;
        //     addRobot(nh);
        //     break;
        // case 2:
        //     std::cout << "1" << std::endl;
        //     break;
        case 3:
            runAddTarget(nh);
            break;
        case 4:
            runDeleteTarget(nh);
            break;
        case 5:
            runReceiveSupplies(nh);
            break;
        case 6:
            runSendSupplies(nh);
            break;
        case 7:
            runAddDefaultTarget(nh);
            break;
        case 10:
            std::cout << "10" << std::endl;
            return false;
        case 98:
            runStartSystem(nh);
            break;
        case 99:
            runShutdownSystem(nh);
            break;
        default: 
            std::cout << "Wrong Command Please Try Again" << std::endl;
    }
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_fleet_manager");
    ros::NodeHandle nh;

    nh.param("get_waypoint_list_service", get_waypoint_list_service_name, get_waypoint_list_service_name);
    nh.param("add_target_service", add_target_service_name, add_target_service_name);
    nh.param("delete_target_service", delete_target_service_name, delete_target_service_name);
    nh.param("add_default_target_service", add_default_target_service_name, add_default_target_service_name);
    nh.param("run_system_service", run_system_service_name, run_system_service_name);

    loadWaypoint(nh);

    while(ros::ok()) {
        if(!runFleetManagerMenu(nh)) {
            break;
        }
    }
    ROS_INFO("Exit System");
    return 0;
}