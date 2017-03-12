#include <ros/ros.h>

#include "nav_msgs/SetMap.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/String.h"

#include "miltbot_map/SetMap.h"
#include "miltbot_map/Waypoint.h"
#include "miltbot_map/GetWaypointList.h"
#include "miltbot_navigation/move_base_data.h"

#include <iostream>

// std::string building;
std::string building_floor_req;
std::vector<miltbot::MoveBaseGoalData> lifts;
nav_msgs::OccupancyGrid *map;
nav_msgs::OccupancyGrid mapb4_f17;
nav_msgs::OccupancyGrid mapb4_20;
nav_msgs::OccupancyGrid map_dynamic;
std::vector<nav_msgs::OccupancyGrid> maps;
geometry_msgs::PoseWithCovarianceStamped initial_pose;

ros::ServiceClient set_map_client;
ros::ServiceClient get_waypoint_list_client;

int target_number = 0;
int floor_flag = 0;
bool pub_flag = false;
bool get_map1 = true;
bool get_map2 = true;
bool get_map3 = true;

void mapCallback(const nav_msgs::OccupancyGrid &msg) {
    if(get_map1) {
        mapb4_f17 = msg;
        ROS_INFO("Map1: %d",mapb4_f17.info.width);
        pub_flag = true;
        get_map1 = false;
    }
}

void mapCallback2(const nav_msgs::OccupancyGrid &msg) {
    if(get_map2) {
        mapb4_20 = msg;
        ROS_INFO("Map2: %d",mapb4_20.info.width);
        pub_flag = true;
        get_map2 = false;
    }
}

void mapDynamicCallback(const nav_msgs::OccupancyGrid &msg) {
    // if(get_map3) {
        map_dynamic = msg;
        ROS_INFO("Map Dynamic: %d",map_dynamic.info.width);
        pub_flag = true;
        // get_map3 = false;
    // }
}

void initialposeCallback(const geometry_msgs::PoseWithCovarianceStamped &msg) {
    initial_pose = msg;
    ROS_INFO("Initial Pose: %s",initial_pose.header.frame_id.c_str());
}

// void floorCallback(const std_msgs::String::ConstPtr &msg) {
//     std::string floor_ = msg->data;
//     if(floor_ == "Floor 20") {
//         floor_flag = 1;
//     }
//     else if(floor_ == "Floor 17") {
//         floor_flag = 2;
//     }
// }

void setWaypoint(std::vector<miltbot_map::Waypoint> waypoints) {
    lifts.clear();
    ROS_INFO("lifts size: %ld", lifts.size());
    for(int i = 0; i < waypoints.size(); i++) {
           miltbot::MoveBaseGoalData data;
           data.setGoalName(waypoints[i].name);
           data.setBuilding(waypoints[i].building);
           data.setBuildingFloor(waypoints[i].floor);
           data.setGoal(waypoints[i].goal);
           lifts.push_back(data);
    } 
}

bool callGetWaypointClientService() {
    miltbot_map::GetWaypointList srv;
    srv.request.building = "Building 4";
    srv.request.floor = building_floor_req;
    std::vector<miltbot_map::Waypoint> waypoints;
    if(get_waypoint_list_client.call(srv)) {
        waypoints = srv.response.waypoints;
        if(waypoints.size() > 0) {
            setWaypoint(waypoints);
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

bool callSetMapClientService() {
    nav_msgs::SetMap set_map_srv;
    set_map_srv.request.map = *map;
    std_msgs::Header header;
    header.frame_id = "map";
    if(!callGetWaypointClientService())
        return false;
    move_base_msgs::MoveBaseGoal goal = lifts[target_number].getGoal();
    geometry_msgs::PoseWithCovariance pose;
    ROS_WARN("%lf", goal.target_pose.pose.position.x);
    pose.pose.position.x = goal.target_pose.pose.position.x;
    pose.pose.position.y = goal.target_pose.pose.position.y;
    pose.pose.orientation.z = goal.target_pose.pose.orientation.z;
    pose.pose.orientation.w = goal.target_pose.pose.orientation.w;
    pose.covariance = {0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942};
    set_map_srv.request.initial_pose.header = header;
    set_map_srv.request.initial_pose.pose = pose; 
    if(set_map_client.call(set_map_srv)) {
        return set_map_srv.response.success;   
    }
    else {
        ROS_ERROR("Failed to call service set_map");
        return false;
    }
}

bool setMapService(miltbot_map::SetMap::Request &req, miltbot_map::SetMap::Response &res) {
    building_floor_req = req.floor;
    target_number = req.target_number;
    if(building_floor_req == "Floor 20" || building_floor_req == "Floor 20 Lift") {
        map = &mapb4_20;
        callSetMapClientService();
    }
    else if(building_floor_req == "Floor 17" || building_floor_req == "Floor 17 Lift") {
        map = &mapb4_f17;
        callSetMapClientService();
    }
    else if(building_floor_req == "Lift") {
        map = &map_dynamic;
    }
    else {
        res.flag = false;
        return false;
    }
    pub_flag = true;
    building_floor_req = "";
    res.flag = true;
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "maps_server");
    ros::NodeHandle nh;

    std::string map1_sub_topic_name("/build4_f17/map");
    std::string map2_sub_topic_name("/build4_f20/map");
    std::string map_dynamic_sub_topic_name("map_dynamic");
    std::string set_map_server_service_name("set_map_service");
    std::string set_map_client_service_name("set_map");
    std::string map_pub_topic_name("map");
    std::string get_waypoint_list_client_service_name("get_waypoint_list");
    // std::string initialpose_sub_topic_name();

    nh.param("map1_sub_topic", map1_sub_topic_name, map1_sub_topic_name);
    nh.param("map2_sub_topic", map2_sub_topic_name, map2_sub_topic_name);
    nh.param("map_dynamic_sub_topic", map_dynamic_sub_topic_name, map_dynamic_sub_topic_name);
    nh.param("set_map_server_service", set_map_server_service_name, set_map_server_service_name);
    nh.param("set_map_client_service", set_map_client_service_name, set_map_client_service_name);
    nh.param("get_waypoint_list_client_service", get_waypoint_list_client_service_name, get_waypoint_list_client_service_name);
    nh.param("map_pub_topic", map_pub_topic_name, map_pub_topic_name);
    
    ros::Subscriber map_sub = nh.subscribe(map1_sub_topic_name, 1, mapCallback);
    ros::Subscriber map_sub2 = nh.subscribe(map2_sub_topic_name, 1, mapCallback2);
    ros::Subscriber map_dynamic_sub = nh.subscribe(map_dynamic_sub_topic_name, 1, mapDynamicCallback);
    // ros::Subscriber floor_sub = nh.subscribe("/icreate/building", 1000, floorCallback);
    ros::ServiceServer service = nh.advertiseService(set_map_server_service_name, setMapService);
    set_map_client = nh.serviceClient<nav_msgs::SetMap>(set_map_client_service_name);
    get_waypoint_list_client = nh.serviceClient<miltbot_map::GetWaypointList>(get_waypoint_list_client_service_name);
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>(map_pub_topic_name, 1, true);
    ros::Rate r(10);
    map = &mapb4_20;
    get_map1 = true;
    get_map2 = true;
    while(ros::ok()) {
        ros::spinOnce();
        if(pub_flag) {
            map_pub.publish(*map);
            pub_flag = false;
        }
        r.sleep();
    }
    return 0;
}