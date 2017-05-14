#include <ros/ros.h>

#include "nav_msgs/SetMap.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/String.h"

#include "miltbot_map/SetMap.h"
#include "miltbot_common/Waypoint.h"
#include "miltbot_map/GetWaypointList.h"

#include <iostream>

std::string frame_id;
std::string building;
std::string building_floor_req;
std::vector<miltbot_common::Waypoint> lifts;
nav_msgs::OccupancyGrid *map;
nav_msgs::OccupancyGrid mapb4_f17;
nav_msgs::OccupancyGrid mapb4_f19;
nav_msgs::OccupancyGrid mapb4_f20;
nav_msgs::OccupancyGrid map_dynamic;
nav_msgs::OccupancyGrid premap;
geometry_msgs::PoseWithCovarianceStamped initial_pose;

ros::ServiceClient set_map_client;
ros::ServiceClient get_waypoint_list_client;

int target_number = 0;
int floor_flag = 0;
bool pub_flag = false;
// bool pub_flag1 = false;
// bool pub_flag2 = false;
// bool pub_flag3 = false;
// bool pub_flag4 = false;
bool pub_flag5 = false;
// bool get_map1 = true;
// bool get_map2 = true;
// bool get_map3 = true;
// bool get_map4 = true;
bool get_map5 = true;

void mapCallback(const nav_msgs::OccupancyGrid &msg) {
    // if(get_map1) {
        mapb4_f17 = msg;
        mapb4_f17.header.frame_id = frame_id;
        ROS_INFO("Map1: %d %s",mapb4_f17.info.width, frame_id.c_str());
        // pub_flag = true;
    // }
}

void mapCallback2(const nav_msgs::OccupancyGrid &msg) {
    // if(get_map2) {
        mapb4_f19 = msg;
        mapb4_f19.header.frame_id = frame_id;
        ROS_INFO("Map2: %d %s",mapb4_f19.info.width, frame_id.c_str());
        // pub_flag = true;
    // }
}

void mapCallback3(const nav_msgs::OccupancyGrid &msg) {
    // if(get_map2) {
        mapb4_f20 = msg;
        mapb4_f20.header.frame_id = frame_id;
        ROS_INFO("Map3: %d %s",mapb4_f20.info.width, frame_id.c_str());
        // pub_flag = true;
    // }
}

void mapDynamicCallback(const nav_msgs::OccupancyGrid &msg) {
    map_dynamic = msg;
    map_dynamic.header.frame_id = frame_id;
    ROS_INFO("Map Dynamic: %d",map_dynamic.info.width);
    // pub_flag4 = true;
    pub_flag = true;
}

void premapCallback(const nav_msgs::OccupancyGrid &msg) {
    if(get_map5) {
        premap = msg;
        premap.header.frame_id = frame_id;
        ROS_INFO("Pre Map : %d %s",premap.info.width, frame_id.c_str());
        pub_flag5 = true;
        get_map5 = false;
        pub_flag = true;
    }
}

void initialposeCallback(const geometry_msgs::PoseWithCovarianceStamped &msg) {
    initial_pose = msg;
    ROS_INFO("Initial Pose: %s",initial_pose.header.frame_id.c_str());
}

bool callGetWaypointClientService() {
    ROS_INFO("Get Waypoint");
    miltbot_map::GetWaypointList srv;
    srv.request.building = building;
    srv.request.floor = building_floor_req;
    std::vector<miltbot_common::Waypoint> waypoints;
    if(get_waypoint_list_client.call(srv)) {
        waypoints = srv.response.waypoints;
        if(waypoints.size() > 0) {
            lifts = waypoints;
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
    ROS_INFO("Set Map Client");
    nav_msgs::SetMap set_map_srv;
    set_map_srv.request.map = *map;
    std_msgs::Header header;
    header.frame_id = frame_id;
    if(!callGetWaypointClientService())
        return false;
    move_base_msgs::MoveBaseGoal goal = lifts[target_number].goal;
    geometry_msgs::PoseWithCovariance pose;
    pose.pose.position.x = goal.target_pose.pose.position.x;
    pose.pose.position.y = goal.target_pose.pose.position.y;
    pose.pose.orientation.z = goal.target_pose.pose.orientation.z;
    pose.pose.orientation.w = goal.target_pose.pose.orientation.w;
    pose.covariance = {0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942};
    set_map_srv.request.initial_pose.header = header;
    set_map_srv.request.initial_pose.pose = pose; 
    ROS_WARN("%lf", goal.target_pose.pose.position.x);
    if(set_map_client.call(set_map_srv)) {
        ROS_INFO("Set Default Map");
        return set_map_srv.response.success;   
    }
    else {
        ROS_ERROR("Failed to call service set_map");
        return false;
    }
}

bool setMapService(miltbot_map::SetMap::Request &req, miltbot_map::SetMap::Response &res) {
    ROS_INFO("Set Map");
    building_floor_req = req.floor;
    target_number = req.target_number;
    if(building_floor_req == "Floor 20" || building_floor_req == "Floor 20 Lift") {
        map = &mapb4_f20;
        if(!callSetMapClientService()) return false;
    }
    else if(building_floor_req == "Floor 19" || building_floor_req == "Floor 19 Lift") {
        map = &mapb4_f19;
        if(!callSetMapClientService()) {
            ROS_INFO("Good 19");
            return false;
        } 
    }
    else if(building_floor_req == "Floor 17" || building_floor_req == "Floor 17 Lift") {
        map = &mapb4_f17;
        if(!callSetMapClientService()) {
            ROS_INFO("Good 17");
            return false;
        } 
    }
    else if(building_floor_req == "Lift") {
        map = &map_dynamic;
    }
    else {
        res.flag = false;
        return false;
    }
    pub_flag = true;
    ROS_INFO("Out");
    building_floor_req = "";
    res.flag = true;
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "maps_server");
    ros::NodeHandle nh;

    std::string map_list("map_list.csv");
    std::string map1_sub_topic_name("/build4_f17/map");
    std::string map2_sub_topic_name("/build4_f19/map");
    std::string map3_sub_topic_name("/build4_f20/map");
    std::string map_dynamic_sub_topic_name("map_dynamic");
    std::string premap_sub_topic_name("premap");
    std::string set_map_server_service_name("set_map_service");
    std::string set_map_client_service_name("set_map");
    std::string map_pub_topic_name("map");
    std::string get_waypoint_list_client_service_name("/get_waypoint_list");
    // std::string initialpose_sub_topic_name();

    building = "Building 4";
    frame_id = "map";

    nh.param("map1_sub_topic", map1_sub_topic_name, map1_sub_topic_name);
    nh.param("map2_sub_topic", map2_sub_topic_name, map2_sub_topic_name);
    nh.param("map3_sub_topic", map3_sub_topic_name, map3_sub_topic_name);
    nh.param("map_dynamic_sub_topic", map_dynamic_sub_topic_name, map_dynamic_sub_topic_name);
    nh.param("premap_sub_topic", premap_sub_topic_name, premap_sub_topic_name);
    nh.param("set_map_server_service", set_map_server_service_name, set_map_server_service_name);
    nh.param("set_map_client_service", set_map_client_service_name, set_map_client_service_name);
    nh.param("get_waypoint_list_client_service", get_waypoint_list_client_service_name, get_waypoint_list_client_service_name);
    nh.param("map_pub_topic", map_pub_topic_name, map_pub_topic_name);
    nh.param("maps_server/map_frame_id", frame_id, frame_id);

    ROS_INFO("%s",set_map_server_service_name.c_str());
    ROS_INFO("%s",set_map_client_service_name.c_str());
    ROS_INFO("%s",get_waypoint_list_client_service_name.c_str());
    ROS_INFO("%s",map_pub_topic_name.c_str());
    ROS_INFO("%s",frame_id.c_str());
    
    ros::Subscriber map_sub = nh.subscribe(map1_sub_topic_name, 1, mapCallback);
    ros::Subscriber map_sub2 = nh.subscribe(map2_sub_topic_name, 1, mapCallback2);
    ros::Subscriber map_sub3 = nh.subscribe(map3_sub_topic_name, 1, mapCallback3);
    ros::Subscriber map_dynamic_sub = nh.subscribe(map_dynamic_sub_topic_name, 1, mapDynamicCallback);
    ros::Subscriber premap_sub = nh.subscribe(premap_sub_topic_name, 1, premapCallback);
    ros::ServiceServer service = nh.advertiseService(set_map_server_service_name, setMapService);
    set_map_client = nh.serviceClient<nav_msgs::SetMap>(set_map_client_service_name);
    get_waypoint_list_client = nh.serviceClient<miltbot_map::GetWaypointList>(get_waypoint_list_client_service_name);
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>(map_pub_topic_name, 1, true);
    ros::Rate r(10);
    map = &premap;
    // get_map1 = true;
    // get_map2 = true;
    // get_map4 = true;
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