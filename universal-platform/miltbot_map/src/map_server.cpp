#include <ros/ros.h>

#include "nav_msgs/SetMap.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/String.h"

#include "miltbot_map/SetMapServer.h"

#include <iostream>

// std::string building;
std::string building_floor_req;
nav_msgs::OccupancyGrid *map;
nav_msgs::OccupancyGrid map1;
nav_msgs::OccupancyGrid map2;
nav_msgs::OccupancyGrid map_dynamic;
std::vector<nav_msgs::OccupancyGrid> maps;
geometry_msgs::PoseWithCovarianceStamped initial_pose;
int floor_flag = 0;
bool pub_flag = false;

void mapCallback(const nav_msgs::OccupancyGrid &msg) {
    map1 = msg;
    ROS_INFO("Map1: %d",map1.info.width);
    pub_flag = true;
}

void mapCallback2(const nav_msgs::OccupancyGrid &msg) {
    map2 = msg;
    ROS_INFO("Map2: %d",map2.info.width);
    pub_flag = true;
}

void mapDynamicCallback(const nav_msgs::OccupancyGrid &msg) {
    map_dynamic = msg;
    ROS_INFO("Map Dynamic: %d",map_dynamic.info.width);
    pub_flag = true;
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

bool setMapServerService(miltbot_map::SetMapServer::Request &req,miltbot_map::SetMapServer::Response &res) {
    building_floor_req = req.floor;
    if(building_floor_req == "Floor 20") {
        map = &map2;
    }
    else if(building_floor_req == "Floor 17") {
        map = &map1;
    }
    else if(building_floor_req == "Lift") {
        map = &map_dynamic;
    }
    pub_flag = true;
    building_floor_req = "";
    res.flag = true;
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_server");
    ros::NodeHandle nh;

    std::string map1_sub_topic_name("/build4_f17/map");
    std::string map2_sub_topic_name("/build4_f20/map");
    std::string map_dynamic_sub_topic_name("/map_dynamic");
    std::string set_map_service_name("set_map");
    std::string map_pub_topic_name("map");
    // std::string initialpose_sub_topic_name();

    nh.param("map1_sub_topic", map1_sub_topic_name, map1_sub_topic_name);
    nh.param("map2_sub_topic", map2_sub_topic_name, map2_sub_topic_name);
    nh.param("map_dynamic_sub_topic", map_dynamic_sub_topic_name, map_dynamic_sub_topic_name);
    nh.param("set_map_service", set_map_service_name, set_map_service_name);
    nh.param("map_pub_topic", map_pub_topic_name, map_pub_topic_name);
    
    ros::Subscriber map_sub = nh.subscribe(map1_sub_topic_name, 10, mapCallback);
    ros::Subscriber map_sub2 = nh.subscribe(map2_sub_topic_name, 10, mapCallback2);
    ros::Subscriber map_dynamic_sub = nh.subscribe(map_dynamic_sub_topic_name, 10, mapDynamicCallback);
    // ros::Subscriber initialpose_sub = nh.subscribe("/icreate/amcl_pose", 1000, initialposeCallback);
    // ros::Subscriber floor_sub = nh.subscribe("/icreate/building", 1000, floorCallback);
    ros::ServiceServer service = nh.advertiseService(set_map_service_name, setMapServerService);
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>(map_pub_topic_name, 1000);
    // ros::Rate r(30);
    map = &map2;
    while(ros::ok()) {
        ros::spinOnce();
        if(pub_flag) {
            map_pub.publish(*map);
            pub_flag = false;
        }
    }
    return 0;
}