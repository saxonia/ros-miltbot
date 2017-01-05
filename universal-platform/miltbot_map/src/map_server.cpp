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
// std_msgs::Int8MultiArray map_data;
// std_msgs::Int8MultiArray map1_data;
// std_msgs::Int8MultiArray map2_data;
nav_msgs::OccupancyGrid map;
nav_msgs::OccupancyGrid map1;
nav_msgs::OccupancyGrid map2;
geometry_msgs::PoseWithCovarianceStamped initial_pose;
int floor_flag = 0;

void loadMap() {

}

bool callService() {
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<nav_msgs::SetMap>("/icreate/set_map");
    nav_msgs::SetMap srv;
    srv.request.map = map;
    //รับ initialpose มาจาก topic 
    srv.request.initial_pose = initial_pose;
    if(client.call(srv)) {
        return srv.response.success;
    }
    else {
        return false;
    }
}

void mapCallback(const nav_msgs::OccupancyGrid &msg) {
    map1 = msg;
    ROS_INFO("Map1: %c",map1.info.width);
}

void mapCallback2(const nav_msgs::OccupancyGrid &msg) {
    map2 = msg;
    ROS_INFO("Map2: %c",map2.info.width);
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
        map = map1;
    }
    else if(building_floor_req == "Floor 17") {
        map = map2;
    }
    building_floor_req = "";
    res.flag = callService();
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_server");
    ros::NodeHandle nh;
    
    ros::Subscriber map_sub = nh.subscribe("/icreate/map", 1000, mapCallback);
    ros::Subscriber map_sub2 = nh.subscribe("/icreate/map2", 1000, mapCallback2);
    ros::Subscriber initialpose_sub = nh.subscribe("/icreate/amcl_pose", 1000, initialposeCallback);
    // ros::Subscriber floor_sub = nh.subscribe("/icreate/building", 1000, floorCallback);
    ros::ServiceServer service = nh.advertiseService("set_map_server", setMapServerService);
    ros::spin();
    return 0;
}