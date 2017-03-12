#include "map.h"

// namespace icreate {

Map::Map() {

}

Map::~Map() {

}

void Map::addMapData(Key id, MapData map_data) {
    // id.first = deleteStringQuote(id.first);
    // id.second = deleteStringQuote(id.second);
    if(data.find(id) != data.end()) {
        std::cout << "Add Map Data: " << map_data.name  << " " << id.first << " " << id.second << std::endl;
        std::vector<MapData> tmp = data[id];
        tmp.push_back(map_data);
        data.at(id) = tmp;
    }
    else {
        std::cout << "Add Map Data2: " << map_data.name << " " << id.first << " " << id.second << std::endl;
        MapData arr[] = {map_data};
        std::vector<MapData> new_data(arr, arr + sizeof(arr) / sizeof(arr[0]) );
        // data.insert(std::make_pair(id, new_data));
        data[id] = new_data;
    }
    
}

Map::MapData Map::getMapData(Key id, int idx) {
    MapData res;
    if(data.find(id) == data.end()) {
        ROS_WARN("Error to get MapData: Wrong id");
    }
    else if(idx >= data[id].size()) {
        ROS_WARN("Error to get MapData: Wrong idx");
    }
    else {
        ROS_INFO("Finish get MapData");
        res = data[id][idx];
    }
    return res;
}

std::vector<miltbot_common::Waypoint> Map::getWaypointList(std::string building, std::string building_floor) {
    std::cout << "Request: " <<  building << ", " << building_floor << std::endl;
    std::vector<MapData> map_data_list;
    std::vector<miltbot_common::Waypoint> res;
    
    Key key = std::make_pair(makeStringQuote(building) , makeStringQuote(building_floor));
    if(data.find(key) == data.end()) {
        ROS_WARN("Error to get MapData: Wrong building or floor");
    }
    else {
        ROS_INFO("Finish get MapData");
        map_data_list = data[key];
    }
    for(int i = 0; i < map_data_list.size(); i++) {
        miltbot_common::Waypoint waypoint;
        waypoint.name = map_data_list[i].name;
        waypoint.building = building;
        waypoint.building_floor = building_floor;
        waypoint.goal = convertPointToMoveBaseGoal(map_data_list[i].point);
        res.push_back(waypoint);
    }
    return res;
}

std::string Map::makeStringQuote(std::string data) {
    return "\"" + data + "\"";
    // return data;
}

std::string Map::deleteStringQuote(std::string data) {
    std::size_t pos = data.find("\"");
    return data.substr(pos+1,data.size()-2);
}

move_base_msgs::MoveBaseGoal Map::convertPointToMoveBaseGoal(std::vector<double> point) {
    move_base_msgs::MoveBaseGoal res;
    if(point.size() != 6) return res;
    res.target_pose.pose.position.x = point[0];
    res.target_pose.pose.position.y = point[1];
    res.target_pose.pose.orientation.x = point[2];
    res.target_pose.pose.orientation.y = point[3];
    res.target_pose.pose.orientation.z = point[4];
    res.target_pose.pose.orientation.w = point[5];
    return res;
}

// }