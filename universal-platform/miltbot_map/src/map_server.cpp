#include <ros/ros.h>
#include <ros/package.h>

#include <fstream>
#include <map>

#include "map.h"
#include <miltbot_map/GetWaypointName.h>

Map map;

int toint(std::string s) {
    return atoi(s.c_str());
}

void loadWaypointFile(Map &map, std::string filename) {
    std::string path = ros::package::getPath("miltbot_map") + "/waypoint/" + filename;
    std::cout << "Show path: " << path << std::endl;
    std::ifstream inFile(path.c_str());
    std::string line; 

    std::cout << "Importing waypoint file" << std::endl;
    //First Value is Waypoint Counts 
    getline(inFile, line);
    int waypoint_count = toint(line);
    std::cout << "Waypoint file Counted : " << waypoint_count <<std::endl;
    //Second & Third are Waypoint building & floor 
    getline(inFile, line);
    std::stringstream strstr(line);
    std::string building_name;
    getline(strstr, building_name, ',');
    std::string building_floor;
    getline(strstr, building_floor, ',');
    building_floor = building_floor.substr(1,building_floor.size()-1);

    std::vector<std::string> target_name;
    std::vector<std::string> tokenized;
    //Read POI Line By Line 
    while(getline(inFile,line)){
     // New Line
       std::stringstream strstr(line);
       std::string word = "";
     // Ignore First Param (Place's Name)
       getline(strstr,word,',');
       target_name.push_back(word);
     // Gather Params
       while(getline(strstr,word,',')){
           tokenized.push_back(word);
       }
    }

    //Count All Parameters
    size_t counter = tokenized.size();
    std::cout << "[POINT_READER]Counter  : " << counter <<std::endl; 
    size_t point_amount = (size_t)(counter / 6.0);
    std::cout << "[POINT_READER]Div Count : " << point_amount << " ,  File Count = " << waypoint_count <<std::endl;

    std::vector<std::string>::iterator word_it;
    word_it = tokenized.begin();

    // Create move_base GOAL and Push into vector ! 
    for(int point_index = 0  ; point_index < point_amount ; point_index++){
        std::vector<double> point;
        for(int i = 0; i < 6; i++) {
            point.push_back(std::atof((word_it++)->c_str()));
        }
        Map::MapData data(target_name[point_index],point);
        Key key = std::make_pair(building_name, building_floor); 
        std::cout << building_name << " " << building_floor << point_amount << std::endl; 
        map.addMapData(key, data);
        std::cout << map.data[key].size() << std::endl;
        if(word_it == tokenized.end())break;
    }
    // for(std::map<Key,std::vector<Map::MapData> >::iterator it = map.data.begin(); it != map.data.end(); ++it) {
    //     std::cout << "Let : " << it->first.first << " " << it->first.second << "\n";
    //     if(map.data[std::make_pair("\"Building 3\"","\"Floor 20\"")].size() > 0)
    //         std::cout << "ZZZ : " << map.data[std::make_pair("\"Building 3\"","\"Floor 20\"")][0].name << std::endl;
    // }
    // Key key = std::make_pair("\"Building 3\"","\"Floor 20\""); 
    // std::cout << map.data[std::make_pair("\"Building 3\"","\"Floor 20\"")].size() << std::endl;
    // std::cout << map.data[key].size() << std::endl;
    // if(map.data[std::make_pair("\"Building 3\"","\"Floor 20\"")].size() > 0)
    if(map.data.at(std::make_pair("\"Building 3\"","\"Floor 20\"")).size() > 0)
    {
        // std::cout << "ZZZ : " << map.data[std::make_pair("\"Building 3\"","\"Floor 20\"")][0].name << std::endl;
    }


}

std::vector<std::string> loadWaypointFileList(std::string filename) {
    std::string path = ros::package::getPath("miltbot_map") + "/waypoint/" + filename;
    std::ifstream inFile(path.c_str());
    std::string line; 
    std::vector<std::string> waypoint_list;

    std::cout << "Importing waypoint file lists" << std::endl;

    getline(inFile, line);
    int waypoint_list_count = toint(line);
    std::cout << "Waypoint file Counted : " << waypoint_list_count <<std::endl;

    while(getline(inFile, line)) {
        waypoint_list.push_back(line);
    }

    //Count All Parameters
    size_t counter = waypoint_list.size();
    std::cout << "Counter : " << counter << std::endl;
    
    return waypoint_list;
} 

void loadWaypoint(Map &map) {
    std::vector<std::string> waypoint_list = loadWaypointFileList("waypoint_list.csv");
    for(int i = 0; i < waypoint_list.size(); i++) {
        std::cout << "Show : " << waypoint_list[i] << std::endl;
        // [FUTURE] do delete string quote before  send to load_waypoint_file
        loadWaypointFile(map, waypoint_list[i]);   
    } 
} 

bool getWaypointNameService(miltbot_map::GetWaypointName::Request &req, 
                            miltbot_map::GetWaypointName::Response &res) {
    std::pair<std::vector<std::string>, std::vector<std::string> > ret = map.getWaypointNameList(req.building);
    res.data = ret.first;
    res.floor = ret.second; 
    return true;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "map_server");
    ros::NodeHandle nh;

    loadWaypoint(map);
    if(map.data[std::make_pair("\"Building 3\"","\"Floor 20\"")].size() > 0)
        std::cout << "ZZZ : " << map.data[std::make_pair("\"Building 3\"","\"Floor 20\"")][0].name << std::endl;
    ros::ServiceServer service = nh.advertiseService("get_waypoint_name", getWaypointNameService);
    ros::spin();

    return 0;
}