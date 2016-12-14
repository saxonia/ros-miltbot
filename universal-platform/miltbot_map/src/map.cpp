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
        // if(data[std::make_pair("\"Building 3\"","\"Floor 20\"")].size() > 0)
            // std::cout << "ZZZ : " << data[std::make_pair("\"Building 3\"","\"Floor 20\"")][1].name << std::endl;
        // std::cout << "Key : " << id.first << id.second << std::endl;
        std::vector<MapData> tmp = data[id];
        // std::cout << tmp.size() << std::endl;
        tmp.push_back(map_data);
        // std::cout << tmp.size() << std::endl;
        // data.insert(std::make_pair(id, tmp));
        data.at(id) = tmp;
    }
    else {
        std::cout << "Add Map Data2: " << map_data.name << " " << id.first << " " << id.second << std::endl;
        // if(data[std::make_pair("\"Building 3\"","\"Floor 20\"")].size() > 0)
            // std::cout << "ZZZ : " << data[std::make_pair("\"Building 3\"","\"Floor 20\"")][1].name << std::endl;
        static const MapData arr[] = {map_data};
        std::vector<MapData> new_data(arr, arr + sizeof(arr) / sizeof(arr[0]) );
        data.insert(std::make_pair(id, new_data));
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

std::pair<std::vector<std::string>, std::vector<std::string> > Map::getWaypointNameList(std::string building) {
    std::cout << building << std::endl;
    std::vector<MapData> map_data_list;
    std::vector<std::string> name_list;
    std::vector<std::string> floor_list;
    for(std::map<Key,std::vector<MapData> >::iterator it = data.begin(); it != data.end(); ++it) {
        if(it->first.first == makeStringQuote(building)) {
            Key key = std::make_pair(makeStringQuote(building) , it->first.second);
            //Key key = std::make_pair(building, f);
            if(data.find(key) == data.end()) {
                ROS_WARN("Error to get MapData: Wrong building or floor");
            }
            else {
                ROS_INFO("Finish get MapData");
                std::cout << data[key].size() << std::endl;
                map_data_list = data[key];
            }
            for(int i = 0; i < map_data_list.size(); i++) {
                name_list.push_back(map_data_list[i].name);
                floor_list.push_back(it->first.second);
            }
        }
    }
    std::pair<std::vector<std::string>, std::vector<std::string> > res =  std::make_pair(name_list, floor_list);
    
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

// }