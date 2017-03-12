#include <ros/ros.h>
#include <map>
#include <vector>

#include "miltbot_common/Waypoint.h"

// namespace icreate {
    typedef std::pair<std::string, std::string> Key;

class Map {
    public:
        Map();

        ~Map();

        struct MapData {
            std::string name;
            std::vector<double> point;

            MapData() {}

            MapData(std::string name_, std::vector<double> point_) {
                name = name_;
                point = point_;
            }
        };

        void addMapData(Key id, MapData map_data);

        MapData getMapData(Key id, int idx);

        std::vector<miltbot_common::Waypoint> getWaypointList(std::string building, std::string building_floor);

    private:
        std::string makeStringQuote(std::string data);

        std::string deleteStringQuote(std::string data);

        move_base_msgs::MoveBaseGoal convertPointToMoveBaseGoal(std::vector<double> point);

    public:
        std::map<Key, std::vector<MapData> > data;

    private:
        
};

// }