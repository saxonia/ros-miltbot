#include <ros/ros.h>
#include <map>
#include <vector>

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

        std::pair<std::vector<std::string>, std::vector<std::string> > getWaypointNameList(std::string building);

    private:
        std::string makeStringQuote(std::string data);

        std::string deleteStringQuote(std::string data);

    public:
        std::map<Key, std::vector<MapData> > data;

    private:
        
};

// }