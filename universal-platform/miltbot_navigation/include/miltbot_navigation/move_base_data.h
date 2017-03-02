// #include <ros/ros.h>
#include <move_base_msgs/MoveBaseGoal.h>

namespace miltbot {

class MoveBaseGoalData {
    public:
        MoveBaseGoalData(void);

        MoveBaseGoalData(std::string goal_name, move_base_msgs::MoveBaseGoal goal, std::string building, std::string building_floor);

        MoveBaseGoalData(std::string goal_name, move_base_msgs::MoveBaseGoal goal, std::string building, std::string building_floor, std::string task);

        ~MoveBaseGoalData(void);

        void setGoal(move_base_msgs::MoveBaseGoal &goal);

        move_base_msgs::MoveBaseGoal getGoal(); 

        void setGoalName(std::string goal_name);

        std::string getGoalName();  

        void setBuilding(std::string building);

        std::string getBuilding();

        void setBuildingFloor(std::string building_floor);

        std::string getBuildingFloor();

        void setTask(std::string task);

        std::string getTask();

        void setPriority(int priority);

        int getPriority();

    private:

    public:
        std::string goal_name;
        move_base_msgs::MoveBaseGoal goal;
        std::string building;
        std::string building_floor;
        std::string task;
        int priority;
    
    private:
        
};

}