#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/String.h>

#include <tf/transform_listener.h>

#include <icreate_state/SetRobotState.h>

#ifndef __ICREATE_NAVIGATION_ROBOT
#define __ICREATE_NAVIGATION_ROBOT
 
namespace icreate {

class MoveBaseGoalData {
    public:
        MoveBaseGoalData(void);

        MoveBaseGoalData(std::string goal_name, move_base_msgs::MoveBaseGoal goal, std::string building, std::string building_floor);

        ~MoveBaseGoalData(void);

        void setGoal(move_base_msgs::MoveBaseGoal &goal);

        move_base_msgs::MoveBaseGoal getGoal(); 

        void setGoalName(std::string goal_name);

        std::string getGoalName();  

        void setBuilding(std::string building);

        std::string getBuilding();

        void setBuildingFloor(std::string building_floor);

        std::string getBuildingFloor();

    private:

    public:
        std::string goal_name;
        move_base_msgs::MoveBaseGoal goal;
        std::string building;
        std::string building_floor;
    
    private:
        
};

class Robot {
    public:
        Robot(std::string building, std::string building_floor, std::string base_frame_id, std::string robot_frame_id);

        ~Robot(void);

        bool setCurrentPosition(std::string base_frame_id, std::string robot_frame_id, 
                                std::string building, std::string building_floor);

        MoveBaseGoalData getCurrentPosition();

        bool setStartPosition(MoveBaseGoalData &data);

        MoveBaseGoalData getStartPosition();

        bool setEndPosition(MoveBaseGoalData &data);

        MoveBaseGoalData getEndPosition();
        
        void addTargetQueue(MoveBaseGoalData data);

        void deleteTargetQueue(int idx);

        void setNavigationMode();

        int getNavigationMode();

        void sendStateRequest(std::string state_request);    

    private:

    public:
        MoveBaseGoalData    startPosition;
        MoveBaseGoalData    endPosition;
        MoveBaseGoalData    currentPosition;

        //Service Client
        ros::ServiceClient client;

        std::vector<MoveBaseGoalData> target_queue;

        std::string current_state;

        std::string building;
        std::string building_floor;

        std::string base_frame_id;
        std::string robot_frame_id;

    private:
        //NodeHandle
        ros::NodeHandle nh_;

        int navigation_mode;

        std::string state_sub_topic_name_;
        std::string set_robot_state_service_name_;
};

}

#endif