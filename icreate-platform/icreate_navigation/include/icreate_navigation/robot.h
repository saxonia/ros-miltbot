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
        Robot(void);

        ~Robot(void);

        bool setCurrentPosition(std::string base_frame_id, std::string robot_frame_id, 
                                std::string building, std::string building_floor);

        // move_base_msgs::MoveBaseGoal getCurrentPosition();
        MoveBaseGoalData getCurrentPosition();

        // bool setEndPosition(move_base_msgs::MoveBaseGoal goal);
        bool setEndPosition(MoveBaseGoalData data);

        void sendStateRequest(std::string state_request);    

    private:

    public:
        // move_base_msgs::MoveBaseGoal    startPosition;
        // move_base_msgs::MoveBaseGoal    endPosition;
        // move_base_msgs::MoveBaseGoal    currentPosition;
        MoveBaseGoalData    startPosition;
        MoveBaseGoalData    endPosition;
        MoveBaseGoalData    currentPosition;

        //Publisher & Subscriber
        ros::Publisher state_req_pub;
        ros::Subscriber state_sub;

        //Service Client
        ros::ServiceClient client;

        std::string current_state;

        // Request Flags
        // bool requestToSendStateReq;

    private:
        //NodeHandle
        ros::NodeHandle nh_;

        const std::string state_sub_topic = "/state";
        const std::string set_robot_state_service = "/set_robot_state";
};

}

#endif