#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_srvs/Empty.h>

#include <fstream>
#include <vector>

#include "icreate_navigation/robot.h"
#include "miltbot_map/GetWaypointList.h"
#include "miltbot_map/Waypoint.h"
// #include <icreate_transportation/RunTransportation.h>

#ifndef __ICREATE_NAVIGATION_SINGLE_NAVIGATION
#define __ICREATE_NAVIGATION_SINGLE_NAVIGATION

namespace icreate {

class SingleNavigation {
    public:
        SingleNavigation();

        SingleNavigation(std::string building, std::string building_floor);

        ~SingleNavigation();

        void setRobotTarget(MoveBaseGoalData &data);

        MoveBaseGoalData getRobotTarget();

        void setRobotGoal(std::string frame_id);

        move_base_msgs::MoveBaseGoal getRobotGoal();

        void doneRobotGoal();
        
        bool sendWaypointRequest(std::string building, std::string building_floor);

        void displayWaypoints();

        void displayLiftWaypoints();

        bool showNavigationMenu(std::string base_frame_id, std::string robot_frame_id);

        int showWaypointMenu();

        bool setNextStepMode();

        bool runRecoveryMode();
        
        void setNavigationMode(int mode);

        int getNavigationMode();

        void createTimer(int duration);

        void clearCostmap();

    private:
        void  timerCallback(const ros::TimerEvent &event);
        // Convert String To Int
        int toint(std::string s); //The conversion function

        void setWaypoint(std::vector<miltbot_map::Waypoint> waypoints, std::string building_floor);

    public:
        icreate::Robot robot;

        std::string building;
        std::string building_floor;

        bool requestToSetNewGoal;
        bool requestToCreateTimer;

    private:
        //NodeHandle
        ros::NodeHandle nh_;

        //SeviceClient for clear costmap service
        ros::ServiceClient clear_costmap_client_;

        //Timer
        ros::Timer timer_;

        // Move base Goal
        move_base_msgs::MoveBaseGoal goal;

        MoveBaseGoalData target;

        std::vector<MoveBaseGoalData> target_queue;
        std::vector<MoveBaseGoalData> targets;
        std::vector<MoveBaseGoalData> lifts;      
        std::vector<MoveBaseGoalData>::iterator target_iterator; 

        std::string base_frame_id; 
        std::string robot_frame_id; 

        int navigation_mode;

        std::string clear_costmap_service_name_;
        std::string get_waypoint_list_service_name_;
};

}

#endif