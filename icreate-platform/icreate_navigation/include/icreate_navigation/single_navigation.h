#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_srvs/Empty.h>

#include <fstream>
#include <iostream>
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
        SingleNavigation(std::string building, std::string building_floor);

        ~SingleNavigation(void);

        void setRobotTarget(MoveBaseGoalData data);

        // void setRobotTarget();
        
        MoveBaseGoalData getRobotTarget();

        void setRobotGoal(std::string frame_id);

        // void setRobotGoal(std::string frame_id);

        move_base_msgs::MoveBaseGoal getRobotGoal();

        void doneRobotGoal(Robot &robot);
        
        bool sendWaypointRequest(std::string building, std::string building_floor);

        void displayWaypoints();

        void displayLiftWaypoints();

        void setupRobotToRun(Robot &robot, std::string base_frame_id, std::string robot_frame_id);

        bool start(Robot &robot);

        int showWaypointMenu();

        bool update(Robot &robot);

        bool runRecoveryMode();

        void createTimer(int duration);

        void clearCostmap();

    private:
        void  timerCallback(const ros::TimerEvent &event);
        // Convert String To Int
        int toint(std::string s); //The conversion function

        void setWaypoint(std::vector<miltbot_map::Waypoint> waypoints, std::string building_floor);

        std::string substrBuildingFloor(std::string building_floor);

    public:
        std::string building;
        std::string building_floor;
        std::string building_floor_lift;
        int building_floor_num;

        std::vector<MoveBaseGoalData> targets;
        std::vector<MoveBaseGoalData> lifts;

        bool requestToSetNewGoal;

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
        // std::vector<MoveBaseGoalData>::iterator target_iterator; 

        bool isFinishQueue;

        std::string base_frame_id; 
        std::string robot_frame_id; 

        std::string clear_costmap_service_name_;
        std::string get_waypoint_list_service_name_;
};

}

#endif