#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_srvs/Empty.h>

#include <fstream>
// #include <iostream>
#include <vector>

#include "icreate_navigation/robot.h"
#include <icreate_transportation/RunTransportation.h>

#ifndef __ICREATE_NAVIGATION_SINGLE_NAVIGATION
#define __ICREATE_NAVIGATION_SINGLE_NAVIGATION


namespace icreate {
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class SingleNavigation {
    public:
        SingleNavigation();

        ~SingleNavigation();

        void setRobotTarget(move_base_msgs::MoveBaseGoal &goal);
        // void setRobotTarget(MoveBaseGoalData &data);

        void setRobotTarget(int selected_point);

        move_base_msgs::MoveBaseGoal getRobotTarget();
        // MoveBaseGoalData getRobotTarget();

        void setRobotGoal(std::string frame_id);

        move_base_msgs::MoveBaseGoal getRobotGoal();
        // MoveBaseGoalData getRobotGoal();

        std::string doneRobotGoal(std::string robot_state);

        std::string failRobotGoal(std::string robot_state, bool &finish);

        std::string activeRobotGoal(std::string robot_state, std::string state_req);

        void getFeedbackRobotGoal();

        void readWaypointConstant();

        void readLiftFile(std::string package_name, std::string filename);

        void readWaypointFile(std::string package_name, std::string filename);

        void displayWaypoints();

        std::string getWaitForDelivery();

        int getWaitForNextPoint(int wait_time);

        void getUserInput(Robot &robot, std::string base_frame_id, std::string robot_frame_id);

        void getNextStep(Robot &robot, std::string base_frame_id, std::string robot_frame_id);

        void setNavigationMode(int mode);

        int getNavigationMode();

        void setTimer(int duration);
        

    private:
        void  timerCallback(const ros::TimerEvent &event);
        // Convert String To Int
        int toint(std::string s); //The conversion function


    public:
        enum navigation_mode_list {
            GOSPECIFIC = 0,
            DELIVERBACKTOCURRENT = 1,
            DELIVERBACKTOBASE = 2,
            EXECUTEQ = 3
        };

        enum inputMode {
            inputUser = 0,
            waitParcel = 1,
            waitQueue = 2,
            failGoal = 3
            
        };


        // Move base Goal
        move_base_msgs::MoveBaseGoal goal;

        move_base_msgs::MoveBaseGoal target;

        // std::vector<MoveBaseGoalData> targets;
        // std::vector<MoveBaseGoalData> lifts;
        std::vector<move_base_msgs::MoveBaseGoal> targets;
        std::vector<move_base_msgs::MoveBaseGoal> lifts;
        // std::vector<MoveBaseGoalData>::iterator targets_iterator;
        // std::vector<MoveBaseGoalData>::iterator lift;
        std::vector<move_base_msgs::MoveBaseGoal>::iterator targets_iterator;
        std::vector<move_base_msgs::MoveBaseGoal>::iterator lift;
        std::vector<std::string> target_name;
        std::vector<std::string> lift_name;

        //Costmap Clearing Service Client (/move_base_node/clear_costmaps)
        ros::ServiceClient client;
        std_srvs::Empty clearer;

        // MoveBaseClient ac;
        bool requestToSetNewGoal;

        //Timer
        ros::Timer timer;
        bool requestToCreateTimer;

        std::vector<int> sequence;

        // Navigation Mode
        // 0 : Go to Specific Point
        // 1 : Delivery and Come Back to This Place
        // 2 : Execute The Memorized Sequence

    private:
        //NodeHandle
        ros::NodeHandle nh_;

        int navigation_mode;
        // move_base_msgs::MoveBaseAction action;
        int selected_point;

        const static int SEQUENCE_LENGTH = 4;

        const std::string clear_costmap_service = "/move_base/clear_costmaps";
        
        //Sequence for execution
        int targetId;
        int input_mode;
};

}

#endif