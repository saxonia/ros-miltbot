#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>

#include "miltbot_navigation/move_base_data.h"
#include "miltbot_state/SetRobotState.h"
#include "miltbot_system/AddTarget.h"
#include "miltbot_map/GetWaypointList.h"
#include "icreate_navigation/RunGmappingService.h"
#include "icreate_lift_navigation/GetMiddleRange.h"
#include "miltbot_map/SetMap.h"
#include "miltbot_system/RunSystem.h"

#ifndef __MILTBOT_NAVIGATION_NAVIGATION
#define __MILTBOT_NAVIGATION_NAVIGATION

namespace miltbot {

//Client Service of move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class Navigation {
    public:
        Navigation(std::string base_frame_id, std::string robot_frame_id, std::string building, std::string building_floor);

        ~Navigation(void);

        enum navigationCase {
            ONSAMEFLOOR = 0,
            ONDIFFFLOOR = 1,
            ONDIFFBUILDING = 2
        };

        void addTargetQueue(MoveBaseGoalData data);

        void deleteTargetQueue(int idx);

        void addDefaultTargetQueue(MoveBaseGoalData data);

        void setBuilding(std::string building);

        std::string getBuilding();

        void setBuildingFloor(std::string building_floor); 

        std::string getBuildingFloor();

        bool setCurrentPosition(std::string current_position_name);

        bool setCurrentPosition(MoveBaseGoalData current_position);

        MoveBaseGoalData getCurrentPosition();

        void setRobotTarget(MoveBaseGoalData data);
        
        MoveBaseGoalData getRobotTarget();

        void setRobotGoal(std::string frame_id);

        move_base_msgs::MoveBaseGoal getRobotGoal();

        bool verifyTarget();

        bool update();

        void runMoveBase();

        void runLiftNavigation();

        void createTimer(int duration);

        void clearCostmap();


    private:
        void timerCallback(const ros::TimerEvent &event);

        bool waitMoveBaseServer(float wait_duration);

        bool verifyTargetBuilding(MoveBaseGoalData current, MoveBaseGoalData target);

        bool verifyTargetFloor(MoveBaseGoalData current, MoveBaseGoalData target);

        void displayLiftWaypoints();
        
        int waitForIncomingLift();

        bool verifyLiftDoor();

        bool waitUserInputLift();

        void initializeLiftForwardMoveBase();

        void initializeLiftRotateMoveBase();

        void initializeLiftForwardOutMoveBase();
        
        void goalDoneCallback(const actionlib::SimpleClientGoalState &state, 
                            const move_base_msgs::MoveBaseResultConstPtr &result);

        void goalActiveCallback();

        void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback);

        bool addTargetService(miltbot_system::AddTarget::Request &req,
                            miltbot_system::AddTarget::Response &res);

        bool addDefaultTargetService(miltbot_system::AddTarget::Request &req,
                            miltbot_system::AddTarget::Response &res);

        bool runSystemService(miltbot_system::RunSystem::Request &req,
                            miltbot_system::RunSystem::Response &res);
        
        void sendStateRequest(std::string state_request);  

        bool sendWaypointRequest(std::string building, std::string building_floor);  

        void setWaypoint(std::vector<miltbot_map::Waypoint> waypoints);

    public:
        bool requestToSetNewGoal;
        bool isSystemWorking;
        bool isDoneGoal;
        bool isLiftNavigation;

        std::vector<MoveBaseGoalData> default_queue;
        std::vector<MoveBaseGoalData> target_queue;
        std::vector<MoveBaseGoalData> lifts;
        MoveBaseGoalData    currentPosition;

        std::string base_frame_id;
        std::string robot_frame_id;
        std::string current_state;
        std::string building;
        std::string building_floor;
        std::string building_floor_lift;

        int navigation_case;
        int lift_navigation_step;
        int fail_goal_value_;

        ros::ServiceServer add_target_service_server;
        ros::ServiceServer add_default_target_service_server;
        ros::ServiceServer run_system_service_server;

    private:
        //NodeHandle
        ros::NodeHandle nh_;

        MoveBaseClient ac;     

        //SeviceClient for clear costmap service
        ros::ServiceClient clear_costmap_client_;
        ros::ServiceClient set_robot_state_client_;
        ros::ServiceClient run_gmapping_client_;
        ros::ServiceClient get_middle_range_client_;
        ros::ServiceClient set_map_service_client_;

        //Timer
        ros::Timer timer_;

        // Move base Goal
        move_base_msgs::MoveBaseGoal goal;

        MoveBaseGoalData target;

        int done_goal_number;
        int fail_goal_count;

        float mid_range;

        std::string clear_costmap_service_name_;
        std::string get_waypoint_list_service_name_;
        std::string move_base_topic_name_;
        float move_base_wait_time_;
        std::string add_target_service_name_;
        std::string set_robot_state_service_name_;
        std::string run_gmapping_service_name_;
        std::string set_map_service_name_;
        std::string get_middle_range_service_name_;
        std::string add_default_target_service_name_;
        std::string run_system_service_name_;

};

}

#endif