#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>

// #include "miltbot_navigation/move_base_data.h"

#include "icreate_navigation/RunGmappingService.h"
#include "icreate_lift_navigation/GetMiddleRange.h"

#include "miltbot_common/WaypointList.h"
#include "miltbot_map/GetWaypointList.h"
#include "miltbot_map/SetMap.h"
#include "miltbot_state/SetRobotState.h"
#include "miltbot_system/ViewTargetQueue.h"
#include "miltbot_system/AddTarget.h"
#include "miltbot_system/DeleteTarget.h"
#include "miltbot_system/RunSystem.h"
#include "miltbot_transportation/RunTransportation.h"

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

        void addTargetQueue(miltbot_common::Waypoint data);

        void deleteTargetQueue(long id);

        void addDefaultTargetQueue(miltbot_common::Waypoint data);

        void setBuilding(std::string building);

        std::string getBuilding();

        void setBuildingFloor(std::string building_floor); 

        std::string getBuildingFloor();

        bool setCurrentPosition(std::string current_position_name);

        bool setCurrentPosition(miltbot_common::Waypoint current_position);

        miltbot_common::Waypoint getCurrentPosition();

        void setRobotTarget(miltbot_common::Waypoint data);
        
        miltbot_common::Waypoint getRobotTarget();

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

        bool verifyTargetBuilding(miltbot_common::Waypoint current, miltbot_common::Waypoint target);

        bool verifyTargetFloor(miltbot_common::Waypoint current, miltbot_common::Waypoint target);

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

        bool viewTargetQueueService(miltbot_system::ViewTargetQueue::Request &req,
                                miltbot_system::ViewTargetQueue::Response &res);

        bool addTargetService(miltbot_system::AddTarget::Request &req,
                            miltbot_system::AddTarget::Response &res);

        bool deleteTargetService(miltbot_system::DeleteTarget::Request &req,
                            miltbot_system::DeleteTarget::Response &res);

        bool addDefaultTargetService(miltbot_system::AddTarget::Request &req,
                            miltbot_system::AddTarget::Response &res);

        bool runSystemService(miltbot_system::RunSystem::Request &req,
                            miltbot_system::RunSystem::Response &res);
        
        void callRunTransportationService(std::string mode);
        
        void sendStateRequest(std::string state_request);  

        bool sendWaypointRequest(std::string building, std::string building_floor);

        void sendMoveBaseCancel();  

        void setWaypoint(std::vector<miltbot_common::Waypoint> waypoints);

    public:
        bool requestToSetNewGoal;
        bool isSystemWorking;
        bool isDoneGoal;
        bool isLiftNavigation;

        std::vector<miltbot_common::Waypoint> default_queue;
        std::vector<miltbot_common::Waypoint> target_queue;
        std::vector<miltbot_common::Waypoint> lifts;
        miltbot_common::Waypoint    currentPosition;

        std::string base_frame_id;
        std::string robot_frame_id;
        std::string current_state;
        std::string building;
        std::string building_floor;
        std::string building_floor_lift;

        int navigation_case;
        int lift_navigation_step;
        int fail_goal_value_;

        ros::ServiceServer view_target_queue_server;
        ros::ServiceServer add_target_service_server;
        ros::ServiceServer delete_target_service_server;
        ros::ServiceServer add_default_target_service_server;
        ros::ServiceServer run_system_service_server;

    private:
        //NodeHandle
        ros::NodeHandle nh_;

        MoveBaseClient ac;     

        //Publisher
        ros::Publisher move_base_cancel_pub;
        ros::Publisher target_queue_pub;

        //SeviceClient for clear costmap service
        ros::ServiceClient clear_costmap_client_;
        ros::ServiceClient set_robot_state_client_;
        ros::ServiceClient run_gmapping_client_;
        ros::ServiceClient get_middle_range_client_;
        ros::ServiceClient set_map_service_client_;
        ros::ServiceClient run_transportation_client_;

        //Timer
        ros::Timer timer_;

        // Move base Goal
        move_base_msgs::MoveBaseGoal goal;

        miltbot_common::Waypoint target;

        int done_goal_number;
        int fail_goal_count;

        float mid_range;

        std::string clear_costmap_service_name_;
        std::string get_waypoint_list_service_name_;
        std::string move_base_topic_name_;
        float move_base_wait_time_;
        std::string view_target_queue_service_name_;
        std::string add_target_service_name_;
        std::string delete_target_service_name_;
        std::string set_robot_state_service_name_;
        std::string run_gmapping_service_name_;
        std::string set_map_service_name_;
        std::string get_middle_range_service_name_;
        std::string add_default_target_service_name_;
        std::string run_system_service_name_;
        std::string run_transportation_service_name_;
        std::string move_base_cancel_pub_topic_name_;
        std::string target_queue_pub_topic_name_;
        

};

}

#endif