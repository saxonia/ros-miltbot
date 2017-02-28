#include <ros/ros.h>

#include <vector>
#include <iostream>
#include <algorithm>

#include "icreate_navigation/single_navigation.h"
#include "icreate_navigation/RunGmappingService.h"
#include "miltbot_map/SetMapServer.h"
#include "icreate_lift_navigation/GetMiddleRange.h"

namespace icreate {

class MultiNavigation {
    public:
        MultiNavigation();

        // MultiNavigation(int floor_count);

        ~MultiNavigation();

        bool addSingleNavigation(std::string building, std::string building_floor);

        bool removeSingleNavigation(std::string building, std::string building_floor);
        
        int getCountNavigation();

        SingleNavigation getSingleNavigation(size_t idx);

        void doneRobotGoal(Robot &robot);

        void setupRobotToRun(Robot &robot, std::string base_frame_id, std::string robot_frame_id);

        bool verifyTarget(Robot &robot);

        bool verifyTargetBuilding(MoveBaseGoalData current, MoveBaseGoalData target);

        bool verifyTargetFloor(MoveBaseGoalData current, MoveBaseGoalData target);

        void displayBuildingFloor();
        
        bool setupTargetQueue(Robot &robot);

        bool setNextStepMode(Robot &robot);

        void runLiftNavigation(Robot &robot);

        std::pair<int, int> showWaypointMenu();

        void createTimer(int duration);


    private:
        int waitForIncomingLift();

        bool verifyLiftDoor();

        void initializeSimpleForwardMoveBaseTarget(Robot &robot, std::string goal_name);        

        bool waitUserInputLift();
        
        bool static comparator (SingleNavigation i,SingleNavigation j);

    public:
        std::vector<SingleNavigation> navigations_;

        bool requestToSetNewGoal;

        int nav_idx;

        int navigation_case;
        int lift_navigation_step;
        int lift_number;

    private:
        ros::NodeHandle nh_;

        int floor_count_;

        bool isFinishQueue;

        std::string run_gmapping_service_name_;
        std::string set_map_service_name_;

        float mid_range;
};

}
