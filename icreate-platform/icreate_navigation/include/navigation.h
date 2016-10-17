#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

namespace icreate {

class Navigation {
    public:
        Navigation();

        ~Navigation();

        void setRobotGoal(std::string frame_id);

        move_base_msgs::MoveBaseGoal getRobotGoal();

        std::string doneRobotGoal(std::string robot_state, bool &finish);

        std::string activeRobotGoal(std::string robot_state);

        void getFeedbackRobotGoal();

        void read_waypoint_constant();


        

    private:

    public:
        // Move base Goal
        move_base_msgs::MoveBaseGoal goal;

        std::vector<move_base_msgs::MoveBaseGoal> targets;
        std::vector<move_base_msgs::MoveBaseGoal>::iterator target;

    private:
        // move_base_msgs::MoveBaseAction action;
};

}