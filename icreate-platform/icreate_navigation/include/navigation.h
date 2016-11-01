#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <fstream>

    

namespace icreate {
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Navigation {
    public:
        Navigation();

        ~Navigation();

        void setRobotTarget(move_base_msgs::MoveBaseGoal goal);

        void setRobotTarget(int selected_point);

        move_base_msgs::MoveBaseGoal getRobotTarget();

        void setRobotGoal(std::string frame_id);

        move_base_msgs::MoveBaseGoal getRobotGoal();

        std::string doneRobotGoal(std::string robot_state, int &mode);

        std::string failRobotGoal(std::string robot_state, bool &finish);

        std::string activeRobotGoal(std::string robot_state, std::string state_req);

        void getFeedbackRobotGoal();

        void read_waypoint_constant();

        void read_waypoint_file(std::string filename);

        void displayWaypoints();

        void getUserInput();
        

    private:
        // Convert String To Int
        int toint(std::string s); //The conversion function


    public:
        // Move base Goal
        move_base_msgs::MoveBaseGoal goal;

        std::vector<move_base_msgs::MoveBaseGoal> targets;
        std::vector<move_base_msgs::MoveBaseGoal>::iterator target;
        std::vector<std::string> target_name;

        MoveBaseClient* ac();

        // Navigation Mode
        // 0 : Go to Specific Point
        // 1 : Delivery and Come Back to This Place
        // 2 : Execute The Memorized Sequence
        int navigation_mode;

    private:
        // move_base_msgs::MoveBaseAction action;
        int selected_point;
};

}