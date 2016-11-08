#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_srvs/Empty.h>

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

        void readWaypointConstant();

        void readWaypointFile(std::string filename);

        void displayWaypoints();

        void getUserInput();

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

        // Move base Goal
        move_base_msgs::MoveBaseGoal goal;

        std::vector<move_base_msgs::MoveBaseGoal> targets;
        std::vector<move_base_msgs::MoveBaseGoal>::iterator target;
        std::vector<std::string> target_name;

        //Costmap Clearing Service Client (/move_base_node/clear_costmaps)
        ros::ServiceClient client;
        std_srvs::Empty clearer;

        MoveBaseClient ac;

        //Timer
        ros::Timer timer;
        bool requestToCreateTimer;

        // Navigation Mode
        // 0 : Go to Specific Point
        // 1 : Delivery and Come Back to This Place
        // 2 : Execute The Memorized Sequence

    private:
        //NodeHandle
        // ros::NodeHandle nh_;

        int navigation_mode;
        // move_base_msgs::MoveBaseAction action;
        int selected_point;
};

}