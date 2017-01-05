#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/String.h>

#include <tf/transform_listener.h>

#include <icreate_state/SetRobotState.h>
 
namespace icreate {

class Robot {
    public:
        Robot(void);

        ~Robot(void);

        bool setCurrentPosition();

        move_base_msgs::MoveBaseGoal getCurrentPosition();

        bool setEndPosition(move_base_msgs::MoveBaseGoal goal);

        void sendStateRequest();    

    private:
        void stateCallback(const std_msgs::String::ConstPtr& msg);

    public:
        move_base_msgs::MoveBaseGoal    startPosition;
        move_base_msgs::MoveBaseGoal    endPosition;
        move_base_msgs::MoveBaseGoal    currentPosition;

        //Publisher & Subscriber
        ros::Publisher state_req_pub;
        ros::Subscriber state_sub;
        std_msgs::String state_req_msg;

        //Service Client
        ros::ServiceClient client;

        std::string current_state;

        // Request Flags
        bool requestToSendStateReq;

    private:
        //NodeHandle
        ros::NodeHandle nh_;
};

}