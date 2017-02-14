#include <ros/ros.h>
#include <std_msgs/String.h>
// #include <string>

#include <icreate_state/SetRobotState.h>

class robotState {
    public:
        robotState();

        ros::NodeHandle nh_;
        ros::Publisher state_pub_;
        ros::Subscriber state_req_sub_;
        ros::ServiceServer service_;

        enum state_list {
            IDLE = 0,
            GOING = 1,
            WAITING = 2,
            BACKTOBASE = 3,
            SENDSUPPLIES = 4,
            SINGLERUN = 5,
            EXECUTESEQ = 6,
            USINGLIFT = 7
            
        };
        std::string state_req;

        std::string convertToStateName(int state);

        int getState();
        void setState(std::string s);

        bool setRobotStateService(icreate_state::SetRobotState::Request &req,
                                  icreate_state::SetRobotState::Response &res);


    private:
        void stateCallback(const std_msgs::String::ConstPtr& msg);

        std::string state_pub_topic_name_, state_req_sub_topic_name_; 
        std::string set_robot_state_service_name_;
        int state;
};

robotState::robotState(): 
    state_pub_topic_name_("state"),
    state_req_sub_topic_name_("state_req"),
    set_robot_state_service_name_("set_robot_state")
{   
    nh_.param("/icreate_state/state_pub_topic",state_pub_topic_name_,state_pub_topic_name_);
    nh_.param("/icreate_state/state_req_sub_topic",state_req_sub_topic_name_,state_req_sub_topic_name_);
    nh_.param("/icreate_state/set_robot_state_service", set_robot_state_service_name_, set_robot_state_service_name_);
    //Initialize Publisher
    state_pub_ = nh_.advertise<std_msgs::String>(state_pub_topic_name_,1);

    //Initialize Subscriber
    // state_req_sub_ = nh_.subscribe(state_req_sub_topic_name_,10,&robotState::stateCallback, this);

    service_ = nh_.advertiseService(set_robot_state_service_name_, &robotState::setRobotStateService, this);

    state = robotState::IDLE;
    state_req = "";
}

void robotState::stateCallback(const std_msgs::String::ConstPtr& msg) {
    setState(msg->data.c_str());
    ROS_INFO("robot state request: %s",msg->data.c_str());
    ROS_INFO("robot state : %s",convertToStateName(state).c_str());
}

std::string robotState::convertToStateName(int state) {
    std::string stateName;
    switch(state) {
        case 0:
            stateName = "IDLE";
            break;
        case 1:
            stateName = "GOING";
            break;
        case 2:
            stateName = "WAITING";
            break;
        case 3:
            stateName = "BACKTOBASE";
            break;
        case 4:
            stateName = "SENDSUPPLIES";
            break;
        case 5:
            stateName = "SINGLERUN";
            break;
        case 6:
            stateName = "EXECUTESEQ";
            break;
        case 7:
            stateName = "USINGLIFT";
    }
    return stateName;
}

int robotState::getState() {
    return state;
}

void robotState::setState(std::string s) {
    state_req = s;
    if(s == "IDLE") {
        state = 0;
    }
    else if(s == "GOING") {
        state = 1;
    }
    else if(s == "WAITING") {
        state = 2;
    }
    else if(s == "BACKTOBASE") {
        state = 3;
    }
    else if(s == "SENDSUPPLIES") {
        state = 4;
    }
    else if(s == "SINGLERUN") {
        state = 5;
    }
    else if(s == "EXECUTESEQ") {
        state = 6;
    }
    else if(s == "USINGLIFT") {
        state = 7;
    }
}

bool robotState::setRobotStateService(icreate_state::SetRobotState::Request &req,
                                  icreate_state::SetRobotState::Response &res) {
    setState(req.req);
    ROS_INFO("robot state request: %s",req.req.c_str());
    ROS_INFO("robot state : %s",convertToStateName(state).c_str());
    res.res = convertToStateName(state);
    return true;
}

int main(int argc, char** argv) {
    
    //Initialize Node
    ros::init(argc, argv, "icreate_state");

    robotState robotState;

    ros::Rate loop_rate(10);

    while(ros::ok()) {
        std_msgs::String msg;
        ros::spinOnce();

        msg.data = robotState.convertToStateName(robotState.getState());
        // ROS_INFO("robot state: %s", msg.data.c_str());
        
        robotState.state_pub_.publish(msg);
        loop_rate.sleep();
    }
    
    return 0;
}