#include <ros/ros.h>
// #include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>

#include <tf/transform_listener.h>

// #include <iostream>
// #include <sstream>
#include <fstream>
// #include <cstdlib>
// #include <termios.h>
// #include <ctime>

#include "icreate_navigation/single_navigation.h"

//Publisher 
ros::Publisher state_req_pub;
ros::Subscriber state_sub;
std_msgs::String state_req_msg;

//Client Service of move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//Timer
ros::Timer timer;
bool requestToCreateTimer;

//Costmap Clearing Service Client (/move_base_node/clear_costmaps)
ros::ServiceClient client;
std_srvs::Empty clearer;

//Sending New Goal Request
bool requestToSetNewGoal; //sendNewGoal;

//Finish Flag
bool finish;

//State Machine Enumeration
std::string robotState;
std::string state_req;

void userInput(){
    std::cout << "Press Any Key To Start Navigation Or ESC To Exit" << std::endl;
    if((int) getchar() != 0) {
      return;
    }
}

/// CALLBACKS Function

// STATE CALLBACK
void stateCallback(const std_msgs::String::ConstPtr& msg) {
    robotState = msg->data;
    ROS_INFO("Robot state: %s",robotState.c_str());
}

// TIMER CALLBACK
void timerCallback(const ros::TimerEvent &event){
	ROS_INFO("[TimerCallback] clear costmap");
    //Clear Costmap
  	client.call(clearer);
  	//This Timer is finish , request to create it again;
    // requestToCreateTimer = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "two_point_navigation_pubsub");

    //Class
    icreate::Robot robot;
    icreate::SingleNavigation navigation; 
    std::string a = navigation.doneRobotGoal("GOING");
    std::cout << a << std::endl;   
    // robot.sendStateRequest();
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<icreate_state::SetRobotState>("set_robot_state");
    icreate_state::SetRobotState srv;
    // srv.request.req = state_req_msg;
    srv.request.req = "GOING";
    std::cout << srv.request.req << std::endl;
    if (client.call(srv))
    {
    //   current_state = srv.response.res;
    }
    else
    {
      ROS_ERROR("Failed to call service xxxxxxx");
    }

    ROS_INFO("Exiting Waypoint Navigation");
    return 0;
}
