#include <ros/ros.h>
#include <ros/package.h>
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

#include "robot.h"
#include "navigation.h"

//Class
icreate::Robot robot;
icreate::Navigation navigation;

//Publisher 
ros::Publisher state_pub;
ros::Subscriber state_sub;
std_msgs::String state_msg;

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

//Location Marking Request
bool requestToMarkLocation;

//Waypoints 
std::vector<move_base_msgs::MoveBaseGoal> targets;
std::vector<move_base_msgs::MoveBaseGoal>::iterator target;
std::vector<std::string> target_name;

//Goal 
move_base_msgs::MoveBaseGoal startPoint;
move_base_msgs::MoveBaseGoal  endPoint;
move_base_msgs::MoveBaseGoal  currentPosition;

//State Machine Enumeration
std::string robotState;


void userInput(){
    std::cout << "Press Any Key To Start Navigation" << std::endl;
    if((int) getchar() != 0) {
      return;
    }
}

/// CALLBACKS Function

// STATE CALLBACK
void stateCallback(const std_msgs::String::ConstPtr& msg) {
    robotState = msg->data;
}

// TIMER CALLBACK
void timerCallback(const ros::TimerEvent &event){
	//Clear Costmap
  	client.call(clearer);
  	//This Timer is finish , request to create it again;
    requestToCreateTimer = true;
    return;
}

void goalDoneCallback_state(const actionlib::SimpleClientGoalState &state, 
  const move_base_msgs::MoveBaseResultConstPtr &result){
    ROS_INFO("Goal Done Now !!!!!");

    //Navigation completed
    if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED){
    	//Clear Costmap
  		client.call(clearer);
        state_msg.data = navigation.doneRobotGoal(robotState, finish);
    }
    //Navigation failed
    if(state.state_ == actionlib::SimpleClientGoalState::ABORTED){
      ROS_WARN("Failed to reach the goal...");
    }

    std::cout << "Goal Finished Or Cancelled by Joy" << std::endl;

}

void goalActiveCallback(){
    state_msg.data = navigation.activeRobotGoal(robotState);
}

void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback){
    navigation.getFeedbackRobotGoal();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_navigation_goal");

    // Tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);
    
    navigation.read_waypoint_constant();

    ROS_INFO("Successfully Load waypoints !");

    if(!robot.setCurrentPosition()) {
        return -1;
    }

    // Request To Create timer
    requestToCreateTimer = true;

    // Callback polling Rate 
    ros::Rate r(30);

    // Wait for the action server to come up
    int tries = 0;
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up %d",tries+1);
      tries++;
      if(tries == 3){
        ROS_INFO("Failed to Start Waypoint Node");
        return -1;
      }
    }
    ROS_INFO("Navigation Waypoint Node Initialized !");

    // Subscriber to Get Current position 
    ros::NodeHandle n;

    // Subsribe Robot State Topic
    state_sub = n.subscribe("state",100,stateCallback);
    state_pub = n.advertise<std_msgs::String>("state_req",10);

    // Subscribe to Map Clearing Service 
    client = n.serviceClient<std_srvs::Empty>("/move_base_node/clear_costmaps");

    // Ask User For Input
    userInput();

    finish = false;

    // Loop for Setting Goal and Navigate ! 
    requestToSetNewGoal = true;

    // Start the Navigation Waypoint Loop
    while(ros::ok() && !finish ){

        // The Next Goal ! 
        if(requestToSetNewGoal){

            // Do this Target until its end
            requestToSetNewGoal = false;

            navigation.setRobotGoal("/map");
            
            // Send Goal to Navigation Stack
            ac.sendGoal(navigation.getRobotGoal(), 
                        boost::bind(&goalDoneCallback_state, _1, _2), 
                        boost::bind(&goalActiveCallback), boost::bind(&goalFeedbackCallback, _1));
            state_pub.publish(state_msg);
        }

        //Check if the timer should be create.
        if(requestToCreateTimer){
            timer = n.createTimer(ros::Duration(10), timerCallback);
            requestToCreateTimer = false;  //We Don't want to create timer anymore
        }
 
        // Spinning the loop and Callback
        ros::spinOnce();
        r.sleep();
    }

    ROS_INFO("Exiting Waypoint Navigation");
    return 0;
}
