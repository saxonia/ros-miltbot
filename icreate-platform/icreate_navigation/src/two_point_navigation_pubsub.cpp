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


//Publisher 
ros::Publisher state_pub;
ros::Subscriber state_sub;
std_msgs::String msg;

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
  int robot_state;
  std::string robotState;


// Convert String To Int
int toint(std::string s) //The conversion function
{
    return atoi(s.c_str());
}

void markCurrentLocation(){

    tf::TransformListener listener;  
    tf::StampedTransform transform;
    ros::Time now = ros::Time::now();
    try {
      ROS_INFO("WAITING FOR TRANSFORM FRAME");
        listener.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("map", "base_footprint", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    startPoint.target_pose.pose.position.x    = transform.getOrigin().x();
    startPoint.target_pose.pose.position.y    = transform.getOrigin().y();
    startPoint.target_pose.pose.orientation.x = transform.getRotation().x();
    startPoint.target_pose.pose.orientation.y = transform.getRotation().y();
    startPoint.target_pose.pose.orientation.z = transform.getRotation().z();
    startPoint.target_pose.pose.orientation.w = transform.getRotation().w();

    std::cout << "RETURNING POINT MARKED : ";
    std::cout << startPoint.target_pose.pose.position.x <<","<<startPoint.target_pose.pose.position.y  <<std::endl;
}

// 
void userInput(){
    std::cout << "Press Any Key To Start Navigation" << std::endl;
    if((int) getchar() != 0) {
      return;
    }
}

void read_waypoint_constant()
{
    move_base_msgs::MoveBaseGoal newPoint;
    newPoint.target_pose.pose.position.x    = -6.326;
    newPoint.target_pose.pose.position.y    = -0.674;
    newPoint.target_pose.pose.orientation.x = 0.000;
    newPoint.target_pose.pose.orientation.y = 0.000;
    newPoint.target_pose.pose.orientation.z = -2.870;
    newPoint.target_pose.pose.orientation.w = 0.958;
    targets.push_back(newPoint);
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
        ROS_INFO("The goal was reached!");
        if(robotState == "GOING") {
            msg.data = "WAITING";
            // ----------------------------------- Not Implemented Now ------------------------ //
            // waitfordelivery();
        }
        else if(robotState == "BACKTOBASE") {
            msg.data = "IDLE";  
        }
    }
    //Navigation failed
    if(state.state_ == actionlib::SimpleClientGoalState::ABORTED){
      ROS_WARN("Failed to reach the goal...");
    }

    std::cout << "Goal Finished Or Cancelled by Joy" << std::endl;

}

void goalActiveCallback(){
    ROS_INFO("Goal active! Now !!");
    if(robotState == "IDLE") {
        msg.data = "GOING";
    }
    else if(robotState == "WAITING") {
        msg.data = "BACKTOBASE";
    } 
    
}


void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback){
    ROS_INFO("Getting feedback! How cool is that?");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_navigation_goal");

    // Tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Read waypoint from constant
    read_waypoint_constant();

    ROS_INFO("Successfully Load waypoints !");
    
    // Point The iterator to the beginning of the sequence
    target = targets.begin();

    // Marking Current Location (For First Time Usage)
    markCurrentLocation();

    // Request To Create timer
    requestToCreateTimer = true;

    // Callback polling Rate 
    ros::Rate r(30);

    // Flag For Marking Base Location (Request Current Location Flag)
    requestToMarkLocation = false;

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

            // Move base Goal
            move_base_msgs::MoveBaseGoal goal;
          
            // Send a goal to the robot
            goal.target_pose.header.frame_id = "/map";
            goal.target_pose.header.stamp = ros::Time::now();
          
            // Set the goal
            goal.target_pose.pose.position.x    = target->target_pose.pose.position.x;
            goal.target_pose.pose.position.y    = target->target_pose.pose.position.y;
            goal.target_pose.pose.orientation.x = target->target_pose.pose.orientation.x;
            goal.target_pose.pose.orientation.y = target->target_pose.pose.orientation.y;
            goal.target_pose.pose.orientation.z = target->target_pose.pose.orientation.z;
            goal.target_pose.pose.orientation.w = target->target_pose.pose.orientation.w;
            std::cout << "[AGENT] MARKED NEW TARGET ! " << std::endl;
            // Send Goal to Navigation Stack
            ac.sendGoal(goal, 
                        boost::bind(&goalDoneCallback_state, _1, _2), 
                        boost::bind(&goalActiveCallback), boost::bind(&goalFeedbackCallback, _1));
            state_pub.publish(msg);
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
