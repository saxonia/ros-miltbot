#include <ros/ros.h>
// #include <actionlib/client/simple_action_client.h>
// #include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

// #include "robot.h"
#include <iostream>
#include <string>
#include "navigation.h"



//Publisher & Subscriber
ros::Publisher state_req_pub;
ros::Subscriber state_sub;
std_msgs::String state_req_msg;
std::string current_state;
std::string state_req;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//Costmap Clearing Service Client (/move_base_node/clear_costmaps)
ros::ServiceClient client;
std_srvs::Empty clearer;

//Timer
ros::Timer timer;

//Sequence for Execution
int targetId;

bool requestToCreateTimer;
bool requestToMarkLocation;
bool requestToSendStateReq;
bool requestToSetNewGoal;
bool isDoneGoal;
bool isNextStep;
bool isFrontLiftToLift;
int doneGoalNumber;

void setLiftPosition(icreate::Navigation &navigation, int liftNumber) {
    navigation.lift = navigation.lifts.begin() + liftNumber;
    navigation.target = navigation.lift;
    ROS_INFO("set Robot Lift: %s", navigation.lift_name[liftNumber].c_str());
    requestToSetNewGoal = true;
}

bool userInput(icreate::Robot &robot,icreate::Navigation &navigation) {
    std::cout << "Press Any Key To Start Navigation Or ESC To Exit" << std::endl;
    std::string key;
    std::cin >> key;
    
    robot.setCurrentPosition();
    robot.state_req_msg.data = "SINGLERUN";
    
    //Edit -----------------------------------------
    move_base_msgs::MoveBaseGoal newPoint;
    newPoint.target_pose.pose.position.x    = 4.538;
    newPoint.target_pose.pose.position.y    = 10.365;
    newPoint.target_pose.pose.orientation.x = 0.000;
    newPoint.target_pose.pose.orientation.y = 0.000;
    newPoint.target_pose.pose.orientation.z = 0.983;
    newPoint.target_pose.pose.orientation.w = 0.180;
    navigation.targets.push_back(newPoint);
    
    navigation.target = navigation.targets.begin(); 
    //End ---------------------------------
	robot.setEndPosition(navigation.targets[0]);
	requestToSetNewGoal = true;
	robot.requestToSendStateReq = true;
    return true;
    // I'm not sure about ascii code == 0 what it is.
}

void nextStep(icreate::Robot &robot,icreate::Navigation &navigation) {
    std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;
	std::cout << "[AGENT] Select UP OR DOWN";
    std::string value;
    std::cin >> value;
    
    std::cout << "You Selected : " << value <<std::endl;
    // if(strcmp(value,"u") != 0 && strcmp(value,"d") != 0) {
    if( value != "u" && value != "d") {
        std::cout << "[AGENT] Wrong INput End" <<std::endl; 
        return;
    }
    std::cout << "[AGENT] Wait For Number of Lift" <<std::endl;
    std::string liftNumber;
    std::cin >> liftNumber;
    std::cout << "[AGENT] Lift Number " + liftNumber + " is on the floor" <<std::endl;
    setLiftPosition(navigation, std::stoi(liftNumber));
    robot.setEndPosition(navigation.lifts[std::stoi(liftNumber)]);
    ROS_INFO("Get");
    robot.requestToSendStateReq = true;
}

void setFrontLiftToLift(icreate::Robot &robot,icreate::Navigation &navigation) {

}

// STATE CALLBACK
void stateCallback(const std_msgs::String::ConstPtr& msg) {
    current_state = msg->data;
	ROS_INFO("Robot state: %s",current_state.c_str());
}

// TIMER CALLBACK
void timerCallback(const ros::TimerEvent &event){
	ROS_INFO("[TimerCallback] clear costmap");
	//Clear Costmap
  	client.call(clearer);
  	//This Timer is finish , request to create it again;
    // requestToCreateTimer = true;
}

void goalDoneCallback_state(const actionlib::SimpleClientGoalState &state, 
  const move_base_msgs::MoveBaseResultConstPtr &result){
    ROS_INFO("Goal Done Now !!!!!");

    //Navigation completed
    if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED){
        doneGoalNumber = 1;
    }
    //Navigation failed
    if(state.state_ == actionlib::SimpleClientGoalState::ABORTED){
        doneGoalNumber = 4;
    }

    std::cout << "Goal Finished Or Cancelled by Joy" << std::endl;
	//   requestToSendStateReq = true;
	  isDoneGoal = true;
}

void goalActiveCallback(){
    // state_req_msg.data = navigation.activeRobotGoal(current_state, state_req);
	  // requestToSendStateReq = true;
}

void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback){
    // navigation.getFeedbackRobotGoal();
}

void aaa(icreate::Navigation &navigation, icreate::Robot &robot) {
	ROS_INFO("SUCCEEDED %s",robot.current_state.c_str());
	robot.state_req_msg.data = navigation.doneRobotGoal(robot.current_state);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "multifloor_test");

    // Class
    icreate::Robot robot;
    icreate::Navigation navigation;

    MoveBaseClient ac("/icreate/move_base", true);

    ros::Rate r(30);

    navigation.readLiftFile("/waypoint/build4_f20lift.csv");

    robot.setCurrentPosition();

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
    ros::NodeHandle nh;

    // Subsribe Robot State Topic
    // state_sub = nh.subscribe("/icreate/state",100,stateCallback);
    // state_req_pub = nh.advertise<std_msgs::String>("/icreate/state_req",10);

    // Subscribe to Map Clearing Service 
    // client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

    // requestToCreateTimer = true;
    // requestToSendStateReq = false;
	isDoneGoal = false;
    isNextStep = true;
    doneGoalNumber = 0;
    // requestToSetNewGoal = false;

    navigation.target = navigation.targets.begin();
    navigation.lift = navigation.lifts.begin();


    userInput(robot, navigation);    

    while(ros::ok()) {
        ros::spinOnce();
        r.sleep();

        // if(requestToSendStateReq) {
        //     ROS_INFO("Loop Send State Request");
        //             requestToSendStateReq = false;
        //             state_req_pub.publish(state_req_msg);
        // }
        if(robot.requestToSendStateReq) {
			// ROS_INFO("Stateee: %s",  robot.state_req_msg.data.c_str());
			robot.sendStateRequest();
		}

        if(isDoneGoal)
		{
			isDoneGoal = false;
			if(doneGoalNumber == 1)
			{
				aaa(navigation, robot);
			}
			robot.requestToSendStateReq = true;
			isNextStep = true;
		}

        if(isNextStep) {
			ROS_INFO("Loop Done Goal");
			isNextStep = false;
			// navigation.getNextStep(robot);
            nextStep(robot, navigation);
		}

        if(isFrontLiftToLift) {
            ROS_INFO("Move From Front Lift To Inside Lift");
            isFrontLiftToLift = false;
            setFrontLiftToLift(robot, navigation);
        }

        if(requestToSetNewGoal) {
            ROS_INFO("Loop Set New Goal");
                    requestToSetNewGoal = false;
            navigation.setRobotGoal("/map");
            ac.sendGoal(navigation.goal, 
                        boost::bind(&goalDoneCallback_state, _1, _2), 
                        boost::bind(&goalActiveCallback), boost::bind(&goalFeedbackCallback, _1));
        }
        // if(navigation.requestToSetNewGoal) {
		// 	ROS_INFO("Loop Set New Goal");
		// 	navigation.requestToSetNewGoal = false;

		// 	navigation.setRobotGoal("/map");
		// 	ROS_INFO("Stateee: %s",  robot.current_state.c_str());
		// 	ac.sendGoal(navigation.goal, 
        //               boost::bind(&goalDoneCallback_state, _1, _2, navigation, robot), 
        //               boost::bind(&goalActiveCallback, navigation, robot), boost::bind(&goalFeedbackCallback, _1, navigation));
		// }

        // if(requestToCreateTimer) {
        //     ROS_INFO("Loop Create Timer");
        //     requestToCreateTimer = false;
        //     timer = nh.createTimer(ros::Duration(10), timerCallback);
        // }
        if(navigation.requestToCreateTimer) {
			navigation.setTimer(10);
		}
    }

    ROS_INFO("Exiting MultiFloor Test Navigation");
    return 0;
}