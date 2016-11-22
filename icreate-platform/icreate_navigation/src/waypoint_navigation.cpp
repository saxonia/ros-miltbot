#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

// #include "robot.h"
#include "navigation.h"

//Client Service of move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool isDoneGoal;
int doneGoalNumber;
bool isNextStep;

//Finish Flag
bool finish;

//State Machine Enumeration
std::string state_req;

bool waitMoveBaseServer(MoveBaseClient &ac) {
	int tries = 0;
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up %d",tries+1);
      tries++;
      if(tries == 3){
        ROS_INFO("Failed to Start Waypoint Node");
        return false;
      }
    }
    ROS_INFO("Navigation Waypoint Node Initialized !");
	return true;
}


/// CALLBACKS Function

void goalDoneCallback_state(const actionlib::SimpleClientGoalState &state, 
  const move_base_msgs::MoveBaseResultConstPtr &result){
    ROS_INFO("Goal Done Now !!!!!");

    //Navigation completed
    if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED){
		doneGoalNumber = 1;
	}

	if(state.state_ == actionlib::SimpleClientGoalState::REJECTED) {
		ROS_INFO("REJECTED");
		doneGoalNumber = 2;
	}

	if(state.state_ == actionlib::SimpleClientGoalState::LOST) {
		ROS_INFO("LOST");
		doneGoalNumber = 3;
	}

    //Navigation failed
    if(state.state_ == actionlib::SimpleClientGoalState::ABORTED){
      	ROS_WARN("Failed to reach the goal...");
		// robot.state_req_msg.data = navigation.failRobotGoal(robot.current_state, finish);
		// input_mode = 3;
		doneGoalNumber = 4;
    }

    std::cout << "Goal Finished Or Cancelled by Joy" << std::endl;
	// robot.requestToSendStateReq = true;
	isDoneGoal = true;
}

void goalActiveCallback(icreate::Navigation &navigation, icreate::Robot &robot){
    robot.state_req_msg.data = navigation.activeRobotGoal(robot.current_state, state_req);
	robot.requestToSendStateReq = true;
}

void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback, icreate::Navigation &navigation, icreate::Robot &robot){
    // navigation.getFeedbackRobotGoal();
	// ROS_INFO("FEEDBACK %s",robot.current_state.c_str());
}

void runControlLoop(icreate::Navigation &navigation, icreate::Robot robot) {

}

void aaa(icreate::Navigation &navigation, icreate::Robot &robot) {
	ROS_INFO("SUCCEEDED %s",robot.current_state.c_str());
	robot.state_req_msg.data = navigation.doneRobotGoal(robot.current_state);
}

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "waypoint_navigation");

	//Class
	icreate::Robot robot;
	icreate::Navigation navigation;

    MoveBaseClient ac("/icreate/move_base", true);

	// Callback polling Rate 
    ros::Rate r(30);

    navigation.readWaypointFile("/waypoint/build4_f20.csv");
    // if(!robot.setCurrentPosition()) {
    //     return -1;
    // }
	robot.setCurrentPosition();

    // Wait for the action server to come up
    if(!waitMoveBaseServer(ac)) {
		return -1;
	}
		

    // Subscriber to Get Current position 
    ros::NodeHandle nh;

	// Point The iterator to the beginning of the sequence
    navigation.target = navigation.targets.begin();
	
	// Request To Create timer
    // requestToCreateTimer = true;

	// requestToSendStateReq = false;
	isDoneGoal = false;
	doneGoalNumber = 0;
    // requestToSetNewGoal = false;

    // Ask User For Input
    navigation.getUserInput(robot);
	

	

	// Start the Navigation Waypoint Loop
	while(ros::ok() && !finish) {
		ros::spinOnce();
		r.sleep();

		// if(requestToSendStateReq) {
		// 	ROS_INFO("Loop Send State Request");
		// 	requestToSendStateReq = false;
		// 	state_req_pub.publish(state_req_msg);
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
			navigation.getNextStep(robot);
		}

		if(navigation.requestToSetNewGoal) {
			ROS_INFO("Loop Set New Goal");
			navigation.requestToSetNewGoal = false;

			navigation.setRobotGoal("/map");
			ROS_INFO("Stateee: %s",  robot.current_state.c_str());
			ac.sendGoal(navigation.goal, 
                      boost::bind(&goalDoneCallback_state, _1, _2), 
                      boost::bind(&goalActiveCallback, navigation, robot), boost::bind(&goalFeedbackCallback, _1, navigation, robot));
		}
		// ROS_INFO("LOOP %s",robot.current_state.c_str());

		// if(requestToCreateTimer) {
		// 	// ROS_INFO("Loop Create Timer");
		// 	requestToCreateTimer = false;
		// 	timer = nh.createTimer(ros::Duration(10), timerCallback);
		// }
		if(navigation.requestToCreateTimer) {
			navigation.setTimer(10);
		}

	}

	ROS_INFO("Exiting Waypoint Navigation");
    return 0;
}