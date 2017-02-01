#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

// #include "robot.h"
#include "icreate_navigation/single_navigation.h"
// #include <icreate_navigation/single_navigation.h>

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
	// Wait for the action server to come up
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

void goalActiveCallback(icreate::SingleNavigation &navigation, icreate::Robot &robot){
    // robot.state_req_msg.data = navigation.activeRobotGoal(robot.current_state, state_req);
	// robot.requestToSendStateReq = true;
}

void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback){
    // navigation.getFeedbackRobotGoal();
	// ROS_INFO("FEEDBACK %s",robot.current_state.c_str());
}

void runControlLoop(icreate::SingleNavigation &navigation, icreate::Robot robot) {

}

void aaa(icreate::SingleNavigation &navigation, icreate::Robot &robot) {
	ROS_INFO("SUCCEEDED %s",robot.current_state.c_str());
	robot.sendStateRequest(navigation.doneRobotGoal(robot.current_state));
}

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "waypoint_navigation");
	ros::NodeHandle nh;

	//Class
	icreate::Robot robot("B", "A");
	icreate::SingleNavigation navigation;

	std::string move_base_topic_name("/move_base");
    std::string waypoint_file_path("/waypoint/build4_f20.csv");
    std::string base_frame_id("/map");
    std::string package_name("icreate_navigation");
	int polling_rate(30);
    nh.param("/waypoint_navigation/move_base_topic", move_base_topic_name, move_base_topic_name);
    nh.param("/waypoint_navigation/waypoint_file_path", waypoint_file_path, waypoint_file_path);
    nh.param("/waypoint_navigation/base_frame_id", base_frame_id, base_frame_id);
    nh.param("/waypoint_navigation/package_name", package_name, package_name); 
	nh.param("/waypoint_navigation/polling_rate", polling_rate, polling_rate);
    MoveBaseClient ac(move_base_topic_name, true);

	// Callback polling Rate 
    ros::Rate r(polling_rate);

	//เปลี่ยนเป็นรับ waypoint & lift มาจาก waypoint_server ????
	navigation.readWaypointFile(package_name, waypoint_file_path);
    // if(!robot.setCurrentPosition()) {
    //     return -1;
    // }
	robot.setCurrentPosition("map", "base_footprint", "Building 4", "Floor 20");

    // Wait for the action server to come up
    // if(!waitMoveBaseServer(ac)) {
	// 	return -1;
	// }
	waitMoveBaseServer(ac);

	// Point The iterator to the beginning of the sequence
    navigation.target_iterator = navigation.targets.begin();
	
	// Request To Create timer
    // requestToCreateTimer = true;

	isDoneGoal = false;
	doneGoalNumber = 0;
    // requestToSetNewGoal = false;

    // Ask User For Input
    // getUserInput(navigation, robot);
	//Remember This Location as startPoint
	navigation.getUserInput(robot, "map", "base_footprint");

	// Start the Navigation Waypoint Loop
	while(ros::ok() && !finish) {
		ros::spinOnce();
		r.sleep();

		// if(robot.requestToSendStateReq) {
			// ROS_INFO("Stateee: %s",  robot.state_req_msg.data.c_str());
			// robot.sendStateRequest();
		// }

		if(isDoneGoal)
		{
			isDoneGoal = false;
			if(doneGoalNumber == 1)
			{
				aaa(navigation, robot);
			}
			isNextStep = true;
		}

		if(isNextStep) {
			ROS_INFO("Loop Done Goal");
			isNextStep = false;
			navigation.getNextStep(robot, "map", "base_footprint");
		}

		if(navigation.requestToSetNewGoal) {
		// if(navigation.requestToSetNewGoal && verifyActiveState(robot)) {
			ROS_INFO("Loop Set New Goal");
			navigation.requestToSetNewGoal = false;

			navigation.setRobotGoal("/map");
			ROS_INFO("Stateee: %s",  robot.current_state.c_str());
			ac.sendGoal(navigation.goal, 
                      boost::bind(&goalDoneCallback_state, _1, _2), 
                      boost::bind(&goalActiveCallback, navigation, robot), boost::bind(&goalFeedbackCallback, _1));
		}
		if(navigation.requestToCreateTimer) {
			navigation.setTimer(10);
		}
	}
	ROS_INFO("Exiting Waypoint Navigation");
    return 0;
}