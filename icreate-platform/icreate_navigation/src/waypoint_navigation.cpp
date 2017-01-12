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

// void getUserInput(icreate::SingleNavigation &single_navigation, icreate::Robot &robot) {
// 	int selected_point = -1;
//     while(true) {
//         single_navigation.displayWaypoints();
//         single_navigation.setNavigationMode(-1);

//         // Ask for ID and wait user input
// 	    std::cout << "[AGENT] Input Target Waypoints ID : " ;
// 	    std::cin >> selected_point;

//         if(selected_point > -1 && selected_point < single_navigation.targets.size()){ 
//             // Select Mode for Transportation 
// 		    std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;
// 		    std::cout << "[AGENT] Select Mode" <<std::endl;
// 		    std::cout << "[ 0 ] Go to Specific Point." <<std::endl;
// 		    std::cout << "[ 1 ] Delivery and Come Back to This Place." <<std::endl;
// 		    std::cout << "[ 2 ] Delivery and Come Back to Base Station" <<std::endl;
// 			std::cout << "[ 3 ] Execute The Memorized Sequence" <<std::endl;
// 		    std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;
// 		    std::cout << "Select Mode[0,1,2] : " ;
// 			int input; 
// 		    std::cin  >> input;
// 			single_navigation.setNavigationMode(input); 
// 		    std::cout << "You Selected : " << single_navigation.getNavigationMode() <<std::endl;
// 		    std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;

//             // Do Action depends on the Mode selected
// 		    switch(single_navigation.getNavigationMode()){
// 		    	//MODE : Go to Specific Point
// 			    	case 0:
//                         //Set the Robot State 
//                         robot.state_req_msg.data = "SINGLERUN";
// 			    		//Set New Goal
// 			    		single_navigation.setRobotTarget(selected_point);
// 						robot.setEndPosition(single_navigation.getRobotTarget());
// 			    		single_navigation.requestToSetNewGoal = true;
// 						robot.requestToSendStateReq = true;
// 						// finish = false;
// 			    	    return;
// 		    	//MODE : Delivery and Come back to this place
// 			    	case 1 :
// 			    		//Remember This Location as startPoint
// 			    		robot.setCurrentPosition();
// 			    		//Set the Robot State 
//                         robot.state_req_msg.data = "GOING";
// 			    		//Set the endPoint to go
// 			      		single_navigation.setRobotTarget(selected_point);
// 						robot.setEndPosition(single_navigation.getRobotTarget());
// 			      		single_navigation.requestToSetNewGoal = true;
// 						robot.requestToSendStateReq = true;
// 			    		return;
// 				//MODE : Delivery and Come back to base
// 			    	case 2:
// 			    		//Remember This Location as startPoint
// 			    		robot.setCurrentPosition();
// 			    		//Set the Robot State 
//                         robot.state_req_msg.data = "GOING";
// 			    		//Set the endPoint to go
// 			      		single_navigation.setRobotTarget(selected_point);
// 						robot.setEndPosition(single_navigation.getRobotTarget());
// 			      		single_navigation.requestToSetNewGoal = true;
// 						robot.requestToSendStateReq = true;
// 			    		return;
// 			  	//MODE : Execute the Sequence
// 					case 3:
// 						//Set the Robot State 
// 						robot.state_req_msg.data = "EXECUTESEQ";
// 			    		//Set Target to Next Sequence 
// 						single_navigation.setRobotTarget(single_navigation.sequence[single_navigation.targetId]);
// 						robot.setEndPosition(single_navigation.getRobotTarget());
// 			    		//Set the new Goal
// 			    		single_navigation.requestToSetNewGoal = true;
// 						robot.requestToSendStateReq = true;
// 			      		return;
// 			    //MODE : EXIT
// 					case 99:
// 						//Set the Robot State 
// 			      		robot.state_req_msg.data = "IDLE";
// 						single_navigation.requestToSetNewGoal = false;
// 						robot.requestToSendStateReq = false;
// 			      		// finish = true;
// 			      		return;
// 			    //Not Specified : Do Nothing ^_^
// 		      	default:
// 		      		robot.state_req_msg.data = "IDLE";
// 		      		std::cout << "[AGENT]You Selected NOTHING" <<std::endl;
// 		      		single_navigation.setNavigationMode(-1);
// 					single_navigation.requestToSetNewGoal = false;
// 					robot.requestToSendStateReq = false;
// 					return;
// 		    }//Mode Select
//         }   
//     }
// }

bool verifyActiveState(icreate::Robot &robot) {
	if(robot.current_state == "GOING")
		return true;
	else if(robot.current_state == "BACKTOBASE")
		return true;
	else if(robot.current_state == "SENDSUPPLIES")
		return true;
	else if(robot.current_state == "SINGLERUN")
		return true;
	else if(robot.current_state == "EXECUTEQ")
		return true;
	else
		return false;
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
	robot.state_req_msg.data = navigation.doneRobotGoal(robot.current_state);
	ROS_INFO("aaa Request ! %s",robot.state_req_msg.data.c_str());
	robot.sendStateRequest(robot.state_req_msg.data);
}

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "waypoint_navigation");
	ros::NodeHandle nh;

	//Class
	icreate::Robot robot;
	icreate::SingleNavigation navigation;

	std::string move_base_topic_name("/move_base");
    std::string lift_file_path("/waypoint/build4_f20l.csv");
    std::string base_frame_id("/map");
    std::string package_name("icreate_navigation");
    nh.param("/waypoint_navigation/move_base_topic", move_base_topic_name, move_base_topic_name);
    nh.param("/waypoint_navigation/lift_file_path", lift_file_path, lift_file_path);
    nh.param("/waypoint_navigation/base_frame_id", base_frame_id, base_frame_id);
    nh.param("/waypoint_navigation/package_name", package_name, package_name); 
    MoveBaseClient ac(move_base_topic_name, true);

	// Callback polling Rate 
    ros::Rate r(30);

	navigation.readWaypointFile(package_name, lift_file_path);
    // if(!robot.setCurrentPosition()) {
    //     return -1;
    // }
	robot.setCurrentPosition("map", "base_footprint");

    // Wait for the action server to come up
    // if(!waitMoveBaseServer(ac)) {
	// 	return -1;
	// }
	waitMoveBaseServer(ac);

	// Point The iterator to the beginning of the sequence
    navigation.targets_iterator = navigation.targets.begin();
	
	// Request To Create timer
    // requestToCreateTimer = true;

	// requestToSendStateReq = false;
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