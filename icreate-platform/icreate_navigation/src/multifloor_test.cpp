#include <ros/ros.h>
// #include <actionlib/client/simple_action_client.h>
// #include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

// #include "robot.h"
#include <iostream>
#include <string>
#include "icreate_navigation/single_navigation.h"



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

icreate::MoveBaseGoalData target_goal;

void initializeWaitLiftPosition(std::string goal_name) {
    move_base_msgs::MoveBaseGoal new_point;
    new_point.target_pose.pose.position.x    = 0.327;
    new_point.target_pose.pose.position.y    = 1.634;
    new_point.target_pose.pose.orientation.x = 0.000;
    new_point.target_pose.pose.orientation.y = 0.000;
    new_point.target_pose.pose.orientation.z = -0.686;
    new_point.target_pose.pose.orientation.w = 0.727;
    target_goal.setGoal(new_point);
    target_goal.setGoalName(goal_name);
}

bool waitMoveBaseServer(MoveBaseClient &ac, float wait_duration) {
	// Wait for the action server to come up
    int tries = 0;
    while(!ac.waitForServer(ros::Duration(wait_duration))){
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

// void setLiftPosition(icreate::SingleNavigation &navigation, int liftNumber) {
//     navigation.lift = navigation.lifts.begin() + liftNumber;
//     navigation.target_iterator = navigation.lift;
//     ROS_INFO("set Robot Lift: %s", navigation.lift_name[liftNumber].c_str());
//     requestToSetNewGoal = true;
// }

void setupToRunRobot(icreate::SingleNavigation &single_navigation,icreate::Robot &robot) {
    single_navigation.setRobotTarget(target_goal);
    robot.setEndPosition(target_goal);
	// single_navigation.requestToSetNewGoal = true;
	robot.sendStateRequest("SINGLERUN");
    ROS_INFO("Setup Robot: %s",target_goal.getGoalName().c_str());
}

bool userInput(icreate::Robot &robot, icreate::SingleNavigation &navigation) {
    std::cout << "Press Any Key To Start Navigation Or ESC To Exit" << std::endl;
    std::string key;
    std::cin >> key;
    
    robot.setCurrentPosition("map", "base_footprint", "Building 4", "Floor 20");
    
    
    // navigation.targets.push_back(newPoint);
    // navigation.target_iterator = navigation.targets.begin(); 
    
    //End ---------------------------------
    // icreate::MoveBaseGoalData data(navigation.targets[0],navigation.target_name[0]);
	// robot.setEndPosition(data);
	requestToSetNewGoal = true;
    // robot.state_req_msg.data = "SINGLERUN";
	// robot.requestToSendStateReq = true;
    robot.sendStateRequest("SINGLERUN");
    return true;
    // I'm not sure about ascii code == 0 what it is.
}

void nextStep(icreate::Robot &robot,icreate::SingleNavigation &navigation) {
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
    // setLiftPosition(navigation, std::stoi(liftNumber));
    // icreate::MoveBaseGoalData data(navigation.lifts[std::stoi(liftNumber)], navigation.lift_name[std::stoi(liftNumber)]);
    // robot.setEndPsdosition(data);
    ROS_INFO("Get");
    // robot.requestToSendStateReq = true;
}

void setFrontLiftToLift(icreate::Robot &robot,icreate::SingleNavigation &navigation) {

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
}

void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback){
    // navigation.getFeedbackRobotGoal();
}

void aaa(icreate::SingleNavigation &navigation, icreate::Robot &robot) {
	ROS_INFO("SUCCEEDED %s",robot.current_state.c_str());
    navigation.doneRobotGoal(robot);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "multifloor_test");
    ros::NodeHandle nh;

    std::string move_base_topic_name("/move_base");
    std::string lift_file_path("/waypoint/build4_f20lift.csv");
    std::string base_frame_id("map");
    std::string robot_frame_id("base_footprint");
    std::string package_name("icreate_navigation");
    std::string building_name("Building 4");
    std::string building_floor_name("Floor 20");
    int polling_rate(30);
    nh.param("move_base_topic", move_base_topic_name, move_base_topic_name);
    nh.param("lift_file_path", lift_file_path, lift_file_path);
    nh.param("base_frame_id", base_frame_id, base_frame_id);
    nh.param("package_name", package_name, package_name);
    nh.param("polling_rate", polling_rate, polling_rate);

    // Class
    icreate::Robot robot(building_name, building_floor_name, base_frame_id, robot_frame_id);
    icreate::SingleNavigation navigation(building_name, building_floor_name);

    MoveBaseClient ac(move_base_topic_name, true);

    ros::Rate r(polling_rate);

    waitMoveBaseServer(ac, 5.0);

	isDoneGoal = false;
    isNextStep = false;
    doneGoalNumber = -1;

    initializeWaitLiftPosition("Lift Position");

    userInput(robot, navigation); 

    setupToRunRobot(navigation, robot); 
    navigation.createTimer(10);  

    while(ros::ok()) {
        ros::spinOnce();
        r.sleep();

        // if(requestToSendStateReq) {
        //     ROS_INFO("Loop Send State Request");
        //             requestToSendStateReq = false;
        //             state_req_pub.publish(state_req_msg);
        // }
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
			// robot.requestToSendStateReq = true;
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
            navigation.setRobotGoal(base_frame_id);
            ac.sendGoal(navigation.getRobotGoal(), 
                        boost::bind(&goalDoneCallback_state, _1, _2), 
                        boost::bind(&goalActiveCallback), boost::bind(&goalFeedbackCallback, _1));
        }
    }

    ROS_INFO("Exiting MultiFloor Test Navigation");
    return 0;
}