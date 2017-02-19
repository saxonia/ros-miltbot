#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include "icreate_navigation/single_navigation.h"

//Client Service of move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool isDoneGoal;
int doneGoalNumber;
bool isNextStep;

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

/// CALLBACKS Function

void goalDoneCallback_state(const actionlib::SimpleClientGoalState &state, 
  const move_base_msgs::MoveBaseResultConstPtr &result){
    ROS_INFO("Goal Done Now !!!!!");
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
      	ROS_WARN("ABORTED : Failed to reach the goal...");
		// robot.state_req_msg.data = navigation.failRobotGoal(robot.current_state, finish);
		// input_mode = 3;
		doneGoalNumber = 4;
    }

    std::cout << "Goal Finished Or Cancelled by Joy" << std::endl;
	// robot.requestToSendStateReq = true;
	isDoneGoal = true;
}

void goalActiveCallback(){

}

void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback){
}

void setGoalSucceedEvent(icreate::SingleNavigation &single_navigation, icreate::Robot &robot) {
    ROS_INFO("SUCCEEDED %s",robot.current_state.c_str());
    single_navigation.doneRobotGoal(robot);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_navigation_new");
    ros::NodeHandle nh;

    std::string move_base_topic_name("/move_base");
    // std::string base_frame_id("/map");
    std::string base_frame_id("map");
    std::string robot_frame_id("base_footprint");
    std::string package_name("icreate_navigation");
    std::string building_name("Building 4");
    std::string building_floor_name("Floor 20");
	int polling_rate(30);
    int timer_duration(10);
    nh.param("/waypoint_navigation/move_base_topic", move_base_topic_name, move_base_topic_name);
    nh.param("/waypoint_navigation/base_frame_id", base_frame_id, base_frame_id);
    nh.param("/waypoint_navigation/robot_frame_id", robot_frame_id, robot_frame_id);
    nh.param("/waypoint_navigation/package_name", package_name, package_name); 
	nh.param("/waypoint_navigation/polling_rate", polling_rate, polling_rate);
    nh.param("/waypoint_navigation/timer_duration", timer_duration, timer_duration);
    nh.param("/waypoint_navigation/building", building_name, building_name);
    nh.param("/waypoint_navigation/building_floor", building_floor_name, building_floor_name);

    //Initialize Class
    icreate::SingleNavigation single_navigation(building_name, building_floor_name);
    icreate::Robot robot(building_name, building_floor_name, base_frame_id, robot_frame_id);
    
    MoveBaseClient ac(move_base_topic_name, true);

    // Callback polling Rate 
    ros::Rate r(polling_rate);
    waitMoveBaseServer(ac, 5.0);

    single_navigation.start(robot);
    single_navigation.setupRobotToRun(robot, base_frame_id, robot_frame_id);
    isDoneGoal = false;
    doneGoalNumber = -1;
    isNextStep = false;
    single_navigation.requestToCreateTimer = true;
    while(ros::ok()) {
        ros::spinOnce();
        r.sleep();

        if(isDoneGoal) {
			isDoneGoal = false;
			if(doneGoalNumber == 1)
			{
				setGoalSucceedEvent(single_navigation, robot);
                isNextStep = true;
			}
            else {
                single_navigation.start(robot);
                single_navigation.setupRobotToRun(robot, base_frame_id, robot_frame_id);
                // single_navigation.runRecoveryMode();
            }
		}

        if(isNextStep) {
            isNextStep = false;
            if(!single_navigation.update(robot)) {
                return -3;
            }
            single_navigation.setupRobotToRun(robot, base_frame_id, robot_frame_id);
        }

        if(single_navigation.requestToSetNewGoal) {
            ROS_INFO("Loop Set New Goal");
            single_navigation.requestToSetNewGoal = false;
            single_navigation.setRobotGoal(base_frame_id);
            // isDoneGoal = true;
            // doneGoalNumber = 1;
            ac.sendGoal(single_navigation.getRobotGoal(), 
                      boost::bind(&goalDoneCallback_state, _1, _2), 
                      boost::bind(&goalActiveCallback), boost::bind(&goalFeedbackCallback, _1));
        }

        if(single_navigation.requestToCreateTimer) {
			single_navigation.createTimer(timer_duration);
		}

    }
    ROS_INFO("Exiting Waypoint Navigation");
    return 0;
}