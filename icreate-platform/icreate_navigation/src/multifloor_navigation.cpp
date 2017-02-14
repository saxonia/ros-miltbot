#include <ros/ros.h>
#include <iostream>

#include "icreate_navigation/multi_navigation.h"

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

void setGoalSucceedEvent(icreate::MultiNavigation &multi_navigation, icreate::Robot &robot) {
    ROS_INFO("SUCCEEDED %s",robot.current_state.c_str());
    multi_navigation.doneRobotGoal(robot);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "multifloor_navigation");
    ros::NodeHandle nh;

    std::string base_frame_id("map");
    std::string robot_frame_id("base_footprint");
    int polling_rate(30);
    int timer_duration(10);
    std::string move_base_topic_name("/move_base");
    

    nh.param("/waypoint_navigation/base_frame_id", base_frame_id, base_frame_id);
    nh.param("/waypoint_navigation/move_base_topic", move_base_topic_name, move_base_topic_name);
    nh.param("/waypoint_navigation/polling_rate", polling_rate, polling_rate);

    icreate::Robot robot("Building 4", "Floor 20");
    robot.setCurrentPosition(base_frame_id, robot_frame_id, "Building 4", "Floor 20");
    icreate::MultiNavigation multi_navigation;
    multi_navigation.addSingleNavigation("Building 4", "Floor 20");
    multi_navigation.addSingleNavigation("Building 4", "Floor 17");

    MoveBaseClient ac(move_base_topic_name, true);
    waitMoveBaseServer(ac, 5.0);

    ros::Rate r(polling_rate);

    multi_navigation.setupTargetQueue(robot);
    multi_navigation.verifyTarget(robot);
    multi_navigation.setupRobotToRun(robot, base_frame_id, robot_frame_id);
    isDoneGoal = false;
    doneGoalNumber = -1;
    isNextStep = false;
    multi_navigation.navigations_[multi_navigation.nav_idx].requestToCreateTimer = true;
    
    while(ros::ok()) {
        ros::spinOnce();
        r.sleep();

        if(isDoneGoal) {
			isDoneGoal = false;
			if(doneGoalNumber == 1)
			{
				setGoalSucceedEvent(multi_navigation, robot);
                isNextStep = true;
			}
            else {
                // multi_navigation.setupNavigationQueue(robot);
                // single_navigation.setupRobotToRun(robot, base_frame_id, robot_frame_id);
                // single_navigation.runRecoveryMode();
            }
		}

        if(isNextStep) {
            isNextStep = false;
            if(!multi_navigation.setNextStepMode(robot)) {
                return -3;
            }
            multi_navigation.verifyTarget(robot);
            multi_navigation.setupRobotToRun(robot, base_frame_id, robot_frame_id);
        }

        if(multi_navigation.navigations_[multi_navigation.nav_idx].requestToSetNewGoal) {
            ROS_INFO("Loop Set New Goal");
            multi_navigation.navigations_[multi_navigation.nav_idx].requestToSetNewGoal = false;
            if(multi_navigation.lift_navigation_step > 1 && multi_navigation.lift_navigation_step < 6) {
                ROS_INFO("Loop Set New Goal 1");
                multi_navigation.navigations_[multi_navigation.nav_idx].setRobotGoal(robot_frame_id);
            }
            else {
                ROS_INFO("Loop Set New Goal 2");
                multi_navigation.navigations_[multi_navigation.nav_idx].setRobotGoal(base_frame_id);
            }
            ROS_WARN("%lf",multi_navigation.navigations_[multi_navigation.nav_idx].getRobotGoal().target_pose.pose.position.x);
            ac.sendGoal(multi_navigation.navigations_[multi_navigation.nav_idx].getRobotGoal(), 
                      boost::bind(&goalDoneCallback_state, _1, _2), 
                      boost::bind(&goalActiveCallback), boost::bind(&goalFeedbackCallback, _1));
        }

        if(multi_navigation.navigations_[multi_navigation.nav_idx].requestToCreateTimer) {
			multi_navigation.navigations_[multi_navigation.nav_idx].createTimer(timer_duration);
		}
    }

    return 0;
}