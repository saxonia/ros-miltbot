#include <ros/ros.h>
#include <iostream>

#include <icreate_navigation/single_navigation.h>
// #include "single_navigation.h"

//Client Service of move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool isDoneGoal;
bool isNextStep;
bool finish;
int count;

int doneGoalNumber;

icreate::MoveBaseGoalData forward_goal;

void initializeMoveBaseTarget() {
    //ต้องรู้จุดที่หุ่นยนต์ปัจจุบัน แล้วสั่งให้เดินไปทีละ 30 ซม. ???
    //สั่งหุ่นยนต์เดินไปยาวๆ ถ้าเจอ obstacle แล้วหา path ไม่ได้ถือว่าเสร็จ ?
    move_base_msgs::MoveBaseGoal new_point;
    new_point.target_pose.pose.position.x = 0.5;
    // new_point.target_pose.pose.position.y = 0;
    // new_point.target_pose.pose.orientation.x = 0;
    // new_point.target_pose.pose.orientation.y = 0;
    // new_point.target_pose.pose.orientation.z = 0;
    new_point.target_pose.pose.orientation.w = 1.0;
    forward_goal.setGoal(new_point);
    forward_goal.setGoalName("Going to Lift");
}

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

void setupToRunRobot(icreate::SingleNavigation &single_navigation,icreate::Robot &robot) {
    // single_navigation.setRobotTarget(forward_goal.getGoal());
    robot.setEndPosition(forward_goal);
    robot.state_req_msg.data = "SINGLERUN";
	single_navigation.requestToSetNewGoal = true;
	robot.requestToSendStateReq = true;
}

bool getUserInput() {
    bool flag;
    while(true) {
        std::cout << "Start Lift Navigation" << std::endl;
        std::cout << "Please press \"y\" to start navigation " << std::endl;
        std::cout << "or Please press \"n\" to stop navigation " << std::endl;
        std::string input;
        std::cin >> input;
        if(input == "y" || input == "Y") {
            flag = true;
            break;
        }
        else if(input == "n" || input == "N") {
            flag = false;
            break;
        }
    }
    return flag;
}

void getNextStep(icreate::SingleNavigation &single_navigation,icreate::Robot &robot) {
    if(count == 0) finish = true;
    setupToRunRobot(single_navigation, robot);
    count++;
}

void goalDoneCallback(const actionlib::SimpleClientGoalState &state, 
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
		doneGoalNumber = 4;
    }

    std::cout << "Goal Finished Or Cancelled by Joy" << std::endl;
	isDoneGoal = true;

}

void goalActiveCallback(){
}

void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback){
}

void callDoneRobotGoal(icreate::SingleNavigation &single_navigation, icreate::Robot &robot) {
    ROS_INFO("SUCCEEDED %s",robot.current_state.c_str());
	robot.state_req_msg.data = single_navigation.doneRobotGoal(robot.current_state);
	ROS_INFO("Request ! %s",robot.state_req_msg.data.c_str());
	// robot.requestToSendStateReq = true;
	robot.sendStateRequest();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lift_test");
    ros::NodeHandle nh;

    icreate::Robot robot;
    icreate::SingleNavigation single_navigation;

    std::string move_base_topic("/move_base");
    int polling_rate(30);
    std::string base_frame_id("map");
    std::string robot_frame_id("base_footprint");
    nh.param("move_base_topic", move_base_topic, move_base_topic);
    nh.param("polling_rate", polling_rate, polling_rate);
    MoveBaseClient ac(move_base_topic, true);

    // Callback polling rate
    ros::Rate r(polling_rate);

    // if(!robot.setCurrentPosition(base_frame_id, robot_frame_id)) {
    //     return -1;
    // }
    robot.setCurrentPosition(base_frame_id, robot_frame_id);
    // if(!waitMoveBaseServer(ac)) {
    //     return -2;
    // }
    waitMoveBaseServer(ac);

    initializeMoveBaseTarget();
    isDoneGoal = false;
    doneGoalNumber = -1;
    finish = false;
    count = 0;

    if(!getUserInput())
        return -3;

    setupToRunRobot(single_navigation, robot);

    while(ros::ok() && !finish) {
        ros::spinOnce();
        r.sleep();

        if(robot.requestToSendStateReq) {
            robot.sendStateRequest();
        }
        ROS_INFO("ffff2");
        if(isDoneGoal) {
			isDoneGoal = false;
			if(doneGoalNumber == 1)
			{
				// aaa(navigation, robot);
                callDoneRobotGoal(single_navigation, robot);
			}
			isNextStep = true;
		}

        if(isNextStep) {
            isNextStep = false;
            getNextStep(single_navigation, robot);
            robot.sendStateRequest();
        }

        if(single_navigation.requestToSetNewGoal) {
            ROS_INFO("Loop Set New Goal");
            single_navigation.requestToSetNewGoal = false;
            single_navigation.setRobotGoal("/base_footprint");
            ac.sendGoal(single_navigation.goal, 
                boost::bind(&goalDoneCallback, _1, _2), 
                boost::bind(&goalActiveCallback), 
                boost::bind(&goalFeedbackCallback, _1));
        }

        if(single_navigation.requestToCreateTimer) {
			single_navigation.setTimer(10);
		}
    }

    return 0;
}