#include <ros/ros.h>
#include <iostream>

#include <icreate_navigation/single_navigation.h>
// #include "single_navigation.h"

//Client Service of move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool isDoneGoal;
bool isNextStep;
bool finish;
bool requestToSetupRobot;
int count;

int doneGoalNumber;

icreate::MoveBaseGoalData forward_goal;

void initializeSimpleRotateMoveBase(std::string goal_name) {
    move_base_msgs::MoveBaseGoal new_point;
    new_point.target_pose.pose.orientation.z = -1.0;
    new_point.target_pose.pose.orientation.w = 0.0;
    forward_goal.setGoal(new_point);
    forward_goal.setGoalName(goal_name);
}

void initializeSimpleForwardMoveBaseTarget(std::string goal_name) {
    //ต้องรู้จุดที่หุ่นยนต์ปัจจุบัน แล้วสั่งให้เดินไปทีละ 30 ซม. ???
    //รับค่าระยะมาจากกล้องแล้วใส่เป็น input position x
    float x_position = 0;
    // ros::ServiceClient client = nh.serverClient<std_msgs::Int16>("/get_depth_distance");
    // std_msgs::Int16 srv;
    // if(client.call(srv)) {
        // x_position = srv.response.distance;
    // }
    // else {
        // ROS_ERROR("Fail to call Service get_depth_distance");
    // }
    x_position = 2.5;
    move_base_msgs::MoveBaseGoal new_point;
    new_point.target_pose.pose.position.x = x_position;
    new_point.target_pose.pose.orientation.w = 1;
    forward_goal.setGoal(new_point);
    forward_goal.setGoalName(goal_name);
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
    single_navigation.setRobotTarget(forward_goal);
    robot.setEndPosition(forward_goal);
	single_navigation.requestToSetNewGoal = true;
	robot.sendStateRequest("SINGLERUN");
    ROS_INFO("Setup Robot: %s",forward_goal.getGoalName().c_str());
}

bool Start() {
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

bool waitUserInputLift() {
    bool flag;
    while(true) {
        std::cout << "Waiting for lift stop on the target floor" << std::endl;
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
        else {
            std::cout << "Wrong Input Please Try Again" << std::endl;
        }
    }
    return flag;
}

bool getNextStep(icreate::SingleNavigation &single_navigation,icreate::Robot &robot) {
    // if(count == 1) { 
    //     finish = true;
    //     return false;
    // }
    // setupToRunRobot(single_navigation, robot);
    // return true;
    if(forward_goal.getGoalName() == "Going To Lift") {
        initializeSimpleRotateMoveBase("Rotate In Lift");
        setupToRunRobot(single_navigation, robot);
        return true;
    }
    else if(forward_goal.getGoalName() == "Rotate In Lift") {
        bool flag = waitUserInputLift();
        if(!flag) {
            return false;
        }
        initializeSimpleForwardMoveBaseTarget("Going Out Lift");
        setupToRunRobot(single_navigation, robot);
        return true;
    }
    else {
        ROS_WARN("Warning");
        return false;
    }
}

void goalDoneCallback(const actionlib::SimpleClientGoalState &state, 
  const move_base_msgs::MoveBaseResultConstPtr &result){
      ROS_INFO("Goal Done Now !!!!!");
      //Navigation completed
    doneGoalNumber = -1;
    if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("SUCCEEDED");
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
        //หยุดแล้วแสดงว่าอาจจะเข้าลิฟต์ได้รึเปล่าก็ไม่รู้
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
	robot.sendStateRequest(single_navigation.doneRobotGoal(robot.current_state));
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
    robot.setCurrentPosition(base_frame_id, robot_frame_id, "Building 4", "Floor 20");
    // if(!waitMoveBaseServer(ac)) {
    //     return -2;
    // }
    waitMoveBaseServer(ac);

    initializeSimpleForwardMoveBaseTarget("Going To Lift");
    isDoneGoal = false;
    doneGoalNumber = -1;
    finish = false;
    count = 0;

    if(!Start())
        return -3;

    setupToRunRobot(single_navigation, robot);

    while(ros::ok() && !finish) {
        ros::spinOnce();
        r.sleep();

        if(isDoneGoal) {
			isDoneGoal = false;
			if(doneGoalNumber == 1)
			{
				// aaa(navigation, robot);
                callDoneRobotGoal(single_navigation, robot);
                isNextStep = true;
			}
		}

        if(isNextStep) {
            isNextStep = false;
            if(!getNextStep(single_navigation, robot))
                break;
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