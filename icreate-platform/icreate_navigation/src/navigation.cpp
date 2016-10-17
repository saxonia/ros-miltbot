#include "navigation.h"

namespace icreate {

Navigation::Navigation() {
    // ac("move_base", true);
}

Navigation::~Navigation() {

}

void Navigation::setRobotGoal(std::string frame_id) {
    //frame_id = "/map"
    goal.target_pose.header.frame_id = frame_id;
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x    = target->target_pose.pose.position.x;
    goal.target_pose.pose.position.y    = target->target_pose.pose.position.y;
    goal.target_pose.pose.orientation.x = target->target_pose.pose.orientation.x;
    goal.target_pose.pose.orientation.y = target->target_pose.pose.orientation.y;
    goal.target_pose.pose.orientation.z = target->target_pose.pose.orientation.z;
    goal.target_pose.pose.orientation.w = target->target_pose.pose.orientation.w;
    std::cout << "[AGENT] SET NEW GOAL ! " << std::endl;


}

move_base_msgs::MoveBaseGoal Navigation::getRobotGoal() {
    return goal;
}

std::string Navigation::doneRobotGoal(std::string robot_state, bool &finish) {
    std::string state_msg; 
    ROS_INFO("The goal was reached!");
    if(robot_state == "GOING") {
        state_msg = "WAITING";
        // ----------------------------------- Not Implemented Now ------------------------ //
        // waitfordelivery();
    }
    else if(robot_state == "BACKTOBASE") {
        state_msg = "IDLE";
        finish = true;  
    }
    return state_msg;
}

std::string Navigation::activeRobotGoal(std::string robot_state) {
    std::string msg;
    ROS_INFO("Goal active! Now !!");
    if(robot_state == "IDLE") {
        msg = "GOING";
    }
    else if(robot_state == "WAITING") {
        msg = "BACKTOBASE";
    }
    return msg;
}

void Navigation::getFeedbackRobotGoal() {
    ROS_INFO("Getting feedback! How cool is that?");
}

void Navigation::read_waypoint_constant() {
    move_base_msgs::MoveBaseGoal newPoint;
    newPoint.target_pose.pose.position.x    = -6.326;
    newPoint.target_pose.pose.position.y    = -0.674;
    newPoint.target_pose.pose.orientation.x = 0.000;
    newPoint.target_pose.pose.orientation.y = 0.000;
    newPoint.target_pose.pose.orientation.z = -2.870;
    newPoint.target_pose.pose.orientation.w = 0.958;
    targets.push_back(newPoint);

    target = targets.begin();    
}

}