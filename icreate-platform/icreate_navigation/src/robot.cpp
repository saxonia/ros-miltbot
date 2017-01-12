#include "icreate_navigation/robot.h"

namespace icreate {

MoveBaseGoalData::MoveBaseGoalData(void) {

}

MoveBaseGoalData::MoveBaseGoalData(move_base_msgs::MoveBaseGoal goal, std::string goal_name) {

}

MoveBaseGoalData::~MoveBaseGoalData(void) {}

void MoveBaseGoalData::setGoal(move_base_msgs::MoveBaseGoal &goal) {
    this->goal = goal;
}

move_base_msgs::MoveBaseGoal MoveBaseGoalData::getGoal() {
    return this->goal;
}

void MoveBaseGoalData::setGoalName(std::string goal_name) {
    this->goal_name = goal_name;
}

std::string MoveBaseGoalData::getGoalName() {
    return this->goal_name;
}

Robot::Robot(void) {
    // Subsribe Robot State Topic
    ROS_INFO("Create Robot Class");
    client = nh_.serviceClient<icreate_state::SetRobotState>(set_robot_state_service);
    current_state = "IDLE";
}

Robot::~Robot(void) {

}

bool Robot::setCurrentPosition(std::string base_frame_id, std::string robot_frame_id) {
    tf::TransformListener listener;  
    tf::StampedTransform transform;
    ros::Time now = ros::Time::now();
    try {
        ROS_INFO("WAITING FOR TRANSFORM FRAME");
        listener.waitForTransform(base_frame_id, robot_frame_id, ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform(base_frame_id, robot_frame_id, ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        return false;
    }

    this->startPosition.goal_name = "Current Position"; 
    this->startPosition.goal.target_pose.pose.position.x    = transform.getOrigin().x();
    this->startPosition.goal.target_pose.pose.position.y    = transform.getOrigin().y();
    this->startPosition.goal.target_pose.pose.orientation.x = transform.getRotation().x();
    this->startPosition.goal.target_pose.pose.orientation.y = transform.getRotation().y();
    this->startPosition.goal.target_pose.pose.orientation.z = transform.getRotation().z();
    this->startPosition.goal.target_pose.pose.orientation.w = transform.getRotation().w();

    std::cout << "RETURNING POSITION MARKED : ";
    std::cout << this->startPosition.goal.target_pose.pose.position.x << "," 
            << this->startPosition.goal.target_pose.pose.position.y << " at Time: " << now <<std::endl;
    return true;
}

// move_base_msgs::MoveBaseGoal Robot::getCurrentPosition() {
//     return currentPosition;
// }

 MoveBaseGoalData Robot::getCurrentPosition() {
    return currentPosition;
}

// bool Robot::setEndPosition(move_base_msgs::MoveBaseGoal goal) {
//     endPosition = goal;
//     ros::Time now = ros::Time::now();

//     std::cout << "RETURNING POSITION MARKED : ";
//     std::cout << endPosition.target_pose.pose.position.x << "," 
//             << endPosition.target_pose.pose.position.y << " at Time: " << now <<std::endl;

//     return true;
// }

bool Robot::setEndPosition(MoveBaseGoalData data) {
    this->endPosition = data;
    ros::Time now = ros::Time::now();

    std::cout << "RETURNING POSITION MARKED : ";
    std::cout << endPosition.goal.target_pose.pose.position.x << "," 
            << endPosition.goal.target_pose.pose.position.y << " at Time: " << now <<std::endl;

    return true;
}

void Robot::sendStateRequest(std::string state_request) {
    ROS_INFO("Send State Request: %s",state_request.c_str());
    // ros::ServiceClient client = nh_.serviceClient<icreate_state::SetRobotState>(set_robot_state_service);
    icreate_state::SetRobotState srv;
    state_req_msg.data = state_request;
    srv.request.req = state_request;
    if (this->client.call(srv))
    {
      current_state = srv.response.res;
      
    }
    else
    {
      ROS_ERROR("Failed to call service set_robot_state");
    }
}

}