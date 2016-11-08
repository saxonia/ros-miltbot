#include "robot.h"

namespace icreate {

Robot::Robot(void) {
    // Subsribe Robot State Topic
    ROS_INFO("Create Robot Class");
    // state_sub = nh_.subscribe("/icreate/state",100,&Robot::stateCallback, this);
    // state_req_pub = nh_.advertise<std_msgs::String>("/icreate/state_req",10);
    // requestToSendStateReq = false;
}

Robot::~Robot(void) {

}


bool Robot::setCurrentPosition() {
    tf::TransformListener listener;  
    tf::StampedTransform transform;
    ros::Time now = ros::Time::now();
    try {
        ROS_INFO("WAITING FOR TRANSFORM FRAME");
        listener.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("map", "base_footprint", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        return false;
    }

    startPosition.target_pose.pose.position.x    = transform.getOrigin().x();
    startPosition.target_pose.pose.position.y    = transform.getOrigin().y();
    startPosition.target_pose.pose.orientation.x = transform.getRotation().x();
    startPosition.target_pose.pose.orientation.y = transform.getRotation().y();
    startPosition.target_pose.pose.orientation.z = transform.getRotation().z();
    startPosition.target_pose.pose.orientation.w = transform.getRotation().w();

    std::cout << "RETURNING POSITION MARKED : ";
    std::cout << startPosition.target_pose.pose.position.x << "," 
            << startPosition.target_pose.pose.position.y << " at Time: " << now <<std::endl;
    return true;
}

move_base_msgs::MoveBaseGoal Robot::getCurrentPosition() {
    return currentPosition;
}

bool Robot::setEndPosition(move_base_msgs::MoveBaseGoal goal) {
    endPosition = goal;
    ros::Time now = ros::Time::now();

    std::cout << "RETURNING POSITION MARKED : ";
    std::cout << endPosition.target_pose.pose.position.x << "," 
            << endPosition.target_pose.pose.position.y << " at Time: " << now <<std::endl;

    return true;
}

void Robot::sendStateRequest() {
    ROS_INFO("Loop Send State Request");
	requestToSendStateReq = false;
	state_req_pub.publish(state_req_msg);
}

// STATE CALLBACK
void Robot::stateCallback(const std_msgs::String::ConstPtr& msg) {
    current_state = msg->data;
	ROS_INFO("Robot state: %s",current_state.c_str());
}

}