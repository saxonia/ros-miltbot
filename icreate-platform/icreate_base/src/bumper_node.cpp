#include <ros/ros.h>
#include <iostream>

#include <create_node/TurtlebotSensorState.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include "miltbot_common/Waypoint.h"
#include "miltbot_system/RunSystem.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::ServiceClient run_system_client;
ros::Publisher move_base_cancel_pub;


void initializeSimpleBackwardMoveBase(std::string frame_id) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.pose.position.x = -0.5;
    goal.target_pose.pose.orientation.w = 1.0;
    goal.target_pose.header.frame_id = frame_id;
    goal.target_pose.header.stamp = ros::Time::now();
    MoveBaseClient ac("move_base", true);
    ac.sendGoal(goal);
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, the base moved 1 meter forward");
    else
      ROS_INFO("The base failed to move forward 1 meter for some reason");
}

void initializeSimpleRotateMoveBase(std::string frame_id) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.pose.orientation.z = 0.5;
    goal.target_pose.pose.orientation.w = 1;
    goal.target_pose.header.frame_id = frame_id;
    goal.target_pose.header.stamp = ros::Time::now();
    MoveBaseClient ac("move_base", true);
    ac.sendGoal(goal);
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, the base moved 1 meter forward");
    else
      ROS_INFO("The base failed to move forward 1 meter for some reason");
}

void callRunSystemService(bool status) {
    miltbot_system::RunSystem srv;
    srv.request.status = status;
    if(run_system_client.call(srv)) {
        bool flag = srv.response.success;
    }
    else {
        ROS_ERROR("Failed to call service run_system");
    }
}

void turtlebotSensorCallback(const create_node::TurtlebotSensorState &msg) {
    // ROS_DEBUG_STREAM(msg);
    ROS_INFO("Bump Wheel Drops : %d",msg.bumps_wheeldrops);
    if(msg.bumps_wheeldrops > 0) {
        callRunSystemService(false);
        move_base_cancel_pub.publish(*new actionlib_msgs::GoalID());
        initializeSimpleBackwardMoveBase("base_footprint");
        initializeSimpleRotateMoveBase("base_footprint");
        callRunSystemService(true);
    }
}

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "bumper_node");
    ros::NodeHandle nh;

    std::string turtlebot_state_sub_topic_name("icreate_node/sensor_state");
    std::string move_base_cancel_pub_topic_name("move_base/cancel");
    std::string run_system_service_name("run_system");

    nh.param("turtlebot_state_sub_topic", turtlebot_state_sub_topic_name, turtlebot_state_sub_topic_name);
    nh.param("move_base_cancel_pub_topic", move_base_cancel_pub_topic_name, move_base_cancel_pub_topic_name);
    nh.param("run_system_service", run_system_service_name, run_system_service_name);


    ros::Subscriber turtlebot_sensor_sub = nh.subscribe(turtlebot_state_sub_topic_name, 10, turtlebotSensorCallback);
    ros::Publisher move_base_cancel_pub = nh.advertise<actionlib_msgs::GoalID>(move_base_cancel_pub_topic_name, 1);
    ros::ServiceClient run_system_client = nh.serviceClient<miltbot_system::RunSystem>(run_system_service_name);
    ros::spin();
    return 0;
}