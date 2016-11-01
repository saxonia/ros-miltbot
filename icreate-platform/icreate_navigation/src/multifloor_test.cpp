#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include "robot.h"
#include "navigation.h"

icreate::Robot robot;
icreate::Navigation navigation;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv) {

    ros::init(argc, argv, "multifloor_test");

    MoveBaseClient ac("/icreate/move_base", true);

    robot.setCurrentPosition();

    bool requestToCreateTimer = true;

    ros::Rate r(30);

    return 0;
}