#include "navigation.h"

namespace icreate {

Navigation::Navigation() {
    this->ac("move_base", true);
    navigation_mode = -1;
    // client = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    // requestToCreateTimer = true;
}

Navigation::~Navigation() {

}

void Navigation::setRobotTarget(move_base_msgs::MoveBaseGoal goal) {
    targets.push_back(goal);
    target = targets.end() - 1;
}

void Navigation::setRobotTarget(int selected_point) {
    Navigation::selected_point = selected_point;
    // *target = targets[selected_point];
    target = targets.begin() + selected_point;
    ROS_WARN("SElect Point: %d %lf",selected_point, (*target).target_pose.pose.position.x);
}

move_base_msgs::MoveBaseGoal Navigation::getRobotTarget() {
    ROS_WARN("get Robot Target: %d",selected_point);
    return targets[selected_point];
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

std::string Navigation::doneRobotGoal(std::string robot_state, int &mode) {
    std::string state_msg; 
    ROS_INFO("The goal was reached!");
    if(robot_state == "GOING") {
        state_msg = "WAITING";
        mode = 1;
    }
    else if(robot_state == "WAITING") {
        
    }
    else if(robot_state == "BACKTOBASE") {
        state_msg = "IDLE";
        mode = 0;
    }
    else if(robot_state == "SINGLERUN") {
        state_msg = "IDLE";
        mode = 0;
    }
    else if(robot_state == "EXECUTESEQ") {
        state_msg = "IDLE";
        mode = 2;
    }
    return state_msg;
}

std::string Navigation::failRobotGoal(std::string robot_state, bool &finish) {
    std::string state_msg;
    //  if(robot_state == "GOING") {

    //  }
    //  else if(robot_state == "SINGLERUN") {

    //  }
    //  else if(robot_state == "EXECUTESEQ") {
    //  }
         
    state_msg = "IDLE";
    return state_msg;
}

std::string Navigation::activeRobotGoal(std::string robot_state, std::string state_req) {
    std::string msg;
    ROS_INFO("Goal active! Now !!");
    if(robot_state == "IDLE" && state_req == "SINGLERUN") {
        msg = "SINGLERUN";
    }
    else if(robot_state == "IDLE" && state_req == "GOING") {
        msg = "GOING";
    }
    else if(robot_state == "IDLE" && state_req == "EXECUTESEQ") {
        msg = "EXECUTESEQ";
    }
    else if(robot_state == "WAITING") {
        msg = "BACKTOBASE";
    }
    // else {
    //     msg = "IDLE";
    // }
    return msg;
}

void Navigation::getFeedbackRobotGoal() {
    ROS_INFO("Getting feedback! How cool is that?");
}

void Navigation::readWaypointConstant() {
    move_base_msgs::MoveBaseGoal newPoint;
    newPoint.target_pose.pose.position.x    = 7.687;
    newPoint.target_pose.pose.position.y    = 14.260;
    newPoint.target_pose.pose.orientation.x = 0.000;
    newPoint.target_pose.pose.orientation.y = 0.000;
    newPoint.target_pose.pose.orientation.z = 0.552;
    newPoint.target_pose.pose.orientation.w = 0.834;
    targets.push_back(newPoint);

    target = targets.begin();    
}

void Navigation::readWaypointFile(std::string filename) {
    std::vector<std::string> tokenized;
    std::string path = ros::package::getPath("icreate_navigation")+ filename;
    std::ifstream inFile(path.c_str());
    std::string line;   
    //Prompt to User
    std::cout << "[POINT_READER]IMPORTING Point of interest !" <<std::endl;     
    //First Value is Waypoint Counts 
    getline(inFile,line);
    int waypoint_count = toint(line);
    std::cout << "[POINT_READER]Points Counted : " << waypoint_count <<std::endl;

    //Read POI Line By Line 
    while(getline(inFile,line)){
     // New Line
       std::stringstream strstr(line);
       std::string word = "";
     // Ignore First Param (Place's Name)
       getline(strstr,word,',');
       target_name.push_back(word);
     // Gather Params
       while(getline(strstr,word,',')){
           tokenized.push_back(word);
       }
    }

    //Count All Parameters
    size_t counter = tokenized.size();
    std::cout << "[POINT_READER]Counter  : " << counter <<std::endl; 
    size_t point_amount = (size_t)(counter / 6.0);
    std::cout << "[POINT_READER]Div Count : " << point_amount << " ,  File Count = " << waypoint_count <<std::endl;

    std::vector<std::string>::iterator word_it;
    word_it = tokenized.begin();

    // Create move_base GOAL and Push into vector ! 
    for(int point_index = 0  ; point_index < counter ; point_index++){
       move_base_msgs::MoveBaseGoal newPoint;
       newPoint.target_pose.pose.position.x    = std::atof((word_it++)->c_str());
       newPoint.target_pose.pose.position.y    = std::atof((word_it++)->c_str());
       newPoint.target_pose.pose.orientation.x = std::atof((word_it++)->c_str());
       newPoint.target_pose.pose.orientation.y = std::atof((word_it++)->c_str());
       newPoint.target_pose.pose.orientation.z = std::atof((word_it++)->c_str());
       newPoint.target_pose.pose.orientation.w = std::atof((word_it++)->c_str());
       targets.push_back(newPoint);
       if(word_it == tokenized.end())break;
    }

    //Prompt End of Process
    std::cout << "[POINT_READER] Point of Interest in imported" << std::endl;
    ROS_INFO("Successfully Load waypoints !");
}

// Display Waypoints
void Navigation::displayWaypoints() {
    for(int i = 0 ; i < target_name.size() ; i++) {
      std::cout <<"["<<i<<"] " << target_name[i] << " " << targets[i].target_pose.pose.position.x <<std::endl; 
    }
}

void Navigation::getUserInput() {

}

void Navigation::setNavigationMode(int mode) {
    navigation_mode = mode;
}

int Navigation::getNavigationMode() {
    return navigation_mode;
}

void Navigation::setTimer(int duration) {
    ROS_INFO("Loop Create Timer");
	requestToCreateTimer = false;
    // timer = nh_.createTimer(ros::Duration(duration), &Navigation::timerCallback, this);
}

void Navigation::timerCallback(const ros::TimerEvent &event) {
    ROS_INFO("[TimerCallback] clear costmap");
	//Clear Costmap
  	client.call(clearer);
}

// Convert String To Int
int Navigation::toint(std::string s) { //The conversion function 
    return atoi(s.c_str());
}

}