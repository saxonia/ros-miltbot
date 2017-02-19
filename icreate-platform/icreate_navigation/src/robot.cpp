#include "icreate_navigation/robot.h"

namespace icreate {

MoveBaseGoalData::MoveBaseGoalData(void) {

}

MoveBaseGoalData::MoveBaseGoalData(std::string goal_name, move_base_msgs::MoveBaseGoal goal, std::string building, std::string building_floor) {
    this->goal_name = goal_name;
    this->goal = goal;
    this->building = building;
    this->building_floor = building_floor;
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

void MoveBaseGoalData::setBuilding(std::string building) {
    this->building = building;
}

std::string MoveBaseGoalData::getBuilding() {
    return this->building;
}

void MoveBaseGoalData::setBuildingFloor(std::string building_floor) {
    this->building_floor = building_floor;
}

std::string MoveBaseGoalData::getBuildingFloor() {
    return this->building_floor;
}

Robot::Robot(std::string building, std::string building_floor, std::string base_frame_id, std::string robot_frame_id):
    state_sub_topic_name_("/state"),
    set_robot_state_service_name_("/set_robot_state")
{
    nh_.param("state_sub_topic", state_sub_topic_name_, state_sub_topic_name_);
    nh_.param("set_robot_state_service", set_robot_state_service_name_, set_robot_state_service_name_);
    this->client = nh_.serviceClient<icreate_state::SetRobotState>(set_robot_state_service_name_);
    this->target_queue.reserve(0);
    this->current_state = "IDLE";
    this->building = building;
    this->building_floor = building_floor;
    this->base_frame_id = base_frame_id;
    this->robot_frame_id = robot_frame_id;
    this->navigation_mode = -1;
    this->setCurrentPosition(base_frame_id, robot_frame_id, building, building_floor);
    ROS_INFO("Create Robot Class");
}

Robot::~Robot(void) {

}

bool Robot::setCurrentPosition(std::string base_frame_id, std::string robot_frame_id,
                                std::string building, std::string building_floor) {
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

    this->currentPosition.goal_name = "Current Position";
    this->currentPosition.goal.target_pose.pose.position.x    = transform.getOrigin().x();
    this->currentPosition.goal.target_pose.pose.position.y    = transform.getOrigin().y();
    this->currentPosition.goal.target_pose.pose.orientation.x = transform.getRotation().x();
    this->currentPosition.goal.target_pose.pose.orientation.y = transform.getRotation().y();
    this->currentPosition.goal.target_pose.pose.orientation.z = transform.getRotation().z();
    this->currentPosition.goal.target_pose.pose.orientation.w = transform.getRotation().w();
    this->currentPosition.building = building;
    this->currentPosition.building_floor = building_floor;
    
    std::cout << "RETURNING POSITION MARKED : ";
    std::cout << this->currentPosition.goal.target_pose.pose.position.x << "," 
            << this->currentPosition.goal.target_pose.pose.position.y << " at Time: " << now <<std::endl;
    return true;
}

 MoveBaseGoalData Robot::getCurrentPosition() {
    return this->currentPosition;
}

bool Robot::setStartPosition(MoveBaseGoalData &data) {
    this->startPosition = data;
    return true;
}

MoveBaseGoalData Robot::getStartPosition() {
    return this->startPosition;
}

bool Robot::setEndPosition(MoveBaseGoalData &data) {
    this->endPosition = data;
    return true;
}

MoveBaseGoalData Robot::getEndPosition() {
    return this->endPosition;
}

void Robot::addTargetQueue(MoveBaseGoalData data) {
    this->target_queue.push_back(data);
    ROS_INFO("Target Queue: %ld", target_queue.size());
}

void Robot::deleteTargetQueue(int idx) {
    this->target_queue.erase(this->target_queue.begin() + idx);
    ROS_INFO("Target Queue: %ld", target_queue.size());
}

void Robot::setNavigationMode() {
    while(ros::ok()) {
		this->navigation_mode = -1;
		// Select Mode for Transportation 
		std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;
		std::cout << "[AGENT] Please Select Mode Do You Need To Run" <<std::endl;
		std::cout << "[ 0 ] Run to Specific Point." <<std::endl;
		std::cout << "[ 1 ] Going and Wait for Cargo." <<std::endl;
		std::cout << "[ 2 ] Delivery to Target Place." <<std::endl;
		std::cout << "[ 3 ] Going and Come Back to This Place." <<std::endl;
		std::cout << "[ 4 ] Going and Come Back to Base Station." <<std::endl;
		std::cout << "[ 5 ] Going to n Places and Send Supplies in 1 Place." << std::endl;
		std::cout << "[ 6 ] Going 1 Place and Send Supplies in n Places" << std::endl;
		std::cout << "[ 7 ] Going n Places and Send Supplies in n Places" << std::endl;
		std::cout << "[ 10 ] Execute The Memorized Sequence" <<std::endl;
        std::cout << "[ 99 ] Exit Sysytem." << std::endl;
		std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;
		std::cout << "Select Mode[0,1,2,3,4,5,6,7,10,99] : " ;
		
        int mode; 
		std::cin  >> mode;
		this->navigation_mode = mode;
		std::cout << "You Selected : " << mode <<std::endl;
		std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;

        std::string state;
        // Do Action depends on the Mode selected
        std::cout << this->navigation_mode << std::endl;
		switch(this->navigation_mode){
            //MODE : Go to Specific Point
		    case 0: {
		    	state = "SINGLERUN";
                break;
            }
			//MODE : Going and Wait for Cargo
			case 1 : { 
                state = "GOING";
                break;
            }
			//MODE : Delivery to Target Place
	    	case 2: {
	      		state = "SENDSUPPLIES";
                break;
            }
            //MODE : Going and Come Back to This Place
	    	case 3: {
	      		state = "GOING";
                break;
            }
            //MODE : Going and Come Back to Base Station
		    case 4: { 
                state = "GOING";
                break;
            }
            //MODE : Going to n Places and Send Supplies in 1 Place
		    case 5: { 
                state = "GOING";
                break;
            }
            //MODE : Going 1 Place and Send Supplies in n Places
		    case 6: { 
                state = "GOING";
                break;
            }
            //MODE : Going n Places and Send Supplies in n Places
		    case 7: {
                state = "GOING";
                break;
            }
		  	//MODE : Execute The Memorized Sequence
			case 10: {
                ROS_WARN("Cannot Use This Mode Now"); 
                continue;
            }
		    //MODE : EXIT
			case 99: {
				//Set the Robot State 
		  		state = "IDLE";
                break;
            }
            default:
                ROS_WARN("Wrong Selected Navigation Mode");
		        std::cout << "Please Try Again" << std::endl;
                continue;
        }
        this->sendStateRequest(state);
        break;
    }
}

int Robot::getNavigationMode() {
    return this->navigation_mode;
}

void Robot::sendStateRequest(std::string state_request) {
    ROS_INFO("Send State Request: %s",state_request.c_str());
    icreate_state::SetRobotState srv;
    srv.request.req = state_request;
    if (this->client.call(srv))
    {
      this->current_state = srv.response.res;
    }
    else
    {
      ROS_ERROR("Failed to call service set_robot_state");
    }
}

}