#include "icreate_navigation/single_navigation_new.h"

namespace icreate {

SingleNavigation::SingleNavigation(): 
    clear_costmap_service_name_("/move_base/clear_costmaps"),
    get_waypoint_list_service_name_("get_waypoint_list")
{
    nh_.param("clear_costmap_service", clear_costmap_service_name_, clear_costmap_service_name_);
    nh_.param("get_waypoint_list_service", get_waypoint_list_service_name_, get_waypoint_list_service_name_);
    navigation_mode = -1;
    clear_costmap_client_ = nh_.serviceClient<std_srvs::Empty>(clear_costmap_service_name_);
}

SingleNavigation::SingleNavigation(std::string building, std::string building_floor): 
    clear_costmap_service_name_("/move_base/clear_costmaps"),
    get_waypoint_list_service_name_("get_waypoint_list")
{
    nh_.param("clear_costmap_service", clear_costmap_service_name_, clear_costmap_service_name_);
    nh_.param("get_waypoint_list_service", get_waypoint_list_service_name_, get_waypoint_list_service_name_);
    this->navigation_mode = -1;
    this->clear_costmap_client_ = nh_.serviceClient<std_srvs::Empty>(clear_costmap_service_name_);
    this->building = building;
    this->building_floor = building_floor;
}

SingleNavigation::~SingleNavigation() {

}

void SingleNavigation::setRobotTarget(MoveBaseGoalData &data) {
    ROS_INFO("setRobotTarget: %s",data.getGoalName().c_str());
    this->target = data;
}

MoveBaseGoalData SingleNavigation::getRobotTarget() {
    ROS_INFO("get Robot Target: %s",this->target.getGoalName().c_str());
    return this->target;
}

void SingleNavigation::setRobotGoal(std::string frame_id) {
    
    this->goal = this->target.getGoal();    
    this->goal.target_pose.header.frame_id = frame_id;
    this->goal.target_pose.header.stamp = ros::Time::now();
    std::cout << "[AGENT] SET NEW GOAL ! " << std::endl;
}

move_base_msgs::MoveBaseGoal SingleNavigation::getRobotGoal() {
    return this->goal;
}

// Called when finish move base
void SingleNavigation::doneRobotGoal() {
    this->clearCostmap();
    target_queue.erase(target_queue.first());
    std::string state_request;
    if(this->robot.current_state == "GOING") {
        state_request = "WAITING";
    }
    else if(this->robot.current_state == "BACKTOBASE") {
        state_request = "IDLE";
    }
    else if(this->robot.current_state == "SENDSUPPLIES") {
        state_request = "WAITING";
    }
    else if(this->robot.current_state == "SINGLERUN") {
        state_request = "IDLE";
    }
    this->robot.sendStateRequest(state_request);
    ROS_INFO("State Request: %s",state_request.c_str());
}

bool SingleNavigation::sendWaypointRequest(std::string building, std::string building_floor) {
    ros::ServiceClient client = nh_.serviceClient<miltbot_map::GetWaypointList>(get_waypoint_list_service_name_);
    miltbot_map::GetWaypointList srv;
    srv.request.building = building;
    srv.request.floor = building_floor;
    std::vector<miltbot_map::Waypoint> waypoints;
    if(client.call(srv)) {
        waypoints = srv.response.waypoints;
        this->building = building;
        this->building_floor = building_floor;
        this->setWaypoint(waypoints, building_floor);
        return true;
    }
    else {
        ROS_ERROR("Failed to call service get_waypoint_list");
        return false;
    }

}

void SingleNavigation::displayWaypoints() {
    for(int i = 0 ; i < this->targets.size() ; i++) {
      std::cout <<"["<<i<<"] " << this->targets[i].getGoalName() << " " <<std::endl; 
    }
    std::cout <<"["<<99<<"] " << "Finish" << " " <<std::endl; 
}

void SingleNavigation::displayLiftWaypoints() {
    for(int i = 0 ; i < this->lifts.size() ; i++) {
      std::cout <<"["<<i<<"] " << this->lifts[i].getGoalName() << " " <<std::endl; 
    }
    std::cout <<"["<<99<<"] " << "Finish" << " " <<std::endl; 
}

bool SingleNavigation::showNavigationMenu(std::string base_frame_id, std::string robot_frame_id) {
    this->base_frame_id = base_frame_id;
    this->robot_frame_id = robot_frame_id;
    int selected_point = -1;
    while(ros::ok()) {
		this->setNavigationMode(-1);
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
		this->setNavigationMode(mode);
		std::cout << "You Selected : " << mode <<std::endl;
		std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;

        if(mode >= 0 && mode < 10) {
            selected_point = this->showWaypointMenu();
            if(selected_point == -1) {
                continue;
            }
            //Set the endPoint to go
            this->setRobotTarget(this->targets[selected_point]);
            target_queue.push_back(this->targets[selected_point]);
            robot.setEndPosition(this->target);
        }

        // bool flag = false;
        std::string state;
        // Do Action depends on the Mode selected
        std::cout << this->getNavigationMode() << std::endl;
		switch(this->getNavigationMode()){
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
                while(ros::ok()) {
                    std::cout << "Please insert next point !!!" << std::endl;
                    selected_point = this->showWaypointMenu();
                    if(selected_point == -1) {
                        continue;
                    }
                    else if(selected_point == 99) {
                        break;
                    }
                    target_queue.push_back(this->targets[selected_point]);
                    robot.setEndPosition(this->targets[selected_point]);
                }
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
                while(true) {
                    std::cout << "Please insert next point !!!" << std::endl;
                    selected_point = this->showWaypointMenu();
                    if(selected_point == -1) {
                        continue;
                    }
                    else if(selected_point == 99) {
                        break;
                    }
                    target_queue.push_back(this->targets[selected_point]);
                    robot.setEndPosition(this->targets[selected_point]);
                }
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
		  		this->robot.sendStateRequest("IDLE");
				this->requestToSetNewGoal = false;
	      		return false;
            }
            default:
                ROS_WARN("Wrong Selected Navigation Mode");
		        std::cout << "Please Try Again" << std::endl;
                continue;
        }

        //Remember This Location as startPoint
		robot.setCurrentPosition(base_frame_id, robot_frame_id, this->building, this->building_floor);
        this->robot.sendStateRequest(state);
        this->requestToSetNewGoal = true;
        ROS_INFO("Target Queue: %ld", target_queue.size());
        return true;
    }
}

int SingleNavigation::showWaypointMenu() {
    this->displayWaypoints();
	// Ask for ID and wait user input
	std::cout << "[AGENT] Input Target Waypoints ID : " ;
    int selected_point;
	std::cin >> selected_point;
    if((selected_point < 0 || selected_point > this->targets.size()) && selected_point != 99) {
        ROS_WARN("Wrong Select Point To Navigate Try Again");
        return -1;
    }
    return selected_point;
}

bool SingleNavigation::setNextStepMode() {
    ROS_INFO("Navigation Mode: %d",this->navigation_mode);
    std::string state;
    int selected_point = -1;
    switch(this->navigation_mode) {
        //MODE : Go to Specific Point
		    case 0: {
		    	this->showNavigationMenu(this->base_frame_id, this->robot_frame_id);
                break;
            }
			//MODE : Going and Wait for Cargo
			case 1 : { 
                this->showNavigationMenu(this->base_frame_id, this->robot_frame_id);
                break;
            }
			//MODE : Delivery to Target Place
	    	case 2: {
                this->showNavigationMenu(this->base_frame_id, this->robot_frame_id);
	      		break;
            }
            //MODE : Going and Come Back to This Place
	    	case 3: {
                this->setRobotTarget(this->robot.startPosition);
                target_queue.push_back(this->robot.startPosition);
                robot.setCurrentPosition(this->base_frame_id, this->robot_frame_id, this->building, this->building_floor);
                robot.setEndPosition(this->target);
                this->robot.sendStateRequest(state);
                this->requestToSetNewGoal = true;
                ROS_INFO("Target Queue: %ld", target_queue.size());
                break;
            }
            //MODE : Going and Come Back to Base Station
		    case 4: { 
                this->setRobotTarget(this->robot.startPosition);
                target_queue.push_back(this->robot.startPosition);
                robot.setCurrentPosition(this->base_frame_id, this->robot_frame_id, this->building, this->building_floor);
                robot.setEndPosition(this->target);
                this->robot.sendStateRequest(state);
                this->requestToSetNewGoal = true;
                ROS_INFO("Target Queue: %ld", target_queue.size());
                break;
            }
            //MODE : Going to n Places and Send Supplies in 1 Place
		    case 5: {
                while(ros::ok()) {
                    std::cout << "Please insert next point !!!" << std::endl;
                    int selected_point = this->showWaypointMenu();
                    if(selected_point == -1) {
                        continue;
                    }
                    else if(selected_point == 99) {
                        break;
                    }
                    this->setRobotTarget(this->targets[selected_point]);
                    target_queue.push_back(this->targets[selected_point]);
                    robot.setEndPosition(this->targets[selected_point]);
                }
                selected_point = this->showWaypointMenu();
                if(selected_point == -1) {
                    continue;
                }
                //Set the endPoint to go
                this->setRobotTarget(this->targets[selected_point]);
                target_queue.push_back(this->targets[selected_point]);
                robot.setEndPosition(this->target); 

                break;
            }
            //MODE : Going 1 Place and Send Supplies in n Places
		    case 6: { 
                break;
            }
            //MODE : Going n Places and Send Supplies in n Places
		    case 7: {
                break;
            }
		  	//MODE : Execute The Memorized Sequence
			case 10: {
                ROS_WARN("Cannot Use This Mode Now");
                return false; 
                // continue;
            }
            default:
                ROS_WARN("Wrong Selected Navigation Mode");
		        std::cout << "Please Try Again" << std::endl;
                return false; 
    }
    return true;
}

bool SingleNavigation::runRecoveryMode() {
    return true;
}

void SingleNavigation::setNavigationMode(int mode) {
    this->navigation_mode = mode;
}

int SingleNavigation::getNavigationMode() {
    return this->navigation_mode;
}

void SingleNavigation::createTimer(int duration) {
    this->requestToCreateTimer = false;
    ROS_INFO("Create Timer");
    timer_ = nh_.createTimer(ros::Duration(duration), &SingleNavigation::timerCallback, this);
}

void SingleNavigation::clearCostmap() {
    std_srvs::Empty clearer;
    if(this->clear_costmap_client_.call(clearer)) {
        ROS_INFO("Finish clear costmaps");
    }
    else {
        ROS_ERROR("Fail to call Service %s", this->clear_costmap_service_name_.c_str());
    }
}

void SingleNavigation::timerCallback(const ros::TimerEvent &event) {
    ROS_INFO("[TimerCallback] clear costmap");
	this->clearCostmap();
}

// Convert String To Int
int SingleNavigation::toint(std::string s) { //The conversion function 
    return atoi(s.c_str());
}

void SingleNavigation::setWaypoint(std::vector<miltbot_map::Waypoint> waypoints, std::string building_floor) {
    std::size_t found = building_floor.find("Lift");
    for(int i = 0; i < waypoints.size(); i++) {
           MoveBaseGoalData data;
           data.setGoalName(waypoints[i].name);
           data.setBuilding(waypoints[i].building);
           data.setBuildingFloor(waypoints[i].floor);
           data.setGoal(waypoints[i].goal);
           if (found != std::string::npos) {   
               this->lifts.push_back(data);
           }
           else {
               this->targets.push_back(data);
           }
    } 
    
      
}

}