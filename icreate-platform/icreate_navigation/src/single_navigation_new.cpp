#include "icreate_navigation/single_navigation_new.h"

namespace icreate {

SingleNavigation::SingleNavigation(std::string building, std::string building_floor): 
    clear_costmap_service_name_("/move_base/clear_costmaps"),
    get_waypoint_list_service_name_("get_waypoint_list")
{
    nh_.param("clear_costmap_service", clear_costmap_service_name_, clear_costmap_service_name_);
    nh_.param("get_waypoint_list_service", get_waypoint_list_service_name_, get_waypoint_list_service_name_);
    this->clear_costmap_client_ = nh_.serviceClient<std_srvs::Empty>(clear_costmap_service_name_);
    this->building = building;
    this->building_floor = building_floor;
    this->building_floor_num = this->toint(this->substrBuildingFloor(building_floor));
    this->targets.reserve(0);
    this->lifts.reserve(0);
    this->requestToSetNewGoal = false;
    this->requestToCreateTimer = false;
    this->isFinishQueue = false;
    this->base_frame_id = "";
    this->robot_frame_id = "";
    this->sendWaypointRequest(building, building_floor);
}

SingleNavigation::~SingleNavigation(void) {
    
}

void SingleNavigation::setRobotTarget(MoveBaseGoalData &data) {
    ROS_INFO("setRobotTarget: %s",data.getGoalName().c_str());
    this->target = data;
}

// void SingleNavigation::setRobotTarget() {
//     ROS_INFO("setRobotTarget: %s",this->targets[selected_point].getGoalName().c_str());
//     this->target = this->targets[selected_point];
// }

MoveBaseGoalData SingleNavigation::getRobotTarget() {
    ROS_INFO("get Robot Target: %s",this->target.getGoalName().c_str());
    return this->target;
}

void SingleNavigation::setRobotGoal(std::string frame_id) {
    ROS_WARN("GOAL Tar: %s",this->target.getBuildingFloor().c_str());
    ROS_WARN("%lf",this->getRobotGoal().target_pose.pose.position.x);
    this->goal = this->target.getGoal();    
    this->goal.target_pose.header.frame_id = frame_id;
    this->goal.target_pose.header.stamp = ros::Time::now();
    std::cout << "[AGENT] SET NEW GOAL ! " << std::endl;
}

move_base_msgs::MoveBaseGoal SingleNavigation::getRobotGoal() {
    return this->goal;
}

// Called when finish move base
void SingleNavigation::doneRobotGoal(Robot &robot) {
    this->clearCostmap();
    if(robot.target_queue.size() > 0) {
        robot.setStartPosition(robot.target_queue[0]);
        robot.deleteTargetQueue(0);
    }

    std::string state_request;
    if(robot.current_state == "GOING") {
        state_request = "WAITING";
    }
    else if(robot.current_state == "BACKTOBASE") {
        state_request = "IDLE";
    }
    else if(robot.current_state == "SENDSUPPLIES") {
        state_request = "WAITING";
    }
    else if(robot.current_state == "SINGLERUN") {
        state_request = "IDLE";
    }
    robot.sendStateRequest(state_request);
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
        if(waypoints.size() > 0) {
            this->building = building;
            this->building_floor_num = this->toint(this->substrBuildingFloor(building_floor));
            this->setWaypoint(waypoints, building_floor);
        }
        else {
            ROS_WARN("Failed to receive waypoints");
        }
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
    std::cout <<"["<<99<<"] " << "Cancel" << " " <<std::endl; 
}

void SingleNavigation::displayLiftWaypoints() {
    for(int i = 0 ; i < this->lifts.size() ; i++) {
      std::cout <<"["<<i<<"] " << this->lifts[i].getGoalName() << " " <<std::endl; 
    }
    std::cout <<"["<<99<<"] " << "Cancel" << " " <<std::endl; 
}

void SingleNavigation::setupRobotToRun(Robot &robot, std::string base_frame_id, std::string robot_frame_id) {
    //ถกเถียงกันต่อว่าควรอยู่ตรงไหน
    this->base_frame_id = base_frame_id;
    this->robot_frame_id = robot_frame_id;
    this->requestToSetNewGoal = true;
    robot.setCurrentPosition(base_frame_id, robot_frame_id, this->building, this->building_floor);
    this->setRobotTarget(robot.target_queue[0]);
    robot.setEndPosition(robot.target_queue[robot.target_queue.size()-1]);
}

bool SingleNavigation::setupNavigationQueue(Robot &robot) {
    robot.setNavigationMode();
    int navigation_mode = robot.getNavigationMode();
    this->isFinishQueue = false;
    int selected_point = -1;
    this->requestToSetNewGoal = false;
    if(navigation_mode >= 0 && navigation_mode < 10) {
        selected_point = this->showWaypointMenu();
        if(selected_point == 99) {
            return false;
        }
        robot.addTargetQueue(this->targets[selected_point]);
    }
    std::string state;
    // Do Action depends on the Mode selected
    std::cout << navigation_mode << std::endl;
	switch(navigation_mode){
        //MODE : Go to Specific Point
	    case 0: {
            break;
        }
		//MODE : Going and Wait for Cargo
		case 1 : { 
            break;
        }
		//MODE : Delivery to Target Place
		case 2: {
            break;
        }
        //MODE : Going and Come Back to This Place
		case 3: {
            break;
        }
        //MODE : Going and Come Back to Base Station
	    case 4: { 
            break;
        }
        //MODE : Going to n Places and Send Supplies in 1 Place
	    case 5: { 
            while(ros::ok()) {
                std::cout << "Please insert next point !!!" << std::endl;
                selected_point = this->showWaypointMenu();
                if(selected_point == 99) {
                    break;
                }
                robot.addTargetQueue(this->targets[selected_point]);
            }
            break;
        }
        //MODE : Going 1 Place and Send Supplies in n Places
	    case 6: { 
            break;
        }
        //MODE : Going n Places and Send Supplies in n Places
	    case 7: {
            while(true) {
                std::cout << "Please insert next point !!!" << std::endl;
                selected_point = this->showWaypointMenu();
                if(selected_point == 99) {
                    break;
                }
                robot.addTargetQueue(this->targets[selected_point]);
            }
            break;
        }
        default:
            return false;
    }
    return true;
}

int SingleNavigation::showWaypointMenu() {
    int selected_point;
    while(ros::ok()) {
        this->displayWaypoints();
	    // Ask for ID and wait user input
	    std::cout << "[AGENT] Input Target Waypoints ID : " ;
	    std::cin >> selected_point;
        if((selected_point < 0 || selected_point >= this->targets.size()) && selected_point != 99) {
            ROS_WARN("Wrong Select Point To Navigate Try Again");
            continue;
        }
        return selected_point;
    }
}

bool SingleNavigation::setNextStepMode(Robot &robot) {
    std::string state;
    int selected_point = -1;
    switch(robot.getNavigationMode()) {
        //MODE : Go to Specific Point
		    case 0: {
		    	this->setupNavigationQueue(robot);
                break;
            }
			//MODE : Going and Wait for Cargo
			case 1 : { 
                this->setupNavigationQueue(robot);
                break;
            }
			//MODE : Delivery to Target Place
	    	case 2: {
                this->setupNavigationQueue(robot);
	      		break;
            }
            //MODE : Going and Come Back to This Place
	    	case 3: {
                if(this->isFinishQueue) {
                    this->setupNavigationQueue(robot);
                }
                else {
                    state = "GOING";
                    robot.addTargetQueue(robot.getStartPosition());
                    robot.sendStateRequest(state);
                    this->isFinishQueue = true;
                }
                break;
            }
            //MODE : Going and Come Back to Base Station
		    case 4: {
                if(this->isFinishQueue) {
                    this->setupNavigationQueue(robot);
                }
                else {
                    state = "BACKTOBASE"; 
                    robot.addTargetQueue(this->targets[0]);
                    robot.sendStateRequest(state);
                    this->isFinishQueue = true;
                }
                break;
            }
            //MODE : Going to n Places and Send Supplies in 1 Place
		    case 5: {
                if(robot.target_queue.size() == 0) {
                    if(this->isFinishQueue) {
                        this->setupNavigationQueue(robot);
                    }
                    else {
                        state = "SENDSUPPLIES";
                        selected_point = this->showWaypointMenu();
                        if(selected_point == 99) {
                            return false;
                        }
                        robot.addTargetQueue(this->targets[selected_point]);
                        this->isFinishQueue = true;
                    }
                }
                else {
                    state = "GOING";
                }
                robot.sendStateRequest(state);
                break;
            }
            //MODE : Going 1 Place and Send Supplies in n Places
		    case 6: {
                if(robot.target_queue.size() == 0) {
                    if(this->isFinishQueue) {
                        this->setupNavigationQueue(robot);
                    }
                    else {
                        while(ros::ok()) {
                            std::cout << "Please insert point !!!" << std::endl;
                            selected_point = this->showWaypointMenu();
                            if(selected_point == 99) {
                                break;
                            }
                            robot.addTargetQueue(this->targets[selected_point]);
                        }
                        this->isFinishQueue = true;
                    }   
                }
                else {
                    state = "SENDSUPPLIES";
                }
                robot.sendStateRequest(state);
                break;
            }
            //MODE : Going n Places and Send Supplies in n Places
		    case 7: {
                if(robot.target_queue.size() != 0 && !this->isFinishQueue) {
                    state = "GOING";
                }
                else if(robot.target_queue.size() != 0 && this->isFinishQueue) {
                    state = "SENDSUPPLIES";
                }
                else {
                    if(this->isFinishQueue) {
                        this->setupNavigationQueue(robot);
                    }
                    else {
                        while(ros::ok()) {
                            std::cout << "Please insert point !!!" << std::endl;
                            selected_point = this->showWaypointMenu();
                            if(selected_point == 99) {
                                break;
                            }
                            robot.addTargetQueue(this->targets[selected_point]);
                        }
                        this->isFinishQueue = true;
                    }
                    state = "SENDSUPPLIES";
                }
                robot.sendStateRequest(state);
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
               this->building_floor = building_floor;
           }
    } 
}

std::string SingleNavigation::substrBuildingFloor(std::string building_floor) {
    return building_floor.substr(6);
}

}