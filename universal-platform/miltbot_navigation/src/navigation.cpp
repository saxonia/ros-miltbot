#include "miltbot_navigation/navigation.h"

namespace miltbot {

Navigation::Navigation(std::string base_frame_id, std::string robot_frame_id, std::string building, std::string building_floor):
    base_frame_id("map"),
    robot_frame_id("base_footprint"),
    clear_costmap_service_name_("/move_base/clear_costmaps"),
    get_waypoint_list_service_name_("get_waypoint_list"),
    move_base_topic_name_("move_base"),
    move_base_wait_time_(2.0), 
    add_target_service_name_("add_target"),
    fail_goal_value_(2),
    set_robot_state_service_name_("/set_robot_state"),
    ac("move_base", true) 
{
    nh_.param("clear_costmap_service", clear_costmap_service_name_, clear_costmap_service_name_);
    nh_.param("get_waypoint_list_service", get_waypoint_list_service_name_, get_waypoint_list_service_name_);
    nh_.param("move_base_wait_time_", move_base_wait_time_, move_base_wait_time_);
    this->clear_costmap_client_ = nh_.serviceClient<std_srvs::Empty>(clear_costmap_service_name_);
    this->set_robot_state_client_ = nh_.serviceClient<miltbot_state::SetRobotState>(set_robot_state_service_name_);
    this->base_frame_id = base_frame_id;
    this->robot_frame_id = robot_frame_id;
    this->building = building;
    this->building_floor = building_floor;
    this->requestToSetNewGoal = false;
    this->current_state = "IDLE";
    this->navigation_case = -1;
    this->isSystemWorking = true;
    this->isDoneGoal = true;
    this->isLiftNavigation = false;
    this->done_goal_number = -1;
    this->fail_goal_count = 0;
    this->lift_navigation_step = 0;
    this->lifts.reserve(0);
    this->waitMoveBaseServer(move_base_wait_time_);
    this->setCurrentPosition("Current Position");
    this->sendWaypointRequest(building,building_floor + " Lift");
    this->add_target_service_server = nh_.advertiseService(this->add_target_service_name_, &Navigation::addTargetService, this);
}

Navigation::~Navigation(void) 
{

}

void Navigation::addTargetQueue(MoveBaseGoalData data) {
    this->target_queue.push_back(data);
    ROS_INFO("Target Queue: %ld", target_queue.size());
}

void Navigation::deleteTargetQueue(int idx) {
    this->target_queue.erase(this->target_queue.begin() + idx);
    ROS_INFO("Target Queue: %ld", target_queue.size());
}

void Navigation::setBuilding(std::string building) {
    this->building = building;
}

std::string Navigation::getBuilding() {
    return this->building;
}

void Navigation::setBuildingFloor(std::string building_floor) {
    this->building_floor = building_floor;
}

std::string Navigation::getBuildingFloor() {
    return this->building_floor;
}

bool Navigation::setCurrentPosition(std::string current_position_name) {
    tf::TransformListener listener;  
    tf::StampedTransform transform;
    ros::Time now = ros::Time::now();
    try {
        ROS_INFO("WAITING FOR TRANSFORM FRAME");
        listener.waitForTransform(this->base_frame_id, this->robot_frame_id, ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform(this->base_frame_id, this->robot_frame_id, ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        return false;
    }

    this->currentPosition.goal_name = current_position_name;
    this->currentPosition.goal.target_pose.pose.position.x    = transform.getOrigin().x();
    this->currentPosition.goal.target_pose.pose.position.y    = transform.getOrigin().y();
    this->currentPosition.goal.target_pose.pose.orientation.x = transform.getRotation().x();
    this->currentPosition.goal.target_pose.pose.orientation.y = transform.getRotation().y();
    this->currentPosition.goal.target_pose.pose.orientation.z = transform.getRotation().z();
    this->currentPosition.goal.target_pose.pose.orientation.w = transform.getRotation().w();
    this->currentPosition.building = this->building;
    this->currentPosition.building_floor = this->building_floor;
    
    std::cout << "RETURNING POSITION MARKED : ";
    std::cout << this->currentPosition.goal.target_pose.pose.position.x << "," 
            << this->currentPosition.goal.target_pose.pose.position.y << " at Time: " << now <<std::endl;
    return true;
}

bool Navigation::setCurrentPosition(MoveBaseGoalData current_position) {
    this->currentPosition = current_position;
}

 MoveBaseGoalData Navigation::getCurrentPosition() {
    return this->currentPosition;
}

void Navigation::setRobotTarget(MoveBaseGoalData data) {
    // ROS_INFO("setRobotTarget: %s",data.getGoalName().c_str());
    this->target = data;
}

MoveBaseGoalData Navigation::getRobotTarget() {
    // ROS_INFO("get Robot Target: %s",this->target.getGoalName().c_str());
    return this->target;
}

void Navigation::setRobotGoal(std::string frame_id) {
    this->goal = this->target.getGoal(); 
    this->goal.target_pose.header.frame_id = frame_id;
    this->goal.target_pose.header.stamp = ros::Time::now();
    std::cout << "[AGENT] SET NEW GOAL ! " << std::endl;
}

move_base_msgs::MoveBaseGoal Navigation::getRobotGoal() {
    return this->goal;
}

bool Navigation::verifyTarget() {
    this->navigation_case = -1;
    bool flag1 = verifyTargetBuilding(this->currentPosition, this->target_queue[0]);
    bool flag2 = verifyTargetFloor(this->currentPosition, this->target_queue[0]);
    return true;
    if(flag1 && flag2) {
        ROS_INFO("Verify Target : case 0");
        this->navigation_case = Navigation::ONSAMEFLOOR;
    }
    else if(flag1 && !flag2) {
        ROS_INFO("Verify Target : case 1");
        this->navigation_case = Navigation::ONDIFFFLOOR;
        this->isLiftNavigation = true;
    }
    else {
        ROS_INFO("Verify Target : case 2");
        this->navigation_case = Navigation::ONDIFFBUILDING;
        return false;
    }
    return true;
}

bool Navigation::update() {
    if(this->isSystemWorking) {
        // ROS_INFO("Navigation System is stil working");
        //เช็คว่าทำงานเสร็จรึยัง
        if(this->isDoneGoal) {
            this->isDoneGoal = false;
            if(this->target_queue.size() > 0) {
                //เช็คว่าจุดหมายที่จะไปอยู่ชั้นเดียวกันมั้ย ถ้าอยู่ชั้นเดียวกันก็เซ็ตเลย ถ้าไม่ก็เซ็ต target ไปอยู่ที่ส่วนของ Lift
                this->verifyTarget();
                if(this->isLiftNavigation) {
                    this->runLiftNavigation();
                }
                else {
                    this->setRobotTarget(this->target_queue[0]);
                    this->setRobotGoal(this->base_frame_id);
                    this->sendStateRequest(this->target_queue[0].getTask());
                    this->runMoveBase();
                }
                 
            }
            else if(this->target_queue.size() == 0) {
                //สั่งให้หุ่นยนต์กลับไปยังจุด base station
                this->isDoneGoal = true;
            }
            else {
                return false;
            }
        }
        else {
            //ระบบยังทำงาน ไม่เสร็จ ปล่อยผ่านไป
        }
    }
    else {
        //ระบบหยุดทำงาน ทำอะไรต่อ ?
        ROS_WARN("Navigation System is shuting down"); 
    }
    return true;
}

void Navigation::runMoveBase() {
    ac.sendGoal(goal, 
                boost::bind(&Navigation::goalDoneCallback, this, _1, _2), 
                boost::bind(&Navigation::goalActiveCallback, this), boost::bind(&Navigation::goalFeedbackCallback, this, _1));
}

void Navigation::runLiftNavigation() {
    switch(this->lift_navigation_step) {
        //Step 0: Move to Lift Ground
        case 0: {
            MoveBaseGoalData data = this->lifts.back();
            data.task = "USINGLIFT";
            this->target_queue.insert(this->target_queue.begin(), data);
            this->setRobotTarget(this->target_queue[0]);
            this->sendStateRequest(this->target_queue[0].getTask());
            this->setRobotGoal(this->base_frame_id);
            this->runMoveBase();
            break;
        }
        //Step 1: Wait & Move to in front of the inncoming lift
        case 1: {
            int liftNumber = waitForIncomingLift();
            this->target_queue.insert(this->target_queue.begin(),this->lifts[liftNumber]);
            this->setRobotTarget(this->target_queue[0]);
            this->setRobotGoal(this->base_frame_id);
            this->sendStateRequest(this->target_queue[0].getTask());
            this->runMoveBase();
            break;
        }
        //Step 2: Wait Lift Door Open & Move inside the lift
        case 2: {
            bool flag;
            while(ros::ok()) {
                if(!this->verifyLiftDoor()) {
                    int liftNumber = waitForIncomingLift();
                    this->target_queue.insert(this->target_queue.begin(),this->lifts[liftNumber]);
                    this->setRobotTarget(this->target_queue[0]);
                    this->setRobotGoal(this->base_frame_id);
                    this->sendStateRequest(this->target_queue[0].getTask());
                    this->runMoveBase();
                    this->lift_navigation_step--;
                    break;
                }
                // ros::ServiceClient client = nh_.serviceClient<icreate_navigation::RunGmappingService>(run_gmapping_service_name_);
        //         icreate_navigation::RunGmappingService srv;
        //         srv.request.task = "open";
        //         if(client.call(srv)) {
        //             flag = srv.response.success;
        //         }
        //         else {
        //             ROS_WARN("Failed to run gmapping");
        //             continue;
        //         }
                // break;
            }
            if(flag) {
                // initializeSimpleForwardMoveBaseTarget(robot, "Going To Lift");
            }
            break;
        }
        // case 3: {
        //     ROS_ERROR("Come 3");
        //     move_base_msgs::MoveBaseGoal new_point;
        //     new_point.target_pose.pose.orientation.z = -1.0;
        //     new_point.target_pose.pose.orientation.w = 0.0;
        //     MoveBaseGoalData data("Rotate In Lift", new_point, this->navigations_[this->nav_idx].building, this->navigations_[this->nav_idx].building_floor_lift);
        //     robot.target_queue.insert(robot.target_queue.begin(),data);
        //     robot.sendStateRequest("USINGLIFT");
        //     this->navigations_[this->nav_idx].requestToSetNewGoal = true;
        //     break;
        // }
        // case 4: {
        //     ROS_ERROR("Come 4");
        //     bool flag = waitUserInputLift();
        //     if(!flag) {
        //         //ไม่แน่ใจว่าใช้ได้มั้ย
        //         return ;
        //     }
        //     while(ros::ok()) {
        //         ros::ServiceClient client = nh_.serviceClient<icreate_navigation::RunGmappingService>(run_gmapping_service_name_);
        //         icreate_navigation::RunGmappingService srv;
        //         srv.request.task = "restart";
        //         if(client.call(srv)) {
        //             flag = srv.response.success;
        //         }
        //         else {
        //             ROS_WARN("Failed to run gmapping");
        //             continue;
        //         }
        //         break;
        //     }
        //     if(flag) {
        //         mid_range += 0.5;
        //         move_base_msgs::MoveBaseGoal new_point;
        //         new_point.target_pose.pose.position.x = mid_range;
        //         new_point.target_pose.pose.orientation.w = 1;
        //         MoveBaseGoalData data("Going Out Lift", new_point,this->navigations_[this->nav_idx].building, this->navigations_[this->nav_idx].building_floor_lift);
        //         robot.target_queue.insert(robot.target_queue.begin(),data);
        //         this->navigations_[this->nav_idx].requestToSetNewGoal = true;
        //     }
        //     break;
        // }
        // case 5: {
        //     bool flag = false;
        //     ros::ServiceClient client = nh_.serviceClient<icreate_navigation::RunGmappingService>(run_gmapping_service_name_);
        //     icreate_navigation::RunGmappingService srv;
        //     srv.request.task = "close";
        //     if(client.call(srv)) {
        //         flag = srv.response.success;
        //         // break;
        //     }
        //     else {
        //         ROS_WARN("Failed to run gmapping");
        //     }
        //     if(flag) {
        //         this->navigation_case = 0;
        //         client = nh_.serviceClient<miltbot_map::SetMapServer>("set_map_service");
        //         miltbot_map::SetMapServer srv2;
        //         srv2.request.floor = robot.target_queue[0].building_floor;
        //         if(client.call(srv2)) {
        //             bool flag2 = srv2.response.flag;
        //         }
        //         else {
        //            ROS_WARN("Failed to run set map");
        //         }
        //     }
        //     break;
        // }
    }
}

void Navigation::createTimer(int duration) {
    ROS_INFO("Create Timer");
    timer_ = nh_.createTimer(ros::Duration(duration), &Navigation::timerCallback, this);
}

void Navigation::clearCostmap() {
    std_srvs::Empty clearer;
    if(this->clear_costmap_client_.call(clearer)) {
        ROS_INFO("Finish clear costmaps");
    }
    else {
        ROS_ERROR("Fail to call Service %s", this->clear_costmap_service_name_.c_str());
    }
}

void Navigation::timerCallback(const ros::TimerEvent &event) {
    ROS_INFO("[TimerCallback] clear costmap");
	this->clearCostmap();
}

bool Navigation::waitMoveBaseServer(float wait_duration) {
	// Wait for the action server to come up
	int tries = 0;
    while(!ac.waitForServer(ros::Duration(wait_duration))){
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

bool Navigation::verifyTargetBuilding(MoveBaseGoalData current, MoveBaseGoalData target) {
    if(current.getBuilding() == target.getBuilding())
        return true;
    else 
        return false;
}

bool Navigation::verifyTargetFloor(MoveBaseGoalData current, MoveBaseGoalData target) {
    if(current.getBuildingFloor() == target.getBuildingFloor())
        return true;
    else 
        return false;
}

void Navigation::displayLiftWaypoints() {
    for(int i = 0 ; i < this->lifts.size() ; i++) {
      std::cout <<"["<<i<<"] " << this->lifts[i].getGoalName() << " " <<std::endl; 
    }
    std::cout <<"["<<99<<"] " << "Cancel" << " " <<std::endl; 
}

int Navigation::waitForIncomingLift() {  
    while(ros::ok()) {
        std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<" <<std::endl;
	    std::cout << "[AGENT] Select UP OR DOWN : ";
        std::string value;
        std::cin >> value;

        std::cout << "You Selected : " << value <<std::endl;
        // if(strcmp(value,"u") != 0 && strcmp(value,"d") != 0) {
        if( value != "u" && value != "d") {
            std::cout << "[AGENT] Wrong INput End" <<std::endl; 
            continue;
        }
        this->displayLiftWaypoints();
        std::cout << "[AGENT] Wait For Number of Lift : " <<std::endl;
        std::string liftNumber;
        std::cin >> liftNumber;
        std::cout << "[AGENT] Lift Number " + liftNumber + " is on the floor" <<std::endl;
        try {
            int num = atoi(liftNumber.c_str());
            if(num < 0 || num >= this->lifts.size()) {        
                std::cout << "[AGENT] Wrong Lift Number Input" <<std::endl; 
                continue;
            }
            return num;
        }
        catch(std::exception& e) {
            std::cout << "Standard exception: " << e.what() << std::endl;
            continue;
        }
    }
}

bool Navigation::verifyLiftDoor() {
    std::cout << "Wait For Verifying Lift Door" << std::endl;
    std::cout << "Press c to cancel or any key to continue" << std::endl;
    std::cout << "Your input : ";
    std::string in;
    std::cin >> in;
    if(in == "c") {
        return false;
    }
    return true;
}

void Navigation::goalDoneCallback(const actionlib::SimpleClientGoalState &state, 
  const move_base_msgs::MoveBaseResultConstPtr &result){
    ROS_INFO("Goal Done Now !!!!!");
    this->clearCostmap();
    if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED){
		done_goal_number = 1;
        if(this->target_queue.size() > 0) {
            // robot.setStartPosition(robot.target_queue[0]);
            this->setCurrentPosition(this->target_queue[0]);
            this->deleteTargetQueue(0);

            std::string state_request;
            if(this->current_state == "SINGLERUN") {
                state_request = "IDLE";
            }
            else if(this->current_state == "USINGLIFT") {
                state_request = "USINGLIFT";
            }
            this->sendStateRequest(state_request);
        }
        else {
            ROS_ERROR("Error target_queue size");
        }
	}
	else if(state.state_ == actionlib::SimpleClientGoalState::REJECTED) {
		ROS_INFO("REJECTED");
		done_goal_number = 2;
	}
	else if(state.state_ == actionlib::SimpleClientGoalState::LOST) {
		ROS_INFO("LOST");
		done_goal_number = 3;
	}
    else if(state.state_ == actionlib::SimpleClientGoalState::ABORTED){
      	ROS_WARN("ABORTED : Failed to reach the goal...");
		done_goal_number = 4;
        if(this->fail_goal_count >= this->fail_goal_value_) {
            this->setCurrentPosition(this->target_queue[0]);
            this->deleteTargetQueue(0);
            this->fail_goal_count = 0;
        }
        else {
            this->setCurrentPosition("Current Position");
            this->fail_goal_count++;
        }
        
    }

    std::cout << "Goal Finished Or Cancelled by Joy" << std::endl;
    this->isDoneGoal = true;
}

void Navigation::goalActiveCallback(){}

void Navigation::goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback){}

bool Navigation::addTargetService(miltbot_system::AddTarget::Request &req,
                            miltbot_system::AddTarget::Response &res) {
    MoveBaseGoalData data(req.waypoint.name, req.waypoint.goal, req.waypoint.building, req.waypoint.floor, req.task);
    this->addTargetQueue(data);
    // req.priority;
    return true;
}

void Navigation::sendStateRequest(std::string state_request) {
    ROS_INFO("Send State Request: %s",state_request.c_str());
    miltbot_state::SetRobotState srv;
    srv.request.req = state_request;
    if (this->set_robot_state_client_.call(srv))
    {
        this->current_state = srv.response.res;
    }
    else
    {
        ROS_ERROR("Failed to call service set_robot_state");
    }
}

bool Navigation::sendWaypointRequest(std::string building, std::string building_floor) {
    ros::ServiceClient client = nh_.serviceClient<miltbot_map::GetWaypointList>(this->get_waypoint_list_service_name_);
    miltbot_map::GetWaypointList srv;
    srv.request.building = building;
    srv.request.floor = building_floor;
    std::vector<miltbot_map::Waypoint> waypoints;
    if(client.call(srv)) {
        waypoints = srv.response.waypoints;
        if(waypoints.size() > 0) {
            this->building = building;
            this->building_floor = building_floor;
            // this->building_floor_num = this->toint(this->substrBuildingFloor(building_floor));
            this->setWaypoint(waypoints);
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

void Navigation::setWaypoint(std::vector<miltbot_map::Waypoint> waypoints) {
    for(int i = 0; i < waypoints.size(); i++) {
           MoveBaseGoalData data;
           data.setGoalName(waypoints[i].name);
           data.setBuilding(waypoints[i].building);
           data.setBuildingFloor(waypoints[i].floor);
           data.setGoal(waypoints[i].goal);
           this->lifts.push_back(data);
    } 
}

}

