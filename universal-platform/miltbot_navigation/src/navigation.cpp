#include "miltbot_navigation/navigation.h"

namespace miltbot {

Navigation::Navigation(std::string base_frame_id, std::string robot_frame_id, std::string building, std::string building_floor):
    base_frame_id("map"),
    robot_frame_id("base_footprint"),
    clear_costmap_service_name_("move_base/clear_costmaps"),
    get_waypoint_list_service_name_("/get_waypoint_list"),
    move_base_topic_name_("move_base"),
    move_base_wait_time_(2.0),
    view_target_queue_service_name_("view_target_queue"), 
    add_target_service_name_("add_target"),
    delete_target_service_name_("delete_target"),
    add_default_target_service_name_("add_default_target"),
    fail_goal_value_(2),
    set_robot_state_service_name_("set_robot_state"),
    run_gmapping_service_name_("run_gmapping"),
    set_map_service_name_("set_map_service"),
    get_middle_range_service_name_("get_middle_range"),
    run_system_service_name_("run_system"),
    run_transportation_service_name_("run_transportation"),
    ac("move_base", true) 
{
    nh_.param("clear_costmap_service", clear_costmap_service_name_, clear_costmap_service_name_);
    nh_.param("/get_waypoint_list_service", get_waypoint_list_service_name_, get_waypoint_list_service_name_);
    nh_.param("navigation_node/move_base_wait_time", move_base_wait_time_, move_base_wait_time_);
    nh_.param("run_gmapping_service", run_gmapping_service_name_, run_gmapping_service_name_);
    nh_.param("set_map_service", set_map_service_name_, set_map_service_name_);
    nh_.param("get_middle_range_service", get_middle_range_service_name_, get_middle_range_service_name_);
    nh_.param("view_target_queue_service", view_target_queue_service_name_, view_target_queue_service_name_);
    nh_.param("add_target_service", add_target_service_name_, add_target_service_name_);
    nh_.param("delete_target_service", delete_target_service_name_, delete_target_service_name_);
    nh_.param("add_default_target_service", add_default_target_service_name_, add_default_target_service_name_);
    nh_.param("run_system_service", run_system_service_name_, run_system_service_name_);
    nh_.param("run_transportation_service", run_transportation_service_name_, run_transportation_service_name_);
    
    this->clear_costmap_client_ = nh_.serviceClient<std_srvs::Empty>(clear_costmap_service_name_);
    this->set_robot_state_client_ = nh_.serviceClient<miltbot_state::SetRobotState>(set_robot_state_service_name_);
    this->run_gmapping_client_ = nh_.serviceClient<icreate_navigation::RunGmappingService>(run_gmapping_service_name_);
    this->get_middle_range_client_ = nh_.serviceClient<icreate_lift_navigation::GetMiddleRange>(get_middle_range_service_name_);
    this->set_map_service_client_ = nh_.serviceClient<miltbot_map::SetMap>(set_map_service_name_);
    this->run_transportation_client_ = nh_.serviceClient<miltbot_transportation::RunTransportation>(run_transportation_service_name_);
    this->base_frame_id = base_frame_id;
    this->robot_frame_id = robot_frame_id;
    this->building = building;
    this->building_floor = building_floor;
    this->building_floor_lift = building_floor + " Lift";
    this->current_state = "IDLE";
    this->navigation_case = -1;
    this->isSystemWorking = false;
    this->isDoneGoal = true;
    this->isLiftNavigation = false;
    this->done_goal_number = -1;
    this->fail_goal_count = 0;
    this->lift_navigation_step = 0;
    this->mid_range = 0;
    this->lifts.clear();
    this->default_queue.clear();
    this->waitMoveBaseServer(move_base_wait_time_);
    this->setCurrentPosition("Current Position");
    this->sendWaypointRequest(building,building_floor + " Lift");
    this->view_target_queue_server = nh_.advertiseService(this->view_target_queue_service_name_, &Navigation::viewTargetQueueService, this);
    this->add_target_service_server = nh_.advertiseService(this->add_target_service_name_, &Navigation::addTargetService, this);
    this->delete_target_service_server = nh_.advertiseService(this->delete_target_service_name_, &Navigation::deleteTargetService, this);
    this->add_default_target_service_server = nh_.advertiseService(this->add_default_target_service_name_, &Navigation::addDefaultTargetService, this);
    this->run_system_service_server = nh_.advertiseService(this->run_system_service_name_, &Navigation::runSystemService, this);
}

Navigation::~Navigation(void) 
{

}

void Navigation::addTargetQueue(miltbot_common::Waypoint data) {
    this->target_queue.push_back(data);
    ROS_WARN("%ld",data.id);
    ROS_INFO("Target Queue: %ld", target_queue.size());
}

void Navigation::deleteTargetQueue(long id) {
    // if(this->target.id == id) {
    //     std::cout << "Cannot delete this queue. it already run" << std::endl;
    // }
    // else {
        for(int i = 0;i < target_queue.size(); i++) {
            ROS_WARN("%ld",this->target_queue[i].id);
            if(target_queue[i].id == id) {
                this->target_queue.erase(this->target_queue.begin() + i);
                break;
            }
        }
        ROS_INFO("Target Queue: %ld", target_queue.size());
    // }
    
}

void Navigation::addDefaultTargetQueue(miltbot_common::Waypoint data) {
    this->default_queue.push_back(data);
    ROS_INFO("Default Queue: %ld", default_queue.size());
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

    this->currentPosition.name = current_position_name;
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

bool Navigation::setCurrentPosition(miltbot_common::Waypoint current_position) {
    this->currentPosition = current_position;
}

 miltbot_common::Waypoint Navigation::getCurrentPosition() {
    return this->currentPosition;
}

void Navigation::setRobotTarget(miltbot_common::Waypoint data) {
    // ROS_INFO("setRobotTarget: %s",data.getGoalName().c_str());
    this->target = data;
}

miltbot_common::Waypoint Navigation::getRobotTarget() {
    // ROS_INFO("get Robot Target: %s",this->target.getGoalName().c_str());
    return this->target;
}

void Navigation::setRobotGoal(std::string frame_id) {
    this->goal = this->target.goal; 
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
    ROS_DEBUG("%s %s",this->currentPosition.name.c_str(), this->target_queue[0].name.c_str());
    ROS_DEBUG("%s %s",this->currentPosition.building.c_str(), this->target_queue[0].building.c_str());
    ROS_DEBUG("%s %s",this->currentPosition.building_floor.c_str(), this->target_queue[0].building_floor.c_str());
    ROS_DEBUG("%s %s",this->currentPosition.task.c_str(), this->target_queue[0].task.c_str());
    if(flag1 && flag2) {
        ROS_INFO("Verify Target : case 0");
        this->navigation_case = Navigation::ONSAMEFLOOR;
        this->isLiftNavigation = false;
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
                ROS_INFO("Before Check");
                this->verifyTarget();
                if(this->isLiftNavigation) {
                    ROS_INFO("After Check Yes");
                    this->runLiftNavigation();
                }
                else {
                    ROS_INFO("After Check Non");
                    this->setRobotTarget(this->target_queue[0]);
                    this->setRobotGoal(this->base_frame_id);
                    this->sendStateRequest(this->target_queue[0].task);
                    this->runMoveBase();
                }
                 
            }
            else if(this->target_queue.size() == 0) {
                //สั่งให้หุ่นยนต์กลับไปยังจุด base station
                if(this->default_queue.size() > 0) {
                    this->target_queue = this->default_queue;
                }
                else {

                }
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
        // ROS_WARN("Navigation System is shuted down"); 
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
            miltbot_common::Waypoint data = this->lifts.back();
            data.task = "USINGLIFT";
            this->target_queue.insert(this->target_queue.begin(), data);
            this->setRobotTarget(this->target_queue[0]);
            this->sendStateRequest(this->target_queue[0].task);
            this->setRobotGoal(this->base_frame_id);
            this->runMoveBase();
            break;
        }
        //Step 1: Wait & Move to in front of the inncoming lift
        case 1: {
            int liftNumber = waitForIncomingLift();
            miltbot_common::Waypoint data = this->lifts[liftNumber];
            data.task = "USINGLIFT";
            this->target_queue.insert(this->target_queue.begin(), data);
            this->setRobotTarget(this->target_queue[0]);
            this->setRobotGoal(this->base_frame_id);
            this->sendStateRequest(this->target_queue[0].task);
            this->runMoveBase();
            break;
        }
        //Step 2: Wait Lift Door Open & Move inside the lift
        case 2: {
            bool flag;
            while(ros::ok()) {
                if(!this->verifyLiftDoor()) {
                    int liftNumber = waitForIncomingLift();
                    miltbot_common::Waypoint data = this->lifts[liftNumber];
                    data.task = "USINGLIFT";
                    this->target_queue.insert(this->target_queue.begin(), data);
                    this->setRobotTarget(this->target_queue[0]);
                    this->setRobotGoal(this->base_frame_id);
                    this->sendStateRequest(this->target_queue[0].task);
                    this->runMoveBase();
                    this->lift_navigation_step--;
                    break;
                }
                icreate_navigation::RunGmappingService srv;
                srv.request.task = "open";
                if(run_gmapping_client_.call(srv)) {
                    flag = srv.response.success;
                }
                else {
                    ROS_WARN("Failed to run gmapping");
                    continue;
                }
                break;
            }
            if(flag) {
                initializeLiftForwardMoveBase();
            }
            break;
        }
        case 3: {
            initializeLiftRotateMoveBase();
            break;
        }
        case 4: {
            waitUserInputLift();
            bool flag;
            while(ros::ok()) {
                icreate_navigation::RunGmappingService srv;
                srv.request.task = "restart";
                if(run_gmapping_client_.call(srv)) {
                    flag = srv.response.success;
                }
                else {
                    ROS_WARN("Failed to run gmapping");
                    continue;
                }
                break;
            }
            if(flag) {
                initializeLiftForwardOutMoveBase();
            }
            break;
        }
        case 5: {
            bool flag = false;
            icreate_navigation::RunGmappingService srv;
            // srv.request.task = "close";
            // if(run_gmapping_client_.call(srv)) {
            //     flag = srv.response.success;
            // }
            // else {
            //     ROS_WARN("Failed to run gmapping");
            // }
            // if(flag) {
            //     this->navigation_case = 0;
            //     miltbot_map::SetMap srv2;
            //     //Set out lift position
            //     srv2.request.floor = this->target_queue[0].building_floor + " Lift";
            //     srv2.request.target_number = this->lift_number;
            //     if(set_map_service_client_.call(srv2)) {
            //         bool flag2 = srv2.response.flag;
            //     }
            //     else {
            //        ROS_WARN("Failed to run set map");
            //     }
            // }
            break;
        }
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

bool Navigation::verifyTargetBuilding(miltbot_common::Waypoint current, miltbot_common::Waypoint target) {
    if(current.building == target.building)
        return true;
    else 
        return false;
}

bool Navigation::verifyTargetFloor(miltbot_common::Waypoint current, miltbot_common::Waypoint target) {
    if(current.building_floor == target.building_floor)
        return true;
    else 
        return false;
}

void Navigation::displayLiftWaypoints() {
    for(int i = 0 ; i < this->lifts.size() ; i++) {
      std::cout <<"["<<i<<"] " << this->lifts[i].name << " " <<std::endl; 
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
        std::cout << "[AGENT] Wait For Number of Lift : ";
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

bool Navigation::waitUserInputLift() {
    bool flag;
    while(ros::ok()) {
        std::cout << "Waiting for lift stop on the target floor" << std::endl;
        std::cout << "Please press \"y\" to start navigation " << std::endl;
        // std::cout << "or Please press \"n\" to stop navigation " << std::endl;
        std::string input;
        std::cin >> input;
        if(input == "y" || input == "Y") {
            flag = true;
            break;
        }
        // else if(input == "n" || input == "N") {
        //     flag = false;
        //     break;
        // }
        else {
            std::cout << "Wrong Input Please Try Again" << std::endl;
        }
    }
    return flag;
}

void Navigation::initializeLiftForwardMoveBase() {
    icreate_lift_navigation::GetMiddleRange srv;
    if(get_middle_range_client_.call(srv)) {
        this->mid_range = srv.response.mid_range;
        ROS_INFO("Get Mid Range data: %f",mid_range);
        this->mid_range -= 0.5;
        move_base_msgs::MoveBaseGoal new_point;
        new_point.target_pose.pose.position.x = mid_range;
        new_point.target_pose.pose.orientation.w = 1;
        miltbot_common::Waypoint data;
        data.name = "Going To Lift";
        data.building = this->building;
        data.building_floor = this->building_floor_lift;
        data.goal = new_point;
        data.task = "USINGLIFT";
        this->target_queue.insert(this->target_queue.begin(),data);
        this->setRobotTarget(this->target_queue[0]);
        this->sendStateRequest(this->target_queue[0].task);
        this->setRobotGoal(this->robot_frame_id);
        this->runMoveBase();
    }
    else {
        ROS_ERROR("Fail to call Service get_depth_distance");
    }
}

void Navigation::initializeLiftRotateMoveBase() {
    move_base_msgs::MoveBaseGoal new_point;
    new_point.target_pose.pose.orientation.z = -1.0;
    new_point.target_pose.pose.orientation.w = 0.0;
    miltbot_common::Waypoint data;
    data.name = "Rotate In Lift";
    data.building = this->building;
    data.building_floor = this->building_floor_lift;
    data.goal = new_point;
    data.task = "USINGLIFT";
    this->target_queue.insert(this->target_queue.begin(), data);
    this->setRobotTarget(this->target_queue[0]);
    this->sendStateRequest(this->target_queue[0].task);
    this->setRobotGoal(this->robot_frame_id);
    this->runMoveBase();
}

void Navigation::initializeLiftForwardOutMoveBase() {
    this->mid_range += 0.5;
    move_base_msgs::MoveBaseGoal new_point;
    new_point.target_pose.pose.position.x = mid_range;
    new_point.target_pose.pose.orientation.w = 1;
    miltbot_common::Waypoint data;
    data.name = "Going Out Lift";
    data.building = this->building;
    data.building_floor = this->building_floor_lift;
    data.goal = new_point;
    data.task = "USINGLIFT";
    this->target_queue.insert(this->target_queue.begin(),data);
    this->setRobotTarget(this->target_queue[0]);
    this->sendStateRequest(this->target_queue[0].task);
    this->setRobotGoal(this->robot_frame_id);
    this->runMoveBase();
}

void Navigation::goalDoneCallback(const actionlib::SimpleClientGoalState &state, 
  const move_base_msgs::MoveBaseResultConstPtr &result){
    ROS_INFO("Goal Done Now !!!!!");
    this->clearCostmap();
    if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("SUCCEEDED");
        done_goal_number = 1;
        if(this->target_queue.size() > 0) {
            if(this->target_queue[0].task == "USINGLIFT") {
                this->lift_navigation_step++;
            }
            else if(this->target_queue[0].task == "GOING") {
                this->callRunTransportationService("receive");
            }
            else if(this->target_queue[0].task == "SENDSUPPLIES") {
                this->callRunTransportationService("send");
            }
            // robot.setStartPosition(robot.target_queue[0]);
            this->setCurrentPosition(this->target_queue[0]);
            this->deleteTargetQueue(this->target_queue[0].id);

            std::string state_request;
            if(this->current_state == "SINGLERUN") {
                state_request = "IDLE";
            }
            else if(this->current_state == "GOING") {
                state_request = "WAITING";
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
            ROS_DEBUG("Change Task");
            this->setCurrentPosition(this->target_queue[0]);
            this->deleteTargetQueue(this->target_queue[0].id);
            this->fail_goal_count = 0;
        }
        else {
            ROS_DEBUG("Still Do Task");
            this->setCurrentPosition("Current Position");
            this->fail_goal_count++;
        }
        
    }
    else {
        ROS_WARN("UNKNOWN");
    }

    std::cout << "Goal Finished Or Cancelled by Joy" << std::endl;
    this->isDoneGoal = true;
}

void Navigation::goalActiveCallback(){}

void Navigation::goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback){}

bool Navigation::viewTargetQueueService(miltbot_system::ViewTargetQueue::Request &req,
                                miltbot_system::ViewTargetQueue::Response &res) {
    res.target_queue = this->target_queue;
    return true;
}

bool Navigation::addTargetService(miltbot_system::AddTarget::Request &req,
                            miltbot_system::AddTarget::Response &res) {
    this->addTargetQueue(req.waypoint);
    res.success = true;
    return true;
}

bool Navigation::deleteTargetService(miltbot_system::DeleteTarget::Request &req,
                            miltbot_system::DeleteTarget::Response &res) {
    this->deleteTargetQueue(req.id);
    res.success = true;
    return true;
}

bool Navigation::addDefaultTargetService(miltbot_system::AddTarget::Request &req,
                            miltbot_system::AddTarget::Response &res) {
    this->addDefaultTargetQueue(req.waypoint);
    res.success = true;
    return true;
}

bool Navigation::runSystemService(miltbot_system::RunSystem::Request &req,
                            miltbot_system::RunSystem::Response &res) {
    this->isSystemWorking = req.status;
    res.success = true;
    return true;
}

void Navigation::callRunTransportationService(std::string mode) {
    miltbot_transportation::RunTransportation srv;
    srv.request.mode = mode;
    if(run_transportation_client_.call(srv)) {
        bool flag = srv.response.success;
    } 
    else {
        ROS_ERROR("Failed to call service run_transportation");
    }
}

void Navigation::sendStateRequest(std::string state_request) {
    ROS_INFO("Send State Request: %s",state_request.c_str());
    miltbot_state::SetRobotState srv;
    srv.request.req = state_request;
    if (this->set_robot_state_client_.call(srv)) {
        this->current_state = srv.response.res;
    }
    else {
        ROS_ERROR("Failed to call service set_robot_state");
    }
}

bool Navigation::sendWaypointRequest(std::string building, std::string building_floor) {
    ros::ServiceClient client = nh_.serviceClient<miltbot_map::GetWaypointList>(this->get_waypoint_list_service_name_);
    miltbot_map::GetWaypointList srv;
    srv.request.building = building;
    srv.request.floor = building_floor;
    std::vector<miltbot_common::Waypoint> waypoints;
    if(client.call(srv)) {
        waypoints = srv.response.waypoints;
        if(waypoints.size() > 0) {
            this->building = building;
            this->building_floor = building_floor;
            // this->building_floor_num = this->toint(this->substrBuildingFloor(building_floor));
            // this->setWaypoint(waypoints);
            this->lifts = waypoints;
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

void Navigation::setWaypoint(std::vector<miltbot_common::Waypoint> waypoints) {
    for(int i = 0; i < waypoints.size(); i++) {
        //    miltbot_common::Waypoint data;
        //    data.setGoalName(waypoints[i].name);
        //    data.setBuilding(waypoints[i].building);
        //    data.setBuildingFloor(waypoints[i].floor);
        //    data.setGoal(waypoints[i].goal);
        //    this->lifts.push_back(data);
    } 
}

}

