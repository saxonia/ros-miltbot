#include "miltbot_navigation/navigation.h"

namespace miltbot {

Navigation::Navigation(std::string base_frame_id, std::string robot_frame_id, std::string building, std::string building_floor):
    base_frame_id("map"),
    robot_frame_id("base_footprint"),
    move_base_topic_name_("move_base"),
    move_base_wait_time_(2.0),
    move_base_cancel_pub_topic_name_("move_base/cancel"),
    target_queue_pub_topic_name_("target_queue"),
    navigation_state_pub_topic_name_("navigation_state"),
    clear_costmap_service_name_("move_base/clear_costmaps"),
    get_waypoint_list_service_name_("/get_waypoint_list"),
    view_target_queue_service_name_("view_target_queue"), 
    add_target_service_name_("add_target"),
    delete_target_service_name_("delete_target"),
    add_default_target_service_name_("add_default_target"),
    delete_default_target_service_name_("delete_default_target"),
    fail_goal_value_(2),
    set_robot_state_service_name_("set_robot_state"),
    run_gmapping_service_name_("run_gmapping"),
    set_map_service_name_("set_map_service"),
    get_middle_range_service_name_("get_middle_range"),
    run_system_service_name_("run_system"),
    run_charging_navigation_service_name_("run_charging_navigation"),
    run_transportation_service_name_("run_transportation"),
    is_front_lift_service_name_("is_front_lift"),
    is_lift_open_service_name_("is_lift_open"),
    ac("move_base", true) 
{
    nh_.param("navigation_node/move_base_wait_time", move_base_wait_time_, move_base_wait_time_);
    nh_.param("move_base_cancel_pub_topic", move_base_cancel_pub_topic_name_, move_base_cancel_pub_topic_name_);
    nh_.param("navigation_state_pub_topic", navigation_state_pub_topic_name_, navigation_state_pub_topic_name_);
    nh_.param("target_queue_pub_topic", target_queue_pub_topic_name_, target_queue_pub_topic_name_);
    nh_.param("clear_costmap_service", clear_costmap_service_name_, clear_costmap_service_name_);
    nh_.param("get_waypoint_list_service", get_waypoint_list_service_name_, get_waypoint_list_service_name_);
    nh_.param("run_gmapping_service", run_gmapping_service_name_, run_gmapping_service_name_);
    nh_.param("set_map_service", set_map_service_name_, set_map_service_name_);
    nh_.param("get_middle_range_service", get_middle_range_service_name_, get_middle_range_service_name_);
    nh_.param("view_target_queue_service", view_target_queue_service_name_, view_target_queue_service_name_);
    nh_.param("add_target_service", add_target_service_name_, add_target_service_name_);
    nh_.param("delete_target_service", delete_target_service_name_, delete_target_service_name_);
    nh_.param("add_default_target_service", add_default_target_service_name_, add_default_target_service_name_);
    nh_.param("run_system_service", run_system_service_name_, run_system_service_name_);
    nh_.param("run_charging_navigation_service", run_charging_navigation_service_name_, run_charging_navigation_service_name_);
    nh_.param("run_transportation_service", run_transportation_service_name_, run_transportation_service_name_);

    this->base_frame_id = base_frame_id;
    this->robot_frame_id = robot_frame_id;
    this->building = building;
    this->building_floor = building_floor;
    this->building_floor_lift = building_floor + " Lift";
    this->current_state = "IDLE";
    this->navigation_case = -1;
    this->isSystemWorking = false;
    this->isNormalNavigation = true;
    this->isChargingNavigation = false;
    this->isSystemRecoveryNavigation = false;
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
    this->move_base_cancel_pub = nh_.advertise<actionlib_msgs::GoalID>(move_base_cancel_pub_topic_name_, 1);
    this->target_queue_pub = nh_.advertise<miltbot_common::WaypointList>(target_queue_pub_topic_name_, 1);
    this->navigation_state_pub = nh_.advertise<miltbot_navigation::NavigationState>(navigation_state_pub_topic_name_, 1);
    this->view_target_queue_server = nh_.advertiseService(this->view_target_queue_service_name_, &Navigation::viewTargetQueueService, this);
    this->add_target_service_server = nh_.advertiseService(this->add_target_service_name_, &Navigation::addTargetService, this);
    this->delete_target_service_server = nh_.advertiseService(this->delete_target_service_name_, &Navigation::deleteTargetService, this);
    this->add_default_target_service_server = nh_.advertiseService(this->add_default_target_service_name_, &Navigation::addDefaultTargetService, this);
    this->delete_default_target_service_server = nh_.advertiseService(this->delete_default_target_service_name_, &Navigation::deleteDefaultTargetService, this);
    this->run_system_service_server = nh_.advertiseService(this->run_system_service_name_, &Navigation::runSystemService, this);
    this->run_charging_navigation_service_server = nh_.advertiseService(this->run_charging_navigation_service_name_, &Navigation::runChargingNavigationService, this);
    this->clear_costmap_client_ = nh_.serviceClient<std_srvs::Empty>(clear_costmap_service_name_);
    this->set_robot_state_client_ = nh_.serviceClient<miltbot_state::SetRobotState>(set_robot_state_service_name_);
    this->run_gmapping_client_ = nh_.serviceClient<icreate_navigation::RunGmappingService>(run_gmapping_service_name_);
    this->get_middle_range_client_ = nh_.serviceClient<miltbot_navigation::GetMiddleRange>(get_middle_range_service_name_);
    this->set_map_service_client_ = nh_.serviceClient<miltbot_map::SetMap>(set_map_service_name_);
    this->run_transportation_client_ = nh_.serviceClient<miltbot_transportation::RunTransportation>(run_transportation_service_name_);
    this->is_front_lift_client_ = nh_.serviceClient<miltbot_vision::IsFrontLift>(is_front_lift_service_name_);
    this->is_lift_open_client_ = nh_.serviceClient<miltbot_navigation::IsLiftOpen>(is_lift_open_service_name_);
}

Navigation::~Navigation(void) 
{

}

void Navigation::addTargetQueue(miltbot_common::Waypoint data) {
    this->target_queue.push_back(data);
    ROS_INFO("Target Queue Size: %ld", target_queue.size());
}

void Navigation::deleteTargetQueue(long id) {
    if(this->target.id == id) {
        std::cout << "Cannot delete this queue. it already run" << std::endl;
        this->sendMoveBaseCancel();  
    }
    for(int i = 0;i < target_queue.size(); i++) {
        if(target_queue[i].id == id) {
            this->target_queue.erase(this->target_queue.begin() + i);
            break;
        }
    }
    ROS_INFO("Target Queue Size: %ld", target_queue.size());
}

void Navigation::addDefaultTargetQueue(miltbot_common::Waypoint data) {
    this->default_queue.push_back(data);
    ROS_INFO("Default Queue Size: %ld", default_queue.size());
}

void Navigation::deleteDefaultTargetQueue(long id) {
    if(this->target.id == id) {
        std::cout << "Cannot delete this queue. it already run" << std::endl;
        this->sendMoveBaseCancel();  
    }
    for(int i = 0;i < default_queue.size(); i++) {
        if(default_queue[i].id == id) {
            this->default_queue.erase(this->default_queue.begin() + i);
            break;
        }
    }
    ROS_INFO("Default Queue Size: %ld", default_queue.size());
}

void Navigation::addChargingQueue(miltbot_common::Waypoint data) {
    this->charging_queue.push_back(data);
    ROS_INFO("Charging Queue Size: %ld", charging_queue.size());
}

void Navigation::deleteChargingQueue(long id) {
    if(this->target.id == id) {
        std::cout << "Cannot delete this queue. it already run" << std::endl;
        this->sendMoveBaseCancel();  
    }
    for(int i = 0;i < charging_queue.size(); i++) {
        if(charging_queue[i].id == id) {
            this->charging_queue.erase(this->charging_queue.begin() + i);
            break;
        }
    }
    ROS_INFO("Charging Queue Size: %ld", charging_queue.size());
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
    this->currentPosition.building = this->building;
    this->currentPosition.building_floor = this->building_floor;
    this->currentPosition.goal.target_pose.pose.position.x    = transform.getOrigin().x();
    this->currentPosition.goal.target_pose.pose.position.y    = transform.getOrigin().y();
    this->currentPosition.goal.target_pose.pose.orientation.x = transform.getRotation().x();
    this->currentPosition.goal.target_pose.pose.orientation.y = transform.getRotation().y();
    this->currentPosition.goal.target_pose.pose.orientation.z = transform.getRotation().z();
    this->currentPosition.goal.target_pose.pose.orientation.w = transform.getRotation().w();
    
    
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
    ROS_INFO("%s %s",this->currentPosition.name.c_str(), this->target_queue[0].name.c_str());
    ROS_INFO("%s %s",this->currentPosition.building.c_str(), this->target_queue[0].building.c_str());
    ROS_INFO("%s %s",this->currentPosition.building_floor.c_str(), this->target_queue[0].building_floor.c_str());
    ROS_INFO("%s %s",this->currentPosition.task.c_str(), this->target_queue[0].task.c_str());
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
    this->updateTargetQueue();
    this->updateNavigationState();
    if(this->isSystemWorking) {
        // ROS_INFO("Navigation System is stil working");
        //เช็คว่าทำงานเสร็จรึยัง
        if(this->isNormalNavigation) {
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
        else if(this->isChargingNavigation) {
            //อย่าลืมไปเพิ่ม topic หยุดเดิน ถ้าหุ่นไม่หยุด
            ROS_INFO("GO %d",this->isDoneGoal);
            if(this->isDoneGoal) {
                this->isDoneGoal = false;
                if(this->charging_queue.size() > 0) {
                    //เช็คว่าจุดหมายที่จะไปอยู่ชั้นเดียวกันมั้ย ถ้าอยู่ชั้นเดียวกันก็เซ็ตเลย ถ้าไม่ก็เซ็ต target ไปอยู่ที่ส่วนของ Lift
                    ROS_INFO("Before Check");
                    // this->verifyTarget();
                    // if(this->isLiftNavigation) {
                        // ROS_INFO("After Check Yes");
                        // this->runLiftNavigation();
                    // }
                    // else {
                        ROS_INFO("After Check Non");
                        this->setRobotTarget(this->charging_queue[0]);
                        this->setRobotGoal(this->base_frame_id);
                        this->sendStateRequest(this->charging_queue[0].task);
                        this->runMoveBase();
                    // }
                }
                else if(this->charging_queue.size() == 0) {
                    ROS_INFO("Zero Queue");
                    this->isDoneGoal = true;
                }
                else {
                    return false;
                }
            }
            else {
                //ระบบยังทำงาน ไม่เสร็จ ปล่อยผ่านไป
                ROS_WARN("None");
            }
            this->isChargingNavigation = false;
        }
        else if(this->isSystemRecoveryNavigation) {

        }
    }
    else {
        //ระบบหยุดทำงาน ทำอะไรต่อ ?
        // ROS_WARN("Navigation System is shuted down"); 
    }
    return true;
}

void Navigation::updateTargetQueue() {
    miltbot_common::WaypointList waypoint_list;
    waypoint_list.waypoints = this->target_queue;
    this->target_queue_pub.publish(waypoint_list);
}

void Navigation::updateNavigationState() {
    miltbot_navigation::NavigationState navigation_state;
    navigation_state.task = "";
    navigation_state.building = this->building;
    navigation_state.building_floor = this->building_floor;
    this->navigation_state_pub.publish(navigation_state);
}

void Navigation::runMoveBase() {
    ac.sendGoal(goal, 
                boost::bind(&Navigation::goalDoneCallback, this, _1, _2), 
                boost::bind(&Navigation::goalActiveCallback, this), boost::bind(&Navigation::goalFeedbackCallback, this, _1));
}

void Navigation::runLiftNavigation() {
    ROS_INFO("Into Lift Navigation");
    switch(this->lift_navigation_step) {
        //Step 0: Move to Lift Ground
        case 0: {
            ROS_INFO("Lift Navigation: Step 0");
            // this->isDoneGoal = true;
            // this->lift_navigation_step++;
            // break;
            miltbot_common::Waypoint data = this->lifts.back();
            data.task = "USINGLIFT";
            this->target_queue.insert(this->target_queue.begin(), data);
            this->setRobotTarget(this->target_queue[0]);
            this->sendStateRequest(this->target_queue[0].task);
            this->setRobotGoal(this->base_frame_id);
            this->runMoveBase();
            break;
        }
        //Step 1: Wait & Move to in front of the incoming lift
        case 1: {
            ROS_INFO("Lift Navigation: Step 2");
            this->target_number = waitForIncomingLift();
            // this->isDoneGoal = true;
            // this->lift_navigation_step++;
            // break;
            miltbot_common::Waypoint data = this->lifts[this->target_number];
            data.task = "USINGLIFT";
            this->target_queue.insert(this->target_queue.begin(), data);
            this->setRobotTarget(this->target_queue[0]);
            this->setRobotGoal(this->base_frame_id);
            this->sendStateRequest(this->target_queue[0].task);
            this->runMoveBase();
            break;
        }
        //Step 2:Verify Robot is in front of lift door
        case 2: {
            ROS_INFO("Lift Navigation: Step 2");
            if(this->verifyFrontDoor()) {
                ROS_INFO("Verify Front Lift Door OK");
                miltbot_navigation::GetMiddleRange srv;
                if(get_middle_range_client_.call(srv)) {
                    this->mid_range = srv.response.mid_range;
                }
                else {
                    ROS_ERROR("Fail to call Service get_depth_distance");
                }
                this->lift_navigation_step++;
            }
            else {
                ROS_INFO("Verify Front Lift Door Fail");
                this->lift_navigation_step = 0;
            }
            this->isDoneGoal = true;
        }
        //Step 3: Wait Lift Door Open & Move inside the lift
        case 3: {
            ROS_INFO("Lift Navigation: Step 3");
            bool flag = false;
            int verify_door_fail = 0;
            while(ros::ok()) {
                if(verify_door_fail == 10000) {
                    ROS_WARN("Verify Lift Door Fail");
                    this->lift_navigation_step = 1;
                    this->isDoneGoal = true;
                    break;
                }
                if(!this->verifyLiftDoor()) {
                    verify_door_fail++;
                    continue;
                }
                ROS_INFO("Verify Door Open OK");
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
                // this->isDoneGoal = true;
                // this->lift_navigation_step++;
                // break;
                initializeLiftForwardMoveBase();
            }
            break;
        }
        case 4: {
            ROS_INFO("Lift Navigation: Step 4");
            // this->waitUserInputLift();
            // this->isDoneGoal = true;
            // this->lift_navigation_step++;
            // break;
            this->initializeLiftRotateMoveBase();
            break;
        }
        case 5: {
            ROS_INFO("Lift Navigation: Step 5");
            // this->waitUserInputLift();
            bool flag = false;
            int verify_door_fail = 0;
            while(ros::ok()) {
                if(verify_door_fail == 10000) {
                    ROS_WARN("Verify Lift Door Fail");
                    this->lift_navigation_step = 1;
                    this->isDoneGoal = true;
                    break;
                }
                if(!this->verifyLiftDoor()) {
                    verify_door_fail++;
                    continue;
                }
                ROS_INFO("Verify Door Open OK");
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
                // this->isDoneGoal = true;
                // this->lift_navigation_step++;
                // break;
                initializeLiftForwardOutMoveBase();
            }
            break;
        }
        case 6: {
            ROS_INFO("Lift Navigation: Step 6");
            // this->waitUserInputLift();
            bool flag = false;
            icreate_navigation::RunGmappingService srv;
            srv.request.task = "close";
            if(run_gmapping_client_.call(srv)) {
                flag = srv.response.success;
            }
            else {
                ROS_WARN("Failed to run gmapping");
            }
            if(flag) {
                this->navigation_case = 0;
                miltbot_map::SetMap srv2;
                //Set out lift position
                srv2.request.floor = this->target_queue[0].building_floor + " Lift";
                srv2.request.target_number = this->target_number;
                ROS_WARN("Target Building Floor : %s",this->target_queue[0].building_floor.c_str());
                ROS_WARN("Target Number : %d",this->target_number);
                if(set_map_service_client_.call(srv2)) {
                    bool flag2 = srv2.response.flag;
                    this->building = this->target_queue[0].building;
                    this->building_floor = this->target_queue[0].building_floor;
                    this->lift_navigation_step = 0;
                    this->isDoneGoal = true;
                    this->setCurrentPosition("Current Position");
                }
                else {
                   ROS_WARN("Failed to run set map");
                }
            }
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

bool Navigation::verifyFrontDoor() {
    miltbot_vision::IsFrontLift srv;
    if(this->is_front_lift_client_.call(srv)) {
        return srv.response.is_front_lift;
    }
    else {
        ROS_ERROR("Fail to call Service is_front_lift");
        return false;
    }
}

bool Navigation::verifyLiftDoor() {
    // std::cout << "Wait For Verifying Lift Door" << std::endl;
    // std::cout << "Press c to cancel or any key to continue" << std::endl;
    // std::cout << "Your input : ";
    // std::string in;
    // std::cin >> in;
    // if(in == "c") {
    //     return false;
    // }
    // return true;
    miltbot_navigation::IsLiftOpen srv;
    srv.request.mid_range = this->mid_range;
    if(this->is_lift_open_client_.call(srv)) {
        return srv.response.is_lift_open;
    }
    else {
        ROS_ERROR("Fail to call Service is_lift_open");
        return false;
    }
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
    miltbot_navigation::GetMiddleRange srv;
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
        if(this->isNormalNavigation) {
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
                else {
                    ROS_WARN("No Current State");
                }
                this->sendStateRequest(state_request);
            }
            else {
                ROS_ERROR("Error target_queue size");
            }
        }
        else if(this->isChargingNavigation) {
            this->setCurrentPosition(this->charging_queue[0]);
            this->deleteChargingQueue(this->charging_queue[0].id);
            std::string state_request = "IDLE";
            this->sendStateRequest(state_request);
        }
        else if(this->isSystemRecoveryNavigation) {

        }
        else {

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
        if(this->isNormalNavigation) {
            if(this->fail_goal_count >= this->fail_goal_value_) {
                ROS_INFO("Change Task");
                this->setCurrentPosition("Current Position");
                this->deleteTargetQueue(this->target_queue[0].id);
                this->fail_goal_count = 0;
            }
            else {
                ROS_INFO("Still Do Task");
                this->setCurrentPosition("Current Position");
                this->fail_goal_count++;
            }
        }
        else if(this->isChargingNavigation) {
            if(this->fail_goal_count >= this->fail_goal_value_) {
                ROS_INFO("Change Task");
                this->setCurrentPosition("Current Position");
                this->deleteTargetQueue(this->charging_queue[0].id);
                this->fail_goal_count = 0;
            }
            else {
                ROS_INFO("Still Do Task");
                this->setCurrentPosition("Current Position");
                this->fail_goal_count++;
            }
        }
        else if(this->isSystemRecoveryNavigation) {

        }
        else {

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
    res.default_queue = this->default_queue;
    res.charging_queue = this->charging_queue;
    return true;
}

bool Navigation::addTargetService(miltbot_system::AddTarget::Request &req,
                            miltbot_system::AddTarget::Response &res) {
    if(req.waypoint.task == "BACKTOCHARGE") {
        this->addChargingQueue(req.waypoint);
    }
    else {
        this->addTargetQueue(req.waypoint);
    }
    
    res.success = true;
    return true;
}

bool Navigation::deleteTargetService(miltbot_system::DeleteTarget::Request &req,
                            miltbot_system::DeleteTarget::Response &res) {
    this->deleteChargingQueue(req.id);
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

bool Navigation::deleteDefaultTargetService(miltbot_system::DeleteTarget::Request &req,
                            miltbot_system::DeleteTarget::Response &res) {
    this->deleteDefaultTargetQueue(req.id);
}

bool Navigation::runSystemService(miltbot_system::RunSystem::Request &req,
                            miltbot_system::RunSystem::Response &res) {
    this->isSystemWorking = req.status;
    if(req.status) {
        ROS_INFO("Start System");
    }
    else {
        ROS_INFO("Stop System");
        this->sendMoveBaseCancel();
    }
    res.success = true;
    return true;
}

bool Navigation::runChargingNavigationService(miltbot_system::RunSystem::Request &req,
                            miltbot_system::RunSystem::Response &res) {
    this->isChargingNavigation = req.status;
    ROS_INFO("got charge status %d",req.status);
    if(req.status) {
        ROS_INFO("Start Charge Process");
        this->isNormalNavigation = false;
        this->isSystemRecoveryNavigation = false;
    }
    else {
        ROS_INFO("Stop Charge Process");
        this->sendMoveBaseCancel();
        this->isNormalNavigation = true;
        this->isSystemRecoveryNavigation = true;
    }
    this->fail_goal_count = 0;
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

void Navigation::sendMoveBaseCancel() {
    ROS_WARN("Move Base Cancel Goal");
    move_base_cancel_pub.publish(*new actionlib_msgs::GoalID());
}

}

