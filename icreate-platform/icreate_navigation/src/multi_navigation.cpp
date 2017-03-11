#include "icreate_navigation/multi_navigation.h"

namespace icreate {

MultiNavigation::MultiNavigation(void): 
    run_gmapping_service_name_("run_gmapping"),
    set_map_service_name_("set_map")
{
    nh_.param("run_gmapping_service", run_gmapping_service_name_, run_gmapping_service_name_);
    nh_.param("set_map_service", set_map_service_name_, set_map_service_name_);
    this->navigations_.reserve(0);
    this->requestToSetNewGoal = false;
    this->nav_idx = -1;
    this->navigation_case = -1;
    this->lift_navigation_step = -1;
    this->lift_number = 0;
    this->floor_count_ = 0;
    this->isFinishQueue = false;
    this->mid_range = 0.0;
}

// MultiNavigation::MultiNavigation(int floor_count) {
//     this->floor_count_ = floor_count;
// }

MultiNavigation::~MultiNavigation() {

}
bool MultiNavigation::addSingleNavigation(std::string building, std::string building_floor) {
    for(int i = 0; i < this->navigations_.size(); i++) {
        std::string building_ = this->navigations_[i].building;
        std::string building_floor_ = this->navigations_[i].building_floor;
        if(building_ == building && building_floor_ == building_floor) {
            ROS_WARN("Already Have This Navigation");
            return false;
        }
    }
    icreate::SingleNavigation navigation(building, building_floor);
    this->navigations_.push_back(navigation);
    std::sort(this->navigations_.begin(),this->navigations_.end(),this->comparator);
    this->floor_count_++;
    this->nav_idx++;
    return true;
}

bool MultiNavigation::removeSingleNavigation(std::string building, std::string building_floor) {
    for(int i = 0; i < this->navigations_.size(); i++) {
        std::string building_ = this->navigations_[i].building;
        std::string building_floor_ = this->navigations_[i].building_floor;
        if(building_ == building && building_floor_ == building_floor) {
            this->navigations_.erase(this->navigations_.begin() + i);
            this->floor_count_--;
            return true;
        }
    }
    ROS_WARN("Can't Find This Navigation");
    return false;
}

int MultiNavigation::getCountNavigation() {
    return this->navigations_.size();
}

SingleNavigation MultiNavigation::getSingleNavigation(size_t idx) {
    return this->navigations_[idx];
}

void MultiNavigation::doneRobotGoal(Robot &robot) {
    this->navigations_[this->nav_idx].doneRobotGoal(robot);
    if(this->navigation_case == 0) {
        this->lift_navigation_step = 0;
    }
    else if(this->navigation_case == 1) {
        this->lift_navigation_step++;
    }
}

void MultiNavigation::setupRobotToRun(Robot &robot, std::string base_frame_id, std::string robot_frame_id) {
    // robot.setCurrentPosition(base_frame_id, robot_frame_id, robot.building, robot.building_floor);
    this->navigations_[this->nav_idx].setupRobotToRun(robot, base_frame_id, robot_frame_id);
}

bool MultiNavigation::verifyTarget(Robot &robot) {
    if(!(this->navigations_[this->nav_idx].building == robot.building 
        && this->navigations_[this->nav_idx].building_floor == robot.building_floor)) {
        for(int i = 0; i < this->navigations_.size(); i++) {
            ROS_INFO("%s %s",this->navigations_[i].building.c_str(), robot.building.c_str());
            ROS_INFO("%s %s",this->navigations_[i].building_floor.c_str(), robot.building_floor.c_str());
            if(this->navigations_[i].building == robot.building 
                && this->navigations_[i].building_floor == robot.building_floor) {
                this->nav_idx = i;
                break;
            }
        }
    }
    this->navigation_case = -1;
    bool flag1 = verifyTargetBuilding(robot.currentPosition, robot.target_queue[0]);
    ROS_INFO("%s %s",robot.currentPosition.building.c_str(), robot.target_queue[0].building.c_str());
    ROS_INFO("%s %s",robot.currentPosition.building_floor.c_str(), robot.target_queue[0].building_floor.c_str());
    bool flag2 = verifyTargetFloor(robot.currentPosition, robot.target_queue[0]);
    if(flag1 && flag2) {
        ROS_INFO("Verify Target : case 0");
        this->navigation_case = 0;
        this->navigations_[this->nav_idx].requestToSetNewGoal = true;
    }
    else if(flag1 && !flag2) {
        ROS_INFO("Verify Target : case 1");
        this->navigation_case = 1;
        runLiftNavigation(robot);
    }
    else {
        ROS_INFO("Verify Target : case 2");
        this->navigation_case = 2;
        return false;
    }
    return true;
}

bool MultiNavigation::verifyTargetBuilding(MoveBaseGoalData current, MoveBaseGoalData target) {
    if(current.getBuilding() == target.getBuilding())
        return true;
    else 
        return false;
}

bool MultiNavigation::verifyTargetFloor(MoveBaseGoalData current, MoveBaseGoalData target) {
    if(current.getBuildingFloor() == target.getBuildingFloor())
        return true;
    else 
        return false;
}

void MultiNavigation::displayBuildingFloor() {
    for(int i = 0;i < this->navigations_.size(); i++) {
        std::cout <<"["<<i<<"] " << this->navigations_[i].building_floor <<std::endl; 
    }
    std::cout <<"["<<99<<"] " << "Cancel" <<std::endl;
}

bool MultiNavigation::setupTargetQueue(Robot &robot) {
    robot.setNavigationMode();
    int navigation_mode = robot.getNavigationMode();
    int selected_point = -1;
    // this->requestToSetNewGoal = false;
    this->lift_navigation_step = 5;
    if(navigation_mode >= 0 && navigation_mode < 10) {
        std::pair<int, int> selected = this->showWaypointMenu();
        if(selected.first == 99 || selected.second == 99) {
            return false;
        }
        icreate::MoveBaseGoalData data = this->navigations_[selected.first].targets[selected.second];
        robot.addTargetQueue(data);
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
                std::pair<int, int> selected = this->showWaypointMenu();
                if(selected.first == 99 || selected.second == 99) {
                    break;
                }
                icreate::MoveBaseGoalData data = this->navigations_[selected.first].targets[selected.second];
                robot.addTargetQueue(data);
            }
            break;
        }
        //MODE : Going 1 Place and Send Supplies in n Places
	    case 6: { 
            break;
        }
        //MODE : Going n Places and Send Supplies in n Places
	    case 7: {
            while(ros::ok()) {
                std::cout << "Please insert next point !!!" << std::endl;
                std::pair<int, int> selected = this->showWaypointMenu();
                if(selected.first == 99 || selected.second == 99) {
                    break;
                }
                icreate::MoveBaseGoalData data = this->navigations_[selected.first].targets[selected.second];
                robot.addTargetQueue(data);
            }
            break;
        }
        default:
            return false;
    }
    this->verifyTarget(robot);
    return true;
}

bool MultiNavigation::setNextStepMode(Robot &robot) {
    //เช็คว่าก่อนหน้านี้ทำอะไรมา
    //navigation_case == 0 อยู่ชั้นเดียวกัน รัน target_queue ต่อไปได้เลย 
    //// รัน nextStepMode ของ single_navigation ได้เลย 
    std::string state;
    switch(robot.getNavigationMode()) {
        //MODE : Go to Specific Point
	    case 0: {
            if(robot.target_queue.size() == 0) {
	    	    this->setupTargetQueue(robot);
            }
            else {
                state = "SINGLERUN";
                robot.sendStateRequest(state);
            }
            break;
        }
		//MODE : Going and Wait for Cargo
		case 1 : { 
            this->setupTargetQueue(robot);
            break;
        }
		//MODE : Delivery to Target Place
		case 2: {
            this->setupTargetQueue(robot);
	  		break;
        }
        //MODE : Going and Come Back to This Place
		case 3: {
            if(this->isFinishQueue) {
                this->setupTargetQueue(robot);
            }
            else {
                state = "GOING";
                robot.addTargetQueue(robot.getStartPosition());
                robot.sendStateRequest(state);
                this->isFinishQueue = true;
            }
            break;
        }
        // //MODE : Going and Come Back to Base Station
	    // case 4: {
        //     if(this->isFinishQueue) {
        //         this->setupTargetQueue(robot);
        //     }
        //     else {
        //         state = "BACKTOBASE"; 
        //         robot.addTargetQueue(this->targets[0]);
        //         robot.sendStateRequest(state);
        //         this->isFinishQueue = true;
        //     }
        //     break;
        // }
        // //MODE : Going to n Places and Send Supplies in 1 Place
	    // case 5: {
        //     if(robot.target_queue.size() == 0) {
        //         if(this->isFinishQueue) {
        //             this->setupTargetQueue(robot);
        //         }
        //         else {
        //             state = "SENDSUPPLIES";
        //             selected_point = this->showWaypointMenu();
        //             if(selected_point == 99) {
        //                 return false;
        //             }
        //             robot.addTargetQueue(this->targets[selected_point]);
        //             this->isFinishQueue = true;
        //         }
        //     }
        //     else {
        //         state = "GOING";
        //     }
        //     robot.sendStateRequest(state);
        //     break;
        // }
        // //MODE : Going 1 Place and Send Supplies in n Places
	    // case 6: {
        //     if(robot.target_queue.size() == 0) {
        //         if(this->isFinishQueue) {
        //             this->setupTargetQueue(robot);
        //         }
        //         else {
        //             while(ros::ok()) {
        //                 std::cout << "Please insert point !!!" << std::endl;
        //                 selected_point = this->showWaypointMenu();
        //                 if(selected_point == 99) {
        //                     break;
        //                 }
        //                 robot.addTargetQueue(this->targets[selected_point]);
        //             }
        //             this->isFinishQueue = true;
        //         }   
        //     }
        //     else {
        //         state = "SENDSUPPLIES";
        //     }
        //     robot.sendStateRequest(state);
        //     break;
        // }
        // //MODE : Going n Places and Send Supplies in n Places
	    // case 7: {
        //     if(robot.target_queue.size() != 0 && !this->isFinishQueue) {
        //         state = "GOING";
        //     }
        //     else if(robot.target_queue.size() != 0 && this->isFinishQueue) {
        //         state = "SENDSUPPLIES";
        //     }
        //     else {
        //         if(this->isFinishQueue) {
        //             this->setupNavigationQueue(robot);
        //         }
        //         else {
        //             while(ros::ok()) {
        //                 std::cout << "Please insert point !!!" << std::endl;
        //                 selected_point = this->showWaypointMenu();
        //                 if(selected_point == 99) {
        //                     break;
        //                 }
        //                 robot.addTargetQueue(this->targets[selected_point]);
        //             }
        //             this->isFinishQueue = true;
        //         }
        //         state = "SENDSUPPLIES";
        //     }
        //     robot.sendStateRequest(state);
        //     break;
        // }
	  	// //MODE : Execute The Memorized Sequence
		// case 10: {
        //     ROS_WARN("Cannot Use This Mode Now");
        //     return false; 
        //     // continue;
        // }
        default:
            ROS_WARN("Wrong Selected Navigation Mode");
	        std::cout << "Please Try Again" << std::endl;
            return false; 
    }
    return true;
}

void MultiNavigation::runLiftNavigation(Robot &robot) {
    //navigation_case == 1 อยู่คนละชั้น 
    //// มี 3 step ย่อย ๑เดินไปที่จุดรอลิฟต์ ๒รอรับค่าลิฟต์ที่มาถึง มาแล้วให้หุ่นไปหยุดที่หน้าลิฟต์ 
    //// ๓เช็คว่าประตูลิฟต์เปิดรึยัง ถ้ายังรอต่อจนลิฟต์เปิด ถ้าเปิดแล้วสั่งรัน service gmapping
    //// แล้วสั่งหุ่นเดินเข้าลิฟต์ ๔หุ่นหมุนตัวในลิฟต์ ๕หุ่นเดินออกจากลิฟต์ 
    ROS_WARN("%d",this->nav_idx);
    ROS_WARN("%d",this->lift_navigation_step);
    switch(this->lift_navigation_step) {
        //Step 0: Move to Lift Ground
        case 0: {
            MoveBaseGoalData data = this->navigations_[this->nav_idx].lifts.back();
            robot.target_queue.insert(robot.target_queue.begin(), data);
            robot.sendStateRequest("USINGLIFT");
            this->navigations_[this->nav_idx].requestToSetNewGoal = true;
            break;
        }
        //Step 1: Wait & Move to in front of the inncoming lift
        case 1: {
            this->lift_number = waitForIncomingLift();
            robot.target_queue.insert(robot.target_queue.begin(),this->navigations_[this->nav_idx].lifts[lift_number]);
            robot.sendStateRequest("USINGLIFT");
            this->navigations_[this->nav_idx].requestToSetNewGoal = true;
            break;
        }
        //Step 2: Wait Lift Door Open & Move inside the lift
        case 2: {
            ROS_ERROR("Come 2");
            bool flag;
            while(ros::ok()) {
                if(!this->verifyLiftDoor()) {
                    lift_number = waitForIncomingLift();
                    robot.target_queue.insert(robot.target_queue.begin(),this->navigations_[this->nav_idx].lifts[lift_number]);
                    robot.sendStateRequest("USINGLIFT");
                    this->navigations_[this->nav_idx].requestToSetNewGoal = true;
                    this->lift_navigation_step--;
                    break;
                }
                ros::ServiceClient client = nh_.serviceClient<icreate_navigation::RunGmappingService>(run_gmapping_service_name_);
                icreate_navigation::RunGmappingService srv;
                srv.request.task = "open";
                if(client.call(srv)) {
                    flag = srv.response.success;
                }
                else {
                    ROS_WARN("Failed to run gmapping");
                    continue;
                }
                break;
            }
            if(flag) {
                initializeSimpleForwardMoveBaseTarget(robot, "Going To Lift");
            }
            break;
        }
        case 3: {
            ROS_ERROR("Come 3");
            move_base_msgs::MoveBaseGoal new_point;
            new_point.target_pose.pose.orientation.z = -1.0;
            new_point.target_pose.pose.orientation.w = 0.0;
            MoveBaseGoalData data("Rotate In Lift", new_point, this->navigations_[this->nav_idx].building, this->navigations_[this->nav_idx].building_floor_lift);
            robot.target_queue.insert(robot.target_queue.begin(),data);
            robot.sendStateRequest("USINGLIFT");
            this->navigations_[this->nav_idx].requestToSetNewGoal = true;
            break;
        }
        case 4: {
            ROS_ERROR("Come 4");
            bool flag = waitUserInputLift();
            if(!flag) {
                //ไม่แน่ใจว่าใช้ได้มั้ย
                return ;
            }
            while(ros::ok()) {
                ros::ServiceClient client = nh_.serviceClient<icreate_navigation::RunGmappingService>(run_gmapping_service_name_);
                icreate_navigation::RunGmappingService srv;
                srv.request.task = "restart";
                if(client.call(srv)) {
                    flag = srv.response.success;
                }
                else {
                    ROS_WARN("Failed to run gmapping");
                    continue;
                }
                break;
            }
            if(flag) {
                mid_range += 0.5;
                move_base_msgs::MoveBaseGoal new_point;
                new_point.target_pose.pose.position.x = mid_range;
                new_point.target_pose.pose.orientation.w = 1;
                MoveBaseGoalData data("Going Out Lift", new_point,this->navigations_[this->nav_idx].building, this->navigations_[this->nav_idx].building_floor_lift);
                robot.target_queue.insert(robot.target_queue.begin(),data);
                this->navigations_[this->nav_idx].requestToSetNewGoal = true;
            }
            break;
        }
        case 5: {
            this->lift_number = 0;
            bool flag = false;
            ros::ServiceClient client = nh_.serviceClient<icreate_navigation::RunGmappingService>(run_gmapping_service_name_);
            // icreate_navigation::RunGmappingService srv;
            // srv.request.task = "close";
            // if(client.call(srv)) {
            //     flag = srv.response.success;
            //     // break;
            // }
            // else {
            //     ROS_WARN("Failed to run gmapping");
            // }
            // if(flag) {
                this->navigation_case = 0;
                client = nh_.serviceClient<miltbot_map::SetMap>("set_map_service");
                miltbot_map::SetMap srv2;
                srv2.request.floor = robot.target_queue[0].building_floor + " Lift";
                srv2.request.target_number = this->lift_number;
                if(client.call(srv2)) {
                    bool flag2 = srv2.response.flag;
                    robot.building = robot.target_queue[0].building;
                    robot.building_floor = robot.target_queue[0].building_floor;
                }
                else {
                   ROS_WARN("Failed to run set map");
                }
            // }
            break;
        }
    }
}

std::pair<int, int> MultiNavigation::showWaypointMenu() {
    while(ros::ok()) {
        //แสดงเรียงลำดับตาม อาคาร ชั้น จุดหมาย
        this->displayBuildingFloor();
        // Ask for ID and wait user input
        std::cout << "[AGENT] Input Target Floor : ";
        int selected_floor;
        std::cin >> selected_floor;
        if((selected_floor < 0 || selected_floor >= this->navigations_.size()) && selected_floor != 99) {
            ROS_WARN("Wrong Select Floor To Navigate Try Again");
            continue;
        }
        else if(selected_floor == 99) {
            return std::make_pair(selected_floor, -1);
        }
        int selected_point = this->navigations_[selected_floor].showWaypointMenu();
        return std::make_pair(selected_floor, selected_point);
    }
}

void MultiNavigation::createTimer(int duration) {
    for(int i = 0; i < this->navigations_.size(); i++) {
        this->navigations_[i].createTimer(duration);
    }
}

int MultiNavigation::waitForIncomingLift() {  
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
        this->navigations_[this->nav_idx].displayLiftWaypoints();
        std::cout << "[AGENT] Wait For Number of Lift : " <<std::endl;
        std::string liftNumber;
        std::cin >> liftNumber;
        std::cout << "[AGENT] Lift Number " + liftNumber + " is on the floor" <<std::endl;
        try {
            int num = atoi(liftNumber.c_str());
            if(num < 0 || num >= this->navigations_[this->nav_idx].lifts.size()) {        
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
    return 0;
}

bool MultiNavigation::verifyLiftDoor() {
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

void MultiNavigation::initializeSimpleForwardMoveBaseTarget(Robot &robot, std::string goal_name) {
    //ต้องรู้จุดที่หุ่นยนต์ปัจจุบัน แล้วสั่งให้เดินไปทีละ 30 ซม. ???
    //รับค่าระยะมาจากกล้องแล้วใส่เป็น input position x
    ros::ServiceClient client = nh_.serviceClient<icreate_lift_navigation::GetMiddleRange>("get_middle_range");
    icreate_lift_navigation::GetMiddleRange srv;
    if(client.call(srv)) {
        this->mid_range = srv.response.mid_range;
        ROS_INFO("Get Mid Range data: %f",mid_range);
        this->mid_range -= 0.5;
        move_base_msgs::MoveBaseGoal new_point;
        new_point.target_pose.pose.position.x = this->mid_range;
        new_point.target_pose.pose.orientation.w = 1;
        MoveBaseGoalData data("Going To Lift", new_point, this->navigations_[this->nav_idx].building, this->navigations_[this->nav_idx].building_floor_lift);
        robot.target_queue.insert(robot.target_queue.begin(),data);
        this->navigations_[this->nav_idx].requestToSetNewGoal = true;
    }
    else {
        ROS_ERROR("Fail to call Service get_depth_distance");
    }
}

bool MultiNavigation::waitUserInputLift() {
    bool flag;
    while(ros::ok()) {
        std::cout << "Waiting for lift stop on the target floor" << std::endl;
        std::cout << "Please press \"y\" to start navigation " << std::endl;
        std::cout << "or Please press \"n\" to stop navigation " << std::endl;
        std::string input;
        std::cin >> input;
        if(input == "y" || input == "Y") {
            flag = true;
            break;
        }
        else if(input == "n" || input == "N") {
            flag = false;
            break;
        }
        else {
            std::cout << "Wrong Input Please Try Again" << std::endl;
        }
    }
    return flag;
}

bool MultiNavigation::comparator(SingleNavigation i,SingleNavigation j) {
    return i.building_floor_num < j.building_floor_num;
}

}