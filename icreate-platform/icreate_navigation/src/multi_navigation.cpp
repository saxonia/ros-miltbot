#include "icreate_navigation/multi_navigation.h"

namespace icreate {

MultiNavigation::MultiNavigation(void): 
    run_gmapping_service_name_("run_gmapping"),
    set_map_service_name_("set_map")
{
    nh_.param("run_gmapping_service", run_gmapping_service_name_, run_gmapping_service_name_);
    nh_.param("set_map_service", set_map_service_name_, set_map_service_name_);
    this->requestToSetNewGoal = false;
    this->nav_idx = -1;
    this->lift_navigation_step = -1;
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
    navigation.sendWaypointRequest(building, building_floor + " Lift");
    this->navigations_.push_back(navigation);
    std::sort(this->navigations_.begin(),this->navigations_.end(),this->comparator);
    
    return true;
}

bool MultiNavigation::removeSingleNavigation(std::string building, std::string building_floor) {
    for(int i = 0; i < this->navigations_.size(); i++) {
        std::string building_ = this->navigations_[i].building;
        std::string building_floor_ = this->navigations_[i].building_floor;
        if(building_ == building && building_floor_ == building_floor) {
            this->navigations_.erase(this->navigations_.begin() + i);
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
}

void MultiNavigation::setupRobotToRun(Robot &robot, std::string base_frame_id, std::string robot_frame_id) {
    robot.setCurrentPosition(base_frame_id, robot_frame_id, robot.building, robot.building_floor);
    this->navigations_[this->nav_idx].setupRobotToRun(robot, base_frame_id, robot_frame_id);
}

bool MultiNavigation::verifyTarget(Robot &robot) {
    for(int i = 0; i < this->navigations_.size(); i++) {
        ROS_INFO("%s %s",this->navigations_[i].building.c_str(), robot.building.c_str());
        ROS_INFO("%s %s",this->navigations_[i].building_floor.c_str(), robot.building_floor.c_str());
        if(this->navigations_[i].building == robot.building 
            && this->navigations_[i].building_floor == robot.building_floor) {
            this->nav_idx = i;
            break;
        }
    }
    this->navigation_case = -1;
    bool flag1 = verifyTargetBuilding(robot.currentPosition, robot.target_queue[0]);
    ROS_INFO("%s %s",robot.currentPosition.building.c_str(), robot.target_queue[0].building.c_str());
    ROS_INFO("%s %s",robot.currentPosition.building_floor.c_str(), robot.target_queue[0].building_floor.c_str());
    bool flag2 = verifyTargetFloor(robot.currentPosition, robot.target_queue[0]);
    if(flag1 && flag2) {
        this->navigation_case = 0;
        ROS_INFO("Verify Target : case 0");
    }
    else if(flag1 && !flag2) {
        ROS_INFO("Verify Target : case 1");
        this->navigation_case = 1;
        this->lift_navigation_step = 0;
        runLiftNavigation(robot);
        ROS_INFO("SAx2");
        
    }
    else {
        ROS_INFO("Verify Target : case 2");
        this->navigation_case = 2;
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
    this->requestToSetNewGoal = false;
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
    return true;
}

bool MultiNavigation::setNextStepMode(Robot &robot) {
    //เช็คว่าก่อนหน้านี้ทำอะไรมา
    //navigation_case == 0 อยู่ชั้นเดียวกัน รัน target_queue ต่อไปได้เลย 
    //// รัน nextStepMode ของ single_navigation ได้เลย 
    //navigation_case == 1 อยู่คนละชั้น 
    //// มี 3 step ย่อย ๑เดินไปที่จุดรอลิฟต์ ๒รอรับค่าลิฟต์ที่มาถึง มาแล้วให้หุ่นไปหยุดที่หน้าลิฟต์ 
    //// ๓เช็คว่าประตูลิฟต์เปิดรึยัง ถ้ายังรอต่อจนลิฟต์เปิด ถ้าเปิดแล้วสั่งรัน service gmapping
    //// แล้วสั่งหุ่นเดินเข้าลิฟต์ ๔หุ่นหมุนตัวในลิฟต์ ๕หุ่นเดินออกจากลิฟต์ 

    switch(this->navigation_case) {
        //CASE : On Same Building Same Floor
        case 0: {
            this->navigations_[this->nav_idx].setNextStepMode(robot);
            break;
        }
        //CASE : On Same Building Different Floor
        case 1: {
            this->runLiftNavigation(robot);
            break;
        }
        //CASE : On Different Building
        case 2: {
            break;
        }
    }
    return true;
}

void MultiNavigation::runLiftNavigation(Robot &robot) {
    switch(this->lift_navigation_step) {
        case 0: {
            ROS_INFO("SAx3 %d %ld", this->nav_idx, this->navigations_[this->nav_idx].lifts.size() );
            MoveBaseGoalData data = this->navigations_[this->nav_idx].lifts.back();
            robot.target_queue.insert(robot.target_queue.begin(), data);
            break;
        }
        case 1: {
            int liftNumber = waitForIncomingLift();
            robot.target_queue.insert(robot.target_queue.begin(),this->navigations_[this->nav_idx].lifts[liftNumber]);
            break;
        }
        case 2: {
            bool flag;
            while(ros::ok()) {
                if(!verifyLiftDoor()) {
                    continue;
                }
                // ros::ServiceClient client = nh_.serviceClient<miltbot_map::SetMapServer>(set_map_service_name_);
                // miltbot_map::SetMapServer set_map_srv;
                // set_map_srv.request.floor = "Lift";
                // if(client.call(srv)) {
                //     // flag = srv.response.flag;
                //     // break;
                // }
                // else {
                //     ROS_WARN("Failed to run gmapping");
                // }
                ros::ServiceClient client = nh_.serviceClient<icreate_navigation::RunGmappingService>(run_gmapping_service_name_);
                icreate_navigation::RunGmappingService srv;
                srv.request.task = "open";
                if(client.call(srv)) {
                    flag = srv.response.success;
                    break;
                }
                else {
                    ROS_WARN("Failed to run gmapping");
                }
            }
            if(flag) {
                float x_position = 2.3;
                move_base_msgs::MoveBaseGoal new_point;
                new_point.target_pose.pose.position.x = x_position;
                new_point.target_pose.pose.orientation.w = 1;
                MoveBaseGoalData data("Going To Lift", new_point,"Building 4", "Floor 20 Lift");
                robot.target_queue.insert(robot.target_queue.begin(),data);
            }
            break;
        }
        case 3: {
            move_base_msgs::MoveBaseGoal new_point;
            new_point.target_pose.pose.orientation.z = -1.0;
            new_point.target_pose.pose.orientation.w = 0.0;
            MoveBaseGoalData data("Rotate In Lift", new_point,"Building 4", "Floor 20 Lift");
            robot.target_queue.insert(robot.target_queue.begin(),data);
            break;
        }
        case 4: {
            std::cout << "Wait For Get Out Of Lift : " ;
            std::string in;
            std::cin >> in;
            float x_position = 2.3;
            move_base_msgs::MoveBaseGoal new_point;
            new_point.target_pose.pose.position.x = x_position;
            new_point.target_pose.pose.orientation.w = 1;
            MoveBaseGoalData data("Going Out Lift", new_point,"Building 4", "Floor 20 Lift");
            robot.target_queue.insert(robot.target_queue.begin(),data);
        }
        case 5: {
            bool flag;
            ros::ServiceClient client = nh_.serviceClient<icreate_navigation::RunGmappingService>(run_gmapping_service_name_);
            icreate_navigation::RunGmappingService srv;
            srv.request.task = "close";
            if(client.call(srv)) {
                flag = srv.response.success;
                break;
            }
            else {
                ROS_WARN("Failed to run gmapping");
            }
            this->navigation_case = 0;
            break;
        }

    }
    this->lift_navigation_step++;
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
        std::cout << "[AGENT] Wait For Number of Lift" <<std::endl;
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
    std::string in;
    std::cin >> in;
    return true;
}

void initializeSimpleForwardMoveBaseTarget(std::string goal_name) {
    //ต้องรู้จุดที่หุ่นยนต์ปัจจุบัน แล้วสั่งให้เดินไปทีละ 30 ซม. ???
    //รับค่าระยะมาจากกล้องแล้วใส่เป็น input position x
    float x_position = 0;
    // ros::ServiceClient client = nh.serverClient<std_msgs::Int16>("/get_depth_distance");
    // std_msgs::Int16 srv;
    // if(client.call(srv)) {
        // x_position = srv.response.distance;
    // }
    // else {
        // ROS_ERROR("Fail to call Service get_depth_distance");
    // }
    x_position = 2.3;
    move_base_msgs::MoveBaseGoal new_point;
    new_point.target_pose.pose.position.x = x_position;
    new_point.target_pose.pose.orientation.w = 1;
    // forward_goal.setGoal(new_point);
    // forward_goal.setGoalName(goal_name);
}

bool MultiNavigation::comparator(SingleNavigation i,SingleNavigation j) {
    return i.building_floor_num < j.building_floor_num;
}

}