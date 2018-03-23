#include <ros/ros.h>
// #include "miltbot_navigation/RunMoveBase.h"

#include "miltbot_navigation/GetMiddleRange.h"
#include "miltbot_navigation/RunGmappingService.h"

ros::ServiceClient get_middle_range_client_;
ros::ServiceClient run_gmapping_client_;

int lift_navigation_step;
float mid_range;
bool isDoneGoal;

bool verifyFrontDoor() {
    return true;
    miltbot_vision::IsFrontLift srv;
    if(this->is_front_lift_client_.call(srv)) {
        return srv.response.is_front_lift;
    }
    else {
        ROS_ERROR("Fail to call Service is_front_lift");
        return false;
    }
}

bool verifyLiftDoor() {
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

int main(int argc, char** argv) {
    ros::init(argc, argv, "lift_node");
    ros::NodeHandle nh;

    std::string move_base_topic_name("move_base");
    std::string base_frame_id("map");
    std::string robot_frame_id("base_footprint");
    std::string package_name("icreate_navigation");
    std::string building_name("Building 4");
    std::string building_floor_name("Floor 20");
	int polling_rate(30);
    int timer_duration(10);
    nh.param("navigation_node/move_base_topic", move_base_topic_name, move_base_topic_name);
    nh.param("navigation_node/base_frame_id", base_frame_id, base_frame_id);
    nh.param("navigation_node/robot_frame_id", robot_frame_id, robot_frame_id);
    nh.param("navigation_node/package_name", package_name, package_name); 
	nh.param("navigation_node/polling_rate", polling_rate, polling_rate);
    nh.param("navigation_node/timer_duration", timer_duration, timer_duration);
    nh.param("navigation_node/building", building_name, building_name);
    nh.param("navigation_node/building_floor", building_floor_name, building_floor_name);
    
    std::string run_gmapping_service_name_("run_gmapping");
    std::string get_middle_range_service_name_("get_middle_range");
    this->run_gmapping_client_ = nh_.serviceClient<miltbot_navigation::RunGmappingService>(run_gmapping_service_name_);
    this->get_middle_range_client_ = nh_.serviceClient<miltbot_navigation::GetMiddleRange>(get_middle_range_service_name_);

    lift_navigation_step = 2;
    while(ros::ok()) {
    //     ros::spinOnce();
        r.sleep();
        switch(lift_navigation_step) {
            case 2: {
                if(verifyFrontDoor()) {
                    ROS_INFO("Verify Front Lift Door OK");
                    while(ros::ok()) {
                        miltbot_navigation::GetMiddleRange srv;
                        if(get_middle_range_client_.call(srv)) {
                            mid_range = srv.response.mid_range;
                        }
                        else {
                            ROS_ERROR("Fail to call Service get_depth_distance");
                        }
                        if(mid_range < 100.0) {
                            ROS_INFO("Range OK");
                            break;
                        }
                    }
                    lift_navigation_step++;
                }
                else {
                    ROS_INFO("Verify Front Lift Door Fail");
                    lift_navigation_step = 0;
                }
                ROS_WARN("Before Error");
                isDoneGoal = true;
                break;
            }
            case 3: {
                ROS_INFO("Lift Navigation: Step 3");
                ROS_INFO("isDonegoal: %d",isDoneGoal);
                bool flag = false;
                int verify_door_fail = 0;
                while(ros::ok()) {
                    if(verify_door_fail == 10000) {
                        ROS_WARN("Verify Lift Door Fail");
                        lift_navigation_step = 1;
                        isDoneGoal = true;
                        break;
                    }
                    if(!verifyLiftDoor()) {
                        verify_door_fail++;
                        continue;
                    }
                    ROS_INFO("Verify Door Open OK");
                    miltbot_navigation::RunGmappingService srv;
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
                    ROS_INFO("Finish run Gmapping");
                    // this->isDoneGoal = true;
                    // this->lift_navigation_step++;
                    // break;
                    this->initializeLiftForwardMoveBase();
                }
                break;
            }
            case 4: {
                ROS_INFO("Lift Navigation: Step 4");
                ROS_INFO("isDonegoal: %d",this->isDoneGoal);
                // this->waitUserInputLift();
                // this->isDoneGoal = true;
                // this->lift_navigation_step++;
                // break;
                this->initializeLiftRotateMoveBase();
                break;
            }
            case 5: {
                ROS_INFO("Lift Navigation: Step 5");
                ROS_INFO("isDonegoal: %d",this->isDoneGoal);
                while(ros::ok()) {
                    miltbot_navigation::GetMiddleRange srv;
                    if(get_middle_range_client_.call(srv)) {
                        this->mid_range = srv.response.mid_range;
                    }
                    else {
                        ROS_ERROR("Fail to call Service get_depth_distance");
                    }
                    if(this->mid_range < 100.0) {
                        ROS_INFO("Range OK");
                        break;
                    }
                }
                // this->waitUserInputLift();
                bool flag = false;
                int verify_door_fail = 0;
                while(ros::ok()) {
                    // if(verify_door_fail == 10000) {
                    //     ROS_WARN("Verify Lift Door Fail");
                    //     this->lift_navigation_step = 1;
                    //     this->isDoneGoal = true;
                    //     break;
                    // }
                    if(!this->verifyLiftDoor()) {
                        verify_door_fail++;
                        continue;
                    }
                    ROS_INFO("Verify Door Open OK");
                    miltbot_navigation::RunGmappingService srv;
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
                ROS_INFO("isDonegoal: %d",this->isDoneGoal);
                // this->waitUserInputLift();
                bool flag = false;
                miltbot_navigation::RunGmappingService srv;
                srv.request.task = "close";
                if(run_gmapping_client_.call(srv)) {
                    flag = srv.response.success;
                }
                else {
                    ROS_WARN("Failed to run gmapping");
                }
                if(flag) {
                    this->initializeLiftRotateMoveBase();
                }
                break;
            }
        }  
        
    }
    

}