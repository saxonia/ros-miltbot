#include <ros/ros.h>
#include <ros/package.h>

// #include <vector>
#include <fstream>

#include "miltbot_system/RobotList.h"
#include "miltbot_system/GetRobotList.h"



std::vector<std::string> robot_name_list;

int toint(std::string s) {
    return atoi(s.c_str());
}

void loadRobotFile(std::string filename) {
    std::string path = ros::package::getPath("miltbot_system") + "/robot/" + filename;
    std::cout << "Show path: " << path << std::endl;
    std::ifstream inFile(path.c_str());
    std::string line; 

    std::cout << "Importing robot file" << std::endl;
    //First Value is Robot Namespace 
    getline(inFile, line);
    std::string robot_name = line;
    std::cout << "Robot Name : " << robot_name <<std::endl;
    robot_name_list.push_back(robot_name);
}

std::vector<std::string> loadRobotFileList(std::string filename) {
    std::string path = ros::package::getPath("miltbot_system") + "/robot/" + filename;
    std::ifstream inFile(path.c_str());
    std::string line; 
    std::vector<std::string> robot_list;

    std::cout << "Importing robot file list" << std::endl;

    getline(inFile, line);
    int robot_list_count = toint(line);
    std::cout << "Robot file Counted : " << robot_list_count <<std::endl;

    while(getline(inFile, line)) {
        robot_list.push_back(line);
    }

    //Count All Parameters
    size_t counter = robot_list.size();
    std::cout << "Counter : " << counter << std::endl;
    
    return robot_list;
} 

void loadRobot(std::string robot_list_file_name) {
    std::vector<std::string> robot_list = loadRobotFileList(robot_list_file_name);
    for(int i = 0; i < robot_list.size(); i++) {
        std::cout << "Show : " << robot_list[i] << std::endl;
        // [FUTURE] do delete string quote before  send to load_robot_file
        loadRobotFile(robot_list[i]);   
    }
} 

bool getRobotListService(miltbot_system::GetRobotList::Request &req, 
                            miltbot_system::GetRobotList::Response &res) {
    res.robot_list = robot_name_list;
    return true;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_server");
    // ros::NodeHandle nh("~");
    ros::NodeHandle nh;

    std::string get_robot_list_service_name("get_robot_list");
    std::string robot_list_pub_topic_name("robot_list");
    std::string robot_list_file_name("robot_list.csv");

    // nh.param("get_robot_list_service",get_robot_list_service_name, get_robot_list_service_name);
    // nh.param("robot_list_file",robot_list_file_name, robot_list_file_name);

    loadRobot(robot_list_file_name);
    ros::Publisher pub = nh.advertise<miltbot_system::RobotList>(robot_list_pub_topic_name, 1);
    ros::ServiceServer service = nh.advertiseService(get_robot_list_service_name, getRobotListService);
    ros::Rate rate(30);
    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
        miltbot_system::RobotList robot_list;
        robot_list.robot_list = robot_name_list;
        pub.publish(robot_list);
    }

    return 0;
}