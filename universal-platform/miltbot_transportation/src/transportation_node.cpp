#include <ros/ros.h>
#include <iostream>

#include "miltbot_transportation/RunTransportation.h"
// #include <icreate_transportation/CargoList.h>
// #include <string>

// #include <algorithm>

std::vector<std::string> cargo_list;
// const int max_cargo_size = 10;
// int gen_id = 0;

// void showMenu() {
//     std::cout << "!!! Transportation System !!!" << std::endl;
//     std::cout << "Please select your command" << std::endl;
//     std::cout << "[0] Send Supplies" << std::endl;
//     std::cout << "[1] Receive Suplies" << std::endl;
//     std::cout << "[99] Cancel Transportation" << std::endl;
// }

// std::string generateCargoId() {
//     int c = cargo_list.size();
//     std::cout << c << std::endl;
//     if(c < 0 || c > 10) {
//         std::cout << "Error With Cargo List" << std::endl;
//         return "";
//     }
//     else if(c == 10) {
//         std::cout << "Cargo List is Full" << std::endl;
//         return "";
//     }
//     gen_id = gen_id%INT_MAX; 
//     std::string id = "00" + std::to_string(gen_id); 
//     gen_id++;
//     return id;
// }

void showCargoList() {
    for(int i = 0; i < cargo_list.size(); i++) {
        std::cout << "["<<i<<"] " << cargo_list[i] << std::endl;
    }
    std::cout << "[99] Cancel" << std::endl;
}

bool runReceiveSupplies() {
    std::cout << "Please insert name of supplies that you want to send" << std::endl;
    std::cout << "Or Press 'n' to Cancel" << std::endl;
    std::cout << "Your Input: ";
    std::string supplies_name;
    std::cin >>  supplies_name;
    if(supplies_name == "n" || supplies_name == "N") {
        std::cout << std::endl;
        return false;
    }
    cargo_list.push_back(supplies_name);
    std::cout << "Finished send : " << supplies_name << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;
    return true;
}

bool runSendSupplies() {
    while(ros::ok()) {
        showCargoList();
        std::cout << "Please select you cargo id to receive" << std::endl;
        std::cout << "Your Input: ";
        int selected_cargo;
        std::string input;
        std::cin >>  input;
        try {
            selected_cargo = atoi(input.c_str());
            if((selected_cargo < 0 || selected_cargo >= cargo_list.size()) && selected_cargo != 99) {
                ROS_WARN("Wrong Input Please Try Again");
                std::cout << std::endl;
                std::cout << std::endl;
                continue;
            }
            else if(selected_cargo == 99) {
                std::cout << std::endl;
                std::cout << std::endl;
                return false;
            }
            else {
                std::string tmp = cargo_list[selected_cargo];
                cargo_list.erase(cargo_list.begin() + selected_cargo);
                std::cout << "Finish receive : " << tmp << std::endl;
                std::cout << std::endl;
                std::cout << std::endl;
                break;
            }
        }
        catch(std::exception &e) {
            std::cout << "Standard exception: " << e.what() << std::endl;
            continue;
        }
    }
    return true;
}

bool runTransportationService(miltbot_transportation::RunTransportation::Request &req,
                       miltbot_transportation::RunTransportation::Response &res) {
    std::string mode = req.mode;
    std::cout << "!!! Transportation System !!!" << std::endl;
    if(mode == "receive") {
        res.success = runReceiveSupplies();
    }
    else if(mode == "send") {
        res.success = runSendSupplies();
    }
    else {
        res.success = false;
    }
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "transportation_node");
    ros::NodeHandle nh;
    std::string transportation_service_name("run_transportation");
    int polling_rate(30);
    // std::string transportation_pub_name("cargo_list");
    nh.param("transportation_node/transportation_service", transportation_service_name, transportation_service_name);
    // nh.param("/icreate_transportation/transportation_pub", transportation_pub_name, transportation_pub_name);

    ros::ServiceServer transportation_service = nh.advertiseService(transportation_service_name, runTransportationService);
    // ros::Publisher transportation_pub = nh.advertise<icreate_transportation::CargoList>(transportation_pub_name, 1000);
    // ros::Rate loop_rate(polling_rate);
    // while(ros::ok()) {
    //     icreate_transportation::CargoList msg;
    //     msg.data = cargo_list;
    //     transportation_pub.publish(msg);
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    ros::spin();
    return 0;
}