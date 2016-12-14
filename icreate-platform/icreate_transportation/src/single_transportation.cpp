#include <ros/ros.h>

#include <icreate_transportation/RunTransportation.h>
#include <icreate_transportation/CargoList.h>
#include <string>
#include <iostream>
#include <algorithm>

std::vector<std::string> cargo_list;
const int max_cargo_size = 10;
int gen_id = 0;

void showMenu() {
    std::cout << "!!! Transportation System !!!" << std::endl;
    std::cout << "Please select your command" << std::endl;
    std::cout << "[0] Send Supplies" << std::endl;
    std::cout << "[1] Receive Suplies" << std::endl;
    std::cout << "[99] Cancel Transportation" << std::endl;
}

std::string generateCargoId() {
    int c = cargo_list.size();
    std::cout << c << std::endl;
    if(c < 0 || c > 10) {
        std::cout << "Error With Cargo List" << std::endl;
        return "";
    }
    else if(c == 10) {
        std::cout << "Cargo List is Full" << std::endl;
        return "";
    }
    gen_id = gen_id%INT_MAX; 
    std::string id = "00" + std::to_string(gen_id); 
    gen_id++;
    return id;
}

bool addCargo() {
    std::cout << "Please put your cargo to the robot" << std::endl;
    std::cout << "Please insert \"y\" to confirm cargo" << std::endl;
    std::cout << "Your input : " ;
    std::string input;
    std::cin >> input;
    if(input != "y"){
        std::cout << "Wrong Input type" << std::endl;
        return false;
    } 
    std::string ret = generateCargoId();
    if(ret != "") {
        std::cout << "Your cargo id: " << ret << std::endl;
        cargo_list.push_back(ret);
        std::sort(cargo_list.begin(), cargo_list.end());
        return true;
    }
    else {
        return false;
    }
}

bool removeCargo() {
    std::cout << "Please insert id of cargo to remove" << std::endl;
    std::cout << "Your id: ";
    int input;
    std::cin >> input;
    if(input < 0 || input >= cargo_list.size()) {
        std::cout << "Can't find your id in cargo list" << std::endl;
        return false;
    }
    std::string tmp = cargo_list[input];
    cargo_list.erase(cargo_list.begin()+input);
    std::cout << "Finish remove : " << tmp << std::endl;
    return true;
    
}

void showCargo() {
    for(int i = 0; i < cargo_list.size(); i++) {
        std::cout <<"["<<i<<"] " << cargo_list[i] << std::endl;
    }
}

bool runTransportation(icreate_transportation::RunTransportation::Request &req,
                       icreate_transportation::RunTransportation::Response &res) {
    while(ros::ok()) {
        showMenu();
        std::cout << "Your Command : " ;
        std::string input;
        bool flag = false;
        std::cin >> input;
        try {
            std::cout << input.c_str() << std::endl;
            switch(std::atoi(input.c_str())) {
                case 0:
                    res.mode = "sending";
                    if(!addCargo()) {
                        continue;
                    }
                    flag = true;
                    break;
                case 1:
                    res.mode = "receiving";
                    showCargo();
                    if(!removeCargo()) {
                        continue;
                    }
                    flag = true;
                    break;
                case 99:
                    res.mode = "cancel";
                    flag = true;
                    break;
                default:
                    std::cout << std::endl;
                    std::cout << "Wrong input please select again" << std::endl;
                    std::cout << std::endl;
            }
            if(flag) {
                std::cout << "Finish return mode" << std::endl;
                return true;
            }  
        }
        catch(std::exception &e) {
            std::cout << "Standard exception: " << e.what() << std::endl;
            continue;
        }
    }
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "single_transportation");
    ros::NodeHandle nh;
    ros::ServiceServer transportation_service = nh.advertiseService("run_transportation", runTransportation);
    ros::Publisher transportation_pub = nh.advertise<icreate_transportation::CargoList>("cargo_list", 1000);
    ros::Rate loop_rate(10);
    while(ros::ok()) {
        icreate_transportation::CargoList msg;
        msg.data = cargo_list;
        transportation_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();

    }
    // ros::Subscriber transportation_req_sub = nh.subcribe("transportation_req", 1000, transportCallback); 
    ros::spin();
    return 0;
}