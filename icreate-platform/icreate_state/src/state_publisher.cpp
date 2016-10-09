#include <ros/ros.h>
#include <std_msgs/String.h>
// #include <string>

class robotState {
    public:
        enum state_list {
            IDLE = 0,
            WAITING = 1,
            GOING = 2,
            BACKTOBASE = 3,
            SENDSUPPLIES = 4
        };
        int state;

        std::string convertToStateName(int state)
        {
            std::string stateName;
            switch(state)
            {
                case 0:
                    stateName = "IDLE";
                    break;
                case 1:
                    stateName = "WAITING";
                    break;
                case 2:
                    stateName = "GOING";
                    break;
                case 3:
                    stateName = "BACKTOBASE";
                    break;
                case 4:
                    stateName = "SENDSUPPLIES";
                    break;
            }
            return stateName;
        }
};

int main(int argc, char** argv) {
    
    //Initialize Node
    ros::init(argc, argv, "icreate_state");

    ros::NodeHandle nh;

    //Initialize Publisher
    ros::Publisher status_pub = nh.advertise<std_msgs::String>("state",10);

    ros::Rate loop_rate(10);

    robotState robotState;

    robotState.state = robotState::IDLE;

    while(ros::ok()) {
        std_msgs::String msg;

        if(robotState.state == robotState::IDLE) {
            robotState.state = robotState::GOING;
        }
        else if(robotState.state == robotState::GOING) {
            robotState.state = robotState::SENDSUPPLIES;
        }
        else if(robotState.state == robotState::SENDSUPPLIES) {
            robotState.state = robotState::BACKTOBASE;
        }
        else if(robotState.state == robotState::BACKTOBASE) {
            robotState.state = robotState::IDLE;
        }


        msg.data = robotState.convertToStateName(robotState.state);
        ROS_INFO("robot state: %s", msg.data.c_str());
        
        status_pub.publish(msg);
        ros::spinOnce();

        loop_rate.sleep();
    }
    
    return 0;
}