#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <actionlib_msgs/GoalID.h>

#include "miltbot_system/RunSystem.h"

class MiltbotJoy
{
public:
    MiltbotJoy();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;
    ros::Publisher twist_pub_, auto_stop_pub_, stop_pub_;
    ros::Subscriber joy_sub_;
    ros::ServiceClient run_system_service_;

    std::string twist_pub_topic_name_,joy_sub_topic_name_, auto_stop_pub_topic_name_;
    std::string run_system_service_name_;
    int linear_, angular_, deadman_, cancel_, start_, stop_;
    double l_scale_, a_scale_;
    bool trigger_button;
    
};

MiltbotJoy::MiltbotJoy():
    twist_pub_topic_name_("cmd_vel"),
    joy_sub_topic_name_("joy"),
    auto_stop_pub_topic_name_("move_base/cancel"),
    run_system_service_name_("run_system"),
    linear_(1),
    angular_(3),
    deadman_(2),
    cancel_(4),
    start_(2),
    stop_(1)
{
    //param ("NAME" , VAR , VAR );
    nh_.param("axis_linear", linear_, linear_);
    nh_.param("axis_angular", angular_, angular_);
    nh_.param("button_deadman_switch", deadman_, deadman_ );
    nh_.param("button_start", start_, start_ );
    nh_.param("button_stop", stop_, stop_ );
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);
    nh_.param("twist_pub_topic", twist_pub_topic_name_,twist_pub_topic_name_);
    nh_.param("joy_sub_topic", joy_sub_topic_name_,joy_sub_topic_name_);
    nh_.param("auto_stop_pub_topic", auto_stop_pub_topic_name_,auto_stop_pub_topic_name_);
    nh_.param("run_system_service", run_system_service_name_,run_system_service_name_);

    //Publisher and Subscriber
    twist_pub_  = nh_.advertise<geometry_msgs::Twist>(twist_pub_topic_name_, 1);
    joy_sub_    = nh_.subscribe<sensor_msgs::Joy>(joy_sub_topic_name_, 10, &MiltbotJoy::joyCallback, this);

    //Navigation Stopper
    auto_stop_pub_ = nh_.advertise<actionlib_msgs::GoalID>(auto_stop_pub_topic_name_, 1);
    stop_pub_ = nh_.advertise<actionlib_msgs::GoalID>(auto_stop_pub_topic_name_,1);

    run_system_service_ = nh_.serviceClient<miltbot_system::RunSystem>(run_system_service_name_);

    trigger_button = false;
}

void MiltbotJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    //Goal Nav Cancle Button
    int goal_cancel_button;
    //Geometry Joystick Control
    geometry_msgs::Twist twist;
    //Button
    int deadman_triggered;
    int start_system, stop_system;

    //Read Value From Right Joystick (HORIZONTAL ACCESS ONLY)
    twist.angular.z = a_scale_*joy->axes[angular_];
    //Read Value From Left Joystick (VERTICAL ACCESS ONLY)
    twist.linear.x = l_scale_*joy->axes[linear_];

    deadman_triggered = joy->axes[deadman_];
    goal_cancel_button = joy->buttons[cancel_];

    start_system = joy->buttons[start_];
    stop_system = joy->buttons[stop_];

    if(goal_cancel_button == 1) {
        stop_pub_.publish(*new actionlib_msgs::GoalID());
    }
    else if (deadman_triggered == -1)
    {
        twist_pub_.publish(twist);
        trigger_button = false;
    }
    else if(deadman_triggered != -1 && !trigger_button)
    {
        //Publish 0,0,0 (stop)
        twist_pub_.publish(*new geometry_msgs::Twist());
        //Publish Goal Cancel Message
        auto_stop_pub_.publish(*new actionlib_msgs::GoalID());
        trigger_button = true;
    }
    else if(start_system == 1) {
        miltbot_system::RunSystem srv;
        srv.request.status = true;
        if(run_system_service_.call(srv)) {
            bool flag = srv.response.success;
        }
        else {
            ROS_ERROR("Failed to call service run_system");
        }
    }
    else if(stop_system == 1) {
        miltbot_system::RunSystem srv;
        srv.request.status = false;
        if(run_system_service_.call(srv)) {
            bool flag = srv.response.success;
            //Publish 0,0,0 (stop)
            twist_pub_.publish(*new geometry_msgs::Twist());
            //Publish Goal Cancel Message
            auto_stop_pub_.publish(*new actionlib_msgs::GoalID());
        }
        else {
            ROS_ERROR("Failed to call service run_system");
        }
    }
}


int main(int argc,char** argv)
{
    ros::init(argc,argv,"miltbot_teleop_joy_node");
    MiltbotJoy miltbot_joy;
    ros::spin();
    return 0;
}