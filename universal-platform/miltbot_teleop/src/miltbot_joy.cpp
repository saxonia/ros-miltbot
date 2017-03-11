#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <actionlib_msgs/GoalID.h>

class MiltbotJoy
{
public:
    MiltbotJoy();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;
    ros::Publisher twist_pub_, auto_stop_pub_, stop_pub_;
    ros::Subscriber joy_sub_;

    std::string twist_pub_topic_name_,joy_sub_topic_name_, auto_stop_pub_topic_name_;
    int linear_, angular_, deadman_, cancel_;
    double l_scale_, a_scale_;
    bool trigger_button;
    
};

MiltbotJoy::MiltbotJoy():
    twist_pub_topic_name_("cmd_vel"),
    joy_sub_topic_name_("joy"),
    auto_stop_pub_topic_name_("move_base/cancel"),
    linear_(1),
    angular_(3),
    deadman_(2),
    cancel_(4)
{
    //param ("NAME" , VAR , VAR );
    nh_.param("axis_linear", linear_, linear_);
    nh_.param("axis_angular", angular_, angular_);
    nh_.param("button_deadman_switch", deadman_, deadman_ );
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);
    nh_.param("twist_pub_topic", twist_pub_topic_name_,twist_pub_topic_name_);
    nh_.param("joy_sub_topic", joy_sub_topic_name_,joy_sub_topic_name_);
    std::cout << twist_pub_topic_name_ << std::endl;
    std::cout << l_scale_ << std::endl;
    nh_.param("auto_stop_pub_topic", auto_stop_pub_topic_name_,auto_stop_pub_topic_name_);

    //Publisher and Subscriber
    twist_pub_  = nh_.advertise<geometry_msgs::Twist>(twist_pub_topic_name_, 1);
    joy_sub_    = nh_.subscribe<sensor_msgs::Joy>(joy_sub_topic_name_, 10, &MiltbotJoy::joyCallback, this);

    //Navigation Stopper
    auto_stop_pub_ = nh_.advertise<actionlib_msgs::GoalID>(auto_stop_pub_topic_name_, 1);
    stop_pub_ = nh_.advertise<actionlib_msgs::GoalID>(auto_stop_pub_topic_name_,1);

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

    //Read Value From Right Joystick (HORIZONTAL ACCESS ONLY)
    twist.angular.z = a_scale_*joy->axes[angular_];
    //Read Value From Left Joystick (VERTICAL ACCESS ONLY)
    twist.linear.x = l_scale_*joy->axes[linear_];

    deadman_triggered = joy->axes[deadman_];
    goal_cancel_button = joy->buttons[cancel_];

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
}


int main(int argc,char** argv)
{
    ros::init(argc,argv,"miltbot_teleop_joy_node");
    MiltbotJoy miltbot_joy;
    ros::spin();
    return 0;
}