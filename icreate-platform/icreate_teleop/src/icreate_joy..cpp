#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class IcreateJoy
{
public:
    IcreateJoy();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::nodeHandle nh_;
    ros::Publisher twist_pub, auto_stop_pub_;
    ros::Subscriber joy_sub_;

    std::string twist_pub_topic_name_, auto_stop_pub_topic_name_, joy_sub_topic_name_;
    int linear_, angular_, deadman_;
    double l_scale_, a_scale_;
    bool trigger_button;
};

IcreateJoy::IcreateJoy():
    twist_pub_topic_name_("/cmd_vel"),
    joy_sub_topic_name_("joy"),
    auto_stop_pub_topic_name_("/move_base/cancel"),
    linear_(1),
    angular_(3),
    deadman_(2)
{
    //param ("NAME" , VAR , VAR );
    nh_.param("axis_linear", linear_, linear_);
    nh_.param("axis_angular", angular_, angular_);
    nh_.param("button_deadman_switch", deadman_, deadman_ );
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);
    nh_.param("twist_pub_topic", twist_pub_topic_name_);
    nh_.param("joy_sub_topic", joy_sub_topic_name_);
    nh_.param("auto_stop_pub_topic", auto_stop_pub_topic_name_);
}



int main(int argc,char** argv)
{
    ros:init(argc,argv,"icreate_teleop_joy_node");

    return 0;
}