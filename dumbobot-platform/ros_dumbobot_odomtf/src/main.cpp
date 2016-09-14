#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#define ENCODER_ROUND_TICKS 3126

ros::Time current_time_encoder, last_time_encoder;

double DistancePerCount = 0.0002; 
double x =0 ;
double y =0 ;
double th =0;
double vx;
double vy;
double vth;
double W,V;
double deltaLeft;
double deltaRight;
double d = 0.4;
long previous_left_ticks,previous_right_ticks;

// New Encoder Var
double left_encoder,right_encoder;
double prev_left_encoder , prev_right_encoder; 

double linear;  //linear velocity
double angular; //angular velocity 
ros::Time current_time, last_time;

void WheelCallback(const geometry_msgs::Vector3::ConstPtr& ticks){
    // Update Current Encoder Position 
    left_encoder = ticks->x;
    right_encoder = ticks->y;

    //std::cout << "Got Left Encoder  = " << left_encoder <<std::endl;
    //std::cout << "Got Right Encoder  = " << right_encoder <<std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/ros_dumbobot/wheel_encoder", 10, WheelCallback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);   //50
  tf::TransformBroadcaster odom_broadcaster;

  //Initialize Timer 
      current_time = ros::Time::now();
      last_time = ros::Time::now();  //prevent null

  // Parameters Here
    //tick Width = 4975 tick per meters
      double ticks_meter = 5050;//4975;//5000;//4975;
      double wheel_radius_multiplier = 1.0;
      double wheel_radius_ = 0.095 * wheel_radius_multiplier;
      double wheel_separation_multiplier = 1.6; //1.7
      double wheel_separation_ = 0.4 * wheel_separation_multiplier;
  // Update Loop (1Hz sec update)
  //ros::Rate r(10);
   // ros::Rate r(10); //Working Param
      ros::Rate r(50); //Experimental
  while(n.ok()){

    // "Now" timestamp
    current_time = ros::Time::now();

    // Calculate Odometry
        /// Time
        double dt       = (current_time - last_time).toSec();

        /// Delta of Encoders
        deltaLeft       = left_encoder - prev_left_encoder;
        deltaRight      = right_encoder - prev_right_encoder;

        /// Convert Ticks To Meter
        double deltaLeft_meter  = deltaLeft/ticks_meter;
        double deltaRight_meter = deltaRight/ticks_meter;

        /// Memories the Encoder Value in this State
        prev_right_encoder  = right_encoder;
        prev_left_encoder   = left_encoder;

        /// Calculate actual distance traveled 
        double actualDistance   = (deltaRight_meter + deltaLeft_meter)/2;
        double theta            = (deltaRight_meter - deltaLeft_meter)/wheel_separation_;

        /// [ODOM] Calculate Velocities
        linear = actualDistance / dt;
        angular = theta / dt;

        //std::cout << "SPEED IS = " << linear << std::endl;
        //std::cout << "ANGULAR IS = " << angular << std::endl;

        if(actualDistance != 0){
            /// [TF+ODOM] Calculate Distance Traveled in (x,y) Format 
            double temp_x =  cos(theta) * actualDistance;
            double temp_y = -sin(theta) * actualDistance;

            /// Calculate Final position
            x += ( cos(th) * temp_x - sin(th) * temp_y );
            y += ( sin(th) * temp_x + cos(th) * temp_y );
        }
        if(theta !=0){
            th += theta;
        }

    // Publish TF of The Static Part
    //publish_tf();

    // Publish TF for the moving Part
    /// since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    /// first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    // ODOM
    /// next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    /// set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    /// set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = linear;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = angular;

    /// publish the message
    odom_pub.publish(odom);  

    last_time = current_time;
    ros::spinOnce();
    r.sleep();   
  }
}
