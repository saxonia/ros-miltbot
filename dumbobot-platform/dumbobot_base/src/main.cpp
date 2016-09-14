/*********************************************************************
ros-dumbobot Node
Author : Theppasith Nisitsukcharoen
<theppasith@gmail.com , Theppasith.N@Student.chula.ac.th>
Department of Computer Engineering , Chulalongkorn University
*********************************************************************/

/**
 * Node entry-point locates Here !
 * Handles ROS setup, and serial port connection/reconnection.
**/
#include <string>
#include "controller.h"
#include <iostream>
#include <ros/ros.h>
#include <termios.h>
#include <geometry_msgs/Twist.h>

//Odom and tf
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


 #ifndef MIN
#define MIN(a,b) ((a < b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b) ((a > b) ? (a) : (b))
#endif

double              vl,vr;
int                 left_dir , right_dir , encoder_request_button;
double              linear_x,angular_z;
bool                stop_published =false;
dumbo::Controller   *controller;

ros::Time current_time_encoder, last_time_encoder;
double oneMetre = 4975;
double oneRound = 3125;
//Odom and tf 
geometry_msgs::Vector3 ticks;

//

void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
 /*
    CHANGE SOME GLOBAL VAR WHICH IS USE IN CONTROLLOOP()
 */
  linear_x = msg->linear.x;
  angular_z = msg->angular.z;

  if(linear_x == -0.00)linear_x = 0;
  if(angular_z == -0.00)angular_z = 0;

}

/*
  // Scale Linear and Angular Velocity onto [-1,1]
    double linear_x_scaled = linear_x / maximum_linear ;
    double angular_z_scaled = angular_z / maximum_angular;
     //angular_z_scaled *= 2.8;//1.7; //
    angular_z_scaled *= 1.8;
    //std::cout << "Linear_x  = " <<linear_x_scaled <<std::endl;
    //std::cout << "Angular_Z = " << angular_z_scaled <<std::endl;

  // Compute wheels velocities: [DIFF DRIVE MODEL]
    // const double vel_left  = (linear_x - angular_z * wheel_separation / 2.0);
    // const double vel_right = (linear_x + angular_z * wheel_separation / 2.0);
    double vel_left  = (linear_x_scaled - angular_z_scaled);
    double vel_right = (linear_x_scaled + angular_z_scaled);

*/

// Receive Linear and Angular Velocity and Turn into Motor Effort [-255,255]
void control_loop_cmd_vel_new(){

  // Constant Params
    const double maximum_linear = 0.60;//0.60;
    const double maximum_angular = 1.78;//1.78;
    double width_robot = 0.4; //40 CM from Wheel to Wheel
    double wheelRadius = 0.095; //9.5 CM Wheel Center to Circumference
    double wheel_separation_multiplier = 1.7;
    double wheel_separation = width_robot * wheel_separation_multiplier; //Wheel Separation

  // Scale Linear and Angular Velocity onto [-1,1]
    double linear_x_scaled = linear_x / maximum_linear ;
    double angular_z_scaled = angular_z / maximum_angular;
     //angular_z_scaled *= 2.8;//1.7; //
    angular_z_scaled *= 3.5;
    //std::cout << "Linear_x  = " <<linear_x_scaled <<std::endl;
    //std::cout << "Angular_Z = " << angular_z_scaled <<std::endl;

  // Compute wheels velocities: [DIFF DRIVE MODEL]
      double vel_left  = (linear_x_scaled - angular_z_scaled * wheel_separation / 2.0);
      double vel_right = (linear_x_scaled + angular_z_scaled * wheel_separation / 2.0);
    //double vel_left  = (linear_x_scaled - angular_z_scaled);
    //double vel_right = (linear_x_scaled + angular_z_scaled);


    if( fabs(vel_left) > 1.0 )
    {
      vel_right /= fabs(vel_left);
      vel_left /= fabs(vel_left);
    }

    if( fabs(vel_right) > 1.0 )
    {
      vel_left /= fabs(vel_right);
      vel_right /= fabs(vel_right);
    }

    std::cout << "Velocity[-1,1] : " << vel_left << " , " << vel_right << std::endl;

  // Assign Power to each wheels
    vl = vel_left ; 
    vr = vel_right ;

  // Scaler 
  int command_vl = (vl) * 255;
  int command_vr = (vr) * 255;

  // Direction Bits
  left_dir = (vl < 0)? 2:1;
  right_dir = (vr < 0)? 2:1;

  std::cout << "Velocity  : " << command_vl << " , " << command_vr << std::endl;  

  // Absolute the Command message since the direction base on the direction bit
  // only the magnitude needed
  command_vl = (command_vl < 0 )? -command_vl:command_vl; 
  command_vr = (command_vr < 0 )? -command_vr:command_vr;

  // Send Drive Command
  controller->driveDirect(command_vl,left_dir,command_vr,right_dir);

  // Request to Read Encoder Everytime That Driving Command Sends
  controller->send_read_encoder();

  // Read the incoming Encoder in Vector3 Format (L , R , NULL)
    /*
        3126 ticks Per Revolute
        Dumbobot use Absolute Encoder 
        it counts since the robot turned on.
    */
  ticks = controller->read_encoder();
}


void control_loop_cmd_vel(){

  // Get Value From cmd_vel
  double vel_x = linear_x;
  double vel_th = angular_z;
  // Params
  double width_robot = 0.4; //40 CM from Wheel to Wheel
  double wheelRadius = 0.095; //9.5 CM Wheel Center to Circumference
  // Calculate the transmission power
  double velDiff = (width_robot * vel_th) / 2.0;
  double leftPower = (vel_x - velDiff);
  double rightPower = (vel_x + velDiff);

  // Assign Power to each wheels
  vl = leftPower ; 
  vr = rightPower ;

  // Limitors
  vl = MAX(vl , -0.60);
  vr = MAX(vr , -0.60);
  vl = MIN(vl , 0.60);
  vr = MIN(vr , 0.60);

  //std::cout << "LEFT POWER = " << vl <<std::endl;
  //std::cout << "RIGHT POWER = " << vr <<std::endl;

  // Direction Bits
  left_dir = (vl < 0)? 2:1;
  right_dir = (vr < 0)? 2:1;

  // Scaler to meet Byte (0-255)
  int command_vl = (vl/0.60)*255;
  int command_vr = (vr/0.60)*255;

  // Absolute the Command message since the direction base on the direction bit
  // only the magnitude needed
  command_vl = (command_vl < 0 )? -command_vl:command_vl; 
  command_vr = (command_vr < 0 )? -command_vr:command_vr;

  // Send Drive Command
  controller->driveDirect(command_vl,left_dir,command_vr,right_dir);

  // Request to Read Encoder Everytime That Driving Command Sends
  controller->send_read_encoder();

  // Read the incoming Encoder in Vector3 Format (L , R , NULL)
  /*
      3126 ticks Per Revolute
      Dumbobot use Absolute Encoder 
      it counts since the robot turned on.
  */
  ticks = controller->read_encoder();
}

/*
 *   Main Loop of This Node
 */
 int main(int argc, char **argv){

  // Create Node Called "ros_dumbobot"
    ros::init(argc, argv, "ros_dumbobot");
    ros::NodeHandle nh("~");

  // Serial Setting
    std::string port = "/dev/ttyACM0";
    int32_t baud = 9600;
    nh.param<std::string>("port", port, port);
    nh.param<int32_t>("baud", baud, baud);
    ROS_INFO("DUMBOBOT connecting to port %s", port.c_str());

  // Subscribe to Joystick Command
    ros::Subscriber sub = nh.subscribe("/ros_dumbobot/cmd_vel", 1, cmd_velCallback);

  // Publish the Encoder Data
    ros::Publisher wheel_encoder_pub = nh.advertise<geometry_msgs::Vector3>("/ros_dumbobot/wheel_encoder", 100);

  // Interface to ATMEGA128 and Motor Controller
    controller = new dumbo::Controller(port.c_str(),baud);
    bool initialize = false;
    
  // Spinner which poll for callback
    ros::AsyncSpinner spinner(50);
    spinner.start(); 

  // Start the Main Loop
    while (ros::ok()) {
      ROS_DEBUG("Attempting connection to %s at %i baud.", port.c_str(), baud);
      if(!controller->connected()){
        controller->connect(); 
      }
      if (controller->connected()) {
        // Single time Initializer
           if(!initialize){
              //One Second Booting
                sleep(1);
              controller->spinOnce(); //Clear Everything that is on the buffer( Clear size = 50)
              ROS_INFO("DUMBOBOT Booting Finished");
              initialize = true;
              //test();
           }
              // Main Control Loop
              control_loop_cmd_vel_new();
              // Publish Encoder to system
              wheel_encoder_pub.publish(ticks);
              // Sleep Between Loops
              usleep(10*1000);
      } else {
        ROS_DEBUG("Problem connecting to serial device.");
        ROS_ERROR_STREAM_ONCE("Problem connecting to port " << port << ". Trying again every 1 second.");
        sleep(1);
      }
    }
    //End Loop
    spinner.stop();
    controller->send_stop();
    return 0;

}
