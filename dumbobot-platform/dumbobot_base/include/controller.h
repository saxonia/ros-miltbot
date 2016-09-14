/*********************************************************************
Dumbobot - Controller
Author : Theppasith Nisitsukcharoen
<theppasith@gmail.com , Theppasith.N@Student.chula.ac.th>
Department of Computer Engineering , Chulalongkorn University
*********************************************************************/

#include <boost/thread/condition_variable.hpp>
#include <boost/lexical_cast.hpp>
#include <stdint.h>
#include <string>
#include <serial/serial.h>
#include <ros/ros.h>
#define BUFFERSIZE 50

//Odom and tf
#include <geometry_msgs/Vector3.h>

namespace dumbo{



class Controller {

private:
  std::string port_name_;
  serial::Serial ser;

  const char *port_;
  int baud_;
  bool connected_;
  bool receiving_script_messages_;
  std::string version_;
  serial::Serial *serial_;

  uint8_t buff[BUFFERSIZE];
  ros::NodeHandle nh_;
  ros::Publisher pub_status_;

  void read();
  void write(std::string);

public:
  Controller (const char *port, int baud);
  ~Controller();

  void connect();
  bool connected() { return connected_; }
  void spinOnce() { read(); }
  void openSerialPort(std::string port);
  void send_forward(int speed);
  void send_stop();
  void send_read_encoder();
  // Send commands to motor driver.
  void driveDirect(int left_speed , int left_dir , int right_speed , int right_dir);
  void driveTutor(int left_speed , int left_dir , int right_speed , int right_dir);
  int read_drive_command();
  int ser_send_avail();
  geometry_msgs::Vector3 read_encoder();
  int readtick();
};

}


