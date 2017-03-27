//Arduino 1.0+ Only
//Arduino 1.0+ Only

/*Based largely on code by  Jim Lindblom

 Get pressure, altitude, and temperature from the BMP085.
 Serial.print it out at 9600 baud to serial monitor.
 */

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
//#include <sensor_msgs/Imu.h>
#include <Wire.h>
#include <BMP085.h>
#include "I2Cdev.h"
#include "MPU6050.h"


BMP085 pressure_m;
MPU6050 accelgyro;

ros::NodeHandle nh;
std_msgs::Float32MultiArray imu_msg;
ros::Publisher imu_pub("arduino/imu", &imu_msg);

void setup(){
  //initialize Serial buadrate
  Serial.begin(9600);
  //initialize SDA & SCL pin
  Wire.begin();
  //Calibrate Pressure Sensor
  pressure_m.bmp085Calibration();

  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  nh.initNode();
  nh.advertise(imu_pub);
}

void loop()
{
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  float temperature = pressure_m.bmp085GetTemperature(); //MUST be called first
  float pressure = pressure_m.bmp085GetPressure();
  float altitude = pressure_m.calcAltitude(pressure);

  Serial.print("Temperature: ");
  Serial.print(temperature, 2); //display 2 decimal places
  Serial.println("deg C");

  Serial.print("Pressure: ");
  Serial.print(pressure, 0); //whole number only.
  Serial.println(" Pa");

  Serial.print("Altitude: ");
  Serial.print(altitude, 2); //display 2 decimal places
  Serial.println(" M");

  Serial.println();//line break

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.print("a/g:\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.println(gz);

  //Publish data using ROSLIB
  float imu_data[6];
  imu_data[0] = gx;
  imu_data[1] = gy;
  imu_data[2] = gz;
  imu_data[3] = ax;
  imu_data[4] = ay;
  imu_data[5] = az;
  imu_msg.data = imu_data;
  imu_pub.publish(&imu_msg);
  nh.spinOnce();

  delay(1000); //wait a second and get values again.
  
}
