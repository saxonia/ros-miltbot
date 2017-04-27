#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>

#include "sensor_msgs/CompressedImage.h"
#include "std_msgs/String.h"

#include "miltbot_vision/IsFrontLift.h"

#include "mono_camera.h"

using namespace std;

// #define IMAGE
// #define VIDEO
#define KINECT_STREAM
//#define WEBCAM_STREAM

int H_MIN = 42;
int H_MAX = 71;
int S_MIN = 60;
int S_MAX = 255;
int V_MIN = 26;
int V_MAX = 255;

cv::Mat deleteNoise(cv::Mat src, cv::Mat inRange) {
	cv::Mat imgBitwise;
	cv::bitwise_and(src,src,imgBitwise,inRange);
	cv::Mat imgMorph;

	cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
	//dilate with larger element so make sure object is nicely visible
	cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(8,8));

	cv::erode(imgBitwise,imgMorph,erodeElement);
	cv::erode(imgMorph,imgMorph,erodeElement);

	cv::dilate(imgMorph,imgMorph,dilateElement);
	cv::dilate(imgMorph,imgMorph,dilateElement);
    // return imgBitwise;
	return imgMorph;
}

bool detectColorTape(cv::Mat src) {
    cv::Mat hsv_image;
	cv::Mat hsv_inRange;
    cv::Mat color_img;
    cv::medianBlur(src, color_img, 11);
	cv::cvtColor(color_img, hsv_image, CV_BGR2HSV);
    //Green Light Color
	cv::inRange(hsv_image, cv::Scalar(H_MIN,S_MIN,V_MIN), cv::Scalar(H_MAX,S_MAX,V_MAX), hsv_inRange);
	cv::Mat greenLightImage = deleteNoise(color_img,hsv_inRange);
	// cv::Mat greenLightRes = locatePosition(colorImg,greenLightImage,"LIGHT GREEN");
    // this->color_tape_view = hsv_inRange.clone();
    // this->color_tape_proc = greenLightImage.clone();
    cv::imshow("1",hsv_inRange);
    cv::imshow("2",greenLightImage);
    // return this->verifyLiftDoor();
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "door_detection_node");
    ros::NodeHandle nh;
    ROS_INFO("START");

    std::string is_front_lift_service_name("is_front_lift");

    #ifdef IMAGE
        ROS_INFO("IMAGE MODE");
        std::string package_path = ros::package::getPath("miltbot_vision");
        // cv::String image_path(package_path + "/images/door.jpg"); // by default
        cv::String image_path("/home/saxonia/Pictures/door.png");
        if( argc > 1)
        {
            image_path = argv[1];
        }
        cv::Mat src;
        src = cv::imread(image_path, cv::IMREAD_COLOR);
        if(!src.data) {
            ROS_ERROR("errorr");
            return -1;
        }
        cv::resize(src,src,cv::Size(640,480));
        detectColorTape(src);
        
        cv::namedWindow("src");
        cv::imshow("src",src);
        cv::waitKey(0);
        // if(key == 27) 
    #endif
    #ifdef KINECT_STREAM
        ROS_INFO("KINECT STREAM MODE");
        miltbot::MonoCamera cam;
        
        ros::Rate rate(50);
        ROS_INFO("Before loop");
        // cv::namedWindow("color_img");
        // cv::namedWindow("color_tape");
        // cv::namedWindow("color_tape_proc");
        // cv::moveWindow("color_img", 100, 100);
        // cv::moveWindow("color_tape", 100, 100);
        // cv::moveWindow("color_tape_proc", 100, 100);
        while(ros::ok())
        {
            // cam.createTrackbars();
            ros::spinOnce();
            rate.sleep();
            if(!cam.color_view.data || !cam.depth_view.data) {
                continue;
            }
            // cam.detectColorTape(cam.color_view);
            // cv::namedWindow("depth_img");
            // cam.resizeImage(cam.color_view, 240, 240);
            // cam.resizeImage(cam.depth_view, 240, 240);
            // cv::imshow("color_img",cam.color_view);
            // cv::imshow("depth_img",cam.depth_view);
            // cv::imshow("depth_norm",cam.depth_norm);
            // cv::imshow("color_tape",cam.color_tape_view);
            // cv::imshow("color_tape_proc",cam.color_tape_proc);
            
            int key = cv::waitKey(30);
            if(key == 27) break;
        }
    #endif
    ROS_INFO("END");
    cv::destroyAllWindows();

    return 0;
}