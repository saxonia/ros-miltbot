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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "door_detection_node");
    ros::NodeHandle nh;
    ROS_INFO("START");

    std::string is_front_lift_service_name("is_front_lift");

    #ifdef IMAGE
        ROS_INFO("IMAGE MODE");
        std::string package_path = ros::package::getPath("miltbot_vision");
        cv::String image_path(package_path + "/images/door.jpg"); // by default
        if( argc > 1)
        {
            image_path = argv[1];
        }
        cv::Mat src;
        src = cv::imread(image_path, cv::IMREAD_COLOR);
        cv::resize(src,src,cv::Size(src.cols/2,src.rows/2));
        cv::namedWindow("src");
        cv::imshow("src",src);
        cv::waitKey(0);
        // if(key == 27) 
    #endif
    #ifdef KINECT_STREAM
        ROS_INFO("KINECT STREAM MODE");
        miltbot::MonoCamera cam;
        cam.createTrackbars();
        ros::Rate rate(30);
        ROS_INFO("Before loop");
        while(ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
            if(!cam.color_view.data || !cam.depth_view.data) {
                continue;
            }
            cam.detectColorTape(cam.color_view);
            cv::namedWindow("color_img");
            cv::namedWindow("depth_img");
            // cam.resizeImage(cam.color_view, 240, 240);
            // cam.resizeImage(cam.depth_view, 240, 240);
            cv::imshow("color_img",cam.color_view);
            // cv::imshow("depth_img",cam.depth_view);
            cv::imshow("depth_norm",cam.depth_norm);
            cv::imshow("color_tape",cam.color_tape_view);
            int key = cv::waitKey(30);
            if(key == 27) break;
        }
    #endif
    ROS_INFO("END");
    cv::destroyAllWindows();

    return 0;
}