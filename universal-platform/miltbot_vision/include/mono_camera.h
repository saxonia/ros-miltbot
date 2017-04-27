// #ifndef MONOCAMERA_H
// #define MONOCAMERA_H

#include <opencv2/opencv.hpp>
#include <iostream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "sensor_msgs/CompressedImage.h"
// #include "sensor_msgs/Image.h"

#include "miltbot_vision/IsFrontLift.h"

#ifndef __MILTBOT_VISION_MONO_CAMERA
#define __MILTBOT_VISION_MONO_CAMERA

namespace miltbot {

// const int imageWidth = 640;
// const int imageHeight = 480;

// #pragma once
void on_trackbar( int, void* );


class MonoCamera
{
    public:
        MonoCamera(void);

        ~MonoCamera(void);

        void colorImageCallback(const sensor_msgs::ImageConstPtr& msg);

        void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);

        // bool getParameter(string ns, ros::NodeHandle& n);

        void resizeImage(cv::Mat &src, int imageWidth,int imageHeight);

        void flipImage(cv::Mat &src, int id);

        bool detectColorTape(cv::Mat src);

        void createTrackbars();

    private:
        cv::Mat deleteNoise(cv::Mat src, cv::Mat inRange);

        bool verifyLiftDoor();


        bool isFrontLiftService(miltbot_vision::IsFrontLift::Request &req,
                        miltbot_vision::IsFrontLift::Response &res);

    public:
        // std::string name;
        cv::Mat color_view;
        cv::Mat depth_view;
        cv::Mat mono_view;
        cv::Mat depth_norm;
        cv::Mat color_tape_view;
        cv::Mat color_tape_proc;
        // cv::Mat viewGray;
        // cv::Size imageSize;
        // cv::Mat cameraMatrix;
        // cv::Mat distCoeffs;
        // std::vector<cv::Mat> rvecs;
        // std::vector<cv::Mat> tvecs;
		// cv::Mat rectificationMatrix;
		// cv::Mat projectionMatrix;
		// cv::Mat viewRes;

    private:
        ros::NodeHandle nh_;

        //Subscriber
        ros::Subscriber color_image_sub;
        ros::Subscriber depth_image_sub;

        //Service Server
        ros::ServiceServer is_front_lift_server;

        std::string color_image_sub_topic_name_;
        std::string depth_image_sub_topic_name_;
        std::string is_front_lift_service_name_;

        int H_MIN = 42;
        int H_MAX = 70;
        int S_MIN = 115;
        // int S_MIN = 60;
        int S_MAX = 255;
        // int V_MIN = 26;
        int V_MIN = 32;
        int V_MAX = 255;
        const std::string trackbarWindowName = "Trackbars";

		// std::vector<float> cmData;
		// int distortRows;
		// int distortCols;
		// std::vector<float> distortData;
		// int rectRows;
		// int rectCols;
		// std::vector<float> rectData;
		// int projectRows;
		// int projrectCols;
		// std::vector<float> projectData;
		// bool checkParameter;
};

}

#endif