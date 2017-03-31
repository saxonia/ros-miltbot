// #ifndef MONOCAMERA_H
// #define MONOCAMERA_H

#include <opencv2/opencv.hpp>
#include <iostream>

#include <ros/ros.h>
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/image_encodings.h>

#include "sensor_msgs/CompressedImage.h"
// #include "sensor_msgs/Image.h"

#ifndef __MILTBOT_VISION_MONO_CAMERA
#define __MILTBOT_VISION_MONO_CAMERA

namespace miltbot {

// const int imageWidth = 640;
// const int imageHeight = 480;

// #pragma once
class MonoCamera
{
    public:
        MonoCamera(void);

        ~MonoCamera(void);

        void colorImageCallback(const sensor_msgs::CompressedImage& msg);

        void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);

        // bool getParameter(string ns, ros::NodeHandle& n);

        void resizeImage(cv::Mat &src, int imageWidth,int imageHeight);

        void flipImage(cv::Mat &src, int id);

    private:

    public:
        // std::string name;
        cv::Mat color_view;
        cv::Mat depth_view;
        cv::Mat mono_view;
        cv::Mat depth_norm;
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
        ros::Subscriber color_image_sub;
        ros::Subscriber depth_image_sub;

        std::string color_image_sub_topic_name_;
        std::string depth_image_sub_topic_name_;

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