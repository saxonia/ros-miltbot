#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <iostream>

#include "sensor_msgs/CompressedImage.h"
#include "std_msgs/String.h"
#include "mono_camera.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "zeabus_vision_mono_start_node");
    ros::NodeHandle nh;
    miltbot::MonoCamera cam;
    ros::Rate rate(30);
    ROS_INFO("Before loop");
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
        if(!cam.color_view.data || !cam.depth_view.data) {
            continue;
        }
        cv::namedWindow("color_img");
        cv::namedWindow("depth_img");
        // cam.resizeImage(cam.color_view, 240, 240);
        // cam.resizeImage(cam.depth_view, 240, 240);
        cv::imshow("color_img",cam.color_view);
        cv::imshow("depth_img",cam.depth_view);
        cv::imshow("depth_norm",cam.depth_norm);
        int key = cv::waitKey(30);
        if(key == 27) break;
    }
    cv::destroyAllWindows();

    return 0;
}