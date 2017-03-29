#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <iostream>

#include "sensor_msgs/CompressedImage.h"
#include "std_msgs/String.h"
#include "mono_camera.h"

using namespace std;

int main(int argc, char **argv)
{
    printf("Sax");
    ros::init(argc, argv, "zeabus_vision_mono_start_node");
    ros::NodeHandle nh;
    string param;
    // nh.param<std::string>("mono_start_node/camera",param,"/cv_camera/image_raw/compressed");
    miltbot::MonoCamera cam;

    while(ros::ok())
    {
        ros::spinOnce();
        if(!cam.color_view.data)
        {
            continue;
        }
        cv::namedWindow("color_img");
        // cam.resizeImage(imageWidth,imageHeight);
        cv::imshow("img",cam.color_view);
        int key = cv::waitKey(30);
        if(key == 27) break;
    }
    cv::destroyAllWindows();

    return 0;
}