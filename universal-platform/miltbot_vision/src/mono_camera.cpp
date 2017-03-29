#include "mono_camera.h"

namespace miltbot {

MonoCamera::MonoCamera(void):
    color_image_sub_topic_name_(""),
    depth_image_sub_topic_name_("")
{

}

MonoCamera::~MonoCamera(void)
{

}

// MonoCamera::MonoCamera()
// {
//     this->name = name;
//     // imageSize = cv::Size(imageWidth,imageHeight);
//     // sub = n.subscribe(name, 1000, &MonoCamera::monoCameraCallback,this);
//     // cout << name << endl;
// }

void MonoCamera::colorImageCallback(const sensor_msgs::CompressedImage& msg)
{
    std::vector<unsigned char> array = msg.data;
    color_view = cv::imdecode(array,1);
}

// bool MonoCamera::getParameter(string ns, ros::NodeHandle& n)
// {
//     checkParameter = n.getParam(ns+"/camera_matrix/data",cmData);
//     if(!checkParameter) return false;
//     cameraMatrix = cv::Mat(cv::Size(3,3),CV_32F,cmData.data());
//     checkParameter = n.getParam(ns+"/distortion_coefficients/rows",distortRows);
//     if(!checkParameter) return false;
//     checkParameter = n.getParam(ns+"/distortion_coefficients/cols",distortCols);
//     if(!checkParameter) return false;
//     checkParameter = n.getParam(ns+"/distortion_coefficients/data",distortData);
//     if(!checkParameter) return false;
//     distCoeffs = cv::Mat(cv::Size(distortCols,distortRows),CV_32F,distortData.data());
//     checkParameter = n.getParam(ns+"/rectification_matrix/rows",rectRows);
//     if(!checkParameter) return false;
//     checkParameter = n.getParam(ns+"/rectification_matrix/cols",rectCols);
//     if(!checkParameter) return false;
//     checkParameter = n.getParam(ns+"/rectification_matrix/data",rectData);
//     if(!checkParameter) return false;
//     rectificationMatrix = cv::Mat(cv::Size(rectCols,rectRows),CV_32F,rectData.data());
//     checkParameter = n.getParam(ns+"/projection_matrix/rows",projectRows);
//     if(!checkParameter) return false;
//     checkParameter = n.getParam(ns+"/projection_matrix/cols",projrectCols);
//     if(!checkParameter) return false;
//     checkParameter = n.getParam(ns+"/projection_matrix/data",projectData);
//     if(!checkParameter) return false;
//     projectionMatrix = cv::Mat(cv::Size(projrectCols,projectRows),CV_32F,projectData.data());
//     return true;
// }

// void MonoCamera::resizeImage(int imageWidth,int imageHeight)
// {
// 	cv::resize(view,view,cv::Size(imageWidth,imageHeight));
// 	imageSize = view.size();
// }

// void MonoCamera::flipImage(int id)
// {
//     cv::flip(view,view,id);
// }

}