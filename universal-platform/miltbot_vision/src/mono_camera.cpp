#include "mono_camera.h"

namespace miltbot {

MonoCamera::MonoCamera(void):
    color_image_sub_topic_name_(""),
    depth_image_sub_topic_name_("")
{
    this->color_image_sub = nh_.subscribe("camera/rgb/image_color/compressed", 1, &MonoCamera::colorImageCallback, this);
    this->depth_image_sub = nh_.subscribe("camera/depth/image", 1, &MonoCamera::depthImageCallback, this);
}

MonoCamera::~MonoCamera(void)
{

}

void MonoCamera::colorImageCallback(const sensor_msgs::CompressedImage& msg) {
    std::vector<unsigned char> array = msg.data;
    // ROS_INFO("Get Data Color %ld",array.size());
    this->color_view = cv::imdecode(array,1);
    // ROS_WARN("Sizeeee : %d x %d",color_view.rows, color_view.cols);
}

void MonoCamera::depthImageCallback(const sensor_msgs::ImageConstPtr& msg) {
    // std::vector<unsigned char> array = msg.data;
    // ROS_INFO("Get Data Depth %ld",array.size());
    // this->depth_view = cv::imdecode(array,CV_LOAD_IMAGE_ANYDEPTH);
    // ROS_WARN("Sizeeee : %d x %d",depth_view.rows, depth_view.cols);
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);//now cv_ptr is the matrix, do not forget "TYPE_" before "16UC1"
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    this->depth_view = cv_ptr->image;
    cv::normalize(this->depth_view, this->depth_norm, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    
    ROS_WARN("Value1 : %d %lf",depth_view.at<unsigned char>(298, 143),depth_view.at<float>(298, 143));
    ROS_WARN("Value2 : %d %lf",depth_view.at<unsigned char>(393 ,417),depth_view.at<float>(393 ,417));
    ROS_WARN("Value3 : %d %lf",depth_view.at<unsigned char>(396, 579),depth_view.at<float>(396, 579));
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

void MonoCamera::resizeImage(cv::Mat &src, int imageWidth, int imageHeight) {
	cv::resize(src,src,cv::Size(imageWidth,imageHeight));
	// imageSize = view.size();
}

void MonoCamera::flipImage(cv::Mat &src, int id) {
    cv::flip(src,src,id);
}

}