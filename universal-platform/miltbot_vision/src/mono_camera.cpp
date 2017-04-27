#include "mono_camera.h"

namespace miltbot {

MonoCamera::MonoCamera(void):
    color_image_sub_topic_name_("camera/rgb/image_color"),
    depth_image_sub_topic_name_("camera/depth/image_raw"),
    is_front_lift_service_name_("is_front_lift")
{
    // nh_.param("color_image_sub_topic", color_image_sub_topic_name_, color_image_sub_topic_name_);
    // nh_.param("depth_image_sub_topic", depth_image_sub_topic_name_, depth_image_sub_topic_name_);
    this->color_image_sub = nh_.subscribe(color_image_sub_topic_name_, 1, &MonoCamera::colorImageCallback, this);
    this->depth_image_sub = nh_.subscribe(depth_image_sub_topic_name_, 1, &MonoCamera::depthImageCallback, this);
    this->is_front_lift_server = nh_.advertiseService(is_front_lift_service_name_, &MonoCamera::isFrontLiftService, this);
}

MonoCamera::~MonoCamera(void)
{

}

void MonoCamera::colorImageCallback(const sensor_msgs::ImageConstPtr& msg) {
    // std::vector<unsigned char> array = msg.data;
    // this->color_view = cv::imdecode(array,1);
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);//now cv_ptr is the matrix, do not forget "TYPE_" before "16UC1"
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    this->color_view = cv_ptr->image;
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
    
    // ROS_WARN("Value1 : %d %lf",depth_view.at<unsigned char>(298, 143),depth_view.at<float>(298, 143));
    // ROS_WARN("Value2 : %d %lf",depth_view.at<unsigned char>(393 ,417),depth_view.at<float>(393 ,417));
    // ROS_WARN("Value3 : %d %lf",depth_view.at<unsigned char>(396, 579),depth_view.at<float>(396, 579));
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

bool MonoCamera::detectColorTape(cv::Mat src) {
    cv::Mat hsv_image;
	cv::Mat hsv_inRange;
    cv::Mat color_img;
    cv::medianBlur(src, color_img, 11);
	cv::cvtColor(color_img, hsv_image, CV_BGR2HSV);
    //Green Light Color
	cv::inRange(hsv_image, cv::Scalar(H_MIN,S_MIN,V_MIN), cv::Scalar(H_MAX,S_MAX,V_MAX), hsv_inRange);
	cv::Mat greenLightImage = this->deleteNoise(color_img,hsv_inRange);
	// cv::Mat greenLightRes = locatePosition(colorImg,greenLightImage,"LIGHT GREEN");
    this->color_tape_view = hsv_inRange.clone();
    this->color_tape_proc = greenLightImage.clone();

    return this->verifyLiftDoor();
}

// Create Trackbars for color Segmentation
void MonoCamera::createTrackbars()
{
	cv::namedWindow(trackbarWindowName,CV_WINDOW_AUTOSIZE);
	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf( TrackbarName, "H_MIN", H_MIN);
	sprintf( TrackbarName, "H_MAX", H_MAX);
	// sprintf( TrackbarName, "L_MIN", L_MIN);
	// sprintf( TrackbarName, "L_MAX", L_MAX);
	sprintf( TrackbarName, "S_MIN", S_MIN);
	sprintf( TrackbarName, "S_MAX", S_MAX);
	sprintf( TrackbarName, "V_MIN", V_MIN);
	sprintf( TrackbarName, "V_MAX", V_MAX);

	cv::createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, 128, on_trackbar);
	cv::createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, 128, on_trackbar);
	cv::createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar);
	cv::createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar);
	cv::createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar);
	cv::createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar);
}

cv::Mat MonoCamera::deleteNoise(cv::Mat src, cv::Mat inRange) {
	cv::Mat imgBitwise;
	cv::bitwise_and(src,src,imgBitwise,inRange);
	cv::Mat imgMorph;
	/*Mat kernel = Mat::ones(Size(20,20),CV_8U);
	Mat kernel1 = getStructuringElement(MORPH_ELLIPSE,Size(20,20));
	erode(imgBitwise,imgMorph,kernel1);
	dilate(imgMorph,imgMorph,kernel1);
	morphologyEx(imgMorph,imgMorph,MORPH_OPEN,kernel);*/

	cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(8,8));
	//dilate with larger element so make sure object is nicely visible
	cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10,10));

	cv::erode(imgBitwise,imgMorph,erodeElement);
	cv::erode(imgMorph,imgMorph,erodeElement);

	cv::dilate(imgMorph,imgMorph,dilateElement);
	cv::dilate(imgMorph,imgMorph,dilateElement);

	return imgMorph;
}

bool MonoCamera::verifyLiftDoor() {
    cv::Mat color_tape_mono_view;
    cv::cvtColor(this->color_tape_proc, color_tape_mono_view, CV_BGR2GRAY);
    std::vector< std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(color_tape_mono_view, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	if(contours.size() > 0)
	{
		//int BiggestContourIdx = findBiggestContour(contours);
		// res = imgProc.clone();
		//if(BiggestContourIdx < 0) return res;
		// drawContours(res,contours,-1,Scalar(255,255,255));

		// for(int i = 0;i < contours.size();i++)
		// {
		// 	Moments mu = moments(contours[i]);
		// 	int x = mu.m10/mu.m00;
		// 	int y = mu.m01/mu.m00;
		// 	putText(res,color,Point(x-20,y-30),1,1,Scalar(255,255,255));
		// 	putText(src,color,Point(x-20,y-30),1,1,Scalar(0,0,0));
		// }
        return true;
	}
    return false;
}

void on_trackbar( int, void* ) {
	//This function gets called whenever a trackbar position is changed
}

bool MonoCamera::isFrontLiftService(miltbot_vision::IsFrontLift::Request &req,
                        miltbot_vision::IsFrontLift::Response &res) {
	res.is_front_lift = true;
    // res.is_front_lift = detectColorTape(this->color_view);
    return true;
}

}