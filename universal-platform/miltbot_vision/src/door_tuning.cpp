#include <Windows.h>
#include <iostream>
#include <stdio.h>
#include <NuiApi.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>
#include "KinectConnector.h"

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

// ------------------- declare global variable ------------- //
int H_MIN = 0;
int H_MAX = 179;
int L_MIN = 0;
int L_MAX = 255;
int S_MIN = 0;
int S_MAX = 255;
int V_MIN = 0;
int V_MAX = 255;
//Red/////////////
//int H_R_MIN = 162;
//int S_R_MIN = 109;
//int V_R_MIN = 103;
//int H_R_MAX = 255;
//int S_R_MAX = 255;
//int V_R_MAX = 255;
int H_R_MIN = 161;
int S_R_MIN = 42;
int V_R_MIN = 0;
int H_R_MAX = 179;
int S_R_MAX = 255;
int V_R_MAX = 255;
//Orange
int H_O_MIN = 0;
int S_O_MIN = 136;
int V_O_MIN = 0;
int H_O_MAX = 13;
int S_O_MAX = 247;
int V_O_MAX = 247;
//Yellow//////////
int H_Y_MIN = 24;
int S_Y_MIN = 88;
int V_Y_MIN = 0;
int H_Y_MAX = 43;
int S_Y_MAX = 255;
int V_Y_MAX = 255;
//Yellow Orange//////
int H_YO_MIN = 8;
int S_YO_MIN = 87;
int V_YO_MIN = 38;
int H_YO_MAX = 24;
int S_YO_MAX = 255;
int V_YO_MAX = 255;
//Megenta//////////////
int H_MA_MIN = 143;
int S_MA_MIN = 95;
int V_MA_MIN = 0;
int H_MA_MAX = 169;
int S_MA_MAX = 255;
int V_MA_MAX = 255;
//Pink//////////////
int H_PI_MIN = 157;
int S_PI_MIN = 51;
int V_PI_MIN = 30;
int H_PI_MAX = 162;
int S_PI_MAX = 219;
int V_PI_MAX = 240;
//Purple////////////
int H_PU_MIN = 116;
int S_PU_MIN = 81;
int V_PU_MIN = 35;
int H_PU_MAX = 143;
int S_PU_MAX = 255;
int V_PU_MAX = 255;
//Green Dark/////////
int H_GD_MIN = 56;
int S_GD_MIN = 43;
int V_GD_MIN = 0;
int H_GD_MAX = 81;
int S_GD_MAX = 255;
int V_GD_MAX = 255;
//Green light////////
int H_GL_MIN = 44;
int S_GL_MIN = 0;
int V_GL_MIN = 0;
int H_GL_MAX = 52;
int S_GL_MAX = 255;
int V_GL_MAX = 255;
//Green Blue//////////
int H_GB_MIN = 80;
int S_GB_MIN = 75;
int V_GB_MIN = 0;
int H_GB_MAX = 99;
int S_GB_MAX = 255;
int V_GB_MAX = 255;
//Blue//////////
int H_B_MIN = 99;
int S_B_MIN = 107;
int V_B_MIN = 0;
int H_B_MAX = 114;
int S_B_MAX = 255;
int V_B_MAX = 255;
//////////////////////
const string trackbarWindowName = "Trackbars";

// Mouse Callback function
void CallBackFunc(int event, int x, int y, int flags, void *userdata)
{
	if(event == EVENT_LBUTTONDOWN)
	{
		cout << "x = " << x << ", y = " << y << endl;
		Mat m = *(Mat*)userdata;
		int b = m.at<Vec3b>(y,x)[0];
		int g = m.at<Vec3b>(y,x)[1];
		int r = m.at<Vec3b>(y,x)[2];
		cout << "B = " << b << " " << "G = " << g << " " << "R = " << r << endl;
		cvtColor(m,m,CV_BGR2HSV);
		int h = m.at<Vec3b>(y,x)[0];
		int s = m.at<Vec3b>(y,x)[1];
		int v = m.at<Vec3b>(y,x)[2];
		cout << "H = " << h << " s = " << s << " v = " << v << "\n";
		cout << "\n";
	}
}

void on_trackbar( int, void* )
{
	//This function gets called whenever a trackbar position is changed
}

// Create Trackbars for color Segmentation
void createTrackbars()
{
	namedWindow(trackbarWindowName,CV_WINDOW_AUTOSIZE);
	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf( TrackbarName, "H_MIN", H_MIN);
	sprintf( TrackbarName, "H_MAX", H_MAX);
	sprintf( TrackbarName, "L_MIN", L_MIN);
	sprintf( TrackbarName, "L_MAX", L_MAX);
	sprintf( TrackbarName, "S_MIN", S_MIN);
	sprintf( TrackbarName, "S_MAX", S_MAX);
	sprintf( TrackbarName, "V_MIN", V_MIN);
	sprintf( TrackbarName, "V_MAX", V_MAX);

	createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar );
	createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar );
	createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar );
	createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar );
	createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar );
	createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar );
}

int findBiggestContour(vector< vector<Point> > contours)
{
	int indexOfBiggestContour = -1;
	int sizeOfBiggestContour = 0;
	for(int i = 0;i < contours.size();i++)
	{
		if(contourArea(contours[i]) > sizeOfBiggestContour){
			sizeOfBiggestContour = contourArea(contours[i]);
			indexOfBiggestContour = i;
		}
	}
	if(indexOfBiggestContour == -1) return -1;
	return indexOfBiggestContour;
}

Mat deleteNoise(Mat src,Mat inRange)
{
	Mat imgBitwise;
	bitwise_and(src,src,imgBitwise,inRange);
	Mat imgMorph;
	/*Mat kernel = Mat::ones(Size(20,20),CV_8U);
	Mat kernel1 = getStructuringElement(MORPH_ELLIPSE,Size(20,20));
	erode(imgBitwise,imgMorph,kernel1);
	dilate(imgMorph,imgMorph,kernel1);
	morphologyEx(imgMorph,imgMorph,MORPH_OPEN,kernel);*/

	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(10,10));
	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));

	erode(imgBitwise,imgMorph,erodeElement);
	erode(imgMorph,imgMorph,erodeElement);

	dilate(imgMorph,imgMorph,dilateElement);
	dilate(imgMorph,imgMorph,dilateElement);

	return imgMorph;
}


Mat locatePosition(Mat src,Mat imgProc,string color)
{
	Mat img8u = imgProc.clone();
	cvtColor(img8u,img8u,CV_BGR2GRAY);
	img8u.convertTo(img8u,CV_8UC1);

	Mat res = Mat::zeros(src.rows,src.cols,CV_8UC3);
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(img8u,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
	if(contours.size() > 0)
	{
		//int BiggestContourIdx = findBiggestContour(contours);
		res = imgProc.clone();
		//if(BiggestContourIdx < 0) return res;
		drawContours(res,contours,-1,Scalar(255,255,255));
		
		for(int i = 0;i < contours.size();i++)
		{
			Moments mu = moments(contours[i]);
			int x = mu.m10/mu.m00;
			int y = mu.m01/mu.m00;
			putText(res,color,Point(x-20,y-30),1,1,Scalar(255,255,255));
			putText(src,color,Point(x-20,y-30),1,1,Scalar(0,0,0));
		}
		
	}
	return res;
}

int setColorRange(int push)
{
	if(push == 27) return -1;
	if(push == 'c'){
		String temp;
		cin >> temp;
		if(temp == "R"){
			H_R_MIN = H_MIN;
			S_R_MIN = S_MIN;
			V_R_MIN = V_MIN;
			H_R_MAX = H_MAX;
			S_R_MAX = S_MAX;
			V_R_MAX = V_MAX;
			cout << "change red successful\n";
		}
		else if(temp == "Y"){
			H_Y_MIN = H_MIN;
			S_Y_MIN = S_MIN;
			V_Y_MIN = V_MIN;
			H_Y_MAX = H_MAX;
			S_Y_MAX = S_MAX;
			V_Y_MAX = V_MAX;
			cout << "change yellow successful\n";
		}
		else if(temp == "YO"){
			H_YO_MIN = H_MIN;
			S_YO_MIN = S_MIN;
			V_YO_MIN = V_MIN;
			H_YO_MAX = H_MAX;
			S_YO_MAX = S_MAX;
			V_YO_MAX = V_MAX;
			cout << "change yellow orange successful\n";
		}
		else if(temp == "MA"){
			H_MA_MIN = H_MIN;
			S_MA_MIN = S_MIN;
			V_MA_MIN = V_MIN;
			H_MA_MAX = H_MAX;
			S_MA_MAX = S_MAX;
			V_MA_MAX = V_MAX;
			cout << "change pink successful\n";
		}
		else if(temp == "PI"){
			H_PI_MIN = H_MIN;
			S_PI_MIN = S_MIN;
			V_PI_MIN = V_MIN;
			H_PI_MAX = H_MAX;
			S_PI_MAX = S_MAX;
			V_PI_MAX = V_MAX;
			cout << "change pink successful\n";
		}
		else if(temp == "PU"){
			H_PU_MIN = H_MIN;
			S_PU_MIN = S_MIN;
			V_PU_MIN = V_MIN;
			H_PU_MAX = H_MAX;
			S_PU_MAX = S_MAX;
			V_PU_MAX = V_MAX;
			cout << "change purple successful\n";
		}
		else if(temp == "GD"){
			H_GD_MIN = H_MIN;
			S_GD_MIN = S_MIN;
			V_GD_MIN = V_MIN;
			H_GD_MAX = H_MAX;
			S_GD_MAX = S_MAX;
			V_GD_MAX = V_MAX;
			cout << "change green dark successful\n";
		}
		else if(temp == "GL"){
			H_GL_MIN = H_MIN;
			S_GL_MIN = S_MIN;
			V_GL_MIN = V_MIN;
			H_GL_MAX = H_MAX;
			S_GL_MAX = S_MAX;
			V_GL_MAX = V_MAX;
			cout << "change green light successful\n";
		}
		else if(temp == "GB"){
			H_GB_MIN = H_MIN;
			S_GB_MIN = S_MIN;
			V_GB_MIN = V_MIN;
			H_GB_MAX = H_MAX;
			S_GB_MAX = S_MAX;
			V_GB_MAX = V_MAX;
			cout << "change green blue successful\n";
		}
		else if(temp == "B"){
			H_B_MIN = H_MIN;
			S_B_MIN = S_MIN;
			V_B_MIN = V_MIN;
			H_B_MAX = H_MAX;
			S_B_MAX = S_MAX;
			V_B_MAX = V_MAX;
			cout << "change blue successful\n";
		}
		else cout << "unknown command\n";

	}
	if(push == 'p'){
		printf("red    =    %d %d %d    -    %d %d %d\n",H_R_MIN,S_R_MIN,V_R_MIN,H_R_MAX,S_R_MAX,V_R_MAX);
		printf("yellow    =    %d %d %d    -    %d %d %d\n",H_Y_MIN,S_Y_MIN,V_Y_MIN,H_Y_MAX,S_Y_MAX,V_Y_MAX);
		printf("yellow    =    %d %d %d    -    %d %d %d\n",H_YO_MIN,S_YO_MIN,V_YO_MIN,H_YO_MAX,S_YO_MAX,V_YO_MAX);
		printf("pink    =    %d %d %d    -    %d %d %d\n",H_PI_MIN,S_PI_MIN,V_PI_MIN,H_PI_MAX,S_PI_MAX,V_PI_MAX);
		printf("purple    =    %d %d %d    -    %d %d %d\n",H_PU_MIN,S_PU_MIN,V_PU_MIN,H_PU_MAX,S_PU_MAX,V_PU_MAX);
		printf("green D    =    %d %d %d    -    %d %d %d\n",H_GD_MIN,S_GD_MIN,V_GD_MIN,H_GD_MAX,S_GD_MAX,V_GD_MAX);
		printf("green L    =    %d %d %d    -    %d %d %d\n",H_GL_MIN,S_GL_MIN,V_GL_MIN,H_GL_MAX,S_GL_MAX,V_GL_MAX);
		printf("green B    =    %d %d %d    -    %d %d %d\n",H_GB_MIN,S_GB_MIN,V_GB_MIN,H_GB_MAX,S_GB_MAX,V_GB_MAX);
		printf("blue    =    %d %d %d    -    %d %d %d\n",H_B_MIN,S_B_MIN,V_B_MIN,H_B_MAX,S_B_MAX,V_B_MAX);
	}
	return 0;
}



int main(){

	KinectConnector kin = KinectConnector();
	if(!kin.Connect()) return 1;

	createTrackbars();

	while(true){
		Mat depthImg;
		Mat colorImg;
		Mat indexImg;
		Mat pointImg;

		kin.GrabData(depthImg,colorImg,indexImg,pointImg);

		Mat hsv_image;
		Mat hsv_inRange;

		Mat tempColorImg = colorImg.clone();
		//GaussianBlur(tempColorImg,tempColorImg,Size(5,5),3);
		medianBlur(tempColorImg,tempColorImg,11);
		//blur(tempColorImg,tempColorImg,Size(5,5));
		Mat convertColorImg;
		tempColorImg.convertTo(convertColorImg,CV_8UC3);
		cvtColor(convertColorImg,hsv_image,CV_BGR2HSV);
		Mat tt;

		//Detect Color
		//inRange(hsv_image,Scalar(H_MIN,L_MIN,S_MIN),Scalar(H_MAX,L_MAX,S_MAX),tt);
		//inRange(hsv_image,Scalar(164,140,55),Scalar(198,255,188),hsv_inRange);
		Mat hsv_blur;
		GaussianBlur(hsv_image,hsv_image,Size(3,3),1);
		inRange(hsv_image,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),hsv_inRange);

		//show the adjust result
		imshow("hsv",hsv_inRange);

		//Red Color
		//inRange(hsv_image,Scalar(164,140,50),Scalar(198,255,188),hsv_inRange);
		////Yellow Color
		//inRange(hsv_image,Scalar(24,198,67),Scalar(59,255,255),hsv_inRange);
		//Mat yellowImage = deleteNoise(convertColorImg,hsv_inRange);
		////Yellow Orange Color
		//inRange(hsv_image,Scalar(14,206,61),Scalar(24,255,255),hsv_inRange);
		//Mat yellowOrangeImage = deleteNoise(convertColorImg,hsv_inRange);
		////Pink Color
		//inRange(hsv_image,Scalar(146,40,0),Scalar(162,255,255),hsv_inRange);
		//Mat pinkImage = deleteNoise(convertColorImg,hsv_inRange);
		////Purple Color
		//inRange(hsv_image,Scalar(14,206,61),Scalar(24,255,255),hsv_inRange);
		//Mat purpleImage = deleteNoise(convertColorImg,hsv_inRange);
		//Green Dark Color
		//inRange(hsv_image,Scalar(164,140,50),Scalar(198,255,188),hsv_inRange);
		//Mat greenDarkImage = deleteNoise(convertColorImg,hsv_inRange);
		////Green Light Color
		//inRange(hsv_image,Scalar(164,140,50),Scalar(198,255,188),hsv_inRange);
		//Mat greenLightImage = deleteNoise(convertColorImg,hsv_inRange);
		
		//Red Color
		inRange(hsv_image,Scalar(H_R_MIN,S_R_MIN,V_R_MIN),Scalar(H_R_MAX,S_R_MAX,V_R_MAX),hsv_inRange);
		Mat redtmp = hsv_inRange.clone();
		Mat redImage = deleteNoise(convertColorImg,hsv_inRange);
		Mat redRes = locatePosition(colorImg,redImage,"RED");
		//Orange Color
		inRange(hsv_image,Scalar(H_O_MIN,S_O_MIN,V_O_MIN),Scalar(H_O_MAX,S_O_MAX,V_O_MAX),hsv_inRange);
		//Mat redtmp = hsv_inRange.clone();
		Mat orangeImage = deleteNoise(convertColorImg,hsv_inRange);
		Mat orangeRes = locatePosition(colorImg,orangeImage,"ORANGE");
		//Yellow Color
		inRange(hsv_image,Scalar(H_Y_MIN,S_Y_MIN,V_Y_MIN),Scalar(H_Y_MAX,S_Y_MAX,V_Y_MAX),hsv_inRange);
		Mat yellowImage = deleteNoise(convertColorImg,hsv_inRange);
		Mat yellowRes = locatePosition(colorImg,yellowImage,"YELLOW");
		//Yellow Orange Color
		inRange(hsv_image,Scalar(H_YO_MIN,S_YO_MIN,V_YO_MIN),Scalar(H_YO_MAX,S_YO_MAX,V_YO_MAX),hsv_inRange);
		Mat yellowOrangeImage = deleteNoise(convertColorImg,hsv_inRange);
		Mat yellowOrangeRes = locatePosition(colorImg,yellowOrangeImage,"YO");
		
		//Magenta Color
		inRange(hsv_image,Scalar(H_MA_MIN,S_MA_MIN,V_MA_MIN),Scalar(H_MA_MAX,S_MA_MAX,V_MA_MAX),hsv_inRange);
		Mat megentaImage = deleteNoise(convertColorImg,hsv_inRange);
		Mat megentaRes = locatePosition(colorImg,megentaImage,"MAGENTA");
		//Pink Color
		inRange(hsv_image,Scalar(H_PI_MIN,S_PI_MIN,V_PI_MIN),Scalar(H_PI_MAX,S_PI_MAX,V_PI_MAX),hsv_inRange);
		Mat pinkImage = deleteNoise(convertColorImg,hsv_inRange);
		Mat pinkRes = locatePosition(colorImg,pinkImage,"PINK");
		//Purple Color
		inRange(hsv_image,Scalar(H_PU_MIN,S_PU_MIN,V_PU_MIN),Scalar(H_PU_MAX,S_PU_MAX,V_PU_MAX),hsv_inRange);
		Mat purpleImage = deleteNoise(convertColorImg,hsv_inRange);
		Mat purpleRes = locatePosition(colorImg,purpleImage,"PURPLE");
		
		//Green Dark Color
		inRange(hsv_image,Scalar(H_GD_MIN,S_GD_MIN,V_GD_MIN),Scalar(H_GD_MAX,S_GD_MAX,V_GD_MAX),hsv_inRange);
		Mat greenDarkImage = deleteNoise(convertColorImg,hsv_inRange);
		Mat greenDarkRes = locatePosition(colorImg,greenDarkImage,"DARK GREEN");
		imshow("inrange",hsv_inRange);
		//Green Light Color
		inRange(hsv_image,Scalar(H_GL_MIN,S_GL_MIN,V_GL_MIN),Scalar(H_GL_MAX,S_GL_MAX,V_GL_MAX),hsv_inRange);
		Mat greenLightImage = deleteNoise(convertColorImg,hsv_inRange);
		Mat greenLightRes = locatePosition(colorImg,greenLightImage,"LIGHT GREEN");
		//Green Blue Color
		inRange(hsv_image,Scalar(H_GB_MIN,S_GB_MIN,V_GB_MIN),Scalar(H_GB_MAX,S_GB_MAX,V_GB_MAX),hsv_inRange);
		Mat greenBlueImage = deleteNoise(convertColorImg,hsv_inRange);
		Mat greenBlueRes = locatePosition(colorImg,greenBlueImage,"BLUE GREEN");
		//Blue Color
		inRange(hsv_image,Scalar(H_B_MIN,S_B_MIN,V_B_MIN),Scalar(H_B_MAX,S_B_MAX,V_B_MAX),hsv_inRange);
		Mat blueImage = deleteNoise(convertColorImg,hsv_inRange);
		Mat blueRes = locatePosition(colorImg,blueImage,"BLUE");
		
		
		//Merge all color image
		Mat res1;
		Mat tmp2;
		bitwise_or(redImage,yellowImage,tmp2);
		bitwise_or(tmp2,yellowOrangeImage,tmp2);
		bitwise_or(tmp2,megentaImage,tmp2);
		bitwise_or(tmp2,purpleImage,tmp2);
		bitwise_or(tmp2,greenDarkImage,tmp2);
		bitwise_or(tmp2,greenLightImage,tmp2);
		bitwise_or(tmp2,greenBlueImage,tmp2);
		//bitwise_or(tmp2,pink,tmp2);
		bitwise_or(tmp2,blueImage,res1);

		Mat res;
		Mat tmp1;
		bitwise_or(redRes,yellowRes,tmp1);
		bitwise_or(tmp1,yellowOrangeRes,tmp1);
		bitwise_or(tmp1,megentaRes,tmp1);
		bitwise_or(tmp1,purpleRes,tmp1);
		bitwise_or(tmp1,greenDarkRes,tmp1);
		bitwise_or(tmp1,greenLightRes,tmp1);
		bitwise_or(tmp1,greenBlueRes,tmp1);
		//bitwise_or(tmp1,pinkRes,tmp1);
		bitwise_or(tmp1,orangeRes,tmp1);
		bitwise_or(tmp1,blueRes,res);

		//imshow("depthImg",depthImg);
		imshow("colorImg",colorImg);
		//imshow("RES",temp3);
		//imshow("inrange",tt);
		imshow("redRes",res1);
		//imshow("redImage",redImage);
		//imshow
		imshow("RES",res);
		//imshow("indexImg",indexImg);
		//imshow("pointImg",pointImg);
		setMouseCallback("colorImg",CallBackFunc,&colorImg);

		int push = waitKey(1);
		int key = setColorRange(push);
		if(key == -1) break;
	}
	return 0;
}

