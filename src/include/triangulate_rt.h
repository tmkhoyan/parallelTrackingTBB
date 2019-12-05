
/*
 * Description:    Parallel tracker and feature extraction method initial release
 *
 * Author:          Tigran Mkhoyan
 * Email :          t.mkhoyan@tudelft.nl
 */



#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include "opencv2/flann/miniflann.hpp"
#include <opencv2/flann.hpp>
#include <opencv2/video/tracking.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <iostream>
#include <string>
#include <stdexcept>

#include <numeric>

#include <fstream>
#include <iterator>
#include <vector>
#include <array>
#include <math.h>
#include <iomanip>   
#include <algorithm>   

#include <ctime>
#include <math.h>

//mkdir
#include <sys/stat.h>
#include <stdexcept>
#include <exception>




#include <iostream>
#include <map>
#include <vector>
#include <sstream>      // std::stringstream
#include <algorithm>
#include <cctype>

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>


//thread and queue
#include <thread>         // std::thread, std::this_thread::yield
#include <chrono>  // for high_resolution_clock
#include <mutex>
#include <queue>
#include <thread>
#include <numeric>


//chrono
#include <chrono>
#include <condition_variable>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

//pylon
#include <pylon/PylonIncludes.h>
#include <pylon/gige/BaslerGigEInstantCamera.h>
#include <pylon/gige/_BaslerGigECameraParams.h>
#include <pylon/gige/BaslerGigEInstantCameraArray.h>

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::microseconds;

//pylon
using namespace Pylon;
using namespace std;

using namespace Basler_GigECameraParams;
using namespace Basler_GigEStreamParams;

// include helper
#include "../include/helper.h"
#include "utils.h" // key press etc
// include thread and queue

/* ## ---------------------------------------------- Kalman parameters------------------------------------------------------*/


// also track width
// #define NSTATE 6
// #define NMEAS 4

#define NSTATE 4
#define NMEAS 2
#define NCONTR 0
// #define NTRACKERS 10 // number of kalman filter instances

unsigned int type = CV_32F;



// // redis
// #include <sw/redis++/redis++.h> 
// using namespace sw::redis;

// #define REDIS_KEY_ID1 "cam1";
// #define REDIS_KEY_ID2 "cam2";

// #define IMG_ROW 480
// #define IMG_COL 640
// #define IMG_DEPTH 3 // 3 or 4 channel

// #define REDIS_SERVER_IP "tcp://localhost:6379"
// auto IMGTYPE= (IMG_DEPTH==3)? CV_8UC3 : CV_8UC4;


/*--------------------------------some parameters--------------------------------------*/
// int FPS = 30;
int32_t const MAX_FRAME_COUNT(10000000);
static const size_t c_maxCamerasToUse = 2;
int camWidth = 1088;
int camHeight = 600;
int exitCode = 0;
int img_W = 400;
int img_H = 600;
int txtFontFace = cv::FONT_HERSHEY_COMPLEX_SMALL;
double txtFontScale = 1;
int txtThickness = 1;
//file storage containers
// typedef std::chrono::high_resolution_clock Clock;
cv::FileStorage fs;
// cv::FileStorage fs_l;
// cv::FileStorage fs_r;
cv::FileStorage fs_o;
std::map<std::string,int> camIDmap; //indexing map for cameras to get right order


    /* ----------------------------------------------  triangulation specific functions -----------------------------------------------------*/
// calculate projection matrix
  cv::Mat calcProjMat(cv::Mat K,cv::Mat R,cv::Mat t){
  //rotation matrix 3x3; and R|T 3x4 matrix
  cv::Mat Rt(3, 4, CV_64FC1); 
  //Append translation vector to rotation matrix
  cv::hconcat(R,t,Rt);
  //Compute projection matrix K*[R|T]
  return(K*Rt);
  }

struct stereoParams {
	bool isSet;
	cv::Mat K1; //camera matrix 1
	cv::Mat K2; 
	cv::Mat D1; //distortion matrix
	cv::Mat D2;
	cv::Mat P1; // Projection matrix from origin WC to frame coords K1*[I|[0 0 0]T] camera 1 is origin
	cv::Mat P2; // K2*[R|t]        camera 2 need rotation and translation from cam 1
	cv::Mat Q;
	cv::Mat R; //rotation matrix     cam1-->2
	cv::Mat T; //translation vector cam1-->2
	//  constructor and destructor
	stereoParams(): 
		isSet(false), 
		K1(3,3,CV_64FC1), K2(3,3,CV_64FC1),
		D1(1,5,CV_64FC1), D2(1,5,CV_64FC1),
		P1(3,4,CV_64FC1), P2(3,4,CV_64FC1),
		Q(4,4,CV_64FC1),
		R(3,3,CV_64FC1),
		T(1,3,CV_64FC1)
	{} 
	~stereoParams() {} 
	// calculate projection matrix
	cv::Mat calcProjMat(cv::Mat K,cv::Mat R,cv::Mat t){
		//rotation matrix 3x3; and R|T 3x4 matrix
		cv::Mat Rt(3, 4, CV_64FC1); 
		//Append translation vector to rotation matrix
		cv::hconcat(R,t,Rt);
		//Compute projection matrix K*[R|T]
		return(K*Rt);
	};
	bool loadParams(cv::FileStorage fs){
		fs["K1"] >> K1; fs["K2"] >> K2;
		fs["D1"] >> D1; fs["D2"] >> D2;
		fs["R"]  >> R;
		fs["T"]  >> T;
		fs["Q"]  >> Q;
		return true;
	}
	// calculate projection matrices
	bool configureParams(){
		P1 = calcProjMat(K1,cv::Mat::eye(3,3,CV_64FC1),cv::Mat::zeros(3,1,CV_64FC1));
		P2 = calcProjMat(K2,R,T);
		isSet = true;
		return isSet;
	}
};


template <typename T> // type CV_64FC1 points preferred
cv::Mat triangulateUndistPoints(std::vector<T> pxyL, std::vector<T> pxyR,	stereoParams camParams){

	//undistort
	vector<cv::Point2d>  pxyL_unidst;
	vector<cv::Point2d>  pxyR_unidst;

	cv::undistortPoints(pxyL, pxyL_unidst, camParams.K1, camParams.D1, cv::noArray(), camParams.K1);
	cv::undistortPoints(pxyR, pxyR_unidst, camParams.K2, camParams.D2, cv::noArray(), camParams.K2);


	cv::Mat p4D; cv::triangulatePoints(camParams.P1, camParams.P2, pxyL_unidst, pxyR_unidst, p4D);
	// cout << P4D << endl; 
	// cv::Mat p4D; cv::triangulatePoints(camParams.P1, camParams.P2, pxyL, pxyR, p4D);
	cv::Mat pt3d; convertPointsFromHomogeneous(cv::Mat(p4D.t()).reshape(4, 1),pt3d); 
	// Mat pt3d; convertPointsFromHomogeneous(p4D.t(),pt3d);
	return pt3d;

}

cv::SimpleBlobDetector::Params setupBlobDetector(){
	// Setup SimpleBlobDetector parameters.
	cv::SimpleBlobDetector::Params params;

	// Change thresholds
	params.minThreshold = 10;
	// params.maxThreshold = 300;
	params.maxThreshold = 300;

	// Filter by Area.
	params.filterByArea = true;
	params.minArea = 1500;
	// params.minArea = 1000;

	// Filter by Circularity
	params.filterByCircularity = true;
	params.minCircularity = 0.1;

	// Filter by Convexity
	params.filterByConvexity = true;
	params.minConvexity = 0.87;

	// Filter by Inertia
	params.filterByInertia = true;
	params.minInertiaRatio = 0.01;

	return params;
}

    /* ----------------------------------------------  pylon specific functions -----------------------------------------------------*/


std::map<std::string,int> conFigurePylon(CBaslerGigEInstantCameraArray &cameras, CTlFactory& tlFactory, DeviceInfoList_t &devices, double FPS){

	// Create and attach all Pylon Devices.
	for ( size_t i = 0; i < cameras.GetSize(); ++i)
	{
		cameras[ i ].Attach( tlFactory.CreateDevice( devices[ i ]));
		// Print the model name of the camera and name of the camera.
		cout << "Using device " << cameras[ i ].GetDeviceInfo().GetModelName()       << endl;
		cout << "Using device " << cameras[ i ].GetDeviceInfo().GetUserDefinedName() << " (model: " << cameras[ i ].GetDeviceInfo().GetModelName() << ")" << endl;
		camIDmap[std::string(cameras[ i ].GetDeviceInfo().GetUserDefinedName())] = i;
	}
	/*-------------------Synchronous Freerun Setup---------------------------*/

	//		cameras.StartGrabbing(GrabStrategy_OneByOne, GrabLoop_ProvidedByInstantCamera);
	cameras.StartGrabbing();

	for (size_t i = 0; i < cameras.GetSize(); ++i)
	{
		cameras[i].Open();
		// check and Enable PTP
		if (GenApi::IsWritable(cameras[i].GevIEEE1588))
		{
			std::cout << "PTP available for [" << cameras[ i ].GetDeviceInfo().GetUserDefinedName() << "]. Setting Synchronous Freerun" << std::endl;
			cameras[i].GevIEEE1588.SetValue(true);
			// Make sure the frame trigger is set to Off to enable free run
			cameras[i].TriggerSelector.SetValue(Basler_GigECameraParams::TriggerSelector_FrameStart);
			cameras[i].TriggerMode.SetValue(Basler_GigECameraParams::TriggerMode_Off);
			// Let the free run start immediately without a specific start time
			cameras[i].SyncFreeRunTimerStartTimeLow.SetValue(0);
			cameras[i].SyncFreeRunTimerStartTimeHigh.SetValue(0);
			// Set the trigger rate to 30 frames per second
			cameras[i].SyncFreeRunTimerTriggerRateAbs.SetValue(FPS);
			// Apply the changes
			cameras[i].SyncFreeRunTimerUpdate.Execute();
			// Start the synchronous free run
			cameras[i].SyncFreeRunTimerEnable.SetValue(true);

		} else {
			std::cout << " no PTP available. Closing programm" << std::endl;
			exitCode = 1;
		}
	}

	// obtain first initial few frames
	CGrabResultPtr ptrGrabResultCam1;
	CGrabResultPtr ptrGrabResultCam2;

	for(int i=0; i< 10 ; i++){
		cameras[0].RetrieveResult( 10000, ptrGrabResultCam1, TimeoutHandling_ThrowException);
		cameras[1].RetrieveResult( 10000, ptrGrabResultCam2, TimeoutHandling_ThrowException);
		//get ptp parameters
		//cameras[cameraContextValue ].GevIEEE1588DataSetLatch.Execute();

		//clockId = cameras[cameraContextValue ].GevIEEE1588ClockId.GetValue();
		//offset  = cameras[cameraContextValue ].GevIEEE1588OffsetFromMaster.GetValue();
		std::cout << "getting first few frames, captured frame :"  << i << std::endl;
	}
	cout << "Camera configuration is done" << endl;

	return camIDmap;
}

void checkCamID(std::vector<std::string>  IDs, std::map<std::string,int> camIDmap){
	for(auto id : IDs){
			bool idIspresent = camIDmap.find(id) != camIDmap.end(); // check if id with this tring exhists otherwise throw exception
			if(!idIspresent)
				throw RUNTIME_EXCEPTION( "Cam ID" + id + "is not present in device list");
		}
	}

	cv::Mat convertPylonImage(CGrabResultPtr ptrGrabResultCam, bool cw90Rot){
		CImageFormatConverter formatConverter;
		formatConverter.OutputPixelFormat= PixelType_BGR8packed;
		cv::Mat openCvImage;
	CPylonImage pylonImage; //pylon image for conversion
	formatConverter.Convert(pylonImage, ptrGrabResultCam);
	formatConverter.OutputBitAlignment = Pylon::OutputBitAlignmentEnums::OutputBitAlignment_MsbAligned;
	formatConverter.OutputPixelFormat = Pylon::PixelType::PixelType_BGR8packed;
	openCvImage.create(pylonImage.GetHeight(), pylonImage.GetWidth(), CV_8UC3);
	memcpy(openCvImage.ptr(), pylonImage.GetBuffer(), 3*pylonImage.GetWidth()*pylonImage.GetHeight());
	openCvImage = cw90Rot? rotateImg90(openCvImage) : openCvImage;
	return openCvImage;

}


inline std::string makeValidDir(const std::string & str){

if (mkdir(str.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1)
{
    if( errno == EEXIST ) { // alredy exists
      std::cout << "folder: " << str << " --- exists. " << std::endl;
    } else {
        std::cout << "cannot create folder error:" << strerror(errno) << std::endl;
        exit(0);
    }
}
  return (str.back() == '/' ? str : (str +"/"));
}


/*------------------------------------------------------ kalman filtering   --------------------------------------------------------------------*/

cv::KalmanFilter initKalman(){

	//kalman filter param
	int nstate = 4;
	int nmeas = 2;
	int ncontr = 0;

	unsigned int type = CV_32F;

	cv::KalmanFilter kf(nstate, nmeas, ncontr, type);


	cv::setIdentity(kf.transitionMatrix);


	kf.measurementMatrix = cv::Mat::zeros(nmeas, nstate, type);
	kf.measurementMatrix.at<float>(0) = 1.0f;
	kf.measurementMatrix.at<float>(5) = 1.0f;

	kf.processNoiseCov.at<float>(0) = 1e-2;
	kf.processNoiseCov.at<float>(5) = 1e-2;
	kf.processNoiseCov.at<float>(10) = 5.0f;
	kf.processNoiseCov.at<float>(15) = 5.0f;

	// Measures Noise Covariance Matrix R
	cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));

	return kf;

}
cv::Rect predictKalmanRect(cv::KalmanFilter &kf, cv::Mat &States, int offset, double dT, int w, int h){

	// int offset = 5;//pix
	// >>>> Matrix A
	kf.transitionMatrix.at<float>(2) = dT;
	kf.transitionMatrix.at<float>(7) = dT;
	// <<<< Matrix A

	States = kf.predict();

	cv::Rect kalmanRect;

	kalmanRect.x = States.at<float>(0) -(w/2) +offset;//- kalmanRect.width / 2;
	kalmanRect.y = States.at<float>(1) -(h/2) +offset;//- kalmanRect.height / 2;

	kalmanRect.width = w;
	kalmanRect.height = h;

	return kalmanRect;
}
