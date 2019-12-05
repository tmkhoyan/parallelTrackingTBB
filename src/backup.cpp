/*
 * Description:     This code will run blob in parallel op parts of image and use basler camera image to traingulate
 *
 * Author:          Tigran Mkhoyan
 * Email :          t.mkhoyan@tudelft.nl
 */


// #include "../include/triangulate_rt.h"
#include "include/triangulate_rt.h"
#include "include/parallel_detector.h"  // parallel dedector is defined in parallel_detector

#define TOGGLE_PAUSE // uncomment to use basler cam
#define R_AVE_ALPHA .8 // running average exp for about 100 values
#define TBB_THREAD_SIZE 4 //size of threads and matrix chunks
#define SAVE_FRAMES  //size of threads and matrix chunks



int NTRACKERS=2; // number of kalman filter instances
auto keyPressed = -1;
bool pausePressed = false;

std::vector<std::string> camIDs{"CAMLE","CAMTE"}; //left and right cam
cv::Scalar centrColor(0,255,0);//= //CV_RGB(255,0,255);
size_t centrSize =4;

bool KeyPressed(void);
std::string getTime(std::string format="%d-%m-%Y_%H-%M-%S");

template <typename T>
inline std::vector<cv::Point_<T>> key2point(const std::vector<cv::KeyPoint> &keypoint);

inline void rotateImg90_(cv::Mat &image){
	cv::transpose(image, image);
	cv::flip(image, image, +1);
  // return image;
}

// // create 8 threads and use TBB
//         auto start1 = high_resolution_clock::now();
        
//         cv::parallel_for_(cv::Range(0, 8), Parallel_process(img, out,keypoints_par, 9, 8));
        
//         auto stop1 = high_resolution_clock::now();


double Tick = 0;
bool foundL = false;
int notFoundCountL = 0;
bool foundR = false;
int notFoundCountR = 0;
int tracker_w = 80;  
int tracker_h = 80;
// #define DEBUG 1 //debug flag
#define MYFONT 5
// #define MAX_NMARKER 20
// #define MAX_NVAR (MAX_NMARKER*3*2+1) // reserve a buffer of size 100 thats 6 parameters per marker.


// containers to save video

std::vector< cv::Mat> recImgL;
std::vector< cv::Mat> recImgR;


cv::VideoWriter videoL;
cv::VideoWriter videoR;

  
/* ## ----------------------------------------------  main -----------------------------------------------------*/

int main(int argc, char const *argv[]){

	/* ##----------------------------------------------  setup -----------------------------------------------------*/
	// std::vector <std::string> lines;
	// lines.reserve(1000000);
	std::string SDate = getTime();

	double FPS;
	int waitFPS = (argc>1)? atoi(argv[1]) : 200; //msec
	uint32_t NimagesTograb 	= (argc>2? atoi(argv[2]): 1000000);
	std::string flname    = (argc>3)? argv[3] : "../calib_cvtest/cam_stereo_pts.yml"; //stereo

	std::string cap_id1   = (argc>4)? argv[4] : "CAMLE.mkv"; //stereo
	std::string cap_id2   = (argc>5)? argv[5] : "CAMTE.mkv"; //stereo
     std::string im_out_path = (argc>6)? argv[6] : "data_out/";
  	int nstep = (argc>7)? atoi(argv[7]) : 1;

 makeValidDir(im_out_path);
  
  //put bacjk trainling
  im_out_path = im_out_path+"/";

  std::string videosrcNameR = im_out_path + "CamR_" + getTime() + "_.mkv "; //add trailing white space for gstreamer
  std::string videosrcNameL = im_out_path + "CamL_" + getTime() + "_.mkv "; //add trailing white space for gstreamer



	// std::string flname    = (argc>5)? argv[5] : "../calib_cvtest/cam_stereo_pts.yml"; //stereo

	// std::cout << "fl_l"       << "\t" << flname_l << std::endl;

	fs.open(flname     , cv::FileStorage::READ);
	fs_o.open(im_out_path+ "3Dpts.yml"  , cv::FileStorage::WRITE);

	/*--------------------------------storage containers--------------------------------------*/

	stereoParams camParams; // structure stereoparameters;
	// bool isSet; 

	/*--------------------------------get stereo parameters--------------------------------------*/
	//load settings from file
	bool isRead = camParams.loadParams(fs);
	
	//calculate projection matrices
	bool isSet = camParams.configureParams();

	if(isSet){
		cout << "Extrinsic parameters are set" << endl;
		cout << "Projection matrix Cam1:" << camParams.P1 << endl;
		cout << "Projection matrix Cam2:" << camParams.P2 << endl << endl;
	}

	std::vector<cv::Point2d> PxyL; PxyL.reserve(NimagesTograb);
	std::vector<cv::Point2d> PxyR; PxyL.reserve(NimagesTograb);
	std::vector<cv::Point3d> Pxyz; Pxyz.reserve(NimagesTograb);

	std::vector<cv::Point2d> PxyL_kf; PxyL_kf.reserve(NimagesTograb);
	std::vector<cv::Point2d> PxyR_kf; PxyL_kf.reserve(NimagesTograb);
	std::vector<cv::Point3d> Pxyz_kf; Pxyz_kf.reserve(NimagesTograb);

     std::vector<cv::Point3d> PlostCorrespondence; PlostCorrespondence.reserve(NimagesTograb);

	std::vector<double> nstepTimeOffset 		;  nstepTimeOffset.reserve(NimagesTograb); 			
	std::vector<double> kalmanPredictTimeOffset	;  kalmanPredictTimeOffset.reserve(NimagesTograb);

	cv::VideoCapture cap1;
	cv::VideoCapture cap2;

	cv::Mat openCvImage1;
	cv::Mat openCvImage2;

		//needed for par
	cv::Mat openCvImage1_;
	cv::Mat openCvImage2_;


	// 	std::string redisServerIP=REDIS_SERVER_IP;
	// // buffer to receive image
	// sw::redis::OptionalString buffCam1;
	// sw::redis::OptionalString buffCam2;

	// // key ids 
	// std::string keyId1=REDIS_KEY_ID1;
	// std::string keyId2=REDIS_KEY_ID2;

	// size_t rows = IMG_ROW;
	// size_t cols = IMG_COL;

	/*--------------------------------setup kalman filter--------------------------------------*/
	// std::vector<bool> OKL (NTRACKERS, false);
	// std::vector<bool> OKR (NTRACKERS, false);

	vector<size_t> kalmanIDS(NTRACKERS);   iota(kalmanIDS.begin(), kalmanIDS.end(), 0);

  for(auto p: kalmanIDS)
  	cout << p << endl;

  int aa ; cin >> aa;

	// //init kalman filters
	std::vector <cv::KalmanFilter> KalmanFilterInstancesL(NTRACKERS); 
	std::vector <cv::Mat> KalmanStateMatricesL(NTRACKERS,cv::Mat(NSTATE, 1, type)); 
	std::vector <cv::Mat> KalmanMeasMatricesL(NTRACKERS,cv::Mat(NMEAS, 1, type)); 

	std::vector<bool> foundKalmanL (NTRACKERS, false);
	std::vector<int>  lostCountKalmanL(NTRACKERS, 0);

		std::vector<bool> foundKalmanR (NTRACKERS, false);
	std::vector<int>  lostCountKalmanR(NTRACKERS, 0);

	//matrices must be properly initialized!
	int k = 0;
	for(auto & KF : KalmanFilterInstancesL){
		KF = initKalman();
		KalmanStateMatricesL[k] = cv::Mat(NSTATE, 1, type);
		KalmanMeasMatricesL[k]  = cv::Mat(NMEAS, 1, type);
		// KalmanFilterInstances.push_back
		k++;
	}

	std::vector <cv::KalmanFilter> KalmanFilterInstancesR(NTRACKERS); 
	std::vector <cv::Mat> KalmanStateMatricesR(NTRACKERS,cv::Mat(NSTATE, 1, type)); 
	std::vector <cv::Mat> KalmanMeasMatricesR(NTRACKERS,cv::Mat(NMEAS, 1, type)); 
	k = 0;
	for(auto & KF : KalmanFilterInstancesR){
		KF = initKalman();
		KalmanStateMatricesR[k] = cv::Mat(NSTATE, 1, type);
		KalmanMeasMatricesR[k]  = cv::Mat(NMEAS, 1, type);
		// KalmanFilterInstances.push_back
		k++;
	}

	/*--------------------------------setup pylon--------------------------------------*/

	try{
		cap1.open(cap_id1);
		cap2.open(cap_id2);

		double fps = cap1.get(cv::CAP_PROP_FPS);
		int nframes = cap1.get(cv::CAP_PROP_FRAME_COUNT);
		recImgL.reserve(nframes);
		recImgR.reserve(nframes);


		if(!cap1.isOpened() or !cap2.isOpened()){
			cout << "Error opening video stream or file" << endl;
			exitCode= 1;
		}

		/*------------------- convert to opencv and draw---------------------------*/
		//get image from vide0
		// cv::Mat openCvImage1;
		// cv::Mat openCvImage2;

		// //needed for par
		// cv::Mat openCvImage1_;
		// cv::Mat openCvImage2_;

		cap1 >> openCvImage1; openCvImage1=rotateImg90(openCvImage1); openCvImage1.copyTo(openCvImage1_);
		cap2 >> openCvImage2; openCvImage2=rotateImg90(openCvImage2); openCvImage2.copyTo(openCvImage2_);

		cv::namedWindow(camIDs[0],cv::WINDOW_NORMAL);
		cv::namedWindow(camIDs[1],cv::WINDOW_NORMAL);

		//show
		cv::imshow(camIDs[0],openCvImage1);
		cv::imshow(camIDs[1],openCvImage2);

		//resize if needed
    	// cv::resizeWindow(camIDs[0],img_W, img_H);
    	// cv::resizeWindow(camIDs[1],img_W, img_H);

		//move windows side by side
		cv::moveWindow(camIDs[0],0,0);
		cv::moveWindow(camIDs[1],openCvImage1.rows,0);
		cv::waitKey(1000);

// By Color : [ Note : This feature appears to be broken. I checked the code, and it appears to have a logical error ]  First you need to set filterByColor = 1. Set blobColor = 0 to select darker blobs, and blobColor = 255 for lighter blobs. 
		/*------------------- setup feature detector---------------------------*/
		// std::vector<cv::KeyPoint> keypoints1;
		// std::vector<cv::KeyPoint> keypoints2;

		//non parallel
		// cv::SimpleBlobDetector::Params = 
		// cv::Ptr<cv::SimpleBlobDetector> detector1 = cv::SimpleBlobDetector::create(setupBlobDetector());
		// cv::Ptr<cv::SimpleBlobDetector> detector2 = cv::SimpleBlobDetector::create(setupBlobDetector());
		// detector1->detect(openCvImage1, keypoints1);
		// detector2->detect(openCvImage2, keypoints2);



		double accumulator=0.0;
		double total_read_time(0.0);
		int32_t frame_count(0);
		high_resolution_clock::time_point tstart(high_resolution_clock::now()); //total timing
		double fpsID1 	= FPS;
		double fpsID2	= FPS;

		int loopcounter= 0 ;
		int cnt_frames;

		openCvImage1_ = cv::Mat::zeros(openCvImage1.size(), CV_8UC3);
		openCvImage2_ = cv::Mat::zeros(openCvImage2.size(), CV_8UC3);

		/* ##---------------------------------------------- grab images -----------------------------------------------------*/
		for (; frame_count < MAX_FRAME_COUNT; ++frame_count) {
			
			std::vector<cv::KeyPoint> keypoints1; keypoints1.reserve(100);
			std::vector<cv::KeyPoint> keypoints2; keypoints1.reserve(100);


			//retrieves camera image 
			// cap1 >> openCvImage1; openCvImage1=rotateImg90(openCvImage1); openCvImage1.copyTo(openCvImage1_); 
			// cap2 >> openCvImage2; openCvImage2=rotateImg90(openCvImage2); openCvImage2.copyTo(openCvImage2_);	
			// cap1 >> openCvImage1; openCvImage1=rotateImg90(openCvImage1); //openCvImage1.copyTo(openCvImage1_); 
			// cap2 >> openCvImage2; openCvImage2=rotateImg90(openCvImage2); //openCvImage2.copyTo(openCvImage2_);

			high_resolution_clock::time_point t1(high_resolution_clock::now()); cap1 >> openCvImage1; openCvImage1=rotateImg90(openCvImage1); 
			high_resolution_clock::time_point t2(high_resolution_clock::now()); cap2 >> openCvImage2; openCvImage2=rotateImg90(openCvImage2); 

			double dt_us_offset(static_cast<double>(duration_cast<microseconds>(t2 - t1).count()));
				cout << "cam2 receive latency: " << dt_us_offset << " microsec" <<endl;



			if(openCvImage1.empty() or openCvImage2.empty())
				break;

				// total_read_time += dt_us;

			/* ##---------------------------------------------- detect points  -----------------------------------------------------*/
			//define points to store keep scope to for loop
			vector<cv::Point2d> pxyL;
			vector<cv::Point2d> pxyR;

			//non parallel
			// //detect
			// detector1->detect(openCvImage1, keypoints1);
			// detector2->detect(openCvImage2, keypoints2);
			// //draw
			// cv::drawKeypoints(openCvImage1, keypoints1, openCvImage1, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
			// cv::drawKeypoints(openCvImage2, keypoints2, openCvImage2, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
			// // cv::drawKeypoints(openCvImage2, keypoints2, openCvImage2, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        	 	auto td1 = high_resolution_clock::now();
       		 cv::parallel_for_(cv::Range(0, TBB_THREAD_SIZE), Par_Detector(openCvImage1, keypoints1, TBB_THREAD_SIZE));
       		 cv::parallel_for_(cv::Range(0, TBB_THREAD_SIZE), Par_Detector(openCvImage2, keypoints2, TBB_THREAD_SIZE));
       		 	auto td2 = high_resolution_clock::now();
        	 	auto tdiff = (duration_cast<microseconds>(td2 - td1)).count()/1000;
        		//get running average
			 accumulator = (R_AVE_ALPHA * tdiff) + (1.0 - R_AVE_ALPHA) * accumulator;
        		cout << "Detector parallel Time:      " <<  tdiff << " ms," << "r.average :" << accumulator << " ms" << endl;
        		//draw running average


			// // create 8 threads and use TBB
			// auto td1 = high_resolution_clock::now();
			// cv::parallel_for_(cv::Range(0, 8), Par_Detector(openCvImage1, openCvImage1_, keypoints1, 8));
			// cv::parallel_for_(cv::Range(0, 8), Par_Detector(openCvImage2, openCvImage2_, keypoints2, 8));
			// auto td2 = high_resolution_clock::now();
			// auto tdiff = (duration_cast<microseconds>(td2 - td1)).count()/1000;
   //      	 	// auto time_taken2 = duration2.count()/1000;
			// cout << "Detector parallel Time: " <<  tdiff << "ms" << endl;

			std::cout << keypoints1.size() << std::endl;
			std::cout << keypoints2.size() << std::endl;

			//draw center
			// if(keypoints1.size() and keypoints2.size()){
			// circle(openCvImage1,cv::Point2d(keypoints1[0].pt), size, color, -1);//, 8, 0 );
			// circle(openCvImage2,cv::Point2d(keypoints2[0].pt), size, color, -1);//, 8, 0 );
			// }
 		// pause and use kalman

			std::vector<bool> OKL(NTRACKERS,false);
			std::vector<bool> OKR(NTRACKERS,false);
 		 
 		 // update new position from drawkeypoints apply kalman prediction




   
 		 if(!(frame_count % nstep)){
 		 	high_resolution_clock::time_point t1_pausematch(high_resolution_clock::now()); 
			cv::drawKeypoints(openCvImage1, keypoints1, openCvImage1, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
			cv::drawKeypoints(openCvImage2, keypoints2, openCvImage2, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

			//push to pint vector;
			if(keypoints1.size())
				pxyL.push_back(cv::Point2d(keypoints1[0].pt));
				// std::cout << key2point<double>(keypoints1) << std::endl;
			if(keypoints2.size()){
				pxyR.push_back(cv::Point2d(keypoints2[0].pt));
				// std::cout << key2point<double>(keypoints2) << std::endl;
			}
			high_resolution_clock::time_point t2_pausematch(high_resolution_clock::now()); 
		     double t2_pausematch_offset(static_cast<double>(duration_cast<microseconds>(t2_pausematch - t1_pausematch).count()));
			nstepTimeOffset.push_back(t2_pausematch_offset);
			} else{
			nstepTimeOffset.push_back(0.0 / 0.0);
			}

			// ok condition is a valid keypoint
			// studip way no time to fix
		     for(int k = 0; k<keypoints1.size(); k++){
		     	OKL[k] = true;
		     	// cout << "found edcdwcwc" << endl;
		     }

		     for(int k = 0; k<keypoints2.size(); k++){
		     	OKR[k] = true;
		     	// cout << "found edcdwcw2c" << endl;

		     }


			/* ##---------------------------------------------- kalman filter  -----------------------------------------------------*/
			double prevTick = Tick;
			Tick = (double) cv::getTickCount();
			// dt is now the one taken from imagelist parameters
			// double dT = dTs[frameCounter]*2;
			// double dT = dTs[frameCounter];
			// dT *= 8;
			double dT_EKF = (Tick - prevTick) / cv::getTickFrequency();
			dT_EKF *= 8; // make the tracking les laggy


		//add this part to othe loop
			// for(int k =0; k < NTRACKERS; k++){ 
			// for(auto k: kalmanIDS){
			// 	if(foundKalman[k]){
			// 		//predicts and draws kalman filtered states       
			// 		cv::Rect kalmanRectL = predictKalmanRect(KalmanFilterInstancesL[k], KalmanStateMatricesL[k], 5, dT_EKF,tracker_w,tracker_h); 
			// 		cv::Rect kalmanRectR = predictKalmanRect(KalmanFilterInstancesR[k], KalmanStateMatricesR[k], 5, dT_EKF,tracker_w,tracker_h); 


			// 		cv::Point centerL;
			// 		center.x = KalmanStateMatricesL[k].at<float>(0);
			// 		center.y = KalmanStateMatricesL[k].at<float>(1);

			// 		cv::Point centerR;
			// 		center.x = KalmanStateMatricesR[k].at<float>(0);
			// 		center.y = KalmanStateMatricesR[k].at<float>(1);

			// 		//  cam 1
			// 		cv::circle(openCvImage1, centerL, 2, CV_RGB(255,0,0), -1);
			// 		cv::rectangle(openCvImage1, kalmanRectL, CV_RGB(255,0,0), 2);

			// 		cv::Point bboxNumberL = Point(kalmanRectL.tl().x+5,kalmanRectL.tl().y+15); //bottom-right with slight offset
			// 		cv::Rect rectNumberL(Point(kalmanRectL.tl().x+20,kalmanRectL.tl().y+20),kalmanRectL.tl()); 
			// 		//   // std::cout << OK[k] << endl;
			// 		//   cv::circle(frame,centroidBoxes[k],3,colormap[k],2,1);
			// 		rectangle(openCvImage1, rectNumberL, Scalar( 0, 0, 255 ), 2, 1 );
			// 		putText(openCvImage1, std::to_string(k) , bboxNumberL , MYFONT, 0.75, Scalar( 0, 0, 255 ),2);

			// 		//  cam 2
			// 		cv::circle(openCvImage2, centerR, 2, CV_RGB(255,0,0), -1);
			// 		cv::rectangle(openCvImage2, kalmanRectR, CV_RGB(255,0,0), 2);

			// 		cv::Point bboxNumberR = Point(kalmanRectR.tl().x+5,kalmanRectR.tl().y+15); //bottom-right with slight offset
			// 		cv::Rect rectNumberR(Point(kalmanRectR.tl().x+20,kalmanRectR.tl().y+20),kalmanRectR.tl()); 
			// 		//   // std::cout << OK[k] << endl;
			// 		//   cv::circle(frame,centroidBoxes[k],3,colormap[k],2,1);
			// 		rectangle(openCvImage2, rectNumberR, Scalar( 0, 0, 255 ), 2, 1 );
			// 		putText(openCvImage2, std::to_string(k) , bboxNumberR , MYFONT, 0.75, Scalar( 0, 0, 255 ),2);

			// 		//   rectangle(frame, bboxes[k], Scalar( 255, 0, 0 ), 2, 1 );       
			// 	}
			// }

/* ##---------------------------------------------- draw  filter  -----------------------------------------------------*/

cout << "kalman dTF: " << dT_EKF << endl;

// calculate calman average predict operation
double dt_kf_offset_kf = 0;
	for(auto k: kalmanIDS){
				double dt_us_offset_ = 0;
				if(foundKalmanL[k]){

			// double dt_us_offset(static_cast<double>(duration_cast<microseconds>(t2 - t1).count()));
				// cout << "cam2 receive latency: " << dt_us_offset << " microsec" <<endl;

					high_resolution_clock::time_point t1(high_resolution_clock::now()); 
					//predicts and draws kalman filtered states       
					cv::Rect kalmanRectL = predictKalmanRect(KalmanFilterInstancesL[k], KalmanStateMatricesL[k], 5, dT_EKF,tracker_w,tracker_h); 
					high_resolution_clock::time_point t2(high_resolution_clock::now()); 
					dt_us_offset_ = (static_cast<double>(duration_cast<microseconds>(t2 - t1).count()));

					cv::Point center;
					center.x = KalmanStateMatricesL[k].at<float>(0);
					center.y = KalmanStateMatricesL[k].at<float>(1);
					//  cam 1
					cv::circle(openCvImage1, center, 2, CV_RGB(255,0,0), -1);
					cv::rectangle(openCvImage1, kalmanRectL, CV_RGB(255,0,0), 2);

					cv::Point bboxNumber = cv::Point(kalmanRectL.tl().x+5,kalmanRectL.tl().y+15); //bottom-right with slight offset
					cv::Rect rectNumber(cv::Point(kalmanRectL.tl().x+20,kalmanRectL.tl().y+20),kalmanRectL.tl()); 
					//   // std::cout << OK[k] << endl;
					//   cv::circle(frame,centroidBoxes[k],3,colormap[k],2,1);
					cv::rectangle(openCvImage1, rectNumber, cv::Scalar( 0, 0, 255 ), 2, 1 );
					cv::putText(openCvImage1, std::to_string(k) , bboxNumber , MYFONT, 0.75, cv::Scalar( 0, 0, 255 ),2);
					//   rectangle(frame, bboxes[k], Scalar( 255, 0, 0 ), 2, 1 );       
					dt_us_offset_ += dt_us_offset_;
				}
			dt_kf_offset_kf += dt_us_offset_;

			}
 
		for(auto k: kalmanIDS){
				double dt_us_offset_ = 0;
				if(foundKalmanR[k]){
					//predicts and draws kalman filtered states 
					high_resolution_clock::time_point t1(high_resolution_clock::now());       
					cv::Rect kalmanRectR = predictKalmanRect(KalmanFilterInstancesR[k], KalmanStateMatricesR[k], 5, dT_EKF,tracker_w,tracker_h); 
					high_resolution_clock::time_point t2(high_resolution_clock::now()); 
					dt_us_offset_ = (static_cast<double>(duration_cast<microseconds>(t2 - t1).count()));


					cv::Point center;
					center.x = KalmanStateMatricesR[k].at<float>(0);
					center.y = KalmanStateMatricesR[k].at<float>(1);
					//  cam 1
					cv::circle(openCvImage2, center, 2, CV_RGB(255,0,0), -1);
					cv::rectangle(openCvImage2, kalmanRectR, CV_RGB(255,0,0), 2);

					cv::Point bboxNumber = cv::Point(kalmanRectR.tl().x+5,kalmanRectR.tl().y+15); //bottom-right with slight offset
					cv::Rect rectNumber(cv::Point(kalmanRectR.tl().x+20,kalmanRectR.tl().y+20),kalmanRectR.tl()); 
					//   // std::cout << OK[k] << endl;
					//   cv::circle(frame,centroidBoxes[k],3,colormap[k],2,1);
					cv::rectangle(openCvImage2, rectNumber, cv::Scalar( 0, 0, 255 ), 2, 1 );
					cv::putText(openCvImage2, std::to_string(k) , bboxNumber , MYFONT, 0.75, cv::Scalar( 0, 0, 255 ),2);
					//   rectangle(frame, bboxes[k], Scalar( 255, 0, 0 ), 2, 1 );
					dt_us_offset_ += dt_us_offset_;
				}
				dt_kf_offset_kf += dt_us_offset_;
			}

				kalmanPredictTimeOffset.push_back(dt_kf_offset_kf);

				cout << "kalman predict latency: " << dt_kf_offset_kf << " microsec" <<endl;
				cout << "nstep  calc    latency: " << nstepTimeOffset[frame_count] << " microsec" <<endl;

			}

			// ------------------

			// //if some nodes lost draw home many lost else show locked number
			// if(std::accumulate(OK.begin(), OK.end(),0)<NTRACKERS){
			// 	putText(frame_, "Locked node(s) [ " + ssnodesYes.str() +" ]", Point(200,80), MYFONT, 0.75, Scalar(0,255,0),2);
			// 	putText(frame_, "Lost node(s)   [ " + ssnodesNo.str() + " ]", Point(200,100), MYFONT, 0.75, Scalar(0,0,255),2);
			// }
			// else{ 
			// 	putText(frame_, "Locked node(s) [ " + ssnodesYes.str() +" ]", Point(200,80), MYFONT, 0.75, Scalar(0,255,0),2);
			// }

			// ssnodesYes.str(std::string()); //clear stringstream
			// ssnodesNo.str(std::string()); //clear stringstream

/* ##---------------------------------------------- kalman update   -----------------------------------------------------*/
			// k = 0; 

			for(auto k: kalmanIDS){

				notFoundCountL = lostCountKalmanL[k];
				foundL         =  foundKalmanL[k];

				if (!OKL[k])
				{
					// if no tracking available from kcf increment count of lost trackings for each tracker 
					notFoundCountL++; 
					cout << "notFoundCountL:" << notFoundCountL << endl;
					if( notFoundCountL >= 100 )
					{
						foundL = false; //after 100> more lost counts set to false this forces the filter to reinitialize 
					}
				}
				else
				{
					//else update the measurement readings 
					notFoundCountL = 0;

					KalmanMeasMatricesL[k].at<float>(0) = pxyL[k].x ;//+ bboxes[LockedIndeces[0]].width / 2;
					KalmanMeasMatricesL[k].at<float>(1) = pxyL[k].y ;//+ bboxes[LockedIndeces[0]].height / 2;

					if (!foundL) // First detection!
					{
						// >>>> Initialization
						KalmanFilterInstancesL[k].errorCovPre.at<float>(0) = 1; // px
						KalmanFilterInstancesL[k].errorCovPre.at<float>(5) = 1; // px
						KalmanFilterInstancesL[k].errorCovPre.at<float>(10) = 1;
						KalmanFilterInstancesL[k].errorCovPre.at<float>(15) = 1;

						KalmanStateMatricesL[k].at<float>(0) = KalmanMeasMatricesL[k].at<float>(0);
						KalmanStateMatricesL[k].at<float>(1) = KalmanMeasMatricesL[k].at<float>(1);
						KalmanStateMatricesL[k].at<float>(2) = 0;
						KalmanStateMatricesL[k].at<float>(3) = 0;

						// <<<< Initialization

						KalmanFilterInstancesL[k].statePost = KalmanStateMatricesL[k];

						foundL = true;
					}
					else
						KalmanFilterInstancesL[k].correct(KalmanMeasMatricesL[k]); // Kalman Correction
				}

				lostCountKalmanL[k] = notFoundCountL;
				foundKalmanL[k]     = foundL; 
			}


			for(auto k: kalmanIDS){

				notFoundCountR = lostCountKalmanR[k];
				foundR         =  foundKalmanR[k];

				if (!OKR[k])
				{
					// if no tracking available from kcf increment count of lost trackings for each tracker 
					notFoundCountR++; 
					cout << "notFoundCountR:" << notFoundCountR << endl;
					if( notFoundCountR >= 100 )
					{
						foundR = false; //after 100> more lost counts set to false this forces the filter to reinitialize 
					}
				}
				else
				{
					//else update the measurement readings 
					notFoundCountR = 0;

					KalmanMeasMatricesR[k].at<float>(0) = pxyR[k].x ;//+ bboxes[LockedIndeces[0]].width / 2;
					KalmanMeasMatricesR[k].at<float>(1) = pxyR[k].y ;//+ bboxes[LockedIndeces[0]].height / 2;

					if (!foundR) // First detection!
					{
						// >>>> Initialization
						KalmanFilterInstancesR[k].errorCovPre.at<float>(0) = 1; // px
						KalmanFilterInstancesR[k].errorCovPre.at<float>(5) = 1; // px
						KalmanFilterInstancesR[k].errorCovPre.at<float>(10) = 1;
						KalmanFilterInstancesR[k].errorCovPre.at<float>(15) = 1;

						KalmanStateMatricesR[k].at<float>(0) = KalmanMeasMatricesR[k].at<float>(0);
						KalmanStateMatricesR[k].at<float>(1) = KalmanMeasMatricesR[k].at<float>(1);
						KalmanStateMatricesR[k].at<float>(2) = 0;
						KalmanStateMatricesR[k].at<float>(3) = 0;

						// <<<< Initialization

						KalmanFilterInstancesR[k].statePost = KalmanStateMatricesR[k];

						foundR = true;
					}
					else
						KalmanFilterInstancesR[k].correct(KalmanMeasMatricesR[k]); // Kalman Correction
				}

				lostCountKalmanR[k] = notFoundCountR;
				foundKalmanR[k]     = foundR; 
			}


						// push points
			 cv::Poipt pkfL = cv::Point2f(KalmanStateMatricesL[0].at<float>(0), KalmanStateMatricesL[0].at<float>(1));
			 cv::Poipt pkfR = cv::Point2f(KalmanStateMatricesR[0].at<float>(0), KalmanStateMatricesR[0].at<float>(1));

			 PxyL_kf.push_back(pkfL);
			 PxyR_kf.push_back(pkfR);

// --------------

// #ifdef WRITE_DATA
// 		// if(WRITE_DATA)

// 			for(int i=0; i<NTRACKERS; i++){
// 				//KCF-EKF
// 				cv::Mat kcf_tmp = (cv::Mat_<float>(1,3) <<centroidBoxes[i].x,centroidBoxes[i].y,0);
// 				cv::Mat ekf_tmp = (cv::Mat_<float>(1,3) <<KalmanStateMatrices[i].at<float>(0),KalmanStateMatrices[i].at<float>(1),0);

// 				KCF_xyzt[i].push_back(kcf_tmp); //or  kcf_tmp.copyTo(KCF_xyzt[i].row(recframe_cnt)); only if matrix is reallocated with NMAXFRAMES!
// 				EKF_xyzt[i].push_back(ekf_tmp); //or  EKF_xyzt[i].row(recframe_cnt).copyTo(ekf_tmp);

// 				//dbscan values if tracking with dbscan is run

// 			}
// 		// }
// #endif

			/* ##---------------------------------------------- triangulate  -----------------------------------------------------*/
			//if we have correspondence and at least one common point
		
			//  we can always triangulate kalman filter output
			cv::Mat p3Dkf =  triangulateUndistPoints(pkfL, pkfR,camParams);
			Pxyz_kf.push_back(p3D.at<cv::Point3d>(0,0));

				// cv::Mat p3D =  triangulateUndistPoints(pxyL, pxyR,camParams);




			if(keypoints1.size() and keypoints2.size()){
				cv::Mat p3D =  triangulateUndistPoints(pxyL, pxyR,camParams);
				// cv::Mat p3Dkf =  triangulateUndistPoints(pkfL, pkfR,camParams);

				//push 3d coords todo do all points
				PxyL.push_back(pxyL[0]);
				PxyR.push_back(pxyR[0]);
				Pxyz.push_back(p3D.at<cv::Point3d>(0,0));

				// PxyL.push_back(pxyL[0]);
				// PxyR.push_back(pxyR[0]);
				// Pxyz_kf.push_back(p3D.at<cv::Point3d>(0,0));

				//draw green circles
				circle(openCvImage1,cv::Point2d(keypoints1[0].pt), centrSize, centrColor, -1);//, 8, 0 );
				circle(openCvImage2,cv::Point2d(keypoints2[0].pt), centrSize, centrColor, -1);//, 8, 0 );

			//parse coordinates 
				std::stringstream ss;
				ss << "p[1](" << std::setprecision(2) << (p3D.at<cv::Point3d>(0,0)) << ")" ;			
				std::string p3D_str = eraseSpace(ss.str());
			// p3D_str.erase(remove_if(input.begin(), input.end(), isspace),input.end())
				std::cout << "3D coordinate: " << p3D_str << endl;

			//recoverpose
			//draw arrows 
			//put text
				cv::putText(openCvImage1, p3D_str, cv::Point(pxyL[0].x+10,pxyL[0].y+10), txtFontFace, txtFontScale, cv::Scalar::all(255), txtThickness, 1);
				cv::putText(openCvImage2, p3D_str, cv::Point(pxyR[0].x+10,pxyR[0].y+10), txtFontFace, txtFontScale, cv::Scalar::all(255), txtThickness, 1);
			/* ---------------------------------------------- show -----------------------------------------------------*/
				cv::imshow(camIDs[0],openCvImage1);
				cv::imshow(camIDs[1],openCvImage2);


#ifdef SAVE_FRAMES
				recImgL.push_back(openCvImage1);
				recImgR.push_back(openCvImage2);
#endif


			}else{
			// cv::putText(openCvImage1, eraseSpace("p[1]([0.1,-0.025,0.16])"), cv::Point2d(100,100), fontFace, fontScale, cv::Scalar(0,155,0), thickness, 1);
				
				cv::Mat p3D =  triangulateUndistPoints(pxyL, pxyR,camParams);

				PxyL.push_back(pxyL[0]);
				PxyR.push_back(pxyR[0]);
				Pxyz.push_back(p3D.at<cv::Point3d>(0,0));
				Pxyz.push_back(p3D.at<cv::Point3d>(0,0));


				cv::putText(openCvImage1, "no correspondence", cv::Point(100,20), txtFontFace, txtFontScale, cv::Scalar::all(255,165,0), txtThickness, 1);
				cv::putText(openCvImage1, p3D_str, cv::Point(pxyL[0].x+10,pxyL[0].y+10), txtFontFace, txtFontScale, cv::Scalar::all(255,165,0), txtThickness, 1);
				cv::putText(openCvImage2, p3D_str, cv::Point(pxyR[0].x+10,pxyR[0].y+10), txtFontFace, txtFontScale, cv::Scalar::all(255,165,0), txtThickness, 1);

				cv::imshow(camIDs[0],openCvImage1);
				cv::imshow(camIDs[1],openCvImage2);
			}

      if(keyPressed=='p' | keyPressed=='P'){ //P or p
        pausePressed = pausePressed==true ? false: true;
      } else if(keyPressed==13){
        break;
      } //enter

      if(pausePressed){
        while(cv::waitKey(50) !=' '){}//waiting for space input to move to next frame
      }

    keyPressed = (cv::waitKey(1));

		} // end for loop
	} //end try
	catch(const GenericException &e)
	{
		cerr << "An exception occurred." << e.GetDescription() << endl;
		exitCode = 1;
		// return 1;
	}

	// write points
	fs_o  << "Pts3D"  <<  point3Dmat(Pxyz);
	fs_o  << "PtsxyL" <<  point2Dmat(PxyR);
	fs_o  << "PtsxyR" <<  point2Dmat(PxyL);

	//write kalman filter
	fs_o  << "Pts3D_KF"  <<  point3Dmat(Pxyz_kf);
	fs_o  << "PtsxyL_KF" <<  point2Dmat(PxyL_kf);
	fs_o  << "PtsxyR-KF" <<  point2Dmat(PxyR_kf);

	fs_o  << "Pts3D_KF"  <<  point3Dmat(Pxyz_kf);
	fs_o  << "PtsxyL_KF" <<  point2Dmat(PxyL_kf);
	fs_o  << "PtsxyR-KF" <<  point2Dmat(PxyR_kf);

	fs_o << "boolPnomatch" << PlostCorrespondence;
	fs_o << "nstepTimeOffset" << nstepTimeOffset;
	fs_o << "kalmanPredictTimeOffset" << kalmanPredictTimeOffset;

	//      std::vector<cv::Point3d> PlostCorrespondence; PlostCorrespondence.reserve(NimagesTograb);

	// std::vector<double> nstepTimeOffset 		;  nstepTimeOffset.reserve(NimagesTograb); 			
	// std::vector<double> kalmanPredictTimeOffset	;  kalmanPredictTimeOffset.reserve(NimagesTograb);


	// //write kalman filter
	// Pxyz_kf
	// PxyL_kf
	// PxyR_kf


	// for(int i = 0; i<NTRACKERS; i++){
	// 	// cout << KCF_xyzt[i] << endl;
	// 	// cout << "bla" << i << endl;
	// 	// cout << EKF_xyzt[i] << endl;
	// 	fs_0 << "kf"+to_string(i) << KCF_xyzt[i];
	// 	// fs_0 << "ef"+to_string(i) << EKF_xyzt[i];
	// }

	//write frames
	/* ---------------------------------------------- save frames -----------------------------------------------------*/

	auto fps = cap1.get(cv::CAP_PROP_FPS);
	// video.open(videosrcNameLL, 0,(double)20, cv::Size(1088,600), true);
	videoL.open(videosrcNameL, 0,fps, recImgL.front().size(), true);
	videoR.open(videosrcNameR, 0,fps, recImgR.front().size(), true);

	std::cout << "nfranmes: " << recImgL.size();
	std::cout<< " writing images..." << std::endl;
	for(auto frame : recImgL){
	  if(!frame.empty()){
	    cv::imshow("aa", frame);
	    cv::waitKey(1);
	    videoL.write(frame);
	  }
	}
	for(auto frame : recImgR){
	  if(!frame.empty()){
	    cv::imshow("aa", frame);
	    cv::waitKey(1);
	    videoR.write(frame);
	  }
	}

	cap1.release();
	cap2.release();
	cv::destroyAllWindows();
	return exitCode;
}
/* ---------------------------------------------- exit programm -----------------------------------------------------*/
// // Rename the function
// template <typename T> // type CV_64FC1 points preferred
// cv::Mat triangulateUndistPoints_(std::vector<T> pxyL, std::vector<T> pxyR,	stereoParams camParams){

// 	//undistort
// 	vector<cv::Point2d>  pxyL_unidst;
// 	vector<cv::Point2d>  pxyR_unidst;

// 	cv::undistortPoints(pxyL, pxyL_unidst, camParams.K1, camParams.D1, cv::noArray(), camParams.K1);
// 	cv::undistortPoints(pxyR, pxyR_unidst, camParams.K2, camParams.D2, cv::noArray(), camParams.K2);

// 	// cout << pxyL << ",\t" << pxyL_unidst << endl;
// 	// cout << pxyR << ",\t" << pxyR_unidst << endl;

// 	// cout << camParams.P1 << endl;
// 	// cout << camParams.isSet << endl;

// 	//containr for triangulated points in homogenious coords
// 	//(4,pxyL.size(),CV_64F);

// 	cv::Mat p4D; cv::triangulatePoints(camParams.P1, camParams.P2, pxyL_unidst, pxyR_unidst, p4D);
// 	// cout << P4D << endl; 
// 	// cv::Mat p4D; cv::triangulatePoints(camParams.P1, camParams.P2, pxyL, pxyR, p4D);
// 	cv::Mat pt3d; convertPointsFromHomogeneous(cv::Mat(p4D.t()).reshape(4, 1),pt3d); 
// 	// Mat pt3d; convertPointsFromHomogeneous(p4D.t(),pt3d);
// 	return pt3d;

// }