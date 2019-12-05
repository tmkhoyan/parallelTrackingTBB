/*
 * Description:    Parallel tracker and feature extraction method initial release
 *
 * Author:          Tigran Mkhoyan
 * Email :          t.mkhoyan@tudelft.nl
 */


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

 makeValidDir(im_out_path);
  
  //put bacjk trainling
  im_out_path = im_out_path+"/";

  std::string videosrcNameR = im_out_path + "CamR_" + getTime() + "_.mkv "; //add trailing white space for gstreamer
  std::string videosrcNameL = im_out_path + "CamL_" + getTime() + "_.mkv "; //add trailing white space for gstreamer


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


	cv::VideoCapture cap1;
	cv::VideoCapture cap2;

	cv::Mat openCvImage1;
	cv::Mat openCvImage2;

		//needed for par
	cv::Mat openCvImage1_;
	cv::Mat openCvImage2_;


	/*--------------------------------setup kalman filter--------------------------------------*/

	vector<size_t> kalmanIDS(NTRACKERS);   iota(kalmanIDS.begin(), kalmanIDS.end(), 0);

  for(auto p: kalmanIDS)
  	cout << p << endl;


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

		cap1 >> openCvImage1; openCvImage1=rotateImg90(openCvImage1); openCvImage1.copyTo(openCvImage1_);
		cap2 >> openCvImage2; openCvImage2=rotateImg90(openCvImage2); openCvImage2.copyTo(openCvImage2_);

		cv::namedWindow(camIDs[0],cv::WINDOW_NORMAL);
		cv::namedWindow(camIDs[1],cv::WINDOW_NORMAL);

		//show
		cv::imshow(camIDs[0],openCvImage1);
		cv::imshow(camIDs[1],openCvImage2);


		//move windows side by side
		cv::moveWindow(camIDs[0],0,0);
		cv::moveWindow(camIDs[1],openCvImage1.rows,0);
		cv::waitKey(10);

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

			std::vector<bool> OKL(NTRACKERS,false);
			std::vector<bool> OKR(NTRACKERS,false);

			// ok condition is a valid keypoint
			// studip way no time to fix
		     for(int k = 0; k<keypoints1.size(); k++){
		     	OKL[k] = true;
		     	cout << "found edcdwcwc" << endl;
		     }

		     for(int k = 0; k<keypoints2.size(); k++){
		     	OKR[k] = true;
		     	cout << "found edcdwcw2c" << endl;

		     }

			/* ##---------------------------------------------- kalman filter  -----------------------------------------------------*/
			double prevTick = Tick;
			Tick = (double) cv::getTickCount();

			double dT_EKF = (Tick - prevTick) / cv::getTickFrequency();
			dT_EKF *= 8; // make the tracking les laggy

/* ##---------------------------------------------- draw  filter  -----------------------------------------------------*/

cout << "kalman dTF: " << dT_EKF << endl;
	for(auto k: kalmanIDS){
				if(foundKalmanL[k]){
					//predicts and draws kalman filtered states       
					cv::Rect kalmanRectL = predictKalmanRect(KalmanFilterInstancesL[k], KalmanStateMatricesL[k], 5, dT_EKF,tracker_w,tracker_h); 

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
				}
			}
	
		for(auto k: kalmanIDS){
				if(foundKalmanR[k]){
					//predicts and draws kalman filtered states       
					cv::Rect kalmanRectR = predictKalmanRect(KalmanFilterInstancesR[k], KalmanStateMatricesR[k], 5, dT_EKF,tracker_w,tracker_h); 

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
				}
			}


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

			/* ##---------------------------------------------- triangulate  -----------------------------------------------------*/
			//if we have correspondence and at least one common point
		
			if(keypoints1.size() and keypoints2.size()){
				cv::Mat p3D =  triangulateUndistPoints(pxyL, pxyR,camParams);

				//push 3d coords
				PxyL.push_back(pxyL[0]);
				PxyR.push_back(pxyR[0]);
				Pxyz.push_back(p3D.at<cv::Point3d>(0,0));

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


	//write frames
	/* ---------------------------------------------- save frames -----------------------------------------------------*/

	auto fps = cap1.get(cv::CAP_PROP_FPS);
	// video.open(videosrcNameLL, 0,(double)20, cv::Size(1088,600), true);
	videoL.open(videosrcNameL, 0,fps, recImgL.front().size(), true);
	videoR.open(videosrcNameR, 0,fps, recImgR.front().size(), true);

	std::cout << "nfranmes: " << recImgL.size();
	std::cout<< " writing images..." << std::endl;

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