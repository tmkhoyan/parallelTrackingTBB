
#include "main.h"
#include "queue.h"
#include "worker.h"

#include <thread>
#include <vector>
#define INIP "127.0.0.1"

#define ENABLE_UDP
#define BUFLEN SIZEVAR*NVAR


// ============================================================================ 

// int FPS = 30;??
int x = 0;
static const size_t c_maxCamerasToUse = 2;


int camWidth = 1088;
int camHeight = 600;
int exitCode = 0;

std::string ID1  = "CAMTE";
std::string ID2  = "CAMLE";


void diep(char *s);
void write(std::string line);
bool KeyPressed(void);
std::string getTime(std::string format="%d-%m-%Y_%H-%M-%S");


int main(int argc, char const *argv[])
{

	/*---------------------------setup udp----------------------------------*/
#ifdef ENABLE_UDP
	int convertedbuffer[NVAR];
	struct sockaddr_in si_me, si_other;
	struct sockaddr_storage serverStorage;
	fd_set readfdsTemp;

	struct timeval tvu;
	//  gettimeofday(&tv, NULL);
	//dont wait
	tvu.tv_sec = 0;
	tvu.tv_usec = 1;

	int s, i, slen=sizeof(si_other);
	char buf[BUFLEN];

	if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
		diep((char*)"socket");

	memset((char *) &si_me, 0, sizeof(si_me));
	si_me.sin_family = AF_INET;
	si_me.sin_port = htons(PORT);
	//si_me.sin_addr.s_addr = inet_addr("10.0.0.115");
	si_me.sin_addr.s_addr = inet_addr("192.168.0.115");
	if (bind(s, (struct sockaddr *)&si_me, sizeof(si_me))==-1)
		diep((char *)"bind");


	fcntl(s, F_SETFL, O_NONBLOCK); 
#endif
	/*---------------------------setup udp----------------------------------*/

	std::vector <std::string> lines;
	lines.reserve(1000000);
	std::string SDate = getTime();

	double FPS = (argc>1)? atoi(argv[1]) : 50;
	uint32_t NimagesTograb 	= (argc>2? atoi(argv[2]): 1000000);
	double TTL_WAIT 	= (argc>3)? atoi(argv[3]) : 10;
	const int Nmeas 	= (argc>4)? atoi(argv[4]) : 3; //3 measuremnts

	std::string txtflname 	= (argc>5)? txtflname.append(argv[5]) :  "";
	txtflname += SDate;
	std::string saveopts 	= (argc>6)? argv[6] :  "images";

	std::ofstream outFile("imagelist[" + txtflname + "].txt");
	//	std::ofstream outFileIter("masterfile_" + txtflname + ".txt");

	//	std::vector< std::string > vecFlname;
	//	std::vector< std::string > vecLines;


	// And fetch the actual values, so we can create our video correctly
	int32_t frame_width(static_cast<int32_t>(camWidth));
	int32_t frame_height(static_cast<int32_t>(camHeight));
	double video_fps(std::max(10.0, FPS)); // Some default in case it's 0

	std::cout << "Capturing images (" << frame_width << "x" << frame_height
		<< ") at " << video_fps << " FPS." << std::endl;

	// The synchronized queues, one per video source/storage worker pair
	std::vector<frame_queue> queue(2);

	// Let's create our storage workers -- let's have two, to simulate your scenario
	// and to keep it interesting, have each one write a different format
	std::vector <storage_worker> storage;

	/* ------------------------- start up queue ---------------------------------------*/

	storage.emplace_back(std::ref(queue[0]), 0
			, std::string("appsrc ! videoconvert ! omxh264enc preset-level=3 bitrate=10000000 ! matroskamux ! filesink location=" + ID1 + "[" +getTime() +"].mkv ")
			, CV_FOURCC('X', '2', '6', '4') //doesnt do anything with appsrc file sink
			, video_fps
			, cv::Size(frame_width, frame_height)
			, true);

	storage.emplace_back(std::ref(queue[1]), 1
			, std::string("appsrc ! videoconvert ! omxh264enc preset-level=3 bitrate=10000000 ! matroskamux ! filesink location=" + ID2 + "[" +getTime() +"].mkv ")
			, CV_FOURCC('I', 'Y', 'U', 'V') //doesnt do anything with appsrc file sink
			, video_fps
			, cv::Size(frame_width, frame_height)
			, true);

	// And start the worker threads for each storage worker
	std::vector<std::thread> storage_thread;

	/* ----------------- setup GPIO ----------------------------------------------*/ 

	cout << "Testing the GPIO Pins" << endl;

	//define pin
	jetsonTX2GPIONumber readPin = gpio397 ;     // input

	gpioExport(readPin) ;
	//set direction
	gpioSetDirection(readPin,inputPin) ; 


	//gpio setup
	unsigned int clockval = low; 
	int mesCounter = 0;
	// proceed to grabbing
	auto start = std::chrono::high_resolution_clock::now();
	auto startIter = std::chrono::high_resolution_clock::now();
	/* -----------------------------------------------------------------------------*/ 	

	/* -------------------------- start pylon ----------------------------------------------------- */


	// Pylon::PylonAutoInitTerm autoInitTerm;
	PylonInitialize();
	// try 
	// Get the transport layer factory.
	try
	{
		CTlFactory& tlFactory = CTlFactory::GetInstance();

		// Get all attached devices and exit application if no device is found.
		DeviceInfoList_t devices;
		if ( tlFactory.EnumerateDevices(devices) == 0 )
		{
			throw RUNTIME_EXCEPTION( "No camera present.");
		}

		/*-------------------Create cameras and setup---------------------------*/
		CBaslerGigEInstantCameraArray cameras( min( devices.size(), c_maxCamerasToUse));

		// Create and attach all Pylon Devices.
		for ( size_t i = 0; i < cameras.GetSize(); ++i)
		{
			cameras[ i ].Attach( tlFactory.CreateDevice( devices[ i ]));
			// Print the model name of the camera.
			cout << "Using device " << cameras[ i ].GetDeviceInfo().GetModelName() << endl;
			// Print the model name of the camera.
			cout << "Using device "
				<< cameras[ i ].GetDeviceInfo().GetUserDefinedName() 
				<< " (model: " << cameras[ i ].GetDeviceInfo().GetModelName() 
				<< ")" << endl;

		}

		ID1 = (std::string)cameras[0].GetDeviceInfo().GetUserDefinedName();
		ID2 = (std::string)cameras[1].GetDeviceInfo().GetUserDefinedName();

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
		/*------------------------------------------------------------------*/

		// grabbing
		// This smart pointer will receive the grab result data.  
		CImageFormatConverter formatConverter;
		formatConverter.OutputPixelFormat= PixelType_BGR8packed;
		//	PixelType_RGB8packed
		CPylonImage pylonImage;   
		CGrabResultPtr ptrGrabResultCam1;
		CGrabResultPtr ptrGrabResultCam2;
		int loopcounter= 0 ;
		int cnt_frames;


		/*-------------------Obtain few initial frames---------------------------*/

		//get one image before starting thread
		cv::Mat openCvImage1;
		cv::Mat openCvImage2;
		//get a 10 frames
		for(int i; i< 10 ; i++){

			cameras[0].RetrieveResult( 10000, ptrGrabResultCam1, TimeoutHandling_ThrowException);
			cameras[1].RetrieveResult( 10000, ptrGrabResultCam2, TimeoutHandling_ThrowException);
			//get ptp parameters
			//cameras[cameraContextValue ].GevIEEE1588DataSetLatch.Execute();

			//clockId = cameras[cameraContextValue ].GevIEEE1588ClockId.GetValue();
			//offset  = cameras[cameraContextValue ].GevIEEE1588OffsetFromMaster.GetValue();
			std::cout << "getting first few frames, captured frame :"  << i << std:: endl;
		}

		//cam1
		formatConverter.Convert(pylonImage, ptrGrabResultCam1);
		formatConverter.OutputBitAlignment = Pylon::OutputBitAlignmentEnums::OutputBitAlignment_MsbAligned;
		formatConverter.OutputPixelFormat = Pylon::PixelType::PixelType_BGR8packed;
		openCvImage1.create(pylonImage.GetHeight(), pylonImage.GetWidth(), CV_8UC3);
		memcpy(openCvImage1.ptr(), pylonImage.GetBuffer(), 3*pylonImage.GetWidth()*pylonImage.GetHeight());

		//cam2
		formatConverter.Convert(pylonImage, ptrGrabResultCam2);
		formatConverter.OutputBitAlignment = Pylon::OutputBitAlignmentEnums::OutputBitAlignment_MsbAligned;
		formatConverter.OutputPixelFormat = Pylon::PixelType::PixelType_BGR8packed;
		openCvImage2.create(pylonImage.GetHeight(), pylonImage.GetWidth(), CV_8UC3);
		memcpy(openCvImage2.ptr(), pylonImage.GetBuffer(), 3*pylonImage.GetWidth()*pylonImage.GetHeight());


		for (auto& s : storage) {
			storage_thread.emplace_back(&storage_worker::run, &s);
		}

		bool toggleID= false;
		int32_t const MAX_FRAME_COUNT(10000000);
		double total_read_time(0.0);
		int32_t frame_count(0);
		high_resolution_clock::time_point tstart(high_resolution_clock::now());
		double fpsID1 	= FPS;
		double fpsID2	= FPS;
		// cout << MAX_FRAME_COUNT << endl;
		//stream obj to store header of data file
		std::string line; 
		std::ostringstream ss;
		ss  << "counter"        		<< "\t"
		    << "GPIO"				<< "\t"
			<< "telapsed" 			<< "\t"
			<< "offset[ID1=" 		<< ID1 << ",ID2=" << ID2 << "]" << "\t"
			<< "dt_us" 			<< "\t"
			<< "total_read_time"  		<< "\t"  
			<< "fpsID1[" 			<< 0 << "]" << "\t" 
			<< "fpsID2[" 			<< 0 << "]" << "\t" ;
			// << "accX[" 			<< 0 << "]" << "\t"  			//gravity raw
			// << "accY[" 			<< 0 << "]" << "\t" 
			// << "accZ[" 			<< 0 << "]" << "\t" 
			// << "accXz[" 			<< 0 << "]" << "\t"			 // accel x,y,z no gravity
			// << "accYz[" 			<< 0 << "]" << "\t" 
			// << "accZz[" 			<< 0 << "]" << "\t"
			// << "accXg[" 			<< 0 << "]" << "\t"			 //gravity in g units 
			// << "accYg[" 			<< 0 << "]" << "\t" 
			// << "accZg[" 			<< 0 << "]" << "\t"
			// << "gyroX[" 			<< 0 << "]" << "\t"				//gyro reading
			// << "gyroY[" 			<< 0 << "]" << "\t"
			// << "gyroZ[" 			<< 0 << "]" << "\t" 
			// << "gyroXeul[" 			<< 0 << "]" << "\t"				//euler using quaternions
			// << "gyroYeul[" 			<< 0 << "]" << "\t"
			// << "gyroZeul[" 			<< 0 << "]" << "\t";    
			//getzero values
#ifdef ENABLE_UDP

				if (recvfrom(s, buf, BUFLEN, 0, (struct sockaddr *)&serverStorage, (socklen_t *)&slen)==-1){
					diep((char *)"recvfrom()");
				} else {
				ss 	<< "accX[" 			    << *((float*) &buf[0*4 ])<< "]" << "\t"  			//gravity raw
					<< "accY[" 			    << *((float*) &buf[1*4 ])<< "]" << "\t" 
					<< "accZ[" 			    << *((float*) &buf[2*4 ])<< "]" << "\t" 
					<< "accXz[" 			<< *((float*) &buf[3*4 ])<< "]" << "\t"			 // accel x,y,z no gravity
					<< "accYz[" 			<< *((float*) &buf[4*4 ])<< "]" << "\t" 
					<< "accZz[" 			<< *((float*) &buf[5*4 ])<< "]" << "\t"
					<< "accXg[" 			<< *((float*) &buf[6*4 ])<< "]" << "\t"			 //gravity in g units 
					<< "accYg[" 			<< *((float*) &buf[7*4 ])<< "]" << "\t" 
					<< "accZg[" 			<< *((float*) &buf[8*4 ])<< "]" << "\t"
					<< "gyroX[" 			<< *((float*) &buf[9*4 ])<< "]" << "\t"				//gyro reading
					<< "gyroY[" 			<< *((float*) &buf[10*4]) << "]" << "\t"
					<< "gyroZ[" 			<< *((float*) &buf[11*4]) << "]" << "\t" 
					<< "gyroXeul[" 			<< *((float*) &buf[12*4]) << "]" << "\t"				//euler using quaternions
					<< "gyroYeul[" 			<< *((float*) &buf[13*4]) << "]" << "\t"     
					<< "gyroZeul[" 			<< *((float*) &buf[13*4]) << "]" << "\t";  
				}
#endif
		line = ss.str();
		lines.push_back(line);	

		/*  --------------------------- wait for TTL signal to start timer ------------ */ 			
		//wait for TTL signal high to start grabbing print status of ttl pin 
		gpioGetValue(readPin, &clockval);

		while(!clockval){
			gpioGetValue(readPin, &clockval);
			//if(clockval || waitKey(1) ){
			if(clockval){
				std::cout << "TTL Triggered. Start recording." << std::endl;
				//counter measurements
				mesCounter = 0;
				break;
			}
			std::cout << "TTL low, Total measurements=" << Nmeas << ", waiting." <<std::endl;
			usleep(1); // sleep for a millisecond
		}	
		/*  ----------------------------------------------------------------------------- */ 


		/*-------------------Main capture loop---------------------------*/

		for (; frame_count < MAX_FRAME_COUNT; ++frame_count) {

			//check ttl signal get GPIO status 
			//TODO! consider putting in thread and remove here
			gpioGetValue(readPin, &clockval); 
			auto finish = std::chrono::high_resolution_clock::now();
			std::chrono::duration<double> elapsed = finish - start;

			if(elapsed.count()>TTL_WAIT){
				//LastcameraContextValue= -1; 
				//LastGrabSucceeded =  1;
				//fps_cam = -1;
				fpsID1 = 0;
				fpsID2 = 0;
				//wait ttl
				//TODO! remove pin checking in the main loop
				//gpioGetValue(readPin, &clockval);  
				if(!clockval){
			std::cout << "TTL low, Nmeas=" << mesCounter << ", waiting,"
						<< " Elapsedtime:" <<  elapsed.count() << std::endl;
				} else {
					mesCounter++;
					start = std::chrono::high_resolution_clock::now();
					std::cout << "TTL Triggered. Start recording at : " 
						<< elapsed.count() << std::endl;

				}
			} 
			else { // get camera result
			//start thread text file 
			//        std::thread twrite(write, line);

			high_resolution_clock::time_point t1(high_resolution_clock::now());

			double telapsed(static_cast<double>(duration_cast<microseconds>(t1 - tstart).count()));

			//retrieves camera image 
			cameras[0].RetrieveResult( 10000, ptrGrabResultCam1, TimeoutHandling_ThrowException);
			high_resolution_clock::time_point t1_offset(high_resolution_clock::now());
			cameras[1].RetrieveResult( 10000, ptrGrabResultCam2, TimeoutHandling_ThrowException);
			high_resolution_clock::time_point t2_offset(high_resolution_clock::now());
			//	toggleID = !toggleID;
			double dt_us_offset(static_cast<double>(duration_cast<microseconds>(t2_offset- t1_offset).count()));
			//              total_time__offset+= dt_us;
			if(ptrGrabResultCam1->GrabSucceeded() && ptrGrabResultCam2->GrabSucceeded()){				 

 					fpsID1 = cameras[0].ResultingFrameRateAbs.GetValue();
 					fpsID2 = cameras[1].ResultingFrameRateAbs.GetValue();


				//		cout << "read1 id1: " << ID1 << endl;
				cout << "cam offset: id2: " << dt_us_offset << endl;

				//get val clock 

				//cam1
				formatConverter.Convert(pylonImage, ptrGrabResultCam1);
				formatConverter.OutputBitAlignment = Pylon::OutputBitAlignmentEnums::OutputBitAlignment_MsbAligned;
				formatConverter.OutputPixelFormat = Pylon::PixelType::PixelType_BGR8packed;
				openCvImage1.create(pylonImage.GetHeight(), pylonImage.GetWidth(), CV_8UC3);
				memcpy(openCvImage1.ptr(), pylonImage.GetBuffer(), 3*pylonImage.GetWidth()*pylonImage.GetHeight());

				//cam2
				formatConverter.Convert(pylonImage, ptrGrabResultCam2);
				formatConverter.OutputBitAlignment = Pylon::OutputBitAlignmentEnums::OutputBitAlignment_MsbAligned;
				formatConverter.OutputPixelFormat = Pylon::PixelType::PixelType_BGR8packed;
				openCvImage2.create(pylonImage.GetHeight(), pylonImage.GetWidth(), CV_8UC3);
				memcpy(openCvImage2.ptr(), pylonImage.GetBuffer(), 3*pylonImage.GetWidth()*pylonImage.GetHeight());


				queue[0].push(openCvImage1.clone());
				queue[1].push(openCvImage2.clone());


				high_resolution_clock::time_point t2(high_resolution_clock::now());
				double dt_us(static_cast<double>(duration_cast<microseconds>(t2 - t1).count()));
				total_read_time += dt_us;

				std::cout << "Captured image #" << frame_count << " in "
					<< (dt_us / 1000.0) << " ms" << std::endl;

				std::ostringstream ss;
				ss 	<< frame_count   	<< "\t"
		    		<< clockval			<< "\t"
					<< telapsed 		<< "\t"
					<< dt_us_offset 	<< "\t"
					<< dt_us 		<< "\t"
					<< total_read_time  	<< "\t" 
					<< fpsID1 		<< "\t" 
					<< fpsID2 		<< "\t";

				// thread to write message
				//				line = ss.str();
				//				lines.push_back(line);	
				//	twrite.join();	
#ifdef ENABLE_UDP
				if (recvfrom(s, buf, BUFLEN, 0, (struct sockaddr *)&serverStorage, (socklen_t *)&slen)==-1){
					diep((char *)"recvfrom()");
				} else {
					ss 	<< *((float*) &buf[0*4 ]) << "\t" //acc raw
						<< *((float*) &buf[1*4 ]) << "\t" 
						<< *((float*) &buf[2*4 ]) << "\t"
						<< *((float*) &buf[3*4 ]) << "\t" //acc min gravity
						<< *((float*) &buf[4*4 ]) << "\t"
						<< *((float*) &buf[5*4 ]) << "\t"
						<< *((float*) &buf[6*4 ]) << "\t" //acc in g units
						<< *((float*) &buf[7*4 ]) << "\t"
						<< *((float*) &buf[8*4 ]) << "\t"
						<< *((float*) &buf[9*4 ]) << "\t" //gyro 
						<< *((float*) &buf[10*4]) << "\t"
						<< *((float*) &buf[11*4]) << "\t"
						<< *((float*) &buf[12*4]) << "\t" //gyro 
						<< *((float*) &buf[13*4]) << "\t"
						<< *((float*) &buf[14*4]) << "\t"

				}
#endif

				line = ss.str();
				lines.push_back(line);	
					//	twrite.join();	


					//if grab succeeded else
			} else {

				cout << "Error: " << ptrGrabResultCam1->GetErrorCode() << " " 
					<< ptrGrabResultCam1->GetErrorDescription() << endl;
				cout << "Error: " << ptrGrabResultCam2->GetErrorCode() << " " 
					<< ptrGrabResultCam2->GetErrorDescription() << endl;
			}

				//if key pressed break or while number of measurements exceeded
			if(KeyPressed() | ((mesCounter+1) >= Nmeas)){
				break;
			}
			} //ttl wait loop
		} // main for loop

		// We're done reading, cancel all the queues
		for (auto& q : queue) {
			q.cancel();
		}

		// And join all the worker threads, waiting for them to finish
		for (auto& st : storage_thread) {
			st.join();
		}

		if (frame_count == 0) {
			std::cerr << "No frames captured.\n";
			return -1;
		}

		// Report the timings
		total_read_time /= 1000.0;
		double total_write_time_a(storage[0].total_time_ms());
		double total_write_time_b(storage[1].total_time_ms());

		std::cout << "Completed processing " << frame_count << " images:\n"
				  << "  average capture time = " << (total_read_time / frame_count) << " ms\n"
				  << "  average write time [" << ID1 << "] ="<< (total_write_time_a / frame_count) << " ms\n"
				  << "  average write time [" << ID2 << "] ="<< (total_write_time_b / frame_count) << " ms\n"
				  << "  ID1 = " << ID1 					<<"\n" 
				  << "  ID2 = " << ID2 					<<"\n" 
				  << std::endl;
// write data  file
		for(auto line: lines){      
			outFile << line << "\n";
		}
		
		outFile << "\n-------------------------------end of data-------------------------------" << "\n"
				<< "Completed processing " << frame_count << " images:\n"
				<< " average capture time = " << (total_read_time / frame_count) << " ms\n"
				<< "  average write time [" << ID1 << "] ="<< (total_write_time_a / frame_count)  << " ms\n"
				<< "  average write time [" << ID2 << "] ="<< (total_write_time_b / frame_count)  << " ms\n"
				<< "\n-------------------------------settings-------------------------------" << "\n"
				<< "  ID1:        		  " << "\t"	<< ID1  		<< "\n" 
				<< "  ID2:		  	  " << "\t"	<< ID2 			<< "\n" 
				<< "  FPS target: 		  " << "\t"	<< FPS 			<< "\n" 
				<< "  Image width (pix):	  " << "\t"	<< camWidth 	<< "\n"
				<< "  Image height (pix): 	  " << "\t"	<< camHeight	<< "\n"
				<< "  TTL wait (s):		  " << "\t"	<< TTL_WAIT		<< "\n"
				<< "  N sequence:		  " << "\t"	<< Nmeas		<< "\n"
				<< "  Measurements done:          " << "\t"	<< mesCounter		<< "\n"
				<< "  maxframes:		  " << "\t"	<< NimagesTograb		<< "\n"
				<< "  Filename:			  " << "\t"	<< txtflname	<< "\n"
				<< "  Image dir:		  " << "\t"	<< saveopts		<< "\n"
				<< "  SDate:			  " << "\t"	<< SDate		<< "\n"
				<< "  EDate:			  " << "\t"	<< getTime()	<< "\n";

		outFile.close();
	}
	catch(const GenericException &e)
	{
		// Error handling
		cerr << "An exception occurred." 
			<< e.GetDescription() << endl;
		exitCode = 1;
	}
	#ifdef ENABLE_UDP
	close(s);
	#endif
	PylonTerminate(); 

	return exitCode;
}

// ============================================================================

