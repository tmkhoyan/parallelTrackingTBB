#pragma once

#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui.hpp>

#include <thread>         // std::thread, std::this_thread::yield
#include <chrono>  // for high_resolution_clock

#include <chrono>
#include <condition_variable>
// #include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include <pylon/PylonIncludes.h>
#include <pylon/gige/BaslerGigEInstantCamera.h>
#include <pylon/gige/_BaslerGigECameraParams.h>
#include <pylon/gige/BaslerGigEInstantCameraArray.h>



//udp 
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>


#define NVAR 9
#define NPACK 10
#define PORT 9909
#define SIZEVAR 4 // size of received variable

//jetson GPIO stuff
#include "jetsonGPIO.h"

// #define ENABLE_UDP
// ============================================================================
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::microseconds;

using namespace Pylon;
using namespace std;

using namespace Basler_GigECameraParams;
using namespace Basler_GigEStreamParams;

bool KeyPressed(void)
{
#if defined(PYLON_WIN_BUILD)
	return _kbhit() != 0;
#elif defined(PYLON_UNIX_BUILD)
	struct termios savedTermios;
	int savedFL;
	struct termios termios;
	int ch;

	tcgetattr(STDIN_FILENO, &savedTermios);
	savedFL = fcntl(STDIN_FILENO, F_GETFL, 0);

	termios = savedTermios;
	termios.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &termios);
	fcntl(STDIN_FILENO, F_SETFL, savedFL | O_NONBLOCK);

	ch = getchar();

	fcntl(STDIN_FILENO, F_SETFL, savedFL);
	tcsetattr(STDIN_FILENO, TCSANOW, &savedTermios);

	if (ch != EOF)
	{
		ungetc(ch, stdin);
	}

	return ch != EOF;
#endif
}


std::string getTime(std::string format){
	auto t = std::time(nullptr);
	auto tm = *std::localtime(&t);

	std::ostringstream oss;
	oss << std::put_time(&tm,format.c_str());
	auto str = oss.str();
	return str;
}
int getkey() {
	int character;
	struct termios orig_term_attr;
	struct termios new_term_attr;

	/* set the terminal to raw mode */
	tcgetattr(fileno(stdin), &orig_term_attr);
	memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
	new_term_attr.c_lflag &= ~(ECHO|ICANON);
	new_term_attr.c_cc[VTIME] = 0;
	new_term_attr.c_cc[VMIN] = 0;
	tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

	/* read a character from the stdin stream without blocking */
	/*   returns EOF (-1) if no character is available */
	character = fgetc(stdin);

	/* restore the original terminal attributes */
	tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

	return character;
}

void diep(char *s)
{
	perror(s);
	exit(1);
}

void write(std::string line)
{
    std::cout << line << "\n";
}

//class CSampleImageEventHandler : public CImageEventHandler
//{
//public:
//    virtual void OnImageGrabbed( CInstantCamera& camera, const CGrabResultPtr& ptrGrabResult)
//    {
//#ifdef PYLON_WIN_BUILD
//        // Display the image
//        Pylon::DisplayImage(1, ptrGrabResult);
//#endif

//        cout << "CSampleImageEventHandler::OnImageGrabbed called." << std::endl;
//        cout << std::endl;
//        cout << std::endl;
//    }
//};
