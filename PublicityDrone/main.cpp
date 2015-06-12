#include "ardrone/ardrone.h"
#include "PIDcontroller.h"

#include <chrono>
#include <stdint.h>
#include <stdio.h>

//#define _DEBUG 1
//#define _DEBUG_CONTOURS 1
#define _DEBUG_PID 1
//#define _DEBUG_TRACKING 1
//#define _DEBUG_COMBINED 1

//#define _READKEY 

#define MIN_AREA 100.0
#define WANTED_DISTANCE 50.0
#define MARKER_MIDDLE_X 320.0
#define MARKER_MIDDLE_Y 180.0

struct areaindex{
	double area;
	int index;
};

void *coordinateProcess(void *);
void *HUDprocess(void *);
void *trackingProcess(void *);
void *followingProcess(void *);

bool compareArea(const areaindex &, const areaindex &);

bool running;
bool following;
ARDrone ardrone;

double pos[3];

cv::Rect rect1;
cv::Rect rect2;
double marker_x;
double marker_y;
double area;

double marker2_x;
double marker2_y;
double area2;

double markerMiddle_x;
double markerMiddle_y;
double distance;

double distanceError;
double heightError;
double sideError;
double angleError;
double dT;

pthread_mutex_t *coordMutex;
pthread_mutex_t *followMutex;

int main()
{
	running = TRUE;
	following = FALSE;

	int contours_index_fin = 0;

	distanceError = 0.0;
	heightError = 0.0;
	sideError = 0.0;
	angleError = 0.0;
	dT = 0.0;

	if (!ardrone.open()){
		CVDRONE_ERROR("Could not open connection to ARDrone!");
		return -1;
	}

	std::cout << "Starting Program" << std::endl;
	std::cout << "Battery level is: " << ardrone.getBatteryPercentage() << "%" << std::endl;

	pthread_t coordThread;
	pthread_create(&coordThread, NULL, coordinateProcess, NULL);

	pthread_t hudThread;
	pthread_create(&hudThread, NULL, HUDprocess, NULL);

	pthread_t visionThread;
	pthread_create(&visionThread, NULL, trackingProcess, NULL);

	pthread_t followThread;
	pthread_create(&followThread, NULL, followingProcess, NULL);

	pthread_join(hudThread, NULL);
	pthread_join(coordThread, NULL);
	pthread_join(visionThread, NULL);
	pthread_join(followThread, NULL);
	cv::destroyAllWindows();

	ardrone.close();
}

void *coordinateProcess(void *params){
	//Starting position
	pos[0] = 0.0;
	pos[1] = 0.0;
	pos[2] = 0.0;
	
	//Kalman filter
	cv::KalmanFilter kalman(6, 4, 0);
	
	//Sampling time
	const double dt = 0.033;

	//Transisition Matrix (x, y, z, vx, vy, vz)
	cv::Mat1f F(6, 6);
	F << 1.0, 0.0, 0.0, dt, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0, dt, 0.0,
		0.0, 0.0, 1.0, 0.0, 0.0, dt,
		0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
	kalman.transitionMatrix = F;

	// Measurement matrix (0, 0, z, vx, vy, vz)
	cv::Mat1f H(4, 6);
	H << 0, 0, 1, 0, 0, 0,
		0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 1;
	kalman.measurementMatrix = H;

	// Process noise covariance (x, y, z, vx, vy, vz)
	cv::Mat1f Q(6, 6);
	Q << 0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.3, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.3, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.3;
	kalman.processNoiseCov = Q;

	// Measurement noise covariance (z, vx, vy, vz)
	cv::Mat1f R(4, 4);
	R << 0.1, 0.0, 0.00, 0.00,
		0.0, 0.1, 0.00, 0.00,
		0.0, 0.0, 0.05, 0.00,
		0.0, 0.0, 0.00, 0.05;
	kalman.measurementNoiseCov = R;

	while (running){
		Sleep(33);

		// Get an image
		cv::Mat image = ardrone.getImage();

		// Prediction
		cv::Mat prediction = kalman.predict();

		// Altitude
		double altitude = ardrone.getAltitude();

		// Orientations
		double roll = ardrone.getRoll();
		double pitch = ardrone.getPitch();
		double yaw = ardrone.getYaw();

		// Velocities
		double vx, vy, vz;
		double velocity = ardrone.getVelocity(&vx, &vy, &vz);
		cv::Mat V = (cv::Mat1f(3, 1) << vx, vy, vz);

		// Rotation matrices
		cv::Mat RZ = (cv::Mat1f(3, 3) << cos(yaw), -sin(yaw), 0.0,
			sin(yaw), cos(yaw), 0.0,
			0.0, 0.0, 1.0);
		cv::Mat RY = (cv::Mat1f(3, 3) << cos(pitch), 0.0, sin(pitch),
			0.0, 1.0, 0.0,
			-sin(pitch), 0.0, cos(pitch));
		cv::Mat RX = (cv::Mat1f(3, 3) << 1.0, 0.0, 0.0,
			0.0, cos(roll), -sin(roll),
			0.0, sin(roll), cos(roll));

		// Time [s]
		static int64 last = cv::getTickCount();
		double dt = (cv::getTickCount() - last) / cv::getTickFrequency();
		last = cv::getTickCount();

		// Local movements (z, vx, vy, vz)
		cv::Mat1f M = RZ * RY * RX * V * dt;
		cv::Mat measurement = (cv::Mat1f(4, 1) << altitude, M(0, 0), M(1, 0), M(2, 0));

		// Correction
		cv::Mat1f estimated = kalman.correct(measurement);

		if (coordMutex) pthread_mutex_lock(coordMutex);
		pos[0] = estimated(0, 0);
		pos[1] = estimated(1, 0);
		pos[2] = estimated(2, 0);
		if (coordMutex) pthread_mutex_unlock(coordMutex);


#ifdef _DEBUG
		std::cout << "X: " << pos[0] << " Y: " << pos[1] << " Z: " << pos[2] << std::endl;
#endif
	}

	return NULL;
}

void *HUDprocess(void *params){
	cv::namedWindow("Camera", CV_WINDOW_AUTOSIZE);
	cv::moveWindow("Camera", 0, 0);
	cv::resizeWindow("Camera", 1280, 720);

	cv::Size windowSize(1280, 720);
	cv::Scalar batteryColor(0, 0, 255);
	cv::Scalar rectColor(0, 255, 0);

	cv::Point batteryLocation;
	batteryLocation.x = 0;
	batteryLocation.y = 50;

	while (running){
		cv::Mat image = ardrone.getImage();

		cv::rectangle(image, rect1, rectColor);
		cv::rectangle(image, rect2, rectColor);

		cv::circle(image, cv::Point((int)marker_x, (int)marker_y), 4, cv::Scalar(0, 255, 0), -1);
		cv::circle(image, cv::Point((int)marker2_x, (int)marker2_y), 4, cv::Scalar(0, 255, 0), -1);

		cv::circle(image, cv::Point((int)markerMiddle_x, (int)markerMiddle_y), 4, cv::Scalar(255, 0, 0), -1);

		cv::Mat imageOut;
		cv::resize(image, imageOut, windowSize);

		std::string battery = "Battery: " + std::to_string(ardrone.getBatteryPercentage()) + "%";
		cv::putText(imageOut, battery, batteryLocation, CV_FONT_HERSHEY_SIMPLEX, 1.0, batteryColor);

		cv::imshow("Camera", imageOut);

		int key = cv::waitKey(1);
#ifndef _READKEY
		switch (key){
		case 27:
			running = FALSE;
			break;
		case 108:
			ardrone.takeoff();
			break;
		case 116:
			ardrone.landing();
			break;
		case 102:
			following = !following;
			break;
		default:
			break;
		}
#else
		std::cout << key << std::endl;
#endif
	}
	return NULL;
}

void *trackingProcess(void *params){
	bool tagFound = false;
	
	// Thresholds
	int minHorange = 0, maxHorange = 255;
	int minSorange = 0, maxSorange = 255;
	int minVorange = 0, maxVorange = 255;

	int minHblue = 0, maxHblue = 255;
	int minSblue = 0, maxSblue = 255;
	int minVblue = 0, maxVblue = 255;

	// XML save data
	std::string filename("thresholds.xml");
	cv::FileStorage fs(filename, cv::FileStorage::READ);

	// If there is a save file then read it
	if (fs.isOpened()) {
		maxHorange = fs["H_MAX"];
		minHorange = fs["H_MIN"];
		maxSorange = fs["S_MAX"];
		minSorange = fs["S_MIN"];
		maxVorange = fs["V_MAX"];
		minVorange = fs["V_MIN"];

		maxHblue = fs["H_MAX_BLUE"];
		minHblue = fs["H_MIN_BLUE"];
		maxSblue = fs["S_MAX_BLUE"];
		minSblue = fs["S_MIN_BLUE"];
		maxVblue = fs["V_MAX_BLUE"];
		minVblue = fs["V_MIN_BLUE"];

		fs.release();
	}

	// Create a window
	cv::namedWindow("binarized orange");
	cv::createTrackbar("H max", "binarized orange", &maxHorange, 255);
	cv::createTrackbar("H min", "binarized orange", &minHorange, 255);
	cv::createTrackbar("S max", "binarized orange", &maxSorange, 255);
	cv::createTrackbar("S min", "binarized orange", &minSorange, 255);
	cv::createTrackbar("V max", "binarized orange", &maxVorange, 255);
	cv::createTrackbar("V min", "binarized orange", &minVorange, 255);
	cv::resizeWindow("binarized orange", 0, 0);
	cv::moveWindow("binarized orange", 0, 640);

	cv::namedWindow("binarized blue");
	cv::createTrackbar("H max", "binarized blue", &maxHblue, 255);
	cv::createTrackbar("H min", "binarized blue", &minHblue, 255);
	cv::createTrackbar("S max", "binarized blue", &maxSblue, 255);
	cv::createTrackbar("S min", "binarized blue", &minSblue, 255);
	cv::createTrackbar("V max", "binarized blue", &maxVblue, 255);
	cv::createTrackbar("V min", "binarized blue", &minVblue, 255);
	cv::resizeWindow("binarized blue", 0, 0);
	cv::moveWindow("binarized blue", 0, 640);

	int64 last = cv::getTickCount();

	while (running){
		cv::waitKey(1);
		cv::Mat image = ardrone.getImage();

		cv::Mat hsvimage;
		cv::cvtColor(image, hsvimage, cv::COLOR_BGR2HSV_FULL);

		cv::Mat binarizedorange;
		cv::Scalar lowerorange(minHorange, minSorange, minVorange);
		cv::Scalar upperorange(maxHorange, maxSorange, maxVorange);
		cv::inRange(hsvimage, lowerorange, upperorange, binarizedorange);

		cv::imshow("binarized orange", binarizedorange);

		cv::Mat binarizedblue;
		cv::Scalar lowerblue(minHblue, minSblue, minVblue);
		cv::Scalar upperblue(maxHblue, maxSblue, maxVblue);
		cv::inRange(hsvimage, lowerblue, upperblue, binarizedblue);

		cv::imshow("binarized blue", binarizedblue);

		//removing noise
		cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
		cv::morphologyEx(binarizedorange, binarizedorange, cv::MORPH_OPEN, kernel);
		cv::morphologyEx(binarizedblue, binarizedblue, cv::MORPH_OPEN, kernel);

		//Dilating the binarized denoised images
		const int dilation_size = 5;
		cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1), cv::Point(dilation_size, dilation_size));
		cv::dilate(binarizedorange, binarizedorange, element);
		cv::dilate(binarizedblue, binarizedblue, element);

		cv::Mat combinedBinarized;
		cv::bitwise_and(binarizedblue, binarizedorange, combinedBinarized);

#ifdef _DEBUG_COMBINED
		cv::imshow("combined", combinedBinarized);
#endif
		//Detect contours
		std::vector<std::vector<cv::Point>> contours;
		cv::findContours(combinedBinarized.clone(), contours, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

#ifdef _DEBUG_CONTOURS
		cv::drawContours(image, contours, -1, cv::Scalar(0,0,255));
		cv::imshow("DEBUG CONTOURS", image);
#endif
		if (contours.size() >= 2){
			//Find largest contour
			std::vector<struct areaindex> areacontours;
			for (size_t i = 0; i < contours.size(); i++){
				double area = fabs(cv::contourArea(contours[i]));
				areacontours.push_back(areaindex());
				areacontours[i].area = area;
				areacontours[i].index = i;
			}

			std::sort(areacontours.begin(), areacontours.end(), compareArea);

			int indexContour = areacontours[0].index;
			int indexContour2 = areacontours[1].index;

			area = areacontours[0].area;
			area2 = areacontours[1].area;

			//Object detected
			if (indexContour >= 0 && indexContour2 >= 0 && indexContour != indexContour2 && area >= MIN_AREA && area2 >= MIN_AREA){
				cv::Moments moments = cv::moments(contours[indexContour], true);
				marker_y = (int)(moments.m01 / moments.m00);
				marker_x = (int)(moments.m10 / moments.m00);

				cv::Moments moments2 = cv::moments(contours[indexContour2], true);
				marker2_y = (int)(moments2.m01 / moments2.m00);
				marker2_x = (int)(moments2.m10 / moments2.m00);

				//Show result
				rect1 = cv::boundingRect(contours[indexContour]);
				rect2 = cv::boundingRect(contours[indexContour2]);

				markerMiddle_x = (marker_x + marker2_x) / 2.0;
				markerMiddle_y = (marker_y + marker2_y) / 2.0;

				double dX = abs(marker_x - marker2_x);
				double dY = abs(marker_y - marker2_y);

				//TODO REDO DISTANCE CALCULATIONS, NOT WORKING CURRENTLY
				//CONSTANT ZERO DISTANCE!?

				distance = sqrt(pow(dX, 2) + pow(dY, 2));

				tagFound = true;
			}
			else
				tagFound = false;

			if (tagFound && following){
				if (followMutex) pthread_mutex_lock(followMutex);
				distanceError = distance - WANTED_DISTANCE;
				angleError = MARKER_MIDDLE_X - markerMiddle_x;
				heightError = MARKER_MIDDLE_Y - markerMiddle_y;
				dT = (cv::getTickCount() - last) / cv::getTickFrequency();
				std::cout << dT << std::endl;
				last = cv::getTickCount();
				if (followMutex) pthread_mutex_unlock(followMutex);
#ifdef _DEBUG_TRACKING
				std::cout << "dT: " << dT
					<< ", EX: " << distanceError
					<< ", EY: " << sideError
					<< ", EZ: " << heightError
					<< ", ER: " << angleError
					<< '\r' << std::flush;
#endif
			}
		}
	}

	// Save thresholds
	fs.open(filename, cv::FileStorage::WRITE);
	if (fs.isOpened()) {
		cv::write(fs, "H_MAX", maxHorange);
		cv::write(fs, "H_MIN", minHorange);
		cv::write(fs, "S_MAX", maxSorange);
		cv::write(fs, "S_MIN", minSorange);
		cv::write(fs, "V_MAX", maxVorange);
		cv::write(fs, "V_MIN", minVorange);

		cv::write(fs, "H_MAX_BLUE", maxHblue);
		cv::write(fs, "H_MIN_BLUE", minHblue);
		cv::write(fs, "S_MAX_BLUE", maxSblue);
		cv::write(fs, "S_MIN_BLUE", minSblue);
		cv::write(fs, "V_MAX_BLUE", maxVblue);
		cv::write(fs, "V_MIN_BLUE", minVblue);

		fs.release();
	}

	return NULL;
}

bool compareArea(const areaindex &a, const areaindex &b){
	return a.area > b.area;
}

void *followingProcess(void *param){
	//			GAINS:		P	I	D	dT
	PIDcontroller PIDdistance(0.0, 0.0, 0.0, 0.0);
	PIDcontroller PIDheight(0.005, 0.0, 0.0, 0.0);
	PIDcontroller PIDangle(0.005, 0.0, 0.0, 0.0);

	while (running){
		if (following){
			if (followMutex) pthread_mutex_lock(followMutex);
			double ux = PIDdistance.calcControl(distanceError, dT);
			double uz = PIDheight.calcControl(heightError, dT);
			double ur = PIDangle.calcControl(angleError, dT);
			if (followMutex) pthread_mutex_unlock(followMutex);

			ardrone.move3D(0.0, 0.0, 0.0, ur);

#ifdef _DEBUG_PID
			system("cls");
			std::cout << "Battery: " << ardrone.getBatteryPercentage() << "%" << std::endl
				<< "dT: " << dT << std::endl
				<< "EX: " << distanceError << std::endl
				<< "EZ: " << heightError << std::endl
				<< "ER: " << angleError << std::endl
				<< "UX: " << ux << std::endl
				<< "UZ: " << uz << std::endl
				<< "UR: " << ur << std::endl;
#endif
		}
	}

	return NULL;
}