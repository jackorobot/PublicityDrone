#include "ardrone/ardrone.h"
#include "PIDcontroller.h"

#include <stdint.h>
#include <stdio.h>

//#define _DEBUG 1
//#define _READKEY 1

#define WANTED_AREA 500.0
#define MARKER_MIDDLE_X 320.0
#define MARKER_MIDDLE_Y 180.0

struct areaindex{
	double area;
	int index;
};

void *coordinateProcess(void *);
void *HUDprocess(void *);
void *trackingProcess(void *);

bool compareArea(const areaindex &, const areaindex &);

bool running;
bool following;
ARDrone ardrone;

double pos[3];

cv::Rect rect1;
cv::Rect rect2;
double marker1_x;
double marker1_y;
double marker2_x;
double marker2_y;
double area1;
double area2;

double arealeft;
double arearight;
double markerMiddle_x;
double markerMiddle_y;

pthread_mutex_t *coordMutex;

int main()
{
	running = TRUE;

	following = FALSE;

	int contours_index_fin = 0;

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

	pthread_join(hudThread, NULL);
	pthread_join(coordThread, NULL);
	pthread_join(visionThread, NULL);
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

		cv::circle(image, cv::Point(markerMiddle_x, markerMiddle_y), 4, cv::Scalar(255, 0, 0), -1);

		std::stringstream stringarea1;
		stringarea1 << area1;
		std::stringstream stringarea2;
		stringarea2 << area2;

		cv::putText(image, stringarea1.str(), cv::Point((int)marker1_x, (int)marker1_y), CV_FONT_HERSHEY_SIMPLEX, 1.0, batteryColor);
		cv::putText(image, stringarea2.str(), cv::Point((int)marker2_x, (int)marker2_y), CV_FONT_HERSHEY_SIMPLEX, 1.0, batteryColor);

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
	// Thresholds
	int minH = 0, maxH = 255;
	int minS = 0, maxS = 255;
	int minV = 0, maxV = 255;

	// XML save data
	std::string filename("thresholds.xml");
	cv::FileStorage fs(filename, cv::FileStorage::READ);

	// If there is a save file then read it
	if (fs.isOpened()) {
		maxH = fs["H_MAX"];
		minH = fs["H_MIN"];
		maxS = fs["S_MAX"];
		minS = fs["S_MIN"];
		maxV = fs["V_MAX"];
		minV = fs["V_MIN"];
		fs.release();
	}

	// Create a window
	cv::namedWindow("binarized");
	cv::createTrackbar("H max", "binarized", &maxH, 255);
	cv::createTrackbar("H min", "binarized", &minH, 255);
	cv::createTrackbar("S max", "binarized", &maxS, 255);
	cv::createTrackbar("S min", "binarized", &minS, 255);
	cv::createTrackbar("V max", "binarized", &maxV, 255);
	cv::createTrackbar("V min", "binarized", &minV, 255);
	cv::resizeWindow("binarized", 0, 0);
	cv::moveWindow("binarized", 0, 640);

	while (running){
		cv::waitKey(1);
		cv::Mat image = ardrone.getImage();

		cv::Mat hsvimage;
		cv::cvtColor(image, hsvimage, cv::COLOR_BGR2HSV_FULL);

		cv::Mat binarized;
		cv::Scalar lower(minH, minS, minV);
		cv::Scalar upper(maxH, maxS, maxV);
		cv::inRange(hsvimage, lower, upper, binarized);

		cv::imshow("binarized", binarized);

		//removing noise
		cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
		cv::morphologyEx(binarized, binarized, cv::MORPH_CLOSE, kernel);

		//Detect contours
		std::vector<std::vector<cv::Point>> contours;
		cv::findContours(binarized.clone(), contours, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

#ifdef _DEBUG
		cv::drawContours(image, contours, -1, cv::Scalar(0,0,255));
		cv::imshow("DEBUG CONTOURS", image);
#endif

		//Find largest contour
		std::vector<struct areaindex> areacontours;
		for (size_t i = 0; i < contours.size(); i++){
			double area = fabs(cv::contourArea(contours[i]));
			areacontours.push_back(areaindex());
			areacontours[i].area = area;
			areacontours[i].index = i;
		}

		std::sort(areacontours.begin(), areacontours.end(), compareArea);

		int indexContour1 = areacontours[0].index;
		int indexContour2 = areacontours[1].index;

		area1 = areacontours[0].area;
		area2 = areacontours[1].area;

		//Object detected
		if (indexContour1 >= 0 && indexContour2 >= 0 && indexContour1 != indexContour2){
			cv::Moments moments1 = cv::moments(contours[indexContour1], true);
			marker1_y = (int)(moments1.m01 / moments1.m00);
			marker1_x = (int)(moments1.m10 / moments1.m00);

			cv::Moments moments2 = cv::moments(contours[indexContour2], true);
			marker2_y = (int)(moments2.m01 / moments2.m00);
			marker2_x = (int)(moments2.m10 / moments2.m00);

			if (marker1_x > marker2_x){
				arealeft = areacontours[0].area;
				arearight = areacontours[1].area;
			}
			else{
				arealeft = areacontours[1].area;
				arearight = areacontours[0].area;
			}

			markerMiddle_x = (marker1_x + marker2_x) / 2;
			markerMiddle_y = (marker1_y + marker2_y) / 2;

			//Show result
			rect1 = cv::boundingRect(contours[indexContour1]);
			rect2 = cv::boundingRect(contours[indexContour2]);
		}

		if (following){
			double distanceError = (area1 + area2) / 2 - WANTED_AREA;
			double angleError = MARKER_MIDDLE_X - markerMiddle_x;
			double heightError = MARKER_MIDDLE_Y - markerMiddle_y;
			double sideError = (arealeft - arearight);

		}
	}

	// Save thresholds
	fs.open(filename, cv::FileStorage::WRITE);
	if (fs.isOpened()) {
		cv::write(fs, "H_MAX", maxH);
		cv::write(fs, "H_MIN", minH);
		cv::write(fs, "S_MAX", maxS);
		cv::write(fs, "S_MIN", minS);
		cv::write(fs, "V_MAX", maxV);
		cv::write(fs, "V_MIN", minV);
		fs.release();
	}

	return NULL;
}

bool compareArea(const areaindex &a, const areaindex &b){
	return a.area > b.area;
}