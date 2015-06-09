#include "ardrone/ardrone.h"

#include <stdint.h>
#include <stdio.h>

//#define _DEBUG 1

void *coordinateProcess(void *);
void *HUDprocess(void *);
void *trackingProcess(void *);

bool running;
ARDrone ardrone;

double pos[3];

cv::Point2f rect[4];

pthread_mutex_t *coordMutex;

int main()
{
	running = TRUE;

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
		cv::waitKey(33);

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

		cv::line(image, rect[0], rect[1], cv::Scalar(0, 255, 0));
		cv::line(image, rect[1], rect[2], cv::Scalar(0, 255, 0));
		cv::line(image, rect[2], rect[3], cv::Scalar(0, 255, 0));
		cv::line(image, rect[3], rect[0], cv::Scalar(0, 255, 0));

		cv::Mat imageOut;
		cv::resize(image, imageOut, windowSize);

		std::string battery = "Battery: " + std::to_string(ardrone.getBatteryPercentage()) + "%";
		cv::putText(imageOut, battery, batteryLocation, CV_FONT_HERSHEY_SIMPLEX, 1.0, batteryColor);

		cv::imshow("Camera", imageOut);

		int key = cv::waitKey(1);
		switch (key){
		case 27:
			running = FALSE;
			break;
		case 116:
			ardrone.takeoff();
			break;
		case 108:
			ardrone.landing();
			break;
		default:
			break;
		}
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

		//Find largest contour
		int contour_index = -1;
		double max_area = 0.0;
		for (size_t i = 0; i < contours.size(); i++){
			double area = fabs(cv::contourArea(contours[i]));
			if (area > max_area){
				contour_index = i;
				max_area = area;
			}
		}

		//Object detected
		if (contour_index >= 0){
			//cv::Moments moments = cv::moments(contours[contour_index], true);
			//double marker_y = (int)(moments.m01 / moments.m00);
			//double marker_x = (int)(moments.m10 / moments.m00);

			////Show result
			//rect = cv::boundingRect(contours[contour_index]);

			cv::RotatedRect boundingBox = cv::minAreaRect(contours[contour_index]);
			cv::Point2f corners[4];
			boundingBox.points(corners);

			std::memcpy(rect, corners, sizeof(corners));
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