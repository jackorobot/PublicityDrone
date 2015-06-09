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

pthread_mutex_t *coordMutex;

int main()
{
	running = TRUE;

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
	cv::Scalar batteryColor = cv::Scalar(0, 0, 255);

	cv::Point batteryLocation;
	batteryLocation.x = 0;
	batteryLocation.y = 50;

	while (running){
		cv::Mat image = ardrone.getImage();

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


	return NULL;
}