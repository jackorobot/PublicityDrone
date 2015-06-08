#include "main.h"

#include <stdint.h>
#include <chrono>
#include <stdio.h>

//#define _DEBUG 1

void *coordinateProcess(void *params){
	struct coordThreadParams *coordParams = (struct coordThreadParams*)params;
	ARDrone ardrone = coordParams->ardrone;
	struct coordinates startCoords = coordParams->startingCoords;

	double *coord_x = coordParams->coord_x;
	double *coord_y = coordParams->coord_y;
	double *coord_z = coordParams->coord_z;

	*coord_x = startCoords.x;
	*coord_y = startCoords.y;
	*coord_z = startCoords.z;

	double current_vx;
	double current_vy;
	double current_vz;

	auto currentTime = std::chrono::high_resolution_clock::now();
	auto lastTime = std::chrono::high_resolution_clock::now();

	while (running){
		double pitch = ardrone.getPitch();
		double yaw = ardrone.getYaw();
		double roll = ardrone.getRoll();
		double ds = sqrt(pow(ardrone.getVelocity(&current_vx, &current_vy, &current_vz),2.0) - current_vz*current_vz);
		currentTime = std::chrono::high_resolution_clock::now();
		long long dtmicros = std::chrono::duration_cast<std::chrono::microseconds>(currentTime - lastTime).count();
		lastTime = currentTime;

		double dts = dtmicros * 1000000.0;

		double dx = cos(yaw) * ds;
		double dy = sin(yaw) * ds;
		double dz = current_vz;

		if (coordMutex) pthread_mutex_lock(&coordMutex);
		*coord_x += dx * dts;
		*coord_y += dy * dts;
		*coord_z += dz * dts;
		if (coordMutex) pthread_mutex_unlock(&coordMutex);

#ifdef _DEBUG
		std::cout << "X: " << *coord_x << " Y: " << *coord_y << " Z: " << *coord_z << std::endl;
#endif
	}

	return NULL;
}

void *HUDprocess(void *params){
	struct ardroneParam *hudparams = (struct ardroneParam*)params;
	ARDrone ardrone = hudparams->ardrone;

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

int main()
{
	running = TRUE;
	
	ARDrone ardrone;

	if (!ardrone.open()){
		CVDRONE_ERROR("Could not open connection to ARDrone!");
		return -1;
	}

	std::cout << "Starting Program" << std::endl;
	std::cout << "Battery level is: " << ardrone.getBatteryPercentage() << "%" << std::endl;

	struct coordinates startingcoords;
	startingcoords.x = 0.0;
	startingcoords.y = 0.0;
	startingcoords.z = 0.0;

	double coord_x;
	double coord_y;
	double coord_z;

	struct coordThreadParams coordParams;
	coordParams.ardrone = ardrone;
	coordParams.startingCoords = startingcoords;
	coordParams.coord_x = &coord_x;
	coordParams.coord_y = &coord_y;
	coordParams.coord_z = &coord_z;

	pthread_t coordThread;
	pthread_create(&coordThread, NULL, coordinateProcess, &coordParams);

	struct ardroneParam hudParams;
	hudParams.ardrone = ardrone;

	pthread_t hudThread;
	pthread_create(&hudThread, NULL, HUDprocess, &hudParams);

	pthread_join(hudThread, NULL);
	pthread_join(coordThread, NULL);
	cv::destroyAllWindows();
}