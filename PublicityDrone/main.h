#include "ardrone/ardrone.h"

void *coordinateProcess(void *);

bool running;

pthread_mutex_t coordMutex;

struct ardroneParam{
	ARDrone ardrone;
};

struct coordinates{
	double x;
	double y;
	double z;
};

struct coordThreadParams{
	ARDrone ardrone;
	struct coordinates startingCoords;
	double *coord_x;
	double *coord_y;
	double *coord_z;
};