#include "PIDcontroller.h"


PIDcontroller::PIDcontroller()
{
	Kp = 0.0;
	Ki = 0.0;
	Kd = 0.0;
	dT = 0.0;
	e_prev = 0.0;
	e_int = 0.0;
}

PIDcontroller::PIDcontroller(double Kp_in, double Ki_in, double Kd_in, double dT_in, double e_prev_in, double e_int_in){
	Kp = Kp_in;
	Ki = Ki_in;
	Kd = Kd_in;
	dT = dT_in;
	e_prev = e_prev_in;
	e_int = e_int_in;
}

PIDcontroller::~PIDcontroller()
{
}

void PIDcontroller::setGains(double Kp_in, double Ki_in, double Kd_in, double dT_in){
	Kp = Kp_in;
	Ki = Ki_in;
	Kd = Kd_in;
	dT = dT_in;
}

double PIDcontroller::calcControl(double e_now){
	e_int = e_now * dT + e_int;
	double de = (e_now - e_prev) / dT;
	double u = (Kp*e_now) + (Ki*e_int) + (Kd*de);
	e_prev = e_now;
	return u;
}

double PIDcontroller::calcControl(double e_now, double dT_new){
	dT = dT_new;
	return calcControl(e_now);
}