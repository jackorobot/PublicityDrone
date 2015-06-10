#pragma once
class PIDcontroller
{
public:
	PIDcontroller();
	PIDcontroller(double, double, double, double, double, double);

	void setGains(double, double, double, double);

	double calcControl(double);
	double calcControl(double, double);

	~PIDcontroller();

private:
	double Kp, Ki, Kd;
	double dT;
	double e_prev;
	double e_int;
};

