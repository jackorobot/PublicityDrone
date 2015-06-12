#pragma once
class PIDcontroller
{
public:
	PIDcontroller();
	PIDcontroller(double Kp_in, double Ki_in, double Kd_in, double dT_in, double e_prev_in = 0.0, double e_int_in = 0.0);

	void setGains(double Kp_in, double Ki_in, double Kd_in, double dT_in);

	double calcControl(double e_now);
	double calcControl(double e_now, double dT_new);

	~PIDcontroller();

private:
	double Kp, Ki, Kd;
	double dT;
	double e_prev;
	double e_int;
};

