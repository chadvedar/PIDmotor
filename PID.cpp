#include "PID.hpp"

double PID::calculate(double setpoint, double pv, double dt){
	double error = setpoint - pv;
	double FFPout = this->ff_kp * setpoint;
	double Pout = this->kp * error;
	
	this->integral += error * dt;
	double Iout = this->ki * this->integral;

	double derivative = (error - this->prev_error)/dt;
	double Dout = this->kd * derivative;

	double output =  FFPout + Pout + Iout + Dout;

	if (output > this->max_output) output = this->max_output;
	else if (output < this->min_output) output = this->min_output;

	this->prev_error = error;

	return output;
}
