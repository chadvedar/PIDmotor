#ifndef _PID
#define _PID

class PID{
	public:
		double ff_kp;
		double kp;
		double ki;
		double kd;

		double max_output;
		double min_output;

		double integral;
		double prev_error;

		PID(double kp, double ki, double kd, double ff_kp, double max_output, double min_output):
			kp(kp),
			ki(ki),
			kd(kd),
			ff_kp(ff_kp),
			max_output(max_output),
			min_output(min_output),
			prev_error(0.0),
			integral(0.0){}
		
        PID(){}
        
		~PID(){}

		double calculate(double setpoint, double pv, double dt);

};
#endif