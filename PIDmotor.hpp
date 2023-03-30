#ifndef _PIDmotor
#define _PIDmotor

#include "mbed.h"
#include "PID.hpp"
#include <chrono>

using namespace std::chrono;

class PIDmotor{
    public:
        PIDmotor(PinName motorB, PinName motorA) : pwm_motor_b(motorB), pwm_motor_a(motorA) {} 
        Ticker timer_motor_control;
        PID pid;

        float target_speed;
        float current_speed;
        int max_pid_output = 4000;
        int min_pid_output = -4000;
        int pid_output;
        microseconds loop_freq = 40000us;
        int dt;

        void set_pid_gain(float kp, float ki, float kd, float kff);
        void start();
        void init();
        void update_current_speed(float rpm);
        void set_max_min_pid_output(int, int);
        void set_target_speed(float rpm);

    private:
        PwmOut pwm_motor_a;
        PwmOut pwm_motor_b;    

        void move_forward(int);
        void move_backward(int);
        void generate_motor_pwm(int);
        void control();

};

#endif