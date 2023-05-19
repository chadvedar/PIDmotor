/*
# Copyright 2023 FIBO
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# Authors: Natee #
*/

#include "PIDmotor.hpp"

void PIDmotor::init(){
    this->max_pid_output = 4000;
    this->min_pid_output = -4000;
    this->loop_freq = 0.04;

    dt = loop_freq * 1000000;
    set_max_min_pid_output(max_pid_output, min_pid_output);

    pwm_motor_a.period_us(max_pid_output);
    pwm_motor_b.period_us(max_pid_output);
}

void PIDmotor::set_pid_gain(float kp, float ki, float kd, float kff){
    pid.kp = kp;
    pid.ki = ki;
    pid.kd = kd;
    pid.ff_kp = kff;
}

void PIDmotor::set_max_min_pid_output(int max, int min){
    pid.max_output = max;
    pid.min_output = min;
}

void PIDmotor::start(){
    timer_motor_control.attach(callback(this, &PIDmotor::control), loop_freq);
}

void PIDmotor::control(){
    pid_output = int(pid.calculate(target_speed, current_speed, dt));
    generate_motor_pwm(pid_output);
}

void PIDmotor::update_current_speed(float rpm){
    current_speed = rpm;
}

void PIDmotor::set_target_speed(float rpm){
    target_speed = rpm;
}

void PIDmotor::move_forward(int pwm){
    pwm_motor_a.pulsewidth_us(pwm);
    pwm_motor_b.pulsewidth_us(0);
}

void PIDmotor::move_backward(int pwm){
    pwm_motor_a.pulsewidth_us(0);
    pwm_motor_b.pulsewidth_us(pwm);
}

void PIDmotor::generate_motor_pwm(int pulseWidth){
    if(pulseWidth < 0){
        move_backward(-1*pulseWidth);
    }
    else{
        move_forward(pulseWidth);
    }
}
