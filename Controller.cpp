//
//  Controller.cpp
//  MotorModel
//
//  Created by Andrew Chen on 9/24/20.
//  Copyright © 2020 Andrew Chen. All rights reserved.
//

#include "Controller.hpp"

Controller::Controller(double theta_dot, double time){
    dt=theta_dot;
    t=time;
    foc_error=0;
    foc_integral=0;
    vel_error=0;
    vel_integral=0;
    pos_error=0;
    pos_integral=0;
}

//INPUT: Current  OUTPUT: Voltage
double Controller::foc_block(double cur, double tar){
    double Kp=1; //0.01
    double Ki=0; //40
    double Kd=.00;
    double newError=tar-cur;
    double proportional=Kp*newError;
    foc_integral+=Ki*((newError+foc_error)/2)*dt;
    double derivative=Kd*(newError-foc_error)/dt;
    foc_error=newError;
    return proportional+foc_integral+derivative;
}


//INPUT: Velocity  OUTPUT: Current
double Controller::velocity_block(double vel, double tar){
    double Kp=5;
    double Ki=0.1;
    double Kd=0.000;
    double newError=tar-vel;
    double proportional=Kp*newError;
    vel_integral+=Ki*((newError+vel_error)/2)*dt;
    double derivative=Kd*(newError-vel_error)/dt;
    vel_error=newError;
    return proportional+vel_integral+derivative;
}

//INPUT: Position  OUTPUT: Current
double Controller::pos_block(double pos, double tar){
    double Kp=150;
    double Ki= 20;
    double Kd=0;   //100;
    double newError=tar-pos;
    double proportional=Kp*newError;
    pos_integral+=Ki*((newError+pos_error)/2)*dt;
    double derivative=Kd*(newError-pos_error)/dt;
    pos_error=newError;
    return proportional+pos_integral+derivative;
}

double Controller::current_control(double cur, double tar){
    return foc_block(cur, tar);
}

double Controller::velocity_control(double vel, double cur, double target_vel){
    return foc_block(cur, velocity_block(vel, target_vel));
}

double Controller::direct_control(double pos, double cur, double target_pos){
    return foc_block(cur, pos_block(pos, target_pos));
}
