//
//  constants.h
//  CoupledObject
//
//  Created by Andrew Chen on 8/24/20.
//  Copyright © 2020 Andrew Chen. All rights reserved.
//

#ifndef constants_h
#define constants_h

#include <Eigen/Dense>

typedef Eigen::VectorXd state_type;

const size_t N = 2;


//TIME SETTINGS
double t=40;
double dt=0.01;

//PENDULUM SETTINGS
const double g= 9.81; //gravity
const double l= 3;       //length of pendulum
const double b= 0.2;        //damping factor
const double m= 2.5;       //mass of pendulum
double theta=0;  //position
double theta_dot=0;   //angular velocity

//MOTOR SETTINGS
const double R = 2;      // Resistance
const double L = 1.5;      // Inductance
const double K= 2.5;       // Motor Constant
const double B = 20;      // Damping Factor
const double J = 1.5;      // Inertia
double Vm = 0;    // Voltage to Motor
double i= 0;
double torque=0;

//Target theta

double targetTheta=3.14/3;

#endif /* constants_h */
