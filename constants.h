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
double t=20;
double dt=0.0001;

//PENDULUM SETTINGS
const double g= 9.81; //gravity
const double l= 0.37;       //length of pendulum
const double b= 0.07;        //damping factor
const double m= 0.4;       //mass of pendulum
double theta=0;  //position
double theta_dot=0;   //angular velocity

//MOTOR SETTINGS
const double Vmax=12;
const double R = 0.16;      // Resistance
const double L = 0.00018;      // Inductance***
const double K= 0.088;       // Motor Constant
const double B = 0.001;      // Damping Factor
const double J = 0.0001;      // Inertia
double Vm = .1579;    // Voltage to Motor
double i= 0.987;
double torque=0;



//Target theta

double targetTheta=2.5;

#endif /* constants_h */
