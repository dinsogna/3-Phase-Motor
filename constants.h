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
double t=30;
double dt=0.01;

//PENDULUM SETTINGS
const double g= 9.81; //gravity
const double l= 3.00;       //length of pendulum
const double b= 0.2;        //damping factor
const double m= 2.50;       //mass of pendulum
double theta=3.14*2/3;  //position
double theta_dot=0;   //angular velocity

//MOTOR SETTINGS
const double R = 2.00;      // Resistance
const double L = 1.50;      // Inductance
const double K= 2.500;       // Motor Constant
const double B = 20.00;      // Damping Factor
const double J = 1.500;      // Inertia
double Vm = 50.974;    // Voltage to Motor
double i= 25.487;
double torque=0;

#endif /* constants_h */
