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
double dt=0.01;

//PENDULUM SETTINGS
const double g= 9.81; //gravity
double l= 1.00;       //length of pendulum
double b= 0.2;        //damping factor
double m= 1.00;       //mass of pendulum
double theta=3.14/2;  //position
double theta_dot=0;   //angular velocity

//MOTOR SETTINGS
double R = 1.00;      // Resistance
double L = 1.00;      // Inductance
double K= 1.00;       // Motor Constant
double B = 0.00;      // Damping Factor
double J = 1.00;      // Inertia
double Vm = 8.272970; // Voltage to Motor
double i=1;
double torque=0;

#endif /* constants_h */
