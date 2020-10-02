//
//  main.cpp
//  MotorModel
//
//  Created by Andrew Chen on 9/24/20.
//  Copyright © 2020 Andrew Chen. All rights reserved.
//

#include <iostream>
#include "System.hpp"
#include "Coupled.hpp"

int main() {
    double theta=0;
    double target_theta=1;
    double theta_dot=0;
    double iq=10;
    double voltage=0.16;
    double torque=0.8798;
    double time=4;
    double dt=0.0001;
    
    //Coupled x(theta, theta_dot, iq, voltage, torque, target_theta, time, dt);
    //x.printOutput();
    
    Motor x(theta_dot, iq, voltage, time, dt);
    x.rk4_full(torque);
    x.printOutput();
}
