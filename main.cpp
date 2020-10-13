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
    double target=0;
    double theta_dot=0;
    double iq=0;
    double voltage=10;
    double torque=1;
    double time=1;
    double dt=0.0001;
    
    //Coupled x(theta, theta_dot, iq, voltage, torque, target, time, dt);
    //x.printOutput();
    
    Motor x(theta_dot, iq, voltage, time, dt);
    x.rk4_full(torque, target);
    x.printOutput();
}
