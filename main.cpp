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
    
	//INITIAL CONDITIONS
    double theta=0;      //initial theta (radians)
    double theta_dot=0;  //initial theta dot (radians/s)
    double iq=0;         //initial iq (amps)
    double voltage=0.1;  //initial voltage to motor Vm (volts)
    double torque=0;     //initial external torque (N*m)
    double time=1;      //total time interval (seconds)
    double dt=0.0001;       //size of one time step

    /*CONTROLLER SELECT
	* 0: No Controller
	* 1: Current Controller
	* 2: Velocity Controller
	* 3: Direct Torque Controller
	*/

    int cont_select = 0;
    double target=0;     //target for controller (torque, thetadot, or current)


    // //test
    // int gearrat = 10;
    // int test = 1000;

    // test /= gearrat;
    // std::cout << test << std::endl;
    // exit(0);

    Coupled x(theta, theta_dot, iq, voltage, torque, target, time, dt, cont_select);
    x.run();
    x.printOutput();
    
    //Motor x(theta_dot, iq, voltage, time, dt);
    //x.rk4_full(torque, target, cont_select);
    //x.printOutput();
}
