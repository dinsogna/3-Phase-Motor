
//  main.cpp


#include <iostream>
#include "System.hpp"
#include "Coupled.hpp"

int main() {
    
	//INITIAL CONDITIONS
    double theta=0;      //initial theta (radians)
    double theta_dot=0;  //initial theta dot (radians/s)
    double iq=0;         //initial iq (amps)
    double voltage=0;  //initial voltage to motor Vm (volts)
    double torque=0;     //initial external torque (N*m)
    double time=1;      //total time interval (seconds)
    double dt=0.00001;       //size of one time step

    /*CONTROLLER SELECT
	* 0: No Controller
	* 1: Current Controller
	* 2: Direct Position Controller
	* 3: Velocity Controller
	*/

    int cont_select = 2;
    double target=0.1;     //target for controller (torque, thetadot, or current)


    Coupled x(theta, theta_dot, iq, voltage, torque, target, time, dt, cont_select);
    x.run();
    x.printConsole();
    x.printFile("output.csv");

}
