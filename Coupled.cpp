//
//  Coupled.cpp
//  MotorModel
//
//  Created by Andrew Chen on 9/24/20.
//  Copyright © 2020 Andrew Chen. All rights reserved.
//

#include "Coupled.hpp"
#include <iostream>

Coupled::Coupled(double theta, double theta_dot, double Iq, double Vm, double tor, double tar, double time, double dt)
:pend(theta, theta_dot, time, dt), mot(theta_dot, Iq, Vm, time, dt){
    torque=tor;
    mot.change_volt(cont.newVoltage(theta, tar, dt, 0));
    int totalSteps = (int) (time) / dt;
    state_type next_pend(N);
    state_type next_motor(N);
    T.push_back(0);
    T.push_back(T[0] + dt);
    pend.addState(pend.rk4_step(pend.getState(0), dt, torque));
    mot.change_volt(cont.newVoltage(pend.getState(1)(0), tar, dt, 0));
    next_motor<<pend.getState(1)(1), Iq;
    mot.addState(mot.rk4_step(next_motor, dt, torque));
    next_pend<<pend.getState(1)(0), mot.getState(1)(0);
    for(int j=1; j<totalSteps; j++){
        T.push_back(T[j]+dt);
        pend.addState(pend.rk4_step(next_pend, dt, torque));
        mot.change_volt(cont.newVoltage(pend.getState(j+1)(0), tar, dt, 0));
        next_motor<<pend.getState(j+1)(1), mot.getState(j)(1);
        mot.addState(mot.rk4_step(next_motor, dt, torque));
        next_pend<<pend.getState(j+1)(0), mot.getState(j+1)(0);
    }
}

void Coupled::printOutput() {
    std::cout.setf(std::ios::fixed);
    std::cout.precision(5);
    std::cout<<"               PENDULUM                         MOTOR "<<std::endl;
    std::cout<<"Time           theta           thetadot         thetadot         current         "<<std::endl;
    for(int a=0; a<T.size(); a++){
        std::cout<< T[a] <<"        "<< pend.getState(a)(0) <<"         "<< pend.getState(a)(1)<<"         "<<mot.getState(a)(0)<<"         "<<mot.getState(a)(1)<<std::endl;
    }
    std::cout << "Finish RK4 COUPLED" << std::endl;
    
}
