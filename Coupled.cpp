//
//  Coupled.cpp
//  MotorModel
//
//  Created by Andrew Chen on 9/24/20.
//  Copyright © 2020 Andrew Chen. All rights reserved.
//

#include "Coupled.hpp"
#include <iostream>
#include <fstream>
#include <cmath>

Coupled::Coupled(double theta, double theta_dot, double Iq, double Vm, double tor, double tar, double t, double step, int cont)
:pend(theta, theta_dot, t, step), mot(theta_dot, Iq, Vm, t, step), cont(step, t){
    
    torque=tor;
    motor_torque.push_back(torque);
    T.push_back(0);
    target = tar;
    time = t;
    dt = step; 
    cont_select = cont;
 }
    
void Coupled::run() {
	
	int totalSteps = (int) (time) / dt;
    state_type next_pend(N);
    state_type next_motor(N);
    next_pend<<pend.getState(0)(0), pend.getState(0)(1);
    
    for(int j=0; j<totalSteps; j++){
        T.push_back(T[j]+dt); //add next time step to T vector (Coupled object)
        pend.addState(pend.rk4_step(next_pend, dt, torque)); //execute 1 RK4 for Pendulum, update "values" vector (Pendlum object) with new theta, thetadot; update torque
        next_motor<<pend.getState(j+1)(1), mot.getState(j)(1); //Update current motor state with Pendulum's new velocity, current motor iq current, and udpated torque
	    mot.addState(mot.controlled_rk4_step(next_motor, dt, torque, target, cont_select, j+1)); //execute 1 RK4 for Motor, update "values" vector (Motor object) with new thetadot, iq
	    motor_torque.push_back(torque); //log torque
	    next_pend<<pend.getState(j+1)(0), mot.getState(j+1)(0);

    }
}
    

void Coupled::printConsole() {
    std::cout.setf(std::ios::fixed);
    std::cout.precision(5);
    std::cout<<"               PENDULUM                         MOTOR "<<std::endl;
    std::cout<<"Time           theta           thetadot         thetadot         current         "<<std::endl;

    for(int a=0; a<T.size(); a++){
        std::cout<< T[a] <<"        "<< pend.getState(a)(0) <<"         "<< pend.getState(a)(1)<<"         "<<mot.getState(a)(0)<<"         "<<mot.getState(a)(1)<<"         "<<motor_torque[a]<<"         "<<0.2*sin(3.14*30*T[a])<<std::endl;
        
    }
    std::cout << "Finish RK4 COUPLED" << std::endl;
    
}

void Coupled::printFile(std::string fileName) {
    
    std::ofstream myfile;
    myfile.open(fileName);

    std::cout.setf(std::ios::fixed);
    std::cout.precision(5);
    
    myfile <<"Time,Pendulum Theta,Pendulum Thetadot,Motor Thetadot,Motor Current" << std::endl;

    for(int a=0; a<T.size(); a++){
        myfile << T[a] << "," << pend.getState(a)(0) << "," << pend.getState(a)(1) << "," <<mot.getState(a)(0) << "," <<mot.getState(a)(1) << std::endl;

        
    }
    myfile.close();
    std::cout << "Results printed to " << fileName << std::endl;
    
}
