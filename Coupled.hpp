//
//  Coupled.hpp
//  CoupledObject
//
//  Created by Andrew Chen on 8/24/20.
//  Copyright © 2020 Andrew Chen. All rights reserved.
//

#ifndef Coupled_hpp
#define Coupled_hpp
#include "constants.h"
#include "Pendulum.hpp"
#include "Motor.hpp"
#include "Controller.hpp"
#include <iostream>

class Coupled{
public:
    Coupled():pend(theta, theta_dot), mot(theta_dot, i), cont(targetTheta) {
        //Vm=cont.newVoltage(theta, dt);
        int totalSteps = (int) (t) / dt;
        state_type next_pend(N);
        state_type next_motor(N);
        T.push_back(0);
        T.push_back(T[0] + dt);
        pend.addState(pend.rk4_step(pend.getState(0), dt, torque));
        //Vm=cont.newVoltage(pend.getState(1)(0), dt);
        next_motor<<pend.getState(1)(1), i;
        mot.addState(mot.rk4_step(next_motor, dt, torque));
        next_pend<<pend.getState(1)(0), mot.getState(1)(0);
        for(int j=1; j<totalSteps; j++){
            T.push_back(T[j]+dt);
            pend.addState(pend.rk4_step(next_pend, dt, torque));
            //Vm=cont.newVoltage(pend.getState(j+1)(0), dt);
            next_motor<<pend.getState(j+1)(1), mot.getState(j)(1);
            mot.addState(mot.rk4_step(next_motor, dt, torque));
            next_pend<<pend.getState(j+1)(0), mot.getState(j+1)(0);
        } 
        
    }
    
    void printOutput() {
        std::cout.setf(std::ios::fixed);
        std::cout.precision(5);
        std::cout<<"               PENDULUM                         MOTOR "<<std::endl;
        std::cout<<"Time           theta           thetadot         thetadot         current         error"<<std::endl;
        for(int a=0; a<T.size(); a++){
            std::cout<< T[a] <<"        "<< pend.getState(a)(0) <<"         "<< pend.getState(a)(1)<<"         "<<mot.getState(a)(0)<<"         "<<mot.getState(a)(1)<<"         "<<targetTheta-pend.getState(a)(0)<<"         "<<Vm<<std::endl;
        }
        std::cout << "Finish RK4 COUPLED" << std::endl;
        
    }
    
    
private:
    Pendulum pend;
    Motor mot;
    Controller cont;
    std::vector<double> T;
};

#endif /* Coupled_hpp */
