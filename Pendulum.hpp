//
//  Pendulum.hpp
//  CoupledObject
//
//  Created by Andrew Chen on 8/24/20.
//  Copyright © 2020 Andrew Chen. All rights reserved.
//

#ifndef Pendulum_hpp
#define Pendulum_hpp

#include "System.hpp"
#include "constants.h"
class Pendulum: public System{
public:
    Pendulum(double init1, double init2):System(init1, init2){}
    
    virtual state_type calculate(const state_type X, const double tor){
        state_type state(N);
        state(0) = X(1); // state(0) => dtheta = x
        state(1) = (tor)/(m*l*l) - (b*X(1))/(m*l*l) -(g/l)*sin(X(0)); // state(1) => ddtheta = -(g/l)*sin(theta)
        return state;
    }
    
    virtual state_type rk4_step(state_type state, double dt, double &tor){
        double h = dt;
        double h2 = 0.5*h;
        double h6 = h/6.0;

        state_type k1 = calculate(state, tor);
        state_type k2 = calculate(state + h2*k1, tor);
        state_type k3 = calculate(state + h2*k1, tor);
        state_type k4 = calculate(state + h*k3, tor);
        
        double a=(k1(1) + 2.0*(k2(1) + k3(1)) + k4(1))/6;
        state_type newState= state+h6*(k1 + 2.0*(k2 + k3) + k4);
        tor=((a*m*l*l)+(g*m*l*sin(newState(0)))+(b*newState(1))); //test
        //newState(1)*=10; //test
        return newState;
        
    }
    
};


#endif /* Pendulum_hpp */
