//
//  Motor.hpp
//  CoupledObject
//
//  Created by Andrew Chen on 8/24/20.
//  Copyright © 2020 Andrew Chen. All rights reserved.
//

#ifndef Motor_hpp
#define Motor_hpp

#include "System.hpp"
#include "constants.h"
class Motor: public System{
public:
    Motor(double init1, double init2):System(init1, init2){}
    
    virtual state_type calculate(const state_type X, const double tor){
        state_type state(N);

        Eigen::MatrixXd A(2,2);
        A << (-B/J), (K/J), (-K/L), (-R/L);

        Eigen::MatrixXd B(2,2);
        B << (-1/J), 0, 0, (1/L);
        
        Eigen::MatrixXd C(2,1);
        C << tor, Vm;

        state = A*X + B*C;
        
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
        tor= (J*a) + (B*newState(0)) - (K*newState(1));
        return newState;
    }
};


#endif /* Motor_hpp */
