//
//  System.cpp
//  MotorModel
//
//  Created by Andrew Chen on 9/24/20.
//  Copyright © 2020 Andrew Chen. All rights reserved.
//

#include "System.hpp"
#include <iostream>
#include <cmath>

//=========================================================================================
//SYSTEM FUNCTIONS
//=========================================================================================
System::System(double init1, double init2, double time, double dt){
    T.push_back(0);
    for(int i=0; i< (int) (time)/dt; i++)
        T.push_back(T[i]+dt);
    state_type init(N);
    init<<init1, init2;
    values.push_back(init);
}

void System::addState(state_type x){values.push_back(x);}
state_type System::getState(int x) {return values[x];}
int System::getSize() {return values.size();}

void System::rk4_full(double torque){
    for(int i=0; i<T.size(); i++){
        addState(rk4_step(getState(i), T[1], torque));
    }
}



void System::printOutput(){
    std::cout.setf(std::ios::fixed);
    std::cout.precision(5);
    std::cout<<"Time         First         Second"<<std::endl;
    for(int i=0; i<values.size()-1; i++)
        std::cout<<T[i]<<"         "<<values[i](0)<<"         "<<values[i](1)<<std::endl;
    std::cout << "Finished System" << std::endl;
}


//=========================================================================================
//PENDULUM FUNCTIONS
//=========================================================================================
Pendulum::Pendulum(double theta, double theta_dot, double time, double dt):System(theta, theta_dot, time, dt){
    g=9.81;
    l= 0.37;
    b= 0.07;
    m= 0.4;
}

state_type Pendulum::calculate(const state_type X, const double tor){
    state_type state(N);
    state(0) = X(1); // state(0) => dtheta = x
    state(1) = (tor)/(m*l*l) - (b*X(1))/(m*l*l) -(g/l)*sin(X(0)); // state(1) => ddtheta = -(g/l)*sin(theta)
    return state;
}

state_type Pendulum::rk4_step(state_type state, double dt, double &tor){
    double h = dt;
    double h2 = 0.5*h;
    double h6 = h/6.0;

    state_type k1 = calculate(state, tor);
    state_type k2 = calculate(state + h2*k1, tor);
    state_type k3 = calculate(state + h2*k1, tor);
    state_type k4 = calculate(state + h*k3, tor);
    
    double a=(k1(1) + 2.0*(k2(1) + k3(1)) + k4(1))/6;
    state_type newState= state+h6*(k1 + 2.0*(k2 + k3) + k4);
    tor=((a*m*l*l)+(g*m*l*sin(newState(0)))+(b*newState(1)));
    return newState;
    
}


//=========================================================================================
//MOTOR FUNCTIONS
//=========================================================================================
Motor::Motor(double theta_dot, double Iq, double voltage, double time, double dt):System(theta_dot, Iq, time, dt){
    Vmax= 30;
    R= 0.16;
    L= 0.00018;
    K= 0.088;
    B= 0.001;
    J= 0.0001;
    Vm= voltage;
    reference.push_back(B*theta_dot);
}

state_type Motor::calculate(const state_type X, const double tor){
    /*if(Vm>Vmax)
        Vm=Vmax;
    else if(Vm<0-Vmax)
        Vm=0-Vmax;*/
    
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

state_type Motor::rk4_step(state_type state, double dt, double &tor){
    
    tor/=10;
    state(0)*=10;
    double h = dt;
    double h2 = 0.5*h;
    double h6 = h/6.0;

    state_type k1 = calculate(state, tor);
    state_type k2 = calculate(state + h2*k1, tor);
    state_type k3 = calculate(state + h2*k1, tor);
    state_type k4 = calculate(state + h*k3, tor);
    
    double a=(k1(1) + 2.0*(k2(1) + k3(1)) + k4(1))/6;
    state_type newState= state+h6*(k1 + 2.0*(k2 + k3) + k4);
    tor= ((-J*a) - (B*newState(0)) + (K*newState(1)))*10;
    reference.push_back((J*a)+(B*newState(0)));
    newState(0)/=10;
    return newState;
}

double Motor::get_target_curr(double target_tor, int index){
    return (reference[index]+(target_tor/10))/K;
}

void Motor::change_volt(double v) {Vm=v;}
