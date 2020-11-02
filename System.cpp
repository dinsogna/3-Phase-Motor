//
//  System.cpp
//  MotorModel
//
//  Created by Andrew Chen on 9/24/20.
//  Copyright © 2020 Andrew Chen. All rights reserved.
//

#include "System.hpp"
#include <iostream>
#include <fstream>
#include <cmath>
#include "Controller.hpp"

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
state_type System::getBackState() {return values.back();}
int System::getSize() {return values.size();}
double System::getTime(int i){return T[i];}
int System::getTimeSize(){return T.size();}




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

void Pendulum::run(double torque){
    for(int i=0; i<getTimeSize(); i++){
        double input_torque=torque;
        addState(rk4_step(getState(i), getTime(1), input_torque));
    }
}

void Pendulum::printConsole(){
    std::cout.setf(std::ios::fixed);
    std::cout.precision(5);
    std::cout<<"Time         Theta         Thetadot"<<std::endl;
    for(int i=0; i<getTimeSize(); i++)
        std::cout<<getTime(i)<<"         "<<getState(i)(0)<<"         "<<getState(i)(1)<<std::endl;
    std::cout << "Finished Pendulum" << std::endl;
}

void Pendulum::printFile(std::string fileName) {
    std::ofstream myfile;
    myfile.open(fileName);
    std::cout.setf(std::ios::fixed);
    std::cout.precision(5);
    myfile <<"Time, Theta, Thetadot" << std::endl;
    for(int a=0; a<getTimeSize(); a++)
        myfile << getTime(a) << "," << getState(a)(0) << "," << getState(a)(1)<< std::endl;
    myfile.close();
    std::cout << "Results printed to " << fileName << std::endl;
    
}

//=========================================================================================
//MOTOR FUNCTIONS
//=========================================================================================
Motor::Motor(double theta_dot, double Iq, double voltage, double time, double dt):System(theta_dot, Iq, time, dt), cont(dt, time){
    Vmax= 30;
    R= 0.16;
    L= 0.00018;
    K= 0.088;
    B= .001;
    J= 0.0001;
    Vm= voltage;
    relative_theta=0;
    gear_ratio = 10;
}

state_type Motor::calculate(const state_type X, const double tor){
    if(Vm>Vmax)
        Vm=Vmax;
    else if(Vm<0-Vmax)
        Vm=0-Vmax;
    
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
    

    tor/=gear_ratio;
    state(0)*=gear_ratio;
    double h = dt;
    double h2 = 0.5*h;
    double h6 = h/6.0;

    state_type k1 = calculate(state, tor);
    state_type k2 = calculate(state + h2*k1, tor);
    state_type k3 = calculate(state + h2*k1, tor);
    state_type k4 = calculate(state + h*k3, tor);
    
    double a=(k1(0) + (2.0*(k2(0) + k3(0))) + k4(0))/(6*gear_ratio);
    
    state_type newState = state+(h6*(k1 + (2.0*(k2 + k3)) + k4));
    tor= ((-J*a) - (B*newState(0)) + (K*newState(1)))*gear_ratio;
    newState(0)/= gear_ratio;
    relative_theta += (newState(0)*dt);
    return newState;
}

state_type Motor::controlled_rk4_step(state_type state, double dt, double &tor, double target, int cont_select, int reference){
    
	switch(cont_select) {
		case 1: //CURRENT CONTROLLER
    		Vm=Vmax*cont.current_control(state(1), target);
			break;
		case 2: //DIRECT POSITION CONTROLLER
    		Vm=Vmax*cont.direct_control(relative_theta, state(1), target, reference);
			break;
		case 3: //VELOCITY CONTROLLER
			Vm = Vmax*cont.velocity_control(state(0), state(1), target, reference);
			break;
        case 4: //SIN TEST
            Vm=Vmax*cont.direct_control(relative_theta, state(1), 0.2*sin(3.14*30*reference*dt), reference);
            break;
		default:
			break;
	}
    return rk4_step(state, dt, tor);
}



void Motor::run(double torque, double target, int cont_select){
    for(int i=0; i<getTimeSize(); i++){
        double input_torque=torque;
        addState(controlled_rk4_step(getState(i), getTime(1), input_torque, target, cont_select, i+1));
    }
}

void Motor::printConsole(){
    std::cout.setf(std::ios::fixed);
    std::cout.precision(5);
    std::cout<<"Time         Thetadot         Current"<<std::endl;
    for(int i=0; i<getTimeSize(); i++)
        std::cout<<getTime(i)<<"         "<<getState(i)(0)<<"         "<<getState(i)(1)<<std::endl;
    std::cout << "Finished Motor" << std::endl;
}

void Motor::printFile(std::string fileName) {
    std::ofstream myfile;
    myfile.open(fileName);
    std::cout.setf(std::ios::fixed);
    std::cout.precision(5);
    
    myfile <<"Time, Thetadot, Current" << std::endl;

    for(int a=0; a<getTimeSize(); a++)
        myfile << getTime(a) << "," << getState(a)(0) << "," << getState(a)(1)<< std::endl;
    myfile.close();
    std::cout << "Results printed to " << fileName << std::endl;
}
