//
//  System.hpp
//  MotorModel
//
//  Created by Andrew Chen on 9/24/20.
//  Copyright © 2020 Andrew Chen. All rights reserved.
//

#ifndef System_hpp
#define System_hpp

#include <vector>
#include <Eigen/Dense>
#include "Controller.hpp"

typedef Eigen::VectorXd state_type;

const size_t N = 2;

//=========================================================================================
//SYSTEM CLASS
//=========================================================================================
class System{
public:
    System(double init1, double init2, double time, double dt);
    virtual state_type calculate(const state_type X, const double tor)=0;
    virtual state_type rk4_step(state_type state, double dt, double &tor)=0;
    virtual void printConsole()=0;
    virtual void printFile(std::string filename)=0;
    void addState(state_type x);
    state_type getState(int x);
    state_type getBackState();
    int getSize();
    double getTime(int i);
    int getTimeSize();
    

private:
    std::vector<state_type> values;
    std::vector<double> T; // for motor only testing
};

//=========================================================================================
//PENDULUM CLASS
//=========================================================================================
class Pendulum: public System{
public:
    Pendulum(double theta, double theta_dot, double time, double dt);
    
    virtual state_type calculate(const state_type X, const double tor);
    virtual state_type rk4_step(state_type state, double dt, double &tor);
    virtual void rk4_full(double torque);
    virtual void printConsole();
    virtual void printFile(std::string filename);
private:
    double g;
    double l;
    double b;
    double m;
    
};

//=========================================================================================
//MOTOR CLASS
//=========================================================================================
class Motor: public System{
public:
    Motor(double theta_dot, double Iq, double voltage, double time, double dt);
    
    virtual state_type calculate(const state_type X, const double tor);
    virtual state_type rk4_step(state_type state, double dt, double &tor);
    state_type controlled_rk4_step(state_type state, double dt, double &tor, double target, int cont_select, int reference);
    virtual void rk4_full(double torque, double target, int cont_select);
    virtual void printConsole();
    virtual void printFile(std::string filename);
    
private:
    double Vmax;
    double R;
    double L;
    double K;
    double B;
    double J;
    double Vm;
    double Iq;
    double gear_ratio; 
    Controller cont;
    double relative_theta;
};


#endif /* System_hpp */
