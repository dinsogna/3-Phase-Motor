//
//  Controller.hpp
//  MotorModel
//
//  Created by Andrew Chen on 9/24/20.
//  Copyright © 2020 Andrew Chen. All rights reserved.
//

#ifndef Controller_hpp
#define Controller_hpp

#include <cmath>

class Controller{
public:
    Controller(){
        Kp=100;
        Ki=1;
        Kd=0;
        proportional=0;
        integral=0;
        derivative=0;
        error=0;
    }
    
    double newVoltage(double theta, double tar, double dt, double t){
        //target=sin(3.14*14*t);
        target=tar;
        double newError=target-theta;
        proportional=Kp*newError;
        integral+=Ki*((newError+error)/2)*dt;
        derivative=Kd*(newError-error)/dt;
        error=newError;
        return proportional+integral+derivative;
    }
    
    
    
private:
    double Kp;
    double Ki;
    double Kd;
    double proportional;
    double integral;
    double derivative;
    double target;
    double error;
};
#endif /* Controller_hpp */
