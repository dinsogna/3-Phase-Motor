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
    Controller(double theta_dot, double time);
    
    //INPUT: Current  OUTPUT: Voltage
    double foc_block(double cur, double tar);
    //INPUT: Velocity  OUTPUT: Current
    double velocity_block(double vel, double tar);
    //INPUT: Position  OUTPUT: Current
    double pos_block(double pos, double tar);
    
    
    double current_control(double cur, double tar);
    double velocity_control(double vel, double cur, double target_vel, int reference);
    double direct_control(double pos, double cur, double target_pos, int reference);
    
private:
    double t;
    double dt;
    double foc_error;
    double foc_integral;
    double vel_error;
    double vel_integral;
    double pos_error;
    double pos_integral;
    double tar_cur;
};
#endif /* Controller_hpp */
