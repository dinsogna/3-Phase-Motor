//
//  main.cpp
//  SimplePendulum
//
//  Created by Andrew Chen on 7/19/20.
//  Copyright © 2020 Andrew Chen. All rights reserved.
//

#include <iostream>
#include <cmath>
using namespace std;


//Variables(gravity, length, damping factor, mass)
double g= 9.81;
double l= 1.00;
double b= 0.5;
double m= 1.00;

//values for theta and x where x equals theta dot
struct y{
    double theta;
    double x;
};

//INPUTS: initial=initial conditions  time=the amount of time it simulates      freq= how often a data point is calculated
void equation(y initial, double time, double freq){
    int size= (int) time/freq;
    y values[size];
    double theta1=initial.theta;
    double x1= initial.x;
    for(int i=0; i<size; i++){
        double theta2 =theta1+(freq*x1);
        double x2 =x1+(freq*(((-g/l)*sin(theta1))  -   ((b*x1)/(m*l))));
        values[i].theta=theta2;
        values[i].x=x2;
        theta1=theta2;
        x1=x2;
    }
    cout.setf(ios::fixed);
    cout.precision(3);
    
    //prints out the information
    cout<<"Time     Theta     Thetadot"<<endl;
    for(int j=0; j<size; j++){
        cout<<j*freq<<"        "<<values[j].theta<<"         "<<values[j].x<<endl;
    }
}



int main() {
    y test;
    //set initial position
    test.theta=3.14/2;
    //set initial speed
    test.x=2;
    
    //As of now, this only works when freq=0.01 or less
    //when freq=0.5 or bigger, the gaps between each data pointis too large and the values do some wacky stuff
    equation(test, 10, 0.01);
}
