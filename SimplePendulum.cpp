//
//  main.cpp
//  SimplePendulum
//
//  Created by Andrew Chen on 7/19/20.
//  Copyright © 2020 Andrew Chen. All rights reserved.
//

#include <fstream>
#include <iostream>
#include <cmath>
#include <chrono>
using namespace std;


//Variables(gravity, length, damping factor, mass)
const double g= 9.81;
double l= 1.00;
double b= 0.5;
double m= 1.00;

//values for theta and x where x equals theta dot
struct y{
    double theta;
    double x;
};

//INPUTS: initial=initial conditions  time=the amount of time it simulates      freq= how often a data point is calculated
void equation(y initial, double time, double freq, ostream& outf){
    int size= (int) time/freq;
    y values[size];
    double torque[size];
    double theta1=initial.theta;
    double x1= initial.x;
    for(int i=0; i<size; i++){
        
        //equation that we can modify that shows the input torque over time
        double tor= 5;
        
        double theta2 =theta1+(freq*x1);
        double x2 =x1+(freq*(((-g/l)*sin(theta1))  -   ((b*x1)/(m*l*l)) +(tor/(m*l*l))));
        values[i].theta=theta2;
        values[i].x=x2;
        torque[i]= tor;
        theta1=theta2;
        x1=x2;
    }
    outf.setf(ios::fixed);
    outf.precision(3);
    
    //prints out the information
    outf<<"Time           Theta          Thetadot       Torque"<<endl;
    for(int j=0; j<size; j++){
        outf<<j*freq<<"          "<<values[j].theta<<"          "<<values[j].x<<"          "<<torque[j]<<endl;
    }
}



int main() {
    y test;
    //set initial position
    test.theta=3.14/2;
    //set initial speed
    test.x=0;
    
    //set your own file path if you want the txt to go to a specific folder
    //CAUTION: if file name is already present, it will override data. If no file present, it will create it for you
    //         make sure you change file name so that you don't override files that you wanted to keep
    ofstream outfile("results.txt");
    if ( ! outfile ){
        cerr << "Error: Cannot create file!" << endl;
        return 1;
    }

    //starts timer
    auto start= std::chrono::high_resolution_clock::now();
    
    //As of now, this only works when freq=0.01 or less
    //when freq=0.5 or bigger, the gaps between each data pointis too large and the values do some wacky stuff
    equation(test, 20, 0.001, outfile);
    
    //ends timer
    auto stop= std::chrono::high_resolution_clock::now();
    
    //calculates time diff between stop and start and outputs it
    auto duration = duration_cast<std::chrono::milliseconds>(stop - start);
    outfile << "Time taken by function: " << duration.count() << " milliseconds" << endl;
}
