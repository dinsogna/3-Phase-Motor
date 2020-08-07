// g++ -I /Users/dinsogna/Documents/RoMeLa/eigen ependrk4.cpp -o ependrk4
// plot "<./epend" using 1:2 with lines

/*
* Author: Dominic Insogna
* Damped Driven Pendulum Model: RK4
*/


#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <array>

using namespace std;
using namespace Eigen;


const size_t N = 2;
const size_t SIZE = 100000;
array<double, SIZE> T;
size_t cnt = 1;

typedef VectorXd state_type;
array<state_type, SIZE> OUT;


//Variables(gravity, length, damping factor, mass)
double g= 9.81;
double l= 1.00;
double b= 0.2;
double m= 1.00;
double u= 3.00;


state_type pendulum(const double t, const state_type X) {
	state_type state(N);
	state(0) = X(1); // state(0) => dtheta = x
	state(1) = (u)/(m*l*l) - (b*X(1))/(m*l*l) -(g/l)*sin(X(0)); // state(1) => ddtheta = -(g/l)*sin(theta)
	return state;
}


state_type rk4_step(double t, state_type state, double dt) {
    
    double h = dt;
    double h2 = 0.5*h;
    double h6 = h/6.0;

    state_type k1 = pendulum(t, state);
    state_type k2 = pendulum(t + h2, state + h2*k1);
    state_type k3 = pendulum(t + h2, state + h2*k1);
    state_type k4 = pendulum(t + h, state + h*k3);
	

	return state + h6*(k1 + 2.0*(k2 + k3) + k4);
}

void rk4(state_type init, double start, double end, double dt) {
    
    int totalSteps = (int) (end - start) / dt;
    T[0] = start;
    OUT[0] = init;

    for(int i=0; i<totalSteps; i++){
    	T[i+1] = T[i] + dt;
    	OUT[i+1] = rk4_step(T[i], OUT[i], dt);
    	cnt ++;	
    }
 }

void printOutput() {
	cout.setf(ios::fixed);
    cout.precision(17);

	cout<<"Time     Theta     Thetadot"<<endl;
    for(int i=0; i<cnt; i++){
        state_type state = OUT[i];
        cout<< T[i] <<"        "<< state(0) <<"         "<< state(1) <<endl;
    }
    cout << "Finish RK4 Pendulum" << endl;
}


int main() {
	state_type v(N);
	v << 1, 2;
	cout << v << endl;

	state_type init(N);
	init << 1,0; // Initial conditions: theta, dtheta
	cout << init << endl;
	rk4(init, 0, 10, .01);
	printOutput();
}