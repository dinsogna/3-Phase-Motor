/*

Runge Kutta 4 Method
*/

#include <iostream>
#include <cmath>
#include <fstream>
#include <string>
#include <cstdlib>
#include <array>
using namespace std;

// Arrays to store output of RK4
const int SIZE = 1000000;

array<double, SIZE> T;
array<double, SIZE> THETA;
array<double, SIZE> THETA_DOT;


// Pendulum equation constants
// theta_dot_dot = (u/m*l*l) - (b/m*l*l)*theta_dot - (g/l)*sin(theta)
const double g = 9.81;
double l,m,b,u; // length, mass, damping constant, torque

double f1(const double& time, const double& x1, const double& x2);
double f2(const double& time, const double& x1, const double& x2);
void RK(const double& time, const double& X10, const double& X20, const double& h);
void RKSTEP(double& t, double& x1, double& x2, double& dt);


int main() {
	// SET Global Vars / Pendulum Conditions
	l = 1; // length
	m = 1; // mass
	b = 0; // damping constant
	u = 0; // constant torque


	// INPUT: Set initial conditions
	double H = .01; // discrete time step in seconds
	double TIME = 10; // total time interval in seconds
	double X10 = 1; // THETA: initial position (radians)
	double X20 = 0; // THETA_DOT initial angular velocity (radians / s)
	int i;
	double RANGE = TIME / H;
	// invoke RK4 method
	RK(TIME, X10, X20, H);

	// output data to .dat file, or to console
	ofstream output("rk4_pendulum.dat");
	output.precision(17);
	output <<"TIME THETA THETA_DOT" << endl;
	for(i=0; i<RANGE; i++) {
		output << T[i] << "	" << THETA[i] << "	" << THETA_DOT[i] << '\n';
	}
	output.close();
	// graph results with gnuplot
}
// NOTE
// x1 = theta (position)
// x2 = x / theta_dot (velocity)

// f1: theta_dot = x
double f1(const double& time, const double& x1, const double& x2) {
	return x2;
}

// f2: x_dot = (u/(m*l*l)) - (b/(m*l*l))*x - (g/l)*sin(theta)
double f2(const double& time, const double& x1, const double& x2) {
	// return -(g/l)*sin(x1);
	return (u/(m*l*l)) - (b/(m*l*l))*x2 - (g/l)*sin(x1); 
}
// Loop of RKSteps over a time interval
// time = total time interval, h = time step
void RK(const double& time, const double& X10, const double& X20, const double& h) {
	
	double dt = h;
	double TS, X1S, X2S;
	int RANGE = (int) time / h;

	T[0]          = 0.0;
	THETA[0]      = X10;
	THETA_DOT[0]  = X20;
	TS     = 0.0;
	X1S    = X10;
	X2S    = X20;

	for(int i = 1; i < RANGE; i++) {
		RKSTEP(TS, X1S, X2S, dt);
		T[i] = TS;
		THETA[i] = X1S;
		THETA_DOT[i] = X2S;
	}
}

// A single RK step
void RKSTEP(double& t, double& x1, double& x2, double& dt) {
  	
  	double k11, k12, k13, k14, k21, k22, k23, k24;
  	double h, h2;

  	h = dt;
  	h2 = dt / 2.0; 

  	// Runge Kutta 4 calculations
	k11=f1(t,x1,x2);
	k21=f2(t,x1,x2);
	k12=f1(t+h2,x1+h2*k11,x2+h2*k21);
	k22=f2(t+h2,x1+h2*k11,x2+h2*k21);
	k13=f1(t+h2,x1+h2*k12,x2+h2*k22);
	k23=f2(t+h2,x1+h2*k12,x2+h2*k22);
	k14=f1(t+h ,x1+h *k13,x2+h *k23);
	k24=f2(t+h ,x1+h *k13,x2+h *k23);

	// directly update variables
	t = t+h;
	x1 = x1 + (h/6.0) * (k11 + 2.0*k12 + 2.0*k13 + k14);
	x2 = x2 + (h/6.0) * (k21 * 2.0*k22 + 2.0*k23 + k24);
}


