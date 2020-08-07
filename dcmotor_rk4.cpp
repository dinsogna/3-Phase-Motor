// g++ -I /Users/dinsogna/Documents/RoMeLa/eigen dcmotor_rk4.cpp -o dcmotor_rk4
// plot "<./dcmotor_rk4" using 1:2 with lines

/*
* Author: Dominic Insogna
* DC Motor Model: RK4
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



// DC Motor Vars
double R = 1.00;   // Resistance
double L = 1.00;   // Inductance 
double K = 1.00;   // Motor Constant
double B = 1.00;   // Damping Factor
double J = 1.00;   // Intertia

double Td = 1.00; // External Load Torque
double Vm = 1.00; // Voltage to Motor

/* NOTES
input: Voltage to Motors armature:
output: Rotational Speed of Motor Shaft: dtheta
Assuming constant magnetic field, we get: 
- Motor Torque: T = Kt*i, 
- Back EMF: e = Ke*dtheta
In SI units, the motor torque and back emf constants are equal, that is Kt = Ke = K
K represents both motor torque and back emf constants

Using Newton's 2nd Law and Kirchoff's Voltage law, we get the following equations of DC motor:
- Mechanical: J(ddtheta) + b(dtheta) = Ki
- Electrical: L(di/dt) + Ri = V - K(dtheta)
*/

state_type dcmotor(const double t, const state_type X) {
	state_type state(N);

    MatrixXd A(2,2);
    A << (-B/J), (K/J), (-K/L), (-R/L);

    MatrixXd B(2,2);
    B << (-1/J), 0, 0, (1/L);
    
    MatrixXd C(2,1);
    C << Td, Vm;


    state = A*X + B*C;
	
	return state;
}


state_type rk4_step(double t, state_type state, double dt) {
    
    double h = dt;
    double h2 = 0.5*h;
    double h6 = h/6.0;

    state_type k1 = dcmotor(t, state);
    state_type k2 = dcmotor(t + h2, state + h2*k1);
    state_type k3 = dcmotor(t + h2, state + h2*k1);
    state_type k4 = dcmotor(t + h, state + h*k3);
	

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

	cout<<"Time     dTheta     i"<<endl;
    for(int i=0; i<cnt; i++){
        state_type state = OUT[i];
        cout<< T[i] <<"        "<< state(0) <<"         "<< state(1) <<endl;
    }
    cout << "Finish RK4 DC Motor" << endl;
}


int main() {
	state_type init(N);
	init << 1,1; // Initial conditions: dtheta, i
	cout << init << endl;
	rk4(init, 0, 10, .01);
	printOutput();
}