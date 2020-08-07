// g++ -I /Users/dinsogna/Documents/RoMeLa/eigen epend.cpp -o epend
// plot "<./epend" using 1:2 with lines


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
double u= 0.00;


state_type f(const state_type X) {
	state_type state(N);
	state(0) = X(1); // state(0) => dtheta = x
	state(1) = (u)/(m*l*l) - (b*X(1))/(m*l*l) -(g/l)*sin(X(0)); // state(1) => ddtheta = -(g/l)*sin(theta)
	return state;
}


state_type euler_step(state_type state, double dt) {
    
	
	return state + dt*f(state);
}

void euler(state_type init, double start, double end, double dt) {
    
    int totalSteps = (int) (end - start) / dt;
    T[0] = start;
    OUT[0] = init;

    for(int i=0; i<totalSteps; i++){
    	T[i+1] = T[i] + dt;
    	OUT[i+1] = euler_step(OUT[i], dt);
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
}


int main() {
	state_type v(N);
	v << 1, 2;
	cout << v << endl;

	state_type init(N);
	init << 1,0; // Initial conditions: theta, dtheta
	cout << init << endl;
	euler(init, 0, 10, .01);
	printOutput();
}