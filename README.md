# DC Motor & Pendulum System Model in C++

## Authors
- Andrew Chen: achen2001@g.ucla.edu
- Samuel Gessow: sgessow@gmail.com
- Dominic Insogna: dinsogna@seas.upenn.edu
- Gabriel Fernandez (Advisor): gabriel808@g.ucla.edu

## Description
C++ model of a three-phase (PMSM) brushless DC motor connected to a simple physical pendulum using the Runge-Kutta 4 integration method

## Setup and Dependencies
- C++ Version: 11
- External Libraries: Eigen 3
  - You must install the Eigen 3 C++ library and include in your build path to run this model
  - Download: http://eigen.tuxfamily.org/index.php?title=Main_Page#Download
  - Tutorial: http://eigen.tuxfamily.org/dox/GettingStarted.html

## Pendulum Equation
Motion of the pendulum is modeled using the 2nd order ODE: 

![Pend1](https://latex.codecogs.com/gif.latex?ml%5E2%20%5Cddot%5Ctheta%20%3D%20u%20-%20b%5Cdot%5Ctheta%20-mgl%5Csin%5Ctheta)

Where we define:
•	theta: angular position in radians
•	dtheta: first time derivative of theta
•	ddtheta: second time derivative of theta
•	u: external torque
•	b: damping constant
•	m: mass of pendulum
•	l: length of pendulum

## Motor Equation
The PMSM can be modeled using a system of 2nd Order ODEs representing the mechanical and electrical properties of the motor:

![Mot1](https://latex.codecogs.com/gif.latex?J%5Cddot%5Ctheta%20%3D%20-B%5Cdot%5Ctheta%20&plus;%20Ki_q%20-%20T_d)\
![Mot2](https://latex.codecogs.com/gif.latex?L%5Cfrac%7Bdi_q%7D%7Bdt%7D%20%3D%20-Ri_q%20-%20K%5Cdot%5Ctheta%20&plus;%20V_q%20-%20L%5Cdot%5Ctheta%20i_d)\
![Mot3](https://latex.codecogs.com/gif.latex?L%5Cfrac%7Bdi_d%7D%7Bdt%7D%20%3D%20-Ri_d%20&plus;%20V_d&plus;-%20L%5Cdot%5Ctheta%20i_q)

For simplification, we ignore the coupling terms and the third equation (heat generation), and get:

![Mot5](https://latex.codecogs.com/png.latex?J%5Cddot%5Ctheta%20%3D%20-B%5Cdot%5Ctheta%20&plus;%20Ki_q%20-%20T_d%20%5C%5C%20L%5Cfrac%7Bdi_q%7D%7Bdt%7D%20%3D%20-Ri_q%20-%20K%5Cdot%5Ctheta%20&plus;%20V_m)

## State Space System
We convert our systems of equations into state space form in order to discretely integrate using the Runge-Kutta method.

![SSPend](https://latex.codecogs.com/png.latex?%5Cfrac%7Bd%7D%7Bdt%7D%20%5Cbegin%7Bbmatrix%7D%20%5Ctheta%20%5C%5C%20%5Cdot%5Ctheta%20%5Cend%7Bbmatrix%7D%20%3D%20%5Cbegin%7Bbmatrix%7D%20%5Cdot%5Ctheta%20%5C%5C%20%5Cfrac%7Bu%7D%7Bml%5E2%7D%20-%20%5Cfrac%7Bb%5Cdot%5Ctheta%7D%7Bml%5E2%7D%20-%20%5Cfrac%7Bg%7D%7Bl%7D%5Csin%5Ctheta%20%5Cend%7Bbmatrix%7D)

![SSMot](https://latex.codecogs.com/gif.latex?%5Cfrac%7Bd%7D%7Bdt%7D%20%5Cbegin%7Bbmatrix%7D%20%5Cdot%5Ctheta%20%5C%5C%20i%20%5Cend%7Bbmatrix%7D%20%3D%20%5Cbegin%7Bbmatrix%7D%20%5Cfrac%7B-B%7D%7BJ%7D%20%26%20%5Cfrac%7BK%7D%7BJ%7D%5C%5C%20%5Cfrac%7B-K%7D%7BL%7D%20%26%20%5Cfrac%7B-R%7D%7BJ%7D%20%5Cend%7Bbmatrix%7D%20%5Cbegin%7Bbmatrix%7D%20%5Cdot%5Ctheta%20%5C%5C%20i%20%5Cend%7Bbmatrix%7D%20&plus;%20%5Cbegin%7Bbmatrix%7D%20%5Cfrac%7B-1%7D%7BJ%7D%20%26%200%5C%5C%200%20%26%20%5Cfrac%7B-1%7D%7BJ%7D%20%5Cend%7Bbmatrix%7D%20%5Cbegin%7Bbmatrix%7D%20T_d%5C%5C%20V_m%20%5Cend%7Bbmatrix%7D)

## The Runge-Kutta 4 Method
Pendulum Example:\
Initial Conditions:
```c++
double g= 9.81;
double l= 1.00;
double b= 0.2;
double m= 1.00;
double u= 3.00;
.
.
.
int main() {
	state_type init(N);
	init << 1,0; // Initial conditions: theta, dtheta
	rk4(init, 0, 10, .01);
	printOutput();
}
```
Output:
![ependrk4](https://github.com/dinsogna/3-Phase-Motor/blob/master/images/epend4k4.png)

DC Motor Example:\
Initial Conditions
```c++
double R = 1.00;   // Resistance
double L = 1.00;   // Inductance 
double K= 1.00;    // Motor Constant
double B = 0.00;   // Damping Factor
double J = 1.00;   // Inertia

double Td = 2.00; // External Load Torque
double Vm = 5.00; // Voltage to Motor
.
.
.
int main() {
	state_type init(N);
	init << 1,1; // Initial conditions: dtheta, i
	rk4(init, 0, 10, .01);
	printOutput();
}
```

![dcmotor_rk4](https://github.com/dinsogna/3-Phase-Motor/blob/master/images/dcmotor_rk4.png)


## Source File Description / Map
The C++ has four main classes:
- System: Superclass that includes Pendulum and Motor objects and their characteristics.
- Coupled: Couples the Pendulum and Motor objects.
- Controller: Implements controller logic
- Main: Sets initial conditions and runs model

## Running the Model
Step 1: Set the pendulum and motor properties in System class \
Step 2: Set initial conditions in main() \
Step 3: Run main() \
Step 4: Output is printed to console and/or output file \


## Static Testing
In a static state, such that angular acceleration and velocity are zero, our motor can be represented by:\
![Stat1](https://latex.codecogs.com/gif.latex?Ki_q%20%3D%20T_d) \
![Stat2](https://latex.codecogs.com/gif.latex?V_m%20%3D%20Ri_q)

Coupling the motor to the pendulum, we get:\
![Stat3](https://latex.codecogs.com/gif.latex?Ki_q%20%3D%20mgl%5Csin%5Ctheta) \
![Stat2](https://latex.codecogs.com/gif.latex?V_m%20%3D%20Ri_q)


## Dynamic Testing
For dynamic testing, we compared the output of our nonlinear model to a linearized closed form solution (using Matlab). We see that the models track well for small input voltages, but diverge with higher voltages.

[TBD] INCLUDE EQUATIONS\

Vm = 0.001 \
![output_step_0_001](https://github.com/dinsogna/3-Phase-Motor/blob/master/images/output_step_0_001.png) \

Vm = 0.025 \
![output_step_0_025](https://github.com/dinsogna/3-Phase-Motor/blob/master/images/output_step_0_025.png) \

Vm = 0.1 \
![output_step_0_1](https://github.com/dinsogna/3-Phase-Motor/blob/master/images/output_step_0_1.png) \

Vm = 0.2 \
![output_step_0_2](https://github.com/dinsogna/3-Phase-Motor/blob/master/images/output_step_0_2.png) \




## PID Controller








