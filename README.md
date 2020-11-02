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

For discrete integration of the system, we use the Runge-Kutta 4 Method: https://en.wikipedia.org/wiki/Runge–Kutta_methods

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

Example: 

![Static-Test](https://lh6.googleusercontent.com/ZMCoguS-I42E5v7VdwFqnhNZE3gaBBjj15Srw1bcaTWSiESxxZ5J8ha40FUHNz3Gu-4VFmib2MBRyVJiSZqYu2Bt4q4Jh_cOJngOf9Q)


## Dynamic Testing - Linearized Model
We linearized our model and compared the output to a linearized closed form solution (using Matlab). We see that the models track well all target thetas.

Target Theta = .05 radians \
![Dynamic-Linear-Test1](https://lh5.googleusercontent.com/6_rz4pLFEqE-kImjueMaSRFqXLYjIKU5D7bzcG5TV8swZXEGTIE_sd6Bnh_Bt3M5-oOvq29l1O5kOE4eUJ3cweOJXhoqWcfztHrl1HV_)

Target Theta = .3 radians \
![Dynamic-Linear-Test3](https://lh6.googleusercontent.com/zqULvk_F5id_YL1WguthgIoGfkU7S14OPsSNm0tWe0I8kVSouqqbCoFwVh4d6HhXE6-486YUBuDjq72U_WX2SriSEujTsAlV_I8qIEiV)

## Dynamic Testing - Nonlinear Model
We compared our nonlinear model to a linearized closed form solution (using Matlab). We see that the models track well for small thetas, but diverges for larger angles.

Target Theta = .05 radians \
![Dynamic-Nonlinear-Test1](https://lh5.googleusercontent.com/-EFRhzWC2p0ZYLYThZkSSjvvZF-aGPS5Y2B4XPDkCya6xkRM7KCIZS_Y_D7la2tUkomzuJJKAsX1Vh6mQuMTyV5VeGDFRbwCeuoed8Vw)

Target Theta = .3 radians \
![Dynamic-Nonlinear-Test3](https://lh4.googleusercontent.com/7sl9BJlY3AloX_oqvUtdueLXSRfyiiIhGE35gLVtBKP-CXIEY0Y4fgsceI3YCPf6rSL-8xv_JSi4A7leSc12kIi1ce3JZhRJokPdJMVX)


## PID Controller

3 Types of Controllers are implemented:
- Current Controller
- Direct Force Controller (Position)
- Velocity Controller \

NOTE: Controller object is implemented into the Motor class which means the Motor Object can use the controllers by itself as well








