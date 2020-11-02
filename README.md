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


## PID Controllers

3 Types of Controllers are implemented:
- Current Controller
- Direct Force Controller (Position)
- Velocity Controller \

NOTE: Controller object is implemented into the Motor class which means the Motor Object can use the controllers by itself as well

## Current Control

![Current-Block](https://lh4.googleusercontent.com/ThcV6LKUb4RncDMt8zfWv21-IXhpASJQYsUCP5crXS2pRuMW0XkwY74KkxdPQr5Foq3ujaajKej9NhuCbRrpjDY7t9-R6YNPVA8Y1_XLc_nWy-837tY4wGgx1TeuT5gGDgNixBDq7As)

Example with gains:
- P = 0.07
- I = 0.02
- D = 0
- Target current = 0.1A

![Current-Graph](https://lh4.googleusercontent.com/3r9FkyBXx2piluFJjRb6LvMQNKPNRiYUiIgSIZuwYMBpwzgEUKYukwbmOeGXyHbNNVDUBos-aWgvQGTYitV2D50ni5aMWyMgRTbTHsOsI0C_s0s6hn5wavBdfRRz0ZGe4sz0TJ5gwKk)

## Direct Force Control (Position Control)

![DF-Block](https://lh5.googleusercontent.com/bjCzKmLPfc48_pTrsYphNCnxQlZ7TnE3CM7y5FmZpNwDeRmoYP9TacBbtaUCtAGhd-ZeWLGeGNSWYFMz5CQ5JOAmABkSO-8ygHswC6qugO_BJQ_0WqWNywzBi408u8wbJI-uQE70fWE)

Example with gains:
- P = 300
- I = 20
- D = 1.3
- Target position = 0.1 radians

![DF-Graph](https://lh5.googleusercontent.com/M8V6_fsfIRaBB3f0dPTOm4xeGIalILcE3-c7gJYFIMdDWf2kQ9oh0g3X6VcD2nSqmxDoIe0lWaLOXXyIOgOWjUbSxbrmf7Gpn-8DOD2NuCiTVvWu2YLI1B1FcBV-BBKY8xDtt2LCC4k)

## Velocity Control

![Velocity-Block](https://lh3.googleusercontent.com/dJTENk6kOCVQkvCllq6utwWWp6G8TFQUImiB-6hQDNnck_JKCECAd3j2g8TfrZdGNddkJK5oPdf1svxlBkCvzJB5I2wETp53BWancj81lSHDzZm-Z_VrYjuQ2sDs7AQ3TQ0xf2tTjm8)

Example with gains:
- P = 100
- I = 1
- D = 0
- Target velocity = radians/s

![Velocity-Graph](https://lh6.googleusercontent.com/ux4sEevQeI4DdUZxv8qvQIAtlLPXLOMpM3OKEqzPA-5N7D6ZgGtidmKhixs0xSS9UQFM9ApXm4t7eajNSMyAnY1z2a4cqsA3H5sgjWYpMk0g1r6uK1YNl6TevVXce-ND9fkm92SXpbY)

## Closed Loop Test - Step Response

Target = .1 radians \
![SR1](https://lh6.googleusercontent.com/xOBeIUrqgFUxi6SdgLUIRn_qnj8DhbdZeu6C4yFe-G5lGAID4sLn5PP8w-72vJeEL18WdoMwj0HA0p1grkS3awTbEpyVfXT02oDuK_q3Q6pOcjlW2fAWjenBHRjWKUI0l2PgjGONAC4)

Target = .2 radians \
![SR2](https://lh6.googleusercontent.com/QvPU6GMYqmQeCaCAG8KmM1ui4TO0mft-sSZyaN0YeCx2B-jVe-1op1rlJ3TtU8bS8LltI7eUDs0EP1LGmw9KCTnBMzTGseAnFpjNVQe8pFMA2nJOrJOq5LIyxYfjOuoqQVHxxPj0ukM)

Target = .3 radians \
![SR3](https://lh5.googleusercontent.com/w7Z8_u2QOSddqKwrfWeOBS4kzXXeqAh6qnzLvlaNx_BOpCG-5mKGaxJTqa_qBXJn7GJ1EGWt9q05aXHwpA4a4nAV2gAO1caXBrQCPcOnMiTxx-4vLEYgJfsanSrOHyRPXhj31VSyJc8)

## Closed Loop Test - Sine Wave Position Test

5Hz \
![SW1](https://lh4.googleusercontent.com/SZJYTnnwwTNr8mBWm71ie_WsUkoKrX6-_1F3XzsAjn47eDvRjYVdjXUd0_46QlpZUNEYT-Su-Ws1YKnrklq1ptGD4fLKhldiObeRxwvw)

10Hz \
![SW2](https://lh3.googleusercontent.com/ycPtGPuwBfesi3yL6arGJQ5oocu75MXk9h8p_J2rZ0Yd7ogddXgQQ3mEu_7UULjg9McHr3CZdarathYLF-1SJa8Ob3KIZbacPZW5XIXe-JZx4-W4ugwczcMF54QT-WBPchBpjphaDEI)

15 Hz \
![SW3](https://lh4.googleusercontent.com/auqHYz7Gx1H_OYmJ_zTm-rN1f8naODRApNDBJW3Jz8MwDzNQj_wQMpNNl1rFZqpwBStrqNnCj8Neq_IThVYkvuXJZoHaNdSjmbmc06_PtpJVMl26NozB_ArywJ1de8QilcDu2GTTVnQ)









