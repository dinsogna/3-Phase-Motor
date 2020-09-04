# Three-Phase Motor & Pendulum System in C++

\*To Do:
-	C++ version
-	Graphs – control loops, dynamics equations – how they are connected, 
-	 Maybe illustration of the code structure
-	How to run test codes / comparisons / basic tests
-	Authors / Email address
-	Github Markup / Latex? For math
\*

**Authors**
- Andrew Chen: achen2001@g.ucla.edu
- Samuel Gessow: sgessow@gmail.com
- Dominic Insogna: dinsogna@seas.upenn.edu
- Gabriel Fernandez (Advisor): gabriel808@g.ucla.edu

**Description**\
C++ model of a three-phase (PMSM) DC motor connected to a simple physical pendulum using the Runge-Kutta 4 integration method

**Setup and Dependencies**
- C++ Version: 11
- External Libraries: Eigen 3
  - You must install the Eigen 3 C++ library and include in your build path to run this model
  - Download: http://eigen.tuxfamily.org/index.php?title=Main_Page#Download
  - Tutorial: http://eigen.tuxfamily.org/dox/GettingStarted.html

**Pendulum Equations**\
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

We can rewrite this 2nd order ODE as a system of two 1st order ODEs as such:

![Pend2](https://latex.codecogs.com/gif.latex?%5Cdot%5Ctheta%20%3D%20x)\
![Pend3](https://latex.codecogs.com/gif.latex?%5Cfrac%7Bdx%7D%7Bdt%7D%20%3D%20%5Cfrac%7Bu%7D%7Bml%5E2%7D%20-%20%5Cfrac%7Bb%5Cdot%5Ctheta%7D%7Bml%5E2%7D%20-%20%5Cfrac%7Bg%7D%7Bl%7D%5Csin%5Ctheta)

**PMSM Equations**\
The PMSM can be modeled using a system of 2nd Order ODEs representing the mechanical and electrical properties of the motor:

![Mot1](https://latex.codecogs.com/gif.latex?J%5Cddot%5Ctheta%20%3D%20-B%5Cdot%5Ctheta%20&plus;%20Ki_q%20-%20T_d)\
![Mot2](https://latex.codecogs.com/gif.latex?L%5Cfrac%7Bdi_q%7D%7Bdt%7D%20%3D%20-Ri_q%20-%20K%5Cdot%5Ctheta%20&plus;%20V_q%20-%20L%5Cdot%5Ctheta%20i_d)\
![Mot3](https://latex.codecogs.com/gif.latex?L%5Cfrac%7Bdi_d%7D%7Bdt%7D%20%3D%20-Ri_d%20&plus;%20V_d&plus;-%20L%5Cdot%5Ctheta%20i_q)

Where we define:
- theta: angular position in radians
- dtheta: first time derivative of theta
- ddtheta: second time derivative of theta
- J: Inertia
- L: Inductance
- T_d: external torque
- B: damping constant
- R: Electrical Resistance
- K: Motor constant
- i_q: quadrature current
- i_d: 

For our model, we ignore the coupling terms, and also ignore the id…[TBD]

**State Space System**\
We convert our systems of equations into state space form in order to discrete integrate using the Runge-Kutta method.

Pendulum:
![SSPend](https://latex.codecogs.com/gif.latex?%5Cfrac%7Bd%7D%7Bdt%7D%20%5Cbegin%7Bbmatrix%7D%20%5Ctheta%20%5C%5C%20x%20%5Cend%7Bbmatrix%7D%20%3D%20%5Cbegin%7Bbmatrix%7D%20x%20%5C%5C%20%5Cfrac%7Bu%7D%7Bml%5E2%7D%20-%20%5Cfrac%7Bb%5Cdot%5Ctheta%7D%7Bml%5E2%7D%20-%20%5Cfrac%7Bg%7D%7Bl%7D%5Csin%5Ctheta%20%5Cend%7Bbmatrix%7D)

Motor:
![SSMot](https://latex.codecogs.com/gif.latex?%5Cfrac%7Bd%7D%7Bdt%7D%20%5Cbegin%7Bbmatrix%7D%20%5Cdot%5Ctheta%20%5C%5C%20i%20%5Cend%7Bbmatrix%7D%20%3D%20%5Cbegin%7Bbmatrix%7D%20%5Cfrac%7B-B%7D%7BJ%7D%20%26%20%5Cfrac%7BK%7D%7BJ%7D%5C%5C%20%5Cfrac%7B-K%7D%7BL%7D%20%26%20%5Cfrac%7B-R%7D%7BJ%7D%20%5Cend%7Bbmatrix%7D%20%5Cbegin%7Bbmatrix%7D%20%5Cdot%5Ctheta%20%5C%5C%20i%20%5Cend%7Bbmatrix%7D%20&plus;%20%5Cbegin%7Bbmatrix%7D%20%5Cfrac%7B-1%7D%7BJ%7D%20%26%200%5C%5C%200%20%26%20%5Cfrac%7B-1%7D%7BJ%7D%20%5Cend%7Bbmatrix%7D%20%5Cbegin%7Bbmatrix%7D%20T_d%5C%5C%20V_m%20%5Cend%7Bbmatrix%7D)

**Source File Description / Map**\
-	Pendulum.cpp
Simple Pendulum[TBD]

-	Motor.cpp
Three phase motor [TBD]

-	Coupler.cpp
Couples the pendulum to the motor. [TBD]

-	Main.cpp
[TBD]

**Running the Model**\
[TBD]

**Custom RK4 vs ODEINT**\

**Static Testing**\
In a static state, such that 

**PID Controller**\


**Dynamic Testing**\

