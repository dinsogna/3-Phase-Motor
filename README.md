README
Three Phase Motor Model

Notes
-	C++ version
-	Graphs – control loops, dynamics equations – how they are connected, 
-	 Maybe illustration of the code structure
-	How to run test codes / comparisons / basic tests
-	Authors / Email address
-	Github Markup / Latex? For math

Description/
C++ model of a three-phase (PMSM) DC motor connected to a simple physical pendulum using the Runge-Kutta 4 integration method

Setup and Dependencies
•	You must install the Eigen 3 C++ library and include in your build path to run this model
•	Download: http://eigen.tuxfamily.org/index.php?title=Main_Page#Download
•	Tutorial: http://eigen.tuxfamily.org/dox/GettingStarted.html

Pendulum Equations
Motion of the pendulum is modeled using the 2nd order ODE: 

(m*l*l)*ddtheta = u – b*dtheta – (m*g*l)*sin(theta)

Where we define:
•	theta: angular position in radians
•	dtheta: first time derivative of theta
•	ddtheta: second time derivative of theta
•	u: external torque
•	b: damping constant
•	m: mass of pendulum
•	l: length of pendulum

We can rewrite this 2nd order ODE as a system of two 1st order ODEs as such:

dtheta = x
d[x]/dt = u / (m*l*l) – b*dtheta / (m*l*l) – (g/l)*sin(theta)

PMSM Equations
The PMSM can be modeled using a system of 2nd Order ODEs representing the mechanical and electrical dynamics of the motor:

J*ddtheta = -B*dtheta +K*i_q - T_d
L*d[i_q]/dt = -R*i_q – K*dtheta + V_q – L*dtheta* i_d
L*d[i_d]/dt = -R*i_d – K*dtheta + V_d – L*dtheta* i_q


Where we define:
•	theta: angular position in radians
•	dtheta: first time derivative of theta
•	ddtheta: second time derivative of theta
•	J: Inertia
•	L: Inductance
•	T_d: external torque
•	B: damping constant
•	R: Electrical Resistance
•	K: Motor constant
•	i_q: quadrature current
•	i_d: 

For our model, we ignore the coupling terms, and also ignore the id…[TBD]

Source File Descriptions / 
-	Pendulum.cpp
Simple Pendulum[TBD]

-	Motor.cpp
Three phase motor [TBD]

-	Coupler.cpp
Couples the pendulum to the motor. [TBD]

-	Main.cpp
[TBD]

Running the Model
[TBD]

Testing
[TBD]

