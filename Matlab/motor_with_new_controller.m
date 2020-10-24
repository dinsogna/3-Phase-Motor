clc;clear; close all;
m=.4;
l=.37;
g=9.81;
B_p=.07;


K=0.088;
R=0.16;
L=1.8e-4;
B_m=1e-3;
J=1e-4;

A=[0,1,0;-1*m*g*l/(10*(10*J+m*l^2/10)),-(10*B_m+B_p/10)/(10*J+m*l^2/10),K/(10*J+m*l^2/10);0,-10*K/L,-R/L];
B=[1,0,0;
    0,1,0;
    0,0,1/L];
C=eye(3);
D=zeros(3);
S_ss=ss(A,B,C,D);
S=tf(S_ss);
S.InputName={'T','u','V'};
S.OutputName={'t_dot','I','theta'};
%% old method did that not work take 2 above
% %% Constants
% %pendulum
% m=.4;
% l=.37;
% g=9.81;
% B_p=.07;
% 
% %motor
% K=0.088;
% R=0.16;
% L=1.8e-4;
% B_m=1e-3;
% J=1e-4;
% %% Motor
% % state space
% Am_m=[-B_m/J,K/J;
%     -K/L,-R/L];
% Bm_m=[-1/J,1;
%     0,1/L];
% Cm_m=eye(2);
% Dm_m=zeros(2);
% motor_ss=ss(Am_m,Bm_m,Cm_m,Dm_m);
% % transfer function, 2 input, 2 output
% M=tf(motor_ss);
% M.InputName={'u','V'};
% M.OutputName={'t_dot','I'};
% % make block for output torque
% S = sumblk('Tm =')
% %% Pendulum
% % transfer function of linearized pendulum
% P=tf([0,0,1],[m*l^2,1,m*g*l]);

