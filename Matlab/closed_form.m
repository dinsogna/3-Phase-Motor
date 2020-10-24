clc;clear all; close all;

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
B=[0,0,0;
    0,0,0;
    0,0,1/L];
C=[1,0,0];
D=0;
[b,a]=ss2tf(A,B,C,D)

num=b;
den=a;
FT = tf(num,den);                                           % Transfer Function Object
syms s t            ;                                        % Invoke Symbolic Math Toolbox
snum = poly2sym(num, s)  ;                                % Symbolic Numerator Polynomial
sden = poly2sym(den, s)  ;                                 % Symbolic Denominator Polynomial
%FT_time_domain = ilaplace(snum/sden)     ;                   % Inverse Laplace Transform
%FT_time_domain = simplify(FT_time_domain, 'Steps',10)      % Simplify To Get Nice Result
%FT_time_domain = collect(FT_time_domain, exp(-t))    % Optional Further Factorization
[y,t]=step(.3*FT,50);
plot(t,y);
hold on;
header1 = 'Time';
header2 = 'Theta';
fid=fopen('50_Step_0_3.txt','w');
fprintf(fid, [ header1 ' ' header2 '\n']);
fprintf(fid, '%f %f \n', [t y]');
fclose(fid);true
% TF_d=c2d(FT,1,'foh');
% [y_d,t_d]=step(.1*TF_d,60)
% plot(t_d,y_d,"--")
