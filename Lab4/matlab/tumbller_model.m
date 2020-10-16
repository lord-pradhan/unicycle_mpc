function [A,B,C,D] = tumbller_model()

%% Robot parameters
mcart = 0.493;
mpend = 0.312;
Ipend = 0.00024;
L = 0.04;
f = 0.01;
kt = 0.11;
R = 10;
r  = 0.0335;
g = 9.81;

%% Linear Dynamics model 
A22 = -(Ipend + mpend*L^2)*f/(Ipend*(mcart + mpend) + mcart*mpend*L^2);
A23 = (mpend^2*g*L^2)/(Ipend*(mcart + mpend) + mcart*mpend*L^2);
A42 = -mpend*L*f / (Ipend*(mcart + mpend) + mcart*mpend*L^2);
A43 = mpend*g*L*(mcart + mpend) / (Ipend*(mcart + mpend) + mcart*mpend*L^2);

A = [   0,  1,   0,   0;...
        0,  A22,    A23,    0;...
        0,  0,  0,  1;...
        0,  A42, A43, 0];

B12 = (Ipend + mpend*L^2)/(Ipend*(mcart+mpend) + mcart*mpend*L^2);
B14 = mpend*L/(Ipend*(mcart + mpend) + mcart*mpend*L^2);
B = [0; B12; B14; 0];

%% Output 
C = [   1, 0, 0, 0;...
        0, 0, 1, 0];

D = [0;0];
end

