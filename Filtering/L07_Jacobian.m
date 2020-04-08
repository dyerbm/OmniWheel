%% Calculates Jacobian matrix (linearized A) for L07
clear; % Clears workspace
close all; % Closes all figures, etc.
clc; % Clears the screen
syms x1 x2 x3 x4 u; % Sets up the states, input as symbols
syms T A Dp M V0 Beta L Ql a1 a2 a3; % More symolics...
f1 = x1 + T*x2; % First function
f2 = x2 + T*x3; % Second function
f3 = (1 - T*((a2*V0+M*Beta*L)/(M*V0)))*x3 - T*((A^2*Beta+a2*L*Beta)/(M*V0))*x2 - T/(M*V0)*(2*a1*V0*x2*x3+Beta*L*(a1*x2^2+a3))*sign(x2) + T*((A*Beta)/(M*V0))*u; % Third function
f4 = (1/A)*(a2*x2 + (a1*x2^2+a3)*sign(x2)); % Fourth function
F = [diff(f1,x1) diff(f1,x2) diff(f1,x3) diff(f1,x4); % Definition for 4x4 Jacobian
    diff(f2,x1) diff(f2,x2) diff(f2,x3) diff(f2,x4);
    diff(f3,x1) diff(f3,x2) diff(f3,x3) diff(f3,x4);
    diff(f4,x1) diff(f4,x2) diff(f4,x3) diff(f4,x4)];
F = simplify(F); % Simplifies the symbolic form of F