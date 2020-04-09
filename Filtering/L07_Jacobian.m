%% Calculates Jacobian matrix (linearized A) for L07
clear; % Clears workspace
close all; % Closes all figures, etc.
clc; % Clears the screen
syms x y t u1 u2 u3 u4; % Sets up the states, input as symbols
syms wr rr vs1 vs2 vs3 vs4 T; % More symolics...
Binv = [(-cos(t)-sin(t))/(2*sqrt(2)), (-cos(t)+sin(t))/(2*sqrt(2)), (cos(t)+sin(t))/(2*sqrt(2)), (cos(t)-sin(t))/(2*sqrt(2));
    (cos(t)-sin(t))/(2*sqrt(2)), (-cos(t)-sin(t))/(2*sqrt(2)), (-cos(t)+sin(t))/(2*sqrt(2)), (cos(t)+sin(t))/(2*sqrt(2));
    1/(4*rr), 1/(4*rr), 1/(4*rr), 1/(4*rr)];
u=[u1;u2;u3;u4];
vs=[vs1;vs2;vs3;vs4];

dyn = Binv*(wr*u+vs)*T+[x;y;t];
f1=dyn(1);
f2=dyn(2);
f3=dyn(3);

F = [diff(f1,x) diff(f1,y) diff(f1,t); % Definition for 4x4 Jacobian
    diff(f2,x) diff(f2,y) diff(f2,t);
    diff(f3,x) diff(f3,y) diff(f3,t)];
F = simplify(F); % Simplifies the symbolic form of F