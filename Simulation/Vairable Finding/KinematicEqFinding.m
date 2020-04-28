
clc
clear all

dt=0.02;
syms x y z
eqns = [x+y-2*z == 0,
        x+y+z == 1,
        2*y-z == -5];
[A,b] = equationsToMatrix(eqns)
vars = symvar(eqns)
