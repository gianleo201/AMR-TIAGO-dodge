clc;
clear all;
close all;

Ts = 0.01;
EXPORT = 1;

VelocityDamper = 0; %put 1 if you want the velocity damper constraint, 0 if you want the CBF-based one

BEGIN_ACADO; % Always start with "BEGIN_ACADO".

DifferentialState x1 x2 x3 x4 x5 x6;
Control tau1 tau2 tau3;
OnlineData x_obs y_obs r_obs x01 x02 x03 x04 x05 x06;

n_XD = length(diffStates);
n_U = length(controls);

%% Differential Equation

% % Differential Equation (built from matlab function)
% f = acado.DifferentialEquation(); % Set the differential equation object
% f.linkMatlabODE('TIAGO'); % Link to a Matlab ODE

% gravity acceleration
g0 = 9.81;

% set up parameters of the robot

% torso and base
hb = 0.193;
rt1 = 0.062;
rt2 = 0.159;
rt3 = 0.02435;

m1 = 27;
ht = 0.597+0.2;
rt4 = 0.155;

% arm1 (link 2)
m2 = 2.08+1.79+2.28;
l2 = 0.32;
I2zz = 0.06;

% arm2 (link 3)
m3 = 1.89+1.07+0.2+0.6+0.2;
l3 = 0.34+0.25;
I3zz = 0.006;

% Differential equations
f = dot([x1;x2;x3;x4;x5;x6]) == [
    x4;
    x5;
    x6;
    -(4*I3zz*l2^3*m2^2*x5^2*sin(x2) - 4*l2^2*l3^2*m3^2*tau1 - 8*I3zz*l2^2*m2*tau1 - 8*I2zz*l3^2*m3*tau1 - 32*I3zz*l2^2*m3*tau1 - 32*I2zz*I3zz*tau1 + 32*I3zz*l2^3*m3^2*x5^2*sin(x2) - 4*I2zz*g0*l3^2*m3^2*sin(2*x2 + 2*x3) - 2*l2^2*l3^2*m2*m3*tau1 + 4*l2^2*l3^2*m3^2*tau1*cos(2*x3) - 8*l2^2*l3*m3^2*tau3*cos(x2 + x3) - 16*I2zz*l3*m3*tau3*cos(x2 + x3) - 4*l2*l3^2*m3^2*tau2*cos(x2) + 4*l2*l3^2*m3^2*tau3*cos(x2) - 16*I3zz*l2*m2*tau2*cos(x2) + 16*I3zz*l2*m2*tau3*cos(x2) - 32*I3zz*l2*m3*tau2*cos(x2) + 32*I3zz*l2*m3*tau3*cos(x2) + 4*I2zz*l3^3*m3^2*x5^2*sin(x2 + x3) + 4*I2zz*l3^3*m3^2*x6^2*sin(x2 + x3) + 4*l2*l3^2*m3^2*tau2*cos(x2 + 2*x3) + 8*l2^2*l3*m3^2*tau3*cos(x2 - x3) - 4*l2*l3^2*m3^2*tau3*cos(x2 + 2*x3) - 4*I3zz*g0*l2^2*m2^2*sin(2*x2) - 16*I3zz*g0*l2^2*m3^2*sin(2*x2) + 24*I3zz*l2^3*m2*m3*x5^2*sin(x2) + g0*l2^2*l3^2*m2*m3^2*sin(2*x2 + 2*x3) + l2^2*l3^3*m2*m3^2*x5^2*sin(x2 - x3) + l2^2*l3^3*m2*m3^2*x6^2*sin(x2 - x3) - l2^3*l3^2*m2*m3^2*x5^2*sin(x2 + 2*x3) + 8*I3zz*l2^2*l3*m3^2*x5^2*sin(x2 + x3) + 8*I3zz*l2^2*l3*m3^2*x6^2*sin(x2 + x3) + 16*I2zz*I3zz*l3*m3*x5^2*sin(x2 + x3) + 16*I2zz*I3zz*l3*m3*x6^2*sin(x2 + x3) + 4*I2zz*l2*l3^2*m3^2*x5^2*sin(x2) + 16*I2zz*I3zz*l2*m2*x5^2*sin(x2) + 32*I2zz*I3zz*l2*m3*x5^2*sin(x2) + 4*I2zz*l2*l3^2*m3^2*x5^2*sin(x2 + 2*x3) + 8*I3zz*l2^2*l3*m3^2*x5^2*sin(x2 - x3) + 8*I3zz*l2^2*l3*m3^2*x6^2*sin(x2 - x3) - 4*l2*l3^2*m2*m3*tau2*cos(x2) + 4*l2*l3^2*m2*m3*tau3*cos(x2) - 2*g0*l2^2*l3^2*m2*m3^2*sin(2*x2) - g0*l2^2*l3^2*m2^2*m3*sin(2*x2) + 3*l2^3*l3^2*m2*m3^2*x5^2*sin(x2) + l2^3*l3^2*m2^2*m3*x5^2*sin(x2) + 8*I2zz*l3^3*m3^2*x5*x6*sin(x2 + x3) + 4*l2^2*l3*m2*m3*tau3*cos(x2 - x3) - 16*I3zz*g0*l2^2*m2*m3*sin(2*x2) + 2*l2^2*l3^3*m2*m3^2*x5*x6*sin(x2 - x3) + 16*I3zz*l2^2*l3*m3^2*x5*x6*sin(x2 + x3) + 4*I2zz*l2*l3^2*m2*m3*x5^2*sin(x2) + 32*I2zz*I3zz*l3*m3*x5*x6*sin(x2 + x3) + 4*I3zz*l2^2*l3*m2*m3*x5^2*sin(x2 - x3) + 4*I3zz*l2^2*l3*m2*m3*x6^2*sin(x2 - x3) + 16*I3zz*l2^2*l3*m3^2*x5*x6*sin(x2 - x3) + 8*I3zz*l2^2*l3*m2*m3*x5*x6*sin(x2 - x3))/(4*I3zz*l2^2*m2^2 + 4*I2zz*l3^2*m3^2 + 16*I3zz*l2^2*m3^2 + 32*I2zz*I3zz*m1 + 32*I2zz*I3zz*m2 + 32*I2zz*I3zz*m3 + 4*l2^2*l3^2*m1*m3^2 + 3*l2^2*l3^2*m2*m3^2 + l2^2*l3^2*m2^2*m3 + 8*I3zz*l2^2*m1*m2 + 8*I2zz*l3^2*m1*m3 + 32*I3zz*l2^2*m1*m3 + 8*I2zz*l3^2*m2*m3 + 24*I3zz*l2^2*m2*m3 - 4*I3zz*l2^2*m2^2*cos(2*x2) - 16*I3zz*l2^2*m3^2*cos(2*x2) - 4*I2zz*l3^2*m3^2*cos(2*x2 + 2*x3) + 2*l2^2*l3^2*m1*m2*m3 - 4*l2^2*l3^2*m1*m3^2*cos(2*x3) - 2*l2^2*l3^2*m2*m3^2*cos(2*x2) - l2^2*l3^2*m2^2*m3*cos(2*x2) - 2*l2^2*l3^2*m2*m3^2*cos(2*x3) - 16*I3zz*l2^2*m2*m3*cos(2*x2) + l2^2*l3^2*m2*m3^2*cos(2*x2 + 2*x3));
    (2*(2*l3^2*m3^2*tau2 - 2*l3^2*m3^2*tau3 + 16*I3zz*m1*tau2 - 16*I3zz*m1*tau3 + 16*I3zz*m2*tau2 - 16*I3zz*m2*tau3 + 16*I3zz*m3*tau2 - 16*I3zz*m3*tau3 + 4*l3^2*m1*m3*tau2 - 4*l3^2*m1*m3*tau3 + 4*l3^2*m2*m3*tau2 - 4*l3^2*m2*m3*tau3 - 2*l3^2*m3^2*tau2*cos(2*x2 + 2*x3) + 2*l3^2*m3^2*tau3*cos(2*x2 + 2*x3) + 8*I3zz*g0*l2*m2^2*sin(x2) + 16*I3zz*g0*l2*m3^2*sin(x2) - 4*l2*l3*m3^2*tau3*cos(x3) - 2*I3zz*l2^2*m2^2*x5^2*sin(2*x2) - 8*I3zz*l2^2*m3^2*x5^2*sin(2*x2) + 4*l2*l3*m3^2*tau3*cos(2*x2 + x3) + 2*l2*l3^2*m3^2*tau1*cos(x2) + 8*I3zz*l2*m2*tau1*cos(x2) + 16*I3zz*l2*m3*tau1*cos(x2) - 2*l2*l3^2*m3^2*tau1*cos(x2 + 2*x3) + 4*I3zz*l2*l3*m3^2*x5^2*sin(x3) + 4*I3zz*l2*l3*m3^2*x6^2*sin(x3) + 2*g0*l2*l3^2*m1*m3^2*sin(x2) + 3*g0*l2*l3^2*m2*m3^2*sin(x2) + 2*g0*l2*l3^2*m2^2*m3*sin(x2) + 8*I3zz*g0*l2*m1*m2*sin(x2) + 16*I3zz*g0*l2*m1*m3*sin(x2) + 24*I3zz*g0*l2*m2*m3*sin(x2) - 4*I3zz*l2*l3*m3^2*x5^2*sin(2*x2 + x3) - 4*I3zz*l2*l3*m3^2*x6^2*sin(2*x2 + x3) + 2*l2^2*l3^2*m1*m3^2*x5^2*sin(2*x3) - l2^2*l3^2*m2*m3^2*x5^2*sin(2*x2) - (l2^2*l3^2*m2^2*m3*x5^2*sin(2*x2))/2 + l2^2*l3^2*m2*m3^2*x5^2*sin(2*x3) - 2*g0*l2*l3^2*m1*m3^2*sin(x2 + 2*x3) - g0*l2*l3^2*m2*m3^2*sin(x2 + 2*x3) - 8*l2*l3*m1*m3*tau3*cos(x3) - 6*l2*l3*m2*m3*tau3*cos(x3) - 8*I3zz*l2^2*m2*m3*x5^2*sin(2*x2) + 2*l2*l3^3*m1*m3^2*x5^2*sin(x3) + 2*l2*l3^3*m1*m3^2*x6^2*sin(x3) + (3*l2*l3^3*m2*m3^2*x5^2*sin(x3))/2 + (3*l2*l3^3*m2*m3^2*x6^2*sin(x3))/2 + 2*l2*l3*m2*m3*tau3*cos(2*x2 + x3) + 2*l2*l3^2*m2*m3*tau1*cos(x2) - (l2*l3^3*m2*m3^2*x5^2*sin(2*x2 + x3))/2 - (l2*l3^3*m2*m3^2*x6^2*sin(2*x2 + x3))/2 + 8*I3zz*l2*l3*m1*m3*x5^2*sin(x3) + 8*I3zz*l2*l3*m1*m3*x6^2*sin(x3) + 6*I3zz*l2*l3*m2*m3*x5^2*sin(x3) + 6*I3zz*l2*l3*m2*m3*x6^2*sin(x3) + 8*I3zz*l2*l3*m3^2*x5*x6*sin(x3) + 2*g0*l2*l3^2*m1*m2*m3*sin(x2) - 2*I3zz*l2*l3*m2*m3*x5^2*sin(2*x2 + x3) - 2*I3zz*l2*l3*m2*m3*x6^2*sin(2*x2 + x3) - 8*I3zz*l2*l3*m3^2*x5*x6*sin(2*x2 + x3) + 4*l2*l3^3*m1*m3^2*x5*x6*sin(x3) + 3*l2*l3^3*m2*m3^2*x5*x6*sin(x3) - l2*l3^3*m2*m3^2*x5*x6*sin(2*x2 + x3) + 16*I3zz*l2*l3*m1*m3*x5*x6*sin(x3) + 12*I3zz*l2*l3*m2*m3*x5*x6*sin(x3) - 4*I3zz*l2*l3*m2*m3*x5*x6*sin(2*x2 + x3)))/(4*I3zz*l2^2*m2^2 + 4*I2zz*l3^2*m3^2 + 16*I3zz*l2^2*m3^2 + 32*I2zz*I3zz*m1 + 32*I2zz*I3zz*m2 + 32*I2zz*I3zz*m3 + 4*l2^2*l3^2*m1*m3^2 + 3*l2^2*l3^2*m2*m3^2 + l2^2*l3^2*m2^2*m3 + 8*I3zz*l2^2*m1*m2 + 8*I2zz*l3^2*m1*m3 + 32*I3zz*l2^2*m1*m3 + 8*I2zz*l3^2*m2*m3 + 24*I3zz*l2^2*m2*m3 - 4*I3zz*l2^2*m2^2*cos(2*x2) - 16*I3zz*l2^2*m3^2*cos(2*x2) - 4*I2zz*l3^2*m3^2*cos(2*x2 + 2*x3) + 2*l2^2*l3^2*m1*m2*m3 - 4*l2^2*l3^2*m1*m3^2*cos(2*x3) - 2*l2^2*l3^2*m2*m3^2*cos(2*x2) - l2^2*l3^2*m2^2*m3*cos(2*x2) - 2*l2^2*l3^2*m2*m3^2*cos(2*x3) - 16*I3zz*l2^2*m2*m3*cos(2*x2) + l2^2*l3^2*m2*m3^2*cos(2*x2 + 2*x3));
    (4*(tau3 + (g0*l3*m3*sin(x2 + x3))/2 - (l2*l3*m3*x5^2*sin(x3))/2)*(8*I2zz*m1 + 8*I2zz*m2 + 8*I3zz*m1 + 8*I2zz*m3 + 8*I3zz*m2 + 8*I3zz*m3 + l2^2*m2^2 + 4*l2^2*m3^2 + l3^2*m3^2 + 2*l2^2*m1*m2 + 8*l2^2*m1*m3 + 6*l2^2*m2*m3 + 2*l3^2*m1*m3 + 2*l3^2*m2*m3 - l2^2*m2^2*cos(2*x2) - 4*l2^2*m3^2*cos(2*x2) - l3^2*m3^2*cos(2*x2 + 2*x3) - 4*l2*l3*m3^2*cos(2*x2 + x3) - 4*l2^2*m2*m3*cos(2*x2) + 4*l2*l3*m3^2*cos(x3) - 2*l2*l3*m2*m3*cos(2*x2 + x3) + 8*l2*l3*m1*m3*cos(x3) + 6*l2*l3*m2*m3*cos(x3)))/(4*I3zz*l2^2*m2^2 + 4*I2zz*l3^2*m3^2 + 16*I3zz*l2^2*m3^2 + 32*I2zz*I3zz*m1 + 32*I2zz*I3zz*m2 + 32*I2zz*I3zz*m3 + 4*l2^2*l3^2*m1*m3^2 + 3*l2^2*l3^2*m2*m3^2 + l2^2*l3^2*m2^2*m3 + 8*I3zz*l2^2*m1*m2 + 8*I2zz*l3^2*m1*m3 + 32*I3zz*l2^2*m1*m3 + 8*I2zz*l3^2*m2*m3 + 24*I3zz*l2^2*m2*m3 - 4*I3zz*l2^2*m2^2*cos(2*x2) - 16*I3zz*l2^2*m3^2*cos(2*x2) - 4*I2zz*l3^2*m3^2*cos(2*x2 + 2*x3) + 2*l2^2*l3^2*m1*m2*m3 - 4*l2^2*l3^2*m1*m3^2*cos(2*x3) - 2*l2^2*l3^2*m2*m3^2*cos(2*x2) - l2^2*l3^2*m2^2*m3*cos(2*x2) - 2*l2^2*l3^2*m2*m3^2*cos(2*x3) - 16*I3zz*l2^2*m2*m3*cos(2*x2) + l2^2*l3^2*m2*m3^2*cos(2*x2 + 2*x3)) - (4*(tau2 + g0*m3*((l3*sin(x2 + x3))/2 + l2*sin(x2)) + (g0*l2*m2*sin(x2))/2 + (l2*l3*m3*x6*sin(x3)*(2*x5 + x6))/2)*(8*I3zz*m1 + 8*I3zz*m2 + 8*I3zz*m3 + l3^2*m3^2 + 2*l3^2*m1*m3 + 2*l3^2*m2*m3 - l3^2*m3^2*cos(2*x2 + 2*x3) - 2*l2*l3*m3^2*cos(2*x2 + x3) + 2*l2*l3*m3^2*cos(x3) - l2*l3*m2*m3*cos(2*x2 + x3) + 4*l2*l3*m1*m3*cos(x3) + 3*l2*l3*m2*m3*cos(x3)))/(4*I3zz*l2^2*m2^2 + 4*I2zz*l3^2*m3^2 + 16*I3zz*l2^2*m3^2 + 32*I2zz*I3zz*m1 + 32*I2zz*I3zz*m2 + 32*I2zz*I3zz*m3 + 4*l2^2*l3^2*m1*m3^2 + 3*l2^2*l3^2*m2*m3^2 + l2^2*l3^2*m2^2*m3 + 8*I3zz*l2^2*m1*m2 + 8*I2zz*l3^2*m1*m3 + 32*I3zz*l2^2*m1*m3 + 8*I2zz*l3^2*m2*m3 + 24*I3zz*l2^2*m2*m3 - 4*I3zz*l2^2*m2^2*cos(2*x2) - 16*I3zz*l2^2*m3^2*cos(2*x2) - 4*I2zz*l3^2*m3^2*cos(2*x2 + 2*x3) + 2*l2^2*l3^2*m1*m2*m3 - 4*l2^2*l3^2*m1*m3^2*cos(2*x3) - 2*l2^2*l3^2*m2*m3^2*cos(2*x2) - l2^2*l3^2*m2^2*m3*cos(2*x2) - 2*l2^2*l3^2*m2*m3^2*cos(2*x3) - 16*I3zz*l2^2*m2*m3*cos(2*x2) + l2^2*l3^2*m2*m3^2*cos(2*x2 + 2*x3)) + (4*((l3*m3*x5^2*sin(x2 + x3))/2 - tau1 + (l3*m3*x6^2*sin(x2 + x3))/2 + (l2*m2*x5^2*sin(x2))/2 + l2*m3*x5^2*sin(x2) + l3*m3*x5*x6*sin(x2 + x3))*(l2*l3^2*m3^2*cos(x2) - 4*I2zz*l3*m3*cos(x2 + x3) - 2*l2^2*l3*m3^2*cos(x2 + x3) + 4*I3zz*l2*m2*cos(x2) + 8*I3zz*l2*m3*cos(x2) + 2*l2^2*l3*m3^2*cos(x2 - x3) - l2*l3^2*m3^2*cos(x2 + 2*x3) + l2*l3^2*m2*m3*cos(x2) + l2^2*l3*m2*m3*cos(x2 - x3)))/(4*I3zz*l2^2*m2^2 + 4*I2zz*l3^2*m3^2 + 16*I3zz*l2^2*m3^2 + 32*I2zz*I3zz*m1 + 32*I2zz*I3zz*m2 + 32*I2zz*I3zz*m3 + 4*l2^2*l3^2*m1*m3^2 + 3*l2^2*l3^2*m2*m3^2 + l2^2*l3^2*m2^2*m3 + 8*I3zz*l2^2*m1*m2 + 8*I2zz*l3^2*m1*m3 + 32*I3zz*l2^2*m1*m3 + 8*I2zz*l3^2*m2*m3 + 24*I3zz*l2^2*m2*m3 - 4*I3zz*l2^2*m2^2*cos(2*x2) - 16*I3zz*l2^2*m3^2*cos(2*x2) - 4*I2zz*l3^2*m3^2*cos(2*x2 + 2*x3) + 2*l2^2*l3^2*m1*m2*m3 - 4*l2^2*l3^2*m1*m3^2*cos(2*x3) - 2*l2^2*l3^2*m2*m3^2*cos(2*x2) - l2^2*l3^2*m2^2*m3*cos(2*x2) - 2*l2^2*l3^2*m2*m3^2*cos(2*x3) - 16*I3zz*l2^2*m2*m3*cos(2*x2) + l2^2*l3^2*m2*m3^2*cos(2*x2 + 2*x3))
    ];
    
% output functions: E-E position + E-E velocity + joint velocities + control inputs
h = { x1 - rt1 + rt4 - l3*sin(x2 + x3) - l2*sin(x2), hb + ht + l3*cos(x2 + x3) + l2*cos(x2), x4 - x5*(l3*cos(x2 + x3) + l2*cos(x2)) - l3*x6*cos(x2 + x3), - x5*(l3*sin(x2 + x3) + l2*sin(x2)) - l3*x6*sin(x2 + x3), x4, x5, x6, tau1, tau2, tau3};
hN = { x1 - rt1 + rt4 - l3*sin(x2 + x3) - l2*sin(x2), hb + ht + l3*cos(x2 + x3) + l2*cos(x2), x4 - x5*(l3*cos(x2 + x3) + l2*cos(x2)) - l3*x6*cos(x2 + x3), - x5*(l3*sin(x2 + x3) + l2*sin(x2)) - l3*x6*sin(x2 + x3), x4, x5, x6};


%% SIMexport
acadoSet('problemname', 'sim');

numSteps = 5;

sim = acado.SIMexport( Ts );

sim.setModel(f);  % set the ODE to be integrated


% set some option for the integration
sim.set( 'INTEGRATOR_TYPE',             'INT_IRK_RIIA5' );
sim.set( 'NUM_INTEGRATOR_STEPS',        numSteps        );

if EXPORT
    sim.exportCode( 'export_SIM' );
    
    cd export_SIM
    make_acado_integrator('../integrate_tiago')
    cd ..
end

%% MPCexport
acadoSet('problemname', 'mpc');

N = 15;
ocp = acado.OCP( 0.0, N*Ts, N );

% set eye matrices to default values: real values will be set before
% simulation
W_mat = eye(10,10);
W = acado.BMatrix(W_mat);
WN_mat = eye(7,7);
WN = acado.BMatrix(WN_mat);

ocp.minimizeLSQ( W, h );   % set objective function
ocp.minimizeLSQEndTerm( WN, hN );  % set objective function for final state

% dynamic Model
ocp.setModel(f);

% % joint limits
% ocp.subjectTo( -pi <= [x1;x3] <= pi );
% ocp.subjectTo( -2*pi <= x2 <= 2*pi );

% joint velocity limits
ocp.subjectTo( -1 <= x4 <= 1 );
ocp.subjectTo( -5 <= x5 <= 5 );
ocp.subjectTo( -5 <= x6 <= 5 );

% joint force/torque limits
ocp.subjectTo( -60 <= tau1 <= 60);
ocp.subjectTo(-39 <= [tau2;tau3] <= 39);


% classical collision avoidance constraint
% ocp.subjectTo(norm([x1 - rt1 + rt4;(hb + ht)/2] - [x_obs;y_obs]) - r_obs - ((hb+ht)/2) >= 0);
% ocp.subjectTo(norm([x1 - rt1 + rt4 - (l2/2)*sin(x2); hb + ht + (l2/2)*cos(x2)] - [x_obs;y_obs]) - r_obs - (l2/2) >= 0);
% ocp.subjectTo(norm([x1 - rt1 + rt4 - (l3/2)*sin(x2 + x3) - l2*sin(x2); hb + ht + (l3/2)*cos(x2 + x3) + l2*cos(x2)] - [x_obs;y_obs]) - r_obs - (l3/2) >= 0);

n_points_link=5;
% P_obs=[0, 1.3]; %row vector, first one horizontal position, second one vertical
% r_obs=0.25;

if VelocityDamper==1
    %--------------------------------------------------------------------------
                             % VELOCITY DAMPER CONSTRAINT
    %--------------------------------------------------------------------------
    epsilon=0.5;
    ds=0;
    di=0.1;
    
    x0=[-2; -pi/4; -pi/4; 0; 0; 0];
    
    [d, dot_d, ddot_d, d0]=SimplifiedDistanceNDerivatives4Robot([0; 1.5], 0.25, x0) %obstacle pos as column vector!
    
    k=0;
    for i=1:3
        if d0(i) <= ds
            k=k+1; %just counting how many bodies of the robot are inside the security distance
        end
    end

    if k>0
        warning('The body of the robot is inside the security distance! Velocity damper constraints are not enforced.')
    else
        for i = 1:3
            ocp.subjectTo( eval(((((di-d(i))^2)^(1/2)+(di-d(i)))/(2*((di-d(i))^2)^(1/2)))*(dot_d(i)+Ts*ddot_d(i)-epsilon*(d(i)-ds)/(di-ds))) >=0)
        end
    end
    
    %---------------------------------------------------------------------------
else
    %--------------------------------------------------------------------------
                             % CBF-BASED CONSTRAINT
    %--------------------------------------------------------------------------
    % x0=[-2; -pi/4; -pi/4; 0; 0; 0];
    % [H, DHDT1,DHDT2,K]=CBF_Const_TIAGO(n_points_link,x0);
    [H, DHDT1,DHDT2,K]=CBF_Const_TIAGO(n_points_link);
    j=1;
    while j<= 4
        i=1;
        while i<= n_points_link+1 
        if H(j,i) ==0
                break
        else
        ocp.subjectTo(eval(DHDT2{j}(i) + K{j,i}*[H(j,i) ; DHDT1{j}(i)]) >=0);
        end
            i = i+1;
        end
            j =j+1;
    end
    %--------------------------------------------------------------------------
end

mpc = acado.OCPexport( ocp );

% set several option for the algorithm
mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mpc.set( 'SPARSE_QP_SOLUTION',          'FULL_CONDENSING_N2');
mpc.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL4'       );
mpc.set( 'NUM_INTEGRATOR_STEPS',        2*N                 );
mpc.set( 'QP_SOLVER',                   'QP_QPOASES'    	);
mpc.set( 'HOTSTART_QP',                 'NO'             	);
mpc.set( 'LEVENBERG_MARQUARDT', 		 1e-10				);
% mpc.set( 'GENERATE_SIMULINK_INTERFACE', 'YES'               );

if EXPORT
    mpc.exportCode( 'export_MPC' ); 
%     copyfile('../../../../../../external_packages/qpoases', 'export_MPC/qpoases', 'f')
    % !!!!!!!!!  ASSUMING ACADOtoolkit BASE FOLDER LOCATED IN THE HOME
    % DIRECTORY  !!!!!!!!
    % if it is not the case for out change the source path of the
    % ACADOtoolkit base directory !!!
    copyfile('~/ACADOtoolkit/external_packages/qpoases', 'export_MPC/qpoases', 'f')
    
    cd export_MPC
    make_acado_solver('../acado_MPCstep')
    cd ..
end

END_ACADO; % End with "END ACADO" to compile.

%% PREPARE SIMULATION

% generate reference
syms t;
h_x = -1.25+0.5*t;
h_y = 1.4+0.25*sin(t);
% generate derivative of reference
dh_x = diff(h_x,t);
dh_y = diff(h_y,t);
% evaluate trajectories numerically
t = linspace(0,5,5/Ts);
h_ref = double([subs(h_x,t).' subs(h_y,t).']);
dh_ref = double([subs(dh_x,t).' subs(dh_y,t).']);
% generate obstacles
obs = [0 1.5 0.25];

%% PARAMETERS SIMULATION

% initial state
X0 = [-2 -pi/4 -pi/4 0 0 0];

% Xref = [1.5 pi/3 -pi/2 0 0 0];
% input.x = repmat(Xref,N+1,1);
input.x = zeros(N+1,n_XD);

% Online data
input.od = repmat([obs X0],N+1,1);
% input.od = repmat(obs,N+1,1);

% Input torques reference
Uref = repmat(compute_gravity_term(X0(1:3)),N,1);
input.u = Uref;

% Output reference
Yref = [h_ref(1:N,:) dh_ref(1:N,:) Uref];
input.y = [Yref Uref];
input.yN = Yref(N,:);

% weight matrices
input.W = diag([50 50 1 1 0 0.0002 0.0002 0 0.001 0.01]);  % weights on [x y dx dy x4 x5 x6 tau1 tau2 tau3];
input.WN = diag([10 10 1 1 0 0 0]);   % weights on [x y dx dy x4 x5 x6];

%% SIMULATION LOOP
display('------------------------------------------------------------------')
display('               Simulation Loop'                                    )
display('------------------------------------------------------------------')

iter = 0; time = 0;
Tf = 5;
KKT_MPC = []; INFO_MPC = []; INFO_CPU_TIME = [];
controls_MPC = [];
state_sim = X0;
input_sim = zeros(1,n_U);

visualize;
while time(end) < Tf
    tic
    % Solve NMPC OCP
    input.x0 = state_sim(end,:);

    input.od = repmat([obs state_sim(end,:)],N+1,1); % pass X0 as an online data element

    Yref = [get_ref(h_ref,iter,N) get_ref(dh_ref,iter,N) zeros(N,3)];
    Uref = repmat(compute_gravity_term(state_sim(1:3)),N,1);
    input.u = Uref;
    input.y = [Yref Uref];  % reference for the output
    input.yN = Yref(N,:);  % reference for the final output state
    output = acado_MPCstep(input);
    
    % Save the MPC step
    INFO_MPC = [INFO_MPC; output.info];
    KKT_MPC = [KKT_MPC; output.info.kktValue];
    INFO_CPU_TIME = [INFO_CPU_TIME;output.info.cpuTime];
    controls_MPC = [controls_MPC; output.u(1,:)];
    input.x = output.x;
    input.u = output.u;
    
    % Simulate system
    sim_input.x = state_sim(end,:).';
    sim_input.u = output.u(1,:).';
    states = integrate_tiago(sim_input);
    state_sim = [state_sim; states.value'];
    input_sim = [input_sim; output.u(1,:)];
    
    iter = iter+1;
    nextTime = iter*Ts; 
    disp(['current time: ' num2str(nextTime) '   ' char(9) ' (RTI step: ' num2str(INFO_CPU_TIME(end)*1e6) ' µs)'])
    time = [time nextTime];
    
    visualize;
    pause(abs(Ts-toc));
end

% print the cpu-time plot at the end ( plotting it in real time slows down
% the simulation )
subplot(4,1,4);
plot(time(1:end-1),INFO_CPU_TIME,'linewidth',1.5,'color','k','linestyle','--','marker','.');hold on
plot(time(1:end-1),repmat(Ts,length(time)-1),'LineWidth',2,'color','red');
grid on;
xlabel('t [s]','FontSize',13);    ylabel('CPU time [s]','FontSize',13)


%% plot evolution of the joints: [x1 x2 x3]

%plot(time,state_sim(:,1:3)); grid on; hold on;

%% plot evolution of control inputs [tau1 tau2 tau3]

%stairs(time,input_sim); grid on; hold on; legend('tau1','tau2','tau3', "Location","best");

%% function: get the reference points

function samples = get_ref(ref,num_iter,N)
    num_iter = num_iter + 1;  % to be consistent with the inidces
    num_residual_samples = length(ref)-(num_iter+N);
    if num_residual_samples >= 0
        samples = ref(num_iter:num_iter+N-1,:);
    else
        if num_iter > length(ref)
            num_iter = length(ref);
        end
        samples = [ref(num_iter:end,:);repmat(ref(end,:),-num_residual_samples-1,1)];
    end
    
end

%% function: compute the gravity term
function g = compute_gravity_term(q)
    g0 = 9.81;

    q1 = q(1); q2 = q(2); q3 = q(3);

    m2 = 2.08+1.79+2.28;
    l2 = 0.32;

    m3 = 1.89+1.07+0.2+0.6+0.2;
    l3 = 0.34+0.25;

    g(1) = 0;
    g(2) = - g0*m3*((l3*sin(q2 + q3))/2 + l2*sin(q2)) - (g0*l2*m2*sin(q2))/2;
    g(3) = -(g0*l3*m3*sin(q2 + q3))/2;

end
