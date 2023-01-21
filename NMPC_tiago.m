clc;
clear all;
close all;

Ts = 0.01;
EXPORT = 1;

BEGIN_ACADO; % Always start with "BEGIN_ACADO".

DifferentialState x1 x2 x3 x4 x5 x6;
Control tau1 tau2 tau3;

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
    
% ouput function
h = [diffStates; controls];
hN = [diffStates];

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

N = 25;
ocp = acado.OCP( 0.0, N*Ts, N );

W_mat = eye(n_XD+n_U,n_XD+n_U);
WN_mat = eye(n_XD,n_XD);
W = acado.BMatrix(W_mat);
WN = acado.BMatrix(WN_mat);

ocp.minimizeLSQ( W, h );
ocp.minimizeLSQEndTerm( WN, hN );

ocp.subjectTo( -2*pi <= [x1;x2;x3] <= 2*pi );  % joint limits
ocp.subjectTo( -10.0 <= [x4;x5;x6] <= 10.0 );  % joint velocity limits
% joint effort limits
ocp.subjectTo(-100 <= [tau2;tau3] <=  100);
ocp.subjectTo( -10000 <= tau1 <= 10000);

ocp.setModel(f);     % set the ODE as constriant

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

%% PARAMETERS SIMULATION
X0 = [-1 -pi/4 -pi/2 0 0 0];
Xref = [1.5 pi/3 -pi/2 0 0 0];
input.x = repmat(Xref,N+1,1);
Xref = repmat(Xref,N,1);
input.od = [];

Uref = zeros(N,n_U);
input.u = Uref;

input.y = [Xref(1:N,:) Uref];
input.yN = Xref(N,:);

input.W = diag([10 1 1 1e-3 1e-3 1e-3 0 0 0]);  % weights on [x1 x2 x3 x4 x5 x6];
input.WN = diag([10 1 1 1 1 1]);  % weights on [x1 x2 x3 x4 x5 x6];

%% SIMULATION LOOP
display('------------------------------------------------------------------')
display('               Simulation Loop'                                    )
display('------------------------------------------------------------------')

iter = 0; time = 0;
Tf = 5;
KKT_MPC = []; INFO_MPC = [];
controls_MPC = [];
state_sim = X0;
input_sim = zeros(1,n_U);

visualize;
while time(end) < Tf
    tic
    % Solve NMPC OCP
    input.x0 = state_sim(end,:);
    output = acado_MPCstep(input);
    
    % Save the MPC step
    INFO_MPC = [INFO_MPC; output.info];
    KKT_MPC = [KKT_MPC; output.info.kktValue];
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
    disp(['current time: ' num2str(nextTime) '   ' char(9) ' (RTI step: ' num2str(output.info.cpuTime*1e6) ' µs)'])
    time = [time nextTime];
    
    visualize;
    pause(abs(Ts-toc));
end


%% plot evolution of the joints: [x1 x2 x3]

plot(time,state_sim(:,1:3)); grid on; hold on;

%% plot evolution of control inputs [tau1 tau2 tau3]

%stairs(time,input_sim); grid on; hold on; legend('tau1','tau2','tau3', "Location","best");
