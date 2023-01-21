function [ dx ] = TIAGO(t,x,u,p,w)
    
    % load the equation from matlab file
    % load("Tiago_eqs.mat","eqs");
    
    % define states
    x1 = x(1); x2 = x(2); x3 = x(3);
    x4 = x(4); x5 = x(5); x6 = x(6);
    
    % define inputs
    tau1 = u(1); tau2 = u(2); tau3 = u(3);
    % tau1 = 0; tau2 = 0; tau3 = 0;

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

    
    % return the equations
    dx(1) = x4;
    dx(2) = x5;
    dx(3) = x6;

    dx(4) = -(4*I3zz*l2^3*m2^2*x5^2*sin(x2) - 4*l2^2*l3^2*m3^2*tau1 - 8*I3zz*l2^2*m2*tau1 - 8*I2zz*l3^2*m3*tau1 - 32*I3zz*l2^2*m3*tau1 - 32*I2zz*I3zz*tau1 + 32*I3zz*l2^3*m3^2*x5^2*sin(x2) - 4*I2zz*g0*l3^2*m3^2*sin(2*x2 + 2*x3) - 2*l2^2*l3^2*m2*m3*tau1 + 4*l2^2*l3^2*m3^2*tau1*cos(2*x3) - 8*l2^2*l3*m3^2*tau3*cos(x2 + x3) - 16*I2zz*l3*m3*tau3*cos(x2 + x3) - 4*l2*l3^2*m3^2*tau2*cos(x2) + 4*l2*l3^2*m3^2*tau3*cos(x2) - 16*I3zz*l2*m2*tau2*cos(x2) + 16*I3zz*l2*m2*tau3*cos(x2) - 32*I3zz*l2*m3*tau2*cos(x2) + 32*I3zz*l2*m3*tau3*cos(x2) + 4*I2zz*l3^3*m3^2*x5^2*sin(x2 + x3) + 4*I2zz*l3^3*m3^2*x6^2*sin(x2 + x3) + 4*l2*l3^2*m3^2*tau2*cos(x2 + 2*x3) + 8*l2^2*l3*m3^2*tau3*cos(x2 - x3) - 4*l2*l3^2*m3^2*tau3*cos(x2 + 2*x3) - 4*I3zz*g0*l2^2*m2^2*sin(2*x2) - 16*I3zz*g0*l2^2*m3^2*sin(2*x2) + 24*I3zz*l2^3*m2*m3*x5^2*sin(x2) + g0*l2^2*l3^2*m2*m3^2*sin(2*x2 + 2*x3) + l2^2*l3^3*m2*m3^2*x5^2*sin(x2 - x3) + l2^2*l3^3*m2*m3^2*x6^2*sin(x2 - x3) - l2^3*l3^2*m2*m3^2*x5^2*sin(x2 + 2*x3) + 8*I3zz*l2^2*l3*m3^2*x5^2*sin(x2 + x3) + 8*I3zz*l2^2*l3*m3^2*x6^2*sin(x2 + x3) + 16*I2zz*I3zz*l3*m3*x5^2*sin(x2 + x3) + 16*I2zz*I3zz*l3*m3*x6^2*sin(x2 + x3) + 4*I2zz*l2*l3^2*m3^2*x5^2*sin(x2) + 16*I2zz*I3zz*l2*m2*x5^2*sin(x2) + 32*I2zz*I3zz*l2*m3*x5^2*sin(x2) + 4*I2zz*l2*l3^2*m3^2*x5^2*sin(x2 + 2*x3) + 8*I3zz*l2^2*l3*m3^2*x5^2*sin(x2 - x3) + 8*I3zz*l2^2*l3*m3^2*x6^2*sin(x2 - x3) - 4*l2*l3^2*m2*m3*tau2*cos(x2) + 4*l2*l3^2*m2*m3*tau3*cos(x2) - 2*g0*l2^2*l3^2*m2*m3^2*sin(2*x2) - g0*l2^2*l3^2*m2^2*m3*sin(2*x2) + 3*l2^3*l3^2*m2*m3^2*x5^2*sin(x2) + l2^3*l3^2*m2^2*m3*x5^2*sin(x2) + 8*I2zz*l3^3*m3^2*x5*x6*sin(x2 + x3) + 4*l2^2*l3*m2*m3*tau3*cos(x2 - x3) - 16*I3zz*g0*l2^2*m2*m3*sin(2*x2) + 2*l2^2*l3^3*m2*m3^2*x5*x6*sin(x2 - x3) + 16*I3zz*l2^2*l3*m3^2*x5*x6*sin(x2 + x3) + 4*I2zz*l2*l3^2*m2*m3*x5^2*sin(x2) + 32*I2zz*I3zz*l3*m3*x5*x6*sin(x2 + x3) + 4*I3zz*l2^2*l3*m2*m3*x5^2*sin(x2 - x3) + 4*I3zz*l2^2*l3*m2*m3*x6^2*sin(x2 - x3) + 16*I3zz*l2^2*l3*m3^2*x5*x6*sin(x2 - x3) + 8*I3zz*l2^2*l3*m2*m3*x5*x6*sin(x2 - x3))/(4*I3zz*l2^2*m2^2 + 4*I2zz*l3^2*m3^2 + 16*I3zz*l2^2*m3^2 + 32*I2zz*I3zz*m1 + 32*I2zz*I3zz*m2 + 32*I2zz*I3zz*m3 + 4*l2^2*l3^2*m1*m3^2 + 3*l2^2*l3^2*m2*m3^2 + l2^2*l3^2*m2^2*m3 + 8*I3zz*l2^2*m1*m2 + 8*I2zz*l3^2*m1*m3 + 32*I3zz*l2^2*m1*m3 + 8*I2zz*l3^2*m2*m3 + 24*I3zz*l2^2*m2*m3 - 4*I3zz*l2^2*m2^2*cos(2*x2) - 16*I3zz*l2^2*m3^2*cos(2*x2) - 4*I2zz*l3^2*m3^2*cos(2*x2 + 2*x3) + 2*l2^2*l3^2*m1*m2*m3 - 4*l2^2*l3^2*m1*m3^2*cos(2*x3) - 2*l2^2*l3^2*m2*m3^2*cos(2*x2) - l2^2*l3^2*m2^2*m3*cos(2*x2) - 2*l2^2*l3^2*m2*m3^2*cos(2*x3) - 16*I3zz*l2^2*m2*m3*cos(2*x2) + l2^2*l3^2*m2*m3^2*cos(2*x2 + 2*x3));
    dx(5) = (2*(2*l3^2*m3^2*tau2 - 2*l3^2*m3^2*tau3 + 16*I3zz*m1*tau2 - 16*I3zz*m1*tau3 + 16*I3zz*m2*tau2 - 16*I3zz*m2*tau3 + 16*I3zz*m3*tau2 - 16*I3zz*m3*tau3 + 4*l3^2*m1*m3*tau2 - 4*l3^2*m1*m3*tau3 + 4*l3^2*m2*m3*tau2 - 4*l3^2*m2*m3*tau3 - 2*l3^2*m3^2*tau2*cos(2*x2 + 2*x3) + 2*l3^2*m3^2*tau3*cos(2*x2 + 2*x3) + 8*I3zz*g0*l2*m2^2*sin(x2) + 16*I3zz*g0*l2*m3^2*sin(x2) - 4*l2*l3*m3^2*tau3*cos(x3) - 2*I3zz*l2^2*m2^2*x5^2*sin(2*x2) - 8*I3zz*l2^2*m3^2*x5^2*sin(2*x2) + 4*l2*l3*m3^2*tau3*cos(2*x2 + x3) + 2*l2*l3^2*m3^2*tau1*cos(x2) + 8*I3zz*l2*m2*tau1*cos(x2) + 16*I3zz*l2*m3*tau1*cos(x2) - 2*l2*l3^2*m3^2*tau1*cos(x2 + 2*x3) + 4*I3zz*l2*l3*m3^2*x5^2*sin(x3) + 4*I3zz*l2*l3*m3^2*x6^2*sin(x3) + 2*g0*l2*l3^2*m1*m3^2*sin(x2) + 3*g0*l2*l3^2*m2*m3^2*sin(x2) + 2*g0*l2*l3^2*m2^2*m3*sin(x2) + 8*I3zz*g0*l2*m1*m2*sin(x2) + 16*I3zz*g0*l2*m1*m3*sin(x2) + 24*I3zz*g0*l2*m2*m3*sin(x2) - 4*I3zz*l2*l3*m3^2*x5^2*sin(2*x2 + x3) - 4*I3zz*l2*l3*m3^2*x6^2*sin(2*x2 + x3) + 2*l2^2*l3^2*m1*m3^2*x5^2*sin(2*x3) - l2^2*l3^2*m2*m3^2*x5^2*sin(2*x2) - (l2^2*l3^2*m2^2*m3*x5^2*sin(2*x2))/2 + l2^2*l3^2*m2*m3^2*x5^2*sin(2*x3) - 2*g0*l2*l3^2*m1*m3^2*sin(x2 + 2*x3) - g0*l2*l3^2*m2*m3^2*sin(x2 + 2*x3) - 8*l2*l3*m1*m3*tau3*cos(x3) - 6*l2*l3*m2*m3*tau3*cos(x3) - 8*I3zz*l2^2*m2*m3*x5^2*sin(2*x2) + 2*l2*l3^3*m1*m3^2*x5^2*sin(x3) + 2*l2*l3^3*m1*m3^2*x6^2*sin(x3) + (3*l2*l3^3*m2*m3^2*x5^2*sin(x3))/2 + (3*l2*l3^3*m2*m3^2*x6^2*sin(x3))/2 + 2*l2*l3*m2*m3*tau3*cos(2*x2 + x3) + 2*l2*l3^2*m2*m3*tau1*cos(x2) - (l2*l3^3*m2*m3^2*x5^2*sin(2*x2 + x3))/2 - (l2*l3^3*m2*m3^2*x6^2*sin(2*x2 + x3))/2 + 8*I3zz*l2*l3*m1*m3*x5^2*sin(x3) + 8*I3zz*l2*l3*m1*m3*x6^2*sin(x3) + 6*I3zz*l2*l3*m2*m3*x5^2*sin(x3) + 6*I3zz*l2*l3*m2*m3*x6^2*sin(x3) + 8*I3zz*l2*l3*m3^2*x5*x6*sin(x3) + 2*g0*l2*l3^2*m1*m2*m3*sin(x2) - 2*I3zz*l2*l3*m2*m3*x5^2*sin(2*x2 + x3) - 2*I3zz*l2*l3*m2*m3*x6^2*sin(2*x2 + x3) - 8*I3zz*l2*l3*m3^2*x5*x6*sin(2*x2 + x3) + 4*l2*l3^3*m1*m3^2*x5*x6*sin(x3) + 3*l2*l3^3*m2*m3^2*x5*x6*sin(x3) - l2*l3^3*m2*m3^2*x5*x6*sin(2*x2 + x3) + 16*I3zz*l2*l3*m1*m3*x5*x6*sin(x3) + 12*I3zz*l2*l3*m2*m3*x5*x6*sin(x3) - 4*I3zz*l2*l3*m2*m3*x5*x6*sin(2*x2 + x3)))/(4*I3zz*l2^2*m2^2 + 4*I2zz*l3^2*m3^2 + 16*I3zz*l2^2*m3^2 + 32*I2zz*I3zz*m1 + 32*I2zz*I3zz*m2 + 32*I2zz*I3zz*m3 + 4*l2^2*l3^2*m1*m3^2 + 3*l2^2*l3^2*m2*m3^2 + l2^2*l3^2*m2^2*m3 + 8*I3zz*l2^2*m1*m2 + 8*I2zz*l3^2*m1*m3 + 32*I3zz*l2^2*m1*m3 + 8*I2zz*l3^2*m2*m3 + 24*I3zz*l2^2*m2*m3 - 4*I3zz*l2^2*m2^2*cos(2*x2) - 16*I3zz*l2^2*m3^2*cos(2*x2) - 4*I2zz*l3^2*m3^2*cos(2*x2 + 2*x3) + 2*l2^2*l3^2*m1*m2*m3 - 4*l2^2*l3^2*m1*m3^2*cos(2*x3) - 2*l2^2*l3^2*m2*m3^2*cos(2*x2) - l2^2*l3^2*m2^2*m3*cos(2*x2) - 2*l2^2*l3^2*m2*m3^2*cos(2*x3) - 16*I3zz*l2^2*m2*m3*cos(2*x2) + l2^2*l3^2*m2*m3^2*cos(2*x2 + 2*x3));
    dx(6) = (4*(tau3 + (g0*l3*m3*sin(x2 + x3))/2 - (l2*l3*m3*x5^2*sin(x3))/2)*(8*I2zz*m1 + 8*I2zz*m2 + 8*I3zz*m1 + 8*I2zz*m3 + 8*I3zz*m2 + 8*I3zz*m3 + l2^2*m2^2 + 4*l2^2*m3^2 + l3^2*m3^2 + 2*l2^2*m1*m2 + 8*l2^2*m1*m3 + 6*l2^2*m2*m3 + 2*l3^2*m1*m3 + 2*l3^2*m2*m3 - l2^2*m2^2*cos(2*x2) - 4*l2^2*m3^2*cos(2*x2) - l3^2*m3^2*cos(2*x2 + 2*x3) - 4*l2*l3*m3^2*cos(2*x2 + x3) - 4*l2^2*m2*m3*cos(2*x2) + 4*l2*l3*m3^2*cos(x3) - 2*l2*l3*m2*m3*cos(2*x2 + x3) + 8*l2*l3*m1*m3*cos(x3) + 6*l2*l3*m2*m3*cos(x3)))/(4*I3zz*l2^2*m2^2 + 4*I2zz*l3^2*m3^2 + 16*I3zz*l2^2*m3^2 + 32*I2zz*I3zz*m1 + 32*I2zz*I3zz*m2 + 32*I2zz*I3zz*m3 + 4*l2^2*l3^2*m1*m3^2 + 3*l2^2*l3^2*m2*m3^2 + l2^2*l3^2*m2^2*m3 + 8*I3zz*l2^2*m1*m2 + 8*I2zz*l3^2*m1*m3 + 32*I3zz*l2^2*m1*m3 + 8*I2zz*l3^2*m2*m3 + 24*I3zz*l2^2*m2*m3 - 4*I3zz*l2^2*m2^2*cos(2*x2) - 16*I3zz*l2^2*m3^2*cos(2*x2) - 4*I2zz*l3^2*m3^2*cos(2*x2 + 2*x3) + 2*l2^2*l3^2*m1*m2*m3 - 4*l2^2*l3^2*m1*m3^2*cos(2*x3) - 2*l2^2*l3^2*m2*m3^2*cos(2*x2) - l2^2*l3^2*m2^2*m3*cos(2*x2) - 2*l2^2*l3^2*m2*m3^2*cos(2*x3) - 16*I3zz*l2^2*m2*m3*cos(2*x2) + l2^2*l3^2*m2*m3^2*cos(2*x2 + 2*x3)) - (4*(tau2 + g0*m3*((l3*sin(x2 + x3))/2 + l2*sin(x2)) + (g0*l2*m2*sin(x2))/2 + (l2*l3*m3*x6*sin(x3)*(2*x5 + x6))/2)*(8*I3zz*m1 + 8*I3zz*m2 + 8*I3zz*m3 + l3^2*m3^2 + 2*l3^2*m1*m3 + 2*l3^2*m2*m3 - l3^2*m3^2*cos(2*x2 + 2*x3) - 2*l2*l3*m3^2*cos(2*x2 + x3) + 2*l2*l3*m3^2*cos(x3) - l2*l3*m2*m3*cos(2*x2 + x3) + 4*l2*l3*m1*m3*cos(x3) + 3*l2*l3*m2*m3*cos(x3)))/(4*I3zz*l2^2*m2^2 + 4*I2zz*l3^2*m3^2 + 16*I3zz*l2^2*m3^2 + 32*I2zz*I3zz*m1 + 32*I2zz*I3zz*m2 + 32*I2zz*I3zz*m3 + 4*l2^2*l3^2*m1*m3^2 + 3*l2^2*l3^2*m2*m3^2 + l2^2*l3^2*m2^2*m3 + 8*I3zz*l2^2*m1*m2 + 8*I2zz*l3^2*m1*m3 + 32*I3zz*l2^2*m1*m3 + 8*I2zz*l3^2*m2*m3 + 24*I3zz*l2^2*m2*m3 - 4*I3zz*l2^2*m2^2*cos(2*x2) - 16*I3zz*l2^2*m3^2*cos(2*x2) - 4*I2zz*l3^2*m3^2*cos(2*x2 + 2*x3) + 2*l2^2*l3^2*m1*m2*m3 - 4*l2^2*l3^2*m1*m3^2*cos(2*x3) - 2*l2^2*l3^2*m2*m3^2*cos(2*x2) - l2^2*l3^2*m2^2*m3*cos(2*x2) - 2*l2^2*l3^2*m2*m3^2*cos(2*x3) - 16*I3zz*l2^2*m2*m3*cos(2*x2) + l2^2*l3^2*m2*m3^2*cos(2*x2 + 2*x3)) + (4*((l3*m3*x5^2*sin(x2 + x3))/2 - tau1 + (l3*m3*x6^2*sin(x2 + x3))/2 + (l2*m2*x5^2*sin(x2))/2 + l2*m3*x5^2*sin(x2) + l3*m3*x5*x6*sin(x2 + x3))*(l2*l3^2*m3^2*cos(x2) - 4*I2zz*l3*m3*cos(x2 + x3) - 2*l2^2*l3*m3^2*cos(x2 + x3) + 4*I3zz*l2*m2*cos(x2) + 8*I3zz*l2*m3*cos(x2) + 2*l2^2*l3*m3^2*cos(x2 - x3) - l2*l3^2*m3^2*cos(x2 + 2*x3) + l2*l3^2*m2*m3*cos(x2) + l2^2*l3*m2*m3*cos(x2 - x3)))/(4*I3zz*l2^2*m2^2 + 4*I2zz*l3^2*m3^2 + 16*I3zz*l2^2*m3^2 + 32*I2zz*I3zz*m1 + 32*I2zz*I3zz*m2 + 32*I2zz*I3zz*m3 + 4*l2^2*l3^2*m1*m3^2 + 3*l2^2*l3^2*m2*m3^2 + l2^2*l3^2*m2^2*m3 + 8*I3zz*l2^2*m1*m2 + 8*I2zz*l3^2*m1*m3 + 32*I3zz*l2^2*m1*m3 + 8*I2zz*l3^2*m2*m3 + 24*I3zz*l2^2*m2*m3 - 4*I3zz*l2^2*m2^2*cos(2*x2) - 16*I3zz*l2^2*m3^2*cos(2*x2) - 4*I2zz*l3^2*m3^2*cos(2*x2 + 2*x3) + 2*l2^2*l3^2*m1*m2*m3 - 4*l2^2*l3^2*m1*m3^2*cos(2*x3) - 2*l2^2*l3^2*m2*m3^2*cos(2*x2) - l2^2*l3^2*m2^2*m3*cos(2*x2) - 2*l2^2*l3^2*m2*m3^2*cos(2*x3) - 16*I3zz*l2^2*m2*m3*cos(2*x2) + l2^2*l3^2*m2*m3^2*cos(2*x2 + 2*x3));


end