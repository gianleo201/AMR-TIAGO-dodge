function [ dx ] = TIAGo(t,x,u,p,w)
    
    % load the equation from matlab file
    load("Tiago_eqs.mat","eqs");
    
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

    
    % return the eqations
    dx = subs(eqs);


end
