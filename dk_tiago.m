function [ x ] = dk_tiago(q)
    % this function returns the position of the joints of the robot given
    % the joint state q = [q1 q2 q3]
    
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

    q1 = q(1); q2 = q(2); q3 = q(3);

    % first joint position
    x(1,1) = q1 - rt1 + rt4;
    x(1,2) = hb + ht;

    % second joint position
    x(2,1) = q1 - rt1 + rt4 - l2*sin(q2);
    x(2,2) = hb + ht + l2*cos(q2);

    % E-E position 
    x(3,1) = q1 - rt1 + rt4 - l3*sin(q2 + q3) - l2*sin(q2);
    x(3,2) = hb + ht + l3*cos(q2 + q3) + l2*cos(q2);
    
    % center of the first link position
    x(4,1) = q1;
    x(4,2) = (hb+ht)/2;
    
    % center of the second link position
    x(5,1) = q1 - rt1 + rt4 - l2/2*sin(q2);
    x(5,2) = hb + ht + l2/2*cos(q2);
    
    % center of the third link position
    x(6,1) = q1 - rt1 + rt4 - l3/2*sin(q2 + q3) - l2*sin(q2);
    x(6,2) = hb + ht + l3/2*cos(q2 + q3) + l2*cos(q2);

end
