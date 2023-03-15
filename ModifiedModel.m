clc;
clear all;
close all;

%% Definition of symbols

% DH symbols
syms alpha a d theta;

% other symbols
syms g0;
syms rt1 rt2 rt3 rt4 rw hb ht mb mt mw db dw real;


% number of joints
N = 5;

% vector of variables
syms dc [N 1] real;
syms m [N 1] real;
syms q [N 1] real;
syms qd [N 1] real;
syms qdd [N 1] real;
syms l [N 1] real;
syms tau [N 1] real;

%total mass of the first link, considering the wheels
m1=mb+mt+2*mw;

I = cell(1,N); %initialized as empty

for i=1:N

    % Inertia terms
    nus1 = sym(sprintf("I%dxx",i)); 
    %basically if i=2 it returns I2xx, and so on as a sym variable
    nus2 = sym(sprintf("I%dyy",i));
    nus3 = sym(sprintf("I%dzz",i));
    eval(['I' num2str(i) 'xx=nus1' ]);
    %If i=2, it's assigning the name "I2xx" to the sym nus1
    %eval in Matlab evaluate the matlab CODE specified in the expression:
    %eval(expression).
    eval(['I' num2str(i) 'yy=nus2' ]);
    eval(['I' num2str(i) 'zz=nus3' ]);

    %in our case I verified inertia terms besides the diagonal
    % ones don't intervene
    % (CHECK THIS)

    % Inertia matrix
    %we're creating the intertia matrix for each joint, like now we've Ii_j matrix  

    eval(['I' num2str(i) '=[nus1 0 0;0 nus2 0;0 0 nus3];']);

    eval(['I{' num2str(i) '}=I' num2str(i)]);
    %it's assigning to the element of the CELL I{2} the symbolic inertia
    %matrix created
end

clear nus1 nus2 nus3 nus4 nus5 nus6 nus7;

% gravity vector
g_0 = [-g0;0;0];

%% DH table

% alpha a d theta

% DHTABLE = [-pi/2 rw+hb+ht q1-rt1+rt4 0;
%             0    l2 0  q2;
%             0    l3 0  q3];

DHTABLE = [pi/2 0 q1 pi/2;
           pi/2 -rt1+rt4 q2+hb+ht+rw pi/2;
           0 0 0 q3+(pi/2);
           0 l2 0 q4;
           0 l3 0 q5;];

%% General DH transformation matrix

TDH = [ cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
        sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
          0             sin(alpha)             cos(alpha)            d;
          0               0                      0                   1];

%% Build transformation matrix for each link

A = cell(1,N);

%remember that the DHTABLE row order is: alpha a d theta

for i = 1:N
    alpha = DHTABLE(i,1);
    a = DHTABLE(i,2);
    d = DHTABLE(i,3);
    theta = DHTABLE(i,4);
    A{i} = subs(TDH);
end

%% Direct kinematics / differential kinematics

T = eye(4);

Ts = cell(1,N);

for i = 1:N
    T = T*A{i};
    T = simplify(T);
    Ts{i} = T;
end

T0N = T;


% direct kinematics
f = T0N(1:3,4);


% analitic jacobian
J = simplify(jacobian(f,q));



% geometric jacobian
geomJ = sym(zeros(6,N));
for i = 1:N

    if i == 1
        R = eye(4);
    else
        R = Ts{i-1};
    end

    % prismatic joint
    if has( DHTABLE(i,3), q(:) )
        geomJ(1:3,i) = R(1:3,3);
        geomJ(4:6,i) = [0;0;0];
    % revolute joint
    elseif has( DHTABLE(i,4), q(:) )
        geomJ(1:3,i) = simplify(cross(R(1:3,3),f-R(1:3,4)));
        geomJ(4:6,i) = R(1:3,3);
    else
        disp('ERROR OCCURED CHECK DHTABLE');
        break;
    end
end

% second order differential kinematics: jacobian time derivative Jdot
% of course, time derivative of each element of the jacobian matrix
Jdot = jacobian_diff(J,q,qd);

%% link's mass' center positions 

syms CoM [3 1 4] real
CoM0(:, :, 1)=[rw/2; 0; q1-db/2]; %CoM position wrt of frame 0 for the wheel 1
CoM0(:, :, 2)=[rw/2; 0; q1+db/2]; % //// wheel 2
CoM0(:, :, 3)=[rw; 0; q1]; % //// base
CoM0(:, :, 4)=[rw+hb+rt2; 0; q1-rt1+rt3]; % //// torso


CoM0p1=1/m1*(mb*CoM0(:, :, 3)+mt*CoM0(:, :, 4)+mw*CoM0(:, :, 1)+mw*CoM0(:, :, 2)); %CoM of the first link wrt frame 0

Iw=[1/12*mw*(3*(rw/2)^2+dw^2) 0 0; 0 1/2*mw*(rw/2)^2 0; 0 0 1/12*mw*(3*(rw/2)^2+dw^2)]; %inertia matrix for a wheel expressed in frame 0
syms Ibxx Ibyy Ibzz Itxx Ityy Itzz real
Ib=[Ibxx 0 0; 0 Ibyy 0; 0 0 Ibzz]; %inertia matrix for the body expressed in frame 0
It=[Itxx 0 0; 0 Ityy 0; 0 0 Itzz]; %inertia matrix for the torso expressed in frame 0

syms rc [3 1 4] real % distance vector between CoMs of the first link component and the total CoM of the first link
for i=1:4
    rc(:,:, i)=CoM0(:,:,i)-CoM0p1;
end
% we use Steiner's theorem recursively
I1tot = Iw+mw*(((rc(:,:, 1)).')*(rc(:,:, 1))*eye(3)-(rc(:,:, 1))*((rc(:,:, 1)).')); 
I1tot = I1tot+Iw+mw*(((rc(:,:, 2)).')*(rc(:,:, 2))*eye(3)-(rc(:,:, 2))*((rc(:,:, 2)).'));
I1tot = I1tot+Ib+mb*(((rc(:,:, 3)).')*(rc(:,:, 3))*eye(3)-(rc(:,:, 3))*((rc(:,:, 3)).'));
I1tot = I1tot+It+mt*(((rc(:,:, 4)).')*(rc(:,:, 4))*eye(3)-(rc(:,:, 4))*((rc(:,:, 4)).'));
I1tot=simplify(I1tot); %this should be the total moment of inertia matrix of the first link wrt to the global CoM of the first link

% CoM1p1h = inv(A{1}) * [CoM0p1; 1]; %CoM of the first link wrt frame 1 in homogenous coordinates
% CoM1p1 = CoM1p1h(1:3); %CoM of the first link wrt frame 1
% 
% CoMsp = [CoM1p1.';
%          -l2/2    0     0;
%          -l3/2    0     0];

CoM1p1h = simplify(inv(Ts{1}) * [CoM0p1; 1]);
CoM1p1 = CoM1p1h(1:3);

CoM1p2h = simplify(inv(Ts{2}) * [CoM0p1; 1]);
CoM1p2 = CoM1p2h(1:3);

CoM1p3h = simplify(inv(Ts{3}) * [CoM0p1; 1]);
CoM1p3 = CoM1p3h(1:3);

CoMsp = [CoM1p1.';
         CoM1p2.';
         CoM1p3.';
         -l2/2    0     0;
         -l3/2    0     0];

% 1st and 2nd have no mass and inertia
I1xx = 0; I1yy = 0; I1zz = 0; m1 = 0;
I1 = subs(I1);
I2xx = 0; I2yy = 0; I2zz = 0; m2 = 0;
I2 = subs(I2);
m = subs(m);
% the 3rd link is attached to robot (is a body frame) base so it has mass and inertia of the
% base


%% Direct Dynamics

Kth = cell(1,N);
K = 0;

%initialization
w = [0;0;0];
v_c = [0;0;0];
v = [0;0;0];

%% Kinetic energy and inertia matrix 

for i = 1:N
    
    % {i-1}^R_{i}
    Rot = A{i}(1:3,1:3);
    
    % {i}^r{i-1,i}
    r = simplify(Rot.'*A{i}(1:3,4));

    % prismatic joint
    if has( DHTABLE(i,3), q(:) )
        w = Rot.' * w;
        v = Rot.' * (v + qd(i)*[0;0;1]) + cross(w,r);
        rci = CoMsp(i,:).';
        v_c = simplify( v + cross(w,rci) );
    % revolut joint
    elseif has( DHTABLE(i,4), q(:) )
        w = Rot.' * (w+qd(i)*[0;0;1]);
        v = Rot.' * v + cross(w,r);
        rci = CoMsp(i,:).';
        v_c = simplify( v + cross(w,rci) );
    else
        disp('ERROR OCCURED CHECK DHTABLE');
        break;
    end
    temp = simplify(  (1/2)*m(i)*(v_c.')*v_c + (1/2)*(w.')*I{i}*w  );
    Kth{i} = temp;
    K = K + temp;
    K = simplify(K);

end

% extract inertia matrix
M = simplify(hessian(K,qd));

% compute inertia matrix derivative
Mdot = jacobian_diff(M,q,qd);

%% Coriolis and centrifugal terms

c = sym('C',[N 1]);
S = sym('S',[N N]);

for i = 1:N
    temp1 = (jacobian(M(:,i),q));
    
    % compute Christoffel symbols
    C = simplify( (1/2) * (temp1+temp1.'-diff(M,q(i))) );

    % compute c/c term
    c(i) = simplify(qd.' * C * qd);

    % compute i-th S' row (for skew-symmetry of Md-2S)
    S(i,:) = simplify(qd.' * C);
end


%% Gravity terms

U = 0;

for i = 1:N
    temp1 = (Ts{i}*[CoMsp(i,:).';1]);
    temp = simplify(-m(i)*g_0.'*temp1(1:3,1));
    U = simplify( U + temp );
end

G = jacobian(U,q).';


%% Dynamic model

model = simplify(M*qdd+c+G);

%% compute the reaction forces when q2,q3 are constrained to be 0

Ac = [0 0;1 0;0 1;0 0;0 0];
Qf = [0 1 0 0 0;0 0 1 0 0];
Qr = [1 0 0 0 0;0 0 0 1 0;0 0 0 0 1];
Gc = eye(3);


lambda_react = simplify( subs(-Qf * ( M*Qr.'*inv(Qr*M*Qr.')*Qr*(tau-c) + c ) , [q2 q3 qd2 qd3],[0 0 0 0]) );

f2 = lambda_react(1);
mu3 = lambda_react(2);

%% use symbols of the model used in acado

% run this section separately from the upper file

syms x [6 1] real;
syms tau [3 1] real;
syms m1 m2 m3 I2xx I2yy I2zz;

f2 = subs(f2, ...
    [q1 q2 q3 q4 q5 qd1 qd2 qd3 qd4 qd5 tau1 tau2 tau3 tau4 tau5 m3 m4 m5 I4xx I4yy I4zz I5xx I5yy I5zz], ...
    [x1 0  0  x2 x3 x4  0   0   x5  x6  tau1 0    0    tau2 tau3 m1 m2 m3 I2xx I2yy I2zz I3xx I3yy I3zz]);

mu3 = subs(mu3, ...
    [q1 q2 q3 q4 q5 qd1 qd2 qd3 qd4 qd5 tau1 tau2 tau3 tau4 tau5 m3 m4 m5 I4xx I4yy I4zz I5xx I5yy I5zz], ...
    [x1 0  0  x2 x3 x4  0   0   x5  x6  tau1 0    0    tau2 tau3 m1 m2 m3 I2xx I2yy I2zz I3xx I3yy I3zz]);

% Mm1 = simplify(mu3+f2*(x1-(db/2)-rt1+rt4));
% Mm2 = simplify(-mu3-f2*(x1+(db/2)-rt1+rt4));

Mm1 = simplify(mu3+f2*(-(db/2)));
Mm2 = simplify(-mu3-f2*((db/2)));


%% return 1st order ODEs of the dynamic model

% syms x [2*N 1];
% syms tau [N 1];
% 
% invM = simplify(inv(M));
% 
% eqs = simplify(subs(invM*(tau-c-G),[q;qd],[x(1:N);x(N+1:2*N)]));
% 
% eqs = [x4;x5;x6;eqs];

%% save equations in a file

% save("Model_eqs.mat","eqs");

%% useful functions


% compute time derivative of the jacobian
function J_dot = jacobian_diff(J,var,dot_var)
    J_dot = sym('J_dot',size(J));
    s = size(J);
    rows = s(1);
    columns = s(2);
    for i=1:rows
        for j=1:columns
            pd = jacobian(J(i,j),var) * dot_var;
            J_dot(i,j) = simplify(pd);
        end
    end
end

 