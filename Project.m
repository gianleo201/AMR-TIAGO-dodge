clc;
clear all;
close all;

%% Definition of symbols

% DH symbols
syms alpha a d theta;

% other symbols
syms g0;
syms rt1 rt2 rt3 rt4 rw hb ht mb mt mw;


% number of joints
N = 3;

% vector of variables
syms dc [N 1];
syms m [N 1];
syms q [N 1];
syms qd [N 1];
syms qdd [N 1];
syms l [N 1];

%total mass of the first link, not considering the wheels
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

DHTABLE = [-pi/2 rw+hb+ht q1-rt1+rt4 0;
            0    l2 0  -(pi/2-q2);
            0    l3 0  q3];

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

%% joint's mass' center positions 

CoM0p1=1/m1*(mb*[rw; 0; q1]+mt*[rw+hb+rt2; 0; q1-rt1+rt3]+2*mw*[rw; 0; q1]);

CoM1p1h = inv(A{1}) * [CoM0p1; 1];
CoM1p1 = CoM1p1h(1:3);

CoMsp = [CoM1p1.';
         -l2/2    0     0;
         -l3/2    0     0];

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

%% return 1st order ODEs of the dynamic model

syms x [2*N 1];
syms tau [N 1];

invM = simplify(inv(M));

eqs = simplify(subs(invM*(tau-c-G),[q;qd],[x(1:N);x(N+1:2*N)]));

eqs = [x4;x5;x6;eqs];

%% save equations in a file

save("Model_eqs.mat","eqs");

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

 