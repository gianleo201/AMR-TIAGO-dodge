
% x0=[0; pi/4; pi/4; 0; 0; 0];
%[H, DHDT1,DHDT2,K]=CBF_Const_TIAGO([0 1.3],0.1 ,1,x0); %CBF is initial conditions+model dependent constraint
%h(r)>= -K [h; h(r-1)] r=2
%ocp.subjectTo(DHDT2{j}(i) >= - K{j,i}*[H(j,i) ; DHDT1{j}(i)]);

% old function signature (with obs data dependence removed)
% function [H, DHDT1, DHDT2, K]=CBF_Const_TIAGO(n_point_link,x0)
function [H, DHDT1, DHDT2, K]=CBF_Const_TIAGO(n_point_link)
format shortE;
syms x [6 1] real; syms tau [1 3] real; syms t real;

syms x0 [6 1] real; syms x_obs y_obs r_obs real; P_obs = [x_obs y_obs];

dx = TIAGO(t,x,tau,0,0);

%coordinate is (z0,x0), z0-horizontal =, x0-vertical like (x,y)
% P1| z | x 
% P2| z | x 
% P3| z | x 
hb = 0.193;
rt1 = 0.062;
ht = 0.597+0.2;
rt4 = 0.155;
l2 = 0.32;
l3 = 0.34+0.25;
P=cell([4 n_point_link+1]);
H= sym(zeros([4 n_point_link+1]));
P(1,1)= {[x1 , 0]};
H(1,1)= simplify(norm(P{1,1}-P_obs)^2 - (r_obs + (1/(2*n_point_link+2))*(hb + ht))^2);
P(2,1)= {[x1 - rt1 + rt4 , hb + ht]};
H(2,1)= simplify(norm(P{2,1}-P_obs)^2 - (r_obs + (1/(2*n_point_link+2))*l2)^2);
P(3,1)= {[x1 - rt1 + rt4 - l2*sin(x2) , hb + ht + l2*cos(x2)]};
H(3,1)= simplify(norm(P{3,1}-P_obs)^2 - (r_obs + (1/(2*n_point_link+2))*l3)^2);
P(4,1)= {[x1 - rt1 + rt4 - l3*sin(x2 + x3) - l2*sin(x2) , hb + ht + l3*cos(x2 + x3) + l2*cos(x2)]};
H(4,1)= simplify(norm(P{4,1}-P_obs)^2 - (r_obs + (1/(2*n_point_link+2))*l3)^2);

i=1;
while i <= n_point_link
    %link1
    P(1,i+1)= {[x1 - rt1 + rt4 , (i/(n_point_link+1))*(hb + ht)]}; 
    H(1,i+1)= simplify(norm(P{1,i+1}-P_obs)^2 - (r_obs + (1/(2*n_point_link+2))*(hb + ht))^2);
    %link2
    l2_i= (i/(n_point_link+1))*l2;
    P(2,i+1)= {[x1 - rt1 + rt4 - l2_i*sin(x2) , hb + ht + l2_i*cos(x2)]};
    H(2,i+1)= simplify(norm(P{2,i+1}-P_obs)^2 - (r_obs + (1/(2*n_point_link+2))*l2)^2);
    %link3
    l3_i= (i/(n_point_link+1))*l3;
    P(3,i+1)={[x1 - rt1 + rt4 - l3_i*sin(x2 + x3) - l2*sin(x2) , hb + ht + l3_i*cos(x2 + x3) + l2*cos(x2)]};
    H(3,i+1)= simplify(norm(P{3,i+1}-P_obs)^2 - (r_obs + (1/(2*n_point_link+2))*l3)^2);
    i = i+1;
end
DHDT2=cell([4 1]);
DHDT1=cell([4 1]);
K=cell([4 n_point_link+1]);
F=[0 1; 0 0]; 
G=[0;1]; 
P=sym(zeros([1 2]));
% P=zeros([1 2]);
j=1;
while j<= 4
    DHDT1(j)={simplify(jacobian(H(j,:), x)*transpose(dx))};
    DHDT2(j)={simplify(jacobian(DHDT1{j}, x)*transpose(dx))};
    i=1;
    while i<= n_point_link+1

        v0=subs(H(j,i),x,x0);
        dv0=subs(DHDT1{j}(i), x, x0);

        
        P(1) = -(dv0/v0) + 1;
        P(2) = 30;

        %P(1)= max(-(dv0/v0), 35); % eignvalues of F-GK= - [P1 P2]; P>0
        %P(2)= 30;          % assumed and enforced by the control from NMPC.      
        
        
        % Do eigenvalue assignment in closed form solution: easy for a 2-degree
        % polynomial
        K(j,i) = {simplify([P(1)*P(2) P(1)+P(2)])};


        i = i+1;
    end
    j =j+1;
end

end
