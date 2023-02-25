
function [d, dot_d, ddot_d]=DistanceNDerivatives4Robot(P_obs,r_obs,n_point_link)

syms x [6 1] real; syms tau [1 3] real; syms t real;
dx = TIAGO(t,x,tau,0,0);


%coordinate is (z0; x0) (column vector), z0-horizontal =, x0-vertical like (x,y)

hb = 0.193;
rt1 = 0.062;
ht = 0.597+0.2;
rt4 = 0.155;
l2 = 0.32;
l3 = 0.34+0.25;
P=cell([4 n_point_link+1]);


d=cell([4 n_point_link+1]);
dot_d=cell([4 n_point_link+1]);
ddot_d=cell([4 n_point_link+1]);

P(1,1)= {[x1, 0]};

[d{1, 1}, dot_d{1,1}, ddot_d{1,1}]=DistanceNDerivatives(P_obs, P{1,1}, (1/(2*n_point_link+2))*(hb + ht), dx, r_obs, x);

P(2,1)= {[x1 - rt1 + rt4, hb + ht]};
[d{2, 1}, dot_d{2,1}, ddot_d{2,1}]=DistanceNDerivatives(P_obs, P{2,1}, (1/(2*n_point_link+2))*l2, dx, r_obs, x);

P(3,1)= {[x1 - rt1 + rt4 - l2*sin(x2), hb + ht + l2*cos(x2)]};
[d{3, 1}, dot_d{3,1}, ddot_d{3,1}]=DistanceNDerivatives(P_obs, P{3,1}, (1/(2*n_point_link+2))*l3, dx, r_obs, x);

P(4,1)= {[x1 - rt1 + rt4 - l3*sin(x2 + x3) - l2*sin(x2), hb + ht + l3*cos(x2 + x3) + l2*cos(x2)]};
[d{4, 1}, dot_d{4,1}, ddot_d{4,1}]=DistanceNDerivatives(P_obs, P{4,1}, (1/(2*n_point_link+2))*l3, dx, r_obs, x);

i=1;
while i <= n_point_link
    %link1
    P(1,i+1)= {[x1 - rt1 + rt4, (i/(n_point_link+1))*(hb + ht)]}; 
    [d{1, i+1}, dot_d{1,i+1}, ddot_d{1,i+1}]=DistanceNDerivatives(P_obs, P{1,i+1}, (1/(2*n_point_link+2))*(hb+ht), dx, r_obs, x);
    %link2
    l2_i= (i/(n_point_link+1))*l2;
    P(2,i+1)= {[x1 - rt1 + rt4 - l2_i*sin(x2), hb + ht + l2_i*cos(x2)]};
    [d{2, i+1}, dot_d{2,i+1}, ddot_d{2,i+1}]=DistanceNDerivatives(P_obs, P{2,i+1}, (1/(2*n_point_link+2))*l2, dx, r_obs, x);
    %link3
    l3_i= (i/(n_point_link+1))*l3;
    P(3,i+1)={[x1 - rt1 + rt4 - l3_i*sin(x2 + x3) - l2*sin(x2), hb + ht + l3_i*cos(x2 + x3) + l2*cos(x2)]};
    [d{3, i+1}, dot_d{3,i+1}, ddot_d{3,i+1}]=DistanceNDerivatives(P_obs, P{3,i+1}, (1/(2*n_point_link+2))*l3, dx, r_obs, x);
    i = i+1;
end
end