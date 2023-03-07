function [d, dot_d, ddot_d, d0]=SimplifiedDistanceNDerivatives4Robot(P_obs,r_obs, x0)

syms x [6 1] real; syms tau [1 3] real; syms t real;
dx = TIAGO(t,x,tau,0,0);


%coordinate is (x; y) (column vector), x -->horizontal, y --> vertical


hb = 0.193;
rt1 = 0.062;
ht = 0.597+0.2;
rt4 = 0.155;
l2 = 0.32;
l3 = 0.34+0.25;

P = sym('P', [2 1 3]);
d = sym('d', [1, 3]);
d0 = sym('d0', [1 3]);
dot_d = sym('dot_d', [1, 3]);
ddot_d = sym('ddot_d', [1, 3]);
r = sym('r', [1 3]);

r(1) = (hb+ht)/2;
r(2) = l2/2;
r(3)= l3/2;

P(:, :, 1) = [x1; (hb+ht)/2];
P(:, :, 2) = [x1 - rt1 + rt4 - l2/2*sin(x2) ; hb + ht + l2/2*cos(x2)];
P(:, :, 3) = [x1 - rt1 + rt4 - l3/2*sin(x2 + x3) - l2*sin(x2); hb + ht + l3/2*cos(x2 + x3) + l2*cos(x2)];

for i=1:3
    [d(i), dot_d(i), ddot_d(i)]=DistanceNDerivatives(P(:, :, i), r(i), P_obs, r_obs, x, dx);
    d0(i)=subs(d(i), x, x0);
end


end
