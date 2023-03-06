function [d, dot_d, ddot_d]=DistanceNDerivatives(P, r, P_obs, r_obs, x, dx)
    assume(x, 'real')
    O=P_obs;
    vec_d1=(P-O);
    d=simplify(((vec_d1.')*(vec_d1))^(1/2)-(r_obs+r));
    d1=simplify(((vec_d1.')*(vec_d1))^(1/2));
    n=(1/d1)*vec_d1;
    Rb=P-r*n; 
    jac_Rb=simplify(jacobian(Rb, x(1:3)));
    dot_d=simplify(n.'*jac_Rb*(x(4:6)));
    dot_jac_Rb = sym('dot_jac_Rb',size(jac_Rb));
    s = size(jac_Rb);
    rows = s(1);
    columns = s(2);
    for k=1:rows
        for j=1:columns
            pd = jacobian(jac_Rb(k,j), x(1:3)) * (x(4:6));
            dot_jac_Rb(k,j) = simplify(pd);
        end
    end
    ddot_d=(n.'*(dot_jac_Rb*(x(4:6))+jac_Rb*(dx(4:6).')));
    fprintf('Distance and its derivatives calculated for a point.\n')
end
