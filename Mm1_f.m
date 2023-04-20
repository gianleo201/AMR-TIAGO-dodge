function Mm1 = Mm1_f(x1,x2,x3,x4,x5,x6,tau1,tau2,tau3)
%Mm1_f
%    Mm1 = Mm1_f(X1,X2,X3,X4,X5,X6,TAU1,TAU2,TAU3)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    20-Apr-2023 13:05:03

t2 = cos(x2);
t3 = sin(x2);
t4 = x2+x3;
t5 = x2.*2.0;
t6 = x3.*2.0;
t7 = x5.^2;
t8 = x6.^2;
t14 = -x3;
t9 = cos(t5);
t10 = cos(t6);
t11 = sin(t5);
t12 = cos(t4);
t13 = sin(t4);
t15 = t4+x3;
t18 = t14+x2;
t19 = t4.*2.0;
t16 = cos(t15);
t17 = sin(t15);
t20 = cos(t18);
t21 = sin(t18);
t22 = cos(t19);
t23 = sin(t19);
et1 = t9.*9.907318846653355e+2+t10.*4.054192382983532e+4+t11.*1.21424779176056e+4-t22.*1.659649144922511e+2-t23.*2.034077373012169e+3;
et2 = tau1.*(-1.927914909827622e+2)+tau2.*6.026983277740648e+2+t2.*t7.*1.363564812807935e+2-t3.*t7.*6.186688306288662e+2;
et3 = t7.*t12.*1.68066354101675e+1-t7.*t13.*8.107281861669674e+1+t8.*t12.*1.68066354101675e+1-t8.*t13.*8.107281861669674e+1;
et4 = t7.*t16.*(-1.491765192059838e+1)+t7.*t17.*6.635114774351624e+1+t7.*t20.*4.62107954971619e+1-t7.*t21.*2.055376634136275e+2+t8.*t20.*4.62107954971619e+1;
et5 = t8.*t21.*(-2.055376634136275e+2)+t2.*tau2.*1.780593006447637e+3-t2.*tau3.*1.780593006447637e+3+t3.*tau2.*4.284292176429376e+2-t3.*tau3.*4.284292176429376e+2;
et6 = t9.*tau1.*2.764966949190514e+1-t9.*tau2.*2.540162562416641e+1+t10.*tau1.*8.337667501542703e+1-t10.*tau2.*2.767833146975847e+2+t11.*tau1.*4.496087735477456;
et7 = t12.*tau3.*9.572381754318032e+2+t13.*tau3.*2.06143405681619e+2-t16.*tau2.*6.809436344777936e+2+t16.*tau3.*6.809436344777936e+2-t17.*tau2.*1.530957709421016e+2;
et8 = t17.*tau3.*1.530957709421016e+2-t20.*tau3.*1.312224199568917e+3-t22.*tau1.*4.631803118472502-t21.*tau3.*2.950258513481075e+2+t22.*tau2.*4.255216461619202;
et9 = t23.*tau1.*(-7.531733137065991e-1)+t12.*x5.*x6.*3.361327082033501e+1-t13.*x5.*x6.*1.621456372333935e+2+t20.*x5.*x6.*9.24215909943238e+1;
et10 = t21.*x5.*x6.*(-4.11075326827255e+2)-8.973062785871481e+4;
Mm1 = (et1+et2+et3+et4+et5+et6+et7+et8+et9+et10)./(t9.*2.540162562416641e+1+t10.*2.767833146975847e+2-t22.*4.255216461619202-6.026983277740648e+2);
