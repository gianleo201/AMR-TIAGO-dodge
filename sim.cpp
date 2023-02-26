/*
*    This file is part of ACADO Toolkit.
*
*    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
*    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
*    Developed within the Optimization in Engineering Center (OPTEC) under
*    supervision of Moritz Diehl. All rights reserved.
*
*    ACADO Toolkit is free software; you can redistribute it and/or
*    modify it under the terms of the GNU Lesser General Public
*    License as published by the Free Software Foundation; either
*    version 3 of the License, or (at your option) any later version.
*
*    ACADO Toolkit is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*    Lesser General Public License for more details.
*
*    You should have received a copy of the GNU Lesser General Public
*    License along with ACADO Toolkit; if not, write to the Free Software
*    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
*/


/**
*    Author David Ariens, Rien Quirynen
*    Date 2009-2013
*    http://www.acadotoolkit.org/matlab 
*/

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado/utils/matlab_acado_utils.hpp>

USING_NAMESPACE_ACADO

#include <mex.h>


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) 
 { 
 
    MatlabConsoleStreamBuf mybuf;
    RedirectStream redirect(std::cout, mybuf);
    clearAllStaticCounters( ); 
 
    mexPrintf("\nACADO Toolkit for Matlab - Developed by David Ariens and Rien Quirynen, 2009-2013 \n"); 
    mexPrintf("Support available at http://www.acadotoolkit.org/matlab \n \n"); 

    if (nrhs != 0){ 
      mexErrMsgTxt("This problem expects 0 right hand side argument(s) since you have defined 0 MexInput(s)");
    } 
 
    TIME autotime;
    DifferentialState x1;
    DifferentialState x2;
    DifferentialState x3;
    DifferentialState x4;
    DifferentialState x5;
    DifferentialState x6;
    Control tau1;
    Control tau2;
    Control tau3;
    OnlineData x_obs; 
    OnlineData y_obs; 
    OnlineData r_obs; 
    OnlineData x01; 
    OnlineData x02; 
    OnlineData x03; 
    OnlineData x04; 
    OnlineData x05; 
    OnlineData x06; 
    SIMexport ExportModule1( 1, 0.01 );
    ExportModule1.set( GENERATE_MATLAB_INTERFACE, 1 );
    uint options_flag;
    options_flag = ExportModule1.set( INTEGRATOR_TYPE, INT_IRK_RIIA5 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: INTEGRATOR_TYPE");
    options_flag = ExportModule1.set( NUM_INTEGRATOR_STEPS, 5 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: NUM_INTEGRATOR_STEPS");
    DifferentialEquation acadodata_f1;
    acadodata_f1 << dot(x1) == x4;
    acadodata_f1 << dot(x2) == x5;
    acadodata_f1 << dot(x3) == x6;
    acadodata_f1 << dot(x4) == 1/(1.03218025458124841975e+02-1.31010359040000068376e+00*cos((2.00000000000000000000e+00*x2+2.00000000000000000000e+00*x3))-1.54156400640000051938e-01*cos(2.00000000000000000000e+00*x2)-2.39409561600000070358e-01*cos(2.00000000000000000000e+00*x2)+3.43771182120960183681e+00*cos((2.00000000000000000000e+00*x2+2.00000000000000000000e+00*x3))+4.68778884710400163272e+01-5.33887063142400286608e+00*cos(2.00000000000000000000e+00*x2)-6.03695734456320280970e+01*cos(2.00000000000000000000e+00*x3)-6.87542364241920367363e+00*cos(2.00000000000000000000e+00*x2)-6.87542364241920367363e+00*cos(2.00000000000000000000e+00*x3)-9.29525760000000228489e-02*cos(2.00000000000000000000e+00*x2))*(1.08513630720000051610e+01*cos(x2)*tau2-1.08513630720000051610e+01*cos(x2)*tau3+1.10006778278707262331e+00*pow(x5,2.00000000000000000000e+00)*sin((2.00000000000000000000e+00*x3+x2))-1.13356800000000007556e-02*pow(x5,2.00000000000000000000e+00)*sin(x2)-1.14916589568000063193e-01*pow(x5,2.00000000000000000000e+00)*sin(x2)+1.15199999999999989908e-02*tau1+1.28521162218240068142e+01*sin((2.00000000000000000000e+00*x2+2.00000000000000000000e+00*x3))-1.34576640000000028602e-02*pow(x5,2.00000000000000000000e+00)*sin((x2+x3))-1.34576640000000028602e-02*pow(x6,2.00000000000000000000e+00)*sin((x2+x3))-1.45981440000000003976e-02*pow(x5,2.00000000000000000000e+00)*sin(x2)+1.51227429027840054587e+00*sin(2.00000000000000000000e+00*x2)-1.54592223667200090453e+00*sin((x2+x3))*x5*x6-1.70843860205568121025e+00*pow(x5,2.00000000000000000000e+00)*sin(x2)+1.73621809152000072807e+00*tau1+1.88928000000000012593e-01*cos(x2)*tau2-1.88928000000000012593e-01*cos(x2)*tau3-2.02824997451366551005e+00*pow(x5,2.00000000000000000000e+00)*sin((x2-x3))-2.02824997451366551005e+00*pow(x6,2.00000000000000000000e+00)*sin((x2-x3))-2.23591012761600094194e+00*cos(2.00000000000000000000e+00*x3)*tau1+2.23591012761600094194e+00*tau1+2.24294400000000049289e+00*cos((x2+x3))*tau3+2.34860779929600083094e+00*sin(2.00000000000000000000e+00*x2)+2.43302400000000029756e-01*cos(x2)*tau2-2.43302400000000029756e-01*cos(x2)*tau3-2.69153280000000057204e-02*sin((x2+x3))*x5*x6-2.97448243200000086994e-02*pow(x5,2.00000000000000000000e+00)*sin(x2)+3.02284800000000054843e-02*tau1-3.30020334836121786992e+00*pow(x5,2.00000000000000000000e+00)*sin(x2)-3.37239529660661929711e+01*sin((2.00000000000000000000e+00*x2+2.00000000000000000000e+00*x3))-3.53129103360000154432e-02*pow(x5,2.00000000000000000000e+00)*sin((x2-x3))-3.53129103360000154432e-02*pow(x6,2.00000000000000000000e+00)*sin((x2-x3))-4.05649994902733102009e+00*sin((x2-x3))*x5*x6-4.19233148928000121103e-01*pow(x5,2.00000000000000000000e+00)*sin((2.00000000000000000000e+00*x3+x2))-4.19233148928000121103e-01*pow(x5,2.00000000000000000000e+00)*sin(x2)-4.54761381888000165707e-02*pow(x5,2.00000000000000000000e+00)*sin((x2+x3))-4.54761381888000165707e-02*pow(x5,2.00000000000000000000e+00)*sin((x2-x3))-4.54761381888000165707e-02*pow(x6,2.00000000000000000000e+00)*sin((x2+x3))-4.54761381888000165707e-02*pow(x6,2.00000000000000000000e+00)*sin((x2-x3))+5.23743208942694664643e+01*sin(2.00000000000000000000e+00*x2)-5.88548505600000204652e+00*cos((x2-x3))*tau3-6.51081784320000189759e-01*pow(x5,2.00000000000000000000e+00)*sin(x2)+6.61668480000000225338e-01*tau1+6.74479059321323859422e+01*sin(2.00000000000000000000e+00*x2)-6.98721914880000305459e+00*cos((2.00000000000000000000e+00*x3+x2))*tau2+6.98721914880000305459e+00*cos((2.00000000000000000000e+00*x3+x2))*tau3+6.98721914880000305459e+00*cos(x2)*tau2-6.98721914880000305459e+00*cos(x2)*tau3-7.06258206720000308865e-02*sin((x2-x3))*x5*x6+7.57935636480000329840e+00*cos((x2+x3))*tau3-7.57935636480000329840e+00*cos((x2-x3))*tau3-7.72961118336000452267e-01*pow(x5,2.00000000000000000000e+00)*sin((x2+x3))-7.72961118336000452267e-01*pow(x6,2.00000000000000000000e+00)*sin((x2+x3))+7.78567680000000206242e-02*tau1-9.09522763776000331415e-02*sin((x2+x3))*x5*x6-9.09522763776000331415e-02*sin((x2-x3))*x5*x6+9.11864770560000259536e-01*sin(2.00000000000000000000e+00*x2)-9.86600964096000371262e-02*pow(x5,2.00000000000000000000e+00)*sin(x2));
    acadodata_f1 << dot(x5) == (-1.05387353018956858364e+02*sin((2.00000000000000000000e+00*x3+x2))-1.09175299200000051059e+01*cos((2.00000000000000000000e+00*x2+2.00000000000000000000e+00*x3))*tau2+1.09175299200000051059e+01*cos((2.00000000000000000000e+00*x2+2.00000000000000000000e+00*x3))*tau3+1.09175299200000051059e+01*tau2-1.09175299200000051059e+01*tau3+1.10090990592000039783e+01*sin(x2)-1.10352844800000043923e-01*sin((2.00000000000000000000e+00*x2+x3))*x5*x6+1.11306401040384074008e+02*sin(x3)*x5*x6+1.18427443200000030998e+01*cos((2.00000000000000000000e+00*x2+x3))*tau3-1.18427443200000030998e+01*cos(x3)*tau3-1.19704780800000035179e-01*pow(x5,2.00000000000000000000e+00)*sin(2.00000000000000000000e+00*x2)+1.21651200000000014878e-01*cos(x2)*tau1-1.42112931840000067396e-01*sin((2.00000000000000000000e+00*x2+x3))*x5*x6+1.42112931840000067396e-01*sin(x3)*x5*x6+1.43710026844032063309e+03*sin(x2)+1.48875408000000049924e+02*tau2-1.48875408000000049924e+02*tau3-1.61491968000000042593e+02*cos(x3)*tau3+1.65529267200000052007e-01*pow(x5,2.00000000000000000000e+00)*sin(x3)+1.65529267200000052007e-01*pow(x6,2.00000000000000000000e+00)*sin(x3)+1.90148435110656137681e+01*sin(x3)*x5*x6+1.90148435110656137681e+01/2.00000000000000000000e+00*pow(x5,2.00000000000000000000e+00)*sin(x3)+1.90148435110656137681e+01/2.00000000000000000000e+00*pow(x6,2.00000000000000000000e+00)*sin(x3)+1.93790361600000049513e+00*sin(x3)*x5*x6-1/2.00000000000000000000e+00*5.33887063142400286608e+00*pow(x5,2.00000000000000000000e+00)*sin(2.00000000000000000000e+00*x2)-1/2.00000000000000000000e+00*6.33828117035520488542e+00*pow(x5,2.00000000000000000000e+00)*sin((2.00000000000000000000e+00*x2+x3))-1/2.00000000000000000000e+00*6.33828117035520488542e+00*pow(x6,2.00000000000000000000e+00)*sin((2.00000000000000000000e+00*x2+x3))+2.50206796800000006442e+01*sin(x2)+2.59200000000000008171e+00*tau2-2.59200000000000008171e+00*tau3-2.75882112000000034868e+01*cos(x3)*tau3+3.01847867228160140485e+01*pow(x5,2.00000000000000000000e+00)*sin(2.00000000000000000000e+00*x3)+3.16162059056870532459e+02*sin(x2)+3.22217533439999996858e+01*sin(x2)+3.27339505589184170731e+02*sin(x2)+3.31058534400000104014e-01*sin(x3)*x5*x6+3.39105096000000187928e+01*tau2-3.39105096000000187928e+01*tau3-3.43771182120960183681e+00*pow(x5,2.00000000000000000000e+00)*sin(2.00000000000000000000e+00*x2)+3.43771182120960183681e+00*pow(x5,2.00000000000000000000e+00)*sin(2.00000000000000000000e+00*x3)-3.49360957440000152729e+00*cos((2.00000000000000000000e+00*x3+x2))*tau1+3.49360957440000152729e+00*cos(x2)*tau1+3.80160000000000053433e-01*tau2-3.80160000000000053433e-01*tau3-4.64762880000000114245e-02*pow(x5,2.00000000000000000000e+00)*sin(2.00000000000000000000e+00*x2)+4.72585715712000098421e+00*sin(x2)+5.42568153600000258052e+00*cos(x2)*tau1-5.51764224000000219617e-02*pow(x5,2.00000000000000000000e+00)*sin((2.00000000000000000000e+00*x2+x3))-5.51764224000000219617e-02*pow(x6,2.00000000000000000000e+00)*sin((2.00000000000000000000e+00*x2+x3))+5.56532005201920370041e+01*pow(x5,2.00000000000000000000e+00)*sin(x3)+5.56532005201920370041e+01*pow(x6,2.00000000000000000000e+00)*sin(x3)+5.69915481600000095597e+00*sin(x2)+5.90400000000000035882e-01*tau2-5.90400000000000035882e-01*tau3-6.33828117035520488542e+00*sin((2.00000000000000000000e+00*x2+x3))*x5*x6-7.10564659200000336980e-02*pow(x5,2.00000000000000000000e+00)*sin((2.00000000000000000000e+00*x2+x3))+7.10564659200000336980e-02*pow(x5,2.00000000000000000000e+00)*sin(x3)-7.10564659200000336980e-02*pow(x6,2.00000000000000000000e+00)*sin((2.00000000000000000000e+00*x2+x3))+7.10564659200000336980e-02*pow(x6,2.00000000000000000000e+00)*sin(x3)-7.70782003200000259691e-02*pow(x5,2.00000000000000000000e+00)*sin(2.00000000000000000000e+00*x2)+9.19607040000000175439e+00*cos((2.00000000000000000000e+00*x2+x3))*tau3-9.25352367971328476415e+02*sin((2.00000000000000000000e+00*x3+x2))+9.25352367971328476415e+02*sin(x2)+9.44640000000000062963e-02*cos(x2)*tau1+9.68951808000000247567e-01*pow(x5,2.00000000000000000000e+00)*sin(x3)+9.68951808000000247567e-01*pow(x6,2.00000000000000000000e+00)*sin(x3))/(1.03218025458124841975e+02-1.31010359040000068376e+00*cos((2.00000000000000000000e+00*x2+2.00000000000000000000e+00*x3))-1.54156400640000051938e-01*cos(2.00000000000000000000e+00*x2)-2.39409561600000070358e-01*cos(2.00000000000000000000e+00*x2)+3.43771182120960183681e+00*cos((2.00000000000000000000e+00*x2+2.00000000000000000000e+00*x3))+4.68778884710400163272e+01-5.33887063142400286608e+00*cos(2.00000000000000000000e+00*x2)-6.03695734456320280970e+01*cos(2.00000000000000000000e+00*x3)-6.87542364241920367363e+00*cos(2.00000000000000000000e+00*x2)-6.87542364241920367363e+00*cos(2.00000000000000000000e+00*x3)-9.29525760000000228489e-02*cos(2.00000000000000000000e+00*x2))*2.00000000000000000000e+00;
    acadodata_f1 << dot(x6) == (-((1/2.00000000000000000000e+00*5.90000000000000079936e-01*sin((x2+x3))+3.20000000000000006661e-01*sin(x2))*3.88476000000000070145e+01+(2.00000000000000000000e+00*x5+x6)/2.00000000000000000000e+00*7.47648000000000201304e-01*sin(x3)*x6+1.93060800000000014620e+01/2.00000000000000000000e+00*sin(x2)+tau2)/(1.03218025458124841975e+02-1.31010359040000068376e+00*cos((2.00000000000000000000e+00*x2+2.00000000000000000000e+00*x3))-1.54156400640000051938e-01*cos(2.00000000000000000000e+00*x2)-2.39409561600000070358e-01*cos(2.00000000000000000000e+00*x2)+3.43771182120960183681e+00*cos((2.00000000000000000000e+00*x2+2.00000000000000000000e+00*x3))+4.68778884710400163272e+01-5.33887063142400286608e+00*cos(2.00000000000000000000e+00*x2)-6.03695734456320280970e+01*cos(2.00000000000000000000e+00*x3)-6.87542364241920367363e+00*cos(2.00000000000000000000e+00*x2)-6.87542364241920367363e+00*cos(2.00000000000000000000e+00*x3)-9.29525760000000228489e-02*cos(2.00000000000000000000e+00*x2))*(1.37941056000000017434e+01*cos(x3)-4.59803520000000087720e+00*cos((2.00000000000000000000e+00*x2+x3))-5.45876496000000255293e+00*cos((2.00000000000000000000e+00*x2+2.00000000000000000000e+00*x3))-5.92137216000000154992e+00*cos((2.00000000000000000000e+00*x2+x3))+5.92137216000000154992e+00*cos(x3)+8.07459840000000212967e+01*cos(x3)+9.86330037600000366638e+01)*4.00000000000000000000e+00+(-1.18427443200000030998e+01*cos((2.00000000000000000000e+00*x2+x3))+1.18427443200000030998e+01*cos(x3)+1.61491968000000042593e+02*cos(x3)+2.63301012720000073841e+02+2.75882112000000034868e+01*cos(x3)-3.87302400000000091040e+00*cos(2.00000000000000000000e+00*x2)-5.45876496000000255293e+00*cos((2.00000000000000000000e+00*x2+2.00000000000000000000e+00*x3))-6.42318336000000122965e+00*cos(2.00000000000000000000e+00*x2)-9.19607040000000175439e+00*cos((2.00000000000000000000e+00*x2+x3))-9.97539840000000310738e+00*cos(2.00000000000000000000e+00*x2))/(1.03218025458124841975e+02-1.31010359040000068376e+00*cos((2.00000000000000000000e+00*x2+2.00000000000000000000e+00*x3))-1.54156400640000051938e-01*cos(2.00000000000000000000e+00*x2)-2.39409561600000070358e-01*cos(2.00000000000000000000e+00*x2)+3.43771182120960183681e+00*cos((2.00000000000000000000e+00*x2+2.00000000000000000000e+00*x3))+4.68778884710400163272e+01-5.33887063142400286608e+00*cos(2.00000000000000000000e+00*x2)-6.03695734456320280970e+01*cos(2.00000000000000000000e+00*x3)-6.87542364241920367363e+00*cos(2.00000000000000000000e+00*x2)-6.87542364241920367363e+00*cos(2.00000000000000000000e+00*x3)-9.29525760000000228489e-02*cos(2.00000000000000000000e+00*x2))*(1/2.00000000000000000000e+00*2.29200840000000063412e+01*sin((x2+x3))-1/2.00000000000000000000e+00*7.47648000000000201304e-01*pow(x5,2.00000000000000000000e+00)*sin(x3)+tau3)*4.00000000000000000000e+00+1/(1.03218025458124841975e+02-1.31010359040000068376e+00*cos((2.00000000000000000000e+00*x2+2.00000000000000000000e+00*x3))-1.54156400640000051938e-01*cos(2.00000000000000000000e+00*x2)-2.39409561600000070358e-01*cos(2.00000000000000000000e+00*x2)+3.43771182120960183681e+00*cos((2.00000000000000000000e+00*x2+2.00000000000000000000e+00*x3))+4.68778884710400163272e+01-5.33887063142400286608e+00*cos(2.00000000000000000000e+00*x2)-6.03695734456320280970e+01*cos(2.00000000000000000000e+00*x3)-6.87542364241920367363e+00*cos(2.00000000000000000000e+00*x2)-6.87542364241920367363e+00*cos(2.00000000000000000000e+00*x3)-9.29525760000000228489e-02*cos(2.00000000000000000000e+00*x2))*(1.26720000000000010409e+00*pow(x5,2.00000000000000000000e+00)*sin(x2)+1.96800000000000019362e+00/2.00000000000000000000e+00*pow(x5,2.00000000000000000000e+00)*sin(x2)+1/2.00000000000000000000e+00*2.33640000000000069846e+00*pow(x5,2.00000000000000000000e+00)*sin((x2+x3))+1/2.00000000000000000000e+00*2.33640000000000069846e+00*pow(x6,2.00000000000000000000e+00)*sin((x2+x3))+2.33640000000000069846e+00*sin((x2+x3))*x5*x6-tau1)*(1.47137126400000051163e+00*cos((x2-x3))-1.74680478720000076365e+00*cos((2.00000000000000000000e+00*x3+x2))+1.74680478720000076365e+00*cos(x2)-1.89483909120000082460e+00*cos((x2+x3))+1.89483909120000082460e+00*cos((x2-x3))+2.71284076800000129026e+00*cos(x2)+4.72320000000000031481e-02*cos(x2)-5.60736000000000123222e-01*cos((x2+x3))+6.08256000000000074390e-02*cos(x2))*4.00000000000000000000e+00);

    ExportModule1.setModel( acadodata_f1 );

    uint export_flag = 0;
    ExportModule1.setTimingSteps( 0 );
    export_flag = ExportModule1.exportCode( "export_SIM" );
    if(export_flag != 0) mexErrMsgTxt("ACADO export failed because of the above error(s)!");


    clearAllStaticCounters( ); 
 
} 

