/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 6];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 6 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 6 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 6 + 3];
acadoWorkspace.state[4] = acadoVariables.x[lRun1 * 6 + 4];
acadoWorkspace.state[5] = acadoVariables.x[lRun1 * 6 + 5];

acadoWorkspace.state[60] = acadoVariables.u[lRun1 * 3];
acadoWorkspace.state[61] = acadoVariables.u[lRun1 * 3 + 1];
acadoWorkspace.state[62] = acadoVariables.u[lRun1 * 3 + 2];
acadoWorkspace.state[63] = acadoVariables.od[lRun1 * 3];
acadoWorkspace.state[64] = acadoVariables.od[lRun1 * 3 + 1];
acadoWorkspace.state[65] = acadoVariables.od[lRun1 * 3 + 2];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 6] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 6 + 6];
acadoWorkspace.d[lRun1 * 6 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 6 + 7];
acadoWorkspace.d[lRun1 * 6 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 6 + 8];
acadoWorkspace.d[lRun1 * 6 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 6 + 9];
acadoWorkspace.d[lRun1 * 6 + 4] = acadoWorkspace.state[4] - acadoVariables.x[lRun1 * 6 + 10];
acadoWorkspace.d[lRun1 * 6 + 5] = acadoWorkspace.state[5] - acadoVariables.x[lRun1 * 6 + 11];

acadoWorkspace.evGx[lRun1 * 36] = acadoWorkspace.state[6];
acadoWorkspace.evGx[lRun1 * 36 + 1] = acadoWorkspace.state[7];
acadoWorkspace.evGx[lRun1 * 36 + 2] = acadoWorkspace.state[8];
acadoWorkspace.evGx[lRun1 * 36 + 3] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 36 + 4] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 36 + 5] = acadoWorkspace.state[11];
acadoWorkspace.evGx[lRun1 * 36 + 6] = acadoWorkspace.state[12];
acadoWorkspace.evGx[lRun1 * 36 + 7] = acadoWorkspace.state[13];
acadoWorkspace.evGx[lRun1 * 36 + 8] = acadoWorkspace.state[14];
acadoWorkspace.evGx[lRun1 * 36 + 9] = acadoWorkspace.state[15];
acadoWorkspace.evGx[lRun1 * 36 + 10] = acadoWorkspace.state[16];
acadoWorkspace.evGx[lRun1 * 36 + 11] = acadoWorkspace.state[17];
acadoWorkspace.evGx[lRun1 * 36 + 12] = acadoWorkspace.state[18];
acadoWorkspace.evGx[lRun1 * 36 + 13] = acadoWorkspace.state[19];
acadoWorkspace.evGx[lRun1 * 36 + 14] = acadoWorkspace.state[20];
acadoWorkspace.evGx[lRun1 * 36 + 15] = acadoWorkspace.state[21];
acadoWorkspace.evGx[lRun1 * 36 + 16] = acadoWorkspace.state[22];
acadoWorkspace.evGx[lRun1 * 36 + 17] = acadoWorkspace.state[23];
acadoWorkspace.evGx[lRun1 * 36 + 18] = acadoWorkspace.state[24];
acadoWorkspace.evGx[lRun1 * 36 + 19] = acadoWorkspace.state[25];
acadoWorkspace.evGx[lRun1 * 36 + 20] = acadoWorkspace.state[26];
acadoWorkspace.evGx[lRun1 * 36 + 21] = acadoWorkspace.state[27];
acadoWorkspace.evGx[lRun1 * 36 + 22] = acadoWorkspace.state[28];
acadoWorkspace.evGx[lRun1 * 36 + 23] = acadoWorkspace.state[29];
acadoWorkspace.evGx[lRun1 * 36 + 24] = acadoWorkspace.state[30];
acadoWorkspace.evGx[lRun1 * 36 + 25] = acadoWorkspace.state[31];
acadoWorkspace.evGx[lRun1 * 36 + 26] = acadoWorkspace.state[32];
acadoWorkspace.evGx[lRun1 * 36 + 27] = acadoWorkspace.state[33];
acadoWorkspace.evGx[lRun1 * 36 + 28] = acadoWorkspace.state[34];
acadoWorkspace.evGx[lRun1 * 36 + 29] = acadoWorkspace.state[35];
acadoWorkspace.evGx[lRun1 * 36 + 30] = acadoWorkspace.state[36];
acadoWorkspace.evGx[lRun1 * 36 + 31] = acadoWorkspace.state[37];
acadoWorkspace.evGx[lRun1 * 36 + 32] = acadoWorkspace.state[38];
acadoWorkspace.evGx[lRun1 * 36 + 33] = acadoWorkspace.state[39];
acadoWorkspace.evGx[lRun1 * 36 + 34] = acadoWorkspace.state[40];
acadoWorkspace.evGx[lRun1 * 36 + 35] = acadoWorkspace.state[41];

acadoWorkspace.evGu[lRun1 * 18] = acadoWorkspace.state[42];
acadoWorkspace.evGu[lRun1 * 18 + 1] = acadoWorkspace.state[43];
acadoWorkspace.evGu[lRun1 * 18 + 2] = acadoWorkspace.state[44];
acadoWorkspace.evGu[lRun1 * 18 + 3] = acadoWorkspace.state[45];
acadoWorkspace.evGu[lRun1 * 18 + 4] = acadoWorkspace.state[46];
acadoWorkspace.evGu[lRun1 * 18 + 5] = acadoWorkspace.state[47];
acadoWorkspace.evGu[lRun1 * 18 + 6] = acadoWorkspace.state[48];
acadoWorkspace.evGu[lRun1 * 18 + 7] = acadoWorkspace.state[49];
acadoWorkspace.evGu[lRun1 * 18 + 8] = acadoWorkspace.state[50];
acadoWorkspace.evGu[lRun1 * 18 + 9] = acadoWorkspace.state[51];
acadoWorkspace.evGu[lRun1 * 18 + 10] = acadoWorkspace.state[52];
acadoWorkspace.evGu[lRun1 * 18 + 11] = acadoWorkspace.state[53];
acadoWorkspace.evGu[lRun1 * 18 + 12] = acadoWorkspace.state[54];
acadoWorkspace.evGu[lRun1 * 18 + 13] = acadoWorkspace.state[55];
acadoWorkspace.evGu[lRun1 * 18 + 14] = acadoWorkspace.state[56];
acadoWorkspace.evGu[lRun1 * 18 + 15] = acadoWorkspace.state[57];
acadoWorkspace.evGu[lRun1 * 18 + 16] = acadoWorkspace.state[58];
acadoWorkspace.evGu[lRun1 * 18 + 17] = acadoWorkspace.state[59];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 6;
/* Vector of auxiliary variables; number of elements: 20. */
real_t* a = acadoWorkspace.objAuxVar;

/* Compute intermediate quantities: */
a[0] = (sin(xd[1]));
a[1] = (sin((xd[1]+xd[2])));
a[2] = (cos(xd[1]));
a[3] = (cos((xd[1]+xd[2])));
a[4] = (cos(xd[1]));
a[5] = (cos((xd[1]+xd[2])));
a[6] = (cos((xd[1]+xd[2])));
a[7] = (sin(xd[1]));
a[8] = (sin((xd[1]+xd[2])));
a[9] = (sin((xd[1]+xd[2])));
a[10] = (cos(xd[1]));
a[11] = (cos((xd[1]+xd[2])));
a[12] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[1])));
a[13] = ((real_t)(-1.0000000000000000e+00)*(sin((xd[1]+xd[2]))));
a[14] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[1])));
a[15] = ((real_t)(-1.0000000000000000e+00)*(sin((xd[1]+xd[2]))));
a[16] = ((real_t)(-1.0000000000000000e+00)*(sin((xd[1]+xd[2]))));
a[17] = (cos(xd[1]));
a[18] = (cos((xd[1]+xd[2])));
a[19] = (cos((xd[1]+xd[2])));

/* Compute outputs: */
out[0] = (((((real_t)(1.5500000000000000e-01)-((real_t)(3.2000000000000001e-01)*a[0]))-((real_t)(5.9000000000000008e-01)*a[1]))-(real_t)(6.2000000000000000e-02))+xd[0]);
out[1] = ((((real_t)(3.2000000000000001e-01)*a[2])+((real_t)(5.9000000000000008e-01)*a[3]))+(real_t)(9.8999999999999999e-01));
out[2] = (((((real_t)(0.0000000000000000e+00)-(((real_t)(3.2000000000000001e-01)*a[4])+((real_t)(5.9000000000000008e-01)*a[5])))*xd[4])-(((real_t)(5.9000000000000008e-01)*a[6])*xd[5]))+xd[3]);
out[3] = ((((real_t)(0.0000000000000000e+00)-xd[4])*(((real_t)(3.2000000000000001e-01)*a[7])+((real_t)(5.9000000000000008e-01)*a[8])))-(((real_t)(5.9000000000000008e-01)*a[9])*xd[5]));
out[4] = xd[3];
out[5] = xd[4];
out[6] = xd[5];
out[7] = u[0];
out[8] = u[1];
out[9] = u[2];
out[10] = (real_t)(1.0000000000000000e+00);
out[11] = (((real_t)(0.0000000000000000e+00)-((real_t)(3.2000000000000001e-01)*a[10]))-((real_t)(5.9000000000000008e-01)*a[11]));
out[12] = ((real_t)(0.0000000000000000e+00)-((real_t)(5.9000000000000008e-01)*a[11]));
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = (real_t)(0.0000000000000000e+00);
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (((real_t)(3.2000000000000001e-01)*a[12])+((real_t)(5.9000000000000008e-01)*a[13]));
out[18] = ((real_t)(5.9000000000000008e-01)*a[13]);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = ((((real_t)(0.0000000000000000e+00)-(((real_t)(3.2000000000000001e-01)*a[14])+((real_t)(5.9000000000000008e-01)*a[15])))*xd[4])-(((real_t)(5.9000000000000008e-01)*a[16])*xd[5]));
out[24] = ((((real_t)(0.0000000000000000e+00)-((real_t)(5.9000000000000008e-01)*a[15]))*xd[4])-(((real_t)(5.9000000000000008e-01)*a[16])*xd[5]));
out[25] = (real_t)(1.0000000000000000e+00);
out[26] = ((real_t)(0.0000000000000000e+00)-(((real_t)(3.2000000000000001e-01)*a[4])+((real_t)(5.9000000000000008e-01)*a[5])));
out[27] = ((real_t)(0.0000000000000000e+00)-((real_t)(5.9000000000000008e-01)*a[6]));
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = ((((real_t)(0.0000000000000000e+00)-xd[4])*(((real_t)(3.2000000000000001e-01)*a[17])+((real_t)(5.9000000000000008e-01)*a[18])))-(((real_t)(5.9000000000000008e-01)*a[19])*xd[5]));
out[30] = ((((real_t)(0.0000000000000000e+00)-xd[4])*((real_t)(5.9000000000000008e-01)*a[18]))-(((real_t)(5.9000000000000008e-01)*a[19])*xd[5]));
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*(((real_t)(3.2000000000000001e-01)*a[7])+((real_t)(5.9000000000000008e-01)*a[8])));
out[33] = ((real_t)(0.0000000000000000e+00)-((real_t)(5.9000000000000008e-01)*a[9]));
out[34] = (real_t)(0.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = (real_t)(1.0000000000000000e+00);
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = (real_t)(0.0000000000000000e+00);
out[40] = (real_t)(0.0000000000000000e+00);
out[41] = (real_t)(0.0000000000000000e+00);
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = (real_t)(0.0000000000000000e+00);
out[44] = (real_t)(1.0000000000000000e+00);
out[45] = (real_t)(0.0000000000000000e+00);
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (real_t)(0.0000000000000000e+00);
out[49] = (real_t)(0.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(1.0000000000000000e+00);
out[52] = (real_t)(0.0000000000000000e+00);
out[53] = (real_t)(0.0000000000000000e+00);
out[54] = (real_t)(0.0000000000000000e+00);
out[55] = (real_t)(0.0000000000000000e+00);
out[56] = (real_t)(0.0000000000000000e+00);
out[57] = (real_t)(0.0000000000000000e+00);
out[58] = (real_t)(0.0000000000000000e+00);
out[59] = (real_t)(0.0000000000000000e+00);
out[60] = (real_t)(0.0000000000000000e+00);
out[61] = (real_t)(0.0000000000000000e+00);
out[62] = (real_t)(0.0000000000000000e+00);
out[63] = (real_t)(0.0000000000000000e+00);
out[64] = (real_t)(0.0000000000000000e+00);
out[65] = (real_t)(0.0000000000000000e+00);
out[66] = (real_t)(0.0000000000000000e+00);
out[67] = (real_t)(0.0000000000000000e+00);
out[68] = (real_t)(0.0000000000000000e+00);
out[69] = (real_t)(0.0000000000000000e+00);
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;
/* Vector of auxiliary variables; number of elements: 20. */
real_t* a = acadoWorkspace.objAuxVar;

/* Compute intermediate quantities: */
a[0] = (sin(xd[1]));
a[1] = (sin((xd[1]+xd[2])));
a[2] = (cos(xd[1]));
a[3] = (cos((xd[1]+xd[2])));
a[4] = (cos(xd[1]));
a[5] = (cos((xd[1]+xd[2])));
a[6] = (cos((xd[1]+xd[2])));
a[7] = (sin(xd[1]));
a[8] = (sin((xd[1]+xd[2])));
a[9] = (sin((xd[1]+xd[2])));
a[10] = (cos(xd[1]));
a[11] = (cos((xd[1]+xd[2])));
a[12] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[1])));
a[13] = ((real_t)(-1.0000000000000000e+00)*(sin((xd[1]+xd[2]))));
a[14] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[1])));
a[15] = ((real_t)(-1.0000000000000000e+00)*(sin((xd[1]+xd[2]))));
a[16] = ((real_t)(-1.0000000000000000e+00)*(sin((xd[1]+xd[2]))));
a[17] = (cos(xd[1]));
a[18] = (cos((xd[1]+xd[2])));
a[19] = (cos((xd[1]+xd[2])));

/* Compute outputs: */
out[0] = (((((real_t)(1.5500000000000000e-01)-((real_t)(3.2000000000000001e-01)*a[0]))-((real_t)(5.9000000000000008e-01)*a[1]))-(real_t)(6.2000000000000000e-02))+xd[0]);
out[1] = ((((real_t)(3.2000000000000001e-01)*a[2])+((real_t)(5.9000000000000008e-01)*a[3]))+(real_t)(9.8999999999999999e-01));
out[2] = (((((real_t)(0.0000000000000000e+00)-(((real_t)(3.2000000000000001e-01)*a[4])+((real_t)(5.9000000000000008e-01)*a[5])))*xd[4])-(((real_t)(5.9000000000000008e-01)*a[6])*xd[5]))+xd[3]);
out[3] = ((((real_t)(0.0000000000000000e+00)-xd[4])*(((real_t)(3.2000000000000001e-01)*a[7])+((real_t)(5.9000000000000008e-01)*a[8])))-(((real_t)(5.9000000000000008e-01)*a[9])*xd[5]));
out[4] = xd[3];
out[5] = xd[4];
out[6] = xd[5];
out[7] = (real_t)(1.0000000000000000e+00);
out[8] = (((real_t)(0.0000000000000000e+00)-((real_t)(3.2000000000000001e-01)*a[10]))-((real_t)(5.9000000000000008e-01)*a[11]));
out[9] = ((real_t)(0.0000000000000000e+00)-((real_t)(5.9000000000000008e-01)*a[11]));
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = (((real_t)(3.2000000000000001e-01)*a[12])+((real_t)(5.9000000000000008e-01)*a[13]));
out[15] = ((real_t)(5.9000000000000008e-01)*a[13]);
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (real_t)(0.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = ((((real_t)(0.0000000000000000e+00)-(((real_t)(3.2000000000000001e-01)*a[14])+((real_t)(5.9000000000000008e-01)*a[15])))*xd[4])-(((real_t)(5.9000000000000008e-01)*a[16])*xd[5]));
out[21] = ((((real_t)(0.0000000000000000e+00)-((real_t)(5.9000000000000008e-01)*a[15]))*xd[4])-(((real_t)(5.9000000000000008e-01)*a[16])*xd[5]));
out[22] = (real_t)(1.0000000000000000e+00);
out[23] = ((real_t)(0.0000000000000000e+00)-(((real_t)(3.2000000000000001e-01)*a[4])+((real_t)(5.9000000000000008e-01)*a[5])));
out[24] = ((real_t)(0.0000000000000000e+00)-((real_t)(5.9000000000000008e-01)*a[6]));
out[25] = (real_t)(0.0000000000000000e+00);
out[26] = ((((real_t)(0.0000000000000000e+00)-xd[4])*(((real_t)(3.2000000000000001e-01)*a[17])+((real_t)(5.9000000000000008e-01)*a[18])))-(((real_t)(5.9000000000000008e-01)*a[19])*xd[5]));
out[27] = ((((real_t)(0.0000000000000000e+00)-xd[4])*((real_t)(5.9000000000000008e-01)*a[18]))-(((real_t)(5.9000000000000008e-01)*a[19])*xd[5]));
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*(((real_t)(3.2000000000000001e-01)*a[7])+((real_t)(5.9000000000000008e-01)*a[8])));
out[30] = ((real_t)(0.0000000000000000e+00)-((real_t)(5.9000000000000008e-01)*a[9]));
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(1.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = (real_t)(0.0000000000000000e+00);
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = (real_t)(0.0000000000000000e+00);
out[40] = (real_t)(0.0000000000000000e+00);
out[41] = (real_t)(1.0000000000000000e+00);
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = (real_t)(0.0000000000000000e+00);
out[44] = (real_t)(0.0000000000000000e+00);
out[45] = (real_t)(0.0000000000000000e+00);
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (real_t)(1.0000000000000000e+00);
}

void acado_evaluatePathConstraints(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* od = in + 9;
/* Vector of auxiliary variables; number of elements: 88. */
real_t* a = acadoWorkspace.conAuxVar;

/* Compute intermediate quantities: */
a[0] = (pow((((real_t)(9.2999999999999999e-02)+xd[0])-od[0]),2));
a[1] = (pow(((real_t)(4.9500000000000000e-01)-od[1]),2));
a[2] = (sqrt((a[0]+a[1])));
a[3] = (sin(xd[1]));
a[4] = (pow((((((real_t)(1.5500000000000000e-01)-((real_t)(1.6000000000000000e-01)*a[3]))-(real_t)(6.2000000000000000e-02))+xd[0])-od[0]),2));
a[5] = (cos(xd[1]));
a[6] = (pow(((((real_t)(1.6000000000000000e-01)*a[5])+(real_t)(9.8999999999999999e-01))-od[1]),2));
a[7] = (sqrt((a[4]+a[6])));
a[8] = (sin((xd[1]+xd[2])));
a[9] = (sin(xd[1]));
a[10] = (pow(((((((real_t)(1.5500000000000000e-01)-((real_t)(2.9500000000000004e-01)*a[8]))-((real_t)(3.2000000000000001e-01)*a[9]))-(real_t)(6.2000000000000000e-02))+xd[0])-od[0]),2));
a[11] = (cos((xd[1]+xd[2])));
a[12] = (cos(xd[1]));
a[13] = (pow((((((real_t)(2.9500000000000004e-01)*a[11])+((real_t)(3.2000000000000001e-01)*a[12]))+(real_t)(9.8999999999999999e-01))-od[1]),2));
a[14] = (sqrt((a[10]+a[13])));
a[15] = ((real_t)(2.0000000000000000e+00)*(((real_t)(9.2999999999999999e-02)+xd[0])-od[0]));
a[16] = (1.0/sqrt((a[0]+a[1])));
a[17] = (a[16]*(real_t)(5.0000000000000000e-01));
a[18] = (a[15]*a[17]);
a[19] = (real_t)(0.0000000000000000e+00);
a[20] = (real_t)(0.0000000000000000e+00);
a[21] = (real_t)(0.0000000000000000e+00);
a[22] = (real_t)(0.0000000000000000e+00);
a[23] = (real_t)(0.0000000000000000e+00);
a[24] = ((real_t)(2.0000000000000000e+00)*(((((real_t)(1.5500000000000000e-01)-((real_t)(1.6000000000000000e-01)*a[3]))-(real_t)(6.2000000000000000e-02))+xd[0])-od[0]));
a[25] = (1.0/sqrt((a[4]+a[6])));
a[26] = (a[25]*(real_t)(5.0000000000000000e-01));
a[27] = (a[24]*a[26]);
a[28] = (cos(xd[1]));
a[29] = (real_t)(1.6000000000000000e-01);
a[30] = (a[28]*a[29]);
a[31] = (real_t)(-1.0000000000000000e+00);
a[32] = (a[30]*a[31]);
a[33] = (a[32]*a[24]);
a[34] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[1])));
a[35] = (real_t)(1.6000000000000000e-01);
a[36] = (a[34]*a[35]);
a[37] = ((real_t)(2.0000000000000000e+00)*((((real_t)(1.6000000000000000e-01)*a[5])+(real_t)(9.8999999999999999e-01))-od[1]));
a[38] = (a[36]*a[37]);
a[39] = ((a[33]+a[38])*a[26]);
a[40] = (real_t)(0.0000000000000000e+00);
a[41] = (real_t)(0.0000000000000000e+00);
a[42] = (real_t)(0.0000000000000000e+00);
a[43] = (real_t)(0.0000000000000000e+00);
a[44] = ((real_t)(2.0000000000000000e+00)*((((((real_t)(1.5500000000000000e-01)-((real_t)(2.9500000000000004e-01)*a[8]))-((real_t)(3.2000000000000001e-01)*a[9]))-(real_t)(6.2000000000000000e-02))+xd[0])-od[0]));
a[45] = (1.0/sqrt((a[10]+a[13])));
a[46] = (a[45]*(real_t)(5.0000000000000000e-01));
a[47] = (a[44]*a[46]);
a[48] = (cos((xd[1]+xd[2])));
a[49] = (real_t)(2.9500000000000004e-01);
a[50] = (a[48]*a[49]);
a[51] = (real_t)(-1.0000000000000000e+00);
a[52] = (a[50]*a[51]);
a[53] = (cos(xd[1]));
a[54] = (real_t)(3.2000000000000001e-01);
a[55] = (a[53]*a[54]);
a[56] = (real_t)(-1.0000000000000000e+00);
a[57] = (a[55]*a[56]);
a[58] = (a[52]+a[57]);
a[59] = (a[58]*a[44]);
a[60] = ((real_t)(-1.0000000000000000e+00)*(sin((xd[1]+xd[2]))));
a[61] = (real_t)(2.9500000000000004e-01);
a[62] = (a[60]*a[61]);
a[63] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[1])));
a[64] = (real_t)(3.2000000000000001e-01);
a[65] = (a[63]*a[64]);
a[66] = (a[62]+a[65]);
a[67] = ((real_t)(2.0000000000000000e+00)*(((((real_t)(2.9500000000000004e-01)*a[11])+((real_t)(3.2000000000000001e-01)*a[12]))+(real_t)(9.8999999999999999e-01))-od[1]));
a[68] = (a[66]*a[67]);
a[69] = ((a[59]+a[68])*a[46]);
a[70] = (a[48]*a[49]);
a[71] = (a[70]*a[51]);
a[72] = (a[71]*a[44]);
a[73] = (a[60]*a[61]);
a[74] = (a[73]*a[67]);
a[75] = ((a[72]+a[74])*a[46]);
a[76] = (real_t)(0.0000000000000000e+00);
a[77] = (real_t)(0.0000000000000000e+00);
a[78] = (real_t)(0.0000000000000000e+00);
a[79] = (real_t)(0.0000000000000000e+00);
a[80] = (real_t)(0.0000000000000000e+00);
a[81] = (real_t)(0.0000000000000000e+00);
a[82] = (real_t)(0.0000000000000000e+00);
a[83] = (real_t)(0.0000000000000000e+00);
a[84] = (real_t)(0.0000000000000000e+00);
a[85] = (real_t)(0.0000000000000000e+00);
a[86] = (real_t)(0.0000000000000000e+00);
a[87] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = (((real_t)(-4.9500000000000000e-01)-od[2])+a[2]);
out[1] = (((real_t)(-1.6000000000000000e-01)-od[2])+a[7]);
out[2] = (((real_t)(-2.9500000000000004e-01)-od[2])+a[14]);
out[3] = a[18];
out[4] = a[19];
out[5] = a[20];
out[6] = a[21];
out[7] = a[22];
out[8] = a[23];
out[9] = a[27];
out[10] = a[39];
out[11] = a[40];
out[12] = a[41];
out[13] = a[42];
out[14] = a[43];
out[15] = a[47];
out[16] = a[69];
out[17] = a[75];
out[18] = a[76];
out[19] = a[77];
out[20] = a[78];
out[21] = a[79];
out[22] = a[80];
out[23] = a[81];
out[24] = a[82];
out[25] = a[83];
out[26] = a[84];
out[27] = a[85];
out[28] = a[86];
out[29] = a[87];
}

void acado_setObjQ1Q2( real_t* const tmpFx, real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = + tmpFx[0]*tmpObjS[0] + tmpFx[6]*tmpObjS[10] + tmpFx[12]*tmpObjS[20] + tmpFx[18]*tmpObjS[30] + tmpFx[24]*tmpObjS[40] + tmpFx[30]*tmpObjS[50] + tmpFx[36]*tmpObjS[60] + tmpFx[42]*tmpObjS[70] + tmpFx[48]*tmpObjS[80] + tmpFx[54]*tmpObjS[90];
tmpQ2[1] = + tmpFx[0]*tmpObjS[1] + tmpFx[6]*tmpObjS[11] + tmpFx[12]*tmpObjS[21] + tmpFx[18]*tmpObjS[31] + tmpFx[24]*tmpObjS[41] + tmpFx[30]*tmpObjS[51] + tmpFx[36]*tmpObjS[61] + tmpFx[42]*tmpObjS[71] + tmpFx[48]*tmpObjS[81] + tmpFx[54]*tmpObjS[91];
tmpQ2[2] = + tmpFx[0]*tmpObjS[2] + tmpFx[6]*tmpObjS[12] + tmpFx[12]*tmpObjS[22] + tmpFx[18]*tmpObjS[32] + tmpFx[24]*tmpObjS[42] + tmpFx[30]*tmpObjS[52] + tmpFx[36]*tmpObjS[62] + tmpFx[42]*tmpObjS[72] + tmpFx[48]*tmpObjS[82] + tmpFx[54]*tmpObjS[92];
tmpQ2[3] = + tmpFx[0]*tmpObjS[3] + tmpFx[6]*tmpObjS[13] + tmpFx[12]*tmpObjS[23] + tmpFx[18]*tmpObjS[33] + tmpFx[24]*tmpObjS[43] + tmpFx[30]*tmpObjS[53] + tmpFx[36]*tmpObjS[63] + tmpFx[42]*tmpObjS[73] + tmpFx[48]*tmpObjS[83] + tmpFx[54]*tmpObjS[93];
tmpQ2[4] = + tmpFx[0]*tmpObjS[4] + tmpFx[6]*tmpObjS[14] + tmpFx[12]*tmpObjS[24] + tmpFx[18]*tmpObjS[34] + tmpFx[24]*tmpObjS[44] + tmpFx[30]*tmpObjS[54] + tmpFx[36]*tmpObjS[64] + tmpFx[42]*tmpObjS[74] + tmpFx[48]*tmpObjS[84] + tmpFx[54]*tmpObjS[94];
tmpQ2[5] = + tmpFx[0]*tmpObjS[5] + tmpFx[6]*tmpObjS[15] + tmpFx[12]*tmpObjS[25] + tmpFx[18]*tmpObjS[35] + tmpFx[24]*tmpObjS[45] + tmpFx[30]*tmpObjS[55] + tmpFx[36]*tmpObjS[65] + tmpFx[42]*tmpObjS[75] + tmpFx[48]*tmpObjS[85] + tmpFx[54]*tmpObjS[95];
tmpQ2[6] = + tmpFx[0]*tmpObjS[6] + tmpFx[6]*tmpObjS[16] + tmpFx[12]*tmpObjS[26] + tmpFx[18]*tmpObjS[36] + tmpFx[24]*tmpObjS[46] + tmpFx[30]*tmpObjS[56] + tmpFx[36]*tmpObjS[66] + tmpFx[42]*tmpObjS[76] + tmpFx[48]*tmpObjS[86] + tmpFx[54]*tmpObjS[96];
tmpQ2[7] = + tmpFx[0]*tmpObjS[7] + tmpFx[6]*tmpObjS[17] + tmpFx[12]*tmpObjS[27] + tmpFx[18]*tmpObjS[37] + tmpFx[24]*tmpObjS[47] + tmpFx[30]*tmpObjS[57] + tmpFx[36]*tmpObjS[67] + tmpFx[42]*tmpObjS[77] + tmpFx[48]*tmpObjS[87] + tmpFx[54]*tmpObjS[97];
tmpQ2[8] = + tmpFx[0]*tmpObjS[8] + tmpFx[6]*tmpObjS[18] + tmpFx[12]*tmpObjS[28] + tmpFx[18]*tmpObjS[38] + tmpFx[24]*tmpObjS[48] + tmpFx[30]*tmpObjS[58] + tmpFx[36]*tmpObjS[68] + tmpFx[42]*tmpObjS[78] + tmpFx[48]*tmpObjS[88] + tmpFx[54]*tmpObjS[98];
tmpQ2[9] = + tmpFx[0]*tmpObjS[9] + tmpFx[6]*tmpObjS[19] + tmpFx[12]*tmpObjS[29] + tmpFx[18]*tmpObjS[39] + tmpFx[24]*tmpObjS[49] + tmpFx[30]*tmpObjS[59] + tmpFx[36]*tmpObjS[69] + tmpFx[42]*tmpObjS[79] + tmpFx[48]*tmpObjS[89] + tmpFx[54]*tmpObjS[99];
tmpQ2[10] = + tmpFx[1]*tmpObjS[0] + tmpFx[7]*tmpObjS[10] + tmpFx[13]*tmpObjS[20] + tmpFx[19]*tmpObjS[30] + tmpFx[25]*tmpObjS[40] + tmpFx[31]*tmpObjS[50] + tmpFx[37]*tmpObjS[60] + tmpFx[43]*tmpObjS[70] + tmpFx[49]*tmpObjS[80] + tmpFx[55]*tmpObjS[90];
tmpQ2[11] = + tmpFx[1]*tmpObjS[1] + tmpFx[7]*tmpObjS[11] + tmpFx[13]*tmpObjS[21] + tmpFx[19]*tmpObjS[31] + tmpFx[25]*tmpObjS[41] + tmpFx[31]*tmpObjS[51] + tmpFx[37]*tmpObjS[61] + tmpFx[43]*tmpObjS[71] + tmpFx[49]*tmpObjS[81] + tmpFx[55]*tmpObjS[91];
tmpQ2[12] = + tmpFx[1]*tmpObjS[2] + tmpFx[7]*tmpObjS[12] + tmpFx[13]*tmpObjS[22] + tmpFx[19]*tmpObjS[32] + tmpFx[25]*tmpObjS[42] + tmpFx[31]*tmpObjS[52] + tmpFx[37]*tmpObjS[62] + tmpFx[43]*tmpObjS[72] + tmpFx[49]*tmpObjS[82] + tmpFx[55]*tmpObjS[92];
tmpQ2[13] = + tmpFx[1]*tmpObjS[3] + tmpFx[7]*tmpObjS[13] + tmpFx[13]*tmpObjS[23] + tmpFx[19]*tmpObjS[33] + tmpFx[25]*tmpObjS[43] + tmpFx[31]*tmpObjS[53] + tmpFx[37]*tmpObjS[63] + tmpFx[43]*tmpObjS[73] + tmpFx[49]*tmpObjS[83] + tmpFx[55]*tmpObjS[93];
tmpQ2[14] = + tmpFx[1]*tmpObjS[4] + tmpFx[7]*tmpObjS[14] + tmpFx[13]*tmpObjS[24] + tmpFx[19]*tmpObjS[34] + tmpFx[25]*tmpObjS[44] + tmpFx[31]*tmpObjS[54] + tmpFx[37]*tmpObjS[64] + tmpFx[43]*tmpObjS[74] + tmpFx[49]*tmpObjS[84] + tmpFx[55]*tmpObjS[94];
tmpQ2[15] = + tmpFx[1]*tmpObjS[5] + tmpFx[7]*tmpObjS[15] + tmpFx[13]*tmpObjS[25] + tmpFx[19]*tmpObjS[35] + tmpFx[25]*tmpObjS[45] + tmpFx[31]*tmpObjS[55] + tmpFx[37]*tmpObjS[65] + tmpFx[43]*tmpObjS[75] + tmpFx[49]*tmpObjS[85] + tmpFx[55]*tmpObjS[95];
tmpQ2[16] = + tmpFx[1]*tmpObjS[6] + tmpFx[7]*tmpObjS[16] + tmpFx[13]*tmpObjS[26] + tmpFx[19]*tmpObjS[36] + tmpFx[25]*tmpObjS[46] + tmpFx[31]*tmpObjS[56] + tmpFx[37]*tmpObjS[66] + tmpFx[43]*tmpObjS[76] + tmpFx[49]*tmpObjS[86] + tmpFx[55]*tmpObjS[96];
tmpQ2[17] = + tmpFx[1]*tmpObjS[7] + tmpFx[7]*tmpObjS[17] + tmpFx[13]*tmpObjS[27] + tmpFx[19]*tmpObjS[37] + tmpFx[25]*tmpObjS[47] + tmpFx[31]*tmpObjS[57] + tmpFx[37]*tmpObjS[67] + tmpFx[43]*tmpObjS[77] + tmpFx[49]*tmpObjS[87] + tmpFx[55]*tmpObjS[97];
tmpQ2[18] = + tmpFx[1]*tmpObjS[8] + tmpFx[7]*tmpObjS[18] + tmpFx[13]*tmpObjS[28] + tmpFx[19]*tmpObjS[38] + tmpFx[25]*tmpObjS[48] + tmpFx[31]*tmpObjS[58] + tmpFx[37]*tmpObjS[68] + tmpFx[43]*tmpObjS[78] + tmpFx[49]*tmpObjS[88] + tmpFx[55]*tmpObjS[98];
tmpQ2[19] = + tmpFx[1]*tmpObjS[9] + tmpFx[7]*tmpObjS[19] + tmpFx[13]*tmpObjS[29] + tmpFx[19]*tmpObjS[39] + tmpFx[25]*tmpObjS[49] + tmpFx[31]*tmpObjS[59] + tmpFx[37]*tmpObjS[69] + tmpFx[43]*tmpObjS[79] + tmpFx[49]*tmpObjS[89] + tmpFx[55]*tmpObjS[99];
tmpQ2[20] = + tmpFx[2]*tmpObjS[0] + tmpFx[8]*tmpObjS[10] + tmpFx[14]*tmpObjS[20] + tmpFx[20]*tmpObjS[30] + tmpFx[26]*tmpObjS[40] + tmpFx[32]*tmpObjS[50] + tmpFx[38]*tmpObjS[60] + tmpFx[44]*tmpObjS[70] + tmpFx[50]*tmpObjS[80] + tmpFx[56]*tmpObjS[90];
tmpQ2[21] = + tmpFx[2]*tmpObjS[1] + tmpFx[8]*tmpObjS[11] + tmpFx[14]*tmpObjS[21] + tmpFx[20]*tmpObjS[31] + tmpFx[26]*tmpObjS[41] + tmpFx[32]*tmpObjS[51] + tmpFx[38]*tmpObjS[61] + tmpFx[44]*tmpObjS[71] + tmpFx[50]*tmpObjS[81] + tmpFx[56]*tmpObjS[91];
tmpQ2[22] = + tmpFx[2]*tmpObjS[2] + tmpFx[8]*tmpObjS[12] + tmpFx[14]*tmpObjS[22] + tmpFx[20]*tmpObjS[32] + tmpFx[26]*tmpObjS[42] + tmpFx[32]*tmpObjS[52] + tmpFx[38]*tmpObjS[62] + tmpFx[44]*tmpObjS[72] + tmpFx[50]*tmpObjS[82] + tmpFx[56]*tmpObjS[92];
tmpQ2[23] = + tmpFx[2]*tmpObjS[3] + tmpFx[8]*tmpObjS[13] + tmpFx[14]*tmpObjS[23] + tmpFx[20]*tmpObjS[33] + tmpFx[26]*tmpObjS[43] + tmpFx[32]*tmpObjS[53] + tmpFx[38]*tmpObjS[63] + tmpFx[44]*tmpObjS[73] + tmpFx[50]*tmpObjS[83] + tmpFx[56]*tmpObjS[93];
tmpQ2[24] = + tmpFx[2]*tmpObjS[4] + tmpFx[8]*tmpObjS[14] + tmpFx[14]*tmpObjS[24] + tmpFx[20]*tmpObjS[34] + tmpFx[26]*tmpObjS[44] + tmpFx[32]*tmpObjS[54] + tmpFx[38]*tmpObjS[64] + tmpFx[44]*tmpObjS[74] + tmpFx[50]*tmpObjS[84] + tmpFx[56]*tmpObjS[94];
tmpQ2[25] = + tmpFx[2]*tmpObjS[5] + tmpFx[8]*tmpObjS[15] + tmpFx[14]*tmpObjS[25] + tmpFx[20]*tmpObjS[35] + tmpFx[26]*tmpObjS[45] + tmpFx[32]*tmpObjS[55] + tmpFx[38]*tmpObjS[65] + tmpFx[44]*tmpObjS[75] + tmpFx[50]*tmpObjS[85] + tmpFx[56]*tmpObjS[95];
tmpQ2[26] = + tmpFx[2]*tmpObjS[6] + tmpFx[8]*tmpObjS[16] + tmpFx[14]*tmpObjS[26] + tmpFx[20]*tmpObjS[36] + tmpFx[26]*tmpObjS[46] + tmpFx[32]*tmpObjS[56] + tmpFx[38]*tmpObjS[66] + tmpFx[44]*tmpObjS[76] + tmpFx[50]*tmpObjS[86] + tmpFx[56]*tmpObjS[96];
tmpQ2[27] = + tmpFx[2]*tmpObjS[7] + tmpFx[8]*tmpObjS[17] + tmpFx[14]*tmpObjS[27] + tmpFx[20]*tmpObjS[37] + tmpFx[26]*tmpObjS[47] + tmpFx[32]*tmpObjS[57] + tmpFx[38]*tmpObjS[67] + tmpFx[44]*tmpObjS[77] + tmpFx[50]*tmpObjS[87] + tmpFx[56]*tmpObjS[97];
tmpQ2[28] = + tmpFx[2]*tmpObjS[8] + tmpFx[8]*tmpObjS[18] + tmpFx[14]*tmpObjS[28] + tmpFx[20]*tmpObjS[38] + tmpFx[26]*tmpObjS[48] + tmpFx[32]*tmpObjS[58] + tmpFx[38]*tmpObjS[68] + tmpFx[44]*tmpObjS[78] + tmpFx[50]*tmpObjS[88] + tmpFx[56]*tmpObjS[98];
tmpQ2[29] = + tmpFx[2]*tmpObjS[9] + tmpFx[8]*tmpObjS[19] + tmpFx[14]*tmpObjS[29] + tmpFx[20]*tmpObjS[39] + tmpFx[26]*tmpObjS[49] + tmpFx[32]*tmpObjS[59] + tmpFx[38]*tmpObjS[69] + tmpFx[44]*tmpObjS[79] + tmpFx[50]*tmpObjS[89] + tmpFx[56]*tmpObjS[99];
tmpQ2[30] = + tmpFx[3]*tmpObjS[0] + tmpFx[9]*tmpObjS[10] + tmpFx[15]*tmpObjS[20] + tmpFx[21]*tmpObjS[30] + tmpFx[27]*tmpObjS[40] + tmpFx[33]*tmpObjS[50] + tmpFx[39]*tmpObjS[60] + tmpFx[45]*tmpObjS[70] + tmpFx[51]*tmpObjS[80] + tmpFx[57]*tmpObjS[90];
tmpQ2[31] = + tmpFx[3]*tmpObjS[1] + tmpFx[9]*tmpObjS[11] + tmpFx[15]*tmpObjS[21] + tmpFx[21]*tmpObjS[31] + tmpFx[27]*tmpObjS[41] + tmpFx[33]*tmpObjS[51] + tmpFx[39]*tmpObjS[61] + tmpFx[45]*tmpObjS[71] + tmpFx[51]*tmpObjS[81] + tmpFx[57]*tmpObjS[91];
tmpQ2[32] = + tmpFx[3]*tmpObjS[2] + tmpFx[9]*tmpObjS[12] + tmpFx[15]*tmpObjS[22] + tmpFx[21]*tmpObjS[32] + tmpFx[27]*tmpObjS[42] + tmpFx[33]*tmpObjS[52] + tmpFx[39]*tmpObjS[62] + tmpFx[45]*tmpObjS[72] + tmpFx[51]*tmpObjS[82] + tmpFx[57]*tmpObjS[92];
tmpQ2[33] = + tmpFx[3]*tmpObjS[3] + tmpFx[9]*tmpObjS[13] + tmpFx[15]*tmpObjS[23] + tmpFx[21]*tmpObjS[33] + tmpFx[27]*tmpObjS[43] + tmpFx[33]*tmpObjS[53] + tmpFx[39]*tmpObjS[63] + tmpFx[45]*tmpObjS[73] + tmpFx[51]*tmpObjS[83] + tmpFx[57]*tmpObjS[93];
tmpQ2[34] = + tmpFx[3]*tmpObjS[4] + tmpFx[9]*tmpObjS[14] + tmpFx[15]*tmpObjS[24] + tmpFx[21]*tmpObjS[34] + tmpFx[27]*tmpObjS[44] + tmpFx[33]*tmpObjS[54] + tmpFx[39]*tmpObjS[64] + tmpFx[45]*tmpObjS[74] + tmpFx[51]*tmpObjS[84] + tmpFx[57]*tmpObjS[94];
tmpQ2[35] = + tmpFx[3]*tmpObjS[5] + tmpFx[9]*tmpObjS[15] + tmpFx[15]*tmpObjS[25] + tmpFx[21]*tmpObjS[35] + tmpFx[27]*tmpObjS[45] + tmpFx[33]*tmpObjS[55] + tmpFx[39]*tmpObjS[65] + tmpFx[45]*tmpObjS[75] + tmpFx[51]*tmpObjS[85] + tmpFx[57]*tmpObjS[95];
tmpQ2[36] = + tmpFx[3]*tmpObjS[6] + tmpFx[9]*tmpObjS[16] + tmpFx[15]*tmpObjS[26] + tmpFx[21]*tmpObjS[36] + tmpFx[27]*tmpObjS[46] + tmpFx[33]*tmpObjS[56] + tmpFx[39]*tmpObjS[66] + tmpFx[45]*tmpObjS[76] + tmpFx[51]*tmpObjS[86] + tmpFx[57]*tmpObjS[96];
tmpQ2[37] = + tmpFx[3]*tmpObjS[7] + tmpFx[9]*tmpObjS[17] + tmpFx[15]*tmpObjS[27] + tmpFx[21]*tmpObjS[37] + tmpFx[27]*tmpObjS[47] + tmpFx[33]*tmpObjS[57] + tmpFx[39]*tmpObjS[67] + tmpFx[45]*tmpObjS[77] + tmpFx[51]*tmpObjS[87] + tmpFx[57]*tmpObjS[97];
tmpQ2[38] = + tmpFx[3]*tmpObjS[8] + tmpFx[9]*tmpObjS[18] + tmpFx[15]*tmpObjS[28] + tmpFx[21]*tmpObjS[38] + tmpFx[27]*tmpObjS[48] + tmpFx[33]*tmpObjS[58] + tmpFx[39]*tmpObjS[68] + tmpFx[45]*tmpObjS[78] + tmpFx[51]*tmpObjS[88] + tmpFx[57]*tmpObjS[98];
tmpQ2[39] = + tmpFx[3]*tmpObjS[9] + tmpFx[9]*tmpObjS[19] + tmpFx[15]*tmpObjS[29] + tmpFx[21]*tmpObjS[39] + tmpFx[27]*tmpObjS[49] + tmpFx[33]*tmpObjS[59] + tmpFx[39]*tmpObjS[69] + tmpFx[45]*tmpObjS[79] + tmpFx[51]*tmpObjS[89] + tmpFx[57]*tmpObjS[99];
tmpQ2[40] = + tmpFx[4]*tmpObjS[0] + tmpFx[10]*tmpObjS[10] + tmpFx[16]*tmpObjS[20] + tmpFx[22]*tmpObjS[30] + tmpFx[28]*tmpObjS[40] + tmpFx[34]*tmpObjS[50] + tmpFx[40]*tmpObjS[60] + tmpFx[46]*tmpObjS[70] + tmpFx[52]*tmpObjS[80] + tmpFx[58]*tmpObjS[90];
tmpQ2[41] = + tmpFx[4]*tmpObjS[1] + tmpFx[10]*tmpObjS[11] + tmpFx[16]*tmpObjS[21] + tmpFx[22]*tmpObjS[31] + tmpFx[28]*tmpObjS[41] + tmpFx[34]*tmpObjS[51] + tmpFx[40]*tmpObjS[61] + tmpFx[46]*tmpObjS[71] + tmpFx[52]*tmpObjS[81] + tmpFx[58]*tmpObjS[91];
tmpQ2[42] = + tmpFx[4]*tmpObjS[2] + tmpFx[10]*tmpObjS[12] + tmpFx[16]*tmpObjS[22] + tmpFx[22]*tmpObjS[32] + tmpFx[28]*tmpObjS[42] + tmpFx[34]*tmpObjS[52] + tmpFx[40]*tmpObjS[62] + tmpFx[46]*tmpObjS[72] + tmpFx[52]*tmpObjS[82] + tmpFx[58]*tmpObjS[92];
tmpQ2[43] = + tmpFx[4]*tmpObjS[3] + tmpFx[10]*tmpObjS[13] + tmpFx[16]*tmpObjS[23] + tmpFx[22]*tmpObjS[33] + tmpFx[28]*tmpObjS[43] + tmpFx[34]*tmpObjS[53] + tmpFx[40]*tmpObjS[63] + tmpFx[46]*tmpObjS[73] + tmpFx[52]*tmpObjS[83] + tmpFx[58]*tmpObjS[93];
tmpQ2[44] = + tmpFx[4]*tmpObjS[4] + tmpFx[10]*tmpObjS[14] + tmpFx[16]*tmpObjS[24] + tmpFx[22]*tmpObjS[34] + tmpFx[28]*tmpObjS[44] + tmpFx[34]*tmpObjS[54] + tmpFx[40]*tmpObjS[64] + tmpFx[46]*tmpObjS[74] + tmpFx[52]*tmpObjS[84] + tmpFx[58]*tmpObjS[94];
tmpQ2[45] = + tmpFx[4]*tmpObjS[5] + tmpFx[10]*tmpObjS[15] + tmpFx[16]*tmpObjS[25] + tmpFx[22]*tmpObjS[35] + tmpFx[28]*tmpObjS[45] + tmpFx[34]*tmpObjS[55] + tmpFx[40]*tmpObjS[65] + tmpFx[46]*tmpObjS[75] + tmpFx[52]*tmpObjS[85] + tmpFx[58]*tmpObjS[95];
tmpQ2[46] = + tmpFx[4]*tmpObjS[6] + tmpFx[10]*tmpObjS[16] + tmpFx[16]*tmpObjS[26] + tmpFx[22]*tmpObjS[36] + tmpFx[28]*tmpObjS[46] + tmpFx[34]*tmpObjS[56] + tmpFx[40]*tmpObjS[66] + tmpFx[46]*tmpObjS[76] + tmpFx[52]*tmpObjS[86] + tmpFx[58]*tmpObjS[96];
tmpQ2[47] = + tmpFx[4]*tmpObjS[7] + tmpFx[10]*tmpObjS[17] + tmpFx[16]*tmpObjS[27] + tmpFx[22]*tmpObjS[37] + tmpFx[28]*tmpObjS[47] + tmpFx[34]*tmpObjS[57] + tmpFx[40]*tmpObjS[67] + tmpFx[46]*tmpObjS[77] + tmpFx[52]*tmpObjS[87] + tmpFx[58]*tmpObjS[97];
tmpQ2[48] = + tmpFx[4]*tmpObjS[8] + tmpFx[10]*tmpObjS[18] + tmpFx[16]*tmpObjS[28] + tmpFx[22]*tmpObjS[38] + tmpFx[28]*tmpObjS[48] + tmpFx[34]*tmpObjS[58] + tmpFx[40]*tmpObjS[68] + tmpFx[46]*tmpObjS[78] + tmpFx[52]*tmpObjS[88] + tmpFx[58]*tmpObjS[98];
tmpQ2[49] = + tmpFx[4]*tmpObjS[9] + tmpFx[10]*tmpObjS[19] + tmpFx[16]*tmpObjS[29] + tmpFx[22]*tmpObjS[39] + tmpFx[28]*tmpObjS[49] + tmpFx[34]*tmpObjS[59] + tmpFx[40]*tmpObjS[69] + tmpFx[46]*tmpObjS[79] + tmpFx[52]*tmpObjS[89] + tmpFx[58]*tmpObjS[99];
tmpQ2[50] = + tmpFx[5]*tmpObjS[0] + tmpFx[11]*tmpObjS[10] + tmpFx[17]*tmpObjS[20] + tmpFx[23]*tmpObjS[30] + tmpFx[29]*tmpObjS[40] + tmpFx[35]*tmpObjS[50] + tmpFx[41]*tmpObjS[60] + tmpFx[47]*tmpObjS[70] + tmpFx[53]*tmpObjS[80] + tmpFx[59]*tmpObjS[90];
tmpQ2[51] = + tmpFx[5]*tmpObjS[1] + tmpFx[11]*tmpObjS[11] + tmpFx[17]*tmpObjS[21] + tmpFx[23]*tmpObjS[31] + tmpFx[29]*tmpObjS[41] + tmpFx[35]*tmpObjS[51] + tmpFx[41]*tmpObjS[61] + tmpFx[47]*tmpObjS[71] + tmpFx[53]*tmpObjS[81] + tmpFx[59]*tmpObjS[91];
tmpQ2[52] = + tmpFx[5]*tmpObjS[2] + tmpFx[11]*tmpObjS[12] + tmpFx[17]*tmpObjS[22] + tmpFx[23]*tmpObjS[32] + tmpFx[29]*tmpObjS[42] + tmpFx[35]*tmpObjS[52] + tmpFx[41]*tmpObjS[62] + tmpFx[47]*tmpObjS[72] + tmpFx[53]*tmpObjS[82] + tmpFx[59]*tmpObjS[92];
tmpQ2[53] = + tmpFx[5]*tmpObjS[3] + tmpFx[11]*tmpObjS[13] + tmpFx[17]*tmpObjS[23] + tmpFx[23]*tmpObjS[33] + tmpFx[29]*tmpObjS[43] + tmpFx[35]*tmpObjS[53] + tmpFx[41]*tmpObjS[63] + tmpFx[47]*tmpObjS[73] + tmpFx[53]*tmpObjS[83] + tmpFx[59]*tmpObjS[93];
tmpQ2[54] = + tmpFx[5]*tmpObjS[4] + tmpFx[11]*tmpObjS[14] + tmpFx[17]*tmpObjS[24] + tmpFx[23]*tmpObjS[34] + tmpFx[29]*tmpObjS[44] + tmpFx[35]*tmpObjS[54] + tmpFx[41]*tmpObjS[64] + tmpFx[47]*tmpObjS[74] + tmpFx[53]*tmpObjS[84] + tmpFx[59]*tmpObjS[94];
tmpQ2[55] = + tmpFx[5]*tmpObjS[5] + tmpFx[11]*tmpObjS[15] + tmpFx[17]*tmpObjS[25] + tmpFx[23]*tmpObjS[35] + tmpFx[29]*tmpObjS[45] + tmpFx[35]*tmpObjS[55] + tmpFx[41]*tmpObjS[65] + tmpFx[47]*tmpObjS[75] + tmpFx[53]*tmpObjS[85] + tmpFx[59]*tmpObjS[95];
tmpQ2[56] = + tmpFx[5]*tmpObjS[6] + tmpFx[11]*tmpObjS[16] + tmpFx[17]*tmpObjS[26] + tmpFx[23]*tmpObjS[36] + tmpFx[29]*tmpObjS[46] + tmpFx[35]*tmpObjS[56] + tmpFx[41]*tmpObjS[66] + tmpFx[47]*tmpObjS[76] + tmpFx[53]*tmpObjS[86] + tmpFx[59]*tmpObjS[96];
tmpQ2[57] = + tmpFx[5]*tmpObjS[7] + tmpFx[11]*tmpObjS[17] + tmpFx[17]*tmpObjS[27] + tmpFx[23]*tmpObjS[37] + tmpFx[29]*tmpObjS[47] + tmpFx[35]*tmpObjS[57] + tmpFx[41]*tmpObjS[67] + tmpFx[47]*tmpObjS[77] + tmpFx[53]*tmpObjS[87] + tmpFx[59]*tmpObjS[97];
tmpQ2[58] = + tmpFx[5]*tmpObjS[8] + tmpFx[11]*tmpObjS[18] + tmpFx[17]*tmpObjS[28] + tmpFx[23]*tmpObjS[38] + tmpFx[29]*tmpObjS[48] + tmpFx[35]*tmpObjS[58] + tmpFx[41]*tmpObjS[68] + tmpFx[47]*tmpObjS[78] + tmpFx[53]*tmpObjS[88] + tmpFx[59]*tmpObjS[98];
tmpQ2[59] = + tmpFx[5]*tmpObjS[9] + tmpFx[11]*tmpObjS[19] + tmpFx[17]*tmpObjS[29] + tmpFx[23]*tmpObjS[39] + tmpFx[29]*tmpObjS[49] + tmpFx[35]*tmpObjS[59] + tmpFx[41]*tmpObjS[69] + tmpFx[47]*tmpObjS[79] + tmpFx[53]*tmpObjS[89] + tmpFx[59]*tmpObjS[99];
tmpQ1[0] = + tmpQ2[0]*tmpFx[0] + tmpQ2[1]*tmpFx[6] + tmpQ2[2]*tmpFx[12] + tmpQ2[3]*tmpFx[18] + tmpQ2[4]*tmpFx[24] + tmpQ2[5]*tmpFx[30] + tmpQ2[6]*tmpFx[36] + tmpQ2[7]*tmpFx[42] + tmpQ2[8]*tmpFx[48] + tmpQ2[9]*tmpFx[54];
tmpQ1[1] = + tmpQ2[0]*tmpFx[1] + tmpQ2[1]*tmpFx[7] + tmpQ2[2]*tmpFx[13] + tmpQ2[3]*tmpFx[19] + tmpQ2[4]*tmpFx[25] + tmpQ2[5]*tmpFx[31] + tmpQ2[6]*tmpFx[37] + tmpQ2[7]*tmpFx[43] + tmpQ2[8]*tmpFx[49] + tmpQ2[9]*tmpFx[55];
tmpQ1[2] = + tmpQ2[0]*tmpFx[2] + tmpQ2[1]*tmpFx[8] + tmpQ2[2]*tmpFx[14] + tmpQ2[3]*tmpFx[20] + tmpQ2[4]*tmpFx[26] + tmpQ2[5]*tmpFx[32] + tmpQ2[6]*tmpFx[38] + tmpQ2[7]*tmpFx[44] + tmpQ2[8]*tmpFx[50] + tmpQ2[9]*tmpFx[56];
tmpQ1[3] = + tmpQ2[0]*tmpFx[3] + tmpQ2[1]*tmpFx[9] + tmpQ2[2]*tmpFx[15] + tmpQ2[3]*tmpFx[21] + tmpQ2[4]*tmpFx[27] + tmpQ2[5]*tmpFx[33] + tmpQ2[6]*tmpFx[39] + tmpQ2[7]*tmpFx[45] + tmpQ2[8]*tmpFx[51] + tmpQ2[9]*tmpFx[57];
tmpQ1[4] = + tmpQ2[0]*tmpFx[4] + tmpQ2[1]*tmpFx[10] + tmpQ2[2]*tmpFx[16] + tmpQ2[3]*tmpFx[22] + tmpQ2[4]*tmpFx[28] + tmpQ2[5]*tmpFx[34] + tmpQ2[6]*tmpFx[40] + tmpQ2[7]*tmpFx[46] + tmpQ2[8]*tmpFx[52] + tmpQ2[9]*tmpFx[58];
tmpQ1[5] = + tmpQ2[0]*tmpFx[5] + tmpQ2[1]*tmpFx[11] + tmpQ2[2]*tmpFx[17] + tmpQ2[3]*tmpFx[23] + tmpQ2[4]*tmpFx[29] + tmpQ2[5]*tmpFx[35] + tmpQ2[6]*tmpFx[41] + tmpQ2[7]*tmpFx[47] + tmpQ2[8]*tmpFx[53] + tmpQ2[9]*tmpFx[59];
tmpQ1[6] = + tmpQ2[10]*tmpFx[0] + tmpQ2[11]*tmpFx[6] + tmpQ2[12]*tmpFx[12] + tmpQ2[13]*tmpFx[18] + tmpQ2[14]*tmpFx[24] + tmpQ2[15]*tmpFx[30] + tmpQ2[16]*tmpFx[36] + tmpQ2[17]*tmpFx[42] + tmpQ2[18]*tmpFx[48] + tmpQ2[19]*tmpFx[54];
tmpQ1[7] = + tmpQ2[10]*tmpFx[1] + tmpQ2[11]*tmpFx[7] + tmpQ2[12]*tmpFx[13] + tmpQ2[13]*tmpFx[19] + tmpQ2[14]*tmpFx[25] + tmpQ2[15]*tmpFx[31] + tmpQ2[16]*tmpFx[37] + tmpQ2[17]*tmpFx[43] + tmpQ2[18]*tmpFx[49] + tmpQ2[19]*tmpFx[55];
tmpQ1[8] = + tmpQ2[10]*tmpFx[2] + tmpQ2[11]*tmpFx[8] + tmpQ2[12]*tmpFx[14] + tmpQ2[13]*tmpFx[20] + tmpQ2[14]*tmpFx[26] + tmpQ2[15]*tmpFx[32] + tmpQ2[16]*tmpFx[38] + tmpQ2[17]*tmpFx[44] + tmpQ2[18]*tmpFx[50] + tmpQ2[19]*tmpFx[56];
tmpQ1[9] = + tmpQ2[10]*tmpFx[3] + tmpQ2[11]*tmpFx[9] + tmpQ2[12]*tmpFx[15] + tmpQ2[13]*tmpFx[21] + tmpQ2[14]*tmpFx[27] + tmpQ2[15]*tmpFx[33] + tmpQ2[16]*tmpFx[39] + tmpQ2[17]*tmpFx[45] + tmpQ2[18]*tmpFx[51] + tmpQ2[19]*tmpFx[57];
tmpQ1[10] = + tmpQ2[10]*tmpFx[4] + tmpQ2[11]*tmpFx[10] + tmpQ2[12]*tmpFx[16] + tmpQ2[13]*tmpFx[22] + tmpQ2[14]*tmpFx[28] + tmpQ2[15]*tmpFx[34] + tmpQ2[16]*tmpFx[40] + tmpQ2[17]*tmpFx[46] + tmpQ2[18]*tmpFx[52] + tmpQ2[19]*tmpFx[58];
tmpQ1[11] = + tmpQ2[10]*tmpFx[5] + tmpQ2[11]*tmpFx[11] + tmpQ2[12]*tmpFx[17] + tmpQ2[13]*tmpFx[23] + tmpQ2[14]*tmpFx[29] + tmpQ2[15]*tmpFx[35] + tmpQ2[16]*tmpFx[41] + tmpQ2[17]*tmpFx[47] + tmpQ2[18]*tmpFx[53] + tmpQ2[19]*tmpFx[59];
tmpQ1[12] = + tmpQ2[20]*tmpFx[0] + tmpQ2[21]*tmpFx[6] + tmpQ2[22]*tmpFx[12] + tmpQ2[23]*tmpFx[18] + tmpQ2[24]*tmpFx[24] + tmpQ2[25]*tmpFx[30] + tmpQ2[26]*tmpFx[36] + tmpQ2[27]*tmpFx[42] + tmpQ2[28]*tmpFx[48] + tmpQ2[29]*tmpFx[54];
tmpQ1[13] = + tmpQ2[20]*tmpFx[1] + tmpQ2[21]*tmpFx[7] + tmpQ2[22]*tmpFx[13] + tmpQ2[23]*tmpFx[19] + tmpQ2[24]*tmpFx[25] + tmpQ2[25]*tmpFx[31] + tmpQ2[26]*tmpFx[37] + tmpQ2[27]*tmpFx[43] + tmpQ2[28]*tmpFx[49] + tmpQ2[29]*tmpFx[55];
tmpQ1[14] = + tmpQ2[20]*tmpFx[2] + tmpQ2[21]*tmpFx[8] + tmpQ2[22]*tmpFx[14] + tmpQ2[23]*tmpFx[20] + tmpQ2[24]*tmpFx[26] + tmpQ2[25]*tmpFx[32] + tmpQ2[26]*tmpFx[38] + tmpQ2[27]*tmpFx[44] + tmpQ2[28]*tmpFx[50] + tmpQ2[29]*tmpFx[56];
tmpQ1[15] = + tmpQ2[20]*tmpFx[3] + tmpQ2[21]*tmpFx[9] + tmpQ2[22]*tmpFx[15] + tmpQ2[23]*tmpFx[21] + tmpQ2[24]*tmpFx[27] + tmpQ2[25]*tmpFx[33] + tmpQ2[26]*tmpFx[39] + tmpQ2[27]*tmpFx[45] + tmpQ2[28]*tmpFx[51] + tmpQ2[29]*tmpFx[57];
tmpQ1[16] = + tmpQ2[20]*tmpFx[4] + tmpQ2[21]*tmpFx[10] + tmpQ2[22]*tmpFx[16] + tmpQ2[23]*tmpFx[22] + tmpQ2[24]*tmpFx[28] + tmpQ2[25]*tmpFx[34] + tmpQ2[26]*tmpFx[40] + tmpQ2[27]*tmpFx[46] + tmpQ2[28]*tmpFx[52] + tmpQ2[29]*tmpFx[58];
tmpQ1[17] = + tmpQ2[20]*tmpFx[5] + tmpQ2[21]*tmpFx[11] + tmpQ2[22]*tmpFx[17] + tmpQ2[23]*tmpFx[23] + tmpQ2[24]*tmpFx[29] + tmpQ2[25]*tmpFx[35] + tmpQ2[26]*tmpFx[41] + tmpQ2[27]*tmpFx[47] + tmpQ2[28]*tmpFx[53] + tmpQ2[29]*tmpFx[59];
tmpQ1[18] = + tmpQ2[30]*tmpFx[0] + tmpQ2[31]*tmpFx[6] + tmpQ2[32]*tmpFx[12] + tmpQ2[33]*tmpFx[18] + tmpQ2[34]*tmpFx[24] + tmpQ2[35]*tmpFx[30] + tmpQ2[36]*tmpFx[36] + tmpQ2[37]*tmpFx[42] + tmpQ2[38]*tmpFx[48] + tmpQ2[39]*tmpFx[54];
tmpQ1[19] = + tmpQ2[30]*tmpFx[1] + tmpQ2[31]*tmpFx[7] + tmpQ2[32]*tmpFx[13] + tmpQ2[33]*tmpFx[19] + tmpQ2[34]*tmpFx[25] + tmpQ2[35]*tmpFx[31] + tmpQ2[36]*tmpFx[37] + tmpQ2[37]*tmpFx[43] + tmpQ2[38]*tmpFx[49] + tmpQ2[39]*tmpFx[55];
tmpQ1[20] = + tmpQ2[30]*tmpFx[2] + tmpQ2[31]*tmpFx[8] + tmpQ2[32]*tmpFx[14] + tmpQ2[33]*tmpFx[20] + tmpQ2[34]*tmpFx[26] + tmpQ2[35]*tmpFx[32] + tmpQ2[36]*tmpFx[38] + tmpQ2[37]*tmpFx[44] + tmpQ2[38]*tmpFx[50] + tmpQ2[39]*tmpFx[56];
tmpQ1[21] = + tmpQ2[30]*tmpFx[3] + tmpQ2[31]*tmpFx[9] + tmpQ2[32]*tmpFx[15] + tmpQ2[33]*tmpFx[21] + tmpQ2[34]*tmpFx[27] + tmpQ2[35]*tmpFx[33] + tmpQ2[36]*tmpFx[39] + tmpQ2[37]*tmpFx[45] + tmpQ2[38]*tmpFx[51] + tmpQ2[39]*tmpFx[57];
tmpQ1[22] = + tmpQ2[30]*tmpFx[4] + tmpQ2[31]*tmpFx[10] + tmpQ2[32]*tmpFx[16] + tmpQ2[33]*tmpFx[22] + tmpQ2[34]*tmpFx[28] + tmpQ2[35]*tmpFx[34] + tmpQ2[36]*tmpFx[40] + tmpQ2[37]*tmpFx[46] + tmpQ2[38]*tmpFx[52] + tmpQ2[39]*tmpFx[58];
tmpQ1[23] = + tmpQ2[30]*tmpFx[5] + tmpQ2[31]*tmpFx[11] + tmpQ2[32]*tmpFx[17] + tmpQ2[33]*tmpFx[23] + tmpQ2[34]*tmpFx[29] + tmpQ2[35]*tmpFx[35] + tmpQ2[36]*tmpFx[41] + tmpQ2[37]*tmpFx[47] + tmpQ2[38]*tmpFx[53] + tmpQ2[39]*tmpFx[59];
tmpQ1[24] = + tmpQ2[40]*tmpFx[0] + tmpQ2[41]*tmpFx[6] + tmpQ2[42]*tmpFx[12] + tmpQ2[43]*tmpFx[18] + tmpQ2[44]*tmpFx[24] + tmpQ2[45]*tmpFx[30] + tmpQ2[46]*tmpFx[36] + tmpQ2[47]*tmpFx[42] + tmpQ2[48]*tmpFx[48] + tmpQ2[49]*tmpFx[54];
tmpQ1[25] = + tmpQ2[40]*tmpFx[1] + tmpQ2[41]*tmpFx[7] + tmpQ2[42]*tmpFx[13] + tmpQ2[43]*tmpFx[19] + tmpQ2[44]*tmpFx[25] + tmpQ2[45]*tmpFx[31] + tmpQ2[46]*tmpFx[37] + tmpQ2[47]*tmpFx[43] + tmpQ2[48]*tmpFx[49] + tmpQ2[49]*tmpFx[55];
tmpQ1[26] = + tmpQ2[40]*tmpFx[2] + tmpQ2[41]*tmpFx[8] + tmpQ2[42]*tmpFx[14] + tmpQ2[43]*tmpFx[20] + tmpQ2[44]*tmpFx[26] + tmpQ2[45]*tmpFx[32] + tmpQ2[46]*tmpFx[38] + tmpQ2[47]*tmpFx[44] + tmpQ2[48]*tmpFx[50] + tmpQ2[49]*tmpFx[56];
tmpQ1[27] = + tmpQ2[40]*tmpFx[3] + tmpQ2[41]*tmpFx[9] + tmpQ2[42]*tmpFx[15] + tmpQ2[43]*tmpFx[21] + tmpQ2[44]*tmpFx[27] + tmpQ2[45]*tmpFx[33] + tmpQ2[46]*tmpFx[39] + tmpQ2[47]*tmpFx[45] + tmpQ2[48]*tmpFx[51] + tmpQ2[49]*tmpFx[57];
tmpQ1[28] = + tmpQ2[40]*tmpFx[4] + tmpQ2[41]*tmpFx[10] + tmpQ2[42]*tmpFx[16] + tmpQ2[43]*tmpFx[22] + tmpQ2[44]*tmpFx[28] + tmpQ2[45]*tmpFx[34] + tmpQ2[46]*tmpFx[40] + tmpQ2[47]*tmpFx[46] + tmpQ2[48]*tmpFx[52] + tmpQ2[49]*tmpFx[58];
tmpQ1[29] = + tmpQ2[40]*tmpFx[5] + tmpQ2[41]*tmpFx[11] + tmpQ2[42]*tmpFx[17] + tmpQ2[43]*tmpFx[23] + tmpQ2[44]*tmpFx[29] + tmpQ2[45]*tmpFx[35] + tmpQ2[46]*tmpFx[41] + tmpQ2[47]*tmpFx[47] + tmpQ2[48]*tmpFx[53] + tmpQ2[49]*tmpFx[59];
tmpQ1[30] = + tmpQ2[50]*tmpFx[0] + tmpQ2[51]*tmpFx[6] + tmpQ2[52]*tmpFx[12] + tmpQ2[53]*tmpFx[18] + tmpQ2[54]*tmpFx[24] + tmpQ2[55]*tmpFx[30] + tmpQ2[56]*tmpFx[36] + tmpQ2[57]*tmpFx[42] + tmpQ2[58]*tmpFx[48] + tmpQ2[59]*tmpFx[54];
tmpQ1[31] = + tmpQ2[50]*tmpFx[1] + tmpQ2[51]*tmpFx[7] + tmpQ2[52]*tmpFx[13] + tmpQ2[53]*tmpFx[19] + tmpQ2[54]*tmpFx[25] + tmpQ2[55]*tmpFx[31] + tmpQ2[56]*tmpFx[37] + tmpQ2[57]*tmpFx[43] + tmpQ2[58]*tmpFx[49] + tmpQ2[59]*tmpFx[55];
tmpQ1[32] = + tmpQ2[50]*tmpFx[2] + tmpQ2[51]*tmpFx[8] + tmpQ2[52]*tmpFx[14] + tmpQ2[53]*tmpFx[20] + tmpQ2[54]*tmpFx[26] + tmpQ2[55]*tmpFx[32] + tmpQ2[56]*tmpFx[38] + tmpQ2[57]*tmpFx[44] + tmpQ2[58]*tmpFx[50] + tmpQ2[59]*tmpFx[56];
tmpQ1[33] = + tmpQ2[50]*tmpFx[3] + tmpQ2[51]*tmpFx[9] + tmpQ2[52]*tmpFx[15] + tmpQ2[53]*tmpFx[21] + tmpQ2[54]*tmpFx[27] + tmpQ2[55]*tmpFx[33] + tmpQ2[56]*tmpFx[39] + tmpQ2[57]*tmpFx[45] + tmpQ2[58]*tmpFx[51] + tmpQ2[59]*tmpFx[57];
tmpQ1[34] = + tmpQ2[50]*tmpFx[4] + tmpQ2[51]*tmpFx[10] + tmpQ2[52]*tmpFx[16] + tmpQ2[53]*tmpFx[22] + tmpQ2[54]*tmpFx[28] + tmpQ2[55]*tmpFx[34] + tmpQ2[56]*tmpFx[40] + tmpQ2[57]*tmpFx[46] + tmpQ2[58]*tmpFx[52] + tmpQ2[59]*tmpFx[58];
tmpQ1[35] = + tmpQ2[50]*tmpFx[5] + tmpQ2[51]*tmpFx[11] + tmpQ2[52]*tmpFx[17] + tmpQ2[53]*tmpFx[23] + tmpQ2[54]*tmpFx[29] + tmpQ2[55]*tmpFx[35] + tmpQ2[56]*tmpFx[41] + tmpQ2[57]*tmpFx[47] + tmpQ2[58]*tmpFx[53] + tmpQ2[59]*tmpFx[59];
}

void acado_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[70];
tmpR2[1] = +tmpObjS[71];
tmpR2[2] = +tmpObjS[72];
tmpR2[3] = +tmpObjS[73];
tmpR2[4] = +tmpObjS[74];
tmpR2[5] = +tmpObjS[75];
tmpR2[6] = +tmpObjS[76];
tmpR2[7] = +tmpObjS[77];
tmpR2[8] = +tmpObjS[78];
tmpR2[9] = +tmpObjS[79];
tmpR2[10] = +tmpObjS[80];
tmpR2[11] = +tmpObjS[81];
tmpR2[12] = +tmpObjS[82];
tmpR2[13] = +tmpObjS[83];
tmpR2[14] = +tmpObjS[84];
tmpR2[15] = +tmpObjS[85];
tmpR2[16] = +tmpObjS[86];
tmpR2[17] = +tmpObjS[87];
tmpR2[18] = +tmpObjS[88];
tmpR2[19] = +tmpObjS[89];
tmpR2[20] = +tmpObjS[90];
tmpR2[21] = +tmpObjS[91];
tmpR2[22] = +tmpObjS[92];
tmpR2[23] = +tmpObjS[93];
tmpR2[24] = +tmpObjS[94];
tmpR2[25] = +tmpObjS[95];
tmpR2[26] = +tmpObjS[96];
tmpR2[27] = +tmpObjS[97];
tmpR2[28] = +tmpObjS[98];
tmpR2[29] = +tmpObjS[99];
tmpR1[0] = + tmpR2[7];
tmpR1[1] = + tmpR2[8];
tmpR1[2] = + tmpR2[9];
tmpR1[3] = + tmpR2[17];
tmpR1[4] = + tmpR2[18];
tmpR1[5] = + tmpR2[19];
tmpR1[6] = + tmpR2[27];
tmpR1[7] = + tmpR2[28];
tmpR1[8] = + tmpR2[29];
}

void acado_setObjQN1QN2( real_t* const tmpFx, real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = + tmpFx[0]*tmpObjSEndTerm[0] + tmpFx[6]*tmpObjSEndTerm[7] + tmpFx[12]*tmpObjSEndTerm[14] + tmpFx[18]*tmpObjSEndTerm[21] + tmpFx[24]*tmpObjSEndTerm[28] + tmpFx[30]*tmpObjSEndTerm[35] + tmpFx[36]*tmpObjSEndTerm[42];
tmpQN2[1] = + tmpFx[0]*tmpObjSEndTerm[1] + tmpFx[6]*tmpObjSEndTerm[8] + tmpFx[12]*tmpObjSEndTerm[15] + tmpFx[18]*tmpObjSEndTerm[22] + tmpFx[24]*tmpObjSEndTerm[29] + tmpFx[30]*tmpObjSEndTerm[36] + tmpFx[36]*tmpObjSEndTerm[43];
tmpQN2[2] = + tmpFx[0]*tmpObjSEndTerm[2] + tmpFx[6]*tmpObjSEndTerm[9] + tmpFx[12]*tmpObjSEndTerm[16] + tmpFx[18]*tmpObjSEndTerm[23] + tmpFx[24]*tmpObjSEndTerm[30] + tmpFx[30]*tmpObjSEndTerm[37] + tmpFx[36]*tmpObjSEndTerm[44];
tmpQN2[3] = + tmpFx[0]*tmpObjSEndTerm[3] + tmpFx[6]*tmpObjSEndTerm[10] + tmpFx[12]*tmpObjSEndTerm[17] + tmpFx[18]*tmpObjSEndTerm[24] + tmpFx[24]*tmpObjSEndTerm[31] + tmpFx[30]*tmpObjSEndTerm[38] + tmpFx[36]*tmpObjSEndTerm[45];
tmpQN2[4] = + tmpFx[0]*tmpObjSEndTerm[4] + tmpFx[6]*tmpObjSEndTerm[11] + tmpFx[12]*tmpObjSEndTerm[18] + tmpFx[18]*tmpObjSEndTerm[25] + tmpFx[24]*tmpObjSEndTerm[32] + tmpFx[30]*tmpObjSEndTerm[39] + tmpFx[36]*tmpObjSEndTerm[46];
tmpQN2[5] = + tmpFx[0]*tmpObjSEndTerm[5] + tmpFx[6]*tmpObjSEndTerm[12] + tmpFx[12]*tmpObjSEndTerm[19] + tmpFx[18]*tmpObjSEndTerm[26] + tmpFx[24]*tmpObjSEndTerm[33] + tmpFx[30]*tmpObjSEndTerm[40] + tmpFx[36]*tmpObjSEndTerm[47];
tmpQN2[6] = + tmpFx[0]*tmpObjSEndTerm[6] + tmpFx[6]*tmpObjSEndTerm[13] + tmpFx[12]*tmpObjSEndTerm[20] + tmpFx[18]*tmpObjSEndTerm[27] + tmpFx[24]*tmpObjSEndTerm[34] + tmpFx[30]*tmpObjSEndTerm[41] + tmpFx[36]*tmpObjSEndTerm[48];
tmpQN2[7] = + tmpFx[1]*tmpObjSEndTerm[0] + tmpFx[7]*tmpObjSEndTerm[7] + tmpFx[13]*tmpObjSEndTerm[14] + tmpFx[19]*tmpObjSEndTerm[21] + tmpFx[25]*tmpObjSEndTerm[28] + tmpFx[31]*tmpObjSEndTerm[35] + tmpFx[37]*tmpObjSEndTerm[42];
tmpQN2[8] = + tmpFx[1]*tmpObjSEndTerm[1] + tmpFx[7]*tmpObjSEndTerm[8] + tmpFx[13]*tmpObjSEndTerm[15] + tmpFx[19]*tmpObjSEndTerm[22] + tmpFx[25]*tmpObjSEndTerm[29] + tmpFx[31]*tmpObjSEndTerm[36] + tmpFx[37]*tmpObjSEndTerm[43];
tmpQN2[9] = + tmpFx[1]*tmpObjSEndTerm[2] + tmpFx[7]*tmpObjSEndTerm[9] + tmpFx[13]*tmpObjSEndTerm[16] + tmpFx[19]*tmpObjSEndTerm[23] + tmpFx[25]*tmpObjSEndTerm[30] + tmpFx[31]*tmpObjSEndTerm[37] + tmpFx[37]*tmpObjSEndTerm[44];
tmpQN2[10] = + tmpFx[1]*tmpObjSEndTerm[3] + tmpFx[7]*tmpObjSEndTerm[10] + tmpFx[13]*tmpObjSEndTerm[17] + tmpFx[19]*tmpObjSEndTerm[24] + tmpFx[25]*tmpObjSEndTerm[31] + tmpFx[31]*tmpObjSEndTerm[38] + tmpFx[37]*tmpObjSEndTerm[45];
tmpQN2[11] = + tmpFx[1]*tmpObjSEndTerm[4] + tmpFx[7]*tmpObjSEndTerm[11] + tmpFx[13]*tmpObjSEndTerm[18] + tmpFx[19]*tmpObjSEndTerm[25] + tmpFx[25]*tmpObjSEndTerm[32] + tmpFx[31]*tmpObjSEndTerm[39] + tmpFx[37]*tmpObjSEndTerm[46];
tmpQN2[12] = + tmpFx[1]*tmpObjSEndTerm[5] + tmpFx[7]*tmpObjSEndTerm[12] + tmpFx[13]*tmpObjSEndTerm[19] + tmpFx[19]*tmpObjSEndTerm[26] + tmpFx[25]*tmpObjSEndTerm[33] + tmpFx[31]*tmpObjSEndTerm[40] + tmpFx[37]*tmpObjSEndTerm[47];
tmpQN2[13] = + tmpFx[1]*tmpObjSEndTerm[6] + tmpFx[7]*tmpObjSEndTerm[13] + tmpFx[13]*tmpObjSEndTerm[20] + tmpFx[19]*tmpObjSEndTerm[27] + tmpFx[25]*tmpObjSEndTerm[34] + tmpFx[31]*tmpObjSEndTerm[41] + tmpFx[37]*tmpObjSEndTerm[48];
tmpQN2[14] = + tmpFx[2]*tmpObjSEndTerm[0] + tmpFx[8]*tmpObjSEndTerm[7] + tmpFx[14]*tmpObjSEndTerm[14] + tmpFx[20]*tmpObjSEndTerm[21] + tmpFx[26]*tmpObjSEndTerm[28] + tmpFx[32]*tmpObjSEndTerm[35] + tmpFx[38]*tmpObjSEndTerm[42];
tmpQN2[15] = + tmpFx[2]*tmpObjSEndTerm[1] + tmpFx[8]*tmpObjSEndTerm[8] + tmpFx[14]*tmpObjSEndTerm[15] + tmpFx[20]*tmpObjSEndTerm[22] + tmpFx[26]*tmpObjSEndTerm[29] + tmpFx[32]*tmpObjSEndTerm[36] + tmpFx[38]*tmpObjSEndTerm[43];
tmpQN2[16] = + tmpFx[2]*tmpObjSEndTerm[2] + tmpFx[8]*tmpObjSEndTerm[9] + tmpFx[14]*tmpObjSEndTerm[16] + tmpFx[20]*tmpObjSEndTerm[23] + tmpFx[26]*tmpObjSEndTerm[30] + tmpFx[32]*tmpObjSEndTerm[37] + tmpFx[38]*tmpObjSEndTerm[44];
tmpQN2[17] = + tmpFx[2]*tmpObjSEndTerm[3] + tmpFx[8]*tmpObjSEndTerm[10] + tmpFx[14]*tmpObjSEndTerm[17] + tmpFx[20]*tmpObjSEndTerm[24] + tmpFx[26]*tmpObjSEndTerm[31] + tmpFx[32]*tmpObjSEndTerm[38] + tmpFx[38]*tmpObjSEndTerm[45];
tmpQN2[18] = + tmpFx[2]*tmpObjSEndTerm[4] + tmpFx[8]*tmpObjSEndTerm[11] + tmpFx[14]*tmpObjSEndTerm[18] + tmpFx[20]*tmpObjSEndTerm[25] + tmpFx[26]*tmpObjSEndTerm[32] + tmpFx[32]*tmpObjSEndTerm[39] + tmpFx[38]*tmpObjSEndTerm[46];
tmpQN2[19] = + tmpFx[2]*tmpObjSEndTerm[5] + tmpFx[8]*tmpObjSEndTerm[12] + tmpFx[14]*tmpObjSEndTerm[19] + tmpFx[20]*tmpObjSEndTerm[26] + tmpFx[26]*tmpObjSEndTerm[33] + tmpFx[32]*tmpObjSEndTerm[40] + tmpFx[38]*tmpObjSEndTerm[47];
tmpQN2[20] = + tmpFx[2]*tmpObjSEndTerm[6] + tmpFx[8]*tmpObjSEndTerm[13] + tmpFx[14]*tmpObjSEndTerm[20] + tmpFx[20]*tmpObjSEndTerm[27] + tmpFx[26]*tmpObjSEndTerm[34] + tmpFx[32]*tmpObjSEndTerm[41] + tmpFx[38]*tmpObjSEndTerm[48];
tmpQN2[21] = + tmpFx[3]*tmpObjSEndTerm[0] + tmpFx[9]*tmpObjSEndTerm[7] + tmpFx[15]*tmpObjSEndTerm[14] + tmpFx[21]*tmpObjSEndTerm[21] + tmpFx[27]*tmpObjSEndTerm[28] + tmpFx[33]*tmpObjSEndTerm[35] + tmpFx[39]*tmpObjSEndTerm[42];
tmpQN2[22] = + tmpFx[3]*tmpObjSEndTerm[1] + tmpFx[9]*tmpObjSEndTerm[8] + tmpFx[15]*tmpObjSEndTerm[15] + tmpFx[21]*tmpObjSEndTerm[22] + tmpFx[27]*tmpObjSEndTerm[29] + tmpFx[33]*tmpObjSEndTerm[36] + tmpFx[39]*tmpObjSEndTerm[43];
tmpQN2[23] = + tmpFx[3]*tmpObjSEndTerm[2] + tmpFx[9]*tmpObjSEndTerm[9] + tmpFx[15]*tmpObjSEndTerm[16] + tmpFx[21]*tmpObjSEndTerm[23] + tmpFx[27]*tmpObjSEndTerm[30] + tmpFx[33]*tmpObjSEndTerm[37] + tmpFx[39]*tmpObjSEndTerm[44];
tmpQN2[24] = + tmpFx[3]*tmpObjSEndTerm[3] + tmpFx[9]*tmpObjSEndTerm[10] + tmpFx[15]*tmpObjSEndTerm[17] + tmpFx[21]*tmpObjSEndTerm[24] + tmpFx[27]*tmpObjSEndTerm[31] + tmpFx[33]*tmpObjSEndTerm[38] + tmpFx[39]*tmpObjSEndTerm[45];
tmpQN2[25] = + tmpFx[3]*tmpObjSEndTerm[4] + tmpFx[9]*tmpObjSEndTerm[11] + tmpFx[15]*tmpObjSEndTerm[18] + tmpFx[21]*tmpObjSEndTerm[25] + tmpFx[27]*tmpObjSEndTerm[32] + tmpFx[33]*tmpObjSEndTerm[39] + tmpFx[39]*tmpObjSEndTerm[46];
tmpQN2[26] = + tmpFx[3]*tmpObjSEndTerm[5] + tmpFx[9]*tmpObjSEndTerm[12] + tmpFx[15]*tmpObjSEndTerm[19] + tmpFx[21]*tmpObjSEndTerm[26] + tmpFx[27]*tmpObjSEndTerm[33] + tmpFx[33]*tmpObjSEndTerm[40] + tmpFx[39]*tmpObjSEndTerm[47];
tmpQN2[27] = + tmpFx[3]*tmpObjSEndTerm[6] + tmpFx[9]*tmpObjSEndTerm[13] + tmpFx[15]*tmpObjSEndTerm[20] + tmpFx[21]*tmpObjSEndTerm[27] + tmpFx[27]*tmpObjSEndTerm[34] + tmpFx[33]*tmpObjSEndTerm[41] + tmpFx[39]*tmpObjSEndTerm[48];
tmpQN2[28] = + tmpFx[4]*tmpObjSEndTerm[0] + tmpFx[10]*tmpObjSEndTerm[7] + tmpFx[16]*tmpObjSEndTerm[14] + tmpFx[22]*tmpObjSEndTerm[21] + tmpFx[28]*tmpObjSEndTerm[28] + tmpFx[34]*tmpObjSEndTerm[35] + tmpFx[40]*tmpObjSEndTerm[42];
tmpQN2[29] = + tmpFx[4]*tmpObjSEndTerm[1] + tmpFx[10]*tmpObjSEndTerm[8] + tmpFx[16]*tmpObjSEndTerm[15] + tmpFx[22]*tmpObjSEndTerm[22] + tmpFx[28]*tmpObjSEndTerm[29] + tmpFx[34]*tmpObjSEndTerm[36] + tmpFx[40]*tmpObjSEndTerm[43];
tmpQN2[30] = + tmpFx[4]*tmpObjSEndTerm[2] + tmpFx[10]*tmpObjSEndTerm[9] + tmpFx[16]*tmpObjSEndTerm[16] + tmpFx[22]*tmpObjSEndTerm[23] + tmpFx[28]*tmpObjSEndTerm[30] + tmpFx[34]*tmpObjSEndTerm[37] + tmpFx[40]*tmpObjSEndTerm[44];
tmpQN2[31] = + tmpFx[4]*tmpObjSEndTerm[3] + tmpFx[10]*tmpObjSEndTerm[10] + tmpFx[16]*tmpObjSEndTerm[17] + tmpFx[22]*tmpObjSEndTerm[24] + tmpFx[28]*tmpObjSEndTerm[31] + tmpFx[34]*tmpObjSEndTerm[38] + tmpFx[40]*tmpObjSEndTerm[45];
tmpQN2[32] = + tmpFx[4]*tmpObjSEndTerm[4] + tmpFx[10]*tmpObjSEndTerm[11] + tmpFx[16]*tmpObjSEndTerm[18] + tmpFx[22]*tmpObjSEndTerm[25] + tmpFx[28]*tmpObjSEndTerm[32] + tmpFx[34]*tmpObjSEndTerm[39] + tmpFx[40]*tmpObjSEndTerm[46];
tmpQN2[33] = + tmpFx[4]*tmpObjSEndTerm[5] + tmpFx[10]*tmpObjSEndTerm[12] + tmpFx[16]*tmpObjSEndTerm[19] + tmpFx[22]*tmpObjSEndTerm[26] + tmpFx[28]*tmpObjSEndTerm[33] + tmpFx[34]*tmpObjSEndTerm[40] + tmpFx[40]*tmpObjSEndTerm[47];
tmpQN2[34] = + tmpFx[4]*tmpObjSEndTerm[6] + tmpFx[10]*tmpObjSEndTerm[13] + tmpFx[16]*tmpObjSEndTerm[20] + tmpFx[22]*tmpObjSEndTerm[27] + tmpFx[28]*tmpObjSEndTerm[34] + tmpFx[34]*tmpObjSEndTerm[41] + tmpFx[40]*tmpObjSEndTerm[48];
tmpQN2[35] = + tmpFx[5]*tmpObjSEndTerm[0] + tmpFx[11]*tmpObjSEndTerm[7] + tmpFx[17]*tmpObjSEndTerm[14] + tmpFx[23]*tmpObjSEndTerm[21] + tmpFx[29]*tmpObjSEndTerm[28] + tmpFx[35]*tmpObjSEndTerm[35] + tmpFx[41]*tmpObjSEndTerm[42];
tmpQN2[36] = + tmpFx[5]*tmpObjSEndTerm[1] + tmpFx[11]*tmpObjSEndTerm[8] + tmpFx[17]*tmpObjSEndTerm[15] + tmpFx[23]*tmpObjSEndTerm[22] + tmpFx[29]*tmpObjSEndTerm[29] + tmpFx[35]*tmpObjSEndTerm[36] + tmpFx[41]*tmpObjSEndTerm[43];
tmpQN2[37] = + tmpFx[5]*tmpObjSEndTerm[2] + tmpFx[11]*tmpObjSEndTerm[9] + tmpFx[17]*tmpObjSEndTerm[16] + tmpFx[23]*tmpObjSEndTerm[23] + tmpFx[29]*tmpObjSEndTerm[30] + tmpFx[35]*tmpObjSEndTerm[37] + tmpFx[41]*tmpObjSEndTerm[44];
tmpQN2[38] = + tmpFx[5]*tmpObjSEndTerm[3] + tmpFx[11]*tmpObjSEndTerm[10] + tmpFx[17]*tmpObjSEndTerm[17] + tmpFx[23]*tmpObjSEndTerm[24] + tmpFx[29]*tmpObjSEndTerm[31] + tmpFx[35]*tmpObjSEndTerm[38] + tmpFx[41]*tmpObjSEndTerm[45];
tmpQN2[39] = + tmpFx[5]*tmpObjSEndTerm[4] + tmpFx[11]*tmpObjSEndTerm[11] + tmpFx[17]*tmpObjSEndTerm[18] + tmpFx[23]*tmpObjSEndTerm[25] + tmpFx[29]*tmpObjSEndTerm[32] + tmpFx[35]*tmpObjSEndTerm[39] + tmpFx[41]*tmpObjSEndTerm[46];
tmpQN2[40] = + tmpFx[5]*tmpObjSEndTerm[5] + tmpFx[11]*tmpObjSEndTerm[12] + tmpFx[17]*tmpObjSEndTerm[19] + tmpFx[23]*tmpObjSEndTerm[26] + tmpFx[29]*tmpObjSEndTerm[33] + tmpFx[35]*tmpObjSEndTerm[40] + tmpFx[41]*tmpObjSEndTerm[47];
tmpQN2[41] = + tmpFx[5]*tmpObjSEndTerm[6] + tmpFx[11]*tmpObjSEndTerm[13] + tmpFx[17]*tmpObjSEndTerm[20] + tmpFx[23]*tmpObjSEndTerm[27] + tmpFx[29]*tmpObjSEndTerm[34] + tmpFx[35]*tmpObjSEndTerm[41] + tmpFx[41]*tmpObjSEndTerm[48];
tmpQN1[0] = + tmpQN2[0]*tmpFx[0] + tmpQN2[1]*tmpFx[6] + tmpQN2[2]*tmpFx[12] + tmpQN2[3]*tmpFx[18] + tmpQN2[4]*tmpFx[24] + tmpQN2[5]*tmpFx[30] + tmpQN2[6]*tmpFx[36];
tmpQN1[1] = + tmpQN2[0]*tmpFx[1] + tmpQN2[1]*tmpFx[7] + tmpQN2[2]*tmpFx[13] + tmpQN2[3]*tmpFx[19] + tmpQN2[4]*tmpFx[25] + tmpQN2[5]*tmpFx[31] + tmpQN2[6]*tmpFx[37];
tmpQN1[2] = + tmpQN2[0]*tmpFx[2] + tmpQN2[1]*tmpFx[8] + tmpQN2[2]*tmpFx[14] + tmpQN2[3]*tmpFx[20] + tmpQN2[4]*tmpFx[26] + tmpQN2[5]*tmpFx[32] + tmpQN2[6]*tmpFx[38];
tmpQN1[3] = + tmpQN2[0]*tmpFx[3] + tmpQN2[1]*tmpFx[9] + tmpQN2[2]*tmpFx[15] + tmpQN2[3]*tmpFx[21] + tmpQN2[4]*tmpFx[27] + tmpQN2[5]*tmpFx[33] + tmpQN2[6]*tmpFx[39];
tmpQN1[4] = + tmpQN2[0]*tmpFx[4] + tmpQN2[1]*tmpFx[10] + tmpQN2[2]*tmpFx[16] + tmpQN2[3]*tmpFx[22] + tmpQN2[4]*tmpFx[28] + tmpQN2[5]*tmpFx[34] + tmpQN2[6]*tmpFx[40];
tmpQN1[5] = + tmpQN2[0]*tmpFx[5] + tmpQN2[1]*tmpFx[11] + tmpQN2[2]*tmpFx[17] + tmpQN2[3]*tmpFx[23] + tmpQN2[4]*tmpFx[29] + tmpQN2[5]*tmpFx[35] + tmpQN2[6]*tmpFx[41];
tmpQN1[6] = + tmpQN2[7]*tmpFx[0] + tmpQN2[8]*tmpFx[6] + tmpQN2[9]*tmpFx[12] + tmpQN2[10]*tmpFx[18] + tmpQN2[11]*tmpFx[24] + tmpQN2[12]*tmpFx[30] + tmpQN2[13]*tmpFx[36];
tmpQN1[7] = + tmpQN2[7]*tmpFx[1] + tmpQN2[8]*tmpFx[7] + tmpQN2[9]*tmpFx[13] + tmpQN2[10]*tmpFx[19] + tmpQN2[11]*tmpFx[25] + tmpQN2[12]*tmpFx[31] + tmpQN2[13]*tmpFx[37];
tmpQN1[8] = + tmpQN2[7]*tmpFx[2] + tmpQN2[8]*tmpFx[8] + tmpQN2[9]*tmpFx[14] + tmpQN2[10]*tmpFx[20] + tmpQN2[11]*tmpFx[26] + tmpQN2[12]*tmpFx[32] + tmpQN2[13]*tmpFx[38];
tmpQN1[9] = + tmpQN2[7]*tmpFx[3] + tmpQN2[8]*tmpFx[9] + tmpQN2[9]*tmpFx[15] + tmpQN2[10]*tmpFx[21] + tmpQN2[11]*tmpFx[27] + tmpQN2[12]*tmpFx[33] + tmpQN2[13]*tmpFx[39];
tmpQN1[10] = + tmpQN2[7]*tmpFx[4] + tmpQN2[8]*tmpFx[10] + tmpQN2[9]*tmpFx[16] + tmpQN2[10]*tmpFx[22] + tmpQN2[11]*tmpFx[28] + tmpQN2[12]*tmpFx[34] + tmpQN2[13]*tmpFx[40];
tmpQN1[11] = + tmpQN2[7]*tmpFx[5] + tmpQN2[8]*tmpFx[11] + tmpQN2[9]*tmpFx[17] + tmpQN2[10]*tmpFx[23] + tmpQN2[11]*tmpFx[29] + tmpQN2[12]*tmpFx[35] + tmpQN2[13]*tmpFx[41];
tmpQN1[12] = + tmpQN2[14]*tmpFx[0] + tmpQN2[15]*tmpFx[6] + tmpQN2[16]*tmpFx[12] + tmpQN2[17]*tmpFx[18] + tmpQN2[18]*tmpFx[24] + tmpQN2[19]*tmpFx[30] + tmpQN2[20]*tmpFx[36];
tmpQN1[13] = + tmpQN2[14]*tmpFx[1] + tmpQN2[15]*tmpFx[7] + tmpQN2[16]*tmpFx[13] + tmpQN2[17]*tmpFx[19] + tmpQN2[18]*tmpFx[25] + tmpQN2[19]*tmpFx[31] + tmpQN2[20]*tmpFx[37];
tmpQN1[14] = + tmpQN2[14]*tmpFx[2] + tmpQN2[15]*tmpFx[8] + tmpQN2[16]*tmpFx[14] + tmpQN2[17]*tmpFx[20] + tmpQN2[18]*tmpFx[26] + tmpQN2[19]*tmpFx[32] + tmpQN2[20]*tmpFx[38];
tmpQN1[15] = + tmpQN2[14]*tmpFx[3] + tmpQN2[15]*tmpFx[9] + tmpQN2[16]*tmpFx[15] + tmpQN2[17]*tmpFx[21] + tmpQN2[18]*tmpFx[27] + tmpQN2[19]*tmpFx[33] + tmpQN2[20]*tmpFx[39];
tmpQN1[16] = + tmpQN2[14]*tmpFx[4] + tmpQN2[15]*tmpFx[10] + tmpQN2[16]*tmpFx[16] + tmpQN2[17]*tmpFx[22] + tmpQN2[18]*tmpFx[28] + tmpQN2[19]*tmpFx[34] + tmpQN2[20]*tmpFx[40];
tmpQN1[17] = + tmpQN2[14]*tmpFx[5] + tmpQN2[15]*tmpFx[11] + tmpQN2[16]*tmpFx[17] + tmpQN2[17]*tmpFx[23] + tmpQN2[18]*tmpFx[29] + tmpQN2[19]*tmpFx[35] + tmpQN2[20]*tmpFx[41];
tmpQN1[18] = + tmpQN2[21]*tmpFx[0] + tmpQN2[22]*tmpFx[6] + tmpQN2[23]*tmpFx[12] + tmpQN2[24]*tmpFx[18] + tmpQN2[25]*tmpFx[24] + tmpQN2[26]*tmpFx[30] + tmpQN2[27]*tmpFx[36];
tmpQN1[19] = + tmpQN2[21]*tmpFx[1] + tmpQN2[22]*tmpFx[7] + tmpQN2[23]*tmpFx[13] + tmpQN2[24]*tmpFx[19] + tmpQN2[25]*tmpFx[25] + tmpQN2[26]*tmpFx[31] + tmpQN2[27]*tmpFx[37];
tmpQN1[20] = + tmpQN2[21]*tmpFx[2] + tmpQN2[22]*tmpFx[8] + tmpQN2[23]*tmpFx[14] + tmpQN2[24]*tmpFx[20] + tmpQN2[25]*tmpFx[26] + tmpQN2[26]*tmpFx[32] + tmpQN2[27]*tmpFx[38];
tmpQN1[21] = + tmpQN2[21]*tmpFx[3] + tmpQN2[22]*tmpFx[9] + tmpQN2[23]*tmpFx[15] + tmpQN2[24]*tmpFx[21] + tmpQN2[25]*tmpFx[27] + tmpQN2[26]*tmpFx[33] + tmpQN2[27]*tmpFx[39];
tmpQN1[22] = + tmpQN2[21]*tmpFx[4] + tmpQN2[22]*tmpFx[10] + tmpQN2[23]*tmpFx[16] + tmpQN2[24]*tmpFx[22] + tmpQN2[25]*tmpFx[28] + tmpQN2[26]*tmpFx[34] + tmpQN2[27]*tmpFx[40];
tmpQN1[23] = + tmpQN2[21]*tmpFx[5] + tmpQN2[22]*tmpFx[11] + tmpQN2[23]*tmpFx[17] + tmpQN2[24]*tmpFx[23] + tmpQN2[25]*tmpFx[29] + tmpQN2[26]*tmpFx[35] + tmpQN2[27]*tmpFx[41];
tmpQN1[24] = + tmpQN2[28]*tmpFx[0] + tmpQN2[29]*tmpFx[6] + tmpQN2[30]*tmpFx[12] + tmpQN2[31]*tmpFx[18] + tmpQN2[32]*tmpFx[24] + tmpQN2[33]*tmpFx[30] + tmpQN2[34]*tmpFx[36];
tmpQN1[25] = + tmpQN2[28]*tmpFx[1] + tmpQN2[29]*tmpFx[7] + tmpQN2[30]*tmpFx[13] + tmpQN2[31]*tmpFx[19] + tmpQN2[32]*tmpFx[25] + tmpQN2[33]*tmpFx[31] + tmpQN2[34]*tmpFx[37];
tmpQN1[26] = + tmpQN2[28]*tmpFx[2] + tmpQN2[29]*tmpFx[8] + tmpQN2[30]*tmpFx[14] + tmpQN2[31]*tmpFx[20] + tmpQN2[32]*tmpFx[26] + tmpQN2[33]*tmpFx[32] + tmpQN2[34]*tmpFx[38];
tmpQN1[27] = + tmpQN2[28]*tmpFx[3] + tmpQN2[29]*tmpFx[9] + tmpQN2[30]*tmpFx[15] + tmpQN2[31]*tmpFx[21] + tmpQN2[32]*tmpFx[27] + tmpQN2[33]*tmpFx[33] + tmpQN2[34]*tmpFx[39];
tmpQN1[28] = + tmpQN2[28]*tmpFx[4] + tmpQN2[29]*tmpFx[10] + tmpQN2[30]*tmpFx[16] + tmpQN2[31]*tmpFx[22] + tmpQN2[32]*tmpFx[28] + tmpQN2[33]*tmpFx[34] + tmpQN2[34]*tmpFx[40];
tmpQN1[29] = + tmpQN2[28]*tmpFx[5] + tmpQN2[29]*tmpFx[11] + tmpQN2[30]*tmpFx[17] + tmpQN2[31]*tmpFx[23] + tmpQN2[32]*tmpFx[29] + tmpQN2[33]*tmpFx[35] + tmpQN2[34]*tmpFx[41];
tmpQN1[30] = + tmpQN2[35]*tmpFx[0] + tmpQN2[36]*tmpFx[6] + tmpQN2[37]*tmpFx[12] + tmpQN2[38]*tmpFx[18] + tmpQN2[39]*tmpFx[24] + tmpQN2[40]*tmpFx[30] + tmpQN2[41]*tmpFx[36];
tmpQN1[31] = + tmpQN2[35]*tmpFx[1] + tmpQN2[36]*tmpFx[7] + tmpQN2[37]*tmpFx[13] + tmpQN2[38]*tmpFx[19] + tmpQN2[39]*tmpFx[25] + tmpQN2[40]*tmpFx[31] + tmpQN2[41]*tmpFx[37];
tmpQN1[32] = + tmpQN2[35]*tmpFx[2] + tmpQN2[36]*tmpFx[8] + tmpQN2[37]*tmpFx[14] + tmpQN2[38]*tmpFx[20] + tmpQN2[39]*tmpFx[26] + tmpQN2[40]*tmpFx[32] + tmpQN2[41]*tmpFx[38];
tmpQN1[33] = + tmpQN2[35]*tmpFx[3] + tmpQN2[36]*tmpFx[9] + tmpQN2[37]*tmpFx[15] + tmpQN2[38]*tmpFx[21] + tmpQN2[39]*tmpFx[27] + tmpQN2[40]*tmpFx[33] + tmpQN2[41]*tmpFx[39];
tmpQN1[34] = + tmpQN2[35]*tmpFx[4] + tmpQN2[36]*tmpFx[10] + tmpQN2[37]*tmpFx[16] + tmpQN2[38]*tmpFx[22] + tmpQN2[39]*tmpFx[28] + tmpQN2[40]*tmpFx[34] + tmpQN2[41]*tmpFx[40];
tmpQN1[35] = + tmpQN2[35]*tmpFx[5] + tmpQN2[36]*tmpFx[11] + tmpQN2[37]*tmpFx[17] + tmpQN2[38]*tmpFx[23] + tmpQN2[39]*tmpFx[29] + tmpQN2[40]*tmpFx[35] + tmpQN2[41]*tmpFx[41];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 25; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 6];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 6 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 6 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 6 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 6 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[runObj * 6 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.u[runObj * 3];
acadoWorkspace.objValueIn[7] = acadoVariables.u[runObj * 3 + 1];
acadoWorkspace.objValueIn[8] = acadoVariables.u[runObj * 3 + 2];
acadoWorkspace.objValueIn[9] = acadoVariables.od[runObj * 3];
acadoWorkspace.objValueIn[10] = acadoVariables.od[runObj * 3 + 1];
acadoWorkspace.objValueIn[11] = acadoVariables.od[runObj * 3 + 2];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 10] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 10 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 10 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 10 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 10 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 10 + 5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.Dy[runObj * 10 + 6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.Dy[runObj * 10 + 7] = acadoWorkspace.objValueOut[7];
acadoWorkspace.Dy[runObj * 10 + 8] = acadoWorkspace.objValueOut[8];
acadoWorkspace.Dy[runObj * 10 + 9] = acadoWorkspace.objValueOut[9];

acado_setObjQ1Q2( &(acadoWorkspace.objValueOut[ 10 ]), acadoVariables.W, &(acadoWorkspace.Q1[ runObj * 36 ]), &(acadoWorkspace.Q2[ runObj * 60 ]) );

acado_setObjR1R2( acadoVariables.W, &(acadoWorkspace.R1[ runObj * 9 ]), &(acadoWorkspace.R2[ runObj * 30 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[150];
acadoWorkspace.objValueIn[1] = acadoVariables.x[151];
acadoWorkspace.objValueIn[2] = acadoVariables.x[152];
acadoWorkspace.objValueIn[3] = acadoVariables.x[153];
acadoWorkspace.objValueIn[4] = acadoVariables.x[154];
acadoWorkspace.objValueIn[5] = acadoVariables.x[155];
acadoWorkspace.objValueIn[6] = acadoVariables.od[75];
acadoWorkspace.objValueIn[7] = acadoVariables.od[76];
acadoWorkspace.objValueIn[8] = acadoVariables.od[77];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.DyN[6] = acadoWorkspace.objValueOut[6];

acado_setObjQN1QN2( &(acadoWorkspace.objValueOut[ 7 ]), acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = Gx1[0];
Gx2[1] = Gx1[1];
Gx2[2] = Gx1[2];
Gx2[3] = Gx1[3];
Gx2[4] = Gx1[4];
Gx2[5] = Gx1[5];
Gx2[6] = Gx1[6];
Gx2[7] = Gx1[7];
Gx2[8] = Gx1[8];
Gx2[9] = Gx1[9];
Gx2[10] = Gx1[10];
Gx2[11] = Gx1[11];
Gx2[12] = Gx1[12];
Gx2[13] = Gx1[13];
Gx2[14] = Gx1[14];
Gx2[15] = Gx1[15];
Gx2[16] = Gx1[16];
Gx2[17] = Gx1[17];
Gx2[18] = Gx1[18];
Gx2[19] = Gx1[19];
Gx2[20] = Gx1[20];
Gx2[21] = Gx1[21];
Gx2[22] = Gx1[22];
Gx2[23] = Gx1[23];
Gx2[24] = Gx1[24];
Gx2[25] = Gx1[25];
Gx2[26] = Gx1[26];
Gx2[27] = Gx1[27];
Gx2[28] = Gx1[28];
Gx2[29] = Gx1[29];
Gx2[30] = Gx1[30];
Gx2[31] = Gx1[31];
Gx2[32] = Gx1[32];
Gx2[33] = Gx1[33];
Gx2[34] = Gx1[34];
Gx2[35] = Gx1[35];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[6] + Gx1[2]*Gx2[12] + Gx1[3]*Gx2[18] + Gx1[4]*Gx2[24] + Gx1[5]*Gx2[30];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[7] + Gx1[2]*Gx2[13] + Gx1[3]*Gx2[19] + Gx1[4]*Gx2[25] + Gx1[5]*Gx2[31];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[8] + Gx1[2]*Gx2[14] + Gx1[3]*Gx2[20] + Gx1[4]*Gx2[26] + Gx1[5]*Gx2[32];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[9] + Gx1[2]*Gx2[15] + Gx1[3]*Gx2[21] + Gx1[4]*Gx2[27] + Gx1[5]*Gx2[33];
Gx3[4] = + Gx1[0]*Gx2[4] + Gx1[1]*Gx2[10] + Gx1[2]*Gx2[16] + Gx1[3]*Gx2[22] + Gx1[4]*Gx2[28] + Gx1[5]*Gx2[34];
Gx3[5] = + Gx1[0]*Gx2[5] + Gx1[1]*Gx2[11] + Gx1[2]*Gx2[17] + Gx1[3]*Gx2[23] + Gx1[4]*Gx2[29] + Gx1[5]*Gx2[35];
Gx3[6] = + Gx1[6]*Gx2[0] + Gx1[7]*Gx2[6] + Gx1[8]*Gx2[12] + Gx1[9]*Gx2[18] + Gx1[10]*Gx2[24] + Gx1[11]*Gx2[30];
Gx3[7] = + Gx1[6]*Gx2[1] + Gx1[7]*Gx2[7] + Gx1[8]*Gx2[13] + Gx1[9]*Gx2[19] + Gx1[10]*Gx2[25] + Gx1[11]*Gx2[31];
Gx3[8] = + Gx1[6]*Gx2[2] + Gx1[7]*Gx2[8] + Gx1[8]*Gx2[14] + Gx1[9]*Gx2[20] + Gx1[10]*Gx2[26] + Gx1[11]*Gx2[32];
Gx3[9] = + Gx1[6]*Gx2[3] + Gx1[7]*Gx2[9] + Gx1[8]*Gx2[15] + Gx1[9]*Gx2[21] + Gx1[10]*Gx2[27] + Gx1[11]*Gx2[33];
Gx3[10] = + Gx1[6]*Gx2[4] + Gx1[7]*Gx2[10] + Gx1[8]*Gx2[16] + Gx1[9]*Gx2[22] + Gx1[10]*Gx2[28] + Gx1[11]*Gx2[34];
Gx3[11] = + Gx1[6]*Gx2[5] + Gx1[7]*Gx2[11] + Gx1[8]*Gx2[17] + Gx1[9]*Gx2[23] + Gx1[10]*Gx2[29] + Gx1[11]*Gx2[35];
Gx3[12] = + Gx1[12]*Gx2[0] + Gx1[13]*Gx2[6] + Gx1[14]*Gx2[12] + Gx1[15]*Gx2[18] + Gx1[16]*Gx2[24] + Gx1[17]*Gx2[30];
Gx3[13] = + Gx1[12]*Gx2[1] + Gx1[13]*Gx2[7] + Gx1[14]*Gx2[13] + Gx1[15]*Gx2[19] + Gx1[16]*Gx2[25] + Gx1[17]*Gx2[31];
Gx3[14] = + Gx1[12]*Gx2[2] + Gx1[13]*Gx2[8] + Gx1[14]*Gx2[14] + Gx1[15]*Gx2[20] + Gx1[16]*Gx2[26] + Gx1[17]*Gx2[32];
Gx3[15] = + Gx1[12]*Gx2[3] + Gx1[13]*Gx2[9] + Gx1[14]*Gx2[15] + Gx1[15]*Gx2[21] + Gx1[16]*Gx2[27] + Gx1[17]*Gx2[33];
Gx3[16] = + Gx1[12]*Gx2[4] + Gx1[13]*Gx2[10] + Gx1[14]*Gx2[16] + Gx1[15]*Gx2[22] + Gx1[16]*Gx2[28] + Gx1[17]*Gx2[34];
Gx3[17] = + Gx1[12]*Gx2[5] + Gx1[13]*Gx2[11] + Gx1[14]*Gx2[17] + Gx1[15]*Gx2[23] + Gx1[16]*Gx2[29] + Gx1[17]*Gx2[35];
Gx3[18] = + Gx1[18]*Gx2[0] + Gx1[19]*Gx2[6] + Gx1[20]*Gx2[12] + Gx1[21]*Gx2[18] + Gx1[22]*Gx2[24] + Gx1[23]*Gx2[30];
Gx3[19] = + Gx1[18]*Gx2[1] + Gx1[19]*Gx2[7] + Gx1[20]*Gx2[13] + Gx1[21]*Gx2[19] + Gx1[22]*Gx2[25] + Gx1[23]*Gx2[31];
Gx3[20] = + Gx1[18]*Gx2[2] + Gx1[19]*Gx2[8] + Gx1[20]*Gx2[14] + Gx1[21]*Gx2[20] + Gx1[22]*Gx2[26] + Gx1[23]*Gx2[32];
Gx3[21] = + Gx1[18]*Gx2[3] + Gx1[19]*Gx2[9] + Gx1[20]*Gx2[15] + Gx1[21]*Gx2[21] + Gx1[22]*Gx2[27] + Gx1[23]*Gx2[33];
Gx3[22] = + Gx1[18]*Gx2[4] + Gx1[19]*Gx2[10] + Gx1[20]*Gx2[16] + Gx1[21]*Gx2[22] + Gx1[22]*Gx2[28] + Gx1[23]*Gx2[34];
Gx3[23] = + Gx1[18]*Gx2[5] + Gx1[19]*Gx2[11] + Gx1[20]*Gx2[17] + Gx1[21]*Gx2[23] + Gx1[22]*Gx2[29] + Gx1[23]*Gx2[35];
Gx3[24] = + Gx1[24]*Gx2[0] + Gx1[25]*Gx2[6] + Gx1[26]*Gx2[12] + Gx1[27]*Gx2[18] + Gx1[28]*Gx2[24] + Gx1[29]*Gx2[30];
Gx3[25] = + Gx1[24]*Gx2[1] + Gx1[25]*Gx2[7] + Gx1[26]*Gx2[13] + Gx1[27]*Gx2[19] + Gx1[28]*Gx2[25] + Gx1[29]*Gx2[31];
Gx3[26] = + Gx1[24]*Gx2[2] + Gx1[25]*Gx2[8] + Gx1[26]*Gx2[14] + Gx1[27]*Gx2[20] + Gx1[28]*Gx2[26] + Gx1[29]*Gx2[32];
Gx3[27] = + Gx1[24]*Gx2[3] + Gx1[25]*Gx2[9] + Gx1[26]*Gx2[15] + Gx1[27]*Gx2[21] + Gx1[28]*Gx2[27] + Gx1[29]*Gx2[33];
Gx3[28] = + Gx1[24]*Gx2[4] + Gx1[25]*Gx2[10] + Gx1[26]*Gx2[16] + Gx1[27]*Gx2[22] + Gx1[28]*Gx2[28] + Gx1[29]*Gx2[34];
Gx3[29] = + Gx1[24]*Gx2[5] + Gx1[25]*Gx2[11] + Gx1[26]*Gx2[17] + Gx1[27]*Gx2[23] + Gx1[28]*Gx2[29] + Gx1[29]*Gx2[35];
Gx3[30] = + Gx1[30]*Gx2[0] + Gx1[31]*Gx2[6] + Gx1[32]*Gx2[12] + Gx1[33]*Gx2[18] + Gx1[34]*Gx2[24] + Gx1[35]*Gx2[30];
Gx3[31] = + Gx1[30]*Gx2[1] + Gx1[31]*Gx2[7] + Gx1[32]*Gx2[13] + Gx1[33]*Gx2[19] + Gx1[34]*Gx2[25] + Gx1[35]*Gx2[31];
Gx3[32] = + Gx1[30]*Gx2[2] + Gx1[31]*Gx2[8] + Gx1[32]*Gx2[14] + Gx1[33]*Gx2[20] + Gx1[34]*Gx2[26] + Gx1[35]*Gx2[32];
Gx3[33] = + Gx1[30]*Gx2[3] + Gx1[31]*Gx2[9] + Gx1[32]*Gx2[15] + Gx1[33]*Gx2[21] + Gx1[34]*Gx2[27] + Gx1[35]*Gx2[33];
Gx3[34] = + Gx1[30]*Gx2[4] + Gx1[31]*Gx2[10] + Gx1[32]*Gx2[16] + Gx1[33]*Gx2[22] + Gx1[34]*Gx2[28] + Gx1[35]*Gx2[34];
Gx3[35] = + Gx1[30]*Gx2[5] + Gx1[31]*Gx2[11] + Gx1[32]*Gx2[17] + Gx1[33]*Gx2[23] + Gx1[34]*Gx2[29] + Gx1[35]*Gx2[35];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[6] + Gx1[3]*Gu1[9] + Gx1[4]*Gu1[12] + Gx1[5]*Gu1[15];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[4] + Gx1[2]*Gu1[7] + Gx1[3]*Gu1[10] + Gx1[4]*Gu1[13] + Gx1[5]*Gu1[16];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[1]*Gu1[5] + Gx1[2]*Gu1[8] + Gx1[3]*Gu1[11] + Gx1[4]*Gu1[14] + Gx1[5]*Gu1[17];
Gu2[3] = + Gx1[6]*Gu1[0] + Gx1[7]*Gu1[3] + Gx1[8]*Gu1[6] + Gx1[9]*Gu1[9] + Gx1[10]*Gu1[12] + Gx1[11]*Gu1[15];
Gu2[4] = + Gx1[6]*Gu1[1] + Gx1[7]*Gu1[4] + Gx1[8]*Gu1[7] + Gx1[9]*Gu1[10] + Gx1[10]*Gu1[13] + Gx1[11]*Gu1[16];
Gu2[5] = + Gx1[6]*Gu1[2] + Gx1[7]*Gu1[5] + Gx1[8]*Gu1[8] + Gx1[9]*Gu1[11] + Gx1[10]*Gu1[14] + Gx1[11]*Gu1[17];
Gu2[6] = + Gx1[12]*Gu1[0] + Gx1[13]*Gu1[3] + Gx1[14]*Gu1[6] + Gx1[15]*Gu1[9] + Gx1[16]*Gu1[12] + Gx1[17]*Gu1[15];
Gu2[7] = + Gx1[12]*Gu1[1] + Gx1[13]*Gu1[4] + Gx1[14]*Gu1[7] + Gx1[15]*Gu1[10] + Gx1[16]*Gu1[13] + Gx1[17]*Gu1[16];
Gu2[8] = + Gx1[12]*Gu1[2] + Gx1[13]*Gu1[5] + Gx1[14]*Gu1[8] + Gx1[15]*Gu1[11] + Gx1[16]*Gu1[14] + Gx1[17]*Gu1[17];
Gu2[9] = + Gx1[18]*Gu1[0] + Gx1[19]*Gu1[3] + Gx1[20]*Gu1[6] + Gx1[21]*Gu1[9] + Gx1[22]*Gu1[12] + Gx1[23]*Gu1[15];
Gu2[10] = + Gx1[18]*Gu1[1] + Gx1[19]*Gu1[4] + Gx1[20]*Gu1[7] + Gx1[21]*Gu1[10] + Gx1[22]*Gu1[13] + Gx1[23]*Gu1[16];
Gu2[11] = + Gx1[18]*Gu1[2] + Gx1[19]*Gu1[5] + Gx1[20]*Gu1[8] + Gx1[21]*Gu1[11] + Gx1[22]*Gu1[14] + Gx1[23]*Gu1[17];
Gu2[12] = + Gx1[24]*Gu1[0] + Gx1[25]*Gu1[3] + Gx1[26]*Gu1[6] + Gx1[27]*Gu1[9] + Gx1[28]*Gu1[12] + Gx1[29]*Gu1[15];
Gu2[13] = + Gx1[24]*Gu1[1] + Gx1[25]*Gu1[4] + Gx1[26]*Gu1[7] + Gx1[27]*Gu1[10] + Gx1[28]*Gu1[13] + Gx1[29]*Gu1[16];
Gu2[14] = + Gx1[24]*Gu1[2] + Gx1[25]*Gu1[5] + Gx1[26]*Gu1[8] + Gx1[27]*Gu1[11] + Gx1[28]*Gu1[14] + Gx1[29]*Gu1[17];
Gu2[15] = + Gx1[30]*Gu1[0] + Gx1[31]*Gu1[3] + Gx1[32]*Gu1[6] + Gx1[33]*Gu1[9] + Gx1[34]*Gu1[12] + Gx1[35]*Gu1[15];
Gu2[16] = + Gx1[30]*Gu1[1] + Gx1[31]*Gu1[4] + Gx1[32]*Gu1[7] + Gx1[33]*Gu1[10] + Gx1[34]*Gu1[13] + Gx1[35]*Gu1[16];
Gu2[17] = + Gx1[30]*Gu1[2] + Gx1[31]*Gu1[5] + Gx1[32]*Gu1[8] + Gx1[33]*Gu1[11] + Gx1[34]*Gu1[14] + Gx1[35]*Gu1[17];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
Gu2[10] = Gu1[10];
Gu2[11] = Gu1[11];
Gu2[12] = Gu1[12];
Gu2[13] = Gu1[13];
Gu2[14] = Gu1[14];
Gu2[15] = Gu1[15];
Gu2[16] = Gu1[16];
Gu2[17] = Gu1[17];
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 225) + (iCol * 3)] = + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9] + Gu1[12]*Gu2[12] + Gu1[15]*Gu2[15];
acadoWorkspace.H[(iRow * 225) + (iCol * 3 + 1)] = + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10] + Gu1[12]*Gu2[13] + Gu1[15]*Gu2[16];
acadoWorkspace.H[(iRow * 225) + (iCol * 3 + 2)] = + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11] + Gu1[12]*Gu2[14] + Gu1[15]*Gu2[17];
acadoWorkspace.H[(iRow * 225 + 75) + (iCol * 3)] = + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9] + Gu1[13]*Gu2[12] + Gu1[16]*Gu2[15];
acadoWorkspace.H[(iRow * 225 + 75) + (iCol * 3 + 1)] = + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10] + Gu1[13]*Gu2[13] + Gu1[16]*Gu2[16];
acadoWorkspace.H[(iRow * 225 + 75) + (iCol * 3 + 2)] = + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11] + Gu1[13]*Gu2[14] + Gu1[16]*Gu2[17];
acadoWorkspace.H[(iRow * 225 + 150) + (iCol * 3)] = + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9] + Gu1[14]*Gu2[12] + Gu1[17]*Gu2[15];
acadoWorkspace.H[(iRow * 225 + 150) + (iCol * 3 + 1)] = + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10] + Gu1[14]*Gu2[13] + Gu1[17]*Gu2[16];
acadoWorkspace.H[(iRow * 225 + 150) + (iCol * 3 + 2)] = + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11] + Gu1[14]*Gu2[14] + Gu1[17]*Gu2[17];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 228] = + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9] + Gu1[12]*Gu2[12] + Gu1[15]*Gu2[15] + R11[0];
acadoWorkspace.H[iRow * 228 + 1] = + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10] + Gu1[12]*Gu2[13] + Gu1[15]*Gu2[16] + R11[1];
acadoWorkspace.H[iRow * 228 + 2] = + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11] + Gu1[12]*Gu2[14] + Gu1[15]*Gu2[17] + R11[2];
acadoWorkspace.H[iRow * 228 + 75] = + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9] + Gu1[13]*Gu2[12] + Gu1[16]*Gu2[15] + R11[3];
acadoWorkspace.H[iRow * 228 + 76] = + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10] + Gu1[13]*Gu2[13] + Gu1[16]*Gu2[16] + R11[4];
acadoWorkspace.H[iRow * 228 + 77] = + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11] + Gu1[13]*Gu2[14] + Gu1[16]*Gu2[17] + R11[5];
acadoWorkspace.H[iRow * 228 + 150] = + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9] + Gu1[14]*Gu2[12] + Gu1[17]*Gu2[15] + R11[6];
acadoWorkspace.H[iRow * 228 + 151] = + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10] + Gu1[14]*Gu2[13] + Gu1[17]*Gu2[16] + R11[7];
acadoWorkspace.H[iRow * 228 + 152] = + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11] + Gu1[14]*Gu2[14] + Gu1[17]*Gu2[17] + R11[8];
acadoWorkspace.H[iRow * 228] += 1.0000000000000000e-10;
acadoWorkspace.H[iRow * 228 + 76] += 1.0000000000000000e-10;
acadoWorkspace.H[iRow * 228 + 152] += 1.0000000000000000e-10;
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[6]*Gu1[3] + Gx1[12]*Gu1[6] + Gx1[18]*Gu1[9] + Gx1[24]*Gu1[12] + Gx1[30]*Gu1[15];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[6]*Gu1[4] + Gx1[12]*Gu1[7] + Gx1[18]*Gu1[10] + Gx1[24]*Gu1[13] + Gx1[30]*Gu1[16];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[6]*Gu1[5] + Gx1[12]*Gu1[8] + Gx1[18]*Gu1[11] + Gx1[24]*Gu1[14] + Gx1[30]*Gu1[17];
Gu2[3] = + Gx1[1]*Gu1[0] + Gx1[7]*Gu1[3] + Gx1[13]*Gu1[6] + Gx1[19]*Gu1[9] + Gx1[25]*Gu1[12] + Gx1[31]*Gu1[15];
Gu2[4] = + Gx1[1]*Gu1[1] + Gx1[7]*Gu1[4] + Gx1[13]*Gu1[7] + Gx1[19]*Gu1[10] + Gx1[25]*Gu1[13] + Gx1[31]*Gu1[16];
Gu2[5] = + Gx1[1]*Gu1[2] + Gx1[7]*Gu1[5] + Gx1[13]*Gu1[8] + Gx1[19]*Gu1[11] + Gx1[25]*Gu1[14] + Gx1[31]*Gu1[17];
Gu2[6] = + Gx1[2]*Gu1[0] + Gx1[8]*Gu1[3] + Gx1[14]*Gu1[6] + Gx1[20]*Gu1[9] + Gx1[26]*Gu1[12] + Gx1[32]*Gu1[15];
Gu2[7] = + Gx1[2]*Gu1[1] + Gx1[8]*Gu1[4] + Gx1[14]*Gu1[7] + Gx1[20]*Gu1[10] + Gx1[26]*Gu1[13] + Gx1[32]*Gu1[16];
Gu2[8] = + Gx1[2]*Gu1[2] + Gx1[8]*Gu1[5] + Gx1[14]*Gu1[8] + Gx1[20]*Gu1[11] + Gx1[26]*Gu1[14] + Gx1[32]*Gu1[17];
Gu2[9] = + Gx1[3]*Gu1[0] + Gx1[9]*Gu1[3] + Gx1[15]*Gu1[6] + Gx1[21]*Gu1[9] + Gx1[27]*Gu1[12] + Gx1[33]*Gu1[15];
Gu2[10] = + Gx1[3]*Gu1[1] + Gx1[9]*Gu1[4] + Gx1[15]*Gu1[7] + Gx1[21]*Gu1[10] + Gx1[27]*Gu1[13] + Gx1[33]*Gu1[16];
Gu2[11] = + Gx1[3]*Gu1[2] + Gx1[9]*Gu1[5] + Gx1[15]*Gu1[8] + Gx1[21]*Gu1[11] + Gx1[27]*Gu1[14] + Gx1[33]*Gu1[17];
Gu2[12] = + Gx1[4]*Gu1[0] + Gx1[10]*Gu1[3] + Gx1[16]*Gu1[6] + Gx1[22]*Gu1[9] + Gx1[28]*Gu1[12] + Gx1[34]*Gu1[15];
Gu2[13] = + Gx1[4]*Gu1[1] + Gx1[10]*Gu1[4] + Gx1[16]*Gu1[7] + Gx1[22]*Gu1[10] + Gx1[28]*Gu1[13] + Gx1[34]*Gu1[16];
Gu2[14] = + Gx1[4]*Gu1[2] + Gx1[10]*Gu1[5] + Gx1[16]*Gu1[8] + Gx1[22]*Gu1[11] + Gx1[28]*Gu1[14] + Gx1[34]*Gu1[17];
Gu2[15] = + Gx1[5]*Gu1[0] + Gx1[11]*Gu1[3] + Gx1[17]*Gu1[6] + Gx1[23]*Gu1[9] + Gx1[29]*Gu1[12] + Gx1[35]*Gu1[15];
Gu2[16] = + Gx1[5]*Gu1[1] + Gx1[11]*Gu1[4] + Gx1[17]*Gu1[7] + Gx1[23]*Gu1[10] + Gx1[29]*Gu1[13] + Gx1[35]*Gu1[16];
Gu2[17] = + Gx1[5]*Gu1[2] + Gx1[11]*Gu1[5] + Gx1[17]*Gu1[8] + Gx1[23]*Gu1[11] + Gx1[29]*Gu1[14] + Gx1[35]*Gu1[17];
}

void acado_multQEW2( real_t* const Q11, real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + Q11[0]*Gu1[0] + Q11[1]*Gu1[3] + Q11[2]*Gu1[6] + Q11[3]*Gu1[9] + Q11[4]*Gu1[12] + Q11[5]*Gu1[15] + Gu2[0];
Gu3[1] = + Q11[0]*Gu1[1] + Q11[1]*Gu1[4] + Q11[2]*Gu1[7] + Q11[3]*Gu1[10] + Q11[4]*Gu1[13] + Q11[5]*Gu1[16] + Gu2[1];
Gu3[2] = + Q11[0]*Gu1[2] + Q11[1]*Gu1[5] + Q11[2]*Gu1[8] + Q11[3]*Gu1[11] + Q11[4]*Gu1[14] + Q11[5]*Gu1[17] + Gu2[2];
Gu3[3] = + Q11[6]*Gu1[0] + Q11[7]*Gu1[3] + Q11[8]*Gu1[6] + Q11[9]*Gu1[9] + Q11[10]*Gu1[12] + Q11[11]*Gu1[15] + Gu2[3];
Gu3[4] = + Q11[6]*Gu1[1] + Q11[7]*Gu1[4] + Q11[8]*Gu1[7] + Q11[9]*Gu1[10] + Q11[10]*Gu1[13] + Q11[11]*Gu1[16] + Gu2[4];
Gu3[5] = + Q11[6]*Gu1[2] + Q11[7]*Gu1[5] + Q11[8]*Gu1[8] + Q11[9]*Gu1[11] + Q11[10]*Gu1[14] + Q11[11]*Gu1[17] + Gu2[5];
Gu3[6] = + Q11[12]*Gu1[0] + Q11[13]*Gu1[3] + Q11[14]*Gu1[6] + Q11[15]*Gu1[9] + Q11[16]*Gu1[12] + Q11[17]*Gu1[15] + Gu2[6];
Gu3[7] = + Q11[12]*Gu1[1] + Q11[13]*Gu1[4] + Q11[14]*Gu1[7] + Q11[15]*Gu1[10] + Q11[16]*Gu1[13] + Q11[17]*Gu1[16] + Gu2[7];
Gu3[8] = + Q11[12]*Gu1[2] + Q11[13]*Gu1[5] + Q11[14]*Gu1[8] + Q11[15]*Gu1[11] + Q11[16]*Gu1[14] + Q11[17]*Gu1[17] + Gu2[8];
Gu3[9] = + Q11[18]*Gu1[0] + Q11[19]*Gu1[3] + Q11[20]*Gu1[6] + Q11[21]*Gu1[9] + Q11[22]*Gu1[12] + Q11[23]*Gu1[15] + Gu2[9];
Gu3[10] = + Q11[18]*Gu1[1] + Q11[19]*Gu1[4] + Q11[20]*Gu1[7] + Q11[21]*Gu1[10] + Q11[22]*Gu1[13] + Q11[23]*Gu1[16] + Gu2[10];
Gu3[11] = + Q11[18]*Gu1[2] + Q11[19]*Gu1[5] + Q11[20]*Gu1[8] + Q11[21]*Gu1[11] + Q11[22]*Gu1[14] + Q11[23]*Gu1[17] + Gu2[11];
Gu3[12] = + Q11[24]*Gu1[0] + Q11[25]*Gu1[3] + Q11[26]*Gu1[6] + Q11[27]*Gu1[9] + Q11[28]*Gu1[12] + Q11[29]*Gu1[15] + Gu2[12];
Gu3[13] = + Q11[24]*Gu1[1] + Q11[25]*Gu1[4] + Q11[26]*Gu1[7] + Q11[27]*Gu1[10] + Q11[28]*Gu1[13] + Q11[29]*Gu1[16] + Gu2[13];
Gu3[14] = + Q11[24]*Gu1[2] + Q11[25]*Gu1[5] + Q11[26]*Gu1[8] + Q11[27]*Gu1[11] + Q11[28]*Gu1[14] + Q11[29]*Gu1[17] + Gu2[14];
Gu3[15] = + Q11[30]*Gu1[0] + Q11[31]*Gu1[3] + Q11[32]*Gu1[6] + Q11[33]*Gu1[9] + Q11[34]*Gu1[12] + Q11[35]*Gu1[15] + Gu2[15];
Gu3[16] = + Q11[30]*Gu1[1] + Q11[31]*Gu1[4] + Q11[32]*Gu1[7] + Q11[33]*Gu1[10] + Q11[34]*Gu1[13] + Q11[35]*Gu1[16] + Gu2[16];
Gu3[17] = + Q11[30]*Gu1[2] + Q11[31]*Gu1[5] + Q11[32]*Gu1[8] + Q11[33]*Gu1[11] + Q11[34]*Gu1[14] + Q11[35]*Gu1[17] + Gu2[17];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[6]*w11[1] + Gx1[12]*w11[2] + Gx1[18]*w11[3] + Gx1[24]*w11[4] + Gx1[30]*w11[5] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[7]*w11[1] + Gx1[13]*w11[2] + Gx1[19]*w11[3] + Gx1[25]*w11[4] + Gx1[31]*w11[5] + w12[1];
w13[2] = + Gx1[2]*w11[0] + Gx1[8]*w11[1] + Gx1[14]*w11[2] + Gx1[20]*w11[3] + Gx1[26]*w11[4] + Gx1[32]*w11[5] + w12[2];
w13[3] = + Gx1[3]*w11[0] + Gx1[9]*w11[1] + Gx1[15]*w11[2] + Gx1[21]*w11[3] + Gx1[27]*w11[4] + Gx1[33]*w11[5] + w12[3];
w13[4] = + Gx1[4]*w11[0] + Gx1[10]*w11[1] + Gx1[16]*w11[2] + Gx1[22]*w11[3] + Gx1[28]*w11[4] + Gx1[34]*w11[5] + w12[4];
w13[5] = + Gx1[5]*w11[0] + Gx1[11]*w11[1] + Gx1[17]*w11[2] + Gx1[23]*w11[3] + Gx1[29]*w11[4] + Gx1[35]*w11[5] + w12[5];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[3]*w11[1] + Gu1[6]*w11[2] + Gu1[9]*w11[3] + Gu1[12]*w11[4] + Gu1[15]*w11[5];
U1[1] += + Gu1[1]*w11[0] + Gu1[4]*w11[1] + Gu1[7]*w11[2] + Gu1[10]*w11[3] + Gu1[13]*w11[4] + Gu1[16]*w11[5];
U1[2] += + Gu1[2]*w11[0] + Gu1[5]*w11[1] + Gu1[8]*w11[2] + Gu1[11]*w11[3] + Gu1[14]*w11[4] + Gu1[17]*w11[5];
}

void acado_macQSbarW2( real_t* const Q11, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Q11[0]*w11[0] + Q11[1]*w11[1] + Q11[2]*w11[2] + Q11[3]*w11[3] + Q11[4]*w11[4] + Q11[5]*w11[5] + w12[0];
w13[1] = + Q11[6]*w11[0] + Q11[7]*w11[1] + Q11[8]*w11[2] + Q11[9]*w11[3] + Q11[10]*w11[4] + Q11[11]*w11[5] + w12[1];
w13[2] = + Q11[12]*w11[0] + Q11[13]*w11[1] + Q11[14]*w11[2] + Q11[15]*w11[3] + Q11[16]*w11[4] + Q11[17]*w11[5] + w12[2];
w13[3] = + Q11[18]*w11[0] + Q11[19]*w11[1] + Q11[20]*w11[2] + Q11[21]*w11[3] + Q11[22]*w11[4] + Q11[23]*w11[5] + w12[3];
w13[4] = + Q11[24]*w11[0] + Q11[25]*w11[1] + Q11[26]*w11[2] + Q11[27]*w11[3] + Q11[28]*w11[4] + Q11[29]*w11[5] + w12[4];
w13[5] = + Q11[30]*w11[0] + Q11[31]*w11[1] + Q11[32]*w11[2] + Q11[33]*w11[3] + Q11[34]*w11[4] + Q11[35]*w11[5] + w12[5];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5];
w12[1] += + Gx1[6]*w11[0] + Gx1[7]*w11[1] + Gx1[8]*w11[2] + Gx1[9]*w11[3] + Gx1[10]*w11[4] + Gx1[11]*w11[5];
w12[2] += + Gx1[12]*w11[0] + Gx1[13]*w11[1] + Gx1[14]*w11[2] + Gx1[15]*w11[3] + Gx1[16]*w11[4] + Gx1[17]*w11[5];
w12[3] += + Gx1[18]*w11[0] + Gx1[19]*w11[1] + Gx1[20]*w11[2] + Gx1[21]*w11[3] + Gx1[22]*w11[4] + Gx1[23]*w11[5];
w12[4] += + Gx1[24]*w11[0] + Gx1[25]*w11[1] + Gx1[26]*w11[2] + Gx1[27]*w11[3] + Gx1[28]*w11[4] + Gx1[29]*w11[5];
w12[5] += + Gx1[30]*w11[0] + Gx1[31]*w11[1] + Gx1[32]*w11[2] + Gx1[33]*w11[3] + Gx1[34]*w11[4] + Gx1[35]*w11[5];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5];
w12[1] += + Gx1[6]*w11[0] + Gx1[7]*w11[1] + Gx1[8]*w11[2] + Gx1[9]*w11[3] + Gx1[10]*w11[4] + Gx1[11]*w11[5];
w12[2] += + Gx1[12]*w11[0] + Gx1[13]*w11[1] + Gx1[14]*w11[2] + Gx1[15]*w11[3] + Gx1[16]*w11[4] + Gx1[17]*w11[5];
w12[3] += + Gx1[18]*w11[0] + Gx1[19]*w11[1] + Gx1[20]*w11[2] + Gx1[21]*w11[3] + Gx1[22]*w11[4] + Gx1[23]*w11[5];
w12[4] += + Gx1[24]*w11[0] + Gx1[25]*w11[1] + Gx1[26]*w11[2] + Gx1[27]*w11[3] + Gx1[28]*w11[4] + Gx1[29]*w11[5];
w12[5] += + Gx1[30]*w11[0] + Gx1[31]*w11[1] + Gx1[32]*w11[2] + Gx1[33]*w11[3] + Gx1[34]*w11[4] + Gx1[35]*w11[5];
w12[0] += + Gu1[0]*U1[0] + Gu1[1]*U1[1] + Gu1[2]*U1[2];
w12[1] += + Gu1[3]*U1[0] + Gu1[4]*U1[1] + Gu1[5]*U1[2];
w12[2] += + Gu1[6]*U1[0] + Gu1[7]*U1[1] + Gu1[8]*U1[2];
w12[3] += + Gu1[9]*U1[0] + Gu1[10]*U1[1] + Gu1[11]*U1[2];
w12[4] += + Gu1[12]*U1[0] + Gu1[13]*U1[1] + Gu1[14]*U1[2];
w12[5] += + Gu1[15]*U1[0] + Gu1[16]*U1[1] + Gu1[17]*U1[2];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 225) + (iCol * 3)] = acadoWorkspace.H[(iCol * 225) + (iRow * 3)];
acadoWorkspace.H[(iRow * 225) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 225 + 75) + (iRow * 3)];
acadoWorkspace.H[(iRow * 225) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 225 + 150) + (iRow * 3)];
acadoWorkspace.H[(iRow * 225 + 75) + (iCol * 3)] = acadoWorkspace.H[(iCol * 225) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 225 + 75) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 225 + 75) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 225 + 75) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 225 + 150) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 225 + 150) + (iCol * 3)] = acadoWorkspace.H[(iCol * 225) + (iRow * 3 + 2)];
acadoWorkspace.H[(iRow * 225 + 150) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 225 + 75) + (iRow * 3 + 2)];
acadoWorkspace.H[(iRow * 225 + 150) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 225 + 150) + (iRow * 3 + 2)];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6] + R2[7]*Dy1[7] + R2[8]*Dy1[8] + R2[9]*Dy1[9];
RDy1[1] = + R2[10]*Dy1[0] + R2[11]*Dy1[1] + R2[12]*Dy1[2] + R2[13]*Dy1[3] + R2[14]*Dy1[4] + R2[15]*Dy1[5] + R2[16]*Dy1[6] + R2[17]*Dy1[7] + R2[18]*Dy1[8] + R2[19]*Dy1[9];
RDy1[2] = + R2[20]*Dy1[0] + R2[21]*Dy1[1] + R2[22]*Dy1[2] + R2[23]*Dy1[3] + R2[24]*Dy1[4] + R2[25]*Dy1[5] + R2[26]*Dy1[6] + R2[27]*Dy1[7] + R2[28]*Dy1[8] + R2[29]*Dy1[9];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6] + Q2[7]*Dy1[7] + Q2[8]*Dy1[8] + Q2[9]*Dy1[9];
QDy1[1] = + Q2[10]*Dy1[0] + Q2[11]*Dy1[1] + Q2[12]*Dy1[2] + Q2[13]*Dy1[3] + Q2[14]*Dy1[4] + Q2[15]*Dy1[5] + Q2[16]*Dy1[6] + Q2[17]*Dy1[7] + Q2[18]*Dy1[8] + Q2[19]*Dy1[9];
QDy1[2] = + Q2[20]*Dy1[0] + Q2[21]*Dy1[1] + Q2[22]*Dy1[2] + Q2[23]*Dy1[3] + Q2[24]*Dy1[4] + Q2[25]*Dy1[5] + Q2[26]*Dy1[6] + Q2[27]*Dy1[7] + Q2[28]*Dy1[8] + Q2[29]*Dy1[9];
QDy1[3] = + Q2[30]*Dy1[0] + Q2[31]*Dy1[1] + Q2[32]*Dy1[2] + Q2[33]*Dy1[3] + Q2[34]*Dy1[4] + Q2[35]*Dy1[5] + Q2[36]*Dy1[6] + Q2[37]*Dy1[7] + Q2[38]*Dy1[8] + Q2[39]*Dy1[9];
QDy1[4] = + Q2[40]*Dy1[0] + Q2[41]*Dy1[1] + Q2[42]*Dy1[2] + Q2[43]*Dy1[3] + Q2[44]*Dy1[4] + Q2[45]*Dy1[5] + Q2[46]*Dy1[6] + Q2[47]*Dy1[7] + Q2[48]*Dy1[8] + Q2[49]*Dy1[9];
QDy1[5] = + Q2[50]*Dy1[0] + Q2[51]*Dy1[1] + Q2[52]*Dy1[2] + Q2[53]*Dy1[3] + Q2[54]*Dy1[4] + Q2[55]*Dy1[5] + Q2[56]*Dy1[6] + Q2[57]*Dy1[7] + Q2[58]*Dy1[8] + Q2[59]*Dy1[9];
}

void acado_multHxE( real_t* const Hx, real_t* const E, int row, int col )
{
acadoWorkspace.A[(row * 225 + 5625) + (col * 3)] = + Hx[0]*E[0] + Hx[1]*E[3] + Hx[2]*E[6] + Hx[3]*E[9] + Hx[4]*E[12] + Hx[5]*E[15];
acadoWorkspace.A[(row * 225 + 5625) + (col * 3 + 1)] = + Hx[0]*E[1] + Hx[1]*E[4] + Hx[2]*E[7] + Hx[3]*E[10] + Hx[4]*E[13] + Hx[5]*E[16];
acadoWorkspace.A[(row * 225 + 5625) + (col * 3 + 2)] = + Hx[0]*E[2] + Hx[1]*E[5] + Hx[2]*E[8] + Hx[3]*E[11] + Hx[4]*E[14] + Hx[5]*E[17];
acadoWorkspace.A[(row * 225 + 5700) + (col * 3)] = + Hx[6]*E[0] + Hx[7]*E[3] + Hx[8]*E[6] + Hx[9]*E[9] + Hx[10]*E[12] + Hx[11]*E[15];
acadoWorkspace.A[(row * 225 + 5700) + (col * 3 + 1)] = + Hx[6]*E[1] + Hx[7]*E[4] + Hx[8]*E[7] + Hx[9]*E[10] + Hx[10]*E[13] + Hx[11]*E[16];
acadoWorkspace.A[(row * 225 + 5700) + (col * 3 + 2)] = + Hx[6]*E[2] + Hx[7]*E[5] + Hx[8]*E[8] + Hx[9]*E[11] + Hx[10]*E[14] + Hx[11]*E[17];
acadoWorkspace.A[(row * 225 + 5775) + (col * 3)] = + Hx[12]*E[0] + Hx[13]*E[3] + Hx[14]*E[6] + Hx[15]*E[9] + Hx[16]*E[12] + Hx[17]*E[15];
acadoWorkspace.A[(row * 225 + 5775) + (col * 3 + 1)] = + Hx[12]*E[1] + Hx[13]*E[4] + Hx[14]*E[7] + Hx[15]*E[10] + Hx[16]*E[13] + Hx[17]*E[16];
acadoWorkspace.A[(row * 225 + 5775) + (col * 3 + 2)] = + Hx[12]*E[2] + Hx[13]*E[5] + Hx[14]*E[8] + Hx[15]*E[11] + Hx[16]*E[14] + Hx[17]*E[17];
}

void acado_macHxd( real_t* const Hx, real_t* const tmpd, real_t* const lbA, real_t* const ubA )
{
acadoWorkspace.evHxd[0] = + Hx[0]*tmpd[0] + Hx[1]*tmpd[1] + Hx[2]*tmpd[2] + Hx[3]*tmpd[3] + Hx[4]*tmpd[4] + Hx[5]*tmpd[5];
acadoWorkspace.evHxd[1] = + Hx[6]*tmpd[0] + Hx[7]*tmpd[1] + Hx[8]*tmpd[2] + Hx[9]*tmpd[3] + Hx[10]*tmpd[4] + Hx[11]*tmpd[5];
acadoWorkspace.evHxd[2] = + Hx[12]*tmpd[0] + Hx[13]*tmpd[1] + Hx[14]*tmpd[2] + Hx[15]*tmpd[3] + Hx[16]*tmpd[4] + Hx[17]*tmpd[5];
lbA[0] -= acadoWorkspace.evHxd[0];
lbA[1] -= acadoWorkspace.evHxd[1];
lbA[2] -= acadoWorkspace.evHxd[2];
ubA[0] -= acadoWorkspace.evHxd[0];
ubA[1] -= acadoWorkspace.evHxd[1];
ubA[2] -= acadoWorkspace.evHxd[2];
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
/** Row vector of size: 75 */
static const int xBoundIndices[ 75 ] = 
{ 9, 10, 11, 15, 16, 17, 21, 22, 23, 27, 28, 29, 33, 34, 35, 39, 40, 41, 45, 46, 47, 51, 52, 53, 57, 58, 59, 63, 64, 65, 69, 70, 71, 75, 76, 77, 81, 82, 83, 87, 88, 89, 93, 94, 95, 99, 100, 101, 105, 106, 107, 111, 112, 113, 117, 118, 119, 123, 124, 125, 129, 130, 131, 135, 136, 137, 141, 142, 143, 147, 148, 149, 153, 154, 155 };
acado_moveGxT( acadoWorkspace.evGx, acadoWorkspace.C );
acado_multGxGx( &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.C, &(acadoWorkspace.C[ 36 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.C[ 36 ]), &(acadoWorkspace.C[ 72 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.C[ 72 ]), &(acadoWorkspace.C[ 108 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.C[ 108 ]), &(acadoWorkspace.C[ 144 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.C[ 144 ]), &(acadoWorkspace.C[ 180 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.C[ 180 ]), &(acadoWorkspace.C[ 216 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.C[ 216 ]), &(acadoWorkspace.C[ 252 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.C[ 252 ]), &(acadoWorkspace.C[ 288 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.C[ 288 ]), &(acadoWorkspace.C[ 324 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 360 ]), &(acadoWorkspace.C[ 324 ]), &(acadoWorkspace.C[ 360 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 396 ]), &(acadoWorkspace.C[ 360 ]), &(acadoWorkspace.C[ 396 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.C[ 396 ]), &(acadoWorkspace.C[ 432 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 468 ]), &(acadoWorkspace.C[ 432 ]), &(acadoWorkspace.C[ 468 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 504 ]), &(acadoWorkspace.C[ 468 ]), &(acadoWorkspace.C[ 504 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 540 ]), &(acadoWorkspace.C[ 504 ]), &(acadoWorkspace.C[ 540 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.C[ 540 ]), &(acadoWorkspace.C[ 576 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 612 ]), &(acadoWorkspace.C[ 576 ]), &(acadoWorkspace.C[ 612 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.C[ 612 ]), &(acadoWorkspace.C[ 648 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 684 ]), &(acadoWorkspace.C[ 648 ]), &(acadoWorkspace.C[ 684 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 720 ]), &(acadoWorkspace.C[ 684 ]), &(acadoWorkspace.C[ 720 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 756 ]), &(acadoWorkspace.C[ 720 ]), &(acadoWorkspace.C[ 756 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 792 ]), &(acadoWorkspace.C[ 756 ]), &(acadoWorkspace.C[ 792 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 828 ]), &(acadoWorkspace.C[ 792 ]), &(acadoWorkspace.C[ 828 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 864 ]), &(acadoWorkspace.C[ 828 ]), &(acadoWorkspace.C[ 864 ]) );
for (lRun2 = 0; lRun2 < 25; ++lRun2)
{
lRun3 = ((lRun2) * (lRun2 * -1 + 51)) / (2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun2 * 18 ]), &(acadoWorkspace.E[ lRun3 * 18 ]) );
for (lRun1 = 1; lRun1 < lRun2 * -1 + 25; ++lRun1)
{
acado_multGxGu( &(acadoWorkspace.evGx[ ((((lRun2) + (lRun1)) * (6)) * (6)) + (0) ]), &(acadoWorkspace.E[ (((((lRun3) + (lRun1)) - (1)) * (6)) * (3)) + (0) ]), &(acadoWorkspace.E[ ((((lRun3) + (lRun1)) * (6)) * (3)) + (0) ]) );
}

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ ((((((lRun3) - (lRun2)) + (25)) - (1)) * (6)) * (3)) + (0) ]), acadoWorkspace.W1 );
for (lRun1 = 24; lRun2 < lRun1; --lRun1)
{
acado_multBTW1( &(acadoWorkspace.evGu[ lRun1 * 18 ]), acadoWorkspace.W1, lRun1, lRun2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ lRun1 * 36 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ lRun1 * 36 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (6)) * (3)) + (0) ]), acadoWorkspace.W2, acadoWorkspace.W1 );
}
acado_multBTW1_R1( &(acadoWorkspace.R1[ lRun2 * 9 ]), &(acadoWorkspace.evGu[ lRun2 * 18 ]), acadoWorkspace.W1, lRun2 );
}

for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
acado_copyHTH( lRun2, lRun1 );
}
}

for (lRun2 = 0; lRun2 < 150; ++lRun2)
acadoWorkspace.sbar[lRun2 + 6] = acadoWorkspace.d[lRun2];

acadoWorkspace.lb[0] = (real_t)-6.0000000000000000e+01 - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[1];
acadoWorkspace.lb[2] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[2];
acadoWorkspace.lb[3] = (real_t)-6.0000000000000000e+01 - acadoVariables.u[3];
acadoWorkspace.lb[4] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)-6.0000000000000000e+01 - acadoVariables.u[6];
acadoWorkspace.lb[7] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[7];
acadoWorkspace.lb[8] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[8];
acadoWorkspace.lb[9] = (real_t)-6.0000000000000000e+01 - acadoVariables.u[9];
acadoWorkspace.lb[10] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[10];
acadoWorkspace.lb[11] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[11];
acadoWorkspace.lb[12] = (real_t)-6.0000000000000000e+01 - acadoVariables.u[12];
acadoWorkspace.lb[13] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[13];
acadoWorkspace.lb[14] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[14];
acadoWorkspace.lb[15] = (real_t)-6.0000000000000000e+01 - acadoVariables.u[15];
acadoWorkspace.lb[16] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[16];
acadoWorkspace.lb[17] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[17];
acadoWorkspace.lb[18] = (real_t)-6.0000000000000000e+01 - acadoVariables.u[18];
acadoWorkspace.lb[19] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[19];
acadoWorkspace.lb[20] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[20];
acadoWorkspace.lb[21] = (real_t)-6.0000000000000000e+01 - acadoVariables.u[21];
acadoWorkspace.lb[22] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[22];
acadoWorkspace.lb[23] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[23];
acadoWorkspace.lb[24] = (real_t)-6.0000000000000000e+01 - acadoVariables.u[24];
acadoWorkspace.lb[25] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[25];
acadoWorkspace.lb[26] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[26];
acadoWorkspace.lb[27] = (real_t)-6.0000000000000000e+01 - acadoVariables.u[27];
acadoWorkspace.lb[28] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[28];
acadoWorkspace.lb[29] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[29];
acadoWorkspace.lb[30] = (real_t)-6.0000000000000000e+01 - acadoVariables.u[30];
acadoWorkspace.lb[31] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[31];
acadoWorkspace.lb[32] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[32];
acadoWorkspace.lb[33] = (real_t)-6.0000000000000000e+01 - acadoVariables.u[33];
acadoWorkspace.lb[34] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[34];
acadoWorkspace.lb[35] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[35];
acadoWorkspace.lb[36] = (real_t)-6.0000000000000000e+01 - acadoVariables.u[36];
acadoWorkspace.lb[37] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[37];
acadoWorkspace.lb[38] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[38];
acadoWorkspace.lb[39] = (real_t)-6.0000000000000000e+01 - acadoVariables.u[39];
acadoWorkspace.lb[40] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[40];
acadoWorkspace.lb[41] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[41];
acadoWorkspace.lb[42] = (real_t)-6.0000000000000000e+01 - acadoVariables.u[42];
acadoWorkspace.lb[43] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[43];
acadoWorkspace.lb[44] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[44];
acadoWorkspace.lb[45] = (real_t)-6.0000000000000000e+01 - acadoVariables.u[45];
acadoWorkspace.lb[46] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[46];
acadoWorkspace.lb[47] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[47];
acadoWorkspace.lb[48] = (real_t)-6.0000000000000000e+01 - acadoVariables.u[48];
acadoWorkspace.lb[49] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[49];
acadoWorkspace.lb[50] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[50];
acadoWorkspace.lb[51] = (real_t)-6.0000000000000000e+01 - acadoVariables.u[51];
acadoWorkspace.lb[52] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[52];
acadoWorkspace.lb[53] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[53];
acadoWorkspace.lb[54] = (real_t)-6.0000000000000000e+01 - acadoVariables.u[54];
acadoWorkspace.lb[55] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[55];
acadoWorkspace.lb[56] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[56];
acadoWorkspace.lb[57] = (real_t)-6.0000000000000000e+01 - acadoVariables.u[57];
acadoWorkspace.lb[58] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[58];
acadoWorkspace.lb[59] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[59];
acadoWorkspace.lb[60] = (real_t)-6.0000000000000000e+01 - acadoVariables.u[60];
acadoWorkspace.lb[61] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[61];
acadoWorkspace.lb[62] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[62];
acadoWorkspace.lb[63] = (real_t)-6.0000000000000000e+01 - acadoVariables.u[63];
acadoWorkspace.lb[64] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[64];
acadoWorkspace.lb[65] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[65];
acadoWorkspace.lb[66] = (real_t)-6.0000000000000000e+01 - acadoVariables.u[66];
acadoWorkspace.lb[67] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[67];
acadoWorkspace.lb[68] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[68];
acadoWorkspace.lb[69] = (real_t)-6.0000000000000000e+01 - acadoVariables.u[69];
acadoWorkspace.lb[70] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[70];
acadoWorkspace.lb[71] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[71];
acadoWorkspace.lb[72] = (real_t)-6.0000000000000000e+01 - acadoVariables.u[72];
acadoWorkspace.lb[73] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[73];
acadoWorkspace.lb[74] = (real_t)-3.9000000000000000e+01 - acadoVariables.u[74];
acadoWorkspace.ub[0] = (real_t)6.0000000000000000e+01 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)3.9000000000000000e+01 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)3.9000000000000000e+01 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)6.0000000000000000e+01 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)3.9000000000000000e+01 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)3.9000000000000000e+01 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)6.0000000000000000e+01 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)3.9000000000000000e+01 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)3.9000000000000000e+01 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)6.0000000000000000e+01 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)3.9000000000000000e+01 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)3.9000000000000000e+01 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)6.0000000000000000e+01 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)3.9000000000000000e+01 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)3.9000000000000000e+01 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)6.0000000000000000e+01 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)3.9000000000000000e+01 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)3.9000000000000000e+01 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)6.0000000000000000e+01 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)3.9000000000000000e+01 - acadoVariables.u[19];
acadoWorkspace.ub[20] = (real_t)3.9000000000000000e+01 - acadoVariables.u[20];
acadoWorkspace.ub[21] = (real_t)6.0000000000000000e+01 - acadoVariables.u[21];
acadoWorkspace.ub[22] = (real_t)3.9000000000000000e+01 - acadoVariables.u[22];
acadoWorkspace.ub[23] = (real_t)3.9000000000000000e+01 - acadoVariables.u[23];
acadoWorkspace.ub[24] = (real_t)6.0000000000000000e+01 - acadoVariables.u[24];
acadoWorkspace.ub[25] = (real_t)3.9000000000000000e+01 - acadoVariables.u[25];
acadoWorkspace.ub[26] = (real_t)3.9000000000000000e+01 - acadoVariables.u[26];
acadoWorkspace.ub[27] = (real_t)6.0000000000000000e+01 - acadoVariables.u[27];
acadoWorkspace.ub[28] = (real_t)3.9000000000000000e+01 - acadoVariables.u[28];
acadoWorkspace.ub[29] = (real_t)3.9000000000000000e+01 - acadoVariables.u[29];
acadoWorkspace.ub[30] = (real_t)6.0000000000000000e+01 - acadoVariables.u[30];
acadoWorkspace.ub[31] = (real_t)3.9000000000000000e+01 - acadoVariables.u[31];
acadoWorkspace.ub[32] = (real_t)3.9000000000000000e+01 - acadoVariables.u[32];
acadoWorkspace.ub[33] = (real_t)6.0000000000000000e+01 - acadoVariables.u[33];
acadoWorkspace.ub[34] = (real_t)3.9000000000000000e+01 - acadoVariables.u[34];
acadoWorkspace.ub[35] = (real_t)3.9000000000000000e+01 - acadoVariables.u[35];
acadoWorkspace.ub[36] = (real_t)6.0000000000000000e+01 - acadoVariables.u[36];
acadoWorkspace.ub[37] = (real_t)3.9000000000000000e+01 - acadoVariables.u[37];
acadoWorkspace.ub[38] = (real_t)3.9000000000000000e+01 - acadoVariables.u[38];
acadoWorkspace.ub[39] = (real_t)6.0000000000000000e+01 - acadoVariables.u[39];
acadoWorkspace.ub[40] = (real_t)3.9000000000000000e+01 - acadoVariables.u[40];
acadoWorkspace.ub[41] = (real_t)3.9000000000000000e+01 - acadoVariables.u[41];
acadoWorkspace.ub[42] = (real_t)6.0000000000000000e+01 - acadoVariables.u[42];
acadoWorkspace.ub[43] = (real_t)3.9000000000000000e+01 - acadoVariables.u[43];
acadoWorkspace.ub[44] = (real_t)3.9000000000000000e+01 - acadoVariables.u[44];
acadoWorkspace.ub[45] = (real_t)6.0000000000000000e+01 - acadoVariables.u[45];
acadoWorkspace.ub[46] = (real_t)3.9000000000000000e+01 - acadoVariables.u[46];
acadoWorkspace.ub[47] = (real_t)3.9000000000000000e+01 - acadoVariables.u[47];
acadoWorkspace.ub[48] = (real_t)6.0000000000000000e+01 - acadoVariables.u[48];
acadoWorkspace.ub[49] = (real_t)3.9000000000000000e+01 - acadoVariables.u[49];
acadoWorkspace.ub[50] = (real_t)3.9000000000000000e+01 - acadoVariables.u[50];
acadoWorkspace.ub[51] = (real_t)6.0000000000000000e+01 - acadoVariables.u[51];
acadoWorkspace.ub[52] = (real_t)3.9000000000000000e+01 - acadoVariables.u[52];
acadoWorkspace.ub[53] = (real_t)3.9000000000000000e+01 - acadoVariables.u[53];
acadoWorkspace.ub[54] = (real_t)6.0000000000000000e+01 - acadoVariables.u[54];
acadoWorkspace.ub[55] = (real_t)3.9000000000000000e+01 - acadoVariables.u[55];
acadoWorkspace.ub[56] = (real_t)3.9000000000000000e+01 - acadoVariables.u[56];
acadoWorkspace.ub[57] = (real_t)6.0000000000000000e+01 - acadoVariables.u[57];
acadoWorkspace.ub[58] = (real_t)3.9000000000000000e+01 - acadoVariables.u[58];
acadoWorkspace.ub[59] = (real_t)3.9000000000000000e+01 - acadoVariables.u[59];
acadoWorkspace.ub[60] = (real_t)6.0000000000000000e+01 - acadoVariables.u[60];
acadoWorkspace.ub[61] = (real_t)3.9000000000000000e+01 - acadoVariables.u[61];
acadoWorkspace.ub[62] = (real_t)3.9000000000000000e+01 - acadoVariables.u[62];
acadoWorkspace.ub[63] = (real_t)6.0000000000000000e+01 - acadoVariables.u[63];
acadoWorkspace.ub[64] = (real_t)3.9000000000000000e+01 - acadoVariables.u[64];
acadoWorkspace.ub[65] = (real_t)3.9000000000000000e+01 - acadoVariables.u[65];
acadoWorkspace.ub[66] = (real_t)6.0000000000000000e+01 - acadoVariables.u[66];
acadoWorkspace.ub[67] = (real_t)3.9000000000000000e+01 - acadoVariables.u[67];
acadoWorkspace.ub[68] = (real_t)3.9000000000000000e+01 - acadoVariables.u[68];
acadoWorkspace.ub[69] = (real_t)6.0000000000000000e+01 - acadoVariables.u[69];
acadoWorkspace.ub[70] = (real_t)3.9000000000000000e+01 - acadoVariables.u[70];
acadoWorkspace.ub[71] = (real_t)3.9000000000000000e+01 - acadoVariables.u[71];
acadoWorkspace.ub[72] = (real_t)6.0000000000000000e+01 - acadoVariables.u[72];
acadoWorkspace.ub[73] = (real_t)3.9000000000000000e+01 - acadoVariables.u[73];
acadoWorkspace.ub[74] = (real_t)3.9000000000000000e+01 - acadoVariables.u[74];

for (lRun1 = 0; lRun1 < 75; ++lRun1)
{
lRun3 = xBoundIndices[ lRun1 ] - 6;
lRun4 = ((lRun3) / (6)) + (1);
for (lRun2 = 0; lRun2 < lRun4; ++lRun2)
{
lRun5 = ((((((lRun2) * (lRun2 * -1 + 49)) / (2)) + (lRun4)) - (1)) * (6)) + ((lRun3) % (6));
acadoWorkspace.A[(lRun1 * 75) + (lRun2 * 3)] = acadoWorkspace.E[lRun5 * 3];
acadoWorkspace.A[(lRun1 * 75) + (lRun2 * 3 + 1)] = acadoWorkspace.E[lRun5 * 3 + 1];
acadoWorkspace.A[(lRun1 * 75) + (lRun2 * 3 + 2)] = acadoWorkspace.E[lRun5 * 3 + 2];
}
}

for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
acadoWorkspace.conValueIn[0] = acadoVariables.x[lRun1 * 6];
acadoWorkspace.conValueIn[1] = acadoVariables.x[lRun1 * 6 + 1];
acadoWorkspace.conValueIn[2] = acadoVariables.x[lRun1 * 6 + 2];
acadoWorkspace.conValueIn[3] = acadoVariables.x[lRun1 * 6 + 3];
acadoWorkspace.conValueIn[4] = acadoVariables.x[lRun1 * 6 + 4];
acadoWorkspace.conValueIn[5] = acadoVariables.x[lRun1 * 6 + 5];
acadoWorkspace.conValueIn[6] = acadoVariables.u[lRun1 * 3];
acadoWorkspace.conValueIn[7] = acadoVariables.u[lRun1 * 3 + 1];
acadoWorkspace.conValueIn[8] = acadoVariables.u[lRun1 * 3 + 2];
acadoWorkspace.conValueIn[9] = acadoVariables.od[lRun1 * 3];
acadoWorkspace.conValueIn[10] = acadoVariables.od[lRun1 * 3 + 1];
acadoWorkspace.conValueIn[11] = acadoVariables.od[lRun1 * 3 + 2];
acado_evaluatePathConstraints( acadoWorkspace.conValueIn, acadoWorkspace.conValueOut );
acadoWorkspace.evH[lRun1 * 3] = acadoWorkspace.conValueOut[0];
acadoWorkspace.evH[lRun1 * 3 + 1] = acadoWorkspace.conValueOut[1];
acadoWorkspace.evH[lRun1 * 3 + 2] = acadoWorkspace.conValueOut[2];

acadoWorkspace.evHx[lRun1 * 18] = acadoWorkspace.conValueOut[3];
acadoWorkspace.evHx[lRun1 * 18 + 1] = acadoWorkspace.conValueOut[4];
acadoWorkspace.evHx[lRun1 * 18 + 2] = acadoWorkspace.conValueOut[5];
acadoWorkspace.evHx[lRun1 * 18 + 3] = acadoWorkspace.conValueOut[6];
acadoWorkspace.evHx[lRun1 * 18 + 4] = acadoWorkspace.conValueOut[7];
acadoWorkspace.evHx[lRun1 * 18 + 5] = acadoWorkspace.conValueOut[8];
acadoWorkspace.evHx[lRun1 * 18 + 6] = acadoWorkspace.conValueOut[9];
acadoWorkspace.evHx[lRun1 * 18 + 7] = acadoWorkspace.conValueOut[10];
acadoWorkspace.evHx[lRun1 * 18 + 8] = acadoWorkspace.conValueOut[11];
acadoWorkspace.evHx[lRun1 * 18 + 9] = acadoWorkspace.conValueOut[12];
acadoWorkspace.evHx[lRun1 * 18 + 10] = acadoWorkspace.conValueOut[13];
acadoWorkspace.evHx[lRun1 * 18 + 11] = acadoWorkspace.conValueOut[14];
acadoWorkspace.evHx[lRun1 * 18 + 12] = acadoWorkspace.conValueOut[15];
acadoWorkspace.evHx[lRun1 * 18 + 13] = acadoWorkspace.conValueOut[16];
acadoWorkspace.evHx[lRun1 * 18 + 14] = acadoWorkspace.conValueOut[17];
acadoWorkspace.evHx[lRun1 * 18 + 15] = acadoWorkspace.conValueOut[18];
acadoWorkspace.evHx[lRun1 * 18 + 16] = acadoWorkspace.conValueOut[19];
acadoWorkspace.evHx[lRun1 * 18 + 17] = acadoWorkspace.conValueOut[20];
acadoWorkspace.evHu[lRun1 * 9] = acadoWorkspace.conValueOut[21];
acadoWorkspace.evHu[lRun1 * 9 + 1] = acadoWorkspace.conValueOut[22];
acadoWorkspace.evHu[lRun1 * 9 + 2] = acadoWorkspace.conValueOut[23];
acadoWorkspace.evHu[lRun1 * 9 + 3] = acadoWorkspace.conValueOut[24];
acadoWorkspace.evHu[lRun1 * 9 + 4] = acadoWorkspace.conValueOut[25];
acadoWorkspace.evHu[lRun1 * 9 + 5] = acadoWorkspace.conValueOut[26];
acadoWorkspace.evHu[lRun1 * 9 + 6] = acadoWorkspace.conValueOut[27];
acadoWorkspace.evHu[lRun1 * 9 + 7] = acadoWorkspace.conValueOut[28];
acadoWorkspace.evHu[lRun1 * 9 + 8] = acadoWorkspace.conValueOut[29];
}



for (lRun2 = 0; lRun2 < 24; ++lRun2)
{
for (lRun3 = 0; lRun3 < lRun2 + 1; ++lRun3)
{
lRun4 = (((lRun3) * (lRun3 * -1 + 49)) / (2)) + (lRun2);
lRun5 = lRun2 + 1;
acado_multHxE( &(acadoWorkspace.evHx[ lRun2 * 18 + 18 ]), &(acadoWorkspace.E[ lRun4 * 18 ]), lRun5, lRun3 );
}
}

acadoWorkspace.A[5625] = acadoWorkspace.evHu[0];
acadoWorkspace.A[5626] = acadoWorkspace.evHu[1];
acadoWorkspace.A[5627] = acadoWorkspace.evHu[2];
acadoWorkspace.A[5700] = acadoWorkspace.evHu[3];
acadoWorkspace.A[5701] = acadoWorkspace.evHu[4];
acadoWorkspace.A[5702] = acadoWorkspace.evHu[5];
acadoWorkspace.A[5775] = acadoWorkspace.evHu[6];
acadoWorkspace.A[5776] = acadoWorkspace.evHu[7];
acadoWorkspace.A[5777] = acadoWorkspace.evHu[8];
acadoWorkspace.A[5853] = acadoWorkspace.evHu[9];
acadoWorkspace.A[5854] = acadoWorkspace.evHu[10];
acadoWorkspace.A[5855] = acadoWorkspace.evHu[11];
acadoWorkspace.A[5928] = acadoWorkspace.evHu[12];
acadoWorkspace.A[5929] = acadoWorkspace.evHu[13];
acadoWorkspace.A[5930] = acadoWorkspace.evHu[14];
acadoWorkspace.A[6003] = acadoWorkspace.evHu[15];
acadoWorkspace.A[6004] = acadoWorkspace.evHu[16];
acadoWorkspace.A[6005] = acadoWorkspace.evHu[17];
acadoWorkspace.A[6081] = acadoWorkspace.evHu[18];
acadoWorkspace.A[6082] = acadoWorkspace.evHu[19];
acadoWorkspace.A[6083] = acadoWorkspace.evHu[20];
acadoWorkspace.A[6156] = acadoWorkspace.evHu[21];
acadoWorkspace.A[6157] = acadoWorkspace.evHu[22];
acadoWorkspace.A[6158] = acadoWorkspace.evHu[23];
acadoWorkspace.A[6231] = acadoWorkspace.evHu[24];
acadoWorkspace.A[6232] = acadoWorkspace.evHu[25];
acadoWorkspace.A[6233] = acadoWorkspace.evHu[26];
acadoWorkspace.A[6309] = acadoWorkspace.evHu[27];
acadoWorkspace.A[6310] = acadoWorkspace.evHu[28];
acadoWorkspace.A[6311] = acadoWorkspace.evHu[29];
acadoWorkspace.A[6384] = acadoWorkspace.evHu[30];
acadoWorkspace.A[6385] = acadoWorkspace.evHu[31];
acadoWorkspace.A[6386] = acadoWorkspace.evHu[32];
acadoWorkspace.A[6459] = acadoWorkspace.evHu[33];
acadoWorkspace.A[6460] = acadoWorkspace.evHu[34];
acadoWorkspace.A[6461] = acadoWorkspace.evHu[35];
acadoWorkspace.A[6537] = acadoWorkspace.evHu[36];
acadoWorkspace.A[6538] = acadoWorkspace.evHu[37];
acadoWorkspace.A[6539] = acadoWorkspace.evHu[38];
acadoWorkspace.A[6612] = acadoWorkspace.evHu[39];
acadoWorkspace.A[6613] = acadoWorkspace.evHu[40];
acadoWorkspace.A[6614] = acadoWorkspace.evHu[41];
acadoWorkspace.A[6687] = acadoWorkspace.evHu[42];
acadoWorkspace.A[6688] = acadoWorkspace.evHu[43];
acadoWorkspace.A[6689] = acadoWorkspace.evHu[44];
acadoWorkspace.A[6765] = acadoWorkspace.evHu[45];
acadoWorkspace.A[6766] = acadoWorkspace.evHu[46];
acadoWorkspace.A[6767] = acadoWorkspace.evHu[47];
acadoWorkspace.A[6840] = acadoWorkspace.evHu[48];
acadoWorkspace.A[6841] = acadoWorkspace.evHu[49];
acadoWorkspace.A[6842] = acadoWorkspace.evHu[50];
acadoWorkspace.A[6915] = acadoWorkspace.evHu[51];
acadoWorkspace.A[6916] = acadoWorkspace.evHu[52];
acadoWorkspace.A[6917] = acadoWorkspace.evHu[53];
acadoWorkspace.A[6993] = acadoWorkspace.evHu[54];
acadoWorkspace.A[6994] = acadoWorkspace.evHu[55];
acadoWorkspace.A[6995] = acadoWorkspace.evHu[56];
acadoWorkspace.A[7068] = acadoWorkspace.evHu[57];
acadoWorkspace.A[7069] = acadoWorkspace.evHu[58];
acadoWorkspace.A[7070] = acadoWorkspace.evHu[59];
acadoWorkspace.A[7143] = acadoWorkspace.evHu[60];
acadoWorkspace.A[7144] = acadoWorkspace.evHu[61];
acadoWorkspace.A[7145] = acadoWorkspace.evHu[62];
acadoWorkspace.A[7221] = acadoWorkspace.evHu[63];
acadoWorkspace.A[7222] = acadoWorkspace.evHu[64];
acadoWorkspace.A[7223] = acadoWorkspace.evHu[65];
acadoWorkspace.A[7296] = acadoWorkspace.evHu[66];
acadoWorkspace.A[7297] = acadoWorkspace.evHu[67];
acadoWorkspace.A[7298] = acadoWorkspace.evHu[68];
acadoWorkspace.A[7371] = acadoWorkspace.evHu[69];
acadoWorkspace.A[7372] = acadoWorkspace.evHu[70];
acadoWorkspace.A[7373] = acadoWorkspace.evHu[71];
acadoWorkspace.A[7449] = acadoWorkspace.evHu[72];
acadoWorkspace.A[7450] = acadoWorkspace.evHu[73];
acadoWorkspace.A[7451] = acadoWorkspace.evHu[74];
acadoWorkspace.A[7524] = acadoWorkspace.evHu[75];
acadoWorkspace.A[7525] = acadoWorkspace.evHu[76];
acadoWorkspace.A[7526] = acadoWorkspace.evHu[77];
acadoWorkspace.A[7599] = acadoWorkspace.evHu[78];
acadoWorkspace.A[7600] = acadoWorkspace.evHu[79];
acadoWorkspace.A[7601] = acadoWorkspace.evHu[80];
acadoWorkspace.A[7677] = acadoWorkspace.evHu[81];
acadoWorkspace.A[7678] = acadoWorkspace.evHu[82];
acadoWorkspace.A[7679] = acadoWorkspace.evHu[83];
acadoWorkspace.A[7752] = acadoWorkspace.evHu[84];
acadoWorkspace.A[7753] = acadoWorkspace.evHu[85];
acadoWorkspace.A[7754] = acadoWorkspace.evHu[86];
acadoWorkspace.A[7827] = acadoWorkspace.evHu[87];
acadoWorkspace.A[7828] = acadoWorkspace.evHu[88];
acadoWorkspace.A[7829] = acadoWorkspace.evHu[89];
acadoWorkspace.A[7905] = acadoWorkspace.evHu[90];
acadoWorkspace.A[7906] = acadoWorkspace.evHu[91];
acadoWorkspace.A[7907] = acadoWorkspace.evHu[92];
acadoWorkspace.A[7980] = acadoWorkspace.evHu[93];
acadoWorkspace.A[7981] = acadoWorkspace.evHu[94];
acadoWorkspace.A[7982] = acadoWorkspace.evHu[95];
acadoWorkspace.A[8055] = acadoWorkspace.evHu[96];
acadoWorkspace.A[8056] = acadoWorkspace.evHu[97];
acadoWorkspace.A[8057] = acadoWorkspace.evHu[98];
acadoWorkspace.A[8133] = acadoWorkspace.evHu[99];
acadoWorkspace.A[8134] = acadoWorkspace.evHu[100];
acadoWorkspace.A[8135] = acadoWorkspace.evHu[101];
acadoWorkspace.A[8208] = acadoWorkspace.evHu[102];
acadoWorkspace.A[8209] = acadoWorkspace.evHu[103];
acadoWorkspace.A[8210] = acadoWorkspace.evHu[104];
acadoWorkspace.A[8283] = acadoWorkspace.evHu[105];
acadoWorkspace.A[8284] = acadoWorkspace.evHu[106];
acadoWorkspace.A[8285] = acadoWorkspace.evHu[107];
acadoWorkspace.A[8361] = acadoWorkspace.evHu[108];
acadoWorkspace.A[8362] = acadoWorkspace.evHu[109];
acadoWorkspace.A[8363] = acadoWorkspace.evHu[110];
acadoWorkspace.A[8436] = acadoWorkspace.evHu[111];
acadoWorkspace.A[8437] = acadoWorkspace.evHu[112];
acadoWorkspace.A[8438] = acadoWorkspace.evHu[113];
acadoWorkspace.A[8511] = acadoWorkspace.evHu[114];
acadoWorkspace.A[8512] = acadoWorkspace.evHu[115];
acadoWorkspace.A[8513] = acadoWorkspace.evHu[116];
acadoWorkspace.A[8589] = acadoWorkspace.evHu[117];
acadoWorkspace.A[8590] = acadoWorkspace.evHu[118];
acadoWorkspace.A[8591] = acadoWorkspace.evHu[119];
acadoWorkspace.A[8664] = acadoWorkspace.evHu[120];
acadoWorkspace.A[8665] = acadoWorkspace.evHu[121];
acadoWorkspace.A[8666] = acadoWorkspace.evHu[122];
acadoWorkspace.A[8739] = acadoWorkspace.evHu[123];
acadoWorkspace.A[8740] = acadoWorkspace.evHu[124];
acadoWorkspace.A[8741] = acadoWorkspace.evHu[125];
acadoWorkspace.A[8817] = acadoWorkspace.evHu[126];
acadoWorkspace.A[8818] = acadoWorkspace.evHu[127];
acadoWorkspace.A[8819] = acadoWorkspace.evHu[128];
acadoWorkspace.A[8892] = acadoWorkspace.evHu[129];
acadoWorkspace.A[8893] = acadoWorkspace.evHu[130];
acadoWorkspace.A[8894] = acadoWorkspace.evHu[131];
acadoWorkspace.A[8967] = acadoWorkspace.evHu[132];
acadoWorkspace.A[8968] = acadoWorkspace.evHu[133];
acadoWorkspace.A[8969] = acadoWorkspace.evHu[134];
acadoWorkspace.A[9045] = acadoWorkspace.evHu[135];
acadoWorkspace.A[9046] = acadoWorkspace.evHu[136];
acadoWorkspace.A[9047] = acadoWorkspace.evHu[137];
acadoWorkspace.A[9120] = acadoWorkspace.evHu[138];
acadoWorkspace.A[9121] = acadoWorkspace.evHu[139];
acadoWorkspace.A[9122] = acadoWorkspace.evHu[140];
acadoWorkspace.A[9195] = acadoWorkspace.evHu[141];
acadoWorkspace.A[9196] = acadoWorkspace.evHu[142];
acadoWorkspace.A[9197] = acadoWorkspace.evHu[143];
acadoWorkspace.A[9273] = acadoWorkspace.evHu[144];
acadoWorkspace.A[9274] = acadoWorkspace.evHu[145];
acadoWorkspace.A[9275] = acadoWorkspace.evHu[146];
acadoWorkspace.A[9348] = acadoWorkspace.evHu[147];
acadoWorkspace.A[9349] = acadoWorkspace.evHu[148];
acadoWorkspace.A[9350] = acadoWorkspace.evHu[149];
acadoWorkspace.A[9423] = acadoWorkspace.evHu[150];
acadoWorkspace.A[9424] = acadoWorkspace.evHu[151];
acadoWorkspace.A[9425] = acadoWorkspace.evHu[152];
acadoWorkspace.A[9501] = acadoWorkspace.evHu[153];
acadoWorkspace.A[9502] = acadoWorkspace.evHu[154];
acadoWorkspace.A[9503] = acadoWorkspace.evHu[155];
acadoWorkspace.A[9576] = acadoWorkspace.evHu[156];
acadoWorkspace.A[9577] = acadoWorkspace.evHu[157];
acadoWorkspace.A[9578] = acadoWorkspace.evHu[158];
acadoWorkspace.A[9651] = acadoWorkspace.evHu[159];
acadoWorkspace.A[9652] = acadoWorkspace.evHu[160];
acadoWorkspace.A[9653] = acadoWorkspace.evHu[161];
acadoWorkspace.A[9729] = acadoWorkspace.evHu[162];
acadoWorkspace.A[9730] = acadoWorkspace.evHu[163];
acadoWorkspace.A[9731] = acadoWorkspace.evHu[164];
acadoWorkspace.A[9804] = acadoWorkspace.evHu[165];
acadoWorkspace.A[9805] = acadoWorkspace.evHu[166];
acadoWorkspace.A[9806] = acadoWorkspace.evHu[167];
acadoWorkspace.A[9879] = acadoWorkspace.evHu[168];
acadoWorkspace.A[9880] = acadoWorkspace.evHu[169];
acadoWorkspace.A[9881] = acadoWorkspace.evHu[170];
acadoWorkspace.A[9957] = acadoWorkspace.evHu[171];
acadoWorkspace.A[9958] = acadoWorkspace.evHu[172];
acadoWorkspace.A[9959] = acadoWorkspace.evHu[173];
acadoWorkspace.A[10032] = acadoWorkspace.evHu[174];
acadoWorkspace.A[10033] = acadoWorkspace.evHu[175];
acadoWorkspace.A[10034] = acadoWorkspace.evHu[176];
acadoWorkspace.A[10107] = acadoWorkspace.evHu[177];
acadoWorkspace.A[10108] = acadoWorkspace.evHu[178];
acadoWorkspace.A[10109] = acadoWorkspace.evHu[179];
acadoWorkspace.A[10185] = acadoWorkspace.evHu[180];
acadoWorkspace.A[10186] = acadoWorkspace.evHu[181];
acadoWorkspace.A[10187] = acadoWorkspace.evHu[182];
acadoWorkspace.A[10260] = acadoWorkspace.evHu[183];
acadoWorkspace.A[10261] = acadoWorkspace.evHu[184];
acadoWorkspace.A[10262] = acadoWorkspace.evHu[185];
acadoWorkspace.A[10335] = acadoWorkspace.evHu[186];
acadoWorkspace.A[10336] = acadoWorkspace.evHu[187];
acadoWorkspace.A[10337] = acadoWorkspace.evHu[188];
acadoWorkspace.A[10413] = acadoWorkspace.evHu[189];
acadoWorkspace.A[10414] = acadoWorkspace.evHu[190];
acadoWorkspace.A[10415] = acadoWorkspace.evHu[191];
acadoWorkspace.A[10488] = acadoWorkspace.evHu[192];
acadoWorkspace.A[10489] = acadoWorkspace.evHu[193];
acadoWorkspace.A[10490] = acadoWorkspace.evHu[194];
acadoWorkspace.A[10563] = acadoWorkspace.evHu[195];
acadoWorkspace.A[10564] = acadoWorkspace.evHu[196];
acadoWorkspace.A[10565] = acadoWorkspace.evHu[197];
acadoWorkspace.A[10641] = acadoWorkspace.evHu[198];
acadoWorkspace.A[10642] = acadoWorkspace.evHu[199];
acadoWorkspace.A[10643] = acadoWorkspace.evHu[200];
acadoWorkspace.A[10716] = acadoWorkspace.evHu[201];
acadoWorkspace.A[10717] = acadoWorkspace.evHu[202];
acadoWorkspace.A[10718] = acadoWorkspace.evHu[203];
acadoWorkspace.A[10791] = acadoWorkspace.evHu[204];
acadoWorkspace.A[10792] = acadoWorkspace.evHu[205];
acadoWorkspace.A[10793] = acadoWorkspace.evHu[206];
acadoWorkspace.A[10869] = acadoWorkspace.evHu[207];
acadoWorkspace.A[10870] = acadoWorkspace.evHu[208];
acadoWorkspace.A[10871] = acadoWorkspace.evHu[209];
acadoWorkspace.A[10944] = acadoWorkspace.evHu[210];
acadoWorkspace.A[10945] = acadoWorkspace.evHu[211];
acadoWorkspace.A[10946] = acadoWorkspace.evHu[212];
acadoWorkspace.A[11019] = acadoWorkspace.evHu[213];
acadoWorkspace.A[11020] = acadoWorkspace.evHu[214];
acadoWorkspace.A[11021] = acadoWorkspace.evHu[215];
acadoWorkspace.A[11097] = acadoWorkspace.evHu[216];
acadoWorkspace.A[11098] = acadoWorkspace.evHu[217];
acadoWorkspace.A[11099] = acadoWorkspace.evHu[218];
acadoWorkspace.A[11172] = acadoWorkspace.evHu[219];
acadoWorkspace.A[11173] = acadoWorkspace.evHu[220];
acadoWorkspace.A[11174] = acadoWorkspace.evHu[221];
acadoWorkspace.A[11247] = acadoWorkspace.evHu[222];
acadoWorkspace.A[11248] = acadoWorkspace.evHu[223];
acadoWorkspace.A[11249] = acadoWorkspace.evHu[224];
acadoWorkspace.lbA[75] = - acadoWorkspace.evH[0];
acadoWorkspace.lbA[76] = - acadoWorkspace.evH[1];
acadoWorkspace.lbA[77] = - acadoWorkspace.evH[2];
acadoWorkspace.lbA[78] = - acadoWorkspace.evH[3];
acadoWorkspace.lbA[79] = - acadoWorkspace.evH[4];
acadoWorkspace.lbA[80] = - acadoWorkspace.evH[5];
acadoWorkspace.lbA[81] = - acadoWorkspace.evH[6];
acadoWorkspace.lbA[82] = - acadoWorkspace.evH[7];
acadoWorkspace.lbA[83] = - acadoWorkspace.evH[8];
acadoWorkspace.lbA[84] = - acadoWorkspace.evH[9];
acadoWorkspace.lbA[85] = - acadoWorkspace.evH[10];
acadoWorkspace.lbA[86] = - acadoWorkspace.evH[11];
acadoWorkspace.lbA[87] = - acadoWorkspace.evH[12];
acadoWorkspace.lbA[88] = - acadoWorkspace.evH[13];
acadoWorkspace.lbA[89] = - acadoWorkspace.evH[14];
acadoWorkspace.lbA[90] = - acadoWorkspace.evH[15];
acadoWorkspace.lbA[91] = - acadoWorkspace.evH[16];
acadoWorkspace.lbA[92] = - acadoWorkspace.evH[17];
acadoWorkspace.lbA[93] = - acadoWorkspace.evH[18];
acadoWorkspace.lbA[94] = - acadoWorkspace.evH[19];
acadoWorkspace.lbA[95] = - acadoWorkspace.evH[20];
acadoWorkspace.lbA[96] = - acadoWorkspace.evH[21];
acadoWorkspace.lbA[97] = - acadoWorkspace.evH[22];
acadoWorkspace.lbA[98] = - acadoWorkspace.evH[23];
acadoWorkspace.lbA[99] = - acadoWorkspace.evH[24];
acadoWorkspace.lbA[100] = - acadoWorkspace.evH[25];
acadoWorkspace.lbA[101] = - acadoWorkspace.evH[26];
acadoWorkspace.lbA[102] = - acadoWorkspace.evH[27];
acadoWorkspace.lbA[103] = - acadoWorkspace.evH[28];
acadoWorkspace.lbA[104] = - acadoWorkspace.evH[29];
acadoWorkspace.lbA[105] = - acadoWorkspace.evH[30];
acadoWorkspace.lbA[106] = - acadoWorkspace.evH[31];
acadoWorkspace.lbA[107] = - acadoWorkspace.evH[32];
acadoWorkspace.lbA[108] = - acadoWorkspace.evH[33];
acadoWorkspace.lbA[109] = - acadoWorkspace.evH[34];
acadoWorkspace.lbA[110] = - acadoWorkspace.evH[35];
acadoWorkspace.lbA[111] = - acadoWorkspace.evH[36];
acadoWorkspace.lbA[112] = - acadoWorkspace.evH[37];
acadoWorkspace.lbA[113] = - acadoWorkspace.evH[38];
acadoWorkspace.lbA[114] = - acadoWorkspace.evH[39];
acadoWorkspace.lbA[115] = - acadoWorkspace.evH[40];
acadoWorkspace.lbA[116] = - acadoWorkspace.evH[41];
acadoWorkspace.lbA[117] = - acadoWorkspace.evH[42];
acadoWorkspace.lbA[118] = - acadoWorkspace.evH[43];
acadoWorkspace.lbA[119] = - acadoWorkspace.evH[44];
acadoWorkspace.lbA[120] = - acadoWorkspace.evH[45];
acadoWorkspace.lbA[121] = - acadoWorkspace.evH[46];
acadoWorkspace.lbA[122] = - acadoWorkspace.evH[47];
acadoWorkspace.lbA[123] = - acadoWorkspace.evH[48];
acadoWorkspace.lbA[124] = - acadoWorkspace.evH[49];
acadoWorkspace.lbA[125] = - acadoWorkspace.evH[50];
acadoWorkspace.lbA[126] = - acadoWorkspace.evH[51];
acadoWorkspace.lbA[127] = - acadoWorkspace.evH[52];
acadoWorkspace.lbA[128] = - acadoWorkspace.evH[53];
acadoWorkspace.lbA[129] = - acadoWorkspace.evH[54];
acadoWorkspace.lbA[130] = - acadoWorkspace.evH[55];
acadoWorkspace.lbA[131] = - acadoWorkspace.evH[56];
acadoWorkspace.lbA[132] = - acadoWorkspace.evH[57];
acadoWorkspace.lbA[133] = - acadoWorkspace.evH[58];
acadoWorkspace.lbA[134] = - acadoWorkspace.evH[59];
acadoWorkspace.lbA[135] = - acadoWorkspace.evH[60];
acadoWorkspace.lbA[136] = - acadoWorkspace.evH[61];
acadoWorkspace.lbA[137] = - acadoWorkspace.evH[62];
acadoWorkspace.lbA[138] = - acadoWorkspace.evH[63];
acadoWorkspace.lbA[139] = - acadoWorkspace.evH[64];
acadoWorkspace.lbA[140] = - acadoWorkspace.evH[65];
acadoWorkspace.lbA[141] = - acadoWorkspace.evH[66];
acadoWorkspace.lbA[142] = - acadoWorkspace.evH[67];
acadoWorkspace.lbA[143] = - acadoWorkspace.evH[68];
acadoWorkspace.lbA[144] = - acadoWorkspace.evH[69];
acadoWorkspace.lbA[145] = - acadoWorkspace.evH[70];
acadoWorkspace.lbA[146] = - acadoWorkspace.evH[71];
acadoWorkspace.lbA[147] = - acadoWorkspace.evH[72];
acadoWorkspace.lbA[148] = - acadoWorkspace.evH[73];
acadoWorkspace.lbA[149] = - acadoWorkspace.evH[74];

acadoWorkspace.ubA[75] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[0];
acadoWorkspace.ubA[76] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[1];
acadoWorkspace.ubA[77] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[2];
acadoWorkspace.ubA[78] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[3];
acadoWorkspace.ubA[79] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[4];
acadoWorkspace.ubA[80] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[5];
acadoWorkspace.ubA[81] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[6];
acadoWorkspace.ubA[82] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[7];
acadoWorkspace.ubA[83] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[8];
acadoWorkspace.ubA[84] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[9];
acadoWorkspace.ubA[85] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[10];
acadoWorkspace.ubA[86] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[11];
acadoWorkspace.ubA[87] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[12];
acadoWorkspace.ubA[88] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[13];
acadoWorkspace.ubA[89] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[14];
acadoWorkspace.ubA[90] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[15];
acadoWorkspace.ubA[91] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[16];
acadoWorkspace.ubA[92] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[17];
acadoWorkspace.ubA[93] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[18];
acadoWorkspace.ubA[94] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[19];
acadoWorkspace.ubA[95] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[20];
acadoWorkspace.ubA[96] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[21];
acadoWorkspace.ubA[97] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[22];
acadoWorkspace.ubA[98] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[23];
acadoWorkspace.ubA[99] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[24];
acadoWorkspace.ubA[100] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[25];
acadoWorkspace.ubA[101] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[26];
acadoWorkspace.ubA[102] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[27];
acadoWorkspace.ubA[103] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[28];
acadoWorkspace.ubA[104] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[29];
acadoWorkspace.ubA[105] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[30];
acadoWorkspace.ubA[106] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[31];
acadoWorkspace.ubA[107] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[32];
acadoWorkspace.ubA[108] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[33];
acadoWorkspace.ubA[109] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[34];
acadoWorkspace.ubA[110] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[35];
acadoWorkspace.ubA[111] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[36];
acadoWorkspace.ubA[112] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[37];
acadoWorkspace.ubA[113] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[38];
acadoWorkspace.ubA[114] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[39];
acadoWorkspace.ubA[115] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[40];
acadoWorkspace.ubA[116] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[41];
acadoWorkspace.ubA[117] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[42];
acadoWorkspace.ubA[118] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[43];
acadoWorkspace.ubA[119] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[44];
acadoWorkspace.ubA[120] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[45];
acadoWorkspace.ubA[121] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[46];
acadoWorkspace.ubA[122] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[47];
acadoWorkspace.ubA[123] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[48];
acadoWorkspace.ubA[124] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[49];
acadoWorkspace.ubA[125] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[50];
acadoWorkspace.ubA[126] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[51];
acadoWorkspace.ubA[127] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[52];
acadoWorkspace.ubA[128] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[53];
acadoWorkspace.ubA[129] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[54];
acadoWorkspace.ubA[130] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[55];
acadoWorkspace.ubA[131] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[56];
acadoWorkspace.ubA[132] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[57];
acadoWorkspace.ubA[133] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[58];
acadoWorkspace.ubA[134] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[59];
acadoWorkspace.ubA[135] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[60];
acadoWorkspace.ubA[136] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[61];
acadoWorkspace.ubA[137] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[62];
acadoWorkspace.ubA[138] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[63];
acadoWorkspace.ubA[139] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[64];
acadoWorkspace.ubA[140] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[65];
acadoWorkspace.ubA[141] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[66];
acadoWorkspace.ubA[142] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[67];
acadoWorkspace.ubA[143] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[68];
acadoWorkspace.ubA[144] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[69];
acadoWorkspace.ubA[145] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[70];
acadoWorkspace.ubA[146] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[71];
acadoWorkspace.ubA[147] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[72];
acadoWorkspace.ubA[148] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[73];
acadoWorkspace.ubA[149] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[74];

}

void acado_condenseFdb(  )
{
int lRun1;
real_t tmp;

acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];
acadoWorkspace.Dx0[5] = acadoVariables.x0[5] - acadoVariables.x[5];
for (lRun1 = 0; lRun1 < 250; ++lRun1)
acadoWorkspace.Dy[lRun1] -= acadoVariables.y[lRun1];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];
acadoWorkspace.DyN[3] -= acadoVariables.yN[3];
acadoWorkspace.DyN[4] -= acadoVariables.yN[4];
acadoWorkspace.DyN[5] -= acadoVariables.yN[5];
acadoWorkspace.DyN[6] -= acadoVariables.yN[6];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 30 ]), &(acadoWorkspace.Dy[ 10 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 60 ]), &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 90 ]), &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 120 ]), &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 150 ]), &(acadoWorkspace.Dy[ 50 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 180 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 210 ]), &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 240 ]), &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 270 ]), &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.g[ 27 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 300 ]), &(acadoWorkspace.Dy[ 100 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 330 ]), &(acadoWorkspace.Dy[ 110 ]), &(acadoWorkspace.g[ 33 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 360 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 390 ]), &(acadoWorkspace.Dy[ 130 ]), &(acadoWorkspace.g[ 39 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 420 ]), &(acadoWorkspace.Dy[ 140 ]), &(acadoWorkspace.g[ 42 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 450 ]), &(acadoWorkspace.Dy[ 150 ]), &(acadoWorkspace.g[ 45 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 480 ]), &(acadoWorkspace.Dy[ 160 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 510 ]), &(acadoWorkspace.Dy[ 170 ]), &(acadoWorkspace.g[ 51 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 540 ]), &(acadoWorkspace.Dy[ 180 ]), &(acadoWorkspace.g[ 54 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 570 ]), &(acadoWorkspace.Dy[ 190 ]), &(acadoWorkspace.g[ 57 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 600 ]), &(acadoWorkspace.Dy[ 200 ]), &(acadoWorkspace.g[ 60 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 630 ]), &(acadoWorkspace.Dy[ 210 ]), &(acadoWorkspace.g[ 63 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 660 ]), &(acadoWorkspace.Dy[ 220 ]), &(acadoWorkspace.g[ 66 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 690 ]), &(acadoWorkspace.Dy[ 230 ]), &(acadoWorkspace.g[ 69 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 720 ]), &(acadoWorkspace.Dy[ 240 ]), &(acadoWorkspace.g[ 72 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 60 ]), &(acadoWorkspace.Dy[ 10 ]), &(acadoWorkspace.QDy[ 6 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 120 ]), &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 180 ]), &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.QDy[ 18 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 240 ]), &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 300 ]), &(acadoWorkspace.Dy[ 50 ]), &(acadoWorkspace.QDy[ 30 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 360 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 420 ]), &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.QDy[ 42 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 480 ]), &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.QDy[ 48 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 540 ]), &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.QDy[ 54 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 600 ]), &(acadoWorkspace.Dy[ 100 ]), &(acadoWorkspace.QDy[ 60 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 660 ]), &(acadoWorkspace.Dy[ 110 ]), &(acadoWorkspace.QDy[ 66 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 720 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.QDy[ 72 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 780 ]), &(acadoWorkspace.Dy[ 130 ]), &(acadoWorkspace.QDy[ 78 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 840 ]), &(acadoWorkspace.Dy[ 140 ]), &(acadoWorkspace.QDy[ 84 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 900 ]), &(acadoWorkspace.Dy[ 150 ]), &(acadoWorkspace.QDy[ 90 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 960 ]), &(acadoWorkspace.Dy[ 160 ]), &(acadoWorkspace.QDy[ 96 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1020 ]), &(acadoWorkspace.Dy[ 170 ]), &(acadoWorkspace.QDy[ 102 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1080 ]), &(acadoWorkspace.Dy[ 180 ]), &(acadoWorkspace.QDy[ 108 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1140 ]), &(acadoWorkspace.Dy[ 190 ]), &(acadoWorkspace.QDy[ 114 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1200 ]), &(acadoWorkspace.Dy[ 200 ]), &(acadoWorkspace.QDy[ 120 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1260 ]), &(acadoWorkspace.Dy[ 210 ]), &(acadoWorkspace.QDy[ 126 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1320 ]), &(acadoWorkspace.Dy[ 220 ]), &(acadoWorkspace.QDy[ 132 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1380 ]), &(acadoWorkspace.Dy[ 230 ]), &(acadoWorkspace.QDy[ 138 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1440 ]), &(acadoWorkspace.Dy[ 240 ]), &(acadoWorkspace.QDy[ 144 ]) );

acadoWorkspace.QDy[150] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[6];
acadoWorkspace.QDy[151] = + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[12]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[13]*acadoWorkspace.DyN[6];
acadoWorkspace.QDy[152] = + acadoWorkspace.QN2[14]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[15]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[16]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[17]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[18]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[19]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[20]*acadoWorkspace.DyN[6];
acadoWorkspace.QDy[153] = + acadoWorkspace.QN2[21]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[22]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[23]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[24]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[25]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[26]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[27]*acadoWorkspace.DyN[6];
acadoWorkspace.QDy[154] = + acadoWorkspace.QN2[28]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[29]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[30]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[31]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[32]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[33]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[34]*acadoWorkspace.DyN[6];
acadoWorkspace.QDy[155] = + acadoWorkspace.QN2[35]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[36]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[37]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[38]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[39]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[40]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[41]*acadoWorkspace.DyN[6];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 6 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.sbar[ 12 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 18 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 30 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 42 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.sbar[ 42 ]), &(acadoWorkspace.sbar[ 48 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.sbar[ 54 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.sbar[ 60 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 360 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.sbar[ 66 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 396 ]), &(acadoWorkspace.sbar[ 66 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 78 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 468 ]), &(acadoWorkspace.sbar[ 78 ]), &(acadoWorkspace.sbar[ 84 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 504 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 540 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 96 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.sbar[ 102 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 612 ]), &(acadoWorkspace.sbar[ 102 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 114 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 684 ]), &(acadoWorkspace.sbar[ 114 ]), &(acadoWorkspace.sbar[ 120 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 720 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.sbar[ 126 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 756 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.sbar[ 132 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 792 ]), &(acadoWorkspace.sbar[ 132 ]), &(acadoWorkspace.sbar[ 138 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 828 ]), &(acadoWorkspace.sbar[ 138 ]), &(acadoWorkspace.sbar[ 144 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 864 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.sbar[ 150 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[150] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[151] + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[152] + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[153] + acadoWorkspace.QN1[4]*acadoWorkspace.sbar[154] + acadoWorkspace.QN1[5]*acadoWorkspace.sbar[155] + acadoWorkspace.QDy[150];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[6]*acadoWorkspace.sbar[150] + acadoWorkspace.QN1[7]*acadoWorkspace.sbar[151] + acadoWorkspace.QN1[8]*acadoWorkspace.sbar[152] + acadoWorkspace.QN1[9]*acadoWorkspace.sbar[153] + acadoWorkspace.QN1[10]*acadoWorkspace.sbar[154] + acadoWorkspace.QN1[11]*acadoWorkspace.sbar[155] + acadoWorkspace.QDy[151];
acadoWorkspace.w1[2] = + acadoWorkspace.QN1[12]*acadoWorkspace.sbar[150] + acadoWorkspace.QN1[13]*acadoWorkspace.sbar[151] + acadoWorkspace.QN1[14]*acadoWorkspace.sbar[152] + acadoWorkspace.QN1[15]*acadoWorkspace.sbar[153] + acadoWorkspace.QN1[16]*acadoWorkspace.sbar[154] + acadoWorkspace.QN1[17]*acadoWorkspace.sbar[155] + acadoWorkspace.QDy[152];
acadoWorkspace.w1[3] = + acadoWorkspace.QN1[18]*acadoWorkspace.sbar[150] + acadoWorkspace.QN1[19]*acadoWorkspace.sbar[151] + acadoWorkspace.QN1[20]*acadoWorkspace.sbar[152] + acadoWorkspace.QN1[21]*acadoWorkspace.sbar[153] + acadoWorkspace.QN1[22]*acadoWorkspace.sbar[154] + acadoWorkspace.QN1[23]*acadoWorkspace.sbar[155] + acadoWorkspace.QDy[153];
acadoWorkspace.w1[4] = + acadoWorkspace.QN1[24]*acadoWorkspace.sbar[150] + acadoWorkspace.QN1[25]*acadoWorkspace.sbar[151] + acadoWorkspace.QN1[26]*acadoWorkspace.sbar[152] + acadoWorkspace.QN1[27]*acadoWorkspace.sbar[153] + acadoWorkspace.QN1[28]*acadoWorkspace.sbar[154] + acadoWorkspace.QN1[29]*acadoWorkspace.sbar[155] + acadoWorkspace.QDy[154];
acadoWorkspace.w1[5] = + acadoWorkspace.QN1[30]*acadoWorkspace.sbar[150] + acadoWorkspace.QN1[31]*acadoWorkspace.sbar[151] + acadoWorkspace.QN1[32]*acadoWorkspace.sbar[152] + acadoWorkspace.QN1[33]*acadoWorkspace.sbar[153] + acadoWorkspace.QN1[34]*acadoWorkspace.sbar[154] + acadoWorkspace.QN1[35]*acadoWorkspace.sbar[155] + acadoWorkspace.QDy[155];
acado_macBTw1( &(acadoWorkspace.evGu[ 432 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 72 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 864 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 144 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 864 ]), &(acadoWorkspace.sbar[ 144 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 414 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 69 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 828 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 138 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 828 ]), &(acadoWorkspace.sbar[ 138 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 396 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 66 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 792 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 132 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 792 ]), &(acadoWorkspace.sbar[ 132 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 378 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 63 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 756 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 126 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 756 ]), &(acadoWorkspace.sbar[ 126 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 360 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 60 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 720 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 120 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 720 ]), &(acadoWorkspace.sbar[ 120 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 342 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 57 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 684 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 114 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 684 ]), &(acadoWorkspace.sbar[ 114 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 324 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 54 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 648 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 108 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 648 ]), &(acadoWorkspace.sbar[ 108 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 306 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 51 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 612 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 102 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 612 ]), &(acadoWorkspace.sbar[ 102 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 288 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 48 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 96 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.sbar[ 96 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 270 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 45 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 540 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 90 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 540 ]), &(acadoWorkspace.sbar[ 90 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 252 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 42 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 504 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 84 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 504 ]), &(acadoWorkspace.sbar[ 84 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 234 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 39 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 468 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 78 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 468 ]), &(acadoWorkspace.sbar[ 78 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 216 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 36 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 432 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 72 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 432 ]), &(acadoWorkspace.sbar[ 72 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 198 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 33 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 396 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 66 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 396 ]), &(acadoWorkspace.sbar[ 66 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 180 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 30 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 360 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 60 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 360 ]), &(acadoWorkspace.sbar[ 60 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 162 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 27 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 324 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 54 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 324 ]), &(acadoWorkspace.sbar[ 54 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 24 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 288 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 48 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.sbar[ 48 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 126 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 21 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 252 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 42 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 252 ]), &(acadoWorkspace.sbar[ 42 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 108 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 18 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 216 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 216 ]), &(acadoWorkspace.sbar[ 36 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 90 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 15 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 180 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 30 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 180 ]), &(acadoWorkspace.sbar[ 30 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 72 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 24 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.sbar[ 24 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 54 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 9 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 108 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 18 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 108 ]), &(acadoWorkspace.sbar[ 18 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 36 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 6 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 72 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 12 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 72 ]), &(acadoWorkspace.sbar[ 12 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 18 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 3 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 6 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.sbar[ 6 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );


tmp = acadoWorkspace.sbar[9] + acadoVariables.x[9];
acadoWorkspace.lbA[0] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[0] = (real_t)1.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[10] + acadoVariables.x[10];
acadoWorkspace.lbA[1] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[1] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[11] + acadoVariables.x[11];
acadoWorkspace.lbA[2] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[2] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[15] + acadoVariables.x[15];
acadoWorkspace.lbA[3] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[3] = (real_t)1.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[16] + acadoVariables.x[16];
acadoWorkspace.lbA[4] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[4] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[17] + acadoVariables.x[17];
acadoWorkspace.lbA[5] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[5] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[21] + acadoVariables.x[21];
acadoWorkspace.lbA[6] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[6] = (real_t)1.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[22] + acadoVariables.x[22];
acadoWorkspace.lbA[7] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[7] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[23] + acadoVariables.x[23];
acadoWorkspace.lbA[8] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[8] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[27] + acadoVariables.x[27];
acadoWorkspace.lbA[9] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[9] = (real_t)1.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[28] + acadoVariables.x[28];
acadoWorkspace.lbA[10] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[10] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[29] + acadoVariables.x[29];
acadoWorkspace.lbA[11] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[11] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[33] + acadoVariables.x[33];
acadoWorkspace.lbA[12] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[12] = (real_t)1.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[34] + acadoVariables.x[34];
acadoWorkspace.lbA[13] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[13] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[35] + acadoVariables.x[35];
acadoWorkspace.lbA[14] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[14] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[39] + acadoVariables.x[39];
acadoWorkspace.lbA[15] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[15] = (real_t)1.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[40] + acadoVariables.x[40];
acadoWorkspace.lbA[16] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[16] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[41] + acadoVariables.x[41];
acadoWorkspace.lbA[17] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[17] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[45] + acadoVariables.x[45];
acadoWorkspace.lbA[18] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[18] = (real_t)1.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[46] + acadoVariables.x[46];
acadoWorkspace.lbA[19] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[19] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[47] + acadoVariables.x[47];
acadoWorkspace.lbA[20] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[20] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[51] + acadoVariables.x[51];
acadoWorkspace.lbA[21] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[21] = (real_t)1.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[52] + acadoVariables.x[52];
acadoWorkspace.lbA[22] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[22] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[53] + acadoVariables.x[53];
acadoWorkspace.lbA[23] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[23] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[57] + acadoVariables.x[57];
acadoWorkspace.lbA[24] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[24] = (real_t)1.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[58] + acadoVariables.x[58];
acadoWorkspace.lbA[25] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[25] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[59] + acadoVariables.x[59];
acadoWorkspace.lbA[26] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[26] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[63] + acadoVariables.x[63];
acadoWorkspace.lbA[27] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[27] = (real_t)1.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[64] + acadoVariables.x[64];
acadoWorkspace.lbA[28] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[28] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[65] + acadoVariables.x[65];
acadoWorkspace.lbA[29] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[29] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[69] + acadoVariables.x[69];
acadoWorkspace.lbA[30] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[30] = (real_t)1.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[70] + acadoVariables.x[70];
acadoWorkspace.lbA[31] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[31] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[71] + acadoVariables.x[71];
acadoWorkspace.lbA[32] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[32] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[75] + acadoVariables.x[75];
acadoWorkspace.lbA[33] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[33] = (real_t)1.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[76] + acadoVariables.x[76];
acadoWorkspace.lbA[34] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[34] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[77] + acadoVariables.x[77];
acadoWorkspace.lbA[35] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[35] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[81] + acadoVariables.x[81];
acadoWorkspace.lbA[36] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[36] = (real_t)1.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[82] + acadoVariables.x[82];
acadoWorkspace.lbA[37] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[37] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[83] + acadoVariables.x[83];
acadoWorkspace.lbA[38] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[38] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[87] + acadoVariables.x[87];
acadoWorkspace.lbA[39] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[39] = (real_t)1.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[88] + acadoVariables.x[88];
acadoWorkspace.lbA[40] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[40] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[89] + acadoVariables.x[89];
acadoWorkspace.lbA[41] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[41] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[93] + acadoVariables.x[93];
acadoWorkspace.lbA[42] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[42] = (real_t)1.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[94] + acadoVariables.x[94];
acadoWorkspace.lbA[43] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[43] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[95] + acadoVariables.x[95];
acadoWorkspace.lbA[44] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[44] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[99] + acadoVariables.x[99];
acadoWorkspace.lbA[45] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[45] = (real_t)1.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[100] + acadoVariables.x[100];
acadoWorkspace.lbA[46] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[46] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[101] + acadoVariables.x[101];
acadoWorkspace.lbA[47] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[47] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[105] + acadoVariables.x[105];
acadoWorkspace.lbA[48] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[48] = (real_t)1.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[106] + acadoVariables.x[106];
acadoWorkspace.lbA[49] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[49] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[107] + acadoVariables.x[107];
acadoWorkspace.lbA[50] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[50] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[111] + acadoVariables.x[111];
acadoWorkspace.lbA[51] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[51] = (real_t)1.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[112] + acadoVariables.x[112];
acadoWorkspace.lbA[52] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[52] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[113] + acadoVariables.x[113];
acadoWorkspace.lbA[53] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[53] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[117] + acadoVariables.x[117];
acadoWorkspace.lbA[54] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[54] = (real_t)1.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[118] + acadoVariables.x[118];
acadoWorkspace.lbA[55] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[55] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[119] + acadoVariables.x[119];
acadoWorkspace.lbA[56] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[56] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[123] + acadoVariables.x[123];
acadoWorkspace.lbA[57] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[57] = (real_t)1.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[124] + acadoVariables.x[124];
acadoWorkspace.lbA[58] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[58] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[125] + acadoVariables.x[125];
acadoWorkspace.lbA[59] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[59] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[129] + acadoVariables.x[129];
acadoWorkspace.lbA[60] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[60] = (real_t)1.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[130] + acadoVariables.x[130];
acadoWorkspace.lbA[61] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[61] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[131] + acadoVariables.x[131];
acadoWorkspace.lbA[62] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[62] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[135] + acadoVariables.x[135];
acadoWorkspace.lbA[63] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[63] = (real_t)1.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[136] + acadoVariables.x[136];
acadoWorkspace.lbA[64] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[64] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[137] + acadoVariables.x[137];
acadoWorkspace.lbA[65] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[65] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[141] + acadoVariables.x[141];
acadoWorkspace.lbA[66] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[66] = (real_t)1.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[142] + acadoVariables.x[142];
acadoWorkspace.lbA[67] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[67] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[143] + acadoVariables.x[143];
acadoWorkspace.lbA[68] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[68] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[147] + acadoVariables.x[147];
acadoWorkspace.lbA[69] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[69] = (real_t)1.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[148] + acadoVariables.x[148];
acadoWorkspace.lbA[70] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[70] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[149] + acadoVariables.x[149];
acadoWorkspace.lbA[71] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[71] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[153] + acadoVariables.x[153];
acadoWorkspace.lbA[72] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[72] = (real_t)1.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[154] + acadoVariables.x[154];
acadoWorkspace.lbA[73] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[73] = (real_t)5.0000000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[155] + acadoVariables.x[155];
acadoWorkspace.lbA[74] = (real_t)-5.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[74] = (real_t)5.0000000000000000e+00 - tmp;

acado_macHxd( acadoWorkspace.evHx, acadoWorkspace.sbar, &(acadoWorkspace.lbA[ 75 ]), &(acadoWorkspace.ubA[ 75 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 18 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.lbA[ 78 ]), &(acadoWorkspace.ubA[ 78 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 36 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.lbA[ 81 ]), &(acadoWorkspace.ubA[ 81 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 54 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.lbA[ 84 ]), &(acadoWorkspace.ubA[ 84 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.lbA[ 87 ]), &(acadoWorkspace.ubA[ 87 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.lbA[ 90 ]), &(acadoWorkspace.ubA[ 90 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.lbA[ 93 ]), &(acadoWorkspace.ubA[ 93 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 126 ]), &(acadoWorkspace.sbar[ 42 ]), &(acadoWorkspace.lbA[ 96 ]), &(acadoWorkspace.ubA[ 96 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.lbA[ 99 ]), &(acadoWorkspace.ubA[ 99 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 162 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.lbA[ 102 ]), &(acadoWorkspace.ubA[ 102 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.lbA[ 105 ]), &(acadoWorkspace.ubA[ 105 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 198 ]), &(acadoWorkspace.sbar[ 66 ]), &(acadoWorkspace.lbA[ 108 ]), &(acadoWorkspace.ubA[ 108 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.lbA[ 111 ]), &(acadoWorkspace.ubA[ 111 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 234 ]), &(acadoWorkspace.sbar[ 78 ]), &(acadoWorkspace.lbA[ 114 ]), &(acadoWorkspace.ubA[ 114 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 252 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.lbA[ 117 ]), &(acadoWorkspace.ubA[ 117 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 270 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.lbA[ 120 ]), &(acadoWorkspace.ubA[ 120 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 288 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.lbA[ 123 ]), &(acadoWorkspace.ubA[ 123 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 306 ]), &(acadoWorkspace.sbar[ 102 ]), &(acadoWorkspace.lbA[ 126 ]), &(acadoWorkspace.ubA[ 126 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 324 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.lbA[ 129 ]), &(acadoWorkspace.ubA[ 129 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 342 ]), &(acadoWorkspace.sbar[ 114 ]), &(acadoWorkspace.lbA[ 132 ]), &(acadoWorkspace.ubA[ 132 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 360 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.lbA[ 135 ]), &(acadoWorkspace.ubA[ 135 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 378 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.lbA[ 138 ]), &(acadoWorkspace.ubA[ 138 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 396 ]), &(acadoWorkspace.sbar[ 132 ]), &(acadoWorkspace.lbA[ 141 ]), &(acadoWorkspace.ubA[ 141 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 414 ]), &(acadoWorkspace.sbar[ 138 ]), &(acadoWorkspace.lbA[ 144 ]), &(acadoWorkspace.ubA[ 144 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 432 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.lbA[ 147 ]), &(acadoWorkspace.ubA[ 147 ]) );

}

void acado_expand(  )
{
int lRun1;
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];
acadoVariables.u[30] += acadoWorkspace.x[30];
acadoVariables.u[31] += acadoWorkspace.x[31];
acadoVariables.u[32] += acadoWorkspace.x[32];
acadoVariables.u[33] += acadoWorkspace.x[33];
acadoVariables.u[34] += acadoWorkspace.x[34];
acadoVariables.u[35] += acadoWorkspace.x[35];
acadoVariables.u[36] += acadoWorkspace.x[36];
acadoVariables.u[37] += acadoWorkspace.x[37];
acadoVariables.u[38] += acadoWorkspace.x[38];
acadoVariables.u[39] += acadoWorkspace.x[39];
acadoVariables.u[40] += acadoWorkspace.x[40];
acadoVariables.u[41] += acadoWorkspace.x[41];
acadoVariables.u[42] += acadoWorkspace.x[42];
acadoVariables.u[43] += acadoWorkspace.x[43];
acadoVariables.u[44] += acadoWorkspace.x[44];
acadoVariables.u[45] += acadoWorkspace.x[45];
acadoVariables.u[46] += acadoWorkspace.x[46];
acadoVariables.u[47] += acadoWorkspace.x[47];
acadoVariables.u[48] += acadoWorkspace.x[48];
acadoVariables.u[49] += acadoWorkspace.x[49];
acadoVariables.u[50] += acadoWorkspace.x[50];
acadoVariables.u[51] += acadoWorkspace.x[51];
acadoVariables.u[52] += acadoWorkspace.x[52];
acadoVariables.u[53] += acadoWorkspace.x[53];
acadoVariables.u[54] += acadoWorkspace.x[54];
acadoVariables.u[55] += acadoWorkspace.x[55];
acadoVariables.u[56] += acadoWorkspace.x[56];
acadoVariables.u[57] += acadoWorkspace.x[57];
acadoVariables.u[58] += acadoWorkspace.x[58];
acadoVariables.u[59] += acadoWorkspace.x[59];
acadoVariables.u[60] += acadoWorkspace.x[60];
acadoVariables.u[61] += acadoWorkspace.x[61];
acadoVariables.u[62] += acadoWorkspace.x[62];
acadoVariables.u[63] += acadoWorkspace.x[63];
acadoVariables.u[64] += acadoWorkspace.x[64];
acadoVariables.u[65] += acadoWorkspace.x[65];
acadoVariables.u[66] += acadoWorkspace.x[66];
acadoVariables.u[67] += acadoWorkspace.x[67];
acadoVariables.u[68] += acadoWorkspace.x[68];
acadoVariables.u[69] += acadoWorkspace.x[69];
acadoVariables.u[70] += acadoWorkspace.x[70];
acadoVariables.u[71] += acadoWorkspace.x[71];
acadoVariables.u[72] += acadoWorkspace.x[72];
acadoVariables.u[73] += acadoWorkspace.x[73];
acadoVariables.u[74] += acadoWorkspace.x[74];
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
for (lRun1 = 0; lRun1 < 150; ++lRun1)
acadoWorkspace.sbar[lRun1 + 6] = acadoWorkspace.d[lRun1];

acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 6 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.evGu[ 18 ]), &(acadoWorkspace.x[ 3 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.sbar[ 12 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.evGu[ 36 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 18 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.evGu[ 54 ]), &(acadoWorkspace.x[ 9 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.evGu[ 72 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 30 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.evGu[ 90 ]), &(acadoWorkspace.x[ 15 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.evGu[ 108 ]), &(acadoWorkspace.x[ 18 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 42 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.evGu[ 126 ]), &(acadoWorkspace.x[ 21 ]), &(acadoWorkspace.sbar[ 42 ]), &(acadoWorkspace.sbar[ 48 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.evGu[ 144 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.sbar[ 54 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.evGu[ 162 ]), &(acadoWorkspace.x[ 27 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.sbar[ 60 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 360 ]), &(acadoWorkspace.evGu[ 180 ]), &(acadoWorkspace.x[ 30 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.sbar[ 66 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 396 ]), &(acadoWorkspace.evGu[ 198 ]), &(acadoWorkspace.x[ 33 ]), &(acadoWorkspace.sbar[ 66 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.evGu[ 216 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 78 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 468 ]), &(acadoWorkspace.evGu[ 234 ]), &(acadoWorkspace.x[ 39 ]), &(acadoWorkspace.sbar[ 78 ]), &(acadoWorkspace.sbar[ 84 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 504 ]), &(acadoWorkspace.evGu[ 252 ]), &(acadoWorkspace.x[ 42 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 540 ]), &(acadoWorkspace.evGu[ 270 ]), &(acadoWorkspace.x[ 45 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 96 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.evGu[ 288 ]), &(acadoWorkspace.x[ 48 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.sbar[ 102 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 612 ]), &(acadoWorkspace.evGu[ 306 ]), &(acadoWorkspace.x[ 51 ]), &(acadoWorkspace.sbar[ 102 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.evGu[ 324 ]), &(acadoWorkspace.x[ 54 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 114 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 684 ]), &(acadoWorkspace.evGu[ 342 ]), &(acadoWorkspace.x[ 57 ]), &(acadoWorkspace.sbar[ 114 ]), &(acadoWorkspace.sbar[ 120 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 720 ]), &(acadoWorkspace.evGu[ 360 ]), &(acadoWorkspace.x[ 60 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.sbar[ 126 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 756 ]), &(acadoWorkspace.evGu[ 378 ]), &(acadoWorkspace.x[ 63 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.sbar[ 132 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 792 ]), &(acadoWorkspace.evGu[ 396 ]), &(acadoWorkspace.x[ 66 ]), &(acadoWorkspace.sbar[ 132 ]), &(acadoWorkspace.sbar[ 138 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 828 ]), &(acadoWorkspace.evGu[ 414 ]), &(acadoWorkspace.x[ 69 ]), &(acadoWorkspace.sbar[ 138 ]), &(acadoWorkspace.sbar[ 144 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 864 ]), &(acadoWorkspace.evGu[ 432 ]), &(acadoWorkspace.x[ 72 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.sbar[ 150 ]) );
for (lRun1 = 0; lRun1 < 156; ++lRun1)
acadoVariables.x[lRun1] += acadoWorkspace.sbar[lRun1];

}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 25; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 6];
acadoWorkspace.state[1] = acadoVariables.x[index * 6 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 6 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 6 + 3];
acadoWorkspace.state[4] = acadoVariables.x[index * 6 + 4];
acadoWorkspace.state[5] = acadoVariables.x[index * 6 + 5];
acadoWorkspace.state[60] = acadoVariables.u[index * 3];
acadoWorkspace.state[61] = acadoVariables.u[index * 3 + 1];
acadoWorkspace.state[62] = acadoVariables.u[index * 3 + 2];
acadoWorkspace.state[63] = acadoVariables.od[index * 3];
acadoWorkspace.state[64] = acadoVariables.od[index * 3 + 1];
acadoWorkspace.state[65] = acadoVariables.od[index * 3 + 2];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 6 + 6] = acadoWorkspace.state[0];
acadoVariables.x[index * 6 + 7] = acadoWorkspace.state[1];
acadoVariables.x[index * 6 + 8] = acadoWorkspace.state[2];
acadoVariables.x[index * 6 + 9] = acadoWorkspace.state[3];
acadoVariables.x[index * 6 + 10] = acadoWorkspace.state[4];
acadoVariables.x[index * 6 + 11] = acadoWorkspace.state[5];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 25; ++index)
{
acadoVariables.x[index * 6] = acadoVariables.x[index * 6 + 6];
acadoVariables.x[index * 6 + 1] = acadoVariables.x[index * 6 + 7];
acadoVariables.x[index * 6 + 2] = acadoVariables.x[index * 6 + 8];
acadoVariables.x[index * 6 + 3] = acadoVariables.x[index * 6 + 9];
acadoVariables.x[index * 6 + 4] = acadoVariables.x[index * 6 + 10];
acadoVariables.x[index * 6 + 5] = acadoVariables.x[index * 6 + 11];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[150] = xEnd[0];
acadoVariables.x[151] = xEnd[1];
acadoVariables.x[152] = xEnd[2];
acadoVariables.x[153] = xEnd[3];
acadoVariables.x[154] = xEnd[4];
acadoVariables.x[155] = xEnd[5];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[150];
acadoWorkspace.state[1] = acadoVariables.x[151];
acadoWorkspace.state[2] = acadoVariables.x[152];
acadoWorkspace.state[3] = acadoVariables.x[153];
acadoWorkspace.state[4] = acadoVariables.x[154];
acadoWorkspace.state[5] = acadoVariables.x[155];
if (uEnd != 0)
{
acadoWorkspace.state[60] = uEnd[0];
acadoWorkspace.state[61] = uEnd[1];
acadoWorkspace.state[62] = uEnd[2];
}
else
{
acadoWorkspace.state[60] = acadoVariables.u[72];
acadoWorkspace.state[61] = acadoVariables.u[73];
acadoWorkspace.state[62] = acadoVariables.u[74];
}
acadoWorkspace.state[63] = acadoVariables.od[75];
acadoWorkspace.state[64] = acadoVariables.od[76];
acadoWorkspace.state[65] = acadoVariables.od[77];

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[150] = acadoWorkspace.state[0];
acadoVariables.x[151] = acadoWorkspace.state[1];
acadoVariables.x[152] = acadoWorkspace.state[2];
acadoVariables.x[153] = acadoWorkspace.state[3];
acadoVariables.x[154] = acadoWorkspace.state[4];
acadoVariables.x[155] = acadoWorkspace.state[5];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 24; ++index)
{
acadoVariables.u[index * 3] = acadoVariables.u[index * 3 + 3];
acadoVariables.u[index * 3 + 1] = acadoVariables.u[index * 3 + 4];
acadoVariables.u[index * 3 + 2] = acadoVariables.u[index * 3 + 5];
}

if (uEnd != 0)
{
acadoVariables.u[72] = uEnd[0];
acadoVariables.u[73] = uEnd[1];
acadoVariables.u[74] = uEnd[2];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59] + acadoWorkspace.g[60]*acadoWorkspace.x[60] + acadoWorkspace.g[61]*acadoWorkspace.x[61] + acadoWorkspace.g[62]*acadoWorkspace.x[62] + acadoWorkspace.g[63]*acadoWorkspace.x[63] + acadoWorkspace.g[64]*acadoWorkspace.x[64] + acadoWorkspace.g[65]*acadoWorkspace.x[65] + acadoWorkspace.g[66]*acadoWorkspace.x[66] + acadoWorkspace.g[67]*acadoWorkspace.x[67] + acadoWorkspace.g[68]*acadoWorkspace.x[68] + acadoWorkspace.g[69]*acadoWorkspace.x[69] + acadoWorkspace.g[70]*acadoWorkspace.x[70] + acadoWorkspace.g[71]*acadoWorkspace.x[71] + acadoWorkspace.g[72]*acadoWorkspace.x[72] + acadoWorkspace.g[73]*acadoWorkspace.x[73] + acadoWorkspace.g[74]*acadoWorkspace.x[74];
kkt = fabs( kkt );
for (index = 0; index < 75; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 150; ++index)
{
prd = acadoWorkspace.y[index + 75];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 10 */
real_t tmpDy[ 10 ];

/** Row vector of size: 7 */
real_t tmpDyN[ 7 ];

for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 6];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 6 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 6 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 6 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 6 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[lRun1 * 6 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.u[lRun1 * 3];
acadoWorkspace.objValueIn[7] = acadoVariables.u[lRun1 * 3 + 1];
acadoWorkspace.objValueIn[8] = acadoVariables.u[lRun1 * 3 + 2];
acadoWorkspace.objValueIn[9] = acadoVariables.od[lRun1 * 3];
acadoWorkspace.objValueIn[10] = acadoVariables.od[lRun1 * 3 + 1];
acadoWorkspace.objValueIn[11] = acadoVariables.od[lRun1 * 3 + 2];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 10] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 10];
acadoWorkspace.Dy[lRun1 * 10 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 10 + 1];
acadoWorkspace.Dy[lRun1 * 10 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 10 + 2];
acadoWorkspace.Dy[lRun1 * 10 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 10 + 3];
acadoWorkspace.Dy[lRun1 * 10 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 10 + 4];
acadoWorkspace.Dy[lRun1 * 10 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 10 + 5];
acadoWorkspace.Dy[lRun1 * 10 + 6] = acadoWorkspace.objValueOut[6] - acadoVariables.y[lRun1 * 10 + 6];
acadoWorkspace.Dy[lRun1 * 10 + 7] = acadoWorkspace.objValueOut[7] - acadoVariables.y[lRun1 * 10 + 7];
acadoWorkspace.Dy[lRun1 * 10 + 8] = acadoWorkspace.objValueOut[8] - acadoVariables.y[lRun1 * 10 + 8];
acadoWorkspace.Dy[lRun1 * 10 + 9] = acadoWorkspace.objValueOut[9] - acadoVariables.y[lRun1 * 10 + 9];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[150];
acadoWorkspace.objValueIn[1] = acadoVariables.x[151];
acadoWorkspace.objValueIn[2] = acadoVariables.x[152];
acadoWorkspace.objValueIn[3] = acadoVariables.x[153];
acadoWorkspace.objValueIn[4] = acadoVariables.x[154];
acadoWorkspace.objValueIn[5] = acadoVariables.x[155];
acadoWorkspace.objValueIn[6] = acadoVariables.od[75];
acadoWorkspace.objValueIn[7] = acadoVariables.od[76];
acadoWorkspace.objValueIn[8] = acadoVariables.od[77];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4] - acadoVariables.yN[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5] - acadoVariables.yN[5];
acadoWorkspace.DyN[6] = acadoWorkspace.objValueOut[6] - acadoVariables.yN[6];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 10]*acadoVariables.W[0];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 10 + 1]*acadoVariables.W[11];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 10 + 2]*acadoVariables.W[22];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 10 + 3]*acadoVariables.W[33];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 10 + 4]*acadoVariables.W[44];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 10 + 5]*acadoVariables.W[55];
tmpDy[6] = + acadoWorkspace.Dy[lRun1 * 10 + 6]*acadoVariables.W[66];
tmpDy[7] = + acadoWorkspace.Dy[lRun1 * 10 + 7]*acadoVariables.W[77];
tmpDy[8] = + acadoWorkspace.Dy[lRun1 * 10 + 8]*acadoVariables.W[88];
tmpDy[9] = + acadoWorkspace.Dy[lRun1 * 10 + 9]*acadoVariables.W[99];
objVal += + acadoWorkspace.Dy[lRun1 * 10]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 10 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 10 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 10 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 10 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 10 + 5]*tmpDy[5] + acadoWorkspace.Dy[lRun1 * 10 + 6]*tmpDy[6] + acadoWorkspace.Dy[lRun1 * 10 + 7]*tmpDy[7] + acadoWorkspace.Dy[lRun1 * 10 + 8]*tmpDy[8] + acadoWorkspace.Dy[lRun1 * 10 + 9]*tmpDy[9];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[8];
tmpDyN[2] = + acadoWorkspace.DyN[2]*acadoVariables.WN[16];
tmpDyN[3] = + acadoWorkspace.DyN[3]*acadoVariables.WN[24];
tmpDyN[4] = + acadoWorkspace.DyN[4]*acadoVariables.WN[32];
tmpDyN[5] = + acadoWorkspace.DyN[5]*acadoVariables.WN[40];
tmpDyN[6] = + acadoWorkspace.DyN[6]*acadoVariables.WN[48];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2] + acadoWorkspace.DyN[3]*tmpDyN[3] + acadoWorkspace.DyN[4]*tmpDyN[4] + acadoWorkspace.DyN[5]*tmpDyN[5] + acadoWorkspace.DyN[6]*tmpDyN[6];

objVal *= 0.5;
return objVal;
}

