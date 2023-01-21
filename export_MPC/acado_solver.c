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

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = u[0];
out[7] = u[1];
out[8] = u[2];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
}

void acado_setObjQ1Q2( real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = +tmpObjS[0];
tmpQ2[1] = +tmpObjS[1];
tmpQ2[2] = +tmpObjS[2];
tmpQ2[3] = +tmpObjS[3];
tmpQ2[4] = +tmpObjS[4];
tmpQ2[5] = +tmpObjS[5];
tmpQ2[6] = +tmpObjS[6];
tmpQ2[7] = +tmpObjS[7];
tmpQ2[8] = +tmpObjS[8];
tmpQ2[9] = +tmpObjS[9];
tmpQ2[10] = +tmpObjS[10];
tmpQ2[11] = +tmpObjS[11];
tmpQ2[12] = +tmpObjS[12];
tmpQ2[13] = +tmpObjS[13];
tmpQ2[14] = +tmpObjS[14];
tmpQ2[15] = +tmpObjS[15];
tmpQ2[16] = +tmpObjS[16];
tmpQ2[17] = +tmpObjS[17];
tmpQ2[18] = +tmpObjS[18];
tmpQ2[19] = +tmpObjS[19];
tmpQ2[20] = +tmpObjS[20];
tmpQ2[21] = +tmpObjS[21];
tmpQ2[22] = +tmpObjS[22];
tmpQ2[23] = +tmpObjS[23];
tmpQ2[24] = +tmpObjS[24];
tmpQ2[25] = +tmpObjS[25];
tmpQ2[26] = +tmpObjS[26];
tmpQ2[27] = +tmpObjS[27];
tmpQ2[28] = +tmpObjS[28];
tmpQ2[29] = +tmpObjS[29];
tmpQ2[30] = +tmpObjS[30];
tmpQ2[31] = +tmpObjS[31];
tmpQ2[32] = +tmpObjS[32];
tmpQ2[33] = +tmpObjS[33];
tmpQ2[34] = +tmpObjS[34];
tmpQ2[35] = +tmpObjS[35];
tmpQ2[36] = +tmpObjS[36];
tmpQ2[37] = +tmpObjS[37];
tmpQ2[38] = +tmpObjS[38];
tmpQ2[39] = +tmpObjS[39];
tmpQ2[40] = +tmpObjS[40];
tmpQ2[41] = +tmpObjS[41];
tmpQ2[42] = +tmpObjS[42];
tmpQ2[43] = +tmpObjS[43];
tmpQ2[44] = +tmpObjS[44];
tmpQ2[45] = +tmpObjS[45];
tmpQ2[46] = +tmpObjS[46];
tmpQ2[47] = +tmpObjS[47];
tmpQ2[48] = +tmpObjS[48];
tmpQ2[49] = +tmpObjS[49];
tmpQ2[50] = +tmpObjS[50];
tmpQ2[51] = +tmpObjS[51];
tmpQ2[52] = +tmpObjS[52];
tmpQ2[53] = +tmpObjS[53];
tmpQ1[0] = + tmpQ2[0];
tmpQ1[1] = + tmpQ2[1];
tmpQ1[2] = + tmpQ2[2];
tmpQ1[3] = + tmpQ2[3];
tmpQ1[4] = + tmpQ2[4];
tmpQ1[5] = + tmpQ2[5];
tmpQ1[6] = + tmpQ2[9];
tmpQ1[7] = + tmpQ2[10];
tmpQ1[8] = + tmpQ2[11];
tmpQ1[9] = + tmpQ2[12];
tmpQ1[10] = + tmpQ2[13];
tmpQ1[11] = + tmpQ2[14];
tmpQ1[12] = + tmpQ2[18];
tmpQ1[13] = + tmpQ2[19];
tmpQ1[14] = + tmpQ2[20];
tmpQ1[15] = + tmpQ2[21];
tmpQ1[16] = + tmpQ2[22];
tmpQ1[17] = + tmpQ2[23];
tmpQ1[18] = + tmpQ2[27];
tmpQ1[19] = + tmpQ2[28];
tmpQ1[20] = + tmpQ2[29];
tmpQ1[21] = + tmpQ2[30];
tmpQ1[22] = + tmpQ2[31];
tmpQ1[23] = + tmpQ2[32];
tmpQ1[24] = + tmpQ2[36];
tmpQ1[25] = + tmpQ2[37];
tmpQ1[26] = + tmpQ2[38];
tmpQ1[27] = + tmpQ2[39];
tmpQ1[28] = + tmpQ2[40];
tmpQ1[29] = + tmpQ2[41];
tmpQ1[30] = + tmpQ2[45];
tmpQ1[31] = + tmpQ2[46];
tmpQ1[32] = + tmpQ2[47];
tmpQ1[33] = + tmpQ2[48];
tmpQ1[34] = + tmpQ2[49];
tmpQ1[35] = + tmpQ2[50];
}

void acado_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[54];
tmpR2[1] = +tmpObjS[55];
tmpR2[2] = +tmpObjS[56];
tmpR2[3] = +tmpObjS[57];
tmpR2[4] = +tmpObjS[58];
tmpR2[5] = +tmpObjS[59];
tmpR2[6] = +tmpObjS[60];
tmpR2[7] = +tmpObjS[61];
tmpR2[8] = +tmpObjS[62];
tmpR2[9] = +tmpObjS[63];
tmpR2[10] = +tmpObjS[64];
tmpR2[11] = +tmpObjS[65];
tmpR2[12] = +tmpObjS[66];
tmpR2[13] = +tmpObjS[67];
tmpR2[14] = +tmpObjS[68];
tmpR2[15] = +tmpObjS[69];
tmpR2[16] = +tmpObjS[70];
tmpR2[17] = +tmpObjS[71];
tmpR2[18] = +tmpObjS[72];
tmpR2[19] = +tmpObjS[73];
tmpR2[20] = +tmpObjS[74];
tmpR2[21] = +tmpObjS[75];
tmpR2[22] = +tmpObjS[76];
tmpR2[23] = +tmpObjS[77];
tmpR2[24] = +tmpObjS[78];
tmpR2[25] = +tmpObjS[79];
tmpR2[26] = +tmpObjS[80];
tmpR1[0] = + tmpR2[6];
tmpR1[1] = + tmpR2[7];
tmpR1[2] = + tmpR2[8];
tmpR1[3] = + tmpR2[15];
tmpR1[4] = + tmpR2[16];
tmpR1[5] = + tmpR2[17];
tmpR1[6] = + tmpR2[24];
tmpR1[7] = + tmpR2[25];
tmpR1[8] = + tmpR2[26];
}

void acado_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[0];
tmpQN2[1] = +tmpObjSEndTerm[1];
tmpQN2[2] = +tmpObjSEndTerm[2];
tmpQN2[3] = +tmpObjSEndTerm[3];
tmpQN2[4] = +tmpObjSEndTerm[4];
tmpQN2[5] = +tmpObjSEndTerm[5];
tmpQN2[6] = +tmpObjSEndTerm[6];
tmpQN2[7] = +tmpObjSEndTerm[7];
tmpQN2[8] = +tmpObjSEndTerm[8];
tmpQN2[9] = +tmpObjSEndTerm[9];
tmpQN2[10] = +tmpObjSEndTerm[10];
tmpQN2[11] = +tmpObjSEndTerm[11];
tmpQN2[12] = +tmpObjSEndTerm[12];
tmpQN2[13] = +tmpObjSEndTerm[13];
tmpQN2[14] = +tmpObjSEndTerm[14];
tmpQN2[15] = +tmpObjSEndTerm[15];
tmpQN2[16] = +tmpObjSEndTerm[16];
tmpQN2[17] = +tmpObjSEndTerm[17];
tmpQN2[18] = +tmpObjSEndTerm[18];
tmpQN2[19] = +tmpObjSEndTerm[19];
tmpQN2[20] = +tmpObjSEndTerm[20];
tmpQN2[21] = +tmpObjSEndTerm[21];
tmpQN2[22] = +tmpObjSEndTerm[22];
tmpQN2[23] = +tmpObjSEndTerm[23];
tmpQN2[24] = +tmpObjSEndTerm[24];
tmpQN2[25] = +tmpObjSEndTerm[25];
tmpQN2[26] = +tmpObjSEndTerm[26];
tmpQN2[27] = +tmpObjSEndTerm[27];
tmpQN2[28] = +tmpObjSEndTerm[28];
tmpQN2[29] = +tmpObjSEndTerm[29];
tmpQN2[30] = +tmpObjSEndTerm[30];
tmpQN2[31] = +tmpObjSEndTerm[31];
tmpQN2[32] = +tmpObjSEndTerm[32];
tmpQN2[33] = +tmpObjSEndTerm[33];
tmpQN2[34] = +tmpObjSEndTerm[34];
tmpQN2[35] = +tmpObjSEndTerm[35];
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = + tmpQN2[2];
tmpQN1[3] = + tmpQN2[3];
tmpQN1[4] = + tmpQN2[4];
tmpQN1[5] = + tmpQN2[5];
tmpQN1[6] = + tmpQN2[6];
tmpQN1[7] = + tmpQN2[7];
tmpQN1[8] = + tmpQN2[8];
tmpQN1[9] = + tmpQN2[9];
tmpQN1[10] = + tmpQN2[10];
tmpQN1[11] = + tmpQN2[11];
tmpQN1[12] = + tmpQN2[12];
tmpQN1[13] = + tmpQN2[13];
tmpQN1[14] = + tmpQN2[14];
tmpQN1[15] = + tmpQN2[15];
tmpQN1[16] = + tmpQN2[16];
tmpQN1[17] = + tmpQN2[17];
tmpQN1[18] = + tmpQN2[18];
tmpQN1[19] = + tmpQN2[19];
tmpQN1[20] = + tmpQN2[20];
tmpQN1[21] = + tmpQN2[21];
tmpQN1[22] = + tmpQN2[22];
tmpQN1[23] = + tmpQN2[23];
tmpQN1[24] = + tmpQN2[24];
tmpQN1[25] = + tmpQN2[25];
tmpQN1[26] = + tmpQN2[26];
tmpQN1[27] = + tmpQN2[27];
tmpQN1[28] = + tmpQN2[28];
tmpQN1[29] = + tmpQN2[29];
tmpQN1[30] = + tmpQN2[30];
tmpQN1[31] = + tmpQN2[31];
tmpQN1[32] = + tmpQN2[32];
tmpQN1[33] = + tmpQN2[33];
tmpQN1[34] = + tmpQN2[34];
tmpQN1[35] = + tmpQN2[35];
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

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 9] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 9 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 9 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 9 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 9 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 9 + 5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.Dy[runObj * 9 + 6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.Dy[runObj * 9 + 7] = acadoWorkspace.objValueOut[7];
acadoWorkspace.Dy[runObj * 9 + 8] = acadoWorkspace.objValueOut[8];

acado_setObjQ1Q2( acadoVariables.W, &(acadoWorkspace.Q1[ runObj * 36 ]), &(acadoWorkspace.Q2[ runObj * 54 ]) );

acado_setObjR1R2( acadoVariables.W, &(acadoWorkspace.R1[ runObj * 9 ]), &(acadoWorkspace.R2[ runObj * 27 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[150];
acadoWorkspace.objValueIn[1] = acadoVariables.x[151];
acadoWorkspace.objValueIn[2] = acadoVariables.x[152];
acadoWorkspace.objValueIn[3] = acadoVariables.x[153];
acadoWorkspace.objValueIn[4] = acadoVariables.x[154];
acadoWorkspace.objValueIn[5] = acadoVariables.x[155];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

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
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6] + R2[7]*Dy1[7] + R2[8]*Dy1[8];
RDy1[1] = + R2[9]*Dy1[0] + R2[10]*Dy1[1] + R2[11]*Dy1[2] + R2[12]*Dy1[3] + R2[13]*Dy1[4] + R2[14]*Dy1[5] + R2[15]*Dy1[6] + R2[16]*Dy1[7] + R2[17]*Dy1[8];
RDy1[2] = + R2[18]*Dy1[0] + R2[19]*Dy1[1] + R2[20]*Dy1[2] + R2[21]*Dy1[3] + R2[22]*Dy1[4] + R2[23]*Dy1[5] + R2[24]*Dy1[6] + R2[25]*Dy1[7] + R2[26]*Dy1[8];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6] + Q2[7]*Dy1[7] + Q2[8]*Dy1[8];
QDy1[1] = + Q2[9]*Dy1[0] + Q2[10]*Dy1[1] + Q2[11]*Dy1[2] + Q2[12]*Dy1[3] + Q2[13]*Dy1[4] + Q2[14]*Dy1[5] + Q2[15]*Dy1[6] + Q2[16]*Dy1[7] + Q2[17]*Dy1[8];
QDy1[2] = + Q2[18]*Dy1[0] + Q2[19]*Dy1[1] + Q2[20]*Dy1[2] + Q2[21]*Dy1[3] + Q2[22]*Dy1[4] + Q2[23]*Dy1[5] + Q2[24]*Dy1[6] + Q2[25]*Dy1[7] + Q2[26]*Dy1[8];
QDy1[3] = + Q2[27]*Dy1[0] + Q2[28]*Dy1[1] + Q2[29]*Dy1[2] + Q2[30]*Dy1[3] + Q2[31]*Dy1[4] + Q2[32]*Dy1[5] + Q2[33]*Dy1[6] + Q2[34]*Dy1[7] + Q2[35]*Dy1[8];
QDy1[4] = + Q2[36]*Dy1[0] + Q2[37]*Dy1[1] + Q2[38]*Dy1[2] + Q2[39]*Dy1[3] + Q2[40]*Dy1[4] + Q2[41]*Dy1[5] + Q2[42]*Dy1[6] + Q2[43]*Dy1[7] + Q2[44]*Dy1[8];
QDy1[5] = + Q2[45]*Dy1[0] + Q2[46]*Dy1[1] + Q2[47]*Dy1[2] + Q2[48]*Dy1[3] + Q2[49]*Dy1[4] + Q2[50]*Dy1[5] + Q2[51]*Dy1[6] + Q2[52]*Dy1[7] + Q2[53]*Dy1[8];
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
/** Row vector of size: 150 */
static const int xBoundIndices[ 150 ] = 
{ 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155 };
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

for (lRun1 = 0; lRun1 < 150; ++lRun1)
acadoWorkspace.sbar[lRun1 + 6] = acadoWorkspace.d[lRun1];

acadoWorkspace.lb[0] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[1];
acadoWorkspace.lb[2] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[2];
acadoWorkspace.lb[3] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[3];
acadoWorkspace.lb[4] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[6];
acadoWorkspace.lb[7] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[7];
acadoWorkspace.lb[8] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[8];
acadoWorkspace.lb[9] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[9];
acadoWorkspace.lb[10] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[10];
acadoWorkspace.lb[11] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[11];
acadoWorkspace.lb[12] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[12];
acadoWorkspace.lb[13] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[13];
acadoWorkspace.lb[14] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[14];
acadoWorkspace.lb[15] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[15];
acadoWorkspace.lb[16] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[16];
acadoWorkspace.lb[17] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[17];
acadoWorkspace.lb[18] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[18];
acadoWorkspace.lb[19] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[19];
acadoWorkspace.lb[20] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[20];
acadoWorkspace.lb[21] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[21];
acadoWorkspace.lb[22] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[22];
acadoWorkspace.lb[23] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[23];
acadoWorkspace.lb[24] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[24];
acadoWorkspace.lb[25] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[25];
acadoWorkspace.lb[26] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[26];
acadoWorkspace.lb[27] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[27];
acadoWorkspace.lb[28] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[28];
acadoWorkspace.lb[29] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[29];
acadoWorkspace.lb[30] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[30];
acadoWorkspace.lb[31] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[31];
acadoWorkspace.lb[32] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[32];
acadoWorkspace.lb[33] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[33];
acadoWorkspace.lb[34] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[34];
acadoWorkspace.lb[35] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[35];
acadoWorkspace.lb[36] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[36];
acadoWorkspace.lb[37] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[37];
acadoWorkspace.lb[38] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[38];
acadoWorkspace.lb[39] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[39];
acadoWorkspace.lb[40] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[40];
acadoWorkspace.lb[41] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[41];
acadoWorkspace.lb[42] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[42];
acadoWorkspace.lb[43] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[43];
acadoWorkspace.lb[44] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[44];
acadoWorkspace.lb[45] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[45];
acadoWorkspace.lb[46] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[46];
acadoWorkspace.lb[47] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[47];
acadoWorkspace.lb[48] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[48];
acadoWorkspace.lb[49] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[49];
acadoWorkspace.lb[50] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[50];
acadoWorkspace.lb[51] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[51];
acadoWorkspace.lb[52] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[52];
acadoWorkspace.lb[53] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[53];
acadoWorkspace.lb[54] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[54];
acadoWorkspace.lb[55] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[55];
acadoWorkspace.lb[56] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[56];
acadoWorkspace.lb[57] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[57];
acadoWorkspace.lb[58] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[58];
acadoWorkspace.lb[59] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[59];
acadoWorkspace.lb[60] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[60];
acadoWorkspace.lb[61] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[61];
acadoWorkspace.lb[62] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[62];
acadoWorkspace.lb[63] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[63];
acadoWorkspace.lb[64] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[64];
acadoWorkspace.lb[65] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[65];
acadoWorkspace.lb[66] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[66];
acadoWorkspace.lb[67] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[67];
acadoWorkspace.lb[68] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[68];
acadoWorkspace.lb[69] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[69];
acadoWorkspace.lb[70] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[70];
acadoWorkspace.lb[71] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[71];
acadoWorkspace.lb[72] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[72];
acadoWorkspace.lb[73] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[73];
acadoWorkspace.lb[74] = (real_t)-1.0000000000000000e+02 - acadoVariables.u[74];
acadoWorkspace.ub[0] = (real_t)1.0000000000000000e+04 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)1.0000000000000000e+02 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)1.0000000000000000e+02 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)1.0000000000000000e+04 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)1.0000000000000000e+02 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)1.0000000000000000e+02 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)1.0000000000000000e+04 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)1.0000000000000000e+02 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)1.0000000000000000e+02 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)1.0000000000000000e+04 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)1.0000000000000000e+02 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)1.0000000000000000e+02 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)1.0000000000000000e+04 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)1.0000000000000000e+02 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)1.0000000000000000e+02 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)1.0000000000000000e+04 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)1.0000000000000000e+02 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)1.0000000000000000e+02 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)1.0000000000000000e+04 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)1.0000000000000000e+02 - acadoVariables.u[19];
acadoWorkspace.ub[20] = (real_t)1.0000000000000000e+02 - acadoVariables.u[20];
acadoWorkspace.ub[21] = (real_t)1.0000000000000000e+04 - acadoVariables.u[21];
acadoWorkspace.ub[22] = (real_t)1.0000000000000000e+02 - acadoVariables.u[22];
acadoWorkspace.ub[23] = (real_t)1.0000000000000000e+02 - acadoVariables.u[23];
acadoWorkspace.ub[24] = (real_t)1.0000000000000000e+04 - acadoVariables.u[24];
acadoWorkspace.ub[25] = (real_t)1.0000000000000000e+02 - acadoVariables.u[25];
acadoWorkspace.ub[26] = (real_t)1.0000000000000000e+02 - acadoVariables.u[26];
acadoWorkspace.ub[27] = (real_t)1.0000000000000000e+04 - acadoVariables.u[27];
acadoWorkspace.ub[28] = (real_t)1.0000000000000000e+02 - acadoVariables.u[28];
acadoWorkspace.ub[29] = (real_t)1.0000000000000000e+02 - acadoVariables.u[29];
acadoWorkspace.ub[30] = (real_t)1.0000000000000000e+04 - acadoVariables.u[30];
acadoWorkspace.ub[31] = (real_t)1.0000000000000000e+02 - acadoVariables.u[31];
acadoWorkspace.ub[32] = (real_t)1.0000000000000000e+02 - acadoVariables.u[32];
acadoWorkspace.ub[33] = (real_t)1.0000000000000000e+04 - acadoVariables.u[33];
acadoWorkspace.ub[34] = (real_t)1.0000000000000000e+02 - acadoVariables.u[34];
acadoWorkspace.ub[35] = (real_t)1.0000000000000000e+02 - acadoVariables.u[35];
acadoWorkspace.ub[36] = (real_t)1.0000000000000000e+04 - acadoVariables.u[36];
acadoWorkspace.ub[37] = (real_t)1.0000000000000000e+02 - acadoVariables.u[37];
acadoWorkspace.ub[38] = (real_t)1.0000000000000000e+02 - acadoVariables.u[38];
acadoWorkspace.ub[39] = (real_t)1.0000000000000000e+04 - acadoVariables.u[39];
acadoWorkspace.ub[40] = (real_t)1.0000000000000000e+02 - acadoVariables.u[40];
acadoWorkspace.ub[41] = (real_t)1.0000000000000000e+02 - acadoVariables.u[41];
acadoWorkspace.ub[42] = (real_t)1.0000000000000000e+04 - acadoVariables.u[42];
acadoWorkspace.ub[43] = (real_t)1.0000000000000000e+02 - acadoVariables.u[43];
acadoWorkspace.ub[44] = (real_t)1.0000000000000000e+02 - acadoVariables.u[44];
acadoWorkspace.ub[45] = (real_t)1.0000000000000000e+04 - acadoVariables.u[45];
acadoWorkspace.ub[46] = (real_t)1.0000000000000000e+02 - acadoVariables.u[46];
acadoWorkspace.ub[47] = (real_t)1.0000000000000000e+02 - acadoVariables.u[47];
acadoWorkspace.ub[48] = (real_t)1.0000000000000000e+04 - acadoVariables.u[48];
acadoWorkspace.ub[49] = (real_t)1.0000000000000000e+02 - acadoVariables.u[49];
acadoWorkspace.ub[50] = (real_t)1.0000000000000000e+02 - acadoVariables.u[50];
acadoWorkspace.ub[51] = (real_t)1.0000000000000000e+04 - acadoVariables.u[51];
acadoWorkspace.ub[52] = (real_t)1.0000000000000000e+02 - acadoVariables.u[52];
acadoWorkspace.ub[53] = (real_t)1.0000000000000000e+02 - acadoVariables.u[53];
acadoWorkspace.ub[54] = (real_t)1.0000000000000000e+04 - acadoVariables.u[54];
acadoWorkspace.ub[55] = (real_t)1.0000000000000000e+02 - acadoVariables.u[55];
acadoWorkspace.ub[56] = (real_t)1.0000000000000000e+02 - acadoVariables.u[56];
acadoWorkspace.ub[57] = (real_t)1.0000000000000000e+04 - acadoVariables.u[57];
acadoWorkspace.ub[58] = (real_t)1.0000000000000000e+02 - acadoVariables.u[58];
acadoWorkspace.ub[59] = (real_t)1.0000000000000000e+02 - acadoVariables.u[59];
acadoWorkspace.ub[60] = (real_t)1.0000000000000000e+04 - acadoVariables.u[60];
acadoWorkspace.ub[61] = (real_t)1.0000000000000000e+02 - acadoVariables.u[61];
acadoWorkspace.ub[62] = (real_t)1.0000000000000000e+02 - acadoVariables.u[62];
acadoWorkspace.ub[63] = (real_t)1.0000000000000000e+04 - acadoVariables.u[63];
acadoWorkspace.ub[64] = (real_t)1.0000000000000000e+02 - acadoVariables.u[64];
acadoWorkspace.ub[65] = (real_t)1.0000000000000000e+02 - acadoVariables.u[65];
acadoWorkspace.ub[66] = (real_t)1.0000000000000000e+04 - acadoVariables.u[66];
acadoWorkspace.ub[67] = (real_t)1.0000000000000000e+02 - acadoVariables.u[67];
acadoWorkspace.ub[68] = (real_t)1.0000000000000000e+02 - acadoVariables.u[68];
acadoWorkspace.ub[69] = (real_t)1.0000000000000000e+04 - acadoVariables.u[69];
acadoWorkspace.ub[70] = (real_t)1.0000000000000000e+02 - acadoVariables.u[70];
acadoWorkspace.ub[71] = (real_t)1.0000000000000000e+02 - acadoVariables.u[71];
acadoWorkspace.ub[72] = (real_t)1.0000000000000000e+04 - acadoVariables.u[72];
acadoWorkspace.ub[73] = (real_t)1.0000000000000000e+02 - acadoVariables.u[73];
acadoWorkspace.ub[74] = (real_t)1.0000000000000000e+02 - acadoVariables.u[74];

for (lRun1 = 0; lRun1 < 150; ++lRun1)
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
for (lRun1 = 0; lRun1 < 225; ++lRun1)
acadoWorkspace.Dy[lRun1] -= acadoVariables.y[lRun1];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];
acadoWorkspace.DyN[3] -= acadoVariables.yN[3];
acadoWorkspace.DyN[4] -= acadoVariables.yN[4];
acadoWorkspace.DyN[5] -= acadoVariables.yN[5];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 27 ]), &(acadoWorkspace.Dy[ 9 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 54 ]), &(acadoWorkspace.Dy[ 18 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 81 ]), &(acadoWorkspace.Dy[ 27 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 108 ]), &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 135 ]), &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 162 ]), &(acadoWorkspace.Dy[ 54 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 189 ]), &(acadoWorkspace.Dy[ 63 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 216 ]), &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 243 ]), &(acadoWorkspace.Dy[ 81 ]), &(acadoWorkspace.g[ 27 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 270 ]), &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 297 ]), &(acadoWorkspace.Dy[ 99 ]), &(acadoWorkspace.g[ 33 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 324 ]), &(acadoWorkspace.Dy[ 108 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 351 ]), &(acadoWorkspace.Dy[ 117 ]), &(acadoWorkspace.g[ 39 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 378 ]), &(acadoWorkspace.Dy[ 126 ]), &(acadoWorkspace.g[ 42 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 405 ]), &(acadoWorkspace.Dy[ 135 ]), &(acadoWorkspace.g[ 45 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 432 ]), &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 459 ]), &(acadoWorkspace.Dy[ 153 ]), &(acadoWorkspace.g[ 51 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 486 ]), &(acadoWorkspace.Dy[ 162 ]), &(acadoWorkspace.g[ 54 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 513 ]), &(acadoWorkspace.Dy[ 171 ]), &(acadoWorkspace.g[ 57 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 540 ]), &(acadoWorkspace.Dy[ 180 ]), &(acadoWorkspace.g[ 60 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 567 ]), &(acadoWorkspace.Dy[ 189 ]), &(acadoWorkspace.g[ 63 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 594 ]), &(acadoWorkspace.Dy[ 198 ]), &(acadoWorkspace.g[ 66 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 621 ]), &(acadoWorkspace.Dy[ 207 ]), &(acadoWorkspace.g[ 69 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 648 ]), &(acadoWorkspace.Dy[ 216 ]), &(acadoWorkspace.g[ 72 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 54 ]), &(acadoWorkspace.Dy[ 9 ]), &(acadoWorkspace.QDy[ 6 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 108 ]), &(acadoWorkspace.Dy[ 18 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 162 ]), &(acadoWorkspace.Dy[ 27 ]), &(acadoWorkspace.QDy[ 18 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 216 ]), &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 270 ]), &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.QDy[ 30 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 324 ]), &(acadoWorkspace.Dy[ 54 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 378 ]), &(acadoWorkspace.Dy[ 63 ]), &(acadoWorkspace.QDy[ 42 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 432 ]), &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.QDy[ 48 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 486 ]), &(acadoWorkspace.Dy[ 81 ]), &(acadoWorkspace.QDy[ 54 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 540 ]), &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.QDy[ 60 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 594 ]), &(acadoWorkspace.Dy[ 99 ]), &(acadoWorkspace.QDy[ 66 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 648 ]), &(acadoWorkspace.Dy[ 108 ]), &(acadoWorkspace.QDy[ 72 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 702 ]), &(acadoWorkspace.Dy[ 117 ]), &(acadoWorkspace.QDy[ 78 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 756 ]), &(acadoWorkspace.Dy[ 126 ]), &(acadoWorkspace.QDy[ 84 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 810 ]), &(acadoWorkspace.Dy[ 135 ]), &(acadoWorkspace.QDy[ 90 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 864 ]), &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.QDy[ 96 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 918 ]), &(acadoWorkspace.Dy[ 153 ]), &(acadoWorkspace.QDy[ 102 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 972 ]), &(acadoWorkspace.Dy[ 162 ]), &(acadoWorkspace.QDy[ 108 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1026 ]), &(acadoWorkspace.Dy[ 171 ]), &(acadoWorkspace.QDy[ 114 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1080 ]), &(acadoWorkspace.Dy[ 180 ]), &(acadoWorkspace.QDy[ 120 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1134 ]), &(acadoWorkspace.Dy[ 189 ]), &(acadoWorkspace.QDy[ 126 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1188 ]), &(acadoWorkspace.Dy[ 198 ]), &(acadoWorkspace.QDy[ 132 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1242 ]), &(acadoWorkspace.Dy[ 207 ]), &(acadoWorkspace.QDy[ 138 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1296 ]), &(acadoWorkspace.Dy[ 216 ]), &(acadoWorkspace.QDy[ 144 ]) );

acadoWorkspace.QDy[150] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[151] = + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[152] = + acadoWorkspace.QN2[12]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[13]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[14]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[15]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[16]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[17]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[153] = + acadoWorkspace.QN2[18]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[19]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[20]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[21]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[22]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[23]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[154] = + acadoWorkspace.QN2[24]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[25]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[26]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[27]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[28]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[29]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[155] = + acadoWorkspace.QN2[30]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[31]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[32]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[33]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[34]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[35]*acadoWorkspace.DyN[5];

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


tmp = acadoWorkspace.sbar[6] + acadoVariables.x[6];
acadoWorkspace.lbA[0] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[0] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[7] + acadoVariables.x[7];
acadoWorkspace.lbA[1] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[1] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[8] + acadoVariables.x[8];
acadoWorkspace.lbA[2] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[2] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[9] + acadoVariables.x[9];
acadoWorkspace.lbA[3] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[3] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[10] + acadoVariables.x[10];
acadoWorkspace.lbA[4] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[4] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[11] + acadoVariables.x[11];
acadoWorkspace.lbA[5] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[5] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[12] + acadoVariables.x[12];
acadoWorkspace.lbA[6] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[6] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[13] + acadoVariables.x[13];
acadoWorkspace.lbA[7] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[7] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[14] + acadoVariables.x[14];
acadoWorkspace.lbA[8] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[8] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[15] + acadoVariables.x[15];
acadoWorkspace.lbA[9] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[9] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[16] + acadoVariables.x[16];
acadoWorkspace.lbA[10] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[10] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[17] + acadoVariables.x[17];
acadoWorkspace.lbA[11] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[11] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[18] + acadoVariables.x[18];
acadoWorkspace.lbA[12] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[12] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[19] + acadoVariables.x[19];
acadoWorkspace.lbA[13] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[13] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[20] + acadoVariables.x[20];
acadoWorkspace.lbA[14] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[14] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[21] + acadoVariables.x[21];
acadoWorkspace.lbA[15] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[15] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[22] + acadoVariables.x[22];
acadoWorkspace.lbA[16] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[16] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[23] + acadoVariables.x[23];
acadoWorkspace.lbA[17] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[17] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[24] + acadoVariables.x[24];
acadoWorkspace.lbA[18] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[18] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[25] + acadoVariables.x[25];
acadoWorkspace.lbA[19] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[19] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[26] + acadoVariables.x[26];
acadoWorkspace.lbA[20] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[20] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[27] + acadoVariables.x[27];
acadoWorkspace.lbA[21] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[21] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[28] + acadoVariables.x[28];
acadoWorkspace.lbA[22] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[22] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[29] + acadoVariables.x[29];
acadoWorkspace.lbA[23] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[23] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[30] + acadoVariables.x[30];
acadoWorkspace.lbA[24] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[24] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[31] + acadoVariables.x[31];
acadoWorkspace.lbA[25] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[25] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[32] + acadoVariables.x[32];
acadoWorkspace.lbA[26] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[26] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[33] + acadoVariables.x[33];
acadoWorkspace.lbA[27] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[27] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[34] + acadoVariables.x[34];
acadoWorkspace.lbA[28] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[28] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[35] + acadoVariables.x[35];
acadoWorkspace.lbA[29] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[29] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[36] + acadoVariables.x[36];
acadoWorkspace.lbA[30] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[30] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[37] + acadoVariables.x[37];
acadoWorkspace.lbA[31] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[31] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[38] + acadoVariables.x[38];
acadoWorkspace.lbA[32] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[32] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[39] + acadoVariables.x[39];
acadoWorkspace.lbA[33] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[33] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[40] + acadoVariables.x[40];
acadoWorkspace.lbA[34] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[34] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[41] + acadoVariables.x[41];
acadoWorkspace.lbA[35] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[35] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[42] + acadoVariables.x[42];
acadoWorkspace.lbA[36] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[36] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[43] + acadoVariables.x[43];
acadoWorkspace.lbA[37] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[37] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[44] + acadoVariables.x[44];
acadoWorkspace.lbA[38] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[38] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[45] + acadoVariables.x[45];
acadoWorkspace.lbA[39] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[39] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[46] + acadoVariables.x[46];
acadoWorkspace.lbA[40] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[40] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[47] + acadoVariables.x[47];
acadoWorkspace.lbA[41] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[41] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[48] + acadoVariables.x[48];
acadoWorkspace.lbA[42] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[42] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[49] + acadoVariables.x[49];
acadoWorkspace.lbA[43] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[43] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[50] + acadoVariables.x[50];
acadoWorkspace.lbA[44] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[44] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[51] + acadoVariables.x[51];
acadoWorkspace.lbA[45] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[45] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[52] + acadoVariables.x[52];
acadoWorkspace.lbA[46] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[46] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[53] + acadoVariables.x[53];
acadoWorkspace.lbA[47] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[47] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[54] + acadoVariables.x[54];
acadoWorkspace.lbA[48] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[48] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[55] + acadoVariables.x[55];
acadoWorkspace.lbA[49] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[49] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[56] + acadoVariables.x[56];
acadoWorkspace.lbA[50] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[50] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[57] + acadoVariables.x[57];
acadoWorkspace.lbA[51] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[51] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[58] + acadoVariables.x[58];
acadoWorkspace.lbA[52] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[52] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[59] + acadoVariables.x[59];
acadoWorkspace.lbA[53] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[53] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[60] + acadoVariables.x[60];
acadoWorkspace.lbA[54] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[54] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[61] + acadoVariables.x[61];
acadoWorkspace.lbA[55] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[55] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[62] + acadoVariables.x[62];
acadoWorkspace.lbA[56] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[56] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[63] + acadoVariables.x[63];
acadoWorkspace.lbA[57] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[57] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[64] + acadoVariables.x[64];
acadoWorkspace.lbA[58] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[58] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[65] + acadoVariables.x[65];
acadoWorkspace.lbA[59] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[59] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[66] + acadoVariables.x[66];
acadoWorkspace.lbA[60] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[60] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[67] + acadoVariables.x[67];
acadoWorkspace.lbA[61] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[61] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[68] + acadoVariables.x[68];
acadoWorkspace.lbA[62] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[62] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[69] + acadoVariables.x[69];
acadoWorkspace.lbA[63] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[63] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[70] + acadoVariables.x[70];
acadoWorkspace.lbA[64] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[64] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[71] + acadoVariables.x[71];
acadoWorkspace.lbA[65] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[65] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[72] + acadoVariables.x[72];
acadoWorkspace.lbA[66] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[66] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[73] + acadoVariables.x[73];
acadoWorkspace.lbA[67] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[67] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[74] + acadoVariables.x[74];
acadoWorkspace.lbA[68] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[68] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[75] + acadoVariables.x[75];
acadoWorkspace.lbA[69] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[69] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[76] + acadoVariables.x[76];
acadoWorkspace.lbA[70] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[70] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[77] + acadoVariables.x[77];
acadoWorkspace.lbA[71] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[71] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[78] + acadoVariables.x[78];
acadoWorkspace.lbA[72] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[72] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[79] + acadoVariables.x[79];
acadoWorkspace.lbA[73] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[73] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[80] + acadoVariables.x[80];
acadoWorkspace.lbA[74] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[74] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[81] + acadoVariables.x[81];
acadoWorkspace.lbA[75] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[75] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[82] + acadoVariables.x[82];
acadoWorkspace.lbA[76] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[76] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[83] + acadoVariables.x[83];
acadoWorkspace.lbA[77] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[77] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[84] + acadoVariables.x[84];
acadoWorkspace.lbA[78] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[78] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[85] + acadoVariables.x[85];
acadoWorkspace.lbA[79] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[79] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[86] + acadoVariables.x[86];
acadoWorkspace.lbA[80] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[80] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[87] + acadoVariables.x[87];
acadoWorkspace.lbA[81] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[81] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[88] + acadoVariables.x[88];
acadoWorkspace.lbA[82] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[82] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[89] + acadoVariables.x[89];
acadoWorkspace.lbA[83] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[83] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[90] + acadoVariables.x[90];
acadoWorkspace.lbA[84] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[84] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[91] + acadoVariables.x[91];
acadoWorkspace.lbA[85] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[85] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[92] + acadoVariables.x[92];
acadoWorkspace.lbA[86] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[86] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[93] + acadoVariables.x[93];
acadoWorkspace.lbA[87] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[87] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[94] + acadoVariables.x[94];
acadoWorkspace.lbA[88] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[88] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[95] + acadoVariables.x[95];
acadoWorkspace.lbA[89] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[89] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[96] + acadoVariables.x[96];
acadoWorkspace.lbA[90] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[90] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[97] + acadoVariables.x[97];
acadoWorkspace.lbA[91] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[91] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[98] + acadoVariables.x[98];
acadoWorkspace.lbA[92] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[92] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[99] + acadoVariables.x[99];
acadoWorkspace.lbA[93] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[93] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[100] + acadoVariables.x[100];
acadoWorkspace.lbA[94] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[94] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[101] + acadoVariables.x[101];
acadoWorkspace.lbA[95] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[95] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[102] + acadoVariables.x[102];
acadoWorkspace.lbA[96] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[96] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[103] + acadoVariables.x[103];
acadoWorkspace.lbA[97] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[97] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[104] + acadoVariables.x[104];
acadoWorkspace.lbA[98] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[98] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[105] + acadoVariables.x[105];
acadoWorkspace.lbA[99] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[99] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[106] + acadoVariables.x[106];
acadoWorkspace.lbA[100] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[100] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[107] + acadoVariables.x[107];
acadoWorkspace.lbA[101] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[101] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[108] + acadoVariables.x[108];
acadoWorkspace.lbA[102] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[102] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[109] + acadoVariables.x[109];
acadoWorkspace.lbA[103] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[103] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[110] + acadoVariables.x[110];
acadoWorkspace.lbA[104] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[104] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[111] + acadoVariables.x[111];
acadoWorkspace.lbA[105] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[105] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[112] + acadoVariables.x[112];
acadoWorkspace.lbA[106] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[106] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[113] + acadoVariables.x[113];
acadoWorkspace.lbA[107] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[107] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[114] + acadoVariables.x[114];
acadoWorkspace.lbA[108] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[108] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[115] + acadoVariables.x[115];
acadoWorkspace.lbA[109] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[109] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[116] + acadoVariables.x[116];
acadoWorkspace.lbA[110] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[110] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[117] + acadoVariables.x[117];
acadoWorkspace.lbA[111] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[111] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[118] + acadoVariables.x[118];
acadoWorkspace.lbA[112] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[112] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[119] + acadoVariables.x[119];
acadoWorkspace.lbA[113] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[113] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[120] + acadoVariables.x[120];
acadoWorkspace.lbA[114] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[114] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[121] + acadoVariables.x[121];
acadoWorkspace.lbA[115] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[115] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[122] + acadoVariables.x[122];
acadoWorkspace.lbA[116] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[116] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[123] + acadoVariables.x[123];
acadoWorkspace.lbA[117] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[117] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[124] + acadoVariables.x[124];
acadoWorkspace.lbA[118] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[118] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[125] + acadoVariables.x[125];
acadoWorkspace.lbA[119] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[119] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[126] + acadoVariables.x[126];
acadoWorkspace.lbA[120] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[120] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[127] + acadoVariables.x[127];
acadoWorkspace.lbA[121] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[121] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[128] + acadoVariables.x[128];
acadoWorkspace.lbA[122] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[122] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[129] + acadoVariables.x[129];
acadoWorkspace.lbA[123] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[123] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[130] + acadoVariables.x[130];
acadoWorkspace.lbA[124] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[124] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[131] + acadoVariables.x[131];
acadoWorkspace.lbA[125] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[125] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[132] + acadoVariables.x[132];
acadoWorkspace.lbA[126] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[126] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[133] + acadoVariables.x[133];
acadoWorkspace.lbA[127] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[127] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[134] + acadoVariables.x[134];
acadoWorkspace.lbA[128] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[128] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[135] + acadoVariables.x[135];
acadoWorkspace.lbA[129] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[129] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[136] + acadoVariables.x[136];
acadoWorkspace.lbA[130] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[130] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[137] + acadoVariables.x[137];
acadoWorkspace.lbA[131] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[131] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[138] + acadoVariables.x[138];
acadoWorkspace.lbA[132] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[132] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[139] + acadoVariables.x[139];
acadoWorkspace.lbA[133] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[133] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[140] + acadoVariables.x[140];
acadoWorkspace.lbA[134] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[134] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[141] + acadoVariables.x[141];
acadoWorkspace.lbA[135] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[135] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[142] + acadoVariables.x[142];
acadoWorkspace.lbA[136] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[136] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[143] + acadoVariables.x[143];
acadoWorkspace.lbA[137] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[137] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[144] + acadoVariables.x[144];
acadoWorkspace.lbA[138] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[138] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[145] + acadoVariables.x[145];
acadoWorkspace.lbA[139] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[139] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[146] + acadoVariables.x[146];
acadoWorkspace.lbA[140] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[140] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[147] + acadoVariables.x[147];
acadoWorkspace.lbA[141] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[141] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[148] + acadoVariables.x[148];
acadoWorkspace.lbA[142] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[142] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[149] + acadoVariables.x[149];
acadoWorkspace.lbA[143] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[143] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[150] + acadoVariables.x[150];
acadoWorkspace.lbA[144] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[144] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[151] + acadoVariables.x[151];
acadoWorkspace.lbA[145] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[145] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[152] + acadoVariables.x[152];
acadoWorkspace.lbA[146] = (real_t)-6.2831853071795862e+00 - tmp;
acadoWorkspace.ubA[146] = (real_t)6.2831853071795862e+00 - tmp;
tmp = acadoWorkspace.sbar[153] + acadoVariables.x[153];
acadoWorkspace.lbA[147] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[147] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[154] + acadoVariables.x[154];
acadoWorkspace.lbA[148] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[148] = (real_t)1.0000000000000000e+01 - tmp;
tmp = acadoWorkspace.sbar[155] + acadoVariables.x[155];
acadoWorkspace.lbA[149] = (real_t)-1.0000000000000000e+01 - tmp;
acadoWorkspace.ubA[149] = (real_t)1.0000000000000000e+01 - tmp;

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
/** Row vector of size: 9 */
real_t tmpDy[ 9 ];

/** Row vector of size: 6 */
real_t tmpDyN[ 6 ];

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

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 9] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 9];
acadoWorkspace.Dy[lRun1 * 9 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 9 + 1];
acadoWorkspace.Dy[lRun1 * 9 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 9 + 2];
acadoWorkspace.Dy[lRun1 * 9 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 9 + 3];
acadoWorkspace.Dy[lRun1 * 9 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 9 + 4];
acadoWorkspace.Dy[lRun1 * 9 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 9 + 5];
acadoWorkspace.Dy[lRun1 * 9 + 6] = acadoWorkspace.objValueOut[6] - acadoVariables.y[lRun1 * 9 + 6];
acadoWorkspace.Dy[lRun1 * 9 + 7] = acadoWorkspace.objValueOut[7] - acadoVariables.y[lRun1 * 9 + 7];
acadoWorkspace.Dy[lRun1 * 9 + 8] = acadoWorkspace.objValueOut[8] - acadoVariables.y[lRun1 * 9 + 8];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[150];
acadoWorkspace.objValueIn[1] = acadoVariables.x[151];
acadoWorkspace.objValueIn[2] = acadoVariables.x[152];
acadoWorkspace.objValueIn[3] = acadoVariables.x[153];
acadoWorkspace.objValueIn[4] = acadoVariables.x[154];
acadoWorkspace.objValueIn[5] = acadoVariables.x[155];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4] - acadoVariables.yN[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5] - acadoVariables.yN[5];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 9]*acadoVariables.W[0];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 9 + 1]*acadoVariables.W[10];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 9 + 2]*acadoVariables.W[20];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 9 + 3]*acadoVariables.W[30];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 9 + 4]*acadoVariables.W[40];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 9 + 5]*acadoVariables.W[50];
tmpDy[6] = + acadoWorkspace.Dy[lRun1 * 9 + 6]*acadoVariables.W[60];
tmpDy[7] = + acadoWorkspace.Dy[lRun1 * 9 + 7]*acadoVariables.W[70];
tmpDy[8] = + acadoWorkspace.Dy[lRun1 * 9 + 8]*acadoVariables.W[80];
objVal += + acadoWorkspace.Dy[lRun1 * 9]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 9 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 9 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 9 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 9 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 9 + 5]*tmpDy[5] + acadoWorkspace.Dy[lRun1 * 9 + 6]*tmpDy[6] + acadoWorkspace.Dy[lRun1 * 9 + 7]*tmpDy[7] + acadoWorkspace.Dy[lRun1 * 9 + 8]*tmpDy[8];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[7];
tmpDyN[2] = + acadoWorkspace.DyN[2]*acadoVariables.WN[14];
tmpDyN[3] = + acadoWorkspace.DyN[3]*acadoVariables.WN[21];
tmpDyN[4] = + acadoWorkspace.DyN[4]*acadoVariables.WN[28];
tmpDyN[5] = + acadoWorkspace.DyN[5]*acadoVariables.WN[35];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2] + acadoWorkspace.DyN[3]*tmpDyN[3] + acadoWorkspace.DyN[4]*tmpDyN[4] + acadoWorkspace.DyN[5]*tmpDyN[5];

objVal *= 0.5;
return objVal;
}

