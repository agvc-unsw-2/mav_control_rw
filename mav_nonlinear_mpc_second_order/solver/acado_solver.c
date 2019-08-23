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


/** Row vector of size: 209 */
real_t state[ 209 ];

int acado_modelSimulation(  )
{
int ret;

int lRun1;
int lRun2;
ret = 0;
#pragma omp parallel for private(lRun1, state) shared(acadoWorkspace, acadoVariables)
for (lRun1 = 0; lRun1 < 5; ++lRun1)
{
state[0] = acadoVariables.x[lRun1 * 12];
state[1] = acadoVariables.x[lRun1 * 12 + 1];
state[2] = acadoVariables.x[lRun1 * 12 + 2];
state[3] = acadoVariables.x[lRun1 * 12 + 3];
state[4] = acadoVariables.x[lRun1 * 12 + 4];
state[5] = acadoVariables.x[lRun1 * 12 + 5];
state[6] = acadoVariables.x[lRun1 * 12 + 6];
state[7] = acadoVariables.x[lRun1 * 12 + 7];
state[8] = acadoVariables.x[lRun1 * 12 + 8];
state[9] = acadoVariables.x[lRun1 * 12 + 9];
state[10] = acadoVariables.x[lRun1 * 12 + 10];
state[11] = acadoVariables.x[lRun1 * 12 + 11];

state[192] = acadoVariables.u[lRun1 * 3];
state[193] = acadoVariables.u[lRun1 * 3 + 1];
state[194] = acadoVariables.u[lRun1 * 3 + 2];
state[195] = acadoVariables.od[lRun1 * 14];
state[196] = acadoVariables.od[lRun1 * 14 + 1];
state[197] = acadoVariables.od[lRun1 * 14 + 2];
state[198] = acadoVariables.od[lRun1 * 14 + 3];
state[199] = acadoVariables.od[lRun1 * 14 + 4];
state[200] = acadoVariables.od[lRun1 * 14 + 5];
state[201] = acadoVariables.od[lRun1 * 14 + 6];
state[202] = acadoVariables.od[lRun1 * 14 + 7];
state[203] = acadoVariables.od[lRun1 * 14 + 8];
state[204] = acadoVariables.od[lRun1 * 14 + 9];
state[205] = acadoVariables.od[lRun1 * 14 + 10];
state[206] = acadoVariables.od[lRun1 * 14 + 11];
state[207] = acadoVariables.od[lRun1 * 14 + 12];
state[208] = acadoVariables.od[lRun1 * 14 + 13];

ret = acado_integrate(state, 1);

acadoWorkspace.d[lRun1 * 12] = state[0] - acadoVariables.x[lRun1 * 12 + 12];
acadoWorkspace.d[lRun1 * 12 + 1] = state[1] - acadoVariables.x[lRun1 * 12 + 13];
acadoWorkspace.d[lRun1 * 12 + 2] = state[2] - acadoVariables.x[lRun1 * 12 + 14];
acadoWorkspace.d[lRun1 * 12 + 3] = state[3] - acadoVariables.x[lRun1 * 12 + 15];
acadoWorkspace.d[lRun1 * 12 + 4] = state[4] - acadoVariables.x[lRun1 * 12 + 16];
acadoWorkspace.d[lRun1 * 12 + 5] = state[5] - acadoVariables.x[lRun1 * 12 + 17];
acadoWorkspace.d[lRun1 * 12 + 6] = state[6] - acadoVariables.x[lRun1 * 12 + 18];
acadoWorkspace.d[lRun1 * 12 + 7] = state[7] - acadoVariables.x[lRun1 * 12 + 19];
acadoWorkspace.d[lRun1 * 12 + 8] = state[8] - acadoVariables.x[lRun1 * 12 + 20];
acadoWorkspace.d[lRun1 * 12 + 9] = state[9] - acadoVariables.x[lRun1 * 12 + 21];
acadoWorkspace.d[lRun1 * 12 + 10] = state[10] - acadoVariables.x[lRun1 * 12 + 22];
acadoWorkspace.d[lRun1 * 12 + 11] = state[11] - acadoVariables.x[lRun1 * 12 + 23];

for (lRun2 = 0; lRun2 < 144; ++lRun2)
acadoWorkspace.evGx[(0) + ((lRun2) + (lRun1 * 144))] = state[lRun2 + 12];


acadoWorkspace.evGu[lRun1 * 36] = state[156];
acadoWorkspace.evGu[lRun1 * 36 + 1] = state[157];
acadoWorkspace.evGu[lRun1 * 36 + 2] = state[158];
acadoWorkspace.evGu[lRun1 * 36 + 3] = state[159];
acadoWorkspace.evGu[lRun1 * 36 + 4] = state[160];
acadoWorkspace.evGu[lRun1 * 36 + 5] = state[161];
acadoWorkspace.evGu[lRun1 * 36 + 6] = state[162];
acadoWorkspace.evGu[lRun1 * 36 + 7] = state[163];
acadoWorkspace.evGu[lRun1 * 36 + 8] = state[164];
acadoWorkspace.evGu[lRun1 * 36 + 9] = state[165];
acadoWorkspace.evGu[lRun1 * 36 + 10] = state[166];
acadoWorkspace.evGu[lRun1 * 36 + 11] = state[167];
acadoWorkspace.evGu[lRun1 * 36 + 12] = state[168];
acadoWorkspace.evGu[lRun1 * 36 + 13] = state[169];
acadoWorkspace.evGu[lRun1 * 36 + 14] = state[170];
acadoWorkspace.evGu[lRun1 * 36 + 15] = state[171];
acadoWorkspace.evGu[lRun1 * 36 + 16] = state[172];
acadoWorkspace.evGu[lRun1 * 36 + 17] = state[173];
acadoWorkspace.evGu[lRun1 * 36 + 18] = state[174];
acadoWorkspace.evGu[lRun1 * 36 + 19] = state[175];
acadoWorkspace.evGu[lRun1 * 36 + 20] = state[176];
acadoWorkspace.evGu[lRun1 * 36 + 21] = state[177];
acadoWorkspace.evGu[lRun1 * 36 + 22] = state[178];
acadoWorkspace.evGu[lRun1 * 36 + 23] = state[179];
acadoWorkspace.evGu[lRun1 * 36 + 24] = state[180];
acadoWorkspace.evGu[lRun1 * 36 + 25] = state[181];
acadoWorkspace.evGu[lRun1 * 36 + 26] = state[182];
acadoWorkspace.evGu[lRun1 * 36 + 27] = state[183];
acadoWorkspace.evGu[lRun1 * 36 + 28] = state[184];
acadoWorkspace.evGu[lRun1 * 36 + 29] = state[185];
acadoWorkspace.evGu[lRun1 * 36 + 30] = state[186];
acadoWorkspace.evGu[lRun1 * 36 + 31] = state[187];
acadoWorkspace.evGu[lRun1 * 36 + 32] = state[188];
acadoWorkspace.evGu[lRun1 * 36 + 33] = state[189];
acadoWorkspace.evGu[lRun1 * 36 + 34] = state[190];
acadoWorkspace.evGu[lRun1 * 36 + 35] = state[191];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 12;
/* Vector of auxiliary variables; number of elements: 4. */
real_t* a = acadoWorkspace.objAuxVar;

/* Compute intermediate quantities: */
a[0] = (cos(xd[4]));
a[1] = (cos(xd[3]));
a[2] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[3])));
a[3] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[4])));

/* Compute outputs: */
out[0] = xd[6];
out[1] = xd[7];
out[2] = xd[8];
out[3] = xd[0];
out[4] = xd[1];
out[5] = xd[2];
out[6] = xd[3];
out[7] = xd[4];
out[8] = xd[9];
out[9] = xd[10];
out[10] = u[0];
out[11] = u[1];
out[12] = (((a[0]*a[1])*u[2])-(real_t)(9.8065999999999995e+00));
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = (real_t)(0.0000000000000000e+00);
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (real_t)(0.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(1.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(0.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(1.0000000000000000e+00);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(0.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = (real_t)(0.0000000000000000e+00);
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = (real_t)(0.0000000000000000e+00);
out[40] = (real_t)(0.0000000000000000e+00);
out[41] = (real_t)(0.0000000000000000e+00);
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = (real_t)(0.0000000000000000e+00);
out[44] = (real_t)(0.0000000000000000e+00);
out[45] = (real_t)(1.0000000000000000e+00);
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (real_t)(0.0000000000000000e+00);
out[49] = (real_t)(1.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(0.0000000000000000e+00);
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
out[62] = (real_t)(1.0000000000000000e+00);
out[63] = (real_t)(0.0000000000000000e+00);
out[64] = (real_t)(0.0000000000000000e+00);
out[65] = (real_t)(0.0000000000000000e+00);
out[66] = (real_t)(0.0000000000000000e+00);
out[67] = (real_t)(0.0000000000000000e+00);
out[68] = (real_t)(0.0000000000000000e+00);
out[69] = (real_t)(0.0000000000000000e+00);
out[70] = (real_t)(0.0000000000000000e+00);
out[71] = (real_t)(0.0000000000000000e+00);
out[72] = (real_t)(0.0000000000000000e+00);
out[73] = (real_t)(0.0000000000000000e+00);
out[74] = (real_t)(0.0000000000000000e+00);
out[75] = (real_t)(1.0000000000000000e+00);
out[76] = (real_t)(0.0000000000000000e+00);
out[77] = (real_t)(0.0000000000000000e+00);
out[78] = (real_t)(0.0000000000000000e+00);
out[79] = (real_t)(0.0000000000000000e+00);
out[80] = (real_t)(0.0000000000000000e+00);
out[81] = (real_t)(0.0000000000000000e+00);
out[82] = (real_t)(0.0000000000000000e+00);
out[83] = (real_t)(0.0000000000000000e+00);
out[84] = (real_t)(0.0000000000000000e+00);
out[85] = (real_t)(0.0000000000000000e+00);
out[86] = (real_t)(0.0000000000000000e+00);
out[87] = (real_t)(0.0000000000000000e+00);
out[88] = (real_t)(1.0000000000000000e+00);
out[89] = (real_t)(0.0000000000000000e+00);
out[90] = (real_t)(0.0000000000000000e+00);
out[91] = (real_t)(0.0000000000000000e+00);
out[92] = (real_t)(0.0000000000000000e+00);
out[93] = (real_t)(0.0000000000000000e+00);
out[94] = (real_t)(0.0000000000000000e+00);
out[95] = (real_t)(0.0000000000000000e+00);
out[96] = (real_t)(0.0000000000000000e+00);
out[97] = (real_t)(0.0000000000000000e+00);
out[98] = (real_t)(0.0000000000000000e+00);
out[99] = (real_t)(0.0000000000000000e+00);
out[100] = (real_t)(0.0000000000000000e+00);
out[101] = (real_t)(1.0000000000000000e+00);
out[102] = (real_t)(0.0000000000000000e+00);
out[103] = (real_t)(0.0000000000000000e+00);
out[104] = (real_t)(0.0000000000000000e+00);
out[105] = (real_t)(0.0000000000000000e+00);
out[106] = (real_t)(0.0000000000000000e+00);
out[107] = (real_t)(0.0000000000000000e+00);
out[108] = (real_t)(0.0000000000000000e+00);
out[109] = (real_t)(0.0000000000000000e+00);
out[110] = (real_t)(0.0000000000000000e+00);
out[111] = (real_t)(0.0000000000000000e+00);
out[112] = (real_t)(0.0000000000000000e+00);
out[113] = (real_t)(0.0000000000000000e+00);
out[114] = (real_t)(0.0000000000000000e+00);
out[115] = (real_t)(0.0000000000000000e+00);
out[116] = (real_t)(0.0000000000000000e+00);
out[117] = (real_t)(0.0000000000000000e+00);
out[118] = (real_t)(1.0000000000000000e+00);
out[119] = (real_t)(0.0000000000000000e+00);
out[120] = (real_t)(0.0000000000000000e+00);
out[121] = (real_t)(0.0000000000000000e+00);
out[122] = (real_t)(0.0000000000000000e+00);
out[123] = (real_t)(0.0000000000000000e+00);
out[124] = (real_t)(0.0000000000000000e+00);
out[125] = (real_t)(0.0000000000000000e+00);
out[126] = (real_t)(0.0000000000000000e+00);
out[127] = (real_t)(0.0000000000000000e+00);
out[128] = (real_t)(0.0000000000000000e+00);
out[129] = (real_t)(0.0000000000000000e+00);
out[130] = (real_t)(0.0000000000000000e+00);
out[131] = (real_t)(1.0000000000000000e+00);
out[132] = (real_t)(0.0000000000000000e+00);
out[133] = (real_t)(0.0000000000000000e+00);
out[134] = (real_t)(0.0000000000000000e+00);
out[135] = (real_t)(0.0000000000000000e+00);
out[136] = (real_t)(0.0000000000000000e+00);
out[137] = (real_t)(0.0000000000000000e+00);
out[138] = (real_t)(0.0000000000000000e+00);
out[139] = (real_t)(0.0000000000000000e+00);
out[140] = (real_t)(0.0000000000000000e+00);
out[141] = (real_t)(0.0000000000000000e+00);
out[142] = (real_t)(0.0000000000000000e+00);
out[143] = (real_t)(0.0000000000000000e+00);
out[144] = (real_t)(0.0000000000000000e+00);
out[145] = (real_t)(0.0000000000000000e+00);
out[146] = (real_t)(0.0000000000000000e+00);
out[147] = (real_t)(0.0000000000000000e+00);
out[148] = (real_t)(0.0000000000000000e+00);
out[149] = (real_t)(0.0000000000000000e+00);
out[150] = (real_t)(0.0000000000000000e+00);
out[151] = (real_t)(0.0000000000000000e+00);
out[152] = (real_t)(0.0000000000000000e+00);
out[153] = (real_t)(0.0000000000000000e+00);
out[154] = (real_t)(0.0000000000000000e+00);
out[155] = (real_t)(0.0000000000000000e+00);
out[156] = (real_t)(0.0000000000000000e+00);
out[157] = (real_t)(0.0000000000000000e+00);
out[158] = (real_t)(0.0000000000000000e+00);
out[159] = (real_t)(0.0000000000000000e+00);
out[160] = ((a[0]*a[2])*u[2]);
out[161] = ((a[3]*a[1])*u[2]);
out[162] = (real_t)(0.0000000000000000e+00);
out[163] = (real_t)(0.0000000000000000e+00);
out[164] = (real_t)(0.0000000000000000e+00);
out[165] = (real_t)(0.0000000000000000e+00);
out[166] = (real_t)(0.0000000000000000e+00);
out[167] = (real_t)(0.0000000000000000e+00);
out[168] = (real_t)(0.0000000000000000e+00);
out[169] = (real_t)(0.0000000000000000e+00);
out[170] = (real_t)(0.0000000000000000e+00);
out[171] = (real_t)(0.0000000000000000e+00);
out[172] = (real_t)(0.0000000000000000e+00);
out[173] = (real_t)(0.0000000000000000e+00);
out[174] = (real_t)(0.0000000000000000e+00);
out[175] = (real_t)(0.0000000000000000e+00);
out[176] = (real_t)(0.0000000000000000e+00);
out[177] = (real_t)(0.0000000000000000e+00);
out[178] = (real_t)(0.0000000000000000e+00);
out[179] = (real_t)(0.0000000000000000e+00);
out[180] = (real_t)(0.0000000000000000e+00);
out[181] = (real_t)(0.0000000000000000e+00);
out[182] = (real_t)(0.0000000000000000e+00);
out[183] = (real_t)(0.0000000000000000e+00);
out[184] = (real_t)(0.0000000000000000e+00);
out[185] = (real_t)(0.0000000000000000e+00);
out[186] = (real_t)(0.0000000000000000e+00);
out[187] = (real_t)(0.0000000000000000e+00);
out[188] = (real_t)(0.0000000000000000e+00);
out[189] = (real_t)(0.0000000000000000e+00);
out[190] = (real_t)(0.0000000000000000e+00);
out[191] = (real_t)(0.0000000000000000e+00);
out[192] = (real_t)(0.0000000000000000e+00);
out[193] = (real_t)(0.0000000000000000e+00);
out[194] = (real_t)(0.0000000000000000e+00);
out[195] = (real_t)(0.0000000000000000e+00);
out[196] = (real_t)(0.0000000000000000e+00);
out[197] = (real_t)(0.0000000000000000e+00);
out[198] = (real_t)(0.0000000000000000e+00);
out[199] = (real_t)(1.0000000000000000e+00);
out[200] = (real_t)(0.0000000000000000e+00);
out[201] = (real_t)(0.0000000000000000e+00);
out[202] = (real_t)(0.0000000000000000e+00);
out[203] = (real_t)(1.0000000000000000e+00);
out[204] = (real_t)(0.0000000000000000e+00);
out[205] = (real_t)(0.0000000000000000e+00);
out[206] = (real_t)(0.0000000000000000e+00);
out[207] = (a[0]*a[1]);
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[6];
out[1] = xd[7];
out[2] = xd[8];
out[3] = xd[0];
out[4] = xd[1];
out[5] = xd[2];
}

void acado_setObjQ1Q2( real_t* const tmpFx, real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = + tmpFx[0]*tmpObjS[0] + tmpFx[12]*tmpObjS[13] + tmpFx[24]*tmpObjS[26] + tmpFx[36]*tmpObjS[39] + tmpFx[48]*tmpObjS[52] + tmpFx[60]*tmpObjS[65] + tmpFx[72]*tmpObjS[78] + tmpFx[84]*tmpObjS[91] + tmpFx[96]*tmpObjS[104] + tmpFx[108]*tmpObjS[117] + tmpFx[120]*tmpObjS[130] + tmpFx[132]*tmpObjS[143] + tmpFx[144]*tmpObjS[156];
tmpQ2[1] = + tmpFx[0]*tmpObjS[1] + tmpFx[12]*tmpObjS[14] + tmpFx[24]*tmpObjS[27] + tmpFx[36]*tmpObjS[40] + tmpFx[48]*tmpObjS[53] + tmpFx[60]*tmpObjS[66] + tmpFx[72]*tmpObjS[79] + tmpFx[84]*tmpObjS[92] + tmpFx[96]*tmpObjS[105] + tmpFx[108]*tmpObjS[118] + tmpFx[120]*tmpObjS[131] + tmpFx[132]*tmpObjS[144] + tmpFx[144]*tmpObjS[157];
tmpQ2[2] = + tmpFx[0]*tmpObjS[2] + tmpFx[12]*tmpObjS[15] + tmpFx[24]*tmpObjS[28] + tmpFx[36]*tmpObjS[41] + tmpFx[48]*tmpObjS[54] + tmpFx[60]*tmpObjS[67] + tmpFx[72]*tmpObjS[80] + tmpFx[84]*tmpObjS[93] + tmpFx[96]*tmpObjS[106] + tmpFx[108]*tmpObjS[119] + tmpFx[120]*tmpObjS[132] + tmpFx[132]*tmpObjS[145] + tmpFx[144]*tmpObjS[158];
tmpQ2[3] = + tmpFx[0]*tmpObjS[3] + tmpFx[12]*tmpObjS[16] + tmpFx[24]*tmpObjS[29] + tmpFx[36]*tmpObjS[42] + tmpFx[48]*tmpObjS[55] + tmpFx[60]*tmpObjS[68] + tmpFx[72]*tmpObjS[81] + tmpFx[84]*tmpObjS[94] + tmpFx[96]*tmpObjS[107] + tmpFx[108]*tmpObjS[120] + tmpFx[120]*tmpObjS[133] + tmpFx[132]*tmpObjS[146] + tmpFx[144]*tmpObjS[159];
tmpQ2[4] = + tmpFx[0]*tmpObjS[4] + tmpFx[12]*tmpObjS[17] + tmpFx[24]*tmpObjS[30] + tmpFx[36]*tmpObjS[43] + tmpFx[48]*tmpObjS[56] + tmpFx[60]*tmpObjS[69] + tmpFx[72]*tmpObjS[82] + tmpFx[84]*tmpObjS[95] + tmpFx[96]*tmpObjS[108] + tmpFx[108]*tmpObjS[121] + tmpFx[120]*tmpObjS[134] + tmpFx[132]*tmpObjS[147] + tmpFx[144]*tmpObjS[160];
tmpQ2[5] = + tmpFx[0]*tmpObjS[5] + tmpFx[12]*tmpObjS[18] + tmpFx[24]*tmpObjS[31] + tmpFx[36]*tmpObjS[44] + tmpFx[48]*tmpObjS[57] + tmpFx[60]*tmpObjS[70] + tmpFx[72]*tmpObjS[83] + tmpFx[84]*tmpObjS[96] + tmpFx[96]*tmpObjS[109] + tmpFx[108]*tmpObjS[122] + tmpFx[120]*tmpObjS[135] + tmpFx[132]*tmpObjS[148] + tmpFx[144]*tmpObjS[161];
tmpQ2[6] = + tmpFx[0]*tmpObjS[6] + tmpFx[12]*tmpObjS[19] + tmpFx[24]*tmpObjS[32] + tmpFx[36]*tmpObjS[45] + tmpFx[48]*tmpObjS[58] + tmpFx[60]*tmpObjS[71] + tmpFx[72]*tmpObjS[84] + tmpFx[84]*tmpObjS[97] + tmpFx[96]*tmpObjS[110] + tmpFx[108]*tmpObjS[123] + tmpFx[120]*tmpObjS[136] + tmpFx[132]*tmpObjS[149] + tmpFx[144]*tmpObjS[162];
tmpQ2[7] = + tmpFx[0]*tmpObjS[7] + tmpFx[12]*tmpObjS[20] + tmpFx[24]*tmpObjS[33] + tmpFx[36]*tmpObjS[46] + tmpFx[48]*tmpObjS[59] + tmpFx[60]*tmpObjS[72] + tmpFx[72]*tmpObjS[85] + tmpFx[84]*tmpObjS[98] + tmpFx[96]*tmpObjS[111] + tmpFx[108]*tmpObjS[124] + tmpFx[120]*tmpObjS[137] + tmpFx[132]*tmpObjS[150] + tmpFx[144]*tmpObjS[163];
tmpQ2[8] = + tmpFx[0]*tmpObjS[8] + tmpFx[12]*tmpObjS[21] + tmpFx[24]*tmpObjS[34] + tmpFx[36]*tmpObjS[47] + tmpFx[48]*tmpObjS[60] + tmpFx[60]*tmpObjS[73] + tmpFx[72]*tmpObjS[86] + tmpFx[84]*tmpObjS[99] + tmpFx[96]*tmpObjS[112] + tmpFx[108]*tmpObjS[125] + tmpFx[120]*tmpObjS[138] + tmpFx[132]*tmpObjS[151] + tmpFx[144]*tmpObjS[164];
tmpQ2[9] = + tmpFx[0]*tmpObjS[9] + tmpFx[12]*tmpObjS[22] + tmpFx[24]*tmpObjS[35] + tmpFx[36]*tmpObjS[48] + tmpFx[48]*tmpObjS[61] + tmpFx[60]*tmpObjS[74] + tmpFx[72]*tmpObjS[87] + tmpFx[84]*tmpObjS[100] + tmpFx[96]*tmpObjS[113] + tmpFx[108]*tmpObjS[126] + tmpFx[120]*tmpObjS[139] + tmpFx[132]*tmpObjS[152] + tmpFx[144]*tmpObjS[165];
tmpQ2[10] = + tmpFx[0]*tmpObjS[10] + tmpFx[12]*tmpObjS[23] + tmpFx[24]*tmpObjS[36] + tmpFx[36]*tmpObjS[49] + tmpFx[48]*tmpObjS[62] + tmpFx[60]*tmpObjS[75] + tmpFx[72]*tmpObjS[88] + tmpFx[84]*tmpObjS[101] + tmpFx[96]*tmpObjS[114] + tmpFx[108]*tmpObjS[127] + tmpFx[120]*tmpObjS[140] + tmpFx[132]*tmpObjS[153] + tmpFx[144]*tmpObjS[166];
tmpQ2[11] = + tmpFx[0]*tmpObjS[11] + tmpFx[12]*tmpObjS[24] + tmpFx[24]*tmpObjS[37] + tmpFx[36]*tmpObjS[50] + tmpFx[48]*tmpObjS[63] + tmpFx[60]*tmpObjS[76] + tmpFx[72]*tmpObjS[89] + tmpFx[84]*tmpObjS[102] + tmpFx[96]*tmpObjS[115] + tmpFx[108]*tmpObjS[128] + tmpFx[120]*tmpObjS[141] + tmpFx[132]*tmpObjS[154] + tmpFx[144]*tmpObjS[167];
tmpQ2[12] = + tmpFx[0]*tmpObjS[12] + tmpFx[12]*tmpObjS[25] + tmpFx[24]*tmpObjS[38] + tmpFx[36]*tmpObjS[51] + tmpFx[48]*tmpObjS[64] + tmpFx[60]*tmpObjS[77] + tmpFx[72]*tmpObjS[90] + tmpFx[84]*tmpObjS[103] + tmpFx[96]*tmpObjS[116] + tmpFx[108]*tmpObjS[129] + tmpFx[120]*tmpObjS[142] + tmpFx[132]*tmpObjS[155] + tmpFx[144]*tmpObjS[168];
tmpQ2[13] = + tmpFx[1]*tmpObjS[0] + tmpFx[13]*tmpObjS[13] + tmpFx[25]*tmpObjS[26] + tmpFx[37]*tmpObjS[39] + tmpFx[49]*tmpObjS[52] + tmpFx[61]*tmpObjS[65] + tmpFx[73]*tmpObjS[78] + tmpFx[85]*tmpObjS[91] + tmpFx[97]*tmpObjS[104] + tmpFx[109]*tmpObjS[117] + tmpFx[121]*tmpObjS[130] + tmpFx[133]*tmpObjS[143] + tmpFx[145]*tmpObjS[156];
tmpQ2[14] = + tmpFx[1]*tmpObjS[1] + tmpFx[13]*tmpObjS[14] + tmpFx[25]*tmpObjS[27] + tmpFx[37]*tmpObjS[40] + tmpFx[49]*tmpObjS[53] + tmpFx[61]*tmpObjS[66] + tmpFx[73]*tmpObjS[79] + tmpFx[85]*tmpObjS[92] + tmpFx[97]*tmpObjS[105] + tmpFx[109]*tmpObjS[118] + tmpFx[121]*tmpObjS[131] + tmpFx[133]*tmpObjS[144] + tmpFx[145]*tmpObjS[157];
tmpQ2[15] = + tmpFx[1]*tmpObjS[2] + tmpFx[13]*tmpObjS[15] + tmpFx[25]*tmpObjS[28] + tmpFx[37]*tmpObjS[41] + tmpFx[49]*tmpObjS[54] + tmpFx[61]*tmpObjS[67] + tmpFx[73]*tmpObjS[80] + tmpFx[85]*tmpObjS[93] + tmpFx[97]*tmpObjS[106] + tmpFx[109]*tmpObjS[119] + tmpFx[121]*tmpObjS[132] + tmpFx[133]*tmpObjS[145] + tmpFx[145]*tmpObjS[158];
tmpQ2[16] = + tmpFx[1]*tmpObjS[3] + tmpFx[13]*tmpObjS[16] + tmpFx[25]*tmpObjS[29] + tmpFx[37]*tmpObjS[42] + tmpFx[49]*tmpObjS[55] + tmpFx[61]*tmpObjS[68] + tmpFx[73]*tmpObjS[81] + tmpFx[85]*tmpObjS[94] + tmpFx[97]*tmpObjS[107] + tmpFx[109]*tmpObjS[120] + tmpFx[121]*tmpObjS[133] + tmpFx[133]*tmpObjS[146] + tmpFx[145]*tmpObjS[159];
tmpQ2[17] = + tmpFx[1]*tmpObjS[4] + tmpFx[13]*tmpObjS[17] + tmpFx[25]*tmpObjS[30] + tmpFx[37]*tmpObjS[43] + tmpFx[49]*tmpObjS[56] + tmpFx[61]*tmpObjS[69] + tmpFx[73]*tmpObjS[82] + tmpFx[85]*tmpObjS[95] + tmpFx[97]*tmpObjS[108] + tmpFx[109]*tmpObjS[121] + tmpFx[121]*tmpObjS[134] + tmpFx[133]*tmpObjS[147] + tmpFx[145]*tmpObjS[160];
tmpQ2[18] = + tmpFx[1]*tmpObjS[5] + tmpFx[13]*tmpObjS[18] + tmpFx[25]*tmpObjS[31] + tmpFx[37]*tmpObjS[44] + tmpFx[49]*tmpObjS[57] + tmpFx[61]*tmpObjS[70] + tmpFx[73]*tmpObjS[83] + tmpFx[85]*tmpObjS[96] + tmpFx[97]*tmpObjS[109] + tmpFx[109]*tmpObjS[122] + tmpFx[121]*tmpObjS[135] + tmpFx[133]*tmpObjS[148] + tmpFx[145]*tmpObjS[161];
tmpQ2[19] = + tmpFx[1]*tmpObjS[6] + tmpFx[13]*tmpObjS[19] + tmpFx[25]*tmpObjS[32] + tmpFx[37]*tmpObjS[45] + tmpFx[49]*tmpObjS[58] + tmpFx[61]*tmpObjS[71] + tmpFx[73]*tmpObjS[84] + tmpFx[85]*tmpObjS[97] + tmpFx[97]*tmpObjS[110] + tmpFx[109]*tmpObjS[123] + tmpFx[121]*tmpObjS[136] + tmpFx[133]*tmpObjS[149] + tmpFx[145]*tmpObjS[162];
tmpQ2[20] = + tmpFx[1]*tmpObjS[7] + tmpFx[13]*tmpObjS[20] + tmpFx[25]*tmpObjS[33] + tmpFx[37]*tmpObjS[46] + tmpFx[49]*tmpObjS[59] + tmpFx[61]*tmpObjS[72] + tmpFx[73]*tmpObjS[85] + tmpFx[85]*tmpObjS[98] + tmpFx[97]*tmpObjS[111] + tmpFx[109]*tmpObjS[124] + tmpFx[121]*tmpObjS[137] + tmpFx[133]*tmpObjS[150] + tmpFx[145]*tmpObjS[163];
tmpQ2[21] = + tmpFx[1]*tmpObjS[8] + tmpFx[13]*tmpObjS[21] + tmpFx[25]*tmpObjS[34] + tmpFx[37]*tmpObjS[47] + tmpFx[49]*tmpObjS[60] + tmpFx[61]*tmpObjS[73] + tmpFx[73]*tmpObjS[86] + tmpFx[85]*tmpObjS[99] + tmpFx[97]*tmpObjS[112] + tmpFx[109]*tmpObjS[125] + tmpFx[121]*tmpObjS[138] + tmpFx[133]*tmpObjS[151] + tmpFx[145]*tmpObjS[164];
tmpQ2[22] = + tmpFx[1]*tmpObjS[9] + tmpFx[13]*tmpObjS[22] + tmpFx[25]*tmpObjS[35] + tmpFx[37]*tmpObjS[48] + tmpFx[49]*tmpObjS[61] + tmpFx[61]*tmpObjS[74] + tmpFx[73]*tmpObjS[87] + tmpFx[85]*tmpObjS[100] + tmpFx[97]*tmpObjS[113] + tmpFx[109]*tmpObjS[126] + tmpFx[121]*tmpObjS[139] + tmpFx[133]*tmpObjS[152] + tmpFx[145]*tmpObjS[165];
tmpQ2[23] = + tmpFx[1]*tmpObjS[10] + tmpFx[13]*tmpObjS[23] + tmpFx[25]*tmpObjS[36] + tmpFx[37]*tmpObjS[49] + tmpFx[49]*tmpObjS[62] + tmpFx[61]*tmpObjS[75] + tmpFx[73]*tmpObjS[88] + tmpFx[85]*tmpObjS[101] + tmpFx[97]*tmpObjS[114] + tmpFx[109]*tmpObjS[127] + tmpFx[121]*tmpObjS[140] + tmpFx[133]*tmpObjS[153] + tmpFx[145]*tmpObjS[166];
tmpQ2[24] = + tmpFx[1]*tmpObjS[11] + tmpFx[13]*tmpObjS[24] + tmpFx[25]*tmpObjS[37] + tmpFx[37]*tmpObjS[50] + tmpFx[49]*tmpObjS[63] + tmpFx[61]*tmpObjS[76] + tmpFx[73]*tmpObjS[89] + tmpFx[85]*tmpObjS[102] + tmpFx[97]*tmpObjS[115] + tmpFx[109]*tmpObjS[128] + tmpFx[121]*tmpObjS[141] + tmpFx[133]*tmpObjS[154] + tmpFx[145]*tmpObjS[167];
tmpQ2[25] = + tmpFx[1]*tmpObjS[12] + tmpFx[13]*tmpObjS[25] + tmpFx[25]*tmpObjS[38] + tmpFx[37]*tmpObjS[51] + tmpFx[49]*tmpObjS[64] + tmpFx[61]*tmpObjS[77] + tmpFx[73]*tmpObjS[90] + tmpFx[85]*tmpObjS[103] + tmpFx[97]*tmpObjS[116] + tmpFx[109]*tmpObjS[129] + tmpFx[121]*tmpObjS[142] + tmpFx[133]*tmpObjS[155] + tmpFx[145]*tmpObjS[168];
tmpQ2[26] = + tmpFx[2]*tmpObjS[0] + tmpFx[14]*tmpObjS[13] + tmpFx[26]*tmpObjS[26] + tmpFx[38]*tmpObjS[39] + tmpFx[50]*tmpObjS[52] + tmpFx[62]*tmpObjS[65] + tmpFx[74]*tmpObjS[78] + tmpFx[86]*tmpObjS[91] + tmpFx[98]*tmpObjS[104] + tmpFx[110]*tmpObjS[117] + tmpFx[122]*tmpObjS[130] + tmpFx[134]*tmpObjS[143] + tmpFx[146]*tmpObjS[156];
tmpQ2[27] = + tmpFx[2]*tmpObjS[1] + tmpFx[14]*tmpObjS[14] + tmpFx[26]*tmpObjS[27] + tmpFx[38]*tmpObjS[40] + tmpFx[50]*tmpObjS[53] + tmpFx[62]*tmpObjS[66] + tmpFx[74]*tmpObjS[79] + tmpFx[86]*tmpObjS[92] + tmpFx[98]*tmpObjS[105] + tmpFx[110]*tmpObjS[118] + tmpFx[122]*tmpObjS[131] + tmpFx[134]*tmpObjS[144] + tmpFx[146]*tmpObjS[157];
tmpQ2[28] = + tmpFx[2]*tmpObjS[2] + tmpFx[14]*tmpObjS[15] + tmpFx[26]*tmpObjS[28] + tmpFx[38]*tmpObjS[41] + tmpFx[50]*tmpObjS[54] + tmpFx[62]*tmpObjS[67] + tmpFx[74]*tmpObjS[80] + tmpFx[86]*tmpObjS[93] + tmpFx[98]*tmpObjS[106] + tmpFx[110]*tmpObjS[119] + tmpFx[122]*tmpObjS[132] + tmpFx[134]*tmpObjS[145] + tmpFx[146]*tmpObjS[158];
tmpQ2[29] = + tmpFx[2]*tmpObjS[3] + tmpFx[14]*tmpObjS[16] + tmpFx[26]*tmpObjS[29] + tmpFx[38]*tmpObjS[42] + tmpFx[50]*tmpObjS[55] + tmpFx[62]*tmpObjS[68] + tmpFx[74]*tmpObjS[81] + tmpFx[86]*tmpObjS[94] + tmpFx[98]*tmpObjS[107] + tmpFx[110]*tmpObjS[120] + tmpFx[122]*tmpObjS[133] + tmpFx[134]*tmpObjS[146] + tmpFx[146]*tmpObjS[159];
tmpQ2[30] = + tmpFx[2]*tmpObjS[4] + tmpFx[14]*tmpObjS[17] + tmpFx[26]*tmpObjS[30] + tmpFx[38]*tmpObjS[43] + tmpFx[50]*tmpObjS[56] + tmpFx[62]*tmpObjS[69] + tmpFx[74]*tmpObjS[82] + tmpFx[86]*tmpObjS[95] + tmpFx[98]*tmpObjS[108] + tmpFx[110]*tmpObjS[121] + tmpFx[122]*tmpObjS[134] + tmpFx[134]*tmpObjS[147] + tmpFx[146]*tmpObjS[160];
tmpQ2[31] = + tmpFx[2]*tmpObjS[5] + tmpFx[14]*tmpObjS[18] + tmpFx[26]*tmpObjS[31] + tmpFx[38]*tmpObjS[44] + tmpFx[50]*tmpObjS[57] + tmpFx[62]*tmpObjS[70] + tmpFx[74]*tmpObjS[83] + tmpFx[86]*tmpObjS[96] + tmpFx[98]*tmpObjS[109] + tmpFx[110]*tmpObjS[122] + tmpFx[122]*tmpObjS[135] + tmpFx[134]*tmpObjS[148] + tmpFx[146]*tmpObjS[161];
tmpQ2[32] = + tmpFx[2]*tmpObjS[6] + tmpFx[14]*tmpObjS[19] + tmpFx[26]*tmpObjS[32] + tmpFx[38]*tmpObjS[45] + tmpFx[50]*tmpObjS[58] + tmpFx[62]*tmpObjS[71] + tmpFx[74]*tmpObjS[84] + tmpFx[86]*tmpObjS[97] + tmpFx[98]*tmpObjS[110] + tmpFx[110]*tmpObjS[123] + tmpFx[122]*tmpObjS[136] + tmpFx[134]*tmpObjS[149] + tmpFx[146]*tmpObjS[162];
tmpQ2[33] = + tmpFx[2]*tmpObjS[7] + tmpFx[14]*tmpObjS[20] + tmpFx[26]*tmpObjS[33] + tmpFx[38]*tmpObjS[46] + tmpFx[50]*tmpObjS[59] + tmpFx[62]*tmpObjS[72] + tmpFx[74]*tmpObjS[85] + tmpFx[86]*tmpObjS[98] + tmpFx[98]*tmpObjS[111] + tmpFx[110]*tmpObjS[124] + tmpFx[122]*tmpObjS[137] + tmpFx[134]*tmpObjS[150] + tmpFx[146]*tmpObjS[163];
tmpQ2[34] = + tmpFx[2]*tmpObjS[8] + tmpFx[14]*tmpObjS[21] + tmpFx[26]*tmpObjS[34] + tmpFx[38]*tmpObjS[47] + tmpFx[50]*tmpObjS[60] + tmpFx[62]*tmpObjS[73] + tmpFx[74]*tmpObjS[86] + tmpFx[86]*tmpObjS[99] + tmpFx[98]*tmpObjS[112] + tmpFx[110]*tmpObjS[125] + tmpFx[122]*tmpObjS[138] + tmpFx[134]*tmpObjS[151] + tmpFx[146]*tmpObjS[164];
tmpQ2[35] = + tmpFx[2]*tmpObjS[9] + tmpFx[14]*tmpObjS[22] + tmpFx[26]*tmpObjS[35] + tmpFx[38]*tmpObjS[48] + tmpFx[50]*tmpObjS[61] + tmpFx[62]*tmpObjS[74] + tmpFx[74]*tmpObjS[87] + tmpFx[86]*tmpObjS[100] + tmpFx[98]*tmpObjS[113] + tmpFx[110]*tmpObjS[126] + tmpFx[122]*tmpObjS[139] + tmpFx[134]*tmpObjS[152] + tmpFx[146]*tmpObjS[165];
tmpQ2[36] = + tmpFx[2]*tmpObjS[10] + tmpFx[14]*tmpObjS[23] + tmpFx[26]*tmpObjS[36] + tmpFx[38]*tmpObjS[49] + tmpFx[50]*tmpObjS[62] + tmpFx[62]*tmpObjS[75] + tmpFx[74]*tmpObjS[88] + tmpFx[86]*tmpObjS[101] + tmpFx[98]*tmpObjS[114] + tmpFx[110]*tmpObjS[127] + tmpFx[122]*tmpObjS[140] + tmpFx[134]*tmpObjS[153] + tmpFx[146]*tmpObjS[166];
tmpQ2[37] = + tmpFx[2]*tmpObjS[11] + tmpFx[14]*tmpObjS[24] + tmpFx[26]*tmpObjS[37] + tmpFx[38]*tmpObjS[50] + tmpFx[50]*tmpObjS[63] + tmpFx[62]*tmpObjS[76] + tmpFx[74]*tmpObjS[89] + tmpFx[86]*tmpObjS[102] + tmpFx[98]*tmpObjS[115] + tmpFx[110]*tmpObjS[128] + tmpFx[122]*tmpObjS[141] + tmpFx[134]*tmpObjS[154] + tmpFx[146]*tmpObjS[167];
tmpQ2[38] = + tmpFx[2]*tmpObjS[12] + tmpFx[14]*tmpObjS[25] + tmpFx[26]*tmpObjS[38] + tmpFx[38]*tmpObjS[51] + tmpFx[50]*tmpObjS[64] + tmpFx[62]*tmpObjS[77] + tmpFx[74]*tmpObjS[90] + tmpFx[86]*tmpObjS[103] + tmpFx[98]*tmpObjS[116] + tmpFx[110]*tmpObjS[129] + tmpFx[122]*tmpObjS[142] + tmpFx[134]*tmpObjS[155] + tmpFx[146]*tmpObjS[168];
tmpQ2[39] = + tmpFx[3]*tmpObjS[0] + tmpFx[15]*tmpObjS[13] + tmpFx[27]*tmpObjS[26] + tmpFx[39]*tmpObjS[39] + tmpFx[51]*tmpObjS[52] + tmpFx[63]*tmpObjS[65] + tmpFx[75]*tmpObjS[78] + tmpFx[87]*tmpObjS[91] + tmpFx[99]*tmpObjS[104] + tmpFx[111]*tmpObjS[117] + tmpFx[123]*tmpObjS[130] + tmpFx[135]*tmpObjS[143] + tmpFx[147]*tmpObjS[156];
tmpQ2[40] = + tmpFx[3]*tmpObjS[1] + tmpFx[15]*tmpObjS[14] + tmpFx[27]*tmpObjS[27] + tmpFx[39]*tmpObjS[40] + tmpFx[51]*tmpObjS[53] + tmpFx[63]*tmpObjS[66] + tmpFx[75]*tmpObjS[79] + tmpFx[87]*tmpObjS[92] + tmpFx[99]*tmpObjS[105] + tmpFx[111]*tmpObjS[118] + tmpFx[123]*tmpObjS[131] + tmpFx[135]*tmpObjS[144] + tmpFx[147]*tmpObjS[157];
tmpQ2[41] = + tmpFx[3]*tmpObjS[2] + tmpFx[15]*tmpObjS[15] + tmpFx[27]*tmpObjS[28] + tmpFx[39]*tmpObjS[41] + tmpFx[51]*tmpObjS[54] + tmpFx[63]*tmpObjS[67] + tmpFx[75]*tmpObjS[80] + tmpFx[87]*tmpObjS[93] + tmpFx[99]*tmpObjS[106] + tmpFx[111]*tmpObjS[119] + tmpFx[123]*tmpObjS[132] + tmpFx[135]*tmpObjS[145] + tmpFx[147]*tmpObjS[158];
tmpQ2[42] = + tmpFx[3]*tmpObjS[3] + tmpFx[15]*tmpObjS[16] + tmpFx[27]*tmpObjS[29] + tmpFx[39]*tmpObjS[42] + tmpFx[51]*tmpObjS[55] + tmpFx[63]*tmpObjS[68] + tmpFx[75]*tmpObjS[81] + tmpFx[87]*tmpObjS[94] + tmpFx[99]*tmpObjS[107] + tmpFx[111]*tmpObjS[120] + tmpFx[123]*tmpObjS[133] + tmpFx[135]*tmpObjS[146] + tmpFx[147]*tmpObjS[159];
tmpQ2[43] = + tmpFx[3]*tmpObjS[4] + tmpFx[15]*tmpObjS[17] + tmpFx[27]*tmpObjS[30] + tmpFx[39]*tmpObjS[43] + tmpFx[51]*tmpObjS[56] + tmpFx[63]*tmpObjS[69] + tmpFx[75]*tmpObjS[82] + tmpFx[87]*tmpObjS[95] + tmpFx[99]*tmpObjS[108] + tmpFx[111]*tmpObjS[121] + tmpFx[123]*tmpObjS[134] + tmpFx[135]*tmpObjS[147] + tmpFx[147]*tmpObjS[160];
tmpQ2[44] = + tmpFx[3]*tmpObjS[5] + tmpFx[15]*tmpObjS[18] + tmpFx[27]*tmpObjS[31] + tmpFx[39]*tmpObjS[44] + tmpFx[51]*tmpObjS[57] + tmpFx[63]*tmpObjS[70] + tmpFx[75]*tmpObjS[83] + tmpFx[87]*tmpObjS[96] + tmpFx[99]*tmpObjS[109] + tmpFx[111]*tmpObjS[122] + tmpFx[123]*tmpObjS[135] + tmpFx[135]*tmpObjS[148] + tmpFx[147]*tmpObjS[161];
tmpQ2[45] = + tmpFx[3]*tmpObjS[6] + tmpFx[15]*tmpObjS[19] + tmpFx[27]*tmpObjS[32] + tmpFx[39]*tmpObjS[45] + tmpFx[51]*tmpObjS[58] + tmpFx[63]*tmpObjS[71] + tmpFx[75]*tmpObjS[84] + tmpFx[87]*tmpObjS[97] + tmpFx[99]*tmpObjS[110] + tmpFx[111]*tmpObjS[123] + tmpFx[123]*tmpObjS[136] + tmpFx[135]*tmpObjS[149] + tmpFx[147]*tmpObjS[162];
tmpQ2[46] = + tmpFx[3]*tmpObjS[7] + tmpFx[15]*tmpObjS[20] + tmpFx[27]*tmpObjS[33] + tmpFx[39]*tmpObjS[46] + tmpFx[51]*tmpObjS[59] + tmpFx[63]*tmpObjS[72] + tmpFx[75]*tmpObjS[85] + tmpFx[87]*tmpObjS[98] + tmpFx[99]*tmpObjS[111] + tmpFx[111]*tmpObjS[124] + tmpFx[123]*tmpObjS[137] + tmpFx[135]*tmpObjS[150] + tmpFx[147]*tmpObjS[163];
tmpQ2[47] = + tmpFx[3]*tmpObjS[8] + tmpFx[15]*tmpObjS[21] + tmpFx[27]*tmpObjS[34] + tmpFx[39]*tmpObjS[47] + tmpFx[51]*tmpObjS[60] + tmpFx[63]*tmpObjS[73] + tmpFx[75]*tmpObjS[86] + tmpFx[87]*tmpObjS[99] + tmpFx[99]*tmpObjS[112] + tmpFx[111]*tmpObjS[125] + tmpFx[123]*tmpObjS[138] + tmpFx[135]*tmpObjS[151] + tmpFx[147]*tmpObjS[164];
tmpQ2[48] = + tmpFx[3]*tmpObjS[9] + tmpFx[15]*tmpObjS[22] + tmpFx[27]*tmpObjS[35] + tmpFx[39]*tmpObjS[48] + tmpFx[51]*tmpObjS[61] + tmpFx[63]*tmpObjS[74] + tmpFx[75]*tmpObjS[87] + tmpFx[87]*tmpObjS[100] + tmpFx[99]*tmpObjS[113] + tmpFx[111]*tmpObjS[126] + tmpFx[123]*tmpObjS[139] + tmpFx[135]*tmpObjS[152] + tmpFx[147]*tmpObjS[165];
tmpQ2[49] = + tmpFx[3]*tmpObjS[10] + tmpFx[15]*tmpObjS[23] + tmpFx[27]*tmpObjS[36] + tmpFx[39]*tmpObjS[49] + tmpFx[51]*tmpObjS[62] + tmpFx[63]*tmpObjS[75] + tmpFx[75]*tmpObjS[88] + tmpFx[87]*tmpObjS[101] + tmpFx[99]*tmpObjS[114] + tmpFx[111]*tmpObjS[127] + tmpFx[123]*tmpObjS[140] + tmpFx[135]*tmpObjS[153] + tmpFx[147]*tmpObjS[166];
tmpQ2[50] = + tmpFx[3]*tmpObjS[11] + tmpFx[15]*tmpObjS[24] + tmpFx[27]*tmpObjS[37] + tmpFx[39]*tmpObjS[50] + tmpFx[51]*tmpObjS[63] + tmpFx[63]*tmpObjS[76] + tmpFx[75]*tmpObjS[89] + tmpFx[87]*tmpObjS[102] + tmpFx[99]*tmpObjS[115] + tmpFx[111]*tmpObjS[128] + tmpFx[123]*tmpObjS[141] + tmpFx[135]*tmpObjS[154] + tmpFx[147]*tmpObjS[167];
tmpQ2[51] = + tmpFx[3]*tmpObjS[12] + tmpFx[15]*tmpObjS[25] + tmpFx[27]*tmpObjS[38] + tmpFx[39]*tmpObjS[51] + tmpFx[51]*tmpObjS[64] + tmpFx[63]*tmpObjS[77] + tmpFx[75]*tmpObjS[90] + tmpFx[87]*tmpObjS[103] + tmpFx[99]*tmpObjS[116] + tmpFx[111]*tmpObjS[129] + tmpFx[123]*tmpObjS[142] + tmpFx[135]*tmpObjS[155] + tmpFx[147]*tmpObjS[168];
tmpQ2[52] = + tmpFx[4]*tmpObjS[0] + tmpFx[16]*tmpObjS[13] + tmpFx[28]*tmpObjS[26] + tmpFx[40]*tmpObjS[39] + tmpFx[52]*tmpObjS[52] + tmpFx[64]*tmpObjS[65] + tmpFx[76]*tmpObjS[78] + tmpFx[88]*tmpObjS[91] + tmpFx[100]*tmpObjS[104] + tmpFx[112]*tmpObjS[117] + tmpFx[124]*tmpObjS[130] + tmpFx[136]*tmpObjS[143] + tmpFx[148]*tmpObjS[156];
tmpQ2[53] = + tmpFx[4]*tmpObjS[1] + tmpFx[16]*tmpObjS[14] + tmpFx[28]*tmpObjS[27] + tmpFx[40]*tmpObjS[40] + tmpFx[52]*tmpObjS[53] + tmpFx[64]*tmpObjS[66] + tmpFx[76]*tmpObjS[79] + tmpFx[88]*tmpObjS[92] + tmpFx[100]*tmpObjS[105] + tmpFx[112]*tmpObjS[118] + tmpFx[124]*tmpObjS[131] + tmpFx[136]*tmpObjS[144] + tmpFx[148]*tmpObjS[157];
tmpQ2[54] = + tmpFx[4]*tmpObjS[2] + tmpFx[16]*tmpObjS[15] + tmpFx[28]*tmpObjS[28] + tmpFx[40]*tmpObjS[41] + tmpFx[52]*tmpObjS[54] + tmpFx[64]*tmpObjS[67] + tmpFx[76]*tmpObjS[80] + tmpFx[88]*tmpObjS[93] + tmpFx[100]*tmpObjS[106] + tmpFx[112]*tmpObjS[119] + tmpFx[124]*tmpObjS[132] + tmpFx[136]*tmpObjS[145] + tmpFx[148]*tmpObjS[158];
tmpQ2[55] = + tmpFx[4]*tmpObjS[3] + tmpFx[16]*tmpObjS[16] + tmpFx[28]*tmpObjS[29] + tmpFx[40]*tmpObjS[42] + tmpFx[52]*tmpObjS[55] + tmpFx[64]*tmpObjS[68] + tmpFx[76]*tmpObjS[81] + tmpFx[88]*tmpObjS[94] + tmpFx[100]*tmpObjS[107] + tmpFx[112]*tmpObjS[120] + tmpFx[124]*tmpObjS[133] + tmpFx[136]*tmpObjS[146] + tmpFx[148]*tmpObjS[159];
tmpQ2[56] = + tmpFx[4]*tmpObjS[4] + tmpFx[16]*tmpObjS[17] + tmpFx[28]*tmpObjS[30] + tmpFx[40]*tmpObjS[43] + tmpFx[52]*tmpObjS[56] + tmpFx[64]*tmpObjS[69] + tmpFx[76]*tmpObjS[82] + tmpFx[88]*tmpObjS[95] + tmpFx[100]*tmpObjS[108] + tmpFx[112]*tmpObjS[121] + tmpFx[124]*tmpObjS[134] + tmpFx[136]*tmpObjS[147] + tmpFx[148]*tmpObjS[160];
tmpQ2[57] = + tmpFx[4]*tmpObjS[5] + tmpFx[16]*tmpObjS[18] + tmpFx[28]*tmpObjS[31] + tmpFx[40]*tmpObjS[44] + tmpFx[52]*tmpObjS[57] + tmpFx[64]*tmpObjS[70] + tmpFx[76]*tmpObjS[83] + tmpFx[88]*tmpObjS[96] + tmpFx[100]*tmpObjS[109] + tmpFx[112]*tmpObjS[122] + tmpFx[124]*tmpObjS[135] + tmpFx[136]*tmpObjS[148] + tmpFx[148]*tmpObjS[161];
tmpQ2[58] = + tmpFx[4]*tmpObjS[6] + tmpFx[16]*tmpObjS[19] + tmpFx[28]*tmpObjS[32] + tmpFx[40]*tmpObjS[45] + tmpFx[52]*tmpObjS[58] + tmpFx[64]*tmpObjS[71] + tmpFx[76]*tmpObjS[84] + tmpFx[88]*tmpObjS[97] + tmpFx[100]*tmpObjS[110] + tmpFx[112]*tmpObjS[123] + tmpFx[124]*tmpObjS[136] + tmpFx[136]*tmpObjS[149] + tmpFx[148]*tmpObjS[162];
tmpQ2[59] = + tmpFx[4]*tmpObjS[7] + tmpFx[16]*tmpObjS[20] + tmpFx[28]*tmpObjS[33] + tmpFx[40]*tmpObjS[46] + tmpFx[52]*tmpObjS[59] + tmpFx[64]*tmpObjS[72] + tmpFx[76]*tmpObjS[85] + tmpFx[88]*tmpObjS[98] + tmpFx[100]*tmpObjS[111] + tmpFx[112]*tmpObjS[124] + tmpFx[124]*tmpObjS[137] + tmpFx[136]*tmpObjS[150] + tmpFx[148]*tmpObjS[163];
tmpQ2[60] = + tmpFx[4]*tmpObjS[8] + tmpFx[16]*tmpObjS[21] + tmpFx[28]*tmpObjS[34] + tmpFx[40]*tmpObjS[47] + tmpFx[52]*tmpObjS[60] + tmpFx[64]*tmpObjS[73] + tmpFx[76]*tmpObjS[86] + tmpFx[88]*tmpObjS[99] + tmpFx[100]*tmpObjS[112] + tmpFx[112]*tmpObjS[125] + tmpFx[124]*tmpObjS[138] + tmpFx[136]*tmpObjS[151] + tmpFx[148]*tmpObjS[164];
tmpQ2[61] = + tmpFx[4]*tmpObjS[9] + tmpFx[16]*tmpObjS[22] + tmpFx[28]*tmpObjS[35] + tmpFx[40]*tmpObjS[48] + tmpFx[52]*tmpObjS[61] + tmpFx[64]*tmpObjS[74] + tmpFx[76]*tmpObjS[87] + tmpFx[88]*tmpObjS[100] + tmpFx[100]*tmpObjS[113] + tmpFx[112]*tmpObjS[126] + tmpFx[124]*tmpObjS[139] + tmpFx[136]*tmpObjS[152] + tmpFx[148]*tmpObjS[165];
tmpQ2[62] = + tmpFx[4]*tmpObjS[10] + tmpFx[16]*tmpObjS[23] + tmpFx[28]*tmpObjS[36] + tmpFx[40]*tmpObjS[49] + tmpFx[52]*tmpObjS[62] + tmpFx[64]*tmpObjS[75] + tmpFx[76]*tmpObjS[88] + tmpFx[88]*tmpObjS[101] + tmpFx[100]*tmpObjS[114] + tmpFx[112]*tmpObjS[127] + tmpFx[124]*tmpObjS[140] + tmpFx[136]*tmpObjS[153] + tmpFx[148]*tmpObjS[166];
tmpQ2[63] = + tmpFx[4]*tmpObjS[11] + tmpFx[16]*tmpObjS[24] + tmpFx[28]*tmpObjS[37] + tmpFx[40]*tmpObjS[50] + tmpFx[52]*tmpObjS[63] + tmpFx[64]*tmpObjS[76] + tmpFx[76]*tmpObjS[89] + tmpFx[88]*tmpObjS[102] + tmpFx[100]*tmpObjS[115] + tmpFx[112]*tmpObjS[128] + tmpFx[124]*tmpObjS[141] + tmpFx[136]*tmpObjS[154] + tmpFx[148]*tmpObjS[167];
tmpQ2[64] = + tmpFx[4]*tmpObjS[12] + tmpFx[16]*tmpObjS[25] + tmpFx[28]*tmpObjS[38] + tmpFx[40]*tmpObjS[51] + tmpFx[52]*tmpObjS[64] + tmpFx[64]*tmpObjS[77] + tmpFx[76]*tmpObjS[90] + tmpFx[88]*tmpObjS[103] + tmpFx[100]*tmpObjS[116] + tmpFx[112]*tmpObjS[129] + tmpFx[124]*tmpObjS[142] + tmpFx[136]*tmpObjS[155] + tmpFx[148]*tmpObjS[168];
tmpQ2[65] = + tmpFx[5]*tmpObjS[0] + tmpFx[17]*tmpObjS[13] + tmpFx[29]*tmpObjS[26] + tmpFx[41]*tmpObjS[39] + tmpFx[53]*tmpObjS[52] + tmpFx[65]*tmpObjS[65] + tmpFx[77]*tmpObjS[78] + tmpFx[89]*tmpObjS[91] + tmpFx[101]*tmpObjS[104] + tmpFx[113]*tmpObjS[117] + tmpFx[125]*tmpObjS[130] + tmpFx[137]*tmpObjS[143] + tmpFx[149]*tmpObjS[156];
tmpQ2[66] = + tmpFx[5]*tmpObjS[1] + tmpFx[17]*tmpObjS[14] + tmpFx[29]*tmpObjS[27] + tmpFx[41]*tmpObjS[40] + tmpFx[53]*tmpObjS[53] + tmpFx[65]*tmpObjS[66] + tmpFx[77]*tmpObjS[79] + tmpFx[89]*tmpObjS[92] + tmpFx[101]*tmpObjS[105] + tmpFx[113]*tmpObjS[118] + tmpFx[125]*tmpObjS[131] + tmpFx[137]*tmpObjS[144] + tmpFx[149]*tmpObjS[157];
tmpQ2[67] = + tmpFx[5]*tmpObjS[2] + tmpFx[17]*tmpObjS[15] + tmpFx[29]*tmpObjS[28] + tmpFx[41]*tmpObjS[41] + tmpFx[53]*tmpObjS[54] + tmpFx[65]*tmpObjS[67] + tmpFx[77]*tmpObjS[80] + tmpFx[89]*tmpObjS[93] + tmpFx[101]*tmpObjS[106] + tmpFx[113]*tmpObjS[119] + tmpFx[125]*tmpObjS[132] + tmpFx[137]*tmpObjS[145] + tmpFx[149]*tmpObjS[158];
tmpQ2[68] = + tmpFx[5]*tmpObjS[3] + tmpFx[17]*tmpObjS[16] + tmpFx[29]*tmpObjS[29] + tmpFx[41]*tmpObjS[42] + tmpFx[53]*tmpObjS[55] + tmpFx[65]*tmpObjS[68] + tmpFx[77]*tmpObjS[81] + tmpFx[89]*tmpObjS[94] + tmpFx[101]*tmpObjS[107] + tmpFx[113]*tmpObjS[120] + tmpFx[125]*tmpObjS[133] + tmpFx[137]*tmpObjS[146] + tmpFx[149]*tmpObjS[159];
tmpQ2[69] = + tmpFx[5]*tmpObjS[4] + tmpFx[17]*tmpObjS[17] + tmpFx[29]*tmpObjS[30] + tmpFx[41]*tmpObjS[43] + tmpFx[53]*tmpObjS[56] + tmpFx[65]*tmpObjS[69] + tmpFx[77]*tmpObjS[82] + tmpFx[89]*tmpObjS[95] + tmpFx[101]*tmpObjS[108] + tmpFx[113]*tmpObjS[121] + tmpFx[125]*tmpObjS[134] + tmpFx[137]*tmpObjS[147] + tmpFx[149]*tmpObjS[160];
tmpQ2[70] = + tmpFx[5]*tmpObjS[5] + tmpFx[17]*tmpObjS[18] + tmpFx[29]*tmpObjS[31] + tmpFx[41]*tmpObjS[44] + tmpFx[53]*tmpObjS[57] + tmpFx[65]*tmpObjS[70] + tmpFx[77]*tmpObjS[83] + tmpFx[89]*tmpObjS[96] + tmpFx[101]*tmpObjS[109] + tmpFx[113]*tmpObjS[122] + tmpFx[125]*tmpObjS[135] + tmpFx[137]*tmpObjS[148] + tmpFx[149]*tmpObjS[161];
tmpQ2[71] = + tmpFx[5]*tmpObjS[6] + tmpFx[17]*tmpObjS[19] + tmpFx[29]*tmpObjS[32] + tmpFx[41]*tmpObjS[45] + tmpFx[53]*tmpObjS[58] + tmpFx[65]*tmpObjS[71] + tmpFx[77]*tmpObjS[84] + tmpFx[89]*tmpObjS[97] + tmpFx[101]*tmpObjS[110] + tmpFx[113]*tmpObjS[123] + tmpFx[125]*tmpObjS[136] + tmpFx[137]*tmpObjS[149] + tmpFx[149]*tmpObjS[162];
tmpQ2[72] = + tmpFx[5]*tmpObjS[7] + tmpFx[17]*tmpObjS[20] + tmpFx[29]*tmpObjS[33] + tmpFx[41]*tmpObjS[46] + tmpFx[53]*tmpObjS[59] + tmpFx[65]*tmpObjS[72] + tmpFx[77]*tmpObjS[85] + tmpFx[89]*tmpObjS[98] + tmpFx[101]*tmpObjS[111] + tmpFx[113]*tmpObjS[124] + tmpFx[125]*tmpObjS[137] + tmpFx[137]*tmpObjS[150] + tmpFx[149]*tmpObjS[163];
tmpQ2[73] = + tmpFx[5]*tmpObjS[8] + tmpFx[17]*tmpObjS[21] + tmpFx[29]*tmpObjS[34] + tmpFx[41]*tmpObjS[47] + tmpFx[53]*tmpObjS[60] + tmpFx[65]*tmpObjS[73] + tmpFx[77]*tmpObjS[86] + tmpFx[89]*tmpObjS[99] + tmpFx[101]*tmpObjS[112] + tmpFx[113]*tmpObjS[125] + tmpFx[125]*tmpObjS[138] + tmpFx[137]*tmpObjS[151] + tmpFx[149]*tmpObjS[164];
tmpQ2[74] = + tmpFx[5]*tmpObjS[9] + tmpFx[17]*tmpObjS[22] + tmpFx[29]*tmpObjS[35] + tmpFx[41]*tmpObjS[48] + tmpFx[53]*tmpObjS[61] + tmpFx[65]*tmpObjS[74] + tmpFx[77]*tmpObjS[87] + tmpFx[89]*tmpObjS[100] + tmpFx[101]*tmpObjS[113] + tmpFx[113]*tmpObjS[126] + tmpFx[125]*tmpObjS[139] + tmpFx[137]*tmpObjS[152] + tmpFx[149]*tmpObjS[165];
tmpQ2[75] = + tmpFx[5]*tmpObjS[10] + tmpFx[17]*tmpObjS[23] + tmpFx[29]*tmpObjS[36] + tmpFx[41]*tmpObjS[49] + tmpFx[53]*tmpObjS[62] + tmpFx[65]*tmpObjS[75] + tmpFx[77]*tmpObjS[88] + tmpFx[89]*tmpObjS[101] + tmpFx[101]*tmpObjS[114] + tmpFx[113]*tmpObjS[127] + tmpFx[125]*tmpObjS[140] + tmpFx[137]*tmpObjS[153] + tmpFx[149]*tmpObjS[166];
tmpQ2[76] = + tmpFx[5]*tmpObjS[11] + tmpFx[17]*tmpObjS[24] + tmpFx[29]*tmpObjS[37] + tmpFx[41]*tmpObjS[50] + tmpFx[53]*tmpObjS[63] + tmpFx[65]*tmpObjS[76] + tmpFx[77]*tmpObjS[89] + tmpFx[89]*tmpObjS[102] + tmpFx[101]*tmpObjS[115] + tmpFx[113]*tmpObjS[128] + tmpFx[125]*tmpObjS[141] + tmpFx[137]*tmpObjS[154] + tmpFx[149]*tmpObjS[167];
tmpQ2[77] = + tmpFx[5]*tmpObjS[12] + tmpFx[17]*tmpObjS[25] + tmpFx[29]*tmpObjS[38] + tmpFx[41]*tmpObjS[51] + tmpFx[53]*tmpObjS[64] + tmpFx[65]*tmpObjS[77] + tmpFx[77]*tmpObjS[90] + tmpFx[89]*tmpObjS[103] + tmpFx[101]*tmpObjS[116] + tmpFx[113]*tmpObjS[129] + tmpFx[125]*tmpObjS[142] + tmpFx[137]*tmpObjS[155] + tmpFx[149]*tmpObjS[168];
tmpQ2[78] = + tmpFx[6]*tmpObjS[0] + tmpFx[18]*tmpObjS[13] + tmpFx[30]*tmpObjS[26] + tmpFx[42]*tmpObjS[39] + tmpFx[54]*tmpObjS[52] + tmpFx[66]*tmpObjS[65] + tmpFx[78]*tmpObjS[78] + tmpFx[90]*tmpObjS[91] + tmpFx[102]*tmpObjS[104] + tmpFx[114]*tmpObjS[117] + tmpFx[126]*tmpObjS[130] + tmpFx[138]*tmpObjS[143] + tmpFx[150]*tmpObjS[156];
tmpQ2[79] = + tmpFx[6]*tmpObjS[1] + tmpFx[18]*tmpObjS[14] + tmpFx[30]*tmpObjS[27] + tmpFx[42]*tmpObjS[40] + tmpFx[54]*tmpObjS[53] + tmpFx[66]*tmpObjS[66] + tmpFx[78]*tmpObjS[79] + tmpFx[90]*tmpObjS[92] + tmpFx[102]*tmpObjS[105] + tmpFx[114]*tmpObjS[118] + tmpFx[126]*tmpObjS[131] + tmpFx[138]*tmpObjS[144] + tmpFx[150]*tmpObjS[157];
tmpQ2[80] = + tmpFx[6]*tmpObjS[2] + tmpFx[18]*tmpObjS[15] + tmpFx[30]*tmpObjS[28] + tmpFx[42]*tmpObjS[41] + tmpFx[54]*tmpObjS[54] + tmpFx[66]*tmpObjS[67] + tmpFx[78]*tmpObjS[80] + tmpFx[90]*tmpObjS[93] + tmpFx[102]*tmpObjS[106] + tmpFx[114]*tmpObjS[119] + tmpFx[126]*tmpObjS[132] + tmpFx[138]*tmpObjS[145] + tmpFx[150]*tmpObjS[158];
tmpQ2[81] = + tmpFx[6]*tmpObjS[3] + tmpFx[18]*tmpObjS[16] + tmpFx[30]*tmpObjS[29] + tmpFx[42]*tmpObjS[42] + tmpFx[54]*tmpObjS[55] + tmpFx[66]*tmpObjS[68] + tmpFx[78]*tmpObjS[81] + tmpFx[90]*tmpObjS[94] + tmpFx[102]*tmpObjS[107] + tmpFx[114]*tmpObjS[120] + tmpFx[126]*tmpObjS[133] + tmpFx[138]*tmpObjS[146] + tmpFx[150]*tmpObjS[159];
tmpQ2[82] = + tmpFx[6]*tmpObjS[4] + tmpFx[18]*tmpObjS[17] + tmpFx[30]*tmpObjS[30] + tmpFx[42]*tmpObjS[43] + tmpFx[54]*tmpObjS[56] + tmpFx[66]*tmpObjS[69] + tmpFx[78]*tmpObjS[82] + tmpFx[90]*tmpObjS[95] + tmpFx[102]*tmpObjS[108] + tmpFx[114]*tmpObjS[121] + tmpFx[126]*tmpObjS[134] + tmpFx[138]*tmpObjS[147] + tmpFx[150]*tmpObjS[160];
tmpQ2[83] = + tmpFx[6]*tmpObjS[5] + tmpFx[18]*tmpObjS[18] + tmpFx[30]*tmpObjS[31] + tmpFx[42]*tmpObjS[44] + tmpFx[54]*tmpObjS[57] + tmpFx[66]*tmpObjS[70] + tmpFx[78]*tmpObjS[83] + tmpFx[90]*tmpObjS[96] + tmpFx[102]*tmpObjS[109] + tmpFx[114]*tmpObjS[122] + tmpFx[126]*tmpObjS[135] + tmpFx[138]*tmpObjS[148] + tmpFx[150]*tmpObjS[161];
tmpQ2[84] = + tmpFx[6]*tmpObjS[6] + tmpFx[18]*tmpObjS[19] + tmpFx[30]*tmpObjS[32] + tmpFx[42]*tmpObjS[45] + tmpFx[54]*tmpObjS[58] + tmpFx[66]*tmpObjS[71] + tmpFx[78]*tmpObjS[84] + tmpFx[90]*tmpObjS[97] + tmpFx[102]*tmpObjS[110] + tmpFx[114]*tmpObjS[123] + tmpFx[126]*tmpObjS[136] + tmpFx[138]*tmpObjS[149] + tmpFx[150]*tmpObjS[162];
tmpQ2[85] = + tmpFx[6]*tmpObjS[7] + tmpFx[18]*tmpObjS[20] + tmpFx[30]*tmpObjS[33] + tmpFx[42]*tmpObjS[46] + tmpFx[54]*tmpObjS[59] + tmpFx[66]*tmpObjS[72] + tmpFx[78]*tmpObjS[85] + tmpFx[90]*tmpObjS[98] + tmpFx[102]*tmpObjS[111] + tmpFx[114]*tmpObjS[124] + tmpFx[126]*tmpObjS[137] + tmpFx[138]*tmpObjS[150] + tmpFx[150]*tmpObjS[163];
tmpQ2[86] = + tmpFx[6]*tmpObjS[8] + tmpFx[18]*tmpObjS[21] + tmpFx[30]*tmpObjS[34] + tmpFx[42]*tmpObjS[47] + tmpFx[54]*tmpObjS[60] + tmpFx[66]*tmpObjS[73] + tmpFx[78]*tmpObjS[86] + tmpFx[90]*tmpObjS[99] + tmpFx[102]*tmpObjS[112] + tmpFx[114]*tmpObjS[125] + tmpFx[126]*tmpObjS[138] + tmpFx[138]*tmpObjS[151] + tmpFx[150]*tmpObjS[164];
tmpQ2[87] = + tmpFx[6]*tmpObjS[9] + tmpFx[18]*tmpObjS[22] + tmpFx[30]*tmpObjS[35] + tmpFx[42]*tmpObjS[48] + tmpFx[54]*tmpObjS[61] + tmpFx[66]*tmpObjS[74] + tmpFx[78]*tmpObjS[87] + tmpFx[90]*tmpObjS[100] + tmpFx[102]*tmpObjS[113] + tmpFx[114]*tmpObjS[126] + tmpFx[126]*tmpObjS[139] + tmpFx[138]*tmpObjS[152] + tmpFx[150]*tmpObjS[165];
tmpQ2[88] = + tmpFx[6]*tmpObjS[10] + tmpFx[18]*tmpObjS[23] + tmpFx[30]*tmpObjS[36] + tmpFx[42]*tmpObjS[49] + tmpFx[54]*tmpObjS[62] + tmpFx[66]*tmpObjS[75] + tmpFx[78]*tmpObjS[88] + tmpFx[90]*tmpObjS[101] + tmpFx[102]*tmpObjS[114] + tmpFx[114]*tmpObjS[127] + tmpFx[126]*tmpObjS[140] + tmpFx[138]*tmpObjS[153] + tmpFx[150]*tmpObjS[166];
tmpQ2[89] = + tmpFx[6]*tmpObjS[11] + tmpFx[18]*tmpObjS[24] + tmpFx[30]*tmpObjS[37] + tmpFx[42]*tmpObjS[50] + tmpFx[54]*tmpObjS[63] + tmpFx[66]*tmpObjS[76] + tmpFx[78]*tmpObjS[89] + tmpFx[90]*tmpObjS[102] + tmpFx[102]*tmpObjS[115] + tmpFx[114]*tmpObjS[128] + tmpFx[126]*tmpObjS[141] + tmpFx[138]*tmpObjS[154] + tmpFx[150]*tmpObjS[167];
tmpQ2[90] = + tmpFx[6]*tmpObjS[12] + tmpFx[18]*tmpObjS[25] + tmpFx[30]*tmpObjS[38] + tmpFx[42]*tmpObjS[51] + tmpFx[54]*tmpObjS[64] + tmpFx[66]*tmpObjS[77] + tmpFx[78]*tmpObjS[90] + tmpFx[90]*tmpObjS[103] + tmpFx[102]*tmpObjS[116] + tmpFx[114]*tmpObjS[129] + tmpFx[126]*tmpObjS[142] + tmpFx[138]*tmpObjS[155] + tmpFx[150]*tmpObjS[168];
tmpQ2[91] = + tmpFx[7]*tmpObjS[0] + tmpFx[19]*tmpObjS[13] + tmpFx[31]*tmpObjS[26] + tmpFx[43]*tmpObjS[39] + tmpFx[55]*tmpObjS[52] + tmpFx[67]*tmpObjS[65] + tmpFx[79]*tmpObjS[78] + tmpFx[91]*tmpObjS[91] + tmpFx[103]*tmpObjS[104] + tmpFx[115]*tmpObjS[117] + tmpFx[127]*tmpObjS[130] + tmpFx[139]*tmpObjS[143] + tmpFx[151]*tmpObjS[156];
tmpQ2[92] = + tmpFx[7]*tmpObjS[1] + tmpFx[19]*tmpObjS[14] + tmpFx[31]*tmpObjS[27] + tmpFx[43]*tmpObjS[40] + tmpFx[55]*tmpObjS[53] + tmpFx[67]*tmpObjS[66] + tmpFx[79]*tmpObjS[79] + tmpFx[91]*tmpObjS[92] + tmpFx[103]*tmpObjS[105] + tmpFx[115]*tmpObjS[118] + tmpFx[127]*tmpObjS[131] + tmpFx[139]*tmpObjS[144] + tmpFx[151]*tmpObjS[157];
tmpQ2[93] = + tmpFx[7]*tmpObjS[2] + tmpFx[19]*tmpObjS[15] + tmpFx[31]*tmpObjS[28] + tmpFx[43]*tmpObjS[41] + tmpFx[55]*tmpObjS[54] + tmpFx[67]*tmpObjS[67] + tmpFx[79]*tmpObjS[80] + tmpFx[91]*tmpObjS[93] + tmpFx[103]*tmpObjS[106] + tmpFx[115]*tmpObjS[119] + tmpFx[127]*tmpObjS[132] + tmpFx[139]*tmpObjS[145] + tmpFx[151]*tmpObjS[158];
tmpQ2[94] = + tmpFx[7]*tmpObjS[3] + tmpFx[19]*tmpObjS[16] + tmpFx[31]*tmpObjS[29] + tmpFx[43]*tmpObjS[42] + tmpFx[55]*tmpObjS[55] + tmpFx[67]*tmpObjS[68] + tmpFx[79]*tmpObjS[81] + tmpFx[91]*tmpObjS[94] + tmpFx[103]*tmpObjS[107] + tmpFx[115]*tmpObjS[120] + tmpFx[127]*tmpObjS[133] + tmpFx[139]*tmpObjS[146] + tmpFx[151]*tmpObjS[159];
tmpQ2[95] = + tmpFx[7]*tmpObjS[4] + tmpFx[19]*tmpObjS[17] + tmpFx[31]*tmpObjS[30] + tmpFx[43]*tmpObjS[43] + tmpFx[55]*tmpObjS[56] + tmpFx[67]*tmpObjS[69] + tmpFx[79]*tmpObjS[82] + tmpFx[91]*tmpObjS[95] + tmpFx[103]*tmpObjS[108] + tmpFx[115]*tmpObjS[121] + tmpFx[127]*tmpObjS[134] + tmpFx[139]*tmpObjS[147] + tmpFx[151]*tmpObjS[160];
tmpQ2[96] = + tmpFx[7]*tmpObjS[5] + tmpFx[19]*tmpObjS[18] + tmpFx[31]*tmpObjS[31] + tmpFx[43]*tmpObjS[44] + tmpFx[55]*tmpObjS[57] + tmpFx[67]*tmpObjS[70] + tmpFx[79]*tmpObjS[83] + tmpFx[91]*tmpObjS[96] + tmpFx[103]*tmpObjS[109] + tmpFx[115]*tmpObjS[122] + tmpFx[127]*tmpObjS[135] + tmpFx[139]*tmpObjS[148] + tmpFx[151]*tmpObjS[161];
tmpQ2[97] = + tmpFx[7]*tmpObjS[6] + tmpFx[19]*tmpObjS[19] + tmpFx[31]*tmpObjS[32] + tmpFx[43]*tmpObjS[45] + tmpFx[55]*tmpObjS[58] + tmpFx[67]*tmpObjS[71] + tmpFx[79]*tmpObjS[84] + tmpFx[91]*tmpObjS[97] + tmpFx[103]*tmpObjS[110] + tmpFx[115]*tmpObjS[123] + tmpFx[127]*tmpObjS[136] + tmpFx[139]*tmpObjS[149] + tmpFx[151]*tmpObjS[162];
tmpQ2[98] = + tmpFx[7]*tmpObjS[7] + tmpFx[19]*tmpObjS[20] + tmpFx[31]*tmpObjS[33] + tmpFx[43]*tmpObjS[46] + tmpFx[55]*tmpObjS[59] + tmpFx[67]*tmpObjS[72] + tmpFx[79]*tmpObjS[85] + tmpFx[91]*tmpObjS[98] + tmpFx[103]*tmpObjS[111] + tmpFx[115]*tmpObjS[124] + tmpFx[127]*tmpObjS[137] + tmpFx[139]*tmpObjS[150] + tmpFx[151]*tmpObjS[163];
tmpQ2[99] = + tmpFx[7]*tmpObjS[8] + tmpFx[19]*tmpObjS[21] + tmpFx[31]*tmpObjS[34] + tmpFx[43]*tmpObjS[47] + tmpFx[55]*tmpObjS[60] + tmpFx[67]*tmpObjS[73] + tmpFx[79]*tmpObjS[86] + tmpFx[91]*tmpObjS[99] + tmpFx[103]*tmpObjS[112] + tmpFx[115]*tmpObjS[125] + tmpFx[127]*tmpObjS[138] + tmpFx[139]*tmpObjS[151] + tmpFx[151]*tmpObjS[164];
tmpQ2[100] = + tmpFx[7]*tmpObjS[9] + tmpFx[19]*tmpObjS[22] + tmpFx[31]*tmpObjS[35] + tmpFx[43]*tmpObjS[48] + tmpFx[55]*tmpObjS[61] + tmpFx[67]*tmpObjS[74] + tmpFx[79]*tmpObjS[87] + tmpFx[91]*tmpObjS[100] + tmpFx[103]*tmpObjS[113] + tmpFx[115]*tmpObjS[126] + tmpFx[127]*tmpObjS[139] + tmpFx[139]*tmpObjS[152] + tmpFx[151]*tmpObjS[165];
tmpQ2[101] = + tmpFx[7]*tmpObjS[10] + tmpFx[19]*tmpObjS[23] + tmpFx[31]*tmpObjS[36] + tmpFx[43]*tmpObjS[49] + tmpFx[55]*tmpObjS[62] + tmpFx[67]*tmpObjS[75] + tmpFx[79]*tmpObjS[88] + tmpFx[91]*tmpObjS[101] + tmpFx[103]*tmpObjS[114] + tmpFx[115]*tmpObjS[127] + tmpFx[127]*tmpObjS[140] + tmpFx[139]*tmpObjS[153] + tmpFx[151]*tmpObjS[166];
tmpQ2[102] = + tmpFx[7]*tmpObjS[11] + tmpFx[19]*tmpObjS[24] + tmpFx[31]*tmpObjS[37] + tmpFx[43]*tmpObjS[50] + tmpFx[55]*tmpObjS[63] + tmpFx[67]*tmpObjS[76] + tmpFx[79]*tmpObjS[89] + tmpFx[91]*tmpObjS[102] + tmpFx[103]*tmpObjS[115] + tmpFx[115]*tmpObjS[128] + tmpFx[127]*tmpObjS[141] + tmpFx[139]*tmpObjS[154] + tmpFx[151]*tmpObjS[167];
tmpQ2[103] = + tmpFx[7]*tmpObjS[12] + tmpFx[19]*tmpObjS[25] + tmpFx[31]*tmpObjS[38] + tmpFx[43]*tmpObjS[51] + tmpFx[55]*tmpObjS[64] + tmpFx[67]*tmpObjS[77] + tmpFx[79]*tmpObjS[90] + tmpFx[91]*tmpObjS[103] + tmpFx[103]*tmpObjS[116] + tmpFx[115]*tmpObjS[129] + tmpFx[127]*tmpObjS[142] + tmpFx[139]*tmpObjS[155] + tmpFx[151]*tmpObjS[168];
tmpQ2[104] = + tmpFx[8]*tmpObjS[0] + tmpFx[20]*tmpObjS[13] + tmpFx[32]*tmpObjS[26] + tmpFx[44]*tmpObjS[39] + tmpFx[56]*tmpObjS[52] + tmpFx[68]*tmpObjS[65] + tmpFx[80]*tmpObjS[78] + tmpFx[92]*tmpObjS[91] + tmpFx[104]*tmpObjS[104] + tmpFx[116]*tmpObjS[117] + tmpFx[128]*tmpObjS[130] + tmpFx[140]*tmpObjS[143] + tmpFx[152]*tmpObjS[156];
tmpQ2[105] = + tmpFx[8]*tmpObjS[1] + tmpFx[20]*tmpObjS[14] + tmpFx[32]*tmpObjS[27] + tmpFx[44]*tmpObjS[40] + tmpFx[56]*tmpObjS[53] + tmpFx[68]*tmpObjS[66] + tmpFx[80]*tmpObjS[79] + tmpFx[92]*tmpObjS[92] + tmpFx[104]*tmpObjS[105] + tmpFx[116]*tmpObjS[118] + tmpFx[128]*tmpObjS[131] + tmpFx[140]*tmpObjS[144] + tmpFx[152]*tmpObjS[157];
tmpQ2[106] = + tmpFx[8]*tmpObjS[2] + tmpFx[20]*tmpObjS[15] + tmpFx[32]*tmpObjS[28] + tmpFx[44]*tmpObjS[41] + tmpFx[56]*tmpObjS[54] + tmpFx[68]*tmpObjS[67] + tmpFx[80]*tmpObjS[80] + tmpFx[92]*tmpObjS[93] + tmpFx[104]*tmpObjS[106] + tmpFx[116]*tmpObjS[119] + tmpFx[128]*tmpObjS[132] + tmpFx[140]*tmpObjS[145] + tmpFx[152]*tmpObjS[158];
tmpQ2[107] = + tmpFx[8]*tmpObjS[3] + tmpFx[20]*tmpObjS[16] + tmpFx[32]*tmpObjS[29] + tmpFx[44]*tmpObjS[42] + tmpFx[56]*tmpObjS[55] + tmpFx[68]*tmpObjS[68] + tmpFx[80]*tmpObjS[81] + tmpFx[92]*tmpObjS[94] + tmpFx[104]*tmpObjS[107] + tmpFx[116]*tmpObjS[120] + tmpFx[128]*tmpObjS[133] + tmpFx[140]*tmpObjS[146] + tmpFx[152]*tmpObjS[159];
tmpQ2[108] = + tmpFx[8]*tmpObjS[4] + tmpFx[20]*tmpObjS[17] + tmpFx[32]*tmpObjS[30] + tmpFx[44]*tmpObjS[43] + tmpFx[56]*tmpObjS[56] + tmpFx[68]*tmpObjS[69] + tmpFx[80]*tmpObjS[82] + tmpFx[92]*tmpObjS[95] + tmpFx[104]*tmpObjS[108] + tmpFx[116]*tmpObjS[121] + tmpFx[128]*tmpObjS[134] + tmpFx[140]*tmpObjS[147] + tmpFx[152]*tmpObjS[160];
tmpQ2[109] = + tmpFx[8]*tmpObjS[5] + tmpFx[20]*tmpObjS[18] + tmpFx[32]*tmpObjS[31] + tmpFx[44]*tmpObjS[44] + tmpFx[56]*tmpObjS[57] + tmpFx[68]*tmpObjS[70] + tmpFx[80]*tmpObjS[83] + tmpFx[92]*tmpObjS[96] + tmpFx[104]*tmpObjS[109] + tmpFx[116]*tmpObjS[122] + tmpFx[128]*tmpObjS[135] + tmpFx[140]*tmpObjS[148] + tmpFx[152]*tmpObjS[161];
tmpQ2[110] = + tmpFx[8]*tmpObjS[6] + tmpFx[20]*tmpObjS[19] + tmpFx[32]*tmpObjS[32] + tmpFx[44]*tmpObjS[45] + tmpFx[56]*tmpObjS[58] + tmpFx[68]*tmpObjS[71] + tmpFx[80]*tmpObjS[84] + tmpFx[92]*tmpObjS[97] + tmpFx[104]*tmpObjS[110] + tmpFx[116]*tmpObjS[123] + tmpFx[128]*tmpObjS[136] + tmpFx[140]*tmpObjS[149] + tmpFx[152]*tmpObjS[162];
tmpQ2[111] = + tmpFx[8]*tmpObjS[7] + tmpFx[20]*tmpObjS[20] + tmpFx[32]*tmpObjS[33] + tmpFx[44]*tmpObjS[46] + tmpFx[56]*tmpObjS[59] + tmpFx[68]*tmpObjS[72] + tmpFx[80]*tmpObjS[85] + tmpFx[92]*tmpObjS[98] + tmpFx[104]*tmpObjS[111] + tmpFx[116]*tmpObjS[124] + tmpFx[128]*tmpObjS[137] + tmpFx[140]*tmpObjS[150] + tmpFx[152]*tmpObjS[163];
tmpQ2[112] = + tmpFx[8]*tmpObjS[8] + tmpFx[20]*tmpObjS[21] + tmpFx[32]*tmpObjS[34] + tmpFx[44]*tmpObjS[47] + tmpFx[56]*tmpObjS[60] + tmpFx[68]*tmpObjS[73] + tmpFx[80]*tmpObjS[86] + tmpFx[92]*tmpObjS[99] + tmpFx[104]*tmpObjS[112] + tmpFx[116]*tmpObjS[125] + tmpFx[128]*tmpObjS[138] + tmpFx[140]*tmpObjS[151] + tmpFx[152]*tmpObjS[164];
tmpQ2[113] = + tmpFx[8]*tmpObjS[9] + tmpFx[20]*tmpObjS[22] + tmpFx[32]*tmpObjS[35] + tmpFx[44]*tmpObjS[48] + tmpFx[56]*tmpObjS[61] + tmpFx[68]*tmpObjS[74] + tmpFx[80]*tmpObjS[87] + tmpFx[92]*tmpObjS[100] + tmpFx[104]*tmpObjS[113] + tmpFx[116]*tmpObjS[126] + tmpFx[128]*tmpObjS[139] + tmpFx[140]*tmpObjS[152] + tmpFx[152]*tmpObjS[165];
tmpQ2[114] = + tmpFx[8]*tmpObjS[10] + tmpFx[20]*tmpObjS[23] + tmpFx[32]*tmpObjS[36] + tmpFx[44]*tmpObjS[49] + tmpFx[56]*tmpObjS[62] + tmpFx[68]*tmpObjS[75] + tmpFx[80]*tmpObjS[88] + tmpFx[92]*tmpObjS[101] + tmpFx[104]*tmpObjS[114] + tmpFx[116]*tmpObjS[127] + tmpFx[128]*tmpObjS[140] + tmpFx[140]*tmpObjS[153] + tmpFx[152]*tmpObjS[166];
tmpQ2[115] = + tmpFx[8]*tmpObjS[11] + tmpFx[20]*tmpObjS[24] + tmpFx[32]*tmpObjS[37] + tmpFx[44]*tmpObjS[50] + tmpFx[56]*tmpObjS[63] + tmpFx[68]*tmpObjS[76] + tmpFx[80]*tmpObjS[89] + tmpFx[92]*tmpObjS[102] + tmpFx[104]*tmpObjS[115] + tmpFx[116]*tmpObjS[128] + tmpFx[128]*tmpObjS[141] + tmpFx[140]*tmpObjS[154] + tmpFx[152]*tmpObjS[167];
tmpQ2[116] = + tmpFx[8]*tmpObjS[12] + tmpFx[20]*tmpObjS[25] + tmpFx[32]*tmpObjS[38] + tmpFx[44]*tmpObjS[51] + tmpFx[56]*tmpObjS[64] + tmpFx[68]*tmpObjS[77] + tmpFx[80]*tmpObjS[90] + tmpFx[92]*tmpObjS[103] + tmpFx[104]*tmpObjS[116] + tmpFx[116]*tmpObjS[129] + tmpFx[128]*tmpObjS[142] + tmpFx[140]*tmpObjS[155] + tmpFx[152]*tmpObjS[168];
tmpQ2[117] = + tmpFx[9]*tmpObjS[0] + tmpFx[21]*tmpObjS[13] + tmpFx[33]*tmpObjS[26] + tmpFx[45]*tmpObjS[39] + tmpFx[57]*tmpObjS[52] + tmpFx[69]*tmpObjS[65] + tmpFx[81]*tmpObjS[78] + tmpFx[93]*tmpObjS[91] + tmpFx[105]*tmpObjS[104] + tmpFx[117]*tmpObjS[117] + tmpFx[129]*tmpObjS[130] + tmpFx[141]*tmpObjS[143] + tmpFx[153]*tmpObjS[156];
tmpQ2[118] = + tmpFx[9]*tmpObjS[1] + tmpFx[21]*tmpObjS[14] + tmpFx[33]*tmpObjS[27] + tmpFx[45]*tmpObjS[40] + tmpFx[57]*tmpObjS[53] + tmpFx[69]*tmpObjS[66] + tmpFx[81]*tmpObjS[79] + tmpFx[93]*tmpObjS[92] + tmpFx[105]*tmpObjS[105] + tmpFx[117]*tmpObjS[118] + tmpFx[129]*tmpObjS[131] + tmpFx[141]*tmpObjS[144] + tmpFx[153]*tmpObjS[157];
tmpQ2[119] = + tmpFx[9]*tmpObjS[2] + tmpFx[21]*tmpObjS[15] + tmpFx[33]*tmpObjS[28] + tmpFx[45]*tmpObjS[41] + tmpFx[57]*tmpObjS[54] + tmpFx[69]*tmpObjS[67] + tmpFx[81]*tmpObjS[80] + tmpFx[93]*tmpObjS[93] + tmpFx[105]*tmpObjS[106] + tmpFx[117]*tmpObjS[119] + tmpFx[129]*tmpObjS[132] + tmpFx[141]*tmpObjS[145] + tmpFx[153]*tmpObjS[158];
tmpQ2[120] = + tmpFx[9]*tmpObjS[3] + tmpFx[21]*tmpObjS[16] + tmpFx[33]*tmpObjS[29] + tmpFx[45]*tmpObjS[42] + tmpFx[57]*tmpObjS[55] + tmpFx[69]*tmpObjS[68] + tmpFx[81]*tmpObjS[81] + tmpFx[93]*tmpObjS[94] + tmpFx[105]*tmpObjS[107] + tmpFx[117]*tmpObjS[120] + tmpFx[129]*tmpObjS[133] + tmpFx[141]*tmpObjS[146] + tmpFx[153]*tmpObjS[159];
tmpQ2[121] = + tmpFx[9]*tmpObjS[4] + tmpFx[21]*tmpObjS[17] + tmpFx[33]*tmpObjS[30] + tmpFx[45]*tmpObjS[43] + tmpFx[57]*tmpObjS[56] + tmpFx[69]*tmpObjS[69] + tmpFx[81]*tmpObjS[82] + tmpFx[93]*tmpObjS[95] + tmpFx[105]*tmpObjS[108] + tmpFx[117]*tmpObjS[121] + tmpFx[129]*tmpObjS[134] + tmpFx[141]*tmpObjS[147] + tmpFx[153]*tmpObjS[160];
tmpQ2[122] = + tmpFx[9]*tmpObjS[5] + tmpFx[21]*tmpObjS[18] + tmpFx[33]*tmpObjS[31] + tmpFx[45]*tmpObjS[44] + tmpFx[57]*tmpObjS[57] + tmpFx[69]*tmpObjS[70] + tmpFx[81]*tmpObjS[83] + tmpFx[93]*tmpObjS[96] + tmpFx[105]*tmpObjS[109] + tmpFx[117]*tmpObjS[122] + tmpFx[129]*tmpObjS[135] + tmpFx[141]*tmpObjS[148] + tmpFx[153]*tmpObjS[161];
tmpQ2[123] = + tmpFx[9]*tmpObjS[6] + tmpFx[21]*tmpObjS[19] + tmpFx[33]*tmpObjS[32] + tmpFx[45]*tmpObjS[45] + tmpFx[57]*tmpObjS[58] + tmpFx[69]*tmpObjS[71] + tmpFx[81]*tmpObjS[84] + tmpFx[93]*tmpObjS[97] + tmpFx[105]*tmpObjS[110] + tmpFx[117]*tmpObjS[123] + tmpFx[129]*tmpObjS[136] + tmpFx[141]*tmpObjS[149] + tmpFx[153]*tmpObjS[162];
tmpQ2[124] = + tmpFx[9]*tmpObjS[7] + tmpFx[21]*tmpObjS[20] + tmpFx[33]*tmpObjS[33] + tmpFx[45]*tmpObjS[46] + tmpFx[57]*tmpObjS[59] + tmpFx[69]*tmpObjS[72] + tmpFx[81]*tmpObjS[85] + tmpFx[93]*tmpObjS[98] + tmpFx[105]*tmpObjS[111] + tmpFx[117]*tmpObjS[124] + tmpFx[129]*tmpObjS[137] + tmpFx[141]*tmpObjS[150] + tmpFx[153]*tmpObjS[163];
tmpQ2[125] = + tmpFx[9]*tmpObjS[8] + tmpFx[21]*tmpObjS[21] + tmpFx[33]*tmpObjS[34] + tmpFx[45]*tmpObjS[47] + tmpFx[57]*tmpObjS[60] + tmpFx[69]*tmpObjS[73] + tmpFx[81]*tmpObjS[86] + tmpFx[93]*tmpObjS[99] + tmpFx[105]*tmpObjS[112] + tmpFx[117]*tmpObjS[125] + tmpFx[129]*tmpObjS[138] + tmpFx[141]*tmpObjS[151] + tmpFx[153]*tmpObjS[164];
tmpQ2[126] = + tmpFx[9]*tmpObjS[9] + tmpFx[21]*tmpObjS[22] + tmpFx[33]*tmpObjS[35] + tmpFx[45]*tmpObjS[48] + tmpFx[57]*tmpObjS[61] + tmpFx[69]*tmpObjS[74] + tmpFx[81]*tmpObjS[87] + tmpFx[93]*tmpObjS[100] + tmpFx[105]*tmpObjS[113] + tmpFx[117]*tmpObjS[126] + tmpFx[129]*tmpObjS[139] + tmpFx[141]*tmpObjS[152] + tmpFx[153]*tmpObjS[165];
tmpQ2[127] = + tmpFx[9]*tmpObjS[10] + tmpFx[21]*tmpObjS[23] + tmpFx[33]*tmpObjS[36] + tmpFx[45]*tmpObjS[49] + tmpFx[57]*tmpObjS[62] + tmpFx[69]*tmpObjS[75] + tmpFx[81]*tmpObjS[88] + tmpFx[93]*tmpObjS[101] + tmpFx[105]*tmpObjS[114] + tmpFx[117]*tmpObjS[127] + tmpFx[129]*tmpObjS[140] + tmpFx[141]*tmpObjS[153] + tmpFx[153]*tmpObjS[166];
tmpQ2[128] = + tmpFx[9]*tmpObjS[11] + tmpFx[21]*tmpObjS[24] + tmpFx[33]*tmpObjS[37] + tmpFx[45]*tmpObjS[50] + tmpFx[57]*tmpObjS[63] + tmpFx[69]*tmpObjS[76] + tmpFx[81]*tmpObjS[89] + tmpFx[93]*tmpObjS[102] + tmpFx[105]*tmpObjS[115] + tmpFx[117]*tmpObjS[128] + tmpFx[129]*tmpObjS[141] + tmpFx[141]*tmpObjS[154] + tmpFx[153]*tmpObjS[167];
tmpQ2[129] = + tmpFx[9]*tmpObjS[12] + tmpFx[21]*tmpObjS[25] + tmpFx[33]*tmpObjS[38] + tmpFx[45]*tmpObjS[51] + tmpFx[57]*tmpObjS[64] + tmpFx[69]*tmpObjS[77] + tmpFx[81]*tmpObjS[90] + tmpFx[93]*tmpObjS[103] + tmpFx[105]*tmpObjS[116] + tmpFx[117]*tmpObjS[129] + tmpFx[129]*tmpObjS[142] + tmpFx[141]*tmpObjS[155] + tmpFx[153]*tmpObjS[168];
tmpQ2[130] = + tmpFx[10]*tmpObjS[0] + tmpFx[22]*tmpObjS[13] + tmpFx[34]*tmpObjS[26] + tmpFx[46]*tmpObjS[39] + tmpFx[58]*tmpObjS[52] + tmpFx[70]*tmpObjS[65] + tmpFx[82]*tmpObjS[78] + tmpFx[94]*tmpObjS[91] + tmpFx[106]*tmpObjS[104] + tmpFx[118]*tmpObjS[117] + tmpFx[130]*tmpObjS[130] + tmpFx[142]*tmpObjS[143] + tmpFx[154]*tmpObjS[156];
tmpQ2[131] = + tmpFx[10]*tmpObjS[1] + tmpFx[22]*tmpObjS[14] + tmpFx[34]*tmpObjS[27] + tmpFx[46]*tmpObjS[40] + tmpFx[58]*tmpObjS[53] + tmpFx[70]*tmpObjS[66] + tmpFx[82]*tmpObjS[79] + tmpFx[94]*tmpObjS[92] + tmpFx[106]*tmpObjS[105] + tmpFx[118]*tmpObjS[118] + tmpFx[130]*tmpObjS[131] + tmpFx[142]*tmpObjS[144] + tmpFx[154]*tmpObjS[157];
tmpQ2[132] = + tmpFx[10]*tmpObjS[2] + tmpFx[22]*tmpObjS[15] + tmpFx[34]*tmpObjS[28] + tmpFx[46]*tmpObjS[41] + tmpFx[58]*tmpObjS[54] + tmpFx[70]*tmpObjS[67] + tmpFx[82]*tmpObjS[80] + tmpFx[94]*tmpObjS[93] + tmpFx[106]*tmpObjS[106] + tmpFx[118]*tmpObjS[119] + tmpFx[130]*tmpObjS[132] + tmpFx[142]*tmpObjS[145] + tmpFx[154]*tmpObjS[158];
tmpQ2[133] = + tmpFx[10]*tmpObjS[3] + tmpFx[22]*tmpObjS[16] + tmpFx[34]*tmpObjS[29] + tmpFx[46]*tmpObjS[42] + tmpFx[58]*tmpObjS[55] + tmpFx[70]*tmpObjS[68] + tmpFx[82]*tmpObjS[81] + tmpFx[94]*tmpObjS[94] + tmpFx[106]*tmpObjS[107] + tmpFx[118]*tmpObjS[120] + tmpFx[130]*tmpObjS[133] + tmpFx[142]*tmpObjS[146] + tmpFx[154]*tmpObjS[159];
tmpQ2[134] = + tmpFx[10]*tmpObjS[4] + tmpFx[22]*tmpObjS[17] + tmpFx[34]*tmpObjS[30] + tmpFx[46]*tmpObjS[43] + tmpFx[58]*tmpObjS[56] + tmpFx[70]*tmpObjS[69] + tmpFx[82]*tmpObjS[82] + tmpFx[94]*tmpObjS[95] + tmpFx[106]*tmpObjS[108] + tmpFx[118]*tmpObjS[121] + tmpFx[130]*tmpObjS[134] + tmpFx[142]*tmpObjS[147] + tmpFx[154]*tmpObjS[160];
tmpQ2[135] = + tmpFx[10]*tmpObjS[5] + tmpFx[22]*tmpObjS[18] + tmpFx[34]*tmpObjS[31] + tmpFx[46]*tmpObjS[44] + tmpFx[58]*tmpObjS[57] + tmpFx[70]*tmpObjS[70] + tmpFx[82]*tmpObjS[83] + tmpFx[94]*tmpObjS[96] + tmpFx[106]*tmpObjS[109] + tmpFx[118]*tmpObjS[122] + tmpFx[130]*tmpObjS[135] + tmpFx[142]*tmpObjS[148] + tmpFx[154]*tmpObjS[161];
tmpQ2[136] = + tmpFx[10]*tmpObjS[6] + tmpFx[22]*tmpObjS[19] + tmpFx[34]*tmpObjS[32] + tmpFx[46]*tmpObjS[45] + tmpFx[58]*tmpObjS[58] + tmpFx[70]*tmpObjS[71] + tmpFx[82]*tmpObjS[84] + tmpFx[94]*tmpObjS[97] + tmpFx[106]*tmpObjS[110] + tmpFx[118]*tmpObjS[123] + tmpFx[130]*tmpObjS[136] + tmpFx[142]*tmpObjS[149] + tmpFx[154]*tmpObjS[162];
tmpQ2[137] = + tmpFx[10]*tmpObjS[7] + tmpFx[22]*tmpObjS[20] + tmpFx[34]*tmpObjS[33] + tmpFx[46]*tmpObjS[46] + tmpFx[58]*tmpObjS[59] + tmpFx[70]*tmpObjS[72] + tmpFx[82]*tmpObjS[85] + tmpFx[94]*tmpObjS[98] + tmpFx[106]*tmpObjS[111] + tmpFx[118]*tmpObjS[124] + tmpFx[130]*tmpObjS[137] + tmpFx[142]*tmpObjS[150] + tmpFx[154]*tmpObjS[163];
tmpQ2[138] = + tmpFx[10]*tmpObjS[8] + tmpFx[22]*tmpObjS[21] + tmpFx[34]*tmpObjS[34] + tmpFx[46]*tmpObjS[47] + tmpFx[58]*tmpObjS[60] + tmpFx[70]*tmpObjS[73] + tmpFx[82]*tmpObjS[86] + tmpFx[94]*tmpObjS[99] + tmpFx[106]*tmpObjS[112] + tmpFx[118]*tmpObjS[125] + tmpFx[130]*tmpObjS[138] + tmpFx[142]*tmpObjS[151] + tmpFx[154]*tmpObjS[164];
tmpQ2[139] = + tmpFx[10]*tmpObjS[9] + tmpFx[22]*tmpObjS[22] + tmpFx[34]*tmpObjS[35] + tmpFx[46]*tmpObjS[48] + tmpFx[58]*tmpObjS[61] + tmpFx[70]*tmpObjS[74] + tmpFx[82]*tmpObjS[87] + tmpFx[94]*tmpObjS[100] + tmpFx[106]*tmpObjS[113] + tmpFx[118]*tmpObjS[126] + tmpFx[130]*tmpObjS[139] + tmpFx[142]*tmpObjS[152] + tmpFx[154]*tmpObjS[165];
tmpQ2[140] = + tmpFx[10]*tmpObjS[10] + tmpFx[22]*tmpObjS[23] + tmpFx[34]*tmpObjS[36] + tmpFx[46]*tmpObjS[49] + tmpFx[58]*tmpObjS[62] + tmpFx[70]*tmpObjS[75] + tmpFx[82]*tmpObjS[88] + tmpFx[94]*tmpObjS[101] + tmpFx[106]*tmpObjS[114] + tmpFx[118]*tmpObjS[127] + tmpFx[130]*tmpObjS[140] + tmpFx[142]*tmpObjS[153] + tmpFx[154]*tmpObjS[166];
tmpQ2[141] = + tmpFx[10]*tmpObjS[11] + tmpFx[22]*tmpObjS[24] + tmpFx[34]*tmpObjS[37] + tmpFx[46]*tmpObjS[50] + tmpFx[58]*tmpObjS[63] + tmpFx[70]*tmpObjS[76] + tmpFx[82]*tmpObjS[89] + tmpFx[94]*tmpObjS[102] + tmpFx[106]*tmpObjS[115] + tmpFx[118]*tmpObjS[128] + tmpFx[130]*tmpObjS[141] + tmpFx[142]*tmpObjS[154] + tmpFx[154]*tmpObjS[167];
tmpQ2[142] = + tmpFx[10]*tmpObjS[12] + tmpFx[22]*tmpObjS[25] + tmpFx[34]*tmpObjS[38] + tmpFx[46]*tmpObjS[51] + tmpFx[58]*tmpObjS[64] + tmpFx[70]*tmpObjS[77] + tmpFx[82]*tmpObjS[90] + tmpFx[94]*tmpObjS[103] + tmpFx[106]*tmpObjS[116] + tmpFx[118]*tmpObjS[129] + tmpFx[130]*tmpObjS[142] + tmpFx[142]*tmpObjS[155] + tmpFx[154]*tmpObjS[168];
tmpQ2[143] = + tmpFx[11]*tmpObjS[0] + tmpFx[23]*tmpObjS[13] + tmpFx[35]*tmpObjS[26] + tmpFx[47]*tmpObjS[39] + tmpFx[59]*tmpObjS[52] + tmpFx[71]*tmpObjS[65] + tmpFx[83]*tmpObjS[78] + tmpFx[95]*tmpObjS[91] + tmpFx[107]*tmpObjS[104] + tmpFx[119]*tmpObjS[117] + tmpFx[131]*tmpObjS[130] + tmpFx[143]*tmpObjS[143] + tmpFx[155]*tmpObjS[156];
tmpQ2[144] = + tmpFx[11]*tmpObjS[1] + tmpFx[23]*tmpObjS[14] + tmpFx[35]*tmpObjS[27] + tmpFx[47]*tmpObjS[40] + tmpFx[59]*tmpObjS[53] + tmpFx[71]*tmpObjS[66] + tmpFx[83]*tmpObjS[79] + tmpFx[95]*tmpObjS[92] + tmpFx[107]*tmpObjS[105] + tmpFx[119]*tmpObjS[118] + tmpFx[131]*tmpObjS[131] + tmpFx[143]*tmpObjS[144] + tmpFx[155]*tmpObjS[157];
tmpQ2[145] = + tmpFx[11]*tmpObjS[2] + tmpFx[23]*tmpObjS[15] + tmpFx[35]*tmpObjS[28] + tmpFx[47]*tmpObjS[41] + tmpFx[59]*tmpObjS[54] + tmpFx[71]*tmpObjS[67] + tmpFx[83]*tmpObjS[80] + tmpFx[95]*tmpObjS[93] + tmpFx[107]*tmpObjS[106] + tmpFx[119]*tmpObjS[119] + tmpFx[131]*tmpObjS[132] + tmpFx[143]*tmpObjS[145] + tmpFx[155]*tmpObjS[158];
tmpQ2[146] = + tmpFx[11]*tmpObjS[3] + tmpFx[23]*tmpObjS[16] + tmpFx[35]*tmpObjS[29] + tmpFx[47]*tmpObjS[42] + tmpFx[59]*tmpObjS[55] + tmpFx[71]*tmpObjS[68] + tmpFx[83]*tmpObjS[81] + tmpFx[95]*tmpObjS[94] + tmpFx[107]*tmpObjS[107] + tmpFx[119]*tmpObjS[120] + tmpFx[131]*tmpObjS[133] + tmpFx[143]*tmpObjS[146] + tmpFx[155]*tmpObjS[159];
tmpQ2[147] = + tmpFx[11]*tmpObjS[4] + tmpFx[23]*tmpObjS[17] + tmpFx[35]*tmpObjS[30] + tmpFx[47]*tmpObjS[43] + tmpFx[59]*tmpObjS[56] + tmpFx[71]*tmpObjS[69] + tmpFx[83]*tmpObjS[82] + tmpFx[95]*tmpObjS[95] + tmpFx[107]*tmpObjS[108] + tmpFx[119]*tmpObjS[121] + tmpFx[131]*tmpObjS[134] + tmpFx[143]*tmpObjS[147] + tmpFx[155]*tmpObjS[160];
tmpQ2[148] = + tmpFx[11]*tmpObjS[5] + tmpFx[23]*tmpObjS[18] + tmpFx[35]*tmpObjS[31] + tmpFx[47]*tmpObjS[44] + tmpFx[59]*tmpObjS[57] + tmpFx[71]*tmpObjS[70] + tmpFx[83]*tmpObjS[83] + tmpFx[95]*tmpObjS[96] + tmpFx[107]*tmpObjS[109] + tmpFx[119]*tmpObjS[122] + tmpFx[131]*tmpObjS[135] + tmpFx[143]*tmpObjS[148] + tmpFx[155]*tmpObjS[161];
tmpQ2[149] = + tmpFx[11]*tmpObjS[6] + tmpFx[23]*tmpObjS[19] + tmpFx[35]*tmpObjS[32] + tmpFx[47]*tmpObjS[45] + tmpFx[59]*tmpObjS[58] + tmpFx[71]*tmpObjS[71] + tmpFx[83]*tmpObjS[84] + tmpFx[95]*tmpObjS[97] + tmpFx[107]*tmpObjS[110] + tmpFx[119]*tmpObjS[123] + tmpFx[131]*tmpObjS[136] + tmpFx[143]*tmpObjS[149] + tmpFx[155]*tmpObjS[162];
tmpQ2[150] = + tmpFx[11]*tmpObjS[7] + tmpFx[23]*tmpObjS[20] + tmpFx[35]*tmpObjS[33] + tmpFx[47]*tmpObjS[46] + tmpFx[59]*tmpObjS[59] + tmpFx[71]*tmpObjS[72] + tmpFx[83]*tmpObjS[85] + tmpFx[95]*tmpObjS[98] + tmpFx[107]*tmpObjS[111] + tmpFx[119]*tmpObjS[124] + tmpFx[131]*tmpObjS[137] + tmpFx[143]*tmpObjS[150] + tmpFx[155]*tmpObjS[163];
tmpQ2[151] = + tmpFx[11]*tmpObjS[8] + tmpFx[23]*tmpObjS[21] + tmpFx[35]*tmpObjS[34] + tmpFx[47]*tmpObjS[47] + tmpFx[59]*tmpObjS[60] + tmpFx[71]*tmpObjS[73] + tmpFx[83]*tmpObjS[86] + tmpFx[95]*tmpObjS[99] + tmpFx[107]*tmpObjS[112] + tmpFx[119]*tmpObjS[125] + tmpFx[131]*tmpObjS[138] + tmpFx[143]*tmpObjS[151] + tmpFx[155]*tmpObjS[164];
tmpQ2[152] = + tmpFx[11]*tmpObjS[9] + tmpFx[23]*tmpObjS[22] + tmpFx[35]*tmpObjS[35] + tmpFx[47]*tmpObjS[48] + tmpFx[59]*tmpObjS[61] + tmpFx[71]*tmpObjS[74] + tmpFx[83]*tmpObjS[87] + tmpFx[95]*tmpObjS[100] + tmpFx[107]*tmpObjS[113] + tmpFx[119]*tmpObjS[126] + tmpFx[131]*tmpObjS[139] + tmpFx[143]*tmpObjS[152] + tmpFx[155]*tmpObjS[165];
tmpQ2[153] = + tmpFx[11]*tmpObjS[10] + tmpFx[23]*tmpObjS[23] + tmpFx[35]*tmpObjS[36] + tmpFx[47]*tmpObjS[49] + tmpFx[59]*tmpObjS[62] + tmpFx[71]*tmpObjS[75] + tmpFx[83]*tmpObjS[88] + tmpFx[95]*tmpObjS[101] + tmpFx[107]*tmpObjS[114] + tmpFx[119]*tmpObjS[127] + tmpFx[131]*tmpObjS[140] + tmpFx[143]*tmpObjS[153] + tmpFx[155]*tmpObjS[166];
tmpQ2[154] = + tmpFx[11]*tmpObjS[11] + tmpFx[23]*tmpObjS[24] + tmpFx[35]*tmpObjS[37] + tmpFx[47]*tmpObjS[50] + tmpFx[59]*tmpObjS[63] + tmpFx[71]*tmpObjS[76] + tmpFx[83]*tmpObjS[89] + tmpFx[95]*tmpObjS[102] + tmpFx[107]*tmpObjS[115] + tmpFx[119]*tmpObjS[128] + tmpFx[131]*tmpObjS[141] + tmpFx[143]*tmpObjS[154] + tmpFx[155]*tmpObjS[167];
tmpQ2[155] = + tmpFx[11]*tmpObjS[12] + tmpFx[23]*tmpObjS[25] + tmpFx[35]*tmpObjS[38] + tmpFx[47]*tmpObjS[51] + tmpFx[59]*tmpObjS[64] + tmpFx[71]*tmpObjS[77] + tmpFx[83]*tmpObjS[90] + tmpFx[95]*tmpObjS[103] + tmpFx[107]*tmpObjS[116] + tmpFx[119]*tmpObjS[129] + tmpFx[131]*tmpObjS[142] + tmpFx[143]*tmpObjS[155] + tmpFx[155]*tmpObjS[168];
tmpQ1[0] = + tmpQ2[0]*tmpFx[0] + tmpQ2[1]*tmpFx[12] + tmpQ2[2]*tmpFx[24] + tmpQ2[3]*tmpFx[36] + tmpQ2[4]*tmpFx[48] + tmpQ2[5]*tmpFx[60] + tmpQ2[6]*tmpFx[72] + tmpQ2[7]*tmpFx[84] + tmpQ2[8]*tmpFx[96] + tmpQ2[9]*tmpFx[108] + tmpQ2[10]*tmpFx[120] + tmpQ2[11]*tmpFx[132] + tmpQ2[12]*tmpFx[144];
tmpQ1[1] = + tmpQ2[0]*tmpFx[1] + tmpQ2[1]*tmpFx[13] + tmpQ2[2]*tmpFx[25] + tmpQ2[3]*tmpFx[37] + tmpQ2[4]*tmpFx[49] + tmpQ2[5]*tmpFx[61] + tmpQ2[6]*tmpFx[73] + tmpQ2[7]*tmpFx[85] + tmpQ2[8]*tmpFx[97] + tmpQ2[9]*tmpFx[109] + tmpQ2[10]*tmpFx[121] + tmpQ2[11]*tmpFx[133] + tmpQ2[12]*tmpFx[145];
tmpQ1[2] = + tmpQ2[0]*tmpFx[2] + tmpQ2[1]*tmpFx[14] + tmpQ2[2]*tmpFx[26] + tmpQ2[3]*tmpFx[38] + tmpQ2[4]*tmpFx[50] + tmpQ2[5]*tmpFx[62] + tmpQ2[6]*tmpFx[74] + tmpQ2[7]*tmpFx[86] + tmpQ2[8]*tmpFx[98] + tmpQ2[9]*tmpFx[110] + tmpQ2[10]*tmpFx[122] + tmpQ2[11]*tmpFx[134] + tmpQ2[12]*tmpFx[146];
tmpQ1[3] = + tmpQ2[0]*tmpFx[3] + tmpQ2[1]*tmpFx[15] + tmpQ2[2]*tmpFx[27] + tmpQ2[3]*tmpFx[39] + tmpQ2[4]*tmpFx[51] + tmpQ2[5]*tmpFx[63] + tmpQ2[6]*tmpFx[75] + tmpQ2[7]*tmpFx[87] + tmpQ2[8]*tmpFx[99] + tmpQ2[9]*tmpFx[111] + tmpQ2[10]*tmpFx[123] + tmpQ2[11]*tmpFx[135] + tmpQ2[12]*tmpFx[147];
tmpQ1[4] = + tmpQ2[0]*tmpFx[4] + tmpQ2[1]*tmpFx[16] + tmpQ2[2]*tmpFx[28] + tmpQ2[3]*tmpFx[40] + tmpQ2[4]*tmpFx[52] + tmpQ2[5]*tmpFx[64] + tmpQ2[6]*tmpFx[76] + tmpQ2[7]*tmpFx[88] + tmpQ2[8]*tmpFx[100] + tmpQ2[9]*tmpFx[112] + tmpQ2[10]*tmpFx[124] + tmpQ2[11]*tmpFx[136] + tmpQ2[12]*tmpFx[148];
tmpQ1[5] = + tmpQ2[0]*tmpFx[5] + tmpQ2[1]*tmpFx[17] + tmpQ2[2]*tmpFx[29] + tmpQ2[3]*tmpFx[41] + tmpQ2[4]*tmpFx[53] + tmpQ2[5]*tmpFx[65] + tmpQ2[6]*tmpFx[77] + tmpQ2[7]*tmpFx[89] + tmpQ2[8]*tmpFx[101] + tmpQ2[9]*tmpFx[113] + tmpQ2[10]*tmpFx[125] + tmpQ2[11]*tmpFx[137] + tmpQ2[12]*tmpFx[149];
tmpQ1[6] = + tmpQ2[0]*tmpFx[6] + tmpQ2[1]*tmpFx[18] + tmpQ2[2]*tmpFx[30] + tmpQ2[3]*tmpFx[42] + tmpQ2[4]*tmpFx[54] + tmpQ2[5]*tmpFx[66] + tmpQ2[6]*tmpFx[78] + tmpQ2[7]*tmpFx[90] + tmpQ2[8]*tmpFx[102] + tmpQ2[9]*tmpFx[114] + tmpQ2[10]*tmpFx[126] + tmpQ2[11]*tmpFx[138] + tmpQ2[12]*tmpFx[150];
tmpQ1[7] = + tmpQ2[0]*tmpFx[7] + tmpQ2[1]*tmpFx[19] + tmpQ2[2]*tmpFx[31] + tmpQ2[3]*tmpFx[43] + tmpQ2[4]*tmpFx[55] + tmpQ2[5]*tmpFx[67] + tmpQ2[6]*tmpFx[79] + tmpQ2[7]*tmpFx[91] + tmpQ2[8]*tmpFx[103] + tmpQ2[9]*tmpFx[115] + tmpQ2[10]*tmpFx[127] + tmpQ2[11]*tmpFx[139] + tmpQ2[12]*tmpFx[151];
tmpQ1[8] = + tmpQ2[0]*tmpFx[8] + tmpQ2[1]*tmpFx[20] + tmpQ2[2]*tmpFx[32] + tmpQ2[3]*tmpFx[44] + tmpQ2[4]*tmpFx[56] + tmpQ2[5]*tmpFx[68] + tmpQ2[6]*tmpFx[80] + tmpQ2[7]*tmpFx[92] + tmpQ2[8]*tmpFx[104] + tmpQ2[9]*tmpFx[116] + tmpQ2[10]*tmpFx[128] + tmpQ2[11]*tmpFx[140] + tmpQ2[12]*tmpFx[152];
tmpQ1[9] = + tmpQ2[0]*tmpFx[9] + tmpQ2[1]*tmpFx[21] + tmpQ2[2]*tmpFx[33] + tmpQ2[3]*tmpFx[45] + tmpQ2[4]*tmpFx[57] + tmpQ2[5]*tmpFx[69] + tmpQ2[6]*tmpFx[81] + tmpQ2[7]*tmpFx[93] + tmpQ2[8]*tmpFx[105] + tmpQ2[9]*tmpFx[117] + tmpQ2[10]*tmpFx[129] + tmpQ2[11]*tmpFx[141] + tmpQ2[12]*tmpFx[153];
tmpQ1[10] = + tmpQ2[0]*tmpFx[10] + tmpQ2[1]*tmpFx[22] + tmpQ2[2]*tmpFx[34] + tmpQ2[3]*tmpFx[46] + tmpQ2[4]*tmpFx[58] + tmpQ2[5]*tmpFx[70] + tmpQ2[6]*tmpFx[82] + tmpQ2[7]*tmpFx[94] + tmpQ2[8]*tmpFx[106] + tmpQ2[9]*tmpFx[118] + tmpQ2[10]*tmpFx[130] + tmpQ2[11]*tmpFx[142] + tmpQ2[12]*tmpFx[154];
tmpQ1[11] = + tmpQ2[0]*tmpFx[11] + tmpQ2[1]*tmpFx[23] + tmpQ2[2]*tmpFx[35] + tmpQ2[3]*tmpFx[47] + tmpQ2[4]*tmpFx[59] + tmpQ2[5]*tmpFx[71] + tmpQ2[6]*tmpFx[83] + tmpQ2[7]*tmpFx[95] + tmpQ2[8]*tmpFx[107] + tmpQ2[9]*tmpFx[119] + tmpQ2[10]*tmpFx[131] + tmpQ2[11]*tmpFx[143] + tmpQ2[12]*tmpFx[155];
tmpQ1[12] = + tmpQ2[13]*tmpFx[0] + tmpQ2[14]*tmpFx[12] + tmpQ2[15]*tmpFx[24] + tmpQ2[16]*tmpFx[36] + tmpQ2[17]*tmpFx[48] + tmpQ2[18]*tmpFx[60] + tmpQ2[19]*tmpFx[72] + tmpQ2[20]*tmpFx[84] + tmpQ2[21]*tmpFx[96] + tmpQ2[22]*tmpFx[108] + tmpQ2[23]*tmpFx[120] + tmpQ2[24]*tmpFx[132] + tmpQ2[25]*tmpFx[144];
tmpQ1[13] = + tmpQ2[13]*tmpFx[1] + tmpQ2[14]*tmpFx[13] + tmpQ2[15]*tmpFx[25] + tmpQ2[16]*tmpFx[37] + tmpQ2[17]*tmpFx[49] + tmpQ2[18]*tmpFx[61] + tmpQ2[19]*tmpFx[73] + tmpQ2[20]*tmpFx[85] + tmpQ2[21]*tmpFx[97] + tmpQ2[22]*tmpFx[109] + tmpQ2[23]*tmpFx[121] + tmpQ2[24]*tmpFx[133] + tmpQ2[25]*tmpFx[145];
tmpQ1[14] = + tmpQ2[13]*tmpFx[2] + tmpQ2[14]*tmpFx[14] + tmpQ2[15]*tmpFx[26] + tmpQ2[16]*tmpFx[38] + tmpQ2[17]*tmpFx[50] + tmpQ2[18]*tmpFx[62] + tmpQ2[19]*tmpFx[74] + tmpQ2[20]*tmpFx[86] + tmpQ2[21]*tmpFx[98] + tmpQ2[22]*tmpFx[110] + tmpQ2[23]*tmpFx[122] + tmpQ2[24]*tmpFx[134] + tmpQ2[25]*tmpFx[146];
tmpQ1[15] = + tmpQ2[13]*tmpFx[3] + tmpQ2[14]*tmpFx[15] + tmpQ2[15]*tmpFx[27] + tmpQ2[16]*tmpFx[39] + tmpQ2[17]*tmpFx[51] + tmpQ2[18]*tmpFx[63] + tmpQ2[19]*tmpFx[75] + tmpQ2[20]*tmpFx[87] + tmpQ2[21]*tmpFx[99] + tmpQ2[22]*tmpFx[111] + tmpQ2[23]*tmpFx[123] + tmpQ2[24]*tmpFx[135] + tmpQ2[25]*tmpFx[147];
tmpQ1[16] = + tmpQ2[13]*tmpFx[4] + tmpQ2[14]*tmpFx[16] + tmpQ2[15]*tmpFx[28] + tmpQ2[16]*tmpFx[40] + tmpQ2[17]*tmpFx[52] + tmpQ2[18]*tmpFx[64] + tmpQ2[19]*tmpFx[76] + tmpQ2[20]*tmpFx[88] + tmpQ2[21]*tmpFx[100] + tmpQ2[22]*tmpFx[112] + tmpQ2[23]*tmpFx[124] + tmpQ2[24]*tmpFx[136] + tmpQ2[25]*tmpFx[148];
tmpQ1[17] = + tmpQ2[13]*tmpFx[5] + tmpQ2[14]*tmpFx[17] + tmpQ2[15]*tmpFx[29] + tmpQ2[16]*tmpFx[41] + tmpQ2[17]*tmpFx[53] + tmpQ2[18]*tmpFx[65] + tmpQ2[19]*tmpFx[77] + tmpQ2[20]*tmpFx[89] + tmpQ2[21]*tmpFx[101] + tmpQ2[22]*tmpFx[113] + tmpQ2[23]*tmpFx[125] + tmpQ2[24]*tmpFx[137] + tmpQ2[25]*tmpFx[149];
tmpQ1[18] = + tmpQ2[13]*tmpFx[6] + tmpQ2[14]*tmpFx[18] + tmpQ2[15]*tmpFx[30] + tmpQ2[16]*tmpFx[42] + tmpQ2[17]*tmpFx[54] + tmpQ2[18]*tmpFx[66] + tmpQ2[19]*tmpFx[78] + tmpQ2[20]*tmpFx[90] + tmpQ2[21]*tmpFx[102] + tmpQ2[22]*tmpFx[114] + tmpQ2[23]*tmpFx[126] + tmpQ2[24]*tmpFx[138] + tmpQ2[25]*tmpFx[150];
tmpQ1[19] = + tmpQ2[13]*tmpFx[7] + tmpQ2[14]*tmpFx[19] + tmpQ2[15]*tmpFx[31] + tmpQ2[16]*tmpFx[43] + tmpQ2[17]*tmpFx[55] + tmpQ2[18]*tmpFx[67] + tmpQ2[19]*tmpFx[79] + tmpQ2[20]*tmpFx[91] + tmpQ2[21]*tmpFx[103] + tmpQ2[22]*tmpFx[115] + tmpQ2[23]*tmpFx[127] + tmpQ2[24]*tmpFx[139] + tmpQ2[25]*tmpFx[151];
tmpQ1[20] = + tmpQ2[13]*tmpFx[8] + tmpQ2[14]*tmpFx[20] + tmpQ2[15]*tmpFx[32] + tmpQ2[16]*tmpFx[44] + tmpQ2[17]*tmpFx[56] + tmpQ2[18]*tmpFx[68] + tmpQ2[19]*tmpFx[80] + tmpQ2[20]*tmpFx[92] + tmpQ2[21]*tmpFx[104] + tmpQ2[22]*tmpFx[116] + tmpQ2[23]*tmpFx[128] + tmpQ2[24]*tmpFx[140] + tmpQ2[25]*tmpFx[152];
tmpQ1[21] = + tmpQ2[13]*tmpFx[9] + tmpQ2[14]*tmpFx[21] + tmpQ2[15]*tmpFx[33] + tmpQ2[16]*tmpFx[45] + tmpQ2[17]*tmpFx[57] + tmpQ2[18]*tmpFx[69] + tmpQ2[19]*tmpFx[81] + tmpQ2[20]*tmpFx[93] + tmpQ2[21]*tmpFx[105] + tmpQ2[22]*tmpFx[117] + tmpQ2[23]*tmpFx[129] + tmpQ2[24]*tmpFx[141] + tmpQ2[25]*tmpFx[153];
tmpQ1[22] = + tmpQ2[13]*tmpFx[10] + tmpQ2[14]*tmpFx[22] + tmpQ2[15]*tmpFx[34] + tmpQ2[16]*tmpFx[46] + tmpQ2[17]*tmpFx[58] + tmpQ2[18]*tmpFx[70] + tmpQ2[19]*tmpFx[82] + tmpQ2[20]*tmpFx[94] + tmpQ2[21]*tmpFx[106] + tmpQ2[22]*tmpFx[118] + tmpQ2[23]*tmpFx[130] + tmpQ2[24]*tmpFx[142] + tmpQ2[25]*tmpFx[154];
tmpQ1[23] = + tmpQ2[13]*tmpFx[11] + tmpQ2[14]*tmpFx[23] + tmpQ2[15]*tmpFx[35] + tmpQ2[16]*tmpFx[47] + tmpQ2[17]*tmpFx[59] + tmpQ2[18]*tmpFx[71] + tmpQ2[19]*tmpFx[83] + tmpQ2[20]*tmpFx[95] + tmpQ2[21]*tmpFx[107] + tmpQ2[22]*tmpFx[119] + tmpQ2[23]*tmpFx[131] + tmpQ2[24]*tmpFx[143] + tmpQ2[25]*tmpFx[155];
tmpQ1[24] = + tmpQ2[26]*tmpFx[0] + tmpQ2[27]*tmpFx[12] + tmpQ2[28]*tmpFx[24] + tmpQ2[29]*tmpFx[36] + tmpQ2[30]*tmpFx[48] + tmpQ2[31]*tmpFx[60] + tmpQ2[32]*tmpFx[72] + tmpQ2[33]*tmpFx[84] + tmpQ2[34]*tmpFx[96] + tmpQ2[35]*tmpFx[108] + tmpQ2[36]*tmpFx[120] + tmpQ2[37]*tmpFx[132] + tmpQ2[38]*tmpFx[144];
tmpQ1[25] = + tmpQ2[26]*tmpFx[1] + tmpQ2[27]*tmpFx[13] + tmpQ2[28]*tmpFx[25] + tmpQ2[29]*tmpFx[37] + tmpQ2[30]*tmpFx[49] + tmpQ2[31]*tmpFx[61] + tmpQ2[32]*tmpFx[73] + tmpQ2[33]*tmpFx[85] + tmpQ2[34]*tmpFx[97] + tmpQ2[35]*tmpFx[109] + tmpQ2[36]*tmpFx[121] + tmpQ2[37]*tmpFx[133] + tmpQ2[38]*tmpFx[145];
tmpQ1[26] = + tmpQ2[26]*tmpFx[2] + tmpQ2[27]*tmpFx[14] + tmpQ2[28]*tmpFx[26] + tmpQ2[29]*tmpFx[38] + tmpQ2[30]*tmpFx[50] + tmpQ2[31]*tmpFx[62] + tmpQ2[32]*tmpFx[74] + tmpQ2[33]*tmpFx[86] + tmpQ2[34]*tmpFx[98] + tmpQ2[35]*tmpFx[110] + tmpQ2[36]*tmpFx[122] + tmpQ2[37]*tmpFx[134] + tmpQ2[38]*tmpFx[146];
tmpQ1[27] = + tmpQ2[26]*tmpFx[3] + tmpQ2[27]*tmpFx[15] + tmpQ2[28]*tmpFx[27] + tmpQ2[29]*tmpFx[39] + tmpQ2[30]*tmpFx[51] + tmpQ2[31]*tmpFx[63] + tmpQ2[32]*tmpFx[75] + tmpQ2[33]*tmpFx[87] + tmpQ2[34]*tmpFx[99] + tmpQ2[35]*tmpFx[111] + tmpQ2[36]*tmpFx[123] + tmpQ2[37]*tmpFx[135] + tmpQ2[38]*tmpFx[147];
tmpQ1[28] = + tmpQ2[26]*tmpFx[4] + tmpQ2[27]*tmpFx[16] + tmpQ2[28]*tmpFx[28] + tmpQ2[29]*tmpFx[40] + tmpQ2[30]*tmpFx[52] + tmpQ2[31]*tmpFx[64] + tmpQ2[32]*tmpFx[76] + tmpQ2[33]*tmpFx[88] + tmpQ2[34]*tmpFx[100] + tmpQ2[35]*tmpFx[112] + tmpQ2[36]*tmpFx[124] + tmpQ2[37]*tmpFx[136] + tmpQ2[38]*tmpFx[148];
tmpQ1[29] = + tmpQ2[26]*tmpFx[5] + tmpQ2[27]*tmpFx[17] + tmpQ2[28]*tmpFx[29] + tmpQ2[29]*tmpFx[41] + tmpQ2[30]*tmpFx[53] + tmpQ2[31]*tmpFx[65] + tmpQ2[32]*tmpFx[77] + tmpQ2[33]*tmpFx[89] + tmpQ2[34]*tmpFx[101] + tmpQ2[35]*tmpFx[113] + tmpQ2[36]*tmpFx[125] + tmpQ2[37]*tmpFx[137] + tmpQ2[38]*tmpFx[149];
tmpQ1[30] = + tmpQ2[26]*tmpFx[6] + tmpQ2[27]*tmpFx[18] + tmpQ2[28]*tmpFx[30] + tmpQ2[29]*tmpFx[42] + tmpQ2[30]*tmpFx[54] + tmpQ2[31]*tmpFx[66] + tmpQ2[32]*tmpFx[78] + tmpQ2[33]*tmpFx[90] + tmpQ2[34]*tmpFx[102] + tmpQ2[35]*tmpFx[114] + tmpQ2[36]*tmpFx[126] + tmpQ2[37]*tmpFx[138] + tmpQ2[38]*tmpFx[150];
tmpQ1[31] = + tmpQ2[26]*tmpFx[7] + tmpQ2[27]*tmpFx[19] + tmpQ2[28]*tmpFx[31] + tmpQ2[29]*tmpFx[43] + tmpQ2[30]*tmpFx[55] + tmpQ2[31]*tmpFx[67] + tmpQ2[32]*tmpFx[79] + tmpQ2[33]*tmpFx[91] + tmpQ2[34]*tmpFx[103] + tmpQ2[35]*tmpFx[115] + tmpQ2[36]*tmpFx[127] + tmpQ2[37]*tmpFx[139] + tmpQ2[38]*tmpFx[151];
tmpQ1[32] = + tmpQ2[26]*tmpFx[8] + tmpQ2[27]*tmpFx[20] + tmpQ2[28]*tmpFx[32] + tmpQ2[29]*tmpFx[44] + tmpQ2[30]*tmpFx[56] + tmpQ2[31]*tmpFx[68] + tmpQ2[32]*tmpFx[80] + tmpQ2[33]*tmpFx[92] + tmpQ2[34]*tmpFx[104] + tmpQ2[35]*tmpFx[116] + tmpQ2[36]*tmpFx[128] + tmpQ2[37]*tmpFx[140] + tmpQ2[38]*tmpFx[152];
tmpQ1[33] = + tmpQ2[26]*tmpFx[9] + tmpQ2[27]*tmpFx[21] + tmpQ2[28]*tmpFx[33] + tmpQ2[29]*tmpFx[45] + tmpQ2[30]*tmpFx[57] + tmpQ2[31]*tmpFx[69] + tmpQ2[32]*tmpFx[81] + tmpQ2[33]*tmpFx[93] + tmpQ2[34]*tmpFx[105] + tmpQ2[35]*tmpFx[117] + tmpQ2[36]*tmpFx[129] + tmpQ2[37]*tmpFx[141] + tmpQ2[38]*tmpFx[153];
tmpQ1[34] = + tmpQ2[26]*tmpFx[10] + tmpQ2[27]*tmpFx[22] + tmpQ2[28]*tmpFx[34] + tmpQ2[29]*tmpFx[46] + tmpQ2[30]*tmpFx[58] + tmpQ2[31]*tmpFx[70] + tmpQ2[32]*tmpFx[82] + tmpQ2[33]*tmpFx[94] + tmpQ2[34]*tmpFx[106] + tmpQ2[35]*tmpFx[118] + tmpQ2[36]*tmpFx[130] + tmpQ2[37]*tmpFx[142] + tmpQ2[38]*tmpFx[154];
tmpQ1[35] = + tmpQ2[26]*tmpFx[11] + tmpQ2[27]*tmpFx[23] + tmpQ2[28]*tmpFx[35] + tmpQ2[29]*tmpFx[47] + tmpQ2[30]*tmpFx[59] + tmpQ2[31]*tmpFx[71] + tmpQ2[32]*tmpFx[83] + tmpQ2[33]*tmpFx[95] + tmpQ2[34]*tmpFx[107] + tmpQ2[35]*tmpFx[119] + tmpQ2[36]*tmpFx[131] + tmpQ2[37]*tmpFx[143] + tmpQ2[38]*tmpFx[155];
tmpQ1[36] = + tmpQ2[39]*tmpFx[0] + tmpQ2[40]*tmpFx[12] + tmpQ2[41]*tmpFx[24] + tmpQ2[42]*tmpFx[36] + tmpQ2[43]*tmpFx[48] + tmpQ2[44]*tmpFx[60] + tmpQ2[45]*tmpFx[72] + tmpQ2[46]*tmpFx[84] + tmpQ2[47]*tmpFx[96] + tmpQ2[48]*tmpFx[108] + tmpQ2[49]*tmpFx[120] + tmpQ2[50]*tmpFx[132] + tmpQ2[51]*tmpFx[144];
tmpQ1[37] = + tmpQ2[39]*tmpFx[1] + tmpQ2[40]*tmpFx[13] + tmpQ2[41]*tmpFx[25] + tmpQ2[42]*tmpFx[37] + tmpQ2[43]*tmpFx[49] + tmpQ2[44]*tmpFx[61] + tmpQ2[45]*tmpFx[73] + tmpQ2[46]*tmpFx[85] + tmpQ2[47]*tmpFx[97] + tmpQ2[48]*tmpFx[109] + tmpQ2[49]*tmpFx[121] + tmpQ2[50]*tmpFx[133] + tmpQ2[51]*tmpFx[145];
tmpQ1[38] = + tmpQ2[39]*tmpFx[2] + tmpQ2[40]*tmpFx[14] + tmpQ2[41]*tmpFx[26] + tmpQ2[42]*tmpFx[38] + tmpQ2[43]*tmpFx[50] + tmpQ2[44]*tmpFx[62] + tmpQ2[45]*tmpFx[74] + tmpQ2[46]*tmpFx[86] + tmpQ2[47]*tmpFx[98] + tmpQ2[48]*tmpFx[110] + tmpQ2[49]*tmpFx[122] + tmpQ2[50]*tmpFx[134] + tmpQ2[51]*tmpFx[146];
tmpQ1[39] = + tmpQ2[39]*tmpFx[3] + tmpQ2[40]*tmpFx[15] + tmpQ2[41]*tmpFx[27] + tmpQ2[42]*tmpFx[39] + tmpQ2[43]*tmpFx[51] + tmpQ2[44]*tmpFx[63] + tmpQ2[45]*tmpFx[75] + tmpQ2[46]*tmpFx[87] + tmpQ2[47]*tmpFx[99] + tmpQ2[48]*tmpFx[111] + tmpQ2[49]*tmpFx[123] + tmpQ2[50]*tmpFx[135] + tmpQ2[51]*tmpFx[147];
tmpQ1[40] = + tmpQ2[39]*tmpFx[4] + tmpQ2[40]*tmpFx[16] + tmpQ2[41]*tmpFx[28] + tmpQ2[42]*tmpFx[40] + tmpQ2[43]*tmpFx[52] + tmpQ2[44]*tmpFx[64] + tmpQ2[45]*tmpFx[76] + tmpQ2[46]*tmpFx[88] + tmpQ2[47]*tmpFx[100] + tmpQ2[48]*tmpFx[112] + tmpQ2[49]*tmpFx[124] + tmpQ2[50]*tmpFx[136] + tmpQ2[51]*tmpFx[148];
tmpQ1[41] = + tmpQ2[39]*tmpFx[5] + tmpQ2[40]*tmpFx[17] + tmpQ2[41]*tmpFx[29] + tmpQ2[42]*tmpFx[41] + tmpQ2[43]*tmpFx[53] + tmpQ2[44]*tmpFx[65] + tmpQ2[45]*tmpFx[77] + tmpQ2[46]*tmpFx[89] + tmpQ2[47]*tmpFx[101] + tmpQ2[48]*tmpFx[113] + tmpQ2[49]*tmpFx[125] + tmpQ2[50]*tmpFx[137] + tmpQ2[51]*tmpFx[149];
tmpQ1[42] = + tmpQ2[39]*tmpFx[6] + tmpQ2[40]*tmpFx[18] + tmpQ2[41]*tmpFx[30] + tmpQ2[42]*tmpFx[42] + tmpQ2[43]*tmpFx[54] + tmpQ2[44]*tmpFx[66] + tmpQ2[45]*tmpFx[78] + tmpQ2[46]*tmpFx[90] + tmpQ2[47]*tmpFx[102] + tmpQ2[48]*tmpFx[114] + tmpQ2[49]*tmpFx[126] + tmpQ2[50]*tmpFx[138] + tmpQ2[51]*tmpFx[150];
tmpQ1[43] = + tmpQ2[39]*tmpFx[7] + tmpQ2[40]*tmpFx[19] + tmpQ2[41]*tmpFx[31] + tmpQ2[42]*tmpFx[43] + tmpQ2[43]*tmpFx[55] + tmpQ2[44]*tmpFx[67] + tmpQ2[45]*tmpFx[79] + tmpQ2[46]*tmpFx[91] + tmpQ2[47]*tmpFx[103] + tmpQ2[48]*tmpFx[115] + tmpQ2[49]*tmpFx[127] + tmpQ2[50]*tmpFx[139] + tmpQ2[51]*tmpFx[151];
tmpQ1[44] = + tmpQ2[39]*tmpFx[8] + tmpQ2[40]*tmpFx[20] + tmpQ2[41]*tmpFx[32] + tmpQ2[42]*tmpFx[44] + tmpQ2[43]*tmpFx[56] + tmpQ2[44]*tmpFx[68] + tmpQ2[45]*tmpFx[80] + tmpQ2[46]*tmpFx[92] + tmpQ2[47]*tmpFx[104] + tmpQ2[48]*tmpFx[116] + tmpQ2[49]*tmpFx[128] + tmpQ2[50]*tmpFx[140] + tmpQ2[51]*tmpFx[152];
tmpQ1[45] = + tmpQ2[39]*tmpFx[9] + tmpQ2[40]*tmpFx[21] + tmpQ2[41]*tmpFx[33] + tmpQ2[42]*tmpFx[45] + tmpQ2[43]*tmpFx[57] + tmpQ2[44]*tmpFx[69] + tmpQ2[45]*tmpFx[81] + tmpQ2[46]*tmpFx[93] + tmpQ2[47]*tmpFx[105] + tmpQ2[48]*tmpFx[117] + tmpQ2[49]*tmpFx[129] + tmpQ2[50]*tmpFx[141] + tmpQ2[51]*tmpFx[153];
tmpQ1[46] = + tmpQ2[39]*tmpFx[10] + tmpQ2[40]*tmpFx[22] + tmpQ2[41]*tmpFx[34] + tmpQ2[42]*tmpFx[46] + tmpQ2[43]*tmpFx[58] + tmpQ2[44]*tmpFx[70] + tmpQ2[45]*tmpFx[82] + tmpQ2[46]*tmpFx[94] + tmpQ2[47]*tmpFx[106] + tmpQ2[48]*tmpFx[118] + tmpQ2[49]*tmpFx[130] + tmpQ2[50]*tmpFx[142] + tmpQ2[51]*tmpFx[154];
tmpQ1[47] = + tmpQ2[39]*tmpFx[11] + tmpQ2[40]*tmpFx[23] + tmpQ2[41]*tmpFx[35] + tmpQ2[42]*tmpFx[47] + tmpQ2[43]*tmpFx[59] + tmpQ2[44]*tmpFx[71] + tmpQ2[45]*tmpFx[83] + tmpQ2[46]*tmpFx[95] + tmpQ2[47]*tmpFx[107] + tmpQ2[48]*tmpFx[119] + tmpQ2[49]*tmpFx[131] + tmpQ2[50]*tmpFx[143] + tmpQ2[51]*tmpFx[155];
tmpQ1[48] = + tmpQ2[52]*tmpFx[0] + tmpQ2[53]*tmpFx[12] + tmpQ2[54]*tmpFx[24] + tmpQ2[55]*tmpFx[36] + tmpQ2[56]*tmpFx[48] + tmpQ2[57]*tmpFx[60] + tmpQ2[58]*tmpFx[72] + tmpQ2[59]*tmpFx[84] + tmpQ2[60]*tmpFx[96] + tmpQ2[61]*tmpFx[108] + tmpQ2[62]*tmpFx[120] + tmpQ2[63]*tmpFx[132] + tmpQ2[64]*tmpFx[144];
tmpQ1[49] = + tmpQ2[52]*tmpFx[1] + tmpQ2[53]*tmpFx[13] + tmpQ2[54]*tmpFx[25] + tmpQ2[55]*tmpFx[37] + tmpQ2[56]*tmpFx[49] + tmpQ2[57]*tmpFx[61] + tmpQ2[58]*tmpFx[73] + tmpQ2[59]*tmpFx[85] + tmpQ2[60]*tmpFx[97] + tmpQ2[61]*tmpFx[109] + tmpQ2[62]*tmpFx[121] + tmpQ2[63]*tmpFx[133] + tmpQ2[64]*tmpFx[145];
tmpQ1[50] = + tmpQ2[52]*tmpFx[2] + tmpQ2[53]*tmpFx[14] + tmpQ2[54]*tmpFx[26] + tmpQ2[55]*tmpFx[38] + tmpQ2[56]*tmpFx[50] + tmpQ2[57]*tmpFx[62] + tmpQ2[58]*tmpFx[74] + tmpQ2[59]*tmpFx[86] + tmpQ2[60]*tmpFx[98] + tmpQ2[61]*tmpFx[110] + tmpQ2[62]*tmpFx[122] + tmpQ2[63]*tmpFx[134] + tmpQ2[64]*tmpFx[146];
tmpQ1[51] = + tmpQ2[52]*tmpFx[3] + tmpQ2[53]*tmpFx[15] + tmpQ2[54]*tmpFx[27] + tmpQ2[55]*tmpFx[39] + tmpQ2[56]*tmpFx[51] + tmpQ2[57]*tmpFx[63] + tmpQ2[58]*tmpFx[75] + tmpQ2[59]*tmpFx[87] + tmpQ2[60]*tmpFx[99] + tmpQ2[61]*tmpFx[111] + tmpQ2[62]*tmpFx[123] + tmpQ2[63]*tmpFx[135] + tmpQ2[64]*tmpFx[147];
tmpQ1[52] = + tmpQ2[52]*tmpFx[4] + tmpQ2[53]*tmpFx[16] + tmpQ2[54]*tmpFx[28] + tmpQ2[55]*tmpFx[40] + tmpQ2[56]*tmpFx[52] + tmpQ2[57]*tmpFx[64] + tmpQ2[58]*tmpFx[76] + tmpQ2[59]*tmpFx[88] + tmpQ2[60]*tmpFx[100] + tmpQ2[61]*tmpFx[112] + tmpQ2[62]*tmpFx[124] + tmpQ2[63]*tmpFx[136] + tmpQ2[64]*tmpFx[148];
tmpQ1[53] = + tmpQ2[52]*tmpFx[5] + tmpQ2[53]*tmpFx[17] + tmpQ2[54]*tmpFx[29] + tmpQ2[55]*tmpFx[41] + tmpQ2[56]*tmpFx[53] + tmpQ2[57]*tmpFx[65] + tmpQ2[58]*tmpFx[77] + tmpQ2[59]*tmpFx[89] + tmpQ2[60]*tmpFx[101] + tmpQ2[61]*tmpFx[113] + tmpQ2[62]*tmpFx[125] + tmpQ2[63]*tmpFx[137] + tmpQ2[64]*tmpFx[149];
tmpQ1[54] = + tmpQ2[52]*tmpFx[6] + tmpQ2[53]*tmpFx[18] + tmpQ2[54]*tmpFx[30] + tmpQ2[55]*tmpFx[42] + tmpQ2[56]*tmpFx[54] + tmpQ2[57]*tmpFx[66] + tmpQ2[58]*tmpFx[78] + tmpQ2[59]*tmpFx[90] + tmpQ2[60]*tmpFx[102] + tmpQ2[61]*tmpFx[114] + tmpQ2[62]*tmpFx[126] + tmpQ2[63]*tmpFx[138] + tmpQ2[64]*tmpFx[150];
tmpQ1[55] = + tmpQ2[52]*tmpFx[7] + tmpQ2[53]*tmpFx[19] + tmpQ2[54]*tmpFx[31] + tmpQ2[55]*tmpFx[43] + tmpQ2[56]*tmpFx[55] + tmpQ2[57]*tmpFx[67] + tmpQ2[58]*tmpFx[79] + tmpQ2[59]*tmpFx[91] + tmpQ2[60]*tmpFx[103] + tmpQ2[61]*tmpFx[115] + tmpQ2[62]*tmpFx[127] + tmpQ2[63]*tmpFx[139] + tmpQ2[64]*tmpFx[151];
tmpQ1[56] = + tmpQ2[52]*tmpFx[8] + tmpQ2[53]*tmpFx[20] + tmpQ2[54]*tmpFx[32] + tmpQ2[55]*tmpFx[44] + tmpQ2[56]*tmpFx[56] + tmpQ2[57]*tmpFx[68] + tmpQ2[58]*tmpFx[80] + tmpQ2[59]*tmpFx[92] + tmpQ2[60]*tmpFx[104] + tmpQ2[61]*tmpFx[116] + tmpQ2[62]*tmpFx[128] + tmpQ2[63]*tmpFx[140] + tmpQ2[64]*tmpFx[152];
tmpQ1[57] = + tmpQ2[52]*tmpFx[9] + tmpQ2[53]*tmpFx[21] + tmpQ2[54]*tmpFx[33] + tmpQ2[55]*tmpFx[45] + tmpQ2[56]*tmpFx[57] + tmpQ2[57]*tmpFx[69] + tmpQ2[58]*tmpFx[81] + tmpQ2[59]*tmpFx[93] + tmpQ2[60]*tmpFx[105] + tmpQ2[61]*tmpFx[117] + tmpQ2[62]*tmpFx[129] + tmpQ2[63]*tmpFx[141] + tmpQ2[64]*tmpFx[153];
tmpQ1[58] = + tmpQ2[52]*tmpFx[10] + tmpQ2[53]*tmpFx[22] + tmpQ2[54]*tmpFx[34] + tmpQ2[55]*tmpFx[46] + tmpQ2[56]*tmpFx[58] + tmpQ2[57]*tmpFx[70] + tmpQ2[58]*tmpFx[82] + tmpQ2[59]*tmpFx[94] + tmpQ2[60]*tmpFx[106] + tmpQ2[61]*tmpFx[118] + tmpQ2[62]*tmpFx[130] + tmpQ2[63]*tmpFx[142] + tmpQ2[64]*tmpFx[154];
tmpQ1[59] = + tmpQ2[52]*tmpFx[11] + tmpQ2[53]*tmpFx[23] + tmpQ2[54]*tmpFx[35] + tmpQ2[55]*tmpFx[47] + tmpQ2[56]*tmpFx[59] + tmpQ2[57]*tmpFx[71] + tmpQ2[58]*tmpFx[83] + tmpQ2[59]*tmpFx[95] + tmpQ2[60]*tmpFx[107] + tmpQ2[61]*tmpFx[119] + tmpQ2[62]*tmpFx[131] + tmpQ2[63]*tmpFx[143] + tmpQ2[64]*tmpFx[155];
tmpQ1[60] = + tmpQ2[65]*tmpFx[0] + tmpQ2[66]*tmpFx[12] + tmpQ2[67]*tmpFx[24] + tmpQ2[68]*tmpFx[36] + tmpQ2[69]*tmpFx[48] + tmpQ2[70]*tmpFx[60] + tmpQ2[71]*tmpFx[72] + tmpQ2[72]*tmpFx[84] + tmpQ2[73]*tmpFx[96] + tmpQ2[74]*tmpFx[108] + tmpQ2[75]*tmpFx[120] + tmpQ2[76]*tmpFx[132] + tmpQ2[77]*tmpFx[144];
tmpQ1[61] = + tmpQ2[65]*tmpFx[1] + tmpQ2[66]*tmpFx[13] + tmpQ2[67]*tmpFx[25] + tmpQ2[68]*tmpFx[37] + tmpQ2[69]*tmpFx[49] + tmpQ2[70]*tmpFx[61] + tmpQ2[71]*tmpFx[73] + tmpQ2[72]*tmpFx[85] + tmpQ2[73]*tmpFx[97] + tmpQ2[74]*tmpFx[109] + tmpQ2[75]*tmpFx[121] + tmpQ2[76]*tmpFx[133] + tmpQ2[77]*tmpFx[145];
tmpQ1[62] = + tmpQ2[65]*tmpFx[2] + tmpQ2[66]*tmpFx[14] + tmpQ2[67]*tmpFx[26] + tmpQ2[68]*tmpFx[38] + tmpQ2[69]*tmpFx[50] + tmpQ2[70]*tmpFx[62] + tmpQ2[71]*tmpFx[74] + tmpQ2[72]*tmpFx[86] + tmpQ2[73]*tmpFx[98] + tmpQ2[74]*tmpFx[110] + tmpQ2[75]*tmpFx[122] + tmpQ2[76]*tmpFx[134] + tmpQ2[77]*tmpFx[146];
tmpQ1[63] = + tmpQ2[65]*tmpFx[3] + tmpQ2[66]*tmpFx[15] + tmpQ2[67]*tmpFx[27] + tmpQ2[68]*tmpFx[39] + tmpQ2[69]*tmpFx[51] + tmpQ2[70]*tmpFx[63] + tmpQ2[71]*tmpFx[75] + tmpQ2[72]*tmpFx[87] + tmpQ2[73]*tmpFx[99] + tmpQ2[74]*tmpFx[111] + tmpQ2[75]*tmpFx[123] + tmpQ2[76]*tmpFx[135] + tmpQ2[77]*tmpFx[147];
tmpQ1[64] = + tmpQ2[65]*tmpFx[4] + tmpQ2[66]*tmpFx[16] + tmpQ2[67]*tmpFx[28] + tmpQ2[68]*tmpFx[40] + tmpQ2[69]*tmpFx[52] + tmpQ2[70]*tmpFx[64] + tmpQ2[71]*tmpFx[76] + tmpQ2[72]*tmpFx[88] + tmpQ2[73]*tmpFx[100] + tmpQ2[74]*tmpFx[112] + tmpQ2[75]*tmpFx[124] + tmpQ2[76]*tmpFx[136] + tmpQ2[77]*tmpFx[148];
tmpQ1[65] = + tmpQ2[65]*tmpFx[5] + tmpQ2[66]*tmpFx[17] + tmpQ2[67]*tmpFx[29] + tmpQ2[68]*tmpFx[41] + tmpQ2[69]*tmpFx[53] + tmpQ2[70]*tmpFx[65] + tmpQ2[71]*tmpFx[77] + tmpQ2[72]*tmpFx[89] + tmpQ2[73]*tmpFx[101] + tmpQ2[74]*tmpFx[113] + tmpQ2[75]*tmpFx[125] + tmpQ2[76]*tmpFx[137] + tmpQ2[77]*tmpFx[149];
tmpQ1[66] = + tmpQ2[65]*tmpFx[6] + tmpQ2[66]*tmpFx[18] + tmpQ2[67]*tmpFx[30] + tmpQ2[68]*tmpFx[42] + tmpQ2[69]*tmpFx[54] + tmpQ2[70]*tmpFx[66] + tmpQ2[71]*tmpFx[78] + tmpQ2[72]*tmpFx[90] + tmpQ2[73]*tmpFx[102] + tmpQ2[74]*tmpFx[114] + tmpQ2[75]*tmpFx[126] + tmpQ2[76]*tmpFx[138] + tmpQ2[77]*tmpFx[150];
tmpQ1[67] = + tmpQ2[65]*tmpFx[7] + tmpQ2[66]*tmpFx[19] + tmpQ2[67]*tmpFx[31] + tmpQ2[68]*tmpFx[43] + tmpQ2[69]*tmpFx[55] + tmpQ2[70]*tmpFx[67] + tmpQ2[71]*tmpFx[79] + tmpQ2[72]*tmpFx[91] + tmpQ2[73]*tmpFx[103] + tmpQ2[74]*tmpFx[115] + tmpQ2[75]*tmpFx[127] + tmpQ2[76]*tmpFx[139] + tmpQ2[77]*tmpFx[151];
tmpQ1[68] = + tmpQ2[65]*tmpFx[8] + tmpQ2[66]*tmpFx[20] + tmpQ2[67]*tmpFx[32] + tmpQ2[68]*tmpFx[44] + tmpQ2[69]*tmpFx[56] + tmpQ2[70]*tmpFx[68] + tmpQ2[71]*tmpFx[80] + tmpQ2[72]*tmpFx[92] + tmpQ2[73]*tmpFx[104] + tmpQ2[74]*tmpFx[116] + tmpQ2[75]*tmpFx[128] + tmpQ2[76]*tmpFx[140] + tmpQ2[77]*tmpFx[152];
tmpQ1[69] = + tmpQ2[65]*tmpFx[9] + tmpQ2[66]*tmpFx[21] + tmpQ2[67]*tmpFx[33] + tmpQ2[68]*tmpFx[45] + tmpQ2[69]*tmpFx[57] + tmpQ2[70]*tmpFx[69] + tmpQ2[71]*tmpFx[81] + tmpQ2[72]*tmpFx[93] + tmpQ2[73]*tmpFx[105] + tmpQ2[74]*tmpFx[117] + tmpQ2[75]*tmpFx[129] + tmpQ2[76]*tmpFx[141] + tmpQ2[77]*tmpFx[153];
tmpQ1[70] = + tmpQ2[65]*tmpFx[10] + tmpQ2[66]*tmpFx[22] + tmpQ2[67]*tmpFx[34] + tmpQ2[68]*tmpFx[46] + tmpQ2[69]*tmpFx[58] + tmpQ2[70]*tmpFx[70] + tmpQ2[71]*tmpFx[82] + tmpQ2[72]*tmpFx[94] + tmpQ2[73]*tmpFx[106] + tmpQ2[74]*tmpFx[118] + tmpQ2[75]*tmpFx[130] + tmpQ2[76]*tmpFx[142] + tmpQ2[77]*tmpFx[154];
tmpQ1[71] = + tmpQ2[65]*tmpFx[11] + tmpQ2[66]*tmpFx[23] + tmpQ2[67]*tmpFx[35] + tmpQ2[68]*tmpFx[47] + tmpQ2[69]*tmpFx[59] + tmpQ2[70]*tmpFx[71] + tmpQ2[71]*tmpFx[83] + tmpQ2[72]*tmpFx[95] + tmpQ2[73]*tmpFx[107] + tmpQ2[74]*tmpFx[119] + tmpQ2[75]*tmpFx[131] + tmpQ2[76]*tmpFx[143] + tmpQ2[77]*tmpFx[155];
tmpQ1[72] = + tmpQ2[78]*tmpFx[0] + tmpQ2[79]*tmpFx[12] + tmpQ2[80]*tmpFx[24] + tmpQ2[81]*tmpFx[36] + tmpQ2[82]*tmpFx[48] + tmpQ2[83]*tmpFx[60] + tmpQ2[84]*tmpFx[72] + tmpQ2[85]*tmpFx[84] + tmpQ2[86]*tmpFx[96] + tmpQ2[87]*tmpFx[108] + tmpQ2[88]*tmpFx[120] + tmpQ2[89]*tmpFx[132] + tmpQ2[90]*tmpFx[144];
tmpQ1[73] = + tmpQ2[78]*tmpFx[1] + tmpQ2[79]*tmpFx[13] + tmpQ2[80]*tmpFx[25] + tmpQ2[81]*tmpFx[37] + tmpQ2[82]*tmpFx[49] + tmpQ2[83]*tmpFx[61] + tmpQ2[84]*tmpFx[73] + tmpQ2[85]*tmpFx[85] + tmpQ2[86]*tmpFx[97] + tmpQ2[87]*tmpFx[109] + tmpQ2[88]*tmpFx[121] + tmpQ2[89]*tmpFx[133] + tmpQ2[90]*tmpFx[145];
tmpQ1[74] = + tmpQ2[78]*tmpFx[2] + tmpQ2[79]*tmpFx[14] + tmpQ2[80]*tmpFx[26] + tmpQ2[81]*tmpFx[38] + tmpQ2[82]*tmpFx[50] + tmpQ2[83]*tmpFx[62] + tmpQ2[84]*tmpFx[74] + tmpQ2[85]*tmpFx[86] + tmpQ2[86]*tmpFx[98] + tmpQ2[87]*tmpFx[110] + tmpQ2[88]*tmpFx[122] + tmpQ2[89]*tmpFx[134] + tmpQ2[90]*tmpFx[146];
tmpQ1[75] = + tmpQ2[78]*tmpFx[3] + tmpQ2[79]*tmpFx[15] + tmpQ2[80]*tmpFx[27] + tmpQ2[81]*tmpFx[39] + tmpQ2[82]*tmpFx[51] + tmpQ2[83]*tmpFx[63] + tmpQ2[84]*tmpFx[75] + tmpQ2[85]*tmpFx[87] + tmpQ2[86]*tmpFx[99] + tmpQ2[87]*tmpFx[111] + tmpQ2[88]*tmpFx[123] + tmpQ2[89]*tmpFx[135] + tmpQ2[90]*tmpFx[147];
tmpQ1[76] = + tmpQ2[78]*tmpFx[4] + tmpQ2[79]*tmpFx[16] + tmpQ2[80]*tmpFx[28] + tmpQ2[81]*tmpFx[40] + tmpQ2[82]*tmpFx[52] + tmpQ2[83]*tmpFx[64] + tmpQ2[84]*tmpFx[76] + tmpQ2[85]*tmpFx[88] + tmpQ2[86]*tmpFx[100] + tmpQ2[87]*tmpFx[112] + tmpQ2[88]*tmpFx[124] + tmpQ2[89]*tmpFx[136] + tmpQ2[90]*tmpFx[148];
tmpQ1[77] = + tmpQ2[78]*tmpFx[5] + tmpQ2[79]*tmpFx[17] + tmpQ2[80]*tmpFx[29] + tmpQ2[81]*tmpFx[41] + tmpQ2[82]*tmpFx[53] + tmpQ2[83]*tmpFx[65] + tmpQ2[84]*tmpFx[77] + tmpQ2[85]*tmpFx[89] + tmpQ2[86]*tmpFx[101] + tmpQ2[87]*tmpFx[113] + tmpQ2[88]*tmpFx[125] + tmpQ2[89]*tmpFx[137] + tmpQ2[90]*tmpFx[149];
tmpQ1[78] = + tmpQ2[78]*tmpFx[6] + tmpQ2[79]*tmpFx[18] + tmpQ2[80]*tmpFx[30] + tmpQ2[81]*tmpFx[42] + tmpQ2[82]*tmpFx[54] + tmpQ2[83]*tmpFx[66] + tmpQ2[84]*tmpFx[78] + tmpQ2[85]*tmpFx[90] + tmpQ2[86]*tmpFx[102] + tmpQ2[87]*tmpFx[114] + tmpQ2[88]*tmpFx[126] + tmpQ2[89]*tmpFx[138] + tmpQ2[90]*tmpFx[150];
tmpQ1[79] = + tmpQ2[78]*tmpFx[7] + tmpQ2[79]*tmpFx[19] + tmpQ2[80]*tmpFx[31] + tmpQ2[81]*tmpFx[43] + tmpQ2[82]*tmpFx[55] + tmpQ2[83]*tmpFx[67] + tmpQ2[84]*tmpFx[79] + tmpQ2[85]*tmpFx[91] + tmpQ2[86]*tmpFx[103] + tmpQ2[87]*tmpFx[115] + tmpQ2[88]*tmpFx[127] + tmpQ2[89]*tmpFx[139] + tmpQ2[90]*tmpFx[151];
tmpQ1[80] = + tmpQ2[78]*tmpFx[8] + tmpQ2[79]*tmpFx[20] + tmpQ2[80]*tmpFx[32] + tmpQ2[81]*tmpFx[44] + tmpQ2[82]*tmpFx[56] + tmpQ2[83]*tmpFx[68] + tmpQ2[84]*tmpFx[80] + tmpQ2[85]*tmpFx[92] + tmpQ2[86]*tmpFx[104] + tmpQ2[87]*tmpFx[116] + tmpQ2[88]*tmpFx[128] + tmpQ2[89]*tmpFx[140] + tmpQ2[90]*tmpFx[152];
tmpQ1[81] = + tmpQ2[78]*tmpFx[9] + tmpQ2[79]*tmpFx[21] + tmpQ2[80]*tmpFx[33] + tmpQ2[81]*tmpFx[45] + tmpQ2[82]*tmpFx[57] + tmpQ2[83]*tmpFx[69] + tmpQ2[84]*tmpFx[81] + tmpQ2[85]*tmpFx[93] + tmpQ2[86]*tmpFx[105] + tmpQ2[87]*tmpFx[117] + tmpQ2[88]*tmpFx[129] + tmpQ2[89]*tmpFx[141] + tmpQ2[90]*tmpFx[153];
tmpQ1[82] = + tmpQ2[78]*tmpFx[10] + tmpQ2[79]*tmpFx[22] + tmpQ2[80]*tmpFx[34] + tmpQ2[81]*tmpFx[46] + tmpQ2[82]*tmpFx[58] + tmpQ2[83]*tmpFx[70] + tmpQ2[84]*tmpFx[82] + tmpQ2[85]*tmpFx[94] + tmpQ2[86]*tmpFx[106] + tmpQ2[87]*tmpFx[118] + tmpQ2[88]*tmpFx[130] + tmpQ2[89]*tmpFx[142] + tmpQ2[90]*tmpFx[154];
tmpQ1[83] = + tmpQ2[78]*tmpFx[11] + tmpQ2[79]*tmpFx[23] + tmpQ2[80]*tmpFx[35] + tmpQ2[81]*tmpFx[47] + tmpQ2[82]*tmpFx[59] + tmpQ2[83]*tmpFx[71] + tmpQ2[84]*tmpFx[83] + tmpQ2[85]*tmpFx[95] + tmpQ2[86]*tmpFx[107] + tmpQ2[87]*tmpFx[119] + tmpQ2[88]*tmpFx[131] + tmpQ2[89]*tmpFx[143] + tmpQ2[90]*tmpFx[155];
tmpQ1[84] = + tmpQ2[91]*tmpFx[0] + tmpQ2[92]*tmpFx[12] + tmpQ2[93]*tmpFx[24] + tmpQ2[94]*tmpFx[36] + tmpQ2[95]*tmpFx[48] + tmpQ2[96]*tmpFx[60] + tmpQ2[97]*tmpFx[72] + tmpQ2[98]*tmpFx[84] + tmpQ2[99]*tmpFx[96] + tmpQ2[100]*tmpFx[108] + tmpQ2[101]*tmpFx[120] + tmpQ2[102]*tmpFx[132] + tmpQ2[103]*tmpFx[144];
tmpQ1[85] = + tmpQ2[91]*tmpFx[1] + tmpQ2[92]*tmpFx[13] + tmpQ2[93]*tmpFx[25] + tmpQ2[94]*tmpFx[37] + tmpQ2[95]*tmpFx[49] + tmpQ2[96]*tmpFx[61] + tmpQ2[97]*tmpFx[73] + tmpQ2[98]*tmpFx[85] + tmpQ2[99]*tmpFx[97] + tmpQ2[100]*tmpFx[109] + tmpQ2[101]*tmpFx[121] + tmpQ2[102]*tmpFx[133] + tmpQ2[103]*tmpFx[145];
tmpQ1[86] = + tmpQ2[91]*tmpFx[2] + tmpQ2[92]*tmpFx[14] + tmpQ2[93]*tmpFx[26] + tmpQ2[94]*tmpFx[38] + tmpQ2[95]*tmpFx[50] + tmpQ2[96]*tmpFx[62] + tmpQ2[97]*tmpFx[74] + tmpQ2[98]*tmpFx[86] + tmpQ2[99]*tmpFx[98] + tmpQ2[100]*tmpFx[110] + tmpQ2[101]*tmpFx[122] + tmpQ2[102]*tmpFx[134] + tmpQ2[103]*tmpFx[146];
tmpQ1[87] = + tmpQ2[91]*tmpFx[3] + tmpQ2[92]*tmpFx[15] + tmpQ2[93]*tmpFx[27] + tmpQ2[94]*tmpFx[39] + tmpQ2[95]*tmpFx[51] + tmpQ2[96]*tmpFx[63] + tmpQ2[97]*tmpFx[75] + tmpQ2[98]*tmpFx[87] + tmpQ2[99]*tmpFx[99] + tmpQ2[100]*tmpFx[111] + tmpQ2[101]*tmpFx[123] + tmpQ2[102]*tmpFx[135] + tmpQ2[103]*tmpFx[147];
tmpQ1[88] = + tmpQ2[91]*tmpFx[4] + tmpQ2[92]*tmpFx[16] + tmpQ2[93]*tmpFx[28] + tmpQ2[94]*tmpFx[40] + tmpQ2[95]*tmpFx[52] + tmpQ2[96]*tmpFx[64] + tmpQ2[97]*tmpFx[76] + tmpQ2[98]*tmpFx[88] + tmpQ2[99]*tmpFx[100] + tmpQ2[100]*tmpFx[112] + tmpQ2[101]*tmpFx[124] + tmpQ2[102]*tmpFx[136] + tmpQ2[103]*tmpFx[148];
tmpQ1[89] = + tmpQ2[91]*tmpFx[5] + tmpQ2[92]*tmpFx[17] + tmpQ2[93]*tmpFx[29] + tmpQ2[94]*tmpFx[41] + tmpQ2[95]*tmpFx[53] + tmpQ2[96]*tmpFx[65] + tmpQ2[97]*tmpFx[77] + tmpQ2[98]*tmpFx[89] + tmpQ2[99]*tmpFx[101] + tmpQ2[100]*tmpFx[113] + tmpQ2[101]*tmpFx[125] + tmpQ2[102]*tmpFx[137] + tmpQ2[103]*tmpFx[149];
tmpQ1[90] = + tmpQ2[91]*tmpFx[6] + tmpQ2[92]*tmpFx[18] + tmpQ2[93]*tmpFx[30] + tmpQ2[94]*tmpFx[42] + tmpQ2[95]*tmpFx[54] + tmpQ2[96]*tmpFx[66] + tmpQ2[97]*tmpFx[78] + tmpQ2[98]*tmpFx[90] + tmpQ2[99]*tmpFx[102] + tmpQ2[100]*tmpFx[114] + tmpQ2[101]*tmpFx[126] + tmpQ2[102]*tmpFx[138] + tmpQ2[103]*tmpFx[150];
tmpQ1[91] = + tmpQ2[91]*tmpFx[7] + tmpQ2[92]*tmpFx[19] + tmpQ2[93]*tmpFx[31] + tmpQ2[94]*tmpFx[43] + tmpQ2[95]*tmpFx[55] + tmpQ2[96]*tmpFx[67] + tmpQ2[97]*tmpFx[79] + tmpQ2[98]*tmpFx[91] + tmpQ2[99]*tmpFx[103] + tmpQ2[100]*tmpFx[115] + tmpQ2[101]*tmpFx[127] + tmpQ2[102]*tmpFx[139] + tmpQ2[103]*tmpFx[151];
tmpQ1[92] = + tmpQ2[91]*tmpFx[8] + tmpQ2[92]*tmpFx[20] + tmpQ2[93]*tmpFx[32] + tmpQ2[94]*tmpFx[44] + tmpQ2[95]*tmpFx[56] + tmpQ2[96]*tmpFx[68] + tmpQ2[97]*tmpFx[80] + tmpQ2[98]*tmpFx[92] + tmpQ2[99]*tmpFx[104] + tmpQ2[100]*tmpFx[116] + tmpQ2[101]*tmpFx[128] + tmpQ2[102]*tmpFx[140] + tmpQ2[103]*tmpFx[152];
tmpQ1[93] = + tmpQ2[91]*tmpFx[9] + tmpQ2[92]*tmpFx[21] + tmpQ2[93]*tmpFx[33] + tmpQ2[94]*tmpFx[45] + tmpQ2[95]*tmpFx[57] + tmpQ2[96]*tmpFx[69] + tmpQ2[97]*tmpFx[81] + tmpQ2[98]*tmpFx[93] + tmpQ2[99]*tmpFx[105] + tmpQ2[100]*tmpFx[117] + tmpQ2[101]*tmpFx[129] + tmpQ2[102]*tmpFx[141] + tmpQ2[103]*tmpFx[153];
tmpQ1[94] = + tmpQ2[91]*tmpFx[10] + tmpQ2[92]*tmpFx[22] + tmpQ2[93]*tmpFx[34] + tmpQ2[94]*tmpFx[46] + tmpQ2[95]*tmpFx[58] + tmpQ2[96]*tmpFx[70] + tmpQ2[97]*tmpFx[82] + tmpQ2[98]*tmpFx[94] + tmpQ2[99]*tmpFx[106] + tmpQ2[100]*tmpFx[118] + tmpQ2[101]*tmpFx[130] + tmpQ2[102]*tmpFx[142] + tmpQ2[103]*tmpFx[154];
tmpQ1[95] = + tmpQ2[91]*tmpFx[11] + tmpQ2[92]*tmpFx[23] + tmpQ2[93]*tmpFx[35] + tmpQ2[94]*tmpFx[47] + tmpQ2[95]*tmpFx[59] + tmpQ2[96]*tmpFx[71] + tmpQ2[97]*tmpFx[83] + tmpQ2[98]*tmpFx[95] + tmpQ2[99]*tmpFx[107] + tmpQ2[100]*tmpFx[119] + tmpQ2[101]*tmpFx[131] + tmpQ2[102]*tmpFx[143] + tmpQ2[103]*tmpFx[155];
tmpQ1[96] = + tmpQ2[104]*tmpFx[0] + tmpQ2[105]*tmpFx[12] + tmpQ2[106]*tmpFx[24] + tmpQ2[107]*tmpFx[36] + tmpQ2[108]*tmpFx[48] + tmpQ2[109]*tmpFx[60] + tmpQ2[110]*tmpFx[72] + tmpQ2[111]*tmpFx[84] + tmpQ2[112]*tmpFx[96] + tmpQ2[113]*tmpFx[108] + tmpQ2[114]*tmpFx[120] + tmpQ2[115]*tmpFx[132] + tmpQ2[116]*tmpFx[144];
tmpQ1[97] = + tmpQ2[104]*tmpFx[1] + tmpQ2[105]*tmpFx[13] + tmpQ2[106]*tmpFx[25] + tmpQ2[107]*tmpFx[37] + tmpQ2[108]*tmpFx[49] + tmpQ2[109]*tmpFx[61] + tmpQ2[110]*tmpFx[73] + tmpQ2[111]*tmpFx[85] + tmpQ2[112]*tmpFx[97] + tmpQ2[113]*tmpFx[109] + tmpQ2[114]*tmpFx[121] + tmpQ2[115]*tmpFx[133] + tmpQ2[116]*tmpFx[145];
tmpQ1[98] = + tmpQ2[104]*tmpFx[2] + tmpQ2[105]*tmpFx[14] + tmpQ2[106]*tmpFx[26] + tmpQ2[107]*tmpFx[38] + tmpQ2[108]*tmpFx[50] + tmpQ2[109]*tmpFx[62] + tmpQ2[110]*tmpFx[74] + tmpQ2[111]*tmpFx[86] + tmpQ2[112]*tmpFx[98] + tmpQ2[113]*tmpFx[110] + tmpQ2[114]*tmpFx[122] + tmpQ2[115]*tmpFx[134] + tmpQ2[116]*tmpFx[146];
tmpQ1[99] = + tmpQ2[104]*tmpFx[3] + tmpQ2[105]*tmpFx[15] + tmpQ2[106]*tmpFx[27] + tmpQ2[107]*tmpFx[39] + tmpQ2[108]*tmpFx[51] + tmpQ2[109]*tmpFx[63] + tmpQ2[110]*tmpFx[75] + tmpQ2[111]*tmpFx[87] + tmpQ2[112]*tmpFx[99] + tmpQ2[113]*tmpFx[111] + tmpQ2[114]*tmpFx[123] + tmpQ2[115]*tmpFx[135] + tmpQ2[116]*tmpFx[147];
tmpQ1[100] = + tmpQ2[104]*tmpFx[4] + tmpQ2[105]*tmpFx[16] + tmpQ2[106]*tmpFx[28] + tmpQ2[107]*tmpFx[40] + tmpQ2[108]*tmpFx[52] + tmpQ2[109]*tmpFx[64] + tmpQ2[110]*tmpFx[76] + tmpQ2[111]*tmpFx[88] + tmpQ2[112]*tmpFx[100] + tmpQ2[113]*tmpFx[112] + tmpQ2[114]*tmpFx[124] + tmpQ2[115]*tmpFx[136] + tmpQ2[116]*tmpFx[148];
tmpQ1[101] = + tmpQ2[104]*tmpFx[5] + tmpQ2[105]*tmpFx[17] + tmpQ2[106]*tmpFx[29] + tmpQ2[107]*tmpFx[41] + tmpQ2[108]*tmpFx[53] + tmpQ2[109]*tmpFx[65] + tmpQ2[110]*tmpFx[77] + tmpQ2[111]*tmpFx[89] + tmpQ2[112]*tmpFx[101] + tmpQ2[113]*tmpFx[113] + tmpQ2[114]*tmpFx[125] + tmpQ2[115]*tmpFx[137] + tmpQ2[116]*tmpFx[149];
tmpQ1[102] = + tmpQ2[104]*tmpFx[6] + tmpQ2[105]*tmpFx[18] + tmpQ2[106]*tmpFx[30] + tmpQ2[107]*tmpFx[42] + tmpQ2[108]*tmpFx[54] + tmpQ2[109]*tmpFx[66] + tmpQ2[110]*tmpFx[78] + tmpQ2[111]*tmpFx[90] + tmpQ2[112]*tmpFx[102] + tmpQ2[113]*tmpFx[114] + tmpQ2[114]*tmpFx[126] + tmpQ2[115]*tmpFx[138] + tmpQ2[116]*tmpFx[150];
tmpQ1[103] = + tmpQ2[104]*tmpFx[7] + tmpQ2[105]*tmpFx[19] + tmpQ2[106]*tmpFx[31] + tmpQ2[107]*tmpFx[43] + tmpQ2[108]*tmpFx[55] + tmpQ2[109]*tmpFx[67] + tmpQ2[110]*tmpFx[79] + tmpQ2[111]*tmpFx[91] + tmpQ2[112]*tmpFx[103] + tmpQ2[113]*tmpFx[115] + tmpQ2[114]*tmpFx[127] + tmpQ2[115]*tmpFx[139] + tmpQ2[116]*tmpFx[151];
tmpQ1[104] = + tmpQ2[104]*tmpFx[8] + tmpQ2[105]*tmpFx[20] + tmpQ2[106]*tmpFx[32] + tmpQ2[107]*tmpFx[44] + tmpQ2[108]*tmpFx[56] + tmpQ2[109]*tmpFx[68] + tmpQ2[110]*tmpFx[80] + tmpQ2[111]*tmpFx[92] + tmpQ2[112]*tmpFx[104] + tmpQ2[113]*tmpFx[116] + tmpQ2[114]*tmpFx[128] + tmpQ2[115]*tmpFx[140] + tmpQ2[116]*tmpFx[152];
tmpQ1[105] = + tmpQ2[104]*tmpFx[9] + tmpQ2[105]*tmpFx[21] + tmpQ2[106]*tmpFx[33] + tmpQ2[107]*tmpFx[45] + tmpQ2[108]*tmpFx[57] + tmpQ2[109]*tmpFx[69] + tmpQ2[110]*tmpFx[81] + tmpQ2[111]*tmpFx[93] + tmpQ2[112]*tmpFx[105] + tmpQ2[113]*tmpFx[117] + tmpQ2[114]*tmpFx[129] + tmpQ2[115]*tmpFx[141] + tmpQ2[116]*tmpFx[153];
tmpQ1[106] = + tmpQ2[104]*tmpFx[10] + tmpQ2[105]*tmpFx[22] + tmpQ2[106]*tmpFx[34] + tmpQ2[107]*tmpFx[46] + tmpQ2[108]*tmpFx[58] + tmpQ2[109]*tmpFx[70] + tmpQ2[110]*tmpFx[82] + tmpQ2[111]*tmpFx[94] + tmpQ2[112]*tmpFx[106] + tmpQ2[113]*tmpFx[118] + tmpQ2[114]*tmpFx[130] + tmpQ2[115]*tmpFx[142] + tmpQ2[116]*tmpFx[154];
tmpQ1[107] = + tmpQ2[104]*tmpFx[11] + tmpQ2[105]*tmpFx[23] + tmpQ2[106]*tmpFx[35] + tmpQ2[107]*tmpFx[47] + tmpQ2[108]*tmpFx[59] + tmpQ2[109]*tmpFx[71] + tmpQ2[110]*tmpFx[83] + tmpQ2[111]*tmpFx[95] + tmpQ2[112]*tmpFx[107] + tmpQ2[113]*tmpFx[119] + tmpQ2[114]*tmpFx[131] + tmpQ2[115]*tmpFx[143] + tmpQ2[116]*tmpFx[155];
tmpQ1[108] = + tmpQ2[117]*tmpFx[0] + tmpQ2[118]*tmpFx[12] + tmpQ2[119]*tmpFx[24] + tmpQ2[120]*tmpFx[36] + tmpQ2[121]*tmpFx[48] + tmpQ2[122]*tmpFx[60] + tmpQ2[123]*tmpFx[72] + tmpQ2[124]*tmpFx[84] + tmpQ2[125]*tmpFx[96] + tmpQ2[126]*tmpFx[108] + tmpQ2[127]*tmpFx[120] + tmpQ2[128]*tmpFx[132] + tmpQ2[129]*tmpFx[144];
tmpQ1[109] = + tmpQ2[117]*tmpFx[1] + tmpQ2[118]*tmpFx[13] + tmpQ2[119]*tmpFx[25] + tmpQ2[120]*tmpFx[37] + tmpQ2[121]*tmpFx[49] + tmpQ2[122]*tmpFx[61] + tmpQ2[123]*tmpFx[73] + tmpQ2[124]*tmpFx[85] + tmpQ2[125]*tmpFx[97] + tmpQ2[126]*tmpFx[109] + tmpQ2[127]*tmpFx[121] + tmpQ2[128]*tmpFx[133] + tmpQ2[129]*tmpFx[145];
tmpQ1[110] = + tmpQ2[117]*tmpFx[2] + tmpQ2[118]*tmpFx[14] + tmpQ2[119]*tmpFx[26] + tmpQ2[120]*tmpFx[38] + tmpQ2[121]*tmpFx[50] + tmpQ2[122]*tmpFx[62] + tmpQ2[123]*tmpFx[74] + tmpQ2[124]*tmpFx[86] + tmpQ2[125]*tmpFx[98] + tmpQ2[126]*tmpFx[110] + tmpQ2[127]*tmpFx[122] + tmpQ2[128]*tmpFx[134] + tmpQ2[129]*tmpFx[146];
tmpQ1[111] = + tmpQ2[117]*tmpFx[3] + tmpQ2[118]*tmpFx[15] + tmpQ2[119]*tmpFx[27] + tmpQ2[120]*tmpFx[39] + tmpQ2[121]*tmpFx[51] + tmpQ2[122]*tmpFx[63] + tmpQ2[123]*tmpFx[75] + tmpQ2[124]*tmpFx[87] + tmpQ2[125]*tmpFx[99] + tmpQ2[126]*tmpFx[111] + tmpQ2[127]*tmpFx[123] + tmpQ2[128]*tmpFx[135] + tmpQ2[129]*tmpFx[147];
tmpQ1[112] = + tmpQ2[117]*tmpFx[4] + tmpQ2[118]*tmpFx[16] + tmpQ2[119]*tmpFx[28] + tmpQ2[120]*tmpFx[40] + tmpQ2[121]*tmpFx[52] + tmpQ2[122]*tmpFx[64] + tmpQ2[123]*tmpFx[76] + tmpQ2[124]*tmpFx[88] + tmpQ2[125]*tmpFx[100] + tmpQ2[126]*tmpFx[112] + tmpQ2[127]*tmpFx[124] + tmpQ2[128]*tmpFx[136] + tmpQ2[129]*tmpFx[148];
tmpQ1[113] = + tmpQ2[117]*tmpFx[5] + tmpQ2[118]*tmpFx[17] + tmpQ2[119]*tmpFx[29] + tmpQ2[120]*tmpFx[41] + tmpQ2[121]*tmpFx[53] + tmpQ2[122]*tmpFx[65] + tmpQ2[123]*tmpFx[77] + tmpQ2[124]*tmpFx[89] + tmpQ2[125]*tmpFx[101] + tmpQ2[126]*tmpFx[113] + tmpQ2[127]*tmpFx[125] + tmpQ2[128]*tmpFx[137] + tmpQ2[129]*tmpFx[149];
tmpQ1[114] = + tmpQ2[117]*tmpFx[6] + tmpQ2[118]*tmpFx[18] + tmpQ2[119]*tmpFx[30] + tmpQ2[120]*tmpFx[42] + tmpQ2[121]*tmpFx[54] + tmpQ2[122]*tmpFx[66] + tmpQ2[123]*tmpFx[78] + tmpQ2[124]*tmpFx[90] + tmpQ2[125]*tmpFx[102] + tmpQ2[126]*tmpFx[114] + tmpQ2[127]*tmpFx[126] + tmpQ2[128]*tmpFx[138] + tmpQ2[129]*tmpFx[150];
tmpQ1[115] = + tmpQ2[117]*tmpFx[7] + tmpQ2[118]*tmpFx[19] + tmpQ2[119]*tmpFx[31] + tmpQ2[120]*tmpFx[43] + tmpQ2[121]*tmpFx[55] + tmpQ2[122]*tmpFx[67] + tmpQ2[123]*tmpFx[79] + tmpQ2[124]*tmpFx[91] + tmpQ2[125]*tmpFx[103] + tmpQ2[126]*tmpFx[115] + tmpQ2[127]*tmpFx[127] + tmpQ2[128]*tmpFx[139] + tmpQ2[129]*tmpFx[151];
tmpQ1[116] = + tmpQ2[117]*tmpFx[8] + tmpQ2[118]*tmpFx[20] + tmpQ2[119]*tmpFx[32] + tmpQ2[120]*tmpFx[44] + tmpQ2[121]*tmpFx[56] + tmpQ2[122]*tmpFx[68] + tmpQ2[123]*tmpFx[80] + tmpQ2[124]*tmpFx[92] + tmpQ2[125]*tmpFx[104] + tmpQ2[126]*tmpFx[116] + tmpQ2[127]*tmpFx[128] + tmpQ2[128]*tmpFx[140] + tmpQ2[129]*tmpFx[152];
tmpQ1[117] = + tmpQ2[117]*tmpFx[9] + tmpQ2[118]*tmpFx[21] + tmpQ2[119]*tmpFx[33] + tmpQ2[120]*tmpFx[45] + tmpQ2[121]*tmpFx[57] + tmpQ2[122]*tmpFx[69] + tmpQ2[123]*tmpFx[81] + tmpQ2[124]*tmpFx[93] + tmpQ2[125]*tmpFx[105] + tmpQ2[126]*tmpFx[117] + tmpQ2[127]*tmpFx[129] + tmpQ2[128]*tmpFx[141] + tmpQ2[129]*tmpFx[153];
tmpQ1[118] = + tmpQ2[117]*tmpFx[10] + tmpQ2[118]*tmpFx[22] + tmpQ2[119]*tmpFx[34] + tmpQ2[120]*tmpFx[46] + tmpQ2[121]*tmpFx[58] + tmpQ2[122]*tmpFx[70] + tmpQ2[123]*tmpFx[82] + tmpQ2[124]*tmpFx[94] + tmpQ2[125]*tmpFx[106] + tmpQ2[126]*tmpFx[118] + tmpQ2[127]*tmpFx[130] + tmpQ2[128]*tmpFx[142] + tmpQ2[129]*tmpFx[154];
tmpQ1[119] = + tmpQ2[117]*tmpFx[11] + tmpQ2[118]*tmpFx[23] + tmpQ2[119]*tmpFx[35] + tmpQ2[120]*tmpFx[47] + tmpQ2[121]*tmpFx[59] + tmpQ2[122]*tmpFx[71] + tmpQ2[123]*tmpFx[83] + tmpQ2[124]*tmpFx[95] + tmpQ2[125]*tmpFx[107] + tmpQ2[126]*tmpFx[119] + tmpQ2[127]*tmpFx[131] + tmpQ2[128]*tmpFx[143] + tmpQ2[129]*tmpFx[155];
tmpQ1[120] = + tmpQ2[130]*tmpFx[0] + tmpQ2[131]*tmpFx[12] + tmpQ2[132]*tmpFx[24] + tmpQ2[133]*tmpFx[36] + tmpQ2[134]*tmpFx[48] + tmpQ2[135]*tmpFx[60] + tmpQ2[136]*tmpFx[72] + tmpQ2[137]*tmpFx[84] + tmpQ2[138]*tmpFx[96] + tmpQ2[139]*tmpFx[108] + tmpQ2[140]*tmpFx[120] + tmpQ2[141]*tmpFx[132] + tmpQ2[142]*tmpFx[144];
tmpQ1[121] = + tmpQ2[130]*tmpFx[1] + tmpQ2[131]*tmpFx[13] + tmpQ2[132]*tmpFx[25] + tmpQ2[133]*tmpFx[37] + tmpQ2[134]*tmpFx[49] + tmpQ2[135]*tmpFx[61] + tmpQ2[136]*tmpFx[73] + tmpQ2[137]*tmpFx[85] + tmpQ2[138]*tmpFx[97] + tmpQ2[139]*tmpFx[109] + tmpQ2[140]*tmpFx[121] + tmpQ2[141]*tmpFx[133] + tmpQ2[142]*tmpFx[145];
tmpQ1[122] = + tmpQ2[130]*tmpFx[2] + tmpQ2[131]*tmpFx[14] + tmpQ2[132]*tmpFx[26] + tmpQ2[133]*tmpFx[38] + tmpQ2[134]*tmpFx[50] + tmpQ2[135]*tmpFx[62] + tmpQ2[136]*tmpFx[74] + tmpQ2[137]*tmpFx[86] + tmpQ2[138]*tmpFx[98] + tmpQ2[139]*tmpFx[110] + tmpQ2[140]*tmpFx[122] + tmpQ2[141]*tmpFx[134] + tmpQ2[142]*tmpFx[146];
tmpQ1[123] = + tmpQ2[130]*tmpFx[3] + tmpQ2[131]*tmpFx[15] + tmpQ2[132]*tmpFx[27] + tmpQ2[133]*tmpFx[39] + tmpQ2[134]*tmpFx[51] + tmpQ2[135]*tmpFx[63] + tmpQ2[136]*tmpFx[75] + tmpQ2[137]*tmpFx[87] + tmpQ2[138]*tmpFx[99] + tmpQ2[139]*tmpFx[111] + tmpQ2[140]*tmpFx[123] + tmpQ2[141]*tmpFx[135] + tmpQ2[142]*tmpFx[147];
tmpQ1[124] = + tmpQ2[130]*tmpFx[4] + tmpQ2[131]*tmpFx[16] + tmpQ2[132]*tmpFx[28] + tmpQ2[133]*tmpFx[40] + tmpQ2[134]*tmpFx[52] + tmpQ2[135]*tmpFx[64] + tmpQ2[136]*tmpFx[76] + tmpQ2[137]*tmpFx[88] + tmpQ2[138]*tmpFx[100] + tmpQ2[139]*tmpFx[112] + tmpQ2[140]*tmpFx[124] + tmpQ2[141]*tmpFx[136] + tmpQ2[142]*tmpFx[148];
tmpQ1[125] = + tmpQ2[130]*tmpFx[5] + tmpQ2[131]*tmpFx[17] + tmpQ2[132]*tmpFx[29] + tmpQ2[133]*tmpFx[41] + tmpQ2[134]*tmpFx[53] + tmpQ2[135]*tmpFx[65] + tmpQ2[136]*tmpFx[77] + tmpQ2[137]*tmpFx[89] + tmpQ2[138]*tmpFx[101] + tmpQ2[139]*tmpFx[113] + tmpQ2[140]*tmpFx[125] + tmpQ2[141]*tmpFx[137] + tmpQ2[142]*tmpFx[149];
tmpQ1[126] = + tmpQ2[130]*tmpFx[6] + tmpQ2[131]*tmpFx[18] + tmpQ2[132]*tmpFx[30] + tmpQ2[133]*tmpFx[42] + tmpQ2[134]*tmpFx[54] + tmpQ2[135]*tmpFx[66] + tmpQ2[136]*tmpFx[78] + tmpQ2[137]*tmpFx[90] + tmpQ2[138]*tmpFx[102] + tmpQ2[139]*tmpFx[114] + tmpQ2[140]*tmpFx[126] + tmpQ2[141]*tmpFx[138] + tmpQ2[142]*tmpFx[150];
tmpQ1[127] = + tmpQ2[130]*tmpFx[7] + tmpQ2[131]*tmpFx[19] + tmpQ2[132]*tmpFx[31] + tmpQ2[133]*tmpFx[43] + tmpQ2[134]*tmpFx[55] + tmpQ2[135]*tmpFx[67] + tmpQ2[136]*tmpFx[79] + tmpQ2[137]*tmpFx[91] + tmpQ2[138]*tmpFx[103] + tmpQ2[139]*tmpFx[115] + tmpQ2[140]*tmpFx[127] + tmpQ2[141]*tmpFx[139] + tmpQ2[142]*tmpFx[151];
tmpQ1[128] = + tmpQ2[130]*tmpFx[8] + tmpQ2[131]*tmpFx[20] + tmpQ2[132]*tmpFx[32] + tmpQ2[133]*tmpFx[44] + tmpQ2[134]*tmpFx[56] + tmpQ2[135]*tmpFx[68] + tmpQ2[136]*tmpFx[80] + tmpQ2[137]*tmpFx[92] + tmpQ2[138]*tmpFx[104] + tmpQ2[139]*tmpFx[116] + tmpQ2[140]*tmpFx[128] + tmpQ2[141]*tmpFx[140] + tmpQ2[142]*tmpFx[152];
tmpQ1[129] = + tmpQ2[130]*tmpFx[9] + tmpQ2[131]*tmpFx[21] + tmpQ2[132]*tmpFx[33] + tmpQ2[133]*tmpFx[45] + tmpQ2[134]*tmpFx[57] + tmpQ2[135]*tmpFx[69] + tmpQ2[136]*tmpFx[81] + tmpQ2[137]*tmpFx[93] + tmpQ2[138]*tmpFx[105] + tmpQ2[139]*tmpFx[117] + tmpQ2[140]*tmpFx[129] + tmpQ2[141]*tmpFx[141] + tmpQ2[142]*tmpFx[153];
tmpQ1[130] = + tmpQ2[130]*tmpFx[10] + tmpQ2[131]*tmpFx[22] + tmpQ2[132]*tmpFx[34] + tmpQ2[133]*tmpFx[46] + tmpQ2[134]*tmpFx[58] + tmpQ2[135]*tmpFx[70] + tmpQ2[136]*tmpFx[82] + tmpQ2[137]*tmpFx[94] + tmpQ2[138]*tmpFx[106] + tmpQ2[139]*tmpFx[118] + tmpQ2[140]*tmpFx[130] + tmpQ2[141]*tmpFx[142] + tmpQ2[142]*tmpFx[154];
tmpQ1[131] = + tmpQ2[130]*tmpFx[11] + tmpQ2[131]*tmpFx[23] + tmpQ2[132]*tmpFx[35] + tmpQ2[133]*tmpFx[47] + tmpQ2[134]*tmpFx[59] + tmpQ2[135]*tmpFx[71] + tmpQ2[136]*tmpFx[83] + tmpQ2[137]*tmpFx[95] + tmpQ2[138]*tmpFx[107] + tmpQ2[139]*tmpFx[119] + tmpQ2[140]*tmpFx[131] + tmpQ2[141]*tmpFx[143] + tmpQ2[142]*tmpFx[155];
tmpQ1[132] = + tmpQ2[143]*tmpFx[0] + tmpQ2[144]*tmpFx[12] + tmpQ2[145]*tmpFx[24] + tmpQ2[146]*tmpFx[36] + tmpQ2[147]*tmpFx[48] + tmpQ2[148]*tmpFx[60] + tmpQ2[149]*tmpFx[72] + tmpQ2[150]*tmpFx[84] + tmpQ2[151]*tmpFx[96] + tmpQ2[152]*tmpFx[108] + tmpQ2[153]*tmpFx[120] + tmpQ2[154]*tmpFx[132] + tmpQ2[155]*tmpFx[144];
tmpQ1[133] = + tmpQ2[143]*tmpFx[1] + tmpQ2[144]*tmpFx[13] + tmpQ2[145]*tmpFx[25] + tmpQ2[146]*tmpFx[37] + tmpQ2[147]*tmpFx[49] + tmpQ2[148]*tmpFx[61] + tmpQ2[149]*tmpFx[73] + tmpQ2[150]*tmpFx[85] + tmpQ2[151]*tmpFx[97] + tmpQ2[152]*tmpFx[109] + tmpQ2[153]*tmpFx[121] + tmpQ2[154]*tmpFx[133] + tmpQ2[155]*tmpFx[145];
tmpQ1[134] = + tmpQ2[143]*tmpFx[2] + tmpQ2[144]*tmpFx[14] + tmpQ2[145]*tmpFx[26] + tmpQ2[146]*tmpFx[38] + tmpQ2[147]*tmpFx[50] + tmpQ2[148]*tmpFx[62] + tmpQ2[149]*tmpFx[74] + tmpQ2[150]*tmpFx[86] + tmpQ2[151]*tmpFx[98] + tmpQ2[152]*tmpFx[110] + tmpQ2[153]*tmpFx[122] + tmpQ2[154]*tmpFx[134] + tmpQ2[155]*tmpFx[146];
tmpQ1[135] = + tmpQ2[143]*tmpFx[3] + tmpQ2[144]*tmpFx[15] + tmpQ2[145]*tmpFx[27] + tmpQ2[146]*tmpFx[39] + tmpQ2[147]*tmpFx[51] + tmpQ2[148]*tmpFx[63] + tmpQ2[149]*tmpFx[75] + tmpQ2[150]*tmpFx[87] + tmpQ2[151]*tmpFx[99] + tmpQ2[152]*tmpFx[111] + tmpQ2[153]*tmpFx[123] + tmpQ2[154]*tmpFx[135] + tmpQ2[155]*tmpFx[147];
tmpQ1[136] = + tmpQ2[143]*tmpFx[4] + tmpQ2[144]*tmpFx[16] + tmpQ2[145]*tmpFx[28] + tmpQ2[146]*tmpFx[40] + tmpQ2[147]*tmpFx[52] + tmpQ2[148]*tmpFx[64] + tmpQ2[149]*tmpFx[76] + tmpQ2[150]*tmpFx[88] + tmpQ2[151]*tmpFx[100] + tmpQ2[152]*tmpFx[112] + tmpQ2[153]*tmpFx[124] + tmpQ2[154]*tmpFx[136] + tmpQ2[155]*tmpFx[148];
tmpQ1[137] = + tmpQ2[143]*tmpFx[5] + tmpQ2[144]*tmpFx[17] + tmpQ2[145]*tmpFx[29] + tmpQ2[146]*tmpFx[41] + tmpQ2[147]*tmpFx[53] + tmpQ2[148]*tmpFx[65] + tmpQ2[149]*tmpFx[77] + tmpQ2[150]*tmpFx[89] + tmpQ2[151]*tmpFx[101] + tmpQ2[152]*tmpFx[113] + tmpQ2[153]*tmpFx[125] + tmpQ2[154]*tmpFx[137] + tmpQ2[155]*tmpFx[149];
tmpQ1[138] = + tmpQ2[143]*tmpFx[6] + tmpQ2[144]*tmpFx[18] + tmpQ2[145]*tmpFx[30] + tmpQ2[146]*tmpFx[42] + tmpQ2[147]*tmpFx[54] + tmpQ2[148]*tmpFx[66] + tmpQ2[149]*tmpFx[78] + tmpQ2[150]*tmpFx[90] + tmpQ2[151]*tmpFx[102] + tmpQ2[152]*tmpFx[114] + tmpQ2[153]*tmpFx[126] + tmpQ2[154]*tmpFx[138] + tmpQ2[155]*tmpFx[150];
tmpQ1[139] = + tmpQ2[143]*tmpFx[7] + tmpQ2[144]*tmpFx[19] + tmpQ2[145]*tmpFx[31] + tmpQ2[146]*tmpFx[43] + tmpQ2[147]*tmpFx[55] + tmpQ2[148]*tmpFx[67] + tmpQ2[149]*tmpFx[79] + tmpQ2[150]*tmpFx[91] + tmpQ2[151]*tmpFx[103] + tmpQ2[152]*tmpFx[115] + tmpQ2[153]*tmpFx[127] + tmpQ2[154]*tmpFx[139] + tmpQ2[155]*tmpFx[151];
tmpQ1[140] = + tmpQ2[143]*tmpFx[8] + tmpQ2[144]*tmpFx[20] + tmpQ2[145]*tmpFx[32] + tmpQ2[146]*tmpFx[44] + tmpQ2[147]*tmpFx[56] + tmpQ2[148]*tmpFx[68] + tmpQ2[149]*tmpFx[80] + tmpQ2[150]*tmpFx[92] + tmpQ2[151]*tmpFx[104] + tmpQ2[152]*tmpFx[116] + tmpQ2[153]*tmpFx[128] + tmpQ2[154]*tmpFx[140] + tmpQ2[155]*tmpFx[152];
tmpQ1[141] = + tmpQ2[143]*tmpFx[9] + tmpQ2[144]*tmpFx[21] + tmpQ2[145]*tmpFx[33] + tmpQ2[146]*tmpFx[45] + tmpQ2[147]*tmpFx[57] + tmpQ2[148]*tmpFx[69] + tmpQ2[149]*tmpFx[81] + tmpQ2[150]*tmpFx[93] + tmpQ2[151]*tmpFx[105] + tmpQ2[152]*tmpFx[117] + tmpQ2[153]*tmpFx[129] + tmpQ2[154]*tmpFx[141] + tmpQ2[155]*tmpFx[153];
tmpQ1[142] = + tmpQ2[143]*tmpFx[10] + tmpQ2[144]*tmpFx[22] + tmpQ2[145]*tmpFx[34] + tmpQ2[146]*tmpFx[46] + tmpQ2[147]*tmpFx[58] + tmpQ2[148]*tmpFx[70] + tmpQ2[149]*tmpFx[82] + tmpQ2[150]*tmpFx[94] + tmpQ2[151]*tmpFx[106] + tmpQ2[152]*tmpFx[118] + tmpQ2[153]*tmpFx[130] + tmpQ2[154]*tmpFx[142] + tmpQ2[155]*tmpFx[154];
tmpQ1[143] = + tmpQ2[143]*tmpFx[11] + tmpQ2[144]*tmpFx[23] + tmpQ2[145]*tmpFx[35] + tmpQ2[146]*tmpFx[47] + tmpQ2[147]*tmpFx[59] + tmpQ2[148]*tmpFx[71] + tmpQ2[149]*tmpFx[83] + tmpQ2[150]*tmpFx[95] + tmpQ2[151]*tmpFx[107] + tmpQ2[152]*tmpFx[119] + tmpQ2[153]*tmpFx[131] + tmpQ2[154]*tmpFx[143] + tmpQ2[155]*tmpFx[155];
}

void acado_setObjR1R2( real_t* const tmpFu, real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = + tmpFu[0]*tmpObjS[0] + tmpFu[3]*tmpObjS[13] + tmpFu[6]*tmpObjS[26] + tmpFu[9]*tmpObjS[39] + tmpFu[12]*tmpObjS[52] + tmpFu[15]*tmpObjS[65] + tmpFu[18]*tmpObjS[78] + tmpFu[21]*tmpObjS[91] + tmpFu[24]*tmpObjS[104] + tmpFu[27]*tmpObjS[117] + tmpFu[30]*tmpObjS[130] + tmpFu[33]*tmpObjS[143] + tmpFu[36]*tmpObjS[156];
tmpR2[1] = + tmpFu[0]*tmpObjS[1] + tmpFu[3]*tmpObjS[14] + tmpFu[6]*tmpObjS[27] + tmpFu[9]*tmpObjS[40] + tmpFu[12]*tmpObjS[53] + tmpFu[15]*tmpObjS[66] + tmpFu[18]*tmpObjS[79] + tmpFu[21]*tmpObjS[92] + tmpFu[24]*tmpObjS[105] + tmpFu[27]*tmpObjS[118] + tmpFu[30]*tmpObjS[131] + tmpFu[33]*tmpObjS[144] + tmpFu[36]*tmpObjS[157];
tmpR2[2] = + tmpFu[0]*tmpObjS[2] + tmpFu[3]*tmpObjS[15] + tmpFu[6]*tmpObjS[28] + tmpFu[9]*tmpObjS[41] + tmpFu[12]*tmpObjS[54] + tmpFu[15]*tmpObjS[67] + tmpFu[18]*tmpObjS[80] + tmpFu[21]*tmpObjS[93] + tmpFu[24]*tmpObjS[106] + tmpFu[27]*tmpObjS[119] + tmpFu[30]*tmpObjS[132] + tmpFu[33]*tmpObjS[145] + tmpFu[36]*tmpObjS[158];
tmpR2[3] = + tmpFu[0]*tmpObjS[3] + tmpFu[3]*tmpObjS[16] + tmpFu[6]*tmpObjS[29] + tmpFu[9]*tmpObjS[42] + tmpFu[12]*tmpObjS[55] + tmpFu[15]*tmpObjS[68] + tmpFu[18]*tmpObjS[81] + tmpFu[21]*tmpObjS[94] + tmpFu[24]*tmpObjS[107] + tmpFu[27]*tmpObjS[120] + tmpFu[30]*tmpObjS[133] + tmpFu[33]*tmpObjS[146] + tmpFu[36]*tmpObjS[159];
tmpR2[4] = + tmpFu[0]*tmpObjS[4] + tmpFu[3]*tmpObjS[17] + tmpFu[6]*tmpObjS[30] + tmpFu[9]*tmpObjS[43] + tmpFu[12]*tmpObjS[56] + tmpFu[15]*tmpObjS[69] + tmpFu[18]*tmpObjS[82] + tmpFu[21]*tmpObjS[95] + tmpFu[24]*tmpObjS[108] + tmpFu[27]*tmpObjS[121] + tmpFu[30]*tmpObjS[134] + tmpFu[33]*tmpObjS[147] + tmpFu[36]*tmpObjS[160];
tmpR2[5] = + tmpFu[0]*tmpObjS[5] + tmpFu[3]*tmpObjS[18] + tmpFu[6]*tmpObjS[31] + tmpFu[9]*tmpObjS[44] + tmpFu[12]*tmpObjS[57] + tmpFu[15]*tmpObjS[70] + tmpFu[18]*tmpObjS[83] + tmpFu[21]*tmpObjS[96] + tmpFu[24]*tmpObjS[109] + tmpFu[27]*tmpObjS[122] + tmpFu[30]*tmpObjS[135] + tmpFu[33]*tmpObjS[148] + tmpFu[36]*tmpObjS[161];
tmpR2[6] = + tmpFu[0]*tmpObjS[6] + tmpFu[3]*tmpObjS[19] + tmpFu[6]*tmpObjS[32] + tmpFu[9]*tmpObjS[45] + tmpFu[12]*tmpObjS[58] + tmpFu[15]*tmpObjS[71] + tmpFu[18]*tmpObjS[84] + tmpFu[21]*tmpObjS[97] + tmpFu[24]*tmpObjS[110] + tmpFu[27]*tmpObjS[123] + tmpFu[30]*tmpObjS[136] + tmpFu[33]*tmpObjS[149] + tmpFu[36]*tmpObjS[162];
tmpR2[7] = + tmpFu[0]*tmpObjS[7] + tmpFu[3]*tmpObjS[20] + tmpFu[6]*tmpObjS[33] + tmpFu[9]*tmpObjS[46] + tmpFu[12]*tmpObjS[59] + tmpFu[15]*tmpObjS[72] + tmpFu[18]*tmpObjS[85] + tmpFu[21]*tmpObjS[98] + tmpFu[24]*tmpObjS[111] + tmpFu[27]*tmpObjS[124] + tmpFu[30]*tmpObjS[137] + tmpFu[33]*tmpObjS[150] + tmpFu[36]*tmpObjS[163];
tmpR2[8] = + tmpFu[0]*tmpObjS[8] + tmpFu[3]*tmpObjS[21] + tmpFu[6]*tmpObjS[34] + tmpFu[9]*tmpObjS[47] + tmpFu[12]*tmpObjS[60] + tmpFu[15]*tmpObjS[73] + tmpFu[18]*tmpObjS[86] + tmpFu[21]*tmpObjS[99] + tmpFu[24]*tmpObjS[112] + tmpFu[27]*tmpObjS[125] + tmpFu[30]*tmpObjS[138] + tmpFu[33]*tmpObjS[151] + tmpFu[36]*tmpObjS[164];
tmpR2[9] = + tmpFu[0]*tmpObjS[9] + tmpFu[3]*tmpObjS[22] + tmpFu[6]*tmpObjS[35] + tmpFu[9]*tmpObjS[48] + tmpFu[12]*tmpObjS[61] + tmpFu[15]*tmpObjS[74] + tmpFu[18]*tmpObjS[87] + tmpFu[21]*tmpObjS[100] + tmpFu[24]*tmpObjS[113] + tmpFu[27]*tmpObjS[126] + tmpFu[30]*tmpObjS[139] + tmpFu[33]*tmpObjS[152] + tmpFu[36]*tmpObjS[165];
tmpR2[10] = + tmpFu[0]*tmpObjS[10] + tmpFu[3]*tmpObjS[23] + tmpFu[6]*tmpObjS[36] + tmpFu[9]*tmpObjS[49] + tmpFu[12]*tmpObjS[62] + tmpFu[15]*tmpObjS[75] + tmpFu[18]*tmpObjS[88] + tmpFu[21]*tmpObjS[101] + tmpFu[24]*tmpObjS[114] + tmpFu[27]*tmpObjS[127] + tmpFu[30]*tmpObjS[140] + tmpFu[33]*tmpObjS[153] + tmpFu[36]*tmpObjS[166];
tmpR2[11] = + tmpFu[0]*tmpObjS[11] + tmpFu[3]*tmpObjS[24] + tmpFu[6]*tmpObjS[37] + tmpFu[9]*tmpObjS[50] + tmpFu[12]*tmpObjS[63] + tmpFu[15]*tmpObjS[76] + tmpFu[18]*tmpObjS[89] + tmpFu[21]*tmpObjS[102] + tmpFu[24]*tmpObjS[115] + tmpFu[27]*tmpObjS[128] + tmpFu[30]*tmpObjS[141] + tmpFu[33]*tmpObjS[154] + tmpFu[36]*tmpObjS[167];
tmpR2[12] = + tmpFu[0]*tmpObjS[12] + tmpFu[3]*tmpObjS[25] + tmpFu[6]*tmpObjS[38] + tmpFu[9]*tmpObjS[51] + tmpFu[12]*tmpObjS[64] + tmpFu[15]*tmpObjS[77] + tmpFu[18]*tmpObjS[90] + tmpFu[21]*tmpObjS[103] + tmpFu[24]*tmpObjS[116] + tmpFu[27]*tmpObjS[129] + tmpFu[30]*tmpObjS[142] + tmpFu[33]*tmpObjS[155] + tmpFu[36]*tmpObjS[168];
tmpR2[13] = + tmpFu[1]*tmpObjS[0] + tmpFu[4]*tmpObjS[13] + tmpFu[7]*tmpObjS[26] + tmpFu[10]*tmpObjS[39] + tmpFu[13]*tmpObjS[52] + tmpFu[16]*tmpObjS[65] + tmpFu[19]*tmpObjS[78] + tmpFu[22]*tmpObjS[91] + tmpFu[25]*tmpObjS[104] + tmpFu[28]*tmpObjS[117] + tmpFu[31]*tmpObjS[130] + tmpFu[34]*tmpObjS[143] + tmpFu[37]*tmpObjS[156];
tmpR2[14] = + tmpFu[1]*tmpObjS[1] + tmpFu[4]*tmpObjS[14] + tmpFu[7]*tmpObjS[27] + tmpFu[10]*tmpObjS[40] + tmpFu[13]*tmpObjS[53] + tmpFu[16]*tmpObjS[66] + tmpFu[19]*tmpObjS[79] + tmpFu[22]*tmpObjS[92] + tmpFu[25]*tmpObjS[105] + tmpFu[28]*tmpObjS[118] + tmpFu[31]*tmpObjS[131] + tmpFu[34]*tmpObjS[144] + tmpFu[37]*tmpObjS[157];
tmpR2[15] = + tmpFu[1]*tmpObjS[2] + tmpFu[4]*tmpObjS[15] + tmpFu[7]*tmpObjS[28] + tmpFu[10]*tmpObjS[41] + tmpFu[13]*tmpObjS[54] + tmpFu[16]*tmpObjS[67] + tmpFu[19]*tmpObjS[80] + tmpFu[22]*tmpObjS[93] + tmpFu[25]*tmpObjS[106] + tmpFu[28]*tmpObjS[119] + tmpFu[31]*tmpObjS[132] + tmpFu[34]*tmpObjS[145] + tmpFu[37]*tmpObjS[158];
tmpR2[16] = + tmpFu[1]*tmpObjS[3] + tmpFu[4]*tmpObjS[16] + tmpFu[7]*tmpObjS[29] + tmpFu[10]*tmpObjS[42] + tmpFu[13]*tmpObjS[55] + tmpFu[16]*tmpObjS[68] + tmpFu[19]*tmpObjS[81] + tmpFu[22]*tmpObjS[94] + tmpFu[25]*tmpObjS[107] + tmpFu[28]*tmpObjS[120] + tmpFu[31]*tmpObjS[133] + tmpFu[34]*tmpObjS[146] + tmpFu[37]*tmpObjS[159];
tmpR2[17] = + tmpFu[1]*tmpObjS[4] + tmpFu[4]*tmpObjS[17] + tmpFu[7]*tmpObjS[30] + tmpFu[10]*tmpObjS[43] + tmpFu[13]*tmpObjS[56] + tmpFu[16]*tmpObjS[69] + tmpFu[19]*tmpObjS[82] + tmpFu[22]*tmpObjS[95] + tmpFu[25]*tmpObjS[108] + tmpFu[28]*tmpObjS[121] + tmpFu[31]*tmpObjS[134] + tmpFu[34]*tmpObjS[147] + tmpFu[37]*tmpObjS[160];
tmpR2[18] = + tmpFu[1]*tmpObjS[5] + tmpFu[4]*tmpObjS[18] + tmpFu[7]*tmpObjS[31] + tmpFu[10]*tmpObjS[44] + tmpFu[13]*tmpObjS[57] + tmpFu[16]*tmpObjS[70] + tmpFu[19]*tmpObjS[83] + tmpFu[22]*tmpObjS[96] + tmpFu[25]*tmpObjS[109] + tmpFu[28]*tmpObjS[122] + tmpFu[31]*tmpObjS[135] + tmpFu[34]*tmpObjS[148] + tmpFu[37]*tmpObjS[161];
tmpR2[19] = + tmpFu[1]*tmpObjS[6] + tmpFu[4]*tmpObjS[19] + tmpFu[7]*tmpObjS[32] + tmpFu[10]*tmpObjS[45] + tmpFu[13]*tmpObjS[58] + tmpFu[16]*tmpObjS[71] + tmpFu[19]*tmpObjS[84] + tmpFu[22]*tmpObjS[97] + tmpFu[25]*tmpObjS[110] + tmpFu[28]*tmpObjS[123] + tmpFu[31]*tmpObjS[136] + tmpFu[34]*tmpObjS[149] + tmpFu[37]*tmpObjS[162];
tmpR2[20] = + tmpFu[1]*tmpObjS[7] + tmpFu[4]*tmpObjS[20] + tmpFu[7]*tmpObjS[33] + tmpFu[10]*tmpObjS[46] + tmpFu[13]*tmpObjS[59] + tmpFu[16]*tmpObjS[72] + tmpFu[19]*tmpObjS[85] + tmpFu[22]*tmpObjS[98] + tmpFu[25]*tmpObjS[111] + tmpFu[28]*tmpObjS[124] + tmpFu[31]*tmpObjS[137] + tmpFu[34]*tmpObjS[150] + tmpFu[37]*tmpObjS[163];
tmpR2[21] = + tmpFu[1]*tmpObjS[8] + tmpFu[4]*tmpObjS[21] + tmpFu[7]*tmpObjS[34] + tmpFu[10]*tmpObjS[47] + tmpFu[13]*tmpObjS[60] + tmpFu[16]*tmpObjS[73] + tmpFu[19]*tmpObjS[86] + tmpFu[22]*tmpObjS[99] + tmpFu[25]*tmpObjS[112] + tmpFu[28]*tmpObjS[125] + tmpFu[31]*tmpObjS[138] + tmpFu[34]*tmpObjS[151] + tmpFu[37]*tmpObjS[164];
tmpR2[22] = + tmpFu[1]*tmpObjS[9] + tmpFu[4]*tmpObjS[22] + tmpFu[7]*tmpObjS[35] + tmpFu[10]*tmpObjS[48] + tmpFu[13]*tmpObjS[61] + tmpFu[16]*tmpObjS[74] + tmpFu[19]*tmpObjS[87] + tmpFu[22]*tmpObjS[100] + tmpFu[25]*tmpObjS[113] + tmpFu[28]*tmpObjS[126] + tmpFu[31]*tmpObjS[139] + tmpFu[34]*tmpObjS[152] + tmpFu[37]*tmpObjS[165];
tmpR2[23] = + tmpFu[1]*tmpObjS[10] + tmpFu[4]*tmpObjS[23] + tmpFu[7]*tmpObjS[36] + tmpFu[10]*tmpObjS[49] + tmpFu[13]*tmpObjS[62] + tmpFu[16]*tmpObjS[75] + tmpFu[19]*tmpObjS[88] + tmpFu[22]*tmpObjS[101] + tmpFu[25]*tmpObjS[114] + tmpFu[28]*tmpObjS[127] + tmpFu[31]*tmpObjS[140] + tmpFu[34]*tmpObjS[153] + tmpFu[37]*tmpObjS[166];
tmpR2[24] = + tmpFu[1]*tmpObjS[11] + tmpFu[4]*tmpObjS[24] + tmpFu[7]*tmpObjS[37] + tmpFu[10]*tmpObjS[50] + tmpFu[13]*tmpObjS[63] + tmpFu[16]*tmpObjS[76] + tmpFu[19]*tmpObjS[89] + tmpFu[22]*tmpObjS[102] + tmpFu[25]*tmpObjS[115] + tmpFu[28]*tmpObjS[128] + tmpFu[31]*tmpObjS[141] + tmpFu[34]*tmpObjS[154] + tmpFu[37]*tmpObjS[167];
tmpR2[25] = + tmpFu[1]*tmpObjS[12] + tmpFu[4]*tmpObjS[25] + tmpFu[7]*tmpObjS[38] + tmpFu[10]*tmpObjS[51] + tmpFu[13]*tmpObjS[64] + tmpFu[16]*tmpObjS[77] + tmpFu[19]*tmpObjS[90] + tmpFu[22]*tmpObjS[103] + tmpFu[25]*tmpObjS[116] + tmpFu[28]*tmpObjS[129] + tmpFu[31]*tmpObjS[142] + tmpFu[34]*tmpObjS[155] + tmpFu[37]*tmpObjS[168];
tmpR2[26] = + tmpFu[2]*tmpObjS[0] + tmpFu[5]*tmpObjS[13] + tmpFu[8]*tmpObjS[26] + tmpFu[11]*tmpObjS[39] + tmpFu[14]*tmpObjS[52] + tmpFu[17]*tmpObjS[65] + tmpFu[20]*tmpObjS[78] + tmpFu[23]*tmpObjS[91] + tmpFu[26]*tmpObjS[104] + tmpFu[29]*tmpObjS[117] + tmpFu[32]*tmpObjS[130] + tmpFu[35]*tmpObjS[143] + tmpFu[38]*tmpObjS[156];
tmpR2[27] = + tmpFu[2]*tmpObjS[1] + tmpFu[5]*tmpObjS[14] + tmpFu[8]*tmpObjS[27] + tmpFu[11]*tmpObjS[40] + tmpFu[14]*tmpObjS[53] + tmpFu[17]*tmpObjS[66] + tmpFu[20]*tmpObjS[79] + tmpFu[23]*tmpObjS[92] + tmpFu[26]*tmpObjS[105] + tmpFu[29]*tmpObjS[118] + tmpFu[32]*tmpObjS[131] + tmpFu[35]*tmpObjS[144] + tmpFu[38]*tmpObjS[157];
tmpR2[28] = + tmpFu[2]*tmpObjS[2] + tmpFu[5]*tmpObjS[15] + tmpFu[8]*tmpObjS[28] + tmpFu[11]*tmpObjS[41] + tmpFu[14]*tmpObjS[54] + tmpFu[17]*tmpObjS[67] + tmpFu[20]*tmpObjS[80] + tmpFu[23]*tmpObjS[93] + tmpFu[26]*tmpObjS[106] + tmpFu[29]*tmpObjS[119] + tmpFu[32]*tmpObjS[132] + tmpFu[35]*tmpObjS[145] + tmpFu[38]*tmpObjS[158];
tmpR2[29] = + tmpFu[2]*tmpObjS[3] + tmpFu[5]*tmpObjS[16] + tmpFu[8]*tmpObjS[29] + tmpFu[11]*tmpObjS[42] + tmpFu[14]*tmpObjS[55] + tmpFu[17]*tmpObjS[68] + tmpFu[20]*tmpObjS[81] + tmpFu[23]*tmpObjS[94] + tmpFu[26]*tmpObjS[107] + tmpFu[29]*tmpObjS[120] + tmpFu[32]*tmpObjS[133] + tmpFu[35]*tmpObjS[146] + tmpFu[38]*tmpObjS[159];
tmpR2[30] = + tmpFu[2]*tmpObjS[4] + tmpFu[5]*tmpObjS[17] + tmpFu[8]*tmpObjS[30] + tmpFu[11]*tmpObjS[43] + tmpFu[14]*tmpObjS[56] + tmpFu[17]*tmpObjS[69] + tmpFu[20]*tmpObjS[82] + tmpFu[23]*tmpObjS[95] + tmpFu[26]*tmpObjS[108] + tmpFu[29]*tmpObjS[121] + tmpFu[32]*tmpObjS[134] + tmpFu[35]*tmpObjS[147] + tmpFu[38]*tmpObjS[160];
tmpR2[31] = + tmpFu[2]*tmpObjS[5] + tmpFu[5]*tmpObjS[18] + tmpFu[8]*tmpObjS[31] + tmpFu[11]*tmpObjS[44] + tmpFu[14]*tmpObjS[57] + tmpFu[17]*tmpObjS[70] + tmpFu[20]*tmpObjS[83] + tmpFu[23]*tmpObjS[96] + tmpFu[26]*tmpObjS[109] + tmpFu[29]*tmpObjS[122] + tmpFu[32]*tmpObjS[135] + tmpFu[35]*tmpObjS[148] + tmpFu[38]*tmpObjS[161];
tmpR2[32] = + tmpFu[2]*tmpObjS[6] + tmpFu[5]*tmpObjS[19] + tmpFu[8]*tmpObjS[32] + tmpFu[11]*tmpObjS[45] + tmpFu[14]*tmpObjS[58] + tmpFu[17]*tmpObjS[71] + tmpFu[20]*tmpObjS[84] + tmpFu[23]*tmpObjS[97] + tmpFu[26]*tmpObjS[110] + tmpFu[29]*tmpObjS[123] + tmpFu[32]*tmpObjS[136] + tmpFu[35]*tmpObjS[149] + tmpFu[38]*tmpObjS[162];
tmpR2[33] = + tmpFu[2]*tmpObjS[7] + tmpFu[5]*tmpObjS[20] + tmpFu[8]*tmpObjS[33] + tmpFu[11]*tmpObjS[46] + tmpFu[14]*tmpObjS[59] + tmpFu[17]*tmpObjS[72] + tmpFu[20]*tmpObjS[85] + tmpFu[23]*tmpObjS[98] + tmpFu[26]*tmpObjS[111] + tmpFu[29]*tmpObjS[124] + tmpFu[32]*tmpObjS[137] + tmpFu[35]*tmpObjS[150] + tmpFu[38]*tmpObjS[163];
tmpR2[34] = + tmpFu[2]*tmpObjS[8] + tmpFu[5]*tmpObjS[21] + tmpFu[8]*tmpObjS[34] + tmpFu[11]*tmpObjS[47] + tmpFu[14]*tmpObjS[60] + tmpFu[17]*tmpObjS[73] + tmpFu[20]*tmpObjS[86] + tmpFu[23]*tmpObjS[99] + tmpFu[26]*tmpObjS[112] + tmpFu[29]*tmpObjS[125] + tmpFu[32]*tmpObjS[138] + tmpFu[35]*tmpObjS[151] + tmpFu[38]*tmpObjS[164];
tmpR2[35] = + tmpFu[2]*tmpObjS[9] + tmpFu[5]*tmpObjS[22] + tmpFu[8]*tmpObjS[35] + tmpFu[11]*tmpObjS[48] + tmpFu[14]*tmpObjS[61] + tmpFu[17]*tmpObjS[74] + tmpFu[20]*tmpObjS[87] + tmpFu[23]*tmpObjS[100] + tmpFu[26]*tmpObjS[113] + tmpFu[29]*tmpObjS[126] + tmpFu[32]*tmpObjS[139] + tmpFu[35]*tmpObjS[152] + tmpFu[38]*tmpObjS[165];
tmpR2[36] = + tmpFu[2]*tmpObjS[10] + tmpFu[5]*tmpObjS[23] + tmpFu[8]*tmpObjS[36] + tmpFu[11]*tmpObjS[49] + tmpFu[14]*tmpObjS[62] + tmpFu[17]*tmpObjS[75] + tmpFu[20]*tmpObjS[88] + tmpFu[23]*tmpObjS[101] + tmpFu[26]*tmpObjS[114] + tmpFu[29]*tmpObjS[127] + tmpFu[32]*tmpObjS[140] + tmpFu[35]*tmpObjS[153] + tmpFu[38]*tmpObjS[166];
tmpR2[37] = + tmpFu[2]*tmpObjS[11] + tmpFu[5]*tmpObjS[24] + tmpFu[8]*tmpObjS[37] + tmpFu[11]*tmpObjS[50] + tmpFu[14]*tmpObjS[63] + tmpFu[17]*tmpObjS[76] + tmpFu[20]*tmpObjS[89] + tmpFu[23]*tmpObjS[102] + tmpFu[26]*tmpObjS[115] + tmpFu[29]*tmpObjS[128] + tmpFu[32]*tmpObjS[141] + tmpFu[35]*tmpObjS[154] + tmpFu[38]*tmpObjS[167];
tmpR2[38] = + tmpFu[2]*tmpObjS[12] + tmpFu[5]*tmpObjS[25] + tmpFu[8]*tmpObjS[38] + tmpFu[11]*tmpObjS[51] + tmpFu[14]*tmpObjS[64] + tmpFu[17]*tmpObjS[77] + tmpFu[20]*tmpObjS[90] + tmpFu[23]*tmpObjS[103] + tmpFu[26]*tmpObjS[116] + tmpFu[29]*tmpObjS[129] + tmpFu[32]*tmpObjS[142] + tmpFu[35]*tmpObjS[155] + tmpFu[38]*tmpObjS[168];
tmpR1[0] = + tmpR2[0]*tmpFu[0] + tmpR2[1]*tmpFu[3] + tmpR2[2]*tmpFu[6] + tmpR2[3]*tmpFu[9] + tmpR2[4]*tmpFu[12] + tmpR2[5]*tmpFu[15] + tmpR2[6]*tmpFu[18] + tmpR2[7]*tmpFu[21] + tmpR2[8]*tmpFu[24] + tmpR2[9]*tmpFu[27] + tmpR2[10]*tmpFu[30] + tmpR2[11]*tmpFu[33] + tmpR2[12]*tmpFu[36];
tmpR1[1] = + tmpR2[0]*tmpFu[1] + tmpR2[1]*tmpFu[4] + tmpR2[2]*tmpFu[7] + tmpR2[3]*tmpFu[10] + tmpR2[4]*tmpFu[13] + tmpR2[5]*tmpFu[16] + tmpR2[6]*tmpFu[19] + tmpR2[7]*tmpFu[22] + tmpR2[8]*tmpFu[25] + tmpR2[9]*tmpFu[28] + tmpR2[10]*tmpFu[31] + tmpR2[11]*tmpFu[34] + tmpR2[12]*tmpFu[37];
tmpR1[2] = + tmpR2[0]*tmpFu[2] + tmpR2[1]*tmpFu[5] + tmpR2[2]*tmpFu[8] + tmpR2[3]*tmpFu[11] + tmpR2[4]*tmpFu[14] + tmpR2[5]*tmpFu[17] + tmpR2[6]*tmpFu[20] + tmpR2[7]*tmpFu[23] + tmpR2[8]*tmpFu[26] + tmpR2[9]*tmpFu[29] + tmpR2[10]*tmpFu[32] + tmpR2[11]*tmpFu[35] + tmpR2[12]*tmpFu[38];
tmpR1[3] = + tmpR2[13]*tmpFu[0] + tmpR2[14]*tmpFu[3] + tmpR2[15]*tmpFu[6] + tmpR2[16]*tmpFu[9] + tmpR2[17]*tmpFu[12] + tmpR2[18]*tmpFu[15] + tmpR2[19]*tmpFu[18] + tmpR2[20]*tmpFu[21] + tmpR2[21]*tmpFu[24] + tmpR2[22]*tmpFu[27] + tmpR2[23]*tmpFu[30] + tmpR2[24]*tmpFu[33] + tmpR2[25]*tmpFu[36];
tmpR1[4] = + tmpR2[13]*tmpFu[1] + tmpR2[14]*tmpFu[4] + tmpR2[15]*tmpFu[7] + tmpR2[16]*tmpFu[10] + tmpR2[17]*tmpFu[13] + tmpR2[18]*tmpFu[16] + tmpR2[19]*tmpFu[19] + tmpR2[20]*tmpFu[22] + tmpR2[21]*tmpFu[25] + tmpR2[22]*tmpFu[28] + tmpR2[23]*tmpFu[31] + tmpR2[24]*tmpFu[34] + tmpR2[25]*tmpFu[37];
tmpR1[5] = + tmpR2[13]*tmpFu[2] + tmpR2[14]*tmpFu[5] + tmpR2[15]*tmpFu[8] + tmpR2[16]*tmpFu[11] + tmpR2[17]*tmpFu[14] + tmpR2[18]*tmpFu[17] + tmpR2[19]*tmpFu[20] + tmpR2[20]*tmpFu[23] + tmpR2[21]*tmpFu[26] + tmpR2[22]*tmpFu[29] + tmpR2[23]*tmpFu[32] + tmpR2[24]*tmpFu[35] + tmpR2[25]*tmpFu[38];
tmpR1[6] = + tmpR2[26]*tmpFu[0] + tmpR2[27]*tmpFu[3] + tmpR2[28]*tmpFu[6] + tmpR2[29]*tmpFu[9] + tmpR2[30]*tmpFu[12] + tmpR2[31]*tmpFu[15] + tmpR2[32]*tmpFu[18] + tmpR2[33]*tmpFu[21] + tmpR2[34]*tmpFu[24] + tmpR2[35]*tmpFu[27] + tmpR2[36]*tmpFu[30] + tmpR2[37]*tmpFu[33] + tmpR2[38]*tmpFu[36];
tmpR1[7] = + tmpR2[26]*tmpFu[1] + tmpR2[27]*tmpFu[4] + tmpR2[28]*tmpFu[7] + tmpR2[29]*tmpFu[10] + tmpR2[30]*tmpFu[13] + tmpR2[31]*tmpFu[16] + tmpR2[32]*tmpFu[19] + tmpR2[33]*tmpFu[22] + tmpR2[34]*tmpFu[25] + tmpR2[35]*tmpFu[28] + tmpR2[36]*tmpFu[31] + tmpR2[37]*tmpFu[34] + tmpR2[38]*tmpFu[37];
tmpR1[8] = + tmpR2[26]*tmpFu[2] + tmpR2[27]*tmpFu[5] + tmpR2[28]*tmpFu[8] + tmpR2[29]*tmpFu[11] + tmpR2[30]*tmpFu[14] + tmpR2[31]*tmpFu[17] + tmpR2[32]*tmpFu[20] + tmpR2[33]*tmpFu[23] + tmpR2[34]*tmpFu[26] + tmpR2[35]*tmpFu[29] + tmpR2[36]*tmpFu[32] + tmpR2[37]*tmpFu[35] + tmpR2[38]*tmpFu[38];
}

void acado_setObjS1( real_t* const tmpFx, real_t* const tmpFu, real_t* const tmpObjS, real_t* const tmpS1 )
{
/** Matrix of size: 12 x 13 (row major format) */
real_t tmpS2[ 156 ];

tmpS2[0] = + tmpFx[0]*tmpObjS[0] + tmpFx[12]*tmpObjS[13] + tmpFx[24]*tmpObjS[26] + tmpFx[36]*tmpObjS[39] + tmpFx[48]*tmpObjS[52] + tmpFx[60]*tmpObjS[65] + tmpFx[72]*tmpObjS[78] + tmpFx[84]*tmpObjS[91] + tmpFx[96]*tmpObjS[104] + tmpFx[108]*tmpObjS[117] + tmpFx[120]*tmpObjS[130] + tmpFx[132]*tmpObjS[143] + tmpFx[144]*tmpObjS[156];
tmpS2[1] = + tmpFx[0]*tmpObjS[1] + tmpFx[12]*tmpObjS[14] + tmpFx[24]*tmpObjS[27] + tmpFx[36]*tmpObjS[40] + tmpFx[48]*tmpObjS[53] + tmpFx[60]*tmpObjS[66] + tmpFx[72]*tmpObjS[79] + tmpFx[84]*tmpObjS[92] + tmpFx[96]*tmpObjS[105] + tmpFx[108]*tmpObjS[118] + tmpFx[120]*tmpObjS[131] + tmpFx[132]*tmpObjS[144] + tmpFx[144]*tmpObjS[157];
tmpS2[2] = + tmpFx[0]*tmpObjS[2] + tmpFx[12]*tmpObjS[15] + tmpFx[24]*tmpObjS[28] + tmpFx[36]*tmpObjS[41] + tmpFx[48]*tmpObjS[54] + tmpFx[60]*tmpObjS[67] + tmpFx[72]*tmpObjS[80] + tmpFx[84]*tmpObjS[93] + tmpFx[96]*tmpObjS[106] + tmpFx[108]*tmpObjS[119] + tmpFx[120]*tmpObjS[132] + tmpFx[132]*tmpObjS[145] + tmpFx[144]*tmpObjS[158];
tmpS2[3] = + tmpFx[0]*tmpObjS[3] + tmpFx[12]*tmpObjS[16] + tmpFx[24]*tmpObjS[29] + tmpFx[36]*tmpObjS[42] + tmpFx[48]*tmpObjS[55] + tmpFx[60]*tmpObjS[68] + tmpFx[72]*tmpObjS[81] + tmpFx[84]*tmpObjS[94] + tmpFx[96]*tmpObjS[107] + tmpFx[108]*tmpObjS[120] + tmpFx[120]*tmpObjS[133] + tmpFx[132]*tmpObjS[146] + tmpFx[144]*tmpObjS[159];
tmpS2[4] = + tmpFx[0]*tmpObjS[4] + tmpFx[12]*tmpObjS[17] + tmpFx[24]*tmpObjS[30] + tmpFx[36]*tmpObjS[43] + tmpFx[48]*tmpObjS[56] + tmpFx[60]*tmpObjS[69] + tmpFx[72]*tmpObjS[82] + tmpFx[84]*tmpObjS[95] + tmpFx[96]*tmpObjS[108] + tmpFx[108]*tmpObjS[121] + tmpFx[120]*tmpObjS[134] + tmpFx[132]*tmpObjS[147] + tmpFx[144]*tmpObjS[160];
tmpS2[5] = + tmpFx[0]*tmpObjS[5] + tmpFx[12]*tmpObjS[18] + tmpFx[24]*tmpObjS[31] + tmpFx[36]*tmpObjS[44] + tmpFx[48]*tmpObjS[57] + tmpFx[60]*tmpObjS[70] + tmpFx[72]*tmpObjS[83] + tmpFx[84]*tmpObjS[96] + tmpFx[96]*tmpObjS[109] + tmpFx[108]*tmpObjS[122] + tmpFx[120]*tmpObjS[135] + tmpFx[132]*tmpObjS[148] + tmpFx[144]*tmpObjS[161];
tmpS2[6] = + tmpFx[0]*tmpObjS[6] + tmpFx[12]*tmpObjS[19] + tmpFx[24]*tmpObjS[32] + tmpFx[36]*tmpObjS[45] + tmpFx[48]*tmpObjS[58] + tmpFx[60]*tmpObjS[71] + tmpFx[72]*tmpObjS[84] + tmpFx[84]*tmpObjS[97] + tmpFx[96]*tmpObjS[110] + tmpFx[108]*tmpObjS[123] + tmpFx[120]*tmpObjS[136] + tmpFx[132]*tmpObjS[149] + tmpFx[144]*tmpObjS[162];
tmpS2[7] = + tmpFx[0]*tmpObjS[7] + tmpFx[12]*tmpObjS[20] + tmpFx[24]*tmpObjS[33] + tmpFx[36]*tmpObjS[46] + tmpFx[48]*tmpObjS[59] + tmpFx[60]*tmpObjS[72] + tmpFx[72]*tmpObjS[85] + tmpFx[84]*tmpObjS[98] + tmpFx[96]*tmpObjS[111] + tmpFx[108]*tmpObjS[124] + tmpFx[120]*tmpObjS[137] + tmpFx[132]*tmpObjS[150] + tmpFx[144]*tmpObjS[163];
tmpS2[8] = + tmpFx[0]*tmpObjS[8] + tmpFx[12]*tmpObjS[21] + tmpFx[24]*tmpObjS[34] + tmpFx[36]*tmpObjS[47] + tmpFx[48]*tmpObjS[60] + tmpFx[60]*tmpObjS[73] + tmpFx[72]*tmpObjS[86] + tmpFx[84]*tmpObjS[99] + tmpFx[96]*tmpObjS[112] + tmpFx[108]*tmpObjS[125] + tmpFx[120]*tmpObjS[138] + tmpFx[132]*tmpObjS[151] + tmpFx[144]*tmpObjS[164];
tmpS2[9] = + tmpFx[0]*tmpObjS[9] + tmpFx[12]*tmpObjS[22] + tmpFx[24]*tmpObjS[35] + tmpFx[36]*tmpObjS[48] + tmpFx[48]*tmpObjS[61] + tmpFx[60]*tmpObjS[74] + tmpFx[72]*tmpObjS[87] + tmpFx[84]*tmpObjS[100] + tmpFx[96]*tmpObjS[113] + tmpFx[108]*tmpObjS[126] + tmpFx[120]*tmpObjS[139] + tmpFx[132]*tmpObjS[152] + tmpFx[144]*tmpObjS[165];
tmpS2[10] = + tmpFx[0]*tmpObjS[10] + tmpFx[12]*tmpObjS[23] + tmpFx[24]*tmpObjS[36] + tmpFx[36]*tmpObjS[49] + tmpFx[48]*tmpObjS[62] + tmpFx[60]*tmpObjS[75] + tmpFx[72]*tmpObjS[88] + tmpFx[84]*tmpObjS[101] + tmpFx[96]*tmpObjS[114] + tmpFx[108]*tmpObjS[127] + tmpFx[120]*tmpObjS[140] + tmpFx[132]*tmpObjS[153] + tmpFx[144]*tmpObjS[166];
tmpS2[11] = + tmpFx[0]*tmpObjS[11] + tmpFx[12]*tmpObjS[24] + tmpFx[24]*tmpObjS[37] + tmpFx[36]*tmpObjS[50] + tmpFx[48]*tmpObjS[63] + tmpFx[60]*tmpObjS[76] + tmpFx[72]*tmpObjS[89] + tmpFx[84]*tmpObjS[102] + tmpFx[96]*tmpObjS[115] + tmpFx[108]*tmpObjS[128] + tmpFx[120]*tmpObjS[141] + tmpFx[132]*tmpObjS[154] + tmpFx[144]*tmpObjS[167];
tmpS2[12] = + tmpFx[0]*tmpObjS[12] + tmpFx[12]*tmpObjS[25] + tmpFx[24]*tmpObjS[38] + tmpFx[36]*tmpObjS[51] + tmpFx[48]*tmpObjS[64] + tmpFx[60]*tmpObjS[77] + tmpFx[72]*tmpObjS[90] + tmpFx[84]*tmpObjS[103] + tmpFx[96]*tmpObjS[116] + tmpFx[108]*tmpObjS[129] + tmpFx[120]*tmpObjS[142] + tmpFx[132]*tmpObjS[155] + tmpFx[144]*tmpObjS[168];
tmpS2[13] = + tmpFx[1]*tmpObjS[0] + tmpFx[13]*tmpObjS[13] + tmpFx[25]*tmpObjS[26] + tmpFx[37]*tmpObjS[39] + tmpFx[49]*tmpObjS[52] + tmpFx[61]*tmpObjS[65] + tmpFx[73]*tmpObjS[78] + tmpFx[85]*tmpObjS[91] + tmpFx[97]*tmpObjS[104] + tmpFx[109]*tmpObjS[117] + tmpFx[121]*tmpObjS[130] + tmpFx[133]*tmpObjS[143] + tmpFx[145]*tmpObjS[156];
tmpS2[14] = + tmpFx[1]*tmpObjS[1] + tmpFx[13]*tmpObjS[14] + tmpFx[25]*tmpObjS[27] + tmpFx[37]*tmpObjS[40] + tmpFx[49]*tmpObjS[53] + tmpFx[61]*tmpObjS[66] + tmpFx[73]*tmpObjS[79] + tmpFx[85]*tmpObjS[92] + tmpFx[97]*tmpObjS[105] + tmpFx[109]*tmpObjS[118] + tmpFx[121]*tmpObjS[131] + tmpFx[133]*tmpObjS[144] + tmpFx[145]*tmpObjS[157];
tmpS2[15] = + tmpFx[1]*tmpObjS[2] + tmpFx[13]*tmpObjS[15] + tmpFx[25]*tmpObjS[28] + tmpFx[37]*tmpObjS[41] + tmpFx[49]*tmpObjS[54] + tmpFx[61]*tmpObjS[67] + tmpFx[73]*tmpObjS[80] + tmpFx[85]*tmpObjS[93] + tmpFx[97]*tmpObjS[106] + tmpFx[109]*tmpObjS[119] + tmpFx[121]*tmpObjS[132] + tmpFx[133]*tmpObjS[145] + tmpFx[145]*tmpObjS[158];
tmpS2[16] = + tmpFx[1]*tmpObjS[3] + tmpFx[13]*tmpObjS[16] + tmpFx[25]*tmpObjS[29] + tmpFx[37]*tmpObjS[42] + tmpFx[49]*tmpObjS[55] + tmpFx[61]*tmpObjS[68] + tmpFx[73]*tmpObjS[81] + tmpFx[85]*tmpObjS[94] + tmpFx[97]*tmpObjS[107] + tmpFx[109]*tmpObjS[120] + tmpFx[121]*tmpObjS[133] + tmpFx[133]*tmpObjS[146] + tmpFx[145]*tmpObjS[159];
tmpS2[17] = + tmpFx[1]*tmpObjS[4] + tmpFx[13]*tmpObjS[17] + tmpFx[25]*tmpObjS[30] + tmpFx[37]*tmpObjS[43] + tmpFx[49]*tmpObjS[56] + tmpFx[61]*tmpObjS[69] + tmpFx[73]*tmpObjS[82] + tmpFx[85]*tmpObjS[95] + tmpFx[97]*tmpObjS[108] + tmpFx[109]*tmpObjS[121] + tmpFx[121]*tmpObjS[134] + tmpFx[133]*tmpObjS[147] + tmpFx[145]*tmpObjS[160];
tmpS2[18] = + tmpFx[1]*tmpObjS[5] + tmpFx[13]*tmpObjS[18] + tmpFx[25]*tmpObjS[31] + tmpFx[37]*tmpObjS[44] + tmpFx[49]*tmpObjS[57] + tmpFx[61]*tmpObjS[70] + tmpFx[73]*tmpObjS[83] + tmpFx[85]*tmpObjS[96] + tmpFx[97]*tmpObjS[109] + tmpFx[109]*tmpObjS[122] + tmpFx[121]*tmpObjS[135] + tmpFx[133]*tmpObjS[148] + tmpFx[145]*tmpObjS[161];
tmpS2[19] = + tmpFx[1]*tmpObjS[6] + tmpFx[13]*tmpObjS[19] + tmpFx[25]*tmpObjS[32] + tmpFx[37]*tmpObjS[45] + tmpFx[49]*tmpObjS[58] + tmpFx[61]*tmpObjS[71] + tmpFx[73]*tmpObjS[84] + tmpFx[85]*tmpObjS[97] + tmpFx[97]*tmpObjS[110] + tmpFx[109]*tmpObjS[123] + tmpFx[121]*tmpObjS[136] + tmpFx[133]*tmpObjS[149] + tmpFx[145]*tmpObjS[162];
tmpS2[20] = + tmpFx[1]*tmpObjS[7] + tmpFx[13]*tmpObjS[20] + tmpFx[25]*tmpObjS[33] + tmpFx[37]*tmpObjS[46] + tmpFx[49]*tmpObjS[59] + tmpFx[61]*tmpObjS[72] + tmpFx[73]*tmpObjS[85] + tmpFx[85]*tmpObjS[98] + tmpFx[97]*tmpObjS[111] + tmpFx[109]*tmpObjS[124] + tmpFx[121]*tmpObjS[137] + tmpFx[133]*tmpObjS[150] + tmpFx[145]*tmpObjS[163];
tmpS2[21] = + tmpFx[1]*tmpObjS[8] + tmpFx[13]*tmpObjS[21] + tmpFx[25]*tmpObjS[34] + tmpFx[37]*tmpObjS[47] + tmpFx[49]*tmpObjS[60] + tmpFx[61]*tmpObjS[73] + tmpFx[73]*tmpObjS[86] + tmpFx[85]*tmpObjS[99] + tmpFx[97]*tmpObjS[112] + tmpFx[109]*tmpObjS[125] + tmpFx[121]*tmpObjS[138] + tmpFx[133]*tmpObjS[151] + tmpFx[145]*tmpObjS[164];
tmpS2[22] = + tmpFx[1]*tmpObjS[9] + tmpFx[13]*tmpObjS[22] + tmpFx[25]*tmpObjS[35] + tmpFx[37]*tmpObjS[48] + tmpFx[49]*tmpObjS[61] + tmpFx[61]*tmpObjS[74] + tmpFx[73]*tmpObjS[87] + tmpFx[85]*tmpObjS[100] + tmpFx[97]*tmpObjS[113] + tmpFx[109]*tmpObjS[126] + tmpFx[121]*tmpObjS[139] + tmpFx[133]*tmpObjS[152] + tmpFx[145]*tmpObjS[165];
tmpS2[23] = + tmpFx[1]*tmpObjS[10] + tmpFx[13]*tmpObjS[23] + tmpFx[25]*tmpObjS[36] + tmpFx[37]*tmpObjS[49] + tmpFx[49]*tmpObjS[62] + tmpFx[61]*tmpObjS[75] + tmpFx[73]*tmpObjS[88] + tmpFx[85]*tmpObjS[101] + tmpFx[97]*tmpObjS[114] + tmpFx[109]*tmpObjS[127] + tmpFx[121]*tmpObjS[140] + tmpFx[133]*tmpObjS[153] + tmpFx[145]*tmpObjS[166];
tmpS2[24] = + tmpFx[1]*tmpObjS[11] + tmpFx[13]*tmpObjS[24] + tmpFx[25]*tmpObjS[37] + tmpFx[37]*tmpObjS[50] + tmpFx[49]*tmpObjS[63] + tmpFx[61]*tmpObjS[76] + tmpFx[73]*tmpObjS[89] + tmpFx[85]*tmpObjS[102] + tmpFx[97]*tmpObjS[115] + tmpFx[109]*tmpObjS[128] + tmpFx[121]*tmpObjS[141] + tmpFx[133]*tmpObjS[154] + tmpFx[145]*tmpObjS[167];
tmpS2[25] = + tmpFx[1]*tmpObjS[12] + tmpFx[13]*tmpObjS[25] + tmpFx[25]*tmpObjS[38] + tmpFx[37]*tmpObjS[51] + tmpFx[49]*tmpObjS[64] + tmpFx[61]*tmpObjS[77] + tmpFx[73]*tmpObjS[90] + tmpFx[85]*tmpObjS[103] + tmpFx[97]*tmpObjS[116] + tmpFx[109]*tmpObjS[129] + tmpFx[121]*tmpObjS[142] + tmpFx[133]*tmpObjS[155] + tmpFx[145]*tmpObjS[168];
tmpS2[26] = + tmpFx[2]*tmpObjS[0] + tmpFx[14]*tmpObjS[13] + tmpFx[26]*tmpObjS[26] + tmpFx[38]*tmpObjS[39] + tmpFx[50]*tmpObjS[52] + tmpFx[62]*tmpObjS[65] + tmpFx[74]*tmpObjS[78] + tmpFx[86]*tmpObjS[91] + tmpFx[98]*tmpObjS[104] + tmpFx[110]*tmpObjS[117] + tmpFx[122]*tmpObjS[130] + tmpFx[134]*tmpObjS[143] + tmpFx[146]*tmpObjS[156];
tmpS2[27] = + tmpFx[2]*tmpObjS[1] + tmpFx[14]*tmpObjS[14] + tmpFx[26]*tmpObjS[27] + tmpFx[38]*tmpObjS[40] + tmpFx[50]*tmpObjS[53] + tmpFx[62]*tmpObjS[66] + tmpFx[74]*tmpObjS[79] + tmpFx[86]*tmpObjS[92] + tmpFx[98]*tmpObjS[105] + tmpFx[110]*tmpObjS[118] + tmpFx[122]*tmpObjS[131] + tmpFx[134]*tmpObjS[144] + tmpFx[146]*tmpObjS[157];
tmpS2[28] = + tmpFx[2]*tmpObjS[2] + tmpFx[14]*tmpObjS[15] + tmpFx[26]*tmpObjS[28] + tmpFx[38]*tmpObjS[41] + tmpFx[50]*tmpObjS[54] + tmpFx[62]*tmpObjS[67] + tmpFx[74]*tmpObjS[80] + tmpFx[86]*tmpObjS[93] + tmpFx[98]*tmpObjS[106] + tmpFx[110]*tmpObjS[119] + tmpFx[122]*tmpObjS[132] + tmpFx[134]*tmpObjS[145] + tmpFx[146]*tmpObjS[158];
tmpS2[29] = + tmpFx[2]*tmpObjS[3] + tmpFx[14]*tmpObjS[16] + tmpFx[26]*tmpObjS[29] + tmpFx[38]*tmpObjS[42] + tmpFx[50]*tmpObjS[55] + tmpFx[62]*tmpObjS[68] + tmpFx[74]*tmpObjS[81] + tmpFx[86]*tmpObjS[94] + tmpFx[98]*tmpObjS[107] + tmpFx[110]*tmpObjS[120] + tmpFx[122]*tmpObjS[133] + tmpFx[134]*tmpObjS[146] + tmpFx[146]*tmpObjS[159];
tmpS2[30] = + tmpFx[2]*tmpObjS[4] + tmpFx[14]*tmpObjS[17] + tmpFx[26]*tmpObjS[30] + tmpFx[38]*tmpObjS[43] + tmpFx[50]*tmpObjS[56] + tmpFx[62]*tmpObjS[69] + tmpFx[74]*tmpObjS[82] + tmpFx[86]*tmpObjS[95] + tmpFx[98]*tmpObjS[108] + tmpFx[110]*tmpObjS[121] + tmpFx[122]*tmpObjS[134] + tmpFx[134]*tmpObjS[147] + tmpFx[146]*tmpObjS[160];
tmpS2[31] = + tmpFx[2]*tmpObjS[5] + tmpFx[14]*tmpObjS[18] + tmpFx[26]*tmpObjS[31] + tmpFx[38]*tmpObjS[44] + tmpFx[50]*tmpObjS[57] + tmpFx[62]*tmpObjS[70] + tmpFx[74]*tmpObjS[83] + tmpFx[86]*tmpObjS[96] + tmpFx[98]*tmpObjS[109] + tmpFx[110]*tmpObjS[122] + tmpFx[122]*tmpObjS[135] + tmpFx[134]*tmpObjS[148] + tmpFx[146]*tmpObjS[161];
tmpS2[32] = + tmpFx[2]*tmpObjS[6] + tmpFx[14]*tmpObjS[19] + tmpFx[26]*tmpObjS[32] + tmpFx[38]*tmpObjS[45] + tmpFx[50]*tmpObjS[58] + tmpFx[62]*tmpObjS[71] + tmpFx[74]*tmpObjS[84] + tmpFx[86]*tmpObjS[97] + tmpFx[98]*tmpObjS[110] + tmpFx[110]*tmpObjS[123] + tmpFx[122]*tmpObjS[136] + tmpFx[134]*tmpObjS[149] + tmpFx[146]*tmpObjS[162];
tmpS2[33] = + tmpFx[2]*tmpObjS[7] + tmpFx[14]*tmpObjS[20] + tmpFx[26]*tmpObjS[33] + tmpFx[38]*tmpObjS[46] + tmpFx[50]*tmpObjS[59] + tmpFx[62]*tmpObjS[72] + tmpFx[74]*tmpObjS[85] + tmpFx[86]*tmpObjS[98] + tmpFx[98]*tmpObjS[111] + tmpFx[110]*tmpObjS[124] + tmpFx[122]*tmpObjS[137] + tmpFx[134]*tmpObjS[150] + tmpFx[146]*tmpObjS[163];
tmpS2[34] = + tmpFx[2]*tmpObjS[8] + tmpFx[14]*tmpObjS[21] + tmpFx[26]*tmpObjS[34] + tmpFx[38]*tmpObjS[47] + tmpFx[50]*tmpObjS[60] + tmpFx[62]*tmpObjS[73] + tmpFx[74]*tmpObjS[86] + tmpFx[86]*tmpObjS[99] + tmpFx[98]*tmpObjS[112] + tmpFx[110]*tmpObjS[125] + tmpFx[122]*tmpObjS[138] + tmpFx[134]*tmpObjS[151] + tmpFx[146]*tmpObjS[164];
tmpS2[35] = + tmpFx[2]*tmpObjS[9] + tmpFx[14]*tmpObjS[22] + tmpFx[26]*tmpObjS[35] + tmpFx[38]*tmpObjS[48] + tmpFx[50]*tmpObjS[61] + tmpFx[62]*tmpObjS[74] + tmpFx[74]*tmpObjS[87] + tmpFx[86]*tmpObjS[100] + tmpFx[98]*tmpObjS[113] + tmpFx[110]*tmpObjS[126] + tmpFx[122]*tmpObjS[139] + tmpFx[134]*tmpObjS[152] + tmpFx[146]*tmpObjS[165];
tmpS2[36] = + tmpFx[2]*tmpObjS[10] + tmpFx[14]*tmpObjS[23] + tmpFx[26]*tmpObjS[36] + tmpFx[38]*tmpObjS[49] + tmpFx[50]*tmpObjS[62] + tmpFx[62]*tmpObjS[75] + tmpFx[74]*tmpObjS[88] + tmpFx[86]*tmpObjS[101] + tmpFx[98]*tmpObjS[114] + tmpFx[110]*tmpObjS[127] + tmpFx[122]*tmpObjS[140] + tmpFx[134]*tmpObjS[153] + tmpFx[146]*tmpObjS[166];
tmpS2[37] = + tmpFx[2]*tmpObjS[11] + tmpFx[14]*tmpObjS[24] + tmpFx[26]*tmpObjS[37] + tmpFx[38]*tmpObjS[50] + tmpFx[50]*tmpObjS[63] + tmpFx[62]*tmpObjS[76] + tmpFx[74]*tmpObjS[89] + tmpFx[86]*tmpObjS[102] + tmpFx[98]*tmpObjS[115] + tmpFx[110]*tmpObjS[128] + tmpFx[122]*tmpObjS[141] + tmpFx[134]*tmpObjS[154] + tmpFx[146]*tmpObjS[167];
tmpS2[38] = + tmpFx[2]*tmpObjS[12] + tmpFx[14]*tmpObjS[25] + tmpFx[26]*tmpObjS[38] + tmpFx[38]*tmpObjS[51] + tmpFx[50]*tmpObjS[64] + tmpFx[62]*tmpObjS[77] + tmpFx[74]*tmpObjS[90] + tmpFx[86]*tmpObjS[103] + tmpFx[98]*tmpObjS[116] + tmpFx[110]*tmpObjS[129] + tmpFx[122]*tmpObjS[142] + tmpFx[134]*tmpObjS[155] + tmpFx[146]*tmpObjS[168];
tmpS2[39] = + tmpFx[3]*tmpObjS[0] + tmpFx[15]*tmpObjS[13] + tmpFx[27]*tmpObjS[26] + tmpFx[39]*tmpObjS[39] + tmpFx[51]*tmpObjS[52] + tmpFx[63]*tmpObjS[65] + tmpFx[75]*tmpObjS[78] + tmpFx[87]*tmpObjS[91] + tmpFx[99]*tmpObjS[104] + tmpFx[111]*tmpObjS[117] + tmpFx[123]*tmpObjS[130] + tmpFx[135]*tmpObjS[143] + tmpFx[147]*tmpObjS[156];
tmpS2[40] = + tmpFx[3]*tmpObjS[1] + tmpFx[15]*tmpObjS[14] + tmpFx[27]*tmpObjS[27] + tmpFx[39]*tmpObjS[40] + tmpFx[51]*tmpObjS[53] + tmpFx[63]*tmpObjS[66] + tmpFx[75]*tmpObjS[79] + tmpFx[87]*tmpObjS[92] + tmpFx[99]*tmpObjS[105] + tmpFx[111]*tmpObjS[118] + tmpFx[123]*tmpObjS[131] + tmpFx[135]*tmpObjS[144] + tmpFx[147]*tmpObjS[157];
tmpS2[41] = + tmpFx[3]*tmpObjS[2] + tmpFx[15]*tmpObjS[15] + tmpFx[27]*tmpObjS[28] + tmpFx[39]*tmpObjS[41] + tmpFx[51]*tmpObjS[54] + tmpFx[63]*tmpObjS[67] + tmpFx[75]*tmpObjS[80] + tmpFx[87]*tmpObjS[93] + tmpFx[99]*tmpObjS[106] + tmpFx[111]*tmpObjS[119] + tmpFx[123]*tmpObjS[132] + tmpFx[135]*tmpObjS[145] + tmpFx[147]*tmpObjS[158];
tmpS2[42] = + tmpFx[3]*tmpObjS[3] + tmpFx[15]*tmpObjS[16] + tmpFx[27]*tmpObjS[29] + tmpFx[39]*tmpObjS[42] + tmpFx[51]*tmpObjS[55] + tmpFx[63]*tmpObjS[68] + tmpFx[75]*tmpObjS[81] + tmpFx[87]*tmpObjS[94] + tmpFx[99]*tmpObjS[107] + tmpFx[111]*tmpObjS[120] + tmpFx[123]*tmpObjS[133] + tmpFx[135]*tmpObjS[146] + tmpFx[147]*tmpObjS[159];
tmpS2[43] = + tmpFx[3]*tmpObjS[4] + tmpFx[15]*tmpObjS[17] + tmpFx[27]*tmpObjS[30] + tmpFx[39]*tmpObjS[43] + tmpFx[51]*tmpObjS[56] + tmpFx[63]*tmpObjS[69] + tmpFx[75]*tmpObjS[82] + tmpFx[87]*tmpObjS[95] + tmpFx[99]*tmpObjS[108] + tmpFx[111]*tmpObjS[121] + tmpFx[123]*tmpObjS[134] + tmpFx[135]*tmpObjS[147] + tmpFx[147]*tmpObjS[160];
tmpS2[44] = + tmpFx[3]*tmpObjS[5] + tmpFx[15]*tmpObjS[18] + tmpFx[27]*tmpObjS[31] + tmpFx[39]*tmpObjS[44] + tmpFx[51]*tmpObjS[57] + tmpFx[63]*tmpObjS[70] + tmpFx[75]*tmpObjS[83] + tmpFx[87]*tmpObjS[96] + tmpFx[99]*tmpObjS[109] + tmpFx[111]*tmpObjS[122] + tmpFx[123]*tmpObjS[135] + tmpFx[135]*tmpObjS[148] + tmpFx[147]*tmpObjS[161];
tmpS2[45] = + tmpFx[3]*tmpObjS[6] + tmpFx[15]*tmpObjS[19] + tmpFx[27]*tmpObjS[32] + tmpFx[39]*tmpObjS[45] + tmpFx[51]*tmpObjS[58] + tmpFx[63]*tmpObjS[71] + tmpFx[75]*tmpObjS[84] + tmpFx[87]*tmpObjS[97] + tmpFx[99]*tmpObjS[110] + tmpFx[111]*tmpObjS[123] + tmpFx[123]*tmpObjS[136] + tmpFx[135]*tmpObjS[149] + tmpFx[147]*tmpObjS[162];
tmpS2[46] = + tmpFx[3]*tmpObjS[7] + tmpFx[15]*tmpObjS[20] + tmpFx[27]*tmpObjS[33] + tmpFx[39]*tmpObjS[46] + tmpFx[51]*tmpObjS[59] + tmpFx[63]*tmpObjS[72] + tmpFx[75]*tmpObjS[85] + tmpFx[87]*tmpObjS[98] + tmpFx[99]*tmpObjS[111] + tmpFx[111]*tmpObjS[124] + tmpFx[123]*tmpObjS[137] + tmpFx[135]*tmpObjS[150] + tmpFx[147]*tmpObjS[163];
tmpS2[47] = + tmpFx[3]*tmpObjS[8] + tmpFx[15]*tmpObjS[21] + tmpFx[27]*tmpObjS[34] + tmpFx[39]*tmpObjS[47] + tmpFx[51]*tmpObjS[60] + tmpFx[63]*tmpObjS[73] + tmpFx[75]*tmpObjS[86] + tmpFx[87]*tmpObjS[99] + tmpFx[99]*tmpObjS[112] + tmpFx[111]*tmpObjS[125] + tmpFx[123]*tmpObjS[138] + tmpFx[135]*tmpObjS[151] + tmpFx[147]*tmpObjS[164];
tmpS2[48] = + tmpFx[3]*tmpObjS[9] + tmpFx[15]*tmpObjS[22] + tmpFx[27]*tmpObjS[35] + tmpFx[39]*tmpObjS[48] + tmpFx[51]*tmpObjS[61] + tmpFx[63]*tmpObjS[74] + tmpFx[75]*tmpObjS[87] + tmpFx[87]*tmpObjS[100] + tmpFx[99]*tmpObjS[113] + tmpFx[111]*tmpObjS[126] + tmpFx[123]*tmpObjS[139] + tmpFx[135]*tmpObjS[152] + tmpFx[147]*tmpObjS[165];
tmpS2[49] = + tmpFx[3]*tmpObjS[10] + tmpFx[15]*tmpObjS[23] + tmpFx[27]*tmpObjS[36] + tmpFx[39]*tmpObjS[49] + tmpFx[51]*tmpObjS[62] + tmpFx[63]*tmpObjS[75] + tmpFx[75]*tmpObjS[88] + tmpFx[87]*tmpObjS[101] + tmpFx[99]*tmpObjS[114] + tmpFx[111]*tmpObjS[127] + tmpFx[123]*tmpObjS[140] + tmpFx[135]*tmpObjS[153] + tmpFx[147]*tmpObjS[166];
tmpS2[50] = + tmpFx[3]*tmpObjS[11] + tmpFx[15]*tmpObjS[24] + tmpFx[27]*tmpObjS[37] + tmpFx[39]*tmpObjS[50] + tmpFx[51]*tmpObjS[63] + tmpFx[63]*tmpObjS[76] + tmpFx[75]*tmpObjS[89] + tmpFx[87]*tmpObjS[102] + tmpFx[99]*tmpObjS[115] + tmpFx[111]*tmpObjS[128] + tmpFx[123]*tmpObjS[141] + tmpFx[135]*tmpObjS[154] + tmpFx[147]*tmpObjS[167];
tmpS2[51] = + tmpFx[3]*tmpObjS[12] + tmpFx[15]*tmpObjS[25] + tmpFx[27]*tmpObjS[38] + tmpFx[39]*tmpObjS[51] + tmpFx[51]*tmpObjS[64] + tmpFx[63]*tmpObjS[77] + tmpFx[75]*tmpObjS[90] + tmpFx[87]*tmpObjS[103] + tmpFx[99]*tmpObjS[116] + tmpFx[111]*tmpObjS[129] + tmpFx[123]*tmpObjS[142] + tmpFx[135]*tmpObjS[155] + tmpFx[147]*tmpObjS[168];
tmpS2[52] = + tmpFx[4]*tmpObjS[0] + tmpFx[16]*tmpObjS[13] + tmpFx[28]*tmpObjS[26] + tmpFx[40]*tmpObjS[39] + tmpFx[52]*tmpObjS[52] + tmpFx[64]*tmpObjS[65] + tmpFx[76]*tmpObjS[78] + tmpFx[88]*tmpObjS[91] + tmpFx[100]*tmpObjS[104] + tmpFx[112]*tmpObjS[117] + tmpFx[124]*tmpObjS[130] + tmpFx[136]*tmpObjS[143] + tmpFx[148]*tmpObjS[156];
tmpS2[53] = + tmpFx[4]*tmpObjS[1] + tmpFx[16]*tmpObjS[14] + tmpFx[28]*tmpObjS[27] + tmpFx[40]*tmpObjS[40] + tmpFx[52]*tmpObjS[53] + tmpFx[64]*tmpObjS[66] + tmpFx[76]*tmpObjS[79] + tmpFx[88]*tmpObjS[92] + tmpFx[100]*tmpObjS[105] + tmpFx[112]*tmpObjS[118] + tmpFx[124]*tmpObjS[131] + tmpFx[136]*tmpObjS[144] + tmpFx[148]*tmpObjS[157];
tmpS2[54] = + tmpFx[4]*tmpObjS[2] + tmpFx[16]*tmpObjS[15] + tmpFx[28]*tmpObjS[28] + tmpFx[40]*tmpObjS[41] + tmpFx[52]*tmpObjS[54] + tmpFx[64]*tmpObjS[67] + tmpFx[76]*tmpObjS[80] + tmpFx[88]*tmpObjS[93] + tmpFx[100]*tmpObjS[106] + tmpFx[112]*tmpObjS[119] + tmpFx[124]*tmpObjS[132] + tmpFx[136]*tmpObjS[145] + tmpFx[148]*tmpObjS[158];
tmpS2[55] = + tmpFx[4]*tmpObjS[3] + tmpFx[16]*tmpObjS[16] + tmpFx[28]*tmpObjS[29] + tmpFx[40]*tmpObjS[42] + tmpFx[52]*tmpObjS[55] + tmpFx[64]*tmpObjS[68] + tmpFx[76]*tmpObjS[81] + tmpFx[88]*tmpObjS[94] + tmpFx[100]*tmpObjS[107] + tmpFx[112]*tmpObjS[120] + tmpFx[124]*tmpObjS[133] + tmpFx[136]*tmpObjS[146] + tmpFx[148]*tmpObjS[159];
tmpS2[56] = + tmpFx[4]*tmpObjS[4] + tmpFx[16]*tmpObjS[17] + tmpFx[28]*tmpObjS[30] + tmpFx[40]*tmpObjS[43] + tmpFx[52]*tmpObjS[56] + tmpFx[64]*tmpObjS[69] + tmpFx[76]*tmpObjS[82] + tmpFx[88]*tmpObjS[95] + tmpFx[100]*tmpObjS[108] + tmpFx[112]*tmpObjS[121] + tmpFx[124]*tmpObjS[134] + tmpFx[136]*tmpObjS[147] + tmpFx[148]*tmpObjS[160];
tmpS2[57] = + tmpFx[4]*tmpObjS[5] + tmpFx[16]*tmpObjS[18] + tmpFx[28]*tmpObjS[31] + tmpFx[40]*tmpObjS[44] + tmpFx[52]*tmpObjS[57] + tmpFx[64]*tmpObjS[70] + tmpFx[76]*tmpObjS[83] + tmpFx[88]*tmpObjS[96] + tmpFx[100]*tmpObjS[109] + tmpFx[112]*tmpObjS[122] + tmpFx[124]*tmpObjS[135] + tmpFx[136]*tmpObjS[148] + tmpFx[148]*tmpObjS[161];
tmpS2[58] = + tmpFx[4]*tmpObjS[6] + tmpFx[16]*tmpObjS[19] + tmpFx[28]*tmpObjS[32] + tmpFx[40]*tmpObjS[45] + tmpFx[52]*tmpObjS[58] + tmpFx[64]*tmpObjS[71] + tmpFx[76]*tmpObjS[84] + tmpFx[88]*tmpObjS[97] + tmpFx[100]*tmpObjS[110] + tmpFx[112]*tmpObjS[123] + tmpFx[124]*tmpObjS[136] + tmpFx[136]*tmpObjS[149] + tmpFx[148]*tmpObjS[162];
tmpS2[59] = + tmpFx[4]*tmpObjS[7] + tmpFx[16]*tmpObjS[20] + tmpFx[28]*tmpObjS[33] + tmpFx[40]*tmpObjS[46] + tmpFx[52]*tmpObjS[59] + tmpFx[64]*tmpObjS[72] + tmpFx[76]*tmpObjS[85] + tmpFx[88]*tmpObjS[98] + tmpFx[100]*tmpObjS[111] + tmpFx[112]*tmpObjS[124] + tmpFx[124]*tmpObjS[137] + tmpFx[136]*tmpObjS[150] + tmpFx[148]*tmpObjS[163];
tmpS2[60] = + tmpFx[4]*tmpObjS[8] + tmpFx[16]*tmpObjS[21] + tmpFx[28]*tmpObjS[34] + tmpFx[40]*tmpObjS[47] + tmpFx[52]*tmpObjS[60] + tmpFx[64]*tmpObjS[73] + tmpFx[76]*tmpObjS[86] + tmpFx[88]*tmpObjS[99] + tmpFx[100]*tmpObjS[112] + tmpFx[112]*tmpObjS[125] + tmpFx[124]*tmpObjS[138] + tmpFx[136]*tmpObjS[151] + tmpFx[148]*tmpObjS[164];
tmpS2[61] = + tmpFx[4]*tmpObjS[9] + tmpFx[16]*tmpObjS[22] + tmpFx[28]*tmpObjS[35] + tmpFx[40]*tmpObjS[48] + tmpFx[52]*tmpObjS[61] + tmpFx[64]*tmpObjS[74] + tmpFx[76]*tmpObjS[87] + tmpFx[88]*tmpObjS[100] + tmpFx[100]*tmpObjS[113] + tmpFx[112]*tmpObjS[126] + tmpFx[124]*tmpObjS[139] + tmpFx[136]*tmpObjS[152] + tmpFx[148]*tmpObjS[165];
tmpS2[62] = + tmpFx[4]*tmpObjS[10] + tmpFx[16]*tmpObjS[23] + tmpFx[28]*tmpObjS[36] + tmpFx[40]*tmpObjS[49] + tmpFx[52]*tmpObjS[62] + tmpFx[64]*tmpObjS[75] + tmpFx[76]*tmpObjS[88] + tmpFx[88]*tmpObjS[101] + tmpFx[100]*tmpObjS[114] + tmpFx[112]*tmpObjS[127] + tmpFx[124]*tmpObjS[140] + tmpFx[136]*tmpObjS[153] + tmpFx[148]*tmpObjS[166];
tmpS2[63] = + tmpFx[4]*tmpObjS[11] + tmpFx[16]*tmpObjS[24] + tmpFx[28]*tmpObjS[37] + tmpFx[40]*tmpObjS[50] + tmpFx[52]*tmpObjS[63] + tmpFx[64]*tmpObjS[76] + tmpFx[76]*tmpObjS[89] + tmpFx[88]*tmpObjS[102] + tmpFx[100]*tmpObjS[115] + tmpFx[112]*tmpObjS[128] + tmpFx[124]*tmpObjS[141] + tmpFx[136]*tmpObjS[154] + tmpFx[148]*tmpObjS[167];
tmpS2[64] = + tmpFx[4]*tmpObjS[12] + tmpFx[16]*tmpObjS[25] + tmpFx[28]*tmpObjS[38] + tmpFx[40]*tmpObjS[51] + tmpFx[52]*tmpObjS[64] + tmpFx[64]*tmpObjS[77] + tmpFx[76]*tmpObjS[90] + tmpFx[88]*tmpObjS[103] + tmpFx[100]*tmpObjS[116] + tmpFx[112]*tmpObjS[129] + tmpFx[124]*tmpObjS[142] + tmpFx[136]*tmpObjS[155] + tmpFx[148]*tmpObjS[168];
tmpS2[65] = + tmpFx[5]*tmpObjS[0] + tmpFx[17]*tmpObjS[13] + tmpFx[29]*tmpObjS[26] + tmpFx[41]*tmpObjS[39] + tmpFx[53]*tmpObjS[52] + tmpFx[65]*tmpObjS[65] + tmpFx[77]*tmpObjS[78] + tmpFx[89]*tmpObjS[91] + tmpFx[101]*tmpObjS[104] + tmpFx[113]*tmpObjS[117] + tmpFx[125]*tmpObjS[130] + tmpFx[137]*tmpObjS[143] + tmpFx[149]*tmpObjS[156];
tmpS2[66] = + tmpFx[5]*tmpObjS[1] + tmpFx[17]*tmpObjS[14] + tmpFx[29]*tmpObjS[27] + tmpFx[41]*tmpObjS[40] + tmpFx[53]*tmpObjS[53] + tmpFx[65]*tmpObjS[66] + tmpFx[77]*tmpObjS[79] + tmpFx[89]*tmpObjS[92] + tmpFx[101]*tmpObjS[105] + tmpFx[113]*tmpObjS[118] + tmpFx[125]*tmpObjS[131] + tmpFx[137]*tmpObjS[144] + tmpFx[149]*tmpObjS[157];
tmpS2[67] = + tmpFx[5]*tmpObjS[2] + tmpFx[17]*tmpObjS[15] + tmpFx[29]*tmpObjS[28] + tmpFx[41]*tmpObjS[41] + tmpFx[53]*tmpObjS[54] + tmpFx[65]*tmpObjS[67] + tmpFx[77]*tmpObjS[80] + tmpFx[89]*tmpObjS[93] + tmpFx[101]*tmpObjS[106] + tmpFx[113]*tmpObjS[119] + tmpFx[125]*tmpObjS[132] + tmpFx[137]*tmpObjS[145] + tmpFx[149]*tmpObjS[158];
tmpS2[68] = + tmpFx[5]*tmpObjS[3] + tmpFx[17]*tmpObjS[16] + tmpFx[29]*tmpObjS[29] + tmpFx[41]*tmpObjS[42] + tmpFx[53]*tmpObjS[55] + tmpFx[65]*tmpObjS[68] + tmpFx[77]*tmpObjS[81] + tmpFx[89]*tmpObjS[94] + tmpFx[101]*tmpObjS[107] + tmpFx[113]*tmpObjS[120] + tmpFx[125]*tmpObjS[133] + tmpFx[137]*tmpObjS[146] + tmpFx[149]*tmpObjS[159];
tmpS2[69] = + tmpFx[5]*tmpObjS[4] + tmpFx[17]*tmpObjS[17] + tmpFx[29]*tmpObjS[30] + tmpFx[41]*tmpObjS[43] + tmpFx[53]*tmpObjS[56] + tmpFx[65]*tmpObjS[69] + tmpFx[77]*tmpObjS[82] + tmpFx[89]*tmpObjS[95] + tmpFx[101]*tmpObjS[108] + tmpFx[113]*tmpObjS[121] + tmpFx[125]*tmpObjS[134] + tmpFx[137]*tmpObjS[147] + tmpFx[149]*tmpObjS[160];
tmpS2[70] = + tmpFx[5]*tmpObjS[5] + tmpFx[17]*tmpObjS[18] + tmpFx[29]*tmpObjS[31] + tmpFx[41]*tmpObjS[44] + tmpFx[53]*tmpObjS[57] + tmpFx[65]*tmpObjS[70] + tmpFx[77]*tmpObjS[83] + tmpFx[89]*tmpObjS[96] + tmpFx[101]*tmpObjS[109] + tmpFx[113]*tmpObjS[122] + tmpFx[125]*tmpObjS[135] + tmpFx[137]*tmpObjS[148] + tmpFx[149]*tmpObjS[161];
tmpS2[71] = + tmpFx[5]*tmpObjS[6] + tmpFx[17]*tmpObjS[19] + tmpFx[29]*tmpObjS[32] + tmpFx[41]*tmpObjS[45] + tmpFx[53]*tmpObjS[58] + tmpFx[65]*tmpObjS[71] + tmpFx[77]*tmpObjS[84] + tmpFx[89]*tmpObjS[97] + tmpFx[101]*tmpObjS[110] + tmpFx[113]*tmpObjS[123] + tmpFx[125]*tmpObjS[136] + tmpFx[137]*tmpObjS[149] + tmpFx[149]*tmpObjS[162];
tmpS2[72] = + tmpFx[5]*tmpObjS[7] + tmpFx[17]*tmpObjS[20] + tmpFx[29]*tmpObjS[33] + tmpFx[41]*tmpObjS[46] + tmpFx[53]*tmpObjS[59] + tmpFx[65]*tmpObjS[72] + tmpFx[77]*tmpObjS[85] + tmpFx[89]*tmpObjS[98] + tmpFx[101]*tmpObjS[111] + tmpFx[113]*tmpObjS[124] + tmpFx[125]*tmpObjS[137] + tmpFx[137]*tmpObjS[150] + tmpFx[149]*tmpObjS[163];
tmpS2[73] = + tmpFx[5]*tmpObjS[8] + tmpFx[17]*tmpObjS[21] + tmpFx[29]*tmpObjS[34] + tmpFx[41]*tmpObjS[47] + tmpFx[53]*tmpObjS[60] + tmpFx[65]*tmpObjS[73] + tmpFx[77]*tmpObjS[86] + tmpFx[89]*tmpObjS[99] + tmpFx[101]*tmpObjS[112] + tmpFx[113]*tmpObjS[125] + tmpFx[125]*tmpObjS[138] + tmpFx[137]*tmpObjS[151] + tmpFx[149]*tmpObjS[164];
tmpS2[74] = + tmpFx[5]*tmpObjS[9] + tmpFx[17]*tmpObjS[22] + tmpFx[29]*tmpObjS[35] + tmpFx[41]*tmpObjS[48] + tmpFx[53]*tmpObjS[61] + tmpFx[65]*tmpObjS[74] + tmpFx[77]*tmpObjS[87] + tmpFx[89]*tmpObjS[100] + tmpFx[101]*tmpObjS[113] + tmpFx[113]*tmpObjS[126] + tmpFx[125]*tmpObjS[139] + tmpFx[137]*tmpObjS[152] + tmpFx[149]*tmpObjS[165];
tmpS2[75] = + tmpFx[5]*tmpObjS[10] + tmpFx[17]*tmpObjS[23] + tmpFx[29]*tmpObjS[36] + tmpFx[41]*tmpObjS[49] + tmpFx[53]*tmpObjS[62] + tmpFx[65]*tmpObjS[75] + tmpFx[77]*tmpObjS[88] + tmpFx[89]*tmpObjS[101] + tmpFx[101]*tmpObjS[114] + tmpFx[113]*tmpObjS[127] + tmpFx[125]*tmpObjS[140] + tmpFx[137]*tmpObjS[153] + tmpFx[149]*tmpObjS[166];
tmpS2[76] = + tmpFx[5]*tmpObjS[11] + tmpFx[17]*tmpObjS[24] + tmpFx[29]*tmpObjS[37] + tmpFx[41]*tmpObjS[50] + tmpFx[53]*tmpObjS[63] + tmpFx[65]*tmpObjS[76] + tmpFx[77]*tmpObjS[89] + tmpFx[89]*tmpObjS[102] + tmpFx[101]*tmpObjS[115] + tmpFx[113]*tmpObjS[128] + tmpFx[125]*tmpObjS[141] + tmpFx[137]*tmpObjS[154] + tmpFx[149]*tmpObjS[167];
tmpS2[77] = + tmpFx[5]*tmpObjS[12] + tmpFx[17]*tmpObjS[25] + tmpFx[29]*tmpObjS[38] + tmpFx[41]*tmpObjS[51] + tmpFx[53]*tmpObjS[64] + tmpFx[65]*tmpObjS[77] + tmpFx[77]*tmpObjS[90] + tmpFx[89]*tmpObjS[103] + tmpFx[101]*tmpObjS[116] + tmpFx[113]*tmpObjS[129] + tmpFx[125]*tmpObjS[142] + tmpFx[137]*tmpObjS[155] + tmpFx[149]*tmpObjS[168];
tmpS2[78] = + tmpFx[6]*tmpObjS[0] + tmpFx[18]*tmpObjS[13] + tmpFx[30]*tmpObjS[26] + tmpFx[42]*tmpObjS[39] + tmpFx[54]*tmpObjS[52] + tmpFx[66]*tmpObjS[65] + tmpFx[78]*tmpObjS[78] + tmpFx[90]*tmpObjS[91] + tmpFx[102]*tmpObjS[104] + tmpFx[114]*tmpObjS[117] + tmpFx[126]*tmpObjS[130] + tmpFx[138]*tmpObjS[143] + tmpFx[150]*tmpObjS[156];
tmpS2[79] = + tmpFx[6]*tmpObjS[1] + tmpFx[18]*tmpObjS[14] + tmpFx[30]*tmpObjS[27] + tmpFx[42]*tmpObjS[40] + tmpFx[54]*tmpObjS[53] + tmpFx[66]*tmpObjS[66] + tmpFx[78]*tmpObjS[79] + tmpFx[90]*tmpObjS[92] + tmpFx[102]*tmpObjS[105] + tmpFx[114]*tmpObjS[118] + tmpFx[126]*tmpObjS[131] + tmpFx[138]*tmpObjS[144] + tmpFx[150]*tmpObjS[157];
tmpS2[80] = + tmpFx[6]*tmpObjS[2] + tmpFx[18]*tmpObjS[15] + tmpFx[30]*tmpObjS[28] + tmpFx[42]*tmpObjS[41] + tmpFx[54]*tmpObjS[54] + tmpFx[66]*tmpObjS[67] + tmpFx[78]*tmpObjS[80] + tmpFx[90]*tmpObjS[93] + tmpFx[102]*tmpObjS[106] + tmpFx[114]*tmpObjS[119] + tmpFx[126]*tmpObjS[132] + tmpFx[138]*tmpObjS[145] + tmpFx[150]*tmpObjS[158];
tmpS2[81] = + tmpFx[6]*tmpObjS[3] + tmpFx[18]*tmpObjS[16] + tmpFx[30]*tmpObjS[29] + tmpFx[42]*tmpObjS[42] + tmpFx[54]*tmpObjS[55] + tmpFx[66]*tmpObjS[68] + tmpFx[78]*tmpObjS[81] + tmpFx[90]*tmpObjS[94] + tmpFx[102]*tmpObjS[107] + tmpFx[114]*tmpObjS[120] + tmpFx[126]*tmpObjS[133] + tmpFx[138]*tmpObjS[146] + tmpFx[150]*tmpObjS[159];
tmpS2[82] = + tmpFx[6]*tmpObjS[4] + tmpFx[18]*tmpObjS[17] + tmpFx[30]*tmpObjS[30] + tmpFx[42]*tmpObjS[43] + tmpFx[54]*tmpObjS[56] + tmpFx[66]*tmpObjS[69] + tmpFx[78]*tmpObjS[82] + tmpFx[90]*tmpObjS[95] + tmpFx[102]*tmpObjS[108] + tmpFx[114]*tmpObjS[121] + tmpFx[126]*tmpObjS[134] + tmpFx[138]*tmpObjS[147] + tmpFx[150]*tmpObjS[160];
tmpS2[83] = + tmpFx[6]*tmpObjS[5] + tmpFx[18]*tmpObjS[18] + tmpFx[30]*tmpObjS[31] + tmpFx[42]*tmpObjS[44] + tmpFx[54]*tmpObjS[57] + tmpFx[66]*tmpObjS[70] + tmpFx[78]*tmpObjS[83] + tmpFx[90]*tmpObjS[96] + tmpFx[102]*tmpObjS[109] + tmpFx[114]*tmpObjS[122] + tmpFx[126]*tmpObjS[135] + tmpFx[138]*tmpObjS[148] + tmpFx[150]*tmpObjS[161];
tmpS2[84] = + tmpFx[6]*tmpObjS[6] + tmpFx[18]*tmpObjS[19] + tmpFx[30]*tmpObjS[32] + tmpFx[42]*tmpObjS[45] + tmpFx[54]*tmpObjS[58] + tmpFx[66]*tmpObjS[71] + tmpFx[78]*tmpObjS[84] + tmpFx[90]*tmpObjS[97] + tmpFx[102]*tmpObjS[110] + tmpFx[114]*tmpObjS[123] + tmpFx[126]*tmpObjS[136] + tmpFx[138]*tmpObjS[149] + tmpFx[150]*tmpObjS[162];
tmpS2[85] = + tmpFx[6]*tmpObjS[7] + tmpFx[18]*tmpObjS[20] + tmpFx[30]*tmpObjS[33] + tmpFx[42]*tmpObjS[46] + tmpFx[54]*tmpObjS[59] + tmpFx[66]*tmpObjS[72] + tmpFx[78]*tmpObjS[85] + tmpFx[90]*tmpObjS[98] + tmpFx[102]*tmpObjS[111] + tmpFx[114]*tmpObjS[124] + tmpFx[126]*tmpObjS[137] + tmpFx[138]*tmpObjS[150] + tmpFx[150]*tmpObjS[163];
tmpS2[86] = + tmpFx[6]*tmpObjS[8] + tmpFx[18]*tmpObjS[21] + tmpFx[30]*tmpObjS[34] + tmpFx[42]*tmpObjS[47] + tmpFx[54]*tmpObjS[60] + tmpFx[66]*tmpObjS[73] + tmpFx[78]*tmpObjS[86] + tmpFx[90]*tmpObjS[99] + tmpFx[102]*tmpObjS[112] + tmpFx[114]*tmpObjS[125] + tmpFx[126]*tmpObjS[138] + tmpFx[138]*tmpObjS[151] + tmpFx[150]*tmpObjS[164];
tmpS2[87] = + tmpFx[6]*tmpObjS[9] + tmpFx[18]*tmpObjS[22] + tmpFx[30]*tmpObjS[35] + tmpFx[42]*tmpObjS[48] + tmpFx[54]*tmpObjS[61] + tmpFx[66]*tmpObjS[74] + tmpFx[78]*tmpObjS[87] + tmpFx[90]*tmpObjS[100] + tmpFx[102]*tmpObjS[113] + tmpFx[114]*tmpObjS[126] + tmpFx[126]*tmpObjS[139] + tmpFx[138]*tmpObjS[152] + tmpFx[150]*tmpObjS[165];
tmpS2[88] = + tmpFx[6]*tmpObjS[10] + tmpFx[18]*tmpObjS[23] + tmpFx[30]*tmpObjS[36] + tmpFx[42]*tmpObjS[49] + tmpFx[54]*tmpObjS[62] + tmpFx[66]*tmpObjS[75] + tmpFx[78]*tmpObjS[88] + tmpFx[90]*tmpObjS[101] + tmpFx[102]*tmpObjS[114] + tmpFx[114]*tmpObjS[127] + tmpFx[126]*tmpObjS[140] + tmpFx[138]*tmpObjS[153] + tmpFx[150]*tmpObjS[166];
tmpS2[89] = + tmpFx[6]*tmpObjS[11] + tmpFx[18]*tmpObjS[24] + tmpFx[30]*tmpObjS[37] + tmpFx[42]*tmpObjS[50] + tmpFx[54]*tmpObjS[63] + tmpFx[66]*tmpObjS[76] + tmpFx[78]*tmpObjS[89] + tmpFx[90]*tmpObjS[102] + tmpFx[102]*tmpObjS[115] + tmpFx[114]*tmpObjS[128] + tmpFx[126]*tmpObjS[141] + tmpFx[138]*tmpObjS[154] + tmpFx[150]*tmpObjS[167];
tmpS2[90] = + tmpFx[6]*tmpObjS[12] + tmpFx[18]*tmpObjS[25] + tmpFx[30]*tmpObjS[38] + tmpFx[42]*tmpObjS[51] + tmpFx[54]*tmpObjS[64] + tmpFx[66]*tmpObjS[77] + tmpFx[78]*tmpObjS[90] + tmpFx[90]*tmpObjS[103] + tmpFx[102]*tmpObjS[116] + tmpFx[114]*tmpObjS[129] + tmpFx[126]*tmpObjS[142] + tmpFx[138]*tmpObjS[155] + tmpFx[150]*tmpObjS[168];
tmpS2[91] = + tmpFx[7]*tmpObjS[0] + tmpFx[19]*tmpObjS[13] + tmpFx[31]*tmpObjS[26] + tmpFx[43]*tmpObjS[39] + tmpFx[55]*tmpObjS[52] + tmpFx[67]*tmpObjS[65] + tmpFx[79]*tmpObjS[78] + tmpFx[91]*tmpObjS[91] + tmpFx[103]*tmpObjS[104] + tmpFx[115]*tmpObjS[117] + tmpFx[127]*tmpObjS[130] + tmpFx[139]*tmpObjS[143] + tmpFx[151]*tmpObjS[156];
tmpS2[92] = + tmpFx[7]*tmpObjS[1] + tmpFx[19]*tmpObjS[14] + tmpFx[31]*tmpObjS[27] + tmpFx[43]*tmpObjS[40] + tmpFx[55]*tmpObjS[53] + tmpFx[67]*tmpObjS[66] + tmpFx[79]*tmpObjS[79] + tmpFx[91]*tmpObjS[92] + tmpFx[103]*tmpObjS[105] + tmpFx[115]*tmpObjS[118] + tmpFx[127]*tmpObjS[131] + tmpFx[139]*tmpObjS[144] + tmpFx[151]*tmpObjS[157];
tmpS2[93] = + tmpFx[7]*tmpObjS[2] + tmpFx[19]*tmpObjS[15] + tmpFx[31]*tmpObjS[28] + tmpFx[43]*tmpObjS[41] + tmpFx[55]*tmpObjS[54] + tmpFx[67]*tmpObjS[67] + tmpFx[79]*tmpObjS[80] + tmpFx[91]*tmpObjS[93] + tmpFx[103]*tmpObjS[106] + tmpFx[115]*tmpObjS[119] + tmpFx[127]*tmpObjS[132] + tmpFx[139]*tmpObjS[145] + tmpFx[151]*tmpObjS[158];
tmpS2[94] = + tmpFx[7]*tmpObjS[3] + tmpFx[19]*tmpObjS[16] + tmpFx[31]*tmpObjS[29] + tmpFx[43]*tmpObjS[42] + tmpFx[55]*tmpObjS[55] + tmpFx[67]*tmpObjS[68] + tmpFx[79]*tmpObjS[81] + tmpFx[91]*tmpObjS[94] + tmpFx[103]*tmpObjS[107] + tmpFx[115]*tmpObjS[120] + tmpFx[127]*tmpObjS[133] + tmpFx[139]*tmpObjS[146] + tmpFx[151]*tmpObjS[159];
tmpS2[95] = + tmpFx[7]*tmpObjS[4] + tmpFx[19]*tmpObjS[17] + tmpFx[31]*tmpObjS[30] + tmpFx[43]*tmpObjS[43] + tmpFx[55]*tmpObjS[56] + tmpFx[67]*tmpObjS[69] + tmpFx[79]*tmpObjS[82] + tmpFx[91]*tmpObjS[95] + tmpFx[103]*tmpObjS[108] + tmpFx[115]*tmpObjS[121] + tmpFx[127]*tmpObjS[134] + tmpFx[139]*tmpObjS[147] + tmpFx[151]*tmpObjS[160];
tmpS2[96] = + tmpFx[7]*tmpObjS[5] + tmpFx[19]*tmpObjS[18] + tmpFx[31]*tmpObjS[31] + tmpFx[43]*tmpObjS[44] + tmpFx[55]*tmpObjS[57] + tmpFx[67]*tmpObjS[70] + tmpFx[79]*tmpObjS[83] + tmpFx[91]*tmpObjS[96] + tmpFx[103]*tmpObjS[109] + tmpFx[115]*tmpObjS[122] + tmpFx[127]*tmpObjS[135] + tmpFx[139]*tmpObjS[148] + tmpFx[151]*tmpObjS[161];
tmpS2[97] = + tmpFx[7]*tmpObjS[6] + tmpFx[19]*tmpObjS[19] + tmpFx[31]*tmpObjS[32] + tmpFx[43]*tmpObjS[45] + tmpFx[55]*tmpObjS[58] + tmpFx[67]*tmpObjS[71] + tmpFx[79]*tmpObjS[84] + tmpFx[91]*tmpObjS[97] + tmpFx[103]*tmpObjS[110] + tmpFx[115]*tmpObjS[123] + tmpFx[127]*tmpObjS[136] + tmpFx[139]*tmpObjS[149] + tmpFx[151]*tmpObjS[162];
tmpS2[98] = + tmpFx[7]*tmpObjS[7] + tmpFx[19]*tmpObjS[20] + tmpFx[31]*tmpObjS[33] + tmpFx[43]*tmpObjS[46] + tmpFx[55]*tmpObjS[59] + tmpFx[67]*tmpObjS[72] + tmpFx[79]*tmpObjS[85] + tmpFx[91]*tmpObjS[98] + tmpFx[103]*tmpObjS[111] + tmpFx[115]*tmpObjS[124] + tmpFx[127]*tmpObjS[137] + tmpFx[139]*tmpObjS[150] + tmpFx[151]*tmpObjS[163];
tmpS2[99] = + tmpFx[7]*tmpObjS[8] + tmpFx[19]*tmpObjS[21] + tmpFx[31]*tmpObjS[34] + tmpFx[43]*tmpObjS[47] + tmpFx[55]*tmpObjS[60] + tmpFx[67]*tmpObjS[73] + tmpFx[79]*tmpObjS[86] + tmpFx[91]*tmpObjS[99] + tmpFx[103]*tmpObjS[112] + tmpFx[115]*tmpObjS[125] + tmpFx[127]*tmpObjS[138] + tmpFx[139]*tmpObjS[151] + tmpFx[151]*tmpObjS[164];
tmpS2[100] = + tmpFx[7]*tmpObjS[9] + tmpFx[19]*tmpObjS[22] + tmpFx[31]*tmpObjS[35] + tmpFx[43]*tmpObjS[48] + tmpFx[55]*tmpObjS[61] + tmpFx[67]*tmpObjS[74] + tmpFx[79]*tmpObjS[87] + tmpFx[91]*tmpObjS[100] + tmpFx[103]*tmpObjS[113] + tmpFx[115]*tmpObjS[126] + tmpFx[127]*tmpObjS[139] + tmpFx[139]*tmpObjS[152] + tmpFx[151]*tmpObjS[165];
tmpS2[101] = + tmpFx[7]*tmpObjS[10] + tmpFx[19]*tmpObjS[23] + tmpFx[31]*tmpObjS[36] + tmpFx[43]*tmpObjS[49] + tmpFx[55]*tmpObjS[62] + tmpFx[67]*tmpObjS[75] + tmpFx[79]*tmpObjS[88] + tmpFx[91]*tmpObjS[101] + tmpFx[103]*tmpObjS[114] + tmpFx[115]*tmpObjS[127] + tmpFx[127]*tmpObjS[140] + tmpFx[139]*tmpObjS[153] + tmpFx[151]*tmpObjS[166];
tmpS2[102] = + tmpFx[7]*tmpObjS[11] + tmpFx[19]*tmpObjS[24] + tmpFx[31]*tmpObjS[37] + tmpFx[43]*tmpObjS[50] + tmpFx[55]*tmpObjS[63] + tmpFx[67]*tmpObjS[76] + tmpFx[79]*tmpObjS[89] + tmpFx[91]*tmpObjS[102] + tmpFx[103]*tmpObjS[115] + tmpFx[115]*tmpObjS[128] + tmpFx[127]*tmpObjS[141] + tmpFx[139]*tmpObjS[154] + tmpFx[151]*tmpObjS[167];
tmpS2[103] = + tmpFx[7]*tmpObjS[12] + tmpFx[19]*tmpObjS[25] + tmpFx[31]*tmpObjS[38] + tmpFx[43]*tmpObjS[51] + tmpFx[55]*tmpObjS[64] + tmpFx[67]*tmpObjS[77] + tmpFx[79]*tmpObjS[90] + tmpFx[91]*tmpObjS[103] + tmpFx[103]*tmpObjS[116] + tmpFx[115]*tmpObjS[129] + tmpFx[127]*tmpObjS[142] + tmpFx[139]*tmpObjS[155] + tmpFx[151]*tmpObjS[168];
tmpS2[104] = + tmpFx[8]*tmpObjS[0] + tmpFx[20]*tmpObjS[13] + tmpFx[32]*tmpObjS[26] + tmpFx[44]*tmpObjS[39] + tmpFx[56]*tmpObjS[52] + tmpFx[68]*tmpObjS[65] + tmpFx[80]*tmpObjS[78] + tmpFx[92]*tmpObjS[91] + tmpFx[104]*tmpObjS[104] + tmpFx[116]*tmpObjS[117] + tmpFx[128]*tmpObjS[130] + tmpFx[140]*tmpObjS[143] + tmpFx[152]*tmpObjS[156];
tmpS2[105] = + tmpFx[8]*tmpObjS[1] + tmpFx[20]*tmpObjS[14] + tmpFx[32]*tmpObjS[27] + tmpFx[44]*tmpObjS[40] + tmpFx[56]*tmpObjS[53] + tmpFx[68]*tmpObjS[66] + tmpFx[80]*tmpObjS[79] + tmpFx[92]*tmpObjS[92] + tmpFx[104]*tmpObjS[105] + tmpFx[116]*tmpObjS[118] + tmpFx[128]*tmpObjS[131] + tmpFx[140]*tmpObjS[144] + tmpFx[152]*tmpObjS[157];
tmpS2[106] = + tmpFx[8]*tmpObjS[2] + tmpFx[20]*tmpObjS[15] + tmpFx[32]*tmpObjS[28] + tmpFx[44]*tmpObjS[41] + tmpFx[56]*tmpObjS[54] + tmpFx[68]*tmpObjS[67] + tmpFx[80]*tmpObjS[80] + tmpFx[92]*tmpObjS[93] + tmpFx[104]*tmpObjS[106] + tmpFx[116]*tmpObjS[119] + tmpFx[128]*tmpObjS[132] + tmpFx[140]*tmpObjS[145] + tmpFx[152]*tmpObjS[158];
tmpS2[107] = + tmpFx[8]*tmpObjS[3] + tmpFx[20]*tmpObjS[16] + tmpFx[32]*tmpObjS[29] + tmpFx[44]*tmpObjS[42] + tmpFx[56]*tmpObjS[55] + tmpFx[68]*tmpObjS[68] + tmpFx[80]*tmpObjS[81] + tmpFx[92]*tmpObjS[94] + tmpFx[104]*tmpObjS[107] + tmpFx[116]*tmpObjS[120] + tmpFx[128]*tmpObjS[133] + tmpFx[140]*tmpObjS[146] + tmpFx[152]*tmpObjS[159];
tmpS2[108] = + tmpFx[8]*tmpObjS[4] + tmpFx[20]*tmpObjS[17] + tmpFx[32]*tmpObjS[30] + tmpFx[44]*tmpObjS[43] + tmpFx[56]*tmpObjS[56] + tmpFx[68]*tmpObjS[69] + tmpFx[80]*tmpObjS[82] + tmpFx[92]*tmpObjS[95] + tmpFx[104]*tmpObjS[108] + tmpFx[116]*tmpObjS[121] + tmpFx[128]*tmpObjS[134] + tmpFx[140]*tmpObjS[147] + tmpFx[152]*tmpObjS[160];
tmpS2[109] = + tmpFx[8]*tmpObjS[5] + tmpFx[20]*tmpObjS[18] + tmpFx[32]*tmpObjS[31] + tmpFx[44]*tmpObjS[44] + tmpFx[56]*tmpObjS[57] + tmpFx[68]*tmpObjS[70] + tmpFx[80]*tmpObjS[83] + tmpFx[92]*tmpObjS[96] + tmpFx[104]*tmpObjS[109] + tmpFx[116]*tmpObjS[122] + tmpFx[128]*tmpObjS[135] + tmpFx[140]*tmpObjS[148] + tmpFx[152]*tmpObjS[161];
tmpS2[110] = + tmpFx[8]*tmpObjS[6] + tmpFx[20]*tmpObjS[19] + tmpFx[32]*tmpObjS[32] + tmpFx[44]*tmpObjS[45] + tmpFx[56]*tmpObjS[58] + tmpFx[68]*tmpObjS[71] + tmpFx[80]*tmpObjS[84] + tmpFx[92]*tmpObjS[97] + tmpFx[104]*tmpObjS[110] + tmpFx[116]*tmpObjS[123] + tmpFx[128]*tmpObjS[136] + tmpFx[140]*tmpObjS[149] + tmpFx[152]*tmpObjS[162];
tmpS2[111] = + tmpFx[8]*tmpObjS[7] + tmpFx[20]*tmpObjS[20] + tmpFx[32]*tmpObjS[33] + tmpFx[44]*tmpObjS[46] + tmpFx[56]*tmpObjS[59] + tmpFx[68]*tmpObjS[72] + tmpFx[80]*tmpObjS[85] + tmpFx[92]*tmpObjS[98] + tmpFx[104]*tmpObjS[111] + tmpFx[116]*tmpObjS[124] + tmpFx[128]*tmpObjS[137] + tmpFx[140]*tmpObjS[150] + tmpFx[152]*tmpObjS[163];
tmpS2[112] = + tmpFx[8]*tmpObjS[8] + tmpFx[20]*tmpObjS[21] + tmpFx[32]*tmpObjS[34] + tmpFx[44]*tmpObjS[47] + tmpFx[56]*tmpObjS[60] + tmpFx[68]*tmpObjS[73] + tmpFx[80]*tmpObjS[86] + tmpFx[92]*tmpObjS[99] + tmpFx[104]*tmpObjS[112] + tmpFx[116]*tmpObjS[125] + tmpFx[128]*tmpObjS[138] + tmpFx[140]*tmpObjS[151] + tmpFx[152]*tmpObjS[164];
tmpS2[113] = + tmpFx[8]*tmpObjS[9] + tmpFx[20]*tmpObjS[22] + tmpFx[32]*tmpObjS[35] + tmpFx[44]*tmpObjS[48] + tmpFx[56]*tmpObjS[61] + tmpFx[68]*tmpObjS[74] + tmpFx[80]*tmpObjS[87] + tmpFx[92]*tmpObjS[100] + tmpFx[104]*tmpObjS[113] + tmpFx[116]*tmpObjS[126] + tmpFx[128]*tmpObjS[139] + tmpFx[140]*tmpObjS[152] + tmpFx[152]*tmpObjS[165];
tmpS2[114] = + tmpFx[8]*tmpObjS[10] + tmpFx[20]*tmpObjS[23] + tmpFx[32]*tmpObjS[36] + tmpFx[44]*tmpObjS[49] + tmpFx[56]*tmpObjS[62] + tmpFx[68]*tmpObjS[75] + tmpFx[80]*tmpObjS[88] + tmpFx[92]*tmpObjS[101] + tmpFx[104]*tmpObjS[114] + tmpFx[116]*tmpObjS[127] + tmpFx[128]*tmpObjS[140] + tmpFx[140]*tmpObjS[153] + tmpFx[152]*tmpObjS[166];
tmpS2[115] = + tmpFx[8]*tmpObjS[11] + tmpFx[20]*tmpObjS[24] + tmpFx[32]*tmpObjS[37] + tmpFx[44]*tmpObjS[50] + tmpFx[56]*tmpObjS[63] + tmpFx[68]*tmpObjS[76] + tmpFx[80]*tmpObjS[89] + tmpFx[92]*tmpObjS[102] + tmpFx[104]*tmpObjS[115] + tmpFx[116]*tmpObjS[128] + tmpFx[128]*tmpObjS[141] + tmpFx[140]*tmpObjS[154] + tmpFx[152]*tmpObjS[167];
tmpS2[116] = + tmpFx[8]*tmpObjS[12] + tmpFx[20]*tmpObjS[25] + tmpFx[32]*tmpObjS[38] + tmpFx[44]*tmpObjS[51] + tmpFx[56]*tmpObjS[64] + tmpFx[68]*tmpObjS[77] + tmpFx[80]*tmpObjS[90] + tmpFx[92]*tmpObjS[103] + tmpFx[104]*tmpObjS[116] + tmpFx[116]*tmpObjS[129] + tmpFx[128]*tmpObjS[142] + tmpFx[140]*tmpObjS[155] + tmpFx[152]*tmpObjS[168];
tmpS2[117] = + tmpFx[9]*tmpObjS[0] + tmpFx[21]*tmpObjS[13] + tmpFx[33]*tmpObjS[26] + tmpFx[45]*tmpObjS[39] + tmpFx[57]*tmpObjS[52] + tmpFx[69]*tmpObjS[65] + tmpFx[81]*tmpObjS[78] + tmpFx[93]*tmpObjS[91] + tmpFx[105]*tmpObjS[104] + tmpFx[117]*tmpObjS[117] + tmpFx[129]*tmpObjS[130] + tmpFx[141]*tmpObjS[143] + tmpFx[153]*tmpObjS[156];
tmpS2[118] = + tmpFx[9]*tmpObjS[1] + tmpFx[21]*tmpObjS[14] + tmpFx[33]*tmpObjS[27] + tmpFx[45]*tmpObjS[40] + tmpFx[57]*tmpObjS[53] + tmpFx[69]*tmpObjS[66] + tmpFx[81]*tmpObjS[79] + tmpFx[93]*tmpObjS[92] + tmpFx[105]*tmpObjS[105] + tmpFx[117]*tmpObjS[118] + tmpFx[129]*tmpObjS[131] + tmpFx[141]*tmpObjS[144] + tmpFx[153]*tmpObjS[157];
tmpS2[119] = + tmpFx[9]*tmpObjS[2] + tmpFx[21]*tmpObjS[15] + tmpFx[33]*tmpObjS[28] + tmpFx[45]*tmpObjS[41] + tmpFx[57]*tmpObjS[54] + tmpFx[69]*tmpObjS[67] + tmpFx[81]*tmpObjS[80] + tmpFx[93]*tmpObjS[93] + tmpFx[105]*tmpObjS[106] + tmpFx[117]*tmpObjS[119] + tmpFx[129]*tmpObjS[132] + tmpFx[141]*tmpObjS[145] + tmpFx[153]*tmpObjS[158];
tmpS2[120] = + tmpFx[9]*tmpObjS[3] + tmpFx[21]*tmpObjS[16] + tmpFx[33]*tmpObjS[29] + tmpFx[45]*tmpObjS[42] + tmpFx[57]*tmpObjS[55] + tmpFx[69]*tmpObjS[68] + tmpFx[81]*tmpObjS[81] + tmpFx[93]*tmpObjS[94] + tmpFx[105]*tmpObjS[107] + tmpFx[117]*tmpObjS[120] + tmpFx[129]*tmpObjS[133] + tmpFx[141]*tmpObjS[146] + tmpFx[153]*tmpObjS[159];
tmpS2[121] = + tmpFx[9]*tmpObjS[4] + tmpFx[21]*tmpObjS[17] + tmpFx[33]*tmpObjS[30] + tmpFx[45]*tmpObjS[43] + tmpFx[57]*tmpObjS[56] + tmpFx[69]*tmpObjS[69] + tmpFx[81]*tmpObjS[82] + tmpFx[93]*tmpObjS[95] + tmpFx[105]*tmpObjS[108] + tmpFx[117]*tmpObjS[121] + tmpFx[129]*tmpObjS[134] + tmpFx[141]*tmpObjS[147] + tmpFx[153]*tmpObjS[160];
tmpS2[122] = + tmpFx[9]*tmpObjS[5] + tmpFx[21]*tmpObjS[18] + tmpFx[33]*tmpObjS[31] + tmpFx[45]*tmpObjS[44] + tmpFx[57]*tmpObjS[57] + tmpFx[69]*tmpObjS[70] + tmpFx[81]*tmpObjS[83] + tmpFx[93]*tmpObjS[96] + tmpFx[105]*tmpObjS[109] + tmpFx[117]*tmpObjS[122] + tmpFx[129]*tmpObjS[135] + tmpFx[141]*tmpObjS[148] + tmpFx[153]*tmpObjS[161];
tmpS2[123] = + tmpFx[9]*tmpObjS[6] + tmpFx[21]*tmpObjS[19] + tmpFx[33]*tmpObjS[32] + tmpFx[45]*tmpObjS[45] + tmpFx[57]*tmpObjS[58] + tmpFx[69]*tmpObjS[71] + tmpFx[81]*tmpObjS[84] + tmpFx[93]*tmpObjS[97] + tmpFx[105]*tmpObjS[110] + tmpFx[117]*tmpObjS[123] + tmpFx[129]*tmpObjS[136] + tmpFx[141]*tmpObjS[149] + tmpFx[153]*tmpObjS[162];
tmpS2[124] = + tmpFx[9]*tmpObjS[7] + tmpFx[21]*tmpObjS[20] + tmpFx[33]*tmpObjS[33] + tmpFx[45]*tmpObjS[46] + tmpFx[57]*tmpObjS[59] + tmpFx[69]*tmpObjS[72] + tmpFx[81]*tmpObjS[85] + tmpFx[93]*tmpObjS[98] + tmpFx[105]*tmpObjS[111] + tmpFx[117]*tmpObjS[124] + tmpFx[129]*tmpObjS[137] + tmpFx[141]*tmpObjS[150] + tmpFx[153]*tmpObjS[163];
tmpS2[125] = + tmpFx[9]*tmpObjS[8] + tmpFx[21]*tmpObjS[21] + tmpFx[33]*tmpObjS[34] + tmpFx[45]*tmpObjS[47] + tmpFx[57]*tmpObjS[60] + tmpFx[69]*tmpObjS[73] + tmpFx[81]*tmpObjS[86] + tmpFx[93]*tmpObjS[99] + tmpFx[105]*tmpObjS[112] + tmpFx[117]*tmpObjS[125] + tmpFx[129]*tmpObjS[138] + tmpFx[141]*tmpObjS[151] + tmpFx[153]*tmpObjS[164];
tmpS2[126] = + tmpFx[9]*tmpObjS[9] + tmpFx[21]*tmpObjS[22] + tmpFx[33]*tmpObjS[35] + tmpFx[45]*tmpObjS[48] + tmpFx[57]*tmpObjS[61] + tmpFx[69]*tmpObjS[74] + tmpFx[81]*tmpObjS[87] + tmpFx[93]*tmpObjS[100] + tmpFx[105]*tmpObjS[113] + tmpFx[117]*tmpObjS[126] + tmpFx[129]*tmpObjS[139] + tmpFx[141]*tmpObjS[152] + tmpFx[153]*tmpObjS[165];
tmpS2[127] = + tmpFx[9]*tmpObjS[10] + tmpFx[21]*tmpObjS[23] + tmpFx[33]*tmpObjS[36] + tmpFx[45]*tmpObjS[49] + tmpFx[57]*tmpObjS[62] + tmpFx[69]*tmpObjS[75] + tmpFx[81]*tmpObjS[88] + tmpFx[93]*tmpObjS[101] + tmpFx[105]*tmpObjS[114] + tmpFx[117]*tmpObjS[127] + tmpFx[129]*tmpObjS[140] + tmpFx[141]*tmpObjS[153] + tmpFx[153]*tmpObjS[166];
tmpS2[128] = + tmpFx[9]*tmpObjS[11] + tmpFx[21]*tmpObjS[24] + tmpFx[33]*tmpObjS[37] + tmpFx[45]*tmpObjS[50] + tmpFx[57]*tmpObjS[63] + tmpFx[69]*tmpObjS[76] + tmpFx[81]*tmpObjS[89] + tmpFx[93]*tmpObjS[102] + tmpFx[105]*tmpObjS[115] + tmpFx[117]*tmpObjS[128] + tmpFx[129]*tmpObjS[141] + tmpFx[141]*tmpObjS[154] + tmpFx[153]*tmpObjS[167];
tmpS2[129] = + tmpFx[9]*tmpObjS[12] + tmpFx[21]*tmpObjS[25] + tmpFx[33]*tmpObjS[38] + tmpFx[45]*tmpObjS[51] + tmpFx[57]*tmpObjS[64] + tmpFx[69]*tmpObjS[77] + tmpFx[81]*tmpObjS[90] + tmpFx[93]*tmpObjS[103] + tmpFx[105]*tmpObjS[116] + tmpFx[117]*tmpObjS[129] + tmpFx[129]*tmpObjS[142] + tmpFx[141]*tmpObjS[155] + tmpFx[153]*tmpObjS[168];
tmpS2[130] = + tmpFx[10]*tmpObjS[0] + tmpFx[22]*tmpObjS[13] + tmpFx[34]*tmpObjS[26] + tmpFx[46]*tmpObjS[39] + tmpFx[58]*tmpObjS[52] + tmpFx[70]*tmpObjS[65] + tmpFx[82]*tmpObjS[78] + tmpFx[94]*tmpObjS[91] + tmpFx[106]*tmpObjS[104] + tmpFx[118]*tmpObjS[117] + tmpFx[130]*tmpObjS[130] + tmpFx[142]*tmpObjS[143] + tmpFx[154]*tmpObjS[156];
tmpS2[131] = + tmpFx[10]*tmpObjS[1] + tmpFx[22]*tmpObjS[14] + tmpFx[34]*tmpObjS[27] + tmpFx[46]*tmpObjS[40] + tmpFx[58]*tmpObjS[53] + tmpFx[70]*tmpObjS[66] + tmpFx[82]*tmpObjS[79] + tmpFx[94]*tmpObjS[92] + tmpFx[106]*tmpObjS[105] + tmpFx[118]*tmpObjS[118] + tmpFx[130]*tmpObjS[131] + tmpFx[142]*tmpObjS[144] + tmpFx[154]*tmpObjS[157];
tmpS2[132] = + tmpFx[10]*tmpObjS[2] + tmpFx[22]*tmpObjS[15] + tmpFx[34]*tmpObjS[28] + tmpFx[46]*tmpObjS[41] + tmpFx[58]*tmpObjS[54] + tmpFx[70]*tmpObjS[67] + tmpFx[82]*tmpObjS[80] + tmpFx[94]*tmpObjS[93] + tmpFx[106]*tmpObjS[106] + tmpFx[118]*tmpObjS[119] + tmpFx[130]*tmpObjS[132] + tmpFx[142]*tmpObjS[145] + tmpFx[154]*tmpObjS[158];
tmpS2[133] = + tmpFx[10]*tmpObjS[3] + tmpFx[22]*tmpObjS[16] + tmpFx[34]*tmpObjS[29] + tmpFx[46]*tmpObjS[42] + tmpFx[58]*tmpObjS[55] + tmpFx[70]*tmpObjS[68] + tmpFx[82]*tmpObjS[81] + tmpFx[94]*tmpObjS[94] + tmpFx[106]*tmpObjS[107] + tmpFx[118]*tmpObjS[120] + tmpFx[130]*tmpObjS[133] + tmpFx[142]*tmpObjS[146] + tmpFx[154]*tmpObjS[159];
tmpS2[134] = + tmpFx[10]*tmpObjS[4] + tmpFx[22]*tmpObjS[17] + tmpFx[34]*tmpObjS[30] + tmpFx[46]*tmpObjS[43] + tmpFx[58]*tmpObjS[56] + tmpFx[70]*tmpObjS[69] + tmpFx[82]*tmpObjS[82] + tmpFx[94]*tmpObjS[95] + tmpFx[106]*tmpObjS[108] + tmpFx[118]*tmpObjS[121] + tmpFx[130]*tmpObjS[134] + tmpFx[142]*tmpObjS[147] + tmpFx[154]*tmpObjS[160];
tmpS2[135] = + tmpFx[10]*tmpObjS[5] + tmpFx[22]*tmpObjS[18] + tmpFx[34]*tmpObjS[31] + tmpFx[46]*tmpObjS[44] + tmpFx[58]*tmpObjS[57] + tmpFx[70]*tmpObjS[70] + tmpFx[82]*tmpObjS[83] + tmpFx[94]*tmpObjS[96] + tmpFx[106]*tmpObjS[109] + tmpFx[118]*tmpObjS[122] + tmpFx[130]*tmpObjS[135] + tmpFx[142]*tmpObjS[148] + tmpFx[154]*tmpObjS[161];
tmpS2[136] = + tmpFx[10]*tmpObjS[6] + tmpFx[22]*tmpObjS[19] + tmpFx[34]*tmpObjS[32] + tmpFx[46]*tmpObjS[45] + tmpFx[58]*tmpObjS[58] + tmpFx[70]*tmpObjS[71] + tmpFx[82]*tmpObjS[84] + tmpFx[94]*tmpObjS[97] + tmpFx[106]*tmpObjS[110] + tmpFx[118]*tmpObjS[123] + tmpFx[130]*tmpObjS[136] + tmpFx[142]*tmpObjS[149] + tmpFx[154]*tmpObjS[162];
tmpS2[137] = + tmpFx[10]*tmpObjS[7] + tmpFx[22]*tmpObjS[20] + tmpFx[34]*tmpObjS[33] + tmpFx[46]*tmpObjS[46] + tmpFx[58]*tmpObjS[59] + tmpFx[70]*tmpObjS[72] + tmpFx[82]*tmpObjS[85] + tmpFx[94]*tmpObjS[98] + tmpFx[106]*tmpObjS[111] + tmpFx[118]*tmpObjS[124] + tmpFx[130]*tmpObjS[137] + tmpFx[142]*tmpObjS[150] + tmpFx[154]*tmpObjS[163];
tmpS2[138] = + tmpFx[10]*tmpObjS[8] + tmpFx[22]*tmpObjS[21] + tmpFx[34]*tmpObjS[34] + tmpFx[46]*tmpObjS[47] + tmpFx[58]*tmpObjS[60] + tmpFx[70]*tmpObjS[73] + tmpFx[82]*tmpObjS[86] + tmpFx[94]*tmpObjS[99] + tmpFx[106]*tmpObjS[112] + tmpFx[118]*tmpObjS[125] + tmpFx[130]*tmpObjS[138] + tmpFx[142]*tmpObjS[151] + tmpFx[154]*tmpObjS[164];
tmpS2[139] = + tmpFx[10]*tmpObjS[9] + tmpFx[22]*tmpObjS[22] + tmpFx[34]*tmpObjS[35] + tmpFx[46]*tmpObjS[48] + tmpFx[58]*tmpObjS[61] + tmpFx[70]*tmpObjS[74] + tmpFx[82]*tmpObjS[87] + tmpFx[94]*tmpObjS[100] + tmpFx[106]*tmpObjS[113] + tmpFx[118]*tmpObjS[126] + tmpFx[130]*tmpObjS[139] + tmpFx[142]*tmpObjS[152] + tmpFx[154]*tmpObjS[165];
tmpS2[140] = + tmpFx[10]*tmpObjS[10] + tmpFx[22]*tmpObjS[23] + tmpFx[34]*tmpObjS[36] + tmpFx[46]*tmpObjS[49] + tmpFx[58]*tmpObjS[62] + tmpFx[70]*tmpObjS[75] + tmpFx[82]*tmpObjS[88] + tmpFx[94]*tmpObjS[101] + tmpFx[106]*tmpObjS[114] + tmpFx[118]*tmpObjS[127] + tmpFx[130]*tmpObjS[140] + tmpFx[142]*tmpObjS[153] + tmpFx[154]*tmpObjS[166];
tmpS2[141] = + tmpFx[10]*tmpObjS[11] + tmpFx[22]*tmpObjS[24] + tmpFx[34]*tmpObjS[37] + tmpFx[46]*tmpObjS[50] + tmpFx[58]*tmpObjS[63] + tmpFx[70]*tmpObjS[76] + tmpFx[82]*tmpObjS[89] + tmpFx[94]*tmpObjS[102] + tmpFx[106]*tmpObjS[115] + tmpFx[118]*tmpObjS[128] + tmpFx[130]*tmpObjS[141] + tmpFx[142]*tmpObjS[154] + tmpFx[154]*tmpObjS[167];
tmpS2[142] = + tmpFx[10]*tmpObjS[12] + tmpFx[22]*tmpObjS[25] + tmpFx[34]*tmpObjS[38] + tmpFx[46]*tmpObjS[51] + tmpFx[58]*tmpObjS[64] + tmpFx[70]*tmpObjS[77] + tmpFx[82]*tmpObjS[90] + tmpFx[94]*tmpObjS[103] + tmpFx[106]*tmpObjS[116] + tmpFx[118]*tmpObjS[129] + tmpFx[130]*tmpObjS[142] + tmpFx[142]*tmpObjS[155] + tmpFx[154]*tmpObjS[168];
tmpS2[143] = + tmpFx[11]*tmpObjS[0] + tmpFx[23]*tmpObjS[13] + tmpFx[35]*tmpObjS[26] + tmpFx[47]*tmpObjS[39] + tmpFx[59]*tmpObjS[52] + tmpFx[71]*tmpObjS[65] + tmpFx[83]*tmpObjS[78] + tmpFx[95]*tmpObjS[91] + tmpFx[107]*tmpObjS[104] + tmpFx[119]*tmpObjS[117] + tmpFx[131]*tmpObjS[130] + tmpFx[143]*tmpObjS[143] + tmpFx[155]*tmpObjS[156];
tmpS2[144] = + tmpFx[11]*tmpObjS[1] + tmpFx[23]*tmpObjS[14] + tmpFx[35]*tmpObjS[27] + tmpFx[47]*tmpObjS[40] + tmpFx[59]*tmpObjS[53] + tmpFx[71]*tmpObjS[66] + tmpFx[83]*tmpObjS[79] + tmpFx[95]*tmpObjS[92] + tmpFx[107]*tmpObjS[105] + tmpFx[119]*tmpObjS[118] + tmpFx[131]*tmpObjS[131] + tmpFx[143]*tmpObjS[144] + tmpFx[155]*tmpObjS[157];
tmpS2[145] = + tmpFx[11]*tmpObjS[2] + tmpFx[23]*tmpObjS[15] + tmpFx[35]*tmpObjS[28] + tmpFx[47]*tmpObjS[41] + tmpFx[59]*tmpObjS[54] + tmpFx[71]*tmpObjS[67] + tmpFx[83]*tmpObjS[80] + tmpFx[95]*tmpObjS[93] + tmpFx[107]*tmpObjS[106] + tmpFx[119]*tmpObjS[119] + tmpFx[131]*tmpObjS[132] + tmpFx[143]*tmpObjS[145] + tmpFx[155]*tmpObjS[158];
tmpS2[146] = + tmpFx[11]*tmpObjS[3] + tmpFx[23]*tmpObjS[16] + tmpFx[35]*tmpObjS[29] + tmpFx[47]*tmpObjS[42] + tmpFx[59]*tmpObjS[55] + tmpFx[71]*tmpObjS[68] + tmpFx[83]*tmpObjS[81] + tmpFx[95]*tmpObjS[94] + tmpFx[107]*tmpObjS[107] + tmpFx[119]*tmpObjS[120] + tmpFx[131]*tmpObjS[133] + tmpFx[143]*tmpObjS[146] + tmpFx[155]*tmpObjS[159];
tmpS2[147] = + tmpFx[11]*tmpObjS[4] + tmpFx[23]*tmpObjS[17] + tmpFx[35]*tmpObjS[30] + tmpFx[47]*tmpObjS[43] + tmpFx[59]*tmpObjS[56] + tmpFx[71]*tmpObjS[69] + tmpFx[83]*tmpObjS[82] + tmpFx[95]*tmpObjS[95] + tmpFx[107]*tmpObjS[108] + tmpFx[119]*tmpObjS[121] + tmpFx[131]*tmpObjS[134] + tmpFx[143]*tmpObjS[147] + tmpFx[155]*tmpObjS[160];
tmpS2[148] = + tmpFx[11]*tmpObjS[5] + tmpFx[23]*tmpObjS[18] + tmpFx[35]*tmpObjS[31] + tmpFx[47]*tmpObjS[44] + tmpFx[59]*tmpObjS[57] + tmpFx[71]*tmpObjS[70] + tmpFx[83]*tmpObjS[83] + tmpFx[95]*tmpObjS[96] + tmpFx[107]*tmpObjS[109] + tmpFx[119]*tmpObjS[122] + tmpFx[131]*tmpObjS[135] + tmpFx[143]*tmpObjS[148] + tmpFx[155]*tmpObjS[161];
tmpS2[149] = + tmpFx[11]*tmpObjS[6] + tmpFx[23]*tmpObjS[19] + tmpFx[35]*tmpObjS[32] + tmpFx[47]*tmpObjS[45] + tmpFx[59]*tmpObjS[58] + tmpFx[71]*tmpObjS[71] + tmpFx[83]*tmpObjS[84] + tmpFx[95]*tmpObjS[97] + tmpFx[107]*tmpObjS[110] + tmpFx[119]*tmpObjS[123] + tmpFx[131]*tmpObjS[136] + tmpFx[143]*tmpObjS[149] + tmpFx[155]*tmpObjS[162];
tmpS2[150] = + tmpFx[11]*tmpObjS[7] + tmpFx[23]*tmpObjS[20] + tmpFx[35]*tmpObjS[33] + tmpFx[47]*tmpObjS[46] + tmpFx[59]*tmpObjS[59] + tmpFx[71]*tmpObjS[72] + tmpFx[83]*tmpObjS[85] + tmpFx[95]*tmpObjS[98] + tmpFx[107]*tmpObjS[111] + tmpFx[119]*tmpObjS[124] + tmpFx[131]*tmpObjS[137] + tmpFx[143]*tmpObjS[150] + tmpFx[155]*tmpObjS[163];
tmpS2[151] = + tmpFx[11]*tmpObjS[8] + tmpFx[23]*tmpObjS[21] + tmpFx[35]*tmpObjS[34] + tmpFx[47]*tmpObjS[47] + tmpFx[59]*tmpObjS[60] + tmpFx[71]*tmpObjS[73] + tmpFx[83]*tmpObjS[86] + tmpFx[95]*tmpObjS[99] + tmpFx[107]*tmpObjS[112] + tmpFx[119]*tmpObjS[125] + tmpFx[131]*tmpObjS[138] + tmpFx[143]*tmpObjS[151] + tmpFx[155]*tmpObjS[164];
tmpS2[152] = + tmpFx[11]*tmpObjS[9] + tmpFx[23]*tmpObjS[22] + tmpFx[35]*tmpObjS[35] + tmpFx[47]*tmpObjS[48] + tmpFx[59]*tmpObjS[61] + tmpFx[71]*tmpObjS[74] + tmpFx[83]*tmpObjS[87] + tmpFx[95]*tmpObjS[100] + tmpFx[107]*tmpObjS[113] + tmpFx[119]*tmpObjS[126] + tmpFx[131]*tmpObjS[139] + tmpFx[143]*tmpObjS[152] + tmpFx[155]*tmpObjS[165];
tmpS2[153] = + tmpFx[11]*tmpObjS[10] + tmpFx[23]*tmpObjS[23] + tmpFx[35]*tmpObjS[36] + tmpFx[47]*tmpObjS[49] + tmpFx[59]*tmpObjS[62] + tmpFx[71]*tmpObjS[75] + tmpFx[83]*tmpObjS[88] + tmpFx[95]*tmpObjS[101] + tmpFx[107]*tmpObjS[114] + tmpFx[119]*tmpObjS[127] + tmpFx[131]*tmpObjS[140] + tmpFx[143]*tmpObjS[153] + tmpFx[155]*tmpObjS[166];
tmpS2[154] = + tmpFx[11]*tmpObjS[11] + tmpFx[23]*tmpObjS[24] + tmpFx[35]*tmpObjS[37] + tmpFx[47]*tmpObjS[50] + tmpFx[59]*tmpObjS[63] + tmpFx[71]*tmpObjS[76] + tmpFx[83]*tmpObjS[89] + tmpFx[95]*tmpObjS[102] + tmpFx[107]*tmpObjS[115] + tmpFx[119]*tmpObjS[128] + tmpFx[131]*tmpObjS[141] + tmpFx[143]*tmpObjS[154] + tmpFx[155]*tmpObjS[167];
tmpS2[155] = + tmpFx[11]*tmpObjS[12] + tmpFx[23]*tmpObjS[25] + tmpFx[35]*tmpObjS[38] + tmpFx[47]*tmpObjS[51] + tmpFx[59]*tmpObjS[64] + tmpFx[71]*tmpObjS[77] + tmpFx[83]*tmpObjS[90] + tmpFx[95]*tmpObjS[103] + tmpFx[107]*tmpObjS[116] + tmpFx[119]*tmpObjS[129] + tmpFx[131]*tmpObjS[142] + tmpFx[143]*tmpObjS[155] + tmpFx[155]*tmpObjS[168];
tmpS1[0] = + tmpS2[0]*tmpFu[0] + tmpS2[1]*tmpFu[3] + tmpS2[2]*tmpFu[6] + tmpS2[3]*tmpFu[9] + tmpS2[4]*tmpFu[12] + tmpS2[5]*tmpFu[15] + tmpS2[6]*tmpFu[18] + tmpS2[7]*tmpFu[21] + tmpS2[8]*tmpFu[24] + tmpS2[9]*tmpFu[27] + tmpS2[10]*tmpFu[30] + tmpS2[11]*tmpFu[33] + tmpS2[12]*tmpFu[36];
tmpS1[1] = + tmpS2[0]*tmpFu[1] + tmpS2[1]*tmpFu[4] + tmpS2[2]*tmpFu[7] + tmpS2[3]*tmpFu[10] + tmpS2[4]*tmpFu[13] + tmpS2[5]*tmpFu[16] + tmpS2[6]*tmpFu[19] + tmpS2[7]*tmpFu[22] + tmpS2[8]*tmpFu[25] + tmpS2[9]*tmpFu[28] + tmpS2[10]*tmpFu[31] + tmpS2[11]*tmpFu[34] + tmpS2[12]*tmpFu[37];
tmpS1[2] = + tmpS2[0]*tmpFu[2] + tmpS2[1]*tmpFu[5] + tmpS2[2]*tmpFu[8] + tmpS2[3]*tmpFu[11] + tmpS2[4]*tmpFu[14] + tmpS2[5]*tmpFu[17] + tmpS2[6]*tmpFu[20] + tmpS2[7]*tmpFu[23] + tmpS2[8]*tmpFu[26] + tmpS2[9]*tmpFu[29] + tmpS2[10]*tmpFu[32] + tmpS2[11]*tmpFu[35] + tmpS2[12]*tmpFu[38];
tmpS1[3] = + tmpS2[13]*tmpFu[0] + tmpS2[14]*tmpFu[3] + tmpS2[15]*tmpFu[6] + tmpS2[16]*tmpFu[9] + tmpS2[17]*tmpFu[12] + tmpS2[18]*tmpFu[15] + tmpS2[19]*tmpFu[18] + tmpS2[20]*tmpFu[21] + tmpS2[21]*tmpFu[24] + tmpS2[22]*tmpFu[27] + tmpS2[23]*tmpFu[30] + tmpS2[24]*tmpFu[33] + tmpS2[25]*tmpFu[36];
tmpS1[4] = + tmpS2[13]*tmpFu[1] + tmpS2[14]*tmpFu[4] + tmpS2[15]*tmpFu[7] + tmpS2[16]*tmpFu[10] + tmpS2[17]*tmpFu[13] + tmpS2[18]*tmpFu[16] + tmpS2[19]*tmpFu[19] + tmpS2[20]*tmpFu[22] + tmpS2[21]*tmpFu[25] + tmpS2[22]*tmpFu[28] + tmpS2[23]*tmpFu[31] + tmpS2[24]*tmpFu[34] + tmpS2[25]*tmpFu[37];
tmpS1[5] = + tmpS2[13]*tmpFu[2] + tmpS2[14]*tmpFu[5] + tmpS2[15]*tmpFu[8] + tmpS2[16]*tmpFu[11] + tmpS2[17]*tmpFu[14] + tmpS2[18]*tmpFu[17] + tmpS2[19]*tmpFu[20] + tmpS2[20]*tmpFu[23] + tmpS2[21]*tmpFu[26] + tmpS2[22]*tmpFu[29] + tmpS2[23]*tmpFu[32] + tmpS2[24]*tmpFu[35] + tmpS2[25]*tmpFu[38];
tmpS1[6] = + tmpS2[26]*tmpFu[0] + tmpS2[27]*tmpFu[3] + tmpS2[28]*tmpFu[6] + tmpS2[29]*tmpFu[9] + tmpS2[30]*tmpFu[12] + tmpS2[31]*tmpFu[15] + tmpS2[32]*tmpFu[18] + tmpS2[33]*tmpFu[21] + tmpS2[34]*tmpFu[24] + tmpS2[35]*tmpFu[27] + tmpS2[36]*tmpFu[30] + tmpS2[37]*tmpFu[33] + tmpS2[38]*tmpFu[36];
tmpS1[7] = + tmpS2[26]*tmpFu[1] + tmpS2[27]*tmpFu[4] + tmpS2[28]*tmpFu[7] + tmpS2[29]*tmpFu[10] + tmpS2[30]*tmpFu[13] + tmpS2[31]*tmpFu[16] + tmpS2[32]*tmpFu[19] + tmpS2[33]*tmpFu[22] + tmpS2[34]*tmpFu[25] + tmpS2[35]*tmpFu[28] + tmpS2[36]*tmpFu[31] + tmpS2[37]*tmpFu[34] + tmpS2[38]*tmpFu[37];
tmpS1[8] = + tmpS2[26]*tmpFu[2] + tmpS2[27]*tmpFu[5] + tmpS2[28]*tmpFu[8] + tmpS2[29]*tmpFu[11] + tmpS2[30]*tmpFu[14] + tmpS2[31]*tmpFu[17] + tmpS2[32]*tmpFu[20] + tmpS2[33]*tmpFu[23] + tmpS2[34]*tmpFu[26] + tmpS2[35]*tmpFu[29] + tmpS2[36]*tmpFu[32] + tmpS2[37]*tmpFu[35] + tmpS2[38]*tmpFu[38];
tmpS1[9] = + tmpS2[39]*tmpFu[0] + tmpS2[40]*tmpFu[3] + tmpS2[41]*tmpFu[6] + tmpS2[42]*tmpFu[9] + tmpS2[43]*tmpFu[12] + tmpS2[44]*tmpFu[15] + tmpS2[45]*tmpFu[18] + tmpS2[46]*tmpFu[21] + tmpS2[47]*tmpFu[24] + tmpS2[48]*tmpFu[27] + tmpS2[49]*tmpFu[30] + tmpS2[50]*tmpFu[33] + tmpS2[51]*tmpFu[36];
tmpS1[10] = + tmpS2[39]*tmpFu[1] + tmpS2[40]*tmpFu[4] + tmpS2[41]*tmpFu[7] + tmpS2[42]*tmpFu[10] + tmpS2[43]*tmpFu[13] + tmpS2[44]*tmpFu[16] + tmpS2[45]*tmpFu[19] + tmpS2[46]*tmpFu[22] + tmpS2[47]*tmpFu[25] + tmpS2[48]*tmpFu[28] + tmpS2[49]*tmpFu[31] + tmpS2[50]*tmpFu[34] + tmpS2[51]*tmpFu[37];
tmpS1[11] = + tmpS2[39]*tmpFu[2] + tmpS2[40]*tmpFu[5] + tmpS2[41]*tmpFu[8] + tmpS2[42]*tmpFu[11] + tmpS2[43]*tmpFu[14] + tmpS2[44]*tmpFu[17] + tmpS2[45]*tmpFu[20] + tmpS2[46]*tmpFu[23] + tmpS2[47]*tmpFu[26] + tmpS2[48]*tmpFu[29] + tmpS2[49]*tmpFu[32] + tmpS2[50]*tmpFu[35] + tmpS2[51]*tmpFu[38];
tmpS1[12] = + tmpS2[52]*tmpFu[0] + tmpS2[53]*tmpFu[3] + tmpS2[54]*tmpFu[6] + tmpS2[55]*tmpFu[9] + tmpS2[56]*tmpFu[12] + tmpS2[57]*tmpFu[15] + tmpS2[58]*tmpFu[18] + tmpS2[59]*tmpFu[21] + tmpS2[60]*tmpFu[24] + tmpS2[61]*tmpFu[27] + tmpS2[62]*tmpFu[30] + tmpS2[63]*tmpFu[33] + tmpS2[64]*tmpFu[36];
tmpS1[13] = + tmpS2[52]*tmpFu[1] + tmpS2[53]*tmpFu[4] + tmpS2[54]*tmpFu[7] + tmpS2[55]*tmpFu[10] + tmpS2[56]*tmpFu[13] + tmpS2[57]*tmpFu[16] + tmpS2[58]*tmpFu[19] + tmpS2[59]*tmpFu[22] + tmpS2[60]*tmpFu[25] + tmpS2[61]*tmpFu[28] + tmpS2[62]*tmpFu[31] + tmpS2[63]*tmpFu[34] + tmpS2[64]*tmpFu[37];
tmpS1[14] = + tmpS2[52]*tmpFu[2] + tmpS2[53]*tmpFu[5] + tmpS2[54]*tmpFu[8] + tmpS2[55]*tmpFu[11] + tmpS2[56]*tmpFu[14] + tmpS2[57]*tmpFu[17] + tmpS2[58]*tmpFu[20] + tmpS2[59]*tmpFu[23] + tmpS2[60]*tmpFu[26] + tmpS2[61]*tmpFu[29] + tmpS2[62]*tmpFu[32] + tmpS2[63]*tmpFu[35] + tmpS2[64]*tmpFu[38];
tmpS1[15] = + tmpS2[65]*tmpFu[0] + tmpS2[66]*tmpFu[3] + tmpS2[67]*tmpFu[6] + tmpS2[68]*tmpFu[9] + tmpS2[69]*tmpFu[12] + tmpS2[70]*tmpFu[15] + tmpS2[71]*tmpFu[18] + tmpS2[72]*tmpFu[21] + tmpS2[73]*tmpFu[24] + tmpS2[74]*tmpFu[27] + tmpS2[75]*tmpFu[30] + tmpS2[76]*tmpFu[33] + tmpS2[77]*tmpFu[36];
tmpS1[16] = + tmpS2[65]*tmpFu[1] + tmpS2[66]*tmpFu[4] + tmpS2[67]*tmpFu[7] + tmpS2[68]*tmpFu[10] + tmpS2[69]*tmpFu[13] + tmpS2[70]*tmpFu[16] + tmpS2[71]*tmpFu[19] + tmpS2[72]*tmpFu[22] + tmpS2[73]*tmpFu[25] + tmpS2[74]*tmpFu[28] + tmpS2[75]*tmpFu[31] + tmpS2[76]*tmpFu[34] + tmpS2[77]*tmpFu[37];
tmpS1[17] = + tmpS2[65]*tmpFu[2] + tmpS2[66]*tmpFu[5] + tmpS2[67]*tmpFu[8] + tmpS2[68]*tmpFu[11] + tmpS2[69]*tmpFu[14] + tmpS2[70]*tmpFu[17] + tmpS2[71]*tmpFu[20] + tmpS2[72]*tmpFu[23] + tmpS2[73]*tmpFu[26] + tmpS2[74]*tmpFu[29] + tmpS2[75]*tmpFu[32] + tmpS2[76]*tmpFu[35] + tmpS2[77]*tmpFu[38];
tmpS1[18] = + tmpS2[78]*tmpFu[0] + tmpS2[79]*tmpFu[3] + tmpS2[80]*tmpFu[6] + tmpS2[81]*tmpFu[9] + tmpS2[82]*tmpFu[12] + tmpS2[83]*tmpFu[15] + tmpS2[84]*tmpFu[18] + tmpS2[85]*tmpFu[21] + tmpS2[86]*tmpFu[24] + tmpS2[87]*tmpFu[27] + tmpS2[88]*tmpFu[30] + tmpS2[89]*tmpFu[33] + tmpS2[90]*tmpFu[36];
tmpS1[19] = + tmpS2[78]*tmpFu[1] + tmpS2[79]*tmpFu[4] + tmpS2[80]*tmpFu[7] + tmpS2[81]*tmpFu[10] + tmpS2[82]*tmpFu[13] + tmpS2[83]*tmpFu[16] + tmpS2[84]*tmpFu[19] + tmpS2[85]*tmpFu[22] + tmpS2[86]*tmpFu[25] + tmpS2[87]*tmpFu[28] + tmpS2[88]*tmpFu[31] + tmpS2[89]*tmpFu[34] + tmpS2[90]*tmpFu[37];
tmpS1[20] = + tmpS2[78]*tmpFu[2] + tmpS2[79]*tmpFu[5] + tmpS2[80]*tmpFu[8] + tmpS2[81]*tmpFu[11] + tmpS2[82]*tmpFu[14] + tmpS2[83]*tmpFu[17] + tmpS2[84]*tmpFu[20] + tmpS2[85]*tmpFu[23] + tmpS2[86]*tmpFu[26] + tmpS2[87]*tmpFu[29] + tmpS2[88]*tmpFu[32] + tmpS2[89]*tmpFu[35] + tmpS2[90]*tmpFu[38];
tmpS1[21] = + tmpS2[91]*tmpFu[0] + tmpS2[92]*tmpFu[3] + tmpS2[93]*tmpFu[6] + tmpS2[94]*tmpFu[9] + tmpS2[95]*tmpFu[12] + tmpS2[96]*tmpFu[15] + tmpS2[97]*tmpFu[18] + tmpS2[98]*tmpFu[21] + tmpS2[99]*tmpFu[24] + tmpS2[100]*tmpFu[27] + tmpS2[101]*tmpFu[30] + tmpS2[102]*tmpFu[33] + tmpS2[103]*tmpFu[36];
tmpS1[22] = + tmpS2[91]*tmpFu[1] + tmpS2[92]*tmpFu[4] + tmpS2[93]*tmpFu[7] + tmpS2[94]*tmpFu[10] + tmpS2[95]*tmpFu[13] + tmpS2[96]*tmpFu[16] + tmpS2[97]*tmpFu[19] + tmpS2[98]*tmpFu[22] + tmpS2[99]*tmpFu[25] + tmpS2[100]*tmpFu[28] + tmpS2[101]*tmpFu[31] + tmpS2[102]*tmpFu[34] + tmpS2[103]*tmpFu[37];
tmpS1[23] = + tmpS2[91]*tmpFu[2] + tmpS2[92]*tmpFu[5] + tmpS2[93]*tmpFu[8] + tmpS2[94]*tmpFu[11] + tmpS2[95]*tmpFu[14] + tmpS2[96]*tmpFu[17] + tmpS2[97]*tmpFu[20] + tmpS2[98]*tmpFu[23] + tmpS2[99]*tmpFu[26] + tmpS2[100]*tmpFu[29] + tmpS2[101]*tmpFu[32] + tmpS2[102]*tmpFu[35] + tmpS2[103]*tmpFu[38];
tmpS1[24] = + tmpS2[104]*tmpFu[0] + tmpS2[105]*tmpFu[3] + tmpS2[106]*tmpFu[6] + tmpS2[107]*tmpFu[9] + tmpS2[108]*tmpFu[12] + tmpS2[109]*tmpFu[15] + tmpS2[110]*tmpFu[18] + tmpS2[111]*tmpFu[21] + tmpS2[112]*tmpFu[24] + tmpS2[113]*tmpFu[27] + tmpS2[114]*tmpFu[30] + tmpS2[115]*tmpFu[33] + tmpS2[116]*tmpFu[36];
tmpS1[25] = + tmpS2[104]*tmpFu[1] + tmpS2[105]*tmpFu[4] + tmpS2[106]*tmpFu[7] + tmpS2[107]*tmpFu[10] + tmpS2[108]*tmpFu[13] + tmpS2[109]*tmpFu[16] + tmpS2[110]*tmpFu[19] + tmpS2[111]*tmpFu[22] + tmpS2[112]*tmpFu[25] + tmpS2[113]*tmpFu[28] + tmpS2[114]*tmpFu[31] + tmpS2[115]*tmpFu[34] + tmpS2[116]*tmpFu[37];
tmpS1[26] = + tmpS2[104]*tmpFu[2] + tmpS2[105]*tmpFu[5] + tmpS2[106]*tmpFu[8] + tmpS2[107]*tmpFu[11] + tmpS2[108]*tmpFu[14] + tmpS2[109]*tmpFu[17] + tmpS2[110]*tmpFu[20] + tmpS2[111]*tmpFu[23] + tmpS2[112]*tmpFu[26] + tmpS2[113]*tmpFu[29] + tmpS2[114]*tmpFu[32] + tmpS2[115]*tmpFu[35] + tmpS2[116]*tmpFu[38];
tmpS1[27] = + tmpS2[117]*tmpFu[0] + tmpS2[118]*tmpFu[3] + tmpS2[119]*tmpFu[6] + tmpS2[120]*tmpFu[9] + tmpS2[121]*tmpFu[12] + tmpS2[122]*tmpFu[15] + tmpS2[123]*tmpFu[18] + tmpS2[124]*tmpFu[21] + tmpS2[125]*tmpFu[24] + tmpS2[126]*tmpFu[27] + tmpS2[127]*tmpFu[30] + tmpS2[128]*tmpFu[33] + tmpS2[129]*tmpFu[36];
tmpS1[28] = + tmpS2[117]*tmpFu[1] + tmpS2[118]*tmpFu[4] + tmpS2[119]*tmpFu[7] + tmpS2[120]*tmpFu[10] + tmpS2[121]*tmpFu[13] + tmpS2[122]*tmpFu[16] + tmpS2[123]*tmpFu[19] + tmpS2[124]*tmpFu[22] + tmpS2[125]*tmpFu[25] + tmpS2[126]*tmpFu[28] + tmpS2[127]*tmpFu[31] + tmpS2[128]*tmpFu[34] + tmpS2[129]*tmpFu[37];
tmpS1[29] = + tmpS2[117]*tmpFu[2] + tmpS2[118]*tmpFu[5] + tmpS2[119]*tmpFu[8] + tmpS2[120]*tmpFu[11] + tmpS2[121]*tmpFu[14] + tmpS2[122]*tmpFu[17] + tmpS2[123]*tmpFu[20] + tmpS2[124]*tmpFu[23] + tmpS2[125]*tmpFu[26] + tmpS2[126]*tmpFu[29] + tmpS2[127]*tmpFu[32] + tmpS2[128]*tmpFu[35] + tmpS2[129]*tmpFu[38];
tmpS1[30] = + tmpS2[130]*tmpFu[0] + tmpS2[131]*tmpFu[3] + tmpS2[132]*tmpFu[6] + tmpS2[133]*tmpFu[9] + tmpS2[134]*tmpFu[12] + tmpS2[135]*tmpFu[15] + tmpS2[136]*tmpFu[18] + tmpS2[137]*tmpFu[21] + tmpS2[138]*tmpFu[24] + tmpS2[139]*tmpFu[27] + tmpS2[140]*tmpFu[30] + tmpS2[141]*tmpFu[33] + tmpS2[142]*tmpFu[36];
tmpS1[31] = + tmpS2[130]*tmpFu[1] + tmpS2[131]*tmpFu[4] + tmpS2[132]*tmpFu[7] + tmpS2[133]*tmpFu[10] + tmpS2[134]*tmpFu[13] + tmpS2[135]*tmpFu[16] + tmpS2[136]*tmpFu[19] + tmpS2[137]*tmpFu[22] + tmpS2[138]*tmpFu[25] + tmpS2[139]*tmpFu[28] + tmpS2[140]*tmpFu[31] + tmpS2[141]*tmpFu[34] + tmpS2[142]*tmpFu[37];
tmpS1[32] = + tmpS2[130]*tmpFu[2] + tmpS2[131]*tmpFu[5] + tmpS2[132]*tmpFu[8] + tmpS2[133]*tmpFu[11] + tmpS2[134]*tmpFu[14] + tmpS2[135]*tmpFu[17] + tmpS2[136]*tmpFu[20] + tmpS2[137]*tmpFu[23] + tmpS2[138]*tmpFu[26] + tmpS2[139]*tmpFu[29] + tmpS2[140]*tmpFu[32] + tmpS2[141]*tmpFu[35] + tmpS2[142]*tmpFu[38];
tmpS1[33] = + tmpS2[143]*tmpFu[0] + tmpS2[144]*tmpFu[3] + tmpS2[145]*tmpFu[6] + tmpS2[146]*tmpFu[9] + tmpS2[147]*tmpFu[12] + tmpS2[148]*tmpFu[15] + tmpS2[149]*tmpFu[18] + tmpS2[150]*tmpFu[21] + tmpS2[151]*tmpFu[24] + tmpS2[152]*tmpFu[27] + tmpS2[153]*tmpFu[30] + tmpS2[154]*tmpFu[33] + tmpS2[155]*tmpFu[36];
tmpS1[34] = + tmpS2[143]*tmpFu[1] + tmpS2[144]*tmpFu[4] + tmpS2[145]*tmpFu[7] + tmpS2[146]*tmpFu[10] + tmpS2[147]*tmpFu[13] + tmpS2[148]*tmpFu[16] + tmpS2[149]*tmpFu[19] + tmpS2[150]*tmpFu[22] + tmpS2[151]*tmpFu[25] + tmpS2[152]*tmpFu[28] + tmpS2[153]*tmpFu[31] + tmpS2[154]*tmpFu[34] + tmpS2[155]*tmpFu[37];
tmpS1[35] = + tmpS2[143]*tmpFu[2] + tmpS2[144]*tmpFu[5] + tmpS2[145]*tmpFu[8] + tmpS2[146]*tmpFu[11] + tmpS2[147]*tmpFu[14] + tmpS2[148]*tmpFu[17] + tmpS2[149]*tmpFu[20] + tmpS2[150]*tmpFu[23] + tmpS2[151]*tmpFu[26] + tmpS2[152]*tmpFu[29] + tmpS2[153]*tmpFu[32] + tmpS2[154]*tmpFu[35] + tmpS2[155]*tmpFu[38];
}

void acado_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[18];
tmpQN2[1] = +tmpObjSEndTerm[19];
tmpQN2[2] = +tmpObjSEndTerm[20];
tmpQN2[3] = +tmpObjSEndTerm[21];
tmpQN2[4] = +tmpObjSEndTerm[22];
tmpQN2[5] = +tmpObjSEndTerm[23];
tmpQN2[6] = +tmpObjSEndTerm[24];
tmpQN2[7] = +tmpObjSEndTerm[25];
tmpQN2[8] = +tmpObjSEndTerm[26];
tmpQN2[9] = +tmpObjSEndTerm[27];
tmpQN2[10] = +tmpObjSEndTerm[28];
tmpQN2[11] = +tmpObjSEndTerm[29];
tmpQN2[12] = +tmpObjSEndTerm[30];
tmpQN2[13] = +tmpObjSEndTerm[31];
tmpQN2[14] = +tmpObjSEndTerm[32];
tmpQN2[15] = +tmpObjSEndTerm[33];
tmpQN2[16] = +tmpObjSEndTerm[34];
tmpQN2[17] = +tmpObjSEndTerm[35];
tmpQN2[18] = 0.0;
;
tmpQN2[19] = 0.0;
;
tmpQN2[20] = 0.0;
;
tmpQN2[21] = 0.0;
;
tmpQN2[22] = 0.0;
;
tmpQN2[23] = 0.0;
;
tmpQN2[24] = 0.0;
;
tmpQN2[25] = 0.0;
;
tmpQN2[26] = 0.0;
;
tmpQN2[27] = 0.0;
;
tmpQN2[28] = 0.0;
;
tmpQN2[29] = 0.0;
;
tmpQN2[30] = 0.0;
;
tmpQN2[31] = 0.0;
;
tmpQN2[32] = 0.0;
;
tmpQN2[33] = 0.0;
;
tmpQN2[34] = 0.0;
;
tmpQN2[35] = 0.0;
;
tmpQN2[36] = +tmpObjSEndTerm[0];
tmpQN2[37] = +tmpObjSEndTerm[1];
tmpQN2[38] = +tmpObjSEndTerm[2];
tmpQN2[39] = +tmpObjSEndTerm[3];
tmpQN2[40] = +tmpObjSEndTerm[4];
tmpQN2[41] = +tmpObjSEndTerm[5];
tmpQN2[42] = +tmpObjSEndTerm[6];
tmpQN2[43] = +tmpObjSEndTerm[7];
tmpQN2[44] = +tmpObjSEndTerm[8];
tmpQN2[45] = +tmpObjSEndTerm[9];
tmpQN2[46] = +tmpObjSEndTerm[10];
tmpQN2[47] = +tmpObjSEndTerm[11];
tmpQN2[48] = +tmpObjSEndTerm[12];
tmpQN2[49] = +tmpObjSEndTerm[13];
tmpQN2[50] = +tmpObjSEndTerm[14];
tmpQN2[51] = +tmpObjSEndTerm[15];
tmpQN2[52] = +tmpObjSEndTerm[16];
tmpQN2[53] = +tmpObjSEndTerm[17];
tmpQN2[54] = 0.0;
;
tmpQN2[55] = 0.0;
;
tmpQN2[56] = 0.0;
;
tmpQN2[57] = 0.0;
;
tmpQN2[58] = 0.0;
;
tmpQN2[59] = 0.0;
;
tmpQN2[60] = 0.0;
;
tmpQN2[61] = 0.0;
;
tmpQN2[62] = 0.0;
;
tmpQN2[63] = 0.0;
;
tmpQN2[64] = 0.0;
;
tmpQN2[65] = 0.0;
;
tmpQN2[66] = 0.0;
;
tmpQN2[67] = 0.0;
;
tmpQN2[68] = 0.0;
;
tmpQN2[69] = 0.0;
;
tmpQN2[70] = 0.0;
;
tmpQN2[71] = 0.0;
;
tmpQN1[0] = + tmpQN2[3];
tmpQN1[1] = + tmpQN2[4];
tmpQN1[2] = + tmpQN2[5];
tmpQN1[3] = 0.0;
;
tmpQN1[4] = 0.0;
;
tmpQN1[5] = 0.0;
;
tmpQN1[6] = + tmpQN2[0];
tmpQN1[7] = + tmpQN2[1];
tmpQN1[8] = + tmpQN2[2];
tmpQN1[9] = 0.0;
;
tmpQN1[10] = 0.0;
;
tmpQN1[11] = 0.0;
;
tmpQN1[12] = + tmpQN2[9];
tmpQN1[13] = + tmpQN2[10];
tmpQN1[14] = + tmpQN2[11];
tmpQN1[15] = 0.0;
;
tmpQN1[16] = 0.0;
;
tmpQN1[17] = 0.0;
;
tmpQN1[18] = + tmpQN2[6];
tmpQN1[19] = + tmpQN2[7];
tmpQN1[20] = + tmpQN2[8];
tmpQN1[21] = 0.0;
;
tmpQN1[22] = 0.0;
;
tmpQN1[23] = 0.0;
;
tmpQN1[24] = + tmpQN2[15];
tmpQN1[25] = + tmpQN2[16];
tmpQN1[26] = + tmpQN2[17];
tmpQN1[27] = 0.0;
;
tmpQN1[28] = 0.0;
;
tmpQN1[29] = 0.0;
;
tmpQN1[30] = + tmpQN2[12];
tmpQN1[31] = + tmpQN2[13];
tmpQN1[32] = + tmpQN2[14];
tmpQN1[33] = 0.0;
;
tmpQN1[34] = 0.0;
;
tmpQN1[35] = 0.0;
;
tmpQN1[36] = + tmpQN2[21];
tmpQN1[37] = + tmpQN2[22];
tmpQN1[38] = + tmpQN2[23];
tmpQN1[39] = 0.0;
;
tmpQN1[40] = 0.0;
;
tmpQN1[41] = 0.0;
;
tmpQN1[42] = + tmpQN2[18];
tmpQN1[43] = + tmpQN2[19];
tmpQN1[44] = + tmpQN2[20];
tmpQN1[45] = 0.0;
;
tmpQN1[46] = 0.0;
;
tmpQN1[47] = 0.0;
;
tmpQN1[48] = + tmpQN2[27];
tmpQN1[49] = + tmpQN2[28];
tmpQN1[50] = + tmpQN2[29];
tmpQN1[51] = 0.0;
;
tmpQN1[52] = 0.0;
;
tmpQN1[53] = 0.0;
;
tmpQN1[54] = + tmpQN2[24];
tmpQN1[55] = + tmpQN2[25];
tmpQN1[56] = + tmpQN2[26];
tmpQN1[57] = 0.0;
;
tmpQN1[58] = 0.0;
;
tmpQN1[59] = 0.0;
;
tmpQN1[60] = + tmpQN2[33];
tmpQN1[61] = + tmpQN2[34];
tmpQN1[62] = + tmpQN2[35];
tmpQN1[63] = 0.0;
;
tmpQN1[64] = 0.0;
;
tmpQN1[65] = 0.0;
;
tmpQN1[66] = + tmpQN2[30];
tmpQN1[67] = + tmpQN2[31];
tmpQN1[68] = + tmpQN2[32];
tmpQN1[69] = 0.0;
;
tmpQN1[70] = 0.0;
;
tmpQN1[71] = 0.0;
;
tmpQN1[72] = + tmpQN2[39];
tmpQN1[73] = + tmpQN2[40];
tmpQN1[74] = + tmpQN2[41];
tmpQN1[75] = 0.0;
;
tmpQN1[76] = 0.0;
;
tmpQN1[77] = 0.0;
;
tmpQN1[78] = + tmpQN2[36];
tmpQN1[79] = + tmpQN2[37];
tmpQN1[80] = + tmpQN2[38];
tmpQN1[81] = 0.0;
;
tmpQN1[82] = 0.0;
;
tmpQN1[83] = 0.0;
;
tmpQN1[84] = + tmpQN2[45];
tmpQN1[85] = + tmpQN2[46];
tmpQN1[86] = + tmpQN2[47];
tmpQN1[87] = 0.0;
;
tmpQN1[88] = 0.0;
;
tmpQN1[89] = 0.0;
;
tmpQN1[90] = + tmpQN2[42];
tmpQN1[91] = + tmpQN2[43];
tmpQN1[92] = + tmpQN2[44];
tmpQN1[93] = 0.0;
;
tmpQN1[94] = 0.0;
;
tmpQN1[95] = 0.0;
;
tmpQN1[96] = + tmpQN2[51];
tmpQN1[97] = + tmpQN2[52];
tmpQN1[98] = + tmpQN2[53];
tmpQN1[99] = 0.0;
;
tmpQN1[100] = 0.0;
;
tmpQN1[101] = 0.0;
;
tmpQN1[102] = + tmpQN2[48];
tmpQN1[103] = + tmpQN2[49];
tmpQN1[104] = + tmpQN2[50];
tmpQN1[105] = 0.0;
;
tmpQN1[106] = 0.0;
;
tmpQN1[107] = 0.0;
;
tmpQN1[108] = + tmpQN2[57];
tmpQN1[109] = + tmpQN2[58];
tmpQN1[110] = + tmpQN2[59];
tmpQN1[111] = 0.0;
;
tmpQN1[112] = 0.0;
;
tmpQN1[113] = 0.0;
;
tmpQN1[114] = + tmpQN2[54];
tmpQN1[115] = + tmpQN2[55];
tmpQN1[116] = + tmpQN2[56];
tmpQN1[117] = 0.0;
;
tmpQN1[118] = 0.0;
;
tmpQN1[119] = 0.0;
;
tmpQN1[120] = + tmpQN2[63];
tmpQN1[121] = + tmpQN2[64];
tmpQN1[122] = + tmpQN2[65];
tmpQN1[123] = 0.0;
;
tmpQN1[124] = 0.0;
;
tmpQN1[125] = 0.0;
;
tmpQN1[126] = + tmpQN2[60];
tmpQN1[127] = + tmpQN2[61];
tmpQN1[128] = + tmpQN2[62];
tmpQN1[129] = 0.0;
;
tmpQN1[130] = 0.0;
;
tmpQN1[131] = 0.0;
;
tmpQN1[132] = + tmpQN2[69];
tmpQN1[133] = + tmpQN2[70];
tmpQN1[134] = + tmpQN2[71];
tmpQN1[135] = 0.0;
;
tmpQN1[136] = 0.0;
;
tmpQN1[137] = 0.0;
;
tmpQN1[138] = + tmpQN2[66];
tmpQN1[139] = + tmpQN2[67];
tmpQN1[140] = + tmpQN2[68];
tmpQN1[141] = 0.0;
;
tmpQN1[142] = 0.0;
;
tmpQN1[143] = 0.0;
;
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 5; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 12];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 12 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 12 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 12 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 12 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[runObj * 12 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[runObj * 12 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[runObj * 12 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[runObj * 12 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.x[runObj * 12 + 9];
acadoWorkspace.objValueIn[10] = acadoVariables.x[runObj * 12 + 10];
acadoWorkspace.objValueIn[11] = acadoVariables.x[runObj * 12 + 11];
acadoWorkspace.objValueIn[12] = acadoVariables.u[runObj * 3];
acadoWorkspace.objValueIn[13] = acadoVariables.u[runObj * 3 + 1];
acadoWorkspace.objValueIn[14] = acadoVariables.u[runObj * 3 + 2];
acadoWorkspace.objValueIn[15] = acadoVariables.od[runObj * 14];
acadoWorkspace.objValueIn[16] = acadoVariables.od[runObj * 14 + 1];
acadoWorkspace.objValueIn[17] = acadoVariables.od[runObj * 14 + 2];
acadoWorkspace.objValueIn[18] = acadoVariables.od[runObj * 14 + 3];
acadoWorkspace.objValueIn[19] = acadoVariables.od[runObj * 14 + 4];
acadoWorkspace.objValueIn[20] = acadoVariables.od[runObj * 14 + 5];
acadoWorkspace.objValueIn[21] = acadoVariables.od[runObj * 14 + 6];
acadoWorkspace.objValueIn[22] = acadoVariables.od[runObj * 14 + 7];
acadoWorkspace.objValueIn[23] = acadoVariables.od[runObj * 14 + 8];
acadoWorkspace.objValueIn[24] = acadoVariables.od[runObj * 14 + 9];
acadoWorkspace.objValueIn[25] = acadoVariables.od[runObj * 14 + 10];
acadoWorkspace.objValueIn[26] = acadoVariables.od[runObj * 14 + 11];
acadoWorkspace.objValueIn[27] = acadoVariables.od[runObj * 14 + 12];
acadoWorkspace.objValueIn[28] = acadoVariables.od[runObj * 14 + 13];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 13] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 13 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 13 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 13 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 13 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 13 + 5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.Dy[runObj * 13 + 6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.Dy[runObj * 13 + 7] = acadoWorkspace.objValueOut[7];
acadoWorkspace.Dy[runObj * 13 + 8] = acadoWorkspace.objValueOut[8];
acadoWorkspace.Dy[runObj * 13 + 9] = acadoWorkspace.objValueOut[9];
acadoWorkspace.Dy[runObj * 13 + 10] = acadoWorkspace.objValueOut[10];
acadoWorkspace.Dy[runObj * 13 + 11] = acadoWorkspace.objValueOut[11];
acadoWorkspace.Dy[runObj * 13 + 12] = acadoWorkspace.objValueOut[12];

acado_setObjQ1Q2( &(acadoWorkspace.objValueOut[ 13 ]), acadoVariables.W, &(acadoWorkspace.Q1[ runObj * 144 ]), &(acadoWorkspace.Q2[ runObj * 156 ]) );

acado_setObjR1R2( &(acadoWorkspace.objValueOut[ 169 ]), acadoVariables.W, &(acadoWorkspace.R1[ runObj * 9 ]), &(acadoWorkspace.R2[ runObj * 39 ]) );

acado_setObjS1( &(acadoWorkspace.objValueOut[ 13 ]), &(acadoWorkspace.objValueOut[ 169 ]), acadoVariables.W, &(acadoWorkspace.S1[ runObj * 36 ]) );
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[60];
acadoWorkspace.objValueIn[1] = acadoVariables.x[61];
acadoWorkspace.objValueIn[2] = acadoVariables.x[62];
acadoWorkspace.objValueIn[3] = acadoVariables.x[63];
acadoWorkspace.objValueIn[4] = acadoVariables.x[64];
acadoWorkspace.objValueIn[5] = acadoVariables.x[65];
acadoWorkspace.objValueIn[6] = acadoVariables.x[66];
acadoWorkspace.objValueIn[7] = acadoVariables.x[67];
acadoWorkspace.objValueIn[8] = acadoVariables.x[68];
acadoWorkspace.objValueIn[9] = acadoVariables.x[69];
acadoWorkspace.objValueIn[10] = acadoVariables.x[70];
acadoWorkspace.objValueIn[11] = acadoVariables.x[71];
acadoWorkspace.objValueIn[12] = acadoVariables.od[70];
acadoWorkspace.objValueIn[13] = acadoVariables.od[71];
acadoWorkspace.objValueIn[14] = acadoVariables.od[72];
acadoWorkspace.objValueIn[15] = acadoVariables.od[73];
acadoWorkspace.objValueIn[16] = acadoVariables.od[74];
acadoWorkspace.objValueIn[17] = acadoVariables.od[75];
acadoWorkspace.objValueIn[18] = acadoVariables.od[76];
acadoWorkspace.objValueIn[19] = acadoVariables.od[77];
acadoWorkspace.objValueIn[20] = acadoVariables.od[78];
acadoWorkspace.objValueIn[21] = acadoVariables.od[79];
acadoWorkspace.objValueIn[22] = acadoVariables.od[80];
acadoWorkspace.objValueIn[23] = acadoVariables.od[81];
acadoWorkspace.objValueIn[24] = acadoVariables.od[82];
acadoWorkspace.objValueIn[25] = acadoVariables.od[83];
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
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[6] + Gx1[3]*Gu1[9] + Gx1[4]*Gu1[12] + Gx1[5]*Gu1[15] + Gx1[6]*Gu1[18] + Gx1[7]*Gu1[21] + Gx1[8]*Gu1[24] + Gx1[9]*Gu1[27] + Gx1[10]*Gu1[30] + Gx1[11]*Gu1[33];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[4] + Gx1[2]*Gu1[7] + Gx1[3]*Gu1[10] + Gx1[4]*Gu1[13] + Gx1[5]*Gu1[16] + Gx1[6]*Gu1[19] + Gx1[7]*Gu1[22] + Gx1[8]*Gu1[25] + Gx1[9]*Gu1[28] + Gx1[10]*Gu1[31] + Gx1[11]*Gu1[34];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[1]*Gu1[5] + Gx1[2]*Gu1[8] + Gx1[3]*Gu1[11] + Gx1[4]*Gu1[14] + Gx1[5]*Gu1[17] + Gx1[6]*Gu1[20] + Gx1[7]*Gu1[23] + Gx1[8]*Gu1[26] + Gx1[9]*Gu1[29] + Gx1[10]*Gu1[32] + Gx1[11]*Gu1[35];
Gu2[3] = + Gx1[12]*Gu1[0] + Gx1[13]*Gu1[3] + Gx1[14]*Gu1[6] + Gx1[15]*Gu1[9] + Gx1[16]*Gu1[12] + Gx1[17]*Gu1[15] + Gx1[18]*Gu1[18] + Gx1[19]*Gu1[21] + Gx1[20]*Gu1[24] + Gx1[21]*Gu1[27] + Gx1[22]*Gu1[30] + Gx1[23]*Gu1[33];
Gu2[4] = + Gx1[12]*Gu1[1] + Gx1[13]*Gu1[4] + Gx1[14]*Gu1[7] + Gx1[15]*Gu1[10] + Gx1[16]*Gu1[13] + Gx1[17]*Gu1[16] + Gx1[18]*Gu1[19] + Gx1[19]*Gu1[22] + Gx1[20]*Gu1[25] + Gx1[21]*Gu1[28] + Gx1[22]*Gu1[31] + Gx1[23]*Gu1[34];
Gu2[5] = + Gx1[12]*Gu1[2] + Gx1[13]*Gu1[5] + Gx1[14]*Gu1[8] + Gx1[15]*Gu1[11] + Gx1[16]*Gu1[14] + Gx1[17]*Gu1[17] + Gx1[18]*Gu1[20] + Gx1[19]*Gu1[23] + Gx1[20]*Gu1[26] + Gx1[21]*Gu1[29] + Gx1[22]*Gu1[32] + Gx1[23]*Gu1[35];
Gu2[6] = + Gx1[24]*Gu1[0] + Gx1[25]*Gu1[3] + Gx1[26]*Gu1[6] + Gx1[27]*Gu1[9] + Gx1[28]*Gu1[12] + Gx1[29]*Gu1[15] + Gx1[30]*Gu1[18] + Gx1[31]*Gu1[21] + Gx1[32]*Gu1[24] + Gx1[33]*Gu1[27] + Gx1[34]*Gu1[30] + Gx1[35]*Gu1[33];
Gu2[7] = + Gx1[24]*Gu1[1] + Gx1[25]*Gu1[4] + Gx1[26]*Gu1[7] + Gx1[27]*Gu1[10] + Gx1[28]*Gu1[13] + Gx1[29]*Gu1[16] + Gx1[30]*Gu1[19] + Gx1[31]*Gu1[22] + Gx1[32]*Gu1[25] + Gx1[33]*Gu1[28] + Gx1[34]*Gu1[31] + Gx1[35]*Gu1[34];
Gu2[8] = + Gx1[24]*Gu1[2] + Gx1[25]*Gu1[5] + Gx1[26]*Gu1[8] + Gx1[27]*Gu1[11] + Gx1[28]*Gu1[14] + Gx1[29]*Gu1[17] + Gx1[30]*Gu1[20] + Gx1[31]*Gu1[23] + Gx1[32]*Gu1[26] + Gx1[33]*Gu1[29] + Gx1[34]*Gu1[32] + Gx1[35]*Gu1[35];
Gu2[9] = + Gx1[36]*Gu1[0] + Gx1[37]*Gu1[3] + Gx1[38]*Gu1[6] + Gx1[39]*Gu1[9] + Gx1[40]*Gu1[12] + Gx1[41]*Gu1[15] + Gx1[42]*Gu1[18] + Gx1[43]*Gu1[21] + Gx1[44]*Gu1[24] + Gx1[45]*Gu1[27] + Gx1[46]*Gu1[30] + Gx1[47]*Gu1[33];
Gu2[10] = + Gx1[36]*Gu1[1] + Gx1[37]*Gu1[4] + Gx1[38]*Gu1[7] + Gx1[39]*Gu1[10] + Gx1[40]*Gu1[13] + Gx1[41]*Gu1[16] + Gx1[42]*Gu1[19] + Gx1[43]*Gu1[22] + Gx1[44]*Gu1[25] + Gx1[45]*Gu1[28] + Gx1[46]*Gu1[31] + Gx1[47]*Gu1[34];
Gu2[11] = + Gx1[36]*Gu1[2] + Gx1[37]*Gu1[5] + Gx1[38]*Gu1[8] + Gx1[39]*Gu1[11] + Gx1[40]*Gu1[14] + Gx1[41]*Gu1[17] + Gx1[42]*Gu1[20] + Gx1[43]*Gu1[23] + Gx1[44]*Gu1[26] + Gx1[45]*Gu1[29] + Gx1[46]*Gu1[32] + Gx1[47]*Gu1[35];
Gu2[12] = + Gx1[48]*Gu1[0] + Gx1[49]*Gu1[3] + Gx1[50]*Gu1[6] + Gx1[51]*Gu1[9] + Gx1[52]*Gu1[12] + Gx1[53]*Gu1[15] + Gx1[54]*Gu1[18] + Gx1[55]*Gu1[21] + Gx1[56]*Gu1[24] + Gx1[57]*Gu1[27] + Gx1[58]*Gu1[30] + Gx1[59]*Gu1[33];
Gu2[13] = + Gx1[48]*Gu1[1] + Gx1[49]*Gu1[4] + Gx1[50]*Gu1[7] + Gx1[51]*Gu1[10] + Gx1[52]*Gu1[13] + Gx1[53]*Gu1[16] + Gx1[54]*Gu1[19] + Gx1[55]*Gu1[22] + Gx1[56]*Gu1[25] + Gx1[57]*Gu1[28] + Gx1[58]*Gu1[31] + Gx1[59]*Gu1[34];
Gu2[14] = + Gx1[48]*Gu1[2] + Gx1[49]*Gu1[5] + Gx1[50]*Gu1[8] + Gx1[51]*Gu1[11] + Gx1[52]*Gu1[14] + Gx1[53]*Gu1[17] + Gx1[54]*Gu1[20] + Gx1[55]*Gu1[23] + Gx1[56]*Gu1[26] + Gx1[57]*Gu1[29] + Gx1[58]*Gu1[32] + Gx1[59]*Gu1[35];
Gu2[15] = + Gx1[60]*Gu1[0] + Gx1[61]*Gu1[3] + Gx1[62]*Gu1[6] + Gx1[63]*Gu1[9] + Gx1[64]*Gu1[12] + Gx1[65]*Gu1[15] + Gx1[66]*Gu1[18] + Gx1[67]*Gu1[21] + Gx1[68]*Gu1[24] + Gx1[69]*Gu1[27] + Gx1[70]*Gu1[30] + Gx1[71]*Gu1[33];
Gu2[16] = + Gx1[60]*Gu1[1] + Gx1[61]*Gu1[4] + Gx1[62]*Gu1[7] + Gx1[63]*Gu1[10] + Gx1[64]*Gu1[13] + Gx1[65]*Gu1[16] + Gx1[66]*Gu1[19] + Gx1[67]*Gu1[22] + Gx1[68]*Gu1[25] + Gx1[69]*Gu1[28] + Gx1[70]*Gu1[31] + Gx1[71]*Gu1[34];
Gu2[17] = + Gx1[60]*Gu1[2] + Gx1[61]*Gu1[5] + Gx1[62]*Gu1[8] + Gx1[63]*Gu1[11] + Gx1[64]*Gu1[14] + Gx1[65]*Gu1[17] + Gx1[66]*Gu1[20] + Gx1[67]*Gu1[23] + Gx1[68]*Gu1[26] + Gx1[69]*Gu1[29] + Gx1[70]*Gu1[32] + Gx1[71]*Gu1[35];
Gu2[18] = + Gx1[72]*Gu1[0] + Gx1[73]*Gu1[3] + Gx1[74]*Gu1[6] + Gx1[75]*Gu1[9] + Gx1[76]*Gu1[12] + Gx1[77]*Gu1[15] + Gx1[78]*Gu1[18] + Gx1[79]*Gu1[21] + Gx1[80]*Gu1[24] + Gx1[81]*Gu1[27] + Gx1[82]*Gu1[30] + Gx1[83]*Gu1[33];
Gu2[19] = + Gx1[72]*Gu1[1] + Gx1[73]*Gu1[4] + Gx1[74]*Gu1[7] + Gx1[75]*Gu1[10] + Gx1[76]*Gu1[13] + Gx1[77]*Gu1[16] + Gx1[78]*Gu1[19] + Gx1[79]*Gu1[22] + Gx1[80]*Gu1[25] + Gx1[81]*Gu1[28] + Gx1[82]*Gu1[31] + Gx1[83]*Gu1[34];
Gu2[20] = + Gx1[72]*Gu1[2] + Gx1[73]*Gu1[5] + Gx1[74]*Gu1[8] + Gx1[75]*Gu1[11] + Gx1[76]*Gu1[14] + Gx1[77]*Gu1[17] + Gx1[78]*Gu1[20] + Gx1[79]*Gu1[23] + Gx1[80]*Gu1[26] + Gx1[81]*Gu1[29] + Gx1[82]*Gu1[32] + Gx1[83]*Gu1[35];
Gu2[21] = + Gx1[84]*Gu1[0] + Gx1[85]*Gu1[3] + Gx1[86]*Gu1[6] + Gx1[87]*Gu1[9] + Gx1[88]*Gu1[12] + Gx1[89]*Gu1[15] + Gx1[90]*Gu1[18] + Gx1[91]*Gu1[21] + Gx1[92]*Gu1[24] + Gx1[93]*Gu1[27] + Gx1[94]*Gu1[30] + Gx1[95]*Gu1[33];
Gu2[22] = + Gx1[84]*Gu1[1] + Gx1[85]*Gu1[4] + Gx1[86]*Gu1[7] + Gx1[87]*Gu1[10] + Gx1[88]*Gu1[13] + Gx1[89]*Gu1[16] + Gx1[90]*Gu1[19] + Gx1[91]*Gu1[22] + Gx1[92]*Gu1[25] + Gx1[93]*Gu1[28] + Gx1[94]*Gu1[31] + Gx1[95]*Gu1[34];
Gu2[23] = + Gx1[84]*Gu1[2] + Gx1[85]*Gu1[5] + Gx1[86]*Gu1[8] + Gx1[87]*Gu1[11] + Gx1[88]*Gu1[14] + Gx1[89]*Gu1[17] + Gx1[90]*Gu1[20] + Gx1[91]*Gu1[23] + Gx1[92]*Gu1[26] + Gx1[93]*Gu1[29] + Gx1[94]*Gu1[32] + Gx1[95]*Gu1[35];
Gu2[24] = + Gx1[96]*Gu1[0] + Gx1[97]*Gu1[3] + Gx1[98]*Gu1[6] + Gx1[99]*Gu1[9] + Gx1[100]*Gu1[12] + Gx1[101]*Gu1[15] + Gx1[102]*Gu1[18] + Gx1[103]*Gu1[21] + Gx1[104]*Gu1[24] + Gx1[105]*Gu1[27] + Gx1[106]*Gu1[30] + Gx1[107]*Gu1[33];
Gu2[25] = + Gx1[96]*Gu1[1] + Gx1[97]*Gu1[4] + Gx1[98]*Gu1[7] + Gx1[99]*Gu1[10] + Gx1[100]*Gu1[13] + Gx1[101]*Gu1[16] + Gx1[102]*Gu1[19] + Gx1[103]*Gu1[22] + Gx1[104]*Gu1[25] + Gx1[105]*Gu1[28] + Gx1[106]*Gu1[31] + Gx1[107]*Gu1[34];
Gu2[26] = + Gx1[96]*Gu1[2] + Gx1[97]*Gu1[5] + Gx1[98]*Gu1[8] + Gx1[99]*Gu1[11] + Gx1[100]*Gu1[14] + Gx1[101]*Gu1[17] + Gx1[102]*Gu1[20] + Gx1[103]*Gu1[23] + Gx1[104]*Gu1[26] + Gx1[105]*Gu1[29] + Gx1[106]*Gu1[32] + Gx1[107]*Gu1[35];
Gu2[27] = + Gx1[108]*Gu1[0] + Gx1[109]*Gu1[3] + Gx1[110]*Gu1[6] + Gx1[111]*Gu1[9] + Gx1[112]*Gu1[12] + Gx1[113]*Gu1[15] + Gx1[114]*Gu1[18] + Gx1[115]*Gu1[21] + Gx1[116]*Gu1[24] + Gx1[117]*Gu1[27] + Gx1[118]*Gu1[30] + Gx1[119]*Gu1[33];
Gu2[28] = + Gx1[108]*Gu1[1] + Gx1[109]*Gu1[4] + Gx1[110]*Gu1[7] + Gx1[111]*Gu1[10] + Gx1[112]*Gu1[13] + Gx1[113]*Gu1[16] + Gx1[114]*Gu1[19] + Gx1[115]*Gu1[22] + Gx1[116]*Gu1[25] + Gx1[117]*Gu1[28] + Gx1[118]*Gu1[31] + Gx1[119]*Gu1[34];
Gu2[29] = + Gx1[108]*Gu1[2] + Gx1[109]*Gu1[5] + Gx1[110]*Gu1[8] + Gx1[111]*Gu1[11] + Gx1[112]*Gu1[14] + Gx1[113]*Gu1[17] + Gx1[114]*Gu1[20] + Gx1[115]*Gu1[23] + Gx1[116]*Gu1[26] + Gx1[117]*Gu1[29] + Gx1[118]*Gu1[32] + Gx1[119]*Gu1[35];
Gu2[30] = + Gx1[120]*Gu1[0] + Gx1[121]*Gu1[3] + Gx1[122]*Gu1[6] + Gx1[123]*Gu1[9] + Gx1[124]*Gu1[12] + Gx1[125]*Gu1[15] + Gx1[126]*Gu1[18] + Gx1[127]*Gu1[21] + Gx1[128]*Gu1[24] + Gx1[129]*Gu1[27] + Gx1[130]*Gu1[30] + Gx1[131]*Gu1[33];
Gu2[31] = + Gx1[120]*Gu1[1] + Gx1[121]*Gu1[4] + Gx1[122]*Gu1[7] + Gx1[123]*Gu1[10] + Gx1[124]*Gu1[13] + Gx1[125]*Gu1[16] + Gx1[126]*Gu1[19] + Gx1[127]*Gu1[22] + Gx1[128]*Gu1[25] + Gx1[129]*Gu1[28] + Gx1[130]*Gu1[31] + Gx1[131]*Gu1[34];
Gu2[32] = + Gx1[120]*Gu1[2] + Gx1[121]*Gu1[5] + Gx1[122]*Gu1[8] + Gx1[123]*Gu1[11] + Gx1[124]*Gu1[14] + Gx1[125]*Gu1[17] + Gx1[126]*Gu1[20] + Gx1[127]*Gu1[23] + Gx1[128]*Gu1[26] + Gx1[129]*Gu1[29] + Gx1[130]*Gu1[32] + Gx1[131]*Gu1[35];
Gu2[33] = + Gx1[132]*Gu1[0] + Gx1[133]*Gu1[3] + Gx1[134]*Gu1[6] + Gx1[135]*Gu1[9] + Gx1[136]*Gu1[12] + Gx1[137]*Gu1[15] + Gx1[138]*Gu1[18] + Gx1[139]*Gu1[21] + Gx1[140]*Gu1[24] + Gx1[141]*Gu1[27] + Gx1[142]*Gu1[30] + Gx1[143]*Gu1[33];
Gu2[34] = + Gx1[132]*Gu1[1] + Gx1[133]*Gu1[4] + Gx1[134]*Gu1[7] + Gx1[135]*Gu1[10] + Gx1[136]*Gu1[13] + Gx1[137]*Gu1[16] + Gx1[138]*Gu1[19] + Gx1[139]*Gu1[22] + Gx1[140]*Gu1[25] + Gx1[141]*Gu1[28] + Gx1[142]*Gu1[31] + Gx1[143]*Gu1[34];
Gu2[35] = + Gx1[132]*Gu1[2] + Gx1[133]*Gu1[5] + Gx1[134]*Gu1[8] + Gx1[135]*Gu1[11] + Gx1[136]*Gu1[14] + Gx1[137]*Gu1[17] + Gx1[138]*Gu1[20] + Gx1[139]*Gu1[23] + Gx1[140]*Gu1[26] + Gx1[141]*Gu1[29] + Gx1[142]*Gu1[32] + Gx1[143]*Gu1[35];
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
Gu2[18] = Gu1[18];
Gu2[19] = Gu1[19];
Gu2[20] = Gu1[20];
Gu2[21] = Gu1[21];
Gu2[22] = Gu1[22];
Gu2[23] = Gu1[23];
Gu2[24] = Gu1[24];
Gu2[25] = Gu1[25];
Gu2[26] = Gu1[26];
Gu2[27] = Gu1[27];
Gu2[28] = Gu1[28];
Gu2[29] = Gu1[29];
Gu2[30] = Gu1[30];
Gu2[31] = Gu1[31];
Gu2[32] = Gu1[32];
Gu2[33] = Gu1[33];
Gu2[34] = Gu1[34];
Gu2[35] = Gu1[35];
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 45) + (iCol * 3)] = + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9] + Gu1[12]*Gu2[12] + Gu1[15]*Gu2[15] + Gu1[18]*Gu2[18] + Gu1[21]*Gu2[21] + Gu1[24]*Gu2[24] + Gu1[27]*Gu2[27] + Gu1[30]*Gu2[30] + Gu1[33]*Gu2[33];
acadoWorkspace.H[(iRow * 45) + (iCol * 3 + 1)] = + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10] + Gu1[12]*Gu2[13] + Gu1[15]*Gu2[16] + Gu1[18]*Gu2[19] + Gu1[21]*Gu2[22] + Gu1[24]*Gu2[25] + Gu1[27]*Gu2[28] + Gu1[30]*Gu2[31] + Gu1[33]*Gu2[34];
acadoWorkspace.H[(iRow * 45) + (iCol * 3 + 2)] = + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11] + Gu1[12]*Gu2[14] + Gu1[15]*Gu2[17] + Gu1[18]*Gu2[20] + Gu1[21]*Gu2[23] + Gu1[24]*Gu2[26] + Gu1[27]*Gu2[29] + Gu1[30]*Gu2[32] + Gu1[33]*Gu2[35];
acadoWorkspace.H[(iRow * 45 + 15) + (iCol * 3)] = + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9] + Gu1[13]*Gu2[12] + Gu1[16]*Gu2[15] + Gu1[19]*Gu2[18] + Gu1[22]*Gu2[21] + Gu1[25]*Gu2[24] + Gu1[28]*Gu2[27] + Gu1[31]*Gu2[30] + Gu1[34]*Gu2[33];
acadoWorkspace.H[(iRow * 45 + 15) + (iCol * 3 + 1)] = + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10] + Gu1[13]*Gu2[13] + Gu1[16]*Gu2[16] + Gu1[19]*Gu2[19] + Gu1[22]*Gu2[22] + Gu1[25]*Gu2[25] + Gu1[28]*Gu2[28] + Gu1[31]*Gu2[31] + Gu1[34]*Gu2[34];
acadoWorkspace.H[(iRow * 45 + 15) + (iCol * 3 + 2)] = + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11] + Gu1[13]*Gu2[14] + Gu1[16]*Gu2[17] + Gu1[19]*Gu2[20] + Gu1[22]*Gu2[23] + Gu1[25]*Gu2[26] + Gu1[28]*Gu2[29] + Gu1[31]*Gu2[32] + Gu1[34]*Gu2[35];
acadoWorkspace.H[(iRow * 45 + 30) + (iCol * 3)] = + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9] + Gu1[14]*Gu2[12] + Gu1[17]*Gu2[15] + Gu1[20]*Gu2[18] + Gu1[23]*Gu2[21] + Gu1[26]*Gu2[24] + Gu1[29]*Gu2[27] + Gu1[32]*Gu2[30] + Gu1[35]*Gu2[33];
acadoWorkspace.H[(iRow * 45 + 30) + (iCol * 3 + 1)] = + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10] + Gu1[14]*Gu2[13] + Gu1[17]*Gu2[16] + Gu1[20]*Gu2[19] + Gu1[23]*Gu2[22] + Gu1[26]*Gu2[25] + Gu1[29]*Gu2[28] + Gu1[32]*Gu2[31] + Gu1[35]*Gu2[34];
acadoWorkspace.H[(iRow * 45 + 30) + (iCol * 3 + 2)] = + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11] + Gu1[14]*Gu2[14] + Gu1[17]*Gu2[17] + Gu1[20]*Gu2[20] + Gu1[23]*Gu2[23] + Gu1[26]*Gu2[26] + Gu1[29]*Gu2[29] + Gu1[32]*Gu2[32] + Gu1[35]*Gu2[35];
}

void acado_mac_S1T_E( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 45) + (iCol * 3)] += + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9] + Gu1[12]*Gu2[12] + Gu1[15]*Gu2[15] + Gu1[18]*Gu2[18] + Gu1[21]*Gu2[21] + Gu1[24]*Gu2[24] + Gu1[27]*Gu2[27] + Gu1[30]*Gu2[30] + Gu1[33]*Gu2[33];
acadoWorkspace.H[(iRow * 45) + (iCol * 3 + 1)] += + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10] + Gu1[12]*Gu2[13] + Gu1[15]*Gu2[16] + Gu1[18]*Gu2[19] + Gu1[21]*Gu2[22] + Gu1[24]*Gu2[25] + Gu1[27]*Gu2[28] + Gu1[30]*Gu2[31] + Gu1[33]*Gu2[34];
acadoWorkspace.H[(iRow * 45) + (iCol * 3 + 2)] += + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11] + Gu1[12]*Gu2[14] + Gu1[15]*Gu2[17] + Gu1[18]*Gu2[20] + Gu1[21]*Gu2[23] + Gu1[24]*Gu2[26] + Gu1[27]*Gu2[29] + Gu1[30]*Gu2[32] + Gu1[33]*Gu2[35];
acadoWorkspace.H[(iRow * 45 + 15) + (iCol * 3)] += + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9] + Gu1[13]*Gu2[12] + Gu1[16]*Gu2[15] + Gu1[19]*Gu2[18] + Gu1[22]*Gu2[21] + Gu1[25]*Gu2[24] + Gu1[28]*Gu2[27] + Gu1[31]*Gu2[30] + Gu1[34]*Gu2[33];
acadoWorkspace.H[(iRow * 45 + 15) + (iCol * 3 + 1)] += + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10] + Gu1[13]*Gu2[13] + Gu1[16]*Gu2[16] + Gu1[19]*Gu2[19] + Gu1[22]*Gu2[22] + Gu1[25]*Gu2[25] + Gu1[28]*Gu2[28] + Gu1[31]*Gu2[31] + Gu1[34]*Gu2[34];
acadoWorkspace.H[(iRow * 45 + 15) + (iCol * 3 + 2)] += + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11] + Gu1[13]*Gu2[14] + Gu1[16]*Gu2[17] + Gu1[19]*Gu2[20] + Gu1[22]*Gu2[23] + Gu1[25]*Gu2[26] + Gu1[28]*Gu2[29] + Gu1[31]*Gu2[32] + Gu1[34]*Gu2[35];
acadoWorkspace.H[(iRow * 45 + 30) + (iCol * 3)] += + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9] + Gu1[14]*Gu2[12] + Gu1[17]*Gu2[15] + Gu1[20]*Gu2[18] + Gu1[23]*Gu2[21] + Gu1[26]*Gu2[24] + Gu1[29]*Gu2[27] + Gu1[32]*Gu2[30] + Gu1[35]*Gu2[33];
acadoWorkspace.H[(iRow * 45 + 30) + (iCol * 3 + 1)] += + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10] + Gu1[14]*Gu2[13] + Gu1[17]*Gu2[16] + Gu1[20]*Gu2[19] + Gu1[23]*Gu2[22] + Gu1[26]*Gu2[25] + Gu1[29]*Gu2[28] + Gu1[32]*Gu2[31] + Gu1[35]*Gu2[34];
acadoWorkspace.H[(iRow * 45 + 30) + (iCol * 3 + 2)] += + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11] + Gu1[14]*Gu2[14] + Gu1[17]*Gu2[17] + Gu1[20]*Gu2[20] + Gu1[23]*Gu2[23] + Gu1[26]*Gu2[26] + Gu1[29]*Gu2[29] + Gu1[32]*Gu2[32] + Gu1[35]*Gu2[35];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 48] = + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9] + Gu1[12]*Gu2[12] + Gu1[15]*Gu2[15] + Gu1[18]*Gu2[18] + Gu1[21]*Gu2[21] + Gu1[24]*Gu2[24] + Gu1[27]*Gu2[27] + Gu1[30]*Gu2[30] + Gu1[33]*Gu2[33] + R11[0];
acadoWorkspace.H[iRow * 48 + 1] = + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10] + Gu1[12]*Gu2[13] + Gu1[15]*Gu2[16] + Gu1[18]*Gu2[19] + Gu1[21]*Gu2[22] + Gu1[24]*Gu2[25] + Gu1[27]*Gu2[28] + Gu1[30]*Gu2[31] + Gu1[33]*Gu2[34] + R11[1];
acadoWorkspace.H[iRow * 48 + 2] = + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11] + Gu1[12]*Gu2[14] + Gu1[15]*Gu2[17] + Gu1[18]*Gu2[20] + Gu1[21]*Gu2[23] + Gu1[24]*Gu2[26] + Gu1[27]*Gu2[29] + Gu1[30]*Gu2[32] + Gu1[33]*Gu2[35] + R11[2];
acadoWorkspace.H[iRow * 48 + 15] = + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9] + Gu1[13]*Gu2[12] + Gu1[16]*Gu2[15] + Gu1[19]*Gu2[18] + Gu1[22]*Gu2[21] + Gu1[25]*Gu2[24] + Gu1[28]*Gu2[27] + Gu1[31]*Gu2[30] + Gu1[34]*Gu2[33] + R11[3];
acadoWorkspace.H[iRow * 48 + 16] = + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10] + Gu1[13]*Gu2[13] + Gu1[16]*Gu2[16] + Gu1[19]*Gu2[19] + Gu1[22]*Gu2[22] + Gu1[25]*Gu2[25] + Gu1[28]*Gu2[28] + Gu1[31]*Gu2[31] + Gu1[34]*Gu2[34] + R11[4];
acadoWorkspace.H[iRow * 48 + 17] = + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11] + Gu1[13]*Gu2[14] + Gu1[16]*Gu2[17] + Gu1[19]*Gu2[20] + Gu1[22]*Gu2[23] + Gu1[25]*Gu2[26] + Gu1[28]*Gu2[29] + Gu1[31]*Gu2[32] + Gu1[34]*Gu2[35] + R11[5];
acadoWorkspace.H[iRow * 48 + 30] = + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9] + Gu1[14]*Gu2[12] + Gu1[17]*Gu2[15] + Gu1[20]*Gu2[18] + Gu1[23]*Gu2[21] + Gu1[26]*Gu2[24] + Gu1[29]*Gu2[27] + Gu1[32]*Gu2[30] + Gu1[35]*Gu2[33] + R11[6];
acadoWorkspace.H[iRow * 48 + 31] = + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10] + Gu1[14]*Gu2[13] + Gu1[17]*Gu2[16] + Gu1[20]*Gu2[19] + Gu1[23]*Gu2[22] + Gu1[26]*Gu2[25] + Gu1[29]*Gu2[28] + Gu1[32]*Gu2[31] + Gu1[35]*Gu2[34] + R11[7];
acadoWorkspace.H[iRow * 48 + 32] = + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11] + Gu1[14]*Gu2[14] + Gu1[17]*Gu2[17] + Gu1[20]*Gu2[20] + Gu1[23]*Gu2[23] + Gu1[26]*Gu2[26] + Gu1[29]*Gu2[29] + Gu1[32]*Gu2[32] + Gu1[35]*Gu2[35] + R11[8];
acadoWorkspace.H[iRow * 48] += 1.0000000000000000e-10;
acadoWorkspace.H[iRow * 48 + 16] += 1.0000000000000000e-10;
acadoWorkspace.H[iRow * 48 + 32] += 1.0000000000000000e-10;
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[12]*Gu1[3] + Gx1[24]*Gu1[6] + Gx1[36]*Gu1[9] + Gx1[48]*Gu1[12] + Gx1[60]*Gu1[15] + Gx1[72]*Gu1[18] + Gx1[84]*Gu1[21] + Gx1[96]*Gu1[24] + Gx1[108]*Gu1[27] + Gx1[120]*Gu1[30] + Gx1[132]*Gu1[33];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[12]*Gu1[4] + Gx1[24]*Gu1[7] + Gx1[36]*Gu1[10] + Gx1[48]*Gu1[13] + Gx1[60]*Gu1[16] + Gx1[72]*Gu1[19] + Gx1[84]*Gu1[22] + Gx1[96]*Gu1[25] + Gx1[108]*Gu1[28] + Gx1[120]*Gu1[31] + Gx1[132]*Gu1[34];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[12]*Gu1[5] + Gx1[24]*Gu1[8] + Gx1[36]*Gu1[11] + Gx1[48]*Gu1[14] + Gx1[60]*Gu1[17] + Gx1[72]*Gu1[20] + Gx1[84]*Gu1[23] + Gx1[96]*Gu1[26] + Gx1[108]*Gu1[29] + Gx1[120]*Gu1[32] + Gx1[132]*Gu1[35];
Gu2[3] = + Gx1[1]*Gu1[0] + Gx1[13]*Gu1[3] + Gx1[25]*Gu1[6] + Gx1[37]*Gu1[9] + Gx1[49]*Gu1[12] + Gx1[61]*Gu1[15] + Gx1[73]*Gu1[18] + Gx1[85]*Gu1[21] + Gx1[97]*Gu1[24] + Gx1[109]*Gu1[27] + Gx1[121]*Gu1[30] + Gx1[133]*Gu1[33];
Gu2[4] = + Gx1[1]*Gu1[1] + Gx1[13]*Gu1[4] + Gx1[25]*Gu1[7] + Gx1[37]*Gu1[10] + Gx1[49]*Gu1[13] + Gx1[61]*Gu1[16] + Gx1[73]*Gu1[19] + Gx1[85]*Gu1[22] + Gx1[97]*Gu1[25] + Gx1[109]*Gu1[28] + Gx1[121]*Gu1[31] + Gx1[133]*Gu1[34];
Gu2[5] = + Gx1[1]*Gu1[2] + Gx1[13]*Gu1[5] + Gx1[25]*Gu1[8] + Gx1[37]*Gu1[11] + Gx1[49]*Gu1[14] + Gx1[61]*Gu1[17] + Gx1[73]*Gu1[20] + Gx1[85]*Gu1[23] + Gx1[97]*Gu1[26] + Gx1[109]*Gu1[29] + Gx1[121]*Gu1[32] + Gx1[133]*Gu1[35];
Gu2[6] = + Gx1[2]*Gu1[0] + Gx1[14]*Gu1[3] + Gx1[26]*Gu1[6] + Gx1[38]*Gu1[9] + Gx1[50]*Gu1[12] + Gx1[62]*Gu1[15] + Gx1[74]*Gu1[18] + Gx1[86]*Gu1[21] + Gx1[98]*Gu1[24] + Gx1[110]*Gu1[27] + Gx1[122]*Gu1[30] + Gx1[134]*Gu1[33];
Gu2[7] = + Gx1[2]*Gu1[1] + Gx1[14]*Gu1[4] + Gx1[26]*Gu1[7] + Gx1[38]*Gu1[10] + Gx1[50]*Gu1[13] + Gx1[62]*Gu1[16] + Gx1[74]*Gu1[19] + Gx1[86]*Gu1[22] + Gx1[98]*Gu1[25] + Gx1[110]*Gu1[28] + Gx1[122]*Gu1[31] + Gx1[134]*Gu1[34];
Gu2[8] = + Gx1[2]*Gu1[2] + Gx1[14]*Gu1[5] + Gx1[26]*Gu1[8] + Gx1[38]*Gu1[11] + Gx1[50]*Gu1[14] + Gx1[62]*Gu1[17] + Gx1[74]*Gu1[20] + Gx1[86]*Gu1[23] + Gx1[98]*Gu1[26] + Gx1[110]*Gu1[29] + Gx1[122]*Gu1[32] + Gx1[134]*Gu1[35];
Gu2[9] = + Gx1[3]*Gu1[0] + Gx1[15]*Gu1[3] + Gx1[27]*Gu1[6] + Gx1[39]*Gu1[9] + Gx1[51]*Gu1[12] + Gx1[63]*Gu1[15] + Gx1[75]*Gu1[18] + Gx1[87]*Gu1[21] + Gx1[99]*Gu1[24] + Gx1[111]*Gu1[27] + Gx1[123]*Gu1[30] + Gx1[135]*Gu1[33];
Gu2[10] = + Gx1[3]*Gu1[1] + Gx1[15]*Gu1[4] + Gx1[27]*Gu1[7] + Gx1[39]*Gu1[10] + Gx1[51]*Gu1[13] + Gx1[63]*Gu1[16] + Gx1[75]*Gu1[19] + Gx1[87]*Gu1[22] + Gx1[99]*Gu1[25] + Gx1[111]*Gu1[28] + Gx1[123]*Gu1[31] + Gx1[135]*Gu1[34];
Gu2[11] = + Gx1[3]*Gu1[2] + Gx1[15]*Gu1[5] + Gx1[27]*Gu1[8] + Gx1[39]*Gu1[11] + Gx1[51]*Gu1[14] + Gx1[63]*Gu1[17] + Gx1[75]*Gu1[20] + Gx1[87]*Gu1[23] + Gx1[99]*Gu1[26] + Gx1[111]*Gu1[29] + Gx1[123]*Gu1[32] + Gx1[135]*Gu1[35];
Gu2[12] = + Gx1[4]*Gu1[0] + Gx1[16]*Gu1[3] + Gx1[28]*Gu1[6] + Gx1[40]*Gu1[9] + Gx1[52]*Gu1[12] + Gx1[64]*Gu1[15] + Gx1[76]*Gu1[18] + Gx1[88]*Gu1[21] + Gx1[100]*Gu1[24] + Gx1[112]*Gu1[27] + Gx1[124]*Gu1[30] + Gx1[136]*Gu1[33];
Gu2[13] = + Gx1[4]*Gu1[1] + Gx1[16]*Gu1[4] + Gx1[28]*Gu1[7] + Gx1[40]*Gu1[10] + Gx1[52]*Gu1[13] + Gx1[64]*Gu1[16] + Gx1[76]*Gu1[19] + Gx1[88]*Gu1[22] + Gx1[100]*Gu1[25] + Gx1[112]*Gu1[28] + Gx1[124]*Gu1[31] + Gx1[136]*Gu1[34];
Gu2[14] = + Gx1[4]*Gu1[2] + Gx1[16]*Gu1[5] + Gx1[28]*Gu1[8] + Gx1[40]*Gu1[11] + Gx1[52]*Gu1[14] + Gx1[64]*Gu1[17] + Gx1[76]*Gu1[20] + Gx1[88]*Gu1[23] + Gx1[100]*Gu1[26] + Gx1[112]*Gu1[29] + Gx1[124]*Gu1[32] + Gx1[136]*Gu1[35];
Gu2[15] = + Gx1[5]*Gu1[0] + Gx1[17]*Gu1[3] + Gx1[29]*Gu1[6] + Gx1[41]*Gu1[9] + Gx1[53]*Gu1[12] + Gx1[65]*Gu1[15] + Gx1[77]*Gu1[18] + Gx1[89]*Gu1[21] + Gx1[101]*Gu1[24] + Gx1[113]*Gu1[27] + Gx1[125]*Gu1[30] + Gx1[137]*Gu1[33];
Gu2[16] = + Gx1[5]*Gu1[1] + Gx1[17]*Gu1[4] + Gx1[29]*Gu1[7] + Gx1[41]*Gu1[10] + Gx1[53]*Gu1[13] + Gx1[65]*Gu1[16] + Gx1[77]*Gu1[19] + Gx1[89]*Gu1[22] + Gx1[101]*Gu1[25] + Gx1[113]*Gu1[28] + Gx1[125]*Gu1[31] + Gx1[137]*Gu1[34];
Gu2[17] = + Gx1[5]*Gu1[2] + Gx1[17]*Gu1[5] + Gx1[29]*Gu1[8] + Gx1[41]*Gu1[11] + Gx1[53]*Gu1[14] + Gx1[65]*Gu1[17] + Gx1[77]*Gu1[20] + Gx1[89]*Gu1[23] + Gx1[101]*Gu1[26] + Gx1[113]*Gu1[29] + Gx1[125]*Gu1[32] + Gx1[137]*Gu1[35];
Gu2[18] = + Gx1[6]*Gu1[0] + Gx1[18]*Gu1[3] + Gx1[30]*Gu1[6] + Gx1[42]*Gu1[9] + Gx1[54]*Gu1[12] + Gx1[66]*Gu1[15] + Gx1[78]*Gu1[18] + Gx1[90]*Gu1[21] + Gx1[102]*Gu1[24] + Gx1[114]*Gu1[27] + Gx1[126]*Gu1[30] + Gx1[138]*Gu1[33];
Gu2[19] = + Gx1[6]*Gu1[1] + Gx1[18]*Gu1[4] + Gx1[30]*Gu1[7] + Gx1[42]*Gu1[10] + Gx1[54]*Gu1[13] + Gx1[66]*Gu1[16] + Gx1[78]*Gu1[19] + Gx1[90]*Gu1[22] + Gx1[102]*Gu1[25] + Gx1[114]*Gu1[28] + Gx1[126]*Gu1[31] + Gx1[138]*Gu1[34];
Gu2[20] = + Gx1[6]*Gu1[2] + Gx1[18]*Gu1[5] + Gx1[30]*Gu1[8] + Gx1[42]*Gu1[11] + Gx1[54]*Gu1[14] + Gx1[66]*Gu1[17] + Gx1[78]*Gu1[20] + Gx1[90]*Gu1[23] + Gx1[102]*Gu1[26] + Gx1[114]*Gu1[29] + Gx1[126]*Gu1[32] + Gx1[138]*Gu1[35];
Gu2[21] = + Gx1[7]*Gu1[0] + Gx1[19]*Gu1[3] + Gx1[31]*Gu1[6] + Gx1[43]*Gu1[9] + Gx1[55]*Gu1[12] + Gx1[67]*Gu1[15] + Gx1[79]*Gu1[18] + Gx1[91]*Gu1[21] + Gx1[103]*Gu1[24] + Gx1[115]*Gu1[27] + Gx1[127]*Gu1[30] + Gx1[139]*Gu1[33];
Gu2[22] = + Gx1[7]*Gu1[1] + Gx1[19]*Gu1[4] + Gx1[31]*Gu1[7] + Gx1[43]*Gu1[10] + Gx1[55]*Gu1[13] + Gx1[67]*Gu1[16] + Gx1[79]*Gu1[19] + Gx1[91]*Gu1[22] + Gx1[103]*Gu1[25] + Gx1[115]*Gu1[28] + Gx1[127]*Gu1[31] + Gx1[139]*Gu1[34];
Gu2[23] = + Gx1[7]*Gu1[2] + Gx1[19]*Gu1[5] + Gx1[31]*Gu1[8] + Gx1[43]*Gu1[11] + Gx1[55]*Gu1[14] + Gx1[67]*Gu1[17] + Gx1[79]*Gu1[20] + Gx1[91]*Gu1[23] + Gx1[103]*Gu1[26] + Gx1[115]*Gu1[29] + Gx1[127]*Gu1[32] + Gx1[139]*Gu1[35];
Gu2[24] = + Gx1[8]*Gu1[0] + Gx1[20]*Gu1[3] + Gx1[32]*Gu1[6] + Gx1[44]*Gu1[9] + Gx1[56]*Gu1[12] + Gx1[68]*Gu1[15] + Gx1[80]*Gu1[18] + Gx1[92]*Gu1[21] + Gx1[104]*Gu1[24] + Gx1[116]*Gu1[27] + Gx1[128]*Gu1[30] + Gx1[140]*Gu1[33];
Gu2[25] = + Gx1[8]*Gu1[1] + Gx1[20]*Gu1[4] + Gx1[32]*Gu1[7] + Gx1[44]*Gu1[10] + Gx1[56]*Gu1[13] + Gx1[68]*Gu1[16] + Gx1[80]*Gu1[19] + Gx1[92]*Gu1[22] + Gx1[104]*Gu1[25] + Gx1[116]*Gu1[28] + Gx1[128]*Gu1[31] + Gx1[140]*Gu1[34];
Gu2[26] = + Gx1[8]*Gu1[2] + Gx1[20]*Gu1[5] + Gx1[32]*Gu1[8] + Gx1[44]*Gu1[11] + Gx1[56]*Gu1[14] + Gx1[68]*Gu1[17] + Gx1[80]*Gu1[20] + Gx1[92]*Gu1[23] + Gx1[104]*Gu1[26] + Gx1[116]*Gu1[29] + Gx1[128]*Gu1[32] + Gx1[140]*Gu1[35];
Gu2[27] = + Gx1[9]*Gu1[0] + Gx1[21]*Gu1[3] + Gx1[33]*Gu1[6] + Gx1[45]*Gu1[9] + Gx1[57]*Gu1[12] + Gx1[69]*Gu1[15] + Gx1[81]*Gu1[18] + Gx1[93]*Gu1[21] + Gx1[105]*Gu1[24] + Gx1[117]*Gu1[27] + Gx1[129]*Gu1[30] + Gx1[141]*Gu1[33];
Gu2[28] = + Gx1[9]*Gu1[1] + Gx1[21]*Gu1[4] + Gx1[33]*Gu1[7] + Gx1[45]*Gu1[10] + Gx1[57]*Gu1[13] + Gx1[69]*Gu1[16] + Gx1[81]*Gu1[19] + Gx1[93]*Gu1[22] + Gx1[105]*Gu1[25] + Gx1[117]*Gu1[28] + Gx1[129]*Gu1[31] + Gx1[141]*Gu1[34];
Gu2[29] = + Gx1[9]*Gu1[2] + Gx1[21]*Gu1[5] + Gx1[33]*Gu1[8] + Gx1[45]*Gu1[11] + Gx1[57]*Gu1[14] + Gx1[69]*Gu1[17] + Gx1[81]*Gu1[20] + Gx1[93]*Gu1[23] + Gx1[105]*Gu1[26] + Gx1[117]*Gu1[29] + Gx1[129]*Gu1[32] + Gx1[141]*Gu1[35];
Gu2[30] = + Gx1[10]*Gu1[0] + Gx1[22]*Gu1[3] + Gx1[34]*Gu1[6] + Gx1[46]*Gu1[9] + Gx1[58]*Gu1[12] + Gx1[70]*Gu1[15] + Gx1[82]*Gu1[18] + Gx1[94]*Gu1[21] + Gx1[106]*Gu1[24] + Gx1[118]*Gu1[27] + Gx1[130]*Gu1[30] + Gx1[142]*Gu1[33];
Gu2[31] = + Gx1[10]*Gu1[1] + Gx1[22]*Gu1[4] + Gx1[34]*Gu1[7] + Gx1[46]*Gu1[10] + Gx1[58]*Gu1[13] + Gx1[70]*Gu1[16] + Gx1[82]*Gu1[19] + Gx1[94]*Gu1[22] + Gx1[106]*Gu1[25] + Gx1[118]*Gu1[28] + Gx1[130]*Gu1[31] + Gx1[142]*Gu1[34];
Gu2[32] = + Gx1[10]*Gu1[2] + Gx1[22]*Gu1[5] + Gx1[34]*Gu1[8] + Gx1[46]*Gu1[11] + Gx1[58]*Gu1[14] + Gx1[70]*Gu1[17] + Gx1[82]*Gu1[20] + Gx1[94]*Gu1[23] + Gx1[106]*Gu1[26] + Gx1[118]*Gu1[29] + Gx1[130]*Gu1[32] + Gx1[142]*Gu1[35];
Gu2[33] = + Gx1[11]*Gu1[0] + Gx1[23]*Gu1[3] + Gx1[35]*Gu1[6] + Gx1[47]*Gu1[9] + Gx1[59]*Gu1[12] + Gx1[71]*Gu1[15] + Gx1[83]*Gu1[18] + Gx1[95]*Gu1[21] + Gx1[107]*Gu1[24] + Gx1[119]*Gu1[27] + Gx1[131]*Gu1[30] + Gx1[143]*Gu1[33];
Gu2[34] = + Gx1[11]*Gu1[1] + Gx1[23]*Gu1[4] + Gx1[35]*Gu1[7] + Gx1[47]*Gu1[10] + Gx1[59]*Gu1[13] + Gx1[71]*Gu1[16] + Gx1[83]*Gu1[19] + Gx1[95]*Gu1[22] + Gx1[107]*Gu1[25] + Gx1[119]*Gu1[28] + Gx1[131]*Gu1[31] + Gx1[143]*Gu1[34];
Gu2[35] = + Gx1[11]*Gu1[2] + Gx1[23]*Gu1[5] + Gx1[35]*Gu1[8] + Gx1[47]*Gu1[11] + Gx1[59]*Gu1[14] + Gx1[71]*Gu1[17] + Gx1[83]*Gu1[20] + Gx1[95]*Gu1[23] + Gx1[107]*Gu1[26] + Gx1[119]*Gu1[29] + Gx1[131]*Gu1[32] + Gx1[143]*Gu1[35];
}

void acado_multQEW2( real_t* const Q11, real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + Q11[0]*Gu1[0] + Q11[1]*Gu1[3] + Q11[2]*Gu1[6] + Q11[3]*Gu1[9] + Q11[4]*Gu1[12] + Q11[5]*Gu1[15] + Q11[6]*Gu1[18] + Q11[7]*Gu1[21] + Q11[8]*Gu1[24] + Q11[9]*Gu1[27] + Q11[10]*Gu1[30] + Q11[11]*Gu1[33] + Gu2[0];
Gu3[1] = + Q11[0]*Gu1[1] + Q11[1]*Gu1[4] + Q11[2]*Gu1[7] + Q11[3]*Gu1[10] + Q11[4]*Gu1[13] + Q11[5]*Gu1[16] + Q11[6]*Gu1[19] + Q11[7]*Gu1[22] + Q11[8]*Gu1[25] + Q11[9]*Gu1[28] + Q11[10]*Gu1[31] + Q11[11]*Gu1[34] + Gu2[1];
Gu3[2] = + Q11[0]*Gu1[2] + Q11[1]*Gu1[5] + Q11[2]*Gu1[8] + Q11[3]*Gu1[11] + Q11[4]*Gu1[14] + Q11[5]*Gu1[17] + Q11[6]*Gu1[20] + Q11[7]*Gu1[23] + Q11[8]*Gu1[26] + Q11[9]*Gu1[29] + Q11[10]*Gu1[32] + Q11[11]*Gu1[35] + Gu2[2];
Gu3[3] = + Q11[12]*Gu1[0] + Q11[13]*Gu1[3] + Q11[14]*Gu1[6] + Q11[15]*Gu1[9] + Q11[16]*Gu1[12] + Q11[17]*Gu1[15] + Q11[18]*Gu1[18] + Q11[19]*Gu1[21] + Q11[20]*Gu1[24] + Q11[21]*Gu1[27] + Q11[22]*Gu1[30] + Q11[23]*Gu1[33] + Gu2[3];
Gu3[4] = + Q11[12]*Gu1[1] + Q11[13]*Gu1[4] + Q11[14]*Gu1[7] + Q11[15]*Gu1[10] + Q11[16]*Gu1[13] + Q11[17]*Gu1[16] + Q11[18]*Gu1[19] + Q11[19]*Gu1[22] + Q11[20]*Gu1[25] + Q11[21]*Gu1[28] + Q11[22]*Gu1[31] + Q11[23]*Gu1[34] + Gu2[4];
Gu3[5] = + Q11[12]*Gu1[2] + Q11[13]*Gu1[5] + Q11[14]*Gu1[8] + Q11[15]*Gu1[11] + Q11[16]*Gu1[14] + Q11[17]*Gu1[17] + Q11[18]*Gu1[20] + Q11[19]*Gu1[23] + Q11[20]*Gu1[26] + Q11[21]*Gu1[29] + Q11[22]*Gu1[32] + Q11[23]*Gu1[35] + Gu2[5];
Gu3[6] = + Q11[24]*Gu1[0] + Q11[25]*Gu1[3] + Q11[26]*Gu1[6] + Q11[27]*Gu1[9] + Q11[28]*Gu1[12] + Q11[29]*Gu1[15] + Q11[30]*Gu1[18] + Q11[31]*Gu1[21] + Q11[32]*Gu1[24] + Q11[33]*Gu1[27] + Q11[34]*Gu1[30] + Q11[35]*Gu1[33] + Gu2[6];
Gu3[7] = + Q11[24]*Gu1[1] + Q11[25]*Gu1[4] + Q11[26]*Gu1[7] + Q11[27]*Gu1[10] + Q11[28]*Gu1[13] + Q11[29]*Gu1[16] + Q11[30]*Gu1[19] + Q11[31]*Gu1[22] + Q11[32]*Gu1[25] + Q11[33]*Gu1[28] + Q11[34]*Gu1[31] + Q11[35]*Gu1[34] + Gu2[7];
Gu3[8] = + Q11[24]*Gu1[2] + Q11[25]*Gu1[5] + Q11[26]*Gu1[8] + Q11[27]*Gu1[11] + Q11[28]*Gu1[14] + Q11[29]*Gu1[17] + Q11[30]*Gu1[20] + Q11[31]*Gu1[23] + Q11[32]*Gu1[26] + Q11[33]*Gu1[29] + Q11[34]*Gu1[32] + Q11[35]*Gu1[35] + Gu2[8];
Gu3[9] = + Q11[36]*Gu1[0] + Q11[37]*Gu1[3] + Q11[38]*Gu1[6] + Q11[39]*Gu1[9] + Q11[40]*Gu1[12] + Q11[41]*Gu1[15] + Q11[42]*Gu1[18] + Q11[43]*Gu1[21] + Q11[44]*Gu1[24] + Q11[45]*Gu1[27] + Q11[46]*Gu1[30] + Q11[47]*Gu1[33] + Gu2[9];
Gu3[10] = + Q11[36]*Gu1[1] + Q11[37]*Gu1[4] + Q11[38]*Gu1[7] + Q11[39]*Gu1[10] + Q11[40]*Gu1[13] + Q11[41]*Gu1[16] + Q11[42]*Gu1[19] + Q11[43]*Gu1[22] + Q11[44]*Gu1[25] + Q11[45]*Gu1[28] + Q11[46]*Gu1[31] + Q11[47]*Gu1[34] + Gu2[10];
Gu3[11] = + Q11[36]*Gu1[2] + Q11[37]*Gu1[5] + Q11[38]*Gu1[8] + Q11[39]*Gu1[11] + Q11[40]*Gu1[14] + Q11[41]*Gu1[17] + Q11[42]*Gu1[20] + Q11[43]*Gu1[23] + Q11[44]*Gu1[26] + Q11[45]*Gu1[29] + Q11[46]*Gu1[32] + Q11[47]*Gu1[35] + Gu2[11];
Gu3[12] = + Q11[48]*Gu1[0] + Q11[49]*Gu1[3] + Q11[50]*Gu1[6] + Q11[51]*Gu1[9] + Q11[52]*Gu1[12] + Q11[53]*Gu1[15] + Q11[54]*Gu1[18] + Q11[55]*Gu1[21] + Q11[56]*Gu1[24] + Q11[57]*Gu1[27] + Q11[58]*Gu1[30] + Q11[59]*Gu1[33] + Gu2[12];
Gu3[13] = + Q11[48]*Gu1[1] + Q11[49]*Gu1[4] + Q11[50]*Gu1[7] + Q11[51]*Gu1[10] + Q11[52]*Gu1[13] + Q11[53]*Gu1[16] + Q11[54]*Gu1[19] + Q11[55]*Gu1[22] + Q11[56]*Gu1[25] + Q11[57]*Gu1[28] + Q11[58]*Gu1[31] + Q11[59]*Gu1[34] + Gu2[13];
Gu3[14] = + Q11[48]*Gu1[2] + Q11[49]*Gu1[5] + Q11[50]*Gu1[8] + Q11[51]*Gu1[11] + Q11[52]*Gu1[14] + Q11[53]*Gu1[17] + Q11[54]*Gu1[20] + Q11[55]*Gu1[23] + Q11[56]*Gu1[26] + Q11[57]*Gu1[29] + Q11[58]*Gu1[32] + Q11[59]*Gu1[35] + Gu2[14];
Gu3[15] = + Q11[60]*Gu1[0] + Q11[61]*Gu1[3] + Q11[62]*Gu1[6] + Q11[63]*Gu1[9] + Q11[64]*Gu1[12] + Q11[65]*Gu1[15] + Q11[66]*Gu1[18] + Q11[67]*Gu1[21] + Q11[68]*Gu1[24] + Q11[69]*Gu1[27] + Q11[70]*Gu1[30] + Q11[71]*Gu1[33] + Gu2[15];
Gu3[16] = + Q11[60]*Gu1[1] + Q11[61]*Gu1[4] + Q11[62]*Gu1[7] + Q11[63]*Gu1[10] + Q11[64]*Gu1[13] + Q11[65]*Gu1[16] + Q11[66]*Gu1[19] + Q11[67]*Gu1[22] + Q11[68]*Gu1[25] + Q11[69]*Gu1[28] + Q11[70]*Gu1[31] + Q11[71]*Gu1[34] + Gu2[16];
Gu3[17] = + Q11[60]*Gu1[2] + Q11[61]*Gu1[5] + Q11[62]*Gu1[8] + Q11[63]*Gu1[11] + Q11[64]*Gu1[14] + Q11[65]*Gu1[17] + Q11[66]*Gu1[20] + Q11[67]*Gu1[23] + Q11[68]*Gu1[26] + Q11[69]*Gu1[29] + Q11[70]*Gu1[32] + Q11[71]*Gu1[35] + Gu2[17];
Gu3[18] = + Q11[72]*Gu1[0] + Q11[73]*Gu1[3] + Q11[74]*Gu1[6] + Q11[75]*Gu1[9] + Q11[76]*Gu1[12] + Q11[77]*Gu1[15] + Q11[78]*Gu1[18] + Q11[79]*Gu1[21] + Q11[80]*Gu1[24] + Q11[81]*Gu1[27] + Q11[82]*Gu1[30] + Q11[83]*Gu1[33] + Gu2[18];
Gu3[19] = + Q11[72]*Gu1[1] + Q11[73]*Gu1[4] + Q11[74]*Gu1[7] + Q11[75]*Gu1[10] + Q11[76]*Gu1[13] + Q11[77]*Gu1[16] + Q11[78]*Gu1[19] + Q11[79]*Gu1[22] + Q11[80]*Gu1[25] + Q11[81]*Gu1[28] + Q11[82]*Gu1[31] + Q11[83]*Gu1[34] + Gu2[19];
Gu3[20] = + Q11[72]*Gu1[2] + Q11[73]*Gu1[5] + Q11[74]*Gu1[8] + Q11[75]*Gu1[11] + Q11[76]*Gu1[14] + Q11[77]*Gu1[17] + Q11[78]*Gu1[20] + Q11[79]*Gu1[23] + Q11[80]*Gu1[26] + Q11[81]*Gu1[29] + Q11[82]*Gu1[32] + Q11[83]*Gu1[35] + Gu2[20];
Gu3[21] = + Q11[84]*Gu1[0] + Q11[85]*Gu1[3] + Q11[86]*Gu1[6] + Q11[87]*Gu1[9] + Q11[88]*Gu1[12] + Q11[89]*Gu1[15] + Q11[90]*Gu1[18] + Q11[91]*Gu1[21] + Q11[92]*Gu1[24] + Q11[93]*Gu1[27] + Q11[94]*Gu1[30] + Q11[95]*Gu1[33] + Gu2[21];
Gu3[22] = + Q11[84]*Gu1[1] + Q11[85]*Gu1[4] + Q11[86]*Gu1[7] + Q11[87]*Gu1[10] + Q11[88]*Gu1[13] + Q11[89]*Gu1[16] + Q11[90]*Gu1[19] + Q11[91]*Gu1[22] + Q11[92]*Gu1[25] + Q11[93]*Gu1[28] + Q11[94]*Gu1[31] + Q11[95]*Gu1[34] + Gu2[22];
Gu3[23] = + Q11[84]*Gu1[2] + Q11[85]*Gu1[5] + Q11[86]*Gu1[8] + Q11[87]*Gu1[11] + Q11[88]*Gu1[14] + Q11[89]*Gu1[17] + Q11[90]*Gu1[20] + Q11[91]*Gu1[23] + Q11[92]*Gu1[26] + Q11[93]*Gu1[29] + Q11[94]*Gu1[32] + Q11[95]*Gu1[35] + Gu2[23];
Gu3[24] = + Q11[96]*Gu1[0] + Q11[97]*Gu1[3] + Q11[98]*Gu1[6] + Q11[99]*Gu1[9] + Q11[100]*Gu1[12] + Q11[101]*Gu1[15] + Q11[102]*Gu1[18] + Q11[103]*Gu1[21] + Q11[104]*Gu1[24] + Q11[105]*Gu1[27] + Q11[106]*Gu1[30] + Q11[107]*Gu1[33] + Gu2[24];
Gu3[25] = + Q11[96]*Gu1[1] + Q11[97]*Gu1[4] + Q11[98]*Gu1[7] + Q11[99]*Gu1[10] + Q11[100]*Gu1[13] + Q11[101]*Gu1[16] + Q11[102]*Gu1[19] + Q11[103]*Gu1[22] + Q11[104]*Gu1[25] + Q11[105]*Gu1[28] + Q11[106]*Gu1[31] + Q11[107]*Gu1[34] + Gu2[25];
Gu3[26] = + Q11[96]*Gu1[2] + Q11[97]*Gu1[5] + Q11[98]*Gu1[8] + Q11[99]*Gu1[11] + Q11[100]*Gu1[14] + Q11[101]*Gu1[17] + Q11[102]*Gu1[20] + Q11[103]*Gu1[23] + Q11[104]*Gu1[26] + Q11[105]*Gu1[29] + Q11[106]*Gu1[32] + Q11[107]*Gu1[35] + Gu2[26];
Gu3[27] = + Q11[108]*Gu1[0] + Q11[109]*Gu1[3] + Q11[110]*Gu1[6] + Q11[111]*Gu1[9] + Q11[112]*Gu1[12] + Q11[113]*Gu1[15] + Q11[114]*Gu1[18] + Q11[115]*Gu1[21] + Q11[116]*Gu1[24] + Q11[117]*Gu1[27] + Q11[118]*Gu1[30] + Q11[119]*Gu1[33] + Gu2[27];
Gu3[28] = + Q11[108]*Gu1[1] + Q11[109]*Gu1[4] + Q11[110]*Gu1[7] + Q11[111]*Gu1[10] + Q11[112]*Gu1[13] + Q11[113]*Gu1[16] + Q11[114]*Gu1[19] + Q11[115]*Gu1[22] + Q11[116]*Gu1[25] + Q11[117]*Gu1[28] + Q11[118]*Gu1[31] + Q11[119]*Gu1[34] + Gu2[28];
Gu3[29] = + Q11[108]*Gu1[2] + Q11[109]*Gu1[5] + Q11[110]*Gu1[8] + Q11[111]*Gu1[11] + Q11[112]*Gu1[14] + Q11[113]*Gu1[17] + Q11[114]*Gu1[20] + Q11[115]*Gu1[23] + Q11[116]*Gu1[26] + Q11[117]*Gu1[29] + Q11[118]*Gu1[32] + Q11[119]*Gu1[35] + Gu2[29];
Gu3[30] = + Q11[120]*Gu1[0] + Q11[121]*Gu1[3] + Q11[122]*Gu1[6] + Q11[123]*Gu1[9] + Q11[124]*Gu1[12] + Q11[125]*Gu1[15] + Q11[126]*Gu1[18] + Q11[127]*Gu1[21] + Q11[128]*Gu1[24] + Q11[129]*Gu1[27] + Q11[130]*Gu1[30] + Q11[131]*Gu1[33] + Gu2[30];
Gu3[31] = + Q11[120]*Gu1[1] + Q11[121]*Gu1[4] + Q11[122]*Gu1[7] + Q11[123]*Gu1[10] + Q11[124]*Gu1[13] + Q11[125]*Gu1[16] + Q11[126]*Gu1[19] + Q11[127]*Gu1[22] + Q11[128]*Gu1[25] + Q11[129]*Gu1[28] + Q11[130]*Gu1[31] + Q11[131]*Gu1[34] + Gu2[31];
Gu3[32] = + Q11[120]*Gu1[2] + Q11[121]*Gu1[5] + Q11[122]*Gu1[8] + Q11[123]*Gu1[11] + Q11[124]*Gu1[14] + Q11[125]*Gu1[17] + Q11[126]*Gu1[20] + Q11[127]*Gu1[23] + Q11[128]*Gu1[26] + Q11[129]*Gu1[29] + Q11[130]*Gu1[32] + Q11[131]*Gu1[35] + Gu2[32];
Gu3[33] = + Q11[132]*Gu1[0] + Q11[133]*Gu1[3] + Q11[134]*Gu1[6] + Q11[135]*Gu1[9] + Q11[136]*Gu1[12] + Q11[137]*Gu1[15] + Q11[138]*Gu1[18] + Q11[139]*Gu1[21] + Q11[140]*Gu1[24] + Q11[141]*Gu1[27] + Q11[142]*Gu1[30] + Q11[143]*Gu1[33] + Gu2[33];
Gu3[34] = + Q11[132]*Gu1[1] + Q11[133]*Gu1[4] + Q11[134]*Gu1[7] + Q11[135]*Gu1[10] + Q11[136]*Gu1[13] + Q11[137]*Gu1[16] + Q11[138]*Gu1[19] + Q11[139]*Gu1[22] + Q11[140]*Gu1[25] + Q11[141]*Gu1[28] + Q11[142]*Gu1[31] + Q11[143]*Gu1[34] + Gu2[34];
Gu3[35] = + Q11[132]*Gu1[2] + Q11[133]*Gu1[5] + Q11[134]*Gu1[8] + Q11[135]*Gu1[11] + Q11[136]*Gu1[14] + Q11[137]*Gu1[17] + Q11[138]*Gu1[20] + Q11[139]*Gu1[23] + Q11[140]*Gu1[26] + Q11[141]*Gu1[29] + Q11[142]*Gu1[32] + Q11[143]*Gu1[35] + Gu2[35];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[12]*w11[1] + Gx1[24]*w11[2] + Gx1[36]*w11[3] + Gx1[48]*w11[4] + Gx1[60]*w11[5] + Gx1[72]*w11[6] + Gx1[84]*w11[7] + Gx1[96]*w11[8] + Gx1[108]*w11[9] + Gx1[120]*w11[10] + Gx1[132]*w11[11] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[13]*w11[1] + Gx1[25]*w11[2] + Gx1[37]*w11[3] + Gx1[49]*w11[4] + Gx1[61]*w11[5] + Gx1[73]*w11[6] + Gx1[85]*w11[7] + Gx1[97]*w11[8] + Gx1[109]*w11[9] + Gx1[121]*w11[10] + Gx1[133]*w11[11] + w12[1];
w13[2] = + Gx1[2]*w11[0] + Gx1[14]*w11[1] + Gx1[26]*w11[2] + Gx1[38]*w11[3] + Gx1[50]*w11[4] + Gx1[62]*w11[5] + Gx1[74]*w11[6] + Gx1[86]*w11[7] + Gx1[98]*w11[8] + Gx1[110]*w11[9] + Gx1[122]*w11[10] + Gx1[134]*w11[11] + w12[2];
w13[3] = + Gx1[3]*w11[0] + Gx1[15]*w11[1] + Gx1[27]*w11[2] + Gx1[39]*w11[3] + Gx1[51]*w11[4] + Gx1[63]*w11[5] + Gx1[75]*w11[6] + Gx1[87]*w11[7] + Gx1[99]*w11[8] + Gx1[111]*w11[9] + Gx1[123]*w11[10] + Gx1[135]*w11[11] + w12[3];
w13[4] = + Gx1[4]*w11[0] + Gx1[16]*w11[1] + Gx1[28]*w11[2] + Gx1[40]*w11[3] + Gx1[52]*w11[4] + Gx1[64]*w11[5] + Gx1[76]*w11[6] + Gx1[88]*w11[7] + Gx1[100]*w11[8] + Gx1[112]*w11[9] + Gx1[124]*w11[10] + Gx1[136]*w11[11] + w12[4];
w13[5] = + Gx1[5]*w11[0] + Gx1[17]*w11[1] + Gx1[29]*w11[2] + Gx1[41]*w11[3] + Gx1[53]*w11[4] + Gx1[65]*w11[5] + Gx1[77]*w11[6] + Gx1[89]*w11[7] + Gx1[101]*w11[8] + Gx1[113]*w11[9] + Gx1[125]*w11[10] + Gx1[137]*w11[11] + w12[5];
w13[6] = + Gx1[6]*w11[0] + Gx1[18]*w11[1] + Gx1[30]*w11[2] + Gx1[42]*w11[3] + Gx1[54]*w11[4] + Gx1[66]*w11[5] + Gx1[78]*w11[6] + Gx1[90]*w11[7] + Gx1[102]*w11[8] + Gx1[114]*w11[9] + Gx1[126]*w11[10] + Gx1[138]*w11[11] + w12[6];
w13[7] = + Gx1[7]*w11[0] + Gx1[19]*w11[1] + Gx1[31]*w11[2] + Gx1[43]*w11[3] + Gx1[55]*w11[4] + Gx1[67]*w11[5] + Gx1[79]*w11[6] + Gx1[91]*w11[7] + Gx1[103]*w11[8] + Gx1[115]*w11[9] + Gx1[127]*w11[10] + Gx1[139]*w11[11] + w12[7];
w13[8] = + Gx1[8]*w11[0] + Gx1[20]*w11[1] + Gx1[32]*w11[2] + Gx1[44]*w11[3] + Gx1[56]*w11[4] + Gx1[68]*w11[5] + Gx1[80]*w11[6] + Gx1[92]*w11[7] + Gx1[104]*w11[8] + Gx1[116]*w11[9] + Gx1[128]*w11[10] + Gx1[140]*w11[11] + w12[8];
w13[9] = + Gx1[9]*w11[0] + Gx1[21]*w11[1] + Gx1[33]*w11[2] + Gx1[45]*w11[3] + Gx1[57]*w11[4] + Gx1[69]*w11[5] + Gx1[81]*w11[6] + Gx1[93]*w11[7] + Gx1[105]*w11[8] + Gx1[117]*w11[9] + Gx1[129]*w11[10] + Gx1[141]*w11[11] + w12[9];
w13[10] = + Gx1[10]*w11[0] + Gx1[22]*w11[1] + Gx1[34]*w11[2] + Gx1[46]*w11[3] + Gx1[58]*w11[4] + Gx1[70]*w11[5] + Gx1[82]*w11[6] + Gx1[94]*w11[7] + Gx1[106]*w11[8] + Gx1[118]*w11[9] + Gx1[130]*w11[10] + Gx1[142]*w11[11] + w12[10];
w13[11] = + Gx1[11]*w11[0] + Gx1[23]*w11[1] + Gx1[35]*w11[2] + Gx1[47]*w11[3] + Gx1[59]*w11[4] + Gx1[71]*w11[5] + Gx1[83]*w11[6] + Gx1[95]*w11[7] + Gx1[107]*w11[8] + Gx1[119]*w11[9] + Gx1[131]*w11[10] + Gx1[143]*w11[11] + w12[11];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[3]*w11[1] + Gu1[6]*w11[2] + Gu1[9]*w11[3] + Gu1[12]*w11[4] + Gu1[15]*w11[5] + Gu1[18]*w11[6] + Gu1[21]*w11[7] + Gu1[24]*w11[8] + Gu1[27]*w11[9] + Gu1[30]*w11[10] + Gu1[33]*w11[11];
U1[1] += + Gu1[1]*w11[0] + Gu1[4]*w11[1] + Gu1[7]*w11[2] + Gu1[10]*w11[3] + Gu1[13]*w11[4] + Gu1[16]*w11[5] + Gu1[19]*w11[6] + Gu1[22]*w11[7] + Gu1[25]*w11[8] + Gu1[28]*w11[9] + Gu1[31]*w11[10] + Gu1[34]*w11[11];
U1[2] += + Gu1[2]*w11[0] + Gu1[5]*w11[1] + Gu1[8]*w11[2] + Gu1[11]*w11[3] + Gu1[14]*w11[4] + Gu1[17]*w11[5] + Gu1[20]*w11[6] + Gu1[23]*w11[7] + Gu1[26]*w11[8] + Gu1[29]*w11[9] + Gu1[32]*w11[10] + Gu1[35]*w11[11];
}

void acado_macS1TSbar( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[3]*w11[1] + Gu1[6]*w11[2] + Gu1[9]*w11[3] + Gu1[12]*w11[4] + Gu1[15]*w11[5] + Gu1[18]*w11[6] + Gu1[21]*w11[7] + Gu1[24]*w11[8] + Gu1[27]*w11[9] + Gu1[30]*w11[10] + Gu1[33]*w11[11];
U1[1] += + Gu1[1]*w11[0] + Gu1[4]*w11[1] + Gu1[7]*w11[2] + Gu1[10]*w11[3] + Gu1[13]*w11[4] + Gu1[16]*w11[5] + Gu1[19]*w11[6] + Gu1[22]*w11[7] + Gu1[25]*w11[8] + Gu1[28]*w11[9] + Gu1[31]*w11[10] + Gu1[34]*w11[11];
U1[2] += + Gu1[2]*w11[0] + Gu1[5]*w11[1] + Gu1[8]*w11[2] + Gu1[11]*w11[3] + Gu1[14]*w11[4] + Gu1[17]*w11[5] + Gu1[20]*w11[6] + Gu1[23]*w11[7] + Gu1[26]*w11[8] + Gu1[29]*w11[9] + Gu1[32]*w11[10] + Gu1[35]*w11[11];
}

void acado_macQSbarW2( real_t* const Q11, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Q11[0]*w11[0] + Q11[1]*w11[1] + Q11[2]*w11[2] + Q11[3]*w11[3] + Q11[4]*w11[4] + Q11[5]*w11[5] + Q11[6]*w11[6] + Q11[7]*w11[7] + Q11[8]*w11[8] + Q11[9]*w11[9] + Q11[10]*w11[10] + Q11[11]*w11[11] + w12[0];
w13[1] = + Q11[12]*w11[0] + Q11[13]*w11[1] + Q11[14]*w11[2] + Q11[15]*w11[3] + Q11[16]*w11[4] + Q11[17]*w11[5] + Q11[18]*w11[6] + Q11[19]*w11[7] + Q11[20]*w11[8] + Q11[21]*w11[9] + Q11[22]*w11[10] + Q11[23]*w11[11] + w12[1];
w13[2] = + Q11[24]*w11[0] + Q11[25]*w11[1] + Q11[26]*w11[2] + Q11[27]*w11[3] + Q11[28]*w11[4] + Q11[29]*w11[5] + Q11[30]*w11[6] + Q11[31]*w11[7] + Q11[32]*w11[8] + Q11[33]*w11[9] + Q11[34]*w11[10] + Q11[35]*w11[11] + w12[2];
w13[3] = + Q11[36]*w11[0] + Q11[37]*w11[1] + Q11[38]*w11[2] + Q11[39]*w11[3] + Q11[40]*w11[4] + Q11[41]*w11[5] + Q11[42]*w11[6] + Q11[43]*w11[7] + Q11[44]*w11[8] + Q11[45]*w11[9] + Q11[46]*w11[10] + Q11[47]*w11[11] + w12[3];
w13[4] = + Q11[48]*w11[0] + Q11[49]*w11[1] + Q11[50]*w11[2] + Q11[51]*w11[3] + Q11[52]*w11[4] + Q11[53]*w11[5] + Q11[54]*w11[6] + Q11[55]*w11[7] + Q11[56]*w11[8] + Q11[57]*w11[9] + Q11[58]*w11[10] + Q11[59]*w11[11] + w12[4];
w13[5] = + Q11[60]*w11[0] + Q11[61]*w11[1] + Q11[62]*w11[2] + Q11[63]*w11[3] + Q11[64]*w11[4] + Q11[65]*w11[5] + Q11[66]*w11[6] + Q11[67]*w11[7] + Q11[68]*w11[8] + Q11[69]*w11[9] + Q11[70]*w11[10] + Q11[71]*w11[11] + w12[5];
w13[6] = + Q11[72]*w11[0] + Q11[73]*w11[1] + Q11[74]*w11[2] + Q11[75]*w11[3] + Q11[76]*w11[4] + Q11[77]*w11[5] + Q11[78]*w11[6] + Q11[79]*w11[7] + Q11[80]*w11[8] + Q11[81]*w11[9] + Q11[82]*w11[10] + Q11[83]*w11[11] + w12[6];
w13[7] = + Q11[84]*w11[0] + Q11[85]*w11[1] + Q11[86]*w11[2] + Q11[87]*w11[3] + Q11[88]*w11[4] + Q11[89]*w11[5] + Q11[90]*w11[6] + Q11[91]*w11[7] + Q11[92]*w11[8] + Q11[93]*w11[9] + Q11[94]*w11[10] + Q11[95]*w11[11] + w12[7];
w13[8] = + Q11[96]*w11[0] + Q11[97]*w11[1] + Q11[98]*w11[2] + Q11[99]*w11[3] + Q11[100]*w11[4] + Q11[101]*w11[5] + Q11[102]*w11[6] + Q11[103]*w11[7] + Q11[104]*w11[8] + Q11[105]*w11[9] + Q11[106]*w11[10] + Q11[107]*w11[11] + w12[8];
w13[9] = + Q11[108]*w11[0] + Q11[109]*w11[1] + Q11[110]*w11[2] + Q11[111]*w11[3] + Q11[112]*w11[4] + Q11[113]*w11[5] + Q11[114]*w11[6] + Q11[115]*w11[7] + Q11[116]*w11[8] + Q11[117]*w11[9] + Q11[118]*w11[10] + Q11[119]*w11[11] + w12[9];
w13[10] = + Q11[120]*w11[0] + Q11[121]*w11[1] + Q11[122]*w11[2] + Q11[123]*w11[3] + Q11[124]*w11[4] + Q11[125]*w11[5] + Q11[126]*w11[6] + Q11[127]*w11[7] + Q11[128]*w11[8] + Q11[129]*w11[9] + Q11[130]*w11[10] + Q11[131]*w11[11] + w12[10];
w13[11] = + Q11[132]*w11[0] + Q11[133]*w11[1] + Q11[134]*w11[2] + Q11[135]*w11[3] + Q11[136]*w11[4] + Q11[137]*w11[5] + Q11[138]*w11[6] + Q11[139]*w11[7] + Q11[140]*w11[8] + Q11[141]*w11[9] + Q11[142]*w11[10] + Q11[143]*w11[11] + w12[11];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6] + Gx1[7]*w11[7] + Gx1[8]*w11[8] + Gx1[9]*w11[9] + Gx1[10]*w11[10] + Gx1[11]*w11[11];
w12[1] += + Gx1[12]*w11[0] + Gx1[13]*w11[1] + Gx1[14]*w11[2] + Gx1[15]*w11[3] + Gx1[16]*w11[4] + Gx1[17]*w11[5] + Gx1[18]*w11[6] + Gx1[19]*w11[7] + Gx1[20]*w11[8] + Gx1[21]*w11[9] + Gx1[22]*w11[10] + Gx1[23]*w11[11];
w12[2] += + Gx1[24]*w11[0] + Gx1[25]*w11[1] + Gx1[26]*w11[2] + Gx1[27]*w11[3] + Gx1[28]*w11[4] + Gx1[29]*w11[5] + Gx1[30]*w11[6] + Gx1[31]*w11[7] + Gx1[32]*w11[8] + Gx1[33]*w11[9] + Gx1[34]*w11[10] + Gx1[35]*w11[11];
w12[3] += + Gx1[36]*w11[0] + Gx1[37]*w11[1] + Gx1[38]*w11[2] + Gx1[39]*w11[3] + Gx1[40]*w11[4] + Gx1[41]*w11[5] + Gx1[42]*w11[6] + Gx1[43]*w11[7] + Gx1[44]*w11[8] + Gx1[45]*w11[9] + Gx1[46]*w11[10] + Gx1[47]*w11[11];
w12[4] += + Gx1[48]*w11[0] + Gx1[49]*w11[1] + Gx1[50]*w11[2] + Gx1[51]*w11[3] + Gx1[52]*w11[4] + Gx1[53]*w11[5] + Gx1[54]*w11[6] + Gx1[55]*w11[7] + Gx1[56]*w11[8] + Gx1[57]*w11[9] + Gx1[58]*w11[10] + Gx1[59]*w11[11];
w12[5] += + Gx1[60]*w11[0] + Gx1[61]*w11[1] + Gx1[62]*w11[2] + Gx1[63]*w11[3] + Gx1[64]*w11[4] + Gx1[65]*w11[5] + Gx1[66]*w11[6] + Gx1[67]*w11[7] + Gx1[68]*w11[8] + Gx1[69]*w11[9] + Gx1[70]*w11[10] + Gx1[71]*w11[11];
w12[6] += + Gx1[72]*w11[0] + Gx1[73]*w11[1] + Gx1[74]*w11[2] + Gx1[75]*w11[3] + Gx1[76]*w11[4] + Gx1[77]*w11[5] + Gx1[78]*w11[6] + Gx1[79]*w11[7] + Gx1[80]*w11[8] + Gx1[81]*w11[9] + Gx1[82]*w11[10] + Gx1[83]*w11[11];
w12[7] += + Gx1[84]*w11[0] + Gx1[85]*w11[1] + Gx1[86]*w11[2] + Gx1[87]*w11[3] + Gx1[88]*w11[4] + Gx1[89]*w11[5] + Gx1[90]*w11[6] + Gx1[91]*w11[7] + Gx1[92]*w11[8] + Gx1[93]*w11[9] + Gx1[94]*w11[10] + Gx1[95]*w11[11];
w12[8] += + Gx1[96]*w11[0] + Gx1[97]*w11[1] + Gx1[98]*w11[2] + Gx1[99]*w11[3] + Gx1[100]*w11[4] + Gx1[101]*w11[5] + Gx1[102]*w11[6] + Gx1[103]*w11[7] + Gx1[104]*w11[8] + Gx1[105]*w11[9] + Gx1[106]*w11[10] + Gx1[107]*w11[11];
w12[9] += + Gx1[108]*w11[0] + Gx1[109]*w11[1] + Gx1[110]*w11[2] + Gx1[111]*w11[3] + Gx1[112]*w11[4] + Gx1[113]*w11[5] + Gx1[114]*w11[6] + Gx1[115]*w11[7] + Gx1[116]*w11[8] + Gx1[117]*w11[9] + Gx1[118]*w11[10] + Gx1[119]*w11[11];
w12[10] += + Gx1[120]*w11[0] + Gx1[121]*w11[1] + Gx1[122]*w11[2] + Gx1[123]*w11[3] + Gx1[124]*w11[4] + Gx1[125]*w11[5] + Gx1[126]*w11[6] + Gx1[127]*w11[7] + Gx1[128]*w11[8] + Gx1[129]*w11[9] + Gx1[130]*w11[10] + Gx1[131]*w11[11];
w12[11] += + Gx1[132]*w11[0] + Gx1[133]*w11[1] + Gx1[134]*w11[2] + Gx1[135]*w11[3] + Gx1[136]*w11[4] + Gx1[137]*w11[5] + Gx1[138]*w11[6] + Gx1[139]*w11[7] + Gx1[140]*w11[8] + Gx1[141]*w11[9] + Gx1[142]*w11[10] + Gx1[143]*w11[11];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6] + Gx1[7]*w11[7] + Gx1[8]*w11[8] + Gx1[9]*w11[9] + Gx1[10]*w11[10] + Gx1[11]*w11[11];
w12[1] += + Gx1[12]*w11[0] + Gx1[13]*w11[1] + Gx1[14]*w11[2] + Gx1[15]*w11[3] + Gx1[16]*w11[4] + Gx1[17]*w11[5] + Gx1[18]*w11[6] + Gx1[19]*w11[7] + Gx1[20]*w11[8] + Gx1[21]*w11[9] + Gx1[22]*w11[10] + Gx1[23]*w11[11];
w12[2] += + Gx1[24]*w11[0] + Gx1[25]*w11[1] + Gx1[26]*w11[2] + Gx1[27]*w11[3] + Gx1[28]*w11[4] + Gx1[29]*w11[5] + Gx1[30]*w11[6] + Gx1[31]*w11[7] + Gx1[32]*w11[8] + Gx1[33]*w11[9] + Gx1[34]*w11[10] + Gx1[35]*w11[11];
w12[3] += + Gx1[36]*w11[0] + Gx1[37]*w11[1] + Gx1[38]*w11[2] + Gx1[39]*w11[3] + Gx1[40]*w11[4] + Gx1[41]*w11[5] + Gx1[42]*w11[6] + Gx1[43]*w11[7] + Gx1[44]*w11[8] + Gx1[45]*w11[9] + Gx1[46]*w11[10] + Gx1[47]*w11[11];
w12[4] += + Gx1[48]*w11[0] + Gx1[49]*w11[1] + Gx1[50]*w11[2] + Gx1[51]*w11[3] + Gx1[52]*w11[4] + Gx1[53]*w11[5] + Gx1[54]*w11[6] + Gx1[55]*w11[7] + Gx1[56]*w11[8] + Gx1[57]*w11[9] + Gx1[58]*w11[10] + Gx1[59]*w11[11];
w12[5] += + Gx1[60]*w11[0] + Gx1[61]*w11[1] + Gx1[62]*w11[2] + Gx1[63]*w11[3] + Gx1[64]*w11[4] + Gx1[65]*w11[5] + Gx1[66]*w11[6] + Gx1[67]*w11[7] + Gx1[68]*w11[8] + Gx1[69]*w11[9] + Gx1[70]*w11[10] + Gx1[71]*w11[11];
w12[6] += + Gx1[72]*w11[0] + Gx1[73]*w11[1] + Gx1[74]*w11[2] + Gx1[75]*w11[3] + Gx1[76]*w11[4] + Gx1[77]*w11[5] + Gx1[78]*w11[6] + Gx1[79]*w11[7] + Gx1[80]*w11[8] + Gx1[81]*w11[9] + Gx1[82]*w11[10] + Gx1[83]*w11[11];
w12[7] += + Gx1[84]*w11[0] + Gx1[85]*w11[1] + Gx1[86]*w11[2] + Gx1[87]*w11[3] + Gx1[88]*w11[4] + Gx1[89]*w11[5] + Gx1[90]*w11[6] + Gx1[91]*w11[7] + Gx1[92]*w11[8] + Gx1[93]*w11[9] + Gx1[94]*w11[10] + Gx1[95]*w11[11];
w12[8] += + Gx1[96]*w11[0] + Gx1[97]*w11[1] + Gx1[98]*w11[2] + Gx1[99]*w11[3] + Gx1[100]*w11[4] + Gx1[101]*w11[5] + Gx1[102]*w11[6] + Gx1[103]*w11[7] + Gx1[104]*w11[8] + Gx1[105]*w11[9] + Gx1[106]*w11[10] + Gx1[107]*w11[11];
w12[9] += + Gx1[108]*w11[0] + Gx1[109]*w11[1] + Gx1[110]*w11[2] + Gx1[111]*w11[3] + Gx1[112]*w11[4] + Gx1[113]*w11[5] + Gx1[114]*w11[6] + Gx1[115]*w11[7] + Gx1[116]*w11[8] + Gx1[117]*w11[9] + Gx1[118]*w11[10] + Gx1[119]*w11[11];
w12[10] += + Gx1[120]*w11[0] + Gx1[121]*w11[1] + Gx1[122]*w11[2] + Gx1[123]*w11[3] + Gx1[124]*w11[4] + Gx1[125]*w11[5] + Gx1[126]*w11[6] + Gx1[127]*w11[7] + Gx1[128]*w11[8] + Gx1[129]*w11[9] + Gx1[130]*w11[10] + Gx1[131]*w11[11];
w12[11] += + Gx1[132]*w11[0] + Gx1[133]*w11[1] + Gx1[134]*w11[2] + Gx1[135]*w11[3] + Gx1[136]*w11[4] + Gx1[137]*w11[5] + Gx1[138]*w11[6] + Gx1[139]*w11[7] + Gx1[140]*w11[8] + Gx1[141]*w11[9] + Gx1[142]*w11[10] + Gx1[143]*w11[11];
w12[0] += + Gu1[0]*U1[0] + Gu1[1]*U1[1] + Gu1[2]*U1[2];
w12[1] += + Gu1[3]*U1[0] + Gu1[4]*U1[1] + Gu1[5]*U1[2];
w12[2] += + Gu1[6]*U1[0] + Gu1[7]*U1[1] + Gu1[8]*U1[2];
w12[3] += + Gu1[9]*U1[0] + Gu1[10]*U1[1] + Gu1[11]*U1[2];
w12[4] += + Gu1[12]*U1[0] + Gu1[13]*U1[1] + Gu1[14]*U1[2];
w12[5] += + Gu1[15]*U1[0] + Gu1[16]*U1[1] + Gu1[17]*U1[2];
w12[6] += + Gu1[18]*U1[0] + Gu1[19]*U1[1] + Gu1[20]*U1[2];
w12[7] += + Gu1[21]*U1[0] + Gu1[22]*U1[1] + Gu1[23]*U1[2];
w12[8] += + Gu1[24]*U1[0] + Gu1[25]*U1[1] + Gu1[26]*U1[2];
w12[9] += + Gu1[27]*U1[0] + Gu1[28]*U1[1] + Gu1[29]*U1[2];
w12[10] += + Gu1[30]*U1[0] + Gu1[31]*U1[1] + Gu1[32]*U1[2];
w12[11] += + Gu1[33]*U1[0] + Gu1[34]*U1[1] + Gu1[35]*U1[2];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 45) + (iCol * 3)] = acadoWorkspace.H[(iCol * 45) + (iRow * 3)];
acadoWorkspace.H[(iRow * 45) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 45 + 15) + (iRow * 3)];
acadoWorkspace.H[(iRow * 45) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 45 + 30) + (iRow * 3)];
acadoWorkspace.H[(iRow * 45 + 15) + (iCol * 3)] = acadoWorkspace.H[(iCol * 45) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 45 + 15) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 45 + 15) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 45 + 15) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 45 + 30) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 45 + 30) + (iCol * 3)] = acadoWorkspace.H[(iCol * 45) + (iRow * 3 + 2)];
acadoWorkspace.H[(iRow * 45 + 30) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 45 + 15) + (iRow * 3 + 2)];
acadoWorkspace.H[(iRow * 45 + 30) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 45 + 30) + (iRow * 3 + 2)];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6] + R2[7]*Dy1[7] + R2[8]*Dy1[8] + R2[9]*Dy1[9] + R2[10]*Dy1[10] + R2[11]*Dy1[11] + R2[12]*Dy1[12];
RDy1[1] = + R2[13]*Dy1[0] + R2[14]*Dy1[1] + R2[15]*Dy1[2] + R2[16]*Dy1[3] + R2[17]*Dy1[4] + R2[18]*Dy1[5] + R2[19]*Dy1[6] + R2[20]*Dy1[7] + R2[21]*Dy1[8] + R2[22]*Dy1[9] + R2[23]*Dy1[10] + R2[24]*Dy1[11] + R2[25]*Dy1[12];
RDy1[2] = + R2[26]*Dy1[0] + R2[27]*Dy1[1] + R2[28]*Dy1[2] + R2[29]*Dy1[3] + R2[30]*Dy1[4] + R2[31]*Dy1[5] + R2[32]*Dy1[6] + R2[33]*Dy1[7] + R2[34]*Dy1[8] + R2[35]*Dy1[9] + R2[36]*Dy1[10] + R2[37]*Dy1[11] + R2[38]*Dy1[12];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6] + Q2[7]*Dy1[7] + Q2[8]*Dy1[8] + Q2[9]*Dy1[9] + Q2[10]*Dy1[10] + Q2[11]*Dy1[11] + Q2[12]*Dy1[12];
QDy1[1] = + Q2[13]*Dy1[0] + Q2[14]*Dy1[1] + Q2[15]*Dy1[2] + Q2[16]*Dy1[3] + Q2[17]*Dy1[4] + Q2[18]*Dy1[5] + Q2[19]*Dy1[6] + Q2[20]*Dy1[7] + Q2[21]*Dy1[8] + Q2[22]*Dy1[9] + Q2[23]*Dy1[10] + Q2[24]*Dy1[11] + Q2[25]*Dy1[12];
QDy1[2] = + Q2[26]*Dy1[0] + Q2[27]*Dy1[1] + Q2[28]*Dy1[2] + Q2[29]*Dy1[3] + Q2[30]*Dy1[4] + Q2[31]*Dy1[5] + Q2[32]*Dy1[6] + Q2[33]*Dy1[7] + Q2[34]*Dy1[8] + Q2[35]*Dy1[9] + Q2[36]*Dy1[10] + Q2[37]*Dy1[11] + Q2[38]*Dy1[12];
QDy1[3] = + Q2[39]*Dy1[0] + Q2[40]*Dy1[1] + Q2[41]*Dy1[2] + Q2[42]*Dy1[3] + Q2[43]*Dy1[4] + Q2[44]*Dy1[5] + Q2[45]*Dy1[6] + Q2[46]*Dy1[7] + Q2[47]*Dy1[8] + Q2[48]*Dy1[9] + Q2[49]*Dy1[10] + Q2[50]*Dy1[11] + Q2[51]*Dy1[12];
QDy1[4] = + Q2[52]*Dy1[0] + Q2[53]*Dy1[1] + Q2[54]*Dy1[2] + Q2[55]*Dy1[3] + Q2[56]*Dy1[4] + Q2[57]*Dy1[5] + Q2[58]*Dy1[6] + Q2[59]*Dy1[7] + Q2[60]*Dy1[8] + Q2[61]*Dy1[9] + Q2[62]*Dy1[10] + Q2[63]*Dy1[11] + Q2[64]*Dy1[12];
QDy1[5] = + Q2[65]*Dy1[0] + Q2[66]*Dy1[1] + Q2[67]*Dy1[2] + Q2[68]*Dy1[3] + Q2[69]*Dy1[4] + Q2[70]*Dy1[5] + Q2[71]*Dy1[6] + Q2[72]*Dy1[7] + Q2[73]*Dy1[8] + Q2[74]*Dy1[9] + Q2[75]*Dy1[10] + Q2[76]*Dy1[11] + Q2[77]*Dy1[12];
QDy1[6] = + Q2[78]*Dy1[0] + Q2[79]*Dy1[1] + Q2[80]*Dy1[2] + Q2[81]*Dy1[3] + Q2[82]*Dy1[4] + Q2[83]*Dy1[5] + Q2[84]*Dy1[6] + Q2[85]*Dy1[7] + Q2[86]*Dy1[8] + Q2[87]*Dy1[9] + Q2[88]*Dy1[10] + Q2[89]*Dy1[11] + Q2[90]*Dy1[12];
QDy1[7] = + Q2[91]*Dy1[0] + Q2[92]*Dy1[1] + Q2[93]*Dy1[2] + Q2[94]*Dy1[3] + Q2[95]*Dy1[4] + Q2[96]*Dy1[5] + Q2[97]*Dy1[6] + Q2[98]*Dy1[7] + Q2[99]*Dy1[8] + Q2[100]*Dy1[9] + Q2[101]*Dy1[10] + Q2[102]*Dy1[11] + Q2[103]*Dy1[12];
QDy1[8] = + Q2[104]*Dy1[0] + Q2[105]*Dy1[1] + Q2[106]*Dy1[2] + Q2[107]*Dy1[3] + Q2[108]*Dy1[4] + Q2[109]*Dy1[5] + Q2[110]*Dy1[6] + Q2[111]*Dy1[7] + Q2[112]*Dy1[8] + Q2[113]*Dy1[9] + Q2[114]*Dy1[10] + Q2[115]*Dy1[11] + Q2[116]*Dy1[12];
QDy1[9] = + Q2[117]*Dy1[0] + Q2[118]*Dy1[1] + Q2[119]*Dy1[2] + Q2[120]*Dy1[3] + Q2[121]*Dy1[4] + Q2[122]*Dy1[5] + Q2[123]*Dy1[6] + Q2[124]*Dy1[7] + Q2[125]*Dy1[8] + Q2[126]*Dy1[9] + Q2[127]*Dy1[10] + Q2[128]*Dy1[11] + Q2[129]*Dy1[12];
QDy1[10] = + Q2[130]*Dy1[0] + Q2[131]*Dy1[1] + Q2[132]*Dy1[2] + Q2[133]*Dy1[3] + Q2[134]*Dy1[4] + Q2[135]*Dy1[5] + Q2[136]*Dy1[6] + Q2[137]*Dy1[7] + Q2[138]*Dy1[8] + Q2[139]*Dy1[9] + Q2[140]*Dy1[10] + Q2[141]*Dy1[11] + Q2[142]*Dy1[12];
QDy1[11] = + Q2[143]*Dy1[0] + Q2[144]*Dy1[1] + Q2[145]*Dy1[2] + Q2[146]*Dy1[3] + Q2[147]*Dy1[4] + Q2[148]*Dy1[5] + Q2[149]*Dy1[6] + Q2[150]*Dy1[7] + Q2[151]*Dy1[8] + Q2[152]*Dy1[9] + Q2[153]*Dy1[10] + Q2[154]*Dy1[11] + Q2[155]*Dy1[12];
}

void acado_condensePrep(  )
{
/* Column: 0 */
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
acado_multGxGu( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.E, &(acadoWorkspace.E[ 36 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.E[ 72 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.E[ 108 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.E[ 144 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 144 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.W1, 4, 0 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 144 ]), &(acadoWorkspace.E[ 108 ]), 4, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.E[ 108 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 108 ]), acadoWorkspace.W1, 3, 0 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 108 ]), &(acadoWorkspace.E[ 72 ]), 3, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 432 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 432 ]), &(acadoWorkspace.E[ 72 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 72 ]), acadoWorkspace.W1, 2, 0 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 72 ]), &(acadoWorkspace.E[ 36 ]), 2, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 288 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 36 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 36 ]), acadoWorkspace.W1, 1, 0 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 36 ]), acadoWorkspace.E, 1, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 144 ]), acadoWorkspace.E, acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( acadoWorkspace.R1, acadoWorkspace.evGu, acadoWorkspace.W1, 0 );

/* Column: 1 */
acado_moveGuE( &(acadoWorkspace.evGu[ 36 ]), &(acadoWorkspace.E[ 180 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.E[ 216 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.E[ 252 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.E[ 288 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 288 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.W1, 4, 1 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 144 ]), &(acadoWorkspace.E[ 252 ]), 4, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.E[ 252 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 108 ]), acadoWorkspace.W1, 3, 1 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 108 ]), &(acadoWorkspace.E[ 216 ]), 3, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 432 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 432 ]), &(acadoWorkspace.E[ 216 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 72 ]), acadoWorkspace.W1, 2, 1 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 72 ]), &(acadoWorkspace.E[ 180 ]), 2, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 288 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 180 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 9 ]), &(acadoWorkspace.evGu[ 36 ]), acadoWorkspace.W1, 1 );

/* Column: 2 */
acado_moveGuE( &(acadoWorkspace.evGu[ 72 ]), &(acadoWorkspace.E[ 324 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.E[ 360 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.E[ 396 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 396 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.W1, 4, 2 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 144 ]), &(acadoWorkspace.E[ 360 ]), 4, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.E[ 360 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 108 ]), acadoWorkspace.W1, 3, 2 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 108 ]), &(acadoWorkspace.E[ 324 ]), 3, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 432 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 432 ]), &(acadoWorkspace.E[ 324 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 18 ]), &(acadoWorkspace.evGu[ 72 ]), acadoWorkspace.W1, 2 );

/* Column: 3 */
acado_moveGuE( &(acadoWorkspace.evGu[ 108 ]), &(acadoWorkspace.E[ 432 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.E[ 468 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 468 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.W1, 4, 3 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 144 ]), &(acadoWorkspace.E[ 432 ]), 4, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.E[ 432 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 27 ]), &(acadoWorkspace.evGu[ 108 ]), acadoWorkspace.W1, 3 );

/* Column: 4 */
acado_moveGuE( &(acadoWorkspace.evGu[ 144 ]), &(acadoWorkspace.E[ 504 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 504 ]), acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 36 ]), &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.W1, 4 );

acado_copyHTH( 0, 1 );
acado_copyHTH( 0, 2 );
acado_copyHTH( 1, 2 );
acado_copyHTH( 0, 3 );
acado_copyHTH( 1, 3 );
acado_copyHTH( 2, 3 );
acado_copyHTH( 0, 4 );
acado_copyHTH( 1, 4 );
acado_copyHTH( 2, 4 );
acado_copyHTH( 3, 4 );

acadoWorkspace.sbar[12] = acadoWorkspace.d[0];
acadoWorkspace.sbar[13] = acadoWorkspace.d[1];
acadoWorkspace.sbar[14] = acadoWorkspace.d[2];
acadoWorkspace.sbar[15] = acadoWorkspace.d[3];
acadoWorkspace.sbar[16] = acadoWorkspace.d[4];
acadoWorkspace.sbar[17] = acadoWorkspace.d[5];
acadoWorkspace.sbar[18] = acadoWorkspace.d[6];
acadoWorkspace.sbar[19] = acadoWorkspace.d[7];
acadoWorkspace.sbar[20] = acadoWorkspace.d[8];
acadoWorkspace.sbar[21] = acadoWorkspace.d[9];
acadoWorkspace.sbar[22] = acadoWorkspace.d[10];
acadoWorkspace.sbar[23] = acadoWorkspace.d[11];
acadoWorkspace.sbar[24] = acadoWorkspace.d[12];
acadoWorkspace.sbar[25] = acadoWorkspace.d[13];
acadoWorkspace.sbar[26] = acadoWorkspace.d[14];
acadoWorkspace.sbar[27] = acadoWorkspace.d[15];
acadoWorkspace.sbar[28] = acadoWorkspace.d[16];
acadoWorkspace.sbar[29] = acadoWorkspace.d[17];
acadoWorkspace.sbar[30] = acadoWorkspace.d[18];
acadoWorkspace.sbar[31] = acadoWorkspace.d[19];
acadoWorkspace.sbar[32] = acadoWorkspace.d[20];
acadoWorkspace.sbar[33] = acadoWorkspace.d[21];
acadoWorkspace.sbar[34] = acadoWorkspace.d[22];
acadoWorkspace.sbar[35] = acadoWorkspace.d[23];
acadoWorkspace.sbar[36] = acadoWorkspace.d[24];
acadoWorkspace.sbar[37] = acadoWorkspace.d[25];
acadoWorkspace.sbar[38] = acadoWorkspace.d[26];
acadoWorkspace.sbar[39] = acadoWorkspace.d[27];
acadoWorkspace.sbar[40] = acadoWorkspace.d[28];
acadoWorkspace.sbar[41] = acadoWorkspace.d[29];
acadoWorkspace.sbar[42] = acadoWorkspace.d[30];
acadoWorkspace.sbar[43] = acadoWorkspace.d[31];
acadoWorkspace.sbar[44] = acadoWorkspace.d[32];
acadoWorkspace.sbar[45] = acadoWorkspace.d[33];
acadoWorkspace.sbar[46] = acadoWorkspace.d[34];
acadoWorkspace.sbar[47] = acadoWorkspace.d[35];
acadoWorkspace.sbar[48] = acadoWorkspace.d[36];
acadoWorkspace.sbar[49] = acadoWorkspace.d[37];
acadoWorkspace.sbar[50] = acadoWorkspace.d[38];
acadoWorkspace.sbar[51] = acadoWorkspace.d[39];
acadoWorkspace.sbar[52] = acadoWorkspace.d[40];
acadoWorkspace.sbar[53] = acadoWorkspace.d[41];
acadoWorkspace.sbar[54] = acadoWorkspace.d[42];
acadoWorkspace.sbar[55] = acadoWorkspace.d[43];
acadoWorkspace.sbar[56] = acadoWorkspace.d[44];
acadoWorkspace.sbar[57] = acadoWorkspace.d[45];
acadoWorkspace.sbar[58] = acadoWorkspace.d[46];
acadoWorkspace.sbar[59] = acadoWorkspace.d[47];
acadoWorkspace.sbar[60] = acadoWorkspace.d[48];
acadoWorkspace.sbar[61] = acadoWorkspace.d[49];
acadoWorkspace.sbar[62] = acadoWorkspace.d[50];
acadoWorkspace.sbar[63] = acadoWorkspace.d[51];
acadoWorkspace.sbar[64] = acadoWorkspace.d[52];
acadoWorkspace.sbar[65] = acadoWorkspace.d[53];
acadoWorkspace.sbar[66] = acadoWorkspace.d[54];
acadoWorkspace.sbar[67] = acadoWorkspace.d[55];
acadoWorkspace.sbar[68] = acadoWorkspace.d[56];
acadoWorkspace.sbar[69] = acadoWorkspace.d[57];
acadoWorkspace.sbar[70] = acadoWorkspace.d[58];
acadoWorkspace.sbar[71] = acadoWorkspace.d[59];

}

void acado_condenseFdb(  )
{
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];
acadoWorkspace.Dx0[5] = acadoVariables.x0[5] - acadoVariables.x[5];
acadoWorkspace.Dx0[6] = acadoVariables.x0[6] - acadoVariables.x[6];
acadoWorkspace.Dx0[7] = acadoVariables.x0[7] - acadoVariables.x[7];
acadoWorkspace.Dx0[8] = acadoVariables.x0[8] - acadoVariables.x[8];
acadoWorkspace.Dx0[9] = acadoVariables.x0[9] - acadoVariables.x[9];
acadoWorkspace.Dx0[10] = acadoVariables.x0[10] - acadoVariables.x[10];
acadoWorkspace.Dx0[11] = acadoVariables.x0[11] - acadoVariables.x[11];
acadoWorkspace.Dy[0] -= acadoVariables.y[0];
acadoWorkspace.Dy[1] -= acadoVariables.y[1];
acadoWorkspace.Dy[2] -= acadoVariables.y[2];
acadoWorkspace.Dy[3] -= acadoVariables.y[3];
acadoWorkspace.Dy[4] -= acadoVariables.y[4];
acadoWorkspace.Dy[5] -= acadoVariables.y[5];
acadoWorkspace.Dy[6] -= acadoVariables.y[6];
acadoWorkspace.Dy[7] -= acadoVariables.y[7];
acadoWorkspace.Dy[8] -= acadoVariables.y[8];
acadoWorkspace.Dy[9] -= acadoVariables.y[9];
acadoWorkspace.Dy[10] -= acadoVariables.y[10];
acadoWorkspace.Dy[11] -= acadoVariables.y[11];
acadoWorkspace.Dy[12] -= acadoVariables.y[12];
acadoWorkspace.Dy[13] -= acadoVariables.y[13];
acadoWorkspace.Dy[14] -= acadoVariables.y[14];
acadoWorkspace.Dy[15] -= acadoVariables.y[15];
acadoWorkspace.Dy[16] -= acadoVariables.y[16];
acadoWorkspace.Dy[17] -= acadoVariables.y[17];
acadoWorkspace.Dy[18] -= acadoVariables.y[18];
acadoWorkspace.Dy[19] -= acadoVariables.y[19];
acadoWorkspace.Dy[20] -= acadoVariables.y[20];
acadoWorkspace.Dy[21] -= acadoVariables.y[21];
acadoWorkspace.Dy[22] -= acadoVariables.y[22];
acadoWorkspace.Dy[23] -= acadoVariables.y[23];
acadoWorkspace.Dy[24] -= acadoVariables.y[24];
acadoWorkspace.Dy[25] -= acadoVariables.y[25];
acadoWorkspace.Dy[26] -= acadoVariables.y[26];
acadoWorkspace.Dy[27] -= acadoVariables.y[27];
acadoWorkspace.Dy[28] -= acadoVariables.y[28];
acadoWorkspace.Dy[29] -= acadoVariables.y[29];
acadoWorkspace.Dy[30] -= acadoVariables.y[30];
acadoWorkspace.Dy[31] -= acadoVariables.y[31];
acadoWorkspace.Dy[32] -= acadoVariables.y[32];
acadoWorkspace.Dy[33] -= acadoVariables.y[33];
acadoWorkspace.Dy[34] -= acadoVariables.y[34];
acadoWorkspace.Dy[35] -= acadoVariables.y[35];
acadoWorkspace.Dy[36] -= acadoVariables.y[36];
acadoWorkspace.Dy[37] -= acadoVariables.y[37];
acadoWorkspace.Dy[38] -= acadoVariables.y[38];
acadoWorkspace.Dy[39] -= acadoVariables.y[39];
acadoWorkspace.Dy[40] -= acadoVariables.y[40];
acadoWorkspace.Dy[41] -= acadoVariables.y[41];
acadoWorkspace.Dy[42] -= acadoVariables.y[42];
acadoWorkspace.Dy[43] -= acadoVariables.y[43];
acadoWorkspace.Dy[44] -= acadoVariables.y[44];
acadoWorkspace.Dy[45] -= acadoVariables.y[45];
acadoWorkspace.Dy[46] -= acadoVariables.y[46];
acadoWorkspace.Dy[47] -= acadoVariables.y[47];
acadoWorkspace.Dy[48] -= acadoVariables.y[48];
acadoWorkspace.Dy[49] -= acadoVariables.y[49];
acadoWorkspace.Dy[50] -= acadoVariables.y[50];
acadoWorkspace.Dy[51] -= acadoVariables.y[51];
acadoWorkspace.Dy[52] -= acadoVariables.y[52];
acadoWorkspace.Dy[53] -= acadoVariables.y[53];
acadoWorkspace.Dy[54] -= acadoVariables.y[54];
acadoWorkspace.Dy[55] -= acadoVariables.y[55];
acadoWorkspace.Dy[56] -= acadoVariables.y[56];
acadoWorkspace.Dy[57] -= acadoVariables.y[57];
acadoWorkspace.Dy[58] -= acadoVariables.y[58];
acadoWorkspace.Dy[59] -= acadoVariables.y[59];
acadoWorkspace.Dy[60] -= acadoVariables.y[60];
acadoWorkspace.Dy[61] -= acadoVariables.y[61];
acadoWorkspace.Dy[62] -= acadoVariables.y[62];
acadoWorkspace.Dy[63] -= acadoVariables.y[63];
acadoWorkspace.Dy[64] -= acadoVariables.y[64];
acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];
acadoWorkspace.DyN[3] -= acadoVariables.yN[3];
acadoWorkspace.DyN[4] -= acadoVariables.yN[4];
acadoWorkspace.DyN[5] -= acadoVariables.yN[5];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 39 ]), &(acadoWorkspace.Dy[ 13 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 78 ]), &(acadoWorkspace.Dy[ 26 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 117 ]), &(acadoWorkspace.Dy[ 39 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 156 ]), &(acadoWorkspace.Dy[ 52 ]), &(acadoWorkspace.g[ 12 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 156 ]), &(acadoWorkspace.Dy[ 13 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 312 ]), &(acadoWorkspace.Dy[ 26 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 468 ]), &(acadoWorkspace.Dy[ 39 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 624 ]), &(acadoWorkspace.Dy[ 52 ]), &(acadoWorkspace.QDy[ 48 ]) );

acadoWorkspace.QDy[60] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[61] = + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[62] = + acadoWorkspace.QN2[12]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[13]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[14]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[15]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[16]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[17]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[63] = + acadoWorkspace.QN2[18]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[19]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[20]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[21]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[22]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[23]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[64] = + acadoWorkspace.QN2[24]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[25]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[26]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[27]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[28]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[29]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[65] = + acadoWorkspace.QN2[30]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[31]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[32]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[33]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[34]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[35]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[66] = + acadoWorkspace.QN2[36]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[37]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[38]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[39]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[40]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[41]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[67] = + acadoWorkspace.QN2[42]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[43]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[44]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[45]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[46]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[47]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[68] = + acadoWorkspace.QN2[48]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[49]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[50]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[51]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[52]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[53]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[69] = + acadoWorkspace.QN2[54]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[55]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[56]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[57]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[58]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[59]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[70] = + acadoWorkspace.QN2[60]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[61]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[62]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[63]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[64]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[65]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[71] = + acadoWorkspace.QN2[66]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[67]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[68]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[69]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[70]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[71]*acadoWorkspace.DyN[5];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acadoWorkspace.sbar[7] = acadoWorkspace.Dx0[7];
acadoWorkspace.sbar[8] = acadoWorkspace.Dx0[8];
acadoWorkspace.sbar[9] = acadoWorkspace.Dx0[9];
acadoWorkspace.sbar[10] = acadoWorkspace.Dx0[10];
acadoWorkspace.sbar[11] = acadoWorkspace.Dx0[11];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 12 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 48 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.sbar[ 60 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[60] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[61] + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[62] + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[63] + acadoWorkspace.QN1[4]*acadoWorkspace.sbar[64] + acadoWorkspace.QN1[5]*acadoWorkspace.sbar[65] + acadoWorkspace.QN1[6]*acadoWorkspace.sbar[66] + acadoWorkspace.QN1[7]*acadoWorkspace.sbar[67] + acadoWorkspace.QN1[8]*acadoWorkspace.sbar[68] + acadoWorkspace.QN1[9]*acadoWorkspace.sbar[69] + acadoWorkspace.QN1[10]*acadoWorkspace.sbar[70] + acadoWorkspace.QN1[11]*acadoWorkspace.sbar[71] + acadoWorkspace.QDy[60];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[12]*acadoWorkspace.sbar[60] + acadoWorkspace.QN1[13]*acadoWorkspace.sbar[61] + acadoWorkspace.QN1[14]*acadoWorkspace.sbar[62] + acadoWorkspace.QN1[15]*acadoWorkspace.sbar[63] + acadoWorkspace.QN1[16]*acadoWorkspace.sbar[64] + acadoWorkspace.QN1[17]*acadoWorkspace.sbar[65] + acadoWorkspace.QN1[18]*acadoWorkspace.sbar[66] + acadoWorkspace.QN1[19]*acadoWorkspace.sbar[67] + acadoWorkspace.QN1[20]*acadoWorkspace.sbar[68] + acadoWorkspace.QN1[21]*acadoWorkspace.sbar[69] + acadoWorkspace.QN1[22]*acadoWorkspace.sbar[70] + acadoWorkspace.QN1[23]*acadoWorkspace.sbar[71] + acadoWorkspace.QDy[61];
acadoWorkspace.w1[2] = + acadoWorkspace.QN1[24]*acadoWorkspace.sbar[60] + acadoWorkspace.QN1[25]*acadoWorkspace.sbar[61] + acadoWorkspace.QN1[26]*acadoWorkspace.sbar[62] + acadoWorkspace.QN1[27]*acadoWorkspace.sbar[63] + acadoWorkspace.QN1[28]*acadoWorkspace.sbar[64] + acadoWorkspace.QN1[29]*acadoWorkspace.sbar[65] + acadoWorkspace.QN1[30]*acadoWorkspace.sbar[66] + acadoWorkspace.QN1[31]*acadoWorkspace.sbar[67] + acadoWorkspace.QN1[32]*acadoWorkspace.sbar[68] + acadoWorkspace.QN1[33]*acadoWorkspace.sbar[69] + acadoWorkspace.QN1[34]*acadoWorkspace.sbar[70] + acadoWorkspace.QN1[35]*acadoWorkspace.sbar[71] + acadoWorkspace.QDy[62];
acadoWorkspace.w1[3] = + acadoWorkspace.QN1[36]*acadoWorkspace.sbar[60] + acadoWorkspace.QN1[37]*acadoWorkspace.sbar[61] + acadoWorkspace.QN1[38]*acadoWorkspace.sbar[62] + acadoWorkspace.QN1[39]*acadoWorkspace.sbar[63] + acadoWorkspace.QN1[40]*acadoWorkspace.sbar[64] + acadoWorkspace.QN1[41]*acadoWorkspace.sbar[65] + acadoWorkspace.QN1[42]*acadoWorkspace.sbar[66] + acadoWorkspace.QN1[43]*acadoWorkspace.sbar[67] + acadoWorkspace.QN1[44]*acadoWorkspace.sbar[68] + acadoWorkspace.QN1[45]*acadoWorkspace.sbar[69] + acadoWorkspace.QN1[46]*acadoWorkspace.sbar[70] + acadoWorkspace.QN1[47]*acadoWorkspace.sbar[71] + acadoWorkspace.QDy[63];
acadoWorkspace.w1[4] = + acadoWorkspace.QN1[48]*acadoWorkspace.sbar[60] + acadoWorkspace.QN1[49]*acadoWorkspace.sbar[61] + acadoWorkspace.QN1[50]*acadoWorkspace.sbar[62] + acadoWorkspace.QN1[51]*acadoWorkspace.sbar[63] + acadoWorkspace.QN1[52]*acadoWorkspace.sbar[64] + acadoWorkspace.QN1[53]*acadoWorkspace.sbar[65] + acadoWorkspace.QN1[54]*acadoWorkspace.sbar[66] + acadoWorkspace.QN1[55]*acadoWorkspace.sbar[67] + acadoWorkspace.QN1[56]*acadoWorkspace.sbar[68] + acadoWorkspace.QN1[57]*acadoWorkspace.sbar[69] + acadoWorkspace.QN1[58]*acadoWorkspace.sbar[70] + acadoWorkspace.QN1[59]*acadoWorkspace.sbar[71] + acadoWorkspace.QDy[64];
acadoWorkspace.w1[5] = + acadoWorkspace.QN1[60]*acadoWorkspace.sbar[60] + acadoWorkspace.QN1[61]*acadoWorkspace.sbar[61] + acadoWorkspace.QN1[62]*acadoWorkspace.sbar[62] + acadoWorkspace.QN1[63]*acadoWorkspace.sbar[63] + acadoWorkspace.QN1[64]*acadoWorkspace.sbar[64] + acadoWorkspace.QN1[65]*acadoWorkspace.sbar[65] + acadoWorkspace.QN1[66]*acadoWorkspace.sbar[66] + acadoWorkspace.QN1[67]*acadoWorkspace.sbar[67] + acadoWorkspace.QN1[68]*acadoWorkspace.sbar[68] + acadoWorkspace.QN1[69]*acadoWorkspace.sbar[69] + acadoWorkspace.QN1[70]*acadoWorkspace.sbar[70] + acadoWorkspace.QN1[71]*acadoWorkspace.sbar[71] + acadoWorkspace.QDy[65];
acadoWorkspace.w1[6] = + acadoWorkspace.QN1[72]*acadoWorkspace.sbar[60] + acadoWorkspace.QN1[73]*acadoWorkspace.sbar[61] + acadoWorkspace.QN1[74]*acadoWorkspace.sbar[62] + acadoWorkspace.QN1[75]*acadoWorkspace.sbar[63] + acadoWorkspace.QN1[76]*acadoWorkspace.sbar[64] + acadoWorkspace.QN1[77]*acadoWorkspace.sbar[65] + acadoWorkspace.QN1[78]*acadoWorkspace.sbar[66] + acadoWorkspace.QN1[79]*acadoWorkspace.sbar[67] + acadoWorkspace.QN1[80]*acadoWorkspace.sbar[68] + acadoWorkspace.QN1[81]*acadoWorkspace.sbar[69] + acadoWorkspace.QN1[82]*acadoWorkspace.sbar[70] + acadoWorkspace.QN1[83]*acadoWorkspace.sbar[71] + acadoWorkspace.QDy[66];
acadoWorkspace.w1[7] = + acadoWorkspace.QN1[84]*acadoWorkspace.sbar[60] + acadoWorkspace.QN1[85]*acadoWorkspace.sbar[61] + acadoWorkspace.QN1[86]*acadoWorkspace.sbar[62] + acadoWorkspace.QN1[87]*acadoWorkspace.sbar[63] + acadoWorkspace.QN1[88]*acadoWorkspace.sbar[64] + acadoWorkspace.QN1[89]*acadoWorkspace.sbar[65] + acadoWorkspace.QN1[90]*acadoWorkspace.sbar[66] + acadoWorkspace.QN1[91]*acadoWorkspace.sbar[67] + acadoWorkspace.QN1[92]*acadoWorkspace.sbar[68] + acadoWorkspace.QN1[93]*acadoWorkspace.sbar[69] + acadoWorkspace.QN1[94]*acadoWorkspace.sbar[70] + acadoWorkspace.QN1[95]*acadoWorkspace.sbar[71] + acadoWorkspace.QDy[67];
acadoWorkspace.w1[8] = + acadoWorkspace.QN1[96]*acadoWorkspace.sbar[60] + acadoWorkspace.QN1[97]*acadoWorkspace.sbar[61] + acadoWorkspace.QN1[98]*acadoWorkspace.sbar[62] + acadoWorkspace.QN1[99]*acadoWorkspace.sbar[63] + acadoWorkspace.QN1[100]*acadoWorkspace.sbar[64] + acadoWorkspace.QN1[101]*acadoWorkspace.sbar[65] + acadoWorkspace.QN1[102]*acadoWorkspace.sbar[66] + acadoWorkspace.QN1[103]*acadoWorkspace.sbar[67] + acadoWorkspace.QN1[104]*acadoWorkspace.sbar[68] + acadoWorkspace.QN1[105]*acadoWorkspace.sbar[69] + acadoWorkspace.QN1[106]*acadoWorkspace.sbar[70] + acadoWorkspace.QN1[107]*acadoWorkspace.sbar[71] + acadoWorkspace.QDy[68];
acadoWorkspace.w1[9] = + acadoWorkspace.QN1[108]*acadoWorkspace.sbar[60] + acadoWorkspace.QN1[109]*acadoWorkspace.sbar[61] + acadoWorkspace.QN1[110]*acadoWorkspace.sbar[62] + acadoWorkspace.QN1[111]*acadoWorkspace.sbar[63] + acadoWorkspace.QN1[112]*acadoWorkspace.sbar[64] + acadoWorkspace.QN1[113]*acadoWorkspace.sbar[65] + acadoWorkspace.QN1[114]*acadoWorkspace.sbar[66] + acadoWorkspace.QN1[115]*acadoWorkspace.sbar[67] + acadoWorkspace.QN1[116]*acadoWorkspace.sbar[68] + acadoWorkspace.QN1[117]*acadoWorkspace.sbar[69] + acadoWorkspace.QN1[118]*acadoWorkspace.sbar[70] + acadoWorkspace.QN1[119]*acadoWorkspace.sbar[71] + acadoWorkspace.QDy[69];
acadoWorkspace.w1[10] = + acadoWorkspace.QN1[120]*acadoWorkspace.sbar[60] + acadoWorkspace.QN1[121]*acadoWorkspace.sbar[61] + acadoWorkspace.QN1[122]*acadoWorkspace.sbar[62] + acadoWorkspace.QN1[123]*acadoWorkspace.sbar[63] + acadoWorkspace.QN1[124]*acadoWorkspace.sbar[64] + acadoWorkspace.QN1[125]*acadoWorkspace.sbar[65] + acadoWorkspace.QN1[126]*acadoWorkspace.sbar[66] + acadoWorkspace.QN1[127]*acadoWorkspace.sbar[67] + acadoWorkspace.QN1[128]*acadoWorkspace.sbar[68] + acadoWorkspace.QN1[129]*acadoWorkspace.sbar[69] + acadoWorkspace.QN1[130]*acadoWorkspace.sbar[70] + acadoWorkspace.QN1[131]*acadoWorkspace.sbar[71] + acadoWorkspace.QDy[70];
acadoWorkspace.w1[11] = + acadoWorkspace.QN1[132]*acadoWorkspace.sbar[60] + acadoWorkspace.QN1[133]*acadoWorkspace.sbar[61] + acadoWorkspace.QN1[134]*acadoWorkspace.sbar[62] + acadoWorkspace.QN1[135]*acadoWorkspace.sbar[63] + acadoWorkspace.QN1[136]*acadoWorkspace.sbar[64] + acadoWorkspace.QN1[137]*acadoWorkspace.sbar[65] + acadoWorkspace.QN1[138]*acadoWorkspace.sbar[66] + acadoWorkspace.QN1[139]*acadoWorkspace.sbar[67] + acadoWorkspace.QN1[140]*acadoWorkspace.sbar[68] + acadoWorkspace.QN1[141]*acadoWorkspace.sbar[69] + acadoWorkspace.QN1[142]*acadoWorkspace.sbar[70] + acadoWorkspace.QN1[143]*acadoWorkspace.sbar[71] + acadoWorkspace.QDy[71];
acado_macBTw1( &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 144 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 48 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.sbar[ 48 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 108 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 9 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 108 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 432 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 432 ]), &(acadoWorkspace.sbar[ 36 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 72 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 6 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 72 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 288 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 24 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.sbar[ 24 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 36 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 3 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 36 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 12 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.sbar[ 12 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );
acado_macS1TSbar( acadoWorkspace.S1, acadoWorkspace.sbar, acadoWorkspace.g );

acadoWorkspace.lb[0] = acadoVariables.lbValues[0] - acadoVariables.u[0];
acadoWorkspace.lb[1] = acadoVariables.lbValues[1] - acadoVariables.u[1];
acadoWorkspace.lb[2] = acadoVariables.lbValues[2] - acadoVariables.u[2];
acadoWorkspace.lb[3] = acadoVariables.lbValues[3] - acadoVariables.u[3];
acadoWorkspace.lb[4] = acadoVariables.lbValues[4] - acadoVariables.u[4];
acadoWorkspace.lb[5] = acadoVariables.lbValues[5] - acadoVariables.u[5];
acadoWorkspace.lb[6] = acadoVariables.lbValues[6] - acadoVariables.u[6];
acadoWorkspace.lb[7] = acadoVariables.lbValues[7] - acadoVariables.u[7];
acadoWorkspace.lb[8] = acadoVariables.lbValues[8] - acadoVariables.u[8];
acadoWorkspace.lb[9] = acadoVariables.lbValues[9] - acadoVariables.u[9];
acadoWorkspace.lb[10] = acadoVariables.lbValues[10] - acadoVariables.u[10];
acadoWorkspace.lb[11] = acadoVariables.lbValues[11] - acadoVariables.u[11];
acadoWorkspace.lb[12] = acadoVariables.lbValues[12] - acadoVariables.u[12];
acadoWorkspace.lb[13] = acadoVariables.lbValues[13] - acadoVariables.u[13];
acadoWorkspace.lb[14] = acadoVariables.lbValues[14] - acadoVariables.u[14];
acadoWorkspace.ub[0] = acadoVariables.ubValues[0] - acadoVariables.u[0];
acadoWorkspace.ub[1] = acadoVariables.ubValues[1] - acadoVariables.u[1];
acadoWorkspace.ub[2] = acadoVariables.ubValues[2] - acadoVariables.u[2];
acadoWorkspace.ub[3] = acadoVariables.ubValues[3] - acadoVariables.u[3];
acadoWorkspace.ub[4] = acadoVariables.ubValues[4] - acadoVariables.u[4];
acadoWorkspace.ub[5] = acadoVariables.ubValues[5] - acadoVariables.u[5];
acadoWorkspace.ub[6] = acadoVariables.ubValues[6] - acadoVariables.u[6];
acadoWorkspace.ub[7] = acadoVariables.ubValues[7] - acadoVariables.u[7];
acadoWorkspace.ub[8] = acadoVariables.ubValues[8] - acadoVariables.u[8];
acadoWorkspace.ub[9] = acadoVariables.ubValues[9] - acadoVariables.u[9];
acadoWorkspace.ub[10] = acadoVariables.ubValues[10] - acadoVariables.u[10];
acadoWorkspace.ub[11] = acadoVariables.ubValues[11] - acadoVariables.u[11];
acadoWorkspace.ub[12] = acadoVariables.ubValues[12] - acadoVariables.u[12];
acadoWorkspace.ub[13] = acadoVariables.ubValues[13] - acadoVariables.u[13];
acadoWorkspace.ub[14] = acadoVariables.ubValues[14] - acadoVariables.u[14];

}

void acado_expand(  )
{
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
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acadoWorkspace.sbar[7] = acadoWorkspace.Dx0[7];
acadoWorkspace.sbar[8] = acadoWorkspace.Dx0[8];
acadoWorkspace.sbar[9] = acadoWorkspace.Dx0[9];
acadoWorkspace.sbar[10] = acadoWorkspace.Dx0[10];
acadoWorkspace.sbar[11] = acadoWorkspace.Dx0[11];
acadoWorkspace.sbar[12] = acadoWorkspace.d[0];
acadoWorkspace.sbar[13] = acadoWorkspace.d[1];
acadoWorkspace.sbar[14] = acadoWorkspace.d[2];
acadoWorkspace.sbar[15] = acadoWorkspace.d[3];
acadoWorkspace.sbar[16] = acadoWorkspace.d[4];
acadoWorkspace.sbar[17] = acadoWorkspace.d[5];
acadoWorkspace.sbar[18] = acadoWorkspace.d[6];
acadoWorkspace.sbar[19] = acadoWorkspace.d[7];
acadoWorkspace.sbar[20] = acadoWorkspace.d[8];
acadoWorkspace.sbar[21] = acadoWorkspace.d[9];
acadoWorkspace.sbar[22] = acadoWorkspace.d[10];
acadoWorkspace.sbar[23] = acadoWorkspace.d[11];
acadoWorkspace.sbar[24] = acadoWorkspace.d[12];
acadoWorkspace.sbar[25] = acadoWorkspace.d[13];
acadoWorkspace.sbar[26] = acadoWorkspace.d[14];
acadoWorkspace.sbar[27] = acadoWorkspace.d[15];
acadoWorkspace.sbar[28] = acadoWorkspace.d[16];
acadoWorkspace.sbar[29] = acadoWorkspace.d[17];
acadoWorkspace.sbar[30] = acadoWorkspace.d[18];
acadoWorkspace.sbar[31] = acadoWorkspace.d[19];
acadoWorkspace.sbar[32] = acadoWorkspace.d[20];
acadoWorkspace.sbar[33] = acadoWorkspace.d[21];
acadoWorkspace.sbar[34] = acadoWorkspace.d[22];
acadoWorkspace.sbar[35] = acadoWorkspace.d[23];
acadoWorkspace.sbar[36] = acadoWorkspace.d[24];
acadoWorkspace.sbar[37] = acadoWorkspace.d[25];
acadoWorkspace.sbar[38] = acadoWorkspace.d[26];
acadoWorkspace.sbar[39] = acadoWorkspace.d[27];
acadoWorkspace.sbar[40] = acadoWorkspace.d[28];
acadoWorkspace.sbar[41] = acadoWorkspace.d[29];
acadoWorkspace.sbar[42] = acadoWorkspace.d[30];
acadoWorkspace.sbar[43] = acadoWorkspace.d[31];
acadoWorkspace.sbar[44] = acadoWorkspace.d[32];
acadoWorkspace.sbar[45] = acadoWorkspace.d[33];
acadoWorkspace.sbar[46] = acadoWorkspace.d[34];
acadoWorkspace.sbar[47] = acadoWorkspace.d[35];
acadoWorkspace.sbar[48] = acadoWorkspace.d[36];
acadoWorkspace.sbar[49] = acadoWorkspace.d[37];
acadoWorkspace.sbar[50] = acadoWorkspace.d[38];
acadoWorkspace.sbar[51] = acadoWorkspace.d[39];
acadoWorkspace.sbar[52] = acadoWorkspace.d[40];
acadoWorkspace.sbar[53] = acadoWorkspace.d[41];
acadoWorkspace.sbar[54] = acadoWorkspace.d[42];
acadoWorkspace.sbar[55] = acadoWorkspace.d[43];
acadoWorkspace.sbar[56] = acadoWorkspace.d[44];
acadoWorkspace.sbar[57] = acadoWorkspace.d[45];
acadoWorkspace.sbar[58] = acadoWorkspace.d[46];
acadoWorkspace.sbar[59] = acadoWorkspace.d[47];
acadoWorkspace.sbar[60] = acadoWorkspace.d[48];
acadoWorkspace.sbar[61] = acadoWorkspace.d[49];
acadoWorkspace.sbar[62] = acadoWorkspace.d[50];
acadoWorkspace.sbar[63] = acadoWorkspace.d[51];
acadoWorkspace.sbar[64] = acadoWorkspace.d[52];
acadoWorkspace.sbar[65] = acadoWorkspace.d[53];
acadoWorkspace.sbar[66] = acadoWorkspace.d[54];
acadoWorkspace.sbar[67] = acadoWorkspace.d[55];
acadoWorkspace.sbar[68] = acadoWorkspace.d[56];
acadoWorkspace.sbar[69] = acadoWorkspace.d[57];
acadoWorkspace.sbar[70] = acadoWorkspace.d[58];
acadoWorkspace.sbar[71] = acadoWorkspace.d[59];
acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 12 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.evGu[ 36 ]), &(acadoWorkspace.x[ 3 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.evGu[ 72 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.evGu[ 108 ]), &(acadoWorkspace.x[ 9 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 48 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.evGu[ 144 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.sbar[ 60 ]) );
acadoVariables.x[0] += acadoWorkspace.sbar[0];
acadoVariables.x[1] += acadoWorkspace.sbar[1];
acadoVariables.x[2] += acadoWorkspace.sbar[2];
acadoVariables.x[3] += acadoWorkspace.sbar[3];
acadoVariables.x[4] += acadoWorkspace.sbar[4];
acadoVariables.x[5] += acadoWorkspace.sbar[5];
acadoVariables.x[6] += acadoWorkspace.sbar[6];
acadoVariables.x[7] += acadoWorkspace.sbar[7];
acadoVariables.x[8] += acadoWorkspace.sbar[8];
acadoVariables.x[9] += acadoWorkspace.sbar[9];
acadoVariables.x[10] += acadoWorkspace.sbar[10];
acadoVariables.x[11] += acadoWorkspace.sbar[11];
acadoVariables.x[12] += acadoWorkspace.sbar[12];
acadoVariables.x[13] += acadoWorkspace.sbar[13];
acadoVariables.x[14] += acadoWorkspace.sbar[14];
acadoVariables.x[15] += acadoWorkspace.sbar[15];
acadoVariables.x[16] += acadoWorkspace.sbar[16];
acadoVariables.x[17] += acadoWorkspace.sbar[17];
acadoVariables.x[18] += acadoWorkspace.sbar[18];
acadoVariables.x[19] += acadoWorkspace.sbar[19];
acadoVariables.x[20] += acadoWorkspace.sbar[20];
acadoVariables.x[21] += acadoWorkspace.sbar[21];
acadoVariables.x[22] += acadoWorkspace.sbar[22];
acadoVariables.x[23] += acadoWorkspace.sbar[23];
acadoVariables.x[24] += acadoWorkspace.sbar[24];
acadoVariables.x[25] += acadoWorkspace.sbar[25];
acadoVariables.x[26] += acadoWorkspace.sbar[26];
acadoVariables.x[27] += acadoWorkspace.sbar[27];
acadoVariables.x[28] += acadoWorkspace.sbar[28];
acadoVariables.x[29] += acadoWorkspace.sbar[29];
acadoVariables.x[30] += acadoWorkspace.sbar[30];
acadoVariables.x[31] += acadoWorkspace.sbar[31];
acadoVariables.x[32] += acadoWorkspace.sbar[32];
acadoVariables.x[33] += acadoWorkspace.sbar[33];
acadoVariables.x[34] += acadoWorkspace.sbar[34];
acadoVariables.x[35] += acadoWorkspace.sbar[35];
acadoVariables.x[36] += acadoWorkspace.sbar[36];
acadoVariables.x[37] += acadoWorkspace.sbar[37];
acadoVariables.x[38] += acadoWorkspace.sbar[38];
acadoVariables.x[39] += acadoWorkspace.sbar[39];
acadoVariables.x[40] += acadoWorkspace.sbar[40];
acadoVariables.x[41] += acadoWorkspace.sbar[41];
acadoVariables.x[42] += acadoWorkspace.sbar[42];
acadoVariables.x[43] += acadoWorkspace.sbar[43];
acadoVariables.x[44] += acadoWorkspace.sbar[44];
acadoVariables.x[45] += acadoWorkspace.sbar[45];
acadoVariables.x[46] += acadoWorkspace.sbar[46];
acadoVariables.x[47] += acadoWorkspace.sbar[47];
acadoVariables.x[48] += acadoWorkspace.sbar[48];
acadoVariables.x[49] += acadoWorkspace.sbar[49];
acadoVariables.x[50] += acadoWorkspace.sbar[50];
acadoVariables.x[51] += acadoWorkspace.sbar[51];
acadoVariables.x[52] += acadoWorkspace.sbar[52];
acadoVariables.x[53] += acadoWorkspace.sbar[53];
acadoVariables.x[54] += acadoWorkspace.sbar[54];
acadoVariables.x[55] += acadoWorkspace.sbar[55];
acadoVariables.x[56] += acadoWorkspace.sbar[56];
acadoVariables.x[57] += acadoWorkspace.sbar[57];
acadoVariables.x[58] += acadoWorkspace.sbar[58];
acadoVariables.x[59] += acadoWorkspace.sbar[59];
acadoVariables.x[60] += acadoWorkspace.sbar[60];
acadoVariables.x[61] += acadoWorkspace.sbar[61];
acadoVariables.x[62] += acadoWorkspace.sbar[62];
acadoVariables.x[63] += acadoWorkspace.sbar[63];
acadoVariables.x[64] += acadoWorkspace.sbar[64];
acadoVariables.x[65] += acadoWorkspace.sbar[65];
acadoVariables.x[66] += acadoWorkspace.sbar[66];
acadoVariables.x[67] += acadoWorkspace.sbar[67];
acadoVariables.x[68] += acadoWorkspace.sbar[68];
acadoVariables.x[69] += acadoWorkspace.sbar[69];
acadoVariables.x[70] += acadoWorkspace.sbar[70];
acadoVariables.x[71] += acadoWorkspace.sbar[71];
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
acadoVariables.lbValues[0] = -7.8539816339744828e-01;
acadoVariables.lbValues[1] = -7.8539816339744828e-01;
acadoVariables.lbValues[2] = 4.9032999999999998e+00;
acadoVariables.lbValues[3] = -7.8539816339744828e-01;
acadoVariables.lbValues[4] = -7.8539816339744828e-01;
acadoVariables.lbValues[5] = 4.9032999999999998e+00;
acadoVariables.lbValues[6] = -7.8539816339744828e-01;
acadoVariables.lbValues[7] = -7.8539816339744828e-01;
acadoVariables.lbValues[8] = 4.9032999999999998e+00;
acadoVariables.lbValues[9] = -7.8539816339744828e-01;
acadoVariables.lbValues[10] = -7.8539816339744828e-01;
acadoVariables.lbValues[11] = 4.9032999999999998e+00;
acadoVariables.lbValues[12] = -7.8539816339744828e-01;
acadoVariables.lbValues[13] = -7.8539816339744828e-01;
acadoVariables.lbValues[14] = 4.9032999999999998e+00;
acadoVariables.ubValues[0] = 7.8539816339744828e-01;
acadoVariables.ubValues[1] = 7.8539816339744828e-01;
acadoVariables.ubValues[2] = 1.4709899999999999e+01;
acadoVariables.ubValues[3] = 7.8539816339744828e-01;
acadoVariables.ubValues[4] = 7.8539816339744828e-01;
acadoVariables.ubValues[5] = 1.4709899999999999e+01;
acadoVariables.ubValues[6] = 7.8539816339744828e-01;
acadoVariables.ubValues[7] = 7.8539816339744828e-01;
acadoVariables.ubValues[8] = 1.4709899999999999e+01;
acadoVariables.ubValues[9] = 7.8539816339744828e-01;
acadoVariables.ubValues[10] = 7.8539816339744828e-01;
acadoVariables.ubValues[11] = 1.4709899999999999e+01;
acadoVariables.ubValues[12] = 7.8539816339744828e-01;
acadoVariables.ubValues[13] = 7.8539816339744828e-01;
acadoVariables.ubValues[14] = 1.4709899999999999e+01;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 5; ++index)
{
state[0] = acadoVariables.x[index * 12];
state[1] = acadoVariables.x[index * 12 + 1];
state[2] = acadoVariables.x[index * 12 + 2];
state[3] = acadoVariables.x[index * 12 + 3];
state[4] = acadoVariables.x[index * 12 + 4];
state[5] = acadoVariables.x[index * 12 + 5];
state[6] = acadoVariables.x[index * 12 + 6];
state[7] = acadoVariables.x[index * 12 + 7];
state[8] = acadoVariables.x[index * 12 + 8];
state[9] = acadoVariables.x[index * 12 + 9];
state[10] = acadoVariables.x[index * 12 + 10];
state[11] = acadoVariables.x[index * 12 + 11];
state[192] = acadoVariables.u[index * 3];
state[193] = acadoVariables.u[index * 3 + 1];
state[194] = acadoVariables.u[index * 3 + 2];
state[195] = acadoVariables.od[index * 14];
state[196] = acadoVariables.od[index * 14 + 1];
state[197] = acadoVariables.od[index * 14 + 2];
state[198] = acadoVariables.od[index * 14 + 3];
state[199] = acadoVariables.od[index * 14 + 4];
state[200] = acadoVariables.od[index * 14 + 5];
state[201] = acadoVariables.od[index * 14 + 6];
state[202] = acadoVariables.od[index * 14 + 7];
state[203] = acadoVariables.od[index * 14 + 8];
state[204] = acadoVariables.od[index * 14 + 9];
state[205] = acadoVariables.od[index * 14 + 10];
state[206] = acadoVariables.od[index * 14 + 11];
state[207] = acadoVariables.od[index * 14 + 12];
state[208] = acadoVariables.od[index * 14 + 13];

acado_integrate(state, index == 0);

acadoVariables.x[index * 12 + 12] = state[0];
acadoVariables.x[index * 12 + 13] = state[1];
acadoVariables.x[index * 12 + 14] = state[2];
acadoVariables.x[index * 12 + 15] = state[3];
acadoVariables.x[index * 12 + 16] = state[4];
acadoVariables.x[index * 12 + 17] = state[5];
acadoVariables.x[index * 12 + 18] = state[6];
acadoVariables.x[index * 12 + 19] = state[7];
acadoVariables.x[index * 12 + 20] = state[8];
acadoVariables.x[index * 12 + 21] = state[9];
acadoVariables.x[index * 12 + 22] = state[10];
acadoVariables.x[index * 12 + 23] = state[11];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 5; ++index)
{
acadoVariables.x[index * 12] = acadoVariables.x[index * 12 + 12];
acadoVariables.x[index * 12 + 1] = acadoVariables.x[index * 12 + 13];
acadoVariables.x[index * 12 + 2] = acadoVariables.x[index * 12 + 14];
acadoVariables.x[index * 12 + 3] = acadoVariables.x[index * 12 + 15];
acadoVariables.x[index * 12 + 4] = acadoVariables.x[index * 12 + 16];
acadoVariables.x[index * 12 + 5] = acadoVariables.x[index * 12 + 17];
acadoVariables.x[index * 12 + 6] = acadoVariables.x[index * 12 + 18];
acadoVariables.x[index * 12 + 7] = acadoVariables.x[index * 12 + 19];
acadoVariables.x[index * 12 + 8] = acadoVariables.x[index * 12 + 20];
acadoVariables.x[index * 12 + 9] = acadoVariables.x[index * 12 + 21];
acadoVariables.x[index * 12 + 10] = acadoVariables.x[index * 12 + 22];
acadoVariables.x[index * 12 + 11] = acadoVariables.x[index * 12 + 23];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[60] = xEnd[0];
acadoVariables.x[61] = xEnd[1];
acadoVariables.x[62] = xEnd[2];
acadoVariables.x[63] = xEnd[3];
acadoVariables.x[64] = xEnd[4];
acadoVariables.x[65] = xEnd[5];
acadoVariables.x[66] = xEnd[6];
acadoVariables.x[67] = xEnd[7];
acadoVariables.x[68] = xEnd[8];
acadoVariables.x[69] = xEnd[9];
acadoVariables.x[70] = xEnd[10];
acadoVariables.x[71] = xEnd[11];
}
else if (strategy == 2) 
{
state[0] = acadoVariables.x[60];
state[1] = acadoVariables.x[61];
state[2] = acadoVariables.x[62];
state[3] = acadoVariables.x[63];
state[4] = acadoVariables.x[64];
state[5] = acadoVariables.x[65];
state[6] = acadoVariables.x[66];
state[7] = acadoVariables.x[67];
state[8] = acadoVariables.x[68];
state[9] = acadoVariables.x[69];
state[10] = acadoVariables.x[70];
state[11] = acadoVariables.x[71];
if (uEnd != 0)
{
state[192] = uEnd[0];
state[193] = uEnd[1];
state[194] = uEnd[2];
}
else
{
state[192] = acadoVariables.u[12];
state[193] = acadoVariables.u[13];
state[194] = acadoVariables.u[14];
}
state[195] = acadoVariables.od[70];
state[196] = acadoVariables.od[71];
state[197] = acadoVariables.od[72];
state[198] = acadoVariables.od[73];
state[199] = acadoVariables.od[74];
state[200] = acadoVariables.od[75];
state[201] = acadoVariables.od[76];
state[202] = acadoVariables.od[77];
state[203] = acadoVariables.od[78];
state[204] = acadoVariables.od[79];
state[205] = acadoVariables.od[80];
state[206] = acadoVariables.od[81];
state[207] = acadoVariables.od[82];
state[208] = acadoVariables.od[83];

acado_integrate(state, 1);

acadoVariables.x[60] = state[0];
acadoVariables.x[61] = state[1];
acadoVariables.x[62] = state[2];
acadoVariables.x[63] = state[3];
acadoVariables.x[64] = state[4];
acadoVariables.x[65] = state[5];
acadoVariables.x[66] = state[6];
acadoVariables.x[67] = state[7];
acadoVariables.x[68] = state[8];
acadoVariables.x[69] = state[9];
acadoVariables.x[70] = state[10];
acadoVariables.x[71] = state[11];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 4; ++index)
{
acadoVariables.u[index * 3] = acadoVariables.u[index * 3 + 3];
acadoVariables.u[index * 3 + 1] = acadoVariables.u[index * 3 + 4];
acadoVariables.u[index * 3 + 2] = acadoVariables.u[index * 3 + 5];
}

if (uEnd != 0)
{
acadoVariables.u[12] = uEnd[0];
acadoVariables.u[13] = uEnd[1];
acadoVariables.u[14] = uEnd[2];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14];
kkt = fabs( kkt );
for (index = 0; index < 15; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 13 */
real_t tmpDy[ 13 ];

/** Row vector of size: 6 */
real_t tmpDyN[ 6 ];

for (lRun1 = 0; lRun1 < 5; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 12];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 12 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 12 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 12 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 12 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[lRun1 * 12 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[lRun1 * 12 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[lRun1 * 12 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[lRun1 * 12 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.x[lRun1 * 12 + 9];
acadoWorkspace.objValueIn[10] = acadoVariables.x[lRun1 * 12 + 10];
acadoWorkspace.objValueIn[11] = acadoVariables.x[lRun1 * 12 + 11];
acadoWorkspace.objValueIn[12] = acadoVariables.u[lRun1 * 3];
acadoWorkspace.objValueIn[13] = acadoVariables.u[lRun1 * 3 + 1];
acadoWorkspace.objValueIn[14] = acadoVariables.u[lRun1 * 3 + 2];
acadoWorkspace.objValueIn[15] = acadoVariables.od[lRun1 * 14];
acadoWorkspace.objValueIn[16] = acadoVariables.od[lRun1 * 14 + 1];
acadoWorkspace.objValueIn[17] = acadoVariables.od[lRun1 * 14 + 2];
acadoWorkspace.objValueIn[18] = acadoVariables.od[lRun1 * 14 + 3];
acadoWorkspace.objValueIn[19] = acadoVariables.od[lRun1 * 14 + 4];
acadoWorkspace.objValueIn[20] = acadoVariables.od[lRun1 * 14 + 5];
acadoWorkspace.objValueIn[21] = acadoVariables.od[lRun1 * 14 + 6];
acadoWorkspace.objValueIn[22] = acadoVariables.od[lRun1 * 14 + 7];
acadoWorkspace.objValueIn[23] = acadoVariables.od[lRun1 * 14 + 8];
acadoWorkspace.objValueIn[24] = acadoVariables.od[lRun1 * 14 + 9];
acadoWorkspace.objValueIn[25] = acadoVariables.od[lRun1 * 14 + 10];
acadoWorkspace.objValueIn[26] = acadoVariables.od[lRun1 * 14 + 11];
acadoWorkspace.objValueIn[27] = acadoVariables.od[lRun1 * 14 + 12];
acadoWorkspace.objValueIn[28] = acadoVariables.od[lRun1 * 14 + 13];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 13] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 13];
acadoWorkspace.Dy[lRun1 * 13 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 13 + 1];
acadoWorkspace.Dy[lRun1 * 13 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 13 + 2];
acadoWorkspace.Dy[lRun1 * 13 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 13 + 3];
acadoWorkspace.Dy[lRun1 * 13 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 13 + 4];
acadoWorkspace.Dy[lRun1 * 13 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 13 + 5];
acadoWorkspace.Dy[lRun1 * 13 + 6] = acadoWorkspace.objValueOut[6] - acadoVariables.y[lRun1 * 13 + 6];
acadoWorkspace.Dy[lRun1 * 13 + 7] = acadoWorkspace.objValueOut[7] - acadoVariables.y[lRun1 * 13 + 7];
acadoWorkspace.Dy[lRun1 * 13 + 8] = acadoWorkspace.objValueOut[8] - acadoVariables.y[lRun1 * 13 + 8];
acadoWorkspace.Dy[lRun1 * 13 + 9] = acadoWorkspace.objValueOut[9] - acadoVariables.y[lRun1 * 13 + 9];
acadoWorkspace.Dy[lRun1 * 13 + 10] = acadoWorkspace.objValueOut[10] - acadoVariables.y[lRun1 * 13 + 10];
acadoWorkspace.Dy[lRun1 * 13 + 11] = acadoWorkspace.objValueOut[11] - acadoVariables.y[lRun1 * 13 + 11];
acadoWorkspace.Dy[lRun1 * 13 + 12] = acadoWorkspace.objValueOut[12] - acadoVariables.y[lRun1 * 13 + 12];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[60];
acadoWorkspace.objValueIn[1] = acadoVariables.x[61];
acadoWorkspace.objValueIn[2] = acadoVariables.x[62];
acadoWorkspace.objValueIn[3] = acadoVariables.x[63];
acadoWorkspace.objValueIn[4] = acadoVariables.x[64];
acadoWorkspace.objValueIn[5] = acadoVariables.x[65];
acadoWorkspace.objValueIn[6] = acadoVariables.x[66];
acadoWorkspace.objValueIn[7] = acadoVariables.x[67];
acadoWorkspace.objValueIn[8] = acadoVariables.x[68];
acadoWorkspace.objValueIn[9] = acadoVariables.x[69];
acadoWorkspace.objValueIn[10] = acadoVariables.x[70];
acadoWorkspace.objValueIn[11] = acadoVariables.x[71];
acadoWorkspace.objValueIn[12] = acadoVariables.od[70];
acadoWorkspace.objValueIn[13] = acadoVariables.od[71];
acadoWorkspace.objValueIn[14] = acadoVariables.od[72];
acadoWorkspace.objValueIn[15] = acadoVariables.od[73];
acadoWorkspace.objValueIn[16] = acadoVariables.od[74];
acadoWorkspace.objValueIn[17] = acadoVariables.od[75];
acadoWorkspace.objValueIn[18] = acadoVariables.od[76];
acadoWorkspace.objValueIn[19] = acadoVariables.od[77];
acadoWorkspace.objValueIn[20] = acadoVariables.od[78];
acadoWorkspace.objValueIn[21] = acadoVariables.od[79];
acadoWorkspace.objValueIn[22] = acadoVariables.od[80];
acadoWorkspace.objValueIn[23] = acadoVariables.od[81];
acadoWorkspace.objValueIn[24] = acadoVariables.od[82];
acadoWorkspace.objValueIn[25] = acadoVariables.od[83];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4] - acadoVariables.yN[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5] - acadoVariables.yN[5];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 5; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 13]*acadoVariables.W[0];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 13 + 1]*acadoVariables.W[14];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 13 + 2]*acadoVariables.W[28];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 13 + 3]*acadoVariables.W[42];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 13 + 4]*acadoVariables.W[56];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 13 + 5]*acadoVariables.W[70];
tmpDy[6] = + acadoWorkspace.Dy[lRun1 * 13 + 6]*acadoVariables.W[84];
tmpDy[7] = + acadoWorkspace.Dy[lRun1 * 13 + 7]*acadoVariables.W[98];
tmpDy[8] = + acadoWorkspace.Dy[lRun1 * 13 + 8]*acadoVariables.W[112];
tmpDy[9] = + acadoWorkspace.Dy[lRun1 * 13 + 9]*acadoVariables.W[126];
tmpDy[10] = + acadoWorkspace.Dy[lRun1 * 13 + 10]*acadoVariables.W[140];
tmpDy[11] = + acadoWorkspace.Dy[lRun1 * 13 + 11]*acadoVariables.W[154];
tmpDy[12] = + acadoWorkspace.Dy[lRun1 * 13 + 12]*acadoVariables.W[168];
objVal += + acadoWorkspace.Dy[lRun1 * 13]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 13 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 13 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 13 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 13 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 13 + 5]*tmpDy[5] + acadoWorkspace.Dy[lRun1 * 13 + 6]*tmpDy[6] + acadoWorkspace.Dy[lRun1 * 13 + 7]*tmpDy[7] + acadoWorkspace.Dy[lRun1 * 13 + 8]*tmpDy[8] + acadoWorkspace.Dy[lRun1 * 13 + 9]*tmpDy[9] + acadoWorkspace.Dy[lRun1 * 13 + 10]*tmpDy[10] + acadoWorkspace.Dy[lRun1 * 13 + 11]*tmpDy[11] + acadoWorkspace.Dy[lRun1 * 13 + 12]*tmpDy[12];
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

