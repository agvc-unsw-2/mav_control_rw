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


real_t rk_dim12_swap;

/** Column vector of size: 12 */
real_t rk_dim12_bPerm[ 12 ];

/** Column vector of size: 71 */
real_t auxVar[ 71 ];

real_t rk_ttt;

/** Row vector of size: 29 */
real_t rk_xxx[ 29 ];

/** Column vector of size: 12 */
real_t rk_kkk[ 12 ];

/** Matrix of size: 12 x 12 (row major format) */
real_t rk_A[ 144 ];

/** Column vector of size: 12 */
real_t rk_b[ 12 ];

/** Row vector of size: 12 */
int rk_dim12_perm[ 12 ];

/** Column vector of size: 12 */
real_t rk_rhsTemp[ 12 ];

/** Row vector of size: 180 */
real_t rk_diffsTemp2[ 180 ];

/** Column vector of size: 12 */
real_t rk_diffK[ 12 ];

/** Matrix of size: 12 x 15 (row major format) */
real_t rk_diffsPrev2[ 180 ];

/** Matrix of size: 12 x 15 (row major format) */
real_t rk_diffsNew2[ 180 ];

#pragma omp threadprivate( auxVar, rk_ttt, rk_xxx, rk_kkk, rk_diffK, rk_rhsTemp, rk_dim12_perm, rk_A, rk_b, rk_diffsPrev2, rk_diffsNew2, rk_diffsTemp2, rk_dim12_swap, rk_dim12_bPerm )

void acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 12;
const real_t* od = in + 15;
/* Vector of auxiliary variables; number of elements: 31. */
real_t* a = auxVar;

/* Compute intermediate quantities: */
a[0] = (cos(xd[3]));
a[1] = (cos(xd[5]));
a[2] = (sin(xd[4]));
a[3] = (sin(xd[3]));
a[4] = (sin(xd[5]));
a[5] = (sin(xd[4]));
a[6] = (cos(xd[4]));
a[7] = (cos(xd[5]));
a[8] = (cos(xd[4]));
a[9] = (sin(xd[5]));
a[10] = (((((a[5]*od[6])*u[2])*xd[2])+((((a[6]*a[7])*od[6])*u[2])*xd[0]))-((((a[8]*od[6])*a[9])*u[2])*xd[1]));
a[11] = (cos(xd[3]));
a[12] = (sin(xd[4]));
a[13] = (sin(xd[5]));
a[14] = (cos(xd[5]));
a[15] = (sin(xd[3]));
a[16] = (cos(xd[3]));
a[17] = (sin(xd[5]));
a[18] = (cos(xd[5]));
a[19] = (sin(xd[4]));
a[20] = (sin(xd[3]));
a[21] = (cos(xd[3]));
a[22] = (cos(xd[5]));
a[23] = (sin(xd[4]));
a[24] = (sin(xd[3]));
a[25] = (sin(xd[5]));
a[26] = (cos(xd[4]));
a[27] = (sin(xd[3]));
a[28] = (((((((a[16]*a[17])-((a[18]*a[19])*a[20]))*od[7])*u[2])*xd[0])-(((((a[21]*a[22])+((a[23]*a[24])*a[25]))*od[7])*u[2])*xd[1]))-((((a[26]*od[7])*a[27])*u[2])*xd[2]));
a[29] = (cos(xd[4]));
a[30] = (cos(xd[3]));

/* Compute outputs: */
out[0] = ((((((a[0]*a[1])*a[2])+(a[3]*a[4]))*u[2])-a[10])+od[8]);
out[1] = ((((((a[11]*a[12])*a[13])-(a[14]*a[15]))*u[2])-a[28])+od[9]);
out[2] = (((real_t)(-9.8065999999999995e+00)+((a[29]*a[30])*u[2]))+od[10]);
out[3] = xd[9];
out[4] = xd[10];
out[5] = xd[11];
out[6] = xd[0];
out[7] = xd[1];
out[8] = xd[2];
out[9] = (((((od[2]*od[1])*od[1])*u[0])-(((((real_t)(2.0000000000000000e+00)*od[0])*od[1])*xd[9])+((od[1]*od[1])*xd[3])))+od[11]);
out[10] = (((((od[5]*od[4])*od[4])*u[1])-(((((real_t)(2.0000000000000000e+00)*od[3])*od[4])*xd[10])+((od[4]*od[4])*xd[4])))+od[12]);
out[11] = od[13];
}



void acado_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 12;
const real_t* od = in + 15;
/* Vector of auxiliary variables; number of elements: 71. */
real_t* a = auxVar;

/* Compute intermediate quantities: */
a[0] = (cos(xd[4]));
a[1] = (cos(xd[5]));
a[2] = (((a[0]*a[1])*od[6])*u[2]);
a[3] = (cos(xd[4]));
a[4] = (sin(xd[5]));
a[5] = ((real_t)(0.0000000000000000e+00)-(((a[3]*od[6])*a[4])*u[2]));
a[6] = (sin(xd[4]));
a[7] = ((a[6]*od[6])*u[2]);
a[8] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[3])));
a[9] = (cos(xd[5]));
a[10] = (sin(xd[4]));
a[11] = (cos(xd[3]));
a[12] = (sin(xd[5]));
a[13] = (cos(xd[3]));
a[14] = (cos(xd[4]));
a[15] = (cos(xd[4]));
a[16] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[4])));
a[17] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[4])));
a[18] = (((((a[15]*od[6])*u[2])*xd[2])+((((a[16]*a[1])*od[6])*u[2])*xd[0]))-((((a[17]*od[6])*a[4])*u[2])*xd[1]));
a[19] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[5])));
a[20] = (sin(xd[3]));
a[21] = (cos(xd[5]));
a[22] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[5])));
a[23] = (cos(xd[5]));
a[24] = (((((a[0]*a[22])*od[6])*u[2])*xd[0])-((((a[3]*od[6])*a[23])*u[2])*xd[1]));
a[25] = ((((a[6]*od[6])*xd[2])+(((a[0]*a[1])*od[6])*xd[0]))-(((a[3]*od[6])*a[4])*xd[1]));
a[26] = (cos(xd[3]));
a[27] = (sin(xd[5]));
a[28] = (cos(xd[5]));
a[29] = (sin(xd[4]));
a[30] = (sin(xd[3]));
a[31] = ((((a[26]*a[27])-((a[28]*a[29])*a[30]))*od[7])*u[2]);
a[32] = (cos(xd[3]));
a[33] = (cos(xd[5]));
a[34] = (sin(xd[4]));
a[35] = (sin(xd[3]));
a[36] = (sin(xd[5]));
a[37] = ((real_t)(0.0000000000000000e+00)-((((a[32]*a[33])+((a[34]*a[35])*a[36]))*od[7])*u[2]));
a[38] = (cos(xd[4]));
a[39] = (sin(xd[3]));
a[40] = ((real_t)(0.0000000000000000e+00)-(((a[38]*od[7])*a[39])*u[2]));
a[41] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[3])));
a[42] = (sin(xd[4]));
a[43] = (sin(xd[5]));
a[44] = (cos(xd[5]));
a[45] = (cos(xd[3]));
a[46] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[3])));
a[47] = (cos(xd[3]));
a[48] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[3])));
a[49] = (cos(xd[3]));
a[50] = (cos(xd[3]));
a[51] = (((((((a[46]*a[27])-((a[28]*a[29])*a[47]))*od[7])*u[2])*xd[0])-(((((a[48]*a[33])+((a[34]*a[49])*a[36]))*od[7])*u[2])*xd[1]))-((((a[38]*od[7])*a[50])*u[2])*xd[2]));
a[52] = (cos(xd[3]));
a[53] = (cos(xd[4]));
a[54] = (cos(xd[4]));
a[55] = (cos(xd[4]));
a[56] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[4])));
a[57] = (((((((real_t)(0.0000000000000000e+00)-((a[28]*a[54])*a[30]))*od[7])*u[2])*xd[0])-(((((a[55]*a[35])*a[36])*od[7])*u[2])*xd[1]))-((((a[56]*od[7])*a[39])*u[2])*xd[2]));
a[58] = (cos(xd[5]));
a[59] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[5])));
a[60] = (sin(xd[3]));
a[61] = (cos(xd[5]));
a[62] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[5])));
a[63] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[5])));
a[64] = (cos(xd[5]));
a[65] = ((((((a[26]*a[61])-((a[62]*a[29])*a[30]))*od[7])*u[2])*xd[0])-(((((a[32]*a[63])+((a[34]*a[35])*a[64]))*od[7])*u[2])*xd[1]));
a[66] = ((((((a[26]*a[27])-((a[28]*a[29])*a[30]))*od[7])*xd[0])-((((a[32]*a[33])+((a[34]*a[35])*a[36]))*od[7])*xd[1]))-(((a[38]*od[7])*a[39])*xd[2]));
a[67] = (cos(xd[4]));
a[68] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[3])));
a[69] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[4])));
a[70] = (cos(xd[3]));

/* Compute outputs: */
out[0] = ((real_t)(0.0000000000000000e+00)-a[2]);
out[1] = ((real_t)(0.0000000000000000e+00)-a[5]);
out[2] = ((real_t)(0.0000000000000000e+00)-a[7]);
out[3] = ((((a[8]*a[9])*a[10])+(a[11]*a[12]))*u[2]);
out[4] = ((((a[13]*a[9])*a[14])*u[2])-a[18]);
out[5] = (((((a[13]*a[19])*a[10])+(a[20]*a[21]))*u[2])-a[24]);
out[6] = (real_t)(0.0000000000000000e+00);
out[7] = (real_t)(0.0000000000000000e+00);
out[8] = (real_t)(0.0000000000000000e+00);
out[9] = (real_t)(0.0000000000000000e+00);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = ((((a[13]*a[9])*a[10])+(a[20]*a[12]))-a[25]);
out[15] = ((real_t)(0.0000000000000000e+00)-a[31]);
out[16] = ((real_t)(0.0000000000000000e+00)-a[37]);
out[17] = ((real_t)(0.0000000000000000e+00)-a[40]);
out[18] = (((((a[41]*a[42])*a[43])-(a[44]*a[45]))*u[2])-a[51]);
out[19] = ((((a[52]*a[53])*a[43])*u[2])-a[57]);
out[20] = (((((a[52]*a[42])*a[58])-(a[59]*a[60]))*u[2])-a[65]);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(0.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = ((((a[52]*a[42])*a[43])-(a[44]*a[60]))-a[66]);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = ((a[67]*a[68])*u[2]);
out[34] = ((a[69]*a[70])*u[2]);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = (real_t)(0.0000000000000000e+00);
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = (real_t)(0.0000000000000000e+00);
out[40] = (real_t)(0.0000000000000000e+00);
out[41] = (real_t)(0.0000000000000000e+00);
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = (real_t)(0.0000000000000000e+00);
out[44] = (a[67]*a[70]);
out[45] = (real_t)(0.0000000000000000e+00);
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (real_t)(0.0000000000000000e+00);
out[49] = (real_t)(0.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(0.0000000000000000e+00);
out[52] = (real_t)(0.0000000000000000e+00);
out[53] = (real_t)(0.0000000000000000e+00);
out[54] = (real_t)(1.0000000000000000e+00);
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
out[70] = (real_t)(1.0000000000000000e+00);
out[71] = (real_t)(0.0000000000000000e+00);
out[72] = (real_t)(0.0000000000000000e+00);
out[73] = (real_t)(0.0000000000000000e+00);
out[74] = (real_t)(0.0000000000000000e+00);
out[75] = (real_t)(0.0000000000000000e+00);
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
out[86] = (real_t)(1.0000000000000000e+00);
out[87] = (real_t)(0.0000000000000000e+00);
out[88] = (real_t)(0.0000000000000000e+00);
out[89] = (real_t)(0.0000000000000000e+00);
out[90] = (real_t)(1.0000000000000000e+00);
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
out[101] = (real_t)(0.0000000000000000e+00);
out[102] = (real_t)(0.0000000000000000e+00);
out[103] = (real_t)(0.0000000000000000e+00);
out[104] = (real_t)(0.0000000000000000e+00);
out[105] = (real_t)(0.0000000000000000e+00);
out[106] = (real_t)(1.0000000000000000e+00);
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
out[118] = (real_t)(0.0000000000000000e+00);
out[119] = (real_t)(0.0000000000000000e+00);
out[120] = (real_t)(0.0000000000000000e+00);
out[121] = (real_t)(0.0000000000000000e+00);
out[122] = (real_t)(1.0000000000000000e+00);
out[123] = (real_t)(0.0000000000000000e+00);
out[124] = (real_t)(0.0000000000000000e+00);
out[125] = (real_t)(0.0000000000000000e+00);
out[126] = (real_t)(0.0000000000000000e+00);
out[127] = (real_t)(0.0000000000000000e+00);
out[128] = (real_t)(0.0000000000000000e+00);
out[129] = (real_t)(0.0000000000000000e+00);
out[130] = (real_t)(0.0000000000000000e+00);
out[131] = (real_t)(0.0000000000000000e+00);
out[132] = (real_t)(0.0000000000000000e+00);
out[133] = (real_t)(0.0000000000000000e+00);
out[134] = (real_t)(0.0000000000000000e+00);
out[135] = (real_t)(0.0000000000000000e+00);
out[136] = (real_t)(0.0000000000000000e+00);
out[137] = (real_t)(0.0000000000000000e+00);
out[138] = ((real_t)(0.0000000000000000e+00)-(od[1]*od[1]));
out[139] = (real_t)(0.0000000000000000e+00);
out[140] = (real_t)(0.0000000000000000e+00);
out[141] = (real_t)(0.0000000000000000e+00);
out[142] = (real_t)(0.0000000000000000e+00);
out[143] = (real_t)(0.0000000000000000e+00);
out[144] = ((real_t)(0.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*od[0])*od[1]));
out[145] = (real_t)(0.0000000000000000e+00);
out[146] = (real_t)(0.0000000000000000e+00);
out[147] = ((od[2]*od[1])*od[1]);
out[148] = (real_t)(0.0000000000000000e+00);
out[149] = (real_t)(0.0000000000000000e+00);
out[150] = (real_t)(0.0000000000000000e+00);
out[151] = (real_t)(0.0000000000000000e+00);
out[152] = (real_t)(0.0000000000000000e+00);
out[153] = (real_t)(0.0000000000000000e+00);
out[154] = ((real_t)(0.0000000000000000e+00)-(od[4]*od[4]));
out[155] = (real_t)(0.0000000000000000e+00);
out[156] = (real_t)(0.0000000000000000e+00);
out[157] = (real_t)(0.0000000000000000e+00);
out[158] = (real_t)(0.0000000000000000e+00);
out[159] = (real_t)(0.0000000000000000e+00);
out[160] = ((real_t)(0.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*od[3])*od[4]));
out[161] = (real_t)(0.0000000000000000e+00);
out[162] = (real_t)(0.0000000000000000e+00);
out[163] = ((od[5]*od[4])*od[4]);
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
}



void acado_solve_dim12_triangular( real_t* const A, real_t* const b )
{

b[11] = b[11]/A[143];
b[10] -= + A[131]*b[11];
b[10] = b[10]/A[130];
b[9] -= + A[119]*b[11];
b[9] -= + A[118]*b[10];
b[9] = b[9]/A[117];
b[8] -= + A[107]*b[11];
b[8] -= + A[106]*b[10];
b[8] -= + A[105]*b[9];
b[8] = b[8]/A[104];
b[7] -= + A[95]*b[11];
b[7] -= + A[94]*b[10];
b[7] -= + A[93]*b[9];
b[7] -= + A[92]*b[8];
b[7] = b[7]/A[91];
b[6] -= + A[83]*b[11];
b[6] -= + A[82]*b[10];
b[6] -= + A[81]*b[9];
b[6] -= + A[80]*b[8];
b[6] -= + A[79]*b[7];
b[6] = b[6]/A[78];
b[5] -= + A[71]*b[11];
b[5] -= + A[70]*b[10];
b[5] -= + A[69]*b[9];
b[5] -= + A[68]*b[8];
b[5] -= + A[67]*b[7];
b[5] -= + A[66]*b[6];
b[5] = b[5]/A[65];
b[4] -= + A[59]*b[11];
b[4] -= + A[58]*b[10];
b[4] -= + A[57]*b[9];
b[4] -= + A[56]*b[8];
b[4] -= + A[55]*b[7];
b[4] -= + A[54]*b[6];
b[4] -= + A[53]*b[5];
b[4] = b[4]/A[52];
b[3] -= + A[47]*b[11];
b[3] -= + A[46]*b[10];
b[3] -= + A[45]*b[9];
b[3] -= + A[44]*b[8];
b[3] -= + A[43]*b[7];
b[3] -= + A[42]*b[6];
b[3] -= + A[41]*b[5];
b[3] -= + A[40]*b[4];
b[3] = b[3]/A[39];
b[2] -= + A[35]*b[11];
b[2] -= + A[34]*b[10];
b[2] -= + A[33]*b[9];
b[2] -= + A[32]*b[8];
b[2] -= + A[31]*b[7];
b[2] -= + A[30]*b[6];
b[2] -= + A[29]*b[5];
b[2] -= + A[28]*b[4];
b[2] -= + A[27]*b[3];
b[2] = b[2]/A[26];
b[1] -= + A[23]*b[11];
b[1] -= + A[22]*b[10];
b[1] -= + A[21]*b[9];
b[1] -= + A[20]*b[8];
b[1] -= + A[19]*b[7];
b[1] -= + A[18]*b[6];
b[1] -= + A[17]*b[5];
b[1] -= + A[16]*b[4];
b[1] -= + A[15]*b[3];
b[1] -= + A[14]*b[2];
b[1] = b[1]/A[13];
b[0] -= + A[11]*b[11];
b[0] -= + A[10]*b[10];
b[0] -= + A[9]*b[9];
b[0] -= + A[8]*b[8];
b[0] -= + A[7]*b[7];
b[0] -= + A[6]*b[6];
b[0] -= + A[5]*b[5];
b[0] -= + A[4]*b[4];
b[0] -= + A[3]*b[3];
b[0] -= + A[2]*b[2];
b[0] -= + A[1]*b[1];
b[0] = b[0]/A[0];
}

real_t acado_solve_dim12_system( real_t* const A, real_t* const b, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 12; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
for( i=0; i < (11); i++ ) {
	indexMax = i;
	valueMax = fabs(A[i*12+i]);
	for( j=(i+1); j < 12; j++ ) {
		temp = fabs(A[j*12+i]);
		if( temp > valueMax ) {
			indexMax = j;
			valueMax = temp;
		}
	}
	if( indexMax > i ) {
for (k = 0; k < 12; ++k)
{
	rk_dim12_swap = A[i*12+k];
	A[i*12+k] = A[indexMax*12+k];
	A[indexMax*12+k] = rk_dim12_swap;
}
	rk_dim12_swap = b[i];
	b[i] = b[indexMax];
	b[indexMax] = rk_dim12_swap;
	intSwap = rk_perm[i];
	rk_perm[i] = rk_perm[indexMax];
	rk_perm[indexMax] = intSwap;
	}
	det *= A[i*12+i];
	for( j=i+1; j < 12; j++ ) {
		A[j*12+i] = -A[j*12+i]/A[i*12+i];
		for( k=i+1; k < 12; k++ ) {
			A[j*12+k] += A[j*12+i] * A[i*12+k];
		}
		b[j] += A[j*12+i] * b[i];
	}
}
det *= A[143];
det = fabs(det);
acado_solve_dim12_triangular( A, b );
return det;
}

void acado_solve_dim12_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{

rk_dim12_bPerm[0] = b[rk_perm[0]];
rk_dim12_bPerm[1] = b[rk_perm[1]];
rk_dim12_bPerm[2] = b[rk_perm[2]];
rk_dim12_bPerm[3] = b[rk_perm[3]];
rk_dim12_bPerm[4] = b[rk_perm[4]];
rk_dim12_bPerm[5] = b[rk_perm[5]];
rk_dim12_bPerm[6] = b[rk_perm[6]];
rk_dim12_bPerm[7] = b[rk_perm[7]];
rk_dim12_bPerm[8] = b[rk_perm[8]];
rk_dim12_bPerm[9] = b[rk_perm[9]];
rk_dim12_bPerm[10] = b[rk_perm[10]];
rk_dim12_bPerm[11] = b[rk_perm[11]];
rk_dim12_bPerm[1] += A[12]*rk_dim12_bPerm[0];

rk_dim12_bPerm[2] += A[24]*rk_dim12_bPerm[0];
rk_dim12_bPerm[2] += A[25]*rk_dim12_bPerm[1];

rk_dim12_bPerm[3] += A[36]*rk_dim12_bPerm[0];
rk_dim12_bPerm[3] += A[37]*rk_dim12_bPerm[1];
rk_dim12_bPerm[3] += A[38]*rk_dim12_bPerm[2];

rk_dim12_bPerm[4] += A[48]*rk_dim12_bPerm[0];
rk_dim12_bPerm[4] += A[49]*rk_dim12_bPerm[1];
rk_dim12_bPerm[4] += A[50]*rk_dim12_bPerm[2];
rk_dim12_bPerm[4] += A[51]*rk_dim12_bPerm[3];

rk_dim12_bPerm[5] += A[60]*rk_dim12_bPerm[0];
rk_dim12_bPerm[5] += A[61]*rk_dim12_bPerm[1];
rk_dim12_bPerm[5] += A[62]*rk_dim12_bPerm[2];
rk_dim12_bPerm[5] += A[63]*rk_dim12_bPerm[3];
rk_dim12_bPerm[5] += A[64]*rk_dim12_bPerm[4];

rk_dim12_bPerm[6] += A[72]*rk_dim12_bPerm[0];
rk_dim12_bPerm[6] += A[73]*rk_dim12_bPerm[1];
rk_dim12_bPerm[6] += A[74]*rk_dim12_bPerm[2];
rk_dim12_bPerm[6] += A[75]*rk_dim12_bPerm[3];
rk_dim12_bPerm[6] += A[76]*rk_dim12_bPerm[4];
rk_dim12_bPerm[6] += A[77]*rk_dim12_bPerm[5];

rk_dim12_bPerm[7] += A[84]*rk_dim12_bPerm[0];
rk_dim12_bPerm[7] += A[85]*rk_dim12_bPerm[1];
rk_dim12_bPerm[7] += A[86]*rk_dim12_bPerm[2];
rk_dim12_bPerm[7] += A[87]*rk_dim12_bPerm[3];
rk_dim12_bPerm[7] += A[88]*rk_dim12_bPerm[4];
rk_dim12_bPerm[7] += A[89]*rk_dim12_bPerm[5];
rk_dim12_bPerm[7] += A[90]*rk_dim12_bPerm[6];

rk_dim12_bPerm[8] += A[96]*rk_dim12_bPerm[0];
rk_dim12_bPerm[8] += A[97]*rk_dim12_bPerm[1];
rk_dim12_bPerm[8] += A[98]*rk_dim12_bPerm[2];
rk_dim12_bPerm[8] += A[99]*rk_dim12_bPerm[3];
rk_dim12_bPerm[8] += A[100]*rk_dim12_bPerm[4];
rk_dim12_bPerm[8] += A[101]*rk_dim12_bPerm[5];
rk_dim12_bPerm[8] += A[102]*rk_dim12_bPerm[6];
rk_dim12_bPerm[8] += A[103]*rk_dim12_bPerm[7];

rk_dim12_bPerm[9] += A[108]*rk_dim12_bPerm[0];
rk_dim12_bPerm[9] += A[109]*rk_dim12_bPerm[1];
rk_dim12_bPerm[9] += A[110]*rk_dim12_bPerm[2];
rk_dim12_bPerm[9] += A[111]*rk_dim12_bPerm[3];
rk_dim12_bPerm[9] += A[112]*rk_dim12_bPerm[4];
rk_dim12_bPerm[9] += A[113]*rk_dim12_bPerm[5];
rk_dim12_bPerm[9] += A[114]*rk_dim12_bPerm[6];
rk_dim12_bPerm[9] += A[115]*rk_dim12_bPerm[7];
rk_dim12_bPerm[9] += A[116]*rk_dim12_bPerm[8];

rk_dim12_bPerm[10] += A[120]*rk_dim12_bPerm[0];
rk_dim12_bPerm[10] += A[121]*rk_dim12_bPerm[1];
rk_dim12_bPerm[10] += A[122]*rk_dim12_bPerm[2];
rk_dim12_bPerm[10] += A[123]*rk_dim12_bPerm[3];
rk_dim12_bPerm[10] += A[124]*rk_dim12_bPerm[4];
rk_dim12_bPerm[10] += A[125]*rk_dim12_bPerm[5];
rk_dim12_bPerm[10] += A[126]*rk_dim12_bPerm[6];
rk_dim12_bPerm[10] += A[127]*rk_dim12_bPerm[7];
rk_dim12_bPerm[10] += A[128]*rk_dim12_bPerm[8];
rk_dim12_bPerm[10] += A[129]*rk_dim12_bPerm[9];

rk_dim12_bPerm[11] += A[132]*rk_dim12_bPerm[0];
rk_dim12_bPerm[11] += A[133]*rk_dim12_bPerm[1];
rk_dim12_bPerm[11] += A[134]*rk_dim12_bPerm[2];
rk_dim12_bPerm[11] += A[135]*rk_dim12_bPerm[3];
rk_dim12_bPerm[11] += A[136]*rk_dim12_bPerm[4];
rk_dim12_bPerm[11] += A[137]*rk_dim12_bPerm[5];
rk_dim12_bPerm[11] += A[138]*rk_dim12_bPerm[6];
rk_dim12_bPerm[11] += A[139]*rk_dim12_bPerm[7];
rk_dim12_bPerm[11] += A[140]*rk_dim12_bPerm[8];
rk_dim12_bPerm[11] += A[141]*rk_dim12_bPerm[9];
rk_dim12_bPerm[11] += A[142]*rk_dim12_bPerm[10];


acado_solve_dim12_triangular( A, rk_dim12_bPerm );
b[0] = rk_dim12_bPerm[0];
b[1] = rk_dim12_bPerm[1];
b[2] = rk_dim12_bPerm[2];
b[3] = rk_dim12_bPerm[3];
b[4] = rk_dim12_bPerm[4];
b[5] = rk_dim12_bPerm[5];
b[6] = rk_dim12_bPerm[6];
b[7] = rk_dim12_bPerm[7];
b[8] = rk_dim12_bPerm[8];
b[9] = rk_dim12_bPerm[9];
b[10] = rk_dim12_bPerm[10];
b[11] = rk_dim12_bPerm[11];
}



/** Column vector of size: 1 */
static const real_t acado_Ah_mat[ 1 ] = 
{ 8.3333333333333332e-03 };


/* Fixed step size:0.0166667 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int i;
int j;
int k;
int run;
int run1;
int tmp_index1;
int tmp_index2;

real_t det;

rk_ttt = 0.0000000000000000e+00;
rk_xxx[12] = rk_eta[192];
rk_xxx[13] = rk_eta[193];
rk_xxx[14] = rk_eta[194];
rk_xxx[15] = rk_eta[195];
rk_xxx[16] = rk_eta[196];
rk_xxx[17] = rk_eta[197];
rk_xxx[18] = rk_eta[198];
rk_xxx[19] = rk_eta[199];
rk_xxx[20] = rk_eta[200];
rk_xxx[21] = rk_eta[201];
rk_xxx[22] = rk_eta[202];
rk_xxx[23] = rk_eta[203];
rk_xxx[24] = rk_eta[204];
rk_xxx[25] = rk_eta[205];
rk_xxx[26] = rk_eta[206];
rk_xxx[27] = rk_eta[207];
rk_xxx[28] = rk_eta[208];

for (run = 0; run < 6; ++run)
{
if( run > 0 ) {
for (i = 0; i < 12; ++i)
{
rk_diffsPrev2[i * 15] = rk_eta[i * 12 + 12];
rk_diffsPrev2[i * 15 + 1] = rk_eta[i * 12 + 13];
rk_diffsPrev2[i * 15 + 2] = rk_eta[i * 12 + 14];
rk_diffsPrev2[i * 15 + 3] = rk_eta[i * 12 + 15];
rk_diffsPrev2[i * 15 + 4] = rk_eta[i * 12 + 16];
rk_diffsPrev2[i * 15 + 5] = rk_eta[i * 12 + 17];
rk_diffsPrev2[i * 15 + 6] = rk_eta[i * 12 + 18];
rk_diffsPrev2[i * 15 + 7] = rk_eta[i * 12 + 19];
rk_diffsPrev2[i * 15 + 8] = rk_eta[i * 12 + 20];
rk_diffsPrev2[i * 15 + 9] = rk_eta[i * 12 + 21];
rk_diffsPrev2[i * 15 + 10] = rk_eta[i * 12 + 22];
rk_diffsPrev2[i * 15 + 11] = rk_eta[i * 12 + 23];
rk_diffsPrev2[i * 15 + 12] = rk_eta[i * 3 + 156];
rk_diffsPrev2[i * 15 + 13] = rk_eta[i * 3 + 157];
rk_diffsPrev2[i * 15 + 14] = rk_eta[i * 3 + 158];
}
}
if( resetIntegrator ) {
for (i = 0; i < 1; ++i)
{
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 12; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1]*rk_kkk[tmp_index1];
}
acado_diffs( rk_xxx, &(rk_diffsTemp2[ run1 * 180 ]) );
for (j = 0; j < 12; ++j)
{
tmp_index1 = (run1 * 12) + (j);
rk_A[tmp_index1 * 12] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 180) + (j * 15)];
rk_A[tmp_index1 * 12 + 1] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 180) + (j * 15 + 1)];
rk_A[tmp_index1 * 12 + 2] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 180) + (j * 15 + 2)];
rk_A[tmp_index1 * 12 + 3] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 180) + (j * 15 + 3)];
rk_A[tmp_index1 * 12 + 4] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 180) + (j * 15 + 4)];
rk_A[tmp_index1 * 12 + 5] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 180) + (j * 15 + 5)];
rk_A[tmp_index1 * 12 + 6] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 180) + (j * 15 + 6)];
rk_A[tmp_index1 * 12 + 7] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 180) + (j * 15 + 7)];
rk_A[tmp_index1 * 12 + 8] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 180) + (j * 15 + 8)];
rk_A[tmp_index1 * 12 + 9] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 180) + (j * 15 + 9)];
rk_A[tmp_index1 * 12 + 10] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 180) + (j * 15 + 10)];
rk_A[tmp_index1 * 12 + 11] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 180) + (j * 15 + 11)];
if( 0 == run1 ) rk_A[(tmp_index1 * 12) + (j)] -= 1.0000000000000000e+00;
}
acado_rhs( rk_xxx, rk_rhsTemp );
rk_b[run1 * 12] = rk_kkk[run1] - rk_rhsTemp[0];
rk_b[run1 * 12 + 1] = rk_kkk[run1 + 1] - rk_rhsTemp[1];
rk_b[run1 * 12 + 2] = rk_kkk[run1 + 2] - rk_rhsTemp[2];
rk_b[run1 * 12 + 3] = rk_kkk[run1 + 3] - rk_rhsTemp[3];
rk_b[run1 * 12 + 4] = rk_kkk[run1 + 4] - rk_rhsTemp[4];
rk_b[run1 * 12 + 5] = rk_kkk[run1 + 5] - rk_rhsTemp[5];
rk_b[run1 * 12 + 6] = rk_kkk[run1 + 6] - rk_rhsTemp[6];
rk_b[run1 * 12 + 7] = rk_kkk[run1 + 7] - rk_rhsTemp[7];
rk_b[run1 * 12 + 8] = rk_kkk[run1 + 8] - rk_rhsTemp[8];
rk_b[run1 * 12 + 9] = rk_kkk[run1 + 9] - rk_rhsTemp[9];
rk_b[run1 * 12 + 10] = rk_kkk[run1 + 10] - rk_rhsTemp[10];
rk_b[run1 * 12 + 11] = rk_kkk[run1 + 11] - rk_rhsTemp[11];
}
det = acado_solve_dim12_system( rk_A, rk_b, rk_dim12_perm );
for (j = 0; j < 1; ++j)
{
rk_kkk[j] += rk_b[j * 12];
rk_kkk[j + 1] += rk_b[j * 12 + 1];
rk_kkk[j + 2] += rk_b[j * 12 + 2];
rk_kkk[j + 3] += rk_b[j * 12 + 3];
rk_kkk[j + 4] += rk_b[j * 12 + 4];
rk_kkk[j + 5] += rk_b[j * 12 + 5];
rk_kkk[j + 6] += rk_b[j * 12 + 6];
rk_kkk[j + 7] += rk_b[j * 12 + 7];
rk_kkk[j + 8] += rk_b[j * 12 + 8];
rk_kkk[j + 9] += rk_b[j * 12 + 9];
rk_kkk[j + 10] += rk_b[j * 12 + 10];
rk_kkk[j + 11] += rk_b[j * 12 + 11];
}
}
}
for (i = 0; i < 2; ++i)
{
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 12; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1]*rk_kkk[tmp_index1];
}
acado_rhs( rk_xxx, rk_rhsTemp );
rk_b[run1 * 12] = rk_kkk[run1] - rk_rhsTemp[0];
rk_b[run1 * 12 + 1] = rk_kkk[run1 + 1] - rk_rhsTemp[1];
rk_b[run1 * 12 + 2] = rk_kkk[run1 + 2] - rk_rhsTemp[2];
rk_b[run1 * 12 + 3] = rk_kkk[run1 + 3] - rk_rhsTemp[3];
rk_b[run1 * 12 + 4] = rk_kkk[run1 + 4] - rk_rhsTemp[4];
rk_b[run1 * 12 + 5] = rk_kkk[run1 + 5] - rk_rhsTemp[5];
rk_b[run1 * 12 + 6] = rk_kkk[run1 + 6] - rk_rhsTemp[6];
rk_b[run1 * 12 + 7] = rk_kkk[run1 + 7] - rk_rhsTemp[7];
rk_b[run1 * 12 + 8] = rk_kkk[run1 + 8] - rk_rhsTemp[8];
rk_b[run1 * 12 + 9] = rk_kkk[run1 + 9] - rk_rhsTemp[9];
rk_b[run1 * 12 + 10] = rk_kkk[run1 + 10] - rk_rhsTemp[10];
rk_b[run1 * 12 + 11] = rk_kkk[run1 + 11] - rk_rhsTemp[11];
}
acado_solve_dim12_system_reuse( rk_A, rk_b, rk_dim12_perm );
for (j = 0; j < 1; ++j)
{
rk_kkk[j] += rk_b[j * 12];
rk_kkk[j + 1] += rk_b[j * 12 + 1];
rk_kkk[j + 2] += rk_b[j * 12 + 2];
rk_kkk[j + 3] += rk_b[j * 12 + 3];
rk_kkk[j + 4] += rk_b[j * 12 + 4];
rk_kkk[j + 5] += rk_b[j * 12 + 5];
rk_kkk[j + 6] += rk_b[j * 12 + 6];
rk_kkk[j + 7] += rk_b[j * 12 + 7];
rk_kkk[j + 8] += rk_b[j * 12 + 8];
rk_kkk[j + 9] += rk_b[j * 12 + 9];
rk_kkk[j + 10] += rk_b[j * 12 + 10];
rk_kkk[j + 11] += rk_b[j * 12 + 11];
}
}
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 12; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1]*rk_kkk[tmp_index1];
}
acado_diffs( rk_xxx, &(rk_diffsTemp2[ run1 * 180 ]) );
for (j = 0; j < 12; ++j)
{
tmp_index1 = (run1 * 12) + (j);
rk_A[tmp_index1 * 12] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 180) + (j * 15)];
rk_A[tmp_index1 * 12 + 1] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 180) + (j * 15 + 1)];
rk_A[tmp_index1 * 12 + 2] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 180) + (j * 15 + 2)];
rk_A[tmp_index1 * 12 + 3] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 180) + (j * 15 + 3)];
rk_A[tmp_index1 * 12 + 4] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 180) + (j * 15 + 4)];
rk_A[tmp_index1 * 12 + 5] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 180) + (j * 15 + 5)];
rk_A[tmp_index1 * 12 + 6] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 180) + (j * 15 + 6)];
rk_A[tmp_index1 * 12 + 7] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 180) + (j * 15 + 7)];
rk_A[tmp_index1 * 12 + 8] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 180) + (j * 15 + 8)];
rk_A[tmp_index1 * 12 + 9] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 180) + (j * 15 + 9)];
rk_A[tmp_index1 * 12 + 10] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 180) + (j * 15 + 10)];
rk_A[tmp_index1 * 12 + 11] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 180) + (j * 15 + 11)];
if( 0 == run1 ) rk_A[(tmp_index1 * 12) + (j)] -= 1.0000000000000000e+00;
}
}
for (run1 = 0; run1 < 12; ++run1)
{
for (i = 0; i < 1; ++i)
{
rk_b[i * 12] = - rk_diffsTemp2[(i * 180) + (run1)];
rk_b[i * 12 + 1] = - rk_diffsTemp2[(i * 180) + (run1 + 15)];
rk_b[i * 12 + 2] = - rk_diffsTemp2[(i * 180) + (run1 + 30)];
rk_b[i * 12 + 3] = - rk_diffsTemp2[(i * 180) + (run1 + 45)];
rk_b[i * 12 + 4] = - rk_diffsTemp2[(i * 180) + (run1 + 60)];
rk_b[i * 12 + 5] = - rk_diffsTemp2[(i * 180) + (run1 + 75)];
rk_b[i * 12 + 6] = - rk_diffsTemp2[(i * 180) + (run1 + 90)];
rk_b[i * 12 + 7] = - rk_diffsTemp2[(i * 180) + (run1 + 105)];
rk_b[i * 12 + 8] = - rk_diffsTemp2[(i * 180) + (run1 + 120)];
rk_b[i * 12 + 9] = - rk_diffsTemp2[(i * 180) + (run1 + 135)];
rk_b[i * 12 + 10] = - rk_diffsTemp2[(i * 180) + (run1 + 150)];
rk_b[i * 12 + 11] = - rk_diffsTemp2[(i * 180) + (run1 + 165)];
}
if( 0 == run1 ) {
det = acado_solve_dim12_system( rk_A, rk_b, rk_dim12_perm );
}
 else {
acado_solve_dim12_system_reuse( rk_A, rk_b, rk_dim12_perm );
}
for (i = 0; i < 1; ++i)
{
rk_diffK[i] = rk_b[i * 12];
rk_diffK[i + 1] = rk_b[i * 12 + 1];
rk_diffK[i + 2] = rk_b[i * 12 + 2];
rk_diffK[i + 3] = rk_b[i * 12 + 3];
rk_diffK[i + 4] = rk_b[i * 12 + 4];
rk_diffK[i + 5] = rk_b[i * 12 + 5];
rk_diffK[i + 6] = rk_b[i * 12 + 6];
rk_diffK[i + 7] = rk_b[i * 12 + 7];
rk_diffK[i + 8] = rk_b[i * 12 + 8];
rk_diffK[i + 9] = rk_b[i * 12 + 9];
rk_diffK[i + 10] = rk_b[i * 12 + 10];
rk_diffK[i + 11] = rk_b[i * 12 + 11];
}
for (i = 0; i < 12; ++i)
{
rk_diffsNew2[(i * 15) + (run1)] = (i == run1-0);
rk_diffsNew2[(i * 15) + (run1)] += + rk_diffK[i]*(real_t)1.6666666666666666e-02;
}
}
for (run1 = 0; run1 < 3; ++run1)
{
for (i = 0; i < 1; ++i)
{
for (j = 0; j < 12; ++j)
{
tmp_index1 = (i * 12) + (j);
tmp_index2 = (run1) + (j * 15);
rk_b[tmp_index1] = - rk_diffsTemp2[(i * 180) + (tmp_index2 + 12)];
}
}
acado_solve_dim12_system_reuse( rk_A, rk_b, rk_dim12_perm );
for (i = 0; i < 1; ++i)
{
rk_diffK[i] = rk_b[i * 12];
rk_diffK[i + 1] = rk_b[i * 12 + 1];
rk_diffK[i + 2] = rk_b[i * 12 + 2];
rk_diffK[i + 3] = rk_b[i * 12 + 3];
rk_diffK[i + 4] = rk_b[i * 12 + 4];
rk_diffK[i + 5] = rk_b[i * 12 + 5];
rk_diffK[i + 6] = rk_b[i * 12 + 6];
rk_diffK[i + 7] = rk_b[i * 12 + 7];
rk_diffK[i + 8] = rk_b[i * 12 + 8];
rk_diffK[i + 9] = rk_b[i * 12 + 9];
rk_diffK[i + 10] = rk_b[i * 12 + 10];
rk_diffK[i + 11] = rk_b[i * 12 + 11];
}
for (i = 0; i < 12; ++i)
{
rk_diffsNew2[(i * 15) + (run1 + 12)] = + rk_diffK[i]*(real_t)1.6666666666666666e-02;
}
}
rk_eta[0] += + rk_kkk[0]*(real_t)1.6666666666666666e-02;
rk_eta[1] += + rk_kkk[1]*(real_t)1.6666666666666666e-02;
rk_eta[2] += + rk_kkk[2]*(real_t)1.6666666666666666e-02;
rk_eta[3] += + rk_kkk[3]*(real_t)1.6666666666666666e-02;
rk_eta[4] += + rk_kkk[4]*(real_t)1.6666666666666666e-02;
rk_eta[5] += + rk_kkk[5]*(real_t)1.6666666666666666e-02;
rk_eta[6] += + rk_kkk[6]*(real_t)1.6666666666666666e-02;
rk_eta[7] += + rk_kkk[7]*(real_t)1.6666666666666666e-02;
rk_eta[8] += + rk_kkk[8]*(real_t)1.6666666666666666e-02;
rk_eta[9] += + rk_kkk[9]*(real_t)1.6666666666666666e-02;
rk_eta[10] += + rk_kkk[10]*(real_t)1.6666666666666666e-02;
rk_eta[11] += + rk_kkk[11]*(real_t)1.6666666666666666e-02;
if( run == 0 ) {
for (i = 0; i < 12; ++i)
{
for (j = 0; j < 12; ++j)
{
tmp_index2 = (j) + (i * 12);
rk_eta[tmp_index2 + 12] = rk_diffsNew2[(i * 15) + (j)];
}
for (j = 0; j < 3; ++j)
{
tmp_index2 = (j) + (i * 3);
rk_eta[tmp_index2 + 156] = rk_diffsNew2[(i * 15) + (j + 12)];
}
}
}
else {
for (i = 0; i < 12; ++i)
{
for (j = 0; j < 12; ++j)
{
tmp_index2 = (j) + (i * 12);
rk_eta[tmp_index2 + 12] = + rk_diffsNew2[i * 15]*rk_diffsPrev2[j];
rk_eta[tmp_index2 + 12] += + rk_diffsNew2[i * 15 + 1]*rk_diffsPrev2[j + 15];
rk_eta[tmp_index2 + 12] += + rk_diffsNew2[i * 15 + 2]*rk_diffsPrev2[j + 30];
rk_eta[tmp_index2 + 12] += + rk_diffsNew2[i * 15 + 3]*rk_diffsPrev2[j + 45];
rk_eta[tmp_index2 + 12] += + rk_diffsNew2[i * 15 + 4]*rk_diffsPrev2[j + 60];
rk_eta[tmp_index2 + 12] += + rk_diffsNew2[i * 15 + 5]*rk_diffsPrev2[j + 75];
rk_eta[tmp_index2 + 12] += + rk_diffsNew2[i * 15 + 6]*rk_diffsPrev2[j + 90];
rk_eta[tmp_index2 + 12] += + rk_diffsNew2[i * 15 + 7]*rk_diffsPrev2[j + 105];
rk_eta[tmp_index2 + 12] += + rk_diffsNew2[i * 15 + 8]*rk_diffsPrev2[j + 120];
rk_eta[tmp_index2 + 12] += + rk_diffsNew2[i * 15 + 9]*rk_diffsPrev2[j + 135];
rk_eta[tmp_index2 + 12] += + rk_diffsNew2[i * 15 + 10]*rk_diffsPrev2[j + 150];
rk_eta[tmp_index2 + 12] += + rk_diffsNew2[i * 15 + 11]*rk_diffsPrev2[j + 165];
}
for (j = 0; j < 3; ++j)
{
tmp_index2 = (j) + (i * 3);
rk_eta[tmp_index2 + 156] = rk_diffsNew2[(i * 15) + (j + 12)];
rk_eta[tmp_index2 + 156] += + rk_diffsNew2[i * 15]*rk_diffsPrev2[j + 12];
rk_eta[tmp_index2 + 156] += + rk_diffsNew2[i * 15 + 1]*rk_diffsPrev2[j + 27];
rk_eta[tmp_index2 + 156] += + rk_diffsNew2[i * 15 + 2]*rk_diffsPrev2[j + 42];
rk_eta[tmp_index2 + 156] += + rk_diffsNew2[i * 15 + 3]*rk_diffsPrev2[j + 57];
rk_eta[tmp_index2 + 156] += + rk_diffsNew2[i * 15 + 4]*rk_diffsPrev2[j + 72];
rk_eta[tmp_index2 + 156] += + rk_diffsNew2[i * 15 + 5]*rk_diffsPrev2[j + 87];
rk_eta[tmp_index2 + 156] += + rk_diffsNew2[i * 15 + 6]*rk_diffsPrev2[j + 102];
rk_eta[tmp_index2 + 156] += + rk_diffsNew2[i * 15 + 7]*rk_diffsPrev2[j + 117];
rk_eta[tmp_index2 + 156] += + rk_diffsNew2[i * 15 + 8]*rk_diffsPrev2[j + 132];
rk_eta[tmp_index2 + 156] += + rk_diffsNew2[i * 15 + 9]*rk_diffsPrev2[j + 147];
rk_eta[tmp_index2 + 156] += + rk_diffsNew2[i * 15 + 10]*rk_diffsPrev2[j + 162];
rk_eta[tmp_index2 + 156] += + rk_diffsNew2[i * 15 + 11]*rk_diffsPrev2[j + 177];
}
}
}
resetIntegrator = 0;
rk_ttt += 1.6666666666666666e-01;
}
for (i = 0; i < 12; ++i)
{
}
if( det < 1e-12 ) {
error = 2;
} else if( det < 1e-6 ) {
error = 1;
} else {
error = 0;
}
return error;
}



