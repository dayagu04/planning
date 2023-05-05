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
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 4];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 4 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 4 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 4 + 3];

acadoWorkspace.state[24] = acadoVariables.u[lRun1];
acadoWorkspace.state[25] = acadoVariables.od[lRun1 * 6];
acadoWorkspace.state[26] = acadoVariables.od[lRun1 * 6 + 1];
acadoWorkspace.state[27] = acadoVariables.od[lRun1 * 6 + 2];
acadoWorkspace.state[28] = acadoVariables.od[lRun1 * 6 + 3];
acadoWorkspace.state[29] = acadoVariables.od[lRun1 * 6 + 4];
acadoWorkspace.state[30] = acadoVariables.od[lRun1 * 6 + 5];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 4] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 4 + 4];
acadoWorkspace.d[lRun1 * 4 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 4 + 5];
acadoWorkspace.d[lRun1 * 4 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 4 + 6];
acadoWorkspace.d[lRun1 * 4 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 4 + 7];

acadoWorkspace.evGx[lRun1 * 16] = acadoWorkspace.state[4];
acadoWorkspace.evGx[lRun1 * 16 + 1] = acadoWorkspace.state[5];
acadoWorkspace.evGx[lRun1 * 16 + 2] = acadoWorkspace.state[6];
acadoWorkspace.evGx[lRun1 * 16 + 3] = acadoWorkspace.state[7];
acadoWorkspace.evGx[lRun1 * 16 + 4] = acadoWorkspace.state[8];
acadoWorkspace.evGx[lRun1 * 16 + 5] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 16 + 6] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 16 + 7] = acadoWorkspace.state[11];
acadoWorkspace.evGx[lRun1 * 16 + 8] = acadoWorkspace.state[12];
acadoWorkspace.evGx[lRun1 * 16 + 9] = acadoWorkspace.state[13];
acadoWorkspace.evGx[lRun1 * 16 + 10] = acadoWorkspace.state[14];
acadoWorkspace.evGx[lRun1 * 16 + 11] = acadoWorkspace.state[15];
acadoWorkspace.evGx[lRun1 * 16 + 12] = acadoWorkspace.state[16];
acadoWorkspace.evGx[lRun1 * 16 + 13] = acadoWorkspace.state[17];
acadoWorkspace.evGx[lRun1 * 16 + 14] = acadoWorkspace.state[18];
acadoWorkspace.evGx[lRun1 * 16 + 15] = acadoWorkspace.state[19];

acadoWorkspace.evGu[lRun1 * 4] = acadoWorkspace.state[20];
acadoWorkspace.evGu[lRun1 * 4 + 1] = acadoWorkspace.state[21];
acadoWorkspace.evGu[lRun1 * 4 + 2] = acadoWorkspace.state[22];
acadoWorkspace.evGu[lRun1 * 4 + 3] = acadoWorkspace.state[23];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 4;
const real_t* od = in + 5;

/* Compute outputs: */
out[0] = (od[2]-xd[1]);
out[1] = (od[3]-xd[2]);
out[2] = u[0];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* od = in + 4;

/* Compute outputs: */
out[0] = (od[2]-xd[1]);
out[1] = (od[3]-xd[2]);
}

void acado_setObjQ1Q2( real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = 0.0;
;
tmpQ2[1] = 0.0;
;
tmpQ2[2] = 0.0;
;
tmpQ2[3] = + (real_t)-1.0000000000000000e+00*tmpObjS[0];
tmpQ2[4] = + (real_t)-1.0000000000000000e+00*tmpObjS[1];
tmpQ2[5] = + (real_t)-1.0000000000000000e+00*tmpObjS[2];
tmpQ2[6] = + (real_t)-1.0000000000000000e+00*tmpObjS[3];
tmpQ2[7] = + (real_t)-1.0000000000000000e+00*tmpObjS[4];
tmpQ2[8] = + (real_t)-1.0000000000000000e+00*tmpObjS[5];
tmpQ2[9] = 0.0;
;
tmpQ2[10] = 0.0;
;
tmpQ2[11] = 0.0;
;
tmpQ1[0] = 0.0;
;
tmpQ1[1] = + tmpQ2[0]*(real_t)-1.0000000000000000e+00;
tmpQ1[2] = + tmpQ2[1]*(real_t)-1.0000000000000000e+00;
tmpQ1[3] = 0.0;
;
tmpQ1[4] = 0.0;
;
tmpQ1[5] = + tmpQ2[3]*(real_t)-1.0000000000000000e+00;
tmpQ1[6] = + tmpQ2[4]*(real_t)-1.0000000000000000e+00;
tmpQ1[7] = 0.0;
;
tmpQ1[8] = 0.0;
;
tmpQ1[9] = + tmpQ2[6]*(real_t)-1.0000000000000000e+00;
tmpQ1[10] = + tmpQ2[7]*(real_t)-1.0000000000000000e+00;
tmpQ1[11] = 0.0;
;
tmpQ1[12] = 0.0;
;
tmpQ1[13] = + tmpQ2[9]*(real_t)-1.0000000000000000e+00;
tmpQ1[14] = + tmpQ2[10]*(real_t)-1.0000000000000000e+00;
tmpQ1[15] = 0.0;
;
}

void acado_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[6];
tmpR2[1] = +tmpObjS[7];
tmpR2[2] = +tmpObjS[8];
tmpR1[0] = + tmpR2[2];
}

void acado_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = 0.0;
;
tmpQN2[1] = 0.0;
;
tmpQN2[2] = + (real_t)-1.0000000000000000e+00*tmpObjSEndTerm[0];
tmpQN2[3] = + (real_t)-1.0000000000000000e+00*tmpObjSEndTerm[1];
tmpQN2[4] = + (real_t)-1.0000000000000000e+00*tmpObjSEndTerm[2];
tmpQN2[5] = + (real_t)-1.0000000000000000e+00*tmpObjSEndTerm[3];
tmpQN2[6] = 0.0;
;
tmpQN2[7] = 0.0;
;
tmpQN1[0] = 0.0;
;
tmpQN1[1] = + tmpQN2[0]*(real_t)-1.0000000000000000e+00;
tmpQN1[2] = + tmpQN2[1]*(real_t)-1.0000000000000000e+00;
tmpQN1[3] = 0.0;
;
tmpQN1[4] = 0.0;
;
tmpQN1[5] = + tmpQN2[2]*(real_t)-1.0000000000000000e+00;
tmpQN1[6] = + tmpQN2[3]*(real_t)-1.0000000000000000e+00;
tmpQN1[7] = 0.0;
;
tmpQN1[8] = 0.0;
;
tmpQN1[9] = + tmpQN2[4]*(real_t)-1.0000000000000000e+00;
tmpQN1[10] = + tmpQN2[5]*(real_t)-1.0000000000000000e+00;
tmpQN1[11] = 0.0;
;
tmpQN1[12] = 0.0;
;
tmpQN1[13] = + tmpQN2[6]*(real_t)-1.0000000000000000e+00;
tmpQN1[14] = + tmpQN2[7]*(real_t)-1.0000000000000000e+00;
tmpQN1[15] = 0.0;
;
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 25; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 4];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 4 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 4 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 4 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.u[runObj];
acadoWorkspace.objValueIn[5] = acadoVariables.od[runObj * 6];
acadoWorkspace.objValueIn[6] = acadoVariables.od[runObj * 6 + 1];
acadoWorkspace.objValueIn[7] = acadoVariables.od[runObj * 6 + 2];
acadoWorkspace.objValueIn[8] = acadoVariables.od[runObj * 6 + 3];
acadoWorkspace.objValueIn[9] = acadoVariables.od[runObj * 6 + 4];
acadoWorkspace.objValueIn[10] = acadoVariables.od[runObj * 6 + 5];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 3] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 3 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 3 + 2] = acadoWorkspace.objValueOut[2];

acado_setObjQ1Q2( &(acadoVariables.W[ runObj * 9 ]), &(acadoWorkspace.Q1[ runObj * 16 ]), &(acadoWorkspace.Q2[ runObj * 12 ]) );

acado_setObjR1R2( &(acadoVariables.W[ runObj * 9 ]), &(acadoWorkspace.R1[ runObj ]), &(acadoWorkspace.R2[ runObj * 3 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[100];
acadoWorkspace.objValueIn[1] = acadoVariables.x[101];
acadoWorkspace.objValueIn[2] = acadoVariables.x[102];
acadoWorkspace.objValueIn[3] = acadoVariables.x[103];
acadoWorkspace.objValueIn[4] = acadoVariables.od[150];
acadoWorkspace.objValueIn[5] = acadoVariables.od[151];
acadoWorkspace.objValueIn[6] = acadoVariables.od[152];
acadoWorkspace.objValueIn[7] = acadoVariables.od[153];
acadoWorkspace.objValueIn[8] = acadoVariables.od[154];
acadoWorkspace.objValueIn[9] = acadoVariables.od[155];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3];
dNew[1] += + Gx1[4]*dOld[0] + Gx1[5]*dOld[1] + Gx1[6]*dOld[2] + Gx1[7]*dOld[3];
dNew[2] += + Gx1[8]*dOld[0] + Gx1[9]*dOld[1] + Gx1[10]*dOld[2] + Gx1[11]*dOld[3];
dNew[3] += + Gx1[12]*dOld[0] + Gx1[13]*dOld[1] + Gx1[14]*dOld[2] + Gx1[15]*dOld[3];
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
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[4] + Gx1[2]*Gx2[8] + Gx1[3]*Gx2[12];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[5] + Gx1[2]*Gx2[9] + Gx1[3]*Gx2[13];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[6] + Gx1[2]*Gx2[10] + Gx1[3]*Gx2[14];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[7] + Gx1[2]*Gx2[11] + Gx1[3]*Gx2[15];
Gx3[4] = + Gx1[4]*Gx2[0] + Gx1[5]*Gx2[4] + Gx1[6]*Gx2[8] + Gx1[7]*Gx2[12];
Gx3[5] = + Gx1[4]*Gx2[1] + Gx1[5]*Gx2[5] + Gx1[6]*Gx2[9] + Gx1[7]*Gx2[13];
Gx3[6] = + Gx1[4]*Gx2[2] + Gx1[5]*Gx2[6] + Gx1[6]*Gx2[10] + Gx1[7]*Gx2[14];
Gx3[7] = + Gx1[4]*Gx2[3] + Gx1[5]*Gx2[7] + Gx1[6]*Gx2[11] + Gx1[7]*Gx2[15];
Gx3[8] = + Gx1[8]*Gx2[0] + Gx1[9]*Gx2[4] + Gx1[10]*Gx2[8] + Gx1[11]*Gx2[12];
Gx3[9] = + Gx1[8]*Gx2[1] + Gx1[9]*Gx2[5] + Gx1[10]*Gx2[9] + Gx1[11]*Gx2[13];
Gx3[10] = + Gx1[8]*Gx2[2] + Gx1[9]*Gx2[6] + Gx1[10]*Gx2[10] + Gx1[11]*Gx2[14];
Gx3[11] = + Gx1[8]*Gx2[3] + Gx1[9]*Gx2[7] + Gx1[10]*Gx2[11] + Gx1[11]*Gx2[15];
Gx3[12] = + Gx1[12]*Gx2[0] + Gx1[13]*Gx2[4] + Gx1[14]*Gx2[8] + Gx1[15]*Gx2[12];
Gx3[13] = + Gx1[12]*Gx2[1] + Gx1[13]*Gx2[5] + Gx1[14]*Gx2[9] + Gx1[15]*Gx2[13];
Gx3[14] = + Gx1[12]*Gx2[2] + Gx1[13]*Gx2[6] + Gx1[14]*Gx2[10] + Gx1[15]*Gx2[14];
Gx3[15] = + Gx1[12]*Gx2[3] + Gx1[13]*Gx2[7] + Gx1[14]*Gx2[11] + Gx1[15]*Gx2[15];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[1] + Gx1[2]*Gu1[2] + Gx1[3]*Gu1[3];
Gu2[1] = + Gx1[4]*Gu1[0] + Gx1[5]*Gu1[1] + Gx1[6]*Gu1[2] + Gx1[7]*Gu1[3];
Gu2[2] = + Gx1[8]*Gu1[0] + Gx1[9]*Gu1[1] + Gx1[10]*Gu1[2] + Gx1[11]*Gu1[3];
Gu2[3] = + Gx1[12]*Gu1[0] + Gx1[13]*Gu1[1] + Gx1[14]*Gu1[2] + Gx1[15]*Gu1[3];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
}

void acado_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
acadoWorkspace.H[(iRow * 29 + 116) + (iCol + 4)] += + Gu1[0]*Gu2[0] + Gu1[1]*Gu2[1] + Gu1[2]*Gu2[2] + Gu1[3]*Gu2[3];
}

void acado_setBlockH11_R1( int iRow, int iCol, real_t* const R11 )
{
acadoWorkspace.H[(iRow * 29 + 116) + (iCol + 4)] = R11[0];
}

void acado_zeroBlockH11( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 29 + 116) + (iCol + 4)] = 0.0000000000000000e+00;
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 29 + 116) + (iCol + 4)] = acadoWorkspace.H[(iCol * 29 + 116) + (iRow + 4)];
}

void acado_multQ1d( real_t* const Gx1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3];
dNew[1] = + Gx1[4]*dOld[0] + Gx1[5]*dOld[1] + Gx1[6]*dOld[2] + Gx1[7]*dOld[3];
dNew[2] = + Gx1[8]*dOld[0] + Gx1[9]*dOld[1] + Gx1[10]*dOld[2] + Gx1[11]*dOld[3];
dNew[3] = + Gx1[12]*dOld[0] + Gx1[13]*dOld[1] + Gx1[14]*dOld[2] + Gx1[15]*dOld[3];
}

void acado_multQN1d( real_t* const QN1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + acadoWorkspace.QN1[0]*dOld[0] + acadoWorkspace.QN1[1]*dOld[1] + acadoWorkspace.QN1[2]*dOld[2] + acadoWorkspace.QN1[3]*dOld[3];
dNew[1] = + acadoWorkspace.QN1[4]*dOld[0] + acadoWorkspace.QN1[5]*dOld[1] + acadoWorkspace.QN1[6]*dOld[2] + acadoWorkspace.QN1[7]*dOld[3];
dNew[2] = + acadoWorkspace.QN1[8]*dOld[0] + acadoWorkspace.QN1[9]*dOld[1] + acadoWorkspace.QN1[10]*dOld[2] + acadoWorkspace.QN1[11]*dOld[3];
dNew[3] = + acadoWorkspace.QN1[12]*dOld[0] + acadoWorkspace.QN1[13]*dOld[1] + acadoWorkspace.QN1[14]*dOld[2] + acadoWorkspace.QN1[15]*dOld[3];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2];
QDy1[1] = + Q2[3]*Dy1[0] + Q2[4]*Dy1[1] + Q2[5]*Dy1[2];
QDy1[2] = + Q2[6]*Dy1[0] + Q2[7]*Dy1[1] + Q2[8]*Dy1[2];
QDy1[3] = + Q2[9]*Dy1[0] + Q2[10]*Dy1[1] + Q2[11]*Dy1[2];
}

void acado_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[1]*QDy1[1] + E1[2]*QDy1[2] + E1[3]*QDy1[3];
}

void acado_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[1]*Gx1[4] + E1[2]*Gx1[8] + E1[3]*Gx1[12];
H101[1] += + E1[0]*Gx1[1] + E1[1]*Gx1[5] + E1[2]*Gx1[9] + E1[3]*Gx1[13];
H101[2] += + E1[0]*Gx1[2] + E1[1]*Gx1[6] + E1[2]*Gx1[10] + E1[3]*Gx1[14];
H101[3] += + E1[0]*Gx1[3] + E1[1]*Gx1[7] + E1[2]*Gx1[11] + E1[3]*Gx1[15];
}

void acado_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 4; lCopy++) H101[ lCopy ] = 0; }
}

void acado_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0];
dNew[1] += + E1[1]*U1[0];
dNew[2] += + E1[2]*U1[0];
dNew[3] += + E1[3]*U1[0];
}

void acado_zeroBlockH00(  )
{
acadoWorkspace.H[0] = 0.0000000000000000e+00;
acadoWorkspace.H[1] = 0.0000000000000000e+00;
acadoWorkspace.H[2] = 0.0000000000000000e+00;
acadoWorkspace.H[3] = 0.0000000000000000e+00;
acadoWorkspace.H[29] = 0.0000000000000000e+00;
acadoWorkspace.H[30] = 0.0000000000000000e+00;
acadoWorkspace.H[31] = 0.0000000000000000e+00;
acadoWorkspace.H[32] = 0.0000000000000000e+00;
acadoWorkspace.H[58] = 0.0000000000000000e+00;
acadoWorkspace.H[59] = 0.0000000000000000e+00;
acadoWorkspace.H[60] = 0.0000000000000000e+00;
acadoWorkspace.H[61] = 0.0000000000000000e+00;
acadoWorkspace.H[87] = 0.0000000000000000e+00;
acadoWorkspace.H[88] = 0.0000000000000000e+00;
acadoWorkspace.H[89] = 0.0000000000000000e+00;
acadoWorkspace.H[90] = 0.0000000000000000e+00;
}

void acado_multCTQC( real_t* const Gx1, real_t* const Gx2 )
{
acadoWorkspace.H[0] += + Gx1[0]*Gx2[0] + Gx1[4]*Gx2[4] + Gx1[8]*Gx2[8] + Gx1[12]*Gx2[12];
acadoWorkspace.H[1] += + Gx1[0]*Gx2[1] + Gx1[4]*Gx2[5] + Gx1[8]*Gx2[9] + Gx1[12]*Gx2[13];
acadoWorkspace.H[2] += + Gx1[0]*Gx2[2] + Gx1[4]*Gx2[6] + Gx1[8]*Gx2[10] + Gx1[12]*Gx2[14];
acadoWorkspace.H[3] += + Gx1[0]*Gx2[3] + Gx1[4]*Gx2[7] + Gx1[8]*Gx2[11] + Gx1[12]*Gx2[15];
acadoWorkspace.H[29] += + Gx1[1]*Gx2[0] + Gx1[5]*Gx2[4] + Gx1[9]*Gx2[8] + Gx1[13]*Gx2[12];
acadoWorkspace.H[30] += + Gx1[1]*Gx2[1] + Gx1[5]*Gx2[5] + Gx1[9]*Gx2[9] + Gx1[13]*Gx2[13];
acadoWorkspace.H[31] += + Gx1[1]*Gx2[2] + Gx1[5]*Gx2[6] + Gx1[9]*Gx2[10] + Gx1[13]*Gx2[14];
acadoWorkspace.H[32] += + Gx1[1]*Gx2[3] + Gx1[5]*Gx2[7] + Gx1[9]*Gx2[11] + Gx1[13]*Gx2[15];
acadoWorkspace.H[58] += + Gx1[2]*Gx2[0] + Gx1[6]*Gx2[4] + Gx1[10]*Gx2[8] + Gx1[14]*Gx2[12];
acadoWorkspace.H[59] += + Gx1[2]*Gx2[1] + Gx1[6]*Gx2[5] + Gx1[10]*Gx2[9] + Gx1[14]*Gx2[13];
acadoWorkspace.H[60] += + Gx1[2]*Gx2[2] + Gx1[6]*Gx2[6] + Gx1[10]*Gx2[10] + Gx1[14]*Gx2[14];
acadoWorkspace.H[61] += + Gx1[2]*Gx2[3] + Gx1[6]*Gx2[7] + Gx1[10]*Gx2[11] + Gx1[14]*Gx2[15];
acadoWorkspace.H[87] += + Gx1[3]*Gx2[0] + Gx1[7]*Gx2[4] + Gx1[11]*Gx2[8] + Gx1[15]*Gx2[12];
acadoWorkspace.H[88] += + Gx1[3]*Gx2[1] + Gx1[7]*Gx2[5] + Gx1[11]*Gx2[9] + Gx1[15]*Gx2[13];
acadoWorkspace.H[89] += + Gx1[3]*Gx2[2] + Gx1[7]*Gx2[6] + Gx1[11]*Gx2[10] + Gx1[15]*Gx2[14];
acadoWorkspace.H[90] += + Gx1[3]*Gx2[3] + Gx1[7]*Gx2[7] + Gx1[11]*Gx2[11] + Gx1[15]*Gx2[15];
}

void acado_multHxC( real_t* const Hx, real_t* const Gx, real_t* const A01 )
{
A01[0] = + Hx[0]*Gx[0] + Hx[1]*Gx[4] + Hx[2]*Gx[8] + Hx[3]*Gx[12];
A01[1] = + Hx[0]*Gx[1] + Hx[1]*Gx[5] + Hx[2]*Gx[9] + Hx[3]*Gx[13];
A01[2] = + Hx[0]*Gx[2] + Hx[1]*Gx[6] + Hx[2]*Gx[10] + Hx[3]*Gx[14];
A01[3] = + Hx[0]*Gx[3] + Hx[1]*Gx[7] + Hx[2]*Gx[11] + Hx[3]*Gx[15];
A01[29] = + Hx[4]*Gx[0] + Hx[5]*Gx[4] + Hx[6]*Gx[8] + Hx[7]*Gx[12];
A01[30] = + Hx[4]*Gx[1] + Hx[5]*Gx[5] + Hx[6]*Gx[9] + Hx[7]*Gx[13];
A01[31] = + Hx[4]*Gx[2] + Hx[5]*Gx[6] + Hx[6]*Gx[10] + Hx[7]*Gx[14];
A01[32] = + Hx[4]*Gx[3] + Hx[5]*Gx[7] + Hx[6]*Gx[11] + Hx[7]*Gx[15];
A01[58] = + Hx[8]*Gx[0] + Hx[9]*Gx[4] + Hx[10]*Gx[8] + Hx[11]*Gx[12];
A01[59] = + Hx[8]*Gx[1] + Hx[9]*Gx[5] + Hx[10]*Gx[9] + Hx[11]*Gx[13];
A01[60] = + Hx[8]*Gx[2] + Hx[9]*Gx[6] + Hx[10]*Gx[10] + Hx[11]*Gx[14];
A01[61] = + Hx[8]*Gx[3] + Hx[9]*Gx[7] + Hx[10]*Gx[11] + Hx[11]*Gx[15];
A01[87] = + Hx[12]*Gx[0] + Hx[13]*Gx[4] + Hx[14]*Gx[8] + Hx[15]*Gx[12];
A01[88] = + Hx[12]*Gx[1] + Hx[13]*Gx[5] + Hx[14]*Gx[9] + Hx[15]*Gx[13];
A01[89] = + Hx[12]*Gx[2] + Hx[13]*Gx[6] + Hx[14]*Gx[10] + Hx[15]*Gx[14];
A01[90] = + Hx[12]*Gx[3] + Hx[13]*Gx[7] + Hx[14]*Gx[11] + Hx[15]*Gx[15];
}

void acado_multHxE( real_t* const Hx, real_t* const E, int row, int col )
{
acadoWorkspace.A[(row * 116 + 725) + (col + 4)] = + Hx[0]*E[0] + Hx[1]*E[1] + Hx[2]*E[2] + Hx[3]*E[3];
acadoWorkspace.A[(row * 116 + 754) + (col + 4)] = + Hx[4]*E[0] + Hx[5]*E[1] + Hx[6]*E[2] + Hx[7]*E[3];
acadoWorkspace.A[(row * 116 + 783) + (col + 4)] = + Hx[8]*E[0] + Hx[9]*E[1] + Hx[10]*E[2] + Hx[11]*E[3];
acadoWorkspace.A[(row * 116 + 812) + (col + 4)] = + Hx[12]*E[0] + Hx[13]*E[1] + Hx[14]*E[2] + Hx[15]*E[3];
}

void acado_macHxd( real_t* const Hx, real_t* const tmpd, real_t* const lbA, real_t* const ubA )
{
acadoWorkspace.evHxd[0] = + Hx[0]*tmpd[0] + Hx[1]*tmpd[1] + Hx[2]*tmpd[2] + Hx[3]*tmpd[3];
acadoWorkspace.evHxd[1] = + Hx[4]*tmpd[0] + Hx[5]*tmpd[1] + Hx[6]*tmpd[2] + Hx[7]*tmpd[3];
acadoWorkspace.evHxd[2] = + Hx[8]*tmpd[0] + Hx[9]*tmpd[1] + Hx[10]*tmpd[2] + Hx[11]*tmpd[3];
acadoWorkspace.evHxd[3] = + Hx[12]*tmpd[0] + Hx[13]*tmpd[1] + Hx[14]*tmpd[2] + Hx[15]*tmpd[3];
lbA[0] -= acadoWorkspace.evHxd[0];
lbA[1] -= acadoWorkspace.evHxd[1];
lbA[2] -= acadoWorkspace.evHxd[2];
lbA[3] -= acadoWorkspace.evHxd[3];
ubA[0] -= acadoWorkspace.evHxd[0];
ubA[1] -= acadoWorkspace.evHxd[1];
ubA[2] -= acadoWorkspace.evHxd[2];
ubA[3] -= acadoWorkspace.evHxd[3];
}

void acado_evaluatePathConstraints(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 4;
const real_t* od = in + 5;
/* Vector of auxiliary variables; number of elements: 20. */
real_t* a = acadoWorkspace.conAuxVar;

/* Compute intermediate quantities: */
a[0] = (real_t)(0.0000000000000000e+00);
a[1] = (real_t)(0.0000000000000000e+00);
a[2] = (real_t)(0.0000000000000000e+00);
a[3] = (real_t)(-1.0000000000000000e+00);
a[4] = (real_t)(0.0000000000000000e+00);
a[5] = (real_t)(0.0000000000000000e+00);
a[6] = (real_t)(0.0000000000000000e+00);
a[7] = (real_t)(-1.0000000000000000e+00);
a[8] = (real_t)(0.0000000000000000e+00);
a[9] = (real_t)(0.0000000000000000e+00);
a[10] = (real_t)(0.0000000000000000e+00);
a[11] = (real_t)(0.0000000000000000e+00);
a[12] = (real_t)(0.0000000000000000e+00);
a[13] = (real_t)(0.0000000000000000e+00);
a[14] = (real_t)(0.0000000000000000e+00);
a[15] = (real_t)(0.0000000000000000e+00);
a[16] = (real_t)(0.0000000000000000e+00);
a[17] = (real_t)(0.0000000000000000e+00);
a[18] = (real_t)(-1.0000000000000000e+00);
a[19] = (real_t)(-1.0000000000000000e+00);

/* Compute outputs: */
out[0] = (((real_t)(0.0000000000000000e+00)-od[5])-xd[3]);
out[1] = (((real_t)(0.0000000000000000e+00)-xd[3])+od[5]);
out[2] = (((real_t)(0.0000000000000000e+00)-od[4])-u[0]);
out[3] = (((real_t)(0.0000000000000000e+00)-u[0])+od[4]);
out[4] = a[0];
out[5] = a[1];
out[6] = a[2];
out[7] = a[3];
out[8] = a[4];
out[9] = a[5];
out[10] = a[6];
out[11] = a[7];
out[12] = a[8];
out[13] = a[9];
out[14] = a[10];
out[15] = a[11];
out[16] = a[12];
out[17] = a[13];
out[18] = a[14];
out[19] = a[15];
out[20] = a[16];
out[21] = a[17];
out[22] = a[18];
out[23] = a[19];
}

void acado_macCTSlx( real_t* const C0, real_t* const g0 )
{
g0[0] += 0.0;
;
g0[1] += 0.0;
;
g0[2] += 0.0;
;
g0[3] += 0.0;
;
}

void acado_macETSlu( real_t* const E0, real_t* const g1 )
{
g1[0] += 0.0;
;
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
/** Row vector of size: 25 */
static const int xBoundIndices[ 25 ] = 
{ 6, 10, 14, 18, 22, 26, 30, 34, 38, 42, 46, 50, 54, 58, 62, 66, 70, 74, 78, 82, 86, 90, 94, 98, 102 };
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
for (lRun1 = 1; lRun1 < 25; ++lRun1)
{
acado_moveGxT( &(acadoWorkspace.evGx[ lRun1 * 16 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ lRun1 * 4-4 ]), &(acadoWorkspace.evGx[ lRun1 * 16 ]), &(acadoWorkspace.d[ lRun1 * 4 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ lRun1 * 16-16 ]), &(acadoWorkspace.evGx[ lRun1 * 16 ]) );
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
lRun4 = (((lRun1) * (lRun1-1)) / (2)) + (lRun2);
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ lRun4 * 4 ]), &(acadoWorkspace.E[ lRun3 * 4 ]) );
}
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun1 * 4 ]), &(acadoWorkspace.E[ lRun3 * 4 ]) );
}

acado_multGxGx( &(acadoWorkspace.Q1[ 16 ]), acadoWorkspace.evGx, acadoWorkspace.QGx );
acado_multGxGx( &(acadoWorkspace.Q1[ 32 ]), &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.QGx[ 16 ]) );
acado_multGxGx( &(acadoWorkspace.Q1[ 48 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.QGx[ 32 ]) );
acado_multGxGx( &(acadoWorkspace.Q1[ 64 ]), &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.QGx[ 48 ]) );
acado_multGxGx( &(acadoWorkspace.Q1[ 80 ]), &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.QGx[ 64 ]) );
acado_multGxGx( &(acadoWorkspace.Q1[ 96 ]), &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.QGx[ 80 ]) );
acado_multGxGx( &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.QGx[ 96 ]) );
acado_multGxGx( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.QGx[ 112 ]) );
acado_multGxGx( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.QGx[ 128 ]) );
acado_multGxGx( &(acadoWorkspace.Q1[ 160 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.QGx[ 144 ]) );
acado_multGxGx( &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.QGx[ 160 ]) );
acado_multGxGx( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.QGx[ 176 ]) );
acado_multGxGx( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.QGx[ 192 ]) );
acado_multGxGx( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.QGx[ 208 ]) );
acado_multGxGx( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.QGx[ 224 ]) );
acado_multGxGx( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.QGx[ 240 ]) );
acado_multGxGx( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.QGx[ 256 ]) );
acado_multGxGx( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.QGx[ 272 ]) );
acado_multGxGx( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.QGx[ 288 ]) );
acado_multGxGx( &(acadoWorkspace.Q1[ 320 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.QGx[ 304 ]) );
acado_multGxGx( &(acadoWorkspace.Q1[ 336 ]), &(acadoWorkspace.evGx[ 320 ]), &(acadoWorkspace.QGx[ 320 ]) );
acado_multGxGx( &(acadoWorkspace.Q1[ 352 ]), &(acadoWorkspace.evGx[ 336 ]), &(acadoWorkspace.QGx[ 336 ]) );
acado_multGxGx( &(acadoWorkspace.Q1[ 368 ]), &(acadoWorkspace.evGx[ 352 ]), &(acadoWorkspace.QGx[ 352 ]) );
acado_multGxGx( &(acadoWorkspace.Q1[ 384 ]), &(acadoWorkspace.evGx[ 368 ]), &(acadoWorkspace.QGx[ 368 ]) );
acado_multGxGx( acadoWorkspace.QN1, &(acadoWorkspace.evGx[ 384 ]), &(acadoWorkspace.QGx[ 384 ]) );

for (lRun1 = 0; lRun1 < 24; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( &(acadoWorkspace.Q1[ lRun1 * 16 + 16 ]), &(acadoWorkspace.E[ lRun3 * 4 ]), &(acadoWorkspace.QE[ lRun3 * 4 ]) );
}
}

for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ lRun3 * 4 ]), &(acadoWorkspace.QE[ lRun3 * 4 ]) );
}

acado_zeroBlockH00(  );
acado_multCTQC( acadoWorkspace.evGx, acadoWorkspace.QGx );
acado_multCTQC( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.QGx[ 16 ]) );
acado_multCTQC( &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.QGx[ 32 ]) );
acado_multCTQC( &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.QGx[ 48 ]) );
acado_multCTQC( &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.QGx[ 64 ]) );
acado_multCTQC( &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.QGx[ 80 ]) );
acado_multCTQC( &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.QGx[ 96 ]) );
acado_multCTQC( &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.QGx[ 112 ]) );
acado_multCTQC( &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.QGx[ 128 ]) );
acado_multCTQC( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.QGx[ 144 ]) );
acado_multCTQC( &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.QGx[ 160 ]) );
acado_multCTQC( &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.QGx[ 176 ]) );
acado_multCTQC( &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.QGx[ 192 ]) );
acado_multCTQC( &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.QGx[ 208 ]) );
acado_multCTQC( &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.QGx[ 224 ]) );
acado_multCTQC( &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.QGx[ 240 ]) );
acado_multCTQC( &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.QGx[ 256 ]) );
acado_multCTQC( &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.QGx[ 272 ]) );
acado_multCTQC( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.QGx[ 288 ]) );
acado_multCTQC( &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.QGx[ 304 ]) );
acado_multCTQC( &(acadoWorkspace.evGx[ 320 ]), &(acadoWorkspace.QGx[ 320 ]) );
acado_multCTQC( &(acadoWorkspace.evGx[ 336 ]), &(acadoWorkspace.QGx[ 336 ]) );
acado_multCTQC( &(acadoWorkspace.evGx[ 352 ]), &(acadoWorkspace.QGx[ 352 ]) );
acado_multCTQC( &(acadoWorkspace.evGx[ 368 ]), &(acadoWorkspace.QGx[ 368 ]) );
acado_multCTQC( &(acadoWorkspace.evGx[ 384 ]), &(acadoWorkspace.QGx[ 384 ]) );

for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
acado_zeroBlockH10( &(acadoWorkspace.H10[ lRun1 * 4 ]) );
for (lRun2 = lRun1; lRun2 < 25; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_multQETGx( &(acadoWorkspace.QE[ lRun3 * 4 ]), &(acadoWorkspace.evGx[ lRun2 * 16 ]), &(acadoWorkspace.H10[ lRun1 * 4 ]) );
}
}

acadoWorkspace.H[4] = acadoWorkspace.H10[0];
acadoWorkspace.H[5] = acadoWorkspace.H10[4];
acadoWorkspace.H[6] = acadoWorkspace.H10[8];
acadoWorkspace.H[7] = acadoWorkspace.H10[12];
acadoWorkspace.H[8] = acadoWorkspace.H10[16];
acadoWorkspace.H[9] = acadoWorkspace.H10[20];
acadoWorkspace.H[10] = acadoWorkspace.H10[24];
acadoWorkspace.H[11] = acadoWorkspace.H10[28];
acadoWorkspace.H[12] = acadoWorkspace.H10[32];
acadoWorkspace.H[13] = acadoWorkspace.H10[36];
acadoWorkspace.H[14] = acadoWorkspace.H10[40];
acadoWorkspace.H[15] = acadoWorkspace.H10[44];
acadoWorkspace.H[16] = acadoWorkspace.H10[48];
acadoWorkspace.H[17] = acadoWorkspace.H10[52];
acadoWorkspace.H[18] = acadoWorkspace.H10[56];
acadoWorkspace.H[19] = acadoWorkspace.H10[60];
acadoWorkspace.H[20] = acadoWorkspace.H10[64];
acadoWorkspace.H[21] = acadoWorkspace.H10[68];
acadoWorkspace.H[22] = acadoWorkspace.H10[72];
acadoWorkspace.H[23] = acadoWorkspace.H10[76];
acadoWorkspace.H[24] = acadoWorkspace.H10[80];
acadoWorkspace.H[25] = acadoWorkspace.H10[84];
acadoWorkspace.H[26] = acadoWorkspace.H10[88];
acadoWorkspace.H[27] = acadoWorkspace.H10[92];
acadoWorkspace.H[28] = acadoWorkspace.H10[96];
acadoWorkspace.H[33] = acadoWorkspace.H10[1];
acadoWorkspace.H[34] = acadoWorkspace.H10[5];
acadoWorkspace.H[35] = acadoWorkspace.H10[9];
acadoWorkspace.H[36] = acadoWorkspace.H10[13];
acadoWorkspace.H[37] = acadoWorkspace.H10[17];
acadoWorkspace.H[38] = acadoWorkspace.H10[21];
acadoWorkspace.H[39] = acadoWorkspace.H10[25];
acadoWorkspace.H[40] = acadoWorkspace.H10[29];
acadoWorkspace.H[41] = acadoWorkspace.H10[33];
acadoWorkspace.H[42] = acadoWorkspace.H10[37];
acadoWorkspace.H[43] = acadoWorkspace.H10[41];
acadoWorkspace.H[44] = acadoWorkspace.H10[45];
acadoWorkspace.H[45] = acadoWorkspace.H10[49];
acadoWorkspace.H[46] = acadoWorkspace.H10[53];
acadoWorkspace.H[47] = acadoWorkspace.H10[57];
acadoWorkspace.H[48] = acadoWorkspace.H10[61];
acadoWorkspace.H[49] = acadoWorkspace.H10[65];
acadoWorkspace.H[50] = acadoWorkspace.H10[69];
acadoWorkspace.H[51] = acadoWorkspace.H10[73];
acadoWorkspace.H[52] = acadoWorkspace.H10[77];
acadoWorkspace.H[53] = acadoWorkspace.H10[81];
acadoWorkspace.H[54] = acadoWorkspace.H10[85];
acadoWorkspace.H[55] = acadoWorkspace.H10[89];
acadoWorkspace.H[56] = acadoWorkspace.H10[93];
acadoWorkspace.H[57] = acadoWorkspace.H10[97];
acadoWorkspace.H[62] = acadoWorkspace.H10[2];
acadoWorkspace.H[63] = acadoWorkspace.H10[6];
acadoWorkspace.H[64] = acadoWorkspace.H10[10];
acadoWorkspace.H[65] = acadoWorkspace.H10[14];
acadoWorkspace.H[66] = acadoWorkspace.H10[18];
acadoWorkspace.H[67] = acadoWorkspace.H10[22];
acadoWorkspace.H[68] = acadoWorkspace.H10[26];
acadoWorkspace.H[69] = acadoWorkspace.H10[30];
acadoWorkspace.H[70] = acadoWorkspace.H10[34];
acadoWorkspace.H[71] = acadoWorkspace.H10[38];
acadoWorkspace.H[72] = acadoWorkspace.H10[42];
acadoWorkspace.H[73] = acadoWorkspace.H10[46];
acadoWorkspace.H[74] = acadoWorkspace.H10[50];
acadoWorkspace.H[75] = acadoWorkspace.H10[54];
acadoWorkspace.H[76] = acadoWorkspace.H10[58];
acadoWorkspace.H[77] = acadoWorkspace.H10[62];
acadoWorkspace.H[78] = acadoWorkspace.H10[66];
acadoWorkspace.H[79] = acadoWorkspace.H10[70];
acadoWorkspace.H[80] = acadoWorkspace.H10[74];
acadoWorkspace.H[81] = acadoWorkspace.H10[78];
acadoWorkspace.H[82] = acadoWorkspace.H10[82];
acadoWorkspace.H[83] = acadoWorkspace.H10[86];
acadoWorkspace.H[84] = acadoWorkspace.H10[90];
acadoWorkspace.H[85] = acadoWorkspace.H10[94];
acadoWorkspace.H[86] = acadoWorkspace.H10[98];
acadoWorkspace.H[91] = acadoWorkspace.H10[3];
acadoWorkspace.H[92] = acadoWorkspace.H10[7];
acadoWorkspace.H[93] = acadoWorkspace.H10[11];
acadoWorkspace.H[94] = acadoWorkspace.H10[15];
acadoWorkspace.H[95] = acadoWorkspace.H10[19];
acadoWorkspace.H[96] = acadoWorkspace.H10[23];
acadoWorkspace.H[97] = acadoWorkspace.H10[27];
acadoWorkspace.H[98] = acadoWorkspace.H10[31];
acadoWorkspace.H[99] = acadoWorkspace.H10[35];
acadoWorkspace.H[100] = acadoWorkspace.H10[39];
acadoWorkspace.H[101] = acadoWorkspace.H10[43];
acadoWorkspace.H[102] = acadoWorkspace.H10[47];
acadoWorkspace.H[103] = acadoWorkspace.H10[51];
acadoWorkspace.H[104] = acadoWorkspace.H10[55];
acadoWorkspace.H[105] = acadoWorkspace.H10[59];
acadoWorkspace.H[106] = acadoWorkspace.H10[63];
acadoWorkspace.H[107] = acadoWorkspace.H10[67];
acadoWorkspace.H[108] = acadoWorkspace.H10[71];
acadoWorkspace.H[109] = acadoWorkspace.H10[75];
acadoWorkspace.H[110] = acadoWorkspace.H10[79];
acadoWorkspace.H[111] = acadoWorkspace.H10[83];
acadoWorkspace.H[112] = acadoWorkspace.H10[87];
acadoWorkspace.H[113] = acadoWorkspace.H10[91];
acadoWorkspace.H[114] = acadoWorkspace.H10[95];
acadoWorkspace.H[115] = acadoWorkspace.H10[99];

for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
acado_setBlockH11_R1( lRun1, lRun1, &(acadoWorkspace.R1[ lRun1 ]) );
lRun2 = lRun1;
for (lRun3 = lRun1; lRun3 < 25; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
acado_setBlockH11( lRun1, lRun2, &(acadoWorkspace.E[ lRun4 * 4 ]), &(acadoWorkspace.QE[ lRun5 * 4 ]) );
}
for (lRun2 = lRun1 + 1; lRun2 < 25; ++lRun2)
{
acado_zeroBlockH11( lRun1, lRun2 );
for (lRun3 = lRun2; lRun3 < 25; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
acado_setBlockH11( lRun1, lRun2, &(acadoWorkspace.E[ lRun4 * 4 ]), &(acadoWorkspace.QE[ lRun5 * 4 ]) );
}
}
}

for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
acado_copyHTH( lRun1, lRun2 );
}
}

acadoWorkspace.H[116] = acadoWorkspace.H10[0];
acadoWorkspace.H[117] = acadoWorkspace.H10[1];
acadoWorkspace.H[118] = acadoWorkspace.H10[2];
acadoWorkspace.H[119] = acadoWorkspace.H10[3];
acadoWorkspace.H[145] = acadoWorkspace.H10[4];
acadoWorkspace.H[146] = acadoWorkspace.H10[5];
acadoWorkspace.H[147] = acadoWorkspace.H10[6];
acadoWorkspace.H[148] = acadoWorkspace.H10[7];
acadoWorkspace.H[174] = acadoWorkspace.H10[8];
acadoWorkspace.H[175] = acadoWorkspace.H10[9];
acadoWorkspace.H[176] = acadoWorkspace.H10[10];
acadoWorkspace.H[177] = acadoWorkspace.H10[11];
acadoWorkspace.H[203] = acadoWorkspace.H10[12];
acadoWorkspace.H[204] = acadoWorkspace.H10[13];
acadoWorkspace.H[205] = acadoWorkspace.H10[14];
acadoWorkspace.H[206] = acadoWorkspace.H10[15];
acadoWorkspace.H[232] = acadoWorkspace.H10[16];
acadoWorkspace.H[233] = acadoWorkspace.H10[17];
acadoWorkspace.H[234] = acadoWorkspace.H10[18];
acadoWorkspace.H[235] = acadoWorkspace.H10[19];
acadoWorkspace.H[261] = acadoWorkspace.H10[20];
acadoWorkspace.H[262] = acadoWorkspace.H10[21];
acadoWorkspace.H[263] = acadoWorkspace.H10[22];
acadoWorkspace.H[264] = acadoWorkspace.H10[23];
acadoWorkspace.H[290] = acadoWorkspace.H10[24];
acadoWorkspace.H[291] = acadoWorkspace.H10[25];
acadoWorkspace.H[292] = acadoWorkspace.H10[26];
acadoWorkspace.H[293] = acadoWorkspace.H10[27];
acadoWorkspace.H[319] = acadoWorkspace.H10[28];
acadoWorkspace.H[320] = acadoWorkspace.H10[29];
acadoWorkspace.H[321] = acadoWorkspace.H10[30];
acadoWorkspace.H[322] = acadoWorkspace.H10[31];
acadoWorkspace.H[348] = acadoWorkspace.H10[32];
acadoWorkspace.H[349] = acadoWorkspace.H10[33];
acadoWorkspace.H[350] = acadoWorkspace.H10[34];
acadoWorkspace.H[351] = acadoWorkspace.H10[35];
acadoWorkspace.H[377] = acadoWorkspace.H10[36];
acadoWorkspace.H[378] = acadoWorkspace.H10[37];
acadoWorkspace.H[379] = acadoWorkspace.H10[38];
acadoWorkspace.H[380] = acadoWorkspace.H10[39];
acadoWorkspace.H[406] = acadoWorkspace.H10[40];
acadoWorkspace.H[407] = acadoWorkspace.H10[41];
acadoWorkspace.H[408] = acadoWorkspace.H10[42];
acadoWorkspace.H[409] = acadoWorkspace.H10[43];
acadoWorkspace.H[435] = acadoWorkspace.H10[44];
acadoWorkspace.H[436] = acadoWorkspace.H10[45];
acadoWorkspace.H[437] = acadoWorkspace.H10[46];
acadoWorkspace.H[438] = acadoWorkspace.H10[47];
acadoWorkspace.H[464] = acadoWorkspace.H10[48];
acadoWorkspace.H[465] = acadoWorkspace.H10[49];
acadoWorkspace.H[466] = acadoWorkspace.H10[50];
acadoWorkspace.H[467] = acadoWorkspace.H10[51];
acadoWorkspace.H[493] = acadoWorkspace.H10[52];
acadoWorkspace.H[494] = acadoWorkspace.H10[53];
acadoWorkspace.H[495] = acadoWorkspace.H10[54];
acadoWorkspace.H[496] = acadoWorkspace.H10[55];
acadoWorkspace.H[522] = acadoWorkspace.H10[56];
acadoWorkspace.H[523] = acadoWorkspace.H10[57];
acadoWorkspace.H[524] = acadoWorkspace.H10[58];
acadoWorkspace.H[525] = acadoWorkspace.H10[59];
acadoWorkspace.H[551] = acadoWorkspace.H10[60];
acadoWorkspace.H[552] = acadoWorkspace.H10[61];
acadoWorkspace.H[553] = acadoWorkspace.H10[62];
acadoWorkspace.H[554] = acadoWorkspace.H10[63];
acadoWorkspace.H[580] = acadoWorkspace.H10[64];
acadoWorkspace.H[581] = acadoWorkspace.H10[65];
acadoWorkspace.H[582] = acadoWorkspace.H10[66];
acadoWorkspace.H[583] = acadoWorkspace.H10[67];
acadoWorkspace.H[609] = acadoWorkspace.H10[68];
acadoWorkspace.H[610] = acadoWorkspace.H10[69];
acadoWorkspace.H[611] = acadoWorkspace.H10[70];
acadoWorkspace.H[612] = acadoWorkspace.H10[71];
acadoWorkspace.H[638] = acadoWorkspace.H10[72];
acadoWorkspace.H[639] = acadoWorkspace.H10[73];
acadoWorkspace.H[640] = acadoWorkspace.H10[74];
acadoWorkspace.H[641] = acadoWorkspace.H10[75];
acadoWorkspace.H[667] = acadoWorkspace.H10[76];
acadoWorkspace.H[668] = acadoWorkspace.H10[77];
acadoWorkspace.H[669] = acadoWorkspace.H10[78];
acadoWorkspace.H[670] = acadoWorkspace.H10[79];
acadoWorkspace.H[696] = acadoWorkspace.H10[80];
acadoWorkspace.H[697] = acadoWorkspace.H10[81];
acadoWorkspace.H[698] = acadoWorkspace.H10[82];
acadoWorkspace.H[699] = acadoWorkspace.H10[83];
acadoWorkspace.H[725] = acadoWorkspace.H10[84];
acadoWorkspace.H[726] = acadoWorkspace.H10[85];
acadoWorkspace.H[727] = acadoWorkspace.H10[86];
acadoWorkspace.H[728] = acadoWorkspace.H10[87];
acadoWorkspace.H[754] = acadoWorkspace.H10[88];
acadoWorkspace.H[755] = acadoWorkspace.H10[89];
acadoWorkspace.H[756] = acadoWorkspace.H10[90];
acadoWorkspace.H[757] = acadoWorkspace.H10[91];
acadoWorkspace.H[783] = acadoWorkspace.H10[92];
acadoWorkspace.H[784] = acadoWorkspace.H10[93];
acadoWorkspace.H[785] = acadoWorkspace.H10[94];
acadoWorkspace.H[786] = acadoWorkspace.H10[95];
acadoWorkspace.H[812] = acadoWorkspace.H10[96];
acadoWorkspace.H[813] = acadoWorkspace.H10[97];
acadoWorkspace.H[814] = acadoWorkspace.H10[98];
acadoWorkspace.H[815] = acadoWorkspace.H10[99];

acado_multQ1d( &(acadoWorkspace.Q1[ 16 ]), acadoWorkspace.d, acadoWorkspace.Qd );
acado_multQ1d( &(acadoWorkspace.Q1[ 32 ]), &(acadoWorkspace.d[ 4 ]), &(acadoWorkspace.Qd[ 4 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 48 ]), &(acadoWorkspace.d[ 8 ]), &(acadoWorkspace.Qd[ 8 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 64 ]), &(acadoWorkspace.d[ 12 ]), &(acadoWorkspace.Qd[ 12 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 80 ]), &(acadoWorkspace.d[ 16 ]), &(acadoWorkspace.Qd[ 16 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 96 ]), &(acadoWorkspace.d[ 20 ]), &(acadoWorkspace.Qd[ 20 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.d[ 24 ]), &(acadoWorkspace.Qd[ 24 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.d[ 28 ]), &(acadoWorkspace.Qd[ 28 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.d[ 32 ]), &(acadoWorkspace.Qd[ 32 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 160 ]), &(acadoWorkspace.d[ 36 ]), &(acadoWorkspace.Qd[ 36 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.d[ 40 ]), &(acadoWorkspace.Qd[ 40 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.d[ 44 ]), &(acadoWorkspace.Qd[ 44 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.d[ 48 ]), &(acadoWorkspace.Qd[ 48 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.d[ 52 ]), &(acadoWorkspace.Qd[ 52 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.d[ 56 ]), &(acadoWorkspace.Qd[ 56 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.Qd[ 60 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.d[ 64 ]), &(acadoWorkspace.Qd[ 64 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.d[ 68 ]), &(acadoWorkspace.Qd[ 68 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.d[ 72 ]), &(acadoWorkspace.Qd[ 72 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 320 ]), &(acadoWorkspace.d[ 76 ]), &(acadoWorkspace.Qd[ 76 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 336 ]), &(acadoWorkspace.d[ 80 ]), &(acadoWorkspace.Qd[ 80 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 352 ]), &(acadoWorkspace.d[ 84 ]), &(acadoWorkspace.Qd[ 84 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 368 ]), &(acadoWorkspace.d[ 88 ]), &(acadoWorkspace.Qd[ 88 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 384 ]), &(acadoWorkspace.d[ 92 ]), &(acadoWorkspace.Qd[ 92 ]) );
acado_multQN1d( acadoWorkspace.QN1, &(acadoWorkspace.d[ 96 ]), &(acadoWorkspace.Qd[ 96 ]) );

acado_macCTSlx( acadoWorkspace.evGx, acadoWorkspace.g );
acado_macCTSlx( &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.g );
acado_macCTSlx( &(acadoWorkspace.evGx[ 32 ]), acadoWorkspace.g );
acado_macCTSlx( &(acadoWorkspace.evGx[ 48 ]), acadoWorkspace.g );
acado_macCTSlx( &(acadoWorkspace.evGx[ 64 ]), acadoWorkspace.g );
acado_macCTSlx( &(acadoWorkspace.evGx[ 80 ]), acadoWorkspace.g );
acado_macCTSlx( &(acadoWorkspace.evGx[ 96 ]), acadoWorkspace.g );
acado_macCTSlx( &(acadoWorkspace.evGx[ 112 ]), acadoWorkspace.g );
acado_macCTSlx( &(acadoWorkspace.evGx[ 128 ]), acadoWorkspace.g );
acado_macCTSlx( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.g );
acado_macCTSlx( &(acadoWorkspace.evGx[ 160 ]), acadoWorkspace.g );
acado_macCTSlx( &(acadoWorkspace.evGx[ 176 ]), acadoWorkspace.g );
acado_macCTSlx( &(acadoWorkspace.evGx[ 192 ]), acadoWorkspace.g );
acado_macCTSlx( &(acadoWorkspace.evGx[ 208 ]), acadoWorkspace.g );
acado_macCTSlx( &(acadoWorkspace.evGx[ 224 ]), acadoWorkspace.g );
acado_macCTSlx( &(acadoWorkspace.evGx[ 240 ]), acadoWorkspace.g );
acado_macCTSlx( &(acadoWorkspace.evGx[ 256 ]), acadoWorkspace.g );
acado_macCTSlx( &(acadoWorkspace.evGx[ 272 ]), acadoWorkspace.g );
acado_macCTSlx( &(acadoWorkspace.evGx[ 288 ]), acadoWorkspace.g );
acado_macCTSlx( &(acadoWorkspace.evGx[ 304 ]), acadoWorkspace.g );
acado_macCTSlx( &(acadoWorkspace.evGx[ 320 ]), acadoWorkspace.g );
acado_macCTSlx( &(acadoWorkspace.evGx[ 336 ]), acadoWorkspace.g );
acado_macCTSlx( &(acadoWorkspace.evGx[ 352 ]), acadoWorkspace.g );
acado_macCTSlx( &(acadoWorkspace.evGx[ 368 ]), acadoWorkspace.g );
acado_macCTSlx( &(acadoWorkspace.evGx[ 384 ]), acadoWorkspace.g );
for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 25; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_macETSlu( &(acadoWorkspace.QE[ lRun3 * 4 ]), &(acadoWorkspace.g[ lRun1 + 4 ]) );
}
}
acadoWorkspace.lb[4] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[0];
acadoWorkspace.lb[5] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[1];
acadoWorkspace.lb[6] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[2];
acadoWorkspace.lb[7] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[3];
acadoWorkspace.lb[8] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[4];
acadoWorkspace.lb[9] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[5];
acadoWorkspace.lb[10] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[6];
acadoWorkspace.lb[11] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[7];
acadoWorkspace.lb[12] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[8];
acadoWorkspace.lb[13] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[9];
acadoWorkspace.lb[14] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[10];
acadoWorkspace.lb[15] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[11];
acadoWorkspace.lb[16] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[12];
acadoWorkspace.lb[17] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[13];
acadoWorkspace.lb[18] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[14];
acadoWorkspace.lb[19] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[15];
acadoWorkspace.lb[20] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[16];
acadoWorkspace.lb[21] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[17];
acadoWorkspace.lb[22] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[18];
acadoWorkspace.lb[23] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[19];
acadoWorkspace.lb[24] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[20];
acadoWorkspace.lb[25] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[21];
acadoWorkspace.lb[26] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[22];
acadoWorkspace.lb[27] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[23];
acadoWorkspace.lb[28] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[24];
acadoWorkspace.ub[4] = (real_t)1.0000000000000000e+12 - acadoVariables.u[0];
acadoWorkspace.ub[5] = (real_t)1.0000000000000000e+12 - acadoVariables.u[1];
acadoWorkspace.ub[6] = (real_t)1.0000000000000000e+12 - acadoVariables.u[2];
acadoWorkspace.ub[7] = (real_t)1.0000000000000000e+12 - acadoVariables.u[3];
acadoWorkspace.ub[8] = (real_t)1.0000000000000000e+12 - acadoVariables.u[4];
acadoWorkspace.ub[9] = (real_t)1.0000000000000000e+12 - acadoVariables.u[5];
acadoWorkspace.ub[10] = (real_t)1.0000000000000000e+12 - acadoVariables.u[6];
acadoWorkspace.ub[11] = (real_t)1.0000000000000000e+12 - acadoVariables.u[7];
acadoWorkspace.ub[12] = (real_t)1.0000000000000000e+12 - acadoVariables.u[8];
acadoWorkspace.ub[13] = (real_t)1.0000000000000000e+12 - acadoVariables.u[9];
acadoWorkspace.ub[14] = (real_t)1.0000000000000000e+12 - acadoVariables.u[10];
acadoWorkspace.ub[15] = (real_t)1.0000000000000000e+12 - acadoVariables.u[11];
acadoWorkspace.ub[16] = (real_t)1.0000000000000000e+12 - acadoVariables.u[12];
acadoWorkspace.ub[17] = (real_t)1.0000000000000000e+12 - acadoVariables.u[13];
acadoWorkspace.ub[18] = (real_t)1.0000000000000000e+12 - acadoVariables.u[14];
acadoWorkspace.ub[19] = (real_t)1.0000000000000000e+12 - acadoVariables.u[15];
acadoWorkspace.ub[20] = (real_t)1.0000000000000000e+12 - acadoVariables.u[16];
acadoWorkspace.ub[21] = (real_t)1.0000000000000000e+12 - acadoVariables.u[17];
acadoWorkspace.ub[22] = (real_t)1.0000000000000000e+12 - acadoVariables.u[18];
acadoWorkspace.ub[23] = (real_t)1.0000000000000000e+12 - acadoVariables.u[19];
acadoWorkspace.ub[24] = (real_t)1.0000000000000000e+12 - acadoVariables.u[20];
acadoWorkspace.ub[25] = (real_t)1.0000000000000000e+12 - acadoVariables.u[21];
acadoWorkspace.ub[26] = (real_t)1.0000000000000000e+12 - acadoVariables.u[22];
acadoWorkspace.ub[27] = (real_t)1.0000000000000000e+12 - acadoVariables.u[23];
acadoWorkspace.ub[28] = (real_t)1.0000000000000000e+12 - acadoVariables.u[24];

for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
lRun3 = xBoundIndices[ lRun1 ] - 4;
lRun4 = ((lRun3) / (4)) + (1);
acadoWorkspace.A[lRun1 * 29] = acadoWorkspace.evGx[lRun3 * 4];
acadoWorkspace.A[lRun1 * 29 + 1] = acadoWorkspace.evGx[lRun3 * 4 + 1];
acadoWorkspace.A[lRun1 * 29 + 2] = acadoWorkspace.evGx[lRun3 * 4 + 2];
acadoWorkspace.A[lRun1 * 29 + 3] = acadoWorkspace.evGx[lRun3 * 4 + 3];
for (lRun2 = 0; lRun2 < lRun4; ++lRun2)
{
lRun5 = (((((lRun4) * (lRun4-1)) / (2)) + (lRun2)) * (4)) + ((lRun3) % (4));
acadoWorkspace.A[(lRun1 * 29) + (lRun2 + 4)] = acadoWorkspace.E[lRun5];
}
}

for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
acadoWorkspace.conValueIn[0] = acadoVariables.x[lRun1 * 4];
acadoWorkspace.conValueIn[1] = acadoVariables.x[lRun1 * 4 + 1];
acadoWorkspace.conValueIn[2] = acadoVariables.x[lRun1 * 4 + 2];
acadoWorkspace.conValueIn[3] = acadoVariables.x[lRun1 * 4 + 3];
acadoWorkspace.conValueIn[4] = acadoVariables.u[lRun1];
acadoWorkspace.conValueIn[5] = acadoVariables.od[lRun1 * 6];
acadoWorkspace.conValueIn[6] = acadoVariables.od[lRun1 * 6 + 1];
acadoWorkspace.conValueIn[7] = acadoVariables.od[lRun1 * 6 + 2];
acadoWorkspace.conValueIn[8] = acadoVariables.od[lRun1 * 6 + 3];
acadoWorkspace.conValueIn[9] = acadoVariables.od[lRun1 * 6 + 4];
acadoWorkspace.conValueIn[10] = acadoVariables.od[lRun1 * 6 + 5];
acado_evaluatePathConstraints( acadoWorkspace.conValueIn, acadoWorkspace.conValueOut );
acadoWorkspace.evH[lRun1 * 4] = acadoWorkspace.conValueOut[0];
acadoWorkspace.evH[lRun1 * 4 + 1] = acadoWorkspace.conValueOut[1];
acadoWorkspace.evH[lRun1 * 4 + 2] = acadoWorkspace.conValueOut[2];
acadoWorkspace.evH[lRun1 * 4 + 3] = acadoWorkspace.conValueOut[3];

acadoWorkspace.evHx[lRun1 * 16] = acadoWorkspace.conValueOut[4];
acadoWorkspace.evHx[lRun1 * 16 + 1] = acadoWorkspace.conValueOut[5];
acadoWorkspace.evHx[lRun1 * 16 + 2] = acadoWorkspace.conValueOut[6];
acadoWorkspace.evHx[lRun1 * 16 + 3] = acadoWorkspace.conValueOut[7];
acadoWorkspace.evHx[lRun1 * 16 + 4] = acadoWorkspace.conValueOut[8];
acadoWorkspace.evHx[lRun1 * 16 + 5] = acadoWorkspace.conValueOut[9];
acadoWorkspace.evHx[lRun1 * 16 + 6] = acadoWorkspace.conValueOut[10];
acadoWorkspace.evHx[lRun1 * 16 + 7] = acadoWorkspace.conValueOut[11];
acadoWorkspace.evHx[lRun1 * 16 + 8] = acadoWorkspace.conValueOut[12];
acadoWorkspace.evHx[lRun1 * 16 + 9] = acadoWorkspace.conValueOut[13];
acadoWorkspace.evHx[lRun1 * 16 + 10] = acadoWorkspace.conValueOut[14];
acadoWorkspace.evHx[lRun1 * 16 + 11] = acadoWorkspace.conValueOut[15];
acadoWorkspace.evHx[lRun1 * 16 + 12] = acadoWorkspace.conValueOut[16];
acadoWorkspace.evHx[lRun1 * 16 + 13] = acadoWorkspace.conValueOut[17];
acadoWorkspace.evHx[lRun1 * 16 + 14] = acadoWorkspace.conValueOut[18];
acadoWorkspace.evHx[lRun1 * 16 + 15] = acadoWorkspace.conValueOut[19];
acadoWorkspace.evHu[lRun1 * 4] = acadoWorkspace.conValueOut[20];
acadoWorkspace.evHu[lRun1 * 4 + 1] = acadoWorkspace.conValueOut[21];
acadoWorkspace.evHu[lRun1 * 4 + 2] = acadoWorkspace.conValueOut[22];
acadoWorkspace.evHu[lRun1 * 4 + 3] = acadoWorkspace.conValueOut[23];
}

acadoWorkspace.A[725] = acadoWorkspace.evHx[0];
acadoWorkspace.A[726] = acadoWorkspace.evHx[1];
acadoWorkspace.A[727] = acadoWorkspace.evHx[2];
acadoWorkspace.A[728] = acadoWorkspace.evHx[3];
acadoWorkspace.A[754] = acadoWorkspace.evHx[4];
acadoWorkspace.A[755] = acadoWorkspace.evHx[5];
acadoWorkspace.A[756] = acadoWorkspace.evHx[6];
acadoWorkspace.A[757] = acadoWorkspace.evHx[7];
acadoWorkspace.A[783] = acadoWorkspace.evHx[8];
acadoWorkspace.A[784] = acadoWorkspace.evHx[9];
acadoWorkspace.A[785] = acadoWorkspace.evHx[10];
acadoWorkspace.A[786] = acadoWorkspace.evHx[11];
acadoWorkspace.A[812] = acadoWorkspace.evHx[12];
acadoWorkspace.A[813] = acadoWorkspace.evHx[13];
acadoWorkspace.A[814] = acadoWorkspace.evHx[14];
acadoWorkspace.A[815] = acadoWorkspace.evHx[15];

acado_multHxC( &(acadoWorkspace.evHx[ 16 ]), acadoWorkspace.evGx, &(acadoWorkspace.A[ 841 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 32 ]), &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.A[ 957 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.A[ 1073 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 64 ]), &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.A[ 1189 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 80 ]), &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.A[ 1305 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.A[ 1421 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 112 ]), &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.A[ 1537 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 128 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.A[ 1653 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.A[ 1769 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.A[ 1885 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 176 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.A[ 2001 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.A[ 2117 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 208 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.A[ 2233 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 224 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.A[ 2349 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 240 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.A[ 2465 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 256 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.A[ 2581 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 272 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.A[ 2697 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 288 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.A[ 2813 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 304 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.A[ 2929 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 320 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.A[ 3045 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 336 ]), &(acadoWorkspace.evGx[ 320 ]), &(acadoWorkspace.A[ 3161 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 352 ]), &(acadoWorkspace.evGx[ 336 ]), &(acadoWorkspace.A[ 3277 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 368 ]), &(acadoWorkspace.evGx[ 352 ]), &(acadoWorkspace.A[ 3393 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 384 ]), &(acadoWorkspace.evGx[ 368 ]), &(acadoWorkspace.A[ 3509 ]) );

for (lRun2 = 0; lRun2 < 24; ++lRun2)
{
for (lRun3 = 0; lRun3 < lRun2 + 1; ++lRun3)
{
lRun4 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun3);
lRun5 = lRun2 + 1;
acado_multHxE( &(acadoWorkspace.evHx[ lRun2 * 16 + 16 ]), &(acadoWorkspace.E[ lRun4 * 4 ]), lRun5, lRun3 );
}
}

acadoWorkspace.A[729] = acadoWorkspace.evHu[0];
acadoWorkspace.A[758] = acadoWorkspace.evHu[1];
acadoWorkspace.A[787] = acadoWorkspace.evHu[2];
acadoWorkspace.A[816] = acadoWorkspace.evHu[3];
acadoWorkspace.A[846] = acadoWorkspace.evHu[4];
acadoWorkspace.A[875] = acadoWorkspace.evHu[5];
acadoWorkspace.A[904] = acadoWorkspace.evHu[6];
acadoWorkspace.A[933] = acadoWorkspace.evHu[7];
acadoWorkspace.A[963] = acadoWorkspace.evHu[8];
acadoWorkspace.A[992] = acadoWorkspace.evHu[9];
acadoWorkspace.A[1021] = acadoWorkspace.evHu[10];
acadoWorkspace.A[1050] = acadoWorkspace.evHu[11];
acadoWorkspace.A[1080] = acadoWorkspace.evHu[12];
acadoWorkspace.A[1109] = acadoWorkspace.evHu[13];
acadoWorkspace.A[1138] = acadoWorkspace.evHu[14];
acadoWorkspace.A[1167] = acadoWorkspace.evHu[15];
acadoWorkspace.A[1197] = acadoWorkspace.evHu[16];
acadoWorkspace.A[1226] = acadoWorkspace.evHu[17];
acadoWorkspace.A[1255] = acadoWorkspace.evHu[18];
acadoWorkspace.A[1284] = acadoWorkspace.evHu[19];
acadoWorkspace.A[1314] = acadoWorkspace.evHu[20];
acadoWorkspace.A[1343] = acadoWorkspace.evHu[21];
acadoWorkspace.A[1372] = acadoWorkspace.evHu[22];
acadoWorkspace.A[1401] = acadoWorkspace.evHu[23];
acadoWorkspace.A[1431] = acadoWorkspace.evHu[24];
acadoWorkspace.A[1460] = acadoWorkspace.evHu[25];
acadoWorkspace.A[1489] = acadoWorkspace.evHu[26];
acadoWorkspace.A[1518] = acadoWorkspace.evHu[27];
acadoWorkspace.A[1548] = acadoWorkspace.evHu[28];
acadoWorkspace.A[1577] = acadoWorkspace.evHu[29];
acadoWorkspace.A[1606] = acadoWorkspace.evHu[30];
acadoWorkspace.A[1635] = acadoWorkspace.evHu[31];
acadoWorkspace.A[1665] = acadoWorkspace.evHu[32];
acadoWorkspace.A[1694] = acadoWorkspace.evHu[33];
acadoWorkspace.A[1723] = acadoWorkspace.evHu[34];
acadoWorkspace.A[1752] = acadoWorkspace.evHu[35];
acadoWorkspace.A[1782] = acadoWorkspace.evHu[36];
acadoWorkspace.A[1811] = acadoWorkspace.evHu[37];
acadoWorkspace.A[1840] = acadoWorkspace.evHu[38];
acadoWorkspace.A[1869] = acadoWorkspace.evHu[39];
acadoWorkspace.A[1899] = acadoWorkspace.evHu[40];
acadoWorkspace.A[1928] = acadoWorkspace.evHu[41];
acadoWorkspace.A[1957] = acadoWorkspace.evHu[42];
acadoWorkspace.A[1986] = acadoWorkspace.evHu[43];
acadoWorkspace.A[2016] = acadoWorkspace.evHu[44];
acadoWorkspace.A[2045] = acadoWorkspace.evHu[45];
acadoWorkspace.A[2074] = acadoWorkspace.evHu[46];
acadoWorkspace.A[2103] = acadoWorkspace.evHu[47];
acadoWorkspace.A[2133] = acadoWorkspace.evHu[48];
acadoWorkspace.A[2162] = acadoWorkspace.evHu[49];
acadoWorkspace.A[2191] = acadoWorkspace.evHu[50];
acadoWorkspace.A[2220] = acadoWorkspace.evHu[51];
acadoWorkspace.A[2250] = acadoWorkspace.evHu[52];
acadoWorkspace.A[2279] = acadoWorkspace.evHu[53];
acadoWorkspace.A[2308] = acadoWorkspace.evHu[54];
acadoWorkspace.A[2337] = acadoWorkspace.evHu[55];
acadoWorkspace.A[2367] = acadoWorkspace.evHu[56];
acadoWorkspace.A[2396] = acadoWorkspace.evHu[57];
acadoWorkspace.A[2425] = acadoWorkspace.evHu[58];
acadoWorkspace.A[2454] = acadoWorkspace.evHu[59];
acadoWorkspace.A[2484] = acadoWorkspace.evHu[60];
acadoWorkspace.A[2513] = acadoWorkspace.evHu[61];
acadoWorkspace.A[2542] = acadoWorkspace.evHu[62];
acadoWorkspace.A[2571] = acadoWorkspace.evHu[63];
acadoWorkspace.A[2601] = acadoWorkspace.evHu[64];
acadoWorkspace.A[2630] = acadoWorkspace.evHu[65];
acadoWorkspace.A[2659] = acadoWorkspace.evHu[66];
acadoWorkspace.A[2688] = acadoWorkspace.evHu[67];
acadoWorkspace.A[2718] = acadoWorkspace.evHu[68];
acadoWorkspace.A[2747] = acadoWorkspace.evHu[69];
acadoWorkspace.A[2776] = acadoWorkspace.evHu[70];
acadoWorkspace.A[2805] = acadoWorkspace.evHu[71];
acadoWorkspace.A[2835] = acadoWorkspace.evHu[72];
acadoWorkspace.A[2864] = acadoWorkspace.evHu[73];
acadoWorkspace.A[2893] = acadoWorkspace.evHu[74];
acadoWorkspace.A[2922] = acadoWorkspace.evHu[75];
acadoWorkspace.A[2952] = acadoWorkspace.evHu[76];
acadoWorkspace.A[2981] = acadoWorkspace.evHu[77];
acadoWorkspace.A[3010] = acadoWorkspace.evHu[78];
acadoWorkspace.A[3039] = acadoWorkspace.evHu[79];
acadoWorkspace.A[3069] = acadoWorkspace.evHu[80];
acadoWorkspace.A[3098] = acadoWorkspace.evHu[81];
acadoWorkspace.A[3127] = acadoWorkspace.evHu[82];
acadoWorkspace.A[3156] = acadoWorkspace.evHu[83];
acadoWorkspace.A[3186] = acadoWorkspace.evHu[84];
acadoWorkspace.A[3215] = acadoWorkspace.evHu[85];
acadoWorkspace.A[3244] = acadoWorkspace.evHu[86];
acadoWorkspace.A[3273] = acadoWorkspace.evHu[87];
acadoWorkspace.A[3303] = acadoWorkspace.evHu[88];
acadoWorkspace.A[3332] = acadoWorkspace.evHu[89];
acadoWorkspace.A[3361] = acadoWorkspace.evHu[90];
acadoWorkspace.A[3390] = acadoWorkspace.evHu[91];
acadoWorkspace.A[3420] = acadoWorkspace.evHu[92];
acadoWorkspace.A[3449] = acadoWorkspace.evHu[93];
acadoWorkspace.A[3478] = acadoWorkspace.evHu[94];
acadoWorkspace.A[3507] = acadoWorkspace.evHu[95];
acadoWorkspace.A[3537] = acadoWorkspace.evHu[96];
acadoWorkspace.A[3566] = acadoWorkspace.evHu[97];
acadoWorkspace.A[3595] = acadoWorkspace.evHu[98];
acadoWorkspace.A[3624] = acadoWorkspace.evHu[99];
acadoWorkspace.lbA[25] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[0];
acadoWorkspace.lbA[26] = - acadoWorkspace.evH[1];
acadoWorkspace.lbA[27] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[2];
acadoWorkspace.lbA[28] = - acadoWorkspace.evH[3];
acadoWorkspace.lbA[29] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[4];
acadoWorkspace.lbA[30] = - acadoWorkspace.evH[5];
acadoWorkspace.lbA[31] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[6];
acadoWorkspace.lbA[32] = - acadoWorkspace.evH[7];
acadoWorkspace.lbA[33] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[8];
acadoWorkspace.lbA[34] = - acadoWorkspace.evH[9];
acadoWorkspace.lbA[35] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[10];
acadoWorkspace.lbA[36] = - acadoWorkspace.evH[11];
acadoWorkspace.lbA[37] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[12];
acadoWorkspace.lbA[38] = - acadoWorkspace.evH[13];
acadoWorkspace.lbA[39] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[14];
acadoWorkspace.lbA[40] = - acadoWorkspace.evH[15];
acadoWorkspace.lbA[41] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[16];
acadoWorkspace.lbA[42] = - acadoWorkspace.evH[17];
acadoWorkspace.lbA[43] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[18];
acadoWorkspace.lbA[44] = - acadoWorkspace.evH[19];
acadoWorkspace.lbA[45] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[20];
acadoWorkspace.lbA[46] = - acadoWorkspace.evH[21];
acadoWorkspace.lbA[47] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[22];
acadoWorkspace.lbA[48] = - acadoWorkspace.evH[23];
acadoWorkspace.lbA[49] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[24];
acadoWorkspace.lbA[50] = - acadoWorkspace.evH[25];
acadoWorkspace.lbA[51] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[26];
acadoWorkspace.lbA[52] = - acadoWorkspace.evH[27];
acadoWorkspace.lbA[53] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[28];
acadoWorkspace.lbA[54] = - acadoWorkspace.evH[29];
acadoWorkspace.lbA[55] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[30];
acadoWorkspace.lbA[56] = - acadoWorkspace.evH[31];
acadoWorkspace.lbA[57] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[32];
acadoWorkspace.lbA[58] = - acadoWorkspace.evH[33];
acadoWorkspace.lbA[59] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[34];
acadoWorkspace.lbA[60] = - acadoWorkspace.evH[35];
acadoWorkspace.lbA[61] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[36];
acadoWorkspace.lbA[62] = - acadoWorkspace.evH[37];
acadoWorkspace.lbA[63] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[38];
acadoWorkspace.lbA[64] = - acadoWorkspace.evH[39];
acadoWorkspace.lbA[65] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[40];
acadoWorkspace.lbA[66] = - acadoWorkspace.evH[41];
acadoWorkspace.lbA[67] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[42];
acadoWorkspace.lbA[68] = - acadoWorkspace.evH[43];
acadoWorkspace.lbA[69] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[44];
acadoWorkspace.lbA[70] = - acadoWorkspace.evH[45];
acadoWorkspace.lbA[71] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[46];
acadoWorkspace.lbA[72] = - acadoWorkspace.evH[47];
acadoWorkspace.lbA[73] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[48];
acadoWorkspace.lbA[74] = - acadoWorkspace.evH[49];
acadoWorkspace.lbA[75] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[50];
acadoWorkspace.lbA[76] = - acadoWorkspace.evH[51];
acadoWorkspace.lbA[77] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[52];
acadoWorkspace.lbA[78] = - acadoWorkspace.evH[53];
acadoWorkspace.lbA[79] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[54];
acadoWorkspace.lbA[80] = - acadoWorkspace.evH[55];
acadoWorkspace.lbA[81] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[56];
acadoWorkspace.lbA[82] = - acadoWorkspace.evH[57];
acadoWorkspace.lbA[83] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[58];
acadoWorkspace.lbA[84] = - acadoWorkspace.evH[59];
acadoWorkspace.lbA[85] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[60];
acadoWorkspace.lbA[86] = - acadoWorkspace.evH[61];
acadoWorkspace.lbA[87] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[62];
acadoWorkspace.lbA[88] = - acadoWorkspace.evH[63];
acadoWorkspace.lbA[89] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[64];
acadoWorkspace.lbA[90] = - acadoWorkspace.evH[65];
acadoWorkspace.lbA[91] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[66];
acadoWorkspace.lbA[92] = - acadoWorkspace.evH[67];
acadoWorkspace.lbA[93] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[68];
acadoWorkspace.lbA[94] = - acadoWorkspace.evH[69];
acadoWorkspace.lbA[95] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[70];
acadoWorkspace.lbA[96] = - acadoWorkspace.evH[71];
acadoWorkspace.lbA[97] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[72];
acadoWorkspace.lbA[98] = - acadoWorkspace.evH[73];
acadoWorkspace.lbA[99] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[74];
acadoWorkspace.lbA[100] = - acadoWorkspace.evH[75];
acadoWorkspace.lbA[101] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[76];
acadoWorkspace.lbA[102] = - acadoWorkspace.evH[77];
acadoWorkspace.lbA[103] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[78];
acadoWorkspace.lbA[104] = - acadoWorkspace.evH[79];
acadoWorkspace.lbA[105] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[80];
acadoWorkspace.lbA[106] = - acadoWorkspace.evH[81];
acadoWorkspace.lbA[107] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[82];
acadoWorkspace.lbA[108] = - acadoWorkspace.evH[83];
acadoWorkspace.lbA[109] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[84];
acadoWorkspace.lbA[110] = - acadoWorkspace.evH[85];
acadoWorkspace.lbA[111] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[86];
acadoWorkspace.lbA[112] = - acadoWorkspace.evH[87];
acadoWorkspace.lbA[113] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[88];
acadoWorkspace.lbA[114] = - acadoWorkspace.evH[89];
acadoWorkspace.lbA[115] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[90];
acadoWorkspace.lbA[116] = - acadoWorkspace.evH[91];
acadoWorkspace.lbA[117] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[92];
acadoWorkspace.lbA[118] = - acadoWorkspace.evH[93];
acadoWorkspace.lbA[119] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[94];
acadoWorkspace.lbA[120] = - acadoWorkspace.evH[95];
acadoWorkspace.lbA[121] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[96];
acadoWorkspace.lbA[122] = - acadoWorkspace.evH[97];
acadoWorkspace.lbA[123] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[98];
acadoWorkspace.lbA[124] = - acadoWorkspace.evH[99];

acadoWorkspace.ubA[25] = - acadoWorkspace.evH[0];
acadoWorkspace.ubA[26] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[1];
acadoWorkspace.ubA[27] = - acadoWorkspace.evH[2];
acadoWorkspace.ubA[28] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[3];
acadoWorkspace.ubA[29] = - acadoWorkspace.evH[4];
acadoWorkspace.ubA[30] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[5];
acadoWorkspace.ubA[31] = - acadoWorkspace.evH[6];
acadoWorkspace.ubA[32] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[7];
acadoWorkspace.ubA[33] = - acadoWorkspace.evH[8];
acadoWorkspace.ubA[34] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[9];
acadoWorkspace.ubA[35] = - acadoWorkspace.evH[10];
acadoWorkspace.ubA[36] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[11];
acadoWorkspace.ubA[37] = - acadoWorkspace.evH[12];
acadoWorkspace.ubA[38] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[13];
acadoWorkspace.ubA[39] = - acadoWorkspace.evH[14];
acadoWorkspace.ubA[40] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[15];
acadoWorkspace.ubA[41] = - acadoWorkspace.evH[16];
acadoWorkspace.ubA[42] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[17];
acadoWorkspace.ubA[43] = - acadoWorkspace.evH[18];
acadoWorkspace.ubA[44] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[19];
acadoWorkspace.ubA[45] = - acadoWorkspace.evH[20];
acadoWorkspace.ubA[46] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[21];
acadoWorkspace.ubA[47] = - acadoWorkspace.evH[22];
acadoWorkspace.ubA[48] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[23];
acadoWorkspace.ubA[49] = - acadoWorkspace.evH[24];
acadoWorkspace.ubA[50] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[25];
acadoWorkspace.ubA[51] = - acadoWorkspace.evH[26];
acadoWorkspace.ubA[52] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[27];
acadoWorkspace.ubA[53] = - acadoWorkspace.evH[28];
acadoWorkspace.ubA[54] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[29];
acadoWorkspace.ubA[55] = - acadoWorkspace.evH[30];
acadoWorkspace.ubA[56] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[31];
acadoWorkspace.ubA[57] = - acadoWorkspace.evH[32];
acadoWorkspace.ubA[58] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[33];
acadoWorkspace.ubA[59] = - acadoWorkspace.evH[34];
acadoWorkspace.ubA[60] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[35];
acadoWorkspace.ubA[61] = - acadoWorkspace.evH[36];
acadoWorkspace.ubA[62] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[37];
acadoWorkspace.ubA[63] = - acadoWorkspace.evH[38];
acadoWorkspace.ubA[64] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[39];
acadoWorkspace.ubA[65] = - acadoWorkspace.evH[40];
acadoWorkspace.ubA[66] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[41];
acadoWorkspace.ubA[67] = - acadoWorkspace.evH[42];
acadoWorkspace.ubA[68] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[43];
acadoWorkspace.ubA[69] = - acadoWorkspace.evH[44];
acadoWorkspace.ubA[70] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[45];
acadoWorkspace.ubA[71] = - acadoWorkspace.evH[46];
acadoWorkspace.ubA[72] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[47];
acadoWorkspace.ubA[73] = - acadoWorkspace.evH[48];
acadoWorkspace.ubA[74] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[49];
acadoWorkspace.ubA[75] = - acadoWorkspace.evH[50];
acadoWorkspace.ubA[76] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[51];
acadoWorkspace.ubA[77] = - acadoWorkspace.evH[52];
acadoWorkspace.ubA[78] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[53];
acadoWorkspace.ubA[79] = - acadoWorkspace.evH[54];
acadoWorkspace.ubA[80] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[55];
acadoWorkspace.ubA[81] = - acadoWorkspace.evH[56];
acadoWorkspace.ubA[82] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[57];
acadoWorkspace.ubA[83] = - acadoWorkspace.evH[58];
acadoWorkspace.ubA[84] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[59];
acadoWorkspace.ubA[85] = - acadoWorkspace.evH[60];
acadoWorkspace.ubA[86] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[61];
acadoWorkspace.ubA[87] = - acadoWorkspace.evH[62];
acadoWorkspace.ubA[88] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[63];
acadoWorkspace.ubA[89] = - acadoWorkspace.evH[64];
acadoWorkspace.ubA[90] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[65];
acadoWorkspace.ubA[91] = - acadoWorkspace.evH[66];
acadoWorkspace.ubA[92] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[67];
acadoWorkspace.ubA[93] = - acadoWorkspace.evH[68];
acadoWorkspace.ubA[94] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[69];
acadoWorkspace.ubA[95] = - acadoWorkspace.evH[70];
acadoWorkspace.ubA[96] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[71];
acadoWorkspace.ubA[97] = - acadoWorkspace.evH[72];
acadoWorkspace.ubA[98] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[73];
acadoWorkspace.ubA[99] = - acadoWorkspace.evH[74];
acadoWorkspace.ubA[100] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[75];
acadoWorkspace.ubA[101] = - acadoWorkspace.evH[76];
acadoWorkspace.ubA[102] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[77];
acadoWorkspace.ubA[103] = - acadoWorkspace.evH[78];
acadoWorkspace.ubA[104] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[79];
acadoWorkspace.ubA[105] = - acadoWorkspace.evH[80];
acadoWorkspace.ubA[106] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[81];
acadoWorkspace.ubA[107] = - acadoWorkspace.evH[82];
acadoWorkspace.ubA[108] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[83];
acadoWorkspace.ubA[109] = - acadoWorkspace.evH[84];
acadoWorkspace.ubA[110] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[85];
acadoWorkspace.ubA[111] = - acadoWorkspace.evH[86];
acadoWorkspace.ubA[112] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[87];
acadoWorkspace.ubA[113] = - acadoWorkspace.evH[88];
acadoWorkspace.ubA[114] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[89];
acadoWorkspace.ubA[115] = - acadoWorkspace.evH[90];
acadoWorkspace.ubA[116] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[91];
acadoWorkspace.ubA[117] = - acadoWorkspace.evH[92];
acadoWorkspace.ubA[118] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[93];
acadoWorkspace.ubA[119] = - acadoWorkspace.evH[94];
acadoWorkspace.ubA[120] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[95];
acadoWorkspace.ubA[121] = - acadoWorkspace.evH[96];
acadoWorkspace.ubA[122] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[97];
acadoWorkspace.ubA[123] = - acadoWorkspace.evH[98];
acadoWorkspace.ubA[124] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[99];

acado_macHxd( &(acadoWorkspace.evHx[ 16 ]), acadoWorkspace.d, &(acadoWorkspace.lbA[ 29 ]), &(acadoWorkspace.ubA[ 29 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 32 ]), &(acadoWorkspace.d[ 4 ]), &(acadoWorkspace.lbA[ 33 ]), &(acadoWorkspace.ubA[ 33 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.d[ 8 ]), &(acadoWorkspace.lbA[ 37 ]), &(acadoWorkspace.ubA[ 37 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 64 ]), &(acadoWorkspace.d[ 12 ]), &(acadoWorkspace.lbA[ 41 ]), &(acadoWorkspace.ubA[ 41 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 80 ]), &(acadoWorkspace.d[ 16 ]), &(acadoWorkspace.lbA[ 45 ]), &(acadoWorkspace.ubA[ 45 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.d[ 20 ]), &(acadoWorkspace.lbA[ 49 ]), &(acadoWorkspace.ubA[ 49 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 112 ]), &(acadoWorkspace.d[ 24 ]), &(acadoWorkspace.lbA[ 53 ]), &(acadoWorkspace.ubA[ 53 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 128 ]), &(acadoWorkspace.d[ 28 ]), &(acadoWorkspace.lbA[ 57 ]), &(acadoWorkspace.ubA[ 57 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.d[ 32 ]), &(acadoWorkspace.lbA[ 61 ]), &(acadoWorkspace.ubA[ 61 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.d[ 36 ]), &(acadoWorkspace.lbA[ 65 ]), &(acadoWorkspace.ubA[ 65 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 176 ]), &(acadoWorkspace.d[ 40 ]), &(acadoWorkspace.lbA[ 69 ]), &(acadoWorkspace.ubA[ 69 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.d[ 44 ]), &(acadoWorkspace.lbA[ 73 ]), &(acadoWorkspace.ubA[ 73 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 208 ]), &(acadoWorkspace.d[ 48 ]), &(acadoWorkspace.lbA[ 77 ]), &(acadoWorkspace.ubA[ 77 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 224 ]), &(acadoWorkspace.d[ 52 ]), &(acadoWorkspace.lbA[ 81 ]), &(acadoWorkspace.ubA[ 81 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 240 ]), &(acadoWorkspace.d[ 56 ]), &(acadoWorkspace.lbA[ 85 ]), &(acadoWorkspace.ubA[ 85 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 256 ]), &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.lbA[ 89 ]), &(acadoWorkspace.ubA[ 89 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 272 ]), &(acadoWorkspace.d[ 64 ]), &(acadoWorkspace.lbA[ 93 ]), &(acadoWorkspace.ubA[ 93 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 288 ]), &(acadoWorkspace.d[ 68 ]), &(acadoWorkspace.lbA[ 97 ]), &(acadoWorkspace.ubA[ 97 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 304 ]), &(acadoWorkspace.d[ 72 ]), &(acadoWorkspace.lbA[ 101 ]), &(acadoWorkspace.ubA[ 101 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 320 ]), &(acadoWorkspace.d[ 76 ]), &(acadoWorkspace.lbA[ 105 ]), &(acadoWorkspace.ubA[ 105 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 336 ]), &(acadoWorkspace.d[ 80 ]), &(acadoWorkspace.lbA[ 109 ]), &(acadoWorkspace.ubA[ 109 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 352 ]), &(acadoWorkspace.d[ 84 ]), &(acadoWorkspace.lbA[ 113 ]), &(acadoWorkspace.ubA[ 113 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 368 ]), &(acadoWorkspace.d[ 88 ]), &(acadoWorkspace.lbA[ 117 ]), &(acadoWorkspace.ubA[ 117 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 384 ]), &(acadoWorkspace.d[ 92 ]), &(acadoWorkspace.lbA[ 121 ]), &(acadoWorkspace.ubA[ 121 ]) );

}

void acado_condenseFdb(  )
{
int lRun1;
int lRun2;
int lRun3;
real_t tmp;

acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];

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
acadoWorkspace.Dy[65] -= acadoVariables.y[65];
acadoWorkspace.Dy[66] -= acadoVariables.y[66];
acadoWorkspace.Dy[67] -= acadoVariables.y[67];
acadoWorkspace.Dy[68] -= acadoVariables.y[68];
acadoWorkspace.Dy[69] -= acadoVariables.y[69];
acadoWorkspace.Dy[70] -= acadoVariables.y[70];
acadoWorkspace.Dy[71] -= acadoVariables.y[71];
acadoWorkspace.Dy[72] -= acadoVariables.y[72];
acadoWorkspace.Dy[73] -= acadoVariables.y[73];
acadoWorkspace.Dy[74] -= acadoVariables.y[74];
acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 3 ]), &(acadoWorkspace.Dy[ 3 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 6 ]), &(acadoWorkspace.Dy[ 6 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 9 ]), &(acadoWorkspace.Dy[ 9 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 12 ]), &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 15 ]), &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 18 ]), &(acadoWorkspace.Dy[ 18 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 21 ]), &(acadoWorkspace.Dy[ 21 ]), &(acadoWorkspace.g[ 11 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 24 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 27 ]), &(acadoWorkspace.Dy[ 27 ]), &(acadoWorkspace.g[ 13 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 30 ]), &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 33 ]), &(acadoWorkspace.Dy[ 33 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 36 ]), &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 39 ]), &(acadoWorkspace.Dy[ 39 ]), &(acadoWorkspace.g[ 17 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 42 ]), &(acadoWorkspace.Dy[ 42 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 45 ]), &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.g[ 19 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 48 ]), &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 51 ]), &(acadoWorkspace.Dy[ 51 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 54 ]), &(acadoWorkspace.Dy[ 54 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 57 ]), &(acadoWorkspace.Dy[ 57 ]), &(acadoWorkspace.g[ 23 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 60 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 63 ]), &(acadoWorkspace.Dy[ 63 ]), &(acadoWorkspace.g[ 25 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 66 ]), &(acadoWorkspace.Dy[ 66 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 69 ]), &(acadoWorkspace.Dy[ 69 ]), &(acadoWorkspace.g[ 27 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 72 ]), &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.g[ 28 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 12 ]), &(acadoWorkspace.Dy[ 3 ]), &(acadoWorkspace.QDy[ 4 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 24 ]), &(acadoWorkspace.Dy[ 6 ]), &(acadoWorkspace.QDy[ 8 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 36 ]), &(acadoWorkspace.Dy[ 9 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 48 ]), &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.QDy[ 16 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 60 ]), &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.QDy[ 20 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 72 ]), &(acadoWorkspace.Dy[ 18 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 84 ]), &(acadoWorkspace.Dy[ 21 ]), &(acadoWorkspace.QDy[ 28 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 96 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.QDy[ 32 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 108 ]), &(acadoWorkspace.Dy[ 27 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 120 ]), &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.QDy[ 40 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 132 ]), &(acadoWorkspace.Dy[ 33 ]), &(acadoWorkspace.QDy[ 44 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 144 ]), &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.QDy[ 48 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 156 ]), &(acadoWorkspace.Dy[ 39 ]), &(acadoWorkspace.QDy[ 52 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 168 ]), &(acadoWorkspace.Dy[ 42 ]), &(acadoWorkspace.QDy[ 56 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 180 ]), &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.QDy[ 60 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 192 ]), &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.QDy[ 64 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 204 ]), &(acadoWorkspace.Dy[ 51 ]), &(acadoWorkspace.QDy[ 68 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 216 ]), &(acadoWorkspace.Dy[ 54 ]), &(acadoWorkspace.QDy[ 72 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 228 ]), &(acadoWorkspace.Dy[ 57 ]), &(acadoWorkspace.QDy[ 76 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 240 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.QDy[ 80 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 252 ]), &(acadoWorkspace.Dy[ 63 ]), &(acadoWorkspace.QDy[ 84 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 264 ]), &(acadoWorkspace.Dy[ 66 ]), &(acadoWorkspace.QDy[ 88 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 276 ]), &(acadoWorkspace.Dy[ 69 ]), &(acadoWorkspace.QDy[ 92 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 288 ]), &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.QDy[ 96 ]) );

acadoWorkspace.QDy[100] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[101] = + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[102] = + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[103] = + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[1];

acadoWorkspace.QDy[4] += acadoWorkspace.Qd[0];
acadoWorkspace.QDy[5] += acadoWorkspace.Qd[1];
acadoWorkspace.QDy[6] += acadoWorkspace.Qd[2];
acadoWorkspace.QDy[7] += acadoWorkspace.Qd[3];
acadoWorkspace.QDy[8] += acadoWorkspace.Qd[4];
acadoWorkspace.QDy[9] += acadoWorkspace.Qd[5];
acadoWorkspace.QDy[10] += acadoWorkspace.Qd[6];
acadoWorkspace.QDy[11] += acadoWorkspace.Qd[7];
acadoWorkspace.QDy[12] += acadoWorkspace.Qd[8];
acadoWorkspace.QDy[13] += acadoWorkspace.Qd[9];
acadoWorkspace.QDy[14] += acadoWorkspace.Qd[10];
acadoWorkspace.QDy[15] += acadoWorkspace.Qd[11];
acadoWorkspace.QDy[16] += acadoWorkspace.Qd[12];
acadoWorkspace.QDy[17] += acadoWorkspace.Qd[13];
acadoWorkspace.QDy[18] += acadoWorkspace.Qd[14];
acadoWorkspace.QDy[19] += acadoWorkspace.Qd[15];
acadoWorkspace.QDy[20] += acadoWorkspace.Qd[16];
acadoWorkspace.QDy[21] += acadoWorkspace.Qd[17];
acadoWorkspace.QDy[22] += acadoWorkspace.Qd[18];
acadoWorkspace.QDy[23] += acadoWorkspace.Qd[19];
acadoWorkspace.QDy[24] += acadoWorkspace.Qd[20];
acadoWorkspace.QDy[25] += acadoWorkspace.Qd[21];
acadoWorkspace.QDy[26] += acadoWorkspace.Qd[22];
acadoWorkspace.QDy[27] += acadoWorkspace.Qd[23];
acadoWorkspace.QDy[28] += acadoWorkspace.Qd[24];
acadoWorkspace.QDy[29] += acadoWorkspace.Qd[25];
acadoWorkspace.QDy[30] += acadoWorkspace.Qd[26];
acadoWorkspace.QDy[31] += acadoWorkspace.Qd[27];
acadoWorkspace.QDy[32] += acadoWorkspace.Qd[28];
acadoWorkspace.QDy[33] += acadoWorkspace.Qd[29];
acadoWorkspace.QDy[34] += acadoWorkspace.Qd[30];
acadoWorkspace.QDy[35] += acadoWorkspace.Qd[31];
acadoWorkspace.QDy[36] += acadoWorkspace.Qd[32];
acadoWorkspace.QDy[37] += acadoWorkspace.Qd[33];
acadoWorkspace.QDy[38] += acadoWorkspace.Qd[34];
acadoWorkspace.QDy[39] += acadoWorkspace.Qd[35];
acadoWorkspace.QDy[40] += acadoWorkspace.Qd[36];
acadoWorkspace.QDy[41] += acadoWorkspace.Qd[37];
acadoWorkspace.QDy[42] += acadoWorkspace.Qd[38];
acadoWorkspace.QDy[43] += acadoWorkspace.Qd[39];
acadoWorkspace.QDy[44] += acadoWorkspace.Qd[40];
acadoWorkspace.QDy[45] += acadoWorkspace.Qd[41];
acadoWorkspace.QDy[46] += acadoWorkspace.Qd[42];
acadoWorkspace.QDy[47] += acadoWorkspace.Qd[43];
acadoWorkspace.QDy[48] += acadoWorkspace.Qd[44];
acadoWorkspace.QDy[49] += acadoWorkspace.Qd[45];
acadoWorkspace.QDy[50] += acadoWorkspace.Qd[46];
acadoWorkspace.QDy[51] += acadoWorkspace.Qd[47];
acadoWorkspace.QDy[52] += acadoWorkspace.Qd[48];
acadoWorkspace.QDy[53] += acadoWorkspace.Qd[49];
acadoWorkspace.QDy[54] += acadoWorkspace.Qd[50];
acadoWorkspace.QDy[55] += acadoWorkspace.Qd[51];
acadoWorkspace.QDy[56] += acadoWorkspace.Qd[52];
acadoWorkspace.QDy[57] += acadoWorkspace.Qd[53];
acadoWorkspace.QDy[58] += acadoWorkspace.Qd[54];
acadoWorkspace.QDy[59] += acadoWorkspace.Qd[55];
acadoWorkspace.QDy[60] += acadoWorkspace.Qd[56];
acadoWorkspace.QDy[61] += acadoWorkspace.Qd[57];
acadoWorkspace.QDy[62] += acadoWorkspace.Qd[58];
acadoWorkspace.QDy[63] += acadoWorkspace.Qd[59];
acadoWorkspace.QDy[64] += acadoWorkspace.Qd[60];
acadoWorkspace.QDy[65] += acadoWorkspace.Qd[61];
acadoWorkspace.QDy[66] += acadoWorkspace.Qd[62];
acadoWorkspace.QDy[67] += acadoWorkspace.Qd[63];
acadoWorkspace.QDy[68] += acadoWorkspace.Qd[64];
acadoWorkspace.QDy[69] += acadoWorkspace.Qd[65];
acadoWorkspace.QDy[70] += acadoWorkspace.Qd[66];
acadoWorkspace.QDy[71] += acadoWorkspace.Qd[67];
acadoWorkspace.QDy[72] += acadoWorkspace.Qd[68];
acadoWorkspace.QDy[73] += acadoWorkspace.Qd[69];
acadoWorkspace.QDy[74] += acadoWorkspace.Qd[70];
acadoWorkspace.QDy[75] += acadoWorkspace.Qd[71];
acadoWorkspace.QDy[76] += acadoWorkspace.Qd[72];
acadoWorkspace.QDy[77] += acadoWorkspace.Qd[73];
acadoWorkspace.QDy[78] += acadoWorkspace.Qd[74];
acadoWorkspace.QDy[79] += acadoWorkspace.Qd[75];
acadoWorkspace.QDy[80] += acadoWorkspace.Qd[76];
acadoWorkspace.QDy[81] += acadoWorkspace.Qd[77];
acadoWorkspace.QDy[82] += acadoWorkspace.Qd[78];
acadoWorkspace.QDy[83] += acadoWorkspace.Qd[79];
acadoWorkspace.QDy[84] += acadoWorkspace.Qd[80];
acadoWorkspace.QDy[85] += acadoWorkspace.Qd[81];
acadoWorkspace.QDy[86] += acadoWorkspace.Qd[82];
acadoWorkspace.QDy[87] += acadoWorkspace.Qd[83];
acadoWorkspace.QDy[88] += acadoWorkspace.Qd[84];
acadoWorkspace.QDy[89] += acadoWorkspace.Qd[85];
acadoWorkspace.QDy[90] += acadoWorkspace.Qd[86];
acadoWorkspace.QDy[91] += acadoWorkspace.Qd[87];
acadoWorkspace.QDy[92] += acadoWorkspace.Qd[88];
acadoWorkspace.QDy[93] += acadoWorkspace.Qd[89];
acadoWorkspace.QDy[94] += acadoWorkspace.Qd[90];
acadoWorkspace.QDy[95] += acadoWorkspace.Qd[91];
acadoWorkspace.QDy[96] += acadoWorkspace.Qd[92];
acadoWorkspace.QDy[97] += acadoWorkspace.Qd[93];
acadoWorkspace.QDy[98] += acadoWorkspace.Qd[94];
acadoWorkspace.QDy[99] += acadoWorkspace.Qd[95];
acadoWorkspace.QDy[100] += acadoWorkspace.Qd[96];
acadoWorkspace.QDy[101] += acadoWorkspace.Qd[97];
acadoWorkspace.QDy[102] += acadoWorkspace.Qd[98];
acadoWorkspace.QDy[103] += acadoWorkspace.Qd[99];

acadoWorkspace.g[0] = + acadoWorkspace.evGx[0]*acadoWorkspace.QDy[4] + acadoWorkspace.evGx[4]*acadoWorkspace.QDy[5] + acadoWorkspace.evGx[8]*acadoWorkspace.QDy[6] + acadoWorkspace.evGx[12]*acadoWorkspace.QDy[7] + acadoWorkspace.evGx[16]*acadoWorkspace.QDy[8] + acadoWorkspace.evGx[20]*acadoWorkspace.QDy[9] + acadoWorkspace.evGx[24]*acadoWorkspace.QDy[10] + acadoWorkspace.evGx[28]*acadoWorkspace.QDy[11] + acadoWorkspace.evGx[32]*acadoWorkspace.QDy[12] + acadoWorkspace.evGx[36]*acadoWorkspace.QDy[13] + acadoWorkspace.evGx[40]*acadoWorkspace.QDy[14] + acadoWorkspace.evGx[44]*acadoWorkspace.QDy[15] + acadoWorkspace.evGx[48]*acadoWorkspace.QDy[16] + acadoWorkspace.evGx[52]*acadoWorkspace.QDy[17] + acadoWorkspace.evGx[56]*acadoWorkspace.QDy[18] + acadoWorkspace.evGx[60]*acadoWorkspace.QDy[19] + acadoWorkspace.evGx[64]*acadoWorkspace.QDy[20] + acadoWorkspace.evGx[68]*acadoWorkspace.QDy[21] + acadoWorkspace.evGx[72]*acadoWorkspace.QDy[22] + acadoWorkspace.evGx[76]*acadoWorkspace.QDy[23] + acadoWorkspace.evGx[80]*acadoWorkspace.QDy[24] + acadoWorkspace.evGx[84]*acadoWorkspace.QDy[25] + acadoWorkspace.evGx[88]*acadoWorkspace.QDy[26] + acadoWorkspace.evGx[92]*acadoWorkspace.QDy[27] + acadoWorkspace.evGx[96]*acadoWorkspace.QDy[28] + acadoWorkspace.evGx[100]*acadoWorkspace.QDy[29] + acadoWorkspace.evGx[104]*acadoWorkspace.QDy[30] + acadoWorkspace.evGx[108]*acadoWorkspace.QDy[31] + acadoWorkspace.evGx[112]*acadoWorkspace.QDy[32] + acadoWorkspace.evGx[116]*acadoWorkspace.QDy[33] + acadoWorkspace.evGx[120]*acadoWorkspace.QDy[34] + acadoWorkspace.evGx[124]*acadoWorkspace.QDy[35] + acadoWorkspace.evGx[128]*acadoWorkspace.QDy[36] + acadoWorkspace.evGx[132]*acadoWorkspace.QDy[37] + acadoWorkspace.evGx[136]*acadoWorkspace.QDy[38] + acadoWorkspace.evGx[140]*acadoWorkspace.QDy[39] + acadoWorkspace.evGx[144]*acadoWorkspace.QDy[40] + acadoWorkspace.evGx[148]*acadoWorkspace.QDy[41] + acadoWorkspace.evGx[152]*acadoWorkspace.QDy[42] + acadoWorkspace.evGx[156]*acadoWorkspace.QDy[43] + acadoWorkspace.evGx[160]*acadoWorkspace.QDy[44] + acadoWorkspace.evGx[164]*acadoWorkspace.QDy[45] + acadoWorkspace.evGx[168]*acadoWorkspace.QDy[46] + acadoWorkspace.evGx[172]*acadoWorkspace.QDy[47] + acadoWorkspace.evGx[176]*acadoWorkspace.QDy[48] + acadoWorkspace.evGx[180]*acadoWorkspace.QDy[49] + acadoWorkspace.evGx[184]*acadoWorkspace.QDy[50] + acadoWorkspace.evGx[188]*acadoWorkspace.QDy[51] + acadoWorkspace.evGx[192]*acadoWorkspace.QDy[52] + acadoWorkspace.evGx[196]*acadoWorkspace.QDy[53] + acadoWorkspace.evGx[200]*acadoWorkspace.QDy[54] + acadoWorkspace.evGx[204]*acadoWorkspace.QDy[55] + acadoWorkspace.evGx[208]*acadoWorkspace.QDy[56] + acadoWorkspace.evGx[212]*acadoWorkspace.QDy[57] + acadoWorkspace.evGx[216]*acadoWorkspace.QDy[58] + acadoWorkspace.evGx[220]*acadoWorkspace.QDy[59] + acadoWorkspace.evGx[224]*acadoWorkspace.QDy[60] + acadoWorkspace.evGx[228]*acadoWorkspace.QDy[61] + acadoWorkspace.evGx[232]*acadoWorkspace.QDy[62] + acadoWorkspace.evGx[236]*acadoWorkspace.QDy[63] + acadoWorkspace.evGx[240]*acadoWorkspace.QDy[64] + acadoWorkspace.evGx[244]*acadoWorkspace.QDy[65] + acadoWorkspace.evGx[248]*acadoWorkspace.QDy[66] + acadoWorkspace.evGx[252]*acadoWorkspace.QDy[67] + acadoWorkspace.evGx[256]*acadoWorkspace.QDy[68] + acadoWorkspace.evGx[260]*acadoWorkspace.QDy[69] + acadoWorkspace.evGx[264]*acadoWorkspace.QDy[70] + acadoWorkspace.evGx[268]*acadoWorkspace.QDy[71] + acadoWorkspace.evGx[272]*acadoWorkspace.QDy[72] + acadoWorkspace.evGx[276]*acadoWorkspace.QDy[73] + acadoWorkspace.evGx[280]*acadoWorkspace.QDy[74] + acadoWorkspace.evGx[284]*acadoWorkspace.QDy[75] + acadoWorkspace.evGx[288]*acadoWorkspace.QDy[76] + acadoWorkspace.evGx[292]*acadoWorkspace.QDy[77] + acadoWorkspace.evGx[296]*acadoWorkspace.QDy[78] + acadoWorkspace.evGx[300]*acadoWorkspace.QDy[79] + acadoWorkspace.evGx[304]*acadoWorkspace.QDy[80] + acadoWorkspace.evGx[308]*acadoWorkspace.QDy[81] + acadoWorkspace.evGx[312]*acadoWorkspace.QDy[82] + acadoWorkspace.evGx[316]*acadoWorkspace.QDy[83] + acadoWorkspace.evGx[320]*acadoWorkspace.QDy[84] + acadoWorkspace.evGx[324]*acadoWorkspace.QDy[85] + acadoWorkspace.evGx[328]*acadoWorkspace.QDy[86] + acadoWorkspace.evGx[332]*acadoWorkspace.QDy[87] + acadoWorkspace.evGx[336]*acadoWorkspace.QDy[88] + acadoWorkspace.evGx[340]*acadoWorkspace.QDy[89] + acadoWorkspace.evGx[344]*acadoWorkspace.QDy[90] + acadoWorkspace.evGx[348]*acadoWorkspace.QDy[91] + acadoWorkspace.evGx[352]*acadoWorkspace.QDy[92] + acadoWorkspace.evGx[356]*acadoWorkspace.QDy[93] + acadoWorkspace.evGx[360]*acadoWorkspace.QDy[94] + acadoWorkspace.evGx[364]*acadoWorkspace.QDy[95] + acadoWorkspace.evGx[368]*acadoWorkspace.QDy[96] + acadoWorkspace.evGx[372]*acadoWorkspace.QDy[97] + acadoWorkspace.evGx[376]*acadoWorkspace.QDy[98] + acadoWorkspace.evGx[380]*acadoWorkspace.QDy[99] + acadoWorkspace.evGx[384]*acadoWorkspace.QDy[100] + acadoWorkspace.evGx[388]*acadoWorkspace.QDy[101] + acadoWorkspace.evGx[392]*acadoWorkspace.QDy[102] + acadoWorkspace.evGx[396]*acadoWorkspace.QDy[103];
acadoWorkspace.g[1] = + acadoWorkspace.evGx[1]*acadoWorkspace.QDy[4] + acadoWorkspace.evGx[5]*acadoWorkspace.QDy[5] + acadoWorkspace.evGx[9]*acadoWorkspace.QDy[6] + acadoWorkspace.evGx[13]*acadoWorkspace.QDy[7] + acadoWorkspace.evGx[17]*acadoWorkspace.QDy[8] + acadoWorkspace.evGx[21]*acadoWorkspace.QDy[9] + acadoWorkspace.evGx[25]*acadoWorkspace.QDy[10] + acadoWorkspace.evGx[29]*acadoWorkspace.QDy[11] + acadoWorkspace.evGx[33]*acadoWorkspace.QDy[12] + acadoWorkspace.evGx[37]*acadoWorkspace.QDy[13] + acadoWorkspace.evGx[41]*acadoWorkspace.QDy[14] + acadoWorkspace.evGx[45]*acadoWorkspace.QDy[15] + acadoWorkspace.evGx[49]*acadoWorkspace.QDy[16] + acadoWorkspace.evGx[53]*acadoWorkspace.QDy[17] + acadoWorkspace.evGx[57]*acadoWorkspace.QDy[18] + acadoWorkspace.evGx[61]*acadoWorkspace.QDy[19] + acadoWorkspace.evGx[65]*acadoWorkspace.QDy[20] + acadoWorkspace.evGx[69]*acadoWorkspace.QDy[21] + acadoWorkspace.evGx[73]*acadoWorkspace.QDy[22] + acadoWorkspace.evGx[77]*acadoWorkspace.QDy[23] + acadoWorkspace.evGx[81]*acadoWorkspace.QDy[24] + acadoWorkspace.evGx[85]*acadoWorkspace.QDy[25] + acadoWorkspace.evGx[89]*acadoWorkspace.QDy[26] + acadoWorkspace.evGx[93]*acadoWorkspace.QDy[27] + acadoWorkspace.evGx[97]*acadoWorkspace.QDy[28] + acadoWorkspace.evGx[101]*acadoWorkspace.QDy[29] + acadoWorkspace.evGx[105]*acadoWorkspace.QDy[30] + acadoWorkspace.evGx[109]*acadoWorkspace.QDy[31] + acadoWorkspace.evGx[113]*acadoWorkspace.QDy[32] + acadoWorkspace.evGx[117]*acadoWorkspace.QDy[33] + acadoWorkspace.evGx[121]*acadoWorkspace.QDy[34] + acadoWorkspace.evGx[125]*acadoWorkspace.QDy[35] + acadoWorkspace.evGx[129]*acadoWorkspace.QDy[36] + acadoWorkspace.evGx[133]*acadoWorkspace.QDy[37] + acadoWorkspace.evGx[137]*acadoWorkspace.QDy[38] + acadoWorkspace.evGx[141]*acadoWorkspace.QDy[39] + acadoWorkspace.evGx[145]*acadoWorkspace.QDy[40] + acadoWorkspace.evGx[149]*acadoWorkspace.QDy[41] + acadoWorkspace.evGx[153]*acadoWorkspace.QDy[42] + acadoWorkspace.evGx[157]*acadoWorkspace.QDy[43] + acadoWorkspace.evGx[161]*acadoWorkspace.QDy[44] + acadoWorkspace.evGx[165]*acadoWorkspace.QDy[45] + acadoWorkspace.evGx[169]*acadoWorkspace.QDy[46] + acadoWorkspace.evGx[173]*acadoWorkspace.QDy[47] + acadoWorkspace.evGx[177]*acadoWorkspace.QDy[48] + acadoWorkspace.evGx[181]*acadoWorkspace.QDy[49] + acadoWorkspace.evGx[185]*acadoWorkspace.QDy[50] + acadoWorkspace.evGx[189]*acadoWorkspace.QDy[51] + acadoWorkspace.evGx[193]*acadoWorkspace.QDy[52] + acadoWorkspace.evGx[197]*acadoWorkspace.QDy[53] + acadoWorkspace.evGx[201]*acadoWorkspace.QDy[54] + acadoWorkspace.evGx[205]*acadoWorkspace.QDy[55] + acadoWorkspace.evGx[209]*acadoWorkspace.QDy[56] + acadoWorkspace.evGx[213]*acadoWorkspace.QDy[57] + acadoWorkspace.evGx[217]*acadoWorkspace.QDy[58] + acadoWorkspace.evGx[221]*acadoWorkspace.QDy[59] + acadoWorkspace.evGx[225]*acadoWorkspace.QDy[60] + acadoWorkspace.evGx[229]*acadoWorkspace.QDy[61] + acadoWorkspace.evGx[233]*acadoWorkspace.QDy[62] + acadoWorkspace.evGx[237]*acadoWorkspace.QDy[63] + acadoWorkspace.evGx[241]*acadoWorkspace.QDy[64] + acadoWorkspace.evGx[245]*acadoWorkspace.QDy[65] + acadoWorkspace.evGx[249]*acadoWorkspace.QDy[66] + acadoWorkspace.evGx[253]*acadoWorkspace.QDy[67] + acadoWorkspace.evGx[257]*acadoWorkspace.QDy[68] + acadoWorkspace.evGx[261]*acadoWorkspace.QDy[69] + acadoWorkspace.evGx[265]*acadoWorkspace.QDy[70] + acadoWorkspace.evGx[269]*acadoWorkspace.QDy[71] + acadoWorkspace.evGx[273]*acadoWorkspace.QDy[72] + acadoWorkspace.evGx[277]*acadoWorkspace.QDy[73] + acadoWorkspace.evGx[281]*acadoWorkspace.QDy[74] + acadoWorkspace.evGx[285]*acadoWorkspace.QDy[75] + acadoWorkspace.evGx[289]*acadoWorkspace.QDy[76] + acadoWorkspace.evGx[293]*acadoWorkspace.QDy[77] + acadoWorkspace.evGx[297]*acadoWorkspace.QDy[78] + acadoWorkspace.evGx[301]*acadoWorkspace.QDy[79] + acadoWorkspace.evGx[305]*acadoWorkspace.QDy[80] + acadoWorkspace.evGx[309]*acadoWorkspace.QDy[81] + acadoWorkspace.evGx[313]*acadoWorkspace.QDy[82] + acadoWorkspace.evGx[317]*acadoWorkspace.QDy[83] + acadoWorkspace.evGx[321]*acadoWorkspace.QDy[84] + acadoWorkspace.evGx[325]*acadoWorkspace.QDy[85] + acadoWorkspace.evGx[329]*acadoWorkspace.QDy[86] + acadoWorkspace.evGx[333]*acadoWorkspace.QDy[87] + acadoWorkspace.evGx[337]*acadoWorkspace.QDy[88] + acadoWorkspace.evGx[341]*acadoWorkspace.QDy[89] + acadoWorkspace.evGx[345]*acadoWorkspace.QDy[90] + acadoWorkspace.evGx[349]*acadoWorkspace.QDy[91] + acadoWorkspace.evGx[353]*acadoWorkspace.QDy[92] + acadoWorkspace.evGx[357]*acadoWorkspace.QDy[93] + acadoWorkspace.evGx[361]*acadoWorkspace.QDy[94] + acadoWorkspace.evGx[365]*acadoWorkspace.QDy[95] + acadoWorkspace.evGx[369]*acadoWorkspace.QDy[96] + acadoWorkspace.evGx[373]*acadoWorkspace.QDy[97] + acadoWorkspace.evGx[377]*acadoWorkspace.QDy[98] + acadoWorkspace.evGx[381]*acadoWorkspace.QDy[99] + acadoWorkspace.evGx[385]*acadoWorkspace.QDy[100] + acadoWorkspace.evGx[389]*acadoWorkspace.QDy[101] + acadoWorkspace.evGx[393]*acadoWorkspace.QDy[102] + acadoWorkspace.evGx[397]*acadoWorkspace.QDy[103];
acadoWorkspace.g[2] = + acadoWorkspace.evGx[2]*acadoWorkspace.QDy[4] + acadoWorkspace.evGx[6]*acadoWorkspace.QDy[5] + acadoWorkspace.evGx[10]*acadoWorkspace.QDy[6] + acadoWorkspace.evGx[14]*acadoWorkspace.QDy[7] + acadoWorkspace.evGx[18]*acadoWorkspace.QDy[8] + acadoWorkspace.evGx[22]*acadoWorkspace.QDy[9] + acadoWorkspace.evGx[26]*acadoWorkspace.QDy[10] + acadoWorkspace.evGx[30]*acadoWorkspace.QDy[11] + acadoWorkspace.evGx[34]*acadoWorkspace.QDy[12] + acadoWorkspace.evGx[38]*acadoWorkspace.QDy[13] + acadoWorkspace.evGx[42]*acadoWorkspace.QDy[14] + acadoWorkspace.evGx[46]*acadoWorkspace.QDy[15] + acadoWorkspace.evGx[50]*acadoWorkspace.QDy[16] + acadoWorkspace.evGx[54]*acadoWorkspace.QDy[17] + acadoWorkspace.evGx[58]*acadoWorkspace.QDy[18] + acadoWorkspace.evGx[62]*acadoWorkspace.QDy[19] + acadoWorkspace.evGx[66]*acadoWorkspace.QDy[20] + acadoWorkspace.evGx[70]*acadoWorkspace.QDy[21] + acadoWorkspace.evGx[74]*acadoWorkspace.QDy[22] + acadoWorkspace.evGx[78]*acadoWorkspace.QDy[23] + acadoWorkspace.evGx[82]*acadoWorkspace.QDy[24] + acadoWorkspace.evGx[86]*acadoWorkspace.QDy[25] + acadoWorkspace.evGx[90]*acadoWorkspace.QDy[26] + acadoWorkspace.evGx[94]*acadoWorkspace.QDy[27] + acadoWorkspace.evGx[98]*acadoWorkspace.QDy[28] + acadoWorkspace.evGx[102]*acadoWorkspace.QDy[29] + acadoWorkspace.evGx[106]*acadoWorkspace.QDy[30] + acadoWorkspace.evGx[110]*acadoWorkspace.QDy[31] + acadoWorkspace.evGx[114]*acadoWorkspace.QDy[32] + acadoWorkspace.evGx[118]*acadoWorkspace.QDy[33] + acadoWorkspace.evGx[122]*acadoWorkspace.QDy[34] + acadoWorkspace.evGx[126]*acadoWorkspace.QDy[35] + acadoWorkspace.evGx[130]*acadoWorkspace.QDy[36] + acadoWorkspace.evGx[134]*acadoWorkspace.QDy[37] + acadoWorkspace.evGx[138]*acadoWorkspace.QDy[38] + acadoWorkspace.evGx[142]*acadoWorkspace.QDy[39] + acadoWorkspace.evGx[146]*acadoWorkspace.QDy[40] + acadoWorkspace.evGx[150]*acadoWorkspace.QDy[41] + acadoWorkspace.evGx[154]*acadoWorkspace.QDy[42] + acadoWorkspace.evGx[158]*acadoWorkspace.QDy[43] + acadoWorkspace.evGx[162]*acadoWorkspace.QDy[44] + acadoWorkspace.evGx[166]*acadoWorkspace.QDy[45] + acadoWorkspace.evGx[170]*acadoWorkspace.QDy[46] + acadoWorkspace.evGx[174]*acadoWorkspace.QDy[47] + acadoWorkspace.evGx[178]*acadoWorkspace.QDy[48] + acadoWorkspace.evGx[182]*acadoWorkspace.QDy[49] + acadoWorkspace.evGx[186]*acadoWorkspace.QDy[50] + acadoWorkspace.evGx[190]*acadoWorkspace.QDy[51] + acadoWorkspace.evGx[194]*acadoWorkspace.QDy[52] + acadoWorkspace.evGx[198]*acadoWorkspace.QDy[53] + acadoWorkspace.evGx[202]*acadoWorkspace.QDy[54] + acadoWorkspace.evGx[206]*acadoWorkspace.QDy[55] + acadoWorkspace.evGx[210]*acadoWorkspace.QDy[56] + acadoWorkspace.evGx[214]*acadoWorkspace.QDy[57] + acadoWorkspace.evGx[218]*acadoWorkspace.QDy[58] + acadoWorkspace.evGx[222]*acadoWorkspace.QDy[59] + acadoWorkspace.evGx[226]*acadoWorkspace.QDy[60] + acadoWorkspace.evGx[230]*acadoWorkspace.QDy[61] + acadoWorkspace.evGx[234]*acadoWorkspace.QDy[62] + acadoWorkspace.evGx[238]*acadoWorkspace.QDy[63] + acadoWorkspace.evGx[242]*acadoWorkspace.QDy[64] + acadoWorkspace.evGx[246]*acadoWorkspace.QDy[65] + acadoWorkspace.evGx[250]*acadoWorkspace.QDy[66] + acadoWorkspace.evGx[254]*acadoWorkspace.QDy[67] + acadoWorkspace.evGx[258]*acadoWorkspace.QDy[68] + acadoWorkspace.evGx[262]*acadoWorkspace.QDy[69] + acadoWorkspace.evGx[266]*acadoWorkspace.QDy[70] + acadoWorkspace.evGx[270]*acadoWorkspace.QDy[71] + acadoWorkspace.evGx[274]*acadoWorkspace.QDy[72] + acadoWorkspace.evGx[278]*acadoWorkspace.QDy[73] + acadoWorkspace.evGx[282]*acadoWorkspace.QDy[74] + acadoWorkspace.evGx[286]*acadoWorkspace.QDy[75] + acadoWorkspace.evGx[290]*acadoWorkspace.QDy[76] + acadoWorkspace.evGx[294]*acadoWorkspace.QDy[77] + acadoWorkspace.evGx[298]*acadoWorkspace.QDy[78] + acadoWorkspace.evGx[302]*acadoWorkspace.QDy[79] + acadoWorkspace.evGx[306]*acadoWorkspace.QDy[80] + acadoWorkspace.evGx[310]*acadoWorkspace.QDy[81] + acadoWorkspace.evGx[314]*acadoWorkspace.QDy[82] + acadoWorkspace.evGx[318]*acadoWorkspace.QDy[83] + acadoWorkspace.evGx[322]*acadoWorkspace.QDy[84] + acadoWorkspace.evGx[326]*acadoWorkspace.QDy[85] + acadoWorkspace.evGx[330]*acadoWorkspace.QDy[86] + acadoWorkspace.evGx[334]*acadoWorkspace.QDy[87] + acadoWorkspace.evGx[338]*acadoWorkspace.QDy[88] + acadoWorkspace.evGx[342]*acadoWorkspace.QDy[89] + acadoWorkspace.evGx[346]*acadoWorkspace.QDy[90] + acadoWorkspace.evGx[350]*acadoWorkspace.QDy[91] + acadoWorkspace.evGx[354]*acadoWorkspace.QDy[92] + acadoWorkspace.evGx[358]*acadoWorkspace.QDy[93] + acadoWorkspace.evGx[362]*acadoWorkspace.QDy[94] + acadoWorkspace.evGx[366]*acadoWorkspace.QDy[95] + acadoWorkspace.evGx[370]*acadoWorkspace.QDy[96] + acadoWorkspace.evGx[374]*acadoWorkspace.QDy[97] + acadoWorkspace.evGx[378]*acadoWorkspace.QDy[98] + acadoWorkspace.evGx[382]*acadoWorkspace.QDy[99] + acadoWorkspace.evGx[386]*acadoWorkspace.QDy[100] + acadoWorkspace.evGx[390]*acadoWorkspace.QDy[101] + acadoWorkspace.evGx[394]*acadoWorkspace.QDy[102] + acadoWorkspace.evGx[398]*acadoWorkspace.QDy[103];
acadoWorkspace.g[3] = + acadoWorkspace.evGx[3]*acadoWorkspace.QDy[4] + acadoWorkspace.evGx[7]*acadoWorkspace.QDy[5] + acadoWorkspace.evGx[11]*acadoWorkspace.QDy[6] + acadoWorkspace.evGx[15]*acadoWorkspace.QDy[7] + acadoWorkspace.evGx[19]*acadoWorkspace.QDy[8] + acadoWorkspace.evGx[23]*acadoWorkspace.QDy[9] + acadoWorkspace.evGx[27]*acadoWorkspace.QDy[10] + acadoWorkspace.evGx[31]*acadoWorkspace.QDy[11] + acadoWorkspace.evGx[35]*acadoWorkspace.QDy[12] + acadoWorkspace.evGx[39]*acadoWorkspace.QDy[13] + acadoWorkspace.evGx[43]*acadoWorkspace.QDy[14] + acadoWorkspace.evGx[47]*acadoWorkspace.QDy[15] + acadoWorkspace.evGx[51]*acadoWorkspace.QDy[16] + acadoWorkspace.evGx[55]*acadoWorkspace.QDy[17] + acadoWorkspace.evGx[59]*acadoWorkspace.QDy[18] + acadoWorkspace.evGx[63]*acadoWorkspace.QDy[19] + acadoWorkspace.evGx[67]*acadoWorkspace.QDy[20] + acadoWorkspace.evGx[71]*acadoWorkspace.QDy[21] + acadoWorkspace.evGx[75]*acadoWorkspace.QDy[22] + acadoWorkspace.evGx[79]*acadoWorkspace.QDy[23] + acadoWorkspace.evGx[83]*acadoWorkspace.QDy[24] + acadoWorkspace.evGx[87]*acadoWorkspace.QDy[25] + acadoWorkspace.evGx[91]*acadoWorkspace.QDy[26] + acadoWorkspace.evGx[95]*acadoWorkspace.QDy[27] + acadoWorkspace.evGx[99]*acadoWorkspace.QDy[28] + acadoWorkspace.evGx[103]*acadoWorkspace.QDy[29] + acadoWorkspace.evGx[107]*acadoWorkspace.QDy[30] + acadoWorkspace.evGx[111]*acadoWorkspace.QDy[31] + acadoWorkspace.evGx[115]*acadoWorkspace.QDy[32] + acadoWorkspace.evGx[119]*acadoWorkspace.QDy[33] + acadoWorkspace.evGx[123]*acadoWorkspace.QDy[34] + acadoWorkspace.evGx[127]*acadoWorkspace.QDy[35] + acadoWorkspace.evGx[131]*acadoWorkspace.QDy[36] + acadoWorkspace.evGx[135]*acadoWorkspace.QDy[37] + acadoWorkspace.evGx[139]*acadoWorkspace.QDy[38] + acadoWorkspace.evGx[143]*acadoWorkspace.QDy[39] + acadoWorkspace.evGx[147]*acadoWorkspace.QDy[40] + acadoWorkspace.evGx[151]*acadoWorkspace.QDy[41] + acadoWorkspace.evGx[155]*acadoWorkspace.QDy[42] + acadoWorkspace.evGx[159]*acadoWorkspace.QDy[43] + acadoWorkspace.evGx[163]*acadoWorkspace.QDy[44] + acadoWorkspace.evGx[167]*acadoWorkspace.QDy[45] + acadoWorkspace.evGx[171]*acadoWorkspace.QDy[46] + acadoWorkspace.evGx[175]*acadoWorkspace.QDy[47] + acadoWorkspace.evGx[179]*acadoWorkspace.QDy[48] + acadoWorkspace.evGx[183]*acadoWorkspace.QDy[49] + acadoWorkspace.evGx[187]*acadoWorkspace.QDy[50] + acadoWorkspace.evGx[191]*acadoWorkspace.QDy[51] + acadoWorkspace.evGx[195]*acadoWorkspace.QDy[52] + acadoWorkspace.evGx[199]*acadoWorkspace.QDy[53] + acadoWorkspace.evGx[203]*acadoWorkspace.QDy[54] + acadoWorkspace.evGx[207]*acadoWorkspace.QDy[55] + acadoWorkspace.evGx[211]*acadoWorkspace.QDy[56] + acadoWorkspace.evGx[215]*acadoWorkspace.QDy[57] + acadoWorkspace.evGx[219]*acadoWorkspace.QDy[58] + acadoWorkspace.evGx[223]*acadoWorkspace.QDy[59] + acadoWorkspace.evGx[227]*acadoWorkspace.QDy[60] + acadoWorkspace.evGx[231]*acadoWorkspace.QDy[61] + acadoWorkspace.evGx[235]*acadoWorkspace.QDy[62] + acadoWorkspace.evGx[239]*acadoWorkspace.QDy[63] + acadoWorkspace.evGx[243]*acadoWorkspace.QDy[64] + acadoWorkspace.evGx[247]*acadoWorkspace.QDy[65] + acadoWorkspace.evGx[251]*acadoWorkspace.QDy[66] + acadoWorkspace.evGx[255]*acadoWorkspace.QDy[67] + acadoWorkspace.evGx[259]*acadoWorkspace.QDy[68] + acadoWorkspace.evGx[263]*acadoWorkspace.QDy[69] + acadoWorkspace.evGx[267]*acadoWorkspace.QDy[70] + acadoWorkspace.evGx[271]*acadoWorkspace.QDy[71] + acadoWorkspace.evGx[275]*acadoWorkspace.QDy[72] + acadoWorkspace.evGx[279]*acadoWorkspace.QDy[73] + acadoWorkspace.evGx[283]*acadoWorkspace.QDy[74] + acadoWorkspace.evGx[287]*acadoWorkspace.QDy[75] + acadoWorkspace.evGx[291]*acadoWorkspace.QDy[76] + acadoWorkspace.evGx[295]*acadoWorkspace.QDy[77] + acadoWorkspace.evGx[299]*acadoWorkspace.QDy[78] + acadoWorkspace.evGx[303]*acadoWorkspace.QDy[79] + acadoWorkspace.evGx[307]*acadoWorkspace.QDy[80] + acadoWorkspace.evGx[311]*acadoWorkspace.QDy[81] + acadoWorkspace.evGx[315]*acadoWorkspace.QDy[82] + acadoWorkspace.evGx[319]*acadoWorkspace.QDy[83] + acadoWorkspace.evGx[323]*acadoWorkspace.QDy[84] + acadoWorkspace.evGx[327]*acadoWorkspace.QDy[85] + acadoWorkspace.evGx[331]*acadoWorkspace.QDy[86] + acadoWorkspace.evGx[335]*acadoWorkspace.QDy[87] + acadoWorkspace.evGx[339]*acadoWorkspace.QDy[88] + acadoWorkspace.evGx[343]*acadoWorkspace.QDy[89] + acadoWorkspace.evGx[347]*acadoWorkspace.QDy[90] + acadoWorkspace.evGx[351]*acadoWorkspace.QDy[91] + acadoWorkspace.evGx[355]*acadoWorkspace.QDy[92] + acadoWorkspace.evGx[359]*acadoWorkspace.QDy[93] + acadoWorkspace.evGx[363]*acadoWorkspace.QDy[94] + acadoWorkspace.evGx[367]*acadoWorkspace.QDy[95] + acadoWorkspace.evGx[371]*acadoWorkspace.QDy[96] + acadoWorkspace.evGx[375]*acadoWorkspace.QDy[97] + acadoWorkspace.evGx[379]*acadoWorkspace.QDy[98] + acadoWorkspace.evGx[383]*acadoWorkspace.QDy[99] + acadoWorkspace.evGx[387]*acadoWorkspace.QDy[100] + acadoWorkspace.evGx[391]*acadoWorkspace.QDy[101] + acadoWorkspace.evGx[395]*acadoWorkspace.QDy[102] + acadoWorkspace.evGx[399]*acadoWorkspace.QDy[103];


for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 25; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_multEQDy( &(acadoWorkspace.E[ lRun3 * 4 ]), &(acadoWorkspace.QDy[ lRun2 * 4 + 4 ]), &(acadoWorkspace.g[ lRun1 + 4 ]) );
}
}

acadoWorkspace.lb[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.lb[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.lb[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.lb[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.ub[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.ub[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.ub[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.ub[3] = acadoWorkspace.Dx0[3];
tmp = acadoVariables.x[6] + acadoWorkspace.d[2];
acadoWorkspace.lbA[0] = (real_t)-1.5707963268000000e+00 - tmp;
acadoWorkspace.ubA[0] = (real_t)1.5707963268000000e+00 - tmp;
tmp = acadoVariables.x[10] + acadoWorkspace.d[6];
acadoWorkspace.lbA[1] = (real_t)-1.5707963268000000e+00 - tmp;
acadoWorkspace.ubA[1] = (real_t)1.5707963268000000e+00 - tmp;
tmp = acadoVariables.x[14] + acadoWorkspace.d[10];
acadoWorkspace.lbA[2] = (real_t)-1.5707963268000000e+00 - tmp;
acadoWorkspace.ubA[2] = (real_t)1.5707963268000000e+00 - tmp;
tmp = acadoVariables.x[18] + acadoWorkspace.d[14];
acadoWorkspace.lbA[3] = (real_t)-1.5707963268000000e+00 - tmp;
acadoWorkspace.ubA[3] = (real_t)1.5707963268000000e+00 - tmp;
tmp = acadoVariables.x[22] + acadoWorkspace.d[18];
acadoWorkspace.lbA[4] = (real_t)-1.5707963268000000e+00 - tmp;
acadoWorkspace.ubA[4] = (real_t)1.5707963268000000e+00 - tmp;
tmp = acadoVariables.x[26] + acadoWorkspace.d[22];
acadoWorkspace.lbA[5] = (real_t)-1.5707963268000000e+00 - tmp;
acadoWorkspace.ubA[5] = (real_t)1.5707963268000000e+00 - tmp;
tmp = acadoVariables.x[30] + acadoWorkspace.d[26];
acadoWorkspace.lbA[6] = (real_t)-1.5707963268000000e+00 - tmp;
acadoWorkspace.ubA[6] = (real_t)1.5707963268000000e+00 - tmp;
tmp = acadoVariables.x[34] + acadoWorkspace.d[30];
acadoWorkspace.lbA[7] = (real_t)-1.5707963268000000e+00 - tmp;
acadoWorkspace.ubA[7] = (real_t)1.5707963268000000e+00 - tmp;
tmp = acadoVariables.x[38] + acadoWorkspace.d[34];
acadoWorkspace.lbA[8] = (real_t)-1.5707963268000000e+00 - tmp;
acadoWorkspace.ubA[8] = (real_t)1.5707963268000000e+00 - tmp;
tmp = acadoVariables.x[42] + acadoWorkspace.d[38];
acadoWorkspace.lbA[9] = (real_t)-1.5707963268000000e+00 - tmp;
acadoWorkspace.ubA[9] = (real_t)1.5707963268000000e+00 - tmp;
tmp = acadoVariables.x[46] + acadoWorkspace.d[42];
acadoWorkspace.lbA[10] = (real_t)-1.5707963268000000e+00 - tmp;
acadoWorkspace.ubA[10] = (real_t)1.5707963268000000e+00 - tmp;
tmp = acadoVariables.x[50] + acadoWorkspace.d[46];
acadoWorkspace.lbA[11] = (real_t)-1.5707963268000000e+00 - tmp;
acadoWorkspace.ubA[11] = (real_t)1.5707963268000000e+00 - tmp;
tmp = acadoVariables.x[54] + acadoWorkspace.d[50];
acadoWorkspace.lbA[12] = (real_t)-1.5707963268000000e+00 - tmp;
acadoWorkspace.ubA[12] = (real_t)1.5707963268000000e+00 - tmp;
tmp = acadoVariables.x[58] + acadoWorkspace.d[54];
acadoWorkspace.lbA[13] = (real_t)-1.5707963268000000e+00 - tmp;
acadoWorkspace.ubA[13] = (real_t)1.5707963268000000e+00 - tmp;
tmp = acadoVariables.x[62] + acadoWorkspace.d[58];
acadoWorkspace.lbA[14] = (real_t)-1.5707963268000000e+00 - tmp;
acadoWorkspace.ubA[14] = (real_t)1.5707963268000000e+00 - tmp;
tmp = acadoVariables.x[66] + acadoWorkspace.d[62];
acadoWorkspace.lbA[15] = (real_t)-1.5707963268000000e+00 - tmp;
acadoWorkspace.ubA[15] = (real_t)1.5707963268000000e+00 - tmp;
tmp = acadoVariables.x[70] + acadoWorkspace.d[66];
acadoWorkspace.lbA[16] = (real_t)-1.5707963268000000e+00 - tmp;
acadoWorkspace.ubA[16] = (real_t)1.5707963268000000e+00 - tmp;
tmp = acadoVariables.x[74] + acadoWorkspace.d[70];
acadoWorkspace.lbA[17] = (real_t)-1.5707963268000000e+00 - tmp;
acadoWorkspace.ubA[17] = (real_t)1.5707963268000000e+00 - tmp;
tmp = acadoVariables.x[78] + acadoWorkspace.d[74];
acadoWorkspace.lbA[18] = (real_t)-1.5707963268000000e+00 - tmp;
acadoWorkspace.ubA[18] = (real_t)1.5707963268000000e+00 - tmp;
tmp = acadoVariables.x[82] + acadoWorkspace.d[78];
acadoWorkspace.lbA[19] = (real_t)-1.5707963268000000e+00 - tmp;
acadoWorkspace.ubA[19] = (real_t)1.5707963268000000e+00 - tmp;
tmp = acadoVariables.x[86] + acadoWorkspace.d[82];
acadoWorkspace.lbA[20] = (real_t)-1.5707963268000000e+00 - tmp;
acadoWorkspace.ubA[20] = (real_t)1.5707963268000000e+00 - tmp;
tmp = acadoVariables.x[90] + acadoWorkspace.d[86];
acadoWorkspace.lbA[21] = (real_t)-1.5707963268000000e+00 - tmp;
acadoWorkspace.ubA[21] = (real_t)1.5707963268000000e+00 - tmp;
tmp = acadoVariables.x[94] + acadoWorkspace.d[90];
acadoWorkspace.lbA[22] = (real_t)-1.5707963268000000e+00 - tmp;
acadoWorkspace.ubA[22] = (real_t)1.5707963268000000e+00 - tmp;
tmp = acadoVariables.x[98] + acadoWorkspace.d[94];
acadoWorkspace.lbA[23] = (real_t)-1.5707963268000000e+00 - tmp;
acadoWorkspace.ubA[23] = (real_t)1.5707963268000000e+00 - tmp;
tmp = acadoVariables.x[102] + acadoWorkspace.d[98];
acadoWorkspace.lbA[24] = (real_t)-1.5707963268000000e+00 - tmp;
acadoWorkspace.ubA[24] = (real_t)1.5707963268000000e+00 - tmp;

}

void acado_expand(  )
{
int lRun1;
int lRun2;
int lRun3;
acadoVariables.x[0] += acadoWorkspace.x[0];
acadoVariables.x[1] += acadoWorkspace.x[1];
acadoVariables.x[2] += acadoWorkspace.x[2];
acadoVariables.x[3] += acadoWorkspace.x[3];

acadoVariables.u[0] += acadoWorkspace.x[4];
acadoVariables.u[1] += acadoWorkspace.x[5];
acadoVariables.u[2] += acadoWorkspace.x[6];
acadoVariables.u[3] += acadoWorkspace.x[7];
acadoVariables.u[4] += acadoWorkspace.x[8];
acadoVariables.u[5] += acadoWorkspace.x[9];
acadoVariables.u[6] += acadoWorkspace.x[10];
acadoVariables.u[7] += acadoWorkspace.x[11];
acadoVariables.u[8] += acadoWorkspace.x[12];
acadoVariables.u[9] += acadoWorkspace.x[13];
acadoVariables.u[10] += acadoWorkspace.x[14];
acadoVariables.u[11] += acadoWorkspace.x[15];
acadoVariables.u[12] += acadoWorkspace.x[16];
acadoVariables.u[13] += acadoWorkspace.x[17];
acadoVariables.u[14] += acadoWorkspace.x[18];
acadoVariables.u[15] += acadoWorkspace.x[19];
acadoVariables.u[16] += acadoWorkspace.x[20];
acadoVariables.u[17] += acadoWorkspace.x[21];
acadoVariables.u[18] += acadoWorkspace.x[22];
acadoVariables.u[19] += acadoWorkspace.x[23];
acadoVariables.u[20] += acadoWorkspace.x[24];
acadoVariables.u[21] += acadoWorkspace.x[25];
acadoVariables.u[22] += acadoWorkspace.x[26];
acadoVariables.u[23] += acadoWorkspace.x[27];
acadoVariables.u[24] += acadoWorkspace.x[28];

acadoVariables.x[4] += + acadoWorkspace.evGx[0]*acadoWorkspace.x[0] + acadoWorkspace.evGx[1]*acadoWorkspace.x[1] + acadoWorkspace.evGx[2]*acadoWorkspace.x[2] + acadoWorkspace.evGx[3]*acadoWorkspace.x[3] + acadoWorkspace.d[0];
acadoVariables.x[5] += + acadoWorkspace.evGx[4]*acadoWorkspace.x[0] + acadoWorkspace.evGx[5]*acadoWorkspace.x[1] + acadoWorkspace.evGx[6]*acadoWorkspace.x[2] + acadoWorkspace.evGx[7]*acadoWorkspace.x[3] + acadoWorkspace.d[1];
acadoVariables.x[6] += + acadoWorkspace.evGx[8]*acadoWorkspace.x[0] + acadoWorkspace.evGx[9]*acadoWorkspace.x[1] + acadoWorkspace.evGx[10]*acadoWorkspace.x[2] + acadoWorkspace.evGx[11]*acadoWorkspace.x[3] + acadoWorkspace.d[2];
acadoVariables.x[7] += + acadoWorkspace.evGx[12]*acadoWorkspace.x[0] + acadoWorkspace.evGx[13]*acadoWorkspace.x[1] + acadoWorkspace.evGx[14]*acadoWorkspace.x[2] + acadoWorkspace.evGx[15]*acadoWorkspace.x[3] + acadoWorkspace.d[3];
acadoVariables.x[8] += + acadoWorkspace.evGx[16]*acadoWorkspace.x[0] + acadoWorkspace.evGx[17]*acadoWorkspace.x[1] + acadoWorkspace.evGx[18]*acadoWorkspace.x[2] + acadoWorkspace.evGx[19]*acadoWorkspace.x[3] + acadoWorkspace.d[4];
acadoVariables.x[9] += + acadoWorkspace.evGx[20]*acadoWorkspace.x[0] + acadoWorkspace.evGx[21]*acadoWorkspace.x[1] + acadoWorkspace.evGx[22]*acadoWorkspace.x[2] + acadoWorkspace.evGx[23]*acadoWorkspace.x[3] + acadoWorkspace.d[5];
acadoVariables.x[10] += + acadoWorkspace.evGx[24]*acadoWorkspace.x[0] + acadoWorkspace.evGx[25]*acadoWorkspace.x[1] + acadoWorkspace.evGx[26]*acadoWorkspace.x[2] + acadoWorkspace.evGx[27]*acadoWorkspace.x[3] + acadoWorkspace.d[6];
acadoVariables.x[11] += + acadoWorkspace.evGx[28]*acadoWorkspace.x[0] + acadoWorkspace.evGx[29]*acadoWorkspace.x[1] + acadoWorkspace.evGx[30]*acadoWorkspace.x[2] + acadoWorkspace.evGx[31]*acadoWorkspace.x[3] + acadoWorkspace.d[7];
acadoVariables.x[12] += + acadoWorkspace.evGx[32]*acadoWorkspace.x[0] + acadoWorkspace.evGx[33]*acadoWorkspace.x[1] + acadoWorkspace.evGx[34]*acadoWorkspace.x[2] + acadoWorkspace.evGx[35]*acadoWorkspace.x[3] + acadoWorkspace.d[8];
acadoVariables.x[13] += + acadoWorkspace.evGx[36]*acadoWorkspace.x[0] + acadoWorkspace.evGx[37]*acadoWorkspace.x[1] + acadoWorkspace.evGx[38]*acadoWorkspace.x[2] + acadoWorkspace.evGx[39]*acadoWorkspace.x[3] + acadoWorkspace.d[9];
acadoVariables.x[14] += + acadoWorkspace.evGx[40]*acadoWorkspace.x[0] + acadoWorkspace.evGx[41]*acadoWorkspace.x[1] + acadoWorkspace.evGx[42]*acadoWorkspace.x[2] + acadoWorkspace.evGx[43]*acadoWorkspace.x[3] + acadoWorkspace.d[10];
acadoVariables.x[15] += + acadoWorkspace.evGx[44]*acadoWorkspace.x[0] + acadoWorkspace.evGx[45]*acadoWorkspace.x[1] + acadoWorkspace.evGx[46]*acadoWorkspace.x[2] + acadoWorkspace.evGx[47]*acadoWorkspace.x[3] + acadoWorkspace.d[11];
acadoVariables.x[16] += + acadoWorkspace.evGx[48]*acadoWorkspace.x[0] + acadoWorkspace.evGx[49]*acadoWorkspace.x[1] + acadoWorkspace.evGx[50]*acadoWorkspace.x[2] + acadoWorkspace.evGx[51]*acadoWorkspace.x[3] + acadoWorkspace.d[12];
acadoVariables.x[17] += + acadoWorkspace.evGx[52]*acadoWorkspace.x[0] + acadoWorkspace.evGx[53]*acadoWorkspace.x[1] + acadoWorkspace.evGx[54]*acadoWorkspace.x[2] + acadoWorkspace.evGx[55]*acadoWorkspace.x[3] + acadoWorkspace.d[13];
acadoVariables.x[18] += + acadoWorkspace.evGx[56]*acadoWorkspace.x[0] + acadoWorkspace.evGx[57]*acadoWorkspace.x[1] + acadoWorkspace.evGx[58]*acadoWorkspace.x[2] + acadoWorkspace.evGx[59]*acadoWorkspace.x[3] + acadoWorkspace.d[14];
acadoVariables.x[19] += + acadoWorkspace.evGx[60]*acadoWorkspace.x[0] + acadoWorkspace.evGx[61]*acadoWorkspace.x[1] + acadoWorkspace.evGx[62]*acadoWorkspace.x[2] + acadoWorkspace.evGx[63]*acadoWorkspace.x[3] + acadoWorkspace.d[15];
acadoVariables.x[20] += + acadoWorkspace.evGx[64]*acadoWorkspace.x[0] + acadoWorkspace.evGx[65]*acadoWorkspace.x[1] + acadoWorkspace.evGx[66]*acadoWorkspace.x[2] + acadoWorkspace.evGx[67]*acadoWorkspace.x[3] + acadoWorkspace.d[16];
acadoVariables.x[21] += + acadoWorkspace.evGx[68]*acadoWorkspace.x[0] + acadoWorkspace.evGx[69]*acadoWorkspace.x[1] + acadoWorkspace.evGx[70]*acadoWorkspace.x[2] + acadoWorkspace.evGx[71]*acadoWorkspace.x[3] + acadoWorkspace.d[17];
acadoVariables.x[22] += + acadoWorkspace.evGx[72]*acadoWorkspace.x[0] + acadoWorkspace.evGx[73]*acadoWorkspace.x[1] + acadoWorkspace.evGx[74]*acadoWorkspace.x[2] + acadoWorkspace.evGx[75]*acadoWorkspace.x[3] + acadoWorkspace.d[18];
acadoVariables.x[23] += + acadoWorkspace.evGx[76]*acadoWorkspace.x[0] + acadoWorkspace.evGx[77]*acadoWorkspace.x[1] + acadoWorkspace.evGx[78]*acadoWorkspace.x[2] + acadoWorkspace.evGx[79]*acadoWorkspace.x[3] + acadoWorkspace.d[19];
acadoVariables.x[24] += + acadoWorkspace.evGx[80]*acadoWorkspace.x[0] + acadoWorkspace.evGx[81]*acadoWorkspace.x[1] + acadoWorkspace.evGx[82]*acadoWorkspace.x[2] + acadoWorkspace.evGx[83]*acadoWorkspace.x[3] + acadoWorkspace.d[20];
acadoVariables.x[25] += + acadoWorkspace.evGx[84]*acadoWorkspace.x[0] + acadoWorkspace.evGx[85]*acadoWorkspace.x[1] + acadoWorkspace.evGx[86]*acadoWorkspace.x[2] + acadoWorkspace.evGx[87]*acadoWorkspace.x[3] + acadoWorkspace.d[21];
acadoVariables.x[26] += + acadoWorkspace.evGx[88]*acadoWorkspace.x[0] + acadoWorkspace.evGx[89]*acadoWorkspace.x[1] + acadoWorkspace.evGx[90]*acadoWorkspace.x[2] + acadoWorkspace.evGx[91]*acadoWorkspace.x[3] + acadoWorkspace.d[22];
acadoVariables.x[27] += + acadoWorkspace.evGx[92]*acadoWorkspace.x[0] + acadoWorkspace.evGx[93]*acadoWorkspace.x[1] + acadoWorkspace.evGx[94]*acadoWorkspace.x[2] + acadoWorkspace.evGx[95]*acadoWorkspace.x[3] + acadoWorkspace.d[23];
acadoVariables.x[28] += + acadoWorkspace.evGx[96]*acadoWorkspace.x[0] + acadoWorkspace.evGx[97]*acadoWorkspace.x[1] + acadoWorkspace.evGx[98]*acadoWorkspace.x[2] + acadoWorkspace.evGx[99]*acadoWorkspace.x[3] + acadoWorkspace.d[24];
acadoVariables.x[29] += + acadoWorkspace.evGx[100]*acadoWorkspace.x[0] + acadoWorkspace.evGx[101]*acadoWorkspace.x[1] + acadoWorkspace.evGx[102]*acadoWorkspace.x[2] + acadoWorkspace.evGx[103]*acadoWorkspace.x[3] + acadoWorkspace.d[25];
acadoVariables.x[30] += + acadoWorkspace.evGx[104]*acadoWorkspace.x[0] + acadoWorkspace.evGx[105]*acadoWorkspace.x[1] + acadoWorkspace.evGx[106]*acadoWorkspace.x[2] + acadoWorkspace.evGx[107]*acadoWorkspace.x[3] + acadoWorkspace.d[26];
acadoVariables.x[31] += + acadoWorkspace.evGx[108]*acadoWorkspace.x[0] + acadoWorkspace.evGx[109]*acadoWorkspace.x[1] + acadoWorkspace.evGx[110]*acadoWorkspace.x[2] + acadoWorkspace.evGx[111]*acadoWorkspace.x[3] + acadoWorkspace.d[27];
acadoVariables.x[32] += + acadoWorkspace.evGx[112]*acadoWorkspace.x[0] + acadoWorkspace.evGx[113]*acadoWorkspace.x[1] + acadoWorkspace.evGx[114]*acadoWorkspace.x[2] + acadoWorkspace.evGx[115]*acadoWorkspace.x[3] + acadoWorkspace.d[28];
acadoVariables.x[33] += + acadoWorkspace.evGx[116]*acadoWorkspace.x[0] + acadoWorkspace.evGx[117]*acadoWorkspace.x[1] + acadoWorkspace.evGx[118]*acadoWorkspace.x[2] + acadoWorkspace.evGx[119]*acadoWorkspace.x[3] + acadoWorkspace.d[29];
acadoVariables.x[34] += + acadoWorkspace.evGx[120]*acadoWorkspace.x[0] + acadoWorkspace.evGx[121]*acadoWorkspace.x[1] + acadoWorkspace.evGx[122]*acadoWorkspace.x[2] + acadoWorkspace.evGx[123]*acadoWorkspace.x[3] + acadoWorkspace.d[30];
acadoVariables.x[35] += + acadoWorkspace.evGx[124]*acadoWorkspace.x[0] + acadoWorkspace.evGx[125]*acadoWorkspace.x[1] + acadoWorkspace.evGx[126]*acadoWorkspace.x[2] + acadoWorkspace.evGx[127]*acadoWorkspace.x[3] + acadoWorkspace.d[31];
acadoVariables.x[36] += + acadoWorkspace.evGx[128]*acadoWorkspace.x[0] + acadoWorkspace.evGx[129]*acadoWorkspace.x[1] + acadoWorkspace.evGx[130]*acadoWorkspace.x[2] + acadoWorkspace.evGx[131]*acadoWorkspace.x[3] + acadoWorkspace.d[32];
acadoVariables.x[37] += + acadoWorkspace.evGx[132]*acadoWorkspace.x[0] + acadoWorkspace.evGx[133]*acadoWorkspace.x[1] + acadoWorkspace.evGx[134]*acadoWorkspace.x[2] + acadoWorkspace.evGx[135]*acadoWorkspace.x[3] + acadoWorkspace.d[33];
acadoVariables.x[38] += + acadoWorkspace.evGx[136]*acadoWorkspace.x[0] + acadoWorkspace.evGx[137]*acadoWorkspace.x[1] + acadoWorkspace.evGx[138]*acadoWorkspace.x[2] + acadoWorkspace.evGx[139]*acadoWorkspace.x[3] + acadoWorkspace.d[34];
acadoVariables.x[39] += + acadoWorkspace.evGx[140]*acadoWorkspace.x[0] + acadoWorkspace.evGx[141]*acadoWorkspace.x[1] + acadoWorkspace.evGx[142]*acadoWorkspace.x[2] + acadoWorkspace.evGx[143]*acadoWorkspace.x[3] + acadoWorkspace.d[35];
acadoVariables.x[40] += + acadoWorkspace.evGx[144]*acadoWorkspace.x[0] + acadoWorkspace.evGx[145]*acadoWorkspace.x[1] + acadoWorkspace.evGx[146]*acadoWorkspace.x[2] + acadoWorkspace.evGx[147]*acadoWorkspace.x[3] + acadoWorkspace.d[36];
acadoVariables.x[41] += + acadoWorkspace.evGx[148]*acadoWorkspace.x[0] + acadoWorkspace.evGx[149]*acadoWorkspace.x[1] + acadoWorkspace.evGx[150]*acadoWorkspace.x[2] + acadoWorkspace.evGx[151]*acadoWorkspace.x[3] + acadoWorkspace.d[37];
acadoVariables.x[42] += + acadoWorkspace.evGx[152]*acadoWorkspace.x[0] + acadoWorkspace.evGx[153]*acadoWorkspace.x[1] + acadoWorkspace.evGx[154]*acadoWorkspace.x[2] + acadoWorkspace.evGx[155]*acadoWorkspace.x[3] + acadoWorkspace.d[38];
acadoVariables.x[43] += + acadoWorkspace.evGx[156]*acadoWorkspace.x[0] + acadoWorkspace.evGx[157]*acadoWorkspace.x[1] + acadoWorkspace.evGx[158]*acadoWorkspace.x[2] + acadoWorkspace.evGx[159]*acadoWorkspace.x[3] + acadoWorkspace.d[39];
acadoVariables.x[44] += + acadoWorkspace.evGx[160]*acadoWorkspace.x[0] + acadoWorkspace.evGx[161]*acadoWorkspace.x[1] + acadoWorkspace.evGx[162]*acadoWorkspace.x[2] + acadoWorkspace.evGx[163]*acadoWorkspace.x[3] + acadoWorkspace.d[40];
acadoVariables.x[45] += + acadoWorkspace.evGx[164]*acadoWorkspace.x[0] + acadoWorkspace.evGx[165]*acadoWorkspace.x[1] + acadoWorkspace.evGx[166]*acadoWorkspace.x[2] + acadoWorkspace.evGx[167]*acadoWorkspace.x[3] + acadoWorkspace.d[41];
acadoVariables.x[46] += + acadoWorkspace.evGx[168]*acadoWorkspace.x[0] + acadoWorkspace.evGx[169]*acadoWorkspace.x[1] + acadoWorkspace.evGx[170]*acadoWorkspace.x[2] + acadoWorkspace.evGx[171]*acadoWorkspace.x[3] + acadoWorkspace.d[42];
acadoVariables.x[47] += + acadoWorkspace.evGx[172]*acadoWorkspace.x[0] + acadoWorkspace.evGx[173]*acadoWorkspace.x[1] + acadoWorkspace.evGx[174]*acadoWorkspace.x[2] + acadoWorkspace.evGx[175]*acadoWorkspace.x[3] + acadoWorkspace.d[43];
acadoVariables.x[48] += + acadoWorkspace.evGx[176]*acadoWorkspace.x[0] + acadoWorkspace.evGx[177]*acadoWorkspace.x[1] + acadoWorkspace.evGx[178]*acadoWorkspace.x[2] + acadoWorkspace.evGx[179]*acadoWorkspace.x[3] + acadoWorkspace.d[44];
acadoVariables.x[49] += + acadoWorkspace.evGx[180]*acadoWorkspace.x[0] + acadoWorkspace.evGx[181]*acadoWorkspace.x[1] + acadoWorkspace.evGx[182]*acadoWorkspace.x[2] + acadoWorkspace.evGx[183]*acadoWorkspace.x[3] + acadoWorkspace.d[45];
acadoVariables.x[50] += + acadoWorkspace.evGx[184]*acadoWorkspace.x[0] + acadoWorkspace.evGx[185]*acadoWorkspace.x[1] + acadoWorkspace.evGx[186]*acadoWorkspace.x[2] + acadoWorkspace.evGx[187]*acadoWorkspace.x[3] + acadoWorkspace.d[46];
acadoVariables.x[51] += + acadoWorkspace.evGx[188]*acadoWorkspace.x[0] + acadoWorkspace.evGx[189]*acadoWorkspace.x[1] + acadoWorkspace.evGx[190]*acadoWorkspace.x[2] + acadoWorkspace.evGx[191]*acadoWorkspace.x[3] + acadoWorkspace.d[47];
acadoVariables.x[52] += + acadoWorkspace.evGx[192]*acadoWorkspace.x[0] + acadoWorkspace.evGx[193]*acadoWorkspace.x[1] + acadoWorkspace.evGx[194]*acadoWorkspace.x[2] + acadoWorkspace.evGx[195]*acadoWorkspace.x[3] + acadoWorkspace.d[48];
acadoVariables.x[53] += + acadoWorkspace.evGx[196]*acadoWorkspace.x[0] + acadoWorkspace.evGx[197]*acadoWorkspace.x[1] + acadoWorkspace.evGx[198]*acadoWorkspace.x[2] + acadoWorkspace.evGx[199]*acadoWorkspace.x[3] + acadoWorkspace.d[49];
acadoVariables.x[54] += + acadoWorkspace.evGx[200]*acadoWorkspace.x[0] + acadoWorkspace.evGx[201]*acadoWorkspace.x[1] + acadoWorkspace.evGx[202]*acadoWorkspace.x[2] + acadoWorkspace.evGx[203]*acadoWorkspace.x[3] + acadoWorkspace.d[50];
acadoVariables.x[55] += + acadoWorkspace.evGx[204]*acadoWorkspace.x[0] + acadoWorkspace.evGx[205]*acadoWorkspace.x[1] + acadoWorkspace.evGx[206]*acadoWorkspace.x[2] + acadoWorkspace.evGx[207]*acadoWorkspace.x[3] + acadoWorkspace.d[51];
acadoVariables.x[56] += + acadoWorkspace.evGx[208]*acadoWorkspace.x[0] + acadoWorkspace.evGx[209]*acadoWorkspace.x[1] + acadoWorkspace.evGx[210]*acadoWorkspace.x[2] + acadoWorkspace.evGx[211]*acadoWorkspace.x[3] + acadoWorkspace.d[52];
acadoVariables.x[57] += + acadoWorkspace.evGx[212]*acadoWorkspace.x[0] + acadoWorkspace.evGx[213]*acadoWorkspace.x[1] + acadoWorkspace.evGx[214]*acadoWorkspace.x[2] + acadoWorkspace.evGx[215]*acadoWorkspace.x[3] + acadoWorkspace.d[53];
acadoVariables.x[58] += + acadoWorkspace.evGx[216]*acadoWorkspace.x[0] + acadoWorkspace.evGx[217]*acadoWorkspace.x[1] + acadoWorkspace.evGx[218]*acadoWorkspace.x[2] + acadoWorkspace.evGx[219]*acadoWorkspace.x[3] + acadoWorkspace.d[54];
acadoVariables.x[59] += + acadoWorkspace.evGx[220]*acadoWorkspace.x[0] + acadoWorkspace.evGx[221]*acadoWorkspace.x[1] + acadoWorkspace.evGx[222]*acadoWorkspace.x[2] + acadoWorkspace.evGx[223]*acadoWorkspace.x[3] + acadoWorkspace.d[55];
acadoVariables.x[60] += + acadoWorkspace.evGx[224]*acadoWorkspace.x[0] + acadoWorkspace.evGx[225]*acadoWorkspace.x[1] + acadoWorkspace.evGx[226]*acadoWorkspace.x[2] + acadoWorkspace.evGx[227]*acadoWorkspace.x[3] + acadoWorkspace.d[56];
acadoVariables.x[61] += + acadoWorkspace.evGx[228]*acadoWorkspace.x[0] + acadoWorkspace.evGx[229]*acadoWorkspace.x[1] + acadoWorkspace.evGx[230]*acadoWorkspace.x[2] + acadoWorkspace.evGx[231]*acadoWorkspace.x[3] + acadoWorkspace.d[57];
acadoVariables.x[62] += + acadoWorkspace.evGx[232]*acadoWorkspace.x[0] + acadoWorkspace.evGx[233]*acadoWorkspace.x[1] + acadoWorkspace.evGx[234]*acadoWorkspace.x[2] + acadoWorkspace.evGx[235]*acadoWorkspace.x[3] + acadoWorkspace.d[58];
acadoVariables.x[63] += + acadoWorkspace.evGx[236]*acadoWorkspace.x[0] + acadoWorkspace.evGx[237]*acadoWorkspace.x[1] + acadoWorkspace.evGx[238]*acadoWorkspace.x[2] + acadoWorkspace.evGx[239]*acadoWorkspace.x[3] + acadoWorkspace.d[59];
acadoVariables.x[64] += + acadoWorkspace.evGx[240]*acadoWorkspace.x[0] + acadoWorkspace.evGx[241]*acadoWorkspace.x[1] + acadoWorkspace.evGx[242]*acadoWorkspace.x[2] + acadoWorkspace.evGx[243]*acadoWorkspace.x[3] + acadoWorkspace.d[60];
acadoVariables.x[65] += + acadoWorkspace.evGx[244]*acadoWorkspace.x[0] + acadoWorkspace.evGx[245]*acadoWorkspace.x[1] + acadoWorkspace.evGx[246]*acadoWorkspace.x[2] + acadoWorkspace.evGx[247]*acadoWorkspace.x[3] + acadoWorkspace.d[61];
acadoVariables.x[66] += + acadoWorkspace.evGx[248]*acadoWorkspace.x[0] + acadoWorkspace.evGx[249]*acadoWorkspace.x[1] + acadoWorkspace.evGx[250]*acadoWorkspace.x[2] + acadoWorkspace.evGx[251]*acadoWorkspace.x[3] + acadoWorkspace.d[62];
acadoVariables.x[67] += + acadoWorkspace.evGx[252]*acadoWorkspace.x[0] + acadoWorkspace.evGx[253]*acadoWorkspace.x[1] + acadoWorkspace.evGx[254]*acadoWorkspace.x[2] + acadoWorkspace.evGx[255]*acadoWorkspace.x[3] + acadoWorkspace.d[63];
acadoVariables.x[68] += + acadoWorkspace.evGx[256]*acadoWorkspace.x[0] + acadoWorkspace.evGx[257]*acadoWorkspace.x[1] + acadoWorkspace.evGx[258]*acadoWorkspace.x[2] + acadoWorkspace.evGx[259]*acadoWorkspace.x[3] + acadoWorkspace.d[64];
acadoVariables.x[69] += + acadoWorkspace.evGx[260]*acadoWorkspace.x[0] + acadoWorkspace.evGx[261]*acadoWorkspace.x[1] + acadoWorkspace.evGx[262]*acadoWorkspace.x[2] + acadoWorkspace.evGx[263]*acadoWorkspace.x[3] + acadoWorkspace.d[65];
acadoVariables.x[70] += + acadoWorkspace.evGx[264]*acadoWorkspace.x[0] + acadoWorkspace.evGx[265]*acadoWorkspace.x[1] + acadoWorkspace.evGx[266]*acadoWorkspace.x[2] + acadoWorkspace.evGx[267]*acadoWorkspace.x[3] + acadoWorkspace.d[66];
acadoVariables.x[71] += + acadoWorkspace.evGx[268]*acadoWorkspace.x[0] + acadoWorkspace.evGx[269]*acadoWorkspace.x[1] + acadoWorkspace.evGx[270]*acadoWorkspace.x[2] + acadoWorkspace.evGx[271]*acadoWorkspace.x[3] + acadoWorkspace.d[67];
acadoVariables.x[72] += + acadoWorkspace.evGx[272]*acadoWorkspace.x[0] + acadoWorkspace.evGx[273]*acadoWorkspace.x[1] + acadoWorkspace.evGx[274]*acadoWorkspace.x[2] + acadoWorkspace.evGx[275]*acadoWorkspace.x[3] + acadoWorkspace.d[68];
acadoVariables.x[73] += + acadoWorkspace.evGx[276]*acadoWorkspace.x[0] + acadoWorkspace.evGx[277]*acadoWorkspace.x[1] + acadoWorkspace.evGx[278]*acadoWorkspace.x[2] + acadoWorkspace.evGx[279]*acadoWorkspace.x[3] + acadoWorkspace.d[69];
acadoVariables.x[74] += + acadoWorkspace.evGx[280]*acadoWorkspace.x[0] + acadoWorkspace.evGx[281]*acadoWorkspace.x[1] + acadoWorkspace.evGx[282]*acadoWorkspace.x[2] + acadoWorkspace.evGx[283]*acadoWorkspace.x[3] + acadoWorkspace.d[70];
acadoVariables.x[75] += + acadoWorkspace.evGx[284]*acadoWorkspace.x[0] + acadoWorkspace.evGx[285]*acadoWorkspace.x[1] + acadoWorkspace.evGx[286]*acadoWorkspace.x[2] + acadoWorkspace.evGx[287]*acadoWorkspace.x[3] + acadoWorkspace.d[71];
acadoVariables.x[76] += + acadoWorkspace.evGx[288]*acadoWorkspace.x[0] + acadoWorkspace.evGx[289]*acadoWorkspace.x[1] + acadoWorkspace.evGx[290]*acadoWorkspace.x[2] + acadoWorkspace.evGx[291]*acadoWorkspace.x[3] + acadoWorkspace.d[72];
acadoVariables.x[77] += + acadoWorkspace.evGx[292]*acadoWorkspace.x[0] + acadoWorkspace.evGx[293]*acadoWorkspace.x[1] + acadoWorkspace.evGx[294]*acadoWorkspace.x[2] + acadoWorkspace.evGx[295]*acadoWorkspace.x[3] + acadoWorkspace.d[73];
acadoVariables.x[78] += + acadoWorkspace.evGx[296]*acadoWorkspace.x[0] + acadoWorkspace.evGx[297]*acadoWorkspace.x[1] + acadoWorkspace.evGx[298]*acadoWorkspace.x[2] + acadoWorkspace.evGx[299]*acadoWorkspace.x[3] + acadoWorkspace.d[74];
acadoVariables.x[79] += + acadoWorkspace.evGx[300]*acadoWorkspace.x[0] + acadoWorkspace.evGx[301]*acadoWorkspace.x[1] + acadoWorkspace.evGx[302]*acadoWorkspace.x[2] + acadoWorkspace.evGx[303]*acadoWorkspace.x[3] + acadoWorkspace.d[75];
acadoVariables.x[80] += + acadoWorkspace.evGx[304]*acadoWorkspace.x[0] + acadoWorkspace.evGx[305]*acadoWorkspace.x[1] + acadoWorkspace.evGx[306]*acadoWorkspace.x[2] + acadoWorkspace.evGx[307]*acadoWorkspace.x[3] + acadoWorkspace.d[76];
acadoVariables.x[81] += + acadoWorkspace.evGx[308]*acadoWorkspace.x[0] + acadoWorkspace.evGx[309]*acadoWorkspace.x[1] + acadoWorkspace.evGx[310]*acadoWorkspace.x[2] + acadoWorkspace.evGx[311]*acadoWorkspace.x[3] + acadoWorkspace.d[77];
acadoVariables.x[82] += + acadoWorkspace.evGx[312]*acadoWorkspace.x[0] + acadoWorkspace.evGx[313]*acadoWorkspace.x[1] + acadoWorkspace.evGx[314]*acadoWorkspace.x[2] + acadoWorkspace.evGx[315]*acadoWorkspace.x[3] + acadoWorkspace.d[78];
acadoVariables.x[83] += + acadoWorkspace.evGx[316]*acadoWorkspace.x[0] + acadoWorkspace.evGx[317]*acadoWorkspace.x[1] + acadoWorkspace.evGx[318]*acadoWorkspace.x[2] + acadoWorkspace.evGx[319]*acadoWorkspace.x[3] + acadoWorkspace.d[79];
acadoVariables.x[84] += + acadoWorkspace.evGx[320]*acadoWorkspace.x[0] + acadoWorkspace.evGx[321]*acadoWorkspace.x[1] + acadoWorkspace.evGx[322]*acadoWorkspace.x[2] + acadoWorkspace.evGx[323]*acadoWorkspace.x[3] + acadoWorkspace.d[80];
acadoVariables.x[85] += + acadoWorkspace.evGx[324]*acadoWorkspace.x[0] + acadoWorkspace.evGx[325]*acadoWorkspace.x[1] + acadoWorkspace.evGx[326]*acadoWorkspace.x[2] + acadoWorkspace.evGx[327]*acadoWorkspace.x[3] + acadoWorkspace.d[81];
acadoVariables.x[86] += + acadoWorkspace.evGx[328]*acadoWorkspace.x[0] + acadoWorkspace.evGx[329]*acadoWorkspace.x[1] + acadoWorkspace.evGx[330]*acadoWorkspace.x[2] + acadoWorkspace.evGx[331]*acadoWorkspace.x[3] + acadoWorkspace.d[82];
acadoVariables.x[87] += + acadoWorkspace.evGx[332]*acadoWorkspace.x[0] + acadoWorkspace.evGx[333]*acadoWorkspace.x[1] + acadoWorkspace.evGx[334]*acadoWorkspace.x[2] + acadoWorkspace.evGx[335]*acadoWorkspace.x[3] + acadoWorkspace.d[83];
acadoVariables.x[88] += + acadoWorkspace.evGx[336]*acadoWorkspace.x[0] + acadoWorkspace.evGx[337]*acadoWorkspace.x[1] + acadoWorkspace.evGx[338]*acadoWorkspace.x[2] + acadoWorkspace.evGx[339]*acadoWorkspace.x[3] + acadoWorkspace.d[84];
acadoVariables.x[89] += + acadoWorkspace.evGx[340]*acadoWorkspace.x[0] + acadoWorkspace.evGx[341]*acadoWorkspace.x[1] + acadoWorkspace.evGx[342]*acadoWorkspace.x[2] + acadoWorkspace.evGx[343]*acadoWorkspace.x[3] + acadoWorkspace.d[85];
acadoVariables.x[90] += + acadoWorkspace.evGx[344]*acadoWorkspace.x[0] + acadoWorkspace.evGx[345]*acadoWorkspace.x[1] + acadoWorkspace.evGx[346]*acadoWorkspace.x[2] + acadoWorkspace.evGx[347]*acadoWorkspace.x[3] + acadoWorkspace.d[86];
acadoVariables.x[91] += + acadoWorkspace.evGx[348]*acadoWorkspace.x[0] + acadoWorkspace.evGx[349]*acadoWorkspace.x[1] + acadoWorkspace.evGx[350]*acadoWorkspace.x[2] + acadoWorkspace.evGx[351]*acadoWorkspace.x[3] + acadoWorkspace.d[87];
acadoVariables.x[92] += + acadoWorkspace.evGx[352]*acadoWorkspace.x[0] + acadoWorkspace.evGx[353]*acadoWorkspace.x[1] + acadoWorkspace.evGx[354]*acadoWorkspace.x[2] + acadoWorkspace.evGx[355]*acadoWorkspace.x[3] + acadoWorkspace.d[88];
acadoVariables.x[93] += + acadoWorkspace.evGx[356]*acadoWorkspace.x[0] + acadoWorkspace.evGx[357]*acadoWorkspace.x[1] + acadoWorkspace.evGx[358]*acadoWorkspace.x[2] + acadoWorkspace.evGx[359]*acadoWorkspace.x[3] + acadoWorkspace.d[89];
acadoVariables.x[94] += + acadoWorkspace.evGx[360]*acadoWorkspace.x[0] + acadoWorkspace.evGx[361]*acadoWorkspace.x[1] + acadoWorkspace.evGx[362]*acadoWorkspace.x[2] + acadoWorkspace.evGx[363]*acadoWorkspace.x[3] + acadoWorkspace.d[90];
acadoVariables.x[95] += + acadoWorkspace.evGx[364]*acadoWorkspace.x[0] + acadoWorkspace.evGx[365]*acadoWorkspace.x[1] + acadoWorkspace.evGx[366]*acadoWorkspace.x[2] + acadoWorkspace.evGx[367]*acadoWorkspace.x[3] + acadoWorkspace.d[91];
acadoVariables.x[96] += + acadoWorkspace.evGx[368]*acadoWorkspace.x[0] + acadoWorkspace.evGx[369]*acadoWorkspace.x[1] + acadoWorkspace.evGx[370]*acadoWorkspace.x[2] + acadoWorkspace.evGx[371]*acadoWorkspace.x[3] + acadoWorkspace.d[92];
acadoVariables.x[97] += + acadoWorkspace.evGx[372]*acadoWorkspace.x[0] + acadoWorkspace.evGx[373]*acadoWorkspace.x[1] + acadoWorkspace.evGx[374]*acadoWorkspace.x[2] + acadoWorkspace.evGx[375]*acadoWorkspace.x[3] + acadoWorkspace.d[93];
acadoVariables.x[98] += + acadoWorkspace.evGx[376]*acadoWorkspace.x[0] + acadoWorkspace.evGx[377]*acadoWorkspace.x[1] + acadoWorkspace.evGx[378]*acadoWorkspace.x[2] + acadoWorkspace.evGx[379]*acadoWorkspace.x[3] + acadoWorkspace.d[94];
acadoVariables.x[99] += + acadoWorkspace.evGx[380]*acadoWorkspace.x[0] + acadoWorkspace.evGx[381]*acadoWorkspace.x[1] + acadoWorkspace.evGx[382]*acadoWorkspace.x[2] + acadoWorkspace.evGx[383]*acadoWorkspace.x[3] + acadoWorkspace.d[95];
acadoVariables.x[100] += + acadoWorkspace.evGx[384]*acadoWorkspace.x[0] + acadoWorkspace.evGx[385]*acadoWorkspace.x[1] + acadoWorkspace.evGx[386]*acadoWorkspace.x[2] + acadoWorkspace.evGx[387]*acadoWorkspace.x[3] + acadoWorkspace.d[96];
acadoVariables.x[101] += + acadoWorkspace.evGx[388]*acadoWorkspace.x[0] + acadoWorkspace.evGx[389]*acadoWorkspace.x[1] + acadoWorkspace.evGx[390]*acadoWorkspace.x[2] + acadoWorkspace.evGx[391]*acadoWorkspace.x[3] + acadoWorkspace.d[97];
acadoVariables.x[102] += + acadoWorkspace.evGx[392]*acadoWorkspace.x[0] + acadoWorkspace.evGx[393]*acadoWorkspace.x[1] + acadoWorkspace.evGx[394]*acadoWorkspace.x[2] + acadoWorkspace.evGx[395]*acadoWorkspace.x[3] + acadoWorkspace.d[98];
acadoVariables.x[103] += + acadoWorkspace.evGx[396]*acadoWorkspace.x[0] + acadoWorkspace.evGx[397]*acadoWorkspace.x[1] + acadoWorkspace.evGx[398]*acadoWorkspace.x[2] + acadoWorkspace.evGx[399]*acadoWorkspace.x[3] + acadoWorkspace.d[99];

for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multEDu( &(acadoWorkspace.E[ lRun3 * 4 ]), &(acadoWorkspace.x[ lRun2 + 4 ]), &(acadoVariables.x[ lRun1 * 4 + 4 ]) );
}
}
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
acadoWorkspace.state[0] = acadoVariables.x[index * 4];
acadoWorkspace.state[1] = acadoVariables.x[index * 4 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 4 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 4 + 3];
acadoWorkspace.state[24] = acadoVariables.u[index];
acadoWorkspace.state[25] = acadoVariables.od[index * 6];
acadoWorkspace.state[26] = acadoVariables.od[index * 6 + 1];
acadoWorkspace.state[27] = acadoVariables.od[index * 6 + 2];
acadoWorkspace.state[28] = acadoVariables.od[index * 6 + 3];
acadoWorkspace.state[29] = acadoVariables.od[index * 6 + 4];
acadoWorkspace.state[30] = acadoVariables.od[index * 6 + 5];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 4 + 4] = acadoWorkspace.state[0];
acadoVariables.x[index * 4 + 5] = acadoWorkspace.state[1];
acadoVariables.x[index * 4 + 6] = acadoWorkspace.state[2];
acadoVariables.x[index * 4 + 7] = acadoWorkspace.state[3];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 25; ++index)
{
acadoVariables.x[index * 4] = acadoVariables.x[index * 4 + 4];
acadoVariables.x[index * 4 + 1] = acadoVariables.x[index * 4 + 5];
acadoVariables.x[index * 4 + 2] = acadoVariables.x[index * 4 + 6];
acadoVariables.x[index * 4 + 3] = acadoVariables.x[index * 4 + 7];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[100] = xEnd[0];
acadoVariables.x[101] = xEnd[1];
acadoVariables.x[102] = xEnd[2];
acadoVariables.x[103] = xEnd[3];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[100];
acadoWorkspace.state[1] = acadoVariables.x[101];
acadoWorkspace.state[2] = acadoVariables.x[102];
acadoWorkspace.state[3] = acadoVariables.x[103];
if (uEnd != 0)
{
acadoWorkspace.state[24] = uEnd[0];
}
else
{
acadoWorkspace.state[24] = acadoVariables.u[24];
}
acadoWorkspace.state[25] = acadoVariables.od[150];
acadoWorkspace.state[26] = acadoVariables.od[151];
acadoWorkspace.state[27] = acadoVariables.od[152];
acadoWorkspace.state[28] = acadoVariables.od[153];
acadoWorkspace.state[29] = acadoVariables.od[154];
acadoWorkspace.state[30] = acadoVariables.od[155];

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[100] = acadoWorkspace.state[0];
acadoVariables.x[101] = acadoWorkspace.state[1];
acadoVariables.x[102] = acadoWorkspace.state[2];
acadoVariables.x[103] = acadoWorkspace.state[3];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 24; ++index)
{
acadoVariables.u[index] = acadoVariables.u[index + 1];
}

if (uEnd != 0)
{
acadoVariables.u[24] = uEnd[0];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28];
kkt = fabs( kkt );
for (index = 0; index < 29; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 125; ++index)
{
prd = acadoWorkspace.y[index + 29];
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
/** Row vector of size: 3 */
real_t tmpDy[ 3 ];

/** Row vector of size: 2 */
real_t tmpDyN[ 2 ];

for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 4];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 4 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 4 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 4 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.u[lRun1];
acadoWorkspace.objValueIn[5] = acadoVariables.od[lRun1 * 6];
acadoWorkspace.objValueIn[6] = acadoVariables.od[lRun1 * 6 + 1];
acadoWorkspace.objValueIn[7] = acadoVariables.od[lRun1 * 6 + 2];
acadoWorkspace.objValueIn[8] = acadoVariables.od[lRun1 * 6 + 3];
acadoWorkspace.objValueIn[9] = acadoVariables.od[lRun1 * 6 + 4];
acadoWorkspace.objValueIn[10] = acadoVariables.od[lRun1 * 6 + 5];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 3] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 3];
acadoWorkspace.Dy[lRun1 * 3 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 3 + 1];
acadoWorkspace.Dy[lRun1 * 3 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 3 + 2];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[100];
acadoWorkspace.objValueIn[1] = acadoVariables.x[101];
acadoWorkspace.objValueIn[2] = acadoVariables.x[102];
acadoWorkspace.objValueIn[3] = acadoVariables.x[103];
acadoWorkspace.objValueIn[4] = acadoVariables.od[150];
acadoWorkspace.objValueIn[5] = acadoVariables.od[151];
acadoWorkspace.objValueIn[6] = acadoVariables.od[152];
acadoWorkspace.objValueIn[7] = acadoVariables.od[153];
acadoWorkspace.objValueIn[8] = acadoVariables.od[154];
acadoWorkspace.objValueIn[9] = acadoVariables.od[155];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 3]*acadoVariables.W[lRun1 * 9] + acadoWorkspace.Dy[lRun1 * 3 + 1]*acadoVariables.W[lRun1 * 9 + 3] + acadoWorkspace.Dy[lRun1 * 3 + 2]*acadoVariables.W[lRun1 * 9 + 6];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 3]*acadoVariables.W[lRun1 * 9 + 1] + acadoWorkspace.Dy[lRun1 * 3 + 1]*acadoVariables.W[lRun1 * 9 + 4] + acadoWorkspace.Dy[lRun1 * 3 + 2]*acadoVariables.W[lRun1 * 9 + 7];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 3]*acadoVariables.W[lRun1 * 9 + 2] + acadoWorkspace.Dy[lRun1 * 3 + 1]*acadoVariables.W[lRun1 * 9 + 5] + acadoWorkspace.Dy[lRun1 * 3 + 2]*acadoVariables.W[lRun1 * 9 + 8];
objVal += + acadoWorkspace.Dy[lRun1 * 3]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 3 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 3 + 2]*tmpDy[2];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[3];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1];

objVal *= 0.5;
return objVal;
}

