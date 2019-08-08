/* Produced by CVXGEN, 2019-08-01 23:42:46 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef SOLVER_H
#define SOLVER_H
/* Uncomment the next line to remove all library dependencies. */
/*#define ZERO_LIBRARY_MODE */
#ifdef MATLAB_MEX_FILE
/* Matlab functions. MATLAB_MEX_FILE will be defined by the mex compiler. */
/* If you are not using the mex compiler, this functionality will not intrude, */
/* as it will be completely disabled at compile-time. */
#include "mex.h"
#else
#ifndef ZERO_LIBRARY_MODE
#include <stdio.h>
#endif
#endif

#ifdef __cplusplus
 extern "C" {
#endif

/* Space must be allocated somewhere (testsolver.c, csolve.c or your own */
/* program) for the global variables vars, params, work and settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define pm(A, m, n) printmatrix(#A, A, m, n, 1)
#endif
typedef struct Params_t {
  double x_0[12];
  double x_ss_0[12];
  double Q[144];
  double u_ss_0[3];
  double R[9];
  double u_prev[3];
  double R_omega[9];
  double x_ss_1[12];
  double u_ss_1[3];
  double x_ss_2[12];
  double u_ss_2[3];
  double x_ss_3[12];
  double u_ss_3[3];
  double x_ss_4[12];
  double u_ss_4[3];
  double x_ss_5[12];
  double u_ss_5[3];
  double x_ss_6[12];
  double u_ss_6[3];
  double x_ss_7[12];
  double u_ss_7[3];
  double x_ss_8[12];
  double u_ss_8[3];
  double x_ss_9[12];
  double u_ss_9[3];
  double x_ss_10[12];
  double u_ss_10[3];
  double x_ss_11[12];
  double u_ss_11[3];
  double x_ss_12[12];
  double u_ss_12[3];
  double x_ss_13[12];
  double u_ss_13[3];
  double x_ss_14[12];
  double Q_final[144];
  double A[144];
  double B[36];
  double Bd[72];
  double d[6];
  double u_min[3];
  double u_max[3];
  double *x[1];
  double *x_ss[15];
  double *u_ss[14];
} Params;
typedef struct Vars_t {
  double *u_0; /* 3 rows. */
  double *x_1; /* 12 rows. */
  double *u_1; /* 3 rows. */
  double *t_01; /* 3 rows. */
  double *x_2; /* 12 rows. */
  double *u_2; /* 3 rows. */
  double *t_02; /* 3 rows. */
  double *x_3; /* 12 rows. */
  double *u_3; /* 3 rows. */
  double *t_03; /* 3 rows. */
  double *x_4; /* 12 rows. */
  double *u_4; /* 3 rows. */
  double *t_04; /* 3 rows. */
  double *x_5; /* 12 rows. */
  double *u_5; /* 3 rows. */
  double *t_05; /* 3 rows. */
  double *x_6; /* 12 rows. */
  double *u_6; /* 3 rows. */
  double *t_06; /* 3 rows. */
  double *x_7; /* 12 rows. */
  double *u_7; /* 3 rows. */
  double *t_07; /* 3 rows. */
  double *x_8; /* 12 rows. */
  double *u_8; /* 3 rows. */
  double *t_08; /* 3 rows. */
  double *x_9; /* 12 rows. */
  double *u_9; /* 3 rows. */
  double *t_09; /* 3 rows. */
  double *x_10; /* 12 rows. */
  double *u_10; /* 3 rows. */
  double *t_10; /* 3 rows. */
  double *x_11; /* 12 rows. */
  double *u_11; /* 3 rows. */
  double *t_11; /* 3 rows. */
  double *x_12; /* 12 rows. */
  double *u_12; /* 3 rows. */
  double *t_12; /* 3 rows. */
  double *x_13; /* 12 rows. */
  double *u_13; /* 3 rows. */
  double *t_13; /* 3 rows. */
  double *x_14; /* 12 rows. */
  double *u[14];
  double *x[15];
} Vars;
typedef struct Workspace_t {
  double h[84];
  double s_inv[84];
  double s_inv_z[84];
  double b[207];
  double q[249];
  double rhs[624];
  double x[624];
  double *s;
  double *z;
  double *y;
  double lhs_aff[624];
  double lhs_cc[624];
  double buffer[624];
  double buffer2[624];
  double KKT[4251];
  double L[8055];
  double d[624];
  double v[624];
  double d_inv[624];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  double quad_345851330560[1];
  double quad_503357964288[1];
  double quad_689417555968[1];
  double quad_590025695232[1];
  double quad_24887435264[1];
  double quad_373406756864[1];
  double quad_282534416384[1];
  double quad_569492807680[1];
  double quad_658085392384[1];
  double quad_268442972160[1];
  double quad_212760903680[1];
  double quad_604237524992[1];
  double quad_899615596544[1];
  double quad_9807298560[1];
  double quad_730290806784[1];
  double quad_691068739584[1];
  double quad_512929845248[1];
  double quad_46428270592[1];
  double quad_907958816768[1];
  double quad_364190564352[1];
  double quad_603410014208[1];
  double quad_253294055424[1];
  double quad_406480764928[1];
  double quad_450189660160[1];
  double quad_622448181248[1];
  double quad_135073144832[1];
  double quad_512757641216[1];
  double quad_265658822656[1];
  double quad_225439469568[1];
  double quad_282763468800[1];
  int converged;
} Workspace;
typedef struct Settings_t {
  double resid_tol;
  double eps;
  int max_iters;
  int refine_steps;
  int better_start;
  /* Better start obviates the need for s_init and z_init. */
  double s_init;
  double z_init;
  int verbose;
  /* Show extra details of the iterative refinement steps. */
  int verbose_refinement;
  int debug;
  /* For regularization. Minimum value of abs(D_ii) in the kkt D factor. */
  double kkt_reg;
} Settings;
extern Vars vars;
extern Params params;
extern Workspace work;
extern Settings settings;
/* Function definitions in ldl.c: */
void ldl_solve(double *target, double *var);
void ldl_factor(void);
double check_factorization(void);
void matrix_multiply(double *result, double *source);
double check_residual(double *target, double *multiplicand);
void fill_KKT(void);

/* Function definitions in matrix_support.c: */
void multbymA(double *lhs, double *rhs);
void multbymAT(double *lhs, double *rhs);
void multbymG(double *lhs, double *rhs);
void multbymGT(double *lhs, double *rhs);
void multbyP(double *lhs, double *rhs);
void fillq(void);
void fillh(void);
void fillb(void);
void pre_ops(void);

/* Function definitions in solver.c: */
double eval_gap(void);
void set_defaults(void);
void setup_pointers(void);
void setup_indexed_params(void);
void setup_indexed_optvars(void);
void setup_indexing(void);
void set_start(void);
double eval_objv(void);
void fillrhs_aff(void);
void fillrhs_cc(void);
void refine(double *target, double *var);
double calc_ineq_resid_squared(void);
double calc_eq_resid_squared(void);
void better_start(void);
void fillrhs_start(void);
long solve(void);

/* Function definitions in testsolver.c: */
int main(int argc, char **argv);
void load_default_data(void);

/* Function definitions in util.c: */
void tic(void);
float toc(void);
float tocq(void);
void printmatrix(char *name, double *A, int m, int n, int sparse);
double unif(double lower, double upper);
float ran1(long*idum, int reset);
float randn_internal(long *idum, int reset);
double randn(void);
void reset_rand(void);

#ifdef __cplusplus
 }
#endif

#endif
