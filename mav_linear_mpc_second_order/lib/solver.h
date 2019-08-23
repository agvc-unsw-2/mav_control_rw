/* Produced by CVXGEN, 2019-08-10 00:40:10 -0400.  */
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
  double x_0[10];
  double x_ss_0[10];
  double Q[100];
  double u_ss_0[3];
  double R[9];
  double u_prev[3];
  double R_omega[9];
  double x_ss_1[10];
  double u_ss_1[3];
  double x_ss_2[10];
  double u_ss_2[3];
  double x_ss_3[10];
  double u_ss_3[3];
  double x_ss_4[10];
  double u_ss_4[3];
  double x_ss_5[10];
  double u_ss_5[3];
  double x_ss_6[10];
  double u_ss_6[3];
  double x_ss_7[10];
  double u_ss_7[3];
  double x_ss_8[10];
  double u_ss_8[3];
  double x_ss_9[10];
  double u_ss_9[3];
  double x_ss_10[10];
  double u_ss_10[3];
  double x_ss_11[10];
  double u_ss_11[3];
  double x_ss_12[10];
  double u_ss_12[3];
  double x_ss_13[10];
  double u_ss_13[3];
  double x_ss_14[10];
  double u_ss_14[3];
  double x_ss_15[10];
  double u_ss_15[3];
  double x_ss_16[10];
  double Q_final[100];
  double A[100];
  double B[30];
  double Bd[50];
  double d[5];
  double u_min[3];
  double u_max[3];
  double *x[1];
  double *x_ss[17];
  double *u_ss[16];
} Params;
typedef struct Vars_t {
  double *u_0; /* 3 rows. */
  double *x_1; /* 10 rows. */
  double *u_1; /* 3 rows. */
  double *t_01; /* 3 rows. */
  double *x_2; /* 10 rows. */
  double *u_2; /* 3 rows. */
  double *t_02; /* 3 rows. */
  double *x_3; /* 10 rows. */
  double *u_3; /* 3 rows. */
  double *t_03; /* 3 rows. */
  double *x_4; /* 10 rows. */
  double *u_4; /* 3 rows. */
  double *t_04; /* 3 rows. */
  double *x_5; /* 10 rows. */
  double *u_5; /* 3 rows. */
  double *t_05; /* 3 rows. */
  double *x_6; /* 10 rows. */
  double *u_6; /* 3 rows. */
  double *t_06; /* 3 rows. */
  double *x_7; /* 10 rows. */
  double *u_7; /* 3 rows. */
  double *t_07; /* 3 rows. */
  double *x_8; /* 10 rows. */
  double *u_8; /* 3 rows. */
  double *t_08; /* 3 rows. */
  double *x_9; /* 10 rows. */
  double *u_9; /* 3 rows. */
  double *t_09; /* 3 rows. */
  double *x_10; /* 10 rows. */
  double *u_10; /* 3 rows. */
  double *t_10; /* 3 rows. */
  double *x_11; /* 10 rows. */
  double *u_11; /* 3 rows. */
  double *t_11; /* 3 rows. */
  double *x_12; /* 10 rows. */
  double *u_12; /* 3 rows. */
  double *t_12; /* 3 rows. */
  double *x_13; /* 10 rows. */
  double *u_13; /* 3 rows. */
  double *t_13; /* 3 rows. */
  double *x_14; /* 10 rows. */
  double *u_14; /* 3 rows. */
  double *t_14; /* 3 rows. */
  double *x_15; /* 10 rows. */
  double *u_15; /* 3 rows. */
  double *t_15; /* 3 rows. */
  double *x_16; /* 10 rows. */
  double *u[16];
  double *x[17];
} Vars;
typedef struct Workspace_t {
  double h[96];
  double s_inv[96];
  double s_inv_z[96];
  double b[205];
  double q[253];
  double rhs[650];
  double x[650];
  double *s;
  double *z;
  double *y;
  double lhs_aff[650];
  double lhs_cc[650];
  double buffer[650];
  double buffer2[650];
  double KKT[3725];
  double L[6825];
  double d[650];
  double v[650];
  double d_inv[650];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  double quad_203336859648[1];
  double quad_447791480832[1];
  double quad_295866224640[1];
  double quad_600568381440[1];
  double quad_885004812288[1];
  double quad_898851794944[1];
  double quad_469996625920[1];
  double quad_88433618944[1];
  double quad_768722419712[1];
  double quad_240204779520[1];
  double quad_757472997376[1];
  double quad_635618762752[1];
  double quad_179070365696[1];
  double quad_732753989632[1];
  double quad_384536985600[1];
  double quad_427523055616[1];
  double quad_498084270080[1];
  double quad_976046530560[1];
  double quad_452736155648[1];
  double quad_688550678528[1];
  double quad_31005663232[1];
  double quad_304816418816[1];
  double quad_68114292736[1];
  double quad_819339411456[1];
  double quad_618980339712[1];
  double quad_101800079360[1];
  double quad_615069044736[1];
  double quad_976903761920[1];
  double quad_374380896256[1];
  double quad_141299838976[1];
  double quad_74079137792[1];
  double quad_343404097536[1];
  double quad_811402448896[1];
  double quad_136942120960[1];
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
