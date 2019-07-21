/* Produced by CVXGEN, 2015-02-11 07:30:06 -0500.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
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

// TODO: Change all of these hard coded variables as appropriate

#define STATE_SIZE 10
#define INPUT_SIZE 3
#define DISTURBANCE_SIZE 5
#define PREDICTION_HORIZON 20
typedef struct Params_t {
  double x_0[STATE_SIZE];
  double x_ss_0[STATE_SIZE];
  double Q[STATE_SIZE * STATE_SIZE];
  double u_ss[INPUT_SIZE];
  double R[INPUT_SIZE * INPUT_SIZE];
  double u_prev[INPUT_SIZE];
  double R_omega[INPUT_SIZE * INPUT_SIZE];
  double x_ss_1[STATE_SIZE];
  double x_ss_2[STATE_SIZE];
  double x_ss_3[STATE_SIZE];
  double x_ss_4[STATE_SIZE];
  double x_ss_5[STATE_SIZE];
  double x_ss_6[STATE_SIZE];
  double x_ss_7[STATE_SIZE];
  double x_ss_8[STATE_SIZE];
  double x_ss_9[STATE_SIZE];
  double x_ss_10[STATE_SIZE];
  double x_ss_11[STATE_SIZE];
  double x_ss_12[STATE_SIZE];
  double x_ss_13[STATE_SIZE];
  double x_ss_14[STATE_SIZE];
  double x_ss_15[STATE_SIZE];
  double x_ss_16[STATE_SIZE];
  double x_ss_17[STATE_SIZE];
  double x_ss_18[STATE_SIZE];
  double x_ss_19[STATE_SIZE];
  double Q_final[STATE_SIZE * STATE_SIZE];
  double A[STATE_SIZE * STATE_SIZE];
  double B[STATE_SIZE * INPUT_SIZE];
  double Bd[STATE_SIZE * DISTURBANCE_SIZE];
  double d[DISTURBANCE_SIZE];
  double u_min[INPUT_SIZE];
  double u_max[INPUT_SIZE];
  double *x[1];
  double *x_ss[PREDICTION_HORIZON];
} Params;
// TODO: Change these comments because they are obsolete
typedef struct Vars_t {
  double *u_0; /* 3 rows. */
  double *x_1; /* 8 rows. */
  double *u_1; /* 3 rows. */
  double *t_01; /* 3 rows. */
  double *x_2; /* 8 rows. */
  double *u_2; /* 3 rows. */
  double *t_02; /* 3 rows. */
  double *x_3; /* 8 rows. */
  double *u_3; /* 3 rows. */
  double *t_03; /* 3 rows. */
  double *x_4; /* 8 rows. */
  double *u_4; /* 3 rows. */
  double *t_04; /* 3 rows. */
  double *x_5; /* 8 rows. */
  double *u_5; /* 3 rows. */
  double *t_05; /* 3 rows. */
  double *x_6; /* 8 rows. */
  double *u_6; /* 3 rows. */
  double *t_06; /* 3 rows. */
  double *x_7; /* 8 rows. */
  double *u_7; /* 3 rows. */
  double *t_07; /* 3 rows. */
  double *x_8; /* 8 rows. */
  double *u_8; /* 3 rows. */
  double *t_08; /* 3 rows. */
  double *x_9; /* 8 rows. */
  double *u_9; /* 3 rows. */
  double *t_09; /* 3 rows. */
  double *x_10; /* 8 rows. */
  double *u_10; /* 3 rows. */
  double *t_10; /* 3 rows. */
  double *x_11; /* 8 rows. */
  double *u_11; /* 3 rows. */
  double *t_11; /* 3 rows. */
  double *x_12; /* 8 rows. */
  double *u_12; /* 3 rows. */
  double *t_12; /* 3 rows. */
  double *x_13; /* 8 rows. */
  double *u_13; /* 3 rows. */
  double *t_13; /* 3 rows. */
  double *x_14; /* 8 rows. */
  double *u_14; /* 3 rows. */
  double *t_14; /* 3 rows. */
  double *x_15; /* 8 rows. */
  double *u_15; /* 3 rows. */
  double *t_15; /* 3 rows. */
  double *x_16; /* 8 rows. */
  double *u_16; /* 3 rows. */
  double *t_16; /* 3 rows. */
  double *x_17; /* 8 rows. */
  double *u_17; /* 3 rows. */
  double *t_17; /* 3 rows. */
  double *x_18; /* 8 rows. */
  double *u_18; /* 3 rows. */
  double *t_18; /* 3 rows. */
  double *x_19; /* 8 rows. */
  double *u[PREDICTION_HORIZON - 1];
  double *x[PREDICTION_HORIZON];
} Vars;
typedef struct Workspace_t {
  double h[114];
  double s_inv[114];
  double s_inv_z[114];
  double b[206];
  double q[263];
  double rhs[697];
  double x[697];
  double *s;
  double *z;
  double *y;
  double lhs_aff[697];
  double lhs_cc[697];
  double buffer[697];
  double buffer2[697];
  double KKT[3284];
  double L[5723];
  double d[697];
  double v[697];
  double d_inv[697];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  double quad_203336859648[1];
  double quad_830833205248[1];
  double quad_295866224640[1];
  double quad_600568381440[1];
  double quad_898851794944[1];
  double quad_88433618944[1];
  double quad_240204779520[1];
  double quad_635618762752[1];
  double quad_732753989632[1];
  double quad_427523055616[1];
  double quad_976046530560[1];
  double quad_688550678528[1];
  double quad_304816418816[1];
  double quad_819339411456[1];
  double quad_101800079360[1];
  double quad_976903761920[1];
  double quad_141299838976[1];
  double quad_343404097536[1];
  double quad_815806124032[1];
  double quad_997002137600[1];
  double quad_141630619648[1];
  double quad_854393544704[1];
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
