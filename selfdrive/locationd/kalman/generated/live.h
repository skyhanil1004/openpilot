/******************************************************************************
 *                       Code generated with sympy 1.4                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6412539246219425063);
void inv_err_fun(double *nom_x, double *true_x, double *out_3447978071008418269);
void H_mod_fun(double *state, double *out_2612123311179984213);
void f_fun(double *state, double dt, double *out_3461949183891425270);
void F_fun(double *state, double dt, double *out_8149062268694723908);
void h_3(double *state, double *unused, double *out_6072684242542102547);
void H_3(double *state, double *unused, double *out_8991321652720524795);
void h_4(double *state, double *unused, double *out_6409971745426839602);
void H_4(double *state, double *unused, double *out_5429016731915119215);
void h_9(double *state, double *unused, double *out_1429423819866436857);
void H_9(double *state, double *unused, double *out_4044767390977250369);
void h_10(double *state, double *unused, double *out_6452066904172374942);
void H_10(double *state, double *unused, double *out_6695875842646951432);
void h_12(double *state, double *unused, double *out_3485962628728471013);
void H_12(double *state, double *unused, double *out_7103856635566649839);
void h_13(double *state, double *unused, double *out_3561606103561857895);
void H_13(double *state, double *unused, double *out_5028534438946590382);
void h_14(double *state, double *unused, double *out_1429423819866436857);
void H_14(double *state, double *unused, double *out_4044767390977250369);
void h_19(double *state, double *unused, double *out_3911386592410781289);
void H_19(double *state, double *unused, double *out_5146428357178863147);
#define DIM 23
#define EDIM 22
#define MEDIM 22
typedef void (*Hfun)(double *, double *, double *);

void predict(double *x, double *P, double *Q, double dt);
const static double MAHA_THRESH_3 = 3.841459;
void update_3(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814728;
void update_4(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_9 = 7.814728;
void update_9(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_10 = 7.814728;
void update_10(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_12 = 7.814728;
void update_12(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_13 = 7.814728;
void update_13(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_14 = 7.814728;
void update_14(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_19 = 7.814728;
void update_19(double *, double *, double *, double *, double *);