/******************************************************************************
 *                       Code generated with sympy 1.4                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3414498779711784903);
void inv_err_fun(double *nom_x, double *true_x, double *out_3801369187224533461);
void H_mod_fun(double *state, double *out_4854957942003700907);
void f_fun(double *state, double dt, double *out_6741758115209166220);
void F_fun(double *state, double dt, double *out_2107885154203272808);
void h_3(double *state, double *unused, double *out_7981152162671521753);
void H_3(double *state, double *unused, double *out_2519822197561930229);
void h_4(double *state, double *unused, double *out_426845743463941710);
void H_4(double *state, double *unused, double *out_8754257968195554305);
void h_9(double *state, double *unused, double *out_20488707581374143);
void H_9(double *state, double *unused, double *out_3470864254473947255);
void h_10(double *state, double *unused, double *out_176226521415790442);
void H_10(double *state, double *unused, double *out_4842421067945354980);
void h_12(double *state, double *unused, double *out_433346968145880749);
void H_12(double *state, double *unused, double *out_236952447915613241);
void h_13(double *state, double *unused, double *out_9198273837189256251);
void H_13(double *state, double *unused, double *out_4080054580062789982);
void h_14(double *state, double *unused, double *out_20488707581374143);
void H_14(double *state, double *unused, double *out_3470864254473947255);
void h_19(double *state, double *unused, double *out_473024383151157687);
void H_19(double *state, double *unused, double *out_120558291359287869);
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