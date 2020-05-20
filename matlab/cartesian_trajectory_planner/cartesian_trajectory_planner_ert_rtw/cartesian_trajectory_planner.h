//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: cartesian_trajectory_planner.h
//
// Code generated for Simulink model 'cartesian_trajectory_planner'.
//
// Model version                  : 1.129
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Wed May 20 16:41:34 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_cartesian_trajectory_planner_h_
#define RTW_HEADER_cartesian_trajectory_planner_h_
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <string.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros_initialize.h"
#include "ctimefun.h"
#include "cartesian_trajectory_planner_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include "rtGetInf.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// Block signals (default storage)
typedef struct {
  SL_Bus_cartesian_trajectory_plan_JointTrajectoryPoint_1eqsod msg;// '<Root>/MATLAB Function' 
  uint32_T uv[625];
  uint32_T uv1[625];
  real_T xi[257];
  real_T dv[36];
  real_T unusedU0[36];
  real_T H[36];
  real_T P[36];
  real_T V[36];
  real_T H_m[36];
  real_T sNew[36];
  real_T X[36];
  real_T unusedU0_c[36];
  real_T H_k[36];
  real_T P_c[36];
  real_T V_b[36];
  real_T H_p[36];
  real_T sNew_c[36];
  real_T X_f[36];
  real_T X_g[36];
  real_T X_g1[36];
  real_T X_m[36];
  real_T X_n[36];
  real_T X_p[36];
  real_T c_A_data[36];
  real_T out[16];
  real_T out_j[16];
  real_T POSA[16];                     // '<Root>/Completa'
  real_T Td[16];
  real_T T_data[16];
  real_T T1[16];
  real_T Tc2p[16];
  real_T Tj[16];
  real_T T1j[16];
  real_T TJ[16];
  real_T a[16];
  real_T b[16];
  real_T a_d[16];
  real_T TJ_g[16];
  real_T a_l[16];
  real_T b_d[16];
  real_T a_dy[16];
  real_T T2inv[16];
  real_T T2[16];
  real_T T1_l[16];
  real_T T[16];
  real_T Tdh[16];
  real_T c_f1[16];
  real_T a_o[16];
  real_T b_b[16];
  real_T a_n[16];
  real_T Td_b[16];
  real_T T_data_l[16];
  real_T T1_h[16];
  real_T Tc2p_b[16];
  real_T Tj_d[16];
  real_T T1j_e[16];
  real_T T1_b[16];
  real_T Tc2p_j[16];
  real_T Tj_f[16];
  real_T T1j_a[16];
  real_T T1_j[16];
  real_T Tc2p_jz[16];
  real_T Tj_o[16];
  real_T T1j_n[16];
  real_T T1_i[16];
  real_T Tc2p_o[16];
  real_T Tj_n[16];
  real_T T1j_m[16];
  real_T T1_c[16];
  real_T Tc2p_m[16];
  real_T Tj_m[16];
  real_T T1j_j[16];
  real_T c_f1_h[16];
  real_T a_c[16];
  real_T b_c[16];
  real_T a_p[16];
  real_T obj[16];
  real_T Td_p[16];
  real_T T_data_a[16];
  real_T Td_e[16];
  real_T T_data_ax[16];
  real_T Td_a[16];
  real_T T_data_i[16];
  real_T Td_l[16];
  real_T T_data_o[16];
  f_cell_wrap_cartesian_traject_T expl_temp;
  f_cell_wrap_cartesian_traject_T expl_temp_l;
  real_T poslim_data[12];
  real_T poslim_data_o[12];
  real_T poslim_data_i[12];
  real_T poslim_data_f[12];
  real_T Matrot[9];
  real_T R[9];
  real_T tempR[9];
  real_T T_i[9];
  real_T Td_f[9];
  real_T R_g[9];
  real_T R_c[9];
  real_T R_o[9];
  real_T tempR_l[9];
  real_T R_m[9];
  real_T tempR_m[9];
  real_T R_cn[9];
  real_T R_f[9];
  real_T R_p[9];
  real_T tempR_e[9];
  real_T T_o[9];
  real_T Td_h[9];
  real_T V_l[9];
  real_T b_I[9];
  real_T b_U[9];
  real_T A[9];
  real_T A_h[9];
  real_T R_me[9];
  real_T R_mc[9];
  real_T R_h[9];
  real_T R_cs[9];
  real_T R_k[9];
  real_T R_pc[9];
  real_T R_px[9];
  real_T R_p4[9];
  real_T R_a[9];
  real_T R_j[9];
  real_T R_e[9];
  real_T tempR_o[9];
  real_T T_b[9];
  real_T Td_ao[9];
  real_T T_g[9];
  real_T Td_ex[9];
  real_T T_f[9];
  real_T Td_h2[9];
  real_T T_e[9];
  real_T Td_c[9];
  real_T Vel[6];                       // '<Root>/Traslazione '
  real_T MatrixMultiply1[6];           // '<Root>/MatrixMultiply1'
  real_T dv1[6];
  real_T qvSolRaw[6];
  real_T c_xSol[6];
  real_T x[6];
  real_T Hg[6];
  real_T sNew_a[6];
  real_T e[6];
  real_T y[6];
  real_T qv_data[6];
  real_T x_d[6];
  real_T Hg_a[6];
  real_T sNew_p[6];
  real_T e_m[6];
  real_T y_o[6];
  real_T qv_data_n[6];
  real_T qv_data_l[6];
  real_T qv_data_p[6];
  real_T qv_data_pt[6];
  real_T qv_data_f[6];
  real_T e_i[6];
  real_T y_ox[6];
  real_T e_k[6];
  real_T y_i[6];
  real_T e_o[6];
  real_T y_m[6];
  real_T e_c[6];
  real_T y_f[6];
  real_T unusedExpr[5];
  real_T unusedExpr_h[5];
  int8_T msubspace_data[36];
  int8_T msubspace_data_m[36];
  int8_T msubspace_data_a[36];
  int8_T msubspace_data_k[36];
  real_T v[4];
  real_T result_data[4];
  real_T result_data_p[4];
  real_T v_b[4];
  real_T result_data_c[4];
  real_T v_n[4];
  real_T v_i[4];
  real_T v_m[4];
  real_T v_j[4];
  real_T P_e[3];
  real_T out_m[3];
  real_T out_m0[3];
  real_T v_jg[3];
  real_T v_f[3];
  real_T R_a5[3];
  real_T v_g[3];
  real_T v_nr[3];
  real_T vspecial_data[3];
  real_T s[3];
  real_T e_d[3];
  real_T work[3];
  real_T v_na[3];
  int32_T e_data[6];
  int32_T e_data_c[6];
  char_T cv[18];
  SL_Bus_cartesian_trajectory_planner_rosgraph_msgs_Clock In1;// '<S13>/In1'
  SL_Bus_cartesian_trajectory_planner_rosgraph_msgs_Clock b_varargout_2;
  real_T ub[2];
  creal_T v_fx;
  creal_T u;
  creal_T u_p;
  creal_T dc;
  char_T sol_Status_data[14];
  char_T cv1[13];
  char_T cv2[11];
  char_T a_p2[11];
  char_T b_n[9];
  char_T b_k[9];
  char_T b_n3[9];
  char_T b_o[9];
  char_T b_g[9];
  char_T b_cq[9];
  char_T partial_match_data[9];
  char_T b_vstr[9];
  char_T b_cj[9];
  char_T b_m[8];
  char_T b_j[8];
  char_T b_kn[8];
  char_T b_mx[8];
  char_T b_p[8];
  int8_T chainmask[8];
  char_T b_dz[8];
  char_T b_g4[8];
  char_T b_cx[8];
  char_T b_i[8];
  char_T b_dx[8];
  char_T vstr[8];
  char_T b_g4k[8];
  real_T tc;
  real_T SpMax;
  real_T s_i;
  real_T s_vel;
  real_T tcstar;
  real_T tf;
  real_T tetaFIN;
  real_T teta_vel;
  real_T tft;
  real_T maxval;
  real_T value;
  real_T value_h;
  real_T value_n;
  real_T value_o;
  real_T value_c;
  real_T value_b;
  real_T time;                         // '<Root>/MATLAB Function1'
  real_T rtb_signal2_idx_1;
  real_T rtb_signal2_idx_2;
  real_T d;
  real_T bid;
  real_T numPositions;
  real_T ndbl;
  real_T apnd;
  real_T cdiff;
  real_T u0;
  real_T u1;
  real_T tol;
  real_T err;
  real_T iter;
  real_T cost;
  real_T b_gamma;
  real_T beta;
  real_T sigma;
  real_T costNew;
  real_T m;
  real_T s_e;
  real_T A_d;
  real_T sigma_i;
  real_T s_g;
  real_T bid1;
  real_T bid2;
  real_T qidx_idx_1;
  real_T cth;
  real_T sth;
  real_T tempR_tmp;
  real_T tempR_tmp_n;
  real_T tempR_tmp_l;
  real_T tempR_tmp_c;
  real_T tempR_tmp_nc;
  real_T cost_p;
  real_T b_gamma_d;
  real_T beta_o;
  real_T sigma_j;
  real_T costNew_c;
  real_T m_h;
  real_T s_d;
  real_T A_c;
  real_T sigma_p;
  real_T b_pi;
  real_T axang_idx_0;
  real_T axang_idx_1;
  real_T axang_idx_2;
  real_T bid_a;
  real_T b_index;
  real_T x_o;
  real_T d_u;
  real_T endeffectorIndex;
  real_T s_j;
  real_T idx_idx_1;
  real_T n;
  real_T k;
  real_T sth_p;
  real_T tempR_tmp_o;
  real_T tempR_tmp_lm;
  real_T tempR_tmp_k;
  real_T tempR_tmp_j;
  real_T s_f;
  real_T a_cm;
  real_T q;
  real_T nrm;
  real_T rt;
  real_T ztest;
  real_T smm1;
  real_T emm1;
  real_T sqds;
  real_T b_f;
  real_T unusedU2;
  real_T d_sn;
  real_T roe;
  real_T absa;
  real_T absb;
  real_T scale;
  real_T ads;
  real_T bds;
  real_T bid1_n;
  real_T bid2_i;
  real_T qidx_idx_1_l;
  real_T bid1_i;
  real_T bid2_k;
  real_T qidx_idx_1_f;
  real_T bid1_a;
  real_T bid2_d;
  real_T qidx_idx_1_e;
  real_T bid1_e;
  real_T bid2_b;
  real_T qidx_idx_1_a;
  real_T bid1_il;
  real_T bid2_f;
  real_T qidx_idx_1_j;
  real_T n_o;
  real_T k_f;
  real_T sth_o;
  real_T tempR_tmp_ln;
  real_T tempR_tmp_lu;
  real_T tempR_tmp_g;
  real_T tempR_tmp_d;
  real_T ssq;
  real_T c;
  real_T bid_d;
  real_T b_index_j;
  real_T pid;
  real_T b_index_f;
  real_T obj_idx_0;
  real_T obj_idx_1;
  real_T obj_idx_2;
  real_T tol_j;
  real_T scale_h;
  real_T absxk;
  real_T t;
  real_T smax;
  real_T scale_c;
  real_T absxk_n;
  real_T t_k;
  real_T s_a;
  real_T s_fn;
  real_T s_jz;
  real_T s_k;
  uint32_T b_u[2];
  uint32_T u32[2];
  uint32_T b_u_b[2];
  int32_T sol_Status_size[2];
  int32_T T_size[2];
  int32_T T_size_c[2];
  int32_T T_size_l[2];
  int32_T T_size_f[2];
  int32_T T_size_d[2];
  int32_T T_size_j[2];
  char_T cv3[7];
  char_T cv4[6];
  int8_T iv[6];
  int8_T iv1[6];
  int8_T iv2[6];
  int8_T iv3[6];
  char_T b_h[5];
  char_T b_e[5];
  char_T b_hn[5];
  char_T cv5[5];
  char_T b_ku[5];
  char_T b_jw[5];
  char_T b_oo[5];
  char_T b_cs[5];
  char_T b_hw[5];
  char_T b_iy[5];
  char_T b_pl[5];
  char_T b_f0[5];
  char_T b_ew[5];
  char_T b_nh[5];
  char_T b_ho[5];
  char_T c_vstr[5];
  char_T b_hp[5];
  char_T cv6[4];
  int32_T r1;
  int32_T r2;
  int32_T r3;
  int32_T rtemp;
  int32_T Matrot_tmp;
  int32_T Matrot_tmp_f;
  int32_T c_i;
  int32_T partialTrueCount;
  int32_T nm1d2;
  int32_T k_f4;
  int32_T c_c;
  int32_T i;
  int32_T loop_ub;
  int32_T c_n;
  int32_T ix;
  int32_T nx;
  int32_T i_h;
  int32_T unnamed_idx_1;
  int32_T idxl;
  int32_T j;
  int32_T nx_k;
  int32_T m_hz;
  int32_T inner;
  int32_T n_b;
  int32_T idx;
  int32_T b_i_o;
  int32_T coffset;
  int32_T boffset;
  int32_T aoffset;
  int32_T i_n;
  int32_T g_idx_0;
  int32_T n_m;
  int32_T boffset_k;
  int32_T b_j_j;
  int32_T loop_ub_h;
  int32_T Td_tmp;
  int32_T jointSign;
  int32_T g;
  int32_T j_f;
  int32_T b_kstr;
  int32_T b_i_d;
  int32_T n_l;
  int32_T loop_ub_k;
  int32_T qv_size;
  int32_T coffset_tmp;
  int32_T b_kstr_i;
  int32_T loop_ub_h5;
  int32_T a_tmp;
  int32_T a_tmp_tmp;
  int32_T b_kstr_m;
  int32_T loop_ub_g;
  int32_T newNumel;
  int32_T i_l;
  int32_T idxl_m;
  int32_T j_n;
  int32_T nx_g;
  int32_T m_d;
  int32_T inner_m;
  int32_T n_f;
  int32_T idx_g;
  int32_T b_i_j;
  int32_T coffset_c;
  int32_T boffset_e;
  int32_T aoffset_m;
  int32_T i_o;
  int32_T g_idx_0_a;
  int32_T b_kstr_j;
  int32_T loop_ub_ga;
  int32_T a_tmp_j;
  int32_T a_tmp_tmp_e;
  int32_T i_j;
  int32_T i_jb;
  int32_T b_i_g;
  int32_T b_kstr_o;
  int32_T loop_ub_hm;
  int32_T b_kstr_c;
  int32_T loop_ub_a;
  int32_T b_kstr_l;
  int32_T loop_ub_j;
  int32_T b_k_i;
  int32_T b_mti;
  int32_T b_j_m;
  int32_T b_kk;
  int32_T d_f;
  int32_T b_k_o;
  int32_T newNumel_i;
  int32_T i_e;
  int32_T d_j;
  int32_T b_k_o4;
  int32_T i_f;
  int32_T b_mti_m;
  int32_T k_a;
  int32_T c_h;
  int32_T b_i_oc;
  int32_T kstr;
  int32_T b_kstr_h;
  int32_T n_j;
  int32_T loop_ub_g3;
  int32_T coffset_tmp_j;
  int32_T d_l;
  int32_T e_kb;
  int32_T ntilecols;
  int32_T b_jtilecol;
  int32_T b_kstr_d;
  int32_T loop_ub_n;
  int32_T b_kstr_j5;
  int32_T loop_ub_a3;
  int32_T n_h;
  int32_T boffset_i;
  int32_T b_j_d;
  int32_T loop_ub_b;
  int32_T Td_tmp_h;
  int32_T b_k_p;
  int32_T loop_ub_nk;
  int32_T jointSign_j;
  int32_T g_o;
  int32_T j_b;
  int32_T b_kstr_jk;
  int32_T b_i_e;
  int32_T n_i;
  int32_T loop_ub_ng;
  int32_T qv_size_i;
  int32_T coffset_tmp_p;
  int32_T jointSign_o;
  int32_T g_m;
  int32_T j_o;
  int32_T b_kstr_g;
  int32_T b_i_ez;
  int32_T n_iz;
  int32_T loop_ub_gb;
  int32_T qv_size_g;
  int32_T coffset_tmp_g;
  int32_T jointSign_g;
  int32_T g_c;
  int32_T j_k;
  int32_T b_kstr_ds;
  int32_T b_i_k;
  int32_T n_p;
  int32_T loop_ub_p;
  int32_T qv_size_m;
  int32_T coffset_tmp_k;
  int32_T jointSign_a;
  int32_T g_f;
  int32_T j_c;
  int32_T b_kstr_jkx;
  int32_T b_i_kb;
  int32_T n_hj;
  int32_T loop_ub_d;
  int32_T qv_size_j;
  int32_T coffset_tmp_n;
  int32_T jointSign_j0;
  int32_T g_l;
  int32_T j_p;
  int32_T b_kstr_p;
  int32_T b_i_l;
  int32_T n_ld;
  int32_T loop_ub_hb;
  int32_T qv_size_c;
  int32_T coffset_tmp_g0;
  int32_T d_e;
  int32_T e_n;
  int32_T ntilecols_f;
  int32_T b_jtilecol_n;
  int32_T b_kstr_e;
  int32_T loop_ub_by;
  int32_T b_kstr_a;
  int32_T b_info;
  int32_T jm1;
  int32_T idxAjj;
  int32_T b_j_i;
  int32_T d_n;
  int32_T ia;
  int32_T ix_f;
  int32_T iy;
  int32_T k_i;
  int32_T c_A_size_idx_0;
  int32_T c_A_size_idx_1;
  int32_T i_k;
  int32_T b_i_b;
  int32_T b_kstr_da;
  int32_T loop_ub_hs;
  int32_T b_i_n;
  int32_T b_kstr_f;
  int32_T loop_ub_ay;
  int32_T b_kstr_mu;
  int32_T loop_ub_gu;
  int32_T nmatched;
  int32_T minnanb;
  int32_T kstr_n;
  int32_T loop_ub_c;
  int32_T partial_match_size_idx_1;
  int32_T rankR;
  int32_T na;
  int32_T minmn;
  int32_T nb;
  int32_T minmana;
  int32_T m_d4;
  int32_T nb_k;
  int32_T mn;
  int32_T b_i_c;
  int32_T ma;
  int32_T minmn_j;
  int32_T ii;
  int32_T nmi;
  int32_T mmi;
  int32_T pvt;
  int32_T itemp;
  int32_T lastc;
  int32_T kend;
  int32_T ix_m;
  int32_T iy_i;
  int32_T d_b;
  int32_T b_oq;
  int32_T m_g;
  int32_T kend_e;
  int32_T k_in;
  int32_T n_e;
  int32_T boffset_is;
  int32_T b_j_m5;
  int32_T loop_ub_dl;
  int32_T Td_tmp_j;
  int32_T n_pe;
  int32_T boffset_b;
  int32_T b_j_p;
  int32_T loop_ub_nm;
  int32_T Td_tmp_c;
  int32_T n_n;
  int32_T boffset_d;
  int32_T b_j_ih;
  int32_T loop_ub_no;
  int32_T Td_tmp_b;
  int32_T n_bt;
  int32_T boffset_j;
  int32_T b_j_n;
  int32_T loop_ub_f;
  int32_T Td_tmp_e;
  uint32_T r;
  uint32_T mti;
  uint32_T y_in;
  uint32_T r_a;
  c_robotics_core_internal_NLPS_T exitFlag;
  c_robotics_core_internal_NLPS_T exitFlagPrev;
  boolean_T b_varargout_1;
  boolean_T y_fd;
  boolean_T flag;
  boolean_T nextBodyIsParent;
  boolean_T b_bool;
  boolean_T b_bool_k;
  boolean_T b_bool_c;
  boolean_T flag_j;
  boolean_T b_bool_l;
  boolean_T b_bool_a;
  boolean_T b_bool_i;
  boolean_T b_bool_o;
  boolean_T b_bool_b;
  boolean_T b_bool_an;
  boolean_T b_bool_aq;
  boolean_T nextBodyIsParent_i;
  boolean_T b_bool_iz;
  boolean_T nextBodyIsParent_l;
  boolean_T b_bool_o5;
  boolean_T nextBodyIsParent_p;
  boolean_T b_bool_om;
  boolean_T nextBodyIsParent_c;
  boolean_T b_bool_oc;
  boolean_T nextBodyIsParent_o;
  boolean_T b_bool_h;
  boolean_T b_bool_ih;
  boolean_T b_bool_g;
  boolean_T b_bool_c1;
  boolean_T matched;
  boolean_T b_bool_ok;
} B_cartesian_trajectory_planne_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  robotics_slmanip_internal__as_T obj; // '<S5>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_88;// '<S5>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_89;// '<S5>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_90;// '<S5>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_91;// '<S5>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_92;// '<S5>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_93;// '<S5>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_94;// '<S5>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_95;// '<S5>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_96;// '<S5>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_97;// '<S5>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_98;// '<S5>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_99;// '<S5>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_100;// '<S5>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_101;// '<S5>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_102;// '<S5>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_103;// '<S5>/MATLAB System'
  robotics_slmanip_internal_blo_T obj_g;// '<S3>/MATLAB System'
  f_robotics_manip_internal_IKE_T gobj_85;// '<S5>/MATLAB System'
  robotics_slmanip_internal_b_a_T obj_f;// '<S4>/MATLAB System'
  x_robotics_manip_internal_Rig_T gobj_83;// '<S5>/MATLAB System'
  x_robotics_manip_internal_Rig_T gobj_84;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_1;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_2;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_3;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_4;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_5;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_6;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_7;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_8;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_9;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_10;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_11;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_12;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_13;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_14;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_15;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_16;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_17;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_18;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_19;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_20;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_21;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_22;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_23;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_24;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_25;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_26;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_27;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_28;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_29;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_30;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_31;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_32;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_33;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_34;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_35;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_36;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_37;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_38;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_39;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_40;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_41;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_42;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_43;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_44;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_45;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_46;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_47;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_48;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_49;// '<S5>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_50;// '<S5>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_1_h;// '<S3>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_2_m;// '<S3>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_3_l;// '<S3>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_4_j;// '<S3>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_5_g;// '<S3>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_6_a;// '<S3>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_7_b;// '<S3>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_8_d;// '<S3>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_9_g;// '<S3>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_10_f;// '<S3>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_11_i;// '<S3>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_12_b;// '<S3>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_13_a;// '<S3>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_14_h;// '<S3>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_15_g;// '<S3>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_16_c;// '<S3>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_1_f;// '<S4>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_2_a;// '<S4>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_3_i;// '<S4>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_4_f;// '<S4>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_5_n;// '<S4>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_6_j;// '<S4>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_7_n;// '<S4>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_8_l;// '<S4>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_9_l;// '<S4>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_10_n;// '<S4>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_11_c;// '<S4>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_12_e;// '<S4>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_13_p;// '<S4>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_14_hu;// '<S4>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_15_l;// '<S4>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_16_k;// '<S4>/MATLAB System'
  h_robotics_core_internal_Damp_T gobj_86;// '<S5>/MATLAB System'
  h_robotics_core_internal_Damp_T gobj_87;// '<S5>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_51;// '<S5>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_52;// '<S5>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_53;// '<S5>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_54;// '<S5>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_55;// '<S5>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_56;// '<S5>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_57;// '<S5>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_58;// '<S5>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_59;// '<S5>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_60;// '<S5>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_61;// '<S5>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_62;// '<S5>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_63;// '<S5>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_64;// '<S5>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_65;// '<S5>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_66;// '<S5>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_67;// '<S5>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_68;// '<S5>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_69;// '<S5>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_70;// '<S5>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_71;// '<S5>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_72;// '<S5>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_73;// '<S5>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_74;// '<S5>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_75;// '<S5>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_76;// '<S5>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_77;// '<S5>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_78;// '<S5>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_79;// '<S5>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_80;// '<S5>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_81;// '<S5>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_82;// '<S5>/MATLAB System'
  ros_slros_internal_block_GetP_T obj_e;// '<S11>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_d;// '<S11>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_m;// '<S11>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_h;// '<S11>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_mf;// '<S11>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_o;// '<S11>/Get Parameter5'
  ros_slros_internal_block_GetP_T obj_e0;// '<S15>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_a;// '<S15>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_p;// '<S15>/Get Parameter5'
  ros_slros_internal_block_GetP_T obj_l;// '<S15>/Get Parameter6'
  ros_slros_internal_block_GetP_T obj_mb;// '<S15>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_c;// '<S15>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_f2;// '<S15>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_es;// '<S14>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_f4;// '<S14>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_en;// '<S14>/Get Parameter5'
  ros_slros_internal_block_GetP_T obj_l0;// '<S14>/Get Parameter6'
  ros_slros_internal_block_GetP_T obj_i;// '<S14>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_db;// '<S14>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_oq;// '<S14>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_di;// '<S10>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_cj;// '<S10>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_cu;// '<Root>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_k;// '<Root>/Get Parameter1'
  ros_slros_internal_block_Publ_T obj_n;// '<S8>/SinkBlock'
  ros_slros_internal_block_Subs_T obj_gl;// '<S9>/SourceBlock'
  real_T Delay_DSTATE[6];              // '<Root>/Delay'
  real_T Reciprocal_DWORK4[36];        // '<Root>/Reciprocal'
  uint32_T state_m[625];               // '<S5>/MATLAB System'
} DW_cartesian_trajectory_plann_T;

// Parameters (default storage)
struct P_cartesian_trajectory_planne_T_ {
  SL_Bus_cartesian_trajectory_plan_JointTrajectoryPoint_1eqsod Constant_Value;// Computed Parameter: Constant_Value
                                                                      //  Referenced by: '<S1>/Constant'

  SL_Bus_cartesian_trajectory_planner_rosgraph_msgs_Clock Out1_Y0;// Computed Parameter: Out1_Y0
                                                                     //  Referenced by: '<S13>/Out1'

  SL_Bus_cartesian_trajectory_planner_rosgraph_msgs_Clock Constant_Value_e;// Computed Parameter: Constant_Value_e
                                                                      //  Referenced by: '<S9>/Constant'

  real_T Delay_InitialCondition[6];    // Expression: zeros(6, 1, 1)
                                          //  Referenced by: '<Root>/Delay'

};

// Real-time Model Data Structure
struct tag_RTM_cartesian_trajectory__T {
  const char_T *errorStatus;
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_cartesian_trajectory_planne_T cartesian_trajectory_planner_P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern B_cartesian_trajectory_planne_T cartesian_trajectory_planner_B;

#ifdef __cplusplus

}
#endif

// Block states (default storage)
extern DW_cartesian_trajectory_plann_T cartesian_trajectory_planner_DW;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void cartesian_trajectory_planner_initialize(void);
  extern void cartesian_trajectory_planner_step(void);
  extern void cartesian_trajectory_planner_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_cartesian_trajectory_T *const cartesian_trajectory_planner_M;

#ifdef __cplusplus

}
#endif

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<Root>/Display' : Unused code path elimination
//  Block '<Root>/Display1' : Unused code path elimination
//  Block '<Root>/Display2' : Unused code path elimination
//  Block '<Root>/Display3' : Unused code path elimination
//  Block '<Root>/Display4' : Unused code path elimination


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'cartesian_trajectory_planner'
//  '<S1>'   : 'cartesian_trajectory_planner/Blank Message'
//  '<S2>'   : 'cartesian_trajectory_planner/Completa'
//  '<S3>'   : 'cartesian_trajectory_planner/Get Jacobian'
//  '<S4>'   : 'cartesian_trajectory_planner/Get Transform'
//  '<S5>'   : 'cartesian_trajectory_planner/Inverse Kinematics'
//  '<S6>'   : 'cartesian_trajectory_planner/MATLAB Function'
//  '<S7>'   : 'cartesian_trajectory_planner/MATLAB Function1'
//  '<S8>'   : 'cartesian_trajectory_planner/Publish'
//  '<S9>'   : 'cartesian_trajectory_planner/Subscribe'
//  '<S10>'  : 'cartesian_trajectory_planner/Subsystem'
//  '<S11>'  : 'cartesian_trajectory_planner/Subsystem1'
//  '<S12>'  : 'cartesian_trajectory_planner/Traslazione '
//  '<S13>'  : 'cartesian_trajectory_planner/Subscribe/Enabled Subsystem'
//  '<S14>'  : 'cartesian_trajectory_planner/Subsystem/Subsystem'
//  '<S15>'  : 'cartesian_trajectory_planner/Subsystem/Subsystem1'

#endif                            // RTW_HEADER_cartesian_trajectory_planner_h_

//
// File trailer for generated code.
//
// [EOF]
//
