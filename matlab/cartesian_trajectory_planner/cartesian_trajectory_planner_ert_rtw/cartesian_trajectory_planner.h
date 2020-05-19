//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: cartesian_trajectory_planner.h
//
// Code generated for Simulink model 'cartesian_trajectory_planner'.
//
// Model version                  : 1.130
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Tue May 19 17:06:24 2020
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
#include "ctimefun.h"
#include "slros_initialize.h"
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
  real_T out_l[16];
  real_T tfCalc[16];
  real_T Td[16];
  real_T T_data[16];
  real_T T1[16];
  real_T Tc2p[16];
  real_T Tj[16];
  real_T T1j[16];
  real_T TJ[16];
  real_T a[16];
  real_T b[16];
  real_T a_j[16];
  real_T TJ_d[16];
  real_T a_g[16];
  real_T b_l[16];
  real_T a_d[16];
  real_T T2inv[16];
  real_T T2[16];
  real_T T1_d[16];
  real_T T[16];
  real_T Tdh[16];
  real_T c_f1[16];
  real_T a_l[16];
  real_T b_o[16];
  real_T a_b[16];
  real_T Td_n[16];
  real_T T_data_b[16];
  real_T T1_l[16];
  real_T Tc2p_h[16];
  real_T Tj_b[16];
  real_T T1j_d[16];
  real_T T1_e[16];
  real_T Tc2p_b[16];
  real_T Tj_j[16];
  real_T T1j_f[16];
  real_T T1_a[16];
  real_T Tc2p_j[16];
  real_T Tj_jz[16];
  real_T T1j_o[16];
  real_T T1_n[16];
  real_T Tc2p_i[16];
  real_T Tj_o[16];
  real_T T1j_n[16];
  real_T T1_m[16];
  real_T Tc2p_c[16];
  real_T Tj_m[16];
  real_T T1j_m[16];
  real_T obj[16];
  real_T Td_j[16];
  real_T T_data_h[16];
  real_T Td_c[16];
  real_T T_data_c[16];
  real_T Td_p[16];
  real_T T_data_p[16];
  real_T Td_a[16];
  real_T T_data_e[16];
  f_cell_wrap_cartesian_traject_T expl_temp;
  real_T poslim_data[12];
  real_T poslim_data_a[12];
  real_T poslim_data_as[12];
  real_T poslim_data_i[12];
  real_T R[9];
  real_T tempR[9];
  real_T out_lt[9];
  real_T T_o[9];
  real_T Td_o[9];
  real_T R_i[9];
  real_T R_f[9];
  real_T R_iz[9];
  real_T tempR_f[9];
  real_T R_g[9];
  real_T tempR_c[9];
  real_T R_o[9];
  real_T R_l[9];
  real_T R_m[9];
  real_T tempR_m[9];
  real_T T_c[9];
  real_T Td_f[9];
  real_T V_p[9];
  real_T b_I[9];
  real_T b_U[9];
  real_T A[9];
  real_T A_e[9];
  real_T R_o4[9];
  real_T R_h[9];
  real_T R_l5[9];
  real_T R_h2[9];
  real_T R_me[9];
  real_T R_mc[9];
  real_T R_h3[9];
  real_T R_c[9];
  real_T R_k[9];
  real_T R_p[9];
  real_T T_p[9];
  real_T Td_p4[9];
  real_T T_a[9];
  real_T Td_jd[9];
  real_T T_e[9];
  real_T Td_ol[9];
  real_T T_b[9];
  real_T Td_ao[9];
  real_T b_varargout_2[6];
  real_T vCalc[6];
  real_T value[6];
  real_T dv1[6];
  real_T qvSolRaw[6];
  real_T c_xSol[6];
  real_T x[6];
  real_T Hg[6];
  real_T sNew_g[6];
  real_T e[6];
  real_T y[6];
  real_T qv_data[6];
  real_T x_e[6];
  real_T Hg_f[6];
  real_T sNew_h[6];
  real_T e_e[6];
  real_T y_c[6];
  real_T qv_data_a[6];
  real_T qv_data_d[6];
  real_T qv_data_af[6];
  real_T qv_data_p[6];
  real_T qv_data_m[6];
  real_T e_o[6];
  real_T y_n[6];
  real_T e_l[6];
  real_T y_p[6];
  real_T e_p[6];
  real_T y_f[6];
  real_T e_i[6];
  real_T y_o[6];
  real_T unusedExpr[5];
  real_T unusedExpr_k[5];
  int8_T msubspace_data[36];
  int8_T msubspace_data_i[36];
  int8_T msubspace_data_o[36];
  int8_T msubspace_data_m[36];
  real_T v[4];
  real_T result_data[4];
  real_T result_data_c[4];
  real_T v_f[4];
  real_T v_h[4];
  real_T v_m[4];
  real_T v_a[4];
  real_T v_k[4];
  real_T w[3];
  real_T a_p[3];
  real_T b_f1[3];
  real_T tfCalc_tmp[3];
  real_T v_b[3];
  real_T v_c[3];
  real_T R_n[3];
  real_T v_i[3];
  real_T v_my[3];
  real_T vspecial_data[3];
  real_T s[3];
  real_T e_j[3];
  real_T work[3];
  int32_T e_data[6];
  char_T cv[18];
  SL_Bus_cartesian_trajectory_planner_rosgraph_msgs_Clock In1;// '<S10>/In1'
  SL_Bus_cartesian_trajectory_planner_rosgraph_msgs_Clock b_varargout_2_m;
  real_T signal1[2];
  real_T ub[2];
  creal_T v_e;
  creal_T u;
  creal_T u_m;
  creal_T dc;
  char_T sol_Status_data[14];
  char_T cv1[13];
  char_T cv2[11];
  char_T a_jg[11];
  char_T b_f[9];
  char_T b_a[9];
  char_T b_g[9];
  char_T b_n[9];
  char_T b_d[9];
  char_T partial_match_data[9];
  char_T b_vstr[9];
  char_T b_na[9];
  char_T b_c[8];
  char_T b_fx[8];
  char_T b_p[8];
  char_T b_p2[8];
  char_T b_nj[8];
  int8_T chainmask[8];
  char_T b_k[8];
  char_T b_n3[8];
  char_T b_gg[8];
  char_T vstr[8];
  char_T b_cq[8];
  real_T value_m;
  real_T value_p;
  real_T value_d;
  real_T value_g;
  real_T value_c;
  real_T value_cx;
  real_T value_i;
  real_T b_dx;
  real_T time;                         // '<Root>/MATLAB Function1'
  real_T tempR_tmp;
  real_T tempR_tmp_g;
  real_T tempR_tmp_l;
  real_T tempR_tmp_f;
  real_T tempR_tmp_d;
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
  real_T s_j;
  real_T A_i;
  real_T sigma_h;
  real_T s_n;
  real_T bid1;
  real_T bid2;
  real_T qidx_idx_1;
  real_T cth;
  real_T sth;
  real_T tempR_tmp_o;
  real_T tempR_tmp_c;
  real_T tempR_tmp_b;
  real_T tempR_tmp_e;
  real_T tempR_tmp_dd;
  real_T cost_i;
  real_T b_gamma_g;
  real_T beta_n;
  real_T sigma_l;
  real_T costNew_c;
  real_T m_n;
  real_T s_p;
  real_T A_d;
  real_T sigma_o;
  real_T b_j;
  real_T axang_idx_0;
  real_T axang_idx_1;
  real_T axang_idx_2;
  real_T bid_c;
  real_T b_index;
  real_T x_h;
  real_T d_u;
  real_T endeffectorIndex;
  real_T s_d;
  real_T idx_idx_1;
  real_T n;
  real_T k;
  real_T sth_c;
  real_T tempR_tmp_p;
  real_T tempR_tmp_pi;
  real_T tempR_tmp_a;
  real_T tempR_tmp_ow;
  real_T s_jw;
  real_T a_pi;
  real_T q;
  real_T nrm;
  real_T rt;
  real_T ztest;
  real_T smm1;
  real_T emm1;
  real_T sqds;
  real_T b_ob;
  real_T unusedU2;
  real_T d_sn;
  real_T roe;
  real_T absa;
  real_T absb;
  real_T scale;
  real_T ads;
  real_T bds;
  real_T bid1_l;
  real_T bid2_k;
  real_T qidx_idx_1_j;
  real_T bid1_f;
  real_T bid2_c;
  real_T qidx_idx_1_f;
  real_T bid1_n;
  real_T bid2_i;
  real_T qidx_idx_1_l;
  real_T bid1_i;
  real_T bid2_k1;
  real_T qidx_idx_1_fc;
  real_T bid1_a;
  real_T bid2_d;
  real_T qidx_idx_1_e;
  real_T ssq;
  real_T c;
  real_T bid_e;
  real_T b_index_b;
  real_T pid;
  real_T b_index_a;
  real_T obj_idx_0;
  real_T obj_idx_1;
  real_T obj_idx_2;
  real_T tol_i;
  real_T scale_f;
  real_T absxk;
  real_T t;
  real_T smax;
  real_T scale_j;
  real_T absxk_o;
  real_T t_f;
  real_T s_o;
  real_T s_l;
  real_T s_lu;
  real_T s_g;
  uint32_T b_u[2];
  uint32_T u32[2];
  uint32_T b_u_d[2];
  int32_T sol_Status_size[2];
  int32_T T_size[2];
  int32_T T_size_o[2];
  int32_T T_size_c[2];
  int32_T T_size_m[2];
  int32_T T_size_j[2];
  int32_T T_size_k[2];
  char_T cv3[7];
  char_T cv4[6];
  int8_T iv[6];
  int8_T iv1[6];
  int8_T iv2[6];
  int8_T iv3[6];
  char_T b_dv[5];
  char_T b_jo[5];
  char_T b_f1s[5];
  char_T b_js[5];
  char_T b_h[5];
  char_T b_cf[5];
  char_T b_nu[5];
  char_T b_ki[5];
  char_T b_av[5];
  char_T b_fn[5];
  char_T b_jz[5];
  char_T b_kl[5];
  char_T b_b[5];
  char_T c_vstr[5];
  char_T b_hm[5];
  char_T cv5[4];
  int32_T b_k_e;
  int32_T subsa_idx_1;
  int32_T out_tmp;
  int32_T out_tmp_h;
  int32_T c_k;
  int32_T partialTrueCount;
  int32_T nm1d2;
  int32_T k_j;
  int32_T c_o;
  int32_T i;
  int32_T loop_ub;
  int32_T c_c;
  int32_T ix;
  int32_T nx;
  int32_T i_h;
  int32_T unnamed_idx_1;
  int32_T idxl;
  int32_T j;
  int32_T nx_i;
  int32_T m_p;
  int32_T inner;
  int32_T n_f;
  int32_T idx;
  int32_T b_i;
  int32_T coffset;
  int32_T boffset;
  int32_T aoffset;
  int32_T i_e;
  int32_T g_idx_0;
  int32_T n_n;
  int32_T boffset_h;
  int32_T b_j_h;
  int32_T loop_ub_f;
  int32_T Td_tmp;
  int32_T jointSign;
  int32_T g;
  int32_T j_i;
  int32_T b_i_f;
  int32_T b_kstr;
  int32_T n_c;
  int32_T loop_ub_n;
  int32_T qv_size;
  int32_T coffset_tmp;
  int32_T b_kstr_h;
  int32_T loop_ub_k;
  int32_T a_tmp;
  int32_T a_tmp_tmp;
  int32_T b_kstr_hz;
  int32_T loop_ub_b;
  int32_T newNumel;
  int32_T i_o;
  int32_T idxl_n;
  int32_T j_m;
  int32_T nx_k;
  int32_T m_j;
  int32_T inner_h;
  int32_T n_f2;
  int32_T idx_d;
  int32_T b_i_l;
  int32_T coffset_k;
  int32_T boffset_i;
  int32_T aoffset_h;
  int32_T i_m;
  int32_T g_idx_0_g;
  int32_T b_kstr_l;
  int32_T loop_ub_m;
  int32_T a_tmp_n;
  int32_T a_tmp_tmp_g;
  int32_T i_d;
  int32_T i_mq;
  int32_T b_i_ft;
  int32_T b_kstr_g;
  int32_T loop_ub_j;
  int32_T b_kstr_c;
  int32_T loop_ub_e;
  int32_T b_kstr_m;
  int32_T loop_ub_o;
  int32_T b_k_a;
  int32_T b_mti;
  int32_T b_j_j;
  int32_T b_kk;
  int32_T d;
  int32_T b_k_g;
  int32_T newNumel_j;
  int32_T i_ee;
  int32_T d_j;
  int32_T b_k_j;
  int32_T i_g;
  int32_T b_mti_o;
  int32_T k_h;
  int32_T c_c2;
  int32_T b_i_a;
  int32_T kstr;
  int32_T b_kstr_lu;
  int32_T n_j;
  int32_T loop_ub_i;
  int32_T coffset_tmp_m;
  int32_T d_f;
  int32_T e_on;
  int32_T ntilecols;
  int32_T b_jtilecol;
  int32_T b_kstr_i;
  int32_T loop_ub_e0;
  int32_T b_kstr_j;
  int32_T loop_ub_o4;
  int32_T n_fr;
  int32_T boffset_m;
  int32_T b_j_a;
  int32_T loop_ub_h;
  int32_T Td_tmp_o;
  int32_T b_k_h;
  int32_T loop_ub_j3;
  int32_T jointSign_g;
  int32_T g_j;
  int32_T j_l;
  int32_T b_i_k;
  int32_T b_kstr_d;
  int32_T n_np;
  int32_T loop_ub_j5;
  int32_T qv_size_a;
  int32_T coffset_tmp_h;
  int32_T jointSign_i;
  int32_T g_d;
  int32_T j_b;
  int32_T b_i_h;
  int32_T b_kstr_p;
  int32_T n_nk;
  int32_T loop_ub_jz;
  int32_T qv_size_o;
  int32_T coffset_tmp_b;
  int32_T jointSign_j;
  int32_T g_e;
  int32_T j_i4;
  int32_T b_i_n;
  int32_T b_kstr_ie;
  int32_T n_p;
  int32_T loop_ub_og;
  int32_T qv_size_m;
  int32_T coffset_tmp_o;
  int32_T jointSign_gz;
  int32_T g_ez;
  int32_T j_iz;
  int32_T b_i_g;
  int32_T b_kstr_ge;
  int32_T n_g;
  int32_T loop_ub_g;
  int32_T qv_size_c;
  int32_T coffset_tmp_k;
  int32_T jointSign_d;
  int32_T g_k;
  int32_T j_p;
  int32_T b_i_p;
  int32_T b_kstr_m4;
  int32_T n_k;
  int32_T loop_ub_a;
  int32_T qv_size_f;
  int32_T coffset_tmp_c;
  int32_T b_info;
  int32_T jm1;
  int32_T idxAjj;
  int32_T b_j_jk;
  int32_T d_k;
  int32_T ia;
  int32_T ix_h;
  int32_T iy;
  int32_T k_d;
  int32_T c_A_size_idx_0;
  int32_T c_A_size_idx_1;
  int32_T i_j;
  int32_T b_i_np;
  int32_T b_kstr_j0;
  int32_T loop_ub_l;
  int32_T b_kstr_ph;
  int32_T b_i_po;
  int32_T loop_ub_ly;
  int32_T b_kstr_ld;
  int32_T loop_ub_hb;
  int32_T nmatched;
  int32_T minnanb;
  int32_T kstr_c;
  int32_T loop_ub_g0;
  int32_T partial_match_size_idx_1;
  int32_T rankR;
  int32_T na;
  int32_T minmn;
  int32_T nb;
  int32_T minmana;
  int32_T m_e;
  int32_T nb_n;
  int32_T mn;
  int32_T b_i_fn;
  int32_T ma;
  int32_T minmn_n;
  int32_T ii;
  int32_T nmi;
  int32_T mmi;
  int32_T pvt;
  int32_T itemp;
  int32_T lastc;
  int32_T kend;
  int32_T ix_e;
  int32_T iy_b;
  int32_T d_a;
  int32_T b_i2;
  int32_T m_nq;
  int32_T kend_f;
  int32_T k_i;
  int32_T n_k3;
  int32_T boffset_b;
  int32_T b_j_d;
  int32_T loop_ub_hs;
  int32_T Td_tmp_n;
  int32_T n_fj;
  int32_T boffset_a;
  int32_T b_j_m;
  int32_T loop_ub_gu;
  int32_T Td_tmp_nh;
  int32_T n_ch;
  int32_T boffset_d;
  int32_T b_j_k;
  int32_T loop_ub_c;
  int32_T Td_tmp_j;
  int32_T n_m;
  int32_T boffset_ie;
  int32_T b_j_b;
  int32_T loop_ub_oq;
  int32_T Td_tmp_g;
  uint32_T r;
  uint32_T mti;
  uint32_T y_e;
  uint32_T r_i;
  c_robotics_core_internal_NLPS_T exitFlag;
  c_robotics_core_internal_NLPS_T exitFlagPrev;
  boolean_T b_varargout_1;
  boolean_T y_eb;
  boolean_T flag;
  boolean_T nextBodyIsParent;
  boolean_T b_bool;
  boolean_T b_bool_i;
  boolean_T b_bool_m;
  boolean_T flag_d;
  boolean_T b_bool_j;
  boolean_T b_bool_p;
  boolean_T b_bool_b;
  boolean_T b_bool_pn;
  boolean_T b_bool_n;
  boolean_T b_bool_c;
  boolean_T b_bool_nh;
  boolean_T nextBodyIsParent_d;
  boolean_T b_bool_ih;
  boolean_T nextBodyIsParent_n;
  boolean_T b_bool_bi;
  boolean_T nextBodyIsParent_b;
  boolean_T b_bool_jk;
  boolean_T nextBodyIsParent_n0;
  boolean_T b_bool_f;
  boolean_T nextBodyIsParent_e;
  boolean_T b_bool_in;
  boolean_T b_bool_a;
  boolean_T b_bool_fd;
  boolean_T b_bool_k;
  boolean_T matched;
  boolean_T b_bool_cg;
} B_cartesian_trajectory_planne_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  robotics_slmanip_internal_b_a_T obj; // '<S3>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_88;// '<S3>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_89;// '<S3>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_90;// '<S3>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_91;// '<S3>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_92;// '<S3>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_93;// '<S3>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_94;// '<S3>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_95;// '<S3>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_96;// '<S3>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_97;// '<S3>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_98;// '<S3>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_99;// '<S3>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_100;// '<S3>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_101;// '<S3>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_102;// '<S3>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_103;// '<S3>/MATLAB System'
  robotics_slmanip_internal_blo_T obj_o;// '<S2>/MATLAB System'
  f_robotics_manip_internal_IKE_T gobj_85;// '<S3>/MATLAB System'
  robotics_slcore_internal_bl_a_T obj_e;// '<Root>/Transform Trajectory'
  x_robotics_manip_internal_Rig_T gobj_83;// '<S3>/MATLAB System'
  x_robotics_manip_internal_Rig_T gobj_84;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_1;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_2;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_3;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_4;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_5;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_6;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_7;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_8;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_9;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_10;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_11;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_12;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_13;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_14;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_15;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_16;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_17;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_18;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_19;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_20;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_21;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_22;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_23;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_24;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_25;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_26;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_27;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_28;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_29;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_30;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_31;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_32;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_33;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_34;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_35;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_36;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_37;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_38;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_39;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_40;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_41;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_42;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_43;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_44;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_45;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_46;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_47;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_48;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_49;// '<S3>/MATLAB System'
  c_rigidBodyJoint_cartesian__a_T gobj_50;// '<S3>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_1_n;// '<S2>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_2_d;// '<S2>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_3_m;// '<S2>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_4_p;// '<S2>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_5_c;// '<S2>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_6_m;// '<S2>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_7_a;// '<S2>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_8_h;// '<S2>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_9_a;// '<S2>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_10_b;// '<S2>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_11_h;// '<S2>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_12_j;// '<S2>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_13_m;// '<S2>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_14_i;// '<S2>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_15_h;// '<S2>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_16_d;// '<S2>/MATLAB System'
  h_robotics_core_internal_Damp_T gobj_86;// '<S3>/MATLAB System'
  h_robotics_core_internal_Damp_T gobj_87;// '<S3>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_51;// '<S3>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_52;// '<S3>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_53;// '<S3>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_54;// '<S3>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_55;// '<S3>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_56;// '<S3>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_57;// '<S3>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_58;// '<S3>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_59;// '<S3>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_60;// '<S3>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_61;// '<S3>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_62;// '<S3>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_63;// '<S3>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_64;// '<S3>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_65;// '<S3>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_66;// '<S3>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_67;// '<S3>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_68;// '<S3>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_69;// '<S3>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_70;// '<S3>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_71;// '<S3>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_72;// '<S3>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_73;// '<S3>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_74;// '<S3>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_75;// '<S3>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_76;// '<S3>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_77;// '<S3>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_78;// '<S3>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_79;// '<S3>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_80;// '<S3>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_81;// '<S3>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_82;// '<S3>/MATLAB System'
  ros_slros_internal_block_GetP_T obj_c;// '<S9>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_f;// '<S9>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_e0;// '<S9>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_a;// '<S9>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_p;// '<S9>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_l;// '<S9>/Get Parameter5'
  ros_slros_internal_block_GetP_T obj_f4;// '<S12>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_en;// '<S12>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_l0;// '<S12>/Get Parameter5'
  ros_slros_internal_block_GetP_T obj_n;// '<S12>/Get Parameter6'
  ros_slros_internal_block_GetP_T obj_d;// '<S12>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_oq;// '<S12>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_es;// '<S12>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_g;// '<S11>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_or;// '<S11>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_dz;// '<S11>/Get Parameter5'
  ros_slros_internal_block_GetP_T obj_di;// '<S11>/Get Parameter6'
  ros_slros_internal_block_GetP_T obj_b;// '<S11>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_e0t;// '<S11>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_pd;// '<S11>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_k;// '<S8>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_ky;// '<S8>/Get Parameter1'
  ros_slros_internal_block_Publ_T obj_kh;// '<S6>/SinkBlock'
  ros_slros_internal_block_Subs_T obj_j;// '<S7>/SourceBlock'
  real_T Delay_DSTATE[6];              // '<Root>/Delay'
  real_T Reciprocal_DWORK4[36];        // '<Root>/Reciprocal'
  uint32_T state_b[625];               // '<S3>/MATLAB System'
} DW_cartesian_trajectory_plann_T;

// Parameters (default storage)
struct P_cartesian_trajectory_planne_T_ {
  SL_Bus_cartesian_trajectory_plan_JointTrajectoryPoint_1eqsod Constant_Value;// Computed Parameter: Constant_Value
                                                                      //  Referenced by: '<S1>/Constant'

  SL_Bus_cartesian_trajectory_planner_rosgraph_msgs_Clock Out1_Y0;// Computed Parameter: Out1_Y0
                                                                     //  Referenced by: '<S10>/Out1'

  SL_Bus_cartesian_trajectory_planner_rosgraph_msgs_Clock Constant_Value_p;// Computed Parameter: Constant_Value_p
                                                                      //  Referenced by: '<S7>/Constant'

  real_T Delay_InitialCondition[6];    // Expression: [0; 0; 0; 0; 0; 0]
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
//  '<S2>'   : 'cartesian_trajectory_planner/Get Jacobian'
//  '<S3>'   : 'cartesian_trajectory_planner/Inverse Kinematics'
//  '<S4>'   : 'cartesian_trajectory_planner/MATLAB Function'
//  '<S5>'   : 'cartesian_trajectory_planner/MATLAB Function1'
//  '<S6>'   : 'cartesian_trajectory_planner/Publish'
//  '<S7>'   : 'cartesian_trajectory_planner/Subscribe'
//  '<S8>'   : 'cartesian_trajectory_planner/Subsystem'
//  '<S9>'   : 'cartesian_trajectory_planner/Subsystem1'
//  '<S10>'  : 'cartesian_trajectory_planner/Subscribe/Enabled Subsystem'
//  '<S11>'  : 'cartesian_trajectory_planner/Subsystem/Subsystem'
//  '<S12>'  : 'cartesian_trajectory_planner/Subsystem/Subsystem1'

#endif                            // RTW_HEADER_cartesian_trajectory_planner_h_

//
// File trailer for generated code.
//
// [EOF]
//
