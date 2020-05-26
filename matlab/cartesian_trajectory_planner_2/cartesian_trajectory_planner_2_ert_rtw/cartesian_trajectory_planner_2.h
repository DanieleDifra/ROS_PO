//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: cartesian_trajectory_planner_2.h
//
// Code generated for Simulink model 'cartesian_trajectory_planner_2'.
//
// Model version                  : 1.139
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Mon May 25 16:52:18 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_cartesian_trajectory_planner_2_h_
#define RTW_HEADER_cartesian_trajectory_planner_2_h_
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
#include "cartesian_trajectory_planner_2_types.h"
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

#define cartesian_trajectory_planner_2_M (cartesian_trajectory_planner_M)

// Block signals for system '<S2>/MATLAB System'
typedef struct {
  uint32_T uv[625];
  uint32_T uv1[625];
  real_T xi[257];
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
  real_T c_A_data[36];
  real_T u0[16];
  real_T Td[16];
  real_T T_data[16];
  real_T T1[16];
  real_T Tc2p[16];
  real_T Tj[16];
  real_T T1j[16];
  real_T TJ[16];
  real_T a[16];
  real_T b[16];
  real_T a_p[16];
  real_T TJ_l[16];
  real_T a_j[16];
  real_T b_d[16];
  real_T a_g[16];
  real_T Td_l[16];
  real_T T_data_d[16];
  real_T T1_d[16];
  real_T Tc2p_l[16];
  real_T Tj_o[16];
  real_T T1j_b[16];
  real_T T1_n[16];
  real_T Tc2p_b[16];
  real_T Tj_l[16];
  real_T T1j_h[16];
  real_T T1_b[16];
  real_T Tc2p_d[16];
  real_T Tj_e[16];
  real_T T1j_bj[16];
  real_T T1_j[16];
  real_T Tc2p_f[16];
  real_T Tj_a[16];
  real_T T1j_j[16];
  real_T T1_jz[16];
  real_T Tc2p_o[16];
  real_T Tj_n[16];
  real_T T1j_i[16];
  real_T obj[16];
  real_T Td_o[16];
  real_T T_data_n[16];
  real_T Td_m[16];
  real_T T_data_c[16];
  real_T Td_md[16];
  real_T T_data_m[16];
  real_T Td_j[16];
  real_T T_data_h[16];
  real_T poslim_data[12];
  real_T poslim_data_c[12];
  real_T poslim_data_ct[12];
  real_T poslim_data_p[12];
  real_T T[9];
  real_T Td_p[9];
  real_T R[9];
  real_T R_a[9];
  real_T R_e[9];
  real_T tempR[9];
  real_T R_ax[9];
  real_T tempR_a[9];
  real_T T_i[9];
  real_T Td_lt[9];
  real_T V_o[9];
  real_T b_I[9];
  real_T b_U[9];
  real_T A[9];
  real_T A_o[9];
  real_T R_i[9];
  real_T R_f[9];
  real_T R_iz[9];
  real_T R_ff[9];
  real_T R_g[9];
  real_T R_c[9];
  real_T R_o[9];
  real_T R_l[9];
  real_T R_m[9];
  real_T R_mj[9];
  real_T T_c[9];
  real_T Td_f[9];
  real_T T_p[9];
  real_T Td_e[9];
  real_T T_o[9];
  real_T Td_h[9];
  real_T T_l[9];
  real_T Td_h2[9];
  real_T u1[6];
  real_T u2[6];
  real_T qvSolRaw[6];
  real_T c_xSol[6];
  real_T x[6];
  real_T Hg[6];
  real_T sNew_m[6];
  real_T e[6];
  real_T y[6];
  real_T qv_data[6];
  real_T x_m[6];
  real_T Hg_h[6];
  real_T sNew_cs[6];
  real_T e_k[6];
  real_T y_p[6];
  real_T qv_data_p[6];
  real_T qv_data_p4[6];
  real_T qv_data_a[6];
  real_T qv_data_j[6];
  real_T qv_data_e[6];
  real_T e_o[6];
  real_T y_b[6];
  real_T e_a[6];
  real_T y_g[6];
  real_T e_e[6];
  real_T y_f[6];
  real_T e_h[6];
  real_T y_e[6];
  real_T unusedExpr[5];
  real_T unusedExpr_c[5];
  int8_T msubspace_data[36];
  int8_T msubspace_data_a[36];
  int8_T msubspace_data_d[36];
  int8_T msubspace_data_af[36];
  real_T v[4];
  real_T result_data[4];
  real_T v_p[4];
  real_T v_m[4];
  real_T v_o[4];
  real_T v_n[4];
  real_T v_l[4];
  ikblock_info_bus MATLABSystem_o2;    // '<S2>/MATLAB System'
  real_T MATLABSystem_o1[6];           // '<S2>/MATLAB System'
  real_T v_pe[3];
  real_T v_pt[3];
  real_T v_f[3];
  real_T vspecial_data[3];
  real_T s[3];
  real_T e_i[3];
  real_T work[3];
  creal_T v_ox;
  creal_T u;
  creal_T u_k;
  creal_T dc;
  real_T ub[2];
  char_T b_varargout_2_Status_data[14];
  char_T b_i[9];
  char_T b_o[9];
  char_T b_m[9];
  char_T b_c[9];
  char_T partial_match_data[9];
  char_T b_vstr[9];
  char_T b_f[9];
  char_T b_h[8];
  char_T b_m4[8];
  char_T b_a[8];
  char_T b_k[8];
  char_T b_p[8];
  char_T b_ch[8];
  char_T vstr[8];
  char_T b_n[8];
  real_T b_varargout_2_ExitFlag;
  real_T bid;
  real_T numPositions;
  real_T ndbl;
  real_T apnd;
  real_T cdiff;
  real_T u0_m;
  real_T u1_m;
  real_T d;
  real_T d1;
  real_T d2;
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
  real_T A_f;
  real_T sigma_a;
  real_T s_g;
  real_T bid1;
  real_T bid2;
  real_T qidx_idx_1;
  real_T cth;
  real_T sth;
  real_T tempR_tmp;
  real_T tempR_tmp_n;
  real_T tempR_tmp_d;
  real_T tempR_tmp_na;
  real_T tempR_tmp_c;
  real_T cost_f;
  real_T b_gamma_p;
  real_T beta_p;
  real_T sigma_n;
  real_T costNew_k;
  real_T m_n;
  real_T s_o;
  real_T A_g;
  real_T sigma_c;
  real_T b_cj;
  real_T axang_idx_0;
  real_T axang_idx_1;
  real_T axang_idx_2;
  real_T bid_m;
  real_T b_index;
  real_T x_j;
  real_T d_u;
  real_T s_k;
  real_T a_m;
  real_T q;
  real_T nrm;
  real_T rt;
  real_T ztest;
  real_T smm1;
  real_T emm1;
  real_T sqds;
  real_T b_pr;
  real_T unusedU2;
  real_T d_sn;
  real_T roe;
  real_T absa;
  real_T absb;
  real_T scale;
  real_T ads;
  real_T bds;
  real_T bid1_d;
  real_T bid2_g;
  real_T qidx_idx_1_c;
  real_T bid1_c;
  real_T bid2_i;
  real_T qidx_idx_1_d;
  real_T bid1_g;
  real_T bid2_l;
  real_T qidx_idx_1_f;
  real_T bid1_db;
  real_T bid2_j;
  real_T qidx_idx_1_i;
  real_T bid1_h;
  real_T bid2_n;
  real_T qidx_idx_1_o;
  real_T ssq;
  real_T c;
  real_T bid_c;
  real_T b_index_b;
  real_T pid;
  real_T b_index_e;
  real_T obj_idx_0;
  real_T obj_idx_1;
  real_T obj_idx_2;
  real_T s_d;
  real_T s_i;
  real_T s_gs;
  real_T s_n;
  real_T tol_l;
  real_T scale_c;
  real_T absxk;
  real_T t;
  real_T smax;
  real_T scale_n;
  real_T absxk_p;
  real_T t_d;
  uint32_T b_u[2];
  uint32_T u32[2];
  uint32_T b_u_o[2];
  int32_T b_varargout_2_Status_size[2];
  int32_T T_size[2];
  int32_T T_size_b[2];
  int32_T T_size_i[2];
  int32_T T_size_m[2];
  int32_T T_size_j[2];
  int32_T T_size_e[2];
  char_T b_j[7];
  int8_T iv[6];
  int8_T iv1[6];
  int8_T iv2[6];
  int8_T iv3[6];
  char_T b_c2[5];
  char_T b_hp[5];
  char_T b_da[5];
  char_T b_c5[5];
  char_T b_pw[5];
  char_T b_pi[5];
  char_T b_a4[5];
  char_T b_ow[5];
  char_T b_jw[5];
  char_T b_pie[5];
  char_T b_ob[5];
  char_T c_vstr[5];
  char_T b_l[5];
  int32_T kstr;
  int32_T i;
  int32_T c_k;
  int32_T partialTrueCount;
  int32_T nm1d2;
  int32_T k;
  int32_T c_j;
  int32_T i_f;
  int32_T loop_ub;
  int32_T c_c;
  int32_T ix;
  int32_T nx;
  int32_T i_fq;
  int32_T unnamed_idx_1;
  int32_T idxl;
  int32_T j;
  int32_T nx_n;
  int32_T m_i;
  int32_T inner;
  int32_T n;
  int32_T idx;
  int32_T b_i_l;
  int32_T coffset;
  int32_T boffset;
  int32_T aoffset;
  int32_T i_i;
  int32_T g_idx_0;
  int32_T n_k;
  int32_T boffset_f;
  int32_T b_j_a;
  int32_T loop_ub_d;
  int32_T Td_tmp;
  int32_T jointSign;
  int32_T g;
  int32_T j_e;
  int32_T b_i_e;
  int32_T b_kstr;
  int32_T n_b;
  int32_T loop_ub_a;
  int32_T qv_size;
  int32_T coffset_tmp;
  int32_T b_kstr_i;
  int32_T loop_ub_f;
  int32_T a_tmp;
  int32_T a_tmp_tmp;
  int32_T b_kstr_j;
  int32_T loop_ub_o;
  int32_T newNumel;
  int32_T i_fr;
  int32_T idxl_o;
  int32_T j_l;
  int32_T nx_l;
  int32_T m_g;
  int32_T inner_d;
  int32_T n_d;
  int32_T idx_j;
  int32_T b_i_f;
  int32_T coffset_j;
  int32_T boffset_h;
  int32_T aoffset_c;
  int32_T i_n;
  int32_T g_idx_0_k;
  int32_T b_kstr_a;
  int32_T loop_ub_fn;
  int32_T a_tmp_j;
  int32_T a_tmp_tmp_k;
  int32_T i_b;
  int32_T b_i_h;
  int32_T b_kstr_e;
  int32_T loop_ub_h;
  int32_T b_kstr_k;
  int32_T loop_ub_j;
  int32_T b_kstr_o;
  int32_T loop_ub_c;
  int32_T b_k_h;
  int32_T b_mti;
  int32_T b_j_i;
  int32_T b_kk;
  int32_T d_p;
  int32_T b_k_f;
  int32_T newNumel_e;
  int32_T i_nh;
  int32_T d_h;
  int32_T b_k_hp;
  int32_T i_fn;
  int32_T b_mti_i;
  int32_T k_f;
  int32_T n_c;
  int32_T boffset_n;
  int32_T b_j_h;
  int32_T loop_ub_k;
  int32_T Td_tmp_h;
  int32_T b_k_b;
  int32_T loop_ub_os;
  int32_T jointSign_n;
  int32_T g_m;
  int32_T j_k;
  int32_T b_i_j;
  int32_T b_kstr_h;
  int32_T n_f;
  int32_T loop_ub_dy;
  int32_T qv_size_l;
  int32_T coffset_tmp_k;
  int32_T jointSign_i;
  int32_T g_h;
  int32_T j_m;
  int32_T b_i_g;
  int32_T b_kstr_l;
  int32_T n_m;
  int32_T loop_ub_n;
  int32_T qv_size_g;
  int32_T coffset_tmp_d;
  int32_T jointSign_m;
  int32_T g_f;
  int32_T j_g;
  int32_T b_i_jc;
  int32_T b_kstr_c;
  int32_T n_e;
  int32_T loop_ub_m;
  int32_T qv_size_o;
  int32_T coffset_tmp_a;
  int32_T jointSign_j;
  int32_T g_g;
  int32_T j_j;
  int32_T b_i_ee;
  int32_T b_kstr_jy;
  int32_T n_j;
  int32_T loop_ub_g;
  int32_T qv_size_om;
  int32_T coffset_tmp_h;
  int32_T jointSign_c;
  int32_T g_a;
  int32_T j_lu;
  int32_T b_i_jcs;
  int32_T b_kstr_ib;
  int32_T n_mi;
  int32_T loop_ub_ft;
  int32_T qv_size_on;
  int32_T coffset_tmp_i;
  int32_T b_info;
  int32_T jm1;
  int32_T idxAjj;
  int32_T b_j_e;
  int32_T d_j;
  int32_T ia;
  int32_T ix_o;
  int32_T iy;
  int32_T k_fr;
  int32_T c_A_size_idx_0;
  int32_T c_A_size_idx_1;
  int32_T i_m;
  int32_T b_i_a;
  int32_T b_kstr_hi;
  int32_T loop_ub_oc;
  int32_T b_kstr_hn;
  int32_T b_i_j3;
  int32_T loop_ub_g3;
  int32_T b_kstr_jz;
  int32_T loop_ub_l;
  int32_T nmatched;
  int32_T minnanb;
  int32_T kstr_k;
  int32_T loop_ub_dr;
  int32_T partial_match_size_idx_1;
  int32_T n_n;
  int32_T boffset_j;
  int32_T b_j_a3;
  int32_T loop_ub_hz;
  int32_T Td_tmp_i;
  int32_T n_dn;
  int32_T boffset_b;
  int32_T b_j_hj;
  int32_T loop_ub_p;
  int32_T Td_tmp_n;
  int32_T n_jz;
  int32_T boffset_o;
  int32_T b_j_b;
  int32_T loop_ub_jk;
  int32_T Td_tmp_e;
  int32_T n_i;
  int32_T boffset_ng;
  int32_T b_j_ie;
  int32_T loop_ub_p3;
  int32_T Td_tmp_o;
  int32_T rankR;
  int32_T na;
  int32_T minmn;
  int32_T nb;
  int32_T minmana;
  int32_T m_m;
  int32_T nb_o;
  int32_T mn;
  int32_T b_i_gz;
  int32_T ma;
  int32_T minmn_e;
  int32_T ii;
  int32_T nmi;
  int32_T mmi;
  int32_T pvt;
  int32_T itemp;
  int32_T lastc;
  int32_T kend;
  int32_T ix_i;
  int32_T iy_g;
  int32_T d_g;
  int32_T b_g;
  int32_T m_gr;
  int32_T kend_c;
  int32_T k_k;
  uint32_T r;
  uint32_T mti;
  uint32_T y_d;
  uint32_T r_k;
  c_robotics_core_internal_NLPS_T exitFlag;
  c_robotics_core_internal_NLPS_T exitFlagPrev;
  boolean_T b_bool;
  boolean_T y_pd;
  boolean_T flag;
  boolean_T nextBodyIsParent;
  boolean_T b_bool_p;
  boolean_T b_bool_m;
  boolean_T b_bool_k;
  boolean_T flag_a;
  boolean_T b_bool_f;
  boolean_T b_bool_c;
  boolean_T b_bool_j;
  boolean_T b_bool_kb;
  boolean_T nextBodyIsParent_h;
  boolean_T b_bool_d;
  boolean_T nextBodyIsParent_j;
  boolean_T b_bool_n;
  boolean_T nextBodyIsParent_j0;
  boolean_T b_bool_l;
  boolean_T nextBodyIsParent_p;
  boolean_T b_bool_po;
  boolean_T nextBodyIsParent_l;
  boolean_T b_bool_ld;
  boolean_T b_bool_h;
  boolean_T b_bool_cg;
  boolean_T b_bool_g;
  boolean_T matched;
  boolean_T b_bool_e;
} B_MATLABSystem_cartesian_traj_T;

// Block states (default storage) for system '<S2>/MATLAB System'
typedef struct {
  robotics_slmanip_internal_blo_T obj; // '<S2>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_88;// '<S2>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_89;// '<S2>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_90;// '<S2>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_91;// '<S2>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_92;// '<S2>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_93;// '<S2>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_94;// '<S2>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_95;// '<S2>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_96;// '<S2>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_97;// '<S2>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_98;// '<S2>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_99;// '<S2>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_100;// '<S2>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_101;// '<S2>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_102;// '<S2>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_103;// '<S2>/MATLAB System'
  f_robotics_manip_internal_IKE_T gobj_85;// '<S2>/MATLAB System'
  x_robotics_manip_internal_Rig_T gobj_83;// '<S2>/MATLAB System'
  x_robotics_manip_internal_Rig_T gobj_84;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_1;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_2;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_3;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_4;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_5;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_6;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_7;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_8;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_9;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_10;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_11;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_12;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_13;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_14;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_15;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_16;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_17;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_18;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_19;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_20;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_21;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_22;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_23;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_24;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_25;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_26;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_27;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_28;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_29;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_30;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_31;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_32;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_33;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_34;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_35;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_36;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_37;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_38;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_39;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_40;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_41;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_42;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_43;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_44;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_45;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_46;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_47;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_48;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_49;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_tr_T gobj_50;// '<S2>/MATLAB System'
  h_robotics_core_internal_Damp_T gobj_86;// '<S2>/MATLAB System'
  h_robotics_core_internal_Damp_T gobj_87;// '<S2>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_51;// '<S2>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_52;// '<S2>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_53;// '<S2>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_54;// '<S2>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_55;// '<S2>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_56;// '<S2>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_57;// '<S2>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_58;// '<S2>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_59;// '<S2>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_60;// '<S2>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_61;// '<S2>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_62;// '<S2>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_63;// '<S2>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_64;// '<S2>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_65;// '<S2>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_66;// '<S2>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_67;// '<S2>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_68;// '<S2>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_69;// '<S2>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_70;// '<S2>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_71;// '<S2>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_72;// '<S2>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_73;// '<S2>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_74;// '<S2>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_75;// '<S2>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_76;// '<S2>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_77;// '<S2>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_78;// '<S2>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_79;// '<S2>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_80;// '<S2>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_81;// '<S2>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_82;// '<S2>/MATLAB System'
  uint32_T state;                      // '<S2>/MATLAB System'
  uint32_T state_a[2];                 // '<S2>/MATLAB System'
  uint32_T state_e[625];               // '<S2>/MATLAB System'
  uint32_T method;                     // '<S2>/MATLAB System'
  uint32_T method_m;                   // '<S2>/MATLAB System'
  uint32_T state_ar[2];                // '<S2>/MATLAB System'
  boolean_T objisempty;                // '<S2>/MATLAB System'
  boolean_T state_not_empty;           // '<S2>/MATLAB System'
  boolean_T state_not_empty_i;         // '<S2>/MATLAB System'
  boolean_T state_not_empty_j;         // '<S2>/MATLAB System'
  boolean_T method_not_empty;          // '<S2>/MATLAB System'
  boolean_T method_not_empty_k;        // '<S2>/MATLAB System'
  boolean_T state_not_empty_k;         // '<S2>/MATLAB System'
} DW_MATLABSystem_cartesian_tra_T;

// Block signals for system '<S13>/Coordinate Transformation Conversion'
typedef struct {
  real_T CoordinateTransformationConve_g[16];
                                // '<S13>/Coordinate Transformation Conversion'
  real_T R[9];
  real_T tempR[9];
  real_T u1[3];
  real_T b;
  real_T u0_idx_0;
  real_T u0_idx_1;
} B_CoordinateTransformationCon_T;

// Block states (default storage) for system '<S13>/Coordinate Transformation Conversion' 
typedef struct {
  robotics_slcore_internal_bloc_T obj;
                                // '<S13>/Coordinate Transformation Conversion'
  boolean_T objisempty;         // '<S13>/Coordinate Transformation Conversion'
} DW_CoordinateTransformationCo_T;

// Block signals (default storage)
typedef struct {
  SL_Bus_cartesian_trajectory_plan_JointTrajectoryPoint_na5m06 msg;// '<Root>/MATLAB Function2' 
  real_T t[18];
  real_T MatrixConcatenate[12];        // '<Root>/Matrix Concatenate'
  real_T max_vel[6];
  real_T acc[6];
  real_T act_max_vel[6];
  real_T signes[6];
  real_T qdd[6];                       // '<Root>/MATLAB Function3'
  real_T q[6];                         // '<Root>/MATLAB Function3'
  real_T qd[6];                        // '<Root>/MATLAB Function3'
  real_T Delay1[6];                    // '<Root>/Delay1'
  real_T TmpSignalConversionAtMATLAB[6];
  real_T Delay[6];                     // '<Root>/Delay'
  real_T TmpSignalConversionAtCoor_l[4];
  real_T TmpSignalConversionAtCoordi[4];
  real_T TmpSignalConversionAtCoo_pn[3];
  real_T TmpSignalConversionAtCoor_p[3];
  char_T cv[18];
  SL_Bus_cartesian_trajectory_planner_2_rosgraph_msgs_Clock In1;// '<S12>/In1'
  SL_Bus_cartesian_trajectory_planner_2_rosgraph_msgs_Clock b_varargout_2;
  char_T cv1[13];
  char_T cv2[12];
  char_T cv3[11];
  real_T t_up;
  real_T dist;
  real_T delayed_time;                 // '<Root>/MATLAB Function1'
  real_T d;
  char_T cv4[7];
  char_T cv5[6];
  int32_T k;
  int32_T i;
  int32_T i_m;
  int32_T i_c;
  boolean_T b_varargout_1;
  B_CoordinateTransformationCon_T CoordinateTransformationConv_pn;
                                // '<S13>/Coordinate Transformation Conversion'
  B_CoordinateTransformationCon_T CoordinateTransformationConve_p;
                                // '<S13>/Coordinate Transformation Conversion'
  B_MATLABSystem_cartesian_traj_T MATLABSystem_p;// '<S2>/MATLAB System'
  B_MATLABSystem_cartesian_traj_T MATLABSystem;// '<S2>/MATLAB System'
} B_cartesian_trajectory_planne_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  ros_slros_internal_block_GetP_T obj; // '<S11>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_m;// '<S11>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_c;// '<S11>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_f;// '<S11>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_e;// '<S11>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_a;// '<S11>/Get Parameter5'
  ros_slros_internal_block_GetP_T obj_o;// '<S14>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_es;// '<S14>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_f4;// '<S14>/Get Parameter5'
  ros_slros_internal_block_GetP_T obj_en;// '<S14>/Get Parameter6'
  ros_slros_internal_block_GetP_T obj_b;// '<S14>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_i;// '<S14>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_d;// '<S14>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_e0;// '<S13>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_p;// '<S13>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_g;// '<S13>/Get Parameter5'
  ros_slros_internal_block_GetP_T obj_or;// '<S13>/Get Parameter6'
  ros_slros_internal_block_GetP_T obj_a3;// '<S13>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_bh;// '<S13>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_b0;// '<S13>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_iy;// '<S9>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_cu;// '<S9>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_k;// '<S9>/Get Parameter3'
  ros_slros_internal_block_Publ_T obj_ej;// '<S7>/SinkBlock'
  ros_slros_internal_block_Subs_T obj_cs;// '<S8>/SourceBlock'
  ros_slros_internal_block_SetP_T obj_gt;// '<S10>/Set Parameter'
  ros_slros_internal_block_SetP_T obj_kg;// '<S10>/Set Parameter1'
  ros_slros_internal_block_SetP_T obj_ef;// '<S10>/Set Parameter2'
  ros_slros_internal_block_SetP_T obj_kf;// '<S10>/Set Parameter3'
  ros_slros_internal_block_SetP_T obj_gc;// '<S10>/Set Parameter4'
  ros_slros_internal_block_SetP_T obj_oe;// '<S10>/Set Parameter5'
  real_T Delay_DSTATE[6];              // '<Root>/Delay'
  real_T Delay1_DSTATE[6];             // '<Root>/Delay1'
  DW_CoordinateTransformationCo_T CoordinateTransformationConv_pn;
                                // '<S13>/Coordinate Transformation Conversion'
  DW_CoordinateTransformationCo_T CoordinateTransformationConve_p;
                                // '<S13>/Coordinate Transformation Conversion'
  DW_MATLABSystem_cartesian_tra_T MATLABSystem_p;// '<S2>/MATLAB System'
  DW_MATLABSystem_cartesian_tra_T MATLABSystem;// '<S2>/MATLAB System'
} DW_cartesian_trajectory_plann_T;

// Parameters (default storage)
struct P_cartesian_trajectory_planne_T_ {
  SL_Bus_cartesian_trajectory_plan_JointTrajectoryPoint_na5m06 Constant_Value;// Computed Parameter: Constant_Value
                                                                      //  Referenced by: '<S1>/Constant'

  SL_Bus_cartesian_trajectory_planner_2_rosgraph_msgs_Clock Out1_Y0;// Computed Parameter: Out1_Y0
                                                                      //  Referenced by: '<S12>/Out1'

  SL_Bus_cartesian_trajectory_planner_2_rosgraph_msgs_Clock Constant_Value_a;// Computed Parameter: Constant_Value_a
                                                                      //  Referenced by: '<S8>/Constant'

  real_T Delay_InitialCondition[6];    // Expression: zeros(6, 1)
                                          //  Referenced by: '<Root>/Delay'

  real_T Delay1_InitialCondition[6];   // Expression: zeros(6,1)
                                          //  Referenced by: '<Root>/Delay1'

};

// Real-time Model Data Structure
struct tag_RTM_cartesian_trajectory__T {
  const char_T *errorStatus;
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_cartesian_trajectory_planne_T cartesian_trajectory_planner__P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern B_cartesian_trajectory_planne_T cartesian_trajectory_planner__B;

#ifdef __cplusplus

}
#endif

// Block states (default storage)
extern DW_cartesian_trajectory_plann_T cartesian_trajectory_planner_DW;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void cartesian_trajectory_planner_2_initialize(void);
  extern void cartesian_trajectory_planner_2_step(void);
  extern void cartesian_trajectory_planner_2_terminate(void);

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
//  Block '<Root>/Display2' : Unused code path elimination


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
//  '<Root>' : 'cartesian_trajectory_planner_2'
//  '<S1>'   : 'cartesian_trajectory_planner_2/Blank Message'
//  '<S2>'   : 'cartesian_trajectory_planner_2/Inverse Kinematics'
//  '<S3>'   : 'cartesian_trajectory_planner_2/Inverse Kinematics1'
//  '<S4>'   : 'cartesian_trajectory_planner_2/MATLAB Function1'
//  '<S5>'   : 'cartesian_trajectory_planner_2/MATLAB Function2'
//  '<S6>'   : 'cartesian_trajectory_planner_2/MATLAB Function3'
//  '<S7>'   : 'cartesian_trajectory_planner_2/Publish'
//  '<S8>'   : 'cartesian_trajectory_planner_2/Subscribe'
//  '<S9>'   : 'cartesian_trajectory_planner_2/Subsystem'
//  '<S10>'  : 'cartesian_trajectory_planner_2/set_initial_conditions'
//  '<S11>'  : 'cartesian_trajectory_planner_2/weights'
//  '<S12>'  : 'cartesian_trajectory_planner_2/Subscribe/Enabled Subsystem'
//  '<S13>'  : 'cartesian_trajectory_planner_2/Subsystem/Subsystem'
//  '<S14>'  : 'cartesian_trajectory_planner_2/Subsystem/Subsystem1'

#endif                          // RTW_HEADER_cartesian_trajectory_planner_2_h_

//
// File trailer for generated code.
//
// [EOF]
//
