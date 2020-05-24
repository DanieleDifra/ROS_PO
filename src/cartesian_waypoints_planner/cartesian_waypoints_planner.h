//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: cartesian_waypoints_planner.h
//
// Code generated for Simulink model 'cartesian_waypoints_planner'.
//
// Model version                  : 1.169
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Sun May 24 12:36:19 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_cartesian_waypoints_planner_h_
#define RTW_HEADER_cartesian_waypoints_planner_h_
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
#include "cartesian_waypoints_planner_types.h"
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
  SL_Bus_cartesian_waypoints_plann_Float32MultiArray_iutvhs msg;// '<Root>/MATLAB Function1' 
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
  real_T out[16];
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
  real_T R[9];
  real_T tempR[9];
  real_T T[9];
  real_T Td_p[9];
  real_T R_a[9];
  real_T R_e[9];
  real_T R_ax[9];
  real_T tempR_a[9];
  real_T R_i[9];
  real_T tempR_l[9];
  real_T T_o[9];
  real_T Td_o2[9];
  real_T V_i[9];
  real_T b_I[9];
  real_T b_U[9];
  real_T A[9];
  real_T A_f[9];
  real_T R_iz[9];
  real_T R_f[9];
  real_T R_g[9];
  real_T R_c[9];
  real_T R_o[9];
  real_T R_l[9];
  real_T R_m[9];
  real_T R_mj[9];
  real_T R_cn[9];
  real_T R_fm[9];
  real_T T_p[9];
  real_T Td_e[9];
  real_T T_o4[9];
  real_T Td_h[9];
  real_T T_l[9];
  real_T Td_h2[9];
  real_T T_m[9];
  real_T Td_mc[9];
  real_T b_varargout_1[6];
  real_T value[6];
  real_T qvSolRaw[6];
  real_T c_xSol[6];
  real_T x[6];
  real_T Hg[6];
  real_T sNew_h[6];
  real_T e[6];
  real_T y[6];
  real_T qv_data[6];
  real_T x_c[6];
  real_T Hg_k[6];
  real_T sNew_p[6];
  real_T e_p[6];
  real_T y_p[6];
  real_T qv_data_a[6];
  real_T qv_data_j[6];
  real_T qv_data_e[6];
  real_T qv_data_o[6];
  real_T qv_data_b[6];
  real_T e_a[6];
  real_T y_g[6];
  real_T e_e[6];
  real_T y_f[6];
  real_T e_h[6];
  real_T y_e[6];
  real_T e_c[6];
  real_T y_a[6];
  real_T unusedExpr[5];
  real_T unusedExpr_d[5];
  int8_T msubspace_data[36];
  int8_T msubspace_data_a[36];
  int8_T msubspace_data_p[36];
  int8_T msubspace_data_m[36];
  real_T v[4];
  real_T result_data[4];
  real_T v_o[4];
  real_T v_n[4];
  real_T v_l[4];
  real_T v_p[4];
  real_T v_pt[4];
  real_T vec[3];
  real_T v_f[3];
  real_T v_i[3];
  real_T v_ox[3];
  real_T vspecial_data[3];
  real_T s[3];
  real_T e_k[3];
  real_T work[3];
  char_T cv[17];
  creal_T v_ie;
  creal_T u;
  creal_T u_o;
  creal_T dc;
  real_T ub[2];
  char_T sol_Status_data[14];
  char_T cv1[11];
  char_T cv2[10];
  char_T b_m[9];
  char_T b_c[9];
  char_T b_f[9];
  char_T b_h[9];
  char_T partial_match_data[9];
  char_T b_vstr[9];
  char_T b_m4[9];
  char_T b_a[8];
  char_T b_k[8];
  char_T b_p[8];
  char_T b_b[8];
  char_T b_ch[8];
  char_T b_i[8];
  char_T vstr[8];
  char_T b_my[8];
  real_T quat_fin_c;
  real_T dp;
  real_T theta0;
  real_T b_n;
  real_T c_n;
  real_T d_n;
  real_T value_j;
  real_T value_f;
  real_T value_a;
  real_T value_g;
  real_T value_n;
  real_T value_d;
  real_T value_na;
  real_T value_c;
  real_T value_fx;
  real_T value_p;
  real_T rtb_TmpSignalConversionAtSFu__p;
  real_T rtb_TmpSignalConversionAtSFu__n;
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
  real_T s_k;
  real_T A_n;
  real_T sigma_o;
  real_T s_g;
  real_T bid1;
  real_T bid2;
  real_T qidx_idx_1;
  real_T cth;
  real_T sth;
  real_T tempR_tmp;
  real_T tempR_tmp_c;
  real_T tempR_tmp_cj;
  real_T tempR_tmp_m;
  real_T tempR_tmp_j;
  real_T cost_k;
  real_T b_gamma_m;
  real_T beta_p;
  real_T sigma_d;
  real_T costNew_g;
  real_T m_c;
  real_T s_c;
  real_T A_i;
  real_T sigma_dx;
  real_T b_g;
  real_T axang_idx_0;
  real_T axang_idx_1;
  real_T axang_idx_2;
  real_T bid_l;
  real_T b_index;
  real_T x_f;
  real_T d_u;
  real_T s_d;
  real_T a_jr;
  real_T q;
  real_T nrm;
  real_T rt;
  real_T ztest;
  real_T smm1;
  real_T emm1;
  real_T sqds;
  real_T b_i3;
  real_T unusedU2;
  real_T d_sn;
  real_T roe;
  real_T absa;
  real_T absb;
  real_T scale;
  real_T ads;
  real_T bds;
  real_T bid1_h;
  real_T bid2_n;
  real_T qidx_idx_1_o;
  real_T bid1_c;
  real_T bid2_b;
  real_T qidx_idx_1_e;
  real_T bid1_d;
  real_T bid2_i;
  real_T qidx_idx_1_g;
  real_T bid1_n;
  real_T bid2_l;
  real_T qidx_idx_1_c;
  real_T bid1_nc;
  real_T bid2_p;
  real_T qidx_idx_1_d;
  real_T ssq;
  real_T c;
  real_T bid_o;
  real_T b_index_j;
  real_T pid;
  real_T b_index_c;
  real_T obj_idx_0;
  real_T obj_idx_1;
  real_T obj_idx_2;
  real_T s_h;
  real_T s_da;
  real_T s_c5;
  real_T s_p;
  real_T tol_p;
  real_T scale_a;
  real_T absxk;
  real_T t;
  real_T smax;
  real_T scale_o;
  real_T absxk_j;
  real_T t_p;
  uint32_T b_u[2];
  uint32_T u32[2];
  uint32_T b_u_o[2];
  int32_T sol_Status_size[2];
  int32_T T_size[2];
  int32_T T_size_n[2];
  int32_T T_size_j[2];
  int32_T T_size_e[2];
  int32_T T_size_m[2];
  int32_T T_size_m0[2];
  char_T cv3[6];
  int8_T iv[6];
  int8_T iv1[6];
  int8_T iv2[6];
  int8_T iv3[6];
  char_T b_l[5];
  char_T b_kv[5];
  char_T b_j[5];
  char_T b_fz[5];
  char_T b_cm[5];
  char_T b_fq[5];
  char_T b_nl[5];
  char_T b_iu[5];
  char_T b_l4[5];
  char_T b_i3r[5];
  char_T b_k1[5];
  char_T c_vstr[5];
  char_T b_fc[5];
  int32_T value_al;
  int32_T subsa_idx_1;
  int32_T c_d;
  int32_T partialTrueCount;
  int32_T nm1d2;
  int32_T k;
  int32_T c_e;
  int32_T i;
  int32_T loop_ub;
  int32_T c_eh;
  int32_T ix;
  int32_T nx;
  int32_T i_b;
  int32_T unnamed_idx_1;
  int32_T idxl;
  int32_T j;
  int32_T nx_a;
  int32_T m_i;
  int32_T inner;
  int32_T n;
  int32_T idx;
  int32_T b_i_f;
  int32_T coffset;
  int32_T boffset;
  int32_T aoffset;
  int32_T i_j;
  int32_T g_idx_0;
  int32_T n_o;
  int32_T boffset_f;
  int32_T b_j_o;
  int32_T loop_ub_l;
  int32_T Td_tmp;
  int32_T jointSign;
  int32_T g;
  int32_T j_l;
  int32_T b_i_g;
  int32_T b_kstr;
  int32_T n_d;
  int32_T loop_ub_d;
  int32_T qv_size;
  int32_T coffset_tmp;
  int32_T b_kstr_j;
  int32_T loop_ub_f;
  int32_T a_tmp;
  int32_T a_tmp_tmp;
  int32_T b_kstr_js;
  int32_T loop_ub_h;
  int32_T newNumel;
  int32_T i_c;
  int32_T idxl_n;
  int32_T j_k;
  int32_T nx_av;
  int32_T m_f;
  int32_T inner_j;
  int32_T n_k;
  int32_T idx_b;
  int32_T b_i_h;
  int32_T coffset_e;
  int32_T boffset_h;
  int32_T aoffset_k;
  int32_T i_jw;
  int32_T g_idx_0_o;
  int32_T b_kstr_c;
  int32_T loop_ub_hw;
  int32_T a_tmp_i;
  int32_T a_tmp_tmp_p;
  int32_T i_f;
  int32_T i_e;
  int32_T b_i_n;
  int32_T b_kstr_h;
  int32_T loop_ub_hp;
  int32_T b_kstr_f;
  int32_T loop_ub_i;
  int32_T b_kstr_f4;
  int32_T loop_ub_c;
  int32_T b_k_n;
  int32_T b_mti;
  int32_T b_j_h;
  int32_T b_kk;
  int32_T d;
  int32_T b_k_k;
  int32_T newNumel_h;
  int32_T i_be;
  int32_T d_o;
  int32_T b_k_nq;
  int32_T i_m;
  int32_T b_mti_k;
  int32_T k_j;
  int32_T n_h;
  int32_T boffset_f2;
  int32_T b_j_d;
  int32_T loop_ub_li;
  int32_T Td_tmp_k;
  int32_T b_k_i;
  int32_T loop_ub_h5;
  int32_T jointSign_m;
  int32_T g_g;
  int32_T j_lf;
  int32_T b_i_m;
  int32_T b_kstr_n;
  int32_T n_g;
  int32_T loop_ub_dj;
  int32_T qv_size_m;
  int32_T coffset_tmp_f;
  int32_T jointSign_g;
  int32_T g_j;
  int32_T j_c;
  int32_T b_i_e;
  int32_T b_kstr_m;
  int32_T n_of;
  int32_T loop_ub_a;
  int32_T qv_size_j;
  int32_T coffset_tmp_g;
  int32_T jointSign_j;
  int32_T g_e;
  int32_T j_j;
  int32_T b_i_j;
  int32_T b_kstr_g;
  int32_T n_om;
  int32_T loop_ub_hm;
  int32_T qv_size_c;
  int32_T coffset_tmp_a;
  int32_T jointSign_l;
  int32_T g_jc;
  int32_T j_i;
  int32_T b_i_mi;
  int32_T b_kstr_ft;
  int32_T n_on;
  int32_T loop_ub_iz;
  int32_T qv_size_e;
  int32_T coffset_tmp_j;
  int32_T jointSign_o;
  int32_T g_f;
  int32_T j_m;
  int32_T b_i_a;
  int32_T b_kstr_hi;
  int32_T n_oc;
  int32_T loop_ub_hn;
  int32_T qv_size_j3;
  int32_T coffset_tmp_g3;
  int32_T b_info;
  int32_T jm1;
  int32_T idxAjj;
  int32_T b_j_j;
  int32_T d_l;
  int32_T ia;
  int32_T ix_k;
  int32_T iy;
  int32_T k_d;
  int32_T c_A_size_idx_0;
  int32_T c_A_size_idx_1;
  int32_T i_n;
  int32_T b_i_j5;
  int32_T b_kstr_a;
  int32_T loop_ub_hz;
  int32_T b_kstr_i;
  int32_T b_i_d;
  int32_T loop_ub_b;
  int32_T b_kstr_hj;
  int32_T loop_ub_p;
  int32_T nmatched;
  int32_T minnanb;
  int32_T kstr;
  int32_T loop_ub_n;
  int32_T partial_match_size_idx_1;
  int32_T n_j;
  int32_T boffset_o;
  int32_T b_j_b;
  int32_T loop_ub_j;
  int32_T Td_tmp_e;
  int32_T n_i;
  int32_T boffset_n;
  int32_T b_j_i;
  int32_T loop_ub_p3;
  int32_T Td_tmp_o;
  int32_T n_m;
  int32_T boffset_ot;
  int32_T b_j_g;
  int32_T loop_ub_e;
  int32_T Td_tmp_i;
  int32_T n_gb;
  int32_T boffset_g;
  int32_T b_j_gb;
  int32_T loop_ub_g;
  int32_T Td_tmp_c;
  int32_T rankR;
  int32_T na;
  int32_T minmn;
  int32_T nb;
  int32_T minmana;
  int32_T m_k;
  int32_T nb_d;
  int32_T mn;
  int32_T b_i_k;
  int32_T ma;
  int32_T minmn_p;
  int32_T ii;
  int32_T nmi;
  int32_T mmi;
  int32_T pvt;
  int32_T itemp;
  int32_T lastc;
  int32_T kend;
  int32_T ix_p;
  int32_T iy_m;
  int32_T d_k;
  int32_T b_a0;
  int32_T m_ft;
  int32_T kend_c;
  int32_T k_jk;
  uint32_T r;
  uint32_T mti;
  uint32_T y_k;
  uint32_T r_h;
  c_robotics_core_internal_NLPS_T exitFlag;
  c_robotics_core_internal_NLPS_T exitFlagPrev;
  boolean_T y_d;
  boolean_T flag;
  boolean_T nextBodyIsParent;
  boolean_T b_bool;
  boolean_T b_bool_j;
  boolean_T b_bool_n;
  boolean_T flag_j;
  boolean_T b_bool_l;
  boolean_T b_bool_p;
  boolean_T b_bool_po;
  boolean_T b_bool_ly;
  boolean_T nextBodyIsParent_l;
  boolean_T b_bool_h;
  boolean_T nextBodyIsParent_c;
  boolean_T b_bool_g;
  boolean_T nextBodyIsParent_e;
  boolean_T b_bool_n3;
  boolean_T nextBodyIsParent_f;
  boolean_T b_bool_nl;
  boolean_T nextBodyIsParent_e2;
  boolean_T b_bool_b;
  boolean_T b_bool_a;
  boolean_T b_bool_i;
  boolean_T b_bool_nq;
  boolean_T matched;
  boolean_T b_bool_f;
} B_cartesian_waypoints_planner_T;

// Block states (default storage) for system '<Root>'
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
  c_rigidBodyJoint_cartesian_wa_T gobj_1;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_2;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_3;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_4;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_5;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_6;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_7;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_8;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_9;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_10;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_11;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_12;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_13;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_14;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_15;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_16;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_17;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_18;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_19;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_20;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_21;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_22;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_23;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_24;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_25;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_26;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_27;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_28;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_29;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_30;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_31;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_32;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_33;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_34;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_35;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_36;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_37;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_38;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_39;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_40;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_41;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_42;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_43;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_44;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_45;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_46;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_47;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_48;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_49;// '<S2>/MATLAB System'
  c_rigidBodyJoint_cartesian_wa_T gobj_50;// '<S2>/MATLAB System'
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
  ros_slros_internal_block_GetP_T obj_e;// '<S8>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_d;// '<S8>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_m;// '<S8>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_h;// '<S8>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_mf;// '<S8>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_o;// '<S8>/Get Parameter5'
  ros_slros_internal_block_GetP_T obj_mb;// '<S10>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_c;// '<S10>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_f;// '<S10>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_e0;// '<S10>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_a;// '<S10>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_p;// '<S10>/Get Parameter5'
  ros_slros_internal_block_GetP_T obj_l;// '<S10>/Get Parameter6'
  ros_slros_internal_block_GetP_T obj_i;// '<S9>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_db;// '<S9>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_oq;// '<S9>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_es;// '<S9>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_f4;// '<S9>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_en;// '<S9>/Get Parameter5'
  ros_slros_internal_block_GetP_T obj_l0;// '<S9>/Get Parameter6'
  ros_slros_internal_block_GetP_T obj_eq;// '<S7>/Get Parameter'
  ros_slros_internal_block_Publ_T obj_b;// '<S6>/SinkBlock'
  real_T Delay2_DSTATE[120];           // '<Root>/Delay2'
  real_T Delay3_DSTATE;                // '<Root>/Delay3'
  real_T Delay_DSTATE[6];              // '<Root>/Delay'
  real_T Delay1_DSTATE;                // '<Root>/Delay1'
  uint32_T state_m[625];               // '<S2>/MATLAB System'
} DW_cartesian_waypoints_planne_T;

// Parameters (default storage)
struct P_cartesian_waypoints_planner_T_ {
  SL_Bus_cartesian_waypoints_plann_Float32MultiArray_iutvhs Constant_Value;// Computed Parameter: Constant_Value
                                                                      //  Referenced by: '<S1>/Constant'

  real_T Delay2_InitialCondition[120]; // Expression: zeros(6, 20)
                                          //  Referenced by: '<Root>/Delay2'

  real_T Delay3_InitialCondition;      // Expression: 1
                                          //  Referenced by: '<Root>/Delay3'

  real_T Delay_InitialCondition[6];    // Expression: zeros(6, 1, 1)
                                          //  Referenced by: '<Root>/Delay'

  real_T Delay1_InitialCondition;      // Expression: 1
                                          //  Referenced by: '<Root>/Delay1'

};

// Real-time Model Data Structure
struct tag_RTM_cartesian_waypoints_p_T {
  const char_T *errorStatus;
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_cartesian_waypoints_planner_T cartesian_waypoints_planner_P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern B_cartesian_waypoints_planner_T cartesian_waypoints_planner_B;

#ifdef __cplusplus

}
#endif

// Block states (default storage)
extern DW_cartesian_waypoints_planne_T cartesian_waypoints_planner_DW;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void cartesian_waypoints_planner_initialize(void);
  extern void cartesian_waypoints_planner_step(void);
  extern void cartesian_waypoints_planner_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_cartesian_waypoints__T *const cartesian_waypoints_planner_M;

#ifdef __cplusplus

}
#endif

//-
//  These blocks were eliminated from the model due to optimizations:
//
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
//  '<Root>' : 'cartesian_waypoints_planner'
//  '<S1>'   : 'cartesian_waypoints_planner/Blank Message'
//  '<S2>'   : 'cartesian_waypoints_planner/Inverse Kinematics'
//  '<S3>'   : 'cartesian_waypoints_planner/MATLAB Function'
//  '<S4>'   : 'cartesian_waypoints_planner/MATLAB Function1'
//  '<S5>'   : 'cartesian_waypoints_planner/MATLAB Function2'
//  '<S6>'   : 'cartesian_waypoints_planner/Publish'
//  '<S7>'   : 'cartesian_waypoints_planner/Subsystem'
//  '<S8>'   : 'cartesian_waypoints_planner/Subsystem1'
//  '<S9>'   : 'cartesian_waypoints_planner/Subsystem/Subsystem'
//  '<S10>'  : 'cartesian_waypoints_planner/Subsystem/Subsystem1'

#endif                             // RTW_HEADER_cartesian_waypoints_planner_h_

//
// File trailer for generated code.
//
// [EOF]
//
