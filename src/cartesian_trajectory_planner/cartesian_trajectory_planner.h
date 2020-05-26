//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: cartesian_trajectory_planner.h
//
// Code generated for Simulink model 'cartesian_trajectory_planner'.
//
// Model version                  : 1.131
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Mon May 25 16:42:52 2020
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
#ifndef rtmGetContStateDisabled
# define rtmGetContStateDisabled(rtm)  ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
# define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
# define rtmGetContStates(rtm)         ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
# define rtmSetContStates(rtm, val)    ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
# define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
# define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetIntgData
# define rtmGetIntgData(rtm)           ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
# define rtmSetIntgData(rtm, val)      ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
# define rtmGetOdeF(rtm)               ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
# define rtmSetOdeF(rtm, val)          ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
# define rtmGetOdeY(rtm)               ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
# define rtmSetOdeY(rtm, val)          ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
# define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
# define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
# define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
# define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetZCCacheNeedsReset
# define rtmGetZCCacheNeedsReset(rtm)  ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
# define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
# define rtmGetdX(rtm)                 ((rtm)->derivs)
#endif

#ifndef rtmSetdX
# define rtmSetdX(rtm, val)            ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
# define rtmGetTPtr(rtm)               ((rtm)->Timing.t)
#endif

// Block signals for system '<S4>/MATLAB System'
typedef struct {
  real_T c_f1[16];
  real_T a[16];
  real_T b[16];
  real_T a_m[16];
  f_cell_wrap_cartesian_traject_T expl_temp;
  real_T R[9];
  real_T tempR[9];
  real_T u0[6];
  real_T result_data[4];
  real_T v[3];
  int32_T e_data[6];
  char_T b_c[9];
  char_T b_k[9];
  char_T b_cx[9];
  char_T b_b[9];
  char_T b_p[9];
  char_T b_cv[9];
  char_T b_f[9];
  char_T b_g[8];
  char_T b_g1[8];
  char_T b_m[8];
  char_T b_n[8];
  char_T b_pp[8];
  char_T b_l[8];
  char_T b_j[8];
  char_T b_d[8];
  real_T n;
  real_T k;
  real_T sth;
  real_T tempR_tmp;
  real_T tempR_tmp_g;
  real_T tempR_tmp_l;
  real_T tempR_tmp_d;
  char_T b_dy[5];
  int32_T i;
  int32_T d;
  int32_T e;
  int32_T ntilecols;
  int32_T b_jtilecol;
  int32_T b_kstr;
} B_MATLABSystem_cartesian_traj_T;

// Block states (default storage) for system '<S4>/MATLAB System'
typedef struct {
  robotics_slmanip_internal_blo_T obj; // '<S4>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_1;// '<S4>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_2;// '<S4>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_3;// '<S4>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_4;// '<S4>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_5;// '<S4>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_6;// '<S4>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_7;// '<S4>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_8;// '<S4>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_9;// '<S4>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_10;// '<S4>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_11;// '<S4>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_12;// '<S4>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_13;// '<S4>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_14;// '<S4>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_15;// '<S4>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_16;// '<S4>/MATLAB System'
  boolean_T objisempty;                // '<S4>/MATLAB System'
} DW_MATLABSystem_cartesian_tra_T;

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
  real_T Matrot[9];
  real_T R[9];
  real_T tempR[9];
  real_T T_l[9];
  real_T Td_o[9];
  real_T R_o[9];
  real_T R_i[9];
  real_T R_f[9];
  real_T tempR_i[9];
  real_T R_ff[9];
  real_T tempR_g[9];
  real_T R_c[9];
  real_T R_o3[9];
  real_T R_l[9];
  real_T tempR_m[9];
  real_T T_m[9];
  real_T Td_cn[9];
  real_T V_f[9];
  real_T b_I[9];
  real_T b_U[9];
  real_T A[9];
  real_T A_p[9];
  real_T R_e[9];
  real_T R_o4[9];
  real_T R_h[9];
  real_T R_l5[9];
  real_T R_h2[9];
  real_T R_m[9];
  real_T R_mc[9];
  real_T R_h3[9];
  real_T R_cs[9];
  real_T R_k[9];
  real_T T_p[9];
  real_T Td_px[9];
  real_T T_p4[9];
  real_T Td_ap[9];
  real_T T_j[9];
  real_T Td_e[9];
  real_T T_o[9];
  real_T Td_b[9];
  real_T MATLABSystem_o1[6];           // '<S6>/MATLAB System'
  real_T Vel[6];                       // '<Root>/Traslazione '
  real_T value[6];
  real_T Integrator[6];                // '<Root>/Integrator'
  real_T qvSolRaw[6];
  real_T c_xSol[6];
  real_T x[6];
  real_T Hg[6];
  real_T sNew_a[6];
  real_T e[6];
  real_T y[6];
  real_T qv_data[6];
  real_T x_g[6];
  real_T Hg_e[6];
  real_T sNew_f[6];
  real_T e_h[6];
  real_T y_e[6];
  real_T qv_data_c[6];
  real_T qv_data_a[6];
  real_T qv_data_d[6];
  real_T qv_data_af[6];
  real_T qv_data_p[6];
  real_T e_m[6];
  real_T y_o[6];
  real_T e_n[6];
  real_T y_l[6];
  real_T e_p[6];
  real_T y_p[6];
  real_T e_f[6];
  real_T y_i[6];
  real_T unusedExpr[5];
  real_T unusedExpr_o[5];
  int8_T msubspace_data[36];
  int8_T msubspace_data_k[36];
  int8_T msubspace_data_i[36];
  int8_T msubspace_data_o[36];
  real_T v[4];
  real_T result_data[4];
  real_T result_data_m[4];
  real_T v_c[4];
  real_T v_f[4];
  real_T v_h[4];
  real_T v_m[4];
  real_T v_a[4];
  real_T P_k[3];
  real_T out_p[3];
  real_T out_b[3];
  real_T v_ch[3];
  real_T v_n[3];
  real_T R_ij[3];
  real_T v_my[3];
  real_T v_j[3];
  real_T vspecial_data[3];
  real_T s[3];
  real_T e_e[3];
  real_T work[3];
  int32_T e_data[6];
  char_T cv[18];
  SL_Bus_cartesian_trajectory_planner_rosgraph_msgs_Clock In1;// '<S14>/In1'
  real_T Delay[6];                     // '<Root>/Delay'
  real_T MatrixMultiply1[6];           // '<Root>/MatrixMultiply1'
  SL_Bus_cartesian_trajectory_planner_rosgraph_msgs_Clock b_varargout_2;
  real_T ub[2];
  creal_T v_mv;
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
  real_T time;                         // '<Root>/MATLAB Function1'
  real_T tc;
  real_T SpMax;
  real_T s_m;
  real_T s_vel;
  real_T tcstar;
  real_T tf;
  real_T tetaFIN;
  real_T teta_vel;
  real_T tft;
  real_T maxval;
  real_T value_p;
  real_T value_d;
  real_T value_g;
  real_T value_c;
  real_T value_cx;
  real_T rtb_signal2_idx_0;
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
  real_T s_i;
  real_T A_d;
  real_T sigma_g;
  real_T s_l;
  real_T bid1;
  real_T bid2;
  real_T qidx_idx_1;
  real_T cth;
  real_T sth;
  real_T tempR_tmp;
  real_T tempR_tmp_f;
  real_T tempR_tmp_d;
  real_T tempR_tmp_j;
  real_T tempR_tmp_i;
  real_T bid_h;
  real_T b_index;
  real_T cost_n;
  real_T b_gamma_o;
  real_T beta_c;
  real_T sigma_b;
  real_T costNew_e;
  real_T m_d;
  real_T s_ik;
  real_T A_g;
  real_T sigma_n;
  real_T b_l0;
  real_T axang_idx_0;
  real_T axang_idx_1;
  real_T axang_idx_2;
  real_T x_c;
  real_T d_u;
  real_T endeffectorIndex;
  real_T s_n;
  real_T idx_idx_1;
  real_T n;
  real_T k;
  real_T sth_p;
  real_T tempR_tmp_dk;
  real_T tempR_tmp_o;
  real_T tempR_tmp_jr;
  real_T tempR_tmp_c;
  real_T s_h;
  real_T a_da;
  real_T q;
  real_T nrm;
  real_T rt;
  real_T ztest;
  real_T smm1;
  real_T emm1;
  real_T sqds;
  real_T b_c5;
  real_T unusedU2;
  real_T d_sn;
  real_T roe;
  real_T absa;
  real_T absb;
  real_T scale;
  real_T ads;
  real_T bds;
  real_T bid1_p;
  real_T bid2_p;
  real_T qidx_idx_1_a;
  real_T bid1_o;
  real_T bid2_j;
  real_T qidx_idx_1_p;
  real_T bid1_ob;
  real_T bid2_l;
  real_T qidx_idx_1_k;
  real_T bid1_j;
  real_T bid2_f;
  real_T qidx_idx_1_c;
  real_T bid1_f;
  real_T bid2_n;
  real_T qidx_idx_1_i;
  real_T ssq;
  real_T c;
  real_T tol_l;
  real_T scale_i;
  real_T absxk;
  real_T t;
  real_T smax;
  real_T scale_k;
  real_T absxk_f;
  real_T t_a;
  real_T pid;
  real_T b_index_d;
  real_T obj_idx_0;
  real_T obj_idx_1;
  real_T obj_idx_2;
  real_T s_e;
  real_T s_eh;
  real_T s_b;
  real_T s_a;
  uint32_T b_u[2];
  uint32_T u32[2];
  uint32_T b_u_i[2];
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
  char_T b_fa[5];
  char_T b_j[5];
  char_T cv5[5];
  char_T b_oo[5];
  char_T b_fr[5];
  char_T b_oy[5];
  char_T b_ln[5];
  char_T b_lu[5];
  char_T b_gf[5];
  char_T b_d4[5];
  char_T b_dv[5];
  char_T b_jo[5];
  char_T b_f1[5];
  char_T c_vstr[5];
  char_T b_js[5];
  char_T cv6[4];
  int32_T r1;
  int32_T r2;
  int32_T r3;
  int32_T rtemp;
  int32_T Matrot_tmp;
  int32_T Matrot_tmp_h;
  int32_T c_c;
  int32_T partialTrueCount;
  int32_T nm1d2;
  int32_T k_n;
  int32_T c_k;
  int32_T i;
  int32_T loop_ub;
  int32_T c_a;
  int32_T ix;
  int32_T nx;
  int32_T i_f;
  int32_T unnamed_idx_1;
  int32_T idxl;
  int32_T j;
  int32_T nx_j;
  int32_T m_k;
  int32_T inner;
  int32_T n_b;
  int32_T idx;
  int32_T b_i;
  int32_T coffset;
  int32_T boffset;
  int32_T aoffset;
  int32_T i_h;
  int32_T g_idx_0;
  int32_T n_e;
  int32_T boffset_h;
  int32_T b_j_k;
  int32_T loop_ub_j;
  int32_T Td_tmp;
  int32_T jointSign;
  int32_T g;
  int32_T j_o;
  int32_T b_i_c;
  int32_T b_kstr;
  int32_T n_h;
  int32_T loop_ub_i;
  int32_T qv_size;
  int32_T coffset_tmp;
  int32_T b_kstr_p;
  int32_T loop_ub_f;
  int32_T a_tmp;
  int32_T a_tmp_tmp;
  int32_T b_kstr_e;
  int32_T loop_ub_n;
  int32_T newNumel;
  int32_T i_ho;
  int32_T i_hp;
  int32_T b_i_f;
  int32_T b_kstr_i;
  int32_T loop_ub_f4;
  int32_T b_kstr_c;
  int32_T loop_ub_nn;
  int32_T b_kstr_h;
  int32_T loop_ub_k;
  int32_T b_k_h;
  int32_T b_mti;
  int32_T b_j_b;
  int32_T b_kk;
  int32_T idxl_o;
  int32_T j_n;
  int32_T nx_m;
  int32_T m_ku;
  int32_T inner_j;
  int32_T n_hc;
  int32_T idx_f;
  int32_T b_i_d;
  int32_T coffset_l;
  int32_T boffset_k;
  int32_T aoffset_i;
  int32_T i_h5;
  int32_T g_idx_0_m;
  int32_T b_kstr_g;
  int32_T loop_ub_l;
  int32_T a_tmp_m;
  int32_T a_tmp_tmp_n;
  int32_T i_g;
  int32_T d_d;
  int32_T b_k_m;
  int32_T newNumel_f;
  int32_T i_gd;
  int32_T d_j;
  int32_T b_k_c;
  int32_T i_e;
  int32_T b_mti_m;
  int32_T k_o;
  int32_T c_ai;
  int32_T b_i_j;
  int32_T kstr;
  int32_T b_kstr_ga;
  int32_T n_j;
  int32_T loop_ub_e;
  int32_T coffset_tmp_j;
  int32_T d_jb;
  int32_T e_g;
  int32_T ntilecols;
  int32_T b_jtilecol;
  int32_T b_kstr_o;
  int32_T loop_ub_h;
  int32_T b_kstr_c2;
  int32_T loop_ub_a;
  int32_T n_l;
  int32_T boffset_j;
  int32_T b_j_i;
  int32_T loop_ub_m;
  int32_T Td_tmp_f;
  int32_T b_k_o;
  int32_T loop_ub_iz;
  int32_T jointSign_e;
  int32_T g_j;
  int32_T j_o4;
  int32_T b_i_fr;
  int32_T b_kstr_m;
  int32_T n_a;
  int32_T loop_ub_hi;
  int32_T qv_size_o;
  int32_T coffset_tmp_h;
  int32_T jointSign_j;
  int32_T g_g;
  int32_T j_j;
  int32_T b_i_l;
  int32_T b_kstr_k;
  int32_T n_d;
  int32_T loop_ub_np;
  int32_T qv_size_j;
  int32_T coffset_tmp_a;
  int32_T jointSign_h;
  int32_T g_i;
  int32_T j_d;
  int32_T b_i_b;
  int32_T b_kstr_hj;
  int32_T n_p;
  int32_T loop_ub_nk;
  int32_T qv_size_jz;
  int32_T coffset_tmp_o;
  int32_T jointSign_b;
  int32_T g_jk;
  int32_T j_e;
  int32_T b_i_i;
  int32_T b_kstr_n;
  int32_T n_i;
  int32_T loop_ub_p;
  int32_T qv_size_og;
  int32_T coffset_tmp_m;
  int32_T jointSign_o;
  int32_T g_gz;
  int32_T j_ez;
  int32_T b_i_iz;
  int32_T b_kstr_gb;
  int32_T n_g;
  int32_T loop_ub_g;
  int32_T qv_size_g;
  int32_T coffset_tmp_c;
  int32_T b_info;
  int32_T jm1;
  int32_T idxAjj;
  int32_T b_j_kt;
  int32_T d_ds;
  int32_T ia;
  int32_T ix_k;
  int32_T iy;
  int32_T k_p;
  int32_T c_A_size_idx_0;
  int32_T c_A_size_idx_1;
  int32_T i_p;
  int32_T rankR;
  int32_T na;
  int32_T minmn;
  int32_T nb;
  int32_T minmana;
  int32_T m_m;
  int32_T nb_k;
  int32_T mn;
  int32_T b_i_a;
  int32_T ma;
  int32_T minmn_f;
  int32_T ii;
  int32_T nmi;
  int32_T mmi;
  int32_T pvt;
  int32_T itemp;
  int32_T lastc;
  int32_T kend;
  int32_T ix_c;
  int32_T iy_j;
  int32_T d_k;
  int32_T b_h;
  int32_T m_d1;
  int32_T kend_j;
  int32_T k_np;
  int32_T b_kstr_j;
  int32_T b_i_lc;
  int32_T loop_ub_ph;
  int32_T b_kstr_po;
  int32_T loop_ub_ly;
  int32_T nmatched;
  int32_T minnanb;
  int32_T kstr_l;
  int32_T loop_ub_hb;
  int32_T partial_match_size_idx_1;
  int32_T n_c;
  int32_T boffset_g;
  int32_T b_j_e;
  int32_T loop_ub_n3;
  int32_T Td_tmp_fn;
  int32_T n_n;
  int32_T boffset_e;
  int32_T b_j_by;
  int32_T loop_ub_av;
  int32_T Td_tmp_i;
  int32_T n_nq;
  int32_T boffset_f;
  int32_T b_j_i4;
  int32_T loop_ub_k3;
  int32_T Td_tmp_b;
  int32_T n_da;
  int32_T boffset_hs;
  int32_T b_j_n;
  int32_T loop_ub_fj;
  int32_T Td_tmp_a;
  uint32_T r;
  uint32_T mti;
  uint32_T y_m;
  uint32_T r_g;
  c_robotics_core_internal_NLPS_T exitFlag;
  c_robotics_core_internal_NLPS_T exitFlagPrev;
  boolean_T b_varargout_1;
  boolean_T y_n;
  boolean_T flag;
  boolean_T nextBodyIsParent;
  boolean_T b_bool;
  boolean_T b_bool_c;
  boolean_T b_bool_d;
  boolean_T b_bool_k;
  boolean_T b_bool_cl;
  boolean_T b_bool_j;
  boolean_T flag_m;
  boolean_T b_bool_i;
  boolean_T b_bool_b;
  boolean_T b_bool_o;
  boolean_T b_bool_g;
  boolean_T nextBodyIsParent_e;
  boolean_T b_bool_in;
  boolean_T nextBodyIsParent_eb;
  boolean_T b_bool_is;
  boolean_T nextBodyIsParent_m;
  boolean_T b_bool_dl;
  boolean_T nextBodyIsParent_j;
  boolean_T b_bool_p;
  boolean_T nextBodyIsParent_b;
  boolean_T b_bool_pn;
  boolean_T b_bool_n;
  boolean_T b_bool_ce;
  boolean_T matched;
  boolean_T b_bool_nh;
  B_MATLABSystem_cartesian_traj_T MATLABSystem_h;// '<S4>/MATLAB System'
  B_MATLABSystem_cartesian_traj_T MATLABSystem_d;// '<S4>/MATLAB System'
} B_cartesian_trajectory_planne_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  robotics_slmanip_internal__as_T obj; // '<S6>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_88;// '<S6>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_89;// '<S6>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_90;// '<S6>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_91;// '<S6>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_92;// '<S6>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_93;// '<S6>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_94;// '<S6>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_95;// '<S6>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_96;// '<S6>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_97;// '<S6>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_98;// '<S6>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_99;// '<S6>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_100;// '<S6>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_101;// '<S6>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_102;// '<S6>/MATLAB System'
  v_robotics_manip_internal_Rig_T gobj_103;// '<S6>/MATLAB System'
  robotics_slmanip_internal_b_a_T obj_g;// '<S3>/MATLAB System'
  f_robotics_manip_internal_IKE_T gobj_85;// '<S6>/MATLAB System'
  x_robotics_manip_internal_Rig_T gobj_83;// '<S6>/MATLAB System'
  x_robotics_manip_internal_Rig_T gobj_84;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_1;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_2;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_3;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_4;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_5;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_6;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_7;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_8;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_9;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_10;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_11;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_12;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_13;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_14;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_15;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_16;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_17;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_18;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_19;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_20;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_21;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_22;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_23;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_24;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_25;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_26;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_27;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_28;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_29;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_30;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_31;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_32;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_33;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_34;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_35;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_36;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_37;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_38;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_39;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_40;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_41;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_42;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_43;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_44;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_45;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_46;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_47;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_48;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_49;// '<S6>/MATLAB System'
  c_rigidBodyJoint_cartesian_as_T gobj_50;// '<S6>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_1_h;// '<S3>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_2_m;// '<S3>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_3_l;// '<S3>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_4_j;// '<S3>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_5_g;// '<S3>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_6_a;// '<S3>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_7_b;// '<S3>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_8_d;// '<S3>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_9_g;// '<S3>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_10_f;// '<S3>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_11_i;// '<S3>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_12_b;// '<S3>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_13_a;// '<S3>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_14_h;// '<S3>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_15_g;// '<S3>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_16_c;// '<S3>/MATLAB System'
  h_robotics_core_internal_Damp_T gobj_86;// '<S6>/MATLAB System'
  h_robotics_core_internal_Damp_T gobj_87;// '<S6>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_51;// '<S6>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_52;// '<S6>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_53;// '<S6>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_54;// '<S6>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_55;// '<S6>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_56;// '<S6>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_57;// '<S6>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_58;// '<S6>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_59;// '<S6>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_60;// '<S6>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_61;// '<S6>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_62;// '<S6>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_63;// '<S6>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_64;// '<S6>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_65;// '<S6>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_66;// '<S6>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_67;// '<S6>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_68;// '<S6>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_69;// '<S6>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_70;// '<S6>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_71;// '<S6>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_72;// '<S6>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_73;// '<S6>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_74;// '<S6>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_75;// '<S6>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_76;// '<S6>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_77;// '<S6>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_78;// '<S6>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_79;// '<S6>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_80;// '<S6>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_81;// '<S6>/MATLAB System'
  w_robotics_manip_internal_Rig_T gobj_82;// '<S6>/MATLAB System'
  ros_slros_internal_block_GetP_T obj_e;// '<S12>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_d;// '<S12>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_m;// '<S12>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_h;// '<S12>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_mf;// '<S12>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_o;// '<S12>/Get Parameter5'
  ros_slros_internal_block_GetP_T obj_e0;// '<S16>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_a;// '<S16>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_p;// '<S16>/Get Parameter5'
  ros_slros_internal_block_GetP_T obj_l;// '<S16>/Get Parameter6'
  ros_slros_internal_block_GetP_T obj_mb;// '<S16>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_c;// '<S16>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_f;// '<S16>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_es;// '<S15>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_f4;// '<S15>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_en;// '<S15>/Get Parameter5'
  ros_slros_internal_block_GetP_T obj_l0;// '<S15>/Get Parameter6'
  ros_slros_internal_block_GetP_T obj_i;// '<S15>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_db;// '<S15>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_oq;// '<S15>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_di;// '<S11>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_cj;// '<S11>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_cu;// '<Root>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_k;// '<Root>/Get Parameter1'
  ros_slros_internal_block_Publ_T obj_n;// '<S9>/SinkBlock'
  ros_slros_internal_block_Subs_T obj_gl;// '<S10>/SourceBlock'
  real_T Delay_DSTATE[6];              // '<Root>/Delay'
  real_T Reciprocal_DWORK4[36];        // '<Root>/Reciprocal'
  uint32_T state_m[625];               // '<S6>/MATLAB System'
  DW_MATLABSystem_cartesian_tra_T MATLABSystem_h;// '<S4>/MATLAB System'
  DW_MATLABSystem_cartesian_tra_T MATLABSystem_d;// '<S4>/MATLAB System'
} DW_cartesian_trajectory_plann_T;

// Continuous states (default storage)
typedef struct {
  real_T Integrator_CSTATE[6];         // '<Root>/Integrator'
} X_cartesian_trajectory_planne_T;

// State derivatives (default storage)
typedef struct {
  real_T Integrator_CSTATE[6];         // '<Root>/Integrator'
} XDot_cartesian_trajectory_pla_T;

// State disabled
typedef struct {
  boolean_T Integrator_CSTATE[6];      // '<Root>/Integrator'
} XDis_cartesian_trajectory_pla_T;

#ifndef ODE3_INTG
#define ODE3_INTG

// ODE3 Integration Data
typedef struct {
  real_T *y;                           // output
  real_T *f[3];                        // derivatives
} ODE3_IntgData;

#endif

// Parameters (default storage)
struct P_cartesian_trajectory_planne_T_ {
  SL_Bus_cartesian_trajectory_plan_JointTrajectoryPoint_1eqsod Constant_Value;// Computed Parameter: Constant_Value
                                                                      //  Referenced by: '<S1>/Constant'

  SL_Bus_cartesian_trajectory_planner_rosgraph_msgs_Clock Out1_Y0;// Computed Parameter: Out1_Y0
                                                                     //  Referenced by: '<S14>/Out1'

  SL_Bus_cartesian_trajectory_planner_rosgraph_msgs_Clock Constant_Value_e;// Computed Parameter: Constant_Value_e
                                                                      //  Referenced by: '<S10>/Constant'

  real_T Delay_InitialCondition[6];    // Expression: zeros(6, 1, 1)
                                          //  Referenced by: '<Root>/Delay'

  real_T Integrator_IC;                // Expression: 0
                                          //  Referenced by: '<Root>/Integrator'

};

// Real-time Model Data Structure
struct tag_RTM_cartesian_trajectory__T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_cartesian_trajectory_planne_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[6];
  real_T odeF[3][6];
  ODE3_IntgData intgData;

  //
  //  Sizes:
  //  The following substructure contains sizes information
  //  for many of the model attributes such as inputs, outputs,
  //  dwork, sample times, etc.

  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  //
  //  Timing:
  //  The following substructure contains information regarding
  //  the timing information for the model.

  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
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

// Continuous states (default storage)
extern X_cartesian_trajectory_planne_T cartesian_trajectory_planner_X;

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
//  Block '<Root>/Display5' : Unused code path elimination


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
//  '<S5>'   : 'cartesian_trajectory_planner/Get Transform1'
//  '<S6>'   : 'cartesian_trajectory_planner/Inverse Kinematics'
//  '<S7>'   : 'cartesian_trajectory_planner/MATLAB Function'
//  '<S8>'   : 'cartesian_trajectory_planner/MATLAB Function1'
//  '<S9>'   : 'cartesian_trajectory_planner/Publish'
//  '<S10>'  : 'cartesian_trajectory_planner/Subscribe'
//  '<S11>'  : 'cartesian_trajectory_planner/Subsystem'
//  '<S12>'  : 'cartesian_trajectory_planner/Subsystem1'
//  '<S13>'  : 'cartesian_trajectory_planner/Traslazione '
//  '<S14>'  : 'cartesian_trajectory_planner/Subscribe/Enabled Subsystem'
//  '<S15>'  : 'cartesian_trajectory_planner/Subsystem/Subsystem'
//  '<S16>'  : 'cartesian_trajectory_planner/Subsystem/Subsystem1'

#endif                            // RTW_HEADER_cartesian_trajectory_planner_h_

//
// File trailer for generated code.
//
// [EOF]
//
