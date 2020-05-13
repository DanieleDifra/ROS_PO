//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: dynsim_2link_planar.h
//
// Code generated for Simulink model 'dynsim_2link_planar'.
//
// Model version                  : 1.119
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Thu May 14 00:29:05 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_dynsim_2link_planar_h_
#define RTW_HEADER_dynsim_2link_planar_h_
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros_initialize.h"
#include "dynsim_2link_planar_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
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

// Block signals (default storage)
typedef struct {
  SL_Bus_dynsim_2link_planar_sensor_msgs_JointState msg;// '<Root>/Assign to JointState msg' 
  SL_Bus_dynsim_2link_planar_std_msgs_Float64MultiArray In1;// '<S18>/In1'
  SL_Bus_dynsim_2link_planar_std_msgs_Float64MultiArray b_varargout_2;
  SL_Bus_dynsim_2link_planar_std_msgs_MultiArrayDimension
    b_varargout_2_Layout_Dim[16];
  real_T b_varargout_2_Data[128];
  real_T X[36];
  real_T b_I[36];
  real_T R[36];
  real_T X_m[36];
  creal_T eigVec[16];
  creal_T At[16];
  SL_Bus_dynsim_2link_planar_geometry_msgs_PoseStamped BusAssignment;// '<S12>/Bus Assignment' 
  SL_Bus_dynsim_2link_planar_geometry_msgs_TwistStamped BusAssignment1;// '<S12>/Bus Assignment1' 
  real_T T1[16];
  real_T T2[16];
  real_T R_k[16];
  real_T T2inv[16];
  real_T T2_c[16];
  real_T T1_b[16];
  real_T T[16];
  real_T Tdh[16];
  real_T c_f1[16];
  real_T a[16];
  real_T b[16];
  real_T a_p[16];
  real_T T_c[16];
  real_T Tinv[16];
  real_T TJ[16];
  real_T obj[16];
  real_T T_f[16];
  real_T dv[16];
  real_T TJ_g[16];
  real_T obj_g[16];
  real_T b_V[16];
  real_T b_A[16];
  real_T c_f1_m[16];
  real_T a_n[16];
  real_T b_p[16];
  real_T a_l[16];
  f_cell_wrap_dynsim_2link_pl_a_T expl_temp;
  f_cell_wrap_dynsim_2link_pl_a_T expl_temp_c;
  real_T R_j[9];
  real_T R_d[9];
  real_T R_g[9];
  real_T R_l[9];
  real_T R_dh[9];
  real_T tempR[9];
  real_T R_dy[9];
  real_T R_lx[9];
  real_T dv1[9];
  real_T R_o[9];
  real_T tempR_b[9];
  real_T R_n[9];
  real_T tempR_bs[9];
  real_T R_ln[9];
  real_T tempR_h[9];
  real_T dv2[9];
  real_T dv3[9];
  real_T R_b[9];
  real_T R_da[9];
  creal_T eigVal[4];
  creal_T beta1[4];
  creal_T work1[4];
  real_T Velocity[6];                  // '<S1>/Velocity'
  real_T MATLABSystem[6];              // '<S13>/MATLAB System'
  real_T MatrixMultiply[6];            // '<S4>/MatrixMultiply'
  real_T a0[6];
  real_T q_data[6];
  real_T X_e[6];
  real_T b_I_b[6];
  real_T q_data_j[6];
  int32_T nonFixedIndices_data[8];
  int32_T ii_data[8];
  real_T cartOrn[4];
  real_T result_data[4];
  real_T result_data_f[4];
  real_T work[4];
  real_T result_data_a[4];
  real_T rworka[4];
  real_T work_j[4];
  int32_T e_data[6];
  int32_T e_data_j[6];
  real_T R_o4[3];
  real_T v[3];
  real_T v_n[3];
  real_T v_i[3];
  real_T tau[3];
  real_T v_o[3];
  real_T v_nv[3];
  real_T b_v[3];
  int32_T rscale[4];
  int8_T b_I_m[16];
  SL_Bus_dynsim_2link_planar_rosgraph_msgs_Clock msg_k;// '<Root>/Assign to Time msg' 
  creal_T s;
  creal_T ctemp;
  creal_T ad22;
  creal_T ascale;
  cell_wrap_0_dynsim_2link_plan_T b_c;
  cell_wrap_0_dynsim_2link_plan_T c;
  cell_wrap_0_dynsim_2link_plan_T d;
  cell_wrap_0_dynsim_2link_plan_T e;
  cell_wrap_0_dynsim_2link_plan_T f;
  cell_wrap_0_dynsim_2link_plan_T g;
  char_T b_m[11];
  char_T a_m[11];
  char_T b_j[9];
  char_T b_h[9];
  char_T b_c0[9];
  int8_T chainmask[8];
  char_T b_ct[8];
  char_T b_px[8];
  char_T b_p5[8];
  char_T b_a[8];
  boolean_T mask[8];
  char_T b_e[8];
  char_T b_ax[8];
  char_T b_as[8];
  real_T K23;
  real_T K24;
  real_T K34;
  real_T bid1;
  real_T vNum;
  real_T j;
  real_T endeffectorIndex;
  real_T s_i;
  real_T idx_idx_1;
  real_T n;
  real_T k;
  real_T sth;
  real_T tempR_tmp;
  real_T tempR_tmp_l;
  real_T tempR_tmp_o;
  real_T tempR_tmp_o2;
  real_T nb;
  real_T a_idx_1;
  real_T a_idx_0;
  real_T b_idx_0;
  real_T b_idx_1;
  real_T cth;
  real_T sth_i;
  real_T tempR_tmp_f;
  real_T tempR_tmp_i;
  real_T tempR_tmp_ff;
  real_T tempR_tmp_g;
  real_T tempR_tmp_c;
  real_T nb_o;
  real_T vNum_l;
  real_T pid;
  real_T s_m;
  real_T p_idx_1;
  real_T b_idx_0_m;
  real_T b_idx_1_c;
  real_T b_f;
  real_T axang_idx_0;
  real_T axang_idx_1;
  real_T axang_idx_2;
  real_T colnorm;
  real_T scale;
  real_T absxk;
  real_T t;
  real_T anrm;
  real_T anrmto;
  real_T mul;
  real_T d_p;
  real_T d1;
  real_T atmp_re;
  real_T atmp_im;
  real_T anorm;
  real_T b_atol;
  real_T absxr;
  real_T absxi;
  real_T ar;
  real_T ai;
  real_T t1_re;
  real_T t1_im;
  real_T shift_re;
  real_T shift_im;
  real_T shift_im_e;
  real_T eshift_re;
  real_T eshift_im;
  real_T scale_o;
  real_T g2;
  real_T f2s;
  real_T di;
  real_T x;
  real_T fs_re;
  real_T fs_im;
  real_T gs_re;
  real_T gs_im;
  real_T a_h;
  real_T n_l;
  real_T k_h;
  real_T sth_m;
  real_T tempR_tmp_m;
  real_T tempR_tmp_h;
  real_T tempR_tmp_cs;
  real_T tempR_tmp_k;
  real_T tst;
  real_T htmp1;
  real_T htmp2;
  real_T ba;
  real_T aa;
  real_T h12;
  real_T h21s;
  real_T unusedU1;
  real_T unusedU2;
  real_T unusedU3;
  real_T p;
  real_T bcmax;
  real_T bcmis;
  real_T scale_p;
  real_T z;
  real_T tau_p;
  real_T anorm_p;
  real_T ascale_a;
  real_T temp;
  real_T acoeff;
  real_T scale_j;
  real_T dmin;
  real_T f_y;
  real_T salpha_re;
  real_T salpha_im;
  real_T work2_idx_2_im;
  real_T work2_idx_3_re;
  real_T work2_idx_3_im;
  real_T alpha1;
  real_T xnorm;
  real_T c_e;
  char_T b_o[5];
  char_T b_b[5];
  char_T b_ao[5];
  char_T b_g[5];
  char_T b_ex[5];
  char_T b_fi[5];
  char_T b_h2[5];
  char_T b_ei[5];
  int32_T b_kstr;
  int32_T n_c;
  int32_T iend;
  int32_T i;
  int32_T u1;
  int32_T rtb_MATLABSystem_tmp;
  int32_T i_a;
  int32_T c_d;
  int32_T b_i;
  int32_T kstr;
  int32_T b_kstr_a;
  int32_T n_p;
  int32_T loop_ub;
  int32_T coffset_tmp;
  int32_T d_m;
  int32_T e_o;
  int32_T ntilecols;
  int32_T b_jtilecol;
  int32_T b_kstr_n;
  int32_T loop_ub_l;
  int32_T kstr_p;
  int32_T b_kstr_p;
  int32_T b_k;
  int32_T p_f;
  int32_T m;
  int32_T inner;
  int32_T aoffset;
  int32_T i_i;
  int32_T q_size;
  int32_T unnamed_idx_1;
  int32_T loop_ub_tmp;
  int32_T q_size_tmp;
  int32_T kstr_o;
  int32_T b_kstr_k;
  int32_T obj_tmp;
  int32_T obj_tmp_tmp;
  int32_T kstr_i;
  int32_T b_kstr_o;
  int32_T b_i_m;
  int32_T f_c;
  int32_T cb;
  int32_T idx;
  int32_T n_f;
  int32_T nm1d2;
  int32_T m_h;
  int32_T coffset;
  int32_T boffset;
  int32_T aoffset_m;
  int32_T loop_ub_a;
  int32_T q_size_k;
  int32_T pid_tmp;
  int32_T X_tmp;
  int32_T coffset_tmp_p;
  int32_T kstr_b;
  int32_T b_kstr_c;
  int32_T obj_tmp_n;
  int32_T obj_tmp_tmp_i;
  int32_T b_j_m;
  int32_T i_j;
  int32_T ihi;
  int32_T i_e;
  int32_T jcol;
  int32_T c_i;
  int32_T k_m;
  int32_T ii;
  int32_T nzcount;
  int32_T jj;
  int32_T j_m;
  int32_T ifirst;
  int32_T istart;
  int32_T ilast;
  int32_T ilastm1;
  int32_T iiter;
  int32_T jp1;
  int32_T jiter;
  int32_T i_jg;
  int32_T ctemp_tmp;
  int32_T ctemp_tmp_tmp;
  int32_T count;
  int32_T rescaledir;
  int32_T d_f;
  int32_T e_a;
  int32_T ntilecols_g;
  int32_T b_jtilecol_n;
  int32_T b_kstr_d;
  int32_T loop_ub_n;
  int32_T kstr_c;
  int32_T b_kstr_f;
  int32_T i1;
  int32_T i2;
  int32_T X_tmp_p;
  int32_T X_tmp_p2;
  int32_T i_n;
  int32_T L;
  int32_T k_k;
  int32_T m_n;
  int32_T nr;
  int32_T hoffset;
  int32_T j_o;
  int32_T b_j_g;
  int32_T c_j;
  int32_T ix;
  int32_T s_tmp;
  int32_T b_cq;
  int32_T c_c;
  int32_T i_m;
  int32_T c_j_j;
  int32_T e_jr;
  int32_T c_x_tmp;
  int32_T c_x_tmp_tmp;
  int32_T d_re_tmp;
  int32_T work2_idx_1_re_tmp;
  int32_T d_re_tmp_tmp;
  int32_T i3;
  int32_T Tinv_tmp;
  int32_T knt;
  int32_T lastc;
  int32_T rowleft;
  int32_T iac;
  int32_T g_k;
  int32_T b_ia;
  int32_T jy;
  int32_T b_ix;
  int32_T lastv;
  int32_T lastc_m;
  int32_T coltop;
  int32_T ix_p;
  int32_T iac_d;
  int32_T d_g;
  int32_T b_ia_c;
  int32_T jy_c;
  int32_T newNumel;
  int32_T i_if;
  int32_T newNumel_d;
  int32_T i_g;
  int32_T newNumel_l;
  int32_T i_f;
  int32_T i_d;
  int32_T i_jr;
  int32_T i_i3;
  uint32_T b_varargout_2_Data_SL_Info_Curr;
  uint32_T b_varargout_2_Data_SL_Info_Rece;
  uint32_T b_varargout_2_Layout_DataOffset;
  uint32_T b_varargout_2_Layout_Dim_SL_Inf;
  uint32_T b_varargout_2_Layout_Dim_SL_I_c;
  boolean_T b_bool;
  boolean_T b_bool_b;
  boolean_T b_bool_e;
  boolean_T b_bool_d;
  boolean_T b_bool_i;
  boolean_T b_bool_g;
  boolean_T b_bool_n;
  boolean_T b_bool_l;
  boolean_T b_bool_c;
  boolean_T p_n;
  boolean_T ilascl;
  boolean_T found;
  boolean_T failed;
  boolean_T goto60;
  boolean_T goto70;
  boolean_T goto90;
  boolean_T b_bool_p;
  boolean_T b_bool_dk;
  boolean_T goto150;
  boolean_T b_oi;
  boolean_T lscalea;
  boolean_T lscaleb;
} B_dynsim_2link_planar_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  robotics_slmanip_internal_blo_T obj; // '<S13>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_1;// '<S13>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_2;// '<S13>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_3;// '<S13>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_4;// '<S13>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_5;// '<S13>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_6;// '<S13>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_7;// '<S13>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_8;// '<S13>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_9;// '<S13>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_10;// '<S13>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_11;// '<S13>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_12;// '<S13>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_13;// '<S13>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_14;// '<S13>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_15;// '<S13>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_16;// '<S13>/MATLAB System'
  robotics_slmanip_internal_b_a_T obj_f;// '<S15>/MATLAB System'
  robotics_slmanip_internal__aw_T obj_i;// '<S16>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_1_m;// '<S15>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_2_o;// '<S15>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_3_g;// '<S15>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_4_g;// '<S15>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_5_a;// '<S15>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_6_a;// '<S15>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_7_o;// '<S15>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_8_i;// '<S15>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_9_p;// '<S15>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_10_l;// '<S15>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_11_c;// '<S15>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_12_d;// '<S15>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_13_i;// '<S15>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_14_o;// '<S15>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_15_c;// '<S15>/MATLAB System'
  n_robotics_manip_internal_R_a_T gobj_16_a;// '<S15>/MATLAB System'
  n_robotics_manip_internal__aw_T gobj_1_p;// '<S16>/MATLAB System'
  n_robotics_manip_internal__aw_T gobj_2_on;// '<S16>/MATLAB System'
  n_robotics_manip_internal__aw_T gobj_3_g0;// '<S16>/MATLAB System'
  n_robotics_manip_internal__aw_T gobj_4_h;// '<S16>/MATLAB System'
  n_robotics_manip_internal__aw_T gobj_5_at;// '<S16>/MATLAB System'
  n_robotics_manip_internal__aw_T gobj_6_j;// '<S16>/MATLAB System'
  n_robotics_manip_internal__aw_T gobj_7_n;// '<S16>/MATLAB System'
  n_robotics_manip_internal__aw_T gobj_8_p;// '<S16>/MATLAB System'
  n_robotics_manip_internal__aw_T gobj_9_c;// '<S16>/MATLAB System'
  n_robotics_manip_internal__aw_T gobj_10_g;// '<S16>/MATLAB System'
  n_robotics_manip_internal__aw_T gobj_11_d;// '<S16>/MATLAB System'
  n_robotics_manip_internal__aw_T gobj_12_n;// '<S16>/MATLAB System'
  n_robotics_manip_internal__aw_T gobj_13_f;// '<S16>/MATLAB System'
  n_robotics_manip_internal__aw_T gobj_14_om;// '<S16>/MATLAB System'
  n_robotics_manip_internal__aw_T gobj_15_e;// '<S16>/MATLAB System'
  n_robotics_manip_internal__aw_T gobj_16_n;// '<S16>/MATLAB System'
  ros_slros_internal_block_GetP_T obj_d;// '<S14>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_l;// '<S14>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_a;// '<S14>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_dp;// '<S14>/Get Parameter5'
  ros_slros_internal_block_GetP_T obj_h;// '<S14>/Get Parameter6'
  ros_slros_internal_block_GetP_T obj_f0;// '<S14>/Get Parameter7'
  ros_slros_internal_block_GetP_T obj_e;// '<S14>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_ez;// '<S14>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_io;// '<S14>/Get Parameter8'
  ros_slros_internal_block_GetP_T obj_ezd;// '<S14>/Get Parameter9'
  ros_slros_internal_block_GetP_T obj_n;// '<S14>/Get Parameter10'
  ros_slros_internal_block_GetP_T obj_p;// '<S14>/Get Parameter11'
  ros_slros_internal_block_Publ_T obj_dj;// '<S10>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_g;// '<S9>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_iv;// '<S8>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_c;// '<S7>/SinkBlock'
  ros_slros_internal_block_Subs_T obj_fk;// '<S11>/SourceBlock'
  int_T Position_IWORK;                // '<S1>/Position'
  int_T Velocity_IWORK;                // '<S1>/Velocity'
} DW_dynsim_2link_planar_T;

// Continuous states (default storage)
typedef struct {
  real_T Position_CSTATE[6];           // '<S1>/Position'
  real_T Velocity_CSTATE[6];           // '<S1>/Velocity'
} X_dynsim_2link_planar_T;

// State derivatives (default storage)
typedef struct {
  real_T Position_CSTATE[6];           // '<S1>/Position'
  real_T Velocity_CSTATE[6];           // '<S1>/Velocity'
} XDot_dynsim_2link_planar_T;

// State disabled
typedef struct {
  boolean_T Position_CSTATE[6];        // '<S1>/Position'
  boolean_T Velocity_CSTATE[6];        // '<S1>/Velocity'
} XDis_dynsim_2link_planar_T;

#ifndef ODE3_INTG
#define ODE3_INTG

// ODE3 Integration Data
typedef struct {
  real_T *y;                           // output
  real_T *f[3];                        // derivatives
} ODE3_IntgData;

#endif

// Parameters (default storage)
struct P_dynsim_2link_planar_T_ {
  SL_Bus_dynsim_2link_planar_sensor_msgs_JointState Constant_Value;// Computed Parameter: Constant_Value
                                                                      //  Referenced by: '<S5>/Constant'

  SL_Bus_dynsim_2link_planar_std_msgs_Float64MultiArray Out1_Y0;// Computed Parameter: Out1_Y0
                                                                   //  Referenced by: '<S18>/Out1'

  SL_Bus_dynsim_2link_planar_std_msgs_Float64MultiArray Constant_Value_n;// Computed Parameter: Constant_Value_n
                                                                      //  Referenced by: '<S11>/Constant'

  SL_Bus_dynsim_2link_planar_geometry_msgs_PoseStamped Constant_Value_l;// Computed Parameter: Constant_Value_l
                                                                      //  Referenced by: '<S19>/Constant'

  SL_Bus_dynsim_2link_planar_geometry_msgs_TwistStamped Constant_Value_a;// Computed Parameter: Constant_Value_a
                                                                      //  Referenced by: '<S20>/Constant'

  SL_Bus_dynsim_2link_planar_rosgraph_msgs_Clock Constant_Value_i;// Computed Parameter: Constant_Value_i
                                                                     //  Referenced by: '<S6>/Constant'

  real_T Constant_Value_ij[48];        // Expression: zeros(6,8)
                                          //  Referenced by: '<S1>/Constant'

};

// Real-time Model Data Structure
struct tag_RTM_dynsim_2link_planar_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_dynsim_2link_planar_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[12];
  real_T odeF[3][12];
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
    boolean_T firstInitCondFlag;
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

  extern P_dynsim_2link_planar_T dynsim_2link_planar_P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern B_dynsim_2link_planar_T dynsim_2link_planar_B;

#ifdef __cplusplus

}
#endif

// Continuous states (default storage)
extern X_dynsim_2link_planar_T dynsim_2link_planar_X;

// Block states (default storage)
extern DW_dynsim_2link_planar_T dynsim_2link_planar_DW;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void dynsim_2link_planar_initialize(void);
  extern void dynsim_2link_planar_step(void);
  extern void dynsim_2link_planar_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_dynsim_2link_planar_T *const dynsim_2link_planar_M;

#ifdef __cplusplus

}
#endif

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S13>/Reshape' : Reshape block reduction
//  Block '<S4>/Reshape' : Reshape block reduction


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
//  '<Root>' : 'dynsim_2link_planar'
//  '<S1>'   : 'dynsim_2link_planar/2link_planar robot dynamic model'
//  '<S2>'   : 'dynsim_2link_planar/Assign to JointState msg'
//  '<S3>'   : 'dynsim_2link_planar/Assign to Time msg'
//  '<S4>'   : 'dynsim_2link_planar/Forward_Kinematic'
//  '<S5>'   : 'dynsim_2link_planar/JointState'
//  '<S6>'   : 'dynsim_2link_planar/JointState1'
//  '<S7>'   : 'dynsim_2link_planar/Publish'
//  '<S8>'   : 'dynsim_2link_planar/Publish1'
//  '<S9>'   : 'dynsim_2link_planar/Publish2'
//  '<S10>'  : 'dynsim_2link_planar/Publish3'
//  '<S11>'  : 'dynsim_2link_planar/Subscribe'
//  '<S12>'  : 'dynsim_2link_planar/Subsystem1'
//  '<S13>'  : 'dynsim_2link_planar/2link_planar robot dynamic model/Forward Dynamics'
//  '<S14>'  : 'dynsim_2link_planar/2link_planar robot dynamic model/Subsystem'
//  '<S15>'  : 'dynsim_2link_planar/Forward_Kinematic/Get Jacobian'
//  '<S16>'  : 'dynsim_2link_planar/Forward_Kinematic/Get Transform'
//  '<S17>'  : 'dynsim_2link_planar/Forward_Kinematic/MATLAB Function'
//  '<S18>'  : 'dynsim_2link_planar/Subscribe/Enabled Subsystem'
//  '<S19>'  : 'dynsim_2link_planar/Subsystem1/Blank Message'
//  '<S20>'  : 'dynsim_2link_planar/Subsystem1/Blank Message1'
//  '<S21>'  : 'dynsim_2link_planar/Subsystem1/MATLAB Function'

#endif                                 // RTW_HEADER_dynsim_2link_planar_h_

//
// File trailer for generated code.
//
// [EOF]
//
