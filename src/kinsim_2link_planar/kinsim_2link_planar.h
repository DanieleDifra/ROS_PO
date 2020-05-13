//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: kinsim_2link_planar.h
//
// Code generated for Simulink model 'kinsim_2link_planar'.
//
// Model version                  : 1.124
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Wed May 13 15:54:45 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_kinsim_2link_planar_h_
#define RTW_HEADER_kinsim_2link_planar_h_
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <string.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros_initialize.h"
#include "kinsim_2link_planar_types.h"
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
  SL_Bus_kinsim_2link_planar_sensor_msgs_JointState msg;// '<Root>/Assign to JointState msg' 
  SL_Bus_kinsim_2link_planar_JointTrajectoryPoint_1csgl9 In1;// '<S17>/In1'
  SL_Bus_kinsim_2link_planar_JointTrajectoryPoint_1csgl9 b_varargout_2;
  real_T b_varargout_2_Positions[128];
  real_T b_varargout_2_Velocities[128];
  real_T b_varargout_2_Accelerations[128];
  real_T b_varargout_2_Effort[128];
  real_T X[36];
  creal_T eigVec[16];
  creal_T At[16];
  real_T T2[16];
  real_T R[16];
  real_T T2inv[16];
  real_T T2_c[16];
  real_T T1[16];
  real_T T[16];
  real_T Tdh[16];
  real_T c_f1[16];
  real_T a[16];
  real_T b[16];
  real_T a_k[16];
  real_T b_V[16];
  real_T b_A[16];
  real_T c_f1_c[16];
  real_T a_b[16];
  real_T b_p[16];
  real_T a_c[16];
  f_cell_wrap_kinsim_2link_plan_T expl_temp;
  f_cell_wrap_kinsim_2link_plan_T expl_temp_m;
  real_T R_f[9];
  real_T R_g[9];
  real_T R_g1[9];
  real_T R_m[9];
  real_T R_n[9];
  real_T tempR[9];
  real_T R_p[9];
  real_T tempR_l[9];
  creal_T eigVal[4];
  creal_T beta1[4];
  creal_T work1[4];
  SL_Bus_kinsim_2link_planar_geometry_msgs_Pose BusAssignment;// '<Root>/Bus Assignment' 
  SL_Bus_kinsim_2link_planar_geometry_msgs_Twist BusAssignment1;// '<Root>/Bus Assignment1' 
  real_T MatrixMultiply[6];            // '<S5>/MatrixMultiply'
  int8_T msubspace_data[36];
  real_T cartOrn[4];
  real_T result_data[4];
  real_T work[4];
  real_T result_data_j[4];
  real_T rworka[4];
  real_T work_d[4];
  int32_T e_data[6];
  int32_T e_data_g[6];
  real_T R_l[3];
  real_T v[3];
  real_T tau[3];
  real_T v_d[3];
  real_T v_dy[3];
  real_T b_v[3];
  char_T cv[18];
  int8_T T1_l[16];
  int32_T rscale[4];
  int8_T b_I[16];
  SL_Bus_kinsim_2link_planar_rosgraph_msgs_Clock msg_l;// '<Root>/Assign to Time msg' 
  creal_T s;
  creal_T ctemp;
  creal_T ad22;
  creal_T ascale;
  char_T cv1[15];
  char_T cv2[14];
  char_T cv3[12];
  char_T b_o[9];
  char_T b_b[9];
  char_T b_n[9];
  char_T b_bs[8];
  char_T b_l[8];
  char_T b_h[8];
  char_T b_bn[8];
  real_T K13;
  real_T K14;
  real_T K23;
  real_T K24;
  real_T K34;
  real_T bid2;
  real_T endeffectorIndex;
  real_T s_d;
  real_T idx_idx_1;
  real_T n;
  real_T k;
  real_T sth;
  real_T tempR_tmp;
  real_T tempR_tmp_e;
  real_T tempR_tmp_b;
  real_T tempR_tmp_j;
  real_T colnorm;
  real_T scale;
  real_T absxk;
  real_T t;
  real_T anrm;
  real_T anrmto;
  real_T mul;
  real_T d;
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
  real_T shift_im_f;
  real_T eshift_re;
  real_T eshift_im;
  real_T scale_a;
  real_T g2;
  real_T f2s;
  real_T di;
  real_T x;
  real_T fs_re;
  real_T fs_im;
  real_T gs_re;
  real_T gs_im;
  real_T a_j;
  real_T n_j;
  real_T k_o;
  real_T sth_n;
  real_T tempR_tmp_i;
  real_T tempR_tmp_o;
  real_T tempR_tmp_n;
  real_T tempR_tmp_m;
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
  real_T scale_c;
  real_T z;
  real_T tau_m;
  real_T anorm_m;
  real_T ascale_j;
  real_T temp;
  real_T acoeff;
  real_T scale_h;
  real_T dmin;
  real_T f_y;
  real_T salpha_re;
  real_T salpha_im;
  real_T work2_idx_2_im;
  real_T work2_idx_3_re;
  real_T work2_idx_3_im;
  real_T alpha1;
  real_T xnorm;
  real_T c;
  real_T scale_c0;
  real_T g2_c;
  real_T f2s_p;
  real_T di_p;
  real_T x_a;
  real_T fs_re_e;
  real_T fs_im_a;
  real_T gs_re_a;
  cell_wrap_0_kinsim_2link_plan_T b_i;
  cell_wrap_0_kinsim_2link_plan_T c_l;
  cell_wrap_0_kinsim_2link_plan_T d_o;
  cell_wrap_0_kinsim_2link_plan_T e;
  cell_wrap_0_kinsim_2link_plan_T f;
  cell_wrap_0_kinsim_2link_plan_T g;
  int8_T chainmask[6];
  char_T bname[6];
  char_T b_o2[6];
  char_T a_i[6];
  char_T bname_f[6];
  char_T b_iz[5];
  char_T b_f[5];
  char_T b_g[5];
  int32_T ret;
  int32_T loop_ub;
  int32_T rtb_MATLABSystem_tmp;
  int32_T rtb_MATLABSystem_tmp_tmp;
  int32_T c_c;
  int32_T b_i_o;
  int32_T kstr;
  int32_T n_l;
  int32_T ret_m;
  int32_T loop_ub_m;
  int32_T coffset_tmp;
  int32_T d_c;
  int32_T e_f;
  int32_T ntilecols;
  int32_T b_jtilecol;
  int32_T b_kstr;
  int32_T loop_ub_p;
  int32_T kstr_e;
  int32_T b_kstr_o;
  int32_T b_j;
  int32_T i;
  int32_T ihi;
  int32_T i_h;
  int32_T jcol;
  int32_T c_i;
  int32_T k_l;
  int32_T ii;
  int32_T nzcount;
  int32_T jj;
  int32_T j;
  int32_T ifirst;
  int32_T istart;
  int32_T ilast;
  int32_T ilastm1;
  int32_T iiter;
  int32_T jp1;
  int32_T jiter;
  int32_T i_h2;
  int32_T ctemp_tmp;
  int32_T ctemp_tmp_tmp;
  int32_T d_m;
  int32_T e_m;
  int32_T ntilecols_h;
  int32_T b_jtilecol_c;
  int32_T b_kstr_k;
  int32_T loop_ub_pc;
  int32_T kstr_p;
  int32_T b_kstr_p;
  int32_T i_a;
  int32_T L;
  int32_T k_j;
  int32_T m;
  int32_T nr;
  int32_T hoffset;
  int32_T j_e;
  int32_T b_j_o;
  int32_T c_j;
  int32_T ix;
  int32_T i_b;
  int32_T c_j_a;
  int32_T e_jr;
  int32_T c_x_tmp;
  int32_T c_x_tmp_tmp;
  int32_T d_re_tmp;
  int32_T knt;
  int32_T lastc;
  int32_T rowleft;
  int32_T iac;
  int32_T g_g;
  int32_T b_ia;
  int32_T jy;
  int32_T b_ix;
  int32_T lastv;
  int32_T lastc_e;
  int32_T coltop;
  int32_T ix_f;
  int32_T iac_h;
  int32_T newNumel;
  int32_T i_e;
  int32_T newNumel_c;
  int32_T i_ax;
  int32_T i_d;
  uint32_T b_varargout_2_Positions_SL_Info;
  uint32_T b_varargout_2_Positions_SL_In_p;
  uint32_T b_varargout_2_Velocities_SL_Inf;
  uint32_T b_varargout_2_Velocities_SL_I_o;
  uint32_T b_varargout_2_Accelerations_SL_;
  uint32_T b_varargout_2_Accelerations_S_l;
  uint32_T b_varargout_2_Effort_SL_Info_Cu;
  uint32_T b_varargout_2_Effort_SL_Info_Re;
  boolean_T b_bool;
  boolean_T b_bool_f;
  boolean_T b_bool_i;
  boolean_T b_bool_o;
  boolean_T b_bool_k;
  boolean_T b_bool_ie;
} B_kinsim_2link_planar_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  robotics_slmanip_internal_blo_T obj; // '<S14>/MATLAB System'
  robotics_slmanip_internal_b_l_T obj_b;// '<S15>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_1;// '<S14>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_2;// '<S14>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_3;// '<S14>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_4;// '<S14>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_5;// '<S14>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_6;// '<S14>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_7;// '<S14>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_8;// '<S14>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_9;// '<S14>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_10;// '<S14>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_11;// '<S14>/MATLAB System'
  l_robotics_manip_internal_Rig_T gobj_12;// '<S14>/MATLAB System'
  l_robotics_manip_internal_R_l_T gobj_1_a;// '<S15>/MATLAB System'
  l_robotics_manip_internal_R_l_T gobj_2_o;// '<S15>/MATLAB System'
  l_robotics_manip_internal_R_l_T gobj_3_j;// '<S15>/MATLAB System'
  l_robotics_manip_internal_R_l_T gobj_4_i;// '<S15>/MATLAB System'
  l_robotics_manip_internal_R_l_T gobj_5_a;// '<S15>/MATLAB System'
  l_robotics_manip_internal_R_l_T gobj_6_g;// '<S15>/MATLAB System'
  l_robotics_manip_internal_R_l_T gobj_7_l;// '<S15>/MATLAB System'
  l_robotics_manip_internal_R_l_T gobj_8_j;// '<S15>/MATLAB System'
  l_robotics_manip_internal_R_l_T gobj_9_p;// '<S15>/MATLAB System'
  l_robotics_manip_internal_R_l_T gobj_10_b;// '<S15>/MATLAB System'
  l_robotics_manip_internal_R_l_T gobj_11_l;// '<S15>/MATLAB System'
  l_robotics_manip_internal_R_l_T gobj_12_b;// '<S15>/MATLAB System'
  ros_slros_internal_block_GetP_T obj_e;// '<S13>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_p;// '<S13>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_n;// '<S13>/Get Parameter5'
  ros_slros_internal_block_GetP_T obj_em;// '<S13>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_l;// '<S13>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_ng;// '<S13>/Get Parameter1'
  ros_slros_internal_block_Publ_T obj_bs;// '<S11>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_m;// '<S10>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_f;// '<S9>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_nr;// '<S8>/SinkBlock'
  ros_slros_internal_block_Subs_T obj_pp;// '<S12>/SourceBlock'
  int_T Integrator_IWORK;              // '<Root>/Integrator'
} DW_kinsim_2link_planar_T;

// Continuous states (default storage)
typedef struct {
  real_T Integrator_CSTATE[6];         // '<Root>/Integrator'
} X_kinsim_2link_planar_T;

// State derivatives (default storage)
typedef struct {
  real_T Integrator_CSTATE[6];         // '<Root>/Integrator'
} XDot_kinsim_2link_planar_T;

// State disabled
typedef struct {
  boolean_T Integrator_CSTATE[6];      // '<Root>/Integrator'
} XDis_kinsim_2link_planar_T;

#ifndef ODE3_INTG
#define ODE3_INTG

// ODE3 Integration Data
typedef struct {
  real_T *y;                           // output
  real_T *f[3];                        // derivatives
} ODE3_IntgData;

#endif

// Parameters (default storage)
struct P_kinsim_2link_planar_T_ {
  SL_Bus_kinsim_2link_planar_sensor_msgs_JointState Constant_Value;// Computed Parameter: Constant_Value
                                                                      //  Referenced by: '<S6>/Constant'

  SL_Bus_kinsim_2link_planar_JointTrajectoryPoint_1csgl9 Out1_Y0;// Computed Parameter: Out1_Y0
                                                                    //  Referenced by: '<S17>/Out1'

  SL_Bus_kinsim_2link_planar_JointTrajectoryPoint_1csgl9 Constant_Value_h;// Computed Parameter: Constant_Value_h
                                                                      //  Referenced by: '<S12>/Constant'

  SL_Bus_kinsim_2link_planar_geometry_msgs_Pose Constant_Value_f;// Computed Parameter: Constant_Value_f
                                                                    //  Referenced by: '<S3>/Constant'

  SL_Bus_kinsim_2link_planar_geometry_msgs_Twist Constant_Value_p;// Computed Parameter: Constant_Value_p
                                                                     //  Referenced by: '<S4>/Constant'

  SL_Bus_kinsim_2link_planar_rosgraph_msgs_Clock Constant_Value_o;// Computed Parameter: Constant_Value_o
                                                                     //  Referenced by: '<S7>/Constant'

};

// Real-time Model Data Structure
struct tag_RTM_kinsim_2link_planar_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_kinsim_2link_planar_T *contStates;
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

  extern P_kinsim_2link_planar_T kinsim_2link_planar_P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern B_kinsim_2link_planar_T kinsim_2link_planar_B;

#ifdef __cplusplus

}
#endif

// Continuous states (default storage)
extern X_kinsim_2link_planar_T kinsim_2link_planar_X;

// Block states (default storage)
extern DW_kinsim_2link_planar_T kinsim_2link_planar_DW;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void kinsim_2link_planar_initialize(void);
  extern void kinsim_2link_planar_step(void);
  extern void kinsim_2link_planar_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_kinsim_2link_planar_T *const kinsim_2link_planar_M;

#ifdef __cplusplus

}
#endif

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S5>/Reshape' : Reshape block reduction


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
//  '<Root>' : 'kinsim_2link_planar'
//  '<S1>'   : 'kinsim_2link_planar/Assign to JointState msg'
//  '<S2>'   : 'kinsim_2link_planar/Assign to Time msg'
//  '<S3>'   : 'kinsim_2link_planar/Blank Message'
//  '<S4>'   : 'kinsim_2link_planar/Blank Message1'
//  '<S5>'   : 'kinsim_2link_planar/Forward_Kinematic'
//  '<S6>'   : 'kinsim_2link_planar/JointState'
//  '<S7>'   : 'kinsim_2link_planar/JointState1'
//  '<S8>'   : 'kinsim_2link_planar/Publish'
//  '<S9>'   : 'kinsim_2link_planar/Publish1'
//  '<S10>'  : 'kinsim_2link_planar/Publish2'
//  '<S11>'  : 'kinsim_2link_planar/Publish3'
//  '<S12>'  : 'kinsim_2link_planar/Subscribe'
//  '<S13>'  : 'kinsim_2link_planar/Subsystem'
//  '<S14>'  : 'kinsim_2link_planar/Forward_Kinematic/Get Jacobian'
//  '<S15>'  : 'kinsim_2link_planar/Forward_Kinematic/Get Transform'
//  '<S16>'  : 'kinsim_2link_planar/Forward_Kinematic/MATLAB Function'
//  '<S17>'  : 'kinsim_2link_planar/Subscribe/Enabled Subsystem'

#endif                                 // RTW_HEADER_kinsim_2link_planar_h_

//
// File trailer for generated code.
//
// [EOF]
//
