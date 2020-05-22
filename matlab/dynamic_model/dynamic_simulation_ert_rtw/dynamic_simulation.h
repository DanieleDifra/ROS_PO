//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: dynamic_simulation.h
//
// Code generated for Simulink model 'dynamic_simulation'.
//
// Model version                  : 1.131
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Fri May 22 10:58:09 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_dynamic_simulation_h_
#define RTW_HEADER_dynamic_simulation_h_
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros_initialize.h"
#include "dynamic_simulation_types.h"
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
  SL_Bus_dynamic_simulation_sensor_msgs_JointState BusAssignment;// '<S2>/Bus Assignment' 
  SL_Bus_dynamic_simulation_std_msgs_Float64MultiArray In1;// '<S13>/In1'
  SL_Bus_dynamic_simulation_std_msgs_Float64MultiArray b_varargout_2;
  SL_Bus_dynamic_simulation_std_msgs_MultiArrayDimension
    b_varargout_2_Layout_Dim[16];
  real_T b_varargout_2_Data[128];
  real_T X[36];
  real_T A[36];
  uint8_T TmpSignalConversionAtSFun_b[192];// '<S2>/Assign to JointState msg'
  real_T T[16];
  real_T dv[16];
  real_T TJ[16];
  real_T obj[16];
  real_T TJ_m[16];
  real_T obj_c[16];
  real_T R[9];
  real_T tempR[9];
  real_T R_k[9];
  real_T tempR_c[9];
  real_T dv1[9];
  real_T dv2[9];
  real_T R_b[9];
  real_T R_p[9];
  real_T MatrixDivide[6];              // '<S6>/Matrix Divide'
  real_T Velocity[6];                  // '<S6>/Velocity'
  real_T q_data[6];
  int8_T msubspace_data[36];
  int8_T msubspace_data_c[36];
  int8_T msubspace_data_f[36];
  int8_T msubspace_data_g[36];
  int8_T msubspace_data_g1[36];
  int8_T msubspace_data_m[36];
  int8_T msubspace_data_n[36];
  int8_T msubspace_data_p[36];
  int8_T msubspace_data_l[36];
  char_T charValue[32];
  char_T charValue_j[32];
  char_T charValue_d[32];
  char_T charValue_g[32];
  char_T charValue_l[32];
  char_T charValue_dh[32];
  real_T result_data[4];
  char_T cv[25];
  real_T v[3];
  real_T v_d[3];
  uint32_T TmpSignalConversionAtSFunct[6];// '<S2>/Assign to JointState msg'
  SL_Bus_dynamic_simulation_rosgraph_msgs_Clock BusAssignment_o;// '<Root>/Bus Assignment' 
  char_T cv1[14];
  char_T cv2[13];
  char_T cv3[12];
  char_T initialValue[11];
  char_T b[9];
  char_T b_l[9];
  char_T b_o[9];
  char_T b_b[9];
  char_T b_n[9];
  char_T b_bs[9];
  char_T b_ln[9];
  char_T b_h[9];
  char_T b_bn[9];
  char_T b_d[9];
  char_T b_e[8];
  char_T b_bj[8];
  char_T b_j[8];
  real_T value;
  real_T value_f;
  real_T value_a;
  real_T value_j;
  real_T value_jz;
  real_T Clock1;                       // '<Root>/Clock1'
  real_T nb;
  real_T vNum;
  real_T pid;
  real_T s;
  real_T p_idx_1;
  real_T b_idx_0;
  real_T b_idx_1;
  real_T cth;
  real_T sth;
  real_T tempR_tmp;
  real_T tempR_tmp_o;
  real_T tempR_tmp_n;
  real_T tempR_tmp_i;
  real_T tempR_tmp_oy;
  real_T b_nv;
  real_T axang_idx_0;
  real_T axang_idx_1;
  real_T axang_idx_2;
  real_T smax;
  char_T b_m[5];
  char_T b_c[5];
  int32_T c;
  int32_T d;
  int32_T i;
  int32_T loop_ub;
  int32_T c_m;
  int32_T f;
  int32_T g;
  int32_T cb;
  int32_T n;
  int32_T m;
  int32_T coffset;
  int32_T boffset;
  int32_T aoffset;
  int32_T k;
  int32_T loop_ub_m;
  int32_T q_size;
  int32_T c_tmp;
  int32_T pid_tmp;
  int32_T X_tmp;
  int32_T kstr;
  int32_T b_kstr;
  int32_T obj_tmp;
  int32_T obj_tmp_tmp;
  int32_T kstr_j;
  int32_T kstr_h;
  int32_T b_kstr_c;
  int32_T obj_tmp_c;
  int32_T obj_tmp_tmp_p;
  int32_T i_p;
  int32_T i1;
  int32_T X_tmp_a;
  int32_T i2;
  int32_T newNumel;
  int32_T i_e;
  int32_T newNumel_a;
  int32_T i_a;
  int32_T i_i;
  int32_T i_l;
  uint32_T b_varargout_2_Data_SL_Info_Curr;
  uint32_T b_varargout_2_Data_SL_Info_Rece;
  uint32_T b_varargout_2_Layout_DataOffset;
  uint32_T b_varargout_2_Layout_Dim_SL_Inf;
  uint32_T b_varargout_2_Layout_Dim_SL_I_f;
  boolean_T b_varargout_1;
} B_dynamic_simulation_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  robotics_slmanip_internal_blo_T obj; // '<S11>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_1;// '<S11>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_2;// '<S11>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_3;// '<S11>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_4;// '<S11>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_5;// '<S11>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_6;// '<S11>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_7;// '<S11>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_8;// '<S11>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_9;// '<S11>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_10;// '<S11>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_11;// '<S11>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_12;// '<S11>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_13;// '<S11>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_14;// '<S11>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_15;// '<S11>/MATLAB System'
  n_robotics_manip_internal_Rig_T gobj_16;// '<S11>/MATLAB System'
  ros_slros_internal_block_GetP_T obj_l;// '<S12>/Get Parameter12'
  ros_slros_internal_block_GetP_T obj_lm;// '<S12>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_a;// '<S12>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_d;// '<S12>/Get Parameter5'
  ros_slros_internal_block_GetP_T obj_h;// '<S12>/Get Parameter6'
  ros_slros_internal_block_GetP_T obj_f;// '<S12>/Get Parameter7'
  ros_slros_internal_block_GetP_T obj_e;// '<S12>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_ez;// '<S12>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_i;// '<S12>/Get Parameter8'
  ros_slros_internal_block_GetP_T obj_ezd;// '<S12>/Get Parameter9'
  ros_slros_internal_block_GetP_T obj_n;// '<S12>/Get Parameter10'
  ros_slros_internal_block_GetP_T obj_p;// '<S12>/Get Parameter11'
  ros_slros_internal_block_GetP_T obj_m;// '<S10>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_c;// '<S10>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_px;// '<S10>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_nt;// '<S10>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_o;// '<S10>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_m4;// '<S10>/Get Parameter5'
  ros_slros_internal_block_Publ_T obj_iv;// '<S5>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_cd;// '<S4>/SinkBlock'
  ros_slros_internal_block_Subs_T obj_fk;// '<S7>/SourceBlock'
  int_T Position_IWORK;                // '<S6>/Position'
  int_T Velocity_IWORK;                // '<S6>/Velocity'
} DW_dynamic_simulation_T;

// Continuous states (default storage)
typedef struct {
  real_T Position_CSTATE[6];           // '<S6>/Position'
  real_T Velocity_CSTATE[6];           // '<S6>/Velocity'
} X_dynamic_simulation_T;

// State derivatives (default storage)
typedef struct {
  real_T Position_CSTATE[6];           // '<S6>/Position'
  real_T Velocity_CSTATE[6];           // '<S6>/Velocity'
} XDot_dynamic_simulation_T;

// State disabled
typedef struct {
  boolean_T Position_CSTATE[6];        // '<S6>/Position'
  boolean_T Velocity_CSTATE[6];        // '<S6>/Velocity'
} XDis_dynamic_simulation_T;

#ifndef ODE3_INTG
#define ODE3_INTG

// ODE3 Integration Data
typedef struct {
  real_T *y;                           // output
  real_T *f[3];                        // derivatives
} ODE3_IntgData;

#endif

// Parameters (default storage)
struct P_dynamic_simulation_T_ {
  uint32_T name_max_length;            // Variable: name_max_length
                                          //  Referenced by: '<S2>/Constant'

  SL_Bus_dynamic_simulation_sensor_msgs_JointState Constant_Value;// Computed Parameter: Constant_Value
                                                                     //  Referenced by: '<S9>/Constant'

  SL_Bus_dynamic_simulation_std_msgs_Float64MultiArray Out1_Y0;// Computed Parameter: Out1_Y0
                                                                  //  Referenced by: '<S13>/Out1'

  SL_Bus_dynamic_simulation_std_msgs_Float64MultiArray Constant_Value_n;// Computed Parameter: Constant_Value_n
                                                                      //  Referenced by: '<S7>/Constant'

  SL_Bus_dynamic_simulation_rosgraph_msgs_Clock Constant_Value_i;// Computed Parameter: Constant_Value_i
                                                                    //  Referenced by: '<S1>/Constant'

};

// Real-time Model Data Structure
struct tag_RTM_dynamic_simulation_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_dynamic_simulation_T *contStates;
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

  extern P_dynamic_simulation_T dynamic_simulation_P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern B_dynamic_simulation_T dynamic_simulation_B;

#ifdef __cplusplus

}
#endif

// Continuous states (default storage)
extern X_dynamic_simulation_T dynamic_simulation_X;

// Block states (default storage)
extern DW_dynamic_simulation_T dynamic_simulation_DW;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void dynamic_simulation_initialize(void);
  extern void dynamic_simulation_step(void);
  extern void dynamic_simulation_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_dynamic_simulation_T *const dynamic_simulation_M;

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
//  '<Root>' : 'dynamic_simulation'
//  '<S1>'   : 'dynamic_simulation/JointState1'
//  '<S2>'   : 'dynamic_simulation/Joint_State_Msg_Creator'
//  '<S3>'   : 'dynamic_simulation/MATLAB Function'
//  '<S4>'   : 'dynamic_simulation/Publish'
//  '<S5>'   : 'dynamic_simulation/Publish1'
//  '<S6>'   : 'dynamic_simulation/Robot Dynamic Model'
//  '<S7>'   : 'dynamic_simulation/Subscribe'
//  '<S8>'   : 'dynamic_simulation/Joint_State_Msg_Creator/Assign to JointState msg'
//  '<S9>'   : 'dynamic_simulation/Joint_State_Msg_Creator/Blank Message'
//  '<S10>'  : 'dynamic_simulation/Joint_State_Msg_Creator/Get Joint Names'
//  '<S11>'  : 'dynamic_simulation/Robot Dynamic Model/Joint Space Mass Matrix'
//  '<S12>'  : 'dynamic_simulation/Robot Dynamic Model/Subsystem'
//  '<S13>'  : 'dynamic_simulation/Subscribe/Enabled Subsystem'

#endif                                 // RTW_HEADER_dynamic_simulation_h_

//
// File trailer for generated code.
//
// [EOF]
//
