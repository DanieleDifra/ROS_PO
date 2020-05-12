//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: kinsim_2link_planar.h
//
// Code generated for Simulink model 'kinsim_2link_planar'.
//
// Model version                  : 1.122
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Tue May 12 22:47:11 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_kinsim_2link_planar_h_
#define RTW_HEADER_kinsim_2link_planar_h_
#include <math.h>
#include <string.h>
#include <stddef.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros_initialize.h"
#include "kinsim_2link_planar_types.h"
#include "rt_defines.h"
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
  SL_Bus_kinsim_2link_planar_sensor_msgs_JointState msg_d;// '<Root>/Assign to CartesianState msg' 
  SL_Bus_kinsim_2link_planar_JointTrajectoryPoint_1csgl9 In1;// '<S12>/In1'
  SL_Bus_kinsim_2link_planar_JointTrajectoryPoint_1csgl9 b_varargout_2;
  real_T b_varargout_2_Positions[128];
  real_T b_varargout_2_Velocities[128];
  real_T b_varargout_2_Accelerations[128];
  real_T b_varargout_2_Effort[128];
  real_T A[96];
  real_T T[96];
  real_T J[36];
  real_T T_m[16];
  real_T dv[9];
  real_T cartVel[6];                   // '<Root>/MATLAB Function'
  real_T cartPos[6];                   // '<Root>/MATLAB Function'
  char_T cv[32];
  real_T dv1[3];
  SL_Bus_kinsim_2link_planar_rosgraph_msgs_Clock msg_l;// '<Root>/Assign to Time msg' 
  int32_T T_size[3];
  int32_T T_size_c[3];
  int32_T tmp_size[3];
  int32_T T_size_k[3];
  int32_T sy_size[3];
  real_T sy;
  real_T value;
  real_T value_c;
  real_T b_varargout_2_TimeFromStart_Sec;
  real_T T_data;
  real_T T_data_b;
  real_T tmp_data;
  real_T T_data_p;
  real_T eulShaped_idx_1;
  real_T eulShaped_idx_2;
  real_T z1_data;
  cell_wrap_0_kinsim_2link_plan_T b;
  cell_wrap_0_kinsim_2link_plan_T c;
  cell_wrap_0_kinsim_2link_plan_T d;
  cell_wrap_0_kinsim_2link_plan_T e;
  cell_wrap_0_kinsim_2link_plan_T f;
  cell_wrap_0_kinsim_2link_plan_T g;
  int32_T i;
  int32_T b_i;
  uint32_T b_varargout_2_Positions_SL_Info;
} B_kinsim_2link_planar_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  ros_slros_internal_block_GetP_T obj; // '<S11>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_p;// '<S11>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_n;// '<S11>/Get Parameter5'
  ros_slros_internal_block_GetP_T obj_e;// '<S11>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_l;// '<S11>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_ng;// '<S11>/Get Parameter1'
  ros_slros_internal_block_Publ_T obj_m;// '<S9>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_f;// '<S8>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_nr;// '<S7>/SinkBlock'
  ros_slros_internal_block_Subs_T obj_pp;// '<S10>/SourceBlock'
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
  real_T dh[18];                       // Variable: dh
                                          //  Referenced by: '<Root>/Constant'

  SL_Bus_kinsim_2link_planar_sensor_msgs_JointState Constant_Value;// Computed Parameter: Constant_Value
                                                                      //  Referenced by: '<S4>/Constant'

  SL_Bus_kinsim_2link_planar_JointTrajectoryPoint_1csgl9 Out1_Y0;// Computed Parameter: Out1_Y0
                                                                    //  Referenced by: '<S12>/Out1'

  SL_Bus_kinsim_2link_planar_JointTrajectoryPoint_1csgl9 Constant_Value_h;// Computed Parameter: Constant_Value_h
                                                                      //  Referenced by: '<S10>/Constant'

  SL_Bus_kinsim_2link_planar_rosgraph_msgs_Clock Constant_Value_o;// Computed Parameter: Constant_Value_o
                                                                     //  Referenced by: '<S5>/Constant'

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
//  Block '<S11>/Constant' : Unused code path elimination


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
//  '<S1>'   : 'kinsim_2link_planar/Assign to CartesianState msg'
//  '<S2>'   : 'kinsim_2link_planar/Assign to JointState msg'
//  '<S3>'   : 'kinsim_2link_planar/Assign to Time msg'
//  '<S4>'   : 'kinsim_2link_planar/JointState'
//  '<S5>'   : 'kinsim_2link_planar/JointState1'
//  '<S6>'   : 'kinsim_2link_planar/MATLAB Function'
//  '<S7>'   : 'kinsim_2link_planar/Publish'
//  '<S8>'   : 'kinsim_2link_planar/Publish1'
//  '<S9>'   : 'kinsim_2link_planar/Publish2'
//  '<S10>'  : 'kinsim_2link_planar/Subscribe'
//  '<S11>'  : 'kinsim_2link_planar/Subsystem'
//  '<S12>'  : 'kinsim_2link_planar/Subscribe/Enabled Subsystem'

#endif                                 // RTW_HEADER_kinsim_2link_planar_h_

//
// File trailer for generated code.
//
// [EOF]
//
