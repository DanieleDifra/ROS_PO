//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: kinematic_simulation.h
//
// Code generated for Simulink model 'kinematic_simulation'.
//
// Model version                  : 1.137
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Mon May 25 17:31:29 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_kinematic_simulation_h_
#define RTW_HEADER_kinematic_simulation_h_
#include <math.h>
#include <string.h>
#include <stddef.h>
#include "rtwtypes.h"
#include "zero_crossing_types.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros_initialize.h"
#include "kinematic_simulation_types.h"
#include "rt_zcfcn.h"

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
  SL_Bus_kinematic_simulation_sensor_msgs_JointState BusAssignment;// '<S2>/Bus Assignment' 
  SL_Bus_kinematic_simulation_JointTrajectoryPoint_qcxomb In1;// '<S12>/In1'
  SL_Bus_kinematic_simulation_JointTrajectoryPoint_qcxomb b_varargout_2;
  real_T b_varargout_2_Positions[128];
  real_T b_varargout_2_Velocities[128];
  real_T b_varargout_2_Accelerations[128];
  real_T b_varargout_2_Effort[128];
  uint8_T TmpSignalConversionAtSFun_c[192];// '<S2>/Assign to JointState msg'
  real_T TmpSignalConversionAtIntegrator[6];
  char_T charValue[32];
  char_T charValue_m[32];
  char_T charValue_c[32];
  char_T charValue_k[32];
  char_T charValue_cx[32];
  char_T charValue_b[32];
  char_T cv[25];
  uint32_T TmpSignalConversionAtSFunct[6];// '<S2>/Assign to JointState msg'
  char_T cv1[18];
  SL_Bus_kinematic_simulation_rosgraph_msgs_Clock BusAssignment_e;// '<Root>/Bus Assignment' 
  real_T b_varargout_2_TimeFromStart_Sec;
  real_T Clock;                        // '<Root>/Clock'
} B_kinematic_simulation_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  ros_slros_internal_block_GetP_T obj; // '<S11>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_f;// '<S11>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_b;// '<S11>/Get Parameter5'
  ros_slros_internal_block_GetP_T obj_n;// '<S11>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_j;// '<S11>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_e;// '<S11>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_c;// '<S10>/Get Parameter6'
  ros_slros_internal_block_GetP_T obj_l;// '<S10>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_i;// '<S10>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_c4;// '<S10>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_cn;// '<S10>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_o;// '<S10>/Get Parameter5'
  ros_slros_internal_block_Publ_T obj_d;// '<S5>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_nr;// '<S4>/SinkBlock'
  ros_slros_internal_block_Subs_T obj_p;// '<S7>/SourceBlock'
  int_T Integrator_IWORK;              // '<S6>/Integrator'
} DW_kinematic_simulation_T;

// Continuous states (default storage)
typedef struct {
  real_T Integrator_CSTATE[6];         // '<S6>/Integrator'
} X_kinematic_simulation_T;

// State derivatives (default storage)
typedef struct {
  real_T Integrator_CSTATE[6];         // '<S6>/Integrator'
} XDot_kinematic_simulation_T;

// State disabled
typedef struct {
  boolean_T Integrator_CSTATE[6];      // '<S6>/Integrator'
} XDis_kinematic_simulation_T;

// Zero-crossing (trigger) state
typedef struct {
  ZCSigState Integrator_Reset_ZCE[6];  // '<S6>/Integrator'
} PrevZCX_kinematic_simulation_T;

#ifndef ODE3_INTG
#define ODE3_INTG

// ODE3 Integration Data
typedef struct {
  real_T *y;                           // output
  real_T *f[3];                        // derivatives
} ODE3_IntgData;

#endif

// Parameters (default storage)
struct P_kinematic_simulation_T_ {
  uint32_T name_max_length;            // Variable: name_max_length
                                          //  Referenced by: '<S2>/Constant'

  SL_Bus_kinematic_simulation_sensor_msgs_JointState Constant_Value;// Computed Parameter: Constant_Value
                                                                      //  Referenced by: '<S9>/Constant'

  SL_Bus_kinematic_simulation_JointTrajectoryPoint_qcxomb Out1_Y0;// Computed Parameter: Out1_Y0
                                                                     //  Referenced by: '<S12>/Out1'

  SL_Bus_kinematic_simulation_JointTrajectoryPoint_qcxomb Constant_Value_h;// Computed Parameter: Constant_Value_h
                                                                      //  Referenced by: '<S7>/Constant'

  SL_Bus_kinematic_simulation_rosgraph_msgs_Clock Constant_Value_j;// Computed Parameter: Constant_Value_j
                                                                      //  Referenced by: '<S1>/Constant'

};

// Real-time Model Data Structure
struct tag_RTM_kinematic_simulation_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_kinematic_simulation_T *contStates;
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

  extern P_kinematic_simulation_T kinematic_simulation_P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern B_kinematic_simulation_T kinematic_simulation_B;

#ifdef __cplusplus

}
#endif

// Continuous states (default storage)
extern X_kinematic_simulation_T kinematic_simulation_X;

// Block states (default storage)
extern DW_kinematic_simulation_T kinematic_simulation_DW;

// Zero-crossing (trigger) state
extern PrevZCX_kinematic_simulation_T kinematic_simulation_PrevZCX;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void kinematic_simulation_initialize(void);
  extern void kinematic_simulation_step(void);
  extern void kinematic_simulation_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_kinematic_simulation_T *const kinematic_simulation_M;

#ifdef __cplusplus

}
#endif

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S6>/Display' : Unused code path elimination


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
//  '<Root>' : 'kinematic_simulation'
//  '<S1>'   : 'kinematic_simulation/Blank Message'
//  '<S2>'   : 'kinematic_simulation/Joint_State_Msg_Creator'
//  '<S3>'   : 'kinematic_simulation/MATLAB Function'
//  '<S4>'   : 'kinematic_simulation/Publish'
//  '<S5>'   : 'kinematic_simulation/Publish1'
//  '<S6>'   : 'kinematic_simulation/Robot Kinematic Model'
//  '<S7>'   : 'kinematic_simulation/Subscribe'
//  '<S8>'   : 'kinematic_simulation/Joint_State_Msg_Creator/Assign to JointState msg'
//  '<S9>'   : 'kinematic_simulation/Joint_State_Msg_Creator/Blank Message'
//  '<S10>'  : 'kinematic_simulation/Joint_State_Msg_Creator/Get Joint Names'
//  '<S11>'  : 'kinematic_simulation/Robot Kinematic Model/initial_configurations'
//  '<S12>'  : 'kinematic_simulation/Subscribe/Enabled Subsystem'

#endif                                 // RTW_HEADER_kinematic_simulation_h_

//
// File trailer for generated code.
//
// [EOF]
//
