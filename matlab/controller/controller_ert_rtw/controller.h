//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: controller.h
//
// Code generated for Simulink model 'controller'.
//
// Model version                  : 1.35
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Wed May 27 21:29:22 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_controller_h_
#define RTW_HEADER_controller_h_
#include <string.h>
#include <stddef.h>
#include "rtwtypes.h"
#include "slros_initialize.h"
#include "controller_types.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// Block signals (default storage)
typedef struct {
  SL_Bus_controller_sensor_msgs_JointState In1;// '<S8>/In1'
  SL_Bus_controller_sensor_msgs_JointState b_varargout_2;
  SL_Bus_controller_trajectory_msgs_JointTrajectoryPoint In1_l;// '<S9>/In1'
  SL_Bus_controller_trajectory_msgs_JointTrajectoryPoint b_varargout_2_m;
  SL_Bus_controller_std_msgs_Float64MultiArray msg;// '<Root>/MATLAB Function'
  SL_Bus_controller_std_msgs_String b_varargout_2_Name[16];
  real_T b_varargout_2_Positions[128];
  real_T b_varargout_2_Velocities[128];
  real_T b_varargout_2_Accelerations[128];
  real_T b_varargout_2_Effort[128];
  SL_Bus_controller_std_msgs_Header b_varargout_2_Header;
  real_T Add[6];                       // '<Root>/Add'
  real_T MatrixMultiply1[6];           // '<Root>/MatrixMultiply1'
  real_T value;
  real_T value_c;
  real_T value_k;
  real_T value_cx;
  real_T b_varargout_2_TimeFromStart_Sec;
  real_T b_varargout_2_TimeFromStart_Nse;
  real_T d;
  real_T d1;
  real_T d2;
} B_controller_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  ros_slros_internal_block_GetP_T obj; // '<S7>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_k;// '<S7>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_a;// '<S7>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_n;// '<S7>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_nr;// '<S7>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_as;// '<S7>/Get Parameter5'
  ros_slros_internal_block_GetP_T obj_j;// '<S6>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_l;// '<S6>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_nc;// '<S6>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_e;// '<S6>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_j1;// '<S6>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_f;// '<S6>/Get Parameter5'
  ros_slros_internal_block_Publ_T obj_b;// '<S3>/SinkBlock'
  ros_slros_internal_block_Subs_T obj_ez;// '<S5>/SourceBlock'
  ros_slros_internal_block_Subs_T obj_lb;// '<S4>/SourceBlock'
} DW_controller_T;

// Parameters (default storage)
struct P_controller_T_ {
  SL_Bus_controller_sensor_msgs_JointState Out1_Y0;// Computed Parameter: Out1_Y0
                                                      //  Referenced by: '<S8>/Out1'

  SL_Bus_controller_sensor_msgs_JointState Constant_Value;// Computed Parameter: Constant_Value
                                                             //  Referenced by: '<S4>/Constant'

  SL_Bus_controller_trajectory_msgs_JointTrajectoryPoint Out1_Y0_d;// Computed Parameter: Out1_Y0_d
                                                                      //  Referenced by: '<S9>/Out1'

  SL_Bus_controller_trajectory_msgs_JointTrajectoryPoint Constant_Value_j;// Computed Parameter: Constant_Value_j
                                                                      //  Referenced by: '<S5>/Constant'

  SL_Bus_controller_std_msgs_Float64MultiArray Constant_Value_d;// Computed Parameter: Constant_Value_d
                                                                   //  Referenced by: '<S1>/Constant'

};

// Real-time Model Data Structure
struct tag_RTM_controller_T {
  const char_T *errorStatus;
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_controller_T controller_P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern B_controller_T controller_B;

#ifdef __cplusplus

}
#endif

// Block states (default storage)
extern DW_controller_T controller_DW;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void controller_initialize(void);
  extern void controller_step(void);
  extern void controller_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_controller_T *const controller_M;

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
//  '<Root>' : 'controller'
//  '<S1>'   : 'controller/Blank Message'
//  '<S2>'   : 'controller/MATLAB Function'
//  '<S3>'   : 'controller/Publish'
//  '<S4>'   : 'controller/Subscribe'
//  '<S5>'   : 'controller/Subscribe1'
//  '<S6>'   : 'controller/Subsystem'
//  '<S7>'   : 'controller/Subsystem1'
//  '<S8>'   : 'controller/Subscribe/Enabled Subsystem'
//  '<S9>'   : 'controller/Subscribe1/Enabled Subsystem'

#endif                                 // RTW_HEADER_controller_h_

//
// File trailer for generated code.
//
// [EOF]
//
