//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: controller_joint.h
//
// Code generated for Simulink model 'controller_joint'.
//
// Model version                  : 1.27
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Mon May 25 12:26:01 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_controller_joint_h_
#define RTW_HEADER_controller_joint_h_
#include <string.h>
#include <stddef.h>
#include "rtwtypes.h"
#include "slros_initialize.h"
#include "controller_joint_types.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// Block signals (default storage)
typedef struct {
  SL_Bus_controller_joint_sensor_msgs_JointState In1;// '<S11>/In1'
  SL_Bus_controller_joint_sensor_msgs_JointState b_varargout_2;
  SL_Bus_controller_joint_std_msgs_Float64MultiArray msg;// '<Root>/MATLAB Function' 
  SL_Bus_controller_joint_std_msgs_String b_varargout_2_Name[16];
  real_T b_varargout_2_Position[128];
  real_T b_varargout_2_Velocity[128];
  real_T b_varargout_2_Effort[128];
  SL_Bus_controller_joint_std_msgs_Header b_varargout_2_Header;
  real_T Add[6];                       // '<Root>/Add'
  real_T MatrixMultiply1[6];           // '<Root>/MatrixMultiply1'
  SL_Bus_controller_joint_rosgraph_msgs_Clock In1_p;// '<S12>/In1'
  SL_Bus_controller_joint_rosgraph_msgs_Clock b_varargout_2_m;
  char_T cv[15];
  real_T value;
  real_T value_c;
  real_T value_k;
  real_T value_cx;
  real_T value_b;
  real_T value_p;
  real_T value_cv;
  real_T value_f;
  real_T value_g;
  real_T value_g1;
  real_T value_m;
  real_T value_n;
  real_T value_pp;
  real_T value_l;
  real_T value_j;
  real_T value_d;
} B_controller_joint_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  ros_slros_internal_block_GetP_T obj; // '<S10>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_f;// '<S10>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_d;// '<S10>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_o;// '<S10>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_d5;// '<S10>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_l;// '<S10>/Get Parameter5'
  ros_slros_internal_block_GetP_T obj_fs;// '<S9>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_p;// '<S9>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_h;// '<S9>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_b;// '<S9>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_ob;// '<S9>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_i;// '<S9>/Get Parameter5'
  ros_slros_internal_block_GetP_T obj_g;// '<S8>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_k;// '<S8>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_a;// '<S8>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_n;// '<S8>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_nr;// '<S8>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_as;// '<S8>/Get Parameter5'
  ros_slros_internal_block_GetP_T obj_j;// '<S7>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_lf;// '<S7>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_nc;// '<S7>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_e;// '<S7>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_j1;// '<S7>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_f2;// '<S7>/Get Parameter5'
  ros_slros_internal_block_GetP_T obj_az;// '<Root>/Get Parameter'
  ros_slros_internal_block_Publ_T obj_bq;// '<S4>/SinkBlock'
  ros_slros_internal_block_Subs_T obj_dv;// '<S6>/SourceBlock'
  ros_slros_internal_block_Subs_T obj_lb;// '<S5>/SourceBlock'
} DW_controller_joint_T;

// Parameters (default storage)
struct P_controller_joint_T_ {
  SL_Bus_controller_joint_sensor_msgs_JointState Out1_Y0;// Computed Parameter: Out1_Y0
                                                            //  Referenced by: '<S11>/Out1'

  SL_Bus_controller_joint_sensor_msgs_JointState Constant_Value;// Computed Parameter: Constant_Value
                                                                   //  Referenced by: '<S5>/Constant'

  SL_Bus_controller_joint_std_msgs_Float64MultiArray Constant_Value_d;// Computed Parameter: Constant_Value_d
                                                                      //  Referenced by: '<S1>/Constant'

  SL_Bus_controller_joint_rosgraph_msgs_Clock Out1_Y0_k;// Computed Parameter: Out1_Y0_k
                                                           //  Referenced by: '<S12>/Out1'

  SL_Bus_controller_joint_rosgraph_msgs_Clock Constant_Value_m;// Computed Parameter: Constant_Value_m
                                                                  //  Referenced by: '<S6>/Constant'

  real_T Switch_Threshold;             // Expression: 0
                                          //  Referenced by: '<Root>/Switch'

};

// Real-time Model Data Structure
struct tag_RTM_controller_joint_T {
  const char_T *errorStatus;
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_controller_joint_T controller_joint_P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern B_controller_joint_T controller_joint_B;

#ifdef __cplusplus

}
#endif

// Block states (default storage)
extern DW_controller_joint_T controller_joint_DW;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void controller_joint_initialize(void);
  extern void controller_joint_step(void);
  extern void controller_joint_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_controller_joint_T *const controller_joint_M;

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
//  '<Root>' : 'controller_joint'
//  '<S1>'   : 'controller_joint/Blank Message'
//  '<S2>'   : 'controller_joint/MATLAB Function'
//  '<S3>'   : 'controller_joint/MATLAB Function2'
//  '<S4>'   : 'controller_joint/Publish'
//  '<S5>'   : 'controller_joint/Subscribe'
//  '<S6>'   : 'controller_joint/Subscribe1'
//  '<S7>'   : 'controller_joint/Subsystem'
//  '<S8>'   : 'controller_joint/Subsystem1'
//  '<S9>'   : 'controller_joint/Subsystem2'
//  '<S10>'  : 'controller_joint/Subsystem3'
//  '<S11>'  : 'controller_joint/Subscribe/Enabled Subsystem'
//  '<S12>'  : 'controller_joint/Subscribe1/Enabled Subsystem'

#endif                                 // RTW_HEADER_controller_joint_h_

//
// File trailer for generated code.
//
// [EOF]
//
