//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: controller_cartesian.h
//
// Code generated for Simulink model 'controller_cartesian'.
//
// Model version                  : 1.2
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Tue May 26 00:11:50 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_controller_cartesian_h_
#define RTW_HEADER_controller_cartesian_h_
#include <math.h>
#include <string.h>
#include <stddef.h>
#include "rtwtypes.h"
#include "slros_initialize.h"
#include "controller_cartesian_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
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
  SL_Bus_controller_cartesian_sensor_msgs_JointState In1;// '<S12>/In1'
  SL_Bus_controller_cartesian_sensor_msgs_JointState b_varargout_2;
  SL_Bus_controller_cartesian_std_msgs_Float64MultiArray msg;// '<Root>/MATLAB Function' 
  SL_Bus_controller_cartesian_std_msgs_Float32MultiArray In1_o;// '<S14>/In1'
  SL_Bus_controller_cartesian_std_msgs_Float32MultiArray b_varargout_2_m;
  real_T t[342];
  SL_Bus_controller_cartesian_std_msgs_MultiArrayDimension
    b_varargout_2_Layout_Dim[16];
  SL_Bus_controller_cartesian_std_msgs_String b_varargout_2_Name[16];
  real_T b_varargout_2_Position[128];
  real_T b_varargout_2_Velocity[128];
  real_T b_varargout_2_Effort[128];
  real_T matrix[120];                  // '<Root>/MATLAB Function1'
  real_T act_max_vel[114];
  real_T signes[114];
  real32_T b_varargout_2_Data[128];
  real_T max_vel[20];
  real_T acc[20];
  SL_Bus_controller_cartesian_std_msgs_Header b_varargout_2_Header;
  real_T MatrixMultiply1[6];           // '<Root>/MatrixMultiply1'
  real_T q[6];                         // '<Root>/MATLAB Function3'
  char_T cv[17];
  SL_Bus_controller_cartesian_rosgraph_msgs_Clock In1_n;// '<S13>/In1'
  SL_Bus_controller_cartesian_rosgraph_msgs_Clock b_varargout_2_c;
  real_T t_up;
  real_T dist;
  real_T t1;
  real_T value;
  real_T value_k;
  real_T value_c;
  real_T value_b;
  real_T delayed_time;                 // '<Root>/MATLAB Function2'
  real_T d;
  real_T d1;
  real_T d2;
} B_controller_cartesian_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  ros_slros_internal_block_GetP_T obj; // '<S11>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_p;// '<S11>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_h;// '<S11>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_b;// '<S11>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_m;// '<S11>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_e;// '<S11>/Get Parameter5'
  ros_slros_internal_block_GetP_T obj_h0;// '<S10>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_bu;// '<S10>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_o;// '<S10>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_hb;// '<S10>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_g;// '<S10>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_l;// '<S10>/Get Parameter5'
  ros_slros_internal_block_GetP_T obj_bo;// '<Root>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_lh;// '<Root>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_i;// '<Root>/Get Parameter2'
  ros_slros_internal_block_Publ_T obj_ga;// '<S6>/SinkBlock'
  ros_slros_internal_block_Subs_T obj_e3;// '<S9>/SourceBlock'
  ros_slros_internal_block_Subs_T obj_mm;// '<S8>/SourceBlock'
  ros_slros_internal_block_Subs_T obj_el;// '<S7>/SourceBlock'
} DW_controller_cartesian_T;

// Parameters (default storage)
struct P_controller_cartesian_T_ {
  SL_Bus_controller_cartesian_sensor_msgs_JointState Out1_Y0;// Computed Parameter: Out1_Y0
                                                                //  Referenced by: '<S12>/Out1'

  SL_Bus_controller_cartesian_sensor_msgs_JointState Constant_Value;// Computed Parameter: Constant_Value
                                                                      //  Referenced by: '<S7>/Constant'

  SL_Bus_controller_cartesian_std_msgs_Float64MultiArray Constant_Value_j;// Computed Parameter: Constant_Value_j
                                                                      //  Referenced by: '<S1>/Constant'

  SL_Bus_controller_cartesian_std_msgs_Float32MultiArray Out1_Y0_l;// Computed Parameter: Out1_Y0_l
                                                                      //  Referenced by: '<S14>/Out1'

  SL_Bus_controller_cartesian_std_msgs_Float32MultiArray Constant_Value_jv;// Computed Parameter: Constant_Value_jv
                                                                      //  Referenced by: '<S9>/Constant'

  SL_Bus_controller_cartesian_rosgraph_msgs_Clock Out1_Y0_p;// Computed Parameter: Out1_Y0_p
                                                               //  Referenced by: '<S13>/Out1'

  SL_Bus_controller_cartesian_rosgraph_msgs_Clock Constant_Value_jw;// Computed Parameter: Constant_Value_jw
                                                                      //  Referenced by: '<S8>/Constant'

};

// Real-time Model Data Structure
struct tag_RTM_controller_cartesian_T {
  const char_T *errorStatus;
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_controller_cartesian_T controller_cartesian_P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern B_controller_cartesian_T controller_cartesian_B;

#ifdef __cplusplus

}
#endif

// Block states (default storage)
extern DW_controller_cartesian_T controller_cartesian_DW;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void controller_cartesian_initialize(void);
  extern void controller_cartesian_step(void);
  extern void controller_cartesian_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_controller_cartesian_T *const controller_cartesian_M;

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
//  '<Root>' : 'controller_cartesian'
//  '<S1>'   : 'controller_cartesian/Blank Message'
//  '<S2>'   : 'controller_cartesian/MATLAB Function'
//  '<S3>'   : 'controller_cartesian/MATLAB Function1'
//  '<S4>'   : 'controller_cartesian/MATLAB Function2'
//  '<S5>'   : 'controller_cartesian/MATLAB Function3'
//  '<S6>'   : 'controller_cartesian/Publish'
//  '<S7>'   : 'controller_cartesian/Subscribe'
//  '<S8>'   : 'controller_cartesian/Subscribe1'
//  '<S9>'   : 'controller_cartesian/Subscribe2'
//  '<S10>'  : 'controller_cartesian/Subsystem'
//  '<S11>'  : 'controller_cartesian/Subsystem1'
//  '<S12>'  : 'controller_cartesian/Subscribe/Enabled Subsystem'
//  '<S13>'  : 'controller_cartesian/Subscribe1/Enabled Subsystem'
//  '<S14>'  : 'controller_cartesian/Subscribe2/Enabled Subsystem'

#endif                                 // RTW_HEADER_controller_cartesian_h_

//
// File trailer for generated code.
//
// [EOF]
//
