//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: joint_trajectory_planner.h
//
// Code generated for Simulink model 'joint_trajectory_planner'.
//
// Model version                  : 1.8
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Mon May 25 17:28:28 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_joint_trajectory_planner_h_
#define RTW_HEADER_joint_trajectory_planner_h_
#include <math.h>
#include <string.h>
#include <stddef.h>
#include "rtwtypes.h"
#include "slros_initialize.h"
#include "joint_trajectory_planner_types.h"
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
  SL_Bus_joint_trajectory_planner_JointTrajectoryPoint_y0wep msg;// '<Root>/MATLAB Function' 
  real_T ppd_coefs[72];
  real_T ppdd_coefs[72];
  real_T pp_coefs[72];
  real_T dCoeffs[72];
  real_T ddCoeffs[72];
  real_T coefsWithFlatStart[48];
  real_T coefMat[24];
  real_T newSegmentCoeffs[24];
  real_T t[18];
  real_T unusedU1[12];
  real_T unusedU2[12];
  real_T unusedU3[12];
  real_T MatrixConcatenate[12];        // '<S2>/Matrix Concatenate'
  real_T max_vel[6];
  real_T acc[6];
  real_T act_max_vel[6];
  real_T signes[6];
  real_T qdd[6];                       // '<Root>/MATLAB Function3'
  real_T qd[6];                        // '<Root>/MATLAB Function3'
  real_T q[6];                         // '<Root>/MATLAB Function3'
  real_T dv[6];
  real_T dv1[6];
  real_T dv2[6];
  real_T ppd_breaks[4];
  real_T ppdd_breaks[4];
  real_T pp_breaks[4];
  real_T derivativeBreaks[4];
  real_T modBreaks[4];
  char_T cv[18];
  char_T cv1[16];
  SL_Bus_joint_trajectory_planner_rosgraph_msgs_Clock In1;// '<S10>/In1'
  SL_Bus_joint_trajectory_planner_rosgraph_msgs_Clock b_varargout_2;
  real_T signal2[2];
  real_T wayPoints[2];
  real_T varargin_2[2];
  real_T delayed_time;
  real_T t_up;
  real_T dist;
  real_T t1;
  real_T d;
  real_T finalTime;
  real_T u0;
  real_T holdPoint;
  real_T evalPointVector_idx_0;
  real_T evalPointVector_idx_1;
  real_T evalPointVector_idx_2;
  real_T d1;
  real_T xloc;
  int32_T b_k;
  int32_T i;
} B_joint_trajectory_planner_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  robotics_slcore_internal_bloc_T obj; // '<Root>/Polynomial Trajectory'
  ros_slros_internal_block_GetP_T obj_j;// '<S9>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_b;// '<S9>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_d;// '<S9>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_js;// '<S9>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_h;// '<S9>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_n;// '<S9>/Get Parameter5'
  ros_slros_internal_block_GetP_T obj_bh;// '<S8>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_m;// '<S8>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_e;// '<S8>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_dw;// '<S8>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_bs;// '<S8>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_a;// '<S8>/Get Parameter5'
  ros_slros_internal_block_GetP_T obj_g;// '<S2>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_e0;// '<S2>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_gm;// '<S2>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_o;// '<S2>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_l;// '<S2>/Get Parameter1'
  ros_slros_internal_block_Publ_T obj_dw0;// '<S6>/SinkBlock'
  ros_slros_internal_block_Subs_T obj_c;// '<S7>/SourceBlock'
} DW_joint_trajectory_planner_T;

// Parameters (default storage)
struct P_joint_trajectory_planner_T_ {
  SL_Bus_joint_trajectory_planner_JointTrajectoryPoint_y0wep Constant_Value;// Computed Parameter: Constant_Value
                                                                      //  Referenced by: '<S1>/Constant'

  SL_Bus_joint_trajectory_planner_rosgraph_msgs_Clock Out1_Y0;// Computed Parameter: Out1_Y0
                                                                 //  Referenced by: '<S10>/Out1'

  SL_Bus_joint_trajectory_planner_rosgraph_msgs_Clock Constant_Value_o;// Computed Parameter: Constant_Value_o
                                                                      //  Referenced by: '<S7>/Constant'

  real_T PolynomialTrajectory_VelocityBo[12];// Expression: zeros( 6, 2 )
                                                //  Referenced by: '<Root>/Polynomial Trajectory'

  int32_T Switch1_Threshold;           // Computed Parameter: Switch1_Threshold
                                          //  Referenced by: '<Root>/Switch1'

  int32_T Switch_Threshold;            // Computed Parameter: Switch_Threshold
                                          //  Referenced by: '<Root>/Switch'

  int32_T Switch2_Threshold;           // Computed Parameter: Switch2_Threshold
                                          //  Referenced by: '<Root>/Switch2'

};

// Real-time Model Data Structure
struct tag_RTM_joint_trajectory_plan_T {
  const char_T *errorStatus;
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_joint_trajectory_planner_T joint_trajectory_planner_P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern B_joint_trajectory_planner_T joint_trajectory_planner_B;

#ifdef __cplusplus

}
#endif

// Block states (default storage)
extern DW_joint_trajectory_planner_T joint_trajectory_planner_DW;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void joint_trajectory_planner_initialize(void);
  extern void joint_trajectory_planner_step(void);
  extern void joint_trajectory_planner_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_joint_trajectory_pla_T *const joint_trajectory_planner_M;

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
//  '<Root>' : 'joint_trajectory_planner'
//  '<S1>'   : 'joint_trajectory_planner/Blank Message'
//  '<S2>'   : 'joint_trajectory_planner/Get Parameters'
//  '<S3>'   : 'joint_trajectory_planner/MATLAB Function'
//  '<S4>'   : 'joint_trajectory_planner/MATLAB Function1'
//  '<S5>'   : 'joint_trajectory_planner/MATLAB Function3'
//  '<S6>'   : 'joint_trajectory_planner/Publish'
//  '<S7>'   : 'joint_trajectory_planner/Subscribe'
//  '<S8>'   : 'joint_trajectory_planner/Get Parameters/Subsystem'
//  '<S9>'   : 'joint_trajectory_planner/Get Parameters/Subsystem1'
//  '<S10>'  : 'joint_trajectory_planner/Subscribe/Enabled Subsystem'

#endif                                // RTW_HEADER_joint_trajectory_planner_h_

//
// File trailer for generated code.
//
// [EOF]
//
