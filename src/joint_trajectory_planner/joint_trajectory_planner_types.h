//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: joint_trajectory_planner_types.h
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
#ifndef RTW_HEADER_joint_trajectory_planner_types_h_
#define RTW_HEADER_joint_trajectory_planner_types_h_
#include "rtwtypes.h"

// Model Code Variants
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_

typedef struct {
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
} SL_Bus_ROSVariableLengthArrayInfo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_joint_trajectory_planner_ros_time_Duration_
#define DEFINED_TYPEDEF_FOR_SL_Bus_joint_trajectory_planner_ros_time_Duration_

// MsgType=ros_time/Duration
typedef struct {
  real_T Sec;
  real_T Nsec;
} SL_Bus_joint_trajectory_planner_ros_time_Duration;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_joint_trajectory_planner_JointTrajectoryPoint_y0wep_
#define DEFINED_TYPEDEF_FOR_SL_Bus_joint_trajectory_planner_JointTrajectoryPoint_y0wep_

// MsgType=trajectory_msgs/JointTrajectoryPoint
typedef struct {
  // IsVarLen=1:VarLenCategory=data:VarLenElem=Positions_SL_Info:TruncateAction=warn 
  real_T Positions[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Positions
  SL_Bus_ROSVariableLengthArrayInfo Positions_SL_Info;

  // IsVarLen=1:VarLenCategory=data:VarLenElem=Velocities_SL_Info:TruncateAction=warn 
  real_T Velocities[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Velocities
  SL_Bus_ROSVariableLengthArrayInfo Velocities_SL_Info;

  // IsVarLen=1:VarLenCategory=data:VarLenElem=Accelerations_SL_Info:TruncateAction=warn 
  real_T Accelerations[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Accelerations
  SL_Bus_ROSVariableLengthArrayInfo Accelerations_SL_Info;

  // IsVarLen=1:VarLenCategory=data:VarLenElem=Effort_SL_Info:TruncateAction=warn 
  real_T Effort[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Effort
  SL_Bus_ROSVariableLengthArrayInfo Effort_SL_Info;

  // MsgType=ros_time/Duration
  SL_Bus_joint_trajectory_planner_ros_time_Duration TimeFromStart;
} SL_Bus_joint_trajectory_planner_JointTrajectoryPoint_y0wep;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_joint_trajectory_planner_ros_time_Time_
#define DEFINED_TYPEDEF_FOR_SL_Bus_joint_trajectory_planner_ros_time_Time_

// MsgType=ros_time/Time
typedef struct {
  real_T Sec;
  real_T Nsec;
} SL_Bus_joint_trajectory_planner_ros_time_Time;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_joint_trajectory_planner_rosgraph_msgs_Clock_
#define DEFINED_TYPEDEF_FOR_SL_Bus_joint_trajectory_planner_rosgraph_msgs_Clock_

// MsgType=rosgraph_msgs/Clock
typedef struct {
  // MsgType=ros_time/Time
  SL_Bus_joint_trajectory_planner_ros_time_Time Clock_;
} SL_Bus_joint_trajectory_planner_rosgraph_msgs_Clock;

#endif

#ifndef struct_tag_KSdGoEc2IyOHz4CLi4rcCD
#define struct_tag_KSdGoEc2IyOHz4CLi4rcCD

struct tag_KSdGoEc2IyOHz4CLi4rcCD
{
  int32_T __dummy;
};

#endif                                 //struct_tag_KSdGoEc2IyOHz4CLi4rcCD

#ifndef typedef_e_robotics_slcore_internal_bl_T
#define typedef_e_robotics_slcore_internal_bl_T

typedef struct tag_KSdGoEc2IyOHz4CLi4rcCD e_robotics_slcore_internal_bl_T;

#endif                                 //typedef_e_robotics_slcore_internal_bl_T

#ifndef struct_tag_PzhaB0v2Sx4ikuHWZx5WUB
#define struct_tag_PzhaB0v2Sx4ikuHWZx5WUB

struct tag_PzhaB0v2Sx4ikuHWZx5WUB
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  e_robotics_slcore_internal_bl_T SampleTimeHandler;
};

#endif                                 //struct_tag_PzhaB0v2Sx4ikuHWZx5WUB

#ifndef typedef_ros_slros_internal_block_GetP_T
#define typedef_ros_slros_internal_block_GetP_T

typedef struct tag_PzhaB0v2Sx4ikuHWZx5WUB ros_slros_internal_block_GetP_T;

#endif                                 //typedef_ros_slros_internal_block_GetP_T

#ifndef struct_tag_rkSooZHJZnr3Dpfu1LNqfH
#define struct_tag_rkSooZHJZnr3Dpfu1LNqfH

struct tag_rkSooZHJZnr3Dpfu1LNqfH
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
};

#endif                                 //struct_tag_rkSooZHJZnr3Dpfu1LNqfH

#ifndef typedef_ros_slros_internal_block_Publ_T
#define typedef_ros_slros_internal_block_Publ_T

typedef struct tag_rkSooZHJZnr3Dpfu1LNqfH ros_slros_internal_block_Publ_T;

#endif                                 //typedef_ros_slros_internal_block_Publ_T

#ifndef struct_tag_9SewJ4y3IXNs5GrZti8qkG
#define struct_tag_9SewJ4y3IXNs5GrZti8qkG

struct tag_9SewJ4y3IXNs5GrZti8qkG
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
};

#endif                                 //struct_tag_9SewJ4y3IXNs5GrZti8qkG

#ifndef typedef_ros_slros_internal_block_Subs_T
#define typedef_ros_slros_internal_block_Subs_T

typedef struct tag_9SewJ4y3IXNs5GrZti8qkG ros_slros_internal_block_Subs_T;

#endif                                 //typedef_ros_slros_internal_block_Subs_T

#ifndef struct_tag_PMfBDzoakfdM9QAdfx2o6D
#define struct_tag_PMfBDzoakfdM9QAdfx2o6D

struct tag_PMfBDzoakfdM9QAdfx2o6D
{
  uint32_T f1[8];
};

#endif                                 //struct_tag_PMfBDzoakfdM9QAdfx2o6D

#ifndef typedef_cell_wrap_joint_trajectory_pl_T
#define typedef_cell_wrap_joint_trajectory_pl_T

typedef struct tag_PMfBDzoakfdM9QAdfx2o6D cell_wrap_joint_trajectory_pl_T;

#endif                                 //typedef_cell_wrap_joint_trajectory_pl_T

#ifndef struct_tag_pzFePiHQF4SDrxHoNaa5Y
#define struct_tag_pzFePiHQF4SDrxHoNaa5Y

struct tag_pzFePiHQF4SDrxHoNaa5Y
{
  int32_T isInitialized;
  boolean_T TunablePropsChanged;
  cell_wrap_joint_trajectory_pl_T inputVarSize[3];
  real_T VelocityBoundaryCondition[12];
  real_T AccelerationBoundaryCondition[10];
};

#endif                                 //struct_tag_pzFePiHQF4SDrxHoNaa5Y

#ifndef typedef_robotics_slcore_internal_bloc_T
#define typedef_robotics_slcore_internal_bloc_T

typedef struct tag_pzFePiHQF4SDrxHoNaa5Y robotics_slcore_internal_bloc_T;

#endif                                 //typedef_robotics_slcore_internal_bloc_T

// Parameters (default storage)
typedef struct P_joint_trajectory_planner_T_ P_joint_trajectory_planner_T;

// Forward declaration for rtModel
typedef struct tag_RTM_joint_trajectory_plan_T RT_MODEL_joint_trajectory_pla_T;

#endif                          // RTW_HEADER_joint_trajectory_planner_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
