//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: joint_trajectory_planner_types.h
//
// Code generated for Simulink model 'joint_trajectory_planner'.
//
// Model version                  : 1.5
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Sun May 17 23:56:12 2020
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

#ifndef struct_emxArray_real_T_1x7
#define struct_emxArray_real_T_1x7

struct emxArray_real_T_1x7
{
  real_T data[7];
  int32_T size[2];
};

#endif                                 //struct_emxArray_real_T_1x7

#ifndef typedef_emxArray_real_T_1x7_joint_tra_T
#define typedef_emxArray_real_T_1x7_joint_tra_T

typedef struct emxArray_real_T_1x7 emxArray_real_T_1x7_joint_tra_T;

#endif                                 //typedef_emxArray_real_T_1x7_joint_tra_T

#ifndef struct_emxArray_real_T_6x6x3
#define struct_emxArray_real_T_6x6x3

struct emxArray_real_T_6x6x3
{
  real_T data[108];
  int32_T size[3];
};

#endif                                 //struct_emxArray_real_T_6x6x3

#ifndef typedef_emxArray_real_T_6x6x3_joint_t_T
#define typedef_emxArray_real_T_6x6x3_joint_t_T

typedef struct emxArray_real_T_6x6x3 emxArray_real_T_6x6x3_joint_t_T;

#endif                                 //typedef_emxArray_real_T_6x6x3_joint_t_T

#ifndef struct_tag_06c2DDfmr4zcnTqhww20ZC
#define struct_tag_06c2DDfmr4zcnTqhww20ZC

struct tag_06c2DDfmr4zcnTqhww20ZC
{
  emxArray_real_T_1x7_joint_tra_T breaks;
  emxArray_real_T_6x6x3_joint_t_T coefs;
};

#endif                                 //struct_tag_06c2DDfmr4zcnTqhww20ZC

#ifndef typedef_s_06c2DDfmr4zcnTqhww20ZC_join_T
#define typedef_s_06c2DDfmr4zcnTqhww20ZC_join_T

typedef struct tag_06c2DDfmr4zcnTqhww20ZC s_06c2DDfmr4zcnTqhww20ZC_join_T;

#endif                                 //typedef_s_06c2DDfmr4zcnTqhww20ZC_join_T

#ifndef struct_emxArray_real_T_1x5
#define struct_emxArray_real_T_1x5

struct emxArray_real_T_1x5
{
  real_T data[5];
  int32_T size[2];
};

#endif                                 //struct_emxArray_real_T_1x5

#ifndef typedef_emxArray_real_T_1x5_joint_tra_T
#define typedef_emxArray_real_T_1x5_joint_tra_T

typedef struct emxArray_real_T_1x5 emxArray_real_T_1x5_joint_tra_T;

#endif                                 //typedef_emxArray_real_T_1x5_joint_tra_T

#ifndef struct_tag_pAdZwyzDKFMs78Zw3azLwB
#define struct_tag_pAdZwyzDKFMs78Zw3azLwB

struct tag_pAdZwyzDKFMs78Zw3azLwB
{
  emxArray_real_T_1x5_joint_tra_T f1;
};

#endif                                 //struct_tag_pAdZwyzDKFMs78Zw3azLwB

#ifndef typedef_f_cell_wrap_joint_trajectory__T
#define typedef_f_cell_wrap_joint_trajectory__T

typedef struct tag_pAdZwyzDKFMs78Zw3azLwB f_cell_wrap_joint_trajectory__T;

#endif                                 //typedef_f_cell_wrap_joint_trajectory__T

#ifndef struct_emxArray_real_T_18x3
#define struct_emxArray_real_T_18x3

struct emxArray_real_T_18x3
{
  real_T data[54];
  int32_T size[2];
};

#endif                                 //struct_emxArray_real_T_18x3

#ifndef typedef_emxArray_real_T_18x3_joint_tr_T
#define typedef_emxArray_real_T_18x3_joint_tr_T

typedef struct emxArray_real_T_18x3 emxArray_real_T_18x3_joint_tr_T;

#endif                                 //typedef_emxArray_real_T_18x3_joint_tr_T

#ifndef struct_tag_4enwL8LeteOUifFosK3af
#define struct_tag_4enwL8LeteOUifFosK3af

struct tag_4enwL8LeteOUifFosK3af
{
  emxArray_real_T_18x3_joint_tr_T f1;
};

#endif                                 //struct_tag_4enwL8LeteOUifFosK3af

#ifndef typedef_g_cell_wrap_joint_trajectory__T
#define typedef_g_cell_wrap_joint_trajectory__T

typedef struct tag_4enwL8LeteOUifFosK3af g_cell_wrap_joint_trajectory__T;

#endif                                 //typedef_g_cell_wrap_joint_trajectory__T

#ifndef struct_tag_FsykqfbjNNuE0BFHsFVRCE
#define struct_tag_FsykqfbjNNuE0BFHsFVRCE

struct tag_FsykqfbjNNuE0BFHsFVRCE
{
  real_T f1[6];
  real_T f2;
};

#endif                                 //struct_tag_FsykqfbjNNuE0BFHsFVRCE

#ifndef typedef_g_cell_joint_trajectory_plann_T
#define typedef_g_cell_joint_trajectory_plann_T

typedef struct tag_FsykqfbjNNuE0BFHsFVRCE g_cell_joint_trajectory_plann_T;

#endif                                 //typedef_g_cell_joint_trajectory_plann_T

#ifndef struct_tag_5zdrOwEBCNIh1LgbqnM3yB
#define struct_tag_5zdrOwEBCNIh1LgbqnM3yB

struct tag_5zdrOwEBCNIh1LgbqnM3yB
{
  g_cell_joint_trajectory_plann_T ParsedResults;
};

#endif                                 //struct_tag_5zdrOwEBCNIh1LgbqnM3yB

#ifndef typedef_c_robotics_core_internal_code_T
#define typedef_c_robotics_core_internal_code_T

typedef struct tag_5zdrOwEBCNIh1LgbqnM3yB c_robotics_core_internal_code_T;

#endif                                 //typedef_c_robotics_core_internal_code_T

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

#ifndef struct_tag_8Y13cAEkrxA1KBxYU14FTD
#define struct_tag_8Y13cAEkrxA1KBxYU14FTD

struct tag_8Y13cAEkrxA1KBxYU14FTD
{
  int32_T isInitialized;
  boolean_T TunablePropsChanged;
  cell_wrap_joint_trajectory_pl_T inputVarSize[4];
  real_T PeakVelocity[2];
  real_T Acceleration[2];
};

#endif                                 //struct_tag_8Y13cAEkrxA1KBxYU14FTD

#ifndef typedef_robotics_slcore_internal_bloc_T
#define typedef_robotics_slcore_internal_bloc_T

typedef struct tag_8Y13cAEkrxA1KBxYU14FTD robotics_slcore_internal_bloc_T;

#endif                                 //typedef_robotics_slcore_internal_bloc_T

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

#ifndef typedef_robotics_slcore_internal_bl_a_T
#define typedef_robotics_slcore_internal_bl_a_T

typedef struct tag_pzFePiHQF4SDrxHoNaa5Y robotics_slcore_internal_bl_a_T;

#endif                                 //typedef_robotics_slcore_internal_bl_a_T

#ifndef struct_emxArray_tag_06c2DDfmr4zcnTqhww
#define struct_emxArray_tag_06c2DDfmr4zcnTqhww

struct emxArray_tag_06c2DDfmr4zcnTqhww
{
  s_06c2DDfmr4zcnTqhww20ZC_join_T data[6];
  int32_T size;
};

#endif                                 //struct_emxArray_tag_06c2DDfmr4zcnTqhww

#ifndef typedef_emxArray_s_06c2DDfmr4zcnTqhww_T
#define typedef_emxArray_s_06c2DDfmr4zcnTqhww_T

typedef struct emxArray_tag_06c2DDfmr4zcnTqhww emxArray_s_06c2DDfmr4zcnTqhww_T;

#endif                                 //typedef_emxArray_s_06c2DDfmr4zcnTqhww_T

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
