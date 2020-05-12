//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: kinsim_2link_planar_types.h
//
// Code generated for Simulink model 'kinsim_2link_planar'.
//
// Model version                  : 1.120
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Tue May 12 21:46:34 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_kinsim_2link_planar_types_h_
#define RTW_HEADER_kinsim_2link_planar_types_h_
#include "rtwtypes.h"

// Model Code Variants
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_

typedef struct {
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
} SL_Bus_ROSVariableLengthArrayInfo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_kinsim_2link_planar_std_msgs_String_
#define DEFINED_TYPEDEF_FOR_SL_Bus_kinsim_2link_planar_std_msgs_String_

// MsgType=std_msgs/String
typedef struct {
  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=Data_SL_Info:TruncateAction=warn 
  uint8_T Data[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Data
  SL_Bus_ROSVariableLengthArrayInfo Data_SL_Info;
} SL_Bus_kinsim_2link_planar_std_msgs_String;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_kinsim_2link_planar_ros_time_Time_
#define DEFINED_TYPEDEF_FOR_SL_Bus_kinsim_2link_planar_ros_time_Time_

// MsgType=ros_time/Time
typedef struct {
  real_T Sec;
  real_T Nsec;
} SL_Bus_kinsim_2link_planar_ros_time_Time;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_kinsim_2link_planar_std_msgs_Header_
#define DEFINED_TYPEDEF_FOR_SL_Bus_kinsim_2link_planar_std_msgs_Header_

// MsgType=std_msgs/Header
typedef struct {
  uint32_T Seq;

  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=FrameId_SL_Info:TruncateAction=warn 
  uint8_T FrameId[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=FrameId
  SL_Bus_ROSVariableLengthArrayInfo FrameId_SL_Info;

  // MsgType=ros_time/Time
  SL_Bus_kinsim_2link_planar_ros_time_Time Stamp;
} SL_Bus_kinsim_2link_planar_std_msgs_Header;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_kinsim_2link_planar_sensor_msgs_JointState_
#define DEFINED_TYPEDEF_FOR_SL_Bus_kinsim_2link_planar_sensor_msgs_JointState_

// MsgType=sensor_msgs/JointState
typedef struct {
  // MsgType=std_msgs/String:PrimitiveROSType=string[]:IsVarLen=1:VarLenCategory=data:VarLenElem=Name_SL_Info:TruncateAction=warn 
  SL_Bus_kinsim_2link_planar_std_msgs_String Name[16];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Name
  SL_Bus_ROSVariableLengthArrayInfo Name_SL_Info;

  // IsVarLen=1:VarLenCategory=data:VarLenElem=Position_SL_Info:TruncateAction=warn 
  real_T Position[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Position
  SL_Bus_ROSVariableLengthArrayInfo Position_SL_Info;

  // IsVarLen=1:VarLenCategory=data:VarLenElem=Velocity_SL_Info:TruncateAction=warn 
  real_T Velocity[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Velocity
  SL_Bus_ROSVariableLengthArrayInfo Velocity_SL_Info;

  // IsVarLen=1:VarLenCategory=data:VarLenElem=Effort_SL_Info:TruncateAction=warn 
  real_T Effort[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Effort
  SL_Bus_ROSVariableLengthArrayInfo Effort_SL_Info;

  // MsgType=std_msgs/Header
  SL_Bus_kinsim_2link_planar_std_msgs_Header Header;
} SL_Bus_kinsim_2link_planar_sensor_msgs_JointState;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_kinsim_2link_planar_rosgraph_msgs_Clock_
#define DEFINED_TYPEDEF_FOR_SL_Bus_kinsim_2link_planar_rosgraph_msgs_Clock_

// MsgType=rosgraph_msgs/Clock
typedef struct {
  // MsgType=ros_time/Time
  SL_Bus_kinsim_2link_planar_ros_time_Time Clock_;
} SL_Bus_kinsim_2link_planar_rosgraph_msgs_Clock;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_kinsim_2link_planar_ros_time_Duration_
#define DEFINED_TYPEDEF_FOR_SL_Bus_kinsim_2link_planar_ros_time_Duration_

// MsgType=ros_time/Duration
typedef struct {
  real_T Sec;
  real_T Nsec;
} SL_Bus_kinsim_2link_planar_ros_time_Duration;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_kinsim_2link_planar_JointTrajectoryPoint_1csgl9_
#define DEFINED_TYPEDEF_FOR_SL_Bus_kinsim_2link_planar_JointTrajectoryPoint_1csgl9_

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
  SL_Bus_kinsim_2link_planar_ros_time_Duration TimeFromStart;
} SL_Bus_kinsim_2link_planar_JointTrajectoryPoint_1csgl9;

#endif

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

// Custom Type definition for MATLAB Function: '<Root>/Assign to JointState msg' 
#ifndef struct_tag_rGJxeAeFOtyQ9icBnCewuB
#define struct_tag_rGJxeAeFOtyQ9icBnCewuB

struct tag_rGJxeAeFOtyQ9icBnCewuB
{
  char_T f1[7];
};

#endif                                 //struct_tag_rGJxeAeFOtyQ9icBnCewuB

#ifndef typedef_cell_wrap_0_kinsim_2link_plan_T
#define typedef_cell_wrap_0_kinsim_2link_plan_T

typedef struct tag_rGJxeAeFOtyQ9icBnCewuB cell_wrap_0_kinsim_2link_plan_T;

#endif                                 //typedef_cell_wrap_0_kinsim_2link_plan_T

// Parameters (default storage)
typedef struct P_kinsim_2link_planar_T_ P_kinsim_2link_planar_T;

// Forward declaration for rtModel
typedef struct tag_RTM_kinsim_2link_planar_T RT_MODEL_kinsim_2link_planar_T;

#endif                               // RTW_HEADER_kinsim_2link_planar_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
