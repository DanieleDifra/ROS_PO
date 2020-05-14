//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: dynamic_simulation_types.h
//
// Code generated for Simulink model 'dynamic_simulation'.
//
// Model version                  : 1.125
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Thu May 14 23:02:45 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_dynamic_simulation_types_h_
#define RTW_HEADER_dynamic_simulation_types_h_
#include "rtwtypes.h"

// Model Code Variants
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_

typedef struct {
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
} SL_Bus_ROSVariableLengthArrayInfo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_dynamic_simulation_ros_time_Time_
#define DEFINED_TYPEDEF_FOR_SL_Bus_dynamic_simulation_ros_time_Time_

// MsgType=ros_time/Time
typedef struct {
  real_T Sec;
  real_T Nsec;
} SL_Bus_dynamic_simulation_ros_time_Time;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_dynamic_simulation_std_msgs_Header_
#define DEFINED_TYPEDEF_FOR_SL_Bus_dynamic_simulation_std_msgs_Header_

// MsgType=std_msgs/Header
typedef struct {
  uint32_T Seq;

  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=FrameId_SL_Info:TruncateAction=warn 
  uint8_T FrameId[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=FrameId
  SL_Bus_ROSVariableLengthArrayInfo FrameId_SL_Info;

  // MsgType=ros_time/Time
  SL_Bus_dynamic_simulation_ros_time_Time Stamp;
} SL_Bus_dynamic_simulation_std_msgs_Header;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_dynamic_simulation_geometry_msgs_Point_
#define DEFINED_TYPEDEF_FOR_SL_Bus_dynamic_simulation_geometry_msgs_Point_

// MsgType=geometry_msgs/Point
typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
} SL_Bus_dynamic_simulation_geometry_msgs_Point;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_dynamic_simulation_geometry_msgs_Quaternion_
#define DEFINED_TYPEDEF_FOR_SL_Bus_dynamic_simulation_geometry_msgs_Quaternion_

// MsgType=geometry_msgs/Quaternion
typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
  real_T W;
} SL_Bus_dynamic_simulation_geometry_msgs_Quaternion;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_dynamic_simulation_geometry_msgs_Pose_
#define DEFINED_TYPEDEF_FOR_SL_Bus_dynamic_simulation_geometry_msgs_Pose_

// MsgType=geometry_msgs/Pose
typedef struct {
  // MsgType=geometry_msgs/Point
  SL_Bus_dynamic_simulation_geometry_msgs_Point Position;

  // MsgType=geometry_msgs/Quaternion
  SL_Bus_dynamic_simulation_geometry_msgs_Quaternion Orientation;
} SL_Bus_dynamic_simulation_geometry_msgs_Pose;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_dynamic_simulation_geometry_msgs_PoseStamped_
#define DEFINED_TYPEDEF_FOR_SL_Bus_dynamic_simulation_geometry_msgs_PoseStamped_

// MsgType=geometry_msgs/PoseStamped
typedef struct {
  // MsgType=std_msgs/Header
  SL_Bus_dynamic_simulation_std_msgs_Header Header;

  // MsgType=geometry_msgs/Pose
  SL_Bus_dynamic_simulation_geometry_msgs_Pose Pose;
} SL_Bus_dynamic_simulation_geometry_msgs_PoseStamped;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_dynamic_simulation_geometry_msgs_Vector3_
#define DEFINED_TYPEDEF_FOR_SL_Bus_dynamic_simulation_geometry_msgs_Vector3_

// MsgType=geometry_msgs/Vector3
typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
} SL_Bus_dynamic_simulation_geometry_msgs_Vector3;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_dynamic_simulation_geometry_msgs_Twist_
#define DEFINED_TYPEDEF_FOR_SL_Bus_dynamic_simulation_geometry_msgs_Twist_

// MsgType=geometry_msgs/Twist
typedef struct {
  // MsgType=geometry_msgs/Vector3
  SL_Bus_dynamic_simulation_geometry_msgs_Vector3 Linear;

  // MsgType=geometry_msgs/Vector3
  SL_Bus_dynamic_simulation_geometry_msgs_Vector3 Angular;
} SL_Bus_dynamic_simulation_geometry_msgs_Twist;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_dynamic_simulation_geometry_msgs_TwistStamped_
#define DEFINED_TYPEDEF_FOR_SL_Bus_dynamic_simulation_geometry_msgs_TwistStamped_

// MsgType=geometry_msgs/TwistStamped
typedef struct {
  // MsgType=std_msgs/Header
  SL_Bus_dynamic_simulation_std_msgs_Header Header;

  // MsgType=geometry_msgs/Twist
  SL_Bus_dynamic_simulation_geometry_msgs_Twist Twist;
} SL_Bus_dynamic_simulation_geometry_msgs_TwistStamped;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_dynamic_simulation_std_msgs_String_
#define DEFINED_TYPEDEF_FOR_SL_Bus_dynamic_simulation_std_msgs_String_

// MsgType=std_msgs/String
typedef struct {
  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=Data_SL_Info:TruncateAction=warn 
  uint8_T Data[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Data
  SL_Bus_ROSVariableLengthArrayInfo Data_SL_Info;
} SL_Bus_dynamic_simulation_std_msgs_String;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_dynamic_simulation_sensor_msgs_JointState_
#define DEFINED_TYPEDEF_FOR_SL_Bus_dynamic_simulation_sensor_msgs_JointState_

// MsgType=sensor_msgs/JointState
typedef struct {
  // MsgType=std_msgs/String:PrimitiveROSType=string[]:IsVarLen=1:VarLenCategory=data:VarLenElem=Name_SL_Info:TruncateAction=warn 
  SL_Bus_dynamic_simulation_std_msgs_String Name[16];

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
  SL_Bus_dynamic_simulation_std_msgs_Header Header;
} SL_Bus_dynamic_simulation_sensor_msgs_JointState;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_dynamic_simulation_rosgraph_msgs_Clock_
#define DEFINED_TYPEDEF_FOR_SL_Bus_dynamic_simulation_rosgraph_msgs_Clock_

// MsgType=rosgraph_msgs/Clock
typedef struct {
  // MsgType=ros_time/Time
  SL_Bus_dynamic_simulation_ros_time_Time Clock_;
} SL_Bus_dynamic_simulation_rosgraph_msgs_Clock;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_dynamic_simulation_std_msgs_MultiArrayDimension_
#define DEFINED_TYPEDEF_FOR_SL_Bus_dynamic_simulation_std_msgs_MultiArrayDimension_

// MsgType=std_msgs/MultiArrayDimension
typedef struct {
  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=Label_SL_Info:TruncateAction=warn 
  uint8_T Label[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Label
  SL_Bus_ROSVariableLengthArrayInfo Label_SL_Info;
  uint32_T Size;
  uint32_T Stride;
} SL_Bus_dynamic_simulation_std_msgs_MultiArrayDimension;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_dynamic_simulation_std_msgs_MultiArrayLayout_
#define DEFINED_TYPEDEF_FOR_SL_Bus_dynamic_simulation_std_msgs_MultiArrayLayout_

// MsgType=std_msgs/MultiArrayLayout
typedef struct {
  uint32_T DataOffset;

  // MsgType=std_msgs/MultiArrayDimension:IsVarLen=1:VarLenCategory=data:VarLenElem=Dim_SL_Info:TruncateAction=warn 
  SL_Bus_dynamic_simulation_std_msgs_MultiArrayDimension Dim[16];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Dim
  SL_Bus_ROSVariableLengthArrayInfo Dim_SL_Info;
} SL_Bus_dynamic_simulation_std_msgs_MultiArrayLayout;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_dynamic_simulation_std_msgs_Float64MultiArray_
#define DEFINED_TYPEDEF_FOR_SL_Bus_dynamic_simulation_std_msgs_Float64MultiArray_

// MsgType=std_msgs/Float64MultiArray
typedef struct {
  // IsVarLen=1:VarLenCategory=data:VarLenElem=Data_SL_Info:TruncateAction=warn
  real_T Data[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Data
  SL_Bus_ROSVariableLengthArrayInfo Data_SL_Info;

  // MsgType=std_msgs/MultiArrayLayout
  SL_Bus_dynamic_simulation_std_msgs_MultiArrayLayout Layout;
} SL_Bus_dynamic_simulation_std_msgs_Float64MultiArray;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_cZjEk0xuhkMBv51ViHEB3B_
#define DEFINED_TYPEDEF_FOR_struct_cZjEk0xuhkMBv51ViHEB3B_

typedef struct {
  real_T NameLength;
  uint8_T Name[13];
  real_T ParentIndex;
  real_T NumChildren;
  real_T ChildrenIndices[8];
  real_T Mass;
  real_T CenterOfMass[3];
  real_T Inertia[9];
  real_T SpatialInertia[36];
} struct_cZjEk0xuhkMBv51ViHEB3B;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_00NLo0To7Ffzi993Vbz1mG_
#define DEFINED_TYPEDEF_FOR_struct_00NLo0To7Ffzi993Vbz1mG_

typedef struct {
  uint8_T Type;
  real_T NameLength;
  uint8_T Name[15];
  real_T VelocityNumber;
  real_T PositionNumber;
  real_T MotionSubspace[36];
  real_T JointAxis[3];
  real_T PositionLimits[14];
  real_T HomePosition[7];
  real_T JointToParentTransform[16];
  real_T ChildToJointTransform[16];
} struct_00NLo0To7Ffzi993Vbz1mG;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_OIFmJDQYndD4J3m8EdJEvH_
#define DEFINED_TYPEDEF_FOR_struct_OIFmJDQYndD4J3m8EdJEvH_

typedef struct {
  real_T NumBodies;
  real_T Gravity[3];
  real_T NumNonFixedBodies;
  real_T PositionNumber;
  real_T VelocityNumber;
  real_T PositionDoFMap[16];
  real_T VelocityDoFMap[16];
  real_T MaxNameLength;
  real_T MaxJointPositionNumber;
  uint8_T DataFormat;
  real_T JointPositionLimits[12];
  struct_cZjEk0xuhkMBv51ViHEB3B Bodies[9];
  struct_00NLo0To7Ffzi993Vbz1mG Joints[9];
} struct_OIFmJDQYndD4J3m8EdJEvH;

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

#ifndef struct_emxArray_char_T
#define struct_emxArray_char_T

struct emxArray_char_T
{
  char_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_char_T

#ifndef typedef_emxArray_char_T_dynamic_simul_T
#define typedef_emxArray_char_T_dynamic_simul_T

typedef struct emxArray_char_T emxArray_char_T_dynamic_simul_T;

#endif                                 //typedef_emxArray_char_T_dynamic_simul_T

#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_real_T

#ifndef typedef_emxArray_real_T_dynamic_simul_T
#define typedef_emxArray_real_T_dynamic_simul_T

typedef struct emxArray_real_T emxArray_real_T_dynamic_simul_T;

#endif                                 //typedef_emxArray_real_T_dynamic_simul_T

// Custom Type definition for MATLAB Function: '<Root>/Assign to JointState msg' 
#ifndef struct_tag_bT1sNYKeiTHCFMEFtQq2wB
#define struct_tag_bT1sNYKeiTHCFMEFtQq2wB

struct tag_bT1sNYKeiTHCFMEFtQq2wB
{
  char_T f1[11];
};

#endif                                 //struct_tag_bT1sNYKeiTHCFMEFtQq2wB

#ifndef typedef_cell_wrap_0_dynamic_simulatio_T
#define typedef_cell_wrap_0_dynamic_simulatio_T

typedef struct tag_bT1sNYKeiTHCFMEFtQq2wB cell_wrap_0_dynamic_simulatio_T;

#endif                                 //typedef_cell_wrap_0_dynamic_simulatio_T

#ifndef struct_tag_sS24XwgKvOMFYbEXa7sc1bB
#define struct_tag_sS24XwgKvOMFYbEXa7sc1bB

struct tag_sS24XwgKvOMFYbEXa7sc1bB
{
  real_T NameLength;
  uint8_T Name[13];
  real_T ParentIndex;
  real_T NumChildren;
  real_T ChildrenIndices[8];
  real_T Mass;
  real_T CenterOfMass[3];
  real_T Inertia[9];
  real_T SpatialInertia[36];
};

#endif                                 //struct_tag_sS24XwgKvOMFYbEXa7sc1bB

#ifndef typedef_sS24XwgKvOMFYbEXa7sc1bB_dynam_T
#define typedef_sS24XwgKvOMFYbEXa7sc1bB_dynam_T

typedef struct tag_sS24XwgKvOMFYbEXa7sc1bB sS24XwgKvOMFYbEXa7sc1bB_dynam_T;

#endif                                 //typedef_sS24XwgKvOMFYbEXa7sc1bB_dynam_T

#ifndef struct_tag_sg9AtqjjkXglWcXWTAtiJmG
#define struct_tag_sg9AtqjjkXglWcXWTAtiJmG

struct tag_sg9AtqjjkXglWcXWTAtiJmG
{
  uint8_T Type;
  real_T NameLength;
  uint8_T Name[15];
  real_T VelocityNumber;
  real_T PositionNumber;
  real_T MotionSubspace[36];
  real_T JointAxis[3];
  real_T PositionLimits[14];
  real_T HomePosition[7];
  real_T JointToParentTransform[16];
  real_T ChildToJointTransform[16];
};

#endif                                 //struct_tag_sg9AtqjjkXglWcXWTAtiJmG

#ifndef typedef_sg9AtqjjkXglWcXWTAtiJmG_dynam_T
#define typedef_sg9AtqjjkXglWcXWTAtiJmG_dynam_T

typedef struct tag_sg9AtqjjkXglWcXWTAtiJmG sg9AtqjjkXglWcXWTAtiJmG_dynam_T;

#endif                                 //typedef_sg9AtqjjkXglWcXWTAtiJmG_dynam_T

#ifndef struct_tag_8EP4ctv7s0SDnh3V6WCLOG
#define struct_tag_8EP4ctv7s0SDnh3V6WCLOG

struct tag_8EP4ctv7s0SDnh3V6WCLOG
{
  real_T f1[16];
};

#endif                                 //struct_tag_8EP4ctv7s0SDnh3V6WCLOG

#ifndef typedef_f_cell_wrap_dynamic_simulatio_T
#define typedef_f_cell_wrap_dynamic_simulatio_T

typedef struct tag_8EP4ctv7s0SDnh3V6WCLOG f_cell_wrap_dynamic_simulatio_T;

#endif                                 //typedef_f_cell_wrap_dynamic_simulatio_T

#ifndef struct_tag_rHHvNgfpb6xtLaCol8pZFD
#define struct_tag_rHHvNgfpb6xtLaCol8pZFD

struct tag_rHHvNgfpb6xtLaCol8pZFD
{
  emxArray_char_T_dynamic_simul_T *Type;
  real_T PositionNumber;
  emxArray_real_T_dynamic_simul_T *MotionSubspace;
  real_T JointToParentTransform[16];
  real_T ChildToJointTransform[16];
  real_T JointAxisInternal[3];
};

#endif                                 //struct_tag_rHHvNgfpb6xtLaCol8pZFD

#ifndef typedef_c_rigidBodyJoint_dynamic_simu_T
#define typedef_c_rigidBodyJoint_dynamic_simu_T

typedef struct tag_rHHvNgfpb6xtLaCol8pZFD c_rigidBodyJoint_dynamic_simu_T;

#endif                                 //typedef_c_rigidBodyJoint_dynamic_simu_T

#ifndef struct_tag_C18ji2bP7k8qnDZFFReimG
#define struct_tag_C18ji2bP7k8qnDZFFReimG

struct tag_C18ji2bP7k8qnDZFFReimG
{
  real_T Index;
  emxArray_char_T_dynamic_simul_T *NameInternal;
  c_rigidBodyJoint_dynamic_simu_T JointInternal;
  real_T ParentIndex;
};

#endif                                 //struct_tag_C18ji2bP7k8qnDZFFReimG

#ifndef typedef_n_robotics_manip_internal_Rig_T
#define typedef_n_robotics_manip_internal_Rig_T

typedef struct tag_C18ji2bP7k8qnDZFFReimG n_robotics_manip_internal_Rig_T;

#endif                                 //typedef_n_robotics_manip_internal_Rig_T

#ifndef struct_tag_9ECOrffzte7r0XovPIluy
#define struct_tag_9ECOrffzte7r0XovPIluy

struct tag_9ECOrffzte7r0XovPIluy
{
  emxArray_char_T_dynamic_simul_T *NameInternal;
  c_rigidBodyJoint_dynamic_simu_T JointInternal;
};

#endif                                 //struct_tag_9ECOrffzte7r0XovPIluy

#ifndef typedef_o_robotics_manip_internal_Rig_T
#define typedef_o_robotics_manip_internal_Rig_T

typedef struct tag_9ECOrffzte7r0XovPIluy o_robotics_manip_internal_Rig_T;

#endif                                 //typedef_o_robotics_manip_internal_Rig_T

#ifndef struct_tag_TOxUAI1qvqVBKNJvydpO7F
#define struct_tag_TOxUAI1qvqVBKNJvydpO7F

struct tag_TOxUAI1qvqVBKNJvydpO7F
{
  real_T NumBodies;
  o_robotics_manip_internal_Rig_T Base;
  n_robotics_manip_internal_Rig_T *Bodies[8];
  real_T PositionNumber;
  real_T VelocityNumber;
  real_T PositionDoFMap[16];
};

#endif                                 //struct_tag_TOxUAI1qvqVBKNJvydpO7F

#ifndef typedef_p_robotics_manip_internal_Rig_T
#define typedef_p_robotics_manip_internal_Rig_T

typedef struct tag_TOxUAI1qvqVBKNJvydpO7F p_robotics_manip_internal_Rig_T;

#endif                                 //typedef_p_robotics_manip_internal_Rig_T

#ifndef struct_tag_8WuLxcHshihGuDAyFMTJLD
#define struct_tag_8WuLxcHshihGuDAyFMTJLD

struct tag_8WuLxcHshihGuDAyFMTJLD
{
  int32_T isInitialized;
  p_robotics_manip_internal_Rig_T TreeInternal;
};

#endif                                 //struct_tag_8WuLxcHshihGuDAyFMTJLD

#ifndef typedef_robotics_slmanip_internal_blo_T
#define typedef_robotics_slmanip_internal_blo_T

typedef struct tag_8WuLxcHshihGuDAyFMTJLD robotics_slmanip_internal_blo_T;

#endif                                 //typedef_robotics_slmanip_internal_blo_T

#ifndef struct_tag_2xAJzS7rvIyFzfuC7FOGlD
#define struct_tag_2xAJzS7rvIyFzfuC7FOGlD

struct tag_2xAJzS7rvIyFzfuC7FOGlD
{
  emxArray_char_T_dynamic_simul_T *Type;
  real_T PositionNumber;
  real_T JointToParentTransform[16];
  real_T ChildToJointTransform[16];
  real_T JointAxisInternal[3];
};

#endif                                 //struct_tag_2xAJzS7rvIyFzfuC7FOGlD

#ifndef typedef_c_rigidBodyJoint_dynamic_si_k_T
#define typedef_c_rigidBodyJoint_dynamic_si_k_T

typedef struct tag_2xAJzS7rvIyFzfuC7FOGlD c_rigidBodyJoint_dynamic_si_k_T;

#endif                                 //typedef_c_rigidBodyJoint_dynamic_si_k_T

#ifndef struct_tag_BPMfb1nwAW0cVckEDiFVnG
#define struct_tag_BPMfb1nwAW0cVckEDiFVnG

struct tag_BPMfb1nwAW0cVckEDiFVnG
{
  emxArray_char_T_dynamic_simul_T *NameInternal;
  c_rigidBodyJoint_dynamic_si_k_T JointInternal;
  real_T ParentIndex;
};

#endif                                 //struct_tag_BPMfb1nwAW0cVckEDiFVnG

#ifndef typedef_n_robotics_manip_internal_R_k_T
#define typedef_n_robotics_manip_internal_R_k_T

typedef struct tag_BPMfb1nwAW0cVckEDiFVnG n_robotics_manip_internal_R_k_T;

#endif                                 //typedef_n_robotics_manip_internal_R_k_T

#ifndef struct_tag_EpnOXkcFkOFmcaZiu2CuCD
#define struct_tag_EpnOXkcFkOFmcaZiu2CuCD

struct tag_EpnOXkcFkOFmcaZiu2CuCD
{
  emxArray_char_T_dynamic_simul_T *NameInternal;
  c_rigidBodyJoint_dynamic_si_k_T JointInternal;
};

#endif                                 //struct_tag_EpnOXkcFkOFmcaZiu2CuCD

#ifndef typedef_o_robotics_manip_internal_R_k_T
#define typedef_o_robotics_manip_internal_R_k_T

typedef struct tag_EpnOXkcFkOFmcaZiu2CuCD o_robotics_manip_internal_R_k_T;

#endif                                 //typedef_o_robotics_manip_internal_R_k_T

#ifndef struct_tag_30tOpKFHjBx7LOwU1qyzZE
#define struct_tag_30tOpKFHjBx7LOwU1qyzZE

struct tag_30tOpKFHjBx7LOwU1qyzZE
{
  real_T NumBodies;
  o_robotics_manip_internal_R_k_T Base;
  n_robotics_manip_internal_R_k_T *Bodies[8];
  real_T PositionNumber;
};

#endif                                 //struct_tag_30tOpKFHjBx7LOwU1qyzZE

#ifndef typedef_p_robotics_manip_internal_R_k_T
#define typedef_p_robotics_manip_internal_R_k_T

typedef struct tag_30tOpKFHjBx7LOwU1qyzZE p_robotics_manip_internal_R_k_T;

#endif                                 //typedef_p_robotics_manip_internal_R_k_T

#ifndef struct_tag_OhlsWxsKBuXkrIadR09rYF
#define struct_tag_OhlsWxsKBuXkrIadR09rYF

struct tag_OhlsWxsKBuXkrIadR09rYF
{
  int32_T isInitialized;
  p_robotics_manip_internal_R_k_T TreeInternal;
};

#endif                                 //struct_tag_OhlsWxsKBuXkrIadR09rYF

#ifndef typedef_robotics_slmanip_internal_b_k_T
#define typedef_robotics_slmanip_internal_b_k_T

typedef struct tag_OhlsWxsKBuXkrIadR09rYF robotics_slmanip_internal_b_k_T;

#endif                                 //typedef_robotics_slmanip_internal_b_k_T

#ifndef struct_tag_pGgszObO16I6TGXaEMnuXB
#define struct_tag_pGgszObO16I6TGXaEMnuXB

struct tag_pGgszObO16I6TGXaEMnuXB
{
  real_T f1[36];
};

#endif                                 //struct_tag_pGgszObO16I6TGXaEMnuXB

#ifndef typedef_f_cell_wrap_dynamic_simulat_k_T
#define typedef_f_cell_wrap_dynamic_simulat_k_T

typedef struct tag_pGgszObO16I6TGXaEMnuXB f_cell_wrap_dynamic_simulat_k_T;

#endif                                 //typedef_f_cell_wrap_dynamic_simulat_k_T

#ifndef struct_tag_QsLFVUgAtzQUBY1V99co0D
#define struct_tag_QsLFVUgAtzQUBY1V99co0D

struct tag_QsLFVUgAtzQUBY1V99co0D
{
  emxArray_char_T_dynamic_simul_T *Type;
  emxArray_real_T_dynamic_simul_T *MotionSubspace;
  real_T JointToParentTransform[16];
  real_T ChildToJointTransform[16];
  real_T JointAxisInternal[3];
};

#endif                                 //struct_tag_QsLFVUgAtzQUBY1V99co0D

#ifndef typedef_c_rigidBodyJoint_dynamic_s_kq_T
#define typedef_c_rigidBodyJoint_dynamic_s_kq_T

typedef struct tag_QsLFVUgAtzQUBY1V99co0D c_rigidBodyJoint_dynamic_s_kq_T;

#endif                                 //typedef_c_rigidBodyJoint_dynamic_s_kq_T

#ifndef struct_tag_IYTryndM9hCl2aQvRVOEpC
#define struct_tag_IYTryndM9hCl2aQvRVOEpC

struct tag_IYTryndM9hCl2aQvRVOEpC
{
  real_T Index;
  c_rigidBodyJoint_dynamic_s_kq_T JointInternal;
  real_T ParentIndex;
  real_T SpatialInertia[36];
};

#endif                                 //struct_tag_IYTryndM9hCl2aQvRVOEpC

#ifndef typedef_n_robotics_manip_internal__kq_T
#define typedef_n_robotics_manip_internal__kq_T

typedef struct tag_IYTryndM9hCl2aQvRVOEpC n_robotics_manip_internal__kq_T;

#endif                                 //typedef_n_robotics_manip_internal__kq_T

#ifndef struct_tag_Y04n94zUMq8rhLX3OxwtfD
#define struct_tag_Y04n94zUMq8rhLX3OxwtfD

struct tag_Y04n94zUMq8rhLX3OxwtfD
{
  c_rigidBodyJoint_dynamic_s_kq_T JointInternal;
};

#endif                                 //struct_tag_Y04n94zUMq8rhLX3OxwtfD

#ifndef typedef_o_robotics_manip_internal__kq_T
#define typedef_o_robotics_manip_internal__kq_T

typedef struct tag_Y04n94zUMq8rhLX3OxwtfD o_robotics_manip_internal__kq_T;

#endif                                 //typedef_o_robotics_manip_internal__kq_T

#ifndef struct_tag_CGxMVC4zETAqBEj5tfaJlD
#define struct_tag_CGxMVC4zETAqBEj5tfaJlD

struct tag_CGxMVC4zETAqBEj5tfaJlD
{
  real_T NumBodies;
  o_robotics_manip_internal__kq_T Base;
  real_T Gravity[3];
  n_robotics_manip_internal__kq_T *Bodies[8];
  real_T VelocityNumber;
  real_T PositionDoFMap[16];
  real_T VelocityDoFMap[16];
};

#endif                                 //struct_tag_CGxMVC4zETAqBEj5tfaJlD

#ifndef typedef_p_robotics_manip_internal__kq_T
#define typedef_p_robotics_manip_internal__kq_T

typedef struct tag_CGxMVC4zETAqBEj5tfaJlD p_robotics_manip_internal__kq_T;

#endif                                 //typedef_p_robotics_manip_internal__kq_T

#ifndef struct_tag_M95ulqvrzjlE3M76uzDFDD
#define struct_tag_M95ulqvrzjlE3M76uzDFDD

struct tag_M95ulqvrzjlE3M76uzDFDD
{
  int32_T isInitialized;
  p_robotics_manip_internal__kq_T TreeInternal;
};

#endif                                 //struct_tag_M95ulqvrzjlE3M76uzDFDD

#ifndef typedef_robotics_slmanip_internal__kq_T
#define typedef_robotics_slmanip_internal__kq_T

typedef struct tag_M95ulqvrzjlE3M76uzDFDD robotics_slmanip_internal__kq_T;

#endif                                 //typedef_robotics_slmanip_internal__kq_T

#ifndef struct_emxArray_tag_8EP4ctv7s0SDnh3V6W
#define struct_emxArray_tag_8EP4ctv7s0SDnh3V6W

struct emxArray_tag_8EP4ctv7s0SDnh3V6W
{
  f_cell_wrap_dynamic_simulatio_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_tag_8EP4ctv7s0SDnh3V6W

#ifndef typedef_emxArray_f_cell_wrap_dynamic__T
#define typedef_emxArray_f_cell_wrap_dynamic__T

typedef struct emxArray_tag_8EP4ctv7s0SDnh3V6W emxArray_f_cell_wrap_dynamic__T;

#endif                                 //typedef_emxArray_f_cell_wrap_dynamic__T

#ifndef struct_emxArray_tag_pGgszObO16I6TGXaEM
#define struct_emxArray_tag_pGgszObO16I6TGXaEM

struct emxArray_tag_pGgszObO16I6TGXaEM
{
  f_cell_wrap_dynamic_simulat_k_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_tag_pGgszObO16I6TGXaEM

#ifndef typedef_emxArray_f_cell_wrap_dynami_k_T
#define typedef_emxArray_f_cell_wrap_dynami_k_T

typedef struct emxArray_tag_pGgszObO16I6TGXaEM emxArray_f_cell_wrap_dynami_k_T;

#endif                                 //typedef_emxArray_f_cell_wrap_dynami_k_T

// Parameters (default storage)
typedef struct P_dynamic_simulation_T_ P_dynamic_simulation_T;

// Forward declaration for rtModel
typedef struct tag_RTM_dynamic_simulation_T RT_MODEL_dynamic_simulation_T;

#endif                                // RTW_HEADER_dynamic_simulation_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
