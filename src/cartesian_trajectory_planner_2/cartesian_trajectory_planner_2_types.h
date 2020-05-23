//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: cartesian_trajectory_planner_2_types.h
//
// Code generated for Simulink model 'cartesian_trajectory_planner_2'.
//
// Model version                  : 1.136
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Sat May 23 17:30:41 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_cartesian_trajectory_planner_2_types_h_
#define RTW_HEADER_cartesian_trajectory_planner_2_types_h_
#include "rtwtypes.h"

// Model Code Variants
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_

typedef struct {
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
} SL_Bus_ROSVariableLengthArrayInfo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_cartesian_trajectory_planner_2_ros_time_Duration_
#define DEFINED_TYPEDEF_FOR_SL_Bus_cartesian_trajectory_planner_2_ros_time_Duration_

// MsgType=ros_time/Duration
typedef struct {
  real_T Sec;
  real_T Nsec;
} SL_Bus_cartesian_trajectory_planner_2_ros_time_Duration;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_cartesian_trajectory_plan_JointTrajectoryPoint_na5m06_
#define DEFINED_TYPEDEF_FOR_SL_Bus_cartesian_trajectory_plan_JointTrajectoryPoint_na5m06_

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
  SL_Bus_cartesian_trajectory_planner_2_ros_time_Duration TimeFromStart;
} SL_Bus_cartesian_trajectory_plan_JointTrajectoryPoint_na5m06;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_cartesian_trajectory_planner_2_ros_time_Time_
#define DEFINED_TYPEDEF_FOR_SL_Bus_cartesian_trajectory_planner_2_ros_time_Time_

// MsgType=ros_time/Time
typedef struct {
  real_T Sec;
  real_T Nsec;
} SL_Bus_cartesian_trajectory_planner_2_ros_time_Time;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_cartesian_trajectory_planner_2_rosgraph_msgs_Clock_
#define DEFINED_TYPEDEF_FOR_SL_Bus_cartesian_trajectory_planner_2_rosgraph_msgs_Clock_

// MsgType=rosgraph_msgs/Clock
typedef struct {
  // MsgType=ros_time/Time
  SL_Bus_cartesian_trajectory_planner_2_ros_time_Time Clock_;
} SL_Bus_cartesian_trajectory_planner_2_rosgraph_msgs_Clock;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ikblock_info_bus_
#define DEFINED_TYPEDEF_FOR_ikblock_info_bus_

// Bus associated with Info output of the Inverse Kinematics block.
typedef struct {
  // Number of iterations run by the algorithm.
  real_T Iterations;

  // The magnitude of pose error for the solution compared to the desired end-effector pose. 
  real_T PoseErrorNorm;

  // Code that gives more details on the algorithm execution and what caused it to return. 
  uint16_T ExitFlag;

  // Integer describing whether the solution is within the tolerance (1) or is the best possible solution the algorithm could find (2). 
  uint8_T Status;
} ikblock_info_bus;

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

#ifndef DEFINED_TYPEDEF_FOR_struct_MH1icwhz7mkMseeLmnjXIE_
#define DEFINED_TYPEDEF_FOR_struct_MH1icwhz7mkMseeLmnjXIE_

typedef struct {
  boolean_T EnforceJointLimits;
  boolean_T AllowRandomRestart;
  real_T MaxIterations;
  real_T MaxTime;
  real_T GradientTolerance;
  real_T SolutionTolerance;
  real_T StepTolerance;
} struct_MH1icwhz7mkMseeLmnjXIE;

#endif

#ifndef struct_tag_d9Cd9FR17y4PCQL0V3usIB
#define struct_tag_d9Cd9FR17y4PCQL0V3usIB

struct tag_d9Cd9FR17y4PCQL0V3usIB
{
  real_T StartTime;
};

#endif                                 //struct_tag_d9Cd9FR17y4PCQL0V3usIB

#ifndef typedef_f_robotics_core_internal_Syst_T
#define typedef_f_robotics_core_internal_Syst_T

typedef struct tag_d9Cd9FR17y4PCQL0V3usIB f_robotics_core_internal_Syst_T;

#endif                                 //typedef_f_robotics_core_internal_Syst_T

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

#ifndef typedef_emxArray_char_T_cartesian_tra_T
#define typedef_emxArray_char_T_cartesian_tra_T

typedef struct emxArray_char_T emxArray_char_T_cartesian_tra_T;

#endif                                 //typedef_emxArray_char_T_cartesian_tra_T

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

#ifndef typedef_emxArray_real_T_cartesian_tra_T
#define typedef_emxArray_real_T_cartesian_tra_T

typedef struct emxArray_real_T emxArray_real_T_cartesian_tra_T;

#endif                                 //typedef_emxArray_real_T_cartesian_tra_T

#ifndef struct_emxArray_int32_T
#define struct_emxArray_int32_T

struct emxArray_int32_T
{
  int32_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_int32_T

#ifndef typedef_emxArray_int32_T_cartesian_tr_T
#define typedef_emxArray_int32_T_cartesian_tr_T

typedef struct emxArray_int32_T emxArray_int32_T_cartesian_tr_T;

#endif                                 //typedef_emxArray_int32_T_cartesian_tr_T

#ifndef struct_emxArray_boolean_T
#define struct_emxArray_boolean_T

struct emxArray_boolean_T
{
  boolean_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_boolean_T

#ifndef typedef_emxArray_boolean_T_cartesian__T
#define typedef_emxArray_boolean_T_cartesian__T

typedef struct emxArray_boolean_T emxArray_boolean_T_cartesian__T;

#endif                                 //typedef_emxArray_boolean_T_cartesian__T

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

#ifndef typedef_sS24XwgKvOMFYbEXa7sc1bB_carte_T
#define typedef_sS24XwgKvOMFYbEXa7sc1bB_carte_T

typedef struct tag_sS24XwgKvOMFYbEXa7sc1bB sS24XwgKvOMFYbEXa7sc1bB_carte_T;

#endif                                 //typedef_sS24XwgKvOMFYbEXa7sc1bB_carte_T

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

#ifndef typedef_sg9AtqjjkXglWcXWTAtiJmG_carte_T
#define typedef_sg9AtqjjkXglWcXWTAtiJmG_carte_T

typedef struct tag_sg9AtqjjkXglWcXWTAtiJmG sg9AtqjjkXglWcXWTAtiJmG_carte_T;

#endif                                 //typedef_sg9AtqjjkXglWcXWTAtiJmG_carte_T

#ifndef struct_tag_a9w8IowFHzogdOiVBZxs7
#define struct_tag_a9w8IowFHzogdOiVBZxs7

struct tag_a9w8IowFHzogdOiVBZxs7
{
  emxArray_char_T_cartesian_tra_T *Type;
  real_T VelocityNumber;
  real_T PositionNumber;
  emxArray_real_T_cartesian_tra_T *MotionSubspace;
  boolean_T InTree;
  real_T JointToParentTransform[16];
  real_T ChildToJointTransform[16];
  emxArray_char_T_cartesian_tra_T *NameInternal;
  emxArray_real_T_cartesian_tra_T *PositionLimitsInternal;
  emxArray_real_T_cartesian_tra_T *HomePositionInternal;
  real_T JointAxisInternal[3];
};

#endif                                 //struct_tag_a9w8IowFHzogdOiVBZxs7

#ifndef typedef_c_rigidBodyJoint_cartesian_tr_T
#define typedef_c_rigidBodyJoint_cartesian_tr_T

typedef struct tag_a9w8IowFHzogdOiVBZxs7 c_rigidBodyJoint_cartesian_tr_T;

#endif                                 //typedef_c_rigidBodyJoint_cartesian_tr_T

#ifndef struct_tag_Ba6v2kM2i80AQ3teGuWbQD
#define struct_tag_Ba6v2kM2i80AQ3teGuWbQD

struct tag_Ba6v2kM2i80AQ3teGuWbQD
{
  real_T Index;
  emxArray_char_T_cartesian_tra_T *NameInternal;
  c_rigidBodyJoint_cartesian_tr_T JointInternal;
  real_T ParentIndex;
  real_T MassInternal;
  real_T CenterOfMassInternal[3];
  real_T InertiaInternal[9];
  real_T SpatialInertia[36];
};

#endif                                 //struct_tag_Ba6v2kM2i80AQ3teGuWbQD

#ifndef typedef_v_robotics_manip_internal_Rig_T
#define typedef_v_robotics_manip_internal_Rig_T

typedef struct tag_Ba6v2kM2i80AQ3teGuWbQD v_robotics_manip_internal_Rig_T;

#endif                                 //typedef_v_robotics_manip_internal_Rig_T

#ifndef struct_tag_0AxmFhqo47Mw76v4Vj4AdE
#define struct_tag_0AxmFhqo47Mw76v4Vj4AdE

struct tag_0AxmFhqo47Mw76v4Vj4AdE
{
  real_T Index;
  emxArray_char_T_cartesian_tra_T *NameInternal;
  c_rigidBodyJoint_cartesian_tr_T *JointInternal;
  real_T ParentIndex;
};

#endif                                 //struct_tag_0AxmFhqo47Mw76v4Vj4AdE

#ifndef typedef_w_robotics_manip_internal_Rig_T
#define typedef_w_robotics_manip_internal_Rig_T

typedef struct tag_0AxmFhqo47Mw76v4Vj4AdE w_robotics_manip_internal_Rig_T;

#endif                                 //typedef_w_robotics_manip_internal_Rig_T

#ifndef struct_tag_GB8Etqd6oGuOYCER5vjKxC
#define struct_tag_GB8Etqd6oGuOYCER5vjKxC

struct tag_GB8Etqd6oGuOYCER5vjKxC
{
  real_T NumBodies;
  w_robotics_manip_internal_Rig_T Base;
  w_robotics_manip_internal_Rig_T *Bodies[8];
  real_T NumNonFixedBodies;
  real_T PositionNumber;
  real_T VelocityNumber;
  real_T PositionDoFMap[16];
  real_T VelocityDoFMap[16];
};

#endif                                 //struct_tag_GB8Etqd6oGuOYCER5vjKxC

#ifndef typedef_x_robotics_manip_internal_Rig_T
#define typedef_x_robotics_manip_internal_Rig_T

typedef struct tag_GB8Etqd6oGuOYCER5vjKxC x_robotics_manip_internal_Rig_T;

#endif                                 //typedef_x_robotics_manip_internal_Rig_T

#ifndef struct_tag_KnC6mlO1VBfnPbaet73v2G
#define struct_tag_KnC6mlO1VBfnPbaet73v2G

struct tag_KnC6mlO1VBfnPbaet73v2G
{
  x_robotics_manip_internal_Rig_T *Robot;
  real_T WeightMatrix[36];
  emxArray_char_T_cartesian_tra_T *BodyName;
  real_T Tform[16];
  emxArray_real_T_cartesian_tra_T *ErrTemp;
  real_T CostTemp;
  emxArray_real_T_cartesian_tra_T *GradTemp;
};

#endif                                 //struct_tag_KnC6mlO1VBfnPbaet73v2G

#ifndef typedef_f_robotics_manip_internal_IKE_T
#define typedef_f_robotics_manip_internal_IKE_T

typedef struct tag_KnC6mlO1VBfnPbaet73v2G f_robotics_manip_internal_IKE_T;

#endif                                 //typedef_f_robotics_manip_internal_IKE_T

#ifndef struct_tag_qVc3czRnWHHteds2In6P4B
#define struct_tag_qVc3czRnWHHteds2In6P4B

struct tag_qVc3czRnWHHteds2In6P4B
{
  char_T Name[22];
  emxArray_real_T_cartesian_tra_T *ConstraintMatrix;
  emxArray_real_T_cartesian_tra_T *ConstraintBound;
  boolean_T ConstraintsOn;
  real_T SolutionTolerance;
  boolean_T RandomRestart;
  f_robotics_manip_internal_IKE_T *ExtraArgs;
  real_T MaxNumIteration;
  real_T MaxTime;
  real_T SeedInternal[6];
  real_T MaxTimeInternal;
  real_T MaxNumIterationInternal;
  real_T StepTolerance;
  f_robotics_core_internal_Syst_T TimeObj;
  real_T GradientTolerance;
  real_T ArmijoRuleBeta;
  real_T ArmijoRuleSigma;
  f_robotics_core_internal_Syst_T TimeObjInternal;
};

#endif                                 //struct_tag_qVc3czRnWHHteds2In6P4B

#ifndef typedef_h_robotics_core_internal_Damp_T
#define typedef_h_robotics_core_internal_Damp_T

typedef struct tag_qVc3czRnWHHteds2In6P4B h_robotics_core_internal_Damp_T;

#endif                                 //typedef_h_robotics_core_internal_Damp_T

#ifndef struct_tag_uH0N8Zuc6JP0FtQwOfy7kE
#define struct_tag_uH0N8Zuc6JP0FtQwOfy7kE

struct tag_uH0N8Zuc6JP0FtQwOfy7kE
{
  real_T NumBodies;
  v_robotics_manip_internal_Rig_T Base;
  real_T Gravity[3];
  v_robotics_manip_internal_Rig_T *Bodies[8];
};

#endif                                 //struct_tag_uH0N8Zuc6JP0FtQwOfy7kE

#ifndef typedef_y_robotics_manip_internal_Rig_T
#define typedef_y_robotics_manip_internal_Rig_T

typedef struct tag_uH0N8Zuc6JP0FtQwOfy7kE y_robotics_manip_internal_Rig_T;

#endif                                 //typedef_y_robotics_manip_internal_Rig_T

#ifndef struct_tag_etdDPy7RU0or6X7LBQh1x
#define struct_tag_etdDPy7RU0or6X7LBQh1x

struct tag_etdDPy7RU0or6X7LBQh1x
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  h_robotics_core_internal_Damp_T *Solver;
  emxArray_real_T_cartesian_tra_T *Limits;
  x_robotics_manip_internal_Rig_T *RigidBodyTreeInternal;
};

#endif                                 //struct_tag_etdDPy7RU0or6X7LBQh1x

#ifndef typedef_b_inverseKinematics_cartesian_T
#define typedef_b_inverseKinematics_cartesian_T

typedef struct tag_etdDPy7RU0or6X7LBQh1x b_inverseKinematics_cartesian_T;

#endif                                 //typedef_b_inverseKinematics_cartesian_T

#ifndef struct_tag_sxjfBB9iGq1brg4yDDoQjB
#define struct_tag_sxjfBB9iGq1brg4yDDoQjB

struct tag_sxjfBB9iGq1brg4yDDoQjB
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  y_robotics_manip_internal_Rig_T TreeInternal;
  b_inverseKinematics_cartesian_T IKInternal;
};

#endif                                 //struct_tag_sxjfBB9iGq1brg4yDDoQjB

#ifndef typedef_robotics_slmanip_internal_blo_T
#define typedef_robotics_slmanip_internal_blo_T

typedef struct tag_sxjfBB9iGq1brg4yDDoQjB robotics_slmanip_internal_blo_T;

#endif                                 //typedef_robotics_slmanip_internal_blo_T

#ifndef typedef_c_robotics_core_internal_NLPS_T
#define typedef_c_robotics_core_internal_NLPS_T

typedef int32_T c_robotics_core_internal_NLPS_T;

#endif                                 //typedef_c_robotics_core_internal_NLPS_T

#ifndef robotics_core_internal_NLPSolverExitFlags_constants
#define robotics_core_internal_NLPSolverExitFlags_constants

// enum robotics_core_internal_NLPSolverExitFlags
const c_robotics_core_internal_NLPS_T LocalMinimumFound = 1;
const c_robotics_core_internal_NLPS_T IterationLimitExceeded = 2;
const c_robotics_core_internal_NLPS_T TimeLimitExceeded = 3;
const c_robotics_core_internal_NLPS_T StepSizeBelowMinimum = 4;
const c_robotics_core_internal_NLPS_T ChangeInErrorBelowMinimum = 5;
const c_robotics_core_internal_NLPS_T SearchDirectionInvalid = 6;
const c_robotics_core_internal_NLPS_T HessianNotPositiveSemidefinite = 7;
const c_robotics_core_internal_NLPS_T TrustRegionRadiusBelowMinimum = 8;

#endif                     //robotics_core_internal_NLPSolverExitFlags_constants

#ifndef struct_tag_vxHWSOYrO9xtYchIOe7EKG
#define struct_tag_vxHWSOYrO9xtYchIOe7EKG

struct tag_vxHWSOYrO9xtYchIOe7EKG
{
  int32_T isInitialized;
};

#endif                                 //struct_tag_vxHWSOYrO9xtYchIOe7EKG

#ifndef typedef_robotics_slcore_internal_bloc_T
#define typedef_robotics_slcore_internal_bloc_T

typedef struct tag_vxHWSOYrO9xtYchIOe7EKG robotics_slcore_internal_bloc_T;

#endif                                 //typedef_robotics_slcore_internal_bloc_T

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

#ifndef struct_tag_rVaL2jCthDg1Nc4cghuMrG
#define struct_tag_rVaL2jCthDg1Nc4cghuMrG

struct tag_rVaL2jCthDg1Nc4cghuMrG
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
};

#endif                                 //struct_tag_rVaL2jCthDg1Nc4cghuMrG

#ifndef typedef_ros_slros_internal_block_SetP_T
#define typedef_ros_slros_internal_block_SetP_T

typedef struct tag_rVaL2jCthDg1Nc4cghuMrG ros_slros_internal_block_SetP_T;

#endif                                 //typedef_ros_slros_internal_block_SetP_T

// Parameters (default storage)
typedef struct P_cartesian_trajectory_planne_T_ P_cartesian_trajectory_planne_T;

// Forward declaration for rtModel
typedef struct tag_RTM_cartesian_trajectory__T RT_MODEL_cartesian_trajectory_T;

#endif                    // RTW_HEADER_cartesian_trajectory_planner_2_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
