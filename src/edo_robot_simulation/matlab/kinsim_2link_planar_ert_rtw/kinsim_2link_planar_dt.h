//
//  kinsim_2link_planar_dt.h
//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  Code generation for model "kinsim_2link_planar".
//
//  Model version              : 1.122
//  Simulink Coder version : 9.3 (R2020a) 18-Nov-2019
//  C++ source code generated on : Tue May 12 23:09:10 2020
//
//  Target selection: ert.tlc
//  Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
//  Code generation objectives: Unspecified
//  Validation result: Not run


#include "ext_types.h"

// data type size table
static uint_T rtDataTypeSizes[] = {
  sizeof(real_T),
  sizeof(real32_T),
  sizeof(int8_T),
  sizeof(uint8_T),
  sizeof(int16_T),
  sizeof(uint16_T),
  sizeof(int32_T),
  sizeof(uint32_T),
  sizeof(boolean_T),
  sizeof(fcn_call_T),
  sizeof(int_T),
  sizeof(pointer_T),
  sizeof(action_T),
  2*sizeof(uint32_T),
  sizeof(SL_Bus_ROSVariableLengthArrayInfo),
  sizeof(SL_Bus_kinsim_2link_planar_std_msgs_String),
  sizeof(SL_Bus_kinsim_2link_planar_ros_time_Time),
  sizeof(SL_Bus_kinsim_2link_planar_std_msgs_Header),
  sizeof(SL_Bus_kinsim_2link_planar_sensor_msgs_JointState),
  sizeof(SL_Bus_kinsim_2link_planar_rosgraph_msgs_Clock),
  sizeof(SL_Bus_kinsim_2link_planar_ros_time_Duration),
  sizeof(SL_Bus_kinsim_2link_planar_JointTrajectoryPoint_1csgl9),
  sizeof(ros_slros_internal_block_Publ_T),
  sizeof(ros_slros_internal_block_Subs_T),
  sizeof(ros_slros_internal_block_GetP_T)
};

// data type name table
static const char_T * rtDataTypeNames[] = {
  "real_T",
  "real32_T",
  "int8_T",
  "uint8_T",
  "int16_T",
  "uint16_T",
  "int32_T",
  "uint32_T",
  "boolean_T",
  "fcn_call_T",
  "int_T",
  "pointer_T",
  "action_T",
  "timer_uint32_pair_T",
  "SL_Bus_ROSVariableLengthArrayInfo",
  "SL_Bus_kinsim_2link_planar_std_msgs_String",
  "SL_Bus_kinsim_2link_planar_ros_time_Time",
  "SL_Bus_kinsim_2link_planar_std_msgs_Header",
  "SL_Bus_kinsim_2link_planar_sensor_msgs_JointState",
  "SL_Bus_kinsim_2link_planar_rosgraph_msgs_Clock",
  "SL_Bus_kinsim_2link_planar_ros_time_Duration",
  "SL_Bus_kinsim_2link_planar_JointTrajectoryPoint_1csgl9",
  "ros_slros_internal_block_Publ_T",
  "ros_slros_internal_block_Subs_T",
  "ros_slros_internal_block_GetP_T"
};

// data type transitions for block I/O structure
static DataTypeTransition rtBTransitions[] = {
  { (char_T *)(&kinsim_2link_planar_B.In1), 21, 0, 1 }
  ,

  { (char_T *)(&kinsim_2link_planar_DW.obj), 24, 0, 6 },

  { (char_T *)(&kinsim_2link_planar_DW.obj_m), 22, 0, 3 },

  { (char_T *)(&kinsim_2link_planar_DW.obj_pp), 23, 0, 1 },

  { (char_T *)(&kinsim_2link_planar_DW.Integrator_IWORK), 10, 0, 1 },

  { (char_T *)(&kinsim_2link_planar_DW.EnabledSubsystem_SubsysRanBC), 2, 0, 1 }
};

// data type transition table for block I/O structure
static DataTypeTransitionTable rtBTransTable = {
  6U,
  rtBTransitions
};

// data type transitions for Parameters structure
static DataTypeTransition rtPTransitions[] = {
  { (char_T *)(&kinsim_2link_planar_P.dh[0]), 0, 0, 18 },

  { (char_T *)(&kinsim_2link_planar_P.Constant_Value), 18, 0, 1 },

  { (char_T *)(&kinsim_2link_planar_P.Out1_Y0), 21, 0, 1 },

  { (char_T *)(&kinsim_2link_planar_P.Constant_Value_h), 21, 0, 1 },

  { (char_T *)(&kinsim_2link_planar_P.Constant_Value_o), 19, 0, 1 }
};

// data type transition table for Parameters structure
static DataTypeTransitionTable rtPTransTable = {
  5U,
  rtPTransitions
};

// [EOF] kinsim_2link_planar_dt.h
