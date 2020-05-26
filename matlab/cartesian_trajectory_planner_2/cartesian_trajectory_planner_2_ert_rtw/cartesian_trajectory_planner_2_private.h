//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: cartesian_trajectory_planner_2_private.h
//
// Code generated for Simulink model 'cartesian_trajectory_planner_2'.
//
// Model version                  : 1.139
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Mon May 25 16:52:18 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_cartesian_trajectory_planner_2_private_h_
#define RTW_HEADER_cartesian_trajectory_planner_2_private_h_
#include "rtwtypes.h"
#include "cartesian_trajectory_planner_2.h"

extern real_T rt_hypotd_snf(real_T u0, real_T u1);
extern real_T rt_atan2d_snf(real_T u0, real_T u1);
extern real_T rt_roundd_snf(real_T u);
extern int32_T div_s32_floor(int32_T numerator, int32_T denominator);
extern void cartesian_tra_MATLABSystem_Init(B_MATLABSystem_cartesian_traj_T
  *localB, DW_MATLABSystem_cartesian_tra_T *localDW);
extern void cartesian_trajecto_MATLABSystem(const real_T rtu_0[16], const real_T
  rtu_1[6], const real_T rtu_2[6], B_MATLABSystem_cartesian_traj_T *localB,
  DW_MATLABSystem_cartesian_tra_T *localDW);
extern void CoordinateTransformationCo_Init(DW_CoordinateTransformationCo_T
  *localDW);
extern void CoordinateTransformationConvers(const real_T rtu_0[4], const real_T
  rtu_1[3], B_CoordinateTransformationCon_T *localB);
extern void cartesian_tra_MATLABSystem_Term(DW_MATLABSystem_cartesian_tra_T
  *localDW);

#endif                  // RTW_HEADER_cartesian_trajectory_planner_2_private_h_

//
// File trailer for generated code.
//
// [EOF]
//
