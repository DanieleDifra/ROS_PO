//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: cartesian_trajectory_planner_private.h
//
// Code generated for Simulink model 'cartesian_trajectory_planner'.
//
// Model version                  : 1.130
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Tue May 19 17:06:24 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_cartesian_trajectory_planner_private_h_
#define RTW_HEADER_cartesian_trajectory_planner_private_h_
#include "rtwtypes.h"

extern real_T rt_hypotd_snf(real_T u0, real_T u1);
extern real_T rt_atan2d_snf(real_T u0, real_T u1);
extern void rt_invd6x6_snf(const real_T u[36], real_T y[36]);
extern int32_T div_s32_floor(int32_T numerator, int32_T denominator);

#endif                    // RTW_HEADER_cartesian_trajectory_planner_private_h_

//
// File trailer for generated code.
//
// [EOF]
//
