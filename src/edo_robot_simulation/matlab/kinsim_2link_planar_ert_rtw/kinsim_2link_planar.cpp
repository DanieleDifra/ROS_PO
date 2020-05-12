//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: kinsim_2link_planar.cpp
//
// Code generated for Simulink model 'kinsim_2link_planar'.
//
// Model version                  : 1.122
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Tue May 12 23:09:10 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "kinsim_2link_planar.h"
#include "kinsim_2link_planar_private.h"
#include "kinsim_2link_planar_dt.h"

// Block signals (default storage)
B_kinsim_2link_planar_T kinsim_2link_planar_B;

// Continuous states
X_kinsim_2link_planar_T kinsim_2link_planar_X;

// Block states (default storage)
DW_kinsim_2link_planar_T kinsim_2link_planar_DW;

// Real-time model
RT_MODEL_kinsim_2link_planar_T kinsim_2link_planar_M_ =
  RT_MODEL_kinsim_2link_planar_T();
RT_MODEL_kinsim_2link_planar_T *const kinsim_2link_planar_M =
  &kinsim_2link_planar_M_;

// Forward declaration for local functions
static void kinsim_2link_pl_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Positions[128], uint32_T *varargout_2_Positions_SL_Info_C,
  uint32_T *varargout_2_Positions_SL_Info_R, real_T varargout_2_Velocities[128],
  uint32_T *varargout_2_Velocities_SL_Info_, uint32_T
  *varargout_2_Velocities_SL_Inf_0, real_T varargout_2_Accelerations[128],
  uint32_T *varargout_2_Accelerations_SL_In, uint32_T
  *varargout_2_Accelerations_SL__0, real_T varargout_2_Effort[128], uint32_T
  *varargout_2_Effort_SL_Info_Curr, uint32_T *varargout_2_Effort_SL_Info_Rece,
  real_T *varargout_2_TimeFromStart_Sec, real_T *varargout_2_TimeFromStart_Nsec);
static void kinsim_2link_planar_atan2(const real_T y_data[], const int32_T
  y_size[3], const real_T x_data[], const int32_T x_size[3], real_T r_data[],
  int32_T r_size[3]);
static void matlabCodegenHandle_matlabC_li5(ros_slros_internal_block_Subs_T *obj);
static void matlabCodegenHandle_matlab_li5a(ros_slros_internal_block_GetP_T *obj);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Publ_T *obj);

//
// This function updates continuous states using the ODE3 fixed-step
// solver algorithm
//
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  // Solver Matrices
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = static_cast<ODE3_IntgData *>(rtsiGetSolverData(si));
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 6;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  // Save the state values at time t in y, we'll use x as ynew.
  (void) memcpy(y, x,
                static_cast<uint_T>(nXc)*sizeof(real_T));

  // Assumes that rtsiSetT and ModelOutputs are up-to-date
  // f0 = f(t,y)
  rtsiSetdX(si, f0);
  kinsim_2link_planar_derivatives();

  // f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*));
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  kinsim_2link_planar_step();
  kinsim_2link_planar_derivatives();

  // f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*));
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  kinsim_2link_planar_step();
  kinsim_2link_planar_derivatives();

  // tnew = t + hA(3);
  // ynew = y + f*hB(:,3);
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

static void kinsim_2link_pl_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Positions[128], uint32_T *varargout_2_Positions_SL_Info_C,
  uint32_T *varargout_2_Positions_SL_Info_R, real_T varargout_2_Velocities[128],
  uint32_T *varargout_2_Velocities_SL_Info_, uint32_T
  *varargout_2_Velocities_SL_Inf_0, real_T varargout_2_Accelerations[128],
  uint32_T *varargout_2_Accelerations_SL_In, uint32_T
  *varargout_2_Accelerations_SL__0, real_T varargout_2_Effort[128], uint32_T
  *varargout_2_Effort_SL_Info_Curr, uint32_T *varargout_2_Effort_SL_Info_Rece,
  real_T *varargout_2_TimeFromStart_Sec, real_T *varargout_2_TimeFromStart_Nsec)
{
  *varargout_1 = Sub_kinsim_2link_planar_16.getLatestMessage
    (&kinsim_2link_planar_B.b_varargout_2);
  *varargout_2_Positions_SL_Info_C =
    kinsim_2link_planar_B.b_varargout_2.Positions_SL_Info.CurrentLength;
  *varargout_2_Positions_SL_Info_R =
    kinsim_2link_planar_B.b_varargout_2.Positions_SL_Info.ReceivedLength;
  *varargout_2_Velocities_SL_Info_ =
    kinsim_2link_planar_B.b_varargout_2.Velocities_SL_Info.CurrentLength;
  *varargout_2_Velocities_SL_Inf_0 =
    kinsim_2link_planar_B.b_varargout_2.Velocities_SL_Info.ReceivedLength;
  *varargout_2_Accelerations_SL_In =
    kinsim_2link_planar_B.b_varargout_2.Accelerations_SL_Info.CurrentLength;
  *varargout_2_Accelerations_SL__0 =
    kinsim_2link_planar_B.b_varargout_2.Accelerations_SL_Info.ReceivedLength;
  memcpy(&varargout_2_Positions[0],
         &kinsim_2link_planar_B.b_varargout_2.Positions[0], sizeof(real_T) << 7U);
  memcpy(&varargout_2_Velocities[0],
         &kinsim_2link_planar_B.b_varargout_2.Velocities[0], sizeof(real_T) <<
         7U);
  memcpy(&varargout_2_Accelerations[0],
         &kinsim_2link_planar_B.b_varargout_2.Accelerations[0], sizeof(real_T) <<
         7U);
  memcpy(&varargout_2_Effort[0], &kinsim_2link_planar_B.b_varargout_2.Effort[0],
         sizeof(real_T) << 7U);
  *varargout_2_Effort_SL_Info_Curr =
    kinsim_2link_planar_B.b_varargout_2.Effort_SL_Info.CurrentLength;
  *varargout_2_Effort_SL_Info_Rece =
    kinsim_2link_planar_B.b_varargout_2.Effort_SL_Info.ReceivedLength;
  *varargout_2_TimeFromStart_Sec =
    kinsim_2link_planar_B.b_varargout_2.TimeFromStart.Sec;
  *varargout_2_TimeFromStart_Nsec =
    kinsim_2link_planar_B.b_varargout_2.TimeFromStart.Nsec;
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T u0_0;
  int32_T u1_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = atan2(static_cast<real_T>(u0_0), static_cast<real_T>(u1_0));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

// Function for MATLAB Function: '<Root>/MATLAB Function'
static void kinsim_2link_planar_atan2(const real_T y_data[], const int32_T
  y_size[3], const real_T x_data[], const int32_T x_size[3], real_T r_data[],
  int32_T r_size[3])
{
  int32_T loop_ub;
  int32_T tmp;
  int8_T csz_idx_2;
  if (y_size[2] <= x_size[2]) {
    csz_idx_2 = static_cast<int8_T>(y_size[2]);
    tmp = static_cast<int8_T>(y_size[2]);
  } else {
    csz_idx_2 = 0;
    tmp = 0;
  }

  loop_ub = tmp - 1;
  if (0 <= loop_ub) {
    memcpy(&kinsim_2link_planar_B.z1_data, &r_data[0], (loop_ub + 1) * sizeof
           (real_T));
  }

  if (0 <= csz_idx_2 - 1) {
    kinsim_2link_planar_B.z1_data = rt_atan2d_snf(y_data[0], x_data[0]);
  }

  r_size[0] = 1;
  r_size[1] = 1;
  r_size[2] = csz_idx_2;
  loop_ub = tmp - 1;
  if (0 <= loop_ub) {
    memcpy(&r_data[0], &kinsim_2link_planar_B.z1_data, (loop_ub + 1) * sizeof
           (real_T));
  }
}

static void matlabCodegenHandle_matlabC_li5(ros_slros_internal_block_Subs_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void matlabCodegenHandle_matlab_li5a(ros_slros_internal_block_GetP_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Publ_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

// Model step function
void kinsim_2link_planar_step(void)
{
  uint32_T b_varargout_2_Positions_SL_Info;
  uint32_T b_varargout_2_Velocities_SL_Inf;
  uint32_T b_varargout_2_Velocities_SL_I_0;
  uint32_T b_varargout_2_Accelerations_SL_;
  uint32_T b_varargout_2_Accelerations_S_0;
  uint32_T b_varargout_2_Effort_SL_Info_Cu;
  uint32_T b_varargout_2_Effort_SL_Info_Re;
  boolean_T b_varargout_1;
  int32_T T_tmp;
  int32_T A_tmp;
  int32_T T_tmp_0;
  int32_T J_tmp;
  static const char_T h[7] = { 'j', 'o', 'i', 'n', 't', '_', '1' };

  static const char_T i[7] = { 'j', 'o', 'i', 'n', 't', '_', '2' };

  static const char_T j[7] = { 'j', 'o', 'i', 'n', 't', '_', '3' };

  static const char_T k[7] = { 'j', 'o', 'i', 'n', 't', '_', '4' };

  static const char_T l[7] = { 'j', 'o', 'i', 'n', 't', '_', '5' };

  static const char_T m[7] = { 'j', 'o', 'i', 'n', 't', '_', '6' };

  if (rtmIsMajorTimeStep(kinsim_2link_planar_M)) {
    // set solver stop time
    rtsiSetSolverStopTime(&kinsim_2link_planar_M->solverInfo,
                          ((kinsim_2link_planar_M->Timing.clockTick0+1)*
      kinsim_2link_planar_M->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if (rtmIsMinorTimeStep(kinsim_2link_planar_M)) {
    kinsim_2link_planar_M->Timing.t[0] = rtsiGetT
      (&kinsim_2link_planar_M->solverInfo);
  }

  // Reset subsysRan breadcrumbs
  srClearBC(kinsim_2link_planar_DW.EnabledSubsystem_SubsysRanBC);
  if (rtmIsMajorTimeStep(kinsim_2link_planar_M)) {
    // Outputs for Atomic SubSystem: '<Root>/Subscribe'
    // MATLABSystem: '<S10>/SourceBlock' incorporates:
    //   Inport: '<S12>/In1'

    kinsim_2link_pl_SystemCore_step(&b_varargout_1,
      kinsim_2link_planar_B.b_varargout_2_Positions,
      &kinsim_2link_planar_B.b_varargout_2_Positions_SL_Info,
      &b_varargout_2_Positions_SL_Info,
      kinsim_2link_planar_B.b_varargout_2_Velocities,
      &b_varargout_2_Velocities_SL_Inf, &b_varargout_2_Velocities_SL_I_0,
      kinsim_2link_planar_B.b_varargout_2_Accelerations,
      &b_varargout_2_Accelerations_SL_, &b_varargout_2_Accelerations_S_0,
      kinsim_2link_planar_B.b_varargout_2_Effort,
      &b_varargout_2_Effort_SL_Info_Cu, &b_varargout_2_Effort_SL_Info_Re,
      &kinsim_2link_planar_B.b_varargout_2_TimeFromStart_Sec,
      &kinsim_2link_planar_B.sy);

    // Outputs for Enabled SubSystem: '<S10>/Enabled Subsystem' incorporates:
    //   EnablePort: '<S12>/Enable'

    if (b_varargout_1) {
      kinsim_2link_planar_B.In1.Positions_SL_Info.CurrentLength =
        kinsim_2link_planar_B.b_varargout_2_Positions_SL_Info;
      kinsim_2link_planar_B.In1.Positions_SL_Info.ReceivedLength =
        b_varargout_2_Positions_SL_Info;
      kinsim_2link_planar_B.In1.Velocities_SL_Info.CurrentLength =
        b_varargout_2_Velocities_SL_Inf;
      kinsim_2link_planar_B.In1.Velocities_SL_Info.ReceivedLength =
        b_varargout_2_Velocities_SL_I_0;
      kinsim_2link_planar_B.In1.Accelerations_SL_Info.CurrentLength =
        b_varargout_2_Accelerations_SL_;
      kinsim_2link_planar_B.In1.Accelerations_SL_Info.ReceivedLength =
        b_varargout_2_Accelerations_S_0;
      memcpy(&kinsim_2link_planar_B.In1.Positions[0],
             &kinsim_2link_planar_B.b_varargout_2_Positions[0], sizeof(real_T) <<
             7U);
      memcpy(&kinsim_2link_planar_B.In1.Velocities[0],
             &kinsim_2link_planar_B.b_varargout_2_Velocities[0], sizeof(real_T) <<
             7U);
      memcpy(&kinsim_2link_planar_B.In1.Accelerations[0],
             &kinsim_2link_planar_B.b_varargout_2_Accelerations[0], sizeof
             (real_T) << 7U);
      memcpy(&kinsim_2link_planar_B.In1.Effort[0],
             &kinsim_2link_planar_B.b_varargout_2_Effort[0], sizeof(real_T) <<
             7U);
      kinsim_2link_planar_B.In1.Effort_SL_Info.CurrentLength =
        b_varargout_2_Effort_SL_Info_Cu;
      kinsim_2link_planar_B.In1.Effort_SL_Info.ReceivedLength =
        b_varargout_2_Effort_SL_Info_Re;
      kinsim_2link_planar_B.In1.TimeFromStart.Sec =
        kinsim_2link_planar_B.b_varargout_2_TimeFromStart_Sec;
      kinsim_2link_planar_B.In1.TimeFromStart.Nsec = kinsim_2link_planar_B.sy;
      srUpdateBC(kinsim_2link_planar_DW.EnabledSubsystem_SubsysRanBC);
    }

    // End of MATLABSystem: '<S10>/SourceBlock'
    // End of Outputs for SubSystem: '<S10>/Enabled Subsystem'
    // End of Outputs for SubSystem: '<Root>/Subscribe'
  }

  // MATLABSystem: '<S11>/Get Parameter'
  ParamGet_kinsim_2link_planar_61.get_parameter
    (&kinsim_2link_planar_B.b_varargout_2_TimeFromStart_Sec);

  // MATLABSystem: '<S11>/Get Parameter2'
  ParamGet_kinsim_2link_planar_85.get_parameter(&kinsim_2link_planar_B.sy);

  // MATLABSystem: '<S11>/Get Parameter5'
  ParamGet_kinsim_2link_planar_88.get_parameter
    (&kinsim_2link_planar_B.eulShaped_idx_1);

  // MATLABSystem: '<S11>/Get Parameter4'
  ParamGet_kinsim_2link_planar_87.get_parameter
    (&kinsim_2link_planar_B.eulShaped_idx_2);

  // MATLABSystem: '<S11>/Get Parameter3'
  ParamGet_kinsim_2link_planar_86.get_parameter(&kinsim_2link_planar_B.value);

  // MATLABSystem: '<S11>/Get Parameter1'
  ParamGet_kinsim_2link_planar_65.get_parameter(&kinsim_2link_planar_B.value_c);

  // Integrator: '<Root>/Integrator' incorporates:
  //   MATLABSystem: '<S11>/Get Parameter'
  //   MATLABSystem: '<S11>/Get Parameter1'
  //   MATLABSystem: '<S11>/Get Parameter2'
  //   MATLABSystem: '<S11>/Get Parameter3'
  //   MATLABSystem: '<S11>/Get Parameter4'
  //   MATLABSystem: '<S11>/Get Parameter5'

  if (kinsim_2link_planar_DW.Integrator_IWORK != 0) {
    kinsim_2link_planar_X.Integrator_CSTATE[0] =
      kinsim_2link_planar_B.b_varargout_2_TimeFromStart_Sec;
    kinsim_2link_planar_X.Integrator_CSTATE[1] = kinsim_2link_planar_B.sy;
    kinsim_2link_planar_X.Integrator_CSTATE[2] =
      kinsim_2link_planar_B.eulShaped_idx_1;
    kinsim_2link_planar_X.Integrator_CSTATE[3] =
      kinsim_2link_planar_B.eulShaped_idx_2;
    kinsim_2link_planar_X.Integrator_CSTATE[4] = kinsim_2link_planar_B.value;
    kinsim_2link_planar_X.Integrator_CSTATE[5] = kinsim_2link_planar_B.value_c;
  }

  // MATLAB Function: '<Root>/MATLAB Function' incorporates:
  //   Constant: '<Root>/Constant'
  //   Integrator: '<Root>/Integrator'

  memset(&kinsim_2link_planar_B.T[0], 0, 96U * sizeof(real_T));
  for (kinsim_2link_planar_B.i = 0; kinsim_2link_planar_B.i < 6;
       kinsim_2link_planar_B.i++) {
    kinsim_2link_planar_B.b_varargout_2_TimeFromStart_Sec = sin
      (kinsim_2link_planar_X.Integrator_CSTATE[kinsim_2link_planar_B.i]);
    kinsim_2link_planar_B.sy = cos
      (kinsim_2link_planar_X.Integrator_CSTATE[kinsim_2link_planar_B.i]);
    kinsim_2link_planar_B.eulShaped_idx_2 =
      kinsim_2link_planar_P.dh[kinsim_2link_planar_B.i + 12];
    kinsim_2link_planar_B.eulShaped_idx_1 = cos
      (kinsim_2link_planar_B.eulShaped_idx_2);
    kinsim_2link_planar_B.eulShaped_idx_2 = sin
      (kinsim_2link_planar_B.eulShaped_idx_2);
    A_tmp = kinsim_2link_planar_B.i << 4;
    kinsim_2link_planar_B.A[A_tmp] = kinsim_2link_planar_B.sy;
    kinsim_2link_planar_B.A[A_tmp + 4] =
      -kinsim_2link_planar_B.b_varargout_2_TimeFromStart_Sec *
      kinsim_2link_planar_B.eulShaped_idx_1;
    kinsim_2link_planar_B.A[A_tmp + 8] =
      kinsim_2link_planar_B.b_varargout_2_TimeFromStart_Sec *
      kinsim_2link_planar_B.eulShaped_idx_2;
    kinsim_2link_planar_B.value =
      kinsim_2link_planar_P.dh[kinsim_2link_planar_B.i + 6];
    kinsim_2link_planar_B.A[A_tmp + 12] = kinsim_2link_planar_B.value *
      kinsim_2link_planar_B.sy;
    kinsim_2link_planar_B.A[A_tmp + 1] =
      kinsim_2link_planar_B.b_varargout_2_TimeFromStart_Sec;
    kinsim_2link_planar_B.A[A_tmp + 5] = kinsim_2link_planar_B.sy *
      kinsim_2link_planar_B.eulShaped_idx_1;
    kinsim_2link_planar_B.A[A_tmp + 9] = -kinsim_2link_planar_B.sy *
      kinsim_2link_planar_B.eulShaped_idx_2;
    kinsim_2link_planar_B.A[A_tmp + 13] = kinsim_2link_planar_B.value *
      kinsim_2link_planar_B.b_varargout_2_TimeFromStart_Sec;
    kinsim_2link_planar_B.A[A_tmp + 2] = 0.0;
    kinsim_2link_planar_B.A[A_tmp + 6] = kinsim_2link_planar_B.eulShaped_idx_2;
    kinsim_2link_planar_B.A[A_tmp + 10] = kinsim_2link_planar_B.eulShaped_idx_1;
    kinsim_2link_planar_B.A[A_tmp + 14] =
      kinsim_2link_planar_P.dh[kinsim_2link_planar_B.i];
    kinsim_2link_planar_B.A[A_tmp + 3] = 0.0;
    kinsim_2link_planar_B.A[A_tmp + 7] = 0.0;
    kinsim_2link_planar_B.A[A_tmp + 11] = 0.0;
    kinsim_2link_planar_B.A[A_tmp + 15] = 1.0;
  }

  for (kinsim_2link_planar_B.i = 0; kinsim_2link_planar_B.i < 4;
       kinsim_2link_planar_B.i++) {
    A_tmp = kinsim_2link_planar_B.i << 2;
    kinsim_2link_planar_B.T[A_tmp] = kinsim_2link_planar_B.A[A_tmp];
    T_tmp = A_tmp + 1;
    kinsim_2link_planar_B.T[T_tmp] = kinsim_2link_planar_B.A[T_tmp];
    T_tmp = A_tmp + 2;
    kinsim_2link_planar_B.T[T_tmp] = kinsim_2link_planar_B.A[T_tmp];
    A_tmp += 3;
    kinsim_2link_planar_B.T[A_tmp] = kinsim_2link_planar_B.A[A_tmp];
  }

  for (kinsim_2link_planar_B.b_i = 0; kinsim_2link_planar_B.b_i < 5;
       kinsim_2link_planar_B.b_i++) {
    for (kinsim_2link_planar_B.i = 0; kinsim_2link_planar_B.i < 4;
         kinsim_2link_planar_B.i++) {
      for (J_tmp = 0; J_tmp < 4; J_tmp++) {
        A_tmp = J_tmp << 2;
        T_tmp = kinsim_2link_planar_B.i + A_tmp;
        kinsim_2link_planar_B.T_m[T_tmp] = 0.0;
        A_tmp += (kinsim_2link_planar_B.b_i + 1) << 4;
        T_tmp_0 = (kinsim_2link_planar_B.b_i << 4) + kinsim_2link_planar_B.i;
        kinsim_2link_planar_B.T_m[T_tmp] += kinsim_2link_planar_B.A[A_tmp] *
          kinsim_2link_planar_B.T[T_tmp_0];
        kinsim_2link_planar_B.T_m[T_tmp] += kinsim_2link_planar_B.A[A_tmp + 1] *
          kinsim_2link_planar_B.T[T_tmp_0 + 4];
        kinsim_2link_planar_B.T_m[T_tmp] += kinsim_2link_planar_B.A[A_tmp + 2] *
          kinsim_2link_planar_B.T[T_tmp_0 + 8];
        kinsim_2link_planar_B.T_m[T_tmp] += kinsim_2link_planar_B.A[A_tmp + 3] *
          kinsim_2link_planar_B.T[T_tmp_0 + 12];
      }
    }

    for (kinsim_2link_planar_B.i = 0; kinsim_2link_planar_B.i < 4;
         kinsim_2link_planar_B.i++) {
      A_tmp = kinsim_2link_planar_B.i << 2;
      T_tmp = A_tmp + ((kinsim_2link_planar_B.b_i + 1) << 4);
      kinsim_2link_planar_B.T[T_tmp] = kinsim_2link_planar_B.T_m[A_tmp];
      kinsim_2link_planar_B.T[T_tmp + 1] = kinsim_2link_planar_B.T_m[A_tmp + 1];
      kinsim_2link_planar_B.T[T_tmp + 2] = kinsim_2link_planar_B.T_m[A_tmp + 2];
      kinsim_2link_planar_B.T[T_tmp + 3] = kinsim_2link_planar_B.T_m[A_tmp + 3];
    }
  }

  kinsim_2link_planar_B.sy = sqrt(kinsim_2link_planar_B.T[80] *
    kinsim_2link_planar_B.T[80] + kinsim_2link_planar_B.T[81] *
    kinsim_2link_planar_B.T[81]);
  kinsim_2link_planar_B.b_varargout_2_TimeFromStart_Sec = rt_atan2d_snf
    (kinsim_2link_planar_B.T[86], kinsim_2link_planar_B.T[90]);
  kinsim_2link_planar_B.cartPos[4] = rt_atan2d_snf(-kinsim_2link_planar_B.T[82],
    kinsim_2link_planar_B.sy);
  kinsim_2link_planar_B.cartPos[3] = rt_atan2d_snf(kinsim_2link_planar_B.T[81],
    kinsim_2link_planar_B.T[80]);
  if (kinsim_2link_planar_B.sy < 2.2204460492503131E-15) {
    kinsim_2link_planar_B.T_size[0] = 1;
    kinsim_2link_planar_B.T_size[1] = 1;
    kinsim_2link_planar_B.T_size[2] = 1;
    kinsim_2link_planar_B.T_size_c[0] = 1;
    kinsim_2link_planar_B.T_size_c[1] = 1;
    kinsim_2link_planar_B.T_size_c[2] = 1;
    kinsim_2link_planar_B.T_size_k[0] = 1;
    kinsim_2link_planar_B.T_size_k[1] = 1;
    kinsim_2link_planar_B.T_size_k[2] = 1;
    kinsim_2link_planar_B.T_data = -kinsim_2link_planar_B.T[89];
    kinsim_2link_planar_B.T_data_b = kinsim_2link_planar_B.T[85];
    kinsim_2link_planar_B.T_data_p = -kinsim_2link_planar_B.T[82];
    kinsim_2link_planar_atan2(&kinsim_2link_planar_B.T_data,
      kinsim_2link_planar_B.T_size, &kinsim_2link_planar_B.T_data_b,
      kinsim_2link_planar_B.T_size_c, &kinsim_2link_planar_B.tmp_data,
      kinsim_2link_planar_B.tmp_size);
    kinsim_2link_planar_B.sy_size[0] = 1;
    kinsim_2link_planar_B.sy_size[1] = 1;
    kinsim_2link_planar_B.sy_size[2] = 1;
    kinsim_2link_planar_B.T_data = kinsim_2link_planar_B.sy;
    kinsim_2link_planar_atan2(&kinsim_2link_planar_B.T_data_p,
      kinsim_2link_planar_B.T_size_k, &kinsim_2link_planar_B.T_data,
      kinsim_2link_planar_B.sy_size, &kinsim_2link_planar_B.T_data_b,
      kinsim_2link_planar_B.T_size);
    A_tmp = kinsim_2link_planar_B.tmp_size[2];
    for (kinsim_2link_planar_B.i = 0; kinsim_2link_planar_B.i < A_tmp;
         kinsim_2link_planar_B.i++) {
      kinsim_2link_planar_B.b_varargout_2_TimeFromStart_Sec =
        kinsim_2link_planar_B.tmp_data;
    }

    A_tmp = kinsim_2link_planar_B.T_size[2];
    for (kinsim_2link_planar_B.i = 0; kinsim_2link_planar_B.i < A_tmp;
         kinsim_2link_planar_B.i++) {
      kinsim_2link_planar_B.cartPos[4] = kinsim_2link_planar_B.T_data_b;
    }

    kinsim_2link_planar_B.cartPos[3] = 0.0;
  }

  memset(&kinsim_2link_planar_B.J[0], 0, 36U * sizeof(real_T));
  kinsim_2link_planar_B.J[0] = 0.0 * kinsim_2link_planar_B.T[94] -
    kinsim_2link_planar_B.T[93];
  kinsim_2link_planar_B.J[1] = kinsim_2link_planar_B.T[92] - 0.0 *
    kinsim_2link_planar_B.T[94];
  kinsim_2link_planar_B.J[2] = 0.0 * kinsim_2link_planar_B.T[93] - 0.0 *
    kinsim_2link_planar_B.T[92];
  kinsim_2link_planar_B.J[3] = 0.0;
  kinsim_2link_planar_B.J[4] = 0.0;
  kinsim_2link_planar_B.J[5] = 1.0;
  for (kinsim_2link_planar_B.i = 0; kinsim_2link_planar_B.i < 5;
       kinsim_2link_planar_B.i++) {
    A_tmp = kinsim_2link_planar_B.i + 1;
    T_tmp_0 = kinsim_2link_planar_B.i << 4;
    kinsim_2link_planar_B.sy = kinsim_2link_planar_B.T[92] -
      kinsim_2link_planar_B.T[T_tmp_0 + 12];
    T_tmp = 6 * (kinsim_2link_planar_B.i + 1);
    kinsim_2link_planar_B.b_i = T_tmp_0 + 8;
    kinsim_2link_planar_B.J[T_tmp + 3] =
      kinsim_2link_planar_B.T[kinsim_2link_planar_B.b_i];
    kinsim_2link_planar_B.eulShaped_idx_1 = kinsim_2link_planar_B.T[93] -
      kinsim_2link_planar_B.T[T_tmp_0 + 13];
    J_tmp = T_tmp_0 + 9;
    kinsim_2link_planar_B.J[T_tmp + 4] = kinsim_2link_planar_B.T[J_tmp];
    kinsim_2link_planar_B.eulShaped_idx_2 = kinsim_2link_planar_B.T[94] -
      kinsim_2link_planar_B.T[T_tmp_0 + 14];
    T_tmp_0 += 10;
    kinsim_2link_planar_B.J[T_tmp + 5] = kinsim_2link_planar_B.T[T_tmp_0];
    kinsim_2link_planar_B.J[6 * A_tmp] = kinsim_2link_planar_B.T[J_tmp] *
      kinsim_2link_planar_B.eulShaped_idx_2 - kinsim_2link_planar_B.T[T_tmp_0] *
      kinsim_2link_planar_B.eulShaped_idx_1;
    kinsim_2link_planar_B.J[6 * A_tmp + 1] = kinsim_2link_planar_B.T[T_tmp_0] *
      kinsim_2link_planar_B.sy -
      kinsim_2link_planar_B.T[kinsim_2link_planar_B.b_i] *
      kinsim_2link_planar_B.eulShaped_idx_2;
    kinsim_2link_planar_B.J[6 * A_tmp + 2] =
      kinsim_2link_planar_B.T[kinsim_2link_planar_B.b_i] *
      kinsim_2link_planar_B.eulShaped_idx_1 - kinsim_2link_planar_B.T[J_tmp] *
      kinsim_2link_planar_B.sy;
  }

  for (kinsim_2link_planar_B.i = 0; kinsim_2link_planar_B.i < 6;
       kinsim_2link_planar_B.i++) {
    kinsim_2link_planar_B.cartVel[kinsim_2link_planar_B.i] = 0.0;
    for (J_tmp = 0; J_tmp < 6; J_tmp++) {
      kinsim_2link_planar_B.cartVel[kinsim_2link_planar_B.i] +=
        kinsim_2link_planar_B.J[6 * J_tmp + kinsim_2link_planar_B.i] *
        kinsim_2link_planar_B.In1.Velocities[J_tmp];
    }
  }

  kinsim_2link_planar_B.sy = sin(kinsim_2link_planar_B.cartPos[3]);
  kinsim_2link_planar_B.eulShaped_idx_1 = sin(kinsim_2link_planar_B.cartPos[4]);
  kinsim_2link_planar_B.eulShaped_idx_2 = cos(kinsim_2link_planar_B.cartPos[3]);
  kinsim_2link_planar_B.dv[0] = 0.0;
  kinsim_2link_planar_B.dv[3] = -kinsim_2link_planar_B.sy;
  kinsim_2link_planar_B.dv[6] = kinsim_2link_planar_B.eulShaped_idx_2 *
    kinsim_2link_planar_B.eulShaped_idx_1;
  kinsim_2link_planar_B.dv[1] = 0.0;
  kinsim_2link_planar_B.dv[4] = kinsim_2link_planar_B.eulShaped_idx_2;
  kinsim_2link_planar_B.dv[7] = kinsim_2link_planar_B.sy *
    kinsim_2link_planar_B.eulShaped_idx_1;
  kinsim_2link_planar_B.dv[2] = 1.0;
  kinsim_2link_planar_B.dv[5] = 0.0;
  kinsim_2link_planar_B.dv[8] = cos(kinsim_2link_planar_B.cartPos[4]);
  for (kinsim_2link_planar_B.i = 0; kinsim_2link_planar_B.i < 3;
       kinsim_2link_planar_B.i++) {
    kinsim_2link_planar_B.dv1[kinsim_2link_planar_B.i] =
      kinsim_2link_planar_B.dv[kinsim_2link_planar_B.i + 6] *
      kinsim_2link_planar_B.cartVel[5] +
      (kinsim_2link_planar_B.dv[kinsim_2link_planar_B.i + 3] *
       kinsim_2link_planar_B.cartVel[4] +
       kinsim_2link_planar_B.dv[kinsim_2link_planar_B.i] *
       kinsim_2link_planar_B.cartVel[3]);
  }

  // Clock: '<Root>/Clock1' incorporates:
  //   Clock: '<Root>/Clock'

  kinsim_2link_planar_B.sy = kinsim_2link_planar_M->Timing.t[0];

  // MATLAB Function: '<Root>/Assign to CartesianState msg' incorporates:
  //   Clock: '<Root>/Clock1'
  //   Constant: '<S4>/Constant'
  //   MATLAB Function: '<Root>/MATLAB Function'

  kinsim_2link_planar_B.msg_d = kinsim_2link_planar_P.Constant_Value;
  if (kinsim_2link_planar_B.sy < 0.0) {
    // MATLAB Function: '<Root>/Assign to JointState msg'
    kinsim_2link_planar_B.eulShaped_idx_1 = ceil(kinsim_2link_planar_B.sy);
  } else {
    // MATLAB Function: '<Root>/Assign to JointState msg'
    kinsim_2link_planar_B.eulShaped_idx_1 = floor(kinsim_2link_planar_B.sy);
  }

  kinsim_2link_planar_B.msg_d.Header.Stamp.Sec =
    kinsim_2link_planar_B.eulShaped_idx_1;
  kinsim_2link_planar_B.eulShaped_idx_2 = (kinsim_2link_planar_B.sy -
    kinsim_2link_planar_B.eulShaped_idx_1) * 1.0E+9;
  if (kinsim_2link_planar_B.eulShaped_idx_2 < 0.0) {
    // MATLAB Function: '<Root>/Assign to JointState msg'
    kinsim_2link_planar_B.eulShaped_idx_2 = ceil
      (kinsim_2link_planar_B.eulShaped_idx_2);
  } else {
    // MATLAB Function: '<Root>/Assign to JointState msg'
    kinsim_2link_planar_B.eulShaped_idx_2 = floor
      (kinsim_2link_planar_B.eulShaped_idx_2);
  }

  kinsim_2link_planar_B.msg_d.Header.Stamp.Nsec =
    kinsim_2link_planar_B.eulShaped_idx_2;
  kinsim_2link_planar_B.msg_d.Name_SL_Info.CurrentLength = 6U;
  kinsim_2link_planar_B.msg_d.Position_SL_Info.CurrentLength = 6U;
  kinsim_2link_planar_B.msg_d.Velocity_SL_Info.CurrentLength = 6U;
  kinsim_2link_planar_B.msg_d.Name[0].Data_SL_Info.CurrentLength = 7U;
  kinsim_2link_planar_B.msg_d.Position[0] = kinsim_2link_planar_B.T[92];
  kinsim_2link_planar_B.msg_d.Velocity[0] = kinsim_2link_planar_B.cartVel[0];
  kinsim_2link_planar_B.msg_d.Name[1].Data_SL_Info.CurrentLength = 7U;
  kinsim_2link_planar_B.msg_d.Position[1] = kinsim_2link_planar_B.T[93];
  kinsim_2link_planar_B.msg_d.Velocity[1] = kinsim_2link_planar_B.cartVel[1];
  kinsim_2link_planar_B.msg_d.Name[2].Data_SL_Info.CurrentLength = 7U;
  kinsim_2link_planar_B.msg_d.Position[2] = kinsim_2link_planar_B.T[94];
  kinsim_2link_planar_B.msg_d.Velocity[2] = kinsim_2link_planar_B.cartVel[2];
  kinsim_2link_planar_B.msg_d.Name[3].Data_SL_Info.CurrentLength = 7U;
  kinsim_2link_planar_B.msg_d.Position[3] = kinsim_2link_planar_B.cartPos[3];
  kinsim_2link_planar_B.msg_d.Velocity[3] = kinsim_2link_planar_B.dv1[0];
  kinsim_2link_planar_B.msg_d.Name[4].Data_SL_Info.CurrentLength = 7U;
  kinsim_2link_planar_B.msg_d.Position[4] = kinsim_2link_planar_B.cartPos[4];
  kinsim_2link_planar_B.msg_d.Velocity[4] = kinsim_2link_planar_B.dv1[1];
  for (kinsim_2link_planar_B.i = 0; kinsim_2link_planar_B.i < 7;
       kinsim_2link_planar_B.i++) {
    kinsim_2link_planar_B.b.f1[kinsim_2link_planar_B.i] =
      h[kinsim_2link_planar_B.i];
    kinsim_2link_planar_B.c.f1[kinsim_2link_planar_B.i] =
      i[kinsim_2link_planar_B.i];
    kinsim_2link_planar_B.d.f1[kinsim_2link_planar_B.i] =
      j[kinsim_2link_planar_B.i];
    kinsim_2link_planar_B.e.f1[kinsim_2link_planar_B.i] =
      k[kinsim_2link_planar_B.i];
    kinsim_2link_planar_B.f.f1[kinsim_2link_planar_B.i] =
      l[kinsim_2link_planar_B.i];
    kinsim_2link_planar_B.g.f1[kinsim_2link_planar_B.i] =
      m[kinsim_2link_planar_B.i];
    kinsim_2link_planar_B.msg_d.Name[0].Data[kinsim_2link_planar_B.i] =
      static_cast<uint8_T>(kinsim_2link_planar_B.b.f1[kinsim_2link_planar_B.i]);
    kinsim_2link_planar_B.msg_d.Name[1].Data[kinsim_2link_planar_B.i] =
      static_cast<uint8_T>(kinsim_2link_planar_B.c.f1[kinsim_2link_planar_B.i]);
    kinsim_2link_planar_B.msg_d.Name[2].Data[kinsim_2link_planar_B.i] =
      static_cast<uint8_T>(kinsim_2link_planar_B.d.f1[kinsim_2link_planar_B.i]);
    kinsim_2link_planar_B.msg_d.Name[3].Data[kinsim_2link_planar_B.i] =
      static_cast<uint8_T>(kinsim_2link_planar_B.e.f1[kinsim_2link_planar_B.i]);
    kinsim_2link_planar_B.msg_d.Name[4].Data[kinsim_2link_planar_B.i] =
      static_cast<uint8_T>(kinsim_2link_planar_B.f.f1[kinsim_2link_planar_B.i]);
    kinsim_2link_planar_B.msg_d.Name[5].Data[kinsim_2link_planar_B.i] =
      static_cast<uint8_T>(kinsim_2link_planar_B.g.f1[kinsim_2link_planar_B.i]);
  }

  kinsim_2link_planar_B.msg_d.Name[5].Data_SL_Info.CurrentLength = 7U;
  kinsim_2link_planar_B.msg_d.Position[5] =
    kinsim_2link_planar_B.b_varargout_2_TimeFromStart_Sec;
  kinsim_2link_planar_B.msg_d.Velocity[5] = kinsim_2link_planar_B.dv1[2];

  // End of MATLAB Function: '<Root>/Assign to CartesianState msg'

  // Outputs for Atomic SubSystem: '<Root>/Publish2'
  // MATLABSystem: '<S9>/SinkBlock'
  Pub_kinsim_2link_planar_79.publish(&kinsim_2link_planar_B.msg_d);

  // End of Outputs for SubSystem: '<Root>/Publish2'

  // MATLAB Function: '<Root>/Assign to JointState msg' incorporates:
  //   Constant: '<S4>/Constant'
  //   Integrator: '<Root>/Integrator'

  kinsim_2link_planar_B.msg_d = kinsim_2link_planar_P.Constant_Value;
  kinsim_2link_planar_B.msg_d.Header.Stamp.Sec =
    kinsim_2link_planar_B.eulShaped_idx_1;
  kinsim_2link_planar_B.msg_d.Header.Stamp.Nsec =
    kinsim_2link_planar_B.eulShaped_idx_2;
  kinsim_2link_planar_B.msg_d.Name_SL_Info.CurrentLength = 6U;
  kinsim_2link_planar_B.msg_d.Position_SL_Info.CurrentLength = 6U;
  kinsim_2link_planar_B.msg_d.Velocity_SL_Info.CurrentLength = 6U;
  kinsim_2link_planar_B.msg_d.Name[0].Data_SL_Info.CurrentLength = 7U;
  kinsim_2link_planar_B.msg_d.Position[0] =
    kinsim_2link_planar_X.Integrator_CSTATE[0];
  kinsim_2link_planar_B.msg_d.Velocity[0] =
    kinsim_2link_planar_B.In1.Velocities[0];
  kinsim_2link_planar_B.msg_d.Name[1].Data_SL_Info.CurrentLength = 7U;
  kinsim_2link_planar_B.msg_d.Position[1] =
    kinsim_2link_planar_X.Integrator_CSTATE[1];
  kinsim_2link_planar_B.msg_d.Velocity[1] =
    kinsim_2link_planar_B.In1.Velocities[1];
  kinsim_2link_planar_B.msg_d.Name[2].Data_SL_Info.CurrentLength = 7U;
  kinsim_2link_planar_B.msg_d.Position[2] =
    kinsim_2link_planar_X.Integrator_CSTATE[2];
  kinsim_2link_planar_B.msg_d.Velocity[2] =
    kinsim_2link_planar_B.In1.Velocities[2];
  kinsim_2link_planar_B.msg_d.Name[3].Data_SL_Info.CurrentLength = 7U;
  kinsim_2link_planar_B.msg_d.Position[3] =
    kinsim_2link_planar_X.Integrator_CSTATE[3];
  kinsim_2link_planar_B.msg_d.Velocity[3] =
    kinsim_2link_planar_B.In1.Velocities[3];
  kinsim_2link_planar_B.msg_d.Name[4].Data_SL_Info.CurrentLength = 7U;
  kinsim_2link_planar_B.msg_d.Position[4] =
    kinsim_2link_planar_X.Integrator_CSTATE[4];
  kinsim_2link_planar_B.msg_d.Velocity[4] =
    kinsim_2link_planar_B.In1.Velocities[4];
  for (kinsim_2link_planar_B.i = 0; kinsim_2link_planar_B.i < 7;
       kinsim_2link_planar_B.i++) {
    kinsim_2link_planar_B.b.f1[kinsim_2link_planar_B.i] =
      h[kinsim_2link_planar_B.i];
    kinsim_2link_planar_B.c.f1[kinsim_2link_planar_B.i] =
      i[kinsim_2link_planar_B.i];
    kinsim_2link_planar_B.d.f1[kinsim_2link_planar_B.i] =
      j[kinsim_2link_planar_B.i];
    kinsim_2link_planar_B.e.f1[kinsim_2link_planar_B.i] =
      k[kinsim_2link_planar_B.i];
    kinsim_2link_planar_B.f.f1[kinsim_2link_planar_B.i] =
      l[kinsim_2link_planar_B.i];
    kinsim_2link_planar_B.g.f1[kinsim_2link_planar_B.i] =
      m[kinsim_2link_planar_B.i];
    kinsim_2link_planar_B.msg_d.Name[0].Data[kinsim_2link_planar_B.i] =
      static_cast<uint8_T>(kinsim_2link_planar_B.b.f1[kinsim_2link_planar_B.i]);
    kinsim_2link_planar_B.msg_d.Name[1].Data[kinsim_2link_planar_B.i] =
      static_cast<uint8_T>(kinsim_2link_planar_B.c.f1[kinsim_2link_planar_B.i]);
    kinsim_2link_planar_B.msg_d.Name[2].Data[kinsim_2link_planar_B.i] =
      static_cast<uint8_T>(kinsim_2link_planar_B.d.f1[kinsim_2link_planar_B.i]);
    kinsim_2link_planar_B.msg_d.Name[3].Data[kinsim_2link_planar_B.i] =
      static_cast<uint8_T>(kinsim_2link_planar_B.e.f1[kinsim_2link_planar_B.i]);
    kinsim_2link_planar_B.msg_d.Name[4].Data[kinsim_2link_planar_B.i] =
      static_cast<uint8_T>(kinsim_2link_planar_B.f.f1[kinsim_2link_planar_B.i]);
    kinsim_2link_planar_B.msg_d.Name[5].Data[kinsim_2link_planar_B.i] =
      static_cast<uint8_T>(kinsim_2link_planar_B.g.f1[kinsim_2link_planar_B.i]);
  }

  kinsim_2link_planar_B.msg_d.Name[5].Data_SL_Info.CurrentLength = 7U;
  kinsim_2link_planar_B.msg_d.Position[5] =
    kinsim_2link_planar_X.Integrator_CSTATE[5];
  kinsim_2link_planar_B.msg_d.Velocity[5] =
    kinsim_2link_planar_B.In1.Velocities[5];

  // Outputs for Atomic SubSystem: '<Root>/Publish'
  // MATLABSystem: '<S7>/SinkBlock'
  Pub_kinsim_2link_planar_22.publish(&kinsim_2link_planar_B.msg_d);

  // End of Outputs for SubSystem: '<Root>/Publish'

  // MATLAB Function: '<Root>/Assign to Time msg'
  if (kinsim_2link_planar_B.sy < 0.0) {
    kinsim_2link_planar_B.eulShaped_idx_1 = ceil(kinsim_2link_planar_B.sy);
  } else {
    kinsim_2link_planar_B.eulShaped_idx_1 = floor(kinsim_2link_planar_B.sy);
  }

  kinsim_2link_planar_B.msg_l.Clock_.Sec = kinsim_2link_planar_B.eulShaped_idx_1;
  kinsim_2link_planar_B.eulShaped_idx_2 = (kinsim_2link_planar_B.sy -
    kinsim_2link_planar_B.eulShaped_idx_1) * 1.0E+9;
  if (kinsim_2link_planar_B.eulShaped_idx_2 < 0.0) {
    kinsim_2link_planar_B.msg_l.Clock_.Nsec = ceil
      (kinsim_2link_planar_B.eulShaped_idx_2);
  } else {
    kinsim_2link_planar_B.msg_l.Clock_.Nsec = floor
      (kinsim_2link_planar_B.eulShaped_idx_2);
  }

  // End of MATLAB Function: '<Root>/Assign to Time msg'

  // Outputs for Atomic SubSystem: '<Root>/Publish1'
  // MATLABSystem: '<S8>/SinkBlock'
  Pub_kinsim_2link_planar_50.publish(&kinsim_2link_planar_B.msg_l);

  // End of Outputs for SubSystem: '<Root>/Publish1'
  if (rtmIsMajorTimeStep(kinsim_2link_planar_M)) {
    // Update for Integrator: '<Root>/Integrator'
    kinsim_2link_planar_DW.Integrator_IWORK = 0;

    // External mode
    rtExtModeUploadCheckTrigger(2);

    {                                  // Sample time: [0.0s, 0.0s]
      rtExtModeUpload(0, (real_T)kinsim_2link_planar_M->Timing.t[0]);
    }

    if (rtmIsMajorTimeStep(kinsim_2link_planar_M)) {// Sample time: [0.05s, 0.0s] 
      rtExtModeUpload(1, (real_T)((kinsim_2link_planar_M->Timing.clockTick1) *
        0.05));
    }
  }                                    // end MajorTimeStep

  if (rtmIsMajorTimeStep(kinsim_2link_planar_M)) {
    // signal main to stop simulation
    {                                  // Sample time: [0.0s, 0.0s]
      if ((rtmGetTFinal(kinsim_2link_planar_M)!=-1) &&
          !((rtmGetTFinal(kinsim_2link_planar_M)-
             ((kinsim_2link_planar_M->Timing.clockTick1) * 0.05)) >
            ((kinsim_2link_planar_M->Timing.clockTick1) * 0.05) * (DBL_EPSILON)))
      {
        rtmSetErrorStatus(kinsim_2link_planar_M, "Simulation finished");
      }

      if (rtmGetStopRequested(kinsim_2link_planar_M)) {
        rtmSetErrorStatus(kinsim_2link_planar_M, "Simulation finished");
      }
    }

    rt_ertODEUpdateContinuousStates(&kinsim_2link_planar_M->solverInfo);

    // Update absolute time for base rate
    // The "clockTick0" counts the number of times the code of this task has
    //  been executed. The absolute time is the multiplication of "clockTick0"
    //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
    //  overflow during the application lifespan selected.

    ++kinsim_2link_planar_M->Timing.clockTick0;
    kinsim_2link_planar_M->Timing.t[0] = rtsiGetSolverStopTime
      (&kinsim_2link_planar_M->solverInfo);

    {
      // Update absolute timer for sample time: [0.05s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.05, which is the step size
      //  of the task. Size of "clockTick1" ensures timer will not overflow during the
      //  application lifespan selected.

      kinsim_2link_planar_M->Timing.clockTick1++;
    }
  }                                    // end MajorTimeStep
}

// Derivatives for root system: '<Root>'
void kinsim_2link_planar_derivatives(void)
{
  int32_T i;
  XDot_kinsim_2link_planar_T *_rtXdot;
  _rtXdot = ((XDot_kinsim_2link_planar_T *) kinsim_2link_planar_M->derivs);

  // Derivatives for Integrator: '<Root>/Integrator'
  for (i = 0; i < 6; i++) {
    _rtXdot->Integrator_CSTATE[i] = kinsim_2link_planar_B.In1.Velocities[i];
  }

  // End of Derivatives for Integrator: '<Root>/Integrator'
}

// Model initialize function
void kinsim_2link_planar_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&kinsim_2link_planar_M->solverInfo,
                          &kinsim_2link_planar_M->Timing.simTimeStep);
    rtsiSetTPtr(&kinsim_2link_planar_M->solverInfo, &rtmGetTPtr
                (kinsim_2link_planar_M));
    rtsiSetStepSizePtr(&kinsim_2link_planar_M->solverInfo,
                       &kinsim_2link_planar_M->Timing.stepSize0);
    rtsiSetdXPtr(&kinsim_2link_planar_M->solverInfo,
                 &kinsim_2link_planar_M->derivs);
    rtsiSetContStatesPtr(&kinsim_2link_planar_M->solverInfo, (real_T **)
                         &kinsim_2link_planar_M->contStates);
    rtsiSetNumContStatesPtr(&kinsim_2link_planar_M->solverInfo,
      &kinsim_2link_planar_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&kinsim_2link_planar_M->solverInfo,
      &kinsim_2link_planar_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&kinsim_2link_planar_M->solverInfo,
      &kinsim_2link_planar_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&kinsim_2link_planar_M->solverInfo,
      &kinsim_2link_planar_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&kinsim_2link_planar_M->solverInfo,
                          (&rtmGetErrorStatus(kinsim_2link_planar_M)));
    rtsiSetRTModelPtr(&kinsim_2link_planar_M->solverInfo, kinsim_2link_planar_M);
  }

  rtsiSetSimTimeStep(&kinsim_2link_planar_M->solverInfo, MAJOR_TIME_STEP);
  kinsim_2link_planar_M->intgData.y = kinsim_2link_planar_M->odeY;
  kinsim_2link_planar_M->intgData.f[0] = kinsim_2link_planar_M->odeF[0];
  kinsim_2link_planar_M->intgData.f[1] = kinsim_2link_planar_M->odeF[1];
  kinsim_2link_planar_M->intgData.f[2] = kinsim_2link_planar_M->odeF[2];
  kinsim_2link_planar_M->contStates = ((X_kinsim_2link_planar_T *)
    &kinsim_2link_planar_X);
  rtsiSetSolverData(&kinsim_2link_planar_M->solverInfo, static_cast<void *>
                    (&kinsim_2link_planar_M->intgData));
  rtsiSetSolverName(&kinsim_2link_planar_M->solverInfo,"ode3");
  rtmSetTPtr(kinsim_2link_planar_M, &kinsim_2link_planar_M->Timing.tArray[0]);
  rtmSetTFinal(kinsim_2link_planar_M, -1);
  kinsim_2link_planar_M->Timing.stepSize0 = 0.05;
  rtmSetFirstInitCond(kinsim_2link_planar_M, 1);

  // External mode info
  kinsim_2link_planar_M->Sizes.checksums[0] = (3253106006U);
  kinsim_2link_planar_M->Sizes.checksums[1] = (1111772405U);
  kinsim_2link_planar_M->Sizes.checksums[2] = (1627745148U);
  kinsim_2link_planar_M->Sizes.checksums[3] = (2534738041U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[20];
    kinsim_2link_planar_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = &rtAlwaysEnabled;
    systemRan[2] = &rtAlwaysEnabled;
    systemRan[3] = &rtAlwaysEnabled;
    systemRan[4] = &rtAlwaysEnabled;
    systemRan[5] = &rtAlwaysEnabled;
    systemRan[6] = &rtAlwaysEnabled;
    systemRan[7] = &rtAlwaysEnabled;
    systemRan[8] = &rtAlwaysEnabled;
    systemRan[9] = &rtAlwaysEnabled;
    systemRan[10] = &rtAlwaysEnabled;
    systemRan[11] = &rtAlwaysEnabled;
    systemRan[12] = (sysRanDType *)
      &kinsim_2link_planar_DW.EnabledSubsystem_SubsysRanBC;
    systemRan[13] = &rtAlwaysEnabled;
    systemRan[14] = &rtAlwaysEnabled;
    systemRan[15] = &rtAlwaysEnabled;
    systemRan[16] = &rtAlwaysEnabled;
    systemRan[17] = &rtAlwaysEnabled;
    systemRan[18] = &rtAlwaysEnabled;
    systemRan[19] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(kinsim_2link_planar_M->extModeInfo,
      &kinsim_2link_planar_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(kinsim_2link_planar_M->extModeInfo,
                        kinsim_2link_planar_M->Sizes.checksums);
    rteiSetTPtr(kinsim_2link_planar_M->extModeInfo, rtmGetTPtr
                (kinsim_2link_planar_M));
  }

  // data type transition information
  {
    static DataTypeTransInfo dtInfo;
    kinsim_2link_planar_M->SpecialInfo.mappingInfo = (&dtInfo);
    dtInfo.numDataTypes = 25;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    // Block I/O transition table
    dtInfo.BTransTable = &rtBTransTable;

    // Parameters transition table
    dtInfo.PTransTable = &rtPTransTable;
  }

  {
    char_T tmp[18];
    char_T tmp_0[14];
    char_T tmp_1[7];
    int32_T i;
    static const char_T tmp_2[17] = { '/', 'j', 'o', 'i', 'n', 't', '_', 't',
      'r', 'a', 'j', 'e', 'c', 't', 'o', 'r', 'y' };

    static const char_T tmp_3[17] = { '/', 'c', 'a', 'r', 't', 'e', 's', 'i',
      'a', 'n', '_', 's', 't', 'a', 't', 'e', 's' };

    static const char_T tmp_4[13] = { '/', 'j', 'o', 'i', 'n', 't', '_', 's',
      't', 'a', 't', 'e', 's' };

    static const char_T tmp_5[6] = { '/', 'c', 'l', 'o', 'c', 'k' };

    static const char_T tmp_6[31] = { '/', 'k', 'i', 'n', 's', 'i', 'm', '_',
      '2', 'l', 'i', 'n', 'k', '_', 'p', 'l', 'a', 'n', 'a', 'r', '/', 'q', '1',
      '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    static const char_T tmp_7[31] = { '/', 'k', 'i', 'n', 's', 'i', 'm', '_',
      '2', 'l', 'i', 'n', 'k', '_', 'p', 'l', 'a', 'n', 'a', 'r', '/', 'q', '2',
      '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    static const char_T tmp_8[31] = { '/', 'k', 'i', 'n', 's', 'i', 'm', '_',
      '2', 'l', 'i', 'n', 'k', '_', 'p', 'l', 'a', 'n', 'a', 'r', '/', 'q', '3',
      '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    static const char_T tmp_9[31] = { '/', 'k', 'i', 'n', 's', 'i', 'm', '_',
      '2', 'l', 'i', 'n', 'k', '_', 'p', 'l', 'a', 'n', 'a', 'r', '/', 'q', '4',
      '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    static const char_T tmp_a[31] = { '/', 'k', 'i', 'n', 's', 'i', 'm', '_',
      '2', 'l', 'i', 'n', 'k', '_', 'p', 'l', 'a', 'n', 'a', 'r', '/', 'q', '5',
      '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    static const char_T tmp_b[31] = { '/', 'k', 'i', 'n', 's', 'i', 'm', '_',
      '2', 'l', 'i', 'n', 'k', '_', 'p', 'l', 'a', 'n', 'a', 'r', '/', 'q', '6',
      '_', 'i', 'n', 'i', 't', 'i', 'a', 'l' };

    // InitializeConditions for Integrator: '<Root>/Integrator'
    if (rtmIsFirstInitCond(kinsim_2link_planar_M)) {
      kinsim_2link_planar_X.Integrator_CSTATE[0] = 0.0;
      kinsim_2link_planar_X.Integrator_CSTATE[1] = 0.0;
      kinsim_2link_planar_X.Integrator_CSTATE[2] = 0.0;
      kinsim_2link_planar_X.Integrator_CSTATE[3] = 0.0;
      kinsim_2link_planar_X.Integrator_CSTATE[4] = 0.0;
      kinsim_2link_planar_X.Integrator_CSTATE[5] = 0.0;
    }

    kinsim_2link_planar_DW.Integrator_IWORK = 1;

    // End of InitializeConditions for Integrator: '<Root>/Integrator'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S10>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S12>/Out1'
    kinsim_2link_planar_B.In1 = kinsim_2link_planar_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S10>/Enabled Subsystem'

    // Start for MATLABSystem: '<S10>/SourceBlock'
    kinsim_2link_planar_DW.obj_pp.matlabCodegenIsDeleted = false;
    kinsim_2link_planar_DW.obj_pp.isInitialized = 1;
    for (i = 0; i < 17; i++) {
      tmp[i] = tmp_2[i];
    }

    tmp[17] = '\x00';
    Sub_kinsim_2link_planar_16.createSubscriber(tmp, 1);
    kinsim_2link_planar_DW.obj_pp.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish2'
    // Start for MATLABSystem: '<S9>/SinkBlock'
    kinsim_2link_planar_DW.obj_m.matlabCodegenIsDeleted = false;
    kinsim_2link_planar_DW.obj_m.isInitialized = 1;
    for (i = 0; i < 17; i++) {
      tmp[i] = tmp_3[i];
    }

    tmp[17] = '\x00';
    Pub_kinsim_2link_planar_79.createPublisher(tmp, 1);
    kinsim_2link_planar_DW.obj_m.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S9>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish2'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish'
    // Start for MATLABSystem: '<S7>/SinkBlock'
    kinsim_2link_planar_DW.obj_nr.matlabCodegenIsDeleted = false;
    kinsim_2link_planar_DW.obj_nr.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      tmp_0[i] = tmp_4[i];
    }

    tmp_0[13] = '\x00';
    Pub_kinsim_2link_planar_22.createPublisher(tmp_0, 1);
    kinsim_2link_planar_DW.obj_nr.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S7>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish1'
    // Start for MATLABSystem: '<S8>/SinkBlock'
    kinsim_2link_planar_DW.obj_f.matlabCodegenIsDeleted = false;
    kinsim_2link_planar_DW.obj_f.isInitialized = 1;
    for (i = 0; i < 6; i++) {
      tmp_1[i] = tmp_5[i];
    }

    tmp_1[6] = '\x00';
    Pub_kinsim_2link_planar_50.createPublisher(tmp_1, 1);
    kinsim_2link_planar_DW.obj_f.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S8>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish1'

    // Start for MATLABSystem: '<S11>/Get Parameter'
    kinsim_2link_planar_DW.obj.matlabCodegenIsDeleted = false;
    kinsim_2link_planar_DW.obj.isInitialized = 1;
    for (i = 0; i < 31; i++) {
      kinsim_2link_planar_B.cv[i] = tmp_6[i];
    }

    kinsim_2link_planar_B.cv[31] = '\x00';
    ParamGet_kinsim_2link_planar_61.initialize(kinsim_2link_planar_B.cv);
    ParamGet_kinsim_2link_planar_61.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinsim_2link_planar_61.set_initial_value(0.78539816339744828);
    kinsim_2link_planar_DW.obj.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter'

    // Start for MATLABSystem: '<S11>/Get Parameter2'
    kinsim_2link_planar_DW.obj_p.matlabCodegenIsDeleted = false;
    kinsim_2link_planar_DW.obj_p.isInitialized = 1;
    for (i = 0; i < 31; i++) {
      kinsim_2link_planar_B.cv[i] = tmp_7[i];
    }

    kinsim_2link_planar_B.cv[31] = '\x00';
    ParamGet_kinsim_2link_planar_85.initialize(kinsim_2link_planar_B.cv);
    ParamGet_kinsim_2link_planar_85.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinsim_2link_planar_85.set_initial_value(-0.78539816339744828);
    kinsim_2link_planar_DW.obj_p.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter2'

    // Start for MATLABSystem: '<S11>/Get Parameter5'
    kinsim_2link_planar_DW.obj_n.matlabCodegenIsDeleted = false;
    kinsim_2link_planar_DW.obj_n.isInitialized = 1;
    for (i = 0; i < 31; i++) {
      kinsim_2link_planar_B.cv[i] = tmp_8[i];
    }

    kinsim_2link_planar_B.cv[31] = '\x00';
    ParamGet_kinsim_2link_planar_88.initialize(kinsim_2link_planar_B.cv);
    ParamGet_kinsim_2link_planar_88.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinsim_2link_planar_88.set_initial_value(-0.78539816339744828);
    kinsim_2link_planar_DW.obj_n.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter5'

    // Start for MATLABSystem: '<S11>/Get Parameter4'
    kinsim_2link_planar_DW.obj_e.matlabCodegenIsDeleted = false;
    kinsim_2link_planar_DW.obj_e.isInitialized = 1;
    for (i = 0; i < 31; i++) {
      kinsim_2link_planar_B.cv[i] = tmp_9[i];
    }

    kinsim_2link_planar_B.cv[31] = '\x00';
    ParamGet_kinsim_2link_planar_87.initialize(kinsim_2link_planar_B.cv);
    ParamGet_kinsim_2link_planar_87.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinsim_2link_planar_87.set_initial_value(-0.78539816339744828);
    kinsim_2link_planar_DW.obj_e.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter4'

    // Start for MATLABSystem: '<S11>/Get Parameter3'
    kinsim_2link_planar_DW.obj_l.matlabCodegenIsDeleted = false;
    kinsim_2link_planar_DW.obj_l.isInitialized = 1;
    for (i = 0; i < 31; i++) {
      kinsim_2link_planar_B.cv[i] = tmp_a[i];
    }

    kinsim_2link_planar_B.cv[31] = '\x00';
    ParamGet_kinsim_2link_planar_86.initialize(kinsim_2link_planar_B.cv);
    ParamGet_kinsim_2link_planar_86.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinsim_2link_planar_86.set_initial_value(-0.78539816339744828);
    kinsim_2link_planar_DW.obj_l.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter3'

    // Start for MATLABSystem: '<S11>/Get Parameter1'
    kinsim_2link_planar_DW.obj_ng.matlabCodegenIsDeleted = false;
    kinsim_2link_planar_DW.obj_ng.isInitialized = 1;
    for (i = 0; i < 31; i++) {
      kinsim_2link_planar_B.cv[i] = tmp_b[i];
    }

    kinsim_2link_planar_B.cv[31] = '\x00';
    ParamGet_kinsim_2link_planar_65.initialize(kinsim_2link_planar_B.cv);
    ParamGet_kinsim_2link_planar_65.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinsim_2link_planar_65.set_initial_value(-0.78539816339744828);
    kinsim_2link_planar_DW.obj_ng.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter1'
  }

  // set "at time zero" to false
  if (rtmIsFirstInitCond(kinsim_2link_planar_M)) {
    rtmSetFirstInitCond(kinsim_2link_planar_M, 0);
  }
}

// Model terminate function
void kinsim_2link_planar_terminate(void)
{
  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S10>/SourceBlock'
  matlabCodegenHandle_matlabC_li5(&kinsim_2link_planar_DW.obj_pp);

  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for MATLABSystem: '<S11>/Get Parameter'
  matlabCodegenHandle_matlab_li5a(&kinsim_2link_planar_DW.obj);

  // Terminate for MATLABSystem: '<S11>/Get Parameter2'
  matlabCodegenHandle_matlab_li5a(&kinsim_2link_planar_DW.obj_p);

  // Terminate for MATLABSystem: '<S11>/Get Parameter5'
  matlabCodegenHandle_matlab_li5a(&kinsim_2link_planar_DW.obj_n);

  // Terminate for MATLABSystem: '<S11>/Get Parameter4'
  matlabCodegenHandle_matlab_li5a(&kinsim_2link_planar_DW.obj_e);

  // Terminate for MATLABSystem: '<S11>/Get Parameter3'
  matlabCodegenHandle_matlab_li5a(&kinsim_2link_planar_DW.obj_l);

  // Terminate for MATLABSystem: '<S11>/Get Parameter1'
  matlabCodegenHandle_matlab_li5a(&kinsim_2link_planar_DW.obj_ng);

  // Terminate for Atomic SubSystem: '<Root>/Publish2'
  // Terminate for MATLABSystem: '<S9>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&kinsim_2link_planar_DW.obj_m);

  // End of Terminate for SubSystem: '<Root>/Publish2'

  // Terminate for Atomic SubSystem: '<Root>/Publish'
  // Terminate for MATLABSystem: '<S7>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&kinsim_2link_planar_DW.obj_nr);

  // End of Terminate for SubSystem: '<Root>/Publish'

  // Terminate for Atomic SubSystem: '<Root>/Publish1'
  // Terminate for MATLABSystem: '<S8>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&kinsim_2link_planar_DW.obj_f);

  // End of Terminate for SubSystem: '<Root>/Publish1'
}

//
// File trailer for generated code.
//
// [EOF]
//
