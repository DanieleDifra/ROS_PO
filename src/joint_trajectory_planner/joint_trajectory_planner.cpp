//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: joint_trajectory_planner.cpp
//
// Code generated for Simulink model 'joint_trajectory_planner'.
//
// Model version                  : 1.11
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Thu May 28 15:59:49 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "joint_trajectory_planner.h"
#include "joint_trajectory_planner_private.h"

// Block signals (default storage)
B_joint_trajectory_planner_T joint_trajectory_planner_B;

// Block states (default storage)
DW_joint_trajectory_planner_T joint_trajectory_planner_DW;

// Real-time model
RT_MODEL_joint_trajectory_pla_T joint_trajectory_planner_M_ =
  RT_MODEL_joint_trajectory_pla_T();
RT_MODEL_joint_trajectory_pla_T *const joint_trajectory_planner_M =
  &joint_trajectory_planner_M_;

// Forward declaration for local functions
static real_T joint_trajectory_pl_rt_powd_snf(real_T u0, real_T u1);
static void joint_traje_generateCubicCoeffs(const real_T posPts[2], const real_T
  velPts[2], real_T finalTime, real_T coeffVec[4]);
static void jo_addFlatSegmentsToPPFormParts(const real_T oldbreaks[2], const
  real_T oldCoeffs[24], real_T newBreaks[4], real_T newCoefs[72]);
static void joint_trajectory_planner_ppval(const real_T pp_breaks[4], const
  real_T pp_coefs[72], const real_T x[2], real_T v[12]);
static void joint_trajectory__cubicpolytraj(const real_T wayPoints[12], const
  real_T timePoints[2], const real_T t[2], const real_T varargin_2[12], real_T
  q[12], real_T qd[12], real_T qdd[12], real_T pp_breaks[4], real_T pp_coefs[72]);
static void PolyTrajSys_computePPDerivative(const real_T pp_breaks[4], const
  real_T pp_coefs[72], real_T t, real_T ppd_breaks[4], real_T ppd_coefs[72],
  real_T ppdd_breaks[4], real_T ppdd_coefs[72]);
static void joint_trajectory_planne_ppval_a(const real_T pp_breaks[4], const
  real_T pp_coefs[72], real_T x, real_T v[6]);
static void joint_traject_matlabCodegenHa_n(ros_slros_internal_block_Subs_T *obj);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_GetP_T *obj);
static void joint_traject_matlabCodegenHa_a(ros_slros_internal_block_Publ_T *obj);
static void joint_trajecto_SystemCore_setup(robotics_slcore_internal_bloc_T *obj);
static real_T joint_trajectory_pl_rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  real_T tmp;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    joint_trajectory_planner_B.d1 = fabs(u0);
    tmp = fabs(u1);
    if (rtIsInf(u1)) {
      if (joint_trajectory_planner_B.d1 == 1.0) {
        y = 1.0;
      } else if (joint_trajectory_planner_B.d1 > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp == 0.0) {
      y = 1.0;
    } else if (tmp == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = (rtNaN);
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

static void joint_traje_generateCubicCoeffs(const real_T posPts[2], const real_T
  velPts[2], real_T finalTime, real_T coeffVec[4])
{
  real_T posPts_idx_0;
  real_T posPts_idx_1;
  real_T coeffVec_tmp;
  posPts_idx_0 = posPts[1] - (finalTime * velPts[0] + posPts[0]);
  posPts_idx_1 = velPts[1] - (0.0 * posPts[0] + velPts[0]);
  coeffVec_tmp = finalTime * finalTime;
  coeffVec[2] = 3.0 / coeffVec_tmp * posPts_idx_0 + -1.0 / finalTime *
    posPts_idx_1;
  coeffVec[0] = 1.0 / coeffVec_tmp * posPts_idx_1 + -2.0 /
    joint_trajectory_pl_rt_powd_snf(finalTime, 3.0) * posPts_idx_0;
  coeffVec[3] = posPts[0];
  coeffVec[1] = coeffVec[2];
  coeffVec[2] = velPts[0];
}

static void jo_addFlatSegmentsToPPFormParts(const real_T oldbreaks[2], const
  real_T oldCoeffs[24], real_T newBreaks[4], real_T newCoefs[72])
{
  int32_T i;
  int32_T i_0;
  int32_T coefsWithFlatStart_tmp;
  memset(&joint_trajectory_planner_B.newSegmentCoeffs[0], 0, 24U * sizeof(real_T));
  for (i = 0; i < 6; i++) {
    joint_trajectory_planner_B.newSegmentCoeffs[i + 18] = 0.0;
    joint_trajectory_planner_B.newSegmentCoeffs[i + 18] += oldCoeffs[i] * 0.0;
    joint_trajectory_planner_B.newSegmentCoeffs[i + 18] += oldCoeffs[i + 6] *
      0.0;
    joint_trajectory_planner_B.newSegmentCoeffs[i + 18] += oldCoeffs[i + 12] *
      0.0;
    joint_trajectory_planner_B.newSegmentCoeffs[i + 18] += oldCoeffs[i + 18];
  }

  memset(&joint_trajectory_planner_B.coefsWithFlatStart[0], 0, 48U * sizeof
         (real_T));
  for (i = 0; i < 4; i++) {
    for (i_0 = 0; i_0 < 6; i_0++) {
      coefsWithFlatStart_tmp = 6 * i + i_0;
      joint_trajectory_planner_B.coefsWithFlatStart[((i_0 + 1) + 12 * i) - 1] =
        joint_trajectory_planner_B.newSegmentCoeffs[coefsWithFlatStart_tmp];
      joint_trajectory_planner_B.coefsWithFlatStart[((i_0 + 7) + 12 * i) - 1] =
        oldCoeffs[coefsWithFlatStart_tmp];
    }
  }

  joint_trajectory_planner_B.holdPoint = oldbreaks[1] - oldbreaks[0];
  joint_trajectory_planner_B.evalPointVector_idx_0 =
    joint_trajectory_pl_rt_powd_snf(joint_trajectory_planner_B.holdPoint, 3.0);
  joint_trajectory_planner_B.evalPointVector_idx_1 =
    joint_trajectory_pl_rt_powd_snf(joint_trajectory_planner_B.holdPoint, 2.0);
  joint_trajectory_planner_B.evalPointVector_idx_2 =
    joint_trajectory_pl_rt_powd_snf(joint_trajectory_planner_B.holdPoint, 1.0);
  joint_trajectory_planner_B.holdPoint = joint_trajectory_pl_rt_powd_snf
    (joint_trajectory_planner_B.holdPoint, 0.0);
  memset(&joint_trajectory_planner_B.newSegmentCoeffs[0], 0, 24U * sizeof(real_T));
  for (i = 0; i < 6; i++) {
    joint_trajectory_planner_B.newSegmentCoeffs[i + 18] = 0.0;
    joint_trajectory_planner_B.newSegmentCoeffs[i + 18] +=
      joint_trajectory_planner_B.coefsWithFlatStart[i + 6] *
      joint_trajectory_planner_B.evalPointVector_idx_0;
    joint_trajectory_planner_B.newSegmentCoeffs[i + 18] +=
      joint_trajectory_planner_B.coefsWithFlatStart[i + 18] *
      joint_trajectory_planner_B.evalPointVector_idx_1;
    joint_trajectory_planner_B.newSegmentCoeffs[i + 18] +=
      joint_trajectory_planner_B.coefsWithFlatStart[i + 30] *
      joint_trajectory_planner_B.evalPointVector_idx_2;
    joint_trajectory_planner_B.newSegmentCoeffs[i + 18] +=
      joint_trajectory_planner_B.coefsWithFlatStart[i + 42] *
      joint_trajectory_planner_B.holdPoint;
  }

  memset(&newCoefs[0], 0, 72U * sizeof(real_T));
  for (i = 0; i < 4; i++) {
    for (i_0 = 0; i_0 < 12; i_0++) {
      newCoefs[((i_0 + 1) + 18 * i) - 1] =
        joint_trajectory_planner_B.coefsWithFlatStart[12 * i + i_0];
    }

    for (i_0 = 0; i_0 < 6; i_0++) {
      newCoefs[((i_0 + 13) + 18 * i) - 1] =
        joint_trajectory_planner_B.newSegmentCoeffs[6 * i + i_0];
    }
  }

  newBreaks[0] = oldbreaks[0] - 1.0;
  newBreaks[1] = oldbreaks[0];
  newBreaks[2] = oldbreaks[1];
  newBreaks[3] = oldbreaks[1] + 1.0;
}

static void joint_trajectory_planner_ppval(const real_T pp_breaks[4], const
  real_T pp_coefs[72], const real_T x[2], real_T v[12])
{
  int32_T iv0;
  int32_T j;
  int32_T b_ix;
  int32_T low_i;
  int32_T low_ip1;
  int32_T high_i;
  int32_T mid_i;
  int32_T v_tmp;
  for (b_ix = 0; b_ix < 2; b_ix++) {
    iv0 = b_ix * 6 - 1;
    if (rtIsNaN(x[b_ix])) {
      for (low_ip1 = 0; low_ip1 < 6; low_ip1++) {
        v[(iv0 + low_ip1) + 1] = x[b_ix];
      }
    } else {
      low_i = 0;
      low_ip1 = 1;
      high_i = 4;
      while (high_i > low_ip1 + 1) {
        mid_i = ((low_i + high_i) + 1) >> 1;
        if (x[b_ix] >= pp_breaks[mid_i - 1]) {
          low_i = mid_i - 1;
          low_ip1 = mid_i;
        } else {
          high_i = mid_i;
        }
      }

      low_ip1 = low_i * 6;
      joint_trajectory_planner_B.xloc = x[b_ix] - pp_breaks[low_i];
      for (low_i = 0; low_i < 6; low_i++) {
        v[(iv0 + low_i) + 1] = pp_coefs[low_ip1 + low_i];
      }

      for (low_i = 0; low_i < 3; low_i++) {
        high_i = ((low_i + 1) * 18 + low_ip1) - 1;
        for (mid_i = 0; mid_i < 6; mid_i++) {
          j = mid_i + 1;
          v_tmp = iv0 + j;
          v[v_tmp] = v[v_tmp] * joint_trajectory_planner_B.xloc +
            pp_coefs[high_i + j];
        }
      }
    }
  }
}

static void joint_trajectory__cubicpolytraj(const real_T wayPoints[12], const
  real_T timePoints[2], const real_T t[2], const real_T varargin_2[12], real_T
  q[12], real_T qd[12], real_T qdd[12], real_T pp_breaks[4], real_T pp_coefs[72])
{
  boolean_T x[2];
  int32_T ii_data;
  int32_T idx;
  int32_T b_ii;
  int32_T ii_size_idx_0;
  boolean_T exitg1;
  memset(&joint_trajectory_planner_B.coefMat[0], 0, 24U * sizeof(real_T));
  joint_trajectory_planner_B.finalTime = timePoints[1] - timePoints[0];
  for (idx = 0; idx < 6; idx++) {
    joint_trajectory_planner_B.wayPoints[0] = wayPoints[idx];
    joint_trajectory_planner_B.varargin_2[0] = varargin_2[idx];
    joint_trajectory_planner_B.wayPoints[1] = wayPoints[idx + 6];
    joint_trajectory_planner_B.varargin_2[1] = varargin_2[idx + 6];
    joint_traje_generateCubicCoeffs(joint_trajectory_planner_B.wayPoints,
      joint_trajectory_planner_B.varargin_2,
      joint_trajectory_planner_B.finalTime, joint_trajectory_planner_B.modBreaks);
    joint_trajectory_planner_B.coefMat[idx] =
      joint_trajectory_planner_B.modBreaks[0];
    joint_trajectory_planner_B.coefMat[idx + 6] =
      joint_trajectory_planner_B.modBreaks[1];
    joint_trajectory_planner_B.coefMat[idx + 12] =
      joint_trajectory_planner_B.modBreaks[2];
    joint_trajectory_planner_B.coefMat[idx + 18] =
      joint_trajectory_planner_B.modBreaks[3];
  }

  jo_addFlatSegmentsToPPFormParts(timePoints, joint_trajectory_planner_B.coefMat,
    joint_trajectory_planner_B.modBreaks, joint_trajectory_planner_B.ddCoeffs);
  pp_breaks[0] = joint_trajectory_planner_B.modBreaks[0];
  pp_breaks[1] = joint_trajectory_planner_B.modBreaks[1];
  pp_breaks[2] = joint_trajectory_planner_B.modBreaks[2];
  pp_breaks[3] = joint_trajectory_planner_B.modBreaks[3];
  memcpy(&pp_coefs[0], &joint_trajectory_planner_B.ddCoeffs[0], 72U * sizeof
         (real_T));
  joint_trajectory_planner_ppval(joint_trajectory_planner_B.modBreaks,
    joint_trajectory_planner_B.ddCoeffs, t, q);
  joint_trajectory_planner_B.derivativeBreaks[0] =
    joint_trajectory_planner_B.modBreaks[0];
  joint_trajectory_planner_B.derivativeBreaks[1] =
    joint_trajectory_planner_B.modBreaks[1];
  joint_trajectory_planner_B.derivativeBreaks[3] =
    joint_trajectory_planner_B.modBreaks[3];
  joint_trajectory_planner_B.finalTime = 0.01;
  x[0] = (t[0] > joint_trajectory_planner_B.modBreaks[2]);
  x[1] = (t[1] > joint_trajectory_planner_B.modBreaks[2]);
  idx = 0;
  ii_size_idx_0 = 1;
  b_ii = 1;
  exitg1 = false;
  while ((!exitg1) && (b_ii - 1 < 2)) {
    if (x[b_ii - 1]) {
      idx = 1;
      ii_data = b_ii;
      exitg1 = true;
    } else {
      b_ii++;
    }
  }

  if (idx == 0) {
    ii_size_idx_0 = 0;
  }

  if (ii_size_idx_0 != 0) {
    joint_trajectory_planner_B.u0 = (t[ii_data - 1] -
      joint_trajectory_planner_B.modBreaks[2]) / 2.0;
    if (joint_trajectory_planner_B.u0 < 0.01) {
      joint_trajectory_planner_B.finalTime = joint_trajectory_planner_B.u0;
    }
  }

  joint_trajectory_planner_B.derivativeBreaks[2] =
    joint_trajectory_planner_B.modBreaks[2] +
    joint_trajectory_planner_B.finalTime;
  memset(&joint_trajectory_planner_B.dCoeffs[0], 0, 72U * sizeof(real_T));
  for (idx = 0; idx < 3; idx++) {
    ii_size_idx_0 = 3 - idx;
    b_ii = idx + 1;
    for (ii_data = 0; ii_data < 18; ii_data++) {
      joint_trajectory_planner_B.dCoeffs[ii_data + 18 * (idx + 1)] =
        joint_trajectory_planner_B.ddCoeffs[(b_ii - 1) * 18 + ii_data] *
        static_cast<real_T>(ii_size_idx_0);
    }
  }

  joint_trajectory_planner_ppval(joint_trajectory_planner_B.derivativeBreaks,
    joint_trajectory_planner_B.dCoeffs, t, qd);
  memset(&joint_trajectory_planner_B.ddCoeffs[0], 0, 72U * sizeof(real_T));
  for (idx = 0; idx < 3; idx++) {
    ii_size_idx_0 = 3 - idx;
    b_ii = idx + 1;
    for (ii_data = 0; ii_data < 18; ii_data++) {
      joint_trajectory_planner_B.ddCoeffs[ii_data + 18 * (idx + 1)] =
        joint_trajectory_planner_B.dCoeffs[(b_ii - 1) * 18 + ii_data] *
        static_cast<real_T>(ii_size_idx_0);
    }
  }

  joint_trajectory_planner_ppval(joint_trajectory_planner_B.derivativeBreaks,
    joint_trajectory_planner_B.ddCoeffs, t, qdd);
}

static void PolyTrajSys_computePPDerivative(const real_T pp_breaks[4], const
  real_T pp_coefs[72], real_T t, real_T ppd_breaks[4], real_T ppd_coefs[72],
  real_T ppdd_breaks[4], real_T ppdd_coefs[72])
{
  real_T dt;
  int32_T b_i;
  int32_T i;
  int32_T ii_size_idx_0;
  int32_T ii_size_idx_1;
  real_T u0;
  ppdd_breaks[0] = pp_breaks[0];
  ppdd_breaks[1] = pp_breaks[1];
  ppdd_breaks[3] = pp_breaks[3];
  dt = 0.01;
  if (t > pp_breaks[2]) {
    ii_size_idx_0 = 1;
    ii_size_idx_1 = 1;
  } else {
    ii_size_idx_0 = 0;
    ii_size_idx_1 = 0;
  }

  if ((ii_size_idx_0 != 0) && (ii_size_idx_1 != 0)) {
    u0 = (t - pp_breaks[2]) / 2.0;
    if (u0 < 0.01) {
      dt = u0;
    }
  }

  ppdd_breaks[2] = pp_breaks[2] + dt;
  memset(&ppd_coefs[0], 0, 72U * sizeof(real_T));
  for (ii_size_idx_0 = 0; ii_size_idx_0 < 3; ii_size_idx_0++) {
    ii_size_idx_1 = 3 - ii_size_idx_0;
    b_i = ii_size_idx_0 + 1;
    for (i = 0; i < 18; i++) {
      ppd_coefs[i + 18 * (ii_size_idx_0 + 1)] = pp_coefs[(b_i - 1) * 18 + i] *
        static_cast<real_T>(ii_size_idx_1);
    }
  }

  memset(&ppdd_coefs[0], 0, 72U * sizeof(real_T));
  for (ii_size_idx_0 = 0; ii_size_idx_0 < 3; ii_size_idx_0++) {
    ii_size_idx_1 = 3 - ii_size_idx_0;
    b_i = ii_size_idx_0 + 1;
    for (i = 0; i < 18; i++) {
      ppdd_coefs[i + 18 * (ii_size_idx_0 + 1)] = ppd_coefs[(b_i - 1) * 18 + i] *
        static_cast<real_T>(ii_size_idx_1);
    }
  }

  ppd_breaks[0] = pp_breaks[0];
  ppd_breaks[1] = pp_breaks[1];
  ppd_breaks[2] = ppdd_breaks[2];
  ppd_breaks[3] = pp_breaks[3];
}

static void joint_trajectory_planne_ppval_a(const real_T pp_breaks[4], const
  real_T pp_coefs[72], real_T x, real_T v[6])
{
  real_T xloc;
  int32_T low_i;
  int32_T low_ip1;
  int32_T high_i;
  int32_T mid_i;
  if (rtIsNaN(x)) {
    for (low_ip1 = 0; low_ip1 < 6; low_ip1++) {
      v[low_ip1] = x;
    }
  } else {
    low_i = 0;
    low_ip1 = 1;
    high_i = 4;
    while (high_i > low_ip1 + 1) {
      mid_i = ((low_i + high_i) + 1) >> 1;
      if (x >= pp_breaks[mid_i - 1]) {
        low_i = mid_i - 1;
        low_ip1 = mid_i;
      } else {
        high_i = mid_i;
      }
    }

    low_ip1 = low_i * 6;
    xloc = x - pp_breaks[low_i];
    for (low_i = 0; low_i < 6; low_i++) {
      v[low_i] = pp_coefs[low_ip1 + low_i];
    }

    for (low_i = 0; low_i < 3; low_i++) {
      high_i = ((low_i + 1) * 18 + low_ip1) - 1;
      for (mid_i = 0; mid_i < 6; mid_i++) {
        v[mid_i] = pp_coefs[(high_i + mid_i) + 1] + xloc * v[mid_i];
      }
    }
  }
}

static void joint_traject_matlabCodegenHa_n(ros_slros_internal_block_Subs_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_GetP_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void joint_traject_matlabCodegenHa_a(ros_slros_internal_block_Publ_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void joint_trajecto_SystemCore_setup(robotics_slcore_internal_bloc_T *obj)
{
  obj->isInitialized = 1;
  obj->TunablePropsChanged = false;
}

// Model step function
void joint_trajectory_planner_step(void)
{
  boolean_T p;
  boolean_T b_varargout_1;
  boolean_T value;
  int32_T tmp;
  boolean_T exitg1;

  // Outputs for Atomic SubSystem: '<Root>/Subscribe'
  // MATLABSystem: '<S7>/SourceBlock' incorporates:
  //   Inport: '<S10>/In1'

  b_varargout_1 = Sub_joint_trajectory_planner_15.getLatestMessage
    (&joint_trajectory_planner_B.b_varargout_2);

  // Outputs for Enabled SubSystem: '<S7>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S10>/Enable'

  if (b_varargout_1) {
    joint_trajectory_planner_B.In1 = joint_trajectory_planner_B.b_varargout_2;
  }

  // End of MATLABSystem: '<S7>/SourceBlock'
  // End of Outputs for SubSystem: '<S7>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe'

  // MATLABSystem: '<S2>/Get Parameter'
  ParamGet_joint_trajectory_planner_26.get_parameter
    (&joint_trajectory_planner_B.t1);

  // MATLAB Function: '<Root>/MATLAB Function1' incorporates:
  //   MATLABSystem: '<S2>/Get Parameter'

  joint_trajectory_planner_B.time = joint_trajectory_planner_B.In1.Clock_.Nsec /
    1.0E+9 + joint_trajectory_planner_B.In1.Clock_.Sec;
  joint_trajectory_planner_B.delayed_time = joint_trajectory_planner_B.time -
    joint_trajectory_planner_B.t1;
  if (joint_trajectory_planner_B.delayed_time < 0.0) {
    joint_trajectory_planner_B.delayed_time = 0.0;
  }

  // SignalConversion generated from: '<S2>/Matrix Concatenate' incorporates:
  //   MATLABSystem: '<S8>/Get Parameter'
  //   MATLABSystem: '<S8>/Get Parameter1'
  //   MATLABSystem: '<S8>/Get Parameter2'
  //   MATLABSystem: '<S8>/Get Parameter3'
  //   MATLABSystem: '<S8>/Get Parameter4'
  //   MATLABSystem: '<S8>/Get Parameter5'

  ParamGet_joint_trajectory_planner_34.get_parameter
    (&joint_trajectory_planner_B.MatrixConcatenate[0]);
  ParamGet_joint_trajectory_planner_35.get_parameter
    (&joint_trajectory_planner_B.MatrixConcatenate[1]);
  ParamGet_joint_trajectory_planner_36.get_parameter
    (&joint_trajectory_planner_B.MatrixConcatenate[2]);
  ParamGet_joint_trajectory_planner_37.get_parameter
    (&joint_trajectory_planner_B.MatrixConcatenate[3]);
  ParamGet_joint_trajectory_planner_38.get_parameter
    (&joint_trajectory_planner_B.MatrixConcatenate[4]);
  ParamGet_joint_trajectory_planner_39.get_parameter
    (&joint_trajectory_planner_B.MatrixConcatenate[5]);

  // SignalConversion generated from: '<S2>/Matrix Concatenate' incorporates:
  //   MATLABSystem: '<S9>/Get Parameter'
  //   MATLABSystem: '<S9>/Get Parameter1'
  //   MATLABSystem: '<S9>/Get Parameter2'
  //   MATLABSystem: '<S9>/Get Parameter3'
  //   MATLABSystem: '<S9>/Get Parameter4'
  //   MATLABSystem: '<S9>/Get Parameter5'

  ParamGet_joint_trajectory_planner_41.get_parameter
    (&joint_trajectory_planner_B.MatrixConcatenate[6]);
  ParamGet_joint_trajectory_planner_42.get_parameter
    (&joint_trajectory_planner_B.MatrixConcatenate[7]);
  ParamGet_joint_trajectory_planner_43.get_parameter
    (&joint_trajectory_planner_B.MatrixConcatenate[8]);
  ParamGet_joint_trajectory_planner_44.get_parameter
    (&joint_trajectory_planner_B.MatrixConcatenate[9]);
  ParamGet_joint_trajectory_planner_45.get_parameter
    (&joint_trajectory_planner_B.MatrixConcatenate[10]);
  ParamGet_joint_trajectory_planner_46.get_parameter
    (&joint_trajectory_planner_B.MatrixConcatenate[11]);

  // SignalConversion generated from: '<Root>/Polynomial Trajectory' incorporates:
  //   MATLABSystem: '<S2>/Get Parameter'
  //   MATLABSystem: '<S2>/Get Parameter3'

  ParamGet_joint_trajectory_planner_49.get_parameter
    (&joint_trajectory_planner_B.signal2[1]);
  joint_trajectory_planner_B.signal2[0] = joint_trajectory_planner_B.t1;

  // MATLABSystem: '<Root>/Polynomial Trajectory' incorporates:
  //   MATLAB Function: '<Root>/MATLAB Function1'

  b_varargout_1 = false;
  p = true;
  joint_trajectory_planner_B.i = 0;
  exitg1 = false;
  while ((!exitg1) && (joint_trajectory_planner_B.i < 12)) {
    if (!(joint_trajectory_planner_DW.obj.VelocityBoundaryCondition[joint_trajectory_planner_B.i]
          ==
          joint_trajectory_planner_P.PolynomialTrajectory_VelocityBo[joint_trajectory_planner_B.i]))
    {
      p = false;
      exitg1 = true;
    } else {
      joint_trajectory_planner_B.i++;
    }
  }

  if (p) {
    b_varargout_1 = true;
  }

  if (!b_varargout_1) {
    if (joint_trajectory_planner_DW.obj.isInitialized == 1) {
      joint_trajectory_planner_DW.obj.TunablePropsChanged = true;
    }

    memcpy(&joint_trajectory_planner_DW.obj.VelocityBoundaryCondition[0],
           &joint_trajectory_planner_P.PolynomialTrajectory_VelocityBo[0], 12U *
           sizeof(real_T));
  }

  if (joint_trajectory_planner_DW.obj.TunablePropsChanged) {
    joint_trajectory_planner_DW.obj.TunablePropsChanged = false;
  }

  joint_trajectory__cubicpolytraj(joint_trajectory_planner_B.MatrixConcatenate,
    joint_trajectory_planner_B.signal2, joint_trajectory_planner_B.signal2,
    joint_trajectory_planner_DW.obj.VelocityBoundaryCondition,
    joint_trajectory_planner_B.unusedU1, joint_trajectory_planner_B.unusedU2,
    joint_trajectory_planner_B.unusedU3, joint_trajectory_planner_B.pp_breaks,
    joint_trajectory_planner_B.pp_coefs);
  PolyTrajSys_computePPDerivative(joint_trajectory_planner_B.pp_breaks,
    joint_trajectory_planner_B.pp_coefs, joint_trajectory_planner_B.time,
    joint_trajectory_planner_B.ppd_breaks, joint_trajectory_planner_B.ppd_coefs,
    joint_trajectory_planner_B.ppdd_breaks,
    joint_trajectory_planner_B.ppdd_coefs);

  // MATLABSystem: '<S2>/Get Parameter4'
  ParamGet_joint_trajectory_planner_50.get_parameter
    (&joint_trajectory_planner_B.value_m);

  // MATLABSystem: '<S2>/Get Parameter2'
  ParamGet_joint_trajectory_planner_28.get_parameter
    (&joint_trajectory_planner_B.t1);

  // MATLABSystem: '<S2>/Get Parameter1'
  ParamGet_joint_trajectory_planner_27.get_parameter
    (&joint_trajectory_planner_B.value);

  // MATLAB Function: '<Root>/MATLAB Function4' incorporates:
  //   MATLAB Function: '<Root>/MATLAB Function1'
  //   MATLABSystem: '<S2>/Get Parameter1'
  //   MATLABSystem: '<S2>/Get Parameter2'

  for (joint_trajectory_planner_B.i = 0; joint_trajectory_planner_B.i < 6;
       joint_trajectory_planner_B.i++) {
    joint_trajectory_planner_B.max_vel[joint_trajectory_planner_B.i] =
      joint_trajectory_planner_B.t1;
    joint_trajectory_planner_B.acc[joint_trajectory_planner_B.i] =
      joint_trajectory_planner_B.value;
  }

  memset(&joint_trajectory_planner_B.t[0], 0, 18U * sizeof(real_T));
  for (joint_trajectory_planner_B.i = 0; joint_trajectory_planner_B.i < 36;
       joint_trajectory_planner_B.i++) {
    joint_trajectory_planner_B.d_t[joint_trajectory_planner_B.i] = 0.0;
    joint_trajectory_planner_B.steps[joint_trajectory_planner_B.i] = 1.0;
  }

  for (joint_trajectory_planner_B.i = 0; joint_trajectory_planner_B.i < 6;
       joint_trajectory_planner_B.i++) {
    joint_trajectory_planner_B.t1 =
      joint_trajectory_planner_B.d_t[joint_trajectory_planner_B.i];
    joint_trajectory_planner_B.value =
      joint_trajectory_planner_B.steps[joint_trajectory_planner_B.i];
    joint_trajectory_planner_B.t_up =
      joint_trajectory_planner_B.max_vel[joint_trajectory_planner_B.i] /
      joint_trajectory_planner_B.acc[joint_trajectory_planner_B.i];
    joint_trajectory_planner_B.dist =
      joint_trajectory_planner_B.MatrixConcatenate[joint_trajectory_planner_B.i
      + 6] -
      joint_trajectory_planner_B.MatrixConcatenate[joint_trajectory_planner_B.i];
    if (joint_trajectory_planner_B.dist < 0.0) {
      joint_trajectory_planner_B.signes[joint_trajectory_planner_B.i] = -1.0;
    } else if (joint_trajectory_planner_B.dist > 0.0) {
      joint_trajectory_planner_B.signes[joint_trajectory_planner_B.i] = 1.0;
    } else if (joint_trajectory_planner_B.dist == 0.0) {
      joint_trajectory_planner_B.signes[joint_trajectory_planner_B.i] = 0.0;
    } else {
      joint_trajectory_planner_B.signes[joint_trajectory_planner_B.i] = (rtNaN);
    }

    joint_trajectory_planner_B.dist = fabs(joint_trajectory_planner_B.dist);
    joint_trajectory_planner_B.d = joint_trajectory_planner_B.t_up *
      joint_trajectory_planner_B.max_vel[joint_trajectory_planner_B.i];
    if (joint_trajectory_planner_B.dist < joint_trajectory_planner_B.d) {
      joint_trajectory_planner_B.t_up = sqrt(joint_trajectory_planner_B.dist /
        joint_trajectory_planner_B.acc[joint_trajectory_planner_B.i]);
      joint_trajectory_planner_B.t[joint_trajectory_planner_B.i + 6] = 0.0;
      joint_trajectory_planner_B.t[joint_trajectory_planner_B.i] =
        joint_trajectory_planner_B.t_up;
      joint_trajectory_planner_B.t[joint_trajectory_planner_B.i + 12] =
        joint_trajectory_planner_B.t_up;
    } else {
      joint_trajectory_planner_B.t[joint_trajectory_planner_B.i] =
        joint_trajectory_planner_B.t_up;
      joint_trajectory_planner_B.t[joint_trajectory_planner_B.i + 12] =
        joint_trajectory_planner_B.t_up;
      joint_trajectory_planner_B.t[joint_trajectory_planner_B.i + 6] =
        (joint_trajectory_planner_B.dist - joint_trajectory_planner_B.d) /
        joint_trajectory_planner_B.max_vel[joint_trajectory_planner_B.i];
    }

    joint_trajectory_planner_B.dist =
      joint_trajectory_planner_B.t[joint_trajectory_planner_B.i + 6];
    joint_trajectory_planner_B.d =
      joint_trajectory_planner_B.t[joint_trajectory_planner_B.i + 12];
    if (((joint_trajectory_planner_B.delayed_time -
          joint_trajectory_planner_B.t[joint_trajectory_planner_B.i]) -
         joint_trajectory_planner_B.dist) - joint_trajectory_planner_B.d >= 0.0)
    {
      joint_trajectory_planner_B.value =
        joint_trajectory_planner_B.steps[joint_trajectory_planner_B.i] + 1.0;
      joint_trajectory_planner_B.t1 =
        ((joint_trajectory_planner_B.d_t[joint_trajectory_planner_B.i] +
          joint_trajectory_planner_B.t[joint_trajectory_planner_B.i]) +
         joint_trajectory_planner_B.dist) + joint_trajectory_planner_B.d;
    }

    joint_trajectory_planner_B.steps[joint_trajectory_planner_B.i] =
      joint_trajectory_planner_B.value;
    joint_trajectory_planner_B.d_t[joint_trajectory_planner_B.i] =
      joint_trajectory_planner_B.t1;
  }

  // MATLAB Function: '<Root>/MATLAB Function' incorporates:
  //   BusAssignment: '<Root>/Bus Assignment'
  //   Constant: '<S1>/Constant'

  joint_trajectory_planner_B.msg = joint_trajectory_planner_P.Constant_Value;

  // BusAssignment: '<Root>/Bus Assignment' incorporates:
  //   MATLAB Function: '<Root>/MATLAB Function'

  joint_trajectory_planner_B.msg.TimeFromStart.Sec =
    joint_trajectory_planner_B.In1.Clock_.Sec;
  joint_trajectory_planner_B.msg.TimeFromStart.Nsec =
    joint_trajectory_planner_B.In1.Clock_.Nsec;

  // MATLAB Function: '<Root>/MATLAB Function'
  joint_trajectory_planner_B.msg.Velocities_SL_Info.CurrentLength = 6U;

  // Switch: '<Root>/Switch' incorporates:
  //   MATLABSystem: '<S2>/Get Parameter4'

  b_varargout_1 = (joint_trajectory_planner_B.value_m >
                   joint_trajectory_planner_P.Switch_Threshold);

  // MATLABSystem: '<Root>/Polynomial Trajectory' incorporates:
  //   MATLAB Function: '<Root>/MATLAB Function1'

  joint_trajectory_planne_ppval_a(joint_trajectory_planner_B.ppd_breaks,
    joint_trajectory_planner_B.ppd_coefs, joint_trajectory_planner_B.time,
    joint_trajectory_planner_B.dv);

  // MATLAB Function: '<Root>/MATLAB Function'
  joint_trajectory_planner_B.msg.Positions_SL_Info.CurrentLength = 6U;

  // Switch: '<Root>/Switch1' incorporates:
  //   MATLABSystem: '<S2>/Get Parameter4'

  p = (joint_trajectory_planner_B.value_m >
       joint_trajectory_planner_P.Switch1_Threshold);

  // MATLABSystem: '<Root>/Polynomial Trajectory' incorporates:
  //   MATLAB Function: '<Root>/MATLAB Function1'

  joint_trajectory_planne_ppval_a(joint_trajectory_planner_B.pp_breaks,
    joint_trajectory_planner_B.pp_coefs, joint_trajectory_planner_B.time,
    joint_trajectory_planner_B.dv1);

  // MATLAB Function: '<Root>/MATLAB Function'
  joint_trajectory_planner_B.msg.Accelerations_SL_Info.CurrentLength = 6U;

  // Switch: '<Root>/Switch2' incorporates:
  //   MATLABSystem: '<S2>/Get Parameter4'

  value = (joint_trajectory_planner_B.value_m >
           joint_trajectory_planner_P.Switch2_Threshold);

  // MATLABSystem: '<Root>/Polynomial Trajectory' incorporates:
  //   MATLAB Function: '<Root>/MATLAB Function1'

  joint_trajectory_planne_ppval_a(joint_trajectory_planner_B.ppdd_breaks,
    joint_trajectory_planner_B.ppdd_coefs, joint_trajectory_planner_B.time,
    joint_trajectory_planner_B.dv2);
  for (joint_trajectory_planner_B.value_m = 0;
       joint_trajectory_planner_B.value_m < 6;
       joint_trajectory_planner_B.value_m++) {
    // MATLAB Function: '<Root>/MATLAB Function4' incorporates:
    //   MATLAB Function: '<Root>/MATLAB Function1'

    joint_trajectory_planner_B.t1 = joint_trajectory_planner_B.delayed_time -
      joint_trajectory_planner_B.d_t[joint_trajectory_planner_B.value_m];
    joint_trajectory_planner_B.rtb_q_tmp = (static_cast<int32_T>
      (joint_trajectory_planner_B.steps[joint_trajectory_planner_B.value_m]) - 1)
      * 6 + joint_trajectory_planner_B.value_m;
    if (joint_trajectory_planner_B.steps[joint_trajectory_planner_B.value_m] ==
        2.0) {
      joint_trajectory_planner_B.time =
        joint_trajectory_planner_B.MatrixConcatenate[joint_trajectory_planner_B.value_m
        + 6];
      joint_trajectory_planner_B.t1 = 0.0;
      joint_trajectory_planner_B.value = 0.0;
    } else if (joint_trajectory_planner_B.t1 <= 0.0) {
      joint_trajectory_planner_B.time =
        joint_trajectory_planner_B.MatrixConcatenate[joint_trajectory_planner_B.rtb_q_tmp];
      joint_trajectory_planner_B.t1 = 0.0;
      joint_trajectory_planner_B.value = 0.0;
    } else {
      joint_trajectory_planner_B.dist = joint_trajectory_planner_B.t1 -
        joint_trajectory_planner_B.t[joint_trajectory_planner_B.rtb_q_tmp];
      if (joint_trajectory_planner_B.dist < 0.0) {
        joint_trajectory_planner_B.time = 0.5 *
          joint_trajectory_planner_B.acc[joint_trajectory_planner_B.value_m] *
          (joint_trajectory_planner_B.t1 * joint_trajectory_planner_B.t1) *
          joint_trajectory_planner_B.signes[joint_trajectory_planner_B.value_m]
          + joint_trajectory_planner_B.MatrixConcatenate[joint_trajectory_planner_B.rtb_q_tmp];
        joint_trajectory_planner_B.t1 =
          joint_trajectory_planner_B.acc[joint_trajectory_planner_B.value_m] *
          joint_trajectory_planner_B.t1 *
          joint_trajectory_planner_B.signes[joint_trajectory_planner_B.value_m];
        joint_trajectory_planner_B.value =
          joint_trajectory_planner_B.acc[joint_trajectory_planner_B.value_m] *
          joint_trajectory_planner_B.signes[joint_trajectory_planner_B.value_m];
      } else {
        joint_trajectory_planner_B.i = joint_trajectory_planner_B.rtb_q_tmp + 6;
        joint_trajectory_planner_B.dist -=
          joint_trajectory_planner_B.t[joint_trajectory_planner_B.i];
        if (joint_trajectory_planner_B.dist < 0.0) {
          joint_trajectory_planner_B.time = (joint_trajectory_planner_B.t1 -
            joint_trajectory_planner_B.t[joint_trajectory_planner_B.rtb_q_tmp] /
            2.0) *
            joint_trajectory_planner_B.max_vel[joint_trajectory_planner_B.value_m]
            * joint_trajectory_planner_B.signes[joint_trajectory_planner_B.value_m]
            + joint_trajectory_planner_B.MatrixConcatenate[joint_trajectory_planner_B.rtb_q_tmp];
          joint_trajectory_planner_B.t1 =
            joint_trajectory_planner_B.max_vel[joint_trajectory_planner_B.value_m]
            * joint_trajectory_planner_B.signes[joint_trajectory_planner_B.value_m];
          joint_trajectory_planner_B.value = 0.0;
        } else {
          tmp = joint_trajectory_planner_B.rtb_q_tmp + 12;
          if (joint_trajectory_planner_B.dist - joint_trajectory_planner_B.t[tmp]
              < 0.0) {
            joint_trajectory_planner_B.t1 =
              ((joint_trajectory_planner_B.t[joint_trajectory_planner_B.i] +
                joint_trajectory_planner_B.t[joint_trajectory_planner_B.rtb_q_tmp])
               + joint_trajectory_planner_B.t[tmp]) -
              joint_trajectory_planner_B.t1;
            joint_trajectory_planner_B.time =
              joint_trajectory_planner_B.MatrixConcatenate[(static_cast<int32_T>
              (joint_trajectory_planner_B.steps[joint_trajectory_planner_B.value_m]
               + 1.0) - 1) * 6 + joint_trajectory_planner_B.value_m] - 0.5 *
              joint_trajectory_planner_B.acc[joint_trajectory_planner_B.value_m]
              * (joint_trajectory_planner_B.t1 * joint_trajectory_planner_B.t1) *
              joint_trajectory_planner_B.signes[joint_trajectory_planner_B.value_m];
            joint_trajectory_planner_B.t1 = joint_trajectory_planner_B.t1 *
              joint_trajectory_planner_B.acc[joint_trajectory_planner_B.value_m]
              * joint_trajectory_planner_B.signes[joint_trajectory_planner_B.value_m];
            joint_trajectory_planner_B.value =
              -joint_trajectory_planner_B.acc[joint_trajectory_planner_B.value_m]
              * joint_trajectory_planner_B.signes[joint_trajectory_planner_B.value_m];
          } else {
            joint_trajectory_planner_B.time =
              joint_trajectory_planner_B.MatrixConcatenate[joint_trajectory_planner_B.rtb_q_tmp];
            joint_trajectory_planner_B.t1 = 0.0;
            joint_trajectory_planner_B.value = 0.0;
          }
        }
      }
    }

    // Switch: '<Root>/Switch'
    if (b_varargout_1) {
      // MATLAB Function: '<Root>/MATLAB Function'
      joint_trajectory_planner_B.msg.Velocities[joint_trajectory_planner_B.value_m]
        = joint_trajectory_planner_B.dv[joint_trajectory_planner_B.value_m];
    } else {
      // MATLAB Function: '<Root>/MATLAB Function'
      joint_trajectory_planner_B.msg.Velocities[joint_trajectory_planner_B.value_m]
        = joint_trajectory_planner_B.t1;
    }

    // Switch: '<Root>/Switch1'
    if (p) {
      // MATLAB Function: '<Root>/MATLAB Function'
      joint_trajectory_planner_B.msg.Positions[joint_trajectory_planner_B.value_m]
        = joint_trajectory_planner_B.dv1[joint_trajectory_planner_B.value_m];
    } else {
      // MATLAB Function: '<Root>/MATLAB Function'
      joint_trajectory_planner_B.msg.Positions[joint_trajectory_planner_B.value_m]
        = joint_trajectory_planner_B.time;
    }

    // Switch: '<Root>/Switch2'
    if (value) {
      // MATLAB Function: '<Root>/MATLAB Function'
      joint_trajectory_planner_B.msg.Accelerations[joint_trajectory_planner_B.value_m]
        = joint_trajectory_planner_B.dv2[joint_trajectory_planner_B.value_m];
    } else {
      // MATLAB Function: '<Root>/MATLAB Function'
      joint_trajectory_planner_B.msg.Accelerations[joint_trajectory_planner_B.value_m]
        = joint_trajectory_planner_B.value;
    }
  }

  // Outputs for Atomic SubSystem: '<Root>/Publish'
  // MATLABSystem: '<S6>/SinkBlock'
  Pub_joint_trajectory_planner_5.publish(&joint_trajectory_planner_B.msg);

  // End of Outputs for SubSystem: '<Root>/Publish'
}

// Model initialize function
void joint_trajectory_planner_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    char_T tmp[7];
    char_T tmp_0[13];
    char_T tmp_1[12];
    char_T tmp_2[10];
    char_T tmp_3[4];
    int32_T i;
    static const char_T tmp_4[6] = { '/', 'c', 'l', 'o', 'c', 'k' };

    static const char_T tmp_5[17] = { '/', 'j', 'o', 'i', 'n', 't', '_', 't',
      'r', 'a', 'j', 'e', 'c', 't', 'o', 'r', 'y' };

    static const char_T tmp_6[12] = { '/', 's', 't', 'a', 'r', 't', '_', 'd',
      'e', 'l', 'a', 'y' };

    static const char_T tmp_7[11] = { '/', 'q', '1', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_8[11] = { '/', 'q', '2', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_9[11] = { '/', 'q', '3', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_a[11] = { '/', 'q', '4', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_b[11] = { '/', 'q', '5', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_c[11] = { '/', 'q', '6', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_d[9] = { '/', 'q', '1', '_', 'f', 'i', 'n', 'a', 'l'
    };

    static const char_T tmp_e[9] = { '/', 'q', '2', '_', 'f', 'i', 'n', 'a', 'l'
    };

    static const char_T tmp_f[9] = { '/', 'q', '3', '_', 'f', 'i', 'n', 'a', 'l'
    };

    static const char_T tmp_g[9] = { '/', 'q', '4', '_', 'f', 'i', 'n', 'a', 'l'
    };

    static const char_T tmp_h[9] = { '/', 'q', '5', '_', 'f', 'i', 'n', 'a', 'l'
    };

    static const char_T tmp_i[9] = { '/', 'q', '6', '_', 'f', 'i', 'n', 'a', 'l'
    };

    static const char_T tmp_j[15] = { '/', 't', 'r', 'a', 'j', 'e', 'c', 't',
      'o', 'r', 'y', 'T', 'y', 'p', 'e' };

    static const char_T tmp_k[12] = { '/', 'm', 'a', 'x', '_', 'a', 'n', 'g',
      '_', 'v', 'e', 'l' };

    static const char_T tmp_l[12] = { '/', 'm', 'a', 'x', '_', 'a', 'n', 'g',
      '_', 'a', 'c', 'c' };

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S7>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S10>/Out1'
    joint_trajectory_planner_B.In1 = joint_trajectory_planner_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S7>/Enabled Subsystem'

    // Start for MATLABSystem: '<S7>/SourceBlock'
    joint_trajectory_planner_DW.obj_c.matlabCodegenIsDeleted = false;
    joint_trajectory_planner_DW.obj_c.isInitialized = 1;
    for (i = 0; i < 6; i++) {
      tmp[i] = tmp_4[i];
    }

    tmp[6] = '\x00';
    Sub_joint_trajectory_planner_15.createSubscriber(tmp, 1);
    joint_trajectory_planner_DW.obj_c.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S7>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish'
    // Start for MATLABSystem: '<S6>/SinkBlock'
    joint_trajectory_planner_DW.obj_dw0.matlabCodegenIsDeleted = false;
    joint_trajectory_planner_DW.obj_dw0.isInitialized = 1;
    for (i = 0; i < 17; i++) {
      joint_trajectory_planner_B.cv[i] = tmp_5[i];
    }

    joint_trajectory_planner_B.cv[17] = '\x00';
    Pub_joint_trajectory_planner_5.createPublisher(joint_trajectory_planner_B.cv,
      1);
    joint_trajectory_planner_DW.obj_dw0.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish'

    // Start for MATLABSystem: '<S2>/Get Parameter'
    joint_trajectory_planner_DW.obj_g.matlabCodegenIsDeleted = false;
    joint_trajectory_planner_DW.obj_g.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      tmp_0[i] = tmp_6[i];
    }

    tmp_0[12] = '\x00';
    ParamGet_joint_trajectory_planner_26.initialize(tmp_0);
    ParamGet_joint_trajectory_planner_26.initialize_error_codes(0, 1, 2, 3);
    ParamGet_joint_trajectory_planner_26.set_initial_value(20.0);
    joint_trajectory_planner_DW.obj_g.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S2>/Get Parameter'

    // Start for MATLABSystem: '<S8>/Get Parameter'
    joint_trajectory_planner_DW.obj_bh.matlabCodegenIsDeleted = false;
    joint_trajectory_planner_DW.obj_bh.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      tmp_1[i] = tmp_7[i];
    }

    tmp_1[11] = '\x00';
    ParamGet_joint_trajectory_planner_34.initialize(tmp_1);
    ParamGet_joint_trajectory_planner_34.initialize_error_codes(0, 1, 2, 3);
    ParamGet_joint_trajectory_planner_34.set_initial_value(0.0);
    joint_trajectory_planner_DW.obj_bh.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S8>/Get Parameter'

    // Start for MATLABSystem: '<S8>/Get Parameter1'
    joint_trajectory_planner_DW.obj_m.matlabCodegenIsDeleted = false;
    joint_trajectory_planner_DW.obj_m.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      tmp_1[i] = tmp_8[i];
    }

    tmp_1[11] = '\x00';
    ParamGet_joint_trajectory_planner_35.initialize(tmp_1);
    ParamGet_joint_trajectory_planner_35.initialize_error_codes(0, 1, 2, 3);
    ParamGet_joint_trajectory_planner_35.set_initial_value(0.0);
    joint_trajectory_planner_DW.obj_m.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S8>/Get Parameter1'

    // Start for MATLABSystem: '<S8>/Get Parameter2'
    joint_trajectory_planner_DW.obj_e.matlabCodegenIsDeleted = false;
    joint_trajectory_planner_DW.obj_e.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      tmp_1[i] = tmp_9[i];
    }

    tmp_1[11] = '\x00';
    ParamGet_joint_trajectory_planner_36.initialize(tmp_1);
    ParamGet_joint_trajectory_planner_36.initialize_error_codes(0, 1, 2, 3);
    ParamGet_joint_trajectory_planner_36.set_initial_value(0.0);
    joint_trajectory_planner_DW.obj_e.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S8>/Get Parameter2'

    // Start for MATLABSystem: '<S8>/Get Parameter3'
    joint_trajectory_planner_DW.obj_dw.matlabCodegenIsDeleted = false;
    joint_trajectory_planner_DW.obj_dw.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      tmp_1[i] = tmp_a[i];
    }

    tmp_1[11] = '\x00';
    ParamGet_joint_trajectory_planner_37.initialize(tmp_1);
    ParamGet_joint_trajectory_planner_37.initialize_error_codes(0, 1, 2, 3);
    ParamGet_joint_trajectory_planner_37.set_initial_value(0.0);
    joint_trajectory_planner_DW.obj_dw.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S8>/Get Parameter3'

    // Start for MATLABSystem: '<S8>/Get Parameter4'
    joint_trajectory_planner_DW.obj_bs.matlabCodegenIsDeleted = false;
    joint_trajectory_planner_DW.obj_bs.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      tmp_1[i] = tmp_b[i];
    }

    tmp_1[11] = '\x00';
    ParamGet_joint_trajectory_planner_38.initialize(tmp_1);
    ParamGet_joint_trajectory_planner_38.initialize_error_codes(0, 1, 2, 3);
    ParamGet_joint_trajectory_planner_38.set_initial_value(0.0);
    joint_trajectory_planner_DW.obj_bs.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S8>/Get Parameter4'

    // Start for MATLABSystem: '<S8>/Get Parameter5'
    joint_trajectory_planner_DW.obj_a.matlabCodegenIsDeleted = false;
    joint_trajectory_planner_DW.obj_a.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      tmp_1[i] = tmp_c[i];
    }

    tmp_1[11] = '\x00';
    ParamGet_joint_trajectory_planner_39.initialize(tmp_1);
    ParamGet_joint_trajectory_planner_39.initialize_error_codes(0, 1, 2, 3);
    ParamGet_joint_trajectory_planner_39.set_initial_value(0.0);
    joint_trajectory_planner_DW.obj_a.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S8>/Get Parameter5'

    // Start for MATLABSystem: '<S9>/Get Parameter'
    joint_trajectory_planner_DW.obj_j.matlabCodegenIsDeleted = false;
    joint_trajectory_planner_DW.obj_j.isInitialized = 1;
    for (i = 0; i < 9; i++) {
      tmp_2[i] = tmp_d[i];
    }

    tmp_2[9] = '\x00';
    ParamGet_joint_trajectory_planner_41.initialize(tmp_2);
    ParamGet_joint_trajectory_planner_41.initialize_error_codes(0, 1, 2, 3);
    ParamGet_joint_trajectory_planner_41.set_initial_value(6.28);
    joint_trajectory_planner_DW.obj_j.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S9>/Get Parameter'

    // Start for MATLABSystem: '<S9>/Get Parameter1'
    joint_trajectory_planner_DW.obj_b.matlabCodegenIsDeleted = false;
    joint_trajectory_planner_DW.obj_b.isInitialized = 1;
    for (i = 0; i < 9; i++) {
      tmp_2[i] = tmp_e[i];
    }

    tmp_2[9] = '\x00';
    ParamGet_joint_trajectory_planner_42.initialize(tmp_2);
    ParamGet_joint_trajectory_planner_42.initialize_error_codes(0, 1, 2, 3);
    ParamGet_joint_trajectory_planner_42.set_initial_value(6.28);
    joint_trajectory_planner_DW.obj_b.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S9>/Get Parameter1'

    // Start for MATLABSystem: '<S9>/Get Parameter2'
    joint_trajectory_planner_DW.obj_d.matlabCodegenIsDeleted = false;
    joint_trajectory_planner_DW.obj_d.isInitialized = 1;
    for (i = 0; i < 9; i++) {
      tmp_2[i] = tmp_f[i];
    }

    tmp_2[9] = '\x00';
    ParamGet_joint_trajectory_planner_43.initialize(tmp_2);
    ParamGet_joint_trajectory_planner_43.initialize_error_codes(0, 1, 2, 3);
    ParamGet_joint_trajectory_planner_43.set_initial_value(6.28);
    joint_trajectory_planner_DW.obj_d.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S9>/Get Parameter2'

    // Start for MATLABSystem: '<S9>/Get Parameter3'
    joint_trajectory_planner_DW.obj_js.matlabCodegenIsDeleted = false;
    joint_trajectory_planner_DW.obj_js.isInitialized = 1;
    for (i = 0; i < 9; i++) {
      tmp_2[i] = tmp_g[i];
    }

    tmp_2[9] = '\x00';
    ParamGet_joint_trajectory_planner_44.initialize(tmp_2);
    ParamGet_joint_trajectory_planner_44.initialize_error_codes(0, 1, 2, 3);
    ParamGet_joint_trajectory_planner_44.set_initial_value(6.28);
    joint_trajectory_planner_DW.obj_js.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S9>/Get Parameter3'

    // Start for MATLABSystem: '<S9>/Get Parameter4'
    joint_trajectory_planner_DW.obj_h.matlabCodegenIsDeleted = false;
    joint_trajectory_planner_DW.obj_h.isInitialized = 1;
    for (i = 0; i < 9; i++) {
      tmp_2[i] = tmp_h[i];
    }

    tmp_2[9] = '\x00';
    ParamGet_joint_trajectory_planner_45.initialize(tmp_2);
    ParamGet_joint_trajectory_planner_45.initialize_error_codes(0, 1, 2, 3);
    ParamGet_joint_trajectory_planner_45.set_initial_value(6.28);
    joint_trajectory_planner_DW.obj_h.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S9>/Get Parameter4'

    // Start for MATLABSystem: '<S9>/Get Parameter5'
    joint_trajectory_planner_DW.obj_n.matlabCodegenIsDeleted = false;
    joint_trajectory_planner_DW.obj_n.isInitialized = 1;
    for (i = 0; i < 9; i++) {
      tmp_2[i] = tmp_i[i];
    }

    tmp_2[9] = '\x00';
    ParamGet_joint_trajectory_planner_46.initialize(tmp_2);
    ParamGet_joint_trajectory_planner_46.initialize_error_codes(0, 1, 2, 3);
    ParamGet_joint_trajectory_planner_46.set_initial_value(6.28);
    joint_trajectory_planner_DW.obj_n.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S9>/Get Parameter5'

    // Start for MATLABSystem: '<S2>/Get Parameter3'
    joint_trajectory_planner_DW.obj_e0.matlabCodegenIsDeleted = false;
    joint_trajectory_planner_DW.obj_e0.isInitialized = 1;
    tmp_3[0] = '/';
    tmp_3[1] = 't';
    tmp_3[2] = 'f';
    tmp_3[3] = '\x00';
    ParamGet_joint_trajectory_planner_49.initialize(tmp_3);
    ParamGet_joint_trajectory_planner_49.initialize_error_codes(0, 1, 2, 3);
    ParamGet_joint_trajectory_planner_49.set_initial_value(60.0);
    joint_trajectory_planner_DW.obj_e0.isSetupComplete = true;

    // Start for MATLABSystem: '<Root>/Polynomial Trajectory'
    memset(&joint_trajectory_planner_DW.obj.AccelerationBoundaryCondition[0], 0,
           10U * sizeof(real_T));
    joint_trajectory_planner_DW.obj.isInitialized = 0;
    if (joint_trajectory_planner_DW.obj.isInitialized == 1) {
      joint_trajectory_planner_DW.obj.TunablePropsChanged = true;
    }

    memcpy(&joint_trajectory_planner_DW.obj.VelocityBoundaryCondition[0],
           &joint_trajectory_planner_P.PolynomialTrajectory_VelocityBo[0], 12U *
           sizeof(real_T));
    joint_trajecto_SystemCore_setup(&joint_trajectory_planner_DW.obj);

    // End of Start for MATLABSystem: '<Root>/Polynomial Trajectory'

    // Start for MATLABSystem: '<S2>/Get Parameter4'
    joint_trajectory_planner_DW.obj_gm.matlabCodegenIsDeleted = false;
    joint_trajectory_planner_DW.obj_gm.isInitialized = 1;
    for (i = 0; i < 15; i++) {
      joint_trajectory_planner_B.cv1[i] = tmp_j[i];
    }

    joint_trajectory_planner_B.cv1[15] = '\x00';
    ParamGet_joint_trajectory_planner_50.initialize
      (joint_trajectory_planner_B.cv1);
    ParamGet_joint_trajectory_planner_50.initialize_error_codes(0, 1, 2, 3);
    ParamGet_joint_trajectory_planner_50.set_initial_value(1);
    joint_trajectory_planner_DW.obj_gm.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S2>/Get Parameter4'

    // Start for MATLABSystem: '<S2>/Get Parameter2'
    joint_trajectory_planner_DW.obj_o.matlabCodegenIsDeleted = false;
    joint_trajectory_planner_DW.obj_o.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      tmp_0[i] = tmp_k[i];
    }

    tmp_0[12] = '\x00';
    ParamGet_joint_trajectory_planner_28.initialize(tmp_0);
    ParamGet_joint_trajectory_planner_28.initialize_error_codes(0, 1, 2, 3);
    ParamGet_joint_trajectory_planner_28.set_initial_value(1.0);
    joint_trajectory_planner_DW.obj_o.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S2>/Get Parameter2'

    // Start for MATLABSystem: '<S2>/Get Parameter1'
    joint_trajectory_planner_DW.obj_l.matlabCodegenIsDeleted = false;
    joint_trajectory_planner_DW.obj_l.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      tmp_0[i] = tmp_l[i];
    }

    tmp_0[12] = '\x00';
    ParamGet_joint_trajectory_planner_27.initialize(tmp_0);
    ParamGet_joint_trajectory_planner_27.initialize_error_codes(0, 1, 2, 3);
    ParamGet_joint_trajectory_planner_27.set_initial_value(1.0);
    joint_trajectory_planner_DW.obj_l.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S2>/Get Parameter1'
  }
}

// Model terminate function
void joint_trajectory_planner_terminate(void)
{
  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S7>/SourceBlock'
  joint_traject_matlabCodegenHa_n(&joint_trajectory_planner_DW.obj_c);

  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for MATLABSystem: '<S2>/Get Parameter'
  matlabCodegenHandle_matlabCodeg(&joint_trajectory_planner_DW.obj_g);

  // Terminate for MATLABSystem: '<S8>/Get Parameter'
  matlabCodegenHandle_matlabCodeg(&joint_trajectory_planner_DW.obj_bh);

  // Terminate for MATLABSystem: '<S8>/Get Parameter1'
  matlabCodegenHandle_matlabCodeg(&joint_trajectory_planner_DW.obj_m);

  // Terminate for MATLABSystem: '<S8>/Get Parameter2'
  matlabCodegenHandle_matlabCodeg(&joint_trajectory_planner_DW.obj_e);

  // Terminate for MATLABSystem: '<S8>/Get Parameter3'
  matlabCodegenHandle_matlabCodeg(&joint_trajectory_planner_DW.obj_dw);

  // Terminate for MATLABSystem: '<S8>/Get Parameter4'
  matlabCodegenHandle_matlabCodeg(&joint_trajectory_planner_DW.obj_bs);

  // Terminate for MATLABSystem: '<S8>/Get Parameter5'
  matlabCodegenHandle_matlabCodeg(&joint_trajectory_planner_DW.obj_a);

  // Terminate for MATLABSystem: '<S9>/Get Parameter'
  matlabCodegenHandle_matlabCodeg(&joint_trajectory_planner_DW.obj_j);

  // Terminate for MATLABSystem: '<S9>/Get Parameter1'
  matlabCodegenHandle_matlabCodeg(&joint_trajectory_planner_DW.obj_b);

  // Terminate for MATLABSystem: '<S9>/Get Parameter2'
  matlabCodegenHandle_matlabCodeg(&joint_trajectory_planner_DW.obj_d);

  // Terminate for MATLABSystem: '<S9>/Get Parameter3'
  matlabCodegenHandle_matlabCodeg(&joint_trajectory_planner_DW.obj_js);

  // Terminate for MATLABSystem: '<S9>/Get Parameter4'
  matlabCodegenHandle_matlabCodeg(&joint_trajectory_planner_DW.obj_h);

  // Terminate for MATLABSystem: '<S9>/Get Parameter5'
  matlabCodegenHandle_matlabCodeg(&joint_trajectory_planner_DW.obj_n);

  // Terminate for MATLABSystem: '<S2>/Get Parameter3'
  matlabCodegenHandle_matlabCodeg(&joint_trajectory_planner_DW.obj_e0);

  // Terminate for MATLABSystem: '<S2>/Get Parameter4'
  matlabCodegenHandle_matlabCodeg(&joint_trajectory_planner_DW.obj_gm);

  // Terminate for MATLABSystem: '<S2>/Get Parameter2'
  matlabCodegenHandle_matlabCodeg(&joint_trajectory_planner_DW.obj_o);

  // Terminate for MATLABSystem: '<S2>/Get Parameter1'
  matlabCodegenHandle_matlabCodeg(&joint_trajectory_planner_DW.obj_l);

  // Terminate for Atomic SubSystem: '<Root>/Publish'
  // Terminate for MATLABSystem: '<S6>/SinkBlock'
  joint_traject_matlabCodegenHa_a(&joint_trajectory_planner_DW.obj_dw0);

  // End of Terminate for SubSystem: '<Root>/Publish'
}

//
// File trailer for generated code.
//
// [EOF]
//
