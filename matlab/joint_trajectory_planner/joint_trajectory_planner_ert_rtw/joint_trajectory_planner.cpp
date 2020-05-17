//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: joint_trajectory_planner.cpp
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
static void addFlatSegmentsToPPFormParts_a(const real_T oldbreaks[2], const
  real_T oldCoeffs[24], real_T newBreaks[4], real_T newCoefs[72]);
static void joint_trajectory_plann_ppval_ao(const real_T pp_breaks[4], const
  real_T pp_coefs[72], const real_T x[2], real_T v[12]);
static void joint_trajectory__cubicpolytraj(const real_T wayPoints[12], const
  real_T timePoints[2], const real_T t[2], const real_T varargin_2[12], real_T
  q[12], real_T qd[12], real_T qdd[12], real_T pp_breaks[4], real_T pp_coefs[72]);
static void PolyTrajSys_computePPDerivative(const real_T pp_breaks[4], const
  real_T pp_coefs[72], real_T t, real_T ppd_breaks[4], real_T ppd_coefs[72],
  real_T ppdd_breaks[4], real_T ppdd_coefs[72]);
static c_robotics_core_internal_code_T *NameValueParser_NameValueParser
  (c_robotics_core_internal_code_T *obj);
static void joint_traj_computeProfileParams(real_T i, const real_T wayPoints[12],
  const real_T Vel[6], const real_T Acc_data[], real_T *vParam, real_T *aParam,
  real_T *tAParam, real_T *tFParam);
static void j_computeScalarLSPBCoefficients(real_T s0, real_T sF, real_T v,
  real_T a, real_T ta, real_T tf, real_T coefs[9], real_T breaks[4]);
static boolean_T join_checkPolyForMultipleBreaks(const real_T breakMat[24]);
static void joint__processPolynomialResults(const real_T breakMat[24], const
  real_T coeffMat[54], boolean_T hasMultipleBreaks,
  f_cell_wrap_joint_trajectory__T breaksCell[6], g_cell_wrap_joint_trajectory__T
  coeffCell[6]);
static void jo_addFlatSegmentsToPPFormParts(const real_T oldbreaks_data[], const
  int32_T oldbreaks_size[2], const real_T oldCoeffs_data[], const int32_T
  oldCoeffs_size[2], real_T dim, real_T newBreaks_data[], int32_T
  newBreaks_size[2], real_T newCoefs_data[], int32_T newCoefs_size[2]);
static void joint_traj_polyCoeffsDerivative(const real_T coeffs_data[], const
  int32_T coeffs_size[2], real_T dCoeffs_data[], int32_T dCoeffs_size[2]);
static int32_T joint_trajectory_planne_bsearch(const real_T x_data[], const
  int32_T x_size[2], real_T xi);
static void joint_trajectory_planner_ppval(const real_T pp_breaks_data[], const
  int32_T pp_breaks_size[2], const real_T pp_coefs_data[], const int32_T
  pp_coefs_size[3], const real_T x[2], real_T v_data[], int32_T v_size[2]);
static void j_generateTrajectoriesFromCoefs(const real_T breaks_data[], const
  int32_T breaks_size[2], const real_T coeffs_data[], const int32_T coeffs_size
  [2], real_T dim, const real_T t[2], real_T q_data[], int32_T q_size[2], real_T
  qd_data[], int32_T qd_size[2], real_T qdd_data[], int32_T qdd_size[2], real_T
  pp_breaks_data[], int32_T pp_breaks_size[2], real_T pp_coefs_data[], int32_T
  pp_coefs_size[3]);
static void joint_trajectory_pl_trapveltraj(const real_T wayPoints[12], const
  real_T varargin_2[6], real_T varargin_4, real_T q[12], real_T qd[12], real_T
  qdd[12], real_T t[2], s_06c2DDfmr4zcnTqhww20ZC_join_T ppCell_data[], int32_T
  *ppCell_size);
static void TrapVelTrajSys_extract1DimFromP(const real_T pp_breaks_data[], const
  int32_T pp_breaks_size[2], const real_T pp_coefs_data[], const int32_T
  pp_coefs_size[3], real_T breaks_data[], int32_T breaks_size[2], real_T
  oneDimCoeffs_data[], int32_T oneDimCoeffs_size[2]);
static void joint_trajectory_planne_ppval_a(const real_T pp_breaks_data[], const
  int32_T pp_breaks_size[2], const real_T pp_coefs_data[], real_T x, real_T
  v_data[], int32_T *v_size);
static void TrapVelTrajSys_generate1DTrajec(const real_T breaks_data[], const
  int32_T breaks_size[2], const real_T coefs_data[], const int32_T coefs_size[2],
  real_T t, real_T q_data[], int32_T *q_size, real_T qd_data[], int32_T *qd_size,
  real_T qdd_data[], int32_T *qdd_size);
static void TrapVelTrajSys_extract1DimFro_a(const real_T pp_breaks_data[], const
  int32_T pp_breaks_size[2], const real_T pp_coefs_data[], const int32_T
  pp_coefs_size[3], real_T breaks_data[], int32_T breaks_size[2], real_T
  oneDimCoeffs_data[], int32_T oneDimCoeffs_size[2]);
static void TrapVelTrajSys_extract1DimFr_ao(const real_T pp_breaks_data[], const
  int32_T pp_breaks_size[2], const real_T pp_coefs_data[], const int32_T
  pp_coefs_size[3], real_T breaks_data[], int32_T breaks_size[2], real_T
  oneDimCoeffs_data[], int32_T oneDimCoeffs_size[2]);
static void TrapVelTrajSys_extract1DimF_aoq(const real_T pp_breaks_data[], const
  int32_T pp_breaks_size[2], const real_T pp_coefs_data[], const int32_T
  pp_coefs_size[3], real_T breaks_data[], int32_T breaks_size[2], real_T
  oneDimCoeffs_data[], int32_T oneDimCoeffs_size[2]);
static void TrapVelTrajSys_extract1Dim_aoqo(const real_T pp_breaks_data[], const
  int32_T pp_breaks_size[2], const real_T pp_coefs_data[], const int32_T
  pp_coefs_size[3], real_T breaks_data[], int32_T breaks_size[2], real_T
  oneDimCoeffs_data[], int32_T oneDimCoeffs_size[2]);
static void TrapVelTrajSys_extract1Di_aoqov(const real_T pp_breaks_data[], const
  int32_T pp_breaks_size[2], const real_T pp_coefs_data[], const int32_T
  pp_coefs_size[3], real_T breaks_data[], int32_T breaks_size[2], real_T
  oneDimCoeffs_data[], int32_T oneDimCoeffs_size[2]);
static void joint_t_TrapVelTrajSys_stepImpl(real_T time, const real_T
  varargin_1[12], const real_T varargin_2[6], real_T varargin_3, real_T q[6],
  real_T qd[6], real_T qdd[6]);
static void joint_trajectory_plan_ppval_aoq(const real_T pp_breaks[4], const
  real_T pp_coefs[72], real_T x, real_T v[6]);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_GetP_T *obj);
static void joint_traject_matlabCodegenHa_n(ros_slros_internal_block_Subs_T *obj);
static void joint_traject_matlabCodegenHa_a(ros_slros_internal_block_Publ_T *obj);
static void joint_trajec_SystemCore_setup_a(robotics_slcore_internal_bl_a_T *obj);
static void joint_trajecto_SystemCore_setup(robotics_slcore_internal_bloc_T *obj);
int32_T div_nzp_s32(int32_T numerator, int32_T denominator)
{
  uint32_T tempAbsQuotient;
  tempAbsQuotient = (numerator < 0 ? ~static_cast<uint32_T>(numerator) + 1U :
                     static_cast<uint32_T>(numerator)) / (denominator < 0 ? ~
    static_cast<uint32_T>(denominator) + 1U : static_cast<uint32_T>(denominator));
  return (numerator < 0) != (denominator < 0) ? -static_cast<int32_T>
    (tempAbsQuotient) : static_cast<int32_T>(tempAbsQuotient);
}

int32_T div_nzp_s32_floor(int32_T numerator, int32_T denominator)
{
  uint32_T absNumerator;
  uint32_T absDenominator;
  uint32_T tempAbsQuotient;
  boolean_T quotientNeedsNegation;
  absNumerator = numerator < 0 ? ~static_cast<uint32_T>(numerator) + 1U :
    static_cast<uint32_T>(numerator);
  absDenominator = denominator < 0 ? ~static_cast<uint32_T>(denominator) + 1U :
    static_cast<uint32_T>(denominator);
  quotientNeedsNegation = ((numerator < 0) != (denominator < 0));
  tempAbsQuotient = absNumerator / absDenominator;
  if (quotientNeedsNegation) {
    absNumerator %= absDenominator;
    if (absNumerator > 0U) {
      tempAbsQuotient++;
    }
  }

  return quotientNeedsNegation ? -static_cast<int32_T>(tempAbsQuotient) :
    static_cast<int32_T>(tempAbsQuotient);
}

static real_T joint_trajectory_pl_rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    joint_trajectory_planner_B.d1 = fabs(u0);
    joint_trajectory_planner_B.d2 = fabs(u1);
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
    } else if (joint_trajectory_planner_B.d2 == 0.0) {
      y = 1.0;
    } else if (joint_trajectory_planner_B.d2 == 1.0) {
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

static void addFlatSegmentsToPPFormParts_a(const real_T oldbreaks[2], const
  real_T oldCoeffs[24], real_T newBreaks[4], real_T newCoefs[72])
{
  int32_T i;
  int32_T i_0;
  real_T evalPointVector_idx_0;
  real_T evalPointVector_idx_1;
  real_T evalPointVector_idx_2;
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

  joint_trajectory_planner_B.holdPoint_d = oldbreaks[1] - oldbreaks[0];
  evalPointVector_idx_0 = joint_trajectory_pl_rt_powd_snf
    (joint_trajectory_planner_B.holdPoint_d, 3.0);
  evalPointVector_idx_1 = joint_trajectory_pl_rt_powd_snf
    (joint_trajectory_planner_B.holdPoint_d, 2.0);
  evalPointVector_idx_2 = joint_trajectory_pl_rt_powd_snf
    (joint_trajectory_planner_B.holdPoint_d, 1.0);
  joint_trajectory_planner_B.holdPoint_d = joint_trajectory_pl_rt_powd_snf
    (joint_trajectory_planner_B.holdPoint_d, 0.0);
  memset(&joint_trajectory_planner_B.newSegmentCoeffs[0], 0, 24U * sizeof(real_T));
  for (i = 0; i < 6; i++) {
    joint_trajectory_planner_B.newSegmentCoeffs[i + 18] = 0.0;
    joint_trajectory_planner_B.newSegmentCoeffs[i + 18] +=
      joint_trajectory_planner_B.coefsWithFlatStart[i + 6] *
      evalPointVector_idx_0;
    joint_trajectory_planner_B.newSegmentCoeffs[i + 18] +=
      joint_trajectory_planner_B.coefsWithFlatStart[i + 18] *
      evalPointVector_idx_1;
    joint_trajectory_planner_B.newSegmentCoeffs[i + 18] +=
      joint_trajectory_planner_B.coefsWithFlatStart[i + 30] *
      evalPointVector_idx_2;
    joint_trajectory_planner_B.newSegmentCoeffs[i + 18] +=
      joint_trajectory_planner_B.coefsWithFlatStart[i + 42] *
      joint_trajectory_planner_B.holdPoint_d;
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

static void joint_trajectory_plann_ppval_ao(const real_T pp_breaks[4], const
  real_T pp_coefs[72], const real_T x[2], real_T v[12])
{
  int32_T iv0;
  int32_T j;
  real_T xloc;
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
      xloc = x[b_ix] - pp_breaks[low_i];
      for (low_i = 0; low_i < 6; low_i++) {
        v[(iv0 + low_i) + 1] = pp_coefs[low_ip1 + low_i];
      }

      for (low_i = 0; low_i < 3; low_i++) {
        high_i = ((low_i + 1) * 18 + low_ip1) - 1;
        for (mid_i = 0; mid_i < 6; mid_i++) {
          j = mid_i + 1;
          v_tmp = iv0 + j;
          v[v_tmp] = v[v_tmp] * xloc + pp_coefs[high_i + j];
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

  addFlatSegmentsToPPFormParts_a(timePoints, joint_trajectory_planner_B.coefMat,
    joint_trajectory_planner_B.modBreaks, joint_trajectory_planner_B.ddCoeffs);
  pp_breaks[0] = joint_trajectory_planner_B.modBreaks[0];
  pp_breaks[1] = joint_trajectory_planner_B.modBreaks[1];
  pp_breaks[2] = joint_trajectory_planner_B.modBreaks[2];
  pp_breaks[3] = joint_trajectory_planner_B.modBreaks[3];
  memcpy(&pp_coefs[0], &joint_trajectory_planner_B.ddCoeffs[0], 72U * sizeof
         (real_T));
  joint_trajectory_plann_ppval_ao(joint_trajectory_planner_B.modBreaks,
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

  joint_trajectory_plann_ppval_ao(joint_trajectory_planner_B.derivativeBreaks,
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

  joint_trajectory_plann_ppval_ao(joint_trajectory_planner_B.derivativeBreaks,
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

static c_robotics_core_internal_code_T *NameValueParser_NameValueParser
  (c_robotics_core_internal_code_T *obj)
{
  return obj;
}

static void joint_traj_computeProfileParams(real_T i, const real_T wayPoints[12],
  const real_T Vel[6], const real_T Acc_data[], real_T *vParam, real_T *aParam,
  real_T *tAParam, real_T *tFParam)
{
  real_T s0;
  real_T sF;
  int32_T deltaSign;
  int32_T s0_tmp;
  int32_T s0_tmp_0;
  s0_tmp = static_cast<int32_T>(i);
  s0_tmp_0 = s0_tmp - 1;
  s0 = wayPoints[s0_tmp_0];
  s0_tmp += 5;
  sF = wayPoints[s0_tmp];
  deltaSign = 1;
  if (wayPoints[s0_tmp] < wayPoints[s0_tmp_0]) {
    s0 = wayPoints[s0_tmp];
    sF = wayPoints[s0_tmp_0];
    deltaSign = -1;
  }

  *vParam = Vel[s0_tmp_0];
  *aParam = Acc_data[s0_tmp_0];
  *tAParam = Vel[s0_tmp_0] / Acc_data[s0_tmp_0];
  *tFParam = ((Vel[s0_tmp_0] * *tAParam + sF) - s0) / Vel[s0_tmp_0];
  if (s0 == sF) {
    *aParam = 0.0;
    *vParam = 0.0;
    if (rtIsNaN(*tFParam) || (*tFParam == 0.0)) {
      *tFParam = 1.0;
    }

    *tAParam = *tFParam / 3.0;
  }

  *vParam *= static_cast<real_T>(deltaSign);
  *aParam *= static_cast<real_T>(deltaSign);
}

static void j_computeScalarLSPBCoefficients(real_T s0, real_T sF, real_T v,
  real_T a, real_T ta, real_T tf, real_T coefs[9], real_T breaks[4])
{
  real_T coefs_tmp;
  breaks[0] = 0.0;
  breaks[1] = ta;
  breaks[2] = tf - ta;
  breaks[3] = tf;
  memset(&coefs[0], 0, 9U * sizeof(real_T));
  if (v == 0.0) {
    coefs[6] = s0;
    coefs[7] = s0;
    coefs[8] = s0;
  } else {
    coefs[0] = a / 2.0;
    coefs[3] = 0.0;
    coefs[6] = s0;
    coefs[1] = 0.0;
    coefs[4] = v;
    coefs_tmp = a / 2.0 * (ta * ta);
    coefs[7] = coefs_tmp + s0;
    coefs[2] = -a / 2.0;
    coefs[5] = v;
    coefs[8] = (coefs_tmp + sF) - v * ta;
  }
}

static boolean_T join_checkPolyForMultipleBreaks(const real_T breakMat[24])
{
  boolean_T hasMultipleBreaks;
  int32_T b_i;
  real_T y[4];
  boolean_T y_0;
  int32_T b_i_0;
  boolean_T exitg1;
  hasMultipleBreaks = false;
  for (b_i = 0; b_i < 5; b_i++) {
    b_i_0 = b_i + 1;
    y[0] = fabs(breakMat[b_i_0 - 1] - breakMat[b_i + 1]);
    y[1] = fabs(breakMat[b_i_0 + 5] - breakMat[b_i + 7]);
    y[2] = fabs(breakMat[b_i_0 + 11] - breakMat[b_i + 13]);
    y[3] = fabs(breakMat[b_i_0 + 17] - breakMat[b_i + 19]);
    y_0 = false;
    b_i_0 = 0;
    exitg1 = false;
    while ((!exitg1) && (b_i_0 < 4)) {
      if (!(y[b_i_0] > 2.2204460492503131E-16)) {
        b_i_0++;
      } else {
        y_0 = true;
        exitg1 = true;
      }
    }

    if (y_0 || hasMultipleBreaks) {
      hasMultipleBreaks = true;
    } else {
      hasMultipleBreaks = false;
    }
  }

  return hasMultipleBreaks;
}

static void joint__processPolynomialResults(const real_T breakMat[24], const
  real_T coeffMat[54], boolean_T hasMultipleBreaks,
  f_cell_wrap_joint_trajectory__T breaksCell[6], g_cell_wrap_joint_trajectory__T
  coeffCell[6])
{
  real_T maxBreaksTime;
  real_T holdTime;
  int32_T b_idx;
  int32_T k;
  int32_T i;
  int32_T coefs_data_tmp;
  boolean_T exitg1;
  if (!rtIsNaN(breakMat[18])) {
    b_idx = 1;
  } else {
    b_idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 7)) {
      if (!rtIsNaN(breakMat[k + 17])) {
        b_idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (b_idx == 0) {
    maxBreaksTime = breakMat[18];
  } else {
    maxBreaksTime = breakMat[b_idx + 17];
    for (k = b_idx + 1; k < 7; k++) {
      holdTime = breakMat[k + 17];
      if (maxBreaksTime < holdTime) {
        maxBreaksTime = holdTime;
      }
    }
  }

  if (hasMultipleBreaks) {
    if (!rtIsNaN(breakMat[0])) {
      b_idx = 1;
    } else {
      b_idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k < 5)) {
        if (!rtIsNaN(breakMat[(k - 1) * 6])) {
          b_idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (b_idx == 0) {
      holdTime = breakMat[0];
    } else {
      holdTime = breakMat[(b_idx - 1) * 6];
      for (k = b_idx + 1; k < 5; k++) {
        i = (k - 1) * 6;
        if (holdTime < breakMat[i]) {
          holdTime = breakMat[i];
        }
      }
    }

    if (holdTime < maxBreaksTime) {
      breaksCell[0].f1.size[0] = 1;
      breaksCell[0].f1.size[1] = 5;
      breaksCell[0].f1.data[0] = breakMat[0];
      breaksCell[0].f1.data[1] = breakMat[6];
      breaksCell[0].f1.data[2] = breakMat[12];
      breaksCell[0].f1.data[3] = breakMat[18];
      breaksCell[0].f1.data[4] = maxBreaksTime;
      holdTime = breakMat[18] - breakMat[12];
      k = 4;
      memset(&joint_trajectory_planner_B.coefs_data[0], 0, 12U * sizeof(real_T));
      joint_trajectory_planner_B.holdTime[0] = holdTime * holdTime;
      joint_trajectory_planner_B.holdTime[1] = holdTime;
      joint_trajectory_planner_B.holdTime[2] = 1.0;
      holdTime = 0.0;
      for (i = 0; i < 3; i++) {
        b_idx = i << 2;
        joint_trajectory_planner_B.coefs_data[b_idx] = coeffMat[18 * i];
        joint_trajectory_planner_B.coefs_data[b_idx + 1] = coeffMat[18 * i + 6];
        coefs_data_tmp = 18 * i + 12;
        joint_trajectory_planner_B.coefs_data[b_idx + 2] =
          coeffMat[coefs_data_tmp];
        holdTime += coeffMat[coefs_data_tmp] *
          joint_trajectory_planner_B.holdTime[i];
      }

      joint_trajectory_planner_B.coefs_data[11] = holdTime;
    } else {
      k = 3;
      for (i = 0; i < 3; i++) {
        joint_trajectory_planner_B.coefs_data[3 * i] = coeffMat[18 * i];
        joint_trajectory_planner_B.coefs_data[3 * i + 1] = coeffMat[18 * i + 6];
        joint_trajectory_planner_B.coefs_data[3 * i + 2] = coeffMat[18 * i + 12];
      }

      breaksCell[0].f1.size[0] = 1;
      breaksCell[0].f1.size[1] = 4;
      breaksCell[0].f1.data[0] = breakMat[0];
      breaksCell[0].f1.data[1] = breakMat[6];
      breaksCell[0].f1.data[2] = breakMat[12];
      breaksCell[0].f1.data[3] = breakMat[18];
    }

    coeffCell[0].f1.size[0] = k;
    coeffCell[0].f1.size[1] = 3;
    k = k * 3 - 1;
    for (i = 0; i <= k; i++) {
      coeffCell[0].f1.data[i] = joint_trajectory_planner_B.coefs_data[i];
    }

    if (!rtIsNaN(breakMat[1])) {
      b_idx = 1;
    } else {
      b_idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k < 5)) {
        if (!rtIsNaN(breakMat[(k - 1) * 6 + 1])) {
          b_idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (b_idx == 0) {
      holdTime = breakMat[1];
    } else {
      holdTime = breakMat[(b_idx - 1) * 6 + 1];
      for (k = b_idx + 1; k < 5; k++) {
        i = (k - 1) * 6 + 1;
        if (holdTime < breakMat[i]) {
          holdTime = breakMat[i];
        }
      }
    }

    if (holdTime < maxBreaksTime) {
      breaksCell[1].f1.size[0] = 1;
      breaksCell[1].f1.size[1] = 5;
      breaksCell[1].f1.data[0] = breakMat[1];
      breaksCell[1].f1.data[1] = breakMat[7];
      breaksCell[1].f1.data[2] = breakMat[13];
      breaksCell[1].f1.data[3] = breakMat[19];
      breaksCell[1].f1.data[4] = maxBreaksTime;
      holdTime = breakMat[19] - breakMat[13];
      k = 4;
      memset(&joint_trajectory_planner_B.coefs_data[0], 0, 12U * sizeof(real_T));
      joint_trajectory_planner_B.holdTime[0] = holdTime * holdTime;
      joint_trajectory_planner_B.holdTime[1] = holdTime;
      joint_trajectory_planner_B.holdTime[2] = 1.0;
      holdTime = 0.0;
      for (i = 0; i < 3; i++) {
        b_idx = i << 2;
        joint_trajectory_planner_B.coefs_data[b_idx] = coeffMat[18 * i + 1];
        joint_trajectory_planner_B.coefs_data[b_idx + 1] = coeffMat[18 * i + 7];
        coefs_data_tmp = 18 * i + 13;
        joint_trajectory_planner_B.coefs_data[b_idx + 2] =
          coeffMat[coefs_data_tmp];
        holdTime += coeffMat[coefs_data_tmp] *
          joint_trajectory_planner_B.holdTime[i];
      }

      joint_trajectory_planner_B.coefs_data[11] = holdTime;
    } else {
      k = 3;
      for (i = 0; i < 3; i++) {
        joint_trajectory_planner_B.coefs_data[3 * i] = coeffMat[18 * i + 1];
        joint_trajectory_planner_B.coefs_data[3 * i + 1] = coeffMat[18 * i + 7];
        joint_trajectory_planner_B.coefs_data[3 * i + 2] = coeffMat[18 * i + 13];
      }

      breaksCell[1].f1.size[0] = 1;
      breaksCell[1].f1.size[1] = 4;
      breaksCell[1].f1.data[0] = breakMat[1];
      breaksCell[1].f1.data[1] = breakMat[7];
      breaksCell[1].f1.data[2] = breakMat[13];
      breaksCell[1].f1.data[3] = breakMat[19];
    }

    coeffCell[1].f1.size[0] = k;
    coeffCell[1].f1.size[1] = 3;
    k = k * 3 - 1;
    for (i = 0; i <= k; i++) {
      coeffCell[1].f1.data[i] = joint_trajectory_planner_B.coefs_data[i];
    }

    if (!rtIsNaN(breakMat[2])) {
      b_idx = 1;
    } else {
      b_idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k < 5)) {
        if (!rtIsNaN(breakMat[(k - 1) * 6 + 2])) {
          b_idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (b_idx == 0) {
      holdTime = breakMat[2];
    } else {
      holdTime = breakMat[(b_idx - 1) * 6 + 2];
      for (k = b_idx + 1; k < 5; k++) {
        i = (k - 1) * 6 + 2;
        if (holdTime < breakMat[i]) {
          holdTime = breakMat[i];
        }
      }
    }

    if (holdTime < maxBreaksTime) {
      breaksCell[2].f1.size[0] = 1;
      breaksCell[2].f1.size[1] = 5;
      breaksCell[2].f1.data[0] = breakMat[2];
      breaksCell[2].f1.data[1] = breakMat[8];
      breaksCell[2].f1.data[2] = breakMat[14];
      breaksCell[2].f1.data[3] = breakMat[20];
      breaksCell[2].f1.data[4] = maxBreaksTime;
      holdTime = breakMat[20] - breakMat[14];
      k = 4;
      memset(&joint_trajectory_planner_B.coefs_data[0], 0, 12U * sizeof(real_T));
      joint_trajectory_planner_B.holdTime[0] = holdTime * holdTime;
      joint_trajectory_planner_B.holdTime[1] = holdTime;
      joint_trajectory_planner_B.holdTime[2] = 1.0;
      holdTime = 0.0;
      for (i = 0; i < 3; i++) {
        b_idx = i << 2;
        joint_trajectory_planner_B.coefs_data[b_idx] = coeffMat[18 * i + 2];
        joint_trajectory_planner_B.coefs_data[b_idx + 1] = coeffMat[18 * i + 8];
        coefs_data_tmp = 18 * i + 14;
        joint_trajectory_planner_B.coefs_data[b_idx + 2] =
          coeffMat[coefs_data_tmp];
        holdTime += coeffMat[coefs_data_tmp] *
          joint_trajectory_planner_B.holdTime[i];
      }

      joint_trajectory_planner_B.coefs_data[11] = holdTime;
    } else {
      k = 3;
      for (i = 0; i < 3; i++) {
        joint_trajectory_planner_B.coefs_data[3 * i] = coeffMat[18 * i + 2];
        joint_trajectory_planner_B.coefs_data[3 * i + 1] = coeffMat[18 * i + 8];
        joint_trajectory_planner_B.coefs_data[3 * i + 2] = coeffMat[18 * i + 14];
      }

      breaksCell[2].f1.size[0] = 1;
      breaksCell[2].f1.size[1] = 4;
      breaksCell[2].f1.data[0] = breakMat[2];
      breaksCell[2].f1.data[1] = breakMat[8];
      breaksCell[2].f1.data[2] = breakMat[14];
      breaksCell[2].f1.data[3] = breakMat[20];
    }

    coeffCell[2].f1.size[0] = k;
    coeffCell[2].f1.size[1] = 3;
    k = k * 3 - 1;
    for (i = 0; i <= k; i++) {
      coeffCell[2].f1.data[i] = joint_trajectory_planner_B.coefs_data[i];
    }

    if (!rtIsNaN(breakMat[3])) {
      b_idx = 1;
    } else {
      b_idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k < 5)) {
        if (!rtIsNaN(breakMat[(k - 1) * 6 + 3])) {
          b_idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (b_idx == 0) {
      holdTime = breakMat[3];
    } else {
      holdTime = breakMat[(b_idx - 1) * 6 + 3];
      for (k = b_idx + 1; k < 5; k++) {
        i = (k - 1) * 6 + 3;
        if (holdTime < breakMat[i]) {
          holdTime = breakMat[i];
        }
      }
    }

    if (holdTime < maxBreaksTime) {
      breaksCell[3].f1.size[0] = 1;
      breaksCell[3].f1.size[1] = 5;
      breaksCell[3].f1.data[0] = breakMat[3];
      breaksCell[3].f1.data[1] = breakMat[9];
      breaksCell[3].f1.data[2] = breakMat[15];
      breaksCell[3].f1.data[3] = breakMat[21];
      breaksCell[3].f1.data[4] = maxBreaksTime;
      holdTime = breakMat[21] - breakMat[15];
      k = 4;
      memset(&joint_trajectory_planner_B.coefs_data[0], 0, 12U * sizeof(real_T));
      joint_trajectory_planner_B.holdTime[0] = holdTime * holdTime;
      joint_trajectory_planner_B.holdTime[1] = holdTime;
      joint_trajectory_planner_B.holdTime[2] = 1.0;
      holdTime = 0.0;
      for (i = 0; i < 3; i++) {
        b_idx = i << 2;
        joint_trajectory_planner_B.coefs_data[b_idx] = coeffMat[18 * i + 3];
        joint_trajectory_planner_B.coefs_data[b_idx + 1] = coeffMat[18 * i + 9];
        coefs_data_tmp = 18 * i + 15;
        joint_trajectory_planner_B.coefs_data[b_idx + 2] =
          coeffMat[coefs_data_tmp];
        holdTime += coeffMat[coefs_data_tmp] *
          joint_trajectory_planner_B.holdTime[i];
      }

      joint_trajectory_planner_B.coefs_data[11] = holdTime;
    } else {
      k = 3;
      for (i = 0; i < 3; i++) {
        joint_trajectory_planner_B.coefs_data[3 * i] = coeffMat[18 * i + 3];
        joint_trajectory_planner_B.coefs_data[3 * i + 1] = coeffMat[18 * i + 9];
        joint_trajectory_planner_B.coefs_data[3 * i + 2] = coeffMat[18 * i + 15];
      }

      breaksCell[3].f1.size[0] = 1;
      breaksCell[3].f1.size[1] = 4;
      breaksCell[3].f1.data[0] = breakMat[3];
      breaksCell[3].f1.data[1] = breakMat[9];
      breaksCell[3].f1.data[2] = breakMat[15];
      breaksCell[3].f1.data[3] = breakMat[21];
    }

    coeffCell[3].f1.size[0] = k;
    coeffCell[3].f1.size[1] = 3;
    k = k * 3 - 1;
    for (i = 0; i <= k; i++) {
      coeffCell[3].f1.data[i] = joint_trajectory_planner_B.coefs_data[i];
    }

    if (!rtIsNaN(breakMat[4])) {
      b_idx = 1;
    } else {
      b_idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k < 5)) {
        if (!rtIsNaN(breakMat[(k - 1) * 6 + 4])) {
          b_idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (b_idx == 0) {
      holdTime = breakMat[4];
    } else {
      holdTime = breakMat[(b_idx - 1) * 6 + 4];
      for (k = b_idx + 1; k < 5; k++) {
        i = (k - 1) * 6 + 4;
        if (holdTime < breakMat[i]) {
          holdTime = breakMat[i];
        }
      }
    }

    if (holdTime < maxBreaksTime) {
      breaksCell[4].f1.size[0] = 1;
      breaksCell[4].f1.size[1] = 5;
      breaksCell[4].f1.data[0] = breakMat[4];
      breaksCell[4].f1.data[1] = breakMat[10];
      breaksCell[4].f1.data[2] = breakMat[16];
      breaksCell[4].f1.data[3] = breakMat[22];
      breaksCell[4].f1.data[4] = maxBreaksTime;
      holdTime = breakMat[22] - breakMat[16];
      k = 4;
      memset(&joint_trajectory_planner_B.coefs_data[0], 0, 12U * sizeof(real_T));
      joint_trajectory_planner_B.holdTime[0] = holdTime * holdTime;
      joint_trajectory_planner_B.holdTime[1] = holdTime;
      joint_trajectory_planner_B.holdTime[2] = 1.0;
      holdTime = 0.0;
      for (i = 0; i < 3; i++) {
        b_idx = i << 2;
        joint_trajectory_planner_B.coefs_data[b_idx] = coeffMat[18 * i + 4];
        joint_trajectory_planner_B.coefs_data[b_idx + 1] = coeffMat[18 * i + 10];
        coefs_data_tmp = 18 * i + 16;
        joint_trajectory_planner_B.coefs_data[b_idx + 2] =
          coeffMat[coefs_data_tmp];
        holdTime += coeffMat[coefs_data_tmp] *
          joint_trajectory_planner_B.holdTime[i];
      }

      joint_trajectory_planner_B.coefs_data[11] = holdTime;
    } else {
      k = 3;
      for (i = 0; i < 3; i++) {
        joint_trajectory_planner_B.coefs_data[3 * i] = coeffMat[18 * i + 4];
        joint_trajectory_planner_B.coefs_data[3 * i + 1] = coeffMat[18 * i + 10];
        joint_trajectory_planner_B.coefs_data[3 * i + 2] = coeffMat[18 * i + 16];
      }

      breaksCell[4].f1.size[0] = 1;
      breaksCell[4].f1.size[1] = 4;
      breaksCell[4].f1.data[0] = breakMat[4];
      breaksCell[4].f1.data[1] = breakMat[10];
      breaksCell[4].f1.data[2] = breakMat[16];
      breaksCell[4].f1.data[3] = breakMat[22];
    }

    coeffCell[4].f1.size[0] = k;
    coeffCell[4].f1.size[1] = 3;
    k = k * 3 - 1;
    for (i = 0; i <= k; i++) {
      coeffCell[4].f1.data[i] = joint_trajectory_planner_B.coefs_data[i];
    }

    if (!rtIsNaN(breakMat[5])) {
      b_idx = 1;
    } else {
      b_idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k < 5)) {
        if (!rtIsNaN(breakMat[(k - 1) * 6 + 5])) {
          b_idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (b_idx == 0) {
      holdTime = breakMat[5];
    } else {
      holdTime = breakMat[(b_idx - 1) * 6 + 5];
      for (k = b_idx + 1; k < 5; k++) {
        i = (k - 1) * 6 + 5;
        if (holdTime < breakMat[i]) {
          holdTime = breakMat[i];
        }
      }
    }

    if (holdTime < maxBreaksTime) {
      breaksCell[5].f1.size[0] = 1;
      breaksCell[5].f1.size[1] = 5;
      breaksCell[5].f1.data[0] = breakMat[5];
      breaksCell[5].f1.data[1] = breakMat[11];
      breaksCell[5].f1.data[2] = breakMat[17];
      breaksCell[5].f1.data[3] = breakMat[23];
      breaksCell[5].f1.data[4] = maxBreaksTime;
      holdTime = breakMat[23] - breakMat[17];
      k = 4;
      memset(&joint_trajectory_planner_B.coefs_data[0], 0, 12U * sizeof(real_T));
      joint_trajectory_planner_B.holdTime[0] = holdTime * holdTime;
      joint_trajectory_planner_B.holdTime[1] = holdTime;
      joint_trajectory_planner_B.holdTime[2] = 1.0;
      holdTime = 0.0;
      for (i = 0; i < 3; i++) {
        b_idx = i << 2;
        joint_trajectory_planner_B.coefs_data[b_idx] = coeffMat[18 * i + 5];
        joint_trajectory_planner_B.coefs_data[b_idx + 1] = coeffMat[18 * i + 11];
        coefs_data_tmp = 18 * i + 17;
        joint_trajectory_planner_B.coefs_data[b_idx + 2] =
          coeffMat[coefs_data_tmp];
        holdTime += coeffMat[coefs_data_tmp] *
          joint_trajectory_planner_B.holdTime[i];
      }

      joint_trajectory_planner_B.coefs_data[11] = holdTime;
    } else {
      k = 3;
      for (i = 0; i < 3; i++) {
        joint_trajectory_planner_B.coefs_data[3 * i] = coeffMat[18 * i + 5];
        joint_trajectory_planner_B.coefs_data[3 * i + 1] = coeffMat[18 * i + 11];
        joint_trajectory_planner_B.coefs_data[3 * i + 2] = coeffMat[18 * i + 17];
      }

      breaksCell[5].f1.size[0] = 1;
      breaksCell[5].f1.size[1] = 4;
      breaksCell[5].f1.data[0] = breakMat[5];
      breaksCell[5].f1.data[1] = breakMat[11];
      breaksCell[5].f1.data[2] = breakMat[17];
      breaksCell[5].f1.data[3] = breakMat[23];
    }

    coeffCell[5].f1.size[0] = k;
    coeffCell[5].f1.size[1] = 3;
    k = k * 3 - 1;
    for (i = 0; i <= k; i++) {
      coeffCell[5].f1.data[i] = joint_trajectory_planner_B.coefs_data[i];
    }
  } else {
    breaksCell[0].f1.size[0] = 1;
    breaksCell[0].f1.size[1] = 4;
    breaksCell[0].f1.data[0] = breakMat[0];
    breaksCell[0].f1.data[1] = breakMat[6];
    breaksCell[0].f1.data[2] = breakMat[12];
    breaksCell[0].f1.data[3] = breakMat[18];
    coeffCell[0].f1.size[0] = 18;
    coeffCell[0].f1.size[1] = 3;
    breaksCell[1].f1.size[0] = 1;
    breaksCell[1].f1.size[1] = 4;
    breaksCell[1].f1.data[0] = breakMat[0];
    breaksCell[1].f1.data[1] = breakMat[6];
    breaksCell[1].f1.data[2] = breakMat[12];
    breaksCell[1].f1.data[3] = breakMat[18];
    coeffCell[1].f1.size[0] = 18;
    coeffCell[1].f1.size[1] = 3;
    breaksCell[2].f1.size[0] = 1;
    breaksCell[2].f1.size[1] = 4;
    breaksCell[2].f1.data[0] = breakMat[0];
    breaksCell[2].f1.data[1] = breakMat[6];
    breaksCell[2].f1.data[2] = breakMat[12];
    breaksCell[2].f1.data[3] = breakMat[18];
    coeffCell[2].f1.size[0] = 18;
    coeffCell[2].f1.size[1] = 3;
    breaksCell[3].f1.size[0] = 1;
    breaksCell[3].f1.size[1] = 4;
    breaksCell[3].f1.data[0] = breakMat[0];
    breaksCell[3].f1.data[1] = breakMat[6];
    breaksCell[3].f1.data[2] = breakMat[12];
    breaksCell[3].f1.data[3] = breakMat[18];
    coeffCell[3].f1.size[0] = 18;
    coeffCell[3].f1.size[1] = 3;
    breaksCell[4].f1.size[0] = 1;
    breaksCell[4].f1.size[1] = 4;
    breaksCell[4].f1.data[0] = breakMat[0];
    breaksCell[4].f1.data[1] = breakMat[6];
    breaksCell[4].f1.data[2] = breakMat[12];
    breaksCell[4].f1.data[3] = breakMat[18];
    coeffCell[4].f1.size[0] = 18;
    coeffCell[4].f1.size[1] = 3;
    breaksCell[5].f1.size[0] = 1;
    breaksCell[5].f1.size[1] = 4;
    breaksCell[5].f1.data[0] = breakMat[0];
    breaksCell[5].f1.data[1] = breakMat[6];
    breaksCell[5].f1.data[2] = breakMat[12];
    breaksCell[5].f1.data[3] = breakMat[18];
    coeffCell[5].f1.size[0] = 18;
    coeffCell[5].f1.size[1] = 3;
    memcpy(&coeffCell[0].f1.data[0], &coeffMat[0], 54U * sizeof(real_T));
    memcpy(&coeffCell[1].f1.data[0], &coeffMat[0], 54U * sizeof(real_T));
    memcpy(&coeffCell[2].f1.data[0], &coeffMat[0], 54U * sizeof(real_T));
    memcpy(&coeffCell[3].f1.data[0], &coeffMat[0], 54U * sizeof(real_T));
    memcpy(&coeffCell[4].f1.data[0], &coeffMat[0], 54U * sizeof(real_T));
    memcpy(&coeffCell[5].f1.data[0], &coeffMat[0], 54U * sizeof(real_T));
  }
}

static void jo_addFlatSegmentsToPPFormParts(const real_T oldbreaks_data[], const
  int32_T oldbreaks_size[2], const real_T oldCoeffs_data[], const int32_T
  oldCoeffs_size[2], real_T dim, real_T newBreaks_data[], int32_T
  newBreaks_size[2], real_T newCoefs_data[], int32_T newCoefs_size[2])
{
  int32_T h;
  int32_T d;
  int32_T m;
  int32_T b_i;
  int32_T loop_ub;
  int32_T m_tmp;
  int32_T coefsWithFlatStart_size_idx_0_t;
  int32_T m_tmp_0;
  int32_T loop_ub_tmp;
  m_tmp = static_cast<int32_T>(dim);
  m = m_tmp - 1;
  for (b_i = 0; b_i <= m; b_i++) {
    d = m_tmp + b_i;
    joint_trajectory_planner_B.s = oldCoeffs_data[b_i % m_tmp + oldCoeffs_size[0]
      * div_nzp_s32_floor(b_i, m_tmp)] * 0.0 + oldCoeffs_data[d % m_tmp +
      oldCoeffs_size[0] * div_nzp_s32_floor(d, m_tmp)] * 0.0;
    d = (m_tmp << 1) + b_i;
    joint_trajectory_planner_B.valueAtStart_data[b_i] = oldCoeffs_data[d % m_tmp
      + oldCoeffs_size[0] * div_nzp_s32_floor(d, m_tmp)] +
      joint_trajectory_planner_B.s;
  }

  loop_ub_tmp = m_tmp * 3 - 1;
  if (0 <= loop_ub_tmp) {
    memset(&joint_trajectory_planner_B.newSegmentCoeffs_data[0], 0, (loop_ub_tmp
            + 1) * sizeof(real_T));
  }

  for (d = 0; d < m_tmp; d++) {
    joint_trajectory_planner_B.newSegmentCoeffs_data[d + (m_tmp << 1)] =
      joint_trajectory_planner_B.valueAtStart_data[d];
  }

  joint_trajectory_planner_B.coefsWithFlatStart_size_idx_0_t =
    static_cast<real_T>(oldCoeffs_size[0]) + dim;
  coefsWithFlatStart_size_idx_0_t = static_cast<int32_T>
    (joint_trajectory_planner_B.coefsWithFlatStart_size_idx_0_t);
  loop_ub = coefsWithFlatStart_size_idx_0_t * 3 - 1;
  if (0 <= loop_ub) {
    memset(&joint_trajectory_planner_B.coefsWithFlatStart_data[0], 0, (loop_ub +
            1) * sizeof(real_T));
  }

  for (d = 0; d < 3; d++) {
    for (b_i = 0; b_i < m_tmp; b_i++) {
      joint_trajectory_planner_B.coefsWithFlatStart_data[b_i +
        coefsWithFlatStart_size_idx_0_t * d] =
        joint_trajectory_planner_B.newSegmentCoeffs_data[m_tmp * d + b_i];
    }
  }

  if (dim + 1.0 > joint_trajectory_planner_B.coefsWithFlatStart_size_idx_0_t) {
    h = 0;
  } else {
    h = static_cast<int32_T>(dim + 1.0) - 1;
  }

  loop_ub = oldCoeffs_size[0];
  for (d = 0; d < 3; d++) {
    for (b_i = 0; b_i < loop_ub; b_i++) {
      joint_trajectory_planner_B.coefsWithFlatStart_data[(h + b_i) +
        coefsWithFlatStart_size_idx_0_t * d] = oldCoeffs_data[oldCoeffs_size[0] *
        d + b_i];
    }
  }

  h = oldbreaks_size[1] + 1;
  loop_ub = oldbreaks_size[0] * oldbreaks_size[1] - 1;
  joint_trajectory_planner_B.valueAtStart_data[0] = oldbreaks_data[0] - 1.0;
  if (0 <= loop_ub) {
    memcpy(&joint_trajectory_planner_B.valueAtStart_data[1], &oldbreaks_data[0],
           (loop_ub + 1) * sizeof(real_T));
  }

  joint_trajectory_planner_B.coefsWithFlatStart_size_idx_0_t =
    joint_trajectory_planner_B.valueAtStart_data[h - 1];
  joint_trajectory_planner_B.holdPoint =
    joint_trajectory_planner_B.coefsWithFlatStart_size_idx_0_t -
    joint_trajectory_planner_B.valueAtStart_data[h - 2];
  joint_trajectory_planner_B.B_idx_0 = joint_trajectory_pl_rt_powd_snf
    (joint_trajectory_planner_B.holdPoint, 2.0);
  joint_trajectory_planner_B.B_idx_1 = joint_trajectory_pl_rt_powd_snf
    (joint_trajectory_planner_B.holdPoint, 1.0);
  joint_trajectory_planner_B.holdPoint = joint_trajectory_pl_rt_powd_snf
    (joint_trajectory_planner_B.holdPoint, 0.0);
  joint_trajectory_planner_B.s = (static_cast<real_T>
    (coefsWithFlatStart_size_idx_0_t) - dim) + 1.0;
  if (joint_trajectory_planner_B.s > coefsWithFlatStart_size_idx_0_t) {
    loop_ub = 0;
    d = 0;
  } else {
    loop_ub = static_cast<int32_T>(joint_trajectory_planner_B.s) - 1;
    d = coefsWithFlatStart_size_idx_0_t;
  }

  m_tmp_0 = d - loop_ub;
  m = m_tmp_0 - 1;
  for (b_i = 0; b_i <= m; b_i++) {
    d = m_tmp_0 + b_i;
    joint_trajectory_planner_B.s =
      joint_trajectory_planner_B.coefsWithFlatStart_data[(b_i % m_tmp_0 +
      loop_ub) + div_nzp_s32_floor(b_i, m_tmp_0) *
      coefsWithFlatStart_size_idx_0_t] * joint_trajectory_planner_B.B_idx_0 +
      joint_trajectory_planner_B.coefsWithFlatStart_data[(d % m_tmp_0 + loop_ub)
      + div_nzp_s32_floor(d, m_tmp_0) * coefsWithFlatStart_size_idx_0_t] *
      joint_trajectory_planner_B.B_idx_1;
    d = (m_tmp_0 << 1) + b_i;
    joint_trajectory_planner_B.valueAtEnd_data[b_i] =
      joint_trajectory_planner_B.coefsWithFlatStart_data[(d % m_tmp_0 + loop_ub)
      + div_nzp_s32_floor(d, m_tmp_0) * coefsWithFlatStart_size_idx_0_t] *
      joint_trajectory_planner_B.holdPoint + joint_trajectory_planner_B.s;
  }

  if (0 <= loop_ub_tmp) {
    memset(&joint_trajectory_planner_B.newSegmentCoeffs_data[0], 0, (loop_ub_tmp
            + 1) * sizeof(real_T));
  }

  for (d = 0; d < m_tmp_0; d++) {
    joint_trajectory_planner_B.newSegmentCoeffs_data[d + (m_tmp << 1)] =
      joint_trajectory_planner_B.valueAtEnd_data[d];
  }

  loop_ub_tmp = static_cast<int32_T>(static_cast<real_T>
    (coefsWithFlatStart_size_idx_0_t) + dim);
  newCoefs_size[0] = loop_ub_tmp;
  newCoefs_size[1] = 3;
  loop_ub = loop_ub_tmp * 3 - 1;
  if (0 <= loop_ub) {
    memset(&newCoefs_data[0], 0, (loop_ub + 1) * sizeof(real_T));
  }

  for (d = 0; d < 3; d++) {
    for (b_i = 0; b_i < coefsWithFlatStart_size_idx_0_t; b_i++) {
      newCoefs_data[b_i + loop_ub_tmp * d] =
        joint_trajectory_planner_B.coefsWithFlatStart_data[coefsWithFlatStart_size_idx_0_t
        * d + b_i];
    }
  }

  for (d = 0; d < 3; d++) {
    for (b_i = 0; b_i < m_tmp; b_i++) {
      newCoefs_data[((coefsWithFlatStart_size_idx_0_t + (b_i + 1)) + loop_ub_tmp
                     * d) - 1] =
        joint_trajectory_planner_B.newSegmentCoeffs_data[m_tmp * d + b_i];
    }
  }

  newBreaks_size[0] = 1;
  newBreaks_size[1] = h + 1;
  if (0 <= h - 1) {
    memcpy(&newBreaks_data[0], &joint_trajectory_planner_B.valueAtStart_data[0],
           h * sizeof(real_T));
  }

  newBreaks_data[h] = joint_trajectory_planner_B.coefsWithFlatStart_size_idx_0_t
    + 1.0;
}

static void joint_traj_polyCoeffsDerivative(const real_T coeffs_data[], const
  int32_T coeffs_size[2], real_T dCoeffs_data[], int32_T dCoeffs_size[2])
{
  int32_T loop_ub;
  int32_T i;
  int32_T b_idx_0;
  b_idx_0 = coeffs_size[0];
  dCoeffs_size[0] = coeffs_size[0];
  dCoeffs_size[1] = 3;
  loop_ub = coeffs_size[0] * 3 - 1;
  if (0 <= loop_ub) {
    memset(&dCoeffs_data[0], 0, (loop_ub + 1) * sizeof(real_T));
  }

  loop_ub = coeffs_size[0];
  for (i = 0; i < loop_ub; i++) {
    dCoeffs_data[i + b_idx_0] = 2.0 * coeffs_data[i];
  }

  loop_ub = coeffs_size[0];
  for (i = 0; i < loop_ub; i++) {
    dCoeffs_data[i + (b_idx_0 << 1)] = coeffs_data[i + coeffs_size[0]];
  }
}

static int32_T joint_trajectory_planne_bsearch(const real_T x_data[], const
  int32_T x_size[2], real_T xi)
{
  int32_T n;
  int32_T low_ip1;
  int32_T high_i;
  int32_T mid_i;
  n = 1;
  low_ip1 = 1;
  high_i = x_size[1];
  while (high_i > low_ip1 + 1) {
    mid_i = (n >> 1) + (high_i >> 1);
    if (((n & 1) == 1) && ((high_i & 1) == 1)) {
      mid_i++;
    }

    if (xi >= x_data[mid_i - 1]) {
      n = mid_i;
      low_ip1 = mid_i;
    } else {
      high_i = mid_i;
    }
  }

  return n;
}

static void joint_trajectory_planner_ppval(const real_T pp_breaks_data[], const
  int32_T pp_breaks_size[2], const real_T pp_coefs_data[], const int32_T
  pp_coefs_size[3], const real_T x[2], real_T v_data[], int32_T v_size[2])
{
  int32_T elementsPerPage;
  int32_T coefStride;
  int32_T j;
  int32_T ip;
  int32_T icp;
  int32_T ic0;
  int32_T v_data_tmp;
  elementsPerPage = pp_coefs_size[0] - 1;
  coefStride = (pp_breaks_size[1] - 1) * pp_coefs_size[0];
  v_size[0] = pp_coefs_size[0];
  v_size[1] = 2;
  if (pp_coefs_size[0] == 1) {
    if (rtIsNaN(x[0])) {
      v_data[0] = x[0];
    } else {
      ip = joint_trajectory_planne_bsearch(pp_breaks_data, pp_breaks_size, x[0])
        - 1;
      joint_trajectory_planner_B.xloc = x[0] - pp_breaks_data[ip];
      v_data[0] = (joint_trajectory_planner_B.xloc * pp_coefs_data[ip] +
                   pp_coefs_data[ip + coefStride]) *
        joint_trajectory_planner_B.xloc + pp_coefs_data[(coefStride << 1) + ip];
    }

    if (rtIsNaN(x[1])) {
      v_data[1] = x[1];
    } else {
      ip = joint_trajectory_planne_bsearch(pp_breaks_data, pp_breaks_size, x[1])
        - 1;
      joint_trajectory_planner_B.xloc = x[1] - pp_breaks_data[ip];
      v_data[1] = (joint_trajectory_planner_B.xloc * pp_coefs_data[ip] +
                   pp_coefs_data[ip + coefStride]) *
        joint_trajectory_planner_B.xloc + pp_coefs_data[(coefStride << 1) + ip];
    }
  } else {
    if (rtIsNaN(x[0])) {
      for (ip = 0; ip <= elementsPerPage; ip++) {
        v_data[ip] = x[0];
      }
    } else {
      ip = joint_trajectory_planne_bsearch(pp_breaks_data, pp_breaks_size, x[0])
        - 1;
      icp = (elementsPerPage + 1) * ip;
      joint_trajectory_planner_B.xloc = x[0] - pp_breaks_data[ip];
      ic0 = (icp + coefStride) - 1;
      for (ip = 0; ip <= elementsPerPage; ip++) {
        v_data[ip] = pp_coefs_data[icp + ip];
        j = ip + 1;
        v_data[-1 + j] = v_data[-1 + j] * joint_trajectory_planner_B.xloc +
          pp_coefs_data[ic0 + j];
      }

      ic0 = ((coefStride << 1) + icp) - 1;
      for (ip = 0; ip <= elementsPerPage; ip++) {
        j = ip + 1;
        v_data[-1 + j] = v_data[-1 + j] * joint_trajectory_planner_B.xloc +
          pp_coefs_data[ic0 + j];
      }
    }

    if (rtIsNaN(x[1])) {
      for (ip = 0; ip <= elementsPerPage; ip++) {
        v_data[(elementsPerPage + ip) + 1] = x[1];
      }
    } else {
      ip = joint_trajectory_planne_bsearch(pp_breaks_data, pp_breaks_size, x[1])
        - 1;
      icp = (elementsPerPage + 1) * ip;
      joint_trajectory_planner_B.xloc = x[1] - pp_breaks_data[ip];
      ic0 = (icp + coefStride) - 1;
      for (ip = 0; ip <= elementsPerPage; ip++) {
        v_data[(elementsPerPage + ip) + 1] = pp_coefs_data[icp + ip];
        j = ip + 1;
        v_data_tmp = elementsPerPage + j;
        v_data[v_data_tmp] = v_data[v_data_tmp] *
          joint_trajectory_planner_B.xloc + pp_coefs_data[ic0 + j];
      }

      ic0 = ((coefStride << 1) + icp) - 1;
      for (ip = 0; ip <= elementsPerPage; ip++) {
        j = ip + 1;
        v_data_tmp = elementsPerPage + j;
        v_data[v_data_tmp] = v_data[v_data_tmp] *
          joint_trajectory_planner_B.xloc + pp_coefs_data[ic0 + j];
      }
    }
  }
}

static void j_generateTrajectoriesFromCoefs(const real_T breaks_data[], const
  int32_T breaks_size[2], const real_T coeffs_data[], const int32_T coeffs_size
  [2], real_T dim, const real_T t[2], real_T q_data[], int32_T q_size[2], real_T
  qd_data[], int32_T qd_size[2], real_T qdd_data[], int32_T qdd_size[2], real_T
  pp_breaks_data[], int32_T pp_breaks_size[2], real_T pp_coefs_data[], int32_T
  pp_coefs_size[3])
{
  int32_T loop_ub;
  int32_T num_idx_1;
  int32_T num_idx_0_tmp;
  jo_addFlatSegmentsToPPFormParts(breaks_data, breaks_size, coeffs_data,
    coeffs_size, dim, joint_trajectory_planner_B.modBreaks_data,
    joint_trajectory_planner_B.modBreaks_size,
    joint_trajectory_planner_B.modCoeffs_data,
    joint_trajectory_planner_B.modCoeffs_size);
  joint_traj_polyCoeffsDerivative(joint_trajectory_planner_B.modCoeffs_data,
    joint_trajectory_planner_B.modCoeffs_size,
    joint_trajectory_planner_B.tmp_data, joint_trajectory_planner_B.tmp_size);
  joint_trajectory_planner_B.dCoeffs_size[0] =
    joint_trajectory_planner_B.tmp_size[0];
  joint_trajectory_planner_B.dCoeffs_size[1] = 3;
  loop_ub = joint_trajectory_planner_B.tmp_size[0] *
    joint_trajectory_planner_B.tmp_size[1];
  if (0 <= loop_ub - 1) {
    memcpy(&joint_trajectory_planner_B.dCoeffs_data[0],
           &joint_trajectory_planner_B.tmp_data[0], loop_ub * sizeof(real_T));
  }

  num_idx_0_tmp = static_cast<int32_T>(dim);
  num_idx_1 = joint_trajectory_planner_B.modBreaks_size[1] - 1;
  pp_breaks_size[0] = 1;
  pp_breaks_size[1] = joint_trajectory_planner_B.modBreaks_size[1];
  loop_ub = joint_trajectory_planner_B.modBreaks_size[1];
  if (0 <= loop_ub - 1) {
    memcpy(&pp_breaks_data[0], &joint_trajectory_planner_B.modBreaks_data[0],
           loop_ub * sizeof(real_T));
  }

  pp_coefs_size[0] = num_idx_0_tmp;
  pp_coefs_size[1] = num_idx_1;
  pp_coefs_size[2] = 3;
  loop_ub = num_idx_0_tmp * num_idx_1 * 3 - 1;
  if (0 <= loop_ub) {
    memcpy(&pp_coefs_data[0], &joint_trajectory_planner_B.modCoeffs_data[0],
           (loop_ub + 1) * sizeof(real_T));
  }

  joint_trajectory_planner_B.num[0] = num_idx_0_tmp;
  joint_trajectory_planner_B.num[1] = num_idx_1;
  joint_trajectory_planner_B.num[2] = 3;
  joint_trajectory_planner_B.modBreaks_size_m[0] = 1;
  joint_trajectory_planner_B.modBreaks_size_m[1] =
    joint_trajectory_planner_B.modBreaks_size[1];
  loop_ub = joint_trajectory_planner_B.modBreaks_size[1];
  if (0 <= loop_ub - 1) {
    memcpy(&joint_trajectory_planner_B.modBreaks_data_f[0],
           &joint_trajectory_planner_B.modBreaks_data[0], loop_ub * sizeof
           (real_T));
  }

  joint_trajectory_planner_ppval(joint_trajectory_planner_B.modBreaks_data_f,
    joint_trajectory_planner_B.modBreaks_size_m,
    joint_trajectory_planner_B.modCoeffs_data, joint_trajectory_planner_B.num, t,
    q_data, q_size);
  joint_trajectory_planner_B.num[0] = num_idx_0_tmp;
  joint_trajectory_planner_B.num[1] = joint_trajectory_planner_B.modBreaks_size
    [1] - 1;
  joint_trajectory_planner_B.num[2] = 3;
  joint_trajectory_planner_B.modBreaks_size_n[0] = 1;
  joint_trajectory_planner_B.modBreaks_size_n[1] =
    joint_trajectory_planner_B.modBreaks_size[1];
  loop_ub = joint_trajectory_planner_B.modBreaks_size[1];
  if (0 <= loop_ub - 1) {
    memcpy(&joint_trajectory_planner_B.modBreaks_data_f[0],
           &joint_trajectory_planner_B.modBreaks_data[0], loop_ub * sizeof
           (real_T));
  }

  joint_trajectory_planner_ppval(joint_trajectory_planner_B.modBreaks_data_f,
    joint_trajectory_planner_B.modBreaks_size_n,
    joint_trajectory_planner_B.dCoeffs_data, joint_trajectory_planner_B.num, t,
    qd_data, qd_size);
  joint_traj_polyCoeffsDerivative(joint_trajectory_planner_B.dCoeffs_data,
    joint_trajectory_planner_B.dCoeffs_size, joint_trajectory_planner_B.tmp_data,
    joint_trajectory_planner_B.tmp_size);
  joint_trajectory_planner_B.num[0] = num_idx_0_tmp;
  joint_trajectory_planner_B.num[1] = joint_trajectory_planner_B.modBreaks_size
    [1] - 1;
  joint_trajectory_planner_B.num[2] = 3;
  joint_trajectory_planner_B.modBreaks_size_p[0] = 1;
  joint_trajectory_planner_B.modBreaks_size_p[1] =
    joint_trajectory_planner_B.modBreaks_size[1];
  loop_ub = joint_trajectory_planner_B.modBreaks_size[1];
  if (0 <= loop_ub - 1) {
    memcpy(&joint_trajectory_planner_B.modBreaks_data_f[0],
           &joint_trajectory_planner_B.modBreaks_data[0], loop_ub * sizeof
           (real_T));
  }

  joint_trajectory_planner_ppval(joint_trajectory_planner_B.modBreaks_data_f,
    joint_trajectory_planner_B.modBreaks_size_p,
    joint_trajectory_planner_B.tmp_data, joint_trajectory_planner_B.num, t,
    qdd_data, qdd_size);
}

static void joint_trajectory_pl_trapveltraj(const real_T wayPoints[12], const
  real_T varargin_2[6], real_T varargin_4, real_T q[12], real_T qd[12], real_T
  qdd[12], real_T t[2], s_06c2DDfmr4zcnTqhww20ZC_join_T ppCell_data[], int32_T
  *ppCell_size)
{
  boolean_T hasMultipleBreaks;
  int32_T lspbSegIndices_size_idx_1;
  int32_T lspbSegIndices_size_idx_1_tmp;
  boolean_T exitg1;
  NameValueParser_NameValueParser(&joint_trajectory_planner_B.parser);
  for (joint_trajectory_planner_B.i_n = 0; joint_trajectory_planner_B.i_n < 6;
       joint_trajectory_planner_B.i_n++) {
    joint_trajectory_planner_B.parser.ParsedResults.f1[joint_trajectory_planner_B.i_n]
      = varargin_2[joint_trajectory_planner_B.i_n];
  }

  joint_trajectory_planner_B.parser.ParsedResults.f2 = varargin_4;
  for (joint_trajectory_planner_B.i_n = 0; joint_trajectory_planner_B.i_n < 6;
       joint_trajectory_planner_B.i_n++) {
    joint_trajectory_planner_B.a[joint_trajectory_planner_B.i_n] =
      joint_trajectory_planner_B.parser.ParsedResults.f1[joint_trajectory_planner_B.i_n];
  }

  joint_trajectory_planner_B.acc =
    joint_trajectory_planner_B.parser.ParsedResults.f2;
  for (joint_trajectory_planner_B.i_n = 0; joint_trajectory_planner_B.i_n < 6;
       joint_trajectory_planner_B.i_n++) {
    joint_trajectory_planner_B.vel[joint_trajectory_planner_B.i_n] =
      joint_trajectory_planner_B.a[joint_trajectory_planner_B.i_n];
    joint_trajectory_planner_B.matrixInput_data[joint_trajectory_planner_B.i_n] =
      joint_trajectory_planner_B.acc;
  }

  memset(&q[0], 0, 12U * sizeof(real_T));
  memset(&qd[0], 0, 12U * sizeof(real_T));
  memset(&qdd[0], 0, 12U * sizeof(real_T));
  memset(&joint_trajectory_planner_B.coeffMat[0], 0, 54U * sizeof(real_T));
  memset(&joint_trajectory_planner_B.breakMat[0], 0, 24U * sizeof(real_T));
  for (joint_trajectory_planner_B.b_idx = 0; joint_trajectory_planner_B.b_idx <
       6; joint_trajectory_planner_B.b_idx++) {
    joint_traj_computeProfileParams(static_cast<real_T>
      (joint_trajectory_planner_B.b_idx) + 1.0, wayPoints,
      joint_trajectory_planner_B.vel,
      joint_trajectory_planner_B.matrixInput_data,
      &joint_trajectory_planner_B.acc, &joint_trajectory_planner_B.segAcc,
      &joint_trajectory_planner_B.segATime, &joint_trajectory_planner_B.segFTime);
    joint_trajectory_planner_B.parameterMat[joint_trajectory_planner_B.b_idx] =
      wayPoints[joint_trajectory_planner_B.b_idx];
    joint_trajectory_planner_B.parameterMat[joint_trajectory_planner_B.b_idx + 6]
      = wayPoints[joint_trajectory_planner_B.b_idx + 6];
    joint_trajectory_planner_B.parameterMat[joint_trajectory_planner_B.b_idx +
      12] = joint_trajectory_planner_B.acc;
    joint_trajectory_planner_B.parameterMat[joint_trajectory_planner_B.b_idx +
      18] = joint_trajectory_planner_B.segAcc;
    joint_trajectory_planner_B.parameterMat[joint_trajectory_planner_B.b_idx +
      24] = joint_trajectory_planner_B.segATime;
    joint_trajectory_planner_B.parameterMat[joint_trajectory_planner_B.b_idx +
      30] = joint_trajectory_planner_B.segFTime;
    j_computeScalarLSPBCoefficients(wayPoints[joint_trajectory_planner_B.b_idx],
      wayPoints[joint_trajectory_planner_B.b_idx + 6],
      joint_trajectory_planner_B.acc, joint_trajectory_planner_B.segAcc,
      joint_trajectory_planner_B.segATime, joint_trajectory_planner_B.segFTime,
      joint_trajectory_planner_B.coefs, joint_trajectory_planner_B.breaks);
    for (joint_trajectory_planner_B.i_n = 0; joint_trajectory_planner_B.i_n < 18;
         joint_trajectory_planner_B.i_n++) {
      joint_trajectory_planner_B.coefIndex[joint_trajectory_planner_B.i_n] =
        false;
    }

    joint_trajectory_planner_B.i_n = joint_trajectory_planner_B.b_idx + 1;
    lspbSegIndices_size_idx_1_tmp = static_cast<int32_T>(floor(static_cast<
      real_T>((joint_trajectory_planner_B.b_idx - joint_trajectory_planner_B.i_n)
              + 13) / 6.0));
    lspbSegIndices_size_idx_1 = lspbSegIndices_size_idx_1_tmp + 1;
    for (joint_trajectory_planner_B.indivPolyDim = 0;
         joint_trajectory_planner_B.indivPolyDim <=
         lspbSegIndices_size_idx_1_tmp; joint_trajectory_planner_B.indivPolyDim
         ++) {
      joint_trajectory_planner_B.lspbSegIndices_data[joint_trajectory_planner_B.indivPolyDim]
        = 6 * joint_trajectory_planner_B.indivPolyDim +
        joint_trajectory_planner_B.i_n;
    }

    joint_trajectory_planner_B.indivPolyDim = lspbSegIndices_size_idx_1 - 1;
    for (joint_trajectory_planner_B.i_n = 0; joint_trajectory_planner_B.i_n <=
         joint_trajectory_planner_B.indivPolyDim; joint_trajectory_planner_B.i_n
         ++) {
      joint_trajectory_planner_B.g_data[joint_trajectory_planner_B.i_n] =
        static_cast<int32_T>
        (joint_trajectory_planner_B.lspbSegIndices_data[joint_trajectory_planner_B.i_n]);
    }

    for (joint_trajectory_planner_B.i_n = 0; joint_trajectory_planner_B.i_n <
         lspbSegIndices_size_idx_1; joint_trajectory_planner_B.i_n++) {
      joint_trajectory_planner_B.coefIndex[joint_trajectory_planner_B.g_data[joint_trajectory_planner_B.i_n]
        - 1] = true;
    }

    lspbSegIndices_size_idx_1 = 0;
    joint_trajectory_planner_B.indivPolyDim = 0;
    for (joint_trajectory_planner_B.i_n = 0; joint_trajectory_planner_B.i_n < 18;
         joint_trajectory_planner_B.i_n++) {
      if (joint_trajectory_planner_B.coefIndex[joint_trajectory_planner_B.i_n])
      {
        lspbSegIndices_size_idx_1++;
        joint_trajectory_planner_B.o_data[joint_trajectory_planner_B.indivPolyDim]
          = static_cast<int8_T>(joint_trajectory_planner_B.i_n + 1);
        joint_trajectory_planner_B.indivPolyDim++;
      }
    }

    for (joint_trajectory_planner_B.i_n = 0; joint_trajectory_planner_B.i_n < 3;
         joint_trajectory_planner_B.i_n++) {
      for (joint_trajectory_planner_B.indivPolyDim = 0;
           joint_trajectory_planner_B.indivPolyDim < lspbSegIndices_size_idx_1;
           joint_trajectory_planner_B.indivPolyDim++) {
        joint_trajectory_planner_B.coeffMat
          [(joint_trajectory_planner_B.o_data[joint_trajectory_planner_B.indivPolyDim]
            + 18 * joint_trajectory_planner_B.i_n) - 1] =
          joint_trajectory_planner_B.coefs[lspbSegIndices_size_idx_1 *
          joint_trajectory_planner_B.i_n +
          joint_trajectory_planner_B.indivPolyDim];
      }
    }

    joint_trajectory_planner_B.acc =
      joint_trajectory_planner_B.breakMat[joint_trajectory_planner_B.b_idx];
    joint_trajectory_planner_B.breakMat[joint_trajectory_planner_B.b_idx] +=
      joint_trajectory_planner_B.breaks[0];
    joint_trajectory_planner_B.breakMat[joint_trajectory_planner_B.b_idx + 6] =
      joint_trajectory_planner_B.breaks[1] + joint_trajectory_planner_B.acc;
    joint_trajectory_planner_B.breakMat[joint_trajectory_planner_B.b_idx + 12] =
      joint_trajectory_planner_B.breaks[2] + joint_trajectory_planner_B.acc;
    joint_trajectory_planner_B.breakMat[joint_trajectory_planner_B.b_idx + 18] =
      joint_trajectory_planner_B.breaks[3] + joint_trajectory_planner_B.acc;
    joint_trajectory_planner_B.a[joint_trajectory_planner_B.b_idx] =
      joint_trajectory_planner_B.parameterMat[joint_trajectory_planner_B.b_idx +
      30];
  }

  hasMultipleBreaks = join_checkPolyForMultipleBreaks
    (joint_trajectory_planner_B.breakMat);
  joint__processPolynomialResults(joint_trajectory_planner_B.breakMat,
    joint_trajectory_planner_B.coeffMat, hasMultipleBreaks,
    joint_trajectory_planner_B.breaksCell, joint_trajectory_planner_B.coeffsCell);
  if (!rtIsNaN(joint_trajectory_planner_B.a[0])) {
    joint_trajectory_planner_B.b_idx = 1;
  } else {
    joint_trajectory_planner_B.b_idx = 0;
    joint_trajectory_planner_B.i_n = 2;
    exitg1 = false;
    while ((!exitg1) && (joint_trajectory_planner_B.i_n < 7)) {
      if (!rtIsNaN(joint_trajectory_planner_B.a[joint_trajectory_planner_B.i_n -
                   1])) {
        joint_trajectory_planner_B.b_idx = joint_trajectory_planner_B.i_n;
        exitg1 = true;
      } else {
        joint_trajectory_planner_B.i_n++;
      }
    }
  }

  if (joint_trajectory_planner_B.b_idx == 0) {
    t[1] = joint_trajectory_planner_B.a[0];
  } else {
    joint_trajectory_planner_B.segAcc =
      joint_trajectory_planner_B.a[joint_trajectory_planner_B.b_idx - 1];
    for (joint_trajectory_planner_B.i_n = joint_trajectory_planner_B.b_idx + 1;
         joint_trajectory_planner_B.i_n < 7; joint_trajectory_planner_B.i_n++) {
      joint_trajectory_planner_B.acc =
        joint_trajectory_planner_B.a[joint_trajectory_planner_B.i_n - 1];
      if (joint_trajectory_planner_B.segAcc < joint_trajectory_planner_B.acc) {
        joint_trajectory_planner_B.segAcc = joint_trajectory_planner_B.acc;
      }
    }

    t[1] = joint_trajectory_planner_B.segAcc;
  }

  t[0] = 0.0;
  if (hasMultipleBreaks) {
    joint_trajectory_planner_B.i_n = 6;
    joint_trajectory_planner_B.indivPolyDim = 1;
  } else {
    joint_trajectory_planner_B.i_n = 1;
    joint_trajectory_planner_B.indivPolyDim = 6;
  }

  lspbSegIndices_size_idx_1 = joint_trajectory_planner_B.i_n - 1;
  *ppCell_size = joint_trajectory_planner_B.i_n;
  for (joint_trajectory_planner_B.b_idx = 0; joint_trajectory_planner_B.b_idx <=
       lspbSegIndices_size_idx_1; joint_trajectory_planner_B.b_idx++) {
    if (hasMultipleBreaks) {
      lspbSegIndices_size_idx_1_tmp = 1;
      joint_trajectory_planner_B.rowSelection_data[0] = static_cast<int8_T>
        (joint_trajectory_planner_B.b_idx + 1);
      joint_trajectory_planner_B.i_n = joint_trajectory_planner_B.b_idx + 1;
    } else {
      lspbSegIndices_size_idx_1_tmp = 6;
      for (joint_trajectory_planner_B.i_n = 0; joint_trajectory_planner_B.i_n <
           6; joint_trajectory_planner_B.i_n++) {
        joint_trajectory_planner_B.rowSelection_data[joint_trajectory_planner_B.i_n]
          = static_cast<int8_T>(joint_trajectory_planner_B.i_n + 1);
      }

      joint_trajectory_planner_B.i_n = 1;
    }

    j_generateTrajectoriesFromCoefs
      (joint_trajectory_planner_B.breaksCell[joint_trajectory_planner_B.i_n - 1]
       .f1.data,
       joint_trajectory_planner_B.breaksCell[joint_trajectory_planner_B.i_n - 1]
       .f1.size,
       joint_trajectory_planner_B.coeffsCell[joint_trajectory_planner_B.i_n - 1]
       .f1.data,
       joint_trajectory_planner_B.coeffsCell[joint_trajectory_planner_B.i_n - 1]
       .f1.size, static_cast<real_T>(joint_trajectory_planner_B.indivPolyDim), t,
       joint_trajectory_planner_B.b_data_cv, joint_trajectory_planner_B.b_size,
       joint_trajectory_planner_B.c_data, joint_trajectory_planner_B.c_size,
       joint_trajectory_planner_B.d_data, joint_trajectory_planner_B.d_size,
       ppCell_data[joint_trajectory_planner_B.b_idx].breaks.data,
       ppCell_data[joint_trajectory_planner_B.b_idx].breaks.size,
       ppCell_data[joint_trajectory_planner_B.b_idx].coefs.data,
       ppCell_data[joint_trajectory_planner_B.b_idx].coefs.size);
    for (joint_trajectory_planner_B.i_n = 0; joint_trajectory_planner_B.i_n <
         lspbSegIndices_size_idx_1_tmp; joint_trajectory_planner_B.i_n++) {
      joint_trajectory_planner_B.l_tmp_data[joint_trajectory_planner_B.i_n] =
        static_cast<int8_T>
        (joint_trajectory_planner_B.rowSelection_data[joint_trajectory_planner_B.i_n]
         - 1);
    }

    for (joint_trajectory_planner_B.i_n = 0; joint_trajectory_planner_B.i_n <
         lspbSegIndices_size_idx_1_tmp; joint_trajectory_planner_B.i_n++) {
      q[joint_trajectory_planner_B.l_tmp_data[joint_trajectory_planner_B.i_n]] =
        joint_trajectory_planner_B.b_data_cv[joint_trajectory_planner_B.i_n];
    }

    for (joint_trajectory_planner_B.i_n = 0; joint_trajectory_planner_B.i_n <
         lspbSegIndices_size_idx_1_tmp; joint_trajectory_planner_B.i_n++) {
      qd[joint_trajectory_planner_B.l_tmp_data[joint_trajectory_planner_B.i_n]] =
        joint_trajectory_planner_B.c_data[joint_trajectory_planner_B.i_n];
    }

    for (joint_trajectory_planner_B.i_n = 0; joint_trajectory_planner_B.i_n <
         lspbSegIndices_size_idx_1_tmp; joint_trajectory_planner_B.i_n++) {
      qdd[joint_trajectory_planner_B.l_tmp_data[joint_trajectory_planner_B.i_n]]
        = joint_trajectory_planner_B.d_data[joint_trajectory_planner_B.i_n];
    }

    for (joint_trajectory_planner_B.i_n = 0; joint_trajectory_planner_B.i_n <
         lspbSegIndices_size_idx_1_tmp; joint_trajectory_planner_B.i_n++) {
      q[joint_trajectory_planner_B.l_tmp_data[joint_trajectory_planner_B.i_n] +
        6] = joint_trajectory_planner_B.b_data_cv[joint_trajectory_planner_B.i_n
        + joint_trajectory_planner_B.b_size[0]];
    }

    for (joint_trajectory_planner_B.i_n = 0; joint_trajectory_planner_B.i_n <
         lspbSegIndices_size_idx_1_tmp; joint_trajectory_planner_B.i_n++) {
      qd[joint_trajectory_planner_B.l_tmp_data[joint_trajectory_planner_B.i_n] +
        6] = joint_trajectory_planner_B.c_data[joint_trajectory_planner_B.i_n +
        joint_trajectory_planner_B.c_size[0]];
    }

    for (joint_trajectory_planner_B.i_n = 0; joint_trajectory_planner_B.i_n <
         lspbSegIndices_size_idx_1_tmp; joint_trajectory_planner_B.i_n++) {
      qdd[joint_trajectory_planner_B.l_tmp_data[joint_trajectory_planner_B.i_n]
        + 6] = joint_trajectory_planner_B.d_data[joint_trajectory_planner_B.i_n
        + joint_trajectory_planner_B.d_size[0]];
    }
  }
}

static void TrapVelTrajSys_extract1DimFromP(const real_T pp_breaks_data[], const
  int32_T pp_breaks_size[2], const real_T pp_coefs_data[], const int32_T
  pp_coefs_size[3], real_T breaks_data[], int32_T breaks_size[2], real_T
  oneDimCoeffs_data[], int32_T oneDimCoeffs_size[2])
{
  boolean_T coeffIndex_data[36];
  int32_T ntilerows;
  int32_T ibcol;
  boolean_T a[6];
  int32_T i;
  int32_T loop_ub;
  static const boolean_T tmp[6] = { true, false, false, false, false, false };

  breaks_size[0] = 1;
  breaks_size[1] = pp_breaks_size[1];
  loop_ub = pp_breaks_size[1];
  if (0 <= loop_ub - 1) {
    memcpy(&breaks_data[0], &pp_breaks_data[0], loop_ub * sizeof(real_T));
  }

  for (i = 0; i < 6; i++) {
    a[i] = tmp[i];
  }

  ntilerows = pp_breaks_size[1] - 2;
  for (loop_ub = 0; loop_ub <= ntilerows; loop_ub++) {
    ibcol = loop_ub * 6 - 1;
    for (i = 0; i < 6; i++) {
      coeffIndex_data[(ibcol + i) + 1] = a[i];
    }
  }

  ntilerows = (pp_breaks_size[1] - 1) * 6 - 1;
  loop_ub = 0;
  ibcol = 0;
  for (i = 0; i <= ntilerows; i++) {
    if (coeffIndex_data[i]) {
      loop_ub++;
      joint_trajectory_planner_B.b_data_p[ibcol] = i + 1;
      ibcol++;
    }
  }

  ibcol = div_nzp_s32(pp_coefs_size[0] * pp_coefs_size[1] * 3, 3);
  oneDimCoeffs_size[0] = loop_ub;
  oneDimCoeffs_size[1] = 3;
  for (i = 0; i < 3; i++) {
    for (ntilerows = 0; ntilerows < loop_ub; ntilerows++) {
      oneDimCoeffs_data[ntilerows + loop_ub * i] = pp_coefs_data[(ibcol * i +
        joint_trajectory_planner_B.b_data_p[ntilerows]) - 1];
    }
  }
}

static void joint_trajectory_planne_ppval_a(const real_T pp_breaks_data[], const
  int32_T pp_breaks_size[2], const real_T pp_coefs_data[], real_T x, real_T
  v_data[], int32_T *v_size)
{
  int32_T ip;
  real_T xloc;
  *v_size = 1;
  if (rtIsNaN(x)) {
    v_data[0] = x;
  } else {
    ip = joint_trajectory_planne_bsearch(pp_breaks_data, pp_breaks_size, x) - 1;
    xloc = x - pp_breaks_data[ip];
    v_data[0] = (pp_coefs_data[(ip + pp_breaks_size[1]) - 1] + xloc *
                 pp_coefs_data[ip]) * xloc + pp_coefs_data[((pp_breaks_size[1] -
      1) << 1) + ip];
  }
}

static void TrapVelTrajSys_generate1DTrajec(const real_T breaks_data[], const
  int32_T breaks_size[2], const real_T coefs_data[], const int32_T coefs_size[2],
  real_T t, real_T q_data[], int32_T *q_size, real_T qd_data[], int32_T *qd_size,
  real_T qdd_data[], int32_T *qdd_size)
{
  int32_T loop_ub;
  int32_T dCoefs_size[2];
  int32_T breaks_size_0[2];
  int32_T breaks_size_1[2];
  int32_T breaks_size_2[2];
  joint_traj_polyCoeffsDerivative(coefs_data, coefs_size,
    joint_trajectory_planner_B.dCoefs_data, dCoefs_size);
  breaks_size_0[0] = 1;
  breaks_size_0[1] = breaks_size[1];
  loop_ub = breaks_size[1];
  if (0 <= loop_ub - 1) {
    memcpy(&joint_trajectory_planner_B.breaks_data_g[0], &breaks_data[0],
           loop_ub * sizeof(real_T));
  }

  joint_trajectory_planne_ppval_a(joint_trajectory_planner_B.breaks_data_g,
    breaks_size_0, coefs_data, t, q_data, q_size);
  breaks_size_1[0] = 1;
  breaks_size_1[1] = breaks_size[1];
  loop_ub = breaks_size[1];
  if (0 <= loop_ub - 1) {
    memcpy(&joint_trajectory_planner_B.breaks_data_g[0], &breaks_data[0],
           loop_ub * sizeof(real_T));
  }

  joint_trajectory_planne_ppval_a(joint_trajectory_planner_B.breaks_data_g,
    breaks_size_1, joint_trajectory_planner_B.dCoefs_data, t, qd_data, qd_size);
  joint_traj_polyCoeffsDerivative(joint_trajectory_planner_B.dCoefs_data,
    dCoefs_size, joint_trajectory_planner_B.tmp_data_m, breaks_size_0);
  breaks_size_2[0] = 1;
  breaks_size_2[1] = breaks_size[1];
  loop_ub = breaks_size[1];
  if (0 <= loop_ub - 1) {
    memcpy(&joint_trajectory_planner_B.breaks_data_g[0], &breaks_data[0],
           loop_ub * sizeof(real_T));
  }

  joint_trajectory_planne_ppval_a(joint_trajectory_planner_B.breaks_data_g,
    breaks_size_2, joint_trajectory_planner_B.tmp_data_m, t, qdd_data, qdd_size);
}

static void TrapVelTrajSys_extract1DimFro_a(const real_T pp_breaks_data[], const
  int32_T pp_breaks_size[2], const real_T pp_coefs_data[], const int32_T
  pp_coefs_size[3], real_T breaks_data[], int32_T breaks_size[2], real_T
  oneDimCoeffs_data[], int32_T oneDimCoeffs_size[2])
{
  boolean_T coeffIndex_data[36];
  int32_T ntilerows;
  int32_T ibcol;
  boolean_T a[6];
  int32_T i;
  int32_T loop_ub;
  static const boolean_T tmp[6] = { false, true, false, false, false, false };

  breaks_size[0] = 1;
  breaks_size[1] = pp_breaks_size[1];
  loop_ub = pp_breaks_size[1];
  if (0 <= loop_ub - 1) {
    memcpy(&breaks_data[0], &pp_breaks_data[0], loop_ub * sizeof(real_T));
  }

  for (i = 0; i < 6; i++) {
    a[i] = tmp[i];
  }

  ntilerows = pp_breaks_size[1] - 2;
  for (loop_ub = 0; loop_ub <= ntilerows; loop_ub++) {
    ibcol = loop_ub * 6 - 1;
    for (i = 0; i < 6; i++) {
      coeffIndex_data[(ibcol + i) + 1] = a[i];
    }
  }

  ntilerows = (pp_breaks_size[1] - 1) * 6 - 1;
  loop_ub = 0;
  ibcol = 0;
  for (i = 0; i <= ntilerows; i++) {
    if (coeffIndex_data[i]) {
      loop_ub++;
      joint_trajectory_planner_B.b_data_b[ibcol] = i + 1;
      ibcol++;
    }
  }

  ibcol = div_nzp_s32(pp_coefs_size[0] * pp_coefs_size[1] * 3, 3);
  oneDimCoeffs_size[0] = loop_ub;
  oneDimCoeffs_size[1] = 3;
  for (i = 0; i < 3; i++) {
    for (ntilerows = 0; ntilerows < loop_ub; ntilerows++) {
      oneDimCoeffs_data[ntilerows + loop_ub * i] = pp_coefs_data[(ibcol * i +
        joint_trajectory_planner_B.b_data_b[ntilerows]) - 1];
    }
  }
}

static void TrapVelTrajSys_extract1DimFr_ao(const real_T pp_breaks_data[], const
  int32_T pp_breaks_size[2], const real_T pp_coefs_data[], const int32_T
  pp_coefs_size[3], real_T breaks_data[], int32_T breaks_size[2], real_T
  oneDimCoeffs_data[], int32_T oneDimCoeffs_size[2])
{
  boolean_T coeffIndex_data[36];
  int32_T ntilerows;
  int32_T ibcol;
  boolean_T a[6];
  int32_T i;
  int32_T loop_ub;
  static const boolean_T tmp[6] = { false, false, true, false, false, false };

  breaks_size[0] = 1;
  breaks_size[1] = pp_breaks_size[1];
  loop_ub = pp_breaks_size[1];
  if (0 <= loop_ub - 1) {
    memcpy(&breaks_data[0], &pp_breaks_data[0], loop_ub * sizeof(real_T));
  }

  for (i = 0; i < 6; i++) {
    a[i] = tmp[i];
  }

  ntilerows = pp_breaks_size[1] - 2;
  for (loop_ub = 0; loop_ub <= ntilerows; loop_ub++) {
    ibcol = loop_ub * 6 - 1;
    for (i = 0; i < 6; i++) {
      coeffIndex_data[(ibcol + i) + 1] = a[i];
    }
  }

  ntilerows = (pp_breaks_size[1] - 1) * 6 - 1;
  loop_ub = 0;
  ibcol = 0;
  for (i = 0; i <= ntilerows; i++) {
    if (coeffIndex_data[i]) {
      loop_ub++;
      joint_trajectory_planner_B.b_data_cx[ibcol] = i + 1;
      ibcol++;
    }
  }

  ibcol = div_nzp_s32(pp_coefs_size[0] * pp_coefs_size[1] * 3, 3);
  oneDimCoeffs_size[0] = loop_ub;
  oneDimCoeffs_size[1] = 3;
  for (i = 0; i < 3; i++) {
    for (ntilerows = 0; ntilerows < loop_ub; ntilerows++) {
      oneDimCoeffs_data[ntilerows + loop_ub * i] = pp_coefs_data[(ibcol * i +
        joint_trajectory_planner_B.b_data_cx[ntilerows]) - 1];
    }
  }
}

static void TrapVelTrajSys_extract1DimF_aoq(const real_T pp_breaks_data[], const
  int32_T pp_breaks_size[2], const real_T pp_coefs_data[], const int32_T
  pp_coefs_size[3], real_T breaks_data[], int32_T breaks_size[2], real_T
  oneDimCoeffs_data[], int32_T oneDimCoeffs_size[2])
{
  boolean_T coeffIndex_data[36];
  int32_T ntilerows;
  int32_T ibcol;
  boolean_T a[6];
  int32_T i;
  int32_T loop_ub;
  static const boolean_T tmp[6] = { false, false, false, true, false, false };

  breaks_size[0] = 1;
  breaks_size[1] = pp_breaks_size[1];
  loop_ub = pp_breaks_size[1];
  if (0 <= loop_ub - 1) {
    memcpy(&breaks_data[0], &pp_breaks_data[0], loop_ub * sizeof(real_T));
  }

  for (i = 0; i < 6; i++) {
    a[i] = tmp[i];
  }

  ntilerows = pp_breaks_size[1] - 2;
  for (loop_ub = 0; loop_ub <= ntilerows; loop_ub++) {
    ibcol = loop_ub * 6 - 1;
    for (i = 0; i < 6; i++) {
      coeffIndex_data[(ibcol + i) + 1] = a[i];
    }
  }

  ntilerows = (pp_breaks_size[1] - 1) * 6 - 1;
  loop_ub = 0;
  ibcol = 0;
  for (i = 0; i <= ntilerows; i++) {
    if (coeffIndex_data[i]) {
      loop_ub++;
      joint_trajectory_planner_B.b_data_k[ibcol] = i + 1;
      ibcol++;
    }
  }

  ibcol = div_nzp_s32(pp_coefs_size[0] * pp_coefs_size[1] * 3, 3);
  oneDimCoeffs_size[0] = loop_ub;
  oneDimCoeffs_size[1] = 3;
  for (i = 0; i < 3; i++) {
    for (ntilerows = 0; ntilerows < loop_ub; ntilerows++) {
      oneDimCoeffs_data[ntilerows + loop_ub * i] = pp_coefs_data[(ibcol * i +
        joint_trajectory_planner_B.b_data_k[ntilerows]) - 1];
    }
  }
}

static void TrapVelTrajSys_extract1Dim_aoqo(const real_T pp_breaks_data[], const
  int32_T pp_breaks_size[2], const real_T pp_coefs_data[], const int32_T
  pp_coefs_size[3], real_T breaks_data[], int32_T breaks_size[2], real_T
  oneDimCoeffs_data[], int32_T oneDimCoeffs_size[2])
{
  boolean_T coeffIndex_data[36];
  int32_T ntilerows;
  int32_T ibcol;
  boolean_T a[6];
  int32_T i;
  int32_T loop_ub;
  static const boolean_T tmp[6] = { false, false, false, false, true, false };

  breaks_size[0] = 1;
  breaks_size[1] = pp_breaks_size[1];
  loop_ub = pp_breaks_size[1];
  if (0 <= loop_ub - 1) {
    memcpy(&breaks_data[0], &pp_breaks_data[0], loop_ub * sizeof(real_T));
  }

  for (i = 0; i < 6; i++) {
    a[i] = tmp[i];
  }

  ntilerows = pp_breaks_size[1] - 2;
  for (loop_ub = 0; loop_ub <= ntilerows; loop_ub++) {
    ibcol = loop_ub * 6 - 1;
    for (i = 0; i < 6; i++) {
      coeffIndex_data[(ibcol + i) + 1] = a[i];
    }
  }

  ntilerows = (pp_breaks_size[1] - 1) * 6 - 1;
  loop_ub = 0;
  ibcol = 0;
  for (i = 0; i <= ntilerows; i++) {
    if (coeffIndex_data[i]) {
      loop_ub++;
      joint_trajectory_planner_B.b_data_c[ibcol] = i + 1;
      ibcol++;
    }
  }

  ibcol = div_nzp_s32(pp_coefs_size[0] * pp_coefs_size[1] * 3, 3);
  oneDimCoeffs_size[0] = loop_ub;
  oneDimCoeffs_size[1] = 3;
  for (i = 0; i < 3; i++) {
    for (ntilerows = 0; ntilerows < loop_ub; ntilerows++) {
      oneDimCoeffs_data[ntilerows + loop_ub * i] = pp_coefs_data[(ibcol * i +
        joint_trajectory_planner_B.b_data_c[ntilerows]) - 1];
    }
  }
}

static void TrapVelTrajSys_extract1Di_aoqov(const real_T pp_breaks_data[], const
  int32_T pp_breaks_size[2], const real_T pp_coefs_data[], const int32_T
  pp_coefs_size[3], real_T breaks_data[], int32_T breaks_size[2], real_T
  oneDimCoeffs_data[], int32_T oneDimCoeffs_size[2])
{
  boolean_T coeffIndex_data[36];
  int32_T ntilerows;
  int32_T ibcol;
  boolean_T a[6];
  int32_T i;
  int32_T loop_ub;
  static const boolean_T tmp[6] = { false, false, false, false, false, true };

  breaks_size[0] = 1;
  breaks_size[1] = pp_breaks_size[1];
  loop_ub = pp_breaks_size[1];
  if (0 <= loop_ub - 1) {
    memcpy(&breaks_data[0], &pp_breaks_data[0], loop_ub * sizeof(real_T));
  }

  for (i = 0; i < 6; i++) {
    a[i] = tmp[i];
  }

  ntilerows = pp_breaks_size[1] - 2;
  for (loop_ub = 0; loop_ub <= ntilerows; loop_ub++) {
    ibcol = loop_ub * 6 - 1;
    for (i = 0; i < 6; i++) {
      coeffIndex_data[(ibcol + i) + 1] = a[i];
    }
  }

  ntilerows = (pp_breaks_size[1] - 1) * 6 - 1;
  loop_ub = 0;
  ibcol = 0;
  for (i = 0; i <= ntilerows; i++) {
    if (coeffIndex_data[i]) {
      loop_ub++;
      joint_trajectory_planner_B.b_data[ibcol] = i + 1;
      ibcol++;
    }
  }

  ibcol = div_nzp_s32(pp_coefs_size[0] * pp_coefs_size[1] * 3, 3);
  oneDimCoeffs_size[0] = loop_ub;
  oneDimCoeffs_size[1] = 3;
  for (i = 0; i < 3; i++) {
    for (ntilerows = 0; ntilerows < loop_ub; ntilerows++) {
      oneDimCoeffs_data[ntilerows + loop_ub * i] = pp_coefs_data[(ibcol * i +
        joint_trajectory_planner_B.b_data[ntilerows]) - 1];
    }
  }
}

static void joint_t_TrapVelTrajSys_stepImpl(real_T time, const real_T
  varargin_1[12], const real_T varargin_2[6], real_T varargin_3, real_T q[6],
  real_T qd[6], real_T qdd[6])
{
  joint_trajectory_pl_trapveltraj(varargin_1, varargin_2, varargin_3,
    joint_trajectory_planner_B.unusedU10, joint_trajectory_planner_B.unusedU11,
    joint_trajectory_planner_B.unusedU12, joint_trajectory_planner_B.unusedU13,
    joint_trajectory_planner_B.trajPP.data,
    &joint_trajectory_planner_B.trajPP.size);
  if (joint_trajectory_planner_B.trajPP.size > 1) {
    joint_trajectory_planner_B.breaks_size[0] = 1;
    joint_trajectory_planner_B.breaks_size[1] =
      joint_trajectory_planner_B.trajPP.data[0].breaks.size[1];
    joint_trajectory_planner_B.loop_ub = joint_trajectory_planner_B.trajPP.data
      [0].breaks.size[1];
    for (joint_trajectory_planner_B.oneDimCoeffs_size_tmp = 0;
         joint_trajectory_planner_B.oneDimCoeffs_size_tmp <
         joint_trajectory_planner_B.loop_ub;
         joint_trajectory_planner_B.oneDimCoeffs_size_tmp++) {
      joint_trajectory_planner_B.breaks_data[joint_trajectory_planner_B.oneDimCoeffs_size_tmp]
        = joint_trajectory_planner_B.trajPP.data[0]
        .breaks.data[joint_trajectory_planner_B.oneDimCoeffs_size_tmp];
    }

    joint_trajectory_planner_B.oneDimCoeffs_size_tmp = div_nzp_s32
      (joint_trajectory_planner_B.trajPP.data[0].coefs.size[0] *
       joint_trajectory_planner_B.trajPP.data[0].coefs.size[1] * 3, 3);
    joint_trajectory_planner_B.oneDimCoeffs_size[0] =
      joint_trajectory_planner_B.oneDimCoeffs_size_tmp;
    joint_trajectory_planner_B.loop_ub =
      joint_trajectory_planner_B.oneDimCoeffs_size_tmp * 3 - 1;
    for (joint_trajectory_planner_B.oneDimCoeffs_size_tmp = 0;
         joint_trajectory_planner_B.oneDimCoeffs_size_tmp <=
         joint_trajectory_planner_B.loop_ub;
         joint_trajectory_planner_B.oneDimCoeffs_size_tmp++) {
      joint_trajectory_planner_B.oneDimCoeffs_data[joint_trajectory_planner_B.oneDimCoeffs_size_tmp]
        = joint_trajectory_planner_B.trajPP.data[0]
        .coefs.data[joint_trajectory_planner_B.oneDimCoeffs_size_tmp];
    }
  } else {
    TrapVelTrajSys_extract1DimFromP(joint_trajectory_planner_B.trajPP.data[0].
      breaks.data, joint_trajectory_planner_B.trajPP.data[0].breaks.size,
      joint_trajectory_planner_B.trajPP.data[0].coefs.data,
      joint_trajectory_planner_B.trajPP.data[0].coefs.size,
      joint_trajectory_planner_B.breaks_data,
      joint_trajectory_planner_B.breaks_size,
      joint_trajectory_planner_B.oneDimCoeffs_data,
      joint_trajectory_planner_B.oneDimCoeffs_size);
  }

  joint_trajectory_planner_B.evalCoeffs_size[0] =
    joint_trajectory_planner_B.oneDimCoeffs_size[0];
  joint_trajectory_planner_B.evalCoeffs_size[1] = 3;
  joint_trajectory_planner_B.loop_ub =
    joint_trajectory_planner_B.oneDimCoeffs_size[0] * 3 - 1;
  if (0 <= joint_trajectory_planner_B.loop_ub) {
    memset(&joint_trajectory_planner_B.evalCoeffs_data[0], 0,
           (joint_trajectory_planner_B.loop_ub + 1) * sizeof(real_T));
  }

  if (1 > joint_trajectory_planner_B.oneDimCoeffs_size[0]) {
    joint_trajectory_planner_B.f = 0;
  } else {
    joint_trajectory_planner_B.f = joint_trajectory_planner_B.oneDimCoeffs_size
      [0];
  }

  for (joint_trajectory_planner_B.oneDimCoeffs_size_tmp = 0;
       joint_trajectory_planner_B.oneDimCoeffs_size_tmp < 3;
       joint_trajectory_planner_B.oneDimCoeffs_size_tmp++) {
    for (joint_trajectory_planner_B.loop_ub = 0;
         joint_trajectory_planner_B.loop_ub < joint_trajectory_planner_B.f;
         joint_trajectory_planner_B.loop_ub++) {
      joint_trajectory_planner_B.evalCoeffs_data[joint_trajectory_planner_B.loop_ub
        + joint_trajectory_planner_B.evalCoeffs_size[0] *
        joint_trajectory_planner_B.oneDimCoeffs_size_tmp] =
        joint_trajectory_planner_B.oneDimCoeffs_data[joint_trajectory_planner_B.oneDimCoeffs_size
        [0] * joint_trajectory_planner_B.oneDimCoeffs_size_tmp +
        joint_trajectory_planner_B.loop_ub];
    }
  }

  TrapVelTrajSys_generate1DTrajec(joint_trajectory_planner_B.breaks_data,
    joint_trajectory_planner_B.breaks_size,
    joint_trajectory_planner_B.evalCoeffs_data,
    joint_trajectory_planner_B.evalCoeffs_size, time,
    &joint_trajectory_planner_B.d_data_g, &joint_trajectory_planner_B.d_size_b,
    &joint_trajectory_planner_B.c_data_d, &joint_trajectory_planner_B.c_size_o,
    &joint_trajectory_planner_B.b_data_j, &joint_trajectory_planner_B.b_size_l);
  if (joint_trajectory_planner_B.trajPP.size > 1) {
    joint_trajectory_planner_B.breaks_size[0] = 1;
    joint_trajectory_planner_B.breaks_size[1] =
      joint_trajectory_planner_B.trajPP.data[1].breaks.size[1];
    joint_trajectory_planner_B.loop_ub = joint_trajectory_planner_B.trajPP.data
      [1].breaks.size[1];
    for (joint_trajectory_planner_B.oneDimCoeffs_size_tmp = 0;
         joint_trajectory_planner_B.oneDimCoeffs_size_tmp <
         joint_trajectory_planner_B.loop_ub;
         joint_trajectory_planner_B.oneDimCoeffs_size_tmp++) {
      joint_trajectory_planner_B.breaks_data[joint_trajectory_planner_B.oneDimCoeffs_size_tmp]
        = joint_trajectory_planner_B.trajPP.data[1]
        .breaks.data[joint_trajectory_planner_B.oneDimCoeffs_size_tmp];
    }

    joint_trajectory_planner_B.oneDimCoeffs_size_tmp = div_nzp_s32
      (joint_trajectory_planner_B.trajPP.data[1].coefs.size[0] *
       joint_trajectory_planner_B.trajPP.data[1].coefs.size[1] * 3, 3);
    joint_trajectory_planner_B.oneDimCoeffs_size[0] =
      joint_trajectory_planner_B.oneDimCoeffs_size_tmp;
    joint_trajectory_planner_B.loop_ub =
      joint_trajectory_planner_B.oneDimCoeffs_size_tmp * 3 - 1;
    for (joint_trajectory_planner_B.oneDimCoeffs_size_tmp = 0;
         joint_trajectory_planner_B.oneDimCoeffs_size_tmp <=
         joint_trajectory_planner_B.loop_ub;
         joint_trajectory_planner_B.oneDimCoeffs_size_tmp++) {
      joint_trajectory_planner_B.oneDimCoeffs_data[joint_trajectory_planner_B.oneDimCoeffs_size_tmp]
        = joint_trajectory_planner_B.trajPP.data[1]
        .coefs.data[joint_trajectory_planner_B.oneDimCoeffs_size_tmp];
    }
  } else {
    TrapVelTrajSys_extract1DimFro_a(joint_trajectory_planner_B.trajPP.data[0].
      breaks.data, joint_trajectory_planner_B.trajPP.data[0].breaks.size,
      joint_trajectory_planner_B.trajPP.data[0].coefs.data,
      joint_trajectory_planner_B.trajPP.data[0].coefs.size,
      joint_trajectory_planner_B.breaks_data,
      joint_trajectory_planner_B.breaks_size,
      joint_trajectory_planner_B.oneDimCoeffs_data,
      joint_trajectory_planner_B.oneDimCoeffs_size);
  }

  joint_trajectory_planner_B.evalCoeffs_size[0] =
    joint_trajectory_planner_B.oneDimCoeffs_size[0];
  joint_trajectory_planner_B.evalCoeffs_size[1] = 3;
  joint_trajectory_planner_B.loop_ub =
    joint_trajectory_planner_B.oneDimCoeffs_size[0] * 3 - 1;
  if (0 <= joint_trajectory_planner_B.loop_ub) {
    memset(&joint_trajectory_planner_B.evalCoeffs_data[0], 0,
           (joint_trajectory_planner_B.loop_ub + 1) * sizeof(real_T));
  }

  if (1 > joint_trajectory_planner_B.oneDimCoeffs_size[0]) {
    joint_trajectory_planner_B.f = 0;
  } else {
    joint_trajectory_planner_B.f = joint_trajectory_planner_B.oneDimCoeffs_size
      [0];
  }

  for (joint_trajectory_planner_B.oneDimCoeffs_size_tmp = 0;
       joint_trajectory_planner_B.oneDimCoeffs_size_tmp < 3;
       joint_trajectory_planner_B.oneDimCoeffs_size_tmp++) {
    for (joint_trajectory_planner_B.loop_ub = 0;
         joint_trajectory_planner_B.loop_ub < joint_trajectory_planner_B.f;
         joint_trajectory_planner_B.loop_ub++) {
      joint_trajectory_planner_B.evalCoeffs_data[joint_trajectory_planner_B.loop_ub
        + joint_trajectory_planner_B.evalCoeffs_size[0] *
        joint_trajectory_planner_B.oneDimCoeffs_size_tmp] =
        joint_trajectory_planner_B.oneDimCoeffs_data[joint_trajectory_planner_B.oneDimCoeffs_size
        [0] * joint_trajectory_planner_B.oneDimCoeffs_size_tmp +
        joint_trajectory_planner_B.loop_ub];
    }
  }

  TrapVelTrajSys_generate1DTrajec(joint_trajectory_planner_B.breaks_data,
    joint_trajectory_planner_B.breaks_size,
    joint_trajectory_planner_B.evalCoeffs_data,
    joint_trajectory_planner_B.evalCoeffs_size, time,
    &joint_trajectory_planner_B.j_data, &joint_trajectory_planner_B.d_size_b,
    &joint_trajectory_planner_B.i_data, &joint_trajectory_planner_B.c_size_o,
    &joint_trajectory_planner_B.h_data, &joint_trajectory_planner_B.b_size_l);
  if (joint_trajectory_planner_B.trajPP.size > 1) {
    joint_trajectory_planner_B.breaks_size[0] = 1;
    joint_trajectory_planner_B.breaks_size[1] =
      joint_trajectory_planner_B.trajPP.data[2].breaks.size[1];
    joint_trajectory_planner_B.loop_ub = joint_trajectory_planner_B.trajPP.data
      [2].breaks.size[1];
    for (joint_trajectory_planner_B.oneDimCoeffs_size_tmp = 0;
         joint_trajectory_planner_B.oneDimCoeffs_size_tmp <
         joint_trajectory_planner_B.loop_ub;
         joint_trajectory_planner_B.oneDimCoeffs_size_tmp++) {
      joint_trajectory_planner_B.breaks_data[joint_trajectory_planner_B.oneDimCoeffs_size_tmp]
        = joint_trajectory_planner_B.trajPP.data[2]
        .breaks.data[joint_trajectory_planner_B.oneDimCoeffs_size_tmp];
    }

    joint_trajectory_planner_B.oneDimCoeffs_size_tmp = div_nzp_s32
      (joint_trajectory_planner_B.trajPP.data[2].coefs.size[0] *
       joint_trajectory_planner_B.trajPP.data[2].coefs.size[1] * 3, 3);
    joint_trajectory_planner_B.oneDimCoeffs_size[0] =
      joint_trajectory_planner_B.oneDimCoeffs_size_tmp;
    joint_trajectory_planner_B.loop_ub =
      joint_trajectory_planner_B.oneDimCoeffs_size_tmp * 3 - 1;
    for (joint_trajectory_planner_B.oneDimCoeffs_size_tmp = 0;
         joint_trajectory_planner_B.oneDimCoeffs_size_tmp <=
         joint_trajectory_planner_B.loop_ub;
         joint_trajectory_planner_B.oneDimCoeffs_size_tmp++) {
      joint_trajectory_planner_B.oneDimCoeffs_data[joint_trajectory_planner_B.oneDimCoeffs_size_tmp]
        = joint_trajectory_planner_B.trajPP.data[2]
        .coefs.data[joint_trajectory_planner_B.oneDimCoeffs_size_tmp];
    }
  } else {
    TrapVelTrajSys_extract1DimFr_ao(joint_trajectory_planner_B.trajPP.data[0].
      breaks.data, joint_trajectory_planner_B.trajPP.data[0].breaks.size,
      joint_trajectory_planner_B.trajPP.data[0].coefs.data,
      joint_trajectory_planner_B.trajPP.data[0].coefs.size,
      joint_trajectory_planner_B.breaks_data,
      joint_trajectory_planner_B.breaks_size,
      joint_trajectory_planner_B.oneDimCoeffs_data,
      joint_trajectory_planner_B.oneDimCoeffs_size);
  }

  joint_trajectory_planner_B.evalCoeffs_size[0] =
    joint_trajectory_planner_B.oneDimCoeffs_size[0];
  joint_trajectory_planner_B.evalCoeffs_size[1] = 3;
  joint_trajectory_planner_B.loop_ub =
    joint_trajectory_planner_B.oneDimCoeffs_size[0] * 3 - 1;
  if (0 <= joint_trajectory_planner_B.loop_ub) {
    memset(&joint_trajectory_planner_B.evalCoeffs_data[0], 0,
           (joint_trajectory_planner_B.loop_ub + 1) * sizeof(real_T));
  }

  if (1 > joint_trajectory_planner_B.oneDimCoeffs_size[0]) {
    joint_trajectory_planner_B.f = 0;
  } else {
    joint_trajectory_planner_B.f = joint_trajectory_planner_B.oneDimCoeffs_size
      [0];
  }

  for (joint_trajectory_planner_B.oneDimCoeffs_size_tmp = 0;
       joint_trajectory_planner_B.oneDimCoeffs_size_tmp < 3;
       joint_trajectory_planner_B.oneDimCoeffs_size_tmp++) {
    for (joint_trajectory_planner_B.loop_ub = 0;
         joint_trajectory_planner_B.loop_ub < joint_trajectory_planner_B.f;
         joint_trajectory_planner_B.loop_ub++) {
      joint_trajectory_planner_B.evalCoeffs_data[joint_trajectory_planner_B.loop_ub
        + joint_trajectory_planner_B.evalCoeffs_size[0] *
        joint_trajectory_planner_B.oneDimCoeffs_size_tmp] =
        joint_trajectory_planner_B.oneDimCoeffs_data[joint_trajectory_planner_B.oneDimCoeffs_size
        [0] * joint_trajectory_planner_B.oneDimCoeffs_size_tmp +
        joint_trajectory_planner_B.loop_ub];
    }
  }

  TrapVelTrajSys_generate1DTrajec(joint_trajectory_planner_B.breaks_data,
    joint_trajectory_planner_B.breaks_size,
    joint_trajectory_planner_B.evalCoeffs_data,
    joint_trajectory_planner_B.evalCoeffs_size, time,
    &joint_trajectory_planner_B.p_data, &joint_trajectory_planner_B.d_size_b,
    &joint_trajectory_planner_B.o_data_l, &joint_trajectory_planner_B.c_size_o,
    &joint_trajectory_planner_B.n_data, &joint_trajectory_planner_B.b_size_l);
  if (joint_trajectory_planner_B.trajPP.size > 1) {
    joint_trajectory_planner_B.breaks_size[0] = 1;
    joint_trajectory_planner_B.breaks_size[1] =
      joint_trajectory_planner_B.trajPP.data[3].breaks.size[1];
    joint_trajectory_planner_B.loop_ub = joint_trajectory_planner_B.trajPP.data
      [3].breaks.size[1];
    for (joint_trajectory_planner_B.oneDimCoeffs_size_tmp = 0;
         joint_trajectory_planner_B.oneDimCoeffs_size_tmp <
         joint_trajectory_planner_B.loop_ub;
         joint_trajectory_planner_B.oneDimCoeffs_size_tmp++) {
      joint_trajectory_planner_B.breaks_data[joint_trajectory_planner_B.oneDimCoeffs_size_tmp]
        = joint_trajectory_planner_B.trajPP.data[3]
        .breaks.data[joint_trajectory_planner_B.oneDimCoeffs_size_tmp];
    }

    joint_trajectory_planner_B.oneDimCoeffs_size_tmp = div_nzp_s32
      (joint_trajectory_planner_B.trajPP.data[3].coefs.size[0] *
       joint_trajectory_planner_B.trajPP.data[3].coefs.size[1] * 3, 3);
    joint_trajectory_planner_B.oneDimCoeffs_size[0] =
      joint_trajectory_planner_B.oneDimCoeffs_size_tmp;
    joint_trajectory_planner_B.loop_ub =
      joint_trajectory_planner_B.oneDimCoeffs_size_tmp * 3 - 1;
    for (joint_trajectory_planner_B.oneDimCoeffs_size_tmp = 0;
         joint_trajectory_planner_B.oneDimCoeffs_size_tmp <=
         joint_trajectory_planner_B.loop_ub;
         joint_trajectory_planner_B.oneDimCoeffs_size_tmp++) {
      joint_trajectory_planner_B.oneDimCoeffs_data[joint_trajectory_planner_B.oneDimCoeffs_size_tmp]
        = joint_trajectory_planner_B.trajPP.data[3]
        .coefs.data[joint_trajectory_planner_B.oneDimCoeffs_size_tmp];
    }
  } else {
    TrapVelTrajSys_extract1DimF_aoq(joint_trajectory_planner_B.trajPP.data[0].
      breaks.data, joint_trajectory_planner_B.trajPP.data[0].breaks.size,
      joint_trajectory_planner_B.trajPP.data[0].coefs.data,
      joint_trajectory_planner_B.trajPP.data[0].coefs.size,
      joint_trajectory_planner_B.breaks_data,
      joint_trajectory_planner_B.breaks_size,
      joint_trajectory_planner_B.oneDimCoeffs_data,
      joint_trajectory_planner_B.oneDimCoeffs_size);
  }

  joint_trajectory_planner_B.evalCoeffs_size[0] =
    joint_trajectory_planner_B.oneDimCoeffs_size[0];
  joint_trajectory_planner_B.evalCoeffs_size[1] = 3;
  joint_trajectory_planner_B.loop_ub =
    joint_trajectory_planner_B.oneDimCoeffs_size[0] * 3 - 1;
  if (0 <= joint_trajectory_planner_B.loop_ub) {
    memset(&joint_trajectory_planner_B.evalCoeffs_data[0], 0,
           (joint_trajectory_planner_B.loop_ub + 1) * sizeof(real_T));
  }

  if (1 > joint_trajectory_planner_B.oneDimCoeffs_size[0]) {
    joint_trajectory_planner_B.f = 0;
  } else {
    joint_trajectory_planner_B.f = joint_trajectory_planner_B.oneDimCoeffs_size
      [0];
  }

  for (joint_trajectory_planner_B.oneDimCoeffs_size_tmp = 0;
       joint_trajectory_planner_B.oneDimCoeffs_size_tmp < 3;
       joint_trajectory_planner_B.oneDimCoeffs_size_tmp++) {
    for (joint_trajectory_planner_B.loop_ub = 0;
         joint_trajectory_planner_B.loop_ub < joint_trajectory_planner_B.f;
         joint_trajectory_planner_B.loop_ub++) {
      joint_trajectory_planner_B.evalCoeffs_data[joint_trajectory_planner_B.loop_ub
        + joint_trajectory_planner_B.evalCoeffs_size[0] *
        joint_trajectory_planner_B.oneDimCoeffs_size_tmp] =
        joint_trajectory_planner_B.oneDimCoeffs_data[joint_trajectory_planner_B.oneDimCoeffs_size
        [0] * joint_trajectory_planner_B.oneDimCoeffs_size_tmp +
        joint_trajectory_planner_B.loop_ub];
    }
  }

  TrapVelTrajSys_generate1DTrajec(joint_trajectory_planner_B.breaks_data,
    joint_trajectory_planner_B.breaks_size,
    joint_trajectory_planner_B.evalCoeffs_data,
    joint_trajectory_planner_B.evalCoeffs_size, time,
    &joint_trajectory_planner_B.w_data, &joint_trajectory_planner_B.d_size_b,
    &joint_trajectory_planner_B.v_data, &joint_trajectory_planner_B.c_size_o,
    &joint_trajectory_planner_B.u_data, &joint_trajectory_planner_B.b_size_l);
  if (joint_trajectory_planner_B.trajPP.size > 1) {
    joint_trajectory_planner_B.breaks_size[0] = 1;
    joint_trajectory_planner_B.breaks_size[1] =
      joint_trajectory_planner_B.trajPP.data[4].breaks.size[1];
    joint_trajectory_planner_B.loop_ub = joint_trajectory_planner_B.trajPP.data
      [4].breaks.size[1];
    for (joint_trajectory_planner_B.oneDimCoeffs_size_tmp = 0;
         joint_trajectory_planner_B.oneDimCoeffs_size_tmp <
         joint_trajectory_planner_B.loop_ub;
         joint_trajectory_planner_B.oneDimCoeffs_size_tmp++) {
      joint_trajectory_planner_B.breaks_data[joint_trajectory_planner_B.oneDimCoeffs_size_tmp]
        = joint_trajectory_planner_B.trajPP.data[4]
        .breaks.data[joint_trajectory_planner_B.oneDimCoeffs_size_tmp];
    }

    joint_trajectory_planner_B.oneDimCoeffs_size_tmp = div_nzp_s32
      (joint_trajectory_planner_B.trajPP.data[4].coefs.size[0] *
       joint_trajectory_planner_B.trajPP.data[4].coefs.size[1] * 3, 3);
    joint_trajectory_planner_B.oneDimCoeffs_size[0] =
      joint_trajectory_planner_B.oneDimCoeffs_size_tmp;
    joint_trajectory_planner_B.loop_ub =
      joint_trajectory_planner_B.oneDimCoeffs_size_tmp * 3 - 1;
    for (joint_trajectory_planner_B.oneDimCoeffs_size_tmp = 0;
         joint_trajectory_planner_B.oneDimCoeffs_size_tmp <=
         joint_trajectory_planner_B.loop_ub;
         joint_trajectory_planner_B.oneDimCoeffs_size_tmp++) {
      joint_trajectory_planner_B.oneDimCoeffs_data[joint_trajectory_planner_B.oneDimCoeffs_size_tmp]
        = joint_trajectory_planner_B.trajPP.data[4]
        .coefs.data[joint_trajectory_planner_B.oneDimCoeffs_size_tmp];
    }
  } else {
    TrapVelTrajSys_extract1Dim_aoqo(joint_trajectory_planner_B.trajPP.data[0].
      breaks.data, joint_trajectory_planner_B.trajPP.data[0].breaks.size,
      joint_trajectory_planner_B.trajPP.data[0].coefs.data,
      joint_trajectory_planner_B.trajPP.data[0].coefs.size,
      joint_trajectory_planner_B.breaks_data,
      joint_trajectory_planner_B.breaks_size,
      joint_trajectory_planner_B.oneDimCoeffs_data,
      joint_trajectory_planner_B.oneDimCoeffs_size);
  }

  joint_trajectory_planner_B.evalCoeffs_size[0] =
    joint_trajectory_planner_B.oneDimCoeffs_size[0];
  joint_trajectory_planner_B.evalCoeffs_size[1] = 3;
  joint_trajectory_planner_B.loop_ub =
    joint_trajectory_planner_B.oneDimCoeffs_size[0] * 3 - 1;
  if (0 <= joint_trajectory_planner_B.loop_ub) {
    memset(&joint_trajectory_planner_B.evalCoeffs_data[0], 0,
           (joint_trajectory_planner_B.loop_ub + 1) * sizeof(real_T));
  }

  if (1 > joint_trajectory_planner_B.oneDimCoeffs_size[0]) {
    joint_trajectory_planner_B.f = 0;
  } else {
    joint_trajectory_planner_B.f = joint_trajectory_planner_B.oneDimCoeffs_size
      [0];
  }

  for (joint_trajectory_planner_B.oneDimCoeffs_size_tmp = 0;
       joint_trajectory_planner_B.oneDimCoeffs_size_tmp < 3;
       joint_trajectory_planner_B.oneDimCoeffs_size_tmp++) {
    for (joint_trajectory_planner_B.loop_ub = 0;
         joint_trajectory_planner_B.loop_ub < joint_trajectory_planner_B.f;
         joint_trajectory_planner_B.loop_ub++) {
      joint_trajectory_planner_B.evalCoeffs_data[joint_trajectory_planner_B.loop_ub
        + joint_trajectory_planner_B.evalCoeffs_size[0] *
        joint_trajectory_planner_B.oneDimCoeffs_size_tmp] =
        joint_trajectory_planner_B.oneDimCoeffs_data[joint_trajectory_planner_B.oneDimCoeffs_size
        [0] * joint_trajectory_planner_B.oneDimCoeffs_size_tmp +
        joint_trajectory_planner_B.loop_ub];
    }
  }

  TrapVelTrajSys_generate1DTrajec(joint_trajectory_planner_B.breaks_data,
    joint_trajectory_planner_B.breaks_size,
    joint_trajectory_planner_B.evalCoeffs_data,
    joint_trajectory_planner_B.evalCoeffs_size, time,
    &joint_trajectory_planner_B.db_data, &joint_trajectory_planner_B.d_size_b,
    &joint_trajectory_planner_B.cb_data, &joint_trajectory_planner_B.c_size_o,
    &joint_trajectory_planner_B.bb_data, &joint_trajectory_planner_B.b_size_l);
  if (joint_trajectory_planner_B.trajPP.size > 1) {
    joint_trajectory_planner_B.breaks_size[0] = 1;
    joint_trajectory_planner_B.breaks_size[1] =
      joint_trajectory_planner_B.trajPP.data[5].breaks.size[1];
    joint_trajectory_planner_B.loop_ub = joint_trajectory_planner_B.trajPP.data
      [5].breaks.size[1];
    for (joint_trajectory_planner_B.oneDimCoeffs_size_tmp = 0;
         joint_trajectory_planner_B.oneDimCoeffs_size_tmp <
         joint_trajectory_planner_B.loop_ub;
         joint_trajectory_planner_B.oneDimCoeffs_size_tmp++) {
      joint_trajectory_planner_B.breaks_data[joint_trajectory_planner_B.oneDimCoeffs_size_tmp]
        = joint_trajectory_planner_B.trajPP.data[5]
        .breaks.data[joint_trajectory_planner_B.oneDimCoeffs_size_tmp];
    }

    joint_trajectory_planner_B.oneDimCoeffs_size_tmp = div_nzp_s32
      (joint_trajectory_planner_B.trajPP.data[5].coefs.size[0] *
       joint_trajectory_planner_B.trajPP.data[5].coefs.size[1] * 3, 3);
    joint_trajectory_planner_B.oneDimCoeffs_size[0] =
      joint_trajectory_planner_B.oneDimCoeffs_size_tmp;
    joint_trajectory_planner_B.loop_ub =
      joint_trajectory_planner_B.oneDimCoeffs_size_tmp * 3 - 1;
    for (joint_trajectory_planner_B.oneDimCoeffs_size_tmp = 0;
         joint_trajectory_planner_B.oneDimCoeffs_size_tmp <=
         joint_trajectory_planner_B.loop_ub;
         joint_trajectory_planner_B.oneDimCoeffs_size_tmp++) {
      joint_trajectory_planner_B.oneDimCoeffs_data[joint_trajectory_planner_B.oneDimCoeffs_size_tmp]
        = joint_trajectory_planner_B.trajPP.data[5]
        .coefs.data[joint_trajectory_planner_B.oneDimCoeffs_size_tmp];
    }
  } else {
    TrapVelTrajSys_extract1Di_aoqov(joint_trajectory_planner_B.trajPP.data[0].
      breaks.data, joint_trajectory_planner_B.trajPP.data[0].breaks.size,
      joint_trajectory_planner_B.trajPP.data[0].coefs.data,
      joint_trajectory_planner_B.trajPP.data[0].coefs.size,
      joint_trajectory_planner_B.breaks_data,
      joint_trajectory_planner_B.breaks_size,
      joint_trajectory_planner_B.oneDimCoeffs_data,
      joint_trajectory_planner_B.oneDimCoeffs_size);
  }

  joint_trajectory_planner_B.evalCoeffs_size[0] =
    joint_trajectory_planner_B.oneDimCoeffs_size[0];
  joint_trajectory_planner_B.evalCoeffs_size[1] = 3;
  joint_trajectory_planner_B.loop_ub =
    joint_trajectory_planner_B.oneDimCoeffs_size[0] * 3 - 1;
  if (0 <= joint_trajectory_planner_B.loop_ub) {
    memset(&joint_trajectory_planner_B.evalCoeffs_data[0], 0,
           (joint_trajectory_planner_B.loop_ub + 1) * sizeof(real_T));
  }

  if (1 > joint_trajectory_planner_B.oneDimCoeffs_size[0]) {
    joint_trajectory_planner_B.f = 0;
  } else {
    joint_trajectory_planner_B.f = joint_trajectory_planner_B.oneDimCoeffs_size
      [0];
  }

  for (joint_trajectory_planner_B.oneDimCoeffs_size_tmp = 0;
       joint_trajectory_planner_B.oneDimCoeffs_size_tmp < 3;
       joint_trajectory_planner_B.oneDimCoeffs_size_tmp++) {
    for (joint_trajectory_planner_B.loop_ub = 0;
         joint_trajectory_planner_B.loop_ub < joint_trajectory_planner_B.f;
         joint_trajectory_planner_B.loop_ub++) {
      joint_trajectory_planner_B.evalCoeffs_data[joint_trajectory_planner_B.loop_ub
        + joint_trajectory_planner_B.evalCoeffs_size[0] *
        joint_trajectory_planner_B.oneDimCoeffs_size_tmp] =
        joint_trajectory_planner_B.oneDimCoeffs_data[joint_trajectory_planner_B.oneDimCoeffs_size
        [0] * joint_trajectory_planner_B.oneDimCoeffs_size_tmp +
        joint_trajectory_planner_B.loop_ub];
    }
  }

  TrapVelTrajSys_generate1DTrajec(joint_trajectory_planner_B.breaks_data,
    joint_trajectory_planner_B.breaks_size,
    joint_trajectory_planner_B.evalCoeffs_data,
    joint_trajectory_planner_B.evalCoeffs_size, time,
    &joint_trajectory_planner_B.jb_data, &joint_trajectory_planner_B.d_size_b,
    &joint_trajectory_planner_B.ib_data, &joint_trajectory_planner_B.c_size_o,
    &joint_trajectory_planner_B.hb_data, &joint_trajectory_planner_B.b_size_l);
  q[0] = joint_trajectory_planner_B.d_data_g;
  q[1] = joint_trajectory_planner_B.j_data;
  q[2] = joint_trajectory_planner_B.p_data;
  q[3] = joint_trajectory_planner_B.w_data;
  q[4] = joint_trajectory_planner_B.db_data;
  q[5] = joint_trajectory_planner_B.jb_data;
  qd[0] = joint_trajectory_planner_B.c_data_d;
  qd[1] = joint_trajectory_planner_B.i_data;
  qd[2] = joint_trajectory_planner_B.o_data_l;
  qd[3] = joint_trajectory_planner_B.v_data;
  qd[4] = joint_trajectory_planner_B.cb_data;
  qd[5] = joint_trajectory_planner_B.ib_data;
  qdd[0] = joint_trajectory_planner_B.b_data_j;
  qdd[1] = joint_trajectory_planner_B.h_data;
  qdd[2] = joint_trajectory_planner_B.n_data;
  qdd[3] = joint_trajectory_planner_B.u_data;
  qdd[4] = joint_trajectory_planner_B.bb_data;
  qdd[5] = joint_trajectory_planner_B.hb_data;
}

static void joint_trajectory_plan_ppval_aoq(const real_T pp_breaks[4], const
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

static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_GetP_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void joint_traject_matlabCodegenHa_n(ros_slros_internal_block_Subs_T *obj)
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

static void joint_trajec_SystemCore_setup_a(robotics_slcore_internal_bl_a_T *obj)
{
  obj->isInitialized = 1;
  obj->TunablePropsChanged = false;
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
  boolean_T exitg1;

  // MATLABSystem: '<S2>/Get Parameter'
  ParamGet_joint_trajectory_planner_26.get_parameter
    (&joint_trajectory_planner_B.value);

  // Outputs for Atomic SubSystem: '<Root>/Subscribe'
  // MATLABSystem: '<S6>/SourceBlock' incorporates:
  //   Inport: '<S10>/In1'

  b_varargout_1 = Sub_joint_trajectory_planner_15.getLatestMessage
    (&joint_trajectory_planner_B.b_varargout_2_g);

  // Outputs for Enabled SubSystem: '<S6>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S10>/Enable'

  if (b_varargout_1) {
    joint_trajectory_planner_B.In1 = joint_trajectory_planner_B.b_varargout_2_g;
  }

  // End of MATLABSystem: '<S6>/SourceBlock'
  // End of Outputs for SubSystem: '<S6>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe'

  // MATLAB Function: '<Root>/MATLAB Function1' incorporates:
  //   MATLABSystem: '<S2>/Get Parameter'

  joint_trajectory_planner_B.time = joint_trajectory_planner_B.In1.Clock_.Nsec /
    1.0E+9 + joint_trajectory_planner_B.In1.Clock_.Sec;
  joint_trajectory_planner_B.delayed_time = joint_trajectory_planner_B.time -
    joint_trajectory_planner_B.value;
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
  joint_trajectory_planner_B.signal2[0] = joint_trajectory_planner_B.value;

  // MATLABSystem: '<Root>/Polynomial Trajectory' incorporates:
  //   MATLAB Function: '<Root>/MATLAB Function1'

  b_varargout_1 = false;
  p = true;
  joint_trajectory_planner_B.b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (joint_trajectory_planner_B.b_k < 12)) {
    if (!(joint_trajectory_planner_DW.obj.VelocityBoundaryCondition[joint_trajectory_planner_B.b_k]
          ==
          joint_trajectory_planner_P.PolynomialTrajectory_VelocityBo[joint_trajectory_planner_B.b_k]))
    {
      p = false;
      exitg1 = true;
    } else {
      joint_trajectory_planner_B.b_k++;
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
    (&joint_trajectory_planner_B.b_k);

  // MATLABSystem: '<S2>/Get Parameter1'
  ParamGet_joint_trajectory_planner_27.get_parameter
    (&joint_trajectory_planner_B.value);

  // MATLABSystem: '<S2>/Get Parameter2'
  ParamGet_joint_trajectory_planner_28.get_parameter
    (&joint_trajectory_planner_B.value_l);

  // MATLAB Function: '<S2>/MATLAB Function' incorporates:
  //   MATLABSystem: '<S2>/Get Parameter1'
  //   MATLABSystem: '<S2>/Get Parameter2'

  joint_trajectory_planner_B.t_up = joint_trajectory_planner_B.value_l /
    joint_trajectory_planner_B.value;
  for (joint_trajectory_planner_B.i = 0; joint_trajectory_planner_B.i < 6;
       joint_trajectory_planner_B.i++) {
    joint_trajectory_planner_B.res_max_vel[joint_trajectory_planner_B.i] =
      joint_trajectory_planner_B.value_l;
    joint_trajectory_planner_B.d = fabs
      (joint_trajectory_planner_B.MatrixConcatenate[joint_trajectory_planner_B.i]
       - joint_trajectory_planner_B.MatrixConcatenate[joint_trajectory_planner_B.i
       + 6]);
    if (joint_trajectory_planner_B.d < joint_trajectory_planner_B.value_l *
        joint_trajectory_planner_B.t_up) {
      joint_trajectory_planner_B.res_max_vel[joint_trajectory_planner_B.i] =
        joint_trajectory_planner_B.d / joint_trajectory_planner_B.t_up;
    }
  }

  // End of MATLAB Function: '<S2>/MATLAB Function'

  // MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory' incorporates:
  //   MATLAB Function: '<Root>/MATLAB Function1'
  //   MATLABSystem: '<S2>/Get Parameter1'

  if (joint_trajectory_planner_DW.obj_a.TunablePropsChanged) {
    joint_trajectory_planner_DW.obj_a.TunablePropsChanged = false;
  }

  joint_t_TrapVelTrajSys_stepImpl(joint_trajectory_planner_B.delayed_time,
    joint_trajectory_planner_B.MatrixConcatenate,
    joint_trajectory_planner_B.res_max_vel, joint_trajectory_planner_B.value,
    joint_trajectory_planner_B.b_varargout_1,
    joint_trajectory_planner_B.b_varargout_2,
    joint_trajectory_planner_B.b_varargout_3);

  // MATLAB Function: '<Root>/MATLAB Function' incorporates:
  //   Constant: '<S1>/Constant'

  joint_trajectory_planner_B.msg = joint_trajectory_planner_P.Constant_Value;
  joint_trajectory_planner_B.msg.Velocities_SL_Info.CurrentLength = 6U;

  // Switch: '<Root>/Switch' incorporates:
  //   MATLABSystem: '<S2>/Get Parameter4'

  b_varargout_1 = (joint_trajectory_planner_B.b_k >
                   joint_trajectory_planner_P.Switch_Threshold);

  // MATLABSystem: '<Root>/Polynomial Trajectory' incorporates:
  //   MATLAB Function: '<Root>/MATLAB Function1'

  joint_trajectory_plan_ppval_aoq(joint_trajectory_planner_B.ppd_breaks,
    joint_trajectory_planner_B.ppd_coefs, joint_trajectory_planner_B.time,
    joint_trajectory_planner_B.res_max_vel);

  // MATLAB Function: '<Root>/MATLAB Function'
  for (joint_trajectory_planner_B.i = 0; joint_trajectory_planner_B.i < 6;
       joint_trajectory_planner_B.i++) {
    // Switch: '<Root>/Switch' incorporates:
    //   MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory'

    if (b_varargout_1) {
      joint_trajectory_planner_B.msg.Velocities[joint_trajectory_planner_B.i] =
        joint_trajectory_planner_B.res_max_vel[joint_trajectory_planner_B.i];
    } else {
      joint_trajectory_planner_B.msg.Velocities[joint_trajectory_planner_B.i] =
        joint_trajectory_planner_B.b_varargout_2[joint_trajectory_planner_B.i];
    }
  }

  joint_trajectory_planner_B.msg.Positions_SL_Info.CurrentLength = 6U;

  // Switch: '<Root>/Switch1' incorporates:
  //   MATLABSystem: '<S2>/Get Parameter4'

  b_varargout_1 = (joint_trajectory_planner_B.b_k >
                   joint_trajectory_planner_P.Switch1_Threshold);

  // MATLABSystem: '<Root>/Polynomial Trajectory' incorporates:
  //   MATLAB Function: '<Root>/MATLAB Function1'

  joint_trajectory_plan_ppval_aoq(joint_trajectory_planner_B.pp_breaks,
    joint_trajectory_planner_B.pp_coefs, joint_trajectory_planner_B.time,
    joint_trajectory_planner_B.res_max_vel);

  // MATLAB Function: '<Root>/MATLAB Function'
  for (joint_trajectory_planner_B.i = 0; joint_trajectory_planner_B.i < 6;
       joint_trajectory_planner_B.i++) {
    // Switch: '<Root>/Switch1' incorporates:
    //   MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory'

    if (b_varargout_1) {
      joint_trajectory_planner_B.msg.Positions[joint_trajectory_planner_B.i] =
        joint_trajectory_planner_B.res_max_vel[joint_trajectory_planner_B.i];
    } else {
      joint_trajectory_planner_B.msg.Positions[joint_trajectory_planner_B.i] =
        joint_trajectory_planner_B.b_varargout_1[joint_trajectory_planner_B.i];
    }
  }

  joint_trajectory_planner_B.msg.Accelerations_SL_Info.CurrentLength = 6U;

  // Switch: '<Root>/Switch2' incorporates:
  //   MATLABSystem: '<S2>/Get Parameter4'

  b_varargout_1 = (joint_trajectory_planner_B.b_k >
                   joint_trajectory_planner_P.Switch2_Threshold);

  // MATLABSystem: '<Root>/Polynomial Trajectory' incorporates:
  //   MATLAB Function: '<Root>/MATLAB Function1'

  joint_trajectory_plan_ppval_aoq(joint_trajectory_planner_B.ppdd_breaks,
    joint_trajectory_planner_B.ppdd_coefs, joint_trajectory_planner_B.time,
    joint_trajectory_planner_B.res_max_vel);

  // MATLAB Function: '<Root>/MATLAB Function'
  for (joint_trajectory_planner_B.i = 0; joint_trajectory_planner_B.i < 6;
       joint_trajectory_planner_B.i++) {
    // Switch: '<Root>/Switch2' incorporates:
    //   MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory'

    if (b_varargout_1) {
      joint_trajectory_planner_B.msg.Accelerations[joint_trajectory_planner_B.i]
        = joint_trajectory_planner_B.res_max_vel[joint_trajectory_planner_B.i];
    } else {
      joint_trajectory_planner_B.msg.Accelerations[joint_trajectory_planner_B.i]
        = joint_trajectory_planner_B.b_varargout_3[joint_trajectory_planner_B.i];
    }
  }

  // Outputs for Atomic SubSystem: '<Root>/Publish'
  // MATLABSystem: '<S5>/SinkBlock'
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
      '_', 'a', 'c', 'c' };

    static const char_T tmp_l[12] = { '/', 'm', 'a', 'x', '_', 'a', 'n', 'g',
      '_', 'v', 'e', 'l' };

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S6>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S10>/Out1'
    joint_trajectory_planner_B.In1 = joint_trajectory_planner_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S6>/Enabled Subsystem'

    // Start for MATLABSystem: '<S6>/SourceBlock'
    joint_trajectory_planner_DW.obj_c.matlabCodegenIsDeleted = false;
    joint_trajectory_planner_DW.obj_c.isInitialized = 1;
    for (i = 0; i < 6; i++) {
      tmp[i] = tmp_4[i];
    }

    tmp[6] = '\x00';
    Sub_joint_trajectory_planner_15.createSubscriber(tmp, 1);
    joint_trajectory_planner_DW.obj_c.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish'
    // Start for MATLABSystem: '<S5>/SinkBlock'
    joint_trajectory_planner_DW.obj_dw0.matlabCodegenIsDeleted = false;
    joint_trajectory_planner_DW.obj_dw0.isInitialized = 1;
    for (i = 0; i < 17; i++) {
      joint_trajectory_planner_B.cv[i] = tmp_5[i];
    }

    joint_trajectory_planner_B.cv[17] = '\x00';
    Pub_joint_trajectory_planner_5.createPublisher(joint_trajectory_planner_B.cv,
      1);
    joint_trajectory_planner_DW.obj_dw0.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S5>/SinkBlock'
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
    joint_trajectory_planner_DW.obj_ac.matlabCodegenIsDeleted = false;
    joint_trajectory_planner_DW.obj_ac.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      tmp_1[i] = tmp_c[i];
    }

    tmp_1[11] = '\x00';
    ParamGet_joint_trajectory_planner_39.initialize(tmp_1);
    ParamGet_joint_trajectory_planner_39.initialize_error_codes(0, 1, 2, 3);
    ParamGet_joint_trajectory_planner_39.set_initial_value(0.0);
    joint_trajectory_planner_DW.obj_ac.isSetupComplete = true;

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
    joint_trajec_SystemCore_setup_a(&joint_trajectory_planner_DW.obj);

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

    // Start for MATLABSystem: '<S2>/Get Parameter1'
    joint_trajectory_planner_DW.obj_l.matlabCodegenIsDeleted = false;
    joint_trajectory_planner_DW.obj_l.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      tmp_0[i] = tmp_k[i];
    }

    tmp_0[12] = '\x00';
    ParamGet_joint_trajectory_planner_27.initialize(tmp_0);
    ParamGet_joint_trajectory_planner_27.initialize_error_codes(0, 1, 2, 3);
    ParamGet_joint_trajectory_planner_27.set_initial_value(1.0);
    joint_trajectory_planner_DW.obj_l.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S2>/Get Parameter1'

    // Start for MATLABSystem: '<S2>/Get Parameter2'
    joint_trajectory_planner_DW.obj_o.matlabCodegenIsDeleted = false;
    joint_trajectory_planner_DW.obj_o.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      tmp_0[i] = tmp_l[i];
    }

    tmp_0[12] = '\x00';
    ParamGet_joint_trajectory_planner_28.initialize(tmp_0);
    ParamGet_joint_trajectory_planner_28.initialize_error_codes(0, 1, 2, 3);
    ParamGet_joint_trajectory_planner_28.set_initial_value(1.0);
    joint_trajectory_planner_DW.obj_o.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S2>/Get Parameter2'

    // Start for MATLABSystem: '<Root>/Trapezoidal Velocity Profile Trajectory'
    joint_trajectory_planner_DW.obj_a.PeakVelocity[0] = 2.0;
    joint_trajectory_planner_DW.obj_a.Acceleration[0] = 1.0;
    joint_trajectory_planner_DW.obj_a.PeakVelocity[1] = 2.0;
    joint_trajectory_planner_DW.obj_a.Acceleration[1] = 2.0;
    joint_trajectory_planner_DW.obj_a.isInitialized = 0;
    joint_trajecto_SystemCore_setup(&joint_trajectory_planner_DW.obj_a);
  }
}

// Model terminate function
void joint_trajectory_planner_terminate(void)
{
  // Terminate for MATLABSystem: '<S2>/Get Parameter'
  matlabCodegenHandle_matlabCodeg(&joint_trajectory_planner_DW.obj_g);

  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S6>/SourceBlock'
  joint_traject_matlabCodegenHa_n(&joint_trajectory_planner_DW.obj_c);

  // End of Terminate for SubSystem: '<Root>/Subscribe'

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
  matlabCodegenHandle_matlabCodeg(&joint_trajectory_planner_DW.obj_ac);

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

  // Terminate for MATLABSystem: '<S2>/Get Parameter1'
  matlabCodegenHandle_matlabCodeg(&joint_trajectory_planner_DW.obj_l);

  // Terminate for MATLABSystem: '<S2>/Get Parameter2'
  matlabCodegenHandle_matlabCodeg(&joint_trajectory_planner_DW.obj_o);

  // Terminate for Atomic SubSystem: '<Root>/Publish'
  // Terminate for MATLABSystem: '<S5>/SinkBlock'
  joint_traject_matlabCodegenHa_a(&joint_trajectory_planner_DW.obj_dw0);

  // End of Terminate for SubSystem: '<Root>/Publish'
}

//
// File trailer for generated code.
//
// [EOF]
//
