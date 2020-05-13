//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: kinsim_2link_planar.cpp
//
// Code generated for Simulink model 'kinsim_2link_planar'.
//
// Model version                  : 1.124
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Wed May 13 15:54:45 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "kinsim_2link_planar.h"
#include "kinsim_2link_planar_private.h"

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
static void kinsim_2link_pla_emxInit_real_T(emxArray_real_T_kinsim_2link__T
  **pEmxArray, int32_T numDimensions);
static void kinsim_2lin_emxInit_f_cell_wrap(emxArray_f_cell_wrap_kinsim_2_T
  **pEmxArray, int32_T numDimensions);
static void k_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_kinsim_2_T
  *emxArray, int32_T oldNumel);
static void kinsim_2link_pla_emxInit_char_T(emxArray_char_T_kinsim_2link__T
  **pEmxArray, int32_T numDimensions);
static void kinsim_emxEnsureCapacity_char_T(emxArray_char_T_kinsim_2link__T
  *emxArray, int32_T oldNumel);
static void ki_rigidBodyJoint_get_JointAxis(const
  c_rigidBodyJoint_kinsim_2link_T *obj, real_T ax[3]);
static void kinsim_2link_pla_emxFree_char_T(emxArray_char_T_kinsim_2link__T
  **pEmxArray);
static void RigidBodyTree_forwardKinematics(n_robotics_manip_internal_Rig_T *obj,
  const real_T qvec[6], emxArray_f_cell_wrap_kinsim_2_T *Ttree);
static void kinsim_emxEnsureCapacity_real_T(emxArray_real_T_kinsim_2link__T
  *emxArray, int32_T oldNumel);
static void kinsim_2link_pla_emxFree_real_T(emxArray_real_T_kinsim_2link__T
  **pEmxArray);
static void kinsim_2lin_emxFree_f_cell_wrap(emxArray_f_cell_wrap_kinsim_2_T
  **pEmxArray);
static void RigidBodyTree_geometricJacobian(n_robotics_manip_internal_Rig_T *obj,
  const real_T Q[6], emxArray_real_T_kinsim_2link__T *Jac);
static void rigidBodyJoint_get_JointAxis_l(const c_rigidBodyJoint_kinsim_2li_l_T
  *obj, real_T ax[3]);
static void RigidBodyTree_forwardKinemati_l(n_robotics_manip_internal_R_l_T *obj,
  const real_T qvec[6], emxArray_f_cell_wrap_kinsim_2_T *Ttree);
static void kinsim_2link_pl_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Positions[128], uint32_T *varargout_2_Positions_SL_Info_C,
  uint32_T *varargout_2_Positions_SL_Info_R, real_T varargout_2_Velocities[128],
  uint32_T *varargout_2_Velocities_SL_Info_, uint32_T
  *varargout_2_Velocities_SL_Inf_0, real_T varargout_2_Accelerations[128],
  uint32_T *varargout_2_Accelerations_SL_In, uint32_T
  *varargout_2_Accelerations_SL__0, real_T varargout_2_Effort[128], uint32_T
  *varargout_2_Effort_SL_Info_Curr, uint32_T *varargout_2_Effort_SL_Info_Rece,
  real_T *varargout_2_TimeFromStart_Sec, real_T *varargout_2_TimeFromStart_Nsec);
static boolean_T kinsim_2link_plana_anyNonFinite(const real_T x[16]);
static real_T kinsim_2link_plan_rt_hypotd_snf(real_T u0, real_T u1);
static real_T kinsim_2link_planar_xzlangeM(const creal_T x[16]);
static void kinsim_2link_planar_xzlascl(real_T cfrom, real_T cto, creal_T A[16]);
static real_T kinsim_2link_planar_xzlanhs(const creal_T A[16], int32_T ilo,
  int32_T ihi);
static void kinsim_2link_planar_xzlartg_a(const creal_T f, const creal_T g,
  real_T *cs, creal_T *sn);
static void kinsim_2link_planar_xzlartg(const creal_T f, const creal_T g, real_T
  *cs, creal_T *sn, creal_T *r);
static void kinsim_2link_planar_xzhgeqz(creal_T A[16], int32_T ilo, int32_T ihi,
  creal_T Z[16], int32_T *info, creal_T alpha1[4], creal_T beta1[4]);
static void kinsim_2link_planar_xztgevc(const creal_T A[16], creal_T V[16]);
static void kinsim_2link_planar_xzggev(creal_T A[16], int32_T *info, creal_T
  alpha1[4], creal_T beta1[4], creal_T V[16]);
static real_T kinsim_2link_planar_xnrm2(int32_T n, const real_T x[16], int32_T
  ix0);
static void kinsim_2link_planar_xzlarf(int32_T m, int32_T n, int32_T iv0, real_T
  tau, real_T C[16], int32_T ic0, real_T work[4]);
static void kinsim_2link_planar_xgehrd(real_T a[16], real_T tau[3]);
static real_T kinsim_2link_planar_xnrm2_l(int32_T n, const real_T x[3]);
static real_T kinsim_2link_planar_xzlarfg(int32_T n, real_T *alpha1, real_T x[3]);
static void kinsim_2link_planar_xdlanv2(real_T *a, real_T *b, real_T *c, real_T *
  d, real_T *rt1r, real_T *rt1i, real_T *rt2r, real_T *rt2i, real_T *cs, real_T *
  sn);
static void kinsim_2link_planar_xrot(int32_T n, real_T x[16], int32_T ix0,
  int32_T iy0, real_T c, real_T s);
static void kinsim_2link_planar_xrot_g(int32_T n, real_T x[16], int32_T ix0,
  int32_T iy0, real_T c, real_T s);
static int32_T kinsim_2link_planar_eml_dlahqr(real_T h[16], real_T z[16]);
static void kinsim_2link_planar_eig(const real_T A[16], creal_T V[16], creal_T
  D[4]);
static void matlabCodegenHandle_matla_li5a1(ros_slros_internal_block_GetP_T *obj);
static void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_kinsim_2link_T
  *pStruct);
static void emxFreeStruct_m_robotics_manip_(m_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_n_robotics_manip_(n_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct);
static void emxFreeStruct_l_robotics_manip_(l_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_c_rigidBodyJoint1(c_rigidBodyJoint_kinsim_2li_l_T
  *pStruct);
static void emxFreeStruct_m_robotics_mani_l(m_robotics_manip_internal_R_l_T
  *pStruct);
static void emxFreeStruct_n_robotics_mani_l(n_robotics_manip_internal_R_l_T
  *pStruct);
static void emxFreeStruct_robotics_slmani_l(robotics_slmanip_internal_b_l_T
  *pStruct);
static void emxFreeStruct_l_robotics_mani_l(l_robotics_manip_internal_R_l_T
  *pStruct);
static void matlabCodegenHandle_matlab_li5a(ros_slros_internal_block_Subs_T *obj);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Publ_T *obj);
static void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_kinsim_2link_T
  *pStruct);
static void emxInitStruct_m_robotics_manip_(m_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_n_robotics_manip_(n_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct);
static void emxInitStruct_l_robotics_manip_(l_robotics_manip_internal_Rig_T
  *pStruct);
static l_robotics_manip_internal_Rig_T *kinsim_2lin_RigidBody_RigidBody
  (l_robotics_manip_internal_Rig_T *obj);
static l_robotics_manip_internal_Rig_T *kinsim_2l_RigidBody_RigidBody_l
  (l_robotics_manip_internal_Rig_T *obj);
static l_robotics_manip_internal_Rig_T *kinsim_2_RigidBody_RigidBody_li
  (l_robotics_manip_internal_Rig_T *obj);
static l_robotics_manip_internal_Rig_T *kinsim__RigidBody_RigidBody_li5
  (l_robotics_manip_internal_Rig_T *obj);
static l_robotics_manip_internal_Rig_T *kinsim_RigidBody_RigidBody_li5a
  (l_robotics_manip_internal_Rig_T *obj);
static l_robotics_manip_internal_Rig_T *kinsi_RigidBody_RigidBody_li5a1
  (l_robotics_manip_internal_Rig_T *obj);
static m_robotics_manip_internal_Rig_T *kins_RigidBody_RigidBody_li5a1l
  (m_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *kin_RigidBodyTree_RigidBodyTree
  (n_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Rig_T *iobj_0,
   l_robotics_manip_internal_Rig_T *iobj_1, l_robotics_manip_internal_Rig_T
   *iobj_2, l_robotics_manip_internal_Rig_T *iobj_3,
   l_robotics_manip_internal_Rig_T *iobj_4, l_robotics_manip_internal_Rig_T
   *iobj_5);
static void emxInitStruct_c_rigidBodyJoint1(c_rigidBodyJoint_kinsim_2li_l_T
  *pStruct);
static void emxInitStruct_m_robotics_mani_l(m_robotics_manip_internal_R_l_T
  *pStruct);
static void emxInitStruct_n_robotics_mani_l(n_robotics_manip_internal_R_l_T
  *pStruct);
static void emxInitStruct_robotics_slmani_l(robotics_slmanip_internal_b_l_T
  *pStruct);
static void emxInitStruct_l_robotics_mani_l(l_robotics_manip_internal_R_l_T
  *pStruct);
static l_robotics_manip_internal_R_l_T *kin_RigidBody_RigidBody_li5a1ls
  (l_robotics_manip_internal_R_l_T *obj);
static l_robotics_manip_internal_R_l_T *ki_RigidBody_RigidBody_li5a1lsh
  (l_robotics_manip_internal_R_l_T *obj);
static l_robotics_manip_internal_R_l_T *k_RigidBody_RigidBody_li5a1lshy
  (l_robotics_manip_internal_R_l_T *obj);
static n_robotics_manip_internal_R_l_T *k_RigidBodyTree_RigidBodyTree_l
  (n_robotics_manip_internal_R_l_T *obj, l_robotics_manip_internal_R_l_T *iobj_0,
   l_robotics_manip_internal_R_l_T *iobj_1, l_robotics_manip_internal_R_l_T
   *iobj_2, l_robotics_manip_internal_R_l_T *iobj_3,
   l_robotics_manip_internal_R_l_T *iobj_4, l_robotics_manip_internal_R_l_T
   *iobj_5);
int32_T div_nzp_s32(int32_T numerator, int32_T denominator)
{
  uint32_T tempAbsQuotient;
  tempAbsQuotient = (numerator < 0 ? ~static_cast<uint32_T>(numerator) + 1U :
                     static_cast<uint32_T>(numerator)) / (denominator < 0 ? ~
    static_cast<uint32_T>(denominator) + 1U : static_cast<uint32_T>(denominator));
  return (numerator < 0) != (denominator < 0) ? -static_cast<int32_T>
    (tempAbsQuotient) : static_cast<int32_T>(tempAbsQuotient);
}

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

static void kinsim_2link_pla_emxInit_real_T(emxArray_real_T_kinsim_2link__T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_real_T_kinsim_2link__T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_real_T_kinsim_2link__T *)malloc(sizeof
    (emxArray_real_T_kinsim_2link__T));
  emxArray = *pEmxArray;
  emxArray->data = (real_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void kinsim_2lin_emxInit_f_cell_wrap(emxArray_f_cell_wrap_kinsim_2_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_f_cell_wrap_kinsim_2_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_f_cell_wrap_kinsim_2_T *)malloc(sizeof
    (emxArray_f_cell_wrap_kinsim_2_T));
  emxArray = *pEmxArray;
  emxArray->data = (f_cell_wrap_kinsim_2link_plan_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void k_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_kinsim_2_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  kinsim_2link_planar_B.newNumel_c = 1;
  for (kinsim_2link_planar_B.i_ax = 0; kinsim_2link_planar_B.i_ax <
       emxArray->numDimensions; kinsim_2link_planar_B.i_ax++) {
    kinsim_2link_planar_B.newNumel_c *= emxArray->
      size[kinsim_2link_planar_B.i_ax];
  }

  if (kinsim_2link_planar_B.newNumel_c > emxArray->allocatedSize) {
    kinsim_2link_planar_B.i_ax = emxArray->allocatedSize;
    if (kinsim_2link_planar_B.i_ax < 16) {
      kinsim_2link_planar_B.i_ax = 16;
    }

    while (kinsim_2link_planar_B.i_ax < kinsim_2link_planar_B.newNumel_c) {
      if (kinsim_2link_planar_B.i_ax > 1073741823) {
        kinsim_2link_planar_B.i_ax = MAX_int32_T;
      } else {
        kinsim_2link_planar_B.i_ax <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(kinsim_2link_planar_B.i_ax), sizeof
                     (f_cell_wrap_kinsim_2link_plan_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(f_cell_wrap_kinsim_2link_plan_T)
             * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (f_cell_wrap_kinsim_2link_plan_T *)newData;
    emxArray->allocatedSize = kinsim_2link_planar_B.i_ax;
    emxArray->canFreeData = true;
  }
}

static void kinsim_2link_pla_emxInit_char_T(emxArray_char_T_kinsim_2link__T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_char_T_kinsim_2link__T *emxArray;
  *pEmxArray = (emxArray_char_T_kinsim_2link__T *)malloc(sizeof
    (emxArray_char_T_kinsim_2link__T));
  emxArray = *pEmxArray;
  emxArray->data = (char_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (kinsim_2link_planar_B.i_d = 0; kinsim_2link_planar_B.i_d < numDimensions;
       kinsim_2link_planar_B.i_d++) {
    emxArray->size[kinsim_2link_planar_B.i_d] = 0;
  }
}

static void kinsim_emxEnsureCapacity_char_T(emxArray_char_T_kinsim_2link__T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  kinsim_2link_planar_B.newNumel = 1;
  for (kinsim_2link_planar_B.i_e = 0; kinsim_2link_planar_B.i_e <
       emxArray->numDimensions; kinsim_2link_planar_B.i_e++) {
    kinsim_2link_planar_B.newNumel *= emxArray->size[kinsim_2link_planar_B.i_e];
  }

  if (kinsim_2link_planar_B.newNumel > emxArray->allocatedSize) {
    kinsim_2link_planar_B.i_e = emxArray->allocatedSize;
    if (kinsim_2link_planar_B.i_e < 16) {
      kinsim_2link_planar_B.i_e = 16;
    }

    while (kinsim_2link_planar_B.i_e < kinsim_2link_planar_B.newNumel) {
      if (kinsim_2link_planar_B.i_e > 1073741823) {
        kinsim_2link_planar_B.i_e = MAX_int32_T;
      } else {
        kinsim_2link_planar_B.i_e <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(kinsim_2link_planar_B.i_e), sizeof
                     (char_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(char_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (char_T *)newData;
    emxArray->allocatedSize = kinsim_2link_planar_B.i_e;
    emxArray->canFreeData = true;
  }
}

static void ki_rigidBodyJoint_get_JointAxis(const
  c_rigidBodyJoint_kinsim_2link_T *obj, real_T ax[3])
{
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  boolean_T guard1 = false;
  int32_T exitg1;
  for (kinsim_2link_planar_B.b_kstr_o = 0; kinsim_2link_planar_B.b_kstr_o < 8;
       kinsim_2link_planar_B.b_kstr_o++) {
    kinsim_2link_planar_B.b_l[kinsim_2link_planar_B.b_kstr_o] =
      tmp[kinsim_2link_planar_B.b_kstr_o];
  }

  kinsim_2link_planar_B.b_bool_o = false;
  if (obj->Type->size[1] == 8) {
    kinsim_2link_planar_B.b_kstr_o = 1;
    do {
      exitg1 = 0;
      if (kinsim_2link_planar_B.b_kstr_o - 1 < 8) {
        kinsim_2link_planar_B.kstr_e = kinsim_2link_planar_B.b_kstr_o - 1;
        if (obj->Type->data[kinsim_2link_planar_B.kstr_e] !=
            kinsim_2link_planar_B.b_l[kinsim_2link_planar_B.kstr_e]) {
          exitg1 = 1;
        } else {
          kinsim_2link_planar_B.b_kstr_o++;
        }
      } else {
        kinsim_2link_planar_B.b_bool_o = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (kinsim_2link_planar_B.b_bool_o) {
    guard1 = true;
  } else {
    for (kinsim_2link_planar_B.b_kstr_o = 0; kinsim_2link_planar_B.b_kstr_o < 9;
         kinsim_2link_planar_B.b_kstr_o++) {
      kinsim_2link_planar_B.b_o[kinsim_2link_planar_B.b_kstr_o] =
        tmp_0[kinsim_2link_planar_B.b_kstr_o];
    }

    kinsim_2link_planar_B.b_bool_o = false;
    if (obj->Type->size[1] == 9) {
      kinsim_2link_planar_B.b_kstr_o = 1;
      do {
        exitg1 = 0;
        if (kinsim_2link_planar_B.b_kstr_o - 1 < 9) {
          kinsim_2link_planar_B.kstr_e = kinsim_2link_planar_B.b_kstr_o - 1;
          if (obj->Type->data[kinsim_2link_planar_B.kstr_e] !=
              kinsim_2link_planar_B.b_o[kinsim_2link_planar_B.kstr_e]) {
            exitg1 = 1;
          } else {
            kinsim_2link_planar_B.b_kstr_o++;
          }
        } else {
          kinsim_2link_planar_B.b_bool_o = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (kinsim_2link_planar_B.b_bool_o) {
      guard1 = true;
    } else {
      ax[0] = (rtNaN);
      ax[1] = (rtNaN);
      ax[2] = (rtNaN);
    }
  }

  if (guard1) {
    ax[0] = obj->JointAxisInternal[0];
    ax[1] = obj->JointAxisInternal[1];
    ax[2] = obj->JointAxisInternal[2];
  }
}

static void kinsim_2link_pla_emxFree_char_T(emxArray_char_T_kinsim_2link__T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_char_T_kinsim_2link__T *)NULL) {
    if (((*pEmxArray)->data != (char_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_char_T_kinsim_2link__T *)NULL;
  }
}

static void RigidBodyTree_forwardKinematics(n_robotics_manip_internal_Rig_T *obj,
  const real_T qvec[6], emxArray_f_cell_wrap_kinsim_2_T *Ttree)
{
  l_robotics_manip_internal_Rig_T *body;
  emxArray_char_T_kinsim_2link__T *switch_expression;
  static const int8_T tmp[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  kinsim_2link_planar_B.n = obj->NumBodies;
  for (kinsim_2link_planar_B.b_kstr = 0; kinsim_2link_planar_B.b_kstr < 16;
       kinsim_2link_planar_B.b_kstr++) {
    kinsim_2link_planar_B.c_f1[kinsim_2link_planar_B.b_kstr] =
      tmp[kinsim_2link_planar_B.b_kstr];
  }

  kinsim_2link_planar_B.b_kstr = Ttree->size[0] * Ttree->size[1];
  Ttree->size[0] = 1;
  kinsim_2link_planar_B.e_f = static_cast<int32_T>(kinsim_2link_planar_B.n);
  Ttree->size[1] = kinsim_2link_planar_B.e_f;
  k_emxEnsureCapacity_f_cell_wrap(Ttree, kinsim_2link_planar_B.b_kstr);
  if (kinsim_2link_planar_B.e_f != 0) {
    kinsim_2link_planar_B.ntilecols = kinsim_2link_planar_B.e_f - 1;
    if (0 <= kinsim_2link_planar_B.ntilecols) {
      memcpy(&kinsim_2link_planar_B.expl_temp.f1[0],
             &kinsim_2link_planar_B.c_f1[0], sizeof(real_T) << 4U);
    }

    for (kinsim_2link_planar_B.b_jtilecol = 0; kinsim_2link_planar_B.b_jtilecol <=
         kinsim_2link_planar_B.ntilecols; kinsim_2link_planar_B.b_jtilecol++) {
      Ttree->data[kinsim_2link_planar_B.b_jtilecol] =
        kinsim_2link_planar_B.expl_temp;
    }
  }

  kinsim_2link_planar_B.k = 1.0;
  kinsim_2link_planar_B.ntilecols = static_cast<int32_T>(kinsim_2link_planar_B.n)
    - 1;
  kinsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  if (0 <= kinsim_2link_planar_B.ntilecols) {
    for (kinsim_2link_planar_B.b_kstr = 0; kinsim_2link_planar_B.b_kstr < 5;
         kinsim_2link_planar_B.b_kstr++) {
      kinsim_2link_planar_B.b_f[kinsim_2link_planar_B.b_kstr] =
        tmp_0[kinsim_2link_planar_B.b_kstr];
    }
  }

  for (kinsim_2link_planar_B.b_jtilecol = 0; kinsim_2link_planar_B.b_jtilecol <=
       kinsim_2link_planar_B.ntilecols; kinsim_2link_planar_B.b_jtilecol++) {
    body = obj->Bodies[kinsim_2link_planar_B.b_jtilecol];
    kinsim_2link_planar_B.n = body->JointInternal.PositionNumber;
    kinsim_2link_planar_B.n += kinsim_2link_planar_B.k;
    if (kinsim_2link_planar_B.k > kinsim_2link_planar_B.n - 1.0) {
      kinsim_2link_planar_B.e_f = 0;
      kinsim_2link_planar_B.d_c = 0;
    } else {
      kinsim_2link_planar_B.e_f = static_cast<int32_T>(kinsim_2link_planar_B.k)
        - 1;
      kinsim_2link_planar_B.d_c = static_cast<int32_T>(kinsim_2link_planar_B.n -
        1.0);
    }

    kinsim_2link_planar_B.b_kstr = switch_expression->size[0] *
      switch_expression->size[1];
    switch_expression->size[0] = 1;
    switch_expression->size[1] = body->JointInternal.Type->size[1];
    kinsim_emxEnsureCapacity_char_T(switch_expression,
      kinsim_2link_planar_B.b_kstr);
    kinsim_2link_planar_B.loop_ub_p = body->JointInternal.Type->size[0] *
      body->JointInternal.Type->size[1] - 1;
    for (kinsim_2link_planar_B.b_kstr = 0; kinsim_2link_planar_B.b_kstr <=
         kinsim_2link_planar_B.loop_ub_p; kinsim_2link_planar_B.b_kstr++) {
      switch_expression->data[kinsim_2link_planar_B.b_kstr] =
        body->JointInternal.Type->data[kinsim_2link_planar_B.b_kstr];
    }

    kinsim_2link_planar_B.b_bool_i = false;
    if (switch_expression->size[1] == 5) {
      kinsim_2link_planar_B.b_kstr = 1;
      do {
        exitg1 = 0;
        if (kinsim_2link_planar_B.b_kstr - 1 < 5) {
          kinsim_2link_planar_B.loop_ub_p = kinsim_2link_planar_B.b_kstr - 1;
          if (switch_expression->data[kinsim_2link_planar_B.loop_ub_p] !=
              kinsim_2link_planar_B.b_f[kinsim_2link_planar_B.loop_ub_p]) {
            exitg1 = 1;
          } else {
            kinsim_2link_planar_B.b_kstr++;
          }
        } else {
          kinsim_2link_planar_B.b_bool_i = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (kinsim_2link_planar_B.b_bool_i) {
      kinsim_2link_planar_B.b_kstr = 0;
    } else {
      for (kinsim_2link_planar_B.b_kstr = 0; kinsim_2link_planar_B.b_kstr < 8;
           kinsim_2link_planar_B.b_kstr++) {
        kinsim_2link_planar_B.b_bs[kinsim_2link_planar_B.b_kstr] =
          tmp_1[kinsim_2link_planar_B.b_kstr];
      }

      kinsim_2link_planar_B.b_bool_i = false;
      if (switch_expression->size[1] == 8) {
        kinsim_2link_planar_B.b_kstr = 1;
        do {
          exitg1 = 0;
          if (kinsim_2link_planar_B.b_kstr - 1 < 8) {
            kinsim_2link_planar_B.loop_ub_p = kinsim_2link_planar_B.b_kstr - 1;
            if (switch_expression->data[kinsim_2link_planar_B.loop_ub_p] !=
                kinsim_2link_planar_B.b_bs[kinsim_2link_planar_B.loop_ub_p]) {
              exitg1 = 1;
            } else {
              kinsim_2link_planar_B.b_kstr++;
            }
          } else {
            kinsim_2link_planar_B.b_bool_i = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (kinsim_2link_planar_B.b_bool_i) {
        kinsim_2link_planar_B.b_kstr = 1;
      } else {
        kinsim_2link_planar_B.b_kstr = -1;
      }
    }

    switch (kinsim_2link_planar_B.b_kstr) {
     case 0:
      memset(&kinsim_2link_planar_B.c_f1[0], 0, sizeof(real_T) << 4U);
      kinsim_2link_planar_B.c_f1[0] = 1.0;
      kinsim_2link_planar_B.c_f1[5] = 1.0;
      kinsim_2link_planar_B.c_f1[10] = 1.0;
      kinsim_2link_planar_B.c_f1[15] = 1.0;
      break;

     case 1:
      ki_rigidBodyJoint_get_JointAxis(&body->JointInternal,
        kinsim_2link_planar_B.v);
      kinsim_2link_planar_B.d_c -= kinsim_2link_planar_B.e_f;
      for (kinsim_2link_planar_B.b_kstr = 0; kinsim_2link_planar_B.b_kstr <
           kinsim_2link_planar_B.d_c; kinsim_2link_planar_B.b_kstr++) {
        kinsim_2link_planar_B.e_data[kinsim_2link_planar_B.b_kstr] =
          kinsim_2link_planar_B.e_f + kinsim_2link_planar_B.b_kstr;
      }

      kinsim_2link_planar_B.result_data[0] = kinsim_2link_planar_B.v[0];
      kinsim_2link_planar_B.result_data[1] = kinsim_2link_planar_B.v[1];
      kinsim_2link_planar_B.result_data[2] = kinsim_2link_planar_B.v[2];
      if (0 <= (kinsim_2link_planar_B.d_c != 0) - 1) {
        kinsim_2link_planar_B.result_data[3] =
          qvec[kinsim_2link_planar_B.e_data[0]];
      }

      kinsim_2link_planar_B.k = 1.0 / sqrt((kinsim_2link_planar_B.result_data[0]
        * kinsim_2link_planar_B.result_data[0] +
        kinsim_2link_planar_B.result_data[1] *
        kinsim_2link_planar_B.result_data[1]) +
        kinsim_2link_planar_B.result_data[2] *
        kinsim_2link_planar_B.result_data[2]);
      kinsim_2link_planar_B.v[0] = kinsim_2link_planar_B.result_data[0] *
        kinsim_2link_planar_B.k;
      kinsim_2link_planar_B.v[1] = kinsim_2link_planar_B.result_data[1] *
        kinsim_2link_planar_B.k;
      kinsim_2link_planar_B.v[2] = kinsim_2link_planar_B.result_data[2] *
        kinsim_2link_planar_B.k;
      kinsim_2link_planar_B.k = cos(kinsim_2link_planar_B.result_data[3]);
      kinsim_2link_planar_B.sth = sin(kinsim_2link_planar_B.result_data[3]);
      kinsim_2link_planar_B.tempR[0] = kinsim_2link_planar_B.v[0] *
        kinsim_2link_planar_B.v[0] * (1.0 - kinsim_2link_planar_B.k) +
        kinsim_2link_planar_B.k;
      kinsim_2link_planar_B.tempR_tmp = kinsim_2link_planar_B.v[1] *
        kinsim_2link_planar_B.v[0] * (1.0 - kinsim_2link_planar_B.k);
      kinsim_2link_planar_B.tempR_tmp_e = kinsim_2link_planar_B.v[2] *
        kinsim_2link_planar_B.sth;
      kinsim_2link_planar_B.tempR[1] = kinsim_2link_planar_B.tempR_tmp -
        kinsim_2link_planar_B.tempR_tmp_e;
      kinsim_2link_planar_B.tempR_tmp_b = kinsim_2link_planar_B.v[2] *
        kinsim_2link_planar_B.v[0] * (1.0 - kinsim_2link_planar_B.k);
      kinsim_2link_planar_B.tempR_tmp_j = kinsim_2link_planar_B.v[1] *
        kinsim_2link_planar_B.sth;
      kinsim_2link_planar_B.tempR[2] = kinsim_2link_planar_B.tempR_tmp_b +
        kinsim_2link_planar_B.tempR_tmp_j;
      kinsim_2link_planar_B.tempR[3] = kinsim_2link_planar_B.tempR_tmp +
        kinsim_2link_planar_B.tempR_tmp_e;
      kinsim_2link_planar_B.tempR[4] = kinsim_2link_planar_B.v[1] *
        kinsim_2link_planar_B.v[1] * (1.0 - kinsim_2link_planar_B.k) +
        kinsim_2link_planar_B.k;
      kinsim_2link_planar_B.tempR_tmp = kinsim_2link_planar_B.v[2] *
        kinsim_2link_planar_B.v[1] * (1.0 - kinsim_2link_planar_B.k);
      kinsim_2link_planar_B.tempR_tmp_e = kinsim_2link_planar_B.v[0] *
        kinsim_2link_planar_B.sth;
      kinsim_2link_planar_B.tempR[5] = kinsim_2link_planar_B.tempR_tmp -
        kinsim_2link_planar_B.tempR_tmp_e;
      kinsim_2link_planar_B.tempR[6] = kinsim_2link_planar_B.tempR_tmp_b -
        kinsim_2link_planar_B.tempR_tmp_j;
      kinsim_2link_planar_B.tempR[7] = kinsim_2link_planar_B.tempR_tmp +
        kinsim_2link_planar_B.tempR_tmp_e;
      kinsim_2link_planar_B.tempR[8] = kinsim_2link_planar_B.v[2] *
        kinsim_2link_planar_B.v[2] * (1.0 - kinsim_2link_planar_B.k) +
        kinsim_2link_planar_B.k;
      for (kinsim_2link_planar_B.b_kstr = 0; kinsim_2link_planar_B.b_kstr < 3;
           kinsim_2link_planar_B.b_kstr++) {
        kinsim_2link_planar_B.e_f = kinsim_2link_planar_B.b_kstr + 1;
        kinsim_2link_planar_B.R_n[kinsim_2link_planar_B.e_f - 1] =
          kinsim_2link_planar_B.tempR[(kinsim_2link_planar_B.e_f - 1) * 3];
        kinsim_2link_planar_B.e_f = kinsim_2link_planar_B.b_kstr + 1;
        kinsim_2link_planar_B.R_n[kinsim_2link_planar_B.e_f + 2] =
          kinsim_2link_planar_B.tempR[(kinsim_2link_planar_B.e_f - 1) * 3 + 1];
        kinsim_2link_planar_B.e_f = kinsim_2link_planar_B.b_kstr + 1;
        kinsim_2link_planar_B.R_n[kinsim_2link_planar_B.e_f + 5] =
          kinsim_2link_planar_B.tempR[(kinsim_2link_planar_B.e_f - 1) * 3 + 2];
      }

      memset(&kinsim_2link_planar_B.c_f1[0], 0, sizeof(real_T) << 4U);
      for (kinsim_2link_planar_B.b_kstr = 0; kinsim_2link_planar_B.b_kstr < 3;
           kinsim_2link_planar_B.b_kstr++) {
        kinsim_2link_planar_B.d_c = kinsim_2link_planar_B.b_kstr << 2;
        kinsim_2link_planar_B.c_f1[kinsim_2link_planar_B.d_c] =
          kinsim_2link_planar_B.R_n[3 * kinsim_2link_planar_B.b_kstr];
        kinsim_2link_planar_B.c_f1[kinsim_2link_planar_B.d_c + 1] =
          kinsim_2link_planar_B.R_n[3 * kinsim_2link_planar_B.b_kstr + 1];
        kinsim_2link_planar_B.c_f1[kinsim_2link_planar_B.d_c + 2] =
          kinsim_2link_planar_B.R_n[3 * kinsim_2link_planar_B.b_kstr + 2];
      }

      kinsim_2link_planar_B.c_f1[15] = 1.0;
      break;

     default:
      ki_rigidBodyJoint_get_JointAxis(&body->JointInternal,
        kinsim_2link_planar_B.v);
      memset(&kinsim_2link_planar_B.tempR[0], 0, 9U * sizeof(real_T));
      kinsim_2link_planar_B.tempR[0] = 1.0;
      kinsim_2link_planar_B.tempR[4] = 1.0;
      kinsim_2link_planar_B.tempR[8] = 1.0;
      for (kinsim_2link_planar_B.b_kstr = 0; kinsim_2link_planar_B.b_kstr < 3;
           kinsim_2link_planar_B.b_kstr++) {
        kinsim_2link_planar_B.d_c = kinsim_2link_planar_B.b_kstr << 2;
        kinsim_2link_planar_B.c_f1[kinsim_2link_planar_B.d_c] =
          kinsim_2link_planar_B.tempR[3 * kinsim_2link_planar_B.b_kstr];
        kinsim_2link_planar_B.c_f1[kinsim_2link_planar_B.d_c + 1] =
          kinsim_2link_planar_B.tempR[3 * kinsim_2link_planar_B.b_kstr + 1];
        kinsim_2link_planar_B.c_f1[kinsim_2link_planar_B.d_c + 2] =
          kinsim_2link_planar_B.tempR[3 * kinsim_2link_planar_B.b_kstr + 2];
        kinsim_2link_planar_B.c_f1[kinsim_2link_planar_B.b_kstr + 12] =
          kinsim_2link_planar_B.v[kinsim_2link_planar_B.b_kstr] *
          qvec[kinsim_2link_planar_B.e_f];
      }

      kinsim_2link_planar_B.c_f1[3] = 0.0;
      kinsim_2link_planar_B.c_f1[7] = 0.0;
      kinsim_2link_planar_B.c_f1[11] = 0.0;
      kinsim_2link_planar_B.c_f1[15] = 1.0;
      break;
    }

    for (kinsim_2link_planar_B.b_kstr = 0; kinsim_2link_planar_B.b_kstr < 16;
         kinsim_2link_planar_B.b_kstr++) {
      kinsim_2link_planar_B.a[kinsim_2link_planar_B.b_kstr] =
        body->JointInternal.JointToParentTransform[kinsim_2link_planar_B.b_kstr];
    }

    for (kinsim_2link_planar_B.b_kstr = 0; kinsim_2link_planar_B.b_kstr < 16;
         kinsim_2link_planar_B.b_kstr++) {
      kinsim_2link_planar_B.b[kinsim_2link_planar_B.b_kstr] =
        body->JointInternal.ChildToJointTransform[kinsim_2link_planar_B.b_kstr];
    }

    for (kinsim_2link_planar_B.b_kstr = 0; kinsim_2link_planar_B.b_kstr < 4;
         kinsim_2link_planar_B.b_kstr++) {
      for (kinsim_2link_planar_B.e_f = 0; kinsim_2link_planar_B.e_f < 4;
           kinsim_2link_planar_B.e_f++) {
        kinsim_2link_planar_B.d_c = kinsim_2link_planar_B.e_f << 2;
        kinsim_2link_planar_B.loop_ub_p = kinsim_2link_planar_B.b_kstr +
          kinsim_2link_planar_B.d_c;
        kinsim_2link_planar_B.a_k[kinsim_2link_planar_B.loop_ub_p] = 0.0;
        kinsim_2link_planar_B.a_k[kinsim_2link_planar_B.loop_ub_p] +=
          kinsim_2link_planar_B.c_f1[kinsim_2link_planar_B.d_c] *
          kinsim_2link_planar_B.a[kinsim_2link_planar_B.b_kstr];
        kinsim_2link_planar_B.a_k[kinsim_2link_planar_B.loop_ub_p] +=
          kinsim_2link_planar_B.c_f1[kinsim_2link_planar_B.d_c + 1] *
          kinsim_2link_planar_B.a[kinsim_2link_planar_B.b_kstr + 4];
        kinsim_2link_planar_B.a_k[kinsim_2link_planar_B.loop_ub_p] +=
          kinsim_2link_planar_B.c_f1[kinsim_2link_planar_B.d_c + 2] *
          kinsim_2link_planar_B.a[kinsim_2link_planar_B.b_kstr + 8];
        kinsim_2link_planar_B.a_k[kinsim_2link_planar_B.loop_ub_p] +=
          kinsim_2link_planar_B.c_f1[kinsim_2link_planar_B.d_c + 3] *
          kinsim_2link_planar_B.a[kinsim_2link_planar_B.b_kstr + 12];
      }

      for (kinsim_2link_planar_B.e_f = 0; kinsim_2link_planar_B.e_f < 4;
           kinsim_2link_planar_B.e_f++) {
        kinsim_2link_planar_B.d_c = kinsim_2link_planar_B.e_f << 2;
        kinsim_2link_planar_B.loop_ub_p = kinsim_2link_planar_B.b_kstr +
          kinsim_2link_planar_B.d_c;
        Ttree->data[kinsim_2link_planar_B.b_jtilecol]
          .f1[kinsim_2link_planar_B.loop_ub_p] = 0.0;
        Ttree->data[kinsim_2link_planar_B.b_jtilecol]
          .f1[kinsim_2link_planar_B.loop_ub_p] +=
          kinsim_2link_planar_B.b[kinsim_2link_planar_B.d_c] *
          kinsim_2link_planar_B.a_k[kinsim_2link_planar_B.b_kstr];
        Ttree->data[kinsim_2link_planar_B.b_jtilecol]
          .f1[kinsim_2link_planar_B.loop_ub_p] +=
          kinsim_2link_planar_B.b[kinsim_2link_planar_B.d_c + 1] *
          kinsim_2link_planar_B.a_k[kinsim_2link_planar_B.b_kstr + 4];
        Ttree->data[kinsim_2link_planar_B.b_jtilecol]
          .f1[kinsim_2link_planar_B.loop_ub_p] +=
          kinsim_2link_planar_B.b[kinsim_2link_planar_B.d_c + 2] *
          kinsim_2link_planar_B.a_k[kinsim_2link_planar_B.b_kstr + 8];
        Ttree->data[kinsim_2link_planar_B.b_jtilecol]
          .f1[kinsim_2link_planar_B.loop_ub_p] +=
          kinsim_2link_planar_B.b[kinsim_2link_planar_B.d_c + 3] *
          kinsim_2link_planar_B.a_k[kinsim_2link_planar_B.b_kstr + 12];
      }
    }

    kinsim_2link_planar_B.k = kinsim_2link_planar_B.n;
    if (body->ParentIndex > 0.0) {
      for (kinsim_2link_planar_B.b_kstr = 0; kinsim_2link_planar_B.b_kstr < 16;
           kinsim_2link_planar_B.b_kstr++) {
        kinsim_2link_planar_B.a[kinsim_2link_planar_B.b_kstr] = Ttree->data[
          static_cast<int32_T>(body->ParentIndex) - 1]
          .f1[kinsim_2link_planar_B.b_kstr];
      }

      for (kinsim_2link_planar_B.b_kstr = 0; kinsim_2link_planar_B.b_kstr < 4;
           kinsim_2link_planar_B.b_kstr++) {
        for (kinsim_2link_planar_B.e_f = 0; kinsim_2link_planar_B.e_f < 4;
             kinsim_2link_planar_B.e_f++) {
          kinsim_2link_planar_B.d_c = kinsim_2link_planar_B.e_f << 2;
          kinsim_2link_planar_B.loop_ub_p = kinsim_2link_planar_B.b_kstr +
            kinsim_2link_planar_B.d_c;
          kinsim_2link_planar_B.a_k[kinsim_2link_planar_B.loop_ub_p] = 0.0;
          kinsim_2link_planar_B.a_k[kinsim_2link_planar_B.loop_ub_p] +=
            Ttree->data[kinsim_2link_planar_B.b_jtilecol]
            .f1[kinsim_2link_planar_B.d_c] *
            kinsim_2link_planar_B.a[kinsim_2link_planar_B.b_kstr];
          kinsim_2link_planar_B.a_k[kinsim_2link_planar_B.loop_ub_p] +=
            Ttree->data[kinsim_2link_planar_B.b_jtilecol]
            .f1[kinsim_2link_planar_B.d_c + 1] *
            kinsim_2link_planar_B.a[kinsim_2link_planar_B.b_kstr + 4];
          kinsim_2link_planar_B.a_k[kinsim_2link_planar_B.loop_ub_p] +=
            Ttree->data[kinsim_2link_planar_B.b_jtilecol]
            .f1[kinsim_2link_planar_B.d_c + 2] *
            kinsim_2link_planar_B.a[kinsim_2link_planar_B.b_kstr + 8];
          kinsim_2link_planar_B.a_k[kinsim_2link_planar_B.loop_ub_p] +=
            Ttree->data[kinsim_2link_planar_B.b_jtilecol]
            .f1[kinsim_2link_planar_B.d_c + 3] *
            kinsim_2link_planar_B.a[kinsim_2link_planar_B.b_kstr + 12];
        }
      }

      for (kinsim_2link_planar_B.b_kstr = 0; kinsim_2link_planar_B.b_kstr < 16;
           kinsim_2link_planar_B.b_kstr++) {
        Ttree->data[kinsim_2link_planar_B.b_jtilecol]
          .f1[kinsim_2link_planar_B.b_kstr] =
          kinsim_2link_planar_B.a_k[kinsim_2link_planar_B.b_kstr];
      }
    }
  }

  kinsim_2link_pla_emxFree_char_T(&switch_expression);
}

static void kinsim_emxEnsureCapacity_real_T(emxArray_real_T_kinsim_2link__T
  *emxArray, int32_T oldNumel)
{
  int32_T newNumel;
  int32_T i;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(i), sizeof(real_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(real_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (real_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void kinsim_2link_pla_emxFree_real_T(emxArray_real_T_kinsim_2link__T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T_kinsim_2link__T *)NULL) {
    if (((*pEmxArray)->data != (real_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_real_T_kinsim_2link__T *)NULL;
  }
}

static void kinsim_2lin_emxFree_f_cell_wrap(emxArray_f_cell_wrap_kinsim_2_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_f_cell_wrap_kinsim_2_T *)NULL) {
    if (((*pEmxArray)->data != (f_cell_wrap_kinsim_2link_plan_T *)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_f_cell_wrap_kinsim_2_T *)NULL;
  }
}

static void RigidBodyTree_geometricJacobian(n_robotics_manip_internal_Rig_T *obj,
  const real_T Q[6], emxArray_real_T_kinsim_2link__T *Jac)
{
  emxArray_f_cell_wrap_kinsim_2_T *Ttree;
  l_robotics_manip_internal_Rig_T *body;
  emxArray_real_T_kinsim_2link__T *JacSlice;
  emxArray_char_T_kinsim_2link__T *bname;
  emxArray_real_T_kinsim_2link__T *b;
  static const char_T tmp[6] = { 'l', 'i', 'n', 'k', '_', '6' };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  int32_T exitg1;
  boolean_T exitg2;
  kinsim_2lin_emxInit_f_cell_wrap(&Ttree, 2);
  RigidBodyTree_forwardKinematics(obj, Q, Ttree);
  kinsim_2link_planar_B.ret_m = Jac->size[0] * Jac->size[1];
  Jac->size[0] = 6;
  Jac->size[1] = static_cast<int32_T>(obj->VelocityNumber);
  kinsim_emxEnsureCapacity_real_T(Jac, kinsim_2link_planar_B.ret_m);
  kinsim_2link_planar_B.loop_ub_m = 6 * static_cast<int32_T>(obj->VelocityNumber)
    - 1;
  for (kinsim_2link_planar_B.ret_m = 0; kinsim_2link_planar_B.ret_m <=
       kinsim_2link_planar_B.loop_ub_m; kinsim_2link_planar_B.ret_m++) {
    Jac->data[kinsim_2link_planar_B.ret_m] = 0.0;
  }

  for (kinsim_2link_planar_B.ret_m = 0; kinsim_2link_planar_B.ret_m < 6;
       kinsim_2link_planar_B.ret_m++) {
    kinsim_2link_planar_B.chainmask[kinsim_2link_planar_B.ret_m] = 0;
  }

  kinsim_2link_pla_emxInit_char_T(&bname, 2);
  kinsim_2link_planar_B.ret_m = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj->Base.NameInternal->size[1];
  kinsim_emxEnsureCapacity_char_T(bname, kinsim_2link_planar_B.ret_m);
  kinsim_2link_planar_B.loop_ub_m = obj->Base.NameInternal->size[0] *
    obj->Base.NameInternal->size[1] - 1;
  for (kinsim_2link_planar_B.ret_m = 0; kinsim_2link_planar_B.ret_m <=
       kinsim_2link_planar_B.loop_ub_m; kinsim_2link_planar_B.ret_m++) {
    bname->data[kinsim_2link_planar_B.ret_m] = obj->Base.NameInternal->
      data[kinsim_2link_planar_B.ret_m];
  }

  for (kinsim_2link_planar_B.ret_m = 0; kinsim_2link_planar_B.ret_m < 6;
       kinsim_2link_planar_B.ret_m++) {
    kinsim_2link_planar_B.a_i[kinsim_2link_planar_B.ret_m] =
      tmp[kinsim_2link_planar_B.ret_m];
  }

  kinsim_2link_planar_B.b_bool_f = false;
  if (6 == bname->size[1]) {
    kinsim_2link_planar_B.ret_m = 1;
    do {
      exitg1 = 0;
      if (kinsim_2link_planar_B.ret_m - 1 < 6) {
        kinsim_2link_planar_B.kstr = kinsim_2link_planar_B.ret_m - 1;
        if (kinsim_2link_planar_B.a_i[kinsim_2link_planar_B.kstr] != bname->
            data[kinsim_2link_planar_B.kstr]) {
          exitg1 = 1;
        } else {
          kinsim_2link_planar_B.ret_m++;
        }
      } else {
        kinsim_2link_planar_B.b_bool_f = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (kinsim_2link_planar_B.b_bool_f) {
    memset(&kinsim_2link_planar_B.T2inv[0], 0, sizeof(real_T) << 4U);
    kinsim_2link_planar_B.T2inv[0] = 1.0;
    kinsim_2link_planar_B.T2inv[5] = 1.0;
    kinsim_2link_planar_B.T2inv[10] = 1.0;
    kinsim_2link_planar_B.T2inv[15] = 1.0;
    memset(&kinsim_2link_planar_B.T2_c[0], 0, sizeof(real_T) << 4U);
    kinsim_2link_planar_B.T2_c[0] = 1.0;
    kinsim_2link_planar_B.T2_c[5] = 1.0;
    kinsim_2link_planar_B.T2_c[10] = 1.0;
    kinsim_2link_planar_B.T2_c[15] = 1.0;
  } else {
    kinsim_2link_planar_B.endeffectorIndex = -1.0;
    kinsim_2link_planar_B.ret_m = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = obj->Base.NameInternal->size[1];
    kinsim_emxEnsureCapacity_char_T(bname, kinsim_2link_planar_B.ret_m);
    kinsim_2link_planar_B.loop_ub_m = obj->Base.NameInternal->size[0] *
      obj->Base.NameInternal->size[1] - 1;
    for (kinsim_2link_planar_B.ret_m = 0; kinsim_2link_planar_B.ret_m <=
         kinsim_2link_planar_B.loop_ub_m; kinsim_2link_planar_B.ret_m++) {
      bname->data[kinsim_2link_planar_B.ret_m] = obj->Base.NameInternal->
        data[kinsim_2link_planar_B.ret_m];
    }

    for (kinsim_2link_planar_B.ret_m = 0; kinsim_2link_planar_B.ret_m < 6;
         kinsim_2link_planar_B.ret_m++) {
      kinsim_2link_planar_B.a_i[kinsim_2link_planar_B.ret_m] =
        tmp[kinsim_2link_planar_B.ret_m];
    }

    kinsim_2link_planar_B.b_bool_f = false;
    if (bname->size[1] == 6) {
      kinsim_2link_planar_B.ret_m = 1;
      do {
        exitg1 = 0;
        if (kinsim_2link_planar_B.ret_m - 1 < 6) {
          kinsim_2link_planar_B.kstr = kinsim_2link_planar_B.ret_m - 1;
          if (bname->data[kinsim_2link_planar_B.kstr] !=
              kinsim_2link_planar_B.a_i[kinsim_2link_planar_B.kstr]) {
            exitg1 = 1;
          } else {
            kinsim_2link_planar_B.ret_m++;
          }
        } else {
          kinsim_2link_planar_B.b_bool_f = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (kinsim_2link_planar_B.b_bool_f) {
      kinsim_2link_planar_B.endeffectorIndex = 0.0;
    } else {
      kinsim_2link_planar_B.idx_idx_1 = obj->NumBodies;
      kinsim_2link_planar_B.b_i_o = 0;
      exitg2 = false;
      while ((!exitg2) && (kinsim_2link_planar_B.b_i_o <= static_cast<int32_T>
                           (kinsim_2link_planar_B.idx_idx_1) - 1)) {
        body = obj->Bodies[kinsim_2link_planar_B.b_i_o];
        for (kinsim_2link_planar_B.ret_m = 0; kinsim_2link_planar_B.ret_m < 6;
             kinsim_2link_planar_B.ret_m++) {
          kinsim_2link_planar_B.bname_f[kinsim_2link_planar_B.ret_m] =
            body->NameInternal[kinsim_2link_planar_B.ret_m];
        }

        for (kinsim_2link_planar_B.ret_m = 0; kinsim_2link_planar_B.ret_m < 6;
             kinsim_2link_planar_B.ret_m++) {
          kinsim_2link_planar_B.a_i[kinsim_2link_planar_B.ret_m] =
            tmp[kinsim_2link_planar_B.ret_m];
        }

        kinsim_2link_planar_B.ret_m = memcmp(&kinsim_2link_planar_B.bname_f[0],
          &kinsim_2link_planar_B.a_i[0], 6);
        if (kinsim_2link_planar_B.ret_m == 0) {
          kinsim_2link_planar_B.endeffectorIndex = static_cast<real_T>
            (kinsim_2link_planar_B.b_i_o) + 1.0;
          exitg2 = true;
        } else {
          kinsim_2link_planar_B.b_i_o++;
        }
      }
    }

    kinsim_2link_planar_B.b_i_o = static_cast<int32_T>
      (kinsim_2link_planar_B.endeffectorIndex) - 1;
    body = obj->Bodies[kinsim_2link_planar_B.b_i_o];
    for (kinsim_2link_planar_B.ret_m = 0; kinsim_2link_planar_B.ret_m < 16;
         kinsim_2link_planar_B.ret_m++) {
      kinsim_2link_planar_B.T2_c[kinsim_2link_planar_B.ret_m] = Ttree->
        data[kinsim_2link_planar_B.b_i_o].f1[kinsim_2link_planar_B.ret_m];
    }

    for (kinsim_2link_planar_B.ret_m = 0; kinsim_2link_planar_B.ret_m < 3;
         kinsim_2link_planar_B.ret_m++) {
      kinsim_2link_planar_B.R_g1[3 * kinsim_2link_planar_B.ret_m] = Ttree->
        data[kinsim_2link_planar_B.b_i_o].f1[kinsim_2link_planar_B.ret_m];
      kinsim_2link_planar_B.R_g1[3 * kinsim_2link_planar_B.ret_m + 1] =
        Ttree->data[kinsim_2link_planar_B.b_i_o].f1[kinsim_2link_planar_B.ret_m
        + 4];
      kinsim_2link_planar_B.R_g1[3 * kinsim_2link_planar_B.ret_m + 2] =
        Ttree->data[kinsim_2link_planar_B.b_i_o].f1[kinsim_2link_planar_B.ret_m
        + 8];
    }

    for (kinsim_2link_planar_B.ret_m = 0; kinsim_2link_planar_B.ret_m < 9;
         kinsim_2link_planar_B.ret_m++) {
      kinsim_2link_planar_B.R_m[kinsim_2link_planar_B.ret_m] =
        -kinsim_2link_planar_B.R_g1[kinsim_2link_planar_B.ret_m];
    }

    for (kinsim_2link_planar_B.ret_m = 0; kinsim_2link_planar_B.ret_m < 3;
         kinsim_2link_planar_B.ret_m++) {
      kinsim_2link_planar_B.endeffectorIndex = Ttree->
        data[kinsim_2link_planar_B.b_i_o].f1[12] *
        kinsim_2link_planar_B.R_m[kinsim_2link_planar_B.ret_m];
      kinsim_2link_planar_B.loop_ub_m = kinsim_2link_planar_B.ret_m << 2;
      kinsim_2link_planar_B.T2inv[kinsim_2link_planar_B.loop_ub_m] =
        kinsim_2link_planar_B.R_g1[3 * kinsim_2link_planar_B.ret_m];
      kinsim_2link_planar_B.endeffectorIndex +=
        kinsim_2link_planar_B.R_m[kinsim_2link_planar_B.ret_m + 3] * Ttree->
        data[kinsim_2link_planar_B.b_i_o].f1[13];
      kinsim_2link_planar_B.T2inv[kinsim_2link_planar_B.loop_ub_m + 1] =
        kinsim_2link_planar_B.R_g1[3 * kinsim_2link_planar_B.ret_m + 1];
      kinsim_2link_planar_B.endeffectorIndex +=
        kinsim_2link_planar_B.R_m[kinsim_2link_planar_B.ret_m + 6] * Ttree->
        data[kinsim_2link_planar_B.b_i_o].f1[14];
      kinsim_2link_planar_B.T2inv[kinsim_2link_planar_B.loop_ub_m + 2] =
        kinsim_2link_planar_B.R_g1[3 * kinsim_2link_planar_B.ret_m + 2];
      kinsim_2link_planar_B.T2inv[kinsim_2link_planar_B.ret_m + 12] =
        kinsim_2link_planar_B.endeffectorIndex;
    }

    kinsim_2link_planar_B.T2inv[3] = 0.0;
    kinsim_2link_planar_B.T2inv[7] = 0.0;
    kinsim_2link_planar_B.T2inv[11] = 0.0;
    kinsim_2link_planar_B.T2inv[15] = 1.0;
    kinsim_2link_planar_B.chainmask[kinsim_2link_planar_B.b_i_o] = 1;
    while (body->ParentIndex > 0.0) {
      body = obj->Bodies[static_cast<int32_T>(body->ParentIndex) - 1];
      kinsim_2link_planar_B.chainmask[static_cast<int32_T>(body->Index) - 1] = 1;
    }
  }

  kinsim_2link_planar_B.idx_idx_1 = obj->NumBodies;
  kinsim_2link_planar_B.c_c = static_cast<int32_T>
    (kinsim_2link_planar_B.idx_idx_1) - 1;
  kinsim_2link_pla_emxInit_real_T(&JacSlice, 2);
  kinsim_2link_pla_emxInit_real_T(&b, 2);
  if (0 <= kinsim_2link_planar_B.c_c) {
    for (kinsim_2link_planar_B.ret_m = 0; kinsim_2link_planar_B.ret_m < 5;
         kinsim_2link_planar_B.ret_m++) {
      kinsim_2link_planar_B.b_iz[kinsim_2link_planar_B.ret_m] =
        tmp_0[kinsim_2link_planar_B.ret_m];
    }
  }

  for (kinsim_2link_planar_B.b_i_o = 0; kinsim_2link_planar_B.b_i_o <=
       kinsim_2link_planar_B.c_c; kinsim_2link_planar_B.b_i_o++) {
    body = obj->Bodies[kinsim_2link_planar_B.b_i_o];
    kinsim_2link_planar_B.ret_m = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = body->JointInternal.Type->size[1];
    kinsim_emxEnsureCapacity_char_T(bname, kinsim_2link_planar_B.ret_m);
    kinsim_2link_planar_B.loop_ub_m = body->JointInternal.Type->size[0] *
      body->JointInternal.Type->size[1] - 1;
    for (kinsim_2link_planar_B.ret_m = 0; kinsim_2link_planar_B.ret_m <=
         kinsim_2link_planar_B.loop_ub_m; kinsim_2link_planar_B.ret_m++) {
      bname->data[kinsim_2link_planar_B.ret_m] = body->JointInternal.Type->
        data[kinsim_2link_planar_B.ret_m];
    }

    kinsim_2link_planar_B.b_bool_f = false;
    if (bname->size[1] == 5) {
      kinsim_2link_planar_B.ret_m = 1;
      do {
        exitg1 = 0;
        if (kinsim_2link_planar_B.ret_m - 1 < 5) {
          kinsim_2link_planar_B.kstr = kinsim_2link_planar_B.ret_m - 1;
          if (bname->data[kinsim_2link_planar_B.kstr] !=
              kinsim_2link_planar_B.b_iz[kinsim_2link_planar_B.kstr]) {
            exitg1 = 1;
          } else {
            kinsim_2link_planar_B.ret_m++;
          }
        } else {
          kinsim_2link_planar_B.b_bool_f = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if ((!kinsim_2link_planar_B.b_bool_f) &&
        (kinsim_2link_planar_B.chainmask[kinsim_2link_planar_B.b_i_o] != 0)) {
      for (kinsim_2link_planar_B.ret_m = 0; kinsim_2link_planar_B.ret_m < 16;
           kinsim_2link_planar_B.ret_m++) {
        kinsim_2link_planar_B.T1[kinsim_2link_planar_B.ret_m] = Ttree->data[
          static_cast<int32_T>(body->Index) - 1].f1[kinsim_2link_planar_B.ret_m];
      }

      for (kinsim_2link_planar_B.ret_m = 0; kinsim_2link_planar_B.ret_m < 16;
           kinsim_2link_planar_B.ret_m++) {
        kinsim_2link_planar_B.Tdh[kinsim_2link_planar_B.ret_m] =
          body->JointInternal.ChildToJointTransform[kinsim_2link_planar_B.ret_m];
      }

      for (kinsim_2link_planar_B.ret_m = 0; kinsim_2link_planar_B.ret_m < 3;
           kinsim_2link_planar_B.ret_m++) {
        kinsim_2link_planar_B.R_g1[3 * kinsim_2link_planar_B.ret_m] =
          kinsim_2link_planar_B.Tdh[kinsim_2link_planar_B.ret_m];
        kinsim_2link_planar_B.R_g1[3 * kinsim_2link_planar_B.ret_m + 1] =
          kinsim_2link_planar_B.Tdh[kinsim_2link_planar_B.ret_m + 4];
        kinsim_2link_planar_B.R_g1[3 * kinsim_2link_planar_B.ret_m + 2] =
          kinsim_2link_planar_B.Tdh[kinsim_2link_planar_B.ret_m + 8];
      }

      for (kinsim_2link_planar_B.ret_m = 0; kinsim_2link_planar_B.ret_m < 9;
           kinsim_2link_planar_B.ret_m++) {
        kinsim_2link_planar_B.R_m[kinsim_2link_planar_B.ret_m] =
          -kinsim_2link_planar_B.R_g1[kinsim_2link_planar_B.ret_m];
      }

      for (kinsim_2link_planar_B.ret_m = 0; kinsim_2link_planar_B.ret_m < 3;
           kinsim_2link_planar_B.ret_m++) {
        kinsim_2link_planar_B.R_l[kinsim_2link_planar_B.ret_m] =
          kinsim_2link_planar_B.R_m[kinsim_2link_planar_B.ret_m + 6] *
          kinsim_2link_planar_B.Tdh[14] +
          (kinsim_2link_planar_B.R_m[kinsim_2link_planar_B.ret_m + 3] *
           kinsim_2link_planar_B.Tdh[13] +
           kinsim_2link_planar_B.R_m[kinsim_2link_planar_B.ret_m] *
           kinsim_2link_planar_B.Tdh[12]);
      }

      for (kinsim_2link_planar_B.ret_m = 0; kinsim_2link_planar_B.ret_m < 4;
           kinsim_2link_planar_B.ret_m++) {
        for (kinsim_2link_planar_B.kstr = 0; kinsim_2link_planar_B.kstr < 4;
             kinsim_2link_planar_B.kstr++) {
          kinsim_2link_planar_B.n_l = kinsim_2link_planar_B.kstr << 2;
          kinsim_2link_planar_B.loop_ub_m = kinsim_2link_planar_B.ret_m +
            kinsim_2link_planar_B.n_l;
          kinsim_2link_planar_B.Tdh[kinsim_2link_planar_B.loop_ub_m] = 0.0;
          kinsim_2link_planar_B.Tdh[kinsim_2link_planar_B.loop_ub_m] +=
            kinsim_2link_planar_B.T1[kinsim_2link_planar_B.n_l] *
            kinsim_2link_planar_B.T2inv[kinsim_2link_planar_B.ret_m];
          kinsim_2link_planar_B.Tdh[kinsim_2link_planar_B.loop_ub_m] +=
            kinsim_2link_planar_B.T1[kinsim_2link_planar_B.n_l + 1] *
            kinsim_2link_planar_B.T2inv[kinsim_2link_planar_B.ret_m + 4];
          kinsim_2link_planar_B.Tdh[kinsim_2link_planar_B.loop_ub_m] +=
            kinsim_2link_planar_B.T1[kinsim_2link_planar_B.n_l + 2] *
            kinsim_2link_planar_B.T2inv[kinsim_2link_planar_B.ret_m + 8];
          kinsim_2link_planar_B.Tdh[kinsim_2link_planar_B.loop_ub_m] +=
            kinsim_2link_planar_B.T1[kinsim_2link_planar_B.n_l + 3] *
            kinsim_2link_planar_B.T2inv[kinsim_2link_planar_B.ret_m + 12];
        }
      }

      for (kinsim_2link_planar_B.ret_m = 0; kinsim_2link_planar_B.ret_m < 3;
           kinsim_2link_planar_B.ret_m++) {
        kinsim_2link_planar_B.kstr = kinsim_2link_planar_B.ret_m << 2;
        kinsim_2link_planar_B.T1[kinsim_2link_planar_B.kstr] =
          kinsim_2link_planar_B.R_g1[3 * kinsim_2link_planar_B.ret_m];
        kinsim_2link_planar_B.T1[kinsim_2link_planar_B.kstr + 1] =
          kinsim_2link_planar_B.R_g1[3 * kinsim_2link_planar_B.ret_m + 1];
        kinsim_2link_planar_B.T1[kinsim_2link_planar_B.kstr + 2] =
          kinsim_2link_planar_B.R_g1[3 * kinsim_2link_planar_B.ret_m + 2];
        kinsim_2link_planar_B.T1[kinsim_2link_planar_B.ret_m + 12] =
          kinsim_2link_planar_B.R_l[kinsim_2link_planar_B.ret_m];
      }

      kinsim_2link_planar_B.T1[3] = 0.0;
      kinsim_2link_planar_B.T1[7] = 0.0;
      kinsim_2link_planar_B.T1[11] = 0.0;
      kinsim_2link_planar_B.T1[15] = 1.0;
      for (kinsim_2link_planar_B.ret_m = 0; kinsim_2link_planar_B.ret_m < 4;
           kinsim_2link_planar_B.ret_m++) {
        for (kinsim_2link_planar_B.kstr = 0; kinsim_2link_planar_B.kstr < 4;
             kinsim_2link_planar_B.kstr++) {
          kinsim_2link_planar_B.loop_ub_m = kinsim_2link_planar_B.kstr << 2;
          kinsim_2link_planar_B.n_l = kinsim_2link_planar_B.ret_m +
            kinsim_2link_planar_B.loop_ub_m;
          kinsim_2link_planar_B.T[kinsim_2link_planar_B.n_l] = 0.0;
          kinsim_2link_planar_B.T[kinsim_2link_planar_B.n_l] +=
            kinsim_2link_planar_B.T1[kinsim_2link_planar_B.loop_ub_m] *
            kinsim_2link_planar_B.Tdh[kinsim_2link_planar_B.ret_m];
          kinsim_2link_planar_B.T[kinsim_2link_planar_B.n_l] +=
            kinsim_2link_planar_B.T1[kinsim_2link_planar_B.loop_ub_m + 1] *
            kinsim_2link_planar_B.Tdh[kinsim_2link_planar_B.ret_m + 4];
          kinsim_2link_planar_B.T[kinsim_2link_planar_B.n_l] +=
            kinsim_2link_planar_B.T1[kinsim_2link_planar_B.loop_ub_m + 2] *
            kinsim_2link_planar_B.Tdh[kinsim_2link_planar_B.ret_m + 8];
          kinsim_2link_planar_B.T[kinsim_2link_planar_B.n_l] +=
            kinsim_2link_planar_B.T1[kinsim_2link_planar_B.loop_ub_m + 3] *
            kinsim_2link_planar_B.Tdh[kinsim_2link_planar_B.ret_m + 12];
        }
      }

      kinsim_2link_planar_B.endeffectorIndex = obj->
        PositionDoFMap[kinsim_2link_planar_B.b_i_o];
      kinsim_2link_planar_B.idx_idx_1 = obj->
        PositionDoFMap[kinsim_2link_planar_B.b_i_o + 6];
      kinsim_2link_planar_B.R_g1[0] = 0.0;
      kinsim_2link_planar_B.R_g1[3] = -kinsim_2link_planar_B.T[14];
      kinsim_2link_planar_B.R_g1[6] = kinsim_2link_planar_B.T[13];
      kinsim_2link_planar_B.R_g1[1] = kinsim_2link_planar_B.T[14];
      kinsim_2link_planar_B.R_g1[4] = 0.0;
      kinsim_2link_planar_B.R_g1[7] = -kinsim_2link_planar_B.T[12];
      kinsim_2link_planar_B.R_g1[2] = -kinsim_2link_planar_B.T[13];
      kinsim_2link_planar_B.R_g1[5] = kinsim_2link_planar_B.T[12];
      kinsim_2link_planar_B.R_g1[8] = 0.0;
      for (kinsim_2link_planar_B.ret_m = 0; kinsim_2link_planar_B.ret_m < 3;
           kinsim_2link_planar_B.ret_m++) {
        for (kinsim_2link_planar_B.kstr = 0; kinsim_2link_planar_B.kstr < 3;
             kinsim_2link_planar_B.kstr++) {
          kinsim_2link_planar_B.loop_ub_m = kinsim_2link_planar_B.ret_m + 3 *
            kinsim_2link_planar_B.kstr;
          kinsim_2link_planar_B.R_m[kinsim_2link_planar_B.loop_ub_m] = 0.0;
          kinsim_2link_planar_B.n_l = kinsim_2link_planar_B.kstr << 2;
          kinsim_2link_planar_B.R_m[kinsim_2link_planar_B.loop_ub_m] +=
            kinsim_2link_planar_B.T[kinsim_2link_planar_B.n_l] *
            kinsim_2link_planar_B.R_g1[kinsim_2link_planar_B.ret_m];
          kinsim_2link_planar_B.R_m[kinsim_2link_planar_B.loop_ub_m] +=
            kinsim_2link_planar_B.T[kinsim_2link_planar_B.n_l + 1] *
            kinsim_2link_planar_B.R_g1[kinsim_2link_planar_B.ret_m + 3];
          kinsim_2link_planar_B.R_m[kinsim_2link_planar_B.loop_ub_m] +=
            kinsim_2link_planar_B.T[kinsim_2link_planar_B.n_l + 2] *
            kinsim_2link_planar_B.R_g1[kinsim_2link_planar_B.ret_m + 6];
          kinsim_2link_planar_B.X[kinsim_2link_planar_B.kstr + 6 *
            kinsim_2link_planar_B.ret_m] = kinsim_2link_planar_B.T
            [(kinsim_2link_planar_B.ret_m << 2) + kinsim_2link_planar_B.kstr];
          kinsim_2link_planar_B.X[kinsim_2link_planar_B.kstr + 6 *
            (kinsim_2link_planar_B.ret_m + 3)] = 0.0;
        }
      }

      for (kinsim_2link_planar_B.ret_m = 0; kinsim_2link_planar_B.ret_m < 3;
           kinsim_2link_planar_B.ret_m++) {
        kinsim_2link_planar_B.X[6 * kinsim_2link_planar_B.ret_m + 3] =
          kinsim_2link_planar_B.R_m[3 * kinsim_2link_planar_B.ret_m];
        kinsim_2link_planar_B.kstr = kinsim_2link_planar_B.ret_m << 2;
        kinsim_2link_planar_B.loop_ub_m = 6 * (kinsim_2link_planar_B.ret_m + 3);
        kinsim_2link_planar_B.X[kinsim_2link_planar_B.loop_ub_m + 3] =
          kinsim_2link_planar_B.T[kinsim_2link_planar_B.kstr];
        kinsim_2link_planar_B.X[6 * kinsim_2link_planar_B.ret_m + 4] =
          kinsim_2link_planar_B.R_m[3 * kinsim_2link_planar_B.ret_m + 1];
        kinsim_2link_planar_B.X[kinsim_2link_planar_B.loop_ub_m + 4] =
          kinsim_2link_planar_B.T[kinsim_2link_planar_B.kstr + 1];
        kinsim_2link_planar_B.X[6 * kinsim_2link_planar_B.ret_m + 5] =
          kinsim_2link_planar_B.R_m[3 * kinsim_2link_planar_B.ret_m + 2];
        kinsim_2link_planar_B.X[kinsim_2link_planar_B.loop_ub_m + 5] =
          kinsim_2link_planar_B.T[kinsim_2link_planar_B.kstr + 2];
      }

      kinsim_2link_planar_B.ret_m = b->size[0] * b->size[1];
      b->size[0] = 6;
      b->size[1] = body->JointInternal.MotionSubspace->size[1];
      kinsim_emxEnsureCapacity_real_T(b, kinsim_2link_planar_B.ret_m);
      kinsim_2link_planar_B.loop_ub_m = body->JointInternal.MotionSubspace->
        size[0] * body->JointInternal.MotionSubspace->size[1] - 1;
      for (kinsim_2link_planar_B.ret_m = 0; kinsim_2link_planar_B.ret_m <=
           kinsim_2link_planar_B.loop_ub_m; kinsim_2link_planar_B.ret_m++) {
        b->data[kinsim_2link_planar_B.ret_m] =
          body->JointInternal.MotionSubspace->data[kinsim_2link_planar_B.ret_m];
      }

      kinsim_2link_planar_B.n_l = b->size[1] - 1;
      kinsim_2link_planar_B.ret_m = JacSlice->size[0] * JacSlice->size[1];
      JacSlice->size[0] = 6;
      JacSlice->size[1] = b->size[1];
      kinsim_emxEnsureCapacity_real_T(JacSlice, kinsim_2link_planar_B.ret_m);
      for (kinsim_2link_planar_B.ret_m = 0; kinsim_2link_planar_B.ret_m <=
           kinsim_2link_planar_B.n_l; kinsim_2link_planar_B.ret_m++) {
        kinsim_2link_planar_B.coffset_tmp = kinsim_2link_planar_B.ret_m * 6 - 1;
        for (kinsim_2link_planar_B.kstr = 0; kinsim_2link_planar_B.kstr < 6;
             kinsim_2link_planar_B.kstr++) {
          kinsim_2link_planar_B.s_d = 0.0;
          for (kinsim_2link_planar_B.loop_ub_m = 0;
               kinsim_2link_planar_B.loop_ub_m < 6;
               kinsim_2link_planar_B.loop_ub_m++) {
            kinsim_2link_planar_B.s_d +=
              kinsim_2link_planar_B.X[kinsim_2link_planar_B.loop_ub_m * 6 +
              kinsim_2link_planar_B.kstr] * b->data
              [(kinsim_2link_planar_B.coffset_tmp +
                kinsim_2link_planar_B.loop_ub_m) + 1];
          }

          JacSlice->data[(kinsim_2link_planar_B.coffset_tmp +
                          kinsim_2link_planar_B.kstr) + 1] =
            kinsim_2link_planar_B.s_d;
        }
      }

      if (kinsim_2link_planar_B.endeffectorIndex >
          kinsim_2link_planar_B.idx_idx_1) {
        kinsim_2link_planar_B.n_l = 0;
      } else {
        kinsim_2link_planar_B.n_l = static_cast<int32_T>
          (kinsim_2link_planar_B.endeffectorIndex) - 1;
      }

      kinsim_2link_planar_B.loop_ub_m = JacSlice->size[1];
      for (kinsim_2link_planar_B.ret_m = 0; kinsim_2link_planar_B.ret_m <
           kinsim_2link_planar_B.loop_ub_m; kinsim_2link_planar_B.ret_m++) {
        for (kinsim_2link_planar_B.kstr = 0; kinsim_2link_planar_B.kstr < 6;
             kinsim_2link_planar_B.kstr++) {
          Jac->data[kinsim_2link_planar_B.kstr + 6 * (kinsim_2link_planar_B.n_l
            + kinsim_2link_planar_B.ret_m)] = JacSlice->data[6 *
            kinsim_2link_planar_B.ret_m + kinsim_2link_planar_B.kstr];
        }
      }
    }
  }

  kinsim_2link_pla_emxFree_char_T(&bname);
  kinsim_2link_pla_emxFree_real_T(&JacSlice);
  kinsim_2lin_emxFree_f_cell_wrap(&Ttree);
  for (kinsim_2link_planar_B.ret_m = 0; kinsim_2link_planar_B.ret_m < 3;
       kinsim_2link_planar_B.ret_m++) {
    kinsim_2link_planar_B.b_i_o = kinsim_2link_planar_B.ret_m << 2;
    kinsim_2link_planar_B.X[6 * kinsim_2link_planar_B.ret_m] =
      kinsim_2link_planar_B.T2_c[kinsim_2link_planar_B.b_i_o];
    kinsim_2link_planar_B.kstr = 6 * (kinsim_2link_planar_B.ret_m + 3);
    kinsim_2link_planar_B.X[kinsim_2link_planar_B.kstr] = 0.0;
    kinsim_2link_planar_B.X[6 * kinsim_2link_planar_B.ret_m + 3] = 0.0;
    kinsim_2link_planar_B.X[kinsim_2link_planar_B.kstr + 3] =
      kinsim_2link_planar_B.T2_c[kinsim_2link_planar_B.b_i_o];
    kinsim_2link_planar_B.endeffectorIndex =
      kinsim_2link_planar_B.T2_c[kinsim_2link_planar_B.b_i_o + 1];
    kinsim_2link_planar_B.X[6 * kinsim_2link_planar_B.ret_m + 1] =
      kinsim_2link_planar_B.endeffectorIndex;
    kinsim_2link_planar_B.X[kinsim_2link_planar_B.kstr + 1] = 0.0;
    kinsim_2link_planar_B.X[6 * kinsim_2link_planar_B.ret_m + 4] = 0.0;
    kinsim_2link_planar_B.X[kinsim_2link_planar_B.kstr + 4] =
      kinsim_2link_planar_B.endeffectorIndex;
    kinsim_2link_planar_B.endeffectorIndex =
      kinsim_2link_planar_B.T2_c[kinsim_2link_planar_B.b_i_o + 2];
    kinsim_2link_planar_B.X[6 * kinsim_2link_planar_B.ret_m + 2] =
      kinsim_2link_planar_B.endeffectorIndex;
    kinsim_2link_planar_B.X[kinsim_2link_planar_B.kstr + 2] = 0.0;
    kinsim_2link_planar_B.X[6 * kinsim_2link_planar_B.ret_m + 5] = 0.0;
    kinsim_2link_planar_B.X[kinsim_2link_planar_B.kstr + 5] =
      kinsim_2link_planar_B.endeffectorIndex;
  }

  kinsim_2link_planar_B.n_l = Jac->size[1];
  kinsim_2link_planar_B.ret_m = b->size[0] * b->size[1];
  b->size[0] = 6;
  b->size[1] = Jac->size[1];
  kinsim_emxEnsureCapacity_real_T(b, kinsim_2link_planar_B.ret_m);
  kinsim_2link_planar_B.loop_ub_m = Jac->size[0] * Jac->size[1] - 1;
  for (kinsim_2link_planar_B.ret_m = 0; kinsim_2link_planar_B.ret_m <=
       kinsim_2link_planar_B.loop_ub_m; kinsim_2link_planar_B.ret_m++) {
    b->data[kinsim_2link_planar_B.ret_m] = Jac->data[kinsim_2link_planar_B.ret_m];
  }

  kinsim_2link_planar_B.ret_m = Jac->size[0] * Jac->size[1];
  Jac->size[0] = 6;
  Jac->size[1] = kinsim_2link_planar_B.n_l;
  kinsim_emxEnsureCapacity_real_T(Jac, kinsim_2link_planar_B.ret_m);
  for (kinsim_2link_planar_B.ret_m = 0; kinsim_2link_planar_B.ret_m <
       kinsim_2link_planar_B.n_l; kinsim_2link_planar_B.ret_m++) {
    kinsim_2link_planar_B.coffset_tmp = kinsim_2link_planar_B.ret_m * 6 - 1;
    for (kinsim_2link_planar_B.b_i_o = 0; kinsim_2link_planar_B.b_i_o < 6;
         kinsim_2link_planar_B.b_i_o++) {
      kinsim_2link_planar_B.s_d = 0.0;
      for (kinsim_2link_planar_B.loop_ub_m = 0; kinsim_2link_planar_B.loop_ub_m <
           6; kinsim_2link_planar_B.loop_ub_m++) {
        kinsim_2link_planar_B.s_d +=
          kinsim_2link_planar_B.X[kinsim_2link_planar_B.loop_ub_m * 6 +
          kinsim_2link_planar_B.b_i_o] * b->data
          [(kinsim_2link_planar_B.coffset_tmp + kinsim_2link_planar_B.loop_ub_m)
          + 1];
      }

      Jac->data[(kinsim_2link_planar_B.coffset_tmp + kinsim_2link_planar_B.b_i_o)
        + 1] = kinsim_2link_planar_B.s_d;
    }
  }

  kinsim_2link_pla_emxFree_real_T(&b);
}

static void rigidBodyJoint_get_JointAxis_l(const c_rigidBodyJoint_kinsim_2li_l_T
  *obj, real_T ax[3])
{
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  boolean_T guard1 = false;
  int32_T exitg1;
  for (kinsim_2link_planar_B.b_kstr_p = 0; kinsim_2link_planar_B.b_kstr_p < 8;
       kinsim_2link_planar_B.b_kstr_p++) {
    kinsim_2link_planar_B.b_bn[kinsim_2link_planar_B.b_kstr_p] =
      tmp[kinsim_2link_planar_B.b_kstr_p];
  }

  kinsim_2link_planar_B.b_bool_ie = false;
  if (obj->Type->size[1] == 8) {
    kinsim_2link_planar_B.b_kstr_p = 1;
    do {
      exitg1 = 0;
      if (kinsim_2link_planar_B.b_kstr_p - 1 < 8) {
        kinsim_2link_planar_B.kstr_p = kinsim_2link_planar_B.b_kstr_p - 1;
        if (obj->Type->data[kinsim_2link_planar_B.kstr_p] !=
            kinsim_2link_planar_B.b_bn[kinsim_2link_planar_B.kstr_p]) {
          exitg1 = 1;
        } else {
          kinsim_2link_planar_B.b_kstr_p++;
        }
      } else {
        kinsim_2link_planar_B.b_bool_ie = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (kinsim_2link_planar_B.b_bool_ie) {
    guard1 = true;
  } else {
    for (kinsim_2link_planar_B.b_kstr_p = 0; kinsim_2link_planar_B.b_kstr_p < 9;
         kinsim_2link_planar_B.b_kstr_p++) {
      kinsim_2link_planar_B.b_b[kinsim_2link_planar_B.b_kstr_p] =
        tmp_0[kinsim_2link_planar_B.b_kstr_p];
    }

    kinsim_2link_planar_B.b_bool_ie = false;
    if (obj->Type->size[1] == 9) {
      kinsim_2link_planar_B.b_kstr_p = 1;
      do {
        exitg1 = 0;
        if (kinsim_2link_planar_B.b_kstr_p - 1 < 9) {
          kinsim_2link_planar_B.kstr_p = kinsim_2link_planar_B.b_kstr_p - 1;
          if (obj->Type->data[kinsim_2link_planar_B.kstr_p] !=
              kinsim_2link_planar_B.b_b[kinsim_2link_planar_B.kstr_p]) {
            exitg1 = 1;
          } else {
            kinsim_2link_planar_B.b_kstr_p++;
          }
        } else {
          kinsim_2link_planar_B.b_bool_ie = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (kinsim_2link_planar_B.b_bool_ie) {
      guard1 = true;
    } else {
      ax[0] = (rtNaN);
      ax[1] = (rtNaN);
      ax[2] = (rtNaN);
    }
  }

  if (guard1) {
    ax[0] = obj->JointAxisInternal[0];
    ax[1] = obj->JointAxisInternal[1];
    ax[2] = obj->JointAxisInternal[2];
  }
}

static void RigidBodyTree_forwardKinemati_l(n_robotics_manip_internal_R_l_T *obj,
  const real_T qvec[6], emxArray_f_cell_wrap_kinsim_2_T *Ttree)
{
  l_robotics_manip_internal_R_l_T *body;
  emxArray_char_T_kinsim_2link__T *switch_expression;
  static const int8_T tmp[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  kinsim_2link_planar_B.n_j = obj->NumBodies;
  for (kinsim_2link_planar_B.b_kstr_k = 0; kinsim_2link_planar_B.b_kstr_k < 16;
       kinsim_2link_planar_B.b_kstr_k++) {
    kinsim_2link_planar_B.c_f1_c[kinsim_2link_planar_B.b_kstr_k] =
      tmp[kinsim_2link_planar_B.b_kstr_k];
  }

  kinsim_2link_planar_B.b_kstr_k = Ttree->size[0] * Ttree->size[1];
  Ttree->size[0] = 1;
  kinsim_2link_planar_B.e_m = static_cast<int32_T>(kinsim_2link_planar_B.n_j);
  Ttree->size[1] = kinsim_2link_planar_B.e_m;
  k_emxEnsureCapacity_f_cell_wrap(Ttree, kinsim_2link_planar_B.b_kstr_k);
  if (kinsim_2link_planar_B.e_m != 0) {
    kinsim_2link_planar_B.ntilecols_h = kinsim_2link_planar_B.e_m - 1;
    if (0 <= kinsim_2link_planar_B.ntilecols_h) {
      memcpy(&kinsim_2link_planar_B.expl_temp_m.f1[0],
             &kinsim_2link_planar_B.c_f1_c[0], sizeof(real_T) << 4U);
    }

    for (kinsim_2link_planar_B.b_jtilecol_c = 0;
         kinsim_2link_planar_B.b_jtilecol_c <= kinsim_2link_planar_B.ntilecols_h;
         kinsim_2link_planar_B.b_jtilecol_c++) {
      Ttree->data[kinsim_2link_planar_B.b_jtilecol_c] =
        kinsim_2link_planar_B.expl_temp_m;
    }
  }

  kinsim_2link_planar_B.k_o = 1.0;
  kinsim_2link_planar_B.ntilecols_h = static_cast<int32_T>
    (kinsim_2link_planar_B.n_j) - 1;
  kinsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  if (0 <= kinsim_2link_planar_B.ntilecols_h) {
    for (kinsim_2link_planar_B.b_kstr_k = 0; kinsim_2link_planar_B.b_kstr_k < 5;
         kinsim_2link_planar_B.b_kstr_k++) {
      kinsim_2link_planar_B.b_g[kinsim_2link_planar_B.b_kstr_k] =
        tmp_0[kinsim_2link_planar_B.b_kstr_k];
    }
  }

  for (kinsim_2link_planar_B.b_jtilecol_c = 0;
       kinsim_2link_planar_B.b_jtilecol_c <= kinsim_2link_planar_B.ntilecols_h;
       kinsim_2link_planar_B.b_jtilecol_c++) {
    body = obj->Bodies[kinsim_2link_planar_B.b_jtilecol_c];
    kinsim_2link_planar_B.n_j = body->JointInternal.PositionNumber;
    kinsim_2link_planar_B.n_j += kinsim_2link_planar_B.k_o;
    if (kinsim_2link_planar_B.k_o > kinsim_2link_planar_B.n_j - 1.0) {
      kinsim_2link_planar_B.e_m = 0;
      kinsim_2link_planar_B.d_m = 0;
    } else {
      kinsim_2link_planar_B.e_m = static_cast<int32_T>(kinsim_2link_planar_B.k_o)
        - 1;
      kinsim_2link_planar_B.d_m = static_cast<int32_T>(kinsim_2link_planar_B.n_j
        - 1.0);
    }

    kinsim_2link_planar_B.b_kstr_k = switch_expression->size[0] *
      switch_expression->size[1];
    switch_expression->size[0] = 1;
    switch_expression->size[1] = body->JointInternal.Type->size[1];
    kinsim_emxEnsureCapacity_char_T(switch_expression,
      kinsim_2link_planar_B.b_kstr_k);
    kinsim_2link_planar_B.loop_ub_pc = body->JointInternal.Type->size[0] *
      body->JointInternal.Type->size[1] - 1;
    for (kinsim_2link_planar_B.b_kstr_k = 0; kinsim_2link_planar_B.b_kstr_k <=
         kinsim_2link_planar_B.loop_ub_pc; kinsim_2link_planar_B.b_kstr_k++) {
      switch_expression->data[kinsim_2link_planar_B.b_kstr_k] =
        body->JointInternal.Type->data[kinsim_2link_planar_B.b_kstr_k];
    }

    kinsim_2link_planar_B.b_bool_k = false;
    if (switch_expression->size[1] == 5) {
      kinsim_2link_planar_B.b_kstr_k = 1;
      do {
        exitg1 = 0;
        if (kinsim_2link_planar_B.b_kstr_k - 1 < 5) {
          kinsim_2link_planar_B.loop_ub_pc = kinsim_2link_planar_B.b_kstr_k - 1;
          if (switch_expression->data[kinsim_2link_planar_B.loop_ub_pc] !=
              kinsim_2link_planar_B.b_g[kinsim_2link_planar_B.loop_ub_pc]) {
            exitg1 = 1;
          } else {
            kinsim_2link_planar_B.b_kstr_k++;
          }
        } else {
          kinsim_2link_planar_B.b_bool_k = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (kinsim_2link_planar_B.b_bool_k) {
      kinsim_2link_planar_B.b_kstr_k = 0;
    } else {
      for (kinsim_2link_planar_B.b_kstr_k = 0; kinsim_2link_planar_B.b_kstr_k <
           8; kinsim_2link_planar_B.b_kstr_k++) {
        kinsim_2link_planar_B.b_h[kinsim_2link_planar_B.b_kstr_k] =
          tmp_1[kinsim_2link_planar_B.b_kstr_k];
      }

      kinsim_2link_planar_B.b_bool_k = false;
      if (switch_expression->size[1] == 8) {
        kinsim_2link_planar_B.b_kstr_k = 1;
        do {
          exitg1 = 0;
          if (kinsim_2link_planar_B.b_kstr_k - 1 < 8) {
            kinsim_2link_planar_B.loop_ub_pc = kinsim_2link_planar_B.b_kstr_k -
              1;
            if (switch_expression->data[kinsim_2link_planar_B.loop_ub_pc] !=
                kinsim_2link_planar_B.b_h[kinsim_2link_planar_B.loop_ub_pc]) {
              exitg1 = 1;
            } else {
              kinsim_2link_planar_B.b_kstr_k++;
            }
          } else {
            kinsim_2link_planar_B.b_bool_k = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (kinsim_2link_planar_B.b_bool_k) {
        kinsim_2link_planar_B.b_kstr_k = 1;
      } else {
        kinsim_2link_planar_B.b_kstr_k = -1;
      }
    }

    switch (kinsim_2link_planar_B.b_kstr_k) {
     case 0:
      memset(&kinsim_2link_planar_B.c_f1_c[0], 0, sizeof(real_T) << 4U);
      kinsim_2link_planar_B.c_f1_c[0] = 1.0;
      kinsim_2link_planar_B.c_f1_c[5] = 1.0;
      kinsim_2link_planar_B.c_f1_c[10] = 1.0;
      kinsim_2link_planar_B.c_f1_c[15] = 1.0;
      break;

     case 1:
      rigidBodyJoint_get_JointAxis_l(&body->JointInternal,
        kinsim_2link_planar_B.v_d);
      kinsim_2link_planar_B.d_m -= kinsim_2link_planar_B.e_m;
      for (kinsim_2link_planar_B.b_kstr_k = 0; kinsim_2link_planar_B.b_kstr_k <
           kinsim_2link_planar_B.d_m; kinsim_2link_planar_B.b_kstr_k++) {
        kinsim_2link_planar_B.e_data_g[kinsim_2link_planar_B.b_kstr_k] =
          kinsim_2link_planar_B.e_m + kinsim_2link_planar_B.b_kstr_k;
      }

      kinsim_2link_planar_B.result_data_j[0] = kinsim_2link_planar_B.v_d[0];
      kinsim_2link_planar_B.result_data_j[1] = kinsim_2link_planar_B.v_d[1];
      kinsim_2link_planar_B.result_data_j[2] = kinsim_2link_planar_B.v_d[2];
      if (0 <= (kinsim_2link_planar_B.d_m != 0) - 1) {
        kinsim_2link_planar_B.result_data_j[3] =
          qvec[kinsim_2link_planar_B.e_data_g[0]];
      }

      kinsim_2link_planar_B.k_o = 1.0 / sqrt
        ((kinsim_2link_planar_B.result_data_j[0] *
          kinsim_2link_planar_B.result_data_j[0] +
          kinsim_2link_planar_B.result_data_j[1] *
          kinsim_2link_planar_B.result_data_j[1]) +
         kinsim_2link_planar_B.result_data_j[2] *
         kinsim_2link_planar_B.result_data_j[2]);
      kinsim_2link_planar_B.v_d[0] = kinsim_2link_planar_B.result_data_j[0] *
        kinsim_2link_planar_B.k_o;
      kinsim_2link_planar_B.v_d[1] = kinsim_2link_planar_B.result_data_j[1] *
        kinsim_2link_planar_B.k_o;
      kinsim_2link_planar_B.v_d[2] = kinsim_2link_planar_B.result_data_j[2] *
        kinsim_2link_planar_B.k_o;
      kinsim_2link_planar_B.k_o = cos(kinsim_2link_planar_B.result_data_j[3]);
      kinsim_2link_planar_B.sth_n = sin(kinsim_2link_planar_B.result_data_j[3]);
      kinsim_2link_planar_B.tempR_l[0] = kinsim_2link_planar_B.v_d[0] *
        kinsim_2link_planar_B.v_d[0] * (1.0 - kinsim_2link_planar_B.k_o) +
        kinsim_2link_planar_B.k_o;
      kinsim_2link_planar_B.tempR_tmp_i = kinsim_2link_planar_B.v_d[1] *
        kinsim_2link_planar_B.v_d[0] * (1.0 - kinsim_2link_planar_B.k_o);
      kinsim_2link_planar_B.tempR_tmp_o = kinsim_2link_planar_B.v_d[2] *
        kinsim_2link_planar_B.sth_n;
      kinsim_2link_planar_B.tempR_l[1] = kinsim_2link_planar_B.tempR_tmp_i -
        kinsim_2link_planar_B.tempR_tmp_o;
      kinsim_2link_planar_B.tempR_tmp_n = kinsim_2link_planar_B.v_d[2] *
        kinsim_2link_planar_B.v_d[0] * (1.0 - kinsim_2link_planar_B.k_o);
      kinsim_2link_planar_B.tempR_tmp_m = kinsim_2link_planar_B.v_d[1] *
        kinsim_2link_planar_B.sth_n;
      kinsim_2link_planar_B.tempR_l[2] = kinsim_2link_planar_B.tempR_tmp_n +
        kinsim_2link_planar_B.tempR_tmp_m;
      kinsim_2link_planar_B.tempR_l[3] = kinsim_2link_planar_B.tempR_tmp_i +
        kinsim_2link_planar_B.tempR_tmp_o;
      kinsim_2link_planar_B.tempR_l[4] = kinsim_2link_planar_B.v_d[1] *
        kinsim_2link_planar_B.v_d[1] * (1.0 - kinsim_2link_planar_B.k_o) +
        kinsim_2link_planar_B.k_o;
      kinsim_2link_planar_B.tempR_tmp_i = kinsim_2link_planar_B.v_d[2] *
        kinsim_2link_planar_B.v_d[1] * (1.0 - kinsim_2link_planar_B.k_o);
      kinsim_2link_planar_B.tempR_tmp_o = kinsim_2link_planar_B.v_d[0] *
        kinsim_2link_planar_B.sth_n;
      kinsim_2link_planar_B.tempR_l[5] = kinsim_2link_planar_B.tempR_tmp_i -
        kinsim_2link_planar_B.tempR_tmp_o;
      kinsim_2link_planar_B.tempR_l[6] = kinsim_2link_planar_B.tempR_tmp_n -
        kinsim_2link_planar_B.tempR_tmp_m;
      kinsim_2link_planar_B.tempR_l[7] = kinsim_2link_planar_B.tempR_tmp_i +
        kinsim_2link_planar_B.tempR_tmp_o;
      kinsim_2link_planar_B.tempR_l[8] = kinsim_2link_planar_B.v_d[2] *
        kinsim_2link_planar_B.v_d[2] * (1.0 - kinsim_2link_planar_B.k_o) +
        kinsim_2link_planar_B.k_o;
      for (kinsim_2link_planar_B.b_kstr_k = 0; kinsim_2link_planar_B.b_kstr_k <
           3; kinsim_2link_planar_B.b_kstr_k++) {
        kinsim_2link_planar_B.e_m = kinsim_2link_planar_B.b_kstr_k + 1;
        kinsim_2link_planar_B.R_p[kinsim_2link_planar_B.e_m - 1] =
          kinsim_2link_planar_B.tempR_l[(kinsim_2link_planar_B.e_m - 1) * 3];
        kinsim_2link_planar_B.e_m = kinsim_2link_planar_B.b_kstr_k + 1;
        kinsim_2link_planar_B.R_p[kinsim_2link_planar_B.e_m + 2] =
          kinsim_2link_planar_B.tempR_l[(kinsim_2link_planar_B.e_m - 1) * 3 + 1];
        kinsim_2link_planar_B.e_m = kinsim_2link_planar_B.b_kstr_k + 1;
        kinsim_2link_planar_B.R_p[kinsim_2link_planar_B.e_m + 5] =
          kinsim_2link_planar_B.tempR_l[(kinsim_2link_planar_B.e_m - 1) * 3 + 2];
      }

      memset(&kinsim_2link_planar_B.c_f1_c[0], 0, sizeof(real_T) << 4U);
      for (kinsim_2link_planar_B.b_kstr_k = 0; kinsim_2link_planar_B.b_kstr_k <
           3; kinsim_2link_planar_B.b_kstr_k++) {
        kinsim_2link_planar_B.d_m = kinsim_2link_planar_B.b_kstr_k << 2;
        kinsim_2link_planar_B.c_f1_c[kinsim_2link_planar_B.d_m] =
          kinsim_2link_planar_B.R_p[3 * kinsim_2link_planar_B.b_kstr_k];
        kinsim_2link_planar_B.c_f1_c[kinsim_2link_planar_B.d_m + 1] =
          kinsim_2link_planar_B.R_p[3 * kinsim_2link_planar_B.b_kstr_k + 1];
        kinsim_2link_planar_B.c_f1_c[kinsim_2link_planar_B.d_m + 2] =
          kinsim_2link_planar_B.R_p[3 * kinsim_2link_planar_B.b_kstr_k + 2];
      }

      kinsim_2link_planar_B.c_f1_c[15] = 1.0;
      break;

     default:
      rigidBodyJoint_get_JointAxis_l(&body->JointInternal,
        kinsim_2link_planar_B.v_d);
      memset(&kinsim_2link_planar_B.tempR_l[0], 0, 9U * sizeof(real_T));
      kinsim_2link_planar_B.tempR_l[0] = 1.0;
      kinsim_2link_planar_B.tempR_l[4] = 1.0;
      kinsim_2link_planar_B.tempR_l[8] = 1.0;
      for (kinsim_2link_planar_B.b_kstr_k = 0; kinsim_2link_planar_B.b_kstr_k <
           3; kinsim_2link_planar_B.b_kstr_k++) {
        kinsim_2link_planar_B.d_m = kinsim_2link_planar_B.b_kstr_k << 2;
        kinsim_2link_planar_B.c_f1_c[kinsim_2link_planar_B.d_m] =
          kinsim_2link_planar_B.tempR_l[3 * kinsim_2link_planar_B.b_kstr_k];
        kinsim_2link_planar_B.c_f1_c[kinsim_2link_planar_B.d_m + 1] =
          kinsim_2link_planar_B.tempR_l[3 * kinsim_2link_planar_B.b_kstr_k + 1];
        kinsim_2link_planar_B.c_f1_c[kinsim_2link_planar_B.d_m + 2] =
          kinsim_2link_planar_B.tempR_l[3 * kinsim_2link_planar_B.b_kstr_k + 2];
        kinsim_2link_planar_B.c_f1_c[kinsim_2link_planar_B.b_kstr_k + 12] =
          kinsim_2link_planar_B.v_d[kinsim_2link_planar_B.b_kstr_k] *
          qvec[kinsim_2link_planar_B.e_m];
      }

      kinsim_2link_planar_B.c_f1_c[3] = 0.0;
      kinsim_2link_planar_B.c_f1_c[7] = 0.0;
      kinsim_2link_planar_B.c_f1_c[11] = 0.0;
      kinsim_2link_planar_B.c_f1_c[15] = 1.0;
      break;
    }

    for (kinsim_2link_planar_B.b_kstr_k = 0; kinsim_2link_planar_B.b_kstr_k < 16;
         kinsim_2link_planar_B.b_kstr_k++) {
      kinsim_2link_planar_B.a_b[kinsim_2link_planar_B.b_kstr_k] =
        body->
        JointInternal.JointToParentTransform[kinsim_2link_planar_B.b_kstr_k];
    }

    for (kinsim_2link_planar_B.b_kstr_k = 0; kinsim_2link_planar_B.b_kstr_k < 16;
         kinsim_2link_planar_B.b_kstr_k++) {
      kinsim_2link_planar_B.b_p[kinsim_2link_planar_B.b_kstr_k] =
        body->JointInternal.ChildToJointTransform[kinsim_2link_planar_B.b_kstr_k];
    }

    for (kinsim_2link_planar_B.b_kstr_k = 0; kinsim_2link_planar_B.b_kstr_k < 4;
         kinsim_2link_planar_B.b_kstr_k++) {
      for (kinsim_2link_planar_B.e_m = 0; kinsim_2link_planar_B.e_m < 4;
           kinsim_2link_planar_B.e_m++) {
        kinsim_2link_planar_B.d_m = kinsim_2link_planar_B.e_m << 2;
        kinsim_2link_planar_B.loop_ub_pc = kinsim_2link_planar_B.b_kstr_k +
          kinsim_2link_planar_B.d_m;
        kinsim_2link_planar_B.a_c[kinsim_2link_planar_B.loop_ub_pc] = 0.0;
        kinsim_2link_planar_B.a_c[kinsim_2link_planar_B.loop_ub_pc] +=
          kinsim_2link_planar_B.c_f1_c[kinsim_2link_planar_B.d_m] *
          kinsim_2link_planar_B.a_b[kinsim_2link_planar_B.b_kstr_k];
        kinsim_2link_planar_B.a_c[kinsim_2link_planar_B.loop_ub_pc] +=
          kinsim_2link_planar_B.c_f1_c[kinsim_2link_planar_B.d_m + 1] *
          kinsim_2link_planar_B.a_b[kinsim_2link_planar_B.b_kstr_k + 4];
        kinsim_2link_planar_B.a_c[kinsim_2link_planar_B.loop_ub_pc] +=
          kinsim_2link_planar_B.c_f1_c[kinsim_2link_planar_B.d_m + 2] *
          kinsim_2link_planar_B.a_b[kinsim_2link_planar_B.b_kstr_k + 8];
        kinsim_2link_planar_B.a_c[kinsim_2link_planar_B.loop_ub_pc] +=
          kinsim_2link_planar_B.c_f1_c[kinsim_2link_planar_B.d_m + 3] *
          kinsim_2link_planar_B.a_b[kinsim_2link_planar_B.b_kstr_k + 12];
      }

      for (kinsim_2link_planar_B.e_m = 0; kinsim_2link_planar_B.e_m < 4;
           kinsim_2link_planar_B.e_m++) {
        kinsim_2link_planar_B.d_m = kinsim_2link_planar_B.e_m << 2;
        kinsim_2link_planar_B.loop_ub_pc = kinsim_2link_planar_B.b_kstr_k +
          kinsim_2link_planar_B.d_m;
        Ttree->data[kinsim_2link_planar_B.b_jtilecol_c]
          .f1[kinsim_2link_planar_B.loop_ub_pc] = 0.0;
        Ttree->data[kinsim_2link_planar_B.b_jtilecol_c]
          .f1[kinsim_2link_planar_B.loop_ub_pc] +=
          kinsim_2link_planar_B.b_p[kinsim_2link_planar_B.d_m] *
          kinsim_2link_planar_B.a_c[kinsim_2link_planar_B.b_kstr_k];
        Ttree->data[kinsim_2link_planar_B.b_jtilecol_c]
          .f1[kinsim_2link_planar_B.loop_ub_pc] +=
          kinsim_2link_planar_B.b_p[kinsim_2link_planar_B.d_m + 1] *
          kinsim_2link_planar_B.a_c[kinsim_2link_planar_B.b_kstr_k + 4];
        Ttree->data[kinsim_2link_planar_B.b_jtilecol_c]
          .f1[kinsim_2link_planar_B.loop_ub_pc] +=
          kinsim_2link_planar_B.b_p[kinsim_2link_planar_B.d_m + 2] *
          kinsim_2link_planar_B.a_c[kinsim_2link_planar_B.b_kstr_k + 8];
        Ttree->data[kinsim_2link_planar_B.b_jtilecol_c]
          .f1[kinsim_2link_planar_B.loop_ub_pc] +=
          kinsim_2link_planar_B.b_p[kinsim_2link_planar_B.d_m + 3] *
          kinsim_2link_planar_B.a_c[kinsim_2link_planar_B.b_kstr_k + 12];
      }
    }

    kinsim_2link_planar_B.k_o = kinsim_2link_planar_B.n_j;
    if (body->ParentIndex > 0.0) {
      for (kinsim_2link_planar_B.b_kstr_k = 0; kinsim_2link_planar_B.b_kstr_k <
           16; kinsim_2link_planar_B.b_kstr_k++) {
        kinsim_2link_planar_B.a_b[kinsim_2link_planar_B.b_kstr_k] = Ttree->data[
          static_cast<int32_T>(body->ParentIndex) - 1]
          .f1[kinsim_2link_planar_B.b_kstr_k];
      }

      for (kinsim_2link_planar_B.b_kstr_k = 0; kinsim_2link_planar_B.b_kstr_k <
           4; kinsim_2link_planar_B.b_kstr_k++) {
        for (kinsim_2link_planar_B.e_m = 0; kinsim_2link_planar_B.e_m < 4;
             kinsim_2link_planar_B.e_m++) {
          kinsim_2link_planar_B.d_m = kinsim_2link_planar_B.e_m << 2;
          kinsim_2link_planar_B.loop_ub_pc = kinsim_2link_planar_B.b_kstr_k +
            kinsim_2link_planar_B.d_m;
          kinsim_2link_planar_B.a_c[kinsim_2link_planar_B.loop_ub_pc] = 0.0;
          kinsim_2link_planar_B.a_c[kinsim_2link_planar_B.loop_ub_pc] +=
            Ttree->data[kinsim_2link_planar_B.b_jtilecol_c]
            .f1[kinsim_2link_planar_B.d_m] *
            kinsim_2link_planar_B.a_b[kinsim_2link_planar_B.b_kstr_k];
          kinsim_2link_planar_B.a_c[kinsim_2link_planar_B.loop_ub_pc] +=
            Ttree->data[kinsim_2link_planar_B.b_jtilecol_c]
            .f1[kinsim_2link_planar_B.d_m + 1] *
            kinsim_2link_planar_B.a_b[kinsim_2link_planar_B.b_kstr_k + 4];
          kinsim_2link_planar_B.a_c[kinsim_2link_planar_B.loop_ub_pc] +=
            Ttree->data[kinsim_2link_planar_B.b_jtilecol_c]
            .f1[kinsim_2link_planar_B.d_m + 2] *
            kinsim_2link_planar_B.a_b[kinsim_2link_planar_B.b_kstr_k + 8];
          kinsim_2link_planar_B.a_c[kinsim_2link_planar_B.loop_ub_pc] +=
            Ttree->data[kinsim_2link_planar_B.b_jtilecol_c]
            .f1[kinsim_2link_planar_B.d_m + 3] *
            kinsim_2link_planar_B.a_b[kinsim_2link_planar_B.b_kstr_k + 12];
        }
      }

      for (kinsim_2link_planar_B.b_kstr_k = 0; kinsim_2link_planar_B.b_kstr_k <
           16; kinsim_2link_planar_B.b_kstr_k++) {
        Ttree->data[kinsim_2link_planar_B.b_jtilecol_c]
          .f1[kinsim_2link_planar_B.b_kstr_k] =
          kinsim_2link_planar_B.a_c[kinsim_2link_planar_B.b_kstr_k];
      }
    }
  }

  kinsim_2link_pla_emxFree_char_T(&switch_expression);
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

// Function for MATLAB Function: '<S5>/MATLAB Function'
static boolean_T kinsim_2link_plana_anyNonFinite(const real_T x[16])
{
  boolean_T b_p;
  int32_T k;
  b_p = true;
  for (k = 0; k < 16; k++) {
    if (b_p && (rtIsInf(x[k]) || rtIsNaN(x[k]))) {
      b_p = false;
    }
  }

  return !b_p;
}

static real_T kinsim_2link_plan_rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T y;
  kinsim_2link_planar_B.a_j = fabs(u0);
  y = fabs(u1);
  if (kinsim_2link_planar_B.a_j < y) {
    kinsim_2link_planar_B.a_j /= y;
    y *= sqrt(kinsim_2link_planar_B.a_j * kinsim_2link_planar_B.a_j + 1.0);
  } else if (kinsim_2link_planar_B.a_j > y) {
    y /= kinsim_2link_planar_B.a_j;
    y = sqrt(y * y + 1.0) * kinsim_2link_planar_B.a_j;
  } else {
    if (!rtIsNaN(y)) {
      y = kinsim_2link_planar_B.a_j * 1.4142135623730951;
    }
  }

  return y;
}

// Function for MATLAB Function: '<S5>/MATLAB Function'
static real_T kinsim_2link_planar_xzlangeM(const creal_T x[16])
{
  real_T y;
  real_T absxk;
  int32_T k;
  boolean_T exitg1;
  y = 0.0;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 16)) {
    absxk = kinsim_2link_plan_rt_hypotd_snf(x[k].re, x[k].im);
    if (rtIsNaN(absxk)) {
      y = (rtNaN);
      exitg1 = true;
    } else {
      if (absxk > y) {
        y = absxk;
      }

      k++;
    }
  }

  return y;
}

// Function for MATLAB Function: '<S5>/MATLAB Function'
static void kinsim_2link_planar_xzlascl(real_T cfrom, real_T cto, creal_T A[16])
{
  real_T cfromc;
  real_T ctoc;
  boolean_T notdone;
  real_T cfrom1;
  real_T cto1;
  real_T mul;
  int32_T i;
  cfromc = cfrom;
  ctoc = cto;
  notdone = true;
  while (notdone) {
    cfrom1 = cfromc * 2.0041683600089728E-292;
    cto1 = ctoc / 4.9896007738368E+291;
    if ((fabs(cfrom1) > fabs(ctoc)) && (ctoc != 0.0)) {
      mul = 2.0041683600089728E-292;
      cfromc = cfrom1;
    } else if (fabs(cto1) > fabs(cfromc)) {
      mul = 4.9896007738368E+291;
      ctoc = cto1;
    } else {
      mul = ctoc / cfromc;
      notdone = false;
    }

    for (i = 0; i < 16; i++) {
      A[i].re *= mul;
      A[i].im *= mul;
    }
  }
}

// Function for MATLAB Function: '<S5>/MATLAB Function'
static real_T kinsim_2link_planar_xzlanhs(const creal_T A[16], int32_T ilo,
  int32_T ihi)
{
  real_T f;
  real_T scale;
  real_T sumsq;
  boolean_T firstNonZero;
  real_T temp1;
  real_T temp2;
  int32_T j;
  int32_T b;
  int32_T i;
  int32_T reAij_tmp;
  f = 0.0;
  if (ilo <= ihi) {
    scale = 0.0;
    sumsq = 0.0;
    firstNonZero = true;
    for (j = ilo; j <= ihi; j++) {
      b = j + 1;
      if (ihi < j + 1) {
        b = ihi;
      }

      for (i = ilo; i <= b; i++) {
        reAij_tmp = (((j - 1) << 2) + i) - 1;
        if (A[reAij_tmp].re != 0.0) {
          temp1 = fabs(A[reAij_tmp].re);
          if (firstNonZero) {
            sumsq = 1.0;
            scale = temp1;
            firstNonZero = false;
          } else if (scale < temp1) {
            temp2 = scale / temp1;
            sumsq = sumsq * temp2 * temp2 + 1.0;
            scale = temp1;
          } else {
            temp2 = temp1 / scale;
            sumsq += temp2 * temp2;
          }
        }

        if (A[reAij_tmp].im != 0.0) {
          temp1 = fabs(A[reAij_tmp].im);
          if (firstNonZero) {
            sumsq = 1.0;
            scale = temp1;
            firstNonZero = false;
          } else if (scale < temp1) {
            temp2 = scale / temp1;
            sumsq = sumsq * temp2 * temp2 + 1.0;
            scale = temp1;
          } else {
            temp2 = temp1 / scale;
            sumsq += temp2 * temp2;
          }
        }
      }
    }

    f = scale * sqrt(sumsq);
  }

  return f;
}

// Function for MATLAB Function: '<S5>/MATLAB Function'
static void kinsim_2link_planar_xzlartg_a(const creal_T f, const creal_T g,
  real_T *cs, creal_T *sn)
{
  real_T gs_im;
  boolean_T guard1 = false;
  kinsim_2link_planar_B.f2s_p = fabs(f.re);
  kinsim_2link_planar_B.di_p = fabs(f.im);
  kinsim_2link_planar_B.scale_c0 = kinsim_2link_planar_B.f2s_p;
  if (kinsim_2link_planar_B.di_p > kinsim_2link_planar_B.f2s_p) {
    kinsim_2link_planar_B.scale_c0 = kinsim_2link_planar_B.di_p;
  }

  kinsim_2link_planar_B.gs_re_a = fabs(g.re);
  gs_im = fabs(g.im);
  if (gs_im > kinsim_2link_planar_B.gs_re_a) {
    kinsim_2link_planar_B.gs_re_a = gs_im;
  }

  if (kinsim_2link_planar_B.gs_re_a > kinsim_2link_planar_B.scale_c0) {
    kinsim_2link_planar_B.scale_c0 = kinsim_2link_planar_B.gs_re_a;
  }

  kinsim_2link_planar_B.fs_re_e = f.re;
  kinsim_2link_planar_B.fs_im_a = f.im;
  kinsim_2link_planar_B.gs_re_a = g.re;
  gs_im = g.im;
  guard1 = false;
  if (kinsim_2link_planar_B.scale_c0 >= 7.4428285367870146E+137) {
    do {
      kinsim_2link_planar_B.fs_re_e *= 1.3435752215134178E-138;
      kinsim_2link_planar_B.fs_im_a *= 1.3435752215134178E-138;
      kinsim_2link_planar_B.gs_re_a *= 1.3435752215134178E-138;
      gs_im *= 1.3435752215134178E-138;
      kinsim_2link_planar_B.scale_c0 *= 1.3435752215134178E-138;
    } while (!(kinsim_2link_planar_B.scale_c0 < 7.4428285367870146E+137));

    guard1 = true;
  } else if (kinsim_2link_planar_B.scale_c0 <= 1.3435752215134178E-138) {
    if ((g.re == 0.0) && (g.im == 0.0)) {
      *cs = 1.0;
      sn->re = 0.0;
      sn->im = 0.0;
    } else {
      do {
        kinsim_2link_planar_B.fs_re_e *= 7.4428285367870146E+137;
        kinsim_2link_planar_B.fs_im_a *= 7.4428285367870146E+137;
        kinsim_2link_planar_B.gs_re_a *= 7.4428285367870146E+137;
        gs_im *= 7.4428285367870146E+137;
        kinsim_2link_planar_B.scale_c0 *= 7.4428285367870146E+137;
      } while (!(kinsim_2link_planar_B.scale_c0 > 1.3435752215134178E-138));

      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    kinsim_2link_planar_B.scale_c0 = kinsim_2link_planar_B.fs_re_e *
      kinsim_2link_planar_B.fs_re_e + kinsim_2link_planar_B.fs_im_a *
      kinsim_2link_planar_B.fs_im_a;
    kinsim_2link_planar_B.g2_c = kinsim_2link_planar_B.gs_re_a *
      kinsim_2link_planar_B.gs_re_a + gs_im * gs_im;
    kinsim_2link_planar_B.x_a = kinsim_2link_planar_B.g2_c;
    if (1.0 > kinsim_2link_planar_B.g2_c) {
      kinsim_2link_planar_B.x_a = 1.0;
    }

    if (kinsim_2link_planar_B.scale_c0 <= kinsim_2link_planar_B.x_a *
        2.0041683600089728E-292) {
      if ((f.re == 0.0) && (f.im == 0.0)) {
        *cs = 0.0;
        kinsim_2link_planar_B.f2s_p = kinsim_2link_plan_rt_hypotd_snf
          (kinsim_2link_planar_B.gs_re_a, gs_im);
        sn->re = kinsim_2link_planar_B.gs_re_a / kinsim_2link_planar_B.f2s_p;
        sn->im = -gs_im / kinsim_2link_planar_B.f2s_p;
      } else {
        kinsim_2link_planar_B.scale_c0 = sqrt(kinsim_2link_planar_B.g2_c);
        *cs = kinsim_2link_plan_rt_hypotd_snf(kinsim_2link_planar_B.fs_re_e,
          kinsim_2link_planar_B.fs_im_a) / kinsim_2link_planar_B.scale_c0;
        if (kinsim_2link_planar_B.di_p > kinsim_2link_planar_B.f2s_p) {
          kinsim_2link_planar_B.f2s_p = kinsim_2link_planar_B.di_p;
        }

        if (kinsim_2link_planar_B.f2s_p > 1.0) {
          kinsim_2link_planar_B.f2s_p = kinsim_2link_plan_rt_hypotd_snf(f.re,
            f.im);
          kinsim_2link_planar_B.fs_re_e = f.re / kinsim_2link_planar_B.f2s_p;
          kinsim_2link_planar_B.fs_im_a = f.im / kinsim_2link_planar_B.f2s_p;
        } else {
          kinsim_2link_planar_B.fs_re_e = 7.4428285367870146E+137 * f.re;
          kinsim_2link_planar_B.di_p = 7.4428285367870146E+137 * f.im;
          kinsim_2link_planar_B.f2s_p = kinsim_2link_plan_rt_hypotd_snf
            (kinsim_2link_planar_B.fs_re_e, kinsim_2link_planar_B.di_p);
          kinsim_2link_planar_B.fs_re_e /= kinsim_2link_planar_B.f2s_p;
          kinsim_2link_planar_B.fs_im_a = kinsim_2link_planar_B.di_p /
            kinsim_2link_planar_B.f2s_p;
        }

        kinsim_2link_planar_B.gs_re_a /= kinsim_2link_planar_B.scale_c0;
        gs_im = -gs_im / kinsim_2link_planar_B.scale_c0;
        sn->re = kinsim_2link_planar_B.fs_re_e * kinsim_2link_planar_B.gs_re_a -
          kinsim_2link_planar_B.fs_im_a * gs_im;
        sn->im = kinsim_2link_planar_B.fs_re_e * gs_im +
          kinsim_2link_planar_B.fs_im_a * kinsim_2link_planar_B.gs_re_a;
      }
    } else {
      kinsim_2link_planar_B.f2s_p = sqrt(kinsim_2link_planar_B.g2_c /
        kinsim_2link_planar_B.scale_c0 + 1.0);
      kinsim_2link_planar_B.fs_re_e *= kinsim_2link_planar_B.f2s_p;
      kinsim_2link_planar_B.fs_im_a *= kinsim_2link_planar_B.f2s_p;
      *cs = 1.0 / kinsim_2link_planar_B.f2s_p;
      kinsim_2link_planar_B.f2s_p = kinsim_2link_planar_B.scale_c0 +
        kinsim_2link_planar_B.g2_c;
      kinsim_2link_planar_B.fs_re_e /= kinsim_2link_planar_B.f2s_p;
      kinsim_2link_planar_B.fs_im_a /= kinsim_2link_planar_B.f2s_p;
      sn->re = kinsim_2link_planar_B.fs_re_e * kinsim_2link_planar_B.gs_re_a -
        kinsim_2link_planar_B.fs_im_a * -gs_im;
      sn->im = kinsim_2link_planar_B.fs_re_e * -gs_im +
        kinsim_2link_planar_B.fs_im_a * kinsim_2link_planar_B.gs_re_a;
    }
  }
}

// Function for MATLAB Function: '<S5>/MATLAB Function'
static void kinsim_2link_planar_xzlartg(const creal_T f, const creal_T g, real_T
  *cs, creal_T *sn, creal_T *r)
{
  int32_T count;
  int32_T rescaledir;
  boolean_T guard1 = false;
  kinsim_2link_planar_B.f2s = fabs(f.re);
  kinsim_2link_planar_B.di = fabs(f.im);
  kinsim_2link_planar_B.scale_a = kinsim_2link_planar_B.f2s;
  if (kinsim_2link_planar_B.di > kinsim_2link_planar_B.f2s) {
    kinsim_2link_planar_B.scale_a = kinsim_2link_planar_B.di;
  }

  kinsim_2link_planar_B.gs_re = fabs(g.re);
  kinsim_2link_planar_B.gs_im = fabs(g.im);
  if (kinsim_2link_planar_B.gs_im > kinsim_2link_planar_B.gs_re) {
    kinsim_2link_planar_B.gs_re = kinsim_2link_planar_B.gs_im;
  }

  if (kinsim_2link_planar_B.gs_re > kinsim_2link_planar_B.scale_a) {
    kinsim_2link_planar_B.scale_a = kinsim_2link_planar_B.gs_re;
  }

  kinsim_2link_planar_B.fs_re = f.re;
  kinsim_2link_planar_B.fs_im = f.im;
  kinsim_2link_planar_B.gs_re = g.re;
  kinsim_2link_planar_B.gs_im = g.im;
  count = -1;
  rescaledir = 0;
  guard1 = false;
  if (kinsim_2link_planar_B.scale_a >= 7.4428285367870146E+137) {
    do {
      count++;
      kinsim_2link_planar_B.fs_re *= 1.3435752215134178E-138;
      kinsim_2link_planar_B.fs_im *= 1.3435752215134178E-138;
      kinsim_2link_planar_B.gs_re *= 1.3435752215134178E-138;
      kinsim_2link_planar_B.gs_im *= 1.3435752215134178E-138;
      kinsim_2link_planar_B.scale_a *= 1.3435752215134178E-138;
    } while (!(kinsim_2link_planar_B.scale_a < 7.4428285367870146E+137));

    rescaledir = 1;
    guard1 = true;
  } else if (kinsim_2link_planar_B.scale_a <= 1.3435752215134178E-138) {
    if ((g.re == 0.0) && (g.im == 0.0)) {
      *cs = 1.0;
      sn->re = 0.0;
      sn->im = 0.0;
      *r = f;
    } else {
      do {
        count++;
        kinsim_2link_planar_B.fs_re *= 7.4428285367870146E+137;
        kinsim_2link_planar_B.fs_im *= 7.4428285367870146E+137;
        kinsim_2link_planar_B.gs_re *= 7.4428285367870146E+137;
        kinsim_2link_planar_B.gs_im *= 7.4428285367870146E+137;
        kinsim_2link_planar_B.scale_a *= 7.4428285367870146E+137;
      } while (!(kinsim_2link_planar_B.scale_a > 1.3435752215134178E-138));

      rescaledir = -1;
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    kinsim_2link_planar_B.scale_a = kinsim_2link_planar_B.fs_re *
      kinsim_2link_planar_B.fs_re + kinsim_2link_planar_B.fs_im *
      kinsim_2link_planar_B.fs_im;
    kinsim_2link_planar_B.g2 = kinsim_2link_planar_B.gs_re *
      kinsim_2link_planar_B.gs_re + kinsim_2link_planar_B.gs_im *
      kinsim_2link_planar_B.gs_im;
    kinsim_2link_planar_B.x = kinsim_2link_planar_B.g2;
    if (1.0 > kinsim_2link_planar_B.g2) {
      kinsim_2link_planar_B.x = 1.0;
    }

    if (kinsim_2link_planar_B.scale_a <= kinsim_2link_planar_B.x *
        2.0041683600089728E-292) {
      if ((f.re == 0.0) && (f.im == 0.0)) {
        *cs = 0.0;
        r->re = kinsim_2link_plan_rt_hypotd_snf(g.re, g.im);
        r->im = 0.0;
        kinsim_2link_planar_B.f2s = kinsim_2link_plan_rt_hypotd_snf
          (kinsim_2link_planar_B.gs_re, kinsim_2link_planar_B.gs_im);
        sn->re = kinsim_2link_planar_B.gs_re / kinsim_2link_planar_B.f2s;
        sn->im = -kinsim_2link_planar_B.gs_im / kinsim_2link_planar_B.f2s;
      } else {
        kinsim_2link_planar_B.scale_a = sqrt(kinsim_2link_planar_B.g2);
        *cs = kinsim_2link_plan_rt_hypotd_snf(kinsim_2link_planar_B.fs_re,
          kinsim_2link_planar_B.fs_im) / kinsim_2link_planar_B.scale_a;
        if (kinsim_2link_planar_B.di > kinsim_2link_planar_B.f2s) {
          kinsim_2link_planar_B.f2s = kinsim_2link_planar_B.di;
        }

        if (kinsim_2link_planar_B.f2s > 1.0) {
          kinsim_2link_planar_B.f2s = kinsim_2link_plan_rt_hypotd_snf(f.re, f.im);
          kinsim_2link_planar_B.fs_re = f.re / kinsim_2link_planar_B.f2s;
          kinsim_2link_planar_B.fs_im = f.im / kinsim_2link_planar_B.f2s;
        } else {
          kinsim_2link_planar_B.fs_re = 7.4428285367870146E+137 * f.re;
          kinsim_2link_planar_B.di = 7.4428285367870146E+137 * f.im;
          kinsim_2link_planar_B.f2s = kinsim_2link_plan_rt_hypotd_snf
            (kinsim_2link_planar_B.fs_re, kinsim_2link_planar_B.di);
          kinsim_2link_planar_B.fs_re /= kinsim_2link_planar_B.f2s;
          kinsim_2link_planar_B.fs_im = kinsim_2link_planar_B.di /
            kinsim_2link_planar_B.f2s;
        }

        kinsim_2link_planar_B.gs_re /= kinsim_2link_planar_B.scale_a;
        kinsim_2link_planar_B.gs_im = -kinsim_2link_planar_B.gs_im /
          kinsim_2link_planar_B.scale_a;
        sn->re = kinsim_2link_planar_B.fs_re * kinsim_2link_planar_B.gs_re -
          kinsim_2link_planar_B.fs_im * kinsim_2link_planar_B.gs_im;
        sn->im = kinsim_2link_planar_B.fs_re * kinsim_2link_planar_B.gs_im +
          kinsim_2link_planar_B.fs_im * kinsim_2link_planar_B.gs_re;
        r->re = (sn->re * g.re - sn->im * g.im) + *cs * f.re;
        r->im = (sn->re * g.im + sn->im * g.re) + *cs * f.im;
      }
    } else {
      kinsim_2link_planar_B.f2s = sqrt(kinsim_2link_planar_B.g2 /
        kinsim_2link_planar_B.scale_a + 1.0);
      r->re = kinsim_2link_planar_B.f2s * kinsim_2link_planar_B.fs_re;
      r->im = kinsim_2link_planar_B.f2s * kinsim_2link_planar_B.fs_im;
      *cs = 1.0 / kinsim_2link_planar_B.f2s;
      kinsim_2link_planar_B.f2s = kinsim_2link_planar_B.scale_a +
        kinsim_2link_planar_B.g2;
      kinsim_2link_planar_B.fs_re = r->re / kinsim_2link_planar_B.f2s;
      kinsim_2link_planar_B.f2s = r->im / kinsim_2link_planar_B.f2s;
      sn->re = kinsim_2link_planar_B.fs_re * kinsim_2link_planar_B.gs_re -
        kinsim_2link_planar_B.f2s * -kinsim_2link_planar_B.gs_im;
      sn->im = kinsim_2link_planar_B.fs_re * -kinsim_2link_planar_B.gs_im +
        kinsim_2link_planar_B.f2s * kinsim_2link_planar_B.gs_re;
      if (rescaledir > 0) {
        for (rescaledir = 0; rescaledir <= count; rescaledir++) {
          r->re *= 7.4428285367870146E+137;
          r->im *= 7.4428285367870146E+137;
        }
      } else {
        if (rescaledir < 0) {
          for (rescaledir = 0; rescaledir <= count; rescaledir++) {
            r->re *= 1.3435752215134178E-138;
            r->im *= 1.3435752215134178E-138;
          }
        }
      }
    }
  }
}

// Function for MATLAB Function: '<S5>/MATLAB Function'
static void kinsim_2link_planar_xzhgeqz(creal_T A[16], int32_T ilo, int32_T ihi,
  creal_T Z[16], int32_T *info, creal_T alpha1[4], creal_T beta1[4])
{
  boolean_T failed;
  boolean_T goto60;
  boolean_T goto70;
  boolean_T goto90;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  int32_T exitg1;
  boolean_T exitg2;
  *info = 0;
  alpha1[0].re = 0.0;
  alpha1[0].im = 0.0;
  beta1[0].re = 1.0;
  beta1[0].im = 0.0;
  alpha1[1].re = 0.0;
  alpha1[1].im = 0.0;
  beta1[1].re = 1.0;
  beta1[1].im = 0.0;
  alpha1[2].re = 0.0;
  alpha1[2].im = 0.0;
  beta1[2].re = 1.0;
  beta1[2].im = 0.0;
  alpha1[3].re = 0.0;
  alpha1[3].im = 0.0;
  beta1[3].re = 1.0;
  beta1[3].im = 0.0;
  kinsim_2link_planar_B.eshift_re = 0.0;
  kinsim_2link_planar_B.eshift_im = 0.0;
  kinsim_2link_planar_B.ctemp.re = 0.0;
  kinsim_2link_planar_B.ctemp.im = 0.0;
  kinsim_2link_planar_B.anorm = kinsim_2link_planar_xzlanhs(A, ilo, ihi);
  kinsim_2link_planar_B.shift_re = 2.2204460492503131E-16 *
    kinsim_2link_planar_B.anorm;
  kinsim_2link_planar_B.b_atol = 2.2250738585072014E-308;
  if (kinsim_2link_planar_B.shift_re > 2.2250738585072014E-308) {
    kinsim_2link_planar_B.b_atol = kinsim_2link_planar_B.shift_re;
  }

  kinsim_2link_planar_B.shift_re = 2.2250738585072014E-308;
  if (kinsim_2link_planar_B.anorm > 2.2250738585072014E-308) {
    kinsim_2link_planar_B.shift_re = kinsim_2link_planar_B.anorm;
  }

  kinsim_2link_planar_B.anorm = 1.0 / kinsim_2link_planar_B.shift_re;
  failed = true;
  kinsim_2link_planar_B.ilast = ihi;
  while (kinsim_2link_planar_B.ilast + 1 < 5) {
    alpha1[kinsim_2link_planar_B.ilast] = A[(kinsim_2link_planar_B.ilast << 2) +
      kinsim_2link_planar_B.ilast];
    kinsim_2link_planar_B.ilast++;
  }

  guard1 = false;
  guard2 = false;
  if (ihi >= ilo) {
    kinsim_2link_planar_B.ifirst = ilo;
    kinsim_2link_planar_B.istart = ilo;
    kinsim_2link_planar_B.ilast = ihi - 1;
    kinsim_2link_planar_B.ilastm1 = ihi - 2;
    kinsim_2link_planar_B.iiter = 0;
    goto60 = false;
    goto70 = false;
    goto90 = false;
    kinsim_2link_planar_B.jiter = 0;
    do {
      exitg1 = 0;
      if (kinsim_2link_planar_B.jiter <= ((ihi - ilo) + 1) * 30 - 1) {
        if (kinsim_2link_planar_B.ilast + 1 == ilo) {
          goto60 = true;
        } else {
          kinsim_2link_planar_B.jp1 = (kinsim_2link_planar_B.ilastm1 << 2) +
            kinsim_2link_planar_B.ilast;
          if (fabs(A[kinsim_2link_planar_B.jp1].re) + fabs
              (A[kinsim_2link_planar_B.jp1].im) <= kinsim_2link_planar_B.b_atol)
          {
            A[kinsim_2link_planar_B.jp1].re = 0.0;
            A[kinsim_2link_planar_B.jp1].im = 0.0;
            goto60 = true;
          } else {
            kinsim_2link_planar_B.j = kinsim_2link_planar_B.ilastm1;
            guard3 = false;
            exitg2 = false;
            while ((!exitg2) && (kinsim_2link_planar_B.j + 1 >= ilo)) {
              if (kinsim_2link_planar_B.j + 1 == ilo) {
                guard3 = true;
                exitg2 = true;
              } else {
                kinsim_2link_planar_B.jp1 = ((kinsim_2link_planar_B.j - 1) << 2)
                  + kinsim_2link_planar_B.j;
                if (fabs(A[kinsim_2link_planar_B.jp1].re) + fabs
                    (A[kinsim_2link_planar_B.jp1].im) <=
                    kinsim_2link_planar_B.b_atol) {
                  A[kinsim_2link_planar_B.jp1].re = 0.0;
                  A[kinsim_2link_planar_B.jp1].im = 0.0;
                  guard3 = true;
                  exitg2 = true;
                } else {
                  kinsim_2link_planar_B.j--;
                  guard3 = false;
                }
              }
            }

            if (guard3) {
              kinsim_2link_planar_B.ifirst = kinsim_2link_planar_B.j + 1;
              goto70 = true;
            }
          }
        }

        if ((!goto60) && (!goto70)) {
          alpha1[0].re = (rtNaN);
          alpha1[0].im = 0.0;
          beta1[0].re = (rtNaN);
          beta1[0].im = 0.0;
          alpha1[1].re = (rtNaN);
          alpha1[1].im = 0.0;
          beta1[1].re = (rtNaN);
          beta1[1].im = 0.0;
          alpha1[2].re = (rtNaN);
          alpha1[2].im = 0.0;
          beta1[2].re = (rtNaN);
          beta1[2].im = 0.0;
          alpha1[3].re = (rtNaN);
          alpha1[3].im = 0.0;
          beta1[3].re = (rtNaN);
          beta1[3].im = 0.0;
          for (kinsim_2link_planar_B.jp1 = 0; kinsim_2link_planar_B.jp1 < 16;
               kinsim_2link_planar_B.jp1++) {
            Z[kinsim_2link_planar_B.jp1].re = (rtNaN);
            Z[kinsim_2link_planar_B.jp1].im = 0.0;
          }

          *info = 1;
          exitg1 = 1;
        } else if (goto60) {
          goto60 = false;
          alpha1[kinsim_2link_planar_B.ilast] = A[(kinsim_2link_planar_B.ilast <<
            2) + kinsim_2link_planar_B.ilast];
          kinsim_2link_planar_B.ilast = kinsim_2link_planar_B.ilastm1;
          kinsim_2link_planar_B.ilastm1--;
          if (kinsim_2link_planar_B.ilast + 1 < ilo) {
            failed = false;
            guard2 = true;
            exitg1 = 1;
          } else {
            kinsim_2link_planar_B.iiter = 0;
            kinsim_2link_planar_B.eshift_re = 0.0;
            kinsim_2link_planar_B.eshift_im = 0.0;
            kinsim_2link_planar_B.jiter++;
          }
        } else {
          if (goto70) {
            goto70 = false;
            kinsim_2link_planar_B.iiter++;
            if (kinsim_2link_planar_B.iiter - div_nzp_s32
                (kinsim_2link_planar_B.iiter, 10) * 10 != 0) {
              kinsim_2link_planar_B.j = (kinsim_2link_planar_B.ilastm1 << 2) +
                kinsim_2link_planar_B.ilastm1;
              kinsim_2link_planar_B.ar = A[kinsim_2link_planar_B.j].re *
                kinsim_2link_planar_B.anorm;
              kinsim_2link_planar_B.ai = A[kinsim_2link_planar_B.j].im *
                kinsim_2link_planar_B.anorm;
              if (kinsim_2link_planar_B.ai == 0.0) {
                kinsim_2link_planar_B.shift_re = kinsim_2link_planar_B.ar / 0.5;
                kinsim_2link_planar_B.shift_im = 0.0;
              } else if (kinsim_2link_planar_B.ar == 0.0) {
                kinsim_2link_planar_B.shift_re = 0.0;
                kinsim_2link_planar_B.shift_im = kinsim_2link_planar_B.ai / 0.5;
              } else {
                kinsim_2link_planar_B.shift_re = kinsim_2link_planar_B.ar / 0.5;
                kinsim_2link_planar_B.shift_im = kinsim_2link_planar_B.ai / 0.5;
              }

              kinsim_2link_planar_B.j = (kinsim_2link_planar_B.ilast << 2) +
                kinsim_2link_planar_B.ilast;
              kinsim_2link_planar_B.ar = A[kinsim_2link_planar_B.j].re *
                kinsim_2link_planar_B.anorm;
              kinsim_2link_planar_B.ai = A[kinsim_2link_planar_B.j].im *
                kinsim_2link_planar_B.anorm;
              if (kinsim_2link_planar_B.ai == 0.0) {
                kinsim_2link_planar_B.ad22.re = kinsim_2link_planar_B.ar / 0.5;
                kinsim_2link_planar_B.ad22.im = 0.0;
              } else if (kinsim_2link_planar_B.ar == 0.0) {
                kinsim_2link_planar_B.ad22.re = 0.0;
                kinsim_2link_planar_B.ad22.im = kinsim_2link_planar_B.ai / 0.5;
              } else {
                kinsim_2link_planar_B.ad22.re = kinsim_2link_planar_B.ar / 0.5;
                kinsim_2link_planar_B.ad22.im = kinsim_2link_planar_B.ai / 0.5;
              }

              kinsim_2link_planar_B.t1_re = (kinsim_2link_planar_B.shift_re +
                kinsim_2link_planar_B.ad22.re) * 0.5;
              kinsim_2link_planar_B.t1_im = (kinsim_2link_planar_B.shift_im +
                kinsim_2link_planar_B.ad22.im) * 0.5;
              kinsim_2link_planar_B.j = (kinsim_2link_planar_B.ilast << 2) +
                kinsim_2link_planar_B.ilastm1;
              kinsim_2link_planar_B.ar = A[kinsim_2link_planar_B.j].re *
                kinsim_2link_planar_B.anorm;
              kinsim_2link_planar_B.ai = A[kinsim_2link_planar_B.j].im *
                kinsim_2link_planar_B.anorm;
              if (kinsim_2link_planar_B.ai == 0.0) {
                kinsim_2link_planar_B.absxr = kinsim_2link_planar_B.ar / 0.5;
                kinsim_2link_planar_B.absxi = 0.0;
              } else if (kinsim_2link_planar_B.ar == 0.0) {
                kinsim_2link_planar_B.absxr = 0.0;
                kinsim_2link_planar_B.absxi = kinsim_2link_planar_B.ai / 0.5;
              } else {
                kinsim_2link_planar_B.absxr = kinsim_2link_planar_B.ar / 0.5;
                kinsim_2link_planar_B.absxi = kinsim_2link_planar_B.ai / 0.5;
              }

              kinsim_2link_planar_B.j = (kinsim_2link_planar_B.ilastm1 << 2) +
                kinsim_2link_planar_B.ilast;
              kinsim_2link_planar_B.ar = A[kinsim_2link_planar_B.j].re *
                kinsim_2link_planar_B.anorm;
              kinsim_2link_planar_B.ai = A[kinsim_2link_planar_B.j].im *
                kinsim_2link_planar_B.anorm;
              if (kinsim_2link_planar_B.ai == 0.0) {
                kinsim_2link_planar_B.ar /= 0.5;
                kinsim_2link_planar_B.ai = 0.0;
              } else if (kinsim_2link_planar_B.ar == 0.0) {
                kinsim_2link_planar_B.ar = 0.0;
                kinsim_2link_planar_B.ai /= 0.5;
              } else {
                kinsim_2link_planar_B.ar /= 0.5;
                kinsim_2link_planar_B.ai /= 0.5;
              }

              kinsim_2link_planar_B.shift_im_f = kinsim_2link_planar_B.shift_re *
                kinsim_2link_planar_B.ad22.im + kinsim_2link_planar_B.shift_im *
                kinsim_2link_planar_B.ad22.re;
              kinsim_2link_planar_B.shift_re = ((kinsim_2link_planar_B.t1_re *
                kinsim_2link_planar_B.t1_re - kinsim_2link_planar_B.t1_im *
                kinsim_2link_planar_B.t1_im) + (kinsim_2link_planar_B.absxr *
                kinsim_2link_planar_B.ar - kinsim_2link_planar_B.absxi *
                kinsim_2link_planar_B.ai)) - (kinsim_2link_planar_B.shift_re *
                kinsim_2link_planar_B.ad22.re - kinsim_2link_planar_B.shift_im *
                kinsim_2link_planar_B.ad22.im);
              kinsim_2link_planar_B.shift_im = kinsim_2link_planar_B.t1_re *
                kinsim_2link_planar_B.t1_im;
              kinsim_2link_planar_B.shift_im = ((kinsim_2link_planar_B.shift_im
                + kinsim_2link_planar_B.shift_im) + (kinsim_2link_planar_B.absxr
                * kinsim_2link_planar_B.ai + kinsim_2link_planar_B.absxi *
                kinsim_2link_planar_B.ar)) - kinsim_2link_planar_B.shift_im_f;
              if (kinsim_2link_planar_B.shift_im == 0.0) {
                if (kinsim_2link_planar_B.shift_re < 0.0) {
                  kinsim_2link_planar_B.absxr = 0.0;
                  kinsim_2link_planar_B.absxi = sqrt
                    (-kinsim_2link_planar_B.shift_re);
                } else {
                  kinsim_2link_planar_B.absxr = sqrt
                    (kinsim_2link_planar_B.shift_re);
                  kinsim_2link_planar_B.absxi = 0.0;
                }
              } else if (kinsim_2link_planar_B.shift_re == 0.0) {
                if (kinsim_2link_planar_B.shift_im < 0.0) {
                  kinsim_2link_planar_B.absxr = sqrt
                    (-kinsim_2link_planar_B.shift_im / 2.0);
                  kinsim_2link_planar_B.absxi = -kinsim_2link_planar_B.absxr;
                } else {
                  kinsim_2link_planar_B.absxr = sqrt
                    (kinsim_2link_planar_B.shift_im / 2.0);
                  kinsim_2link_planar_B.absxi = kinsim_2link_planar_B.absxr;
                }
              } else if (rtIsNaN(kinsim_2link_planar_B.shift_re)) {
                kinsim_2link_planar_B.absxr = kinsim_2link_planar_B.shift_re;
                kinsim_2link_planar_B.absxi = kinsim_2link_planar_B.shift_re;
              } else if (rtIsNaN(kinsim_2link_planar_B.shift_im)) {
                kinsim_2link_planar_B.absxr = kinsim_2link_planar_B.shift_im;
                kinsim_2link_planar_B.absxi = kinsim_2link_planar_B.shift_im;
              } else if (rtIsInf(kinsim_2link_planar_B.shift_im)) {
                kinsim_2link_planar_B.absxr = fabs
                  (kinsim_2link_planar_B.shift_im);
                kinsim_2link_planar_B.absxi = kinsim_2link_planar_B.shift_im;
              } else if (rtIsInf(kinsim_2link_planar_B.shift_re)) {
                if (kinsim_2link_planar_B.shift_re < 0.0) {
                  kinsim_2link_planar_B.absxr = 0.0;
                  kinsim_2link_planar_B.absxi = kinsim_2link_planar_B.shift_im *
                    -kinsim_2link_planar_B.shift_re;
                } else {
                  kinsim_2link_planar_B.absxr = kinsim_2link_planar_B.shift_re;
                  kinsim_2link_planar_B.absxi = 0.0;
                }
              } else {
                kinsim_2link_planar_B.absxr = fabs
                  (kinsim_2link_planar_B.shift_re);
                kinsim_2link_planar_B.absxi = fabs
                  (kinsim_2link_planar_B.shift_im);
                if ((kinsim_2link_planar_B.absxr > 4.4942328371557893E+307) ||
                    (kinsim_2link_planar_B.absxi > 4.4942328371557893E+307)) {
                  kinsim_2link_planar_B.absxr *= 0.5;
                  kinsim_2link_planar_B.absxi *= 0.5;
                  kinsim_2link_planar_B.absxi = kinsim_2link_plan_rt_hypotd_snf
                    (kinsim_2link_planar_B.absxr, kinsim_2link_planar_B.absxi);
                  if (kinsim_2link_planar_B.absxi > kinsim_2link_planar_B.absxr)
                  {
                    kinsim_2link_planar_B.absxr = sqrt
                      (kinsim_2link_planar_B.absxr / kinsim_2link_planar_B.absxi
                       + 1.0) * sqrt(kinsim_2link_planar_B.absxi);
                  } else {
                    kinsim_2link_planar_B.absxr = sqrt
                      (kinsim_2link_planar_B.absxi) * 1.4142135623730951;
                  }
                } else {
                  kinsim_2link_planar_B.absxr = sqrt
                    ((kinsim_2link_plan_rt_hypotd_snf
                      (kinsim_2link_planar_B.absxr, kinsim_2link_planar_B.absxi)
                      + kinsim_2link_planar_B.absxr) * 0.5);
                }

                if (kinsim_2link_planar_B.shift_re > 0.0) {
                  kinsim_2link_planar_B.absxi = kinsim_2link_planar_B.shift_im /
                    kinsim_2link_planar_B.absxr * 0.5;
                } else {
                  if (kinsim_2link_planar_B.shift_im < 0.0) {
                    kinsim_2link_planar_B.absxi = -kinsim_2link_planar_B.absxr;
                  } else {
                    kinsim_2link_planar_B.absxi = kinsim_2link_planar_B.absxr;
                  }

                  kinsim_2link_planar_B.absxr = kinsim_2link_planar_B.shift_im /
                    kinsim_2link_planar_B.absxi * 0.5;
                }
              }

              if ((kinsim_2link_planar_B.t1_re - kinsim_2link_planar_B.ad22.re) *
                  kinsim_2link_planar_B.absxr + (kinsim_2link_planar_B.t1_im -
                   kinsim_2link_planar_B.ad22.im) * kinsim_2link_planar_B.absxi <=
                  0.0) {
                kinsim_2link_planar_B.shift_re = kinsim_2link_planar_B.t1_re +
                  kinsim_2link_planar_B.absxr;
                kinsim_2link_planar_B.shift_im = kinsim_2link_planar_B.t1_im +
                  kinsim_2link_planar_B.absxi;
              } else {
                kinsim_2link_planar_B.shift_re = kinsim_2link_planar_B.t1_re -
                  kinsim_2link_planar_B.absxr;
                kinsim_2link_planar_B.shift_im = kinsim_2link_planar_B.t1_im -
                  kinsim_2link_planar_B.absxi;
              }
            } else {
              kinsim_2link_planar_B.j = (kinsim_2link_planar_B.ilastm1 << 2) +
                kinsim_2link_planar_B.ilast;
              kinsim_2link_planar_B.ar = A[kinsim_2link_planar_B.j].re *
                kinsim_2link_planar_B.anorm;
              kinsim_2link_planar_B.ai = A[kinsim_2link_planar_B.j].im *
                kinsim_2link_planar_B.anorm;
              if (kinsim_2link_planar_B.ai == 0.0) {
                kinsim_2link_planar_B.absxr = kinsim_2link_planar_B.ar / 0.5;
                kinsim_2link_planar_B.absxi = 0.0;
              } else if (kinsim_2link_planar_B.ar == 0.0) {
                kinsim_2link_planar_B.absxr = 0.0;
                kinsim_2link_planar_B.absxi = kinsim_2link_planar_B.ai / 0.5;
              } else {
                kinsim_2link_planar_B.absxr = kinsim_2link_planar_B.ar / 0.5;
                kinsim_2link_planar_B.absxi = kinsim_2link_planar_B.ai / 0.5;
              }

              kinsim_2link_planar_B.eshift_re += kinsim_2link_planar_B.absxr;
              kinsim_2link_planar_B.eshift_im += kinsim_2link_planar_B.absxi;
              kinsim_2link_planar_B.shift_re = kinsim_2link_planar_B.eshift_re;
              kinsim_2link_planar_B.shift_im = kinsim_2link_planar_B.eshift_im;
            }

            kinsim_2link_planar_B.j = kinsim_2link_planar_B.ilastm1;
            kinsim_2link_planar_B.jp1 = kinsim_2link_planar_B.ilastm1 + 1;
            exitg2 = false;
            while ((!exitg2) && (kinsim_2link_planar_B.j + 1 >
                                 kinsim_2link_planar_B.ifirst)) {
              kinsim_2link_planar_B.istart = kinsim_2link_planar_B.j + 1;
              kinsim_2link_planar_B.ctemp_tmp_tmp = kinsim_2link_planar_B.j << 2;
              kinsim_2link_planar_B.ctemp_tmp =
                kinsim_2link_planar_B.ctemp_tmp_tmp + kinsim_2link_planar_B.j;
              kinsim_2link_planar_B.ctemp.re = A[kinsim_2link_planar_B.ctemp_tmp]
                .re * kinsim_2link_planar_B.anorm -
                kinsim_2link_planar_B.shift_re * 0.5;
              kinsim_2link_planar_B.ctemp.im = A[kinsim_2link_planar_B.ctemp_tmp]
                .im * kinsim_2link_planar_B.anorm -
                kinsim_2link_planar_B.shift_im * 0.5;
              kinsim_2link_planar_B.t1_re = fabs(kinsim_2link_planar_B.ctemp.re)
                + fabs(kinsim_2link_planar_B.ctemp.im);
              kinsim_2link_planar_B.jp1 += kinsim_2link_planar_B.ctemp_tmp_tmp;
              kinsim_2link_planar_B.t1_im = (fabs(A[kinsim_2link_planar_B.jp1].
                re) + fabs(A[kinsim_2link_planar_B.jp1].im)) *
                kinsim_2link_planar_B.anorm;
              kinsim_2link_planar_B.absxr = kinsim_2link_planar_B.t1_re;
              if (kinsim_2link_planar_B.t1_im > kinsim_2link_planar_B.t1_re) {
                kinsim_2link_planar_B.absxr = kinsim_2link_planar_B.t1_im;
              }

              if ((kinsim_2link_planar_B.absxr < 1.0) &&
                  (kinsim_2link_planar_B.absxr != 0.0)) {
                kinsim_2link_planar_B.t1_re /= kinsim_2link_planar_B.absxr;
                kinsim_2link_planar_B.t1_im /= kinsim_2link_planar_B.absxr;
              }

              kinsim_2link_planar_B.jp1 = ((kinsim_2link_planar_B.j - 1) << 2) +
                kinsim_2link_planar_B.j;
              if ((fabs(A[kinsim_2link_planar_B.jp1].re) + fabs
                   (A[kinsim_2link_planar_B.jp1].im)) *
                  kinsim_2link_planar_B.t1_im <= kinsim_2link_planar_B.t1_re *
                  kinsim_2link_planar_B.b_atol) {
                goto90 = true;
                exitg2 = true;
              } else {
                kinsim_2link_planar_B.jp1 = kinsim_2link_planar_B.j;
                kinsim_2link_planar_B.j--;
              }
            }

            if (!goto90) {
              kinsim_2link_planar_B.istart = kinsim_2link_planar_B.ifirst;
              kinsim_2link_planar_B.ctemp_tmp = (((kinsim_2link_planar_B.ifirst
                - 1) << 2) + kinsim_2link_planar_B.ifirst) - 1;
              kinsim_2link_planar_B.ctemp.re = A[kinsim_2link_planar_B.ctemp_tmp]
                .re * kinsim_2link_planar_B.anorm -
                kinsim_2link_planar_B.shift_re * 0.5;
              kinsim_2link_planar_B.ctemp.im = A[kinsim_2link_planar_B.ctemp_tmp]
                .im * kinsim_2link_planar_B.anorm -
                kinsim_2link_planar_B.shift_im * 0.5;
            }

            goto90 = false;
            kinsim_2link_planar_B.j = ((kinsim_2link_planar_B.istart - 1) << 2)
              + kinsim_2link_planar_B.istart;
            kinsim_2link_planar_B.ascale.re = A[kinsim_2link_planar_B.j].re *
              kinsim_2link_planar_B.anorm;
            kinsim_2link_planar_B.ascale.im = A[kinsim_2link_planar_B.j].im *
              kinsim_2link_planar_B.anorm;
            kinsim_2link_planar_xzlartg_a(kinsim_2link_planar_B.ctemp,
              kinsim_2link_planar_B.ascale, &kinsim_2link_planar_B.t1_re,
              &kinsim_2link_planar_B.ad22);
            kinsim_2link_planar_B.j = kinsim_2link_planar_B.istart;
            kinsim_2link_planar_B.jp1 = kinsim_2link_planar_B.istart - 2;
            while (kinsim_2link_planar_B.j < kinsim_2link_planar_B.ilast + 1) {
              if (kinsim_2link_planar_B.j > kinsim_2link_planar_B.istart) {
                kinsim_2link_planar_xzlartg(A[(kinsim_2link_planar_B.j +
                  (kinsim_2link_planar_B.jp1 << 2)) - 1],
                  A[kinsim_2link_planar_B.j + (kinsim_2link_planar_B.jp1 << 2)],
                  &kinsim_2link_planar_B.t1_re, &kinsim_2link_planar_B.ad22, &A
                  [(kinsim_2link_planar_B.j + (kinsim_2link_planar_B.jp1 << 2))
                  - 1]);
                kinsim_2link_planar_B.jp1 = kinsim_2link_planar_B.j +
                  (kinsim_2link_planar_B.jp1 << 2);
                A[kinsim_2link_planar_B.jp1].re = 0.0;
                A[kinsim_2link_planar_B.jp1].im = 0.0;
              }

              kinsim_2link_planar_B.ctemp_tmp = kinsim_2link_planar_B.j - 1;
              while (kinsim_2link_planar_B.ctemp_tmp + 1 < 5) {
                kinsim_2link_planar_B.jp1 = (kinsim_2link_planar_B.ctemp_tmp <<
                  2) + kinsim_2link_planar_B.j;
                kinsim_2link_planar_B.ctemp_tmp_tmp = kinsim_2link_planar_B.jp1
                  - 1;
                kinsim_2link_planar_B.shift_re =
                  A[kinsim_2link_planar_B.ctemp_tmp_tmp].re *
                  kinsim_2link_planar_B.t1_re + (A[kinsim_2link_planar_B.jp1].re
                  * kinsim_2link_planar_B.ad22.re - A[kinsim_2link_planar_B.jp1]
                  .im * kinsim_2link_planar_B.ad22.im);
                kinsim_2link_planar_B.shift_im =
                  A[kinsim_2link_planar_B.ctemp_tmp_tmp].im *
                  kinsim_2link_planar_B.t1_re + (A[kinsim_2link_planar_B.jp1].im
                  * kinsim_2link_planar_B.ad22.re + A[kinsim_2link_planar_B.jp1]
                  .re * kinsim_2link_planar_B.ad22.im);
                kinsim_2link_planar_B.t1_im =
                  A[kinsim_2link_planar_B.ctemp_tmp_tmp].im;
                kinsim_2link_planar_B.absxr =
                  A[kinsim_2link_planar_B.ctemp_tmp_tmp].re;
                A[kinsim_2link_planar_B.jp1].re = A[kinsim_2link_planar_B.jp1].
                  re * kinsim_2link_planar_B.t1_re -
                  (A[kinsim_2link_planar_B.ctemp_tmp_tmp].re *
                   kinsim_2link_planar_B.ad22.re +
                   A[kinsim_2link_planar_B.ctemp_tmp_tmp].im *
                   kinsim_2link_planar_B.ad22.im);
                A[kinsim_2link_planar_B.jp1].im = A[kinsim_2link_planar_B.jp1].
                  im * kinsim_2link_planar_B.t1_re -
                  (kinsim_2link_planar_B.ad22.re * kinsim_2link_planar_B.t1_im -
                   kinsim_2link_planar_B.ad22.im * kinsim_2link_planar_B.absxr);
                A[kinsim_2link_planar_B.ctemp_tmp_tmp].re =
                  kinsim_2link_planar_B.shift_re;
                A[kinsim_2link_planar_B.ctemp_tmp_tmp].im =
                  kinsim_2link_planar_B.shift_im;
                kinsim_2link_planar_B.ctemp_tmp++;
              }

              kinsim_2link_planar_B.ad22.re = -kinsim_2link_planar_B.ad22.re;
              kinsim_2link_planar_B.ad22.im = -kinsim_2link_planar_B.ad22.im;
              kinsim_2link_planar_B.ctemp_tmp = kinsim_2link_planar_B.j;
              if (kinsim_2link_planar_B.ilast + 1 < kinsim_2link_planar_B.j + 2)
              {
                kinsim_2link_planar_B.ctemp_tmp = kinsim_2link_planar_B.ilast -
                  1;
              }

              kinsim_2link_planar_B.i_h2 = 0;
              while (kinsim_2link_planar_B.i_h2 + 1 <=
                     kinsim_2link_planar_B.ctemp_tmp + 2) {
                kinsim_2link_planar_B.jp1 = ((kinsim_2link_planar_B.j - 1) << 2)
                  + kinsim_2link_planar_B.i_h2;
                kinsim_2link_planar_B.ctemp_tmp_tmp = (kinsim_2link_planar_B.j <<
                  2) + kinsim_2link_planar_B.i_h2;
                kinsim_2link_planar_B.shift_re = (A[kinsim_2link_planar_B.jp1].
                  re * kinsim_2link_planar_B.ad22.re -
                  A[kinsim_2link_planar_B.jp1].im *
                  kinsim_2link_planar_B.ad22.im) +
                  A[kinsim_2link_planar_B.ctemp_tmp_tmp].re *
                  kinsim_2link_planar_B.t1_re;
                kinsim_2link_planar_B.shift_im = (A[kinsim_2link_planar_B.jp1].
                  im * kinsim_2link_planar_B.ad22.re +
                  A[kinsim_2link_planar_B.jp1].re *
                  kinsim_2link_planar_B.ad22.im) +
                  A[kinsim_2link_planar_B.ctemp_tmp_tmp].im *
                  kinsim_2link_planar_B.t1_re;
                kinsim_2link_planar_B.t1_im =
                  A[kinsim_2link_planar_B.ctemp_tmp_tmp].im;
                kinsim_2link_planar_B.absxr =
                  A[kinsim_2link_planar_B.ctemp_tmp_tmp].re;
                A[kinsim_2link_planar_B.jp1].re = A[kinsim_2link_planar_B.jp1].
                  re * kinsim_2link_planar_B.t1_re -
                  (A[kinsim_2link_planar_B.ctemp_tmp_tmp].re *
                   kinsim_2link_planar_B.ad22.re +
                   A[kinsim_2link_planar_B.ctemp_tmp_tmp].im *
                   kinsim_2link_planar_B.ad22.im);
                A[kinsim_2link_planar_B.jp1].im = A[kinsim_2link_planar_B.jp1].
                  im * kinsim_2link_planar_B.t1_re -
                  (kinsim_2link_planar_B.ad22.re * kinsim_2link_planar_B.t1_im -
                   kinsim_2link_planar_B.ad22.im * kinsim_2link_planar_B.absxr);
                A[kinsim_2link_planar_B.ctemp_tmp_tmp].re =
                  kinsim_2link_planar_B.shift_re;
                A[kinsim_2link_planar_B.ctemp_tmp_tmp].im =
                  kinsim_2link_planar_B.shift_im;
                kinsim_2link_planar_B.i_h2++;
              }

              kinsim_2link_planar_B.jp1 = (kinsim_2link_planar_B.j - 1) << 2;
              kinsim_2link_planar_B.ctemp_tmp_tmp = kinsim_2link_planar_B.j << 2;
              kinsim_2link_planar_B.shift_re = (Z[kinsim_2link_planar_B.jp1].re *
                kinsim_2link_planar_B.ad22.re - Z[kinsim_2link_planar_B.jp1].im *
                kinsim_2link_planar_B.ad22.im) +
                Z[kinsim_2link_planar_B.ctemp_tmp_tmp].re *
                kinsim_2link_planar_B.t1_re;
              kinsim_2link_planar_B.shift_im = (Z[kinsim_2link_planar_B.jp1].im *
                kinsim_2link_planar_B.ad22.re + Z[kinsim_2link_planar_B.jp1].re *
                kinsim_2link_planar_B.ad22.im) +
                Z[kinsim_2link_planar_B.ctemp_tmp_tmp].im *
                kinsim_2link_planar_B.t1_re;
              kinsim_2link_planar_B.t1_im =
                Z[kinsim_2link_planar_B.ctemp_tmp_tmp].im;
              kinsim_2link_planar_B.absxr =
                Z[kinsim_2link_planar_B.ctemp_tmp_tmp].re;
              Z[kinsim_2link_planar_B.jp1].re = Z[kinsim_2link_planar_B.jp1].re *
                kinsim_2link_planar_B.t1_re -
                (Z[kinsim_2link_planar_B.ctemp_tmp_tmp].re *
                 kinsim_2link_planar_B.ad22.re +
                 Z[kinsim_2link_planar_B.ctemp_tmp_tmp].im *
                 kinsim_2link_planar_B.ad22.im);
              Z[kinsim_2link_planar_B.jp1].im = Z[kinsim_2link_planar_B.jp1].im *
                kinsim_2link_planar_B.t1_re - (kinsim_2link_planar_B.ad22.re *
                kinsim_2link_planar_B.t1_im - kinsim_2link_planar_B.ad22.im *
                kinsim_2link_planar_B.absxr);
              Z[kinsim_2link_planar_B.ctemp_tmp_tmp].re =
                kinsim_2link_planar_B.shift_re;
              Z[kinsim_2link_planar_B.ctemp_tmp_tmp].im =
                kinsim_2link_planar_B.shift_im;
              kinsim_2link_planar_B.ctemp_tmp = kinsim_2link_planar_B.jp1 + 1;
              kinsim_2link_planar_B.i_h2 = kinsim_2link_planar_B.ctemp_tmp_tmp +
                1;
              kinsim_2link_planar_B.shift_re =
                (Z[kinsim_2link_planar_B.ctemp_tmp].re *
                 kinsim_2link_planar_B.ad22.re -
                 Z[kinsim_2link_planar_B.ctemp_tmp].im *
                 kinsim_2link_planar_B.ad22.im) + Z[kinsim_2link_planar_B.i_h2].
                re * kinsim_2link_planar_B.t1_re;
              kinsim_2link_planar_B.shift_im =
                (Z[kinsim_2link_planar_B.ctemp_tmp].im *
                 kinsim_2link_planar_B.ad22.re +
                 Z[kinsim_2link_planar_B.ctemp_tmp].re *
                 kinsim_2link_planar_B.ad22.im) + Z[kinsim_2link_planar_B.i_h2].
                im * kinsim_2link_planar_B.t1_re;
              kinsim_2link_planar_B.t1_im = Z[kinsim_2link_planar_B.i_h2].im;
              kinsim_2link_planar_B.absxr = Z[kinsim_2link_planar_B.i_h2].re;
              Z[kinsim_2link_planar_B.ctemp_tmp].re =
                Z[kinsim_2link_planar_B.ctemp_tmp].re *
                kinsim_2link_planar_B.t1_re - (Z[kinsim_2link_planar_B.i_h2].re *
                kinsim_2link_planar_B.ad22.re + Z[kinsim_2link_planar_B.i_h2].im
                * kinsim_2link_planar_B.ad22.im);
              Z[kinsim_2link_planar_B.ctemp_tmp].im =
                Z[kinsim_2link_planar_B.ctemp_tmp].im *
                kinsim_2link_planar_B.t1_re - (kinsim_2link_planar_B.ad22.re *
                kinsim_2link_planar_B.t1_im - kinsim_2link_planar_B.ad22.im *
                kinsim_2link_planar_B.absxr);
              Z[kinsim_2link_planar_B.i_h2].re = kinsim_2link_planar_B.shift_re;
              Z[kinsim_2link_planar_B.i_h2].im = kinsim_2link_planar_B.shift_im;
              kinsim_2link_planar_B.ctemp_tmp = kinsim_2link_planar_B.jp1 + 2;
              kinsim_2link_planar_B.i_h2 = kinsim_2link_planar_B.ctemp_tmp_tmp +
                2;
              kinsim_2link_planar_B.shift_re =
                (Z[kinsim_2link_planar_B.ctemp_tmp].re *
                 kinsim_2link_planar_B.ad22.re -
                 Z[kinsim_2link_planar_B.ctemp_tmp].im *
                 kinsim_2link_planar_B.ad22.im) + Z[kinsim_2link_planar_B.i_h2].
                re * kinsim_2link_planar_B.t1_re;
              kinsim_2link_planar_B.shift_im =
                (Z[kinsim_2link_planar_B.ctemp_tmp].im *
                 kinsim_2link_planar_B.ad22.re +
                 Z[kinsim_2link_planar_B.ctemp_tmp].re *
                 kinsim_2link_planar_B.ad22.im) + Z[kinsim_2link_planar_B.i_h2].
                im * kinsim_2link_planar_B.t1_re;
              kinsim_2link_planar_B.t1_im = Z[kinsim_2link_planar_B.i_h2].im;
              kinsim_2link_planar_B.absxr = Z[kinsim_2link_planar_B.i_h2].re;
              Z[kinsim_2link_planar_B.ctemp_tmp].re =
                Z[kinsim_2link_planar_B.ctemp_tmp].re *
                kinsim_2link_planar_B.t1_re - (Z[kinsim_2link_planar_B.i_h2].re *
                kinsim_2link_planar_B.ad22.re + Z[kinsim_2link_planar_B.i_h2].im
                * kinsim_2link_planar_B.ad22.im);
              Z[kinsim_2link_planar_B.ctemp_tmp].im =
                Z[kinsim_2link_planar_B.ctemp_tmp].im *
                kinsim_2link_planar_B.t1_re - (kinsim_2link_planar_B.ad22.re *
                kinsim_2link_planar_B.t1_im - kinsim_2link_planar_B.ad22.im *
                kinsim_2link_planar_B.absxr);
              Z[kinsim_2link_planar_B.i_h2].re = kinsim_2link_planar_B.shift_re;
              Z[kinsim_2link_planar_B.i_h2].im = kinsim_2link_planar_B.shift_im;
              kinsim_2link_planar_B.jp1 += 3;
              kinsim_2link_planar_B.ctemp_tmp_tmp += 3;
              kinsim_2link_planar_B.shift_re = (Z[kinsim_2link_planar_B.jp1].re *
                kinsim_2link_planar_B.ad22.re - Z[kinsim_2link_planar_B.jp1].im *
                kinsim_2link_planar_B.ad22.im) +
                Z[kinsim_2link_planar_B.ctemp_tmp_tmp].re *
                kinsim_2link_planar_B.t1_re;
              kinsim_2link_planar_B.shift_im = (Z[kinsim_2link_planar_B.jp1].im *
                kinsim_2link_planar_B.ad22.re + Z[kinsim_2link_planar_B.jp1].re *
                kinsim_2link_planar_B.ad22.im) +
                Z[kinsim_2link_planar_B.ctemp_tmp_tmp].im *
                kinsim_2link_planar_B.t1_re;
              kinsim_2link_planar_B.t1_im =
                Z[kinsim_2link_planar_B.ctemp_tmp_tmp].im;
              kinsim_2link_planar_B.absxr =
                Z[kinsim_2link_planar_B.ctemp_tmp_tmp].re;
              Z[kinsim_2link_planar_B.jp1].re = Z[kinsim_2link_planar_B.jp1].re *
                kinsim_2link_planar_B.t1_re -
                (Z[kinsim_2link_planar_B.ctemp_tmp_tmp].re *
                 kinsim_2link_planar_B.ad22.re +
                 Z[kinsim_2link_planar_B.ctemp_tmp_tmp].im *
                 kinsim_2link_planar_B.ad22.im);
              Z[kinsim_2link_planar_B.jp1].im = Z[kinsim_2link_planar_B.jp1].im *
                kinsim_2link_planar_B.t1_re - (kinsim_2link_planar_B.ad22.re *
                kinsim_2link_planar_B.t1_im - kinsim_2link_planar_B.ad22.im *
                kinsim_2link_planar_B.absxr);
              Z[kinsim_2link_planar_B.ctemp_tmp_tmp].re =
                kinsim_2link_planar_B.shift_re;
              Z[kinsim_2link_planar_B.ctemp_tmp_tmp].im =
                kinsim_2link_planar_B.shift_im;
              kinsim_2link_planar_B.jp1 = kinsim_2link_planar_B.j - 1;
              kinsim_2link_planar_B.j++;
            }
          }

          kinsim_2link_planar_B.jiter++;
        }
      } else {
        guard2 = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  } else {
    guard1 = true;
  }

  if (guard2) {
    if (failed) {
      *info = kinsim_2link_planar_B.ilast + 1;
      kinsim_2link_planar_B.ifirst = 0;
      while (kinsim_2link_planar_B.ifirst <= kinsim_2link_planar_B.ilast) {
        alpha1[kinsim_2link_planar_B.ifirst].re = (rtNaN);
        alpha1[kinsim_2link_planar_B.ifirst].im = 0.0;
        beta1[kinsim_2link_planar_B.ifirst].re = (rtNaN);
        beta1[kinsim_2link_planar_B.ifirst].im = 0.0;
        kinsim_2link_planar_B.ifirst++;
      }

      for (kinsim_2link_planar_B.jp1 = 0; kinsim_2link_planar_B.jp1 < 16;
           kinsim_2link_planar_B.jp1++) {
        Z[kinsim_2link_planar_B.jp1].re = (rtNaN);
        Z[kinsim_2link_planar_B.jp1].im = 0.0;
      }
    } else {
      guard1 = true;
    }
  }

  if (guard1) {
    kinsim_2link_planar_B.ilast = 0;
    while (kinsim_2link_planar_B.ilast <= ilo - 2) {
      alpha1[kinsim_2link_planar_B.ilast] = A[(kinsim_2link_planar_B.ilast << 2)
        + kinsim_2link_planar_B.ilast];
      kinsim_2link_planar_B.ilast++;
    }
  }
}

// Function for MATLAB Function: '<S5>/MATLAB Function'
static void kinsim_2link_planar_xztgevc(const creal_T A[16], creal_T V[16])
{
  boolean_T lscalea;
  boolean_T lscaleb;
  int32_T work2_idx_1_re_tmp;
  int32_T d_re_tmp_tmp;
  kinsim_2link_planar_B.rworka[0] = 0.0;
  kinsim_2link_planar_B.rworka[2] = 0.0;
  kinsim_2link_planar_B.rworka[3] = 0.0;
  kinsim_2link_planar_B.anorm_m = fabs(A[0].re) + fabs(A[0].im);
  kinsim_2link_planar_B.rworka[1] = fabs(A[4].re) + fabs(A[4].im);
  kinsim_2link_planar_B.ascale_j = (fabs(A[5].re) + fabs(A[5].im)) +
    kinsim_2link_planar_B.rworka[1];
  if (kinsim_2link_planar_B.ascale_j > kinsim_2link_planar_B.anorm_m) {
    kinsim_2link_planar_B.anorm_m = kinsim_2link_planar_B.ascale_j;
  }

  kinsim_2link_planar_B.i_b = 0;
  while (kinsim_2link_planar_B.i_b <= 1) {
    kinsim_2link_planar_B.rworka[2] += fabs(A[kinsim_2link_planar_B.i_b + 8].re)
      + fabs(A[kinsim_2link_planar_B.i_b + 8].im);
    kinsim_2link_planar_B.i_b++;
  }

  kinsim_2link_planar_B.ascale_j = (fabs(A[10].re) + fabs(A[10].im)) +
    kinsim_2link_planar_B.rworka[2];
  if (kinsim_2link_planar_B.ascale_j > kinsim_2link_planar_B.anorm_m) {
    kinsim_2link_planar_B.anorm_m = kinsim_2link_planar_B.ascale_j;
  }

  kinsim_2link_planar_B.i_b = 0;
  while (kinsim_2link_planar_B.i_b <= 2) {
    kinsim_2link_planar_B.rworka[3] += fabs(A[kinsim_2link_planar_B.i_b + 12].re)
      + fabs(A[kinsim_2link_planar_B.i_b + 12].im);
    kinsim_2link_planar_B.i_b++;
  }

  kinsim_2link_planar_B.ascale_j = (fabs(A[15].re) + fabs(A[15].im)) +
    kinsim_2link_planar_B.rworka[3];
  if (kinsim_2link_planar_B.ascale_j > kinsim_2link_planar_B.anorm_m) {
    kinsim_2link_planar_B.anorm_m = kinsim_2link_planar_B.ascale_j;
  }

  kinsim_2link_planar_B.ascale_j = kinsim_2link_planar_B.anorm_m;
  if (2.2250738585072014E-308 > kinsim_2link_planar_B.anorm_m) {
    kinsim_2link_planar_B.ascale_j = 2.2250738585072014E-308;
  }

  kinsim_2link_planar_B.ascale_j = 1.0 / kinsim_2link_planar_B.ascale_j;
  for (kinsim_2link_planar_B.i_b = 0; kinsim_2link_planar_B.i_b < 4;
       kinsim_2link_planar_B.i_b++) {
    kinsim_2link_planar_B.c_x_tmp_tmp = (3 - kinsim_2link_planar_B.i_b) << 2;
    kinsim_2link_planar_B.c_x_tmp = (kinsim_2link_planar_B.c_x_tmp_tmp -
      kinsim_2link_planar_B.i_b) + 3;
    kinsim_2link_planar_B.salpha_re = (fabs(A[kinsim_2link_planar_B.c_x_tmp].re)
      + fabs(A[kinsim_2link_planar_B.c_x_tmp].im)) *
      kinsim_2link_planar_B.ascale_j;
    if (1.0 > kinsim_2link_planar_B.salpha_re) {
      kinsim_2link_planar_B.salpha_re = 1.0;
    }

    kinsim_2link_planar_B.temp = 1.0 / kinsim_2link_planar_B.salpha_re;
    kinsim_2link_planar_B.salpha_re = A[kinsim_2link_planar_B.c_x_tmp].re *
      kinsim_2link_planar_B.temp * kinsim_2link_planar_B.ascale_j;
    kinsim_2link_planar_B.salpha_im = A[kinsim_2link_planar_B.c_x_tmp].im *
      kinsim_2link_planar_B.temp * kinsim_2link_planar_B.ascale_j;
    kinsim_2link_planar_B.acoeff = kinsim_2link_planar_B.temp *
      kinsim_2link_planar_B.ascale_j;
    lscalea = ((kinsim_2link_planar_B.temp >= 2.2250738585072014E-308) &&
               (kinsim_2link_planar_B.acoeff < 4.0083367200179456E-292));
    kinsim_2link_planar_B.dmin = fabs(kinsim_2link_planar_B.salpha_re) + fabs
      (kinsim_2link_planar_B.salpha_im);
    lscaleb = ((kinsim_2link_planar_B.dmin >= 2.2250738585072014E-308) &&
               (kinsim_2link_planar_B.dmin < 4.0083367200179456E-292));
    kinsim_2link_planar_B.scale_h = 1.0;
    if (lscalea) {
      kinsim_2link_planar_B.scale_h = kinsim_2link_planar_B.anorm_m;
      if (2.4948003869184E+291 < kinsim_2link_planar_B.anorm_m) {
        kinsim_2link_planar_B.scale_h = 2.4948003869184E+291;
      }

      kinsim_2link_planar_B.scale_h *= 4.0083367200179456E-292 /
        kinsim_2link_planar_B.temp;
    }

    if (lscaleb) {
      kinsim_2link_planar_B.work2_idx_2_im = 4.0083367200179456E-292 /
        kinsim_2link_planar_B.dmin;
      if (kinsim_2link_planar_B.work2_idx_2_im > kinsim_2link_planar_B.scale_h)
      {
        kinsim_2link_planar_B.scale_h = kinsim_2link_planar_B.work2_idx_2_im;
      }
    }

    if (lscalea || lscaleb) {
      kinsim_2link_planar_B.work2_idx_2_im = kinsim_2link_planar_B.acoeff;
      if (1.0 > kinsim_2link_planar_B.acoeff) {
        kinsim_2link_planar_B.work2_idx_2_im = 1.0;
      }

      if (kinsim_2link_planar_B.dmin > kinsim_2link_planar_B.work2_idx_2_im) {
        kinsim_2link_planar_B.work2_idx_2_im = kinsim_2link_planar_B.dmin;
      }

      kinsim_2link_planar_B.dmin = 1.0 / (2.2250738585072014E-308 *
        kinsim_2link_planar_B.work2_idx_2_im);
      if (kinsim_2link_planar_B.dmin < kinsim_2link_planar_B.scale_h) {
        kinsim_2link_planar_B.scale_h = kinsim_2link_planar_B.dmin;
      }

      if (lscalea) {
        kinsim_2link_planar_B.acoeff = kinsim_2link_planar_B.scale_h *
          kinsim_2link_planar_B.temp * kinsim_2link_planar_B.ascale_j;
      } else {
        kinsim_2link_planar_B.acoeff *= kinsim_2link_planar_B.scale_h;
      }

      kinsim_2link_planar_B.salpha_re *= kinsim_2link_planar_B.scale_h;
      kinsim_2link_planar_B.salpha_im *= kinsim_2link_planar_B.scale_h;
    }

    memset(&kinsim_2link_planar_B.work1[0], 0, sizeof(creal_T) << 2U);
    kinsim_2link_planar_B.work1[3 - kinsim_2link_planar_B.i_b].re = 1.0;
    kinsim_2link_planar_B.work1[3 - kinsim_2link_planar_B.i_b].im = 0.0;
    kinsim_2link_planar_B.dmin = 2.2204460492503131E-16 *
      kinsim_2link_planar_B.acoeff * kinsim_2link_planar_B.anorm_m;
    kinsim_2link_planar_B.temp = (fabs(kinsim_2link_planar_B.salpha_re) + fabs
      (kinsim_2link_planar_B.salpha_im)) * 2.2204460492503131E-16;
    if (kinsim_2link_planar_B.temp > kinsim_2link_planar_B.dmin) {
      kinsim_2link_planar_B.dmin = kinsim_2link_planar_B.temp;
    }

    if (2.2250738585072014E-308 > kinsim_2link_planar_B.dmin) {
      kinsim_2link_planar_B.dmin = 2.2250738585072014E-308;
    }

    kinsim_2link_planar_B.c_x_tmp = 0;
    while (kinsim_2link_planar_B.c_x_tmp <= 2 - kinsim_2link_planar_B.i_b) {
      kinsim_2link_planar_B.d_re_tmp = kinsim_2link_planar_B.c_x_tmp_tmp +
        kinsim_2link_planar_B.c_x_tmp;
      kinsim_2link_planar_B.work1[kinsim_2link_planar_B.c_x_tmp].re =
        A[kinsim_2link_planar_B.d_re_tmp].re * kinsim_2link_planar_B.acoeff;
      kinsim_2link_planar_B.work1[kinsim_2link_planar_B.c_x_tmp].im =
        A[kinsim_2link_planar_B.d_re_tmp].im * kinsim_2link_planar_B.acoeff;
      kinsim_2link_planar_B.c_x_tmp++;
    }

    kinsim_2link_planar_B.work1[3 - kinsim_2link_planar_B.i_b].re = 1.0;
    kinsim_2link_planar_B.work1[3 - kinsim_2link_planar_B.i_b].im = 0.0;
    kinsim_2link_planar_B.c_x_tmp = static_cast<int32_T>(((-1.0 - ((-
      static_cast<real_T>(kinsim_2link_planar_B.i_b) + 4.0) - 1.0)) + 1.0) /
      -1.0);
    kinsim_2link_planar_B.c_j_a = 0;
    while (kinsim_2link_planar_B.c_j_a <= kinsim_2link_planar_B.c_x_tmp - 1) {
      work2_idx_1_re_tmp = 2 - (kinsim_2link_planar_B.i_b +
        kinsim_2link_planar_B.c_j_a);
      d_re_tmp_tmp = work2_idx_1_re_tmp << 2;
      kinsim_2link_planar_B.d_re_tmp = d_re_tmp_tmp + work2_idx_1_re_tmp;
      kinsim_2link_planar_B.work2_idx_3_re = A[kinsim_2link_planar_B.d_re_tmp].
        re * kinsim_2link_planar_B.acoeff - kinsim_2link_planar_B.salpha_re;
      kinsim_2link_planar_B.scale_h = A[kinsim_2link_planar_B.d_re_tmp].im *
        kinsim_2link_planar_B.acoeff - kinsim_2link_planar_B.salpha_im;
      if (fabs(kinsim_2link_planar_B.work2_idx_3_re) + fabs
          (kinsim_2link_planar_B.scale_h) <= kinsim_2link_planar_B.dmin) {
        kinsim_2link_planar_B.work2_idx_3_re = kinsim_2link_planar_B.dmin;
        kinsim_2link_planar_B.scale_h = 0.0;
      }

      kinsim_2link_planar_B.work2_idx_2_im = fabs
        (kinsim_2link_planar_B.work2_idx_3_re);
      kinsim_2link_planar_B.work2_idx_3_im = fabs(kinsim_2link_planar_B.scale_h);
      kinsim_2link_planar_B.temp = kinsim_2link_planar_B.work2_idx_2_im +
        kinsim_2link_planar_B.work2_idx_3_im;
      if (kinsim_2link_planar_B.temp < 1.0) {
        kinsim_2link_planar_B.f_y = fabs
          (kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].re) + fabs
          (kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].im);
        if (kinsim_2link_planar_B.f_y >= kinsim_2link_planar_B.temp *
            1.1235582092889474E+307) {
          kinsim_2link_planar_B.temp = 1.0 / kinsim_2link_planar_B.f_y;
          kinsim_2link_planar_B.d_re_tmp = 0;
          while (kinsim_2link_planar_B.d_re_tmp <= 3 - kinsim_2link_planar_B.i_b)
          {
            kinsim_2link_planar_B.work1[kinsim_2link_planar_B.d_re_tmp].re *=
              kinsim_2link_planar_B.temp;
            kinsim_2link_planar_B.work1[kinsim_2link_planar_B.d_re_tmp].im *=
              kinsim_2link_planar_B.temp;
            kinsim_2link_planar_B.d_re_tmp++;
          }
        }
      }

      if (kinsim_2link_planar_B.scale_h == 0.0) {
        if (-kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].im == 0.0) {
          kinsim_2link_planar_B.temp =
            -kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].re /
            kinsim_2link_planar_B.work2_idx_3_re;
          kinsim_2link_planar_B.scale_h = 0.0;
        } else if (-kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].re == 0.0) {
          kinsim_2link_planar_B.temp = 0.0;
          kinsim_2link_planar_B.scale_h =
            -kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].im /
            kinsim_2link_planar_B.work2_idx_3_re;
        } else {
          kinsim_2link_planar_B.temp =
            -kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].re /
            kinsim_2link_planar_B.work2_idx_3_re;
          kinsim_2link_planar_B.scale_h =
            -kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].im /
            kinsim_2link_planar_B.work2_idx_3_re;
        }
      } else if (kinsim_2link_planar_B.work2_idx_3_re == 0.0) {
        if (-kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].re == 0.0) {
          kinsim_2link_planar_B.temp =
            -kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].im /
            kinsim_2link_planar_B.scale_h;
          kinsim_2link_planar_B.scale_h = 0.0;
        } else if (-kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].im == 0.0) {
          kinsim_2link_planar_B.temp = 0.0;
          kinsim_2link_planar_B.scale_h =
            -(-kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].re /
              kinsim_2link_planar_B.scale_h);
        } else {
          kinsim_2link_planar_B.temp =
            -kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].im /
            kinsim_2link_planar_B.scale_h;
          kinsim_2link_planar_B.scale_h =
            -(-kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].re /
              kinsim_2link_planar_B.scale_h);
        }
      } else if (kinsim_2link_planar_B.work2_idx_2_im >
                 kinsim_2link_planar_B.work2_idx_3_im) {
        kinsim_2link_planar_B.work2_idx_2_im = kinsim_2link_planar_B.scale_h /
          kinsim_2link_planar_B.work2_idx_3_re;
        kinsim_2link_planar_B.scale_h = kinsim_2link_planar_B.work2_idx_2_im *
          kinsim_2link_planar_B.scale_h + kinsim_2link_planar_B.work2_idx_3_re;
        kinsim_2link_planar_B.temp = (kinsim_2link_planar_B.work2_idx_2_im *
          -kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].im +
          -kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].re) /
          kinsim_2link_planar_B.scale_h;
        kinsim_2link_planar_B.scale_h =
          (-kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].im -
           kinsim_2link_planar_B.work2_idx_2_im *
           -kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].re) /
          kinsim_2link_planar_B.scale_h;
      } else if (kinsim_2link_planar_B.work2_idx_3_im ==
                 kinsim_2link_planar_B.work2_idx_2_im) {
        kinsim_2link_planar_B.work2_idx_3_re =
          kinsim_2link_planar_B.work2_idx_3_re > 0.0 ? 0.5 : -0.5;
        kinsim_2link_planar_B.scale_h = kinsim_2link_planar_B.scale_h > 0.0 ?
          0.5 : -0.5;
        kinsim_2link_planar_B.temp =
          (-kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].re *
           kinsim_2link_planar_B.work2_idx_3_re +
           -kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].im *
           kinsim_2link_planar_B.scale_h) / kinsim_2link_planar_B.work2_idx_2_im;
        kinsim_2link_planar_B.scale_h =
          (-kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].im *
           kinsim_2link_planar_B.work2_idx_3_re -
           -kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].re *
           kinsim_2link_planar_B.scale_h) / kinsim_2link_planar_B.work2_idx_2_im;
      } else {
        kinsim_2link_planar_B.work2_idx_2_im =
          kinsim_2link_planar_B.work2_idx_3_re / kinsim_2link_planar_B.scale_h;
        kinsim_2link_planar_B.scale_h += kinsim_2link_planar_B.work2_idx_2_im *
          kinsim_2link_planar_B.work2_idx_3_re;
        kinsim_2link_planar_B.temp = (kinsim_2link_planar_B.work2_idx_2_im *
          -kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].re +
          -kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].im) /
          kinsim_2link_planar_B.scale_h;
        kinsim_2link_planar_B.scale_h = (kinsim_2link_planar_B.work2_idx_2_im *
          -kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].im -
          (-kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].re)) /
          kinsim_2link_planar_B.scale_h;
      }

      kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].re =
        kinsim_2link_planar_B.temp;
      kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].im =
        kinsim_2link_planar_B.scale_h;
      if (work2_idx_1_re_tmp + 1 > 1) {
        if (fabs(kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].re) + fabs
            (kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].im) > 1.0) {
          kinsim_2link_planar_B.temp = 1.0 / (fabs
            (kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].re) + fabs
            (kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].im));
          if (kinsim_2link_planar_B.acoeff *
              kinsim_2link_planar_B.rworka[work2_idx_1_re_tmp] >=
              1.1235582092889474E+307 * kinsim_2link_planar_B.temp) {
            kinsim_2link_planar_B.d_re_tmp = 0;
            while (kinsim_2link_planar_B.d_re_tmp <= 3 -
                   kinsim_2link_planar_B.i_b) {
              kinsim_2link_planar_B.work1[kinsim_2link_planar_B.d_re_tmp].re *=
                kinsim_2link_planar_B.temp;
              kinsim_2link_planar_B.work1[kinsim_2link_planar_B.d_re_tmp].im *=
                kinsim_2link_planar_B.temp;
              kinsim_2link_planar_B.d_re_tmp++;
            }
          }
        }

        kinsim_2link_planar_B.work2_idx_3_re = kinsim_2link_planar_B.acoeff *
          kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].re;
        kinsim_2link_planar_B.scale_h = kinsim_2link_planar_B.acoeff *
          kinsim_2link_planar_B.work1[work2_idx_1_re_tmp].im;
        kinsim_2link_planar_B.e_jr = 0;
        while (kinsim_2link_planar_B.e_jr <= work2_idx_1_re_tmp - 1) {
          kinsim_2link_planar_B.d_re_tmp = d_re_tmp_tmp +
            kinsim_2link_planar_B.e_jr;
          kinsim_2link_planar_B.work1[kinsim_2link_planar_B.e_jr].re +=
            A[kinsim_2link_planar_B.d_re_tmp].re *
            kinsim_2link_planar_B.work2_idx_3_re -
            A[kinsim_2link_planar_B.d_re_tmp].im * kinsim_2link_planar_B.scale_h;
          kinsim_2link_planar_B.work1[kinsim_2link_planar_B.e_jr].im +=
            A[kinsim_2link_planar_B.d_re_tmp].im *
            kinsim_2link_planar_B.work2_idx_3_re +
            A[kinsim_2link_planar_B.d_re_tmp].re * kinsim_2link_planar_B.scale_h;
          kinsim_2link_planar_B.e_jr++;
        }
      }

      kinsim_2link_planar_B.c_j_a++;
    }

    kinsim_2link_planar_B.salpha_re = 0.0;
    kinsim_2link_planar_B.salpha_im = 0.0;
    kinsim_2link_planar_B.acoeff = 0.0;
    kinsim_2link_planar_B.dmin = 0.0;
    kinsim_2link_planar_B.scale_h = 0.0;
    kinsim_2link_planar_B.work2_idx_2_im = 0.0;
    kinsim_2link_planar_B.work2_idx_3_re = 0.0;
    kinsim_2link_planar_B.work2_idx_3_im = 0.0;
    kinsim_2link_planar_B.c_x_tmp = 0;
    while (kinsim_2link_planar_B.c_x_tmp <= 3 - kinsim_2link_planar_B.i_b) {
      kinsim_2link_planar_B.c_j_a = kinsim_2link_planar_B.c_x_tmp << 2;
      kinsim_2link_planar_B.salpha_re += V[kinsim_2link_planar_B.c_j_a].re *
        kinsim_2link_planar_B.work1[kinsim_2link_planar_B.c_x_tmp].re -
        V[kinsim_2link_planar_B.c_j_a].im *
        kinsim_2link_planar_B.work1[kinsim_2link_planar_B.c_x_tmp].im;
      kinsim_2link_planar_B.salpha_im += V[kinsim_2link_planar_B.c_j_a].re *
        kinsim_2link_planar_B.work1[kinsim_2link_planar_B.c_x_tmp].im +
        V[kinsim_2link_planar_B.c_j_a].im *
        kinsim_2link_planar_B.work1[kinsim_2link_planar_B.c_x_tmp].re;
      work2_idx_1_re_tmp = kinsim_2link_planar_B.c_j_a + 1;
      kinsim_2link_planar_B.acoeff += V[work2_idx_1_re_tmp].re *
        kinsim_2link_planar_B.work1[kinsim_2link_planar_B.c_x_tmp].re -
        V[work2_idx_1_re_tmp].im *
        kinsim_2link_planar_B.work1[kinsim_2link_planar_B.c_x_tmp].im;
      kinsim_2link_planar_B.dmin += V[work2_idx_1_re_tmp].re *
        kinsim_2link_planar_B.work1[kinsim_2link_planar_B.c_x_tmp].im +
        V[work2_idx_1_re_tmp].im *
        kinsim_2link_planar_B.work1[kinsim_2link_planar_B.c_x_tmp].re;
      work2_idx_1_re_tmp = kinsim_2link_planar_B.c_j_a + 2;
      kinsim_2link_planar_B.scale_h += V[work2_idx_1_re_tmp].re *
        kinsim_2link_planar_B.work1[kinsim_2link_planar_B.c_x_tmp].re -
        V[work2_idx_1_re_tmp].im *
        kinsim_2link_planar_B.work1[kinsim_2link_planar_B.c_x_tmp].im;
      kinsim_2link_planar_B.work2_idx_2_im += V[work2_idx_1_re_tmp].re *
        kinsim_2link_planar_B.work1[kinsim_2link_planar_B.c_x_tmp].im +
        V[work2_idx_1_re_tmp].im *
        kinsim_2link_planar_B.work1[kinsim_2link_planar_B.c_x_tmp].re;
      kinsim_2link_planar_B.c_j_a += 3;
      kinsim_2link_planar_B.work2_idx_3_re += V[kinsim_2link_planar_B.c_j_a].re *
        kinsim_2link_planar_B.work1[kinsim_2link_planar_B.c_x_tmp].re -
        V[kinsim_2link_planar_B.c_j_a].im *
        kinsim_2link_planar_B.work1[kinsim_2link_planar_B.c_x_tmp].im;
      kinsim_2link_planar_B.work2_idx_3_im += V[kinsim_2link_planar_B.c_j_a].re *
        kinsim_2link_planar_B.work1[kinsim_2link_planar_B.c_x_tmp].im +
        V[kinsim_2link_planar_B.c_j_a].im *
        kinsim_2link_planar_B.work1[kinsim_2link_planar_B.c_x_tmp].re;
      kinsim_2link_planar_B.c_x_tmp++;
    }

    kinsim_2link_planar_B.temp = fabs(kinsim_2link_planar_B.salpha_re) + fabs
      (kinsim_2link_planar_B.salpha_im);
    kinsim_2link_planar_B.f_y = fabs(kinsim_2link_planar_B.acoeff) + fabs
      (kinsim_2link_planar_B.dmin);
    if (kinsim_2link_planar_B.f_y > kinsim_2link_planar_B.temp) {
      kinsim_2link_planar_B.temp = kinsim_2link_planar_B.f_y;
    }

    kinsim_2link_planar_B.f_y = fabs(kinsim_2link_planar_B.scale_h) + fabs
      (kinsim_2link_planar_B.work2_idx_2_im);
    if (kinsim_2link_planar_B.f_y > kinsim_2link_planar_B.temp) {
      kinsim_2link_planar_B.temp = kinsim_2link_planar_B.f_y;
    }

    kinsim_2link_planar_B.f_y = fabs(kinsim_2link_planar_B.work2_idx_3_re) +
      fabs(kinsim_2link_planar_B.work2_idx_3_im);
    if (kinsim_2link_planar_B.f_y > kinsim_2link_planar_B.temp) {
      kinsim_2link_planar_B.temp = kinsim_2link_planar_B.f_y;
    }

    if (kinsim_2link_planar_B.temp > 2.2250738585072014E-308) {
      kinsim_2link_planar_B.temp = 1.0 / kinsim_2link_planar_B.temp;
      V[kinsim_2link_planar_B.c_x_tmp_tmp].re = kinsim_2link_planar_B.temp *
        kinsim_2link_planar_B.salpha_re;
      V[kinsim_2link_planar_B.c_x_tmp_tmp].im = kinsim_2link_planar_B.temp *
        kinsim_2link_planar_B.salpha_im;
      kinsim_2link_planar_B.d_re_tmp = ((3 - kinsim_2link_planar_B.i_b) << 2) +
        1;
      V[kinsim_2link_planar_B.d_re_tmp].re = kinsim_2link_planar_B.temp *
        kinsim_2link_planar_B.acoeff;
      V[kinsim_2link_planar_B.d_re_tmp].im = kinsim_2link_planar_B.temp *
        kinsim_2link_planar_B.dmin;
      kinsim_2link_planar_B.d_re_tmp = ((3 - kinsim_2link_planar_B.i_b) << 2) +
        2;
      V[kinsim_2link_planar_B.d_re_tmp].re = kinsim_2link_planar_B.temp *
        kinsim_2link_planar_B.scale_h;
      V[kinsim_2link_planar_B.d_re_tmp].im = kinsim_2link_planar_B.temp *
        kinsim_2link_planar_B.work2_idx_2_im;
      kinsim_2link_planar_B.d_re_tmp = ((3 - kinsim_2link_planar_B.i_b) << 2) +
        3;
      V[kinsim_2link_planar_B.d_re_tmp].re = kinsim_2link_planar_B.temp *
        kinsim_2link_planar_B.work2_idx_3_re;
      V[kinsim_2link_planar_B.d_re_tmp].im = kinsim_2link_planar_B.temp *
        kinsim_2link_planar_B.work2_idx_3_im;
    } else {
      V[kinsim_2link_planar_B.c_x_tmp_tmp].re = 0.0;
      V[kinsim_2link_planar_B.c_x_tmp_tmp].im = 0.0;
      kinsim_2link_planar_B.d_re_tmp = kinsim_2link_planar_B.c_x_tmp_tmp + 1;
      V[kinsim_2link_planar_B.d_re_tmp].re = 0.0;
      V[kinsim_2link_planar_B.d_re_tmp].im = 0.0;
      kinsim_2link_planar_B.d_re_tmp = kinsim_2link_planar_B.c_x_tmp_tmp + 2;
      V[kinsim_2link_planar_B.d_re_tmp].re = 0.0;
      V[kinsim_2link_planar_B.d_re_tmp].im = 0.0;
      kinsim_2link_planar_B.d_re_tmp = kinsim_2link_planar_B.c_x_tmp_tmp + 3;
      V[kinsim_2link_planar_B.d_re_tmp].re = 0.0;
      V[kinsim_2link_planar_B.d_re_tmp].im = 0.0;
    }
  }
}

// Function for MATLAB Function: '<S5>/MATLAB Function'
static void kinsim_2link_planar_xzggev(creal_T A[16], int32_T *info, creal_T
  alpha1[4], creal_T beta1[4], creal_T V[16])
{
  boolean_T ilascl;
  boolean_T found;
  int32_T exitg1;
  int32_T exitg2;
  boolean_T exitg3;
  boolean_T exitg4;
  *info = 0;
  kinsim_2link_planar_B.anrm = kinsim_2link_planar_xzlangeM(A);
  if (rtIsInf(kinsim_2link_planar_B.anrm) || rtIsNaN(kinsim_2link_planar_B.anrm))
  {
    alpha1[0].re = (rtNaN);
    alpha1[0].im = 0.0;
    beta1[0].re = (rtNaN);
    beta1[0].im = 0.0;
    alpha1[1].re = (rtNaN);
    alpha1[1].im = 0.0;
    beta1[1].re = (rtNaN);
    beta1[1].im = 0.0;
    alpha1[2].re = (rtNaN);
    alpha1[2].im = 0.0;
    beta1[2].re = (rtNaN);
    beta1[2].im = 0.0;
    alpha1[3].re = (rtNaN);
    alpha1[3].im = 0.0;
    beta1[3].re = (rtNaN);
    beta1[3].im = 0.0;
    for (kinsim_2link_planar_B.k_l = 0; kinsim_2link_planar_B.k_l < 16;
         kinsim_2link_planar_B.k_l++) {
      V[kinsim_2link_planar_B.k_l].re = (rtNaN);
      V[kinsim_2link_planar_B.k_l].im = 0.0;
    }
  } else {
    ilascl = false;
    kinsim_2link_planar_B.anrmto = kinsim_2link_planar_B.anrm;
    if ((kinsim_2link_planar_B.anrm > 0.0) && (kinsim_2link_planar_B.anrm <
         6.7178761075670888E-139)) {
      kinsim_2link_planar_B.anrmto = 6.7178761075670888E-139;
      ilascl = true;
      kinsim_2link_planar_xzlascl(kinsim_2link_planar_B.anrm,
        kinsim_2link_planar_B.anrmto, A);
    } else {
      if (kinsim_2link_planar_B.anrm > 1.4885657073574029E+138) {
        kinsim_2link_planar_B.anrmto = 1.4885657073574029E+138;
        ilascl = true;
        kinsim_2link_planar_xzlascl(kinsim_2link_planar_B.anrm,
          kinsim_2link_planar_B.anrmto, A);
      }
    }

    kinsim_2link_planar_B.rscale[0] = 1;
    kinsim_2link_planar_B.rscale[1] = 1;
    kinsim_2link_planar_B.rscale[2] = 1;
    kinsim_2link_planar_B.rscale[3] = 1;
    kinsim_2link_planar_B.c_i = 0;
    kinsim_2link_planar_B.ihi = 4;
    do {
      exitg2 = 0;
      kinsim_2link_planar_B.i_h = 0;
      kinsim_2link_planar_B.jcol = 0;
      found = false;
      kinsim_2link_planar_B.ii = kinsim_2link_planar_B.ihi;
      exitg3 = false;
      while ((!exitg3) && (kinsim_2link_planar_B.ii > 0)) {
        kinsim_2link_planar_B.nzcount = 0;
        kinsim_2link_planar_B.i_h = kinsim_2link_planar_B.ii;
        kinsim_2link_planar_B.jcol = kinsim_2link_planar_B.ihi;
        kinsim_2link_planar_B.jj = 0;
        exitg4 = false;
        while ((!exitg4) && (kinsim_2link_planar_B.jj <=
                             kinsim_2link_planar_B.ihi - 1)) {
          kinsim_2link_planar_B.k_l = ((kinsim_2link_planar_B.jj << 2) +
            kinsim_2link_planar_B.ii) - 1;
          if ((A[kinsim_2link_planar_B.k_l].re != 0.0) ||
              (A[kinsim_2link_planar_B.k_l].im != 0.0) ||
              (kinsim_2link_planar_B.jj + 1 == kinsim_2link_planar_B.ii)) {
            if (kinsim_2link_planar_B.nzcount == 0) {
              kinsim_2link_planar_B.jcol = kinsim_2link_planar_B.jj + 1;
              kinsim_2link_planar_B.nzcount = 1;
              kinsim_2link_planar_B.jj++;
            } else {
              kinsim_2link_planar_B.nzcount = 2;
              exitg4 = true;
            }
          } else {
            kinsim_2link_planar_B.jj++;
          }
        }

        if (kinsim_2link_planar_B.nzcount < 2) {
          found = true;
          exitg3 = true;
        } else {
          kinsim_2link_planar_B.ii--;
        }
      }

      if (!found) {
        exitg2 = 2;
      } else {
        if (kinsim_2link_planar_B.i_h != kinsim_2link_planar_B.ihi) {
          kinsim_2link_planar_B.atmp_re = A[kinsim_2link_planar_B.i_h - 1].re;
          kinsim_2link_planar_B.atmp_im = A[kinsim_2link_planar_B.i_h - 1].im;
          A[kinsim_2link_planar_B.i_h - 1] = A[kinsim_2link_planar_B.ihi - 1];
          A[kinsim_2link_planar_B.ihi - 1].re = kinsim_2link_planar_B.atmp_re;
          A[kinsim_2link_planar_B.ihi - 1].im = kinsim_2link_planar_B.atmp_im;
          kinsim_2link_planar_B.atmp_re = A[kinsim_2link_planar_B.i_h + 3].re;
          kinsim_2link_planar_B.atmp_im = A[kinsim_2link_planar_B.i_h + 3].im;
          A[kinsim_2link_planar_B.i_h + 3] = A[kinsim_2link_planar_B.ihi + 3];
          A[kinsim_2link_planar_B.ihi + 3].re = kinsim_2link_planar_B.atmp_re;
          A[kinsim_2link_planar_B.ihi + 3].im = kinsim_2link_planar_B.atmp_im;
          kinsim_2link_planar_B.atmp_re = A[kinsim_2link_planar_B.i_h + 7].re;
          kinsim_2link_planar_B.atmp_im = A[kinsim_2link_planar_B.i_h + 7].im;
          A[kinsim_2link_planar_B.i_h + 7] = A[kinsim_2link_planar_B.ihi + 7];
          A[kinsim_2link_planar_B.ihi + 7].re = kinsim_2link_planar_B.atmp_re;
          A[kinsim_2link_planar_B.ihi + 7].im = kinsim_2link_planar_B.atmp_im;
          kinsim_2link_planar_B.atmp_re = A[kinsim_2link_planar_B.i_h + 11].re;
          kinsim_2link_planar_B.atmp_im = A[kinsim_2link_planar_B.i_h + 11].im;
          A[kinsim_2link_planar_B.i_h + 11] = A[kinsim_2link_planar_B.ihi + 11];
          A[kinsim_2link_planar_B.ihi + 11].re = kinsim_2link_planar_B.atmp_re;
          A[kinsim_2link_planar_B.ihi + 11].im = kinsim_2link_planar_B.atmp_im;
        }

        if (kinsim_2link_planar_B.jcol != kinsim_2link_planar_B.ihi) {
          kinsim_2link_planar_B.ii = 0;
          while (kinsim_2link_planar_B.ii <= kinsim_2link_planar_B.ihi - 1) {
            kinsim_2link_planar_B.i_h = ((kinsim_2link_planar_B.jcol - 1) << 2)
              + kinsim_2link_planar_B.ii;
            kinsim_2link_planar_B.atmp_re = A[kinsim_2link_planar_B.i_h].re;
            kinsim_2link_planar_B.atmp_im = A[kinsim_2link_planar_B.i_h].im;
            kinsim_2link_planar_B.k_l = ((kinsim_2link_planar_B.ihi - 1) << 2) +
              kinsim_2link_planar_B.ii;
            A[kinsim_2link_planar_B.i_h] = A[kinsim_2link_planar_B.k_l];
            A[kinsim_2link_planar_B.k_l].re = kinsim_2link_planar_B.atmp_re;
            A[kinsim_2link_planar_B.k_l].im = kinsim_2link_planar_B.atmp_im;
            kinsim_2link_planar_B.ii++;
          }
        }

        kinsim_2link_planar_B.rscale[kinsim_2link_planar_B.ihi - 1] =
          kinsim_2link_planar_B.jcol;
        kinsim_2link_planar_B.ihi--;
        if (kinsim_2link_planar_B.ihi == 1) {
          kinsim_2link_planar_B.rscale[0] = 1;
          exitg2 = 1;
        }
      }
    } while (exitg2 == 0);

    if (exitg2 == 1) {
    } else {
      do {
        exitg1 = 0;
        kinsim_2link_planar_B.ii = 0;
        kinsim_2link_planar_B.jcol = 0;
        found = false;
        kinsim_2link_planar_B.i_h = kinsim_2link_planar_B.c_i + 1;
        exitg3 = false;
        while ((!exitg3) && (kinsim_2link_planar_B.i_h <=
                             kinsim_2link_planar_B.ihi)) {
          kinsim_2link_planar_B.nzcount = 0;
          kinsim_2link_planar_B.ii = kinsim_2link_planar_B.ihi;
          kinsim_2link_planar_B.jcol = kinsim_2link_planar_B.i_h;
          kinsim_2link_planar_B.jj = kinsim_2link_planar_B.c_i + 1;
          exitg4 = false;
          while ((!exitg4) && (kinsim_2link_planar_B.jj <=
                               kinsim_2link_planar_B.ihi)) {
            kinsim_2link_planar_B.k_l = (((kinsim_2link_planar_B.i_h - 1) << 2)
              + kinsim_2link_planar_B.jj) - 1;
            if ((A[kinsim_2link_planar_B.k_l].re != 0.0) ||
                (A[kinsim_2link_planar_B.k_l].im != 0.0) ||
                (kinsim_2link_planar_B.jj == kinsim_2link_planar_B.i_h)) {
              if (kinsim_2link_planar_B.nzcount == 0) {
                kinsim_2link_planar_B.ii = kinsim_2link_planar_B.jj;
                kinsim_2link_planar_B.nzcount = 1;
                kinsim_2link_planar_B.jj++;
              } else {
                kinsim_2link_planar_B.nzcount = 2;
                exitg4 = true;
              }
            } else {
              kinsim_2link_planar_B.jj++;
            }
          }

          if (kinsim_2link_planar_B.nzcount < 2) {
            found = true;
            exitg3 = true;
          } else {
            kinsim_2link_planar_B.i_h++;
          }
        }

        if (!found) {
          exitg1 = 1;
        } else {
          if (kinsim_2link_planar_B.c_i + 1 != kinsim_2link_planar_B.ii) {
            kinsim_2link_planar_B.nzcount = kinsim_2link_planar_B.c_i;
            while (kinsim_2link_planar_B.nzcount + 1 < 5) {
              kinsim_2link_planar_B.k_l = kinsim_2link_planar_B.nzcount << 2;
              kinsim_2link_planar_B.i_h = (kinsim_2link_planar_B.k_l +
                kinsim_2link_planar_B.ii) - 1;
              kinsim_2link_planar_B.atmp_re = A[kinsim_2link_planar_B.i_h].re;
              kinsim_2link_planar_B.atmp_im = A[kinsim_2link_planar_B.i_h].im;
              kinsim_2link_planar_B.k_l += kinsim_2link_planar_B.c_i;
              A[kinsim_2link_planar_B.i_h] = A[kinsim_2link_planar_B.k_l];
              A[kinsim_2link_planar_B.k_l].re = kinsim_2link_planar_B.atmp_re;
              A[kinsim_2link_planar_B.k_l].im = kinsim_2link_planar_B.atmp_im;
              kinsim_2link_planar_B.nzcount++;
            }
          }

          if (kinsim_2link_planar_B.c_i + 1 != kinsim_2link_planar_B.jcol) {
            kinsim_2link_planar_B.ii = 0;
            while (kinsim_2link_planar_B.ii <= kinsim_2link_planar_B.ihi - 1) {
              kinsim_2link_planar_B.i_h = ((kinsim_2link_planar_B.jcol - 1) << 2)
                + kinsim_2link_planar_B.ii;
              kinsim_2link_planar_B.atmp_re = A[kinsim_2link_planar_B.i_h].re;
              kinsim_2link_planar_B.atmp_im = A[kinsim_2link_planar_B.i_h].im;
              kinsim_2link_planar_B.k_l = (kinsim_2link_planar_B.c_i << 2) +
                kinsim_2link_planar_B.ii;
              A[kinsim_2link_planar_B.i_h] = A[kinsim_2link_planar_B.k_l];
              A[kinsim_2link_planar_B.k_l].re = kinsim_2link_planar_B.atmp_re;
              A[kinsim_2link_planar_B.k_l].im = kinsim_2link_planar_B.atmp_im;
              kinsim_2link_planar_B.ii++;
            }
          }

          kinsim_2link_planar_B.rscale[kinsim_2link_planar_B.c_i] =
            kinsim_2link_planar_B.jcol;
          kinsim_2link_planar_B.c_i++;
          if (kinsim_2link_planar_B.c_i + 1 == kinsim_2link_planar_B.ihi) {
            kinsim_2link_planar_B.rscale[kinsim_2link_planar_B.c_i] =
              kinsim_2link_planar_B.c_i + 1;
            exitg1 = 1;
          }
        }
      } while (exitg1 == 0);
    }

    for (kinsim_2link_planar_B.k_l = 0; kinsim_2link_planar_B.k_l < 16;
         kinsim_2link_planar_B.k_l++) {
      kinsim_2link_planar_B.b_I[kinsim_2link_planar_B.k_l] = 0;
    }

    kinsim_2link_planar_B.b_I[0] = 1;
    kinsim_2link_planar_B.b_I[5] = 1;
    kinsim_2link_planar_B.b_I[10] = 1;
    kinsim_2link_planar_B.b_I[15] = 1;
    for (kinsim_2link_planar_B.k_l = 0; kinsim_2link_planar_B.k_l < 16;
         kinsim_2link_planar_B.k_l++) {
      V[kinsim_2link_planar_B.k_l].re =
        kinsim_2link_planar_B.b_I[kinsim_2link_planar_B.k_l];
      V[kinsim_2link_planar_B.k_l].im = 0.0;
    }

    if (kinsim_2link_planar_B.ihi >= kinsim_2link_planar_B.c_i + 3) {
      kinsim_2link_planar_B.jcol = kinsim_2link_planar_B.c_i;
      while (kinsim_2link_planar_B.jcol + 1 < kinsim_2link_planar_B.ihi - 1) {
        kinsim_2link_planar_B.ii = kinsim_2link_planar_B.ihi - 1;
        while (kinsim_2link_planar_B.ii + 1 > kinsim_2link_planar_B.jcol + 2) {
          kinsim_2link_planar_xzlartg(A[(kinsim_2link_planar_B.ii +
            (kinsim_2link_planar_B.jcol << 2)) - 1], A[kinsim_2link_planar_B.ii
            + (kinsim_2link_planar_B.jcol << 2)], &kinsim_2link_planar_B.mul,
            &kinsim_2link_planar_B.s, &A[(kinsim_2link_planar_B.ii +
            (kinsim_2link_planar_B.jcol << 2)) - 1]);
          kinsim_2link_planar_B.k_l = kinsim_2link_planar_B.ii +
            (kinsim_2link_planar_B.jcol << 2);
          A[kinsim_2link_planar_B.k_l].re = 0.0;
          A[kinsim_2link_planar_B.k_l].im = 0.0;
          kinsim_2link_planar_B.nzcount = kinsim_2link_planar_B.jcol + 1;
          while (kinsim_2link_planar_B.nzcount + 1 < 5) {
            kinsim_2link_planar_B.i_h = (kinsim_2link_planar_B.nzcount << 2) +
              kinsim_2link_planar_B.ii;
            kinsim_2link_planar_B.k_l = kinsim_2link_planar_B.i_h - 1;
            kinsim_2link_planar_B.atmp_re = A[kinsim_2link_planar_B.k_l].re *
              kinsim_2link_planar_B.mul + (A[kinsim_2link_planar_B.i_h].re *
              kinsim_2link_planar_B.s.re - A[kinsim_2link_planar_B.i_h].im *
              kinsim_2link_planar_B.s.im);
            kinsim_2link_planar_B.atmp_im = A[kinsim_2link_planar_B.k_l].im *
              kinsim_2link_planar_B.mul + (A[kinsim_2link_planar_B.i_h].im *
              kinsim_2link_planar_B.s.re + A[kinsim_2link_planar_B.i_h].re *
              kinsim_2link_planar_B.s.im);
            kinsim_2link_planar_B.d = A[kinsim_2link_planar_B.k_l].im;
            kinsim_2link_planar_B.d1 = A[kinsim_2link_planar_B.k_l].re;
            A[kinsim_2link_planar_B.i_h].re = A[kinsim_2link_planar_B.i_h].re *
              kinsim_2link_planar_B.mul - (A[kinsim_2link_planar_B.k_l].re *
              kinsim_2link_planar_B.s.re + A[kinsim_2link_planar_B.k_l].im *
              kinsim_2link_planar_B.s.im);
            A[kinsim_2link_planar_B.i_h].im = A[kinsim_2link_planar_B.i_h].im *
              kinsim_2link_planar_B.mul - (kinsim_2link_planar_B.s.re *
              kinsim_2link_planar_B.d - kinsim_2link_planar_B.s.im *
              kinsim_2link_planar_B.d1);
            A[kinsim_2link_planar_B.k_l].re = kinsim_2link_planar_B.atmp_re;
            A[kinsim_2link_planar_B.k_l].im = kinsim_2link_planar_B.atmp_im;
            kinsim_2link_planar_B.nzcount++;
          }

          kinsim_2link_planar_B.s.re = -kinsim_2link_planar_B.s.re;
          kinsim_2link_planar_B.s.im = -kinsim_2link_planar_B.s.im;
          kinsim_2link_planar_B.nzcount = 0;
          while (kinsim_2link_planar_B.nzcount + 1 <= kinsim_2link_planar_B.ihi)
          {
            kinsim_2link_planar_B.i_h = ((kinsim_2link_planar_B.ii - 1) << 2) +
              kinsim_2link_planar_B.nzcount;
            kinsim_2link_planar_B.k_l = (kinsim_2link_planar_B.ii << 2) +
              kinsim_2link_planar_B.nzcount;
            kinsim_2link_planar_B.atmp_re = (A[kinsim_2link_planar_B.i_h].re *
              kinsim_2link_planar_B.s.re - A[kinsim_2link_planar_B.i_h].im *
              kinsim_2link_planar_B.s.im) + A[kinsim_2link_planar_B.k_l].re *
              kinsim_2link_planar_B.mul;
            kinsim_2link_planar_B.atmp_im = (A[kinsim_2link_planar_B.i_h].im *
              kinsim_2link_planar_B.s.re + A[kinsim_2link_planar_B.i_h].re *
              kinsim_2link_planar_B.s.im) + A[kinsim_2link_planar_B.k_l].im *
              kinsim_2link_planar_B.mul;
            kinsim_2link_planar_B.d = A[kinsim_2link_planar_B.k_l].im;
            kinsim_2link_planar_B.d1 = A[kinsim_2link_planar_B.k_l].re;
            A[kinsim_2link_planar_B.i_h].re = A[kinsim_2link_planar_B.i_h].re *
              kinsim_2link_planar_B.mul - (A[kinsim_2link_planar_B.k_l].re *
              kinsim_2link_planar_B.s.re + A[kinsim_2link_planar_B.k_l].im *
              kinsim_2link_planar_B.s.im);
            A[kinsim_2link_planar_B.i_h].im = A[kinsim_2link_planar_B.i_h].im *
              kinsim_2link_planar_B.mul - (kinsim_2link_planar_B.s.re *
              kinsim_2link_planar_B.d - kinsim_2link_planar_B.s.im *
              kinsim_2link_planar_B.d1);
            A[kinsim_2link_planar_B.k_l].re = kinsim_2link_planar_B.atmp_re;
            A[kinsim_2link_planar_B.k_l].im = kinsim_2link_planar_B.atmp_im;
            kinsim_2link_planar_B.nzcount++;
          }

          kinsim_2link_planar_B.i_h = (kinsim_2link_planar_B.ii - 1) << 2;
          kinsim_2link_planar_B.k_l = kinsim_2link_planar_B.ii << 2;
          kinsim_2link_planar_B.atmp_re = (V[kinsim_2link_planar_B.i_h].re *
            kinsim_2link_planar_B.s.re - V[kinsim_2link_planar_B.i_h].im *
            kinsim_2link_planar_B.s.im) + V[kinsim_2link_planar_B.k_l].re *
            kinsim_2link_planar_B.mul;
          kinsim_2link_planar_B.atmp_im = (V[kinsim_2link_planar_B.i_h].im *
            kinsim_2link_planar_B.s.re + V[kinsim_2link_planar_B.i_h].re *
            kinsim_2link_planar_B.s.im) + V[kinsim_2link_planar_B.k_l].im *
            kinsim_2link_planar_B.mul;
          kinsim_2link_planar_B.d = V[kinsim_2link_planar_B.k_l].re;
          V[kinsim_2link_planar_B.i_h].re = V[kinsim_2link_planar_B.i_h].re *
            kinsim_2link_planar_B.mul - (V[kinsim_2link_planar_B.k_l].re *
            kinsim_2link_planar_B.s.re + V[kinsim_2link_planar_B.k_l].im *
            kinsim_2link_planar_B.s.im);
          V[kinsim_2link_planar_B.i_h].im = V[kinsim_2link_planar_B.i_h].im *
            kinsim_2link_planar_B.mul - (V[kinsim_2link_planar_B.k_l].im *
            kinsim_2link_planar_B.s.re - kinsim_2link_planar_B.s.im *
            kinsim_2link_planar_B.d);
          V[kinsim_2link_planar_B.k_l].re = kinsim_2link_planar_B.atmp_re;
          V[kinsim_2link_planar_B.k_l].im = kinsim_2link_planar_B.atmp_im;
          kinsim_2link_planar_B.nzcount = kinsim_2link_planar_B.i_h + 1;
          kinsim_2link_planar_B.jj = kinsim_2link_planar_B.k_l + 1;
          kinsim_2link_planar_B.atmp_re = (V[kinsim_2link_planar_B.nzcount].re *
            kinsim_2link_planar_B.s.re - V[kinsim_2link_planar_B.nzcount].im *
            kinsim_2link_planar_B.s.im) + V[kinsim_2link_planar_B.jj].re *
            kinsim_2link_planar_B.mul;
          kinsim_2link_planar_B.atmp_im = (V[kinsim_2link_planar_B.nzcount].im *
            kinsim_2link_planar_B.s.re + V[kinsim_2link_planar_B.nzcount].re *
            kinsim_2link_planar_B.s.im) + V[kinsim_2link_planar_B.jj].im *
            kinsim_2link_planar_B.mul;
          kinsim_2link_planar_B.d = V[kinsim_2link_planar_B.jj].re;
          V[kinsim_2link_planar_B.nzcount].re = V[kinsim_2link_planar_B.nzcount]
            .re * kinsim_2link_planar_B.mul - (V[kinsim_2link_planar_B.jj].re *
            kinsim_2link_planar_B.s.re + V[kinsim_2link_planar_B.jj].im *
            kinsim_2link_planar_B.s.im);
          V[kinsim_2link_planar_B.nzcount].im = V[kinsim_2link_planar_B.nzcount]
            .im * kinsim_2link_planar_B.mul - (V[kinsim_2link_planar_B.jj].im *
            kinsim_2link_planar_B.s.re - kinsim_2link_planar_B.s.im *
            kinsim_2link_planar_B.d);
          V[kinsim_2link_planar_B.jj].re = kinsim_2link_planar_B.atmp_re;
          V[kinsim_2link_planar_B.jj].im = kinsim_2link_planar_B.atmp_im;
          kinsim_2link_planar_B.nzcount = kinsim_2link_planar_B.i_h + 2;
          kinsim_2link_planar_B.jj = kinsim_2link_planar_B.k_l + 2;
          kinsim_2link_planar_B.atmp_re = (V[kinsim_2link_planar_B.nzcount].re *
            kinsim_2link_planar_B.s.re - V[kinsim_2link_planar_B.nzcount].im *
            kinsim_2link_planar_B.s.im) + V[kinsim_2link_planar_B.jj].re *
            kinsim_2link_planar_B.mul;
          kinsim_2link_planar_B.atmp_im = (V[kinsim_2link_planar_B.nzcount].im *
            kinsim_2link_planar_B.s.re + V[kinsim_2link_planar_B.nzcount].re *
            kinsim_2link_planar_B.s.im) + V[kinsim_2link_planar_B.jj].im *
            kinsim_2link_planar_B.mul;
          kinsim_2link_planar_B.d = V[kinsim_2link_planar_B.jj].re;
          V[kinsim_2link_planar_B.nzcount].re = V[kinsim_2link_planar_B.nzcount]
            .re * kinsim_2link_planar_B.mul - (V[kinsim_2link_planar_B.jj].re *
            kinsim_2link_planar_B.s.re + V[kinsim_2link_planar_B.jj].im *
            kinsim_2link_planar_B.s.im);
          V[kinsim_2link_planar_B.nzcount].im = V[kinsim_2link_planar_B.nzcount]
            .im * kinsim_2link_planar_B.mul - (V[kinsim_2link_planar_B.jj].im *
            kinsim_2link_planar_B.s.re - kinsim_2link_planar_B.s.im *
            kinsim_2link_planar_B.d);
          V[kinsim_2link_planar_B.jj].re = kinsim_2link_planar_B.atmp_re;
          V[kinsim_2link_planar_B.jj].im = kinsim_2link_planar_B.atmp_im;
          kinsim_2link_planar_B.i_h += 3;
          kinsim_2link_planar_B.k_l += 3;
          kinsim_2link_planar_B.atmp_re = (V[kinsim_2link_planar_B.i_h].re *
            kinsim_2link_planar_B.s.re - V[kinsim_2link_planar_B.i_h].im *
            kinsim_2link_planar_B.s.im) + V[kinsim_2link_planar_B.k_l].re *
            kinsim_2link_planar_B.mul;
          kinsim_2link_planar_B.atmp_im = (V[kinsim_2link_planar_B.i_h].im *
            kinsim_2link_planar_B.s.re + V[kinsim_2link_planar_B.i_h].re *
            kinsim_2link_planar_B.s.im) + V[kinsim_2link_planar_B.k_l].im *
            kinsim_2link_planar_B.mul;
          kinsim_2link_planar_B.d = V[kinsim_2link_planar_B.k_l].re;
          V[kinsim_2link_planar_B.i_h].re = V[kinsim_2link_planar_B.i_h].re *
            kinsim_2link_planar_B.mul - (V[kinsim_2link_planar_B.k_l].re *
            kinsim_2link_planar_B.s.re + V[kinsim_2link_planar_B.k_l].im *
            kinsim_2link_planar_B.s.im);
          V[kinsim_2link_planar_B.i_h].im = V[kinsim_2link_planar_B.i_h].im *
            kinsim_2link_planar_B.mul - (V[kinsim_2link_planar_B.k_l].im *
            kinsim_2link_planar_B.s.re - kinsim_2link_planar_B.s.im *
            kinsim_2link_planar_B.d);
          V[kinsim_2link_planar_B.k_l].re = kinsim_2link_planar_B.atmp_re;
          V[kinsim_2link_planar_B.k_l].im = kinsim_2link_planar_B.atmp_im;
          kinsim_2link_planar_B.ii--;
        }

        kinsim_2link_planar_B.jcol++;
      }
    }

    kinsim_2link_planar_xzhgeqz(A, kinsim_2link_planar_B.c_i + 1,
      kinsim_2link_planar_B.ihi, V, info, alpha1, beta1);
    if (*info == 0) {
      kinsim_2link_planar_xztgevc(A, V);
      if (kinsim_2link_planar_B.c_i + 1 > 1) {
        kinsim_2link_planar_B.c_i--;
        while (kinsim_2link_planar_B.c_i + 1 >= 1) {
          kinsim_2link_planar_B.k_l =
            kinsim_2link_planar_B.rscale[kinsim_2link_planar_B.c_i] - 1;
          if (kinsim_2link_planar_B.c_i + 1 !=
              kinsim_2link_planar_B.rscale[kinsim_2link_planar_B.c_i]) {
            kinsim_2link_planar_B.atmp_re = V[kinsim_2link_planar_B.c_i].re;
            kinsim_2link_planar_B.atmp_im = V[kinsim_2link_planar_B.c_i].im;
            V[kinsim_2link_planar_B.c_i] = V[kinsim_2link_planar_B.k_l];
            V[kinsim_2link_planar_B.k_l].re = kinsim_2link_planar_B.atmp_re;
            V[kinsim_2link_planar_B.k_l].im = kinsim_2link_planar_B.atmp_im;
            kinsim_2link_planar_B.atmp_re = V[kinsim_2link_planar_B.c_i + 4].re;
            kinsim_2link_planar_B.atmp_im = V[kinsim_2link_planar_B.c_i + 4].im;
            V[kinsim_2link_planar_B.c_i + 4] = V[kinsim_2link_planar_B.k_l + 4];
            V[kinsim_2link_planar_B.k_l + 4].re = kinsim_2link_planar_B.atmp_re;
            V[kinsim_2link_planar_B.k_l + 4].im = kinsim_2link_planar_B.atmp_im;
            kinsim_2link_planar_B.atmp_re = V[kinsim_2link_planar_B.c_i + 8].re;
            kinsim_2link_planar_B.atmp_im = V[kinsim_2link_planar_B.c_i + 8].im;
            V[kinsim_2link_planar_B.c_i + 8] = V[kinsim_2link_planar_B.k_l + 8];
            V[kinsim_2link_planar_B.k_l + 8].re = kinsim_2link_planar_B.atmp_re;
            V[kinsim_2link_planar_B.k_l + 8].im = kinsim_2link_planar_B.atmp_im;
            kinsim_2link_planar_B.atmp_re = V[kinsim_2link_planar_B.c_i + 12].re;
            kinsim_2link_planar_B.atmp_im = V[kinsim_2link_planar_B.c_i + 12].im;
            V[kinsim_2link_planar_B.c_i + 12] = V[kinsim_2link_planar_B.k_l + 12];
            V[kinsim_2link_planar_B.k_l + 12].re = kinsim_2link_planar_B.atmp_re;
            V[kinsim_2link_planar_B.k_l + 12].im = kinsim_2link_planar_B.atmp_im;
          }

          kinsim_2link_planar_B.c_i--;
        }
      }

      if (kinsim_2link_planar_B.ihi < 4) {
        while (kinsim_2link_planar_B.ihi + 1 < 5) {
          kinsim_2link_planar_B.k_l =
            kinsim_2link_planar_B.rscale[kinsim_2link_planar_B.ihi] - 1;
          if (kinsim_2link_planar_B.ihi + 1 !=
              kinsim_2link_planar_B.rscale[kinsim_2link_planar_B.ihi]) {
            kinsim_2link_planar_B.atmp_re = V[kinsim_2link_planar_B.ihi].re;
            kinsim_2link_planar_B.atmp_im = V[kinsim_2link_planar_B.ihi].im;
            V[kinsim_2link_planar_B.ihi] = V[kinsim_2link_planar_B.k_l];
            V[kinsim_2link_planar_B.k_l].re = kinsim_2link_planar_B.atmp_re;
            V[kinsim_2link_planar_B.k_l].im = kinsim_2link_planar_B.atmp_im;
            kinsim_2link_planar_B.atmp_re = V[kinsim_2link_planar_B.ihi + 4].re;
            kinsim_2link_planar_B.atmp_im = V[kinsim_2link_planar_B.ihi + 4].im;
            V[kinsim_2link_planar_B.ihi + 4] = V[kinsim_2link_planar_B.k_l + 4];
            V[kinsim_2link_planar_B.k_l + 4].re = kinsim_2link_planar_B.atmp_re;
            V[kinsim_2link_planar_B.k_l + 4].im = kinsim_2link_planar_B.atmp_im;
            kinsim_2link_planar_B.atmp_re = V[kinsim_2link_planar_B.ihi + 8].re;
            kinsim_2link_planar_B.atmp_im = V[kinsim_2link_planar_B.ihi + 8].im;
            V[kinsim_2link_planar_B.ihi + 8] = V[kinsim_2link_planar_B.k_l + 8];
            V[kinsim_2link_planar_B.k_l + 8].re = kinsim_2link_planar_B.atmp_re;
            V[kinsim_2link_planar_B.k_l + 8].im = kinsim_2link_planar_B.atmp_im;
            kinsim_2link_planar_B.atmp_re = V[kinsim_2link_planar_B.ihi + 12].re;
            kinsim_2link_planar_B.atmp_im = V[kinsim_2link_planar_B.ihi + 12].im;
            V[kinsim_2link_planar_B.ihi + 12] = V[kinsim_2link_planar_B.k_l + 12];
            V[kinsim_2link_planar_B.k_l + 12].re = kinsim_2link_planar_B.atmp_re;
            V[kinsim_2link_planar_B.k_l + 12].im = kinsim_2link_planar_B.atmp_im;
          }

          kinsim_2link_planar_B.ihi++;
        }
      }

      for (kinsim_2link_planar_B.ihi = 0; kinsim_2link_planar_B.ihi < 4;
           kinsim_2link_planar_B.ihi++) {
        kinsim_2link_planar_B.c_i = kinsim_2link_planar_B.ihi << 2;
        kinsim_2link_planar_B.atmp_re = fabs(V[kinsim_2link_planar_B.c_i].re) +
          fabs(V[kinsim_2link_planar_B.c_i].im);
        kinsim_2link_planar_B.k_l = kinsim_2link_planar_B.c_i + 1;
        kinsim_2link_planar_B.atmp_im = fabs(V[kinsim_2link_planar_B.k_l].re) +
          fabs(V[kinsim_2link_planar_B.k_l].im);
        if (kinsim_2link_planar_B.atmp_im > kinsim_2link_planar_B.atmp_re) {
          kinsim_2link_planar_B.atmp_re = kinsim_2link_planar_B.atmp_im;
        }

        kinsim_2link_planar_B.i_h = kinsim_2link_planar_B.c_i + 2;
        kinsim_2link_planar_B.atmp_im = fabs(V[kinsim_2link_planar_B.i_h].re) +
          fabs(V[kinsim_2link_planar_B.i_h].im);
        if (kinsim_2link_planar_B.atmp_im > kinsim_2link_planar_B.atmp_re) {
          kinsim_2link_planar_B.atmp_re = kinsim_2link_planar_B.atmp_im;
        }

        kinsim_2link_planar_B.jcol = kinsim_2link_planar_B.c_i + 3;
        kinsim_2link_planar_B.atmp_im = fabs(V[kinsim_2link_planar_B.jcol].re) +
          fabs(V[kinsim_2link_planar_B.jcol].im);
        if (kinsim_2link_planar_B.atmp_im > kinsim_2link_planar_B.atmp_re) {
          kinsim_2link_planar_B.atmp_re = kinsim_2link_planar_B.atmp_im;
        }

        if (kinsim_2link_planar_B.atmp_re >= 6.7178761075670888E-139) {
          kinsim_2link_planar_B.atmp_re = 1.0 / kinsim_2link_planar_B.atmp_re;
          V[kinsim_2link_planar_B.c_i].re *= kinsim_2link_planar_B.atmp_re;
          V[kinsim_2link_planar_B.c_i].im *= kinsim_2link_planar_B.atmp_re;
          V[kinsim_2link_planar_B.k_l].re *= kinsim_2link_planar_B.atmp_re;
          V[kinsim_2link_planar_B.k_l].im *= kinsim_2link_planar_B.atmp_re;
          V[kinsim_2link_planar_B.i_h].re *= kinsim_2link_planar_B.atmp_re;
          V[kinsim_2link_planar_B.i_h].im *= kinsim_2link_planar_B.atmp_re;
          V[kinsim_2link_planar_B.jcol].re *= kinsim_2link_planar_B.atmp_re;
          V[kinsim_2link_planar_B.jcol].im *= kinsim_2link_planar_B.atmp_re;
        }
      }

      if (ilascl) {
        ilascl = true;
        while (ilascl) {
          kinsim_2link_planar_B.atmp_re = kinsim_2link_planar_B.anrmto *
            2.0041683600089728E-292;
          kinsim_2link_planar_B.atmp_im = kinsim_2link_planar_B.anrm /
            4.9896007738368E+291;
          if ((fabs(kinsim_2link_planar_B.atmp_re) > fabs
               (kinsim_2link_planar_B.anrm)) && (kinsim_2link_planar_B.anrm !=
               0.0)) {
            kinsim_2link_planar_B.mul = 2.0041683600089728E-292;
            kinsim_2link_planar_B.anrmto = kinsim_2link_planar_B.atmp_re;
          } else if (fabs(kinsim_2link_planar_B.atmp_im) > fabs
                     (kinsim_2link_planar_B.anrmto)) {
            kinsim_2link_planar_B.mul = 4.9896007738368E+291;
            kinsim_2link_planar_B.anrm = kinsim_2link_planar_B.atmp_im;
          } else {
            kinsim_2link_planar_B.mul = kinsim_2link_planar_B.anrm /
              kinsim_2link_planar_B.anrmto;
            ilascl = false;
          }

          alpha1[0].re *= kinsim_2link_planar_B.mul;
          alpha1[0].im *= kinsim_2link_planar_B.mul;
          alpha1[1].re *= kinsim_2link_planar_B.mul;
          alpha1[1].im *= kinsim_2link_planar_B.mul;
          alpha1[2].re *= kinsim_2link_planar_B.mul;
          alpha1[2].im *= kinsim_2link_planar_B.mul;
          alpha1[3].re *= kinsim_2link_planar_B.mul;
          alpha1[3].im *= kinsim_2link_planar_B.mul;
        }
      }
    }
  }
}

// Function for MATLAB Function: '<S5>/MATLAB Function'
static real_T kinsim_2link_planar_xnrm2(int32_T n, const real_T x[16], int32_T
  ix0)
{
  real_T y;
  real_T scale;
  real_T absxk;
  real_T t;
  int32_T k;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      scale = 3.3121686421112381E-170;
      for (k = ix0; k <= ix0 + 1; k++) {
        absxk = fabs(x[k - 1]);
        if (absxk > scale) {
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

// Function for MATLAB Function: '<S5>/MATLAB Function'
static void kinsim_2link_planar_xzlarf(int32_T m, int32_T n, int32_T iv0, real_T
  tau, real_T C[16], int32_T ic0, real_T work[4])
{
  int32_T d;
  int32_T b_ia;
  int32_T jy;
  int32_T exitg1;
  boolean_T exitg2;
  if (tau != 0.0) {
    kinsim_2link_planar_B.lastv = m;
    kinsim_2link_planar_B.lastc_e = iv0 + m;
    while ((kinsim_2link_planar_B.lastv > 0) && (C[kinsim_2link_planar_B.lastc_e
            - 2] == 0.0)) {
      kinsim_2link_planar_B.lastv--;
      kinsim_2link_planar_B.lastc_e--;
    }

    kinsim_2link_planar_B.lastc_e = n - 1;
    exitg2 = false;
    while ((!exitg2) && (kinsim_2link_planar_B.lastc_e + 1 > 0)) {
      kinsim_2link_planar_B.coltop = (kinsim_2link_planar_B.lastc_e << 2) + ic0;
      jy = kinsim_2link_planar_B.coltop;
      do {
        exitg1 = 0;
        if (jy <= (kinsim_2link_planar_B.coltop + kinsim_2link_planar_B.lastv) -
            1) {
          if (C[jy - 1] != 0.0) {
            exitg1 = 1;
          } else {
            jy++;
          }
        } else {
          kinsim_2link_planar_B.lastc_e--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    kinsim_2link_planar_B.lastv = 0;
    kinsim_2link_planar_B.lastc_e = -1;
  }

  if (kinsim_2link_planar_B.lastv > 0) {
    if (kinsim_2link_planar_B.lastc_e + 1 != 0) {
      kinsim_2link_planar_B.coltop = 0;
      while (kinsim_2link_planar_B.coltop <= kinsim_2link_planar_B.lastc_e) {
        work[kinsim_2link_planar_B.coltop] = 0.0;
        kinsim_2link_planar_B.coltop++;
      }

      kinsim_2link_planar_B.coltop = 0;
      jy = (kinsim_2link_planar_B.lastc_e << 2) + ic0;
      kinsim_2link_planar_B.iac_h = ic0;
      while (kinsim_2link_planar_B.iac_h <= jy) {
        kinsim_2link_planar_B.ix_f = iv0;
        kinsim_2link_planar_B.c = 0.0;
        d = (kinsim_2link_planar_B.iac_h + kinsim_2link_planar_B.lastv) - 1;
        for (b_ia = kinsim_2link_planar_B.iac_h; b_ia <= d; b_ia++) {
          kinsim_2link_planar_B.c += C[b_ia - 1] * C[kinsim_2link_planar_B.ix_f
            - 1];
          kinsim_2link_planar_B.ix_f++;
        }

        work[kinsim_2link_planar_B.coltop] += kinsim_2link_planar_B.c;
        kinsim_2link_planar_B.coltop++;
        kinsim_2link_planar_B.iac_h += 4;
      }
    }

    if (!(-tau == 0.0)) {
      kinsim_2link_planar_B.coltop = ic0 - 1;
      jy = 0;
      kinsim_2link_planar_B.iac_h = 0;
      while (kinsim_2link_planar_B.iac_h <= kinsim_2link_planar_B.lastc_e) {
        if (work[jy] != 0.0) {
          kinsim_2link_planar_B.c = work[jy] * -tau;
          kinsim_2link_planar_B.ix_f = iv0;
          d = kinsim_2link_planar_B.lastv + kinsim_2link_planar_B.coltop;
          for (b_ia = kinsim_2link_planar_B.coltop; b_ia < d; b_ia++) {
            C[b_ia] += C[kinsim_2link_planar_B.ix_f - 1] *
              kinsim_2link_planar_B.c;
            kinsim_2link_planar_B.ix_f++;
          }
        }

        jy++;
        kinsim_2link_planar_B.coltop += 4;
        kinsim_2link_planar_B.iac_h++;
      }
    }
  }
}

// Function for MATLAB Function: '<S5>/MATLAB Function'
static void kinsim_2link_planar_xgehrd(real_T a[16], real_T tau[3])
{
  int32_T exitg1;
  boolean_T exitg2;
  kinsim_2link_planar_B.work_d[0] = 0.0;
  kinsim_2link_planar_B.work_d[1] = 0.0;
  kinsim_2link_planar_B.work_d[2] = 0.0;
  kinsim_2link_planar_B.work_d[3] = 0.0;
  kinsim_2link_planar_B.alpha1 = a[1];
  tau[0] = 0.0;
  kinsim_2link_planar_B.xnorm = kinsim_2link_planar_xnrm2(2, a, 3);
  if (kinsim_2link_planar_B.xnorm != 0.0) {
    kinsim_2link_planar_B.xnorm = kinsim_2link_plan_rt_hypotd_snf(a[1],
      kinsim_2link_planar_B.xnorm);
    if (a[1] >= 0.0) {
      kinsim_2link_planar_B.xnorm = -kinsim_2link_planar_B.xnorm;
    }

    if (fabs(kinsim_2link_planar_B.xnorm) < 1.0020841800044864E-292) {
      kinsim_2link_planar_B.knt = -1;
      do {
        kinsim_2link_planar_B.knt++;
        kinsim_2link_planar_B.lastc = 3;
        while (kinsim_2link_planar_B.lastc <= 4) {
          a[kinsim_2link_planar_B.lastc - 1] *= 9.9792015476736E+291;
          kinsim_2link_planar_B.lastc++;
        }

        kinsim_2link_planar_B.xnorm *= 9.9792015476736E+291;
        kinsim_2link_planar_B.alpha1 *= 9.9792015476736E+291;
      } while (!(fabs(kinsim_2link_planar_B.xnorm) >= 1.0020841800044864E-292));

      kinsim_2link_planar_B.xnorm = kinsim_2link_plan_rt_hypotd_snf
        (kinsim_2link_planar_B.alpha1, kinsim_2link_planar_xnrm2(2, a, 3));
      if (kinsim_2link_planar_B.alpha1 >= 0.0) {
        kinsim_2link_planar_B.xnorm = -kinsim_2link_planar_B.xnorm;
      }

      tau[0] = (kinsim_2link_planar_B.xnorm - kinsim_2link_planar_B.alpha1) /
        kinsim_2link_planar_B.xnorm;
      kinsim_2link_planar_B.alpha1 = 1.0 / (kinsim_2link_planar_B.alpha1 -
        kinsim_2link_planar_B.xnorm);
      kinsim_2link_planar_B.lastc = 3;
      while (kinsim_2link_planar_B.lastc <= 4) {
        a[kinsim_2link_planar_B.lastc - 1] *= kinsim_2link_planar_B.alpha1;
        kinsim_2link_planar_B.lastc++;
      }

      kinsim_2link_planar_B.lastc = 0;
      while (kinsim_2link_planar_B.lastc <= kinsim_2link_planar_B.knt) {
        kinsim_2link_planar_B.xnorm *= 1.0020841800044864E-292;
        kinsim_2link_planar_B.lastc++;
      }

      kinsim_2link_planar_B.alpha1 = kinsim_2link_planar_B.xnorm;
    } else {
      tau[0] = (kinsim_2link_planar_B.xnorm - a[1]) /
        kinsim_2link_planar_B.xnorm;
      kinsim_2link_planar_B.alpha1 = 1.0 / (a[1] - kinsim_2link_planar_B.xnorm);
      kinsim_2link_planar_B.knt = 3;
      while (kinsim_2link_planar_B.knt <= 4) {
        a[kinsim_2link_planar_B.knt - 1] *= kinsim_2link_planar_B.alpha1;
        kinsim_2link_planar_B.knt++;
      }

      kinsim_2link_planar_B.alpha1 = kinsim_2link_planar_B.xnorm;
    }
  }

  a[1] = 1.0;
  if (tau[0] != 0.0) {
    kinsim_2link_planar_B.knt = 2;
    kinsim_2link_planar_B.lastc = 3;
    while ((kinsim_2link_planar_B.knt + 1 > 0) && (a[kinsim_2link_planar_B.lastc]
            == 0.0)) {
      kinsim_2link_planar_B.knt--;
      kinsim_2link_planar_B.lastc--;
    }

    kinsim_2link_planar_B.lastc = 4;
    exitg2 = false;
    while ((!exitg2) && (kinsim_2link_planar_B.lastc > 0)) {
      kinsim_2link_planar_B.rowleft = kinsim_2link_planar_B.lastc + 4;
      kinsim_2link_planar_B.jy = kinsim_2link_planar_B.rowleft;
      do {
        exitg1 = 0;
        if (kinsim_2link_planar_B.jy <= (kinsim_2link_planar_B.knt << 2) +
            kinsim_2link_planar_B.rowleft) {
          if (a[kinsim_2link_planar_B.jy - 1] != 0.0) {
            exitg1 = 1;
          } else {
            kinsim_2link_planar_B.jy += 4;
          }
        } else {
          kinsim_2link_planar_B.lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    kinsim_2link_planar_B.knt = -1;
    kinsim_2link_planar_B.lastc = 0;
  }

  if (kinsim_2link_planar_B.knt + 1 > 0) {
    if (kinsim_2link_planar_B.lastc != 0) {
      kinsim_2link_planar_B.rowleft = 0;
      while (kinsim_2link_planar_B.rowleft <= kinsim_2link_planar_B.lastc - 1) {
        kinsim_2link_planar_B.work_d[kinsim_2link_planar_B.rowleft] = 0.0;
        kinsim_2link_planar_B.rowleft++;
      }

      kinsim_2link_planar_B.rowleft = 1;
      kinsim_2link_planar_B.jy = (kinsim_2link_planar_B.knt << 2) + 5;
      kinsim_2link_planar_B.iac = 5;
      while (kinsim_2link_planar_B.iac <= kinsim_2link_planar_B.jy) {
        kinsim_2link_planar_B.b_ix = 0;
        kinsim_2link_planar_B.g_g = (kinsim_2link_planar_B.iac +
          kinsim_2link_planar_B.lastc) - 1;
        kinsim_2link_planar_B.b_ia = kinsim_2link_planar_B.iac;
        while (kinsim_2link_planar_B.b_ia <= kinsim_2link_planar_B.g_g) {
          kinsim_2link_planar_B.work_d[kinsim_2link_planar_B.b_ix] +=
            a[kinsim_2link_planar_B.b_ia - 1] * a[kinsim_2link_planar_B.rowleft];
          kinsim_2link_planar_B.b_ix++;
          kinsim_2link_planar_B.b_ia++;
        }

        kinsim_2link_planar_B.rowleft++;
        kinsim_2link_planar_B.iac += 4;
      }
    }

    if (!(-tau[0] == 0.0)) {
      kinsim_2link_planar_B.rowleft = 4;
      kinsim_2link_planar_B.jy = 1;
      kinsim_2link_planar_B.iac = 0;
      while (kinsim_2link_planar_B.iac <= kinsim_2link_planar_B.knt) {
        if (a[kinsim_2link_planar_B.jy] != 0.0) {
          kinsim_2link_planar_B.xnorm = a[kinsim_2link_planar_B.jy] * -tau[0];
          kinsim_2link_planar_B.b_ix = 0;
          kinsim_2link_planar_B.g_g = kinsim_2link_planar_B.lastc +
            kinsim_2link_planar_B.rowleft;
          kinsim_2link_planar_B.b_ia = kinsim_2link_planar_B.rowleft;
          while (kinsim_2link_planar_B.b_ia + 1 <= kinsim_2link_planar_B.g_g) {
            a[kinsim_2link_planar_B.b_ia] +=
              kinsim_2link_planar_B.work_d[kinsim_2link_planar_B.b_ix] *
              kinsim_2link_planar_B.xnorm;
            kinsim_2link_planar_B.b_ix++;
            kinsim_2link_planar_B.b_ia++;
          }
        }

        kinsim_2link_planar_B.jy++;
        kinsim_2link_planar_B.rowleft += 4;
        kinsim_2link_planar_B.iac++;
      }
    }
  }

  kinsim_2link_planar_xzlarf(3, 3, 2, tau[0], a, 6, kinsim_2link_planar_B.work_d);
  a[1] = kinsim_2link_planar_B.alpha1;
  kinsim_2link_planar_B.alpha1 = a[6];
  tau[1] = 0.0;
  kinsim_2link_planar_B.xnorm = kinsim_2link_planar_xnrm2(1, a, 8);
  if (kinsim_2link_planar_B.xnorm != 0.0) {
    kinsim_2link_planar_B.xnorm = kinsim_2link_plan_rt_hypotd_snf(a[6],
      kinsim_2link_planar_B.xnorm);
    if (a[6] >= 0.0) {
      kinsim_2link_planar_B.xnorm = -kinsim_2link_planar_B.xnorm;
    }

    if (fabs(kinsim_2link_planar_B.xnorm) < 1.0020841800044864E-292) {
      kinsim_2link_planar_B.knt = -1;
      do {
        kinsim_2link_planar_B.knt++;
        a[7] *= 9.9792015476736E+291;
        kinsim_2link_planar_B.xnorm *= 9.9792015476736E+291;
        kinsim_2link_planar_B.alpha1 *= 9.9792015476736E+291;
      } while (!(fabs(kinsim_2link_planar_B.xnorm) >= 1.0020841800044864E-292));

      kinsim_2link_planar_B.xnorm = kinsim_2link_plan_rt_hypotd_snf
        (kinsim_2link_planar_B.alpha1, kinsim_2link_planar_xnrm2(1, a, 8));
      if (kinsim_2link_planar_B.alpha1 >= 0.0) {
        kinsim_2link_planar_B.xnorm = -kinsim_2link_planar_B.xnorm;
      }

      tau[1] = (kinsim_2link_planar_B.xnorm - kinsim_2link_planar_B.alpha1) /
        kinsim_2link_planar_B.xnorm;
      kinsim_2link_planar_B.alpha1 = 1.0 / (kinsim_2link_planar_B.alpha1 -
        kinsim_2link_planar_B.xnorm);
      a[7] *= kinsim_2link_planar_B.alpha1;
      kinsim_2link_planar_B.lastc = 0;
      while (kinsim_2link_planar_B.lastc <= kinsim_2link_planar_B.knt) {
        kinsim_2link_planar_B.xnorm *= 1.0020841800044864E-292;
        kinsim_2link_planar_B.lastc++;
      }

      kinsim_2link_planar_B.alpha1 = kinsim_2link_planar_B.xnorm;
    } else {
      tau[1] = (kinsim_2link_planar_B.xnorm - a[6]) /
        kinsim_2link_planar_B.xnorm;
      a[7] *= 1.0 / (a[6] - kinsim_2link_planar_B.xnorm);
      kinsim_2link_planar_B.alpha1 = kinsim_2link_planar_B.xnorm;
    }
  }

  a[6] = 1.0;
  if (tau[1] != 0.0) {
    kinsim_2link_planar_B.knt = 1;
    kinsim_2link_planar_B.lastc = 7;
    while ((kinsim_2link_planar_B.knt + 1 > 0) && (a[kinsim_2link_planar_B.lastc]
            == 0.0)) {
      kinsim_2link_planar_B.knt--;
      kinsim_2link_planar_B.lastc--;
    }

    kinsim_2link_planar_B.lastc = 4;
    exitg2 = false;
    while ((!exitg2) && (kinsim_2link_planar_B.lastc > 0)) {
      kinsim_2link_planar_B.rowleft = kinsim_2link_planar_B.lastc + 8;
      kinsim_2link_planar_B.jy = kinsim_2link_planar_B.rowleft;
      do {
        exitg1 = 0;
        if (kinsim_2link_planar_B.jy <= (kinsim_2link_planar_B.knt << 2) +
            kinsim_2link_planar_B.rowleft) {
          if (a[kinsim_2link_planar_B.jy - 1] != 0.0) {
            exitg1 = 1;
          } else {
            kinsim_2link_planar_B.jy += 4;
          }
        } else {
          kinsim_2link_planar_B.lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    kinsim_2link_planar_B.knt = -1;
    kinsim_2link_planar_B.lastc = 0;
  }

  if (kinsim_2link_planar_B.knt + 1 > 0) {
    if (kinsim_2link_planar_B.lastc != 0) {
      kinsim_2link_planar_B.rowleft = 0;
      while (kinsim_2link_planar_B.rowleft <= kinsim_2link_planar_B.lastc - 1) {
        kinsim_2link_planar_B.work_d[kinsim_2link_planar_B.rowleft] = 0.0;
        kinsim_2link_planar_B.rowleft++;
      }

      kinsim_2link_planar_B.rowleft = 6;
      kinsim_2link_planar_B.jy = (kinsim_2link_planar_B.knt << 2) + 9;
      kinsim_2link_planar_B.iac = 9;
      while (kinsim_2link_planar_B.iac <= kinsim_2link_planar_B.jy) {
        kinsim_2link_planar_B.b_ix = 0;
        kinsim_2link_planar_B.g_g = (kinsim_2link_planar_B.iac +
          kinsim_2link_planar_B.lastc) - 1;
        kinsim_2link_planar_B.b_ia = kinsim_2link_planar_B.iac;
        while (kinsim_2link_planar_B.b_ia <= kinsim_2link_planar_B.g_g) {
          kinsim_2link_planar_B.work_d[kinsim_2link_planar_B.b_ix] +=
            a[kinsim_2link_planar_B.b_ia - 1] * a[kinsim_2link_planar_B.rowleft];
          kinsim_2link_planar_B.b_ix++;
          kinsim_2link_planar_B.b_ia++;
        }

        kinsim_2link_planar_B.rowleft++;
        kinsim_2link_planar_B.iac += 4;
      }
    }

    if (!(-tau[1] == 0.0)) {
      kinsim_2link_planar_B.rowleft = 8;
      kinsim_2link_planar_B.jy = 6;
      kinsim_2link_planar_B.iac = 0;
      while (kinsim_2link_planar_B.iac <= kinsim_2link_planar_B.knt) {
        if (a[kinsim_2link_planar_B.jy] != 0.0) {
          kinsim_2link_planar_B.xnorm = a[kinsim_2link_planar_B.jy] * -tau[1];
          kinsim_2link_planar_B.b_ix = 0;
          kinsim_2link_planar_B.g_g = kinsim_2link_planar_B.lastc +
            kinsim_2link_planar_B.rowleft;
          kinsim_2link_planar_B.b_ia = kinsim_2link_planar_B.rowleft;
          while (kinsim_2link_planar_B.b_ia + 1 <= kinsim_2link_planar_B.g_g) {
            a[kinsim_2link_planar_B.b_ia] +=
              kinsim_2link_planar_B.work_d[kinsim_2link_planar_B.b_ix] *
              kinsim_2link_planar_B.xnorm;
            kinsim_2link_planar_B.b_ix++;
            kinsim_2link_planar_B.b_ia++;
          }
        }

        kinsim_2link_planar_B.jy++;
        kinsim_2link_planar_B.rowleft += 4;
        kinsim_2link_planar_B.iac++;
      }
    }
  }

  kinsim_2link_planar_xzlarf(2, 2, 7, tau[1], a, 11,
    kinsim_2link_planar_B.work_d);
  a[6] = kinsim_2link_planar_B.alpha1;
  kinsim_2link_planar_B.alpha1 = a[11];
  tau[2] = 0.0;
  kinsim_2link_planar_B.xnorm = kinsim_2link_planar_xnrm2(0, a, 12);
  if (kinsim_2link_planar_B.xnorm != 0.0) {
    kinsim_2link_planar_B.xnorm = kinsim_2link_plan_rt_hypotd_snf(a[11],
      kinsim_2link_planar_B.xnorm);
    if (a[11] >= 0.0) {
      kinsim_2link_planar_B.xnorm = -kinsim_2link_planar_B.xnorm;
    }

    if (fabs(kinsim_2link_planar_B.xnorm) < 1.0020841800044864E-292) {
      kinsim_2link_planar_B.knt = -1;
      do {
        kinsim_2link_planar_B.knt++;
        kinsim_2link_planar_B.xnorm *= 9.9792015476736E+291;
        kinsim_2link_planar_B.alpha1 *= 9.9792015476736E+291;
      } while (!(fabs(kinsim_2link_planar_B.xnorm) >= 1.0020841800044864E-292));

      kinsim_2link_planar_B.xnorm = kinsim_2link_plan_rt_hypotd_snf
        (kinsim_2link_planar_B.alpha1, kinsim_2link_planar_xnrm2(0, a, 12));
      if (kinsim_2link_planar_B.alpha1 >= 0.0) {
        kinsim_2link_planar_B.xnorm = -kinsim_2link_planar_B.xnorm;
      }

      tau[2] = (kinsim_2link_planar_B.xnorm - kinsim_2link_planar_B.alpha1) /
        kinsim_2link_planar_B.xnorm;
      kinsim_2link_planar_B.lastc = 0;
      while (kinsim_2link_planar_B.lastc <= kinsim_2link_planar_B.knt) {
        kinsim_2link_planar_B.xnorm *= 1.0020841800044864E-292;
        kinsim_2link_planar_B.lastc++;
      }

      kinsim_2link_planar_B.alpha1 = kinsim_2link_planar_B.xnorm;
    } else {
      tau[2] = (kinsim_2link_planar_B.xnorm - a[11]) /
        kinsim_2link_planar_B.xnorm;
      kinsim_2link_planar_B.alpha1 = kinsim_2link_planar_B.xnorm;
    }
  }

  a[11] = 1.0;
  if (tau[2] != 0.0) {
    kinsim_2link_planar_B.knt = 0;
    kinsim_2link_planar_B.lastc = 11;
    while ((kinsim_2link_planar_B.knt + 1 > 0) && (a[kinsim_2link_planar_B.lastc]
            == 0.0)) {
      kinsim_2link_planar_B.knt--;
      kinsim_2link_planar_B.lastc--;
    }

    kinsim_2link_planar_B.lastc = 4;
    exitg2 = false;
    while ((!exitg2) && (kinsim_2link_planar_B.lastc > 0)) {
      kinsim_2link_planar_B.rowleft = kinsim_2link_planar_B.lastc + 12;
      kinsim_2link_planar_B.jy = kinsim_2link_planar_B.rowleft;
      do {
        exitg1 = 0;
        if (kinsim_2link_planar_B.jy <= (kinsim_2link_planar_B.knt << 2) +
            kinsim_2link_planar_B.rowleft) {
          if (a[kinsim_2link_planar_B.jy - 1] != 0.0) {
            exitg1 = 1;
          } else {
            kinsim_2link_planar_B.jy += 4;
          }
        } else {
          kinsim_2link_planar_B.lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    kinsim_2link_planar_B.knt = -1;
    kinsim_2link_planar_B.lastc = 0;
  }

  if (kinsim_2link_planar_B.knt + 1 > 0) {
    if (kinsim_2link_planar_B.lastc != 0) {
      kinsim_2link_planar_B.rowleft = 0;
      while (kinsim_2link_planar_B.rowleft <= kinsim_2link_planar_B.lastc - 1) {
        kinsim_2link_planar_B.work_d[kinsim_2link_planar_B.rowleft] = 0.0;
        kinsim_2link_planar_B.rowleft++;
      }

      kinsim_2link_planar_B.rowleft = 11;
      kinsim_2link_planar_B.jy = (kinsim_2link_planar_B.knt << 2) + 13;
      kinsim_2link_planar_B.iac = 13;
      while (kinsim_2link_planar_B.iac <= kinsim_2link_planar_B.jy) {
        kinsim_2link_planar_B.b_ix = 0;
        kinsim_2link_planar_B.g_g = (kinsim_2link_planar_B.iac +
          kinsim_2link_planar_B.lastc) - 1;
        kinsim_2link_planar_B.b_ia = kinsim_2link_planar_B.iac;
        while (kinsim_2link_planar_B.b_ia <= kinsim_2link_planar_B.g_g) {
          kinsim_2link_planar_B.work_d[kinsim_2link_planar_B.b_ix] +=
            a[kinsim_2link_planar_B.b_ia - 1] * a[kinsim_2link_planar_B.rowleft];
          kinsim_2link_planar_B.b_ix++;
          kinsim_2link_planar_B.b_ia++;
        }

        kinsim_2link_planar_B.rowleft++;
        kinsim_2link_planar_B.iac += 4;
      }
    }

    if (!(-tau[2] == 0.0)) {
      kinsim_2link_planar_B.rowleft = 12;
      kinsim_2link_planar_B.jy = 11;
      kinsim_2link_planar_B.iac = 0;
      while (kinsim_2link_planar_B.iac <= kinsim_2link_planar_B.knt) {
        if (a[kinsim_2link_planar_B.jy] != 0.0) {
          kinsim_2link_planar_B.xnorm = a[kinsim_2link_planar_B.jy] * -tau[2];
          kinsim_2link_planar_B.b_ix = 0;
          kinsim_2link_planar_B.g_g = kinsim_2link_planar_B.lastc +
            kinsim_2link_planar_B.rowleft;
          kinsim_2link_planar_B.b_ia = kinsim_2link_planar_B.rowleft;
          while (kinsim_2link_planar_B.b_ia + 1 <= kinsim_2link_planar_B.g_g) {
            a[kinsim_2link_planar_B.b_ia] +=
              kinsim_2link_planar_B.work_d[kinsim_2link_planar_B.b_ix] *
              kinsim_2link_planar_B.xnorm;
            kinsim_2link_planar_B.b_ix++;
            kinsim_2link_planar_B.b_ia++;
          }
        }

        kinsim_2link_planar_B.jy++;
        kinsim_2link_planar_B.rowleft += 4;
        kinsim_2link_planar_B.iac++;
      }
    }
  }

  kinsim_2link_planar_xzlarf(1, 1, 12, tau[2], a, 16,
    kinsim_2link_planar_B.work_d);
  a[11] = kinsim_2link_planar_B.alpha1;
}

// Function for MATLAB Function: '<S5>/MATLAB Function'
static real_T kinsim_2link_planar_xnrm2_l(int32_T n, const real_T x[3])
{
  real_T y;
  real_T scale;
  real_T absxk;
  real_T t;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[1]);
    } else {
      scale = 3.3121686421112381E-170;
      absxk = fabs(x[1]);
      if (absxk > 3.3121686421112381E-170) {
        y = 1.0;
        scale = absxk;
      } else {
        t = absxk / 3.3121686421112381E-170;
        y = t * t;
      }

      absxk = fabs(x[2]);
      if (absxk > scale) {
        t = scale / absxk;
        y = y * t * t + 1.0;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

// Function for MATLAB Function: '<S5>/MATLAB Function'
static real_T kinsim_2link_planar_xzlarfg(int32_T n, real_T *alpha1, real_T x[3])
{
  real_T tau;
  real_T xnorm;
  int32_T knt;
  int32_T c_k;
  tau = 0.0;
  if (n > 0) {
    xnorm = kinsim_2link_planar_xnrm2_l(n - 1, x);
    if (xnorm != 0.0) {
      xnorm = kinsim_2link_plan_rt_hypotd_snf(*alpha1, xnorm);
      if (*alpha1 >= 0.0) {
        xnorm = -xnorm;
      }

      if (fabs(xnorm) < 1.0020841800044864E-292) {
        knt = -1;
        do {
          knt++;
          for (c_k = 1; c_k < n; c_k++) {
            x[c_k] *= 9.9792015476736E+291;
          }

          xnorm *= 9.9792015476736E+291;
          *alpha1 *= 9.9792015476736E+291;
        } while (!(fabs(xnorm) >= 1.0020841800044864E-292));

        xnorm = kinsim_2link_plan_rt_hypotd_snf(*alpha1,
          kinsim_2link_planar_xnrm2_l(n - 1, x));
        if (*alpha1 >= 0.0) {
          xnorm = -xnorm;
        }

        tau = (xnorm - *alpha1) / xnorm;
        *alpha1 = 1.0 / (*alpha1 - xnorm);
        for (c_k = 1; c_k < n; c_k++) {
          x[c_k] *= *alpha1;
        }

        for (c_k = 0; c_k <= knt; c_k++) {
          xnorm *= 1.0020841800044864E-292;
        }

        *alpha1 = xnorm;
      } else {
        tau = (xnorm - *alpha1) / xnorm;
        *alpha1 = 1.0 / (*alpha1 - xnorm);
        for (knt = 1; knt < n; knt++) {
          x[knt] *= *alpha1;
        }

        *alpha1 = xnorm;
      }
    }
  }

  return tau;
}

// Function for MATLAB Function: '<S5>/MATLAB Function'
static void kinsim_2link_planar_xdlanv2(real_T *a, real_T *b, real_T *c, real_T *
  d, real_T *rt1r, real_T *rt1i, real_T *rt2r, real_T *rt2i, real_T *cs, real_T *
  sn)
{
  int32_T b_0;
  int32_T c_0;
  boolean_T tmp;
  if (*c == 0.0) {
    *cs = 1.0;
    *sn = 0.0;
  } else if (*b == 0.0) {
    *cs = 0.0;
    *sn = 1.0;
    kinsim_2link_planar_B.bcmax = *d;
    *d = *a;
    *a = kinsim_2link_planar_B.bcmax;
    *b = -*c;
    *c = 0.0;
  } else {
    kinsim_2link_planar_B.tau_m = *a - *d;
    if ((kinsim_2link_planar_B.tau_m == 0.0) && ((*b < 0.0) != (*c < 0.0))) {
      *cs = 1.0;
      *sn = 0.0;
    } else {
      kinsim_2link_planar_B.p = 0.5 * kinsim_2link_planar_B.tau_m;
      kinsim_2link_planar_B.bcmis = fabs(*b);
      kinsim_2link_planar_B.z = fabs(*c);
      tmp = rtIsNaN(kinsim_2link_planar_B.z);
      if ((kinsim_2link_planar_B.bcmis > kinsim_2link_planar_B.z) || tmp) {
        kinsim_2link_planar_B.bcmax = kinsim_2link_planar_B.bcmis;
      } else {
        kinsim_2link_planar_B.bcmax = kinsim_2link_planar_B.z;
      }

      if ((kinsim_2link_planar_B.bcmis < kinsim_2link_planar_B.z) || tmp) {
        kinsim_2link_planar_B.z = kinsim_2link_planar_B.bcmis;
      }

      if (!(*b < 0.0)) {
        b_0 = 1;
      } else {
        b_0 = -1;
      }

      if (!(*c < 0.0)) {
        c_0 = 1;
      } else {
        c_0 = -1;
      }

      kinsim_2link_planar_B.bcmis = kinsim_2link_planar_B.z * static_cast<real_T>
        (b_0) * static_cast<real_T>(c_0);
      kinsim_2link_planar_B.scale_c = fabs(kinsim_2link_planar_B.p);
      if ((!(kinsim_2link_planar_B.scale_c > kinsim_2link_planar_B.bcmax)) &&
          (!rtIsNaN(kinsim_2link_planar_B.bcmax))) {
        kinsim_2link_planar_B.scale_c = kinsim_2link_planar_B.bcmax;
      }

      kinsim_2link_planar_B.z = kinsim_2link_planar_B.p /
        kinsim_2link_planar_B.scale_c * kinsim_2link_planar_B.p +
        kinsim_2link_planar_B.bcmax / kinsim_2link_planar_B.scale_c *
        kinsim_2link_planar_B.bcmis;
      if (kinsim_2link_planar_B.z >= 8.8817841970012523E-16) {
        if (!(kinsim_2link_planar_B.p < 0.0)) {
          kinsim_2link_planar_B.tau_m = sqrt(kinsim_2link_planar_B.scale_c) *
            sqrt(kinsim_2link_planar_B.z);
        } else {
          kinsim_2link_planar_B.tau_m = -(sqrt(kinsim_2link_planar_B.scale_c) *
            sqrt(kinsim_2link_planar_B.z));
        }

        kinsim_2link_planar_B.z = kinsim_2link_planar_B.p +
          kinsim_2link_planar_B.tau_m;
        *a = *d + kinsim_2link_planar_B.z;
        *d -= kinsim_2link_planar_B.bcmax / kinsim_2link_planar_B.z *
          kinsim_2link_planar_B.bcmis;
        kinsim_2link_planar_B.tau_m = kinsim_2link_plan_rt_hypotd_snf(*c,
          kinsim_2link_planar_B.z);
        *cs = kinsim_2link_planar_B.z / kinsim_2link_planar_B.tau_m;
        *sn = *c / kinsim_2link_planar_B.tau_m;
        *b -= *c;
        *c = 0.0;
      } else {
        kinsim_2link_planar_B.bcmax = *b + *c;
        kinsim_2link_planar_B.tau_m = kinsim_2link_plan_rt_hypotd_snf
          (kinsim_2link_planar_B.bcmax, kinsim_2link_planar_B.tau_m);
        *cs = sqrt((fabs(kinsim_2link_planar_B.bcmax) /
                    kinsim_2link_planar_B.tau_m + 1.0) * 0.5);
        if (!(kinsim_2link_planar_B.bcmax < 0.0)) {
          b_0 = 1;
        } else {
          b_0 = -1;
        }

        *sn = -(kinsim_2link_planar_B.p / (kinsim_2link_planar_B.tau_m * *cs)) *
          static_cast<real_T>(b_0);
        kinsim_2link_planar_B.p = *a * *cs + *b * *sn;
        kinsim_2link_planar_B.tau_m = -*a * *sn + *b * *cs;
        kinsim_2link_planar_B.bcmax = *c * *cs + *d * *sn;
        kinsim_2link_planar_B.bcmis = -*c * *sn + *d * *cs;
        *b = kinsim_2link_planar_B.tau_m * *cs + kinsim_2link_planar_B.bcmis *
          *sn;
        *c = -kinsim_2link_planar_B.p * *sn + kinsim_2link_planar_B.bcmax * *cs;
        kinsim_2link_planar_B.bcmax = ((kinsim_2link_planar_B.p * *cs +
          kinsim_2link_planar_B.bcmax * *sn) + (-kinsim_2link_planar_B.tau_m *
          *sn + kinsim_2link_planar_B.bcmis * *cs)) * 0.5;
        *a = kinsim_2link_planar_B.bcmax;
        *d = kinsim_2link_planar_B.bcmax;
        if (*c != 0.0) {
          if (*b != 0.0) {
            if ((*b < 0.0) == (*c < 0.0)) {
              kinsim_2link_planar_B.z = sqrt(fabs(*b));
              kinsim_2link_planar_B.bcmis = sqrt(fabs(*c));
              if (!(*c < 0.0)) {
                kinsim_2link_planar_B.p = kinsim_2link_planar_B.z *
                  kinsim_2link_planar_B.bcmis;
              } else {
                kinsim_2link_planar_B.p = -(kinsim_2link_planar_B.z *
                  kinsim_2link_planar_B.bcmis);
              }

              kinsim_2link_planar_B.tau_m = 1.0 / sqrt(fabs(*b + *c));
              *a = kinsim_2link_planar_B.bcmax + kinsim_2link_planar_B.p;
              *d = kinsim_2link_planar_B.bcmax - kinsim_2link_planar_B.p;
              *b -= *c;
              *c = 0.0;
              kinsim_2link_planar_B.p = kinsim_2link_planar_B.z *
                kinsim_2link_planar_B.tau_m;
              kinsim_2link_planar_B.tau_m *= kinsim_2link_planar_B.bcmis;
              kinsim_2link_planar_B.bcmax = *cs * kinsim_2link_planar_B.p - *sn *
                kinsim_2link_planar_B.tau_m;
              *sn = *cs * kinsim_2link_planar_B.tau_m + *sn *
                kinsim_2link_planar_B.p;
              *cs = kinsim_2link_planar_B.bcmax;
            }
          } else {
            *b = -*c;
            *c = 0.0;
            kinsim_2link_planar_B.bcmax = *cs;
            *cs = -*sn;
            *sn = kinsim_2link_planar_B.bcmax;
          }
        }
      }
    }
  }

  *rt1r = *a;
  *rt2r = *d;
  if (*c == 0.0) {
    *rt1i = 0.0;
    *rt2i = 0.0;
  } else {
    *rt1i = sqrt(fabs(*b)) * sqrt(fabs(*c));
    *rt2i = -*rt1i;
  }
}

// Function for MATLAB Function: '<S5>/MATLAB Function'
static void kinsim_2link_planar_xrot(int32_T n, real_T x[16], int32_T ix0,
  int32_T iy0, real_T c, real_T s)
{
  int32_T ix;
  int32_T iy;
  real_T temp;
  int32_T k;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < n; k++) {
    temp = c * x[ix] + s * x[iy];
    x[iy] = c * x[iy] - s * x[ix];
    x[ix] = temp;
    iy += 4;
    ix += 4;
  }
}

// Function for MATLAB Function: '<S5>/MATLAB Function'
static void kinsim_2link_planar_xrot_g(int32_T n, real_T x[16], int32_T ix0,
  int32_T iy0, real_T c, real_T s)
{
  int32_T ix;
  int32_T iy;
  real_T temp;
  int32_T k;
  if (n >= 1) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      temp = c * x[ix] + s * x[iy];
      x[iy] = c * x[iy] - s * x[ix];
      x[ix] = temp;
      iy++;
      ix++;
    }
  }
}

// Function for MATLAB Function: '<S5>/MATLAB Function'
static int32_T kinsim_2link_planar_eml_dlahqr(real_T h[16], real_T z[16])
{
  int32_T info;
  boolean_T goto150;
  int32_T s_tmp;
  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T exitg3;
  info = 0;
  kinsim_2link_planar_B.v_dy[0] = 0.0;
  kinsim_2link_planar_B.v_dy[1] = 0.0;
  kinsim_2link_planar_B.v_dy[2] = 0.0;
  h[2] = 0.0;
  h[3] = 0.0;
  h[7] = 0.0;
  kinsim_2link_planar_B.i_a = 3;
  exitg1 = false;
  while ((!exitg1) && (kinsim_2link_planar_B.i_a + 1 >= 1)) {
    kinsim_2link_planar_B.L = 1;
    goto150 = false;
    kinsim_2link_planar_B.ix = 0;
    exitg2 = false;
    while ((!exitg2) && (kinsim_2link_planar_B.ix < 301)) {
      kinsim_2link_planar_B.k_j = kinsim_2link_planar_B.i_a;
      exitg3 = false;
      while ((!exitg3) && (kinsim_2link_planar_B.k_j + 1 >
                           kinsim_2link_planar_B.L)) {
        s_tmp = ((kinsim_2link_planar_B.k_j - 1) << 2) +
          kinsim_2link_planar_B.k_j;
        kinsim_2link_planar_B.htmp2 = fabs(h[s_tmp]);
        if (kinsim_2link_planar_B.htmp2 <= 4.0083367200179456E-292) {
          exitg3 = true;
        } else {
          kinsim_2link_planar_B.m = (kinsim_2link_planar_B.k_j << 2) +
            kinsim_2link_planar_B.k_j;
          kinsim_2link_planar_B.nr = s_tmp - 1;
          kinsim_2link_planar_B.tst = fabs(h[kinsim_2link_planar_B.nr]) + fabs
            (h[kinsim_2link_planar_B.m]);
          if (kinsim_2link_planar_B.tst == 0.0) {
            if (kinsim_2link_planar_B.k_j - 1 >= 1) {
              kinsim_2link_planar_B.tst = fabs(h[(((kinsim_2link_planar_B.k_j -
                2) << 2) + kinsim_2link_planar_B.k_j) - 1]);
            }

            if (kinsim_2link_planar_B.k_j + 2 <= 4) {
              kinsim_2link_planar_B.tst += fabs(h[((kinsim_2link_planar_B.k_j <<
                2) + kinsim_2link_planar_B.k_j) + 1]);
            }
          }

          if (kinsim_2link_planar_B.htmp2 <= 2.2204460492503131E-16 *
              kinsim_2link_planar_B.tst) {
            kinsim_2link_planar_B.htmp1 = fabs(h[s_tmp]);
            kinsim_2link_planar_B.htmp2 = fabs(h[kinsim_2link_planar_B.m - 1]);
            if (kinsim_2link_planar_B.htmp1 > kinsim_2link_planar_B.htmp2) {
              kinsim_2link_planar_B.tst = kinsim_2link_planar_B.htmp1;
              kinsim_2link_planar_B.ba = kinsim_2link_planar_B.htmp2;
            } else {
              kinsim_2link_planar_B.tst = kinsim_2link_planar_B.htmp2;
              kinsim_2link_planar_B.ba = kinsim_2link_planar_B.htmp1;
            }

            kinsim_2link_planar_B.htmp1 = fabs(h[kinsim_2link_planar_B.m]);
            kinsim_2link_planar_B.htmp2 = fabs(h[kinsim_2link_planar_B.nr] -
              h[kinsim_2link_planar_B.m]);
            if (kinsim_2link_planar_B.htmp1 > kinsim_2link_planar_B.htmp2) {
              kinsim_2link_planar_B.aa = kinsim_2link_planar_B.htmp1;
              kinsim_2link_planar_B.htmp1 = kinsim_2link_planar_B.htmp2;
            } else {
              kinsim_2link_planar_B.aa = kinsim_2link_planar_B.htmp2;
            }

            kinsim_2link_planar_B.htmp2 = kinsim_2link_planar_B.aa +
              kinsim_2link_planar_B.tst;
            kinsim_2link_planar_B.htmp1 = kinsim_2link_planar_B.aa /
              kinsim_2link_planar_B.htmp2 * kinsim_2link_planar_B.htmp1 *
              2.2204460492503131E-16;
            if ((4.0083367200179456E-292 > kinsim_2link_planar_B.htmp1) ||
                rtIsNaN(kinsim_2link_planar_B.htmp1)) {
              kinsim_2link_planar_B.htmp1 = 4.0083367200179456E-292;
            }

            if (kinsim_2link_planar_B.tst / kinsim_2link_planar_B.htmp2 *
                kinsim_2link_planar_B.ba <= kinsim_2link_planar_B.htmp1) {
              exitg3 = true;
            } else {
              kinsim_2link_planar_B.k_j--;
            }
          } else {
            kinsim_2link_planar_B.k_j--;
          }
        }
      }

      kinsim_2link_planar_B.L = kinsim_2link_planar_B.k_j + 1;
      if (kinsim_2link_planar_B.k_j + 1 > 1) {
        h[kinsim_2link_planar_B.k_j + ((kinsim_2link_planar_B.k_j - 1) << 2)] =
          0.0;
      }

      if (kinsim_2link_planar_B.k_j + 1 >= kinsim_2link_planar_B.i_a) {
        goto150 = true;
        exitg2 = true;
      } else {
        switch (kinsim_2link_planar_B.ix) {
         case 10:
          s_tmp = (kinsim_2link_planar_B.k_j << 2) + kinsim_2link_planar_B.k_j;
          kinsim_2link_planar_B.htmp2 = fabs(h[(((kinsim_2link_planar_B.k_j + 1)
            << 2) + kinsim_2link_planar_B.k_j) + 2]) + fabs(h[s_tmp + 1]);
          kinsim_2link_planar_B.ba = h[s_tmp] + 0.75 *
            kinsim_2link_planar_B.htmp2;
          kinsim_2link_planar_B.h12 = -0.4375 * kinsim_2link_planar_B.htmp2;
          kinsim_2link_planar_B.aa = kinsim_2link_planar_B.htmp2;
          kinsim_2link_planar_B.tst = kinsim_2link_planar_B.ba;
          break;

         case 20:
          kinsim_2link_planar_B.htmp2 = fabs(h[(((kinsim_2link_planar_B.i_a - 2)
            << 2) + kinsim_2link_planar_B.i_a) - 1]) + fabs(h
            [((kinsim_2link_planar_B.i_a - 1) << 2) + kinsim_2link_planar_B.i_a]);
          kinsim_2link_planar_B.ba = h[(kinsim_2link_planar_B.i_a << 2) +
            kinsim_2link_planar_B.i_a] + 0.75 * kinsim_2link_planar_B.htmp2;
          kinsim_2link_planar_B.h12 = -0.4375 * kinsim_2link_planar_B.htmp2;
          kinsim_2link_planar_B.aa = kinsim_2link_planar_B.htmp2;
          kinsim_2link_planar_B.tst = kinsim_2link_planar_B.ba;
          break;

         default:
          kinsim_2link_planar_B.ba = h[(((kinsim_2link_planar_B.i_a - 1) << 2) +
            kinsim_2link_planar_B.i_a) - 1];
          kinsim_2link_planar_B.aa = h[((kinsim_2link_planar_B.i_a - 1) << 2) +
            kinsim_2link_planar_B.i_a];
          kinsim_2link_planar_B.h12 = h[((kinsim_2link_planar_B.i_a << 2) +
            kinsim_2link_planar_B.i_a) - 1];
          kinsim_2link_planar_B.tst = h[(kinsim_2link_planar_B.i_a << 2) +
            kinsim_2link_planar_B.i_a];
          break;
        }

        kinsim_2link_planar_B.htmp2 = ((fabs(kinsim_2link_planar_B.ba) + fabs
          (kinsim_2link_planar_B.h12)) + fabs(kinsim_2link_planar_B.aa)) + fabs
          (kinsim_2link_planar_B.tst);
        if (kinsim_2link_planar_B.htmp2 == 0.0) {
          kinsim_2link_planar_B.ba = 0.0;
          kinsim_2link_planar_B.tst = 0.0;
          kinsim_2link_planar_B.htmp1 = 0.0;
          kinsim_2link_planar_B.aa = 0.0;
        } else {
          kinsim_2link_planar_B.ba /= kinsim_2link_planar_B.htmp2;
          kinsim_2link_planar_B.aa /= kinsim_2link_planar_B.htmp2;
          kinsim_2link_planar_B.h12 /= kinsim_2link_planar_B.htmp2;
          kinsim_2link_planar_B.tst /= kinsim_2link_planar_B.htmp2;
          kinsim_2link_planar_B.htmp1 = (kinsim_2link_planar_B.ba +
            kinsim_2link_planar_B.tst) / 2.0;
          kinsim_2link_planar_B.ba = (kinsim_2link_planar_B.ba -
            kinsim_2link_planar_B.htmp1) * (kinsim_2link_planar_B.tst -
            kinsim_2link_planar_B.htmp1) - kinsim_2link_planar_B.h12 *
            kinsim_2link_planar_B.aa;
          kinsim_2link_planar_B.aa = sqrt(fabs(kinsim_2link_planar_B.ba));
          if (kinsim_2link_planar_B.ba >= 0.0) {
            kinsim_2link_planar_B.ba = kinsim_2link_planar_B.htmp1 *
              kinsim_2link_planar_B.htmp2;
            kinsim_2link_planar_B.htmp1 = kinsim_2link_planar_B.ba;
            kinsim_2link_planar_B.tst = kinsim_2link_planar_B.aa *
              kinsim_2link_planar_B.htmp2;
            kinsim_2link_planar_B.aa = -kinsim_2link_planar_B.tst;
          } else {
            kinsim_2link_planar_B.ba = kinsim_2link_planar_B.htmp1 +
              kinsim_2link_planar_B.aa;
            kinsim_2link_planar_B.htmp1 -= kinsim_2link_planar_B.aa;
            if (fabs(kinsim_2link_planar_B.ba - kinsim_2link_planar_B.tst) <=
                fabs(kinsim_2link_planar_B.htmp1 - kinsim_2link_planar_B.tst)) {
              kinsim_2link_planar_B.ba *= kinsim_2link_planar_B.htmp2;
              kinsim_2link_planar_B.htmp1 = kinsim_2link_planar_B.ba;
            } else {
              kinsim_2link_planar_B.htmp1 *= kinsim_2link_planar_B.htmp2;
              kinsim_2link_planar_B.ba = kinsim_2link_planar_B.htmp1;
            }

            kinsim_2link_planar_B.tst = 0.0;
            kinsim_2link_planar_B.aa = 0.0;
          }
        }

        kinsim_2link_planar_B.m = kinsim_2link_planar_B.i_a - 1;
        exitg3 = false;
        while ((!exitg3) && (kinsim_2link_planar_B.m >=
                             kinsim_2link_planar_B.k_j + 1)) {
          s_tmp = ((kinsim_2link_planar_B.m - 1) << 2) + kinsim_2link_planar_B.m;
          kinsim_2link_planar_B.nr = s_tmp - 1;
          kinsim_2link_planar_B.h12 = h[kinsim_2link_planar_B.nr] -
            kinsim_2link_planar_B.htmp1;
          kinsim_2link_planar_B.htmp2 = (fabs(kinsim_2link_planar_B.h12) + fabs
            (kinsim_2link_planar_B.aa)) + fabs(h[s_tmp]);
          kinsim_2link_planar_B.h21s = h[s_tmp] / kinsim_2link_planar_B.htmp2;
          kinsim_2link_planar_B.hoffset = (kinsim_2link_planar_B.m << 2) +
            kinsim_2link_planar_B.m;
          kinsim_2link_planar_B.v_dy[0] = (kinsim_2link_planar_B.h12 /
            kinsim_2link_planar_B.htmp2 * (h[kinsim_2link_planar_B.nr] -
            kinsim_2link_planar_B.ba) + h[kinsim_2link_planar_B.hoffset - 1] *
            kinsim_2link_planar_B.h21s) - kinsim_2link_planar_B.aa /
            kinsim_2link_planar_B.htmp2 * kinsim_2link_planar_B.tst;
          kinsim_2link_planar_B.v_dy[1] = (((h[kinsim_2link_planar_B.nr] +
            h[kinsim_2link_planar_B.hoffset]) - kinsim_2link_planar_B.ba) -
            kinsim_2link_planar_B.htmp1) * kinsim_2link_planar_B.h21s;
          kinsim_2link_planar_B.v_dy[2] = h[kinsim_2link_planar_B.hoffset + 1] *
            kinsim_2link_planar_B.h21s;
          kinsim_2link_planar_B.htmp2 = (fabs(kinsim_2link_planar_B.v_dy[0]) +
            fabs(kinsim_2link_planar_B.v_dy[1])) + fabs
            (kinsim_2link_planar_B.v_dy[2]);
          kinsim_2link_planar_B.v_dy[0] /= kinsim_2link_planar_B.htmp2;
          kinsim_2link_planar_B.v_dy[1] /= kinsim_2link_planar_B.htmp2;
          kinsim_2link_planar_B.v_dy[2] /= kinsim_2link_planar_B.htmp2;
          if (kinsim_2link_planar_B.k_j + 1 == kinsim_2link_planar_B.m) {
            exitg3 = true;
          } else {
            s_tmp = ((kinsim_2link_planar_B.m - 2) << 2) +
              kinsim_2link_planar_B.m;
            if (fabs(h[s_tmp - 1]) * (fabs(kinsim_2link_planar_B.v_dy[1]) + fabs
                 (kinsim_2link_planar_B.v_dy[2])) <= ((fabs(h[s_tmp - 2]) + fabs
                  (h[kinsim_2link_planar_B.nr])) + fabs
                 (h[kinsim_2link_planar_B.hoffset])) * (2.2204460492503131E-16 *
                 fabs(kinsim_2link_planar_B.v_dy[0]))) {
              exitg3 = true;
            } else {
              kinsim_2link_planar_B.m--;
            }
          }
        }

        for (s_tmp = kinsim_2link_planar_B.m; s_tmp <= kinsim_2link_planar_B.i_a;
             s_tmp++) {
          kinsim_2link_planar_B.nr = (kinsim_2link_planar_B.i_a - s_tmp) + 2;
          if (3 < kinsim_2link_planar_B.nr) {
            kinsim_2link_planar_B.nr = 3;
          }

          if (s_tmp > kinsim_2link_planar_B.m) {
            kinsim_2link_planar_B.hoffset = ((s_tmp - 2) << 2) + s_tmp;
            kinsim_2link_planar_B.j_e = 0;
            while (kinsim_2link_planar_B.j_e <= kinsim_2link_planar_B.nr - 1) {
              kinsim_2link_planar_B.v_dy[kinsim_2link_planar_B.j_e] = h
                [(kinsim_2link_planar_B.j_e + kinsim_2link_planar_B.hoffset) - 1];
              kinsim_2link_planar_B.j_e++;
            }
          }

          kinsim_2link_planar_B.tst = kinsim_2link_planar_B.v_dy[0];
          kinsim_2link_planar_B.b_v[0] = kinsim_2link_planar_B.v_dy[0];
          kinsim_2link_planar_B.b_v[1] = kinsim_2link_planar_B.v_dy[1];
          kinsim_2link_planar_B.b_v[2] = kinsim_2link_planar_B.v_dy[2];
          kinsim_2link_planar_B.htmp2 = kinsim_2link_planar_xzlarfg
            (kinsim_2link_planar_B.nr, &kinsim_2link_planar_B.tst,
             kinsim_2link_planar_B.b_v);
          kinsim_2link_planar_B.v_dy[1] = kinsim_2link_planar_B.b_v[1];
          kinsim_2link_planar_B.v_dy[2] = kinsim_2link_planar_B.b_v[2];
          kinsim_2link_planar_B.v_dy[0] = kinsim_2link_planar_B.tst;
          if (s_tmp > kinsim_2link_planar_B.m) {
            h[(s_tmp + ((s_tmp - 2) << 2)) - 1] = kinsim_2link_planar_B.tst;
            h[s_tmp + ((s_tmp - 2) << 2)] = 0.0;
            if (s_tmp < kinsim_2link_planar_B.i_a) {
              h[s_tmp + 1] = 0.0;
            }
          } else {
            if (kinsim_2link_planar_B.m > kinsim_2link_planar_B.k_j + 1) {
              h[s_tmp - 1] *= 1.0 - kinsim_2link_planar_B.htmp2;
            }
          }

          kinsim_2link_planar_B.tst = kinsim_2link_planar_B.b_v[1];
          kinsim_2link_planar_B.ba = kinsim_2link_planar_B.htmp2 *
            kinsim_2link_planar_B.b_v[1];
          switch (kinsim_2link_planar_B.nr) {
           case 3:
            kinsim_2link_planar_B.htmp1 = kinsim_2link_planar_B.b_v[2];
            kinsim_2link_planar_B.aa = kinsim_2link_planar_B.htmp2 *
              kinsim_2link_planar_B.b_v[2];
            kinsim_2link_planar_B.b_j_o = s_tmp - 1;
            while (kinsim_2link_planar_B.b_j_o + 1 < 5) {
              kinsim_2link_planar_B.nr = (kinsim_2link_planar_B.b_j_o << 2) +
                s_tmp;
              kinsim_2link_planar_B.hoffset = kinsim_2link_planar_B.nr - 1;
              kinsim_2link_planar_B.j_e = kinsim_2link_planar_B.nr + 1;
              kinsim_2link_planar_B.h12 = (h[kinsim_2link_planar_B.hoffset] +
                h[kinsim_2link_planar_B.nr] * kinsim_2link_planar_B.tst) +
                h[kinsim_2link_planar_B.j_e] * kinsim_2link_planar_B.htmp1;
              h[kinsim_2link_planar_B.hoffset] -= kinsim_2link_planar_B.h12 *
                kinsim_2link_planar_B.htmp2;
              h[kinsim_2link_planar_B.nr] -= kinsim_2link_planar_B.h12 *
                kinsim_2link_planar_B.ba;
              h[kinsim_2link_planar_B.j_e] -= kinsim_2link_planar_B.h12 *
                kinsim_2link_planar_B.aa;
              kinsim_2link_planar_B.b_j_o++;
            }

            kinsim_2link_planar_B.nr = s_tmp + 3;
            kinsim_2link_planar_B.b_j_o = kinsim_2link_planar_B.i_a + 1;
            if (kinsim_2link_planar_B.nr < kinsim_2link_planar_B.b_j_o) {
              kinsim_2link_planar_B.b_j_o = kinsim_2link_planar_B.nr;
            }

            kinsim_2link_planar_B.c_j = 0;
            while (kinsim_2link_planar_B.c_j <= kinsim_2link_planar_B.b_j_o - 1)
            {
              kinsim_2link_planar_B.nr = ((s_tmp - 1) << 2) +
                kinsim_2link_planar_B.c_j;
              kinsim_2link_planar_B.hoffset = (s_tmp << 2) +
                kinsim_2link_planar_B.c_j;
              kinsim_2link_planar_B.j_e = ((s_tmp + 1) << 2) +
                kinsim_2link_planar_B.c_j;
              kinsim_2link_planar_B.h12 = (h[kinsim_2link_planar_B.nr] +
                h[kinsim_2link_planar_B.hoffset] * kinsim_2link_planar_B.tst) +
                h[kinsim_2link_planar_B.j_e] * kinsim_2link_planar_B.htmp1;
              h[kinsim_2link_planar_B.nr] -= kinsim_2link_planar_B.h12 *
                kinsim_2link_planar_B.htmp2;
              h[kinsim_2link_planar_B.hoffset] -= kinsim_2link_planar_B.h12 *
                kinsim_2link_planar_B.ba;
              h[kinsim_2link_planar_B.j_e] -= kinsim_2link_planar_B.h12 *
                kinsim_2link_planar_B.aa;
              kinsim_2link_planar_B.c_j++;
            }

            for (kinsim_2link_planar_B.b_j_o = 0; kinsim_2link_planar_B.b_j_o <
                 4; kinsim_2link_planar_B.b_j_o++) {
              kinsim_2link_planar_B.nr = ((s_tmp - 1) << 2) +
                kinsim_2link_planar_B.b_j_o;
              kinsim_2link_planar_B.hoffset = (s_tmp << 2) +
                kinsim_2link_planar_B.b_j_o;
              kinsim_2link_planar_B.j_e = ((s_tmp + 1) << 2) +
                kinsim_2link_planar_B.b_j_o;
              kinsim_2link_planar_B.h12 = (z[kinsim_2link_planar_B.nr] +
                z[kinsim_2link_planar_B.hoffset] * kinsim_2link_planar_B.tst) +
                z[kinsim_2link_planar_B.j_e] * kinsim_2link_planar_B.htmp1;
              z[kinsim_2link_planar_B.nr] -= kinsim_2link_planar_B.h12 *
                kinsim_2link_planar_B.htmp2;
              z[kinsim_2link_planar_B.hoffset] -= kinsim_2link_planar_B.h12 *
                kinsim_2link_planar_B.ba;
              z[kinsim_2link_planar_B.j_e] -= kinsim_2link_planar_B.h12 *
                kinsim_2link_planar_B.aa;
            }
            break;

           case 2:
            kinsim_2link_planar_B.j_e = s_tmp - 1;
            while (kinsim_2link_planar_B.j_e + 1 < 5) {
              kinsim_2link_planar_B.nr = (kinsim_2link_planar_B.j_e << 2) +
                s_tmp;
              kinsim_2link_planar_B.hoffset = kinsim_2link_planar_B.nr - 1;
              kinsim_2link_planar_B.h12 = h[kinsim_2link_planar_B.hoffset] +
                h[kinsim_2link_planar_B.nr] * kinsim_2link_planar_B.tst;
              h[kinsim_2link_planar_B.hoffset] -= kinsim_2link_planar_B.h12 *
                kinsim_2link_planar_B.htmp2;
              h[kinsim_2link_planar_B.nr] -= kinsim_2link_planar_B.h12 *
                kinsim_2link_planar_B.ba;
              kinsim_2link_planar_B.j_e++;
            }

            kinsim_2link_planar_B.j_e = 0;
            while (kinsim_2link_planar_B.j_e <= kinsim_2link_planar_B.i_a) {
              kinsim_2link_planar_B.nr = ((s_tmp - 1) << 2) +
                kinsim_2link_planar_B.j_e;
              kinsim_2link_planar_B.hoffset = (s_tmp << 2) +
                kinsim_2link_planar_B.j_e;
              kinsim_2link_planar_B.h12 = h[kinsim_2link_planar_B.nr] +
                h[kinsim_2link_planar_B.hoffset] * kinsim_2link_planar_B.tst;
              h[kinsim_2link_planar_B.nr] -= kinsim_2link_planar_B.h12 *
                kinsim_2link_planar_B.htmp2;
              h[kinsim_2link_planar_B.hoffset] -= kinsim_2link_planar_B.h12 *
                kinsim_2link_planar_B.ba;
              kinsim_2link_planar_B.j_e++;
            }

            for (kinsim_2link_planar_B.j_e = 0; kinsim_2link_planar_B.j_e < 4;
                 kinsim_2link_planar_B.j_e++) {
              kinsim_2link_planar_B.nr = ((s_tmp - 1) << 2) +
                kinsim_2link_planar_B.j_e;
              kinsim_2link_planar_B.hoffset = (s_tmp << 2) +
                kinsim_2link_planar_B.j_e;
              kinsim_2link_planar_B.h12 = z[kinsim_2link_planar_B.nr] +
                z[kinsim_2link_planar_B.hoffset] * kinsim_2link_planar_B.tst;
              z[kinsim_2link_planar_B.nr] -= kinsim_2link_planar_B.h12 *
                kinsim_2link_planar_B.htmp2;
              z[kinsim_2link_planar_B.hoffset] -= kinsim_2link_planar_B.h12 *
                kinsim_2link_planar_B.ba;
            }
            break;
          }
        }

        kinsim_2link_planar_B.ix++;
      }
    }

    if (!goto150) {
      info = kinsim_2link_planar_B.i_a + 1;
      exitg1 = true;
    } else {
      if ((kinsim_2link_planar_B.i_a + 1 != kinsim_2link_planar_B.L) &&
          (kinsim_2link_planar_B.L == kinsim_2link_planar_B.i_a)) {
        kinsim_2link_planar_B.ix = (kinsim_2link_planar_B.i_a - 1) << 2;
        kinsim_2link_planar_B.k_j = kinsim_2link_planar_B.ix +
          kinsim_2link_planar_B.i_a;
        kinsim_2link_planar_B.m = kinsim_2link_planar_B.k_j - 1;
        kinsim_2link_planar_B.ba = h[kinsim_2link_planar_B.m];
        s_tmp = kinsim_2link_planar_B.i_a << 2;
        kinsim_2link_planar_B.nr = s_tmp + kinsim_2link_planar_B.i_a;
        kinsim_2link_planar_B.hoffset = kinsim_2link_planar_B.nr - 1;
        kinsim_2link_planar_B.htmp1 = h[kinsim_2link_planar_B.hoffset];
        kinsim_2link_planar_B.aa = h[kinsim_2link_planar_B.k_j];
        kinsim_2link_planar_B.h12 = h[kinsim_2link_planar_B.nr];
        kinsim_2link_planar_xdlanv2(&kinsim_2link_planar_B.ba,
          &kinsim_2link_planar_B.htmp1, &kinsim_2link_planar_B.aa,
          &kinsim_2link_planar_B.h12, &kinsim_2link_planar_B.h21s,
          &kinsim_2link_planar_B.unusedU1, &kinsim_2link_planar_B.unusedU2,
          &kinsim_2link_planar_B.unusedU3, &kinsim_2link_planar_B.htmp2,
          &kinsim_2link_planar_B.tst);
        h[kinsim_2link_planar_B.m] = kinsim_2link_planar_B.ba;
        h[kinsim_2link_planar_B.hoffset] = kinsim_2link_planar_B.htmp1;
        h[kinsim_2link_planar_B.k_j] = kinsim_2link_planar_B.aa;
        h[kinsim_2link_planar_B.nr] = kinsim_2link_planar_B.h12;
        if (4 > kinsim_2link_planar_B.i_a + 1) {
          kinsim_2link_planar_xrot(3 - kinsim_2link_planar_B.i_a, h,
            kinsim_2link_planar_B.i_a + ((kinsim_2link_planar_B.i_a + 1) << 2),
            (kinsim_2link_planar_B.i_a + ((kinsim_2link_planar_B.i_a + 1) << 2))
            + 1, kinsim_2link_planar_B.htmp2, kinsim_2link_planar_B.tst);
        }

        kinsim_2link_planar_xrot_g(kinsim_2link_planar_B.i_a - 1, h,
          ((kinsim_2link_planar_B.i_a - 1) << 2) + 1, (kinsim_2link_planar_B.i_a
          << 2) + 1, kinsim_2link_planar_B.htmp2, kinsim_2link_planar_B.tst);
        kinsim_2link_planar_B.ba = kinsim_2link_planar_B.htmp2 *
          z[kinsim_2link_planar_B.ix] + kinsim_2link_planar_B.tst * z[s_tmp];
        z[s_tmp] = kinsim_2link_planar_B.htmp2 * z[s_tmp] -
          kinsim_2link_planar_B.tst * z[kinsim_2link_planar_B.ix];
        z[kinsim_2link_planar_B.ix] = kinsim_2link_planar_B.ba;
        kinsim_2link_planar_B.i_a = s_tmp + 1;
        kinsim_2link_planar_B.ix++;
        kinsim_2link_planar_B.ba = kinsim_2link_planar_B.htmp2 *
          z[kinsim_2link_planar_B.ix] + kinsim_2link_planar_B.tst *
          z[kinsim_2link_planar_B.i_a];
        z[kinsim_2link_planar_B.i_a] = kinsim_2link_planar_B.htmp2 *
          z[kinsim_2link_planar_B.i_a] - kinsim_2link_planar_B.tst *
          z[kinsim_2link_planar_B.ix];
        z[kinsim_2link_planar_B.ix] = kinsim_2link_planar_B.ba;
        kinsim_2link_planar_B.i_a++;
        kinsim_2link_planar_B.ix++;
        kinsim_2link_planar_B.ba = kinsim_2link_planar_B.htmp2 *
          z[kinsim_2link_planar_B.ix] + kinsim_2link_planar_B.tst *
          z[kinsim_2link_planar_B.i_a];
        z[kinsim_2link_planar_B.i_a] = kinsim_2link_planar_B.htmp2 *
          z[kinsim_2link_planar_B.i_a] - kinsim_2link_planar_B.tst *
          z[kinsim_2link_planar_B.ix];
        z[kinsim_2link_planar_B.ix] = kinsim_2link_planar_B.ba;
        kinsim_2link_planar_B.i_a++;
        kinsim_2link_planar_B.ix++;
        kinsim_2link_planar_B.ba = kinsim_2link_planar_B.htmp2 *
          z[kinsim_2link_planar_B.ix] + kinsim_2link_planar_B.tst *
          z[kinsim_2link_planar_B.i_a];
        z[kinsim_2link_planar_B.i_a] = kinsim_2link_planar_B.htmp2 *
          z[kinsim_2link_planar_B.i_a] - kinsim_2link_planar_B.tst *
          z[kinsim_2link_planar_B.ix];
        z[kinsim_2link_planar_B.ix] = kinsim_2link_planar_B.ba;
      }

      kinsim_2link_planar_B.i_a = kinsim_2link_planar_B.L - 2;
    }
  }

  return info;
}

// Function for MATLAB Function: '<S5>/MATLAB Function'
static void kinsim_2link_planar_eig(const real_T A[16], creal_T V[16], creal_T
  D[4])
{
  boolean_T p;
  int32_T exitg1;
  boolean_T exitg2;
  if (kinsim_2link_plana_anyNonFinite(A)) {
    for (kinsim_2link_planar_B.b_j = 0; kinsim_2link_planar_B.b_j < 16;
         kinsim_2link_planar_B.b_j++) {
      V[kinsim_2link_planar_B.b_j].re = (rtNaN);
      V[kinsim_2link_planar_B.b_j].im = 0.0;
    }

    D[0].re = (rtNaN);
    D[0].im = 0.0;
    D[1].re = (rtNaN);
    D[1].im = 0.0;
    D[2].re = (rtNaN);
    D[2].im = 0.0;
    D[3].re = (rtNaN);
    D[3].im = 0.0;
  } else {
    p = true;
    kinsim_2link_planar_B.b_j = 0;
    exitg2 = false;
    while ((!exitg2) && (kinsim_2link_planar_B.b_j < 4)) {
      kinsim_2link_planar_B.i = 0;
      do {
        exitg1 = 0;
        if (kinsim_2link_planar_B.i <= kinsim_2link_planar_B.b_j) {
          if (!(A[(kinsim_2link_planar_B.b_j << 2) + kinsim_2link_planar_B.i] ==
                A[(kinsim_2link_planar_B.i << 2) + kinsim_2link_planar_B.b_j]))
          {
            p = false;
            exitg1 = 1;
          } else {
            kinsim_2link_planar_B.i++;
          }
        } else {
          kinsim_2link_planar_B.b_j++;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }

    if (p) {
      if (kinsim_2link_plana_anyNonFinite(A)) {
        for (kinsim_2link_planar_B.b_j = 0; kinsim_2link_planar_B.b_j < 16;
             kinsim_2link_planar_B.b_j++) {
          kinsim_2link_planar_B.b_V[kinsim_2link_planar_B.b_j] = (rtNaN);
        }

        kinsim_2link_planar_B.b_j = 2;
        while (kinsim_2link_planar_B.b_j < 5) {
          kinsim_2link_planar_B.b_V[kinsim_2link_planar_B.b_j - 1] = 0.0;
          kinsim_2link_planar_B.b_j++;
        }

        kinsim_2link_planar_B.b_j = 3;
        while (kinsim_2link_planar_B.b_j < 5) {
          kinsim_2link_planar_B.b_V[kinsim_2link_planar_B.b_j + 3] = 0.0;
          kinsim_2link_planar_B.b_j++;
        }

        kinsim_2link_planar_B.b_V[11] = 0.0;
        for (kinsim_2link_planar_B.b_j = 0; kinsim_2link_planar_B.b_j < 16;
             kinsim_2link_planar_B.b_j++) {
          kinsim_2link_planar_B.b_A[kinsim_2link_planar_B.b_j] = (rtNaN);
        }
      } else {
        memcpy(&kinsim_2link_planar_B.b_A[0], &A[0], sizeof(real_T) << 4U);
        kinsim_2link_planar_xgehrd(kinsim_2link_planar_B.b_A,
          kinsim_2link_planar_B.tau);
        memcpy(&kinsim_2link_planar_B.b_V[0], &kinsim_2link_planar_B.b_A[0],
               sizeof(real_T) << 4U);
        kinsim_2link_planar_B.b_j = 0;
        while (kinsim_2link_planar_B.b_j <= 2) {
          kinsim_2link_planar_B.b_V[kinsim_2link_planar_B.b_j + 12] = 0.0;
          kinsim_2link_planar_B.b_j++;
        }

        kinsim_2link_planar_B.b_j = 0;
        while (kinsim_2link_planar_B.b_j <= 1) {
          kinsim_2link_planar_B.b_V[kinsim_2link_planar_B.b_j + 8] = 0.0;
          kinsim_2link_planar_B.b_j++;
        }

        kinsim_2link_planar_B.b_j = 1;
        while (kinsim_2link_planar_B.b_j + 3 < 5) {
          kinsim_2link_planar_B.b_V[kinsim_2link_planar_B.b_j + 10] =
            kinsim_2link_planar_B.b_V[kinsim_2link_planar_B.b_j + 6];
          kinsim_2link_planar_B.b_j++;
        }

        kinsim_2link_planar_B.b_V[4] = 0.0;
        kinsim_2link_planar_B.b_j = 0;
        while (kinsim_2link_planar_B.b_j + 3 < 5) {
          kinsim_2link_planar_B.b_V[kinsim_2link_planar_B.b_j + 6] =
            kinsim_2link_planar_B.b_V[kinsim_2link_planar_B.b_j + 2];
          kinsim_2link_planar_B.b_j++;
        }

        kinsim_2link_planar_B.work[0] = 0.0;
        kinsim_2link_planar_B.b_V[1] = 0.0;
        kinsim_2link_planar_B.work[1] = 0.0;
        kinsim_2link_planar_B.b_V[2] = 0.0;
        kinsim_2link_planar_B.work[2] = 0.0;
        kinsim_2link_planar_B.b_V[3] = 0.0;
        kinsim_2link_planar_B.work[3] = 0.0;
        kinsim_2link_planar_B.b_V[0] = 1.0;
        kinsim_2link_planar_B.b_V[15] = 1.0 - kinsim_2link_planar_B.tau[2];
        kinsim_2link_planar_B.b_j = 0;
        while (kinsim_2link_planar_B.b_j <= 1) {
          kinsim_2link_planar_B.b_V[14 - kinsim_2link_planar_B.b_j] = 0.0;
          kinsim_2link_planar_B.b_j++;
        }

        kinsim_2link_planar_B.b_V[10] = 1.0;
        kinsim_2link_planar_xzlarf(2, 1, 11, kinsim_2link_planar_B.tau[1],
          kinsim_2link_planar_B.b_V, 15, kinsim_2link_planar_B.work);
        kinsim_2link_planar_B.b_j = 11;
        while (kinsim_2link_planar_B.b_j + 1 <= 12) {
          kinsim_2link_planar_B.b_V[kinsim_2link_planar_B.b_j] *=
            -kinsim_2link_planar_B.tau[1];
          kinsim_2link_planar_B.b_j++;
        }

        kinsim_2link_planar_B.b_V[10] = 1.0 - kinsim_2link_planar_B.tau[1];
        kinsim_2link_planar_B.b_V[9] = 0.0;
        kinsim_2link_planar_B.b_V[5] = 1.0;
        kinsim_2link_planar_xzlarf(3, 2, 6, kinsim_2link_planar_B.tau[0],
          kinsim_2link_planar_B.b_V, 10, kinsim_2link_planar_B.work);
        kinsim_2link_planar_B.b_j = 6;
        while (kinsim_2link_planar_B.b_j + 1 <= 8) {
          kinsim_2link_planar_B.b_V[kinsim_2link_planar_B.b_j] *=
            -kinsim_2link_planar_B.tau[0];
          kinsim_2link_planar_B.b_j++;
        }

        kinsim_2link_planar_B.b_V[5] = 1.0 - kinsim_2link_planar_B.tau[0];
        kinsim_2link_planar_eml_dlahqr(kinsim_2link_planar_B.b_A,
          kinsim_2link_planar_B.b_V);
      }

      for (kinsim_2link_planar_B.b_j = 0; kinsim_2link_planar_B.b_j < 16;
           kinsim_2link_planar_B.b_j++) {
        V[kinsim_2link_planar_B.b_j].re =
          kinsim_2link_planar_B.b_V[kinsim_2link_planar_B.b_j];
        V[kinsim_2link_planar_B.b_j].im = 0.0;
      }

      D[0].re = kinsim_2link_planar_B.b_A[0];
      D[0].im = 0.0;
      D[1].re = kinsim_2link_planar_B.b_A[5];
      D[1].im = 0.0;
      D[2].re = kinsim_2link_planar_B.b_A[10];
      D[2].im = 0.0;
      D[3].re = kinsim_2link_planar_B.b_A[15];
      D[3].im = 0.0;
    } else {
      for (kinsim_2link_planar_B.b_j = 0; kinsim_2link_planar_B.b_j < 16;
           kinsim_2link_planar_B.b_j++) {
        kinsim_2link_planar_B.At[kinsim_2link_planar_B.b_j].re =
          A[kinsim_2link_planar_B.b_j];
        kinsim_2link_planar_B.At[kinsim_2link_planar_B.b_j].im = 0.0;
      }

      kinsim_2link_planar_xzggev(kinsim_2link_planar_B.At,
        &kinsim_2link_planar_B.b_j, D, kinsim_2link_planar_B.beta1, V);
      kinsim_2link_planar_B.colnorm = 0.0;
      kinsim_2link_planar_B.scale = 3.3121686421112381E-170;
      kinsim_2link_planar_B.b_j = 0;
      while (kinsim_2link_planar_B.b_j + 1 <= 4) {
        kinsim_2link_planar_B.absxk = fabs(V[kinsim_2link_planar_B.b_j].re);
        if (kinsim_2link_planar_B.absxk > kinsim_2link_planar_B.scale) {
          kinsim_2link_planar_B.t = kinsim_2link_planar_B.scale /
            kinsim_2link_planar_B.absxk;
          kinsim_2link_planar_B.colnorm = kinsim_2link_planar_B.colnorm *
            kinsim_2link_planar_B.t * kinsim_2link_planar_B.t + 1.0;
          kinsim_2link_planar_B.scale = kinsim_2link_planar_B.absxk;
        } else {
          kinsim_2link_planar_B.t = kinsim_2link_planar_B.absxk /
            kinsim_2link_planar_B.scale;
          kinsim_2link_planar_B.colnorm += kinsim_2link_planar_B.t *
            kinsim_2link_planar_B.t;
        }

        kinsim_2link_planar_B.absxk = fabs(V[kinsim_2link_planar_B.b_j].im);
        if (kinsim_2link_planar_B.absxk > kinsim_2link_planar_B.scale) {
          kinsim_2link_planar_B.t = kinsim_2link_planar_B.scale /
            kinsim_2link_planar_B.absxk;
          kinsim_2link_planar_B.colnorm = kinsim_2link_planar_B.colnorm *
            kinsim_2link_planar_B.t * kinsim_2link_planar_B.t + 1.0;
          kinsim_2link_planar_B.scale = kinsim_2link_planar_B.absxk;
        } else {
          kinsim_2link_planar_B.t = kinsim_2link_planar_B.absxk /
            kinsim_2link_planar_B.scale;
          kinsim_2link_planar_B.colnorm += kinsim_2link_planar_B.t *
            kinsim_2link_planar_B.t;
        }

        kinsim_2link_planar_B.b_j++;
      }

      kinsim_2link_planar_B.colnorm = kinsim_2link_planar_B.scale * sqrt
        (kinsim_2link_planar_B.colnorm);
      kinsim_2link_planar_B.b_j = 0;
      while (kinsim_2link_planar_B.b_j + 1 <= 4) {
        if (V[kinsim_2link_planar_B.b_j].im == 0.0) {
          kinsim_2link_planar_B.scale = V[kinsim_2link_planar_B.b_j].re /
            kinsim_2link_planar_B.colnorm;
          kinsim_2link_planar_B.absxk = 0.0;
        } else if (V[kinsim_2link_planar_B.b_j].re == 0.0) {
          kinsim_2link_planar_B.scale = 0.0;
          kinsim_2link_planar_B.absxk = V[kinsim_2link_planar_B.b_j].im /
            kinsim_2link_planar_B.colnorm;
        } else {
          kinsim_2link_planar_B.scale = V[kinsim_2link_planar_B.b_j].re /
            kinsim_2link_planar_B.colnorm;
          kinsim_2link_planar_B.absxk = V[kinsim_2link_planar_B.b_j].im /
            kinsim_2link_planar_B.colnorm;
        }

        V[kinsim_2link_planar_B.b_j].re = kinsim_2link_planar_B.scale;
        V[kinsim_2link_planar_B.b_j].im = kinsim_2link_planar_B.absxk;
        kinsim_2link_planar_B.b_j++;
      }

      kinsim_2link_planar_B.colnorm = 0.0;
      kinsim_2link_planar_B.scale = 3.3121686421112381E-170;
      kinsim_2link_planar_B.b_j = 4;
      while (kinsim_2link_planar_B.b_j + 1 <= 8) {
        kinsim_2link_planar_B.absxk = fabs(V[kinsim_2link_planar_B.b_j].re);
        if (kinsim_2link_planar_B.absxk > kinsim_2link_planar_B.scale) {
          kinsim_2link_planar_B.t = kinsim_2link_planar_B.scale /
            kinsim_2link_planar_B.absxk;
          kinsim_2link_planar_B.colnorm = kinsim_2link_planar_B.colnorm *
            kinsim_2link_planar_B.t * kinsim_2link_planar_B.t + 1.0;
          kinsim_2link_planar_B.scale = kinsim_2link_planar_B.absxk;
        } else {
          kinsim_2link_planar_B.t = kinsim_2link_planar_B.absxk /
            kinsim_2link_planar_B.scale;
          kinsim_2link_planar_B.colnorm += kinsim_2link_planar_B.t *
            kinsim_2link_planar_B.t;
        }

        kinsim_2link_planar_B.absxk = fabs(V[kinsim_2link_planar_B.b_j].im);
        if (kinsim_2link_planar_B.absxk > kinsim_2link_planar_B.scale) {
          kinsim_2link_planar_B.t = kinsim_2link_planar_B.scale /
            kinsim_2link_planar_B.absxk;
          kinsim_2link_planar_B.colnorm = kinsim_2link_planar_B.colnorm *
            kinsim_2link_planar_B.t * kinsim_2link_planar_B.t + 1.0;
          kinsim_2link_planar_B.scale = kinsim_2link_planar_B.absxk;
        } else {
          kinsim_2link_planar_B.t = kinsim_2link_planar_B.absxk /
            kinsim_2link_planar_B.scale;
          kinsim_2link_planar_B.colnorm += kinsim_2link_planar_B.t *
            kinsim_2link_planar_B.t;
        }

        kinsim_2link_planar_B.b_j++;
      }

      kinsim_2link_planar_B.colnorm = kinsim_2link_planar_B.scale * sqrt
        (kinsim_2link_planar_B.colnorm);
      kinsim_2link_planar_B.b_j = 4;
      while (kinsim_2link_planar_B.b_j + 1 <= 8) {
        if (V[kinsim_2link_planar_B.b_j].im == 0.0) {
          kinsim_2link_planar_B.scale = V[kinsim_2link_planar_B.b_j].re /
            kinsim_2link_planar_B.colnorm;
          kinsim_2link_planar_B.absxk = 0.0;
        } else if (V[kinsim_2link_planar_B.b_j].re == 0.0) {
          kinsim_2link_planar_B.scale = 0.0;
          kinsim_2link_planar_B.absxk = V[kinsim_2link_planar_B.b_j].im /
            kinsim_2link_planar_B.colnorm;
        } else {
          kinsim_2link_planar_B.scale = V[kinsim_2link_planar_B.b_j].re /
            kinsim_2link_planar_B.colnorm;
          kinsim_2link_planar_B.absxk = V[kinsim_2link_planar_B.b_j].im /
            kinsim_2link_planar_B.colnorm;
        }

        V[kinsim_2link_planar_B.b_j].re = kinsim_2link_planar_B.scale;
        V[kinsim_2link_planar_B.b_j].im = kinsim_2link_planar_B.absxk;
        kinsim_2link_planar_B.b_j++;
      }

      kinsim_2link_planar_B.colnorm = 0.0;
      kinsim_2link_planar_B.scale = 3.3121686421112381E-170;
      kinsim_2link_planar_B.b_j = 8;
      while (kinsim_2link_planar_B.b_j + 1 <= 12) {
        kinsim_2link_planar_B.absxk = fabs(V[kinsim_2link_planar_B.b_j].re);
        if (kinsim_2link_planar_B.absxk > kinsim_2link_planar_B.scale) {
          kinsim_2link_planar_B.t = kinsim_2link_planar_B.scale /
            kinsim_2link_planar_B.absxk;
          kinsim_2link_planar_B.colnorm = kinsim_2link_planar_B.colnorm *
            kinsim_2link_planar_B.t * kinsim_2link_planar_B.t + 1.0;
          kinsim_2link_planar_B.scale = kinsim_2link_planar_B.absxk;
        } else {
          kinsim_2link_planar_B.t = kinsim_2link_planar_B.absxk /
            kinsim_2link_planar_B.scale;
          kinsim_2link_planar_B.colnorm += kinsim_2link_planar_B.t *
            kinsim_2link_planar_B.t;
        }

        kinsim_2link_planar_B.absxk = fabs(V[kinsim_2link_planar_B.b_j].im);
        if (kinsim_2link_planar_B.absxk > kinsim_2link_planar_B.scale) {
          kinsim_2link_planar_B.t = kinsim_2link_planar_B.scale /
            kinsim_2link_planar_B.absxk;
          kinsim_2link_planar_B.colnorm = kinsim_2link_planar_B.colnorm *
            kinsim_2link_planar_B.t * kinsim_2link_planar_B.t + 1.0;
          kinsim_2link_planar_B.scale = kinsim_2link_planar_B.absxk;
        } else {
          kinsim_2link_planar_B.t = kinsim_2link_planar_B.absxk /
            kinsim_2link_planar_B.scale;
          kinsim_2link_planar_B.colnorm += kinsim_2link_planar_B.t *
            kinsim_2link_planar_B.t;
        }

        kinsim_2link_planar_B.b_j++;
      }

      kinsim_2link_planar_B.colnorm = kinsim_2link_planar_B.scale * sqrt
        (kinsim_2link_planar_B.colnorm);
      kinsim_2link_planar_B.b_j = 8;
      while (kinsim_2link_planar_B.b_j + 1 <= 12) {
        if (V[kinsim_2link_planar_B.b_j].im == 0.0) {
          kinsim_2link_planar_B.scale = V[kinsim_2link_planar_B.b_j].re /
            kinsim_2link_planar_B.colnorm;
          kinsim_2link_planar_B.absxk = 0.0;
        } else if (V[kinsim_2link_planar_B.b_j].re == 0.0) {
          kinsim_2link_planar_B.scale = 0.0;
          kinsim_2link_planar_B.absxk = V[kinsim_2link_planar_B.b_j].im /
            kinsim_2link_planar_B.colnorm;
        } else {
          kinsim_2link_planar_B.scale = V[kinsim_2link_planar_B.b_j].re /
            kinsim_2link_planar_B.colnorm;
          kinsim_2link_planar_B.absxk = V[kinsim_2link_planar_B.b_j].im /
            kinsim_2link_planar_B.colnorm;
        }

        V[kinsim_2link_planar_B.b_j].re = kinsim_2link_planar_B.scale;
        V[kinsim_2link_planar_B.b_j].im = kinsim_2link_planar_B.absxk;
        kinsim_2link_planar_B.b_j++;
      }

      kinsim_2link_planar_B.colnorm = 0.0;
      kinsim_2link_planar_B.scale = 3.3121686421112381E-170;
      kinsim_2link_planar_B.b_j = 12;
      while (kinsim_2link_planar_B.b_j + 1 <= 16) {
        kinsim_2link_planar_B.absxk = fabs(V[kinsim_2link_planar_B.b_j].re);
        if (kinsim_2link_planar_B.absxk > kinsim_2link_planar_B.scale) {
          kinsim_2link_planar_B.t = kinsim_2link_planar_B.scale /
            kinsim_2link_planar_B.absxk;
          kinsim_2link_planar_B.colnorm = kinsim_2link_planar_B.colnorm *
            kinsim_2link_planar_B.t * kinsim_2link_planar_B.t + 1.0;
          kinsim_2link_planar_B.scale = kinsim_2link_planar_B.absxk;
        } else {
          kinsim_2link_planar_B.t = kinsim_2link_planar_B.absxk /
            kinsim_2link_planar_B.scale;
          kinsim_2link_planar_B.colnorm += kinsim_2link_planar_B.t *
            kinsim_2link_planar_B.t;
        }

        kinsim_2link_planar_B.absxk = fabs(V[kinsim_2link_planar_B.b_j].im);
        if (kinsim_2link_planar_B.absxk > kinsim_2link_planar_B.scale) {
          kinsim_2link_planar_B.t = kinsim_2link_planar_B.scale /
            kinsim_2link_planar_B.absxk;
          kinsim_2link_planar_B.colnorm = kinsim_2link_planar_B.colnorm *
            kinsim_2link_planar_B.t * kinsim_2link_planar_B.t + 1.0;
          kinsim_2link_planar_B.scale = kinsim_2link_planar_B.absxk;
        } else {
          kinsim_2link_planar_B.t = kinsim_2link_planar_B.absxk /
            kinsim_2link_planar_B.scale;
          kinsim_2link_planar_B.colnorm += kinsim_2link_planar_B.t *
            kinsim_2link_planar_B.t;
        }

        kinsim_2link_planar_B.b_j++;
      }

      kinsim_2link_planar_B.colnorm = kinsim_2link_planar_B.scale * sqrt
        (kinsim_2link_planar_B.colnorm);
      kinsim_2link_planar_B.b_j = 12;
      while (kinsim_2link_planar_B.b_j + 1 <= 16) {
        if (V[kinsim_2link_planar_B.b_j].im == 0.0) {
          kinsim_2link_planar_B.scale = V[kinsim_2link_planar_B.b_j].re /
            kinsim_2link_planar_B.colnorm;
          kinsim_2link_planar_B.absxk = 0.0;
        } else if (V[kinsim_2link_planar_B.b_j].re == 0.0) {
          kinsim_2link_planar_B.scale = 0.0;
          kinsim_2link_planar_B.absxk = V[kinsim_2link_planar_B.b_j].im /
            kinsim_2link_planar_B.colnorm;
        } else {
          kinsim_2link_planar_B.scale = V[kinsim_2link_planar_B.b_j].re /
            kinsim_2link_planar_B.colnorm;
          kinsim_2link_planar_B.absxk = V[kinsim_2link_planar_B.b_j].im /
            kinsim_2link_planar_B.colnorm;
        }

        V[kinsim_2link_planar_B.b_j].re = kinsim_2link_planar_B.scale;
        V[kinsim_2link_planar_B.b_j].im = kinsim_2link_planar_B.absxk;
        kinsim_2link_planar_B.b_j++;
      }

      if (kinsim_2link_planar_B.beta1[0].im == 0.0) {
        if (D[0].im == 0.0) {
          kinsim_2link_planar_B.scale = D[0].re / kinsim_2link_planar_B.beta1[0]
            .re;
          kinsim_2link_planar_B.absxk = 0.0;
        } else if (D[0].re == 0.0) {
          kinsim_2link_planar_B.scale = 0.0;
          kinsim_2link_planar_B.absxk = D[0].im / kinsim_2link_planar_B.beta1[0]
            .re;
        } else {
          kinsim_2link_planar_B.scale = D[0].re / kinsim_2link_planar_B.beta1[0]
            .re;
          kinsim_2link_planar_B.absxk = D[0].im / kinsim_2link_planar_B.beta1[0]
            .re;
        }
      } else if (kinsim_2link_planar_B.beta1[0].re == 0.0) {
        if (D[0].re == 0.0) {
          kinsim_2link_planar_B.scale = D[0].im / kinsim_2link_planar_B.beta1[0]
            .im;
          kinsim_2link_planar_B.absxk = 0.0;
        } else if (D[0].im == 0.0) {
          kinsim_2link_planar_B.scale = 0.0;
          kinsim_2link_planar_B.absxk = -(D[0].re / kinsim_2link_planar_B.beta1
            [0].im);
        } else {
          kinsim_2link_planar_B.scale = D[0].im / kinsim_2link_planar_B.beta1[0]
            .im;
          kinsim_2link_planar_B.absxk = -(D[0].re / kinsim_2link_planar_B.beta1
            [0].im);
        }
      } else {
        kinsim_2link_planar_B.colnorm = fabs(kinsim_2link_planar_B.beta1[0].re);
        kinsim_2link_planar_B.scale = fabs(kinsim_2link_planar_B.beta1[0].im);
        if (kinsim_2link_planar_B.colnorm > kinsim_2link_planar_B.scale) {
          kinsim_2link_planar_B.colnorm = kinsim_2link_planar_B.beta1[0].im /
            kinsim_2link_planar_B.beta1[0].re;
          kinsim_2link_planar_B.absxk = kinsim_2link_planar_B.colnorm *
            kinsim_2link_planar_B.beta1[0].im + kinsim_2link_planar_B.beta1[0].
            re;
          kinsim_2link_planar_B.scale = (kinsim_2link_planar_B.colnorm * D[0].im
            + D[0].re) / kinsim_2link_planar_B.absxk;
          kinsim_2link_planar_B.absxk = (D[0].im - kinsim_2link_planar_B.colnorm
            * D[0].re) / kinsim_2link_planar_B.absxk;
        } else if (kinsim_2link_planar_B.scale == kinsim_2link_planar_B.colnorm)
        {
          kinsim_2link_planar_B.absxk = kinsim_2link_planar_B.beta1[0].re > 0.0 ?
            0.5 : -0.5;
          kinsim_2link_planar_B.t = kinsim_2link_planar_B.beta1[0].im > 0.0 ?
            0.5 : -0.5;
          kinsim_2link_planar_B.scale = (D[0].re * kinsim_2link_planar_B.absxk +
            D[0].im * kinsim_2link_planar_B.t) / kinsim_2link_planar_B.colnorm;
          kinsim_2link_planar_B.absxk = (D[0].im * kinsim_2link_planar_B.absxk -
            D[0].re * kinsim_2link_planar_B.t) / kinsim_2link_planar_B.colnorm;
        } else {
          kinsim_2link_planar_B.colnorm = kinsim_2link_planar_B.beta1[0].re /
            kinsim_2link_planar_B.beta1[0].im;
          kinsim_2link_planar_B.absxk = kinsim_2link_planar_B.colnorm *
            kinsim_2link_planar_B.beta1[0].re + kinsim_2link_planar_B.beta1[0].
            im;
          kinsim_2link_planar_B.scale = (kinsim_2link_planar_B.colnorm * D[0].re
            + D[0].im) / kinsim_2link_planar_B.absxk;
          kinsim_2link_planar_B.absxk = (kinsim_2link_planar_B.colnorm * D[0].im
            - D[0].re) / kinsim_2link_planar_B.absxk;
        }
      }

      D[0].re = kinsim_2link_planar_B.scale;
      D[0].im = kinsim_2link_planar_B.absxk;
      if (kinsim_2link_planar_B.beta1[1].im == 0.0) {
        if (D[1].im == 0.0) {
          kinsim_2link_planar_B.scale = D[1].re / kinsim_2link_planar_B.beta1[1]
            .re;
          kinsim_2link_planar_B.absxk = 0.0;
        } else if (D[1].re == 0.0) {
          kinsim_2link_planar_B.scale = 0.0;
          kinsim_2link_planar_B.absxk = D[1].im / kinsim_2link_planar_B.beta1[1]
            .re;
        } else {
          kinsim_2link_planar_B.scale = D[1].re / kinsim_2link_planar_B.beta1[1]
            .re;
          kinsim_2link_planar_B.absxk = D[1].im / kinsim_2link_planar_B.beta1[1]
            .re;
        }
      } else if (kinsim_2link_planar_B.beta1[1].re == 0.0) {
        if (D[1].re == 0.0) {
          kinsim_2link_planar_B.scale = D[1].im / kinsim_2link_planar_B.beta1[1]
            .im;
          kinsim_2link_planar_B.absxk = 0.0;
        } else if (D[1].im == 0.0) {
          kinsim_2link_planar_B.scale = 0.0;
          kinsim_2link_planar_B.absxk = -(D[1].re / kinsim_2link_planar_B.beta1
            [1].im);
        } else {
          kinsim_2link_planar_B.scale = D[1].im / kinsim_2link_planar_B.beta1[1]
            .im;
          kinsim_2link_planar_B.absxk = -(D[1].re / kinsim_2link_planar_B.beta1
            [1].im);
        }
      } else {
        kinsim_2link_planar_B.colnorm = fabs(kinsim_2link_planar_B.beta1[1].re);
        kinsim_2link_planar_B.scale = fabs(kinsim_2link_planar_B.beta1[1].im);
        if (kinsim_2link_planar_B.colnorm > kinsim_2link_planar_B.scale) {
          kinsim_2link_planar_B.colnorm = kinsim_2link_planar_B.beta1[1].im /
            kinsim_2link_planar_B.beta1[1].re;
          kinsim_2link_planar_B.absxk = kinsim_2link_planar_B.colnorm *
            kinsim_2link_planar_B.beta1[1].im + kinsim_2link_planar_B.beta1[1].
            re;
          kinsim_2link_planar_B.scale = (kinsim_2link_planar_B.colnorm * D[1].im
            + D[1].re) / kinsim_2link_planar_B.absxk;
          kinsim_2link_planar_B.absxk = (D[1].im - kinsim_2link_planar_B.colnorm
            * D[1].re) / kinsim_2link_planar_B.absxk;
        } else if (kinsim_2link_planar_B.scale == kinsim_2link_planar_B.colnorm)
        {
          kinsim_2link_planar_B.absxk = kinsim_2link_planar_B.beta1[1].re > 0.0 ?
            0.5 : -0.5;
          kinsim_2link_planar_B.t = kinsim_2link_planar_B.beta1[1].im > 0.0 ?
            0.5 : -0.5;
          kinsim_2link_planar_B.scale = (D[1].re * kinsim_2link_planar_B.absxk +
            D[1].im * kinsim_2link_planar_B.t) / kinsim_2link_planar_B.colnorm;
          kinsim_2link_planar_B.absxk = (D[1].im * kinsim_2link_planar_B.absxk -
            D[1].re * kinsim_2link_planar_B.t) / kinsim_2link_planar_B.colnorm;
        } else {
          kinsim_2link_planar_B.colnorm = kinsim_2link_planar_B.beta1[1].re /
            kinsim_2link_planar_B.beta1[1].im;
          kinsim_2link_planar_B.absxk = kinsim_2link_planar_B.colnorm *
            kinsim_2link_planar_B.beta1[1].re + kinsim_2link_planar_B.beta1[1].
            im;
          kinsim_2link_planar_B.scale = (kinsim_2link_planar_B.colnorm * D[1].re
            + D[1].im) / kinsim_2link_planar_B.absxk;
          kinsim_2link_planar_B.absxk = (kinsim_2link_planar_B.colnorm * D[1].im
            - D[1].re) / kinsim_2link_planar_B.absxk;
        }
      }

      D[1].re = kinsim_2link_planar_B.scale;
      D[1].im = kinsim_2link_planar_B.absxk;
      if (kinsim_2link_planar_B.beta1[2].im == 0.0) {
        if (D[2].im == 0.0) {
          kinsim_2link_planar_B.scale = D[2].re / kinsim_2link_planar_B.beta1[2]
            .re;
          kinsim_2link_planar_B.absxk = 0.0;
        } else if (D[2].re == 0.0) {
          kinsim_2link_planar_B.scale = 0.0;
          kinsim_2link_planar_B.absxk = D[2].im / kinsim_2link_planar_B.beta1[2]
            .re;
        } else {
          kinsim_2link_planar_B.scale = D[2].re / kinsim_2link_planar_B.beta1[2]
            .re;
          kinsim_2link_planar_B.absxk = D[2].im / kinsim_2link_planar_B.beta1[2]
            .re;
        }
      } else if (kinsim_2link_planar_B.beta1[2].re == 0.0) {
        if (D[2].re == 0.0) {
          kinsim_2link_planar_B.scale = D[2].im / kinsim_2link_planar_B.beta1[2]
            .im;
          kinsim_2link_planar_B.absxk = 0.0;
        } else if (D[2].im == 0.0) {
          kinsim_2link_planar_B.scale = 0.0;
          kinsim_2link_planar_B.absxk = -(D[2].re / kinsim_2link_planar_B.beta1
            [2].im);
        } else {
          kinsim_2link_planar_B.scale = D[2].im / kinsim_2link_planar_B.beta1[2]
            .im;
          kinsim_2link_planar_B.absxk = -(D[2].re / kinsim_2link_planar_B.beta1
            [2].im);
        }
      } else {
        kinsim_2link_planar_B.colnorm = fabs(kinsim_2link_planar_B.beta1[2].re);
        kinsim_2link_planar_B.scale = fabs(kinsim_2link_planar_B.beta1[2].im);
        if (kinsim_2link_planar_B.colnorm > kinsim_2link_planar_B.scale) {
          kinsim_2link_planar_B.colnorm = kinsim_2link_planar_B.beta1[2].im /
            kinsim_2link_planar_B.beta1[2].re;
          kinsim_2link_planar_B.absxk = kinsim_2link_planar_B.colnorm *
            kinsim_2link_planar_B.beta1[2].im + kinsim_2link_planar_B.beta1[2].
            re;
          kinsim_2link_planar_B.scale = (kinsim_2link_planar_B.colnorm * D[2].im
            + D[2].re) / kinsim_2link_planar_B.absxk;
          kinsim_2link_planar_B.absxk = (D[2].im - kinsim_2link_planar_B.colnorm
            * D[2].re) / kinsim_2link_planar_B.absxk;
        } else if (kinsim_2link_planar_B.scale == kinsim_2link_planar_B.colnorm)
        {
          kinsim_2link_planar_B.absxk = kinsim_2link_planar_B.beta1[2].re > 0.0 ?
            0.5 : -0.5;
          kinsim_2link_planar_B.t = kinsim_2link_planar_B.beta1[2].im > 0.0 ?
            0.5 : -0.5;
          kinsim_2link_planar_B.scale = (D[2].re * kinsim_2link_planar_B.absxk +
            D[2].im * kinsim_2link_planar_B.t) / kinsim_2link_planar_B.colnorm;
          kinsim_2link_planar_B.absxk = (D[2].im * kinsim_2link_planar_B.absxk -
            D[2].re * kinsim_2link_planar_B.t) / kinsim_2link_planar_B.colnorm;
        } else {
          kinsim_2link_planar_B.colnorm = kinsim_2link_planar_B.beta1[2].re /
            kinsim_2link_planar_B.beta1[2].im;
          kinsim_2link_planar_B.absxk = kinsim_2link_planar_B.colnorm *
            kinsim_2link_planar_B.beta1[2].re + kinsim_2link_planar_B.beta1[2].
            im;
          kinsim_2link_planar_B.scale = (kinsim_2link_planar_B.colnorm * D[2].re
            + D[2].im) / kinsim_2link_planar_B.absxk;
          kinsim_2link_planar_B.absxk = (kinsim_2link_planar_B.colnorm * D[2].im
            - D[2].re) / kinsim_2link_planar_B.absxk;
        }
      }

      D[2].re = kinsim_2link_planar_B.scale;
      D[2].im = kinsim_2link_planar_B.absxk;
      if (kinsim_2link_planar_B.beta1[3].im == 0.0) {
        if (D[3].im == 0.0) {
          kinsim_2link_planar_B.scale = D[3].re / kinsim_2link_planar_B.beta1[3]
            .re;
          kinsim_2link_planar_B.absxk = 0.0;
        } else if (D[3].re == 0.0) {
          kinsim_2link_planar_B.scale = 0.0;
          kinsim_2link_planar_B.absxk = D[3].im / kinsim_2link_planar_B.beta1[3]
            .re;
        } else {
          kinsim_2link_planar_B.scale = D[3].re / kinsim_2link_planar_B.beta1[3]
            .re;
          kinsim_2link_planar_B.absxk = D[3].im / kinsim_2link_planar_B.beta1[3]
            .re;
        }
      } else if (kinsim_2link_planar_B.beta1[3].re == 0.0) {
        if (D[3].re == 0.0) {
          kinsim_2link_planar_B.scale = D[3].im / kinsim_2link_planar_B.beta1[3]
            .im;
          kinsim_2link_planar_B.absxk = 0.0;
        } else if (D[3].im == 0.0) {
          kinsim_2link_planar_B.scale = 0.0;
          kinsim_2link_planar_B.absxk = -(D[3].re / kinsim_2link_planar_B.beta1
            [3].im);
        } else {
          kinsim_2link_planar_B.scale = D[3].im / kinsim_2link_planar_B.beta1[3]
            .im;
          kinsim_2link_planar_B.absxk = -(D[3].re / kinsim_2link_planar_B.beta1
            [3].im);
        }
      } else {
        kinsim_2link_planar_B.colnorm = fabs(kinsim_2link_planar_B.beta1[3].re);
        kinsim_2link_planar_B.scale = fabs(kinsim_2link_planar_B.beta1[3].im);
        if (kinsim_2link_planar_B.colnorm > kinsim_2link_planar_B.scale) {
          kinsim_2link_planar_B.colnorm = kinsim_2link_planar_B.beta1[3].im /
            kinsim_2link_planar_B.beta1[3].re;
          kinsim_2link_planar_B.absxk = kinsim_2link_planar_B.colnorm *
            kinsim_2link_planar_B.beta1[3].im + kinsim_2link_planar_B.beta1[3].
            re;
          kinsim_2link_planar_B.scale = (kinsim_2link_planar_B.colnorm * D[3].im
            + D[3].re) / kinsim_2link_planar_B.absxk;
          kinsim_2link_planar_B.absxk = (D[3].im - kinsim_2link_planar_B.colnorm
            * D[3].re) / kinsim_2link_planar_B.absxk;
        } else if (kinsim_2link_planar_B.scale == kinsim_2link_planar_B.colnorm)
        {
          kinsim_2link_planar_B.absxk = kinsim_2link_planar_B.beta1[3].re > 0.0 ?
            0.5 : -0.5;
          kinsim_2link_planar_B.t = kinsim_2link_planar_B.beta1[3].im > 0.0 ?
            0.5 : -0.5;
          kinsim_2link_planar_B.scale = (D[3].re * kinsim_2link_planar_B.absxk +
            D[3].im * kinsim_2link_planar_B.t) / kinsim_2link_planar_B.colnorm;
          kinsim_2link_planar_B.absxk = (D[3].im * kinsim_2link_planar_B.absxk -
            D[3].re * kinsim_2link_planar_B.t) / kinsim_2link_planar_B.colnorm;
        } else {
          kinsim_2link_planar_B.colnorm = kinsim_2link_planar_B.beta1[3].re /
            kinsim_2link_planar_B.beta1[3].im;
          kinsim_2link_planar_B.absxk = kinsim_2link_planar_B.colnorm *
            kinsim_2link_planar_B.beta1[3].re + kinsim_2link_planar_B.beta1[3].
            im;
          kinsim_2link_planar_B.scale = (kinsim_2link_planar_B.colnorm * D[3].re
            + D[3].im) / kinsim_2link_planar_B.absxk;
          kinsim_2link_planar_B.absxk = (kinsim_2link_planar_B.colnorm * D[3].im
            - D[3].re) / kinsim_2link_planar_B.absxk;
        }
      }

      D[3].re = kinsim_2link_planar_B.scale;
      D[3].im = kinsim_2link_planar_B.absxk;
    }
  }
}

static void matlabCodegenHandle_matla_li5a1(ros_slros_internal_block_GetP_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_kinsim_2link_T
  *pStruct)
{
  kinsim_2link_pla_emxFree_char_T(&pStruct->Type);
  kinsim_2link_pla_emxFree_real_T(&pStruct->MotionSubspace);
}

static void emxFreeStruct_m_robotics_manip_(m_robotics_manip_internal_Rig_T
  *pStruct)
{
  kinsim_2link_pla_emxFree_char_T(&pStruct->NameInternal);
  emxFreeStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static void emxFreeStruct_n_robotics_manip_(n_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_m_robotics_manip_(&pStruct->Base);
}

static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxFreeStruct_n_robotics_manip_(&pStruct->TreeInternal);
}

static void emxFreeStruct_l_robotics_manip_(l_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static void emxFreeStruct_c_rigidBodyJoint1(c_rigidBodyJoint_kinsim_2li_l_T
  *pStruct)
{
  kinsim_2link_pla_emxFree_char_T(&pStruct->Type);
}

static void emxFreeStruct_m_robotics_mani_l(m_robotics_manip_internal_R_l_T
  *pStruct)
{
  kinsim_2link_pla_emxFree_char_T(&pStruct->NameInternal);
  emxFreeStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static void emxFreeStruct_n_robotics_mani_l(n_robotics_manip_internal_R_l_T
  *pStruct)
{
  emxFreeStruct_m_robotics_mani_l(&pStruct->Base);
}

static void emxFreeStruct_robotics_slmani_l(robotics_slmanip_internal_b_l_T
  *pStruct)
{
  emxFreeStruct_n_robotics_mani_l(&pStruct->TreeInternal);
}

static void emxFreeStruct_l_robotics_mani_l(l_robotics_manip_internal_R_l_T
  *pStruct)
{
  emxFreeStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static void matlabCodegenHandle_matlab_li5a(ros_slros_internal_block_Subs_T *obj)
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

static void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_kinsim_2link_T
  *pStruct)
{
  kinsim_2link_pla_emxInit_char_T(&pStruct->Type, 2);
  kinsim_2link_pla_emxInit_real_T(&pStruct->MotionSubspace, 2);
}

static void emxInitStruct_m_robotics_manip_(m_robotics_manip_internal_Rig_T
  *pStruct)
{
  kinsim_2link_pla_emxInit_char_T(&pStruct->NameInternal, 2);
  emxInitStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static void emxInitStruct_n_robotics_manip_(n_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_m_robotics_manip_(&pStruct->Base);
}

static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxInitStruct_n_robotics_manip_(&pStruct->TreeInternal);
}

static void emxInitStruct_l_robotics_manip_(l_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static l_robotics_manip_internal_Rig_T *kinsim_2lin_RigidBody_RigidBody
  (l_robotics_manip_internal_Rig_T *obj)
{
  l_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_kinsim_2link__T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[6] = { 'l', 'i', 'n', 'k', '_', '1' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { -0.99999999997301514, 7.3464102066435871E-6,
    -6.9389E-16, 0.0, 2.6984177572320606E-11, 3.6732051032474579E-6,
    0.99999999999325373, 0.0, 7.3464102065940289E-6, 0.99999999996626887,
    -3.6732051033465739E-6, 0.0, 0.057188, 0.0059831, 0.13343, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->NameInternal[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 0.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  kinsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  kinsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  kinsim_2link_pla_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  kinsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  kinsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 1.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static l_robotics_manip_internal_Rig_T *kinsim_2l_RigidBody_RigidBody_l
  (l_robotics_manip_internal_Rig_T *obj)
{
  l_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_kinsim_2link__T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[6] = { 'l', 'i', 'n', 'k', '_', '2' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 0.88847119465357549, -0.10400474353252914,
    0.44699211356978236, 0.0, -0.29079372810919513, 0.62592701137950757,
    0.7236396783744472, 0.0, -0.35504639691623957, -0.77291551268444,
    0.52587419245342837, 0.0, 0.0, 0.18967, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { -0.88847134048214615, 0.29080043874549294,
    0.3550405356678123, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->NameInternal[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 1.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  kinsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  kinsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  kinsim_2link_pla_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  kinsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  kinsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = -0.88847134048214615;
  obj->JointInternal.JointAxisInternal[1] = 0.29080043874549294;
  obj->JointInternal.JointAxisInternal[2] = 0.3550405356678123;
  return b_obj;
}

static l_robotics_manip_internal_Rig_T *kinsim_2_RigidBody_RigidBody_li
  (l_robotics_manip_internal_Rig_T *obj)
{
  l_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_kinsim_2link__T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[6] = { 'l', 'i', 'n', 'k', '_', '3' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { -0.88847685238438157, 0.29078066152542581,
    0.35504294058603364, 0.0, 0.10401076381482063, -0.62592577039760033,
    0.77291570754049777, 0.0, 0.44697946685256096, 0.72364600243144184,
    0.5258762396012906, 0.0, -0.024558, 0.12737, -0.16578, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->NameInternal[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 2.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  kinsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  kinsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  kinsim_2link_pla_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  kinsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  kinsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 1.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static l_robotics_manip_internal_Rig_T *kinsim__RigidBody_RigidBody_li5
  (l_robotics_manip_internal_Rig_T *obj)
{
  l_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_kinsim_2link__T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[6] = { 'l', 'i', 'n', 'k', '_', '4' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0,
    -3.6732051033465739E-6, -0.99999999999325373, 0.0, -0.0, 0.99999999999325373,
    -3.6732051033465739E-6, 0.0, 0.0088, -0.1588, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->NameInternal[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 3.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  kinsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  kinsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  kinsim_2link_pla_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  kinsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  kinsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = -1.0;
  return b_obj;
}

static l_robotics_manip_internal_Rig_T *kinsim_RigidBody_RigidBody_li5a
  (l_robotics_manip_internal_Rig_T *obj)
{
  l_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_kinsim_2link__T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[6] = { 'l', 'i', 'n', 'k', '_', '5' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { -0.99999999997301514, -7.3464102066435871E-6,
    -1.1102E-14, 0.0, -7.3464102064452637E-6, 0.99999999994603028,
    -7.3464102066435871E-6, 0.0, 5.3980844924276475E-11, -7.3464102064452637E-6,
    -0.99999999997301514, 0.0, 0.0, 0.0, -0.1053, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->NameInternal[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 4.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  kinsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  kinsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  kinsim_2link_pla_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  kinsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  kinsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = -1.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static l_robotics_manip_internal_Rig_T *kinsi_RigidBody_RigidBody_li5a1
  (l_robotics_manip_internal_Rig_T *obj)
{
  l_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_kinsim_2link__T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[6] = { 'l', 'i', 'n', 'k', '_', '6' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -1.249E-14, 0.0,
    -1.2489999999915739E-14, -3.6732051033465739E-6, -0.99999999999325373, 0.0,
    -4.587833174079871E-20, 0.99999999999325373, -3.6732051033465739E-6, 0.0,
    -0.0039, 0.0, 0.1636, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->NameInternal[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 5.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  kinsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  kinsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  kinsim_2link_pla_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  kinsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  kinsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = -1.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static m_robotics_manip_internal_Rig_T *kins_RigidBody_RigidBody_li5a1l
  (m_robotics_manip_internal_Rig_T *obj)
{
  m_robotics_manip_internal_Rig_T *b_obj;
  emxArray_char_T_kinsim_2link__T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[9] = { 'b', 'a', 's', 'e', '_', 'l', 'i', 'n', 'k' };

  static const char_T tmp_1[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_2[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_3[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 9;
  kinsim_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  kinsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  kinsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_2[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      kinsim_2link_planar_B.b_n[b_kstr] = tmp_3[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] !=
              kinsim_2link_planar_B.b_n[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  kinsim_2link_pla_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      kinsim_2link_planar_B.msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      kinsim_2link_planar_B.msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      kinsim_2link_planar_B.msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  kinsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] =
      kinsim_2link_planar_B.msubspace_data[b_kstr];
  }

  return b_obj;
}

static n_robotics_manip_internal_Rig_T *kin_RigidBodyTree_RigidBodyTree
  (n_robotics_manip_internal_Rig_T *obj, l_robotics_manip_internal_Rig_T *iobj_0,
   l_robotics_manip_internal_Rig_T *iobj_1, l_robotics_manip_internal_Rig_T
   *iobj_2, l_robotics_manip_internal_Rig_T *iobj_3,
   l_robotics_manip_internal_Rig_T *iobj_4, l_robotics_manip_internal_Rig_T
   *iobj_5)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int32_T i;
  static const int8_T tmp[12] = { 1, 2, 3, 4, 5, 6, 1, 2, 3, 4, 5, 6 };

  b_obj = obj;
  obj->Bodies[0] = kinsim_2lin_RigidBody_RigidBody(iobj_0);
  obj->Bodies[0]->Index = 1.0;
  obj->Bodies[1] = kinsim_2l_RigidBody_RigidBody_l(iobj_5);
  obj->Bodies[1]->Index = 2.0;
  obj->Bodies[2] = kinsim_2_RigidBody_RigidBody_li(iobj_1);
  obj->Bodies[2]->Index = 3.0;
  obj->Bodies[3] = kinsim__RigidBody_RigidBody_li5(iobj_2);
  obj->Bodies[3]->Index = 4.0;
  obj->Bodies[4] = kinsim_RigidBody_RigidBody_li5a(iobj_3);
  obj->Bodies[4]->Index = 5.0;
  obj->Bodies[5] = kinsi_RigidBody_RigidBody_li5a1(iobj_4);
  obj->Bodies[5]->Index = 6.0;
  obj->NumBodies = 6.0;
  obj->PositionNumber = 6.0;
  obj->VelocityNumber = 6.0;
  for (i = 0; i < 12; i++) {
    obj->PositionDoFMap[i] = tmp[i];
  }

  kins_RigidBody_RigidBody_li5a1l(&obj->Base);
  return b_obj;
}

static void emxInitStruct_c_rigidBodyJoint1(c_rigidBodyJoint_kinsim_2li_l_T
  *pStruct)
{
  kinsim_2link_pla_emxInit_char_T(&pStruct->Type, 2);
}

static void emxInitStruct_m_robotics_mani_l(m_robotics_manip_internal_R_l_T
  *pStruct)
{
  kinsim_2link_pla_emxInit_char_T(&pStruct->NameInternal, 2);
  emxInitStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static void emxInitStruct_n_robotics_mani_l(n_robotics_manip_internal_R_l_T
  *pStruct)
{
  emxInitStruct_m_robotics_mani_l(&pStruct->Base);
}

static void emxInitStruct_robotics_slmani_l(robotics_slmanip_internal_b_l_T
  *pStruct)
{
  emxInitStruct_n_robotics_mani_l(&pStruct->TreeInternal);
}

static void emxInitStruct_l_robotics_mani_l(l_robotics_manip_internal_R_l_T
  *pStruct)
{
  emxInitStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static l_robotics_manip_internal_R_l_T *kin_RigidBody_RigidBody_li5a1ls
  (l_robotics_manip_internal_R_l_T *obj)
{
  l_robotics_manip_internal_R_l_T *b_obj;
  emxArray_char_T_kinsim_2link__T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  static const char_T tmp[6] = { 'l', 'i', 'n', 'k', '_', '1' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_2[16] = { -0.99999999997301514, 7.3464102066435871E-6,
    -6.9389E-16, 0.0, 2.6984177572320606E-11, 3.6732051032474579E-6,
    0.99999999999325373, 0.0, 7.3464102065940289E-6, 0.99999999996626887,
    -3.6732051033465739E-6, 0.0, 0.057188, 0.0059831, 0.13343, 1.0 };

  static const real_T tmp_3[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->NameInternal[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 0.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  kinsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  kinsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_0[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_1[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  kinsim_2link_pla_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_2[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 1.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static l_robotics_manip_internal_R_l_T *ki_RigidBody_RigidBody_li5a1lsh
  (l_robotics_manip_internal_R_l_T *obj)
{
  l_robotics_manip_internal_R_l_T *b_obj;
  emxArray_char_T_kinsim_2link__T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  static const char_T tmp[6] = { 'l', 'i', 'n', 'k', '_', '2' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_2[16] = { 0.88847119465357549, -0.10400474353252914,
    0.44699211356978236, 0.0, -0.29079372810919513, 0.62592701137950757,
    0.7236396783744472, 0.0, -0.35504639691623957, -0.77291551268444,
    0.52587419245342837, 0.0, 0.0, 0.18967, 0.0, 1.0 };

  static const real_T tmp_3[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->NameInternal[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 1.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  kinsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  kinsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_0[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_1[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  kinsim_2link_pla_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_2[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = -0.88847134048214615;
  obj->JointInternal.JointAxisInternal[1] = 0.29080043874549294;
  obj->JointInternal.JointAxisInternal[2] = 0.3550405356678123;
  return b_obj;
}

static l_robotics_manip_internal_R_l_T *k_RigidBody_RigidBody_li5a1lshy
  (l_robotics_manip_internal_R_l_T *obj)
{
  l_robotics_manip_internal_R_l_T *b_obj;
  emxArray_char_T_kinsim_2link__T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  static const char_T tmp[6] = { 'l', 'i', 'n', 'k', '_', '3' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_2[16] = { -0.88847685238438157, 0.29078066152542581,
    0.35504294058603364, 0.0, 0.10401076381482063, -0.62592577039760033,
    0.77291570754049777, 0.0, 0.44697946685256096, 0.72364600243144184,
    0.5258762396012906, 0.0, -0.024558, 0.12737, -0.16578, 1.0 };

  static const real_T tmp_3[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->NameInternal[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 2.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  kinsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  kinsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_0[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_1[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  kinsim_2link_pla_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_2[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 1.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static n_robotics_manip_internal_R_l_T *k_RigidBodyTree_RigidBodyTree_l
  (n_robotics_manip_internal_R_l_T *obj, l_robotics_manip_internal_R_l_T *iobj_0,
   l_robotics_manip_internal_R_l_T *iobj_1, l_robotics_manip_internal_R_l_T
   *iobj_2, l_robotics_manip_internal_R_l_T *iobj_3,
   l_robotics_manip_internal_R_l_T *iobj_4, l_robotics_manip_internal_R_l_T
   *iobj_5)
{
  n_robotics_manip_internal_R_l_T *b_obj;
  m_robotics_manip_internal_R_l_T *obj_0;
  emxArray_char_T_kinsim_2link__T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  static const char_T tmp[6] = { 'l', 'i', 'n', 'k', '_', '4' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_2[16] = { 1.0, 0.0, -0.0, 0.0, 0.0,
    -3.6732051033465739E-6, -0.99999999999325373, 0.0, -0.0, 0.99999999999325373,
    -3.6732051033465739E-6, 0.0, 0.0088, -0.1588, 0.0, 1.0 };

  static const real_T tmp_3[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const char_T tmp_4[6] = { 'l', 'i', 'n', 'k', '_', '5' };

  static const real_T tmp_5[16] = { -0.99999999997301514, -7.3464102066435871E-6,
    -1.1102E-14, 0.0, -7.3464102064452637E-6, 0.99999999994603028,
    -7.3464102066435871E-6, 0.0, 5.3980844924276475E-11, -7.3464102064452637E-6,
    -0.99999999997301514, 0.0, 0.0, 0.0, -0.1053, 1.0 };

  static const char_T tmp_6[6] = { 'l', 'i', 'n', 'k', '_', '6' };

  static const real_T tmp_7[16] = { 1.0, 0.0, -1.249E-14, 0.0,
    -1.2489999999915739E-14, -3.6732051033465739E-6, -0.99999999999325373, 0.0,
    -4.587833174079871E-20, 0.99999999999325373, -3.6732051033465739E-6, 0.0,
    -0.0039, 0.0, 0.1636, 1.0 };

  static const char_T tmp_8[9] = { 'b', 'a', 's', 'e', '_', 'l', 'i', 'n', 'k' };

  static const char_T tmp_9[5] = { 'f', 'i', 'x', 'e', 'd' };

  int32_T exitg1;
  b_obj = obj;
  obj->Bodies[0] = kin_RigidBody_RigidBody_li5a1ls(iobj_0);
  obj->Bodies[1] = ki_RigidBody_RigidBody_li5a1lsh(iobj_5);
  obj->Bodies[2] = k_RigidBody_RigidBody_li5a1lshy(iobj_1);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_2->NameInternal[b_kstr] = tmp[b_kstr];
  }

  iobj_2->ParentIndex = 3.0;
  b_kstr = iobj_2->JointInternal.Type->size[0] * iobj_2->
    JointInternal.Type->size[1];
  iobj_2->JointInternal.Type->size[0] = 1;
  iobj_2->JointInternal.Type->size[1] = 8;
  kinsim_emxEnsureCapacity_char_T(iobj_2->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    iobj_2->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  kinsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_2->JointInternal.Type->size[1];
  kinsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_2->JointInternal.Type->size[0] * iobj_2->
    JointInternal.Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_2->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_0[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_1[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  switch (b_kstr) {
   case 0:
    iobj_2->JointInternal.PositionNumber = 1.0;
    iobj_2->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_2->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_2->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    iobj_2->JointInternal.PositionNumber = 1.0;
    iobj_2->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_2->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_2->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    iobj_2->JointInternal.PositionNumber = 0.0;
    iobj_2->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_2->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_2->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_2->JointInternal.JointToParentTransform[b_kstr] = tmp_2[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_2->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  iobj_2->JointInternal.JointAxisInternal[0] = 0.0;
  iobj_2->JointInternal.JointAxisInternal[1] = 0.0;
  iobj_2->JointInternal.JointAxisInternal[2] = -1.0;
  obj->Bodies[3] = iobj_2;
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_3->NameInternal[b_kstr] = tmp_4[b_kstr];
  }

  iobj_3->ParentIndex = 4.0;
  b_kstr = iobj_3->JointInternal.Type->size[0] * iobj_3->
    JointInternal.Type->size[1];
  iobj_3->JointInternal.Type->size[0] = 1;
  iobj_3->JointInternal.Type->size[1] = 8;
  kinsim_emxEnsureCapacity_char_T(iobj_3->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    iobj_3->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_3->JointInternal.Type->size[1];
  kinsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_3->JointInternal.Type->size[0] * iobj_3->
    JointInternal.Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_3->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_0[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_1[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  switch (b_kstr) {
   case 0:
    iobj_3->JointInternal.PositionNumber = 1.0;
    iobj_3->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_3->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_3->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    iobj_3->JointInternal.PositionNumber = 1.0;
    iobj_3->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_3->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_3->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    iobj_3->JointInternal.PositionNumber = 0.0;
    iobj_3->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_3->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_3->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_3->JointInternal.JointToParentTransform[b_kstr] = tmp_5[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_3->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  iobj_3->JointInternal.JointAxisInternal[0] = -1.0;
  iobj_3->JointInternal.JointAxisInternal[1] = 0.0;
  iobj_3->JointInternal.JointAxisInternal[2] = 0.0;
  obj->Bodies[4] = iobj_3;
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_4->NameInternal[b_kstr] = tmp_6[b_kstr];
  }

  iobj_4->ParentIndex = 5.0;
  b_kstr = iobj_4->JointInternal.Type->size[0] * iobj_4->
    JointInternal.Type->size[1];
  iobj_4->JointInternal.Type->size[0] = 1;
  iobj_4->JointInternal.Type->size[1] = 8;
  kinsim_emxEnsureCapacity_char_T(iobj_4->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    iobj_4->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_4->JointInternal.Type->size[1];
  kinsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_4->JointInternal.Type->size[0] * iobj_4->
    JointInternal.Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_4->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_0[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_1[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  switch (b_kstr) {
   case 0:
    iobj_4->JointInternal.PositionNumber = 1.0;
    iobj_4->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_4->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_4->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    iobj_4->JointInternal.PositionNumber = 1.0;
    iobj_4->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_4->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_4->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    iobj_4->JointInternal.PositionNumber = 0.0;
    iobj_4->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_4->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_4->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_4->JointInternal.JointToParentTransform[b_kstr] = tmp_7[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_4->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  iobj_4->JointInternal.JointAxisInternal[0] = 0.0;
  iobj_4->JointInternal.JointAxisInternal[1] = -1.0;
  iobj_4->JointInternal.JointAxisInternal[2] = 0.0;
  obj->Bodies[5] = iobj_4;
  obj->NumBodies = 6.0;
  obj->PositionNumber = 6.0;
  obj_0 = &obj->Base;
  b_kstr = obj->Base.NameInternal->size[0] * obj->Base.NameInternal->size[1];
  obj->Base.NameInternal->size[0] = 1;
  obj->Base.NameInternal->size[1] = 9;
  kinsim_emxEnsureCapacity_char_T(obj->Base.NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->Base.NameInternal->data[b_kstr] = tmp_8[b_kstr];
  }

  b_kstr = obj->Base.JointInternal.Type->size[0] * obj->
    Base.JointInternal.Type->size[1];
  obj->Base.JointInternal.Type->size[0] = 1;
  obj->Base.JointInternal.Type->size[1] = 5;
  kinsim_emxEnsureCapacity_char_T(obj->Base.JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj_0->JointInternal.Type->data[b_kstr] = tmp_9[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->Base.JointInternal.Type->size[1];
  kinsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->Base.JointInternal.Type->size[0] * obj->
    Base.JointInternal.Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj_0->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_0[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_1[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  kinsim_2link_pla_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    obj->Base.JointInternal.PositionNumber = 1.0;
    obj->Base.JointInternal.JointAxisInternal[0] = 0.0;
    obj->Base.JointInternal.JointAxisInternal[1] = 0.0;
    obj->Base.JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    obj->Base.JointInternal.PositionNumber = 1.0;
    obj->Base.JointInternal.JointAxisInternal[0] = 0.0;
    obj->Base.JointInternal.JointAxisInternal[1] = 0.0;
    obj->Base.JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    obj->Base.JointInternal.PositionNumber = 0.0;
    obj->Base.JointInternal.JointAxisInternal[0] = 0.0;
    obj->Base.JointInternal.JointAxisInternal[1] = 0.0;
    obj->Base.JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  return b_obj;
}

// Model step function
void kinsim_2link_planar_step(void)
{
  emxArray_real_T_kinsim_2link__T *b;
  robotics_slmanip_internal_b_l_T *obj;
  n_robotics_manip_internal_R_l_T *obj_0;
  emxArray_f_cell_wrap_kinsim_2_T *Ttree;
  emxArray_char_T_kinsim_2link__T *bname;
  l_robotics_manip_internal_R_l_T *obj_1;
  static const char_T tmp[6] = { 'l', 'i', 'n', 'k', '_', '6' };

  static const char_T h[7] = { 'j', 'o', 'i', 'n', 't', '_', '1' };

  static const char_T i[7] = { 'j', 'o', 'i', 'n', 't', '_', '2' };

  static const char_T j[7] = { 'j', 'o', 'i', 'n', 't', '_', '3' };

  static const char_T k[7] = { 'j', 'o', 'i', 'n', 't', '_', '4' };

  static const char_T l[7] = { 'j', 'o', 'i', 'n', 't', '_', '5' };

  static const char_T m[7] = { 'j', 'o', 'i', 'n', 't', '_', '6' };

  int32_T exitg1;
  boolean_T exitg2;
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

  // MATLABSystem: '<S13>/Get Parameter'
  ParamGet_kinsim_2link_planar_61.get_parameter(&kinsim_2link_planar_B.bid2);

  // MATLABSystem: '<S13>/Get Parameter2'
  ParamGet_kinsim_2link_planar_85.get_parameter(&kinsim_2link_planar_B.K13);

  // MATLABSystem: '<S13>/Get Parameter5'
  ParamGet_kinsim_2link_planar_88.get_parameter(&kinsim_2link_planar_B.K14);

  // MATLABSystem: '<S13>/Get Parameter4'
  ParamGet_kinsim_2link_planar_87.get_parameter(&kinsim_2link_planar_B.K23);

  // MATLABSystem: '<S13>/Get Parameter3'
  ParamGet_kinsim_2link_planar_86.get_parameter(&kinsim_2link_planar_B.K24);

  // MATLABSystem: '<S13>/Get Parameter1'
  ParamGet_kinsim_2link_planar_65.get_parameter(&kinsim_2link_planar_B.K34);

  // Integrator: '<Root>/Integrator' incorporates:
  //   MATLABSystem: '<S13>/Get Parameter'
  //   MATLABSystem: '<S13>/Get Parameter1'
  //   MATLABSystem: '<S13>/Get Parameter2'
  //   MATLABSystem: '<S13>/Get Parameter3'
  //   MATLABSystem: '<S13>/Get Parameter4'
  //   MATLABSystem: '<S13>/Get Parameter5'

  if (kinsim_2link_planar_DW.Integrator_IWORK != 0) {
    kinsim_2link_planar_X.Integrator_CSTATE[0] = kinsim_2link_planar_B.bid2;
    kinsim_2link_planar_X.Integrator_CSTATE[1] = kinsim_2link_planar_B.K13;
    kinsim_2link_planar_X.Integrator_CSTATE[2] = kinsim_2link_planar_B.K14;
    kinsim_2link_planar_X.Integrator_CSTATE[3] = kinsim_2link_planar_B.K23;
    kinsim_2link_planar_X.Integrator_CSTATE[4] = kinsim_2link_planar_B.K24;
    kinsim_2link_planar_X.Integrator_CSTATE[5] = kinsim_2link_planar_B.K34;
  }

  kinsim_2link_pla_emxInit_real_T(&b, 2);
  kinsim_2lin_emxInit_f_cell_wrap(&Ttree, 2);

  // MATLABSystem: '<S14>/MATLAB System' incorporates:
  //   Integrator: '<Root>/Integrator'

  RigidBodyTree_geometricJacobian(&kinsim_2link_planar_DW.obj.TreeInternal,
    kinsim_2link_planar_X.Integrator_CSTATE, b);

  // MATLABSystem: '<S15>/MATLAB System' incorporates:
  //   Integrator: '<Root>/Integrator'

  obj = &kinsim_2link_planar_DW.obj_b;
  obj_0 = &kinsim_2link_planar_DW.obj_b.TreeInternal;
  RigidBodyTree_forwardKinemati_l(&obj->TreeInternal,
    kinsim_2link_planar_X.Integrator_CSTATE, Ttree);
  for (kinsim_2link_planar_B.ret = 0; kinsim_2link_planar_B.ret < 16;
       kinsim_2link_planar_B.ret++) {
    kinsim_2link_planar_B.T1_l[kinsim_2link_planar_B.ret] = 0;
  }

  kinsim_2link_planar_B.T1_l[0] = 1;
  kinsim_2link_planar_B.T1_l[5] = 1;
  kinsim_2link_planar_B.T1_l[10] = 1;
  kinsim_2link_planar_B.T1_l[15] = 1;
  kinsim_2link_pla_emxInit_char_T(&bname, 2);

  // MATLABSystem: '<S15>/MATLAB System'
  kinsim_2link_planar_B.bid2 = -1.0;
  kinsim_2link_planar_B.ret = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj_0->Base.NameInternal->size[1];
  kinsim_emxEnsureCapacity_char_T(bname, kinsim_2link_planar_B.ret);
  kinsim_2link_planar_B.loop_ub = obj_0->Base.NameInternal->size[0] *
    obj_0->Base.NameInternal->size[1] - 1;
  for (kinsim_2link_planar_B.ret = 0; kinsim_2link_planar_B.ret <=
       kinsim_2link_planar_B.loop_ub; kinsim_2link_planar_B.ret++) {
    bname->data[kinsim_2link_planar_B.ret] = obj_0->Base.NameInternal->
      data[kinsim_2link_planar_B.ret];
  }

  for (kinsim_2link_planar_B.ret = 0; kinsim_2link_planar_B.ret < 6;
       kinsim_2link_planar_B.ret++) {
    kinsim_2link_planar_B.b_o2[kinsim_2link_planar_B.ret] =
      tmp[kinsim_2link_planar_B.ret];
  }

  kinsim_2link_planar_B.b_bool = false;
  if (bname->size[1] == 6) {
    kinsim_2link_planar_B.ret = 1;
    do {
      exitg1 = 0;
      if (kinsim_2link_planar_B.ret - 1 < 6) {
        kinsim_2link_planar_B.loop_ub = kinsim_2link_planar_B.ret - 1;
        if (bname->data[kinsim_2link_planar_B.loop_ub] !=
            kinsim_2link_planar_B.b_o2[kinsim_2link_planar_B.loop_ub]) {
          exitg1 = 1;
        } else {
          kinsim_2link_planar_B.ret++;
        }
      } else {
        kinsim_2link_planar_B.b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  kinsim_2link_pla_emxFree_char_T(&bname);

  // MATLABSystem: '<S15>/MATLAB System'
  if (kinsim_2link_planar_B.b_bool) {
    kinsim_2link_planar_B.bid2 = 0.0;
  } else {
    kinsim_2link_planar_B.K13 = obj->TreeInternal.NumBodies;
    kinsim_2link_planar_B.loop_ub = 0;
    exitg2 = false;
    while ((!exitg2) && (kinsim_2link_planar_B.loop_ub <= static_cast<int32_T>
                         (kinsim_2link_planar_B.K13) - 1)) {
      obj_1 = obj_0->Bodies[kinsim_2link_planar_B.loop_ub];
      for (kinsim_2link_planar_B.ret = 0; kinsim_2link_planar_B.ret < 6;
           kinsim_2link_planar_B.ret++) {
        kinsim_2link_planar_B.bname[kinsim_2link_planar_B.ret] =
          obj_1->NameInternal[kinsim_2link_planar_B.ret];
      }

      for (kinsim_2link_planar_B.ret = 0; kinsim_2link_planar_B.ret < 6;
           kinsim_2link_planar_B.ret++) {
        kinsim_2link_planar_B.b_o2[kinsim_2link_planar_B.ret] =
          tmp[kinsim_2link_planar_B.ret];
      }

      kinsim_2link_planar_B.ret = memcmp(&kinsim_2link_planar_B.bname[0],
        &kinsim_2link_planar_B.b_o2[0], 6);
      if (kinsim_2link_planar_B.ret == 0) {
        kinsim_2link_planar_B.bid2 = static_cast<real_T>
          (kinsim_2link_planar_B.loop_ub) + 1.0;
        exitg2 = true;
      } else {
        kinsim_2link_planar_B.loop_ub++;
      }
    }
  }

  if (kinsim_2link_planar_B.bid2 == 0.0) {
    memset(&kinsim_2link_planar_B.T2[0], 0, sizeof(real_T) << 4U);
    kinsim_2link_planar_B.T2[0] = 1.0;
    kinsim_2link_planar_B.T2[5] = 1.0;
    kinsim_2link_planar_B.T2[10] = 1.0;
    kinsim_2link_planar_B.T2[15] = 1.0;
  } else {
    for (kinsim_2link_planar_B.ret = 0; kinsim_2link_planar_B.ret < 16;
         kinsim_2link_planar_B.ret++) {
      kinsim_2link_planar_B.T2[kinsim_2link_planar_B.ret] = Ttree->data[
        static_cast<int32_T>(kinsim_2link_planar_B.bid2) - 1]
        .f1[kinsim_2link_planar_B.ret];
    }
  }

  kinsim_2lin_emxFree_f_cell_wrap(&Ttree);

  // MATLABSystem: '<S15>/MATLAB System'
  for (kinsim_2link_planar_B.ret = 0; kinsim_2link_planar_B.ret < 3;
       kinsim_2link_planar_B.ret++) {
    kinsim_2link_planar_B.R_f[3 * kinsim_2link_planar_B.ret] =
      kinsim_2link_planar_B.T2[kinsim_2link_planar_B.ret];
    kinsim_2link_planar_B.R_f[3 * kinsim_2link_planar_B.ret + 1] =
      kinsim_2link_planar_B.T2[kinsim_2link_planar_B.ret + 4];
    kinsim_2link_planar_B.R_f[3 * kinsim_2link_planar_B.ret + 2] =
      kinsim_2link_planar_B.T2[kinsim_2link_planar_B.ret + 8];
  }

  for (kinsim_2link_planar_B.ret = 0; kinsim_2link_planar_B.ret < 9;
       kinsim_2link_planar_B.ret++) {
    kinsim_2link_planar_B.R_g[kinsim_2link_planar_B.ret] =
      -kinsim_2link_planar_B.R_f[kinsim_2link_planar_B.ret];
  }

  for (kinsim_2link_planar_B.ret = 0; kinsim_2link_planar_B.ret < 3;
       kinsim_2link_planar_B.ret++) {
    kinsim_2link_planar_B.loop_ub = kinsim_2link_planar_B.ret << 2;
    kinsim_2link_planar_B.R[kinsim_2link_planar_B.loop_ub] =
      kinsim_2link_planar_B.R_f[3 * kinsim_2link_planar_B.ret];
    kinsim_2link_planar_B.R[kinsim_2link_planar_B.loop_ub + 1] =
      kinsim_2link_planar_B.R_f[3 * kinsim_2link_planar_B.ret + 1];
    kinsim_2link_planar_B.R[kinsim_2link_planar_B.loop_ub + 2] =
      kinsim_2link_planar_B.R_f[3 * kinsim_2link_planar_B.ret + 2];
    kinsim_2link_planar_B.R[kinsim_2link_planar_B.ret + 12] =
      kinsim_2link_planar_B.R_g[kinsim_2link_planar_B.ret + 6] *
      kinsim_2link_planar_B.T2[14] +
      (kinsim_2link_planar_B.R_g[kinsim_2link_planar_B.ret + 3] *
       kinsim_2link_planar_B.T2[13] +
       kinsim_2link_planar_B.R_g[kinsim_2link_planar_B.ret] *
       kinsim_2link_planar_B.T2[12]);
  }

  kinsim_2link_planar_B.R[3] = 0.0;
  kinsim_2link_planar_B.R[7] = 0.0;
  kinsim_2link_planar_B.R[11] = 0.0;
  kinsim_2link_planar_B.R[15] = 1.0;
  for (kinsim_2link_planar_B.ret = 0; kinsim_2link_planar_B.ret < 4;
       kinsim_2link_planar_B.ret++) {
    for (kinsim_2link_planar_B.loop_ub = 0; kinsim_2link_planar_B.loop_ub < 4;
         kinsim_2link_planar_B.loop_ub++) {
      kinsim_2link_planar_B.rtb_MATLABSystem_tmp_tmp = kinsim_2link_planar_B.ret
        << 2;
      kinsim_2link_planar_B.rtb_MATLABSystem_tmp = kinsim_2link_planar_B.loop_ub
        + kinsim_2link_planar_B.rtb_MATLABSystem_tmp_tmp;
      kinsim_2link_planar_B.T2[kinsim_2link_planar_B.rtb_MATLABSystem_tmp] = 0.0;
      kinsim_2link_planar_B.T2[kinsim_2link_planar_B.rtb_MATLABSystem_tmp] +=
        static_cast<real_T>
        (kinsim_2link_planar_B.T1_l[kinsim_2link_planar_B.rtb_MATLABSystem_tmp_tmp])
        * kinsim_2link_planar_B.R[kinsim_2link_planar_B.loop_ub];
      kinsim_2link_planar_B.T2[kinsim_2link_planar_B.rtb_MATLABSystem_tmp] +=
        static_cast<real_T>
        (kinsim_2link_planar_B.T1_l[kinsim_2link_planar_B.rtb_MATLABSystem_tmp_tmp
         + 1]) * kinsim_2link_planar_B.R[kinsim_2link_planar_B.loop_ub + 4];
      kinsim_2link_planar_B.T2[kinsim_2link_planar_B.rtb_MATLABSystem_tmp] +=
        static_cast<real_T>
        (kinsim_2link_planar_B.T1_l[kinsim_2link_planar_B.rtb_MATLABSystem_tmp_tmp
         + 2]) * kinsim_2link_planar_B.R[kinsim_2link_planar_B.loop_ub + 8];
      kinsim_2link_planar_B.T2[kinsim_2link_planar_B.rtb_MATLABSystem_tmp] +=
        static_cast<real_T>
        (kinsim_2link_planar_B.T1_l[kinsim_2link_planar_B.rtb_MATLABSystem_tmp_tmp
         + 3]) * kinsim_2link_planar_B.R[kinsim_2link_planar_B.loop_ub + 12];
    }
  }

  if (rtmIsMajorTimeStep(kinsim_2link_planar_M)) {
    // Outputs for Atomic SubSystem: '<Root>/Subscribe'
    // MATLABSystem: '<S12>/SourceBlock' incorporates:
    //   Inport: '<S17>/In1'

    kinsim_2link_pl_SystemCore_step(&kinsim_2link_planar_B.b_bool,
      kinsim_2link_planar_B.b_varargout_2_Positions,
      &kinsim_2link_planar_B.b_varargout_2_Positions_SL_Info,
      &kinsim_2link_planar_B.b_varargout_2_Positions_SL_In_p,
      kinsim_2link_planar_B.b_varargout_2_Velocities,
      &kinsim_2link_planar_B.b_varargout_2_Velocities_SL_Inf,
      &kinsim_2link_planar_B.b_varargout_2_Velocities_SL_I_o,
      kinsim_2link_planar_B.b_varargout_2_Accelerations,
      &kinsim_2link_planar_B.b_varargout_2_Accelerations_SL_,
      &kinsim_2link_planar_B.b_varargout_2_Accelerations_S_l,
      kinsim_2link_planar_B.b_varargout_2_Effort,
      &kinsim_2link_planar_B.b_varargout_2_Effort_SL_Info_Cu,
      &kinsim_2link_planar_B.b_varargout_2_Effort_SL_Info_Re,
      &kinsim_2link_planar_B.bid2, &kinsim_2link_planar_B.K13);

    // Outputs for Enabled SubSystem: '<S12>/Enabled Subsystem' incorporates:
    //   EnablePort: '<S17>/Enable'

    if (kinsim_2link_planar_B.b_bool) {
      kinsim_2link_planar_B.In1.Positions_SL_Info.CurrentLength =
        kinsim_2link_planar_B.b_varargout_2_Positions_SL_Info;
      kinsim_2link_planar_B.In1.Positions_SL_Info.ReceivedLength =
        kinsim_2link_planar_B.b_varargout_2_Positions_SL_In_p;
      kinsim_2link_planar_B.In1.Velocities_SL_Info.CurrentLength =
        kinsim_2link_planar_B.b_varargout_2_Velocities_SL_Inf;
      kinsim_2link_planar_B.In1.Velocities_SL_Info.ReceivedLength =
        kinsim_2link_planar_B.b_varargout_2_Velocities_SL_I_o;
      kinsim_2link_planar_B.In1.Accelerations_SL_Info.CurrentLength =
        kinsim_2link_planar_B.b_varargout_2_Accelerations_SL_;
      kinsim_2link_planar_B.In1.Accelerations_SL_Info.ReceivedLength =
        kinsim_2link_planar_B.b_varargout_2_Accelerations_S_l;
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
        kinsim_2link_planar_B.b_varargout_2_Effort_SL_Info_Cu;
      kinsim_2link_planar_B.In1.Effort_SL_Info.ReceivedLength =
        kinsim_2link_planar_B.b_varargout_2_Effort_SL_Info_Re;
      kinsim_2link_planar_B.In1.TimeFromStart.Sec = kinsim_2link_planar_B.bid2;
      kinsim_2link_planar_B.In1.TimeFromStart.Nsec = kinsim_2link_planar_B.K13;
    }

    // End of MATLABSystem: '<S12>/SourceBlock'
    // End of Outputs for SubSystem: '<S12>/Enabled Subsystem'
    // End of Outputs for SubSystem: '<Root>/Subscribe'
  }

  // Product: '<S5>/MatrixMultiply' incorporates:
  //   MATLABSystem: '<S14>/MATLAB System'

  for (kinsim_2link_planar_B.ret = 0; kinsim_2link_planar_B.ret < 6;
       kinsim_2link_planar_B.ret++) {
    kinsim_2link_planar_B.MatrixMultiply[kinsim_2link_planar_B.ret] = 0.0;
    for (kinsim_2link_planar_B.loop_ub = 0; kinsim_2link_planar_B.loop_ub < 6;
         kinsim_2link_planar_B.loop_ub++) {
      kinsim_2link_planar_B.bid2 = b->data[6 * kinsim_2link_planar_B.loop_ub +
        kinsim_2link_planar_B.ret] *
        kinsim_2link_planar_B.In1.Velocities[kinsim_2link_planar_B.loop_ub] +
        kinsim_2link_planar_B.MatrixMultiply[kinsim_2link_planar_B.ret];
      kinsim_2link_planar_B.MatrixMultiply[kinsim_2link_planar_B.ret] =
        kinsim_2link_planar_B.bid2;
    }
  }

  // End of Product: '<S5>/MatrixMultiply'
  kinsim_2link_pla_emxFree_real_T(&b);

  // MATLAB Function: '<S5>/MATLAB Function'
  kinsim_2link_planar_B.bid2 = kinsim_2link_planar_B.T2[4] +
    kinsim_2link_planar_B.T2[1];
  kinsim_2link_planar_B.K13 = kinsim_2link_planar_B.T2[8] +
    kinsim_2link_planar_B.T2[2];
  kinsim_2link_planar_B.K14 = kinsim_2link_planar_B.T2[6] -
    kinsim_2link_planar_B.T2[9];
  kinsim_2link_planar_B.K23 = kinsim_2link_planar_B.T2[9] +
    kinsim_2link_planar_B.T2[6];
  kinsim_2link_planar_B.K24 = kinsim_2link_planar_B.T2[8] -
    kinsim_2link_planar_B.T2[2];
  kinsim_2link_planar_B.K34 = kinsim_2link_planar_B.T2[1] -
    kinsim_2link_planar_B.T2[4];
  kinsim_2link_planar_B.R[0] = ((kinsim_2link_planar_B.T2[0] -
    kinsim_2link_planar_B.T2[5]) - kinsim_2link_planar_B.T2[10]) / 3.0;
  kinsim_2link_planar_B.R[4] = kinsim_2link_planar_B.bid2 / 3.0;
  kinsim_2link_planar_B.R[8] = kinsim_2link_planar_B.K13 / 3.0;
  kinsim_2link_planar_B.R[12] = kinsim_2link_planar_B.K14 / 3.0;
  kinsim_2link_planar_B.R[1] = kinsim_2link_planar_B.bid2 / 3.0;
  kinsim_2link_planar_B.R[5] = ((kinsim_2link_planar_B.T2[5] -
    kinsim_2link_planar_B.T2[0]) - kinsim_2link_planar_B.T2[10]) / 3.0;
  kinsim_2link_planar_B.R[9] = kinsim_2link_planar_B.K23 / 3.0;
  kinsim_2link_planar_B.R[13] = kinsim_2link_planar_B.K24 / 3.0;
  kinsim_2link_planar_B.R[2] = kinsim_2link_planar_B.K13 / 3.0;
  kinsim_2link_planar_B.R[6] = kinsim_2link_planar_B.K23 / 3.0;
  kinsim_2link_planar_B.R[10] = ((kinsim_2link_planar_B.T2[10] -
    kinsim_2link_planar_B.T2[0]) - kinsim_2link_planar_B.T2[5]) / 3.0;
  kinsim_2link_planar_B.R[14] = kinsim_2link_planar_B.K34 / 3.0;
  kinsim_2link_planar_B.R[3] = kinsim_2link_planar_B.K14 / 3.0;
  kinsim_2link_planar_B.R[7] = kinsim_2link_planar_B.K24 / 3.0;
  kinsim_2link_planar_B.R[11] = kinsim_2link_planar_B.K34 / 3.0;
  kinsim_2link_planar_B.R[15] = ((kinsim_2link_planar_B.T2[0] +
    kinsim_2link_planar_B.T2[5]) + kinsim_2link_planar_B.T2[10]) / 3.0;
  kinsim_2link_planar_eig(kinsim_2link_planar_B.R, kinsim_2link_planar_B.eigVec,
    kinsim_2link_planar_B.eigVal);
  kinsim_2link_planar_B.cartOrn[0] = kinsim_2link_planar_B.eigVal[0].re;
  kinsim_2link_planar_B.cartOrn[1] = kinsim_2link_planar_B.eigVal[1].re;
  kinsim_2link_planar_B.cartOrn[2] = kinsim_2link_planar_B.eigVal[2].re;
  kinsim_2link_planar_B.cartOrn[3] = kinsim_2link_planar_B.eigVal[3].re;
  if (!rtIsNaN(kinsim_2link_planar_B.eigVal[0].re)) {
    kinsim_2link_planar_B.ret = 1;
  } else {
    kinsim_2link_planar_B.ret = 0;
    kinsim_2link_planar_B.loop_ub = 2;
    exitg2 = false;
    while ((!exitg2) && (kinsim_2link_planar_B.loop_ub < 5)) {
      if (!rtIsNaN(kinsim_2link_planar_B.cartOrn[kinsim_2link_planar_B.loop_ub -
                   1])) {
        kinsim_2link_planar_B.ret = kinsim_2link_planar_B.loop_ub;
        exitg2 = true;
      } else {
        kinsim_2link_planar_B.loop_ub++;
      }
    }
  }

  if (kinsim_2link_planar_B.ret != 0) {
    kinsim_2link_planar_B.bid2 =
      kinsim_2link_planar_B.cartOrn[kinsim_2link_planar_B.ret - 1];
    kinsim_2link_planar_B.loop_ub = kinsim_2link_planar_B.ret - 1;
    while (kinsim_2link_planar_B.ret + 1 < 5) {
      if (kinsim_2link_planar_B.bid2 <
          kinsim_2link_planar_B.cartOrn[kinsim_2link_planar_B.ret]) {
        kinsim_2link_planar_B.bid2 =
          kinsim_2link_planar_B.cartOrn[kinsim_2link_planar_B.ret];
        kinsim_2link_planar_B.loop_ub = kinsim_2link_planar_B.ret;
      }

      kinsim_2link_planar_B.ret++;
    }

    kinsim_2link_planar_B.ret = kinsim_2link_planar_B.loop_ub;
  }

  kinsim_2link_planar_B.ret <<= 2;
  kinsim_2link_planar_B.loop_ub = kinsim_2link_planar_B.ret + 3;
  kinsim_2link_planar_B.cartOrn[0] =
    kinsim_2link_planar_B.eigVec[kinsim_2link_planar_B.loop_ub].re;
  kinsim_2link_planar_B.cartOrn[1] =
    kinsim_2link_planar_B.eigVec[kinsim_2link_planar_B.ret].re;
  kinsim_2link_planar_B.rtb_MATLABSystem_tmp_tmp = kinsim_2link_planar_B.ret + 1;
  kinsim_2link_planar_B.cartOrn[2] =
    kinsim_2link_planar_B.eigVec[kinsim_2link_planar_B.rtb_MATLABSystem_tmp_tmp]
    .re;
  kinsim_2link_planar_B.rtb_MATLABSystem_tmp = kinsim_2link_planar_B.ret + 2;
  kinsim_2link_planar_B.cartOrn[3] =
    kinsim_2link_planar_B.eigVec[kinsim_2link_planar_B.rtb_MATLABSystem_tmp].re;
  if (kinsim_2link_planar_B.eigVec[kinsim_2link_planar_B.loop_ub].re < 0.0) {
    kinsim_2link_planar_B.cartOrn[0] =
      -kinsim_2link_planar_B.eigVec[kinsim_2link_planar_B.loop_ub].re;
    kinsim_2link_planar_B.cartOrn[1] =
      -kinsim_2link_planar_B.eigVec[kinsim_2link_planar_B.ret].re;
    kinsim_2link_planar_B.cartOrn[2] =
      -kinsim_2link_planar_B.eigVec[kinsim_2link_planar_B.rtb_MATLABSystem_tmp_tmp]
      .re;
    kinsim_2link_planar_B.cartOrn[3] =
      -kinsim_2link_planar_B.eigVec[kinsim_2link_planar_B.rtb_MATLABSystem_tmp].
      re;
  }

  // BusAssignment: '<Root>/Bus Assignment' incorporates:
  //   MATLAB Function: '<S5>/MATLAB Function'

  kinsim_2link_planar_B.BusAssignment.Position.X = kinsim_2link_planar_B.T2[12];
  kinsim_2link_planar_B.BusAssignment.Position.Y = kinsim_2link_planar_B.T2[13];
  kinsim_2link_planar_B.BusAssignment.Position.Z = kinsim_2link_planar_B.T2[14];
  kinsim_2link_planar_B.BusAssignment.Orientation.X =
    kinsim_2link_planar_B.cartOrn[0];
  kinsim_2link_planar_B.BusAssignment.Orientation.Y =
    kinsim_2link_planar_B.cartOrn[1];
  kinsim_2link_planar_B.BusAssignment.Orientation.Z =
    kinsim_2link_planar_B.cartOrn[2];
  kinsim_2link_planar_B.BusAssignment.Orientation.W =
    kinsim_2link_planar_B.cartOrn[3];

  // Outputs for Atomic SubSystem: '<Root>/Publish2'
  // MATLABSystem: '<S10>/SinkBlock'
  Pub_kinsim_2link_planar_79.publish(&kinsim_2link_planar_B.BusAssignment);

  // End of Outputs for SubSystem: '<Root>/Publish2'

  // BusAssignment: '<Root>/Bus Assignment1' incorporates:
  //   MATLAB Function: '<S5>/MATLAB Function'

  kinsim_2link_planar_B.BusAssignment1.Linear.X =
    kinsim_2link_planar_B.MatrixMultiply[3];
  kinsim_2link_planar_B.BusAssignment1.Linear.Y =
    kinsim_2link_planar_B.MatrixMultiply[4];
  kinsim_2link_planar_B.BusAssignment1.Linear.Z =
    kinsim_2link_planar_B.MatrixMultiply[5];
  kinsim_2link_planar_B.BusAssignment1.Angular.X =
    kinsim_2link_planar_B.MatrixMultiply[0];
  kinsim_2link_planar_B.BusAssignment1.Angular.Y =
    kinsim_2link_planar_B.MatrixMultiply[1];
  kinsim_2link_planar_B.BusAssignment1.Angular.Z =
    kinsim_2link_planar_B.MatrixMultiply[2];

  // Outputs for Atomic SubSystem: '<Root>/Publish3'
  // MATLABSystem: '<S11>/SinkBlock'
  Pub_kinsim_2link_planar_101.publish(&kinsim_2link_planar_B.BusAssignment1);

  // End of Outputs for SubSystem: '<Root>/Publish3'

  // Clock: '<Root>/Clock1' incorporates:
  //   Clock: '<Root>/Clock'

  kinsim_2link_planar_B.bid2 = kinsim_2link_planar_M->Timing.t[0];

  // MATLAB Function: '<Root>/Assign to JointState msg' incorporates:
  //   Clock: '<Root>/Clock1'
  //   Constant: '<S6>/Constant'
  //   Integrator: '<Root>/Integrator'

  kinsim_2link_planar_B.msg = kinsim_2link_planar_P.Constant_Value;
  if (kinsim_2link_planar_B.bid2 < 0.0) {
    kinsim_2link_planar_B.K13 = ceil(kinsim_2link_planar_B.bid2);
  } else {
    kinsim_2link_planar_B.K13 = floor(kinsim_2link_planar_B.bid2);
  }

  kinsim_2link_planar_B.msg.Header.Stamp.Sec = kinsim_2link_planar_B.K13;
  kinsim_2link_planar_B.K13 = (kinsim_2link_planar_B.bid2 -
    kinsim_2link_planar_B.K13) * 1.0E+9;
  if (kinsim_2link_planar_B.K13 < 0.0) {
    kinsim_2link_planar_B.msg.Header.Stamp.Nsec = ceil(kinsim_2link_planar_B.K13);
  } else {
    kinsim_2link_planar_B.msg.Header.Stamp.Nsec = floor
      (kinsim_2link_planar_B.K13);
  }

  kinsim_2link_planar_B.msg.Name_SL_Info.CurrentLength = 6U;
  kinsim_2link_planar_B.msg.Position_SL_Info.CurrentLength = 6U;
  kinsim_2link_planar_B.msg.Velocity_SL_Info.CurrentLength = 6U;
  kinsim_2link_planar_B.msg.Name[0].Data_SL_Info.CurrentLength = 7U;
  kinsim_2link_planar_B.msg.Position[0] =
    kinsim_2link_planar_X.Integrator_CSTATE[0];
  kinsim_2link_planar_B.msg.Velocity[0] = kinsim_2link_planar_B.In1.Velocities[0];
  kinsim_2link_planar_B.msg.Name[1].Data_SL_Info.CurrentLength = 7U;
  kinsim_2link_planar_B.msg.Position[1] =
    kinsim_2link_planar_X.Integrator_CSTATE[1];
  kinsim_2link_planar_B.msg.Velocity[1] = kinsim_2link_planar_B.In1.Velocities[1];
  kinsim_2link_planar_B.msg.Name[2].Data_SL_Info.CurrentLength = 7U;
  kinsim_2link_planar_B.msg.Position[2] =
    kinsim_2link_planar_X.Integrator_CSTATE[2];
  kinsim_2link_planar_B.msg.Velocity[2] = kinsim_2link_planar_B.In1.Velocities[2];
  kinsim_2link_planar_B.msg.Name[3].Data_SL_Info.CurrentLength = 7U;
  kinsim_2link_planar_B.msg.Position[3] =
    kinsim_2link_planar_X.Integrator_CSTATE[3];
  kinsim_2link_planar_B.msg.Velocity[3] = kinsim_2link_planar_B.In1.Velocities[3];
  kinsim_2link_planar_B.msg.Name[4].Data_SL_Info.CurrentLength = 7U;
  kinsim_2link_planar_B.msg.Position[4] =
    kinsim_2link_planar_X.Integrator_CSTATE[4];
  kinsim_2link_planar_B.msg.Velocity[4] = kinsim_2link_planar_B.In1.Velocities[4];
  for (kinsim_2link_planar_B.ret = 0; kinsim_2link_planar_B.ret < 7;
       kinsim_2link_planar_B.ret++) {
    kinsim_2link_planar_B.b_i.f1[kinsim_2link_planar_B.ret] =
      h[kinsim_2link_planar_B.ret];
    kinsim_2link_planar_B.c_l.f1[kinsim_2link_planar_B.ret] =
      i[kinsim_2link_planar_B.ret];
    kinsim_2link_planar_B.d_o.f1[kinsim_2link_planar_B.ret] =
      j[kinsim_2link_planar_B.ret];
    kinsim_2link_planar_B.e.f1[kinsim_2link_planar_B.ret] =
      k[kinsim_2link_planar_B.ret];
    kinsim_2link_planar_B.f.f1[kinsim_2link_planar_B.ret] =
      l[kinsim_2link_planar_B.ret];
    kinsim_2link_planar_B.g.f1[kinsim_2link_planar_B.ret] =
      m[kinsim_2link_planar_B.ret];
    kinsim_2link_planar_B.msg.Name[0].Data[kinsim_2link_planar_B.ret] =
      static_cast<uint8_T>
      (kinsim_2link_planar_B.b_i.f1[kinsim_2link_planar_B.ret]);
    kinsim_2link_planar_B.msg.Name[1].Data[kinsim_2link_planar_B.ret] =
      static_cast<uint8_T>
      (kinsim_2link_planar_B.c_l.f1[kinsim_2link_planar_B.ret]);
    kinsim_2link_planar_B.msg.Name[2].Data[kinsim_2link_planar_B.ret] =
      static_cast<uint8_T>
      (kinsim_2link_planar_B.d_o.f1[kinsim_2link_planar_B.ret]);
    kinsim_2link_planar_B.msg.Name[3].Data[kinsim_2link_planar_B.ret] =
      static_cast<uint8_T>(kinsim_2link_planar_B.e.f1[kinsim_2link_planar_B.ret]);
    kinsim_2link_planar_B.msg.Name[4].Data[kinsim_2link_planar_B.ret] =
      static_cast<uint8_T>(kinsim_2link_planar_B.f.f1[kinsim_2link_planar_B.ret]);
    kinsim_2link_planar_B.msg.Name[5].Data[kinsim_2link_planar_B.ret] =
      static_cast<uint8_T>(kinsim_2link_planar_B.g.f1[kinsim_2link_planar_B.ret]);
  }

  kinsim_2link_planar_B.msg.Name[5].Data_SL_Info.CurrentLength = 7U;
  kinsim_2link_planar_B.msg.Position[5] =
    kinsim_2link_planar_X.Integrator_CSTATE[5];
  kinsim_2link_planar_B.msg.Velocity[5] = kinsim_2link_planar_B.In1.Velocities[5];

  // End of MATLAB Function: '<Root>/Assign to JointState msg'

  // Outputs for Atomic SubSystem: '<Root>/Publish'
  // MATLABSystem: '<S8>/SinkBlock'
  Pub_kinsim_2link_planar_22.publish(&kinsim_2link_planar_B.msg);

  // End of Outputs for SubSystem: '<Root>/Publish'

  // MATLAB Function: '<Root>/Assign to Time msg'
  if (kinsim_2link_planar_B.bid2 < 0.0) {
    kinsim_2link_planar_B.K13 = ceil(kinsim_2link_planar_B.bid2);
  } else {
    kinsim_2link_planar_B.K13 = floor(kinsim_2link_planar_B.bid2);
  }

  kinsim_2link_planar_B.msg_l.Clock_.Sec = kinsim_2link_planar_B.K13;
  kinsim_2link_planar_B.K13 = (kinsim_2link_planar_B.bid2 -
    kinsim_2link_planar_B.K13) * 1.0E+9;
  if (kinsim_2link_planar_B.K13 < 0.0) {
    kinsim_2link_planar_B.msg_l.Clock_.Nsec = ceil(kinsim_2link_planar_B.K13);
  } else {
    kinsim_2link_planar_B.msg_l.Clock_.Nsec = floor(kinsim_2link_planar_B.K13);
  }

  // End of MATLAB Function: '<Root>/Assign to Time msg'

  // Outputs for Atomic SubSystem: '<Root>/Publish1'
  // MATLABSystem: '<S9>/SinkBlock'
  Pub_kinsim_2link_planar_50.publish(&kinsim_2link_planar_B.msg_l);

  // End of Outputs for SubSystem: '<Root>/Publish1'
  if (rtmIsMajorTimeStep(kinsim_2link_planar_M)) {
    // Update for Integrator: '<Root>/Integrator'
    kinsim_2link_planar_DW.Integrator_IWORK = 0;
  }                                    // end MajorTimeStep

  if (rtmIsMajorTimeStep(kinsim_2link_planar_M)) {
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
  kinsim_2link_planar_M->Timing.stepSize0 = 0.05;
  rtmSetFirstInitCond(kinsim_2link_planar_M, 1);

  {
    char_T tmp[7];
    int32_T i;
    static const char_T tmp_0[17] = { '/', 'j', 'o', 'i', 'n', 't', '_', 't',
      'r', 'a', 'j', 'e', 'c', 't', 'o', 'r', 'y' };

    static const char_T tmp_1[14] = { '/', 'c', 'a', 'r', 't', 'e', 's', 'i',
      'a', 'n', '_', 'p', 'o', 's' };

    static const char_T tmp_2[14] = { '/', 'c', 'a', 'r', 't', 'e', 's', 'i',
      'a', 'n', '_', 'v', 'e', 'l' };

    static const char_T tmp_3[13] = { '/', 'j', 'o', 'i', 'n', 't', '_', 's',
      't', 'a', 't', 'e', 's' };

    static const char_T tmp_4[6] = { '/', 'c', 'l', 'o', 'c', 'k' };

    static const char_T tmp_5[11] = { '/', 'q', '1', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_6[11] = { '/', 'q', '2', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_7[11] = { '/', 'q', '3', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_8[11] = { '/', 'q', '4', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_9[11] = { '/', 'q', '5', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_a[11] = { '/', 'q', '6', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

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
    // SystemInitialize for Enabled SubSystem: '<S12>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S17>/Out1'
    kinsim_2link_planar_B.In1 = kinsim_2link_planar_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S12>/Enabled Subsystem'

    // Start for MATLABSystem: '<S12>/SourceBlock'
    kinsim_2link_planar_DW.obj_pp.matlabCodegenIsDeleted = false;
    kinsim_2link_planar_DW.obj_pp.isInitialized = 1;
    for (i = 0; i < 17; i++) {
      kinsim_2link_planar_B.cv[i] = tmp_0[i];
    }

    kinsim_2link_planar_B.cv[17] = '\x00';
    Sub_kinsim_2link_planar_16.createSubscriber(kinsim_2link_planar_B.cv, 1);
    kinsim_2link_planar_DW.obj_pp.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S12>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish2'
    // Start for MATLABSystem: '<S10>/SinkBlock'
    kinsim_2link_planar_DW.obj_m.matlabCodegenIsDeleted = false;
    kinsim_2link_planar_DW.obj_m.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      kinsim_2link_planar_B.cv1[i] = tmp_1[i];
    }

    kinsim_2link_planar_B.cv1[14] = '\x00';
    Pub_kinsim_2link_planar_79.createPublisher(kinsim_2link_planar_B.cv1, 1);
    kinsim_2link_planar_DW.obj_m.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish2'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish3'
    // Start for MATLABSystem: '<S11>/SinkBlock'
    kinsim_2link_planar_DW.obj_bs.matlabCodegenIsDeleted = false;
    kinsim_2link_planar_DW.obj_bs.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      kinsim_2link_planar_B.cv1[i] = tmp_2[i];
    }

    kinsim_2link_planar_B.cv1[14] = '\x00';
    Pub_kinsim_2link_planar_101.createPublisher(kinsim_2link_planar_B.cv1, 1);
    kinsim_2link_planar_DW.obj_bs.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish3'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish'
    // Start for MATLABSystem: '<S8>/SinkBlock'
    kinsim_2link_planar_DW.obj_nr.matlabCodegenIsDeleted = false;
    kinsim_2link_planar_DW.obj_nr.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      kinsim_2link_planar_B.cv2[i] = tmp_3[i];
    }

    kinsim_2link_planar_B.cv2[13] = '\x00';
    Pub_kinsim_2link_planar_22.createPublisher(kinsim_2link_planar_B.cv2, 1);
    kinsim_2link_planar_DW.obj_nr.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S8>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish1'
    // Start for MATLABSystem: '<S9>/SinkBlock'
    kinsim_2link_planar_DW.obj_f.matlabCodegenIsDeleted = false;
    kinsim_2link_planar_DW.obj_f.isInitialized = 1;
    for (i = 0; i < 6; i++) {
      tmp[i] = tmp_4[i];
    }

    tmp[6] = '\x00';
    Pub_kinsim_2link_planar_50.createPublisher(tmp, 1);
    kinsim_2link_planar_DW.obj_f.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S9>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish1'

    // Start for MATLABSystem: '<S13>/Get Parameter'
    kinsim_2link_planar_DW.obj_e.matlabCodegenIsDeleted = false;
    kinsim_2link_planar_DW.obj_e.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      kinsim_2link_planar_B.cv3[i] = tmp_5[i];
    }

    kinsim_2link_planar_B.cv3[11] = '\x00';
    ParamGet_kinsim_2link_planar_61.initialize(kinsim_2link_planar_B.cv3);
    ParamGet_kinsim_2link_planar_61.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinsim_2link_planar_61.set_initial_value(0.0);
    kinsim_2link_planar_DW.obj_e.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S13>/Get Parameter'

    // Start for MATLABSystem: '<S13>/Get Parameter2'
    kinsim_2link_planar_DW.obj_p.matlabCodegenIsDeleted = false;
    kinsim_2link_planar_DW.obj_p.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      kinsim_2link_planar_B.cv3[i] = tmp_6[i];
    }

    kinsim_2link_planar_B.cv3[11] = '\x00';
    ParamGet_kinsim_2link_planar_85.initialize(kinsim_2link_planar_B.cv3);
    ParamGet_kinsim_2link_planar_85.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinsim_2link_planar_85.set_initial_value(0.0);
    kinsim_2link_planar_DW.obj_p.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S13>/Get Parameter2'

    // Start for MATLABSystem: '<S13>/Get Parameter5'
    kinsim_2link_planar_DW.obj_n.matlabCodegenIsDeleted = false;
    kinsim_2link_planar_DW.obj_n.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      kinsim_2link_planar_B.cv3[i] = tmp_7[i];
    }

    kinsim_2link_planar_B.cv3[11] = '\x00';
    ParamGet_kinsim_2link_planar_88.initialize(kinsim_2link_planar_B.cv3);
    ParamGet_kinsim_2link_planar_88.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinsim_2link_planar_88.set_initial_value(0.0);
    kinsim_2link_planar_DW.obj_n.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S13>/Get Parameter5'

    // Start for MATLABSystem: '<S13>/Get Parameter4'
    kinsim_2link_planar_DW.obj_em.matlabCodegenIsDeleted = false;
    kinsim_2link_planar_DW.obj_em.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      kinsim_2link_planar_B.cv3[i] = tmp_8[i];
    }

    kinsim_2link_planar_B.cv3[11] = '\x00';
    ParamGet_kinsim_2link_planar_87.initialize(kinsim_2link_planar_B.cv3);
    ParamGet_kinsim_2link_planar_87.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinsim_2link_planar_87.set_initial_value(0.0);
    kinsim_2link_planar_DW.obj_em.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S13>/Get Parameter4'

    // Start for MATLABSystem: '<S13>/Get Parameter3'
    kinsim_2link_planar_DW.obj_l.matlabCodegenIsDeleted = false;
    kinsim_2link_planar_DW.obj_l.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      kinsim_2link_planar_B.cv3[i] = tmp_9[i];
    }

    kinsim_2link_planar_B.cv3[11] = '\x00';
    ParamGet_kinsim_2link_planar_86.initialize(kinsim_2link_planar_B.cv3);
    ParamGet_kinsim_2link_planar_86.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinsim_2link_planar_86.set_initial_value(0.0);
    kinsim_2link_planar_DW.obj_l.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S13>/Get Parameter3'

    // Start for MATLABSystem: '<S13>/Get Parameter1'
    kinsim_2link_planar_DW.obj_ng.matlabCodegenIsDeleted = false;
    kinsim_2link_planar_DW.obj_ng.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      kinsim_2link_planar_B.cv3[i] = tmp_a[i];
    }

    kinsim_2link_planar_B.cv3[11] = '\x00';
    ParamGet_kinsim_2link_planar_65.initialize(kinsim_2link_planar_B.cv3);
    ParamGet_kinsim_2link_planar_65.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinsim_2link_planar_65.set_initial_value(0.0);
    kinsim_2link_planar_DW.obj_ng.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S13>/Get Parameter1'
    emxInitStruct_robotics_slmanip_(&kinsim_2link_planar_DW.obj);
    emxInitStruct_l_robotics_manip_(&kinsim_2link_planar_DW.gobj_1);
    emxInitStruct_l_robotics_manip_(&kinsim_2link_planar_DW.gobj_12);
    emxInitStruct_l_robotics_manip_(&kinsim_2link_planar_DW.gobj_11);
    emxInitStruct_l_robotics_manip_(&kinsim_2link_planar_DW.gobj_10);
    emxInitStruct_l_robotics_manip_(&kinsim_2link_planar_DW.gobj_9);
    emxInitStruct_l_robotics_manip_(&kinsim_2link_planar_DW.gobj_8);
    emxInitStruct_l_robotics_manip_(&kinsim_2link_planar_DW.gobj_7);
    emxInitStruct_l_robotics_manip_(&kinsim_2link_planar_DW.gobj_6);
    emxInitStruct_l_robotics_manip_(&kinsim_2link_planar_DW.gobj_5);
    emxInitStruct_l_robotics_manip_(&kinsim_2link_planar_DW.gobj_4);
    emxInitStruct_l_robotics_manip_(&kinsim_2link_planar_DW.gobj_3);
    emxInitStruct_l_robotics_manip_(&kinsim_2link_planar_DW.gobj_2);

    // Start for MATLABSystem: '<S14>/MATLAB System'
    kinsim_2link_planar_DW.obj.isInitialized = 0;
    kinsim_2link_planar_DW.obj.isInitialized = 1;
    kin_RigidBodyTree_RigidBodyTree(&kinsim_2link_planar_DW.obj.TreeInternal,
      &kinsim_2link_planar_DW.gobj_2, &kinsim_2link_planar_DW.gobj_4,
      &kinsim_2link_planar_DW.gobj_5, &kinsim_2link_planar_DW.gobj_6,
      &kinsim_2link_planar_DW.gobj_7, &kinsim_2link_planar_DW.gobj_3);
    emxInitStruct_robotics_slmani_l(&kinsim_2link_planar_DW.obj_b);
    emxInitStruct_l_robotics_mani_l(&kinsim_2link_planar_DW.gobj_1_a);
    emxInitStruct_l_robotics_mani_l(&kinsim_2link_planar_DW.gobj_12_b);
    emxInitStruct_l_robotics_mani_l(&kinsim_2link_planar_DW.gobj_11_l);
    emxInitStruct_l_robotics_mani_l(&kinsim_2link_planar_DW.gobj_10_b);
    emxInitStruct_l_robotics_mani_l(&kinsim_2link_planar_DW.gobj_9_p);
    emxInitStruct_l_robotics_mani_l(&kinsim_2link_planar_DW.gobj_8_j);
    emxInitStruct_l_robotics_mani_l(&kinsim_2link_planar_DW.gobj_7_l);
    emxInitStruct_l_robotics_mani_l(&kinsim_2link_planar_DW.gobj_6_g);
    emxInitStruct_l_robotics_mani_l(&kinsim_2link_planar_DW.gobj_5_a);
    emxInitStruct_l_robotics_mani_l(&kinsim_2link_planar_DW.gobj_4_i);
    emxInitStruct_l_robotics_mani_l(&kinsim_2link_planar_DW.gobj_3_j);
    emxInitStruct_l_robotics_mani_l(&kinsim_2link_planar_DW.gobj_2_o);

    // Start for MATLABSystem: '<S15>/MATLAB System'
    kinsim_2link_planar_DW.obj_b.isInitialized = 0;
    kinsim_2link_planar_DW.obj_b.isInitialized = 1;
    k_RigidBodyTree_RigidBodyTree_l(&kinsim_2link_planar_DW.obj_b.TreeInternal,
      &kinsim_2link_planar_DW.gobj_2_o, &kinsim_2link_planar_DW.gobj_4_i,
      &kinsim_2link_planar_DW.gobj_5_a, &kinsim_2link_planar_DW.gobj_6_g,
      &kinsim_2link_planar_DW.gobj_7_l, &kinsim_2link_planar_DW.gobj_3_j);
  }

  // set "at time zero" to false
  if (rtmIsFirstInitCond(kinsim_2link_planar_M)) {
    rtmSetFirstInitCond(kinsim_2link_planar_M, 0);
  }
}

// Model terminate function
void kinsim_2link_planar_terminate(void)
{
  // Terminate for MATLABSystem: '<S13>/Get Parameter'
  matlabCodegenHandle_matla_li5a1(&kinsim_2link_planar_DW.obj_e);

  // Terminate for MATLABSystem: '<S13>/Get Parameter2'
  matlabCodegenHandle_matla_li5a1(&kinsim_2link_planar_DW.obj_p);

  // Terminate for MATLABSystem: '<S13>/Get Parameter5'
  matlabCodegenHandle_matla_li5a1(&kinsim_2link_planar_DW.obj_n);

  // Terminate for MATLABSystem: '<S13>/Get Parameter4'
  matlabCodegenHandle_matla_li5a1(&kinsim_2link_planar_DW.obj_em);

  // Terminate for MATLABSystem: '<S13>/Get Parameter3'
  matlabCodegenHandle_matla_li5a1(&kinsim_2link_planar_DW.obj_l);

  // Terminate for MATLABSystem: '<S13>/Get Parameter1'
  matlabCodegenHandle_matla_li5a1(&kinsim_2link_planar_DW.obj_ng);
  emxFreeStruct_robotics_slmanip_(&kinsim_2link_planar_DW.obj);
  emxFreeStruct_l_robotics_manip_(&kinsim_2link_planar_DW.gobj_1);
  emxFreeStruct_l_robotics_manip_(&kinsim_2link_planar_DW.gobj_12);
  emxFreeStruct_l_robotics_manip_(&kinsim_2link_planar_DW.gobj_11);
  emxFreeStruct_l_robotics_manip_(&kinsim_2link_planar_DW.gobj_10);
  emxFreeStruct_l_robotics_manip_(&kinsim_2link_planar_DW.gobj_9);
  emxFreeStruct_l_robotics_manip_(&kinsim_2link_planar_DW.gobj_8);
  emxFreeStruct_l_robotics_manip_(&kinsim_2link_planar_DW.gobj_7);
  emxFreeStruct_l_robotics_manip_(&kinsim_2link_planar_DW.gobj_6);
  emxFreeStruct_l_robotics_manip_(&kinsim_2link_planar_DW.gobj_5);
  emxFreeStruct_l_robotics_manip_(&kinsim_2link_planar_DW.gobj_4);
  emxFreeStruct_l_robotics_manip_(&kinsim_2link_planar_DW.gobj_3);
  emxFreeStruct_l_robotics_manip_(&kinsim_2link_planar_DW.gobj_2);
  emxFreeStruct_robotics_slmani_l(&kinsim_2link_planar_DW.obj_b);
  emxFreeStruct_l_robotics_mani_l(&kinsim_2link_planar_DW.gobj_1_a);
  emxFreeStruct_l_robotics_mani_l(&kinsim_2link_planar_DW.gobj_12_b);
  emxFreeStruct_l_robotics_mani_l(&kinsim_2link_planar_DW.gobj_11_l);
  emxFreeStruct_l_robotics_mani_l(&kinsim_2link_planar_DW.gobj_10_b);
  emxFreeStruct_l_robotics_mani_l(&kinsim_2link_planar_DW.gobj_9_p);
  emxFreeStruct_l_robotics_mani_l(&kinsim_2link_planar_DW.gobj_8_j);
  emxFreeStruct_l_robotics_mani_l(&kinsim_2link_planar_DW.gobj_7_l);
  emxFreeStruct_l_robotics_mani_l(&kinsim_2link_planar_DW.gobj_6_g);
  emxFreeStruct_l_robotics_mani_l(&kinsim_2link_planar_DW.gobj_5_a);
  emxFreeStruct_l_robotics_mani_l(&kinsim_2link_planar_DW.gobj_4_i);
  emxFreeStruct_l_robotics_mani_l(&kinsim_2link_planar_DW.gobj_3_j);
  emxFreeStruct_l_robotics_mani_l(&kinsim_2link_planar_DW.gobj_2_o);

  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S12>/SourceBlock'
  matlabCodegenHandle_matlab_li5a(&kinsim_2link_planar_DW.obj_pp);

  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for Atomic SubSystem: '<Root>/Publish2'
  // Terminate for MATLABSystem: '<S10>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&kinsim_2link_planar_DW.obj_m);

  // End of Terminate for SubSystem: '<Root>/Publish2'

  // Terminate for Atomic SubSystem: '<Root>/Publish3'
  // Terminate for MATLABSystem: '<S11>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&kinsim_2link_planar_DW.obj_bs);

  // End of Terminate for SubSystem: '<Root>/Publish3'

  // Terminate for Atomic SubSystem: '<Root>/Publish'
  // Terminate for MATLABSystem: '<S8>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&kinsim_2link_planar_DW.obj_nr);

  // End of Terminate for SubSystem: '<Root>/Publish'

  // Terminate for Atomic SubSystem: '<Root>/Publish1'
  // Terminate for MATLABSystem: '<S9>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&kinsim_2link_planar_DW.obj_f);

  // End of Terminate for SubSystem: '<Root>/Publish1'
}

//
// File trailer for generated code.
//
// [EOF]
//
