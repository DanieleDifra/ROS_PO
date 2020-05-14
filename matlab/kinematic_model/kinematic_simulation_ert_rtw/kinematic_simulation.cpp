//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: kinematic_simulation.cpp
//
// Code generated for Simulink model 'kinematic_simulation'.
//
// Model version                  : 1.130
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Thu May 14 23:07:46 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "kinematic_simulation.h"
#include "kinematic_simulation_private.h"

// Block signals (default storage)
B_kinematic_simulation_T kinematic_simulation_B;

// Continuous states
X_kinematic_simulation_T kinematic_simulation_X;

// Block states (default storage)
DW_kinematic_simulation_T kinematic_simulation_DW;

// Real-time model
RT_MODEL_kinematic_simulation_T kinematic_simulation_M_ =
  RT_MODEL_kinematic_simulation_T();
RT_MODEL_kinematic_simulation_T *const kinematic_simulation_M =
  &kinematic_simulation_M_;

// Forward declaration for local functions
static void kinematic_simula_emxInit_real_T(emxArray_real_T_kinematic_sim_T
  **pEmxArray, int32_T numDimensions);
static void kinematic_s_emxInit_f_cell_wrap(emxArray_f_cell_wrap_kinemati_T
  **pEmxArray, int32_T numDimensions);
static void kinematic_simula_emxInit_char_T(emxArray_char_T_kinematic_sim_T
  **pEmxArray, int32_T numDimensions);
static void k_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_kinemati_T
  *emxArray, int32_T oldNumel);
static void kinema_emxEnsureCapacity_char_T(emxArray_char_T_kinematic_sim_T
  *emxArray, int32_T oldNumel);
static void ki_rigidBodyJoint_get_JointAxis(const
  c_rigidBodyJoint_kinematic_si_T *obj, real_T ax[3]);
static void kinematic_simula_emxFree_char_T(emxArray_char_T_kinematic_sim_T
  **pEmxArray);
static void RigidBodyTree_forwardKinematics(p_robotics_manip_internal_Rig_T *obj,
  const real_T qvec[6], emxArray_f_cell_wrap_kinemati_T *Ttree);
static void kinema_emxEnsureCapacity_real_T(emxArray_real_T_kinematic_sim_T
  *emxArray, int32_T oldNumel);
static void kinematic_simula_emxFree_real_T(emxArray_real_T_kinematic_sim_T
  **pEmxArray);
static void kinematic_s_emxFree_f_cell_wrap(emxArray_f_cell_wrap_kinemati_T
  **pEmxArray);
static void RigidBodyTree_geometricJacobian(p_robotics_manip_internal_Rig_T *obj,
  const real_T Q[6], emxArray_real_T_kinematic_sim_T *Jac);
static void rigidBodyJoint_get_JointAxis_a(const c_rigidBodyJoint_kinematic__a_T
  *obj, real_T ax[3]);
static void RigidBodyTree_forwardKinemati_a(p_robotics_manip_internal_R_a_T *obj,
  const real_T qvec[6], emxArray_f_cell_wrap_kinemati_T *Ttree);
static void kinematic_simul_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Positions[128], uint32_T *varargout_2_Positions_SL_Info_C,
  uint32_T *varargout_2_Positions_SL_Info_R, real_T varargout_2_Velocities[128],
  uint32_T *varargout_2_Velocities_SL_Info_, uint32_T
  *varargout_2_Velocities_SL_Inf_0, real_T varargout_2_Accelerations[128],
  uint32_T *varargout_2_Accelerations_SL_In, uint32_T
  *varargout_2_Accelerations_SL__0, real_T varargout_2_Effort[128], uint32_T
  *varargout_2_Effort_SL_Info_Curr, uint32_T *varargout_2_Effort_SL_Info_Rece,
  real_T *varargout_2_TimeFromStart_Sec, real_T *varargout_2_TimeFromStart_Nsec);
static boolean_T kinematic_simulati_anyNonFinite(const real_T x[16]);
static real_T kinematic_simulat_rt_hypotd_snf(real_T u0, real_T u1);
static real_T kinematic_simulation_xzlangeM(const creal_T x[16]);
static void kinematic_simulation_xzlascl(real_T cfrom, real_T cto, creal_T A[16]);
static real_T kinematic_simulation_xzlanhs(const creal_T A[16], int32_T ilo,
  int32_T ihi);
static void kinematic_simulation_xzlartg_a(const creal_T f, const creal_T g,
  real_T *cs, creal_T *sn);
static void kinematic_simulation_xzlartg(const creal_T f, const creal_T g,
  real_T *cs, creal_T *sn, creal_T *r);
static void kinematic_simulation_xzhgeqz(creal_T A[16], int32_T ilo, int32_T ihi,
  creal_T Z[16], int32_T *info, creal_T alpha1[4], creal_T beta1[4]);
static void kinematic_simulation_xztgevc(const creal_T A[16], creal_T V[16]);
static void kinematic_simulation_xzggev(creal_T A[16], int32_T *info, creal_T
  alpha1[4], creal_T beta1[4], creal_T V[16]);
static real_T kinematic_simulation_xnrm2(int32_T n, const real_T x[16], int32_T
  ix0);
static void kinematic_simulation_xzlarf(int32_T m, int32_T n, int32_T iv0,
  real_T tau, real_T C[16], int32_T ic0, real_T work[4]);
static void kinematic_simulation_xgehrd(real_T a[16], real_T tau[3]);
static real_T kinematic_simulation_xnrm2_l(int32_T n, const real_T x[3]);
static real_T kinematic_simulation_xzlarfg(int32_T n, real_T *alpha1, real_T x[3]);
static void kinematic_simulation_xdlanv2(real_T *a, real_T *b, real_T *c, real_T
  *d, real_T *rt1r, real_T *rt1i, real_T *rt2r, real_T *rt2i, real_T *cs, real_T
  *sn);
static void kinematic_simulation_xrot(int32_T n, real_T x[16], int32_T ix0,
  int32_T iy0, real_T c, real_T s);
static void kinematic_simulation_xrot_g(int32_T n, real_T x[16], int32_T ix0,
  int32_T iy0, real_T c, real_T s);
static int32_T kinematic_simulation_eml_dlahqr(real_T h[16], real_T z[16]);
static void kinematic_simulation_eig(const real_T A[16], creal_T V[16], creal_T
  D[4]);
static void matlabCodegenHandle_matla_alw5t(ros_slros_internal_block_GetP_T *obj);
static void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_kinematic_si_T
  *pStruct);
static void emxFreeStruct_o_robotics_manip_(o_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_p_robotics_manip_(p_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct);
static void emxFreeStruct_n_robotics_manip_(n_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_c_rigidBodyJoint1(c_rigidBodyJoint_kinematic__a_T
  *pStruct);
static void emxFreeStruct_o_robotics_mani_a(o_robotics_manip_internal_R_a_T
  *pStruct);
static void emxFreeStruct_p_robotics_mani_a(p_robotics_manip_internal_R_a_T
  *pStruct);
static void emxFreeStruct_robotics_slmani_a(robotics_slmanip_internal_b_a_T
  *pStruct);
static void emxFreeStruct_n_robotics_mani_a(n_robotics_manip_internal_R_a_T
  *pStruct);
static void matlabCodegenHandle_matlab_alw5(ros_slros_internal_block_Subs_T *obj);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Publ_T *obj);
static void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_kinematic_si_T
  *pStruct);
static void emxInitStruct_o_robotics_manip_(o_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_p_robotics_manip_(p_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct);
static void emxInitStruct_n_robotics_manip_(n_robotics_manip_internal_Rig_T
  *pStruct);
static n_robotics_manip_internal_Rig_T *kinematic_s_RigidBody_RigidBody
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *kinematic_RigidBody_RigidBody_a
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *kinemati_RigidBody_RigidBody_al
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *kinemat_RigidBody_RigidBody_alw
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *kinema_RigidBody_RigidBody_alw5
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *kinem_RigidBody_RigidBody_alw5t
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *kine_RigidBody_RigidBody_alw5ty
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *kin_RigidBody_RigidBody_alw5typ
  (n_robotics_manip_internal_Rig_T *obj);
static o_robotics_manip_internal_Rig_T *ki_RigidBody_RigidBody_alw5typh
  (o_robotics_manip_internal_Rig_T *obj);
static p_robotics_manip_internal_Rig_T *kin_RigidBodyTree_RigidBodyTree
  (p_robotics_manip_internal_Rig_T *obj, n_robotics_manip_internal_Rig_T *iobj_0,
   n_robotics_manip_internal_Rig_T *iobj_1, n_robotics_manip_internal_Rig_T
   *iobj_2, n_robotics_manip_internal_Rig_T *iobj_3,
   n_robotics_manip_internal_Rig_T *iobj_4, n_robotics_manip_internal_Rig_T
   *iobj_5, n_robotics_manip_internal_Rig_T *iobj_6,
   n_robotics_manip_internal_Rig_T *iobj_7);
static void emxInitStruct_c_rigidBodyJoint1(c_rigidBodyJoint_kinematic__a_T
  *pStruct);
static void emxInitStruct_o_robotics_mani_a(o_robotics_manip_internal_R_a_T
  *pStruct);
static void emxInitStruct_p_robotics_mani_a(p_robotics_manip_internal_R_a_T
  *pStruct);
static void emxInitStruct_robotics_slmani_a(robotics_slmanip_internal_b_a_T
  *pStruct);
static void emxInitStruct_n_robotics_mani_a(n_robotics_manip_internal_R_a_T
  *pStruct);
static n_robotics_manip_internal_R_a_T *k_RigidBody_RigidBody_alw5typh4
  (n_robotics_manip_internal_R_a_T *obj);
static n_robotics_manip_internal_R_a_T *RigidBody_RigidBody_alw5typh44
  (n_robotics_manip_internal_R_a_T *obj);
static n_robotics_manip_internal_R_a_T *RigidBody_RigidBody_alw5typh44u
  (n_robotics_manip_internal_R_a_T *obj);
static n_robotics_manip_internal_R_a_T *RigidBody_RigidBod_alw5typh44u2
  (n_robotics_manip_internal_R_a_T *obj);
static n_robotics_manip_internal_R_a_T *RigidBody_RigidBo_alw5typh44u2s
  (n_robotics_manip_internal_R_a_T *obj);
static p_robotics_manip_internal_R_a_T *k_RigidBodyTree_RigidBodyTree_a
  (p_robotics_manip_internal_R_a_T *obj, n_robotics_manip_internal_R_a_T *iobj_0,
   n_robotics_manip_internal_R_a_T *iobj_1, n_robotics_manip_internal_R_a_T
   *iobj_2, n_robotics_manip_internal_R_a_T *iobj_3,
   n_robotics_manip_internal_R_a_T *iobj_4, n_robotics_manip_internal_R_a_T
   *iobj_5, n_robotics_manip_internal_R_a_T *iobj_6,
   n_robotics_manip_internal_R_a_T *iobj_7);
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
  kinematic_simulation_derivatives();

  // f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*));
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  kinematic_simulation_step();
  kinematic_simulation_derivatives();

  // f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*));
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  kinematic_simulation_step();
  kinematic_simulation_derivatives();

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

static void kinematic_simula_emxInit_real_T(emxArray_real_T_kinematic_sim_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_real_T_kinematic_sim_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_real_T_kinematic_sim_T *)malloc(sizeof
    (emxArray_real_T_kinematic_sim_T));
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

static void kinematic_s_emxInit_f_cell_wrap(emxArray_f_cell_wrap_kinemati_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_f_cell_wrap_kinemati_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_f_cell_wrap_kinemati_T *)malloc(sizeof
    (emxArray_f_cell_wrap_kinemati_T));
  emxArray = *pEmxArray;
  emxArray->data = (f_cell_wrap_kinematic_simulat_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void kinematic_simula_emxInit_char_T(emxArray_char_T_kinematic_sim_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_char_T_kinematic_sim_T *emxArray;
  *pEmxArray = (emxArray_char_T_kinematic_sim_T *)malloc(sizeof
    (emxArray_char_T_kinematic_sim_T));
  emxArray = *pEmxArray;
  emxArray->data = (char_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (kinematic_simulation_B.i_p = 0; kinematic_simulation_B.i_p <
       numDimensions; kinematic_simulation_B.i_p++) {
    emxArray->size[kinematic_simulation_B.i_p] = 0;
  }
}

static void k_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_kinemati_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  kinematic_simulation_B.newNumel_d = 1;
  for (kinematic_simulation_B.i_af = 0; kinematic_simulation_B.i_af <
       emxArray->numDimensions; kinematic_simulation_B.i_af++) {
    kinematic_simulation_B.newNumel_d *= emxArray->
      size[kinematic_simulation_B.i_af];
  }

  if (kinematic_simulation_B.newNumel_d > emxArray->allocatedSize) {
    kinematic_simulation_B.i_af = emxArray->allocatedSize;
    if (kinematic_simulation_B.i_af < 16) {
      kinematic_simulation_B.i_af = 16;
    }

    while (kinematic_simulation_B.i_af < kinematic_simulation_B.newNumel_d) {
      if (kinematic_simulation_B.i_af > 1073741823) {
        kinematic_simulation_B.i_af = MAX_int32_T;
      } else {
        kinematic_simulation_B.i_af <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(kinematic_simulation_B.i_af), sizeof
                     (f_cell_wrap_kinematic_simulat_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(f_cell_wrap_kinematic_simulat_T)
             * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (f_cell_wrap_kinematic_simulat_T *)newData;
    emxArray->allocatedSize = kinematic_simulation_B.i_af;
    emxArray->canFreeData = true;
  }
}

static void kinema_emxEnsureCapacity_char_T(emxArray_char_T_kinematic_sim_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  kinematic_simulation_B.newNumel = 1;
  for (kinematic_simulation_B.i_a = 0; kinematic_simulation_B.i_a <
       emxArray->numDimensions; kinematic_simulation_B.i_a++) {
    kinematic_simulation_B.newNumel *= emxArray->size[kinematic_simulation_B.i_a];
  }

  if (kinematic_simulation_B.newNumel > emxArray->allocatedSize) {
    kinematic_simulation_B.i_a = emxArray->allocatedSize;
    if (kinematic_simulation_B.i_a < 16) {
      kinematic_simulation_B.i_a = 16;
    }

    while (kinematic_simulation_B.i_a < kinematic_simulation_B.newNumel) {
      if (kinematic_simulation_B.i_a > 1073741823) {
        kinematic_simulation_B.i_a = MAX_int32_T;
      } else {
        kinematic_simulation_B.i_a <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(kinematic_simulation_B.i_a), sizeof
                     (char_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(char_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (char_T *)newData;
    emxArray->allocatedSize = kinematic_simulation_B.i_a;
    emxArray->canFreeData = true;
  }
}

static void ki_rigidBodyJoint_get_JointAxis(const
  c_rigidBodyJoint_kinematic_si_T *obj, real_T ax[3])
{
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  boolean_T guard1 = false;
  int32_T exitg1;
  for (kinematic_simulation_B.b_kstr_l = 0; kinematic_simulation_B.b_kstr_l < 8;
       kinematic_simulation_B.b_kstr_l++) {
    kinematic_simulation_B.b_d[kinematic_simulation_B.b_kstr_l] =
      tmp[kinematic_simulation_B.b_kstr_l];
  }

  kinematic_simulation_B.b_bool_i = false;
  if (obj->Type->size[1] == 8) {
    kinematic_simulation_B.b_kstr_l = 1;
    do {
      exitg1 = 0;
      if (kinematic_simulation_B.b_kstr_l - 1 < 8) {
        kinematic_simulation_B.kstr_h = kinematic_simulation_B.b_kstr_l - 1;
        if (obj->Type->data[kinematic_simulation_B.kstr_h] !=
            kinematic_simulation_B.b_d[kinematic_simulation_B.kstr_h]) {
          exitg1 = 1;
        } else {
          kinematic_simulation_B.b_kstr_l++;
        }
      } else {
        kinematic_simulation_B.b_bool_i = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (kinematic_simulation_B.b_bool_i) {
    guard1 = true;
  } else {
    for (kinematic_simulation_B.b_kstr_l = 0; kinematic_simulation_B.b_kstr_l <
         9; kinematic_simulation_B.b_kstr_l++) {
      kinematic_simulation_B.b_bs[kinematic_simulation_B.b_kstr_l] =
        tmp_0[kinematic_simulation_B.b_kstr_l];
    }

    kinematic_simulation_B.b_bool_i = false;
    if (obj->Type->size[1] == 9) {
      kinematic_simulation_B.b_kstr_l = 1;
      do {
        exitg1 = 0;
        if (kinematic_simulation_B.b_kstr_l - 1 < 9) {
          kinematic_simulation_B.kstr_h = kinematic_simulation_B.b_kstr_l - 1;
          if (obj->Type->data[kinematic_simulation_B.kstr_h] !=
              kinematic_simulation_B.b_bs[kinematic_simulation_B.kstr_h]) {
            exitg1 = 1;
          } else {
            kinematic_simulation_B.b_kstr_l++;
          }
        } else {
          kinematic_simulation_B.b_bool_i = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (kinematic_simulation_B.b_bool_i) {
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

static void kinematic_simula_emxFree_char_T(emxArray_char_T_kinematic_sim_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_char_T_kinematic_sim_T *)NULL) {
    if (((*pEmxArray)->data != (char_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_char_T_kinematic_sim_T *)NULL;
  }
}

static void RigidBodyTree_forwardKinematics(p_robotics_manip_internal_Rig_T *obj,
  const real_T qvec[6], emxArray_f_cell_wrap_kinemati_T *Ttree)
{
  n_robotics_manip_internal_Rig_T *body;
  emxArray_char_T_kinematic_sim_T *switch_expression;
  static const int8_T tmp[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  kinematic_simulation_B.n = obj->NumBodies;
  for (kinematic_simulation_B.b_kstr_e = 0; kinematic_simulation_B.b_kstr_e < 16;
       kinematic_simulation_B.b_kstr_e++) {
    kinematic_simulation_B.c_f1[kinematic_simulation_B.b_kstr_e] =
      tmp[kinematic_simulation_B.b_kstr_e];
  }

  kinematic_simulation_B.b_kstr_e = Ttree->size[0] * Ttree->size[1];
  Ttree->size[0] = 1;
  kinematic_simulation_B.e_p = static_cast<int32_T>(kinematic_simulation_B.n);
  Ttree->size[1] = kinematic_simulation_B.e_p;
  k_emxEnsureCapacity_f_cell_wrap(Ttree, kinematic_simulation_B.b_kstr_e);
  if (kinematic_simulation_B.e_p != 0) {
    kinematic_simulation_B.ntilecols = kinematic_simulation_B.e_p - 1;
    if (0 <= kinematic_simulation_B.ntilecols) {
      memcpy(&kinematic_simulation_B.expl_temp.f1[0],
             &kinematic_simulation_B.c_f1[0], sizeof(real_T) << 4U);
    }

    for (kinematic_simulation_B.b_jtilecol = 0;
         kinematic_simulation_B.b_jtilecol <= kinematic_simulation_B.ntilecols;
         kinematic_simulation_B.b_jtilecol++) {
      Ttree->data[kinematic_simulation_B.b_jtilecol] =
        kinematic_simulation_B.expl_temp;
    }
  }

  kinematic_simulation_B.k = 1.0;
  kinematic_simulation_B.ntilecols = static_cast<int32_T>
    (kinematic_simulation_B.n) - 1;
  kinematic_simula_emxInit_char_T(&switch_expression, 2);
  if (0 <= kinematic_simulation_B.ntilecols) {
    for (kinematic_simulation_B.b_kstr_e = 0; kinematic_simulation_B.b_kstr_e <
         5; kinematic_simulation_B.b_kstr_e++) {
      kinematic_simulation_B.b_ff[kinematic_simulation_B.b_kstr_e] =
        tmp_0[kinematic_simulation_B.b_kstr_e];
    }
  }

  for (kinematic_simulation_B.b_jtilecol = 0; kinematic_simulation_B.b_jtilecol <=
       kinematic_simulation_B.ntilecols; kinematic_simulation_B.b_jtilecol++) {
    body = obj->Bodies[kinematic_simulation_B.b_jtilecol];
    kinematic_simulation_B.n = body->JointInternal.PositionNumber;
    kinematic_simulation_B.n += kinematic_simulation_B.k;
    if (kinematic_simulation_B.k > kinematic_simulation_B.n - 1.0) {
      kinematic_simulation_B.e_p = 0;
      kinematic_simulation_B.d_f = 0;
    } else {
      kinematic_simulation_B.e_p = static_cast<int32_T>(kinematic_simulation_B.k)
        - 1;
      kinematic_simulation_B.d_f = static_cast<int32_T>(kinematic_simulation_B.n
        - 1.0);
    }

    kinematic_simulation_B.b_kstr_e = switch_expression->size[0] *
      switch_expression->size[1];
    switch_expression->size[0] = 1;
    switch_expression->size[1] = body->JointInternal.Type->size[1];
    kinema_emxEnsureCapacity_char_T(switch_expression,
      kinematic_simulation_B.b_kstr_e);
    kinematic_simulation_B.loop_ub_o = body->JointInternal.Type->size[0] *
      body->JointInternal.Type->size[1] - 1;
    for (kinematic_simulation_B.b_kstr_e = 0; kinematic_simulation_B.b_kstr_e <=
         kinematic_simulation_B.loop_ub_o; kinematic_simulation_B.b_kstr_e++) {
      switch_expression->data[kinematic_simulation_B.b_kstr_e] =
        body->JointInternal.Type->data[kinematic_simulation_B.b_kstr_e];
    }

    kinematic_simulation_B.b_bool_k = false;
    if (switch_expression->size[1] == 5) {
      kinematic_simulation_B.b_kstr_e = 1;
      do {
        exitg1 = 0;
        if (kinematic_simulation_B.b_kstr_e - 1 < 5) {
          kinematic_simulation_B.loop_ub_o = kinematic_simulation_B.b_kstr_e - 1;
          if (switch_expression->data[kinematic_simulation_B.loop_ub_o] !=
              kinematic_simulation_B.b_ff[kinematic_simulation_B.loop_ub_o]) {
            exitg1 = 1;
          } else {
            kinematic_simulation_B.b_kstr_e++;
          }
        } else {
          kinematic_simulation_B.b_bool_k = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (kinematic_simulation_B.b_bool_k) {
      kinematic_simulation_B.b_kstr_e = 0;
    } else {
      for (kinematic_simulation_B.b_kstr_e = 0; kinematic_simulation_B.b_kstr_e <
           8; kinematic_simulation_B.b_kstr_e++) {
        kinematic_simulation_B.b_bn[kinematic_simulation_B.b_kstr_e] =
          tmp_1[kinematic_simulation_B.b_kstr_e];
      }

      kinematic_simulation_B.b_bool_k = false;
      if (switch_expression->size[1] == 8) {
        kinematic_simulation_B.b_kstr_e = 1;
        do {
          exitg1 = 0;
          if (kinematic_simulation_B.b_kstr_e - 1 < 8) {
            kinematic_simulation_B.loop_ub_o = kinematic_simulation_B.b_kstr_e -
              1;
            if (switch_expression->data[kinematic_simulation_B.loop_ub_o] !=
                kinematic_simulation_B.b_bn[kinematic_simulation_B.loop_ub_o]) {
              exitg1 = 1;
            } else {
              kinematic_simulation_B.b_kstr_e++;
            }
          } else {
            kinematic_simulation_B.b_bool_k = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (kinematic_simulation_B.b_bool_k) {
        kinematic_simulation_B.b_kstr_e = 1;
      } else {
        kinematic_simulation_B.b_kstr_e = -1;
      }
    }

    switch (kinematic_simulation_B.b_kstr_e) {
     case 0:
      memset(&kinematic_simulation_B.c_f1[0], 0, sizeof(real_T) << 4U);
      kinematic_simulation_B.c_f1[0] = 1.0;
      kinematic_simulation_B.c_f1[5] = 1.0;
      kinematic_simulation_B.c_f1[10] = 1.0;
      kinematic_simulation_B.c_f1[15] = 1.0;
      break;

     case 1:
      ki_rigidBodyJoint_get_JointAxis(&body->JointInternal,
        kinematic_simulation_B.v);
      kinematic_simulation_B.d_f -= kinematic_simulation_B.e_p;
      for (kinematic_simulation_B.b_kstr_e = 0; kinematic_simulation_B.b_kstr_e <
           kinematic_simulation_B.d_f; kinematic_simulation_B.b_kstr_e++) {
        kinematic_simulation_B.e_data[kinematic_simulation_B.b_kstr_e] =
          kinematic_simulation_B.e_p + kinematic_simulation_B.b_kstr_e;
      }

      kinematic_simulation_B.result_data[0] = kinematic_simulation_B.v[0];
      kinematic_simulation_B.result_data[1] = kinematic_simulation_B.v[1];
      kinematic_simulation_B.result_data[2] = kinematic_simulation_B.v[2];
      if (0 <= (kinematic_simulation_B.d_f != 0) - 1) {
        kinematic_simulation_B.result_data[3] =
          qvec[kinematic_simulation_B.e_data[0]];
      }

      kinematic_simulation_B.k = 1.0 / sqrt((kinematic_simulation_B.result_data
        [0] * kinematic_simulation_B.result_data[0] +
        kinematic_simulation_B.result_data[1] *
        kinematic_simulation_B.result_data[1]) +
        kinematic_simulation_B.result_data[2] *
        kinematic_simulation_B.result_data[2]);
      kinematic_simulation_B.v[0] = kinematic_simulation_B.result_data[0] *
        kinematic_simulation_B.k;
      kinematic_simulation_B.v[1] = kinematic_simulation_B.result_data[1] *
        kinematic_simulation_B.k;
      kinematic_simulation_B.v[2] = kinematic_simulation_B.result_data[2] *
        kinematic_simulation_B.k;
      kinematic_simulation_B.k = cos(kinematic_simulation_B.result_data[3]);
      kinematic_simulation_B.sth = sin(kinematic_simulation_B.result_data[3]);
      kinematic_simulation_B.tempR[0] = kinematic_simulation_B.v[0] *
        kinematic_simulation_B.v[0] * (1.0 - kinematic_simulation_B.k) +
        kinematic_simulation_B.k;
      kinematic_simulation_B.tempR_tmp = kinematic_simulation_B.v[1] *
        kinematic_simulation_B.v[0] * (1.0 - kinematic_simulation_B.k);
      kinematic_simulation_B.tempR_tmp_f = kinematic_simulation_B.v[2] *
        kinematic_simulation_B.sth;
      kinematic_simulation_B.tempR[1] = kinematic_simulation_B.tempR_tmp -
        kinematic_simulation_B.tempR_tmp_f;
      kinematic_simulation_B.tempR_tmp_a = kinematic_simulation_B.v[2] *
        kinematic_simulation_B.v[0] * (1.0 - kinematic_simulation_B.k);
      kinematic_simulation_B.tempR_tmp_j = kinematic_simulation_B.v[1] *
        kinematic_simulation_B.sth;
      kinematic_simulation_B.tempR[2] = kinematic_simulation_B.tempR_tmp_a +
        kinematic_simulation_B.tempR_tmp_j;
      kinematic_simulation_B.tempR[3] = kinematic_simulation_B.tempR_tmp +
        kinematic_simulation_B.tempR_tmp_f;
      kinematic_simulation_B.tempR[4] = kinematic_simulation_B.v[1] *
        kinematic_simulation_B.v[1] * (1.0 - kinematic_simulation_B.k) +
        kinematic_simulation_B.k;
      kinematic_simulation_B.tempR_tmp = kinematic_simulation_B.v[2] *
        kinematic_simulation_B.v[1] * (1.0 - kinematic_simulation_B.k);
      kinematic_simulation_B.tempR_tmp_f = kinematic_simulation_B.v[0] *
        kinematic_simulation_B.sth;
      kinematic_simulation_B.tempR[5] = kinematic_simulation_B.tempR_tmp -
        kinematic_simulation_B.tempR_tmp_f;
      kinematic_simulation_B.tempR[6] = kinematic_simulation_B.tempR_tmp_a -
        kinematic_simulation_B.tempR_tmp_j;
      kinematic_simulation_B.tempR[7] = kinematic_simulation_B.tempR_tmp +
        kinematic_simulation_B.tempR_tmp_f;
      kinematic_simulation_B.tempR[8] = kinematic_simulation_B.v[2] *
        kinematic_simulation_B.v[2] * (1.0 - kinematic_simulation_B.k) +
        kinematic_simulation_B.k;
      for (kinematic_simulation_B.b_kstr_e = 0; kinematic_simulation_B.b_kstr_e <
           3; kinematic_simulation_B.b_kstr_e++) {
        kinematic_simulation_B.e_p = kinematic_simulation_B.b_kstr_e + 1;
        kinematic_simulation_B.R_p[kinematic_simulation_B.e_p - 1] =
          kinematic_simulation_B.tempR[(kinematic_simulation_B.e_p - 1) * 3];
        kinematic_simulation_B.e_p = kinematic_simulation_B.b_kstr_e + 1;
        kinematic_simulation_B.R_p[kinematic_simulation_B.e_p + 2] =
          kinematic_simulation_B.tempR[(kinematic_simulation_B.e_p - 1) * 3 + 1];
        kinematic_simulation_B.e_p = kinematic_simulation_B.b_kstr_e + 1;
        kinematic_simulation_B.R_p[kinematic_simulation_B.e_p + 5] =
          kinematic_simulation_B.tempR[(kinematic_simulation_B.e_p - 1) * 3 + 2];
      }

      memset(&kinematic_simulation_B.c_f1[0], 0, sizeof(real_T) << 4U);
      for (kinematic_simulation_B.b_kstr_e = 0; kinematic_simulation_B.b_kstr_e <
           3; kinematic_simulation_B.b_kstr_e++) {
        kinematic_simulation_B.d_f = kinematic_simulation_B.b_kstr_e << 2;
        kinematic_simulation_B.c_f1[kinematic_simulation_B.d_f] =
          kinematic_simulation_B.R_p[3 * kinematic_simulation_B.b_kstr_e];
        kinematic_simulation_B.c_f1[kinematic_simulation_B.d_f + 1] =
          kinematic_simulation_B.R_p[3 * kinematic_simulation_B.b_kstr_e + 1];
        kinematic_simulation_B.c_f1[kinematic_simulation_B.d_f + 2] =
          kinematic_simulation_B.R_p[3 * kinematic_simulation_B.b_kstr_e + 2];
      }

      kinematic_simulation_B.c_f1[15] = 1.0;
      break;

     default:
      ki_rigidBodyJoint_get_JointAxis(&body->JointInternal,
        kinematic_simulation_B.v);
      memset(&kinematic_simulation_B.tempR[0], 0, 9U * sizeof(real_T));
      kinematic_simulation_B.tempR[0] = 1.0;
      kinematic_simulation_B.tempR[4] = 1.0;
      kinematic_simulation_B.tempR[8] = 1.0;
      for (kinematic_simulation_B.b_kstr_e = 0; kinematic_simulation_B.b_kstr_e <
           3; kinematic_simulation_B.b_kstr_e++) {
        kinematic_simulation_B.d_f = kinematic_simulation_B.b_kstr_e << 2;
        kinematic_simulation_B.c_f1[kinematic_simulation_B.d_f] =
          kinematic_simulation_B.tempR[3 * kinematic_simulation_B.b_kstr_e];
        kinematic_simulation_B.c_f1[kinematic_simulation_B.d_f + 1] =
          kinematic_simulation_B.tempR[3 * kinematic_simulation_B.b_kstr_e + 1];
        kinematic_simulation_B.c_f1[kinematic_simulation_B.d_f + 2] =
          kinematic_simulation_B.tempR[3 * kinematic_simulation_B.b_kstr_e + 2];
        kinematic_simulation_B.c_f1[kinematic_simulation_B.b_kstr_e + 12] =
          kinematic_simulation_B.v[kinematic_simulation_B.b_kstr_e] *
          qvec[kinematic_simulation_B.e_p];
      }

      kinematic_simulation_B.c_f1[3] = 0.0;
      kinematic_simulation_B.c_f1[7] = 0.0;
      kinematic_simulation_B.c_f1[11] = 0.0;
      kinematic_simulation_B.c_f1[15] = 1.0;
      break;
    }

    for (kinematic_simulation_B.b_kstr_e = 0; kinematic_simulation_B.b_kstr_e <
         16; kinematic_simulation_B.b_kstr_e++) {
      kinematic_simulation_B.a[kinematic_simulation_B.b_kstr_e] =
        body->
        JointInternal.JointToParentTransform[kinematic_simulation_B.b_kstr_e];
    }

    for (kinematic_simulation_B.b_kstr_e = 0; kinematic_simulation_B.b_kstr_e <
         16; kinematic_simulation_B.b_kstr_e++) {
      kinematic_simulation_B.b[kinematic_simulation_B.b_kstr_e] =
        body->
        JointInternal.ChildToJointTransform[kinematic_simulation_B.b_kstr_e];
    }

    for (kinematic_simulation_B.b_kstr_e = 0; kinematic_simulation_B.b_kstr_e <
         4; kinematic_simulation_B.b_kstr_e++) {
      for (kinematic_simulation_B.e_p = 0; kinematic_simulation_B.e_p < 4;
           kinematic_simulation_B.e_p++) {
        kinematic_simulation_B.d_f = kinematic_simulation_B.e_p << 2;
        kinematic_simulation_B.loop_ub_o = kinematic_simulation_B.b_kstr_e +
          kinematic_simulation_B.d_f;
        kinematic_simulation_B.a_c[kinematic_simulation_B.loop_ub_o] = 0.0;
        kinematic_simulation_B.a_c[kinematic_simulation_B.loop_ub_o] +=
          kinematic_simulation_B.c_f1[kinematic_simulation_B.d_f] *
          kinematic_simulation_B.a[kinematic_simulation_B.b_kstr_e];
        kinematic_simulation_B.a_c[kinematic_simulation_B.loop_ub_o] +=
          kinematic_simulation_B.c_f1[kinematic_simulation_B.d_f + 1] *
          kinematic_simulation_B.a[kinematic_simulation_B.b_kstr_e + 4];
        kinematic_simulation_B.a_c[kinematic_simulation_B.loop_ub_o] +=
          kinematic_simulation_B.c_f1[kinematic_simulation_B.d_f + 2] *
          kinematic_simulation_B.a[kinematic_simulation_B.b_kstr_e + 8];
        kinematic_simulation_B.a_c[kinematic_simulation_B.loop_ub_o] +=
          kinematic_simulation_B.c_f1[kinematic_simulation_B.d_f + 3] *
          kinematic_simulation_B.a[kinematic_simulation_B.b_kstr_e + 12];
      }

      for (kinematic_simulation_B.e_p = 0; kinematic_simulation_B.e_p < 4;
           kinematic_simulation_B.e_p++) {
        kinematic_simulation_B.d_f = kinematic_simulation_B.e_p << 2;
        kinematic_simulation_B.loop_ub_o = kinematic_simulation_B.b_kstr_e +
          kinematic_simulation_B.d_f;
        Ttree->data[kinematic_simulation_B.b_jtilecol]
          .f1[kinematic_simulation_B.loop_ub_o] = 0.0;
        Ttree->data[kinematic_simulation_B.b_jtilecol]
          .f1[kinematic_simulation_B.loop_ub_o] +=
          kinematic_simulation_B.b[kinematic_simulation_B.d_f] *
          kinematic_simulation_B.a_c[kinematic_simulation_B.b_kstr_e];
        Ttree->data[kinematic_simulation_B.b_jtilecol]
          .f1[kinematic_simulation_B.loop_ub_o] +=
          kinematic_simulation_B.b[kinematic_simulation_B.d_f + 1] *
          kinematic_simulation_B.a_c[kinematic_simulation_B.b_kstr_e + 4];
        Ttree->data[kinematic_simulation_B.b_jtilecol]
          .f1[kinematic_simulation_B.loop_ub_o] +=
          kinematic_simulation_B.b[kinematic_simulation_B.d_f + 2] *
          kinematic_simulation_B.a_c[kinematic_simulation_B.b_kstr_e + 8];
        Ttree->data[kinematic_simulation_B.b_jtilecol]
          .f1[kinematic_simulation_B.loop_ub_o] +=
          kinematic_simulation_B.b[kinematic_simulation_B.d_f + 3] *
          kinematic_simulation_B.a_c[kinematic_simulation_B.b_kstr_e + 12];
      }
    }

    kinematic_simulation_B.k = kinematic_simulation_B.n;
    if (body->ParentIndex > 0.0) {
      for (kinematic_simulation_B.b_kstr_e = 0; kinematic_simulation_B.b_kstr_e <
           16; kinematic_simulation_B.b_kstr_e++) {
        kinematic_simulation_B.a[kinematic_simulation_B.b_kstr_e] = Ttree->data[
          static_cast<int32_T>(body->ParentIndex) - 1]
          .f1[kinematic_simulation_B.b_kstr_e];
      }

      for (kinematic_simulation_B.b_kstr_e = 0; kinematic_simulation_B.b_kstr_e <
           4; kinematic_simulation_B.b_kstr_e++) {
        for (kinematic_simulation_B.e_p = 0; kinematic_simulation_B.e_p < 4;
             kinematic_simulation_B.e_p++) {
          kinematic_simulation_B.d_f = kinematic_simulation_B.e_p << 2;
          kinematic_simulation_B.loop_ub_o = kinematic_simulation_B.b_kstr_e +
            kinematic_simulation_B.d_f;
          kinematic_simulation_B.a_c[kinematic_simulation_B.loop_ub_o] = 0.0;
          kinematic_simulation_B.a_c[kinematic_simulation_B.loop_ub_o] +=
            Ttree->data[kinematic_simulation_B.b_jtilecol]
            .f1[kinematic_simulation_B.d_f] *
            kinematic_simulation_B.a[kinematic_simulation_B.b_kstr_e];
          kinematic_simulation_B.a_c[kinematic_simulation_B.loop_ub_o] +=
            Ttree->data[kinematic_simulation_B.b_jtilecol]
            .f1[kinematic_simulation_B.d_f + 1] *
            kinematic_simulation_B.a[kinematic_simulation_B.b_kstr_e + 4];
          kinematic_simulation_B.a_c[kinematic_simulation_B.loop_ub_o] +=
            Ttree->data[kinematic_simulation_B.b_jtilecol]
            .f1[kinematic_simulation_B.d_f + 2] *
            kinematic_simulation_B.a[kinematic_simulation_B.b_kstr_e + 8];
          kinematic_simulation_B.a_c[kinematic_simulation_B.loop_ub_o] +=
            Ttree->data[kinematic_simulation_B.b_jtilecol]
            .f1[kinematic_simulation_B.d_f + 3] *
            kinematic_simulation_B.a[kinematic_simulation_B.b_kstr_e + 12];
        }
      }

      for (kinematic_simulation_B.b_kstr_e = 0; kinematic_simulation_B.b_kstr_e <
           16; kinematic_simulation_B.b_kstr_e++) {
        Ttree->data[kinematic_simulation_B.b_jtilecol]
          .f1[kinematic_simulation_B.b_kstr_e] =
          kinematic_simulation_B.a_c[kinematic_simulation_B.b_kstr_e];
      }
    }
  }

  kinematic_simula_emxFree_char_T(&switch_expression);
}

static void kinema_emxEnsureCapacity_real_T(emxArray_real_T_kinematic_sim_T
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

static void kinematic_simula_emxFree_real_T(emxArray_real_T_kinematic_sim_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T_kinematic_sim_T *)NULL) {
    if (((*pEmxArray)->data != (real_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_real_T_kinematic_sim_T *)NULL;
  }
}

static void kinematic_s_emxFree_f_cell_wrap(emxArray_f_cell_wrap_kinemati_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_f_cell_wrap_kinemati_T *)NULL) {
    if (((*pEmxArray)->data != (f_cell_wrap_kinematic_simulat_T *)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_f_cell_wrap_kinemati_T *)NULL;
  }
}

static void RigidBodyTree_geometricJacobian(p_robotics_manip_internal_Rig_T *obj,
  const real_T Q[6], emxArray_real_T_kinematic_sim_T *Jac)
{
  emxArray_f_cell_wrap_kinemati_T *Ttree;
  n_robotics_manip_internal_Rig_T *body;
  emxArray_real_T_kinematic_sim_T *JacSlice;
  emxArray_char_T_kinematic_sim_T *bname;
  emxArray_real_T_kinematic_sim_T *b;
  static const char_T tmp[11] = { 'e', 'd', 'o', '_', 'l', 'i', 'n', 'k', '_',
    'e', 'e' };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  int32_T exitg1;
  boolean_T exitg2;
  kinematic_s_emxInit_f_cell_wrap(&Ttree, 2);
  RigidBodyTree_forwardKinematics(obj, Q, Ttree);
  kinematic_simulation_B.b_kstr_m = Jac->size[0] * Jac->size[1];
  Jac->size[0] = 6;
  Jac->size[1] = static_cast<int32_T>(obj->VelocityNumber);
  kinema_emxEnsureCapacity_real_T(Jac, kinematic_simulation_B.b_kstr_m);
  kinematic_simulation_B.loop_ub_c = 6 * static_cast<int32_T>
    (obj->VelocityNumber) - 1;
  for (kinematic_simulation_B.b_kstr_m = 0; kinematic_simulation_B.b_kstr_m <=
       kinematic_simulation_B.loop_ub_c; kinematic_simulation_B.b_kstr_m++) {
    Jac->data[kinematic_simulation_B.b_kstr_m] = 0.0;
  }

  for (kinematic_simulation_B.b_kstr_m = 0; kinematic_simulation_B.b_kstr_m < 8;
       kinematic_simulation_B.b_kstr_m++) {
    kinematic_simulation_B.chainmask[kinematic_simulation_B.b_kstr_m] = 0;
  }

  kinematic_simula_emxInit_char_T(&bname, 2);
  kinematic_simulation_B.b_kstr_m = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj->Base.NameInternal->size[1];
  kinema_emxEnsureCapacity_char_T(bname, kinematic_simulation_B.b_kstr_m);
  kinematic_simulation_B.loop_ub_c = obj->Base.NameInternal->size[0] *
    obj->Base.NameInternal->size[1] - 1;
  for (kinematic_simulation_B.b_kstr_m = 0; kinematic_simulation_B.b_kstr_m <=
       kinematic_simulation_B.loop_ub_c; kinematic_simulation_B.b_kstr_m++) {
    bname->data[kinematic_simulation_B.b_kstr_m] = obj->Base.NameInternal->
      data[kinematic_simulation_B.b_kstr_m];
  }

  for (kinematic_simulation_B.b_kstr_m = 0; kinematic_simulation_B.b_kstr_m < 11;
       kinematic_simulation_B.b_kstr_m++) {
    kinematic_simulation_B.a_n[kinematic_simulation_B.b_kstr_m] =
      tmp[kinematic_simulation_B.b_kstr_m];
  }

  kinematic_simulation_B.b_bool_o = false;
  if (11 == bname->size[1]) {
    kinematic_simulation_B.b_kstr_m = 1;
    do {
      exitg1 = 0;
      if (kinematic_simulation_B.b_kstr_m - 1 < 11) {
        kinematic_simulation_B.kstr = kinematic_simulation_B.b_kstr_m - 1;
        if (kinematic_simulation_B.a_n[kinematic_simulation_B.kstr] !=
            bname->data[kinematic_simulation_B.kstr]) {
          exitg1 = 1;
        } else {
          kinematic_simulation_B.b_kstr_m++;
        }
      } else {
        kinematic_simulation_B.b_bool_o = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (kinematic_simulation_B.b_bool_o) {
    memset(&kinematic_simulation_B.T2inv[0], 0, sizeof(real_T) << 4U);
    kinematic_simulation_B.T2inv[0] = 1.0;
    kinematic_simulation_B.T2inv[5] = 1.0;
    kinematic_simulation_B.T2inv[10] = 1.0;
    kinematic_simulation_B.T2inv[15] = 1.0;
    memset(&kinematic_simulation_B.T2_c[0], 0, sizeof(real_T) << 4U);
    kinematic_simulation_B.T2_c[0] = 1.0;
    kinematic_simulation_B.T2_c[5] = 1.0;
    kinematic_simulation_B.T2_c[10] = 1.0;
    kinematic_simulation_B.T2_c[15] = 1.0;
  } else {
    kinematic_simulation_B.endeffectorIndex = -1.0;
    kinematic_simulation_B.b_kstr_m = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = obj->Base.NameInternal->size[1];
    kinema_emxEnsureCapacity_char_T(bname, kinematic_simulation_B.b_kstr_m);
    kinematic_simulation_B.loop_ub_c = obj->Base.NameInternal->size[0] *
      obj->Base.NameInternal->size[1] - 1;
    for (kinematic_simulation_B.b_kstr_m = 0; kinematic_simulation_B.b_kstr_m <=
         kinematic_simulation_B.loop_ub_c; kinematic_simulation_B.b_kstr_m++) {
      bname->data[kinematic_simulation_B.b_kstr_m] = obj->
        Base.NameInternal->data[kinematic_simulation_B.b_kstr_m];
    }

    for (kinematic_simulation_B.b_kstr_m = 0; kinematic_simulation_B.b_kstr_m <
         11; kinematic_simulation_B.b_kstr_m++) {
      kinematic_simulation_B.a_n[kinematic_simulation_B.b_kstr_m] =
        tmp[kinematic_simulation_B.b_kstr_m];
    }

    kinematic_simulation_B.b_bool_o = false;
    if (bname->size[1] == 11) {
      kinematic_simulation_B.b_kstr_m = 1;
      do {
        exitg1 = 0;
        if (kinematic_simulation_B.b_kstr_m - 1 < 11) {
          kinematic_simulation_B.kstr = kinematic_simulation_B.b_kstr_m - 1;
          if (bname->data[kinematic_simulation_B.kstr] !=
              kinematic_simulation_B.a_n[kinematic_simulation_B.kstr]) {
            exitg1 = 1;
          } else {
            kinematic_simulation_B.b_kstr_m++;
          }
        } else {
          kinematic_simulation_B.b_bool_o = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (kinematic_simulation_B.b_bool_o) {
      kinematic_simulation_B.endeffectorIndex = 0.0;
    } else {
      kinematic_simulation_B.idx_idx_1 = obj->NumBodies;
      kinematic_simulation_B.b_i_l = 0;
      exitg2 = false;
      while ((!exitg2) && (kinematic_simulation_B.b_i_l <= static_cast<int32_T>
                           (kinematic_simulation_B.idx_idx_1) - 1)) {
        body = obj->Bodies[kinematic_simulation_B.b_i_l];
        kinematic_simulation_B.b_kstr_m = bname->size[0] * bname->size[1];
        bname->size[0] = 1;
        bname->size[1] = body->NameInternal->size[1];
        kinema_emxEnsureCapacity_char_T(bname, kinematic_simulation_B.b_kstr_m);
        kinematic_simulation_B.loop_ub_c = body->NameInternal->size[0] *
          body->NameInternal->size[1] - 1;
        for (kinematic_simulation_B.b_kstr_m = 0;
             kinematic_simulation_B.b_kstr_m <= kinematic_simulation_B.loop_ub_c;
             kinematic_simulation_B.b_kstr_m++) {
          bname->data[kinematic_simulation_B.b_kstr_m] = body->
            NameInternal->data[kinematic_simulation_B.b_kstr_m];
        }

        for (kinematic_simulation_B.b_kstr_m = 0;
             kinematic_simulation_B.b_kstr_m < 11;
             kinematic_simulation_B.b_kstr_m++) {
          kinematic_simulation_B.a_n[kinematic_simulation_B.b_kstr_m] =
            tmp[kinematic_simulation_B.b_kstr_m];
        }

        kinematic_simulation_B.b_bool_o = false;
        if (bname->size[1] == 11) {
          kinematic_simulation_B.b_kstr_m = 1;
          do {
            exitg1 = 0;
            if (kinematic_simulation_B.b_kstr_m - 1 < 11) {
              kinematic_simulation_B.kstr = kinematic_simulation_B.b_kstr_m - 1;
              if (bname->data[kinematic_simulation_B.kstr] !=
                  kinematic_simulation_B.a_n[kinematic_simulation_B.kstr]) {
                exitg1 = 1;
              } else {
                kinematic_simulation_B.b_kstr_m++;
              }
            } else {
              kinematic_simulation_B.b_bool_o = true;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }

        if (kinematic_simulation_B.b_bool_o) {
          kinematic_simulation_B.endeffectorIndex = static_cast<real_T>
            (kinematic_simulation_B.b_i_l) + 1.0;
          exitg2 = true;
        } else {
          kinematic_simulation_B.b_i_l++;
        }
      }
    }

    kinematic_simulation_B.b_i_l = static_cast<int32_T>
      (kinematic_simulation_B.endeffectorIndex) - 1;
    body = obj->Bodies[kinematic_simulation_B.b_i_l];
    for (kinematic_simulation_B.b_kstr_m = 0; kinematic_simulation_B.b_kstr_m <
         16; kinematic_simulation_B.b_kstr_m++) {
      kinematic_simulation_B.T2_c[kinematic_simulation_B.b_kstr_m] = Ttree->
        data[kinematic_simulation_B.b_i_l].f1[kinematic_simulation_B.b_kstr_m];
    }

    for (kinematic_simulation_B.b_kstr_m = 0; kinematic_simulation_B.b_kstr_m <
         3; kinematic_simulation_B.b_kstr_m++) {
      kinematic_simulation_B.R_m[3 * kinematic_simulation_B.b_kstr_m] =
        Ttree->data[kinematic_simulation_B.b_i_l]
        .f1[kinematic_simulation_B.b_kstr_m];
      kinematic_simulation_B.R_m[3 * kinematic_simulation_B.b_kstr_m + 1] =
        Ttree->data[kinematic_simulation_B.b_i_l]
        .f1[kinematic_simulation_B.b_kstr_m + 4];
      kinematic_simulation_B.R_m[3 * kinematic_simulation_B.b_kstr_m + 2] =
        Ttree->data[kinematic_simulation_B.b_i_l]
        .f1[kinematic_simulation_B.b_kstr_m + 8];
    }

    for (kinematic_simulation_B.b_kstr_m = 0; kinematic_simulation_B.b_kstr_m <
         9; kinematic_simulation_B.b_kstr_m++) {
      kinematic_simulation_B.R_n[kinematic_simulation_B.b_kstr_m] =
        -kinematic_simulation_B.R_m[kinematic_simulation_B.b_kstr_m];
    }

    for (kinematic_simulation_B.b_kstr_m = 0; kinematic_simulation_B.b_kstr_m <
         3; kinematic_simulation_B.b_kstr_m++) {
      kinematic_simulation_B.endeffectorIndex = Ttree->
        data[kinematic_simulation_B.b_i_l].f1[12] *
        kinematic_simulation_B.R_n[kinematic_simulation_B.b_kstr_m];
      kinematic_simulation_B.loop_ub_c = kinematic_simulation_B.b_kstr_m << 2;
      kinematic_simulation_B.T2inv[kinematic_simulation_B.loop_ub_c] =
        kinematic_simulation_B.R_m[3 * kinematic_simulation_B.b_kstr_m];
      kinematic_simulation_B.endeffectorIndex +=
        kinematic_simulation_B.R_n[kinematic_simulation_B.b_kstr_m + 3] *
        Ttree->data[kinematic_simulation_B.b_i_l].f1[13];
      kinematic_simulation_B.T2inv[kinematic_simulation_B.loop_ub_c + 1] =
        kinematic_simulation_B.R_m[3 * kinematic_simulation_B.b_kstr_m + 1];
      kinematic_simulation_B.endeffectorIndex +=
        kinematic_simulation_B.R_n[kinematic_simulation_B.b_kstr_m + 6] *
        Ttree->data[kinematic_simulation_B.b_i_l].f1[14];
      kinematic_simulation_B.T2inv[kinematic_simulation_B.loop_ub_c + 2] =
        kinematic_simulation_B.R_m[3 * kinematic_simulation_B.b_kstr_m + 2];
      kinematic_simulation_B.T2inv[kinematic_simulation_B.b_kstr_m + 12] =
        kinematic_simulation_B.endeffectorIndex;
    }

    kinematic_simulation_B.T2inv[3] = 0.0;
    kinematic_simulation_B.T2inv[7] = 0.0;
    kinematic_simulation_B.T2inv[11] = 0.0;
    kinematic_simulation_B.T2inv[15] = 1.0;
    kinematic_simulation_B.chainmask[kinematic_simulation_B.b_i_l] = 1;
    while (body->ParentIndex > 0.0) {
      body = obj->Bodies[static_cast<int32_T>(body->ParentIndex) - 1];
      kinematic_simulation_B.chainmask[static_cast<int32_T>(body->Index) - 1] =
        1;
    }
  }

  kinematic_simulation_B.idx_idx_1 = obj->NumBodies;
  kinematic_simulation_B.c_o = static_cast<int32_T>
    (kinematic_simulation_B.idx_idx_1) - 1;
  kinematic_simula_emxInit_real_T(&JacSlice, 2);
  kinematic_simula_emxInit_real_T(&b, 2);
  if (0 <= kinematic_simulation_B.c_o) {
    for (kinematic_simulation_B.b_kstr_m = 0; kinematic_simulation_B.b_kstr_m <
         5; kinematic_simulation_B.b_kstr_m++) {
      kinematic_simulation_B.b_i[kinematic_simulation_B.b_kstr_m] =
        tmp_0[kinematic_simulation_B.b_kstr_m];
    }
  }

  for (kinematic_simulation_B.b_i_l = 0; kinematic_simulation_B.b_i_l <=
       kinematic_simulation_B.c_o; kinematic_simulation_B.b_i_l++) {
    body = obj->Bodies[kinematic_simulation_B.b_i_l];
    kinematic_simulation_B.b_kstr_m = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = body->JointInternal.Type->size[1];
    kinema_emxEnsureCapacity_char_T(bname, kinematic_simulation_B.b_kstr_m);
    kinematic_simulation_B.loop_ub_c = body->JointInternal.Type->size[0] *
      body->JointInternal.Type->size[1] - 1;
    for (kinematic_simulation_B.b_kstr_m = 0; kinematic_simulation_B.b_kstr_m <=
         kinematic_simulation_B.loop_ub_c; kinematic_simulation_B.b_kstr_m++) {
      bname->data[kinematic_simulation_B.b_kstr_m] = body->
        JointInternal.Type->data[kinematic_simulation_B.b_kstr_m];
    }

    kinematic_simulation_B.b_bool_o = false;
    if (bname->size[1] == 5) {
      kinematic_simulation_B.b_kstr_m = 1;
      do {
        exitg1 = 0;
        if (kinematic_simulation_B.b_kstr_m - 1 < 5) {
          kinematic_simulation_B.kstr = kinematic_simulation_B.b_kstr_m - 1;
          if (bname->data[kinematic_simulation_B.kstr] !=
              kinematic_simulation_B.b_i[kinematic_simulation_B.kstr]) {
            exitg1 = 1;
          } else {
            kinematic_simulation_B.b_kstr_m++;
          }
        } else {
          kinematic_simulation_B.b_bool_o = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if ((!kinematic_simulation_B.b_bool_o) &&
        (kinematic_simulation_B.chainmask[kinematic_simulation_B.b_i_l] != 0)) {
      for (kinematic_simulation_B.b_kstr_m = 0; kinematic_simulation_B.b_kstr_m <
           16; kinematic_simulation_B.b_kstr_m++) {
        kinematic_simulation_B.T1_k[kinematic_simulation_B.b_kstr_m] =
          Ttree->data[static_cast<int32_T>(body->Index) - 1]
          .f1[kinematic_simulation_B.b_kstr_m];
      }

      for (kinematic_simulation_B.b_kstr_m = 0; kinematic_simulation_B.b_kstr_m <
           16; kinematic_simulation_B.b_kstr_m++) {
        kinematic_simulation_B.Tdh[kinematic_simulation_B.b_kstr_m] =
          body->
          JointInternal.ChildToJointTransform[kinematic_simulation_B.b_kstr_m];
      }

      for (kinematic_simulation_B.b_kstr_m = 0; kinematic_simulation_B.b_kstr_m <
           3; kinematic_simulation_B.b_kstr_m++) {
        kinematic_simulation_B.R_m[3 * kinematic_simulation_B.b_kstr_m] =
          kinematic_simulation_B.Tdh[kinematic_simulation_B.b_kstr_m];
        kinematic_simulation_B.R_m[3 * kinematic_simulation_B.b_kstr_m + 1] =
          kinematic_simulation_B.Tdh[kinematic_simulation_B.b_kstr_m + 4];
        kinematic_simulation_B.R_m[3 * kinematic_simulation_B.b_kstr_m + 2] =
          kinematic_simulation_B.Tdh[kinematic_simulation_B.b_kstr_m + 8];
      }

      for (kinematic_simulation_B.b_kstr_m = 0; kinematic_simulation_B.b_kstr_m <
           9; kinematic_simulation_B.b_kstr_m++) {
        kinematic_simulation_B.R_n[kinematic_simulation_B.b_kstr_m] =
          -kinematic_simulation_B.R_m[kinematic_simulation_B.b_kstr_m];
      }

      for (kinematic_simulation_B.b_kstr_m = 0; kinematic_simulation_B.b_kstr_m <
           3; kinematic_simulation_B.b_kstr_m++) {
        kinematic_simulation_B.R_d[kinematic_simulation_B.b_kstr_m] =
          kinematic_simulation_B.R_n[kinematic_simulation_B.b_kstr_m + 6] *
          kinematic_simulation_B.Tdh[14] +
          (kinematic_simulation_B.R_n[kinematic_simulation_B.b_kstr_m + 3] *
           kinematic_simulation_B.Tdh[13] +
           kinematic_simulation_B.R_n[kinematic_simulation_B.b_kstr_m] *
           kinematic_simulation_B.Tdh[12]);
      }

      for (kinematic_simulation_B.b_kstr_m = 0; kinematic_simulation_B.b_kstr_m <
           4; kinematic_simulation_B.b_kstr_m++) {
        for (kinematic_simulation_B.kstr = 0; kinematic_simulation_B.kstr < 4;
             kinematic_simulation_B.kstr++) {
          kinematic_simulation_B.n_m = kinematic_simulation_B.kstr << 2;
          kinematic_simulation_B.loop_ub_c = kinematic_simulation_B.b_kstr_m +
            kinematic_simulation_B.n_m;
          kinematic_simulation_B.Tdh[kinematic_simulation_B.loop_ub_c] = 0.0;
          kinematic_simulation_B.Tdh[kinematic_simulation_B.loop_ub_c] +=
            kinematic_simulation_B.T1_k[kinematic_simulation_B.n_m] *
            kinematic_simulation_B.T2inv[kinematic_simulation_B.b_kstr_m];
          kinematic_simulation_B.Tdh[kinematic_simulation_B.loop_ub_c] +=
            kinematic_simulation_B.T1_k[kinematic_simulation_B.n_m + 1] *
            kinematic_simulation_B.T2inv[kinematic_simulation_B.b_kstr_m + 4];
          kinematic_simulation_B.Tdh[kinematic_simulation_B.loop_ub_c] +=
            kinematic_simulation_B.T1_k[kinematic_simulation_B.n_m + 2] *
            kinematic_simulation_B.T2inv[kinematic_simulation_B.b_kstr_m + 8];
          kinematic_simulation_B.Tdh[kinematic_simulation_B.loop_ub_c] +=
            kinematic_simulation_B.T1_k[kinematic_simulation_B.n_m + 3] *
            kinematic_simulation_B.T2inv[kinematic_simulation_B.b_kstr_m + 12];
        }
      }

      for (kinematic_simulation_B.b_kstr_m = 0; kinematic_simulation_B.b_kstr_m <
           3; kinematic_simulation_B.b_kstr_m++) {
        kinematic_simulation_B.kstr = kinematic_simulation_B.b_kstr_m << 2;
        kinematic_simulation_B.T1_k[kinematic_simulation_B.kstr] =
          kinematic_simulation_B.R_m[3 * kinematic_simulation_B.b_kstr_m];
        kinematic_simulation_B.T1_k[kinematic_simulation_B.kstr + 1] =
          kinematic_simulation_B.R_m[3 * kinematic_simulation_B.b_kstr_m + 1];
        kinematic_simulation_B.T1_k[kinematic_simulation_B.kstr + 2] =
          kinematic_simulation_B.R_m[3 * kinematic_simulation_B.b_kstr_m + 2];
        kinematic_simulation_B.T1_k[kinematic_simulation_B.b_kstr_m + 12] =
          kinematic_simulation_B.R_d[kinematic_simulation_B.b_kstr_m];
      }

      kinematic_simulation_B.T1_k[3] = 0.0;
      kinematic_simulation_B.T1_k[7] = 0.0;
      kinematic_simulation_B.T1_k[11] = 0.0;
      kinematic_simulation_B.T1_k[15] = 1.0;
      for (kinematic_simulation_B.b_kstr_m = 0; kinematic_simulation_B.b_kstr_m <
           4; kinematic_simulation_B.b_kstr_m++) {
        for (kinematic_simulation_B.kstr = 0; kinematic_simulation_B.kstr < 4;
             kinematic_simulation_B.kstr++) {
          kinematic_simulation_B.loop_ub_c = kinematic_simulation_B.kstr << 2;
          kinematic_simulation_B.n_m = kinematic_simulation_B.b_kstr_m +
            kinematic_simulation_B.loop_ub_c;
          kinematic_simulation_B.T[kinematic_simulation_B.n_m] = 0.0;
          kinematic_simulation_B.T[kinematic_simulation_B.n_m] +=
            kinematic_simulation_B.T1_k[kinematic_simulation_B.loop_ub_c] *
            kinematic_simulation_B.Tdh[kinematic_simulation_B.b_kstr_m];
          kinematic_simulation_B.T[kinematic_simulation_B.n_m] +=
            kinematic_simulation_B.T1_k[kinematic_simulation_B.loop_ub_c + 1] *
            kinematic_simulation_B.Tdh[kinematic_simulation_B.b_kstr_m + 4];
          kinematic_simulation_B.T[kinematic_simulation_B.n_m] +=
            kinematic_simulation_B.T1_k[kinematic_simulation_B.loop_ub_c + 2] *
            kinematic_simulation_B.Tdh[kinematic_simulation_B.b_kstr_m + 8];
          kinematic_simulation_B.T[kinematic_simulation_B.n_m] +=
            kinematic_simulation_B.T1_k[kinematic_simulation_B.loop_ub_c + 3] *
            kinematic_simulation_B.Tdh[kinematic_simulation_B.b_kstr_m + 12];
        }
      }

      kinematic_simulation_B.endeffectorIndex = obj->
        PositionDoFMap[kinematic_simulation_B.b_i_l];
      kinematic_simulation_B.idx_idx_1 = obj->
        PositionDoFMap[kinematic_simulation_B.b_i_l + 8];
      kinematic_simulation_B.R_m[0] = 0.0;
      kinematic_simulation_B.R_m[3] = -kinematic_simulation_B.T[14];
      kinematic_simulation_B.R_m[6] = kinematic_simulation_B.T[13];
      kinematic_simulation_B.R_m[1] = kinematic_simulation_B.T[14];
      kinematic_simulation_B.R_m[4] = 0.0;
      kinematic_simulation_B.R_m[7] = -kinematic_simulation_B.T[12];
      kinematic_simulation_B.R_m[2] = -kinematic_simulation_B.T[13];
      kinematic_simulation_B.R_m[5] = kinematic_simulation_B.T[12];
      kinematic_simulation_B.R_m[8] = 0.0;
      for (kinematic_simulation_B.b_kstr_m = 0; kinematic_simulation_B.b_kstr_m <
           3; kinematic_simulation_B.b_kstr_m++) {
        for (kinematic_simulation_B.kstr = 0; kinematic_simulation_B.kstr < 3;
             kinematic_simulation_B.kstr++) {
          kinematic_simulation_B.loop_ub_c = kinematic_simulation_B.b_kstr_m + 3
            * kinematic_simulation_B.kstr;
          kinematic_simulation_B.R_n[kinematic_simulation_B.loop_ub_c] = 0.0;
          kinematic_simulation_B.n_m = kinematic_simulation_B.kstr << 2;
          kinematic_simulation_B.R_n[kinematic_simulation_B.loop_ub_c] +=
            kinematic_simulation_B.T[kinematic_simulation_B.n_m] *
            kinematic_simulation_B.R_m[kinematic_simulation_B.b_kstr_m];
          kinematic_simulation_B.R_n[kinematic_simulation_B.loop_ub_c] +=
            kinematic_simulation_B.T[kinematic_simulation_B.n_m + 1] *
            kinematic_simulation_B.R_m[kinematic_simulation_B.b_kstr_m + 3];
          kinematic_simulation_B.R_n[kinematic_simulation_B.loop_ub_c] +=
            kinematic_simulation_B.T[kinematic_simulation_B.n_m + 2] *
            kinematic_simulation_B.R_m[kinematic_simulation_B.b_kstr_m + 6];
          kinematic_simulation_B.X[kinematic_simulation_B.kstr + 6 *
            kinematic_simulation_B.b_kstr_m] = kinematic_simulation_B.T
            [(kinematic_simulation_B.b_kstr_m << 2) +
            kinematic_simulation_B.kstr];
          kinematic_simulation_B.X[kinematic_simulation_B.kstr + 6 *
            (kinematic_simulation_B.b_kstr_m + 3)] = 0.0;
        }
      }

      for (kinematic_simulation_B.b_kstr_m = 0; kinematic_simulation_B.b_kstr_m <
           3; kinematic_simulation_B.b_kstr_m++) {
        kinematic_simulation_B.X[6 * kinematic_simulation_B.b_kstr_m + 3] =
          kinematic_simulation_B.R_n[3 * kinematic_simulation_B.b_kstr_m];
        kinematic_simulation_B.kstr = kinematic_simulation_B.b_kstr_m << 2;
        kinematic_simulation_B.loop_ub_c = 6 * (kinematic_simulation_B.b_kstr_m
          + 3);
        kinematic_simulation_B.X[kinematic_simulation_B.loop_ub_c + 3] =
          kinematic_simulation_B.T[kinematic_simulation_B.kstr];
        kinematic_simulation_B.X[6 * kinematic_simulation_B.b_kstr_m + 4] =
          kinematic_simulation_B.R_n[3 * kinematic_simulation_B.b_kstr_m + 1];
        kinematic_simulation_B.X[kinematic_simulation_B.loop_ub_c + 4] =
          kinematic_simulation_B.T[kinematic_simulation_B.kstr + 1];
        kinematic_simulation_B.X[6 * kinematic_simulation_B.b_kstr_m + 5] =
          kinematic_simulation_B.R_n[3 * kinematic_simulation_B.b_kstr_m + 2];
        kinematic_simulation_B.X[kinematic_simulation_B.loop_ub_c + 5] =
          kinematic_simulation_B.T[kinematic_simulation_B.kstr + 2];
      }

      kinematic_simulation_B.b_kstr_m = b->size[0] * b->size[1];
      b->size[0] = 6;
      b->size[1] = body->JointInternal.MotionSubspace->size[1];
      kinema_emxEnsureCapacity_real_T(b, kinematic_simulation_B.b_kstr_m);
      kinematic_simulation_B.loop_ub_c = body->
        JointInternal.MotionSubspace->size[0] *
        body->JointInternal.MotionSubspace->size[1] - 1;
      for (kinematic_simulation_B.b_kstr_m = 0; kinematic_simulation_B.b_kstr_m <=
           kinematic_simulation_B.loop_ub_c; kinematic_simulation_B.b_kstr_m++)
      {
        b->data[kinematic_simulation_B.b_kstr_m] =
          body->JointInternal.MotionSubspace->
          data[kinematic_simulation_B.b_kstr_m];
      }

      kinematic_simulation_B.n_m = b->size[1] - 1;
      kinematic_simulation_B.b_kstr_m = JacSlice->size[0] * JacSlice->size[1];
      JacSlice->size[0] = 6;
      JacSlice->size[1] = b->size[1];
      kinema_emxEnsureCapacity_real_T(JacSlice, kinematic_simulation_B.b_kstr_m);
      for (kinematic_simulation_B.b_kstr_m = 0; kinematic_simulation_B.b_kstr_m <=
           kinematic_simulation_B.n_m; kinematic_simulation_B.b_kstr_m++) {
        kinematic_simulation_B.coffset_tmp = kinematic_simulation_B.b_kstr_m * 6
          - 1;
        for (kinematic_simulation_B.kstr = 0; kinematic_simulation_B.kstr < 6;
             kinematic_simulation_B.kstr++) {
          kinematic_simulation_B.s_j = 0.0;
          for (kinematic_simulation_B.loop_ub_c = 0;
               kinematic_simulation_B.loop_ub_c < 6;
               kinematic_simulation_B.loop_ub_c++) {
            kinematic_simulation_B.s_j +=
              kinematic_simulation_B.X[kinematic_simulation_B.loop_ub_c * 6 +
              kinematic_simulation_B.kstr] * b->data
              [(kinematic_simulation_B.coffset_tmp +
                kinematic_simulation_B.loop_ub_c) + 1];
          }

          JacSlice->data[(kinematic_simulation_B.coffset_tmp +
                          kinematic_simulation_B.kstr) + 1] =
            kinematic_simulation_B.s_j;
        }
      }

      if (kinematic_simulation_B.endeffectorIndex >
          kinematic_simulation_B.idx_idx_1) {
        kinematic_simulation_B.n_m = 0;
      } else {
        kinematic_simulation_B.n_m = static_cast<int32_T>
          (kinematic_simulation_B.endeffectorIndex) - 1;
      }

      kinematic_simulation_B.loop_ub_c = JacSlice->size[1];
      for (kinematic_simulation_B.b_kstr_m = 0; kinematic_simulation_B.b_kstr_m <
           kinematic_simulation_B.loop_ub_c; kinematic_simulation_B.b_kstr_m++)
      {
        for (kinematic_simulation_B.kstr = 0; kinematic_simulation_B.kstr < 6;
             kinematic_simulation_B.kstr++) {
          Jac->data[kinematic_simulation_B.kstr + 6 *
            (kinematic_simulation_B.n_m + kinematic_simulation_B.b_kstr_m)] =
            JacSlice->data[6 * kinematic_simulation_B.b_kstr_m +
            kinematic_simulation_B.kstr];
        }
      }
    }
  }

  kinematic_simula_emxFree_char_T(&bname);
  kinematic_simula_emxFree_real_T(&JacSlice);
  kinematic_s_emxFree_f_cell_wrap(&Ttree);
  for (kinematic_simulation_B.b_kstr_m = 0; kinematic_simulation_B.b_kstr_m < 3;
       kinematic_simulation_B.b_kstr_m++) {
    kinematic_simulation_B.b_i_l = kinematic_simulation_B.b_kstr_m << 2;
    kinematic_simulation_B.X[6 * kinematic_simulation_B.b_kstr_m] =
      kinematic_simulation_B.T2_c[kinematic_simulation_B.b_i_l];
    kinematic_simulation_B.kstr = 6 * (kinematic_simulation_B.b_kstr_m + 3);
    kinematic_simulation_B.X[kinematic_simulation_B.kstr] = 0.0;
    kinematic_simulation_B.X[6 * kinematic_simulation_B.b_kstr_m + 3] = 0.0;
    kinematic_simulation_B.X[kinematic_simulation_B.kstr + 3] =
      kinematic_simulation_B.T2_c[kinematic_simulation_B.b_i_l];
    kinematic_simulation_B.endeffectorIndex =
      kinematic_simulation_B.T2_c[kinematic_simulation_B.b_i_l + 1];
    kinematic_simulation_B.X[6 * kinematic_simulation_B.b_kstr_m + 1] =
      kinematic_simulation_B.endeffectorIndex;
    kinematic_simulation_B.X[kinematic_simulation_B.kstr + 1] = 0.0;
    kinematic_simulation_B.X[6 * kinematic_simulation_B.b_kstr_m + 4] = 0.0;
    kinematic_simulation_B.X[kinematic_simulation_B.kstr + 4] =
      kinematic_simulation_B.endeffectorIndex;
    kinematic_simulation_B.endeffectorIndex =
      kinematic_simulation_B.T2_c[kinematic_simulation_B.b_i_l + 2];
    kinematic_simulation_B.X[6 * kinematic_simulation_B.b_kstr_m + 2] =
      kinematic_simulation_B.endeffectorIndex;
    kinematic_simulation_B.X[kinematic_simulation_B.kstr + 2] = 0.0;
    kinematic_simulation_B.X[6 * kinematic_simulation_B.b_kstr_m + 5] = 0.0;
    kinematic_simulation_B.X[kinematic_simulation_B.kstr + 5] =
      kinematic_simulation_B.endeffectorIndex;
  }

  kinematic_simulation_B.n_m = Jac->size[1];
  kinematic_simulation_B.b_kstr_m = b->size[0] * b->size[1];
  b->size[0] = 6;
  b->size[1] = Jac->size[1];
  kinema_emxEnsureCapacity_real_T(b, kinematic_simulation_B.b_kstr_m);
  kinematic_simulation_B.loop_ub_c = Jac->size[0] * Jac->size[1] - 1;
  for (kinematic_simulation_B.b_kstr_m = 0; kinematic_simulation_B.b_kstr_m <=
       kinematic_simulation_B.loop_ub_c; kinematic_simulation_B.b_kstr_m++) {
    b->data[kinematic_simulation_B.b_kstr_m] = Jac->
      data[kinematic_simulation_B.b_kstr_m];
  }

  kinematic_simulation_B.b_kstr_m = Jac->size[0] * Jac->size[1];
  Jac->size[0] = 6;
  Jac->size[1] = kinematic_simulation_B.n_m;
  kinema_emxEnsureCapacity_real_T(Jac, kinematic_simulation_B.b_kstr_m);
  for (kinematic_simulation_B.b_kstr_m = 0; kinematic_simulation_B.b_kstr_m <
       kinematic_simulation_B.n_m; kinematic_simulation_B.b_kstr_m++) {
    kinematic_simulation_B.coffset_tmp = kinematic_simulation_B.b_kstr_m * 6 - 1;
    for (kinematic_simulation_B.b_i_l = 0; kinematic_simulation_B.b_i_l < 6;
         kinematic_simulation_B.b_i_l++) {
      kinematic_simulation_B.s_j = 0.0;
      for (kinematic_simulation_B.loop_ub_c = 0;
           kinematic_simulation_B.loop_ub_c < 6;
           kinematic_simulation_B.loop_ub_c++) {
        kinematic_simulation_B.s_j +=
          kinematic_simulation_B.X[kinematic_simulation_B.loop_ub_c * 6 +
          kinematic_simulation_B.b_i_l] * b->data
          [(kinematic_simulation_B.coffset_tmp +
            kinematic_simulation_B.loop_ub_c) + 1];
      }

      Jac->data[(kinematic_simulation_B.coffset_tmp +
                 kinematic_simulation_B.b_i_l) + 1] = kinematic_simulation_B.s_j;
    }
  }

  kinematic_simula_emxFree_real_T(&b);
}

static void rigidBodyJoint_get_JointAxis_a(const c_rigidBodyJoint_kinematic__a_T
  *obj, real_T ax[3])
{
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  boolean_T guard1 = false;
  int32_T exitg1;
  for (kinematic_simulation_B.b_kstr_j = 0; kinematic_simulation_B.b_kstr_j < 8;
       kinematic_simulation_B.b_kstr_j++) {
    kinematic_simulation_B.b_bj[kinematic_simulation_B.b_kstr_j] =
      tmp[kinematic_simulation_B.b_kstr_j];
  }

  kinematic_simulation_B.b_bool_m = false;
  if (obj->Type->size[1] == 8) {
    kinematic_simulation_B.b_kstr_j = 1;
    do {
      exitg1 = 0;
      if (kinematic_simulation_B.b_kstr_j - 1 < 8) {
        kinematic_simulation_B.kstr_a = kinematic_simulation_B.b_kstr_j - 1;
        if (obj->Type->data[kinematic_simulation_B.kstr_a] !=
            kinematic_simulation_B.b_bj[kinematic_simulation_B.kstr_a]) {
          exitg1 = 1;
        } else {
          kinematic_simulation_B.b_kstr_j++;
        }
      } else {
        kinematic_simulation_B.b_bool_m = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (kinematic_simulation_B.b_bool_m) {
    guard1 = true;
  } else {
    for (kinematic_simulation_B.b_kstr_j = 0; kinematic_simulation_B.b_kstr_j <
         9; kinematic_simulation_B.b_kstr_j++) {
      kinematic_simulation_B.b_l[kinematic_simulation_B.b_kstr_j] =
        tmp_0[kinematic_simulation_B.b_kstr_j];
    }

    kinematic_simulation_B.b_bool_m = false;
    if (obj->Type->size[1] == 9) {
      kinematic_simulation_B.b_kstr_j = 1;
      do {
        exitg1 = 0;
        if (kinematic_simulation_B.b_kstr_j - 1 < 9) {
          kinematic_simulation_B.kstr_a = kinematic_simulation_B.b_kstr_j - 1;
          if (obj->Type->data[kinematic_simulation_B.kstr_a] !=
              kinematic_simulation_B.b_l[kinematic_simulation_B.kstr_a]) {
            exitg1 = 1;
          } else {
            kinematic_simulation_B.b_kstr_j++;
          }
        } else {
          kinematic_simulation_B.b_bool_m = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (kinematic_simulation_B.b_bool_m) {
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

static void RigidBodyTree_forwardKinemati_a(p_robotics_manip_internal_R_a_T *obj,
  const real_T qvec[6], emxArray_f_cell_wrap_kinemati_T *Ttree)
{
  n_robotics_manip_internal_R_a_T *body;
  emxArray_char_T_kinematic_sim_T *switch_expression;
  static const int8_T tmp[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  kinematic_simulation_B.n_o = obj->NumBodies;
  for (kinematic_simulation_B.b_kstr_p = 0; kinematic_simulation_B.b_kstr_p < 16;
       kinematic_simulation_B.b_kstr_p++) {
    kinematic_simulation_B.c_f1_b[kinematic_simulation_B.b_kstr_p] =
      tmp[kinematic_simulation_B.b_kstr_p];
  }

  kinematic_simulation_B.b_kstr_p = Ttree->size[0] * Ttree->size[1];
  Ttree->size[0] = 1;
  kinematic_simulation_B.e_c = static_cast<int32_T>(kinematic_simulation_B.n_o);
  Ttree->size[1] = kinematic_simulation_B.e_c;
  k_emxEnsureCapacity_f_cell_wrap(Ttree, kinematic_simulation_B.b_kstr_p);
  if (kinematic_simulation_B.e_c != 0) {
    kinematic_simulation_B.ntilecols_k = kinematic_simulation_B.e_c - 1;
    if (0 <= kinematic_simulation_B.ntilecols_k) {
      memcpy(&kinematic_simulation_B.expl_temp_m.f1[0],
             &kinematic_simulation_B.c_f1_b[0], sizeof(real_T) << 4U);
    }

    for (kinematic_simulation_B.b_jtilecol_p = 0;
         kinematic_simulation_B.b_jtilecol_p <=
         kinematic_simulation_B.ntilecols_k; kinematic_simulation_B.b_jtilecol_p
         ++) {
      Ttree->data[kinematic_simulation_B.b_jtilecol_p] =
        kinematic_simulation_B.expl_temp_m;
    }
  }

  kinematic_simulation_B.k_n = 1.0;
  kinematic_simulation_B.ntilecols_k = static_cast<int32_T>
    (kinematic_simulation_B.n_o) - 1;
  kinematic_simula_emxInit_char_T(&switch_expression, 2);
  if (0 <= kinematic_simulation_B.ntilecols_k) {
    for (kinematic_simulation_B.b_kstr_p = 0; kinematic_simulation_B.b_kstr_p <
         5; kinematic_simulation_B.b_kstr_p++) {
      kinematic_simulation_B.b_g[kinematic_simulation_B.b_kstr_p] =
        tmp_0[kinematic_simulation_B.b_kstr_p];
    }
  }

  for (kinematic_simulation_B.b_jtilecol_p = 0;
       kinematic_simulation_B.b_jtilecol_p <= kinematic_simulation_B.ntilecols_k;
       kinematic_simulation_B.b_jtilecol_p++) {
    body = obj->Bodies[kinematic_simulation_B.b_jtilecol_p];
    kinematic_simulation_B.n_o = body->JointInternal.PositionNumber;
    kinematic_simulation_B.n_o += kinematic_simulation_B.k_n;
    if (kinematic_simulation_B.k_n > kinematic_simulation_B.n_o - 1.0) {
      kinematic_simulation_B.e_c = 0;
      kinematic_simulation_B.d_h = 0;
    } else {
      kinematic_simulation_B.e_c = static_cast<int32_T>
        (kinematic_simulation_B.k_n) - 1;
      kinematic_simulation_B.d_h = static_cast<int32_T>
        (kinematic_simulation_B.n_o - 1.0);
    }

    kinematic_simulation_B.b_kstr_p = switch_expression->size[0] *
      switch_expression->size[1];
    switch_expression->size[0] = 1;
    switch_expression->size[1] = body->JointInternal.Type->size[1];
    kinema_emxEnsureCapacity_char_T(switch_expression,
      kinematic_simulation_B.b_kstr_p);
    kinematic_simulation_B.loop_ub_p = body->JointInternal.Type->size[0] *
      body->JointInternal.Type->size[1] - 1;
    for (kinematic_simulation_B.b_kstr_p = 0; kinematic_simulation_B.b_kstr_p <=
         kinematic_simulation_B.loop_ub_p; kinematic_simulation_B.b_kstr_p++) {
      switch_expression->data[kinematic_simulation_B.b_kstr_p] =
        body->JointInternal.Type->data[kinematic_simulation_B.b_kstr_p];
    }

    kinematic_simulation_B.b_bool_oc = false;
    if (switch_expression->size[1] == 5) {
      kinematic_simulation_B.b_kstr_p = 1;
      do {
        exitg1 = 0;
        if (kinematic_simulation_B.b_kstr_p - 1 < 5) {
          kinematic_simulation_B.loop_ub_p = kinematic_simulation_B.b_kstr_p - 1;
          if (switch_expression->data[kinematic_simulation_B.loop_ub_p] !=
              kinematic_simulation_B.b_g[kinematic_simulation_B.loop_ub_p]) {
            exitg1 = 1;
          } else {
            kinematic_simulation_B.b_kstr_p++;
          }
        } else {
          kinematic_simulation_B.b_bool_oc = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (kinematic_simulation_B.b_bool_oc) {
      kinematic_simulation_B.b_kstr_p = 0;
    } else {
      for (kinematic_simulation_B.b_kstr_p = 0; kinematic_simulation_B.b_kstr_p <
           8; kinematic_simulation_B.b_kstr_p++) {
        kinematic_simulation_B.b_e[kinematic_simulation_B.b_kstr_p] =
          tmp_1[kinematic_simulation_B.b_kstr_p];
      }

      kinematic_simulation_B.b_bool_oc = false;
      if (switch_expression->size[1] == 8) {
        kinematic_simulation_B.b_kstr_p = 1;
        do {
          exitg1 = 0;
          if (kinematic_simulation_B.b_kstr_p - 1 < 8) {
            kinematic_simulation_B.loop_ub_p = kinematic_simulation_B.b_kstr_p -
              1;
            if (switch_expression->data[kinematic_simulation_B.loop_ub_p] !=
                kinematic_simulation_B.b_e[kinematic_simulation_B.loop_ub_p]) {
              exitg1 = 1;
            } else {
              kinematic_simulation_B.b_kstr_p++;
            }
          } else {
            kinematic_simulation_B.b_bool_oc = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (kinematic_simulation_B.b_bool_oc) {
        kinematic_simulation_B.b_kstr_p = 1;
      } else {
        kinematic_simulation_B.b_kstr_p = -1;
      }
    }

    switch (kinematic_simulation_B.b_kstr_p) {
     case 0:
      memset(&kinematic_simulation_B.c_f1_b[0], 0, sizeof(real_T) << 4U);
      kinematic_simulation_B.c_f1_b[0] = 1.0;
      kinematic_simulation_B.c_f1_b[5] = 1.0;
      kinematic_simulation_B.c_f1_b[10] = 1.0;
      kinematic_simulation_B.c_f1_b[15] = 1.0;
      break;

     case 1:
      rigidBodyJoint_get_JointAxis_a(&body->JointInternal,
        kinematic_simulation_B.v_d);
      kinematic_simulation_B.d_h -= kinematic_simulation_B.e_c;
      for (kinematic_simulation_B.b_kstr_p = 0; kinematic_simulation_B.b_kstr_p <
           kinematic_simulation_B.d_h; kinematic_simulation_B.b_kstr_p++) {
        kinematic_simulation_B.e_data_l[kinematic_simulation_B.b_kstr_p] =
          kinematic_simulation_B.e_c + kinematic_simulation_B.b_kstr_p;
      }

      kinematic_simulation_B.result_data_d[0] = kinematic_simulation_B.v_d[0];
      kinematic_simulation_B.result_data_d[1] = kinematic_simulation_B.v_d[1];
      kinematic_simulation_B.result_data_d[2] = kinematic_simulation_B.v_d[2];
      if (0 <= (kinematic_simulation_B.d_h != 0) - 1) {
        kinematic_simulation_B.result_data_d[3] =
          qvec[kinematic_simulation_B.e_data_l[0]];
      }

      kinematic_simulation_B.k_n = 1.0 / sqrt
        ((kinematic_simulation_B.result_data_d[0] *
          kinematic_simulation_B.result_data_d[0] +
          kinematic_simulation_B.result_data_d[1] *
          kinematic_simulation_B.result_data_d[1]) +
         kinematic_simulation_B.result_data_d[2] *
         kinematic_simulation_B.result_data_d[2]);
      kinematic_simulation_B.v_d[0] = kinematic_simulation_B.result_data_d[0] *
        kinematic_simulation_B.k_n;
      kinematic_simulation_B.v_d[1] = kinematic_simulation_B.result_data_d[1] *
        kinematic_simulation_B.k_n;
      kinematic_simulation_B.v_d[2] = kinematic_simulation_B.result_data_d[2] *
        kinematic_simulation_B.k_n;
      kinematic_simulation_B.k_n = cos(kinematic_simulation_B.result_data_d[3]);
      kinematic_simulation_B.sth_m = sin(kinematic_simulation_B.result_data_d[3]);
      kinematic_simulation_B.tempR_j[0] = kinematic_simulation_B.v_d[0] *
        kinematic_simulation_B.v_d[0] * (1.0 - kinematic_simulation_B.k_n) +
        kinematic_simulation_B.k_n;
      kinematic_simulation_B.tempR_tmp_c = kinematic_simulation_B.v_d[1] *
        kinematic_simulation_B.v_d[0] * (1.0 - kinematic_simulation_B.k_n);
      kinematic_simulation_B.tempR_tmp_m = kinematic_simulation_B.v_d[2] *
        kinematic_simulation_B.sth_m;
      kinematic_simulation_B.tempR_j[1] = kinematic_simulation_B.tempR_tmp_c -
        kinematic_simulation_B.tempR_tmp_m;
      kinematic_simulation_B.tempR_tmp_m3 = kinematic_simulation_B.v_d[2] *
        kinematic_simulation_B.v_d[0] * (1.0 - kinematic_simulation_B.k_n);
      kinematic_simulation_B.tempR_tmp_ja = kinematic_simulation_B.v_d[1] *
        kinematic_simulation_B.sth_m;
      kinematic_simulation_B.tempR_j[2] = kinematic_simulation_B.tempR_tmp_m3 +
        kinematic_simulation_B.tempR_tmp_ja;
      kinematic_simulation_B.tempR_j[3] = kinematic_simulation_B.tempR_tmp_c +
        kinematic_simulation_B.tempR_tmp_m;
      kinematic_simulation_B.tempR_j[4] = kinematic_simulation_B.v_d[1] *
        kinematic_simulation_B.v_d[1] * (1.0 - kinematic_simulation_B.k_n) +
        kinematic_simulation_B.k_n;
      kinematic_simulation_B.tempR_tmp_c = kinematic_simulation_B.v_d[2] *
        kinematic_simulation_B.v_d[1] * (1.0 - kinematic_simulation_B.k_n);
      kinematic_simulation_B.tempR_tmp_m = kinematic_simulation_B.v_d[0] *
        kinematic_simulation_B.sth_m;
      kinematic_simulation_B.tempR_j[5] = kinematic_simulation_B.tempR_tmp_c -
        kinematic_simulation_B.tempR_tmp_m;
      kinematic_simulation_B.tempR_j[6] = kinematic_simulation_B.tempR_tmp_m3 -
        kinematic_simulation_B.tempR_tmp_ja;
      kinematic_simulation_B.tempR_j[7] = kinematic_simulation_B.tempR_tmp_c +
        kinematic_simulation_B.tempR_tmp_m;
      kinematic_simulation_B.tempR_j[8] = kinematic_simulation_B.v_d[2] *
        kinematic_simulation_B.v_d[2] * (1.0 - kinematic_simulation_B.k_n) +
        kinematic_simulation_B.k_n;
      for (kinematic_simulation_B.b_kstr_p = 0; kinematic_simulation_B.b_kstr_p <
           3; kinematic_simulation_B.b_kstr_p++) {
        kinematic_simulation_B.e_c = kinematic_simulation_B.b_kstr_p + 1;
        kinematic_simulation_B.R_l[kinematic_simulation_B.e_c - 1] =
          kinematic_simulation_B.tempR_j[(kinematic_simulation_B.e_c - 1) * 3];
        kinematic_simulation_B.e_c = kinematic_simulation_B.b_kstr_p + 1;
        kinematic_simulation_B.R_l[kinematic_simulation_B.e_c + 2] =
          kinematic_simulation_B.tempR_j[(kinematic_simulation_B.e_c - 1) * 3 +
          1];
        kinematic_simulation_B.e_c = kinematic_simulation_B.b_kstr_p + 1;
        kinematic_simulation_B.R_l[kinematic_simulation_B.e_c + 5] =
          kinematic_simulation_B.tempR_j[(kinematic_simulation_B.e_c - 1) * 3 +
          2];
      }

      memset(&kinematic_simulation_B.c_f1_b[0], 0, sizeof(real_T) << 4U);
      for (kinematic_simulation_B.b_kstr_p = 0; kinematic_simulation_B.b_kstr_p <
           3; kinematic_simulation_B.b_kstr_p++) {
        kinematic_simulation_B.d_h = kinematic_simulation_B.b_kstr_p << 2;
        kinematic_simulation_B.c_f1_b[kinematic_simulation_B.d_h] =
          kinematic_simulation_B.R_l[3 * kinematic_simulation_B.b_kstr_p];
        kinematic_simulation_B.c_f1_b[kinematic_simulation_B.d_h + 1] =
          kinematic_simulation_B.R_l[3 * kinematic_simulation_B.b_kstr_p + 1];
        kinematic_simulation_B.c_f1_b[kinematic_simulation_B.d_h + 2] =
          kinematic_simulation_B.R_l[3 * kinematic_simulation_B.b_kstr_p + 2];
      }

      kinematic_simulation_B.c_f1_b[15] = 1.0;
      break;

     default:
      rigidBodyJoint_get_JointAxis_a(&body->JointInternal,
        kinematic_simulation_B.v_d);
      memset(&kinematic_simulation_B.tempR_j[0], 0, 9U * sizeof(real_T));
      kinematic_simulation_B.tempR_j[0] = 1.0;
      kinematic_simulation_B.tempR_j[4] = 1.0;
      kinematic_simulation_B.tempR_j[8] = 1.0;
      for (kinematic_simulation_B.b_kstr_p = 0; kinematic_simulation_B.b_kstr_p <
           3; kinematic_simulation_B.b_kstr_p++) {
        kinematic_simulation_B.d_h = kinematic_simulation_B.b_kstr_p << 2;
        kinematic_simulation_B.c_f1_b[kinematic_simulation_B.d_h] =
          kinematic_simulation_B.tempR_j[3 * kinematic_simulation_B.b_kstr_p];
        kinematic_simulation_B.c_f1_b[kinematic_simulation_B.d_h + 1] =
          kinematic_simulation_B.tempR_j[3 * kinematic_simulation_B.b_kstr_p + 1];
        kinematic_simulation_B.c_f1_b[kinematic_simulation_B.d_h + 2] =
          kinematic_simulation_B.tempR_j[3 * kinematic_simulation_B.b_kstr_p + 2];
        kinematic_simulation_B.c_f1_b[kinematic_simulation_B.b_kstr_p + 12] =
          kinematic_simulation_B.v_d[kinematic_simulation_B.b_kstr_p] *
          qvec[kinematic_simulation_B.e_c];
      }

      kinematic_simulation_B.c_f1_b[3] = 0.0;
      kinematic_simulation_B.c_f1_b[7] = 0.0;
      kinematic_simulation_B.c_f1_b[11] = 0.0;
      kinematic_simulation_B.c_f1_b[15] = 1.0;
      break;
    }

    for (kinematic_simulation_B.b_kstr_p = 0; kinematic_simulation_B.b_kstr_p <
         16; kinematic_simulation_B.b_kstr_p++) {
      kinematic_simulation_B.a_p[kinematic_simulation_B.b_kstr_p] =
        body->
        JointInternal.JointToParentTransform[kinematic_simulation_B.b_kstr_p];
    }

    for (kinematic_simulation_B.b_kstr_p = 0; kinematic_simulation_B.b_kstr_p <
         16; kinematic_simulation_B.b_kstr_p++) {
      kinematic_simulation_B.b_c[kinematic_simulation_B.b_kstr_p] =
        body->
        JointInternal.ChildToJointTransform[kinematic_simulation_B.b_kstr_p];
    }

    for (kinematic_simulation_B.b_kstr_p = 0; kinematic_simulation_B.b_kstr_p <
         4; kinematic_simulation_B.b_kstr_p++) {
      for (kinematic_simulation_B.e_c = 0; kinematic_simulation_B.e_c < 4;
           kinematic_simulation_B.e_c++) {
        kinematic_simulation_B.d_h = kinematic_simulation_B.e_c << 2;
        kinematic_simulation_B.loop_ub_p = kinematic_simulation_B.b_kstr_p +
          kinematic_simulation_B.d_h;
        kinematic_simulation_B.a_f[kinematic_simulation_B.loop_ub_p] = 0.0;
        kinematic_simulation_B.a_f[kinematic_simulation_B.loop_ub_p] +=
          kinematic_simulation_B.c_f1_b[kinematic_simulation_B.d_h] *
          kinematic_simulation_B.a_p[kinematic_simulation_B.b_kstr_p];
        kinematic_simulation_B.a_f[kinematic_simulation_B.loop_ub_p] +=
          kinematic_simulation_B.c_f1_b[kinematic_simulation_B.d_h + 1] *
          kinematic_simulation_B.a_p[kinematic_simulation_B.b_kstr_p + 4];
        kinematic_simulation_B.a_f[kinematic_simulation_B.loop_ub_p] +=
          kinematic_simulation_B.c_f1_b[kinematic_simulation_B.d_h + 2] *
          kinematic_simulation_B.a_p[kinematic_simulation_B.b_kstr_p + 8];
        kinematic_simulation_B.a_f[kinematic_simulation_B.loop_ub_p] +=
          kinematic_simulation_B.c_f1_b[kinematic_simulation_B.d_h + 3] *
          kinematic_simulation_B.a_p[kinematic_simulation_B.b_kstr_p + 12];
      }

      for (kinematic_simulation_B.e_c = 0; kinematic_simulation_B.e_c < 4;
           kinematic_simulation_B.e_c++) {
        kinematic_simulation_B.d_h = kinematic_simulation_B.e_c << 2;
        kinematic_simulation_B.loop_ub_p = kinematic_simulation_B.b_kstr_p +
          kinematic_simulation_B.d_h;
        Ttree->data[kinematic_simulation_B.b_jtilecol_p]
          .f1[kinematic_simulation_B.loop_ub_p] = 0.0;
        Ttree->data[kinematic_simulation_B.b_jtilecol_p]
          .f1[kinematic_simulation_B.loop_ub_p] +=
          kinematic_simulation_B.b_c[kinematic_simulation_B.d_h] *
          kinematic_simulation_B.a_f[kinematic_simulation_B.b_kstr_p];
        Ttree->data[kinematic_simulation_B.b_jtilecol_p]
          .f1[kinematic_simulation_B.loop_ub_p] +=
          kinematic_simulation_B.b_c[kinematic_simulation_B.d_h + 1] *
          kinematic_simulation_B.a_f[kinematic_simulation_B.b_kstr_p + 4];
        Ttree->data[kinematic_simulation_B.b_jtilecol_p]
          .f1[kinematic_simulation_B.loop_ub_p] +=
          kinematic_simulation_B.b_c[kinematic_simulation_B.d_h + 2] *
          kinematic_simulation_B.a_f[kinematic_simulation_B.b_kstr_p + 8];
        Ttree->data[kinematic_simulation_B.b_jtilecol_p]
          .f1[kinematic_simulation_B.loop_ub_p] +=
          kinematic_simulation_B.b_c[kinematic_simulation_B.d_h + 3] *
          kinematic_simulation_B.a_f[kinematic_simulation_B.b_kstr_p + 12];
      }
    }

    kinematic_simulation_B.k_n = kinematic_simulation_B.n_o;
    if (body->ParentIndex > 0.0) {
      for (kinematic_simulation_B.b_kstr_p = 0; kinematic_simulation_B.b_kstr_p <
           16; kinematic_simulation_B.b_kstr_p++) {
        kinematic_simulation_B.a_p[kinematic_simulation_B.b_kstr_p] =
          Ttree->data[static_cast<int32_T>(body->ParentIndex) - 1]
          .f1[kinematic_simulation_B.b_kstr_p];
      }

      for (kinematic_simulation_B.b_kstr_p = 0; kinematic_simulation_B.b_kstr_p <
           4; kinematic_simulation_B.b_kstr_p++) {
        for (kinematic_simulation_B.e_c = 0; kinematic_simulation_B.e_c < 4;
             kinematic_simulation_B.e_c++) {
          kinematic_simulation_B.d_h = kinematic_simulation_B.e_c << 2;
          kinematic_simulation_B.loop_ub_p = kinematic_simulation_B.b_kstr_p +
            kinematic_simulation_B.d_h;
          kinematic_simulation_B.a_f[kinematic_simulation_B.loop_ub_p] = 0.0;
          kinematic_simulation_B.a_f[kinematic_simulation_B.loop_ub_p] +=
            Ttree->data[kinematic_simulation_B.b_jtilecol_p]
            .f1[kinematic_simulation_B.d_h] *
            kinematic_simulation_B.a_p[kinematic_simulation_B.b_kstr_p];
          kinematic_simulation_B.a_f[kinematic_simulation_B.loop_ub_p] +=
            Ttree->data[kinematic_simulation_B.b_jtilecol_p]
            .f1[kinematic_simulation_B.d_h + 1] *
            kinematic_simulation_B.a_p[kinematic_simulation_B.b_kstr_p + 4];
          kinematic_simulation_B.a_f[kinematic_simulation_B.loop_ub_p] +=
            Ttree->data[kinematic_simulation_B.b_jtilecol_p]
            .f1[kinematic_simulation_B.d_h + 2] *
            kinematic_simulation_B.a_p[kinematic_simulation_B.b_kstr_p + 8];
          kinematic_simulation_B.a_f[kinematic_simulation_B.loop_ub_p] +=
            Ttree->data[kinematic_simulation_B.b_jtilecol_p]
            .f1[kinematic_simulation_B.d_h + 3] *
            kinematic_simulation_B.a_p[kinematic_simulation_B.b_kstr_p + 12];
        }
      }

      for (kinematic_simulation_B.b_kstr_p = 0; kinematic_simulation_B.b_kstr_p <
           16; kinematic_simulation_B.b_kstr_p++) {
        Ttree->data[kinematic_simulation_B.b_jtilecol_p]
          .f1[kinematic_simulation_B.b_kstr_p] =
          kinematic_simulation_B.a_f[kinematic_simulation_B.b_kstr_p];
      }
    }
  }

  kinematic_simula_emxFree_char_T(&switch_expression);
}

static void kinematic_simul_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Positions[128], uint32_T *varargout_2_Positions_SL_Info_C,
  uint32_T *varargout_2_Positions_SL_Info_R, real_T varargout_2_Velocities[128],
  uint32_T *varargout_2_Velocities_SL_Info_, uint32_T
  *varargout_2_Velocities_SL_Inf_0, real_T varargout_2_Accelerations[128],
  uint32_T *varargout_2_Accelerations_SL_In, uint32_T
  *varargout_2_Accelerations_SL__0, real_T varargout_2_Effort[128], uint32_T
  *varargout_2_Effort_SL_Info_Curr, uint32_T *varargout_2_Effort_SL_Info_Rece,
  real_T *varargout_2_TimeFromStart_Sec, real_T *varargout_2_TimeFromStart_Nsec)
{
  *varargout_1 = Sub_kinematic_simulation_16.getLatestMessage
    (&kinematic_simulation_B.b_varargout_2);
  *varargout_2_Positions_SL_Info_C =
    kinematic_simulation_B.b_varargout_2.Positions_SL_Info.CurrentLength;
  *varargout_2_Positions_SL_Info_R =
    kinematic_simulation_B.b_varargout_2.Positions_SL_Info.ReceivedLength;
  *varargout_2_Velocities_SL_Info_ =
    kinematic_simulation_B.b_varargout_2.Velocities_SL_Info.CurrentLength;
  *varargout_2_Velocities_SL_Inf_0 =
    kinematic_simulation_B.b_varargout_2.Velocities_SL_Info.ReceivedLength;
  *varargout_2_Accelerations_SL_In =
    kinematic_simulation_B.b_varargout_2.Accelerations_SL_Info.CurrentLength;
  *varargout_2_Accelerations_SL__0 =
    kinematic_simulation_B.b_varargout_2.Accelerations_SL_Info.ReceivedLength;
  memcpy(&varargout_2_Positions[0],
         &kinematic_simulation_B.b_varargout_2.Positions[0], sizeof(real_T) <<
         7U);
  memcpy(&varargout_2_Velocities[0],
         &kinematic_simulation_B.b_varargout_2.Velocities[0], sizeof(real_T) <<
         7U);
  memcpy(&varargout_2_Accelerations[0],
         &kinematic_simulation_B.b_varargout_2.Accelerations[0], sizeof(real_T) <<
         7U);
  memcpy(&varargout_2_Effort[0], &kinematic_simulation_B.b_varargout_2.Effort[0],
         sizeof(real_T) << 7U);
  *varargout_2_Effort_SL_Info_Curr =
    kinematic_simulation_B.b_varargout_2.Effort_SL_Info.CurrentLength;
  *varargout_2_Effort_SL_Info_Rece =
    kinematic_simulation_B.b_varargout_2.Effort_SL_Info.ReceivedLength;
  *varargout_2_TimeFromStart_Sec =
    kinematic_simulation_B.b_varargout_2.TimeFromStart.Sec;
  *varargout_2_TimeFromStart_Nsec =
    kinematic_simulation_B.b_varargout_2.TimeFromStart.Nsec;
}

// Function for MATLAB Function: '<S4>/MATLAB Function'
static boolean_T kinematic_simulati_anyNonFinite(const real_T x[16])
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

static real_T kinematic_simulat_rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T y;
  kinematic_simulation_B.a_i = fabs(u0);
  y = fabs(u1);
  if (kinematic_simulation_B.a_i < y) {
    kinematic_simulation_B.a_i /= y;
    y *= sqrt(kinematic_simulation_B.a_i * kinematic_simulation_B.a_i + 1.0);
  } else if (kinematic_simulation_B.a_i > y) {
    y /= kinematic_simulation_B.a_i;
    y = sqrt(y * y + 1.0) * kinematic_simulation_B.a_i;
  } else {
    if (!rtIsNaN(y)) {
      y = kinematic_simulation_B.a_i * 1.4142135623730951;
    }
  }

  return y;
}

// Function for MATLAB Function: '<S4>/MATLAB Function'
static real_T kinematic_simulation_xzlangeM(const creal_T x[16])
{
  real_T y;
  real_T absxk;
  int32_T k;
  boolean_T exitg1;
  y = 0.0;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 16)) {
    absxk = kinematic_simulat_rt_hypotd_snf(x[k].re, x[k].im);
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

// Function for MATLAB Function: '<S4>/MATLAB Function'
static void kinematic_simulation_xzlascl(real_T cfrom, real_T cto, creal_T A[16])
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

// Function for MATLAB Function: '<S4>/MATLAB Function'
static real_T kinematic_simulation_xzlanhs(const creal_T A[16], int32_T ilo,
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

// Function for MATLAB Function: '<S4>/MATLAB Function'
static void kinematic_simulation_xzlartg_a(const creal_T f, const creal_T g,
  real_T *cs, creal_T *sn)
{
  real_T gs_im;
  boolean_T guard1 = false;
  kinematic_simulation_B.f2s_a = fabs(f.re);
  kinematic_simulation_B.di_i = fabs(f.im);
  kinematic_simulation_B.scale_e = kinematic_simulation_B.f2s_a;
  if (kinematic_simulation_B.di_i > kinematic_simulation_B.f2s_a) {
    kinematic_simulation_B.scale_e = kinematic_simulation_B.di_i;
  }

  kinematic_simulation_B.gs_re_i = fabs(g.re);
  gs_im = fabs(g.im);
  if (gs_im > kinematic_simulation_B.gs_re_i) {
    kinematic_simulation_B.gs_re_i = gs_im;
  }

  if (kinematic_simulation_B.gs_re_i > kinematic_simulation_B.scale_e) {
    kinematic_simulation_B.scale_e = kinematic_simulation_B.gs_re_i;
  }

  kinematic_simulation_B.fs_re_o = f.re;
  kinematic_simulation_B.fs_im_o = f.im;
  kinematic_simulation_B.gs_re_i = g.re;
  gs_im = g.im;
  guard1 = false;
  if (kinematic_simulation_B.scale_e >= 7.4428285367870146E+137) {
    do {
      kinematic_simulation_B.fs_re_o *= 1.3435752215134178E-138;
      kinematic_simulation_B.fs_im_o *= 1.3435752215134178E-138;
      kinematic_simulation_B.gs_re_i *= 1.3435752215134178E-138;
      gs_im *= 1.3435752215134178E-138;
      kinematic_simulation_B.scale_e *= 1.3435752215134178E-138;
    } while (!(kinematic_simulation_B.scale_e < 7.4428285367870146E+137));

    guard1 = true;
  } else if (kinematic_simulation_B.scale_e <= 1.3435752215134178E-138) {
    if ((g.re == 0.0) && (g.im == 0.0)) {
      *cs = 1.0;
      sn->re = 0.0;
      sn->im = 0.0;
    } else {
      do {
        kinematic_simulation_B.fs_re_o *= 7.4428285367870146E+137;
        kinematic_simulation_B.fs_im_o *= 7.4428285367870146E+137;
        kinematic_simulation_B.gs_re_i *= 7.4428285367870146E+137;
        gs_im *= 7.4428285367870146E+137;
        kinematic_simulation_B.scale_e *= 7.4428285367870146E+137;
      } while (!(kinematic_simulation_B.scale_e > 1.3435752215134178E-138));

      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    kinematic_simulation_B.scale_e = kinematic_simulation_B.fs_re_o *
      kinematic_simulation_B.fs_re_o + kinematic_simulation_B.fs_im_o *
      kinematic_simulation_B.fs_im_o;
    kinematic_simulation_B.g2_a = kinematic_simulation_B.gs_re_i *
      kinematic_simulation_B.gs_re_i + gs_im * gs_im;
    kinematic_simulation_B.x_l = kinematic_simulation_B.g2_a;
    if (1.0 > kinematic_simulation_B.g2_a) {
      kinematic_simulation_B.x_l = 1.0;
    }

    if (kinematic_simulation_B.scale_e <= kinematic_simulation_B.x_l *
        2.0041683600089728E-292) {
      if ((f.re == 0.0) && (f.im == 0.0)) {
        *cs = 0.0;
        kinematic_simulation_B.f2s_a = kinematic_simulat_rt_hypotd_snf
          (kinematic_simulation_B.gs_re_i, gs_im);
        sn->re = kinematic_simulation_B.gs_re_i / kinematic_simulation_B.f2s_a;
        sn->im = -gs_im / kinematic_simulation_B.f2s_a;
      } else {
        kinematic_simulation_B.scale_e = sqrt(kinematic_simulation_B.g2_a);
        *cs = kinematic_simulat_rt_hypotd_snf(kinematic_simulation_B.fs_re_o,
          kinematic_simulation_B.fs_im_o) / kinematic_simulation_B.scale_e;
        if (kinematic_simulation_B.di_i > kinematic_simulation_B.f2s_a) {
          kinematic_simulation_B.f2s_a = kinematic_simulation_B.di_i;
        }

        if (kinematic_simulation_B.f2s_a > 1.0) {
          kinematic_simulation_B.f2s_a = kinematic_simulat_rt_hypotd_snf(f.re,
            f.im);
          kinematic_simulation_B.fs_re_o = f.re / kinematic_simulation_B.f2s_a;
          kinematic_simulation_B.fs_im_o = f.im / kinematic_simulation_B.f2s_a;
        } else {
          kinematic_simulation_B.fs_re_o = 7.4428285367870146E+137 * f.re;
          kinematic_simulation_B.di_i = 7.4428285367870146E+137 * f.im;
          kinematic_simulation_B.f2s_a = kinematic_simulat_rt_hypotd_snf
            (kinematic_simulation_B.fs_re_o, kinematic_simulation_B.di_i);
          kinematic_simulation_B.fs_re_o /= kinematic_simulation_B.f2s_a;
          kinematic_simulation_B.fs_im_o = kinematic_simulation_B.di_i /
            kinematic_simulation_B.f2s_a;
        }

        kinematic_simulation_B.gs_re_i /= kinematic_simulation_B.scale_e;
        gs_im = -gs_im / kinematic_simulation_B.scale_e;
        sn->re = kinematic_simulation_B.fs_re_o * kinematic_simulation_B.gs_re_i
          - kinematic_simulation_B.fs_im_o * gs_im;
        sn->im = kinematic_simulation_B.fs_re_o * gs_im +
          kinematic_simulation_B.fs_im_o * kinematic_simulation_B.gs_re_i;
      }
    } else {
      kinematic_simulation_B.f2s_a = sqrt(kinematic_simulation_B.g2_a /
        kinematic_simulation_B.scale_e + 1.0);
      kinematic_simulation_B.fs_re_o *= kinematic_simulation_B.f2s_a;
      kinematic_simulation_B.fs_im_o *= kinematic_simulation_B.f2s_a;
      *cs = 1.0 / kinematic_simulation_B.f2s_a;
      kinematic_simulation_B.f2s_a = kinematic_simulation_B.scale_e +
        kinematic_simulation_B.g2_a;
      kinematic_simulation_B.fs_re_o /= kinematic_simulation_B.f2s_a;
      kinematic_simulation_B.fs_im_o /= kinematic_simulation_B.f2s_a;
      sn->re = kinematic_simulation_B.fs_re_o * kinematic_simulation_B.gs_re_i -
        kinematic_simulation_B.fs_im_o * -gs_im;
      sn->im = kinematic_simulation_B.fs_re_o * -gs_im +
        kinematic_simulation_B.fs_im_o * kinematic_simulation_B.gs_re_i;
    }
  }
}

// Function for MATLAB Function: '<S4>/MATLAB Function'
static void kinematic_simulation_xzlartg(const creal_T f, const creal_T g,
  real_T *cs, creal_T *sn, creal_T *r)
{
  int32_T count;
  int32_T rescaledir;
  boolean_T guard1 = false;
  kinematic_simulation_B.f2s = fabs(f.re);
  kinematic_simulation_B.di = fabs(f.im);
  kinematic_simulation_B.scale_n = kinematic_simulation_B.f2s;
  if (kinematic_simulation_B.di > kinematic_simulation_B.f2s) {
    kinematic_simulation_B.scale_n = kinematic_simulation_B.di;
  }

  kinematic_simulation_B.gs_re = fabs(g.re);
  kinematic_simulation_B.gs_im = fabs(g.im);
  if (kinematic_simulation_B.gs_im > kinematic_simulation_B.gs_re) {
    kinematic_simulation_B.gs_re = kinematic_simulation_B.gs_im;
  }

  if (kinematic_simulation_B.gs_re > kinematic_simulation_B.scale_n) {
    kinematic_simulation_B.scale_n = kinematic_simulation_B.gs_re;
  }

  kinematic_simulation_B.fs_re = f.re;
  kinematic_simulation_B.fs_im = f.im;
  kinematic_simulation_B.gs_re = g.re;
  kinematic_simulation_B.gs_im = g.im;
  count = -1;
  rescaledir = 0;
  guard1 = false;
  if (kinematic_simulation_B.scale_n >= 7.4428285367870146E+137) {
    do {
      count++;
      kinematic_simulation_B.fs_re *= 1.3435752215134178E-138;
      kinematic_simulation_B.fs_im *= 1.3435752215134178E-138;
      kinematic_simulation_B.gs_re *= 1.3435752215134178E-138;
      kinematic_simulation_B.gs_im *= 1.3435752215134178E-138;
      kinematic_simulation_B.scale_n *= 1.3435752215134178E-138;
    } while (!(kinematic_simulation_B.scale_n < 7.4428285367870146E+137));

    rescaledir = 1;
    guard1 = true;
  } else if (kinematic_simulation_B.scale_n <= 1.3435752215134178E-138) {
    if ((g.re == 0.0) && (g.im == 0.0)) {
      *cs = 1.0;
      sn->re = 0.0;
      sn->im = 0.0;
      *r = f;
    } else {
      do {
        count++;
        kinematic_simulation_B.fs_re *= 7.4428285367870146E+137;
        kinematic_simulation_B.fs_im *= 7.4428285367870146E+137;
        kinematic_simulation_B.gs_re *= 7.4428285367870146E+137;
        kinematic_simulation_B.gs_im *= 7.4428285367870146E+137;
        kinematic_simulation_B.scale_n *= 7.4428285367870146E+137;
      } while (!(kinematic_simulation_B.scale_n > 1.3435752215134178E-138));

      rescaledir = -1;
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    kinematic_simulation_B.scale_n = kinematic_simulation_B.fs_re *
      kinematic_simulation_B.fs_re + kinematic_simulation_B.fs_im *
      kinematic_simulation_B.fs_im;
    kinematic_simulation_B.g2 = kinematic_simulation_B.gs_re *
      kinematic_simulation_B.gs_re + kinematic_simulation_B.gs_im *
      kinematic_simulation_B.gs_im;
    kinematic_simulation_B.x = kinematic_simulation_B.g2;
    if (1.0 > kinematic_simulation_B.g2) {
      kinematic_simulation_B.x = 1.0;
    }

    if (kinematic_simulation_B.scale_n <= kinematic_simulation_B.x *
        2.0041683600089728E-292) {
      if ((f.re == 0.0) && (f.im == 0.0)) {
        *cs = 0.0;
        r->re = kinematic_simulat_rt_hypotd_snf(g.re, g.im);
        r->im = 0.0;
        kinematic_simulation_B.f2s = kinematic_simulat_rt_hypotd_snf
          (kinematic_simulation_B.gs_re, kinematic_simulation_B.gs_im);
        sn->re = kinematic_simulation_B.gs_re / kinematic_simulation_B.f2s;
        sn->im = -kinematic_simulation_B.gs_im / kinematic_simulation_B.f2s;
      } else {
        kinematic_simulation_B.scale_n = sqrt(kinematic_simulation_B.g2);
        *cs = kinematic_simulat_rt_hypotd_snf(kinematic_simulation_B.fs_re,
          kinematic_simulation_B.fs_im) / kinematic_simulation_B.scale_n;
        if (kinematic_simulation_B.di > kinematic_simulation_B.f2s) {
          kinematic_simulation_B.f2s = kinematic_simulation_B.di;
        }

        if (kinematic_simulation_B.f2s > 1.0) {
          kinematic_simulation_B.f2s = kinematic_simulat_rt_hypotd_snf(f.re,
            f.im);
          kinematic_simulation_B.fs_re = f.re / kinematic_simulation_B.f2s;
          kinematic_simulation_B.fs_im = f.im / kinematic_simulation_B.f2s;
        } else {
          kinematic_simulation_B.fs_re = 7.4428285367870146E+137 * f.re;
          kinematic_simulation_B.di = 7.4428285367870146E+137 * f.im;
          kinematic_simulation_B.f2s = kinematic_simulat_rt_hypotd_snf
            (kinematic_simulation_B.fs_re, kinematic_simulation_B.di);
          kinematic_simulation_B.fs_re /= kinematic_simulation_B.f2s;
          kinematic_simulation_B.fs_im = kinematic_simulation_B.di /
            kinematic_simulation_B.f2s;
        }

        kinematic_simulation_B.gs_re /= kinematic_simulation_B.scale_n;
        kinematic_simulation_B.gs_im = -kinematic_simulation_B.gs_im /
          kinematic_simulation_B.scale_n;
        sn->re = kinematic_simulation_B.fs_re * kinematic_simulation_B.gs_re -
          kinematic_simulation_B.fs_im * kinematic_simulation_B.gs_im;
        sn->im = kinematic_simulation_B.fs_re * kinematic_simulation_B.gs_im +
          kinematic_simulation_B.fs_im * kinematic_simulation_B.gs_re;
        r->re = (sn->re * g.re - sn->im * g.im) + *cs * f.re;
        r->im = (sn->re * g.im + sn->im * g.re) + *cs * f.im;
      }
    } else {
      kinematic_simulation_B.f2s = sqrt(kinematic_simulation_B.g2 /
        kinematic_simulation_B.scale_n + 1.0);
      r->re = kinematic_simulation_B.f2s * kinematic_simulation_B.fs_re;
      r->im = kinematic_simulation_B.f2s * kinematic_simulation_B.fs_im;
      *cs = 1.0 / kinematic_simulation_B.f2s;
      kinematic_simulation_B.f2s = kinematic_simulation_B.scale_n +
        kinematic_simulation_B.g2;
      kinematic_simulation_B.fs_re = r->re / kinematic_simulation_B.f2s;
      kinematic_simulation_B.f2s = r->im / kinematic_simulation_B.f2s;
      sn->re = kinematic_simulation_B.fs_re * kinematic_simulation_B.gs_re -
        kinematic_simulation_B.f2s * -kinematic_simulation_B.gs_im;
      sn->im = kinematic_simulation_B.fs_re * -kinematic_simulation_B.gs_im +
        kinematic_simulation_B.f2s * kinematic_simulation_B.gs_re;
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

// Function for MATLAB Function: '<S4>/MATLAB Function'
static void kinematic_simulation_xzhgeqz(creal_T A[16], int32_T ilo, int32_T ihi,
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
  kinematic_simulation_B.eshift_re = 0.0;
  kinematic_simulation_B.eshift_im = 0.0;
  kinematic_simulation_B.ctemp.re = 0.0;
  kinematic_simulation_B.ctemp.im = 0.0;
  kinematic_simulation_B.anorm = kinematic_simulation_xzlanhs(A, ilo, ihi);
  kinematic_simulation_B.shift_re = 2.2204460492503131E-16 *
    kinematic_simulation_B.anorm;
  kinematic_simulation_B.b_atol = 2.2250738585072014E-308;
  if (kinematic_simulation_B.shift_re > 2.2250738585072014E-308) {
    kinematic_simulation_B.b_atol = kinematic_simulation_B.shift_re;
  }

  kinematic_simulation_B.shift_re = 2.2250738585072014E-308;
  if (kinematic_simulation_B.anorm > 2.2250738585072014E-308) {
    kinematic_simulation_B.shift_re = kinematic_simulation_B.anorm;
  }

  kinematic_simulation_B.anorm = 1.0 / kinematic_simulation_B.shift_re;
  failed = true;
  kinematic_simulation_B.ilast = ihi;
  while (kinematic_simulation_B.ilast + 1 < 5) {
    alpha1[kinematic_simulation_B.ilast] = A[(kinematic_simulation_B.ilast << 2)
      + kinematic_simulation_B.ilast];
    kinematic_simulation_B.ilast++;
  }

  guard1 = false;
  guard2 = false;
  if (ihi >= ilo) {
    kinematic_simulation_B.ifirst = ilo;
    kinematic_simulation_B.istart = ilo;
    kinematic_simulation_B.ilast = ihi - 1;
    kinematic_simulation_B.ilastm1 = ihi - 2;
    kinematic_simulation_B.iiter = 0;
    goto60 = false;
    goto70 = false;
    goto90 = false;
    kinematic_simulation_B.jiter = 0;
    do {
      exitg1 = 0;
      if (kinematic_simulation_B.jiter <= ((ihi - ilo) + 1) * 30 - 1) {
        if (kinematic_simulation_B.ilast + 1 == ilo) {
          goto60 = true;
        } else {
          kinematic_simulation_B.jp1 = (kinematic_simulation_B.ilastm1 << 2) +
            kinematic_simulation_B.ilast;
          if (fabs(A[kinematic_simulation_B.jp1].re) + fabs
              (A[kinematic_simulation_B.jp1].im) <=
              kinematic_simulation_B.b_atol) {
            A[kinematic_simulation_B.jp1].re = 0.0;
            A[kinematic_simulation_B.jp1].im = 0.0;
            goto60 = true;
          } else {
            kinematic_simulation_B.j = kinematic_simulation_B.ilastm1;
            guard3 = false;
            exitg2 = false;
            while ((!exitg2) && (kinematic_simulation_B.j + 1 >= ilo)) {
              if (kinematic_simulation_B.j + 1 == ilo) {
                guard3 = true;
                exitg2 = true;
              } else {
                kinematic_simulation_B.jp1 = ((kinematic_simulation_B.j - 1) <<
                  2) + kinematic_simulation_B.j;
                if (fabs(A[kinematic_simulation_B.jp1].re) + fabs
                    (A[kinematic_simulation_B.jp1].im) <=
                    kinematic_simulation_B.b_atol) {
                  A[kinematic_simulation_B.jp1].re = 0.0;
                  A[kinematic_simulation_B.jp1].im = 0.0;
                  guard3 = true;
                  exitg2 = true;
                } else {
                  kinematic_simulation_B.j--;
                  guard3 = false;
                }
              }
            }

            if (guard3) {
              kinematic_simulation_B.ifirst = kinematic_simulation_B.j + 1;
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
          for (kinematic_simulation_B.jp1 = 0; kinematic_simulation_B.jp1 < 16;
               kinematic_simulation_B.jp1++) {
            Z[kinematic_simulation_B.jp1].re = (rtNaN);
            Z[kinematic_simulation_B.jp1].im = 0.0;
          }

          *info = 1;
          exitg1 = 1;
        } else if (goto60) {
          goto60 = false;
          alpha1[kinematic_simulation_B.ilast] = A[(kinematic_simulation_B.ilast
            << 2) + kinematic_simulation_B.ilast];
          kinematic_simulation_B.ilast = kinematic_simulation_B.ilastm1;
          kinematic_simulation_B.ilastm1--;
          if (kinematic_simulation_B.ilast + 1 < ilo) {
            failed = false;
            guard2 = true;
            exitg1 = 1;
          } else {
            kinematic_simulation_B.iiter = 0;
            kinematic_simulation_B.eshift_re = 0.0;
            kinematic_simulation_B.eshift_im = 0.0;
            kinematic_simulation_B.jiter++;
          }
        } else {
          if (goto70) {
            goto70 = false;
            kinematic_simulation_B.iiter++;
            if (kinematic_simulation_B.iiter - div_nzp_s32
                (kinematic_simulation_B.iiter, 10) * 10 != 0) {
              kinematic_simulation_B.j = (kinematic_simulation_B.ilastm1 << 2) +
                kinematic_simulation_B.ilastm1;
              kinematic_simulation_B.ar = A[kinematic_simulation_B.j].re *
                kinematic_simulation_B.anorm;
              kinematic_simulation_B.ai = A[kinematic_simulation_B.j].im *
                kinematic_simulation_B.anorm;
              if (kinematic_simulation_B.ai == 0.0) {
                kinematic_simulation_B.shift_re = kinematic_simulation_B.ar /
                  0.5;
                kinematic_simulation_B.shift_im = 0.0;
              } else if (kinematic_simulation_B.ar == 0.0) {
                kinematic_simulation_B.shift_re = 0.0;
                kinematic_simulation_B.shift_im = kinematic_simulation_B.ai /
                  0.5;
              } else {
                kinematic_simulation_B.shift_re = kinematic_simulation_B.ar /
                  0.5;
                kinematic_simulation_B.shift_im = kinematic_simulation_B.ai /
                  0.5;
              }

              kinematic_simulation_B.j = (kinematic_simulation_B.ilast << 2) +
                kinematic_simulation_B.ilast;
              kinematic_simulation_B.ar = A[kinematic_simulation_B.j].re *
                kinematic_simulation_B.anorm;
              kinematic_simulation_B.ai = A[kinematic_simulation_B.j].im *
                kinematic_simulation_B.anorm;
              if (kinematic_simulation_B.ai == 0.0) {
                kinematic_simulation_B.ad22.re = kinematic_simulation_B.ar / 0.5;
                kinematic_simulation_B.ad22.im = 0.0;
              } else if (kinematic_simulation_B.ar == 0.0) {
                kinematic_simulation_B.ad22.re = 0.0;
                kinematic_simulation_B.ad22.im = kinematic_simulation_B.ai / 0.5;
              } else {
                kinematic_simulation_B.ad22.re = kinematic_simulation_B.ar / 0.5;
                kinematic_simulation_B.ad22.im = kinematic_simulation_B.ai / 0.5;
              }

              kinematic_simulation_B.t1_re = (kinematic_simulation_B.shift_re +
                kinematic_simulation_B.ad22.re) * 0.5;
              kinematic_simulation_B.t1_im = (kinematic_simulation_B.shift_im +
                kinematic_simulation_B.ad22.im) * 0.5;
              kinematic_simulation_B.j = (kinematic_simulation_B.ilast << 2) +
                kinematic_simulation_B.ilastm1;
              kinematic_simulation_B.ar = A[kinematic_simulation_B.j].re *
                kinematic_simulation_B.anorm;
              kinematic_simulation_B.ai = A[kinematic_simulation_B.j].im *
                kinematic_simulation_B.anorm;
              if (kinematic_simulation_B.ai == 0.0) {
                kinematic_simulation_B.absxr = kinematic_simulation_B.ar / 0.5;
                kinematic_simulation_B.absxi = 0.0;
              } else if (kinematic_simulation_B.ar == 0.0) {
                kinematic_simulation_B.absxr = 0.0;
                kinematic_simulation_B.absxi = kinematic_simulation_B.ai / 0.5;
              } else {
                kinematic_simulation_B.absxr = kinematic_simulation_B.ar / 0.5;
                kinematic_simulation_B.absxi = kinematic_simulation_B.ai / 0.5;
              }

              kinematic_simulation_B.j = (kinematic_simulation_B.ilastm1 << 2) +
                kinematic_simulation_B.ilast;
              kinematic_simulation_B.ar = A[kinematic_simulation_B.j].re *
                kinematic_simulation_B.anorm;
              kinematic_simulation_B.ai = A[kinematic_simulation_B.j].im *
                kinematic_simulation_B.anorm;
              if (kinematic_simulation_B.ai == 0.0) {
                kinematic_simulation_B.ar /= 0.5;
                kinematic_simulation_B.ai = 0.0;
              } else if (kinematic_simulation_B.ar == 0.0) {
                kinematic_simulation_B.ar = 0.0;
                kinematic_simulation_B.ai /= 0.5;
              } else {
                kinematic_simulation_B.ar /= 0.5;
                kinematic_simulation_B.ai /= 0.5;
              }

              kinematic_simulation_B.shift_im_o =
                kinematic_simulation_B.shift_re * kinematic_simulation_B.ad22.im
                + kinematic_simulation_B.shift_im *
                kinematic_simulation_B.ad22.re;
              kinematic_simulation_B.shift_re = ((kinematic_simulation_B.t1_re *
                kinematic_simulation_B.t1_re - kinematic_simulation_B.t1_im *
                kinematic_simulation_B.t1_im) + (kinematic_simulation_B.absxr *
                kinematic_simulation_B.ar - kinematic_simulation_B.absxi *
                kinematic_simulation_B.ai)) - (kinematic_simulation_B.shift_re *
                kinematic_simulation_B.ad22.re - kinematic_simulation_B.shift_im
                * kinematic_simulation_B.ad22.im);
              kinematic_simulation_B.shift_im = kinematic_simulation_B.t1_re *
                kinematic_simulation_B.t1_im;
              kinematic_simulation_B.shift_im =
                ((kinematic_simulation_B.shift_im +
                  kinematic_simulation_B.shift_im) +
                 (kinematic_simulation_B.absxr * kinematic_simulation_B.ai +
                  kinematic_simulation_B.absxi * kinematic_simulation_B.ar)) -
                kinematic_simulation_B.shift_im_o;
              if (kinematic_simulation_B.shift_im == 0.0) {
                if (kinematic_simulation_B.shift_re < 0.0) {
                  kinematic_simulation_B.absxr = 0.0;
                  kinematic_simulation_B.absxi = sqrt
                    (-kinematic_simulation_B.shift_re);
                } else {
                  kinematic_simulation_B.absxr = sqrt
                    (kinematic_simulation_B.shift_re);
                  kinematic_simulation_B.absxi = 0.0;
                }
              } else if (kinematic_simulation_B.shift_re == 0.0) {
                if (kinematic_simulation_B.shift_im < 0.0) {
                  kinematic_simulation_B.absxr = sqrt
                    (-kinematic_simulation_B.shift_im / 2.0);
                  kinematic_simulation_B.absxi = -kinematic_simulation_B.absxr;
                } else {
                  kinematic_simulation_B.absxr = sqrt
                    (kinematic_simulation_B.shift_im / 2.0);
                  kinematic_simulation_B.absxi = kinematic_simulation_B.absxr;
                }
              } else if (rtIsNaN(kinematic_simulation_B.shift_re)) {
                kinematic_simulation_B.absxr = kinematic_simulation_B.shift_re;
                kinematic_simulation_B.absxi = kinematic_simulation_B.shift_re;
              } else if (rtIsNaN(kinematic_simulation_B.shift_im)) {
                kinematic_simulation_B.absxr = kinematic_simulation_B.shift_im;
                kinematic_simulation_B.absxi = kinematic_simulation_B.shift_im;
              } else if (rtIsInf(kinematic_simulation_B.shift_im)) {
                kinematic_simulation_B.absxr = fabs
                  (kinematic_simulation_B.shift_im);
                kinematic_simulation_B.absxi = kinematic_simulation_B.shift_im;
              } else if (rtIsInf(kinematic_simulation_B.shift_re)) {
                if (kinematic_simulation_B.shift_re < 0.0) {
                  kinematic_simulation_B.absxr = 0.0;
                  kinematic_simulation_B.absxi = kinematic_simulation_B.shift_im
                    * -kinematic_simulation_B.shift_re;
                } else {
                  kinematic_simulation_B.absxr = kinematic_simulation_B.shift_re;
                  kinematic_simulation_B.absxi = 0.0;
                }
              } else {
                kinematic_simulation_B.absxr = fabs
                  (kinematic_simulation_B.shift_re);
                kinematic_simulation_B.absxi = fabs
                  (kinematic_simulation_B.shift_im);
                if ((kinematic_simulation_B.absxr > 4.4942328371557893E+307) ||
                    (kinematic_simulation_B.absxi > 4.4942328371557893E+307)) {
                  kinematic_simulation_B.absxr *= 0.5;
                  kinematic_simulation_B.absxi *= 0.5;
                  kinematic_simulation_B.absxi = kinematic_simulat_rt_hypotd_snf
                    (kinematic_simulation_B.absxr, kinematic_simulation_B.absxi);
                  if (kinematic_simulation_B.absxi >
                      kinematic_simulation_B.absxr) {
                    kinematic_simulation_B.absxr = sqrt
                      (kinematic_simulation_B.absxr /
                       kinematic_simulation_B.absxi + 1.0) * sqrt
                      (kinematic_simulation_B.absxi);
                  } else {
                    kinematic_simulation_B.absxr = sqrt
                      (kinematic_simulation_B.absxi) * 1.4142135623730951;
                  }
                } else {
                  kinematic_simulation_B.absxr = sqrt
                    ((kinematic_simulat_rt_hypotd_snf
                      (kinematic_simulation_B.absxr,
                       kinematic_simulation_B.absxi) +
                      kinematic_simulation_B.absxr) * 0.5);
                }

                if (kinematic_simulation_B.shift_re > 0.0) {
                  kinematic_simulation_B.absxi = kinematic_simulation_B.shift_im
                    / kinematic_simulation_B.absxr * 0.5;
                } else {
                  if (kinematic_simulation_B.shift_im < 0.0) {
                    kinematic_simulation_B.absxi = -kinematic_simulation_B.absxr;
                  } else {
                    kinematic_simulation_B.absxi = kinematic_simulation_B.absxr;
                  }

                  kinematic_simulation_B.absxr = kinematic_simulation_B.shift_im
                    / kinematic_simulation_B.absxi * 0.5;
                }
              }

              if ((kinematic_simulation_B.t1_re - kinematic_simulation_B.ad22.re)
                  * kinematic_simulation_B.absxr + (kinematic_simulation_B.t1_im
                   - kinematic_simulation_B.ad22.im) *
                  kinematic_simulation_B.absxi <= 0.0) {
                kinematic_simulation_B.shift_re = kinematic_simulation_B.t1_re +
                  kinematic_simulation_B.absxr;
                kinematic_simulation_B.shift_im = kinematic_simulation_B.t1_im +
                  kinematic_simulation_B.absxi;
              } else {
                kinematic_simulation_B.shift_re = kinematic_simulation_B.t1_re -
                  kinematic_simulation_B.absxr;
                kinematic_simulation_B.shift_im = kinematic_simulation_B.t1_im -
                  kinematic_simulation_B.absxi;
              }
            } else {
              kinematic_simulation_B.j = (kinematic_simulation_B.ilastm1 << 2) +
                kinematic_simulation_B.ilast;
              kinematic_simulation_B.ar = A[kinematic_simulation_B.j].re *
                kinematic_simulation_B.anorm;
              kinematic_simulation_B.ai = A[kinematic_simulation_B.j].im *
                kinematic_simulation_B.anorm;
              if (kinematic_simulation_B.ai == 0.0) {
                kinematic_simulation_B.absxr = kinematic_simulation_B.ar / 0.5;
                kinematic_simulation_B.absxi = 0.0;
              } else if (kinematic_simulation_B.ar == 0.0) {
                kinematic_simulation_B.absxr = 0.0;
                kinematic_simulation_B.absxi = kinematic_simulation_B.ai / 0.5;
              } else {
                kinematic_simulation_B.absxr = kinematic_simulation_B.ar / 0.5;
                kinematic_simulation_B.absxi = kinematic_simulation_B.ai / 0.5;
              }

              kinematic_simulation_B.eshift_re += kinematic_simulation_B.absxr;
              kinematic_simulation_B.eshift_im += kinematic_simulation_B.absxi;
              kinematic_simulation_B.shift_re = kinematic_simulation_B.eshift_re;
              kinematic_simulation_B.shift_im = kinematic_simulation_B.eshift_im;
            }

            kinematic_simulation_B.j = kinematic_simulation_B.ilastm1;
            kinematic_simulation_B.jp1 = kinematic_simulation_B.ilastm1 + 1;
            exitg2 = false;
            while ((!exitg2) && (kinematic_simulation_B.j + 1 >
                                 kinematic_simulation_B.ifirst)) {
              kinematic_simulation_B.istart = kinematic_simulation_B.j + 1;
              kinematic_simulation_B.ctemp_tmp_tmp = kinematic_simulation_B.j <<
                2;
              kinematic_simulation_B.ctemp_tmp =
                kinematic_simulation_B.ctemp_tmp_tmp + kinematic_simulation_B.j;
              kinematic_simulation_B.ctemp.re =
                A[kinematic_simulation_B.ctemp_tmp].re *
                kinematic_simulation_B.anorm - kinematic_simulation_B.shift_re *
                0.5;
              kinematic_simulation_B.ctemp.im =
                A[kinematic_simulation_B.ctemp_tmp].im *
                kinematic_simulation_B.anorm - kinematic_simulation_B.shift_im *
                0.5;
              kinematic_simulation_B.t1_re = fabs
                (kinematic_simulation_B.ctemp.re) + fabs
                (kinematic_simulation_B.ctemp.im);
              kinematic_simulation_B.jp1 += kinematic_simulation_B.ctemp_tmp_tmp;
              kinematic_simulation_B.t1_im = (fabs(A[kinematic_simulation_B.jp1]
                .re) + fabs(A[kinematic_simulation_B.jp1].im)) *
                kinematic_simulation_B.anorm;
              kinematic_simulation_B.absxr = kinematic_simulation_B.t1_re;
              if (kinematic_simulation_B.t1_im > kinematic_simulation_B.t1_re) {
                kinematic_simulation_B.absxr = kinematic_simulation_B.t1_im;
              }

              if ((kinematic_simulation_B.absxr < 1.0) &&
                  (kinematic_simulation_B.absxr != 0.0)) {
                kinematic_simulation_B.t1_re /= kinematic_simulation_B.absxr;
                kinematic_simulation_B.t1_im /= kinematic_simulation_B.absxr;
              }

              kinematic_simulation_B.jp1 = ((kinematic_simulation_B.j - 1) << 2)
                + kinematic_simulation_B.j;
              if ((fabs(A[kinematic_simulation_B.jp1].re) + fabs
                   (A[kinematic_simulation_B.jp1].im)) *
                  kinematic_simulation_B.t1_im <= kinematic_simulation_B.t1_re *
                  kinematic_simulation_B.b_atol) {
                goto90 = true;
                exitg2 = true;
              } else {
                kinematic_simulation_B.jp1 = kinematic_simulation_B.j;
                kinematic_simulation_B.j--;
              }
            }

            if (!goto90) {
              kinematic_simulation_B.istart = kinematic_simulation_B.ifirst;
              kinematic_simulation_B.ctemp_tmp =
                (((kinematic_simulation_B.ifirst - 1) << 2) +
                 kinematic_simulation_B.ifirst) - 1;
              kinematic_simulation_B.ctemp.re =
                A[kinematic_simulation_B.ctemp_tmp].re *
                kinematic_simulation_B.anorm - kinematic_simulation_B.shift_re *
                0.5;
              kinematic_simulation_B.ctemp.im =
                A[kinematic_simulation_B.ctemp_tmp].im *
                kinematic_simulation_B.anorm - kinematic_simulation_B.shift_im *
                0.5;
            }

            goto90 = false;
            kinematic_simulation_B.j = ((kinematic_simulation_B.istart - 1) << 2)
              + kinematic_simulation_B.istart;
            kinematic_simulation_B.ascale.re = A[kinematic_simulation_B.j].re *
              kinematic_simulation_B.anorm;
            kinematic_simulation_B.ascale.im = A[kinematic_simulation_B.j].im *
              kinematic_simulation_B.anorm;
            kinematic_simulation_xzlartg_a(kinematic_simulation_B.ctemp,
              kinematic_simulation_B.ascale, &kinematic_simulation_B.t1_re,
              &kinematic_simulation_B.ad22);
            kinematic_simulation_B.j = kinematic_simulation_B.istart;
            kinematic_simulation_B.jp1 = kinematic_simulation_B.istart - 2;
            while (kinematic_simulation_B.j < kinematic_simulation_B.ilast + 1)
            {
              if (kinematic_simulation_B.j > kinematic_simulation_B.istart) {
                kinematic_simulation_xzlartg(A[(kinematic_simulation_B.j +
                  (kinematic_simulation_B.jp1 << 2)) - 1],
                  A[kinematic_simulation_B.j + (kinematic_simulation_B.jp1 << 2)],
                  &kinematic_simulation_B.t1_re, &kinematic_simulation_B.ad22,
                  &A[(kinematic_simulation_B.j + (kinematic_simulation_B.jp1 <<
                  2)) - 1]);
                kinematic_simulation_B.jp1 = kinematic_simulation_B.j +
                  (kinematic_simulation_B.jp1 << 2);
                A[kinematic_simulation_B.jp1].re = 0.0;
                A[kinematic_simulation_B.jp1].im = 0.0;
              }

              kinematic_simulation_B.ctemp_tmp = kinematic_simulation_B.j - 1;
              while (kinematic_simulation_B.ctemp_tmp + 1 < 5) {
                kinematic_simulation_B.jp1 = (kinematic_simulation_B.ctemp_tmp <<
                  2) + kinematic_simulation_B.j;
                kinematic_simulation_B.ctemp_tmp_tmp =
                  kinematic_simulation_B.jp1 - 1;
                kinematic_simulation_B.shift_re =
                  A[kinematic_simulation_B.ctemp_tmp_tmp].re *
                  kinematic_simulation_B.t1_re + (A[kinematic_simulation_B.jp1].
                  re * kinematic_simulation_B.ad22.re -
                  A[kinematic_simulation_B.jp1].im *
                  kinematic_simulation_B.ad22.im);
                kinematic_simulation_B.shift_im =
                  A[kinematic_simulation_B.ctemp_tmp_tmp].im *
                  kinematic_simulation_B.t1_re + (A[kinematic_simulation_B.jp1].
                  im * kinematic_simulation_B.ad22.re +
                  A[kinematic_simulation_B.jp1].re *
                  kinematic_simulation_B.ad22.im);
                kinematic_simulation_B.t1_im =
                  A[kinematic_simulation_B.ctemp_tmp_tmp].im;
                kinematic_simulation_B.absxr =
                  A[kinematic_simulation_B.ctemp_tmp_tmp].re;
                A[kinematic_simulation_B.jp1].re = A[kinematic_simulation_B.jp1]
                  .re * kinematic_simulation_B.t1_re -
                  (A[kinematic_simulation_B.ctemp_tmp_tmp].re *
                   kinematic_simulation_B.ad22.re +
                   A[kinematic_simulation_B.ctemp_tmp_tmp].im *
                   kinematic_simulation_B.ad22.im);
                A[kinematic_simulation_B.jp1].im = A[kinematic_simulation_B.jp1]
                  .im * kinematic_simulation_B.t1_re -
                  (kinematic_simulation_B.ad22.re * kinematic_simulation_B.t1_im
                   - kinematic_simulation_B.ad22.im *
                   kinematic_simulation_B.absxr);
                A[kinematic_simulation_B.ctemp_tmp_tmp].re =
                  kinematic_simulation_B.shift_re;
                A[kinematic_simulation_B.ctemp_tmp_tmp].im =
                  kinematic_simulation_B.shift_im;
                kinematic_simulation_B.ctemp_tmp++;
              }

              kinematic_simulation_B.ad22.re = -kinematic_simulation_B.ad22.re;
              kinematic_simulation_B.ad22.im = -kinematic_simulation_B.ad22.im;
              kinematic_simulation_B.ctemp_tmp = kinematic_simulation_B.j;
              if (kinematic_simulation_B.ilast + 1 < kinematic_simulation_B.j +
                  2) {
                kinematic_simulation_B.ctemp_tmp = kinematic_simulation_B.ilast
                  - 1;
              }

              kinematic_simulation_B.i_m = 0;
              while (kinematic_simulation_B.i_m + 1 <=
                     kinematic_simulation_B.ctemp_tmp + 2) {
                kinematic_simulation_B.jp1 = ((kinematic_simulation_B.j - 1) <<
                  2) + kinematic_simulation_B.i_m;
                kinematic_simulation_B.ctemp_tmp_tmp = (kinematic_simulation_B.j
                  << 2) + kinematic_simulation_B.i_m;
                kinematic_simulation_B.shift_re = (A[kinematic_simulation_B.jp1]
                  .re * kinematic_simulation_B.ad22.re -
                  A[kinematic_simulation_B.jp1].im *
                  kinematic_simulation_B.ad22.im) +
                  A[kinematic_simulation_B.ctemp_tmp_tmp].re *
                  kinematic_simulation_B.t1_re;
                kinematic_simulation_B.shift_im = (A[kinematic_simulation_B.jp1]
                  .im * kinematic_simulation_B.ad22.re +
                  A[kinematic_simulation_B.jp1].re *
                  kinematic_simulation_B.ad22.im) +
                  A[kinematic_simulation_B.ctemp_tmp_tmp].im *
                  kinematic_simulation_B.t1_re;
                kinematic_simulation_B.t1_im =
                  A[kinematic_simulation_B.ctemp_tmp_tmp].im;
                kinematic_simulation_B.absxr =
                  A[kinematic_simulation_B.ctemp_tmp_tmp].re;
                A[kinematic_simulation_B.jp1].re = A[kinematic_simulation_B.jp1]
                  .re * kinematic_simulation_B.t1_re -
                  (A[kinematic_simulation_B.ctemp_tmp_tmp].re *
                   kinematic_simulation_B.ad22.re +
                   A[kinematic_simulation_B.ctemp_tmp_tmp].im *
                   kinematic_simulation_B.ad22.im);
                A[kinematic_simulation_B.jp1].im = A[kinematic_simulation_B.jp1]
                  .im * kinematic_simulation_B.t1_re -
                  (kinematic_simulation_B.ad22.re * kinematic_simulation_B.t1_im
                   - kinematic_simulation_B.ad22.im *
                   kinematic_simulation_B.absxr);
                A[kinematic_simulation_B.ctemp_tmp_tmp].re =
                  kinematic_simulation_B.shift_re;
                A[kinematic_simulation_B.ctemp_tmp_tmp].im =
                  kinematic_simulation_B.shift_im;
                kinematic_simulation_B.i_m++;
              }

              kinematic_simulation_B.jp1 = (kinematic_simulation_B.j - 1) << 2;
              kinematic_simulation_B.ctemp_tmp_tmp = kinematic_simulation_B.j <<
                2;
              kinematic_simulation_B.shift_re = (Z[kinematic_simulation_B.jp1].
                re * kinematic_simulation_B.ad22.re -
                Z[kinematic_simulation_B.jp1].im *
                kinematic_simulation_B.ad22.im) +
                Z[kinematic_simulation_B.ctemp_tmp_tmp].re *
                kinematic_simulation_B.t1_re;
              kinematic_simulation_B.shift_im = (Z[kinematic_simulation_B.jp1].
                im * kinematic_simulation_B.ad22.re +
                Z[kinematic_simulation_B.jp1].re *
                kinematic_simulation_B.ad22.im) +
                Z[kinematic_simulation_B.ctemp_tmp_tmp].im *
                kinematic_simulation_B.t1_re;
              kinematic_simulation_B.t1_im =
                Z[kinematic_simulation_B.ctemp_tmp_tmp].im;
              kinematic_simulation_B.absxr =
                Z[kinematic_simulation_B.ctemp_tmp_tmp].re;
              Z[kinematic_simulation_B.jp1].re = Z[kinematic_simulation_B.jp1].
                re * kinematic_simulation_B.t1_re -
                (Z[kinematic_simulation_B.ctemp_tmp_tmp].re *
                 kinematic_simulation_B.ad22.re +
                 Z[kinematic_simulation_B.ctemp_tmp_tmp].im *
                 kinematic_simulation_B.ad22.im);
              Z[kinematic_simulation_B.jp1].im = Z[kinematic_simulation_B.jp1].
                im * kinematic_simulation_B.t1_re -
                (kinematic_simulation_B.ad22.re * kinematic_simulation_B.t1_im -
                 kinematic_simulation_B.ad22.im * kinematic_simulation_B.absxr);
              Z[kinematic_simulation_B.ctemp_tmp_tmp].re =
                kinematic_simulation_B.shift_re;
              Z[kinematic_simulation_B.ctemp_tmp_tmp].im =
                kinematic_simulation_B.shift_im;
              kinematic_simulation_B.ctemp_tmp = kinematic_simulation_B.jp1 + 1;
              kinematic_simulation_B.i_m = kinematic_simulation_B.ctemp_tmp_tmp
                + 1;
              kinematic_simulation_B.shift_re =
                (Z[kinematic_simulation_B.ctemp_tmp].re *
                 kinematic_simulation_B.ad22.re -
                 Z[kinematic_simulation_B.ctemp_tmp].im *
                 kinematic_simulation_B.ad22.im) + Z[kinematic_simulation_B.i_m]
                .re * kinematic_simulation_B.t1_re;
              kinematic_simulation_B.shift_im =
                (Z[kinematic_simulation_B.ctemp_tmp].im *
                 kinematic_simulation_B.ad22.re +
                 Z[kinematic_simulation_B.ctemp_tmp].re *
                 kinematic_simulation_B.ad22.im) + Z[kinematic_simulation_B.i_m]
                .im * kinematic_simulation_B.t1_re;
              kinematic_simulation_B.t1_im = Z[kinematic_simulation_B.i_m].im;
              kinematic_simulation_B.absxr = Z[kinematic_simulation_B.i_m].re;
              Z[kinematic_simulation_B.ctemp_tmp].re =
                Z[kinematic_simulation_B.ctemp_tmp].re *
                kinematic_simulation_B.t1_re - (Z[kinematic_simulation_B.i_m].re
                * kinematic_simulation_B.ad22.re + Z[kinematic_simulation_B.i_m]
                .im * kinematic_simulation_B.ad22.im);
              Z[kinematic_simulation_B.ctemp_tmp].im =
                Z[kinematic_simulation_B.ctemp_tmp].im *
                kinematic_simulation_B.t1_re - (kinematic_simulation_B.ad22.re *
                kinematic_simulation_B.t1_im - kinematic_simulation_B.ad22.im *
                kinematic_simulation_B.absxr);
              Z[kinematic_simulation_B.i_m].re = kinematic_simulation_B.shift_re;
              Z[kinematic_simulation_B.i_m].im = kinematic_simulation_B.shift_im;
              kinematic_simulation_B.ctemp_tmp = kinematic_simulation_B.jp1 + 2;
              kinematic_simulation_B.i_m = kinematic_simulation_B.ctemp_tmp_tmp
                + 2;
              kinematic_simulation_B.shift_re =
                (Z[kinematic_simulation_B.ctemp_tmp].re *
                 kinematic_simulation_B.ad22.re -
                 Z[kinematic_simulation_B.ctemp_tmp].im *
                 kinematic_simulation_B.ad22.im) + Z[kinematic_simulation_B.i_m]
                .re * kinematic_simulation_B.t1_re;
              kinematic_simulation_B.shift_im =
                (Z[kinematic_simulation_B.ctemp_tmp].im *
                 kinematic_simulation_B.ad22.re +
                 Z[kinematic_simulation_B.ctemp_tmp].re *
                 kinematic_simulation_B.ad22.im) + Z[kinematic_simulation_B.i_m]
                .im * kinematic_simulation_B.t1_re;
              kinematic_simulation_B.t1_im = Z[kinematic_simulation_B.i_m].im;
              kinematic_simulation_B.absxr = Z[kinematic_simulation_B.i_m].re;
              Z[kinematic_simulation_B.ctemp_tmp].re =
                Z[kinematic_simulation_B.ctemp_tmp].re *
                kinematic_simulation_B.t1_re - (Z[kinematic_simulation_B.i_m].re
                * kinematic_simulation_B.ad22.re + Z[kinematic_simulation_B.i_m]
                .im * kinematic_simulation_B.ad22.im);
              Z[kinematic_simulation_B.ctemp_tmp].im =
                Z[kinematic_simulation_B.ctemp_tmp].im *
                kinematic_simulation_B.t1_re - (kinematic_simulation_B.ad22.re *
                kinematic_simulation_B.t1_im - kinematic_simulation_B.ad22.im *
                kinematic_simulation_B.absxr);
              Z[kinematic_simulation_B.i_m].re = kinematic_simulation_B.shift_re;
              Z[kinematic_simulation_B.i_m].im = kinematic_simulation_B.shift_im;
              kinematic_simulation_B.jp1 += 3;
              kinematic_simulation_B.ctemp_tmp_tmp += 3;
              kinematic_simulation_B.shift_re = (Z[kinematic_simulation_B.jp1].
                re * kinematic_simulation_B.ad22.re -
                Z[kinematic_simulation_B.jp1].im *
                kinematic_simulation_B.ad22.im) +
                Z[kinematic_simulation_B.ctemp_tmp_tmp].re *
                kinematic_simulation_B.t1_re;
              kinematic_simulation_B.shift_im = (Z[kinematic_simulation_B.jp1].
                im * kinematic_simulation_B.ad22.re +
                Z[kinematic_simulation_B.jp1].re *
                kinematic_simulation_B.ad22.im) +
                Z[kinematic_simulation_B.ctemp_tmp_tmp].im *
                kinematic_simulation_B.t1_re;
              kinematic_simulation_B.t1_im =
                Z[kinematic_simulation_B.ctemp_tmp_tmp].im;
              kinematic_simulation_B.absxr =
                Z[kinematic_simulation_B.ctemp_tmp_tmp].re;
              Z[kinematic_simulation_B.jp1].re = Z[kinematic_simulation_B.jp1].
                re * kinematic_simulation_B.t1_re -
                (Z[kinematic_simulation_B.ctemp_tmp_tmp].re *
                 kinematic_simulation_B.ad22.re +
                 Z[kinematic_simulation_B.ctemp_tmp_tmp].im *
                 kinematic_simulation_B.ad22.im);
              Z[kinematic_simulation_B.jp1].im = Z[kinematic_simulation_B.jp1].
                im * kinematic_simulation_B.t1_re -
                (kinematic_simulation_B.ad22.re * kinematic_simulation_B.t1_im -
                 kinematic_simulation_B.ad22.im * kinematic_simulation_B.absxr);
              Z[kinematic_simulation_B.ctemp_tmp_tmp].re =
                kinematic_simulation_B.shift_re;
              Z[kinematic_simulation_B.ctemp_tmp_tmp].im =
                kinematic_simulation_B.shift_im;
              kinematic_simulation_B.jp1 = kinematic_simulation_B.j - 1;
              kinematic_simulation_B.j++;
            }
          }

          kinematic_simulation_B.jiter++;
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
      *info = kinematic_simulation_B.ilast + 1;
      kinematic_simulation_B.ifirst = 0;
      while (kinematic_simulation_B.ifirst <= kinematic_simulation_B.ilast) {
        alpha1[kinematic_simulation_B.ifirst].re = (rtNaN);
        alpha1[kinematic_simulation_B.ifirst].im = 0.0;
        beta1[kinematic_simulation_B.ifirst].re = (rtNaN);
        beta1[kinematic_simulation_B.ifirst].im = 0.0;
        kinematic_simulation_B.ifirst++;
      }

      for (kinematic_simulation_B.jp1 = 0; kinematic_simulation_B.jp1 < 16;
           kinematic_simulation_B.jp1++) {
        Z[kinematic_simulation_B.jp1].re = (rtNaN);
        Z[kinematic_simulation_B.jp1].im = 0.0;
      }
    } else {
      guard1 = true;
    }
  }

  if (guard1) {
    kinematic_simulation_B.ilast = 0;
    while (kinematic_simulation_B.ilast <= ilo - 2) {
      alpha1[kinematic_simulation_B.ilast] = A[(kinematic_simulation_B.ilast <<
        2) + kinematic_simulation_B.ilast];
      kinematic_simulation_B.ilast++;
    }
  }
}

// Function for MATLAB Function: '<S4>/MATLAB Function'
static void kinematic_simulation_xztgevc(const creal_T A[16], creal_T V[16])
{
  boolean_T lscalea;
  boolean_T lscaleb;
  int32_T work2_idx_1_re_tmp;
  int32_T d_re_tmp_tmp;
  kinematic_simulation_B.rworka[0] = 0.0;
  kinematic_simulation_B.rworka[2] = 0.0;
  kinematic_simulation_B.rworka[3] = 0.0;
  kinematic_simulation_B.anorm_c = fabs(A[0].re) + fabs(A[0].im);
  kinematic_simulation_B.rworka[1] = fabs(A[4].re) + fabs(A[4].im);
  kinematic_simulation_B.ascale_p = (fabs(A[5].re) + fabs(A[5].im)) +
    kinematic_simulation_B.rworka[1];
  if (kinematic_simulation_B.ascale_p > kinematic_simulation_B.anorm_c) {
    kinematic_simulation_B.anorm_c = kinematic_simulation_B.ascale_p;
  }

  kinematic_simulation_B.i_g = 0;
  while (kinematic_simulation_B.i_g <= 1) {
    kinematic_simulation_B.rworka[2] += fabs(A[kinematic_simulation_B.i_g + 8].
      re) + fabs(A[kinematic_simulation_B.i_g + 8].im);
    kinematic_simulation_B.i_g++;
  }

  kinematic_simulation_B.ascale_p = (fabs(A[10].re) + fabs(A[10].im)) +
    kinematic_simulation_B.rworka[2];
  if (kinematic_simulation_B.ascale_p > kinematic_simulation_B.anorm_c) {
    kinematic_simulation_B.anorm_c = kinematic_simulation_B.ascale_p;
  }

  kinematic_simulation_B.i_g = 0;
  while (kinematic_simulation_B.i_g <= 2) {
    kinematic_simulation_B.rworka[3] += fabs(A[kinematic_simulation_B.i_g + 12].
      re) + fabs(A[kinematic_simulation_B.i_g + 12].im);
    kinematic_simulation_B.i_g++;
  }

  kinematic_simulation_B.ascale_p = (fabs(A[15].re) + fabs(A[15].im)) +
    kinematic_simulation_B.rworka[3];
  if (kinematic_simulation_B.ascale_p > kinematic_simulation_B.anorm_c) {
    kinematic_simulation_B.anorm_c = kinematic_simulation_B.ascale_p;
  }

  kinematic_simulation_B.ascale_p = kinematic_simulation_B.anorm_c;
  if (2.2250738585072014E-308 > kinematic_simulation_B.anorm_c) {
    kinematic_simulation_B.ascale_p = 2.2250738585072014E-308;
  }

  kinematic_simulation_B.ascale_p = 1.0 / kinematic_simulation_B.ascale_p;
  for (kinematic_simulation_B.i_g = 0; kinematic_simulation_B.i_g < 4;
       kinematic_simulation_B.i_g++) {
    kinematic_simulation_B.c_x_tmp_tmp = (3 - kinematic_simulation_B.i_g) << 2;
    kinematic_simulation_B.c_x_tmp = (kinematic_simulation_B.c_x_tmp_tmp -
      kinematic_simulation_B.i_g) + 3;
    kinematic_simulation_B.salpha_re = (fabs(A[kinematic_simulation_B.c_x_tmp].
      re) + fabs(A[kinematic_simulation_B.c_x_tmp].im)) *
      kinematic_simulation_B.ascale_p;
    if (1.0 > kinematic_simulation_B.salpha_re) {
      kinematic_simulation_B.salpha_re = 1.0;
    }

    kinematic_simulation_B.temp = 1.0 / kinematic_simulation_B.salpha_re;
    kinematic_simulation_B.salpha_re = A[kinematic_simulation_B.c_x_tmp].re *
      kinematic_simulation_B.temp * kinematic_simulation_B.ascale_p;
    kinematic_simulation_B.salpha_im = A[kinematic_simulation_B.c_x_tmp].im *
      kinematic_simulation_B.temp * kinematic_simulation_B.ascale_p;
    kinematic_simulation_B.acoeff = kinematic_simulation_B.temp *
      kinematic_simulation_B.ascale_p;
    lscalea = ((kinematic_simulation_B.temp >= 2.2250738585072014E-308) &&
               (kinematic_simulation_B.acoeff < 4.0083367200179456E-292));
    kinematic_simulation_B.dmin = fabs(kinematic_simulation_B.salpha_re) + fabs
      (kinematic_simulation_B.salpha_im);
    lscaleb = ((kinematic_simulation_B.dmin >= 2.2250738585072014E-308) &&
               (kinematic_simulation_B.dmin < 4.0083367200179456E-292));
    kinematic_simulation_B.scale_p = 1.0;
    if (lscalea) {
      kinematic_simulation_B.scale_p = kinematic_simulation_B.anorm_c;
      if (2.4948003869184E+291 < kinematic_simulation_B.anorm_c) {
        kinematic_simulation_B.scale_p = 2.4948003869184E+291;
      }

      kinematic_simulation_B.scale_p *= 4.0083367200179456E-292 /
        kinematic_simulation_B.temp;
    }

    if (lscaleb) {
      kinematic_simulation_B.work2_idx_2_im = 4.0083367200179456E-292 /
        kinematic_simulation_B.dmin;
      if (kinematic_simulation_B.work2_idx_2_im > kinematic_simulation_B.scale_p)
      {
        kinematic_simulation_B.scale_p = kinematic_simulation_B.work2_idx_2_im;
      }
    }

    if (lscalea || lscaleb) {
      kinematic_simulation_B.work2_idx_2_im = kinematic_simulation_B.acoeff;
      if (1.0 > kinematic_simulation_B.acoeff) {
        kinematic_simulation_B.work2_idx_2_im = 1.0;
      }

      if (kinematic_simulation_B.dmin > kinematic_simulation_B.work2_idx_2_im) {
        kinematic_simulation_B.work2_idx_2_im = kinematic_simulation_B.dmin;
      }

      kinematic_simulation_B.dmin = 1.0 / (2.2250738585072014E-308 *
        kinematic_simulation_B.work2_idx_2_im);
      if (kinematic_simulation_B.dmin < kinematic_simulation_B.scale_p) {
        kinematic_simulation_B.scale_p = kinematic_simulation_B.dmin;
      }

      if (lscalea) {
        kinematic_simulation_B.acoeff = kinematic_simulation_B.scale_p *
          kinematic_simulation_B.temp * kinematic_simulation_B.ascale_p;
      } else {
        kinematic_simulation_B.acoeff *= kinematic_simulation_B.scale_p;
      }

      kinematic_simulation_B.salpha_re *= kinematic_simulation_B.scale_p;
      kinematic_simulation_B.salpha_im *= kinematic_simulation_B.scale_p;
    }

    memset(&kinematic_simulation_B.work1[0], 0, sizeof(creal_T) << 2U);
    kinematic_simulation_B.work1[3 - kinematic_simulation_B.i_g].re = 1.0;
    kinematic_simulation_B.work1[3 - kinematic_simulation_B.i_g].im = 0.0;
    kinematic_simulation_B.dmin = 2.2204460492503131E-16 *
      kinematic_simulation_B.acoeff * kinematic_simulation_B.anorm_c;
    kinematic_simulation_B.temp = (fabs(kinematic_simulation_B.salpha_re) + fabs
      (kinematic_simulation_B.salpha_im)) * 2.2204460492503131E-16;
    if (kinematic_simulation_B.temp > kinematic_simulation_B.dmin) {
      kinematic_simulation_B.dmin = kinematic_simulation_B.temp;
    }

    if (2.2250738585072014E-308 > kinematic_simulation_B.dmin) {
      kinematic_simulation_B.dmin = 2.2250738585072014E-308;
    }

    kinematic_simulation_B.c_x_tmp = 0;
    while (kinematic_simulation_B.c_x_tmp <= 2 - kinematic_simulation_B.i_g) {
      kinematic_simulation_B.d_re_tmp = kinematic_simulation_B.c_x_tmp_tmp +
        kinematic_simulation_B.c_x_tmp;
      kinematic_simulation_B.work1[kinematic_simulation_B.c_x_tmp].re =
        A[kinematic_simulation_B.d_re_tmp].re * kinematic_simulation_B.acoeff;
      kinematic_simulation_B.work1[kinematic_simulation_B.c_x_tmp].im =
        A[kinematic_simulation_B.d_re_tmp].im * kinematic_simulation_B.acoeff;
      kinematic_simulation_B.c_x_tmp++;
    }

    kinematic_simulation_B.work1[3 - kinematic_simulation_B.i_g].re = 1.0;
    kinematic_simulation_B.work1[3 - kinematic_simulation_B.i_g].im = 0.0;
    kinematic_simulation_B.c_x_tmp = static_cast<int32_T>(((-1.0 - ((-
      static_cast<real_T>(kinematic_simulation_B.i_g) + 4.0) - 1.0)) + 1.0) /
      -1.0);
    kinematic_simulation_B.c_j_e = 0;
    while (kinematic_simulation_B.c_j_e <= kinematic_simulation_B.c_x_tmp - 1) {
      work2_idx_1_re_tmp = 2 - (kinematic_simulation_B.i_g +
        kinematic_simulation_B.c_j_e);
      d_re_tmp_tmp = work2_idx_1_re_tmp << 2;
      kinematic_simulation_B.d_re_tmp = d_re_tmp_tmp + work2_idx_1_re_tmp;
      kinematic_simulation_B.work2_idx_3_re = A[kinematic_simulation_B.d_re_tmp]
        .re * kinematic_simulation_B.acoeff - kinematic_simulation_B.salpha_re;
      kinematic_simulation_B.scale_p = A[kinematic_simulation_B.d_re_tmp].im *
        kinematic_simulation_B.acoeff - kinematic_simulation_B.salpha_im;
      if (fabs(kinematic_simulation_B.work2_idx_3_re) + fabs
          (kinematic_simulation_B.scale_p) <= kinematic_simulation_B.dmin) {
        kinematic_simulation_B.work2_idx_3_re = kinematic_simulation_B.dmin;
        kinematic_simulation_B.scale_p = 0.0;
      }

      kinematic_simulation_B.work2_idx_2_im = fabs
        (kinematic_simulation_B.work2_idx_3_re);
      kinematic_simulation_B.work2_idx_3_im = fabs
        (kinematic_simulation_B.scale_p);
      kinematic_simulation_B.temp = kinematic_simulation_B.work2_idx_2_im +
        kinematic_simulation_B.work2_idx_3_im;
      if (kinematic_simulation_B.temp < 1.0) {
        kinematic_simulation_B.f_y = fabs
          (kinematic_simulation_B.work1[work2_idx_1_re_tmp].re) + fabs
          (kinematic_simulation_B.work1[work2_idx_1_re_tmp].im);
        if (kinematic_simulation_B.f_y >= kinematic_simulation_B.temp *
            1.1235582092889474E+307) {
          kinematic_simulation_B.temp = 1.0 / kinematic_simulation_B.f_y;
          kinematic_simulation_B.d_re_tmp = 0;
          while (kinematic_simulation_B.d_re_tmp <= 3 -
                 kinematic_simulation_B.i_g) {
            kinematic_simulation_B.work1[kinematic_simulation_B.d_re_tmp].re *=
              kinematic_simulation_B.temp;
            kinematic_simulation_B.work1[kinematic_simulation_B.d_re_tmp].im *=
              kinematic_simulation_B.temp;
            kinematic_simulation_B.d_re_tmp++;
          }
        }
      }

      if (kinematic_simulation_B.scale_p == 0.0) {
        if (-kinematic_simulation_B.work1[work2_idx_1_re_tmp].im == 0.0) {
          kinematic_simulation_B.temp =
            -kinematic_simulation_B.work1[work2_idx_1_re_tmp].re /
            kinematic_simulation_B.work2_idx_3_re;
          kinematic_simulation_B.scale_p = 0.0;
        } else if (-kinematic_simulation_B.work1[work2_idx_1_re_tmp].re == 0.0)
        {
          kinematic_simulation_B.temp = 0.0;
          kinematic_simulation_B.scale_p =
            -kinematic_simulation_B.work1[work2_idx_1_re_tmp].im /
            kinematic_simulation_B.work2_idx_3_re;
        } else {
          kinematic_simulation_B.temp =
            -kinematic_simulation_B.work1[work2_idx_1_re_tmp].re /
            kinematic_simulation_B.work2_idx_3_re;
          kinematic_simulation_B.scale_p =
            -kinematic_simulation_B.work1[work2_idx_1_re_tmp].im /
            kinematic_simulation_B.work2_idx_3_re;
        }
      } else if (kinematic_simulation_B.work2_idx_3_re == 0.0) {
        if (-kinematic_simulation_B.work1[work2_idx_1_re_tmp].re == 0.0) {
          kinematic_simulation_B.temp =
            -kinematic_simulation_B.work1[work2_idx_1_re_tmp].im /
            kinematic_simulation_B.scale_p;
          kinematic_simulation_B.scale_p = 0.0;
        } else if (-kinematic_simulation_B.work1[work2_idx_1_re_tmp].im == 0.0)
        {
          kinematic_simulation_B.temp = 0.0;
          kinematic_simulation_B.scale_p =
            -(-kinematic_simulation_B.work1[work2_idx_1_re_tmp].re /
              kinematic_simulation_B.scale_p);
        } else {
          kinematic_simulation_B.temp =
            -kinematic_simulation_B.work1[work2_idx_1_re_tmp].im /
            kinematic_simulation_B.scale_p;
          kinematic_simulation_B.scale_p =
            -(-kinematic_simulation_B.work1[work2_idx_1_re_tmp].re /
              kinematic_simulation_B.scale_p);
        }
      } else if (kinematic_simulation_B.work2_idx_2_im >
                 kinematic_simulation_B.work2_idx_3_im) {
        kinematic_simulation_B.work2_idx_2_im = kinematic_simulation_B.scale_p /
          kinematic_simulation_B.work2_idx_3_re;
        kinematic_simulation_B.scale_p = kinematic_simulation_B.work2_idx_2_im *
          kinematic_simulation_B.scale_p + kinematic_simulation_B.work2_idx_3_re;
        kinematic_simulation_B.temp = (kinematic_simulation_B.work2_idx_2_im *
          -kinematic_simulation_B.work1[work2_idx_1_re_tmp].im +
          -kinematic_simulation_B.work1[work2_idx_1_re_tmp].re) /
          kinematic_simulation_B.scale_p;
        kinematic_simulation_B.scale_p =
          (-kinematic_simulation_B.work1[work2_idx_1_re_tmp].im -
           kinematic_simulation_B.work2_idx_2_im *
           -kinematic_simulation_B.work1[work2_idx_1_re_tmp].re) /
          kinematic_simulation_B.scale_p;
      } else if (kinematic_simulation_B.work2_idx_3_im ==
                 kinematic_simulation_B.work2_idx_2_im) {
        kinematic_simulation_B.work2_idx_3_re =
          kinematic_simulation_B.work2_idx_3_re > 0.0 ? 0.5 : -0.5;
        kinematic_simulation_B.scale_p = kinematic_simulation_B.scale_p > 0.0 ?
          0.5 : -0.5;
        kinematic_simulation_B.temp =
          (-kinematic_simulation_B.work1[work2_idx_1_re_tmp].re *
           kinematic_simulation_B.work2_idx_3_re +
           -kinematic_simulation_B.work1[work2_idx_1_re_tmp].im *
           kinematic_simulation_B.scale_p) /
          kinematic_simulation_B.work2_idx_2_im;
        kinematic_simulation_B.scale_p =
          (-kinematic_simulation_B.work1[work2_idx_1_re_tmp].im *
           kinematic_simulation_B.work2_idx_3_re -
           -kinematic_simulation_B.work1[work2_idx_1_re_tmp].re *
           kinematic_simulation_B.scale_p) /
          kinematic_simulation_B.work2_idx_2_im;
      } else {
        kinematic_simulation_B.work2_idx_2_im =
          kinematic_simulation_B.work2_idx_3_re / kinematic_simulation_B.scale_p;
        kinematic_simulation_B.scale_p += kinematic_simulation_B.work2_idx_2_im *
          kinematic_simulation_B.work2_idx_3_re;
        kinematic_simulation_B.temp = (kinematic_simulation_B.work2_idx_2_im *
          -kinematic_simulation_B.work1[work2_idx_1_re_tmp].re +
          -kinematic_simulation_B.work1[work2_idx_1_re_tmp].im) /
          kinematic_simulation_B.scale_p;
        kinematic_simulation_B.scale_p = (kinematic_simulation_B.work2_idx_2_im *
          -kinematic_simulation_B.work1[work2_idx_1_re_tmp].im -
          (-kinematic_simulation_B.work1[work2_idx_1_re_tmp].re)) /
          kinematic_simulation_B.scale_p;
      }

      kinematic_simulation_B.work1[work2_idx_1_re_tmp].re =
        kinematic_simulation_B.temp;
      kinematic_simulation_B.work1[work2_idx_1_re_tmp].im =
        kinematic_simulation_B.scale_p;
      if (work2_idx_1_re_tmp + 1 > 1) {
        if (fabs(kinematic_simulation_B.work1[work2_idx_1_re_tmp].re) + fabs
            (kinematic_simulation_B.work1[work2_idx_1_re_tmp].im) > 1.0) {
          kinematic_simulation_B.temp = 1.0 / (fabs
            (kinematic_simulation_B.work1[work2_idx_1_re_tmp].re) + fabs
            (kinematic_simulation_B.work1[work2_idx_1_re_tmp].im));
          if (kinematic_simulation_B.acoeff *
              kinematic_simulation_B.rworka[work2_idx_1_re_tmp] >=
              1.1235582092889474E+307 * kinematic_simulation_B.temp) {
            kinematic_simulation_B.d_re_tmp = 0;
            while (kinematic_simulation_B.d_re_tmp <= 3 -
                   kinematic_simulation_B.i_g) {
              kinematic_simulation_B.work1[kinematic_simulation_B.d_re_tmp].re *=
                kinematic_simulation_B.temp;
              kinematic_simulation_B.work1[kinematic_simulation_B.d_re_tmp].im *=
                kinematic_simulation_B.temp;
              kinematic_simulation_B.d_re_tmp++;
            }
          }
        }

        kinematic_simulation_B.work2_idx_3_re = kinematic_simulation_B.acoeff *
          kinematic_simulation_B.work1[work2_idx_1_re_tmp].re;
        kinematic_simulation_B.scale_p = kinematic_simulation_B.acoeff *
          kinematic_simulation_B.work1[work2_idx_1_re_tmp].im;
        kinematic_simulation_B.e_jr = 0;
        while (kinematic_simulation_B.e_jr <= work2_idx_1_re_tmp - 1) {
          kinematic_simulation_B.d_re_tmp = d_re_tmp_tmp +
            kinematic_simulation_B.e_jr;
          kinematic_simulation_B.work1[kinematic_simulation_B.e_jr].re +=
            A[kinematic_simulation_B.d_re_tmp].re *
            kinematic_simulation_B.work2_idx_3_re -
            A[kinematic_simulation_B.d_re_tmp].im *
            kinematic_simulation_B.scale_p;
          kinematic_simulation_B.work1[kinematic_simulation_B.e_jr].im +=
            A[kinematic_simulation_B.d_re_tmp].im *
            kinematic_simulation_B.work2_idx_3_re +
            A[kinematic_simulation_B.d_re_tmp].re *
            kinematic_simulation_B.scale_p;
          kinematic_simulation_B.e_jr++;
        }
      }

      kinematic_simulation_B.c_j_e++;
    }

    kinematic_simulation_B.salpha_re = 0.0;
    kinematic_simulation_B.salpha_im = 0.0;
    kinematic_simulation_B.acoeff = 0.0;
    kinematic_simulation_B.dmin = 0.0;
    kinematic_simulation_B.scale_p = 0.0;
    kinematic_simulation_B.work2_idx_2_im = 0.0;
    kinematic_simulation_B.work2_idx_3_re = 0.0;
    kinematic_simulation_B.work2_idx_3_im = 0.0;
    kinematic_simulation_B.c_x_tmp = 0;
    while (kinematic_simulation_B.c_x_tmp <= 3 - kinematic_simulation_B.i_g) {
      kinematic_simulation_B.c_j_e = kinematic_simulation_B.c_x_tmp << 2;
      kinematic_simulation_B.salpha_re += V[kinematic_simulation_B.c_j_e].re *
        kinematic_simulation_B.work1[kinematic_simulation_B.c_x_tmp].re -
        V[kinematic_simulation_B.c_j_e].im *
        kinematic_simulation_B.work1[kinematic_simulation_B.c_x_tmp].im;
      kinematic_simulation_B.salpha_im += V[kinematic_simulation_B.c_j_e].re *
        kinematic_simulation_B.work1[kinematic_simulation_B.c_x_tmp].im +
        V[kinematic_simulation_B.c_j_e].im *
        kinematic_simulation_B.work1[kinematic_simulation_B.c_x_tmp].re;
      work2_idx_1_re_tmp = kinematic_simulation_B.c_j_e + 1;
      kinematic_simulation_B.acoeff += V[work2_idx_1_re_tmp].re *
        kinematic_simulation_B.work1[kinematic_simulation_B.c_x_tmp].re -
        V[work2_idx_1_re_tmp].im *
        kinematic_simulation_B.work1[kinematic_simulation_B.c_x_tmp].im;
      kinematic_simulation_B.dmin += V[work2_idx_1_re_tmp].re *
        kinematic_simulation_B.work1[kinematic_simulation_B.c_x_tmp].im +
        V[work2_idx_1_re_tmp].im *
        kinematic_simulation_B.work1[kinematic_simulation_B.c_x_tmp].re;
      work2_idx_1_re_tmp = kinematic_simulation_B.c_j_e + 2;
      kinematic_simulation_B.scale_p += V[work2_idx_1_re_tmp].re *
        kinematic_simulation_B.work1[kinematic_simulation_B.c_x_tmp].re -
        V[work2_idx_1_re_tmp].im *
        kinematic_simulation_B.work1[kinematic_simulation_B.c_x_tmp].im;
      kinematic_simulation_B.work2_idx_2_im += V[work2_idx_1_re_tmp].re *
        kinematic_simulation_B.work1[kinematic_simulation_B.c_x_tmp].im +
        V[work2_idx_1_re_tmp].im *
        kinematic_simulation_B.work1[kinematic_simulation_B.c_x_tmp].re;
      kinematic_simulation_B.c_j_e += 3;
      kinematic_simulation_B.work2_idx_3_re += V[kinematic_simulation_B.c_j_e].
        re * kinematic_simulation_B.work1[kinematic_simulation_B.c_x_tmp].re -
        V[kinematic_simulation_B.c_j_e].im *
        kinematic_simulation_B.work1[kinematic_simulation_B.c_x_tmp].im;
      kinematic_simulation_B.work2_idx_3_im += V[kinematic_simulation_B.c_j_e].
        re * kinematic_simulation_B.work1[kinematic_simulation_B.c_x_tmp].im +
        V[kinematic_simulation_B.c_j_e].im *
        kinematic_simulation_B.work1[kinematic_simulation_B.c_x_tmp].re;
      kinematic_simulation_B.c_x_tmp++;
    }

    kinematic_simulation_B.temp = fabs(kinematic_simulation_B.salpha_re) + fabs
      (kinematic_simulation_B.salpha_im);
    kinematic_simulation_B.f_y = fabs(kinematic_simulation_B.acoeff) + fabs
      (kinematic_simulation_B.dmin);
    if (kinematic_simulation_B.f_y > kinematic_simulation_B.temp) {
      kinematic_simulation_B.temp = kinematic_simulation_B.f_y;
    }

    kinematic_simulation_B.f_y = fabs(kinematic_simulation_B.scale_p) + fabs
      (kinematic_simulation_B.work2_idx_2_im);
    if (kinematic_simulation_B.f_y > kinematic_simulation_B.temp) {
      kinematic_simulation_B.temp = kinematic_simulation_B.f_y;
    }

    kinematic_simulation_B.f_y = fabs(kinematic_simulation_B.work2_idx_3_re) +
      fabs(kinematic_simulation_B.work2_idx_3_im);
    if (kinematic_simulation_B.f_y > kinematic_simulation_B.temp) {
      kinematic_simulation_B.temp = kinematic_simulation_B.f_y;
    }

    if (kinematic_simulation_B.temp > 2.2250738585072014E-308) {
      kinematic_simulation_B.temp = 1.0 / kinematic_simulation_B.temp;
      V[kinematic_simulation_B.c_x_tmp_tmp].re = kinematic_simulation_B.temp *
        kinematic_simulation_B.salpha_re;
      V[kinematic_simulation_B.c_x_tmp_tmp].im = kinematic_simulation_B.temp *
        kinematic_simulation_B.salpha_im;
      kinematic_simulation_B.d_re_tmp = ((3 - kinematic_simulation_B.i_g) << 2)
        + 1;
      V[kinematic_simulation_B.d_re_tmp].re = kinematic_simulation_B.temp *
        kinematic_simulation_B.acoeff;
      V[kinematic_simulation_B.d_re_tmp].im = kinematic_simulation_B.temp *
        kinematic_simulation_B.dmin;
      kinematic_simulation_B.d_re_tmp = ((3 - kinematic_simulation_B.i_g) << 2)
        + 2;
      V[kinematic_simulation_B.d_re_tmp].re = kinematic_simulation_B.temp *
        kinematic_simulation_B.scale_p;
      V[kinematic_simulation_B.d_re_tmp].im = kinematic_simulation_B.temp *
        kinematic_simulation_B.work2_idx_2_im;
      kinematic_simulation_B.d_re_tmp = ((3 - kinematic_simulation_B.i_g) << 2)
        + 3;
      V[kinematic_simulation_B.d_re_tmp].re = kinematic_simulation_B.temp *
        kinematic_simulation_B.work2_idx_3_re;
      V[kinematic_simulation_B.d_re_tmp].im = kinematic_simulation_B.temp *
        kinematic_simulation_B.work2_idx_3_im;
    } else {
      V[kinematic_simulation_B.c_x_tmp_tmp].re = 0.0;
      V[kinematic_simulation_B.c_x_tmp_tmp].im = 0.0;
      kinematic_simulation_B.d_re_tmp = kinematic_simulation_B.c_x_tmp_tmp + 1;
      V[kinematic_simulation_B.d_re_tmp].re = 0.0;
      V[kinematic_simulation_B.d_re_tmp].im = 0.0;
      kinematic_simulation_B.d_re_tmp = kinematic_simulation_B.c_x_tmp_tmp + 2;
      V[kinematic_simulation_B.d_re_tmp].re = 0.0;
      V[kinematic_simulation_B.d_re_tmp].im = 0.0;
      kinematic_simulation_B.d_re_tmp = kinematic_simulation_B.c_x_tmp_tmp + 3;
      V[kinematic_simulation_B.d_re_tmp].re = 0.0;
      V[kinematic_simulation_B.d_re_tmp].im = 0.0;
    }
  }
}

// Function for MATLAB Function: '<S4>/MATLAB Function'
static void kinematic_simulation_xzggev(creal_T A[16], int32_T *info, creal_T
  alpha1[4], creal_T beta1[4], creal_T V[16])
{
  boolean_T ilascl;
  boolean_T found;
  int32_T exitg1;
  int32_T exitg2;
  boolean_T exitg3;
  boolean_T exitg4;
  *info = 0;
  kinematic_simulation_B.anrm = kinematic_simulation_xzlangeM(A);
  if (rtIsInf(kinematic_simulation_B.anrm) || rtIsNaN
      (kinematic_simulation_B.anrm)) {
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
    for (kinematic_simulation_B.k_m = 0; kinematic_simulation_B.k_m < 16;
         kinematic_simulation_B.k_m++) {
      V[kinematic_simulation_B.k_m].re = (rtNaN);
      V[kinematic_simulation_B.k_m].im = 0.0;
    }
  } else {
    ilascl = false;
    kinematic_simulation_B.anrmto = kinematic_simulation_B.anrm;
    if ((kinematic_simulation_B.anrm > 0.0) && (kinematic_simulation_B.anrm <
         6.7178761075670888E-139)) {
      kinematic_simulation_B.anrmto = 6.7178761075670888E-139;
      ilascl = true;
      kinematic_simulation_xzlascl(kinematic_simulation_B.anrm,
        kinematic_simulation_B.anrmto, A);
    } else {
      if (kinematic_simulation_B.anrm > 1.4885657073574029E+138) {
        kinematic_simulation_B.anrmto = 1.4885657073574029E+138;
        ilascl = true;
        kinematic_simulation_xzlascl(kinematic_simulation_B.anrm,
          kinematic_simulation_B.anrmto, A);
      }
    }

    kinematic_simulation_B.rscale[0] = 1;
    kinematic_simulation_B.rscale[1] = 1;
    kinematic_simulation_B.rscale[2] = 1;
    kinematic_simulation_B.rscale[3] = 1;
    kinematic_simulation_B.c_i = 0;
    kinematic_simulation_B.ihi = 4;
    do {
      exitg2 = 0;
      kinematic_simulation_B.i_h = 0;
      kinematic_simulation_B.jcol = 0;
      found = false;
      kinematic_simulation_B.ii = kinematic_simulation_B.ihi;
      exitg3 = false;
      while ((!exitg3) && (kinematic_simulation_B.ii > 0)) {
        kinematic_simulation_B.nzcount = 0;
        kinematic_simulation_B.i_h = kinematic_simulation_B.ii;
        kinematic_simulation_B.jcol = kinematic_simulation_B.ihi;
        kinematic_simulation_B.jj = 0;
        exitg4 = false;
        while ((!exitg4) && (kinematic_simulation_B.jj <=
                             kinematic_simulation_B.ihi - 1)) {
          kinematic_simulation_B.k_m = ((kinematic_simulation_B.jj << 2) +
            kinematic_simulation_B.ii) - 1;
          if ((A[kinematic_simulation_B.k_m].re != 0.0) ||
              (A[kinematic_simulation_B.k_m].im != 0.0) ||
              (kinematic_simulation_B.jj + 1 == kinematic_simulation_B.ii)) {
            if (kinematic_simulation_B.nzcount == 0) {
              kinematic_simulation_B.jcol = kinematic_simulation_B.jj + 1;
              kinematic_simulation_B.nzcount = 1;
              kinematic_simulation_B.jj++;
            } else {
              kinematic_simulation_B.nzcount = 2;
              exitg4 = true;
            }
          } else {
            kinematic_simulation_B.jj++;
          }
        }

        if (kinematic_simulation_B.nzcount < 2) {
          found = true;
          exitg3 = true;
        } else {
          kinematic_simulation_B.ii--;
        }
      }

      if (!found) {
        exitg2 = 2;
      } else {
        if (kinematic_simulation_B.i_h != kinematic_simulation_B.ihi) {
          kinematic_simulation_B.atmp_re = A[kinematic_simulation_B.i_h - 1].re;
          kinematic_simulation_B.atmp_im = A[kinematic_simulation_B.i_h - 1].im;
          A[kinematic_simulation_B.i_h - 1] = A[kinematic_simulation_B.ihi - 1];
          A[kinematic_simulation_B.ihi - 1].re = kinematic_simulation_B.atmp_re;
          A[kinematic_simulation_B.ihi - 1].im = kinematic_simulation_B.atmp_im;
          kinematic_simulation_B.atmp_re = A[kinematic_simulation_B.i_h + 3].re;
          kinematic_simulation_B.atmp_im = A[kinematic_simulation_B.i_h + 3].im;
          A[kinematic_simulation_B.i_h + 3] = A[kinematic_simulation_B.ihi + 3];
          A[kinematic_simulation_B.ihi + 3].re = kinematic_simulation_B.atmp_re;
          A[kinematic_simulation_B.ihi + 3].im = kinematic_simulation_B.atmp_im;
          kinematic_simulation_B.atmp_re = A[kinematic_simulation_B.i_h + 7].re;
          kinematic_simulation_B.atmp_im = A[kinematic_simulation_B.i_h + 7].im;
          A[kinematic_simulation_B.i_h + 7] = A[kinematic_simulation_B.ihi + 7];
          A[kinematic_simulation_B.ihi + 7].re = kinematic_simulation_B.atmp_re;
          A[kinematic_simulation_B.ihi + 7].im = kinematic_simulation_B.atmp_im;
          kinematic_simulation_B.atmp_re = A[kinematic_simulation_B.i_h + 11].re;
          kinematic_simulation_B.atmp_im = A[kinematic_simulation_B.i_h + 11].im;
          A[kinematic_simulation_B.i_h + 11] = A[kinematic_simulation_B.ihi + 11];
          A[kinematic_simulation_B.ihi + 11].re = kinematic_simulation_B.atmp_re;
          A[kinematic_simulation_B.ihi + 11].im = kinematic_simulation_B.atmp_im;
        }

        if (kinematic_simulation_B.jcol != kinematic_simulation_B.ihi) {
          kinematic_simulation_B.ii = 0;
          while (kinematic_simulation_B.ii <= kinematic_simulation_B.ihi - 1) {
            kinematic_simulation_B.i_h = ((kinematic_simulation_B.jcol - 1) << 2)
              + kinematic_simulation_B.ii;
            kinematic_simulation_B.atmp_re = A[kinematic_simulation_B.i_h].re;
            kinematic_simulation_B.atmp_im = A[kinematic_simulation_B.i_h].im;
            kinematic_simulation_B.k_m = ((kinematic_simulation_B.ihi - 1) << 2)
              + kinematic_simulation_B.ii;
            A[kinematic_simulation_B.i_h] = A[kinematic_simulation_B.k_m];
            A[kinematic_simulation_B.k_m].re = kinematic_simulation_B.atmp_re;
            A[kinematic_simulation_B.k_m].im = kinematic_simulation_B.atmp_im;
            kinematic_simulation_B.ii++;
          }
        }

        kinematic_simulation_B.rscale[kinematic_simulation_B.ihi - 1] =
          kinematic_simulation_B.jcol;
        kinematic_simulation_B.ihi--;
        if (kinematic_simulation_B.ihi == 1) {
          kinematic_simulation_B.rscale[0] = 1;
          exitg2 = 1;
        }
      }
    } while (exitg2 == 0);

    if (exitg2 == 1) {
    } else {
      do {
        exitg1 = 0;
        kinematic_simulation_B.ii = 0;
        kinematic_simulation_B.jcol = 0;
        found = false;
        kinematic_simulation_B.i_h = kinematic_simulation_B.c_i + 1;
        exitg3 = false;
        while ((!exitg3) && (kinematic_simulation_B.i_h <=
                             kinematic_simulation_B.ihi)) {
          kinematic_simulation_B.nzcount = 0;
          kinematic_simulation_B.ii = kinematic_simulation_B.ihi;
          kinematic_simulation_B.jcol = kinematic_simulation_B.i_h;
          kinematic_simulation_B.jj = kinematic_simulation_B.c_i + 1;
          exitg4 = false;
          while ((!exitg4) && (kinematic_simulation_B.jj <=
                               kinematic_simulation_B.ihi)) {
            kinematic_simulation_B.k_m = (((kinematic_simulation_B.i_h - 1) << 2)
              + kinematic_simulation_B.jj) - 1;
            if ((A[kinematic_simulation_B.k_m].re != 0.0) ||
                (A[kinematic_simulation_B.k_m].im != 0.0) ||
                (kinematic_simulation_B.jj == kinematic_simulation_B.i_h)) {
              if (kinematic_simulation_B.nzcount == 0) {
                kinematic_simulation_B.ii = kinematic_simulation_B.jj;
                kinematic_simulation_B.nzcount = 1;
                kinematic_simulation_B.jj++;
              } else {
                kinematic_simulation_B.nzcount = 2;
                exitg4 = true;
              }
            } else {
              kinematic_simulation_B.jj++;
            }
          }

          if (kinematic_simulation_B.nzcount < 2) {
            found = true;
            exitg3 = true;
          } else {
            kinematic_simulation_B.i_h++;
          }
        }

        if (!found) {
          exitg1 = 1;
        } else {
          if (kinematic_simulation_B.c_i + 1 != kinematic_simulation_B.ii) {
            kinematic_simulation_B.nzcount = kinematic_simulation_B.c_i;
            while (kinematic_simulation_B.nzcount + 1 < 5) {
              kinematic_simulation_B.k_m = kinematic_simulation_B.nzcount << 2;
              kinematic_simulation_B.i_h = (kinematic_simulation_B.k_m +
                kinematic_simulation_B.ii) - 1;
              kinematic_simulation_B.atmp_re = A[kinematic_simulation_B.i_h].re;
              kinematic_simulation_B.atmp_im = A[kinematic_simulation_B.i_h].im;
              kinematic_simulation_B.k_m += kinematic_simulation_B.c_i;
              A[kinematic_simulation_B.i_h] = A[kinematic_simulation_B.k_m];
              A[kinematic_simulation_B.k_m].re = kinematic_simulation_B.atmp_re;
              A[kinematic_simulation_B.k_m].im = kinematic_simulation_B.atmp_im;
              kinematic_simulation_B.nzcount++;
            }
          }

          if (kinematic_simulation_B.c_i + 1 != kinematic_simulation_B.jcol) {
            kinematic_simulation_B.ii = 0;
            while (kinematic_simulation_B.ii <= kinematic_simulation_B.ihi - 1)
            {
              kinematic_simulation_B.i_h = ((kinematic_simulation_B.jcol - 1) <<
                2) + kinematic_simulation_B.ii;
              kinematic_simulation_B.atmp_re = A[kinematic_simulation_B.i_h].re;
              kinematic_simulation_B.atmp_im = A[kinematic_simulation_B.i_h].im;
              kinematic_simulation_B.k_m = (kinematic_simulation_B.c_i << 2) +
                kinematic_simulation_B.ii;
              A[kinematic_simulation_B.i_h] = A[kinematic_simulation_B.k_m];
              A[kinematic_simulation_B.k_m].re = kinematic_simulation_B.atmp_re;
              A[kinematic_simulation_B.k_m].im = kinematic_simulation_B.atmp_im;
              kinematic_simulation_B.ii++;
            }
          }

          kinematic_simulation_B.rscale[kinematic_simulation_B.c_i] =
            kinematic_simulation_B.jcol;
          kinematic_simulation_B.c_i++;
          if (kinematic_simulation_B.c_i + 1 == kinematic_simulation_B.ihi) {
            kinematic_simulation_B.rscale[kinematic_simulation_B.c_i] =
              kinematic_simulation_B.c_i + 1;
            exitg1 = 1;
          }
        }
      } while (exitg1 == 0);
    }

    for (kinematic_simulation_B.k_m = 0; kinematic_simulation_B.k_m < 16;
         kinematic_simulation_B.k_m++) {
      kinematic_simulation_B.b_I[kinematic_simulation_B.k_m] = 0;
    }

    kinematic_simulation_B.b_I[0] = 1;
    kinematic_simulation_B.b_I[5] = 1;
    kinematic_simulation_B.b_I[10] = 1;
    kinematic_simulation_B.b_I[15] = 1;
    for (kinematic_simulation_B.k_m = 0; kinematic_simulation_B.k_m < 16;
         kinematic_simulation_B.k_m++) {
      V[kinematic_simulation_B.k_m].re =
        kinematic_simulation_B.b_I[kinematic_simulation_B.k_m];
      V[kinematic_simulation_B.k_m].im = 0.0;
    }

    if (kinematic_simulation_B.ihi >= kinematic_simulation_B.c_i + 3) {
      kinematic_simulation_B.jcol = kinematic_simulation_B.c_i;
      while (kinematic_simulation_B.jcol + 1 < kinematic_simulation_B.ihi - 1) {
        kinematic_simulation_B.ii = kinematic_simulation_B.ihi - 1;
        while (kinematic_simulation_B.ii + 1 > kinematic_simulation_B.jcol + 2)
        {
          kinematic_simulation_xzlartg(A[(kinematic_simulation_B.ii +
            (kinematic_simulation_B.jcol << 2)) - 1],
            A[kinematic_simulation_B.ii + (kinematic_simulation_B.jcol << 2)],
            &kinematic_simulation_B.mul, &kinematic_simulation_B.s, &A
            [(kinematic_simulation_B.ii + (kinematic_simulation_B.jcol << 2)) -
            1]);
          kinematic_simulation_B.k_m = kinematic_simulation_B.ii +
            (kinematic_simulation_B.jcol << 2);
          A[kinematic_simulation_B.k_m].re = 0.0;
          A[kinematic_simulation_B.k_m].im = 0.0;
          kinematic_simulation_B.nzcount = kinematic_simulation_B.jcol + 1;
          while (kinematic_simulation_B.nzcount + 1 < 5) {
            kinematic_simulation_B.i_h = (kinematic_simulation_B.nzcount << 2) +
              kinematic_simulation_B.ii;
            kinematic_simulation_B.k_m = kinematic_simulation_B.i_h - 1;
            kinematic_simulation_B.atmp_re = A[kinematic_simulation_B.k_m].re *
              kinematic_simulation_B.mul + (A[kinematic_simulation_B.i_h].re *
              kinematic_simulation_B.s.re - A[kinematic_simulation_B.i_h].im *
              kinematic_simulation_B.s.im);
            kinematic_simulation_B.atmp_im = A[kinematic_simulation_B.k_m].im *
              kinematic_simulation_B.mul + (A[kinematic_simulation_B.i_h].im *
              kinematic_simulation_B.s.re + A[kinematic_simulation_B.i_h].re *
              kinematic_simulation_B.s.im);
            kinematic_simulation_B.d_j = A[kinematic_simulation_B.k_m].im;
            kinematic_simulation_B.d1 = A[kinematic_simulation_B.k_m].re;
            A[kinematic_simulation_B.i_h].re = A[kinematic_simulation_B.i_h].re *
              kinematic_simulation_B.mul - (A[kinematic_simulation_B.k_m].re *
              kinematic_simulation_B.s.re + A[kinematic_simulation_B.k_m].im *
              kinematic_simulation_B.s.im);
            A[kinematic_simulation_B.i_h].im = A[kinematic_simulation_B.i_h].im *
              kinematic_simulation_B.mul - (kinematic_simulation_B.s.re *
              kinematic_simulation_B.d_j - kinematic_simulation_B.s.im *
              kinematic_simulation_B.d1);
            A[kinematic_simulation_B.k_m].re = kinematic_simulation_B.atmp_re;
            A[kinematic_simulation_B.k_m].im = kinematic_simulation_B.atmp_im;
            kinematic_simulation_B.nzcount++;
          }

          kinematic_simulation_B.s.re = -kinematic_simulation_B.s.re;
          kinematic_simulation_B.s.im = -kinematic_simulation_B.s.im;
          kinematic_simulation_B.nzcount = 0;
          while (kinematic_simulation_B.nzcount + 1 <=
                 kinematic_simulation_B.ihi) {
            kinematic_simulation_B.i_h = ((kinematic_simulation_B.ii - 1) << 2)
              + kinematic_simulation_B.nzcount;
            kinematic_simulation_B.k_m = (kinematic_simulation_B.ii << 2) +
              kinematic_simulation_B.nzcount;
            kinematic_simulation_B.atmp_re = (A[kinematic_simulation_B.i_h].re *
              kinematic_simulation_B.s.re - A[kinematic_simulation_B.i_h].im *
              kinematic_simulation_B.s.im) + A[kinematic_simulation_B.k_m].re *
              kinematic_simulation_B.mul;
            kinematic_simulation_B.atmp_im = (A[kinematic_simulation_B.i_h].im *
              kinematic_simulation_B.s.re + A[kinematic_simulation_B.i_h].re *
              kinematic_simulation_B.s.im) + A[kinematic_simulation_B.k_m].im *
              kinematic_simulation_B.mul;
            kinematic_simulation_B.d_j = A[kinematic_simulation_B.k_m].im;
            kinematic_simulation_B.d1 = A[kinematic_simulation_B.k_m].re;
            A[kinematic_simulation_B.i_h].re = A[kinematic_simulation_B.i_h].re *
              kinematic_simulation_B.mul - (A[kinematic_simulation_B.k_m].re *
              kinematic_simulation_B.s.re + A[kinematic_simulation_B.k_m].im *
              kinematic_simulation_B.s.im);
            A[kinematic_simulation_B.i_h].im = A[kinematic_simulation_B.i_h].im *
              kinematic_simulation_B.mul - (kinematic_simulation_B.s.re *
              kinematic_simulation_B.d_j - kinematic_simulation_B.s.im *
              kinematic_simulation_B.d1);
            A[kinematic_simulation_B.k_m].re = kinematic_simulation_B.atmp_re;
            A[kinematic_simulation_B.k_m].im = kinematic_simulation_B.atmp_im;
            kinematic_simulation_B.nzcount++;
          }

          kinematic_simulation_B.i_h = (kinematic_simulation_B.ii - 1) << 2;
          kinematic_simulation_B.k_m = kinematic_simulation_B.ii << 2;
          kinematic_simulation_B.atmp_re = (V[kinematic_simulation_B.i_h].re *
            kinematic_simulation_B.s.re - V[kinematic_simulation_B.i_h].im *
            kinematic_simulation_B.s.im) + V[kinematic_simulation_B.k_m].re *
            kinematic_simulation_B.mul;
          kinematic_simulation_B.atmp_im = (V[kinematic_simulation_B.i_h].im *
            kinematic_simulation_B.s.re + V[kinematic_simulation_B.i_h].re *
            kinematic_simulation_B.s.im) + V[kinematic_simulation_B.k_m].im *
            kinematic_simulation_B.mul;
          kinematic_simulation_B.d_j = V[kinematic_simulation_B.k_m].re;
          V[kinematic_simulation_B.i_h].re = V[kinematic_simulation_B.i_h].re *
            kinematic_simulation_B.mul - (V[kinematic_simulation_B.k_m].re *
            kinematic_simulation_B.s.re + V[kinematic_simulation_B.k_m].im *
            kinematic_simulation_B.s.im);
          V[kinematic_simulation_B.i_h].im = V[kinematic_simulation_B.i_h].im *
            kinematic_simulation_B.mul - (V[kinematic_simulation_B.k_m].im *
            kinematic_simulation_B.s.re - kinematic_simulation_B.s.im *
            kinematic_simulation_B.d_j);
          V[kinematic_simulation_B.k_m].re = kinematic_simulation_B.atmp_re;
          V[kinematic_simulation_B.k_m].im = kinematic_simulation_B.atmp_im;
          kinematic_simulation_B.nzcount = kinematic_simulation_B.i_h + 1;
          kinematic_simulation_B.jj = kinematic_simulation_B.k_m + 1;
          kinematic_simulation_B.atmp_re = (V[kinematic_simulation_B.nzcount].re
            * kinematic_simulation_B.s.re - V[kinematic_simulation_B.nzcount].im
            * kinematic_simulation_B.s.im) + V[kinematic_simulation_B.jj].re *
            kinematic_simulation_B.mul;
          kinematic_simulation_B.atmp_im = (V[kinematic_simulation_B.nzcount].im
            * kinematic_simulation_B.s.re + V[kinematic_simulation_B.nzcount].re
            * kinematic_simulation_B.s.im) + V[kinematic_simulation_B.jj].im *
            kinematic_simulation_B.mul;
          kinematic_simulation_B.d_j = V[kinematic_simulation_B.jj].re;
          V[kinematic_simulation_B.nzcount].re =
            V[kinematic_simulation_B.nzcount].re * kinematic_simulation_B.mul -
            (V[kinematic_simulation_B.jj].re * kinematic_simulation_B.s.re +
             V[kinematic_simulation_B.jj].im * kinematic_simulation_B.s.im);
          V[kinematic_simulation_B.nzcount].im =
            V[kinematic_simulation_B.nzcount].im * kinematic_simulation_B.mul -
            (V[kinematic_simulation_B.jj].im * kinematic_simulation_B.s.re -
             kinematic_simulation_B.s.im * kinematic_simulation_B.d_j);
          V[kinematic_simulation_B.jj].re = kinematic_simulation_B.atmp_re;
          V[kinematic_simulation_B.jj].im = kinematic_simulation_B.atmp_im;
          kinematic_simulation_B.nzcount = kinematic_simulation_B.i_h + 2;
          kinematic_simulation_B.jj = kinematic_simulation_B.k_m + 2;
          kinematic_simulation_B.atmp_re = (V[kinematic_simulation_B.nzcount].re
            * kinematic_simulation_B.s.re - V[kinematic_simulation_B.nzcount].im
            * kinematic_simulation_B.s.im) + V[kinematic_simulation_B.jj].re *
            kinematic_simulation_B.mul;
          kinematic_simulation_B.atmp_im = (V[kinematic_simulation_B.nzcount].im
            * kinematic_simulation_B.s.re + V[kinematic_simulation_B.nzcount].re
            * kinematic_simulation_B.s.im) + V[kinematic_simulation_B.jj].im *
            kinematic_simulation_B.mul;
          kinematic_simulation_B.d_j = V[kinematic_simulation_B.jj].re;
          V[kinematic_simulation_B.nzcount].re =
            V[kinematic_simulation_B.nzcount].re * kinematic_simulation_B.mul -
            (V[kinematic_simulation_B.jj].re * kinematic_simulation_B.s.re +
             V[kinematic_simulation_B.jj].im * kinematic_simulation_B.s.im);
          V[kinematic_simulation_B.nzcount].im =
            V[kinematic_simulation_B.nzcount].im * kinematic_simulation_B.mul -
            (V[kinematic_simulation_B.jj].im * kinematic_simulation_B.s.re -
             kinematic_simulation_B.s.im * kinematic_simulation_B.d_j);
          V[kinematic_simulation_B.jj].re = kinematic_simulation_B.atmp_re;
          V[kinematic_simulation_B.jj].im = kinematic_simulation_B.atmp_im;
          kinematic_simulation_B.i_h += 3;
          kinematic_simulation_B.k_m += 3;
          kinematic_simulation_B.atmp_re = (V[kinematic_simulation_B.i_h].re *
            kinematic_simulation_B.s.re - V[kinematic_simulation_B.i_h].im *
            kinematic_simulation_B.s.im) + V[kinematic_simulation_B.k_m].re *
            kinematic_simulation_B.mul;
          kinematic_simulation_B.atmp_im = (V[kinematic_simulation_B.i_h].im *
            kinematic_simulation_B.s.re + V[kinematic_simulation_B.i_h].re *
            kinematic_simulation_B.s.im) + V[kinematic_simulation_B.k_m].im *
            kinematic_simulation_B.mul;
          kinematic_simulation_B.d_j = V[kinematic_simulation_B.k_m].re;
          V[kinematic_simulation_B.i_h].re = V[kinematic_simulation_B.i_h].re *
            kinematic_simulation_B.mul - (V[kinematic_simulation_B.k_m].re *
            kinematic_simulation_B.s.re + V[kinematic_simulation_B.k_m].im *
            kinematic_simulation_B.s.im);
          V[kinematic_simulation_B.i_h].im = V[kinematic_simulation_B.i_h].im *
            kinematic_simulation_B.mul - (V[kinematic_simulation_B.k_m].im *
            kinematic_simulation_B.s.re - kinematic_simulation_B.s.im *
            kinematic_simulation_B.d_j);
          V[kinematic_simulation_B.k_m].re = kinematic_simulation_B.atmp_re;
          V[kinematic_simulation_B.k_m].im = kinematic_simulation_B.atmp_im;
          kinematic_simulation_B.ii--;
        }

        kinematic_simulation_B.jcol++;
      }
    }

    kinematic_simulation_xzhgeqz(A, kinematic_simulation_B.c_i + 1,
      kinematic_simulation_B.ihi, V, info, alpha1, beta1);
    if (*info == 0) {
      kinematic_simulation_xztgevc(A, V);
      if (kinematic_simulation_B.c_i + 1 > 1) {
        kinematic_simulation_B.c_i--;
        while (kinematic_simulation_B.c_i + 1 >= 1) {
          kinematic_simulation_B.k_m =
            kinematic_simulation_B.rscale[kinematic_simulation_B.c_i] - 1;
          if (kinematic_simulation_B.c_i + 1 !=
              kinematic_simulation_B.rscale[kinematic_simulation_B.c_i]) {
            kinematic_simulation_B.atmp_re = V[kinematic_simulation_B.c_i].re;
            kinematic_simulation_B.atmp_im = V[kinematic_simulation_B.c_i].im;
            V[kinematic_simulation_B.c_i] = V[kinematic_simulation_B.k_m];
            V[kinematic_simulation_B.k_m].re = kinematic_simulation_B.atmp_re;
            V[kinematic_simulation_B.k_m].im = kinematic_simulation_B.atmp_im;
            kinematic_simulation_B.atmp_re = V[kinematic_simulation_B.c_i + 4].
              re;
            kinematic_simulation_B.atmp_im = V[kinematic_simulation_B.c_i + 4].
              im;
            V[kinematic_simulation_B.c_i + 4] = V[kinematic_simulation_B.k_m + 4];
            V[kinematic_simulation_B.k_m + 4].re =
              kinematic_simulation_B.atmp_re;
            V[kinematic_simulation_B.k_m + 4].im =
              kinematic_simulation_B.atmp_im;
            kinematic_simulation_B.atmp_re = V[kinematic_simulation_B.c_i + 8].
              re;
            kinematic_simulation_B.atmp_im = V[kinematic_simulation_B.c_i + 8].
              im;
            V[kinematic_simulation_B.c_i + 8] = V[kinematic_simulation_B.k_m + 8];
            V[kinematic_simulation_B.k_m + 8].re =
              kinematic_simulation_B.atmp_re;
            V[kinematic_simulation_B.k_m + 8].im =
              kinematic_simulation_B.atmp_im;
            kinematic_simulation_B.atmp_re = V[kinematic_simulation_B.c_i + 12].
              re;
            kinematic_simulation_B.atmp_im = V[kinematic_simulation_B.c_i + 12].
              im;
            V[kinematic_simulation_B.c_i + 12] = V[kinematic_simulation_B.k_m +
              12];
            V[kinematic_simulation_B.k_m + 12].re =
              kinematic_simulation_B.atmp_re;
            V[kinematic_simulation_B.k_m + 12].im =
              kinematic_simulation_B.atmp_im;
          }

          kinematic_simulation_B.c_i--;
        }
      }

      if (kinematic_simulation_B.ihi < 4) {
        while (kinematic_simulation_B.ihi + 1 < 5) {
          kinematic_simulation_B.k_m =
            kinematic_simulation_B.rscale[kinematic_simulation_B.ihi] - 1;
          if (kinematic_simulation_B.ihi + 1 !=
              kinematic_simulation_B.rscale[kinematic_simulation_B.ihi]) {
            kinematic_simulation_B.atmp_re = V[kinematic_simulation_B.ihi].re;
            kinematic_simulation_B.atmp_im = V[kinematic_simulation_B.ihi].im;
            V[kinematic_simulation_B.ihi] = V[kinematic_simulation_B.k_m];
            V[kinematic_simulation_B.k_m].re = kinematic_simulation_B.atmp_re;
            V[kinematic_simulation_B.k_m].im = kinematic_simulation_B.atmp_im;
            kinematic_simulation_B.atmp_re = V[kinematic_simulation_B.ihi + 4].
              re;
            kinematic_simulation_B.atmp_im = V[kinematic_simulation_B.ihi + 4].
              im;
            V[kinematic_simulation_B.ihi + 4] = V[kinematic_simulation_B.k_m + 4];
            V[kinematic_simulation_B.k_m + 4].re =
              kinematic_simulation_B.atmp_re;
            V[kinematic_simulation_B.k_m + 4].im =
              kinematic_simulation_B.atmp_im;
            kinematic_simulation_B.atmp_re = V[kinematic_simulation_B.ihi + 8].
              re;
            kinematic_simulation_B.atmp_im = V[kinematic_simulation_B.ihi + 8].
              im;
            V[kinematic_simulation_B.ihi + 8] = V[kinematic_simulation_B.k_m + 8];
            V[kinematic_simulation_B.k_m + 8].re =
              kinematic_simulation_B.atmp_re;
            V[kinematic_simulation_B.k_m + 8].im =
              kinematic_simulation_B.atmp_im;
            kinematic_simulation_B.atmp_re = V[kinematic_simulation_B.ihi + 12].
              re;
            kinematic_simulation_B.atmp_im = V[kinematic_simulation_B.ihi + 12].
              im;
            V[kinematic_simulation_B.ihi + 12] = V[kinematic_simulation_B.k_m +
              12];
            V[kinematic_simulation_B.k_m + 12].re =
              kinematic_simulation_B.atmp_re;
            V[kinematic_simulation_B.k_m + 12].im =
              kinematic_simulation_B.atmp_im;
          }

          kinematic_simulation_B.ihi++;
        }
      }

      for (kinematic_simulation_B.ihi = 0; kinematic_simulation_B.ihi < 4;
           kinematic_simulation_B.ihi++) {
        kinematic_simulation_B.c_i = kinematic_simulation_B.ihi << 2;
        kinematic_simulation_B.atmp_re = fabs(V[kinematic_simulation_B.c_i].re)
          + fabs(V[kinematic_simulation_B.c_i].im);
        kinematic_simulation_B.k_m = kinematic_simulation_B.c_i + 1;
        kinematic_simulation_B.atmp_im = fabs(V[kinematic_simulation_B.k_m].re)
          + fabs(V[kinematic_simulation_B.k_m].im);
        if (kinematic_simulation_B.atmp_im > kinematic_simulation_B.atmp_re) {
          kinematic_simulation_B.atmp_re = kinematic_simulation_B.atmp_im;
        }

        kinematic_simulation_B.i_h = kinematic_simulation_B.c_i + 2;
        kinematic_simulation_B.atmp_im = fabs(V[kinematic_simulation_B.i_h].re)
          + fabs(V[kinematic_simulation_B.i_h].im);
        if (kinematic_simulation_B.atmp_im > kinematic_simulation_B.atmp_re) {
          kinematic_simulation_B.atmp_re = kinematic_simulation_B.atmp_im;
        }

        kinematic_simulation_B.jcol = kinematic_simulation_B.c_i + 3;
        kinematic_simulation_B.atmp_im = fabs(V[kinematic_simulation_B.jcol].re)
          + fabs(V[kinematic_simulation_B.jcol].im);
        if (kinematic_simulation_B.atmp_im > kinematic_simulation_B.atmp_re) {
          kinematic_simulation_B.atmp_re = kinematic_simulation_B.atmp_im;
        }

        if (kinematic_simulation_B.atmp_re >= 6.7178761075670888E-139) {
          kinematic_simulation_B.atmp_re = 1.0 / kinematic_simulation_B.atmp_re;
          V[kinematic_simulation_B.c_i].re *= kinematic_simulation_B.atmp_re;
          V[kinematic_simulation_B.c_i].im *= kinematic_simulation_B.atmp_re;
          V[kinematic_simulation_B.k_m].re *= kinematic_simulation_B.atmp_re;
          V[kinematic_simulation_B.k_m].im *= kinematic_simulation_B.atmp_re;
          V[kinematic_simulation_B.i_h].re *= kinematic_simulation_B.atmp_re;
          V[kinematic_simulation_B.i_h].im *= kinematic_simulation_B.atmp_re;
          V[kinematic_simulation_B.jcol].re *= kinematic_simulation_B.atmp_re;
          V[kinematic_simulation_B.jcol].im *= kinematic_simulation_B.atmp_re;
        }
      }

      if (ilascl) {
        ilascl = true;
        while (ilascl) {
          kinematic_simulation_B.atmp_re = kinematic_simulation_B.anrmto *
            2.0041683600089728E-292;
          kinematic_simulation_B.atmp_im = kinematic_simulation_B.anrm /
            4.9896007738368E+291;
          if ((fabs(kinematic_simulation_B.atmp_re) > fabs
               (kinematic_simulation_B.anrm)) && (kinematic_simulation_B.anrm !=
               0.0)) {
            kinematic_simulation_B.mul = 2.0041683600089728E-292;
            kinematic_simulation_B.anrmto = kinematic_simulation_B.atmp_re;
          } else if (fabs(kinematic_simulation_B.atmp_im) > fabs
                     (kinematic_simulation_B.anrmto)) {
            kinematic_simulation_B.mul = 4.9896007738368E+291;
            kinematic_simulation_B.anrm = kinematic_simulation_B.atmp_im;
          } else {
            kinematic_simulation_B.mul = kinematic_simulation_B.anrm /
              kinematic_simulation_B.anrmto;
            ilascl = false;
          }

          alpha1[0].re *= kinematic_simulation_B.mul;
          alpha1[0].im *= kinematic_simulation_B.mul;
          alpha1[1].re *= kinematic_simulation_B.mul;
          alpha1[1].im *= kinematic_simulation_B.mul;
          alpha1[2].re *= kinematic_simulation_B.mul;
          alpha1[2].im *= kinematic_simulation_B.mul;
          alpha1[3].re *= kinematic_simulation_B.mul;
          alpha1[3].im *= kinematic_simulation_B.mul;
        }
      }
    }
  }
}

// Function for MATLAB Function: '<S4>/MATLAB Function'
static real_T kinematic_simulation_xnrm2(int32_T n, const real_T x[16], int32_T
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

// Function for MATLAB Function: '<S4>/MATLAB Function'
static void kinematic_simulation_xzlarf(int32_T m, int32_T n, int32_T iv0,
  real_T tau, real_T C[16], int32_T ic0, real_T work[4])
{
  int32_T d;
  int32_T b_ia;
  int32_T jy;
  int32_T exitg1;
  boolean_T exitg2;
  if (tau != 0.0) {
    kinematic_simulation_B.lastv = m;
    kinematic_simulation_B.lastc_h = iv0 + m;
    while ((kinematic_simulation_B.lastv > 0) &&
           (C[kinematic_simulation_B.lastc_h - 2] == 0.0)) {
      kinematic_simulation_B.lastv--;
      kinematic_simulation_B.lastc_h--;
    }

    kinematic_simulation_B.lastc_h = n - 1;
    exitg2 = false;
    while ((!exitg2) && (kinematic_simulation_B.lastc_h + 1 > 0)) {
      kinematic_simulation_B.coltop = (kinematic_simulation_B.lastc_h << 2) +
        ic0;
      jy = kinematic_simulation_B.coltop;
      do {
        exitg1 = 0;
        if (jy <= (kinematic_simulation_B.coltop + kinematic_simulation_B.lastv)
            - 1) {
          if (C[jy - 1] != 0.0) {
            exitg1 = 1;
          } else {
            jy++;
          }
        } else {
          kinematic_simulation_B.lastc_h--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    kinematic_simulation_B.lastv = 0;
    kinematic_simulation_B.lastc_h = -1;
  }

  if (kinematic_simulation_B.lastv > 0) {
    if (kinematic_simulation_B.lastc_h + 1 != 0) {
      kinematic_simulation_B.coltop = 0;
      while (kinematic_simulation_B.coltop <= kinematic_simulation_B.lastc_h) {
        work[kinematic_simulation_B.coltop] = 0.0;
        kinematic_simulation_B.coltop++;
      }

      kinematic_simulation_B.coltop = 0;
      jy = (kinematic_simulation_B.lastc_h << 2) + ic0;
      kinematic_simulation_B.iac_c = ic0;
      while (kinematic_simulation_B.iac_c <= jy) {
        kinematic_simulation_B.ix_e = iv0;
        kinematic_simulation_B.c_a = 0.0;
        d = (kinematic_simulation_B.iac_c + kinematic_simulation_B.lastv) - 1;
        for (b_ia = kinematic_simulation_B.iac_c; b_ia <= d; b_ia++) {
          kinematic_simulation_B.c_a += C[b_ia - 1] *
            C[kinematic_simulation_B.ix_e - 1];
          kinematic_simulation_B.ix_e++;
        }

        work[kinematic_simulation_B.coltop] += kinematic_simulation_B.c_a;
        kinematic_simulation_B.coltop++;
        kinematic_simulation_B.iac_c += 4;
      }
    }

    if (!(-tau == 0.0)) {
      kinematic_simulation_B.coltop = ic0 - 1;
      jy = 0;
      kinematic_simulation_B.iac_c = 0;
      while (kinematic_simulation_B.iac_c <= kinematic_simulation_B.lastc_h) {
        if (work[jy] != 0.0) {
          kinematic_simulation_B.c_a = work[jy] * -tau;
          kinematic_simulation_B.ix_e = iv0;
          d = kinematic_simulation_B.lastv + kinematic_simulation_B.coltop;
          for (b_ia = kinematic_simulation_B.coltop; b_ia < d; b_ia++) {
            C[b_ia] += C[kinematic_simulation_B.ix_e - 1] *
              kinematic_simulation_B.c_a;
            kinematic_simulation_B.ix_e++;
          }
        }

        jy++;
        kinematic_simulation_B.coltop += 4;
        kinematic_simulation_B.iac_c++;
      }
    }
  }
}

// Function for MATLAB Function: '<S4>/MATLAB Function'
static void kinematic_simulation_xgehrd(real_T a[16], real_T tau[3])
{
  int32_T exitg1;
  boolean_T exitg2;
  kinematic_simulation_B.work_g[0] = 0.0;
  kinematic_simulation_B.work_g[1] = 0.0;
  kinematic_simulation_B.work_g[2] = 0.0;
  kinematic_simulation_B.work_g[3] = 0.0;
  kinematic_simulation_B.alpha1 = a[1];
  tau[0] = 0.0;
  kinematic_simulation_B.xnorm = kinematic_simulation_xnrm2(2, a, 3);
  if (kinematic_simulation_B.xnorm != 0.0) {
    kinematic_simulation_B.xnorm = kinematic_simulat_rt_hypotd_snf(a[1],
      kinematic_simulation_B.xnorm);
    if (a[1] >= 0.0) {
      kinematic_simulation_B.xnorm = -kinematic_simulation_B.xnorm;
    }

    if (fabs(kinematic_simulation_B.xnorm) < 1.0020841800044864E-292) {
      kinematic_simulation_B.knt = -1;
      do {
        kinematic_simulation_B.knt++;
        kinematic_simulation_B.lastc = 3;
        while (kinematic_simulation_B.lastc <= 4) {
          a[kinematic_simulation_B.lastc - 1] *= 9.9792015476736E+291;
          kinematic_simulation_B.lastc++;
        }

        kinematic_simulation_B.xnorm *= 9.9792015476736E+291;
        kinematic_simulation_B.alpha1 *= 9.9792015476736E+291;
      } while (!(fabs(kinematic_simulation_B.xnorm) >= 1.0020841800044864E-292));

      kinematic_simulation_B.xnorm = kinematic_simulat_rt_hypotd_snf
        (kinematic_simulation_B.alpha1, kinematic_simulation_xnrm2(2, a, 3));
      if (kinematic_simulation_B.alpha1 >= 0.0) {
        kinematic_simulation_B.xnorm = -kinematic_simulation_B.xnorm;
      }

      tau[0] = (kinematic_simulation_B.xnorm - kinematic_simulation_B.alpha1) /
        kinematic_simulation_B.xnorm;
      kinematic_simulation_B.alpha1 = 1.0 / (kinematic_simulation_B.alpha1 -
        kinematic_simulation_B.xnorm);
      kinematic_simulation_B.lastc = 3;
      while (kinematic_simulation_B.lastc <= 4) {
        a[kinematic_simulation_B.lastc - 1] *= kinematic_simulation_B.alpha1;
        kinematic_simulation_B.lastc++;
      }

      kinematic_simulation_B.lastc = 0;
      while (kinematic_simulation_B.lastc <= kinematic_simulation_B.knt) {
        kinematic_simulation_B.xnorm *= 1.0020841800044864E-292;
        kinematic_simulation_B.lastc++;
      }

      kinematic_simulation_B.alpha1 = kinematic_simulation_B.xnorm;
    } else {
      tau[0] = (kinematic_simulation_B.xnorm - a[1]) /
        kinematic_simulation_B.xnorm;
      kinematic_simulation_B.alpha1 = 1.0 / (a[1] - kinematic_simulation_B.xnorm);
      kinematic_simulation_B.knt = 3;
      while (kinematic_simulation_B.knt <= 4) {
        a[kinematic_simulation_B.knt - 1] *= kinematic_simulation_B.alpha1;
        kinematic_simulation_B.knt++;
      }

      kinematic_simulation_B.alpha1 = kinematic_simulation_B.xnorm;
    }
  }

  a[1] = 1.0;
  if (tau[0] != 0.0) {
    kinematic_simulation_B.knt = 2;
    kinematic_simulation_B.lastc = 3;
    while ((kinematic_simulation_B.knt + 1 > 0) &&
           (a[kinematic_simulation_B.lastc] == 0.0)) {
      kinematic_simulation_B.knt--;
      kinematic_simulation_B.lastc--;
    }

    kinematic_simulation_B.lastc = 4;
    exitg2 = false;
    while ((!exitg2) && (kinematic_simulation_B.lastc > 0)) {
      kinematic_simulation_B.rowleft = kinematic_simulation_B.lastc + 4;
      kinematic_simulation_B.jy = kinematic_simulation_B.rowleft;
      do {
        exitg1 = 0;
        if (kinematic_simulation_B.jy <= (kinematic_simulation_B.knt << 2) +
            kinematic_simulation_B.rowleft) {
          if (a[kinematic_simulation_B.jy - 1] != 0.0) {
            exitg1 = 1;
          } else {
            kinematic_simulation_B.jy += 4;
          }
        } else {
          kinematic_simulation_B.lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    kinematic_simulation_B.knt = -1;
    kinematic_simulation_B.lastc = 0;
  }

  if (kinematic_simulation_B.knt + 1 > 0) {
    if (kinematic_simulation_B.lastc != 0) {
      kinematic_simulation_B.rowleft = 0;
      while (kinematic_simulation_B.rowleft <= kinematic_simulation_B.lastc - 1)
      {
        kinematic_simulation_B.work_g[kinematic_simulation_B.rowleft] = 0.0;
        kinematic_simulation_B.rowleft++;
      }

      kinematic_simulation_B.rowleft = 1;
      kinematic_simulation_B.jy = (kinematic_simulation_B.knt << 2) + 5;
      kinematic_simulation_B.iac = 5;
      while (kinematic_simulation_B.iac <= kinematic_simulation_B.jy) {
        kinematic_simulation_B.b_ix = 0;
        kinematic_simulation_B.g_f = (kinematic_simulation_B.iac +
          kinematic_simulation_B.lastc) - 1;
        kinematic_simulation_B.b_ia = kinematic_simulation_B.iac;
        while (kinematic_simulation_B.b_ia <= kinematic_simulation_B.g_f) {
          kinematic_simulation_B.work_g[kinematic_simulation_B.b_ix] +=
            a[kinematic_simulation_B.b_ia - 1] *
            a[kinematic_simulation_B.rowleft];
          kinematic_simulation_B.b_ix++;
          kinematic_simulation_B.b_ia++;
        }

        kinematic_simulation_B.rowleft++;
        kinematic_simulation_B.iac += 4;
      }
    }

    if (!(-tau[0] == 0.0)) {
      kinematic_simulation_B.rowleft = 4;
      kinematic_simulation_B.jy = 1;
      kinematic_simulation_B.iac = 0;
      while (kinematic_simulation_B.iac <= kinematic_simulation_B.knt) {
        if (a[kinematic_simulation_B.jy] != 0.0) {
          kinematic_simulation_B.xnorm = a[kinematic_simulation_B.jy] * -tau[0];
          kinematic_simulation_B.b_ix = 0;
          kinematic_simulation_B.g_f = kinematic_simulation_B.lastc +
            kinematic_simulation_B.rowleft;
          kinematic_simulation_B.b_ia = kinematic_simulation_B.rowleft;
          while (kinematic_simulation_B.b_ia + 1 <= kinematic_simulation_B.g_f)
          {
            a[kinematic_simulation_B.b_ia] +=
              kinematic_simulation_B.work_g[kinematic_simulation_B.b_ix] *
              kinematic_simulation_B.xnorm;
            kinematic_simulation_B.b_ix++;
            kinematic_simulation_B.b_ia++;
          }
        }

        kinematic_simulation_B.jy++;
        kinematic_simulation_B.rowleft += 4;
        kinematic_simulation_B.iac++;
      }
    }
  }

  kinematic_simulation_xzlarf(3, 3, 2, tau[0], a, 6,
    kinematic_simulation_B.work_g);
  a[1] = kinematic_simulation_B.alpha1;
  kinematic_simulation_B.alpha1 = a[6];
  tau[1] = 0.0;
  kinematic_simulation_B.xnorm = kinematic_simulation_xnrm2(1, a, 8);
  if (kinematic_simulation_B.xnorm != 0.0) {
    kinematic_simulation_B.xnorm = kinematic_simulat_rt_hypotd_snf(a[6],
      kinematic_simulation_B.xnorm);
    if (a[6] >= 0.0) {
      kinematic_simulation_B.xnorm = -kinematic_simulation_B.xnorm;
    }

    if (fabs(kinematic_simulation_B.xnorm) < 1.0020841800044864E-292) {
      kinematic_simulation_B.knt = -1;
      do {
        kinematic_simulation_B.knt++;
        a[7] *= 9.9792015476736E+291;
        kinematic_simulation_B.xnorm *= 9.9792015476736E+291;
        kinematic_simulation_B.alpha1 *= 9.9792015476736E+291;
      } while (!(fabs(kinematic_simulation_B.xnorm) >= 1.0020841800044864E-292));

      kinematic_simulation_B.xnorm = kinematic_simulat_rt_hypotd_snf
        (kinematic_simulation_B.alpha1, kinematic_simulation_xnrm2(1, a, 8));
      if (kinematic_simulation_B.alpha1 >= 0.0) {
        kinematic_simulation_B.xnorm = -kinematic_simulation_B.xnorm;
      }

      tau[1] = (kinematic_simulation_B.xnorm - kinematic_simulation_B.alpha1) /
        kinematic_simulation_B.xnorm;
      kinematic_simulation_B.alpha1 = 1.0 / (kinematic_simulation_B.alpha1 -
        kinematic_simulation_B.xnorm);
      a[7] *= kinematic_simulation_B.alpha1;
      kinematic_simulation_B.lastc = 0;
      while (kinematic_simulation_B.lastc <= kinematic_simulation_B.knt) {
        kinematic_simulation_B.xnorm *= 1.0020841800044864E-292;
        kinematic_simulation_B.lastc++;
      }

      kinematic_simulation_B.alpha1 = kinematic_simulation_B.xnorm;
    } else {
      tau[1] = (kinematic_simulation_B.xnorm - a[6]) /
        kinematic_simulation_B.xnorm;
      a[7] *= 1.0 / (a[6] - kinematic_simulation_B.xnorm);
      kinematic_simulation_B.alpha1 = kinematic_simulation_B.xnorm;
    }
  }

  a[6] = 1.0;
  if (tau[1] != 0.0) {
    kinematic_simulation_B.knt = 1;
    kinematic_simulation_B.lastc = 7;
    while ((kinematic_simulation_B.knt + 1 > 0) &&
           (a[kinematic_simulation_B.lastc] == 0.0)) {
      kinematic_simulation_B.knt--;
      kinematic_simulation_B.lastc--;
    }

    kinematic_simulation_B.lastc = 4;
    exitg2 = false;
    while ((!exitg2) && (kinematic_simulation_B.lastc > 0)) {
      kinematic_simulation_B.rowleft = kinematic_simulation_B.lastc + 8;
      kinematic_simulation_B.jy = kinematic_simulation_B.rowleft;
      do {
        exitg1 = 0;
        if (kinematic_simulation_B.jy <= (kinematic_simulation_B.knt << 2) +
            kinematic_simulation_B.rowleft) {
          if (a[kinematic_simulation_B.jy - 1] != 0.0) {
            exitg1 = 1;
          } else {
            kinematic_simulation_B.jy += 4;
          }
        } else {
          kinematic_simulation_B.lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    kinematic_simulation_B.knt = -1;
    kinematic_simulation_B.lastc = 0;
  }

  if (kinematic_simulation_B.knt + 1 > 0) {
    if (kinematic_simulation_B.lastc != 0) {
      kinematic_simulation_B.rowleft = 0;
      while (kinematic_simulation_B.rowleft <= kinematic_simulation_B.lastc - 1)
      {
        kinematic_simulation_B.work_g[kinematic_simulation_B.rowleft] = 0.0;
        kinematic_simulation_B.rowleft++;
      }

      kinematic_simulation_B.rowleft = 6;
      kinematic_simulation_B.jy = (kinematic_simulation_B.knt << 2) + 9;
      kinematic_simulation_B.iac = 9;
      while (kinematic_simulation_B.iac <= kinematic_simulation_B.jy) {
        kinematic_simulation_B.b_ix = 0;
        kinematic_simulation_B.g_f = (kinematic_simulation_B.iac +
          kinematic_simulation_B.lastc) - 1;
        kinematic_simulation_B.b_ia = kinematic_simulation_B.iac;
        while (kinematic_simulation_B.b_ia <= kinematic_simulation_B.g_f) {
          kinematic_simulation_B.work_g[kinematic_simulation_B.b_ix] +=
            a[kinematic_simulation_B.b_ia - 1] *
            a[kinematic_simulation_B.rowleft];
          kinematic_simulation_B.b_ix++;
          kinematic_simulation_B.b_ia++;
        }

        kinematic_simulation_B.rowleft++;
        kinematic_simulation_B.iac += 4;
      }
    }

    if (!(-tau[1] == 0.0)) {
      kinematic_simulation_B.rowleft = 8;
      kinematic_simulation_B.jy = 6;
      kinematic_simulation_B.iac = 0;
      while (kinematic_simulation_B.iac <= kinematic_simulation_B.knt) {
        if (a[kinematic_simulation_B.jy] != 0.0) {
          kinematic_simulation_B.xnorm = a[kinematic_simulation_B.jy] * -tau[1];
          kinematic_simulation_B.b_ix = 0;
          kinematic_simulation_B.g_f = kinematic_simulation_B.lastc +
            kinematic_simulation_B.rowleft;
          kinematic_simulation_B.b_ia = kinematic_simulation_B.rowleft;
          while (kinematic_simulation_B.b_ia + 1 <= kinematic_simulation_B.g_f)
          {
            a[kinematic_simulation_B.b_ia] +=
              kinematic_simulation_B.work_g[kinematic_simulation_B.b_ix] *
              kinematic_simulation_B.xnorm;
            kinematic_simulation_B.b_ix++;
            kinematic_simulation_B.b_ia++;
          }
        }

        kinematic_simulation_B.jy++;
        kinematic_simulation_B.rowleft += 4;
        kinematic_simulation_B.iac++;
      }
    }
  }

  kinematic_simulation_xzlarf(2, 2, 7, tau[1], a, 11,
    kinematic_simulation_B.work_g);
  a[6] = kinematic_simulation_B.alpha1;
  kinematic_simulation_B.alpha1 = a[11];
  tau[2] = 0.0;
  kinematic_simulation_B.xnorm = kinematic_simulation_xnrm2(0, a, 12);
  if (kinematic_simulation_B.xnorm != 0.0) {
    kinematic_simulation_B.xnorm = kinematic_simulat_rt_hypotd_snf(a[11],
      kinematic_simulation_B.xnorm);
    if (a[11] >= 0.0) {
      kinematic_simulation_B.xnorm = -kinematic_simulation_B.xnorm;
    }

    if (fabs(kinematic_simulation_B.xnorm) < 1.0020841800044864E-292) {
      kinematic_simulation_B.knt = -1;
      do {
        kinematic_simulation_B.knt++;
        kinematic_simulation_B.xnorm *= 9.9792015476736E+291;
        kinematic_simulation_B.alpha1 *= 9.9792015476736E+291;
      } while (!(fabs(kinematic_simulation_B.xnorm) >= 1.0020841800044864E-292));

      kinematic_simulation_B.xnorm = kinematic_simulat_rt_hypotd_snf
        (kinematic_simulation_B.alpha1, kinematic_simulation_xnrm2(0, a, 12));
      if (kinematic_simulation_B.alpha1 >= 0.0) {
        kinematic_simulation_B.xnorm = -kinematic_simulation_B.xnorm;
      }

      tau[2] = (kinematic_simulation_B.xnorm - kinematic_simulation_B.alpha1) /
        kinematic_simulation_B.xnorm;
      kinematic_simulation_B.lastc = 0;
      while (kinematic_simulation_B.lastc <= kinematic_simulation_B.knt) {
        kinematic_simulation_B.xnorm *= 1.0020841800044864E-292;
        kinematic_simulation_B.lastc++;
      }

      kinematic_simulation_B.alpha1 = kinematic_simulation_B.xnorm;
    } else {
      tau[2] = (kinematic_simulation_B.xnorm - a[11]) /
        kinematic_simulation_B.xnorm;
      kinematic_simulation_B.alpha1 = kinematic_simulation_B.xnorm;
    }
  }

  a[11] = 1.0;
  if (tau[2] != 0.0) {
    kinematic_simulation_B.knt = 0;
    kinematic_simulation_B.lastc = 11;
    while ((kinematic_simulation_B.knt + 1 > 0) &&
           (a[kinematic_simulation_B.lastc] == 0.0)) {
      kinematic_simulation_B.knt--;
      kinematic_simulation_B.lastc--;
    }

    kinematic_simulation_B.lastc = 4;
    exitg2 = false;
    while ((!exitg2) && (kinematic_simulation_B.lastc > 0)) {
      kinematic_simulation_B.rowleft = kinematic_simulation_B.lastc + 12;
      kinematic_simulation_B.jy = kinematic_simulation_B.rowleft;
      do {
        exitg1 = 0;
        if (kinematic_simulation_B.jy <= (kinematic_simulation_B.knt << 2) +
            kinematic_simulation_B.rowleft) {
          if (a[kinematic_simulation_B.jy - 1] != 0.0) {
            exitg1 = 1;
          } else {
            kinematic_simulation_B.jy += 4;
          }
        } else {
          kinematic_simulation_B.lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    kinematic_simulation_B.knt = -1;
    kinematic_simulation_B.lastc = 0;
  }

  if (kinematic_simulation_B.knt + 1 > 0) {
    if (kinematic_simulation_B.lastc != 0) {
      kinematic_simulation_B.rowleft = 0;
      while (kinematic_simulation_B.rowleft <= kinematic_simulation_B.lastc - 1)
      {
        kinematic_simulation_B.work_g[kinematic_simulation_B.rowleft] = 0.0;
        kinematic_simulation_B.rowleft++;
      }

      kinematic_simulation_B.rowleft = 11;
      kinematic_simulation_B.jy = (kinematic_simulation_B.knt << 2) + 13;
      kinematic_simulation_B.iac = 13;
      while (kinematic_simulation_B.iac <= kinematic_simulation_B.jy) {
        kinematic_simulation_B.b_ix = 0;
        kinematic_simulation_B.g_f = (kinematic_simulation_B.iac +
          kinematic_simulation_B.lastc) - 1;
        kinematic_simulation_B.b_ia = kinematic_simulation_B.iac;
        while (kinematic_simulation_B.b_ia <= kinematic_simulation_B.g_f) {
          kinematic_simulation_B.work_g[kinematic_simulation_B.b_ix] +=
            a[kinematic_simulation_B.b_ia - 1] *
            a[kinematic_simulation_B.rowleft];
          kinematic_simulation_B.b_ix++;
          kinematic_simulation_B.b_ia++;
        }

        kinematic_simulation_B.rowleft++;
        kinematic_simulation_B.iac += 4;
      }
    }

    if (!(-tau[2] == 0.0)) {
      kinematic_simulation_B.rowleft = 12;
      kinematic_simulation_B.jy = 11;
      kinematic_simulation_B.iac = 0;
      while (kinematic_simulation_B.iac <= kinematic_simulation_B.knt) {
        if (a[kinematic_simulation_B.jy] != 0.0) {
          kinematic_simulation_B.xnorm = a[kinematic_simulation_B.jy] * -tau[2];
          kinematic_simulation_B.b_ix = 0;
          kinematic_simulation_B.g_f = kinematic_simulation_B.lastc +
            kinematic_simulation_B.rowleft;
          kinematic_simulation_B.b_ia = kinematic_simulation_B.rowleft;
          while (kinematic_simulation_B.b_ia + 1 <= kinematic_simulation_B.g_f)
          {
            a[kinematic_simulation_B.b_ia] +=
              kinematic_simulation_B.work_g[kinematic_simulation_B.b_ix] *
              kinematic_simulation_B.xnorm;
            kinematic_simulation_B.b_ix++;
            kinematic_simulation_B.b_ia++;
          }
        }

        kinematic_simulation_B.jy++;
        kinematic_simulation_B.rowleft += 4;
        kinematic_simulation_B.iac++;
      }
    }
  }

  kinematic_simulation_xzlarf(1, 1, 12, tau[2], a, 16,
    kinematic_simulation_B.work_g);
  a[11] = kinematic_simulation_B.alpha1;
}

// Function for MATLAB Function: '<S4>/MATLAB Function'
static real_T kinematic_simulation_xnrm2_l(int32_T n, const real_T x[3])
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

// Function for MATLAB Function: '<S4>/MATLAB Function'
static real_T kinematic_simulation_xzlarfg(int32_T n, real_T *alpha1, real_T x[3])
{
  real_T tau;
  real_T xnorm;
  int32_T knt;
  int32_T c_k;
  tau = 0.0;
  if (n > 0) {
    xnorm = kinematic_simulation_xnrm2_l(n - 1, x);
    if (xnorm != 0.0) {
      xnorm = kinematic_simulat_rt_hypotd_snf(*alpha1, xnorm);
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

        xnorm = kinematic_simulat_rt_hypotd_snf(*alpha1,
          kinematic_simulation_xnrm2_l(n - 1, x));
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

// Function for MATLAB Function: '<S4>/MATLAB Function'
static void kinematic_simulation_xdlanv2(real_T *a, real_T *b, real_T *c, real_T
  *d, real_T *rt1r, real_T *rt1i, real_T *rt2r, real_T *rt2i, real_T *cs, real_T
  *sn)
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
    kinematic_simulation_B.bcmax = *d;
    *d = *a;
    *a = kinematic_simulation_B.bcmax;
    *b = -*c;
    *c = 0.0;
  } else {
    kinematic_simulation_B.tau_c = *a - *d;
    if ((kinematic_simulation_B.tau_c == 0.0) && ((*b < 0.0) != (*c < 0.0))) {
      *cs = 1.0;
      *sn = 0.0;
    } else {
      kinematic_simulation_B.p = 0.5 * kinematic_simulation_B.tau_c;
      kinematic_simulation_B.bcmis = fabs(*b);
      kinematic_simulation_B.z = fabs(*c);
      tmp = rtIsNaN(kinematic_simulation_B.z);
      if ((kinematic_simulation_B.bcmis > kinematic_simulation_B.z) || tmp) {
        kinematic_simulation_B.bcmax = kinematic_simulation_B.bcmis;
      } else {
        kinematic_simulation_B.bcmax = kinematic_simulation_B.z;
      }

      if ((kinematic_simulation_B.bcmis < kinematic_simulation_B.z) || tmp) {
        kinematic_simulation_B.z = kinematic_simulation_B.bcmis;
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

      kinematic_simulation_B.bcmis = kinematic_simulation_B.z *
        static_cast<real_T>(b_0) * static_cast<real_T>(c_0);
      kinematic_simulation_B.scale_h = fabs(kinematic_simulation_B.p);
      if ((!(kinematic_simulation_B.scale_h > kinematic_simulation_B.bcmax)) &&
          (!rtIsNaN(kinematic_simulation_B.bcmax))) {
        kinematic_simulation_B.scale_h = kinematic_simulation_B.bcmax;
      }

      kinematic_simulation_B.z = kinematic_simulation_B.p /
        kinematic_simulation_B.scale_h * kinematic_simulation_B.p +
        kinematic_simulation_B.bcmax / kinematic_simulation_B.scale_h *
        kinematic_simulation_B.bcmis;
      if (kinematic_simulation_B.z >= 8.8817841970012523E-16) {
        if (!(kinematic_simulation_B.p < 0.0)) {
          kinematic_simulation_B.tau_c = sqrt(kinematic_simulation_B.scale_h) *
            sqrt(kinematic_simulation_B.z);
        } else {
          kinematic_simulation_B.tau_c = -(sqrt(kinematic_simulation_B.scale_h) *
            sqrt(kinematic_simulation_B.z));
        }

        kinematic_simulation_B.z = kinematic_simulation_B.p +
          kinematic_simulation_B.tau_c;
        *a = *d + kinematic_simulation_B.z;
        *d -= kinematic_simulation_B.bcmax / kinematic_simulation_B.z *
          kinematic_simulation_B.bcmis;
        kinematic_simulation_B.tau_c = kinematic_simulat_rt_hypotd_snf(*c,
          kinematic_simulation_B.z);
        *cs = kinematic_simulation_B.z / kinematic_simulation_B.tau_c;
        *sn = *c / kinematic_simulation_B.tau_c;
        *b -= *c;
        *c = 0.0;
      } else {
        kinematic_simulation_B.bcmax = *b + *c;
        kinematic_simulation_B.tau_c = kinematic_simulat_rt_hypotd_snf
          (kinematic_simulation_B.bcmax, kinematic_simulation_B.tau_c);
        *cs = sqrt((fabs(kinematic_simulation_B.bcmax) /
                    kinematic_simulation_B.tau_c + 1.0) * 0.5);
        if (!(kinematic_simulation_B.bcmax < 0.0)) {
          b_0 = 1;
        } else {
          b_0 = -1;
        }

        *sn = -(kinematic_simulation_B.p / (kinematic_simulation_B.tau_c * *cs))
          * static_cast<real_T>(b_0);
        kinematic_simulation_B.p = *a * *cs + *b * *sn;
        kinematic_simulation_B.tau_c = -*a * *sn + *b * *cs;
        kinematic_simulation_B.bcmax = *c * *cs + *d * *sn;
        kinematic_simulation_B.bcmis = -*c * *sn + *d * *cs;
        *b = kinematic_simulation_B.tau_c * *cs + kinematic_simulation_B.bcmis *
          *sn;
        *c = -kinematic_simulation_B.p * *sn + kinematic_simulation_B.bcmax *
          *cs;
        kinematic_simulation_B.bcmax = ((kinematic_simulation_B.p * *cs +
          kinematic_simulation_B.bcmax * *sn) + (-kinematic_simulation_B.tau_c *
          *sn + kinematic_simulation_B.bcmis * *cs)) * 0.5;
        *a = kinematic_simulation_B.bcmax;
        *d = kinematic_simulation_B.bcmax;
        if (*c != 0.0) {
          if (*b != 0.0) {
            if ((*b < 0.0) == (*c < 0.0)) {
              kinematic_simulation_B.z = sqrt(fabs(*b));
              kinematic_simulation_B.bcmis = sqrt(fabs(*c));
              if (!(*c < 0.0)) {
                kinematic_simulation_B.p = kinematic_simulation_B.z *
                  kinematic_simulation_B.bcmis;
              } else {
                kinematic_simulation_B.p = -(kinematic_simulation_B.z *
                  kinematic_simulation_B.bcmis);
              }

              kinematic_simulation_B.tau_c = 1.0 / sqrt(fabs(*b + *c));
              *a = kinematic_simulation_B.bcmax + kinematic_simulation_B.p;
              *d = kinematic_simulation_B.bcmax - kinematic_simulation_B.p;
              *b -= *c;
              *c = 0.0;
              kinematic_simulation_B.p = kinematic_simulation_B.z *
                kinematic_simulation_B.tau_c;
              kinematic_simulation_B.tau_c *= kinematic_simulation_B.bcmis;
              kinematic_simulation_B.bcmax = *cs * kinematic_simulation_B.p -
                *sn * kinematic_simulation_B.tau_c;
              *sn = *cs * kinematic_simulation_B.tau_c + *sn *
                kinematic_simulation_B.p;
              *cs = kinematic_simulation_B.bcmax;
            }
          } else {
            *b = -*c;
            *c = 0.0;
            kinematic_simulation_B.bcmax = *cs;
            *cs = -*sn;
            *sn = kinematic_simulation_B.bcmax;
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

// Function for MATLAB Function: '<S4>/MATLAB Function'
static void kinematic_simulation_xrot(int32_T n, real_T x[16], int32_T ix0,
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

// Function for MATLAB Function: '<S4>/MATLAB Function'
static void kinematic_simulation_xrot_g(int32_T n, real_T x[16], int32_T ix0,
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

// Function for MATLAB Function: '<S4>/MATLAB Function'
static int32_T kinematic_simulation_eml_dlahqr(real_T h[16], real_T z[16])
{
  int32_T info;
  boolean_T goto150;
  int32_T s_tmp;
  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T exitg3;
  info = 0;
  kinematic_simulation_B.v_l[0] = 0.0;
  kinematic_simulation_B.v_l[1] = 0.0;
  kinematic_simulation_B.v_l[2] = 0.0;
  h[2] = 0.0;
  h[3] = 0.0;
  h[7] = 0.0;
  kinematic_simulation_B.i_e = 3;
  exitg1 = false;
  while ((!exitg1) && (kinematic_simulation_B.i_e + 1 >= 1)) {
    kinematic_simulation_B.L = 1;
    goto150 = false;
    kinematic_simulation_B.ix = 0;
    exitg2 = false;
    while ((!exitg2) && (kinematic_simulation_B.ix < 301)) {
      kinematic_simulation_B.k_o = kinematic_simulation_B.i_e;
      exitg3 = false;
      while ((!exitg3) && (kinematic_simulation_B.k_o + 1 >
                           kinematic_simulation_B.L)) {
        s_tmp = ((kinematic_simulation_B.k_o - 1) << 2) +
          kinematic_simulation_B.k_o;
        kinematic_simulation_B.htmp2 = fabs(h[s_tmp]);
        if (kinematic_simulation_B.htmp2 <= 4.0083367200179456E-292) {
          exitg3 = true;
        } else {
          kinematic_simulation_B.m = (kinematic_simulation_B.k_o << 2) +
            kinematic_simulation_B.k_o;
          kinematic_simulation_B.nr = s_tmp - 1;
          kinematic_simulation_B.tst = fabs(h[kinematic_simulation_B.nr]) + fabs
            (h[kinematic_simulation_B.m]);
          if (kinematic_simulation_B.tst == 0.0) {
            if (kinematic_simulation_B.k_o - 1 >= 1) {
              kinematic_simulation_B.tst = fabs(h[(((kinematic_simulation_B.k_o
                - 2) << 2) + kinematic_simulation_B.k_o) - 1]);
            }

            if (kinematic_simulation_B.k_o + 2 <= 4) {
              kinematic_simulation_B.tst += fabs(h[((kinematic_simulation_B.k_o <<
                2) + kinematic_simulation_B.k_o) + 1]);
            }
          }

          if (kinematic_simulation_B.htmp2 <= 2.2204460492503131E-16 *
              kinematic_simulation_B.tst) {
            kinematic_simulation_B.htmp1 = fabs(h[s_tmp]);
            kinematic_simulation_B.htmp2 = fabs(h[kinematic_simulation_B.m - 1]);
            if (kinematic_simulation_B.htmp1 > kinematic_simulation_B.htmp2) {
              kinematic_simulation_B.tst = kinematic_simulation_B.htmp1;
              kinematic_simulation_B.ba = kinematic_simulation_B.htmp2;
            } else {
              kinematic_simulation_B.tst = kinematic_simulation_B.htmp2;
              kinematic_simulation_B.ba = kinematic_simulation_B.htmp1;
            }

            kinematic_simulation_B.htmp1 = fabs(h[kinematic_simulation_B.m]);
            kinematic_simulation_B.htmp2 = fabs(h[kinematic_simulation_B.nr] -
              h[kinematic_simulation_B.m]);
            if (kinematic_simulation_B.htmp1 > kinematic_simulation_B.htmp2) {
              kinematic_simulation_B.aa = kinematic_simulation_B.htmp1;
              kinematic_simulation_B.htmp1 = kinematic_simulation_B.htmp2;
            } else {
              kinematic_simulation_B.aa = kinematic_simulation_B.htmp2;
            }

            kinematic_simulation_B.htmp2 = kinematic_simulation_B.aa +
              kinematic_simulation_B.tst;
            kinematic_simulation_B.htmp1 = kinematic_simulation_B.aa /
              kinematic_simulation_B.htmp2 * kinematic_simulation_B.htmp1 *
              2.2204460492503131E-16;
            if ((4.0083367200179456E-292 > kinematic_simulation_B.htmp1) ||
                rtIsNaN(kinematic_simulation_B.htmp1)) {
              kinematic_simulation_B.htmp1 = 4.0083367200179456E-292;
            }

            if (kinematic_simulation_B.tst / kinematic_simulation_B.htmp2 *
                kinematic_simulation_B.ba <= kinematic_simulation_B.htmp1) {
              exitg3 = true;
            } else {
              kinematic_simulation_B.k_o--;
            }
          } else {
            kinematic_simulation_B.k_o--;
          }
        }
      }

      kinematic_simulation_B.L = kinematic_simulation_B.k_o + 1;
      if (kinematic_simulation_B.k_o + 1 > 1) {
        h[kinematic_simulation_B.k_o + ((kinematic_simulation_B.k_o - 1) << 2)] =
          0.0;
      }

      if (kinematic_simulation_B.k_o + 1 >= kinematic_simulation_B.i_e) {
        goto150 = true;
        exitg2 = true;
      } else {
        switch (kinematic_simulation_B.ix) {
         case 10:
          s_tmp = (kinematic_simulation_B.k_o << 2) + kinematic_simulation_B.k_o;
          kinematic_simulation_B.htmp2 = fabs(h[(((kinematic_simulation_B.k_o +
            1) << 2) + kinematic_simulation_B.k_o) + 2]) + fabs(h[s_tmp + 1]);
          kinematic_simulation_B.ba = h[s_tmp] + 0.75 *
            kinematic_simulation_B.htmp2;
          kinematic_simulation_B.h12 = -0.4375 * kinematic_simulation_B.htmp2;
          kinematic_simulation_B.aa = kinematic_simulation_B.htmp2;
          kinematic_simulation_B.tst = kinematic_simulation_B.ba;
          break;

         case 20:
          kinematic_simulation_B.htmp2 = fabs(h[(((kinematic_simulation_B.i_e -
            2) << 2) + kinematic_simulation_B.i_e) - 1]) + fabs(h
            [((kinematic_simulation_B.i_e - 1) << 2) +
            kinematic_simulation_B.i_e]);
          kinematic_simulation_B.ba = h[(kinematic_simulation_B.i_e << 2) +
            kinematic_simulation_B.i_e] + 0.75 * kinematic_simulation_B.htmp2;
          kinematic_simulation_B.h12 = -0.4375 * kinematic_simulation_B.htmp2;
          kinematic_simulation_B.aa = kinematic_simulation_B.htmp2;
          kinematic_simulation_B.tst = kinematic_simulation_B.ba;
          break;

         default:
          kinematic_simulation_B.ba = h[(((kinematic_simulation_B.i_e - 1) << 2)
            + kinematic_simulation_B.i_e) - 1];
          kinematic_simulation_B.aa = h[((kinematic_simulation_B.i_e - 1) << 2)
            + kinematic_simulation_B.i_e];
          kinematic_simulation_B.h12 = h[((kinematic_simulation_B.i_e << 2) +
            kinematic_simulation_B.i_e) - 1];
          kinematic_simulation_B.tst = h[(kinematic_simulation_B.i_e << 2) +
            kinematic_simulation_B.i_e];
          break;
        }

        kinematic_simulation_B.htmp2 = ((fabs(kinematic_simulation_B.ba) + fabs
          (kinematic_simulation_B.h12)) + fabs(kinematic_simulation_B.aa)) +
          fabs(kinematic_simulation_B.tst);
        if (kinematic_simulation_B.htmp2 == 0.0) {
          kinematic_simulation_B.ba = 0.0;
          kinematic_simulation_B.tst = 0.0;
          kinematic_simulation_B.htmp1 = 0.0;
          kinematic_simulation_B.aa = 0.0;
        } else {
          kinematic_simulation_B.ba /= kinematic_simulation_B.htmp2;
          kinematic_simulation_B.aa /= kinematic_simulation_B.htmp2;
          kinematic_simulation_B.h12 /= kinematic_simulation_B.htmp2;
          kinematic_simulation_B.tst /= kinematic_simulation_B.htmp2;
          kinematic_simulation_B.htmp1 = (kinematic_simulation_B.ba +
            kinematic_simulation_B.tst) / 2.0;
          kinematic_simulation_B.ba = (kinematic_simulation_B.ba -
            kinematic_simulation_B.htmp1) * (kinematic_simulation_B.tst -
            kinematic_simulation_B.htmp1) - kinematic_simulation_B.h12 *
            kinematic_simulation_B.aa;
          kinematic_simulation_B.aa = sqrt(fabs(kinematic_simulation_B.ba));
          if (kinematic_simulation_B.ba >= 0.0) {
            kinematic_simulation_B.ba = kinematic_simulation_B.htmp1 *
              kinematic_simulation_B.htmp2;
            kinematic_simulation_B.htmp1 = kinematic_simulation_B.ba;
            kinematic_simulation_B.tst = kinematic_simulation_B.aa *
              kinematic_simulation_B.htmp2;
            kinematic_simulation_B.aa = -kinematic_simulation_B.tst;
          } else {
            kinematic_simulation_B.ba = kinematic_simulation_B.htmp1 +
              kinematic_simulation_B.aa;
            kinematic_simulation_B.htmp1 -= kinematic_simulation_B.aa;
            if (fabs(kinematic_simulation_B.ba - kinematic_simulation_B.tst) <=
                fabs(kinematic_simulation_B.htmp1 - kinematic_simulation_B.tst))
            {
              kinematic_simulation_B.ba *= kinematic_simulation_B.htmp2;
              kinematic_simulation_B.htmp1 = kinematic_simulation_B.ba;
            } else {
              kinematic_simulation_B.htmp1 *= kinematic_simulation_B.htmp2;
              kinematic_simulation_B.ba = kinematic_simulation_B.htmp1;
            }

            kinematic_simulation_B.tst = 0.0;
            kinematic_simulation_B.aa = 0.0;
          }
        }

        kinematic_simulation_B.m = kinematic_simulation_B.i_e - 1;
        exitg3 = false;
        while ((!exitg3) && (kinematic_simulation_B.m >=
                             kinematic_simulation_B.k_o + 1)) {
          s_tmp = ((kinematic_simulation_B.m - 1) << 2) +
            kinematic_simulation_B.m;
          kinematic_simulation_B.nr = s_tmp - 1;
          kinematic_simulation_B.h12 = h[kinematic_simulation_B.nr] -
            kinematic_simulation_B.htmp1;
          kinematic_simulation_B.htmp2 = (fabs(kinematic_simulation_B.h12) +
            fabs(kinematic_simulation_B.aa)) + fabs(h[s_tmp]);
          kinematic_simulation_B.h21s = h[s_tmp] / kinematic_simulation_B.htmp2;
          kinematic_simulation_B.hoffset = (kinematic_simulation_B.m << 2) +
            kinematic_simulation_B.m;
          kinematic_simulation_B.v_l[0] = (kinematic_simulation_B.h12 /
            kinematic_simulation_B.htmp2 * (h[kinematic_simulation_B.nr] -
            kinematic_simulation_B.ba) + h[kinematic_simulation_B.hoffset - 1] *
            kinematic_simulation_B.h21s) - kinematic_simulation_B.aa /
            kinematic_simulation_B.htmp2 * kinematic_simulation_B.tst;
          kinematic_simulation_B.v_l[1] = (((h[kinematic_simulation_B.nr] +
            h[kinematic_simulation_B.hoffset]) - kinematic_simulation_B.ba) -
            kinematic_simulation_B.htmp1) * kinematic_simulation_B.h21s;
          kinematic_simulation_B.v_l[2] = h[kinematic_simulation_B.hoffset + 1] *
            kinematic_simulation_B.h21s;
          kinematic_simulation_B.htmp2 = (fabs(kinematic_simulation_B.v_l[0]) +
            fabs(kinematic_simulation_B.v_l[1])) + fabs
            (kinematic_simulation_B.v_l[2]);
          kinematic_simulation_B.v_l[0] /= kinematic_simulation_B.htmp2;
          kinematic_simulation_B.v_l[1] /= kinematic_simulation_B.htmp2;
          kinematic_simulation_B.v_l[2] /= kinematic_simulation_B.htmp2;
          if (kinematic_simulation_B.k_o + 1 == kinematic_simulation_B.m) {
            exitg3 = true;
          } else {
            s_tmp = ((kinematic_simulation_B.m - 2) << 2) +
              kinematic_simulation_B.m;
            if (fabs(h[s_tmp - 1]) * (fabs(kinematic_simulation_B.v_l[1]) + fabs
                 (kinematic_simulation_B.v_l[2])) <= ((fabs(h[s_tmp - 2]) + fabs
                  (h[kinematic_simulation_B.nr])) + fabs
                 (h[kinematic_simulation_B.hoffset])) * (2.2204460492503131E-16 *
                 fabs(kinematic_simulation_B.v_l[0]))) {
              exitg3 = true;
            } else {
              kinematic_simulation_B.m--;
            }
          }
        }

        for (s_tmp = kinematic_simulation_B.m; s_tmp <=
             kinematic_simulation_B.i_e; s_tmp++) {
          kinematic_simulation_B.nr = (kinematic_simulation_B.i_e - s_tmp) + 2;
          if (3 < kinematic_simulation_B.nr) {
            kinematic_simulation_B.nr = 3;
          }

          if (s_tmp > kinematic_simulation_B.m) {
            kinematic_simulation_B.hoffset = ((s_tmp - 2) << 2) + s_tmp;
            kinematic_simulation_B.j_b = 0;
            while (kinematic_simulation_B.j_b <= kinematic_simulation_B.nr - 1)
            {
              kinematic_simulation_B.v_l[kinematic_simulation_B.j_b] = h
                [(kinematic_simulation_B.j_b + kinematic_simulation_B.hoffset) -
                1];
              kinematic_simulation_B.j_b++;
            }
          }

          kinematic_simulation_B.tst = kinematic_simulation_B.v_l[0];
          kinematic_simulation_B.b_v[0] = kinematic_simulation_B.v_l[0];
          kinematic_simulation_B.b_v[1] = kinematic_simulation_B.v_l[1];
          kinematic_simulation_B.b_v[2] = kinematic_simulation_B.v_l[2];
          kinematic_simulation_B.htmp2 = kinematic_simulation_xzlarfg
            (kinematic_simulation_B.nr, &kinematic_simulation_B.tst,
             kinematic_simulation_B.b_v);
          kinematic_simulation_B.v_l[1] = kinematic_simulation_B.b_v[1];
          kinematic_simulation_B.v_l[2] = kinematic_simulation_B.b_v[2];
          kinematic_simulation_B.v_l[0] = kinematic_simulation_B.tst;
          if (s_tmp > kinematic_simulation_B.m) {
            h[(s_tmp + ((s_tmp - 2) << 2)) - 1] = kinematic_simulation_B.tst;
            h[s_tmp + ((s_tmp - 2) << 2)] = 0.0;
            if (s_tmp < kinematic_simulation_B.i_e) {
              h[s_tmp + 1] = 0.0;
            }
          } else {
            if (kinematic_simulation_B.m > kinematic_simulation_B.k_o + 1) {
              h[s_tmp - 1] *= 1.0 - kinematic_simulation_B.htmp2;
            }
          }

          kinematic_simulation_B.tst = kinematic_simulation_B.b_v[1];
          kinematic_simulation_B.ba = kinematic_simulation_B.htmp2 *
            kinematic_simulation_B.b_v[1];
          switch (kinematic_simulation_B.nr) {
           case 3:
            kinematic_simulation_B.htmp1 = kinematic_simulation_B.b_v[2];
            kinematic_simulation_B.aa = kinematic_simulation_B.htmp2 *
              kinematic_simulation_B.b_v[2];
            kinematic_simulation_B.b_j_a = s_tmp - 1;
            while (kinematic_simulation_B.b_j_a + 1 < 5) {
              kinematic_simulation_B.nr = (kinematic_simulation_B.b_j_a << 2) +
                s_tmp;
              kinematic_simulation_B.hoffset = kinematic_simulation_B.nr - 1;
              kinematic_simulation_B.j_b = kinematic_simulation_B.nr + 1;
              kinematic_simulation_B.h12 = (h[kinematic_simulation_B.hoffset] +
                h[kinematic_simulation_B.nr] * kinematic_simulation_B.tst) +
                h[kinematic_simulation_B.j_b] * kinematic_simulation_B.htmp1;
              h[kinematic_simulation_B.hoffset] -= kinematic_simulation_B.h12 *
                kinematic_simulation_B.htmp2;
              h[kinematic_simulation_B.nr] -= kinematic_simulation_B.h12 *
                kinematic_simulation_B.ba;
              h[kinematic_simulation_B.j_b] -= kinematic_simulation_B.h12 *
                kinematic_simulation_B.aa;
              kinematic_simulation_B.b_j_a++;
            }

            kinematic_simulation_B.nr = s_tmp + 3;
            kinematic_simulation_B.b_j_a = kinematic_simulation_B.i_e + 1;
            if (kinematic_simulation_B.nr < kinematic_simulation_B.b_j_a) {
              kinematic_simulation_B.b_j_a = kinematic_simulation_B.nr;
            }

            kinematic_simulation_B.c_j = 0;
            while (kinematic_simulation_B.c_j <= kinematic_simulation_B.b_j_a -
                   1) {
              kinematic_simulation_B.nr = ((s_tmp - 1) << 2) +
                kinematic_simulation_B.c_j;
              kinematic_simulation_B.hoffset = (s_tmp << 2) +
                kinematic_simulation_B.c_j;
              kinematic_simulation_B.j_b = ((s_tmp + 1) << 2) +
                kinematic_simulation_B.c_j;
              kinematic_simulation_B.h12 = (h[kinematic_simulation_B.nr] +
                h[kinematic_simulation_B.hoffset] * kinematic_simulation_B.tst)
                + h[kinematic_simulation_B.j_b] * kinematic_simulation_B.htmp1;
              h[kinematic_simulation_B.nr] -= kinematic_simulation_B.h12 *
                kinematic_simulation_B.htmp2;
              h[kinematic_simulation_B.hoffset] -= kinematic_simulation_B.h12 *
                kinematic_simulation_B.ba;
              h[kinematic_simulation_B.j_b] -= kinematic_simulation_B.h12 *
                kinematic_simulation_B.aa;
              kinematic_simulation_B.c_j++;
            }

            for (kinematic_simulation_B.b_j_a = 0; kinematic_simulation_B.b_j_a <
                 4; kinematic_simulation_B.b_j_a++) {
              kinematic_simulation_B.nr = ((s_tmp - 1) << 2) +
                kinematic_simulation_B.b_j_a;
              kinematic_simulation_B.hoffset = (s_tmp << 2) +
                kinematic_simulation_B.b_j_a;
              kinematic_simulation_B.j_b = ((s_tmp + 1) << 2) +
                kinematic_simulation_B.b_j_a;
              kinematic_simulation_B.h12 = (z[kinematic_simulation_B.nr] +
                z[kinematic_simulation_B.hoffset] * kinematic_simulation_B.tst)
                + z[kinematic_simulation_B.j_b] * kinematic_simulation_B.htmp1;
              z[kinematic_simulation_B.nr] -= kinematic_simulation_B.h12 *
                kinematic_simulation_B.htmp2;
              z[kinematic_simulation_B.hoffset] -= kinematic_simulation_B.h12 *
                kinematic_simulation_B.ba;
              z[kinematic_simulation_B.j_b] -= kinematic_simulation_B.h12 *
                kinematic_simulation_B.aa;
            }
            break;

           case 2:
            kinematic_simulation_B.j_b = s_tmp - 1;
            while (kinematic_simulation_B.j_b + 1 < 5) {
              kinematic_simulation_B.nr = (kinematic_simulation_B.j_b << 2) +
                s_tmp;
              kinematic_simulation_B.hoffset = kinematic_simulation_B.nr - 1;
              kinematic_simulation_B.h12 = h[kinematic_simulation_B.hoffset] +
                h[kinematic_simulation_B.nr] * kinematic_simulation_B.tst;
              h[kinematic_simulation_B.hoffset] -= kinematic_simulation_B.h12 *
                kinematic_simulation_B.htmp2;
              h[kinematic_simulation_B.nr] -= kinematic_simulation_B.h12 *
                kinematic_simulation_B.ba;
              kinematic_simulation_B.j_b++;
            }

            kinematic_simulation_B.j_b = 0;
            while (kinematic_simulation_B.j_b <= kinematic_simulation_B.i_e) {
              kinematic_simulation_B.nr = ((s_tmp - 1) << 2) +
                kinematic_simulation_B.j_b;
              kinematic_simulation_B.hoffset = (s_tmp << 2) +
                kinematic_simulation_B.j_b;
              kinematic_simulation_B.h12 = h[kinematic_simulation_B.nr] +
                h[kinematic_simulation_B.hoffset] * kinematic_simulation_B.tst;
              h[kinematic_simulation_B.nr] -= kinematic_simulation_B.h12 *
                kinematic_simulation_B.htmp2;
              h[kinematic_simulation_B.hoffset] -= kinematic_simulation_B.h12 *
                kinematic_simulation_B.ba;
              kinematic_simulation_B.j_b++;
            }

            for (kinematic_simulation_B.j_b = 0; kinematic_simulation_B.j_b < 4;
                 kinematic_simulation_B.j_b++) {
              kinematic_simulation_B.nr = ((s_tmp - 1) << 2) +
                kinematic_simulation_B.j_b;
              kinematic_simulation_B.hoffset = (s_tmp << 2) +
                kinematic_simulation_B.j_b;
              kinematic_simulation_B.h12 = z[kinematic_simulation_B.nr] +
                z[kinematic_simulation_B.hoffset] * kinematic_simulation_B.tst;
              z[kinematic_simulation_B.nr] -= kinematic_simulation_B.h12 *
                kinematic_simulation_B.htmp2;
              z[kinematic_simulation_B.hoffset] -= kinematic_simulation_B.h12 *
                kinematic_simulation_B.ba;
            }
            break;
          }
        }

        kinematic_simulation_B.ix++;
      }
    }

    if (!goto150) {
      info = kinematic_simulation_B.i_e + 1;
      exitg1 = true;
    } else {
      if ((kinematic_simulation_B.i_e + 1 != kinematic_simulation_B.L) &&
          (kinematic_simulation_B.L == kinematic_simulation_B.i_e)) {
        kinematic_simulation_B.ix = (kinematic_simulation_B.i_e - 1) << 2;
        kinematic_simulation_B.k_o = kinematic_simulation_B.ix +
          kinematic_simulation_B.i_e;
        kinematic_simulation_B.m = kinematic_simulation_B.k_o - 1;
        kinematic_simulation_B.ba = h[kinematic_simulation_B.m];
        s_tmp = kinematic_simulation_B.i_e << 2;
        kinematic_simulation_B.nr = s_tmp + kinematic_simulation_B.i_e;
        kinematic_simulation_B.hoffset = kinematic_simulation_B.nr - 1;
        kinematic_simulation_B.htmp1 = h[kinematic_simulation_B.hoffset];
        kinematic_simulation_B.aa = h[kinematic_simulation_B.k_o];
        kinematic_simulation_B.h12 = h[kinematic_simulation_B.nr];
        kinematic_simulation_xdlanv2(&kinematic_simulation_B.ba,
          &kinematic_simulation_B.htmp1, &kinematic_simulation_B.aa,
          &kinematic_simulation_B.h12, &kinematic_simulation_B.h21s,
          &kinematic_simulation_B.unusedU1, &kinematic_simulation_B.unusedU2,
          &kinematic_simulation_B.unusedU3, &kinematic_simulation_B.htmp2,
          &kinematic_simulation_B.tst);
        h[kinematic_simulation_B.m] = kinematic_simulation_B.ba;
        h[kinematic_simulation_B.hoffset] = kinematic_simulation_B.htmp1;
        h[kinematic_simulation_B.k_o] = kinematic_simulation_B.aa;
        h[kinematic_simulation_B.nr] = kinematic_simulation_B.h12;
        if (4 > kinematic_simulation_B.i_e + 1) {
          kinematic_simulation_xrot(3 - kinematic_simulation_B.i_e, h,
            kinematic_simulation_B.i_e + ((kinematic_simulation_B.i_e + 1) << 2),
            (kinematic_simulation_B.i_e + ((kinematic_simulation_B.i_e + 1) << 2))
            + 1, kinematic_simulation_B.htmp2, kinematic_simulation_B.tst);
        }

        kinematic_simulation_xrot_g(kinematic_simulation_B.i_e - 1, h,
          ((kinematic_simulation_B.i_e - 1) << 2) + 1,
          (kinematic_simulation_B.i_e << 2) + 1, kinematic_simulation_B.htmp2,
          kinematic_simulation_B.tst);
        kinematic_simulation_B.ba = kinematic_simulation_B.htmp2 *
          z[kinematic_simulation_B.ix] + kinematic_simulation_B.tst * z[s_tmp];
        z[s_tmp] = kinematic_simulation_B.htmp2 * z[s_tmp] -
          kinematic_simulation_B.tst * z[kinematic_simulation_B.ix];
        z[kinematic_simulation_B.ix] = kinematic_simulation_B.ba;
        kinematic_simulation_B.i_e = s_tmp + 1;
        kinematic_simulation_B.ix++;
        kinematic_simulation_B.ba = kinematic_simulation_B.htmp2 *
          z[kinematic_simulation_B.ix] + kinematic_simulation_B.tst *
          z[kinematic_simulation_B.i_e];
        z[kinematic_simulation_B.i_e] = kinematic_simulation_B.htmp2 *
          z[kinematic_simulation_B.i_e] - kinematic_simulation_B.tst *
          z[kinematic_simulation_B.ix];
        z[kinematic_simulation_B.ix] = kinematic_simulation_B.ba;
        kinematic_simulation_B.i_e++;
        kinematic_simulation_B.ix++;
        kinematic_simulation_B.ba = kinematic_simulation_B.htmp2 *
          z[kinematic_simulation_B.ix] + kinematic_simulation_B.tst *
          z[kinematic_simulation_B.i_e];
        z[kinematic_simulation_B.i_e] = kinematic_simulation_B.htmp2 *
          z[kinematic_simulation_B.i_e] - kinematic_simulation_B.tst *
          z[kinematic_simulation_B.ix];
        z[kinematic_simulation_B.ix] = kinematic_simulation_B.ba;
        kinematic_simulation_B.i_e++;
        kinematic_simulation_B.ix++;
        kinematic_simulation_B.ba = kinematic_simulation_B.htmp2 *
          z[kinematic_simulation_B.ix] + kinematic_simulation_B.tst *
          z[kinematic_simulation_B.i_e];
        z[kinematic_simulation_B.i_e] = kinematic_simulation_B.htmp2 *
          z[kinematic_simulation_B.i_e] - kinematic_simulation_B.tst *
          z[kinematic_simulation_B.ix];
        z[kinematic_simulation_B.ix] = kinematic_simulation_B.ba;
      }

      kinematic_simulation_B.i_e = kinematic_simulation_B.L - 2;
    }
  }

  return info;
}

// Function for MATLAB Function: '<S4>/MATLAB Function'
static void kinematic_simulation_eig(const real_T A[16], creal_T V[16], creal_T
  D[4])
{
  boolean_T p;
  int32_T exitg1;
  boolean_T exitg2;
  if (kinematic_simulati_anyNonFinite(A)) {
    for (kinematic_simulation_B.b_j = 0; kinematic_simulation_B.b_j < 16;
         kinematic_simulation_B.b_j++) {
      V[kinematic_simulation_B.b_j].re = (rtNaN);
      V[kinematic_simulation_B.b_j].im = 0.0;
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
    kinematic_simulation_B.b_j = 0;
    exitg2 = false;
    while ((!exitg2) && (kinematic_simulation_B.b_j < 4)) {
      kinematic_simulation_B.i = 0;
      do {
        exitg1 = 0;
        if (kinematic_simulation_B.i <= kinematic_simulation_B.b_j) {
          if (!(A[(kinematic_simulation_B.b_j << 2) + kinematic_simulation_B.i] ==
                A[(kinematic_simulation_B.i << 2) + kinematic_simulation_B.b_j]))
          {
            p = false;
            exitg1 = 1;
          } else {
            kinematic_simulation_B.i++;
          }
        } else {
          kinematic_simulation_B.b_j++;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }

    if (p) {
      if (kinematic_simulati_anyNonFinite(A)) {
        for (kinematic_simulation_B.b_j = 0; kinematic_simulation_B.b_j < 16;
             kinematic_simulation_B.b_j++) {
          kinematic_simulation_B.b_V[kinematic_simulation_B.b_j] = (rtNaN);
        }

        kinematic_simulation_B.b_j = 2;
        while (kinematic_simulation_B.b_j < 5) {
          kinematic_simulation_B.b_V[kinematic_simulation_B.b_j - 1] = 0.0;
          kinematic_simulation_B.b_j++;
        }

        kinematic_simulation_B.b_j = 3;
        while (kinematic_simulation_B.b_j < 5) {
          kinematic_simulation_B.b_V[kinematic_simulation_B.b_j + 3] = 0.0;
          kinematic_simulation_B.b_j++;
        }

        kinematic_simulation_B.b_V[11] = 0.0;
        for (kinematic_simulation_B.b_j = 0; kinematic_simulation_B.b_j < 16;
             kinematic_simulation_B.b_j++) {
          kinematic_simulation_B.b_A[kinematic_simulation_B.b_j] = (rtNaN);
        }
      } else {
        memcpy(&kinematic_simulation_B.b_A[0], &A[0], sizeof(real_T) << 4U);
        kinematic_simulation_xgehrd(kinematic_simulation_B.b_A,
          kinematic_simulation_B.tau);
        memcpy(&kinematic_simulation_B.b_V[0], &kinematic_simulation_B.b_A[0],
               sizeof(real_T) << 4U);
        kinematic_simulation_B.b_j = 0;
        while (kinematic_simulation_B.b_j <= 2) {
          kinematic_simulation_B.b_V[kinematic_simulation_B.b_j + 12] = 0.0;
          kinematic_simulation_B.b_j++;
        }

        kinematic_simulation_B.b_j = 0;
        while (kinematic_simulation_B.b_j <= 1) {
          kinematic_simulation_B.b_V[kinematic_simulation_B.b_j + 8] = 0.0;
          kinematic_simulation_B.b_j++;
        }

        kinematic_simulation_B.b_j = 1;
        while (kinematic_simulation_B.b_j + 3 < 5) {
          kinematic_simulation_B.b_V[kinematic_simulation_B.b_j + 10] =
            kinematic_simulation_B.b_V[kinematic_simulation_B.b_j + 6];
          kinematic_simulation_B.b_j++;
        }

        kinematic_simulation_B.b_V[4] = 0.0;
        kinematic_simulation_B.b_j = 0;
        while (kinematic_simulation_B.b_j + 3 < 5) {
          kinematic_simulation_B.b_V[kinematic_simulation_B.b_j + 6] =
            kinematic_simulation_B.b_V[kinematic_simulation_B.b_j + 2];
          kinematic_simulation_B.b_j++;
        }

        kinematic_simulation_B.work[0] = 0.0;
        kinematic_simulation_B.b_V[1] = 0.0;
        kinematic_simulation_B.work[1] = 0.0;
        kinematic_simulation_B.b_V[2] = 0.0;
        kinematic_simulation_B.work[2] = 0.0;
        kinematic_simulation_B.b_V[3] = 0.0;
        kinematic_simulation_B.work[3] = 0.0;
        kinematic_simulation_B.b_V[0] = 1.0;
        kinematic_simulation_B.b_V[15] = 1.0 - kinematic_simulation_B.tau[2];
        kinematic_simulation_B.b_j = 0;
        while (kinematic_simulation_B.b_j <= 1) {
          kinematic_simulation_B.b_V[14 - kinematic_simulation_B.b_j] = 0.0;
          kinematic_simulation_B.b_j++;
        }

        kinematic_simulation_B.b_V[10] = 1.0;
        kinematic_simulation_xzlarf(2, 1, 11, kinematic_simulation_B.tau[1],
          kinematic_simulation_B.b_V, 15, kinematic_simulation_B.work);
        kinematic_simulation_B.b_j = 11;
        while (kinematic_simulation_B.b_j + 1 <= 12) {
          kinematic_simulation_B.b_V[kinematic_simulation_B.b_j] *=
            -kinematic_simulation_B.tau[1];
          kinematic_simulation_B.b_j++;
        }

        kinematic_simulation_B.b_V[10] = 1.0 - kinematic_simulation_B.tau[1];
        kinematic_simulation_B.b_V[9] = 0.0;
        kinematic_simulation_B.b_V[5] = 1.0;
        kinematic_simulation_xzlarf(3, 2, 6, kinematic_simulation_B.tau[0],
          kinematic_simulation_B.b_V, 10, kinematic_simulation_B.work);
        kinematic_simulation_B.b_j = 6;
        while (kinematic_simulation_B.b_j + 1 <= 8) {
          kinematic_simulation_B.b_V[kinematic_simulation_B.b_j] *=
            -kinematic_simulation_B.tau[0];
          kinematic_simulation_B.b_j++;
        }

        kinematic_simulation_B.b_V[5] = 1.0 - kinematic_simulation_B.tau[0];
        kinematic_simulation_eml_dlahqr(kinematic_simulation_B.b_A,
          kinematic_simulation_B.b_V);
      }

      for (kinematic_simulation_B.b_j = 0; kinematic_simulation_B.b_j < 16;
           kinematic_simulation_B.b_j++) {
        V[kinematic_simulation_B.b_j].re =
          kinematic_simulation_B.b_V[kinematic_simulation_B.b_j];
        V[kinematic_simulation_B.b_j].im = 0.0;
      }

      D[0].re = kinematic_simulation_B.b_A[0];
      D[0].im = 0.0;
      D[1].re = kinematic_simulation_B.b_A[5];
      D[1].im = 0.0;
      D[2].re = kinematic_simulation_B.b_A[10];
      D[2].im = 0.0;
      D[3].re = kinematic_simulation_B.b_A[15];
      D[3].im = 0.0;
    } else {
      for (kinematic_simulation_B.b_j = 0; kinematic_simulation_B.b_j < 16;
           kinematic_simulation_B.b_j++) {
        kinematic_simulation_B.At[kinematic_simulation_B.b_j].re =
          A[kinematic_simulation_B.b_j];
        kinematic_simulation_B.At[kinematic_simulation_B.b_j].im = 0.0;
      }

      kinematic_simulation_xzggev(kinematic_simulation_B.At,
        &kinematic_simulation_B.b_j, D, kinematic_simulation_B.beta1, V);
      kinematic_simulation_B.colnorm = 0.0;
      kinematic_simulation_B.scale = 3.3121686421112381E-170;
      kinematic_simulation_B.b_j = 0;
      while (kinematic_simulation_B.b_j + 1 <= 4) {
        kinematic_simulation_B.absxk = fabs(V[kinematic_simulation_B.b_j].re);
        if (kinematic_simulation_B.absxk > kinematic_simulation_B.scale) {
          kinematic_simulation_B.t = kinematic_simulation_B.scale /
            kinematic_simulation_B.absxk;
          kinematic_simulation_B.colnorm = kinematic_simulation_B.colnorm *
            kinematic_simulation_B.t * kinematic_simulation_B.t + 1.0;
          kinematic_simulation_B.scale = kinematic_simulation_B.absxk;
        } else {
          kinematic_simulation_B.t = kinematic_simulation_B.absxk /
            kinematic_simulation_B.scale;
          kinematic_simulation_B.colnorm += kinematic_simulation_B.t *
            kinematic_simulation_B.t;
        }

        kinematic_simulation_B.absxk = fabs(V[kinematic_simulation_B.b_j].im);
        if (kinematic_simulation_B.absxk > kinematic_simulation_B.scale) {
          kinematic_simulation_B.t = kinematic_simulation_B.scale /
            kinematic_simulation_B.absxk;
          kinematic_simulation_B.colnorm = kinematic_simulation_B.colnorm *
            kinematic_simulation_B.t * kinematic_simulation_B.t + 1.0;
          kinematic_simulation_B.scale = kinematic_simulation_B.absxk;
        } else {
          kinematic_simulation_B.t = kinematic_simulation_B.absxk /
            kinematic_simulation_B.scale;
          kinematic_simulation_B.colnorm += kinematic_simulation_B.t *
            kinematic_simulation_B.t;
        }

        kinematic_simulation_B.b_j++;
      }

      kinematic_simulation_B.colnorm = kinematic_simulation_B.scale * sqrt
        (kinematic_simulation_B.colnorm);
      kinematic_simulation_B.b_j = 0;
      while (kinematic_simulation_B.b_j + 1 <= 4) {
        if (V[kinematic_simulation_B.b_j].im == 0.0) {
          kinematic_simulation_B.scale = V[kinematic_simulation_B.b_j].re /
            kinematic_simulation_B.colnorm;
          kinematic_simulation_B.absxk = 0.0;
        } else if (V[kinematic_simulation_B.b_j].re == 0.0) {
          kinematic_simulation_B.scale = 0.0;
          kinematic_simulation_B.absxk = V[kinematic_simulation_B.b_j].im /
            kinematic_simulation_B.colnorm;
        } else {
          kinematic_simulation_B.scale = V[kinematic_simulation_B.b_j].re /
            kinematic_simulation_B.colnorm;
          kinematic_simulation_B.absxk = V[kinematic_simulation_B.b_j].im /
            kinematic_simulation_B.colnorm;
        }

        V[kinematic_simulation_B.b_j].re = kinematic_simulation_B.scale;
        V[kinematic_simulation_B.b_j].im = kinematic_simulation_B.absxk;
        kinematic_simulation_B.b_j++;
      }

      kinematic_simulation_B.colnorm = 0.0;
      kinematic_simulation_B.scale = 3.3121686421112381E-170;
      kinematic_simulation_B.b_j = 4;
      while (kinematic_simulation_B.b_j + 1 <= 8) {
        kinematic_simulation_B.absxk = fabs(V[kinematic_simulation_B.b_j].re);
        if (kinematic_simulation_B.absxk > kinematic_simulation_B.scale) {
          kinematic_simulation_B.t = kinematic_simulation_B.scale /
            kinematic_simulation_B.absxk;
          kinematic_simulation_B.colnorm = kinematic_simulation_B.colnorm *
            kinematic_simulation_B.t * kinematic_simulation_B.t + 1.0;
          kinematic_simulation_B.scale = kinematic_simulation_B.absxk;
        } else {
          kinematic_simulation_B.t = kinematic_simulation_B.absxk /
            kinematic_simulation_B.scale;
          kinematic_simulation_B.colnorm += kinematic_simulation_B.t *
            kinematic_simulation_B.t;
        }

        kinematic_simulation_B.absxk = fabs(V[kinematic_simulation_B.b_j].im);
        if (kinematic_simulation_B.absxk > kinematic_simulation_B.scale) {
          kinematic_simulation_B.t = kinematic_simulation_B.scale /
            kinematic_simulation_B.absxk;
          kinematic_simulation_B.colnorm = kinematic_simulation_B.colnorm *
            kinematic_simulation_B.t * kinematic_simulation_B.t + 1.0;
          kinematic_simulation_B.scale = kinematic_simulation_B.absxk;
        } else {
          kinematic_simulation_B.t = kinematic_simulation_B.absxk /
            kinematic_simulation_B.scale;
          kinematic_simulation_B.colnorm += kinematic_simulation_B.t *
            kinematic_simulation_B.t;
        }

        kinematic_simulation_B.b_j++;
      }

      kinematic_simulation_B.colnorm = kinematic_simulation_B.scale * sqrt
        (kinematic_simulation_B.colnorm);
      kinematic_simulation_B.b_j = 4;
      while (kinematic_simulation_B.b_j + 1 <= 8) {
        if (V[kinematic_simulation_B.b_j].im == 0.0) {
          kinematic_simulation_B.scale = V[kinematic_simulation_B.b_j].re /
            kinematic_simulation_B.colnorm;
          kinematic_simulation_B.absxk = 0.0;
        } else if (V[kinematic_simulation_B.b_j].re == 0.0) {
          kinematic_simulation_B.scale = 0.0;
          kinematic_simulation_B.absxk = V[kinematic_simulation_B.b_j].im /
            kinematic_simulation_B.colnorm;
        } else {
          kinematic_simulation_B.scale = V[kinematic_simulation_B.b_j].re /
            kinematic_simulation_B.colnorm;
          kinematic_simulation_B.absxk = V[kinematic_simulation_B.b_j].im /
            kinematic_simulation_B.colnorm;
        }

        V[kinematic_simulation_B.b_j].re = kinematic_simulation_B.scale;
        V[kinematic_simulation_B.b_j].im = kinematic_simulation_B.absxk;
        kinematic_simulation_B.b_j++;
      }

      kinematic_simulation_B.colnorm = 0.0;
      kinematic_simulation_B.scale = 3.3121686421112381E-170;
      kinematic_simulation_B.b_j = 8;
      while (kinematic_simulation_B.b_j + 1 <= 12) {
        kinematic_simulation_B.absxk = fabs(V[kinematic_simulation_B.b_j].re);
        if (kinematic_simulation_B.absxk > kinematic_simulation_B.scale) {
          kinematic_simulation_B.t = kinematic_simulation_B.scale /
            kinematic_simulation_B.absxk;
          kinematic_simulation_B.colnorm = kinematic_simulation_B.colnorm *
            kinematic_simulation_B.t * kinematic_simulation_B.t + 1.0;
          kinematic_simulation_B.scale = kinematic_simulation_B.absxk;
        } else {
          kinematic_simulation_B.t = kinematic_simulation_B.absxk /
            kinematic_simulation_B.scale;
          kinematic_simulation_B.colnorm += kinematic_simulation_B.t *
            kinematic_simulation_B.t;
        }

        kinematic_simulation_B.absxk = fabs(V[kinematic_simulation_B.b_j].im);
        if (kinematic_simulation_B.absxk > kinematic_simulation_B.scale) {
          kinematic_simulation_B.t = kinematic_simulation_B.scale /
            kinematic_simulation_B.absxk;
          kinematic_simulation_B.colnorm = kinematic_simulation_B.colnorm *
            kinematic_simulation_B.t * kinematic_simulation_B.t + 1.0;
          kinematic_simulation_B.scale = kinematic_simulation_B.absxk;
        } else {
          kinematic_simulation_B.t = kinematic_simulation_B.absxk /
            kinematic_simulation_B.scale;
          kinematic_simulation_B.colnorm += kinematic_simulation_B.t *
            kinematic_simulation_B.t;
        }

        kinematic_simulation_B.b_j++;
      }

      kinematic_simulation_B.colnorm = kinematic_simulation_B.scale * sqrt
        (kinematic_simulation_B.colnorm);
      kinematic_simulation_B.b_j = 8;
      while (kinematic_simulation_B.b_j + 1 <= 12) {
        if (V[kinematic_simulation_B.b_j].im == 0.0) {
          kinematic_simulation_B.scale = V[kinematic_simulation_B.b_j].re /
            kinematic_simulation_B.colnorm;
          kinematic_simulation_B.absxk = 0.0;
        } else if (V[kinematic_simulation_B.b_j].re == 0.0) {
          kinematic_simulation_B.scale = 0.0;
          kinematic_simulation_B.absxk = V[kinematic_simulation_B.b_j].im /
            kinematic_simulation_B.colnorm;
        } else {
          kinematic_simulation_B.scale = V[kinematic_simulation_B.b_j].re /
            kinematic_simulation_B.colnorm;
          kinematic_simulation_B.absxk = V[kinematic_simulation_B.b_j].im /
            kinematic_simulation_B.colnorm;
        }

        V[kinematic_simulation_B.b_j].re = kinematic_simulation_B.scale;
        V[kinematic_simulation_B.b_j].im = kinematic_simulation_B.absxk;
        kinematic_simulation_B.b_j++;
      }

      kinematic_simulation_B.colnorm = 0.0;
      kinematic_simulation_B.scale = 3.3121686421112381E-170;
      kinematic_simulation_B.b_j = 12;
      while (kinematic_simulation_B.b_j + 1 <= 16) {
        kinematic_simulation_B.absxk = fabs(V[kinematic_simulation_B.b_j].re);
        if (kinematic_simulation_B.absxk > kinematic_simulation_B.scale) {
          kinematic_simulation_B.t = kinematic_simulation_B.scale /
            kinematic_simulation_B.absxk;
          kinematic_simulation_B.colnorm = kinematic_simulation_B.colnorm *
            kinematic_simulation_B.t * kinematic_simulation_B.t + 1.0;
          kinematic_simulation_B.scale = kinematic_simulation_B.absxk;
        } else {
          kinematic_simulation_B.t = kinematic_simulation_B.absxk /
            kinematic_simulation_B.scale;
          kinematic_simulation_B.colnorm += kinematic_simulation_B.t *
            kinematic_simulation_B.t;
        }

        kinematic_simulation_B.absxk = fabs(V[kinematic_simulation_B.b_j].im);
        if (kinematic_simulation_B.absxk > kinematic_simulation_B.scale) {
          kinematic_simulation_B.t = kinematic_simulation_B.scale /
            kinematic_simulation_B.absxk;
          kinematic_simulation_B.colnorm = kinematic_simulation_B.colnorm *
            kinematic_simulation_B.t * kinematic_simulation_B.t + 1.0;
          kinematic_simulation_B.scale = kinematic_simulation_B.absxk;
        } else {
          kinematic_simulation_B.t = kinematic_simulation_B.absxk /
            kinematic_simulation_B.scale;
          kinematic_simulation_B.colnorm += kinematic_simulation_B.t *
            kinematic_simulation_B.t;
        }

        kinematic_simulation_B.b_j++;
      }

      kinematic_simulation_B.colnorm = kinematic_simulation_B.scale * sqrt
        (kinematic_simulation_B.colnorm);
      kinematic_simulation_B.b_j = 12;
      while (kinematic_simulation_B.b_j + 1 <= 16) {
        if (V[kinematic_simulation_B.b_j].im == 0.0) {
          kinematic_simulation_B.scale = V[kinematic_simulation_B.b_j].re /
            kinematic_simulation_B.colnorm;
          kinematic_simulation_B.absxk = 0.0;
        } else if (V[kinematic_simulation_B.b_j].re == 0.0) {
          kinematic_simulation_B.scale = 0.0;
          kinematic_simulation_B.absxk = V[kinematic_simulation_B.b_j].im /
            kinematic_simulation_B.colnorm;
        } else {
          kinematic_simulation_B.scale = V[kinematic_simulation_B.b_j].re /
            kinematic_simulation_B.colnorm;
          kinematic_simulation_B.absxk = V[kinematic_simulation_B.b_j].im /
            kinematic_simulation_B.colnorm;
        }

        V[kinematic_simulation_B.b_j].re = kinematic_simulation_B.scale;
        V[kinematic_simulation_B.b_j].im = kinematic_simulation_B.absxk;
        kinematic_simulation_B.b_j++;
      }

      if (kinematic_simulation_B.beta1[0].im == 0.0) {
        if (D[0].im == 0.0) {
          kinematic_simulation_B.scale = D[0].re / kinematic_simulation_B.beta1
            [0].re;
          kinematic_simulation_B.absxk = 0.0;
        } else if (D[0].re == 0.0) {
          kinematic_simulation_B.scale = 0.0;
          kinematic_simulation_B.absxk = D[0].im / kinematic_simulation_B.beta1
            [0].re;
        } else {
          kinematic_simulation_B.scale = D[0].re / kinematic_simulation_B.beta1
            [0].re;
          kinematic_simulation_B.absxk = D[0].im / kinematic_simulation_B.beta1
            [0].re;
        }
      } else if (kinematic_simulation_B.beta1[0].re == 0.0) {
        if (D[0].re == 0.0) {
          kinematic_simulation_B.scale = D[0].im / kinematic_simulation_B.beta1
            [0].im;
          kinematic_simulation_B.absxk = 0.0;
        } else if (D[0].im == 0.0) {
          kinematic_simulation_B.scale = 0.0;
          kinematic_simulation_B.absxk = -(D[0].re /
            kinematic_simulation_B.beta1[0].im);
        } else {
          kinematic_simulation_B.scale = D[0].im / kinematic_simulation_B.beta1
            [0].im;
          kinematic_simulation_B.absxk = -(D[0].re /
            kinematic_simulation_B.beta1[0].im);
        }
      } else {
        kinematic_simulation_B.colnorm = fabs(kinematic_simulation_B.beta1[0].re);
        kinematic_simulation_B.scale = fabs(kinematic_simulation_B.beta1[0].im);
        if (kinematic_simulation_B.colnorm > kinematic_simulation_B.scale) {
          kinematic_simulation_B.colnorm = kinematic_simulation_B.beta1[0].im /
            kinematic_simulation_B.beta1[0].re;
          kinematic_simulation_B.absxk = kinematic_simulation_B.colnorm *
            kinematic_simulation_B.beta1[0].im + kinematic_simulation_B.beta1[0]
            .re;
          kinematic_simulation_B.scale = (kinematic_simulation_B.colnorm * D[0].
            im + D[0].re) / kinematic_simulation_B.absxk;
          kinematic_simulation_B.absxk = (D[0].im -
            kinematic_simulation_B.colnorm * D[0].re) /
            kinematic_simulation_B.absxk;
        } else if (kinematic_simulation_B.scale ==
                   kinematic_simulation_B.colnorm) {
          kinematic_simulation_B.absxk = kinematic_simulation_B.beta1[0].re >
            0.0 ? 0.5 : -0.5;
          kinematic_simulation_B.t = kinematic_simulation_B.beta1[0].im > 0.0 ?
            0.5 : -0.5;
          kinematic_simulation_B.scale = (D[0].re * kinematic_simulation_B.absxk
            + D[0].im * kinematic_simulation_B.t) /
            kinematic_simulation_B.colnorm;
          kinematic_simulation_B.absxk = (D[0].im * kinematic_simulation_B.absxk
            - D[0].re * kinematic_simulation_B.t) /
            kinematic_simulation_B.colnorm;
        } else {
          kinematic_simulation_B.colnorm = kinematic_simulation_B.beta1[0].re /
            kinematic_simulation_B.beta1[0].im;
          kinematic_simulation_B.absxk = kinematic_simulation_B.colnorm *
            kinematic_simulation_B.beta1[0].re + kinematic_simulation_B.beta1[0]
            .im;
          kinematic_simulation_B.scale = (kinematic_simulation_B.colnorm * D[0].
            re + D[0].im) / kinematic_simulation_B.absxk;
          kinematic_simulation_B.absxk = (kinematic_simulation_B.colnorm * D[0].
            im - D[0].re) / kinematic_simulation_B.absxk;
        }
      }

      D[0].re = kinematic_simulation_B.scale;
      D[0].im = kinematic_simulation_B.absxk;
      if (kinematic_simulation_B.beta1[1].im == 0.0) {
        if (D[1].im == 0.0) {
          kinematic_simulation_B.scale = D[1].re / kinematic_simulation_B.beta1
            [1].re;
          kinematic_simulation_B.absxk = 0.0;
        } else if (D[1].re == 0.0) {
          kinematic_simulation_B.scale = 0.0;
          kinematic_simulation_B.absxk = D[1].im / kinematic_simulation_B.beta1
            [1].re;
        } else {
          kinematic_simulation_B.scale = D[1].re / kinematic_simulation_B.beta1
            [1].re;
          kinematic_simulation_B.absxk = D[1].im / kinematic_simulation_B.beta1
            [1].re;
        }
      } else if (kinematic_simulation_B.beta1[1].re == 0.0) {
        if (D[1].re == 0.0) {
          kinematic_simulation_B.scale = D[1].im / kinematic_simulation_B.beta1
            [1].im;
          kinematic_simulation_B.absxk = 0.0;
        } else if (D[1].im == 0.0) {
          kinematic_simulation_B.scale = 0.0;
          kinematic_simulation_B.absxk = -(D[1].re /
            kinematic_simulation_B.beta1[1].im);
        } else {
          kinematic_simulation_B.scale = D[1].im / kinematic_simulation_B.beta1
            [1].im;
          kinematic_simulation_B.absxk = -(D[1].re /
            kinematic_simulation_B.beta1[1].im);
        }
      } else {
        kinematic_simulation_B.colnorm = fabs(kinematic_simulation_B.beta1[1].re);
        kinematic_simulation_B.scale = fabs(kinematic_simulation_B.beta1[1].im);
        if (kinematic_simulation_B.colnorm > kinematic_simulation_B.scale) {
          kinematic_simulation_B.colnorm = kinematic_simulation_B.beta1[1].im /
            kinematic_simulation_B.beta1[1].re;
          kinematic_simulation_B.absxk = kinematic_simulation_B.colnorm *
            kinematic_simulation_B.beta1[1].im + kinematic_simulation_B.beta1[1]
            .re;
          kinematic_simulation_B.scale = (kinematic_simulation_B.colnorm * D[1].
            im + D[1].re) / kinematic_simulation_B.absxk;
          kinematic_simulation_B.absxk = (D[1].im -
            kinematic_simulation_B.colnorm * D[1].re) /
            kinematic_simulation_B.absxk;
        } else if (kinematic_simulation_B.scale ==
                   kinematic_simulation_B.colnorm) {
          kinematic_simulation_B.absxk = kinematic_simulation_B.beta1[1].re >
            0.0 ? 0.5 : -0.5;
          kinematic_simulation_B.t = kinematic_simulation_B.beta1[1].im > 0.0 ?
            0.5 : -0.5;
          kinematic_simulation_B.scale = (D[1].re * kinematic_simulation_B.absxk
            + D[1].im * kinematic_simulation_B.t) /
            kinematic_simulation_B.colnorm;
          kinematic_simulation_B.absxk = (D[1].im * kinematic_simulation_B.absxk
            - D[1].re * kinematic_simulation_B.t) /
            kinematic_simulation_B.colnorm;
        } else {
          kinematic_simulation_B.colnorm = kinematic_simulation_B.beta1[1].re /
            kinematic_simulation_B.beta1[1].im;
          kinematic_simulation_B.absxk = kinematic_simulation_B.colnorm *
            kinematic_simulation_B.beta1[1].re + kinematic_simulation_B.beta1[1]
            .im;
          kinematic_simulation_B.scale = (kinematic_simulation_B.colnorm * D[1].
            re + D[1].im) / kinematic_simulation_B.absxk;
          kinematic_simulation_B.absxk = (kinematic_simulation_B.colnorm * D[1].
            im - D[1].re) / kinematic_simulation_B.absxk;
        }
      }

      D[1].re = kinematic_simulation_B.scale;
      D[1].im = kinematic_simulation_B.absxk;
      if (kinematic_simulation_B.beta1[2].im == 0.0) {
        if (D[2].im == 0.0) {
          kinematic_simulation_B.scale = D[2].re / kinematic_simulation_B.beta1
            [2].re;
          kinematic_simulation_B.absxk = 0.0;
        } else if (D[2].re == 0.0) {
          kinematic_simulation_B.scale = 0.0;
          kinematic_simulation_B.absxk = D[2].im / kinematic_simulation_B.beta1
            [2].re;
        } else {
          kinematic_simulation_B.scale = D[2].re / kinematic_simulation_B.beta1
            [2].re;
          kinematic_simulation_B.absxk = D[2].im / kinematic_simulation_B.beta1
            [2].re;
        }
      } else if (kinematic_simulation_B.beta1[2].re == 0.0) {
        if (D[2].re == 0.0) {
          kinematic_simulation_B.scale = D[2].im / kinematic_simulation_B.beta1
            [2].im;
          kinematic_simulation_B.absxk = 0.0;
        } else if (D[2].im == 0.0) {
          kinematic_simulation_B.scale = 0.0;
          kinematic_simulation_B.absxk = -(D[2].re /
            kinematic_simulation_B.beta1[2].im);
        } else {
          kinematic_simulation_B.scale = D[2].im / kinematic_simulation_B.beta1
            [2].im;
          kinematic_simulation_B.absxk = -(D[2].re /
            kinematic_simulation_B.beta1[2].im);
        }
      } else {
        kinematic_simulation_B.colnorm = fabs(kinematic_simulation_B.beta1[2].re);
        kinematic_simulation_B.scale = fabs(kinematic_simulation_B.beta1[2].im);
        if (kinematic_simulation_B.colnorm > kinematic_simulation_B.scale) {
          kinematic_simulation_B.colnorm = kinematic_simulation_B.beta1[2].im /
            kinematic_simulation_B.beta1[2].re;
          kinematic_simulation_B.absxk = kinematic_simulation_B.colnorm *
            kinematic_simulation_B.beta1[2].im + kinematic_simulation_B.beta1[2]
            .re;
          kinematic_simulation_B.scale = (kinematic_simulation_B.colnorm * D[2].
            im + D[2].re) / kinematic_simulation_B.absxk;
          kinematic_simulation_B.absxk = (D[2].im -
            kinematic_simulation_B.colnorm * D[2].re) /
            kinematic_simulation_B.absxk;
        } else if (kinematic_simulation_B.scale ==
                   kinematic_simulation_B.colnorm) {
          kinematic_simulation_B.absxk = kinematic_simulation_B.beta1[2].re >
            0.0 ? 0.5 : -0.5;
          kinematic_simulation_B.t = kinematic_simulation_B.beta1[2].im > 0.0 ?
            0.5 : -0.5;
          kinematic_simulation_B.scale = (D[2].re * kinematic_simulation_B.absxk
            + D[2].im * kinematic_simulation_B.t) /
            kinematic_simulation_B.colnorm;
          kinematic_simulation_B.absxk = (D[2].im * kinematic_simulation_B.absxk
            - D[2].re * kinematic_simulation_B.t) /
            kinematic_simulation_B.colnorm;
        } else {
          kinematic_simulation_B.colnorm = kinematic_simulation_B.beta1[2].re /
            kinematic_simulation_B.beta1[2].im;
          kinematic_simulation_B.absxk = kinematic_simulation_B.colnorm *
            kinematic_simulation_B.beta1[2].re + kinematic_simulation_B.beta1[2]
            .im;
          kinematic_simulation_B.scale = (kinematic_simulation_B.colnorm * D[2].
            re + D[2].im) / kinematic_simulation_B.absxk;
          kinematic_simulation_B.absxk = (kinematic_simulation_B.colnorm * D[2].
            im - D[2].re) / kinematic_simulation_B.absxk;
        }
      }

      D[2].re = kinematic_simulation_B.scale;
      D[2].im = kinematic_simulation_B.absxk;
      if (kinematic_simulation_B.beta1[3].im == 0.0) {
        if (D[3].im == 0.0) {
          kinematic_simulation_B.scale = D[3].re / kinematic_simulation_B.beta1
            [3].re;
          kinematic_simulation_B.absxk = 0.0;
        } else if (D[3].re == 0.0) {
          kinematic_simulation_B.scale = 0.0;
          kinematic_simulation_B.absxk = D[3].im / kinematic_simulation_B.beta1
            [3].re;
        } else {
          kinematic_simulation_B.scale = D[3].re / kinematic_simulation_B.beta1
            [3].re;
          kinematic_simulation_B.absxk = D[3].im / kinematic_simulation_B.beta1
            [3].re;
        }
      } else if (kinematic_simulation_B.beta1[3].re == 0.0) {
        if (D[3].re == 0.0) {
          kinematic_simulation_B.scale = D[3].im / kinematic_simulation_B.beta1
            [3].im;
          kinematic_simulation_B.absxk = 0.0;
        } else if (D[3].im == 0.0) {
          kinematic_simulation_B.scale = 0.0;
          kinematic_simulation_B.absxk = -(D[3].re /
            kinematic_simulation_B.beta1[3].im);
        } else {
          kinematic_simulation_B.scale = D[3].im / kinematic_simulation_B.beta1
            [3].im;
          kinematic_simulation_B.absxk = -(D[3].re /
            kinematic_simulation_B.beta1[3].im);
        }
      } else {
        kinematic_simulation_B.colnorm = fabs(kinematic_simulation_B.beta1[3].re);
        kinematic_simulation_B.scale = fabs(kinematic_simulation_B.beta1[3].im);
        if (kinematic_simulation_B.colnorm > kinematic_simulation_B.scale) {
          kinematic_simulation_B.colnorm = kinematic_simulation_B.beta1[3].im /
            kinematic_simulation_B.beta1[3].re;
          kinematic_simulation_B.absxk = kinematic_simulation_B.colnorm *
            kinematic_simulation_B.beta1[3].im + kinematic_simulation_B.beta1[3]
            .re;
          kinematic_simulation_B.scale = (kinematic_simulation_B.colnorm * D[3].
            im + D[3].re) / kinematic_simulation_B.absxk;
          kinematic_simulation_B.absxk = (D[3].im -
            kinematic_simulation_B.colnorm * D[3].re) /
            kinematic_simulation_B.absxk;
        } else if (kinematic_simulation_B.scale ==
                   kinematic_simulation_B.colnorm) {
          kinematic_simulation_B.absxk = kinematic_simulation_B.beta1[3].re >
            0.0 ? 0.5 : -0.5;
          kinematic_simulation_B.t = kinematic_simulation_B.beta1[3].im > 0.0 ?
            0.5 : -0.5;
          kinematic_simulation_B.scale = (D[3].re * kinematic_simulation_B.absxk
            + D[3].im * kinematic_simulation_B.t) /
            kinematic_simulation_B.colnorm;
          kinematic_simulation_B.absxk = (D[3].im * kinematic_simulation_B.absxk
            - D[3].re * kinematic_simulation_B.t) /
            kinematic_simulation_B.colnorm;
        } else {
          kinematic_simulation_B.colnorm = kinematic_simulation_B.beta1[3].re /
            kinematic_simulation_B.beta1[3].im;
          kinematic_simulation_B.absxk = kinematic_simulation_B.colnorm *
            kinematic_simulation_B.beta1[3].re + kinematic_simulation_B.beta1[3]
            .im;
          kinematic_simulation_B.scale = (kinematic_simulation_B.colnorm * D[3].
            re + D[3].im) / kinematic_simulation_B.absxk;
          kinematic_simulation_B.absxk = (kinematic_simulation_B.colnorm * D[3].
            im - D[3].re) / kinematic_simulation_B.absxk;
        }
      }

      D[3].re = kinematic_simulation_B.scale;
      D[3].im = kinematic_simulation_B.absxk;
    }
  }
}

static void matlabCodegenHandle_matla_alw5t(ros_slros_internal_block_GetP_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_kinematic_si_T
  *pStruct)
{
  kinematic_simula_emxFree_char_T(&pStruct->Type);
  kinematic_simula_emxFree_real_T(&pStruct->MotionSubspace);
}

static void emxFreeStruct_o_robotics_manip_(o_robotics_manip_internal_Rig_T
  *pStruct)
{
  kinematic_simula_emxFree_char_T(&pStruct->NameInternal);
  emxFreeStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static void emxFreeStruct_p_robotics_manip_(p_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_o_robotics_manip_(&pStruct->Base);
}

static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxFreeStruct_p_robotics_manip_(&pStruct->TreeInternal);
}

static void emxFreeStruct_n_robotics_manip_(n_robotics_manip_internal_Rig_T
  *pStruct)
{
  kinematic_simula_emxFree_char_T(&pStruct->NameInternal);
  emxFreeStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static void emxFreeStruct_c_rigidBodyJoint1(c_rigidBodyJoint_kinematic__a_T
  *pStruct)
{
  kinematic_simula_emxFree_char_T(&pStruct->Type);
}

static void emxFreeStruct_o_robotics_mani_a(o_robotics_manip_internal_R_a_T
  *pStruct)
{
  kinematic_simula_emxFree_char_T(&pStruct->NameInternal);
  emxFreeStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static void emxFreeStruct_p_robotics_mani_a(p_robotics_manip_internal_R_a_T
  *pStruct)
{
  emxFreeStruct_o_robotics_mani_a(&pStruct->Base);
}

static void emxFreeStruct_robotics_slmani_a(robotics_slmanip_internal_b_a_T
  *pStruct)
{
  emxFreeStruct_p_robotics_mani_a(&pStruct->TreeInternal);
}

static void emxFreeStruct_n_robotics_mani_a(n_robotics_manip_internal_R_a_T
  *pStruct)
{
  kinematic_simula_emxFree_char_T(&pStruct->NameInternal);
  emxFreeStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static void matlabCodegenHandle_matlab_alw5(ros_slros_internal_block_Subs_T *obj)
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

static void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_kinematic_si_T
  *pStruct)
{
  kinematic_simula_emxInit_char_T(&pStruct->Type, 2);
  kinematic_simula_emxInit_real_T(&pStruct->MotionSubspace, 2);
}

static void emxInitStruct_o_robotics_manip_(o_robotics_manip_internal_Rig_T
  *pStruct)
{
  kinematic_simula_emxInit_char_T(&pStruct->NameInternal, 2);
  emxInitStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static void emxInitStruct_p_robotics_manip_(p_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_o_robotics_manip_(&pStruct->Base);
}

static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxInitStruct_p_robotics_manip_(&pStruct->TreeInternal);
}

static void emxInitStruct_n_robotics_manip_(n_robotics_manip_internal_Rig_T
  *pStruct)
{
  kinematic_simula_emxInit_char_T(&pStruct->NameInternal, 2);
  emxInitStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static n_robotics_manip_internal_Rig_T *kinematic_s_RigidBody_RigidBody
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_kinematic_sim_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[13] = { 'e', 'd', 'o', '_', 'b', 'a', 's', 'e', '_',
    'l', 'i', 'n', 'k' };

  static const char_T tmp_1[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_2[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_3[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_4[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 13;
  kinema_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 13; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 0.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  kinema_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  kinematic_simula_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinema_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
      b_0[b_kstr] = tmp_3[b_kstr];
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

  kinematic_simula_emxFree_char_T(&switch_expression);
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
  kinema_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_4[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_5[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  kinema_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *kinematic_RigidBody_RigidBody_a
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_kinematic_sim_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[10] = { 'e', 'd', 'o', '_', 'l', 'i', 'n', 'k', '_',
    '1' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.337, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  kinema_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 1.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  kinema_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  kinematic_simula_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinema_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  kinematic_simula_emxFree_char_T(&switch_expression);
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
  kinema_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  kinema_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *kinemati_RigidBody_RigidBody_al
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_kinematic_sim_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[10] = { 'e', 'd', 'o', '_', 'l', 'i', 'n', 'k', '_',
    '2' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0,
    4.8965888601467475E-12, 1.0, 0.0, 0.0, -1.0, 4.8965888601467475E-12, 0.0,
    0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  kinema_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 2.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  kinema_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  kinematic_simula_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinema_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  kinematic_simula_emxFree_char_T(&switch_expression);
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
  kinema_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  kinema_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = -1.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *kinemat_RigidBody_RigidBody_alw
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_kinematic_sim_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[10] = { 'e', 'd', 'o', '_', 'l', 'i', 'n', 'k', '_',
    '3' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 2.0682310711021444E-13,
    2.0682310711021444E-13, 0.0, 2.0682310711021444E-13, -1.0, -0.0, 0.0,
    2.0682310711021444E-13, 4.2775797634723234E-26, -1.0, 0.0, 0.0, 0.2105, 0.0,
    1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  kinema_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 3.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  kinema_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  kinematic_simula_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinema_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  kinematic_simula_emxFree_char_T(&switch_expression);
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
  kinema_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  kinema_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *kinema_RigidBody_RigidBody_alw5
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_kinematic_sim_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[10] = { 'e', 'd', 'o', '_', 'l', 'i', 'n', 'k', '_',
    '4' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0,
    4.8965888601467475E-12, 1.0, 0.0, 0.0, -1.0, 4.8965888601467475E-12, 0.0,
    0.0, -0.268, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  kinema_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 4.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  kinema_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  kinematic_simula_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinema_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  kinematic_simula_emxFree_char_T(&switch_expression);
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
  kinema_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  kinema_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *kinem_RigidBody_RigidBody_alw5t
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_kinematic_sim_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[10] = { 'e', 'd', 'o', '_', 'l', 'i', 'n', 'k', '_',
    '5' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  kinema_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 5.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  kinema_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  kinematic_simula_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinema_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  kinematic_simula_emxFree_char_T(&switch_expression);
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
  kinema_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  kinema_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 1.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *kine_RigidBody_RigidBody_alw5ty
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_kinematic_sim_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[10] = { 'e', 'd', 'o', '_', 'l', 'i', 'n', 'k', '_',
    '6' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.1745, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  kinema_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 6.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  kinema_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  kinematic_simula_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinema_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  kinematic_simula_emxFree_char_T(&switch_expression);
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
  kinema_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  kinema_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *kin_RigidBody_RigidBody_alw5typ
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_kinematic_sim_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[11] = { 'e', 'd', 'o', '_', 'l', 'i', 'n', 'k', '_',
    'e', 'e' };

  static const char_T tmp_1[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_2[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_3[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_4[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 11;
  kinema_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 11; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 7.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  kinema_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  kinematic_simula_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinema_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
      b_0[b_kstr] = tmp_3[b_kstr];
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

  kinematic_simula_emxFree_char_T(&switch_expression);
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
  kinema_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_4[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_5[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  kinema_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static o_robotics_manip_internal_Rig_T *ki_RigidBody_RigidBody_alw5typh
  (o_robotics_manip_internal_Rig_T *obj)
{
  o_robotics_manip_internal_Rig_T *b_obj;
  emxArray_char_T_kinematic_sim_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[5] = { 'w', 'o', 'r', 'l', 'd' };

  static const char_T tmp_1[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_2[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_3[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 5;
  kinema_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  kinema_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  kinematic_simula_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinema_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
      kinematic_simulation_B.b_h[b_kstr] = tmp_3[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] !=
              kinematic_simulation_B.b_h[loop_ub]) {
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

  kinematic_simula_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      kinematic_simulation_B.msubspace_data[b_kstr] = tmp[b_kstr];
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
      kinematic_simulation_B.msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      kinematic_simulation_B.msubspace_data[b_kstr] = 0;
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
  kinema_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] =
      kinematic_simulation_B.msubspace_data[b_kstr];
  }

  return b_obj;
}

static p_robotics_manip_internal_Rig_T *kin_RigidBodyTree_RigidBodyTree
  (p_robotics_manip_internal_Rig_T *obj, n_robotics_manip_internal_Rig_T *iobj_0,
   n_robotics_manip_internal_Rig_T *iobj_1, n_robotics_manip_internal_Rig_T
   *iobj_2, n_robotics_manip_internal_Rig_T *iobj_3,
   n_robotics_manip_internal_Rig_T *iobj_4, n_robotics_manip_internal_Rig_T
   *iobj_5, n_robotics_manip_internal_Rig_T *iobj_6,
   n_robotics_manip_internal_Rig_T *iobj_7)
{
  p_robotics_manip_internal_Rig_T *b_obj;
  int32_T i;
  static const int8_T tmp[16] = { 0, 1, 2, 3, 4, 5, 6, 0, -1, 1, 2, 3, 4, 5, 6,
    -1 };

  b_obj = obj;
  obj->Bodies[0] = kinematic_s_RigidBody_RigidBody(iobj_0);
  obj->Bodies[0]->Index = 1.0;
  obj->Bodies[1] = kinematic_RigidBody_RigidBody_a(iobj_7);
  obj->Bodies[1]->Index = 2.0;
  obj->Bodies[2] = kinemati_RigidBody_RigidBody_al(iobj_1);
  obj->Bodies[2]->Index = 3.0;
  obj->Bodies[3] = kinemat_RigidBody_RigidBody_alw(iobj_2);
  obj->Bodies[3]->Index = 4.0;
  obj->Bodies[4] = kinema_RigidBody_RigidBody_alw5(iobj_3);
  obj->Bodies[4]->Index = 5.0;
  obj->Bodies[5] = kinem_RigidBody_RigidBody_alw5t(iobj_4);
  obj->Bodies[5]->Index = 6.0;
  obj->Bodies[6] = kine_RigidBody_RigidBody_alw5ty(iobj_5);
  obj->Bodies[6]->Index = 7.0;
  obj->Bodies[7] = kin_RigidBody_RigidBody_alw5typ(iobj_6);
  obj->Bodies[7]->Index = 8.0;
  obj->NumBodies = 8.0;
  obj->PositionNumber = 6.0;
  obj->VelocityNumber = 6.0;
  for (i = 0; i < 16; i++) {
    obj->PositionDoFMap[i] = tmp[i];
  }

  ki_RigidBody_RigidBody_alw5typh(&obj->Base);
  return b_obj;
}

static void emxInitStruct_c_rigidBodyJoint1(c_rigidBodyJoint_kinematic__a_T
  *pStruct)
{
  kinematic_simula_emxInit_char_T(&pStruct->Type, 2);
}

static void emxInitStruct_o_robotics_mani_a(o_robotics_manip_internal_R_a_T
  *pStruct)
{
  kinematic_simula_emxInit_char_T(&pStruct->NameInternal, 2);
  emxInitStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static void emxInitStruct_p_robotics_mani_a(p_robotics_manip_internal_R_a_T
  *pStruct)
{
  emxInitStruct_o_robotics_mani_a(&pStruct->Base);
}

static void emxInitStruct_robotics_slmani_a(robotics_slmanip_internal_b_a_T
  *pStruct)
{
  emxInitStruct_p_robotics_mani_a(&pStruct->TreeInternal);
}

static void emxInitStruct_n_robotics_mani_a(n_robotics_manip_internal_R_a_T
  *pStruct)
{
  kinematic_simula_emxInit_char_T(&pStruct->NameInternal, 2);
  emxInitStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static n_robotics_manip_internal_R_a_T *k_RigidBody_RigidBody_alw5typh4
  (n_robotics_manip_internal_R_a_T *obj)
{
  n_robotics_manip_internal_R_a_T *b_obj;
  emxArray_char_T_kinematic_sim_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  static const char_T tmp[13] = { 'e', 'd', 'o', '_', 'b', 'a', 's', 'e', '_',
    'l', 'i', 'n', 'k' };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 13;
  kinema_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 13; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 0.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  kinema_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  kinematic_simula_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinema_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  kinematic_simula_emxFree_char_T(&switch_expression);
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
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static n_robotics_manip_internal_R_a_T *RigidBody_RigidBody_alw5typh44
  (n_robotics_manip_internal_R_a_T *obj)
{
  n_robotics_manip_internal_R_a_T *b_obj;
  emxArray_char_T_kinematic_sim_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  static const char_T tmp[10] = { 'e', 'd', 'o', '_', 'l', 'i', 'n', 'k', '_',
    '1' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_2[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.337, 1.0 };

  static const real_T tmp_3[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  kinema_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 1.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  kinema_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  kinematic_simula_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinema_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  kinematic_simula_emxFree_char_T(&switch_expression);
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
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static n_robotics_manip_internal_R_a_T *RigidBody_RigidBody_alw5typh44u
  (n_robotics_manip_internal_R_a_T *obj)
{
  n_robotics_manip_internal_R_a_T *b_obj;
  emxArray_char_T_kinematic_sim_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  static const char_T tmp[10] = { 'e', 'd', 'o', '_', 'l', 'i', 'n', 'k', '_',
    '2' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_2[16] = { 1.0, 0.0, -0.0, 0.0, 0.0,
    4.8965888601467475E-12, 1.0, 0.0, 0.0, -1.0, 4.8965888601467475E-12, 0.0,
    0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_3[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  kinema_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 2.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  kinema_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  kinematic_simula_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinema_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  kinematic_simula_emxFree_char_T(&switch_expression);
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
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = -1.0;
  return b_obj;
}

static n_robotics_manip_internal_R_a_T *RigidBody_RigidBod_alw5typh44u2
  (n_robotics_manip_internal_R_a_T *obj)
{
  n_robotics_manip_internal_R_a_T *b_obj;
  emxArray_char_T_kinematic_sim_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  static const char_T tmp[10] = { 'e', 'd', 'o', '_', 'l', 'i', 'n', 'k', '_',
    '3' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_2[16] = { 1.0, 2.0682310711021444E-13,
    2.0682310711021444E-13, 0.0, 2.0682310711021444E-13, -1.0, -0.0, 0.0,
    2.0682310711021444E-13, 4.2775797634723234E-26, -1.0, 0.0, 0.0, 0.2105, 0.0,
    1.0 };

  static const real_T tmp_3[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  kinema_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 3.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  kinema_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  kinematic_simula_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinema_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  kinematic_simula_emxFree_char_T(&switch_expression);
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
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static n_robotics_manip_internal_R_a_T *RigidBody_RigidBo_alw5typh44u2s
  (n_robotics_manip_internal_R_a_T *obj)
{
  n_robotics_manip_internal_R_a_T *b_obj;
  emxArray_char_T_kinematic_sim_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  static const char_T tmp[10] = { 'e', 'd', 'o', '_', 'l', 'i', 'n', 'k', '_',
    '4' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_2[16] = { 1.0, 0.0, -0.0, 0.0, 0.0,
    4.8965888601467475E-12, 1.0, 0.0, 0.0, -1.0, 4.8965888601467475E-12, 0.0,
    0.0, -0.268, 0.0, 1.0 };

  static const real_T tmp_3[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  kinema_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 4.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  kinema_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  kinematic_simula_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  kinema_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  kinematic_simula_emxFree_char_T(&switch_expression);
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
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static p_robotics_manip_internal_R_a_T *k_RigidBodyTree_RigidBodyTree_a
  (p_robotics_manip_internal_R_a_T *obj, n_robotics_manip_internal_R_a_T *iobj_0,
   n_robotics_manip_internal_R_a_T *iobj_1, n_robotics_manip_internal_R_a_T
   *iobj_2, n_robotics_manip_internal_R_a_T *iobj_3,
   n_robotics_manip_internal_R_a_T *iobj_4, n_robotics_manip_internal_R_a_T
   *iobj_5, n_robotics_manip_internal_R_a_T *iobj_6,
   n_robotics_manip_internal_R_a_T *iobj_7)
{
  p_robotics_manip_internal_R_a_T *b_obj;
  o_robotics_manip_internal_R_a_T *obj_0;
  emxArray_char_T_kinematic_sim_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  static const char_T tmp[10] = { 'e', 'd', 'o', '_', 'l', 'i', 'n', 'k', '_',
    '5' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_2[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_3[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const char_T tmp_4[10] = { 'e', 'd', 'o', '_', 'l', 'i', 'n', 'k', '_',
    '6' };

  static const real_T tmp_5[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.1745, 1.0 };

  static const char_T tmp_6[11] = { 'e', 'd', 'o', '_', 'l', 'i', 'n', 'k', '_',
    'e', 'e' };

  static const char_T tmp_7[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_8[5] = { 'w', 'o', 'r', 'l', 'd' };

  int32_T exitg1;
  b_obj = obj;
  obj->Bodies[0] = k_RigidBody_RigidBody_alw5typh4(iobj_0);
  obj->Bodies[1] = RigidBody_RigidBody_alw5typh44(iobj_7);
  obj->Bodies[2] = RigidBody_RigidBody_alw5typh44u(iobj_1);
  obj->Bodies[3] = RigidBody_RigidBod_alw5typh44u2(iobj_2);
  obj->Bodies[4] = RigidBody_RigidBo_alw5typh44u2s(iobj_3);
  b_kstr = iobj_4->NameInternal->size[0] * iobj_4->NameInternal->size[1];
  iobj_4->NameInternal->size[0] = 1;
  iobj_4->NameInternal->size[1] = 10;
  kinema_emxEnsureCapacity_char_T(iobj_4->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    iobj_4->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  iobj_4->ParentIndex = 5.0;
  b_kstr = iobj_4->JointInternal.Type->size[0] * iobj_4->
    JointInternal.Type->size[1];
  iobj_4->JointInternal.Type->size[0] = 1;
  iobj_4->JointInternal.Type->size[1] = 8;
  kinema_emxEnsureCapacity_char_T(iobj_4->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    iobj_4->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  kinematic_simula_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_4->JointInternal.Type->size[1];
  kinema_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
    iobj_4->JointInternal.JointToParentTransform[b_kstr] = tmp_2[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_4->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  iobj_4->JointInternal.JointAxisInternal[0] = 0.0;
  iobj_4->JointInternal.JointAxisInternal[1] = 1.0;
  iobj_4->JointInternal.JointAxisInternal[2] = 0.0;
  obj->Bodies[5] = iobj_4;
  b_kstr = iobj_5->NameInternal->size[0] * iobj_5->NameInternal->size[1];
  iobj_5->NameInternal->size[0] = 1;
  iobj_5->NameInternal->size[1] = 10;
  kinema_emxEnsureCapacity_char_T(iobj_5->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    iobj_5->NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  iobj_5->ParentIndex = 6.0;
  b_kstr = iobj_5->JointInternal.Type->size[0] * iobj_5->
    JointInternal.Type->size[1];
  iobj_5->JointInternal.Type->size[0] = 1;
  iobj_5->JointInternal.Type->size[1] = 8;
  kinema_emxEnsureCapacity_char_T(iobj_5->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    iobj_5->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_5->JointInternal.Type->size[1];
  kinema_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_5->JointInternal.Type->size[0] * iobj_5->
    JointInternal.Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_5->JointInternal.Type->data[b_kstr];
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
    iobj_5->JointInternal.PositionNumber = 1.0;
    iobj_5->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_5->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_5->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    iobj_5->JointInternal.PositionNumber = 1.0;
    iobj_5->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_5->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_5->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    iobj_5->JointInternal.PositionNumber = 0.0;
    iobj_5->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_5->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_5->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_5->JointInternal.JointToParentTransform[b_kstr] = tmp_5[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_5->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  iobj_5->JointInternal.JointAxisInternal[0] = 0.0;
  iobj_5->JointInternal.JointAxisInternal[1] = 0.0;
  iobj_5->JointInternal.JointAxisInternal[2] = 1.0;
  obj->Bodies[6] = iobj_5;
  b_kstr = iobj_6->NameInternal->size[0] * iobj_6->NameInternal->size[1];
  iobj_6->NameInternal->size[0] = 1;
  iobj_6->NameInternal->size[1] = 11;
  kinema_emxEnsureCapacity_char_T(iobj_6->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 11; b_kstr++) {
    iobj_6->NameInternal->data[b_kstr] = tmp_6[b_kstr];
  }

  iobj_6->ParentIndex = 7.0;
  b_kstr = iobj_6->JointInternal.Type->size[0] * iobj_6->
    JointInternal.Type->size[1];
  iobj_6->JointInternal.Type->size[0] = 1;
  iobj_6->JointInternal.Type->size[1] = 5;
  kinema_emxEnsureCapacity_char_T(iobj_6->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_6->JointInternal.Type->data[b_kstr] = tmp_7[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_6->JointInternal.Type->size[1];
  kinema_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_6->JointInternal.Type->size[0] * iobj_6->
    JointInternal.Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_6->JointInternal.Type->data[b_kstr];
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
    iobj_6->JointInternal.PositionNumber = 1.0;
    iobj_6->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_6->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_6->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    iobj_6->JointInternal.PositionNumber = 1.0;
    iobj_6->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_6->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_6->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    iobj_6->JointInternal.PositionNumber = 0.0;
    iobj_6->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_6->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_6->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_6->JointInternal.JointToParentTransform[b_kstr] = tmp_2[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_6->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  iobj_6->JointInternal.JointAxisInternal[0] = 0.0;
  iobj_6->JointInternal.JointAxisInternal[1] = 0.0;
  iobj_6->JointInternal.JointAxisInternal[2] = 0.0;
  obj->Bodies[7] = iobj_6;
  obj->NumBodies = 8.0;
  obj->PositionNumber = 6.0;
  obj_0 = &obj->Base;
  b_kstr = obj->Base.NameInternal->size[0] * obj->Base.NameInternal->size[1];
  obj->Base.NameInternal->size[0] = 1;
  obj->Base.NameInternal->size[1] = 5;
  kinema_emxEnsureCapacity_char_T(obj->Base.NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->Base.NameInternal->data[b_kstr] = tmp_8[b_kstr];
  }

  b_kstr = obj->Base.JointInternal.Type->size[0] * obj->
    Base.JointInternal.Type->size[1];
  obj->Base.JointInternal.Type->size[0] = 1;
  obj->Base.JointInternal.Type->size[1] = 5;
  kinema_emxEnsureCapacity_char_T(obj->Base.JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj_0->JointInternal.Type->data[b_kstr] = tmp_7[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->Base.JointInternal.Type->size[1];
  kinema_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  kinematic_simula_emxFree_char_T(&switch_expression);
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
void kinematic_simulation_step(void)
{
  emxArray_real_T_kinematic_sim_T *b;
  robotics_slmanip_internal_b_a_T *obj;
  p_robotics_manip_internal_R_a_T *obj_0;
  emxArray_f_cell_wrap_kinemati_T *Ttree;
  emxArray_char_T_kinematic_sim_T *bname;
  n_robotics_manip_internal_R_a_T *obj_1;
  static const char_T tmp[11] = { 'e', 'd', 'o', '_', 'l', 'i', 'n', 'k', '_',
    'e', 'e' };

  static const char_T tmp_0[5] = { 'w', 'o', 'r', 'l', 'd' };

  static const char_T h[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n', 't', '_',
    '1' };

  static const char_T i[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n', 't', '_',
    '2' };

  static const char_T j[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n', 't', '_',
    '3' };

  static const char_T k[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n', 't', '_',
    '4' };

  static const char_T l[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n', 't', '_',
    '5' };

  static const char_T m[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n', 't', '_',
    '6' };

  int32_T exitg1;
  boolean_T exitg2;
  if (rtmIsMajorTimeStep(kinematic_simulation_M)) {
    // set solver stop time
    rtsiSetSolverStopTime(&kinematic_simulation_M->solverInfo,
                          ((kinematic_simulation_M->Timing.clockTick0+1)*
      kinematic_simulation_M->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if (rtmIsMinorTimeStep(kinematic_simulation_M)) {
    kinematic_simulation_M->Timing.t[0] = rtsiGetT
      (&kinematic_simulation_M->solverInfo);
  }

  // MATLABSystem: '<S12>/Get Parameter'
  ParamGet_kinematic_simulation_61.get_parameter(&kinematic_simulation_B.K12);

  // MATLABSystem: '<S12>/Get Parameter2'
  ParamGet_kinematic_simulation_85.get_parameter(&kinematic_simulation_B.bid1);

  // MATLABSystem: '<S12>/Get Parameter5'
  ParamGet_kinematic_simulation_88.get_parameter(&kinematic_simulation_B.K14);

  // MATLABSystem: '<S12>/Get Parameter4'
  ParamGet_kinematic_simulation_87.get_parameter(&kinematic_simulation_B.K23);

  // MATLABSystem: '<S12>/Get Parameter3'
  ParamGet_kinematic_simulation_86.get_parameter(&kinematic_simulation_B.K24);

  // MATLABSystem: '<S12>/Get Parameter1'
  ParamGet_kinematic_simulation_65.get_parameter(&kinematic_simulation_B.K34);

  // Integrator: '<Root>/Integrator' incorporates:
  //   MATLABSystem: '<S12>/Get Parameter'
  //   MATLABSystem: '<S12>/Get Parameter1'
  //   MATLABSystem: '<S12>/Get Parameter2'
  //   MATLABSystem: '<S12>/Get Parameter3'
  //   MATLABSystem: '<S12>/Get Parameter4'
  //   MATLABSystem: '<S12>/Get Parameter5'

  if (kinematic_simulation_DW.Integrator_IWORK != 0) {
    kinematic_simulation_X.Integrator_CSTATE[0] = kinematic_simulation_B.K12;
    kinematic_simulation_X.Integrator_CSTATE[1] = kinematic_simulation_B.bid1;
    kinematic_simulation_X.Integrator_CSTATE[2] = kinematic_simulation_B.K14;
    kinematic_simulation_X.Integrator_CSTATE[3] = kinematic_simulation_B.K23;
    kinematic_simulation_X.Integrator_CSTATE[4] = kinematic_simulation_B.K24;
    kinematic_simulation_X.Integrator_CSTATE[5] = kinematic_simulation_B.K34;
  }

  kinematic_simula_emxInit_real_T(&b, 2);
  kinematic_s_emxInit_f_cell_wrap(&Ttree, 2);
  kinematic_simula_emxInit_char_T(&bname, 2);

  // MATLABSystem: '<S16>/MATLAB System' incorporates:
  //   Integrator: '<Root>/Integrator'

  RigidBodyTree_geometricJacobian(&kinematic_simulation_DW.obj.TreeInternal,
    kinematic_simulation_X.Integrator_CSTATE, b);

  // MATLABSystem: '<S17>/MATLAB System' incorporates:
  //   Integrator: '<Root>/Integrator'

  obj = &kinematic_simulation_DW.obj_b;
  obj_0 = &kinematic_simulation_DW.obj_b.TreeInternal;
  RigidBodyTree_forwardKinemati_a(&obj->TreeInternal,
    kinematic_simulation_X.Integrator_CSTATE, Ttree);
  kinematic_simulation_B.bid1 = -1.0;
  kinematic_simulation_B.b_kstr = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj_0->Base.NameInternal->size[1];
  kinema_emxEnsureCapacity_char_T(bname, kinematic_simulation_B.b_kstr);
  kinematic_simulation_B.loop_ub = obj_0->Base.NameInternal->size[0] *
    obj_0->Base.NameInternal->size[1] - 1;
  for (kinematic_simulation_B.b_kstr = 0; kinematic_simulation_B.b_kstr <=
       kinematic_simulation_B.loop_ub; kinematic_simulation_B.b_kstr++) {
    bname->data[kinematic_simulation_B.b_kstr] = obj_0->Base.NameInternal->
      data[kinematic_simulation_B.b_kstr];
  }

  for (kinematic_simulation_B.b_kstr = 0; kinematic_simulation_B.b_kstr < 11;
       kinematic_simulation_B.b_kstr++) {
    kinematic_simulation_B.b_b[kinematic_simulation_B.b_kstr] =
      tmp[kinematic_simulation_B.b_kstr];
  }

  kinematic_simulation_B.b_bool = false;
  if (bname->size[1] == 11) {
    kinematic_simulation_B.b_kstr = 1;
    do {
      exitg1 = 0;
      if (kinematic_simulation_B.b_kstr - 1 < 11) {
        kinematic_simulation_B.loop_ub = kinematic_simulation_B.b_kstr - 1;
        if (bname->data[kinematic_simulation_B.loop_ub] !=
            kinematic_simulation_B.b_b[kinematic_simulation_B.loop_ub]) {
          exitg1 = 1;
        } else {
          kinematic_simulation_B.b_kstr++;
        }
      } else {
        kinematic_simulation_B.b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (kinematic_simulation_B.b_bool) {
    kinematic_simulation_B.bid1 = 0.0;
  } else {
    kinematic_simulation_B.K12 = obj->TreeInternal.NumBodies;
    kinematic_simulation_B.b_i_c = 0;
    exitg2 = false;
    while ((!exitg2) && (kinematic_simulation_B.b_i_c <= static_cast<int32_T>
                         (kinematic_simulation_B.K12) - 1)) {
      obj_1 = obj_0->Bodies[kinematic_simulation_B.b_i_c];
      kinematic_simulation_B.b_kstr = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = obj_1->NameInternal->size[1];
      kinema_emxEnsureCapacity_char_T(bname, kinematic_simulation_B.b_kstr);
      kinematic_simulation_B.loop_ub = obj_1->NameInternal->size[0] *
        obj_1->NameInternal->size[1] - 1;
      for (kinematic_simulation_B.b_kstr = 0; kinematic_simulation_B.b_kstr <=
           kinematic_simulation_B.loop_ub; kinematic_simulation_B.b_kstr++) {
        bname->data[kinematic_simulation_B.b_kstr] = obj_1->NameInternal->
          data[kinematic_simulation_B.b_kstr];
      }

      for (kinematic_simulation_B.b_kstr = 0; kinematic_simulation_B.b_kstr < 11;
           kinematic_simulation_B.b_kstr++) {
        kinematic_simulation_B.b_b[kinematic_simulation_B.b_kstr] =
          tmp[kinematic_simulation_B.b_kstr];
      }

      kinematic_simulation_B.b_bool = false;
      if (bname->size[1] == 11) {
        kinematic_simulation_B.b_kstr = 1;
        do {
          exitg1 = 0;
          if (kinematic_simulation_B.b_kstr - 1 < 11) {
            kinematic_simulation_B.loop_ub = kinematic_simulation_B.b_kstr - 1;
            if (bname->data[kinematic_simulation_B.loop_ub] !=
                kinematic_simulation_B.b_b[kinematic_simulation_B.loop_ub]) {
              exitg1 = 1;
            } else {
              kinematic_simulation_B.b_kstr++;
            }
          } else {
            kinematic_simulation_B.b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (kinematic_simulation_B.b_bool) {
        kinematic_simulation_B.bid1 = static_cast<real_T>
          (kinematic_simulation_B.b_i_c) + 1.0;
        exitg2 = true;
      } else {
        kinematic_simulation_B.b_i_c++;
      }
    }
  }

  if (kinematic_simulation_B.bid1 == 0.0) {
    memset(&kinematic_simulation_B.T1[0], 0, sizeof(real_T) << 4U);
    kinematic_simulation_B.T1[0] = 1.0;
    kinematic_simulation_B.T1[5] = 1.0;
    kinematic_simulation_B.T1[10] = 1.0;
    kinematic_simulation_B.T1[15] = 1.0;
  } else {
    for (kinematic_simulation_B.b_kstr = 0; kinematic_simulation_B.b_kstr < 16;
         kinematic_simulation_B.b_kstr++) {
      kinematic_simulation_B.T1[kinematic_simulation_B.b_kstr] = Ttree->data[
        static_cast<int32_T>(kinematic_simulation_B.bid1) - 1]
        .f1[kinematic_simulation_B.b_kstr];
    }
  }

  kinematic_simulation_B.bid1 = -1.0;
  kinematic_simulation_B.b_kstr = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj_0->Base.NameInternal->size[1];
  kinema_emxEnsureCapacity_char_T(bname, kinematic_simulation_B.b_kstr);
  kinematic_simulation_B.loop_ub = obj_0->Base.NameInternal->size[0] *
    obj_0->Base.NameInternal->size[1] - 1;
  for (kinematic_simulation_B.b_kstr = 0; kinematic_simulation_B.b_kstr <=
       kinematic_simulation_B.loop_ub; kinematic_simulation_B.b_kstr++) {
    bname->data[kinematic_simulation_B.b_kstr] = obj_0->Base.NameInternal->
      data[kinematic_simulation_B.b_kstr];
  }

  for (kinematic_simulation_B.b_kstr = 0; kinematic_simulation_B.b_kstr < 5;
       kinematic_simulation_B.b_kstr++) {
    kinematic_simulation_B.b_f[kinematic_simulation_B.b_kstr] =
      tmp_0[kinematic_simulation_B.b_kstr];
  }

  kinematic_simulation_B.b_bool = false;
  if (bname->size[1] == 5) {
    kinematic_simulation_B.b_kstr = 1;
    do {
      exitg1 = 0;
      if (kinematic_simulation_B.b_kstr - 1 < 5) {
        kinematic_simulation_B.loop_ub = kinematic_simulation_B.b_kstr - 1;
        if (bname->data[kinematic_simulation_B.loop_ub] !=
            kinematic_simulation_B.b_f[kinematic_simulation_B.loop_ub]) {
          exitg1 = 1;
        } else {
          kinematic_simulation_B.b_kstr++;
        }
      } else {
        kinematic_simulation_B.b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (kinematic_simulation_B.b_bool) {
    kinematic_simulation_B.bid1 = 0.0;
  } else {
    kinematic_simulation_B.K12 = obj->TreeInternal.NumBodies;
    kinematic_simulation_B.b_i_c = 0;
    exitg2 = false;
    while ((!exitg2) && (kinematic_simulation_B.b_i_c <= static_cast<int32_T>
                         (kinematic_simulation_B.K12) - 1)) {
      obj_1 = obj_0->Bodies[kinematic_simulation_B.b_i_c];
      kinematic_simulation_B.b_kstr = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = obj_1->NameInternal->size[1];
      kinema_emxEnsureCapacity_char_T(bname, kinematic_simulation_B.b_kstr);
      kinematic_simulation_B.loop_ub = obj_1->NameInternal->size[0] *
        obj_1->NameInternal->size[1] - 1;
      for (kinematic_simulation_B.b_kstr = 0; kinematic_simulation_B.b_kstr <=
           kinematic_simulation_B.loop_ub; kinematic_simulation_B.b_kstr++) {
        bname->data[kinematic_simulation_B.b_kstr] = obj_1->NameInternal->
          data[kinematic_simulation_B.b_kstr];
      }

      for (kinematic_simulation_B.b_kstr = 0; kinematic_simulation_B.b_kstr < 5;
           kinematic_simulation_B.b_kstr++) {
        kinematic_simulation_B.b_f[kinematic_simulation_B.b_kstr] =
          tmp_0[kinematic_simulation_B.b_kstr];
      }

      kinematic_simulation_B.b_bool = false;
      if (bname->size[1] == 5) {
        kinematic_simulation_B.b_kstr = 1;
        do {
          exitg1 = 0;
          if (kinematic_simulation_B.b_kstr - 1 < 5) {
            kinematic_simulation_B.loop_ub = kinematic_simulation_B.b_kstr - 1;
            if (bname->data[kinematic_simulation_B.loop_ub] !=
                kinematic_simulation_B.b_f[kinematic_simulation_B.loop_ub]) {
              exitg1 = 1;
            } else {
              kinematic_simulation_B.b_kstr++;
            }
          } else {
            kinematic_simulation_B.b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (kinematic_simulation_B.b_bool) {
        kinematic_simulation_B.bid1 = static_cast<real_T>
          (kinematic_simulation_B.b_i_c) + 1.0;
        exitg2 = true;
      } else {
        kinematic_simulation_B.b_i_c++;
      }
    }
  }

  kinematic_simula_emxFree_char_T(&bname);

  // MATLABSystem: '<S17>/MATLAB System'
  if (kinematic_simulation_B.bid1 == 0.0) {
    memset(&kinematic_simulation_B.T2[0], 0, sizeof(real_T) << 4U);
    kinematic_simulation_B.T2[0] = 1.0;
    kinematic_simulation_B.T2[5] = 1.0;
    kinematic_simulation_B.T2[10] = 1.0;
    kinematic_simulation_B.T2[15] = 1.0;
  } else {
    for (kinematic_simulation_B.b_kstr = 0; kinematic_simulation_B.b_kstr < 16;
         kinematic_simulation_B.b_kstr++) {
      kinematic_simulation_B.T2[kinematic_simulation_B.b_kstr] = Ttree->data[
        static_cast<int32_T>(kinematic_simulation_B.bid1) - 1]
        .f1[kinematic_simulation_B.b_kstr];
    }
  }

  kinematic_s_emxFree_f_cell_wrap(&Ttree);

  // MATLABSystem: '<S17>/MATLAB System'
  for (kinematic_simulation_B.b_kstr = 0; kinematic_simulation_B.b_kstr < 3;
       kinematic_simulation_B.b_kstr++) {
    kinematic_simulation_B.R_g[3 * kinematic_simulation_B.b_kstr] =
      kinematic_simulation_B.T2[kinematic_simulation_B.b_kstr];
    kinematic_simulation_B.R_g[3 * kinematic_simulation_B.b_kstr + 1] =
      kinematic_simulation_B.T2[kinematic_simulation_B.b_kstr + 4];
    kinematic_simulation_B.R_g[3 * kinematic_simulation_B.b_kstr + 2] =
      kinematic_simulation_B.T2[kinematic_simulation_B.b_kstr + 8];
  }

  for (kinematic_simulation_B.b_kstr = 0; kinematic_simulation_B.b_kstr < 9;
       kinematic_simulation_B.b_kstr++) {
    kinematic_simulation_B.R_g1[kinematic_simulation_B.b_kstr] =
      -kinematic_simulation_B.R_g[kinematic_simulation_B.b_kstr];
  }

  for (kinematic_simulation_B.b_kstr = 0; kinematic_simulation_B.b_kstr < 3;
       kinematic_simulation_B.b_kstr++) {
    kinematic_simulation_B.b_i_c = kinematic_simulation_B.b_kstr << 2;
    kinematic_simulation_B.R[kinematic_simulation_B.b_i_c] =
      kinematic_simulation_B.R_g[3 * kinematic_simulation_B.b_kstr];
    kinematic_simulation_B.R[kinematic_simulation_B.b_i_c + 1] =
      kinematic_simulation_B.R_g[3 * kinematic_simulation_B.b_kstr + 1];
    kinematic_simulation_B.R[kinematic_simulation_B.b_i_c + 2] =
      kinematic_simulation_B.R_g[3 * kinematic_simulation_B.b_kstr + 2];
    kinematic_simulation_B.R[kinematic_simulation_B.b_kstr + 12] =
      kinematic_simulation_B.R_g1[kinematic_simulation_B.b_kstr + 6] *
      kinematic_simulation_B.T2[14] +
      (kinematic_simulation_B.R_g1[kinematic_simulation_B.b_kstr + 3] *
       kinematic_simulation_B.T2[13] +
       kinematic_simulation_B.R_g1[kinematic_simulation_B.b_kstr] *
       kinematic_simulation_B.T2[12]);
  }

  kinematic_simulation_B.R[3] = 0.0;
  kinematic_simulation_B.R[7] = 0.0;
  kinematic_simulation_B.R[11] = 0.0;
  kinematic_simulation_B.R[15] = 1.0;
  for (kinematic_simulation_B.b_kstr = 0; kinematic_simulation_B.b_kstr < 4;
       kinematic_simulation_B.b_kstr++) {
    for (kinematic_simulation_B.b_i_c = 0; kinematic_simulation_B.b_i_c < 4;
         kinematic_simulation_B.b_i_c++) {
      kinematic_simulation_B.loop_ub = kinematic_simulation_B.b_kstr << 2;
      kinematic_simulation_B.rtb_MATLABSystem_tmp = kinematic_simulation_B.b_i_c
        + kinematic_simulation_B.loop_ub;
      kinematic_simulation_B.T2[kinematic_simulation_B.rtb_MATLABSystem_tmp] =
        0.0;
      kinematic_simulation_B.T2[kinematic_simulation_B.rtb_MATLABSystem_tmp] +=
        kinematic_simulation_B.T1[kinematic_simulation_B.loop_ub] *
        kinematic_simulation_B.R[kinematic_simulation_B.b_i_c];
      kinematic_simulation_B.T2[kinematic_simulation_B.rtb_MATLABSystem_tmp] +=
        kinematic_simulation_B.T1[kinematic_simulation_B.loop_ub + 1] *
        kinematic_simulation_B.R[kinematic_simulation_B.b_i_c + 4];
      kinematic_simulation_B.T2[kinematic_simulation_B.rtb_MATLABSystem_tmp] +=
        kinematic_simulation_B.T1[kinematic_simulation_B.loop_ub + 2] *
        kinematic_simulation_B.R[kinematic_simulation_B.b_i_c + 8];
      kinematic_simulation_B.T2[kinematic_simulation_B.rtb_MATLABSystem_tmp] +=
        kinematic_simulation_B.T1[kinematic_simulation_B.loop_ub + 3] *
        kinematic_simulation_B.R[kinematic_simulation_B.b_i_c + 12];
    }
  }

  if (rtmIsMajorTimeStep(kinematic_simulation_M)) {
    // Outputs for Atomic SubSystem: '<Root>/Subscribe'
    // MATLABSystem: '<S11>/SourceBlock' incorporates:
    //   Inport: '<S19>/In1'

    kinematic_simul_SystemCore_step(&kinematic_simulation_B.b_bool,
      kinematic_simulation_B.b_varargout_2_Positions,
      &kinematic_simulation_B.b_varargout_2_Positions_SL_Info,
      &kinematic_simulation_B.b_varargout_2_Positions_SL_In_o,
      kinematic_simulation_B.b_varargout_2_Velocities,
      &kinematic_simulation_B.b_varargout_2_Velocities_SL_Inf,
      &kinematic_simulation_B.b_varargout_2_Velocities_SL_I_l,
      kinematic_simulation_B.b_varargout_2_Accelerations,
      &kinematic_simulation_B.b_varargout_2_Accelerations_SL_,
      &kinematic_simulation_B.b_varargout_2_Accelerations_S_p,
      kinematic_simulation_B.b_varargout_2_Effort,
      &kinematic_simulation_B.b_varargout_2_Effort_SL_Info_Cu,
      &kinematic_simulation_B.b_varargout_2_Effort_SL_Info_Re,
      &kinematic_simulation_B.K12, &kinematic_simulation_B.bid1);

    // Outputs for Enabled SubSystem: '<S11>/Enabled Subsystem' incorporates:
    //   EnablePort: '<S19>/Enable'

    if (kinematic_simulation_B.b_bool) {
      kinematic_simulation_B.In1.Positions_SL_Info.CurrentLength =
        kinematic_simulation_B.b_varargout_2_Positions_SL_Info;
      kinematic_simulation_B.In1.Positions_SL_Info.ReceivedLength =
        kinematic_simulation_B.b_varargout_2_Positions_SL_In_o;
      kinematic_simulation_B.In1.Velocities_SL_Info.CurrentLength =
        kinematic_simulation_B.b_varargout_2_Velocities_SL_Inf;
      kinematic_simulation_B.In1.Velocities_SL_Info.ReceivedLength =
        kinematic_simulation_B.b_varargout_2_Velocities_SL_I_l;
      kinematic_simulation_B.In1.Accelerations_SL_Info.CurrentLength =
        kinematic_simulation_B.b_varargout_2_Accelerations_SL_;
      kinematic_simulation_B.In1.Accelerations_SL_Info.ReceivedLength =
        kinematic_simulation_B.b_varargout_2_Accelerations_S_p;
      memcpy(&kinematic_simulation_B.In1.Positions[0],
             &kinematic_simulation_B.b_varargout_2_Positions[0], sizeof(real_T) <<
             7U);
      memcpy(&kinematic_simulation_B.In1.Velocities[0],
             &kinematic_simulation_B.b_varargout_2_Velocities[0], sizeof(real_T)
             << 7U);
      memcpy(&kinematic_simulation_B.In1.Accelerations[0],
             &kinematic_simulation_B.b_varargout_2_Accelerations[0], sizeof
             (real_T) << 7U);
      memcpy(&kinematic_simulation_B.In1.Effort[0],
             &kinematic_simulation_B.b_varargout_2_Effort[0], sizeof(real_T) <<
             7U);
      kinematic_simulation_B.In1.Effort_SL_Info.CurrentLength =
        kinematic_simulation_B.b_varargout_2_Effort_SL_Info_Cu;
      kinematic_simulation_B.In1.Effort_SL_Info.ReceivedLength =
        kinematic_simulation_B.b_varargout_2_Effort_SL_Info_Re;
      kinematic_simulation_B.In1.TimeFromStart.Sec = kinematic_simulation_B.K12;
      kinematic_simulation_B.In1.TimeFromStart.Nsec =
        kinematic_simulation_B.bid1;
    }

    // End of MATLABSystem: '<S11>/SourceBlock'
    // End of Outputs for SubSystem: '<S11>/Enabled Subsystem'
    // End of Outputs for SubSystem: '<Root>/Subscribe'
  }

  // Product: '<S4>/MatrixMultiply' incorporates:
  //   MATLABSystem: '<S16>/MATLAB System'

  for (kinematic_simulation_B.b_kstr = 0; kinematic_simulation_B.b_kstr < 6;
       kinematic_simulation_B.b_kstr++) {
    kinematic_simulation_B.MatrixMultiply[kinematic_simulation_B.b_kstr] = 0.0;
    for (kinematic_simulation_B.b_i_c = 0; kinematic_simulation_B.b_i_c < 6;
         kinematic_simulation_B.b_i_c++) {
      kinematic_simulation_B.K12 = b->data[6 * kinematic_simulation_B.b_i_c +
        kinematic_simulation_B.b_kstr] *
        kinematic_simulation_B.In1.Velocities[kinematic_simulation_B.b_i_c] +
        kinematic_simulation_B.MatrixMultiply[kinematic_simulation_B.b_kstr];
      kinematic_simulation_B.MatrixMultiply[kinematic_simulation_B.b_kstr] =
        kinematic_simulation_B.K12;
    }
  }

  // End of Product: '<S4>/MatrixMultiply'
  kinematic_simula_emxFree_real_T(&b);

  // MATLAB Function: '<S4>/MATLAB Function'
  kinematic_simulation_B.K12 = kinematic_simulation_B.T2[4] +
    kinematic_simulation_B.T2[1];
  kinematic_simulation_B.bid1 = kinematic_simulation_B.T2[8] +
    kinematic_simulation_B.T2[2];
  kinematic_simulation_B.K14 = kinematic_simulation_B.T2[6] -
    kinematic_simulation_B.T2[9];
  kinematic_simulation_B.K23 = kinematic_simulation_B.T2[9] +
    kinematic_simulation_B.T2[6];
  kinematic_simulation_B.K24 = kinematic_simulation_B.T2[8] -
    kinematic_simulation_B.T2[2];
  kinematic_simulation_B.K34 = kinematic_simulation_B.T2[1] -
    kinematic_simulation_B.T2[4];
  kinematic_simulation_B.T1[0] = ((kinematic_simulation_B.T2[0] -
    kinematic_simulation_B.T2[5]) - kinematic_simulation_B.T2[10]) / 3.0;
  kinematic_simulation_B.T1[4] = kinematic_simulation_B.K12 / 3.0;
  kinematic_simulation_B.T1[8] = kinematic_simulation_B.bid1 / 3.0;
  kinematic_simulation_B.T1[12] = kinematic_simulation_B.K14 / 3.0;
  kinematic_simulation_B.T1[1] = kinematic_simulation_B.K12 / 3.0;
  kinematic_simulation_B.T1[5] = ((kinematic_simulation_B.T2[5] -
    kinematic_simulation_B.T2[0]) - kinematic_simulation_B.T2[10]) / 3.0;
  kinematic_simulation_B.T1[9] = kinematic_simulation_B.K23 / 3.0;
  kinematic_simulation_B.T1[13] = kinematic_simulation_B.K24 / 3.0;
  kinematic_simulation_B.T1[2] = kinematic_simulation_B.bid1 / 3.0;
  kinematic_simulation_B.T1[6] = kinematic_simulation_B.K23 / 3.0;
  kinematic_simulation_B.T1[10] = ((kinematic_simulation_B.T2[10] -
    kinematic_simulation_B.T2[0]) - kinematic_simulation_B.T2[5]) / 3.0;
  kinematic_simulation_B.T1[14] = kinematic_simulation_B.K34 / 3.0;
  kinematic_simulation_B.T1[3] = kinematic_simulation_B.K14 / 3.0;
  kinematic_simulation_B.T1[7] = kinematic_simulation_B.K24 / 3.0;
  kinematic_simulation_B.T1[11] = kinematic_simulation_B.K34 / 3.0;
  kinematic_simulation_B.T1[15] = ((kinematic_simulation_B.T2[0] +
    kinematic_simulation_B.T2[5]) + kinematic_simulation_B.T2[10]) / 3.0;
  kinematic_simulation_eig(kinematic_simulation_B.T1,
    kinematic_simulation_B.eigVec, kinematic_simulation_B.eigVal);
  kinematic_simulation_B.cartOrn[0] = kinematic_simulation_B.eigVal[0].re;
  kinematic_simulation_B.cartOrn[1] = kinematic_simulation_B.eigVal[1].re;
  kinematic_simulation_B.cartOrn[2] = kinematic_simulation_B.eigVal[2].re;
  kinematic_simulation_B.cartOrn[3] = kinematic_simulation_B.eigVal[3].re;
  if (!rtIsNaN(kinematic_simulation_B.eigVal[0].re)) {
    kinematic_simulation_B.b_kstr = 1;
  } else {
    kinematic_simulation_B.b_kstr = 0;
    kinematic_simulation_B.b_i_c = 2;
    exitg2 = false;
    while ((!exitg2) && (kinematic_simulation_B.b_i_c < 5)) {
      if (!rtIsNaN(kinematic_simulation_B.cartOrn[kinematic_simulation_B.b_i_c -
                   1])) {
        kinematic_simulation_B.b_kstr = kinematic_simulation_B.b_i_c;
        exitg2 = true;
      } else {
        kinematic_simulation_B.b_i_c++;
      }
    }
  }

  if (kinematic_simulation_B.b_kstr != 0) {
    kinematic_simulation_B.K12 =
      kinematic_simulation_B.cartOrn[kinematic_simulation_B.b_kstr - 1];
    kinematic_simulation_B.b_i_c = kinematic_simulation_B.b_kstr - 1;
    while (kinematic_simulation_B.b_kstr + 1 < 5) {
      if (kinematic_simulation_B.K12 <
          kinematic_simulation_B.cartOrn[kinematic_simulation_B.b_kstr]) {
        kinematic_simulation_B.K12 =
          kinematic_simulation_B.cartOrn[kinematic_simulation_B.b_kstr];
        kinematic_simulation_B.b_i_c = kinematic_simulation_B.b_kstr;
      }

      kinematic_simulation_B.b_kstr++;
    }

    kinematic_simulation_B.b_kstr = kinematic_simulation_B.b_i_c;
  }

  kinematic_simulation_B.b_kstr <<= 2;
  kinematic_simulation_B.b_i_c = kinematic_simulation_B.b_kstr + 3;
  kinematic_simulation_B.cartOrn[0] =
    kinematic_simulation_B.eigVec[kinematic_simulation_B.b_i_c].re;
  kinematic_simulation_B.cartOrn[1] =
    kinematic_simulation_B.eigVec[kinematic_simulation_B.b_kstr].re;
  kinematic_simulation_B.loop_ub = kinematic_simulation_B.b_kstr + 1;
  kinematic_simulation_B.cartOrn[2] =
    kinematic_simulation_B.eigVec[kinematic_simulation_B.loop_ub].re;
  kinematic_simulation_B.rtb_MATLABSystem_tmp = kinematic_simulation_B.b_kstr +
    2;
  kinematic_simulation_B.cartOrn[3] =
    kinematic_simulation_B.eigVec[kinematic_simulation_B.rtb_MATLABSystem_tmp].
    re;
  if (kinematic_simulation_B.eigVec[kinematic_simulation_B.b_i_c].re < 0.0) {
    kinematic_simulation_B.cartOrn[0] =
      -kinematic_simulation_B.eigVec[kinematic_simulation_B.b_i_c].re;
    kinematic_simulation_B.cartOrn[1] =
      -kinematic_simulation_B.eigVec[kinematic_simulation_B.b_kstr].re;
    kinematic_simulation_B.cartOrn[2] =
      -kinematic_simulation_B.eigVec[kinematic_simulation_B.loop_ub].re;
    kinematic_simulation_B.cartOrn[3] =
      -kinematic_simulation_B.eigVec[kinematic_simulation_B.rtb_MATLABSystem_tmp]
      .re;
  }

  // Clock: '<Root>/Clock1' incorporates:
  //   Clock: '<Root>/Clock'

  kinematic_simulation_B.K12 = kinematic_simulation_M->Timing.t[0];

  // MATLAB Function: '<S3>/MATLAB Function' incorporates:
  //   Clock: '<Root>/Clock1'

  if (kinematic_simulation_B.K12 < 0.0) {
    // MATLAB Function: '<Root>/Assign to JointState msg'
    kinematic_simulation_B.bid1 = ceil(kinematic_simulation_B.K12);
  } else {
    // MATLAB Function: '<Root>/Assign to JointState msg'
    kinematic_simulation_B.bid1 = floor(kinematic_simulation_B.K12);
  }

  kinematic_simulation_B.K14 = (kinematic_simulation_B.K12 -
    kinematic_simulation_B.bid1) * 1.0E+9;
  if (kinematic_simulation_B.K14 < 0.0) {
    // MATLAB Function: '<Root>/Assign to JointState msg'
    kinematic_simulation_B.K14 = ceil(kinematic_simulation_B.K14);
  } else {
    // MATLAB Function: '<Root>/Assign to JointState msg'
    kinematic_simulation_B.K14 = floor(kinematic_simulation_B.K14);
  }

  // BusAssignment: '<S3>/Bus Assignment' incorporates:
  //   Constant: '<S13>/Constant'
  //   MATLAB Function: '<S3>/MATLAB Function'
  //   MATLAB Function: '<S4>/MATLAB Function'

  kinematic_simulation_B.BusAssignment = kinematic_simulation_P.Constant_Value_p;
  kinematic_simulation_B.BusAssignment.Header.Stamp.Sec =
    kinematic_simulation_B.bid1;
  kinematic_simulation_B.BusAssignment.Header.Stamp.Nsec =
    kinematic_simulation_B.K14;
  kinematic_simulation_B.BusAssignment.Pose.Position.X =
    kinematic_simulation_B.T2[12];
  kinematic_simulation_B.BusAssignment.Pose.Position.Y =
    kinematic_simulation_B.T2[13];
  kinematic_simulation_B.BusAssignment.Pose.Position.Z =
    kinematic_simulation_B.T2[14];
  kinematic_simulation_B.BusAssignment.Pose.Orientation.X =
    kinematic_simulation_B.cartOrn[0];
  kinematic_simulation_B.BusAssignment.Pose.Orientation.Y =
    kinematic_simulation_B.cartOrn[1];
  kinematic_simulation_B.BusAssignment.Pose.Orientation.Z =
    kinematic_simulation_B.cartOrn[2];
  kinematic_simulation_B.BusAssignment.Pose.Orientation.W =
    kinematic_simulation_B.cartOrn[3];

  // Outputs for Atomic SubSystem: '<Root>/Publish2'
  // MATLABSystem: '<S9>/SinkBlock'
  Pub_kinematic_simulation_79.publish(&kinematic_simulation_B.BusAssignment);

  // End of Outputs for SubSystem: '<Root>/Publish2'

  // BusAssignment: '<S3>/Bus Assignment1' incorporates:
  //   Constant: '<S14>/Constant'
  //   MATLAB Function: '<S3>/MATLAB Function'
  //   MATLAB Function: '<S4>/MATLAB Function'

  kinematic_simulation_B.BusAssignment1 =
    kinematic_simulation_P.Constant_Value_m;
  kinematic_simulation_B.BusAssignment1.Header.Stamp.Sec =
    kinematic_simulation_B.bid1;
  kinematic_simulation_B.BusAssignment1.Header.Stamp.Nsec =
    kinematic_simulation_B.K14;
  kinematic_simulation_B.BusAssignment1.Twist.Linear.X =
    kinematic_simulation_B.MatrixMultiply[3];
  kinematic_simulation_B.BusAssignment1.Twist.Linear.Y =
    kinematic_simulation_B.MatrixMultiply[4];
  kinematic_simulation_B.BusAssignment1.Twist.Linear.Z =
    kinematic_simulation_B.MatrixMultiply[5];
  kinematic_simulation_B.BusAssignment1.Twist.Angular.X =
    kinematic_simulation_B.MatrixMultiply[0];
  kinematic_simulation_B.BusAssignment1.Twist.Angular.Y =
    kinematic_simulation_B.MatrixMultiply[1];
  kinematic_simulation_B.BusAssignment1.Twist.Angular.Z =
    kinematic_simulation_B.MatrixMultiply[2];

  // Outputs for Atomic SubSystem: '<Root>/Publish3'
  // MATLABSystem: '<S10>/SinkBlock'
  Pub_kinematic_simulation_101.publish(&kinematic_simulation_B.BusAssignment1);

  // End of Outputs for SubSystem: '<Root>/Publish3'

  // MATLAB Function: '<Root>/Assign to JointState msg' incorporates:
  //   Constant: '<S5>/Constant'
  //   Integrator: '<Root>/Integrator'

  kinematic_simulation_B.msg = kinematic_simulation_P.Constant_Value;
  kinematic_simulation_B.msg.Header.Stamp.Sec = kinematic_simulation_B.bid1;
  kinematic_simulation_B.msg.Header.Stamp.Nsec = kinematic_simulation_B.K14;
  kinematic_simulation_B.msg.Name_SL_Info.CurrentLength = 6U;
  kinematic_simulation_B.msg.Position_SL_Info.CurrentLength = 6U;
  kinematic_simulation_B.msg.Velocity_SL_Info.CurrentLength = 6U;
  kinematic_simulation_B.msg.Name[0].Data_SL_Info.CurrentLength = 11U;
  kinematic_simulation_B.msg.Position[0] =
    kinematic_simulation_X.Integrator_CSTATE[0];
  kinematic_simulation_B.msg.Velocity[0] =
    kinematic_simulation_B.In1.Velocities[0];
  kinematic_simulation_B.msg.Name[1].Data_SL_Info.CurrentLength = 11U;
  kinematic_simulation_B.msg.Position[1] =
    kinematic_simulation_X.Integrator_CSTATE[1];
  kinematic_simulation_B.msg.Velocity[1] =
    kinematic_simulation_B.In1.Velocities[1];
  kinematic_simulation_B.msg.Name[2].Data_SL_Info.CurrentLength = 11U;
  kinematic_simulation_B.msg.Position[2] =
    kinematic_simulation_X.Integrator_CSTATE[2];
  kinematic_simulation_B.msg.Velocity[2] =
    kinematic_simulation_B.In1.Velocities[2];
  kinematic_simulation_B.msg.Name[3].Data_SL_Info.CurrentLength = 11U;
  kinematic_simulation_B.msg.Position[3] =
    kinematic_simulation_X.Integrator_CSTATE[3];
  kinematic_simulation_B.msg.Velocity[3] =
    kinematic_simulation_B.In1.Velocities[3];
  kinematic_simulation_B.msg.Name[4].Data_SL_Info.CurrentLength = 11U;
  kinematic_simulation_B.msg.Position[4] =
    kinematic_simulation_X.Integrator_CSTATE[4];
  kinematic_simulation_B.msg.Velocity[4] =
    kinematic_simulation_B.In1.Velocities[4];
  for (kinematic_simulation_B.b_kstr = 0; kinematic_simulation_B.b_kstr < 11;
       kinematic_simulation_B.b_kstr++) {
    kinematic_simulation_B.b_o.f1[kinematic_simulation_B.b_kstr] =
      h[kinematic_simulation_B.b_kstr];
    kinematic_simulation_B.c.f1[kinematic_simulation_B.b_kstr] =
      i[kinematic_simulation_B.b_kstr];
    kinematic_simulation_B.d.f1[kinematic_simulation_B.b_kstr] =
      j[kinematic_simulation_B.b_kstr];
    kinematic_simulation_B.e.f1[kinematic_simulation_B.b_kstr] =
      k[kinematic_simulation_B.b_kstr];
    kinematic_simulation_B.f.f1[kinematic_simulation_B.b_kstr] =
      l[kinematic_simulation_B.b_kstr];
    kinematic_simulation_B.g.f1[kinematic_simulation_B.b_kstr] =
      m[kinematic_simulation_B.b_kstr];
    kinematic_simulation_B.msg.Name[0].Data[kinematic_simulation_B.b_kstr] =
      static_cast<uint8_T>
      (kinematic_simulation_B.b_o.f1[kinematic_simulation_B.b_kstr]);
    kinematic_simulation_B.msg.Name[1].Data[kinematic_simulation_B.b_kstr] =
      static_cast<uint8_T>
      (kinematic_simulation_B.c.f1[kinematic_simulation_B.b_kstr]);
    kinematic_simulation_B.msg.Name[2].Data[kinematic_simulation_B.b_kstr] =
      static_cast<uint8_T>
      (kinematic_simulation_B.d.f1[kinematic_simulation_B.b_kstr]);
    kinematic_simulation_B.msg.Name[3].Data[kinematic_simulation_B.b_kstr] =
      static_cast<uint8_T>
      (kinematic_simulation_B.e.f1[kinematic_simulation_B.b_kstr]);
    kinematic_simulation_B.msg.Name[4].Data[kinematic_simulation_B.b_kstr] =
      static_cast<uint8_T>
      (kinematic_simulation_B.f.f1[kinematic_simulation_B.b_kstr]);
    kinematic_simulation_B.msg.Name[5].Data[kinematic_simulation_B.b_kstr] =
      static_cast<uint8_T>
      (kinematic_simulation_B.g.f1[kinematic_simulation_B.b_kstr]);
  }

  kinematic_simulation_B.msg.Name[5].Data_SL_Info.CurrentLength = 11U;
  kinematic_simulation_B.msg.Position[5] =
    kinematic_simulation_X.Integrator_CSTATE[5];
  kinematic_simulation_B.msg.Velocity[5] =
    kinematic_simulation_B.In1.Velocities[5];

  // Outputs for Atomic SubSystem: '<Root>/Publish'
  // MATLABSystem: '<S7>/SinkBlock'
  Pub_kinematic_simulation_22.publish(&kinematic_simulation_B.msg);

  // End of Outputs for SubSystem: '<Root>/Publish'

  // MATLAB Function: '<Root>/Assign to Time msg'
  if (kinematic_simulation_B.K12 < 0.0) {
    kinematic_simulation_B.bid1 = ceil(kinematic_simulation_B.K12);
  } else {
    kinematic_simulation_B.bid1 = floor(kinematic_simulation_B.K12);
  }

  kinematic_simulation_B.msg_l.Clock_.Sec = kinematic_simulation_B.bid1;
  kinematic_simulation_B.K14 = (kinematic_simulation_B.K12 -
    kinematic_simulation_B.bid1) * 1.0E+9;
  if (kinematic_simulation_B.K14 < 0.0) {
    kinematic_simulation_B.msg_l.Clock_.Nsec = ceil(kinematic_simulation_B.K14);
  } else {
    kinematic_simulation_B.msg_l.Clock_.Nsec = floor(kinematic_simulation_B.K14);
  }

  // End of MATLAB Function: '<Root>/Assign to Time msg'

  // Outputs for Atomic SubSystem: '<Root>/Publish1'
  // MATLABSystem: '<S8>/SinkBlock'
  Pub_kinematic_simulation_50.publish(&kinematic_simulation_B.msg_l);

  // End of Outputs for SubSystem: '<Root>/Publish1'
  if (rtmIsMajorTimeStep(kinematic_simulation_M)) {
    // Update for Integrator: '<Root>/Integrator'
    kinematic_simulation_DW.Integrator_IWORK = 0;
  }                                    // end MajorTimeStep

  if (rtmIsMajorTimeStep(kinematic_simulation_M)) {
    rt_ertODEUpdateContinuousStates(&kinematic_simulation_M->solverInfo);

    // Update absolute time for base rate
    // The "clockTick0" counts the number of times the code of this task has
    //  been executed. The absolute time is the multiplication of "clockTick0"
    //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
    //  overflow during the application lifespan selected.

    ++kinematic_simulation_M->Timing.clockTick0;
    kinematic_simulation_M->Timing.t[0] = rtsiGetSolverStopTime
      (&kinematic_simulation_M->solverInfo);

    {
      // Update absolute timer for sample time: [0.05s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.05, which is the step size
      //  of the task. Size of "clockTick1" ensures timer will not overflow during the
      //  application lifespan selected.

      kinematic_simulation_M->Timing.clockTick1++;
    }
  }                                    // end MajorTimeStep
}

// Derivatives for root system: '<Root>'
void kinematic_simulation_derivatives(void)
{
  int32_T i;
  XDot_kinematic_simulation_T *_rtXdot;
  _rtXdot = ((XDot_kinematic_simulation_T *) kinematic_simulation_M->derivs);

  // Derivatives for Integrator: '<Root>/Integrator'
  for (i = 0; i < 6; i++) {
    _rtXdot->Integrator_CSTATE[i] = kinematic_simulation_B.In1.Velocities[i];
  }

  // End of Derivatives for Integrator: '<Root>/Integrator'
}

// Model initialize function
void kinematic_simulation_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&kinematic_simulation_M->solverInfo,
                          &kinematic_simulation_M->Timing.simTimeStep);
    rtsiSetTPtr(&kinematic_simulation_M->solverInfo, &rtmGetTPtr
                (kinematic_simulation_M));
    rtsiSetStepSizePtr(&kinematic_simulation_M->solverInfo,
                       &kinematic_simulation_M->Timing.stepSize0);
    rtsiSetdXPtr(&kinematic_simulation_M->solverInfo,
                 &kinematic_simulation_M->derivs);
    rtsiSetContStatesPtr(&kinematic_simulation_M->solverInfo, (real_T **)
                         &kinematic_simulation_M->contStates);
    rtsiSetNumContStatesPtr(&kinematic_simulation_M->solverInfo,
      &kinematic_simulation_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&kinematic_simulation_M->solverInfo,
      &kinematic_simulation_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&kinematic_simulation_M->solverInfo,
      &kinematic_simulation_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&kinematic_simulation_M->solverInfo,
      &kinematic_simulation_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&kinematic_simulation_M->solverInfo,
                          (&rtmGetErrorStatus(kinematic_simulation_M)));
    rtsiSetRTModelPtr(&kinematic_simulation_M->solverInfo,
                      kinematic_simulation_M);
  }

  rtsiSetSimTimeStep(&kinematic_simulation_M->solverInfo, MAJOR_TIME_STEP);
  kinematic_simulation_M->intgData.y = kinematic_simulation_M->odeY;
  kinematic_simulation_M->intgData.f[0] = kinematic_simulation_M->odeF[0];
  kinematic_simulation_M->intgData.f[1] = kinematic_simulation_M->odeF[1];
  kinematic_simulation_M->intgData.f[2] = kinematic_simulation_M->odeF[2];
  kinematic_simulation_M->contStates = ((X_kinematic_simulation_T *)
    &kinematic_simulation_X);
  rtsiSetSolverData(&kinematic_simulation_M->solverInfo, static_cast<void *>
                    (&kinematic_simulation_M->intgData));
  rtsiSetSolverName(&kinematic_simulation_M->solverInfo,"ode3");
  rtmSetTPtr(kinematic_simulation_M, &kinematic_simulation_M->Timing.tArray[0]);
  kinematic_simulation_M->Timing.stepSize0 = 0.05;
  rtmSetFirstInitCond(kinematic_simulation_M, 1);

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
    if (rtmIsFirstInitCond(kinematic_simulation_M)) {
      kinematic_simulation_X.Integrator_CSTATE[0] = 0.0;
      kinematic_simulation_X.Integrator_CSTATE[1] = 0.0;
      kinematic_simulation_X.Integrator_CSTATE[2] = 0.0;
      kinematic_simulation_X.Integrator_CSTATE[3] = 0.0;
      kinematic_simulation_X.Integrator_CSTATE[4] = 0.0;
      kinematic_simulation_X.Integrator_CSTATE[5] = 0.0;
    }

    kinematic_simulation_DW.Integrator_IWORK = 1;

    // End of InitializeConditions for Integrator: '<Root>/Integrator'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S11>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S19>/Out1'
    kinematic_simulation_B.In1 = kinematic_simulation_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S11>/Enabled Subsystem'

    // Start for MATLABSystem: '<S11>/SourceBlock'
    kinematic_simulation_DW.obj_pp.matlabCodegenIsDeleted = false;
    kinematic_simulation_DW.obj_pp.isInitialized = 1;
    for (i = 0; i < 17; i++) {
      kinematic_simulation_B.cv[i] = tmp_0[i];
    }

    kinematic_simulation_B.cv[17] = '\x00';
    Sub_kinematic_simulation_16.createSubscriber(kinematic_simulation_B.cv, 1);
    kinematic_simulation_DW.obj_pp.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish2'
    // Start for MATLABSystem: '<S9>/SinkBlock'
    kinematic_simulation_DW.obj_m.matlabCodegenIsDeleted = false;
    kinematic_simulation_DW.obj_m.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      kinematic_simulation_B.cv1[i] = tmp_1[i];
    }

    kinematic_simulation_B.cv1[14] = '\x00';
    Pub_kinematic_simulation_79.createPublisher(kinematic_simulation_B.cv1, 1);
    kinematic_simulation_DW.obj_m.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S9>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish2'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish3'
    // Start for MATLABSystem: '<S10>/SinkBlock'
    kinematic_simulation_DW.obj_bs.matlabCodegenIsDeleted = false;
    kinematic_simulation_DW.obj_bs.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      kinematic_simulation_B.cv1[i] = tmp_2[i];
    }

    kinematic_simulation_B.cv1[14] = '\x00';
    Pub_kinematic_simulation_101.createPublisher(kinematic_simulation_B.cv1, 1);
    kinematic_simulation_DW.obj_bs.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish3'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish'
    // Start for MATLABSystem: '<S7>/SinkBlock'
    kinematic_simulation_DW.obj_nr.matlabCodegenIsDeleted = false;
    kinematic_simulation_DW.obj_nr.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      kinematic_simulation_B.cv2[i] = tmp_3[i];
    }

    kinematic_simulation_B.cv2[13] = '\x00';
    Pub_kinematic_simulation_22.createPublisher(kinematic_simulation_B.cv2, 1);
    kinematic_simulation_DW.obj_nr.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S7>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish1'
    // Start for MATLABSystem: '<S8>/SinkBlock'
    kinematic_simulation_DW.obj_f.matlabCodegenIsDeleted = false;
    kinematic_simulation_DW.obj_f.isInitialized = 1;
    for (i = 0; i < 6; i++) {
      tmp[i] = tmp_4[i];
    }

    tmp[6] = '\x00';
    Pub_kinematic_simulation_50.createPublisher(tmp, 1);
    kinematic_simulation_DW.obj_f.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S8>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish1'

    // Start for MATLABSystem: '<S12>/Get Parameter'
    kinematic_simulation_DW.obj_e.matlabCodegenIsDeleted = false;
    kinematic_simulation_DW.obj_e.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      kinematic_simulation_B.cv3[i] = tmp_5[i];
    }

    kinematic_simulation_B.cv3[11] = '\x00';
    ParamGet_kinematic_simulation_61.initialize(kinematic_simulation_B.cv3);
    ParamGet_kinematic_simulation_61.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinematic_simulation_61.set_initial_value(0.0);
    kinematic_simulation_DW.obj_e.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S12>/Get Parameter'

    // Start for MATLABSystem: '<S12>/Get Parameter2'
    kinematic_simulation_DW.obj_p.matlabCodegenIsDeleted = false;
    kinematic_simulation_DW.obj_p.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      kinematic_simulation_B.cv3[i] = tmp_6[i];
    }

    kinematic_simulation_B.cv3[11] = '\x00';
    ParamGet_kinematic_simulation_85.initialize(kinematic_simulation_B.cv3);
    ParamGet_kinematic_simulation_85.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinematic_simulation_85.set_initial_value(0.0);
    kinematic_simulation_DW.obj_p.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S12>/Get Parameter2'

    // Start for MATLABSystem: '<S12>/Get Parameter5'
    kinematic_simulation_DW.obj_n.matlabCodegenIsDeleted = false;
    kinematic_simulation_DW.obj_n.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      kinematic_simulation_B.cv3[i] = tmp_7[i];
    }

    kinematic_simulation_B.cv3[11] = '\x00';
    ParamGet_kinematic_simulation_88.initialize(kinematic_simulation_B.cv3);
    ParamGet_kinematic_simulation_88.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinematic_simulation_88.set_initial_value(0.0);
    kinematic_simulation_DW.obj_n.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S12>/Get Parameter5'

    // Start for MATLABSystem: '<S12>/Get Parameter4'
    kinematic_simulation_DW.obj_em.matlabCodegenIsDeleted = false;
    kinematic_simulation_DW.obj_em.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      kinematic_simulation_B.cv3[i] = tmp_8[i];
    }

    kinematic_simulation_B.cv3[11] = '\x00';
    ParamGet_kinematic_simulation_87.initialize(kinematic_simulation_B.cv3);
    ParamGet_kinematic_simulation_87.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinematic_simulation_87.set_initial_value(0.0);
    kinematic_simulation_DW.obj_em.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S12>/Get Parameter4'

    // Start for MATLABSystem: '<S12>/Get Parameter3'
    kinematic_simulation_DW.obj_l.matlabCodegenIsDeleted = false;
    kinematic_simulation_DW.obj_l.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      kinematic_simulation_B.cv3[i] = tmp_9[i];
    }

    kinematic_simulation_B.cv3[11] = '\x00';
    ParamGet_kinematic_simulation_86.initialize(kinematic_simulation_B.cv3);
    ParamGet_kinematic_simulation_86.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinematic_simulation_86.set_initial_value(0.0);
    kinematic_simulation_DW.obj_l.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S12>/Get Parameter3'

    // Start for MATLABSystem: '<S12>/Get Parameter1'
    kinematic_simulation_DW.obj_ng.matlabCodegenIsDeleted = false;
    kinematic_simulation_DW.obj_ng.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      kinematic_simulation_B.cv3[i] = tmp_a[i];
    }

    kinematic_simulation_B.cv3[11] = '\x00';
    ParamGet_kinematic_simulation_65.initialize(kinematic_simulation_B.cv3);
    ParamGet_kinematic_simulation_65.initialize_error_codes(0, 1, 2, 3);
    ParamGet_kinematic_simulation_65.set_initial_value(0.0);
    kinematic_simulation_DW.obj_ng.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S12>/Get Parameter1'
    emxInitStruct_robotics_slmanip_(&kinematic_simulation_DW.obj);
    emxInitStruct_n_robotics_manip_(&kinematic_simulation_DW.gobj_1);
    emxInitStruct_n_robotics_manip_(&kinematic_simulation_DW.gobj_16);
    emxInitStruct_n_robotics_manip_(&kinematic_simulation_DW.gobj_15);
    emxInitStruct_n_robotics_manip_(&kinematic_simulation_DW.gobj_14);
    emxInitStruct_n_robotics_manip_(&kinematic_simulation_DW.gobj_13);
    emxInitStruct_n_robotics_manip_(&kinematic_simulation_DW.gobj_12);
    emxInitStruct_n_robotics_manip_(&kinematic_simulation_DW.gobj_11);
    emxInitStruct_n_robotics_manip_(&kinematic_simulation_DW.gobj_10);
    emxInitStruct_n_robotics_manip_(&kinematic_simulation_DW.gobj_9);
    emxInitStruct_n_robotics_manip_(&kinematic_simulation_DW.gobj_8);
    emxInitStruct_n_robotics_manip_(&kinematic_simulation_DW.gobj_7);
    emxInitStruct_n_robotics_manip_(&kinematic_simulation_DW.gobj_6);
    emxInitStruct_n_robotics_manip_(&kinematic_simulation_DW.gobj_5);
    emxInitStruct_n_robotics_manip_(&kinematic_simulation_DW.gobj_4);
    emxInitStruct_n_robotics_manip_(&kinematic_simulation_DW.gobj_3);
    emxInitStruct_n_robotics_manip_(&kinematic_simulation_DW.gobj_2);

    // Start for MATLABSystem: '<S16>/MATLAB System'
    kinematic_simulation_DW.obj.isInitialized = 0;
    kinematic_simulation_DW.obj.isInitialized = 1;
    kin_RigidBodyTree_RigidBodyTree(&kinematic_simulation_DW.obj.TreeInternal,
      &kinematic_simulation_DW.gobj_2, &kinematic_simulation_DW.gobj_4,
      &kinematic_simulation_DW.gobj_5, &kinematic_simulation_DW.gobj_6,
      &kinematic_simulation_DW.gobj_7, &kinematic_simulation_DW.gobj_8,
      &kinematic_simulation_DW.gobj_9, &kinematic_simulation_DW.gobj_3);
    emxInitStruct_robotics_slmani_a(&kinematic_simulation_DW.obj_b);
    emxInitStruct_n_robotics_mani_a(&kinematic_simulation_DW.gobj_1_a);
    emxInitStruct_n_robotics_mani_a(&kinematic_simulation_DW.gobj_16_m);
    emxInitStruct_n_robotics_mani_a(&kinematic_simulation_DW.gobj_15_f);
    emxInitStruct_n_robotics_mani_a(&kinematic_simulation_DW.gobj_14_n);
    emxInitStruct_n_robotics_mani_a(&kinematic_simulation_DW.gobj_13_j);
    emxInitStruct_n_robotics_mani_a(&kinematic_simulation_DW.gobj_12_b);
    emxInitStruct_n_robotics_mani_a(&kinematic_simulation_DW.gobj_11_l);
    emxInitStruct_n_robotics_mani_a(&kinematic_simulation_DW.gobj_10_b);
    emxInitStruct_n_robotics_mani_a(&kinematic_simulation_DW.gobj_9_p);
    emxInitStruct_n_robotics_mani_a(&kinematic_simulation_DW.gobj_8_j);
    emxInitStruct_n_robotics_mani_a(&kinematic_simulation_DW.gobj_7_l);
    emxInitStruct_n_robotics_mani_a(&kinematic_simulation_DW.gobj_6_g);
    emxInitStruct_n_robotics_mani_a(&kinematic_simulation_DW.gobj_5_a);
    emxInitStruct_n_robotics_mani_a(&kinematic_simulation_DW.gobj_4_i);
    emxInitStruct_n_robotics_mani_a(&kinematic_simulation_DW.gobj_3_j);
    emxInitStruct_n_robotics_mani_a(&kinematic_simulation_DW.gobj_2_o);

    // Start for MATLABSystem: '<S17>/MATLAB System'
    kinematic_simulation_DW.obj_b.isInitialized = 0;
    kinematic_simulation_DW.obj_b.isInitialized = 1;
    k_RigidBodyTree_RigidBodyTree_a(&kinematic_simulation_DW.obj_b.TreeInternal,
      &kinematic_simulation_DW.gobj_2_o, &kinematic_simulation_DW.gobj_4_i,
      &kinematic_simulation_DW.gobj_5_a, &kinematic_simulation_DW.gobj_6_g,
      &kinematic_simulation_DW.gobj_7_l, &kinematic_simulation_DW.gobj_8_j,
      &kinematic_simulation_DW.gobj_9_p, &kinematic_simulation_DW.gobj_3_j);
  }

  // set "at time zero" to false
  if (rtmIsFirstInitCond(kinematic_simulation_M)) {
    rtmSetFirstInitCond(kinematic_simulation_M, 0);
  }
}

// Model terminate function
void kinematic_simulation_terminate(void)
{
  // Terminate for MATLABSystem: '<S12>/Get Parameter'
  matlabCodegenHandle_matla_alw5t(&kinematic_simulation_DW.obj_e);

  // Terminate for MATLABSystem: '<S12>/Get Parameter2'
  matlabCodegenHandle_matla_alw5t(&kinematic_simulation_DW.obj_p);

  // Terminate for MATLABSystem: '<S12>/Get Parameter5'
  matlabCodegenHandle_matla_alw5t(&kinematic_simulation_DW.obj_n);

  // Terminate for MATLABSystem: '<S12>/Get Parameter4'
  matlabCodegenHandle_matla_alw5t(&kinematic_simulation_DW.obj_em);

  // Terminate for MATLABSystem: '<S12>/Get Parameter3'
  matlabCodegenHandle_matla_alw5t(&kinematic_simulation_DW.obj_l);

  // Terminate for MATLABSystem: '<S12>/Get Parameter1'
  matlabCodegenHandle_matla_alw5t(&kinematic_simulation_DW.obj_ng);
  emxFreeStruct_robotics_slmanip_(&kinematic_simulation_DW.obj);
  emxFreeStruct_n_robotics_manip_(&kinematic_simulation_DW.gobj_1);
  emxFreeStruct_n_robotics_manip_(&kinematic_simulation_DW.gobj_16);
  emxFreeStruct_n_robotics_manip_(&kinematic_simulation_DW.gobj_15);
  emxFreeStruct_n_robotics_manip_(&kinematic_simulation_DW.gobj_14);
  emxFreeStruct_n_robotics_manip_(&kinematic_simulation_DW.gobj_13);
  emxFreeStruct_n_robotics_manip_(&kinematic_simulation_DW.gobj_12);
  emxFreeStruct_n_robotics_manip_(&kinematic_simulation_DW.gobj_11);
  emxFreeStruct_n_robotics_manip_(&kinematic_simulation_DW.gobj_10);
  emxFreeStruct_n_robotics_manip_(&kinematic_simulation_DW.gobj_9);
  emxFreeStruct_n_robotics_manip_(&kinematic_simulation_DW.gobj_8);
  emxFreeStruct_n_robotics_manip_(&kinematic_simulation_DW.gobj_7);
  emxFreeStruct_n_robotics_manip_(&kinematic_simulation_DW.gobj_6);
  emxFreeStruct_n_robotics_manip_(&kinematic_simulation_DW.gobj_5);
  emxFreeStruct_n_robotics_manip_(&kinematic_simulation_DW.gobj_4);
  emxFreeStruct_n_robotics_manip_(&kinematic_simulation_DW.gobj_3);
  emxFreeStruct_n_robotics_manip_(&kinematic_simulation_DW.gobj_2);
  emxFreeStruct_robotics_slmani_a(&kinematic_simulation_DW.obj_b);
  emxFreeStruct_n_robotics_mani_a(&kinematic_simulation_DW.gobj_1_a);
  emxFreeStruct_n_robotics_mani_a(&kinematic_simulation_DW.gobj_16_m);
  emxFreeStruct_n_robotics_mani_a(&kinematic_simulation_DW.gobj_15_f);
  emxFreeStruct_n_robotics_mani_a(&kinematic_simulation_DW.gobj_14_n);
  emxFreeStruct_n_robotics_mani_a(&kinematic_simulation_DW.gobj_13_j);
  emxFreeStruct_n_robotics_mani_a(&kinematic_simulation_DW.gobj_12_b);
  emxFreeStruct_n_robotics_mani_a(&kinematic_simulation_DW.gobj_11_l);
  emxFreeStruct_n_robotics_mani_a(&kinematic_simulation_DW.gobj_10_b);
  emxFreeStruct_n_robotics_mani_a(&kinematic_simulation_DW.gobj_9_p);
  emxFreeStruct_n_robotics_mani_a(&kinematic_simulation_DW.gobj_8_j);
  emxFreeStruct_n_robotics_mani_a(&kinematic_simulation_DW.gobj_7_l);
  emxFreeStruct_n_robotics_mani_a(&kinematic_simulation_DW.gobj_6_g);
  emxFreeStruct_n_robotics_mani_a(&kinematic_simulation_DW.gobj_5_a);
  emxFreeStruct_n_robotics_mani_a(&kinematic_simulation_DW.gobj_4_i);
  emxFreeStruct_n_robotics_mani_a(&kinematic_simulation_DW.gobj_3_j);
  emxFreeStruct_n_robotics_mani_a(&kinematic_simulation_DW.gobj_2_o);

  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S11>/SourceBlock'
  matlabCodegenHandle_matlab_alw5(&kinematic_simulation_DW.obj_pp);

  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for Atomic SubSystem: '<Root>/Publish2'
  // Terminate for MATLABSystem: '<S9>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&kinematic_simulation_DW.obj_m);

  // End of Terminate for SubSystem: '<Root>/Publish2'

  // Terminate for Atomic SubSystem: '<Root>/Publish3'
  // Terminate for MATLABSystem: '<S10>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&kinematic_simulation_DW.obj_bs);

  // End of Terminate for SubSystem: '<Root>/Publish3'

  // Terminate for Atomic SubSystem: '<Root>/Publish'
  // Terminate for MATLABSystem: '<S7>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&kinematic_simulation_DW.obj_nr);

  // End of Terminate for SubSystem: '<Root>/Publish'

  // Terminate for Atomic SubSystem: '<Root>/Publish1'
  // Terminate for MATLABSystem: '<S8>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&kinematic_simulation_DW.obj_f);

  // End of Terminate for SubSystem: '<Root>/Publish1'
}

//
// File trailer for generated code.
//
// [EOF]
//
