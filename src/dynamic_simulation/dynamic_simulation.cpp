//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: dynamic_simulation.cpp
//
// Code generated for Simulink model 'dynamic_simulation'.
//
// Model version                  : 1.127
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Fri May 15 20:39:42 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "dynamic_simulation.h"
#include "dynamic_simulation_private.h"

// Block signals (default storage)
B_dynamic_simulation_T dynamic_simulation_B;

// Continuous states
X_dynamic_simulation_T dynamic_simulation_X;

// Block states (default storage)
DW_dynamic_simulation_T dynamic_simulation_DW;

// Real-time model
RT_MODEL_dynamic_simulation_T dynamic_simulation_M_ =
  RT_MODEL_dynamic_simulation_T();
RT_MODEL_dynamic_simulation_T *const dynamic_simulation_M =
  &dynamic_simulation_M_;

// Forward declaration for local functions
static void dynamic_simulati_emxInit_real_T(emxArray_real_T_dynamic_simul_T
  **pEmxArray, int32_T numDimensions);
static void dynamic_sim_emxInit_f_cell_wrap(emxArray_f_cell_wrap_dynamic__T
  **pEmxArray, int32_T numDimensions);
static void dynamic_simulati_emxInit_char_T(emxArray_char_T_dynamic_simul_T
  **pEmxArray, int32_T numDimensions);
static void d_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_dynamic__T
  *emxArray, int32_T oldNumel);
static void dynami_emxEnsureCapacity_char_T(emxArray_char_T_dynamic_simul_T
  *emxArray, int32_T oldNumel);
static void dy_rigidBodyJoint_get_JointAxis(const
  c_rigidBodyJoint_dynamic_simu_T *obj, real_T ax[3]);
static void dynamic_simulati_emxFree_char_T(emxArray_char_T_dynamic_simul_T
  **pEmxArray);
static void RigidBodyTree_forwardKinematics(p_robotics_manip_internal_Rig_T *obj,
  const real_T qvec[6], emxArray_f_cell_wrap_dynamic__T *Ttree);
static void dynami_emxEnsureCapacity_real_T(emxArray_real_T_dynamic_simul_T
  *emxArray, int32_T oldNumel);
static void dynamic_simulati_emxFree_real_T(emxArray_real_T_dynamic_simul_T
  **pEmxArray);
static void dynamic_sim_emxFree_f_cell_wrap(emxArray_f_cell_wrap_dynamic__T
  **pEmxArray);
static void RigidBodyTree_geometricJacobian(p_robotics_manip_internal_Rig_T *obj,
  const real_T Q[6], emxArray_real_T_dynamic_simul_T *Jac);
static void rigidBodyJoint_get_JointAxis_k(const c_rigidBodyJoint_dynamic_si_k_T
  *obj, real_T ax[3]);
static void RigidBodyTree_forwardKinemati_k(p_robotics_manip_internal_R_k_T *obj,
  const real_T qvec[6], emxArray_f_cell_wrap_dynamic__T *Ttree);
static void dynamic_simulat_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Data[128], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_dynamic_simulation_std_msgs_MultiArrayDimension varargout_2_Layout_Dim
  [16], uint32_T *varargout_2_Layout_Dim_SL_Info_, uint32_T
  *varargout_2_Layout_Dim_SL_Inf_0);
static void dynamic_si_emxInit_f_cell_wrap1(emxArray_f_cell_wrap_dynami_k_T
  **pEmxArray, int32_T numDimensions);
static void emxEnsureCapacity_f_cell_wrap1(emxArray_f_cell_wrap_dynami_k_T
  *emxArray, int32_T oldNumel);
static void rigidBodyJoint_get_JointAxis_kq(const
  c_rigidBodyJoint_dynamic_s_kq_T *obj, real_T ax[3]);
static void dynamic_simulation_cat(real_T varargin_1, real_T varargin_2, real_T
  varargin_3, real_T varargin_4, real_T varargin_5, real_T varargin_6, real_T
  varargin_7, real_T varargin_8, real_T varargin_9, real_T y[9]);
static void rigidBodyJoint_transformBodyT_k(const
  c_rigidBodyJoint_dynamic_s_kq_T *obj, const real_T q_data[], const int32_T
  *q_size, real_T T[16]);
static void rigidBodyJoint_transformBodyToP(const
  c_rigidBodyJoint_dynamic_s_kq_T *obj, real_T T[16]);
static void dynamic_simulation_tforminv(const real_T T[16], real_T Tinv[16]);
static void dynamic_sim_tformToSpatialXform(const real_T T[16], real_T X[36]);
static void dynamic_si_emxFree_f_cell_wrap1(emxArray_f_cell_wrap_dynami_k_T
  **pEmxArray);
static void RigidBodyTreeDynamics_massMatri(p_robotics_manip_internal__kq_T
  *robot, const real_T q[6], emxArray_real_T_dynamic_simul_T *H,
  emxArray_real_T_dynamic_simul_T *lambda);
static void RigidBodyTreeDynamics_inverseDy(p_robotics_manip_internal__kq_T
  *robot, const real_T q[6], const real_T qdot[6], const real_T fext[48], real_T
  tau[6]);
static boolean_T dynamic_simulation_anyNonFinite(const real_T x[16]);
static real_T dynamic_simulatio_rt_hypotd_snf(real_T u0, real_T u1);
static real_T dynamic_simulation_xzlangeM(const creal_T x[16]);
static void dynamic_simulation_xzlascl(real_T cfrom, real_T cto, creal_T A[16]);
static real_T dynamic_simulation_xzlanhs(const creal_T A[16], int32_T ilo,
  int32_T ihi);
static void dynamic_simulation_xzlartg_d(const creal_T f, const creal_T g,
  real_T *cs, creal_T *sn);
static void dynamic_simulation_xzlartg(const creal_T f, const creal_T g, real_T *
  cs, creal_T *sn, creal_T *r);
static void dynamic_simulation_xzhgeqz(creal_T A[16], int32_T ilo, int32_T ihi,
  creal_T Z[16], int32_T *info, creal_T alpha1[4], creal_T beta1[4]);
static void dynamic_simulation_xztgevc(const creal_T A[16], creal_T V[16]);
static void dynamic_simulation_xzggev(creal_T A[16], int32_T *info, creal_T
  alpha1[4], creal_T beta1[4], creal_T V[16]);
static real_T dynamic_simulation_xnrm2(int32_T n, const real_T x[16], int32_T
  ix0);
static void dynamic_simulation_xzlarf(int32_T m, int32_T n, int32_T iv0, real_T
  tau, real_T C[16], int32_T ic0, real_T work[4]);
static void dynamic_simulation_xgehrd(real_T a[16], real_T tau[3]);
static real_T dynamic_simulation_xnrm2_a(int32_T n, const real_T x[3]);
static real_T dynamic_simulation_xzlarfg(int32_T n, real_T *alpha1, real_T x[3]);
static void dynamic_simulation_xdlanv2(real_T *a, real_T *b, real_T *c, real_T
  *d, real_T *rt1r, real_T *rt1i, real_T *rt2r, real_T *rt2i, real_T *cs, real_T
  *sn);
static void dynamic_simulation_xrot(int32_T n, real_T x[16], int32_T ix0,
  int32_T iy0, real_T c, real_T s);
static void dynamic_simulation_xrot_l(int32_T n, real_T x[16], int32_T ix0,
  int32_T iy0, real_T c, real_T s);
static int32_T dynamic_simulation_eml_dlahqr(real_T h[16], real_T z[16]);
static void dynamic_simulation_eig(const real_T A[16], creal_T V[16], creal_T D
  [4]);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_GetP_T *obj);
static void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_dynamic_simu_T
  *pStruct);
static void emxFreeStruct_o_robotics_manip_(o_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_p_robotics_manip_(p_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct);
static void emxFreeStruct_n_robotics_manip_(n_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_c_rigidBodyJoint1(c_rigidBodyJoint_dynamic_si_k_T
  *pStruct);
static void emxFreeStruct_o_robotics_mani_k(o_robotics_manip_internal_R_k_T
  *pStruct);
static void emxFreeStruct_p_robotics_mani_k(p_robotics_manip_internal_R_k_T
  *pStruct);
static void emxFreeStruct_robotics_slmani_k(robotics_slmanip_internal_b_k_T
  *pStruct);
static void emxFreeStruct_n_robotics_mani_k(n_robotics_manip_internal_R_k_T
  *pStruct);
static void dynamic_simul_matlabCodegenHa_e(ros_slros_internal_block_Subs_T *obj);
static void emxFreeStruct_c_rigidBodyJoint2(c_rigidBodyJoint_dynamic_s_kq_T
  *pStruct);
static void emxFreeStruct_o_robotics_man_kq(o_robotics_manip_internal__kq_T
  *pStruct);
static void emxFreeStruct_p_robotics_man_kq(p_robotics_manip_internal__kq_T
  *pStruct);
static void emxFreeStruct_robotics_slman_kq(robotics_slmanip_internal__kq_T
  *pStruct);
static void emxFreeStruct_n_robotics_man_kq(n_robotics_manip_internal__kq_T
  *pStruct);
static void matlabCodegenHandle_matl_kq2tzt(ros_slros_internal_block_Publ_T *obj);
static void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_dynamic_simu_T
  *pStruct);
static void emxInitStruct_o_robotics_manip_(o_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_p_robotics_manip_(p_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct);
static void emxInitStruct_n_robotics_manip_(n_robotics_manip_internal_Rig_T
  *pStruct);
static n_robotics_manip_internal_Rig_T *dynamic_sim_RigidBody_RigidBody
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *dynamic_s_RigidBody_RigidBody_k
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *dynamic__RigidBody_RigidBody_kq
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *dynamic_RigidBody_RigidBody_kq2
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *dynami_RigidBody_RigidBody_kq2t
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *dynam_RigidBody_RigidBody_kq2tz
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *dyna_RigidBody_RigidBody_kq2tzt
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *dyn_RigidBody_RigidBody_kq2tzta
  (n_robotics_manip_internal_Rig_T *obj);
static o_robotics_manip_internal_Rig_T *dy_RigidBody_RigidBody_kq2tztaj
  (o_robotics_manip_internal_Rig_T *obj);
static p_robotics_manip_internal_Rig_T *dyn_RigidBodyTree_RigidBodyTree
  (p_robotics_manip_internal_Rig_T *obj, n_robotics_manip_internal_Rig_T *iobj_0,
   n_robotics_manip_internal_Rig_T *iobj_1, n_robotics_manip_internal_Rig_T
   *iobj_2, n_robotics_manip_internal_Rig_T *iobj_3,
   n_robotics_manip_internal_Rig_T *iobj_4, n_robotics_manip_internal_Rig_T
   *iobj_5, n_robotics_manip_internal_Rig_T *iobj_6,
   n_robotics_manip_internal_Rig_T *iobj_7);
static void emxInitStruct_c_rigidBodyJoint1(c_rigidBodyJoint_dynamic_si_k_T
  *pStruct);
static void emxInitStruct_o_robotics_mani_k(o_robotics_manip_internal_R_k_T
  *pStruct);
static void emxInitStruct_p_robotics_mani_k(p_robotics_manip_internal_R_k_T
  *pStruct);
static void emxInitStruct_robotics_slmani_k(robotics_slmanip_internal_b_k_T
  *pStruct);
static void emxInitStruct_n_robotics_mani_k(n_robotics_manip_internal_R_k_T
  *pStruct);
static n_robotics_manip_internal_R_k_T *d_RigidBody_RigidBody_kq2tztajw
  (n_robotics_manip_internal_R_k_T *obj);
static n_robotics_manip_internal_R_k_T *RigidBody_RigidBody_kq2tztajw1
  (n_robotics_manip_internal_R_k_T *obj);
static n_robotics_manip_internal_R_k_T *RigidBody_RigidBody_kq2tztajw1b
  (n_robotics_manip_internal_R_k_T *obj);
static n_robotics_manip_internal_R_k_T *RigidBody_RigidBod_kq2tztajw1bp
  (n_robotics_manip_internal_R_k_T *obj);
static n_robotics_manip_internal_R_k_T *RigidBody_RigidBo_kq2tztajw1bp5
  (n_robotics_manip_internal_R_k_T *obj);
static p_robotics_manip_internal_R_k_T *d_RigidBodyTree_RigidBodyTree_k
  (p_robotics_manip_internal_R_k_T *obj, n_robotics_manip_internal_R_k_T *iobj_0,
   n_robotics_manip_internal_R_k_T *iobj_1, n_robotics_manip_internal_R_k_T
   *iobj_2, n_robotics_manip_internal_R_k_T *iobj_3,
   n_robotics_manip_internal_R_k_T *iobj_4, n_robotics_manip_internal_R_k_T
   *iobj_5, n_robotics_manip_internal_R_k_T *iobj_6,
   n_robotics_manip_internal_R_k_T *iobj_7);
static void emxInitStruct_c_rigidBodyJoint2(c_rigidBodyJoint_dynamic_s_kq_T
  *pStruct);
static void emxInitStruct_o_robotics_man_kq(o_robotics_manip_internal__kq_T
  *pStruct);
static void emxInitStruct_p_robotics_man_kq(p_robotics_manip_internal__kq_T
  *pStruct);
static void emxInitStruct_robotics_slman_kq(robotics_slmanip_internal__kq_T
  *pStruct);
static void emxInitStruct_n_robotics_man_kq(n_robotics_manip_internal__kq_T
  *pStruct);
static n_robotics_manip_internal__kq_T *RigidBody_RigidB_kq2tztajw1bp5d
  (n_robotics_manip_internal__kq_T *obj);
static n_robotics_manip_internal__kq_T *RigidBody_Rigid_kq2tztajw1bp5do
  (n_robotics_manip_internal__kq_T *obj);
static n_robotics_manip_internal__kq_T *d_RigidBody_Rigid_m
  (n_robotics_manip_internal__kq_T *obj);
static n_robotics_manip_internal__kq_T *d_RigidBody_Rigid_b
  (n_robotics_manip_internal__kq_T *obj);
static n_robotics_manip_internal__kq_T *d_RigidBody_Rigid_a
  (n_robotics_manip_internal__kq_T *obj);
static n_robotics_manip_internal__kq_T *d_RigidBody_Rigid_h
  (n_robotics_manip_internal__kq_T *obj);
static n_robotics_manip_internal__kq_T *d_RigidBody_Rigid_i
  (n_robotics_manip_internal__kq_T *obj);
static n_robotics_manip_internal__kq_T *d_RigidBody_Rigid_mc
  (n_robotics_manip_internal__kq_T *obj);
static o_robotics_manip_internal__kq_T *d_RigidBody_Rigid_e
  (o_robotics_manip_internal__kq_T *obj);
static p_robotics_manip_internal__kq_T *RigidBodyTree_RigidBodyTree_kq
  (p_robotics_manip_internal__kq_T *obj, n_robotics_manip_internal__kq_T *iobj_0,
   n_robotics_manip_internal__kq_T *iobj_1, n_robotics_manip_internal__kq_T
   *iobj_2, n_robotics_manip_internal__kq_T *iobj_3,
   n_robotics_manip_internal__kq_T *iobj_4, n_robotics_manip_internal__kq_T
   *iobj_5, n_robotics_manip_internal__kq_T *iobj_6,
   n_robotics_manip_internal__kq_T *iobj_7);
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
  int_T nXc = 12;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  // Save the state values at time t in y, we'll use x as ynew.
  (void) memcpy(y, x,
                static_cast<uint_T>(nXc)*sizeof(real_T));

  // Assumes that rtsiSetT and ModelOutputs are up-to-date
  // f0 = f(t,y)
  rtsiSetdX(si, f0);
  dynamic_simulation_derivatives();

  // f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*));
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  dynamic_simulation_step();
  dynamic_simulation_derivatives();

  // f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*));
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  dynamic_simulation_step();
  dynamic_simulation_derivatives();

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

static void dynamic_simulati_emxInit_real_T(emxArray_real_T_dynamic_simul_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_real_T_dynamic_simul_T *emxArray;
  *pEmxArray = (emxArray_real_T_dynamic_simul_T *)malloc(sizeof
    (emxArray_real_T_dynamic_simul_T));
  emxArray = *pEmxArray;
  emxArray->data = (real_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (dynamic_simulation_B.i_jr = 0; dynamic_simulation_B.i_jr < numDimensions;
       dynamic_simulation_B.i_jr++) {
    emxArray->size[dynamic_simulation_B.i_jr] = 0;
  }
}

static void dynamic_sim_emxInit_f_cell_wrap(emxArray_f_cell_wrap_dynamic__T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_f_cell_wrap_dynamic__T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_f_cell_wrap_dynamic__T *)malloc(sizeof
    (emxArray_f_cell_wrap_dynamic__T));
  emxArray = *pEmxArray;
  emxArray->data = (f_cell_wrap_dynamic_simulatio_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void dynamic_simulati_emxInit_char_T(emxArray_char_T_dynamic_simul_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_char_T_dynamic_simul_T *emxArray;
  *pEmxArray = (emxArray_char_T_dynamic_simul_T *)malloc(sizeof
    (emxArray_char_T_dynamic_simul_T));
  emxArray = *pEmxArray;
  emxArray->data = (char_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (dynamic_simulation_B.i_d = 0; dynamic_simulation_B.i_d < numDimensions;
       dynamic_simulation_B.i_d++) {
    emxArray->size[dynamic_simulation_B.i_d] = 0;
  }
}

static void d_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_dynamic__T
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

    newData = calloc(static_cast<uint32_T>(i), sizeof
                     (f_cell_wrap_dynamic_simulatio_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(f_cell_wrap_dynamic_simulatio_T)
             * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (f_cell_wrap_dynamic_simulatio_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void dynami_emxEnsureCapacity_char_T(emxArray_char_T_dynamic_simul_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  dynamic_simulation_B.newNumel_d = 1;
  for (dynamic_simulation_B.i_g = 0; dynamic_simulation_B.i_g <
       emxArray->numDimensions; dynamic_simulation_B.i_g++) {
    dynamic_simulation_B.newNumel_d *= emxArray->size[dynamic_simulation_B.i_g];
  }

  if (dynamic_simulation_B.newNumel_d > emxArray->allocatedSize) {
    dynamic_simulation_B.i_g = emxArray->allocatedSize;
    if (dynamic_simulation_B.i_g < 16) {
      dynamic_simulation_B.i_g = 16;
    }

    while (dynamic_simulation_B.i_g < dynamic_simulation_B.newNumel_d) {
      if (dynamic_simulation_B.i_g > 1073741823) {
        dynamic_simulation_B.i_g = MAX_int32_T;
      } else {
        dynamic_simulation_B.i_g <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(dynamic_simulation_B.i_g), sizeof
                     (char_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(char_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (char_T *)newData;
    emxArray->allocatedSize = dynamic_simulation_B.i_g;
    emxArray->canFreeData = true;
  }
}

static void dy_rigidBodyJoint_get_JointAxis(const
  c_rigidBodyJoint_dynamic_simu_T *obj, real_T ax[3])
{
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  boolean_T guard1 = false;
  int32_T exitg1;
  for (dynamic_simulation_B.b_kstr_o = 0; dynamic_simulation_B.b_kstr_o < 8;
       dynamic_simulation_B.b_kstr_o++) {
    dynamic_simulation_B.b_as[dynamic_simulation_B.b_kstr_o] =
      tmp[dynamic_simulation_B.b_kstr_o];
  }

  dynamic_simulation_B.b_bool_d = false;
  if (obj->Type->size[1] == 8) {
    dynamic_simulation_B.b_kstr_o = 1;
    do {
      exitg1 = 0;
      if (dynamic_simulation_B.b_kstr_o - 1 < 8) {
        dynamic_simulation_B.kstr_i = dynamic_simulation_B.b_kstr_o - 1;
        if (obj->Type->data[dynamic_simulation_B.kstr_i] !=
            dynamic_simulation_B.b_as[dynamic_simulation_B.kstr_i]) {
          exitg1 = 1;
        } else {
          dynamic_simulation_B.b_kstr_o++;
        }
      } else {
        dynamic_simulation_B.b_bool_d = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (dynamic_simulation_B.b_bool_d) {
    guard1 = true;
  } else {
    for (dynamic_simulation_B.b_kstr_o = 0; dynamic_simulation_B.b_kstr_o < 9;
         dynamic_simulation_B.b_kstr_o++) {
      dynamic_simulation_B.b_px[dynamic_simulation_B.b_kstr_o] =
        tmp_0[dynamic_simulation_B.b_kstr_o];
    }

    dynamic_simulation_B.b_bool_d = false;
    if (obj->Type->size[1] == 9) {
      dynamic_simulation_B.b_kstr_o = 1;
      do {
        exitg1 = 0;
        if (dynamic_simulation_B.b_kstr_o - 1 < 9) {
          dynamic_simulation_B.kstr_i = dynamic_simulation_B.b_kstr_o - 1;
          if (obj->Type->data[dynamic_simulation_B.kstr_i] !=
              dynamic_simulation_B.b_px[dynamic_simulation_B.kstr_i]) {
            exitg1 = 1;
          } else {
            dynamic_simulation_B.b_kstr_o++;
          }
        } else {
          dynamic_simulation_B.b_bool_d = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynamic_simulation_B.b_bool_d) {
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

static void dynamic_simulati_emxFree_char_T(emxArray_char_T_dynamic_simul_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_char_T_dynamic_simul_T *)NULL) {
    if (((*pEmxArray)->data != (char_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_char_T_dynamic_simul_T *)NULL;
  }
}

static void RigidBodyTree_forwardKinematics(p_robotics_manip_internal_Rig_T *obj,
  const real_T qvec[6], emxArray_f_cell_wrap_dynamic__T *Ttree)
{
  n_robotics_manip_internal_Rig_T *body;
  emxArray_char_T_dynamic_simul_T *switch_expression;
  static const int8_T tmp[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  dynamic_simulation_B.n = obj->NumBodies;
  for (dynamic_simulation_B.b_kstr_p = 0; dynamic_simulation_B.b_kstr_p < 16;
       dynamic_simulation_B.b_kstr_p++) {
    dynamic_simulation_B.c_f1[dynamic_simulation_B.b_kstr_p] =
      tmp[dynamic_simulation_B.b_kstr_p];
  }

  dynamic_simulation_B.b_kstr_p = Ttree->size[0] * Ttree->size[1];
  Ttree->size[0] = 1;
  dynamic_simulation_B.e = static_cast<int32_T>(dynamic_simulation_B.n);
  Ttree->size[1] = dynamic_simulation_B.e;
  d_emxEnsureCapacity_f_cell_wrap(Ttree, dynamic_simulation_B.b_kstr_p);
  if (dynamic_simulation_B.e != 0) {
    dynamic_simulation_B.ntilecols = dynamic_simulation_B.e - 1;
    if (0 <= dynamic_simulation_B.ntilecols) {
      memcpy(&dynamic_simulation_B.expl_temp.f1[0], &dynamic_simulation_B.c_f1[0],
             sizeof(real_T) << 4U);
    }

    for (dynamic_simulation_B.b_jtilecol = 0; dynamic_simulation_B.b_jtilecol <=
         dynamic_simulation_B.ntilecols; dynamic_simulation_B.b_jtilecol++) {
      Ttree->data[dynamic_simulation_B.b_jtilecol] =
        dynamic_simulation_B.expl_temp;
    }
  }

  dynamic_simulation_B.k = 1.0;
  dynamic_simulation_B.ntilecols = static_cast<int32_T>(dynamic_simulation_B.n)
    - 1;
  dynamic_simulati_emxInit_char_T(&switch_expression, 2);
  if (0 <= dynamic_simulation_B.ntilecols) {
    for (dynamic_simulation_B.b_kstr_p = 0; dynamic_simulation_B.b_kstr_p < 5;
         dynamic_simulation_B.b_kstr_p++) {
      dynamic_simulation_B.b_f[dynamic_simulation_B.b_kstr_p] =
        tmp_0[dynamic_simulation_B.b_kstr_p];
    }
  }

  for (dynamic_simulation_B.b_jtilecol = 0; dynamic_simulation_B.b_jtilecol <=
       dynamic_simulation_B.ntilecols; dynamic_simulation_B.b_jtilecol++) {
    body = obj->Bodies[dynamic_simulation_B.b_jtilecol];
    dynamic_simulation_B.n = body->JointInternal.PositionNumber;
    dynamic_simulation_B.n += dynamic_simulation_B.k;
    if (dynamic_simulation_B.k > dynamic_simulation_B.n - 1.0) {
      dynamic_simulation_B.e = 0;
      dynamic_simulation_B.d_p = 0;
    } else {
      dynamic_simulation_B.e = static_cast<int32_T>(dynamic_simulation_B.k) - 1;
      dynamic_simulation_B.d_p = static_cast<int32_T>(dynamic_simulation_B.n -
        1.0);
    }

    dynamic_simulation_B.b_kstr_p = switch_expression->size[0] *
      switch_expression->size[1];
    switch_expression->size[0] = 1;
    switch_expression->size[1] = body->JointInternal.Type->size[1];
    dynami_emxEnsureCapacity_char_T(switch_expression,
      dynamic_simulation_B.b_kstr_p);
    dynamic_simulation_B.loop_ub_f = body->JointInternal.Type->size[0] *
      body->JointInternal.Type->size[1] - 1;
    for (dynamic_simulation_B.b_kstr_p = 0; dynamic_simulation_B.b_kstr_p <=
         dynamic_simulation_B.loop_ub_f; dynamic_simulation_B.b_kstr_p++) {
      switch_expression->data[dynamic_simulation_B.b_kstr_p] =
        body->JointInternal.Type->data[dynamic_simulation_B.b_kstr_p];
    }

    dynamic_simulation_B.b_bool_e = false;
    if (switch_expression->size[1] == 5) {
      dynamic_simulation_B.b_kstr_p = 1;
      do {
        exitg1 = 0;
        if (dynamic_simulation_B.b_kstr_p - 1 < 5) {
          dynamic_simulation_B.loop_ub_f = dynamic_simulation_B.b_kstr_p - 1;
          if (switch_expression->data[dynamic_simulation_B.loop_ub_f] !=
              dynamic_simulation_B.b_f[dynamic_simulation_B.loop_ub_f]) {
            exitg1 = 1;
          } else {
            dynamic_simulation_B.b_kstr_p++;
          }
        } else {
          dynamic_simulation_B.b_bool_e = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynamic_simulation_B.b_bool_e) {
      dynamic_simulation_B.b_kstr_p = 0;
    } else {
      for (dynamic_simulation_B.b_kstr_p = 0; dynamic_simulation_B.b_kstr_p < 8;
           dynamic_simulation_B.b_kstr_p++) {
        dynamic_simulation_B.b_ax[dynamic_simulation_B.b_kstr_p] =
          tmp_1[dynamic_simulation_B.b_kstr_p];
      }

      dynamic_simulation_B.b_bool_e = false;
      if (switch_expression->size[1] == 8) {
        dynamic_simulation_B.b_kstr_p = 1;
        do {
          exitg1 = 0;
          if (dynamic_simulation_B.b_kstr_p - 1 < 8) {
            dynamic_simulation_B.loop_ub_f = dynamic_simulation_B.b_kstr_p - 1;
            if (switch_expression->data[dynamic_simulation_B.loop_ub_f] !=
                dynamic_simulation_B.b_ax[dynamic_simulation_B.loop_ub_f]) {
              exitg1 = 1;
            } else {
              dynamic_simulation_B.b_kstr_p++;
            }
          } else {
            dynamic_simulation_B.b_bool_e = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (dynamic_simulation_B.b_bool_e) {
        dynamic_simulation_B.b_kstr_p = 1;
      } else {
        dynamic_simulation_B.b_kstr_p = -1;
      }
    }

    switch (dynamic_simulation_B.b_kstr_p) {
     case 0:
      memset(&dynamic_simulation_B.c_f1[0], 0, sizeof(real_T) << 4U);
      dynamic_simulation_B.c_f1[0] = 1.0;
      dynamic_simulation_B.c_f1[5] = 1.0;
      dynamic_simulation_B.c_f1[10] = 1.0;
      dynamic_simulation_B.c_f1[15] = 1.0;
      break;

     case 1:
      dy_rigidBodyJoint_get_JointAxis(&body->JointInternal,
        dynamic_simulation_B.v);
      dynamic_simulation_B.d_p -= dynamic_simulation_B.e;
      for (dynamic_simulation_B.b_kstr_p = 0; dynamic_simulation_B.b_kstr_p <
           dynamic_simulation_B.d_p; dynamic_simulation_B.b_kstr_p++) {
        dynamic_simulation_B.e_data[dynamic_simulation_B.b_kstr_p] =
          dynamic_simulation_B.e + dynamic_simulation_B.b_kstr_p;
      }

      dynamic_simulation_B.result_data[0] = dynamic_simulation_B.v[0];
      dynamic_simulation_B.result_data[1] = dynamic_simulation_B.v[1];
      dynamic_simulation_B.result_data[2] = dynamic_simulation_B.v[2];
      if (0 <= (dynamic_simulation_B.d_p != 0) - 1) {
        dynamic_simulation_B.result_data[3] = qvec[dynamic_simulation_B.e_data[0]];
      }

      dynamic_simulation_B.k = 1.0 / sqrt((dynamic_simulation_B.result_data[0] *
        dynamic_simulation_B.result_data[0] + dynamic_simulation_B.result_data[1]
        * dynamic_simulation_B.result_data[1]) +
        dynamic_simulation_B.result_data[2] * dynamic_simulation_B.result_data[2]);
      dynamic_simulation_B.v[0] = dynamic_simulation_B.result_data[0] *
        dynamic_simulation_B.k;
      dynamic_simulation_B.v[1] = dynamic_simulation_B.result_data[1] *
        dynamic_simulation_B.k;
      dynamic_simulation_B.v[2] = dynamic_simulation_B.result_data[2] *
        dynamic_simulation_B.k;
      dynamic_simulation_B.k = cos(dynamic_simulation_B.result_data[3]);
      dynamic_simulation_B.sth = sin(dynamic_simulation_B.result_data[3]);
      dynamic_simulation_B.tempR[0] = dynamic_simulation_B.v[0] *
        dynamic_simulation_B.v[0] * (1.0 - dynamic_simulation_B.k) +
        dynamic_simulation_B.k;
      dynamic_simulation_B.tempR_tmp = dynamic_simulation_B.v[1] *
        dynamic_simulation_B.v[0] * (1.0 - dynamic_simulation_B.k);
      dynamic_simulation_B.tempR_tmp_i = dynamic_simulation_B.v[2] *
        dynamic_simulation_B.sth;
      dynamic_simulation_B.tempR[1] = dynamic_simulation_B.tempR_tmp -
        dynamic_simulation_B.tempR_tmp_i;
      dynamic_simulation_B.tempR_tmp_f = dynamic_simulation_B.v[2] *
        dynamic_simulation_B.v[0] * (1.0 - dynamic_simulation_B.k);
      dynamic_simulation_B.tempR_tmp_g = dynamic_simulation_B.v[1] *
        dynamic_simulation_B.sth;
      dynamic_simulation_B.tempR[2] = dynamic_simulation_B.tempR_tmp_f +
        dynamic_simulation_B.tempR_tmp_g;
      dynamic_simulation_B.tempR[3] = dynamic_simulation_B.tempR_tmp +
        dynamic_simulation_B.tempR_tmp_i;
      dynamic_simulation_B.tempR[4] = dynamic_simulation_B.v[1] *
        dynamic_simulation_B.v[1] * (1.0 - dynamic_simulation_B.k) +
        dynamic_simulation_B.k;
      dynamic_simulation_B.tempR_tmp = dynamic_simulation_B.v[2] *
        dynamic_simulation_B.v[1] * (1.0 - dynamic_simulation_B.k);
      dynamic_simulation_B.tempR_tmp_i = dynamic_simulation_B.v[0] *
        dynamic_simulation_B.sth;
      dynamic_simulation_B.tempR[5] = dynamic_simulation_B.tempR_tmp -
        dynamic_simulation_B.tempR_tmp_i;
      dynamic_simulation_B.tempR[6] = dynamic_simulation_B.tempR_tmp_f -
        dynamic_simulation_B.tempR_tmp_g;
      dynamic_simulation_B.tempR[7] = dynamic_simulation_B.tempR_tmp +
        dynamic_simulation_B.tempR_tmp_i;
      dynamic_simulation_B.tempR[8] = dynamic_simulation_B.v[2] *
        dynamic_simulation_B.v[2] * (1.0 - dynamic_simulation_B.k) +
        dynamic_simulation_B.k;
      for (dynamic_simulation_B.b_kstr_p = 0; dynamic_simulation_B.b_kstr_p < 3;
           dynamic_simulation_B.b_kstr_p++) {
        dynamic_simulation_B.e = dynamic_simulation_B.b_kstr_p + 1;
        dynamic_simulation_B.R_dh[dynamic_simulation_B.e - 1] =
          dynamic_simulation_B.tempR[(dynamic_simulation_B.e - 1) * 3];
        dynamic_simulation_B.e = dynamic_simulation_B.b_kstr_p + 1;
        dynamic_simulation_B.R_dh[dynamic_simulation_B.e + 2] =
          dynamic_simulation_B.tempR[(dynamic_simulation_B.e - 1) * 3 + 1];
        dynamic_simulation_B.e = dynamic_simulation_B.b_kstr_p + 1;
        dynamic_simulation_B.R_dh[dynamic_simulation_B.e + 5] =
          dynamic_simulation_B.tempR[(dynamic_simulation_B.e - 1) * 3 + 2];
      }

      memset(&dynamic_simulation_B.c_f1[0], 0, sizeof(real_T) << 4U);
      for (dynamic_simulation_B.b_kstr_p = 0; dynamic_simulation_B.b_kstr_p < 3;
           dynamic_simulation_B.b_kstr_p++) {
        dynamic_simulation_B.d_p = dynamic_simulation_B.b_kstr_p << 2;
        dynamic_simulation_B.c_f1[dynamic_simulation_B.d_p] =
          dynamic_simulation_B.R_dh[3 * dynamic_simulation_B.b_kstr_p];
        dynamic_simulation_B.c_f1[dynamic_simulation_B.d_p + 1] =
          dynamic_simulation_B.R_dh[3 * dynamic_simulation_B.b_kstr_p + 1];
        dynamic_simulation_B.c_f1[dynamic_simulation_B.d_p + 2] =
          dynamic_simulation_B.R_dh[3 * dynamic_simulation_B.b_kstr_p + 2];
      }

      dynamic_simulation_B.c_f1[15] = 1.0;
      break;

     default:
      dy_rigidBodyJoint_get_JointAxis(&body->JointInternal,
        dynamic_simulation_B.v);
      memset(&dynamic_simulation_B.tempR[0], 0, 9U * sizeof(real_T));
      dynamic_simulation_B.tempR[0] = 1.0;
      dynamic_simulation_B.tempR[4] = 1.0;
      dynamic_simulation_B.tempR[8] = 1.0;
      for (dynamic_simulation_B.b_kstr_p = 0; dynamic_simulation_B.b_kstr_p < 3;
           dynamic_simulation_B.b_kstr_p++) {
        dynamic_simulation_B.d_p = dynamic_simulation_B.b_kstr_p << 2;
        dynamic_simulation_B.c_f1[dynamic_simulation_B.d_p] =
          dynamic_simulation_B.tempR[3 * dynamic_simulation_B.b_kstr_p];
        dynamic_simulation_B.c_f1[dynamic_simulation_B.d_p + 1] =
          dynamic_simulation_B.tempR[3 * dynamic_simulation_B.b_kstr_p + 1];
        dynamic_simulation_B.c_f1[dynamic_simulation_B.d_p + 2] =
          dynamic_simulation_B.tempR[3 * dynamic_simulation_B.b_kstr_p + 2];
        dynamic_simulation_B.c_f1[dynamic_simulation_B.b_kstr_p + 12] =
          dynamic_simulation_B.v[dynamic_simulation_B.b_kstr_p] *
          qvec[dynamic_simulation_B.e];
      }

      dynamic_simulation_B.c_f1[3] = 0.0;
      dynamic_simulation_B.c_f1[7] = 0.0;
      dynamic_simulation_B.c_f1[11] = 0.0;
      dynamic_simulation_B.c_f1[15] = 1.0;
      break;
    }

    for (dynamic_simulation_B.b_kstr_p = 0; dynamic_simulation_B.b_kstr_p < 16;
         dynamic_simulation_B.b_kstr_p++) {
      dynamic_simulation_B.a[dynamic_simulation_B.b_kstr_p] =
        body->JointInternal.JointToParentTransform[dynamic_simulation_B.b_kstr_p];
    }

    for (dynamic_simulation_B.b_kstr_p = 0; dynamic_simulation_B.b_kstr_p < 16;
         dynamic_simulation_B.b_kstr_p++) {
      dynamic_simulation_B.b[dynamic_simulation_B.b_kstr_p] =
        body->JointInternal.ChildToJointTransform[dynamic_simulation_B.b_kstr_p];
    }

    for (dynamic_simulation_B.b_kstr_p = 0; dynamic_simulation_B.b_kstr_p < 4;
         dynamic_simulation_B.b_kstr_p++) {
      for (dynamic_simulation_B.e = 0; dynamic_simulation_B.e < 4;
           dynamic_simulation_B.e++) {
        dynamic_simulation_B.d_p = dynamic_simulation_B.e << 2;
        dynamic_simulation_B.loop_ub_f = dynamic_simulation_B.b_kstr_p +
          dynamic_simulation_B.d_p;
        dynamic_simulation_B.a_p[dynamic_simulation_B.loop_ub_f] = 0.0;
        dynamic_simulation_B.a_p[dynamic_simulation_B.loop_ub_f] +=
          dynamic_simulation_B.c_f1[dynamic_simulation_B.d_p] *
          dynamic_simulation_B.a[dynamic_simulation_B.b_kstr_p];
        dynamic_simulation_B.a_p[dynamic_simulation_B.loop_ub_f] +=
          dynamic_simulation_B.c_f1[dynamic_simulation_B.d_p + 1] *
          dynamic_simulation_B.a[dynamic_simulation_B.b_kstr_p + 4];
        dynamic_simulation_B.a_p[dynamic_simulation_B.loop_ub_f] +=
          dynamic_simulation_B.c_f1[dynamic_simulation_B.d_p + 2] *
          dynamic_simulation_B.a[dynamic_simulation_B.b_kstr_p + 8];
        dynamic_simulation_B.a_p[dynamic_simulation_B.loop_ub_f] +=
          dynamic_simulation_B.c_f1[dynamic_simulation_B.d_p + 3] *
          dynamic_simulation_B.a[dynamic_simulation_B.b_kstr_p + 12];
      }

      for (dynamic_simulation_B.e = 0; dynamic_simulation_B.e < 4;
           dynamic_simulation_B.e++) {
        dynamic_simulation_B.d_p = dynamic_simulation_B.e << 2;
        dynamic_simulation_B.loop_ub_f = dynamic_simulation_B.b_kstr_p +
          dynamic_simulation_B.d_p;
        Ttree->data[dynamic_simulation_B.b_jtilecol]
          .f1[dynamic_simulation_B.loop_ub_f] = 0.0;
        Ttree->data[dynamic_simulation_B.b_jtilecol]
          .f1[dynamic_simulation_B.loop_ub_f] +=
          dynamic_simulation_B.b[dynamic_simulation_B.d_p] *
          dynamic_simulation_B.a_p[dynamic_simulation_B.b_kstr_p];
        Ttree->data[dynamic_simulation_B.b_jtilecol]
          .f1[dynamic_simulation_B.loop_ub_f] +=
          dynamic_simulation_B.b[dynamic_simulation_B.d_p + 1] *
          dynamic_simulation_B.a_p[dynamic_simulation_B.b_kstr_p + 4];
        Ttree->data[dynamic_simulation_B.b_jtilecol]
          .f1[dynamic_simulation_B.loop_ub_f] +=
          dynamic_simulation_B.b[dynamic_simulation_B.d_p + 2] *
          dynamic_simulation_B.a_p[dynamic_simulation_B.b_kstr_p + 8];
        Ttree->data[dynamic_simulation_B.b_jtilecol]
          .f1[dynamic_simulation_B.loop_ub_f] +=
          dynamic_simulation_B.b[dynamic_simulation_B.d_p + 3] *
          dynamic_simulation_B.a_p[dynamic_simulation_B.b_kstr_p + 12];
      }
    }

    dynamic_simulation_B.k = dynamic_simulation_B.n;
    if (body->ParentIndex > 0.0) {
      for (dynamic_simulation_B.b_kstr_p = 0; dynamic_simulation_B.b_kstr_p < 16;
           dynamic_simulation_B.b_kstr_p++) {
        dynamic_simulation_B.a[dynamic_simulation_B.b_kstr_p] = Ttree->data[
          static_cast<int32_T>(body->ParentIndex) - 1]
          .f1[dynamic_simulation_B.b_kstr_p];
      }

      for (dynamic_simulation_B.b_kstr_p = 0; dynamic_simulation_B.b_kstr_p < 4;
           dynamic_simulation_B.b_kstr_p++) {
        for (dynamic_simulation_B.e = 0; dynamic_simulation_B.e < 4;
             dynamic_simulation_B.e++) {
          dynamic_simulation_B.d_p = dynamic_simulation_B.e << 2;
          dynamic_simulation_B.loop_ub_f = dynamic_simulation_B.b_kstr_p +
            dynamic_simulation_B.d_p;
          dynamic_simulation_B.a_p[dynamic_simulation_B.loop_ub_f] = 0.0;
          dynamic_simulation_B.a_p[dynamic_simulation_B.loop_ub_f] +=
            Ttree->data[dynamic_simulation_B.b_jtilecol]
            .f1[dynamic_simulation_B.d_p] *
            dynamic_simulation_B.a[dynamic_simulation_B.b_kstr_p];
          dynamic_simulation_B.a_p[dynamic_simulation_B.loop_ub_f] +=
            Ttree->data[dynamic_simulation_B.b_jtilecol]
            .f1[dynamic_simulation_B.d_p + 1] *
            dynamic_simulation_B.a[dynamic_simulation_B.b_kstr_p + 4];
          dynamic_simulation_B.a_p[dynamic_simulation_B.loop_ub_f] +=
            Ttree->data[dynamic_simulation_B.b_jtilecol]
            .f1[dynamic_simulation_B.d_p + 2] *
            dynamic_simulation_B.a[dynamic_simulation_B.b_kstr_p + 8];
          dynamic_simulation_B.a_p[dynamic_simulation_B.loop_ub_f] +=
            Ttree->data[dynamic_simulation_B.b_jtilecol]
            .f1[dynamic_simulation_B.d_p + 3] *
            dynamic_simulation_B.a[dynamic_simulation_B.b_kstr_p + 12];
        }
      }

      for (dynamic_simulation_B.b_kstr_p = 0; dynamic_simulation_B.b_kstr_p < 16;
           dynamic_simulation_B.b_kstr_p++) {
        Ttree->data[dynamic_simulation_B.b_jtilecol]
          .f1[dynamic_simulation_B.b_kstr_p] =
          dynamic_simulation_B.a_p[dynamic_simulation_B.b_kstr_p];
      }
    }
  }

  dynamic_simulati_emxFree_char_T(&switch_expression);
}

static void dynami_emxEnsureCapacity_real_T(emxArray_real_T_dynamic_simul_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  dynamic_simulation_B.newNumel = 1;
  for (dynamic_simulation_B.i_if = 0; dynamic_simulation_B.i_if <
       emxArray->numDimensions; dynamic_simulation_B.i_if++) {
    dynamic_simulation_B.newNumel *= emxArray->size[dynamic_simulation_B.i_if];
  }

  if (dynamic_simulation_B.newNumel > emxArray->allocatedSize) {
    dynamic_simulation_B.i_if = emxArray->allocatedSize;
    if (dynamic_simulation_B.i_if < 16) {
      dynamic_simulation_B.i_if = 16;
    }

    while (dynamic_simulation_B.i_if < dynamic_simulation_B.newNumel) {
      if (dynamic_simulation_B.i_if > 1073741823) {
        dynamic_simulation_B.i_if = MAX_int32_T;
      } else {
        dynamic_simulation_B.i_if <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(dynamic_simulation_B.i_if), sizeof
                     (real_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(real_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (real_T *)newData;
    emxArray->allocatedSize = dynamic_simulation_B.i_if;
    emxArray->canFreeData = true;
  }
}

static void dynamic_simulati_emxFree_real_T(emxArray_real_T_dynamic_simul_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T_dynamic_simul_T *)NULL) {
    if (((*pEmxArray)->data != (real_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_real_T_dynamic_simul_T *)NULL;
  }
}

static void dynamic_sim_emxFree_f_cell_wrap(emxArray_f_cell_wrap_dynamic__T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_f_cell_wrap_dynamic__T *)NULL) {
    if (((*pEmxArray)->data != (f_cell_wrap_dynamic_simulatio_T *)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_f_cell_wrap_dynamic__T *)NULL;
  }
}

static void RigidBodyTree_geometricJacobian(p_robotics_manip_internal_Rig_T *obj,
  const real_T Q[6], emxArray_real_T_dynamic_simul_T *Jac)
{
  emxArray_f_cell_wrap_dynamic__T *Ttree;
  n_robotics_manip_internal_Rig_T *body;
  emxArray_real_T_dynamic_simul_T *JacSlice;
  emxArray_char_T_dynamic_simul_T *bname;
  emxArray_real_T_dynamic_simul_T *b;
  static const char_T tmp[11] = { 'e', 'd', 'o', '_', 'l', 'i', 'n', 'k', '_',
    'e', 'e' };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  int32_T exitg1;
  boolean_T exitg2;
  dynamic_sim_emxInit_f_cell_wrap(&Ttree, 2);
  RigidBodyTree_forwardKinematics(obj, Q, Ttree);
  dynamic_simulation_B.b_kstr_n = Jac->size[0] * Jac->size[1];
  Jac->size[0] = 6;
  Jac->size[1] = static_cast<int32_T>(obj->VelocityNumber);
  dynami_emxEnsureCapacity_real_T(Jac, dynamic_simulation_B.b_kstr_n);
  dynamic_simulation_B.loop_ub = 6 * static_cast<int32_T>(obj->VelocityNumber) -
    1;
  for (dynamic_simulation_B.b_kstr_n = 0; dynamic_simulation_B.b_kstr_n <=
       dynamic_simulation_B.loop_ub; dynamic_simulation_B.b_kstr_n++) {
    Jac->data[dynamic_simulation_B.b_kstr_n] = 0.0;
  }

  for (dynamic_simulation_B.b_kstr_n = 0; dynamic_simulation_B.b_kstr_n < 8;
       dynamic_simulation_B.b_kstr_n++) {
    dynamic_simulation_B.chainmask[dynamic_simulation_B.b_kstr_n] = 0;
  }

  dynamic_simulati_emxInit_char_T(&bname, 2);
  dynamic_simulation_B.b_kstr_n = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj->Base.NameInternal->size[1];
  dynami_emxEnsureCapacity_char_T(bname, dynamic_simulation_B.b_kstr_n);
  dynamic_simulation_B.loop_ub = obj->Base.NameInternal->size[0] *
    obj->Base.NameInternal->size[1] - 1;
  for (dynamic_simulation_B.b_kstr_n = 0; dynamic_simulation_B.b_kstr_n <=
       dynamic_simulation_B.loop_ub; dynamic_simulation_B.b_kstr_n++) {
    bname->data[dynamic_simulation_B.b_kstr_n] = obj->Base.NameInternal->
      data[dynamic_simulation_B.b_kstr_n];
  }

  for (dynamic_simulation_B.b_kstr_n = 0; dynamic_simulation_B.b_kstr_n < 11;
       dynamic_simulation_B.b_kstr_n++) {
    dynamic_simulation_B.a_c[dynamic_simulation_B.b_kstr_n] =
      tmp[dynamic_simulation_B.b_kstr_n];
  }

  dynamic_simulation_B.b_bool_b = false;
  if (11 == bname->size[1]) {
    dynamic_simulation_B.b_kstr_n = 1;
    do {
      exitg1 = 0;
      if (dynamic_simulation_B.b_kstr_n - 1 < 11) {
        dynamic_simulation_B.kstr = dynamic_simulation_B.b_kstr_n - 1;
        if (dynamic_simulation_B.a_c[dynamic_simulation_B.kstr] != bname->
            data[dynamic_simulation_B.kstr]) {
          exitg1 = 1;
        } else {
          dynamic_simulation_B.b_kstr_n++;
        }
      } else {
        dynamic_simulation_B.b_bool_b = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (dynamic_simulation_B.b_bool_b) {
    memset(&dynamic_simulation_B.T2inv[0], 0, sizeof(real_T) << 4U);
    dynamic_simulation_B.T2inv[0] = 1.0;
    dynamic_simulation_B.T2inv[5] = 1.0;
    dynamic_simulation_B.T2inv[10] = 1.0;
    dynamic_simulation_B.T2inv[15] = 1.0;
    memset(&dynamic_simulation_B.T2_c[0], 0, sizeof(real_T) << 4U);
    dynamic_simulation_B.T2_c[0] = 1.0;
    dynamic_simulation_B.T2_c[5] = 1.0;
    dynamic_simulation_B.T2_c[10] = 1.0;
    dynamic_simulation_B.T2_c[15] = 1.0;
  } else {
    dynamic_simulation_B.endeffectorIndex = -1.0;
    dynamic_simulation_B.b_kstr_n = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = obj->Base.NameInternal->size[1];
    dynami_emxEnsureCapacity_char_T(bname, dynamic_simulation_B.b_kstr_n);
    dynamic_simulation_B.loop_ub = obj->Base.NameInternal->size[0] *
      obj->Base.NameInternal->size[1] - 1;
    for (dynamic_simulation_B.b_kstr_n = 0; dynamic_simulation_B.b_kstr_n <=
         dynamic_simulation_B.loop_ub; dynamic_simulation_B.b_kstr_n++) {
      bname->data[dynamic_simulation_B.b_kstr_n] = obj->Base.NameInternal->
        data[dynamic_simulation_B.b_kstr_n];
    }

    for (dynamic_simulation_B.b_kstr_n = 0; dynamic_simulation_B.b_kstr_n < 11;
         dynamic_simulation_B.b_kstr_n++) {
      dynamic_simulation_B.a_c[dynamic_simulation_B.b_kstr_n] =
        tmp[dynamic_simulation_B.b_kstr_n];
    }

    dynamic_simulation_B.b_bool_b = false;
    if (bname->size[1] == 11) {
      dynamic_simulation_B.b_kstr_n = 1;
      do {
        exitg1 = 0;
        if (dynamic_simulation_B.b_kstr_n - 1 < 11) {
          dynamic_simulation_B.kstr = dynamic_simulation_B.b_kstr_n - 1;
          if (bname->data[dynamic_simulation_B.kstr] !=
              dynamic_simulation_B.a_c[dynamic_simulation_B.kstr]) {
            exitg1 = 1;
          } else {
            dynamic_simulation_B.b_kstr_n++;
          }
        } else {
          dynamic_simulation_B.b_bool_b = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynamic_simulation_B.b_bool_b) {
      dynamic_simulation_B.endeffectorIndex = 0.0;
    } else {
      dynamic_simulation_B.idx_idx_1 = obj->NumBodies;
      dynamic_simulation_B.b_i_o = 0;
      exitg2 = false;
      while ((!exitg2) && (dynamic_simulation_B.b_i_o <= static_cast<int32_T>
                           (dynamic_simulation_B.idx_idx_1) - 1)) {
        body = obj->Bodies[dynamic_simulation_B.b_i_o];
        dynamic_simulation_B.b_kstr_n = bname->size[0] * bname->size[1];
        bname->size[0] = 1;
        bname->size[1] = body->NameInternal->size[1];
        dynami_emxEnsureCapacity_char_T(bname, dynamic_simulation_B.b_kstr_n);
        dynamic_simulation_B.loop_ub = body->NameInternal->size[0] *
          body->NameInternal->size[1] - 1;
        for (dynamic_simulation_B.b_kstr_n = 0; dynamic_simulation_B.b_kstr_n <=
             dynamic_simulation_B.loop_ub; dynamic_simulation_B.b_kstr_n++) {
          bname->data[dynamic_simulation_B.b_kstr_n] = body->NameInternal->
            data[dynamic_simulation_B.b_kstr_n];
        }

        for (dynamic_simulation_B.b_kstr_n = 0; dynamic_simulation_B.b_kstr_n <
             11; dynamic_simulation_B.b_kstr_n++) {
          dynamic_simulation_B.a_c[dynamic_simulation_B.b_kstr_n] =
            tmp[dynamic_simulation_B.b_kstr_n];
        }

        dynamic_simulation_B.b_bool_b = false;
        if (bname->size[1] == 11) {
          dynamic_simulation_B.b_kstr_n = 1;
          do {
            exitg1 = 0;
            if (dynamic_simulation_B.b_kstr_n - 1 < 11) {
              dynamic_simulation_B.kstr = dynamic_simulation_B.b_kstr_n - 1;
              if (bname->data[dynamic_simulation_B.kstr] !=
                  dynamic_simulation_B.a_c[dynamic_simulation_B.kstr]) {
                exitg1 = 1;
              } else {
                dynamic_simulation_B.b_kstr_n++;
              }
            } else {
              dynamic_simulation_B.b_bool_b = true;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }

        if (dynamic_simulation_B.b_bool_b) {
          dynamic_simulation_B.endeffectorIndex = static_cast<real_T>
            (dynamic_simulation_B.b_i_o) + 1.0;
          exitg2 = true;
        } else {
          dynamic_simulation_B.b_i_o++;
        }
      }
    }

    dynamic_simulation_B.b_i_o = static_cast<int32_T>
      (dynamic_simulation_B.endeffectorIndex) - 1;
    body = obj->Bodies[dynamic_simulation_B.b_i_o];
    for (dynamic_simulation_B.b_kstr_n = 0; dynamic_simulation_B.b_kstr_n < 16;
         dynamic_simulation_B.b_kstr_n++) {
      dynamic_simulation_B.T2_c[dynamic_simulation_B.b_kstr_n] = Ttree->
        data[dynamic_simulation_B.b_i_o].f1[dynamic_simulation_B.b_kstr_n];
    }

    for (dynamic_simulation_B.b_kstr_n = 0; dynamic_simulation_B.b_kstr_n < 3;
         dynamic_simulation_B.b_kstr_n++) {
      dynamic_simulation_B.R_g[3 * dynamic_simulation_B.b_kstr_n] = Ttree->
        data[dynamic_simulation_B.b_i_o].f1[dynamic_simulation_B.b_kstr_n];
      dynamic_simulation_B.R_g[3 * dynamic_simulation_B.b_kstr_n + 1] =
        Ttree->data[dynamic_simulation_B.b_i_o].f1[dynamic_simulation_B.b_kstr_n
        + 4];
      dynamic_simulation_B.R_g[3 * dynamic_simulation_B.b_kstr_n + 2] =
        Ttree->data[dynamic_simulation_B.b_i_o].f1[dynamic_simulation_B.b_kstr_n
        + 8];
    }

    for (dynamic_simulation_B.b_kstr_n = 0; dynamic_simulation_B.b_kstr_n < 9;
         dynamic_simulation_B.b_kstr_n++) {
      dynamic_simulation_B.R_l[dynamic_simulation_B.b_kstr_n] =
        -dynamic_simulation_B.R_g[dynamic_simulation_B.b_kstr_n];
    }

    for (dynamic_simulation_B.b_kstr_n = 0; dynamic_simulation_B.b_kstr_n < 3;
         dynamic_simulation_B.b_kstr_n++) {
      dynamic_simulation_B.endeffectorIndex = Ttree->
        data[dynamic_simulation_B.b_i_o].f1[12] *
        dynamic_simulation_B.R_l[dynamic_simulation_B.b_kstr_n];
      dynamic_simulation_B.loop_ub = dynamic_simulation_B.b_kstr_n << 2;
      dynamic_simulation_B.T2inv[dynamic_simulation_B.loop_ub] =
        dynamic_simulation_B.R_g[3 * dynamic_simulation_B.b_kstr_n];
      dynamic_simulation_B.endeffectorIndex +=
        dynamic_simulation_B.R_l[dynamic_simulation_B.b_kstr_n + 3] *
        Ttree->data[dynamic_simulation_B.b_i_o].f1[13];
      dynamic_simulation_B.T2inv[dynamic_simulation_B.loop_ub + 1] =
        dynamic_simulation_B.R_g[3 * dynamic_simulation_B.b_kstr_n + 1];
      dynamic_simulation_B.endeffectorIndex +=
        dynamic_simulation_B.R_l[dynamic_simulation_B.b_kstr_n + 6] *
        Ttree->data[dynamic_simulation_B.b_i_o].f1[14];
      dynamic_simulation_B.T2inv[dynamic_simulation_B.loop_ub + 2] =
        dynamic_simulation_B.R_g[3 * dynamic_simulation_B.b_kstr_n + 2];
      dynamic_simulation_B.T2inv[dynamic_simulation_B.b_kstr_n + 12] =
        dynamic_simulation_B.endeffectorIndex;
    }

    dynamic_simulation_B.T2inv[3] = 0.0;
    dynamic_simulation_B.T2inv[7] = 0.0;
    dynamic_simulation_B.T2inv[11] = 0.0;
    dynamic_simulation_B.T2inv[15] = 1.0;
    dynamic_simulation_B.chainmask[dynamic_simulation_B.b_i_o] = 1;
    while (body->ParentIndex > 0.0) {
      body = obj->Bodies[static_cast<int32_T>(body->ParentIndex) - 1];
      dynamic_simulation_B.chainmask[static_cast<int32_T>(body->Index) - 1] = 1;
    }
  }

  dynamic_simulation_B.idx_idx_1 = obj->NumBodies;
  dynamic_simulation_B.c_m = static_cast<int32_T>(dynamic_simulation_B.idx_idx_1)
    - 1;
  dynamic_simulati_emxInit_real_T(&JacSlice, 2);
  dynamic_simulati_emxInit_real_T(&b, 2);
  if (0 <= dynamic_simulation_B.c_m) {
    for (dynamic_simulation_B.b_kstr_n = 0; dynamic_simulation_B.b_kstr_n < 5;
         dynamic_simulation_B.b_kstr_n++) {
      dynamic_simulation_B.b_ex[dynamic_simulation_B.b_kstr_n] =
        tmp_0[dynamic_simulation_B.b_kstr_n];
    }
  }

  for (dynamic_simulation_B.b_i_o = 0; dynamic_simulation_B.b_i_o <=
       dynamic_simulation_B.c_m; dynamic_simulation_B.b_i_o++) {
    body = obj->Bodies[dynamic_simulation_B.b_i_o];
    dynamic_simulation_B.b_kstr_n = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = body->JointInternal.Type->size[1];
    dynami_emxEnsureCapacity_char_T(bname, dynamic_simulation_B.b_kstr_n);
    dynamic_simulation_B.loop_ub = body->JointInternal.Type->size[0] *
      body->JointInternal.Type->size[1] - 1;
    for (dynamic_simulation_B.b_kstr_n = 0; dynamic_simulation_B.b_kstr_n <=
         dynamic_simulation_B.loop_ub; dynamic_simulation_B.b_kstr_n++) {
      bname->data[dynamic_simulation_B.b_kstr_n] = body->
        JointInternal.Type->data[dynamic_simulation_B.b_kstr_n];
    }

    dynamic_simulation_B.b_bool_b = false;
    if (bname->size[1] == 5) {
      dynamic_simulation_B.b_kstr_n = 1;
      do {
        exitg1 = 0;
        if (dynamic_simulation_B.b_kstr_n - 1 < 5) {
          dynamic_simulation_B.kstr = dynamic_simulation_B.b_kstr_n - 1;
          if (bname->data[dynamic_simulation_B.kstr] !=
              dynamic_simulation_B.b_ex[dynamic_simulation_B.kstr]) {
            exitg1 = 1;
          } else {
            dynamic_simulation_B.b_kstr_n++;
          }
        } else {
          dynamic_simulation_B.b_bool_b = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if ((!dynamic_simulation_B.b_bool_b) &&
        (dynamic_simulation_B.chainmask[dynamic_simulation_B.b_i_o] != 0)) {
      for (dynamic_simulation_B.b_kstr_n = 0; dynamic_simulation_B.b_kstr_n < 16;
           dynamic_simulation_B.b_kstr_n++) {
        dynamic_simulation_B.T1_b[dynamic_simulation_B.b_kstr_n] = Ttree->data[
          static_cast<int32_T>(body->Index) - 1]
          .f1[dynamic_simulation_B.b_kstr_n];
      }

      for (dynamic_simulation_B.b_kstr_n = 0; dynamic_simulation_B.b_kstr_n < 16;
           dynamic_simulation_B.b_kstr_n++) {
        dynamic_simulation_B.Tdh[dynamic_simulation_B.b_kstr_n] =
          body->
          JointInternal.ChildToJointTransform[dynamic_simulation_B.b_kstr_n];
      }

      for (dynamic_simulation_B.b_kstr_n = 0; dynamic_simulation_B.b_kstr_n < 3;
           dynamic_simulation_B.b_kstr_n++) {
        dynamic_simulation_B.R_g[3 * dynamic_simulation_B.b_kstr_n] =
          dynamic_simulation_B.Tdh[dynamic_simulation_B.b_kstr_n];
        dynamic_simulation_B.R_g[3 * dynamic_simulation_B.b_kstr_n + 1] =
          dynamic_simulation_B.Tdh[dynamic_simulation_B.b_kstr_n + 4];
        dynamic_simulation_B.R_g[3 * dynamic_simulation_B.b_kstr_n + 2] =
          dynamic_simulation_B.Tdh[dynamic_simulation_B.b_kstr_n + 8];
      }

      for (dynamic_simulation_B.b_kstr_n = 0; dynamic_simulation_B.b_kstr_n < 9;
           dynamic_simulation_B.b_kstr_n++) {
        dynamic_simulation_B.R_l[dynamic_simulation_B.b_kstr_n] =
          -dynamic_simulation_B.R_g[dynamic_simulation_B.b_kstr_n];
      }

      for (dynamic_simulation_B.b_kstr_n = 0; dynamic_simulation_B.b_kstr_n < 3;
           dynamic_simulation_B.b_kstr_n++) {
        dynamic_simulation_B.R_m[dynamic_simulation_B.b_kstr_n] =
          dynamic_simulation_B.R_l[dynamic_simulation_B.b_kstr_n + 6] *
          dynamic_simulation_B.Tdh[14] +
          (dynamic_simulation_B.R_l[dynamic_simulation_B.b_kstr_n + 3] *
           dynamic_simulation_B.Tdh[13] +
           dynamic_simulation_B.R_l[dynamic_simulation_B.b_kstr_n] *
           dynamic_simulation_B.Tdh[12]);
      }

      for (dynamic_simulation_B.b_kstr_n = 0; dynamic_simulation_B.b_kstr_n < 4;
           dynamic_simulation_B.b_kstr_n++) {
        for (dynamic_simulation_B.kstr = 0; dynamic_simulation_B.kstr < 4;
             dynamic_simulation_B.kstr++) {
          dynamic_simulation_B.n_l = dynamic_simulation_B.kstr << 2;
          dynamic_simulation_B.loop_ub = dynamic_simulation_B.b_kstr_n +
            dynamic_simulation_B.n_l;
          dynamic_simulation_B.Tdh[dynamic_simulation_B.loop_ub] = 0.0;
          dynamic_simulation_B.Tdh[dynamic_simulation_B.loop_ub] +=
            dynamic_simulation_B.T1_b[dynamic_simulation_B.n_l] *
            dynamic_simulation_B.T2inv[dynamic_simulation_B.b_kstr_n];
          dynamic_simulation_B.Tdh[dynamic_simulation_B.loop_ub] +=
            dynamic_simulation_B.T1_b[dynamic_simulation_B.n_l + 1] *
            dynamic_simulation_B.T2inv[dynamic_simulation_B.b_kstr_n + 4];
          dynamic_simulation_B.Tdh[dynamic_simulation_B.loop_ub] +=
            dynamic_simulation_B.T1_b[dynamic_simulation_B.n_l + 2] *
            dynamic_simulation_B.T2inv[dynamic_simulation_B.b_kstr_n + 8];
          dynamic_simulation_B.Tdh[dynamic_simulation_B.loop_ub] +=
            dynamic_simulation_B.T1_b[dynamic_simulation_B.n_l + 3] *
            dynamic_simulation_B.T2inv[dynamic_simulation_B.b_kstr_n + 12];
        }
      }

      for (dynamic_simulation_B.b_kstr_n = 0; dynamic_simulation_B.b_kstr_n < 3;
           dynamic_simulation_B.b_kstr_n++) {
        dynamic_simulation_B.kstr = dynamic_simulation_B.b_kstr_n << 2;
        dynamic_simulation_B.T1_b[dynamic_simulation_B.kstr] =
          dynamic_simulation_B.R_g[3 * dynamic_simulation_B.b_kstr_n];
        dynamic_simulation_B.T1_b[dynamic_simulation_B.kstr + 1] =
          dynamic_simulation_B.R_g[3 * dynamic_simulation_B.b_kstr_n + 1];
        dynamic_simulation_B.T1_b[dynamic_simulation_B.kstr + 2] =
          dynamic_simulation_B.R_g[3 * dynamic_simulation_B.b_kstr_n + 2];
        dynamic_simulation_B.T1_b[dynamic_simulation_B.b_kstr_n + 12] =
          dynamic_simulation_B.R_m[dynamic_simulation_B.b_kstr_n];
      }

      dynamic_simulation_B.T1_b[3] = 0.0;
      dynamic_simulation_B.T1_b[7] = 0.0;
      dynamic_simulation_B.T1_b[11] = 0.0;
      dynamic_simulation_B.T1_b[15] = 1.0;
      for (dynamic_simulation_B.b_kstr_n = 0; dynamic_simulation_B.b_kstr_n < 4;
           dynamic_simulation_B.b_kstr_n++) {
        for (dynamic_simulation_B.kstr = 0; dynamic_simulation_B.kstr < 4;
             dynamic_simulation_B.kstr++) {
          dynamic_simulation_B.loop_ub = dynamic_simulation_B.kstr << 2;
          dynamic_simulation_B.n_l = dynamic_simulation_B.b_kstr_n +
            dynamic_simulation_B.loop_ub;
          dynamic_simulation_B.T[dynamic_simulation_B.n_l] = 0.0;
          dynamic_simulation_B.T[dynamic_simulation_B.n_l] +=
            dynamic_simulation_B.T1_b[dynamic_simulation_B.loop_ub] *
            dynamic_simulation_B.Tdh[dynamic_simulation_B.b_kstr_n];
          dynamic_simulation_B.T[dynamic_simulation_B.n_l] +=
            dynamic_simulation_B.T1_b[dynamic_simulation_B.loop_ub + 1] *
            dynamic_simulation_B.Tdh[dynamic_simulation_B.b_kstr_n + 4];
          dynamic_simulation_B.T[dynamic_simulation_B.n_l] +=
            dynamic_simulation_B.T1_b[dynamic_simulation_B.loop_ub + 2] *
            dynamic_simulation_B.Tdh[dynamic_simulation_B.b_kstr_n + 8];
          dynamic_simulation_B.T[dynamic_simulation_B.n_l] +=
            dynamic_simulation_B.T1_b[dynamic_simulation_B.loop_ub + 3] *
            dynamic_simulation_B.Tdh[dynamic_simulation_B.b_kstr_n + 12];
        }
      }

      dynamic_simulation_B.endeffectorIndex = obj->
        PositionDoFMap[dynamic_simulation_B.b_i_o];
      dynamic_simulation_B.idx_idx_1 = obj->
        PositionDoFMap[dynamic_simulation_B.b_i_o + 8];
      dynamic_simulation_B.R_g[0] = 0.0;
      dynamic_simulation_B.R_g[3] = -dynamic_simulation_B.T[14];
      dynamic_simulation_B.R_g[6] = dynamic_simulation_B.T[13];
      dynamic_simulation_B.R_g[1] = dynamic_simulation_B.T[14];
      dynamic_simulation_B.R_g[4] = 0.0;
      dynamic_simulation_B.R_g[7] = -dynamic_simulation_B.T[12];
      dynamic_simulation_B.R_g[2] = -dynamic_simulation_B.T[13];
      dynamic_simulation_B.R_g[5] = dynamic_simulation_B.T[12];
      dynamic_simulation_B.R_g[8] = 0.0;
      for (dynamic_simulation_B.b_kstr_n = 0; dynamic_simulation_B.b_kstr_n < 3;
           dynamic_simulation_B.b_kstr_n++) {
        for (dynamic_simulation_B.kstr = 0; dynamic_simulation_B.kstr < 3;
             dynamic_simulation_B.kstr++) {
          dynamic_simulation_B.loop_ub = dynamic_simulation_B.b_kstr_n + 3 *
            dynamic_simulation_B.kstr;
          dynamic_simulation_B.R_l[dynamic_simulation_B.loop_ub] = 0.0;
          dynamic_simulation_B.n_l = dynamic_simulation_B.kstr << 2;
          dynamic_simulation_B.R_l[dynamic_simulation_B.loop_ub] +=
            dynamic_simulation_B.T[dynamic_simulation_B.n_l] *
            dynamic_simulation_B.R_g[dynamic_simulation_B.b_kstr_n];
          dynamic_simulation_B.R_l[dynamic_simulation_B.loop_ub] +=
            dynamic_simulation_B.T[dynamic_simulation_B.n_l + 1] *
            dynamic_simulation_B.R_g[dynamic_simulation_B.b_kstr_n + 3];
          dynamic_simulation_B.R_l[dynamic_simulation_B.loop_ub] +=
            dynamic_simulation_B.T[dynamic_simulation_B.n_l + 2] *
            dynamic_simulation_B.R_g[dynamic_simulation_B.b_kstr_n + 6];
          dynamic_simulation_B.X[dynamic_simulation_B.kstr + 6 *
            dynamic_simulation_B.b_kstr_n] = dynamic_simulation_B.T
            [(dynamic_simulation_B.b_kstr_n << 2) + dynamic_simulation_B.kstr];
          dynamic_simulation_B.X[dynamic_simulation_B.kstr + 6 *
            (dynamic_simulation_B.b_kstr_n + 3)] = 0.0;
        }
      }

      for (dynamic_simulation_B.b_kstr_n = 0; dynamic_simulation_B.b_kstr_n < 3;
           dynamic_simulation_B.b_kstr_n++) {
        dynamic_simulation_B.X[6 * dynamic_simulation_B.b_kstr_n + 3] =
          dynamic_simulation_B.R_l[3 * dynamic_simulation_B.b_kstr_n];
        dynamic_simulation_B.kstr = dynamic_simulation_B.b_kstr_n << 2;
        dynamic_simulation_B.loop_ub = 6 * (dynamic_simulation_B.b_kstr_n + 3);
        dynamic_simulation_B.X[dynamic_simulation_B.loop_ub + 3] =
          dynamic_simulation_B.T[dynamic_simulation_B.kstr];
        dynamic_simulation_B.X[6 * dynamic_simulation_B.b_kstr_n + 4] =
          dynamic_simulation_B.R_l[3 * dynamic_simulation_B.b_kstr_n + 1];
        dynamic_simulation_B.X[dynamic_simulation_B.loop_ub + 4] =
          dynamic_simulation_B.T[dynamic_simulation_B.kstr + 1];
        dynamic_simulation_B.X[6 * dynamic_simulation_B.b_kstr_n + 5] =
          dynamic_simulation_B.R_l[3 * dynamic_simulation_B.b_kstr_n + 2];
        dynamic_simulation_B.X[dynamic_simulation_B.loop_ub + 5] =
          dynamic_simulation_B.T[dynamic_simulation_B.kstr + 2];
      }

      dynamic_simulation_B.b_kstr_n = b->size[0] * b->size[1];
      b->size[0] = 6;
      b->size[1] = body->JointInternal.MotionSubspace->size[1];
      dynami_emxEnsureCapacity_real_T(b, dynamic_simulation_B.b_kstr_n);
      dynamic_simulation_B.loop_ub = body->JointInternal.MotionSubspace->size[0]
        * body->JointInternal.MotionSubspace->size[1] - 1;
      for (dynamic_simulation_B.b_kstr_n = 0; dynamic_simulation_B.b_kstr_n <=
           dynamic_simulation_B.loop_ub; dynamic_simulation_B.b_kstr_n++) {
        b->data[dynamic_simulation_B.b_kstr_n] =
          body->JointInternal.MotionSubspace->data[dynamic_simulation_B.b_kstr_n];
      }

      dynamic_simulation_B.n_l = b->size[1] - 1;
      dynamic_simulation_B.b_kstr_n = JacSlice->size[0] * JacSlice->size[1];
      JacSlice->size[0] = 6;
      JacSlice->size[1] = b->size[1];
      dynami_emxEnsureCapacity_real_T(JacSlice, dynamic_simulation_B.b_kstr_n);
      for (dynamic_simulation_B.b_kstr_n = 0; dynamic_simulation_B.b_kstr_n <=
           dynamic_simulation_B.n_l; dynamic_simulation_B.b_kstr_n++) {
        dynamic_simulation_B.coffset_tmp = dynamic_simulation_B.b_kstr_n * 6 - 1;
        for (dynamic_simulation_B.kstr = 0; dynamic_simulation_B.kstr < 6;
             dynamic_simulation_B.kstr++) {
          dynamic_simulation_B.s_f = 0.0;
          for (dynamic_simulation_B.loop_ub = 0; dynamic_simulation_B.loop_ub <
               6; dynamic_simulation_B.loop_ub++) {
            dynamic_simulation_B.s_f +=
              dynamic_simulation_B.X[dynamic_simulation_B.loop_ub * 6 +
              dynamic_simulation_B.kstr] * b->data
              [(dynamic_simulation_B.coffset_tmp + dynamic_simulation_B.loop_ub)
              + 1];
          }

          JacSlice->data[(dynamic_simulation_B.coffset_tmp +
                          dynamic_simulation_B.kstr) + 1] =
            dynamic_simulation_B.s_f;
        }
      }

      if (dynamic_simulation_B.endeffectorIndex > dynamic_simulation_B.idx_idx_1)
      {
        dynamic_simulation_B.n_l = 0;
      } else {
        dynamic_simulation_B.n_l = static_cast<int32_T>
          (dynamic_simulation_B.endeffectorIndex) - 1;
      }

      dynamic_simulation_B.loop_ub = JacSlice->size[1];
      for (dynamic_simulation_B.b_kstr_n = 0; dynamic_simulation_B.b_kstr_n <
           dynamic_simulation_B.loop_ub; dynamic_simulation_B.b_kstr_n++) {
        for (dynamic_simulation_B.kstr = 0; dynamic_simulation_B.kstr < 6;
             dynamic_simulation_B.kstr++) {
          Jac->data[dynamic_simulation_B.kstr + 6 * (dynamic_simulation_B.n_l +
            dynamic_simulation_B.b_kstr_n)] = JacSlice->data[6 *
            dynamic_simulation_B.b_kstr_n + dynamic_simulation_B.kstr];
        }
      }
    }
  }

  dynamic_simulati_emxFree_char_T(&bname);
  dynamic_simulati_emxFree_real_T(&JacSlice);
  dynamic_sim_emxFree_f_cell_wrap(&Ttree);
  for (dynamic_simulation_B.b_kstr_n = 0; dynamic_simulation_B.b_kstr_n < 3;
       dynamic_simulation_B.b_kstr_n++) {
    dynamic_simulation_B.b_i_o = dynamic_simulation_B.b_kstr_n << 2;
    dynamic_simulation_B.X[6 * dynamic_simulation_B.b_kstr_n] =
      dynamic_simulation_B.T2_c[dynamic_simulation_B.b_i_o];
    dynamic_simulation_B.kstr = 6 * (dynamic_simulation_B.b_kstr_n + 3);
    dynamic_simulation_B.X[dynamic_simulation_B.kstr] = 0.0;
    dynamic_simulation_B.X[6 * dynamic_simulation_B.b_kstr_n + 3] = 0.0;
    dynamic_simulation_B.X[dynamic_simulation_B.kstr + 3] =
      dynamic_simulation_B.T2_c[dynamic_simulation_B.b_i_o];
    dynamic_simulation_B.endeffectorIndex =
      dynamic_simulation_B.T2_c[dynamic_simulation_B.b_i_o + 1];
    dynamic_simulation_B.X[6 * dynamic_simulation_B.b_kstr_n + 1] =
      dynamic_simulation_B.endeffectorIndex;
    dynamic_simulation_B.X[dynamic_simulation_B.kstr + 1] = 0.0;
    dynamic_simulation_B.X[6 * dynamic_simulation_B.b_kstr_n + 4] = 0.0;
    dynamic_simulation_B.X[dynamic_simulation_B.kstr + 4] =
      dynamic_simulation_B.endeffectorIndex;
    dynamic_simulation_B.endeffectorIndex =
      dynamic_simulation_B.T2_c[dynamic_simulation_B.b_i_o + 2];
    dynamic_simulation_B.X[6 * dynamic_simulation_B.b_kstr_n + 2] =
      dynamic_simulation_B.endeffectorIndex;
    dynamic_simulation_B.X[dynamic_simulation_B.kstr + 2] = 0.0;
    dynamic_simulation_B.X[6 * dynamic_simulation_B.b_kstr_n + 5] = 0.0;
    dynamic_simulation_B.X[dynamic_simulation_B.kstr + 5] =
      dynamic_simulation_B.endeffectorIndex;
  }

  dynamic_simulation_B.n_l = Jac->size[1];
  dynamic_simulation_B.b_kstr_n = b->size[0] * b->size[1];
  b->size[0] = 6;
  b->size[1] = Jac->size[1];
  dynami_emxEnsureCapacity_real_T(b, dynamic_simulation_B.b_kstr_n);
  dynamic_simulation_B.loop_ub = Jac->size[0] * Jac->size[1] - 1;
  for (dynamic_simulation_B.b_kstr_n = 0; dynamic_simulation_B.b_kstr_n <=
       dynamic_simulation_B.loop_ub; dynamic_simulation_B.b_kstr_n++) {
    b->data[dynamic_simulation_B.b_kstr_n] = Jac->
      data[dynamic_simulation_B.b_kstr_n];
  }

  dynamic_simulation_B.b_kstr_n = Jac->size[0] * Jac->size[1];
  Jac->size[0] = 6;
  Jac->size[1] = dynamic_simulation_B.n_l;
  dynami_emxEnsureCapacity_real_T(Jac, dynamic_simulation_B.b_kstr_n);
  for (dynamic_simulation_B.b_kstr_n = 0; dynamic_simulation_B.b_kstr_n <
       dynamic_simulation_B.n_l; dynamic_simulation_B.b_kstr_n++) {
    dynamic_simulation_B.coffset_tmp = dynamic_simulation_B.b_kstr_n * 6 - 1;
    for (dynamic_simulation_B.b_i_o = 0; dynamic_simulation_B.b_i_o < 6;
         dynamic_simulation_B.b_i_o++) {
      dynamic_simulation_B.s_f = 0.0;
      for (dynamic_simulation_B.loop_ub = 0; dynamic_simulation_B.loop_ub < 6;
           dynamic_simulation_B.loop_ub++) {
        dynamic_simulation_B.s_f +=
          dynamic_simulation_B.X[dynamic_simulation_B.loop_ub * 6 +
          dynamic_simulation_B.b_i_o] * b->data
          [(dynamic_simulation_B.coffset_tmp + dynamic_simulation_B.loop_ub) + 1];
      }

      Jac->data[(dynamic_simulation_B.coffset_tmp + dynamic_simulation_B.b_i_o)
        + 1] = dynamic_simulation_B.s_f;
    }
  }

  dynamic_simulati_emxFree_real_T(&b);
}

static void rigidBodyJoint_get_JointAxis_k(const c_rigidBodyJoint_dynamic_si_k_T
  *obj, real_T ax[3])
{
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  boolean_T guard1 = false;
  int32_T exitg1;
  for (dynamic_simulation_B.b_kstr_pm = 0; dynamic_simulation_B.b_kstr_pm < 8;
       dynamic_simulation_B.b_kstr_pm++) {
    dynamic_simulation_B.b_ip[dynamic_simulation_B.b_kstr_pm] =
      tmp[dynamic_simulation_B.b_kstr_pm];
  }

  dynamic_simulation_B.b_bool_dk = false;
  if (obj->Type->size[1] == 8) {
    dynamic_simulation_B.b_kstr_pm = 1;
    do {
      exitg1 = 0;
      if (dynamic_simulation_B.b_kstr_pm - 1 < 8) {
        dynamic_simulation_B.kstr_f = dynamic_simulation_B.b_kstr_pm - 1;
        if (obj->Type->data[dynamic_simulation_B.kstr_f] !=
            dynamic_simulation_B.b_ip[dynamic_simulation_B.kstr_f]) {
          exitg1 = 1;
        } else {
          dynamic_simulation_B.b_kstr_pm++;
        }
      } else {
        dynamic_simulation_B.b_bool_dk = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (dynamic_simulation_B.b_bool_dk) {
    guard1 = true;
  } else {
    for (dynamic_simulation_B.b_kstr_pm = 0; dynamic_simulation_B.b_kstr_pm < 9;
         dynamic_simulation_B.b_kstr_pm++) {
      dynamic_simulation_B.b_a[dynamic_simulation_B.b_kstr_pm] =
        tmp_0[dynamic_simulation_B.b_kstr_pm];
    }

    dynamic_simulation_B.b_bool_dk = false;
    if (obj->Type->size[1] == 9) {
      dynamic_simulation_B.b_kstr_pm = 1;
      do {
        exitg1 = 0;
        if (dynamic_simulation_B.b_kstr_pm - 1 < 9) {
          dynamic_simulation_B.kstr_f = dynamic_simulation_B.b_kstr_pm - 1;
          if (obj->Type->data[dynamic_simulation_B.kstr_f] !=
              dynamic_simulation_B.b_a[dynamic_simulation_B.kstr_f]) {
            exitg1 = 1;
          } else {
            dynamic_simulation_B.b_kstr_pm++;
          }
        } else {
          dynamic_simulation_B.b_bool_dk = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynamic_simulation_B.b_bool_dk) {
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

static void RigidBodyTree_forwardKinemati_k(p_robotics_manip_internal_R_k_T *obj,
  const real_T qvec[6], emxArray_f_cell_wrap_dynamic__T *Ttree)
{
  n_robotics_manip_internal_R_k_T *body;
  emxArray_char_T_dynamic_simul_T *switch_expression;
  static const int8_T tmp[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  dynamic_simulation_B.n_h = obj->NumBodies;
  for (dynamic_simulation_B.b_kstr_na = 0; dynamic_simulation_B.b_kstr_na < 16;
       dynamic_simulation_B.b_kstr_na++) {
    dynamic_simulation_B.c_f1_m[dynamic_simulation_B.b_kstr_na] =
      tmp[dynamic_simulation_B.b_kstr_na];
  }

  dynamic_simulation_B.b_kstr_na = Ttree->size[0] * Ttree->size[1];
  Ttree->size[0] = 1;
  dynamic_simulation_B.e_g = static_cast<int32_T>(dynamic_simulation_B.n_h);
  Ttree->size[1] = dynamic_simulation_B.e_g;
  d_emxEnsureCapacity_f_cell_wrap(Ttree, dynamic_simulation_B.b_kstr_na);
  if (dynamic_simulation_B.e_g != 0) {
    dynamic_simulation_B.ntilecols_n = dynamic_simulation_B.e_g - 1;
    if (0 <= dynamic_simulation_B.ntilecols_n) {
      memcpy(&dynamic_simulation_B.expl_temp_c.f1[0],
             &dynamic_simulation_B.c_f1_m[0], sizeof(real_T) << 4U);
    }

    for (dynamic_simulation_B.b_jtilecol_d = 0;
         dynamic_simulation_B.b_jtilecol_d <= dynamic_simulation_B.ntilecols_n;
         dynamic_simulation_B.b_jtilecol_d++) {
      Ttree->data[dynamic_simulation_B.b_jtilecol_d] =
        dynamic_simulation_B.expl_temp_c;
    }
  }

  dynamic_simulation_B.k_c = 1.0;
  dynamic_simulation_B.ntilecols_n = static_cast<int32_T>
    (dynamic_simulation_B.n_h) - 1;
  dynamic_simulati_emxInit_char_T(&switch_expression, 2);
  if (0 <= dynamic_simulation_B.ntilecols_n) {
    for (dynamic_simulation_B.b_kstr_na = 0; dynamic_simulation_B.b_kstr_na < 5;
         dynamic_simulation_B.b_kstr_na++) {
      dynamic_simulation_B.b_d[dynamic_simulation_B.b_kstr_na] =
        tmp_0[dynamic_simulation_B.b_kstr_na];
    }
  }

  for (dynamic_simulation_B.b_jtilecol_d = 0; dynamic_simulation_B.b_jtilecol_d <=
       dynamic_simulation_B.ntilecols_n; dynamic_simulation_B.b_jtilecol_d++) {
    body = obj->Bodies[dynamic_simulation_B.b_jtilecol_d];
    dynamic_simulation_B.n_h = body->JointInternal.PositionNumber;
    dynamic_simulation_B.n_h += dynamic_simulation_B.k_c;
    if (dynamic_simulation_B.k_c > dynamic_simulation_B.n_h - 1.0) {
      dynamic_simulation_B.e_g = 0;
      dynamic_simulation_B.d_a = 0;
    } else {
      dynamic_simulation_B.e_g = static_cast<int32_T>(dynamic_simulation_B.k_c)
        - 1;
      dynamic_simulation_B.d_a = static_cast<int32_T>(dynamic_simulation_B.n_h -
        1.0);
    }

    dynamic_simulation_B.b_kstr_na = switch_expression->size[0] *
      switch_expression->size[1];
    switch_expression->size[0] = 1;
    switch_expression->size[1] = body->JointInternal.Type->size[1];
    dynami_emxEnsureCapacity_char_T(switch_expression,
      dynamic_simulation_B.b_kstr_na);
    dynamic_simulation_B.loop_ub_c = body->JointInternal.Type->size[0] *
      body->JointInternal.Type->size[1] - 1;
    for (dynamic_simulation_B.b_kstr_na = 0; dynamic_simulation_B.b_kstr_na <=
         dynamic_simulation_B.loop_ub_c; dynamic_simulation_B.b_kstr_na++) {
      switch_expression->data[dynamic_simulation_B.b_kstr_na] =
        body->JointInternal.Type->data[dynamic_simulation_B.b_kstr_na];
    }

    dynamic_simulation_B.b_bool_p = false;
    if (switch_expression->size[1] == 5) {
      dynamic_simulation_B.b_kstr_na = 1;
      do {
        exitg1 = 0;
        if (dynamic_simulation_B.b_kstr_na - 1 < 5) {
          dynamic_simulation_B.loop_ub_c = dynamic_simulation_B.b_kstr_na - 1;
          if (switch_expression->data[dynamic_simulation_B.loop_ub_c] !=
              dynamic_simulation_B.b_d[dynamic_simulation_B.loop_ub_c]) {
            exitg1 = 1;
          } else {
            dynamic_simulation_B.b_kstr_na++;
          }
        } else {
          dynamic_simulation_B.b_bool_p = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynamic_simulation_B.b_bool_p) {
      dynamic_simulation_B.b_kstr_na = 0;
    } else {
      for (dynamic_simulation_B.b_kstr_na = 0; dynamic_simulation_B.b_kstr_na <
           8; dynamic_simulation_B.b_kstr_na++) {
        dynamic_simulation_B.b_o2[dynamic_simulation_B.b_kstr_na] =
          tmp_1[dynamic_simulation_B.b_kstr_na];
      }

      dynamic_simulation_B.b_bool_p = false;
      if (switch_expression->size[1] == 8) {
        dynamic_simulation_B.b_kstr_na = 1;
        do {
          exitg1 = 0;
          if (dynamic_simulation_B.b_kstr_na - 1 < 8) {
            dynamic_simulation_B.loop_ub_c = dynamic_simulation_B.b_kstr_na - 1;
            if (switch_expression->data[dynamic_simulation_B.loop_ub_c] !=
                dynamic_simulation_B.b_o2[dynamic_simulation_B.loop_ub_c]) {
              exitg1 = 1;
            } else {
              dynamic_simulation_B.b_kstr_na++;
            }
          } else {
            dynamic_simulation_B.b_bool_p = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (dynamic_simulation_B.b_bool_p) {
        dynamic_simulation_B.b_kstr_na = 1;
      } else {
        dynamic_simulation_B.b_kstr_na = -1;
      }
    }

    switch (dynamic_simulation_B.b_kstr_na) {
     case 0:
      memset(&dynamic_simulation_B.c_f1_m[0], 0, sizeof(real_T) << 4U);
      dynamic_simulation_B.c_f1_m[0] = 1.0;
      dynamic_simulation_B.c_f1_m[5] = 1.0;
      dynamic_simulation_B.c_f1_m[10] = 1.0;
      dynamic_simulation_B.c_f1_m[15] = 1.0;
      break;

     case 1:
      rigidBodyJoint_get_JointAxis_k(&body->JointInternal,
        dynamic_simulation_B.v_m3);
      dynamic_simulation_B.d_a -= dynamic_simulation_B.e_g;
      for (dynamic_simulation_B.b_kstr_na = 0; dynamic_simulation_B.b_kstr_na <
           dynamic_simulation_B.d_a; dynamic_simulation_B.b_kstr_na++) {
        dynamic_simulation_B.e_data_n[dynamic_simulation_B.b_kstr_na] =
          dynamic_simulation_B.e_g + dynamic_simulation_B.b_kstr_na;
      }

      dynamic_simulation_B.result_data_i[0] = dynamic_simulation_B.v_m3[0];
      dynamic_simulation_B.result_data_i[1] = dynamic_simulation_B.v_m3[1];
      dynamic_simulation_B.result_data_i[2] = dynamic_simulation_B.v_m3[2];
      if (0 <= (dynamic_simulation_B.d_a != 0) - 1) {
        dynamic_simulation_B.result_data_i[3] =
          qvec[dynamic_simulation_B.e_data_n[0]];
      }

      dynamic_simulation_B.k_c = 1.0 / sqrt((dynamic_simulation_B.result_data_i
        [0] * dynamic_simulation_B.result_data_i[0] +
        dynamic_simulation_B.result_data_i[1] *
        dynamic_simulation_B.result_data_i[1]) +
        dynamic_simulation_B.result_data_i[2] *
        dynamic_simulation_B.result_data_i[2]);
      dynamic_simulation_B.v_m3[0] = dynamic_simulation_B.result_data_i[0] *
        dynamic_simulation_B.k_c;
      dynamic_simulation_B.v_m3[1] = dynamic_simulation_B.result_data_i[1] *
        dynamic_simulation_B.k_c;
      dynamic_simulation_B.v_m3[2] = dynamic_simulation_B.result_data_i[2] *
        dynamic_simulation_B.k_c;
      dynamic_simulation_B.k_c = cos(dynamic_simulation_B.result_data_i[3]);
      dynamic_simulation_B.sth_k = sin(dynamic_simulation_B.result_data_i[3]);
      dynamic_simulation_B.tempR_h[0] = dynamic_simulation_B.v_m3[0] *
        dynamic_simulation_B.v_m3[0] * (1.0 - dynamic_simulation_B.k_c) +
        dynamic_simulation_B.k_c;
      dynamic_simulation_B.tempR_tmp_p = dynamic_simulation_B.v_m3[1] *
        dynamic_simulation_B.v_m3[0] * (1.0 - dynamic_simulation_B.k_c);
      dynamic_simulation_B.tempR_tmp_px = dynamic_simulation_B.v_m3[2] *
        dynamic_simulation_B.sth_k;
      dynamic_simulation_B.tempR_h[1] = dynamic_simulation_B.tempR_tmp_p -
        dynamic_simulation_B.tempR_tmp_px;
      dynamic_simulation_B.tempR_tmp_p4 = dynamic_simulation_B.v_m3[2] *
        dynamic_simulation_B.v_m3[0] * (1.0 - dynamic_simulation_B.k_c);
      dynamic_simulation_B.tempR_tmp_a = dynamic_simulation_B.v_m3[1] *
        dynamic_simulation_B.sth_k;
      dynamic_simulation_B.tempR_h[2] = dynamic_simulation_B.tempR_tmp_p4 +
        dynamic_simulation_B.tempR_tmp_a;
      dynamic_simulation_B.tempR_h[3] = dynamic_simulation_B.tempR_tmp_p +
        dynamic_simulation_B.tempR_tmp_px;
      dynamic_simulation_B.tempR_h[4] = dynamic_simulation_B.v_m3[1] *
        dynamic_simulation_B.v_m3[1] * (1.0 - dynamic_simulation_B.k_c) +
        dynamic_simulation_B.k_c;
      dynamic_simulation_B.tempR_tmp_p = dynamic_simulation_B.v_m3[2] *
        dynamic_simulation_B.v_m3[1] * (1.0 - dynamic_simulation_B.k_c);
      dynamic_simulation_B.tempR_tmp_px = dynamic_simulation_B.v_m3[0] *
        dynamic_simulation_B.sth_k;
      dynamic_simulation_B.tempR_h[5] = dynamic_simulation_B.tempR_tmp_p -
        dynamic_simulation_B.tempR_tmp_px;
      dynamic_simulation_B.tempR_h[6] = dynamic_simulation_B.tempR_tmp_p4 -
        dynamic_simulation_B.tempR_tmp_a;
      dynamic_simulation_B.tempR_h[7] = dynamic_simulation_B.tempR_tmp_p +
        dynamic_simulation_B.tempR_tmp_px;
      dynamic_simulation_B.tempR_h[8] = dynamic_simulation_B.v_m3[2] *
        dynamic_simulation_B.v_m3[2] * (1.0 - dynamic_simulation_B.k_c) +
        dynamic_simulation_B.k_c;
      for (dynamic_simulation_B.b_kstr_na = 0; dynamic_simulation_B.b_kstr_na <
           3; dynamic_simulation_B.b_kstr_na++) {
        dynamic_simulation_B.e_g = dynamic_simulation_B.b_kstr_na + 1;
        dynamic_simulation_B.R_ln[dynamic_simulation_B.e_g - 1] =
          dynamic_simulation_B.tempR_h[(dynamic_simulation_B.e_g - 1) * 3];
        dynamic_simulation_B.e_g = dynamic_simulation_B.b_kstr_na + 1;
        dynamic_simulation_B.R_ln[dynamic_simulation_B.e_g + 2] =
          dynamic_simulation_B.tempR_h[(dynamic_simulation_B.e_g - 1) * 3 + 1];
        dynamic_simulation_B.e_g = dynamic_simulation_B.b_kstr_na + 1;
        dynamic_simulation_B.R_ln[dynamic_simulation_B.e_g + 5] =
          dynamic_simulation_B.tempR_h[(dynamic_simulation_B.e_g - 1) * 3 + 2];
      }

      memset(&dynamic_simulation_B.c_f1_m[0], 0, sizeof(real_T) << 4U);
      for (dynamic_simulation_B.b_kstr_na = 0; dynamic_simulation_B.b_kstr_na <
           3; dynamic_simulation_B.b_kstr_na++) {
        dynamic_simulation_B.d_a = dynamic_simulation_B.b_kstr_na << 2;
        dynamic_simulation_B.c_f1_m[dynamic_simulation_B.d_a] =
          dynamic_simulation_B.R_ln[3 * dynamic_simulation_B.b_kstr_na];
        dynamic_simulation_B.c_f1_m[dynamic_simulation_B.d_a + 1] =
          dynamic_simulation_B.R_ln[3 * dynamic_simulation_B.b_kstr_na + 1];
        dynamic_simulation_B.c_f1_m[dynamic_simulation_B.d_a + 2] =
          dynamic_simulation_B.R_ln[3 * dynamic_simulation_B.b_kstr_na + 2];
      }

      dynamic_simulation_B.c_f1_m[15] = 1.0;
      break;

     default:
      rigidBodyJoint_get_JointAxis_k(&body->JointInternal,
        dynamic_simulation_B.v_m3);
      memset(&dynamic_simulation_B.tempR_h[0], 0, 9U * sizeof(real_T));
      dynamic_simulation_B.tempR_h[0] = 1.0;
      dynamic_simulation_B.tempR_h[4] = 1.0;
      dynamic_simulation_B.tempR_h[8] = 1.0;
      for (dynamic_simulation_B.b_kstr_na = 0; dynamic_simulation_B.b_kstr_na <
           3; dynamic_simulation_B.b_kstr_na++) {
        dynamic_simulation_B.d_a = dynamic_simulation_B.b_kstr_na << 2;
        dynamic_simulation_B.c_f1_m[dynamic_simulation_B.d_a] =
          dynamic_simulation_B.tempR_h[3 * dynamic_simulation_B.b_kstr_na];
        dynamic_simulation_B.c_f1_m[dynamic_simulation_B.d_a + 1] =
          dynamic_simulation_B.tempR_h[3 * dynamic_simulation_B.b_kstr_na + 1];
        dynamic_simulation_B.c_f1_m[dynamic_simulation_B.d_a + 2] =
          dynamic_simulation_B.tempR_h[3 * dynamic_simulation_B.b_kstr_na + 2];
        dynamic_simulation_B.c_f1_m[dynamic_simulation_B.b_kstr_na + 12] =
          dynamic_simulation_B.v_m3[dynamic_simulation_B.b_kstr_na] *
          qvec[dynamic_simulation_B.e_g];
      }

      dynamic_simulation_B.c_f1_m[3] = 0.0;
      dynamic_simulation_B.c_f1_m[7] = 0.0;
      dynamic_simulation_B.c_f1_m[11] = 0.0;
      dynamic_simulation_B.c_f1_m[15] = 1.0;
      break;
    }

    for (dynamic_simulation_B.b_kstr_na = 0; dynamic_simulation_B.b_kstr_na < 16;
         dynamic_simulation_B.b_kstr_na++) {
      dynamic_simulation_B.a_n[dynamic_simulation_B.b_kstr_na] =
        body->
        JointInternal.JointToParentTransform[dynamic_simulation_B.b_kstr_na];
    }

    for (dynamic_simulation_B.b_kstr_na = 0; dynamic_simulation_B.b_kstr_na < 16;
         dynamic_simulation_B.b_kstr_na++) {
      dynamic_simulation_B.b_p[dynamic_simulation_B.b_kstr_na] =
        body->JointInternal.ChildToJointTransform[dynamic_simulation_B.b_kstr_na];
    }

    for (dynamic_simulation_B.b_kstr_na = 0; dynamic_simulation_B.b_kstr_na < 4;
         dynamic_simulation_B.b_kstr_na++) {
      for (dynamic_simulation_B.e_g = 0; dynamic_simulation_B.e_g < 4;
           dynamic_simulation_B.e_g++) {
        dynamic_simulation_B.d_a = dynamic_simulation_B.e_g << 2;
        dynamic_simulation_B.loop_ub_c = dynamic_simulation_B.b_kstr_na +
          dynamic_simulation_B.d_a;
        dynamic_simulation_B.a_l[dynamic_simulation_B.loop_ub_c] = 0.0;
        dynamic_simulation_B.a_l[dynamic_simulation_B.loop_ub_c] +=
          dynamic_simulation_B.c_f1_m[dynamic_simulation_B.d_a] *
          dynamic_simulation_B.a_n[dynamic_simulation_B.b_kstr_na];
        dynamic_simulation_B.a_l[dynamic_simulation_B.loop_ub_c] +=
          dynamic_simulation_B.c_f1_m[dynamic_simulation_B.d_a + 1] *
          dynamic_simulation_B.a_n[dynamic_simulation_B.b_kstr_na + 4];
        dynamic_simulation_B.a_l[dynamic_simulation_B.loop_ub_c] +=
          dynamic_simulation_B.c_f1_m[dynamic_simulation_B.d_a + 2] *
          dynamic_simulation_B.a_n[dynamic_simulation_B.b_kstr_na + 8];
        dynamic_simulation_B.a_l[dynamic_simulation_B.loop_ub_c] +=
          dynamic_simulation_B.c_f1_m[dynamic_simulation_B.d_a + 3] *
          dynamic_simulation_B.a_n[dynamic_simulation_B.b_kstr_na + 12];
      }

      for (dynamic_simulation_B.e_g = 0; dynamic_simulation_B.e_g < 4;
           dynamic_simulation_B.e_g++) {
        dynamic_simulation_B.d_a = dynamic_simulation_B.e_g << 2;
        dynamic_simulation_B.loop_ub_c = dynamic_simulation_B.b_kstr_na +
          dynamic_simulation_B.d_a;
        Ttree->data[dynamic_simulation_B.b_jtilecol_d]
          .f1[dynamic_simulation_B.loop_ub_c] = 0.0;
        Ttree->data[dynamic_simulation_B.b_jtilecol_d]
          .f1[dynamic_simulation_B.loop_ub_c] +=
          dynamic_simulation_B.b_p[dynamic_simulation_B.d_a] *
          dynamic_simulation_B.a_l[dynamic_simulation_B.b_kstr_na];
        Ttree->data[dynamic_simulation_B.b_jtilecol_d]
          .f1[dynamic_simulation_B.loop_ub_c] +=
          dynamic_simulation_B.b_p[dynamic_simulation_B.d_a + 1] *
          dynamic_simulation_B.a_l[dynamic_simulation_B.b_kstr_na + 4];
        Ttree->data[dynamic_simulation_B.b_jtilecol_d]
          .f1[dynamic_simulation_B.loop_ub_c] +=
          dynamic_simulation_B.b_p[dynamic_simulation_B.d_a + 2] *
          dynamic_simulation_B.a_l[dynamic_simulation_B.b_kstr_na + 8];
        Ttree->data[dynamic_simulation_B.b_jtilecol_d]
          .f1[dynamic_simulation_B.loop_ub_c] +=
          dynamic_simulation_B.b_p[dynamic_simulation_B.d_a + 3] *
          dynamic_simulation_B.a_l[dynamic_simulation_B.b_kstr_na + 12];
      }
    }

    dynamic_simulation_B.k_c = dynamic_simulation_B.n_h;
    if (body->ParentIndex > 0.0) {
      for (dynamic_simulation_B.b_kstr_na = 0; dynamic_simulation_B.b_kstr_na <
           16; dynamic_simulation_B.b_kstr_na++) {
        dynamic_simulation_B.a_n[dynamic_simulation_B.b_kstr_na] = Ttree->data[
          static_cast<int32_T>(body->ParentIndex) - 1]
          .f1[dynamic_simulation_B.b_kstr_na];
      }

      for (dynamic_simulation_B.b_kstr_na = 0; dynamic_simulation_B.b_kstr_na <
           4; dynamic_simulation_B.b_kstr_na++) {
        for (dynamic_simulation_B.e_g = 0; dynamic_simulation_B.e_g < 4;
             dynamic_simulation_B.e_g++) {
          dynamic_simulation_B.d_a = dynamic_simulation_B.e_g << 2;
          dynamic_simulation_B.loop_ub_c = dynamic_simulation_B.b_kstr_na +
            dynamic_simulation_B.d_a;
          dynamic_simulation_B.a_l[dynamic_simulation_B.loop_ub_c] = 0.0;
          dynamic_simulation_B.a_l[dynamic_simulation_B.loop_ub_c] +=
            Ttree->data[dynamic_simulation_B.b_jtilecol_d]
            .f1[dynamic_simulation_B.d_a] *
            dynamic_simulation_B.a_n[dynamic_simulation_B.b_kstr_na];
          dynamic_simulation_B.a_l[dynamic_simulation_B.loop_ub_c] +=
            Ttree->data[dynamic_simulation_B.b_jtilecol_d]
            .f1[dynamic_simulation_B.d_a + 1] *
            dynamic_simulation_B.a_n[dynamic_simulation_B.b_kstr_na + 4];
          dynamic_simulation_B.a_l[dynamic_simulation_B.loop_ub_c] +=
            Ttree->data[dynamic_simulation_B.b_jtilecol_d]
            .f1[dynamic_simulation_B.d_a + 2] *
            dynamic_simulation_B.a_n[dynamic_simulation_B.b_kstr_na + 8];
          dynamic_simulation_B.a_l[dynamic_simulation_B.loop_ub_c] +=
            Ttree->data[dynamic_simulation_B.b_jtilecol_d]
            .f1[dynamic_simulation_B.d_a + 3] *
            dynamic_simulation_B.a_n[dynamic_simulation_B.b_kstr_na + 12];
        }
      }

      for (dynamic_simulation_B.b_kstr_na = 0; dynamic_simulation_B.b_kstr_na <
           16; dynamic_simulation_B.b_kstr_na++) {
        Ttree->data[dynamic_simulation_B.b_jtilecol_d]
          .f1[dynamic_simulation_B.b_kstr_na] =
          dynamic_simulation_B.a_l[dynamic_simulation_B.b_kstr_na];
      }
    }
  }

  dynamic_simulati_emxFree_char_T(&switch_expression);
}

static void dynamic_simulat_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Data[128], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_dynamic_simulation_std_msgs_MultiArrayDimension varargout_2_Layout_Dim
  [16], uint32_T *varargout_2_Layout_Dim_SL_Info_, uint32_T
  *varargout_2_Layout_Dim_SL_Inf_0)
{
  *varargout_1 = Sub_dynamic_simulation_160.getLatestMessage
    (&dynamic_simulation_B.b_varargout_2);
  memcpy(&varargout_2_Data[0], &dynamic_simulation_B.b_varargout_2.Data[0],
         sizeof(real_T) << 7U);
  *varargout_2_Data_SL_Info_Curren =
    dynamic_simulation_B.b_varargout_2.Data_SL_Info.CurrentLength;
  *varargout_2_Data_SL_Info_Receiv =
    dynamic_simulation_B.b_varargout_2.Data_SL_Info.ReceivedLength;
  *varargout_2_Layout_DataOffset =
    dynamic_simulation_B.b_varargout_2.Layout.DataOffset;
  memcpy(&varargout_2_Layout_Dim[0],
         &dynamic_simulation_B.b_varargout_2.Layout.Dim[0], sizeof
         (SL_Bus_dynamic_simulation_std_msgs_MultiArrayDimension) << 4U);
  *varargout_2_Layout_Dim_SL_Info_ =
    dynamic_simulation_B.b_varargout_2.Layout.Dim_SL_Info.CurrentLength;
  *varargout_2_Layout_Dim_SL_Inf_0 =
    dynamic_simulation_B.b_varargout_2.Layout.Dim_SL_Info.ReceivedLength;
}

static void dynamic_si_emxInit_f_cell_wrap1(emxArray_f_cell_wrap_dynami_k_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_f_cell_wrap_dynami_k_T *emxArray;
  *pEmxArray = (emxArray_f_cell_wrap_dynami_k_T *)malloc(sizeof
    (emxArray_f_cell_wrap_dynami_k_T));
  emxArray = *pEmxArray;
  emxArray->data = (f_cell_wrap_dynamic_simulat_k_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (dynamic_simulation_B.i_i3 = 0; dynamic_simulation_B.i_i3 < numDimensions;
       dynamic_simulation_B.i_i3++) {
    emxArray->size[dynamic_simulation_B.i_i3] = 0;
  }
}

static void emxEnsureCapacity_f_cell_wrap1(emxArray_f_cell_wrap_dynami_k_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  dynamic_simulation_B.newNumel_l = 1;
  for (dynamic_simulation_B.i_f1 = 0; dynamic_simulation_B.i_f1 <
       emxArray->numDimensions; dynamic_simulation_B.i_f1++) {
    dynamic_simulation_B.newNumel_l *= emxArray->size[dynamic_simulation_B.i_f1];
  }

  if (dynamic_simulation_B.newNumel_l > emxArray->allocatedSize) {
    dynamic_simulation_B.i_f1 = emxArray->allocatedSize;
    if (dynamic_simulation_B.i_f1 < 16) {
      dynamic_simulation_B.i_f1 = 16;
    }

    while (dynamic_simulation_B.i_f1 < dynamic_simulation_B.newNumel_l) {
      if (dynamic_simulation_B.i_f1 > 1073741823) {
        dynamic_simulation_B.i_f1 = MAX_int32_T;
      } else {
        dynamic_simulation_B.i_f1 <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(dynamic_simulation_B.i_f1), sizeof
                     (f_cell_wrap_dynamic_simulat_k_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(f_cell_wrap_dynamic_simulat_k_T)
             * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (f_cell_wrap_dynamic_simulat_k_T *)newData;
    emxArray->allocatedSize = dynamic_simulation_B.i_f1;
    emxArray->canFreeData = true;
  }
}

static void rigidBodyJoint_get_JointAxis_kq(const
  c_rigidBodyJoint_dynamic_s_kq_T *obj, real_T ax[3])
{
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  boolean_T guard1 = false;
  int32_T exitg1;
  for (dynamic_simulation_B.b_kstr_f = 0; dynamic_simulation_B.b_kstr_f < 8;
       dynamic_simulation_B.b_kstr_f++) {
    dynamic_simulation_B.b_l[dynamic_simulation_B.b_kstr_f] =
      tmp[dynamic_simulation_B.b_kstr_f];
  }

  dynamic_simulation_B.b_bool_n = false;
  if (obj->Type->size[1] == 8) {
    dynamic_simulation_B.b_kstr_f = 1;
    do {
      exitg1 = 0;
      if (dynamic_simulation_B.b_kstr_f - 1 < 8) {
        dynamic_simulation_B.kstr_c = dynamic_simulation_B.b_kstr_f - 1;
        if (obj->Type->data[dynamic_simulation_B.kstr_c] !=
            dynamic_simulation_B.b_l[dynamic_simulation_B.kstr_c]) {
          exitg1 = 1;
        } else {
          dynamic_simulation_B.b_kstr_f++;
        }
      } else {
        dynamic_simulation_B.b_bool_n = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (dynamic_simulation_B.b_bool_n) {
    guard1 = true;
  } else {
    for (dynamic_simulation_B.b_kstr_f = 0; dynamic_simulation_B.b_kstr_f < 9;
         dynamic_simulation_B.b_kstr_f++) {
      dynamic_simulation_B.b_p5[dynamic_simulation_B.b_kstr_f] =
        tmp_0[dynamic_simulation_B.b_kstr_f];
    }

    dynamic_simulation_B.b_bool_n = false;
    if (obj->Type->size[1] == 9) {
      dynamic_simulation_B.b_kstr_f = 1;
      do {
        exitg1 = 0;
        if (dynamic_simulation_B.b_kstr_f - 1 < 9) {
          dynamic_simulation_B.kstr_c = dynamic_simulation_B.b_kstr_f - 1;
          if (obj->Type->data[dynamic_simulation_B.kstr_c] !=
              dynamic_simulation_B.b_p5[dynamic_simulation_B.kstr_c]) {
            exitg1 = 1;
          } else {
            dynamic_simulation_B.b_kstr_f++;
          }
        } else {
          dynamic_simulation_B.b_bool_n = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynamic_simulation_B.b_bool_n) {
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

static void dynamic_simulation_cat(real_T varargin_1, real_T varargin_2, real_T
  varargin_3, real_T varargin_4, real_T varargin_5, real_T varargin_6, real_T
  varargin_7, real_T varargin_8, real_T varargin_9, real_T y[9])
{
  y[0] = varargin_1;
  y[1] = varargin_2;
  y[2] = varargin_3;
  y[3] = varargin_4;
  y[4] = varargin_5;
  y[5] = varargin_6;
  y[6] = varargin_7;
  y[7] = varargin_8;
  y[8] = varargin_9;
}

static void rigidBodyJoint_transformBodyT_k(const
  c_rigidBodyJoint_dynamic_s_kq_T *obj, const real_T q_data[], const int32_T
  *q_size, real_T T[16])
{
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  for (dynamic_simulation_B.b_kstr_m = 0; dynamic_simulation_B.b_kstr_m < 5;
       dynamic_simulation_B.b_kstr_m++) {
    dynamic_simulation_B.b_ei[dynamic_simulation_B.b_kstr_m] =
      tmp[dynamic_simulation_B.b_kstr_m];
  }

  dynamic_simulation_B.b_bool_g = false;
  if (obj->Type->size[1] == 5) {
    dynamic_simulation_B.b_kstr_m = 1;
    do {
      exitg1 = 0;
      if (dynamic_simulation_B.b_kstr_m - 1 < 5) {
        dynamic_simulation_B.kstr_o = dynamic_simulation_B.b_kstr_m - 1;
        if (obj->Type->data[dynamic_simulation_B.kstr_o] !=
            dynamic_simulation_B.b_ei[dynamic_simulation_B.kstr_o]) {
          exitg1 = 1;
        } else {
          dynamic_simulation_B.b_kstr_m++;
        }
      } else {
        dynamic_simulation_B.b_bool_g = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (dynamic_simulation_B.b_bool_g) {
    dynamic_simulation_B.b_kstr_m = 0;
  } else {
    for (dynamic_simulation_B.b_kstr_m = 0; dynamic_simulation_B.b_kstr_m < 8;
         dynamic_simulation_B.b_kstr_m++) {
      dynamic_simulation_B.b_i[dynamic_simulation_B.b_kstr_m] =
        tmp_0[dynamic_simulation_B.b_kstr_m];
    }

    dynamic_simulation_B.b_bool_g = false;
    if (obj->Type->size[1] == 8) {
      dynamic_simulation_B.b_kstr_m = 1;
      do {
        exitg1 = 0;
        if (dynamic_simulation_B.b_kstr_m - 1 < 8) {
          dynamic_simulation_B.kstr_o = dynamic_simulation_B.b_kstr_m - 1;
          if (obj->Type->data[dynamic_simulation_B.kstr_o] !=
              dynamic_simulation_B.b_i[dynamic_simulation_B.kstr_o]) {
            exitg1 = 1;
          } else {
            dynamic_simulation_B.b_kstr_m++;
          }
        } else {
          dynamic_simulation_B.b_bool_g = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynamic_simulation_B.b_bool_g) {
      dynamic_simulation_B.b_kstr_m = 1;
    } else {
      dynamic_simulation_B.b_kstr_m = -1;
    }
  }

  switch (dynamic_simulation_B.b_kstr_m) {
   case 0:
    memset(&dynamic_simulation_B.TJ[0], 0, sizeof(real_T) << 4U);
    dynamic_simulation_B.TJ[0] = 1.0;
    dynamic_simulation_B.TJ[5] = 1.0;
    dynamic_simulation_B.TJ[10] = 1.0;
    dynamic_simulation_B.TJ[15] = 1.0;
    break;

   case 1:
    rigidBodyJoint_get_JointAxis_kq(obj, dynamic_simulation_B.v_c);
    dynamic_simulation_B.result_data_n[0] = dynamic_simulation_B.v_c[0];
    dynamic_simulation_B.result_data_n[1] = dynamic_simulation_B.v_c[1];
    dynamic_simulation_B.result_data_n[2] = dynamic_simulation_B.v_c[2];
    if (0 <= (*q_size != 0) - 1) {
      dynamic_simulation_B.result_data_n[3] = q_data[0];
    }

    dynamic_simulation_B.cth = 1.0 / sqrt((dynamic_simulation_B.result_data_n[0]
      * dynamic_simulation_B.result_data_n[0] +
      dynamic_simulation_B.result_data_n[1] *
      dynamic_simulation_B.result_data_n[1]) +
      dynamic_simulation_B.result_data_n[2] *
      dynamic_simulation_B.result_data_n[2]);
    dynamic_simulation_B.v_c[0] = dynamic_simulation_B.result_data_n[0] *
      dynamic_simulation_B.cth;
    dynamic_simulation_B.v_c[1] = dynamic_simulation_B.result_data_n[1] *
      dynamic_simulation_B.cth;
    dynamic_simulation_B.v_c[2] = dynamic_simulation_B.result_data_n[2] *
      dynamic_simulation_B.cth;
    dynamic_simulation_B.cth = cos(dynamic_simulation_B.result_data_n[3]);
    dynamic_simulation_B.sth_c = sin(dynamic_simulation_B.result_data_n[3]);
    dynamic_simulation_B.tempR_tmp_o = dynamic_simulation_B.v_c[1] *
      dynamic_simulation_B.v_c[0] * (1.0 - dynamic_simulation_B.cth);
    dynamic_simulation_B.tempR_tmp_l = dynamic_simulation_B.v_c[2] *
      dynamic_simulation_B.sth_c;
    dynamic_simulation_B.tempR_tmp_m = dynamic_simulation_B.v_c[2] *
      dynamic_simulation_B.v_c[0] * (1.0 - dynamic_simulation_B.cth);
    dynamic_simulation_B.tempR_tmp_mj = dynamic_simulation_B.v_c[1] *
      dynamic_simulation_B.sth_c;
    dynamic_simulation_B.tempR_tmp_c = dynamic_simulation_B.v_c[2] *
      dynamic_simulation_B.v_c[1] * (1.0 - dynamic_simulation_B.cth);
    dynamic_simulation_B.sth_c *= dynamic_simulation_B.v_c[0];
    dynamic_simulation_cat(dynamic_simulation_B.v_c[0] *
      dynamic_simulation_B.v_c[0] * (1.0 - dynamic_simulation_B.cth) +
      dynamic_simulation_B.cth, dynamic_simulation_B.tempR_tmp_o -
      dynamic_simulation_B.tempR_tmp_l, dynamic_simulation_B.tempR_tmp_m +
      dynamic_simulation_B.tempR_tmp_mj, dynamic_simulation_B.tempR_tmp_o +
      dynamic_simulation_B.tempR_tmp_l, dynamic_simulation_B.v_c[1] *
      dynamic_simulation_B.v_c[1] * (1.0 - dynamic_simulation_B.cth) +
      dynamic_simulation_B.cth, dynamic_simulation_B.tempR_tmp_c -
      dynamic_simulation_B.sth_c, dynamic_simulation_B.tempR_tmp_m -
      dynamic_simulation_B.tempR_tmp_mj, dynamic_simulation_B.tempR_tmp_c +
      dynamic_simulation_B.sth_c, dynamic_simulation_B.v_c[2] *
      dynamic_simulation_B.v_c[2] * (1.0 - dynamic_simulation_B.cth) +
      dynamic_simulation_B.cth, dynamic_simulation_B.tempR_b);
    for (dynamic_simulation_B.b_kstr_m = 0; dynamic_simulation_B.b_kstr_m < 3;
         dynamic_simulation_B.b_kstr_m++) {
      dynamic_simulation_B.kstr_o = dynamic_simulation_B.b_kstr_m + 1;
      dynamic_simulation_B.R_o[dynamic_simulation_B.kstr_o - 1] =
        dynamic_simulation_B.tempR_b[(dynamic_simulation_B.kstr_o - 1) * 3];
      dynamic_simulation_B.kstr_o = dynamic_simulation_B.b_kstr_m + 1;
      dynamic_simulation_B.R_o[dynamic_simulation_B.kstr_o + 2] =
        dynamic_simulation_B.tempR_b[(dynamic_simulation_B.kstr_o - 1) * 3 + 1];
      dynamic_simulation_B.kstr_o = dynamic_simulation_B.b_kstr_m + 1;
      dynamic_simulation_B.R_o[dynamic_simulation_B.kstr_o + 5] =
        dynamic_simulation_B.tempR_b[(dynamic_simulation_B.kstr_o - 1) * 3 + 2];
    }

    memset(&dynamic_simulation_B.TJ[0], 0, sizeof(real_T) << 4U);
    for (dynamic_simulation_B.b_kstr_m = 0; dynamic_simulation_B.b_kstr_m < 3;
         dynamic_simulation_B.b_kstr_m++) {
      dynamic_simulation_B.kstr_o = dynamic_simulation_B.b_kstr_m << 2;
      dynamic_simulation_B.TJ[dynamic_simulation_B.kstr_o] =
        dynamic_simulation_B.R_o[3 * dynamic_simulation_B.b_kstr_m];
      dynamic_simulation_B.TJ[dynamic_simulation_B.kstr_o + 1] =
        dynamic_simulation_B.R_o[3 * dynamic_simulation_B.b_kstr_m + 1];
      dynamic_simulation_B.TJ[dynamic_simulation_B.kstr_o + 2] =
        dynamic_simulation_B.R_o[3 * dynamic_simulation_B.b_kstr_m + 2];
    }

    dynamic_simulation_B.TJ[15] = 1.0;
    break;

   default:
    rigidBodyJoint_get_JointAxis_kq(obj, dynamic_simulation_B.v_c);
    memset(&dynamic_simulation_B.tempR_b[0], 0, 9U * sizeof(real_T));
    dynamic_simulation_B.tempR_b[0] = 1.0;
    dynamic_simulation_B.tempR_b[4] = 1.0;
    dynamic_simulation_B.tempR_b[8] = 1.0;
    for (dynamic_simulation_B.b_kstr_m = 0; dynamic_simulation_B.b_kstr_m < 3;
         dynamic_simulation_B.b_kstr_m++) {
      dynamic_simulation_B.kstr_o = dynamic_simulation_B.b_kstr_m << 2;
      dynamic_simulation_B.TJ[dynamic_simulation_B.kstr_o] =
        dynamic_simulation_B.tempR_b[3 * dynamic_simulation_B.b_kstr_m];
      dynamic_simulation_B.TJ[dynamic_simulation_B.kstr_o + 1] =
        dynamic_simulation_B.tempR_b[3 * dynamic_simulation_B.b_kstr_m + 1];
      dynamic_simulation_B.TJ[dynamic_simulation_B.kstr_o + 2] =
        dynamic_simulation_B.tempR_b[3 * dynamic_simulation_B.b_kstr_m + 2];
      dynamic_simulation_B.TJ[dynamic_simulation_B.b_kstr_m + 12] =
        dynamic_simulation_B.v_c[dynamic_simulation_B.b_kstr_m] * q_data[0];
    }

    dynamic_simulation_B.TJ[3] = 0.0;
    dynamic_simulation_B.TJ[7] = 0.0;
    dynamic_simulation_B.TJ[11] = 0.0;
    dynamic_simulation_B.TJ[15] = 1.0;
    break;
  }

  for (dynamic_simulation_B.b_kstr_m = 0; dynamic_simulation_B.b_kstr_m < 4;
       dynamic_simulation_B.b_kstr_m++) {
    for (dynamic_simulation_B.kstr_o = 0; dynamic_simulation_B.kstr_o < 4;
         dynamic_simulation_B.kstr_o++) {
      dynamic_simulation_B.obj_tmp_tmp = dynamic_simulation_B.kstr_o << 2;
      dynamic_simulation_B.obj_tmp = dynamic_simulation_B.b_kstr_m +
        dynamic_simulation_B.obj_tmp_tmp;
      dynamic_simulation_B.obj[dynamic_simulation_B.obj_tmp] = 0.0;
      dynamic_simulation_B.obj[dynamic_simulation_B.obj_tmp] +=
        dynamic_simulation_B.TJ[dynamic_simulation_B.obj_tmp_tmp] *
        obj->JointToParentTransform[dynamic_simulation_B.b_kstr_m];
      dynamic_simulation_B.obj[dynamic_simulation_B.obj_tmp] +=
        dynamic_simulation_B.TJ[dynamic_simulation_B.obj_tmp_tmp + 1] *
        obj->JointToParentTransform[dynamic_simulation_B.b_kstr_m + 4];
      dynamic_simulation_B.obj[dynamic_simulation_B.obj_tmp] +=
        dynamic_simulation_B.TJ[dynamic_simulation_B.obj_tmp_tmp + 2] *
        obj->JointToParentTransform[dynamic_simulation_B.b_kstr_m + 8];
      dynamic_simulation_B.obj[dynamic_simulation_B.obj_tmp] +=
        dynamic_simulation_B.TJ[dynamic_simulation_B.obj_tmp_tmp + 3] *
        obj->JointToParentTransform[dynamic_simulation_B.b_kstr_m + 12];
    }

    for (dynamic_simulation_B.kstr_o = 0; dynamic_simulation_B.kstr_o < 4;
         dynamic_simulation_B.kstr_o++) {
      dynamic_simulation_B.obj_tmp_tmp = dynamic_simulation_B.kstr_o << 2;
      dynamic_simulation_B.obj_tmp = dynamic_simulation_B.b_kstr_m +
        dynamic_simulation_B.obj_tmp_tmp;
      T[dynamic_simulation_B.obj_tmp] = 0.0;
      T[dynamic_simulation_B.obj_tmp] += obj->
        ChildToJointTransform[dynamic_simulation_B.obj_tmp_tmp] *
        dynamic_simulation_B.obj[dynamic_simulation_B.b_kstr_m];
      T[dynamic_simulation_B.obj_tmp] += obj->
        ChildToJointTransform[dynamic_simulation_B.obj_tmp_tmp + 1] *
        dynamic_simulation_B.obj[dynamic_simulation_B.b_kstr_m + 4];
      T[dynamic_simulation_B.obj_tmp] += obj->
        ChildToJointTransform[dynamic_simulation_B.obj_tmp_tmp + 2] *
        dynamic_simulation_B.obj[dynamic_simulation_B.b_kstr_m + 8];
      T[dynamic_simulation_B.obj_tmp] += obj->
        ChildToJointTransform[dynamic_simulation_B.obj_tmp_tmp + 3] *
        dynamic_simulation_B.obj[dynamic_simulation_B.b_kstr_m + 12];
    }
  }
}

static void rigidBodyJoint_transformBodyToP(const
  c_rigidBodyJoint_dynamic_s_kq_T *obj, real_T T[16])
{
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  for (dynamic_simulation_B.b_kstr_i = 0; dynamic_simulation_B.b_kstr_i < 5;
       dynamic_simulation_B.b_kstr_i++) {
    dynamic_simulation_B.b_axz[dynamic_simulation_B.b_kstr_i] =
      tmp[dynamic_simulation_B.b_kstr_i];
  }

  dynamic_simulation_B.b_bool_c = false;
  if (obj->Type->size[1] == 5) {
    dynamic_simulation_B.b_kstr_i = 1;
    do {
      exitg1 = 0;
      if (dynamic_simulation_B.b_kstr_i - 1 < 5) {
        dynamic_simulation_B.kstr_n = dynamic_simulation_B.b_kstr_i - 1;
        if (obj->Type->data[dynamic_simulation_B.kstr_n] !=
            dynamic_simulation_B.b_axz[dynamic_simulation_B.kstr_n]) {
          exitg1 = 1;
        } else {
          dynamic_simulation_B.b_kstr_i++;
        }
      } else {
        dynamic_simulation_B.b_bool_c = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (dynamic_simulation_B.b_bool_c) {
    dynamic_simulation_B.b_kstr_i = 0;
  } else {
    for (dynamic_simulation_B.b_kstr_i = 0; dynamic_simulation_B.b_kstr_i < 8;
         dynamic_simulation_B.b_kstr_i++) {
      dynamic_simulation_B.b_o[dynamic_simulation_B.b_kstr_i] =
        tmp_0[dynamic_simulation_B.b_kstr_i];
    }

    dynamic_simulation_B.b_bool_c = false;
    if (obj->Type->size[1] == 8) {
      dynamic_simulation_B.b_kstr_i = 1;
      do {
        exitg1 = 0;
        if (dynamic_simulation_B.b_kstr_i - 1 < 8) {
          dynamic_simulation_B.kstr_n = dynamic_simulation_B.b_kstr_i - 1;
          if (obj->Type->data[dynamic_simulation_B.kstr_n] !=
              dynamic_simulation_B.b_o[dynamic_simulation_B.kstr_n]) {
            exitg1 = 1;
          } else {
            dynamic_simulation_B.b_kstr_i++;
          }
        } else {
          dynamic_simulation_B.b_bool_c = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynamic_simulation_B.b_bool_c) {
      dynamic_simulation_B.b_kstr_i = 1;
    } else {
      dynamic_simulation_B.b_kstr_i = -1;
    }
  }

  switch (dynamic_simulation_B.b_kstr_i) {
   case 0:
    memset(&dynamic_simulation_B.TJ_g[0], 0, sizeof(real_T) << 4U);
    dynamic_simulation_B.TJ_g[0] = 1.0;
    dynamic_simulation_B.TJ_g[5] = 1.0;
    dynamic_simulation_B.TJ_g[10] = 1.0;
    dynamic_simulation_B.TJ_g[15] = 1.0;
    break;

   case 1:
    rigidBodyJoint_get_JointAxis_kq(obj, dynamic_simulation_B.v_m);
    dynamic_simulation_B.axang_idx_0 = dynamic_simulation_B.v_m[0];
    dynamic_simulation_B.axang_idx_1 = dynamic_simulation_B.v_m[1];
    dynamic_simulation_B.axang_idx_2 = dynamic_simulation_B.v_m[2];
    dynamic_simulation_B.b_l5 = 1.0 / sqrt((dynamic_simulation_B.axang_idx_0 *
      dynamic_simulation_B.axang_idx_0 + dynamic_simulation_B.axang_idx_1 *
      dynamic_simulation_B.axang_idx_1) + dynamic_simulation_B.axang_idx_2 *
      dynamic_simulation_B.axang_idx_2);
    dynamic_simulation_B.v_m[0] = dynamic_simulation_B.axang_idx_0 *
      dynamic_simulation_B.b_l5;
    dynamic_simulation_B.v_m[1] = dynamic_simulation_B.axang_idx_1 *
      dynamic_simulation_B.b_l5;
    dynamic_simulation_B.v_m[2] = dynamic_simulation_B.axang_idx_2 *
      dynamic_simulation_B.b_l5;
    dynamic_simulation_B.axang_idx_0 = dynamic_simulation_B.v_m[1] *
      dynamic_simulation_B.v_m[0] * 0.0;
    dynamic_simulation_B.axang_idx_1 = dynamic_simulation_B.v_m[2] *
      dynamic_simulation_B.v_m[0] * 0.0;
    dynamic_simulation_B.axang_idx_2 = dynamic_simulation_B.v_m[2] *
      dynamic_simulation_B.v_m[1] * 0.0;
    dynamic_simulation_cat(dynamic_simulation_B.v_m[0] *
      dynamic_simulation_B.v_m[0] * 0.0 + 1.0, dynamic_simulation_B.axang_idx_0
      - dynamic_simulation_B.v_m[2] * 0.0, dynamic_simulation_B.axang_idx_1 +
      dynamic_simulation_B.v_m[1] * 0.0, dynamic_simulation_B.axang_idx_0 +
      dynamic_simulation_B.v_m[2] * 0.0, dynamic_simulation_B.v_m[1] *
      dynamic_simulation_B.v_m[1] * 0.0 + 1.0, dynamic_simulation_B.axang_idx_2
      - dynamic_simulation_B.v_m[0] * 0.0, dynamic_simulation_B.axang_idx_1 -
      dynamic_simulation_B.v_m[1] * 0.0, dynamic_simulation_B.axang_idx_2 +
      dynamic_simulation_B.v_m[0] * 0.0, dynamic_simulation_B.v_m[2] *
      dynamic_simulation_B.v_m[2] * 0.0 + 1.0, dynamic_simulation_B.tempR_bs);
    for (dynamic_simulation_B.b_kstr_i = 0; dynamic_simulation_B.b_kstr_i < 3;
         dynamic_simulation_B.b_kstr_i++) {
      dynamic_simulation_B.kstr_n = dynamic_simulation_B.b_kstr_i + 1;
      dynamic_simulation_B.R_n[dynamic_simulation_B.kstr_n - 1] =
        dynamic_simulation_B.tempR_bs[(dynamic_simulation_B.kstr_n - 1) * 3];
      dynamic_simulation_B.kstr_n = dynamic_simulation_B.b_kstr_i + 1;
      dynamic_simulation_B.R_n[dynamic_simulation_B.kstr_n + 2] =
        dynamic_simulation_B.tempR_bs[(dynamic_simulation_B.kstr_n - 1) * 3 + 1];
      dynamic_simulation_B.kstr_n = dynamic_simulation_B.b_kstr_i + 1;
      dynamic_simulation_B.R_n[dynamic_simulation_B.kstr_n + 5] =
        dynamic_simulation_B.tempR_bs[(dynamic_simulation_B.kstr_n - 1) * 3 + 2];
    }

    memset(&dynamic_simulation_B.TJ_g[0], 0, sizeof(real_T) << 4U);
    for (dynamic_simulation_B.b_kstr_i = 0; dynamic_simulation_B.b_kstr_i < 3;
         dynamic_simulation_B.b_kstr_i++) {
      dynamic_simulation_B.kstr_n = dynamic_simulation_B.b_kstr_i << 2;
      dynamic_simulation_B.TJ_g[dynamic_simulation_B.kstr_n] =
        dynamic_simulation_B.R_n[3 * dynamic_simulation_B.b_kstr_i];
      dynamic_simulation_B.TJ_g[dynamic_simulation_B.kstr_n + 1] =
        dynamic_simulation_B.R_n[3 * dynamic_simulation_B.b_kstr_i + 1];
      dynamic_simulation_B.TJ_g[dynamic_simulation_B.kstr_n + 2] =
        dynamic_simulation_B.R_n[3 * dynamic_simulation_B.b_kstr_i + 2];
    }

    dynamic_simulation_B.TJ_g[15] = 1.0;
    break;

   default:
    rigidBodyJoint_get_JointAxis_kq(obj, dynamic_simulation_B.v_m);
    memset(&dynamic_simulation_B.tempR_bs[0], 0, 9U * sizeof(real_T));
    dynamic_simulation_B.tempR_bs[0] = 1.0;
    dynamic_simulation_B.tempR_bs[4] = 1.0;
    dynamic_simulation_B.tempR_bs[8] = 1.0;
    for (dynamic_simulation_B.b_kstr_i = 0; dynamic_simulation_B.b_kstr_i < 3;
         dynamic_simulation_B.b_kstr_i++) {
      dynamic_simulation_B.kstr_n = dynamic_simulation_B.b_kstr_i << 2;
      dynamic_simulation_B.TJ_g[dynamic_simulation_B.kstr_n] =
        dynamic_simulation_B.tempR_bs[3 * dynamic_simulation_B.b_kstr_i];
      dynamic_simulation_B.TJ_g[dynamic_simulation_B.kstr_n + 1] =
        dynamic_simulation_B.tempR_bs[3 * dynamic_simulation_B.b_kstr_i + 1];
      dynamic_simulation_B.TJ_g[dynamic_simulation_B.kstr_n + 2] =
        dynamic_simulation_B.tempR_bs[3 * dynamic_simulation_B.b_kstr_i + 2];
      dynamic_simulation_B.TJ_g[dynamic_simulation_B.b_kstr_i + 12] =
        dynamic_simulation_B.v_m[dynamic_simulation_B.b_kstr_i] * 0.0;
    }

    dynamic_simulation_B.TJ_g[3] = 0.0;
    dynamic_simulation_B.TJ_g[7] = 0.0;
    dynamic_simulation_B.TJ_g[11] = 0.0;
    dynamic_simulation_B.TJ_g[15] = 1.0;
    break;
  }

  for (dynamic_simulation_B.b_kstr_i = 0; dynamic_simulation_B.b_kstr_i < 4;
       dynamic_simulation_B.b_kstr_i++) {
    for (dynamic_simulation_B.kstr_n = 0; dynamic_simulation_B.kstr_n < 4;
         dynamic_simulation_B.kstr_n++) {
      dynamic_simulation_B.obj_tmp_tmp_j = dynamic_simulation_B.kstr_n << 2;
      dynamic_simulation_B.obj_tmp_m = dynamic_simulation_B.b_kstr_i +
        dynamic_simulation_B.obj_tmp_tmp_j;
      dynamic_simulation_B.obj_g[dynamic_simulation_B.obj_tmp_m] = 0.0;
      dynamic_simulation_B.obj_g[dynamic_simulation_B.obj_tmp_m] +=
        dynamic_simulation_B.TJ_g[dynamic_simulation_B.obj_tmp_tmp_j] *
        obj->JointToParentTransform[dynamic_simulation_B.b_kstr_i];
      dynamic_simulation_B.obj_g[dynamic_simulation_B.obj_tmp_m] +=
        dynamic_simulation_B.TJ_g[dynamic_simulation_B.obj_tmp_tmp_j + 1] *
        obj->JointToParentTransform[dynamic_simulation_B.b_kstr_i + 4];
      dynamic_simulation_B.obj_g[dynamic_simulation_B.obj_tmp_m] +=
        dynamic_simulation_B.TJ_g[dynamic_simulation_B.obj_tmp_tmp_j + 2] *
        obj->JointToParentTransform[dynamic_simulation_B.b_kstr_i + 8];
      dynamic_simulation_B.obj_g[dynamic_simulation_B.obj_tmp_m] +=
        dynamic_simulation_B.TJ_g[dynamic_simulation_B.obj_tmp_tmp_j + 3] *
        obj->JointToParentTransform[dynamic_simulation_B.b_kstr_i + 12];
    }

    for (dynamic_simulation_B.kstr_n = 0; dynamic_simulation_B.kstr_n < 4;
         dynamic_simulation_B.kstr_n++) {
      dynamic_simulation_B.obj_tmp_tmp_j = dynamic_simulation_B.kstr_n << 2;
      dynamic_simulation_B.obj_tmp_m = dynamic_simulation_B.b_kstr_i +
        dynamic_simulation_B.obj_tmp_tmp_j;
      T[dynamic_simulation_B.obj_tmp_m] = 0.0;
      T[dynamic_simulation_B.obj_tmp_m] += obj->
        ChildToJointTransform[dynamic_simulation_B.obj_tmp_tmp_j] *
        dynamic_simulation_B.obj_g[dynamic_simulation_B.b_kstr_i];
      T[dynamic_simulation_B.obj_tmp_m] += obj->
        ChildToJointTransform[dynamic_simulation_B.obj_tmp_tmp_j + 1] *
        dynamic_simulation_B.obj_g[dynamic_simulation_B.b_kstr_i + 4];
      T[dynamic_simulation_B.obj_tmp_m] += obj->
        ChildToJointTransform[dynamic_simulation_B.obj_tmp_tmp_j + 2] *
        dynamic_simulation_B.obj_g[dynamic_simulation_B.b_kstr_i + 8];
      T[dynamic_simulation_B.obj_tmp_m] += obj->
        ChildToJointTransform[dynamic_simulation_B.obj_tmp_tmp_j + 3] *
        dynamic_simulation_B.obj_g[dynamic_simulation_B.b_kstr_i + 12];
    }
  }
}

static void dynamic_simulation_tforminv(const real_T T[16], real_T Tinv[16])
{
  for (dynamic_simulation_B.i3 = 0; dynamic_simulation_B.i3 < 3;
       dynamic_simulation_B.i3++) {
    dynamic_simulation_B.R_b[3 * dynamic_simulation_B.i3] =
      T[dynamic_simulation_B.i3];
    dynamic_simulation_B.R_b[3 * dynamic_simulation_B.i3 + 1] =
      T[dynamic_simulation_B.i3 + 4];
    dynamic_simulation_B.R_b[3 * dynamic_simulation_B.i3 + 2] =
      T[dynamic_simulation_B.i3 + 8];
  }

  for (dynamic_simulation_B.i3 = 0; dynamic_simulation_B.i3 < 9;
       dynamic_simulation_B.i3++) {
    dynamic_simulation_B.R_da[dynamic_simulation_B.i3] =
      -dynamic_simulation_B.R_b[dynamic_simulation_B.i3];
  }

  for (dynamic_simulation_B.i3 = 0; dynamic_simulation_B.i3 < 3;
       dynamic_simulation_B.i3++) {
    dynamic_simulation_B.Tinv_tmp = dynamic_simulation_B.i3 << 2;
    Tinv[dynamic_simulation_B.Tinv_tmp] = dynamic_simulation_B.R_b[3 *
      dynamic_simulation_B.i3];
    Tinv[dynamic_simulation_B.Tinv_tmp + 1] = dynamic_simulation_B.R_b[3 *
      dynamic_simulation_B.i3 + 1];
    Tinv[dynamic_simulation_B.Tinv_tmp + 2] = dynamic_simulation_B.R_b[3 *
      dynamic_simulation_B.i3 + 2];
    Tinv[dynamic_simulation_B.i3 + 12] =
      dynamic_simulation_B.R_da[dynamic_simulation_B.i3 + 6] * T[14] +
      (dynamic_simulation_B.R_da[dynamic_simulation_B.i3 + 3] * T[13] +
       dynamic_simulation_B.R_da[dynamic_simulation_B.i3] * T[12]);
  }

  Tinv[3] = 0.0;
  Tinv[7] = 0.0;
  Tinv[11] = 0.0;
  Tinv[15] = 1.0;
}

static void dynamic_sim_tformToSpatialXform(const real_T T[16], real_T X[36])
{
  dynamic_simulation_B.dv2[0] = 0.0;
  dynamic_simulation_B.dv2[3] = -T[14];
  dynamic_simulation_B.dv2[6] = T[13];
  dynamic_simulation_B.dv2[1] = T[14];
  dynamic_simulation_B.dv2[4] = 0.0;
  dynamic_simulation_B.dv2[7] = -T[12];
  dynamic_simulation_B.dv2[2] = -T[13];
  dynamic_simulation_B.dv2[5] = T[12];
  dynamic_simulation_B.dv2[8] = 0.0;
  for (dynamic_simulation_B.i1 = 0; dynamic_simulation_B.i1 < 3;
       dynamic_simulation_B.i1++) {
    for (dynamic_simulation_B.X_tmp_p = 0; dynamic_simulation_B.X_tmp_p < 3;
         dynamic_simulation_B.X_tmp_p++) {
      dynamic_simulation_B.X_tmp_n = dynamic_simulation_B.i1 + 3 *
        dynamic_simulation_B.X_tmp_p;
      dynamic_simulation_B.dv3[dynamic_simulation_B.X_tmp_n] = 0.0;
      dynamic_simulation_B.i2 = dynamic_simulation_B.X_tmp_p << 2;
      dynamic_simulation_B.dv3[dynamic_simulation_B.X_tmp_n] +=
        T[dynamic_simulation_B.i2] *
        dynamic_simulation_B.dv2[dynamic_simulation_B.i1];
      dynamic_simulation_B.dv3[dynamic_simulation_B.X_tmp_n] +=
        T[dynamic_simulation_B.i2 + 1] *
        dynamic_simulation_B.dv2[dynamic_simulation_B.i1 + 3];
      dynamic_simulation_B.dv3[dynamic_simulation_B.X_tmp_n] +=
        T[dynamic_simulation_B.i2 + 2] *
        dynamic_simulation_B.dv2[dynamic_simulation_B.i1 + 6];
      X[dynamic_simulation_B.X_tmp_p + 6 * dynamic_simulation_B.i1] = T
        [(dynamic_simulation_B.i1 << 2) + dynamic_simulation_B.X_tmp_p];
      X[dynamic_simulation_B.X_tmp_p + 6 * (dynamic_simulation_B.i1 + 3)] = 0.0;
    }
  }

  for (dynamic_simulation_B.i1 = 0; dynamic_simulation_B.i1 < 3;
       dynamic_simulation_B.i1++) {
    X[6 * dynamic_simulation_B.i1 + 3] = dynamic_simulation_B.dv3[3 *
      dynamic_simulation_B.i1];
    dynamic_simulation_B.X_tmp_p = dynamic_simulation_B.i1 << 2;
    dynamic_simulation_B.X_tmp_n = 6 * (dynamic_simulation_B.i1 + 3);
    X[dynamic_simulation_B.X_tmp_n + 3] = T[dynamic_simulation_B.X_tmp_p];
    X[6 * dynamic_simulation_B.i1 + 4] = dynamic_simulation_B.dv3[3 *
      dynamic_simulation_B.i1 + 1];
    X[dynamic_simulation_B.X_tmp_n + 4] = T[dynamic_simulation_B.X_tmp_p + 1];
    X[6 * dynamic_simulation_B.i1 + 5] = dynamic_simulation_B.dv3[3 *
      dynamic_simulation_B.i1 + 2];
    X[dynamic_simulation_B.X_tmp_n + 5] = T[dynamic_simulation_B.X_tmp_p + 2];
  }
}

static void dynamic_si_emxFree_f_cell_wrap1(emxArray_f_cell_wrap_dynami_k_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_f_cell_wrap_dynami_k_T *)NULL) {
    if (((*pEmxArray)->data != (f_cell_wrap_dynamic_simulat_k_T *)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_f_cell_wrap_dynami_k_T *)NULL;
  }
}

static void RigidBodyTreeDynamics_massMatri(p_robotics_manip_internal__kq_T
  *robot, const real_T q[6], emxArray_real_T_dynamic_simul_T *H,
  emxArray_real_T_dynamic_simul_T *lambda)
{
  emxArray_f_cell_wrap_dynami_k_T *Ic;
  emxArray_f_cell_wrap_dynami_k_T *X;
  emxArray_real_T_dynamic_simul_T *lambda_;
  emxArray_real_T_dynamic_simul_T *Si;
  emxArray_real_T_dynamic_simul_T *Fi;
  emxArray_real_T_dynamic_simul_T *Hji;
  emxArray_real_T_dynamic_simul_T *s;
  n_robotics_manip_internal__kq_T *obj;
  emxArray_char_T_dynamic_simul_T *a;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  boolean_T exitg1;
  int32_T exitg2;
  dynamic_simulation_B.nb_f = robot->NumBodies;
  dynamic_simulation_B.vNum_p = robot->VelocityNumber;
  dynamic_simulation_B.f = H->size[0] * H->size[1];
  dynamic_simulation_B.b_i_h = static_cast<int32_T>(dynamic_simulation_B.vNum_p);
  H->size[0] = dynamic_simulation_B.b_i_h;
  H->size[1] = dynamic_simulation_B.b_i_h;
  dynami_emxEnsureCapacity_real_T(H, dynamic_simulation_B.f);
  dynamic_simulation_B.loop_ub_p = dynamic_simulation_B.b_i_h *
    dynamic_simulation_B.b_i_h - 1;
  for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f <=
       dynamic_simulation_B.loop_ub_p; dynamic_simulation_B.f++) {
    H->data[dynamic_simulation_B.f] = 0.0;
  }

  dynamic_simulati_emxInit_real_T(&lambda_, 2);
  dynamic_simulation_B.f = lambda_->size[0] * lambda_->size[1];
  lambda_->size[0] = 1;
  dynamic_simulation_B.nm1d2 = static_cast<int32_T>(dynamic_simulation_B.nb_f);
  lambda_->size[1] = dynamic_simulation_B.nm1d2;
  dynami_emxEnsureCapacity_real_T(lambda_, dynamic_simulation_B.f);
  dynamic_simulation_B.idx = dynamic_simulation_B.nm1d2 - 1;
  for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f <=
       dynamic_simulation_B.idx; dynamic_simulation_B.f++) {
    lambda_->data[dynamic_simulation_B.f] = 0.0;
  }

  dynamic_simulation_B.f = lambda->size[0] * lambda->size[1];
  lambda->size[0] = 1;
  lambda->size[1] = dynamic_simulation_B.b_i_h;
  dynami_emxEnsureCapacity_real_T(lambda, dynamic_simulation_B.f);
  dynamic_simulation_B.loop_ub_p = dynamic_simulation_B.b_i_h - 1;
  for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f <=
       dynamic_simulation_B.loop_ub_p; dynamic_simulation_B.f++) {
    lambda->data[dynamic_simulation_B.f] = 0.0;
  }

  dynamic_si_emxInit_f_cell_wrap1(&Ic, 2);
  dynamic_si_emxInit_f_cell_wrap1(&X, 2);
  dynamic_simulation_B.f = Ic->size[0] * Ic->size[1];
  Ic->size[0] = 1;
  Ic->size[1] = dynamic_simulation_B.nm1d2;
  emxEnsureCapacity_f_cell_wrap1(Ic, dynamic_simulation_B.f);
  dynamic_simulation_B.f = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = dynamic_simulation_B.nm1d2;
  emxEnsureCapacity_f_cell_wrap1(X, dynamic_simulation_B.f);
  for (dynamic_simulation_B.b_i_h = 0; dynamic_simulation_B.b_i_h <=
       dynamic_simulation_B.idx; dynamic_simulation_B.b_i_h++) {
    for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f < 36;
         dynamic_simulation_B.f++) {
      Ic->data[dynamic_simulation_B.b_i_h].f1[dynamic_simulation_B.f] =
        robot->Bodies[dynamic_simulation_B.b_i_h]->
        SpatialInertia[dynamic_simulation_B.f];
    }

    dynamic_simulation_B.vNum_p = robot->
      PositionDoFMap[dynamic_simulation_B.b_i_h];
    dynamic_simulation_B.p_idx_1 = robot->
      PositionDoFMap[dynamic_simulation_B.b_i_h + 8];
    if (dynamic_simulation_B.p_idx_1 < dynamic_simulation_B.vNum_p) {
      obj = robot->Bodies[dynamic_simulation_B.b_i_h];
      rigidBodyJoint_transformBodyToP(&obj->JointInternal,
        dynamic_simulation_B.T_f);
    } else {
      if (dynamic_simulation_B.vNum_p > dynamic_simulation_B.p_idx_1) {
        dynamic_simulation_B.nm1d2 = 0;
        dynamic_simulation_B.f = -1;
      } else {
        dynamic_simulation_B.nm1d2 = static_cast<int32_T>
          (dynamic_simulation_B.vNum_p) - 1;
        dynamic_simulation_B.f = static_cast<int32_T>
          (dynamic_simulation_B.p_idx_1) - 1;
      }

      obj = robot->Bodies[dynamic_simulation_B.b_i_h];
      dynamic_simulation_B.loop_ub_p = dynamic_simulation_B.f -
        dynamic_simulation_B.nm1d2;
      dynamic_simulation_B.q_size_b = dynamic_simulation_B.loop_ub_p + 1;
      for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f <=
           dynamic_simulation_B.loop_ub_p; dynamic_simulation_B.f++) {
        dynamic_simulation_B.q_data_j[dynamic_simulation_B.f] =
          q[dynamic_simulation_B.nm1d2 + dynamic_simulation_B.f];
      }

      rigidBodyJoint_transformBodyT_k(&obj->JointInternal,
        dynamic_simulation_B.q_data_j, &dynamic_simulation_B.q_size_b,
        dynamic_simulation_B.T_f);
    }

    dynamic_simulation_tforminv(dynamic_simulation_B.T_f,
      dynamic_simulation_B.dv);
    dynamic_sim_tformToSpatialXform(dynamic_simulation_B.dv, X->
      data[dynamic_simulation_B.b_i_h].f1);
  }

  dynamic_simulation_B.nm1d2 = static_cast<int32_T>(((-1.0 -
    dynamic_simulation_B.nb_f) + 1.0) / -1.0) - 1;
  dynamic_simulati_emxInit_real_T(&Si, 2);
  dynamic_simulati_emxInit_real_T(&Fi, 2);
  dynamic_simulati_emxInit_real_T(&Hji, 2);
  dynamic_simulati_emxInit_char_T(&a, 2);
  for (dynamic_simulation_B.idx = 0; dynamic_simulation_B.idx <=
       dynamic_simulation_B.nm1d2; dynamic_simulation_B.idx++) {
    dynamic_simulation_B.n_m = static_cast<int32_T>(dynamic_simulation_B.nb_f +
      -static_cast<real_T>(dynamic_simulation_B.idx));
    dynamic_simulation_B.pid_tmp = dynamic_simulation_B.n_m - 1;
    dynamic_simulation_B.pid = robot->Bodies[dynamic_simulation_B.pid_tmp]
      ->ParentIndex;
    dynamic_simulation_B.vNum_p = robot->VelocityDoFMap[dynamic_simulation_B.n_m
      - 1];
    dynamic_simulation_B.p_idx_1 = robot->
      VelocityDoFMap[dynamic_simulation_B.n_m + 7];
    if (dynamic_simulation_B.pid > 0.0) {
      for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f < 6;
           dynamic_simulation_B.f++) {
        for (dynamic_simulation_B.b_i_h = 0; dynamic_simulation_B.b_i_h < 6;
             dynamic_simulation_B.b_i_h++) {
          dynamic_simulation_B.X_tmp = dynamic_simulation_B.f + 6 *
            dynamic_simulation_B.b_i_h;
          dynamic_simulation_B.X_m[dynamic_simulation_B.X_tmp] = 0.0;
          for (dynamic_simulation_B.loop_ub_p = 0;
               dynamic_simulation_B.loop_ub_p < 6;
               dynamic_simulation_B.loop_ub_p++) {
            dynamic_simulation_B.X_m[dynamic_simulation_B.X_tmp] += X->
              data[dynamic_simulation_B.pid_tmp].f1[6 * dynamic_simulation_B.f +
              dynamic_simulation_B.loop_ub_p] * Ic->
              data[dynamic_simulation_B.pid_tmp].f1[6 *
              dynamic_simulation_B.b_i_h + dynamic_simulation_B.loop_ub_p];
          }
        }
      }

      for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f < 6;
           dynamic_simulation_B.f++) {
        for (dynamic_simulation_B.b_i_h = 0; dynamic_simulation_B.b_i_h < 6;
             dynamic_simulation_B.b_i_h++) {
          dynamic_simulation_B.b_idx_0_o = 0.0;
          for (dynamic_simulation_B.loop_ub_p = 0;
               dynamic_simulation_B.loop_ub_p < 6;
               dynamic_simulation_B.loop_ub_p++) {
            dynamic_simulation_B.b_idx_0_o += dynamic_simulation_B.X_m[6 *
              dynamic_simulation_B.loop_ub_p + dynamic_simulation_B.f] * X->
              data[dynamic_simulation_B.pid_tmp].f1[6 *
              dynamic_simulation_B.b_i_h + dynamic_simulation_B.loop_ub_p];
          }

          dynamic_simulation_B.loop_ub_p = 6 * dynamic_simulation_B.b_i_h +
            dynamic_simulation_B.f;
          Ic->data[static_cast<int32_T>(dynamic_simulation_B.pid) - 1]
            .f1[dynamic_simulation_B.loop_ub_p] +=
            dynamic_simulation_B.b_idx_0_o;
        }
      }

      lambda_->data[dynamic_simulation_B.pid_tmp] = dynamic_simulation_B.pid;
      exitg1 = false;
      while ((!exitg1) && (lambda_->data[dynamic_simulation_B.pid_tmp] > 0.0)) {
        obj = robot->Bodies[static_cast<int32_T>(lambda_->
          data[dynamic_simulation_B.pid_tmp]) - 1];
        dynamic_simulation_B.f = a->size[0] * a->size[1];
        a->size[0] = 1;
        a->size[1] = obj->JointInternal.Type->size[1];
        dynami_emxEnsureCapacity_char_T(a, dynamic_simulation_B.f);
        dynamic_simulation_B.loop_ub_p = obj->JointInternal.Type->size[0] *
          obj->JointInternal.Type->size[1] - 1;
        for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f <=
             dynamic_simulation_B.loop_ub_p; dynamic_simulation_B.f++) {
          a->data[dynamic_simulation_B.f] = obj->JointInternal.Type->
            data[dynamic_simulation_B.f];
        }

        for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f < 5;
             dynamic_simulation_B.f++) {
          dynamic_simulation_B.b_ch[dynamic_simulation_B.f] =
            tmp[dynamic_simulation_B.f];
        }

        dynamic_simulation_B.b_bool_l = false;
        if (a->size[1] == 5) {
          dynamic_simulation_B.f = 1;
          do {
            exitg2 = 0;
            if (dynamic_simulation_B.f - 1 < 5) {
              dynamic_simulation_B.b_i_h = dynamic_simulation_B.f - 1;
              if (a->data[dynamic_simulation_B.b_i_h] !=
                  dynamic_simulation_B.b_ch[dynamic_simulation_B.b_i_h]) {
                exitg2 = 1;
              } else {
                dynamic_simulation_B.f++;
              }
            } else {
              dynamic_simulation_B.b_bool_l = true;
              exitg2 = 1;
            }
          } while (exitg2 == 0);
        }

        if (dynamic_simulation_B.b_bool_l) {
          lambda_->data[dynamic_simulation_B.pid_tmp] = robot->Bodies[
            static_cast<int32_T>(lambda_->data[dynamic_simulation_B.pid_tmp]) -
            1]->ParentIndex;
        } else {
          exitg1 = true;
        }
      }
    }

    dynamic_simulation_B.b_idx_0_o = robot->
      VelocityDoFMap[dynamic_simulation_B.n_m - 1];
    dynamic_simulation_B.b_idx_1_h = robot->
      VelocityDoFMap[dynamic_simulation_B.n_m + 7];
    if (dynamic_simulation_B.b_idx_0_o <= dynamic_simulation_B.b_idx_1_h) {
      obj = robot->Bodies[dynamic_simulation_B.pid_tmp];
      dynamic_simulation_B.f = Si->size[0] * Si->size[1];
      Si->size[0] = 6;
      Si->size[1] = obj->JointInternal.MotionSubspace->size[1];
      dynami_emxEnsureCapacity_real_T(Si, dynamic_simulation_B.f);
      dynamic_simulation_B.loop_ub_p = obj->JointInternal.MotionSubspace->size[0]
        * obj->JointInternal.MotionSubspace->size[1] - 1;
      for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f <=
           dynamic_simulation_B.loop_ub_p; dynamic_simulation_B.f++) {
        Si->data[dynamic_simulation_B.f] = obj->
          JointInternal.MotionSubspace->data[dynamic_simulation_B.f];
      }

      dynamic_simulation_B.n_m = Si->size[1] - 1;
      dynamic_simulation_B.f = Fi->size[0] * Fi->size[1];
      Fi->size[0] = 6;
      Fi->size[1] = Si->size[1];
      dynami_emxEnsureCapacity_real_T(Fi, dynamic_simulation_B.f);
      for (dynamic_simulation_B.loop_ub_p = 0; dynamic_simulation_B.loop_ub_p <=
           dynamic_simulation_B.n_m; dynamic_simulation_B.loop_ub_p++) {
        dynamic_simulation_B.coffset_tmp_c = dynamic_simulation_B.loop_ub_p * 6
          - 1;
        for (dynamic_simulation_B.b_i_h = 0; dynamic_simulation_B.b_i_h < 6;
             dynamic_simulation_B.b_i_h++) {
          dynamic_simulation_B.s_e = 0.0;
          for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f < 6;
               dynamic_simulation_B.f++) {
            dynamic_simulation_B.s_e += Ic->data[dynamic_simulation_B.pid_tmp].
              f1[dynamic_simulation_B.f * 6 + dynamic_simulation_B.b_i_h] *
              Si->data[(dynamic_simulation_B.coffset_tmp_c +
                        dynamic_simulation_B.f) + 1];
          }

          Fi->data[(dynamic_simulation_B.coffset_tmp_c +
                    dynamic_simulation_B.b_i_h) + 1] = dynamic_simulation_B.s_e;
        }
      }

      if (dynamic_simulation_B.vNum_p > dynamic_simulation_B.p_idx_1) {
        dynamic_simulation_B.coffset_tmp_c = 0;
        dynamic_simulation_B.cb = 0;
      } else {
        dynamic_simulation_B.coffset_tmp_c = static_cast<int32_T>
          (dynamic_simulation_B.vNum_p) - 1;
        dynamic_simulation_B.cb = static_cast<int32_T>
          (dynamic_simulation_B.vNum_p) - 1;
      }

      dynamic_simulation_B.m_a = Si->size[1];
      dynamic_simulation_B.n_m = Fi->size[1] - 1;
      dynamic_simulation_B.f = Hji->size[0] * Hji->size[1];
      Hji->size[0] = Si->size[1];
      Hji->size[1] = Fi->size[1];
      dynami_emxEnsureCapacity_real_T(Hji, dynamic_simulation_B.f);
      for (dynamic_simulation_B.loop_ub_p = 0; dynamic_simulation_B.loop_ub_p <=
           dynamic_simulation_B.n_m; dynamic_simulation_B.loop_ub_p++) {
        dynamic_simulation_B.coffset = dynamic_simulation_B.loop_ub_p *
          dynamic_simulation_B.m_a - 1;
        dynamic_simulation_B.boffset = dynamic_simulation_B.loop_ub_p * 6 - 1;
        for (dynamic_simulation_B.b_i_h = 0; dynamic_simulation_B.b_i_h <
             dynamic_simulation_B.m_a; dynamic_simulation_B.b_i_h++) {
          dynamic_simulation_B.aoffset_k = dynamic_simulation_B.b_i_h * 6 - 1;
          dynamic_simulation_B.s_e = 0.0;
          for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f < 6;
               dynamic_simulation_B.f++) {
            dynamic_simulation_B.X_tmp = dynamic_simulation_B.f + 1;
            dynamic_simulation_B.s_e += Si->data[dynamic_simulation_B.aoffset_k
              + dynamic_simulation_B.X_tmp] * Fi->
              data[dynamic_simulation_B.boffset + dynamic_simulation_B.X_tmp];
          }

          Hji->data[(dynamic_simulation_B.coffset + dynamic_simulation_B.b_i_h)
            + 1] = dynamic_simulation_B.s_e;
        }
      }

      dynamic_simulation_B.loop_ub_p = Hji->size[1];
      for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f <
           dynamic_simulation_B.loop_ub_p; dynamic_simulation_B.f++) {
        dynamic_simulation_B.n_m = Hji->size[0];
        for (dynamic_simulation_B.b_i_h = 0; dynamic_simulation_B.b_i_h <
             dynamic_simulation_B.n_m; dynamic_simulation_B.b_i_h++) {
          H->data[(dynamic_simulation_B.coffset_tmp_c +
                   dynamic_simulation_B.b_i_h) + H->size[0] *
            (dynamic_simulation_B.cb + dynamic_simulation_B.f)] = Hji->data
            [Hji->size[0] * dynamic_simulation_B.f + dynamic_simulation_B.b_i_h];
        }
      }

      dynamic_simulation_B.n_m = Fi->size[1];
      dynamic_simulation_B.f = Si->size[0] * Si->size[1];
      Si->size[0] = 6;
      Si->size[1] = Fi->size[1];
      dynami_emxEnsureCapacity_real_T(Si, dynamic_simulation_B.f);
      dynamic_simulation_B.loop_ub_p = Fi->size[0] * Fi->size[1] - 1;
      for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f <=
           dynamic_simulation_B.loop_ub_p; dynamic_simulation_B.f++) {
        Si->data[dynamic_simulation_B.f] = Fi->data[dynamic_simulation_B.f];
      }

      dynamic_simulation_B.f = Fi->size[0] * Fi->size[1];
      Fi->size[0] = 6;
      Fi->size[1] = dynamic_simulation_B.n_m;
      dynami_emxEnsureCapacity_real_T(Fi, dynamic_simulation_B.f);
      for (dynamic_simulation_B.loop_ub_p = 0; dynamic_simulation_B.loop_ub_p <
           dynamic_simulation_B.n_m; dynamic_simulation_B.loop_ub_p++) {
        dynamic_simulation_B.coffset_tmp_c = dynamic_simulation_B.loop_ub_p * 6
          - 1;
        for (dynamic_simulation_B.b_i_h = 0; dynamic_simulation_B.b_i_h < 6;
             dynamic_simulation_B.b_i_h++) {
          dynamic_simulation_B.aoffset_k = dynamic_simulation_B.b_i_h * 6 - 1;
          dynamic_simulation_B.s_e = 0.0;
          for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f < 6;
               dynamic_simulation_B.f++) {
            dynamic_simulation_B.X_tmp = dynamic_simulation_B.f + 1;
            dynamic_simulation_B.s_e += X->data[dynamic_simulation_B.pid_tmp]
              .f1[dynamic_simulation_B.aoffset_k + dynamic_simulation_B.X_tmp] *
              Si->data[dynamic_simulation_B.coffset_tmp_c +
              dynamic_simulation_B.X_tmp];
          }

          Fi->data[(dynamic_simulation_B.coffset_tmp_c +
                    dynamic_simulation_B.b_i_h) + 1] = dynamic_simulation_B.s_e;
        }
      }

      while (dynamic_simulation_B.pid > 0.0) {
        dynamic_simulation_B.b_i_h = static_cast<int32_T>
          (dynamic_simulation_B.pid);
        dynamic_simulation_B.pid_tmp = dynamic_simulation_B.b_i_h - 1;
        obj = robot->Bodies[dynamic_simulation_B.pid_tmp];
        dynamic_simulation_B.f = Si->size[0] * Si->size[1];
        Si->size[0] = 6;
        Si->size[1] = obj->JointInternal.MotionSubspace->size[1];
        dynami_emxEnsureCapacity_real_T(Si, dynamic_simulation_B.f);
        dynamic_simulation_B.loop_ub_p = obj->JointInternal.MotionSubspace->
          size[0] * obj->JointInternal.MotionSubspace->size[1] - 1;
        for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f <=
             dynamic_simulation_B.loop_ub_p; dynamic_simulation_B.f++) {
          Si->data[dynamic_simulation_B.f] = obj->
            JointInternal.MotionSubspace->data[dynamic_simulation_B.f];
        }

        dynamic_simulation_B.b_idx_0_o = robot->
          VelocityDoFMap[dynamic_simulation_B.b_i_h - 1];
        dynamic_simulation_B.b_idx_1_h = robot->
          VelocityDoFMap[dynamic_simulation_B.b_i_h + 7];
        if (dynamic_simulation_B.b_idx_0_o <= dynamic_simulation_B.b_idx_1_h) {
          dynamic_simulation_B.m_a = Si->size[1];
          dynamic_simulation_B.n_m = Fi->size[1] - 1;
          dynamic_simulation_B.f = Hji->size[0] * Hji->size[1];
          Hji->size[0] = Si->size[1];
          Hji->size[1] = Fi->size[1];
          dynami_emxEnsureCapacity_real_T(Hji, dynamic_simulation_B.f);
          for (dynamic_simulation_B.loop_ub_p = 0;
               dynamic_simulation_B.loop_ub_p <= dynamic_simulation_B.n_m;
               dynamic_simulation_B.loop_ub_p++) {
            dynamic_simulation_B.coffset = dynamic_simulation_B.loop_ub_p *
              dynamic_simulation_B.m_a - 1;
            dynamic_simulation_B.boffset = dynamic_simulation_B.loop_ub_p * 6 -
              1;
            for (dynamic_simulation_B.b_i_h = 0; dynamic_simulation_B.b_i_h <
                 dynamic_simulation_B.m_a; dynamic_simulation_B.b_i_h++) {
              dynamic_simulation_B.aoffset_k = dynamic_simulation_B.b_i_h * 6 -
                1;
              dynamic_simulation_B.s_e = 0.0;
              for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f < 6;
                   dynamic_simulation_B.f++) {
                dynamic_simulation_B.X_tmp = dynamic_simulation_B.f + 1;
                dynamic_simulation_B.s_e += Si->
                  data[dynamic_simulation_B.aoffset_k +
                  dynamic_simulation_B.X_tmp] * Fi->
                  data[dynamic_simulation_B.boffset + dynamic_simulation_B.X_tmp];
              }

              Hji->data[(dynamic_simulation_B.coffset +
                         dynamic_simulation_B.b_i_h) + 1] =
                dynamic_simulation_B.s_e;
            }
          }

          if (dynamic_simulation_B.b_idx_0_o > dynamic_simulation_B.b_idx_1_h) {
            dynamic_simulation_B.X_tmp = 0;
          } else {
            dynamic_simulation_B.X_tmp = static_cast<int32_T>
              (dynamic_simulation_B.b_idx_0_o) - 1;
          }

          if (dynamic_simulation_B.vNum_p > dynamic_simulation_B.p_idx_1) {
            dynamic_simulation_B.coffset_tmp_c = 0;
          } else {
            dynamic_simulation_B.coffset_tmp_c = static_cast<int32_T>
              (dynamic_simulation_B.vNum_p) - 1;
          }

          dynamic_simulation_B.loop_ub_p = Hji->size[1];
          for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f <
               dynamic_simulation_B.loop_ub_p; dynamic_simulation_B.f++) {
            dynamic_simulation_B.n_m = Hji->size[0];
            for (dynamic_simulation_B.b_i_h = 0; dynamic_simulation_B.b_i_h <
                 dynamic_simulation_B.n_m; dynamic_simulation_B.b_i_h++) {
              H->data[(dynamic_simulation_B.X_tmp + dynamic_simulation_B.b_i_h)
                + H->size[0] * (dynamic_simulation_B.coffset_tmp_c +
                                dynamic_simulation_B.f)] = Hji->data[Hji->size[0]
                * dynamic_simulation_B.f + dynamic_simulation_B.b_i_h];
            }
          }

          if (dynamic_simulation_B.vNum_p > dynamic_simulation_B.p_idx_1) {
            dynamic_simulation_B.X_tmp = 0;
          } else {
            dynamic_simulation_B.X_tmp = static_cast<int32_T>
              (dynamic_simulation_B.vNum_p) - 1;
          }

          if (dynamic_simulation_B.b_idx_0_o > dynamic_simulation_B.b_idx_1_h) {
            dynamic_simulation_B.coffset_tmp_c = 0;
          } else {
            dynamic_simulation_B.coffset_tmp_c = static_cast<int32_T>
              (dynamic_simulation_B.b_idx_0_o) - 1;
          }

          dynamic_simulation_B.loop_ub_p = Hji->size[0];
          for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f <
               dynamic_simulation_B.loop_ub_p; dynamic_simulation_B.f++) {
            dynamic_simulation_B.n_m = Hji->size[1];
            for (dynamic_simulation_B.b_i_h = 0; dynamic_simulation_B.b_i_h <
                 dynamic_simulation_B.n_m; dynamic_simulation_B.b_i_h++) {
              H->data[(dynamic_simulation_B.X_tmp + dynamic_simulation_B.b_i_h)
                + H->size[0] * (dynamic_simulation_B.coffset_tmp_c +
                                dynamic_simulation_B.f)] = Hji->data[Hji->size[0]
                * dynamic_simulation_B.b_i_h + dynamic_simulation_B.f];
            }
          }
        }

        dynamic_simulation_B.n_m = Fi->size[1];
        dynamic_simulation_B.f = Si->size[0] * Si->size[1];
        Si->size[0] = 6;
        Si->size[1] = Fi->size[1];
        dynami_emxEnsureCapacity_real_T(Si, dynamic_simulation_B.f);
        dynamic_simulation_B.loop_ub_p = Fi->size[0] * Fi->size[1] - 1;
        for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f <=
             dynamic_simulation_B.loop_ub_p; dynamic_simulation_B.f++) {
          Si->data[dynamic_simulation_B.f] = Fi->data[dynamic_simulation_B.f];
        }

        dynamic_simulation_B.f = Fi->size[0] * Fi->size[1];
        Fi->size[0] = 6;
        Fi->size[1] = dynamic_simulation_B.n_m;
        dynami_emxEnsureCapacity_real_T(Fi, dynamic_simulation_B.f);
        for (dynamic_simulation_B.loop_ub_p = 0; dynamic_simulation_B.loop_ub_p <
             dynamic_simulation_B.n_m; dynamic_simulation_B.loop_ub_p++) {
          dynamic_simulation_B.coffset_tmp_c = dynamic_simulation_B.loop_ub_p *
            6 - 1;
          for (dynamic_simulation_B.b_i_h = 0; dynamic_simulation_B.b_i_h < 6;
               dynamic_simulation_B.b_i_h++) {
            dynamic_simulation_B.aoffset_k = dynamic_simulation_B.b_i_h * 6 - 1;
            dynamic_simulation_B.s_e = 0.0;
            for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f < 6;
                 dynamic_simulation_B.f++) {
              dynamic_simulation_B.X_tmp = dynamic_simulation_B.f + 1;
              dynamic_simulation_B.s_e += X->data[dynamic_simulation_B.pid_tmp].
                f1[dynamic_simulation_B.aoffset_k + dynamic_simulation_B.X_tmp] *
                Si->data[dynamic_simulation_B.coffset_tmp_c +
                dynamic_simulation_B.X_tmp];
            }

            Fi->data[(dynamic_simulation_B.coffset_tmp_c +
                      dynamic_simulation_B.b_i_h) + 1] =
              dynamic_simulation_B.s_e;
          }
        }

        dynamic_simulation_B.pid = robot->Bodies[dynamic_simulation_B.pid_tmp]
          ->ParentIndex;
      }
    }
  }

  dynamic_simulati_emxFree_char_T(&a);
  dynamic_simulati_emxFree_real_T(&Hji);
  dynamic_simulati_emxFree_real_T(&Fi);
  dynamic_simulati_emxFree_real_T(&Si);
  dynamic_si_emxFree_f_cell_wrap1(&X);
  dynamic_si_emxFree_f_cell_wrap1(&Ic);
  for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f < 8;
       dynamic_simulation_B.f++) {
    dynamic_simulation_B.mask[dynamic_simulation_B.f] = (robot->
      VelocityDoFMap[dynamic_simulation_B.f] <= robot->
      VelocityDoFMap[dynamic_simulation_B.f + 8]);
  }

  dynamic_simulation_B.idx = 0;
  dynamic_simulation_B.f = 1;
  exitg1 = false;
  while ((!exitg1) && (dynamic_simulation_B.f - 1 < 8)) {
    if (dynamic_simulation_B.mask[dynamic_simulation_B.f - 1]) {
      dynamic_simulation_B.idx++;
      dynamic_simulation_B.ii_data[dynamic_simulation_B.idx - 1] =
        dynamic_simulation_B.f;
      if (dynamic_simulation_B.idx >= 8) {
        exitg1 = true;
      } else {
        dynamic_simulation_B.f++;
      }
    } else {
      dynamic_simulation_B.f++;
    }
  }

  if (1 > dynamic_simulation_B.idx) {
    dynamic_simulation_B.idx = 0;
  }

  for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f <
       dynamic_simulation_B.idx; dynamic_simulation_B.f++) {
    dynamic_simulation_B.nonFixedIndices_data[dynamic_simulation_B.f] =
      dynamic_simulation_B.ii_data[dynamic_simulation_B.f];
  }

  dynamic_simulation_B.b_i_h = dynamic_simulation_B.idx - 1;
  dynamic_simulati_emxInit_real_T(&s, 2);
  for (dynamic_simulation_B.idx = 0; dynamic_simulation_B.idx <=
       dynamic_simulation_B.b_i_h; dynamic_simulation_B.idx++) {
    dynamic_simulation_B.vNum_p = robot->
      VelocityDoFMap[dynamic_simulation_B.nonFixedIndices_data[dynamic_simulation_B.idx]
      - 1];
    dynamic_simulation_B.p_idx_1 = robot->
      VelocityDoFMap[dynamic_simulation_B.nonFixedIndices_data[dynamic_simulation_B.idx]
      + 7];
    if (rtIsNaN(dynamic_simulation_B.vNum_p) || rtIsNaN
        (dynamic_simulation_B.p_idx_1)) {
      dynamic_simulation_B.f = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = 1;
      dynami_emxEnsureCapacity_real_T(s, dynamic_simulation_B.f);
      s->data[0] = (rtNaN);
    } else if (dynamic_simulation_B.p_idx_1 < dynamic_simulation_B.vNum_p) {
      s->size[0] = 1;
      s->size[1] = 0;
    } else if ((rtIsInf(dynamic_simulation_B.vNum_p) || rtIsInf
                (dynamic_simulation_B.p_idx_1)) && (dynamic_simulation_B.vNum_p ==
                dynamic_simulation_B.p_idx_1)) {
      dynamic_simulation_B.f = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = 1;
      dynami_emxEnsureCapacity_real_T(s, dynamic_simulation_B.f);
      s->data[0] = (rtNaN);
    } else if (floor(dynamic_simulation_B.vNum_p) == dynamic_simulation_B.vNum_p)
    {
      dynamic_simulation_B.f = s->size[0] * s->size[1];
      s->size[0] = 1;
      dynamic_simulation_B.loop_ub_p = static_cast<int32_T>(floor
        (dynamic_simulation_B.p_idx_1 - dynamic_simulation_B.vNum_p));
      s->size[1] = dynamic_simulation_B.loop_ub_p + 1;
      dynami_emxEnsureCapacity_real_T(s, dynamic_simulation_B.f);
      for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f <=
           dynamic_simulation_B.loop_ub_p; dynamic_simulation_B.f++) {
        s->data[dynamic_simulation_B.f] = dynamic_simulation_B.vNum_p +
          static_cast<real_T>(dynamic_simulation_B.f);
      }
    } else {
      dynamic_simulation_B.nb_f = floor((dynamic_simulation_B.p_idx_1 -
        dynamic_simulation_B.vNum_p) + 0.5);
      dynamic_simulation_B.pid = dynamic_simulation_B.vNum_p +
        dynamic_simulation_B.nb_f;
      dynamic_simulation_B.b_idx_0_o = dynamic_simulation_B.pid -
        dynamic_simulation_B.p_idx_1;
      dynamic_simulation_B.b_idx_1_h = fabs(dynamic_simulation_B.vNum_p);
      dynamic_simulation_B.s_e = fabs(dynamic_simulation_B.p_idx_1);
      if ((dynamic_simulation_B.b_idx_1_h > dynamic_simulation_B.s_e) || rtIsNaN
          (dynamic_simulation_B.s_e)) {
        dynamic_simulation_B.s_e = dynamic_simulation_B.b_idx_1_h;
      }

      if (fabs(dynamic_simulation_B.b_idx_0_o) < 4.4408920985006262E-16 *
          dynamic_simulation_B.s_e) {
        dynamic_simulation_B.nb_f++;
        dynamic_simulation_B.pid = dynamic_simulation_B.p_idx_1;
      } else if (dynamic_simulation_B.b_idx_0_o > 0.0) {
        dynamic_simulation_B.pid = (dynamic_simulation_B.nb_f - 1.0) +
          dynamic_simulation_B.vNum_p;
      } else {
        dynamic_simulation_B.nb_f++;
      }

      if (dynamic_simulation_B.nb_f >= 0.0) {
        dynamic_simulation_B.f = static_cast<int32_T>(dynamic_simulation_B.nb_f);
      } else {
        dynamic_simulation_B.f = 0;
      }

      dynamic_simulation_B.n_m = dynamic_simulation_B.f - 1;
      dynamic_simulation_B.f = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = dynamic_simulation_B.n_m + 1;
      dynami_emxEnsureCapacity_real_T(s, dynamic_simulation_B.f);
      if (dynamic_simulation_B.n_m + 1 > 0) {
        s->data[0] = dynamic_simulation_B.vNum_p;
        if (dynamic_simulation_B.n_m + 1 > 1) {
          s->data[dynamic_simulation_B.n_m] = dynamic_simulation_B.pid;
          dynamic_simulation_B.nm1d2 = ((dynamic_simulation_B.n_m < 0) +
            dynamic_simulation_B.n_m) >> 1;
          dynamic_simulation_B.loop_ub_p = dynamic_simulation_B.nm1d2 - 2;
          for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f <=
               dynamic_simulation_B.loop_ub_p; dynamic_simulation_B.f++) {
            dynamic_simulation_B.X_tmp = dynamic_simulation_B.f + 1;
            s->data[dynamic_simulation_B.X_tmp] = dynamic_simulation_B.vNum_p +
              static_cast<real_T>(dynamic_simulation_B.X_tmp);
            s->data[dynamic_simulation_B.n_m - dynamic_simulation_B.X_tmp] =
              dynamic_simulation_B.pid - static_cast<real_T>
              (dynamic_simulation_B.X_tmp);
          }

          if (dynamic_simulation_B.nm1d2 << 1 == dynamic_simulation_B.n_m) {
            s->data[dynamic_simulation_B.nm1d2] = (dynamic_simulation_B.vNum_p +
              dynamic_simulation_B.pid) / 2.0;
          } else {
            s->data[dynamic_simulation_B.nm1d2] = dynamic_simulation_B.vNum_p +
              static_cast<real_T>(dynamic_simulation_B.nm1d2);
            s->data[dynamic_simulation_B.nm1d2 + 1] = dynamic_simulation_B.pid -
              static_cast<real_T>(dynamic_simulation_B.nm1d2);
          }
        }
      }
    }

    if (dynamic_simulation_B.vNum_p > dynamic_simulation_B.p_idx_1) {
      dynamic_simulation_B.nm1d2 = 0;
    } else {
      dynamic_simulation_B.nm1d2 = static_cast<int32_T>
        (dynamic_simulation_B.vNum_p) - 1;
    }

    dynamic_simulation_B.loop_ub_p = s->size[1];
    for (dynamic_simulation_B.f = 0; dynamic_simulation_B.f <
         dynamic_simulation_B.loop_ub_p; dynamic_simulation_B.f++) {
      lambda->data[dynamic_simulation_B.nm1d2 + dynamic_simulation_B.f] =
        s->data[dynamic_simulation_B.f] - 1.0;
    }

    if (lambda_->
        data[dynamic_simulation_B.nonFixedIndices_data[dynamic_simulation_B.idx]
        - 1] == 0.0) {
      lambda->data[static_cast<int32_T>(dynamic_simulation_B.vNum_p) - 1] = 0.0;
    } else {
      dynamic_simulation_B.f = static_cast<int32_T>(lambda_->
        data[dynamic_simulation_B.nonFixedIndices_data[dynamic_simulation_B.idx]
        - 1]);
      dynamic_simulation_B.b_idx_1_h = robot->
        VelocityDoFMap[dynamic_simulation_B.f + 7];
      lambda->data[static_cast<int32_T>(dynamic_simulation_B.vNum_p) - 1] =
        dynamic_simulation_B.b_idx_1_h;
    }
  }

  dynamic_simulati_emxFree_real_T(&s);
  dynamic_simulati_emxFree_real_T(&lambda_);
}

static void RigidBodyTreeDynamics_inverseDy(p_robotics_manip_internal__kq_T
  *robot, const real_T q[6], const real_T qdot[6], const real_T fext[48], real_T
  tau[6])
{
  emxArray_f_cell_wrap_dynami_k_T *X;
  emxArray_f_cell_wrap_dynami_k_T *Xtree;
  emxArray_real_T_dynamic_simul_T *vJ;
  emxArray_real_T_dynamic_simul_T *vB;
  emxArray_real_T_dynamic_simul_T *aB;
  emxArray_real_T_dynamic_simul_T *f;
  emxArray_real_T_dynamic_simul_T *S;
  emxArray_real_T_dynamic_simul_T *taui;
  n_robotics_manip_internal__kq_T *obj;
  emxArray_char_T_dynamic_simul_T *a;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  int32_T exitg1;
  dynamic_simulation_B.a0[0] = 0.0;
  dynamic_simulation_B.a0[1] = 0.0;
  dynamic_simulation_B.a0[2] = 0.0;
  dynamic_simulation_B.a0[3] = -robot->Gravity[0];
  dynamic_simulation_B.a0[4] = -robot->Gravity[1];
  dynamic_simulation_B.a0[5] = -robot->Gravity[2];
  dynamic_simulati_emxInit_real_T(&vJ, 2);
  dynamic_simulation_B.nb = robot->NumBodies;
  dynamic_simulation_B.i_i = vJ->size[0] * vJ->size[1];
  vJ->size[0] = 6;
  dynamic_simulation_B.unnamed_idx_1 = static_cast<int32_T>
    (dynamic_simulation_B.nb);
  vJ->size[1] = dynamic_simulation_B.unnamed_idx_1;
  dynami_emxEnsureCapacity_real_T(vJ, dynamic_simulation_B.i_i);
  dynamic_simulation_B.loop_ub_tmp = 6 * dynamic_simulation_B.unnamed_idx_1 - 1;
  for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i <=
       dynamic_simulation_B.loop_ub_tmp; dynamic_simulation_B.i_i++) {
    vJ->data[dynamic_simulation_B.i_i] = 0.0;
  }

  dynamic_simulati_emxInit_real_T(&vB, 2);
  dynamic_simulation_B.i_i = vB->size[0] * vB->size[1];
  vB->size[0] = 6;
  vB->size[1] = dynamic_simulation_B.unnamed_idx_1;
  dynami_emxEnsureCapacity_real_T(vB, dynamic_simulation_B.i_i);
  for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i <=
       dynamic_simulation_B.loop_ub_tmp; dynamic_simulation_B.i_i++) {
    vB->data[dynamic_simulation_B.i_i] = 0.0;
  }

  dynamic_simulati_emxInit_real_T(&aB, 2);
  dynamic_simulation_B.i_i = aB->size[0] * aB->size[1];
  aB->size[0] = 6;
  aB->size[1] = dynamic_simulation_B.unnamed_idx_1;
  dynami_emxEnsureCapacity_real_T(aB, dynamic_simulation_B.i_i);
  for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i <=
       dynamic_simulation_B.loop_ub_tmp; dynamic_simulation_B.i_i++) {
    aB->data[dynamic_simulation_B.i_i] = 0.0;
  }

  for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i < 6;
       dynamic_simulation_B.i_i++) {
    tau[dynamic_simulation_B.i_i] = 0.0;
  }

  dynamic_si_emxInit_f_cell_wrap1(&X, 2);
  dynamic_si_emxInit_f_cell_wrap1(&Xtree, 2);
  dynamic_simulation_B.loop_ub_tmp = dynamic_simulation_B.unnamed_idx_1 - 1;
  dynamic_simulation_B.i_i = Xtree->size[0] * Xtree->size[1];
  Xtree->size[0] = 1;
  Xtree->size[1] = dynamic_simulation_B.unnamed_idx_1;
  emxEnsureCapacity_f_cell_wrap1(Xtree, dynamic_simulation_B.i_i);
  dynamic_simulation_B.i_i = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = dynamic_simulation_B.unnamed_idx_1;
  emxEnsureCapacity_f_cell_wrap1(X, dynamic_simulation_B.i_i);
  for (dynamic_simulation_B.b_k = 0; dynamic_simulation_B.b_k <=
       dynamic_simulation_B.loop_ub_tmp; dynamic_simulation_B.b_k++) {
    for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i < 36;
         dynamic_simulation_B.i_i++) {
      Xtree->data[dynamic_simulation_B.b_k].f1[dynamic_simulation_B.i_i] = 0.0;
    }

    for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i < 6;
         dynamic_simulation_B.i_i++) {
      Xtree->data[dynamic_simulation_B.b_k].f1[dynamic_simulation_B.i_i + 6 *
        dynamic_simulation_B.i_i] = 1.0;
    }

    for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i < 36;
         dynamic_simulation_B.i_i++) {
      X->data[dynamic_simulation_B.b_k].f1[dynamic_simulation_B.i_i] = 0.0;
    }

    for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i < 6;
         dynamic_simulation_B.i_i++) {
      X->data[dynamic_simulation_B.b_k].f1[dynamic_simulation_B.i_i + 6 *
        dynamic_simulation_B.i_i] = 1.0;
    }
  }

  dynamic_simulati_emxInit_real_T(&f, 2);
  dynamic_simulation_B.i_i = f->size[0] * f->size[1];
  f->size[0] = 6;
  f->size[1] = dynamic_simulation_B.unnamed_idx_1;
  dynami_emxEnsureCapacity_real_T(f, dynamic_simulation_B.i_i);
  dynamic_simulati_emxInit_real_T(&S, 2);
  if (0 <= dynamic_simulation_B.loop_ub_tmp) {
    dynamic_simulation_B.dv1[0] = 0.0;
    dynamic_simulation_B.dv1[4] = 0.0;
    dynamic_simulation_B.dv1[8] = 0.0;
  }

  for (dynamic_simulation_B.unnamed_idx_1 = 0;
       dynamic_simulation_B.unnamed_idx_1 <= dynamic_simulation_B.loop_ub_tmp;
       dynamic_simulation_B.unnamed_idx_1++) {
    obj = robot->Bodies[dynamic_simulation_B.unnamed_idx_1];
    dynamic_simulation_B.i_i = S->size[0] * S->size[1];
    S->size[0] = 6;
    S->size[1] = obj->JointInternal.MotionSubspace->size[1];
    dynami_emxEnsureCapacity_real_T(S, dynamic_simulation_B.i_i);
    dynamic_simulation_B.b_k = obj->JointInternal.MotionSubspace->size[0] *
      obj->JointInternal.MotionSubspace->size[1] - 1;
    for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i <=
         dynamic_simulation_B.b_k; dynamic_simulation_B.i_i++) {
      S->data[dynamic_simulation_B.i_i] = obj->
        JointInternal.MotionSubspace->data[dynamic_simulation_B.i_i];
    }

    dynamic_simulation_B.a_idx_0 = robot->
      PositionDoFMap[dynamic_simulation_B.unnamed_idx_1];
    dynamic_simulation_B.a_idx_1 = robot->
      PositionDoFMap[dynamic_simulation_B.unnamed_idx_1 + 8];
    dynamic_simulation_B.b_idx_0 = robot->
      VelocityDoFMap[dynamic_simulation_B.unnamed_idx_1];
    dynamic_simulation_B.b_idx_1 = robot->
      VelocityDoFMap[dynamic_simulation_B.unnamed_idx_1 + 8];
    if (dynamic_simulation_B.a_idx_1 < dynamic_simulation_B.a_idx_0) {
      obj = robot->Bodies[dynamic_simulation_B.unnamed_idx_1];
      rigidBodyJoint_transformBodyToP(&obj->JointInternal,
        dynamic_simulation_B.T_c);
      for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i < 6;
           dynamic_simulation_B.i_i++) {
        vJ->data[dynamic_simulation_B.i_i + 6 *
          dynamic_simulation_B.unnamed_idx_1] = 0.0;
      }
    } else {
      if (dynamic_simulation_B.a_idx_0 > dynamic_simulation_B.a_idx_1) {
        dynamic_simulation_B.b_k = 0;
        dynamic_simulation_B.i_i = -1;
      } else {
        dynamic_simulation_B.b_k = static_cast<int32_T>
          (dynamic_simulation_B.a_idx_0) - 1;
        dynamic_simulation_B.i_i = static_cast<int32_T>
          (dynamic_simulation_B.a_idx_1) - 1;
      }

      if (dynamic_simulation_B.b_idx_0 > dynamic_simulation_B.b_idx_1) {
        dynamic_simulation_B.p_k = -1;
      } else {
        dynamic_simulation_B.p_k = static_cast<int32_T>
          (dynamic_simulation_B.b_idx_0) - 2;
      }

      obj = robot->Bodies[dynamic_simulation_B.unnamed_idx_1];
      dynamic_simulation_B.q_size_tmp = dynamic_simulation_B.i_i -
        dynamic_simulation_B.b_k;
      dynamic_simulation_B.q_size = dynamic_simulation_B.q_size_tmp + 1;
      for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i <=
           dynamic_simulation_B.q_size_tmp; dynamic_simulation_B.i_i++) {
        dynamic_simulation_B.q_data[dynamic_simulation_B.i_i] =
          q[dynamic_simulation_B.b_k + dynamic_simulation_B.i_i];
      }

      rigidBodyJoint_transformBodyT_k(&obj->JointInternal,
        dynamic_simulation_B.q_data, &dynamic_simulation_B.q_size,
        dynamic_simulation_B.T_c);
      dynamic_simulation_B.inner = S->size[1] - 1;
      for (dynamic_simulation_B.b_k = 0; dynamic_simulation_B.b_k < 6;
           dynamic_simulation_B.b_k++) {
        vJ->data[dynamic_simulation_B.b_k + 6 *
          dynamic_simulation_B.unnamed_idx_1] = 0.0;
      }

      for (dynamic_simulation_B.b_k = 0; dynamic_simulation_B.b_k <=
           dynamic_simulation_B.inner; dynamic_simulation_B.b_k++) {
        dynamic_simulation_B.aoffset = dynamic_simulation_B.b_k * 6 - 1;
        for (dynamic_simulation_B.q_size_tmp = 0;
             dynamic_simulation_B.q_size_tmp < 6;
             dynamic_simulation_B.q_size_tmp++) {
          dynamic_simulation_B.i_i = 6 * dynamic_simulation_B.unnamed_idx_1 +
            dynamic_simulation_B.q_size_tmp;
          vJ->data[dynamic_simulation_B.i_i] += S->data
            [(dynamic_simulation_B.aoffset + dynamic_simulation_B.q_size_tmp) +
            1] * qdot[(dynamic_simulation_B.p_k + dynamic_simulation_B.b_k) + 1];
        }
      }
    }

    for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i < 3;
         dynamic_simulation_B.i_i++) {
      dynamic_simulation_B.R_dy[3 * dynamic_simulation_B.i_i] =
        dynamic_simulation_B.T_c[dynamic_simulation_B.i_i];
      dynamic_simulation_B.R_dy[3 * dynamic_simulation_B.i_i + 1] =
        dynamic_simulation_B.T_c[dynamic_simulation_B.i_i + 4];
      dynamic_simulation_B.R_dy[3 * dynamic_simulation_B.i_i + 2] =
        dynamic_simulation_B.T_c[dynamic_simulation_B.i_i + 8];
    }

    for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i < 9;
         dynamic_simulation_B.i_i++) {
      dynamic_simulation_B.R_lx[dynamic_simulation_B.i_i] =
        -dynamic_simulation_B.R_dy[dynamic_simulation_B.i_i];
    }

    for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i < 3;
         dynamic_simulation_B.i_i++) {
      dynamic_simulation_B.b_k = dynamic_simulation_B.i_i << 2;
      dynamic_simulation_B.Tinv[dynamic_simulation_B.b_k] =
        dynamic_simulation_B.R_dy[3 * dynamic_simulation_B.i_i];
      dynamic_simulation_B.Tinv[dynamic_simulation_B.b_k + 1] =
        dynamic_simulation_B.R_dy[3 * dynamic_simulation_B.i_i + 1];
      dynamic_simulation_B.Tinv[dynamic_simulation_B.b_k + 2] =
        dynamic_simulation_B.R_dy[3 * dynamic_simulation_B.i_i + 2];
      dynamic_simulation_B.Tinv[dynamic_simulation_B.i_i + 12] =
        dynamic_simulation_B.R_lx[dynamic_simulation_B.i_i + 6] *
        dynamic_simulation_B.T_c[14] +
        (dynamic_simulation_B.R_lx[dynamic_simulation_B.i_i + 3] *
         dynamic_simulation_B.T_c[13] +
         dynamic_simulation_B.R_lx[dynamic_simulation_B.i_i] *
         dynamic_simulation_B.T_c[12]);
    }

    dynamic_simulation_B.Tinv[3] = 0.0;
    dynamic_simulation_B.Tinv[7] = 0.0;
    dynamic_simulation_B.Tinv[11] = 0.0;
    dynamic_simulation_B.Tinv[15] = 1.0;
    dynamic_simulation_B.dv1[3] = -dynamic_simulation_B.Tinv[14];
    dynamic_simulation_B.dv1[6] = dynamic_simulation_B.Tinv[13];
    dynamic_simulation_B.dv1[1] = dynamic_simulation_B.Tinv[14];
    dynamic_simulation_B.dv1[7] = -dynamic_simulation_B.Tinv[12];
    dynamic_simulation_B.dv1[2] = -dynamic_simulation_B.Tinv[13];
    dynamic_simulation_B.dv1[5] = dynamic_simulation_B.Tinv[12];
    for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i < 3;
         dynamic_simulation_B.i_i++) {
      for (dynamic_simulation_B.b_k = 0; dynamic_simulation_B.b_k < 3;
           dynamic_simulation_B.b_k++) {
        dynamic_simulation_B.q_size_tmp = dynamic_simulation_B.i_i + 3 *
          dynamic_simulation_B.b_k;
        dynamic_simulation_B.R_dy[dynamic_simulation_B.q_size_tmp] = 0.0;
        dynamic_simulation_B.p_k = dynamic_simulation_B.b_k << 2;
        dynamic_simulation_B.R_dy[dynamic_simulation_B.q_size_tmp] +=
          dynamic_simulation_B.Tinv[dynamic_simulation_B.p_k] *
          dynamic_simulation_B.dv1[dynamic_simulation_B.i_i];
        dynamic_simulation_B.R_dy[dynamic_simulation_B.q_size_tmp] +=
          dynamic_simulation_B.Tinv[dynamic_simulation_B.p_k + 1] *
          dynamic_simulation_B.dv1[dynamic_simulation_B.i_i + 3];
        dynamic_simulation_B.R_dy[dynamic_simulation_B.q_size_tmp] +=
          dynamic_simulation_B.Tinv[dynamic_simulation_B.p_k + 2] *
          dynamic_simulation_B.dv1[dynamic_simulation_B.i_i + 6];
        X->data[dynamic_simulation_B.unnamed_idx_1].f1[dynamic_simulation_B.b_k
          + 6 * dynamic_simulation_B.i_i] = dynamic_simulation_B.Tinv
          [(dynamic_simulation_B.i_i << 2) + dynamic_simulation_B.b_k];
        X->data[dynamic_simulation_B.unnamed_idx_1].f1[dynamic_simulation_B.b_k
          + 6 * (dynamic_simulation_B.i_i + 3)] = 0.0;
      }
    }

    for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i < 3;
         dynamic_simulation_B.i_i++) {
      X->data[dynamic_simulation_B.unnamed_idx_1].f1[6 *
        dynamic_simulation_B.i_i + 3] = dynamic_simulation_B.R_dy[3 *
        dynamic_simulation_B.i_i];
      dynamic_simulation_B.b_k = dynamic_simulation_B.i_i << 2;
      dynamic_simulation_B.q_size_tmp = 6 * (dynamic_simulation_B.i_i + 3);
      X->data[dynamic_simulation_B.unnamed_idx_1]
        .f1[dynamic_simulation_B.q_size_tmp + 3] =
        dynamic_simulation_B.Tinv[dynamic_simulation_B.b_k];
      X->data[dynamic_simulation_B.unnamed_idx_1].f1[6 *
        dynamic_simulation_B.i_i + 4] = dynamic_simulation_B.R_dy[3 *
        dynamic_simulation_B.i_i + 1];
      X->data[dynamic_simulation_B.unnamed_idx_1]
        .f1[dynamic_simulation_B.q_size_tmp + 4] =
        dynamic_simulation_B.Tinv[dynamic_simulation_B.b_k + 1];
      X->data[dynamic_simulation_B.unnamed_idx_1].f1[6 *
        dynamic_simulation_B.i_i + 5] = dynamic_simulation_B.R_dy[3 *
        dynamic_simulation_B.i_i + 2];
      X->data[dynamic_simulation_B.unnamed_idx_1]
        .f1[dynamic_simulation_B.q_size_tmp + 5] =
        dynamic_simulation_B.Tinv[dynamic_simulation_B.b_k + 2];
    }

    dynamic_simulation_B.a_idx_0 = robot->
      Bodies[dynamic_simulation_B.unnamed_idx_1]->ParentIndex;
    if (dynamic_simulation_B.a_idx_0 > 0.0) {
      dynamic_simulation_B.m = static_cast<int32_T>(dynamic_simulation_B.a_idx_0);
      for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i < 6;
           dynamic_simulation_B.i_i++) {
        dynamic_simulation_B.a_idx_1 = 0.0;
        for (dynamic_simulation_B.b_k = 0; dynamic_simulation_B.b_k < 6;
             dynamic_simulation_B.b_k++) {
          dynamic_simulation_B.a_idx_1 += vB->data[(dynamic_simulation_B.m - 1) *
            6 + dynamic_simulation_B.b_k] * X->
            data[dynamic_simulation_B.unnamed_idx_1].f1[6 *
            dynamic_simulation_B.b_k + dynamic_simulation_B.i_i];
        }

        dynamic_simulation_B.q_data[dynamic_simulation_B.i_i] = vJ->data[6 *
          dynamic_simulation_B.unnamed_idx_1 + dynamic_simulation_B.i_i] +
          dynamic_simulation_B.a_idx_1;
      }

      for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i < 6;
           dynamic_simulation_B.i_i++) {
        vB->data[dynamic_simulation_B.i_i + 6 *
          dynamic_simulation_B.unnamed_idx_1] =
          dynamic_simulation_B.q_data[dynamic_simulation_B.i_i];
      }

      dynamic_simulation_B.inner = S->size[1] - 1;
      for (dynamic_simulation_B.b_k = 0; dynamic_simulation_B.b_k < 6;
           dynamic_simulation_B.b_k++) {
        dynamic_simulation_B.q_data[dynamic_simulation_B.b_k] = 0.0;
      }

      for (dynamic_simulation_B.b_k = 0; dynamic_simulation_B.b_k <=
           dynamic_simulation_B.inner; dynamic_simulation_B.b_k++) {
        dynamic_simulation_B.aoffset = dynamic_simulation_B.b_k * 6 - 1;
        for (dynamic_simulation_B.q_size_tmp = 0;
             dynamic_simulation_B.q_size_tmp < 6;
             dynamic_simulation_B.q_size_tmp++) {
          dynamic_simulation_B.a_idx_1 = S->data[(dynamic_simulation_B.aoffset +
            dynamic_simulation_B.q_size_tmp) + 1] * 0.0 +
            dynamic_simulation_B.q_data[dynamic_simulation_B.q_size_tmp];
          dynamic_simulation_B.q_data[dynamic_simulation_B.q_size_tmp] =
            dynamic_simulation_B.a_idx_1;
        }
      }

      dynamic_simulation_B.R_dy[0] = 0.0;
      dynamic_simulation_B.b_k = 6 * dynamic_simulation_B.unnamed_idx_1 + 2;
      dynamic_simulation_B.R_dy[3] = -vB->data[dynamic_simulation_B.b_k];
      dynamic_simulation_B.i_i = 6 * dynamic_simulation_B.unnamed_idx_1 + 1;
      dynamic_simulation_B.R_dy[6] = vB->data[dynamic_simulation_B.i_i];
      dynamic_simulation_B.R_dy[1] = vB->data[dynamic_simulation_B.b_k];
      dynamic_simulation_B.R_dy[4] = 0.0;
      dynamic_simulation_B.R_dy[7] = -vB->data[6 *
        dynamic_simulation_B.unnamed_idx_1];
      dynamic_simulation_B.R_dy[2] = -vB->data[dynamic_simulation_B.i_i];
      dynamic_simulation_B.R_dy[5] = vB->data[6 *
        dynamic_simulation_B.unnamed_idx_1];
      dynamic_simulation_B.R_dy[8] = 0.0;
      dynamic_simulation_B.R[3] = 0.0;
      dynamic_simulation_B.b_k = 6 * dynamic_simulation_B.unnamed_idx_1 + 5;
      dynamic_simulation_B.R[9] = -vB->data[dynamic_simulation_B.b_k];
      dynamic_simulation_B.i_i = 6 * dynamic_simulation_B.unnamed_idx_1 + 4;
      dynamic_simulation_B.R[15] = vB->data[dynamic_simulation_B.i_i];
      dynamic_simulation_B.R[4] = vB->data[dynamic_simulation_B.b_k];
      dynamic_simulation_B.R[10] = 0.0;
      dynamic_simulation_B.b_k = 6 * dynamic_simulation_B.unnamed_idx_1 + 3;
      dynamic_simulation_B.R[16] = -vB->data[dynamic_simulation_B.b_k];
      dynamic_simulation_B.R[5] = -vB->data[dynamic_simulation_B.i_i];
      dynamic_simulation_B.R[11] = vB->data[dynamic_simulation_B.b_k];
      dynamic_simulation_B.R[17] = 0.0;
      for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i < 3;
           dynamic_simulation_B.i_i++) {
        dynamic_simulation_B.a_idx_1 = dynamic_simulation_B.R_dy[3 *
          dynamic_simulation_B.i_i];
        dynamic_simulation_B.R[6 * dynamic_simulation_B.i_i] =
          dynamic_simulation_B.a_idx_1;
        dynamic_simulation_B.b_k = 6 * (dynamic_simulation_B.i_i + 3);
        dynamic_simulation_B.R[dynamic_simulation_B.b_k] = 0.0;
        dynamic_simulation_B.R[dynamic_simulation_B.b_k + 3] =
          dynamic_simulation_B.a_idx_1;
        dynamic_simulation_B.a_idx_1 = dynamic_simulation_B.R_dy[3 *
          dynamic_simulation_B.i_i + 1];
        dynamic_simulation_B.R[6 * dynamic_simulation_B.i_i + 1] =
          dynamic_simulation_B.a_idx_1;
        dynamic_simulation_B.R[dynamic_simulation_B.b_k + 1] = 0.0;
        dynamic_simulation_B.R[dynamic_simulation_B.b_k + 4] =
          dynamic_simulation_B.a_idx_1;
        dynamic_simulation_B.a_idx_1 = dynamic_simulation_B.R_dy[3 *
          dynamic_simulation_B.i_i + 2];
        dynamic_simulation_B.R[6 * dynamic_simulation_B.i_i + 2] =
          dynamic_simulation_B.a_idx_1;
        dynamic_simulation_B.R[dynamic_simulation_B.b_k + 2] = 0.0;
        dynamic_simulation_B.R[dynamic_simulation_B.b_k + 5] =
          dynamic_simulation_B.a_idx_1;
      }

      for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i < 6;
           dynamic_simulation_B.i_i++) {
        dynamic_simulation_B.a_idx_1 = 0.0;
        for (dynamic_simulation_B.b_k = 0; dynamic_simulation_B.b_k < 6;
             dynamic_simulation_B.b_k++) {
          dynamic_simulation_B.a_idx_1 += aB->data[(dynamic_simulation_B.m - 1) *
            6 + dynamic_simulation_B.b_k] * X->
            data[dynamic_simulation_B.unnamed_idx_1].f1[6 *
            dynamic_simulation_B.b_k + dynamic_simulation_B.i_i];
        }

        dynamic_simulation_B.X_e[dynamic_simulation_B.i_i] =
          dynamic_simulation_B.a_idx_1 +
          dynamic_simulation_B.q_data[dynamic_simulation_B.i_i];
      }

      for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i < 6;
           dynamic_simulation_B.i_i++) {
        dynamic_simulation_B.q_data[dynamic_simulation_B.i_i] = 0.0;
        for (dynamic_simulation_B.b_k = 0; dynamic_simulation_B.b_k < 6;
             dynamic_simulation_B.b_k++) {
          dynamic_simulation_B.a_idx_1 = dynamic_simulation_B.R[6 *
            dynamic_simulation_B.b_k + dynamic_simulation_B.i_i] * vJ->data[6 *
            dynamic_simulation_B.unnamed_idx_1 + dynamic_simulation_B.b_k] +
            dynamic_simulation_B.q_data[dynamic_simulation_B.i_i];
          dynamic_simulation_B.q_data[dynamic_simulation_B.i_i] =
            dynamic_simulation_B.a_idx_1;
        }

        aB->data[dynamic_simulation_B.i_i + 6 *
          dynamic_simulation_B.unnamed_idx_1] =
          dynamic_simulation_B.X_e[dynamic_simulation_B.i_i] +
          dynamic_simulation_B.q_data[dynamic_simulation_B.i_i];
      }

      dynamic_simulation_B.R_dy[0] = 0.0;
      dynamic_simulation_B.R_dy[3] = -dynamic_simulation_B.T_c[14];
      dynamic_simulation_B.R_dy[6] = dynamic_simulation_B.T_c[13];
      dynamic_simulation_B.R_dy[1] = dynamic_simulation_B.T_c[14];
      dynamic_simulation_B.R_dy[4] = 0.0;
      dynamic_simulation_B.R_dy[7] = -dynamic_simulation_B.T_c[12];
      dynamic_simulation_B.R_dy[2] = -dynamic_simulation_B.T_c[13];
      dynamic_simulation_B.R_dy[5] = dynamic_simulation_B.T_c[12];
      dynamic_simulation_B.R_dy[8] = 0.0;
      for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i < 3;
           dynamic_simulation_B.i_i++) {
        for (dynamic_simulation_B.b_k = 0; dynamic_simulation_B.b_k < 3;
             dynamic_simulation_B.b_k++) {
          dynamic_simulation_B.q_size_tmp = dynamic_simulation_B.i_i + 3 *
            dynamic_simulation_B.b_k;
          dynamic_simulation_B.R_lx[dynamic_simulation_B.q_size_tmp] = 0.0;
          dynamic_simulation_B.p_k = dynamic_simulation_B.b_k << 2;
          dynamic_simulation_B.R_lx[dynamic_simulation_B.q_size_tmp] +=
            dynamic_simulation_B.T_c[dynamic_simulation_B.p_k] *
            dynamic_simulation_B.R_dy[dynamic_simulation_B.i_i];
          dynamic_simulation_B.R_lx[dynamic_simulation_B.q_size_tmp] +=
            dynamic_simulation_B.T_c[dynamic_simulation_B.p_k + 1] *
            dynamic_simulation_B.R_dy[dynamic_simulation_B.i_i + 3];
          dynamic_simulation_B.R_lx[dynamic_simulation_B.q_size_tmp] +=
            dynamic_simulation_B.T_c[dynamic_simulation_B.p_k + 2] *
            dynamic_simulation_B.R_dy[dynamic_simulation_B.i_i + 6];
          dynamic_simulation_B.R[dynamic_simulation_B.b_k + 6 *
            dynamic_simulation_B.i_i] = dynamic_simulation_B.T_c
            [(dynamic_simulation_B.i_i << 2) + dynamic_simulation_B.b_k];
          dynamic_simulation_B.R[dynamic_simulation_B.b_k + 6 *
            (dynamic_simulation_B.i_i + 3)] = 0.0;
        }
      }

      for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i < 3;
           dynamic_simulation_B.i_i++) {
        dynamic_simulation_B.R[6 * dynamic_simulation_B.i_i + 3] =
          dynamic_simulation_B.R_lx[3 * dynamic_simulation_B.i_i];
        dynamic_simulation_B.b_k = dynamic_simulation_B.i_i << 2;
        dynamic_simulation_B.q_size_tmp = 6 * (dynamic_simulation_B.i_i + 3);
        dynamic_simulation_B.R[dynamic_simulation_B.q_size_tmp + 3] =
          dynamic_simulation_B.T_c[dynamic_simulation_B.b_k];
        dynamic_simulation_B.R[6 * dynamic_simulation_B.i_i + 4] =
          dynamic_simulation_B.R_lx[3 * dynamic_simulation_B.i_i + 1];
        dynamic_simulation_B.R[dynamic_simulation_B.q_size_tmp + 4] =
          dynamic_simulation_B.T_c[dynamic_simulation_B.b_k + 1];
        dynamic_simulation_B.R[6 * dynamic_simulation_B.i_i + 5] =
          dynamic_simulation_B.R_lx[3 * dynamic_simulation_B.i_i + 2];
        dynamic_simulation_B.R[dynamic_simulation_B.q_size_tmp + 5] =
          dynamic_simulation_B.T_c[dynamic_simulation_B.b_k + 2];
      }

      for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i < 6;
           dynamic_simulation_B.i_i++) {
        for (dynamic_simulation_B.b_k = 0; dynamic_simulation_B.b_k < 6;
             dynamic_simulation_B.b_k++) {
          dynamic_simulation_B.p_k = dynamic_simulation_B.i_i + 6 *
            dynamic_simulation_B.b_k;
          dynamic_simulation_B.b_I[dynamic_simulation_B.p_k] = 0.0;
          for (dynamic_simulation_B.q_size_tmp = 0;
               dynamic_simulation_B.q_size_tmp < 6;
               dynamic_simulation_B.q_size_tmp++) {
            dynamic_simulation_B.b_I[dynamic_simulation_B.p_k] += Xtree->data[
              static_cast<int32_T>(dynamic_simulation_B.a_idx_0) - 1].f1[6 *
              dynamic_simulation_B.q_size_tmp + dynamic_simulation_B.i_i] *
              dynamic_simulation_B.R[6 * dynamic_simulation_B.b_k +
              dynamic_simulation_B.q_size_tmp];
          }
        }
      }

      for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i < 36;
           dynamic_simulation_B.i_i++) {
        Xtree->data[dynamic_simulation_B.unnamed_idx_1]
          .f1[dynamic_simulation_B.i_i] =
          dynamic_simulation_B.b_I[dynamic_simulation_B.i_i];
      }
    } else {
      dynamic_simulation_B.inner = S->size[1] - 1;
      for (dynamic_simulation_B.b_k = 0; dynamic_simulation_B.b_k < 6;
           dynamic_simulation_B.b_k++) {
        dynamic_simulation_B.i_i = 6 * dynamic_simulation_B.unnamed_idx_1 +
          dynamic_simulation_B.b_k;
        vB->data[dynamic_simulation_B.i_i] = vJ->data[dynamic_simulation_B.i_i];
        dynamic_simulation_B.q_data[dynamic_simulation_B.b_k] = 0.0;
      }

      for (dynamic_simulation_B.b_k = 0; dynamic_simulation_B.b_k <=
           dynamic_simulation_B.inner; dynamic_simulation_B.b_k++) {
        dynamic_simulation_B.aoffset = dynamic_simulation_B.b_k * 6 - 1;
        for (dynamic_simulation_B.q_size_tmp = 0;
             dynamic_simulation_B.q_size_tmp < 6;
             dynamic_simulation_B.q_size_tmp++) {
          dynamic_simulation_B.a_idx_1 = S->data[(dynamic_simulation_B.aoffset +
            dynamic_simulation_B.q_size_tmp) + 1] * 0.0 +
            dynamic_simulation_B.q_data[dynamic_simulation_B.q_size_tmp];
          dynamic_simulation_B.q_data[dynamic_simulation_B.q_size_tmp] =
            dynamic_simulation_B.a_idx_1;
        }
      }

      for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i < 6;
           dynamic_simulation_B.i_i++) {
        dynamic_simulation_B.a_idx_1 = 0.0;
        for (dynamic_simulation_B.b_k = 0; dynamic_simulation_B.b_k < 6;
             dynamic_simulation_B.b_k++) {
          dynamic_simulation_B.a_idx_1 += X->
            data[dynamic_simulation_B.unnamed_idx_1].f1[6 *
            dynamic_simulation_B.b_k + dynamic_simulation_B.i_i] *
            dynamic_simulation_B.a0[dynamic_simulation_B.b_k];
        }

        aB->data[dynamic_simulation_B.i_i + 6 *
          dynamic_simulation_B.unnamed_idx_1] = dynamic_simulation_B.a_idx_1 +
          dynamic_simulation_B.q_data[dynamic_simulation_B.i_i];
      }

      dynamic_simulation_B.R_dy[0] = 0.0;
      dynamic_simulation_B.R_dy[3] = -dynamic_simulation_B.T_c[14];
      dynamic_simulation_B.R_dy[6] = dynamic_simulation_B.T_c[13];
      dynamic_simulation_B.R_dy[1] = dynamic_simulation_B.T_c[14];
      dynamic_simulation_B.R_dy[4] = 0.0;
      dynamic_simulation_B.R_dy[7] = -dynamic_simulation_B.T_c[12];
      dynamic_simulation_B.R_dy[2] = -dynamic_simulation_B.T_c[13];
      dynamic_simulation_B.R_dy[5] = dynamic_simulation_B.T_c[12];
      dynamic_simulation_B.R_dy[8] = 0.0;
      for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i < 3;
           dynamic_simulation_B.i_i++) {
        for (dynamic_simulation_B.b_k = 0; dynamic_simulation_B.b_k < 3;
             dynamic_simulation_B.b_k++) {
          dynamic_simulation_B.q_size_tmp = dynamic_simulation_B.i_i + 3 *
            dynamic_simulation_B.b_k;
          dynamic_simulation_B.R_lx[dynamic_simulation_B.q_size_tmp] = 0.0;
          dynamic_simulation_B.p_k = dynamic_simulation_B.b_k << 2;
          dynamic_simulation_B.R_lx[dynamic_simulation_B.q_size_tmp] +=
            dynamic_simulation_B.T_c[dynamic_simulation_B.p_k] *
            dynamic_simulation_B.R_dy[dynamic_simulation_B.i_i];
          dynamic_simulation_B.R_lx[dynamic_simulation_B.q_size_tmp] +=
            dynamic_simulation_B.T_c[dynamic_simulation_B.p_k + 1] *
            dynamic_simulation_B.R_dy[dynamic_simulation_B.i_i + 3];
          dynamic_simulation_B.R_lx[dynamic_simulation_B.q_size_tmp] +=
            dynamic_simulation_B.T_c[dynamic_simulation_B.p_k + 2] *
            dynamic_simulation_B.R_dy[dynamic_simulation_B.i_i + 6];
          Xtree->data[dynamic_simulation_B.unnamed_idx_1]
            .f1[dynamic_simulation_B.b_k + 6 * dynamic_simulation_B.i_i] =
            dynamic_simulation_B.T_c[(dynamic_simulation_B.i_i << 2) +
            dynamic_simulation_B.b_k];
          Xtree->data[dynamic_simulation_B.unnamed_idx_1]
            .f1[dynamic_simulation_B.b_k + 6 * (dynamic_simulation_B.i_i + 3)] =
            0.0;
        }
      }

      for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i < 3;
           dynamic_simulation_B.i_i++) {
        Xtree->data[dynamic_simulation_B.unnamed_idx_1].f1[6 *
          dynamic_simulation_B.i_i + 3] = dynamic_simulation_B.R_lx[3 *
          dynamic_simulation_B.i_i];
        dynamic_simulation_B.b_k = dynamic_simulation_B.i_i << 2;
        dynamic_simulation_B.q_size_tmp = 6 * (dynamic_simulation_B.i_i + 3);
        Xtree->data[dynamic_simulation_B.unnamed_idx_1]
          .f1[dynamic_simulation_B.q_size_tmp + 3] =
          dynamic_simulation_B.T_c[dynamic_simulation_B.b_k];
        Xtree->data[dynamic_simulation_B.unnamed_idx_1].f1[6 *
          dynamic_simulation_B.i_i + 4] = dynamic_simulation_B.R_lx[3 *
          dynamic_simulation_B.i_i + 1];
        Xtree->data[dynamic_simulation_B.unnamed_idx_1]
          .f1[dynamic_simulation_B.q_size_tmp + 4] =
          dynamic_simulation_B.T_c[dynamic_simulation_B.b_k + 1];
        Xtree->data[dynamic_simulation_B.unnamed_idx_1].f1[6 *
          dynamic_simulation_B.i_i + 5] = dynamic_simulation_B.R_lx[3 *
          dynamic_simulation_B.i_i + 2];
        Xtree->data[dynamic_simulation_B.unnamed_idx_1]
          .f1[dynamic_simulation_B.q_size_tmp + 5] =
          dynamic_simulation_B.T_c[dynamic_simulation_B.b_k + 2];
      }
    }

    for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i < 36;
         dynamic_simulation_B.i_i++) {
      dynamic_simulation_B.b_I[dynamic_simulation_B.i_i] = robot->
        Bodies[dynamic_simulation_B.unnamed_idx_1]->
        SpatialInertia[dynamic_simulation_B.i_i];
    }

    dynamic_simulation_B.R_dy[0] = 0.0;
    dynamic_simulation_B.b_k = 6 * dynamic_simulation_B.unnamed_idx_1 + 2;
    dynamic_simulation_B.R_dy[3] = -vB->data[dynamic_simulation_B.b_k];
    dynamic_simulation_B.i_i = 6 * dynamic_simulation_B.unnamed_idx_1 + 1;
    dynamic_simulation_B.R_dy[6] = vB->data[dynamic_simulation_B.i_i];
    dynamic_simulation_B.R_dy[1] = vB->data[dynamic_simulation_B.b_k];
    dynamic_simulation_B.R_dy[4] = 0.0;
    dynamic_simulation_B.R_dy[7] = -vB->data[6 *
      dynamic_simulation_B.unnamed_idx_1];
    dynamic_simulation_B.R_dy[2] = -vB->data[dynamic_simulation_B.i_i];
    dynamic_simulation_B.R_dy[5] = vB->data[6 *
      dynamic_simulation_B.unnamed_idx_1];
    dynamic_simulation_B.R_dy[8] = 0.0;
    dynamic_simulation_B.R[18] = 0.0;
    dynamic_simulation_B.b_k = 6 * dynamic_simulation_B.unnamed_idx_1 + 5;
    dynamic_simulation_B.R[24] = -vB->data[dynamic_simulation_B.b_k];
    dynamic_simulation_B.i_i = 6 * dynamic_simulation_B.unnamed_idx_1 + 4;
    dynamic_simulation_B.R[30] = vB->data[dynamic_simulation_B.i_i];
    dynamic_simulation_B.R[19] = vB->data[dynamic_simulation_B.b_k];
    dynamic_simulation_B.R[25] = 0.0;
    dynamic_simulation_B.b_k = 6 * dynamic_simulation_B.unnamed_idx_1 + 3;
    dynamic_simulation_B.R[31] = -vB->data[dynamic_simulation_B.b_k];
    dynamic_simulation_B.R[20] = -vB->data[dynamic_simulation_B.i_i];
    dynamic_simulation_B.R[26] = vB->data[dynamic_simulation_B.b_k];
    dynamic_simulation_B.R[32] = 0.0;
    for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i < 3;
         dynamic_simulation_B.i_i++) {
      dynamic_simulation_B.a_idx_1 = dynamic_simulation_B.R_dy[3 *
        dynamic_simulation_B.i_i];
      dynamic_simulation_B.R[6 * dynamic_simulation_B.i_i] =
        dynamic_simulation_B.a_idx_1;
      dynamic_simulation_B.R[6 * dynamic_simulation_B.i_i + 3] = 0.0;
      dynamic_simulation_B.b_k = 6 * (dynamic_simulation_B.i_i + 3);
      dynamic_simulation_B.R[dynamic_simulation_B.b_k + 3] =
        dynamic_simulation_B.a_idx_1;
      dynamic_simulation_B.a_idx_1 = dynamic_simulation_B.R_dy[3 *
        dynamic_simulation_B.i_i + 1];
      dynamic_simulation_B.R[6 * dynamic_simulation_B.i_i + 1] =
        dynamic_simulation_B.a_idx_1;
      dynamic_simulation_B.R[6 * dynamic_simulation_B.i_i + 4] = 0.0;
      dynamic_simulation_B.R[dynamic_simulation_B.b_k + 4] =
        dynamic_simulation_B.a_idx_1;
      dynamic_simulation_B.a_idx_1 = dynamic_simulation_B.R_dy[3 *
        dynamic_simulation_B.i_i + 2];
      dynamic_simulation_B.R[6 * dynamic_simulation_B.i_i + 2] =
        dynamic_simulation_B.a_idx_1;
      dynamic_simulation_B.R[6 * dynamic_simulation_B.i_i + 5] = 0.0;
      dynamic_simulation_B.R[dynamic_simulation_B.b_k + 5] =
        dynamic_simulation_B.a_idx_1;
    }

    for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i < 6;
         dynamic_simulation_B.i_i++) {
      dynamic_simulation_B.X_e[dynamic_simulation_B.i_i] = 0.0;
      dynamic_simulation_B.b_I_b[dynamic_simulation_B.i_i] = 0.0;
      for (dynamic_simulation_B.b_k = 0; dynamic_simulation_B.b_k < 6;
           dynamic_simulation_B.b_k++) {
        dynamic_simulation_B.a_idx_0 = dynamic_simulation_B.b_I[6 *
          dynamic_simulation_B.b_k + dynamic_simulation_B.i_i];
        dynamic_simulation_B.q_size_tmp = 6 * dynamic_simulation_B.unnamed_idx_1
          + dynamic_simulation_B.b_k;
        dynamic_simulation_B.a_idx_1 = vB->data[dynamic_simulation_B.q_size_tmp]
          * dynamic_simulation_B.a_idx_0 +
          dynamic_simulation_B.X_e[dynamic_simulation_B.i_i];
        dynamic_simulation_B.a_idx_0 = aB->data[dynamic_simulation_B.q_size_tmp]
          * dynamic_simulation_B.a_idx_0 +
          dynamic_simulation_B.b_I_b[dynamic_simulation_B.i_i];
        dynamic_simulation_B.X_e[dynamic_simulation_B.i_i] =
          dynamic_simulation_B.a_idx_1;
        dynamic_simulation_B.b_I_b[dynamic_simulation_B.i_i] =
          dynamic_simulation_B.a_idx_0;
      }
    }

    for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i < 6;
         dynamic_simulation_B.i_i++) {
      dynamic_simulation_B.q_data[dynamic_simulation_B.i_i] = 0.0;
      dynamic_simulation_B.a_idx_1 = 0.0;
      for (dynamic_simulation_B.b_k = 0; dynamic_simulation_B.b_k < 6;
           dynamic_simulation_B.b_k++) {
        dynamic_simulation_B.a_idx_1 += Xtree->
          data[dynamic_simulation_B.unnamed_idx_1].f1[6 *
          dynamic_simulation_B.i_i + dynamic_simulation_B.b_k] * fext[6 *
          dynamic_simulation_B.unnamed_idx_1 + dynamic_simulation_B.b_k];
        dynamic_simulation_B.q_data[dynamic_simulation_B.i_i] +=
          dynamic_simulation_B.R[6 * dynamic_simulation_B.b_k +
          dynamic_simulation_B.i_i] *
          dynamic_simulation_B.X_e[dynamic_simulation_B.b_k];
      }

      f->data[dynamic_simulation_B.i_i + 6 * dynamic_simulation_B.unnamed_idx_1]
        = (dynamic_simulation_B.b_I_b[dynamic_simulation_B.i_i] +
           dynamic_simulation_B.q_data[dynamic_simulation_B.i_i]) -
        dynamic_simulation_B.a_idx_1;
    }
  }

  dynamic_simulati_emxFree_real_T(&aB);
  dynamic_simulati_emxFree_real_T(&vB);
  dynamic_simulati_emxFree_real_T(&vJ);
  dynamic_si_emxFree_f_cell_wrap1(&Xtree);
  dynamic_simulation_B.q_size_tmp = static_cast<int32_T>(((-1.0 -
    dynamic_simulation_B.nb) + 1.0) / -1.0) - 1;
  dynamic_simulati_emxInit_real_T(&taui, 1);
  dynamic_simulati_emxInit_char_T(&a, 2);
  if (0 <= dynamic_simulation_B.q_size_tmp) {
    for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i < 5;
         dynamic_simulation_B.i_i++) {
      dynamic_simulation_B.b_h[dynamic_simulation_B.i_i] =
        tmp[dynamic_simulation_B.i_i];
    }
  }

  for (dynamic_simulation_B.loop_ub_tmp = 0; dynamic_simulation_B.loop_ub_tmp <=
       dynamic_simulation_B.q_size_tmp; dynamic_simulation_B.loop_ub_tmp++) {
    dynamic_simulation_B.a_idx_0 = dynamic_simulation_B.nb + -static_cast<real_T>
      (dynamic_simulation_B.loop_ub_tmp);
    dynamic_simulation_B.p_k = static_cast<int32_T>(dynamic_simulation_B.a_idx_0);
    dynamic_simulation_B.inner = dynamic_simulation_B.p_k - 1;
    obj = robot->Bodies[dynamic_simulation_B.inner];
    dynamic_simulation_B.i_i = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = obj->JointInternal.Type->size[1];
    dynami_emxEnsureCapacity_char_T(a, dynamic_simulation_B.i_i);
    dynamic_simulation_B.b_k = obj->JointInternal.Type->size[0] *
      obj->JointInternal.Type->size[1] - 1;
    for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i <=
         dynamic_simulation_B.b_k; dynamic_simulation_B.i_i++) {
      a->data[dynamic_simulation_B.i_i] = obj->JointInternal.Type->
        data[dynamic_simulation_B.i_i];
    }

    dynamic_simulation_B.b_bool_i = false;
    if (a->size[1] == 5) {
      dynamic_simulation_B.i_i = 1;
      do {
        exitg1 = 0;
        if (dynamic_simulation_B.i_i - 1 < 5) {
          dynamic_simulation_B.unnamed_idx_1 = dynamic_simulation_B.i_i - 1;
          if (a->data[dynamic_simulation_B.unnamed_idx_1] !=
              dynamic_simulation_B.b_h[dynamic_simulation_B.unnamed_idx_1]) {
            exitg1 = 1;
          } else {
            dynamic_simulation_B.i_i++;
          }
        } else {
          dynamic_simulation_B.b_bool_i = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (!dynamic_simulation_B.b_bool_i) {
      obj = robot->Bodies[dynamic_simulation_B.inner];
      dynamic_simulation_B.i_i = S->size[0] * S->size[1];
      S->size[0] = 6;
      S->size[1] = obj->JointInternal.MotionSubspace->size[1];
      dynami_emxEnsureCapacity_real_T(S, dynamic_simulation_B.i_i);
      dynamic_simulation_B.b_k = obj->JointInternal.MotionSubspace->size[0] *
        obj->JointInternal.MotionSubspace->size[1] - 1;
      for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i <=
           dynamic_simulation_B.b_k; dynamic_simulation_B.i_i++) {
        S->data[dynamic_simulation_B.i_i] = obj->
          JointInternal.MotionSubspace->data[dynamic_simulation_B.i_i];
      }

      dynamic_simulation_B.m = S->size[1] - 1;
      dynamic_simulation_B.i_i = taui->size[0];
      taui->size[0] = S->size[1];
      dynami_emxEnsureCapacity_real_T(taui, dynamic_simulation_B.i_i);
      for (dynamic_simulation_B.unnamed_idx_1 = 0;
           dynamic_simulation_B.unnamed_idx_1 <= dynamic_simulation_B.m;
           dynamic_simulation_B.unnamed_idx_1++) {
        dynamic_simulation_B.aoffset = dynamic_simulation_B.unnamed_idx_1 * 6 -
          1;
        dynamic_simulation_B.a_idx_1 = 0.0;
        for (dynamic_simulation_B.b_k = 0; dynamic_simulation_B.b_k < 6;
             dynamic_simulation_B.b_k++) {
          dynamic_simulation_B.a_idx_1 += f->data[(static_cast<int32_T>
            (dynamic_simulation_B.a_idx_0) - 1) * 6 + dynamic_simulation_B.b_k] *
            S->data[(dynamic_simulation_B.aoffset + dynamic_simulation_B.b_k) +
            1];
        }

        taui->data[dynamic_simulation_B.unnamed_idx_1] =
          dynamic_simulation_B.a_idx_1;
      }

      dynamic_simulation_B.b_idx_0 = robot->
        VelocityDoFMap[dynamic_simulation_B.p_k - 1];
      dynamic_simulation_B.b_idx_1 = robot->
        VelocityDoFMap[dynamic_simulation_B.p_k + 7];
      if (dynamic_simulation_B.b_idx_0 > dynamic_simulation_B.b_idx_1) {
        dynamic_simulation_B.b_k = 0;
        dynamic_simulation_B.i_i = 0;
      } else {
        dynamic_simulation_B.b_k = static_cast<int32_T>
          (dynamic_simulation_B.b_idx_0) - 1;
        dynamic_simulation_B.i_i = static_cast<int32_T>
          (dynamic_simulation_B.b_idx_1);
      }

      dynamic_simulation_B.unnamed_idx_1 = dynamic_simulation_B.i_i -
        dynamic_simulation_B.b_k;
      for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i <
           dynamic_simulation_B.unnamed_idx_1; dynamic_simulation_B.i_i++) {
        tau[dynamic_simulation_B.b_k + dynamic_simulation_B.i_i] = taui->
          data[dynamic_simulation_B.i_i];
      }
    }

    dynamic_simulation_B.a_idx_0 = robot->Bodies[dynamic_simulation_B.inner]
      ->ParentIndex;
    if (dynamic_simulation_B.a_idx_0 > 0.0) {
      dynamic_simulation_B.m = static_cast<int32_T>(dynamic_simulation_B.a_idx_0);
      for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i < 6;
           dynamic_simulation_B.i_i++) {
        dynamic_simulation_B.a_idx_1 = 0.0;
        for (dynamic_simulation_B.b_k = 0; dynamic_simulation_B.b_k < 6;
             dynamic_simulation_B.b_k++) {
          dynamic_simulation_B.a_idx_1 += f->data[(dynamic_simulation_B.p_k - 1)
            * 6 + dynamic_simulation_B.b_k] * X->data[dynamic_simulation_B.inner]
            .f1[6 * dynamic_simulation_B.i_i + dynamic_simulation_B.b_k];
        }

        dynamic_simulation_B.a0[dynamic_simulation_B.i_i] = f->data
          [(dynamic_simulation_B.m - 1) * 6 + dynamic_simulation_B.i_i] +
          dynamic_simulation_B.a_idx_1;
      }

      for (dynamic_simulation_B.i_i = 0; dynamic_simulation_B.i_i < 6;
           dynamic_simulation_B.i_i++) {
        f->data[dynamic_simulation_B.i_i + 6 * (dynamic_simulation_B.m - 1)] =
          dynamic_simulation_B.a0[dynamic_simulation_B.i_i];
      }
    }
  }

  dynamic_simulati_emxFree_char_T(&a);
  dynamic_simulati_emxFree_real_T(&taui);
  dynamic_simulati_emxFree_real_T(&S);
  dynamic_simulati_emxFree_real_T(&f);
  dynamic_si_emxFree_f_cell_wrap1(&X);
}

// Function for MATLAB Function: '<S2>/MATLAB Function'
static boolean_T dynamic_simulation_anyNonFinite(const real_T x[16])
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

static real_T dynamic_simulatio_rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T y;
  dynamic_simulation_B.a_m = fabs(u0);
  y = fabs(u1);
  if (dynamic_simulation_B.a_m < y) {
    dynamic_simulation_B.a_m /= y;
    y *= sqrt(dynamic_simulation_B.a_m * dynamic_simulation_B.a_m + 1.0);
  } else if (dynamic_simulation_B.a_m > y) {
    y /= dynamic_simulation_B.a_m;
    y = sqrt(y * y + 1.0) * dynamic_simulation_B.a_m;
  } else {
    if (!rtIsNaN(y)) {
      y = dynamic_simulation_B.a_m * 1.4142135623730951;
    }
  }

  return y;
}

// Function for MATLAB Function: '<S2>/MATLAB Function'
static real_T dynamic_simulation_xzlangeM(const creal_T x[16])
{
  real_T y;
  real_T absxk;
  int32_T k;
  boolean_T exitg1;
  y = 0.0;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 16)) {
    absxk = dynamic_simulatio_rt_hypotd_snf(x[k].re, x[k].im);
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

// Function for MATLAB Function: '<S2>/MATLAB Function'
static void dynamic_simulation_xzlascl(real_T cfrom, real_T cto, creal_T A[16])
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

// Function for MATLAB Function: '<S2>/MATLAB Function'
static real_T dynamic_simulation_xzlanhs(const creal_T A[16], int32_T ilo,
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

// Function for MATLAB Function: '<S2>/MATLAB Function'
static void dynamic_simulation_xzlartg_d(const creal_T f, const creal_T g,
  real_T *cs, creal_T *sn)
{
  real_T scale;
  real_T g2;
  real_T f2s;
  real_T di;
  real_T x;
  real_T fs_re;
  real_T fs_im;
  real_T gs_re;
  real_T gs_im;
  boolean_T guard1 = false;
  f2s = fabs(f.re);
  di = fabs(f.im);
  scale = f2s;
  if (di > f2s) {
    scale = di;
  }

  gs_re = fabs(g.re);
  gs_im = fabs(g.im);
  if (gs_im > gs_re) {
    gs_re = gs_im;
  }

  if (gs_re > scale) {
    scale = gs_re;
  }

  fs_re = f.re;
  fs_im = f.im;
  gs_re = g.re;
  gs_im = g.im;
  guard1 = false;
  if (scale >= 7.4428285367870146E+137) {
    do {
      fs_re *= 1.3435752215134178E-138;
      fs_im *= 1.3435752215134178E-138;
      gs_re *= 1.3435752215134178E-138;
      gs_im *= 1.3435752215134178E-138;
      scale *= 1.3435752215134178E-138;
    } while (!(scale < 7.4428285367870146E+137));

    guard1 = true;
  } else if (scale <= 1.3435752215134178E-138) {
    if ((g.re == 0.0) && (g.im == 0.0)) {
      *cs = 1.0;
      sn->re = 0.0;
      sn->im = 0.0;
    } else {
      do {
        fs_re *= 7.4428285367870146E+137;
        fs_im *= 7.4428285367870146E+137;
        gs_re *= 7.4428285367870146E+137;
        gs_im *= 7.4428285367870146E+137;
        scale *= 7.4428285367870146E+137;
      } while (!(scale > 1.3435752215134178E-138));

      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    scale = fs_re * fs_re + fs_im * fs_im;
    g2 = gs_re * gs_re + gs_im * gs_im;
    x = g2;
    if (1.0 > g2) {
      x = 1.0;
    }

    if (scale <= x * 2.0041683600089728E-292) {
      if ((f.re == 0.0) && (f.im == 0.0)) {
        *cs = 0.0;
        f2s = dynamic_simulatio_rt_hypotd_snf(gs_re, gs_im);
        sn->re = gs_re / f2s;
        sn->im = -gs_im / f2s;
      } else {
        scale = sqrt(g2);
        *cs = dynamic_simulatio_rt_hypotd_snf(fs_re, fs_im) / scale;
        if (di > f2s) {
          f2s = di;
        }

        if (f2s > 1.0) {
          f2s = dynamic_simulatio_rt_hypotd_snf(f.re, f.im);
          fs_re = f.re / f2s;
          fs_im = f.im / f2s;
        } else {
          fs_re = 7.4428285367870146E+137 * f.re;
          di = 7.4428285367870146E+137 * f.im;
          f2s = dynamic_simulatio_rt_hypotd_snf(fs_re, di);
          fs_re /= f2s;
          fs_im = di / f2s;
        }

        gs_re /= scale;
        gs_im = -gs_im / scale;
        sn->re = fs_re * gs_re - fs_im * gs_im;
        sn->im = fs_re * gs_im + fs_im * gs_re;
      }
    } else {
      f2s = sqrt(g2 / scale + 1.0);
      fs_re *= f2s;
      fs_im *= f2s;
      *cs = 1.0 / f2s;
      f2s = scale + g2;
      fs_re /= f2s;
      fs_im /= f2s;
      sn->re = fs_re * gs_re - fs_im * -gs_im;
      sn->im = fs_re * -gs_im + fs_im * gs_re;
    }
  }
}

// Function for MATLAB Function: '<S2>/MATLAB Function'
static void dynamic_simulation_xzlartg(const creal_T f, const creal_T g, real_T *
  cs, creal_T *sn, creal_T *r)
{
  boolean_T guard1 = false;
  dynamic_simulation_B.f2s = fabs(f.re);
  dynamic_simulation_B.di = fabs(f.im);
  dynamic_simulation_B.scale_m = dynamic_simulation_B.f2s;
  if (dynamic_simulation_B.di > dynamic_simulation_B.f2s) {
    dynamic_simulation_B.scale_m = dynamic_simulation_B.di;
  }

  dynamic_simulation_B.gs_re = fabs(g.re);
  dynamic_simulation_B.gs_im = fabs(g.im);
  if (dynamic_simulation_B.gs_im > dynamic_simulation_B.gs_re) {
    dynamic_simulation_B.gs_re = dynamic_simulation_B.gs_im;
  }

  if (dynamic_simulation_B.gs_re > dynamic_simulation_B.scale_m) {
    dynamic_simulation_B.scale_m = dynamic_simulation_B.gs_re;
  }

  dynamic_simulation_B.fs_re = f.re;
  dynamic_simulation_B.fs_im = f.im;
  dynamic_simulation_B.gs_re = g.re;
  dynamic_simulation_B.gs_im = g.im;
  dynamic_simulation_B.count = -1;
  dynamic_simulation_B.rescaledir = 0;
  guard1 = false;
  if (dynamic_simulation_B.scale_m >= 7.4428285367870146E+137) {
    do {
      dynamic_simulation_B.count++;
      dynamic_simulation_B.fs_re *= 1.3435752215134178E-138;
      dynamic_simulation_B.fs_im *= 1.3435752215134178E-138;
      dynamic_simulation_B.gs_re *= 1.3435752215134178E-138;
      dynamic_simulation_B.gs_im *= 1.3435752215134178E-138;
      dynamic_simulation_B.scale_m *= 1.3435752215134178E-138;
    } while (!(dynamic_simulation_B.scale_m < 7.4428285367870146E+137));

    dynamic_simulation_B.rescaledir = 1;
    guard1 = true;
  } else if (dynamic_simulation_B.scale_m <= 1.3435752215134178E-138) {
    if ((g.re == 0.0) && (g.im == 0.0)) {
      *cs = 1.0;
      sn->re = 0.0;
      sn->im = 0.0;
      *r = f;
    } else {
      do {
        dynamic_simulation_B.count++;
        dynamic_simulation_B.fs_re *= 7.4428285367870146E+137;
        dynamic_simulation_B.fs_im *= 7.4428285367870146E+137;
        dynamic_simulation_B.gs_re *= 7.4428285367870146E+137;
        dynamic_simulation_B.gs_im *= 7.4428285367870146E+137;
        dynamic_simulation_B.scale_m *= 7.4428285367870146E+137;
      } while (!(dynamic_simulation_B.scale_m > 1.3435752215134178E-138));

      dynamic_simulation_B.rescaledir = -1;
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    dynamic_simulation_B.scale_m = dynamic_simulation_B.fs_re *
      dynamic_simulation_B.fs_re + dynamic_simulation_B.fs_im *
      dynamic_simulation_B.fs_im;
    dynamic_simulation_B.g2 = dynamic_simulation_B.gs_re *
      dynamic_simulation_B.gs_re + dynamic_simulation_B.gs_im *
      dynamic_simulation_B.gs_im;
    dynamic_simulation_B.x = dynamic_simulation_B.g2;
    if (1.0 > dynamic_simulation_B.g2) {
      dynamic_simulation_B.x = 1.0;
    }

    if (dynamic_simulation_B.scale_m <= dynamic_simulation_B.x *
        2.0041683600089728E-292) {
      if ((f.re == 0.0) && (f.im == 0.0)) {
        *cs = 0.0;
        r->re = dynamic_simulatio_rt_hypotd_snf(g.re, g.im);
        r->im = 0.0;
        dynamic_simulation_B.f2s = dynamic_simulatio_rt_hypotd_snf
          (dynamic_simulation_B.gs_re, dynamic_simulation_B.gs_im);
        sn->re = dynamic_simulation_B.gs_re / dynamic_simulation_B.f2s;
        sn->im = -dynamic_simulation_B.gs_im / dynamic_simulation_B.f2s;
      } else {
        dynamic_simulation_B.scale_m = sqrt(dynamic_simulation_B.g2);
        *cs = dynamic_simulatio_rt_hypotd_snf(dynamic_simulation_B.fs_re,
          dynamic_simulation_B.fs_im) / dynamic_simulation_B.scale_m;
        if (dynamic_simulation_B.di > dynamic_simulation_B.f2s) {
          dynamic_simulation_B.f2s = dynamic_simulation_B.di;
        }

        if (dynamic_simulation_B.f2s > 1.0) {
          dynamic_simulation_B.f2s = dynamic_simulatio_rt_hypotd_snf(f.re, f.im);
          dynamic_simulation_B.fs_re = f.re / dynamic_simulation_B.f2s;
          dynamic_simulation_B.fs_im = f.im / dynamic_simulation_B.f2s;
        } else {
          dynamic_simulation_B.fs_re = 7.4428285367870146E+137 * f.re;
          dynamic_simulation_B.di = 7.4428285367870146E+137 * f.im;
          dynamic_simulation_B.f2s = dynamic_simulatio_rt_hypotd_snf
            (dynamic_simulation_B.fs_re, dynamic_simulation_B.di);
          dynamic_simulation_B.fs_re /= dynamic_simulation_B.f2s;
          dynamic_simulation_B.fs_im = dynamic_simulation_B.di /
            dynamic_simulation_B.f2s;
        }

        dynamic_simulation_B.gs_re /= dynamic_simulation_B.scale_m;
        dynamic_simulation_B.gs_im = -dynamic_simulation_B.gs_im /
          dynamic_simulation_B.scale_m;
        sn->re = dynamic_simulation_B.fs_re * dynamic_simulation_B.gs_re -
          dynamic_simulation_B.fs_im * dynamic_simulation_B.gs_im;
        sn->im = dynamic_simulation_B.fs_re * dynamic_simulation_B.gs_im +
          dynamic_simulation_B.fs_im * dynamic_simulation_B.gs_re;
        r->re = (sn->re * g.re - sn->im * g.im) + *cs * f.re;
        r->im = (sn->re * g.im + sn->im * g.re) + *cs * f.im;
      }
    } else {
      dynamic_simulation_B.f2s = sqrt(dynamic_simulation_B.g2 /
        dynamic_simulation_B.scale_m + 1.0);
      r->re = dynamic_simulation_B.f2s * dynamic_simulation_B.fs_re;
      r->im = dynamic_simulation_B.f2s * dynamic_simulation_B.fs_im;
      *cs = 1.0 / dynamic_simulation_B.f2s;
      dynamic_simulation_B.f2s = dynamic_simulation_B.scale_m +
        dynamic_simulation_B.g2;
      dynamic_simulation_B.fs_re = r->re / dynamic_simulation_B.f2s;
      dynamic_simulation_B.f2s = r->im / dynamic_simulation_B.f2s;
      sn->re = dynamic_simulation_B.fs_re * dynamic_simulation_B.gs_re -
        dynamic_simulation_B.f2s * -dynamic_simulation_B.gs_im;
      sn->im = dynamic_simulation_B.fs_re * -dynamic_simulation_B.gs_im +
        dynamic_simulation_B.f2s * dynamic_simulation_B.gs_re;
      if (dynamic_simulation_B.rescaledir > 0) {
        dynamic_simulation_B.rescaledir = 0;
        while (dynamic_simulation_B.rescaledir <= dynamic_simulation_B.count) {
          r->re *= 7.4428285367870146E+137;
          r->im *= 7.4428285367870146E+137;
          dynamic_simulation_B.rescaledir++;
        }
      } else {
        if (dynamic_simulation_B.rescaledir < 0) {
          dynamic_simulation_B.rescaledir = 0;
          while (dynamic_simulation_B.rescaledir <= dynamic_simulation_B.count)
          {
            r->re *= 1.3435752215134178E-138;
            r->im *= 1.3435752215134178E-138;
            dynamic_simulation_B.rescaledir++;
          }
        }
      }
    }
  }
}

// Function for MATLAB Function: '<S2>/MATLAB Function'
static void dynamic_simulation_xzhgeqz(creal_T A[16], int32_T ilo, int32_T ihi,
  creal_T Z[16], int32_T *info, creal_T alpha1[4], creal_T beta1[4])
{
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
  dynamic_simulation_B.eshift_re = 0.0;
  dynamic_simulation_B.eshift_im = 0.0;
  dynamic_simulation_B.ctemp.re = 0.0;
  dynamic_simulation_B.ctemp.im = 0.0;
  dynamic_simulation_B.anorm = dynamic_simulation_xzlanhs(A, ilo, ihi);
  dynamic_simulation_B.shift_re = 2.2204460492503131E-16 *
    dynamic_simulation_B.anorm;
  dynamic_simulation_B.b_atol = 2.2250738585072014E-308;
  if (dynamic_simulation_B.shift_re > 2.2250738585072014E-308) {
    dynamic_simulation_B.b_atol = dynamic_simulation_B.shift_re;
  }

  dynamic_simulation_B.shift_re = 2.2250738585072014E-308;
  if (dynamic_simulation_B.anorm > 2.2250738585072014E-308) {
    dynamic_simulation_B.shift_re = dynamic_simulation_B.anorm;
  }

  dynamic_simulation_B.anorm = 1.0 / dynamic_simulation_B.shift_re;
  dynamic_simulation_B.failed = true;
  dynamic_simulation_B.ilast = ihi;
  while (dynamic_simulation_B.ilast + 1 < 5) {
    alpha1[dynamic_simulation_B.ilast] = A[(dynamic_simulation_B.ilast << 2) +
      dynamic_simulation_B.ilast];
    dynamic_simulation_B.ilast++;
  }

  guard1 = false;
  guard2 = false;
  if (ihi >= ilo) {
    dynamic_simulation_B.ifirst = ilo;
    dynamic_simulation_B.istart = ilo;
    dynamic_simulation_B.ilast = ihi - 1;
    dynamic_simulation_B.ilastm1 = ihi - 2;
    dynamic_simulation_B.iiter = 0;
    dynamic_simulation_B.goto60 = false;
    dynamic_simulation_B.goto70 = false;
    dynamic_simulation_B.goto90 = false;
    dynamic_simulation_B.jiter = 0;
    do {
      exitg1 = 0;
      if (dynamic_simulation_B.jiter <= ((ihi - ilo) + 1) * 30 - 1) {
        if (dynamic_simulation_B.ilast + 1 == ilo) {
          dynamic_simulation_B.goto60 = true;
        } else {
          dynamic_simulation_B.jp1 = (dynamic_simulation_B.ilastm1 << 2) +
            dynamic_simulation_B.ilast;
          if (fabs(A[dynamic_simulation_B.jp1].re) + fabs
              (A[dynamic_simulation_B.jp1].im) <= dynamic_simulation_B.b_atol) {
            A[dynamic_simulation_B.jp1].re = 0.0;
            A[dynamic_simulation_B.jp1].im = 0.0;
            dynamic_simulation_B.goto60 = true;
          } else {
            dynamic_simulation_B.j_j = dynamic_simulation_B.ilastm1;
            guard3 = false;
            exitg2 = false;
            while ((!exitg2) && (dynamic_simulation_B.j_j + 1 >= ilo)) {
              if (dynamic_simulation_B.j_j + 1 == ilo) {
                guard3 = true;
                exitg2 = true;
              } else {
                dynamic_simulation_B.jp1 = ((dynamic_simulation_B.j_j - 1) << 2)
                  + dynamic_simulation_B.j_j;
                if (fabs(A[dynamic_simulation_B.jp1].re) + fabs
                    (A[dynamic_simulation_B.jp1].im) <=
                    dynamic_simulation_B.b_atol) {
                  A[dynamic_simulation_B.jp1].re = 0.0;
                  A[dynamic_simulation_B.jp1].im = 0.0;
                  guard3 = true;
                  exitg2 = true;
                } else {
                  dynamic_simulation_B.j_j--;
                  guard3 = false;
                }
              }
            }

            if (guard3) {
              dynamic_simulation_B.ifirst = dynamic_simulation_B.j_j + 1;
              dynamic_simulation_B.goto70 = true;
            }
          }
        }

        if ((!dynamic_simulation_B.goto60) && (!dynamic_simulation_B.goto70)) {
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
          for (dynamic_simulation_B.jp1 = 0; dynamic_simulation_B.jp1 < 16;
               dynamic_simulation_B.jp1++) {
            Z[dynamic_simulation_B.jp1].re = (rtNaN);
            Z[dynamic_simulation_B.jp1].im = 0.0;
          }

          *info = 1;
          exitg1 = 1;
        } else if (dynamic_simulation_B.goto60) {
          dynamic_simulation_B.goto60 = false;
          alpha1[dynamic_simulation_B.ilast] = A[(dynamic_simulation_B.ilast <<
            2) + dynamic_simulation_B.ilast];
          dynamic_simulation_B.ilast = dynamic_simulation_B.ilastm1;
          dynamic_simulation_B.ilastm1--;
          if (dynamic_simulation_B.ilast + 1 < ilo) {
            dynamic_simulation_B.failed = false;
            guard2 = true;
            exitg1 = 1;
          } else {
            dynamic_simulation_B.iiter = 0;
            dynamic_simulation_B.eshift_re = 0.0;
            dynamic_simulation_B.eshift_im = 0.0;
            dynamic_simulation_B.jiter++;
          }
        } else {
          if (dynamic_simulation_B.goto70) {
            dynamic_simulation_B.goto70 = false;
            dynamic_simulation_B.iiter++;
            if (dynamic_simulation_B.iiter - div_nzp_s32
                (dynamic_simulation_B.iiter, 10) * 10 != 0) {
              dynamic_simulation_B.j_j = (dynamic_simulation_B.ilastm1 << 2) +
                dynamic_simulation_B.ilastm1;
              dynamic_simulation_B.ar = A[dynamic_simulation_B.j_j].re *
                dynamic_simulation_B.anorm;
              dynamic_simulation_B.ai = A[dynamic_simulation_B.j_j].im *
                dynamic_simulation_B.anorm;
              if (dynamic_simulation_B.ai == 0.0) {
                dynamic_simulation_B.shift_re = dynamic_simulation_B.ar / 0.5;
                dynamic_simulation_B.shift_im = 0.0;
              } else if (dynamic_simulation_B.ar == 0.0) {
                dynamic_simulation_B.shift_re = 0.0;
                dynamic_simulation_B.shift_im = dynamic_simulation_B.ai / 0.5;
              } else {
                dynamic_simulation_B.shift_re = dynamic_simulation_B.ar / 0.5;
                dynamic_simulation_B.shift_im = dynamic_simulation_B.ai / 0.5;
              }

              dynamic_simulation_B.j_j = (dynamic_simulation_B.ilast << 2) +
                dynamic_simulation_B.ilast;
              dynamic_simulation_B.ar = A[dynamic_simulation_B.j_j].re *
                dynamic_simulation_B.anorm;
              dynamic_simulation_B.ai = A[dynamic_simulation_B.j_j].im *
                dynamic_simulation_B.anorm;
              if (dynamic_simulation_B.ai == 0.0) {
                dynamic_simulation_B.ad22.re = dynamic_simulation_B.ar / 0.5;
                dynamic_simulation_B.ad22.im = 0.0;
              } else if (dynamic_simulation_B.ar == 0.0) {
                dynamic_simulation_B.ad22.re = 0.0;
                dynamic_simulation_B.ad22.im = dynamic_simulation_B.ai / 0.5;
              } else {
                dynamic_simulation_B.ad22.re = dynamic_simulation_B.ar / 0.5;
                dynamic_simulation_B.ad22.im = dynamic_simulation_B.ai / 0.5;
              }

              dynamic_simulation_B.t1_re = (dynamic_simulation_B.shift_re +
                dynamic_simulation_B.ad22.re) * 0.5;
              dynamic_simulation_B.t1_im = (dynamic_simulation_B.shift_im +
                dynamic_simulation_B.ad22.im) * 0.5;
              dynamic_simulation_B.j_j = (dynamic_simulation_B.ilast << 2) +
                dynamic_simulation_B.ilastm1;
              dynamic_simulation_B.ar = A[dynamic_simulation_B.j_j].re *
                dynamic_simulation_B.anorm;
              dynamic_simulation_B.ai = A[dynamic_simulation_B.j_j].im *
                dynamic_simulation_B.anorm;
              if (dynamic_simulation_B.ai == 0.0) {
                dynamic_simulation_B.absxr = dynamic_simulation_B.ar / 0.5;
                dynamic_simulation_B.absxi = 0.0;
              } else if (dynamic_simulation_B.ar == 0.0) {
                dynamic_simulation_B.absxr = 0.0;
                dynamic_simulation_B.absxi = dynamic_simulation_B.ai / 0.5;
              } else {
                dynamic_simulation_B.absxr = dynamic_simulation_B.ar / 0.5;
                dynamic_simulation_B.absxi = dynamic_simulation_B.ai / 0.5;
              }

              dynamic_simulation_B.j_j = (dynamic_simulation_B.ilastm1 << 2) +
                dynamic_simulation_B.ilast;
              dynamic_simulation_B.ar = A[dynamic_simulation_B.j_j].re *
                dynamic_simulation_B.anorm;
              dynamic_simulation_B.ai = A[dynamic_simulation_B.j_j].im *
                dynamic_simulation_B.anorm;
              if (dynamic_simulation_B.ai == 0.0) {
                dynamic_simulation_B.ar /= 0.5;
                dynamic_simulation_B.ai = 0.0;
              } else if (dynamic_simulation_B.ar == 0.0) {
                dynamic_simulation_B.ar = 0.0;
                dynamic_simulation_B.ai /= 0.5;
              } else {
                dynamic_simulation_B.ar /= 0.5;
                dynamic_simulation_B.ai /= 0.5;
              }

              dynamic_simulation_B.shift_im_h = dynamic_simulation_B.shift_re *
                dynamic_simulation_B.ad22.im + dynamic_simulation_B.shift_im *
                dynamic_simulation_B.ad22.re;
              dynamic_simulation_B.shift_re = ((dynamic_simulation_B.t1_re *
                dynamic_simulation_B.t1_re - dynamic_simulation_B.t1_im *
                dynamic_simulation_B.t1_im) + (dynamic_simulation_B.absxr *
                dynamic_simulation_B.ar - dynamic_simulation_B.absxi *
                dynamic_simulation_B.ai)) - (dynamic_simulation_B.shift_re *
                dynamic_simulation_B.ad22.re - dynamic_simulation_B.shift_im *
                dynamic_simulation_B.ad22.im);
              dynamic_simulation_B.shift_im = dynamic_simulation_B.t1_re *
                dynamic_simulation_B.t1_im;
              dynamic_simulation_B.shift_im = ((dynamic_simulation_B.shift_im +
                dynamic_simulation_B.shift_im) + (dynamic_simulation_B.absxr *
                dynamic_simulation_B.ai + dynamic_simulation_B.absxi *
                dynamic_simulation_B.ar)) - dynamic_simulation_B.shift_im_h;
              if (dynamic_simulation_B.shift_im == 0.0) {
                if (dynamic_simulation_B.shift_re < 0.0) {
                  dynamic_simulation_B.absxr = 0.0;
                  dynamic_simulation_B.absxi = sqrt
                    (-dynamic_simulation_B.shift_re);
                } else {
                  dynamic_simulation_B.absxr = sqrt
                    (dynamic_simulation_B.shift_re);
                  dynamic_simulation_B.absxi = 0.0;
                }
              } else if (dynamic_simulation_B.shift_re == 0.0) {
                if (dynamic_simulation_B.shift_im < 0.0) {
                  dynamic_simulation_B.absxr = sqrt
                    (-dynamic_simulation_B.shift_im / 2.0);
                  dynamic_simulation_B.absxi = -dynamic_simulation_B.absxr;
                } else {
                  dynamic_simulation_B.absxr = sqrt
                    (dynamic_simulation_B.shift_im / 2.0);
                  dynamic_simulation_B.absxi = dynamic_simulation_B.absxr;
                }
              } else if (rtIsNaN(dynamic_simulation_B.shift_re)) {
                dynamic_simulation_B.absxr = dynamic_simulation_B.shift_re;
                dynamic_simulation_B.absxi = dynamic_simulation_B.shift_re;
              } else if (rtIsNaN(dynamic_simulation_B.shift_im)) {
                dynamic_simulation_B.absxr = dynamic_simulation_B.shift_im;
                dynamic_simulation_B.absxi = dynamic_simulation_B.shift_im;
              } else if (rtIsInf(dynamic_simulation_B.shift_im)) {
                dynamic_simulation_B.absxr = fabs(dynamic_simulation_B.shift_im);
                dynamic_simulation_B.absxi = dynamic_simulation_B.shift_im;
              } else if (rtIsInf(dynamic_simulation_B.shift_re)) {
                if (dynamic_simulation_B.shift_re < 0.0) {
                  dynamic_simulation_B.absxr = 0.0;
                  dynamic_simulation_B.absxi = dynamic_simulation_B.shift_im *
                    -dynamic_simulation_B.shift_re;
                } else {
                  dynamic_simulation_B.absxr = dynamic_simulation_B.shift_re;
                  dynamic_simulation_B.absxi = 0.0;
                }
              } else {
                dynamic_simulation_B.absxr = fabs(dynamic_simulation_B.shift_re);
                dynamic_simulation_B.absxi = fabs(dynamic_simulation_B.shift_im);
                if ((dynamic_simulation_B.absxr > 4.4942328371557893E+307) ||
                    (dynamic_simulation_B.absxi > 4.4942328371557893E+307)) {
                  dynamic_simulation_B.absxr *= 0.5;
                  dynamic_simulation_B.absxi *= 0.5;
                  dynamic_simulation_B.absxi = dynamic_simulatio_rt_hypotd_snf
                    (dynamic_simulation_B.absxr, dynamic_simulation_B.absxi);
                  if (dynamic_simulation_B.absxi > dynamic_simulation_B.absxr) {
                    dynamic_simulation_B.absxr = sqrt(dynamic_simulation_B.absxr
                      / dynamic_simulation_B.absxi + 1.0) * sqrt
                      (dynamic_simulation_B.absxi);
                  } else {
                    dynamic_simulation_B.absxr = sqrt(dynamic_simulation_B.absxi)
                      * 1.4142135623730951;
                  }
                } else {
                  dynamic_simulation_B.absxr = sqrt
                    ((dynamic_simulatio_rt_hypotd_snf(dynamic_simulation_B.absxr,
                       dynamic_simulation_B.absxi) + dynamic_simulation_B.absxr)
                     * 0.5);
                }

                if (dynamic_simulation_B.shift_re > 0.0) {
                  dynamic_simulation_B.absxi = dynamic_simulation_B.shift_im /
                    dynamic_simulation_B.absxr * 0.5;
                } else {
                  if (dynamic_simulation_B.shift_im < 0.0) {
                    dynamic_simulation_B.absxi = -dynamic_simulation_B.absxr;
                  } else {
                    dynamic_simulation_B.absxi = dynamic_simulation_B.absxr;
                  }

                  dynamic_simulation_B.absxr = dynamic_simulation_B.shift_im /
                    dynamic_simulation_B.absxi * 0.5;
                }
              }

              if ((dynamic_simulation_B.t1_re - dynamic_simulation_B.ad22.re) *
                  dynamic_simulation_B.absxr + (dynamic_simulation_B.t1_im -
                   dynamic_simulation_B.ad22.im) * dynamic_simulation_B.absxi <=
                  0.0) {
                dynamic_simulation_B.shift_re = dynamic_simulation_B.t1_re +
                  dynamic_simulation_B.absxr;
                dynamic_simulation_B.shift_im = dynamic_simulation_B.t1_im +
                  dynamic_simulation_B.absxi;
              } else {
                dynamic_simulation_B.shift_re = dynamic_simulation_B.t1_re -
                  dynamic_simulation_B.absxr;
                dynamic_simulation_B.shift_im = dynamic_simulation_B.t1_im -
                  dynamic_simulation_B.absxi;
              }
            } else {
              dynamic_simulation_B.j_j = (dynamic_simulation_B.ilastm1 << 2) +
                dynamic_simulation_B.ilast;
              dynamic_simulation_B.ar = A[dynamic_simulation_B.j_j].re *
                dynamic_simulation_B.anorm;
              dynamic_simulation_B.ai = A[dynamic_simulation_B.j_j].im *
                dynamic_simulation_B.anorm;
              if (dynamic_simulation_B.ai == 0.0) {
                dynamic_simulation_B.absxr = dynamic_simulation_B.ar / 0.5;
                dynamic_simulation_B.absxi = 0.0;
              } else if (dynamic_simulation_B.ar == 0.0) {
                dynamic_simulation_B.absxr = 0.0;
                dynamic_simulation_B.absxi = dynamic_simulation_B.ai / 0.5;
              } else {
                dynamic_simulation_B.absxr = dynamic_simulation_B.ar / 0.5;
                dynamic_simulation_B.absxi = dynamic_simulation_B.ai / 0.5;
              }

              dynamic_simulation_B.eshift_re += dynamic_simulation_B.absxr;
              dynamic_simulation_B.eshift_im += dynamic_simulation_B.absxi;
              dynamic_simulation_B.shift_re = dynamic_simulation_B.eshift_re;
              dynamic_simulation_B.shift_im = dynamic_simulation_B.eshift_im;
            }

            dynamic_simulation_B.j_j = dynamic_simulation_B.ilastm1;
            dynamic_simulation_B.jp1 = dynamic_simulation_B.ilastm1 + 1;
            exitg2 = false;
            while ((!exitg2) && (dynamic_simulation_B.j_j + 1 >
                                 dynamic_simulation_B.ifirst)) {
              dynamic_simulation_B.istart = dynamic_simulation_B.j_j + 1;
              dynamic_simulation_B.ctemp_tmp_tmp = dynamic_simulation_B.j_j << 2;
              dynamic_simulation_B.ctemp_tmp =
                dynamic_simulation_B.ctemp_tmp_tmp + dynamic_simulation_B.j_j;
              dynamic_simulation_B.ctemp.re = A[dynamic_simulation_B.ctemp_tmp].
                re * dynamic_simulation_B.anorm - dynamic_simulation_B.shift_re *
                0.5;
              dynamic_simulation_B.ctemp.im = A[dynamic_simulation_B.ctemp_tmp].
                im * dynamic_simulation_B.anorm - dynamic_simulation_B.shift_im *
                0.5;
              dynamic_simulation_B.t1_re = fabs(dynamic_simulation_B.ctemp.re) +
                fabs(dynamic_simulation_B.ctemp.im);
              dynamic_simulation_B.jp1 += dynamic_simulation_B.ctemp_tmp_tmp;
              dynamic_simulation_B.t1_im = (fabs(A[dynamic_simulation_B.jp1].re)
                + fabs(A[dynamic_simulation_B.jp1].im)) *
                dynamic_simulation_B.anorm;
              dynamic_simulation_B.absxr = dynamic_simulation_B.t1_re;
              if (dynamic_simulation_B.t1_im > dynamic_simulation_B.t1_re) {
                dynamic_simulation_B.absxr = dynamic_simulation_B.t1_im;
              }

              if ((dynamic_simulation_B.absxr < 1.0) &&
                  (dynamic_simulation_B.absxr != 0.0)) {
                dynamic_simulation_B.t1_re /= dynamic_simulation_B.absxr;
                dynamic_simulation_B.t1_im /= dynamic_simulation_B.absxr;
              }

              dynamic_simulation_B.jp1 = ((dynamic_simulation_B.j_j - 1) << 2) +
                dynamic_simulation_B.j_j;
              if ((fabs(A[dynamic_simulation_B.jp1].re) + fabs
                   (A[dynamic_simulation_B.jp1].im)) *
                  dynamic_simulation_B.t1_im <= dynamic_simulation_B.t1_re *
                  dynamic_simulation_B.b_atol) {
                dynamic_simulation_B.goto90 = true;
                exitg2 = true;
              } else {
                dynamic_simulation_B.jp1 = dynamic_simulation_B.j_j;
                dynamic_simulation_B.j_j--;
              }
            }

            if (!dynamic_simulation_B.goto90) {
              dynamic_simulation_B.istart = dynamic_simulation_B.ifirst;
              dynamic_simulation_B.ctemp_tmp = (((dynamic_simulation_B.ifirst -
                1) << 2) + dynamic_simulation_B.ifirst) - 1;
              dynamic_simulation_B.ctemp.re = A[dynamic_simulation_B.ctemp_tmp].
                re * dynamic_simulation_B.anorm - dynamic_simulation_B.shift_re *
                0.5;
              dynamic_simulation_B.ctemp.im = A[dynamic_simulation_B.ctemp_tmp].
                im * dynamic_simulation_B.anorm - dynamic_simulation_B.shift_im *
                0.5;
            }

            dynamic_simulation_B.goto90 = false;
            dynamic_simulation_B.j_j = ((dynamic_simulation_B.istart - 1) << 2)
              + dynamic_simulation_B.istart;
            dynamic_simulation_B.ascale.re = A[dynamic_simulation_B.j_j].re *
              dynamic_simulation_B.anorm;
            dynamic_simulation_B.ascale.im = A[dynamic_simulation_B.j_j].im *
              dynamic_simulation_B.anorm;
            dynamic_simulation_xzlartg_d(dynamic_simulation_B.ctemp,
              dynamic_simulation_B.ascale, &dynamic_simulation_B.t1_re,
              &dynamic_simulation_B.ad22);
            dynamic_simulation_B.j_j = dynamic_simulation_B.istart;
            dynamic_simulation_B.jp1 = dynamic_simulation_B.istart - 2;
            while (dynamic_simulation_B.j_j < dynamic_simulation_B.ilast + 1) {
              if (dynamic_simulation_B.j_j > dynamic_simulation_B.istart) {
                dynamic_simulation_xzlartg(A[(dynamic_simulation_B.j_j +
                  (dynamic_simulation_B.jp1 << 2)) - 1],
                  A[dynamic_simulation_B.j_j + (dynamic_simulation_B.jp1 << 2)],
                  &dynamic_simulation_B.t1_re, &dynamic_simulation_B.ad22, &A
                  [(dynamic_simulation_B.j_j + (dynamic_simulation_B.jp1 << 2))
                  - 1]);
                dynamic_simulation_B.jp1 = dynamic_simulation_B.j_j +
                  (dynamic_simulation_B.jp1 << 2);
                A[dynamic_simulation_B.jp1].re = 0.0;
                A[dynamic_simulation_B.jp1].im = 0.0;
              }

              dynamic_simulation_B.ctemp_tmp = dynamic_simulation_B.j_j - 1;
              while (dynamic_simulation_B.ctemp_tmp + 1 < 5) {
                dynamic_simulation_B.jp1 = (dynamic_simulation_B.ctemp_tmp << 2)
                  + dynamic_simulation_B.j_j;
                dynamic_simulation_B.ctemp_tmp_tmp = dynamic_simulation_B.jp1 -
                  1;
                dynamic_simulation_B.shift_re =
                  A[dynamic_simulation_B.ctemp_tmp_tmp].re *
                  dynamic_simulation_B.t1_re + (A[dynamic_simulation_B.jp1].re *
                  dynamic_simulation_B.ad22.re - A[dynamic_simulation_B.jp1].im *
                  dynamic_simulation_B.ad22.im);
                dynamic_simulation_B.shift_im =
                  A[dynamic_simulation_B.ctemp_tmp_tmp].im *
                  dynamic_simulation_B.t1_re + (A[dynamic_simulation_B.jp1].im *
                  dynamic_simulation_B.ad22.re + A[dynamic_simulation_B.jp1].re *
                  dynamic_simulation_B.ad22.im);
                dynamic_simulation_B.t1_im =
                  A[dynamic_simulation_B.ctemp_tmp_tmp].im;
                dynamic_simulation_B.absxr =
                  A[dynamic_simulation_B.ctemp_tmp_tmp].re;
                A[dynamic_simulation_B.jp1].re = A[dynamic_simulation_B.jp1].re *
                  dynamic_simulation_B.t1_re -
                  (A[dynamic_simulation_B.ctemp_tmp_tmp].re *
                   dynamic_simulation_B.ad22.re +
                   A[dynamic_simulation_B.ctemp_tmp_tmp].im *
                   dynamic_simulation_B.ad22.im);
                A[dynamic_simulation_B.jp1].im = A[dynamic_simulation_B.jp1].im *
                  dynamic_simulation_B.t1_re - (dynamic_simulation_B.ad22.re *
                  dynamic_simulation_B.t1_im - dynamic_simulation_B.ad22.im *
                  dynamic_simulation_B.absxr);
                A[dynamic_simulation_B.ctemp_tmp_tmp].re =
                  dynamic_simulation_B.shift_re;
                A[dynamic_simulation_B.ctemp_tmp_tmp].im =
                  dynamic_simulation_B.shift_im;
                dynamic_simulation_B.ctemp_tmp++;
              }

              dynamic_simulation_B.ad22.re = -dynamic_simulation_B.ad22.re;
              dynamic_simulation_B.ad22.im = -dynamic_simulation_B.ad22.im;
              dynamic_simulation_B.ctemp_tmp = dynamic_simulation_B.j_j;
              if (dynamic_simulation_B.ilast + 1 < dynamic_simulation_B.j_j + 2)
              {
                dynamic_simulation_B.ctemp_tmp = dynamic_simulation_B.ilast - 1;
              }

              dynamic_simulation_B.i_f = 0;
              while (dynamic_simulation_B.i_f + 1 <=
                     dynamic_simulation_B.ctemp_tmp + 2) {
                dynamic_simulation_B.jp1 = ((dynamic_simulation_B.j_j - 1) << 2)
                  + dynamic_simulation_B.i_f;
                dynamic_simulation_B.ctemp_tmp_tmp = (dynamic_simulation_B.j_j <<
                  2) + dynamic_simulation_B.i_f;
                dynamic_simulation_B.shift_re = (A[dynamic_simulation_B.jp1].re *
                  dynamic_simulation_B.ad22.re - A[dynamic_simulation_B.jp1].im *
                  dynamic_simulation_B.ad22.im) +
                  A[dynamic_simulation_B.ctemp_tmp_tmp].re *
                  dynamic_simulation_B.t1_re;
                dynamic_simulation_B.shift_im = (A[dynamic_simulation_B.jp1].im *
                  dynamic_simulation_B.ad22.re + A[dynamic_simulation_B.jp1].re *
                  dynamic_simulation_B.ad22.im) +
                  A[dynamic_simulation_B.ctemp_tmp_tmp].im *
                  dynamic_simulation_B.t1_re;
                dynamic_simulation_B.t1_im =
                  A[dynamic_simulation_B.ctemp_tmp_tmp].im;
                dynamic_simulation_B.absxr =
                  A[dynamic_simulation_B.ctemp_tmp_tmp].re;
                A[dynamic_simulation_B.jp1].re = A[dynamic_simulation_B.jp1].re *
                  dynamic_simulation_B.t1_re -
                  (A[dynamic_simulation_B.ctemp_tmp_tmp].re *
                   dynamic_simulation_B.ad22.re +
                   A[dynamic_simulation_B.ctemp_tmp_tmp].im *
                   dynamic_simulation_B.ad22.im);
                A[dynamic_simulation_B.jp1].im = A[dynamic_simulation_B.jp1].im *
                  dynamic_simulation_B.t1_re - (dynamic_simulation_B.ad22.re *
                  dynamic_simulation_B.t1_im - dynamic_simulation_B.ad22.im *
                  dynamic_simulation_B.absxr);
                A[dynamic_simulation_B.ctemp_tmp_tmp].re =
                  dynamic_simulation_B.shift_re;
                A[dynamic_simulation_B.ctemp_tmp_tmp].im =
                  dynamic_simulation_B.shift_im;
                dynamic_simulation_B.i_f++;
              }

              dynamic_simulation_B.jp1 = (dynamic_simulation_B.j_j - 1) << 2;
              dynamic_simulation_B.ctemp_tmp_tmp = dynamic_simulation_B.j_j << 2;
              dynamic_simulation_B.shift_re = (Z[dynamic_simulation_B.jp1].re *
                dynamic_simulation_B.ad22.re - Z[dynamic_simulation_B.jp1].im *
                dynamic_simulation_B.ad22.im) +
                Z[dynamic_simulation_B.ctemp_tmp_tmp].re *
                dynamic_simulation_B.t1_re;
              dynamic_simulation_B.shift_im = (Z[dynamic_simulation_B.jp1].im *
                dynamic_simulation_B.ad22.re + Z[dynamic_simulation_B.jp1].re *
                dynamic_simulation_B.ad22.im) +
                Z[dynamic_simulation_B.ctemp_tmp_tmp].im *
                dynamic_simulation_B.t1_re;
              dynamic_simulation_B.t1_im = Z[dynamic_simulation_B.ctemp_tmp_tmp]
                .im;
              dynamic_simulation_B.absxr = Z[dynamic_simulation_B.ctemp_tmp_tmp]
                .re;
              Z[dynamic_simulation_B.jp1].re = Z[dynamic_simulation_B.jp1].re *
                dynamic_simulation_B.t1_re -
                (Z[dynamic_simulation_B.ctemp_tmp_tmp].re *
                 dynamic_simulation_B.ad22.re +
                 Z[dynamic_simulation_B.ctemp_tmp_tmp].im *
                 dynamic_simulation_B.ad22.im);
              Z[dynamic_simulation_B.jp1].im = Z[dynamic_simulation_B.jp1].im *
                dynamic_simulation_B.t1_re - (dynamic_simulation_B.ad22.re *
                dynamic_simulation_B.t1_im - dynamic_simulation_B.ad22.im *
                dynamic_simulation_B.absxr);
              Z[dynamic_simulation_B.ctemp_tmp_tmp].re =
                dynamic_simulation_B.shift_re;
              Z[dynamic_simulation_B.ctemp_tmp_tmp].im =
                dynamic_simulation_B.shift_im;
              dynamic_simulation_B.ctemp_tmp = dynamic_simulation_B.jp1 + 1;
              dynamic_simulation_B.i_f = dynamic_simulation_B.ctemp_tmp_tmp + 1;
              dynamic_simulation_B.shift_re = (Z[dynamic_simulation_B.ctemp_tmp]
                .re * dynamic_simulation_B.ad22.re -
                Z[dynamic_simulation_B.ctemp_tmp].im *
                dynamic_simulation_B.ad22.im) + Z[dynamic_simulation_B.i_f].re *
                dynamic_simulation_B.t1_re;
              dynamic_simulation_B.shift_im = (Z[dynamic_simulation_B.ctemp_tmp]
                .im * dynamic_simulation_B.ad22.re +
                Z[dynamic_simulation_B.ctemp_tmp].re *
                dynamic_simulation_B.ad22.im) + Z[dynamic_simulation_B.i_f].im *
                dynamic_simulation_B.t1_re;
              dynamic_simulation_B.t1_im = Z[dynamic_simulation_B.i_f].im;
              dynamic_simulation_B.absxr = Z[dynamic_simulation_B.i_f].re;
              Z[dynamic_simulation_B.ctemp_tmp].re =
                Z[dynamic_simulation_B.ctemp_tmp].re *
                dynamic_simulation_B.t1_re - (Z[dynamic_simulation_B.i_f].re *
                dynamic_simulation_B.ad22.re + Z[dynamic_simulation_B.i_f].im *
                dynamic_simulation_B.ad22.im);
              Z[dynamic_simulation_B.ctemp_tmp].im =
                Z[dynamic_simulation_B.ctemp_tmp].im *
                dynamic_simulation_B.t1_re - (dynamic_simulation_B.ad22.re *
                dynamic_simulation_B.t1_im - dynamic_simulation_B.ad22.im *
                dynamic_simulation_B.absxr);
              Z[dynamic_simulation_B.i_f].re = dynamic_simulation_B.shift_re;
              Z[dynamic_simulation_B.i_f].im = dynamic_simulation_B.shift_im;
              dynamic_simulation_B.ctemp_tmp = dynamic_simulation_B.jp1 + 2;
              dynamic_simulation_B.i_f = dynamic_simulation_B.ctemp_tmp_tmp + 2;
              dynamic_simulation_B.shift_re = (Z[dynamic_simulation_B.ctemp_tmp]
                .re * dynamic_simulation_B.ad22.re -
                Z[dynamic_simulation_B.ctemp_tmp].im *
                dynamic_simulation_B.ad22.im) + Z[dynamic_simulation_B.i_f].re *
                dynamic_simulation_B.t1_re;
              dynamic_simulation_B.shift_im = (Z[dynamic_simulation_B.ctemp_tmp]
                .im * dynamic_simulation_B.ad22.re +
                Z[dynamic_simulation_B.ctemp_tmp].re *
                dynamic_simulation_B.ad22.im) + Z[dynamic_simulation_B.i_f].im *
                dynamic_simulation_B.t1_re;
              dynamic_simulation_B.t1_im = Z[dynamic_simulation_B.i_f].im;
              dynamic_simulation_B.absxr = Z[dynamic_simulation_B.i_f].re;
              Z[dynamic_simulation_B.ctemp_tmp].re =
                Z[dynamic_simulation_B.ctemp_tmp].re *
                dynamic_simulation_B.t1_re - (Z[dynamic_simulation_B.i_f].re *
                dynamic_simulation_B.ad22.re + Z[dynamic_simulation_B.i_f].im *
                dynamic_simulation_B.ad22.im);
              Z[dynamic_simulation_B.ctemp_tmp].im =
                Z[dynamic_simulation_B.ctemp_tmp].im *
                dynamic_simulation_B.t1_re - (dynamic_simulation_B.ad22.re *
                dynamic_simulation_B.t1_im - dynamic_simulation_B.ad22.im *
                dynamic_simulation_B.absxr);
              Z[dynamic_simulation_B.i_f].re = dynamic_simulation_B.shift_re;
              Z[dynamic_simulation_B.i_f].im = dynamic_simulation_B.shift_im;
              dynamic_simulation_B.jp1 += 3;
              dynamic_simulation_B.ctemp_tmp_tmp += 3;
              dynamic_simulation_B.shift_re = (Z[dynamic_simulation_B.jp1].re *
                dynamic_simulation_B.ad22.re - Z[dynamic_simulation_B.jp1].im *
                dynamic_simulation_B.ad22.im) +
                Z[dynamic_simulation_B.ctemp_tmp_tmp].re *
                dynamic_simulation_B.t1_re;
              dynamic_simulation_B.shift_im = (Z[dynamic_simulation_B.jp1].im *
                dynamic_simulation_B.ad22.re + Z[dynamic_simulation_B.jp1].re *
                dynamic_simulation_B.ad22.im) +
                Z[dynamic_simulation_B.ctemp_tmp_tmp].im *
                dynamic_simulation_B.t1_re;
              dynamic_simulation_B.t1_im = Z[dynamic_simulation_B.ctemp_tmp_tmp]
                .im;
              dynamic_simulation_B.absxr = Z[dynamic_simulation_B.ctemp_tmp_tmp]
                .re;
              Z[dynamic_simulation_B.jp1].re = Z[dynamic_simulation_B.jp1].re *
                dynamic_simulation_B.t1_re -
                (Z[dynamic_simulation_B.ctemp_tmp_tmp].re *
                 dynamic_simulation_B.ad22.re +
                 Z[dynamic_simulation_B.ctemp_tmp_tmp].im *
                 dynamic_simulation_B.ad22.im);
              Z[dynamic_simulation_B.jp1].im = Z[dynamic_simulation_B.jp1].im *
                dynamic_simulation_B.t1_re - (dynamic_simulation_B.ad22.re *
                dynamic_simulation_B.t1_im - dynamic_simulation_B.ad22.im *
                dynamic_simulation_B.absxr);
              Z[dynamic_simulation_B.ctemp_tmp_tmp].re =
                dynamic_simulation_B.shift_re;
              Z[dynamic_simulation_B.ctemp_tmp_tmp].im =
                dynamic_simulation_B.shift_im;
              dynamic_simulation_B.jp1 = dynamic_simulation_B.j_j - 1;
              dynamic_simulation_B.j_j++;
            }
          }

          dynamic_simulation_B.jiter++;
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
    if (dynamic_simulation_B.failed) {
      *info = dynamic_simulation_B.ilast + 1;
      dynamic_simulation_B.ifirst = 0;
      while (dynamic_simulation_B.ifirst <= dynamic_simulation_B.ilast) {
        alpha1[dynamic_simulation_B.ifirst].re = (rtNaN);
        alpha1[dynamic_simulation_B.ifirst].im = 0.0;
        beta1[dynamic_simulation_B.ifirst].re = (rtNaN);
        beta1[dynamic_simulation_B.ifirst].im = 0.0;
        dynamic_simulation_B.ifirst++;
      }

      for (dynamic_simulation_B.jp1 = 0; dynamic_simulation_B.jp1 < 16;
           dynamic_simulation_B.jp1++) {
        Z[dynamic_simulation_B.jp1].re = (rtNaN);
        Z[dynamic_simulation_B.jp1].im = 0.0;
      }
    } else {
      guard1 = true;
    }
  }

  if (guard1) {
    dynamic_simulation_B.ilast = 0;
    while (dynamic_simulation_B.ilast <= ilo - 2) {
      alpha1[dynamic_simulation_B.ilast] = A[(dynamic_simulation_B.ilast << 2) +
        dynamic_simulation_B.ilast];
      dynamic_simulation_B.ilast++;
    }
  }
}

// Function for MATLAB Function: '<S2>/MATLAB Function'
static void dynamic_simulation_xztgevc(const creal_T A[16], creal_T V[16])
{
  dynamic_simulation_B.rworka[0] = 0.0;
  dynamic_simulation_B.rworka[2] = 0.0;
  dynamic_simulation_B.rworka[3] = 0.0;
  dynamic_simulation_B.anorm_o = fabs(A[0].re) + fabs(A[0].im);
  dynamic_simulation_B.rworka[1] = fabs(A[4].re) + fabs(A[4].im);
  dynamic_simulation_B.ascale_b = (fabs(A[5].re) + fabs(A[5].im)) +
    dynamic_simulation_B.rworka[1];
  if (dynamic_simulation_B.ascale_b > dynamic_simulation_B.anorm_o) {
    dynamic_simulation_B.anorm_o = dynamic_simulation_B.ascale_b;
  }

  dynamic_simulation_B.i_j = 0;
  while (dynamic_simulation_B.i_j <= 1) {
    dynamic_simulation_B.rworka[2] += fabs(A[dynamic_simulation_B.i_j + 8].re) +
      fabs(A[dynamic_simulation_B.i_j + 8].im);
    dynamic_simulation_B.i_j++;
  }

  dynamic_simulation_B.ascale_b = (fabs(A[10].re) + fabs(A[10].im)) +
    dynamic_simulation_B.rworka[2];
  if (dynamic_simulation_B.ascale_b > dynamic_simulation_B.anorm_o) {
    dynamic_simulation_B.anorm_o = dynamic_simulation_B.ascale_b;
  }

  dynamic_simulation_B.i_j = 0;
  while (dynamic_simulation_B.i_j <= 2) {
    dynamic_simulation_B.rworka[3] += fabs(A[dynamic_simulation_B.i_j + 12].re)
      + fabs(A[dynamic_simulation_B.i_j + 12].im);
    dynamic_simulation_B.i_j++;
  }

  dynamic_simulation_B.ascale_b = (fabs(A[15].re) + fabs(A[15].im)) +
    dynamic_simulation_B.rworka[3];
  if (dynamic_simulation_B.ascale_b > dynamic_simulation_B.anorm_o) {
    dynamic_simulation_B.anorm_o = dynamic_simulation_B.ascale_b;
  }

  dynamic_simulation_B.ascale_b = dynamic_simulation_B.anorm_o;
  if (2.2250738585072014E-308 > dynamic_simulation_B.anorm_o) {
    dynamic_simulation_B.ascale_b = 2.2250738585072014E-308;
  }

  dynamic_simulation_B.ascale_b = 1.0 / dynamic_simulation_B.ascale_b;
  for (dynamic_simulation_B.i_j = 0; dynamic_simulation_B.i_j < 4;
       dynamic_simulation_B.i_j++) {
    dynamic_simulation_B.c_x_tmp_tmp = (3 - dynamic_simulation_B.i_j) << 2;
    dynamic_simulation_B.c_x_tmp = (dynamic_simulation_B.c_x_tmp_tmp -
      dynamic_simulation_B.i_j) + 3;
    dynamic_simulation_B.salpha_re = (fabs(A[dynamic_simulation_B.c_x_tmp].re) +
      fabs(A[dynamic_simulation_B.c_x_tmp].im)) * dynamic_simulation_B.ascale_b;
    if (1.0 > dynamic_simulation_B.salpha_re) {
      dynamic_simulation_B.salpha_re = 1.0;
    }

    dynamic_simulation_B.temp = 1.0 / dynamic_simulation_B.salpha_re;
    dynamic_simulation_B.salpha_re = A[dynamic_simulation_B.c_x_tmp].re *
      dynamic_simulation_B.temp * dynamic_simulation_B.ascale_b;
    dynamic_simulation_B.salpha_im = A[dynamic_simulation_B.c_x_tmp].im *
      dynamic_simulation_B.temp * dynamic_simulation_B.ascale_b;
    dynamic_simulation_B.acoeff = dynamic_simulation_B.temp *
      dynamic_simulation_B.ascale_b;
    dynamic_simulation_B.lscalea = ((dynamic_simulation_B.temp >=
      2.2250738585072014E-308) && (dynamic_simulation_B.acoeff <
      4.0083367200179456E-292));
    dynamic_simulation_B.dmin = fabs(dynamic_simulation_B.salpha_re) + fabs
      (dynamic_simulation_B.salpha_im);
    dynamic_simulation_B.lscaleb = ((dynamic_simulation_B.dmin >=
      2.2250738585072014E-308) && (dynamic_simulation_B.dmin <
      4.0083367200179456E-292));
    dynamic_simulation_B.scale_a = 1.0;
    if (dynamic_simulation_B.lscalea) {
      dynamic_simulation_B.scale_a = dynamic_simulation_B.anorm_o;
      if (2.4948003869184E+291 < dynamic_simulation_B.anorm_o) {
        dynamic_simulation_B.scale_a = 2.4948003869184E+291;
      }

      dynamic_simulation_B.scale_a *= 4.0083367200179456E-292 /
        dynamic_simulation_B.temp;
    }

    if (dynamic_simulation_B.lscaleb) {
      dynamic_simulation_B.work2_idx_2_im = 4.0083367200179456E-292 /
        dynamic_simulation_B.dmin;
      if (dynamic_simulation_B.work2_idx_2_im > dynamic_simulation_B.scale_a) {
        dynamic_simulation_B.scale_a = dynamic_simulation_B.work2_idx_2_im;
      }
    }

    if (dynamic_simulation_B.lscalea || dynamic_simulation_B.lscaleb) {
      dynamic_simulation_B.work2_idx_2_im = dynamic_simulation_B.acoeff;
      if (1.0 > dynamic_simulation_B.acoeff) {
        dynamic_simulation_B.work2_idx_2_im = 1.0;
      }

      if (dynamic_simulation_B.dmin > dynamic_simulation_B.work2_idx_2_im) {
        dynamic_simulation_B.work2_idx_2_im = dynamic_simulation_B.dmin;
      }

      dynamic_simulation_B.dmin = 1.0 / (2.2250738585072014E-308 *
        dynamic_simulation_B.work2_idx_2_im);
      if (dynamic_simulation_B.dmin < dynamic_simulation_B.scale_a) {
        dynamic_simulation_B.scale_a = dynamic_simulation_B.dmin;
      }

      if (dynamic_simulation_B.lscalea) {
        dynamic_simulation_B.acoeff = dynamic_simulation_B.scale_a *
          dynamic_simulation_B.temp * dynamic_simulation_B.ascale_b;
      } else {
        dynamic_simulation_B.acoeff *= dynamic_simulation_B.scale_a;
      }

      dynamic_simulation_B.salpha_re *= dynamic_simulation_B.scale_a;
      dynamic_simulation_B.salpha_im *= dynamic_simulation_B.scale_a;
    }

    memset(&dynamic_simulation_B.work1[0], 0, sizeof(creal_T) << 2U);
    dynamic_simulation_B.work1[3 - dynamic_simulation_B.i_j].re = 1.0;
    dynamic_simulation_B.work1[3 - dynamic_simulation_B.i_j].im = 0.0;
    dynamic_simulation_B.dmin = 2.2204460492503131E-16 *
      dynamic_simulation_B.acoeff * dynamic_simulation_B.anorm_o;
    dynamic_simulation_B.temp = (fabs(dynamic_simulation_B.salpha_re) + fabs
      (dynamic_simulation_B.salpha_im)) * 2.2204460492503131E-16;
    if (dynamic_simulation_B.temp > dynamic_simulation_B.dmin) {
      dynamic_simulation_B.dmin = dynamic_simulation_B.temp;
    }

    if (2.2250738585072014E-308 > dynamic_simulation_B.dmin) {
      dynamic_simulation_B.dmin = 2.2250738585072014E-308;
    }

    dynamic_simulation_B.c_x_tmp = 0;
    while (dynamic_simulation_B.c_x_tmp <= 2 - dynamic_simulation_B.i_j) {
      dynamic_simulation_B.d_re_tmp = dynamic_simulation_B.c_x_tmp_tmp +
        dynamic_simulation_B.c_x_tmp;
      dynamic_simulation_B.work1[dynamic_simulation_B.c_x_tmp].re =
        A[dynamic_simulation_B.d_re_tmp].re * dynamic_simulation_B.acoeff;
      dynamic_simulation_B.work1[dynamic_simulation_B.c_x_tmp].im =
        A[dynamic_simulation_B.d_re_tmp].im * dynamic_simulation_B.acoeff;
      dynamic_simulation_B.c_x_tmp++;
    }

    dynamic_simulation_B.work1[3 - dynamic_simulation_B.i_j].re = 1.0;
    dynamic_simulation_B.work1[3 - dynamic_simulation_B.i_j].im = 0.0;
    dynamic_simulation_B.c_x_tmp = static_cast<int32_T>(((-1.0 - ((-static_cast<
      real_T>(dynamic_simulation_B.i_j) + 4.0) - 1.0)) + 1.0) / -1.0);
    dynamic_simulation_B.c_j_k = 0;
    while (dynamic_simulation_B.c_j_k <= dynamic_simulation_B.c_x_tmp - 1) {
      dynamic_simulation_B.work2_idx_1_re_tmp = 2 - (dynamic_simulation_B.i_j +
        dynamic_simulation_B.c_j_k);
      dynamic_simulation_B.d_re_tmp_tmp =
        dynamic_simulation_B.work2_idx_1_re_tmp << 2;
      dynamic_simulation_B.d_re_tmp = dynamic_simulation_B.d_re_tmp_tmp +
        dynamic_simulation_B.work2_idx_1_re_tmp;
      dynamic_simulation_B.work2_idx_3_re = A[dynamic_simulation_B.d_re_tmp].re *
        dynamic_simulation_B.acoeff - dynamic_simulation_B.salpha_re;
      dynamic_simulation_B.scale_a = A[dynamic_simulation_B.d_re_tmp].im *
        dynamic_simulation_B.acoeff - dynamic_simulation_B.salpha_im;
      if (fabs(dynamic_simulation_B.work2_idx_3_re) + fabs
          (dynamic_simulation_B.scale_a) <= dynamic_simulation_B.dmin) {
        dynamic_simulation_B.work2_idx_3_re = dynamic_simulation_B.dmin;
        dynamic_simulation_B.scale_a = 0.0;
      }

      dynamic_simulation_B.work2_idx_2_im = fabs
        (dynamic_simulation_B.work2_idx_3_re);
      dynamic_simulation_B.work2_idx_3_im = fabs(dynamic_simulation_B.scale_a);
      dynamic_simulation_B.temp = dynamic_simulation_B.work2_idx_2_im +
        dynamic_simulation_B.work2_idx_3_im;
      if (dynamic_simulation_B.temp < 1.0) {
        dynamic_simulation_B.f_y = fabs
          (dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp].
           re) + fabs
          (dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp].
           im);
        if (dynamic_simulation_B.f_y >= dynamic_simulation_B.temp *
            1.1235582092889474E+307) {
          dynamic_simulation_B.temp = 1.0 / dynamic_simulation_B.f_y;
          dynamic_simulation_B.d_re_tmp = 0;
          while (dynamic_simulation_B.d_re_tmp <= 3 - dynamic_simulation_B.i_j)
          {
            dynamic_simulation_B.work1[dynamic_simulation_B.d_re_tmp].re *=
              dynamic_simulation_B.temp;
            dynamic_simulation_B.work1[dynamic_simulation_B.d_re_tmp].im *=
              dynamic_simulation_B.temp;
            dynamic_simulation_B.d_re_tmp++;
          }
        }
      }

      if (dynamic_simulation_B.scale_a == 0.0) {
        if (-dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp]
            .im == 0.0) {
          dynamic_simulation_B.temp =
            -dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp]
            .re / dynamic_simulation_B.work2_idx_3_re;
          dynamic_simulation_B.scale_a = 0.0;
        } else if
            (-dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp]
             .re == 0.0) {
          dynamic_simulation_B.temp = 0.0;
          dynamic_simulation_B.scale_a =
            -dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp]
            .im / dynamic_simulation_B.work2_idx_3_re;
        } else {
          dynamic_simulation_B.temp =
            -dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp]
            .re / dynamic_simulation_B.work2_idx_3_re;
          dynamic_simulation_B.scale_a =
            -dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp]
            .im / dynamic_simulation_B.work2_idx_3_re;
        }
      } else if (dynamic_simulation_B.work2_idx_3_re == 0.0) {
        if (-dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp]
            .re == 0.0) {
          dynamic_simulation_B.temp =
            -dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp]
            .im / dynamic_simulation_B.scale_a;
          dynamic_simulation_B.scale_a = 0.0;
        } else if
            (-dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp]
             .im == 0.0) {
          dynamic_simulation_B.temp = 0.0;
          dynamic_simulation_B.scale_a =
            -(-dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp]
              .re / dynamic_simulation_B.scale_a);
        } else {
          dynamic_simulation_B.temp =
            -dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp]
            .im / dynamic_simulation_B.scale_a;
          dynamic_simulation_B.scale_a =
            -(-dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp]
              .re / dynamic_simulation_B.scale_a);
        }
      } else if (dynamic_simulation_B.work2_idx_2_im >
                 dynamic_simulation_B.work2_idx_3_im) {
        dynamic_simulation_B.work2_idx_2_im = dynamic_simulation_B.scale_a /
          dynamic_simulation_B.work2_idx_3_re;
        dynamic_simulation_B.scale_a = dynamic_simulation_B.work2_idx_2_im *
          dynamic_simulation_B.scale_a + dynamic_simulation_B.work2_idx_3_re;
        dynamic_simulation_B.temp = (dynamic_simulation_B.work2_idx_2_im *
          -dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp].
          im +
          -dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp].
          re) / dynamic_simulation_B.scale_a;
        dynamic_simulation_B.scale_a =
          (-dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp].
           im - dynamic_simulation_B.work2_idx_2_im *
           -dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp].
           re) / dynamic_simulation_B.scale_a;
      } else if (dynamic_simulation_B.work2_idx_3_im ==
                 dynamic_simulation_B.work2_idx_2_im) {
        dynamic_simulation_B.work2_idx_3_re =
          dynamic_simulation_B.work2_idx_3_re > 0.0 ? 0.5 : -0.5;
        dynamic_simulation_B.scale_a = dynamic_simulation_B.scale_a > 0.0 ? 0.5 :
          -0.5;
        dynamic_simulation_B.temp =
          (-dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp].
           re * dynamic_simulation_B.work2_idx_3_re +
           -dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp].
           im * dynamic_simulation_B.scale_a) /
          dynamic_simulation_B.work2_idx_2_im;
        dynamic_simulation_B.scale_a =
          (-dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp].
           im * dynamic_simulation_B.work2_idx_3_re -
           -dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp].
           re * dynamic_simulation_B.scale_a) /
          dynamic_simulation_B.work2_idx_2_im;
      } else {
        dynamic_simulation_B.work2_idx_2_im =
          dynamic_simulation_B.work2_idx_3_re / dynamic_simulation_B.scale_a;
        dynamic_simulation_B.scale_a += dynamic_simulation_B.work2_idx_2_im *
          dynamic_simulation_B.work2_idx_3_re;
        dynamic_simulation_B.temp = (dynamic_simulation_B.work2_idx_2_im *
          -dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp].
          re +
          -dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp].
          im) / dynamic_simulation_B.scale_a;
        dynamic_simulation_B.scale_a = (dynamic_simulation_B.work2_idx_2_im *
          -dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp].
          im -
          (-dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp].
           re)) / dynamic_simulation_B.scale_a;
      }

      dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp].re =
        dynamic_simulation_B.temp;
      dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp].im =
        dynamic_simulation_B.scale_a;
      if (dynamic_simulation_B.work2_idx_1_re_tmp + 1 > 1) {
        if (fabs
            (dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp]
             .re) + fabs
            (dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp]
             .im) > 1.0) {
          dynamic_simulation_B.temp = 1.0 / (fabs
            (dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp]
             .re) + fabs
            (dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp]
             .im));
          if (dynamic_simulation_B.acoeff *
              dynamic_simulation_B.rworka[dynamic_simulation_B.work2_idx_1_re_tmp]
              >= 1.1235582092889474E+307 * dynamic_simulation_B.temp) {
            dynamic_simulation_B.d_re_tmp = 0;
            while (dynamic_simulation_B.d_re_tmp <= 3 - dynamic_simulation_B.i_j)
            {
              dynamic_simulation_B.work1[dynamic_simulation_B.d_re_tmp].re *=
                dynamic_simulation_B.temp;
              dynamic_simulation_B.work1[dynamic_simulation_B.d_re_tmp].im *=
                dynamic_simulation_B.temp;
              dynamic_simulation_B.d_re_tmp++;
            }
          }
        }

        dynamic_simulation_B.work2_idx_3_re = dynamic_simulation_B.acoeff *
          dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp].re;
        dynamic_simulation_B.scale_a = dynamic_simulation_B.acoeff *
          dynamic_simulation_B.work1[dynamic_simulation_B.work2_idx_1_re_tmp].im;
        dynamic_simulation_B.e_jr = 0;
        while (dynamic_simulation_B.e_jr <=
               dynamic_simulation_B.work2_idx_1_re_tmp - 1) {
          dynamic_simulation_B.d_re_tmp = dynamic_simulation_B.d_re_tmp_tmp +
            dynamic_simulation_B.e_jr;
          dynamic_simulation_B.work1[dynamic_simulation_B.e_jr].re +=
            A[dynamic_simulation_B.d_re_tmp].re *
            dynamic_simulation_B.work2_idx_3_re -
            A[dynamic_simulation_B.d_re_tmp].im * dynamic_simulation_B.scale_a;
          dynamic_simulation_B.work1[dynamic_simulation_B.e_jr].im +=
            A[dynamic_simulation_B.d_re_tmp].im *
            dynamic_simulation_B.work2_idx_3_re +
            A[dynamic_simulation_B.d_re_tmp].re * dynamic_simulation_B.scale_a;
          dynamic_simulation_B.e_jr++;
        }
      }

      dynamic_simulation_B.c_j_k++;
    }

    dynamic_simulation_B.salpha_re = 0.0;
    dynamic_simulation_B.salpha_im = 0.0;
    dynamic_simulation_B.acoeff = 0.0;
    dynamic_simulation_B.dmin = 0.0;
    dynamic_simulation_B.scale_a = 0.0;
    dynamic_simulation_B.work2_idx_2_im = 0.0;
    dynamic_simulation_B.work2_idx_3_re = 0.0;
    dynamic_simulation_B.work2_idx_3_im = 0.0;
    dynamic_simulation_B.c_x_tmp = 0;
    while (dynamic_simulation_B.c_x_tmp <= 3 - dynamic_simulation_B.i_j) {
      dynamic_simulation_B.c_j_k = dynamic_simulation_B.c_x_tmp << 2;
      dynamic_simulation_B.salpha_re += V[dynamic_simulation_B.c_j_k].re *
        dynamic_simulation_B.work1[dynamic_simulation_B.c_x_tmp].re -
        V[dynamic_simulation_B.c_j_k].im *
        dynamic_simulation_B.work1[dynamic_simulation_B.c_x_tmp].im;
      dynamic_simulation_B.salpha_im += V[dynamic_simulation_B.c_j_k].re *
        dynamic_simulation_B.work1[dynamic_simulation_B.c_x_tmp].im +
        V[dynamic_simulation_B.c_j_k].im *
        dynamic_simulation_B.work1[dynamic_simulation_B.c_x_tmp].re;
      dynamic_simulation_B.work2_idx_1_re_tmp = dynamic_simulation_B.c_j_k + 1;
      dynamic_simulation_B.acoeff += V[dynamic_simulation_B.work2_idx_1_re_tmp].
        re * dynamic_simulation_B.work1[dynamic_simulation_B.c_x_tmp].re -
        V[dynamic_simulation_B.work2_idx_1_re_tmp].im *
        dynamic_simulation_B.work1[dynamic_simulation_B.c_x_tmp].im;
      dynamic_simulation_B.dmin += V[dynamic_simulation_B.work2_idx_1_re_tmp].re
        * dynamic_simulation_B.work1[dynamic_simulation_B.c_x_tmp].im +
        V[dynamic_simulation_B.work2_idx_1_re_tmp].im *
        dynamic_simulation_B.work1[dynamic_simulation_B.c_x_tmp].re;
      dynamic_simulation_B.work2_idx_1_re_tmp = dynamic_simulation_B.c_j_k + 2;
      dynamic_simulation_B.scale_a += V[dynamic_simulation_B.work2_idx_1_re_tmp]
        .re * dynamic_simulation_B.work1[dynamic_simulation_B.c_x_tmp].re -
        V[dynamic_simulation_B.work2_idx_1_re_tmp].im *
        dynamic_simulation_B.work1[dynamic_simulation_B.c_x_tmp].im;
      dynamic_simulation_B.work2_idx_2_im +=
        V[dynamic_simulation_B.work2_idx_1_re_tmp].re *
        dynamic_simulation_B.work1[dynamic_simulation_B.c_x_tmp].im +
        V[dynamic_simulation_B.work2_idx_1_re_tmp].im *
        dynamic_simulation_B.work1[dynamic_simulation_B.c_x_tmp].re;
      dynamic_simulation_B.c_j_k += 3;
      dynamic_simulation_B.work2_idx_3_re += V[dynamic_simulation_B.c_j_k].re *
        dynamic_simulation_B.work1[dynamic_simulation_B.c_x_tmp].re -
        V[dynamic_simulation_B.c_j_k].im *
        dynamic_simulation_B.work1[dynamic_simulation_B.c_x_tmp].im;
      dynamic_simulation_B.work2_idx_3_im += V[dynamic_simulation_B.c_j_k].re *
        dynamic_simulation_B.work1[dynamic_simulation_B.c_x_tmp].im +
        V[dynamic_simulation_B.c_j_k].im *
        dynamic_simulation_B.work1[dynamic_simulation_B.c_x_tmp].re;
      dynamic_simulation_B.c_x_tmp++;
    }

    dynamic_simulation_B.temp = fabs(dynamic_simulation_B.salpha_re) + fabs
      (dynamic_simulation_B.salpha_im);
    dynamic_simulation_B.f_y = fabs(dynamic_simulation_B.acoeff) + fabs
      (dynamic_simulation_B.dmin);
    if (dynamic_simulation_B.f_y > dynamic_simulation_B.temp) {
      dynamic_simulation_B.temp = dynamic_simulation_B.f_y;
    }

    dynamic_simulation_B.f_y = fabs(dynamic_simulation_B.scale_a) + fabs
      (dynamic_simulation_B.work2_idx_2_im);
    if (dynamic_simulation_B.f_y > dynamic_simulation_B.temp) {
      dynamic_simulation_B.temp = dynamic_simulation_B.f_y;
    }

    dynamic_simulation_B.f_y = fabs(dynamic_simulation_B.work2_idx_3_re) + fabs
      (dynamic_simulation_B.work2_idx_3_im);
    if (dynamic_simulation_B.f_y > dynamic_simulation_B.temp) {
      dynamic_simulation_B.temp = dynamic_simulation_B.f_y;
    }

    if (dynamic_simulation_B.temp > 2.2250738585072014E-308) {
      dynamic_simulation_B.temp = 1.0 / dynamic_simulation_B.temp;
      V[dynamic_simulation_B.c_x_tmp_tmp].re = dynamic_simulation_B.temp *
        dynamic_simulation_B.salpha_re;
      V[dynamic_simulation_B.c_x_tmp_tmp].im = dynamic_simulation_B.temp *
        dynamic_simulation_B.salpha_im;
      dynamic_simulation_B.d_re_tmp = ((3 - dynamic_simulation_B.i_j) << 2) + 1;
      V[dynamic_simulation_B.d_re_tmp].re = dynamic_simulation_B.temp *
        dynamic_simulation_B.acoeff;
      V[dynamic_simulation_B.d_re_tmp].im = dynamic_simulation_B.temp *
        dynamic_simulation_B.dmin;
      dynamic_simulation_B.d_re_tmp = ((3 - dynamic_simulation_B.i_j) << 2) + 2;
      V[dynamic_simulation_B.d_re_tmp].re = dynamic_simulation_B.temp *
        dynamic_simulation_B.scale_a;
      V[dynamic_simulation_B.d_re_tmp].im = dynamic_simulation_B.temp *
        dynamic_simulation_B.work2_idx_2_im;
      dynamic_simulation_B.d_re_tmp = ((3 - dynamic_simulation_B.i_j) << 2) + 3;
      V[dynamic_simulation_B.d_re_tmp].re = dynamic_simulation_B.temp *
        dynamic_simulation_B.work2_idx_3_re;
      V[dynamic_simulation_B.d_re_tmp].im = dynamic_simulation_B.temp *
        dynamic_simulation_B.work2_idx_3_im;
    } else {
      V[dynamic_simulation_B.c_x_tmp_tmp].re = 0.0;
      V[dynamic_simulation_B.c_x_tmp_tmp].im = 0.0;
      dynamic_simulation_B.d_re_tmp = dynamic_simulation_B.c_x_tmp_tmp + 1;
      V[dynamic_simulation_B.d_re_tmp].re = 0.0;
      V[dynamic_simulation_B.d_re_tmp].im = 0.0;
      dynamic_simulation_B.d_re_tmp = dynamic_simulation_B.c_x_tmp_tmp + 2;
      V[dynamic_simulation_B.d_re_tmp].re = 0.0;
      V[dynamic_simulation_B.d_re_tmp].im = 0.0;
      dynamic_simulation_B.d_re_tmp = dynamic_simulation_B.c_x_tmp_tmp + 3;
      V[dynamic_simulation_B.d_re_tmp].re = 0.0;
      V[dynamic_simulation_B.d_re_tmp].im = 0.0;
    }
  }
}

// Function for MATLAB Function: '<S2>/MATLAB Function'
static void dynamic_simulation_xzggev(creal_T A[16], int32_T *info, creal_T
  alpha1[4], creal_T beta1[4], creal_T V[16])
{
  int32_T exitg1;
  int32_T exitg2;
  boolean_T exitg3;
  boolean_T exitg4;
  *info = 0;
  dynamic_simulation_B.anrm = dynamic_simulation_xzlangeM(A);
  if (rtIsInf(dynamic_simulation_B.anrm) || rtIsNaN(dynamic_simulation_B.anrm))
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
    for (dynamic_simulation_B.k_m = 0; dynamic_simulation_B.k_m < 16;
         dynamic_simulation_B.k_m++) {
      V[dynamic_simulation_B.k_m].re = (rtNaN);
      V[dynamic_simulation_B.k_m].im = 0.0;
    }
  } else {
    dynamic_simulation_B.ilascl = false;
    dynamic_simulation_B.anrmto = dynamic_simulation_B.anrm;
    if ((dynamic_simulation_B.anrm > 0.0) && (dynamic_simulation_B.anrm <
         6.7178761075670888E-139)) {
      dynamic_simulation_B.anrmto = 6.7178761075670888E-139;
      dynamic_simulation_B.ilascl = true;
      dynamic_simulation_xzlascl(dynamic_simulation_B.anrm,
        dynamic_simulation_B.anrmto, A);
    } else {
      if (dynamic_simulation_B.anrm > 1.4885657073574029E+138) {
        dynamic_simulation_B.anrmto = 1.4885657073574029E+138;
        dynamic_simulation_B.ilascl = true;
        dynamic_simulation_xzlascl(dynamic_simulation_B.anrm,
          dynamic_simulation_B.anrmto, A);
      }
    }

    dynamic_simulation_B.rscale[0] = 1;
    dynamic_simulation_B.rscale[1] = 1;
    dynamic_simulation_B.rscale[2] = 1;
    dynamic_simulation_B.rscale[3] = 1;
    dynamic_simulation_B.c_i = 0;
    dynamic_simulation_B.ihi = 4;
    do {
      exitg2 = 0;
      dynamic_simulation_B.i_m = 0;
      dynamic_simulation_B.jcol = 0;
      dynamic_simulation_B.found = false;
      dynamic_simulation_B.ii = dynamic_simulation_B.ihi;
      exitg3 = false;
      while ((!exitg3) && (dynamic_simulation_B.ii > 0)) {
        dynamic_simulation_B.nzcount = 0;
        dynamic_simulation_B.i_m = dynamic_simulation_B.ii;
        dynamic_simulation_B.jcol = dynamic_simulation_B.ihi;
        dynamic_simulation_B.jj = 0;
        exitg4 = false;
        while ((!exitg4) && (dynamic_simulation_B.jj <= dynamic_simulation_B.ihi
                             - 1)) {
          dynamic_simulation_B.k_m = ((dynamic_simulation_B.jj << 2) +
            dynamic_simulation_B.ii) - 1;
          if ((A[dynamic_simulation_B.k_m].re != 0.0) ||
              (A[dynamic_simulation_B.k_m].im != 0.0) ||
              (dynamic_simulation_B.jj + 1 == dynamic_simulation_B.ii)) {
            if (dynamic_simulation_B.nzcount == 0) {
              dynamic_simulation_B.jcol = dynamic_simulation_B.jj + 1;
              dynamic_simulation_B.nzcount = 1;
              dynamic_simulation_B.jj++;
            } else {
              dynamic_simulation_B.nzcount = 2;
              exitg4 = true;
            }
          } else {
            dynamic_simulation_B.jj++;
          }
        }

        if (dynamic_simulation_B.nzcount < 2) {
          dynamic_simulation_B.found = true;
          exitg3 = true;
        } else {
          dynamic_simulation_B.ii--;
        }
      }

      if (!dynamic_simulation_B.found) {
        exitg2 = 2;
      } else {
        if (dynamic_simulation_B.i_m != dynamic_simulation_B.ihi) {
          dynamic_simulation_B.atmp_re = A[dynamic_simulation_B.i_m - 1].re;
          dynamic_simulation_B.atmp_im = A[dynamic_simulation_B.i_m - 1].im;
          A[dynamic_simulation_B.i_m - 1] = A[dynamic_simulation_B.ihi - 1];
          A[dynamic_simulation_B.ihi - 1].re = dynamic_simulation_B.atmp_re;
          A[dynamic_simulation_B.ihi - 1].im = dynamic_simulation_B.atmp_im;
          dynamic_simulation_B.atmp_re = A[dynamic_simulation_B.i_m + 3].re;
          dynamic_simulation_B.atmp_im = A[dynamic_simulation_B.i_m + 3].im;
          A[dynamic_simulation_B.i_m + 3] = A[dynamic_simulation_B.ihi + 3];
          A[dynamic_simulation_B.ihi + 3].re = dynamic_simulation_B.atmp_re;
          A[dynamic_simulation_B.ihi + 3].im = dynamic_simulation_B.atmp_im;
          dynamic_simulation_B.atmp_re = A[dynamic_simulation_B.i_m + 7].re;
          dynamic_simulation_B.atmp_im = A[dynamic_simulation_B.i_m + 7].im;
          A[dynamic_simulation_B.i_m + 7] = A[dynamic_simulation_B.ihi + 7];
          A[dynamic_simulation_B.ihi + 7].re = dynamic_simulation_B.atmp_re;
          A[dynamic_simulation_B.ihi + 7].im = dynamic_simulation_B.atmp_im;
          dynamic_simulation_B.atmp_re = A[dynamic_simulation_B.i_m + 11].re;
          dynamic_simulation_B.atmp_im = A[dynamic_simulation_B.i_m + 11].im;
          A[dynamic_simulation_B.i_m + 11] = A[dynamic_simulation_B.ihi + 11];
          A[dynamic_simulation_B.ihi + 11].re = dynamic_simulation_B.atmp_re;
          A[dynamic_simulation_B.ihi + 11].im = dynamic_simulation_B.atmp_im;
        }

        if (dynamic_simulation_B.jcol != dynamic_simulation_B.ihi) {
          dynamic_simulation_B.ii = 0;
          while (dynamic_simulation_B.ii <= dynamic_simulation_B.ihi - 1) {
            dynamic_simulation_B.i_m = ((dynamic_simulation_B.jcol - 1) << 2) +
              dynamic_simulation_B.ii;
            dynamic_simulation_B.atmp_re = A[dynamic_simulation_B.i_m].re;
            dynamic_simulation_B.atmp_im = A[dynamic_simulation_B.i_m].im;
            dynamic_simulation_B.k_m = ((dynamic_simulation_B.ihi - 1) << 2) +
              dynamic_simulation_B.ii;
            A[dynamic_simulation_B.i_m] = A[dynamic_simulation_B.k_m];
            A[dynamic_simulation_B.k_m].re = dynamic_simulation_B.atmp_re;
            A[dynamic_simulation_B.k_m].im = dynamic_simulation_B.atmp_im;
            dynamic_simulation_B.ii++;
          }
        }

        dynamic_simulation_B.rscale[dynamic_simulation_B.ihi - 1] =
          dynamic_simulation_B.jcol;
        dynamic_simulation_B.ihi--;
        if (dynamic_simulation_B.ihi == 1) {
          dynamic_simulation_B.rscale[0] = 1;
          exitg2 = 1;
        }
      }
    } while (exitg2 == 0);

    if (exitg2 == 1) {
    } else {
      do {
        exitg1 = 0;
        dynamic_simulation_B.ii = 0;
        dynamic_simulation_B.jcol = 0;
        dynamic_simulation_B.found = false;
        dynamic_simulation_B.i_m = dynamic_simulation_B.c_i + 1;
        exitg3 = false;
        while ((!exitg3) && (dynamic_simulation_B.i_m <=
                             dynamic_simulation_B.ihi)) {
          dynamic_simulation_B.nzcount = 0;
          dynamic_simulation_B.ii = dynamic_simulation_B.ihi;
          dynamic_simulation_B.jcol = dynamic_simulation_B.i_m;
          dynamic_simulation_B.jj = dynamic_simulation_B.c_i + 1;
          exitg4 = false;
          while ((!exitg4) && (dynamic_simulation_B.jj <=
                               dynamic_simulation_B.ihi)) {
            dynamic_simulation_B.k_m = (((dynamic_simulation_B.i_m - 1) << 2) +
              dynamic_simulation_B.jj) - 1;
            if ((A[dynamic_simulation_B.k_m].re != 0.0) ||
                (A[dynamic_simulation_B.k_m].im != 0.0) ||
                (dynamic_simulation_B.jj == dynamic_simulation_B.i_m)) {
              if (dynamic_simulation_B.nzcount == 0) {
                dynamic_simulation_B.ii = dynamic_simulation_B.jj;
                dynamic_simulation_B.nzcount = 1;
                dynamic_simulation_B.jj++;
              } else {
                dynamic_simulation_B.nzcount = 2;
                exitg4 = true;
              }
            } else {
              dynamic_simulation_B.jj++;
            }
          }

          if (dynamic_simulation_B.nzcount < 2) {
            dynamic_simulation_B.found = true;
            exitg3 = true;
          } else {
            dynamic_simulation_B.i_m++;
          }
        }

        if (!dynamic_simulation_B.found) {
          exitg1 = 1;
        } else {
          if (dynamic_simulation_B.c_i + 1 != dynamic_simulation_B.ii) {
            dynamic_simulation_B.nzcount = dynamic_simulation_B.c_i;
            while (dynamic_simulation_B.nzcount + 1 < 5) {
              dynamic_simulation_B.k_m = dynamic_simulation_B.nzcount << 2;
              dynamic_simulation_B.i_m = (dynamic_simulation_B.k_m +
                dynamic_simulation_B.ii) - 1;
              dynamic_simulation_B.atmp_re = A[dynamic_simulation_B.i_m].re;
              dynamic_simulation_B.atmp_im = A[dynamic_simulation_B.i_m].im;
              dynamic_simulation_B.k_m += dynamic_simulation_B.c_i;
              A[dynamic_simulation_B.i_m] = A[dynamic_simulation_B.k_m];
              A[dynamic_simulation_B.k_m].re = dynamic_simulation_B.atmp_re;
              A[dynamic_simulation_B.k_m].im = dynamic_simulation_B.atmp_im;
              dynamic_simulation_B.nzcount++;
            }
          }

          if (dynamic_simulation_B.c_i + 1 != dynamic_simulation_B.jcol) {
            dynamic_simulation_B.ii = 0;
            while (dynamic_simulation_B.ii <= dynamic_simulation_B.ihi - 1) {
              dynamic_simulation_B.i_m = ((dynamic_simulation_B.jcol - 1) << 2)
                + dynamic_simulation_B.ii;
              dynamic_simulation_B.atmp_re = A[dynamic_simulation_B.i_m].re;
              dynamic_simulation_B.atmp_im = A[dynamic_simulation_B.i_m].im;
              dynamic_simulation_B.k_m = (dynamic_simulation_B.c_i << 2) +
                dynamic_simulation_B.ii;
              A[dynamic_simulation_B.i_m] = A[dynamic_simulation_B.k_m];
              A[dynamic_simulation_B.k_m].re = dynamic_simulation_B.atmp_re;
              A[dynamic_simulation_B.k_m].im = dynamic_simulation_B.atmp_im;
              dynamic_simulation_B.ii++;
            }
          }

          dynamic_simulation_B.rscale[dynamic_simulation_B.c_i] =
            dynamic_simulation_B.jcol;
          dynamic_simulation_B.c_i++;
          if (dynamic_simulation_B.c_i + 1 == dynamic_simulation_B.ihi) {
            dynamic_simulation_B.rscale[dynamic_simulation_B.c_i] =
              dynamic_simulation_B.c_i + 1;
            exitg1 = 1;
          }
        }
      } while (exitg1 == 0);
    }

    for (dynamic_simulation_B.k_m = 0; dynamic_simulation_B.k_m < 16;
         dynamic_simulation_B.k_m++) {
      dynamic_simulation_B.b_I_h[dynamic_simulation_B.k_m] = 0;
    }

    dynamic_simulation_B.b_I_h[0] = 1;
    dynamic_simulation_B.b_I_h[5] = 1;
    dynamic_simulation_B.b_I_h[10] = 1;
    dynamic_simulation_B.b_I_h[15] = 1;
    for (dynamic_simulation_B.k_m = 0; dynamic_simulation_B.k_m < 16;
         dynamic_simulation_B.k_m++) {
      V[dynamic_simulation_B.k_m].re =
        dynamic_simulation_B.b_I_h[dynamic_simulation_B.k_m];
      V[dynamic_simulation_B.k_m].im = 0.0;
    }

    if (dynamic_simulation_B.ihi >= dynamic_simulation_B.c_i + 3) {
      dynamic_simulation_B.jcol = dynamic_simulation_B.c_i;
      while (dynamic_simulation_B.jcol + 1 < dynamic_simulation_B.ihi - 1) {
        dynamic_simulation_B.ii = dynamic_simulation_B.ihi - 1;
        while (dynamic_simulation_B.ii + 1 > dynamic_simulation_B.jcol + 2) {
          dynamic_simulation_xzlartg(A[(dynamic_simulation_B.ii +
            (dynamic_simulation_B.jcol << 2)) - 1], A[dynamic_simulation_B.ii +
            (dynamic_simulation_B.jcol << 2)], &dynamic_simulation_B.mul,
            &dynamic_simulation_B.s, &A[(dynamic_simulation_B.ii +
            (dynamic_simulation_B.jcol << 2)) - 1]);
          dynamic_simulation_B.k_m = dynamic_simulation_B.ii +
            (dynamic_simulation_B.jcol << 2);
          A[dynamic_simulation_B.k_m].re = 0.0;
          A[dynamic_simulation_B.k_m].im = 0.0;
          dynamic_simulation_B.nzcount = dynamic_simulation_B.jcol + 1;
          while (dynamic_simulation_B.nzcount + 1 < 5) {
            dynamic_simulation_B.i_m = (dynamic_simulation_B.nzcount << 2) +
              dynamic_simulation_B.ii;
            dynamic_simulation_B.k_m = dynamic_simulation_B.i_m - 1;
            dynamic_simulation_B.atmp_re = A[dynamic_simulation_B.k_m].re *
              dynamic_simulation_B.mul + (A[dynamic_simulation_B.i_m].re *
              dynamic_simulation_B.s.re - A[dynamic_simulation_B.i_m].im *
              dynamic_simulation_B.s.im);
            dynamic_simulation_B.atmp_im = A[dynamic_simulation_B.k_m].im *
              dynamic_simulation_B.mul + (A[dynamic_simulation_B.i_m].im *
              dynamic_simulation_B.s.re + A[dynamic_simulation_B.i_m].re *
              dynamic_simulation_B.s.im);
            dynamic_simulation_B.d = A[dynamic_simulation_B.k_m].im;
            dynamic_simulation_B.d1 = A[dynamic_simulation_B.k_m].re;
            A[dynamic_simulation_B.i_m].re = A[dynamic_simulation_B.i_m].re *
              dynamic_simulation_B.mul - (A[dynamic_simulation_B.k_m].re *
              dynamic_simulation_B.s.re + A[dynamic_simulation_B.k_m].im *
              dynamic_simulation_B.s.im);
            A[dynamic_simulation_B.i_m].im = A[dynamic_simulation_B.i_m].im *
              dynamic_simulation_B.mul - (dynamic_simulation_B.s.re *
              dynamic_simulation_B.d - dynamic_simulation_B.s.im *
              dynamic_simulation_B.d1);
            A[dynamic_simulation_B.k_m].re = dynamic_simulation_B.atmp_re;
            A[dynamic_simulation_B.k_m].im = dynamic_simulation_B.atmp_im;
            dynamic_simulation_B.nzcount++;
          }

          dynamic_simulation_B.s.re = -dynamic_simulation_B.s.re;
          dynamic_simulation_B.s.im = -dynamic_simulation_B.s.im;
          dynamic_simulation_B.nzcount = 0;
          while (dynamic_simulation_B.nzcount + 1 <= dynamic_simulation_B.ihi) {
            dynamic_simulation_B.i_m = ((dynamic_simulation_B.ii - 1) << 2) +
              dynamic_simulation_B.nzcount;
            dynamic_simulation_B.k_m = (dynamic_simulation_B.ii << 2) +
              dynamic_simulation_B.nzcount;
            dynamic_simulation_B.atmp_re = (A[dynamic_simulation_B.i_m].re *
              dynamic_simulation_B.s.re - A[dynamic_simulation_B.i_m].im *
              dynamic_simulation_B.s.im) + A[dynamic_simulation_B.k_m].re *
              dynamic_simulation_B.mul;
            dynamic_simulation_B.atmp_im = (A[dynamic_simulation_B.i_m].im *
              dynamic_simulation_B.s.re + A[dynamic_simulation_B.i_m].re *
              dynamic_simulation_B.s.im) + A[dynamic_simulation_B.k_m].im *
              dynamic_simulation_B.mul;
            dynamic_simulation_B.d = A[dynamic_simulation_B.k_m].im;
            dynamic_simulation_B.d1 = A[dynamic_simulation_B.k_m].re;
            A[dynamic_simulation_B.i_m].re = A[dynamic_simulation_B.i_m].re *
              dynamic_simulation_B.mul - (A[dynamic_simulation_B.k_m].re *
              dynamic_simulation_B.s.re + A[dynamic_simulation_B.k_m].im *
              dynamic_simulation_B.s.im);
            A[dynamic_simulation_B.i_m].im = A[dynamic_simulation_B.i_m].im *
              dynamic_simulation_B.mul - (dynamic_simulation_B.s.re *
              dynamic_simulation_B.d - dynamic_simulation_B.s.im *
              dynamic_simulation_B.d1);
            A[dynamic_simulation_B.k_m].re = dynamic_simulation_B.atmp_re;
            A[dynamic_simulation_B.k_m].im = dynamic_simulation_B.atmp_im;
            dynamic_simulation_B.nzcount++;
          }

          dynamic_simulation_B.i_m = (dynamic_simulation_B.ii - 1) << 2;
          dynamic_simulation_B.k_m = dynamic_simulation_B.ii << 2;
          dynamic_simulation_B.atmp_re = (V[dynamic_simulation_B.i_m].re *
            dynamic_simulation_B.s.re - V[dynamic_simulation_B.i_m].im *
            dynamic_simulation_B.s.im) + V[dynamic_simulation_B.k_m].re *
            dynamic_simulation_B.mul;
          dynamic_simulation_B.atmp_im = (V[dynamic_simulation_B.i_m].im *
            dynamic_simulation_B.s.re + V[dynamic_simulation_B.i_m].re *
            dynamic_simulation_B.s.im) + V[dynamic_simulation_B.k_m].im *
            dynamic_simulation_B.mul;
          dynamic_simulation_B.d = V[dynamic_simulation_B.k_m].re;
          V[dynamic_simulation_B.i_m].re = V[dynamic_simulation_B.i_m].re *
            dynamic_simulation_B.mul - (V[dynamic_simulation_B.k_m].re *
            dynamic_simulation_B.s.re + V[dynamic_simulation_B.k_m].im *
            dynamic_simulation_B.s.im);
          V[dynamic_simulation_B.i_m].im = V[dynamic_simulation_B.i_m].im *
            dynamic_simulation_B.mul - (V[dynamic_simulation_B.k_m].im *
            dynamic_simulation_B.s.re - dynamic_simulation_B.s.im *
            dynamic_simulation_B.d);
          V[dynamic_simulation_B.k_m].re = dynamic_simulation_B.atmp_re;
          V[dynamic_simulation_B.k_m].im = dynamic_simulation_B.atmp_im;
          dynamic_simulation_B.nzcount = dynamic_simulation_B.i_m + 1;
          dynamic_simulation_B.jj = dynamic_simulation_B.k_m + 1;
          dynamic_simulation_B.atmp_re = (V[dynamic_simulation_B.nzcount].re *
            dynamic_simulation_B.s.re - V[dynamic_simulation_B.nzcount].im *
            dynamic_simulation_B.s.im) + V[dynamic_simulation_B.jj].re *
            dynamic_simulation_B.mul;
          dynamic_simulation_B.atmp_im = (V[dynamic_simulation_B.nzcount].im *
            dynamic_simulation_B.s.re + V[dynamic_simulation_B.nzcount].re *
            dynamic_simulation_B.s.im) + V[dynamic_simulation_B.jj].im *
            dynamic_simulation_B.mul;
          dynamic_simulation_B.d = V[dynamic_simulation_B.jj].re;
          V[dynamic_simulation_B.nzcount].re = V[dynamic_simulation_B.nzcount].
            re * dynamic_simulation_B.mul - (V[dynamic_simulation_B.jj].re *
            dynamic_simulation_B.s.re + V[dynamic_simulation_B.jj].im *
            dynamic_simulation_B.s.im);
          V[dynamic_simulation_B.nzcount].im = V[dynamic_simulation_B.nzcount].
            im * dynamic_simulation_B.mul - (V[dynamic_simulation_B.jj].im *
            dynamic_simulation_B.s.re - dynamic_simulation_B.s.im *
            dynamic_simulation_B.d);
          V[dynamic_simulation_B.jj].re = dynamic_simulation_B.atmp_re;
          V[dynamic_simulation_B.jj].im = dynamic_simulation_B.atmp_im;
          dynamic_simulation_B.nzcount = dynamic_simulation_B.i_m + 2;
          dynamic_simulation_B.jj = dynamic_simulation_B.k_m + 2;
          dynamic_simulation_B.atmp_re = (V[dynamic_simulation_B.nzcount].re *
            dynamic_simulation_B.s.re - V[dynamic_simulation_B.nzcount].im *
            dynamic_simulation_B.s.im) + V[dynamic_simulation_B.jj].re *
            dynamic_simulation_B.mul;
          dynamic_simulation_B.atmp_im = (V[dynamic_simulation_B.nzcount].im *
            dynamic_simulation_B.s.re + V[dynamic_simulation_B.nzcount].re *
            dynamic_simulation_B.s.im) + V[dynamic_simulation_B.jj].im *
            dynamic_simulation_B.mul;
          dynamic_simulation_B.d = V[dynamic_simulation_B.jj].re;
          V[dynamic_simulation_B.nzcount].re = V[dynamic_simulation_B.nzcount].
            re * dynamic_simulation_B.mul - (V[dynamic_simulation_B.jj].re *
            dynamic_simulation_B.s.re + V[dynamic_simulation_B.jj].im *
            dynamic_simulation_B.s.im);
          V[dynamic_simulation_B.nzcount].im = V[dynamic_simulation_B.nzcount].
            im * dynamic_simulation_B.mul - (V[dynamic_simulation_B.jj].im *
            dynamic_simulation_B.s.re - dynamic_simulation_B.s.im *
            dynamic_simulation_B.d);
          V[dynamic_simulation_B.jj].re = dynamic_simulation_B.atmp_re;
          V[dynamic_simulation_B.jj].im = dynamic_simulation_B.atmp_im;
          dynamic_simulation_B.i_m += 3;
          dynamic_simulation_B.k_m += 3;
          dynamic_simulation_B.atmp_re = (V[dynamic_simulation_B.i_m].re *
            dynamic_simulation_B.s.re - V[dynamic_simulation_B.i_m].im *
            dynamic_simulation_B.s.im) + V[dynamic_simulation_B.k_m].re *
            dynamic_simulation_B.mul;
          dynamic_simulation_B.atmp_im = (V[dynamic_simulation_B.i_m].im *
            dynamic_simulation_B.s.re + V[dynamic_simulation_B.i_m].re *
            dynamic_simulation_B.s.im) + V[dynamic_simulation_B.k_m].im *
            dynamic_simulation_B.mul;
          dynamic_simulation_B.d = V[dynamic_simulation_B.k_m].re;
          V[dynamic_simulation_B.i_m].re = V[dynamic_simulation_B.i_m].re *
            dynamic_simulation_B.mul - (V[dynamic_simulation_B.k_m].re *
            dynamic_simulation_B.s.re + V[dynamic_simulation_B.k_m].im *
            dynamic_simulation_B.s.im);
          V[dynamic_simulation_B.i_m].im = V[dynamic_simulation_B.i_m].im *
            dynamic_simulation_B.mul - (V[dynamic_simulation_B.k_m].im *
            dynamic_simulation_B.s.re - dynamic_simulation_B.s.im *
            dynamic_simulation_B.d);
          V[dynamic_simulation_B.k_m].re = dynamic_simulation_B.atmp_re;
          V[dynamic_simulation_B.k_m].im = dynamic_simulation_B.atmp_im;
          dynamic_simulation_B.ii--;
        }

        dynamic_simulation_B.jcol++;
      }
    }

    dynamic_simulation_xzhgeqz(A, dynamic_simulation_B.c_i + 1,
      dynamic_simulation_B.ihi, V, info, alpha1, beta1);
    if (*info == 0) {
      dynamic_simulation_xztgevc(A, V);
      if (dynamic_simulation_B.c_i + 1 > 1) {
        dynamic_simulation_B.c_i--;
        while (dynamic_simulation_B.c_i + 1 >= 1) {
          dynamic_simulation_B.k_m =
            dynamic_simulation_B.rscale[dynamic_simulation_B.c_i] - 1;
          if (dynamic_simulation_B.c_i + 1 !=
              dynamic_simulation_B.rscale[dynamic_simulation_B.c_i]) {
            dynamic_simulation_B.atmp_re = V[dynamic_simulation_B.c_i].re;
            dynamic_simulation_B.atmp_im = V[dynamic_simulation_B.c_i].im;
            V[dynamic_simulation_B.c_i] = V[dynamic_simulation_B.k_m];
            V[dynamic_simulation_B.k_m].re = dynamic_simulation_B.atmp_re;
            V[dynamic_simulation_B.k_m].im = dynamic_simulation_B.atmp_im;
            dynamic_simulation_B.atmp_re = V[dynamic_simulation_B.c_i + 4].re;
            dynamic_simulation_B.atmp_im = V[dynamic_simulation_B.c_i + 4].im;
            V[dynamic_simulation_B.c_i + 4] = V[dynamic_simulation_B.k_m + 4];
            V[dynamic_simulation_B.k_m + 4].re = dynamic_simulation_B.atmp_re;
            V[dynamic_simulation_B.k_m + 4].im = dynamic_simulation_B.atmp_im;
            dynamic_simulation_B.atmp_re = V[dynamic_simulation_B.c_i + 8].re;
            dynamic_simulation_B.atmp_im = V[dynamic_simulation_B.c_i + 8].im;
            V[dynamic_simulation_B.c_i + 8] = V[dynamic_simulation_B.k_m + 8];
            V[dynamic_simulation_B.k_m + 8].re = dynamic_simulation_B.atmp_re;
            V[dynamic_simulation_B.k_m + 8].im = dynamic_simulation_B.atmp_im;
            dynamic_simulation_B.atmp_re = V[dynamic_simulation_B.c_i + 12].re;
            dynamic_simulation_B.atmp_im = V[dynamic_simulation_B.c_i + 12].im;
            V[dynamic_simulation_B.c_i + 12] = V[dynamic_simulation_B.k_m + 12];
            V[dynamic_simulation_B.k_m + 12].re = dynamic_simulation_B.atmp_re;
            V[dynamic_simulation_B.k_m + 12].im = dynamic_simulation_B.atmp_im;
          }

          dynamic_simulation_B.c_i--;
        }
      }

      if (dynamic_simulation_B.ihi < 4) {
        while (dynamic_simulation_B.ihi + 1 < 5) {
          dynamic_simulation_B.k_m =
            dynamic_simulation_B.rscale[dynamic_simulation_B.ihi] - 1;
          if (dynamic_simulation_B.ihi + 1 !=
              dynamic_simulation_B.rscale[dynamic_simulation_B.ihi]) {
            dynamic_simulation_B.atmp_re = V[dynamic_simulation_B.ihi].re;
            dynamic_simulation_B.atmp_im = V[dynamic_simulation_B.ihi].im;
            V[dynamic_simulation_B.ihi] = V[dynamic_simulation_B.k_m];
            V[dynamic_simulation_B.k_m].re = dynamic_simulation_B.atmp_re;
            V[dynamic_simulation_B.k_m].im = dynamic_simulation_B.atmp_im;
            dynamic_simulation_B.atmp_re = V[dynamic_simulation_B.ihi + 4].re;
            dynamic_simulation_B.atmp_im = V[dynamic_simulation_B.ihi + 4].im;
            V[dynamic_simulation_B.ihi + 4] = V[dynamic_simulation_B.k_m + 4];
            V[dynamic_simulation_B.k_m + 4].re = dynamic_simulation_B.atmp_re;
            V[dynamic_simulation_B.k_m + 4].im = dynamic_simulation_B.atmp_im;
            dynamic_simulation_B.atmp_re = V[dynamic_simulation_B.ihi + 8].re;
            dynamic_simulation_B.atmp_im = V[dynamic_simulation_B.ihi + 8].im;
            V[dynamic_simulation_B.ihi + 8] = V[dynamic_simulation_B.k_m + 8];
            V[dynamic_simulation_B.k_m + 8].re = dynamic_simulation_B.atmp_re;
            V[dynamic_simulation_B.k_m + 8].im = dynamic_simulation_B.atmp_im;
            dynamic_simulation_B.atmp_re = V[dynamic_simulation_B.ihi + 12].re;
            dynamic_simulation_B.atmp_im = V[dynamic_simulation_B.ihi + 12].im;
            V[dynamic_simulation_B.ihi + 12] = V[dynamic_simulation_B.k_m + 12];
            V[dynamic_simulation_B.k_m + 12].re = dynamic_simulation_B.atmp_re;
            V[dynamic_simulation_B.k_m + 12].im = dynamic_simulation_B.atmp_im;
          }

          dynamic_simulation_B.ihi++;
        }
      }

      for (dynamic_simulation_B.ihi = 0; dynamic_simulation_B.ihi < 4;
           dynamic_simulation_B.ihi++) {
        dynamic_simulation_B.c_i = dynamic_simulation_B.ihi << 2;
        dynamic_simulation_B.atmp_re = fabs(V[dynamic_simulation_B.c_i].re) +
          fabs(V[dynamic_simulation_B.c_i].im);
        dynamic_simulation_B.k_m = dynamic_simulation_B.c_i + 1;
        dynamic_simulation_B.atmp_im = fabs(V[dynamic_simulation_B.k_m].re) +
          fabs(V[dynamic_simulation_B.k_m].im);
        if (dynamic_simulation_B.atmp_im > dynamic_simulation_B.atmp_re) {
          dynamic_simulation_B.atmp_re = dynamic_simulation_B.atmp_im;
        }

        dynamic_simulation_B.i_m = dynamic_simulation_B.c_i + 2;
        dynamic_simulation_B.atmp_im = fabs(V[dynamic_simulation_B.i_m].re) +
          fabs(V[dynamic_simulation_B.i_m].im);
        if (dynamic_simulation_B.atmp_im > dynamic_simulation_B.atmp_re) {
          dynamic_simulation_B.atmp_re = dynamic_simulation_B.atmp_im;
        }

        dynamic_simulation_B.jcol = dynamic_simulation_B.c_i + 3;
        dynamic_simulation_B.atmp_im = fabs(V[dynamic_simulation_B.jcol].re) +
          fabs(V[dynamic_simulation_B.jcol].im);
        if (dynamic_simulation_B.atmp_im > dynamic_simulation_B.atmp_re) {
          dynamic_simulation_B.atmp_re = dynamic_simulation_B.atmp_im;
        }

        if (dynamic_simulation_B.atmp_re >= 6.7178761075670888E-139) {
          dynamic_simulation_B.atmp_re = 1.0 / dynamic_simulation_B.atmp_re;
          V[dynamic_simulation_B.c_i].re *= dynamic_simulation_B.atmp_re;
          V[dynamic_simulation_B.c_i].im *= dynamic_simulation_B.atmp_re;
          V[dynamic_simulation_B.k_m].re *= dynamic_simulation_B.atmp_re;
          V[dynamic_simulation_B.k_m].im *= dynamic_simulation_B.atmp_re;
          V[dynamic_simulation_B.i_m].re *= dynamic_simulation_B.atmp_re;
          V[dynamic_simulation_B.i_m].im *= dynamic_simulation_B.atmp_re;
          V[dynamic_simulation_B.jcol].re *= dynamic_simulation_B.atmp_re;
          V[dynamic_simulation_B.jcol].im *= dynamic_simulation_B.atmp_re;
        }
      }

      if (dynamic_simulation_B.ilascl) {
        dynamic_simulation_B.ilascl = true;
        while (dynamic_simulation_B.ilascl) {
          dynamic_simulation_B.atmp_re = dynamic_simulation_B.anrmto *
            2.0041683600089728E-292;
          dynamic_simulation_B.atmp_im = dynamic_simulation_B.anrm /
            4.9896007738368E+291;
          if ((fabs(dynamic_simulation_B.atmp_re) > fabs
               (dynamic_simulation_B.anrm)) && (dynamic_simulation_B.anrm != 0.0))
          {
            dynamic_simulation_B.mul = 2.0041683600089728E-292;
            dynamic_simulation_B.anrmto = dynamic_simulation_B.atmp_re;
          } else if (fabs(dynamic_simulation_B.atmp_im) > fabs
                     (dynamic_simulation_B.anrmto)) {
            dynamic_simulation_B.mul = 4.9896007738368E+291;
            dynamic_simulation_B.anrm = dynamic_simulation_B.atmp_im;
          } else {
            dynamic_simulation_B.mul = dynamic_simulation_B.anrm /
              dynamic_simulation_B.anrmto;
            dynamic_simulation_B.ilascl = false;
          }

          alpha1[0].re *= dynamic_simulation_B.mul;
          alpha1[0].im *= dynamic_simulation_B.mul;
          alpha1[1].re *= dynamic_simulation_B.mul;
          alpha1[1].im *= dynamic_simulation_B.mul;
          alpha1[2].re *= dynamic_simulation_B.mul;
          alpha1[2].im *= dynamic_simulation_B.mul;
          alpha1[3].re *= dynamic_simulation_B.mul;
          alpha1[3].im *= dynamic_simulation_B.mul;
        }
      }
    }
  }
}

// Function for MATLAB Function: '<S2>/MATLAB Function'
static real_T dynamic_simulation_xnrm2(int32_T n, const real_T x[16], int32_T
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

// Function for MATLAB Function: '<S2>/MATLAB Function'
static void dynamic_simulation_xzlarf(int32_T m, int32_T n, int32_T iv0, real_T
  tau, real_T C[16], int32_T ic0, real_T work[4])
{
  int32_T exitg1;
  boolean_T exitg2;
  if (tau != 0.0) {
    dynamic_simulation_B.lastv = m;
    dynamic_simulation_B.lastc_m = iv0 + m;
    while ((dynamic_simulation_B.lastv > 0) && (C[dynamic_simulation_B.lastc_m -
            2] == 0.0)) {
      dynamic_simulation_B.lastv--;
      dynamic_simulation_B.lastc_m--;
    }

    dynamic_simulation_B.lastc_m = n - 1;
    exitg2 = false;
    while ((!exitg2) && (dynamic_simulation_B.lastc_m + 1 > 0)) {
      dynamic_simulation_B.coltop = (dynamic_simulation_B.lastc_m << 2) + ic0;
      dynamic_simulation_B.jy_c = dynamic_simulation_B.coltop;
      do {
        exitg1 = 0;
        if (dynamic_simulation_B.jy_c <= (dynamic_simulation_B.coltop +
             dynamic_simulation_B.lastv) - 1) {
          if (C[dynamic_simulation_B.jy_c - 1] != 0.0) {
            exitg1 = 1;
          } else {
            dynamic_simulation_B.jy_c++;
          }
        } else {
          dynamic_simulation_B.lastc_m--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    dynamic_simulation_B.lastv = 0;
    dynamic_simulation_B.lastc_m = -1;
  }

  if (dynamic_simulation_B.lastv > 0) {
    if (dynamic_simulation_B.lastc_m + 1 != 0) {
      dynamic_simulation_B.coltop = 0;
      while (dynamic_simulation_B.coltop <= dynamic_simulation_B.lastc_m) {
        work[dynamic_simulation_B.coltop] = 0.0;
        dynamic_simulation_B.coltop++;
      }

      dynamic_simulation_B.coltop = 0;
      dynamic_simulation_B.jy_c = (dynamic_simulation_B.lastc_m << 2) + ic0;
      dynamic_simulation_B.iac_d = ic0;
      while (dynamic_simulation_B.iac_d <= dynamic_simulation_B.jy_c) {
        dynamic_simulation_B.ix_p = iv0;
        dynamic_simulation_B.c = 0.0;
        dynamic_simulation_B.d_g = (dynamic_simulation_B.iac_d +
          dynamic_simulation_B.lastv) - 1;
        dynamic_simulation_B.b_ia_c = dynamic_simulation_B.iac_d;
        while (dynamic_simulation_B.b_ia_c <= dynamic_simulation_B.d_g) {
          dynamic_simulation_B.c += C[dynamic_simulation_B.b_ia_c - 1] *
            C[dynamic_simulation_B.ix_p - 1];
          dynamic_simulation_B.ix_p++;
          dynamic_simulation_B.b_ia_c++;
        }

        work[dynamic_simulation_B.coltop] += dynamic_simulation_B.c;
        dynamic_simulation_B.coltop++;
        dynamic_simulation_B.iac_d += 4;
      }
    }

    if (!(-tau == 0.0)) {
      dynamic_simulation_B.coltop = ic0 - 1;
      dynamic_simulation_B.jy_c = 0;
      dynamic_simulation_B.iac_d = 0;
      while (dynamic_simulation_B.iac_d <= dynamic_simulation_B.lastc_m) {
        if (work[dynamic_simulation_B.jy_c] != 0.0) {
          dynamic_simulation_B.c = work[dynamic_simulation_B.jy_c] * -tau;
          dynamic_simulation_B.ix_p = iv0;
          dynamic_simulation_B.d_g = dynamic_simulation_B.lastv +
            dynamic_simulation_B.coltop;
          dynamic_simulation_B.b_ia_c = dynamic_simulation_B.coltop;
          while (dynamic_simulation_B.b_ia_c + 1 <= dynamic_simulation_B.d_g) {
            C[dynamic_simulation_B.b_ia_c] += C[dynamic_simulation_B.ix_p - 1] *
              dynamic_simulation_B.c;
            dynamic_simulation_B.ix_p++;
            dynamic_simulation_B.b_ia_c++;
          }
        }

        dynamic_simulation_B.jy_c++;
        dynamic_simulation_B.coltop += 4;
        dynamic_simulation_B.iac_d++;
      }
    }
  }
}

// Function for MATLAB Function: '<S2>/MATLAB Function'
static void dynamic_simulation_xgehrd(real_T a[16], real_T tau[3])
{
  int32_T exitg1;
  boolean_T exitg2;
  dynamic_simulation_B.work_o[0] = 0.0;
  dynamic_simulation_B.work_o[1] = 0.0;
  dynamic_simulation_B.work_o[2] = 0.0;
  dynamic_simulation_B.work_o[3] = 0.0;
  dynamic_simulation_B.alpha1 = a[1];
  tau[0] = 0.0;
  dynamic_simulation_B.xnorm = dynamic_simulation_xnrm2(2, a, 3);
  if (dynamic_simulation_B.xnorm != 0.0) {
    dynamic_simulation_B.xnorm = dynamic_simulatio_rt_hypotd_snf(a[1],
      dynamic_simulation_B.xnorm);
    if (a[1] >= 0.0) {
      dynamic_simulation_B.xnorm = -dynamic_simulation_B.xnorm;
    }

    if (fabs(dynamic_simulation_B.xnorm) < 1.0020841800044864E-292) {
      dynamic_simulation_B.knt = -1;
      do {
        dynamic_simulation_B.knt++;
        dynamic_simulation_B.lastc = 3;
        while (dynamic_simulation_B.lastc <= 4) {
          a[dynamic_simulation_B.lastc - 1] *= 9.9792015476736E+291;
          dynamic_simulation_B.lastc++;
        }

        dynamic_simulation_B.xnorm *= 9.9792015476736E+291;
        dynamic_simulation_B.alpha1 *= 9.9792015476736E+291;
      } while (!(fabs(dynamic_simulation_B.xnorm) >= 1.0020841800044864E-292));

      dynamic_simulation_B.xnorm = dynamic_simulatio_rt_hypotd_snf
        (dynamic_simulation_B.alpha1, dynamic_simulation_xnrm2(2, a, 3));
      if (dynamic_simulation_B.alpha1 >= 0.0) {
        dynamic_simulation_B.xnorm = -dynamic_simulation_B.xnorm;
      }

      tau[0] = (dynamic_simulation_B.xnorm - dynamic_simulation_B.alpha1) /
        dynamic_simulation_B.xnorm;
      dynamic_simulation_B.alpha1 = 1.0 / (dynamic_simulation_B.alpha1 -
        dynamic_simulation_B.xnorm);
      dynamic_simulation_B.lastc = 3;
      while (dynamic_simulation_B.lastc <= 4) {
        a[dynamic_simulation_B.lastc - 1] *= dynamic_simulation_B.alpha1;
        dynamic_simulation_B.lastc++;
      }

      dynamic_simulation_B.lastc = 0;
      while (dynamic_simulation_B.lastc <= dynamic_simulation_B.knt) {
        dynamic_simulation_B.xnorm *= 1.0020841800044864E-292;
        dynamic_simulation_B.lastc++;
      }

      dynamic_simulation_B.alpha1 = dynamic_simulation_B.xnorm;
    } else {
      tau[0] = (dynamic_simulation_B.xnorm - a[1]) / dynamic_simulation_B.xnorm;
      dynamic_simulation_B.alpha1 = 1.0 / (a[1] - dynamic_simulation_B.xnorm);
      dynamic_simulation_B.knt = 3;
      while (dynamic_simulation_B.knt <= 4) {
        a[dynamic_simulation_B.knt - 1] *= dynamic_simulation_B.alpha1;
        dynamic_simulation_B.knt++;
      }

      dynamic_simulation_B.alpha1 = dynamic_simulation_B.xnorm;
    }
  }

  a[1] = 1.0;
  if (tau[0] != 0.0) {
    dynamic_simulation_B.knt = 2;
    dynamic_simulation_B.lastc = 3;
    while ((dynamic_simulation_B.knt + 1 > 0) && (a[dynamic_simulation_B.lastc] ==
            0.0)) {
      dynamic_simulation_B.knt--;
      dynamic_simulation_B.lastc--;
    }

    dynamic_simulation_B.lastc = 4;
    exitg2 = false;
    while ((!exitg2) && (dynamic_simulation_B.lastc > 0)) {
      dynamic_simulation_B.rowleft = dynamic_simulation_B.lastc + 4;
      dynamic_simulation_B.jy = dynamic_simulation_B.rowleft;
      do {
        exitg1 = 0;
        if (dynamic_simulation_B.jy <= (dynamic_simulation_B.knt << 2) +
            dynamic_simulation_B.rowleft) {
          if (a[dynamic_simulation_B.jy - 1] != 0.0) {
            exitg1 = 1;
          } else {
            dynamic_simulation_B.jy += 4;
          }
        } else {
          dynamic_simulation_B.lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    dynamic_simulation_B.knt = -1;
    dynamic_simulation_B.lastc = 0;
  }

  if (dynamic_simulation_B.knt + 1 > 0) {
    if (dynamic_simulation_B.lastc != 0) {
      dynamic_simulation_B.rowleft = 0;
      while (dynamic_simulation_B.rowleft <= dynamic_simulation_B.lastc - 1) {
        dynamic_simulation_B.work_o[dynamic_simulation_B.rowleft] = 0.0;
        dynamic_simulation_B.rowleft++;
      }

      dynamic_simulation_B.rowleft = 1;
      dynamic_simulation_B.jy = (dynamic_simulation_B.knt << 2) + 5;
      dynamic_simulation_B.iac = 5;
      while (dynamic_simulation_B.iac <= dynamic_simulation_B.jy) {
        dynamic_simulation_B.b_ix = 0;
        dynamic_simulation_B.g = (dynamic_simulation_B.iac +
          dynamic_simulation_B.lastc) - 1;
        dynamic_simulation_B.b_ia = dynamic_simulation_B.iac;
        while (dynamic_simulation_B.b_ia <= dynamic_simulation_B.g) {
          dynamic_simulation_B.work_o[dynamic_simulation_B.b_ix] +=
            a[dynamic_simulation_B.b_ia - 1] * a[dynamic_simulation_B.rowleft];
          dynamic_simulation_B.b_ix++;
          dynamic_simulation_B.b_ia++;
        }

        dynamic_simulation_B.rowleft++;
        dynamic_simulation_B.iac += 4;
      }
    }

    if (!(-tau[0] == 0.0)) {
      dynamic_simulation_B.rowleft = 4;
      dynamic_simulation_B.jy = 1;
      dynamic_simulation_B.iac = 0;
      while (dynamic_simulation_B.iac <= dynamic_simulation_B.knt) {
        if (a[dynamic_simulation_B.jy] != 0.0) {
          dynamic_simulation_B.xnorm = a[dynamic_simulation_B.jy] * -tau[0];
          dynamic_simulation_B.b_ix = 0;
          dynamic_simulation_B.g = dynamic_simulation_B.lastc +
            dynamic_simulation_B.rowleft;
          dynamic_simulation_B.b_ia = dynamic_simulation_B.rowleft;
          while (dynamic_simulation_B.b_ia + 1 <= dynamic_simulation_B.g) {
            a[dynamic_simulation_B.b_ia] +=
              dynamic_simulation_B.work_o[dynamic_simulation_B.b_ix] *
              dynamic_simulation_B.xnorm;
            dynamic_simulation_B.b_ix++;
            dynamic_simulation_B.b_ia++;
          }
        }

        dynamic_simulation_B.jy++;
        dynamic_simulation_B.rowleft += 4;
        dynamic_simulation_B.iac++;
      }
    }
  }

  dynamic_simulation_xzlarf(3, 3, 2, tau[0], a, 6, dynamic_simulation_B.work_o);
  a[1] = dynamic_simulation_B.alpha1;
  dynamic_simulation_B.alpha1 = a[6];
  tau[1] = 0.0;
  dynamic_simulation_B.xnorm = dynamic_simulation_xnrm2(1, a, 8);
  if (dynamic_simulation_B.xnorm != 0.0) {
    dynamic_simulation_B.xnorm = dynamic_simulatio_rt_hypotd_snf(a[6],
      dynamic_simulation_B.xnorm);
    if (a[6] >= 0.0) {
      dynamic_simulation_B.xnorm = -dynamic_simulation_B.xnorm;
    }

    if (fabs(dynamic_simulation_B.xnorm) < 1.0020841800044864E-292) {
      dynamic_simulation_B.knt = -1;
      do {
        dynamic_simulation_B.knt++;
        a[7] *= 9.9792015476736E+291;
        dynamic_simulation_B.xnorm *= 9.9792015476736E+291;
        dynamic_simulation_B.alpha1 *= 9.9792015476736E+291;
      } while (!(fabs(dynamic_simulation_B.xnorm) >= 1.0020841800044864E-292));

      dynamic_simulation_B.xnorm = dynamic_simulatio_rt_hypotd_snf
        (dynamic_simulation_B.alpha1, dynamic_simulation_xnrm2(1, a, 8));
      if (dynamic_simulation_B.alpha1 >= 0.0) {
        dynamic_simulation_B.xnorm = -dynamic_simulation_B.xnorm;
      }

      tau[1] = (dynamic_simulation_B.xnorm - dynamic_simulation_B.alpha1) /
        dynamic_simulation_B.xnorm;
      dynamic_simulation_B.alpha1 = 1.0 / (dynamic_simulation_B.alpha1 -
        dynamic_simulation_B.xnorm);
      a[7] *= dynamic_simulation_B.alpha1;
      dynamic_simulation_B.lastc = 0;
      while (dynamic_simulation_B.lastc <= dynamic_simulation_B.knt) {
        dynamic_simulation_B.xnorm *= 1.0020841800044864E-292;
        dynamic_simulation_B.lastc++;
      }

      dynamic_simulation_B.alpha1 = dynamic_simulation_B.xnorm;
    } else {
      tau[1] = (dynamic_simulation_B.xnorm - a[6]) / dynamic_simulation_B.xnorm;
      a[7] *= 1.0 / (a[6] - dynamic_simulation_B.xnorm);
      dynamic_simulation_B.alpha1 = dynamic_simulation_B.xnorm;
    }
  }

  a[6] = 1.0;
  if (tau[1] != 0.0) {
    dynamic_simulation_B.knt = 1;
    dynamic_simulation_B.lastc = 7;
    while ((dynamic_simulation_B.knt + 1 > 0) && (a[dynamic_simulation_B.lastc] ==
            0.0)) {
      dynamic_simulation_B.knt--;
      dynamic_simulation_B.lastc--;
    }

    dynamic_simulation_B.lastc = 4;
    exitg2 = false;
    while ((!exitg2) && (dynamic_simulation_B.lastc > 0)) {
      dynamic_simulation_B.rowleft = dynamic_simulation_B.lastc + 8;
      dynamic_simulation_B.jy = dynamic_simulation_B.rowleft;
      do {
        exitg1 = 0;
        if (dynamic_simulation_B.jy <= (dynamic_simulation_B.knt << 2) +
            dynamic_simulation_B.rowleft) {
          if (a[dynamic_simulation_B.jy - 1] != 0.0) {
            exitg1 = 1;
          } else {
            dynamic_simulation_B.jy += 4;
          }
        } else {
          dynamic_simulation_B.lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    dynamic_simulation_B.knt = -1;
    dynamic_simulation_B.lastc = 0;
  }

  if (dynamic_simulation_B.knt + 1 > 0) {
    if (dynamic_simulation_B.lastc != 0) {
      dynamic_simulation_B.rowleft = 0;
      while (dynamic_simulation_B.rowleft <= dynamic_simulation_B.lastc - 1) {
        dynamic_simulation_B.work_o[dynamic_simulation_B.rowleft] = 0.0;
        dynamic_simulation_B.rowleft++;
      }

      dynamic_simulation_B.rowleft = 6;
      dynamic_simulation_B.jy = (dynamic_simulation_B.knt << 2) + 9;
      dynamic_simulation_B.iac = 9;
      while (dynamic_simulation_B.iac <= dynamic_simulation_B.jy) {
        dynamic_simulation_B.b_ix = 0;
        dynamic_simulation_B.g = (dynamic_simulation_B.iac +
          dynamic_simulation_B.lastc) - 1;
        dynamic_simulation_B.b_ia = dynamic_simulation_B.iac;
        while (dynamic_simulation_B.b_ia <= dynamic_simulation_B.g) {
          dynamic_simulation_B.work_o[dynamic_simulation_B.b_ix] +=
            a[dynamic_simulation_B.b_ia - 1] * a[dynamic_simulation_B.rowleft];
          dynamic_simulation_B.b_ix++;
          dynamic_simulation_B.b_ia++;
        }

        dynamic_simulation_B.rowleft++;
        dynamic_simulation_B.iac += 4;
      }
    }

    if (!(-tau[1] == 0.0)) {
      dynamic_simulation_B.rowleft = 8;
      dynamic_simulation_B.jy = 6;
      dynamic_simulation_B.iac = 0;
      while (dynamic_simulation_B.iac <= dynamic_simulation_B.knt) {
        if (a[dynamic_simulation_B.jy] != 0.0) {
          dynamic_simulation_B.xnorm = a[dynamic_simulation_B.jy] * -tau[1];
          dynamic_simulation_B.b_ix = 0;
          dynamic_simulation_B.g = dynamic_simulation_B.lastc +
            dynamic_simulation_B.rowleft;
          dynamic_simulation_B.b_ia = dynamic_simulation_B.rowleft;
          while (dynamic_simulation_B.b_ia + 1 <= dynamic_simulation_B.g) {
            a[dynamic_simulation_B.b_ia] +=
              dynamic_simulation_B.work_o[dynamic_simulation_B.b_ix] *
              dynamic_simulation_B.xnorm;
            dynamic_simulation_B.b_ix++;
            dynamic_simulation_B.b_ia++;
          }
        }

        dynamic_simulation_B.jy++;
        dynamic_simulation_B.rowleft += 4;
        dynamic_simulation_B.iac++;
      }
    }
  }

  dynamic_simulation_xzlarf(2, 2, 7, tau[1], a, 11, dynamic_simulation_B.work_o);
  a[6] = dynamic_simulation_B.alpha1;
  dynamic_simulation_B.alpha1 = a[11];
  tau[2] = 0.0;
  dynamic_simulation_B.xnorm = dynamic_simulation_xnrm2(0, a, 12);
  if (dynamic_simulation_B.xnorm != 0.0) {
    dynamic_simulation_B.xnorm = dynamic_simulatio_rt_hypotd_snf(a[11],
      dynamic_simulation_B.xnorm);
    if (a[11] >= 0.0) {
      dynamic_simulation_B.xnorm = -dynamic_simulation_B.xnorm;
    }

    if (fabs(dynamic_simulation_B.xnorm) < 1.0020841800044864E-292) {
      dynamic_simulation_B.knt = -1;
      do {
        dynamic_simulation_B.knt++;
        dynamic_simulation_B.xnorm *= 9.9792015476736E+291;
        dynamic_simulation_B.alpha1 *= 9.9792015476736E+291;
      } while (!(fabs(dynamic_simulation_B.xnorm) >= 1.0020841800044864E-292));

      dynamic_simulation_B.xnorm = dynamic_simulatio_rt_hypotd_snf
        (dynamic_simulation_B.alpha1, dynamic_simulation_xnrm2(0, a, 12));
      if (dynamic_simulation_B.alpha1 >= 0.0) {
        dynamic_simulation_B.xnorm = -dynamic_simulation_B.xnorm;
      }

      tau[2] = (dynamic_simulation_B.xnorm - dynamic_simulation_B.alpha1) /
        dynamic_simulation_B.xnorm;
      dynamic_simulation_B.lastc = 0;
      while (dynamic_simulation_B.lastc <= dynamic_simulation_B.knt) {
        dynamic_simulation_B.xnorm *= 1.0020841800044864E-292;
        dynamic_simulation_B.lastc++;
      }

      dynamic_simulation_B.alpha1 = dynamic_simulation_B.xnorm;
    } else {
      tau[2] = (dynamic_simulation_B.xnorm - a[11]) / dynamic_simulation_B.xnorm;
      dynamic_simulation_B.alpha1 = dynamic_simulation_B.xnorm;
    }
  }

  a[11] = 1.0;
  if (tau[2] != 0.0) {
    dynamic_simulation_B.knt = 0;
    dynamic_simulation_B.lastc = 11;
    while ((dynamic_simulation_B.knt + 1 > 0) && (a[dynamic_simulation_B.lastc] ==
            0.0)) {
      dynamic_simulation_B.knt--;
      dynamic_simulation_B.lastc--;
    }

    dynamic_simulation_B.lastc = 4;
    exitg2 = false;
    while ((!exitg2) && (dynamic_simulation_B.lastc > 0)) {
      dynamic_simulation_B.rowleft = dynamic_simulation_B.lastc + 12;
      dynamic_simulation_B.jy = dynamic_simulation_B.rowleft;
      do {
        exitg1 = 0;
        if (dynamic_simulation_B.jy <= (dynamic_simulation_B.knt << 2) +
            dynamic_simulation_B.rowleft) {
          if (a[dynamic_simulation_B.jy - 1] != 0.0) {
            exitg1 = 1;
          } else {
            dynamic_simulation_B.jy += 4;
          }
        } else {
          dynamic_simulation_B.lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    dynamic_simulation_B.knt = -1;
    dynamic_simulation_B.lastc = 0;
  }

  if (dynamic_simulation_B.knt + 1 > 0) {
    if (dynamic_simulation_B.lastc != 0) {
      dynamic_simulation_B.rowleft = 0;
      while (dynamic_simulation_B.rowleft <= dynamic_simulation_B.lastc - 1) {
        dynamic_simulation_B.work_o[dynamic_simulation_B.rowleft] = 0.0;
        dynamic_simulation_B.rowleft++;
      }

      dynamic_simulation_B.rowleft = 11;
      dynamic_simulation_B.jy = (dynamic_simulation_B.knt << 2) + 13;
      dynamic_simulation_B.iac = 13;
      while (dynamic_simulation_B.iac <= dynamic_simulation_B.jy) {
        dynamic_simulation_B.b_ix = 0;
        dynamic_simulation_B.g = (dynamic_simulation_B.iac +
          dynamic_simulation_B.lastc) - 1;
        dynamic_simulation_B.b_ia = dynamic_simulation_B.iac;
        while (dynamic_simulation_B.b_ia <= dynamic_simulation_B.g) {
          dynamic_simulation_B.work_o[dynamic_simulation_B.b_ix] +=
            a[dynamic_simulation_B.b_ia - 1] * a[dynamic_simulation_B.rowleft];
          dynamic_simulation_B.b_ix++;
          dynamic_simulation_B.b_ia++;
        }

        dynamic_simulation_B.rowleft++;
        dynamic_simulation_B.iac += 4;
      }
    }

    if (!(-tau[2] == 0.0)) {
      dynamic_simulation_B.rowleft = 12;
      dynamic_simulation_B.jy = 11;
      dynamic_simulation_B.iac = 0;
      while (dynamic_simulation_B.iac <= dynamic_simulation_B.knt) {
        if (a[dynamic_simulation_B.jy] != 0.0) {
          dynamic_simulation_B.xnorm = a[dynamic_simulation_B.jy] * -tau[2];
          dynamic_simulation_B.b_ix = 0;
          dynamic_simulation_B.g = dynamic_simulation_B.lastc +
            dynamic_simulation_B.rowleft;
          dynamic_simulation_B.b_ia = dynamic_simulation_B.rowleft;
          while (dynamic_simulation_B.b_ia + 1 <= dynamic_simulation_B.g) {
            a[dynamic_simulation_B.b_ia] +=
              dynamic_simulation_B.work_o[dynamic_simulation_B.b_ix] *
              dynamic_simulation_B.xnorm;
            dynamic_simulation_B.b_ix++;
            dynamic_simulation_B.b_ia++;
          }
        }

        dynamic_simulation_B.jy++;
        dynamic_simulation_B.rowleft += 4;
        dynamic_simulation_B.iac++;
      }
    }
  }

  dynamic_simulation_xzlarf(1, 1, 12, tau[2], a, 16, dynamic_simulation_B.work_o);
  a[11] = dynamic_simulation_B.alpha1;
}

// Function for MATLAB Function: '<S2>/MATLAB Function'
static real_T dynamic_simulation_xnrm2_a(int32_T n, const real_T x[3])
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

// Function for MATLAB Function: '<S2>/MATLAB Function'
static real_T dynamic_simulation_xzlarfg(int32_T n, real_T *alpha1, real_T x[3])
{
  real_T tau;
  real_T xnorm;
  int32_T knt;
  int32_T c_k;
  tau = 0.0;
  if (n > 0) {
    xnorm = dynamic_simulation_xnrm2_a(n - 1, x);
    if (xnorm != 0.0) {
      xnorm = dynamic_simulatio_rt_hypotd_snf(*alpha1, xnorm);
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

        xnorm = dynamic_simulatio_rt_hypotd_snf(*alpha1,
          dynamic_simulation_xnrm2_a(n - 1, x));
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

// Function for MATLAB Function: '<S2>/MATLAB Function'
static void dynamic_simulation_xdlanv2(real_T *a, real_T *b, real_T *c, real_T
  *d, real_T *rt1r, real_T *rt1i, real_T *rt2r, real_T *rt2i, real_T *cs, real_T
  *sn)
{
  if (*c == 0.0) {
    *cs = 1.0;
    *sn = 0.0;
  } else if (*b == 0.0) {
    *cs = 0.0;
    *sn = 1.0;
    dynamic_simulation_B.bcmax = *d;
    *d = *a;
    *a = dynamic_simulation_B.bcmax;
    *b = -*c;
    *c = 0.0;
  } else {
    dynamic_simulation_B.tau_e = *a - *d;
    if ((dynamic_simulation_B.tau_e == 0.0) && ((*b < 0.0) != (*c < 0.0))) {
      *cs = 1.0;
      *sn = 0.0;
    } else {
      dynamic_simulation_B.p = 0.5 * dynamic_simulation_B.tau_e;
      dynamic_simulation_B.bcmis = fabs(*b);
      dynamic_simulation_B.z = fabs(*c);
      dynamic_simulation_B.b_oi = rtIsNaN(dynamic_simulation_B.z);
      if ((dynamic_simulation_B.bcmis > dynamic_simulation_B.z) ||
          dynamic_simulation_B.b_oi) {
        dynamic_simulation_B.bcmax = dynamic_simulation_B.bcmis;
      } else {
        dynamic_simulation_B.bcmax = dynamic_simulation_B.z;
      }

      if ((dynamic_simulation_B.bcmis < dynamic_simulation_B.z) ||
          dynamic_simulation_B.b_oi) {
        dynamic_simulation_B.z = dynamic_simulation_B.bcmis;
      }

      if (!(*b < 0.0)) {
        dynamic_simulation_B.b_cj = 1;
      } else {
        dynamic_simulation_B.b_cj = -1;
      }

      if (!(*c < 0.0)) {
        dynamic_simulation_B.c_m1 = 1;
      } else {
        dynamic_simulation_B.c_m1 = -1;
      }

      dynamic_simulation_B.bcmis = dynamic_simulation_B.z * static_cast<real_T>
        (dynamic_simulation_B.b_cj) * static_cast<real_T>
        (dynamic_simulation_B.c_m1);
      dynamic_simulation_B.scale_j = fabs(dynamic_simulation_B.p);
      if ((!(dynamic_simulation_B.scale_j > dynamic_simulation_B.bcmax)) &&
          (!rtIsNaN(dynamic_simulation_B.bcmax))) {
        dynamic_simulation_B.scale_j = dynamic_simulation_B.bcmax;
      }

      dynamic_simulation_B.z = dynamic_simulation_B.p /
        dynamic_simulation_B.scale_j * dynamic_simulation_B.p +
        dynamic_simulation_B.bcmax / dynamic_simulation_B.scale_j *
        dynamic_simulation_B.bcmis;
      if (dynamic_simulation_B.z >= 8.8817841970012523E-16) {
        if (!(dynamic_simulation_B.p < 0.0)) {
          dynamic_simulation_B.tau_e = sqrt(dynamic_simulation_B.scale_j) * sqrt
            (dynamic_simulation_B.z);
        } else {
          dynamic_simulation_B.tau_e = -(sqrt(dynamic_simulation_B.scale_j) *
            sqrt(dynamic_simulation_B.z));
        }

        dynamic_simulation_B.z = dynamic_simulation_B.p +
          dynamic_simulation_B.tau_e;
        *a = *d + dynamic_simulation_B.z;
        *d -= dynamic_simulation_B.bcmax / dynamic_simulation_B.z *
          dynamic_simulation_B.bcmis;
        dynamic_simulation_B.tau_e = dynamic_simulatio_rt_hypotd_snf(*c,
          dynamic_simulation_B.z);
        *cs = dynamic_simulation_B.z / dynamic_simulation_B.tau_e;
        *sn = *c / dynamic_simulation_B.tau_e;
        *b -= *c;
        *c = 0.0;
      } else {
        dynamic_simulation_B.bcmax = *b + *c;
        dynamic_simulation_B.tau_e = dynamic_simulatio_rt_hypotd_snf
          (dynamic_simulation_B.bcmax, dynamic_simulation_B.tau_e);
        *cs = sqrt((fabs(dynamic_simulation_B.bcmax) /
                    dynamic_simulation_B.tau_e + 1.0) * 0.5);
        if (!(dynamic_simulation_B.bcmax < 0.0)) {
          dynamic_simulation_B.b_cj = 1;
        } else {
          dynamic_simulation_B.b_cj = -1;
        }

        *sn = -(dynamic_simulation_B.p / (dynamic_simulation_B.tau_e * *cs)) *
          static_cast<real_T>(dynamic_simulation_B.b_cj);
        dynamic_simulation_B.p = *a * *cs + *b * *sn;
        dynamic_simulation_B.tau_e = -*a * *sn + *b * *cs;
        dynamic_simulation_B.bcmax = *c * *cs + *d * *sn;
        dynamic_simulation_B.bcmis = -*c * *sn + *d * *cs;
        *b = dynamic_simulation_B.tau_e * *cs + dynamic_simulation_B.bcmis * *sn;
        *c = -dynamic_simulation_B.p * *sn + dynamic_simulation_B.bcmax * *cs;
        dynamic_simulation_B.bcmax = ((dynamic_simulation_B.p * *cs +
          dynamic_simulation_B.bcmax * *sn) + (-dynamic_simulation_B.tau_e * *sn
          + dynamic_simulation_B.bcmis * *cs)) * 0.5;
        *a = dynamic_simulation_B.bcmax;
        *d = dynamic_simulation_B.bcmax;
        if (*c != 0.0) {
          if (*b != 0.0) {
            if ((*b < 0.0) == (*c < 0.0)) {
              dynamic_simulation_B.z = sqrt(fabs(*b));
              dynamic_simulation_B.bcmis = sqrt(fabs(*c));
              if (!(*c < 0.0)) {
                dynamic_simulation_B.p = dynamic_simulation_B.z *
                  dynamic_simulation_B.bcmis;
              } else {
                dynamic_simulation_B.p = -(dynamic_simulation_B.z *
                  dynamic_simulation_B.bcmis);
              }

              dynamic_simulation_B.tau_e = 1.0 / sqrt(fabs(*b + *c));
              *a = dynamic_simulation_B.bcmax + dynamic_simulation_B.p;
              *d = dynamic_simulation_B.bcmax - dynamic_simulation_B.p;
              *b -= *c;
              *c = 0.0;
              dynamic_simulation_B.p = dynamic_simulation_B.z *
                dynamic_simulation_B.tau_e;
              dynamic_simulation_B.tau_e *= dynamic_simulation_B.bcmis;
              dynamic_simulation_B.bcmax = *cs * dynamic_simulation_B.p - *sn *
                dynamic_simulation_B.tau_e;
              *sn = *cs * dynamic_simulation_B.tau_e + *sn *
                dynamic_simulation_B.p;
              *cs = dynamic_simulation_B.bcmax;
            }
          } else {
            *b = -*c;
            *c = 0.0;
            dynamic_simulation_B.bcmax = *cs;
            *cs = -*sn;
            *sn = dynamic_simulation_B.bcmax;
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

// Function for MATLAB Function: '<S2>/MATLAB Function'
static void dynamic_simulation_xrot(int32_T n, real_T x[16], int32_T ix0,
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

// Function for MATLAB Function: '<S2>/MATLAB Function'
static void dynamic_simulation_xrot_l(int32_T n, real_T x[16], int32_T ix0,
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

// Function for MATLAB Function: '<S2>/MATLAB Function'
static int32_T dynamic_simulation_eml_dlahqr(real_T h[16], real_T z[16])
{
  int32_T info;
  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T exitg3;
  info = 0;
  dynamic_simulation_B.v_j[0] = 0.0;
  dynamic_simulation_B.v_j[1] = 0.0;
  dynamic_simulation_B.v_j[2] = 0.0;
  h[2] = 0.0;
  h[3] = 0.0;
  h[7] = 0.0;
  dynamic_simulation_B.i_k = 3;
  exitg1 = false;
  while ((!exitg1) && (dynamic_simulation_B.i_k + 1 >= 1)) {
    dynamic_simulation_B.L = 1;
    dynamic_simulation_B.goto150 = false;
    dynamic_simulation_B.ix = 0;
    exitg2 = false;
    while ((!exitg2) && (dynamic_simulation_B.ix < 301)) {
      dynamic_simulation_B.k_n = dynamic_simulation_B.i_k;
      exitg3 = false;
      while ((!exitg3) && (dynamic_simulation_B.k_n + 1 > dynamic_simulation_B.L))
      {
        dynamic_simulation_B.s_tmp = ((dynamic_simulation_B.k_n - 1) << 2) +
          dynamic_simulation_B.k_n;
        dynamic_simulation_B.htmp2 = fabs(h[dynamic_simulation_B.s_tmp]);
        if (dynamic_simulation_B.htmp2 <= 4.0083367200179456E-292) {
          exitg3 = true;
        } else {
          dynamic_simulation_B.m_o = (dynamic_simulation_B.k_n << 2) +
            dynamic_simulation_B.k_n;
          dynamic_simulation_B.nr = dynamic_simulation_B.s_tmp - 1;
          dynamic_simulation_B.tst = fabs(h[dynamic_simulation_B.nr]) + fabs
            (h[dynamic_simulation_B.m_o]);
          if (dynamic_simulation_B.tst == 0.0) {
            if (dynamic_simulation_B.k_n - 1 >= 1) {
              dynamic_simulation_B.tst = fabs(h[(((dynamic_simulation_B.k_n - 2)
                << 2) + dynamic_simulation_B.k_n) - 1]);
            }

            if (dynamic_simulation_B.k_n + 2 <= 4) {
              dynamic_simulation_B.tst += fabs(h[((dynamic_simulation_B.k_n << 2)
                + dynamic_simulation_B.k_n) + 1]);
            }
          }

          if (dynamic_simulation_B.htmp2 <= 2.2204460492503131E-16 *
              dynamic_simulation_B.tst) {
            dynamic_simulation_B.htmp1 = fabs(h[dynamic_simulation_B.s_tmp]);
            dynamic_simulation_B.htmp2 = fabs(h[dynamic_simulation_B.m_o - 1]);
            if (dynamic_simulation_B.htmp1 > dynamic_simulation_B.htmp2) {
              dynamic_simulation_B.tst = dynamic_simulation_B.htmp1;
              dynamic_simulation_B.ba = dynamic_simulation_B.htmp2;
            } else {
              dynamic_simulation_B.tst = dynamic_simulation_B.htmp2;
              dynamic_simulation_B.ba = dynamic_simulation_B.htmp1;
            }

            dynamic_simulation_B.htmp1 = fabs(h[dynamic_simulation_B.m_o]);
            dynamic_simulation_B.htmp2 = fabs(h[dynamic_simulation_B.nr] -
              h[dynamic_simulation_B.m_o]);
            if (dynamic_simulation_B.htmp1 > dynamic_simulation_B.htmp2) {
              dynamic_simulation_B.aa = dynamic_simulation_B.htmp1;
              dynamic_simulation_B.htmp1 = dynamic_simulation_B.htmp2;
            } else {
              dynamic_simulation_B.aa = dynamic_simulation_B.htmp2;
            }

            dynamic_simulation_B.htmp2 = dynamic_simulation_B.aa +
              dynamic_simulation_B.tst;
            dynamic_simulation_B.htmp1 = dynamic_simulation_B.aa /
              dynamic_simulation_B.htmp2 * dynamic_simulation_B.htmp1 *
              2.2204460492503131E-16;
            if ((4.0083367200179456E-292 > dynamic_simulation_B.htmp1) ||
                rtIsNaN(dynamic_simulation_B.htmp1)) {
              dynamic_simulation_B.htmp1 = 4.0083367200179456E-292;
            }

            if (dynamic_simulation_B.tst / dynamic_simulation_B.htmp2 *
                dynamic_simulation_B.ba <= dynamic_simulation_B.htmp1) {
              exitg3 = true;
            } else {
              dynamic_simulation_B.k_n--;
            }
          } else {
            dynamic_simulation_B.k_n--;
          }
        }
      }

      dynamic_simulation_B.L = dynamic_simulation_B.k_n + 1;
      if (dynamic_simulation_B.k_n + 1 > 1) {
        h[dynamic_simulation_B.k_n + ((dynamic_simulation_B.k_n - 1) << 2)] =
          0.0;
      }

      if (dynamic_simulation_B.k_n + 1 >= dynamic_simulation_B.i_k) {
        dynamic_simulation_B.goto150 = true;
        exitg2 = true;
      } else {
        switch (dynamic_simulation_B.ix) {
         case 10:
          dynamic_simulation_B.s_tmp = (dynamic_simulation_B.k_n << 2) +
            dynamic_simulation_B.k_n;
          dynamic_simulation_B.htmp2 = fabs(h[(((dynamic_simulation_B.k_n + 1) <<
            2) + dynamic_simulation_B.k_n) + 2]) + fabs
            (h[dynamic_simulation_B.s_tmp + 1]);
          dynamic_simulation_B.ba = h[dynamic_simulation_B.s_tmp] + 0.75 *
            dynamic_simulation_B.htmp2;
          dynamic_simulation_B.h12 = -0.4375 * dynamic_simulation_B.htmp2;
          dynamic_simulation_B.aa = dynamic_simulation_B.htmp2;
          dynamic_simulation_B.tst = dynamic_simulation_B.ba;
          break;

         case 20:
          dynamic_simulation_B.htmp2 = fabs(h[(((dynamic_simulation_B.i_k - 2) <<
            2) + dynamic_simulation_B.i_k) - 1]) + fabs(h
            [((dynamic_simulation_B.i_k - 1) << 2) + dynamic_simulation_B.i_k]);
          dynamic_simulation_B.ba = h[(dynamic_simulation_B.i_k << 2) +
            dynamic_simulation_B.i_k] + 0.75 * dynamic_simulation_B.htmp2;
          dynamic_simulation_B.h12 = -0.4375 * dynamic_simulation_B.htmp2;
          dynamic_simulation_B.aa = dynamic_simulation_B.htmp2;
          dynamic_simulation_B.tst = dynamic_simulation_B.ba;
          break;

         default:
          dynamic_simulation_B.ba = h[(((dynamic_simulation_B.i_k - 1) << 2) +
            dynamic_simulation_B.i_k) - 1];
          dynamic_simulation_B.aa = h[((dynamic_simulation_B.i_k - 1) << 2) +
            dynamic_simulation_B.i_k];
          dynamic_simulation_B.h12 = h[((dynamic_simulation_B.i_k << 2) +
            dynamic_simulation_B.i_k) - 1];
          dynamic_simulation_B.tst = h[(dynamic_simulation_B.i_k << 2) +
            dynamic_simulation_B.i_k];
          break;
        }

        dynamic_simulation_B.htmp2 = ((fabs(dynamic_simulation_B.ba) + fabs
          (dynamic_simulation_B.h12)) + fabs(dynamic_simulation_B.aa)) + fabs
          (dynamic_simulation_B.tst);
        if (dynamic_simulation_B.htmp2 == 0.0) {
          dynamic_simulation_B.ba = 0.0;
          dynamic_simulation_B.tst = 0.0;
          dynamic_simulation_B.htmp1 = 0.0;
          dynamic_simulation_B.aa = 0.0;
        } else {
          dynamic_simulation_B.ba /= dynamic_simulation_B.htmp2;
          dynamic_simulation_B.aa /= dynamic_simulation_B.htmp2;
          dynamic_simulation_B.h12 /= dynamic_simulation_B.htmp2;
          dynamic_simulation_B.tst /= dynamic_simulation_B.htmp2;
          dynamic_simulation_B.htmp1 = (dynamic_simulation_B.ba +
            dynamic_simulation_B.tst) / 2.0;
          dynamic_simulation_B.ba = (dynamic_simulation_B.ba -
            dynamic_simulation_B.htmp1) * (dynamic_simulation_B.tst -
            dynamic_simulation_B.htmp1) - dynamic_simulation_B.h12 *
            dynamic_simulation_B.aa;
          dynamic_simulation_B.aa = sqrt(fabs(dynamic_simulation_B.ba));
          if (dynamic_simulation_B.ba >= 0.0) {
            dynamic_simulation_B.ba = dynamic_simulation_B.htmp1 *
              dynamic_simulation_B.htmp2;
            dynamic_simulation_B.htmp1 = dynamic_simulation_B.ba;
            dynamic_simulation_B.tst = dynamic_simulation_B.aa *
              dynamic_simulation_B.htmp2;
            dynamic_simulation_B.aa = -dynamic_simulation_B.tst;
          } else {
            dynamic_simulation_B.ba = dynamic_simulation_B.htmp1 +
              dynamic_simulation_B.aa;
            dynamic_simulation_B.htmp1 -= dynamic_simulation_B.aa;
            if (fabs(dynamic_simulation_B.ba - dynamic_simulation_B.tst) <= fabs
                (dynamic_simulation_B.htmp1 - dynamic_simulation_B.tst)) {
              dynamic_simulation_B.ba *= dynamic_simulation_B.htmp2;
              dynamic_simulation_B.htmp1 = dynamic_simulation_B.ba;
            } else {
              dynamic_simulation_B.htmp1 *= dynamic_simulation_B.htmp2;
              dynamic_simulation_B.ba = dynamic_simulation_B.htmp1;
            }

            dynamic_simulation_B.tst = 0.0;
            dynamic_simulation_B.aa = 0.0;
          }
        }

        dynamic_simulation_B.m_o = dynamic_simulation_B.i_k - 1;
        exitg3 = false;
        while ((!exitg3) && (dynamic_simulation_B.m_o >=
                             dynamic_simulation_B.k_n + 1)) {
          dynamic_simulation_B.s_tmp = ((dynamic_simulation_B.m_o - 1) << 2) +
            dynamic_simulation_B.m_o;
          dynamic_simulation_B.nr = dynamic_simulation_B.s_tmp - 1;
          dynamic_simulation_B.h12 = h[dynamic_simulation_B.nr] -
            dynamic_simulation_B.htmp1;
          dynamic_simulation_B.htmp2 = (fabs(dynamic_simulation_B.h12) + fabs
            (dynamic_simulation_B.aa)) + fabs(h[dynamic_simulation_B.s_tmp]);
          dynamic_simulation_B.h21s = h[dynamic_simulation_B.s_tmp] /
            dynamic_simulation_B.htmp2;
          dynamic_simulation_B.hoffset = (dynamic_simulation_B.m_o << 2) +
            dynamic_simulation_B.m_o;
          dynamic_simulation_B.v_j[0] = (dynamic_simulation_B.h12 /
            dynamic_simulation_B.htmp2 * (h[dynamic_simulation_B.nr] -
            dynamic_simulation_B.ba) + h[dynamic_simulation_B.hoffset - 1] *
            dynamic_simulation_B.h21s) - dynamic_simulation_B.aa /
            dynamic_simulation_B.htmp2 * dynamic_simulation_B.tst;
          dynamic_simulation_B.v_j[1] = (((h[dynamic_simulation_B.nr] +
            h[dynamic_simulation_B.hoffset]) - dynamic_simulation_B.ba) -
            dynamic_simulation_B.htmp1) * dynamic_simulation_B.h21s;
          dynamic_simulation_B.v_j[2] = h[dynamic_simulation_B.hoffset + 1] *
            dynamic_simulation_B.h21s;
          dynamic_simulation_B.htmp2 = (fabs(dynamic_simulation_B.v_j[0]) + fabs
            (dynamic_simulation_B.v_j[1])) + fabs(dynamic_simulation_B.v_j[2]);
          dynamic_simulation_B.v_j[0] /= dynamic_simulation_B.htmp2;
          dynamic_simulation_B.v_j[1] /= dynamic_simulation_B.htmp2;
          dynamic_simulation_B.v_j[2] /= dynamic_simulation_B.htmp2;
          if (dynamic_simulation_B.k_n + 1 == dynamic_simulation_B.m_o) {
            exitg3 = true;
          } else {
            dynamic_simulation_B.s_tmp = ((dynamic_simulation_B.m_o - 2) << 2) +
              dynamic_simulation_B.m_o;
            if (fabs(h[dynamic_simulation_B.s_tmp - 1]) * (fabs
                 (dynamic_simulation_B.v_j[1]) + fabs(dynamic_simulation_B.v_j[2]))
                <= ((fabs(h[dynamic_simulation_B.s_tmp - 2]) + fabs
                     (h[dynamic_simulation_B.nr])) + fabs
                    (h[dynamic_simulation_B.hoffset])) * (2.2204460492503131E-16
                 * fabs(dynamic_simulation_B.v_j[0]))) {
              exitg3 = true;
            } else {
              dynamic_simulation_B.m_o--;
            }
          }
        }

        dynamic_simulation_B.s_tmp = dynamic_simulation_B.m_o;
        while (dynamic_simulation_B.s_tmp <= dynamic_simulation_B.i_k) {
          dynamic_simulation_B.nr = (dynamic_simulation_B.i_k -
            dynamic_simulation_B.s_tmp) + 2;
          if (3 < dynamic_simulation_B.nr) {
            dynamic_simulation_B.nr = 3;
          }

          if (dynamic_simulation_B.s_tmp > dynamic_simulation_B.m_o) {
            dynamic_simulation_B.hoffset = ((dynamic_simulation_B.s_tmp - 2) <<
              2) + dynamic_simulation_B.s_tmp;
            dynamic_simulation_B.j_g = 0;
            while (dynamic_simulation_B.j_g <= dynamic_simulation_B.nr - 1) {
              dynamic_simulation_B.v_j[dynamic_simulation_B.j_g] = h
                [(dynamic_simulation_B.j_g + dynamic_simulation_B.hoffset) - 1];
              dynamic_simulation_B.j_g++;
            }
          }

          dynamic_simulation_B.tst = dynamic_simulation_B.v_j[0];
          dynamic_simulation_B.b_v[0] = dynamic_simulation_B.v_j[0];
          dynamic_simulation_B.b_v[1] = dynamic_simulation_B.v_j[1];
          dynamic_simulation_B.b_v[2] = dynamic_simulation_B.v_j[2];
          dynamic_simulation_B.htmp2 = dynamic_simulation_xzlarfg
            (dynamic_simulation_B.nr, &dynamic_simulation_B.tst,
             dynamic_simulation_B.b_v);
          dynamic_simulation_B.v_j[1] = dynamic_simulation_B.b_v[1];
          dynamic_simulation_B.v_j[2] = dynamic_simulation_B.b_v[2];
          dynamic_simulation_B.v_j[0] = dynamic_simulation_B.tst;
          if (dynamic_simulation_B.s_tmp > dynamic_simulation_B.m_o) {
            h[(dynamic_simulation_B.s_tmp + ((dynamic_simulation_B.s_tmp - 2) <<
                2)) - 1] = dynamic_simulation_B.tst;
            h[dynamic_simulation_B.s_tmp + ((dynamic_simulation_B.s_tmp - 2) <<
              2)] = 0.0;
            if (dynamic_simulation_B.s_tmp < dynamic_simulation_B.i_k) {
              h[dynamic_simulation_B.s_tmp + 1] = 0.0;
            }
          } else {
            if (dynamic_simulation_B.m_o > dynamic_simulation_B.k_n + 1) {
              h[dynamic_simulation_B.s_tmp - 1] *= 1.0 -
                dynamic_simulation_B.htmp2;
            }
          }

          dynamic_simulation_B.tst = dynamic_simulation_B.b_v[1];
          dynamic_simulation_B.ba = dynamic_simulation_B.htmp2 *
            dynamic_simulation_B.b_v[1];
          switch (dynamic_simulation_B.nr) {
           case 3:
            dynamic_simulation_B.htmp1 = dynamic_simulation_B.b_v[2];
            dynamic_simulation_B.aa = dynamic_simulation_B.htmp2 *
              dynamic_simulation_B.b_v[2];
            dynamic_simulation_B.b_j_c = dynamic_simulation_B.s_tmp - 1;
            while (dynamic_simulation_B.b_j_c + 1 < 5) {
              dynamic_simulation_B.nr = (dynamic_simulation_B.b_j_c << 2) +
                dynamic_simulation_B.s_tmp;
              dynamic_simulation_B.hoffset = dynamic_simulation_B.nr - 1;
              dynamic_simulation_B.j_g = dynamic_simulation_B.nr + 1;
              dynamic_simulation_B.h12 = (h[dynamic_simulation_B.hoffset] +
                h[dynamic_simulation_B.nr] * dynamic_simulation_B.tst) +
                h[dynamic_simulation_B.j_g] * dynamic_simulation_B.htmp1;
              h[dynamic_simulation_B.hoffset] -= dynamic_simulation_B.h12 *
                dynamic_simulation_B.htmp2;
              h[dynamic_simulation_B.nr] -= dynamic_simulation_B.h12 *
                dynamic_simulation_B.ba;
              h[dynamic_simulation_B.j_g] -= dynamic_simulation_B.h12 *
                dynamic_simulation_B.aa;
              dynamic_simulation_B.b_j_c++;
            }

            dynamic_simulation_B.nr = dynamic_simulation_B.s_tmp + 3;
            dynamic_simulation_B.b_j_c = dynamic_simulation_B.i_k + 1;
            if (dynamic_simulation_B.nr < dynamic_simulation_B.b_j_c) {
              dynamic_simulation_B.b_j_c = dynamic_simulation_B.nr;
            }

            dynamic_simulation_B.c_j = 0;
            while (dynamic_simulation_B.c_j <= dynamic_simulation_B.b_j_c - 1) {
              dynamic_simulation_B.nr = ((dynamic_simulation_B.s_tmp - 1) << 2)
                + dynamic_simulation_B.c_j;
              dynamic_simulation_B.hoffset = (dynamic_simulation_B.s_tmp << 2) +
                dynamic_simulation_B.c_j;
              dynamic_simulation_B.j_g = ((dynamic_simulation_B.s_tmp + 1) << 2)
                + dynamic_simulation_B.c_j;
              dynamic_simulation_B.h12 = (h[dynamic_simulation_B.nr] +
                h[dynamic_simulation_B.hoffset] * dynamic_simulation_B.tst) +
                h[dynamic_simulation_B.j_g] * dynamic_simulation_B.htmp1;
              h[dynamic_simulation_B.nr] -= dynamic_simulation_B.h12 *
                dynamic_simulation_B.htmp2;
              h[dynamic_simulation_B.hoffset] -= dynamic_simulation_B.h12 *
                dynamic_simulation_B.ba;
              h[dynamic_simulation_B.j_g] -= dynamic_simulation_B.h12 *
                dynamic_simulation_B.aa;
              dynamic_simulation_B.c_j++;
            }

            for (dynamic_simulation_B.b_j_c = 0; dynamic_simulation_B.b_j_c < 4;
                 dynamic_simulation_B.b_j_c++) {
              dynamic_simulation_B.nr = ((dynamic_simulation_B.s_tmp - 1) << 2)
                + dynamic_simulation_B.b_j_c;
              dynamic_simulation_B.hoffset = (dynamic_simulation_B.s_tmp << 2) +
                dynamic_simulation_B.b_j_c;
              dynamic_simulation_B.j_g = ((dynamic_simulation_B.s_tmp + 1) << 2)
                + dynamic_simulation_B.b_j_c;
              dynamic_simulation_B.h12 = (z[dynamic_simulation_B.nr] +
                z[dynamic_simulation_B.hoffset] * dynamic_simulation_B.tst) +
                z[dynamic_simulation_B.j_g] * dynamic_simulation_B.htmp1;
              z[dynamic_simulation_B.nr] -= dynamic_simulation_B.h12 *
                dynamic_simulation_B.htmp2;
              z[dynamic_simulation_B.hoffset] -= dynamic_simulation_B.h12 *
                dynamic_simulation_B.ba;
              z[dynamic_simulation_B.j_g] -= dynamic_simulation_B.h12 *
                dynamic_simulation_B.aa;
            }
            break;

           case 2:
            dynamic_simulation_B.j_g = dynamic_simulation_B.s_tmp - 1;
            while (dynamic_simulation_B.j_g + 1 < 5) {
              dynamic_simulation_B.nr = (dynamic_simulation_B.j_g << 2) +
                dynamic_simulation_B.s_tmp;
              dynamic_simulation_B.hoffset = dynamic_simulation_B.nr - 1;
              dynamic_simulation_B.h12 = h[dynamic_simulation_B.hoffset] +
                h[dynamic_simulation_B.nr] * dynamic_simulation_B.tst;
              h[dynamic_simulation_B.hoffset] -= dynamic_simulation_B.h12 *
                dynamic_simulation_B.htmp2;
              h[dynamic_simulation_B.nr] -= dynamic_simulation_B.h12 *
                dynamic_simulation_B.ba;
              dynamic_simulation_B.j_g++;
            }

            dynamic_simulation_B.j_g = 0;
            while (dynamic_simulation_B.j_g <= dynamic_simulation_B.i_k) {
              dynamic_simulation_B.nr = ((dynamic_simulation_B.s_tmp - 1) << 2)
                + dynamic_simulation_B.j_g;
              dynamic_simulation_B.hoffset = (dynamic_simulation_B.s_tmp << 2) +
                dynamic_simulation_B.j_g;
              dynamic_simulation_B.h12 = h[dynamic_simulation_B.nr] +
                h[dynamic_simulation_B.hoffset] * dynamic_simulation_B.tst;
              h[dynamic_simulation_B.nr] -= dynamic_simulation_B.h12 *
                dynamic_simulation_B.htmp2;
              h[dynamic_simulation_B.hoffset] -= dynamic_simulation_B.h12 *
                dynamic_simulation_B.ba;
              dynamic_simulation_B.j_g++;
            }

            for (dynamic_simulation_B.j_g = 0; dynamic_simulation_B.j_g < 4;
                 dynamic_simulation_B.j_g++) {
              dynamic_simulation_B.nr = ((dynamic_simulation_B.s_tmp - 1) << 2)
                + dynamic_simulation_B.j_g;
              dynamic_simulation_B.hoffset = (dynamic_simulation_B.s_tmp << 2) +
                dynamic_simulation_B.j_g;
              dynamic_simulation_B.h12 = z[dynamic_simulation_B.nr] +
                z[dynamic_simulation_B.hoffset] * dynamic_simulation_B.tst;
              z[dynamic_simulation_B.nr] -= dynamic_simulation_B.h12 *
                dynamic_simulation_B.htmp2;
              z[dynamic_simulation_B.hoffset] -= dynamic_simulation_B.h12 *
                dynamic_simulation_B.ba;
            }
            break;
          }

          dynamic_simulation_B.s_tmp++;
        }

        dynamic_simulation_B.ix++;
      }
    }

    if (!dynamic_simulation_B.goto150) {
      info = dynamic_simulation_B.i_k + 1;
      exitg1 = true;
    } else {
      if ((dynamic_simulation_B.i_k + 1 != dynamic_simulation_B.L) &&
          (dynamic_simulation_B.L == dynamic_simulation_B.i_k)) {
        dynamic_simulation_B.ix = (dynamic_simulation_B.i_k - 1) << 2;
        dynamic_simulation_B.k_n = dynamic_simulation_B.ix +
          dynamic_simulation_B.i_k;
        dynamic_simulation_B.m_o = dynamic_simulation_B.k_n - 1;
        dynamic_simulation_B.ba = h[dynamic_simulation_B.m_o];
        dynamic_simulation_B.s_tmp = dynamic_simulation_B.i_k << 2;
        dynamic_simulation_B.nr = dynamic_simulation_B.s_tmp +
          dynamic_simulation_B.i_k;
        dynamic_simulation_B.hoffset = dynamic_simulation_B.nr - 1;
        dynamic_simulation_B.htmp1 = h[dynamic_simulation_B.hoffset];
        dynamic_simulation_B.aa = h[dynamic_simulation_B.k_n];
        dynamic_simulation_B.h12 = h[dynamic_simulation_B.nr];
        dynamic_simulation_xdlanv2(&dynamic_simulation_B.ba,
          &dynamic_simulation_B.htmp1, &dynamic_simulation_B.aa,
          &dynamic_simulation_B.h12, &dynamic_simulation_B.h21s,
          &dynamic_simulation_B.unusedU1, &dynamic_simulation_B.unusedU2,
          &dynamic_simulation_B.unusedU3, &dynamic_simulation_B.htmp2,
          &dynamic_simulation_B.tst);
        h[dynamic_simulation_B.m_o] = dynamic_simulation_B.ba;
        h[dynamic_simulation_B.hoffset] = dynamic_simulation_B.htmp1;
        h[dynamic_simulation_B.k_n] = dynamic_simulation_B.aa;
        h[dynamic_simulation_B.nr] = dynamic_simulation_B.h12;
        if (4 > dynamic_simulation_B.i_k + 1) {
          dynamic_simulation_xrot(3 - dynamic_simulation_B.i_k, h,
            dynamic_simulation_B.i_k + ((dynamic_simulation_B.i_k + 1) << 2),
            (dynamic_simulation_B.i_k + ((dynamic_simulation_B.i_k + 1) << 2)) +
            1, dynamic_simulation_B.htmp2, dynamic_simulation_B.tst);
        }

        dynamic_simulation_xrot_l(dynamic_simulation_B.i_k - 1, h,
          ((dynamic_simulation_B.i_k - 1) << 2) + 1, (dynamic_simulation_B.i_k <<
          2) + 1, dynamic_simulation_B.htmp2, dynamic_simulation_B.tst);
        dynamic_simulation_B.ba = dynamic_simulation_B.htmp2 *
          z[dynamic_simulation_B.ix] + dynamic_simulation_B.tst *
          z[dynamic_simulation_B.s_tmp];
        z[dynamic_simulation_B.s_tmp] = dynamic_simulation_B.htmp2 *
          z[dynamic_simulation_B.s_tmp] - dynamic_simulation_B.tst *
          z[dynamic_simulation_B.ix];
        z[dynamic_simulation_B.ix] = dynamic_simulation_B.ba;
        dynamic_simulation_B.i_k = dynamic_simulation_B.s_tmp + 1;
        dynamic_simulation_B.ix++;
        dynamic_simulation_B.ba = dynamic_simulation_B.htmp2 *
          z[dynamic_simulation_B.ix] + dynamic_simulation_B.tst *
          z[dynamic_simulation_B.i_k];
        z[dynamic_simulation_B.i_k] = dynamic_simulation_B.htmp2 *
          z[dynamic_simulation_B.i_k] - dynamic_simulation_B.tst *
          z[dynamic_simulation_B.ix];
        z[dynamic_simulation_B.ix] = dynamic_simulation_B.ba;
        dynamic_simulation_B.i_k++;
        dynamic_simulation_B.ix++;
        dynamic_simulation_B.ba = dynamic_simulation_B.htmp2 *
          z[dynamic_simulation_B.ix] + dynamic_simulation_B.tst *
          z[dynamic_simulation_B.i_k];
        z[dynamic_simulation_B.i_k] = dynamic_simulation_B.htmp2 *
          z[dynamic_simulation_B.i_k] - dynamic_simulation_B.tst *
          z[dynamic_simulation_B.ix];
        z[dynamic_simulation_B.ix] = dynamic_simulation_B.ba;
        dynamic_simulation_B.i_k++;
        dynamic_simulation_B.ix++;
        dynamic_simulation_B.ba = dynamic_simulation_B.htmp2 *
          z[dynamic_simulation_B.ix] + dynamic_simulation_B.tst *
          z[dynamic_simulation_B.i_k];
        z[dynamic_simulation_B.i_k] = dynamic_simulation_B.htmp2 *
          z[dynamic_simulation_B.i_k] - dynamic_simulation_B.tst *
          z[dynamic_simulation_B.ix];
        z[dynamic_simulation_B.ix] = dynamic_simulation_B.ba;
      }

      dynamic_simulation_B.i_k = dynamic_simulation_B.L - 2;
    }
  }

  return info;
}

// Function for MATLAB Function: '<S2>/MATLAB Function'
static void dynamic_simulation_eig(const real_T A[16], creal_T V[16], creal_T D
  [4])
{
  int32_T exitg1;
  boolean_T exitg2;
  if (dynamic_simulation_anyNonFinite(A)) {
    for (dynamic_simulation_B.b_j = 0; dynamic_simulation_B.b_j < 16;
         dynamic_simulation_B.b_j++) {
      V[dynamic_simulation_B.b_j].re = (rtNaN);
      V[dynamic_simulation_B.b_j].im = 0.0;
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
    dynamic_simulation_B.p_n = true;
    dynamic_simulation_B.b_j = 0;
    exitg2 = false;
    while ((!exitg2) && (dynamic_simulation_B.b_j < 4)) {
      dynamic_simulation_B.i_e = 0;
      do {
        exitg1 = 0;
        if (dynamic_simulation_B.i_e <= dynamic_simulation_B.b_j) {
          if (!(A[(dynamic_simulation_B.b_j << 2) + dynamic_simulation_B.i_e] ==
                A[(dynamic_simulation_B.i_e << 2) + dynamic_simulation_B.b_j]))
          {
            dynamic_simulation_B.p_n = false;
            exitg1 = 1;
          } else {
            dynamic_simulation_B.i_e++;
          }
        } else {
          dynamic_simulation_B.b_j++;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }

    if (dynamic_simulation_B.p_n) {
      if (dynamic_simulation_anyNonFinite(A)) {
        for (dynamic_simulation_B.b_j = 0; dynamic_simulation_B.b_j < 16;
             dynamic_simulation_B.b_j++) {
          dynamic_simulation_B.b_V[dynamic_simulation_B.b_j] = (rtNaN);
        }

        dynamic_simulation_B.b_j = 2;
        while (dynamic_simulation_B.b_j < 5) {
          dynamic_simulation_B.b_V[dynamic_simulation_B.b_j - 1] = 0.0;
          dynamic_simulation_B.b_j++;
        }

        dynamic_simulation_B.b_j = 3;
        while (dynamic_simulation_B.b_j < 5) {
          dynamic_simulation_B.b_V[dynamic_simulation_B.b_j + 3] = 0.0;
          dynamic_simulation_B.b_j++;
        }

        dynamic_simulation_B.b_V[11] = 0.0;
        for (dynamic_simulation_B.b_j = 0; dynamic_simulation_B.b_j < 16;
             dynamic_simulation_B.b_j++) {
          dynamic_simulation_B.b_A[dynamic_simulation_B.b_j] = (rtNaN);
        }
      } else {
        memcpy(&dynamic_simulation_B.b_A[0], &A[0], sizeof(real_T) << 4U);
        dynamic_simulation_xgehrd(dynamic_simulation_B.b_A,
          dynamic_simulation_B.tau);
        memcpy(&dynamic_simulation_B.b_V[0], &dynamic_simulation_B.b_A[0],
               sizeof(real_T) << 4U);
        dynamic_simulation_B.b_j = 0;
        while (dynamic_simulation_B.b_j <= 2) {
          dynamic_simulation_B.b_V[dynamic_simulation_B.b_j + 12] = 0.0;
          dynamic_simulation_B.b_j++;
        }

        dynamic_simulation_B.b_j = 0;
        while (dynamic_simulation_B.b_j <= 1) {
          dynamic_simulation_B.b_V[dynamic_simulation_B.b_j + 8] = 0.0;
          dynamic_simulation_B.b_j++;
        }

        dynamic_simulation_B.b_j = 1;
        while (dynamic_simulation_B.b_j + 3 < 5) {
          dynamic_simulation_B.b_V[dynamic_simulation_B.b_j + 10] =
            dynamic_simulation_B.b_V[dynamic_simulation_B.b_j + 6];
          dynamic_simulation_B.b_j++;
        }

        dynamic_simulation_B.b_V[4] = 0.0;
        dynamic_simulation_B.b_j = 0;
        while (dynamic_simulation_B.b_j + 3 < 5) {
          dynamic_simulation_B.b_V[dynamic_simulation_B.b_j + 6] =
            dynamic_simulation_B.b_V[dynamic_simulation_B.b_j + 2];
          dynamic_simulation_B.b_j++;
        }

        dynamic_simulation_B.work[0] = 0.0;
        dynamic_simulation_B.b_V[1] = 0.0;
        dynamic_simulation_B.work[1] = 0.0;
        dynamic_simulation_B.b_V[2] = 0.0;
        dynamic_simulation_B.work[2] = 0.0;
        dynamic_simulation_B.b_V[3] = 0.0;
        dynamic_simulation_B.work[3] = 0.0;
        dynamic_simulation_B.b_V[0] = 1.0;
        dynamic_simulation_B.b_V[15] = 1.0 - dynamic_simulation_B.tau[2];
        dynamic_simulation_B.b_j = 0;
        while (dynamic_simulation_B.b_j <= 1) {
          dynamic_simulation_B.b_V[14 - dynamic_simulation_B.b_j] = 0.0;
          dynamic_simulation_B.b_j++;
        }

        dynamic_simulation_B.b_V[10] = 1.0;
        dynamic_simulation_xzlarf(2, 1, 11, dynamic_simulation_B.tau[1],
          dynamic_simulation_B.b_V, 15, dynamic_simulation_B.work);
        dynamic_simulation_B.b_j = 11;
        while (dynamic_simulation_B.b_j + 1 <= 12) {
          dynamic_simulation_B.b_V[dynamic_simulation_B.b_j] *=
            -dynamic_simulation_B.tau[1];
          dynamic_simulation_B.b_j++;
        }

        dynamic_simulation_B.b_V[10] = 1.0 - dynamic_simulation_B.tau[1];
        dynamic_simulation_B.b_V[9] = 0.0;
        dynamic_simulation_B.b_V[5] = 1.0;
        dynamic_simulation_xzlarf(3, 2, 6, dynamic_simulation_B.tau[0],
          dynamic_simulation_B.b_V, 10, dynamic_simulation_B.work);
        dynamic_simulation_B.b_j = 6;
        while (dynamic_simulation_B.b_j + 1 <= 8) {
          dynamic_simulation_B.b_V[dynamic_simulation_B.b_j] *=
            -dynamic_simulation_B.tau[0];
          dynamic_simulation_B.b_j++;
        }

        dynamic_simulation_B.b_V[5] = 1.0 - dynamic_simulation_B.tau[0];
        dynamic_simulation_eml_dlahqr(dynamic_simulation_B.b_A,
          dynamic_simulation_B.b_V);
      }

      for (dynamic_simulation_B.b_j = 0; dynamic_simulation_B.b_j < 16;
           dynamic_simulation_B.b_j++) {
        V[dynamic_simulation_B.b_j].re =
          dynamic_simulation_B.b_V[dynamic_simulation_B.b_j];
        V[dynamic_simulation_B.b_j].im = 0.0;
      }

      D[0].re = dynamic_simulation_B.b_A[0];
      D[0].im = 0.0;
      D[1].re = dynamic_simulation_B.b_A[5];
      D[1].im = 0.0;
      D[2].re = dynamic_simulation_B.b_A[10];
      D[2].im = 0.0;
      D[3].re = dynamic_simulation_B.b_A[15];
      D[3].im = 0.0;
    } else {
      for (dynamic_simulation_B.b_j = 0; dynamic_simulation_B.b_j < 16;
           dynamic_simulation_B.b_j++) {
        dynamic_simulation_B.At[dynamic_simulation_B.b_j].re =
          A[dynamic_simulation_B.b_j];
        dynamic_simulation_B.At[dynamic_simulation_B.b_j].im = 0.0;
      }

      dynamic_simulation_xzggev(dynamic_simulation_B.At,
        &dynamic_simulation_B.b_j, D, dynamic_simulation_B.beta1, V);
      dynamic_simulation_B.colnorm = 0.0;
      dynamic_simulation_B.scale = 3.3121686421112381E-170;
      dynamic_simulation_B.b_j = 0;
      while (dynamic_simulation_B.b_j + 1 <= 4) {
        dynamic_simulation_B.absxk = fabs(V[dynamic_simulation_B.b_j].re);
        if (dynamic_simulation_B.absxk > dynamic_simulation_B.scale) {
          dynamic_simulation_B.t = dynamic_simulation_B.scale /
            dynamic_simulation_B.absxk;
          dynamic_simulation_B.colnorm = dynamic_simulation_B.colnorm *
            dynamic_simulation_B.t * dynamic_simulation_B.t + 1.0;
          dynamic_simulation_B.scale = dynamic_simulation_B.absxk;
        } else {
          dynamic_simulation_B.t = dynamic_simulation_B.absxk /
            dynamic_simulation_B.scale;
          dynamic_simulation_B.colnorm += dynamic_simulation_B.t *
            dynamic_simulation_B.t;
        }

        dynamic_simulation_B.absxk = fabs(V[dynamic_simulation_B.b_j].im);
        if (dynamic_simulation_B.absxk > dynamic_simulation_B.scale) {
          dynamic_simulation_B.t = dynamic_simulation_B.scale /
            dynamic_simulation_B.absxk;
          dynamic_simulation_B.colnorm = dynamic_simulation_B.colnorm *
            dynamic_simulation_B.t * dynamic_simulation_B.t + 1.0;
          dynamic_simulation_B.scale = dynamic_simulation_B.absxk;
        } else {
          dynamic_simulation_B.t = dynamic_simulation_B.absxk /
            dynamic_simulation_B.scale;
          dynamic_simulation_B.colnorm += dynamic_simulation_B.t *
            dynamic_simulation_B.t;
        }

        dynamic_simulation_B.b_j++;
      }

      dynamic_simulation_B.colnorm = dynamic_simulation_B.scale * sqrt
        (dynamic_simulation_B.colnorm);
      dynamic_simulation_B.b_j = 0;
      while (dynamic_simulation_B.b_j + 1 <= 4) {
        if (V[dynamic_simulation_B.b_j].im == 0.0) {
          dynamic_simulation_B.scale = V[dynamic_simulation_B.b_j].re /
            dynamic_simulation_B.colnorm;
          dynamic_simulation_B.absxk = 0.0;
        } else if (V[dynamic_simulation_B.b_j].re == 0.0) {
          dynamic_simulation_B.scale = 0.0;
          dynamic_simulation_B.absxk = V[dynamic_simulation_B.b_j].im /
            dynamic_simulation_B.colnorm;
        } else {
          dynamic_simulation_B.scale = V[dynamic_simulation_B.b_j].re /
            dynamic_simulation_B.colnorm;
          dynamic_simulation_B.absxk = V[dynamic_simulation_B.b_j].im /
            dynamic_simulation_B.colnorm;
        }

        V[dynamic_simulation_B.b_j].re = dynamic_simulation_B.scale;
        V[dynamic_simulation_B.b_j].im = dynamic_simulation_B.absxk;
        dynamic_simulation_B.b_j++;
      }

      dynamic_simulation_B.colnorm = 0.0;
      dynamic_simulation_B.scale = 3.3121686421112381E-170;
      dynamic_simulation_B.b_j = 4;
      while (dynamic_simulation_B.b_j + 1 <= 8) {
        dynamic_simulation_B.absxk = fabs(V[dynamic_simulation_B.b_j].re);
        if (dynamic_simulation_B.absxk > dynamic_simulation_B.scale) {
          dynamic_simulation_B.t = dynamic_simulation_B.scale /
            dynamic_simulation_B.absxk;
          dynamic_simulation_B.colnorm = dynamic_simulation_B.colnorm *
            dynamic_simulation_B.t * dynamic_simulation_B.t + 1.0;
          dynamic_simulation_B.scale = dynamic_simulation_B.absxk;
        } else {
          dynamic_simulation_B.t = dynamic_simulation_B.absxk /
            dynamic_simulation_B.scale;
          dynamic_simulation_B.colnorm += dynamic_simulation_B.t *
            dynamic_simulation_B.t;
        }

        dynamic_simulation_B.absxk = fabs(V[dynamic_simulation_B.b_j].im);
        if (dynamic_simulation_B.absxk > dynamic_simulation_B.scale) {
          dynamic_simulation_B.t = dynamic_simulation_B.scale /
            dynamic_simulation_B.absxk;
          dynamic_simulation_B.colnorm = dynamic_simulation_B.colnorm *
            dynamic_simulation_B.t * dynamic_simulation_B.t + 1.0;
          dynamic_simulation_B.scale = dynamic_simulation_B.absxk;
        } else {
          dynamic_simulation_B.t = dynamic_simulation_B.absxk /
            dynamic_simulation_B.scale;
          dynamic_simulation_B.colnorm += dynamic_simulation_B.t *
            dynamic_simulation_B.t;
        }

        dynamic_simulation_B.b_j++;
      }

      dynamic_simulation_B.colnorm = dynamic_simulation_B.scale * sqrt
        (dynamic_simulation_B.colnorm);
      dynamic_simulation_B.b_j = 4;
      while (dynamic_simulation_B.b_j + 1 <= 8) {
        if (V[dynamic_simulation_B.b_j].im == 0.0) {
          dynamic_simulation_B.scale = V[dynamic_simulation_B.b_j].re /
            dynamic_simulation_B.colnorm;
          dynamic_simulation_B.absxk = 0.0;
        } else if (V[dynamic_simulation_B.b_j].re == 0.0) {
          dynamic_simulation_B.scale = 0.0;
          dynamic_simulation_B.absxk = V[dynamic_simulation_B.b_j].im /
            dynamic_simulation_B.colnorm;
        } else {
          dynamic_simulation_B.scale = V[dynamic_simulation_B.b_j].re /
            dynamic_simulation_B.colnorm;
          dynamic_simulation_B.absxk = V[dynamic_simulation_B.b_j].im /
            dynamic_simulation_B.colnorm;
        }

        V[dynamic_simulation_B.b_j].re = dynamic_simulation_B.scale;
        V[dynamic_simulation_B.b_j].im = dynamic_simulation_B.absxk;
        dynamic_simulation_B.b_j++;
      }

      dynamic_simulation_B.colnorm = 0.0;
      dynamic_simulation_B.scale = 3.3121686421112381E-170;
      dynamic_simulation_B.b_j = 8;
      while (dynamic_simulation_B.b_j + 1 <= 12) {
        dynamic_simulation_B.absxk = fabs(V[dynamic_simulation_B.b_j].re);
        if (dynamic_simulation_B.absxk > dynamic_simulation_B.scale) {
          dynamic_simulation_B.t = dynamic_simulation_B.scale /
            dynamic_simulation_B.absxk;
          dynamic_simulation_B.colnorm = dynamic_simulation_B.colnorm *
            dynamic_simulation_B.t * dynamic_simulation_B.t + 1.0;
          dynamic_simulation_B.scale = dynamic_simulation_B.absxk;
        } else {
          dynamic_simulation_B.t = dynamic_simulation_B.absxk /
            dynamic_simulation_B.scale;
          dynamic_simulation_B.colnorm += dynamic_simulation_B.t *
            dynamic_simulation_B.t;
        }

        dynamic_simulation_B.absxk = fabs(V[dynamic_simulation_B.b_j].im);
        if (dynamic_simulation_B.absxk > dynamic_simulation_B.scale) {
          dynamic_simulation_B.t = dynamic_simulation_B.scale /
            dynamic_simulation_B.absxk;
          dynamic_simulation_B.colnorm = dynamic_simulation_B.colnorm *
            dynamic_simulation_B.t * dynamic_simulation_B.t + 1.0;
          dynamic_simulation_B.scale = dynamic_simulation_B.absxk;
        } else {
          dynamic_simulation_B.t = dynamic_simulation_B.absxk /
            dynamic_simulation_B.scale;
          dynamic_simulation_B.colnorm += dynamic_simulation_B.t *
            dynamic_simulation_B.t;
        }

        dynamic_simulation_B.b_j++;
      }

      dynamic_simulation_B.colnorm = dynamic_simulation_B.scale * sqrt
        (dynamic_simulation_B.colnorm);
      dynamic_simulation_B.b_j = 8;
      while (dynamic_simulation_B.b_j + 1 <= 12) {
        if (V[dynamic_simulation_B.b_j].im == 0.0) {
          dynamic_simulation_B.scale = V[dynamic_simulation_B.b_j].re /
            dynamic_simulation_B.colnorm;
          dynamic_simulation_B.absxk = 0.0;
        } else if (V[dynamic_simulation_B.b_j].re == 0.0) {
          dynamic_simulation_B.scale = 0.0;
          dynamic_simulation_B.absxk = V[dynamic_simulation_B.b_j].im /
            dynamic_simulation_B.colnorm;
        } else {
          dynamic_simulation_B.scale = V[dynamic_simulation_B.b_j].re /
            dynamic_simulation_B.colnorm;
          dynamic_simulation_B.absxk = V[dynamic_simulation_B.b_j].im /
            dynamic_simulation_B.colnorm;
        }

        V[dynamic_simulation_B.b_j].re = dynamic_simulation_B.scale;
        V[dynamic_simulation_B.b_j].im = dynamic_simulation_B.absxk;
        dynamic_simulation_B.b_j++;
      }

      dynamic_simulation_B.colnorm = 0.0;
      dynamic_simulation_B.scale = 3.3121686421112381E-170;
      dynamic_simulation_B.b_j = 12;
      while (dynamic_simulation_B.b_j + 1 <= 16) {
        dynamic_simulation_B.absxk = fabs(V[dynamic_simulation_B.b_j].re);
        if (dynamic_simulation_B.absxk > dynamic_simulation_B.scale) {
          dynamic_simulation_B.t = dynamic_simulation_B.scale /
            dynamic_simulation_B.absxk;
          dynamic_simulation_B.colnorm = dynamic_simulation_B.colnorm *
            dynamic_simulation_B.t * dynamic_simulation_B.t + 1.0;
          dynamic_simulation_B.scale = dynamic_simulation_B.absxk;
        } else {
          dynamic_simulation_B.t = dynamic_simulation_B.absxk /
            dynamic_simulation_B.scale;
          dynamic_simulation_B.colnorm += dynamic_simulation_B.t *
            dynamic_simulation_B.t;
        }

        dynamic_simulation_B.absxk = fabs(V[dynamic_simulation_B.b_j].im);
        if (dynamic_simulation_B.absxk > dynamic_simulation_B.scale) {
          dynamic_simulation_B.t = dynamic_simulation_B.scale /
            dynamic_simulation_B.absxk;
          dynamic_simulation_B.colnorm = dynamic_simulation_B.colnorm *
            dynamic_simulation_B.t * dynamic_simulation_B.t + 1.0;
          dynamic_simulation_B.scale = dynamic_simulation_B.absxk;
        } else {
          dynamic_simulation_B.t = dynamic_simulation_B.absxk /
            dynamic_simulation_B.scale;
          dynamic_simulation_B.colnorm += dynamic_simulation_B.t *
            dynamic_simulation_B.t;
        }

        dynamic_simulation_B.b_j++;
      }

      dynamic_simulation_B.colnorm = dynamic_simulation_B.scale * sqrt
        (dynamic_simulation_B.colnorm);
      dynamic_simulation_B.b_j = 12;
      while (dynamic_simulation_B.b_j + 1 <= 16) {
        if (V[dynamic_simulation_B.b_j].im == 0.0) {
          dynamic_simulation_B.scale = V[dynamic_simulation_B.b_j].re /
            dynamic_simulation_B.colnorm;
          dynamic_simulation_B.absxk = 0.0;
        } else if (V[dynamic_simulation_B.b_j].re == 0.0) {
          dynamic_simulation_B.scale = 0.0;
          dynamic_simulation_B.absxk = V[dynamic_simulation_B.b_j].im /
            dynamic_simulation_B.colnorm;
        } else {
          dynamic_simulation_B.scale = V[dynamic_simulation_B.b_j].re /
            dynamic_simulation_B.colnorm;
          dynamic_simulation_B.absxk = V[dynamic_simulation_B.b_j].im /
            dynamic_simulation_B.colnorm;
        }

        V[dynamic_simulation_B.b_j].re = dynamic_simulation_B.scale;
        V[dynamic_simulation_B.b_j].im = dynamic_simulation_B.absxk;
        dynamic_simulation_B.b_j++;
      }

      if (dynamic_simulation_B.beta1[0].im == 0.0) {
        if (D[0].im == 0.0) {
          dynamic_simulation_B.scale = D[0].re / dynamic_simulation_B.beta1[0].
            re;
          dynamic_simulation_B.absxk = 0.0;
        } else if (D[0].re == 0.0) {
          dynamic_simulation_B.scale = 0.0;
          dynamic_simulation_B.absxk = D[0].im / dynamic_simulation_B.beta1[0].
            re;
        } else {
          dynamic_simulation_B.scale = D[0].re / dynamic_simulation_B.beta1[0].
            re;
          dynamic_simulation_B.absxk = D[0].im / dynamic_simulation_B.beta1[0].
            re;
        }
      } else if (dynamic_simulation_B.beta1[0].re == 0.0) {
        if (D[0].re == 0.0) {
          dynamic_simulation_B.scale = D[0].im / dynamic_simulation_B.beta1[0].
            im;
          dynamic_simulation_B.absxk = 0.0;
        } else if (D[0].im == 0.0) {
          dynamic_simulation_B.scale = 0.0;
          dynamic_simulation_B.absxk = -(D[0].re / dynamic_simulation_B.beta1[0]
            .im);
        } else {
          dynamic_simulation_B.scale = D[0].im / dynamic_simulation_B.beta1[0].
            im;
          dynamic_simulation_B.absxk = -(D[0].re / dynamic_simulation_B.beta1[0]
            .im);
        }
      } else {
        dynamic_simulation_B.colnorm = fabs(dynamic_simulation_B.beta1[0].re);
        dynamic_simulation_B.scale = fabs(dynamic_simulation_B.beta1[0].im);
        if (dynamic_simulation_B.colnorm > dynamic_simulation_B.scale) {
          dynamic_simulation_B.colnorm = dynamic_simulation_B.beta1[0].im /
            dynamic_simulation_B.beta1[0].re;
          dynamic_simulation_B.absxk = dynamic_simulation_B.colnorm *
            dynamic_simulation_B.beta1[0].im + dynamic_simulation_B.beta1[0].re;
          dynamic_simulation_B.scale = (dynamic_simulation_B.colnorm * D[0].im +
            D[0].re) / dynamic_simulation_B.absxk;
          dynamic_simulation_B.absxk = (D[0].im - dynamic_simulation_B.colnorm *
            D[0].re) / dynamic_simulation_B.absxk;
        } else if (dynamic_simulation_B.scale == dynamic_simulation_B.colnorm) {
          dynamic_simulation_B.absxk = dynamic_simulation_B.beta1[0].re > 0.0 ?
            0.5 : -0.5;
          dynamic_simulation_B.t = dynamic_simulation_B.beta1[0].im > 0.0 ? 0.5 :
            -0.5;
          dynamic_simulation_B.scale = (D[0].re * dynamic_simulation_B.absxk +
            D[0].im * dynamic_simulation_B.t) / dynamic_simulation_B.colnorm;
          dynamic_simulation_B.absxk = (D[0].im * dynamic_simulation_B.absxk -
            D[0].re * dynamic_simulation_B.t) / dynamic_simulation_B.colnorm;
        } else {
          dynamic_simulation_B.colnorm = dynamic_simulation_B.beta1[0].re /
            dynamic_simulation_B.beta1[0].im;
          dynamic_simulation_B.absxk = dynamic_simulation_B.colnorm *
            dynamic_simulation_B.beta1[0].re + dynamic_simulation_B.beta1[0].im;
          dynamic_simulation_B.scale = (dynamic_simulation_B.colnorm * D[0].re +
            D[0].im) / dynamic_simulation_B.absxk;
          dynamic_simulation_B.absxk = (dynamic_simulation_B.colnorm * D[0].im -
            D[0].re) / dynamic_simulation_B.absxk;
        }
      }

      D[0].re = dynamic_simulation_B.scale;
      D[0].im = dynamic_simulation_B.absxk;
      if (dynamic_simulation_B.beta1[1].im == 0.0) {
        if (D[1].im == 0.0) {
          dynamic_simulation_B.scale = D[1].re / dynamic_simulation_B.beta1[1].
            re;
          dynamic_simulation_B.absxk = 0.0;
        } else if (D[1].re == 0.0) {
          dynamic_simulation_B.scale = 0.0;
          dynamic_simulation_B.absxk = D[1].im / dynamic_simulation_B.beta1[1].
            re;
        } else {
          dynamic_simulation_B.scale = D[1].re / dynamic_simulation_B.beta1[1].
            re;
          dynamic_simulation_B.absxk = D[1].im / dynamic_simulation_B.beta1[1].
            re;
        }
      } else if (dynamic_simulation_B.beta1[1].re == 0.0) {
        if (D[1].re == 0.0) {
          dynamic_simulation_B.scale = D[1].im / dynamic_simulation_B.beta1[1].
            im;
          dynamic_simulation_B.absxk = 0.0;
        } else if (D[1].im == 0.0) {
          dynamic_simulation_B.scale = 0.0;
          dynamic_simulation_B.absxk = -(D[1].re / dynamic_simulation_B.beta1[1]
            .im);
        } else {
          dynamic_simulation_B.scale = D[1].im / dynamic_simulation_B.beta1[1].
            im;
          dynamic_simulation_B.absxk = -(D[1].re / dynamic_simulation_B.beta1[1]
            .im);
        }
      } else {
        dynamic_simulation_B.colnorm = fabs(dynamic_simulation_B.beta1[1].re);
        dynamic_simulation_B.scale = fabs(dynamic_simulation_B.beta1[1].im);
        if (dynamic_simulation_B.colnorm > dynamic_simulation_B.scale) {
          dynamic_simulation_B.colnorm = dynamic_simulation_B.beta1[1].im /
            dynamic_simulation_B.beta1[1].re;
          dynamic_simulation_B.absxk = dynamic_simulation_B.colnorm *
            dynamic_simulation_B.beta1[1].im + dynamic_simulation_B.beta1[1].re;
          dynamic_simulation_B.scale = (dynamic_simulation_B.colnorm * D[1].im +
            D[1].re) / dynamic_simulation_B.absxk;
          dynamic_simulation_B.absxk = (D[1].im - dynamic_simulation_B.colnorm *
            D[1].re) / dynamic_simulation_B.absxk;
        } else if (dynamic_simulation_B.scale == dynamic_simulation_B.colnorm) {
          dynamic_simulation_B.absxk = dynamic_simulation_B.beta1[1].re > 0.0 ?
            0.5 : -0.5;
          dynamic_simulation_B.t = dynamic_simulation_B.beta1[1].im > 0.0 ? 0.5 :
            -0.5;
          dynamic_simulation_B.scale = (D[1].re * dynamic_simulation_B.absxk +
            D[1].im * dynamic_simulation_B.t) / dynamic_simulation_B.colnorm;
          dynamic_simulation_B.absxk = (D[1].im * dynamic_simulation_B.absxk -
            D[1].re * dynamic_simulation_B.t) / dynamic_simulation_B.colnorm;
        } else {
          dynamic_simulation_B.colnorm = dynamic_simulation_B.beta1[1].re /
            dynamic_simulation_B.beta1[1].im;
          dynamic_simulation_B.absxk = dynamic_simulation_B.colnorm *
            dynamic_simulation_B.beta1[1].re + dynamic_simulation_B.beta1[1].im;
          dynamic_simulation_B.scale = (dynamic_simulation_B.colnorm * D[1].re +
            D[1].im) / dynamic_simulation_B.absxk;
          dynamic_simulation_B.absxk = (dynamic_simulation_B.colnorm * D[1].im -
            D[1].re) / dynamic_simulation_B.absxk;
        }
      }

      D[1].re = dynamic_simulation_B.scale;
      D[1].im = dynamic_simulation_B.absxk;
      if (dynamic_simulation_B.beta1[2].im == 0.0) {
        if (D[2].im == 0.0) {
          dynamic_simulation_B.scale = D[2].re / dynamic_simulation_B.beta1[2].
            re;
          dynamic_simulation_B.absxk = 0.0;
        } else if (D[2].re == 0.0) {
          dynamic_simulation_B.scale = 0.0;
          dynamic_simulation_B.absxk = D[2].im / dynamic_simulation_B.beta1[2].
            re;
        } else {
          dynamic_simulation_B.scale = D[2].re / dynamic_simulation_B.beta1[2].
            re;
          dynamic_simulation_B.absxk = D[2].im / dynamic_simulation_B.beta1[2].
            re;
        }
      } else if (dynamic_simulation_B.beta1[2].re == 0.0) {
        if (D[2].re == 0.0) {
          dynamic_simulation_B.scale = D[2].im / dynamic_simulation_B.beta1[2].
            im;
          dynamic_simulation_B.absxk = 0.0;
        } else if (D[2].im == 0.0) {
          dynamic_simulation_B.scale = 0.0;
          dynamic_simulation_B.absxk = -(D[2].re / dynamic_simulation_B.beta1[2]
            .im);
        } else {
          dynamic_simulation_B.scale = D[2].im / dynamic_simulation_B.beta1[2].
            im;
          dynamic_simulation_B.absxk = -(D[2].re / dynamic_simulation_B.beta1[2]
            .im);
        }
      } else {
        dynamic_simulation_B.colnorm = fabs(dynamic_simulation_B.beta1[2].re);
        dynamic_simulation_B.scale = fabs(dynamic_simulation_B.beta1[2].im);
        if (dynamic_simulation_B.colnorm > dynamic_simulation_B.scale) {
          dynamic_simulation_B.colnorm = dynamic_simulation_B.beta1[2].im /
            dynamic_simulation_B.beta1[2].re;
          dynamic_simulation_B.absxk = dynamic_simulation_B.colnorm *
            dynamic_simulation_B.beta1[2].im + dynamic_simulation_B.beta1[2].re;
          dynamic_simulation_B.scale = (dynamic_simulation_B.colnorm * D[2].im +
            D[2].re) / dynamic_simulation_B.absxk;
          dynamic_simulation_B.absxk = (D[2].im - dynamic_simulation_B.colnorm *
            D[2].re) / dynamic_simulation_B.absxk;
        } else if (dynamic_simulation_B.scale == dynamic_simulation_B.colnorm) {
          dynamic_simulation_B.absxk = dynamic_simulation_B.beta1[2].re > 0.0 ?
            0.5 : -0.5;
          dynamic_simulation_B.t = dynamic_simulation_B.beta1[2].im > 0.0 ? 0.5 :
            -0.5;
          dynamic_simulation_B.scale = (D[2].re * dynamic_simulation_B.absxk +
            D[2].im * dynamic_simulation_B.t) / dynamic_simulation_B.colnorm;
          dynamic_simulation_B.absxk = (D[2].im * dynamic_simulation_B.absxk -
            D[2].re * dynamic_simulation_B.t) / dynamic_simulation_B.colnorm;
        } else {
          dynamic_simulation_B.colnorm = dynamic_simulation_B.beta1[2].re /
            dynamic_simulation_B.beta1[2].im;
          dynamic_simulation_B.absxk = dynamic_simulation_B.colnorm *
            dynamic_simulation_B.beta1[2].re + dynamic_simulation_B.beta1[2].im;
          dynamic_simulation_B.scale = (dynamic_simulation_B.colnorm * D[2].re +
            D[2].im) / dynamic_simulation_B.absxk;
          dynamic_simulation_B.absxk = (dynamic_simulation_B.colnorm * D[2].im -
            D[2].re) / dynamic_simulation_B.absxk;
        }
      }

      D[2].re = dynamic_simulation_B.scale;
      D[2].im = dynamic_simulation_B.absxk;
      if (dynamic_simulation_B.beta1[3].im == 0.0) {
        if (D[3].im == 0.0) {
          dynamic_simulation_B.scale = D[3].re / dynamic_simulation_B.beta1[3].
            re;
          dynamic_simulation_B.absxk = 0.0;
        } else if (D[3].re == 0.0) {
          dynamic_simulation_B.scale = 0.0;
          dynamic_simulation_B.absxk = D[3].im / dynamic_simulation_B.beta1[3].
            re;
        } else {
          dynamic_simulation_B.scale = D[3].re / dynamic_simulation_B.beta1[3].
            re;
          dynamic_simulation_B.absxk = D[3].im / dynamic_simulation_B.beta1[3].
            re;
        }
      } else if (dynamic_simulation_B.beta1[3].re == 0.0) {
        if (D[3].re == 0.0) {
          dynamic_simulation_B.scale = D[3].im / dynamic_simulation_B.beta1[3].
            im;
          dynamic_simulation_B.absxk = 0.0;
        } else if (D[3].im == 0.0) {
          dynamic_simulation_B.scale = 0.0;
          dynamic_simulation_B.absxk = -(D[3].re / dynamic_simulation_B.beta1[3]
            .im);
        } else {
          dynamic_simulation_B.scale = D[3].im / dynamic_simulation_B.beta1[3].
            im;
          dynamic_simulation_B.absxk = -(D[3].re / dynamic_simulation_B.beta1[3]
            .im);
        }
      } else {
        dynamic_simulation_B.colnorm = fabs(dynamic_simulation_B.beta1[3].re);
        dynamic_simulation_B.scale = fabs(dynamic_simulation_B.beta1[3].im);
        if (dynamic_simulation_B.colnorm > dynamic_simulation_B.scale) {
          dynamic_simulation_B.colnorm = dynamic_simulation_B.beta1[3].im /
            dynamic_simulation_B.beta1[3].re;
          dynamic_simulation_B.absxk = dynamic_simulation_B.colnorm *
            dynamic_simulation_B.beta1[3].im + dynamic_simulation_B.beta1[3].re;
          dynamic_simulation_B.scale = (dynamic_simulation_B.colnorm * D[3].im +
            D[3].re) / dynamic_simulation_B.absxk;
          dynamic_simulation_B.absxk = (D[3].im - dynamic_simulation_B.colnorm *
            D[3].re) / dynamic_simulation_B.absxk;
        } else if (dynamic_simulation_B.scale == dynamic_simulation_B.colnorm) {
          dynamic_simulation_B.absxk = dynamic_simulation_B.beta1[3].re > 0.0 ?
            0.5 : -0.5;
          dynamic_simulation_B.t = dynamic_simulation_B.beta1[3].im > 0.0 ? 0.5 :
            -0.5;
          dynamic_simulation_B.scale = (D[3].re * dynamic_simulation_B.absxk +
            D[3].im * dynamic_simulation_B.t) / dynamic_simulation_B.colnorm;
          dynamic_simulation_B.absxk = (D[3].im * dynamic_simulation_B.absxk -
            D[3].re * dynamic_simulation_B.t) / dynamic_simulation_B.colnorm;
        } else {
          dynamic_simulation_B.colnorm = dynamic_simulation_B.beta1[3].re /
            dynamic_simulation_B.beta1[3].im;
          dynamic_simulation_B.absxk = dynamic_simulation_B.colnorm *
            dynamic_simulation_B.beta1[3].re + dynamic_simulation_B.beta1[3].im;
          dynamic_simulation_B.scale = (dynamic_simulation_B.colnorm * D[3].re +
            D[3].im) / dynamic_simulation_B.absxk;
          dynamic_simulation_B.absxk = (dynamic_simulation_B.colnorm * D[3].im -
            D[3].re) / dynamic_simulation_B.absxk;
        }
      }

      D[3].re = dynamic_simulation_B.scale;
      D[3].im = dynamic_simulation_B.absxk;
    }
  }
}

static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_GetP_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_dynamic_simu_T
  *pStruct)
{
  dynamic_simulati_emxFree_char_T(&pStruct->Type);
  dynamic_simulati_emxFree_real_T(&pStruct->MotionSubspace);
}

static void emxFreeStruct_o_robotics_manip_(o_robotics_manip_internal_Rig_T
  *pStruct)
{
  dynamic_simulati_emxFree_char_T(&pStruct->NameInternal);
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
  dynamic_simulati_emxFree_char_T(&pStruct->NameInternal);
  emxFreeStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static void emxFreeStruct_c_rigidBodyJoint1(c_rigidBodyJoint_dynamic_si_k_T
  *pStruct)
{
  dynamic_simulati_emxFree_char_T(&pStruct->Type);
}

static void emxFreeStruct_o_robotics_mani_k(o_robotics_manip_internal_R_k_T
  *pStruct)
{
  dynamic_simulati_emxFree_char_T(&pStruct->NameInternal);
  emxFreeStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static void emxFreeStruct_p_robotics_mani_k(p_robotics_manip_internal_R_k_T
  *pStruct)
{
  emxFreeStruct_o_robotics_mani_k(&pStruct->Base);
}

static void emxFreeStruct_robotics_slmani_k(robotics_slmanip_internal_b_k_T
  *pStruct)
{
  emxFreeStruct_p_robotics_mani_k(&pStruct->TreeInternal);
}

static void emxFreeStruct_n_robotics_mani_k(n_robotics_manip_internal_R_k_T
  *pStruct)
{
  dynamic_simulati_emxFree_char_T(&pStruct->NameInternal);
  emxFreeStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static void dynamic_simul_matlabCodegenHa_e(ros_slros_internal_block_Subs_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void emxFreeStruct_c_rigidBodyJoint2(c_rigidBodyJoint_dynamic_s_kq_T
  *pStruct)
{
  dynamic_simulati_emxFree_char_T(&pStruct->Type);
  dynamic_simulati_emxFree_real_T(&pStruct->MotionSubspace);
}

static void emxFreeStruct_o_robotics_man_kq(o_robotics_manip_internal__kq_T
  *pStruct)
{
  emxFreeStruct_c_rigidBodyJoint2(&pStruct->JointInternal);
}

static void emxFreeStruct_p_robotics_man_kq(p_robotics_manip_internal__kq_T
  *pStruct)
{
  emxFreeStruct_o_robotics_man_kq(&pStruct->Base);
}

static void emxFreeStruct_robotics_slman_kq(robotics_slmanip_internal__kq_T
  *pStruct)
{
  emxFreeStruct_p_robotics_man_kq(&pStruct->TreeInternal);
}

static void emxFreeStruct_n_robotics_man_kq(n_robotics_manip_internal__kq_T
  *pStruct)
{
  emxFreeStruct_c_rigidBodyJoint2(&pStruct->JointInternal);
}

static void matlabCodegenHandle_matl_kq2tzt(ros_slros_internal_block_Publ_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_dynamic_simu_T
  *pStruct)
{
  dynamic_simulati_emxInit_char_T(&pStruct->Type, 2);
  dynamic_simulati_emxInit_real_T(&pStruct->MotionSubspace, 2);
}

static void emxInitStruct_o_robotics_manip_(o_robotics_manip_internal_Rig_T
  *pStruct)
{
  dynamic_simulati_emxInit_char_T(&pStruct->NameInternal, 2);
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
  dynamic_simulati_emxInit_char_T(&pStruct->NameInternal, 2);
  emxInitStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static n_robotics_manip_internal_Rig_T *dynamic_sim_RigidBody_RigidBody
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynamic_simul_T *switch_expression;
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
  dynami_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 13; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 0.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  dynami_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynamic_simulati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynami_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynamic_simulati_emxFree_char_T(&switch_expression);
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
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *dynamic_s_RigidBody_RigidBody_k
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynamic_simul_T *switch_expression;
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
  dynami_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 1.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynami_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynamic_simulati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynami_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynamic_simulati_emxFree_char_T(&switch_expression);
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
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *dynamic__RigidBody_RigidBody_kq
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynamic_simul_T *switch_expression;
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
  dynami_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 2.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynami_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynamic_simulati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynami_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynamic_simulati_emxFree_char_T(&switch_expression);
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
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = -1.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *dynamic_RigidBody_RigidBody_kq2
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynamic_simul_T *switch_expression;
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
  dynami_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 3.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynami_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynamic_simulati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynami_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynamic_simulati_emxFree_char_T(&switch_expression);
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
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *dynami_RigidBody_RigidBody_kq2t
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynamic_simul_T *switch_expression;
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
  dynami_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 4.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynami_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynamic_simulati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynami_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynamic_simulati_emxFree_char_T(&switch_expression);
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
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *dynam_RigidBody_RigidBody_kq2tz
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynamic_simul_T *switch_expression;
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
  dynami_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 5.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynami_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynamic_simulati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynami_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynamic_simulati_emxFree_char_T(&switch_expression);
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
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 1.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *dyna_RigidBody_RigidBody_kq2tzt
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynamic_simul_T *switch_expression;
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
  dynami_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 6.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynami_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynamic_simulati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynami_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynamic_simulati_emxFree_char_T(&switch_expression);
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
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *dyn_RigidBody_RigidBody_kq2tzta
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynamic_simul_T *switch_expression;
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
  dynami_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 11; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 7.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  dynami_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynamic_simulati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynami_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynamic_simulati_emxFree_char_T(&switch_expression);
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
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static o_robotics_manip_internal_Rig_T *dy_RigidBody_RigidBody_kq2tztaj
  (o_robotics_manip_internal_Rig_T *obj)
{
  o_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynamic_simul_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
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
  dynami_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  dynami_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynamic_simulati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynami_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynamic_simulati_emxFree_char_T(&switch_expression);
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
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  return b_obj;
}

static p_robotics_manip_internal_Rig_T *dyn_RigidBodyTree_RigidBodyTree
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
  obj->Bodies[0] = dynamic_sim_RigidBody_RigidBody(iobj_0);
  obj->Bodies[0]->Index = 1.0;
  obj->Bodies[1] = dynamic_s_RigidBody_RigidBody_k(iobj_7);
  obj->Bodies[1]->Index = 2.0;
  obj->Bodies[2] = dynamic__RigidBody_RigidBody_kq(iobj_1);
  obj->Bodies[2]->Index = 3.0;
  obj->Bodies[3] = dynamic_RigidBody_RigidBody_kq2(iobj_2);
  obj->Bodies[3]->Index = 4.0;
  obj->Bodies[4] = dynami_RigidBody_RigidBody_kq2t(iobj_3);
  obj->Bodies[4]->Index = 5.0;
  obj->Bodies[5] = dynam_RigidBody_RigidBody_kq2tz(iobj_4);
  obj->Bodies[5]->Index = 6.0;
  obj->Bodies[6] = dyna_RigidBody_RigidBody_kq2tzt(iobj_5);
  obj->Bodies[6]->Index = 7.0;
  obj->Bodies[7] = dyn_RigidBody_RigidBody_kq2tzta(iobj_6);
  obj->Bodies[7]->Index = 8.0;
  obj->NumBodies = 8.0;
  obj->PositionNumber = 6.0;
  obj->VelocityNumber = 6.0;
  for (i = 0; i < 16; i++) {
    obj->PositionDoFMap[i] = tmp[i];
  }

  dy_RigidBody_RigidBody_kq2tztaj(&obj->Base);
  return b_obj;
}

static void emxInitStruct_c_rigidBodyJoint1(c_rigidBodyJoint_dynamic_si_k_T
  *pStruct)
{
  dynamic_simulati_emxInit_char_T(&pStruct->Type, 2);
}

static void emxInitStruct_o_robotics_mani_k(o_robotics_manip_internal_R_k_T
  *pStruct)
{
  dynamic_simulati_emxInit_char_T(&pStruct->NameInternal, 2);
  emxInitStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static void emxInitStruct_p_robotics_mani_k(p_robotics_manip_internal_R_k_T
  *pStruct)
{
  emxInitStruct_o_robotics_mani_k(&pStruct->Base);
}

static void emxInitStruct_robotics_slmani_k(robotics_slmanip_internal_b_k_T
  *pStruct)
{
  emxInitStruct_p_robotics_mani_k(&pStruct->TreeInternal);
}

static void emxInitStruct_n_robotics_mani_k(n_robotics_manip_internal_R_k_T
  *pStruct)
{
  dynamic_simulati_emxInit_char_T(&pStruct->NameInternal, 2);
  emxInitStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static n_robotics_manip_internal_R_k_T *d_RigidBody_RigidBody_kq2tztajw
  (n_robotics_manip_internal_R_k_T *obj)
{
  n_robotics_manip_internal_R_k_T *b_obj;
  emxArray_char_T_dynamic_simul_T *switch_expression;
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
  dynami_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 13; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 0.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  dynami_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  dynamic_simulati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynami_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynamic_simulati_emxFree_char_T(&switch_expression);
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

static n_robotics_manip_internal_R_k_T *RigidBody_RigidBody_kq2tztajw1
  (n_robotics_manip_internal_R_k_T *obj)
{
  n_robotics_manip_internal_R_k_T *b_obj;
  emxArray_char_T_dynamic_simul_T *switch_expression;
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
  dynami_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 1.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynami_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  dynamic_simulati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynami_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynamic_simulati_emxFree_char_T(&switch_expression);
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

static n_robotics_manip_internal_R_k_T *RigidBody_RigidBody_kq2tztajw1b
  (n_robotics_manip_internal_R_k_T *obj)
{
  n_robotics_manip_internal_R_k_T *b_obj;
  emxArray_char_T_dynamic_simul_T *switch_expression;
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
  dynami_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 2.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynami_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  dynamic_simulati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynami_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynamic_simulati_emxFree_char_T(&switch_expression);
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

static n_robotics_manip_internal_R_k_T *RigidBody_RigidBod_kq2tztajw1bp
  (n_robotics_manip_internal_R_k_T *obj)
{
  n_robotics_manip_internal_R_k_T *b_obj;
  emxArray_char_T_dynamic_simul_T *switch_expression;
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
  dynami_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 3.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynami_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  dynamic_simulati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynami_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynamic_simulati_emxFree_char_T(&switch_expression);
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

static n_robotics_manip_internal_R_k_T *RigidBody_RigidBo_kq2tztajw1bp5
  (n_robotics_manip_internal_R_k_T *obj)
{
  n_robotics_manip_internal_R_k_T *b_obj;
  emxArray_char_T_dynamic_simul_T *switch_expression;
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
  dynami_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 4.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynami_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  dynamic_simulati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynami_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynamic_simulati_emxFree_char_T(&switch_expression);
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

static p_robotics_manip_internal_R_k_T *d_RigidBodyTree_RigidBodyTree_k
  (p_robotics_manip_internal_R_k_T *obj, n_robotics_manip_internal_R_k_T *iobj_0,
   n_robotics_manip_internal_R_k_T *iobj_1, n_robotics_manip_internal_R_k_T
   *iobj_2, n_robotics_manip_internal_R_k_T *iobj_3,
   n_robotics_manip_internal_R_k_T *iobj_4, n_robotics_manip_internal_R_k_T
   *iobj_5, n_robotics_manip_internal_R_k_T *iobj_6,
   n_robotics_manip_internal_R_k_T *iobj_7)
{
  p_robotics_manip_internal_R_k_T *b_obj;
  o_robotics_manip_internal_R_k_T *obj_0;
  emxArray_char_T_dynamic_simul_T *switch_expression;
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
  obj->Bodies[0] = d_RigidBody_RigidBody_kq2tztajw(iobj_0);
  obj->Bodies[1] = RigidBody_RigidBody_kq2tztajw1(iobj_7);
  obj->Bodies[2] = RigidBody_RigidBody_kq2tztajw1b(iobj_1);
  obj->Bodies[3] = RigidBody_RigidBod_kq2tztajw1bp(iobj_2);
  obj->Bodies[4] = RigidBody_RigidBo_kq2tztajw1bp5(iobj_3);
  b_kstr = iobj_4->NameInternal->size[0] * iobj_4->NameInternal->size[1];
  iobj_4->NameInternal->size[0] = 1;
  iobj_4->NameInternal->size[1] = 10;
  dynami_emxEnsureCapacity_char_T(iobj_4->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    iobj_4->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  iobj_4->ParentIndex = 5.0;
  b_kstr = iobj_4->JointInternal.Type->size[0] * iobj_4->
    JointInternal.Type->size[1];
  iobj_4->JointInternal.Type->size[0] = 1;
  iobj_4->JointInternal.Type->size[1] = 8;
  dynami_emxEnsureCapacity_char_T(iobj_4->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    iobj_4->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  dynamic_simulati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_4->JointInternal.Type->size[1];
  dynami_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
  dynami_emxEnsureCapacity_char_T(iobj_5->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    iobj_5->NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  iobj_5->ParentIndex = 6.0;
  b_kstr = iobj_5->JointInternal.Type->size[0] * iobj_5->
    JointInternal.Type->size[1];
  iobj_5->JointInternal.Type->size[0] = 1;
  iobj_5->JointInternal.Type->size[1] = 8;
  dynami_emxEnsureCapacity_char_T(iobj_5->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    iobj_5->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_5->JointInternal.Type->size[1];
  dynami_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
  dynami_emxEnsureCapacity_char_T(iobj_6->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 11; b_kstr++) {
    iobj_6->NameInternal->data[b_kstr] = tmp_6[b_kstr];
  }

  iobj_6->ParentIndex = 7.0;
  b_kstr = iobj_6->JointInternal.Type->size[0] * iobj_6->
    JointInternal.Type->size[1];
  iobj_6->JointInternal.Type->size[0] = 1;
  iobj_6->JointInternal.Type->size[1] = 5;
  dynami_emxEnsureCapacity_char_T(iobj_6->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_6->JointInternal.Type->data[b_kstr] = tmp_7[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_6->JointInternal.Type->size[1];
  dynami_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
  dynami_emxEnsureCapacity_char_T(obj->Base.NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->Base.NameInternal->data[b_kstr] = tmp_8[b_kstr];
  }

  b_kstr = obj->Base.JointInternal.Type->size[0] * obj->
    Base.JointInternal.Type->size[1];
  obj->Base.JointInternal.Type->size[0] = 1;
  obj->Base.JointInternal.Type->size[1] = 5;
  dynami_emxEnsureCapacity_char_T(obj->Base.JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj_0->JointInternal.Type->data[b_kstr] = tmp_7[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->Base.JointInternal.Type->size[1];
  dynami_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynamic_simulati_emxFree_char_T(&switch_expression);
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

static void emxInitStruct_c_rigidBodyJoint2(c_rigidBodyJoint_dynamic_s_kq_T
  *pStruct)
{
  dynamic_simulati_emxInit_char_T(&pStruct->Type, 2);
  dynamic_simulati_emxInit_real_T(&pStruct->MotionSubspace, 2);
}

static void emxInitStruct_o_robotics_man_kq(o_robotics_manip_internal__kq_T
  *pStruct)
{
  emxInitStruct_c_rigidBodyJoint2(&pStruct->JointInternal);
}

static void emxInitStruct_p_robotics_man_kq(p_robotics_manip_internal__kq_T
  *pStruct)
{
  emxInitStruct_o_robotics_man_kq(&pStruct->Base);
}

static void emxInitStruct_robotics_slman_kq(robotics_slmanip_internal__kq_T
  *pStruct)
{
  emxInitStruct_p_robotics_man_kq(&pStruct->TreeInternal);
}

static void emxInitStruct_n_robotics_man_kq(n_robotics_manip_internal__kq_T
  *pStruct)
{
  emxInitStruct_c_rigidBodyJoint2(&pStruct->JointInternal);
}

static n_robotics_manip_internal__kq_T *RigidBody_RigidB_kq2tztajw1bp5d
  (n_robotics_manip_internal__kq_T *obj)
{
  n_robotics_manip_internal__kq_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynamic_simul_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.012583419040406959,
    -0.00021487638648447484, -0.00022605919127205462, 0.0, -0.00392971169381184,
    0.00047022930128152475, -0.00021487638648447484, 0.00052369449451288713,
    -0.00011525315957400814, 0.00392971169381184, 0.0, -0.00449464704691423,
    -0.00022605919127205462, -0.00011525315957400814, 0.012646079447789898,
    -0.00047022930128152475, 0.00449464704691423, 0.0, 0.0, 0.00392971169381184,
    -0.00047022930128152475, 0.0785942338762368, 0.0, 0.0, -0.00392971169381184,
    0.0, 0.00449464704691423, 0.0, 0.0785942338762368, 0.0,
    0.00047022930128152475, -0.00449464704691423, 0.0, 0.0, 0.0,
    0.0785942338762368 };

  static const char_T tmp_1[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_2[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_3[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_4[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 0.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  dynami_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynamic_simulati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynami_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynamic_simulati_emxFree_char_T(&switch_expression);
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

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static n_robotics_manip_internal__kq_T *RigidBody_Rigid_kq2tztajw1bp5do
  (n_robotics_manip_internal__kq_T *obj)
{
  n_robotics_manip_internal__kq_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynamic_simul_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.012559660892485551, 0.00032713982710414,
    -1.32683892634271E-6, 0.0, -0.0, 0.0037143634929909515, 0.00032713982710414,
    0.00018048007331848145, 9.17416945099368E-5, 0.0, 0.0, 0.0029444543779393356,
    -1.32683892634271E-6, 9.17416945099368E-5, 0.012672078048055372,
    -0.0037143634929909515, -0.0029444543779393356, 0.0, 0.0, 0.0,
    -0.0037143634929909515, 0.0785942338762368, 0.0, 0.0, -0.0, 0.0,
    -0.0029444543779393356, 0.0, 0.0785942338762368, 0.0, 0.0037143634929909515,
    0.0029444543779393356, 0.0, 0.0, 0.0, 0.0785942338762368 };

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
  obj->ParentIndex = 1.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynami_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynamic_simulati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynami_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynamic_simulati_emxFree_char_T(&switch_expression);
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

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static n_robotics_manip_internal__kq_T *d_RigidBody_Rigid_m
  (n_robotics_manip_internal__kq_T *obj)
{
  n_robotics_manip_internal__kq_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynamic_simul_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.012452446533447339, 0.00097164648860403489,
    -0.0012079400901817208, 0.0, 0.0036112478581453288, 0.0024995324199659588,
    0.00097164648860403489, 0.0077631932790618, 0.0060529024261622953,
    -0.0036112478581453288, 0.0, 0.0012907531029494371, -0.0012079400901817208,
    0.0060529024261622953, 0.0051581160550583753, -0.0024995324199659588,
    -0.0012907531029494371, 0.0, 0.0, -0.0036112478581453288,
    -0.0024995324199659588, 0.0785942338762368, 0.0, 0.0, 0.0036112478581453288,
    0.0, -0.0012907531029494371, 0.0, 0.0785942338762368, 0.0,
    0.0024995324199659588, 0.0012907531029494371, 0.0, 0.0, 0.0,
    0.0785942338762368 };

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
  obj->ParentIndex = 2.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynami_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynamic_simulati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynami_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynamic_simulati_emxFree_char_T(&switch_expression);
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

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = -1.0;
  return b_obj;
}

static n_robotics_manip_internal__kq_T *d_RigidBody_Rigid_b
  (n_robotics_manip_internal__kq_T *obj)
{
  n_robotics_manip_internal__kq_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynamic_simul_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.0123976159829631, 0.00022039108420015264,
    2.1332825710116445E-6, 0.0, -2.56217202436532E-5, 0.0010295844637787021,
    0.00022039108420015264, 0.00014803877895089843, -9.2077339045129963E-5,
    2.56217202436532E-5, 0.0, 0.0024737535112545539, 2.1332825710116445E-6,
    -9.2077339045129963E-5, 0.012477575138803739, -0.0010295844637787021,
    -0.0024737535112545539, 0.0, 0.0, 2.56217202436532E-5,
    -0.0010295844637787021, 0.0785942338762368, 0.0, 0.0, -2.56217202436532E-5,
    0.0, -0.0024737535112545539, 0.0, 0.0785942338762368, 0.0,
    0.0010295844637787021, 0.0024737535112545539, 0.0, 0.0, 0.0,
    0.0785942338762368 };

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
  obj->ParentIndex = 3.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynami_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynamic_simulati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynami_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynamic_simulati_emxFree_char_T(&switch_expression);
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

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static n_robotics_manip_internal__kq_T *d_RigidBody_Rigid_a
  (n_robotics_manip_internal__kq_T *obj)
{
  n_robotics_manip_internal__kq_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynamic_simul_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.0123990349928174, -2.7766271471639167E-6,
    0.00022466935228286869, 0.0, 0.0010818496293063995, 4.275526322867282E-5,
    -2.7766271471639167E-6, 0.012491487094789458, 9.2330220708293281E-5,
    -0.0010818496293063995, 0.0, -0.0026650518765093138, 0.00022466935228286869,
    9.2330220708293281E-5, 0.00016056153744711284, -4.275526322867282E-5,
    0.0026650518765093138, 0.0, 0.0, -0.0010818496293063995,
    -4.275526322867282E-5, 0.0785942338762368, 0.0, 0.0, 0.0010818496293063995,
    0.0, 0.0026650518765093138, 0.0, 0.0785942338762368, 0.0,
    4.275526322867282E-5, -0.0026650518765093138, 0.0, 0.0, 0.0,
    0.0785942338762368 };

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
  obj->ParentIndex = 4.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynami_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynamic_simulati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynami_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynamic_simulati_emxFree_char_T(&switch_expression);
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

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static n_robotics_manip_internal__kq_T *d_RigidBody_Rigid_h
  (n_robotics_manip_internal__kq_T *obj)
{
  n_robotics_manip_internal__kq_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynamic_simul_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.012440329403329006, 2.388185677857016E-6,
    0.00012602126519218373, 0.0, -0.0021015312196166957, -3.5996159115316455E-5,
    2.388185677857016E-6, 0.012510746127660349, -9.077919321137075E-5,
    0.0021015312196166957, 0.0, -0.0023173509858408423, 0.00012602126519218373,
    -9.077919321137075E-5, 0.00013851261456175015, 3.5996159115316455E-5,
    0.0023173509858408423, 0.0, 0.0, 0.0021015312196166957,
    3.5996159115316455E-5, 0.0785942338762368, 0.0, 0.0, -0.0021015312196166957,
    0.0, 0.0023173509858408423, 0.0, 0.0785942338762368, 0.0,
    -3.5996159115316455E-5, -0.0023173509858408423, 0.0, 0.0, 0.0,
    0.0785942338762368 };

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
  obj->ParentIndex = 5.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynami_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynamic_simulati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynami_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynamic_simulati_emxFree_char_T(&switch_expression);
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

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 1.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static n_robotics_manip_internal__kq_T *d_RigidBody_Rigid_i
  (n_robotics_manip_internal__kq_T *obj)
{
  n_robotics_manip_internal__kq_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynamic_simul_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 1.0067862401982823E-5, 2.0153545938371486E-9,
    5.3285284099072352E-9, 0.0, -1.6782149839359722E-7, -0.00026087851925284686,
    2.0153545938371486E-9, 1.4574493611914028E-5, 1.6742291194075022E-9,
    1.6782149839359722E-7, 0.0, -1.957917481258634E-7, 5.3285284099072352E-9,
    1.6742291194075022E-9, 1.006193323459457E-5, 0.00026087851925284686,
    1.957917481258634E-7, 0.0, 0.0, 1.6782149839359722E-7,
    0.00026087851925284686, 0.0279702497322662, 0.0, 0.0, -1.6782149839359722E-7,
    0.0, 1.957917481258634E-7, 0.0, 0.0279702497322662, 0.0,
    -0.00026087851925284686, -1.957917481258634E-7, 0.0, 0.0, 0.0,
    0.0279702497322662 };

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
  obj->ParentIndex = 6.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynami_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynamic_simulati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynami_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynamic_simulati_emxFree_char_T(&switch_expression);
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

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static n_robotics_manip_internal__kq_T *d_RigidBody_Rigid_mc
  (n_robotics_manip_internal__kq_T *obj)
{
  n_robotics_manip_internal__kq_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynamic_simul_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0,
    -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0 };

  static const char_T tmp_1[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_2[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_3[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_4[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 7.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  dynami_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynamic_simulati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynami_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynamic_simulati_emxFree_char_T(&switch_expression);
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

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static o_robotics_manip_internal__kq_T *d_RigidBody_Rigid_e
  (o_robotics_manip_internal__kq_T *obj)
{
  o_robotics_manip_internal__kq_T *b_obj;
  emxArray_char_T_dynamic_simul_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  dynami_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  dynamic_simulati_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynami_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
      dynamic_simulation_B.b_e[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] !=
              dynamic_simulation_B.b_e[loop_ub]) {
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

  dynamic_simulati_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      dynamic_simulation_B.msubspace_data[b_kstr] = tmp[b_kstr];
    }

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
      dynamic_simulation_B.msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      dynamic_simulation_B.msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  dynami_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] =
      dynamic_simulation_B.msubspace_data[b_kstr];
  }

  return b_obj;
}

static p_robotics_manip_internal__kq_T *RigidBodyTree_RigidBodyTree_kq
  (p_robotics_manip_internal__kq_T *obj, n_robotics_manip_internal__kq_T *iobj_0,
   n_robotics_manip_internal__kq_T *iobj_1, n_robotics_manip_internal__kq_T
   *iobj_2, n_robotics_manip_internal__kq_T *iobj_3,
   n_robotics_manip_internal__kq_T *iobj_4, n_robotics_manip_internal__kq_T
   *iobj_5, n_robotics_manip_internal__kq_T *iobj_6,
   n_robotics_manip_internal__kq_T *iobj_7)
{
  p_robotics_manip_internal__kq_T *b_obj;
  int32_T i;
  static const int8_T tmp[16] = { 0, 1, 2, 3, 4, 5, 6, 0, -1, 1, 2, 3, 4, 5, 6,
    -1 };

  b_obj = obj;
  obj->Bodies[0] = RigidBody_RigidB_kq2tztajw1bp5d(iobj_0);
  obj->Bodies[0]->Index = 1.0;
  obj->Bodies[1] = RigidBody_Rigid_kq2tztajw1bp5do(iobj_7);
  obj->Bodies[1]->Index = 2.0;
  obj->Bodies[2] = d_RigidBody_Rigid_m(iobj_1);
  obj->Bodies[2]->Index = 3.0;
  obj->Bodies[3] = d_RigidBody_Rigid_b(iobj_2);
  obj->Bodies[3]->Index = 4.0;
  obj->Bodies[4] = d_RigidBody_Rigid_a(iobj_3);
  obj->Bodies[4]->Index = 5.0;
  obj->Bodies[5] = d_RigidBody_Rigid_h(iobj_4);
  obj->Bodies[5]->Index = 6.0;
  obj->Bodies[6] = d_RigidBody_Rigid_i(iobj_5);
  obj->Bodies[6]->Index = 7.0;
  obj->Bodies[7] = d_RigidBody_Rigid_mc(iobj_6);
  obj->Bodies[7]->Index = 8.0;
  obj->NumBodies = 8.0;
  obj->Gravity[0] = 0.0;
  obj->Gravity[1] = 0.0;
  obj->Gravity[2] = 0.0;
  obj->VelocityNumber = 6.0;
  for (i = 0; i < 16; i++) {
    obj->PositionDoFMap[i] = tmp[i];
  }

  for (i = 0; i < 16; i++) {
    obj->VelocityDoFMap[i] = tmp[i];
  }

  d_RigidBody_Rigid_e(&obj->Base);
  return b_obj;
}

// Model step function
void dynamic_simulation_step(void)
{
  emxArray_real_T_dynamic_simul_T *b;
  robotics_slmanip_internal_b_k_T *obj;
  p_robotics_manip_internal_R_k_T *obj_0;
  emxArray_f_cell_wrap_dynamic__T *Ttree;
  emxArray_char_T_dynamic_simul_T *bname;
  n_robotics_manip_internal_R_k_T *obj_1;
  robotics_slmanip_internal__kq_T *obj_2;
  emxArray_real_T_dynamic_simul_T *L;
  emxArray_real_T_dynamic_simul_T *lambda;
  emxArray_real_T_dynamic_simul_T *H;
  emxArray_real_T_dynamic_simul_T *tmp;
  static const char_T tmp_0[11] = { 'e', 'd', 'o', '_', 'l', 'i', 'n', 'k', '_',
    'e', 'e' };

  static const char_T tmp_1[5] = { 'w', 'o', 'r', 'l', 'd' };

  int32_T exitg1;
  boolean_T exitg2;
  if (rtmIsMajorTimeStep(dynamic_simulation_M)) {
    // set solver stop time
    rtsiSetSolverStopTime(&dynamic_simulation_M->solverInfo,
                          ((dynamic_simulation_M->Timing.clockTick0+1)*
      dynamic_simulation_M->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if (rtmIsMinorTimeStep(dynamic_simulation_M)) {
    dynamic_simulation_M->Timing.t[0] = rtsiGetT
      (&dynamic_simulation_M->solverInfo);
  }

  // MATLABSystem: '<S21>/Get Parameter12'
  ParamGet_dynamic_simulation_194.get_parameter(&dynamic_simulation_B.vNum);

  // MATLABSystem: '<S21>/Get Parameter1'
  ParamGet_dynamic_simulation_132.get_parameter(&dynamic_simulation_B.bid1);

  // MATLABSystem: '<S21>/Get Parameter4'
  ParamGet_dynamic_simulation_137.get_parameter(&dynamic_simulation_B.j);

  // MATLABSystem: '<S21>/Get Parameter5'
  ParamGet_dynamic_simulation_138.get_parameter(&dynamic_simulation_B.K23);

  // MATLABSystem: '<S21>/Get Parameter6'
  ParamGet_dynamic_simulation_139.get_parameter(&dynamic_simulation_B.K24);

  // MATLABSystem: '<S21>/Get Parameter7'
  ParamGet_dynamic_simulation_140.get_parameter(&dynamic_simulation_B.K34);

  // Integrator: '<S10>/Position' incorporates:
  //   MATLABSystem: '<S21>/Get Parameter1'
  //   MATLABSystem: '<S21>/Get Parameter12'
  //   MATLABSystem: '<S21>/Get Parameter4'
  //   MATLABSystem: '<S21>/Get Parameter5'
  //   MATLABSystem: '<S21>/Get Parameter6'
  //   MATLABSystem: '<S21>/Get Parameter7'

  if (dynamic_simulation_DW.Position_IWORK != 0) {
    dynamic_simulation_X.Position_CSTATE[0] = dynamic_simulation_B.vNum;
    dynamic_simulation_X.Position_CSTATE[1] = dynamic_simulation_B.bid1;
    dynamic_simulation_X.Position_CSTATE[2] = dynamic_simulation_B.j;
    dynamic_simulation_X.Position_CSTATE[3] = dynamic_simulation_B.K23;
    dynamic_simulation_X.Position_CSTATE[4] = dynamic_simulation_B.K24;
    dynamic_simulation_X.Position_CSTATE[5] = dynamic_simulation_B.K34;
  }

  dynamic_simulati_emxInit_real_T(&b, 2);
  dynamic_sim_emxInit_f_cell_wrap(&Ttree, 2);
  dynamic_simulati_emxInit_char_T(&bname, 2);

  // MATLABSystem: '<S14>/MATLAB System' incorporates:
  //   Integrator: '<S10>/Position'

  RigidBodyTree_geometricJacobian(&dynamic_simulation_DW.obj_f.TreeInternal,
    dynamic_simulation_X.Position_CSTATE, b);

  // SignalConversion generated from: '<S17>/ SFunction ' incorporates:
  //   MATLAB Function: '<S4>/Assign to JointState msg'
  //   MATLABSystem: '<S19>/Get Parameter'
  //   MATLABSystem: '<S19>/Get Parameter1'
  //   MATLABSystem: '<S19>/Get Parameter2'
  //   MATLABSystem: '<S19>/Get Parameter3'
  //   MATLABSystem: '<S19>/Get Parameter4'
  //   MATLABSystem: '<S19>/Get Parameter5'

  ParamGet_dynamic_simulation_224.get_parameter(32U,
    dynamic_simulation_B.charValue,
    &dynamic_simulation_B.TmpSignalConversionAtSFunct[0]);
  ParamGet_dynamic_simulation_228.get_parameter(32U,
    dynamic_simulation_B.charValue_f,
    &dynamic_simulation_B.TmpSignalConversionAtSFunct[1]);
  ParamGet_dynamic_simulation_229.get_parameter(32U,
    dynamic_simulation_B.charValue_a,
    &dynamic_simulation_B.TmpSignalConversionAtSFunct[2]);
  ParamGet_dynamic_simulation_230.get_parameter(32U,
    dynamic_simulation_B.charValue_j,
    &dynamic_simulation_B.TmpSignalConversionAtSFunct[3]);
  ParamGet_dynamic_simulation_231.get_parameter(32U,
    dynamic_simulation_B.charValue_jz,
    &dynamic_simulation_B.TmpSignalConversionAtSFunct[4]);
  ParamGet_dynamic_simulation_232.get_parameter(32U,
    dynamic_simulation_B.charValue_o,
    &dynamic_simulation_B.TmpSignalConversionAtSFunct[5]);

  // MATLABSystem: '<S15>/MATLAB System' incorporates:
  //   Integrator: '<S10>/Position'

  obj = &dynamic_simulation_DW.obj_i;
  obj_0 = &dynamic_simulation_DW.obj_i.TreeInternal;
  RigidBodyTree_forwardKinemati_k(&obj->TreeInternal,
    dynamic_simulation_X.Position_CSTATE, Ttree);
  dynamic_simulation_B.bid1 = -1.0;
  dynamic_simulation_B.b_kstr = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj_0->Base.NameInternal->size[1];
  dynami_emxEnsureCapacity_char_T(bname, dynamic_simulation_B.b_kstr);
  dynamic_simulation_B.n_a = obj_0->Base.NameInternal->size[0] *
    obj_0->Base.NameInternal->size[1] - 1;
  for (dynamic_simulation_B.b_kstr = 0; dynamic_simulation_B.b_kstr <=
       dynamic_simulation_B.n_a; dynamic_simulation_B.b_kstr++) {
    bname->data[dynamic_simulation_B.b_kstr] = obj_0->Base.NameInternal->
      data[dynamic_simulation_B.b_kstr];
  }

  for (dynamic_simulation_B.b_kstr = 0; dynamic_simulation_B.b_kstr < 11;
       dynamic_simulation_B.b_kstr++) {
    dynamic_simulation_B.b_c[dynamic_simulation_B.b_kstr] =
      tmp_0[dynamic_simulation_B.b_kstr];
  }

  dynamic_simulation_B.b_bool = false;
  if (bname->size[1] == 11) {
    dynamic_simulation_B.b_kstr = 1;
    do {
      exitg1 = 0;
      if (dynamic_simulation_B.b_kstr - 1 < 11) {
        dynamic_simulation_B.n_a = dynamic_simulation_B.b_kstr - 1;
        if (bname->data[dynamic_simulation_B.n_a] !=
            dynamic_simulation_B.b_c[dynamic_simulation_B.n_a]) {
          exitg1 = 1;
        } else {
          dynamic_simulation_B.b_kstr++;
        }
      } else {
        dynamic_simulation_B.b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (dynamic_simulation_B.b_bool) {
    dynamic_simulation_B.bid1 = 0.0;
  } else {
    dynamic_simulation_B.vNum = obj->TreeInternal.NumBodies;
    dynamic_simulation_B.i = 0;
    exitg2 = false;
    while ((!exitg2) && (dynamic_simulation_B.i <= static_cast<int32_T>
                         (dynamic_simulation_B.vNum) - 1)) {
      obj_1 = obj_0->Bodies[dynamic_simulation_B.i];
      dynamic_simulation_B.b_kstr = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = obj_1->NameInternal->size[1];
      dynami_emxEnsureCapacity_char_T(bname, dynamic_simulation_B.b_kstr);
      dynamic_simulation_B.n_a = obj_1->NameInternal->size[0] *
        obj_1->NameInternal->size[1] - 1;
      for (dynamic_simulation_B.b_kstr = 0; dynamic_simulation_B.b_kstr <=
           dynamic_simulation_B.n_a; dynamic_simulation_B.b_kstr++) {
        bname->data[dynamic_simulation_B.b_kstr] = obj_1->NameInternal->
          data[dynamic_simulation_B.b_kstr];
      }

      for (dynamic_simulation_B.b_kstr = 0; dynamic_simulation_B.b_kstr < 11;
           dynamic_simulation_B.b_kstr++) {
        dynamic_simulation_B.b_c[dynamic_simulation_B.b_kstr] =
          tmp_0[dynamic_simulation_B.b_kstr];
      }

      dynamic_simulation_B.b_bool = false;
      if (bname->size[1] == 11) {
        dynamic_simulation_B.b_kstr = 1;
        do {
          exitg1 = 0;
          if (dynamic_simulation_B.b_kstr - 1 < 11) {
            dynamic_simulation_B.n_a = dynamic_simulation_B.b_kstr - 1;
            if (bname->data[dynamic_simulation_B.n_a] !=
                dynamic_simulation_B.b_c[dynamic_simulation_B.n_a]) {
              exitg1 = 1;
            } else {
              dynamic_simulation_B.b_kstr++;
            }
          } else {
            dynamic_simulation_B.b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (dynamic_simulation_B.b_bool) {
        dynamic_simulation_B.bid1 = static_cast<real_T>(dynamic_simulation_B.i)
          + 1.0;
        exitg2 = true;
      } else {
        dynamic_simulation_B.i++;
      }
    }
  }

  if (dynamic_simulation_B.bid1 == 0.0) {
    memset(&dynamic_simulation_B.T1[0], 0, sizeof(real_T) << 4U);
    dynamic_simulation_B.T1[0] = 1.0;
    dynamic_simulation_B.T1[5] = 1.0;
    dynamic_simulation_B.T1[10] = 1.0;
    dynamic_simulation_B.T1[15] = 1.0;
  } else {
    for (dynamic_simulation_B.b_kstr = 0; dynamic_simulation_B.b_kstr < 16;
         dynamic_simulation_B.b_kstr++) {
      dynamic_simulation_B.T1[dynamic_simulation_B.b_kstr] = Ttree->data[
        static_cast<int32_T>(dynamic_simulation_B.bid1) - 1]
        .f1[dynamic_simulation_B.b_kstr];
    }
  }

  dynamic_simulation_B.bid1 = -1.0;
  dynamic_simulation_B.b_kstr = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj_0->Base.NameInternal->size[1];
  dynami_emxEnsureCapacity_char_T(bname, dynamic_simulation_B.b_kstr);
  dynamic_simulation_B.n_a = obj_0->Base.NameInternal->size[0] *
    obj_0->Base.NameInternal->size[1] - 1;
  for (dynamic_simulation_B.b_kstr = 0; dynamic_simulation_B.b_kstr <=
       dynamic_simulation_B.n_a; dynamic_simulation_B.b_kstr++) {
    bname->data[dynamic_simulation_B.b_kstr] = obj_0->Base.NameInternal->
      data[dynamic_simulation_B.b_kstr];
  }

  for (dynamic_simulation_B.b_kstr = 0; dynamic_simulation_B.b_kstr < 5;
       dynamic_simulation_B.b_kstr++) {
    dynamic_simulation_B.b_g[dynamic_simulation_B.b_kstr] =
      tmp_1[dynamic_simulation_B.b_kstr];
  }

  dynamic_simulation_B.b_bool = false;
  if (bname->size[1] == 5) {
    dynamic_simulation_B.b_kstr = 1;
    do {
      exitg1 = 0;
      if (dynamic_simulation_B.b_kstr - 1 < 5) {
        dynamic_simulation_B.n_a = dynamic_simulation_B.b_kstr - 1;
        if (bname->data[dynamic_simulation_B.n_a] !=
            dynamic_simulation_B.b_g[dynamic_simulation_B.n_a]) {
          exitg1 = 1;
        } else {
          dynamic_simulation_B.b_kstr++;
        }
      } else {
        dynamic_simulation_B.b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (dynamic_simulation_B.b_bool) {
    dynamic_simulation_B.bid1 = 0.0;
  } else {
    dynamic_simulation_B.vNum = obj->TreeInternal.NumBodies;
    dynamic_simulation_B.i = 0;
    exitg2 = false;
    while ((!exitg2) && (dynamic_simulation_B.i <= static_cast<int32_T>
                         (dynamic_simulation_B.vNum) - 1)) {
      obj_1 = obj_0->Bodies[dynamic_simulation_B.i];
      dynamic_simulation_B.b_kstr = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = obj_1->NameInternal->size[1];
      dynami_emxEnsureCapacity_char_T(bname, dynamic_simulation_B.b_kstr);
      dynamic_simulation_B.n_a = obj_1->NameInternal->size[0] *
        obj_1->NameInternal->size[1] - 1;
      for (dynamic_simulation_B.b_kstr = 0; dynamic_simulation_B.b_kstr <=
           dynamic_simulation_B.n_a; dynamic_simulation_B.b_kstr++) {
        bname->data[dynamic_simulation_B.b_kstr] = obj_1->NameInternal->
          data[dynamic_simulation_B.b_kstr];
      }

      for (dynamic_simulation_B.b_kstr = 0; dynamic_simulation_B.b_kstr < 5;
           dynamic_simulation_B.b_kstr++) {
        dynamic_simulation_B.b_g[dynamic_simulation_B.b_kstr] =
          tmp_1[dynamic_simulation_B.b_kstr];
      }

      dynamic_simulation_B.b_bool = false;
      if (bname->size[1] == 5) {
        dynamic_simulation_B.b_kstr = 1;
        do {
          exitg1 = 0;
          if (dynamic_simulation_B.b_kstr - 1 < 5) {
            dynamic_simulation_B.n_a = dynamic_simulation_B.b_kstr - 1;
            if (bname->data[dynamic_simulation_B.n_a] !=
                dynamic_simulation_B.b_g[dynamic_simulation_B.n_a]) {
              exitg1 = 1;
            } else {
              dynamic_simulation_B.b_kstr++;
            }
          } else {
            dynamic_simulation_B.b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (dynamic_simulation_B.b_bool) {
        dynamic_simulation_B.bid1 = static_cast<real_T>(dynamic_simulation_B.i)
          + 1.0;
        exitg2 = true;
      } else {
        dynamic_simulation_B.i++;
      }
    }
  }

  dynamic_simulati_emxFree_char_T(&bname);

  // MATLABSystem: '<S15>/MATLAB System'
  if (dynamic_simulation_B.bid1 == 0.0) {
    memset(&dynamic_simulation_B.T2[0], 0, sizeof(real_T) << 4U);
    dynamic_simulation_B.T2[0] = 1.0;
    dynamic_simulation_B.T2[5] = 1.0;
    dynamic_simulation_B.T2[10] = 1.0;
    dynamic_simulation_B.T2[15] = 1.0;
  } else {
    for (dynamic_simulation_B.b_kstr = 0; dynamic_simulation_B.b_kstr < 16;
         dynamic_simulation_B.b_kstr++) {
      dynamic_simulation_B.T2[dynamic_simulation_B.b_kstr] = Ttree->data[
        static_cast<int32_T>(dynamic_simulation_B.bid1) - 1]
        .f1[dynamic_simulation_B.b_kstr];
    }
  }

  dynamic_sim_emxFree_f_cell_wrap(&Ttree);

  // MATLABSystem: '<S15>/MATLAB System'
  for (dynamic_simulation_B.b_kstr = 0; dynamic_simulation_B.b_kstr < 3;
       dynamic_simulation_B.b_kstr++) {
    dynamic_simulation_B.R_j[3 * dynamic_simulation_B.b_kstr] =
      dynamic_simulation_B.T2[dynamic_simulation_B.b_kstr];
    dynamic_simulation_B.R_j[3 * dynamic_simulation_B.b_kstr + 1] =
      dynamic_simulation_B.T2[dynamic_simulation_B.b_kstr + 4];
    dynamic_simulation_B.R_j[3 * dynamic_simulation_B.b_kstr + 2] =
      dynamic_simulation_B.T2[dynamic_simulation_B.b_kstr + 8];
  }

  for (dynamic_simulation_B.b_kstr = 0; dynamic_simulation_B.b_kstr < 9;
       dynamic_simulation_B.b_kstr++) {
    dynamic_simulation_B.R_d[dynamic_simulation_B.b_kstr] =
      -dynamic_simulation_B.R_j[dynamic_simulation_B.b_kstr];
  }

  for (dynamic_simulation_B.b_kstr = 0; dynamic_simulation_B.b_kstr < 3;
       dynamic_simulation_B.b_kstr++) {
    dynamic_simulation_B.i = dynamic_simulation_B.b_kstr << 2;
    dynamic_simulation_B.R_k[dynamic_simulation_B.i] = dynamic_simulation_B.R_j
      [3 * dynamic_simulation_B.b_kstr];
    dynamic_simulation_B.R_k[dynamic_simulation_B.i + 1] =
      dynamic_simulation_B.R_j[3 * dynamic_simulation_B.b_kstr + 1];
    dynamic_simulation_B.R_k[dynamic_simulation_B.i + 2] =
      dynamic_simulation_B.R_j[3 * dynamic_simulation_B.b_kstr + 2];
    dynamic_simulation_B.R_k[dynamic_simulation_B.b_kstr + 12] =
      dynamic_simulation_B.R_d[dynamic_simulation_B.b_kstr + 6] *
      dynamic_simulation_B.T2[14] +
      (dynamic_simulation_B.R_d[dynamic_simulation_B.b_kstr + 3] *
       dynamic_simulation_B.T2[13] +
       dynamic_simulation_B.R_d[dynamic_simulation_B.b_kstr] *
       dynamic_simulation_B.T2[12]);
  }

  dynamic_simulation_B.R_k[3] = 0.0;
  dynamic_simulation_B.R_k[7] = 0.0;
  dynamic_simulation_B.R_k[11] = 0.0;
  dynamic_simulation_B.R_k[15] = 1.0;
  for (dynamic_simulation_B.b_kstr = 0; dynamic_simulation_B.b_kstr < 4;
       dynamic_simulation_B.b_kstr++) {
    for (dynamic_simulation_B.n_a = 0; dynamic_simulation_B.n_a < 4;
         dynamic_simulation_B.n_a++) {
      dynamic_simulation_B.i = dynamic_simulation_B.b_kstr << 2;
      dynamic_simulation_B.rtb_MATLABSystem_tmp = dynamic_simulation_B.n_a +
        dynamic_simulation_B.i;
      dynamic_simulation_B.T2[dynamic_simulation_B.rtb_MATLABSystem_tmp] = 0.0;
      dynamic_simulation_B.T2[dynamic_simulation_B.rtb_MATLABSystem_tmp] +=
        dynamic_simulation_B.T1[dynamic_simulation_B.i] *
        dynamic_simulation_B.R_k[dynamic_simulation_B.n_a];
      dynamic_simulation_B.T2[dynamic_simulation_B.rtb_MATLABSystem_tmp] +=
        dynamic_simulation_B.T1[dynamic_simulation_B.i + 1] *
        dynamic_simulation_B.R_k[dynamic_simulation_B.n_a + 4];
      dynamic_simulation_B.T2[dynamic_simulation_B.rtb_MATLABSystem_tmp] +=
        dynamic_simulation_B.T1[dynamic_simulation_B.i + 2] *
        dynamic_simulation_B.R_k[dynamic_simulation_B.n_a + 8];
      dynamic_simulation_B.T2[dynamic_simulation_B.rtb_MATLABSystem_tmp] +=
        dynamic_simulation_B.T1[dynamic_simulation_B.i + 3] *
        dynamic_simulation_B.R_k[dynamic_simulation_B.n_a + 12];
    }
  }

  // MATLABSystem: '<S21>/Get Parameter2'
  ParamGet_dynamic_simulation_135.get_parameter(&dynamic_simulation_B.vNum);

  // MATLABSystem: '<S21>/Get Parameter3'
  ParamGet_dynamic_simulation_136.get_parameter(&dynamic_simulation_B.bid1);

  // MATLABSystem: '<S21>/Get Parameter8'
  ParamGet_dynamic_simulation_141.get_parameter(&dynamic_simulation_B.j);

  // MATLABSystem: '<S21>/Get Parameter9'
  ParamGet_dynamic_simulation_142.get_parameter(&dynamic_simulation_B.K23);

  // MATLABSystem: '<S21>/Get Parameter10'
  ParamGet_dynamic_simulation_133.get_parameter(&dynamic_simulation_B.K24);

  // MATLABSystem: '<S21>/Get Parameter11'
  ParamGet_dynamic_simulation_134.get_parameter(&dynamic_simulation_B.K34);

  // Integrator: '<S10>/Velocity' incorporates:
  //   MATLABSystem: '<S21>/Get Parameter10'
  //   MATLABSystem: '<S21>/Get Parameter11'
  //   MATLABSystem: '<S21>/Get Parameter2'
  //   MATLABSystem: '<S21>/Get Parameter3'
  //   MATLABSystem: '<S21>/Get Parameter8'
  //   MATLABSystem: '<S21>/Get Parameter9'

  if (dynamic_simulation_DW.Velocity_IWORK != 0) {
    dynamic_simulation_X.Velocity_CSTATE[0] = dynamic_simulation_B.vNum;
    dynamic_simulation_X.Velocity_CSTATE[1] = dynamic_simulation_B.bid1;
    dynamic_simulation_X.Velocity_CSTATE[2] = dynamic_simulation_B.j;
    dynamic_simulation_X.Velocity_CSTATE[3] = dynamic_simulation_B.K23;
    dynamic_simulation_X.Velocity_CSTATE[4] = dynamic_simulation_B.K24;
    dynamic_simulation_X.Velocity_CSTATE[5] = dynamic_simulation_B.K34;
  }

  for (dynamic_simulation_B.i = 0; dynamic_simulation_B.i < 6;
       dynamic_simulation_B.i++) {
    dynamic_simulation_B.Velocity[dynamic_simulation_B.i] =
      dynamic_simulation_X.Velocity_CSTATE[dynamic_simulation_B.i];
  }

  // End of Integrator: '<S10>/Velocity'

  // Product: '<S2>/MatrixMultiply' incorporates:
  //   MATLABSystem: '<S14>/MATLAB System'

  for (dynamic_simulation_B.b_kstr = 0; dynamic_simulation_B.b_kstr < 6;
       dynamic_simulation_B.b_kstr++) {
    dynamic_simulation_B.MatrixMultiply[dynamic_simulation_B.b_kstr] = 0.0;
    for (dynamic_simulation_B.n_a = 0; dynamic_simulation_B.n_a < 6;
         dynamic_simulation_B.n_a++) {
      dynamic_simulation_B.vNum = b->data[6 * dynamic_simulation_B.n_a +
        dynamic_simulation_B.b_kstr] *
        dynamic_simulation_B.Velocity[dynamic_simulation_B.n_a] +
        dynamic_simulation_B.MatrixMultiply[dynamic_simulation_B.b_kstr];
      dynamic_simulation_B.MatrixMultiply[dynamic_simulation_B.b_kstr] =
        dynamic_simulation_B.vNum;
    }
  }

  // End of Product: '<S2>/MatrixMultiply'
  dynamic_simulati_emxFree_real_T(&b);
  if (rtmIsMajorTimeStep(dynamic_simulation_M)) {
    // Outputs for Atomic SubSystem: '<Root>/Subscribe'
    // MATLABSystem: '<S11>/SourceBlock' incorporates:
    //   Inport: '<S22>/In1'

    dynamic_simulat_SystemCore_step(&dynamic_simulation_B.b_bool,
      dynamic_simulation_B.b_varargout_2_Data,
      &dynamic_simulation_B.b_varargout_2_Data_SL_Info_Curr,
      &dynamic_simulation_B.b_varargout_2_Data_SL_Info_Rece,
      &dynamic_simulation_B.b_varargout_2_Layout_DataOffset,
      dynamic_simulation_B.b_varargout_2_Layout_Dim,
      &dynamic_simulation_B.b_varargout_2_Layout_Dim_SL_Inf,
      &dynamic_simulation_B.b_varargout_2_Layout_Dim_SL_I_c);

    // Outputs for Enabled SubSystem: '<S11>/Enabled Subsystem' incorporates:
    //   EnablePort: '<S22>/Enable'

    if (dynamic_simulation_B.b_bool) {
      memcpy(&dynamic_simulation_B.In1.Data[0],
             &dynamic_simulation_B.b_varargout_2_Data[0], sizeof(real_T) << 7U);
      dynamic_simulation_B.In1.Data_SL_Info.CurrentLength =
        dynamic_simulation_B.b_varargout_2_Data_SL_Info_Curr;
      dynamic_simulation_B.In1.Data_SL_Info.ReceivedLength =
        dynamic_simulation_B.b_varargout_2_Data_SL_Info_Rece;
      dynamic_simulation_B.In1.Layout.DataOffset =
        dynamic_simulation_B.b_varargout_2_Layout_DataOffset;
      memcpy(&dynamic_simulation_B.In1.Layout.Dim[0],
             &dynamic_simulation_B.b_varargout_2_Layout_Dim[0], sizeof
             (SL_Bus_dynamic_simulation_std_msgs_MultiArrayDimension) << 4U);
      dynamic_simulation_B.In1.Layout.Dim_SL_Info.CurrentLength =
        dynamic_simulation_B.b_varargout_2_Layout_Dim_SL_Inf;
      dynamic_simulation_B.In1.Layout.Dim_SL_Info.ReceivedLength =
        dynamic_simulation_B.b_varargout_2_Layout_Dim_SL_I_c;
    }

    // End of MATLABSystem: '<S11>/SourceBlock'
    // End of Outputs for SubSystem: '<S11>/Enabled Subsystem'
    // End of Outputs for SubSystem: '<Root>/Subscribe'
  }

  dynamic_simulati_emxInit_real_T(&L, 2);
  dynamic_simulati_emxInit_real_T(&lambda, 2);
  dynamic_simulati_emxInit_real_T(&tmp, 1);

  // MATLABSystem: '<S20>/MATLAB System' incorporates:
  //   Constant: '<S10>/Constant'
  //   Integrator: '<S10>/Position'

  obj_2 = &dynamic_simulation_DW.obj;
  RigidBodyTreeDynamics_massMatri(&dynamic_simulation_DW.obj.TreeInternal,
    dynamic_simulation_X.Position_CSTATE, L, lambda);
  dynamic_simulation_B.vNum = obj_2->TreeInternal.VelocityNumber;
  dynamic_simulation_B.rtb_MATLABSystem_tmp = static_cast<int32_T>
    (dynamic_simulation_B.vNum);
  dynamic_simulation_B.b_kstr = tmp->size[0];
  tmp->size[0] = dynamic_simulation_B.rtb_MATLABSystem_tmp;
  dynami_emxEnsureCapacity_real_T(tmp, dynamic_simulation_B.b_kstr);
  for (dynamic_simulation_B.b_kstr = 0; dynamic_simulation_B.b_kstr <
       dynamic_simulation_B.rtb_MATLABSystem_tmp; dynamic_simulation_B.b_kstr++)
  {
    tmp->data[dynamic_simulation_B.b_kstr] = 0.0;
  }

  RigidBodyTreeDynamics_inverseDy(&obj_2->TreeInternal,
    dynamic_simulation_X.Position_CSTATE, dynamic_simulation_B.Velocity,
    dynamic_simulation_P.Constant_Value_ij, dynamic_simulation_B.MATLABSystem);
  dynamic_simulati_emxFree_real_T(&tmp);

  // MATLABSystem: '<S20>/MATLAB System'
  for (dynamic_simulation_B.b_kstr = 0; dynamic_simulation_B.b_kstr < 6;
       dynamic_simulation_B.b_kstr++) {
    dynamic_simulation_B.MATLABSystem[dynamic_simulation_B.b_kstr] =
      dynamic_simulation_B.In1.Data[dynamic_simulation_B.b_kstr + 1] -
      dynamic_simulation_B.MATLABSystem[dynamic_simulation_B.b_kstr];
  }

  if ((L->size[0] == 0) || (L->size[1] == 0)) {
    dynamic_simulation_B.u1 = 0;
  } else {
    dynamic_simulation_B.i = L->size[0];
    dynamic_simulation_B.u1 = L->size[1];
    if (dynamic_simulation_B.i > dynamic_simulation_B.u1) {
      dynamic_simulation_B.u1 = dynamic_simulation_B.i;
    }
  }

  dynamic_simulati_emxInit_real_T(&H, 2);

  // MATLABSystem: '<S20>/MATLAB System'
  dynamic_simulation_B.b_kstr = H->size[0] * H->size[1];
  H->size[0] = L->size[0];
  H->size[1] = L->size[1];
  dynami_emxEnsureCapacity_real_T(H, dynamic_simulation_B.b_kstr);
  dynamic_simulation_B.n_a = L->size[0] * L->size[1] - 1;
  for (dynamic_simulation_B.b_kstr = 0; dynamic_simulation_B.b_kstr <=
       dynamic_simulation_B.n_a; dynamic_simulation_B.b_kstr++) {
    H->data[dynamic_simulation_B.b_kstr] = L->data[dynamic_simulation_B.b_kstr];
  }

  dynamic_simulation_B.iend = static_cast<int32_T>(((-1.0 - static_cast<real_T>
    (dynamic_simulation_B.u1)) + 1.0) / -1.0) - 1;
  for (dynamic_simulation_B.i = 0; dynamic_simulation_B.i <=
       dynamic_simulation_B.iend; dynamic_simulation_B.i++) {
    dynamic_simulation_B.j = static_cast<real_T>(dynamic_simulation_B.u1) + -
      static_cast<real_T>(dynamic_simulation_B.i);
    dynamic_simulation_B.b_kstr = static_cast<int32_T>(dynamic_simulation_B.j);
    dynamic_simulation_B.n_a = dynamic_simulation_B.b_kstr - 1;
    H->data[(static_cast<int32_T>(dynamic_simulation_B.j) + H->size[0] * (
              static_cast<int32_T>(dynamic_simulation_B.j) - 1)) - 1] = sqrt
      (H->data[(dynamic_simulation_B.n_a * H->size[0] +
                dynamic_simulation_B.b_kstr) - 1]);
    dynamic_simulation_B.bid1 = lambda->data[dynamic_simulation_B.n_a];
    while (dynamic_simulation_B.bid1 > 0.0) {
      dynamic_simulation_B.i_p = static_cast<int32_T>(dynamic_simulation_B.bid1)
        - 1;
      H->data[(static_cast<int32_T>(dynamic_simulation_B.j) + H->size[0] * (
                static_cast<int32_T>(dynamic_simulation_B.bid1) - 1)) - 1] =
        H->data[(dynamic_simulation_B.i_p * H->size[0] +
                 dynamic_simulation_B.b_kstr) - 1] / H->data
        [((static_cast<int32_T>(dynamic_simulation_B.j) - 1) * H->size[0] +
          static_cast<int32_T>(dynamic_simulation_B.j)) - 1];
      dynamic_simulation_B.bid1 = lambda->data[dynamic_simulation_B.i_p];
    }

    dynamic_simulation_B.bid1 = lambda->data[dynamic_simulation_B.n_a];
    while (dynamic_simulation_B.bid1 > 0.0) {
      dynamic_simulation_B.j = dynamic_simulation_B.bid1;
      while (dynamic_simulation_B.j > 0.0) {
        dynamic_simulation_B.n_a = static_cast<int32_T>(dynamic_simulation_B.j)
          - 1;
        H->data[(static_cast<int32_T>(dynamic_simulation_B.bid1) + H->size[0] *
                 (static_cast<int32_T>(dynamic_simulation_B.j) - 1)) - 1] =
          H->data[(dynamic_simulation_B.n_a * H->size[0] + static_cast<int32_T>
                   (dynamic_simulation_B.bid1)) - 1] - H->data
          [((static_cast<int32_T>(dynamic_simulation_B.bid1) - 1) * H->size[0] +
            dynamic_simulation_B.b_kstr) - 1] * H->data[((static_cast<int32_T>
          (dynamic_simulation_B.j) - 1) * H->size[0] +
          dynamic_simulation_B.b_kstr) - 1];
        dynamic_simulation_B.j = lambda->data[dynamic_simulation_B.n_a];
      }

      dynamic_simulation_B.bid1 = lambda->data[static_cast<int32_T>
        (dynamic_simulation_B.bid1) - 1];
    }
  }

  dynamic_simulation_B.b_kstr = L->size[0] * L->size[1];
  L->size[0] = H->size[0];
  L->size[1] = H->size[1];
  dynami_emxEnsureCapacity_real_T(L, dynamic_simulation_B.b_kstr);
  dynamic_simulation_B.n_a = H->size[0] * H->size[1] - 1;
  for (dynamic_simulation_B.b_kstr = 0; dynamic_simulation_B.b_kstr <=
       dynamic_simulation_B.n_a; dynamic_simulation_B.b_kstr++) {
    L->data[dynamic_simulation_B.b_kstr] = H->data[dynamic_simulation_B.b_kstr];
  }

  dynamic_simulation_B.n_a = H->size[1];
  if ((H->size[0] == 0) || (H->size[1] == 0) || (1 >= H->size[1])) {
  } else {
    dynamic_simulation_B.iend = 0;
    for (dynamic_simulation_B.b_kstr = 2; dynamic_simulation_B.b_kstr <=
         dynamic_simulation_B.n_a; dynamic_simulation_B.b_kstr++) {
      for (dynamic_simulation_B.i = 0; dynamic_simulation_B.i <=
           dynamic_simulation_B.iend; dynamic_simulation_B.i++) {
        L->data[dynamic_simulation_B.i + L->size[0] *
          (dynamic_simulation_B.b_kstr - 1)] = 0.0;
      }

      if (dynamic_simulation_B.iend + 1 < H->size[0]) {
        dynamic_simulation_B.iend++;
      }
    }
  }

  dynamic_simulati_emxFree_real_T(&H);

  // MATLABSystem: '<S20>/MATLAB System'
  dynamic_simulation_B.iend = static_cast<int32_T>(((-1.0 -
    dynamic_simulation_B.vNum) + 1.0) / -1.0) - 1;
  for (dynamic_simulation_B.i = 0; dynamic_simulation_B.i <=
       dynamic_simulation_B.iend; dynamic_simulation_B.i++) {
    dynamic_simulation_B.n_a = static_cast<int32_T>(dynamic_simulation_B.vNum +
      -static_cast<real_T>(dynamic_simulation_B.i));
    dynamic_simulation_B.b_kstr = dynamic_simulation_B.n_a - 1;
    dynamic_simulation_B.MATLABSystem[dynamic_simulation_B.b_kstr] /= L->data
      [(dynamic_simulation_B.b_kstr * L->size[0] + dynamic_simulation_B.n_a) - 1];
    dynamic_simulation_B.j = lambda->data[dynamic_simulation_B.b_kstr];
    while (dynamic_simulation_B.j > 0.0) {
      dynamic_simulation_B.u1 = static_cast<int32_T>(dynamic_simulation_B.j) - 1;
      dynamic_simulation_B.MATLABSystem[dynamic_simulation_B.u1] -= L->data
        [(dynamic_simulation_B.u1 * L->size[0] + dynamic_simulation_B.n_a) - 1] *
        dynamic_simulation_B.MATLABSystem[dynamic_simulation_B.b_kstr];
      dynamic_simulation_B.j = lambda->data[dynamic_simulation_B.u1];
    }
  }

  dynamic_simulation_B.n_a = dynamic_simulation_B.rtb_MATLABSystem_tmp - 1;
  for (dynamic_simulation_B.i = 0; dynamic_simulation_B.i <=
       dynamic_simulation_B.n_a; dynamic_simulation_B.i++) {
    dynamic_simulation_B.j = lambda->data[dynamic_simulation_B.i];
    while (dynamic_simulation_B.j > 0.0) {
      dynamic_simulation_B.b_kstr = static_cast<int32_T>(dynamic_simulation_B.j)
        - 1;
      dynamic_simulation_B.MATLABSystem[dynamic_simulation_B.i] -= L->
        data[dynamic_simulation_B.b_kstr * L->size[0] + dynamic_simulation_B.i] *
        dynamic_simulation_B.MATLABSystem[dynamic_simulation_B.b_kstr];
      dynamic_simulation_B.j = lambda->data[dynamic_simulation_B.b_kstr];
    }

    dynamic_simulation_B.MATLABSystem[dynamic_simulation_B.i] /= L->data[L->
      size[0] * dynamic_simulation_B.i + dynamic_simulation_B.i];
  }

  dynamic_simulati_emxFree_real_T(&lambda);
  dynamic_simulati_emxFree_real_T(&L);

  // MATLAB Function: '<S2>/MATLAB Function'
  dynamic_simulation_B.vNum = dynamic_simulation_B.T2[4] +
    dynamic_simulation_B.T2[1];
  dynamic_simulation_B.bid1 = dynamic_simulation_B.T2[8] +
    dynamic_simulation_B.T2[2];
  dynamic_simulation_B.j = dynamic_simulation_B.T2[6] - dynamic_simulation_B.T2
    [9];
  dynamic_simulation_B.K23 = dynamic_simulation_B.T2[9] +
    dynamic_simulation_B.T2[6];
  dynamic_simulation_B.K24 = dynamic_simulation_B.T2[8] -
    dynamic_simulation_B.T2[2];
  dynamic_simulation_B.K34 = dynamic_simulation_B.T2[1] -
    dynamic_simulation_B.T2[4];
  dynamic_simulation_B.T1[0] = ((dynamic_simulation_B.T2[0] -
    dynamic_simulation_B.T2[5]) - dynamic_simulation_B.T2[10]) / 3.0;
  dynamic_simulation_B.T1[4] = dynamic_simulation_B.vNum / 3.0;
  dynamic_simulation_B.T1[8] = dynamic_simulation_B.bid1 / 3.0;
  dynamic_simulation_B.T1[12] = dynamic_simulation_B.j / 3.0;
  dynamic_simulation_B.T1[1] = dynamic_simulation_B.vNum / 3.0;
  dynamic_simulation_B.T1[5] = ((dynamic_simulation_B.T2[5] -
    dynamic_simulation_B.T2[0]) - dynamic_simulation_B.T2[10]) / 3.0;
  dynamic_simulation_B.T1[9] = dynamic_simulation_B.K23 / 3.0;
  dynamic_simulation_B.T1[13] = dynamic_simulation_B.K24 / 3.0;
  dynamic_simulation_B.T1[2] = dynamic_simulation_B.bid1 / 3.0;
  dynamic_simulation_B.T1[6] = dynamic_simulation_B.K23 / 3.0;
  dynamic_simulation_B.T1[10] = ((dynamic_simulation_B.T2[10] -
    dynamic_simulation_B.T2[0]) - dynamic_simulation_B.T2[5]) / 3.0;
  dynamic_simulation_B.T1[14] = dynamic_simulation_B.K34 / 3.0;
  dynamic_simulation_B.T1[3] = dynamic_simulation_B.j / 3.0;
  dynamic_simulation_B.T1[7] = dynamic_simulation_B.K24 / 3.0;
  dynamic_simulation_B.T1[11] = dynamic_simulation_B.K34 / 3.0;
  dynamic_simulation_B.T1[15] = ((dynamic_simulation_B.T2[0] +
    dynamic_simulation_B.T2[5]) + dynamic_simulation_B.T2[10]) / 3.0;
  dynamic_simulation_eig(dynamic_simulation_B.T1, dynamic_simulation_B.eigVec,
    dynamic_simulation_B.eigVal);
  dynamic_simulation_B.cartOrn[0] = dynamic_simulation_B.eigVal[0].re;
  dynamic_simulation_B.cartOrn[1] = dynamic_simulation_B.eigVal[1].re;
  dynamic_simulation_B.cartOrn[2] = dynamic_simulation_B.eigVal[2].re;
  dynamic_simulation_B.cartOrn[3] = dynamic_simulation_B.eigVal[3].re;
  if (!rtIsNaN(dynamic_simulation_B.eigVal[0].re)) {
    dynamic_simulation_B.i = 1;
  } else {
    dynamic_simulation_B.i = 0;
    dynamic_simulation_B.b_kstr = 2;
    exitg2 = false;
    while ((!exitg2) && (dynamic_simulation_B.b_kstr < 5)) {
      if (!rtIsNaN(dynamic_simulation_B.cartOrn[dynamic_simulation_B.b_kstr - 1]))
      {
        dynamic_simulation_B.i = dynamic_simulation_B.b_kstr;
        exitg2 = true;
      } else {
        dynamic_simulation_B.b_kstr++;
      }
    }
  }

  if (dynamic_simulation_B.i != 0) {
    dynamic_simulation_B.vNum =
      dynamic_simulation_B.cartOrn[dynamic_simulation_B.i - 1];
    dynamic_simulation_B.b_kstr = dynamic_simulation_B.i - 1;
    while (dynamic_simulation_B.i + 1 < 5) {
      if (dynamic_simulation_B.vNum <
          dynamic_simulation_B.cartOrn[dynamic_simulation_B.i]) {
        dynamic_simulation_B.vNum =
          dynamic_simulation_B.cartOrn[dynamic_simulation_B.i];
        dynamic_simulation_B.b_kstr = dynamic_simulation_B.i;
      }

      dynamic_simulation_B.i++;
    }

    dynamic_simulation_B.i = dynamic_simulation_B.b_kstr;
  }

  dynamic_simulation_B.i <<= 2;
  dynamic_simulation_B.b_kstr = dynamic_simulation_B.i + 3;
  dynamic_simulation_B.cartOrn[0] =
    dynamic_simulation_B.eigVec[dynamic_simulation_B.b_kstr].re;
  dynamic_simulation_B.cartOrn[1] =
    dynamic_simulation_B.eigVec[dynamic_simulation_B.i].re;
  dynamic_simulation_B.n_a = dynamic_simulation_B.i + 1;
  dynamic_simulation_B.cartOrn[2] =
    dynamic_simulation_B.eigVec[dynamic_simulation_B.n_a].re;
  dynamic_simulation_B.rtb_MATLABSystem_tmp = dynamic_simulation_B.i + 2;
  dynamic_simulation_B.cartOrn[3] =
    dynamic_simulation_B.eigVec[dynamic_simulation_B.rtb_MATLABSystem_tmp].re;
  if (dynamic_simulation_B.eigVec[dynamic_simulation_B.b_kstr].re < 0.0) {
    dynamic_simulation_B.cartOrn[0] =
      -dynamic_simulation_B.eigVec[dynamic_simulation_B.b_kstr].re;
    dynamic_simulation_B.cartOrn[1] =
      -dynamic_simulation_B.eigVec[dynamic_simulation_B.i].re;
    dynamic_simulation_B.cartOrn[2] =
      -dynamic_simulation_B.eigVec[dynamic_simulation_B.n_a].re;
    dynamic_simulation_B.cartOrn[3] =
      -dynamic_simulation_B.eigVec[dynamic_simulation_B.rtb_MATLABSystem_tmp].re;
  }

  // Clock: '<Root>/Clock1'
  dynamic_simulation_B.bid1 = dynamic_simulation_M->Timing.t[0];

  // MATLAB Function: '<Root>/MATLAB Function'
  if (dynamic_simulation_B.bid1 < 0.0) {
    dynamic_simulation_B.vNum = ceil(dynamic_simulation_B.bid1);
  } else {
    dynamic_simulation_B.vNum = floor(dynamic_simulation_B.bid1);
  }

  dynamic_simulation_B.bid1 = (dynamic_simulation_B.bid1 -
    dynamic_simulation_B.vNum) * 1.0E+9;
  if (dynamic_simulation_B.bid1 < 0.0) {
    dynamic_simulation_B.bid1 = ceil(dynamic_simulation_B.bid1);
  } else {
    dynamic_simulation_B.bid1 = floor(dynamic_simulation_B.bid1);
  }

  // BusAssignment: '<S1>/Bus Assignment' incorporates:
  //   BusCreator generated from: '<S1>/Bus Assignment'
  //   Constant: '<S12>/Constant'
  //   MATLAB Function: '<Root>/MATLAB Function'
  //   MATLAB Function: '<S2>/MATLAB Function'

  dynamic_simulation_B.BusAssignment_b = dynamic_simulation_P.Constant_Value_l;
  dynamic_simulation_B.BusAssignment_b.Header.Stamp.Sec =
    dynamic_simulation_B.vNum;
  dynamic_simulation_B.BusAssignment_b.Header.Stamp.Nsec =
    dynamic_simulation_B.bid1;
  dynamic_simulation_B.BusAssignment_b.Pose.Position.X =
    dynamic_simulation_B.T2[12];
  dynamic_simulation_B.BusAssignment_b.Pose.Position.Y =
    dynamic_simulation_B.T2[13];
  dynamic_simulation_B.BusAssignment_b.Pose.Position.Z =
    dynamic_simulation_B.T2[14];
  dynamic_simulation_B.BusAssignment_b.Pose.Orientation.X =
    dynamic_simulation_B.cartOrn[0];
  dynamic_simulation_B.BusAssignment_b.Pose.Orientation.Y =
    dynamic_simulation_B.cartOrn[1];
  dynamic_simulation_B.BusAssignment_b.Pose.Orientation.Z =
    dynamic_simulation_B.cartOrn[2];
  dynamic_simulation_B.BusAssignment_b.Pose.Orientation.W =
    dynamic_simulation_B.cartOrn[3];

  // Outputs for Atomic SubSystem: '<Root>/Publish2'
  // MATLABSystem: '<S8>/SinkBlock'
  Pub_dynamic_simulation_178.publish(&dynamic_simulation_B.BusAssignment_b);

  // End of Outputs for SubSystem: '<Root>/Publish2'

  // BusAssignment: '<Root>/Bus Assignment' incorporates:
  //   BusCreator generated from: '<Root>/Bus Assignment'
  //   MATLAB Function: '<Root>/MATLAB Function'

  dynamic_simulation_B.BusAssignment_o.Clock_.Sec = dynamic_simulation_B.vNum;
  dynamic_simulation_B.BusAssignment_o.Clock_.Nsec = dynamic_simulation_B.bid1;

  // Outputs for Atomic SubSystem: '<Root>/Publish1'
  // MATLABSystem: '<S7>/SinkBlock'
  Pub_dynamic_simulation_158.publish(&dynamic_simulation_B.BusAssignment_o);

  // End of Outputs for SubSystem: '<Root>/Publish1'

  // BusAssignment: '<S1>/Bus Assignment1' incorporates:
  //   BusCreator generated from: '<S1>/Bus Assignment1'
  //   Constant: '<S13>/Constant'
  //   MATLAB Function: '<Root>/MATLAB Function'
  //   MATLAB Function: '<S2>/MATLAB Function'

  dynamic_simulation_B.BusAssignment1 = dynamic_simulation_P.Constant_Value_a;
  dynamic_simulation_B.BusAssignment1.Header.Stamp.Sec =
    dynamic_simulation_B.vNum;
  dynamic_simulation_B.BusAssignment1.Header.Stamp.Nsec =
    dynamic_simulation_B.bid1;
  dynamic_simulation_B.BusAssignment1.Twist.Linear.X =
    dynamic_simulation_B.MatrixMultiply[3];
  dynamic_simulation_B.BusAssignment1.Twist.Linear.Y =
    dynamic_simulation_B.MatrixMultiply[4];
  dynamic_simulation_B.BusAssignment1.Twist.Linear.Z =
    dynamic_simulation_B.MatrixMultiply[5];
  dynamic_simulation_B.BusAssignment1.Twist.Angular.X =
    dynamic_simulation_B.MatrixMultiply[0];
  dynamic_simulation_B.BusAssignment1.Twist.Angular.Y =
    dynamic_simulation_B.MatrixMultiply[1];
  dynamic_simulation_B.BusAssignment1.Twist.Angular.Z =
    dynamic_simulation_B.MatrixMultiply[2];

  // Outputs for Atomic SubSystem: '<Root>/Publish3'
  // MATLABSystem: '<S9>/SinkBlock'
  Pub_dynamic_simulation_179.publish(&dynamic_simulation_B.BusAssignment1);

  // End of Outputs for SubSystem: '<Root>/Publish3'
  for (dynamic_simulation_B.b_kstr = 0; dynamic_simulation_B.b_kstr < 32;
       dynamic_simulation_B.b_kstr++) {
    // SignalConversion generated from: '<S17>/ SFunction ' incorporates:
    //   MATLAB Function: '<S4>/Assign to JointState msg'
    //   MATLABSystem: '<S19>/Get Parameter'
    //   MATLABSystem: '<S19>/Get Parameter1'
    //   MATLABSystem: '<S19>/Get Parameter2'
    //   MATLABSystem: '<S19>/Get Parameter3'
    //   MATLABSystem: '<S19>/Get Parameter4'
    //   MATLABSystem: '<S19>/Get Parameter5'

    dynamic_simulation_B.TmpSignalConversionAtSFun_b[dynamic_simulation_B.b_kstr]
      = static_cast<uint8_T>
      (dynamic_simulation_B.charValue[dynamic_simulation_B.b_kstr]);
    dynamic_simulation_B.TmpSignalConversionAtSFun_b[dynamic_simulation_B.b_kstr
      + 32] = static_cast<uint8_T>
      (dynamic_simulation_B.charValue_f[dynamic_simulation_B.b_kstr]);
    dynamic_simulation_B.TmpSignalConversionAtSFun_b[dynamic_simulation_B.b_kstr
      + 64] = static_cast<uint8_T>
      (dynamic_simulation_B.charValue_a[dynamic_simulation_B.b_kstr]);
    dynamic_simulation_B.TmpSignalConversionAtSFun_b[dynamic_simulation_B.b_kstr
      + 96] = static_cast<uint8_T>
      (dynamic_simulation_B.charValue_j[dynamic_simulation_B.b_kstr]);
    dynamic_simulation_B.TmpSignalConversionAtSFun_b[dynamic_simulation_B.b_kstr
      + 128] = static_cast<uint8_T>
      (dynamic_simulation_B.charValue_jz[dynamic_simulation_B.b_kstr]);
    dynamic_simulation_B.TmpSignalConversionAtSFun_b[dynamic_simulation_B.b_kstr
      + 160] = static_cast<uint8_T>
      (dynamic_simulation_B.charValue_o[dynamic_simulation_B.b_kstr]);
  }

  // BusAssignment: '<S4>/Bus Assignment' incorporates:
  //   Constant: '<S18>/Constant'
  //   MATLAB Function: '<S4>/Assign to JointState msg'

  dynamic_simulation_B.BusAssignment = dynamic_simulation_P.Constant_Value;

  // MATLAB Function: '<S4>/Assign to JointState msg' incorporates:
  //   BusAssignment: '<S4>/Bus Assignment'
  //   Constant: '<S4>/Constant'
  //   Integrator: '<S10>/Position'

  dynamic_simulation_B.BusAssignment.Name_SL_Info.CurrentLength = 6U;
  dynamic_simulation_B.BusAssignment.Position_SL_Info.CurrentLength = 6U;
  dynamic_simulation_B.BusAssignment.Velocity_SL_Info.CurrentLength = 6U;
  for (dynamic_simulation_B.i = 0; dynamic_simulation_B.i < 6;
       dynamic_simulation_B.i++) {
    dynamic_simulation_B.j = ((static_cast<real_T>(dynamic_simulation_B.i) + 1.0)
      - 1.0) * static_cast<real_T>(dynamic_simulation_P.name_max_length);
    if (dynamic_simulation_B.j < 4.294967296E+9) {
      dynamic_simulation_B.b_varargout_2_Data_SL_Info_Rece =
        static_cast<uint32_T>(dynamic_simulation_B.j);
    } else {
      dynamic_simulation_B.b_varargout_2_Data_SL_Info_Rece = MAX_uint32_T;
    }

    dynamic_simulation_B.b_varargout_2_Data_SL_Info_Curr =
      dynamic_simulation_B.b_varargout_2_Data_SL_Info_Rece + /*MW:OvSatOk*/ 1U;
    if (dynamic_simulation_B.b_varargout_2_Data_SL_Info_Curr <
        dynamic_simulation_B.b_varargout_2_Data_SL_Info_Rece) {
      dynamic_simulation_B.b_varargout_2_Data_SL_Info_Curr = MAX_uint32_T;
    }

    dynamic_simulation_B.b_varargout_2_Data_SL_Info_Rece =
      dynamic_simulation_B.b_varargout_2_Data_SL_Info_Curr + /*MW:OvSatOk*/
      dynamic_simulation_B.TmpSignalConversionAtSFunct[dynamic_simulation_B.i];
    if (dynamic_simulation_B.b_varargout_2_Data_SL_Info_Rece <
        dynamic_simulation_B.b_varargout_2_Data_SL_Info_Curr) {
      dynamic_simulation_B.b_varargout_2_Data_SL_Info_Rece = MAX_uint32_T;
    }

    dynamic_simulation_B.b_varargout_2_Data_SL_Info_Rece--;
    if (dynamic_simulation_B.b_varargout_2_Data_SL_Info_Curr >
        dynamic_simulation_B.b_varargout_2_Data_SL_Info_Rece) {
      dynamic_simulation_B.rtb_MATLABSystem_tmp = 0;
      dynamic_simulation_B.n_a = 0;
    } else {
      dynamic_simulation_B.rtb_MATLABSystem_tmp = static_cast<int32_T>
        (dynamic_simulation_B.b_varargout_2_Data_SL_Info_Curr) - 1;
      dynamic_simulation_B.n_a = static_cast<int32_T>
        (dynamic_simulation_B.b_varargout_2_Data_SL_Info_Rece);
    }

    dynamic_simulation_B.n_a -= dynamic_simulation_B.rtb_MATLABSystem_tmp;
    for (dynamic_simulation_B.b_kstr = 0; dynamic_simulation_B.b_kstr <
         dynamic_simulation_B.n_a; dynamic_simulation_B.b_kstr++) {
      dynamic_simulation_B.BusAssignment.Name[dynamic_simulation_B.i]
        .Data[dynamic_simulation_B.b_kstr] =
        dynamic_simulation_B.TmpSignalConversionAtSFun_b[dynamic_simulation_B.rtb_MATLABSystem_tmp
        + dynamic_simulation_B.b_kstr];
    }

    dynamic_simulation_B.BusAssignment.Name[dynamic_simulation_B.i].
      Data_SL_Info.CurrentLength =
      dynamic_simulation_B.TmpSignalConversionAtSFunct[dynamic_simulation_B.i];
    dynamic_simulation_B.BusAssignment.Position[dynamic_simulation_B.i] =
      dynamic_simulation_X.Position_CSTATE[dynamic_simulation_B.i];
    dynamic_simulation_B.BusAssignment.Velocity[dynamic_simulation_B.i] =
      dynamic_simulation_B.Velocity[dynamic_simulation_B.i];
  }

  // BusAssignment: '<S4>/Bus Assignment' incorporates:
  //   BusCreator generated from: '<S4>/Bus Assignment'
  //   MATLAB Function: '<Root>/MATLAB Function'

  dynamic_simulation_B.BusAssignment.Header.Stamp.Sec =
    dynamic_simulation_B.vNum;
  dynamic_simulation_B.BusAssignment.Header.Stamp.Nsec =
    dynamic_simulation_B.bid1;

  // Outputs for Atomic SubSystem: '<Root>/Publish'
  // MATLABSystem: '<S6>/SinkBlock'
  Pub_dynamic_simulation_157.publish(&dynamic_simulation_B.BusAssignment);

  // End of Outputs for SubSystem: '<Root>/Publish'
  if (rtmIsMajorTimeStep(dynamic_simulation_M)) {
    // Update for Integrator: '<S10>/Position'
    dynamic_simulation_DW.Position_IWORK = 0;

    // Update for Integrator: '<S10>/Velocity'
    dynamic_simulation_DW.Velocity_IWORK = 0;
  }                                    // end MajorTimeStep

  if (rtmIsMajorTimeStep(dynamic_simulation_M)) {
    rt_ertODEUpdateContinuousStates(&dynamic_simulation_M->solverInfo);

    // Update absolute time for base rate
    // The "clockTick0" counts the number of times the code of this task has
    //  been executed. The absolute time is the multiplication of "clockTick0"
    //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
    //  overflow during the application lifespan selected.

    ++dynamic_simulation_M->Timing.clockTick0;
    dynamic_simulation_M->Timing.t[0] = rtsiGetSolverStopTime
      (&dynamic_simulation_M->solverInfo);

    {
      // Update absolute timer for sample time: [0.05s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.05, which is the step size
      //  of the task. Size of "clockTick1" ensures timer will not overflow during the
      //  application lifespan selected.

      dynamic_simulation_M->Timing.clockTick1++;
    }
  }                                    // end MajorTimeStep
}

// Derivatives for root system: '<Root>'
void dynamic_simulation_derivatives(void)
{
  int32_T i;
  XDot_dynamic_simulation_T *_rtXdot;
  _rtXdot = ((XDot_dynamic_simulation_T *) dynamic_simulation_M->derivs);
  for (i = 0; i < 6; i++) {
    // Derivatives for Integrator: '<S10>/Position'
    _rtXdot->Position_CSTATE[i] = dynamic_simulation_B.Velocity[i];

    // Derivatives for Integrator: '<S10>/Velocity'
    _rtXdot->Velocity_CSTATE[i] = dynamic_simulation_B.MATLABSystem[i];
  }
}

// Model initialize function
void dynamic_simulation_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&dynamic_simulation_M->solverInfo,
                          &dynamic_simulation_M->Timing.simTimeStep);
    rtsiSetTPtr(&dynamic_simulation_M->solverInfo, &rtmGetTPtr
                (dynamic_simulation_M));
    rtsiSetStepSizePtr(&dynamic_simulation_M->solverInfo,
                       &dynamic_simulation_M->Timing.stepSize0);
    rtsiSetdXPtr(&dynamic_simulation_M->solverInfo,
                 &dynamic_simulation_M->derivs);
    rtsiSetContStatesPtr(&dynamic_simulation_M->solverInfo, (real_T **)
                         &dynamic_simulation_M->contStates);
    rtsiSetNumContStatesPtr(&dynamic_simulation_M->solverInfo,
      &dynamic_simulation_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&dynamic_simulation_M->solverInfo,
      &dynamic_simulation_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&dynamic_simulation_M->solverInfo,
      &dynamic_simulation_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&dynamic_simulation_M->solverInfo,
      &dynamic_simulation_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&dynamic_simulation_M->solverInfo, (&rtmGetErrorStatus
      (dynamic_simulation_M)));
    rtsiSetRTModelPtr(&dynamic_simulation_M->solverInfo, dynamic_simulation_M);
  }

  rtsiSetSimTimeStep(&dynamic_simulation_M->solverInfo, MAJOR_TIME_STEP);
  dynamic_simulation_M->intgData.y = dynamic_simulation_M->odeY;
  dynamic_simulation_M->intgData.f[0] = dynamic_simulation_M->odeF[0];
  dynamic_simulation_M->intgData.f[1] = dynamic_simulation_M->odeF[1];
  dynamic_simulation_M->intgData.f[2] = dynamic_simulation_M->odeF[2];
  dynamic_simulation_M->contStates = ((X_dynamic_simulation_T *)
    &dynamic_simulation_X);
  rtsiSetSolverData(&dynamic_simulation_M->solverInfo, static_cast<void *>
                    (&dynamic_simulation_M->intgData));
  rtsiSetSolverName(&dynamic_simulation_M->solverInfo,"ode3");
  rtmSetTPtr(dynamic_simulation_M, &dynamic_simulation_M->Timing.tArray[0]);
  dynamic_simulation_M->Timing.stepSize0 = 0.05;
  rtmSetFirstInitCond(dynamic_simulation_M, 1);

  {
    char_T tmp[7];
    int32_T i;
    static const char_T tmp_0[13] = { '/', 'j', 'o', 'i', 'n', 't', '_', 't',
      'o', 'r', 'q', 'u', 'e' };

    static const char_T tmp_1[14] = { '/', 'c', 'a', 'r', 't', 'e', 's', 'i',
      'a', 'n', '_', 'p', 'o', 's' };

    static const char_T tmp_2[6] = { '/', 'c', 'l', 'o', 'c', 'k' };

    static const char_T tmp_3[14] = { '/', 'c', 'a', 'r', 't', 'e', 's', 'i',
      'a', 'n', '_', 'v', 'e', 'l' };

    static const char_T tmp_4[13] = { '/', 'j', 'o', 'i', 'n', 't', '_', 's',
      't', 'a', 't', 'e', 's' };

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

    static const char_T tmp_b[24] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      'l', 'e', 'r', '_', 'j', 'o', 'i', 'n', 't', '_', 'n', 'a', 'm', 'e', '_',
      '1' };

    static const char_T tmp_c[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n',
      't', '_', '1' };

    static const char_T tmp_d[24] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      'l', 'e', 'r', '_', 'j', 'o', 'i', 'n', 't', '_', 'n', 'a', 'm', 'e', '_',
      '2' };

    static const char_T tmp_e[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n',
      't', '_', '2' };

    static const char_T tmp_f[24] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      'l', 'e', 'r', '_', 'j', 'o', 'i', 'n', 't', '_', 'n', 'a', 'm', 'e', '_',
      '3' };

    static const char_T tmp_g[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n',
      't', '_', '3' };

    static const char_T tmp_h[24] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      'l', 'e', 'r', '_', 'j', 'o', 'i', 'n', 't', '_', 'n', 'a', 'm', 'e', '_',
      '4' };

    static const char_T tmp_i[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n',
      't', '_', '4' };

    static const char_T tmp_j[24] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      'l', 'e', 'r', '_', 'j', 'o', 'i', 'n', 't', '_', 'n', 'a', 'm', 'e', '_',
      '5' };

    static const char_T tmp_k[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n',
      't', '_', '5' };

    static const char_T tmp_l[24] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      'l', 'e', 'r', '_', 'j', 'o', 'i', 'n', 't', '_', 'n', 'a', 'm', 'e', '_',
      '6' };

    static const char_T tmp_m[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n',
      't', '_', '6' };

    static const char_T tmp_n[12] = { '/', 'q', 'v', '1', '_', 'i', 'n', 'i',
      't', 'i', 'a', 'l' };

    static const char_T tmp_o[12] = { '/', 'q', 'v', '2', '_', 'i', 'n', 'i',
      't', 'i', 'a', 'l' };

    static const char_T tmp_p[12] = { '/', 'q', 'v', '3', '_', 'i', 'n', 'i',
      't', 'i', 'a', 'l' };

    static const char_T tmp_q[12] = { '/', 'q', 'v', '4', '_', 'i', 'n', 'i',
      't', 'i', 'a', 'l' };

    static const char_T tmp_r[12] = { '/', 'q', 'v', '5', '_', 'i', 'n', 'i',
      't', 'i', 'a', 'l' };

    static const char_T tmp_s[12] = { '/', 'q', 'v', '6', '_', 'i', 'n', 'i',
      't', 'i', 'a', 'l' };

    // InitializeConditions for Integrator: '<S10>/Position' incorporates:
    //   Integrator: '<S10>/Velocity'

    if (rtmIsFirstInitCond(dynamic_simulation_M)) {
      dynamic_simulation_X.Position_CSTATE[0] = 0.0;
      dynamic_simulation_X.Position_CSTATE[1] = 0.0;
      dynamic_simulation_X.Position_CSTATE[2] = 0.0;
      dynamic_simulation_X.Position_CSTATE[3] = 0.0;
      dynamic_simulation_X.Position_CSTATE[4] = 0.0;
      dynamic_simulation_X.Position_CSTATE[5] = 0.0;
      dynamic_simulation_X.Velocity_CSTATE[0] = 0.0;
      dynamic_simulation_X.Velocity_CSTATE[1] = 0.0;
      dynamic_simulation_X.Velocity_CSTATE[2] = 0.0;
      dynamic_simulation_X.Velocity_CSTATE[3] = 0.0;
      dynamic_simulation_X.Velocity_CSTATE[4] = 0.0;
      dynamic_simulation_X.Velocity_CSTATE[5] = 0.0;
    }

    dynamic_simulation_DW.Position_IWORK = 1;

    // End of InitializeConditions for Integrator: '<S10>/Position'

    // InitializeConditions for Integrator: '<S10>/Velocity'
    dynamic_simulation_DW.Velocity_IWORK = 1;

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S11>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S22>/Out1'
    dynamic_simulation_B.In1 = dynamic_simulation_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S11>/Enabled Subsystem'

    // Start for MATLABSystem: '<S11>/SourceBlock'
    dynamic_simulation_DW.obj_fk.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_fk.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      dynamic_simulation_B.cv2[i] = tmp_0[i];
    }

    dynamic_simulation_B.cv2[13] = '\x00';
    Sub_dynamic_simulation_160.createSubscriber(dynamic_simulation_B.cv2, 1);
    dynamic_simulation_DW.obj_fk.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish2'
    // Start for MATLABSystem: '<S8>/SinkBlock'
    dynamic_simulation_DW.obj_g.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_g.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      dynamic_simulation_B.cv1[i] = tmp_1[i];
    }

    dynamic_simulation_B.cv1[14] = '\x00';
    Pub_dynamic_simulation_178.createPublisher(dynamic_simulation_B.cv1, 1);
    dynamic_simulation_DW.obj_g.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S8>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish2'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish1'
    // Start for MATLABSystem: '<S7>/SinkBlock'
    dynamic_simulation_DW.obj_iv.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_iv.isInitialized = 1;
    for (i = 0; i < 6; i++) {
      tmp[i] = tmp_2[i];
    }

    tmp[6] = '\x00';
    Pub_dynamic_simulation_158.createPublisher(tmp, 1);
    dynamic_simulation_DW.obj_iv.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S7>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish1'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish3'
    // Start for MATLABSystem: '<S9>/SinkBlock'
    dynamic_simulation_DW.obj_dj.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_dj.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      dynamic_simulation_B.cv1[i] = tmp_3[i];
    }

    dynamic_simulation_B.cv1[14] = '\x00';
    Pub_dynamic_simulation_179.createPublisher(dynamic_simulation_B.cv1, 1);
    dynamic_simulation_DW.obj_dj.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S9>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish3'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish'
    // Start for MATLABSystem: '<S6>/SinkBlock'
    dynamic_simulation_DW.obj_cd.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_cd.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      dynamic_simulation_B.cv2[i] = tmp_4[i];
    }

    dynamic_simulation_B.cv2[13] = '\x00';
    Pub_dynamic_simulation_157.createPublisher(dynamic_simulation_B.cv2, 1);
    dynamic_simulation_DW.obj_cd.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish'

    // Start for MATLABSystem: '<S21>/Get Parameter12'
    dynamic_simulation_DW.obj_l.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_l.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      dynamic_simulation_B.cv4[i] = tmp_5[i];
    }

    dynamic_simulation_B.cv4[11] = '\x00';
    ParamGet_dynamic_simulation_194.initialize(dynamic_simulation_B.cv4);
    ParamGet_dynamic_simulation_194.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynamic_simulation_194.set_initial_value(0.0);
    dynamic_simulation_DW.obj_l.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S21>/Get Parameter12'

    // Start for MATLABSystem: '<S21>/Get Parameter1'
    dynamic_simulation_DW.obj_lm.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_lm.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      dynamic_simulation_B.cv4[i] = tmp_6[i];
    }

    dynamic_simulation_B.cv4[11] = '\x00';
    ParamGet_dynamic_simulation_132.initialize(dynamic_simulation_B.cv4);
    ParamGet_dynamic_simulation_132.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynamic_simulation_132.set_initial_value(0.0);
    dynamic_simulation_DW.obj_lm.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S21>/Get Parameter1'

    // Start for MATLABSystem: '<S21>/Get Parameter4'
    dynamic_simulation_DW.obj_a.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_a.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      dynamic_simulation_B.cv4[i] = tmp_7[i];
    }

    dynamic_simulation_B.cv4[11] = '\x00';
    ParamGet_dynamic_simulation_137.initialize(dynamic_simulation_B.cv4);
    ParamGet_dynamic_simulation_137.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynamic_simulation_137.set_initial_value(0.0);
    dynamic_simulation_DW.obj_a.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S21>/Get Parameter4'

    // Start for MATLABSystem: '<S21>/Get Parameter5'
    dynamic_simulation_DW.obj_d.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_d.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      dynamic_simulation_B.cv4[i] = tmp_8[i];
    }

    dynamic_simulation_B.cv4[11] = '\x00';
    ParamGet_dynamic_simulation_138.initialize(dynamic_simulation_B.cv4);
    ParamGet_dynamic_simulation_138.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynamic_simulation_138.set_initial_value(0.0);
    dynamic_simulation_DW.obj_d.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S21>/Get Parameter5'

    // Start for MATLABSystem: '<S21>/Get Parameter6'
    dynamic_simulation_DW.obj_h.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_h.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      dynamic_simulation_B.cv4[i] = tmp_9[i];
    }

    dynamic_simulation_B.cv4[11] = '\x00';
    ParamGet_dynamic_simulation_139.initialize(dynamic_simulation_B.cv4);
    ParamGet_dynamic_simulation_139.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynamic_simulation_139.set_initial_value(0.0);
    dynamic_simulation_DW.obj_h.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S21>/Get Parameter6'

    // Start for MATLABSystem: '<S21>/Get Parameter7'
    dynamic_simulation_DW.obj_f0.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_f0.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      dynamic_simulation_B.cv4[i] = tmp_a[i];
    }

    dynamic_simulation_B.cv4[11] = '\x00';
    ParamGet_dynamic_simulation_140.initialize(dynamic_simulation_B.cv4);
    ParamGet_dynamic_simulation_140.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynamic_simulation_140.set_initial_value(0.0);
    dynamic_simulation_DW.obj_f0.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S21>/Get Parameter7'
    emxInitStruct_robotics_slmanip_(&dynamic_simulation_DW.obj_f);
    emxInitStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_1_m);
    emxInitStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_16_a);
    emxInitStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_15_c);
    emxInitStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_14_o);
    emxInitStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_13_i);
    emxInitStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_12_d);
    emxInitStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_11_c);
    emxInitStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_10_l);
    emxInitStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_9_p);
    emxInitStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_8_i);
    emxInitStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_7_o);
    emxInitStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_6_a);
    emxInitStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_5_a);
    emxInitStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_4_g);
    emxInitStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_3_g);
    emxInitStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_2_o);

    // Start for MATLABSystem: '<S14>/MATLAB System'
    dynamic_simulation_DW.obj_f.isInitialized = 0;
    dynamic_simulation_DW.obj_f.isInitialized = 1;
    dyn_RigidBodyTree_RigidBodyTree(&dynamic_simulation_DW.obj_f.TreeInternal,
      &dynamic_simulation_DW.gobj_2_o, &dynamic_simulation_DW.gobj_4_g,
      &dynamic_simulation_DW.gobj_5_a, &dynamic_simulation_DW.gobj_6_a,
      &dynamic_simulation_DW.gobj_7_o, &dynamic_simulation_DW.gobj_8_i,
      &dynamic_simulation_DW.gobj_9_p, &dynamic_simulation_DW.gobj_3_g);

    // Start for MATLABSystem: '<S19>/Get Parameter'
    dynamic_simulation_DW.obj_m.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_m.isInitialized = 1;
    for (i = 0; i < 24; i++) {
      dynamic_simulation_B.cv[i] = tmp_b[i];
    }

    dynamic_simulation_B.cv[24] = '\x00';
    ParamGet_dynamic_simulation_224.initialize(dynamic_simulation_B.cv);
    ParamGet_dynamic_simulation_224.initialize_error_codes(0, 1, 2, 3);
    for (i = 0; i < 11; i++) {
      dynamic_simulation_B.initialValue[i] = tmp_c[i];
    }

    ParamGet_dynamic_simulation_224.set_initial_value
      (dynamic_simulation_B.initialValue, 11U);
    dynamic_simulation_DW.obj_m.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S19>/Get Parameter'

    // Start for MATLABSystem: '<S19>/Get Parameter1'
    dynamic_simulation_DW.obj_c.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_c.isInitialized = 1;
    for (i = 0; i < 24; i++) {
      dynamic_simulation_B.cv[i] = tmp_d[i];
    }

    dynamic_simulation_B.cv[24] = '\x00';
    ParamGet_dynamic_simulation_228.initialize(dynamic_simulation_B.cv);
    ParamGet_dynamic_simulation_228.initialize_error_codes(0, 1, 2, 3);
    for (i = 0; i < 11; i++) {
      dynamic_simulation_B.initialValue[i] = tmp_e[i];
    }

    ParamGet_dynamic_simulation_228.set_initial_value
      (dynamic_simulation_B.initialValue, 11U);
    dynamic_simulation_DW.obj_c.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S19>/Get Parameter1'

    // Start for MATLABSystem: '<S19>/Get Parameter2'
    dynamic_simulation_DW.obj_px.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_px.isInitialized = 1;
    for (i = 0; i < 24; i++) {
      dynamic_simulation_B.cv[i] = tmp_f[i];
    }

    dynamic_simulation_B.cv[24] = '\x00';
    ParamGet_dynamic_simulation_229.initialize(dynamic_simulation_B.cv);
    ParamGet_dynamic_simulation_229.initialize_error_codes(0, 1, 2, 3);
    for (i = 0; i < 11; i++) {
      dynamic_simulation_B.initialValue[i] = tmp_g[i];
    }

    ParamGet_dynamic_simulation_229.set_initial_value
      (dynamic_simulation_B.initialValue, 11U);
    dynamic_simulation_DW.obj_px.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S19>/Get Parameter2'

    // Start for MATLABSystem: '<S19>/Get Parameter3'
    dynamic_simulation_DW.obj_nt.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_nt.isInitialized = 1;
    for (i = 0; i < 24; i++) {
      dynamic_simulation_B.cv[i] = tmp_h[i];
    }

    dynamic_simulation_B.cv[24] = '\x00';
    ParamGet_dynamic_simulation_230.initialize(dynamic_simulation_B.cv);
    ParamGet_dynamic_simulation_230.initialize_error_codes(0, 1, 2, 3);
    for (i = 0; i < 11; i++) {
      dynamic_simulation_B.initialValue[i] = tmp_i[i];
    }

    ParamGet_dynamic_simulation_230.set_initial_value
      (dynamic_simulation_B.initialValue, 11U);
    dynamic_simulation_DW.obj_nt.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S19>/Get Parameter3'

    // Start for MATLABSystem: '<S19>/Get Parameter4'
    dynamic_simulation_DW.obj_o.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_o.isInitialized = 1;
    for (i = 0; i < 24; i++) {
      dynamic_simulation_B.cv[i] = tmp_j[i];
    }

    dynamic_simulation_B.cv[24] = '\x00';
    ParamGet_dynamic_simulation_231.initialize(dynamic_simulation_B.cv);
    ParamGet_dynamic_simulation_231.initialize_error_codes(0, 1, 2, 3);
    for (i = 0; i < 11; i++) {
      dynamic_simulation_B.initialValue[i] = tmp_k[i];
    }

    ParamGet_dynamic_simulation_231.set_initial_value
      (dynamic_simulation_B.initialValue, 11U);
    dynamic_simulation_DW.obj_o.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S19>/Get Parameter4'

    // Start for MATLABSystem: '<S19>/Get Parameter5'
    dynamic_simulation_DW.obj_m4.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_m4.isInitialized = 1;
    for (i = 0; i < 24; i++) {
      dynamic_simulation_B.cv[i] = tmp_l[i];
    }

    dynamic_simulation_B.cv[24] = '\x00';
    ParamGet_dynamic_simulation_232.initialize(dynamic_simulation_B.cv);
    ParamGet_dynamic_simulation_232.initialize_error_codes(0, 1, 2, 3);
    for (i = 0; i < 11; i++) {
      dynamic_simulation_B.initialValue[i] = tmp_m[i];
    }

    ParamGet_dynamic_simulation_232.set_initial_value
      (dynamic_simulation_B.initialValue, 11U);
    dynamic_simulation_DW.obj_m4.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S19>/Get Parameter5'
    emxInitStruct_robotics_slmani_k(&dynamic_simulation_DW.obj_i);
    emxInitStruct_n_robotics_mani_k(&dynamic_simulation_DW.gobj_1_p);
    emxInitStruct_n_robotics_mani_k(&dynamic_simulation_DW.gobj_16_n);
    emxInitStruct_n_robotics_mani_k(&dynamic_simulation_DW.gobj_15_e);
    emxInitStruct_n_robotics_mani_k(&dynamic_simulation_DW.gobj_14_om);
    emxInitStruct_n_robotics_mani_k(&dynamic_simulation_DW.gobj_13_f);
    emxInitStruct_n_robotics_mani_k(&dynamic_simulation_DW.gobj_12_n);
    emxInitStruct_n_robotics_mani_k(&dynamic_simulation_DW.gobj_11_d);
    emxInitStruct_n_robotics_mani_k(&dynamic_simulation_DW.gobj_10_g);
    emxInitStruct_n_robotics_mani_k(&dynamic_simulation_DW.gobj_9_c);
    emxInitStruct_n_robotics_mani_k(&dynamic_simulation_DW.gobj_8_p);
    emxInitStruct_n_robotics_mani_k(&dynamic_simulation_DW.gobj_7_n);
    emxInitStruct_n_robotics_mani_k(&dynamic_simulation_DW.gobj_6_j);
    emxInitStruct_n_robotics_mani_k(&dynamic_simulation_DW.gobj_5_at);
    emxInitStruct_n_robotics_mani_k(&dynamic_simulation_DW.gobj_4_h);
    emxInitStruct_n_robotics_mani_k(&dynamic_simulation_DW.gobj_3_g0);
    emxInitStruct_n_robotics_mani_k(&dynamic_simulation_DW.gobj_2_on);

    // Start for MATLABSystem: '<S15>/MATLAB System'
    dynamic_simulation_DW.obj_i.isInitialized = 0;
    dynamic_simulation_DW.obj_i.isInitialized = 1;
    d_RigidBodyTree_RigidBodyTree_k(&dynamic_simulation_DW.obj_i.TreeInternal,
      &dynamic_simulation_DW.gobj_2_on, &dynamic_simulation_DW.gobj_4_h,
      &dynamic_simulation_DW.gobj_5_at, &dynamic_simulation_DW.gobj_6_j,
      &dynamic_simulation_DW.gobj_7_n, &dynamic_simulation_DW.gobj_8_p,
      &dynamic_simulation_DW.gobj_9_c, &dynamic_simulation_DW.gobj_3_g0);

    // Start for MATLABSystem: '<S21>/Get Parameter2'
    dynamic_simulation_DW.obj_e.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_e.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      dynamic_simulation_B.cv3[i] = tmp_n[i];
    }

    dynamic_simulation_B.cv3[12] = '\x00';
    ParamGet_dynamic_simulation_135.initialize(dynamic_simulation_B.cv3);
    ParamGet_dynamic_simulation_135.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynamic_simulation_135.set_initial_value(0.0);
    dynamic_simulation_DW.obj_e.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S21>/Get Parameter2'

    // Start for MATLABSystem: '<S21>/Get Parameter3'
    dynamic_simulation_DW.obj_ez.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_ez.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      dynamic_simulation_B.cv3[i] = tmp_o[i];
    }

    dynamic_simulation_B.cv3[12] = '\x00';
    ParamGet_dynamic_simulation_136.initialize(dynamic_simulation_B.cv3);
    ParamGet_dynamic_simulation_136.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynamic_simulation_136.set_initial_value(0.0);
    dynamic_simulation_DW.obj_ez.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S21>/Get Parameter3'

    // Start for MATLABSystem: '<S21>/Get Parameter8'
    dynamic_simulation_DW.obj_io.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_io.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      dynamic_simulation_B.cv3[i] = tmp_p[i];
    }

    dynamic_simulation_B.cv3[12] = '\x00';
    ParamGet_dynamic_simulation_141.initialize(dynamic_simulation_B.cv3);
    ParamGet_dynamic_simulation_141.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynamic_simulation_141.set_initial_value(0.0);
    dynamic_simulation_DW.obj_io.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S21>/Get Parameter8'

    // Start for MATLABSystem: '<S21>/Get Parameter9'
    dynamic_simulation_DW.obj_ezd.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_ezd.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      dynamic_simulation_B.cv3[i] = tmp_q[i];
    }

    dynamic_simulation_B.cv3[12] = '\x00';
    ParamGet_dynamic_simulation_142.initialize(dynamic_simulation_B.cv3);
    ParamGet_dynamic_simulation_142.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynamic_simulation_142.set_initial_value(0.0);
    dynamic_simulation_DW.obj_ezd.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S21>/Get Parameter9'

    // Start for MATLABSystem: '<S21>/Get Parameter10'
    dynamic_simulation_DW.obj_n.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_n.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      dynamic_simulation_B.cv3[i] = tmp_r[i];
    }

    dynamic_simulation_B.cv3[12] = '\x00';
    ParamGet_dynamic_simulation_133.initialize(dynamic_simulation_B.cv3);
    ParamGet_dynamic_simulation_133.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynamic_simulation_133.set_initial_value(0.0);
    dynamic_simulation_DW.obj_n.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S21>/Get Parameter10'

    // Start for MATLABSystem: '<S21>/Get Parameter11'
    dynamic_simulation_DW.obj_p.matlabCodegenIsDeleted = false;
    dynamic_simulation_DW.obj_p.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      dynamic_simulation_B.cv3[i] = tmp_s[i];
    }

    dynamic_simulation_B.cv3[12] = '\x00';
    ParamGet_dynamic_simulation_134.initialize(dynamic_simulation_B.cv3);
    ParamGet_dynamic_simulation_134.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynamic_simulation_134.set_initial_value(0.0);
    dynamic_simulation_DW.obj_p.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S21>/Get Parameter11'
    emxInitStruct_robotics_slman_kq(&dynamic_simulation_DW.obj);
    emxInitStruct_n_robotics_man_kq(&dynamic_simulation_DW.gobj_1);
    emxInitStruct_n_robotics_man_kq(&dynamic_simulation_DW.gobj_16);
    emxInitStruct_n_robotics_man_kq(&dynamic_simulation_DW.gobj_15);
    emxInitStruct_n_robotics_man_kq(&dynamic_simulation_DW.gobj_14);
    emxInitStruct_n_robotics_man_kq(&dynamic_simulation_DW.gobj_13);
    emxInitStruct_n_robotics_man_kq(&dynamic_simulation_DW.gobj_12);
    emxInitStruct_n_robotics_man_kq(&dynamic_simulation_DW.gobj_11);
    emxInitStruct_n_robotics_man_kq(&dynamic_simulation_DW.gobj_10);
    emxInitStruct_n_robotics_man_kq(&dynamic_simulation_DW.gobj_9);
    emxInitStruct_n_robotics_man_kq(&dynamic_simulation_DW.gobj_8);
    emxInitStruct_n_robotics_man_kq(&dynamic_simulation_DW.gobj_7);
    emxInitStruct_n_robotics_man_kq(&dynamic_simulation_DW.gobj_6);
    emxInitStruct_n_robotics_man_kq(&dynamic_simulation_DW.gobj_5);
    emxInitStruct_n_robotics_man_kq(&dynamic_simulation_DW.gobj_4);
    emxInitStruct_n_robotics_man_kq(&dynamic_simulation_DW.gobj_3);
    emxInitStruct_n_robotics_man_kq(&dynamic_simulation_DW.gobj_2);

    // Start for MATLABSystem: '<S20>/MATLAB System'
    dynamic_simulation_DW.obj.isInitialized = 0;
    dynamic_simulation_DW.obj.isInitialized = 1;
    RigidBodyTree_RigidBodyTree_kq(&dynamic_simulation_DW.obj.TreeInternal,
      &dynamic_simulation_DW.gobj_2, &dynamic_simulation_DW.gobj_4,
      &dynamic_simulation_DW.gobj_5, &dynamic_simulation_DW.gobj_6,
      &dynamic_simulation_DW.gobj_7, &dynamic_simulation_DW.gobj_8,
      &dynamic_simulation_DW.gobj_9, &dynamic_simulation_DW.gobj_3);
  }

  // set "at time zero" to false
  if (rtmIsFirstInitCond(dynamic_simulation_M)) {
    rtmSetFirstInitCond(dynamic_simulation_M, 0);
  }
}

// Model terminate function
void dynamic_simulation_terminate(void)
{
  // Terminate for MATLABSystem: '<S21>/Get Parameter12'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_l);

  // Terminate for MATLABSystem: '<S21>/Get Parameter1'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_lm);

  // Terminate for MATLABSystem: '<S21>/Get Parameter4'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_a);

  // Terminate for MATLABSystem: '<S21>/Get Parameter5'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_d);

  // Terminate for MATLABSystem: '<S21>/Get Parameter6'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_h);

  // Terminate for MATLABSystem: '<S21>/Get Parameter7'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_f0);
  emxFreeStruct_robotics_slmanip_(&dynamic_simulation_DW.obj_f);
  emxFreeStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_1_m);
  emxFreeStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_16_a);
  emxFreeStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_15_c);
  emxFreeStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_14_o);
  emxFreeStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_13_i);
  emxFreeStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_12_d);
  emxFreeStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_11_c);
  emxFreeStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_10_l);
  emxFreeStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_9_p);
  emxFreeStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_8_i);
  emxFreeStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_7_o);
  emxFreeStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_6_a);
  emxFreeStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_5_a);
  emxFreeStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_4_g);
  emxFreeStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_3_g);
  emxFreeStruct_n_robotics_manip_(&dynamic_simulation_DW.gobj_2_o);

  // Terminate for MATLABSystem: '<S19>/Get Parameter'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_m);

  // Terminate for MATLABSystem: '<S19>/Get Parameter1'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_c);

  // Terminate for MATLABSystem: '<S19>/Get Parameter2'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_px);

  // Terminate for MATLABSystem: '<S19>/Get Parameter3'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_nt);

  // Terminate for MATLABSystem: '<S19>/Get Parameter4'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_o);

  // Terminate for MATLABSystem: '<S19>/Get Parameter5'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_m4);
  emxFreeStruct_robotics_slmani_k(&dynamic_simulation_DW.obj_i);
  emxFreeStruct_n_robotics_mani_k(&dynamic_simulation_DW.gobj_1_p);
  emxFreeStruct_n_robotics_mani_k(&dynamic_simulation_DW.gobj_16_n);
  emxFreeStruct_n_robotics_mani_k(&dynamic_simulation_DW.gobj_15_e);
  emxFreeStruct_n_robotics_mani_k(&dynamic_simulation_DW.gobj_14_om);
  emxFreeStruct_n_robotics_mani_k(&dynamic_simulation_DW.gobj_13_f);
  emxFreeStruct_n_robotics_mani_k(&dynamic_simulation_DW.gobj_12_n);
  emxFreeStruct_n_robotics_mani_k(&dynamic_simulation_DW.gobj_11_d);
  emxFreeStruct_n_robotics_mani_k(&dynamic_simulation_DW.gobj_10_g);
  emxFreeStruct_n_robotics_mani_k(&dynamic_simulation_DW.gobj_9_c);
  emxFreeStruct_n_robotics_mani_k(&dynamic_simulation_DW.gobj_8_p);
  emxFreeStruct_n_robotics_mani_k(&dynamic_simulation_DW.gobj_7_n);
  emxFreeStruct_n_robotics_mani_k(&dynamic_simulation_DW.gobj_6_j);
  emxFreeStruct_n_robotics_mani_k(&dynamic_simulation_DW.gobj_5_at);
  emxFreeStruct_n_robotics_mani_k(&dynamic_simulation_DW.gobj_4_h);
  emxFreeStruct_n_robotics_mani_k(&dynamic_simulation_DW.gobj_3_g0);
  emxFreeStruct_n_robotics_mani_k(&dynamic_simulation_DW.gobj_2_on);

  // Terminate for MATLABSystem: '<S21>/Get Parameter2'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_e);

  // Terminate for MATLABSystem: '<S21>/Get Parameter3'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_ez);

  // Terminate for MATLABSystem: '<S21>/Get Parameter8'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_io);

  // Terminate for MATLABSystem: '<S21>/Get Parameter9'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_ezd);

  // Terminate for MATLABSystem: '<S21>/Get Parameter10'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_n);

  // Terminate for MATLABSystem: '<S21>/Get Parameter11'
  matlabCodegenHandle_matlabCodeg(&dynamic_simulation_DW.obj_p);

  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S11>/SourceBlock'
  dynamic_simul_matlabCodegenHa_e(&dynamic_simulation_DW.obj_fk);

  // End of Terminate for SubSystem: '<Root>/Subscribe'
  emxFreeStruct_robotics_slman_kq(&dynamic_simulation_DW.obj);
  emxFreeStruct_n_robotics_man_kq(&dynamic_simulation_DW.gobj_1);
  emxFreeStruct_n_robotics_man_kq(&dynamic_simulation_DW.gobj_16);
  emxFreeStruct_n_robotics_man_kq(&dynamic_simulation_DW.gobj_15);
  emxFreeStruct_n_robotics_man_kq(&dynamic_simulation_DW.gobj_14);
  emxFreeStruct_n_robotics_man_kq(&dynamic_simulation_DW.gobj_13);
  emxFreeStruct_n_robotics_man_kq(&dynamic_simulation_DW.gobj_12);
  emxFreeStruct_n_robotics_man_kq(&dynamic_simulation_DW.gobj_11);
  emxFreeStruct_n_robotics_man_kq(&dynamic_simulation_DW.gobj_10);
  emxFreeStruct_n_robotics_man_kq(&dynamic_simulation_DW.gobj_9);
  emxFreeStruct_n_robotics_man_kq(&dynamic_simulation_DW.gobj_8);
  emxFreeStruct_n_robotics_man_kq(&dynamic_simulation_DW.gobj_7);
  emxFreeStruct_n_robotics_man_kq(&dynamic_simulation_DW.gobj_6);
  emxFreeStruct_n_robotics_man_kq(&dynamic_simulation_DW.gobj_5);
  emxFreeStruct_n_robotics_man_kq(&dynamic_simulation_DW.gobj_4);
  emxFreeStruct_n_robotics_man_kq(&dynamic_simulation_DW.gobj_3);
  emxFreeStruct_n_robotics_man_kq(&dynamic_simulation_DW.gobj_2);

  // Terminate for Atomic SubSystem: '<Root>/Publish2'
  // Terminate for MATLABSystem: '<S8>/SinkBlock'
  matlabCodegenHandle_matl_kq2tzt(&dynamic_simulation_DW.obj_g);

  // End of Terminate for SubSystem: '<Root>/Publish2'

  // Terminate for Atomic SubSystem: '<Root>/Publish1'
  // Terminate for MATLABSystem: '<S7>/SinkBlock'
  matlabCodegenHandle_matl_kq2tzt(&dynamic_simulation_DW.obj_iv);

  // End of Terminate for SubSystem: '<Root>/Publish1'

  // Terminate for Atomic SubSystem: '<Root>/Publish3'
  // Terminate for MATLABSystem: '<S9>/SinkBlock'
  matlabCodegenHandle_matl_kq2tzt(&dynamic_simulation_DW.obj_dj);

  // End of Terminate for SubSystem: '<Root>/Publish3'

  // Terminate for Atomic SubSystem: '<Root>/Publish'
  // Terminate for MATLABSystem: '<S6>/SinkBlock'
  matlabCodegenHandle_matl_kq2tzt(&dynamic_simulation_DW.obj_cd);

  // End of Terminate for SubSystem: '<Root>/Publish'
}

//
// File trailer for generated code.
//
// [EOF]
//
