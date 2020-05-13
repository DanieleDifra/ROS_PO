//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: dynsim_2link_planar.cpp
//
// Code generated for Simulink model 'dynsim_2link_planar'.
//
// Model version                  : 1.117
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Wed May 13 22:25:52 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "dynsim_2link_planar.h"
#include "dynsim_2link_planar_private.h"

// Block signals (default storage)
B_dynsim_2link_planar_T dynsim_2link_planar_B;

// Continuous states
X_dynsim_2link_planar_T dynsim_2link_planar_X;

// Block states (default storage)
DW_dynsim_2link_planar_T dynsim_2link_planar_DW;

// Real-time model
RT_MODEL_dynsim_2link_planar_T dynsim_2link_planar_M_ =
  RT_MODEL_dynsim_2link_planar_T();
RT_MODEL_dynsim_2link_planar_T *const dynsim_2link_planar_M =
  &dynsim_2link_planar_M_;

// Forward declaration for local functions
static void dynsim_2link_pla_emxInit_real_T(emxArray_real_T_dynsim_2link__T
  **pEmxArray, int32_T numDimensions);
static void dynsim_2lin_emxInit_f_cell_wrap(emxArray_f_cell_wrap_dynsim_2_T
  **pEmxArray, int32_T numDimensions);
static void dynsim_2link_pla_emxInit_char_T(emxArray_char_T_dynsim_2link__T
  **pEmxArray, int32_T numDimensions);
static void emxEnsureCapacity_f_cell_wrap1(emxArray_f_cell_wrap_dynsim_2_T
  *emxArray, int32_T oldNumel);
static void dynsim_emxEnsureCapacity_char_T(emxArray_char_T_dynsim_2link__T
  *emxArray, int32_T oldNumel);
static void rigidBodyJoint_get_JointAxis_a(const c_rigidBodyJoint_dynsim_2li_a_T
  *obj, real_T ax[3]);
static void dynsim_2link_pla_emxFree_char_T(emxArray_char_T_dynsim_2link__T
  **pEmxArray);
static void RigidBodyTree_forwardKinematics(p_robotics_manip_internal_R_a_T *obj,
  const real_T qvec[6], emxArray_f_cell_wrap_dynsim_2_T *Ttree);
static void dynsim_emxEnsureCapacity_real_T(emxArray_real_T_dynsim_2link__T
  *emxArray, int32_T oldNumel);
static void dynsim_2link_pla_emxFree_real_T(emxArray_real_T_dynsim_2link__T
  **pEmxArray);
static void dynsim_2lin_emxFree_f_cell_wrap(emxArray_f_cell_wrap_dynsim_2_T
  **pEmxArray);
static void RigidBodyTree_geometricJacobian(p_robotics_manip_internal_R_a_T *obj,
  const real_T Q[6], emxArray_real_T_dynsim_2link__T *Jac);
static void rigidBodyJoint_get_JointAxis_aw(const
  c_rigidBodyJoint_dynsim_2l_aw_T *obj, real_T ax[3]);
static void RigidBodyTree_forwardKinemati_a(p_robotics_manip_internal__aw_T *obj,
  const real_T qvec[6], emxArray_f_cell_wrap_dynsim_2_T *Ttree);
static void dynsim_2link_pl_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Data[128], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_dynsim_2link_planar_std_msgs_MultiArrayDimension
  varargout_2_Layout_Dim[16], uint32_T *varargout_2_Layout_Dim_SL_Info_,
  uint32_T *varargout_2_Layout_Dim_SL_Inf_0);
static void dynsim_2li_emxInit_f_cell_wrap1(emxArray_f_cell_wrap_dynsim_a_T
  **pEmxArray, int32_T numDimensions);
static void d_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_dynsim_a_T
  *emxArray, int32_T oldNumel);
static void dy_rigidBodyJoint_get_JointAxis(const
  c_rigidBodyJoint_dynsim_2link_T *obj, real_T ax[3]);
static void dynsim_2link_planar_cat(real_T varargin_1, real_T varargin_2, real_T
  varargin_3, real_T varargin_4, real_T varargin_5, real_T varargin_6, real_T
  varargin_7, real_T varargin_8, real_T varargin_9, real_T y[9]);
static void rigidBodyJoint_transformBodyT_a(const
  c_rigidBodyJoint_dynsim_2link_T *obj, const real_T q_data[], const int32_T
  *q_size, real_T T[16]);
static void rigidBodyJoint_transformBodyToP(const
  c_rigidBodyJoint_dynsim_2link_T *obj, real_T T[16]);
static void dynsim_2link_planar_tforminv(const real_T T[16], real_T Tinv[16]);
static void dynsim_2lin_tformToSpatialXform(const real_T T[16], real_T X[36]);
static void dynsim_2li_emxFree_f_cell_wrap1(emxArray_f_cell_wrap_dynsim_a_T
  **pEmxArray);
static void RigidBodyTreeDynamics_massMatri(p_robotics_manip_internal_Rig_T
  *robot, const real_T q[6], emxArray_real_T_dynsim_2link__T *H,
  emxArray_real_T_dynsim_2link__T *lambda);
static void RigidBodyTreeDynamics_inverseDy(p_robotics_manip_internal_Rig_T
  *robot, const real_T q[6], const real_T qdot[6], const real_T fext[48], real_T
  tau[6]);
static boolean_T dynsim_2link_plana_anyNonFinite(const real_T x[16]);
static real_T dynsim_2link_plan_rt_hypotd_snf(real_T u0, real_T u1);
static real_T dynsim_2link_planar_xzlangeM(const creal_T x[16]);
static void dynsim_2link_planar_xzlascl(real_T cfrom, real_T cto, creal_T A[16]);
static real_T dynsim_2link_planar_xzlanhs(const creal_T A[16], int32_T ilo,
  int32_T ihi);
static void dynsim_2link_planar_xzlartg_d(const creal_T f, const creal_T g,
  real_T *cs, creal_T *sn);
static void dynsim_2link_planar_xzlartg(const creal_T f, const creal_T g, real_T
  *cs, creal_T *sn, creal_T *r);
static void dynsim_2link_planar_xzhgeqz(creal_T A[16], int32_T ilo, int32_T ihi,
  creal_T Z[16], int32_T *info, creal_T alpha1[4], creal_T beta1[4]);
static void dynsim_2link_planar_xztgevc(const creal_T A[16], creal_T V[16]);
static void dynsim_2link_planar_xzggev(creal_T A[16], int32_T *info, creal_T
  alpha1[4], creal_T beta1[4], creal_T V[16]);
static real_T dynsim_2link_planar_xnrm2(int32_T n, const real_T x[16], int32_T
  ix0);
static void dynsim_2link_planar_xzlarf(int32_T m, int32_T n, int32_T iv0, real_T
  tau, real_T C[16], int32_T ic0, real_T work[4]);
static void dynsim_2link_planar_xgehrd(real_T a[16], real_T tau[3]);
static real_T dynsim_2link_planar_xnrm2_a(int32_T n, const real_T x[3]);
static real_T dynsim_2link_planar_xzlarfg(int32_T n, real_T *alpha1, real_T x[3]);
static void dynsim_2link_planar_xdlanv2(real_T *a, real_T *b, real_T *c, real_T *
  d, real_T *rt1r, real_T *rt1i, real_T *rt2r, real_T *rt2i, real_T *cs, real_T *
  sn);
static void dynsim_2link_planar_xrot(int32_T n, real_T x[16], int32_T ix0,
  int32_T iy0, real_T c, real_T s);
static void dynsim_2link_planar_xrot_l(int32_T n, real_T x[16], int32_T ix0,
  int32_T iy0, real_T c, real_T s);
static int32_T dynsim_2link_planar_eml_dlahqr(real_T h[16], real_T z[16]);
static void dynsim_2link_planar_eig(const real_T A[16], creal_T V[16], creal_T
  D[4]);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_GetP_T *obj);
static void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_dynsim_2li_a_T
  *pStruct);
static void emxFreeStruct_o_robotics_manip_(o_robotics_manip_internal_R_a_T
  *pStruct);
static void emxFreeStruct_p_robotics_manip_(p_robotics_manip_internal_R_a_T
  *pStruct);
static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_b_a_T
  *pStruct);
static void emxFreeStruct_n_robotics_manip_(n_robotics_manip_internal_R_a_T
  *pStruct);
static void emxFreeStruct_c_rigidBodyJoint1(c_rigidBodyJoint_dynsim_2l_aw_T
  *pStruct);
static void emxFreeStruct_o_robotics_mani_a(o_robotics_manip_internal__aw_T
  *pStruct);
static void emxFreeStruct_p_robotics_mani_a(p_robotics_manip_internal__aw_T
  *pStruct);
static void emxFreeStruct_robotics_slmani_a(robotics_slmanip_internal__aw_T
  *pStruct);
static void emxFreeStruct_n_robotics_mani_a(n_robotics_manip_internal__aw_T
  *pStruct);
static void dynsim_2link__matlabCodegenHa_p(ros_slros_internal_block_Subs_T *obj);
static void emxFreeStruct_c_rigidBodyJoint2(c_rigidBodyJoint_dynsim_2link_T
  *pStruct);
static void emxFreeStruct_o_robotics_man_aw(o_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_p_robotics_man_aw(p_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_robotics_slman_aw(robotics_slmanip_internal_blo_T
  *pStruct);
static void emxFreeStruct_n_robotics_man_aw(n_robotics_manip_internal_Rig_T
  *pStruct);
static void matlabCodegenHandl_awtnueqcp3y5(ros_slros_internal_block_Publ_T *obj);
static void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_dynsim_2li_a_T
  *pStruct);
static void emxInitStruct_o_robotics_manip_(o_robotics_manip_internal_R_a_T
  *pStruct);
static void emxInitStruct_p_robotics_manip_(p_robotics_manip_internal_R_a_T
  *pStruct);
static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_b_a_T
  *pStruct);
static void emxInitStruct_n_robotics_manip_(n_robotics_manip_internal_R_a_T
  *pStruct);
static n_robotics_manip_internal_R_a_T *d_RigidBody_RigidBody_awtnueqcp
  (n_robotics_manip_internal_R_a_T *obj);
static n_robotics_manip_internal_R_a_T *RigidBody_RigidBody_awtnueqcp3
  (n_robotics_manip_internal_R_a_T *obj);
static n_robotics_manip_internal_R_a_T *RigidBody_RigidBody_awtnueqcp3y
  (n_robotics_manip_internal_R_a_T *obj);
static n_robotics_manip_internal_R_a_T *RigidBody_RigidBod_awtnueqcp3y5
  (n_robotics_manip_internal_R_a_T *obj);
static n_robotics_manip_internal_R_a_T *RigidBody_RigidBo_awtnueqcp3y53
  (n_robotics_manip_internal_R_a_T *obj);
static n_robotics_manip_internal_R_a_T *RigidBody_RigidB_awtnueqcp3y53m
  (n_robotics_manip_internal_R_a_T *obj);
static n_robotics_manip_internal_R_a_T *RigidBody_Rigid_awtnueqcp3y53m2
  (n_robotics_manip_internal_R_a_T *obj);
static n_robotics_manip_internal_R_a_T *d_RigidBody_Rigid_p
  (n_robotics_manip_internal_R_a_T *obj);
static o_robotics_manip_internal_R_a_T *d_RigidBody_Rigid_c
  (o_robotics_manip_internal_R_a_T *obj);
static p_robotics_manip_internal_R_a_T *d_RigidBodyTree_RigidBodyTree_a
  (p_robotics_manip_internal_R_a_T *obj, n_robotics_manip_internal_R_a_T *iobj_0,
   n_robotics_manip_internal_R_a_T *iobj_1, n_robotics_manip_internal_R_a_T
   *iobj_2, n_robotics_manip_internal_R_a_T *iobj_3,
   n_robotics_manip_internal_R_a_T *iobj_4, n_robotics_manip_internal_R_a_T
   *iobj_5, n_robotics_manip_internal_R_a_T *iobj_6,
   n_robotics_manip_internal_R_a_T *iobj_7);
static void emxInitStruct_c_rigidBodyJoint1(c_rigidBodyJoint_dynsim_2l_aw_T
  *pStruct);
static void emxInitStruct_o_robotics_mani_a(o_robotics_manip_internal__aw_T
  *pStruct);
static void emxInitStruct_p_robotics_mani_a(p_robotics_manip_internal__aw_T
  *pStruct);
static void emxInitStruct_robotics_slmani_a(robotics_slmanip_internal__aw_T
  *pStruct);
static void emxInitStruct_n_robotics_mani_a(n_robotics_manip_internal__aw_T
  *pStruct);
static n_robotics_manip_internal__aw_T *d_RigidBody_Rigid_n
  (n_robotics_manip_internal__aw_T *obj);
static n_robotics_manip_internal__aw_T *d_RigidBody_Rigid_h
  (n_robotics_manip_internal__aw_T *obj);
static n_robotics_manip_internal__aw_T *d_RigidBody_Rigid_j
  (n_robotics_manip_internal__aw_T *obj);
static n_robotics_manip_internal__aw_T *d_RigidBody_Rigid_c3
  (n_robotics_manip_internal__aw_T *obj);
static n_robotics_manip_internal__aw_T *d_RigidBody_Rigid_k
  (n_robotics_manip_internal__aw_T *obj);
static p_robotics_manip_internal__aw_T *RigidBodyTree_RigidBodyTree_aw
  (p_robotics_manip_internal__aw_T *obj, n_robotics_manip_internal__aw_T *iobj_0,
   n_robotics_manip_internal__aw_T *iobj_1, n_robotics_manip_internal__aw_T
   *iobj_2, n_robotics_manip_internal__aw_T *iobj_3,
   n_robotics_manip_internal__aw_T *iobj_4, n_robotics_manip_internal__aw_T
   *iobj_5, n_robotics_manip_internal__aw_T *iobj_6,
   n_robotics_manip_internal__aw_T *iobj_7);
static void emxInitStruct_c_rigidBodyJoint2(c_rigidBodyJoint_dynsim_2link_T
  *pStruct);
static void emxInitStruct_o_robotics_man_aw(o_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_p_robotics_man_aw(p_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_robotics_slman_aw(robotics_slmanip_internal_blo_T
  *pStruct);
static void emxInitStruct_n_robotics_man_aw(n_robotics_manip_internal_Rig_T
  *pStruct);
static n_robotics_manip_internal_Rig_T *dynsim_2lin_RigidBody_RigidBody
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *dynsim_2l_RigidBody_RigidBody_a
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *dynsim_2_RigidBody_RigidBody_aw
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *dynsim__RigidBody_RigidBody_awt
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *dynsim_RigidBody_RigidBody_awtn
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *dynsi_RigidBody_RigidBody_awtnu
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *dyns_RigidBody_RigidBody_awtnue
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *dyn_RigidBody_RigidBody_awtnueq
  (n_robotics_manip_internal_Rig_T *obj);
static o_robotics_manip_internal_Rig_T *dy_RigidBody_RigidBody_awtnueqc
  (o_robotics_manip_internal_Rig_T *obj);
static p_robotics_manip_internal_Rig_T *dyn_RigidBodyTree_RigidBodyTree
  (p_robotics_manip_internal_Rig_T *obj, n_robotics_manip_internal_Rig_T *iobj_0,
   n_robotics_manip_internal_Rig_T *iobj_1, n_robotics_manip_internal_Rig_T
   *iobj_2, n_robotics_manip_internal_Rig_T *iobj_3,
   n_robotics_manip_internal_Rig_T *iobj_4, n_robotics_manip_internal_Rig_T
   *iobj_5, n_robotics_manip_internal_Rig_T *iobj_6,
   n_robotics_manip_internal_Rig_T *iobj_7);
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
  dynsim_2link_planar_derivatives();

  // f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*));
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  dynsim_2link_planar_step();
  dynsim_2link_planar_derivatives();

  // f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*));
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  dynsim_2link_planar_step();
  dynsim_2link_planar_derivatives();

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

static void dynsim_2link_pla_emxInit_real_T(emxArray_real_T_dynsim_2link__T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_real_T_dynsim_2link__T *emxArray;
  *pEmxArray = (emxArray_real_T_dynsim_2link__T *)malloc(sizeof
    (emxArray_real_T_dynsim_2link__T));
  emxArray = *pEmxArray;
  emxArray->data = (real_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (dynsim_2link_planar_B.i_jr = 0; dynsim_2link_planar_B.i_jr <
       numDimensions; dynsim_2link_planar_B.i_jr++) {
    emxArray->size[dynsim_2link_planar_B.i_jr] = 0;
  }
}

static void dynsim_2lin_emxInit_f_cell_wrap(emxArray_f_cell_wrap_dynsim_2_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_f_cell_wrap_dynsim_2_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_f_cell_wrap_dynsim_2_T *)malloc(sizeof
    (emxArray_f_cell_wrap_dynsim_2_T));
  emxArray = *pEmxArray;
  emxArray->data = (f_cell_wrap_dynsim_2link_pl_a_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void dynsim_2link_pla_emxInit_char_T(emxArray_char_T_dynsim_2link__T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_char_T_dynsim_2link__T *emxArray;
  *pEmxArray = (emxArray_char_T_dynsim_2link__T *)malloc(sizeof
    (emxArray_char_T_dynsim_2link__T));
  emxArray = *pEmxArray;
  emxArray->data = (char_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (dynsim_2link_planar_B.i_d = 0; dynsim_2link_planar_B.i_d < numDimensions;
       dynsim_2link_planar_B.i_d++) {
    emxArray->size[dynsim_2link_planar_B.i_d] = 0;
  }
}

static void emxEnsureCapacity_f_cell_wrap1(emxArray_f_cell_wrap_dynsim_2_T
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
                     (f_cell_wrap_dynsim_2link_pl_a_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(f_cell_wrap_dynsim_2link_pl_a_T)
             * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (f_cell_wrap_dynsim_2link_pl_a_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void dynsim_emxEnsureCapacity_char_T(emxArray_char_T_dynsim_2link__T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  dynsim_2link_planar_B.newNumel_d = 1;
  for (dynsim_2link_planar_B.i_g = 0; dynsim_2link_planar_B.i_g <
       emxArray->numDimensions; dynsim_2link_planar_B.i_g++) {
    dynsim_2link_planar_B.newNumel_d *= emxArray->size[dynsim_2link_planar_B.i_g];
  }

  if (dynsim_2link_planar_B.newNumel_d > emxArray->allocatedSize) {
    dynsim_2link_planar_B.i_g = emxArray->allocatedSize;
    if (dynsim_2link_planar_B.i_g < 16) {
      dynsim_2link_planar_B.i_g = 16;
    }

    while (dynsim_2link_planar_B.i_g < dynsim_2link_planar_B.newNumel_d) {
      if (dynsim_2link_planar_B.i_g > 1073741823) {
        dynsim_2link_planar_B.i_g = MAX_int32_T;
      } else {
        dynsim_2link_planar_B.i_g <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(dynsim_2link_planar_B.i_g), sizeof
                     (char_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(char_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (char_T *)newData;
    emxArray->allocatedSize = dynsim_2link_planar_B.i_g;
    emxArray->canFreeData = true;
  }
}

static void rigidBodyJoint_get_JointAxis_a(const c_rigidBodyJoint_dynsim_2li_a_T
  *obj, real_T ax[3])
{
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  boolean_T guard1 = false;
  int32_T exitg1;
  for (dynsim_2link_planar_B.b_kstr_p = 0; dynsim_2link_planar_B.b_kstr_p < 8;
       dynsim_2link_planar_B.b_kstr_p++) {
    dynsim_2link_planar_B.b_px[dynsim_2link_planar_B.b_kstr_p] =
      tmp[dynsim_2link_planar_B.b_kstr_p];
  }

  dynsim_2link_planar_B.b_bool_d = false;
  if (obj->Type->size[1] == 8) {
    dynsim_2link_planar_B.b_kstr_p = 1;
    do {
      exitg1 = 0;
      if (dynsim_2link_planar_B.b_kstr_p - 1 < 8) {
        dynsim_2link_planar_B.kstr_p = dynsim_2link_planar_B.b_kstr_p - 1;
        if (obj->Type->data[dynsim_2link_planar_B.kstr_p] !=
            dynsim_2link_planar_B.b_px[dynsim_2link_planar_B.kstr_p]) {
          exitg1 = 1;
        } else {
          dynsim_2link_planar_B.b_kstr_p++;
        }
      } else {
        dynsim_2link_planar_B.b_bool_d = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (dynsim_2link_planar_B.b_bool_d) {
    guard1 = true;
  } else {
    for (dynsim_2link_planar_B.b_kstr_p = 0; dynsim_2link_planar_B.b_kstr_p < 9;
         dynsim_2link_planar_B.b_kstr_p++) {
      dynsim_2link_planar_B.b_j[dynsim_2link_planar_B.b_kstr_p] =
        tmp_0[dynsim_2link_planar_B.b_kstr_p];
    }

    dynsim_2link_planar_B.b_bool_d = false;
    if (obj->Type->size[1] == 9) {
      dynsim_2link_planar_B.b_kstr_p = 1;
      do {
        exitg1 = 0;
        if (dynsim_2link_planar_B.b_kstr_p - 1 < 9) {
          dynsim_2link_planar_B.kstr_p = dynsim_2link_planar_B.b_kstr_p - 1;
          if (obj->Type->data[dynsim_2link_planar_B.kstr_p] !=
              dynsim_2link_planar_B.b_j[dynsim_2link_planar_B.kstr_p]) {
            exitg1 = 1;
          } else {
            dynsim_2link_planar_B.b_kstr_p++;
          }
        } else {
          dynsim_2link_planar_B.b_bool_d = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynsim_2link_planar_B.b_bool_d) {
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

static void dynsim_2link_pla_emxFree_char_T(emxArray_char_T_dynsim_2link__T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_char_T_dynsim_2link__T *)NULL) {
    if (((*pEmxArray)->data != (char_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_char_T_dynsim_2link__T *)NULL;
  }
}

static void RigidBodyTree_forwardKinematics(p_robotics_manip_internal_R_a_T *obj,
  const real_T qvec[6], emxArray_f_cell_wrap_dynsim_2_T *Ttree)
{
  n_robotics_manip_internal_R_a_T *body;
  emxArray_char_T_dynsim_2link__T *switch_expression;
  static const int8_T tmp[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  dynsim_2link_planar_B.n = obj->NumBodies;
  for (dynsim_2link_planar_B.b_kstr_n = 0; dynsim_2link_planar_B.b_kstr_n < 16;
       dynsim_2link_planar_B.b_kstr_n++) {
    dynsim_2link_planar_B.c_f1[dynsim_2link_planar_B.b_kstr_n] =
      tmp[dynsim_2link_planar_B.b_kstr_n];
  }

  dynsim_2link_planar_B.b_kstr_n = Ttree->size[0] * Ttree->size[1];
  Ttree->size[0] = 1;
  dynsim_2link_planar_B.e_o = static_cast<int32_T>(dynsim_2link_planar_B.n);
  Ttree->size[1] = dynsim_2link_planar_B.e_o;
  emxEnsureCapacity_f_cell_wrap1(Ttree, dynsim_2link_planar_B.b_kstr_n);
  if (dynsim_2link_planar_B.e_o != 0) {
    dynsim_2link_planar_B.ntilecols = dynsim_2link_planar_B.e_o - 1;
    if (0 <= dynsim_2link_planar_B.ntilecols) {
      memcpy(&dynsim_2link_planar_B.expl_temp.f1[0],
             &dynsim_2link_planar_B.c_f1[0], sizeof(real_T) << 4U);
    }

    for (dynsim_2link_planar_B.b_jtilecol = 0; dynsim_2link_planar_B.b_jtilecol <=
         dynsim_2link_planar_B.ntilecols; dynsim_2link_planar_B.b_jtilecol++) {
      Ttree->data[dynsim_2link_planar_B.b_jtilecol] =
        dynsim_2link_planar_B.expl_temp;
    }
  }

  dynsim_2link_planar_B.k = 1.0;
  dynsim_2link_planar_B.ntilecols = static_cast<int32_T>(dynsim_2link_planar_B.n)
    - 1;
  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  if (0 <= dynsim_2link_planar_B.ntilecols) {
    for (dynsim_2link_planar_B.b_kstr_n = 0; dynsim_2link_planar_B.b_kstr_n < 5;
         dynsim_2link_planar_B.b_kstr_n++) {
      dynsim_2link_planar_B.b_ao[dynsim_2link_planar_B.b_kstr_n] =
        tmp_0[dynsim_2link_planar_B.b_kstr_n];
    }
  }

  for (dynsim_2link_planar_B.b_jtilecol = 0; dynsim_2link_planar_B.b_jtilecol <=
       dynsim_2link_planar_B.ntilecols; dynsim_2link_planar_B.b_jtilecol++) {
    body = obj->Bodies[dynsim_2link_planar_B.b_jtilecol];
    dynsim_2link_planar_B.n = body->JointInternal.PositionNumber;
    dynsim_2link_planar_B.n += dynsim_2link_planar_B.k;
    if (dynsim_2link_planar_B.k > dynsim_2link_planar_B.n - 1.0) {
      dynsim_2link_planar_B.e_o = 0;
      dynsim_2link_planar_B.d_m = 0;
    } else {
      dynsim_2link_planar_B.e_o = static_cast<int32_T>(dynsim_2link_planar_B.k)
        - 1;
      dynsim_2link_planar_B.d_m = static_cast<int32_T>(dynsim_2link_planar_B.n -
        1.0);
    }

    dynsim_2link_planar_B.b_kstr_n = switch_expression->size[0] *
      switch_expression->size[1];
    switch_expression->size[0] = 1;
    switch_expression->size[1] = body->JointInternal.Type->size[1];
    dynsim_emxEnsureCapacity_char_T(switch_expression,
      dynsim_2link_planar_B.b_kstr_n);
    dynsim_2link_planar_B.loop_ub_l = body->JointInternal.Type->size[0] *
      body->JointInternal.Type->size[1] - 1;
    for (dynsim_2link_planar_B.b_kstr_n = 0; dynsim_2link_planar_B.b_kstr_n <=
         dynsim_2link_planar_B.loop_ub_l; dynsim_2link_planar_B.b_kstr_n++) {
      switch_expression->data[dynsim_2link_planar_B.b_kstr_n] =
        body->JointInternal.Type->data[dynsim_2link_planar_B.b_kstr_n];
    }

    dynsim_2link_planar_B.b_bool_e = false;
    if (switch_expression->size[1] == 5) {
      dynsim_2link_planar_B.b_kstr_n = 1;
      do {
        exitg1 = 0;
        if (dynsim_2link_planar_B.b_kstr_n - 1 < 5) {
          dynsim_2link_planar_B.loop_ub_l = dynsim_2link_planar_B.b_kstr_n - 1;
          if (switch_expression->data[dynsim_2link_planar_B.loop_ub_l] !=
              dynsim_2link_planar_B.b_ao[dynsim_2link_planar_B.loop_ub_l]) {
            exitg1 = 1;
          } else {
            dynsim_2link_planar_B.b_kstr_n++;
          }
        } else {
          dynsim_2link_planar_B.b_bool_e = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynsim_2link_planar_B.b_bool_e) {
      dynsim_2link_planar_B.b_kstr_n = 0;
    } else {
      for (dynsim_2link_planar_B.b_kstr_n = 0; dynsim_2link_planar_B.b_kstr_n <
           8; dynsim_2link_planar_B.b_kstr_n++) {
        dynsim_2link_planar_B.b_ct[dynsim_2link_planar_B.b_kstr_n] =
          tmp_1[dynsim_2link_planar_B.b_kstr_n];
      }

      dynsim_2link_planar_B.b_bool_e = false;
      if (switch_expression->size[1] == 8) {
        dynsim_2link_planar_B.b_kstr_n = 1;
        do {
          exitg1 = 0;
          if (dynsim_2link_planar_B.b_kstr_n - 1 < 8) {
            dynsim_2link_planar_B.loop_ub_l = dynsim_2link_planar_B.b_kstr_n - 1;
            if (switch_expression->data[dynsim_2link_planar_B.loop_ub_l] !=
                dynsim_2link_planar_B.b_ct[dynsim_2link_planar_B.loop_ub_l]) {
              exitg1 = 1;
            } else {
              dynsim_2link_planar_B.b_kstr_n++;
            }
          } else {
            dynsim_2link_planar_B.b_bool_e = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (dynsim_2link_planar_B.b_bool_e) {
        dynsim_2link_planar_B.b_kstr_n = 1;
      } else {
        dynsim_2link_planar_B.b_kstr_n = -1;
      }
    }

    switch (dynsim_2link_planar_B.b_kstr_n) {
     case 0:
      memset(&dynsim_2link_planar_B.c_f1[0], 0, sizeof(real_T) << 4U);
      dynsim_2link_planar_B.c_f1[0] = 1.0;
      dynsim_2link_planar_B.c_f1[5] = 1.0;
      dynsim_2link_planar_B.c_f1[10] = 1.0;
      dynsim_2link_planar_B.c_f1[15] = 1.0;
      break;

     case 1:
      rigidBodyJoint_get_JointAxis_a(&body->JointInternal,
        dynsim_2link_planar_B.v);
      dynsim_2link_planar_B.d_m -= dynsim_2link_planar_B.e_o;
      for (dynsim_2link_planar_B.b_kstr_n = 0; dynsim_2link_planar_B.b_kstr_n <
           dynsim_2link_planar_B.d_m; dynsim_2link_planar_B.b_kstr_n++) {
        dynsim_2link_planar_B.e_data[dynsim_2link_planar_B.b_kstr_n] =
          dynsim_2link_planar_B.e_o + dynsim_2link_planar_B.b_kstr_n;
      }

      dynsim_2link_planar_B.result_data[0] = dynsim_2link_planar_B.v[0];
      dynsim_2link_planar_B.result_data[1] = dynsim_2link_planar_B.v[1];
      dynsim_2link_planar_B.result_data[2] = dynsim_2link_planar_B.v[2];
      if (0 <= (dynsim_2link_planar_B.d_m != 0) - 1) {
        dynsim_2link_planar_B.result_data[3] =
          qvec[dynsim_2link_planar_B.e_data[0]];
      }

      dynsim_2link_planar_B.k = 1.0 / sqrt((dynsim_2link_planar_B.result_data[0]
        * dynsim_2link_planar_B.result_data[0] +
        dynsim_2link_planar_B.result_data[1] *
        dynsim_2link_planar_B.result_data[1]) +
        dynsim_2link_planar_B.result_data[2] *
        dynsim_2link_planar_B.result_data[2]);
      dynsim_2link_planar_B.v[0] = dynsim_2link_planar_B.result_data[0] *
        dynsim_2link_planar_B.k;
      dynsim_2link_planar_B.v[1] = dynsim_2link_planar_B.result_data[1] *
        dynsim_2link_planar_B.k;
      dynsim_2link_planar_B.v[2] = dynsim_2link_planar_B.result_data[2] *
        dynsim_2link_planar_B.k;
      dynsim_2link_planar_B.k = cos(dynsim_2link_planar_B.result_data[3]);
      dynsim_2link_planar_B.sth = sin(dynsim_2link_planar_B.result_data[3]);
      dynsim_2link_planar_B.tempR[0] = dynsim_2link_planar_B.v[0] *
        dynsim_2link_planar_B.v[0] * (1.0 - dynsim_2link_planar_B.k) +
        dynsim_2link_planar_B.k;
      dynsim_2link_planar_B.tempR_tmp = dynsim_2link_planar_B.v[1] *
        dynsim_2link_planar_B.v[0] * (1.0 - dynsim_2link_planar_B.k);
      dynsim_2link_planar_B.tempR_tmp_l = dynsim_2link_planar_B.v[2] *
        dynsim_2link_planar_B.sth;
      dynsim_2link_planar_B.tempR[1] = dynsim_2link_planar_B.tempR_tmp -
        dynsim_2link_planar_B.tempR_tmp_l;
      dynsim_2link_planar_B.tempR_tmp_o = dynsim_2link_planar_B.v[2] *
        dynsim_2link_planar_B.v[0] * (1.0 - dynsim_2link_planar_B.k);
      dynsim_2link_planar_B.tempR_tmp_o2 = dynsim_2link_planar_B.v[1] *
        dynsim_2link_planar_B.sth;
      dynsim_2link_planar_B.tempR[2] = dynsim_2link_planar_B.tempR_tmp_o +
        dynsim_2link_planar_B.tempR_tmp_o2;
      dynsim_2link_planar_B.tempR[3] = dynsim_2link_planar_B.tempR_tmp +
        dynsim_2link_planar_B.tempR_tmp_l;
      dynsim_2link_planar_B.tempR[4] = dynsim_2link_planar_B.v[1] *
        dynsim_2link_planar_B.v[1] * (1.0 - dynsim_2link_planar_B.k) +
        dynsim_2link_planar_B.k;
      dynsim_2link_planar_B.tempR_tmp = dynsim_2link_planar_B.v[2] *
        dynsim_2link_planar_B.v[1] * (1.0 - dynsim_2link_planar_B.k);
      dynsim_2link_planar_B.tempR_tmp_l = dynsim_2link_planar_B.v[0] *
        dynsim_2link_planar_B.sth;
      dynsim_2link_planar_B.tempR[5] = dynsim_2link_planar_B.tempR_tmp -
        dynsim_2link_planar_B.tempR_tmp_l;
      dynsim_2link_planar_B.tempR[6] = dynsim_2link_planar_B.tempR_tmp_o -
        dynsim_2link_planar_B.tempR_tmp_o2;
      dynsim_2link_planar_B.tempR[7] = dynsim_2link_planar_B.tempR_tmp +
        dynsim_2link_planar_B.tempR_tmp_l;
      dynsim_2link_planar_B.tempR[8] = dynsim_2link_planar_B.v[2] *
        dynsim_2link_planar_B.v[2] * (1.0 - dynsim_2link_planar_B.k) +
        dynsim_2link_planar_B.k;
      for (dynsim_2link_planar_B.b_kstr_n = 0; dynsim_2link_planar_B.b_kstr_n <
           3; dynsim_2link_planar_B.b_kstr_n++) {
        dynsim_2link_planar_B.e_o = dynsim_2link_planar_B.b_kstr_n + 1;
        dynsim_2link_planar_B.R_dh[dynsim_2link_planar_B.e_o - 1] =
          dynsim_2link_planar_B.tempR[(dynsim_2link_planar_B.e_o - 1) * 3];
        dynsim_2link_planar_B.e_o = dynsim_2link_planar_B.b_kstr_n + 1;
        dynsim_2link_planar_B.R_dh[dynsim_2link_planar_B.e_o + 2] =
          dynsim_2link_planar_B.tempR[(dynsim_2link_planar_B.e_o - 1) * 3 + 1];
        dynsim_2link_planar_B.e_o = dynsim_2link_planar_B.b_kstr_n + 1;
        dynsim_2link_planar_B.R_dh[dynsim_2link_planar_B.e_o + 5] =
          dynsim_2link_planar_B.tempR[(dynsim_2link_planar_B.e_o - 1) * 3 + 2];
      }

      memset(&dynsim_2link_planar_B.c_f1[0], 0, sizeof(real_T) << 4U);
      for (dynsim_2link_planar_B.b_kstr_n = 0; dynsim_2link_planar_B.b_kstr_n <
           3; dynsim_2link_planar_B.b_kstr_n++) {
        dynsim_2link_planar_B.d_m = dynsim_2link_planar_B.b_kstr_n << 2;
        dynsim_2link_planar_B.c_f1[dynsim_2link_planar_B.d_m] =
          dynsim_2link_planar_B.R_dh[3 * dynsim_2link_planar_B.b_kstr_n];
        dynsim_2link_planar_B.c_f1[dynsim_2link_planar_B.d_m + 1] =
          dynsim_2link_planar_B.R_dh[3 * dynsim_2link_planar_B.b_kstr_n + 1];
        dynsim_2link_planar_B.c_f1[dynsim_2link_planar_B.d_m + 2] =
          dynsim_2link_planar_B.R_dh[3 * dynsim_2link_planar_B.b_kstr_n + 2];
      }

      dynsim_2link_planar_B.c_f1[15] = 1.0;
      break;

     default:
      rigidBodyJoint_get_JointAxis_a(&body->JointInternal,
        dynsim_2link_planar_B.v);
      memset(&dynsim_2link_planar_B.tempR[0], 0, 9U * sizeof(real_T));
      dynsim_2link_planar_B.tempR[0] = 1.0;
      dynsim_2link_planar_B.tempR[4] = 1.0;
      dynsim_2link_planar_B.tempR[8] = 1.0;
      for (dynsim_2link_planar_B.b_kstr_n = 0; dynsim_2link_planar_B.b_kstr_n <
           3; dynsim_2link_planar_B.b_kstr_n++) {
        dynsim_2link_planar_B.d_m = dynsim_2link_planar_B.b_kstr_n << 2;
        dynsim_2link_planar_B.c_f1[dynsim_2link_planar_B.d_m] =
          dynsim_2link_planar_B.tempR[3 * dynsim_2link_planar_B.b_kstr_n];
        dynsim_2link_planar_B.c_f1[dynsim_2link_planar_B.d_m + 1] =
          dynsim_2link_planar_B.tempR[3 * dynsim_2link_planar_B.b_kstr_n + 1];
        dynsim_2link_planar_B.c_f1[dynsim_2link_planar_B.d_m + 2] =
          dynsim_2link_planar_B.tempR[3 * dynsim_2link_planar_B.b_kstr_n + 2];
        dynsim_2link_planar_B.c_f1[dynsim_2link_planar_B.b_kstr_n + 12] =
          dynsim_2link_planar_B.v[dynsim_2link_planar_B.b_kstr_n] *
          qvec[dynsim_2link_planar_B.e_o];
      }

      dynsim_2link_planar_B.c_f1[3] = 0.0;
      dynsim_2link_planar_B.c_f1[7] = 0.0;
      dynsim_2link_planar_B.c_f1[11] = 0.0;
      dynsim_2link_planar_B.c_f1[15] = 1.0;
      break;
    }

    for (dynsim_2link_planar_B.b_kstr_n = 0; dynsim_2link_planar_B.b_kstr_n < 16;
         dynsim_2link_planar_B.b_kstr_n++) {
      dynsim_2link_planar_B.a[dynsim_2link_planar_B.b_kstr_n] =
        body->
        JointInternal.JointToParentTransform[dynsim_2link_planar_B.b_kstr_n];
    }

    for (dynsim_2link_planar_B.b_kstr_n = 0; dynsim_2link_planar_B.b_kstr_n < 16;
         dynsim_2link_planar_B.b_kstr_n++) {
      dynsim_2link_planar_B.b[dynsim_2link_planar_B.b_kstr_n] =
        body->JointInternal.ChildToJointTransform[dynsim_2link_planar_B.b_kstr_n];
    }

    for (dynsim_2link_planar_B.b_kstr_n = 0; dynsim_2link_planar_B.b_kstr_n < 4;
         dynsim_2link_planar_B.b_kstr_n++) {
      for (dynsim_2link_planar_B.e_o = 0; dynsim_2link_planar_B.e_o < 4;
           dynsim_2link_planar_B.e_o++) {
        dynsim_2link_planar_B.d_m = dynsim_2link_planar_B.e_o << 2;
        dynsim_2link_planar_B.loop_ub_l = dynsim_2link_planar_B.b_kstr_n +
          dynsim_2link_planar_B.d_m;
        dynsim_2link_planar_B.a_p[dynsim_2link_planar_B.loop_ub_l] = 0.0;
        dynsim_2link_planar_B.a_p[dynsim_2link_planar_B.loop_ub_l] +=
          dynsim_2link_planar_B.c_f1[dynsim_2link_planar_B.d_m] *
          dynsim_2link_planar_B.a[dynsim_2link_planar_B.b_kstr_n];
        dynsim_2link_planar_B.a_p[dynsim_2link_planar_B.loop_ub_l] +=
          dynsim_2link_planar_B.c_f1[dynsim_2link_planar_B.d_m + 1] *
          dynsim_2link_planar_B.a[dynsim_2link_planar_B.b_kstr_n + 4];
        dynsim_2link_planar_B.a_p[dynsim_2link_planar_B.loop_ub_l] +=
          dynsim_2link_planar_B.c_f1[dynsim_2link_planar_B.d_m + 2] *
          dynsim_2link_planar_B.a[dynsim_2link_planar_B.b_kstr_n + 8];
        dynsim_2link_planar_B.a_p[dynsim_2link_planar_B.loop_ub_l] +=
          dynsim_2link_planar_B.c_f1[dynsim_2link_planar_B.d_m + 3] *
          dynsim_2link_planar_B.a[dynsim_2link_planar_B.b_kstr_n + 12];
      }

      for (dynsim_2link_planar_B.e_o = 0; dynsim_2link_planar_B.e_o < 4;
           dynsim_2link_planar_B.e_o++) {
        dynsim_2link_planar_B.d_m = dynsim_2link_planar_B.e_o << 2;
        dynsim_2link_planar_B.loop_ub_l = dynsim_2link_planar_B.b_kstr_n +
          dynsim_2link_planar_B.d_m;
        Ttree->data[dynsim_2link_planar_B.b_jtilecol]
          .f1[dynsim_2link_planar_B.loop_ub_l] = 0.0;
        Ttree->data[dynsim_2link_planar_B.b_jtilecol]
          .f1[dynsim_2link_planar_B.loop_ub_l] +=
          dynsim_2link_planar_B.b[dynsim_2link_planar_B.d_m] *
          dynsim_2link_planar_B.a_p[dynsim_2link_planar_B.b_kstr_n];
        Ttree->data[dynsim_2link_planar_B.b_jtilecol]
          .f1[dynsim_2link_planar_B.loop_ub_l] +=
          dynsim_2link_planar_B.b[dynsim_2link_planar_B.d_m + 1] *
          dynsim_2link_planar_B.a_p[dynsim_2link_planar_B.b_kstr_n + 4];
        Ttree->data[dynsim_2link_planar_B.b_jtilecol]
          .f1[dynsim_2link_planar_B.loop_ub_l] +=
          dynsim_2link_planar_B.b[dynsim_2link_planar_B.d_m + 2] *
          dynsim_2link_planar_B.a_p[dynsim_2link_planar_B.b_kstr_n + 8];
        Ttree->data[dynsim_2link_planar_B.b_jtilecol]
          .f1[dynsim_2link_planar_B.loop_ub_l] +=
          dynsim_2link_planar_B.b[dynsim_2link_planar_B.d_m + 3] *
          dynsim_2link_planar_B.a_p[dynsim_2link_planar_B.b_kstr_n + 12];
      }
    }

    dynsim_2link_planar_B.k = dynsim_2link_planar_B.n;
    if (body->ParentIndex > 0.0) {
      for (dynsim_2link_planar_B.b_kstr_n = 0; dynsim_2link_planar_B.b_kstr_n <
           16; dynsim_2link_planar_B.b_kstr_n++) {
        dynsim_2link_planar_B.a[dynsim_2link_planar_B.b_kstr_n] = Ttree->data[
          static_cast<int32_T>(body->ParentIndex) - 1]
          .f1[dynsim_2link_planar_B.b_kstr_n];
      }

      for (dynsim_2link_planar_B.b_kstr_n = 0; dynsim_2link_planar_B.b_kstr_n <
           4; dynsim_2link_planar_B.b_kstr_n++) {
        for (dynsim_2link_planar_B.e_o = 0; dynsim_2link_planar_B.e_o < 4;
             dynsim_2link_planar_B.e_o++) {
          dynsim_2link_planar_B.d_m = dynsim_2link_planar_B.e_o << 2;
          dynsim_2link_planar_B.loop_ub_l = dynsim_2link_planar_B.b_kstr_n +
            dynsim_2link_planar_B.d_m;
          dynsim_2link_planar_B.a_p[dynsim_2link_planar_B.loop_ub_l] = 0.0;
          dynsim_2link_planar_B.a_p[dynsim_2link_planar_B.loop_ub_l] +=
            Ttree->data[dynsim_2link_planar_B.b_jtilecol]
            .f1[dynsim_2link_planar_B.d_m] *
            dynsim_2link_planar_B.a[dynsim_2link_planar_B.b_kstr_n];
          dynsim_2link_planar_B.a_p[dynsim_2link_planar_B.loop_ub_l] +=
            Ttree->data[dynsim_2link_planar_B.b_jtilecol]
            .f1[dynsim_2link_planar_B.d_m + 1] *
            dynsim_2link_planar_B.a[dynsim_2link_planar_B.b_kstr_n + 4];
          dynsim_2link_planar_B.a_p[dynsim_2link_planar_B.loop_ub_l] +=
            Ttree->data[dynsim_2link_planar_B.b_jtilecol]
            .f1[dynsim_2link_planar_B.d_m + 2] *
            dynsim_2link_planar_B.a[dynsim_2link_planar_B.b_kstr_n + 8];
          dynsim_2link_planar_B.a_p[dynsim_2link_planar_B.loop_ub_l] +=
            Ttree->data[dynsim_2link_planar_B.b_jtilecol]
            .f1[dynsim_2link_planar_B.d_m + 3] *
            dynsim_2link_planar_B.a[dynsim_2link_planar_B.b_kstr_n + 12];
        }
      }

      for (dynsim_2link_planar_B.b_kstr_n = 0; dynsim_2link_planar_B.b_kstr_n <
           16; dynsim_2link_planar_B.b_kstr_n++) {
        Ttree->data[dynsim_2link_planar_B.b_jtilecol]
          .f1[dynsim_2link_planar_B.b_kstr_n] =
          dynsim_2link_planar_B.a_p[dynsim_2link_planar_B.b_kstr_n];
      }
    }
  }

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
}

static void dynsim_emxEnsureCapacity_real_T(emxArray_real_T_dynsim_2link__T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  dynsim_2link_planar_B.newNumel = 1;
  for (dynsim_2link_planar_B.i_if = 0; dynsim_2link_planar_B.i_if <
       emxArray->numDimensions; dynsim_2link_planar_B.i_if++) {
    dynsim_2link_planar_B.newNumel *= emxArray->size[dynsim_2link_planar_B.i_if];
  }

  if (dynsim_2link_planar_B.newNumel > emxArray->allocatedSize) {
    dynsim_2link_planar_B.i_if = emxArray->allocatedSize;
    if (dynsim_2link_planar_B.i_if < 16) {
      dynsim_2link_planar_B.i_if = 16;
    }

    while (dynsim_2link_planar_B.i_if < dynsim_2link_planar_B.newNumel) {
      if (dynsim_2link_planar_B.i_if > 1073741823) {
        dynsim_2link_planar_B.i_if = MAX_int32_T;
      } else {
        dynsim_2link_planar_B.i_if <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(dynsim_2link_planar_B.i_if), sizeof
                     (real_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(real_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (real_T *)newData;
    emxArray->allocatedSize = dynsim_2link_planar_B.i_if;
    emxArray->canFreeData = true;
  }
}

static void dynsim_2link_pla_emxFree_real_T(emxArray_real_T_dynsim_2link__T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T_dynsim_2link__T *)NULL) {
    if (((*pEmxArray)->data != (real_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_real_T_dynsim_2link__T *)NULL;
  }
}

static void dynsim_2lin_emxFree_f_cell_wrap(emxArray_f_cell_wrap_dynsim_2_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_f_cell_wrap_dynsim_2_T *)NULL) {
    if (((*pEmxArray)->data != (f_cell_wrap_dynsim_2link_pl_a_T *)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_f_cell_wrap_dynsim_2_T *)NULL;
  }
}

static void RigidBodyTree_geometricJacobian(p_robotics_manip_internal_R_a_T *obj,
  const real_T Q[6], emxArray_real_T_dynsim_2link__T *Jac)
{
  emxArray_f_cell_wrap_dynsim_2_T *Ttree;
  n_robotics_manip_internal_R_a_T *body;
  emxArray_real_T_dynsim_2link__T *JacSlice;
  emxArray_char_T_dynsim_2link__T *bname;
  emxArray_real_T_dynsim_2link__T *b;
  static const char_T tmp[11] = { 'e', 'd', 'o', '_', 'l', 'i', 'n', 'k', '_',
    'e', 'e' };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  int32_T exitg1;
  boolean_T exitg2;
  dynsim_2lin_emxInit_f_cell_wrap(&Ttree, 2);
  RigidBodyTree_forwardKinematics(obj, Q, Ttree);
  dynsim_2link_planar_B.b_kstr_a = Jac->size[0] * Jac->size[1];
  Jac->size[0] = 6;
  Jac->size[1] = static_cast<int32_T>(obj->VelocityNumber);
  dynsim_emxEnsureCapacity_real_T(Jac, dynsim_2link_planar_B.b_kstr_a);
  dynsim_2link_planar_B.loop_ub = 6 * static_cast<int32_T>(obj->VelocityNumber)
    - 1;
  for (dynsim_2link_planar_B.b_kstr_a = 0; dynsim_2link_planar_B.b_kstr_a <=
       dynsim_2link_planar_B.loop_ub; dynsim_2link_planar_B.b_kstr_a++) {
    Jac->data[dynsim_2link_planar_B.b_kstr_a] = 0.0;
  }

  for (dynsim_2link_planar_B.b_kstr_a = 0; dynsim_2link_planar_B.b_kstr_a < 8;
       dynsim_2link_planar_B.b_kstr_a++) {
    dynsim_2link_planar_B.chainmask[dynsim_2link_planar_B.b_kstr_a] = 0;
  }

  dynsim_2link_pla_emxInit_char_T(&bname, 2);
  dynsim_2link_planar_B.b_kstr_a = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj->Base.NameInternal->size[1];
  dynsim_emxEnsureCapacity_char_T(bname, dynsim_2link_planar_B.b_kstr_a);
  dynsim_2link_planar_B.loop_ub = obj->Base.NameInternal->size[0] *
    obj->Base.NameInternal->size[1] - 1;
  for (dynsim_2link_planar_B.b_kstr_a = 0; dynsim_2link_planar_B.b_kstr_a <=
       dynsim_2link_planar_B.loop_ub; dynsim_2link_planar_B.b_kstr_a++) {
    bname->data[dynsim_2link_planar_B.b_kstr_a] = obj->Base.NameInternal->
      data[dynsim_2link_planar_B.b_kstr_a];
  }

  for (dynsim_2link_planar_B.b_kstr_a = 0; dynsim_2link_planar_B.b_kstr_a < 11;
       dynsim_2link_planar_B.b_kstr_a++) {
    dynsim_2link_planar_B.a_m[dynsim_2link_planar_B.b_kstr_a] =
      tmp[dynsim_2link_planar_B.b_kstr_a];
  }

  dynsim_2link_planar_B.b_bool_b = false;
  if (11 == bname->size[1]) {
    dynsim_2link_planar_B.b_kstr_a = 1;
    do {
      exitg1 = 0;
      if (dynsim_2link_planar_B.b_kstr_a - 1 < 11) {
        dynsim_2link_planar_B.kstr = dynsim_2link_planar_B.b_kstr_a - 1;
        if (dynsim_2link_planar_B.a_m[dynsim_2link_planar_B.kstr] != bname->
            data[dynsim_2link_planar_B.kstr]) {
          exitg1 = 1;
        } else {
          dynsim_2link_planar_B.b_kstr_a++;
        }
      } else {
        dynsim_2link_planar_B.b_bool_b = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (dynsim_2link_planar_B.b_bool_b) {
    memset(&dynsim_2link_planar_B.T2inv[0], 0, sizeof(real_T) << 4U);
    dynsim_2link_planar_B.T2inv[0] = 1.0;
    dynsim_2link_planar_B.T2inv[5] = 1.0;
    dynsim_2link_planar_B.T2inv[10] = 1.0;
    dynsim_2link_planar_B.T2inv[15] = 1.0;
    memset(&dynsim_2link_planar_B.T2_c[0], 0, sizeof(real_T) << 4U);
    dynsim_2link_planar_B.T2_c[0] = 1.0;
    dynsim_2link_planar_B.T2_c[5] = 1.0;
    dynsim_2link_planar_B.T2_c[10] = 1.0;
    dynsim_2link_planar_B.T2_c[15] = 1.0;
  } else {
    dynsim_2link_planar_B.endeffectorIndex = -1.0;
    dynsim_2link_planar_B.b_kstr_a = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = obj->Base.NameInternal->size[1];
    dynsim_emxEnsureCapacity_char_T(bname, dynsim_2link_planar_B.b_kstr_a);
    dynsim_2link_planar_B.loop_ub = obj->Base.NameInternal->size[0] *
      obj->Base.NameInternal->size[1] - 1;
    for (dynsim_2link_planar_B.b_kstr_a = 0; dynsim_2link_planar_B.b_kstr_a <=
         dynsim_2link_planar_B.loop_ub; dynsim_2link_planar_B.b_kstr_a++) {
      bname->data[dynsim_2link_planar_B.b_kstr_a] = obj->Base.NameInternal->
        data[dynsim_2link_planar_B.b_kstr_a];
    }

    for (dynsim_2link_planar_B.b_kstr_a = 0; dynsim_2link_planar_B.b_kstr_a < 11;
         dynsim_2link_planar_B.b_kstr_a++) {
      dynsim_2link_planar_B.a_m[dynsim_2link_planar_B.b_kstr_a] =
        tmp[dynsim_2link_planar_B.b_kstr_a];
    }

    dynsim_2link_planar_B.b_bool_b = false;
    if (bname->size[1] == 11) {
      dynsim_2link_planar_B.b_kstr_a = 1;
      do {
        exitg1 = 0;
        if (dynsim_2link_planar_B.b_kstr_a - 1 < 11) {
          dynsim_2link_planar_B.kstr = dynsim_2link_planar_B.b_kstr_a - 1;
          if (bname->data[dynsim_2link_planar_B.kstr] !=
              dynsim_2link_planar_B.a_m[dynsim_2link_planar_B.kstr]) {
            exitg1 = 1;
          } else {
            dynsim_2link_planar_B.b_kstr_a++;
          }
        } else {
          dynsim_2link_planar_B.b_bool_b = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynsim_2link_planar_B.b_bool_b) {
      dynsim_2link_planar_B.endeffectorIndex = 0.0;
    } else {
      dynsim_2link_planar_B.idx_idx_1 = obj->NumBodies;
      dynsim_2link_planar_B.b_i = 0;
      exitg2 = false;
      while ((!exitg2) && (dynsim_2link_planar_B.b_i <= static_cast<int32_T>
                           (dynsim_2link_planar_B.idx_idx_1) - 1)) {
        body = obj->Bodies[dynsim_2link_planar_B.b_i];
        dynsim_2link_planar_B.b_kstr_a = bname->size[0] * bname->size[1];
        bname->size[0] = 1;
        bname->size[1] = body->NameInternal->size[1];
        dynsim_emxEnsureCapacity_char_T(bname, dynsim_2link_planar_B.b_kstr_a);
        dynsim_2link_planar_B.loop_ub = body->NameInternal->size[0] *
          body->NameInternal->size[1] - 1;
        for (dynsim_2link_planar_B.b_kstr_a = 0; dynsim_2link_planar_B.b_kstr_a <=
             dynsim_2link_planar_B.loop_ub; dynsim_2link_planar_B.b_kstr_a++) {
          bname->data[dynsim_2link_planar_B.b_kstr_a] = body->NameInternal->
            data[dynsim_2link_planar_B.b_kstr_a];
        }

        for (dynsim_2link_planar_B.b_kstr_a = 0; dynsim_2link_planar_B.b_kstr_a <
             11; dynsim_2link_planar_B.b_kstr_a++) {
          dynsim_2link_planar_B.a_m[dynsim_2link_planar_B.b_kstr_a] =
            tmp[dynsim_2link_planar_B.b_kstr_a];
        }

        dynsim_2link_planar_B.b_bool_b = false;
        if (bname->size[1] == 11) {
          dynsim_2link_planar_B.b_kstr_a = 1;
          do {
            exitg1 = 0;
            if (dynsim_2link_planar_B.b_kstr_a - 1 < 11) {
              dynsim_2link_planar_B.kstr = dynsim_2link_planar_B.b_kstr_a - 1;
              if (bname->data[dynsim_2link_planar_B.kstr] !=
                  dynsim_2link_planar_B.a_m[dynsim_2link_planar_B.kstr]) {
                exitg1 = 1;
              } else {
                dynsim_2link_planar_B.b_kstr_a++;
              }
            } else {
              dynsim_2link_planar_B.b_bool_b = true;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }

        if (dynsim_2link_planar_B.b_bool_b) {
          dynsim_2link_planar_B.endeffectorIndex = static_cast<real_T>
            (dynsim_2link_planar_B.b_i) + 1.0;
          exitg2 = true;
        } else {
          dynsim_2link_planar_B.b_i++;
        }
      }
    }

    dynsim_2link_planar_B.b_i = static_cast<int32_T>
      (dynsim_2link_planar_B.endeffectorIndex) - 1;
    body = obj->Bodies[dynsim_2link_planar_B.b_i];
    for (dynsim_2link_planar_B.b_kstr_a = 0; dynsim_2link_planar_B.b_kstr_a < 16;
         dynsim_2link_planar_B.b_kstr_a++) {
      dynsim_2link_planar_B.T2_c[dynsim_2link_planar_B.b_kstr_a] = Ttree->
        data[dynsim_2link_planar_B.b_i].f1[dynsim_2link_planar_B.b_kstr_a];
    }

    for (dynsim_2link_planar_B.b_kstr_a = 0; dynsim_2link_planar_B.b_kstr_a < 3;
         dynsim_2link_planar_B.b_kstr_a++) {
      dynsim_2link_planar_B.R_g[3 * dynsim_2link_planar_B.b_kstr_a] =
        Ttree->data[dynsim_2link_planar_B.b_i].f1[dynsim_2link_planar_B.b_kstr_a];
      dynsim_2link_planar_B.R_g[3 * dynsim_2link_planar_B.b_kstr_a + 1] =
        Ttree->data[dynsim_2link_planar_B.b_i].f1[dynsim_2link_planar_B.b_kstr_a
        + 4];
      dynsim_2link_planar_B.R_g[3 * dynsim_2link_planar_B.b_kstr_a + 2] =
        Ttree->data[dynsim_2link_planar_B.b_i].f1[dynsim_2link_planar_B.b_kstr_a
        + 8];
    }

    for (dynsim_2link_planar_B.b_kstr_a = 0; dynsim_2link_planar_B.b_kstr_a < 9;
         dynsim_2link_planar_B.b_kstr_a++) {
      dynsim_2link_planar_B.R_l[dynsim_2link_planar_B.b_kstr_a] =
        -dynsim_2link_planar_B.R_g[dynsim_2link_planar_B.b_kstr_a];
    }

    for (dynsim_2link_planar_B.b_kstr_a = 0; dynsim_2link_planar_B.b_kstr_a < 3;
         dynsim_2link_planar_B.b_kstr_a++) {
      dynsim_2link_planar_B.endeffectorIndex = Ttree->
        data[dynsim_2link_planar_B.b_i].f1[12] *
        dynsim_2link_planar_B.R_l[dynsim_2link_planar_B.b_kstr_a];
      dynsim_2link_planar_B.loop_ub = dynsim_2link_planar_B.b_kstr_a << 2;
      dynsim_2link_planar_B.T2inv[dynsim_2link_planar_B.loop_ub] =
        dynsim_2link_planar_B.R_g[3 * dynsim_2link_planar_B.b_kstr_a];
      dynsim_2link_planar_B.endeffectorIndex +=
        dynsim_2link_planar_B.R_l[dynsim_2link_planar_B.b_kstr_a + 3] *
        Ttree->data[dynsim_2link_planar_B.b_i].f1[13];
      dynsim_2link_planar_B.T2inv[dynsim_2link_planar_B.loop_ub + 1] =
        dynsim_2link_planar_B.R_g[3 * dynsim_2link_planar_B.b_kstr_a + 1];
      dynsim_2link_planar_B.endeffectorIndex +=
        dynsim_2link_planar_B.R_l[dynsim_2link_planar_B.b_kstr_a + 6] *
        Ttree->data[dynsim_2link_planar_B.b_i].f1[14];
      dynsim_2link_planar_B.T2inv[dynsim_2link_planar_B.loop_ub + 2] =
        dynsim_2link_planar_B.R_g[3 * dynsim_2link_planar_B.b_kstr_a + 2];
      dynsim_2link_planar_B.T2inv[dynsim_2link_planar_B.b_kstr_a + 12] =
        dynsim_2link_planar_B.endeffectorIndex;
    }

    dynsim_2link_planar_B.T2inv[3] = 0.0;
    dynsim_2link_planar_B.T2inv[7] = 0.0;
    dynsim_2link_planar_B.T2inv[11] = 0.0;
    dynsim_2link_planar_B.T2inv[15] = 1.0;
    dynsim_2link_planar_B.chainmask[dynsim_2link_planar_B.b_i] = 1;
    while (body->ParentIndex > 0.0) {
      body = obj->Bodies[static_cast<int32_T>(body->ParentIndex) - 1];
      dynsim_2link_planar_B.chainmask[static_cast<int32_T>(body->Index) - 1] = 1;
    }
  }

  dynsim_2link_planar_B.idx_idx_1 = obj->NumBodies;
  dynsim_2link_planar_B.c_d = static_cast<int32_T>
    (dynsim_2link_planar_B.idx_idx_1) - 1;
  dynsim_2link_pla_emxInit_real_T(&JacSlice, 2);
  dynsim_2link_pla_emxInit_real_T(&b, 2);
  if (0 <= dynsim_2link_planar_B.c_d) {
    for (dynsim_2link_planar_B.b_kstr_a = 0; dynsim_2link_planar_B.b_kstr_a < 5;
         dynsim_2link_planar_B.b_kstr_a++) {
      dynsim_2link_planar_B.b_b[dynsim_2link_planar_B.b_kstr_a] =
        tmp_0[dynsim_2link_planar_B.b_kstr_a];
    }
  }

  for (dynsim_2link_planar_B.b_i = 0; dynsim_2link_planar_B.b_i <=
       dynsim_2link_planar_B.c_d; dynsim_2link_planar_B.b_i++) {
    body = obj->Bodies[dynsim_2link_planar_B.b_i];
    dynsim_2link_planar_B.b_kstr_a = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = body->JointInternal.Type->size[1];
    dynsim_emxEnsureCapacity_char_T(bname, dynsim_2link_planar_B.b_kstr_a);
    dynsim_2link_planar_B.loop_ub = body->JointInternal.Type->size[0] *
      body->JointInternal.Type->size[1] - 1;
    for (dynsim_2link_planar_B.b_kstr_a = 0; dynsim_2link_planar_B.b_kstr_a <=
         dynsim_2link_planar_B.loop_ub; dynsim_2link_planar_B.b_kstr_a++) {
      bname->data[dynsim_2link_planar_B.b_kstr_a] = body->
        JointInternal.Type->data[dynsim_2link_planar_B.b_kstr_a];
    }

    dynsim_2link_planar_B.b_bool_b = false;
    if (bname->size[1] == 5) {
      dynsim_2link_planar_B.b_kstr_a = 1;
      do {
        exitg1 = 0;
        if (dynsim_2link_planar_B.b_kstr_a - 1 < 5) {
          dynsim_2link_planar_B.kstr = dynsim_2link_planar_B.b_kstr_a - 1;
          if (bname->data[dynsim_2link_planar_B.kstr] !=
              dynsim_2link_planar_B.b_b[dynsim_2link_planar_B.kstr]) {
            exitg1 = 1;
          } else {
            dynsim_2link_planar_B.b_kstr_a++;
          }
        } else {
          dynsim_2link_planar_B.b_bool_b = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if ((!dynsim_2link_planar_B.b_bool_b) &&
        (dynsim_2link_planar_B.chainmask[dynsim_2link_planar_B.b_i] != 0)) {
      for (dynsim_2link_planar_B.b_kstr_a = 0; dynsim_2link_planar_B.b_kstr_a <
           16; dynsim_2link_planar_B.b_kstr_a++) {
        dynsim_2link_planar_B.T1_b[dynsim_2link_planar_B.b_kstr_a] = Ttree->
          data[static_cast<int32_T>(body->Index) - 1]
          .f1[dynsim_2link_planar_B.b_kstr_a];
      }

      for (dynsim_2link_planar_B.b_kstr_a = 0; dynsim_2link_planar_B.b_kstr_a <
           16; dynsim_2link_planar_B.b_kstr_a++) {
        dynsim_2link_planar_B.Tdh[dynsim_2link_planar_B.b_kstr_a] =
          body->
          JointInternal.ChildToJointTransform[dynsim_2link_planar_B.b_kstr_a];
      }

      for (dynsim_2link_planar_B.b_kstr_a = 0; dynsim_2link_planar_B.b_kstr_a <
           3; dynsim_2link_planar_B.b_kstr_a++) {
        dynsim_2link_planar_B.R_g[3 * dynsim_2link_planar_B.b_kstr_a] =
          dynsim_2link_planar_B.Tdh[dynsim_2link_planar_B.b_kstr_a];
        dynsim_2link_planar_B.R_g[3 * dynsim_2link_planar_B.b_kstr_a + 1] =
          dynsim_2link_planar_B.Tdh[dynsim_2link_planar_B.b_kstr_a + 4];
        dynsim_2link_planar_B.R_g[3 * dynsim_2link_planar_B.b_kstr_a + 2] =
          dynsim_2link_planar_B.Tdh[dynsim_2link_planar_B.b_kstr_a + 8];
      }

      for (dynsim_2link_planar_B.b_kstr_a = 0; dynsim_2link_planar_B.b_kstr_a <
           9; dynsim_2link_planar_B.b_kstr_a++) {
        dynsim_2link_planar_B.R_l[dynsim_2link_planar_B.b_kstr_a] =
          -dynsim_2link_planar_B.R_g[dynsim_2link_planar_B.b_kstr_a];
      }

      for (dynsim_2link_planar_B.b_kstr_a = 0; dynsim_2link_planar_B.b_kstr_a <
           3; dynsim_2link_planar_B.b_kstr_a++) {
        dynsim_2link_planar_B.R_o4[dynsim_2link_planar_B.b_kstr_a] =
          dynsim_2link_planar_B.R_l[dynsim_2link_planar_B.b_kstr_a + 6] *
          dynsim_2link_planar_B.Tdh[14] +
          (dynsim_2link_planar_B.R_l[dynsim_2link_planar_B.b_kstr_a + 3] *
           dynsim_2link_planar_B.Tdh[13] +
           dynsim_2link_planar_B.R_l[dynsim_2link_planar_B.b_kstr_a] *
           dynsim_2link_planar_B.Tdh[12]);
      }

      for (dynsim_2link_planar_B.b_kstr_a = 0; dynsim_2link_planar_B.b_kstr_a <
           4; dynsim_2link_planar_B.b_kstr_a++) {
        for (dynsim_2link_planar_B.kstr = 0; dynsim_2link_planar_B.kstr < 4;
             dynsim_2link_planar_B.kstr++) {
          dynsim_2link_planar_B.n_p = dynsim_2link_planar_B.kstr << 2;
          dynsim_2link_planar_B.loop_ub = dynsim_2link_planar_B.b_kstr_a +
            dynsim_2link_planar_B.n_p;
          dynsim_2link_planar_B.Tdh[dynsim_2link_planar_B.loop_ub] = 0.0;
          dynsim_2link_planar_B.Tdh[dynsim_2link_planar_B.loop_ub] +=
            dynsim_2link_planar_B.T1_b[dynsim_2link_planar_B.n_p] *
            dynsim_2link_planar_B.T2inv[dynsim_2link_planar_B.b_kstr_a];
          dynsim_2link_planar_B.Tdh[dynsim_2link_planar_B.loop_ub] +=
            dynsim_2link_planar_B.T1_b[dynsim_2link_planar_B.n_p + 1] *
            dynsim_2link_planar_B.T2inv[dynsim_2link_planar_B.b_kstr_a + 4];
          dynsim_2link_planar_B.Tdh[dynsim_2link_planar_B.loop_ub] +=
            dynsim_2link_planar_B.T1_b[dynsim_2link_planar_B.n_p + 2] *
            dynsim_2link_planar_B.T2inv[dynsim_2link_planar_B.b_kstr_a + 8];
          dynsim_2link_planar_B.Tdh[dynsim_2link_planar_B.loop_ub] +=
            dynsim_2link_planar_B.T1_b[dynsim_2link_planar_B.n_p + 3] *
            dynsim_2link_planar_B.T2inv[dynsim_2link_planar_B.b_kstr_a + 12];
        }
      }

      for (dynsim_2link_planar_B.b_kstr_a = 0; dynsim_2link_planar_B.b_kstr_a <
           3; dynsim_2link_planar_B.b_kstr_a++) {
        dynsim_2link_planar_B.kstr = dynsim_2link_planar_B.b_kstr_a << 2;
        dynsim_2link_planar_B.T1_b[dynsim_2link_planar_B.kstr] =
          dynsim_2link_planar_B.R_g[3 * dynsim_2link_planar_B.b_kstr_a];
        dynsim_2link_planar_B.T1_b[dynsim_2link_planar_B.kstr + 1] =
          dynsim_2link_planar_B.R_g[3 * dynsim_2link_planar_B.b_kstr_a + 1];
        dynsim_2link_planar_B.T1_b[dynsim_2link_planar_B.kstr + 2] =
          dynsim_2link_planar_B.R_g[3 * dynsim_2link_planar_B.b_kstr_a + 2];
        dynsim_2link_planar_B.T1_b[dynsim_2link_planar_B.b_kstr_a + 12] =
          dynsim_2link_planar_B.R_o4[dynsim_2link_planar_B.b_kstr_a];
      }

      dynsim_2link_planar_B.T1_b[3] = 0.0;
      dynsim_2link_planar_B.T1_b[7] = 0.0;
      dynsim_2link_planar_B.T1_b[11] = 0.0;
      dynsim_2link_planar_B.T1_b[15] = 1.0;
      for (dynsim_2link_planar_B.b_kstr_a = 0; dynsim_2link_planar_B.b_kstr_a <
           4; dynsim_2link_planar_B.b_kstr_a++) {
        for (dynsim_2link_planar_B.kstr = 0; dynsim_2link_planar_B.kstr < 4;
             dynsim_2link_planar_B.kstr++) {
          dynsim_2link_planar_B.loop_ub = dynsim_2link_planar_B.kstr << 2;
          dynsim_2link_planar_B.n_p = dynsim_2link_planar_B.b_kstr_a +
            dynsim_2link_planar_B.loop_ub;
          dynsim_2link_planar_B.T[dynsim_2link_planar_B.n_p] = 0.0;
          dynsim_2link_planar_B.T[dynsim_2link_planar_B.n_p] +=
            dynsim_2link_planar_B.T1_b[dynsim_2link_planar_B.loop_ub] *
            dynsim_2link_planar_B.Tdh[dynsim_2link_planar_B.b_kstr_a];
          dynsim_2link_planar_B.T[dynsim_2link_planar_B.n_p] +=
            dynsim_2link_planar_B.T1_b[dynsim_2link_planar_B.loop_ub + 1] *
            dynsim_2link_planar_B.Tdh[dynsim_2link_planar_B.b_kstr_a + 4];
          dynsim_2link_planar_B.T[dynsim_2link_planar_B.n_p] +=
            dynsim_2link_planar_B.T1_b[dynsim_2link_planar_B.loop_ub + 2] *
            dynsim_2link_planar_B.Tdh[dynsim_2link_planar_B.b_kstr_a + 8];
          dynsim_2link_planar_B.T[dynsim_2link_planar_B.n_p] +=
            dynsim_2link_planar_B.T1_b[dynsim_2link_planar_B.loop_ub + 3] *
            dynsim_2link_planar_B.Tdh[dynsim_2link_planar_B.b_kstr_a + 12];
        }
      }

      dynsim_2link_planar_B.endeffectorIndex = obj->
        PositionDoFMap[dynsim_2link_planar_B.b_i];
      dynsim_2link_planar_B.idx_idx_1 = obj->
        PositionDoFMap[dynsim_2link_planar_B.b_i + 8];
      dynsim_2link_planar_B.R_g[0] = 0.0;
      dynsim_2link_planar_B.R_g[3] = -dynsim_2link_planar_B.T[14];
      dynsim_2link_planar_B.R_g[6] = dynsim_2link_planar_B.T[13];
      dynsim_2link_planar_B.R_g[1] = dynsim_2link_planar_B.T[14];
      dynsim_2link_planar_B.R_g[4] = 0.0;
      dynsim_2link_planar_B.R_g[7] = -dynsim_2link_planar_B.T[12];
      dynsim_2link_planar_B.R_g[2] = -dynsim_2link_planar_B.T[13];
      dynsim_2link_planar_B.R_g[5] = dynsim_2link_planar_B.T[12];
      dynsim_2link_planar_B.R_g[8] = 0.0;
      for (dynsim_2link_planar_B.b_kstr_a = 0; dynsim_2link_planar_B.b_kstr_a <
           3; dynsim_2link_planar_B.b_kstr_a++) {
        for (dynsim_2link_planar_B.kstr = 0; dynsim_2link_planar_B.kstr < 3;
             dynsim_2link_planar_B.kstr++) {
          dynsim_2link_planar_B.loop_ub = dynsim_2link_planar_B.b_kstr_a + 3 *
            dynsim_2link_planar_B.kstr;
          dynsim_2link_planar_B.R_l[dynsim_2link_planar_B.loop_ub] = 0.0;
          dynsim_2link_planar_B.n_p = dynsim_2link_planar_B.kstr << 2;
          dynsim_2link_planar_B.R_l[dynsim_2link_planar_B.loop_ub] +=
            dynsim_2link_planar_B.T[dynsim_2link_planar_B.n_p] *
            dynsim_2link_planar_B.R_g[dynsim_2link_planar_B.b_kstr_a];
          dynsim_2link_planar_B.R_l[dynsim_2link_planar_B.loop_ub] +=
            dynsim_2link_planar_B.T[dynsim_2link_planar_B.n_p + 1] *
            dynsim_2link_planar_B.R_g[dynsim_2link_planar_B.b_kstr_a + 3];
          dynsim_2link_planar_B.R_l[dynsim_2link_planar_B.loop_ub] +=
            dynsim_2link_planar_B.T[dynsim_2link_planar_B.n_p + 2] *
            dynsim_2link_planar_B.R_g[dynsim_2link_planar_B.b_kstr_a + 6];
          dynsim_2link_planar_B.X[dynsim_2link_planar_B.kstr + 6 *
            dynsim_2link_planar_B.b_kstr_a] = dynsim_2link_planar_B.T
            [(dynsim_2link_planar_B.b_kstr_a << 2) + dynsim_2link_planar_B.kstr];
          dynsim_2link_planar_B.X[dynsim_2link_planar_B.kstr + 6 *
            (dynsim_2link_planar_B.b_kstr_a + 3)] = 0.0;
        }
      }

      for (dynsim_2link_planar_B.b_kstr_a = 0; dynsim_2link_planar_B.b_kstr_a <
           3; dynsim_2link_planar_B.b_kstr_a++) {
        dynsim_2link_planar_B.X[6 * dynsim_2link_planar_B.b_kstr_a + 3] =
          dynsim_2link_planar_B.R_l[3 * dynsim_2link_planar_B.b_kstr_a];
        dynsim_2link_planar_B.kstr = dynsim_2link_planar_B.b_kstr_a << 2;
        dynsim_2link_planar_B.loop_ub = 6 * (dynsim_2link_planar_B.b_kstr_a + 3);
        dynsim_2link_planar_B.X[dynsim_2link_planar_B.loop_ub + 3] =
          dynsim_2link_planar_B.T[dynsim_2link_planar_B.kstr];
        dynsim_2link_planar_B.X[6 * dynsim_2link_planar_B.b_kstr_a + 4] =
          dynsim_2link_planar_B.R_l[3 * dynsim_2link_planar_B.b_kstr_a + 1];
        dynsim_2link_planar_B.X[dynsim_2link_planar_B.loop_ub + 4] =
          dynsim_2link_planar_B.T[dynsim_2link_planar_B.kstr + 1];
        dynsim_2link_planar_B.X[6 * dynsim_2link_planar_B.b_kstr_a + 5] =
          dynsim_2link_planar_B.R_l[3 * dynsim_2link_planar_B.b_kstr_a + 2];
        dynsim_2link_planar_B.X[dynsim_2link_planar_B.loop_ub + 5] =
          dynsim_2link_planar_B.T[dynsim_2link_planar_B.kstr + 2];
      }

      dynsim_2link_planar_B.b_kstr_a = b->size[0] * b->size[1];
      b->size[0] = 6;
      b->size[1] = body->JointInternal.MotionSubspace->size[1];
      dynsim_emxEnsureCapacity_real_T(b, dynsim_2link_planar_B.b_kstr_a);
      dynsim_2link_planar_B.loop_ub = body->JointInternal.MotionSubspace->size[0]
        * body->JointInternal.MotionSubspace->size[1] - 1;
      for (dynsim_2link_planar_B.b_kstr_a = 0; dynsim_2link_planar_B.b_kstr_a <=
           dynsim_2link_planar_B.loop_ub; dynsim_2link_planar_B.b_kstr_a++) {
        b->data[dynsim_2link_planar_B.b_kstr_a] =
          body->JointInternal.MotionSubspace->
          data[dynsim_2link_planar_B.b_kstr_a];
      }

      dynsim_2link_planar_B.n_p = b->size[1] - 1;
      dynsim_2link_planar_B.b_kstr_a = JacSlice->size[0] * JacSlice->size[1];
      JacSlice->size[0] = 6;
      JacSlice->size[1] = b->size[1];
      dynsim_emxEnsureCapacity_real_T(JacSlice, dynsim_2link_planar_B.b_kstr_a);
      for (dynsim_2link_planar_B.b_kstr_a = 0; dynsim_2link_planar_B.b_kstr_a <=
           dynsim_2link_planar_B.n_p; dynsim_2link_planar_B.b_kstr_a++) {
        dynsim_2link_planar_B.coffset_tmp = dynsim_2link_planar_B.b_kstr_a * 6 -
          1;
        for (dynsim_2link_planar_B.kstr = 0; dynsim_2link_planar_B.kstr < 6;
             dynsim_2link_planar_B.kstr++) {
          dynsim_2link_planar_B.s_i = 0.0;
          for (dynsim_2link_planar_B.loop_ub = 0; dynsim_2link_planar_B.loop_ub <
               6; dynsim_2link_planar_B.loop_ub++) {
            dynsim_2link_planar_B.s_i +=
              dynsim_2link_planar_B.X[dynsim_2link_planar_B.loop_ub * 6 +
              dynsim_2link_planar_B.kstr] * b->data
              [(dynsim_2link_planar_B.coffset_tmp +
                dynsim_2link_planar_B.loop_ub) + 1];
          }

          JacSlice->data[(dynsim_2link_planar_B.coffset_tmp +
                          dynsim_2link_planar_B.kstr) + 1] =
            dynsim_2link_planar_B.s_i;
        }
      }

      if (dynsim_2link_planar_B.endeffectorIndex >
          dynsim_2link_planar_B.idx_idx_1) {
        dynsim_2link_planar_B.n_p = 0;
      } else {
        dynsim_2link_planar_B.n_p = static_cast<int32_T>
          (dynsim_2link_planar_B.endeffectorIndex) - 1;
      }

      dynsim_2link_planar_B.loop_ub = JacSlice->size[1];
      for (dynsim_2link_planar_B.b_kstr_a = 0; dynsim_2link_planar_B.b_kstr_a <
           dynsim_2link_planar_B.loop_ub; dynsim_2link_planar_B.b_kstr_a++) {
        for (dynsim_2link_planar_B.kstr = 0; dynsim_2link_planar_B.kstr < 6;
             dynsim_2link_planar_B.kstr++) {
          Jac->data[dynsim_2link_planar_B.kstr + 6 * (dynsim_2link_planar_B.n_p
            + dynsim_2link_planar_B.b_kstr_a)] = JacSlice->data[6 *
            dynsim_2link_planar_B.b_kstr_a + dynsim_2link_planar_B.kstr];
        }
      }
    }
  }

  dynsim_2link_pla_emxFree_char_T(&bname);
  dynsim_2link_pla_emxFree_real_T(&JacSlice);
  dynsim_2lin_emxFree_f_cell_wrap(&Ttree);
  for (dynsim_2link_planar_B.b_kstr_a = 0; dynsim_2link_planar_B.b_kstr_a < 3;
       dynsim_2link_planar_B.b_kstr_a++) {
    dynsim_2link_planar_B.b_i = dynsim_2link_planar_B.b_kstr_a << 2;
    dynsim_2link_planar_B.X[6 * dynsim_2link_planar_B.b_kstr_a] =
      dynsim_2link_planar_B.T2_c[dynsim_2link_planar_B.b_i];
    dynsim_2link_planar_B.kstr = 6 * (dynsim_2link_planar_B.b_kstr_a + 3);
    dynsim_2link_planar_B.X[dynsim_2link_planar_B.kstr] = 0.0;
    dynsim_2link_planar_B.X[6 * dynsim_2link_planar_B.b_kstr_a + 3] = 0.0;
    dynsim_2link_planar_B.X[dynsim_2link_planar_B.kstr + 3] =
      dynsim_2link_planar_B.T2_c[dynsim_2link_planar_B.b_i];
    dynsim_2link_planar_B.endeffectorIndex =
      dynsim_2link_planar_B.T2_c[dynsim_2link_planar_B.b_i + 1];
    dynsim_2link_planar_B.X[6 * dynsim_2link_planar_B.b_kstr_a + 1] =
      dynsim_2link_planar_B.endeffectorIndex;
    dynsim_2link_planar_B.X[dynsim_2link_planar_B.kstr + 1] = 0.0;
    dynsim_2link_planar_B.X[6 * dynsim_2link_planar_B.b_kstr_a + 4] = 0.0;
    dynsim_2link_planar_B.X[dynsim_2link_planar_B.kstr + 4] =
      dynsim_2link_planar_B.endeffectorIndex;
    dynsim_2link_planar_B.endeffectorIndex =
      dynsim_2link_planar_B.T2_c[dynsim_2link_planar_B.b_i + 2];
    dynsim_2link_planar_B.X[6 * dynsim_2link_planar_B.b_kstr_a + 2] =
      dynsim_2link_planar_B.endeffectorIndex;
    dynsim_2link_planar_B.X[dynsim_2link_planar_B.kstr + 2] = 0.0;
    dynsim_2link_planar_B.X[6 * dynsim_2link_planar_B.b_kstr_a + 5] = 0.0;
    dynsim_2link_planar_B.X[dynsim_2link_planar_B.kstr + 5] =
      dynsim_2link_planar_B.endeffectorIndex;
  }

  dynsim_2link_planar_B.n_p = Jac->size[1];
  dynsim_2link_planar_B.b_kstr_a = b->size[0] * b->size[1];
  b->size[0] = 6;
  b->size[1] = Jac->size[1];
  dynsim_emxEnsureCapacity_real_T(b, dynsim_2link_planar_B.b_kstr_a);
  dynsim_2link_planar_B.loop_ub = Jac->size[0] * Jac->size[1] - 1;
  for (dynsim_2link_planar_B.b_kstr_a = 0; dynsim_2link_planar_B.b_kstr_a <=
       dynsim_2link_planar_B.loop_ub; dynsim_2link_planar_B.b_kstr_a++) {
    b->data[dynsim_2link_planar_B.b_kstr_a] = Jac->
      data[dynsim_2link_planar_B.b_kstr_a];
  }

  dynsim_2link_planar_B.b_kstr_a = Jac->size[0] * Jac->size[1];
  Jac->size[0] = 6;
  Jac->size[1] = dynsim_2link_planar_B.n_p;
  dynsim_emxEnsureCapacity_real_T(Jac, dynsim_2link_planar_B.b_kstr_a);
  for (dynsim_2link_planar_B.b_kstr_a = 0; dynsim_2link_planar_B.b_kstr_a <
       dynsim_2link_planar_B.n_p; dynsim_2link_planar_B.b_kstr_a++) {
    dynsim_2link_planar_B.coffset_tmp = dynsim_2link_planar_B.b_kstr_a * 6 - 1;
    for (dynsim_2link_planar_B.b_i = 0; dynsim_2link_planar_B.b_i < 6;
         dynsim_2link_planar_B.b_i++) {
      dynsim_2link_planar_B.s_i = 0.0;
      for (dynsim_2link_planar_B.loop_ub = 0; dynsim_2link_planar_B.loop_ub < 6;
           dynsim_2link_planar_B.loop_ub++) {
        dynsim_2link_planar_B.s_i +=
          dynsim_2link_planar_B.X[dynsim_2link_planar_B.loop_ub * 6 +
          dynsim_2link_planar_B.b_i] * b->data
          [(dynsim_2link_planar_B.coffset_tmp + dynsim_2link_planar_B.loop_ub) +
          1];
      }

      Jac->data[(dynsim_2link_planar_B.coffset_tmp + dynsim_2link_planar_B.b_i)
        + 1] = dynsim_2link_planar_B.s_i;
    }
  }

  dynsim_2link_pla_emxFree_real_T(&b);
}

static void rigidBodyJoint_get_JointAxis_aw(const
  c_rigidBodyJoint_dynsim_2l_aw_T *obj, real_T ax[3])
{
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  boolean_T guard1 = false;
  int32_T exitg1;
  for (dynsim_2link_planar_B.b_kstr_f = 0; dynsim_2link_planar_B.b_kstr_f < 8;
       dynsim_2link_planar_B.b_kstr_f++) {
    dynsim_2link_planar_B.b_as[dynsim_2link_planar_B.b_kstr_f] =
      tmp[dynsim_2link_planar_B.b_kstr_f];
  }

  dynsim_2link_planar_B.b_bool_dk = false;
  if (obj->Type->size[1] == 8) {
    dynsim_2link_planar_B.b_kstr_f = 1;
    do {
      exitg1 = 0;
      if (dynsim_2link_planar_B.b_kstr_f - 1 < 8) {
        dynsim_2link_planar_B.kstr_c = dynsim_2link_planar_B.b_kstr_f - 1;
        if (obj->Type->data[dynsim_2link_planar_B.kstr_c] !=
            dynsim_2link_planar_B.b_as[dynsim_2link_planar_B.kstr_c]) {
          exitg1 = 1;
        } else {
          dynsim_2link_planar_B.b_kstr_f++;
        }
      } else {
        dynsim_2link_planar_B.b_bool_dk = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (dynsim_2link_planar_B.b_bool_dk) {
    guard1 = true;
  } else {
    for (dynsim_2link_planar_B.b_kstr_f = 0; dynsim_2link_planar_B.b_kstr_f < 9;
         dynsim_2link_planar_B.b_kstr_f++) {
      dynsim_2link_planar_B.b_c0[dynsim_2link_planar_B.b_kstr_f] =
        tmp_0[dynsim_2link_planar_B.b_kstr_f];
    }

    dynsim_2link_planar_B.b_bool_dk = false;
    if (obj->Type->size[1] == 9) {
      dynsim_2link_planar_B.b_kstr_f = 1;
      do {
        exitg1 = 0;
        if (dynsim_2link_planar_B.b_kstr_f - 1 < 9) {
          dynsim_2link_planar_B.kstr_c = dynsim_2link_planar_B.b_kstr_f - 1;
          if (obj->Type->data[dynsim_2link_planar_B.kstr_c] !=
              dynsim_2link_planar_B.b_c0[dynsim_2link_planar_B.kstr_c]) {
            exitg1 = 1;
          } else {
            dynsim_2link_planar_B.b_kstr_f++;
          }
        } else {
          dynsim_2link_planar_B.b_bool_dk = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynsim_2link_planar_B.b_bool_dk) {
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

static void RigidBodyTree_forwardKinemati_a(p_robotics_manip_internal__aw_T *obj,
  const real_T qvec[6], emxArray_f_cell_wrap_dynsim_2_T *Ttree)
{
  n_robotics_manip_internal__aw_T *body;
  emxArray_char_T_dynsim_2link__T *switch_expression;
  static const int8_T tmp[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  dynsim_2link_planar_B.n_l = obj->NumBodies;
  for (dynsim_2link_planar_B.b_kstr_d = 0; dynsim_2link_planar_B.b_kstr_d < 16;
       dynsim_2link_planar_B.b_kstr_d++) {
    dynsim_2link_planar_B.c_f1_m[dynsim_2link_planar_B.b_kstr_d] =
      tmp[dynsim_2link_planar_B.b_kstr_d];
  }

  dynsim_2link_planar_B.b_kstr_d = Ttree->size[0] * Ttree->size[1];
  Ttree->size[0] = 1;
  dynsim_2link_planar_B.e_a = static_cast<int32_T>(dynsim_2link_planar_B.n_l);
  Ttree->size[1] = dynsim_2link_planar_B.e_a;
  emxEnsureCapacity_f_cell_wrap1(Ttree, dynsim_2link_planar_B.b_kstr_d);
  if (dynsim_2link_planar_B.e_a != 0) {
    dynsim_2link_planar_B.ntilecols_g = dynsim_2link_planar_B.e_a - 1;
    if (0 <= dynsim_2link_planar_B.ntilecols_g) {
      memcpy(&dynsim_2link_planar_B.expl_temp_c.f1[0],
             &dynsim_2link_planar_B.c_f1_m[0], sizeof(real_T) << 4U);
    }

    for (dynsim_2link_planar_B.b_jtilecol_n = 0;
         dynsim_2link_planar_B.b_jtilecol_n <= dynsim_2link_planar_B.ntilecols_g;
         dynsim_2link_planar_B.b_jtilecol_n++) {
      Ttree->data[dynsim_2link_planar_B.b_jtilecol_n] =
        dynsim_2link_planar_B.expl_temp_c;
    }
  }

  dynsim_2link_planar_B.k_h = 1.0;
  dynsim_2link_planar_B.ntilecols_g = static_cast<int32_T>
    (dynsim_2link_planar_B.n_l) - 1;
  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  if (0 <= dynsim_2link_planar_B.ntilecols_g) {
    for (dynsim_2link_planar_B.b_kstr_d = 0; dynsim_2link_planar_B.b_kstr_d < 5;
         dynsim_2link_planar_B.b_kstr_d++) {
      dynsim_2link_planar_B.b_ei[dynsim_2link_planar_B.b_kstr_d] =
        tmp_0[dynsim_2link_planar_B.b_kstr_d];
    }
  }

  for (dynsim_2link_planar_B.b_jtilecol_n = 0;
       dynsim_2link_planar_B.b_jtilecol_n <= dynsim_2link_planar_B.ntilecols_g;
       dynsim_2link_planar_B.b_jtilecol_n++) {
    body = obj->Bodies[dynsim_2link_planar_B.b_jtilecol_n];
    dynsim_2link_planar_B.n_l = body->JointInternal.PositionNumber;
    dynsim_2link_planar_B.n_l += dynsim_2link_planar_B.k_h;
    if (dynsim_2link_planar_B.k_h > dynsim_2link_planar_B.n_l - 1.0) {
      dynsim_2link_planar_B.e_a = 0;
      dynsim_2link_planar_B.d_f = 0;
    } else {
      dynsim_2link_planar_B.e_a = static_cast<int32_T>(dynsim_2link_planar_B.k_h)
        - 1;
      dynsim_2link_planar_B.d_f = static_cast<int32_T>(dynsim_2link_planar_B.n_l
        - 1.0);
    }

    dynsim_2link_planar_B.b_kstr_d = switch_expression->size[0] *
      switch_expression->size[1];
    switch_expression->size[0] = 1;
    switch_expression->size[1] = body->JointInternal.Type->size[1];
    dynsim_emxEnsureCapacity_char_T(switch_expression,
      dynsim_2link_planar_B.b_kstr_d);
    dynsim_2link_planar_B.loop_ub_n = body->JointInternal.Type->size[0] *
      body->JointInternal.Type->size[1] - 1;
    for (dynsim_2link_planar_B.b_kstr_d = 0; dynsim_2link_planar_B.b_kstr_d <=
         dynsim_2link_planar_B.loop_ub_n; dynsim_2link_planar_B.b_kstr_d++) {
      switch_expression->data[dynsim_2link_planar_B.b_kstr_d] =
        body->JointInternal.Type->data[dynsim_2link_planar_B.b_kstr_d];
    }

    dynsim_2link_planar_B.b_bool_p = false;
    if (switch_expression->size[1] == 5) {
      dynsim_2link_planar_B.b_kstr_d = 1;
      do {
        exitg1 = 0;
        if (dynsim_2link_planar_B.b_kstr_d - 1 < 5) {
          dynsim_2link_planar_B.loop_ub_n = dynsim_2link_planar_B.b_kstr_d - 1;
          if (switch_expression->data[dynsim_2link_planar_B.loop_ub_n] !=
              dynsim_2link_planar_B.b_ei[dynsim_2link_planar_B.loop_ub_n]) {
            exitg1 = 1;
          } else {
            dynsim_2link_planar_B.b_kstr_d++;
          }
        } else {
          dynsim_2link_planar_B.b_bool_p = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynsim_2link_planar_B.b_bool_p) {
      dynsim_2link_planar_B.b_kstr_d = 0;
    } else {
      for (dynsim_2link_planar_B.b_kstr_d = 0; dynsim_2link_planar_B.b_kstr_d <
           8; dynsim_2link_planar_B.b_kstr_d++) {
        dynsim_2link_planar_B.b_ax[dynsim_2link_planar_B.b_kstr_d] =
          tmp_1[dynsim_2link_planar_B.b_kstr_d];
      }

      dynsim_2link_planar_B.b_bool_p = false;
      if (switch_expression->size[1] == 8) {
        dynsim_2link_planar_B.b_kstr_d = 1;
        do {
          exitg1 = 0;
          if (dynsim_2link_planar_B.b_kstr_d - 1 < 8) {
            dynsim_2link_planar_B.loop_ub_n = dynsim_2link_planar_B.b_kstr_d - 1;
            if (switch_expression->data[dynsim_2link_planar_B.loop_ub_n] !=
                dynsim_2link_planar_B.b_ax[dynsim_2link_planar_B.loop_ub_n]) {
              exitg1 = 1;
            } else {
              dynsim_2link_planar_B.b_kstr_d++;
            }
          } else {
            dynsim_2link_planar_B.b_bool_p = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (dynsim_2link_planar_B.b_bool_p) {
        dynsim_2link_planar_B.b_kstr_d = 1;
      } else {
        dynsim_2link_planar_B.b_kstr_d = -1;
      }
    }

    switch (dynsim_2link_planar_B.b_kstr_d) {
     case 0:
      memset(&dynsim_2link_planar_B.c_f1_m[0], 0, sizeof(real_T) << 4U);
      dynsim_2link_planar_B.c_f1_m[0] = 1.0;
      dynsim_2link_planar_B.c_f1_m[5] = 1.0;
      dynsim_2link_planar_B.c_f1_m[10] = 1.0;
      dynsim_2link_planar_B.c_f1_m[15] = 1.0;
      break;

     case 1:
      rigidBodyJoint_get_JointAxis_aw(&body->JointInternal,
        dynsim_2link_planar_B.v_o);
      dynsim_2link_planar_B.d_f -= dynsim_2link_planar_B.e_a;
      for (dynsim_2link_planar_B.b_kstr_d = 0; dynsim_2link_planar_B.b_kstr_d <
           dynsim_2link_planar_B.d_f; dynsim_2link_planar_B.b_kstr_d++) {
        dynsim_2link_planar_B.e_data_j[dynsim_2link_planar_B.b_kstr_d] =
          dynsim_2link_planar_B.e_a + dynsim_2link_planar_B.b_kstr_d;
      }

      dynsim_2link_planar_B.result_data_a[0] = dynsim_2link_planar_B.v_o[0];
      dynsim_2link_planar_B.result_data_a[1] = dynsim_2link_planar_B.v_o[1];
      dynsim_2link_planar_B.result_data_a[2] = dynsim_2link_planar_B.v_o[2];
      if (0 <= (dynsim_2link_planar_B.d_f != 0) - 1) {
        dynsim_2link_planar_B.result_data_a[3] =
          qvec[dynsim_2link_planar_B.e_data_j[0]];
      }

      dynsim_2link_planar_B.k_h = 1.0 / sqrt
        ((dynsim_2link_planar_B.result_data_a[0] *
          dynsim_2link_planar_B.result_data_a[0] +
          dynsim_2link_planar_B.result_data_a[1] *
          dynsim_2link_planar_B.result_data_a[1]) +
         dynsim_2link_planar_B.result_data_a[2] *
         dynsim_2link_planar_B.result_data_a[2]);
      dynsim_2link_planar_B.v_o[0] = dynsim_2link_planar_B.result_data_a[0] *
        dynsim_2link_planar_B.k_h;
      dynsim_2link_planar_B.v_o[1] = dynsim_2link_planar_B.result_data_a[1] *
        dynsim_2link_planar_B.k_h;
      dynsim_2link_planar_B.v_o[2] = dynsim_2link_planar_B.result_data_a[2] *
        dynsim_2link_planar_B.k_h;
      dynsim_2link_planar_B.k_h = cos(dynsim_2link_planar_B.result_data_a[3]);
      dynsim_2link_planar_B.sth_m = sin(dynsim_2link_planar_B.result_data_a[3]);
      dynsim_2link_planar_B.tempR_h[0] = dynsim_2link_planar_B.v_o[0] *
        dynsim_2link_planar_B.v_o[0] * (1.0 - dynsim_2link_planar_B.k_h) +
        dynsim_2link_planar_B.k_h;
      dynsim_2link_planar_B.tempR_tmp_m = dynsim_2link_planar_B.v_o[1] *
        dynsim_2link_planar_B.v_o[0] * (1.0 - dynsim_2link_planar_B.k_h);
      dynsim_2link_planar_B.tempR_tmp_h = dynsim_2link_planar_B.v_o[2] *
        dynsim_2link_planar_B.sth_m;
      dynsim_2link_planar_B.tempR_h[1] = dynsim_2link_planar_B.tempR_tmp_m -
        dynsim_2link_planar_B.tempR_tmp_h;
      dynsim_2link_planar_B.tempR_tmp_cs = dynsim_2link_planar_B.v_o[2] *
        dynsim_2link_planar_B.v_o[0] * (1.0 - dynsim_2link_planar_B.k_h);
      dynsim_2link_planar_B.tempR_tmp_k = dynsim_2link_planar_B.v_o[1] *
        dynsim_2link_planar_B.sth_m;
      dynsim_2link_planar_B.tempR_h[2] = dynsim_2link_planar_B.tempR_tmp_cs +
        dynsim_2link_planar_B.tempR_tmp_k;
      dynsim_2link_planar_B.tempR_h[3] = dynsim_2link_planar_B.tempR_tmp_m +
        dynsim_2link_planar_B.tempR_tmp_h;
      dynsim_2link_planar_B.tempR_h[4] = dynsim_2link_planar_B.v_o[1] *
        dynsim_2link_planar_B.v_o[1] * (1.0 - dynsim_2link_planar_B.k_h) +
        dynsim_2link_planar_B.k_h;
      dynsim_2link_planar_B.tempR_tmp_m = dynsim_2link_planar_B.v_o[2] *
        dynsim_2link_planar_B.v_o[1] * (1.0 - dynsim_2link_planar_B.k_h);
      dynsim_2link_planar_B.tempR_tmp_h = dynsim_2link_planar_B.v_o[0] *
        dynsim_2link_planar_B.sth_m;
      dynsim_2link_planar_B.tempR_h[5] = dynsim_2link_planar_B.tempR_tmp_m -
        dynsim_2link_planar_B.tempR_tmp_h;
      dynsim_2link_planar_B.tempR_h[6] = dynsim_2link_planar_B.tempR_tmp_cs -
        dynsim_2link_planar_B.tempR_tmp_k;
      dynsim_2link_planar_B.tempR_h[7] = dynsim_2link_planar_B.tempR_tmp_m +
        dynsim_2link_planar_B.tempR_tmp_h;
      dynsim_2link_planar_B.tempR_h[8] = dynsim_2link_planar_B.v_o[2] *
        dynsim_2link_planar_B.v_o[2] * (1.0 - dynsim_2link_planar_B.k_h) +
        dynsim_2link_planar_B.k_h;
      for (dynsim_2link_planar_B.b_kstr_d = 0; dynsim_2link_planar_B.b_kstr_d <
           3; dynsim_2link_planar_B.b_kstr_d++) {
        dynsim_2link_planar_B.e_a = dynsim_2link_planar_B.b_kstr_d + 1;
        dynsim_2link_planar_B.R_ln[dynsim_2link_planar_B.e_a - 1] =
          dynsim_2link_planar_B.tempR_h[(dynsim_2link_planar_B.e_a - 1) * 3];
        dynsim_2link_planar_B.e_a = dynsim_2link_planar_B.b_kstr_d + 1;
        dynsim_2link_planar_B.R_ln[dynsim_2link_planar_B.e_a + 2] =
          dynsim_2link_planar_B.tempR_h[(dynsim_2link_planar_B.e_a - 1) * 3 + 1];
        dynsim_2link_planar_B.e_a = dynsim_2link_planar_B.b_kstr_d + 1;
        dynsim_2link_planar_B.R_ln[dynsim_2link_planar_B.e_a + 5] =
          dynsim_2link_planar_B.tempR_h[(dynsim_2link_planar_B.e_a - 1) * 3 + 2];
      }

      memset(&dynsim_2link_planar_B.c_f1_m[0], 0, sizeof(real_T) << 4U);
      for (dynsim_2link_planar_B.b_kstr_d = 0; dynsim_2link_planar_B.b_kstr_d <
           3; dynsim_2link_planar_B.b_kstr_d++) {
        dynsim_2link_planar_B.d_f = dynsim_2link_planar_B.b_kstr_d << 2;
        dynsim_2link_planar_B.c_f1_m[dynsim_2link_planar_B.d_f] =
          dynsim_2link_planar_B.R_ln[3 * dynsim_2link_planar_B.b_kstr_d];
        dynsim_2link_planar_B.c_f1_m[dynsim_2link_planar_B.d_f + 1] =
          dynsim_2link_planar_B.R_ln[3 * dynsim_2link_planar_B.b_kstr_d + 1];
        dynsim_2link_planar_B.c_f1_m[dynsim_2link_planar_B.d_f + 2] =
          dynsim_2link_planar_B.R_ln[3 * dynsim_2link_planar_B.b_kstr_d + 2];
      }

      dynsim_2link_planar_B.c_f1_m[15] = 1.0;
      break;

     default:
      rigidBodyJoint_get_JointAxis_aw(&body->JointInternal,
        dynsim_2link_planar_B.v_o);
      memset(&dynsim_2link_planar_B.tempR_h[0], 0, 9U * sizeof(real_T));
      dynsim_2link_planar_B.tempR_h[0] = 1.0;
      dynsim_2link_planar_B.tempR_h[4] = 1.0;
      dynsim_2link_planar_B.tempR_h[8] = 1.0;
      for (dynsim_2link_planar_B.b_kstr_d = 0; dynsim_2link_planar_B.b_kstr_d <
           3; dynsim_2link_planar_B.b_kstr_d++) {
        dynsim_2link_planar_B.d_f = dynsim_2link_planar_B.b_kstr_d << 2;
        dynsim_2link_planar_B.c_f1_m[dynsim_2link_planar_B.d_f] =
          dynsim_2link_planar_B.tempR_h[3 * dynsim_2link_planar_B.b_kstr_d];
        dynsim_2link_planar_B.c_f1_m[dynsim_2link_planar_B.d_f + 1] =
          dynsim_2link_planar_B.tempR_h[3 * dynsim_2link_planar_B.b_kstr_d + 1];
        dynsim_2link_planar_B.c_f1_m[dynsim_2link_planar_B.d_f + 2] =
          dynsim_2link_planar_B.tempR_h[3 * dynsim_2link_planar_B.b_kstr_d + 2];
        dynsim_2link_planar_B.c_f1_m[dynsim_2link_planar_B.b_kstr_d + 12] =
          dynsim_2link_planar_B.v_o[dynsim_2link_planar_B.b_kstr_d] *
          qvec[dynsim_2link_planar_B.e_a];
      }

      dynsim_2link_planar_B.c_f1_m[3] = 0.0;
      dynsim_2link_planar_B.c_f1_m[7] = 0.0;
      dynsim_2link_planar_B.c_f1_m[11] = 0.0;
      dynsim_2link_planar_B.c_f1_m[15] = 1.0;
      break;
    }

    for (dynsim_2link_planar_B.b_kstr_d = 0; dynsim_2link_planar_B.b_kstr_d < 16;
         dynsim_2link_planar_B.b_kstr_d++) {
      dynsim_2link_planar_B.a_n[dynsim_2link_planar_B.b_kstr_d] =
        body->
        JointInternal.JointToParentTransform[dynsim_2link_planar_B.b_kstr_d];
    }

    for (dynsim_2link_planar_B.b_kstr_d = 0; dynsim_2link_planar_B.b_kstr_d < 16;
         dynsim_2link_planar_B.b_kstr_d++) {
      dynsim_2link_planar_B.b_p[dynsim_2link_planar_B.b_kstr_d] =
        body->JointInternal.ChildToJointTransform[dynsim_2link_planar_B.b_kstr_d];
    }

    for (dynsim_2link_planar_B.b_kstr_d = 0; dynsim_2link_planar_B.b_kstr_d < 4;
         dynsim_2link_planar_B.b_kstr_d++) {
      for (dynsim_2link_planar_B.e_a = 0; dynsim_2link_planar_B.e_a < 4;
           dynsim_2link_planar_B.e_a++) {
        dynsim_2link_planar_B.d_f = dynsim_2link_planar_B.e_a << 2;
        dynsim_2link_planar_B.loop_ub_n = dynsim_2link_planar_B.b_kstr_d +
          dynsim_2link_planar_B.d_f;
        dynsim_2link_planar_B.a_l[dynsim_2link_planar_B.loop_ub_n] = 0.0;
        dynsim_2link_planar_B.a_l[dynsim_2link_planar_B.loop_ub_n] +=
          dynsim_2link_planar_B.c_f1_m[dynsim_2link_planar_B.d_f] *
          dynsim_2link_planar_B.a_n[dynsim_2link_planar_B.b_kstr_d];
        dynsim_2link_planar_B.a_l[dynsim_2link_planar_B.loop_ub_n] +=
          dynsim_2link_planar_B.c_f1_m[dynsim_2link_planar_B.d_f + 1] *
          dynsim_2link_planar_B.a_n[dynsim_2link_planar_B.b_kstr_d + 4];
        dynsim_2link_planar_B.a_l[dynsim_2link_planar_B.loop_ub_n] +=
          dynsim_2link_planar_B.c_f1_m[dynsim_2link_planar_B.d_f + 2] *
          dynsim_2link_planar_B.a_n[dynsim_2link_planar_B.b_kstr_d + 8];
        dynsim_2link_planar_B.a_l[dynsim_2link_planar_B.loop_ub_n] +=
          dynsim_2link_planar_B.c_f1_m[dynsim_2link_planar_B.d_f + 3] *
          dynsim_2link_planar_B.a_n[dynsim_2link_planar_B.b_kstr_d + 12];
      }

      for (dynsim_2link_planar_B.e_a = 0; dynsim_2link_planar_B.e_a < 4;
           dynsim_2link_planar_B.e_a++) {
        dynsim_2link_planar_B.d_f = dynsim_2link_planar_B.e_a << 2;
        dynsim_2link_planar_B.loop_ub_n = dynsim_2link_planar_B.b_kstr_d +
          dynsim_2link_planar_B.d_f;
        Ttree->data[dynsim_2link_planar_B.b_jtilecol_n]
          .f1[dynsim_2link_planar_B.loop_ub_n] = 0.0;
        Ttree->data[dynsim_2link_planar_B.b_jtilecol_n]
          .f1[dynsim_2link_planar_B.loop_ub_n] +=
          dynsim_2link_planar_B.b_p[dynsim_2link_planar_B.d_f] *
          dynsim_2link_planar_B.a_l[dynsim_2link_planar_B.b_kstr_d];
        Ttree->data[dynsim_2link_planar_B.b_jtilecol_n]
          .f1[dynsim_2link_planar_B.loop_ub_n] +=
          dynsim_2link_planar_B.b_p[dynsim_2link_planar_B.d_f + 1] *
          dynsim_2link_planar_B.a_l[dynsim_2link_planar_B.b_kstr_d + 4];
        Ttree->data[dynsim_2link_planar_B.b_jtilecol_n]
          .f1[dynsim_2link_planar_B.loop_ub_n] +=
          dynsim_2link_planar_B.b_p[dynsim_2link_planar_B.d_f + 2] *
          dynsim_2link_planar_B.a_l[dynsim_2link_planar_B.b_kstr_d + 8];
        Ttree->data[dynsim_2link_planar_B.b_jtilecol_n]
          .f1[dynsim_2link_planar_B.loop_ub_n] +=
          dynsim_2link_planar_B.b_p[dynsim_2link_planar_B.d_f + 3] *
          dynsim_2link_planar_B.a_l[dynsim_2link_planar_B.b_kstr_d + 12];
      }
    }

    dynsim_2link_planar_B.k_h = dynsim_2link_planar_B.n_l;
    if (body->ParentIndex > 0.0) {
      for (dynsim_2link_planar_B.b_kstr_d = 0; dynsim_2link_planar_B.b_kstr_d <
           16; dynsim_2link_planar_B.b_kstr_d++) {
        dynsim_2link_planar_B.a_n[dynsim_2link_planar_B.b_kstr_d] = Ttree->data[
          static_cast<int32_T>(body->ParentIndex) - 1]
          .f1[dynsim_2link_planar_B.b_kstr_d];
      }

      for (dynsim_2link_planar_B.b_kstr_d = 0; dynsim_2link_planar_B.b_kstr_d <
           4; dynsim_2link_planar_B.b_kstr_d++) {
        for (dynsim_2link_planar_B.e_a = 0; dynsim_2link_planar_B.e_a < 4;
             dynsim_2link_planar_B.e_a++) {
          dynsim_2link_planar_B.d_f = dynsim_2link_planar_B.e_a << 2;
          dynsim_2link_planar_B.loop_ub_n = dynsim_2link_planar_B.b_kstr_d +
            dynsim_2link_planar_B.d_f;
          dynsim_2link_planar_B.a_l[dynsim_2link_planar_B.loop_ub_n] = 0.0;
          dynsim_2link_planar_B.a_l[dynsim_2link_planar_B.loop_ub_n] +=
            Ttree->data[dynsim_2link_planar_B.b_jtilecol_n]
            .f1[dynsim_2link_planar_B.d_f] *
            dynsim_2link_planar_B.a_n[dynsim_2link_planar_B.b_kstr_d];
          dynsim_2link_planar_B.a_l[dynsim_2link_planar_B.loop_ub_n] +=
            Ttree->data[dynsim_2link_planar_B.b_jtilecol_n]
            .f1[dynsim_2link_planar_B.d_f + 1] *
            dynsim_2link_planar_B.a_n[dynsim_2link_planar_B.b_kstr_d + 4];
          dynsim_2link_planar_B.a_l[dynsim_2link_planar_B.loop_ub_n] +=
            Ttree->data[dynsim_2link_planar_B.b_jtilecol_n]
            .f1[dynsim_2link_planar_B.d_f + 2] *
            dynsim_2link_planar_B.a_n[dynsim_2link_planar_B.b_kstr_d + 8];
          dynsim_2link_planar_B.a_l[dynsim_2link_planar_B.loop_ub_n] +=
            Ttree->data[dynsim_2link_planar_B.b_jtilecol_n]
            .f1[dynsim_2link_planar_B.d_f + 3] *
            dynsim_2link_planar_B.a_n[dynsim_2link_planar_B.b_kstr_d + 12];
        }
      }

      for (dynsim_2link_planar_B.b_kstr_d = 0; dynsim_2link_planar_B.b_kstr_d <
           16; dynsim_2link_planar_B.b_kstr_d++) {
        Ttree->data[dynsim_2link_planar_B.b_jtilecol_n]
          .f1[dynsim_2link_planar_B.b_kstr_d] =
          dynsim_2link_planar_B.a_l[dynsim_2link_planar_B.b_kstr_d];
      }
    }
  }

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
}

static void dynsim_2link_pl_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Data[128], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_dynsim_2link_planar_std_msgs_MultiArrayDimension
  varargout_2_Layout_Dim[16], uint32_T *varargout_2_Layout_Dim_SL_Info_,
  uint32_T *varargout_2_Layout_Dim_SL_Inf_0)
{
  *varargout_1 = Sub_dynsim_2link_planar_160.getLatestMessage
    (&dynsim_2link_planar_B.b_varargout_2);
  memcpy(&varargout_2_Data[0], &dynsim_2link_planar_B.b_varargout_2.Data[0],
         sizeof(real_T) << 7U);
  *varargout_2_Data_SL_Info_Curren =
    dynsim_2link_planar_B.b_varargout_2.Data_SL_Info.CurrentLength;
  *varargout_2_Data_SL_Info_Receiv =
    dynsim_2link_planar_B.b_varargout_2.Data_SL_Info.ReceivedLength;
  *varargout_2_Layout_DataOffset =
    dynsim_2link_planar_B.b_varargout_2.Layout.DataOffset;
  memcpy(&varargout_2_Layout_Dim[0],
         &dynsim_2link_planar_B.b_varargout_2.Layout.Dim[0], sizeof
         (SL_Bus_dynsim_2link_planar_std_msgs_MultiArrayDimension) << 4U);
  *varargout_2_Layout_Dim_SL_Info_ =
    dynsim_2link_planar_B.b_varargout_2.Layout.Dim_SL_Info.CurrentLength;
  *varargout_2_Layout_Dim_SL_Inf_0 =
    dynsim_2link_planar_B.b_varargout_2.Layout.Dim_SL_Info.ReceivedLength;
}

static void dynsim_2li_emxInit_f_cell_wrap1(emxArray_f_cell_wrap_dynsim_a_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_f_cell_wrap_dynsim_a_T *emxArray;
  *pEmxArray = (emxArray_f_cell_wrap_dynsim_a_T *)malloc(sizeof
    (emxArray_f_cell_wrap_dynsim_a_T));
  emxArray = *pEmxArray;
  emxArray->data = (f_cell_wrap_dynsim_2link_plan_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (dynsim_2link_planar_B.i_i3 = 0; dynsim_2link_planar_B.i_i3 <
       numDimensions; dynsim_2link_planar_B.i_i3++) {
    emxArray->size[dynsim_2link_planar_B.i_i3] = 0;
  }
}

static void d_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_dynsim_a_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  dynsim_2link_planar_B.newNumel_l = 1;
  for (dynsim_2link_planar_B.i_f = 0; dynsim_2link_planar_B.i_f <
       emxArray->numDimensions; dynsim_2link_planar_B.i_f++) {
    dynsim_2link_planar_B.newNumel_l *= emxArray->size[dynsim_2link_planar_B.i_f];
  }

  if (dynsim_2link_planar_B.newNumel_l > emxArray->allocatedSize) {
    dynsim_2link_planar_B.i_f = emxArray->allocatedSize;
    if (dynsim_2link_planar_B.i_f < 16) {
      dynsim_2link_planar_B.i_f = 16;
    }

    while (dynsim_2link_planar_B.i_f < dynsim_2link_planar_B.newNumel_l) {
      if (dynsim_2link_planar_B.i_f > 1073741823) {
        dynsim_2link_planar_B.i_f = MAX_int32_T;
      } else {
        dynsim_2link_planar_B.i_f <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(dynsim_2link_planar_B.i_f), sizeof
                     (f_cell_wrap_dynsim_2link_plan_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(f_cell_wrap_dynsim_2link_plan_T)
             * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (f_cell_wrap_dynsim_2link_plan_T *)newData;
    emxArray->allocatedSize = dynsim_2link_planar_B.i_f;
    emxArray->canFreeData = true;
  }
}

static void dy_rigidBodyJoint_get_JointAxis(const
  c_rigidBodyJoint_dynsim_2link_T *obj, real_T ax[3])
{
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  boolean_T guard1 = false;
  int32_T exitg1;
  for (dynsim_2link_planar_B.b_kstr_o = 0; dynsim_2link_planar_B.b_kstr_o < 8;
       dynsim_2link_planar_B.b_kstr_o++) {
    dynsim_2link_planar_B.b_a[dynsim_2link_planar_B.b_kstr_o] =
      tmp[dynsim_2link_planar_B.b_kstr_o];
  }

  dynsim_2link_planar_B.b_bool_n = false;
  if (obj->Type->size[1] == 8) {
    dynsim_2link_planar_B.b_kstr_o = 1;
    do {
      exitg1 = 0;
      if (dynsim_2link_planar_B.b_kstr_o - 1 < 8) {
        dynsim_2link_planar_B.kstr_i = dynsim_2link_planar_B.b_kstr_o - 1;
        if (obj->Type->data[dynsim_2link_planar_B.kstr_i] !=
            dynsim_2link_planar_B.b_a[dynsim_2link_planar_B.kstr_i]) {
          exitg1 = 1;
        } else {
          dynsim_2link_planar_B.b_kstr_o++;
        }
      } else {
        dynsim_2link_planar_B.b_bool_n = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (dynsim_2link_planar_B.b_bool_n) {
    guard1 = true;
  } else {
    for (dynsim_2link_planar_B.b_kstr_o = 0; dynsim_2link_planar_B.b_kstr_o < 9;
         dynsim_2link_planar_B.b_kstr_o++) {
      dynsim_2link_planar_B.b_h[dynsim_2link_planar_B.b_kstr_o] =
        tmp_0[dynsim_2link_planar_B.b_kstr_o];
    }

    dynsim_2link_planar_B.b_bool_n = false;
    if (obj->Type->size[1] == 9) {
      dynsim_2link_planar_B.b_kstr_o = 1;
      do {
        exitg1 = 0;
        if (dynsim_2link_planar_B.b_kstr_o - 1 < 9) {
          dynsim_2link_planar_B.kstr_i = dynsim_2link_planar_B.b_kstr_o - 1;
          if (obj->Type->data[dynsim_2link_planar_B.kstr_i] !=
              dynsim_2link_planar_B.b_h[dynsim_2link_planar_B.kstr_i]) {
            exitg1 = 1;
          } else {
            dynsim_2link_planar_B.b_kstr_o++;
          }
        } else {
          dynsim_2link_planar_B.b_bool_n = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynsim_2link_planar_B.b_bool_n) {
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

static void dynsim_2link_planar_cat(real_T varargin_1, real_T varargin_2, real_T
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

static void rigidBodyJoint_transformBodyT_a(const
  c_rigidBodyJoint_dynsim_2link_T *obj, const real_T q_data[], const int32_T
  *q_size, real_T T[16])
{
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  for (dynsim_2link_planar_B.b_kstr_k = 0; dynsim_2link_planar_B.b_kstr_k < 5;
       dynsim_2link_planar_B.b_kstr_k++) {
    dynsim_2link_planar_B.b_ex[dynsim_2link_planar_B.b_kstr_k] =
      tmp[dynsim_2link_planar_B.b_kstr_k];
  }

  dynsim_2link_planar_B.b_bool_g = false;
  if (obj->Type->size[1] == 5) {
    dynsim_2link_planar_B.b_kstr_k = 1;
    do {
      exitg1 = 0;
      if (dynsim_2link_planar_B.b_kstr_k - 1 < 5) {
        dynsim_2link_planar_B.kstr_o = dynsim_2link_planar_B.b_kstr_k - 1;
        if (obj->Type->data[dynsim_2link_planar_B.kstr_o] !=
            dynsim_2link_planar_B.b_ex[dynsim_2link_planar_B.kstr_o]) {
          exitg1 = 1;
        } else {
          dynsim_2link_planar_B.b_kstr_k++;
        }
      } else {
        dynsim_2link_planar_B.b_bool_g = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (dynsim_2link_planar_B.b_bool_g) {
    dynsim_2link_planar_B.b_kstr_k = 0;
  } else {
    for (dynsim_2link_planar_B.b_kstr_k = 0; dynsim_2link_planar_B.b_kstr_k < 8;
         dynsim_2link_planar_B.b_kstr_k++) {
      dynsim_2link_planar_B.b_p5[dynsim_2link_planar_B.b_kstr_k] =
        tmp_0[dynsim_2link_planar_B.b_kstr_k];
    }

    dynsim_2link_planar_B.b_bool_g = false;
    if (obj->Type->size[1] == 8) {
      dynsim_2link_planar_B.b_kstr_k = 1;
      do {
        exitg1 = 0;
        if (dynsim_2link_planar_B.b_kstr_k - 1 < 8) {
          dynsim_2link_planar_B.kstr_o = dynsim_2link_planar_B.b_kstr_k - 1;
          if (obj->Type->data[dynsim_2link_planar_B.kstr_o] !=
              dynsim_2link_planar_B.b_p5[dynsim_2link_planar_B.kstr_o]) {
            exitg1 = 1;
          } else {
            dynsim_2link_planar_B.b_kstr_k++;
          }
        } else {
          dynsim_2link_planar_B.b_bool_g = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynsim_2link_planar_B.b_bool_g) {
      dynsim_2link_planar_B.b_kstr_k = 1;
    } else {
      dynsim_2link_planar_B.b_kstr_k = -1;
    }
  }

  switch (dynsim_2link_planar_B.b_kstr_k) {
   case 0:
    memset(&dynsim_2link_planar_B.TJ[0], 0, sizeof(real_T) << 4U);
    dynsim_2link_planar_B.TJ[0] = 1.0;
    dynsim_2link_planar_B.TJ[5] = 1.0;
    dynsim_2link_planar_B.TJ[10] = 1.0;
    dynsim_2link_planar_B.TJ[15] = 1.0;
    break;

   case 1:
    dy_rigidBodyJoint_get_JointAxis(obj, dynsim_2link_planar_B.v_n);
    dynsim_2link_planar_B.result_data_f[0] = dynsim_2link_planar_B.v_n[0];
    dynsim_2link_planar_B.result_data_f[1] = dynsim_2link_planar_B.v_n[1];
    dynsim_2link_planar_B.result_data_f[2] = dynsim_2link_planar_B.v_n[2];
    if (0 <= (*q_size != 0) - 1) {
      dynsim_2link_planar_B.result_data_f[3] = q_data[0];
    }

    dynsim_2link_planar_B.cth = 1.0 / sqrt((dynsim_2link_planar_B.result_data_f
      [0] * dynsim_2link_planar_B.result_data_f[0] +
      dynsim_2link_planar_B.result_data_f[1] *
      dynsim_2link_planar_B.result_data_f[1]) +
      dynsim_2link_planar_B.result_data_f[2] *
      dynsim_2link_planar_B.result_data_f[2]);
    dynsim_2link_planar_B.v_n[0] = dynsim_2link_planar_B.result_data_f[0] *
      dynsim_2link_planar_B.cth;
    dynsim_2link_planar_B.v_n[1] = dynsim_2link_planar_B.result_data_f[1] *
      dynsim_2link_planar_B.cth;
    dynsim_2link_planar_B.v_n[2] = dynsim_2link_planar_B.result_data_f[2] *
      dynsim_2link_planar_B.cth;
    dynsim_2link_planar_B.cth = cos(dynsim_2link_planar_B.result_data_f[3]);
    dynsim_2link_planar_B.sth_i = sin(dynsim_2link_planar_B.result_data_f[3]);
    dynsim_2link_planar_B.tempR_tmp_f = dynsim_2link_planar_B.v_n[1] *
      dynsim_2link_planar_B.v_n[0] * (1.0 - dynsim_2link_planar_B.cth);
    dynsim_2link_planar_B.tempR_tmp_i = dynsim_2link_planar_B.v_n[2] *
      dynsim_2link_planar_B.sth_i;
    dynsim_2link_planar_B.tempR_tmp_ff = dynsim_2link_planar_B.v_n[2] *
      dynsim_2link_planar_B.v_n[0] * (1.0 - dynsim_2link_planar_B.cth);
    dynsim_2link_planar_B.tempR_tmp_g = dynsim_2link_planar_B.v_n[1] *
      dynsim_2link_planar_B.sth_i;
    dynsim_2link_planar_B.tempR_tmp_c = dynsim_2link_planar_B.v_n[2] *
      dynsim_2link_planar_B.v_n[1] * (1.0 - dynsim_2link_planar_B.cth);
    dynsim_2link_planar_B.sth_i *= dynsim_2link_planar_B.v_n[0];
    dynsim_2link_planar_cat(dynsim_2link_planar_B.v_n[0] *
      dynsim_2link_planar_B.v_n[0] * (1.0 - dynsim_2link_planar_B.cth) +
      dynsim_2link_planar_B.cth, dynsim_2link_planar_B.tempR_tmp_f -
      dynsim_2link_planar_B.tempR_tmp_i, dynsim_2link_planar_B.tempR_tmp_ff +
      dynsim_2link_planar_B.tempR_tmp_g, dynsim_2link_planar_B.tempR_tmp_f +
      dynsim_2link_planar_B.tempR_tmp_i, dynsim_2link_planar_B.v_n[1] *
      dynsim_2link_planar_B.v_n[1] * (1.0 - dynsim_2link_planar_B.cth) +
      dynsim_2link_planar_B.cth, dynsim_2link_planar_B.tempR_tmp_c -
      dynsim_2link_planar_B.sth_i, dynsim_2link_planar_B.tempR_tmp_ff -
      dynsim_2link_planar_B.tempR_tmp_g, dynsim_2link_planar_B.tempR_tmp_c +
      dynsim_2link_planar_B.sth_i, dynsim_2link_planar_B.v_n[2] *
      dynsim_2link_planar_B.v_n[2] * (1.0 - dynsim_2link_planar_B.cth) +
      dynsim_2link_planar_B.cth, dynsim_2link_planar_B.tempR_b);
    for (dynsim_2link_planar_B.b_kstr_k = 0; dynsim_2link_planar_B.b_kstr_k < 3;
         dynsim_2link_planar_B.b_kstr_k++) {
      dynsim_2link_planar_B.kstr_o = dynsim_2link_planar_B.b_kstr_k + 1;
      dynsim_2link_planar_B.R_o[dynsim_2link_planar_B.kstr_o - 1] =
        dynsim_2link_planar_B.tempR_b[(dynsim_2link_planar_B.kstr_o - 1) * 3];
      dynsim_2link_planar_B.kstr_o = dynsim_2link_planar_B.b_kstr_k + 1;
      dynsim_2link_planar_B.R_o[dynsim_2link_planar_B.kstr_o + 2] =
        dynsim_2link_planar_B.tempR_b[(dynsim_2link_planar_B.kstr_o - 1) * 3 + 1];
      dynsim_2link_planar_B.kstr_o = dynsim_2link_planar_B.b_kstr_k + 1;
      dynsim_2link_planar_B.R_o[dynsim_2link_planar_B.kstr_o + 5] =
        dynsim_2link_planar_B.tempR_b[(dynsim_2link_planar_B.kstr_o - 1) * 3 + 2];
    }

    memset(&dynsim_2link_planar_B.TJ[0], 0, sizeof(real_T) << 4U);
    for (dynsim_2link_planar_B.b_kstr_k = 0; dynsim_2link_planar_B.b_kstr_k < 3;
         dynsim_2link_planar_B.b_kstr_k++) {
      dynsim_2link_planar_B.kstr_o = dynsim_2link_planar_B.b_kstr_k << 2;
      dynsim_2link_planar_B.TJ[dynsim_2link_planar_B.kstr_o] =
        dynsim_2link_planar_B.R_o[3 * dynsim_2link_planar_B.b_kstr_k];
      dynsim_2link_planar_B.TJ[dynsim_2link_planar_B.kstr_o + 1] =
        dynsim_2link_planar_B.R_o[3 * dynsim_2link_planar_B.b_kstr_k + 1];
      dynsim_2link_planar_B.TJ[dynsim_2link_planar_B.kstr_o + 2] =
        dynsim_2link_planar_B.R_o[3 * dynsim_2link_planar_B.b_kstr_k + 2];
    }

    dynsim_2link_planar_B.TJ[15] = 1.0;
    break;

   default:
    dy_rigidBodyJoint_get_JointAxis(obj, dynsim_2link_planar_B.v_n);
    memset(&dynsim_2link_planar_B.tempR_b[0], 0, 9U * sizeof(real_T));
    dynsim_2link_planar_B.tempR_b[0] = 1.0;
    dynsim_2link_planar_B.tempR_b[4] = 1.0;
    dynsim_2link_planar_B.tempR_b[8] = 1.0;
    for (dynsim_2link_planar_B.b_kstr_k = 0; dynsim_2link_planar_B.b_kstr_k < 3;
         dynsim_2link_planar_B.b_kstr_k++) {
      dynsim_2link_planar_B.kstr_o = dynsim_2link_planar_B.b_kstr_k << 2;
      dynsim_2link_planar_B.TJ[dynsim_2link_planar_B.kstr_o] =
        dynsim_2link_planar_B.tempR_b[3 * dynsim_2link_planar_B.b_kstr_k];
      dynsim_2link_planar_B.TJ[dynsim_2link_planar_B.kstr_o + 1] =
        dynsim_2link_planar_B.tempR_b[3 * dynsim_2link_planar_B.b_kstr_k + 1];
      dynsim_2link_planar_B.TJ[dynsim_2link_planar_B.kstr_o + 2] =
        dynsim_2link_planar_B.tempR_b[3 * dynsim_2link_planar_B.b_kstr_k + 2];
      dynsim_2link_planar_B.TJ[dynsim_2link_planar_B.b_kstr_k + 12] =
        dynsim_2link_planar_B.v_n[dynsim_2link_planar_B.b_kstr_k] * q_data[0];
    }

    dynsim_2link_planar_B.TJ[3] = 0.0;
    dynsim_2link_planar_B.TJ[7] = 0.0;
    dynsim_2link_planar_B.TJ[11] = 0.0;
    dynsim_2link_planar_B.TJ[15] = 1.0;
    break;
  }

  for (dynsim_2link_planar_B.b_kstr_k = 0; dynsim_2link_planar_B.b_kstr_k < 4;
       dynsim_2link_planar_B.b_kstr_k++) {
    for (dynsim_2link_planar_B.kstr_o = 0; dynsim_2link_planar_B.kstr_o < 4;
         dynsim_2link_planar_B.kstr_o++) {
      dynsim_2link_planar_B.obj_tmp_tmp = dynsim_2link_planar_B.kstr_o << 2;
      dynsim_2link_planar_B.obj_tmp = dynsim_2link_planar_B.b_kstr_k +
        dynsim_2link_planar_B.obj_tmp_tmp;
      dynsim_2link_planar_B.obj[dynsim_2link_planar_B.obj_tmp] = 0.0;
      dynsim_2link_planar_B.obj[dynsim_2link_planar_B.obj_tmp] +=
        dynsim_2link_planar_B.TJ[dynsim_2link_planar_B.obj_tmp_tmp] *
        obj->JointToParentTransform[dynsim_2link_planar_B.b_kstr_k];
      dynsim_2link_planar_B.obj[dynsim_2link_planar_B.obj_tmp] +=
        dynsim_2link_planar_B.TJ[dynsim_2link_planar_B.obj_tmp_tmp + 1] *
        obj->JointToParentTransform[dynsim_2link_planar_B.b_kstr_k + 4];
      dynsim_2link_planar_B.obj[dynsim_2link_planar_B.obj_tmp] +=
        dynsim_2link_planar_B.TJ[dynsim_2link_planar_B.obj_tmp_tmp + 2] *
        obj->JointToParentTransform[dynsim_2link_planar_B.b_kstr_k + 8];
      dynsim_2link_planar_B.obj[dynsim_2link_planar_B.obj_tmp] +=
        dynsim_2link_planar_B.TJ[dynsim_2link_planar_B.obj_tmp_tmp + 3] *
        obj->JointToParentTransform[dynsim_2link_planar_B.b_kstr_k + 12];
    }

    for (dynsim_2link_planar_B.kstr_o = 0; dynsim_2link_planar_B.kstr_o < 4;
         dynsim_2link_planar_B.kstr_o++) {
      dynsim_2link_planar_B.obj_tmp_tmp = dynsim_2link_planar_B.kstr_o << 2;
      dynsim_2link_planar_B.obj_tmp = dynsim_2link_planar_B.b_kstr_k +
        dynsim_2link_planar_B.obj_tmp_tmp;
      T[dynsim_2link_planar_B.obj_tmp] = 0.0;
      T[dynsim_2link_planar_B.obj_tmp] += obj->
        ChildToJointTransform[dynsim_2link_planar_B.obj_tmp_tmp] *
        dynsim_2link_planar_B.obj[dynsim_2link_planar_B.b_kstr_k];
      T[dynsim_2link_planar_B.obj_tmp] += obj->
        ChildToJointTransform[dynsim_2link_planar_B.obj_tmp_tmp + 1] *
        dynsim_2link_planar_B.obj[dynsim_2link_planar_B.b_kstr_k + 4];
      T[dynsim_2link_planar_B.obj_tmp] += obj->
        ChildToJointTransform[dynsim_2link_planar_B.obj_tmp_tmp + 2] *
        dynsim_2link_planar_B.obj[dynsim_2link_planar_B.b_kstr_k + 8];
      T[dynsim_2link_planar_B.obj_tmp] += obj->
        ChildToJointTransform[dynsim_2link_planar_B.obj_tmp_tmp + 3] *
        dynsim_2link_planar_B.obj[dynsim_2link_planar_B.b_kstr_k + 12];
    }
  }
}

static void rigidBodyJoint_transformBodyToP(const
  c_rigidBodyJoint_dynsim_2link_T *obj, real_T T[16])
{
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  for (dynsim_2link_planar_B.b_kstr_c = 0; dynsim_2link_planar_B.b_kstr_c < 5;
       dynsim_2link_planar_B.b_kstr_c++) {
    dynsim_2link_planar_B.b_h2[dynsim_2link_planar_B.b_kstr_c] =
      tmp[dynsim_2link_planar_B.b_kstr_c];
  }

  dynsim_2link_planar_B.b_bool_c = false;
  if (obj->Type->size[1] == 5) {
    dynsim_2link_planar_B.b_kstr_c = 1;
    do {
      exitg1 = 0;
      if (dynsim_2link_planar_B.b_kstr_c - 1 < 5) {
        dynsim_2link_planar_B.kstr_b = dynsim_2link_planar_B.b_kstr_c - 1;
        if (obj->Type->data[dynsim_2link_planar_B.kstr_b] !=
            dynsim_2link_planar_B.b_h2[dynsim_2link_planar_B.kstr_b]) {
          exitg1 = 1;
        } else {
          dynsim_2link_planar_B.b_kstr_c++;
        }
      } else {
        dynsim_2link_planar_B.b_bool_c = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (dynsim_2link_planar_B.b_bool_c) {
    dynsim_2link_planar_B.b_kstr_c = 0;
  } else {
    for (dynsim_2link_planar_B.b_kstr_c = 0; dynsim_2link_planar_B.b_kstr_c < 8;
         dynsim_2link_planar_B.b_kstr_c++) {
      dynsim_2link_planar_B.b_e[dynsim_2link_planar_B.b_kstr_c] =
        tmp_0[dynsim_2link_planar_B.b_kstr_c];
    }

    dynsim_2link_planar_B.b_bool_c = false;
    if (obj->Type->size[1] == 8) {
      dynsim_2link_planar_B.b_kstr_c = 1;
      do {
        exitg1 = 0;
        if (dynsim_2link_planar_B.b_kstr_c - 1 < 8) {
          dynsim_2link_planar_B.kstr_b = dynsim_2link_planar_B.b_kstr_c - 1;
          if (obj->Type->data[dynsim_2link_planar_B.kstr_b] !=
              dynsim_2link_planar_B.b_e[dynsim_2link_planar_B.kstr_b]) {
            exitg1 = 1;
          } else {
            dynsim_2link_planar_B.b_kstr_c++;
          }
        } else {
          dynsim_2link_planar_B.b_bool_c = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (dynsim_2link_planar_B.b_bool_c) {
      dynsim_2link_planar_B.b_kstr_c = 1;
    } else {
      dynsim_2link_planar_B.b_kstr_c = -1;
    }
  }

  switch (dynsim_2link_planar_B.b_kstr_c) {
   case 0:
    memset(&dynsim_2link_planar_B.TJ_g[0], 0, sizeof(real_T) << 4U);
    dynsim_2link_planar_B.TJ_g[0] = 1.0;
    dynsim_2link_planar_B.TJ_g[5] = 1.0;
    dynsim_2link_planar_B.TJ_g[10] = 1.0;
    dynsim_2link_planar_B.TJ_g[15] = 1.0;
    break;

   case 1:
    dy_rigidBodyJoint_get_JointAxis(obj, dynsim_2link_planar_B.v_i);
    dynsim_2link_planar_B.axang_idx_0 = dynsim_2link_planar_B.v_i[0];
    dynsim_2link_planar_B.axang_idx_1 = dynsim_2link_planar_B.v_i[1];
    dynsim_2link_planar_B.axang_idx_2 = dynsim_2link_planar_B.v_i[2];
    dynsim_2link_planar_B.b_f = 1.0 / sqrt((dynsim_2link_planar_B.axang_idx_0 *
      dynsim_2link_planar_B.axang_idx_0 + dynsim_2link_planar_B.axang_idx_1 *
      dynsim_2link_planar_B.axang_idx_1) + dynsim_2link_planar_B.axang_idx_2 *
      dynsim_2link_planar_B.axang_idx_2);
    dynsim_2link_planar_B.v_i[0] = dynsim_2link_planar_B.axang_idx_0 *
      dynsim_2link_planar_B.b_f;
    dynsim_2link_planar_B.v_i[1] = dynsim_2link_planar_B.axang_idx_1 *
      dynsim_2link_planar_B.b_f;
    dynsim_2link_planar_B.v_i[2] = dynsim_2link_planar_B.axang_idx_2 *
      dynsim_2link_planar_B.b_f;
    dynsim_2link_planar_B.axang_idx_0 = dynsim_2link_planar_B.v_i[1] *
      dynsim_2link_planar_B.v_i[0] * 0.0;
    dynsim_2link_planar_B.axang_idx_1 = dynsim_2link_planar_B.v_i[2] *
      dynsim_2link_planar_B.v_i[0] * 0.0;
    dynsim_2link_planar_B.axang_idx_2 = dynsim_2link_planar_B.v_i[2] *
      dynsim_2link_planar_B.v_i[1] * 0.0;
    dynsim_2link_planar_cat(dynsim_2link_planar_B.v_i[0] *
      dynsim_2link_planar_B.v_i[0] * 0.0 + 1.0,
      dynsim_2link_planar_B.axang_idx_0 - dynsim_2link_planar_B.v_i[2] * 0.0,
      dynsim_2link_planar_B.axang_idx_1 + dynsim_2link_planar_B.v_i[1] * 0.0,
      dynsim_2link_planar_B.axang_idx_0 + dynsim_2link_planar_B.v_i[2] * 0.0,
      dynsim_2link_planar_B.v_i[1] * dynsim_2link_planar_B.v_i[1] * 0.0 + 1.0,
      dynsim_2link_planar_B.axang_idx_2 - dynsim_2link_planar_B.v_i[0] * 0.0,
      dynsim_2link_planar_B.axang_idx_1 - dynsim_2link_planar_B.v_i[1] * 0.0,
      dynsim_2link_planar_B.axang_idx_2 + dynsim_2link_planar_B.v_i[0] * 0.0,
      dynsim_2link_planar_B.v_i[2] * dynsim_2link_planar_B.v_i[2] * 0.0 + 1.0,
      dynsim_2link_planar_B.tempR_bs);
    for (dynsim_2link_planar_B.b_kstr_c = 0; dynsim_2link_planar_B.b_kstr_c < 3;
         dynsim_2link_planar_B.b_kstr_c++) {
      dynsim_2link_planar_B.kstr_b = dynsim_2link_planar_B.b_kstr_c + 1;
      dynsim_2link_planar_B.R_n[dynsim_2link_planar_B.kstr_b - 1] =
        dynsim_2link_planar_B.tempR_bs[(dynsim_2link_planar_B.kstr_b - 1) * 3];
      dynsim_2link_planar_B.kstr_b = dynsim_2link_planar_B.b_kstr_c + 1;
      dynsim_2link_planar_B.R_n[dynsim_2link_planar_B.kstr_b + 2] =
        dynsim_2link_planar_B.tempR_bs[(dynsim_2link_planar_B.kstr_b - 1) * 3 +
        1];
      dynsim_2link_planar_B.kstr_b = dynsim_2link_planar_B.b_kstr_c + 1;
      dynsim_2link_planar_B.R_n[dynsim_2link_planar_B.kstr_b + 5] =
        dynsim_2link_planar_B.tempR_bs[(dynsim_2link_planar_B.kstr_b - 1) * 3 +
        2];
    }

    memset(&dynsim_2link_planar_B.TJ_g[0], 0, sizeof(real_T) << 4U);
    for (dynsim_2link_planar_B.b_kstr_c = 0; dynsim_2link_planar_B.b_kstr_c < 3;
         dynsim_2link_planar_B.b_kstr_c++) {
      dynsim_2link_planar_B.kstr_b = dynsim_2link_planar_B.b_kstr_c << 2;
      dynsim_2link_planar_B.TJ_g[dynsim_2link_planar_B.kstr_b] =
        dynsim_2link_planar_B.R_n[3 * dynsim_2link_planar_B.b_kstr_c];
      dynsim_2link_planar_B.TJ_g[dynsim_2link_planar_B.kstr_b + 1] =
        dynsim_2link_planar_B.R_n[3 * dynsim_2link_planar_B.b_kstr_c + 1];
      dynsim_2link_planar_B.TJ_g[dynsim_2link_planar_B.kstr_b + 2] =
        dynsim_2link_planar_B.R_n[3 * dynsim_2link_planar_B.b_kstr_c + 2];
    }

    dynsim_2link_planar_B.TJ_g[15] = 1.0;
    break;

   default:
    dy_rigidBodyJoint_get_JointAxis(obj, dynsim_2link_planar_B.v_i);
    memset(&dynsim_2link_planar_B.tempR_bs[0], 0, 9U * sizeof(real_T));
    dynsim_2link_planar_B.tempR_bs[0] = 1.0;
    dynsim_2link_planar_B.tempR_bs[4] = 1.0;
    dynsim_2link_planar_B.tempR_bs[8] = 1.0;
    for (dynsim_2link_planar_B.b_kstr_c = 0; dynsim_2link_planar_B.b_kstr_c < 3;
         dynsim_2link_planar_B.b_kstr_c++) {
      dynsim_2link_planar_B.kstr_b = dynsim_2link_planar_B.b_kstr_c << 2;
      dynsim_2link_planar_B.TJ_g[dynsim_2link_planar_B.kstr_b] =
        dynsim_2link_planar_B.tempR_bs[3 * dynsim_2link_planar_B.b_kstr_c];
      dynsim_2link_planar_B.TJ_g[dynsim_2link_planar_B.kstr_b + 1] =
        dynsim_2link_planar_B.tempR_bs[3 * dynsim_2link_planar_B.b_kstr_c + 1];
      dynsim_2link_planar_B.TJ_g[dynsim_2link_planar_B.kstr_b + 2] =
        dynsim_2link_planar_B.tempR_bs[3 * dynsim_2link_planar_B.b_kstr_c + 2];
      dynsim_2link_planar_B.TJ_g[dynsim_2link_planar_B.b_kstr_c + 12] =
        dynsim_2link_planar_B.v_i[dynsim_2link_planar_B.b_kstr_c] * 0.0;
    }

    dynsim_2link_planar_B.TJ_g[3] = 0.0;
    dynsim_2link_planar_B.TJ_g[7] = 0.0;
    dynsim_2link_planar_B.TJ_g[11] = 0.0;
    dynsim_2link_planar_B.TJ_g[15] = 1.0;
    break;
  }

  for (dynsim_2link_planar_B.b_kstr_c = 0; dynsim_2link_planar_B.b_kstr_c < 4;
       dynsim_2link_planar_B.b_kstr_c++) {
    for (dynsim_2link_planar_B.kstr_b = 0; dynsim_2link_planar_B.kstr_b < 4;
         dynsim_2link_planar_B.kstr_b++) {
      dynsim_2link_planar_B.obj_tmp_tmp_i = dynsim_2link_planar_B.kstr_b << 2;
      dynsim_2link_planar_B.obj_tmp_n = dynsim_2link_planar_B.b_kstr_c +
        dynsim_2link_planar_B.obj_tmp_tmp_i;
      dynsim_2link_planar_B.obj_g[dynsim_2link_planar_B.obj_tmp_n] = 0.0;
      dynsim_2link_planar_B.obj_g[dynsim_2link_planar_B.obj_tmp_n] +=
        dynsim_2link_planar_B.TJ_g[dynsim_2link_planar_B.obj_tmp_tmp_i] *
        obj->JointToParentTransform[dynsim_2link_planar_B.b_kstr_c];
      dynsim_2link_planar_B.obj_g[dynsim_2link_planar_B.obj_tmp_n] +=
        dynsim_2link_planar_B.TJ_g[dynsim_2link_planar_B.obj_tmp_tmp_i + 1] *
        obj->JointToParentTransform[dynsim_2link_planar_B.b_kstr_c + 4];
      dynsim_2link_planar_B.obj_g[dynsim_2link_planar_B.obj_tmp_n] +=
        dynsim_2link_planar_B.TJ_g[dynsim_2link_planar_B.obj_tmp_tmp_i + 2] *
        obj->JointToParentTransform[dynsim_2link_planar_B.b_kstr_c + 8];
      dynsim_2link_planar_B.obj_g[dynsim_2link_planar_B.obj_tmp_n] +=
        dynsim_2link_planar_B.TJ_g[dynsim_2link_planar_B.obj_tmp_tmp_i + 3] *
        obj->JointToParentTransform[dynsim_2link_planar_B.b_kstr_c + 12];
    }

    for (dynsim_2link_planar_B.kstr_b = 0; dynsim_2link_planar_B.kstr_b < 4;
         dynsim_2link_planar_B.kstr_b++) {
      dynsim_2link_planar_B.obj_tmp_tmp_i = dynsim_2link_planar_B.kstr_b << 2;
      dynsim_2link_planar_B.obj_tmp_n = dynsim_2link_planar_B.b_kstr_c +
        dynsim_2link_planar_B.obj_tmp_tmp_i;
      T[dynsim_2link_planar_B.obj_tmp_n] = 0.0;
      T[dynsim_2link_planar_B.obj_tmp_n] += obj->
        ChildToJointTransform[dynsim_2link_planar_B.obj_tmp_tmp_i] *
        dynsim_2link_planar_B.obj_g[dynsim_2link_planar_B.b_kstr_c];
      T[dynsim_2link_planar_B.obj_tmp_n] += obj->
        ChildToJointTransform[dynsim_2link_planar_B.obj_tmp_tmp_i + 1] *
        dynsim_2link_planar_B.obj_g[dynsim_2link_planar_B.b_kstr_c + 4];
      T[dynsim_2link_planar_B.obj_tmp_n] += obj->
        ChildToJointTransform[dynsim_2link_planar_B.obj_tmp_tmp_i + 2] *
        dynsim_2link_planar_B.obj_g[dynsim_2link_planar_B.b_kstr_c + 8];
      T[dynsim_2link_planar_B.obj_tmp_n] += obj->
        ChildToJointTransform[dynsim_2link_planar_B.obj_tmp_tmp_i + 3] *
        dynsim_2link_planar_B.obj_g[dynsim_2link_planar_B.b_kstr_c + 12];
    }
  }
}

static void dynsim_2link_planar_tforminv(const real_T T[16], real_T Tinv[16])
{
  for (dynsim_2link_planar_B.i3 = 0; dynsim_2link_planar_B.i3 < 3;
       dynsim_2link_planar_B.i3++) {
    dynsim_2link_planar_B.R_b[3 * dynsim_2link_planar_B.i3] =
      T[dynsim_2link_planar_B.i3];
    dynsim_2link_planar_B.R_b[3 * dynsim_2link_planar_B.i3 + 1] =
      T[dynsim_2link_planar_B.i3 + 4];
    dynsim_2link_planar_B.R_b[3 * dynsim_2link_planar_B.i3 + 2] =
      T[dynsim_2link_planar_B.i3 + 8];
  }

  for (dynsim_2link_planar_B.i3 = 0; dynsim_2link_planar_B.i3 < 9;
       dynsim_2link_planar_B.i3++) {
    dynsim_2link_planar_B.R_da[dynsim_2link_planar_B.i3] =
      -dynsim_2link_planar_B.R_b[dynsim_2link_planar_B.i3];
  }

  for (dynsim_2link_planar_B.i3 = 0; dynsim_2link_planar_B.i3 < 3;
       dynsim_2link_planar_B.i3++) {
    dynsim_2link_planar_B.Tinv_tmp = dynsim_2link_planar_B.i3 << 2;
    Tinv[dynsim_2link_planar_B.Tinv_tmp] = dynsim_2link_planar_B.R_b[3 *
      dynsim_2link_planar_B.i3];
    Tinv[dynsim_2link_planar_B.Tinv_tmp + 1] = dynsim_2link_planar_B.R_b[3 *
      dynsim_2link_planar_B.i3 + 1];
    Tinv[dynsim_2link_planar_B.Tinv_tmp + 2] = dynsim_2link_planar_B.R_b[3 *
      dynsim_2link_planar_B.i3 + 2];
    Tinv[dynsim_2link_planar_B.i3 + 12] =
      dynsim_2link_planar_B.R_da[dynsim_2link_planar_B.i3 + 6] * T[14] +
      (dynsim_2link_planar_B.R_da[dynsim_2link_planar_B.i3 + 3] * T[13] +
       dynsim_2link_planar_B.R_da[dynsim_2link_planar_B.i3] * T[12]);
  }

  Tinv[3] = 0.0;
  Tinv[7] = 0.0;
  Tinv[11] = 0.0;
  Tinv[15] = 1.0;
}

static void dynsim_2lin_tformToSpatialXform(const real_T T[16], real_T X[36])
{
  dynsim_2link_planar_B.dv2[0] = 0.0;
  dynsim_2link_planar_B.dv2[3] = -T[14];
  dynsim_2link_planar_B.dv2[6] = T[13];
  dynsim_2link_planar_B.dv2[1] = T[14];
  dynsim_2link_planar_B.dv2[4] = 0.0;
  dynsim_2link_planar_B.dv2[7] = -T[12];
  dynsim_2link_planar_B.dv2[2] = -T[13];
  dynsim_2link_planar_B.dv2[5] = T[12];
  dynsim_2link_planar_B.dv2[8] = 0.0;
  for (dynsim_2link_planar_B.i1 = 0; dynsim_2link_planar_B.i1 < 3;
       dynsim_2link_planar_B.i1++) {
    for (dynsim_2link_planar_B.X_tmp_p = 0; dynsim_2link_planar_B.X_tmp_p < 3;
         dynsim_2link_planar_B.X_tmp_p++) {
      dynsim_2link_planar_B.X_tmp_p2 = dynsim_2link_planar_B.i1 + 3 *
        dynsim_2link_planar_B.X_tmp_p;
      dynsim_2link_planar_B.dv3[dynsim_2link_planar_B.X_tmp_p2] = 0.0;
      dynsim_2link_planar_B.i2 = dynsim_2link_planar_B.X_tmp_p << 2;
      dynsim_2link_planar_B.dv3[dynsim_2link_planar_B.X_tmp_p2] +=
        T[dynsim_2link_planar_B.i2] *
        dynsim_2link_planar_B.dv2[dynsim_2link_planar_B.i1];
      dynsim_2link_planar_B.dv3[dynsim_2link_planar_B.X_tmp_p2] +=
        T[dynsim_2link_planar_B.i2 + 1] *
        dynsim_2link_planar_B.dv2[dynsim_2link_planar_B.i1 + 3];
      dynsim_2link_planar_B.dv3[dynsim_2link_planar_B.X_tmp_p2] +=
        T[dynsim_2link_planar_B.i2 + 2] *
        dynsim_2link_planar_B.dv2[dynsim_2link_planar_B.i1 + 6];
      X[dynsim_2link_planar_B.X_tmp_p + 6 * dynsim_2link_planar_B.i1] = T
        [(dynsim_2link_planar_B.i1 << 2) + dynsim_2link_planar_B.X_tmp_p];
      X[dynsim_2link_planar_B.X_tmp_p + 6 * (dynsim_2link_planar_B.i1 + 3)] =
        0.0;
    }
  }

  for (dynsim_2link_planar_B.i1 = 0; dynsim_2link_planar_B.i1 < 3;
       dynsim_2link_planar_B.i1++) {
    X[6 * dynsim_2link_planar_B.i1 + 3] = dynsim_2link_planar_B.dv3[3 *
      dynsim_2link_planar_B.i1];
    dynsim_2link_planar_B.X_tmp_p = dynsim_2link_planar_B.i1 << 2;
    dynsim_2link_planar_B.X_tmp_p2 = 6 * (dynsim_2link_planar_B.i1 + 3);
    X[dynsim_2link_planar_B.X_tmp_p2 + 3] = T[dynsim_2link_planar_B.X_tmp_p];
    X[6 * dynsim_2link_planar_B.i1 + 4] = dynsim_2link_planar_B.dv3[3 *
      dynsim_2link_planar_B.i1 + 1];
    X[dynsim_2link_planar_B.X_tmp_p2 + 4] = T[dynsim_2link_planar_B.X_tmp_p + 1];
    X[6 * dynsim_2link_planar_B.i1 + 5] = dynsim_2link_planar_B.dv3[3 *
      dynsim_2link_planar_B.i1 + 2];
    X[dynsim_2link_planar_B.X_tmp_p2 + 5] = T[dynsim_2link_planar_B.X_tmp_p + 2];
  }
}

static void dynsim_2li_emxFree_f_cell_wrap1(emxArray_f_cell_wrap_dynsim_a_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_f_cell_wrap_dynsim_a_T *)NULL) {
    if (((*pEmxArray)->data != (f_cell_wrap_dynsim_2link_plan_T *)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_f_cell_wrap_dynsim_a_T *)NULL;
  }
}

static void RigidBodyTreeDynamics_massMatri(p_robotics_manip_internal_Rig_T
  *robot, const real_T q[6], emxArray_real_T_dynsim_2link__T *H,
  emxArray_real_T_dynsim_2link__T *lambda)
{
  emxArray_f_cell_wrap_dynsim_a_T *Ic;
  emxArray_f_cell_wrap_dynsim_a_T *X;
  emxArray_real_T_dynsim_2link__T *lambda_;
  emxArray_real_T_dynsim_2link__T *Si;
  emxArray_real_T_dynsim_2link__T *Fi;
  emxArray_real_T_dynsim_2link__T *Hji;
  emxArray_real_T_dynsim_2link__T *s;
  n_robotics_manip_internal_Rig_T *obj;
  emxArray_char_T_dynsim_2link__T *a;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  boolean_T exitg1;
  int32_T exitg2;
  dynsim_2link_planar_B.nb_o = robot->NumBodies;
  dynsim_2link_planar_B.vNum_l = robot->VelocityNumber;
  dynsim_2link_planar_B.f_c = H->size[0] * H->size[1];
  dynsim_2link_planar_B.b_i_m = static_cast<int32_T>
    (dynsim_2link_planar_B.vNum_l);
  H->size[0] = dynsim_2link_planar_B.b_i_m;
  H->size[1] = dynsim_2link_planar_B.b_i_m;
  dynsim_emxEnsureCapacity_real_T(H, dynsim_2link_planar_B.f_c);
  dynsim_2link_planar_B.loop_ub_a = dynsim_2link_planar_B.b_i_m *
    dynsim_2link_planar_B.b_i_m - 1;
  for (dynsim_2link_planar_B.f_c = 0; dynsim_2link_planar_B.f_c <=
       dynsim_2link_planar_B.loop_ub_a; dynsim_2link_planar_B.f_c++) {
    H->data[dynsim_2link_planar_B.f_c] = 0.0;
  }

  dynsim_2link_pla_emxInit_real_T(&lambda_, 2);
  dynsim_2link_planar_B.f_c = lambda_->size[0] * lambda_->size[1];
  lambda_->size[0] = 1;
  dynsim_2link_planar_B.nm1d2 = static_cast<int32_T>(dynsim_2link_planar_B.nb_o);
  lambda_->size[1] = dynsim_2link_planar_B.nm1d2;
  dynsim_emxEnsureCapacity_real_T(lambda_, dynsim_2link_planar_B.f_c);
  dynsim_2link_planar_B.idx = dynsim_2link_planar_B.nm1d2 - 1;
  for (dynsim_2link_planar_B.f_c = 0; dynsim_2link_planar_B.f_c <=
       dynsim_2link_planar_B.idx; dynsim_2link_planar_B.f_c++) {
    lambda_->data[dynsim_2link_planar_B.f_c] = 0.0;
  }

  dynsim_2link_planar_B.f_c = lambda->size[0] * lambda->size[1];
  lambda->size[0] = 1;
  lambda->size[1] = dynsim_2link_planar_B.b_i_m;
  dynsim_emxEnsureCapacity_real_T(lambda, dynsim_2link_planar_B.f_c);
  dynsim_2link_planar_B.loop_ub_a = dynsim_2link_planar_B.b_i_m - 1;
  for (dynsim_2link_planar_B.f_c = 0; dynsim_2link_planar_B.f_c <=
       dynsim_2link_planar_B.loop_ub_a; dynsim_2link_planar_B.f_c++) {
    lambda->data[dynsim_2link_planar_B.f_c] = 0.0;
  }

  dynsim_2li_emxInit_f_cell_wrap1(&Ic, 2);
  dynsim_2li_emxInit_f_cell_wrap1(&X, 2);
  dynsim_2link_planar_B.f_c = Ic->size[0] * Ic->size[1];
  Ic->size[0] = 1;
  Ic->size[1] = dynsim_2link_planar_B.nm1d2;
  d_emxEnsureCapacity_f_cell_wrap(Ic, dynsim_2link_planar_B.f_c);
  dynsim_2link_planar_B.f_c = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = dynsim_2link_planar_B.nm1d2;
  d_emxEnsureCapacity_f_cell_wrap(X, dynsim_2link_planar_B.f_c);
  for (dynsim_2link_planar_B.b_i_m = 0; dynsim_2link_planar_B.b_i_m <=
       dynsim_2link_planar_B.idx; dynsim_2link_planar_B.b_i_m++) {
    for (dynsim_2link_planar_B.f_c = 0; dynsim_2link_planar_B.f_c < 36;
         dynsim_2link_planar_B.f_c++) {
      Ic->data[dynsim_2link_planar_B.b_i_m].f1[dynsim_2link_planar_B.f_c] =
        robot->Bodies[dynsim_2link_planar_B.b_i_m]->
        SpatialInertia[dynsim_2link_planar_B.f_c];
    }

    dynsim_2link_planar_B.vNum_l = robot->
      PositionDoFMap[dynsim_2link_planar_B.b_i_m];
    dynsim_2link_planar_B.p_idx_1 = robot->
      PositionDoFMap[dynsim_2link_planar_B.b_i_m + 8];
    if (dynsim_2link_planar_B.p_idx_1 < dynsim_2link_planar_B.vNum_l) {
      obj = robot->Bodies[dynsim_2link_planar_B.b_i_m];
      rigidBodyJoint_transformBodyToP(&obj->JointInternal,
        dynsim_2link_planar_B.T_f);
    } else {
      if (dynsim_2link_planar_B.vNum_l > dynsim_2link_planar_B.p_idx_1) {
        dynsim_2link_planar_B.nm1d2 = 0;
        dynsim_2link_planar_B.f_c = -1;
      } else {
        dynsim_2link_planar_B.nm1d2 = static_cast<int32_T>
          (dynsim_2link_planar_B.vNum_l) - 1;
        dynsim_2link_planar_B.f_c = static_cast<int32_T>
          (dynsim_2link_planar_B.p_idx_1) - 1;
      }

      obj = robot->Bodies[dynsim_2link_planar_B.b_i_m];
      dynsim_2link_planar_B.loop_ub_a = dynsim_2link_planar_B.f_c -
        dynsim_2link_planar_B.nm1d2;
      dynsim_2link_planar_B.q_size_k = dynsim_2link_planar_B.loop_ub_a + 1;
      for (dynsim_2link_planar_B.f_c = 0; dynsim_2link_planar_B.f_c <=
           dynsim_2link_planar_B.loop_ub_a; dynsim_2link_planar_B.f_c++) {
        dynsim_2link_planar_B.q_data_j[dynsim_2link_planar_B.f_c] =
          q[dynsim_2link_planar_B.nm1d2 + dynsim_2link_planar_B.f_c];
      }

      rigidBodyJoint_transformBodyT_a(&obj->JointInternal,
        dynsim_2link_planar_B.q_data_j, &dynsim_2link_planar_B.q_size_k,
        dynsim_2link_planar_B.T_f);
    }

    dynsim_2link_planar_tforminv(dynsim_2link_planar_B.T_f,
      dynsim_2link_planar_B.dv);
    dynsim_2lin_tformToSpatialXform(dynsim_2link_planar_B.dv, X->
      data[dynsim_2link_planar_B.b_i_m].f1);
  }

  dynsim_2link_planar_B.nm1d2 = static_cast<int32_T>(((-1.0 -
    dynsim_2link_planar_B.nb_o) + 1.0) / -1.0) - 1;
  dynsim_2link_pla_emxInit_real_T(&Si, 2);
  dynsim_2link_pla_emxInit_real_T(&Fi, 2);
  dynsim_2link_pla_emxInit_real_T(&Hji, 2);
  dynsim_2link_pla_emxInit_char_T(&a, 2);
  for (dynsim_2link_planar_B.idx = 0; dynsim_2link_planar_B.idx <=
       dynsim_2link_planar_B.nm1d2; dynsim_2link_planar_B.idx++) {
    dynsim_2link_planar_B.n_f = static_cast<int32_T>(dynsim_2link_planar_B.nb_o
      + -static_cast<real_T>(dynsim_2link_planar_B.idx));
    dynsim_2link_planar_B.pid_tmp = dynsim_2link_planar_B.n_f - 1;
    dynsim_2link_planar_B.pid = robot->Bodies[dynsim_2link_planar_B.pid_tmp]
      ->ParentIndex;
    dynsim_2link_planar_B.vNum_l = robot->
      VelocityDoFMap[dynsim_2link_planar_B.n_f - 1];
    dynsim_2link_planar_B.p_idx_1 = robot->
      VelocityDoFMap[dynsim_2link_planar_B.n_f + 7];
    if (dynsim_2link_planar_B.pid > 0.0) {
      for (dynsim_2link_planar_B.f_c = 0; dynsim_2link_planar_B.f_c < 6;
           dynsim_2link_planar_B.f_c++) {
        for (dynsim_2link_planar_B.b_i_m = 0; dynsim_2link_planar_B.b_i_m < 6;
             dynsim_2link_planar_B.b_i_m++) {
          dynsim_2link_planar_B.X_tmp = dynsim_2link_planar_B.f_c + 6 *
            dynsim_2link_planar_B.b_i_m;
          dynsim_2link_planar_B.X_m[dynsim_2link_planar_B.X_tmp] = 0.0;
          for (dynsim_2link_planar_B.loop_ub_a = 0;
               dynsim_2link_planar_B.loop_ub_a < 6;
               dynsim_2link_planar_B.loop_ub_a++) {
            dynsim_2link_planar_B.X_m[dynsim_2link_planar_B.X_tmp] += X->
              data[dynsim_2link_planar_B.pid_tmp].f1[6 *
              dynsim_2link_planar_B.f_c + dynsim_2link_planar_B.loop_ub_a] *
              Ic->data[dynsim_2link_planar_B.pid_tmp].f1[6 *
              dynsim_2link_planar_B.b_i_m + dynsim_2link_planar_B.loop_ub_a];
          }
        }
      }

      for (dynsim_2link_planar_B.f_c = 0; dynsim_2link_planar_B.f_c < 6;
           dynsim_2link_planar_B.f_c++) {
        for (dynsim_2link_planar_B.b_i_m = 0; dynsim_2link_planar_B.b_i_m < 6;
             dynsim_2link_planar_B.b_i_m++) {
          dynsim_2link_planar_B.b_idx_0_m = 0.0;
          for (dynsim_2link_planar_B.loop_ub_a = 0;
               dynsim_2link_planar_B.loop_ub_a < 6;
               dynsim_2link_planar_B.loop_ub_a++) {
            dynsim_2link_planar_B.b_idx_0_m += dynsim_2link_planar_B.X_m[6 *
              dynsim_2link_planar_B.loop_ub_a + dynsim_2link_planar_B.f_c] *
              X->data[dynsim_2link_planar_B.pid_tmp].f1[6 *
              dynsim_2link_planar_B.b_i_m + dynsim_2link_planar_B.loop_ub_a];
          }

          dynsim_2link_planar_B.loop_ub_a = 6 * dynsim_2link_planar_B.b_i_m +
            dynsim_2link_planar_B.f_c;
          Ic->data[static_cast<int32_T>(dynsim_2link_planar_B.pid) - 1]
            .f1[dynsim_2link_planar_B.loop_ub_a] +=
            dynsim_2link_planar_B.b_idx_0_m;
        }
      }

      lambda_->data[dynsim_2link_planar_B.pid_tmp] = dynsim_2link_planar_B.pid;
      exitg1 = false;
      while ((!exitg1) && (lambda_->data[dynsim_2link_planar_B.pid_tmp] > 0.0))
      {
        obj = robot->Bodies[static_cast<int32_T>(lambda_->
          data[dynsim_2link_planar_B.pid_tmp]) - 1];
        dynsim_2link_planar_B.f_c = a->size[0] * a->size[1];
        a->size[0] = 1;
        a->size[1] = obj->JointInternal.Type->size[1];
        dynsim_emxEnsureCapacity_char_T(a, dynsim_2link_planar_B.f_c);
        dynsim_2link_planar_B.loop_ub_a = obj->JointInternal.Type->size[0] *
          obj->JointInternal.Type->size[1] - 1;
        for (dynsim_2link_planar_B.f_c = 0; dynsim_2link_planar_B.f_c <=
             dynsim_2link_planar_B.loop_ub_a; dynsim_2link_planar_B.f_c++) {
          a->data[dynsim_2link_planar_B.f_c] = obj->JointInternal.Type->
            data[dynsim_2link_planar_B.f_c];
        }

        for (dynsim_2link_planar_B.f_c = 0; dynsim_2link_planar_B.f_c < 5;
             dynsim_2link_planar_B.f_c++) {
          dynsim_2link_planar_B.b_fi[dynsim_2link_planar_B.f_c] =
            tmp[dynsim_2link_planar_B.f_c];
        }

        dynsim_2link_planar_B.b_bool_l = false;
        if (a->size[1] == 5) {
          dynsim_2link_planar_B.f_c = 1;
          do {
            exitg2 = 0;
            if (dynsim_2link_planar_B.f_c - 1 < 5) {
              dynsim_2link_planar_B.b_i_m = dynsim_2link_planar_B.f_c - 1;
              if (a->data[dynsim_2link_planar_B.b_i_m] !=
                  dynsim_2link_planar_B.b_fi[dynsim_2link_planar_B.b_i_m]) {
                exitg2 = 1;
              } else {
                dynsim_2link_planar_B.f_c++;
              }
            } else {
              dynsim_2link_planar_B.b_bool_l = true;
              exitg2 = 1;
            }
          } while (exitg2 == 0);
        }

        if (dynsim_2link_planar_B.b_bool_l) {
          lambda_->data[dynsim_2link_planar_B.pid_tmp] = robot->Bodies[
            static_cast<int32_T>(lambda_->data[dynsim_2link_planar_B.pid_tmp]) -
            1]->ParentIndex;
        } else {
          exitg1 = true;
        }
      }
    }

    dynsim_2link_planar_B.b_idx_0_m = robot->
      VelocityDoFMap[dynsim_2link_planar_B.n_f - 1];
    dynsim_2link_planar_B.b_idx_1_c = robot->
      VelocityDoFMap[dynsim_2link_planar_B.n_f + 7];
    if (dynsim_2link_planar_B.b_idx_0_m <= dynsim_2link_planar_B.b_idx_1_c) {
      obj = robot->Bodies[dynsim_2link_planar_B.pid_tmp];
      dynsim_2link_planar_B.f_c = Si->size[0] * Si->size[1];
      Si->size[0] = 6;
      Si->size[1] = obj->JointInternal.MotionSubspace->size[1];
      dynsim_emxEnsureCapacity_real_T(Si, dynsim_2link_planar_B.f_c);
      dynsim_2link_planar_B.loop_ub_a = obj->JointInternal.MotionSubspace->size
        [0] * obj->JointInternal.MotionSubspace->size[1] - 1;
      for (dynsim_2link_planar_B.f_c = 0; dynsim_2link_planar_B.f_c <=
           dynsim_2link_planar_B.loop_ub_a; dynsim_2link_planar_B.f_c++) {
        Si->data[dynsim_2link_planar_B.f_c] = obj->
          JointInternal.MotionSubspace->data[dynsim_2link_planar_B.f_c];
      }

      dynsim_2link_planar_B.n_f = Si->size[1] - 1;
      dynsim_2link_planar_B.f_c = Fi->size[0] * Fi->size[1];
      Fi->size[0] = 6;
      Fi->size[1] = Si->size[1];
      dynsim_emxEnsureCapacity_real_T(Fi, dynsim_2link_planar_B.f_c);
      for (dynsim_2link_planar_B.loop_ub_a = 0; dynsim_2link_planar_B.loop_ub_a <=
           dynsim_2link_planar_B.n_f; dynsim_2link_planar_B.loop_ub_a++) {
        dynsim_2link_planar_B.coffset_tmp_p = dynsim_2link_planar_B.loop_ub_a *
          6 - 1;
        for (dynsim_2link_planar_B.b_i_m = 0; dynsim_2link_planar_B.b_i_m < 6;
             dynsim_2link_planar_B.b_i_m++) {
          dynsim_2link_planar_B.s_m = 0.0;
          for (dynsim_2link_planar_B.f_c = 0; dynsim_2link_planar_B.f_c < 6;
               dynsim_2link_planar_B.f_c++) {
            dynsim_2link_planar_B.s_m += Ic->data[dynsim_2link_planar_B.pid_tmp]
              .f1[dynsim_2link_planar_B.f_c * 6 + dynsim_2link_planar_B.b_i_m] *
              Si->data[(dynsim_2link_planar_B.coffset_tmp_p +
                        dynsim_2link_planar_B.f_c) + 1];
          }

          Fi->data[(dynsim_2link_planar_B.coffset_tmp_p +
                    dynsim_2link_planar_B.b_i_m) + 1] =
            dynsim_2link_planar_B.s_m;
        }
      }

      if (dynsim_2link_planar_B.vNum_l > dynsim_2link_planar_B.p_idx_1) {
        dynsim_2link_planar_B.coffset_tmp_p = 0;
        dynsim_2link_planar_B.cb = 0;
      } else {
        dynsim_2link_planar_B.coffset_tmp_p = static_cast<int32_T>
          (dynsim_2link_planar_B.vNum_l) - 1;
        dynsim_2link_planar_B.cb = static_cast<int32_T>
          (dynsim_2link_planar_B.vNum_l) - 1;
      }

      dynsim_2link_planar_B.m_h = Si->size[1];
      dynsim_2link_planar_B.n_f = Fi->size[1] - 1;
      dynsim_2link_planar_B.f_c = Hji->size[0] * Hji->size[1];
      Hji->size[0] = Si->size[1];
      Hji->size[1] = Fi->size[1];
      dynsim_emxEnsureCapacity_real_T(Hji, dynsim_2link_planar_B.f_c);
      for (dynsim_2link_planar_B.loop_ub_a = 0; dynsim_2link_planar_B.loop_ub_a <=
           dynsim_2link_planar_B.n_f; dynsim_2link_planar_B.loop_ub_a++) {
        dynsim_2link_planar_B.coffset = dynsim_2link_planar_B.loop_ub_a *
          dynsim_2link_planar_B.m_h - 1;
        dynsim_2link_planar_B.boffset = dynsim_2link_planar_B.loop_ub_a * 6 - 1;
        for (dynsim_2link_planar_B.b_i_m = 0; dynsim_2link_planar_B.b_i_m <
             dynsim_2link_planar_B.m_h; dynsim_2link_planar_B.b_i_m++) {
          dynsim_2link_planar_B.aoffset_m = dynsim_2link_planar_B.b_i_m * 6 - 1;
          dynsim_2link_planar_B.s_m = 0.0;
          for (dynsim_2link_planar_B.f_c = 0; dynsim_2link_planar_B.f_c < 6;
               dynsim_2link_planar_B.f_c++) {
            dynsim_2link_planar_B.X_tmp = dynsim_2link_planar_B.f_c + 1;
            dynsim_2link_planar_B.s_m += Si->
              data[dynsim_2link_planar_B.aoffset_m + dynsim_2link_planar_B.X_tmp]
              * Fi->data[dynsim_2link_planar_B.boffset +
              dynsim_2link_planar_B.X_tmp];
          }

          Hji->data[(dynsim_2link_planar_B.coffset + dynsim_2link_planar_B.b_i_m)
            + 1] = dynsim_2link_planar_B.s_m;
        }
      }

      dynsim_2link_planar_B.loop_ub_a = Hji->size[1];
      for (dynsim_2link_planar_B.f_c = 0; dynsim_2link_planar_B.f_c <
           dynsim_2link_planar_B.loop_ub_a; dynsim_2link_planar_B.f_c++) {
        dynsim_2link_planar_B.n_f = Hji->size[0];
        for (dynsim_2link_planar_B.b_i_m = 0; dynsim_2link_planar_B.b_i_m <
             dynsim_2link_planar_B.n_f; dynsim_2link_planar_B.b_i_m++) {
          H->data[(dynsim_2link_planar_B.coffset_tmp_p +
                   dynsim_2link_planar_B.b_i_m) + H->size[0] *
            (dynsim_2link_planar_B.cb + dynsim_2link_planar_B.f_c)] = Hji->
            data[Hji->size[0] * dynsim_2link_planar_B.f_c +
            dynsim_2link_planar_B.b_i_m];
        }
      }

      dynsim_2link_planar_B.n_f = Fi->size[1];
      dynsim_2link_planar_B.f_c = Si->size[0] * Si->size[1];
      Si->size[0] = 6;
      Si->size[1] = Fi->size[1];
      dynsim_emxEnsureCapacity_real_T(Si, dynsim_2link_planar_B.f_c);
      dynsim_2link_planar_B.loop_ub_a = Fi->size[0] * Fi->size[1] - 1;
      for (dynsim_2link_planar_B.f_c = 0; dynsim_2link_planar_B.f_c <=
           dynsim_2link_planar_B.loop_ub_a; dynsim_2link_planar_B.f_c++) {
        Si->data[dynsim_2link_planar_B.f_c] = Fi->data[dynsim_2link_planar_B.f_c];
      }

      dynsim_2link_planar_B.f_c = Fi->size[0] * Fi->size[1];
      Fi->size[0] = 6;
      Fi->size[1] = dynsim_2link_planar_B.n_f;
      dynsim_emxEnsureCapacity_real_T(Fi, dynsim_2link_planar_B.f_c);
      for (dynsim_2link_planar_B.loop_ub_a = 0; dynsim_2link_planar_B.loop_ub_a <
           dynsim_2link_planar_B.n_f; dynsim_2link_planar_B.loop_ub_a++) {
        dynsim_2link_planar_B.coffset_tmp_p = dynsim_2link_planar_B.loop_ub_a *
          6 - 1;
        for (dynsim_2link_planar_B.b_i_m = 0; dynsim_2link_planar_B.b_i_m < 6;
             dynsim_2link_planar_B.b_i_m++) {
          dynsim_2link_planar_B.aoffset_m = dynsim_2link_planar_B.b_i_m * 6 - 1;
          dynsim_2link_planar_B.s_m = 0.0;
          for (dynsim_2link_planar_B.f_c = 0; dynsim_2link_planar_B.f_c < 6;
               dynsim_2link_planar_B.f_c++) {
            dynsim_2link_planar_B.X_tmp = dynsim_2link_planar_B.f_c + 1;
            dynsim_2link_planar_B.s_m += X->data[dynsim_2link_planar_B.pid_tmp].
              f1[dynsim_2link_planar_B.aoffset_m + dynsim_2link_planar_B.X_tmp] *
              Si->data[dynsim_2link_planar_B.coffset_tmp_p +
              dynsim_2link_planar_B.X_tmp];
          }

          Fi->data[(dynsim_2link_planar_B.coffset_tmp_p +
                    dynsim_2link_planar_B.b_i_m) + 1] =
            dynsim_2link_planar_B.s_m;
        }
      }

      while (dynsim_2link_planar_B.pid > 0.0) {
        dynsim_2link_planar_B.b_i_m = static_cast<int32_T>
          (dynsim_2link_planar_B.pid);
        dynsim_2link_planar_B.pid_tmp = dynsim_2link_planar_B.b_i_m - 1;
        obj = robot->Bodies[dynsim_2link_planar_B.pid_tmp];
        dynsim_2link_planar_B.f_c = Si->size[0] * Si->size[1];
        Si->size[0] = 6;
        Si->size[1] = obj->JointInternal.MotionSubspace->size[1];
        dynsim_emxEnsureCapacity_real_T(Si, dynsim_2link_planar_B.f_c);
        dynsim_2link_planar_B.loop_ub_a = obj->
          JointInternal.MotionSubspace->size[0] *
          obj->JointInternal.MotionSubspace->size[1] - 1;
        for (dynsim_2link_planar_B.f_c = 0; dynsim_2link_planar_B.f_c <=
             dynsim_2link_planar_B.loop_ub_a; dynsim_2link_planar_B.f_c++) {
          Si->data[dynsim_2link_planar_B.f_c] =
            obj->JointInternal.MotionSubspace->data[dynsim_2link_planar_B.f_c];
        }

        dynsim_2link_planar_B.b_idx_0_m = robot->
          VelocityDoFMap[dynsim_2link_planar_B.b_i_m - 1];
        dynsim_2link_planar_B.b_idx_1_c = robot->
          VelocityDoFMap[dynsim_2link_planar_B.b_i_m + 7];
        if (dynsim_2link_planar_B.b_idx_0_m <= dynsim_2link_planar_B.b_idx_1_c)
        {
          dynsim_2link_planar_B.m_h = Si->size[1];
          dynsim_2link_planar_B.n_f = Fi->size[1] - 1;
          dynsim_2link_planar_B.f_c = Hji->size[0] * Hji->size[1];
          Hji->size[0] = Si->size[1];
          Hji->size[1] = Fi->size[1];
          dynsim_emxEnsureCapacity_real_T(Hji, dynsim_2link_planar_B.f_c);
          for (dynsim_2link_planar_B.loop_ub_a = 0;
               dynsim_2link_planar_B.loop_ub_a <= dynsim_2link_planar_B.n_f;
               dynsim_2link_planar_B.loop_ub_a++) {
            dynsim_2link_planar_B.coffset = dynsim_2link_planar_B.loop_ub_a *
              dynsim_2link_planar_B.m_h - 1;
            dynsim_2link_planar_B.boffset = dynsim_2link_planar_B.loop_ub_a * 6
              - 1;
            for (dynsim_2link_planar_B.b_i_m = 0; dynsim_2link_planar_B.b_i_m <
                 dynsim_2link_planar_B.m_h; dynsim_2link_planar_B.b_i_m++) {
              dynsim_2link_planar_B.aoffset_m = dynsim_2link_planar_B.b_i_m * 6
                - 1;
              dynsim_2link_planar_B.s_m = 0.0;
              for (dynsim_2link_planar_B.f_c = 0; dynsim_2link_planar_B.f_c < 6;
                   dynsim_2link_planar_B.f_c++) {
                dynsim_2link_planar_B.X_tmp = dynsim_2link_planar_B.f_c + 1;
                dynsim_2link_planar_B.s_m += Si->
                  data[dynsim_2link_planar_B.aoffset_m +
                  dynsim_2link_planar_B.X_tmp] * Fi->
                  data[dynsim_2link_planar_B.boffset +
                  dynsim_2link_planar_B.X_tmp];
              }

              Hji->data[(dynsim_2link_planar_B.coffset +
                         dynsim_2link_planar_B.b_i_m) + 1] =
                dynsim_2link_planar_B.s_m;
            }
          }

          if (dynsim_2link_planar_B.b_idx_0_m > dynsim_2link_planar_B.b_idx_1_c)
          {
            dynsim_2link_planar_B.X_tmp = 0;
          } else {
            dynsim_2link_planar_B.X_tmp = static_cast<int32_T>
              (dynsim_2link_planar_B.b_idx_0_m) - 1;
          }

          if (dynsim_2link_planar_B.vNum_l > dynsim_2link_planar_B.p_idx_1) {
            dynsim_2link_planar_B.coffset_tmp_p = 0;
          } else {
            dynsim_2link_planar_B.coffset_tmp_p = static_cast<int32_T>
              (dynsim_2link_planar_B.vNum_l) - 1;
          }

          dynsim_2link_planar_B.loop_ub_a = Hji->size[1];
          for (dynsim_2link_planar_B.f_c = 0; dynsim_2link_planar_B.f_c <
               dynsim_2link_planar_B.loop_ub_a; dynsim_2link_planar_B.f_c++) {
            dynsim_2link_planar_B.n_f = Hji->size[0];
            for (dynsim_2link_planar_B.b_i_m = 0; dynsim_2link_planar_B.b_i_m <
                 dynsim_2link_planar_B.n_f; dynsim_2link_planar_B.b_i_m++) {
              H->data[(dynsim_2link_planar_B.X_tmp + dynsim_2link_planar_B.b_i_m)
                + H->size[0] * (dynsim_2link_planar_B.coffset_tmp_p +
                                dynsim_2link_planar_B.f_c)] = Hji->data
                [Hji->size[0] * dynsim_2link_planar_B.f_c +
                dynsim_2link_planar_B.b_i_m];
            }
          }

          if (dynsim_2link_planar_B.vNum_l > dynsim_2link_planar_B.p_idx_1) {
            dynsim_2link_planar_B.X_tmp = 0;
          } else {
            dynsim_2link_planar_B.X_tmp = static_cast<int32_T>
              (dynsim_2link_planar_B.vNum_l) - 1;
          }

          if (dynsim_2link_planar_B.b_idx_0_m > dynsim_2link_planar_B.b_idx_1_c)
          {
            dynsim_2link_planar_B.coffset_tmp_p = 0;
          } else {
            dynsim_2link_planar_B.coffset_tmp_p = static_cast<int32_T>
              (dynsim_2link_planar_B.b_idx_0_m) - 1;
          }

          dynsim_2link_planar_B.loop_ub_a = Hji->size[0];
          for (dynsim_2link_planar_B.f_c = 0; dynsim_2link_planar_B.f_c <
               dynsim_2link_planar_B.loop_ub_a; dynsim_2link_planar_B.f_c++) {
            dynsim_2link_planar_B.n_f = Hji->size[1];
            for (dynsim_2link_planar_B.b_i_m = 0; dynsim_2link_planar_B.b_i_m <
                 dynsim_2link_planar_B.n_f; dynsim_2link_planar_B.b_i_m++) {
              H->data[(dynsim_2link_planar_B.X_tmp + dynsim_2link_planar_B.b_i_m)
                + H->size[0] * (dynsim_2link_planar_B.coffset_tmp_p +
                                dynsim_2link_planar_B.f_c)] = Hji->data
                [Hji->size[0] * dynsim_2link_planar_B.b_i_m +
                dynsim_2link_planar_B.f_c];
            }
          }
        }

        dynsim_2link_planar_B.n_f = Fi->size[1];
        dynsim_2link_planar_B.f_c = Si->size[0] * Si->size[1];
        Si->size[0] = 6;
        Si->size[1] = Fi->size[1];
        dynsim_emxEnsureCapacity_real_T(Si, dynsim_2link_planar_B.f_c);
        dynsim_2link_planar_B.loop_ub_a = Fi->size[0] * Fi->size[1] - 1;
        for (dynsim_2link_planar_B.f_c = 0; dynsim_2link_planar_B.f_c <=
             dynsim_2link_planar_B.loop_ub_a; dynsim_2link_planar_B.f_c++) {
          Si->data[dynsim_2link_planar_B.f_c] = Fi->
            data[dynsim_2link_planar_B.f_c];
        }

        dynsim_2link_planar_B.f_c = Fi->size[0] * Fi->size[1];
        Fi->size[0] = 6;
        Fi->size[1] = dynsim_2link_planar_B.n_f;
        dynsim_emxEnsureCapacity_real_T(Fi, dynsim_2link_planar_B.f_c);
        for (dynsim_2link_planar_B.loop_ub_a = 0;
             dynsim_2link_planar_B.loop_ub_a < dynsim_2link_planar_B.n_f;
             dynsim_2link_planar_B.loop_ub_a++) {
          dynsim_2link_planar_B.coffset_tmp_p = dynsim_2link_planar_B.loop_ub_a *
            6 - 1;
          for (dynsim_2link_planar_B.b_i_m = 0; dynsim_2link_planar_B.b_i_m < 6;
               dynsim_2link_planar_B.b_i_m++) {
            dynsim_2link_planar_B.aoffset_m = dynsim_2link_planar_B.b_i_m * 6 -
              1;
            dynsim_2link_planar_B.s_m = 0.0;
            for (dynsim_2link_planar_B.f_c = 0; dynsim_2link_planar_B.f_c < 6;
                 dynsim_2link_planar_B.f_c++) {
              dynsim_2link_planar_B.X_tmp = dynsim_2link_planar_B.f_c + 1;
              dynsim_2link_planar_B.s_m += X->data[dynsim_2link_planar_B.pid_tmp]
                .f1[dynsim_2link_planar_B.aoffset_m +
                dynsim_2link_planar_B.X_tmp] * Si->
                data[dynsim_2link_planar_B.coffset_tmp_p +
                dynsim_2link_planar_B.X_tmp];
            }

            Fi->data[(dynsim_2link_planar_B.coffset_tmp_p +
                      dynsim_2link_planar_B.b_i_m) + 1] =
              dynsim_2link_planar_B.s_m;
          }
        }

        dynsim_2link_planar_B.pid = robot->Bodies[dynsim_2link_planar_B.pid_tmp
          ]->ParentIndex;
      }
    }
  }

  dynsim_2link_pla_emxFree_char_T(&a);
  dynsim_2link_pla_emxFree_real_T(&Hji);
  dynsim_2link_pla_emxFree_real_T(&Fi);
  dynsim_2link_pla_emxFree_real_T(&Si);
  dynsim_2li_emxFree_f_cell_wrap1(&X);
  dynsim_2li_emxFree_f_cell_wrap1(&Ic);
  for (dynsim_2link_planar_B.f_c = 0; dynsim_2link_planar_B.f_c < 8;
       dynsim_2link_planar_B.f_c++) {
    dynsim_2link_planar_B.mask[dynsim_2link_planar_B.f_c] =
      (robot->VelocityDoFMap[dynsim_2link_planar_B.f_c] <= robot->
       VelocityDoFMap[dynsim_2link_planar_B.f_c + 8]);
  }

  dynsim_2link_planar_B.idx = 0;
  dynsim_2link_planar_B.f_c = 1;
  exitg1 = false;
  while ((!exitg1) && (dynsim_2link_planar_B.f_c - 1 < 8)) {
    if (dynsim_2link_planar_B.mask[dynsim_2link_planar_B.f_c - 1]) {
      dynsim_2link_planar_B.idx++;
      dynsim_2link_planar_B.ii_data[dynsim_2link_planar_B.idx - 1] =
        dynsim_2link_planar_B.f_c;
      if (dynsim_2link_planar_B.idx >= 8) {
        exitg1 = true;
      } else {
        dynsim_2link_planar_B.f_c++;
      }
    } else {
      dynsim_2link_planar_B.f_c++;
    }
  }

  if (1 > dynsim_2link_planar_B.idx) {
    dynsim_2link_planar_B.idx = 0;
  }

  for (dynsim_2link_planar_B.f_c = 0; dynsim_2link_planar_B.f_c <
       dynsim_2link_planar_B.idx; dynsim_2link_planar_B.f_c++) {
    dynsim_2link_planar_B.nonFixedIndices_data[dynsim_2link_planar_B.f_c] =
      dynsim_2link_planar_B.ii_data[dynsim_2link_planar_B.f_c];
  }

  dynsim_2link_planar_B.b_i_m = dynsim_2link_planar_B.idx - 1;
  dynsim_2link_pla_emxInit_real_T(&s, 2);
  for (dynsim_2link_planar_B.idx = 0; dynsim_2link_planar_B.idx <=
       dynsim_2link_planar_B.b_i_m; dynsim_2link_planar_B.idx++) {
    dynsim_2link_planar_B.vNum_l = robot->
      VelocityDoFMap[dynsim_2link_planar_B.nonFixedIndices_data[dynsim_2link_planar_B.idx]
      - 1];
    dynsim_2link_planar_B.p_idx_1 = robot->
      VelocityDoFMap[dynsim_2link_planar_B.nonFixedIndices_data[dynsim_2link_planar_B.idx]
      + 7];
    if (rtIsNaN(dynsim_2link_planar_B.vNum_l) || rtIsNaN
        (dynsim_2link_planar_B.p_idx_1)) {
      dynsim_2link_planar_B.f_c = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = 1;
      dynsim_emxEnsureCapacity_real_T(s, dynsim_2link_planar_B.f_c);
      s->data[0] = (rtNaN);
    } else if (dynsim_2link_planar_B.p_idx_1 < dynsim_2link_planar_B.vNum_l) {
      s->size[0] = 1;
      s->size[1] = 0;
    } else if ((rtIsInf(dynsim_2link_planar_B.vNum_l) || rtIsInf
                (dynsim_2link_planar_B.p_idx_1)) &&
               (dynsim_2link_planar_B.vNum_l == dynsim_2link_planar_B.p_idx_1))
    {
      dynsim_2link_planar_B.f_c = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = 1;
      dynsim_emxEnsureCapacity_real_T(s, dynsim_2link_planar_B.f_c);
      s->data[0] = (rtNaN);
    } else if (floor(dynsim_2link_planar_B.vNum_l) ==
               dynsim_2link_planar_B.vNum_l) {
      dynsim_2link_planar_B.f_c = s->size[0] * s->size[1];
      s->size[0] = 1;
      dynsim_2link_planar_B.loop_ub_a = static_cast<int32_T>(floor
        (dynsim_2link_planar_B.p_idx_1 - dynsim_2link_planar_B.vNum_l));
      s->size[1] = dynsim_2link_planar_B.loop_ub_a + 1;
      dynsim_emxEnsureCapacity_real_T(s, dynsim_2link_planar_B.f_c);
      for (dynsim_2link_planar_B.f_c = 0; dynsim_2link_planar_B.f_c <=
           dynsim_2link_planar_B.loop_ub_a; dynsim_2link_planar_B.f_c++) {
        s->data[dynsim_2link_planar_B.f_c] = dynsim_2link_planar_B.vNum_l +
          static_cast<real_T>(dynsim_2link_planar_B.f_c);
      }
    } else {
      dynsim_2link_planar_B.nb_o = floor((dynsim_2link_planar_B.p_idx_1 -
        dynsim_2link_planar_B.vNum_l) + 0.5);
      dynsim_2link_planar_B.pid = dynsim_2link_planar_B.vNum_l +
        dynsim_2link_planar_B.nb_o;
      dynsim_2link_planar_B.b_idx_0_m = dynsim_2link_planar_B.pid -
        dynsim_2link_planar_B.p_idx_1;
      dynsim_2link_planar_B.b_idx_1_c = fabs(dynsim_2link_planar_B.vNum_l);
      dynsim_2link_planar_B.s_m = fabs(dynsim_2link_planar_B.p_idx_1);
      if ((dynsim_2link_planar_B.b_idx_1_c > dynsim_2link_planar_B.s_m) ||
          rtIsNaN(dynsim_2link_planar_B.s_m)) {
        dynsim_2link_planar_B.s_m = dynsim_2link_planar_B.b_idx_1_c;
      }

      if (fabs(dynsim_2link_planar_B.b_idx_0_m) < 4.4408920985006262E-16 *
          dynsim_2link_planar_B.s_m) {
        dynsim_2link_planar_B.nb_o++;
        dynsim_2link_planar_B.pid = dynsim_2link_planar_B.p_idx_1;
      } else if (dynsim_2link_planar_B.b_idx_0_m > 0.0) {
        dynsim_2link_planar_B.pid = (dynsim_2link_planar_B.nb_o - 1.0) +
          dynsim_2link_planar_B.vNum_l;
      } else {
        dynsim_2link_planar_B.nb_o++;
      }

      if (dynsim_2link_planar_B.nb_o >= 0.0) {
        dynsim_2link_planar_B.f_c = static_cast<int32_T>
          (dynsim_2link_planar_B.nb_o);
      } else {
        dynsim_2link_planar_B.f_c = 0;
      }

      dynsim_2link_planar_B.n_f = dynsim_2link_planar_B.f_c - 1;
      dynsim_2link_planar_B.f_c = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = dynsim_2link_planar_B.n_f + 1;
      dynsim_emxEnsureCapacity_real_T(s, dynsim_2link_planar_B.f_c);
      if (dynsim_2link_planar_B.n_f + 1 > 0) {
        s->data[0] = dynsim_2link_planar_B.vNum_l;
        if (dynsim_2link_planar_B.n_f + 1 > 1) {
          s->data[dynsim_2link_planar_B.n_f] = dynsim_2link_planar_B.pid;
          dynsim_2link_planar_B.nm1d2 = ((dynsim_2link_planar_B.n_f < 0) +
            dynsim_2link_planar_B.n_f) >> 1;
          dynsim_2link_planar_B.loop_ub_a = dynsim_2link_planar_B.nm1d2 - 2;
          for (dynsim_2link_planar_B.f_c = 0; dynsim_2link_planar_B.f_c <=
               dynsim_2link_planar_B.loop_ub_a; dynsim_2link_planar_B.f_c++) {
            dynsim_2link_planar_B.X_tmp = dynsim_2link_planar_B.f_c + 1;
            s->data[dynsim_2link_planar_B.X_tmp] = dynsim_2link_planar_B.vNum_l
              + static_cast<real_T>(dynsim_2link_planar_B.X_tmp);
            s->data[dynsim_2link_planar_B.n_f - dynsim_2link_planar_B.X_tmp] =
              dynsim_2link_planar_B.pid - static_cast<real_T>
              (dynsim_2link_planar_B.X_tmp);
          }

          if (dynsim_2link_planar_B.nm1d2 << 1 == dynsim_2link_planar_B.n_f) {
            s->data[dynsim_2link_planar_B.nm1d2] = (dynsim_2link_planar_B.vNum_l
              + dynsim_2link_planar_B.pid) / 2.0;
          } else {
            s->data[dynsim_2link_planar_B.nm1d2] = dynsim_2link_planar_B.vNum_l
              + static_cast<real_T>(dynsim_2link_planar_B.nm1d2);
            s->data[dynsim_2link_planar_B.nm1d2 + 1] = dynsim_2link_planar_B.pid
              - static_cast<real_T>(dynsim_2link_planar_B.nm1d2);
          }
        }
      }
    }

    if (dynsim_2link_planar_B.vNum_l > dynsim_2link_planar_B.p_idx_1) {
      dynsim_2link_planar_B.nm1d2 = 0;
    } else {
      dynsim_2link_planar_B.nm1d2 = static_cast<int32_T>
        (dynsim_2link_planar_B.vNum_l) - 1;
    }

    dynsim_2link_planar_B.loop_ub_a = s->size[1];
    for (dynsim_2link_planar_B.f_c = 0; dynsim_2link_planar_B.f_c <
         dynsim_2link_planar_B.loop_ub_a; dynsim_2link_planar_B.f_c++) {
      lambda->data[dynsim_2link_planar_B.nm1d2 + dynsim_2link_planar_B.f_c] =
        s->data[dynsim_2link_planar_B.f_c] - 1.0;
    }

    if (lambda_->
        data[dynsim_2link_planar_B.nonFixedIndices_data[dynsim_2link_planar_B.idx]
        - 1] == 0.0) {
      lambda->data[static_cast<int32_T>(dynsim_2link_planar_B.vNum_l) - 1] = 0.0;
    } else {
      dynsim_2link_planar_B.f_c = static_cast<int32_T>(lambda_->
        data[dynsim_2link_planar_B.nonFixedIndices_data[dynsim_2link_planar_B.idx]
        - 1]);
      dynsim_2link_planar_B.b_idx_1_c = robot->
        VelocityDoFMap[dynsim_2link_planar_B.f_c + 7];
      lambda->data[static_cast<int32_T>(dynsim_2link_planar_B.vNum_l) - 1] =
        dynsim_2link_planar_B.b_idx_1_c;
    }
  }

  dynsim_2link_pla_emxFree_real_T(&s);
  dynsim_2link_pla_emxFree_real_T(&lambda_);
}

static void RigidBodyTreeDynamics_inverseDy(p_robotics_manip_internal_Rig_T
  *robot, const real_T q[6], const real_T qdot[6], const real_T fext[48], real_T
  tau[6])
{
  emxArray_f_cell_wrap_dynsim_a_T *X;
  emxArray_f_cell_wrap_dynsim_a_T *Xtree;
  emxArray_real_T_dynsim_2link__T *vJ;
  emxArray_real_T_dynsim_2link__T *vB;
  emxArray_real_T_dynsim_2link__T *aB;
  emxArray_real_T_dynsim_2link__T *f;
  emxArray_real_T_dynsim_2link__T *S;
  emxArray_real_T_dynsim_2link__T *taui;
  n_robotics_manip_internal_Rig_T *obj;
  emxArray_char_T_dynsim_2link__T *a;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  int32_T exitg1;
  dynsim_2link_planar_B.a0[0] = 0.0;
  dynsim_2link_planar_B.a0[1] = 0.0;
  dynsim_2link_planar_B.a0[2] = 0.0;
  dynsim_2link_planar_B.a0[3] = -robot->Gravity[0];
  dynsim_2link_planar_B.a0[4] = -robot->Gravity[1];
  dynsim_2link_planar_B.a0[5] = -robot->Gravity[2];
  dynsim_2link_pla_emxInit_real_T(&vJ, 2);
  dynsim_2link_planar_B.nb = robot->NumBodies;
  dynsim_2link_planar_B.i_i = vJ->size[0] * vJ->size[1];
  vJ->size[0] = 6;
  dynsim_2link_planar_B.unnamed_idx_1 = static_cast<int32_T>
    (dynsim_2link_planar_B.nb);
  vJ->size[1] = dynsim_2link_planar_B.unnamed_idx_1;
  dynsim_emxEnsureCapacity_real_T(vJ, dynsim_2link_planar_B.i_i);
  dynsim_2link_planar_B.loop_ub_tmp = 6 * dynsim_2link_planar_B.unnamed_idx_1 -
    1;
  for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i <=
       dynsim_2link_planar_B.loop_ub_tmp; dynsim_2link_planar_B.i_i++) {
    vJ->data[dynsim_2link_planar_B.i_i] = 0.0;
  }

  dynsim_2link_pla_emxInit_real_T(&vB, 2);
  dynsim_2link_planar_B.i_i = vB->size[0] * vB->size[1];
  vB->size[0] = 6;
  vB->size[1] = dynsim_2link_planar_B.unnamed_idx_1;
  dynsim_emxEnsureCapacity_real_T(vB, dynsim_2link_planar_B.i_i);
  for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i <=
       dynsim_2link_planar_B.loop_ub_tmp; dynsim_2link_planar_B.i_i++) {
    vB->data[dynsim_2link_planar_B.i_i] = 0.0;
  }

  dynsim_2link_pla_emxInit_real_T(&aB, 2);
  dynsim_2link_planar_B.i_i = aB->size[0] * aB->size[1];
  aB->size[0] = 6;
  aB->size[1] = dynsim_2link_planar_B.unnamed_idx_1;
  dynsim_emxEnsureCapacity_real_T(aB, dynsim_2link_planar_B.i_i);
  for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i <=
       dynsim_2link_planar_B.loop_ub_tmp; dynsim_2link_planar_B.i_i++) {
    aB->data[dynsim_2link_planar_B.i_i] = 0.0;
  }

  for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i < 6;
       dynsim_2link_planar_B.i_i++) {
    tau[dynsim_2link_planar_B.i_i] = 0.0;
  }

  dynsim_2li_emxInit_f_cell_wrap1(&X, 2);
  dynsim_2li_emxInit_f_cell_wrap1(&Xtree, 2);
  dynsim_2link_planar_B.loop_ub_tmp = dynsim_2link_planar_B.unnamed_idx_1 - 1;
  dynsim_2link_planar_B.i_i = Xtree->size[0] * Xtree->size[1];
  Xtree->size[0] = 1;
  Xtree->size[1] = dynsim_2link_planar_B.unnamed_idx_1;
  d_emxEnsureCapacity_f_cell_wrap(Xtree, dynsim_2link_planar_B.i_i);
  dynsim_2link_planar_B.i_i = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = dynsim_2link_planar_B.unnamed_idx_1;
  d_emxEnsureCapacity_f_cell_wrap(X, dynsim_2link_planar_B.i_i);
  for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k <=
       dynsim_2link_planar_B.loop_ub_tmp; dynsim_2link_planar_B.b_k++) {
    for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i < 36;
         dynsim_2link_planar_B.i_i++) {
      Xtree->data[dynsim_2link_planar_B.b_k].f1[dynsim_2link_planar_B.i_i] = 0.0;
    }

    for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i < 6;
         dynsim_2link_planar_B.i_i++) {
      Xtree->data[dynsim_2link_planar_B.b_k].f1[dynsim_2link_planar_B.i_i + 6 *
        dynsim_2link_planar_B.i_i] = 1.0;
    }

    for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i < 36;
         dynsim_2link_planar_B.i_i++) {
      X->data[dynsim_2link_planar_B.b_k].f1[dynsim_2link_planar_B.i_i] = 0.0;
    }

    for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i < 6;
         dynsim_2link_planar_B.i_i++) {
      X->data[dynsim_2link_planar_B.b_k].f1[dynsim_2link_planar_B.i_i + 6 *
        dynsim_2link_planar_B.i_i] = 1.0;
    }
  }

  dynsim_2link_pla_emxInit_real_T(&f, 2);
  dynsim_2link_planar_B.i_i = f->size[0] * f->size[1];
  f->size[0] = 6;
  f->size[1] = dynsim_2link_planar_B.unnamed_idx_1;
  dynsim_emxEnsureCapacity_real_T(f, dynsim_2link_planar_B.i_i);
  dynsim_2link_pla_emxInit_real_T(&S, 2);
  if (0 <= dynsim_2link_planar_B.loop_ub_tmp) {
    dynsim_2link_planar_B.dv1[0] = 0.0;
    dynsim_2link_planar_B.dv1[4] = 0.0;
    dynsim_2link_planar_B.dv1[8] = 0.0;
  }

  for (dynsim_2link_planar_B.unnamed_idx_1 = 0;
       dynsim_2link_planar_B.unnamed_idx_1 <= dynsim_2link_planar_B.loop_ub_tmp;
       dynsim_2link_planar_B.unnamed_idx_1++) {
    obj = robot->Bodies[dynsim_2link_planar_B.unnamed_idx_1];
    dynsim_2link_planar_B.i_i = S->size[0] * S->size[1];
    S->size[0] = 6;
    S->size[1] = obj->JointInternal.MotionSubspace->size[1];
    dynsim_emxEnsureCapacity_real_T(S, dynsim_2link_planar_B.i_i);
    dynsim_2link_planar_B.b_k = obj->JointInternal.MotionSubspace->size[0] *
      obj->JointInternal.MotionSubspace->size[1] - 1;
    for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i <=
         dynsim_2link_planar_B.b_k; dynsim_2link_planar_B.i_i++) {
      S->data[dynsim_2link_planar_B.i_i] = obj->
        JointInternal.MotionSubspace->data[dynsim_2link_planar_B.i_i];
    }

    dynsim_2link_planar_B.a_idx_0 = robot->
      PositionDoFMap[dynsim_2link_planar_B.unnamed_idx_1];
    dynsim_2link_planar_B.a_idx_1 = robot->
      PositionDoFMap[dynsim_2link_planar_B.unnamed_idx_1 + 8];
    dynsim_2link_planar_B.b_idx_0 = robot->
      VelocityDoFMap[dynsim_2link_planar_B.unnamed_idx_1];
    dynsim_2link_planar_B.b_idx_1 = robot->
      VelocityDoFMap[dynsim_2link_planar_B.unnamed_idx_1 + 8];
    if (dynsim_2link_planar_B.a_idx_1 < dynsim_2link_planar_B.a_idx_0) {
      obj = robot->Bodies[dynsim_2link_planar_B.unnamed_idx_1];
      rigidBodyJoint_transformBodyToP(&obj->JointInternal,
        dynsim_2link_planar_B.T_c);
      for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i < 6;
           dynsim_2link_planar_B.i_i++) {
        vJ->data[dynsim_2link_planar_B.i_i + 6 *
          dynsim_2link_planar_B.unnamed_idx_1] = 0.0;
      }
    } else {
      if (dynsim_2link_planar_B.a_idx_0 > dynsim_2link_planar_B.a_idx_1) {
        dynsim_2link_planar_B.b_k = 0;
        dynsim_2link_planar_B.i_i = -1;
      } else {
        dynsim_2link_planar_B.b_k = static_cast<int32_T>
          (dynsim_2link_planar_B.a_idx_0) - 1;
        dynsim_2link_planar_B.i_i = static_cast<int32_T>
          (dynsim_2link_planar_B.a_idx_1) - 1;
      }

      if (dynsim_2link_planar_B.b_idx_0 > dynsim_2link_planar_B.b_idx_1) {
        dynsim_2link_planar_B.p_f = -1;
      } else {
        dynsim_2link_planar_B.p_f = static_cast<int32_T>
          (dynsim_2link_planar_B.b_idx_0) - 2;
      }

      obj = robot->Bodies[dynsim_2link_planar_B.unnamed_idx_1];
      dynsim_2link_planar_B.q_size_tmp = dynsim_2link_planar_B.i_i -
        dynsim_2link_planar_B.b_k;
      dynsim_2link_planar_B.q_size = dynsim_2link_planar_B.q_size_tmp + 1;
      for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i <=
           dynsim_2link_planar_B.q_size_tmp; dynsim_2link_planar_B.i_i++) {
        dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.i_i] =
          q[dynsim_2link_planar_B.b_k + dynsim_2link_planar_B.i_i];
      }

      rigidBodyJoint_transformBodyT_a(&obj->JointInternal,
        dynsim_2link_planar_B.q_data, &dynsim_2link_planar_B.q_size,
        dynsim_2link_planar_B.T_c);
      dynsim_2link_planar_B.inner = S->size[1] - 1;
      for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k < 6;
           dynsim_2link_planar_B.b_k++) {
        vJ->data[dynsim_2link_planar_B.b_k + 6 *
          dynsim_2link_planar_B.unnamed_idx_1] = 0.0;
      }

      for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k <=
           dynsim_2link_planar_B.inner; dynsim_2link_planar_B.b_k++) {
        dynsim_2link_planar_B.aoffset = dynsim_2link_planar_B.b_k * 6 - 1;
        for (dynsim_2link_planar_B.q_size_tmp = 0;
             dynsim_2link_planar_B.q_size_tmp < 6;
             dynsim_2link_planar_B.q_size_tmp++) {
          dynsim_2link_planar_B.i_i = 6 * dynsim_2link_planar_B.unnamed_idx_1 +
            dynsim_2link_planar_B.q_size_tmp;
          vJ->data[dynsim_2link_planar_B.i_i] += S->data
            [(dynsim_2link_planar_B.aoffset + dynsim_2link_planar_B.q_size_tmp)
            + 1] * qdot[(dynsim_2link_planar_B.p_f + dynsim_2link_planar_B.b_k)
            + 1];
        }
      }
    }

    for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i < 3;
         dynsim_2link_planar_B.i_i++) {
      dynsim_2link_planar_B.R_dy[3 * dynsim_2link_planar_B.i_i] =
        dynsim_2link_planar_B.T_c[dynsim_2link_planar_B.i_i];
      dynsim_2link_planar_B.R_dy[3 * dynsim_2link_planar_B.i_i + 1] =
        dynsim_2link_planar_B.T_c[dynsim_2link_planar_B.i_i + 4];
      dynsim_2link_planar_B.R_dy[3 * dynsim_2link_planar_B.i_i + 2] =
        dynsim_2link_planar_B.T_c[dynsim_2link_planar_B.i_i + 8];
    }

    for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i < 9;
         dynsim_2link_planar_B.i_i++) {
      dynsim_2link_planar_B.R_lx[dynsim_2link_planar_B.i_i] =
        -dynsim_2link_planar_B.R_dy[dynsim_2link_planar_B.i_i];
    }

    for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i < 3;
         dynsim_2link_planar_B.i_i++) {
      dynsim_2link_planar_B.b_k = dynsim_2link_planar_B.i_i << 2;
      dynsim_2link_planar_B.Tinv[dynsim_2link_planar_B.b_k] =
        dynsim_2link_planar_B.R_dy[3 * dynsim_2link_planar_B.i_i];
      dynsim_2link_planar_B.Tinv[dynsim_2link_planar_B.b_k + 1] =
        dynsim_2link_planar_B.R_dy[3 * dynsim_2link_planar_B.i_i + 1];
      dynsim_2link_planar_B.Tinv[dynsim_2link_planar_B.b_k + 2] =
        dynsim_2link_planar_B.R_dy[3 * dynsim_2link_planar_B.i_i + 2];
      dynsim_2link_planar_B.Tinv[dynsim_2link_planar_B.i_i + 12] =
        dynsim_2link_planar_B.R_lx[dynsim_2link_planar_B.i_i + 6] *
        dynsim_2link_planar_B.T_c[14] +
        (dynsim_2link_planar_B.R_lx[dynsim_2link_planar_B.i_i + 3] *
         dynsim_2link_planar_B.T_c[13] +
         dynsim_2link_planar_B.R_lx[dynsim_2link_planar_B.i_i] *
         dynsim_2link_planar_B.T_c[12]);
    }

    dynsim_2link_planar_B.Tinv[3] = 0.0;
    dynsim_2link_planar_B.Tinv[7] = 0.0;
    dynsim_2link_planar_B.Tinv[11] = 0.0;
    dynsim_2link_planar_B.Tinv[15] = 1.0;
    dynsim_2link_planar_B.dv1[3] = -dynsim_2link_planar_B.Tinv[14];
    dynsim_2link_planar_B.dv1[6] = dynsim_2link_planar_B.Tinv[13];
    dynsim_2link_planar_B.dv1[1] = dynsim_2link_planar_B.Tinv[14];
    dynsim_2link_planar_B.dv1[7] = -dynsim_2link_planar_B.Tinv[12];
    dynsim_2link_planar_B.dv1[2] = -dynsim_2link_planar_B.Tinv[13];
    dynsim_2link_planar_B.dv1[5] = dynsim_2link_planar_B.Tinv[12];
    for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i < 3;
         dynsim_2link_planar_B.i_i++) {
      for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k < 3;
           dynsim_2link_planar_B.b_k++) {
        dynsim_2link_planar_B.q_size_tmp = dynsim_2link_planar_B.i_i + 3 *
          dynsim_2link_planar_B.b_k;
        dynsim_2link_planar_B.R_dy[dynsim_2link_planar_B.q_size_tmp] = 0.0;
        dynsim_2link_planar_B.p_f = dynsim_2link_planar_B.b_k << 2;
        dynsim_2link_planar_B.R_dy[dynsim_2link_planar_B.q_size_tmp] +=
          dynsim_2link_planar_B.Tinv[dynsim_2link_planar_B.p_f] *
          dynsim_2link_planar_B.dv1[dynsim_2link_planar_B.i_i];
        dynsim_2link_planar_B.R_dy[dynsim_2link_planar_B.q_size_tmp] +=
          dynsim_2link_planar_B.Tinv[dynsim_2link_planar_B.p_f + 1] *
          dynsim_2link_planar_B.dv1[dynsim_2link_planar_B.i_i + 3];
        dynsim_2link_planar_B.R_dy[dynsim_2link_planar_B.q_size_tmp] +=
          dynsim_2link_planar_B.Tinv[dynsim_2link_planar_B.p_f + 2] *
          dynsim_2link_planar_B.dv1[dynsim_2link_planar_B.i_i + 6];
        X->data[dynsim_2link_planar_B.unnamed_idx_1]
          .f1[dynsim_2link_planar_B.b_k + 6 * dynsim_2link_planar_B.i_i] =
          dynsim_2link_planar_B.Tinv[(dynsim_2link_planar_B.i_i << 2) +
          dynsim_2link_planar_B.b_k];
        X->data[dynsim_2link_planar_B.unnamed_idx_1]
          .f1[dynsim_2link_planar_B.b_k + 6 * (dynsim_2link_planar_B.i_i + 3)] =
          0.0;
      }
    }

    for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i < 3;
         dynsim_2link_planar_B.i_i++) {
      X->data[dynsim_2link_planar_B.unnamed_idx_1].f1[6 *
        dynsim_2link_planar_B.i_i + 3] = dynsim_2link_planar_B.R_dy[3 *
        dynsim_2link_planar_B.i_i];
      dynsim_2link_planar_B.b_k = dynsim_2link_planar_B.i_i << 2;
      dynsim_2link_planar_B.q_size_tmp = 6 * (dynsim_2link_planar_B.i_i + 3);
      X->data[dynsim_2link_planar_B.unnamed_idx_1]
        .f1[dynsim_2link_planar_B.q_size_tmp + 3] =
        dynsim_2link_planar_B.Tinv[dynsim_2link_planar_B.b_k];
      X->data[dynsim_2link_planar_B.unnamed_idx_1].f1[6 *
        dynsim_2link_planar_B.i_i + 4] = dynsim_2link_planar_B.R_dy[3 *
        dynsim_2link_planar_B.i_i + 1];
      X->data[dynsim_2link_planar_B.unnamed_idx_1]
        .f1[dynsim_2link_planar_B.q_size_tmp + 4] =
        dynsim_2link_planar_B.Tinv[dynsim_2link_planar_B.b_k + 1];
      X->data[dynsim_2link_planar_B.unnamed_idx_1].f1[6 *
        dynsim_2link_planar_B.i_i + 5] = dynsim_2link_planar_B.R_dy[3 *
        dynsim_2link_planar_B.i_i + 2];
      X->data[dynsim_2link_planar_B.unnamed_idx_1]
        .f1[dynsim_2link_planar_B.q_size_tmp + 5] =
        dynsim_2link_planar_B.Tinv[dynsim_2link_planar_B.b_k + 2];
    }

    dynsim_2link_planar_B.a_idx_0 = robot->
      Bodies[dynsim_2link_planar_B.unnamed_idx_1]->ParentIndex;
    if (dynsim_2link_planar_B.a_idx_0 > 0.0) {
      dynsim_2link_planar_B.m = static_cast<int32_T>
        (dynsim_2link_planar_B.a_idx_0);
      for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i < 6;
           dynsim_2link_planar_B.i_i++) {
        dynsim_2link_planar_B.a_idx_1 = 0.0;
        for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k < 6;
             dynsim_2link_planar_B.b_k++) {
          dynsim_2link_planar_B.a_idx_1 += vB->data[(dynsim_2link_planar_B.m - 1)
            * 6 + dynsim_2link_planar_B.b_k] * X->
            data[dynsim_2link_planar_B.unnamed_idx_1].f1[6 *
            dynsim_2link_planar_B.b_k + dynsim_2link_planar_B.i_i];
        }

        dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.i_i] = vJ->data[6 *
          dynsim_2link_planar_B.unnamed_idx_1 + dynsim_2link_planar_B.i_i] +
          dynsim_2link_planar_B.a_idx_1;
      }

      for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i < 6;
           dynsim_2link_planar_B.i_i++) {
        vB->data[dynsim_2link_planar_B.i_i + 6 *
          dynsim_2link_planar_B.unnamed_idx_1] =
          dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.i_i];
      }

      dynsim_2link_planar_B.inner = S->size[1] - 1;
      for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k < 6;
           dynsim_2link_planar_B.b_k++) {
        dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.b_k] = 0.0;
      }

      for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k <=
           dynsim_2link_planar_B.inner; dynsim_2link_planar_B.b_k++) {
        dynsim_2link_planar_B.aoffset = dynsim_2link_planar_B.b_k * 6 - 1;
        for (dynsim_2link_planar_B.q_size_tmp = 0;
             dynsim_2link_planar_B.q_size_tmp < 6;
             dynsim_2link_planar_B.q_size_tmp++) {
          dynsim_2link_planar_B.a_idx_1 = S->data[(dynsim_2link_planar_B.aoffset
            + dynsim_2link_planar_B.q_size_tmp) + 1] * 0.0 +
            dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.q_size_tmp];
          dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.q_size_tmp] =
            dynsim_2link_planar_B.a_idx_1;
        }
      }

      dynsim_2link_planar_B.R_dy[0] = 0.0;
      dynsim_2link_planar_B.b_k = 6 * dynsim_2link_planar_B.unnamed_idx_1 + 2;
      dynsim_2link_planar_B.R_dy[3] = -vB->data[dynsim_2link_planar_B.b_k];
      dynsim_2link_planar_B.i_i = 6 * dynsim_2link_planar_B.unnamed_idx_1 + 1;
      dynsim_2link_planar_B.R_dy[6] = vB->data[dynsim_2link_planar_B.i_i];
      dynsim_2link_planar_B.R_dy[1] = vB->data[dynsim_2link_planar_B.b_k];
      dynsim_2link_planar_B.R_dy[4] = 0.0;
      dynsim_2link_planar_B.R_dy[7] = -vB->data[6 *
        dynsim_2link_planar_B.unnamed_idx_1];
      dynsim_2link_planar_B.R_dy[2] = -vB->data[dynsim_2link_planar_B.i_i];
      dynsim_2link_planar_B.R_dy[5] = vB->data[6 *
        dynsim_2link_planar_B.unnamed_idx_1];
      dynsim_2link_planar_B.R_dy[8] = 0.0;
      dynsim_2link_planar_B.R[3] = 0.0;
      dynsim_2link_planar_B.b_k = 6 * dynsim_2link_planar_B.unnamed_idx_1 + 5;
      dynsim_2link_planar_B.R[9] = -vB->data[dynsim_2link_planar_B.b_k];
      dynsim_2link_planar_B.i_i = 6 * dynsim_2link_planar_B.unnamed_idx_1 + 4;
      dynsim_2link_planar_B.R[15] = vB->data[dynsim_2link_planar_B.i_i];
      dynsim_2link_planar_B.R[4] = vB->data[dynsim_2link_planar_B.b_k];
      dynsim_2link_planar_B.R[10] = 0.0;
      dynsim_2link_planar_B.b_k = 6 * dynsim_2link_planar_B.unnamed_idx_1 + 3;
      dynsim_2link_planar_B.R[16] = -vB->data[dynsim_2link_planar_B.b_k];
      dynsim_2link_planar_B.R[5] = -vB->data[dynsim_2link_planar_B.i_i];
      dynsim_2link_planar_B.R[11] = vB->data[dynsim_2link_planar_B.b_k];
      dynsim_2link_planar_B.R[17] = 0.0;
      for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i < 3;
           dynsim_2link_planar_B.i_i++) {
        dynsim_2link_planar_B.a_idx_1 = dynsim_2link_planar_B.R_dy[3 *
          dynsim_2link_planar_B.i_i];
        dynsim_2link_planar_B.R[6 * dynsim_2link_planar_B.i_i] =
          dynsim_2link_planar_B.a_idx_1;
        dynsim_2link_planar_B.b_k = 6 * (dynsim_2link_planar_B.i_i + 3);
        dynsim_2link_planar_B.R[dynsim_2link_planar_B.b_k] = 0.0;
        dynsim_2link_planar_B.R[dynsim_2link_planar_B.b_k + 3] =
          dynsim_2link_planar_B.a_idx_1;
        dynsim_2link_planar_B.a_idx_1 = dynsim_2link_planar_B.R_dy[3 *
          dynsim_2link_planar_B.i_i + 1];
        dynsim_2link_planar_B.R[6 * dynsim_2link_planar_B.i_i + 1] =
          dynsim_2link_planar_B.a_idx_1;
        dynsim_2link_planar_B.R[dynsim_2link_planar_B.b_k + 1] = 0.0;
        dynsim_2link_planar_B.R[dynsim_2link_planar_B.b_k + 4] =
          dynsim_2link_planar_B.a_idx_1;
        dynsim_2link_planar_B.a_idx_1 = dynsim_2link_planar_B.R_dy[3 *
          dynsim_2link_planar_B.i_i + 2];
        dynsim_2link_planar_B.R[6 * dynsim_2link_planar_B.i_i + 2] =
          dynsim_2link_planar_B.a_idx_1;
        dynsim_2link_planar_B.R[dynsim_2link_planar_B.b_k + 2] = 0.0;
        dynsim_2link_planar_B.R[dynsim_2link_planar_B.b_k + 5] =
          dynsim_2link_planar_B.a_idx_1;
      }

      for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i < 6;
           dynsim_2link_planar_B.i_i++) {
        dynsim_2link_planar_B.a_idx_1 = 0.0;
        for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k < 6;
             dynsim_2link_planar_B.b_k++) {
          dynsim_2link_planar_B.a_idx_1 += aB->data[(dynsim_2link_planar_B.m - 1)
            * 6 + dynsim_2link_planar_B.b_k] * X->
            data[dynsim_2link_planar_B.unnamed_idx_1].f1[6 *
            dynsim_2link_planar_B.b_k + dynsim_2link_planar_B.i_i];
        }

        dynsim_2link_planar_B.X_e[dynsim_2link_planar_B.i_i] =
          dynsim_2link_planar_B.a_idx_1 +
          dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.i_i];
      }

      for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i < 6;
           dynsim_2link_planar_B.i_i++) {
        dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.i_i] = 0.0;
        for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k < 6;
             dynsim_2link_planar_B.b_k++) {
          dynsim_2link_planar_B.a_idx_1 = dynsim_2link_planar_B.R[6 *
            dynsim_2link_planar_B.b_k + dynsim_2link_planar_B.i_i] * vJ->data[6 *
            dynsim_2link_planar_B.unnamed_idx_1 + dynsim_2link_planar_B.b_k] +
            dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.i_i];
          dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.i_i] =
            dynsim_2link_planar_B.a_idx_1;
        }

        aB->data[dynsim_2link_planar_B.i_i + 6 *
          dynsim_2link_planar_B.unnamed_idx_1] =
          dynsim_2link_planar_B.X_e[dynsim_2link_planar_B.i_i] +
          dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.i_i];
      }

      dynsim_2link_planar_B.R_dy[0] = 0.0;
      dynsim_2link_planar_B.R_dy[3] = -dynsim_2link_planar_B.T_c[14];
      dynsim_2link_planar_B.R_dy[6] = dynsim_2link_planar_B.T_c[13];
      dynsim_2link_planar_B.R_dy[1] = dynsim_2link_planar_B.T_c[14];
      dynsim_2link_planar_B.R_dy[4] = 0.0;
      dynsim_2link_planar_B.R_dy[7] = -dynsim_2link_planar_B.T_c[12];
      dynsim_2link_planar_B.R_dy[2] = -dynsim_2link_planar_B.T_c[13];
      dynsim_2link_planar_B.R_dy[5] = dynsim_2link_planar_B.T_c[12];
      dynsim_2link_planar_B.R_dy[8] = 0.0;
      for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i < 3;
           dynsim_2link_planar_B.i_i++) {
        for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k < 3;
             dynsim_2link_planar_B.b_k++) {
          dynsim_2link_planar_B.q_size_tmp = dynsim_2link_planar_B.i_i + 3 *
            dynsim_2link_planar_B.b_k;
          dynsim_2link_planar_B.R_lx[dynsim_2link_planar_B.q_size_tmp] = 0.0;
          dynsim_2link_planar_B.p_f = dynsim_2link_planar_B.b_k << 2;
          dynsim_2link_planar_B.R_lx[dynsim_2link_planar_B.q_size_tmp] +=
            dynsim_2link_planar_B.T_c[dynsim_2link_planar_B.p_f] *
            dynsim_2link_planar_B.R_dy[dynsim_2link_planar_B.i_i];
          dynsim_2link_planar_B.R_lx[dynsim_2link_planar_B.q_size_tmp] +=
            dynsim_2link_planar_B.T_c[dynsim_2link_planar_B.p_f + 1] *
            dynsim_2link_planar_B.R_dy[dynsim_2link_planar_B.i_i + 3];
          dynsim_2link_planar_B.R_lx[dynsim_2link_planar_B.q_size_tmp] +=
            dynsim_2link_planar_B.T_c[dynsim_2link_planar_B.p_f + 2] *
            dynsim_2link_planar_B.R_dy[dynsim_2link_planar_B.i_i + 6];
          dynsim_2link_planar_B.R[dynsim_2link_planar_B.b_k + 6 *
            dynsim_2link_planar_B.i_i] = dynsim_2link_planar_B.T_c
            [(dynsim_2link_planar_B.i_i << 2) + dynsim_2link_planar_B.b_k];
          dynsim_2link_planar_B.R[dynsim_2link_planar_B.b_k + 6 *
            (dynsim_2link_planar_B.i_i + 3)] = 0.0;
        }
      }

      for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i < 3;
           dynsim_2link_planar_B.i_i++) {
        dynsim_2link_planar_B.R[6 * dynsim_2link_planar_B.i_i + 3] =
          dynsim_2link_planar_B.R_lx[3 * dynsim_2link_planar_B.i_i];
        dynsim_2link_planar_B.b_k = dynsim_2link_planar_B.i_i << 2;
        dynsim_2link_planar_B.q_size_tmp = 6 * (dynsim_2link_planar_B.i_i + 3);
        dynsim_2link_planar_B.R[dynsim_2link_planar_B.q_size_tmp + 3] =
          dynsim_2link_planar_B.T_c[dynsim_2link_planar_B.b_k];
        dynsim_2link_planar_B.R[6 * dynsim_2link_planar_B.i_i + 4] =
          dynsim_2link_planar_B.R_lx[3 * dynsim_2link_planar_B.i_i + 1];
        dynsim_2link_planar_B.R[dynsim_2link_planar_B.q_size_tmp + 4] =
          dynsim_2link_planar_B.T_c[dynsim_2link_planar_B.b_k + 1];
        dynsim_2link_planar_B.R[6 * dynsim_2link_planar_B.i_i + 5] =
          dynsim_2link_planar_B.R_lx[3 * dynsim_2link_planar_B.i_i + 2];
        dynsim_2link_planar_B.R[dynsim_2link_planar_B.q_size_tmp + 5] =
          dynsim_2link_planar_B.T_c[dynsim_2link_planar_B.b_k + 2];
      }

      for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i < 6;
           dynsim_2link_planar_B.i_i++) {
        for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k < 6;
             dynsim_2link_planar_B.b_k++) {
          dynsim_2link_planar_B.p_f = dynsim_2link_planar_B.i_i + 6 *
            dynsim_2link_planar_B.b_k;
          dynsim_2link_planar_B.b_I[dynsim_2link_planar_B.p_f] = 0.0;
          for (dynsim_2link_planar_B.q_size_tmp = 0;
               dynsim_2link_planar_B.q_size_tmp < 6;
               dynsim_2link_planar_B.q_size_tmp++) {
            dynsim_2link_planar_B.b_I[dynsim_2link_planar_B.p_f] += Xtree->data[
              static_cast<int32_T>(dynsim_2link_planar_B.a_idx_0) - 1].f1[6 *
              dynsim_2link_planar_B.q_size_tmp + dynsim_2link_planar_B.i_i] *
              dynsim_2link_planar_B.R[6 * dynsim_2link_planar_B.b_k +
              dynsim_2link_planar_B.q_size_tmp];
          }
        }
      }

      for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i < 36;
           dynsim_2link_planar_B.i_i++) {
        Xtree->data[dynsim_2link_planar_B.unnamed_idx_1]
          .f1[dynsim_2link_planar_B.i_i] =
          dynsim_2link_planar_B.b_I[dynsim_2link_planar_B.i_i];
      }
    } else {
      dynsim_2link_planar_B.inner = S->size[1] - 1;
      for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k < 6;
           dynsim_2link_planar_B.b_k++) {
        dynsim_2link_planar_B.i_i = 6 * dynsim_2link_planar_B.unnamed_idx_1 +
          dynsim_2link_planar_B.b_k;
        vB->data[dynsim_2link_planar_B.i_i] = vJ->data[dynsim_2link_planar_B.i_i];
        dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.b_k] = 0.0;
      }

      for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k <=
           dynsim_2link_planar_B.inner; dynsim_2link_planar_B.b_k++) {
        dynsim_2link_planar_B.aoffset = dynsim_2link_planar_B.b_k * 6 - 1;
        for (dynsim_2link_planar_B.q_size_tmp = 0;
             dynsim_2link_planar_B.q_size_tmp < 6;
             dynsim_2link_planar_B.q_size_tmp++) {
          dynsim_2link_planar_B.a_idx_1 = S->data[(dynsim_2link_planar_B.aoffset
            + dynsim_2link_planar_B.q_size_tmp) + 1] * 0.0 +
            dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.q_size_tmp];
          dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.q_size_tmp] =
            dynsim_2link_planar_B.a_idx_1;
        }
      }

      for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i < 6;
           dynsim_2link_planar_B.i_i++) {
        dynsim_2link_planar_B.a_idx_1 = 0.0;
        for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k < 6;
             dynsim_2link_planar_B.b_k++) {
          dynsim_2link_planar_B.a_idx_1 += X->
            data[dynsim_2link_planar_B.unnamed_idx_1].f1[6 *
            dynsim_2link_planar_B.b_k + dynsim_2link_planar_B.i_i] *
            dynsim_2link_planar_B.a0[dynsim_2link_planar_B.b_k];
        }

        aB->data[dynsim_2link_planar_B.i_i + 6 *
          dynsim_2link_planar_B.unnamed_idx_1] = dynsim_2link_planar_B.a_idx_1 +
          dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.i_i];
      }

      dynsim_2link_planar_B.R_dy[0] = 0.0;
      dynsim_2link_planar_B.R_dy[3] = -dynsim_2link_planar_B.T_c[14];
      dynsim_2link_planar_B.R_dy[6] = dynsim_2link_planar_B.T_c[13];
      dynsim_2link_planar_B.R_dy[1] = dynsim_2link_planar_B.T_c[14];
      dynsim_2link_planar_B.R_dy[4] = 0.0;
      dynsim_2link_planar_B.R_dy[7] = -dynsim_2link_planar_B.T_c[12];
      dynsim_2link_planar_B.R_dy[2] = -dynsim_2link_planar_B.T_c[13];
      dynsim_2link_planar_B.R_dy[5] = dynsim_2link_planar_B.T_c[12];
      dynsim_2link_planar_B.R_dy[8] = 0.0;
      for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i < 3;
           dynsim_2link_planar_B.i_i++) {
        for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k < 3;
             dynsim_2link_planar_B.b_k++) {
          dynsim_2link_planar_B.q_size_tmp = dynsim_2link_planar_B.i_i + 3 *
            dynsim_2link_planar_B.b_k;
          dynsim_2link_planar_B.R_lx[dynsim_2link_planar_B.q_size_tmp] = 0.0;
          dynsim_2link_planar_B.p_f = dynsim_2link_planar_B.b_k << 2;
          dynsim_2link_planar_B.R_lx[dynsim_2link_planar_B.q_size_tmp] +=
            dynsim_2link_planar_B.T_c[dynsim_2link_planar_B.p_f] *
            dynsim_2link_planar_B.R_dy[dynsim_2link_planar_B.i_i];
          dynsim_2link_planar_B.R_lx[dynsim_2link_planar_B.q_size_tmp] +=
            dynsim_2link_planar_B.T_c[dynsim_2link_planar_B.p_f + 1] *
            dynsim_2link_planar_B.R_dy[dynsim_2link_planar_B.i_i + 3];
          dynsim_2link_planar_B.R_lx[dynsim_2link_planar_B.q_size_tmp] +=
            dynsim_2link_planar_B.T_c[dynsim_2link_planar_B.p_f + 2] *
            dynsim_2link_planar_B.R_dy[dynsim_2link_planar_B.i_i + 6];
          Xtree->data[dynsim_2link_planar_B.unnamed_idx_1]
            .f1[dynsim_2link_planar_B.b_k + 6 * dynsim_2link_planar_B.i_i] =
            dynsim_2link_planar_B.T_c[(dynsim_2link_planar_B.i_i << 2) +
            dynsim_2link_planar_B.b_k];
          Xtree->data[dynsim_2link_planar_B.unnamed_idx_1]
            .f1[dynsim_2link_planar_B.b_k + 6 * (dynsim_2link_planar_B.i_i + 3)]
            = 0.0;
        }
      }

      for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i < 3;
           dynsim_2link_planar_B.i_i++) {
        Xtree->data[dynsim_2link_planar_B.unnamed_idx_1].f1[6 *
          dynsim_2link_planar_B.i_i + 3] = dynsim_2link_planar_B.R_lx[3 *
          dynsim_2link_planar_B.i_i];
        dynsim_2link_planar_B.b_k = dynsim_2link_planar_B.i_i << 2;
        dynsim_2link_planar_B.q_size_tmp = 6 * (dynsim_2link_planar_B.i_i + 3);
        Xtree->data[dynsim_2link_planar_B.unnamed_idx_1]
          .f1[dynsim_2link_planar_B.q_size_tmp + 3] =
          dynsim_2link_planar_B.T_c[dynsim_2link_planar_B.b_k];
        Xtree->data[dynsim_2link_planar_B.unnamed_idx_1].f1[6 *
          dynsim_2link_planar_B.i_i + 4] = dynsim_2link_planar_B.R_lx[3 *
          dynsim_2link_planar_B.i_i + 1];
        Xtree->data[dynsim_2link_planar_B.unnamed_idx_1]
          .f1[dynsim_2link_planar_B.q_size_tmp + 4] =
          dynsim_2link_planar_B.T_c[dynsim_2link_planar_B.b_k + 1];
        Xtree->data[dynsim_2link_planar_B.unnamed_idx_1].f1[6 *
          dynsim_2link_planar_B.i_i + 5] = dynsim_2link_planar_B.R_lx[3 *
          dynsim_2link_planar_B.i_i + 2];
        Xtree->data[dynsim_2link_planar_B.unnamed_idx_1]
          .f1[dynsim_2link_planar_B.q_size_tmp + 5] =
          dynsim_2link_planar_B.T_c[dynsim_2link_planar_B.b_k + 2];
      }
    }

    for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i < 36;
         dynsim_2link_planar_B.i_i++) {
      dynsim_2link_planar_B.b_I[dynsim_2link_planar_B.i_i] = robot->
        Bodies[dynsim_2link_planar_B.unnamed_idx_1]->
        SpatialInertia[dynsim_2link_planar_B.i_i];
    }

    dynsim_2link_planar_B.R_dy[0] = 0.0;
    dynsim_2link_planar_B.b_k = 6 * dynsim_2link_planar_B.unnamed_idx_1 + 2;
    dynsim_2link_planar_B.R_dy[3] = -vB->data[dynsim_2link_planar_B.b_k];
    dynsim_2link_planar_B.i_i = 6 * dynsim_2link_planar_B.unnamed_idx_1 + 1;
    dynsim_2link_planar_B.R_dy[6] = vB->data[dynsim_2link_planar_B.i_i];
    dynsim_2link_planar_B.R_dy[1] = vB->data[dynsim_2link_planar_B.b_k];
    dynsim_2link_planar_B.R_dy[4] = 0.0;
    dynsim_2link_planar_B.R_dy[7] = -vB->data[6 *
      dynsim_2link_planar_B.unnamed_idx_1];
    dynsim_2link_planar_B.R_dy[2] = -vB->data[dynsim_2link_planar_B.i_i];
    dynsim_2link_planar_B.R_dy[5] = vB->data[6 *
      dynsim_2link_planar_B.unnamed_idx_1];
    dynsim_2link_planar_B.R_dy[8] = 0.0;
    dynsim_2link_planar_B.R[18] = 0.0;
    dynsim_2link_planar_B.b_k = 6 * dynsim_2link_planar_B.unnamed_idx_1 + 5;
    dynsim_2link_planar_B.R[24] = -vB->data[dynsim_2link_planar_B.b_k];
    dynsim_2link_planar_B.i_i = 6 * dynsim_2link_planar_B.unnamed_idx_1 + 4;
    dynsim_2link_planar_B.R[30] = vB->data[dynsim_2link_planar_B.i_i];
    dynsim_2link_planar_B.R[19] = vB->data[dynsim_2link_planar_B.b_k];
    dynsim_2link_planar_B.R[25] = 0.0;
    dynsim_2link_planar_B.b_k = 6 * dynsim_2link_planar_B.unnamed_idx_1 + 3;
    dynsim_2link_planar_B.R[31] = -vB->data[dynsim_2link_planar_B.b_k];
    dynsim_2link_planar_B.R[20] = -vB->data[dynsim_2link_planar_B.i_i];
    dynsim_2link_planar_B.R[26] = vB->data[dynsim_2link_planar_B.b_k];
    dynsim_2link_planar_B.R[32] = 0.0;
    for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i < 3;
         dynsim_2link_planar_B.i_i++) {
      dynsim_2link_planar_B.a_idx_1 = dynsim_2link_planar_B.R_dy[3 *
        dynsim_2link_planar_B.i_i];
      dynsim_2link_planar_B.R[6 * dynsim_2link_planar_B.i_i] =
        dynsim_2link_planar_B.a_idx_1;
      dynsim_2link_planar_B.R[6 * dynsim_2link_planar_B.i_i + 3] = 0.0;
      dynsim_2link_planar_B.b_k = 6 * (dynsim_2link_planar_B.i_i + 3);
      dynsim_2link_planar_B.R[dynsim_2link_planar_B.b_k + 3] =
        dynsim_2link_planar_B.a_idx_1;
      dynsim_2link_planar_B.a_idx_1 = dynsim_2link_planar_B.R_dy[3 *
        dynsim_2link_planar_B.i_i + 1];
      dynsim_2link_planar_B.R[6 * dynsim_2link_planar_B.i_i + 1] =
        dynsim_2link_planar_B.a_idx_1;
      dynsim_2link_planar_B.R[6 * dynsim_2link_planar_B.i_i + 4] = 0.0;
      dynsim_2link_planar_B.R[dynsim_2link_planar_B.b_k + 4] =
        dynsim_2link_planar_B.a_idx_1;
      dynsim_2link_planar_B.a_idx_1 = dynsim_2link_planar_B.R_dy[3 *
        dynsim_2link_planar_B.i_i + 2];
      dynsim_2link_planar_B.R[6 * dynsim_2link_planar_B.i_i + 2] =
        dynsim_2link_planar_B.a_idx_1;
      dynsim_2link_planar_B.R[6 * dynsim_2link_planar_B.i_i + 5] = 0.0;
      dynsim_2link_planar_B.R[dynsim_2link_planar_B.b_k + 5] =
        dynsim_2link_planar_B.a_idx_1;
    }

    for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i < 6;
         dynsim_2link_planar_B.i_i++) {
      dynsim_2link_planar_B.X_e[dynsim_2link_planar_B.i_i] = 0.0;
      dynsim_2link_planar_B.b_I_b[dynsim_2link_planar_B.i_i] = 0.0;
      for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k < 6;
           dynsim_2link_planar_B.b_k++) {
        dynsim_2link_planar_B.a_idx_0 = dynsim_2link_planar_B.b_I[6 *
          dynsim_2link_planar_B.b_k + dynsim_2link_planar_B.i_i];
        dynsim_2link_planar_B.q_size_tmp = 6 *
          dynsim_2link_planar_B.unnamed_idx_1 + dynsim_2link_planar_B.b_k;
        dynsim_2link_planar_B.a_idx_1 = vB->
          data[dynsim_2link_planar_B.q_size_tmp] * dynsim_2link_planar_B.a_idx_0
          + dynsim_2link_planar_B.X_e[dynsim_2link_planar_B.i_i];
        dynsim_2link_planar_B.a_idx_0 = aB->
          data[dynsim_2link_planar_B.q_size_tmp] * dynsim_2link_planar_B.a_idx_0
          + dynsim_2link_planar_B.b_I_b[dynsim_2link_planar_B.i_i];
        dynsim_2link_planar_B.X_e[dynsim_2link_planar_B.i_i] =
          dynsim_2link_planar_B.a_idx_1;
        dynsim_2link_planar_B.b_I_b[dynsim_2link_planar_B.i_i] =
          dynsim_2link_planar_B.a_idx_0;
      }
    }

    for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i < 6;
         dynsim_2link_planar_B.i_i++) {
      dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.i_i] = 0.0;
      dynsim_2link_planar_B.a_idx_1 = 0.0;
      for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k < 6;
           dynsim_2link_planar_B.b_k++) {
        dynsim_2link_planar_B.a_idx_1 += Xtree->
          data[dynsim_2link_planar_B.unnamed_idx_1].f1[6 *
          dynsim_2link_planar_B.i_i + dynsim_2link_planar_B.b_k] * fext[6 *
          dynsim_2link_planar_B.unnamed_idx_1 + dynsim_2link_planar_B.b_k];
        dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.i_i] +=
          dynsim_2link_planar_B.R[6 * dynsim_2link_planar_B.b_k +
          dynsim_2link_planar_B.i_i] *
          dynsim_2link_planar_B.X_e[dynsim_2link_planar_B.b_k];
      }

      f->data[dynsim_2link_planar_B.i_i + 6 *
        dynsim_2link_planar_B.unnamed_idx_1] =
        (dynsim_2link_planar_B.b_I_b[dynsim_2link_planar_B.i_i] +
         dynsim_2link_planar_B.q_data[dynsim_2link_planar_B.i_i]) -
        dynsim_2link_planar_B.a_idx_1;
    }
  }

  dynsim_2link_pla_emxFree_real_T(&aB);
  dynsim_2link_pla_emxFree_real_T(&vB);
  dynsim_2link_pla_emxFree_real_T(&vJ);
  dynsim_2li_emxFree_f_cell_wrap1(&Xtree);
  dynsim_2link_planar_B.q_size_tmp = static_cast<int32_T>(((-1.0 -
    dynsim_2link_planar_B.nb) + 1.0) / -1.0) - 1;
  dynsim_2link_pla_emxInit_real_T(&taui, 1);
  dynsim_2link_pla_emxInit_char_T(&a, 2);
  if (0 <= dynsim_2link_planar_B.q_size_tmp) {
    for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i < 5;
         dynsim_2link_planar_B.i_i++) {
      dynsim_2link_planar_B.b_g[dynsim_2link_planar_B.i_i] =
        tmp[dynsim_2link_planar_B.i_i];
    }
  }

  for (dynsim_2link_planar_B.loop_ub_tmp = 0; dynsim_2link_planar_B.loop_ub_tmp <=
       dynsim_2link_planar_B.q_size_tmp; dynsim_2link_planar_B.loop_ub_tmp++) {
    dynsim_2link_planar_B.a_idx_0 = dynsim_2link_planar_B.nb +
      -static_cast<real_T>(dynsim_2link_planar_B.loop_ub_tmp);
    dynsim_2link_planar_B.p_f = static_cast<int32_T>
      (dynsim_2link_planar_B.a_idx_0);
    dynsim_2link_planar_B.inner = dynsim_2link_planar_B.p_f - 1;
    obj = robot->Bodies[dynsim_2link_planar_B.inner];
    dynsim_2link_planar_B.i_i = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = obj->JointInternal.Type->size[1];
    dynsim_emxEnsureCapacity_char_T(a, dynsim_2link_planar_B.i_i);
    dynsim_2link_planar_B.b_k = obj->JointInternal.Type->size[0] *
      obj->JointInternal.Type->size[1] - 1;
    for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i <=
         dynsim_2link_planar_B.b_k; dynsim_2link_planar_B.i_i++) {
      a->data[dynsim_2link_planar_B.i_i] = obj->JointInternal.Type->
        data[dynsim_2link_planar_B.i_i];
    }

    dynsim_2link_planar_B.b_bool_i = false;
    if (a->size[1] == 5) {
      dynsim_2link_planar_B.i_i = 1;
      do {
        exitg1 = 0;
        if (dynsim_2link_planar_B.i_i - 1 < 5) {
          dynsim_2link_planar_B.unnamed_idx_1 = dynsim_2link_planar_B.i_i - 1;
          if (a->data[dynsim_2link_planar_B.unnamed_idx_1] !=
              dynsim_2link_planar_B.b_g[dynsim_2link_planar_B.unnamed_idx_1]) {
            exitg1 = 1;
          } else {
            dynsim_2link_planar_B.i_i++;
          }
        } else {
          dynsim_2link_planar_B.b_bool_i = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (!dynsim_2link_planar_B.b_bool_i) {
      obj = robot->Bodies[dynsim_2link_planar_B.inner];
      dynsim_2link_planar_B.i_i = S->size[0] * S->size[1];
      S->size[0] = 6;
      S->size[1] = obj->JointInternal.MotionSubspace->size[1];
      dynsim_emxEnsureCapacity_real_T(S, dynsim_2link_planar_B.i_i);
      dynsim_2link_planar_B.b_k = obj->JointInternal.MotionSubspace->size[0] *
        obj->JointInternal.MotionSubspace->size[1] - 1;
      for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i <=
           dynsim_2link_planar_B.b_k; dynsim_2link_planar_B.i_i++) {
        S->data[dynsim_2link_planar_B.i_i] = obj->
          JointInternal.MotionSubspace->data[dynsim_2link_planar_B.i_i];
      }

      dynsim_2link_planar_B.m = S->size[1] - 1;
      dynsim_2link_planar_B.i_i = taui->size[0];
      taui->size[0] = S->size[1];
      dynsim_emxEnsureCapacity_real_T(taui, dynsim_2link_planar_B.i_i);
      for (dynsim_2link_planar_B.unnamed_idx_1 = 0;
           dynsim_2link_planar_B.unnamed_idx_1 <= dynsim_2link_planar_B.m;
           dynsim_2link_planar_B.unnamed_idx_1++) {
        dynsim_2link_planar_B.aoffset = dynsim_2link_planar_B.unnamed_idx_1 * 6
          - 1;
        dynsim_2link_planar_B.a_idx_1 = 0.0;
        for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k < 6;
             dynsim_2link_planar_B.b_k++) {
          dynsim_2link_planar_B.a_idx_1 += f->data[(static_cast<int32_T>
            (dynsim_2link_planar_B.a_idx_0) - 1) * 6 + dynsim_2link_planar_B.b_k]
            * S->data[(dynsim_2link_planar_B.aoffset + dynsim_2link_planar_B.b_k)
            + 1];
        }

        taui->data[dynsim_2link_planar_B.unnamed_idx_1] =
          dynsim_2link_planar_B.a_idx_1;
      }

      dynsim_2link_planar_B.b_idx_0 = robot->
        VelocityDoFMap[dynsim_2link_planar_B.p_f - 1];
      dynsim_2link_planar_B.b_idx_1 = robot->
        VelocityDoFMap[dynsim_2link_planar_B.p_f + 7];
      if (dynsim_2link_planar_B.b_idx_0 > dynsim_2link_planar_B.b_idx_1) {
        dynsim_2link_planar_B.b_k = 0;
        dynsim_2link_planar_B.i_i = 0;
      } else {
        dynsim_2link_planar_B.b_k = static_cast<int32_T>
          (dynsim_2link_planar_B.b_idx_0) - 1;
        dynsim_2link_planar_B.i_i = static_cast<int32_T>
          (dynsim_2link_planar_B.b_idx_1);
      }

      dynsim_2link_planar_B.unnamed_idx_1 = dynsim_2link_planar_B.i_i -
        dynsim_2link_planar_B.b_k;
      for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i <
           dynsim_2link_planar_B.unnamed_idx_1; dynsim_2link_planar_B.i_i++) {
        tau[dynsim_2link_planar_B.b_k + dynsim_2link_planar_B.i_i] = taui->
          data[dynsim_2link_planar_B.i_i];
      }
    }

    dynsim_2link_planar_B.a_idx_0 = robot->Bodies[dynsim_2link_planar_B.inner]
      ->ParentIndex;
    if (dynsim_2link_planar_B.a_idx_0 > 0.0) {
      dynsim_2link_planar_B.m = static_cast<int32_T>
        (dynsim_2link_planar_B.a_idx_0);
      for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i < 6;
           dynsim_2link_planar_B.i_i++) {
        dynsim_2link_planar_B.a_idx_1 = 0.0;
        for (dynsim_2link_planar_B.b_k = 0; dynsim_2link_planar_B.b_k < 6;
             dynsim_2link_planar_B.b_k++) {
          dynsim_2link_planar_B.a_idx_1 += f->data[(dynsim_2link_planar_B.p_f -
            1) * 6 + dynsim_2link_planar_B.b_k] * X->
            data[dynsim_2link_planar_B.inner].f1[6 * dynsim_2link_planar_B.i_i +
            dynsim_2link_planar_B.b_k];
        }

        dynsim_2link_planar_B.a0[dynsim_2link_planar_B.i_i] = f->data
          [(dynsim_2link_planar_B.m - 1) * 6 + dynsim_2link_planar_B.i_i] +
          dynsim_2link_planar_B.a_idx_1;
      }

      for (dynsim_2link_planar_B.i_i = 0; dynsim_2link_planar_B.i_i < 6;
           dynsim_2link_planar_B.i_i++) {
        f->data[dynsim_2link_planar_B.i_i + 6 * (dynsim_2link_planar_B.m - 1)] =
          dynsim_2link_planar_B.a0[dynsim_2link_planar_B.i_i];
      }
    }
  }

  dynsim_2link_pla_emxFree_char_T(&a);
  dynsim_2link_pla_emxFree_real_T(&taui);
  dynsim_2link_pla_emxFree_real_T(&S);
  dynsim_2link_pla_emxFree_real_T(&f);
  dynsim_2li_emxFree_f_cell_wrap1(&X);
}

// Function for MATLAB Function: '<S4>/MATLAB Function'
static boolean_T dynsim_2link_plana_anyNonFinite(const real_T x[16])
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

static real_T dynsim_2link_plan_rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T y;
  dynsim_2link_planar_B.a_h = fabs(u0);
  y = fabs(u1);
  if (dynsim_2link_planar_B.a_h < y) {
    dynsim_2link_planar_B.a_h /= y;
    y *= sqrt(dynsim_2link_planar_B.a_h * dynsim_2link_planar_B.a_h + 1.0);
  } else if (dynsim_2link_planar_B.a_h > y) {
    y /= dynsim_2link_planar_B.a_h;
    y = sqrt(y * y + 1.0) * dynsim_2link_planar_B.a_h;
  } else {
    if (!rtIsNaN(y)) {
      y = dynsim_2link_planar_B.a_h * 1.4142135623730951;
    }
  }

  return y;
}

// Function for MATLAB Function: '<S4>/MATLAB Function'
static real_T dynsim_2link_planar_xzlangeM(const creal_T x[16])
{
  real_T y;
  real_T absxk;
  int32_T k;
  boolean_T exitg1;
  y = 0.0;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 16)) {
    absxk = dynsim_2link_plan_rt_hypotd_snf(x[k].re, x[k].im);
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
static void dynsim_2link_planar_xzlascl(real_T cfrom, real_T cto, creal_T A[16])
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
static real_T dynsim_2link_planar_xzlanhs(const creal_T A[16], int32_T ilo,
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
static void dynsim_2link_planar_xzlartg_d(const creal_T f, const creal_T g,
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
        f2s = dynsim_2link_plan_rt_hypotd_snf(gs_re, gs_im);
        sn->re = gs_re / f2s;
        sn->im = -gs_im / f2s;
      } else {
        scale = sqrt(g2);
        *cs = dynsim_2link_plan_rt_hypotd_snf(fs_re, fs_im) / scale;
        if (di > f2s) {
          f2s = di;
        }

        if (f2s > 1.0) {
          f2s = dynsim_2link_plan_rt_hypotd_snf(f.re, f.im);
          fs_re = f.re / f2s;
          fs_im = f.im / f2s;
        } else {
          fs_re = 7.4428285367870146E+137 * f.re;
          di = 7.4428285367870146E+137 * f.im;
          f2s = dynsim_2link_plan_rt_hypotd_snf(fs_re, di);
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

// Function for MATLAB Function: '<S4>/MATLAB Function'
static void dynsim_2link_planar_xzlartg(const creal_T f, const creal_T g, real_T
  *cs, creal_T *sn, creal_T *r)
{
  boolean_T guard1 = false;
  dynsim_2link_planar_B.f2s = fabs(f.re);
  dynsim_2link_planar_B.di = fabs(f.im);
  dynsim_2link_planar_B.scale_o = dynsim_2link_planar_B.f2s;
  if (dynsim_2link_planar_B.di > dynsim_2link_planar_B.f2s) {
    dynsim_2link_planar_B.scale_o = dynsim_2link_planar_B.di;
  }

  dynsim_2link_planar_B.gs_re = fabs(g.re);
  dynsim_2link_planar_B.gs_im = fabs(g.im);
  if (dynsim_2link_planar_B.gs_im > dynsim_2link_planar_B.gs_re) {
    dynsim_2link_planar_B.gs_re = dynsim_2link_planar_B.gs_im;
  }

  if (dynsim_2link_planar_B.gs_re > dynsim_2link_planar_B.scale_o) {
    dynsim_2link_planar_B.scale_o = dynsim_2link_planar_B.gs_re;
  }

  dynsim_2link_planar_B.fs_re = f.re;
  dynsim_2link_planar_B.fs_im = f.im;
  dynsim_2link_planar_B.gs_re = g.re;
  dynsim_2link_planar_B.gs_im = g.im;
  dynsim_2link_planar_B.count = -1;
  dynsim_2link_planar_B.rescaledir = 0;
  guard1 = false;
  if (dynsim_2link_planar_B.scale_o >= 7.4428285367870146E+137) {
    do {
      dynsim_2link_planar_B.count++;
      dynsim_2link_planar_B.fs_re *= 1.3435752215134178E-138;
      dynsim_2link_planar_B.fs_im *= 1.3435752215134178E-138;
      dynsim_2link_planar_B.gs_re *= 1.3435752215134178E-138;
      dynsim_2link_planar_B.gs_im *= 1.3435752215134178E-138;
      dynsim_2link_planar_B.scale_o *= 1.3435752215134178E-138;
    } while (!(dynsim_2link_planar_B.scale_o < 7.4428285367870146E+137));

    dynsim_2link_planar_B.rescaledir = 1;
    guard1 = true;
  } else if (dynsim_2link_planar_B.scale_o <= 1.3435752215134178E-138) {
    if ((g.re == 0.0) && (g.im == 0.0)) {
      *cs = 1.0;
      sn->re = 0.0;
      sn->im = 0.0;
      *r = f;
    } else {
      do {
        dynsim_2link_planar_B.count++;
        dynsim_2link_planar_B.fs_re *= 7.4428285367870146E+137;
        dynsim_2link_planar_B.fs_im *= 7.4428285367870146E+137;
        dynsim_2link_planar_B.gs_re *= 7.4428285367870146E+137;
        dynsim_2link_planar_B.gs_im *= 7.4428285367870146E+137;
        dynsim_2link_planar_B.scale_o *= 7.4428285367870146E+137;
      } while (!(dynsim_2link_planar_B.scale_o > 1.3435752215134178E-138));

      dynsim_2link_planar_B.rescaledir = -1;
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    dynsim_2link_planar_B.scale_o = dynsim_2link_planar_B.fs_re *
      dynsim_2link_planar_B.fs_re + dynsim_2link_planar_B.fs_im *
      dynsim_2link_planar_B.fs_im;
    dynsim_2link_planar_B.g2 = dynsim_2link_planar_B.gs_re *
      dynsim_2link_planar_B.gs_re + dynsim_2link_planar_B.gs_im *
      dynsim_2link_planar_B.gs_im;
    dynsim_2link_planar_B.x = dynsim_2link_planar_B.g2;
    if (1.0 > dynsim_2link_planar_B.g2) {
      dynsim_2link_planar_B.x = 1.0;
    }

    if (dynsim_2link_planar_B.scale_o <= dynsim_2link_planar_B.x *
        2.0041683600089728E-292) {
      if ((f.re == 0.0) && (f.im == 0.0)) {
        *cs = 0.0;
        r->re = dynsim_2link_plan_rt_hypotd_snf(g.re, g.im);
        r->im = 0.0;
        dynsim_2link_planar_B.f2s = dynsim_2link_plan_rt_hypotd_snf
          (dynsim_2link_planar_B.gs_re, dynsim_2link_planar_B.gs_im);
        sn->re = dynsim_2link_planar_B.gs_re / dynsim_2link_planar_B.f2s;
        sn->im = -dynsim_2link_planar_B.gs_im / dynsim_2link_planar_B.f2s;
      } else {
        dynsim_2link_planar_B.scale_o = sqrt(dynsim_2link_planar_B.g2);
        *cs = dynsim_2link_plan_rt_hypotd_snf(dynsim_2link_planar_B.fs_re,
          dynsim_2link_planar_B.fs_im) / dynsim_2link_planar_B.scale_o;
        if (dynsim_2link_planar_B.di > dynsim_2link_planar_B.f2s) {
          dynsim_2link_planar_B.f2s = dynsim_2link_planar_B.di;
        }

        if (dynsim_2link_planar_B.f2s > 1.0) {
          dynsim_2link_planar_B.f2s = dynsim_2link_plan_rt_hypotd_snf(f.re, f.im);
          dynsim_2link_planar_B.fs_re = f.re / dynsim_2link_planar_B.f2s;
          dynsim_2link_planar_B.fs_im = f.im / dynsim_2link_planar_B.f2s;
        } else {
          dynsim_2link_planar_B.fs_re = 7.4428285367870146E+137 * f.re;
          dynsim_2link_planar_B.di = 7.4428285367870146E+137 * f.im;
          dynsim_2link_planar_B.f2s = dynsim_2link_plan_rt_hypotd_snf
            (dynsim_2link_planar_B.fs_re, dynsim_2link_planar_B.di);
          dynsim_2link_planar_B.fs_re /= dynsim_2link_planar_B.f2s;
          dynsim_2link_planar_B.fs_im = dynsim_2link_planar_B.di /
            dynsim_2link_planar_B.f2s;
        }

        dynsim_2link_planar_B.gs_re /= dynsim_2link_planar_B.scale_o;
        dynsim_2link_planar_B.gs_im = -dynsim_2link_planar_B.gs_im /
          dynsim_2link_planar_B.scale_o;
        sn->re = dynsim_2link_planar_B.fs_re * dynsim_2link_planar_B.gs_re -
          dynsim_2link_planar_B.fs_im * dynsim_2link_planar_B.gs_im;
        sn->im = dynsim_2link_planar_B.fs_re * dynsim_2link_planar_B.gs_im +
          dynsim_2link_planar_B.fs_im * dynsim_2link_planar_B.gs_re;
        r->re = (sn->re * g.re - sn->im * g.im) + *cs * f.re;
        r->im = (sn->re * g.im + sn->im * g.re) + *cs * f.im;
      }
    } else {
      dynsim_2link_planar_B.f2s = sqrt(dynsim_2link_planar_B.g2 /
        dynsim_2link_planar_B.scale_o + 1.0);
      r->re = dynsim_2link_planar_B.f2s * dynsim_2link_planar_B.fs_re;
      r->im = dynsim_2link_planar_B.f2s * dynsim_2link_planar_B.fs_im;
      *cs = 1.0 / dynsim_2link_planar_B.f2s;
      dynsim_2link_planar_B.f2s = dynsim_2link_planar_B.scale_o +
        dynsim_2link_planar_B.g2;
      dynsim_2link_planar_B.fs_re = r->re / dynsim_2link_planar_B.f2s;
      dynsim_2link_planar_B.f2s = r->im / dynsim_2link_planar_B.f2s;
      sn->re = dynsim_2link_planar_B.fs_re * dynsim_2link_planar_B.gs_re -
        dynsim_2link_planar_B.f2s * -dynsim_2link_planar_B.gs_im;
      sn->im = dynsim_2link_planar_B.fs_re * -dynsim_2link_planar_B.gs_im +
        dynsim_2link_planar_B.f2s * dynsim_2link_planar_B.gs_re;
      if (dynsim_2link_planar_B.rescaledir > 0) {
        dynsim_2link_planar_B.rescaledir = 0;
        while (dynsim_2link_planar_B.rescaledir <= dynsim_2link_planar_B.count)
        {
          r->re *= 7.4428285367870146E+137;
          r->im *= 7.4428285367870146E+137;
          dynsim_2link_planar_B.rescaledir++;
        }
      } else {
        if (dynsim_2link_planar_B.rescaledir < 0) {
          dynsim_2link_planar_B.rescaledir = 0;
          while (dynsim_2link_planar_B.rescaledir <= dynsim_2link_planar_B.count)
          {
            r->re *= 1.3435752215134178E-138;
            r->im *= 1.3435752215134178E-138;
            dynsim_2link_planar_B.rescaledir++;
          }
        }
      }
    }
  }
}

// Function for MATLAB Function: '<S4>/MATLAB Function'
static void dynsim_2link_planar_xzhgeqz(creal_T A[16], int32_T ilo, int32_T ihi,
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
  dynsim_2link_planar_B.eshift_re = 0.0;
  dynsim_2link_planar_B.eshift_im = 0.0;
  dynsim_2link_planar_B.ctemp.re = 0.0;
  dynsim_2link_planar_B.ctemp.im = 0.0;
  dynsim_2link_planar_B.anorm = dynsim_2link_planar_xzlanhs(A, ilo, ihi);
  dynsim_2link_planar_B.shift_re = 2.2204460492503131E-16 *
    dynsim_2link_planar_B.anorm;
  dynsim_2link_planar_B.b_atol = 2.2250738585072014E-308;
  if (dynsim_2link_planar_B.shift_re > 2.2250738585072014E-308) {
    dynsim_2link_planar_B.b_atol = dynsim_2link_planar_B.shift_re;
  }

  dynsim_2link_planar_B.shift_re = 2.2250738585072014E-308;
  if (dynsim_2link_planar_B.anorm > 2.2250738585072014E-308) {
    dynsim_2link_planar_B.shift_re = dynsim_2link_planar_B.anorm;
  }

  dynsim_2link_planar_B.anorm = 1.0 / dynsim_2link_planar_B.shift_re;
  dynsim_2link_planar_B.failed = true;
  dynsim_2link_planar_B.ilast = ihi;
  while (dynsim_2link_planar_B.ilast + 1 < 5) {
    alpha1[dynsim_2link_planar_B.ilast] = A[(dynsim_2link_planar_B.ilast << 2) +
      dynsim_2link_planar_B.ilast];
    dynsim_2link_planar_B.ilast++;
  }

  guard1 = false;
  guard2 = false;
  if (ihi >= ilo) {
    dynsim_2link_planar_B.ifirst = ilo;
    dynsim_2link_planar_B.istart = ilo;
    dynsim_2link_planar_B.ilast = ihi - 1;
    dynsim_2link_planar_B.ilastm1 = ihi - 2;
    dynsim_2link_planar_B.iiter = 0;
    dynsim_2link_planar_B.goto60 = false;
    dynsim_2link_planar_B.goto70 = false;
    dynsim_2link_planar_B.goto90 = false;
    dynsim_2link_planar_B.jiter = 0;
    do {
      exitg1 = 0;
      if (dynsim_2link_planar_B.jiter <= ((ihi - ilo) + 1) * 30 - 1) {
        if (dynsim_2link_planar_B.ilast + 1 == ilo) {
          dynsim_2link_planar_B.goto60 = true;
        } else {
          dynsim_2link_planar_B.jp1 = (dynsim_2link_planar_B.ilastm1 << 2) +
            dynsim_2link_planar_B.ilast;
          if (fabs(A[dynsim_2link_planar_B.jp1].re) + fabs
              (A[dynsim_2link_planar_B.jp1].im) <= dynsim_2link_planar_B.b_atol)
          {
            A[dynsim_2link_planar_B.jp1].re = 0.0;
            A[dynsim_2link_planar_B.jp1].im = 0.0;
            dynsim_2link_planar_B.goto60 = true;
          } else {
            dynsim_2link_planar_B.j_m = dynsim_2link_planar_B.ilastm1;
            guard3 = false;
            exitg2 = false;
            while ((!exitg2) && (dynsim_2link_planar_B.j_m + 1 >= ilo)) {
              if (dynsim_2link_planar_B.j_m + 1 == ilo) {
                guard3 = true;
                exitg2 = true;
              } else {
                dynsim_2link_planar_B.jp1 = ((dynsim_2link_planar_B.j_m - 1) <<
                  2) + dynsim_2link_planar_B.j_m;
                if (fabs(A[dynsim_2link_planar_B.jp1].re) + fabs
                    (A[dynsim_2link_planar_B.jp1].im) <=
                    dynsim_2link_planar_B.b_atol) {
                  A[dynsim_2link_planar_B.jp1].re = 0.0;
                  A[dynsim_2link_planar_B.jp1].im = 0.0;
                  guard3 = true;
                  exitg2 = true;
                } else {
                  dynsim_2link_planar_B.j_m--;
                  guard3 = false;
                }
              }
            }

            if (guard3) {
              dynsim_2link_planar_B.ifirst = dynsim_2link_planar_B.j_m + 1;
              dynsim_2link_planar_B.goto70 = true;
            }
          }
        }

        if ((!dynsim_2link_planar_B.goto60) && (!dynsim_2link_planar_B.goto70))
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
          for (dynsim_2link_planar_B.jp1 = 0; dynsim_2link_planar_B.jp1 < 16;
               dynsim_2link_planar_B.jp1++) {
            Z[dynsim_2link_planar_B.jp1].re = (rtNaN);
            Z[dynsim_2link_planar_B.jp1].im = 0.0;
          }

          *info = 1;
          exitg1 = 1;
        } else if (dynsim_2link_planar_B.goto60) {
          dynsim_2link_planar_B.goto60 = false;
          alpha1[dynsim_2link_planar_B.ilast] = A[(dynsim_2link_planar_B.ilast <<
            2) + dynsim_2link_planar_B.ilast];
          dynsim_2link_planar_B.ilast = dynsim_2link_planar_B.ilastm1;
          dynsim_2link_planar_B.ilastm1--;
          if (dynsim_2link_planar_B.ilast + 1 < ilo) {
            dynsim_2link_planar_B.failed = false;
            guard2 = true;
            exitg1 = 1;
          } else {
            dynsim_2link_planar_B.iiter = 0;
            dynsim_2link_planar_B.eshift_re = 0.0;
            dynsim_2link_planar_B.eshift_im = 0.0;
            dynsim_2link_planar_B.jiter++;
          }
        } else {
          if (dynsim_2link_planar_B.goto70) {
            dynsim_2link_planar_B.goto70 = false;
            dynsim_2link_planar_B.iiter++;
            if (dynsim_2link_planar_B.iiter - div_nzp_s32
                (dynsim_2link_planar_B.iiter, 10) * 10 != 0) {
              dynsim_2link_planar_B.j_m = (dynsim_2link_planar_B.ilastm1 << 2) +
                dynsim_2link_planar_B.ilastm1;
              dynsim_2link_planar_B.ar = A[dynsim_2link_planar_B.j_m].re *
                dynsim_2link_planar_B.anorm;
              dynsim_2link_planar_B.ai = A[dynsim_2link_planar_B.j_m].im *
                dynsim_2link_planar_B.anorm;
              if (dynsim_2link_planar_B.ai == 0.0) {
                dynsim_2link_planar_B.shift_re = dynsim_2link_planar_B.ar / 0.5;
                dynsim_2link_planar_B.shift_im = 0.0;
              } else if (dynsim_2link_planar_B.ar == 0.0) {
                dynsim_2link_planar_B.shift_re = 0.0;
                dynsim_2link_planar_B.shift_im = dynsim_2link_planar_B.ai / 0.5;
              } else {
                dynsim_2link_planar_B.shift_re = dynsim_2link_planar_B.ar / 0.5;
                dynsim_2link_planar_B.shift_im = dynsim_2link_planar_B.ai / 0.5;
              }

              dynsim_2link_planar_B.j_m = (dynsim_2link_planar_B.ilast << 2) +
                dynsim_2link_planar_B.ilast;
              dynsim_2link_planar_B.ar = A[dynsim_2link_planar_B.j_m].re *
                dynsim_2link_planar_B.anorm;
              dynsim_2link_planar_B.ai = A[dynsim_2link_planar_B.j_m].im *
                dynsim_2link_planar_B.anorm;
              if (dynsim_2link_planar_B.ai == 0.0) {
                dynsim_2link_planar_B.ad22.re = dynsim_2link_planar_B.ar / 0.5;
                dynsim_2link_planar_B.ad22.im = 0.0;
              } else if (dynsim_2link_planar_B.ar == 0.0) {
                dynsim_2link_planar_B.ad22.re = 0.0;
                dynsim_2link_planar_B.ad22.im = dynsim_2link_planar_B.ai / 0.5;
              } else {
                dynsim_2link_planar_B.ad22.re = dynsim_2link_planar_B.ar / 0.5;
                dynsim_2link_planar_B.ad22.im = dynsim_2link_planar_B.ai / 0.5;
              }

              dynsim_2link_planar_B.t1_re = (dynsim_2link_planar_B.shift_re +
                dynsim_2link_planar_B.ad22.re) * 0.5;
              dynsim_2link_planar_B.t1_im = (dynsim_2link_planar_B.shift_im +
                dynsim_2link_planar_B.ad22.im) * 0.5;
              dynsim_2link_planar_B.j_m = (dynsim_2link_planar_B.ilast << 2) +
                dynsim_2link_planar_B.ilastm1;
              dynsim_2link_planar_B.ar = A[dynsim_2link_planar_B.j_m].re *
                dynsim_2link_planar_B.anorm;
              dynsim_2link_planar_B.ai = A[dynsim_2link_planar_B.j_m].im *
                dynsim_2link_planar_B.anorm;
              if (dynsim_2link_planar_B.ai == 0.0) {
                dynsim_2link_planar_B.absxr = dynsim_2link_planar_B.ar / 0.5;
                dynsim_2link_planar_B.absxi = 0.0;
              } else if (dynsim_2link_planar_B.ar == 0.0) {
                dynsim_2link_planar_B.absxr = 0.0;
                dynsim_2link_planar_B.absxi = dynsim_2link_planar_B.ai / 0.5;
              } else {
                dynsim_2link_planar_B.absxr = dynsim_2link_planar_B.ar / 0.5;
                dynsim_2link_planar_B.absxi = dynsim_2link_planar_B.ai / 0.5;
              }

              dynsim_2link_planar_B.j_m = (dynsim_2link_planar_B.ilastm1 << 2) +
                dynsim_2link_planar_B.ilast;
              dynsim_2link_planar_B.ar = A[dynsim_2link_planar_B.j_m].re *
                dynsim_2link_planar_B.anorm;
              dynsim_2link_planar_B.ai = A[dynsim_2link_planar_B.j_m].im *
                dynsim_2link_planar_B.anorm;
              if (dynsim_2link_planar_B.ai == 0.0) {
                dynsim_2link_planar_B.ar /= 0.5;
                dynsim_2link_planar_B.ai = 0.0;
              } else if (dynsim_2link_planar_B.ar == 0.0) {
                dynsim_2link_planar_B.ar = 0.0;
                dynsim_2link_planar_B.ai /= 0.5;
              } else {
                dynsim_2link_planar_B.ar /= 0.5;
                dynsim_2link_planar_B.ai /= 0.5;
              }

              dynsim_2link_planar_B.shift_im_e = dynsim_2link_planar_B.shift_re *
                dynsim_2link_planar_B.ad22.im + dynsim_2link_planar_B.shift_im *
                dynsim_2link_planar_B.ad22.re;
              dynsim_2link_planar_B.shift_re = ((dynsim_2link_planar_B.t1_re *
                dynsim_2link_planar_B.t1_re - dynsim_2link_planar_B.t1_im *
                dynsim_2link_planar_B.t1_im) + (dynsim_2link_planar_B.absxr *
                dynsim_2link_planar_B.ar - dynsim_2link_planar_B.absxi *
                dynsim_2link_planar_B.ai)) - (dynsim_2link_planar_B.shift_re *
                dynsim_2link_planar_B.ad22.re - dynsim_2link_planar_B.shift_im *
                dynsim_2link_planar_B.ad22.im);
              dynsim_2link_planar_B.shift_im = dynsim_2link_planar_B.t1_re *
                dynsim_2link_planar_B.t1_im;
              dynsim_2link_planar_B.shift_im = ((dynsim_2link_planar_B.shift_im
                + dynsim_2link_planar_B.shift_im) + (dynsim_2link_planar_B.absxr
                * dynsim_2link_planar_B.ai + dynsim_2link_planar_B.absxi *
                dynsim_2link_planar_B.ar)) - dynsim_2link_planar_B.shift_im_e;
              if (dynsim_2link_planar_B.shift_im == 0.0) {
                if (dynsim_2link_planar_B.shift_re < 0.0) {
                  dynsim_2link_planar_B.absxr = 0.0;
                  dynsim_2link_planar_B.absxi = sqrt
                    (-dynsim_2link_planar_B.shift_re);
                } else {
                  dynsim_2link_planar_B.absxr = sqrt
                    (dynsim_2link_planar_B.shift_re);
                  dynsim_2link_planar_B.absxi = 0.0;
                }
              } else if (dynsim_2link_planar_B.shift_re == 0.0) {
                if (dynsim_2link_planar_B.shift_im < 0.0) {
                  dynsim_2link_planar_B.absxr = sqrt
                    (-dynsim_2link_planar_B.shift_im / 2.0);
                  dynsim_2link_planar_B.absxi = -dynsim_2link_planar_B.absxr;
                } else {
                  dynsim_2link_planar_B.absxr = sqrt
                    (dynsim_2link_planar_B.shift_im / 2.0);
                  dynsim_2link_planar_B.absxi = dynsim_2link_planar_B.absxr;
                }
              } else if (rtIsNaN(dynsim_2link_planar_B.shift_re)) {
                dynsim_2link_planar_B.absxr = dynsim_2link_planar_B.shift_re;
                dynsim_2link_planar_B.absxi = dynsim_2link_planar_B.shift_re;
              } else if (rtIsNaN(dynsim_2link_planar_B.shift_im)) {
                dynsim_2link_planar_B.absxr = dynsim_2link_planar_B.shift_im;
                dynsim_2link_planar_B.absxi = dynsim_2link_planar_B.shift_im;
              } else if (rtIsInf(dynsim_2link_planar_B.shift_im)) {
                dynsim_2link_planar_B.absxr = fabs
                  (dynsim_2link_planar_B.shift_im);
                dynsim_2link_planar_B.absxi = dynsim_2link_planar_B.shift_im;
              } else if (rtIsInf(dynsim_2link_planar_B.shift_re)) {
                if (dynsim_2link_planar_B.shift_re < 0.0) {
                  dynsim_2link_planar_B.absxr = 0.0;
                  dynsim_2link_planar_B.absxi = dynsim_2link_planar_B.shift_im *
                    -dynsim_2link_planar_B.shift_re;
                } else {
                  dynsim_2link_planar_B.absxr = dynsim_2link_planar_B.shift_re;
                  dynsim_2link_planar_B.absxi = 0.0;
                }
              } else {
                dynsim_2link_planar_B.absxr = fabs
                  (dynsim_2link_planar_B.shift_re);
                dynsim_2link_planar_B.absxi = fabs
                  (dynsim_2link_planar_B.shift_im);
                if ((dynsim_2link_planar_B.absxr > 4.4942328371557893E+307) ||
                    (dynsim_2link_planar_B.absxi > 4.4942328371557893E+307)) {
                  dynsim_2link_planar_B.absxr *= 0.5;
                  dynsim_2link_planar_B.absxi *= 0.5;
                  dynsim_2link_planar_B.absxi = dynsim_2link_plan_rt_hypotd_snf
                    (dynsim_2link_planar_B.absxr, dynsim_2link_planar_B.absxi);
                  if (dynsim_2link_planar_B.absxi > dynsim_2link_planar_B.absxr)
                  {
                    dynsim_2link_planar_B.absxr = sqrt
                      (dynsim_2link_planar_B.absxr / dynsim_2link_planar_B.absxi
                       + 1.0) * sqrt(dynsim_2link_planar_B.absxi);
                  } else {
                    dynsim_2link_planar_B.absxr = sqrt
                      (dynsim_2link_planar_B.absxi) * 1.4142135623730951;
                  }
                } else {
                  dynsim_2link_planar_B.absxr = sqrt
                    ((dynsim_2link_plan_rt_hypotd_snf
                      (dynsim_2link_planar_B.absxr, dynsim_2link_planar_B.absxi)
                      + dynsim_2link_planar_B.absxr) * 0.5);
                }

                if (dynsim_2link_planar_B.shift_re > 0.0) {
                  dynsim_2link_planar_B.absxi = dynsim_2link_planar_B.shift_im /
                    dynsim_2link_planar_B.absxr * 0.5;
                } else {
                  if (dynsim_2link_planar_B.shift_im < 0.0) {
                    dynsim_2link_planar_B.absxi = -dynsim_2link_planar_B.absxr;
                  } else {
                    dynsim_2link_planar_B.absxi = dynsim_2link_planar_B.absxr;
                  }

                  dynsim_2link_planar_B.absxr = dynsim_2link_planar_B.shift_im /
                    dynsim_2link_planar_B.absxi * 0.5;
                }
              }

              if ((dynsim_2link_planar_B.t1_re - dynsim_2link_planar_B.ad22.re) *
                  dynsim_2link_planar_B.absxr + (dynsim_2link_planar_B.t1_im -
                   dynsim_2link_planar_B.ad22.im) * dynsim_2link_planar_B.absxi <=
                  0.0) {
                dynsim_2link_planar_B.shift_re = dynsim_2link_planar_B.t1_re +
                  dynsim_2link_planar_B.absxr;
                dynsim_2link_planar_B.shift_im = dynsim_2link_planar_B.t1_im +
                  dynsim_2link_planar_B.absxi;
              } else {
                dynsim_2link_planar_B.shift_re = dynsim_2link_planar_B.t1_re -
                  dynsim_2link_planar_B.absxr;
                dynsim_2link_planar_B.shift_im = dynsim_2link_planar_B.t1_im -
                  dynsim_2link_planar_B.absxi;
              }
            } else {
              dynsim_2link_planar_B.j_m = (dynsim_2link_planar_B.ilastm1 << 2) +
                dynsim_2link_planar_B.ilast;
              dynsim_2link_planar_B.ar = A[dynsim_2link_planar_B.j_m].re *
                dynsim_2link_planar_B.anorm;
              dynsim_2link_planar_B.ai = A[dynsim_2link_planar_B.j_m].im *
                dynsim_2link_planar_B.anorm;
              if (dynsim_2link_planar_B.ai == 0.0) {
                dynsim_2link_planar_B.absxr = dynsim_2link_planar_B.ar / 0.5;
                dynsim_2link_planar_B.absxi = 0.0;
              } else if (dynsim_2link_planar_B.ar == 0.0) {
                dynsim_2link_planar_B.absxr = 0.0;
                dynsim_2link_planar_B.absxi = dynsim_2link_planar_B.ai / 0.5;
              } else {
                dynsim_2link_planar_B.absxr = dynsim_2link_planar_B.ar / 0.5;
                dynsim_2link_planar_B.absxi = dynsim_2link_planar_B.ai / 0.5;
              }

              dynsim_2link_planar_B.eshift_re += dynsim_2link_planar_B.absxr;
              dynsim_2link_planar_B.eshift_im += dynsim_2link_planar_B.absxi;
              dynsim_2link_planar_B.shift_re = dynsim_2link_planar_B.eshift_re;
              dynsim_2link_planar_B.shift_im = dynsim_2link_planar_B.eshift_im;
            }

            dynsim_2link_planar_B.j_m = dynsim_2link_planar_B.ilastm1;
            dynsim_2link_planar_B.jp1 = dynsim_2link_planar_B.ilastm1 + 1;
            exitg2 = false;
            while ((!exitg2) && (dynsim_2link_planar_B.j_m + 1 >
                                 dynsim_2link_planar_B.ifirst)) {
              dynsim_2link_planar_B.istart = dynsim_2link_planar_B.j_m + 1;
              dynsim_2link_planar_B.ctemp_tmp_tmp = dynsim_2link_planar_B.j_m <<
                2;
              dynsim_2link_planar_B.ctemp_tmp =
                dynsim_2link_planar_B.ctemp_tmp_tmp + dynsim_2link_planar_B.j_m;
              dynsim_2link_planar_B.ctemp.re = A[dynsim_2link_planar_B.ctemp_tmp]
                .re * dynsim_2link_planar_B.anorm -
                dynsim_2link_planar_B.shift_re * 0.5;
              dynsim_2link_planar_B.ctemp.im = A[dynsim_2link_planar_B.ctemp_tmp]
                .im * dynsim_2link_planar_B.anorm -
                dynsim_2link_planar_B.shift_im * 0.5;
              dynsim_2link_planar_B.t1_re = fabs(dynsim_2link_planar_B.ctemp.re)
                + fabs(dynsim_2link_planar_B.ctemp.im);
              dynsim_2link_planar_B.jp1 += dynsim_2link_planar_B.ctemp_tmp_tmp;
              dynsim_2link_planar_B.t1_im = (fabs(A[dynsim_2link_planar_B.jp1].
                re) + fabs(A[dynsim_2link_planar_B.jp1].im)) *
                dynsim_2link_planar_B.anorm;
              dynsim_2link_planar_B.absxr = dynsim_2link_planar_B.t1_re;
              if (dynsim_2link_planar_B.t1_im > dynsim_2link_planar_B.t1_re) {
                dynsim_2link_planar_B.absxr = dynsim_2link_planar_B.t1_im;
              }

              if ((dynsim_2link_planar_B.absxr < 1.0) &&
                  (dynsim_2link_planar_B.absxr != 0.0)) {
                dynsim_2link_planar_B.t1_re /= dynsim_2link_planar_B.absxr;
                dynsim_2link_planar_B.t1_im /= dynsim_2link_planar_B.absxr;
              }

              dynsim_2link_planar_B.jp1 = ((dynsim_2link_planar_B.j_m - 1) << 2)
                + dynsim_2link_planar_B.j_m;
              if ((fabs(A[dynsim_2link_planar_B.jp1].re) + fabs
                   (A[dynsim_2link_planar_B.jp1].im)) *
                  dynsim_2link_planar_B.t1_im <= dynsim_2link_planar_B.t1_re *
                  dynsim_2link_planar_B.b_atol) {
                dynsim_2link_planar_B.goto90 = true;
                exitg2 = true;
              } else {
                dynsim_2link_planar_B.jp1 = dynsim_2link_planar_B.j_m;
                dynsim_2link_planar_B.j_m--;
              }
            }

            if (!dynsim_2link_planar_B.goto90) {
              dynsim_2link_planar_B.istart = dynsim_2link_planar_B.ifirst;
              dynsim_2link_planar_B.ctemp_tmp = (((dynsim_2link_planar_B.ifirst
                - 1) << 2) + dynsim_2link_planar_B.ifirst) - 1;
              dynsim_2link_planar_B.ctemp.re = A[dynsim_2link_planar_B.ctemp_tmp]
                .re * dynsim_2link_planar_B.anorm -
                dynsim_2link_planar_B.shift_re * 0.5;
              dynsim_2link_planar_B.ctemp.im = A[dynsim_2link_planar_B.ctemp_tmp]
                .im * dynsim_2link_planar_B.anorm -
                dynsim_2link_planar_B.shift_im * 0.5;
            }

            dynsim_2link_planar_B.goto90 = false;
            dynsim_2link_planar_B.j_m = ((dynsim_2link_planar_B.istart - 1) << 2)
              + dynsim_2link_planar_B.istart;
            dynsim_2link_planar_B.ascale.re = A[dynsim_2link_planar_B.j_m].re *
              dynsim_2link_planar_B.anorm;
            dynsim_2link_planar_B.ascale.im = A[dynsim_2link_planar_B.j_m].im *
              dynsim_2link_planar_B.anorm;
            dynsim_2link_planar_xzlartg_d(dynsim_2link_planar_B.ctemp,
              dynsim_2link_planar_B.ascale, &dynsim_2link_planar_B.t1_re,
              &dynsim_2link_planar_B.ad22);
            dynsim_2link_planar_B.j_m = dynsim_2link_planar_B.istart;
            dynsim_2link_planar_B.jp1 = dynsim_2link_planar_B.istart - 2;
            while (dynsim_2link_planar_B.j_m < dynsim_2link_planar_B.ilast + 1)
            {
              if (dynsim_2link_planar_B.j_m > dynsim_2link_planar_B.istart) {
                dynsim_2link_planar_xzlartg(A[(dynsim_2link_planar_B.j_m +
                  (dynsim_2link_planar_B.jp1 << 2)) - 1],
                  A[dynsim_2link_planar_B.j_m + (dynsim_2link_planar_B.jp1 << 2)],
                  &dynsim_2link_planar_B.t1_re, &dynsim_2link_planar_B.ad22, &A
                  [(dynsim_2link_planar_B.j_m + (dynsim_2link_planar_B.jp1 << 2))
                  - 1]);
                dynsim_2link_planar_B.jp1 = dynsim_2link_planar_B.j_m +
                  (dynsim_2link_planar_B.jp1 << 2);
                A[dynsim_2link_planar_B.jp1].re = 0.0;
                A[dynsim_2link_planar_B.jp1].im = 0.0;
              }

              dynsim_2link_planar_B.ctemp_tmp = dynsim_2link_planar_B.j_m - 1;
              while (dynsim_2link_planar_B.ctemp_tmp + 1 < 5) {
                dynsim_2link_planar_B.jp1 = (dynsim_2link_planar_B.ctemp_tmp <<
                  2) + dynsim_2link_planar_B.j_m;
                dynsim_2link_planar_B.ctemp_tmp_tmp = dynsim_2link_planar_B.jp1
                  - 1;
                dynsim_2link_planar_B.shift_re =
                  A[dynsim_2link_planar_B.ctemp_tmp_tmp].re *
                  dynsim_2link_planar_B.t1_re + (A[dynsim_2link_planar_B.jp1].re
                  * dynsim_2link_planar_B.ad22.re - A[dynsim_2link_planar_B.jp1]
                  .im * dynsim_2link_planar_B.ad22.im);
                dynsim_2link_planar_B.shift_im =
                  A[dynsim_2link_planar_B.ctemp_tmp_tmp].im *
                  dynsim_2link_planar_B.t1_re + (A[dynsim_2link_planar_B.jp1].im
                  * dynsim_2link_planar_B.ad22.re + A[dynsim_2link_planar_B.jp1]
                  .re * dynsim_2link_planar_B.ad22.im);
                dynsim_2link_planar_B.t1_im =
                  A[dynsim_2link_planar_B.ctemp_tmp_tmp].im;
                dynsim_2link_planar_B.absxr =
                  A[dynsim_2link_planar_B.ctemp_tmp_tmp].re;
                A[dynsim_2link_planar_B.jp1].re = A[dynsim_2link_planar_B.jp1].
                  re * dynsim_2link_planar_B.t1_re -
                  (A[dynsim_2link_planar_B.ctemp_tmp_tmp].re *
                   dynsim_2link_planar_B.ad22.re +
                   A[dynsim_2link_planar_B.ctemp_tmp_tmp].im *
                   dynsim_2link_planar_B.ad22.im);
                A[dynsim_2link_planar_B.jp1].im = A[dynsim_2link_planar_B.jp1].
                  im * dynsim_2link_planar_B.t1_re -
                  (dynsim_2link_planar_B.ad22.re * dynsim_2link_planar_B.t1_im -
                   dynsim_2link_planar_B.ad22.im * dynsim_2link_planar_B.absxr);
                A[dynsim_2link_planar_B.ctemp_tmp_tmp].re =
                  dynsim_2link_planar_B.shift_re;
                A[dynsim_2link_planar_B.ctemp_tmp_tmp].im =
                  dynsim_2link_planar_B.shift_im;
                dynsim_2link_planar_B.ctemp_tmp++;
              }

              dynsim_2link_planar_B.ad22.re = -dynsim_2link_planar_B.ad22.re;
              dynsim_2link_planar_B.ad22.im = -dynsim_2link_planar_B.ad22.im;
              dynsim_2link_planar_B.ctemp_tmp = dynsim_2link_planar_B.j_m;
              if (dynsim_2link_planar_B.ilast + 1 < dynsim_2link_planar_B.j_m +
                  2) {
                dynsim_2link_planar_B.ctemp_tmp = dynsim_2link_planar_B.ilast -
                  1;
              }

              dynsim_2link_planar_B.i_jg = 0;
              while (dynsim_2link_planar_B.i_jg + 1 <=
                     dynsim_2link_planar_B.ctemp_tmp + 2) {
                dynsim_2link_planar_B.jp1 = ((dynsim_2link_planar_B.j_m - 1) <<
                  2) + dynsim_2link_planar_B.i_jg;
                dynsim_2link_planar_B.ctemp_tmp_tmp = (dynsim_2link_planar_B.j_m
                  << 2) + dynsim_2link_planar_B.i_jg;
                dynsim_2link_planar_B.shift_re = (A[dynsim_2link_planar_B.jp1].
                  re * dynsim_2link_planar_B.ad22.re -
                  A[dynsim_2link_planar_B.jp1].im *
                  dynsim_2link_planar_B.ad22.im) +
                  A[dynsim_2link_planar_B.ctemp_tmp_tmp].re *
                  dynsim_2link_planar_B.t1_re;
                dynsim_2link_planar_B.shift_im = (A[dynsim_2link_planar_B.jp1].
                  im * dynsim_2link_planar_B.ad22.re +
                  A[dynsim_2link_planar_B.jp1].re *
                  dynsim_2link_planar_B.ad22.im) +
                  A[dynsim_2link_planar_B.ctemp_tmp_tmp].im *
                  dynsim_2link_planar_B.t1_re;
                dynsim_2link_planar_B.t1_im =
                  A[dynsim_2link_planar_B.ctemp_tmp_tmp].im;
                dynsim_2link_planar_B.absxr =
                  A[dynsim_2link_planar_B.ctemp_tmp_tmp].re;
                A[dynsim_2link_planar_B.jp1].re = A[dynsim_2link_planar_B.jp1].
                  re * dynsim_2link_planar_B.t1_re -
                  (A[dynsim_2link_planar_B.ctemp_tmp_tmp].re *
                   dynsim_2link_planar_B.ad22.re +
                   A[dynsim_2link_planar_B.ctemp_tmp_tmp].im *
                   dynsim_2link_planar_B.ad22.im);
                A[dynsim_2link_planar_B.jp1].im = A[dynsim_2link_planar_B.jp1].
                  im * dynsim_2link_planar_B.t1_re -
                  (dynsim_2link_planar_B.ad22.re * dynsim_2link_planar_B.t1_im -
                   dynsim_2link_planar_B.ad22.im * dynsim_2link_planar_B.absxr);
                A[dynsim_2link_planar_B.ctemp_tmp_tmp].re =
                  dynsim_2link_planar_B.shift_re;
                A[dynsim_2link_planar_B.ctemp_tmp_tmp].im =
                  dynsim_2link_planar_B.shift_im;
                dynsim_2link_planar_B.i_jg++;
              }

              dynsim_2link_planar_B.jp1 = (dynsim_2link_planar_B.j_m - 1) << 2;
              dynsim_2link_planar_B.ctemp_tmp_tmp = dynsim_2link_planar_B.j_m <<
                2;
              dynsim_2link_planar_B.shift_re = (Z[dynsim_2link_planar_B.jp1].re *
                dynsim_2link_planar_B.ad22.re - Z[dynsim_2link_planar_B.jp1].im *
                dynsim_2link_planar_B.ad22.im) +
                Z[dynsim_2link_planar_B.ctemp_tmp_tmp].re *
                dynsim_2link_planar_B.t1_re;
              dynsim_2link_planar_B.shift_im = (Z[dynsim_2link_planar_B.jp1].im *
                dynsim_2link_planar_B.ad22.re + Z[dynsim_2link_planar_B.jp1].re *
                dynsim_2link_planar_B.ad22.im) +
                Z[dynsim_2link_planar_B.ctemp_tmp_tmp].im *
                dynsim_2link_planar_B.t1_re;
              dynsim_2link_planar_B.t1_im =
                Z[dynsim_2link_planar_B.ctemp_tmp_tmp].im;
              dynsim_2link_planar_B.absxr =
                Z[dynsim_2link_planar_B.ctemp_tmp_tmp].re;
              Z[dynsim_2link_planar_B.jp1].re = Z[dynsim_2link_planar_B.jp1].re *
                dynsim_2link_planar_B.t1_re -
                (Z[dynsim_2link_planar_B.ctemp_tmp_tmp].re *
                 dynsim_2link_planar_B.ad22.re +
                 Z[dynsim_2link_planar_B.ctemp_tmp_tmp].im *
                 dynsim_2link_planar_B.ad22.im);
              Z[dynsim_2link_planar_B.jp1].im = Z[dynsim_2link_planar_B.jp1].im *
                dynsim_2link_planar_B.t1_re - (dynsim_2link_planar_B.ad22.re *
                dynsim_2link_planar_B.t1_im - dynsim_2link_planar_B.ad22.im *
                dynsim_2link_planar_B.absxr);
              Z[dynsim_2link_planar_B.ctemp_tmp_tmp].re =
                dynsim_2link_planar_B.shift_re;
              Z[dynsim_2link_planar_B.ctemp_tmp_tmp].im =
                dynsim_2link_planar_B.shift_im;
              dynsim_2link_planar_B.ctemp_tmp = dynsim_2link_planar_B.jp1 + 1;
              dynsim_2link_planar_B.i_jg = dynsim_2link_planar_B.ctemp_tmp_tmp +
                1;
              dynsim_2link_planar_B.shift_re =
                (Z[dynsim_2link_planar_B.ctemp_tmp].re *
                 dynsim_2link_planar_B.ad22.re -
                 Z[dynsim_2link_planar_B.ctemp_tmp].im *
                 dynsim_2link_planar_B.ad22.im) + Z[dynsim_2link_planar_B.i_jg].
                re * dynsim_2link_planar_B.t1_re;
              dynsim_2link_planar_B.shift_im =
                (Z[dynsim_2link_planar_B.ctemp_tmp].im *
                 dynsim_2link_planar_B.ad22.re +
                 Z[dynsim_2link_planar_B.ctemp_tmp].re *
                 dynsim_2link_planar_B.ad22.im) + Z[dynsim_2link_planar_B.i_jg].
                im * dynsim_2link_planar_B.t1_re;
              dynsim_2link_planar_B.t1_im = Z[dynsim_2link_planar_B.i_jg].im;
              dynsim_2link_planar_B.absxr = Z[dynsim_2link_planar_B.i_jg].re;
              Z[dynsim_2link_planar_B.ctemp_tmp].re =
                Z[dynsim_2link_planar_B.ctemp_tmp].re *
                dynsim_2link_planar_B.t1_re - (Z[dynsim_2link_planar_B.i_jg].re *
                dynsim_2link_planar_B.ad22.re + Z[dynsim_2link_planar_B.i_jg].im
                * dynsim_2link_planar_B.ad22.im);
              Z[dynsim_2link_planar_B.ctemp_tmp].im =
                Z[dynsim_2link_planar_B.ctemp_tmp].im *
                dynsim_2link_planar_B.t1_re - (dynsim_2link_planar_B.ad22.re *
                dynsim_2link_planar_B.t1_im - dynsim_2link_planar_B.ad22.im *
                dynsim_2link_planar_B.absxr);
              Z[dynsim_2link_planar_B.i_jg].re = dynsim_2link_planar_B.shift_re;
              Z[dynsim_2link_planar_B.i_jg].im = dynsim_2link_planar_B.shift_im;
              dynsim_2link_planar_B.ctemp_tmp = dynsim_2link_planar_B.jp1 + 2;
              dynsim_2link_planar_B.i_jg = dynsim_2link_planar_B.ctemp_tmp_tmp +
                2;
              dynsim_2link_planar_B.shift_re =
                (Z[dynsim_2link_planar_B.ctemp_tmp].re *
                 dynsim_2link_planar_B.ad22.re -
                 Z[dynsim_2link_planar_B.ctemp_tmp].im *
                 dynsim_2link_planar_B.ad22.im) + Z[dynsim_2link_planar_B.i_jg].
                re * dynsim_2link_planar_B.t1_re;
              dynsim_2link_planar_B.shift_im =
                (Z[dynsim_2link_planar_B.ctemp_tmp].im *
                 dynsim_2link_planar_B.ad22.re +
                 Z[dynsim_2link_planar_B.ctemp_tmp].re *
                 dynsim_2link_planar_B.ad22.im) + Z[dynsim_2link_planar_B.i_jg].
                im * dynsim_2link_planar_B.t1_re;
              dynsim_2link_planar_B.t1_im = Z[dynsim_2link_planar_B.i_jg].im;
              dynsim_2link_planar_B.absxr = Z[dynsim_2link_planar_B.i_jg].re;
              Z[dynsim_2link_planar_B.ctemp_tmp].re =
                Z[dynsim_2link_planar_B.ctemp_tmp].re *
                dynsim_2link_planar_B.t1_re - (Z[dynsim_2link_planar_B.i_jg].re *
                dynsim_2link_planar_B.ad22.re + Z[dynsim_2link_planar_B.i_jg].im
                * dynsim_2link_planar_B.ad22.im);
              Z[dynsim_2link_planar_B.ctemp_tmp].im =
                Z[dynsim_2link_planar_B.ctemp_tmp].im *
                dynsim_2link_planar_B.t1_re - (dynsim_2link_planar_B.ad22.re *
                dynsim_2link_planar_B.t1_im - dynsim_2link_planar_B.ad22.im *
                dynsim_2link_planar_B.absxr);
              Z[dynsim_2link_planar_B.i_jg].re = dynsim_2link_planar_B.shift_re;
              Z[dynsim_2link_planar_B.i_jg].im = dynsim_2link_planar_B.shift_im;
              dynsim_2link_planar_B.jp1 += 3;
              dynsim_2link_planar_B.ctemp_tmp_tmp += 3;
              dynsim_2link_planar_B.shift_re = (Z[dynsim_2link_planar_B.jp1].re *
                dynsim_2link_planar_B.ad22.re - Z[dynsim_2link_planar_B.jp1].im *
                dynsim_2link_planar_B.ad22.im) +
                Z[dynsim_2link_planar_B.ctemp_tmp_tmp].re *
                dynsim_2link_planar_B.t1_re;
              dynsim_2link_planar_B.shift_im = (Z[dynsim_2link_planar_B.jp1].im *
                dynsim_2link_planar_B.ad22.re + Z[dynsim_2link_planar_B.jp1].re *
                dynsim_2link_planar_B.ad22.im) +
                Z[dynsim_2link_planar_B.ctemp_tmp_tmp].im *
                dynsim_2link_planar_B.t1_re;
              dynsim_2link_planar_B.t1_im =
                Z[dynsim_2link_planar_B.ctemp_tmp_tmp].im;
              dynsim_2link_planar_B.absxr =
                Z[dynsim_2link_planar_B.ctemp_tmp_tmp].re;
              Z[dynsim_2link_planar_B.jp1].re = Z[dynsim_2link_planar_B.jp1].re *
                dynsim_2link_planar_B.t1_re -
                (Z[dynsim_2link_planar_B.ctemp_tmp_tmp].re *
                 dynsim_2link_planar_B.ad22.re +
                 Z[dynsim_2link_planar_B.ctemp_tmp_tmp].im *
                 dynsim_2link_planar_B.ad22.im);
              Z[dynsim_2link_planar_B.jp1].im = Z[dynsim_2link_planar_B.jp1].im *
                dynsim_2link_planar_B.t1_re - (dynsim_2link_planar_B.ad22.re *
                dynsim_2link_planar_B.t1_im - dynsim_2link_planar_B.ad22.im *
                dynsim_2link_planar_B.absxr);
              Z[dynsim_2link_planar_B.ctemp_tmp_tmp].re =
                dynsim_2link_planar_B.shift_re;
              Z[dynsim_2link_planar_B.ctemp_tmp_tmp].im =
                dynsim_2link_planar_B.shift_im;
              dynsim_2link_planar_B.jp1 = dynsim_2link_planar_B.j_m - 1;
              dynsim_2link_planar_B.j_m++;
            }
          }

          dynsim_2link_planar_B.jiter++;
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
    if (dynsim_2link_planar_B.failed) {
      *info = dynsim_2link_planar_B.ilast + 1;
      dynsim_2link_planar_B.ifirst = 0;
      while (dynsim_2link_planar_B.ifirst <= dynsim_2link_planar_B.ilast) {
        alpha1[dynsim_2link_planar_B.ifirst].re = (rtNaN);
        alpha1[dynsim_2link_planar_B.ifirst].im = 0.0;
        beta1[dynsim_2link_planar_B.ifirst].re = (rtNaN);
        beta1[dynsim_2link_planar_B.ifirst].im = 0.0;
        dynsim_2link_planar_B.ifirst++;
      }

      for (dynsim_2link_planar_B.jp1 = 0; dynsim_2link_planar_B.jp1 < 16;
           dynsim_2link_planar_B.jp1++) {
        Z[dynsim_2link_planar_B.jp1].re = (rtNaN);
        Z[dynsim_2link_planar_B.jp1].im = 0.0;
      }
    } else {
      guard1 = true;
    }
  }

  if (guard1) {
    dynsim_2link_planar_B.ilast = 0;
    while (dynsim_2link_planar_B.ilast <= ilo - 2) {
      alpha1[dynsim_2link_planar_B.ilast] = A[(dynsim_2link_planar_B.ilast << 2)
        + dynsim_2link_planar_B.ilast];
      dynsim_2link_planar_B.ilast++;
    }
  }
}

// Function for MATLAB Function: '<S4>/MATLAB Function'
static void dynsim_2link_planar_xztgevc(const creal_T A[16], creal_T V[16])
{
  dynsim_2link_planar_B.rworka[0] = 0.0;
  dynsim_2link_planar_B.rworka[2] = 0.0;
  dynsim_2link_planar_B.rworka[3] = 0.0;
  dynsim_2link_planar_B.anorm_p = fabs(A[0].re) + fabs(A[0].im);
  dynsim_2link_planar_B.rworka[1] = fabs(A[4].re) + fabs(A[4].im);
  dynsim_2link_planar_B.ascale_a = (fabs(A[5].re) + fabs(A[5].im)) +
    dynsim_2link_planar_B.rworka[1];
  if (dynsim_2link_planar_B.ascale_a > dynsim_2link_planar_B.anorm_p) {
    dynsim_2link_planar_B.anorm_p = dynsim_2link_planar_B.ascale_a;
  }

  dynsim_2link_planar_B.i_m = 0;
  while (dynsim_2link_planar_B.i_m <= 1) {
    dynsim_2link_planar_B.rworka[2] += fabs(A[dynsim_2link_planar_B.i_m + 8].re)
      + fabs(A[dynsim_2link_planar_B.i_m + 8].im);
    dynsim_2link_planar_B.i_m++;
  }

  dynsim_2link_planar_B.ascale_a = (fabs(A[10].re) + fabs(A[10].im)) +
    dynsim_2link_planar_B.rworka[2];
  if (dynsim_2link_planar_B.ascale_a > dynsim_2link_planar_B.anorm_p) {
    dynsim_2link_planar_B.anorm_p = dynsim_2link_planar_B.ascale_a;
  }

  dynsim_2link_planar_B.i_m = 0;
  while (dynsim_2link_planar_B.i_m <= 2) {
    dynsim_2link_planar_B.rworka[3] += fabs(A[dynsim_2link_planar_B.i_m + 12].re)
      + fabs(A[dynsim_2link_planar_B.i_m + 12].im);
    dynsim_2link_planar_B.i_m++;
  }

  dynsim_2link_planar_B.ascale_a = (fabs(A[15].re) + fabs(A[15].im)) +
    dynsim_2link_planar_B.rworka[3];
  if (dynsim_2link_planar_B.ascale_a > dynsim_2link_planar_B.anorm_p) {
    dynsim_2link_planar_B.anorm_p = dynsim_2link_planar_B.ascale_a;
  }

  dynsim_2link_planar_B.ascale_a = dynsim_2link_planar_B.anorm_p;
  if (2.2250738585072014E-308 > dynsim_2link_planar_B.anorm_p) {
    dynsim_2link_planar_B.ascale_a = 2.2250738585072014E-308;
  }

  dynsim_2link_planar_B.ascale_a = 1.0 / dynsim_2link_planar_B.ascale_a;
  for (dynsim_2link_planar_B.i_m = 0; dynsim_2link_planar_B.i_m < 4;
       dynsim_2link_planar_B.i_m++) {
    dynsim_2link_planar_B.c_x_tmp_tmp = (3 - dynsim_2link_planar_B.i_m) << 2;
    dynsim_2link_planar_B.c_x_tmp = (dynsim_2link_planar_B.c_x_tmp_tmp -
      dynsim_2link_planar_B.i_m) + 3;
    dynsim_2link_planar_B.salpha_re = (fabs(A[dynsim_2link_planar_B.c_x_tmp].re)
      + fabs(A[dynsim_2link_planar_B.c_x_tmp].im)) *
      dynsim_2link_planar_B.ascale_a;
    if (1.0 > dynsim_2link_planar_B.salpha_re) {
      dynsim_2link_planar_B.salpha_re = 1.0;
    }

    dynsim_2link_planar_B.temp = 1.0 / dynsim_2link_planar_B.salpha_re;
    dynsim_2link_planar_B.salpha_re = A[dynsim_2link_planar_B.c_x_tmp].re *
      dynsim_2link_planar_B.temp * dynsim_2link_planar_B.ascale_a;
    dynsim_2link_planar_B.salpha_im = A[dynsim_2link_planar_B.c_x_tmp].im *
      dynsim_2link_planar_B.temp * dynsim_2link_planar_B.ascale_a;
    dynsim_2link_planar_B.acoeff = dynsim_2link_planar_B.temp *
      dynsim_2link_planar_B.ascale_a;
    dynsim_2link_planar_B.lscalea = ((dynsim_2link_planar_B.temp >=
      2.2250738585072014E-308) && (dynsim_2link_planar_B.acoeff <
      4.0083367200179456E-292));
    dynsim_2link_planar_B.dmin = fabs(dynsim_2link_planar_B.salpha_re) + fabs
      (dynsim_2link_planar_B.salpha_im);
    dynsim_2link_planar_B.lscaleb = ((dynsim_2link_planar_B.dmin >=
      2.2250738585072014E-308) && (dynsim_2link_planar_B.dmin <
      4.0083367200179456E-292));
    dynsim_2link_planar_B.scale_j = 1.0;
    if (dynsim_2link_planar_B.lscalea) {
      dynsim_2link_planar_B.scale_j = dynsim_2link_planar_B.anorm_p;
      if (2.4948003869184E+291 < dynsim_2link_planar_B.anorm_p) {
        dynsim_2link_planar_B.scale_j = 2.4948003869184E+291;
      }

      dynsim_2link_planar_B.scale_j *= 4.0083367200179456E-292 /
        dynsim_2link_planar_B.temp;
    }

    if (dynsim_2link_planar_B.lscaleb) {
      dynsim_2link_planar_B.work2_idx_2_im = 4.0083367200179456E-292 /
        dynsim_2link_planar_B.dmin;
      if (dynsim_2link_planar_B.work2_idx_2_im > dynsim_2link_planar_B.scale_j)
      {
        dynsim_2link_planar_B.scale_j = dynsim_2link_planar_B.work2_idx_2_im;
      }
    }

    if (dynsim_2link_planar_B.lscalea || dynsim_2link_planar_B.lscaleb) {
      dynsim_2link_planar_B.work2_idx_2_im = dynsim_2link_planar_B.acoeff;
      if (1.0 > dynsim_2link_planar_B.acoeff) {
        dynsim_2link_planar_B.work2_idx_2_im = 1.0;
      }

      if (dynsim_2link_planar_B.dmin > dynsim_2link_planar_B.work2_idx_2_im) {
        dynsim_2link_planar_B.work2_idx_2_im = dynsim_2link_planar_B.dmin;
      }

      dynsim_2link_planar_B.dmin = 1.0 / (2.2250738585072014E-308 *
        dynsim_2link_planar_B.work2_idx_2_im);
      if (dynsim_2link_planar_B.dmin < dynsim_2link_planar_B.scale_j) {
        dynsim_2link_planar_B.scale_j = dynsim_2link_planar_B.dmin;
      }

      if (dynsim_2link_planar_B.lscalea) {
        dynsim_2link_planar_B.acoeff = dynsim_2link_planar_B.scale_j *
          dynsim_2link_planar_B.temp * dynsim_2link_planar_B.ascale_a;
      } else {
        dynsim_2link_planar_B.acoeff *= dynsim_2link_planar_B.scale_j;
      }

      dynsim_2link_planar_B.salpha_re *= dynsim_2link_planar_B.scale_j;
      dynsim_2link_planar_B.salpha_im *= dynsim_2link_planar_B.scale_j;
    }

    memset(&dynsim_2link_planar_B.work1[0], 0, sizeof(creal_T) << 2U);
    dynsim_2link_planar_B.work1[3 - dynsim_2link_planar_B.i_m].re = 1.0;
    dynsim_2link_planar_B.work1[3 - dynsim_2link_planar_B.i_m].im = 0.0;
    dynsim_2link_planar_B.dmin = 2.2204460492503131E-16 *
      dynsim_2link_planar_B.acoeff * dynsim_2link_planar_B.anorm_p;
    dynsim_2link_planar_B.temp = (fabs(dynsim_2link_planar_B.salpha_re) + fabs
      (dynsim_2link_planar_B.salpha_im)) * 2.2204460492503131E-16;
    if (dynsim_2link_planar_B.temp > dynsim_2link_planar_B.dmin) {
      dynsim_2link_planar_B.dmin = dynsim_2link_planar_B.temp;
    }

    if (2.2250738585072014E-308 > dynsim_2link_planar_B.dmin) {
      dynsim_2link_planar_B.dmin = 2.2250738585072014E-308;
    }

    dynsim_2link_planar_B.c_x_tmp = 0;
    while (dynsim_2link_planar_B.c_x_tmp <= 2 - dynsim_2link_planar_B.i_m) {
      dynsim_2link_planar_B.d_re_tmp = dynsim_2link_planar_B.c_x_tmp_tmp +
        dynsim_2link_planar_B.c_x_tmp;
      dynsim_2link_planar_B.work1[dynsim_2link_planar_B.c_x_tmp].re =
        A[dynsim_2link_planar_B.d_re_tmp].re * dynsim_2link_planar_B.acoeff;
      dynsim_2link_planar_B.work1[dynsim_2link_planar_B.c_x_tmp].im =
        A[dynsim_2link_planar_B.d_re_tmp].im * dynsim_2link_planar_B.acoeff;
      dynsim_2link_planar_B.c_x_tmp++;
    }

    dynsim_2link_planar_B.work1[3 - dynsim_2link_planar_B.i_m].re = 1.0;
    dynsim_2link_planar_B.work1[3 - dynsim_2link_planar_B.i_m].im = 0.0;
    dynsim_2link_planar_B.c_x_tmp = static_cast<int32_T>(((-1.0 - ((-
      static_cast<real_T>(dynsim_2link_planar_B.i_m) + 4.0) - 1.0)) + 1.0) /
      -1.0);
    dynsim_2link_planar_B.c_j_j = 0;
    while (dynsim_2link_planar_B.c_j_j <= dynsim_2link_planar_B.c_x_tmp - 1) {
      dynsim_2link_planar_B.work2_idx_1_re_tmp = 2 - (dynsim_2link_planar_B.i_m
        + dynsim_2link_planar_B.c_j_j);
      dynsim_2link_planar_B.d_re_tmp_tmp =
        dynsim_2link_planar_B.work2_idx_1_re_tmp << 2;
      dynsim_2link_planar_B.d_re_tmp = dynsim_2link_planar_B.d_re_tmp_tmp +
        dynsim_2link_planar_B.work2_idx_1_re_tmp;
      dynsim_2link_planar_B.work2_idx_3_re = A[dynsim_2link_planar_B.d_re_tmp].
        re * dynsim_2link_planar_B.acoeff - dynsim_2link_planar_B.salpha_re;
      dynsim_2link_planar_B.scale_j = A[dynsim_2link_planar_B.d_re_tmp].im *
        dynsim_2link_planar_B.acoeff - dynsim_2link_planar_B.salpha_im;
      if (fabs(dynsim_2link_planar_B.work2_idx_3_re) + fabs
          (dynsim_2link_planar_B.scale_j) <= dynsim_2link_planar_B.dmin) {
        dynsim_2link_planar_B.work2_idx_3_re = dynsim_2link_planar_B.dmin;
        dynsim_2link_planar_B.scale_j = 0.0;
      }

      dynsim_2link_planar_B.work2_idx_2_im = fabs
        (dynsim_2link_planar_B.work2_idx_3_re);
      dynsim_2link_planar_B.work2_idx_3_im = fabs(dynsim_2link_planar_B.scale_j);
      dynsim_2link_planar_B.temp = dynsim_2link_planar_B.work2_idx_2_im +
        dynsim_2link_planar_B.work2_idx_3_im;
      if (dynsim_2link_planar_B.temp < 1.0) {
        dynsim_2link_planar_B.f_y = fabs
          (dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp]
           .re) + fabs
          (dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp]
           .im);
        if (dynsim_2link_planar_B.f_y >= dynsim_2link_planar_B.temp *
            1.1235582092889474E+307) {
          dynsim_2link_planar_B.temp = 1.0 / dynsim_2link_planar_B.f_y;
          dynsim_2link_planar_B.d_re_tmp = 0;
          while (dynsim_2link_planar_B.d_re_tmp <= 3 - dynsim_2link_planar_B.i_m)
          {
            dynsim_2link_planar_B.work1[dynsim_2link_planar_B.d_re_tmp].re *=
              dynsim_2link_planar_B.temp;
            dynsim_2link_planar_B.work1[dynsim_2link_planar_B.d_re_tmp].im *=
              dynsim_2link_planar_B.temp;
            dynsim_2link_planar_B.d_re_tmp++;
          }
        }
      }

      if (dynsim_2link_planar_B.scale_j == 0.0) {
        if (-dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp]
            .im == 0.0) {
          dynsim_2link_planar_B.temp =
            -dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp]
            .re / dynsim_2link_planar_B.work2_idx_3_re;
          dynsim_2link_planar_B.scale_j = 0.0;
        } else if
            (-dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp]
             .re == 0.0) {
          dynsim_2link_planar_B.temp = 0.0;
          dynsim_2link_planar_B.scale_j =
            -dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp]
            .im / dynsim_2link_planar_B.work2_idx_3_re;
        } else {
          dynsim_2link_planar_B.temp =
            -dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp]
            .re / dynsim_2link_planar_B.work2_idx_3_re;
          dynsim_2link_planar_B.scale_j =
            -dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp]
            .im / dynsim_2link_planar_B.work2_idx_3_re;
        }
      } else if (dynsim_2link_planar_B.work2_idx_3_re == 0.0) {
        if (-dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp]
            .re == 0.0) {
          dynsim_2link_planar_B.temp =
            -dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp]
            .im / dynsim_2link_planar_B.scale_j;
          dynsim_2link_planar_B.scale_j = 0.0;
        } else if
            (-dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp]
             .im == 0.0) {
          dynsim_2link_planar_B.temp = 0.0;
          dynsim_2link_planar_B.scale_j =
            -(-dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp]
              .re / dynsim_2link_planar_B.scale_j);
        } else {
          dynsim_2link_planar_B.temp =
            -dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp]
            .im / dynsim_2link_planar_B.scale_j;
          dynsim_2link_planar_B.scale_j =
            -(-dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp]
              .re / dynsim_2link_planar_B.scale_j);
        }
      } else if (dynsim_2link_planar_B.work2_idx_2_im >
                 dynsim_2link_planar_B.work2_idx_3_im) {
        dynsim_2link_planar_B.work2_idx_2_im = dynsim_2link_planar_B.scale_j /
          dynsim_2link_planar_B.work2_idx_3_re;
        dynsim_2link_planar_B.scale_j = dynsim_2link_planar_B.work2_idx_2_im *
          dynsim_2link_planar_B.scale_j + dynsim_2link_planar_B.work2_idx_3_re;
        dynsim_2link_planar_B.temp = (dynsim_2link_planar_B.work2_idx_2_im *
          -dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp]
          .im +
          -dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp]
          .re) / dynsim_2link_planar_B.scale_j;
        dynsim_2link_planar_B.scale_j =
          (-dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp]
           .im - dynsim_2link_planar_B.work2_idx_2_im *
           -dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp]
           .re) / dynsim_2link_planar_B.scale_j;
      } else if (dynsim_2link_planar_B.work2_idx_3_im ==
                 dynsim_2link_planar_B.work2_idx_2_im) {
        dynsim_2link_planar_B.work2_idx_3_re =
          dynsim_2link_planar_B.work2_idx_3_re > 0.0 ? 0.5 : -0.5;
        dynsim_2link_planar_B.scale_j = dynsim_2link_planar_B.scale_j > 0.0 ?
          0.5 : -0.5;
        dynsim_2link_planar_B.temp =
          (-dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp]
           .re * dynsim_2link_planar_B.work2_idx_3_re +
           -dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp]
           .im * dynsim_2link_planar_B.scale_j) /
          dynsim_2link_planar_B.work2_idx_2_im;
        dynsim_2link_planar_B.scale_j =
          (-dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp]
           .im * dynsim_2link_planar_B.work2_idx_3_re -
           -dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp]
           .re * dynsim_2link_planar_B.scale_j) /
          dynsim_2link_planar_B.work2_idx_2_im;
      } else {
        dynsim_2link_planar_B.work2_idx_2_im =
          dynsim_2link_planar_B.work2_idx_3_re / dynsim_2link_planar_B.scale_j;
        dynsim_2link_planar_B.scale_j += dynsim_2link_planar_B.work2_idx_2_im *
          dynsim_2link_planar_B.work2_idx_3_re;
        dynsim_2link_planar_B.temp = (dynsim_2link_planar_B.work2_idx_2_im *
          -dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp]
          .re +
          -dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp]
          .im) / dynsim_2link_planar_B.scale_j;
        dynsim_2link_planar_B.scale_j = (dynsim_2link_planar_B.work2_idx_2_im *
          -dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp]
          .im -
          (-dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp]
           .re)) / dynsim_2link_planar_B.scale_j;
      }

      dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp].re =
        dynsim_2link_planar_B.temp;
      dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp].im =
        dynsim_2link_planar_B.scale_j;
      if (dynsim_2link_planar_B.work2_idx_1_re_tmp + 1 > 1) {
        if (fabs
            (dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp]
             .re) + fabs
            (dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp]
             .im) > 1.0) {
          dynsim_2link_planar_B.temp = 1.0 / (fabs
            (dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp]
             .re) + fabs
            (dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp]
             .im));
          if (dynsim_2link_planar_B.acoeff *
              dynsim_2link_planar_B.rworka[dynsim_2link_planar_B.work2_idx_1_re_tmp]
              >= 1.1235582092889474E+307 * dynsim_2link_planar_B.temp) {
            dynsim_2link_planar_B.d_re_tmp = 0;
            while (dynsim_2link_planar_B.d_re_tmp <= 3 -
                   dynsim_2link_planar_B.i_m) {
              dynsim_2link_planar_B.work1[dynsim_2link_planar_B.d_re_tmp].re *=
                dynsim_2link_planar_B.temp;
              dynsim_2link_planar_B.work1[dynsim_2link_planar_B.d_re_tmp].im *=
                dynsim_2link_planar_B.temp;
              dynsim_2link_planar_B.d_re_tmp++;
            }
          }
        }

        dynsim_2link_planar_B.work2_idx_3_re = dynsim_2link_planar_B.acoeff *
          dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp].
          re;
        dynsim_2link_planar_B.scale_j = dynsim_2link_planar_B.acoeff *
          dynsim_2link_planar_B.work1[dynsim_2link_planar_B.work2_idx_1_re_tmp].
          im;
        dynsim_2link_planar_B.e_jr = 0;
        while (dynsim_2link_planar_B.e_jr <=
               dynsim_2link_planar_B.work2_idx_1_re_tmp - 1) {
          dynsim_2link_planar_B.d_re_tmp = dynsim_2link_planar_B.d_re_tmp_tmp +
            dynsim_2link_planar_B.e_jr;
          dynsim_2link_planar_B.work1[dynsim_2link_planar_B.e_jr].re +=
            A[dynsim_2link_planar_B.d_re_tmp].re *
            dynsim_2link_planar_B.work2_idx_3_re -
            A[dynsim_2link_planar_B.d_re_tmp].im * dynsim_2link_planar_B.scale_j;
          dynsim_2link_planar_B.work1[dynsim_2link_planar_B.e_jr].im +=
            A[dynsim_2link_planar_B.d_re_tmp].im *
            dynsim_2link_planar_B.work2_idx_3_re +
            A[dynsim_2link_planar_B.d_re_tmp].re * dynsim_2link_planar_B.scale_j;
          dynsim_2link_planar_B.e_jr++;
        }
      }

      dynsim_2link_planar_B.c_j_j++;
    }

    dynsim_2link_planar_B.salpha_re = 0.0;
    dynsim_2link_planar_B.salpha_im = 0.0;
    dynsim_2link_planar_B.acoeff = 0.0;
    dynsim_2link_planar_B.dmin = 0.0;
    dynsim_2link_planar_B.scale_j = 0.0;
    dynsim_2link_planar_B.work2_idx_2_im = 0.0;
    dynsim_2link_planar_B.work2_idx_3_re = 0.0;
    dynsim_2link_planar_B.work2_idx_3_im = 0.0;
    dynsim_2link_planar_B.c_x_tmp = 0;
    while (dynsim_2link_planar_B.c_x_tmp <= 3 - dynsim_2link_planar_B.i_m) {
      dynsim_2link_planar_B.c_j_j = dynsim_2link_planar_B.c_x_tmp << 2;
      dynsim_2link_planar_B.salpha_re += V[dynsim_2link_planar_B.c_j_j].re *
        dynsim_2link_planar_B.work1[dynsim_2link_planar_B.c_x_tmp].re -
        V[dynsim_2link_planar_B.c_j_j].im *
        dynsim_2link_planar_B.work1[dynsim_2link_planar_B.c_x_tmp].im;
      dynsim_2link_planar_B.salpha_im += V[dynsim_2link_planar_B.c_j_j].re *
        dynsim_2link_planar_B.work1[dynsim_2link_planar_B.c_x_tmp].im +
        V[dynsim_2link_planar_B.c_j_j].im *
        dynsim_2link_planar_B.work1[dynsim_2link_planar_B.c_x_tmp].re;
      dynsim_2link_planar_B.work2_idx_1_re_tmp = dynsim_2link_planar_B.c_j_j + 1;
      dynsim_2link_planar_B.acoeff += V[dynsim_2link_planar_B.work2_idx_1_re_tmp]
        .re * dynsim_2link_planar_B.work1[dynsim_2link_planar_B.c_x_tmp].re -
        V[dynsim_2link_planar_B.work2_idx_1_re_tmp].im *
        dynsim_2link_planar_B.work1[dynsim_2link_planar_B.c_x_tmp].im;
      dynsim_2link_planar_B.dmin += V[dynsim_2link_planar_B.work2_idx_1_re_tmp].
        re * dynsim_2link_planar_B.work1[dynsim_2link_planar_B.c_x_tmp].im +
        V[dynsim_2link_planar_B.work2_idx_1_re_tmp].im *
        dynsim_2link_planar_B.work1[dynsim_2link_planar_B.c_x_tmp].re;
      dynsim_2link_planar_B.work2_idx_1_re_tmp = dynsim_2link_planar_B.c_j_j + 2;
      dynsim_2link_planar_B.scale_j +=
        V[dynsim_2link_planar_B.work2_idx_1_re_tmp].re *
        dynsim_2link_planar_B.work1[dynsim_2link_planar_B.c_x_tmp].re -
        V[dynsim_2link_planar_B.work2_idx_1_re_tmp].im *
        dynsim_2link_planar_B.work1[dynsim_2link_planar_B.c_x_tmp].im;
      dynsim_2link_planar_B.work2_idx_2_im +=
        V[dynsim_2link_planar_B.work2_idx_1_re_tmp].re *
        dynsim_2link_planar_B.work1[dynsim_2link_planar_B.c_x_tmp].im +
        V[dynsim_2link_planar_B.work2_idx_1_re_tmp].im *
        dynsim_2link_planar_B.work1[dynsim_2link_planar_B.c_x_tmp].re;
      dynsim_2link_planar_B.c_j_j += 3;
      dynsim_2link_planar_B.work2_idx_3_re += V[dynsim_2link_planar_B.c_j_j].re *
        dynsim_2link_planar_B.work1[dynsim_2link_planar_B.c_x_tmp].re -
        V[dynsim_2link_planar_B.c_j_j].im *
        dynsim_2link_planar_B.work1[dynsim_2link_planar_B.c_x_tmp].im;
      dynsim_2link_planar_B.work2_idx_3_im += V[dynsim_2link_planar_B.c_j_j].re *
        dynsim_2link_planar_B.work1[dynsim_2link_planar_B.c_x_tmp].im +
        V[dynsim_2link_planar_B.c_j_j].im *
        dynsim_2link_planar_B.work1[dynsim_2link_planar_B.c_x_tmp].re;
      dynsim_2link_planar_B.c_x_tmp++;
    }

    dynsim_2link_planar_B.temp = fabs(dynsim_2link_planar_B.salpha_re) + fabs
      (dynsim_2link_planar_B.salpha_im);
    dynsim_2link_planar_B.f_y = fabs(dynsim_2link_planar_B.acoeff) + fabs
      (dynsim_2link_planar_B.dmin);
    if (dynsim_2link_planar_B.f_y > dynsim_2link_planar_B.temp) {
      dynsim_2link_planar_B.temp = dynsim_2link_planar_B.f_y;
    }

    dynsim_2link_planar_B.f_y = fabs(dynsim_2link_planar_B.scale_j) + fabs
      (dynsim_2link_planar_B.work2_idx_2_im);
    if (dynsim_2link_planar_B.f_y > dynsim_2link_planar_B.temp) {
      dynsim_2link_planar_B.temp = dynsim_2link_planar_B.f_y;
    }

    dynsim_2link_planar_B.f_y = fabs(dynsim_2link_planar_B.work2_idx_3_re) +
      fabs(dynsim_2link_planar_B.work2_idx_3_im);
    if (dynsim_2link_planar_B.f_y > dynsim_2link_planar_B.temp) {
      dynsim_2link_planar_B.temp = dynsim_2link_planar_B.f_y;
    }

    if (dynsim_2link_planar_B.temp > 2.2250738585072014E-308) {
      dynsim_2link_planar_B.temp = 1.0 / dynsim_2link_planar_B.temp;
      V[dynsim_2link_planar_B.c_x_tmp_tmp].re = dynsim_2link_planar_B.temp *
        dynsim_2link_planar_B.salpha_re;
      V[dynsim_2link_planar_B.c_x_tmp_tmp].im = dynsim_2link_planar_B.temp *
        dynsim_2link_planar_B.salpha_im;
      dynsim_2link_planar_B.d_re_tmp = ((3 - dynsim_2link_planar_B.i_m) << 2) +
        1;
      V[dynsim_2link_planar_B.d_re_tmp].re = dynsim_2link_planar_B.temp *
        dynsim_2link_planar_B.acoeff;
      V[dynsim_2link_planar_B.d_re_tmp].im = dynsim_2link_planar_B.temp *
        dynsim_2link_planar_B.dmin;
      dynsim_2link_planar_B.d_re_tmp = ((3 - dynsim_2link_planar_B.i_m) << 2) +
        2;
      V[dynsim_2link_planar_B.d_re_tmp].re = dynsim_2link_planar_B.temp *
        dynsim_2link_planar_B.scale_j;
      V[dynsim_2link_planar_B.d_re_tmp].im = dynsim_2link_planar_B.temp *
        dynsim_2link_planar_B.work2_idx_2_im;
      dynsim_2link_planar_B.d_re_tmp = ((3 - dynsim_2link_planar_B.i_m) << 2) +
        3;
      V[dynsim_2link_planar_B.d_re_tmp].re = dynsim_2link_planar_B.temp *
        dynsim_2link_planar_B.work2_idx_3_re;
      V[dynsim_2link_planar_B.d_re_tmp].im = dynsim_2link_planar_B.temp *
        dynsim_2link_planar_B.work2_idx_3_im;
    } else {
      V[dynsim_2link_planar_B.c_x_tmp_tmp].re = 0.0;
      V[dynsim_2link_planar_B.c_x_tmp_tmp].im = 0.0;
      dynsim_2link_planar_B.d_re_tmp = dynsim_2link_planar_B.c_x_tmp_tmp + 1;
      V[dynsim_2link_planar_B.d_re_tmp].re = 0.0;
      V[dynsim_2link_planar_B.d_re_tmp].im = 0.0;
      dynsim_2link_planar_B.d_re_tmp = dynsim_2link_planar_B.c_x_tmp_tmp + 2;
      V[dynsim_2link_planar_B.d_re_tmp].re = 0.0;
      V[dynsim_2link_planar_B.d_re_tmp].im = 0.0;
      dynsim_2link_planar_B.d_re_tmp = dynsim_2link_planar_B.c_x_tmp_tmp + 3;
      V[dynsim_2link_planar_B.d_re_tmp].re = 0.0;
      V[dynsim_2link_planar_B.d_re_tmp].im = 0.0;
    }
  }
}

// Function for MATLAB Function: '<S4>/MATLAB Function'
static void dynsim_2link_planar_xzggev(creal_T A[16], int32_T *info, creal_T
  alpha1[4], creal_T beta1[4], creal_T V[16])
{
  int32_T exitg1;
  int32_T exitg2;
  boolean_T exitg3;
  boolean_T exitg4;
  *info = 0;
  dynsim_2link_planar_B.anrm = dynsim_2link_planar_xzlangeM(A);
  if (rtIsInf(dynsim_2link_planar_B.anrm) || rtIsNaN(dynsim_2link_planar_B.anrm))
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
    for (dynsim_2link_planar_B.k_m = 0; dynsim_2link_planar_B.k_m < 16;
         dynsim_2link_planar_B.k_m++) {
      V[dynsim_2link_planar_B.k_m].re = (rtNaN);
      V[dynsim_2link_planar_B.k_m].im = 0.0;
    }
  } else {
    dynsim_2link_planar_B.ilascl = false;
    dynsim_2link_planar_B.anrmto = dynsim_2link_planar_B.anrm;
    if ((dynsim_2link_planar_B.anrm > 0.0) && (dynsim_2link_planar_B.anrm <
         6.7178761075670888E-139)) {
      dynsim_2link_planar_B.anrmto = 6.7178761075670888E-139;
      dynsim_2link_planar_B.ilascl = true;
      dynsim_2link_planar_xzlascl(dynsim_2link_planar_B.anrm,
        dynsim_2link_planar_B.anrmto, A);
    } else {
      if (dynsim_2link_planar_B.anrm > 1.4885657073574029E+138) {
        dynsim_2link_planar_B.anrmto = 1.4885657073574029E+138;
        dynsim_2link_planar_B.ilascl = true;
        dynsim_2link_planar_xzlascl(dynsim_2link_planar_B.anrm,
          dynsim_2link_planar_B.anrmto, A);
      }
    }

    dynsim_2link_planar_B.rscale[0] = 1;
    dynsim_2link_planar_B.rscale[1] = 1;
    dynsim_2link_planar_B.rscale[2] = 1;
    dynsim_2link_planar_B.rscale[3] = 1;
    dynsim_2link_planar_B.c_i = 0;
    dynsim_2link_planar_B.ihi = 4;
    do {
      exitg2 = 0;
      dynsim_2link_planar_B.i_e = 0;
      dynsim_2link_planar_B.jcol = 0;
      dynsim_2link_planar_B.found = false;
      dynsim_2link_planar_B.ii = dynsim_2link_planar_B.ihi;
      exitg3 = false;
      while ((!exitg3) && (dynsim_2link_planar_B.ii > 0)) {
        dynsim_2link_planar_B.nzcount = 0;
        dynsim_2link_planar_B.i_e = dynsim_2link_planar_B.ii;
        dynsim_2link_planar_B.jcol = dynsim_2link_planar_B.ihi;
        dynsim_2link_planar_B.jj = 0;
        exitg4 = false;
        while ((!exitg4) && (dynsim_2link_planar_B.jj <=
                             dynsim_2link_planar_B.ihi - 1)) {
          dynsim_2link_planar_B.k_m = ((dynsim_2link_planar_B.jj << 2) +
            dynsim_2link_planar_B.ii) - 1;
          if ((A[dynsim_2link_planar_B.k_m].re != 0.0) ||
              (A[dynsim_2link_planar_B.k_m].im != 0.0) ||
              (dynsim_2link_planar_B.jj + 1 == dynsim_2link_planar_B.ii)) {
            if (dynsim_2link_planar_B.nzcount == 0) {
              dynsim_2link_planar_B.jcol = dynsim_2link_planar_B.jj + 1;
              dynsim_2link_planar_B.nzcount = 1;
              dynsim_2link_planar_B.jj++;
            } else {
              dynsim_2link_planar_B.nzcount = 2;
              exitg4 = true;
            }
          } else {
            dynsim_2link_planar_B.jj++;
          }
        }

        if (dynsim_2link_planar_B.nzcount < 2) {
          dynsim_2link_planar_B.found = true;
          exitg3 = true;
        } else {
          dynsim_2link_planar_B.ii--;
        }
      }

      if (!dynsim_2link_planar_B.found) {
        exitg2 = 2;
      } else {
        if (dynsim_2link_planar_B.i_e != dynsim_2link_planar_B.ihi) {
          dynsim_2link_planar_B.atmp_re = A[dynsim_2link_planar_B.i_e - 1].re;
          dynsim_2link_planar_B.atmp_im = A[dynsim_2link_planar_B.i_e - 1].im;
          A[dynsim_2link_planar_B.i_e - 1] = A[dynsim_2link_planar_B.ihi - 1];
          A[dynsim_2link_planar_B.ihi - 1].re = dynsim_2link_planar_B.atmp_re;
          A[dynsim_2link_planar_B.ihi - 1].im = dynsim_2link_planar_B.atmp_im;
          dynsim_2link_planar_B.atmp_re = A[dynsim_2link_planar_B.i_e + 3].re;
          dynsim_2link_planar_B.atmp_im = A[dynsim_2link_planar_B.i_e + 3].im;
          A[dynsim_2link_planar_B.i_e + 3] = A[dynsim_2link_planar_B.ihi + 3];
          A[dynsim_2link_planar_B.ihi + 3].re = dynsim_2link_planar_B.atmp_re;
          A[dynsim_2link_planar_B.ihi + 3].im = dynsim_2link_planar_B.atmp_im;
          dynsim_2link_planar_B.atmp_re = A[dynsim_2link_planar_B.i_e + 7].re;
          dynsim_2link_planar_B.atmp_im = A[dynsim_2link_planar_B.i_e + 7].im;
          A[dynsim_2link_planar_B.i_e + 7] = A[dynsim_2link_planar_B.ihi + 7];
          A[dynsim_2link_planar_B.ihi + 7].re = dynsim_2link_planar_B.atmp_re;
          A[dynsim_2link_planar_B.ihi + 7].im = dynsim_2link_planar_B.atmp_im;
          dynsim_2link_planar_B.atmp_re = A[dynsim_2link_planar_B.i_e + 11].re;
          dynsim_2link_planar_B.atmp_im = A[dynsim_2link_planar_B.i_e + 11].im;
          A[dynsim_2link_planar_B.i_e + 11] = A[dynsim_2link_planar_B.ihi + 11];
          A[dynsim_2link_planar_B.ihi + 11].re = dynsim_2link_planar_B.atmp_re;
          A[dynsim_2link_planar_B.ihi + 11].im = dynsim_2link_planar_B.atmp_im;
        }

        if (dynsim_2link_planar_B.jcol != dynsim_2link_planar_B.ihi) {
          dynsim_2link_planar_B.ii = 0;
          while (dynsim_2link_planar_B.ii <= dynsim_2link_planar_B.ihi - 1) {
            dynsim_2link_planar_B.i_e = ((dynsim_2link_planar_B.jcol - 1) << 2)
              + dynsim_2link_planar_B.ii;
            dynsim_2link_planar_B.atmp_re = A[dynsim_2link_planar_B.i_e].re;
            dynsim_2link_planar_B.atmp_im = A[dynsim_2link_planar_B.i_e].im;
            dynsim_2link_planar_B.k_m = ((dynsim_2link_planar_B.ihi - 1) << 2) +
              dynsim_2link_planar_B.ii;
            A[dynsim_2link_planar_B.i_e] = A[dynsim_2link_planar_B.k_m];
            A[dynsim_2link_planar_B.k_m].re = dynsim_2link_planar_B.atmp_re;
            A[dynsim_2link_planar_B.k_m].im = dynsim_2link_planar_B.atmp_im;
            dynsim_2link_planar_B.ii++;
          }
        }

        dynsim_2link_planar_B.rscale[dynsim_2link_planar_B.ihi - 1] =
          dynsim_2link_planar_B.jcol;
        dynsim_2link_planar_B.ihi--;
        if (dynsim_2link_planar_B.ihi == 1) {
          dynsim_2link_planar_B.rscale[0] = 1;
          exitg2 = 1;
        }
      }
    } while (exitg2 == 0);

    if (exitg2 == 1) {
    } else {
      do {
        exitg1 = 0;
        dynsim_2link_planar_B.ii = 0;
        dynsim_2link_planar_B.jcol = 0;
        dynsim_2link_planar_B.found = false;
        dynsim_2link_planar_B.i_e = dynsim_2link_planar_B.c_i + 1;
        exitg3 = false;
        while ((!exitg3) && (dynsim_2link_planar_B.i_e <=
                             dynsim_2link_planar_B.ihi)) {
          dynsim_2link_planar_B.nzcount = 0;
          dynsim_2link_planar_B.ii = dynsim_2link_planar_B.ihi;
          dynsim_2link_planar_B.jcol = dynsim_2link_planar_B.i_e;
          dynsim_2link_planar_B.jj = dynsim_2link_planar_B.c_i + 1;
          exitg4 = false;
          while ((!exitg4) && (dynsim_2link_planar_B.jj <=
                               dynsim_2link_planar_B.ihi)) {
            dynsim_2link_planar_B.k_m = (((dynsim_2link_planar_B.i_e - 1) << 2)
              + dynsim_2link_planar_B.jj) - 1;
            if ((A[dynsim_2link_planar_B.k_m].re != 0.0) ||
                (A[dynsim_2link_planar_B.k_m].im != 0.0) ||
                (dynsim_2link_planar_B.jj == dynsim_2link_planar_B.i_e)) {
              if (dynsim_2link_planar_B.nzcount == 0) {
                dynsim_2link_planar_B.ii = dynsim_2link_planar_B.jj;
                dynsim_2link_planar_B.nzcount = 1;
                dynsim_2link_planar_B.jj++;
              } else {
                dynsim_2link_planar_B.nzcount = 2;
                exitg4 = true;
              }
            } else {
              dynsim_2link_planar_B.jj++;
            }
          }

          if (dynsim_2link_planar_B.nzcount < 2) {
            dynsim_2link_planar_B.found = true;
            exitg3 = true;
          } else {
            dynsim_2link_planar_B.i_e++;
          }
        }

        if (!dynsim_2link_planar_B.found) {
          exitg1 = 1;
        } else {
          if (dynsim_2link_planar_B.c_i + 1 != dynsim_2link_planar_B.ii) {
            dynsim_2link_planar_B.nzcount = dynsim_2link_planar_B.c_i;
            while (dynsim_2link_planar_B.nzcount + 1 < 5) {
              dynsim_2link_planar_B.k_m = dynsim_2link_planar_B.nzcount << 2;
              dynsim_2link_planar_B.i_e = (dynsim_2link_planar_B.k_m +
                dynsim_2link_planar_B.ii) - 1;
              dynsim_2link_planar_B.atmp_re = A[dynsim_2link_planar_B.i_e].re;
              dynsim_2link_planar_B.atmp_im = A[dynsim_2link_planar_B.i_e].im;
              dynsim_2link_planar_B.k_m += dynsim_2link_planar_B.c_i;
              A[dynsim_2link_planar_B.i_e] = A[dynsim_2link_planar_B.k_m];
              A[dynsim_2link_planar_B.k_m].re = dynsim_2link_planar_B.atmp_re;
              A[dynsim_2link_planar_B.k_m].im = dynsim_2link_planar_B.atmp_im;
              dynsim_2link_planar_B.nzcount++;
            }
          }

          if (dynsim_2link_planar_B.c_i + 1 != dynsim_2link_planar_B.jcol) {
            dynsim_2link_planar_B.ii = 0;
            while (dynsim_2link_planar_B.ii <= dynsim_2link_planar_B.ihi - 1) {
              dynsim_2link_planar_B.i_e = ((dynsim_2link_planar_B.jcol - 1) << 2)
                + dynsim_2link_planar_B.ii;
              dynsim_2link_planar_B.atmp_re = A[dynsim_2link_planar_B.i_e].re;
              dynsim_2link_planar_B.atmp_im = A[dynsim_2link_planar_B.i_e].im;
              dynsim_2link_planar_B.k_m = (dynsim_2link_planar_B.c_i << 2) +
                dynsim_2link_planar_B.ii;
              A[dynsim_2link_planar_B.i_e] = A[dynsim_2link_planar_B.k_m];
              A[dynsim_2link_planar_B.k_m].re = dynsim_2link_planar_B.atmp_re;
              A[dynsim_2link_planar_B.k_m].im = dynsim_2link_planar_B.atmp_im;
              dynsim_2link_planar_B.ii++;
            }
          }

          dynsim_2link_planar_B.rscale[dynsim_2link_planar_B.c_i] =
            dynsim_2link_planar_B.jcol;
          dynsim_2link_planar_B.c_i++;
          if (dynsim_2link_planar_B.c_i + 1 == dynsim_2link_planar_B.ihi) {
            dynsim_2link_planar_B.rscale[dynsim_2link_planar_B.c_i] =
              dynsim_2link_planar_B.c_i + 1;
            exitg1 = 1;
          }
        }
      } while (exitg1 == 0);
    }

    for (dynsim_2link_planar_B.k_m = 0; dynsim_2link_planar_B.k_m < 16;
         dynsim_2link_planar_B.k_m++) {
      dynsim_2link_planar_B.b_I_m[dynsim_2link_planar_B.k_m] = 0;
    }

    dynsim_2link_planar_B.b_I_m[0] = 1;
    dynsim_2link_planar_B.b_I_m[5] = 1;
    dynsim_2link_planar_B.b_I_m[10] = 1;
    dynsim_2link_planar_B.b_I_m[15] = 1;
    for (dynsim_2link_planar_B.k_m = 0; dynsim_2link_planar_B.k_m < 16;
         dynsim_2link_planar_B.k_m++) {
      V[dynsim_2link_planar_B.k_m].re =
        dynsim_2link_planar_B.b_I_m[dynsim_2link_planar_B.k_m];
      V[dynsim_2link_planar_B.k_m].im = 0.0;
    }

    if (dynsim_2link_planar_B.ihi >= dynsim_2link_planar_B.c_i + 3) {
      dynsim_2link_planar_B.jcol = dynsim_2link_planar_B.c_i;
      while (dynsim_2link_planar_B.jcol + 1 < dynsim_2link_planar_B.ihi - 1) {
        dynsim_2link_planar_B.ii = dynsim_2link_planar_B.ihi - 1;
        while (dynsim_2link_planar_B.ii + 1 > dynsim_2link_planar_B.jcol + 2) {
          dynsim_2link_planar_xzlartg(A[(dynsim_2link_planar_B.ii +
            (dynsim_2link_planar_B.jcol << 2)) - 1], A[dynsim_2link_planar_B.ii
            + (dynsim_2link_planar_B.jcol << 2)], &dynsim_2link_planar_B.mul,
            &dynsim_2link_planar_B.s, &A[(dynsim_2link_planar_B.ii +
            (dynsim_2link_planar_B.jcol << 2)) - 1]);
          dynsim_2link_planar_B.k_m = dynsim_2link_planar_B.ii +
            (dynsim_2link_planar_B.jcol << 2);
          A[dynsim_2link_planar_B.k_m].re = 0.0;
          A[dynsim_2link_planar_B.k_m].im = 0.0;
          dynsim_2link_planar_B.nzcount = dynsim_2link_planar_B.jcol + 1;
          while (dynsim_2link_planar_B.nzcount + 1 < 5) {
            dynsim_2link_planar_B.i_e = (dynsim_2link_planar_B.nzcount << 2) +
              dynsim_2link_planar_B.ii;
            dynsim_2link_planar_B.k_m = dynsim_2link_planar_B.i_e - 1;
            dynsim_2link_planar_B.atmp_re = A[dynsim_2link_planar_B.k_m].re *
              dynsim_2link_planar_B.mul + (A[dynsim_2link_planar_B.i_e].re *
              dynsim_2link_planar_B.s.re - A[dynsim_2link_planar_B.i_e].im *
              dynsim_2link_planar_B.s.im);
            dynsim_2link_planar_B.atmp_im = A[dynsim_2link_planar_B.k_m].im *
              dynsim_2link_planar_B.mul + (A[dynsim_2link_planar_B.i_e].im *
              dynsim_2link_planar_B.s.re + A[dynsim_2link_planar_B.i_e].re *
              dynsim_2link_planar_B.s.im);
            dynsim_2link_planar_B.d_p = A[dynsim_2link_planar_B.k_m].im;
            dynsim_2link_planar_B.d1 = A[dynsim_2link_planar_B.k_m].re;
            A[dynsim_2link_planar_B.i_e].re = A[dynsim_2link_planar_B.i_e].re *
              dynsim_2link_planar_B.mul - (A[dynsim_2link_planar_B.k_m].re *
              dynsim_2link_planar_B.s.re + A[dynsim_2link_planar_B.k_m].im *
              dynsim_2link_planar_B.s.im);
            A[dynsim_2link_planar_B.i_e].im = A[dynsim_2link_planar_B.i_e].im *
              dynsim_2link_planar_B.mul - (dynsim_2link_planar_B.s.re *
              dynsim_2link_planar_B.d_p - dynsim_2link_planar_B.s.im *
              dynsim_2link_planar_B.d1);
            A[dynsim_2link_planar_B.k_m].re = dynsim_2link_planar_B.atmp_re;
            A[dynsim_2link_planar_B.k_m].im = dynsim_2link_planar_B.atmp_im;
            dynsim_2link_planar_B.nzcount++;
          }

          dynsim_2link_planar_B.s.re = -dynsim_2link_planar_B.s.re;
          dynsim_2link_planar_B.s.im = -dynsim_2link_planar_B.s.im;
          dynsim_2link_planar_B.nzcount = 0;
          while (dynsim_2link_planar_B.nzcount + 1 <= dynsim_2link_planar_B.ihi)
          {
            dynsim_2link_planar_B.i_e = ((dynsim_2link_planar_B.ii - 1) << 2) +
              dynsim_2link_planar_B.nzcount;
            dynsim_2link_planar_B.k_m = (dynsim_2link_planar_B.ii << 2) +
              dynsim_2link_planar_B.nzcount;
            dynsim_2link_planar_B.atmp_re = (A[dynsim_2link_planar_B.i_e].re *
              dynsim_2link_planar_B.s.re - A[dynsim_2link_planar_B.i_e].im *
              dynsim_2link_planar_B.s.im) + A[dynsim_2link_planar_B.k_m].re *
              dynsim_2link_planar_B.mul;
            dynsim_2link_planar_B.atmp_im = (A[dynsim_2link_planar_B.i_e].im *
              dynsim_2link_planar_B.s.re + A[dynsim_2link_planar_B.i_e].re *
              dynsim_2link_planar_B.s.im) + A[dynsim_2link_planar_B.k_m].im *
              dynsim_2link_planar_B.mul;
            dynsim_2link_planar_B.d_p = A[dynsim_2link_planar_B.k_m].im;
            dynsim_2link_planar_B.d1 = A[dynsim_2link_planar_B.k_m].re;
            A[dynsim_2link_planar_B.i_e].re = A[dynsim_2link_planar_B.i_e].re *
              dynsim_2link_planar_B.mul - (A[dynsim_2link_planar_B.k_m].re *
              dynsim_2link_planar_B.s.re + A[dynsim_2link_planar_B.k_m].im *
              dynsim_2link_planar_B.s.im);
            A[dynsim_2link_planar_B.i_e].im = A[dynsim_2link_planar_B.i_e].im *
              dynsim_2link_planar_B.mul - (dynsim_2link_planar_B.s.re *
              dynsim_2link_planar_B.d_p - dynsim_2link_planar_B.s.im *
              dynsim_2link_planar_B.d1);
            A[dynsim_2link_planar_B.k_m].re = dynsim_2link_planar_B.atmp_re;
            A[dynsim_2link_planar_B.k_m].im = dynsim_2link_planar_B.atmp_im;
            dynsim_2link_planar_B.nzcount++;
          }

          dynsim_2link_planar_B.i_e = (dynsim_2link_planar_B.ii - 1) << 2;
          dynsim_2link_planar_B.k_m = dynsim_2link_planar_B.ii << 2;
          dynsim_2link_planar_B.atmp_re = (V[dynsim_2link_planar_B.i_e].re *
            dynsim_2link_planar_B.s.re - V[dynsim_2link_planar_B.i_e].im *
            dynsim_2link_planar_B.s.im) + V[dynsim_2link_planar_B.k_m].re *
            dynsim_2link_planar_B.mul;
          dynsim_2link_planar_B.atmp_im = (V[dynsim_2link_planar_B.i_e].im *
            dynsim_2link_planar_B.s.re + V[dynsim_2link_planar_B.i_e].re *
            dynsim_2link_planar_B.s.im) + V[dynsim_2link_planar_B.k_m].im *
            dynsim_2link_planar_B.mul;
          dynsim_2link_planar_B.d_p = V[dynsim_2link_planar_B.k_m].re;
          V[dynsim_2link_planar_B.i_e].re = V[dynsim_2link_planar_B.i_e].re *
            dynsim_2link_planar_B.mul - (V[dynsim_2link_planar_B.k_m].re *
            dynsim_2link_planar_B.s.re + V[dynsim_2link_planar_B.k_m].im *
            dynsim_2link_planar_B.s.im);
          V[dynsim_2link_planar_B.i_e].im = V[dynsim_2link_planar_B.i_e].im *
            dynsim_2link_planar_B.mul - (V[dynsim_2link_planar_B.k_m].im *
            dynsim_2link_planar_B.s.re - dynsim_2link_planar_B.s.im *
            dynsim_2link_planar_B.d_p);
          V[dynsim_2link_planar_B.k_m].re = dynsim_2link_planar_B.atmp_re;
          V[dynsim_2link_planar_B.k_m].im = dynsim_2link_planar_B.atmp_im;
          dynsim_2link_planar_B.nzcount = dynsim_2link_planar_B.i_e + 1;
          dynsim_2link_planar_B.jj = dynsim_2link_planar_B.k_m + 1;
          dynsim_2link_planar_B.atmp_re = (V[dynsim_2link_planar_B.nzcount].re *
            dynsim_2link_planar_B.s.re - V[dynsim_2link_planar_B.nzcount].im *
            dynsim_2link_planar_B.s.im) + V[dynsim_2link_planar_B.jj].re *
            dynsim_2link_planar_B.mul;
          dynsim_2link_planar_B.atmp_im = (V[dynsim_2link_planar_B.nzcount].im *
            dynsim_2link_planar_B.s.re + V[dynsim_2link_planar_B.nzcount].re *
            dynsim_2link_planar_B.s.im) + V[dynsim_2link_planar_B.jj].im *
            dynsim_2link_planar_B.mul;
          dynsim_2link_planar_B.d_p = V[dynsim_2link_planar_B.jj].re;
          V[dynsim_2link_planar_B.nzcount].re = V[dynsim_2link_planar_B.nzcount]
            .re * dynsim_2link_planar_B.mul - (V[dynsim_2link_planar_B.jj].re *
            dynsim_2link_planar_B.s.re + V[dynsim_2link_planar_B.jj].im *
            dynsim_2link_planar_B.s.im);
          V[dynsim_2link_planar_B.nzcount].im = V[dynsim_2link_planar_B.nzcount]
            .im * dynsim_2link_planar_B.mul - (V[dynsim_2link_planar_B.jj].im *
            dynsim_2link_planar_B.s.re - dynsim_2link_planar_B.s.im *
            dynsim_2link_planar_B.d_p);
          V[dynsim_2link_planar_B.jj].re = dynsim_2link_planar_B.atmp_re;
          V[dynsim_2link_planar_B.jj].im = dynsim_2link_planar_B.atmp_im;
          dynsim_2link_planar_B.nzcount = dynsim_2link_planar_B.i_e + 2;
          dynsim_2link_planar_B.jj = dynsim_2link_planar_B.k_m + 2;
          dynsim_2link_planar_B.atmp_re = (V[dynsim_2link_planar_B.nzcount].re *
            dynsim_2link_planar_B.s.re - V[dynsim_2link_planar_B.nzcount].im *
            dynsim_2link_planar_B.s.im) + V[dynsim_2link_planar_B.jj].re *
            dynsim_2link_planar_B.mul;
          dynsim_2link_planar_B.atmp_im = (V[dynsim_2link_planar_B.nzcount].im *
            dynsim_2link_planar_B.s.re + V[dynsim_2link_planar_B.nzcount].re *
            dynsim_2link_planar_B.s.im) + V[dynsim_2link_planar_B.jj].im *
            dynsim_2link_planar_B.mul;
          dynsim_2link_planar_B.d_p = V[dynsim_2link_planar_B.jj].re;
          V[dynsim_2link_planar_B.nzcount].re = V[dynsim_2link_planar_B.nzcount]
            .re * dynsim_2link_planar_B.mul - (V[dynsim_2link_planar_B.jj].re *
            dynsim_2link_planar_B.s.re + V[dynsim_2link_planar_B.jj].im *
            dynsim_2link_planar_B.s.im);
          V[dynsim_2link_planar_B.nzcount].im = V[dynsim_2link_planar_B.nzcount]
            .im * dynsim_2link_planar_B.mul - (V[dynsim_2link_planar_B.jj].im *
            dynsim_2link_planar_B.s.re - dynsim_2link_planar_B.s.im *
            dynsim_2link_planar_B.d_p);
          V[dynsim_2link_planar_B.jj].re = dynsim_2link_planar_B.atmp_re;
          V[dynsim_2link_planar_B.jj].im = dynsim_2link_planar_B.atmp_im;
          dynsim_2link_planar_B.i_e += 3;
          dynsim_2link_planar_B.k_m += 3;
          dynsim_2link_planar_B.atmp_re = (V[dynsim_2link_planar_B.i_e].re *
            dynsim_2link_planar_B.s.re - V[dynsim_2link_planar_B.i_e].im *
            dynsim_2link_planar_B.s.im) + V[dynsim_2link_planar_B.k_m].re *
            dynsim_2link_planar_B.mul;
          dynsim_2link_planar_B.atmp_im = (V[dynsim_2link_planar_B.i_e].im *
            dynsim_2link_planar_B.s.re + V[dynsim_2link_planar_B.i_e].re *
            dynsim_2link_planar_B.s.im) + V[dynsim_2link_planar_B.k_m].im *
            dynsim_2link_planar_B.mul;
          dynsim_2link_planar_B.d_p = V[dynsim_2link_planar_B.k_m].re;
          V[dynsim_2link_planar_B.i_e].re = V[dynsim_2link_planar_B.i_e].re *
            dynsim_2link_planar_B.mul - (V[dynsim_2link_planar_B.k_m].re *
            dynsim_2link_planar_B.s.re + V[dynsim_2link_planar_B.k_m].im *
            dynsim_2link_planar_B.s.im);
          V[dynsim_2link_planar_B.i_e].im = V[dynsim_2link_planar_B.i_e].im *
            dynsim_2link_planar_B.mul - (V[dynsim_2link_planar_B.k_m].im *
            dynsim_2link_planar_B.s.re - dynsim_2link_planar_B.s.im *
            dynsim_2link_planar_B.d_p);
          V[dynsim_2link_planar_B.k_m].re = dynsim_2link_planar_B.atmp_re;
          V[dynsim_2link_planar_B.k_m].im = dynsim_2link_planar_B.atmp_im;
          dynsim_2link_planar_B.ii--;
        }

        dynsim_2link_planar_B.jcol++;
      }
    }

    dynsim_2link_planar_xzhgeqz(A, dynsim_2link_planar_B.c_i + 1,
      dynsim_2link_planar_B.ihi, V, info, alpha1, beta1);
    if (*info == 0) {
      dynsim_2link_planar_xztgevc(A, V);
      if (dynsim_2link_planar_B.c_i + 1 > 1) {
        dynsim_2link_planar_B.c_i--;
        while (dynsim_2link_planar_B.c_i + 1 >= 1) {
          dynsim_2link_planar_B.k_m =
            dynsim_2link_planar_B.rscale[dynsim_2link_planar_B.c_i] - 1;
          if (dynsim_2link_planar_B.c_i + 1 !=
              dynsim_2link_planar_B.rscale[dynsim_2link_planar_B.c_i]) {
            dynsim_2link_planar_B.atmp_re = V[dynsim_2link_planar_B.c_i].re;
            dynsim_2link_planar_B.atmp_im = V[dynsim_2link_planar_B.c_i].im;
            V[dynsim_2link_planar_B.c_i] = V[dynsim_2link_planar_B.k_m];
            V[dynsim_2link_planar_B.k_m].re = dynsim_2link_planar_B.atmp_re;
            V[dynsim_2link_planar_B.k_m].im = dynsim_2link_planar_B.atmp_im;
            dynsim_2link_planar_B.atmp_re = V[dynsim_2link_planar_B.c_i + 4].re;
            dynsim_2link_planar_B.atmp_im = V[dynsim_2link_planar_B.c_i + 4].im;
            V[dynsim_2link_planar_B.c_i + 4] = V[dynsim_2link_planar_B.k_m + 4];
            V[dynsim_2link_planar_B.k_m + 4].re = dynsim_2link_planar_B.atmp_re;
            V[dynsim_2link_planar_B.k_m + 4].im = dynsim_2link_planar_B.atmp_im;
            dynsim_2link_planar_B.atmp_re = V[dynsim_2link_planar_B.c_i + 8].re;
            dynsim_2link_planar_B.atmp_im = V[dynsim_2link_planar_B.c_i + 8].im;
            V[dynsim_2link_planar_B.c_i + 8] = V[dynsim_2link_planar_B.k_m + 8];
            V[dynsim_2link_planar_B.k_m + 8].re = dynsim_2link_planar_B.atmp_re;
            V[dynsim_2link_planar_B.k_m + 8].im = dynsim_2link_planar_B.atmp_im;
            dynsim_2link_planar_B.atmp_re = V[dynsim_2link_planar_B.c_i + 12].re;
            dynsim_2link_planar_B.atmp_im = V[dynsim_2link_planar_B.c_i + 12].im;
            V[dynsim_2link_planar_B.c_i + 12] = V[dynsim_2link_planar_B.k_m + 12];
            V[dynsim_2link_planar_B.k_m + 12].re = dynsim_2link_planar_B.atmp_re;
            V[dynsim_2link_planar_B.k_m + 12].im = dynsim_2link_planar_B.atmp_im;
          }

          dynsim_2link_planar_B.c_i--;
        }
      }

      if (dynsim_2link_planar_B.ihi < 4) {
        while (dynsim_2link_planar_B.ihi + 1 < 5) {
          dynsim_2link_planar_B.k_m =
            dynsim_2link_planar_B.rscale[dynsim_2link_planar_B.ihi] - 1;
          if (dynsim_2link_planar_B.ihi + 1 !=
              dynsim_2link_planar_B.rscale[dynsim_2link_planar_B.ihi]) {
            dynsim_2link_planar_B.atmp_re = V[dynsim_2link_planar_B.ihi].re;
            dynsim_2link_planar_B.atmp_im = V[dynsim_2link_planar_B.ihi].im;
            V[dynsim_2link_planar_B.ihi] = V[dynsim_2link_planar_B.k_m];
            V[dynsim_2link_planar_B.k_m].re = dynsim_2link_planar_B.atmp_re;
            V[dynsim_2link_planar_B.k_m].im = dynsim_2link_planar_B.atmp_im;
            dynsim_2link_planar_B.atmp_re = V[dynsim_2link_planar_B.ihi + 4].re;
            dynsim_2link_planar_B.atmp_im = V[dynsim_2link_planar_B.ihi + 4].im;
            V[dynsim_2link_planar_B.ihi + 4] = V[dynsim_2link_planar_B.k_m + 4];
            V[dynsim_2link_planar_B.k_m + 4].re = dynsim_2link_planar_B.atmp_re;
            V[dynsim_2link_planar_B.k_m + 4].im = dynsim_2link_planar_B.atmp_im;
            dynsim_2link_planar_B.atmp_re = V[dynsim_2link_planar_B.ihi + 8].re;
            dynsim_2link_planar_B.atmp_im = V[dynsim_2link_planar_B.ihi + 8].im;
            V[dynsim_2link_planar_B.ihi + 8] = V[dynsim_2link_planar_B.k_m + 8];
            V[dynsim_2link_planar_B.k_m + 8].re = dynsim_2link_planar_B.atmp_re;
            V[dynsim_2link_planar_B.k_m + 8].im = dynsim_2link_planar_B.atmp_im;
            dynsim_2link_planar_B.atmp_re = V[dynsim_2link_planar_B.ihi + 12].re;
            dynsim_2link_planar_B.atmp_im = V[dynsim_2link_planar_B.ihi + 12].im;
            V[dynsim_2link_planar_B.ihi + 12] = V[dynsim_2link_planar_B.k_m + 12];
            V[dynsim_2link_planar_B.k_m + 12].re = dynsim_2link_planar_B.atmp_re;
            V[dynsim_2link_planar_B.k_m + 12].im = dynsim_2link_planar_B.atmp_im;
          }

          dynsim_2link_planar_B.ihi++;
        }
      }

      for (dynsim_2link_planar_B.ihi = 0; dynsim_2link_planar_B.ihi < 4;
           dynsim_2link_planar_B.ihi++) {
        dynsim_2link_planar_B.c_i = dynsim_2link_planar_B.ihi << 2;
        dynsim_2link_planar_B.atmp_re = fabs(V[dynsim_2link_planar_B.c_i].re) +
          fabs(V[dynsim_2link_planar_B.c_i].im);
        dynsim_2link_planar_B.k_m = dynsim_2link_planar_B.c_i + 1;
        dynsim_2link_planar_B.atmp_im = fabs(V[dynsim_2link_planar_B.k_m].re) +
          fabs(V[dynsim_2link_planar_B.k_m].im);
        if (dynsim_2link_planar_B.atmp_im > dynsim_2link_planar_B.atmp_re) {
          dynsim_2link_planar_B.atmp_re = dynsim_2link_planar_B.atmp_im;
        }

        dynsim_2link_planar_B.i_e = dynsim_2link_planar_B.c_i + 2;
        dynsim_2link_planar_B.atmp_im = fabs(V[dynsim_2link_planar_B.i_e].re) +
          fabs(V[dynsim_2link_planar_B.i_e].im);
        if (dynsim_2link_planar_B.atmp_im > dynsim_2link_planar_B.atmp_re) {
          dynsim_2link_planar_B.atmp_re = dynsim_2link_planar_B.atmp_im;
        }

        dynsim_2link_planar_B.jcol = dynsim_2link_planar_B.c_i + 3;
        dynsim_2link_planar_B.atmp_im = fabs(V[dynsim_2link_planar_B.jcol].re) +
          fabs(V[dynsim_2link_planar_B.jcol].im);
        if (dynsim_2link_planar_B.atmp_im > dynsim_2link_planar_B.atmp_re) {
          dynsim_2link_planar_B.atmp_re = dynsim_2link_planar_B.atmp_im;
        }

        if (dynsim_2link_planar_B.atmp_re >= 6.7178761075670888E-139) {
          dynsim_2link_planar_B.atmp_re = 1.0 / dynsim_2link_planar_B.atmp_re;
          V[dynsim_2link_planar_B.c_i].re *= dynsim_2link_planar_B.atmp_re;
          V[dynsim_2link_planar_B.c_i].im *= dynsim_2link_planar_B.atmp_re;
          V[dynsim_2link_planar_B.k_m].re *= dynsim_2link_planar_B.atmp_re;
          V[dynsim_2link_planar_B.k_m].im *= dynsim_2link_planar_B.atmp_re;
          V[dynsim_2link_planar_B.i_e].re *= dynsim_2link_planar_B.atmp_re;
          V[dynsim_2link_planar_B.i_e].im *= dynsim_2link_planar_B.atmp_re;
          V[dynsim_2link_planar_B.jcol].re *= dynsim_2link_planar_B.atmp_re;
          V[dynsim_2link_planar_B.jcol].im *= dynsim_2link_planar_B.atmp_re;
        }
      }

      if (dynsim_2link_planar_B.ilascl) {
        dynsim_2link_planar_B.ilascl = true;
        while (dynsim_2link_planar_B.ilascl) {
          dynsim_2link_planar_B.atmp_re = dynsim_2link_planar_B.anrmto *
            2.0041683600089728E-292;
          dynsim_2link_planar_B.atmp_im = dynsim_2link_planar_B.anrm /
            4.9896007738368E+291;
          if ((fabs(dynsim_2link_planar_B.atmp_re) > fabs
               (dynsim_2link_planar_B.anrm)) && (dynsim_2link_planar_B.anrm !=
               0.0)) {
            dynsim_2link_planar_B.mul = 2.0041683600089728E-292;
            dynsim_2link_planar_B.anrmto = dynsim_2link_planar_B.atmp_re;
          } else if (fabs(dynsim_2link_planar_B.atmp_im) > fabs
                     (dynsim_2link_planar_B.anrmto)) {
            dynsim_2link_planar_B.mul = 4.9896007738368E+291;
            dynsim_2link_planar_B.anrm = dynsim_2link_planar_B.atmp_im;
          } else {
            dynsim_2link_planar_B.mul = dynsim_2link_planar_B.anrm /
              dynsim_2link_planar_B.anrmto;
            dynsim_2link_planar_B.ilascl = false;
          }

          alpha1[0].re *= dynsim_2link_planar_B.mul;
          alpha1[0].im *= dynsim_2link_planar_B.mul;
          alpha1[1].re *= dynsim_2link_planar_B.mul;
          alpha1[1].im *= dynsim_2link_planar_B.mul;
          alpha1[2].re *= dynsim_2link_planar_B.mul;
          alpha1[2].im *= dynsim_2link_planar_B.mul;
          alpha1[3].re *= dynsim_2link_planar_B.mul;
          alpha1[3].im *= dynsim_2link_planar_B.mul;
        }
      }
    }
  }
}

// Function for MATLAB Function: '<S4>/MATLAB Function'
static real_T dynsim_2link_planar_xnrm2(int32_T n, const real_T x[16], int32_T
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
static void dynsim_2link_planar_xzlarf(int32_T m, int32_T n, int32_T iv0, real_T
  tau, real_T C[16], int32_T ic0, real_T work[4])
{
  int32_T exitg1;
  boolean_T exitg2;
  if (tau != 0.0) {
    dynsim_2link_planar_B.lastv = m;
    dynsim_2link_planar_B.lastc_m = iv0 + m;
    while ((dynsim_2link_planar_B.lastv > 0) && (C[dynsim_2link_planar_B.lastc_m
            - 2] == 0.0)) {
      dynsim_2link_planar_B.lastv--;
      dynsim_2link_planar_B.lastc_m--;
    }

    dynsim_2link_planar_B.lastc_m = n - 1;
    exitg2 = false;
    while ((!exitg2) && (dynsim_2link_planar_B.lastc_m + 1 > 0)) {
      dynsim_2link_planar_B.coltop = (dynsim_2link_planar_B.lastc_m << 2) + ic0;
      dynsim_2link_planar_B.jy_c = dynsim_2link_planar_B.coltop;
      do {
        exitg1 = 0;
        if (dynsim_2link_planar_B.jy_c <= (dynsim_2link_planar_B.coltop +
             dynsim_2link_planar_B.lastv) - 1) {
          if (C[dynsim_2link_planar_B.jy_c - 1] != 0.0) {
            exitg1 = 1;
          } else {
            dynsim_2link_planar_B.jy_c++;
          }
        } else {
          dynsim_2link_planar_B.lastc_m--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    dynsim_2link_planar_B.lastv = 0;
    dynsim_2link_planar_B.lastc_m = -1;
  }

  if (dynsim_2link_planar_B.lastv > 0) {
    if (dynsim_2link_planar_B.lastc_m + 1 != 0) {
      dynsim_2link_planar_B.coltop = 0;
      while (dynsim_2link_planar_B.coltop <= dynsim_2link_planar_B.lastc_m) {
        work[dynsim_2link_planar_B.coltop] = 0.0;
        dynsim_2link_planar_B.coltop++;
      }

      dynsim_2link_planar_B.coltop = 0;
      dynsim_2link_planar_B.jy_c = (dynsim_2link_planar_B.lastc_m << 2) + ic0;
      dynsim_2link_planar_B.iac_d = ic0;
      while (dynsim_2link_planar_B.iac_d <= dynsim_2link_planar_B.jy_c) {
        dynsim_2link_planar_B.ix_p = iv0;
        dynsim_2link_planar_B.c_e = 0.0;
        dynsim_2link_planar_B.d_g = (dynsim_2link_planar_B.iac_d +
          dynsim_2link_planar_B.lastv) - 1;
        dynsim_2link_planar_B.b_ia_c = dynsim_2link_planar_B.iac_d;
        while (dynsim_2link_planar_B.b_ia_c <= dynsim_2link_planar_B.d_g) {
          dynsim_2link_planar_B.c_e += C[dynsim_2link_planar_B.b_ia_c - 1] *
            C[dynsim_2link_planar_B.ix_p - 1];
          dynsim_2link_planar_B.ix_p++;
          dynsim_2link_planar_B.b_ia_c++;
        }

        work[dynsim_2link_planar_B.coltop] += dynsim_2link_planar_B.c_e;
        dynsim_2link_planar_B.coltop++;
        dynsim_2link_planar_B.iac_d += 4;
      }
    }

    if (!(-tau == 0.0)) {
      dynsim_2link_planar_B.coltop = ic0 - 1;
      dynsim_2link_planar_B.jy_c = 0;
      dynsim_2link_planar_B.iac_d = 0;
      while (dynsim_2link_planar_B.iac_d <= dynsim_2link_planar_B.lastc_m) {
        if (work[dynsim_2link_planar_B.jy_c] != 0.0) {
          dynsim_2link_planar_B.c_e = work[dynsim_2link_planar_B.jy_c] * -tau;
          dynsim_2link_planar_B.ix_p = iv0;
          dynsim_2link_planar_B.d_g = dynsim_2link_planar_B.lastv +
            dynsim_2link_planar_B.coltop;
          dynsim_2link_planar_B.b_ia_c = dynsim_2link_planar_B.coltop;
          while (dynsim_2link_planar_B.b_ia_c + 1 <= dynsim_2link_planar_B.d_g)
          {
            C[dynsim_2link_planar_B.b_ia_c] += C[dynsim_2link_planar_B.ix_p - 1]
              * dynsim_2link_planar_B.c_e;
            dynsim_2link_planar_B.ix_p++;
            dynsim_2link_planar_B.b_ia_c++;
          }
        }

        dynsim_2link_planar_B.jy_c++;
        dynsim_2link_planar_B.coltop += 4;
        dynsim_2link_planar_B.iac_d++;
      }
    }
  }
}

// Function for MATLAB Function: '<S4>/MATLAB Function'
static void dynsim_2link_planar_xgehrd(real_T a[16], real_T tau[3])
{
  int32_T exitg1;
  boolean_T exitg2;
  dynsim_2link_planar_B.work_j[0] = 0.0;
  dynsim_2link_planar_B.work_j[1] = 0.0;
  dynsim_2link_planar_B.work_j[2] = 0.0;
  dynsim_2link_planar_B.work_j[3] = 0.0;
  dynsim_2link_planar_B.alpha1 = a[1];
  tau[0] = 0.0;
  dynsim_2link_planar_B.xnorm = dynsim_2link_planar_xnrm2(2, a, 3);
  if (dynsim_2link_planar_B.xnorm != 0.0) {
    dynsim_2link_planar_B.xnorm = dynsim_2link_plan_rt_hypotd_snf(a[1],
      dynsim_2link_planar_B.xnorm);
    if (a[1] >= 0.0) {
      dynsim_2link_planar_B.xnorm = -dynsim_2link_planar_B.xnorm;
    }

    if (fabs(dynsim_2link_planar_B.xnorm) < 1.0020841800044864E-292) {
      dynsim_2link_planar_B.knt = -1;
      do {
        dynsim_2link_planar_B.knt++;
        dynsim_2link_planar_B.lastc = 3;
        while (dynsim_2link_planar_B.lastc <= 4) {
          a[dynsim_2link_planar_B.lastc - 1] *= 9.9792015476736E+291;
          dynsim_2link_planar_B.lastc++;
        }

        dynsim_2link_planar_B.xnorm *= 9.9792015476736E+291;
        dynsim_2link_planar_B.alpha1 *= 9.9792015476736E+291;
      } while (!(fabs(dynsim_2link_planar_B.xnorm) >= 1.0020841800044864E-292));

      dynsim_2link_planar_B.xnorm = dynsim_2link_plan_rt_hypotd_snf
        (dynsim_2link_planar_B.alpha1, dynsim_2link_planar_xnrm2(2, a, 3));
      if (dynsim_2link_planar_B.alpha1 >= 0.0) {
        dynsim_2link_planar_B.xnorm = -dynsim_2link_planar_B.xnorm;
      }

      tau[0] = (dynsim_2link_planar_B.xnorm - dynsim_2link_planar_B.alpha1) /
        dynsim_2link_planar_B.xnorm;
      dynsim_2link_planar_B.alpha1 = 1.0 / (dynsim_2link_planar_B.alpha1 -
        dynsim_2link_planar_B.xnorm);
      dynsim_2link_planar_B.lastc = 3;
      while (dynsim_2link_planar_B.lastc <= 4) {
        a[dynsim_2link_planar_B.lastc - 1] *= dynsim_2link_planar_B.alpha1;
        dynsim_2link_planar_B.lastc++;
      }

      dynsim_2link_planar_B.lastc = 0;
      while (dynsim_2link_planar_B.lastc <= dynsim_2link_planar_B.knt) {
        dynsim_2link_planar_B.xnorm *= 1.0020841800044864E-292;
        dynsim_2link_planar_B.lastc++;
      }

      dynsim_2link_planar_B.alpha1 = dynsim_2link_planar_B.xnorm;
    } else {
      tau[0] = (dynsim_2link_planar_B.xnorm - a[1]) /
        dynsim_2link_planar_B.xnorm;
      dynsim_2link_planar_B.alpha1 = 1.0 / (a[1] - dynsim_2link_planar_B.xnorm);
      dynsim_2link_planar_B.knt = 3;
      while (dynsim_2link_planar_B.knt <= 4) {
        a[dynsim_2link_planar_B.knt - 1] *= dynsim_2link_planar_B.alpha1;
        dynsim_2link_planar_B.knt++;
      }

      dynsim_2link_planar_B.alpha1 = dynsim_2link_planar_B.xnorm;
    }
  }

  a[1] = 1.0;
  if (tau[0] != 0.0) {
    dynsim_2link_planar_B.knt = 2;
    dynsim_2link_planar_B.lastc = 3;
    while ((dynsim_2link_planar_B.knt + 1 > 0) && (a[dynsim_2link_planar_B.lastc]
            == 0.0)) {
      dynsim_2link_planar_B.knt--;
      dynsim_2link_planar_B.lastc--;
    }

    dynsim_2link_planar_B.lastc = 4;
    exitg2 = false;
    while ((!exitg2) && (dynsim_2link_planar_B.lastc > 0)) {
      dynsim_2link_planar_B.rowleft = dynsim_2link_planar_B.lastc + 4;
      dynsim_2link_planar_B.jy = dynsim_2link_planar_B.rowleft;
      do {
        exitg1 = 0;
        if (dynsim_2link_planar_B.jy <= (dynsim_2link_planar_B.knt << 2) +
            dynsim_2link_planar_B.rowleft) {
          if (a[dynsim_2link_planar_B.jy - 1] != 0.0) {
            exitg1 = 1;
          } else {
            dynsim_2link_planar_B.jy += 4;
          }
        } else {
          dynsim_2link_planar_B.lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    dynsim_2link_planar_B.knt = -1;
    dynsim_2link_planar_B.lastc = 0;
  }

  if (dynsim_2link_planar_B.knt + 1 > 0) {
    if (dynsim_2link_planar_B.lastc != 0) {
      dynsim_2link_planar_B.rowleft = 0;
      while (dynsim_2link_planar_B.rowleft <= dynsim_2link_planar_B.lastc - 1) {
        dynsim_2link_planar_B.work_j[dynsim_2link_planar_B.rowleft] = 0.0;
        dynsim_2link_planar_B.rowleft++;
      }

      dynsim_2link_planar_B.rowleft = 1;
      dynsim_2link_planar_B.jy = (dynsim_2link_planar_B.knt << 2) + 5;
      dynsim_2link_planar_B.iac = 5;
      while (dynsim_2link_planar_B.iac <= dynsim_2link_planar_B.jy) {
        dynsim_2link_planar_B.b_ix = 0;
        dynsim_2link_planar_B.g_k = (dynsim_2link_planar_B.iac +
          dynsim_2link_planar_B.lastc) - 1;
        dynsim_2link_planar_B.b_ia = dynsim_2link_planar_B.iac;
        while (dynsim_2link_planar_B.b_ia <= dynsim_2link_planar_B.g_k) {
          dynsim_2link_planar_B.work_j[dynsim_2link_planar_B.b_ix] +=
            a[dynsim_2link_planar_B.b_ia - 1] * a[dynsim_2link_planar_B.rowleft];
          dynsim_2link_planar_B.b_ix++;
          dynsim_2link_planar_B.b_ia++;
        }

        dynsim_2link_planar_B.rowleft++;
        dynsim_2link_planar_B.iac += 4;
      }
    }

    if (!(-tau[0] == 0.0)) {
      dynsim_2link_planar_B.rowleft = 4;
      dynsim_2link_planar_B.jy = 1;
      dynsim_2link_planar_B.iac = 0;
      while (dynsim_2link_planar_B.iac <= dynsim_2link_planar_B.knt) {
        if (a[dynsim_2link_planar_B.jy] != 0.0) {
          dynsim_2link_planar_B.xnorm = a[dynsim_2link_planar_B.jy] * -tau[0];
          dynsim_2link_planar_B.b_ix = 0;
          dynsim_2link_planar_B.g_k = dynsim_2link_planar_B.lastc +
            dynsim_2link_planar_B.rowleft;
          dynsim_2link_planar_B.b_ia = dynsim_2link_planar_B.rowleft;
          while (dynsim_2link_planar_B.b_ia + 1 <= dynsim_2link_planar_B.g_k) {
            a[dynsim_2link_planar_B.b_ia] +=
              dynsim_2link_planar_B.work_j[dynsim_2link_planar_B.b_ix] *
              dynsim_2link_planar_B.xnorm;
            dynsim_2link_planar_B.b_ix++;
            dynsim_2link_planar_B.b_ia++;
          }
        }

        dynsim_2link_planar_B.jy++;
        dynsim_2link_planar_B.rowleft += 4;
        dynsim_2link_planar_B.iac++;
      }
    }
  }

  dynsim_2link_planar_xzlarf(3, 3, 2, tau[0], a, 6, dynsim_2link_planar_B.work_j);
  a[1] = dynsim_2link_planar_B.alpha1;
  dynsim_2link_planar_B.alpha1 = a[6];
  tau[1] = 0.0;
  dynsim_2link_planar_B.xnorm = dynsim_2link_planar_xnrm2(1, a, 8);
  if (dynsim_2link_planar_B.xnorm != 0.0) {
    dynsim_2link_planar_B.xnorm = dynsim_2link_plan_rt_hypotd_snf(a[6],
      dynsim_2link_planar_B.xnorm);
    if (a[6] >= 0.0) {
      dynsim_2link_planar_B.xnorm = -dynsim_2link_planar_B.xnorm;
    }

    if (fabs(dynsim_2link_planar_B.xnorm) < 1.0020841800044864E-292) {
      dynsim_2link_planar_B.knt = -1;
      do {
        dynsim_2link_planar_B.knt++;
        a[7] *= 9.9792015476736E+291;
        dynsim_2link_planar_B.xnorm *= 9.9792015476736E+291;
        dynsim_2link_planar_B.alpha1 *= 9.9792015476736E+291;
      } while (!(fabs(dynsim_2link_planar_B.xnorm) >= 1.0020841800044864E-292));

      dynsim_2link_planar_B.xnorm = dynsim_2link_plan_rt_hypotd_snf
        (dynsim_2link_planar_B.alpha1, dynsim_2link_planar_xnrm2(1, a, 8));
      if (dynsim_2link_planar_B.alpha1 >= 0.0) {
        dynsim_2link_planar_B.xnorm = -dynsim_2link_planar_B.xnorm;
      }

      tau[1] = (dynsim_2link_planar_B.xnorm - dynsim_2link_planar_B.alpha1) /
        dynsim_2link_planar_B.xnorm;
      dynsim_2link_planar_B.alpha1 = 1.0 / (dynsim_2link_planar_B.alpha1 -
        dynsim_2link_planar_B.xnorm);
      a[7] *= dynsim_2link_planar_B.alpha1;
      dynsim_2link_planar_B.lastc = 0;
      while (dynsim_2link_planar_B.lastc <= dynsim_2link_planar_B.knt) {
        dynsim_2link_planar_B.xnorm *= 1.0020841800044864E-292;
        dynsim_2link_planar_B.lastc++;
      }

      dynsim_2link_planar_B.alpha1 = dynsim_2link_planar_B.xnorm;
    } else {
      tau[1] = (dynsim_2link_planar_B.xnorm - a[6]) /
        dynsim_2link_planar_B.xnorm;
      a[7] *= 1.0 / (a[6] - dynsim_2link_planar_B.xnorm);
      dynsim_2link_planar_B.alpha1 = dynsim_2link_planar_B.xnorm;
    }
  }

  a[6] = 1.0;
  if (tau[1] != 0.0) {
    dynsim_2link_planar_B.knt = 1;
    dynsim_2link_planar_B.lastc = 7;
    while ((dynsim_2link_planar_B.knt + 1 > 0) && (a[dynsim_2link_planar_B.lastc]
            == 0.0)) {
      dynsim_2link_planar_B.knt--;
      dynsim_2link_planar_B.lastc--;
    }

    dynsim_2link_planar_B.lastc = 4;
    exitg2 = false;
    while ((!exitg2) && (dynsim_2link_planar_B.lastc > 0)) {
      dynsim_2link_planar_B.rowleft = dynsim_2link_planar_B.lastc + 8;
      dynsim_2link_planar_B.jy = dynsim_2link_planar_B.rowleft;
      do {
        exitg1 = 0;
        if (dynsim_2link_planar_B.jy <= (dynsim_2link_planar_B.knt << 2) +
            dynsim_2link_planar_B.rowleft) {
          if (a[dynsim_2link_planar_B.jy - 1] != 0.0) {
            exitg1 = 1;
          } else {
            dynsim_2link_planar_B.jy += 4;
          }
        } else {
          dynsim_2link_planar_B.lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    dynsim_2link_planar_B.knt = -1;
    dynsim_2link_planar_B.lastc = 0;
  }

  if (dynsim_2link_planar_B.knt + 1 > 0) {
    if (dynsim_2link_planar_B.lastc != 0) {
      dynsim_2link_planar_B.rowleft = 0;
      while (dynsim_2link_planar_B.rowleft <= dynsim_2link_planar_B.lastc - 1) {
        dynsim_2link_planar_B.work_j[dynsim_2link_planar_B.rowleft] = 0.0;
        dynsim_2link_planar_B.rowleft++;
      }

      dynsim_2link_planar_B.rowleft = 6;
      dynsim_2link_planar_B.jy = (dynsim_2link_planar_B.knt << 2) + 9;
      dynsim_2link_planar_B.iac = 9;
      while (dynsim_2link_planar_B.iac <= dynsim_2link_planar_B.jy) {
        dynsim_2link_planar_B.b_ix = 0;
        dynsim_2link_planar_B.g_k = (dynsim_2link_planar_B.iac +
          dynsim_2link_planar_B.lastc) - 1;
        dynsim_2link_planar_B.b_ia = dynsim_2link_planar_B.iac;
        while (dynsim_2link_planar_B.b_ia <= dynsim_2link_planar_B.g_k) {
          dynsim_2link_planar_B.work_j[dynsim_2link_planar_B.b_ix] +=
            a[dynsim_2link_planar_B.b_ia - 1] * a[dynsim_2link_planar_B.rowleft];
          dynsim_2link_planar_B.b_ix++;
          dynsim_2link_planar_B.b_ia++;
        }

        dynsim_2link_planar_B.rowleft++;
        dynsim_2link_planar_B.iac += 4;
      }
    }

    if (!(-tau[1] == 0.0)) {
      dynsim_2link_planar_B.rowleft = 8;
      dynsim_2link_planar_B.jy = 6;
      dynsim_2link_planar_B.iac = 0;
      while (dynsim_2link_planar_B.iac <= dynsim_2link_planar_B.knt) {
        if (a[dynsim_2link_planar_B.jy] != 0.0) {
          dynsim_2link_planar_B.xnorm = a[dynsim_2link_planar_B.jy] * -tau[1];
          dynsim_2link_planar_B.b_ix = 0;
          dynsim_2link_planar_B.g_k = dynsim_2link_planar_B.lastc +
            dynsim_2link_planar_B.rowleft;
          dynsim_2link_planar_B.b_ia = dynsim_2link_planar_B.rowleft;
          while (dynsim_2link_planar_B.b_ia + 1 <= dynsim_2link_planar_B.g_k) {
            a[dynsim_2link_planar_B.b_ia] +=
              dynsim_2link_planar_B.work_j[dynsim_2link_planar_B.b_ix] *
              dynsim_2link_planar_B.xnorm;
            dynsim_2link_planar_B.b_ix++;
            dynsim_2link_planar_B.b_ia++;
          }
        }

        dynsim_2link_planar_B.jy++;
        dynsim_2link_planar_B.rowleft += 4;
        dynsim_2link_planar_B.iac++;
      }
    }
  }

  dynsim_2link_planar_xzlarf(2, 2, 7, tau[1], a, 11,
    dynsim_2link_planar_B.work_j);
  a[6] = dynsim_2link_planar_B.alpha1;
  dynsim_2link_planar_B.alpha1 = a[11];
  tau[2] = 0.0;
  dynsim_2link_planar_B.xnorm = dynsim_2link_planar_xnrm2(0, a, 12);
  if (dynsim_2link_planar_B.xnorm != 0.0) {
    dynsim_2link_planar_B.xnorm = dynsim_2link_plan_rt_hypotd_snf(a[11],
      dynsim_2link_planar_B.xnorm);
    if (a[11] >= 0.0) {
      dynsim_2link_planar_B.xnorm = -dynsim_2link_planar_B.xnorm;
    }

    if (fabs(dynsim_2link_planar_B.xnorm) < 1.0020841800044864E-292) {
      dynsim_2link_planar_B.knt = -1;
      do {
        dynsim_2link_planar_B.knt++;
        dynsim_2link_planar_B.xnorm *= 9.9792015476736E+291;
        dynsim_2link_planar_B.alpha1 *= 9.9792015476736E+291;
      } while (!(fabs(dynsim_2link_planar_B.xnorm) >= 1.0020841800044864E-292));

      dynsim_2link_planar_B.xnorm = dynsim_2link_plan_rt_hypotd_snf
        (dynsim_2link_planar_B.alpha1, dynsim_2link_planar_xnrm2(0, a, 12));
      if (dynsim_2link_planar_B.alpha1 >= 0.0) {
        dynsim_2link_planar_B.xnorm = -dynsim_2link_planar_B.xnorm;
      }

      tau[2] = (dynsim_2link_planar_B.xnorm - dynsim_2link_planar_B.alpha1) /
        dynsim_2link_planar_B.xnorm;
      dynsim_2link_planar_B.lastc = 0;
      while (dynsim_2link_planar_B.lastc <= dynsim_2link_planar_B.knt) {
        dynsim_2link_planar_B.xnorm *= 1.0020841800044864E-292;
        dynsim_2link_planar_B.lastc++;
      }

      dynsim_2link_planar_B.alpha1 = dynsim_2link_planar_B.xnorm;
    } else {
      tau[2] = (dynsim_2link_planar_B.xnorm - a[11]) /
        dynsim_2link_planar_B.xnorm;
      dynsim_2link_planar_B.alpha1 = dynsim_2link_planar_B.xnorm;
    }
  }

  a[11] = 1.0;
  if (tau[2] != 0.0) {
    dynsim_2link_planar_B.knt = 0;
    dynsim_2link_planar_B.lastc = 11;
    while ((dynsim_2link_planar_B.knt + 1 > 0) && (a[dynsim_2link_planar_B.lastc]
            == 0.0)) {
      dynsim_2link_planar_B.knt--;
      dynsim_2link_planar_B.lastc--;
    }

    dynsim_2link_planar_B.lastc = 4;
    exitg2 = false;
    while ((!exitg2) && (dynsim_2link_planar_B.lastc > 0)) {
      dynsim_2link_planar_B.rowleft = dynsim_2link_planar_B.lastc + 12;
      dynsim_2link_planar_B.jy = dynsim_2link_planar_B.rowleft;
      do {
        exitg1 = 0;
        if (dynsim_2link_planar_B.jy <= (dynsim_2link_planar_B.knt << 2) +
            dynsim_2link_planar_B.rowleft) {
          if (a[dynsim_2link_planar_B.jy - 1] != 0.0) {
            exitg1 = 1;
          } else {
            dynsim_2link_planar_B.jy += 4;
          }
        } else {
          dynsim_2link_planar_B.lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    dynsim_2link_planar_B.knt = -1;
    dynsim_2link_planar_B.lastc = 0;
  }

  if (dynsim_2link_planar_B.knt + 1 > 0) {
    if (dynsim_2link_planar_B.lastc != 0) {
      dynsim_2link_planar_B.rowleft = 0;
      while (dynsim_2link_planar_B.rowleft <= dynsim_2link_planar_B.lastc - 1) {
        dynsim_2link_planar_B.work_j[dynsim_2link_planar_B.rowleft] = 0.0;
        dynsim_2link_planar_B.rowleft++;
      }

      dynsim_2link_planar_B.rowleft = 11;
      dynsim_2link_planar_B.jy = (dynsim_2link_planar_B.knt << 2) + 13;
      dynsim_2link_planar_B.iac = 13;
      while (dynsim_2link_planar_B.iac <= dynsim_2link_planar_B.jy) {
        dynsim_2link_planar_B.b_ix = 0;
        dynsim_2link_planar_B.g_k = (dynsim_2link_planar_B.iac +
          dynsim_2link_planar_B.lastc) - 1;
        dynsim_2link_planar_B.b_ia = dynsim_2link_planar_B.iac;
        while (dynsim_2link_planar_B.b_ia <= dynsim_2link_planar_B.g_k) {
          dynsim_2link_planar_B.work_j[dynsim_2link_planar_B.b_ix] +=
            a[dynsim_2link_planar_B.b_ia - 1] * a[dynsim_2link_planar_B.rowleft];
          dynsim_2link_planar_B.b_ix++;
          dynsim_2link_planar_B.b_ia++;
        }

        dynsim_2link_planar_B.rowleft++;
        dynsim_2link_planar_B.iac += 4;
      }
    }

    if (!(-tau[2] == 0.0)) {
      dynsim_2link_planar_B.rowleft = 12;
      dynsim_2link_planar_B.jy = 11;
      dynsim_2link_planar_B.iac = 0;
      while (dynsim_2link_planar_B.iac <= dynsim_2link_planar_B.knt) {
        if (a[dynsim_2link_planar_B.jy] != 0.0) {
          dynsim_2link_planar_B.xnorm = a[dynsim_2link_planar_B.jy] * -tau[2];
          dynsim_2link_planar_B.b_ix = 0;
          dynsim_2link_planar_B.g_k = dynsim_2link_planar_B.lastc +
            dynsim_2link_planar_B.rowleft;
          dynsim_2link_planar_B.b_ia = dynsim_2link_planar_B.rowleft;
          while (dynsim_2link_planar_B.b_ia + 1 <= dynsim_2link_planar_B.g_k) {
            a[dynsim_2link_planar_B.b_ia] +=
              dynsim_2link_planar_B.work_j[dynsim_2link_planar_B.b_ix] *
              dynsim_2link_planar_B.xnorm;
            dynsim_2link_planar_B.b_ix++;
            dynsim_2link_planar_B.b_ia++;
          }
        }

        dynsim_2link_planar_B.jy++;
        dynsim_2link_planar_B.rowleft += 4;
        dynsim_2link_planar_B.iac++;
      }
    }
  }

  dynsim_2link_planar_xzlarf(1, 1, 12, tau[2], a, 16,
    dynsim_2link_planar_B.work_j);
  a[11] = dynsim_2link_planar_B.alpha1;
}

// Function for MATLAB Function: '<S4>/MATLAB Function'
static real_T dynsim_2link_planar_xnrm2_a(int32_T n, const real_T x[3])
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
static real_T dynsim_2link_planar_xzlarfg(int32_T n, real_T *alpha1, real_T x[3])
{
  real_T tau;
  real_T xnorm;
  int32_T knt;
  int32_T c_k;
  tau = 0.0;
  if (n > 0) {
    xnorm = dynsim_2link_planar_xnrm2_a(n - 1, x);
    if (xnorm != 0.0) {
      xnorm = dynsim_2link_plan_rt_hypotd_snf(*alpha1, xnorm);
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

        xnorm = dynsim_2link_plan_rt_hypotd_snf(*alpha1,
          dynsim_2link_planar_xnrm2_a(n - 1, x));
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
static void dynsim_2link_planar_xdlanv2(real_T *a, real_T *b, real_T *c, real_T *
  d, real_T *rt1r, real_T *rt1i, real_T *rt2r, real_T *rt2i, real_T *cs, real_T *
  sn)
{
  if (*c == 0.0) {
    *cs = 1.0;
    *sn = 0.0;
  } else if (*b == 0.0) {
    *cs = 0.0;
    *sn = 1.0;
    dynsim_2link_planar_B.bcmax = *d;
    *d = *a;
    *a = dynsim_2link_planar_B.bcmax;
    *b = -*c;
    *c = 0.0;
  } else {
    dynsim_2link_planar_B.tau_p = *a - *d;
    if ((dynsim_2link_planar_B.tau_p == 0.0) && ((*b < 0.0) != (*c < 0.0))) {
      *cs = 1.0;
      *sn = 0.0;
    } else {
      dynsim_2link_planar_B.p = 0.5 * dynsim_2link_planar_B.tau_p;
      dynsim_2link_planar_B.bcmis = fabs(*b);
      dynsim_2link_planar_B.z = fabs(*c);
      dynsim_2link_planar_B.b_oi = rtIsNaN(dynsim_2link_planar_B.z);
      if ((dynsim_2link_planar_B.bcmis > dynsim_2link_planar_B.z) ||
          dynsim_2link_planar_B.b_oi) {
        dynsim_2link_planar_B.bcmax = dynsim_2link_planar_B.bcmis;
      } else {
        dynsim_2link_planar_B.bcmax = dynsim_2link_planar_B.z;
      }

      if ((dynsim_2link_planar_B.bcmis < dynsim_2link_planar_B.z) ||
          dynsim_2link_planar_B.b_oi) {
        dynsim_2link_planar_B.z = dynsim_2link_planar_B.bcmis;
      }

      if (!(*b < 0.0)) {
        dynsim_2link_planar_B.b_cq = 1;
      } else {
        dynsim_2link_planar_B.b_cq = -1;
      }

      if (!(*c < 0.0)) {
        dynsim_2link_planar_B.c_c = 1;
      } else {
        dynsim_2link_planar_B.c_c = -1;
      }

      dynsim_2link_planar_B.bcmis = dynsim_2link_planar_B.z * static_cast<real_T>
        (dynsim_2link_planar_B.b_cq) * static_cast<real_T>
        (dynsim_2link_planar_B.c_c);
      dynsim_2link_planar_B.scale_p = fabs(dynsim_2link_planar_B.p);
      if ((!(dynsim_2link_planar_B.scale_p > dynsim_2link_planar_B.bcmax)) &&
          (!rtIsNaN(dynsim_2link_planar_B.bcmax))) {
        dynsim_2link_planar_B.scale_p = dynsim_2link_planar_B.bcmax;
      }

      dynsim_2link_planar_B.z = dynsim_2link_planar_B.p /
        dynsim_2link_planar_B.scale_p * dynsim_2link_planar_B.p +
        dynsim_2link_planar_B.bcmax / dynsim_2link_planar_B.scale_p *
        dynsim_2link_planar_B.bcmis;
      if (dynsim_2link_planar_B.z >= 8.8817841970012523E-16) {
        if (!(dynsim_2link_planar_B.p < 0.0)) {
          dynsim_2link_planar_B.tau_p = sqrt(dynsim_2link_planar_B.scale_p) *
            sqrt(dynsim_2link_planar_B.z);
        } else {
          dynsim_2link_planar_B.tau_p = -(sqrt(dynsim_2link_planar_B.scale_p) *
            sqrt(dynsim_2link_planar_B.z));
        }

        dynsim_2link_planar_B.z = dynsim_2link_planar_B.p +
          dynsim_2link_planar_B.tau_p;
        *a = *d + dynsim_2link_planar_B.z;
        *d -= dynsim_2link_planar_B.bcmax / dynsim_2link_planar_B.z *
          dynsim_2link_planar_B.bcmis;
        dynsim_2link_planar_B.tau_p = dynsim_2link_plan_rt_hypotd_snf(*c,
          dynsim_2link_planar_B.z);
        *cs = dynsim_2link_planar_B.z / dynsim_2link_planar_B.tau_p;
        *sn = *c / dynsim_2link_planar_B.tau_p;
        *b -= *c;
        *c = 0.0;
      } else {
        dynsim_2link_planar_B.bcmax = *b + *c;
        dynsim_2link_planar_B.tau_p = dynsim_2link_plan_rt_hypotd_snf
          (dynsim_2link_planar_B.bcmax, dynsim_2link_planar_B.tau_p);
        *cs = sqrt((fabs(dynsim_2link_planar_B.bcmax) /
                    dynsim_2link_planar_B.tau_p + 1.0) * 0.5);
        if (!(dynsim_2link_planar_B.bcmax < 0.0)) {
          dynsim_2link_planar_B.b_cq = 1;
        } else {
          dynsim_2link_planar_B.b_cq = -1;
        }

        *sn = -(dynsim_2link_planar_B.p / (dynsim_2link_planar_B.tau_p * *cs)) *
          static_cast<real_T>(dynsim_2link_planar_B.b_cq);
        dynsim_2link_planar_B.p = *a * *cs + *b * *sn;
        dynsim_2link_planar_B.tau_p = -*a * *sn + *b * *cs;
        dynsim_2link_planar_B.bcmax = *c * *cs + *d * *sn;
        dynsim_2link_planar_B.bcmis = -*c * *sn + *d * *cs;
        *b = dynsim_2link_planar_B.tau_p * *cs + dynsim_2link_planar_B.bcmis *
          *sn;
        *c = -dynsim_2link_planar_B.p * *sn + dynsim_2link_planar_B.bcmax * *cs;
        dynsim_2link_planar_B.bcmax = ((dynsim_2link_planar_B.p * *cs +
          dynsim_2link_planar_B.bcmax * *sn) + (-dynsim_2link_planar_B.tau_p *
          *sn + dynsim_2link_planar_B.bcmis * *cs)) * 0.5;
        *a = dynsim_2link_planar_B.bcmax;
        *d = dynsim_2link_planar_B.bcmax;
        if (*c != 0.0) {
          if (*b != 0.0) {
            if ((*b < 0.0) == (*c < 0.0)) {
              dynsim_2link_planar_B.z = sqrt(fabs(*b));
              dynsim_2link_planar_B.bcmis = sqrt(fabs(*c));
              if (!(*c < 0.0)) {
                dynsim_2link_planar_B.p = dynsim_2link_planar_B.z *
                  dynsim_2link_planar_B.bcmis;
              } else {
                dynsim_2link_planar_B.p = -(dynsim_2link_planar_B.z *
                  dynsim_2link_planar_B.bcmis);
              }

              dynsim_2link_planar_B.tau_p = 1.0 / sqrt(fabs(*b + *c));
              *a = dynsim_2link_planar_B.bcmax + dynsim_2link_planar_B.p;
              *d = dynsim_2link_planar_B.bcmax - dynsim_2link_planar_B.p;
              *b -= *c;
              *c = 0.0;
              dynsim_2link_planar_B.p = dynsim_2link_planar_B.z *
                dynsim_2link_planar_B.tau_p;
              dynsim_2link_planar_B.tau_p *= dynsim_2link_planar_B.bcmis;
              dynsim_2link_planar_B.bcmax = *cs * dynsim_2link_planar_B.p - *sn *
                dynsim_2link_planar_B.tau_p;
              *sn = *cs * dynsim_2link_planar_B.tau_p + *sn *
                dynsim_2link_planar_B.p;
              *cs = dynsim_2link_planar_B.bcmax;
            }
          } else {
            *b = -*c;
            *c = 0.0;
            dynsim_2link_planar_B.bcmax = *cs;
            *cs = -*sn;
            *sn = dynsim_2link_planar_B.bcmax;
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
static void dynsim_2link_planar_xrot(int32_T n, real_T x[16], int32_T ix0,
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
static void dynsim_2link_planar_xrot_l(int32_T n, real_T x[16], int32_T ix0,
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
static int32_T dynsim_2link_planar_eml_dlahqr(real_T h[16], real_T z[16])
{
  int32_T info;
  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T exitg3;
  info = 0;
  dynsim_2link_planar_B.v_nv[0] = 0.0;
  dynsim_2link_planar_B.v_nv[1] = 0.0;
  dynsim_2link_planar_B.v_nv[2] = 0.0;
  h[2] = 0.0;
  h[3] = 0.0;
  h[7] = 0.0;
  dynsim_2link_planar_B.i_n = 3;
  exitg1 = false;
  while ((!exitg1) && (dynsim_2link_planar_B.i_n + 1 >= 1)) {
    dynsim_2link_planar_B.L = 1;
    dynsim_2link_planar_B.goto150 = false;
    dynsim_2link_planar_B.ix = 0;
    exitg2 = false;
    while ((!exitg2) && (dynsim_2link_planar_B.ix < 301)) {
      dynsim_2link_planar_B.k_k = dynsim_2link_planar_B.i_n;
      exitg3 = false;
      while ((!exitg3) && (dynsim_2link_planar_B.k_k + 1 >
                           dynsim_2link_planar_B.L)) {
        dynsim_2link_planar_B.s_tmp = ((dynsim_2link_planar_B.k_k - 1) << 2) +
          dynsim_2link_planar_B.k_k;
        dynsim_2link_planar_B.htmp2 = fabs(h[dynsim_2link_planar_B.s_tmp]);
        if (dynsim_2link_planar_B.htmp2 <= 4.0083367200179456E-292) {
          exitg3 = true;
        } else {
          dynsim_2link_planar_B.m_n = (dynsim_2link_planar_B.k_k << 2) +
            dynsim_2link_planar_B.k_k;
          dynsim_2link_planar_B.nr = dynsim_2link_planar_B.s_tmp - 1;
          dynsim_2link_planar_B.tst = fabs(h[dynsim_2link_planar_B.nr]) + fabs
            (h[dynsim_2link_planar_B.m_n]);
          if (dynsim_2link_planar_B.tst == 0.0) {
            if (dynsim_2link_planar_B.k_k - 1 >= 1) {
              dynsim_2link_planar_B.tst = fabs(h[(((dynsim_2link_planar_B.k_k -
                2) << 2) + dynsim_2link_planar_B.k_k) - 1]);
            }

            if (dynsim_2link_planar_B.k_k + 2 <= 4) {
              dynsim_2link_planar_B.tst += fabs(h[((dynsim_2link_planar_B.k_k <<
                2) + dynsim_2link_planar_B.k_k) + 1]);
            }
          }

          if (dynsim_2link_planar_B.htmp2 <= 2.2204460492503131E-16 *
              dynsim_2link_planar_B.tst) {
            dynsim_2link_planar_B.htmp1 = fabs(h[dynsim_2link_planar_B.s_tmp]);
            dynsim_2link_planar_B.htmp2 = fabs(h[dynsim_2link_planar_B.m_n - 1]);
            if (dynsim_2link_planar_B.htmp1 > dynsim_2link_planar_B.htmp2) {
              dynsim_2link_planar_B.tst = dynsim_2link_planar_B.htmp1;
              dynsim_2link_planar_B.ba = dynsim_2link_planar_B.htmp2;
            } else {
              dynsim_2link_planar_B.tst = dynsim_2link_planar_B.htmp2;
              dynsim_2link_planar_B.ba = dynsim_2link_planar_B.htmp1;
            }

            dynsim_2link_planar_B.htmp1 = fabs(h[dynsim_2link_planar_B.m_n]);
            dynsim_2link_planar_B.htmp2 = fabs(h[dynsim_2link_planar_B.nr] -
              h[dynsim_2link_planar_B.m_n]);
            if (dynsim_2link_planar_B.htmp1 > dynsim_2link_planar_B.htmp2) {
              dynsim_2link_planar_B.aa = dynsim_2link_planar_B.htmp1;
              dynsim_2link_planar_B.htmp1 = dynsim_2link_planar_B.htmp2;
            } else {
              dynsim_2link_planar_B.aa = dynsim_2link_planar_B.htmp2;
            }

            dynsim_2link_planar_B.htmp2 = dynsim_2link_planar_B.aa +
              dynsim_2link_planar_B.tst;
            dynsim_2link_planar_B.htmp1 = dynsim_2link_planar_B.aa /
              dynsim_2link_planar_B.htmp2 * dynsim_2link_planar_B.htmp1 *
              2.2204460492503131E-16;
            if ((4.0083367200179456E-292 > dynsim_2link_planar_B.htmp1) ||
                rtIsNaN(dynsim_2link_planar_B.htmp1)) {
              dynsim_2link_planar_B.htmp1 = 4.0083367200179456E-292;
            }

            if (dynsim_2link_planar_B.tst / dynsim_2link_planar_B.htmp2 *
                dynsim_2link_planar_B.ba <= dynsim_2link_planar_B.htmp1) {
              exitg3 = true;
            } else {
              dynsim_2link_planar_B.k_k--;
            }
          } else {
            dynsim_2link_planar_B.k_k--;
          }
        }
      }

      dynsim_2link_planar_B.L = dynsim_2link_planar_B.k_k + 1;
      if (dynsim_2link_planar_B.k_k + 1 > 1) {
        h[dynsim_2link_planar_B.k_k + ((dynsim_2link_planar_B.k_k - 1) << 2)] =
          0.0;
      }

      if (dynsim_2link_planar_B.k_k + 1 >= dynsim_2link_planar_B.i_n) {
        dynsim_2link_planar_B.goto150 = true;
        exitg2 = true;
      } else {
        switch (dynsim_2link_planar_B.ix) {
         case 10:
          dynsim_2link_planar_B.s_tmp = (dynsim_2link_planar_B.k_k << 2) +
            dynsim_2link_planar_B.k_k;
          dynsim_2link_planar_B.htmp2 = fabs(h[(((dynsim_2link_planar_B.k_k + 1)
            << 2) + dynsim_2link_planar_B.k_k) + 2]) + fabs
            (h[dynsim_2link_planar_B.s_tmp + 1]);
          dynsim_2link_planar_B.ba = h[dynsim_2link_planar_B.s_tmp] + 0.75 *
            dynsim_2link_planar_B.htmp2;
          dynsim_2link_planar_B.h12 = -0.4375 * dynsim_2link_planar_B.htmp2;
          dynsim_2link_planar_B.aa = dynsim_2link_planar_B.htmp2;
          dynsim_2link_planar_B.tst = dynsim_2link_planar_B.ba;
          break;

         case 20:
          dynsim_2link_planar_B.htmp2 = fabs(h[(((dynsim_2link_planar_B.i_n - 2)
            << 2) + dynsim_2link_planar_B.i_n) - 1]) + fabs(h
            [((dynsim_2link_planar_B.i_n - 1) << 2) + dynsim_2link_planar_B.i_n]);
          dynsim_2link_planar_B.ba = h[(dynsim_2link_planar_B.i_n << 2) +
            dynsim_2link_planar_B.i_n] + 0.75 * dynsim_2link_planar_B.htmp2;
          dynsim_2link_planar_B.h12 = -0.4375 * dynsim_2link_planar_B.htmp2;
          dynsim_2link_planar_B.aa = dynsim_2link_planar_B.htmp2;
          dynsim_2link_planar_B.tst = dynsim_2link_planar_B.ba;
          break;

         default:
          dynsim_2link_planar_B.ba = h[(((dynsim_2link_planar_B.i_n - 1) << 2) +
            dynsim_2link_planar_B.i_n) - 1];
          dynsim_2link_planar_B.aa = h[((dynsim_2link_planar_B.i_n - 1) << 2) +
            dynsim_2link_planar_B.i_n];
          dynsim_2link_planar_B.h12 = h[((dynsim_2link_planar_B.i_n << 2) +
            dynsim_2link_planar_B.i_n) - 1];
          dynsim_2link_planar_B.tst = h[(dynsim_2link_planar_B.i_n << 2) +
            dynsim_2link_planar_B.i_n];
          break;
        }

        dynsim_2link_planar_B.htmp2 = ((fabs(dynsim_2link_planar_B.ba) + fabs
          (dynsim_2link_planar_B.h12)) + fabs(dynsim_2link_planar_B.aa)) + fabs
          (dynsim_2link_planar_B.tst);
        if (dynsim_2link_planar_B.htmp2 == 0.0) {
          dynsim_2link_planar_B.ba = 0.0;
          dynsim_2link_planar_B.tst = 0.0;
          dynsim_2link_planar_B.htmp1 = 0.0;
          dynsim_2link_planar_B.aa = 0.0;
        } else {
          dynsim_2link_planar_B.ba /= dynsim_2link_planar_B.htmp2;
          dynsim_2link_planar_B.aa /= dynsim_2link_planar_B.htmp2;
          dynsim_2link_planar_B.h12 /= dynsim_2link_planar_B.htmp2;
          dynsim_2link_planar_B.tst /= dynsim_2link_planar_B.htmp2;
          dynsim_2link_planar_B.htmp1 = (dynsim_2link_planar_B.ba +
            dynsim_2link_planar_B.tst) / 2.0;
          dynsim_2link_planar_B.ba = (dynsim_2link_planar_B.ba -
            dynsim_2link_planar_B.htmp1) * (dynsim_2link_planar_B.tst -
            dynsim_2link_planar_B.htmp1) - dynsim_2link_planar_B.h12 *
            dynsim_2link_planar_B.aa;
          dynsim_2link_planar_B.aa = sqrt(fabs(dynsim_2link_planar_B.ba));
          if (dynsim_2link_planar_B.ba >= 0.0) {
            dynsim_2link_planar_B.ba = dynsim_2link_planar_B.htmp1 *
              dynsim_2link_planar_B.htmp2;
            dynsim_2link_planar_B.htmp1 = dynsim_2link_planar_B.ba;
            dynsim_2link_planar_B.tst = dynsim_2link_planar_B.aa *
              dynsim_2link_planar_B.htmp2;
            dynsim_2link_planar_B.aa = -dynsim_2link_planar_B.tst;
          } else {
            dynsim_2link_planar_B.ba = dynsim_2link_planar_B.htmp1 +
              dynsim_2link_planar_B.aa;
            dynsim_2link_planar_B.htmp1 -= dynsim_2link_planar_B.aa;
            if (fabs(dynsim_2link_planar_B.ba - dynsim_2link_planar_B.tst) <=
                fabs(dynsim_2link_planar_B.htmp1 - dynsim_2link_planar_B.tst)) {
              dynsim_2link_planar_B.ba *= dynsim_2link_planar_B.htmp2;
              dynsim_2link_planar_B.htmp1 = dynsim_2link_planar_B.ba;
            } else {
              dynsim_2link_planar_B.htmp1 *= dynsim_2link_planar_B.htmp2;
              dynsim_2link_planar_B.ba = dynsim_2link_planar_B.htmp1;
            }

            dynsim_2link_planar_B.tst = 0.0;
            dynsim_2link_planar_B.aa = 0.0;
          }
        }

        dynsim_2link_planar_B.m_n = dynsim_2link_planar_B.i_n - 1;
        exitg3 = false;
        while ((!exitg3) && (dynsim_2link_planar_B.m_n >=
                             dynsim_2link_planar_B.k_k + 1)) {
          dynsim_2link_planar_B.s_tmp = ((dynsim_2link_planar_B.m_n - 1) << 2) +
            dynsim_2link_planar_B.m_n;
          dynsim_2link_planar_B.nr = dynsim_2link_planar_B.s_tmp - 1;
          dynsim_2link_planar_B.h12 = h[dynsim_2link_planar_B.nr] -
            dynsim_2link_planar_B.htmp1;
          dynsim_2link_planar_B.htmp2 = (fabs(dynsim_2link_planar_B.h12) + fabs
            (dynsim_2link_planar_B.aa)) + fabs(h[dynsim_2link_planar_B.s_tmp]);
          dynsim_2link_planar_B.h21s = h[dynsim_2link_planar_B.s_tmp] /
            dynsim_2link_planar_B.htmp2;
          dynsim_2link_planar_B.hoffset = (dynsim_2link_planar_B.m_n << 2) +
            dynsim_2link_planar_B.m_n;
          dynsim_2link_planar_B.v_nv[0] = (dynsim_2link_planar_B.h12 /
            dynsim_2link_planar_B.htmp2 * (h[dynsim_2link_planar_B.nr] -
            dynsim_2link_planar_B.ba) + h[dynsim_2link_planar_B.hoffset - 1] *
            dynsim_2link_planar_B.h21s) - dynsim_2link_planar_B.aa /
            dynsim_2link_planar_B.htmp2 * dynsim_2link_planar_B.tst;
          dynsim_2link_planar_B.v_nv[1] = (((h[dynsim_2link_planar_B.nr] +
            h[dynsim_2link_planar_B.hoffset]) - dynsim_2link_planar_B.ba) -
            dynsim_2link_planar_B.htmp1) * dynsim_2link_planar_B.h21s;
          dynsim_2link_planar_B.v_nv[2] = h[dynsim_2link_planar_B.hoffset + 1] *
            dynsim_2link_planar_B.h21s;
          dynsim_2link_planar_B.htmp2 = (fabs(dynsim_2link_planar_B.v_nv[0]) +
            fabs(dynsim_2link_planar_B.v_nv[1])) + fabs
            (dynsim_2link_planar_B.v_nv[2]);
          dynsim_2link_planar_B.v_nv[0] /= dynsim_2link_planar_B.htmp2;
          dynsim_2link_planar_B.v_nv[1] /= dynsim_2link_planar_B.htmp2;
          dynsim_2link_planar_B.v_nv[2] /= dynsim_2link_planar_B.htmp2;
          if (dynsim_2link_planar_B.k_k + 1 == dynsim_2link_planar_B.m_n) {
            exitg3 = true;
          } else {
            dynsim_2link_planar_B.s_tmp = ((dynsim_2link_planar_B.m_n - 2) << 2)
              + dynsim_2link_planar_B.m_n;
            if (fabs(h[dynsim_2link_planar_B.s_tmp - 1]) * (fabs
                 (dynsim_2link_planar_B.v_nv[1]) + fabs
                 (dynsim_2link_planar_B.v_nv[2])) <= ((fabs
                  (h[dynsim_2link_planar_B.s_tmp - 2]) + fabs
                  (h[dynsim_2link_planar_B.nr])) + fabs
                 (h[dynsim_2link_planar_B.hoffset])) * (2.2204460492503131E-16 *
                 fabs(dynsim_2link_planar_B.v_nv[0]))) {
              exitg3 = true;
            } else {
              dynsim_2link_planar_B.m_n--;
            }
          }
        }

        dynsim_2link_planar_B.s_tmp = dynsim_2link_planar_B.m_n;
        while (dynsim_2link_planar_B.s_tmp <= dynsim_2link_planar_B.i_n) {
          dynsim_2link_planar_B.nr = (dynsim_2link_planar_B.i_n -
            dynsim_2link_planar_B.s_tmp) + 2;
          if (3 < dynsim_2link_planar_B.nr) {
            dynsim_2link_planar_B.nr = 3;
          }

          if (dynsim_2link_planar_B.s_tmp > dynsim_2link_planar_B.m_n) {
            dynsim_2link_planar_B.hoffset = ((dynsim_2link_planar_B.s_tmp - 2) <<
              2) + dynsim_2link_planar_B.s_tmp;
            dynsim_2link_planar_B.j_o = 0;
            while (dynsim_2link_planar_B.j_o <= dynsim_2link_planar_B.nr - 1) {
              dynsim_2link_planar_B.v_nv[dynsim_2link_planar_B.j_o] = h
                [(dynsim_2link_planar_B.j_o + dynsim_2link_planar_B.hoffset) - 1];
              dynsim_2link_planar_B.j_o++;
            }
          }

          dynsim_2link_planar_B.tst = dynsim_2link_planar_B.v_nv[0];
          dynsim_2link_planar_B.b_v[0] = dynsim_2link_planar_B.v_nv[0];
          dynsim_2link_planar_B.b_v[1] = dynsim_2link_planar_B.v_nv[1];
          dynsim_2link_planar_B.b_v[2] = dynsim_2link_planar_B.v_nv[2];
          dynsim_2link_planar_B.htmp2 = dynsim_2link_planar_xzlarfg
            (dynsim_2link_planar_B.nr, &dynsim_2link_planar_B.tst,
             dynsim_2link_planar_B.b_v);
          dynsim_2link_planar_B.v_nv[1] = dynsim_2link_planar_B.b_v[1];
          dynsim_2link_planar_B.v_nv[2] = dynsim_2link_planar_B.b_v[2];
          dynsim_2link_planar_B.v_nv[0] = dynsim_2link_planar_B.tst;
          if (dynsim_2link_planar_B.s_tmp > dynsim_2link_planar_B.m_n) {
            h[(dynsim_2link_planar_B.s_tmp + ((dynsim_2link_planar_B.s_tmp - 2) <<
                2)) - 1] = dynsim_2link_planar_B.tst;
            h[dynsim_2link_planar_B.s_tmp + ((dynsim_2link_planar_B.s_tmp - 2) <<
              2)] = 0.0;
            if (dynsim_2link_planar_B.s_tmp < dynsim_2link_planar_B.i_n) {
              h[dynsim_2link_planar_B.s_tmp + 1] = 0.0;
            }
          } else {
            if (dynsim_2link_planar_B.m_n > dynsim_2link_planar_B.k_k + 1) {
              h[dynsim_2link_planar_B.s_tmp - 1] *= 1.0 -
                dynsim_2link_planar_B.htmp2;
            }
          }

          dynsim_2link_planar_B.tst = dynsim_2link_planar_B.b_v[1];
          dynsim_2link_planar_B.ba = dynsim_2link_planar_B.htmp2 *
            dynsim_2link_planar_B.b_v[1];
          switch (dynsim_2link_planar_B.nr) {
           case 3:
            dynsim_2link_planar_B.htmp1 = dynsim_2link_planar_B.b_v[2];
            dynsim_2link_planar_B.aa = dynsim_2link_planar_B.htmp2 *
              dynsim_2link_planar_B.b_v[2];
            dynsim_2link_planar_B.b_j_g = dynsim_2link_planar_B.s_tmp - 1;
            while (dynsim_2link_planar_B.b_j_g + 1 < 5) {
              dynsim_2link_planar_B.nr = (dynsim_2link_planar_B.b_j_g << 2) +
                dynsim_2link_planar_B.s_tmp;
              dynsim_2link_planar_B.hoffset = dynsim_2link_planar_B.nr - 1;
              dynsim_2link_planar_B.j_o = dynsim_2link_planar_B.nr + 1;
              dynsim_2link_planar_B.h12 = (h[dynsim_2link_planar_B.hoffset] +
                h[dynsim_2link_planar_B.nr] * dynsim_2link_planar_B.tst) +
                h[dynsim_2link_planar_B.j_o] * dynsim_2link_planar_B.htmp1;
              h[dynsim_2link_planar_B.hoffset] -= dynsim_2link_planar_B.h12 *
                dynsim_2link_planar_B.htmp2;
              h[dynsim_2link_planar_B.nr] -= dynsim_2link_planar_B.h12 *
                dynsim_2link_planar_B.ba;
              h[dynsim_2link_planar_B.j_o] -= dynsim_2link_planar_B.h12 *
                dynsim_2link_planar_B.aa;
              dynsim_2link_planar_B.b_j_g++;
            }

            dynsim_2link_planar_B.nr = dynsim_2link_planar_B.s_tmp + 3;
            dynsim_2link_planar_B.b_j_g = dynsim_2link_planar_B.i_n + 1;
            if (dynsim_2link_planar_B.nr < dynsim_2link_planar_B.b_j_g) {
              dynsim_2link_planar_B.b_j_g = dynsim_2link_planar_B.nr;
            }

            dynsim_2link_planar_B.c_j = 0;
            while (dynsim_2link_planar_B.c_j <= dynsim_2link_planar_B.b_j_g - 1)
            {
              dynsim_2link_planar_B.nr = ((dynsim_2link_planar_B.s_tmp - 1) << 2)
                + dynsim_2link_planar_B.c_j;
              dynsim_2link_planar_B.hoffset = (dynsim_2link_planar_B.s_tmp << 2)
                + dynsim_2link_planar_B.c_j;
              dynsim_2link_planar_B.j_o = ((dynsim_2link_planar_B.s_tmp + 1) <<
                2) + dynsim_2link_planar_B.c_j;
              dynsim_2link_planar_B.h12 = (h[dynsim_2link_planar_B.nr] +
                h[dynsim_2link_planar_B.hoffset] * dynsim_2link_planar_B.tst) +
                h[dynsim_2link_planar_B.j_o] * dynsim_2link_planar_B.htmp1;
              h[dynsim_2link_planar_B.nr] -= dynsim_2link_planar_B.h12 *
                dynsim_2link_planar_B.htmp2;
              h[dynsim_2link_planar_B.hoffset] -= dynsim_2link_planar_B.h12 *
                dynsim_2link_planar_B.ba;
              h[dynsim_2link_planar_B.j_o] -= dynsim_2link_planar_B.h12 *
                dynsim_2link_planar_B.aa;
              dynsim_2link_planar_B.c_j++;
            }

            for (dynsim_2link_planar_B.b_j_g = 0; dynsim_2link_planar_B.b_j_g <
                 4; dynsim_2link_planar_B.b_j_g++) {
              dynsim_2link_planar_B.nr = ((dynsim_2link_planar_B.s_tmp - 1) << 2)
                + dynsim_2link_planar_B.b_j_g;
              dynsim_2link_planar_B.hoffset = (dynsim_2link_planar_B.s_tmp << 2)
                + dynsim_2link_planar_B.b_j_g;
              dynsim_2link_planar_B.j_o = ((dynsim_2link_planar_B.s_tmp + 1) <<
                2) + dynsim_2link_planar_B.b_j_g;
              dynsim_2link_planar_B.h12 = (z[dynsim_2link_planar_B.nr] +
                z[dynsim_2link_planar_B.hoffset] * dynsim_2link_planar_B.tst) +
                z[dynsim_2link_planar_B.j_o] * dynsim_2link_planar_B.htmp1;
              z[dynsim_2link_planar_B.nr] -= dynsim_2link_planar_B.h12 *
                dynsim_2link_planar_B.htmp2;
              z[dynsim_2link_planar_B.hoffset] -= dynsim_2link_planar_B.h12 *
                dynsim_2link_planar_B.ba;
              z[dynsim_2link_planar_B.j_o] -= dynsim_2link_planar_B.h12 *
                dynsim_2link_planar_B.aa;
            }
            break;

           case 2:
            dynsim_2link_planar_B.j_o = dynsim_2link_planar_B.s_tmp - 1;
            while (dynsim_2link_planar_B.j_o + 1 < 5) {
              dynsim_2link_planar_B.nr = (dynsim_2link_planar_B.j_o << 2) +
                dynsim_2link_planar_B.s_tmp;
              dynsim_2link_planar_B.hoffset = dynsim_2link_planar_B.nr - 1;
              dynsim_2link_planar_B.h12 = h[dynsim_2link_planar_B.hoffset] +
                h[dynsim_2link_planar_B.nr] * dynsim_2link_planar_B.tst;
              h[dynsim_2link_planar_B.hoffset] -= dynsim_2link_planar_B.h12 *
                dynsim_2link_planar_B.htmp2;
              h[dynsim_2link_planar_B.nr] -= dynsim_2link_planar_B.h12 *
                dynsim_2link_planar_B.ba;
              dynsim_2link_planar_B.j_o++;
            }

            dynsim_2link_planar_B.j_o = 0;
            while (dynsim_2link_planar_B.j_o <= dynsim_2link_planar_B.i_n) {
              dynsim_2link_planar_B.nr = ((dynsim_2link_planar_B.s_tmp - 1) << 2)
                + dynsim_2link_planar_B.j_o;
              dynsim_2link_planar_B.hoffset = (dynsim_2link_planar_B.s_tmp << 2)
                + dynsim_2link_planar_B.j_o;
              dynsim_2link_planar_B.h12 = h[dynsim_2link_planar_B.nr] +
                h[dynsim_2link_planar_B.hoffset] * dynsim_2link_planar_B.tst;
              h[dynsim_2link_planar_B.nr] -= dynsim_2link_planar_B.h12 *
                dynsim_2link_planar_B.htmp2;
              h[dynsim_2link_planar_B.hoffset] -= dynsim_2link_planar_B.h12 *
                dynsim_2link_planar_B.ba;
              dynsim_2link_planar_B.j_o++;
            }

            for (dynsim_2link_planar_B.j_o = 0; dynsim_2link_planar_B.j_o < 4;
                 dynsim_2link_planar_B.j_o++) {
              dynsim_2link_planar_B.nr = ((dynsim_2link_planar_B.s_tmp - 1) << 2)
                + dynsim_2link_planar_B.j_o;
              dynsim_2link_planar_B.hoffset = (dynsim_2link_planar_B.s_tmp << 2)
                + dynsim_2link_planar_B.j_o;
              dynsim_2link_planar_B.h12 = z[dynsim_2link_planar_B.nr] +
                z[dynsim_2link_planar_B.hoffset] * dynsim_2link_planar_B.tst;
              z[dynsim_2link_planar_B.nr] -= dynsim_2link_planar_B.h12 *
                dynsim_2link_planar_B.htmp2;
              z[dynsim_2link_planar_B.hoffset] -= dynsim_2link_planar_B.h12 *
                dynsim_2link_planar_B.ba;
            }
            break;
          }

          dynsim_2link_planar_B.s_tmp++;
        }

        dynsim_2link_planar_B.ix++;
      }
    }

    if (!dynsim_2link_planar_B.goto150) {
      info = dynsim_2link_planar_B.i_n + 1;
      exitg1 = true;
    } else {
      if ((dynsim_2link_planar_B.i_n + 1 != dynsim_2link_planar_B.L) &&
          (dynsim_2link_planar_B.L == dynsim_2link_planar_B.i_n)) {
        dynsim_2link_planar_B.ix = (dynsim_2link_planar_B.i_n - 1) << 2;
        dynsim_2link_planar_B.k_k = dynsim_2link_planar_B.ix +
          dynsim_2link_planar_B.i_n;
        dynsim_2link_planar_B.m_n = dynsim_2link_planar_B.k_k - 1;
        dynsim_2link_planar_B.ba = h[dynsim_2link_planar_B.m_n];
        dynsim_2link_planar_B.s_tmp = dynsim_2link_planar_B.i_n << 2;
        dynsim_2link_planar_B.nr = dynsim_2link_planar_B.s_tmp +
          dynsim_2link_planar_B.i_n;
        dynsim_2link_planar_B.hoffset = dynsim_2link_planar_B.nr - 1;
        dynsim_2link_planar_B.htmp1 = h[dynsim_2link_planar_B.hoffset];
        dynsim_2link_planar_B.aa = h[dynsim_2link_planar_B.k_k];
        dynsim_2link_planar_B.h12 = h[dynsim_2link_planar_B.nr];
        dynsim_2link_planar_xdlanv2(&dynsim_2link_planar_B.ba,
          &dynsim_2link_planar_B.htmp1, &dynsim_2link_planar_B.aa,
          &dynsim_2link_planar_B.h12, &dynsim_2link_planar_B.h21s,
          &dynsim_2link_planar_B.unusedU1, &dynsim_2link_planar_B.unusedU2,
          &dynsim_2link_planar_B.unusedU3, &dynsim_2link_planar_B.htmp2,
          &dynsim_2link_planar_B.tst);
        h[dynsim_2link_planar_B.m_n] = dynsim_2link_planar_B.ba;
        h[dynsim_2link_planar_B.hoffset] = dynsim_2link_planar_B.htmp1;
        h[dynsim_2link_planar_B.k_k] = dynsim_2link_planar_B.aa;
        h[dynsim_2link_planar_B.nr] = dynsim_2link_planar_B.h12;
        if (4 > dynsim_2link_planar_B.i_n + 1) {
          dynsim_2link_planar_xrot(3 - dynsim_2link_planar_B.i_n, h,
            dynsim_2link_planar_B.i_n + ((dynsim_2link_planar_B.i_n + 1) << 2),
            (dynsim_2link_planar_B.i_n + ((dynsim_2link_planar_B.i_n + 1) << 2))
            + 1, dynsim_2link_planar_B.htmp2, dynsim_2link_planar_B.tst);
        }

        dynsim_2link_planar_xrot_l(dynsim_2link_planar_B.i_n - 1, h,
          ((dynsim_2link_planar_B.i_n - 1) << 2) + 1, (dynsim_2link_planar_B.i_n
          << 2) + 1, dynsim_2link_planar_B.htmp2, dynsim_2link_planar_B.tst);
        dynsim_2link_planar_B.ba = dynsim_2link_planar_B.htmp2 *
          z[dynsim_2link_planar_B.ix] + dynsim_2link_planar_B.tst *
          z[dynsim_2link_planar_B.s_tmp];
        z[dynsim_2link_planar_B.s_tmp] = dynsim_2link_planar_B.htmp2 *
          z[dynsim_2link_planar_B.s_tmp] - dynsim_2link_planar_B.tst *
          z[dynsim_2link_planar_B.ix];
        z[dynsim_2link_planar_B.ix] = dynsim_2link_planar_B.ba;
        dynsim_2link_planar_B.i_n = dynsim_2link_planar_B.s_tmp + 1;
        dynsim_2link_planar_B.ix++;
        dynsim_2link_planar_B.ba = dynsim_2link_planar_B.htmp2 *
          z[dynsim_2link_planar_B.ix] + dynsim_2link_planar_B.tst *
          z[dynsim_2link_planar_B.i_n];
        z[dynsim_2link_planar_B.i_n] = dynsim_2link_planar_B.htmp2 *
          z[dynsim_2link_planar_B.i_n] - dynsim_2link_planar_B.tst *
          z[dynsim_2link_planar_B.ix];
        z[dynsim_2link_planar_B.ix] = dynsim_2link_planar_B.ba;
        dynsim_2link_planar_B.i_n++;
        dynsim_2link_planar_B.ix++;
        dynsim_2link_planar_B.ba = dynsim_2link_planar_B.htmp2 *
          z[dynsim_2link_planar_B.ix] + dynsim_2link_planar_B.tst *
          z[dynsim_2link_planar_B.i_n];
        z[dynsim_2link_planar_B.i_n] = dynsim_2link_planar_B.htmp2 *
          z[dynsim_2link_planar_B.i_n] - dynsim_2link_planar_B.tst *
          z[dynsim_2link_planar_B.ix];
        z[dynsim_2link_planar_B.ix] = dynsim_2link_planar_B.ba;
        dynsim_2link_planar_B.i_n++;
        dynsim_2link_planar_B.ix++;
        dynsim_2link_planar_B.ba = dynsim_2link_planar_B.htmp2 *
          z[dynsim_2link_planar_B.ix] + dynsim_2link_planar_B.tst *
          z[dynsim_2link_planar_B.i_n];
        z[dynsim_2link_planar_B.i_n] = dynsim_2link_planar_B.htmp2 *
          z[dynsim_2link_planar_B.i_n] - dynsim_2link_planar_B.tst *
          z[dynsim_2link_planar_B.ix];
        z[dynsim_2link_planar_B.ix] = dynsim_2link_planar_B.ba;
      }

      dynsim_2link_planar_B.i_n = dynsim_2link_planar_B.L - 2;
    }
  }

  return info;
}

// Function for MATLAB Function: '<S4>/MATLAB Function'
static void dynsim_2link_planar_eig(const real_T A[16], creal_T V[16], creal_T
  D[4])
{
  int32_T exitg1;
  boolean_T exitg2;
  if (dynsim_2link_plana_anyNonFinite(A)) {
    for (dynsim_2link_planar_B.b_j_m = 0; dynsim_2link_planar_B.b_j_m < 16;
         dynsim_2link_planar_B.b_j_m++) {
      V[dynsim_2link_planar_B.b_j_m].re = (rtNaN);
      V[dynsim_2link_planar_B.b_j_m].im = 0.0;
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
    dynsim_2link_planar_B.p_n = true;
    dynsim_2link_planar_B.b_j_m = 0;
    exitg2 = false;
    while ((!exitg2) && (dynsim_2link_planar_B.b_j_m < 4)) {
      dynsim_2link_planar_B.i_j = 0;
      do {
        exitg1 = 0;
        if (dynsim_2link_planar_B.i_j <= dynsim_2link_planar_B.b_j_m) {
          if (!(A[(dynsim_2link_planar_B.b_j_m << 2) + dynsim_2link_planar_B.i_j]
                == A[(dynsim_2link_planar_B.i_j << 2) +
                dynsim_2link_planar_B.b_j_m])) {
            dynsim_2link_planar_B.p_n = false;
            exitg1 = 1;
          } else {
            dynsim_2link_planar_B.i_j++;
          }
        } else {
          dynsim_2link_planar_B.b_j_m++;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }

    if (dynsim_2link_planar_B.p_n) {
      if (dynsim_2link_plana_anyNonFinite(A)) {
        for (dynsim_2link_planar_B.b_j_m = 0; dynsim_2link_planar_B.b_j_m < 16;
             dynsim_2link_planar_B.b_j_m++) {
          dynsim_2link_planar_B.b_V[dynsim_2link_planar_B.b_j_m] = (rtNaN);
        }

        dynsim_2link_planar_B.b_j_m = 2;
        while (dynsim_2link_planar_B.b_j_m < 5) {
          dynsim_2link_planar_B.b_V[dynsim_2link_planar_B.b_j_m - 1] = 0.0;
          dynsim_2link_planar_B.b_j_m++;
        }

        dynsim_2link_planar_B.b_j_m = 3;
        while (dynsim_2link_planar_B.b_j_m < 5) {
          dynsim_2link_planar_B.b_V[dynsim_2link_planar_B.b_j_m + 3] = 0.0;
          dynsim_2link_planar_B.b_j_m++;
        }

        dynsim_2link_planar_B.b_V[11] = 0.0;
        for (dynsim_2link_planar_B.b_j_m = 0; dynsim_2link_planar_B.b_j_m < 16;
             dynsim_2link_planar_B.b_j_m++) {
          dynsim_2link_planar_B.b_A[dynsim_2link_planar_B.b_j_m] = (rtNaN);
        }
      } else {
        memcpy(&dynsim_2link_planar_B.b_A[0], &A[0], sizeof(real_T) << 4U);
        dynsim_2link_planar_xgehrd(dynsim_2link_planar_B.b_A,
          dynsim_2link_planar_B.tau);
        memcpy(&dynsim_2link_planar_B.b_V[0], &dynsim_2link_planar_B.b_A[0],
               sizeof(real_T) << 4U);
        dynsim_2link_planar_B.b_j_m = 0;
        while (dynsim_2link_planar_B.b_j_m <= 2) {
          dynsim_2link_planar_B.b_V[dynsim_2link_planar_B.b_j_m + 12] = 0.0;
          dynsim_2link_planar_B.b_j_m++;
        }

        dynsim_2link_planar_B.b_j_m = 0;
        while (dynsim_2link_planar_B.b_j_m <= 1) {
          dynsim_2link_planar_B.b_V[dynsim_2link_planar_B.b_j_m + 8] = 0.0;
          dynsim_2link_planar_B.b_j_m++;
        }

        dynsim_2link_planar_B.b_j_m = 1;
        while (dynsim_2link_planar_B.b_j_m + 3 < 5) {
          dynsim_2link_planar_B.b_V[dynsim_2link_planar_B.b_j_m + 10] =
            dynsim_2link_planar_B.b_V[dynsim_2link_planar_B.b_j_m + 6];
          dynsim_2link_planar_B.b_j_m++;
        }

        dynsim_2link_planar_B.b_V[4] = 0.0;
        dynsim_2link_planar_B.b_j_m = 0;
        while (dynsim_2link_planar_B.b_j_m + 3 < 5) {
          dynsim_2link_planar_B.b_V[dynsim_2link_planar_B.b_j_m + 6] =
            dynsim_2link_planar_B.b_V[dynsim_2link_planar_B.b_j_m + 2];
          dynsim_2link_planar_B.b_j_m++;
        }

        dynsim_2link_planar_B.work[0] = 0.0;
        dynsim_2link_planar_B.b_V[1] = 0.0;
        dynsim_2link_planar_B.work[1] = 0.0;
        dynsim_2link_planar_B.b_V[2] = 0.0;
        dynsim_2link_planar_B.work[2] = 0.0;
        dynsim_2link_planar_B.b_V[3] = 0.0;
        dynsim_2link_planar_B.work[3] = 0.0;
        dynsim_2link_planar_B.b_V[0] = 1.0;
        dynsim_2link_planar_B.b_V[15] = 1.0 - dynsim_2link_planar_B.tau[2];
        dynsim_2link_planar_B.b_j_m = 0;
        while (dynsim_2link_planar_B.b_j_m <= 1) {
          dynsim_2link_planar_B.b_V[14 - dynsim_2link_planar_B.b_j_m] = 0.0;
          dynsim_2link_planar_B.b_j_m++;
        }

        dynsim_2link_planar_B.b_V[10] = 1.0;
        dynsim_2link_planar_xzlarf(2, 1, 11, dynsim_2link_planar_B.tau[1],
          dynsim_2link_planar_B.b_V, 15, dynsim_2link_planar_B.work);
        dynsim_2link_planar_B.b_j_m = 11;
        while (dynsim_2link_planar_B.b_j_m + 1 <= 12) {
          dynsim_2link_planar_B.b_V[dynsim_2link_planar_B.b_j_m] *=
            -dynsim_2link_planar_B.tau[1];
          dynsim_2link_planar_B.b_j_m++;
        }

        dynsim_2link_planar_B.b_V[10] = 1.0 - dynsim_2link_planar_B.tau[1];
        dynsim_2link_planar_B.b_V[9] = 0.0;
        dynsim_2link_planar_B.b_V[5] = 1.0;
        dynsim_2link_planar_xzlarf(3, 2, 6, dynsim_2link_planar_B.tau[0],
          dynsim_2link_planar_B.b_V, 10, dynsim_2link_planar_B.work);
        dynsim_2link_planar_B.b_j_m = 6;
        while (dynsim_2link_planar_B.b_j_m + 1 <= 8) {
          dynsim_2link_planar_B.b_V[dynsim_2link_planar_B.b_j_m] *=
            -dynsim_2link_planar_B.tau[0];
          dynsim_2link_planar_B.b_j_m++;
        }

        dynsim_2link_planar_B.b_V[5] = 1.0 - dynsim_2link_planar_B.tau[0];
        dynsim_2link_planar_eml_dlahqr(dynsim_2link_planar_B.b_A,
          dynsim_2link_planar_B.b_V);
      }

      for (dynsim_2link_planar_B.b_j_m = 0; dynsim_2link_planar_B.b_j_m < 16;
           dynsim_2link_planar_B.b_j_m++) {
        V[dynsim_2link_planar_B.b_j_m].re =
          dynsim_2link_planar_B.b_V[dynsim_2link_planar_B.b_j_m];
        V[dynsim_2link_planar_B.b_j_m].im = 0.0;
      }

      D[0].re = dynsim_2link_planar_B.b_A[0];
      D[0].im = 0.0;
      D[1].re = dynsim_2link_planar_B.b_A[5];
      D[1].im = 0.0;
      D[2].re = dynsim_2link_planar_B.b_A[10];
      D[2].im = 0.0;
      D[3].re = dynsim_2link_planar_B.b_A[15];
      D[3].im = 0.0;
    } else {
      for (dynsim_2link_planar_B.b_j_m = 0; dynsim_2link_planar_B.b_j_m < 16;
           dynsim_2link_planar_B.b_j_m++) {
        dynsim_2link_planar_B.At[dynsim_2link_planar_B.b_j_m].re =
          A[dynsim_2link_planar_B.b_j_m];
        dynsim_2link_planar_B.At[dynsim_2link_planar_B.b_j_m].im = 0.0;
      }

      dynsim_2link_planar_xzggev(dynsim_2link_planar_B.At,
        &dynsim_2link_planar_B.b_j_m, D, dynsim_2link_planar_B.beta1, V);
      dynsim_2link_planar_B.colnorm = 0.0;
      dynsim_2link_planar_B.scale = 3.3121686421112381E-170;
      dynsim_2link_planar_B.b_j_m = 0;
      while (dynsim_2link_planar_B.b_j_m + 1 <= 4) {
        dynsim_2link_planar_B.absxk = fabs(V[dynsim_2link_planar_B.b_j_m].re);
        if (dynsim_2link_planar_B.absxk > dynsim_2link_planar_B.scale) {
          dynsim_2link_planar_B.t = dynsim_2link_planar_B.scale /
            dynsim_2link_planar_B.absxk;
          dynsim_2link_planar_B.colnorm = dynsim_2link_planar_B.colnorm *
            dynsim_2link_planar_B.t * dynsim_2link_planar_B.t + 1.0;
          dynsim_2link_planar_B.scale = dynsim_2link_planar_B.absxk;
        } else {
          dynsim_2link_planar_B.t = dynsim_2link_planar_B.absxk /
            dynsim_2link_planar_B.scale;
          dynsim_2link_planar_B.colnorm += dynsim_2link_planar_B.t *
            dynsim_2link_planar_B.t;
        }

        dynsim_2link_planar_B.absxk = fabs(V[dynsim_2link_planar_B.b_j_m].im);
        if (dynsim_2link_planar_B.absxk > dynsim_2link_planar_B.scale) {
          dynsim_2link_planar_B.t = dynsim_2link_planar_B.scale /
            dynsim_2link_planar_B.absxk;
          dynsim_2link_planar_B.colnorm = dynsim_2link_planar_B.colnorm *
            dynsim_2link_planar_B.t * dynsim_2link_planar_B.t + 1.0;
          dynsim_2link_planar_B.scale = dynsim_2link_planar_B.absxk;
        } else {
          dynsim_2link_planar_B.t = dynsim_2link_planar_B.absxk /
            dynsim_2link_planar_B.scale;
          dynsim_2link_planar_B.colnorm += dynsim_2link_planar_B.t *
            dynsim_2link_planar_B.t;
        }

        dynsim_2link_planar_B.b_j_m++;
      }

      dynsim_2link_planar_B.colnorm = dynsim_2link_planar_B.scale * sqrt
        (dynsim_2link_planar_B.colnorm);
      dynsim_2link_planar_B.b_j_m = 0;
      while (dynsim_2link_planar_B.b_j_m + 1 <= 4) {
        if (V[dynsim_2link_planar_B.b_j_m].im == 0.0) {
          dynsim_2link_planar_B.scale = V[dynsim_2link_planar_B.b_j_m].re /
            dynsim_2link_planar_B.colnorm;
          dynsim_2link_planar_B.absxk = 0.0;
        } else if (V[dynsim_2link_planar_B.b_j_m].re == 0.0) {
          dynsim_2link_planar_B.scale = 0.0;
          dynsim_2link_planar_B.absxk = V[dynsim_2link_planar_B.b_j_m].im /
            dynsim_2link_planar_B.colnorm;
        } else {
          dynsim_2link_planar_B.scale = V[dynsim_2link_planar_B.b_j_m].re /
            dynsim_2link_planar_B.colnorm;
          dynsim_2link_planar_B.absxk = V[dynsim_2link_planar_B.b_j_m].im /
            dynsim_2link_planar_B.colnorm;
        }

        V[dynsim_2link_planar_B.b_j_m].re = dynsim_2link_planar_B.scale;
        V[dynsim_2link_planar_B.b_j_m].im = dynsim_2link_planar_B.absxk;
        dynsim_2link_planar_B.b_j_m++;
      }

      dynsim_2link_planar_B.colnorm = 0.0;
      dynsim_2link_planar_B.scale = 3.3121686421112381E-170;
      dynsim_2link_planar_B.b_j_m = 4;
      while (dynsim_2link_planar_B.b_j_m + 1 <= 8) {
        dynsim_2link_planar_B.absxk = fabs(V[dynsim_2link_planar_B.b_j_m].re);
        if (dynsim_2link_planar_B.absxk > dynsim_2link_planar_B.scale) {
          dynsim_2link_planar_B.t = dynsim_2link_planar_B.scale /
            dynsim_2link_planar_B.absxk;
          dynsim_2link_planar_B.colnorm = dynsim_2link_planar_B.colnorm *
            dynsim_2link_planar_B.t * dynsim_2link_planar_B.t + 1.0;
          dynsim_2link_planar_B.scale = dynsim_2link_planar_B.absxk;
        } else {
          dynsim_2link_planar_B.t = dynsim_2link_planar_B.absxk /
            dynsim_2link_planar_B.scale;
          dynsim_2link_planar_B.colnorm += dynsim_2link_planar_B.t *
            dynsim_2link_planar_B.t;
        }

        dynsim_2link_planar_B.absxk = fabs(V[dynsim_2link_planar_B.b_j_m].im);
        if (dynsim_2link_planar_B.absxk > dynsim_2link_planar_B.scale) {
          dynsim_2link_planar_B.t = dynsim_2link_planar_B.scale /
            dynsim_2link_planar_B.absxk;
          dynsim_2link_planar_B.colnorm = dynsim_2link_planar_B.colnorm *
            dynsim_2link_planar_B.t * dynsim_2link_planar_B.t + 1.0;
          dynsim_2link_planar_B.scale = dynsim_2link_planar_B.absxk;
        } else {
          dynsim_2link_planar_B.t = dynsim_2link_planar_B.absxk /
            dynsim_2link_planar_B.scale;
          dynsim_2link_planar_B.colnorm += dynsim_2link_planar_B.t *
            dynsim_2link_planar_B.t;
        }

        dynsim_2link_planar_B.b_j_m++;
      }

      dynsim_2link_planar_B.colnorm = dynsim_2link_planar_B.scale * sqrt
        (dynsim_2link_planar_B.colnorm);
      dynsim_2link_planar_B.b_j_m = 4;
      while (dynsim_2link_planar_B.b_j_m + 1 <= 8) {
        if (V[dynsim_2link_planar_B.b_j_m].im == 0.0) {
          dynsim_2link_planar_B.scale = V[dynsim_2link_planar_B.b_j_m].re /
            dynsim_2link_planar_B.colnorm;
          dynsim_2link_planar_B.absxk = 0.0;
        } else if (V[dynsim_2link_planar_B.b_j_m].re == 0.0) {
          dynsim_2link_planar_B.scale = 0.0;
          dynsim_2link_planar_B.absxk = V[dynsim_2link_planar_B.b_j_m].im /
            dynsim_2link_planar_B.colnorm;
        } else {
          dynsim_2link_planar_B.scale = V[dynsim_2link_planar_B.b_j_m].re /
            dynsim_2link_planar_B.colnorm;
          dynsim_2link_planar_B.absxk = V[dynsim_2link_planar_B.b_j_m].im /
            dynsim_2link_planar_B.colnorm;
        }

        V[dynsim_2link_planar_B.b_j_m].re = dynsim_2link_planar_B.scale;
        V[dynsim_2link_planar_B.b_j_m].im = dynsim_2link_planar_B.absxk;
        dynsim_2link_planar_B.b_j_m++;
      }

      dynsim_2link_planar_B.colnorm = 0.0;
      dynsim_2link_planar_B.scale = 3.3121686421112381E-170;
      dynsim_2link_planar_B.b_j_m = 8;
      while (dynsim_2link_planar_B.b_j_m + 1 <= 12) {
        dynsim_2link_planar_B.absxk = fabs(V[dynsim_2link_planar_B.b_j_m].re);
        if (dynsim_2link_planar_B.absxk > dynsim_2link_planar_B.scale) {
          dynsim_2link_planar_B.t = dynsim_2link_planar_B.scale /
            dynsim_2link_planar_B.absxk;
          dynsim_2link_planar_B.colnorm = dynsim_2link_planar_B.colnorm *
            dynsim_2link_planar_B.t * dynsim_2link_planar_B.t + 1.0;
          dynsim_2link_planar_B.scale = dynsim_2link_planar_B.absxk;
        } else {
          dynsim_2link_planar_B.t = dynsim_2link_planar_B.absxk /
            dynsim_2link_planar_B.scale;
          dynsim_2link_planar_B.colnorm += dynsim_2link_planar_B.t *
            dynsim_2link_planar_B.t;
        }

        dynsim_2link_planar_B.absxk = fabs(V[dynsim_2link_planar_B.b_j_m].im);
        if (dynsim_2link_planar_B.absxk > dynsim_2link_planar_B.scale) {
          dynsim_2link_planar_B.t = dynsim_2link_planar_B.scale /
            dynsim_2link_planar_B.absxk;
          dynsim_2link_planar_B.colnorm = dynsim_2link_planar_B.colnorm *
            dynsim_2link_planar_B.t * dynsim_2link_planar_B.t + 1.0;
          dynsim_2link_planar_B.scale = dynsim_2link_planar_B.absxk;
        } else {
          dynsim_2link_planar_B.t = dynsim_2link_planar_B.absxk /
            dynsim_2link_planar_B.scale;
          dynsim_2link_planar_B.colnorm += dynsim_2link_planar_B.t *
            dynsim_2link_planar_B.t;
        }

        dynsim_2link_planar_B.b_j_m++;
      }

      dynsim_2link_planar_B.colnorm = dynsim_2link_planar_B.scale * sqrt
        (dynsim_2link_planar_B.colnorm);
      dynsim_2link_planar_B.b_j_m = 8;
      while (dynsim_2link_planar_B.b_j_m + 1 <= 12) {
        if (V[dynsim_2link_planar_B.b_j_m].im == 0.0) {
          dynsim_2link_planar_B.scale = V[dynsim_2link_planar_B.b_j_m].re /
            dynsim_2link_planar_B.colnorm;
          dynsim_2link_planar_B.absxk = 0.0;
        } else if (V[dynsim_2link_planar_B.b_j_m].re == 0.0) {
          dynsim_2link_planar_B.scale = 0.0;
          dynsim_2link_planar_B.absxk = V[dynsim_2link_planar_B.b_j_m].im /
            dynsim_2link_planar_B.colnorm;
        } else {
          dynsim_2link_planar_B.scale = V[dynsim_2link_planar_B.b_j_m].re /
            dynsim_2link_planar_B.colnorm;
          dynsim_2link_planar_B.absxk = V[dynsim_2link_planar_B.b_j_m].im /
            dynsim_2link_planar_B.colnorm;
        }

        V[dynsim_2link_planar_B.b_j_m].re = dynsim_2link_planar_B.scale;
        V[dynsim_2link_planar_B.b_j_m].im = dynsim_2link_planar_B.absxk;
        dynsim_2link_planar_B.b_j_m++;
      }

      dynsim_2link_planar_B.colnorm = 0.0;
      dynsim_2link_planar_B.scale = 3.3121686421112381E-170;
      dynsim_2link_planar_B.b_j_m = 12;
      while (dynsim_2link_planar_B.b_j_m + 1 <= 16) {
        dynsim_2link_planar_B.absxk = fabs(V[dynsim_2link_planar_B.b_j_m].re);
        if (dynsim_2link_planar_B.absxk > dynsim_2link_planar_B.scale) {
          dynsim_2link_planar_B.t = dynsim_2link_planar_B.scale /
            dynsim_2link_planar_B.absxk;
          dynsim_2link_planar_B.colnorm = dynsim_2link_planar_B.colnorm *
            dynsim_2link_planar_B.t * dynsim_2link_planar_B.t + 1.0;
          dynsim_2link_planar_B.scale = dynsim_2link_planar_B.absxk;
        } else {
          dynsim_2link_planar_B.t = dynsim_2link_planar_B.absxk /
            dynsim_2link_planar_B.scale;
          dynsim_2link_planar_B.colnorm += dynsim_2link_planar_B.t *
            dynsim_2link_planar_B.t;
        }

        dynsim_2link_planar_B.absxk = fabs(V[dynsim_2link_planar_B.b_j_m].im);
        if (dynsim_2link_planar_B.absxk > dynsim_2link_planar_B.scale) {
          dynsim_2link_planar_B.t = dynsim_2link_planar_B.scale /
            dynsim_2link_planar_B.absxk;
          dynsim_2link_planar_B.colnorm = dynsim_2link_planar_B.colnorm *
            dynsim_2link_planar_B.t * dynsim_2link_planar_B.t + 1.0;
          dynsim_2link_planar_B.scale = dynsim_2link_planar_B.absxk;
        } else {
          dynsim_2link_planar_B.t = dynsim_2link_planar_B.absxk /
            dynsim_2link_planar_B.scale;
          dynsim_2link_planar_B.colnorm += dynsim_2link_planar_B.t *
            dynsim_2link_planar_B.t;
        }

        dynsim_2link_planar_B.b_j_m++;
      }

      dynsim_2link_planar_B.colnorm = dynsim_2link_planar_B.scale * sqrt
        (dynsim_2link_planar_B.colnorm);
      dynsim_2link_planar_B.b_j_m = 12;
      while (dynsim_2link_planar_B.b_j_m + 1 <= 16) {
        if (V[dynsim_2link_planar_B.b_j_m].im == 0.0) {
          dynsim_2link_planar_B.scale = V[dynsim_2link_planar_B.b_j_m].re /
            dynsim_2link_planar_B.colnorm;
          dynsim_2link_planar_B.absxk = 0.0;
        } else if (V[dynsim_2link_planar_B.b_j_m].re == 0.0) {
          dynsim_2link_planar_B.scale = 0.0;
          dynsim_2link_planar_B.absxk = V[dynsim_2link_planar_B.b_j_m].im /
            dynsim_2link_planar_B.colnorm;
        } else {
          dynsim_2link_planar_B.scale = V[dynsim_2link_planar_B.b_j_m].re /
            dynsim_2link_planar_B.colnorm;
          dynsim_2link_planar_B.absxk = V[dynsim_2link_planar_B.b_j_m].im /
            dynsim_2link_planar_B.colnorm;
        }

        V[dynsim_2link_planar_B.b_j_m].re = dynsim_2link_planar_B.scale;
        V[dynsim_2link_planar_B.b_j_m].im = dynsim_2link_planar_B.absxk;
        dynsim_2link_planar_B.b_j_m++;
      }

      if (dynsim_2link_planar_B.beta1[0].im == 0.0) {
        if (D[0].im == 0.0) {
          dynsim_2link_planar_B.scale = D[0].re / dynsim_2link_planar_B.beta1[0]
            .re;
          dynsim_2link_planar_B.absxk = 0.0;
        } else if (D[0].re == 0.0) {
          dynsim_2link_planar_B.scale = 0.0;
          dynsim_2link_planar_B.absxk = D[0].im / dynsim_2link_planar_B.beta1[0]
            .re;
        } else {
          dynsim_2link_planar_B.scale = D[0].re / dynsim_2link_planar_B.beta1[0]
            .re;
          dynsim_2link_planar_B.absxk = D[0].im / dynsim_2link_planar_B.beta1[0]
            .re;
        }
      } else if (dynsim_2link_planar_B.beta1[0].re == 0.0) {
        if (D[0].re == 0.0) {
          dynsim_2link_planar_B.scale = D[0].im / dynsim_2link_planar_B.beta1[0]
            .im;
          dynsim_2link_planar_B.absxk = 0.0;
        } else if (D[0].im == 0.0) {
          dynsim_2link_planar_B.scale = 0.0;
          dynsim_2link_planar_B.absxk = -(D[0].re / dynsim_2link_planar_B.beta1
            [0].im);
        } else {
          dynsim_2link_planar_B.scale = D[0].im / dynsim_2link_planar_B.beta1[0]
            .im;
          dynsim_2link_planar_B.absxk = -(D[0].re / dynsim_2link_planar_B.beta1
            [0].im);
        }
      } else {
        dynsim_2link_planar_B.colnorm = fabs(dynsim_2link_planar_B.beta1[0].re);
        dynsim_2link_planar_B.scale = fabs(dynsim_2link_planar_B.beta1[0].im);
        if (dynsim_2link_planar_B.colnorm > dynsim_2link_planar_B.scale) {
          dynsim_2link_planar_B.colnorm = dynsim_2link_planar_B.beta1[0].im /
            dynsim_2link_planar_B.beta1[0].re;
          dynsim_2link_planar_B.absxk = dynsim_2link_planar_B.colnorm *
            dynsim_2link_planar_B.beta1[0].im + dynsim_2link_planar_B.beta1[0].
            re;
          dynsim_2link_planar_B.scale = (dynsim_2link_planar_B.colnorm * D[0].im
            + D[0].re) / dynsim_2link_planar_B.absxk;
          dynsim_2link_planar_B.absxk = (D[0].im - dynsim_2link_planar_B.colnorm
            * D[0].re) / dynsim_2link_planar_B.absxk;
        } else if (dynsim_2link_planar_B.scale == dynsim_2link_planar_B.colnorm)
        {
          dynsim_2link_planar_B.absxk = dynsim_2link_planar_B.beta1[0].re > 0.0 ?
            0.5 : -0.5;
          dynsim_2link_planar_B.t = dynsim_2link_planar_B.beta1[0].im > 0.0 ?
            0.5 : -0.5;
          dynsim_2link_planar_B.scale = (D[0].re * dynsim_2link_planar_B.absxk +
            D[0].im * dynsim_2link_planar_B.t) / dynsim_2link_planar_B.colnorm;
          dynsim_2link_planar_B.absxk = (D[0].im * dynsim_2link_planar_B.absxk -
            D[0].re * dynsim_2link_planar_B.t) / dynsim_2link_planar_B.colnorm;
        } else {
          dynsim_2link_planar_B.colnorm = dynsim_2link_planar_B.beta1[0].re /
            dynsim_2link_planar_B.beta1[0].im;
          dynsim_2link_planar_B.absxk = dynsim_2link_planar_B.colnorm *
            dynsim_2link_planar_B.beta1[0].re + dynsim_2link_planar_B.beta1[0].
            im;
          dynsim_2link_planar_B.scale = (dynsim_2link_planar_B.colnorm * D[0].re
            + D[0].im) / dynsim_2link_planar_B.absxk;
          dynsim_2link_planar_B.absxk = (dynsim_2link_planar_B.colnorm * D[0].im
            - D[0].re) / dynsim_2link_planar_B.absxk;
        }
      }

      D[0].re = dynsim_2link_planar_B.scale;
      D[0].im = dynsim_2link_planar_B.absxk;
      if (dynsim_2link_planar_B.beta1[1].im == 0.0) {
        if (D[1].im == 0.0) {
          dynsim_2link_planar_B.scale = D[1].re / dynsim_2link_planar_B.beta1[1]
            .re;
          dynsim_2link_planar_B.absxk = 0.0;
        } else if (D[1].re == 0.0) {
          dynsim_2link_planar_B.scale = 0.0;
          dynsim_2link_planar_B.absxk = D[1].im / dynsim_2link_planar_B.beta1[1]
            .re;
        } else {
          dynsim_2link_planar_B.scale = D[1].re / dynsim_2link_planar_B.beta1[1]
            .re;
          dynsim_2link_planar_B.absxk = D[1].im / dynsim_2link_planar_B.beta1[1]
            .re;
        }
      } else if (dynsim_2link_planar_B.beta1[1].re == 0.0) {
        if (D[1].re == 0.0) {
          dynsim_2link_planar_B.scale = D[1].im / dynsim_2link_planar_B.beta1[1]
            .im;
          dynsim_2link_planar_B.absxk = 0.0;
        } else if (D[1].im == 0.0) {
          dynsim_2link_planar_B.scale = 0.0;
          dynsim_2link_planar_B.absxk = -(D[1].re / dynsim_2link_planar_B.beta1
            [1].im);
        } else {
          dynsim_2link_planar_B.scale = D[1].im / dynsim_2link_planar_B.beta1[1]
            .im;
          dynsim_2link_planar_B.absxk = -(D[1].re / dynsim_2link_planar_B.beta1
            [1].im);
        }
      } else {
        dynsim_2link_planar_B.colnorm = fabs(dynsim_2link_planar_B.beta1[1].re);
        dynsim_2link_planar_B.scale = fabs(dynsim_2link_planar_B.beta1[1].im);
        if (dynsim_2link_planar_B.colnorm > dynsim_2link_planar_B.scale) {
          dynsim_2link_planar_B.colnorm = dynsim_2link_planar_B.beta1[1].im /
            dynsim_2link_planar_B.beta1[1].re;
          dynsim_2link_planar_B.absxk = dynsim_2link_planar_B.colnorm *
            dynsim_2link_planar_B.beta1[1].im + dynsim_2link_planar_B.beta1[1].
            re;
          dynsim_2link_planar_B.scale = (dynsim_2link_planar_B.colnorm * D[1].im
            + D[1].re) / dynsim_2link_planar_B.absxk;
          dynsim_2link_planar_B.absxk = (D[1].im - dynsim_2link_planar_B.colnorm
            * D[1].re) / dynsim_2link_planar_B.absxk;
        } else if (dynsim_2link_planar_B.scale == dynsim_2link_planar_B.colnorm)
        {
          dynsim_2link_planar_B.absxk = dynsim_2link_planar_B.beta1[1].re > 0.0 ?
            0.5 : -0.5;
          dynsim_2link_planar_B.t = dynsim_2link_planar_B.beta1[1].im > 0.0 ?
            0.5 : -0.5;
          dynsim_2link_planar_B.scale = (D[1].re * dynsim_2link_planar_B.absxk +
            D[1].im * dynsim_2link_planar_B.t) / dynsim_2link_planar_B.colnorm;
          dynsim_2link_planar_B.absxk = (D[1].im * dynsim_2link_planar_B.absxk -
            D[1].re * dynsim_2link_planar_B.t) / dynsim_2link_planar_B.colnorm;
        } else {
          dynsim_2link_planar_B.colnorm = dynsim_2link_planar_B.beta1[1].re /
            dynsim_2link_planar_B.beta1[1].im;
          dynsim_2link_planar_B.absxk = dynsim_2link_planar_B.colnorm *
            dynsim_2link_planar_B.beta1[1].re + dynsim_2link_planar_B.beta1[1].
            im;
          dynsim_2link_planar_B.scale = (dynsim_2link_planar_B.colnorm * D[1].re
            + D[1].im) / dynsim_2link_planar_B.absxk;
          dynsim_2link_planar_B.absxk = (dynsim_2link_planar_B.colnorm * D[1].im
            - D[1].re) / dynsim_2link_planar_B.absxk;
        }
      }

      D[1].re = dynsim_2link_planar_B.scale;
      D[1].im = dynsim_2link_planar_B.absxk;
      if (dynsim_2link_planar_B.beta1[2].im == 0.0) {
        if (D[2].im == 0.0) {
          dynsim_2link_planar_B.scale = D[2].re / dynsim_2link_planar_B.beta1[2]
            .re;
          dynsim_2link_planar_B.absxk = 0.0;
        } else if (D[2].re == 0.0) {
          dynsim_2link_planar_B.scale = 0.0;
          dynsim_2link_planar_B.absxk = D[2].im / dynsim_2link_planar_B.beta1[2]
            .re;
        } else {
          dynsim_2link_planar_B.scale = D[2].re / dynsim_2link_planar_B.beta1[2]
            .re;
          dynsim_2link_planar_B.absxk = D[2].im / dynsim_2link_planar_B.beta1[2]
            .re;
        }
      } else if (dynsim_2link_planar_B.beta1[2].re == 0.0) {
        if (D[2].re == 0.0) {
          dynsim_2link_planar_B.scale = D[2].im / dynsim_2link_planar_B.beta1[2]
            .im;
          dynsim_2link_planar_B.absxk = 0.0;
        } else if (D[2].im == 0.0) {
          dynsim_2link_planar_B.scale = 0.0;
          dynsim_2link_planar_B.absxk = -(D[2].re / dynsim_2link_planar_B.beta1
            [2].im);
        } else {
          dynsim_2link_planar_B.scale = D[2].im / dynsim_2link_planar_B.beta1[2]
            .im;
          dynsim_2link_planar_B.absxk = -(D[2].re / dynsim_2link_planar_B.beta1
            [2].im);
        }
      } else {
        dynsim_2link_planar_B.colnorm = fabs(dynsim_2link_planar_B.beta1[2].re);
        dynsim_2link_planar_B.scale = fabs(dynsim_2link_planar_B.beta1[2].im);
        if (dynsim_2link_planar_B.colnorm > dynsim_2link_planar_B.scale) {
          dynsim_2link_planar_B.colnorm = dynsim_2link_planar_B.beta1[2].im /
            dynsim_2link_planar_B.beta1[2].re;
          dynsim_2link_planar_B.absxk = dynsim_2link_planar_B.colnorm *
            dynsim_2link_planar_B.beta1[2].im + dynsim_2link_planar_B.beta1[2].
            re;
          dynsim_2link_planar_B.scale = (dynsim_2link_planar_B.colnorm * D[2].im
            + D[2].re) / dynsim_2link_planar_B.absxk;
          dynsim_2link_planar_B.absxk = (D[2].im - dynsim_2link_planar_B.colnorm
            * D[2].re) / dynsim_2link_planar_B.absxk;
        } else if (dynsim_2link_planar_B.scale == dynsim_2link_planar_B.colnorm)
        {
          dynsim_2link_planar_B.absxk = dynsim_2link_planar_B.beta1[2].re > 0.0 ?
            0.5 : -0.5;
          dynsim_2link_planar_B.t = dynsim_2link_planar_B.beta1[2].im > 0.0 ?
            0.5 : -0.5;
          dynsim_2link_planar_B.scale = (D[2].re * dynsim_2link_planar_B.absxk +
            D[2].im * dynsim_2link_planar_B.t) / dynsim_2link_planar_B.colnorm;
          dynsim_2link_planar_B.absxk = (D[2].im * dynsim_2link_planar_B.absxk -
            D[2].re * dynsim_2link_planar_B.t) / dynsim_2link_planar_B.colnorm;
        } else {
          dynsim_2link_planar_B.colnorm = dynsim_2link_planar_B.beta1[2].re /
            dynsim_2link_planar_B.beta1[2].im;
          dynsim_2link_planar_B.absxk = dynsim_2link_planar_B.colnorm *
            dynsim_2link_planar_B.beta1[2].re + dynsim_2link_planar_B.beta1[2].
            im;
          dynsim_2link_planar_B.scale = (dynsim_2link_planar_B.colnorm * D[2].re
            + D[2].im) / dynsim_2link_planar_B.absxk;
          dynsim_2link_planar_B.absxk = (dynsim_2link_planar_B.colnorm * D[2].im
            - D[2].re) / dynsim_2link_planar_B.absxk;
        }
      }

      D[2].re = dynsim_2link_planar_B.scale;
      D[2].im = dynsim_2link_planar_B.absxk;
      if (dynsim_2link_planar_B.beta1[3].im == 0.0) {
        if (D[3].im == 0.0) {
          dynsim_2link_planar_B.scale = D[3].re / dynsim_2link_planar_B.beta1[3]
            .re;
          dynsim_2link_planar_B.absxk = 0.0;
        } else if (D[3].re == 0.0) {
          dynsim_2link_planar_B.scale = 0.0;
          dynsim_2link_planar_B.absxk = D[3].im / dynsim_2link_planar_B.beta1[3]
            .re;
        } else {
          dynsim_2link_planar_B.scale = D[3].re / dynsim_2link_planar_B.beta1[3]
            .re;
          dynsim_2link_planar_B.absxk = D[3].im / dynsim_2link_planar_B.beta1[3]
            .re;
        }
      } else if (dynsim_2link_planar_B.beta1[3].re == 0.0) {
        if (D[3].re == 0.0) {
          dynsim_2link_planar_B.scale = D[3].im / dynsim_2link_planar_B.beta1[3]
            .im;
          dynsim_2link_planar_B.absxk = 0.0;
        } else if (D[3].im == 0.0) {
          dynsim_2link_planar_B.scale = 0.0;
          dynsim_2link_planar_B.absxk = -(D[3].re / dynsim_2link_planar_B.beta1
            [3].im);
        } else {
          dynsim_2link_planar_B.scale = D[3].im / dynsim_2link_planar_B.beta1[3]
            .im;
          dynsim_2link_planar_B.absxk = -(D[3].re / dynsim_2link_planar_B.beta1
            [3].im);
        }
      } else {
        dynsim_2link_planar_B.colnorm = fabs(dynsim_2link_planar_B.beta1[3].re);
        dynsim_2link_planar_B.scale = fabs(dynsim_2link_planar_B.beta1[3].im);
        if (dynsim_2link_planar_B.colnorm > dynsim_2link_planar_B.scale) {
          dynsim_2link_planar_B.colnorm = dynsim_2link_planar_B.beta1[3].im /
            dynsim_2link_planar_B.beta1[3].re;
          dynsim_2link_planar_B.absxk = dynsim_2link_planar_B.colnorm *
            dynsim_2link_planar_B.beta1[3].im + dynsim_2link_planar_B.beta1[3].
            re;
          dynsim_2link_planar_B.scale = (dynsim_2link_planar_B.colnorm * D[3].im
            + D[3].re) / dynsim_2link_planar_B.absxk;
          dynsim_2link_planar_B.absxk = (D[3].im - dynsim_2link_planar_B.colnorm
            * D[3].re) / dynsim_2link_planar_B.absxk;
        } else if (dynsim_2link_planar_B.scale == dynsim_2link_planar_B.colnorm)
        {
          dynsim_2link_planar_B.absxk = dynsim_2link_planar_B.beta1[3].re > 0.0 ?
            0.5 : -0.5;
          dynsim_2link_planar_B.t = dynsim_2link_planar_B.beta1[3].im > 0.0 ?
            0.5 : -0.5;
          dynsim_2link_planar_B.scale = (D[3].re * dynsim_2link_planar_B.absxk +
            D[3].im * dynsim_2link_planar_B.t) / dynsim_2link_planar_B.colnorm;
          dynsim_2link_planar_B.absxk = (D[3].im * dynsim_2link_planar_B.absxk -
            D[3].re * dynsim_2link_planar_B.t) / dynsim_2link_planar_B.colnorm;
        } else {
          dynsim_2link_planar_B.colnorm = dynsim_2link_planar_B.beta1[3].re /
            dynsim_2link_planar_B.beta1[3].im;
          dynsim_2link_planar_B.absxk = dynsim_2link_planar_B.colnorm *
            dynsim_2link_planar_B.beta1[3].re + dynsim_2link_planar_B.beta1[3].
            im;
          dynsim_2link_planar_B.scale = (dynsim_2link_planar_B.colnorm * D[3].re
            + D[3].im) / dynsim_2link_planar_B.absxk;
          dynsim_2link_planar_B.absxk = (dynsim_2link_planar_B.colnorm * D[3].im
            - D[3].re) / dynsim_2link_planar_B.absxk;
        }
      }

      D[3].re = dynsim_2link_planar_B.scale;
      D[3].im = dynsim_2link_planar_B.absxk;
    }
  }
}

static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_GetP_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_dynsim_2li_a_T
  *pStruct)
{
  dynsim_2link_pla_emxFree_char_T(&pStruct->Type);
  dynsim_2link_pla_emxFree_real_T(&pStruct->MotionSubspace);
}

static void emxFreeStruct_o_robotics_manip_(o_robotics_manip_internal_R_a_T
  *pStruct)
{
  dynsim_2link_pla_emxFree_char_T(&pStruct->NameInternal);
  emxFreeStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static void emxFreeStruct_p_robotics_manip_(p_robotics_manip_internal_R_a_T
  *pStruct)
{
  emxFreeStruct_o_robotics_manip_(&pStruct->Base);
}

static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_b_a_T
  *pStruct)
{
  emxFreeStruct_p_robotics_manip_(&pStruct->TreeInternal);
}

static void emxFreeStruct_n_robotics_manip_(n_robotics_manip_internal_R_a_T
  *pStruct)
{
  dynsim_2link_pla_emxFree_char_T(&pStruct->NameInternal);
  emxFreeStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static void emxFreeStruct_c_rigidBodyJoint1(c_rigidBodyJoint_dynsim_2l_aw_T
  *pStruct)
{
  dynsim_2link_pla_emxFree_char_T(&pStruct->Type);
}

static void emxFreeStruct_o_robotics_mani_a(o_robotics_manip_internal__aw_T
  *pStruct)
{
  dynsim_2link_pla_emxFree_char_T(&pStruct->NameInternal);
  emxFreeStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static void emxFreeStruct_p_robotics_mani_a(p_robotics_manip_internal__aw_T
  *pStruct)
{
  emxFreeStruct_o_robotics_mani_a(&pStruct->Base);
}

static void emxFreeStruct_robotics_slmani_a(robotics_slmanip_internal__aw_T
  *pStruct)
{
  emxFreeStruct_p_robotics_mani_a(&pStruct->TreeInternal);
}

static void emxFreeStruct_n_robotics_mani_a(n_robotics_manip_internal__aw_T
  *pStruct)
{
  dynsim_2link_pla_emxFree_char_T(&pStruct->NameInternal);
  emxFreeStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static void dynsim_2link__matlabCodegenHa_p(ros_slros_internal_block_Subs_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void emxFreeStruct_c_rigidBodyJoint2(c_rigidBodyJoint_dynsim_2link_T
  *pStruct)
{
  dynsim_2link_pla_emxFree_char_T(&pStruct->Type);
  dynsim_2link_pla_emxFree_real_T(&pStruct->MotionSubspace);
}

static void emxFreeStruct_o_robotics_man_aw(o_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_c_rigidBodyJoint2(&pStruct->JointInternal);
}

static void emxFreeStruct_p_robotics_man_aw(p_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_o_robotics_man_aw(&pStruct->Base);
}

static void emxFreeStruct_robotics_slman_aw(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxFreeStruct_p_robotics_man_aw(&pStruct->TreeInternal);
}

static void emxFreeStruct_n_robotics_man_aw(n_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_c_rigidBodyJoint2(&pStruct->JointInternal);
}

static void matlabCodegenHandl_awtnueqcp3y5(ros_slros_internal_block_Publ_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_dynsim_2li_a_T
  *pStruct)
{
  dynsim_2link_pla_emxInit_char_T(&pStruct->Type, 2);
  dynsim_2link_pla_emxInit_real_T(&pStruct->MotionSubspace, 2);
}

static void emxInitStruct_o_robotics_manip_(o_robotics_manip_internal_R_a_T
  *pStruct)
{
  dynsim_2link_pla_emxInit_char_T(&pStruct->NameInternal, 2);
  emxInitStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static void emxInitStruct_p_robotics_manip_(p_robotics_manip_internal_R_a_T
  *pStruct)
{
  emxInitStruct_o_robotics_manip_(&pStruct->Base);
}

static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_b_a_T
  *pStruct)
{
  emxInitStruct_p_robotics_manip_(&pStruct->TreeInternal);
}

static void emxInitStruct_n_robotics_manip_(n_robotics_manip_internal_R_a_T
  *pStruct)
{
  dynsim_2link_pla_emxInit_char_T(&pStruct->NameInternal, 2);
  emxInitStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static n_robotics_manip_internal_R_a_T *d_RigidBody_RigidBody_awtnueqcp
  (n_robotics_manip_internal_R_a_T *obj)
{
  n_robotics_manip_internal_R_a_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_2link__T *switch_expression;
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
  dynsim_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 13; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 0.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static n_robotics_manip_internal_R_a_T *RigidBody_RigidBody_awtnueqcp3
  (n_robotics_manip_internal_R_a_T *obj)
{
  n_robotics_manip_internal_R_a_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_2link__T *switch_expression;
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
  dynsim_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 1.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static n_robotics_manip_internal_R_a_T *RigidBody_RigidBody_awtnueqcp3y
  (n_robotics_manip_internal_R_a_T *obj)
{
  n_robotics_manip_internal_R_a_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_2link__T *switch_expression;
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
  dynsim_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 2.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = -1.0;
  return b_obj;
}

static n_robotics_manip_internal_R_a_T *RigidBody_RigidBod_awtnueqcp3y5
  (n_robotics_manip_internal_R_a_T *obj)
{
  n_robotics_manip_internal_R_a_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_2link__T *switch_expression;
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
  dynsim_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 3.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static n_robotics_manip_internal_R_a_T *RigidBody_RigidBo_awtnueqcp3y53
  (n_robotics_manip_internal_R_a_T *obj)
{
  n_robotics_manip_internal_R_a_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_2link__T *switch_expression;
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
  dynsim_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 4.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static n_robotics_manip_internal_R_a_T *RigidBody_RigidB_awtnueqcp3y53m
  (n_robotics_manip_internal_R_a_T *obj)
{
  n_robotics_manip_internal_R_a_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_2link__T *switch_expression;
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
  dynsim_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 5.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 1.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static n_robotics_manip_internal_R_a_T *RigidBody_Rigid_awtnueqcp3y53m2
  (n_robotics_manip_internal_R_a_T *obj)
{
  n_robotics_manip_internal_R_a_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_2link__T *switch_expression;
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
  dynsim_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 6.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static n_robotics_manip_internal_R_a_T *d_RigidBody_Rigid_p
  (n_robotics_manip_internal_R_a_T *obj)
{
  n_robotics_manip_internal_R_a_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_2link__T *switch_expression;
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
  dynsim_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 11; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 7.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static o_robotics_manip_internal_R_a_T *d_RigidBody_Rigid_c
  (o_robotics_manip_internal_R_a_T *obj)
{
  o_robotics_manip_internal_R_a_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_2link__T *switch_expression;
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
  dynsim_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  return b_obj;
}

static p_robotics_manip_internal_R_a_T *d_RigidBodyTree_RigidBodyTree_a
  (p_robotics_manip_internal_R_a_T *obj, n_robotics_manip_internal_R_a_T *iobj_0,
   n_robotics_manip_internal_R_a_T *iobj_1, n_robotics_manip_internal_R_a_T
   *iobj_2, n_robotics_manip_internal_R_a_T *iobj_3,
   n_robotics_manip_internal_R_a_T *iobj_4, n_robotics_manip_internal_R_a_T
   *iobj_5, n_robotics_manip_internal_R_a_T *iobj_6,
   n_robotics_manip_internal_R_a_T *iobj_7)
{
  p_robotics_manip_internal_R_a_T *b_obj;
  int32_T i;
  static const int8_T tmp[16] = { 0, 1, 2, 3, 4, 5, 6, 0, -1, 1, 2, 3, 4, 5, 6,
    -1 };

  b_obj = obj;
  obj->Bodies[0] = d_RigidBody_RigidBody_awtnueqcp(iobj_0);
  obj->Bodies[0]->Index = 1.0;
  obj->Bodies[1] = RigidBody_RigidBody_awtnueqcp3(iobj_7);
  obj->Bodies[1]->Index = 2.0;
  obj->Bodies[2] = RigidBody_RigidBody_awtnueqcp3y(iobj_1);
  obj->Bodies[2]->Index = 3.0;
  obj->Bodies[3] = RigidBody_RigidBod_awtnueqcp3y5(iobj_2);
  obj->Bodies[3]->Index = 4.0;
  obj->Bodies[4] = RigidBody_RigidBo_awtnueqcp3y53(iobj_3);
  obj->Bodies[4]->Index = 5.0;
  obj->Bodies[5] = RigidBody_RigidB_awtnueqcp3y53m(iobj_4);
  obj->Bodies[5]->Index = 6.0;
  obj->Bodies[6] = RigidBody_Rigid_awtnueqcp3y53m2(iobj_5);
  obj->Bodies[6]->Index = 7.0;
  obj->Bodies[7] = d_RigidBody_Rigid_p(iobj_6);
  obj->Bodies[7]->Index = 8.0;
  obj->NumBodies = 8.0;
  obj->PositionNumber = 6.0;
  obj->VelocityNumber = 6.0;
  for (i = 0; i < 16; i++) {
    obj->PositionDoFMap[i] = tmp[i];
  }

  d_RigidBody_Rigid_c(&obj->Base);
  return b_obj;
}

static void emxInitStruct_c_rigidBodyJoint1(c_rigidBodyJoint_dynsim_2l_aw_T
  *pStruct)
{
  dynsim_2link_pla_emxInit_char_T(&pStruct->Type, 2);
}

static void emxInitStruct_o_robotics_mani_a(o_robotics_manip_internal__aw_T
  *pStruct)
{
  dynsim_2link_pla_emxInit_char_T(&pStruct->NameInternal, 2);
  emxInitStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static void emxInitStruct_p_robotics_mani_a(p_robotics_manip_internal__aw_T
  *pStruct)
{
  emxInitStruct_o_robotics_mani_a(&pStruct->Base);
}

static void emxInitStruct_robotics_slmani_a(robotics_slmanip_internal__aw_T
  *pStruct)
{
  emxInitStruct_p_robotics_mani_a(&pStruct->TreeInternal);
}

static void emxInitStruct_n_robotics_mani_a(n_robotics_manip_internal__aw_T
  *pStruct)
{
  dynsim_2link_pla_emxInit_char_T(&pStruct->NameInternal, 2);
  emxInitStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static n_robotics_manip_internal__aw_T *d_RigidBody_Rigid_n
  (n_robotics_manip_internal__aw_T *obj)
{
  n_robotics_manip_internal__aw_T *b_obj;
  emxArray_char_T_dynsim_2link__T *switch_expression;
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
  dynsim_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 13; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 0.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
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

static n_robotics_manip_internal__aw_T *d_RigidBody_Rigid_h
  (n_robotics_manip_internal__aw_T *obj)
{
  n_robotics_manip_internal__aw_T *b_obj;
  emxArray_char_T_dynsim_2link__T *switch_expression;
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
  dynsim_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 1.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
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

static n_robotics_manip_internal__aw_T *d_RigidBody_Rigid_j
  (n_robotics_manip_internal__aw_T *obj)
{
  n_robotics_manip_internal__aw_T *b_obj;
  emxArray_char_T_dynsim_2link__T *switch_expression;
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
  dynsim_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 2.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
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

static n_robotics_manip_internal__aw_T *d_RigidBody_Rigid_c3
  (n_robotics_manip_internal__aw_T *obj)
{
  n_robotics_manip_internal__aw_T *b_obj;
  emxArray_char_T_dynsim_2link__T *switch_expression;
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
  dynsim_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 3.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
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

static n_robotics_manip_internal__aw_T *d_RigidBody_Rigid_k
  (n_robotics_manip_internal__aw_T *obj)
{
  n_robotics_manip_internal__aw_T *b_obj;
  emxArray_char_T_dynsim_2link__T *switch_expression;
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
  dynsim_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 4.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
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

static p_robotics_manip_internal__aw_T *RigidBodyTree_RigidBodyTree_aw
  (p_robotics_manip_internal__aw_T *obj, n_robotics_manip_internal__aw_T *iobj_0,
   n_robotics_manip_internal__aw_T *iobj_1, n_robotics_manip_internal__aw_T
   *iobj_2, n_robotics_manip_internal__aw_T *iobj_3,
   n_robotics_manip_internal__aw_T *iobj_4, n_robotics_manip_internal__aw_T
   *iobj_5, n_robotics_manip_internal__aw_T *iobj_6,
   n_robotics_manip_internal__aw_T *iobj_7)
{
  p_robotics_manip_internal__aw_T *b_obj;
  o_robotics_manip_internal__aw_T *obj_0;
  emxArray_char_T_dynsim_2link__T *switch_expression;
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
  obj->Bodies[0] = d_RigidBody_Rigid_n(iobj_0);
  obj->Bodies[1] = d_RigidBody_Rigid_h(iobj_7);
  obj->Bodies[2] = d_RigidBody_Rigid_j(iobj_1);
  obj->Bodies[3] = d_RigidBody_Rigid_c3(iobj_2);
  obj->Bodies[4] = d_RigidBody_Rigid_k(iobj_3);
  b_kstr = iobj_4->NameInternal->size[0] * iobj_4->NameInternal->size[1];
  iobj_4->NameInternal->size[0] = 1;
  iobj_4->NameInternal->size[1] = 10;
  dynsim_emxEnsureCapacity_char_T(iobj_4->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    iobj_4->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  iobj_4->ParentIndex = 5.0;
  b_kstr = iobj_4->JointInternal.Type->size[0] * iobj_4->
    JointInternal.Type->size[1];
  iobj_4->JointInternal.Type->size[0] = 1;
  iobj_4->JointInternal.Type->size[1] = 8;
  dynsim_emxEnsureCapacity_char_T(iobj_4->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    iobj_4->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_4->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
  dynsim_emxEnsureCapacity_char_T(iobj_5->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    iobj_5->NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  iobj_5->ParentIndex = 6.0;
  b_kstr = iobj_5->JointInternal.Type->size[0] * iobj_5->
    JointInternal.Type->size[1];
  iobj_5->JointInternal.Type->size[0] = 1;
  iobj_5->JointInternal.Type->size[1] = 8;
  dynsim_emxEnsureCapacity_char_T(iobj_5->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    iobj_5->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_5->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
  dynsim_emxEnsureCapacity_char_T(iobj_6->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 11; b_kstr++) {
    iobj_6->NameInternal->data[b_kstr] = tmp_6[b_kstr];
  }

  iobj_6->ParentIndex = 7.0;
  b_kstr = iobj_6->JointInternal.Type->size[0] * iobj_6->
    JointInternal.Type->size[1];
  iobj_6->JointInternal.Type->size[0] = 1;
  iobj_6->JointInternal.Type->size[1] = 5;
  dynsim_emxEnsureCapacity_char_T(iobj_6->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_6->JointInternal.Type->data[b_kstr] = tmp_7[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_6->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
  dynsim_emxEnsureCapacity_char_T(obj->Base.NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->Base.NameInternal->data[b_kstr] = tmp_8[b_kstr];
  }

  b_kstr = obj->Base.JointInternal.Type->size[0] * obj->
    Base.JointInternal.Type->size[1];
  obj->Base.JointInternal.Type->size[0] = 1;
  obj->Base.JointInternal.Type->size[1] = 5;
  dynsim_emxEnsureCapacity_char_T(obj->Base.JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj_0->JointInternal.Type->data[b_kstr] = tmp_7[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->Base.JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
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

static void emxInitStruct_c_rigidBodyJoint2(c_rigidBodyJoint_dynsim_2link_T
  *pStruct)
{
  dynsim_2link_pla_emxInit_char_T(&pStruct->Type, 2);
  dynsim_2link_pla_emxInit_real_T(&pStruct->MotionSubspace, 2);
}

static void emxInitStruct_o_robotics_man_aw(o_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_c_rigidBodyJoint2(&pStruct->JointInternal);
}

static void emxInitStruct_p_robotics_man_aw(p_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_o_robotics_man_aw(&pStruct->Base);
}

static void emxInitStruct_robotics_slman_aw(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxInitStruct_p_robotics_man_aw(&pStruct->TreeInternal);
}

static void emxInitStruct_n_robotics_man_aw(n_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_c_rigidBodyJoint2(&pStruct->JointInternal);
}

static n_robotics_manip_internal_Rig_T *dynsim_2lin_RigidBody_RigidBody
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_2link__T *switch_expression;
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
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *dynsim_2l_RigidBody_RigidBody_a
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_2link__T *switch_expression;
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
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *dynsim_2_RigidBody_RigidBody_aw
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_2link__T *switch_expression;
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
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = -1.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *dynsim__RigidBody_RigidBody_awt
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_2link__T *switch_expression;
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
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *dynsim_RigidBody_RigidBody_awtn
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_2link__T *switch_expression;
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
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *dynsi_RigidBody_RigidBody_awtnu
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_2link__T *switch_expression;
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
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 1.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *dyns_RigidBody_RigidBody_awtnue
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_2link__T *switch_expression;
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
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *dyn_RigidBody_RigidBody_awtnueq
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_2link__T *switch_expression;
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
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static o_robotics_manip_internal_Rig_T *dy_RigidBody_RigidBody_awtnueqc
  (o_robotics_manip_internal_Rig_T *obj)
{
  o_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_dynsim_2link__T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
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
  dynsim_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  dynsim_2link_pla_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  dynsim_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  dynsim_2link_pla_emxFree_char_T(&switch_expression);
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
  dynsim_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  obj->Bodies[0] = dynsim_2lin_RigidBody_RigidBody(iobj_0);
  obj->Bodies[0]->Index = 1.0;
  obj->Bodies[1] = dynsim_2l_RigidBody_RigidBody_a(iobj_7);
  obj->Bodies[1]->Index = 2.0;
  obj->Bodies[2] = dynsim_2_RigidBody_RigidBody_aw(iobj_1);
  obj->Bodies[2]->Index = 3.0;
  obj->Bodies[3] = dynsim__RigidBody_RigidBody_awt(iobj_2);
  obj->Bodies[3]->Index = 4.0;
  obj->Bodies[4] = dynsim_RigidBody_RigidBody_awtn(iobj_3);
  obj->Bodies[4]->Index = 5.0;
  obj->Bodies[5] = dynsi_RigidBody_RigidBody_awtnu(iobj_4);
  obj->Bodies[5]->Index = 6.0;
  obj->Bodies[6] = dyns_RigidBody_RigidBody_awtnue(iobj_5);
  obj->Bodies[6]->Index = 7.0;
  obj->Bodies[7] = dyn_RigidBody_RigidBody_awtnueq(iobj_6);
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

  dy_RigidBody_RigidBody_awtnueqc(&obj->Base);
  return b_obj;
}

// Model step function
void dynsim_2link_planar_step(void)
{
  emxArray_real_T_dynsim_2link__T *b;
  robotics_slmanip_internal__aw_T *obj;
  p_robotics_manip_internal__aw_T *obj_0;
  emxArray_f_cell_wrap_dynsim_2_T *Ttree;
  emxArray_char_T_dynsim_2link__T *bname;
  n_robotics_manip_internal__aw_T *obj_1;
  robotics_slmanip_internal_blo_T *obj_2;
  emxArray_real_T_dynsim_2link__T *L;
  emxArray_real_T_dynsim_2link__T *lambda;
  emxArray_real_T_dynsim_2link__T *H;
  emxArray_real_T_dynsim_2link__T *tmp;
  static const char_T tmp_0[5] = { 'w', 'o', 'r', 'l', 'd' };

  static const char_T tmp_1[11] = { 'e', 'd', 'o', '_', 'l', 'i', 'n', 'k', '_',
    'e', 'e' };

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
  if (rtmIsMajorTimeStep(dynsim_2link_planar_M)) {
    // set solver stop time
    rtsiSetSolverStopTime(&dynsim_2link_planar_M->solverInfo,
                          ((dynsim_2link_planar_M->Timing.clockTick0+1)*
      dynsim_2link_planar_M->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if (rtmIsMinorTimeStep(dynsim_2link_planar_M)) {
    dynsim_2link_planar_M->Timing.t[0] = rtsiGetT
      (&dynsim_2link_planar_M->solverInfo);
  }

  // MATLABSystem: '<S14>/Get Parameter'
  ParamGet_dynsim_2link_planar_131.get_parameter(&dynsim_2link_planar_B.vNum);

  // MATLABSystem: '<S14>/Get Parameter1'
  ParamGet_dynsim_2link_planar_132.get_parameter(&dynsim_2link_planar_B.bid1);

  // MATLABSystem: '<S14>/Get Parameter4'
  ParamGet_dynsim_2link_planar_137.get_parameter(&dynsim_2link_planar_B.j);

  // MATLABSystem: '<S14>/Get Parameter5'
  ParamGet_dynsim_2link_planar_138.get_parameter(&dynsim_2link_planar_B.K23);

  // MATLABSystem: '<S14>/Get Parameter6'
  ParamGet_dynsim_2link_planar_139.get_parameter(&dynsim_2link_planar_B.K24);

  // MATLABSystem: '<S14>/Get Parameter7'
  ParamGet_dynsim_2link_planar_140.get_parameter(&dynsim_2link_planar_B.K34);

  // Integrator: '<S1>/Position' incorporates:
  //   MATLABSystem: '<S14>/Get Parameter'
  //   MATLABSystem: '<S14>/Get Parameter1'
  //   MATLABSystem: '<S14>/Get Parameter4'
  //   MATLABSystem: '<S14>/Get Parameter5'
  //   MATLABSystem: '<S14>/Get Parameter6'
  //   MATLABSystem: '<S14>/Get Parameter7'

  if (dynsim_2link_planar_DW.Position_IWORK != 0) {
    dynsim_2link_planar_X.Position_CSTATE[0] = dynsim_2link_planar_B.vNum;
    dynsim_2link_planar_X.Position_CSTATE[1] = dynsim_2link_planar_B.bid1;
    dynsim_2link_planar_X.Position_CSTATE[2] = dynsim_2link_planar_B.j;
    dynsim_2link_planar_X.Position_CSTATE[3] = dynsim_2link_planar_B.K23;
    dynsim_2link_planar_X.Position_CSTATE[4] = dynsim_2link_planar_B.K24;
    dynsim_2link_planar_X.Position_CSTATE[5] = dynsim_2link_planar_B.K34;
  }

  dynsim_2link_pla_emxInit_real_T(&b, 2);
  dynsim_2lin_emxInit_f_cell_wrap(&Ttree, 2);
  dynsim_2link_pla_emxInit_char_T(&bname, 2);

  // MATLABSystem: '<S15>/MATLAB System' incorporates:
  //   Integrator: '<S1>/Position'

  RigidBodyTree_geometricJacobian(&dynsim_2link_planar_DW.obj_f.TreeInternal,
    dynsim_2link_planar_X.Position_CSTATE, b);

  // MATLABSystem: '<S16>/MATLAB System' incorporates:
  //   Integrator: '<S1>/Position'

  obj = &dynsim_2link_planar_DW.obj_i;
  obj_0 = &dynsim_2link_planar_DW.obj_i.TreeInternal;
  RigidBodyTree_forwardKinemati_a(&obj->TreeInternal,
    dynsim_2link_planar_X.Position_CSTATE, Ttree);
  dynsim_2link_planar_B.bid1 = -1.0;
  dynsim_2link_planar_B.b_kstr = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj_0->Base.NameInternal->size[1];
  dynsim_emxEnsureCapacity_char_T(bname, dynsim_2link_planar_B.b_kstr);
  dynsim_2link_planar_B.n_c = obj_0->Base.NameInternal->size[0] *
    obj_0->Base.NameInternal->size[1] - 1;
  for (dynsim_2link_planar_B.b_kstr = 0; dynsim_2link_planar_B.b_kstr <=
       dynsim_2link_planar_B.n_c; dynsim_2link_planar_B.b_kstr++) {
    bname->data[dynsim_2link_planar_B.b_kstr] = obj_0->Base.NameInternal->
      data[dynsim_2link_planar_B.b_kstr];
  }

  for (dynsim_2link_planar_B.b_kstr = 0; dynsim_2link_planar_B.b_kstr < 5;
       dynsim_2link_planar_B.b_kstr++) {
    dynsim_2link_planar_B.b_o[dynsim_2link_planar_B.b_kstr] =
      tmp_0[dynsim_2link_planar_B.b_kstr];
  }

  dynsim_2link_planar_B.b_bool = false;
  if (bname->size[1] == 5) {
    dynsim_2link_planar_B.b_kstr = 1;
    do {
      exitg1 = 0;
      if (dynsim_2link_planar_B.b_kstr - 1 < 5) {
        dynsim_2link_planar_B.n_c = dynsim_2link_planar_B.b_kstr - 1;
        if (bname->data[dynsim_2link_planar_B.n_c] !=
            dynsim_2link_planar_B.b_o[dynsim_2link_planar_B.n_c]) {
          exitg1 = 1;
        } else {
          dynsim_2link_planar_B.b_kstr++;
        }
      } else {
        dynsim_2link_planar_B.b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (dynsim_2link_planar_B.b_bool) {
    dynsim_2link_planar_B.bid1 = 0.0;
  } else {
    dynsim_2link_planar_B.vNum = obj->TreeInternal.NumBodies;
    dynsim_2link_planar_B.i = 0;
    exitg2 = false;
    while ((!exitg2) && (dynsim_2link_planar_B.i <= static_cast<int32_T>
                         (dynsim_2link_planar_B.vNum) - 1)) {
      obj_1 = obj_0->Bodies[dynsim_2link_planar_B.i];
      dynsim_2link_planar_B.b_kstr = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = obj_1->NameInternal->size[1];
      dynsim_emxEnsureCapacity_char_T(bname, dynsim_2link_planar_B.b_kstr);
      dynsim_2link_planar_B.n_c = obj_1->NameInternal->size[0] *
        obj_1->NameInternal->size[1] - 1;
      for (dynsim_2link_planar_B.b_kstr = 0; dynsim_2link_planar_B.b_kstr <=
           dynsim_2link_planar_B.n_c; dynsim_2link_planar_B.b_kstr++) {
        bname->data[dynsim_2link_planar_B.b_kstr] = obj_1->NameInternal->
          data[dynsim_2link_planar_B.b_kstr];
      }

      for (dynsim_2link_planar_B.b_kstr = 0; dynsim_2link_planar_B.b_kstr < 5;
           dynsim_2link_planar_B.b_kstr++) {
        dynsim_2link_planar_B.b_o[dynsim_2link_planar_B.b_kstr] =
          tmp_0[dynsim_2link_planar_B.b_kstr];
      }

      dynsim_2link_planar_B.b_bool = false;
      if (bname->size[1] == 5) {
        dynsim_2link_planar_B.b_kstr = 1;
        do {
          exitg1 = 0;
          if (dynsim_2link_planar_B.b_kstr - 1 < 5) {
            dynsim_2link_planar_B.n_c = dynsim_2link_planar_B.b_kstr - 1;
            if (bname->data[dynsim_2link_planar_B.n_c] !=
                dynsim_2link_planar_B.b_o[dynsim_2link_planar_B.n_c]) {
              exitg1 = 1;
            } else {
              dynsim_2link_planar_B.b_kstr++;
            }
          } else {
            dynsim_2link_planar_B.b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (dynsim_2link_planar_B.b_bool) {
        dynsim_2link_planar_B.bid1 = static_cast<real_T>(dynsim_2link_planar_B.i)
          + 1.0;
        exitg2 = true;
      } else {
        dynsim_2link_planar_B.i++;
      }
    }
  }

  if (dynsim_2link_planar_B.bid1 == 0.0) {
    memset(&dynsim_2link_planar_B.T1[0], 0, sizeof(real_T) << 4U);
    dynsim_2link_planar_B.T1[0] = 1.0;
    dynsim_2link_planar_B.T1[5] = 1.0;
    dynsim_2link_planar_B.T1[10] = 1.0;
    dynsim_2link_planar_B.T1[15] = 1.0;
  } else {
    for (dynsim_2link_planar_B.b_kstr = 0; dynsim_2link_planar_B.b_kstr < 16;
         dynsim_2link_planar_B.b_kstr++) {
      dynsim_2link_planar_B.T1[dynsim_2link_planar_B.b_kstr] = Ttree->data[
        static_cast<int32_T>(dynsim_2link_planar_B.bid1) - 1]
        .f1[dynsim_2link_planar_B.b_kstr];
    }
  }

  dynsim_2link_planar_B.bid1 = -1.0;
  dynsim_2link_planar_B.b_kstr = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj_0->Base.NameInternal->size[1];
  dynsim_emxEnsureCapacity_char_T(bname, dynsim_2link_planar_B.b_kstr);
  dynsim_2link_planar_B.n_c = obj_0->Base.NameInternal->size[0] *
    obj_0->Base.NameInternal->size[1] - 1;
  for (dynsim_2link_planar_B.b_kstr = 0; dynsim_2link_planar_B.b_kstr <=
       dynsim_2link_planar_B.n_c; dynsim_2link_planar_B.b_kstr++) {
    bname->data[dynsim_2link_planar_B.b_kstr] = obj_0->Base.NameInternal->
      data[dynsim_2link_planar_B.b_kstr];
  }

  for (dynsim_2link_planar_B.b_kstr = 0; dynsim_2link_planar_B.b_kstr < 11;
       dynsim_2link_planar_B.b_kstr++) {
    dynsim_2link_planar_B.b_m[dynsim_2link_planar_B.b_kstr] =
      tmp_1[dynsim_2link_planar_B.b_kstr];
  }

  dynsim_2link_planar_B.b_bool = false;
  if (bname->size[1] == 11) {
    dynsim_2link_planar_B.b_kstr = 1;
    do {
      exitg1 = 0;
      if (dynsim_2link_planar_B.b_kstr - 1 < 11) {
        dynsim_2link_planar_B.n_c = dynsim_2link_planar_B.b_kstr - 1;
        if (bname->data[dynsim_2link_planar_B.n_c] !=
            dynsim_2link_planar_B.b_m[dynsim_2link_planar_B.n_c]) {
          exitg1 = 1;
        } else {
          dynsim_2link_planar_B.b_kstr++;
        }
      } else {
        dynsim_2link_planar_B.b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (dynsim_2link_planar_B.b_bool) {
    dynsim_2link_planar_B.bid1 = 0.0;
  } else {
    dynsim_2link_planar_B.vNum = obj->TreeInternal.NumBodies;
    dynsim_2link_planar_B.i = 0;
    exitg2 = false;
    while ((!exitg2) && (dynsim_2link_planar_B.i <= static_cast<int32_T>
                         (dynsim_2link_planar_B.vNum) - 1)) {
      obj_1 = obj_0->Bodies[dynsim_2link_planar_B.i];
      dynsim_2link_planar_B.b_kstr = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = obj_1->NameInternal->size[1];
      dynsim_emxEnsureCapacity_char_T(bname, dynsim_2link_planar_B.b_kstr);
      dynsim_2link_planar_B.n_c = obj_1->NameInternal->size[0] *
        obj_1->NameInternal->size[1] - 1;
      for (dynsim_2link_planar_B.b_kstr = 0; dynsim_2link_planar_B.b_kstr <=
           dynsim_2link_planar_B.n_c; dynsim_2link_planar_B.b_kstr++) {
        bname->data[dynsim_2link_planar_B.b_kstr] = obj_1->NameInternal->
          data[dynsim_2link_planar_B.b_kstr];
      }

      for (dynsim_2link_planar_B.b_kstr = 0; dynsim_2link_planar_B.b_kstr < 11;
           dynsim_2link_planar_B.b_kstr++) {
        dynsim_2link_planar_B.b_m[dynsim_2link_planar_B.b_kstr] =
          tmp_1[dynsim_2link_planar_B.b_kstr];
      }

      dynsim_2link_planar_B.b_bool = false;
      if (bname->size[1] == 11) {
        dynsim_2link_planar_B.b_kstr = 1;
        do {
          exitg1 = 0;
          if (dynsim_2link_planar_B.b_kstr - 1 < 11) {
            dynsim_2link_planar_B.n_c = dynsim_2link_planar_B.b_kstr - 1;
            if (bname->data[dynsim_2link_planar_B.n_c] !=
                dynsim_2link_planar_B.b_m[dynsim_2link_planar_B.n_c]) {
              exitg1 = 1;
            } else {
              dynsim_2link_planar_B.b_kstr++;
            }
          } else {
            dynsim_2link_planar_B.b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (dynsim_2link_planar_B.b_bool) {
        dynsim_2link_planar_B.bid1 = static_cast<real_T>(dynsim_2link_planar_B.i)
          + 1.0;
        exitg2 = true;
      } else {
        dynsim_2link_planar_B.i++;
      }
    }
  }

  dynsim_2link_pla_emxFree_char_T(&bname);

  // MATLABSystem: '<S16>/MATLAB System'
  if (dynsim_2link_planar_B.bid1 == 0.0) {
    memset(&dynsim_2link_planar_B.T2[0], 0, sizeof(real_T) << 4U);
    dynsim_2link_planar_B.T2[0] = 1.0;
    dynsim_2link_planar_B.T2[5] = 1.0;
    dynsim_2link_planar_B.T2[10] = 1.0;
    dynsim_2link_planar_B.T2[15] = 1.0;
  } else {
    for (dynsim_2link_planar_B.b_kstr = 0; dynsim_2link_planar_B.b_kstr < 16;
         dynsim_2link_planar_B.b_kstr++) {
      dynsim_2link_planar_B.T2[dynsim_2link_planar_B.b_kstr] = Ttree->data[
        static_cast<int32_T>(dynsim_2link_planar_B.bid1) - 1]
        .f1[dynsim_2link_planar_B.b_kstr];
    }
  }

  dynsim_2lin_emxFree_f_cell_wrap(&Ttree);

  // MATLABSystem: '<S16>/MATLAB System'
  for (dynsim_2link_planar_B.b_kstr = 0; dynsim_2link_planar_B.b_kstr < 3;
       dynsim_2link_planar_B.b_kstr++) {
    dynsim_2link_planar_B.R_j[3 * dynsim_2link_planar_B.b_kstr] =
      dynsim_2link_planar_B.T2[dynsim_2link_planar_B.b_kstr];
    dynsim_2link_planar_B.R_j[3 * dynsim_2link_planar_B.b_kstr + 1] =
      dynsim_2link_planar_B.T2[dynsim_2link_planar_B.b_kstr + 4];
    dynsim_2link_planar_B.R_j[3 * dynsim_2link_planar_B.b_kstr + 2] =
      dynsim_2link_planar_B.T2[dynsim_2link_planar_B.b_kstr + 8];
  }

  for (dynsim_2link_planar_B.b_kstr = 0; dynsim_2link_planar_B.b_kstr < 9;
       dynsim_2link_planar_B.b_kstr++) {
    dynsim_2link_planar_B.R_d[dynsim_2link_planar_B.b_kstr] =
      -dynsim_2link_planar_B.R_j[dynsim_2link_planar_B.b_kstr];
  }

  for (dynsim_2link_planar_B.b_kstr = 0; dynsim_2link_planar_B.b_kstr < 3;
       dynsim_2link_planar_B.b_kstr++) {
    dynsim_2link_planar_B.i = dynsim_2link_planar_B.b_kstr << 2;
    dynsim_2link_planar_B.R_k[dynsim_2link_planar_B.i] =
      dynsim_2link_planar_B.R_j[3 * dynsim_2link_planar_B.b_kstr];
    dynsim_2link_planar_B.R_k[dynsim_2link_planar_B.i + 1] =
      dynsim_2link_planar_B.R_j[3 * dynsim_2link_planar_B.b_kstr + 1];
    dynsim_2link_planar_B.R_k[dynsim_2link_planar_B.i + 2] =
      dynsim_2link_planar_B.R_j[3 * dynsim_2link_planar_B.b_kstr + 2];
    dynsim_2link_planar_B.R_k[dynsim_2link_planar_B.b_kstr + 12] =
      dynsim_2link_planar_B.R_d[dynsim_2link_planar_B.b_kstr + 6] *
      dynsim_2link_planar_B.T2[14] +
      (dynsim_2link_planar_B.R_d[dynsim_2link_planar_B.b_kstr + 3] *
       dynsim_2link_planar_B.T2[13] +
       dynsim_2link_planar_B.R_d[dynsim_2link_planar_B.b_kstr] *
       dynsim_2link_planar_B.T2[12]);
  }

  dynsim_2link_planar_B.R_k[3] = 0.0;
  dynsim_2link_planar_B.R_k[7] = 0.0;
  dynsim_2link_planar_B.R_k[11] = 0.0;
  dynsim_2link_planar_B.R_k[15] = 1.0;
  for (dynsim_2link_planar_B.b_kstr = 0; dynsim_2link_planar_B.b_kstr < 4;
       dynsim_2link_planar_B.b_kstr++) {
    for (dynsim_2link_planar_B.n_c = 0; dynsim_2link_planar_B.n_c < 4;
         dynsim_2link_planar_B.n_c++) {
      dynsim_2link_planar_B.i = dynsim_2link_planar_B.b_kstr << 2;
      dynsim_2link_planar_B.rtb_MATLABSystem_tmp = dynsim_2link_planar_B.n_c +
        dynsim_2link_planar_B.i;
      dynsim_2link_planar_B.T2[dynsim_2link_planar_B.rtb_MATLABSystem_tmp] = 0.0;
      dynsim_2link_planar_B.T2[dynsim_2link_planar_B.rtb_MATLABSystem_tmp] +=
        dynsim_2link_planar_B.T1[dynsim_2link_planar_B.i] *
        dynsim_2link_planar_B.R_k[dynsim_2link_planar_B.n_c];
      dynsim_2link_planar_B.T2[dynsim_2link_planar_B.rtb_MATLABSystem_tmp] +=
        dynsim_2link_planar_B.T1[dynsim_2link_planar_B.i + 1] *
        dynsim_2link_planar_B.R_k[dynsim_2link_planar_B.n_c + 4];
      dynsim_2link_planar_B.T2[dynsim_2link_planar_B.rtb_MATLABSystem_tmp] +=
        dynsim_2link_planar_B.T1[dynsim_2link_planar_B.i + 2] *
        dynsim_2link_planar_B.R_k[dynsim_2link_planar_B.n_c + 8];
      dynsim_2link_planar_B.T2[dynsim_2link_planar_B.rtb_MATLABSystem_tmp] +=
        dynsim_2link_planar_B.T1[dynsim_2link_planar_B.i + 3] *
        dynsim_2link_planar_B.R_k[dynsim_2link_planar_B.n_c + 12];
    }
  }

  // MATLABSystem: '<S14>/Get Parameter2'
  ParamGet_dynsim_2link_planar_135.get_parameter(&dynsim_2link_planar_B.vNum);

  // MATLABSystem: '<S14>/Get Parameter3'
  ParamGet_dynsim_2link_planar_136.get_parameter(&dynsim_2link_planar_B.bid1);

  // MATLABSystem: '<S14>/Get Parameter8'
  ParamGet_dynsim_2link_planar_141.get_parameter(&dynsim_2link_planar_B.j);

  // MATLABSystem: '<S14>/Get Parameter9'
  ParamGet_dynsim_2link_planar_142.get_parameter(&dynsim_2link_planar_B.K23);

  // MATLABSystem: '<S14>/Get Parameter10'
  ParamGet_dynsim_2link_planar_133.get_parameter(&dynsim_2link_planar_B.K24);

  // MATLABSystem: '<S14>/Get Parameter11'
  ParamGet_dynsim_2link_planar_134.get_parameter(&dynsim_2link_planar_B.K34);

  // Integrator: '<S1>/Velocity' incorporates:
  //   MATLABSystem: '<S14>/Get Parameter10'
  //   MATLABSystem: '<S14>/Get Parameter11'
  //   MATLABSystem: '<S14>/Get Parameter2'
  //   MATLABSystem: '<S14>/Get Parameter3'
  //   MATLABSystem: '<S14>/Get Parameter8'
  //   MATLABSystem: '<S14>/Get Parameter9'

  if (dynsim_2link_planar_DW.Velocity_IWORK != 0) {
    dynsim_2link_planar_X.Velocity_CSTATE[0] = dynsim_2link_planar_B.vNum;
    dynsim_2link_planar_X.Velocity_CSTATE[1] = dynsim_2link_planar_B.bid1;
    dynsim_2link_planar_X.Velocity_CSTATE[2] = dynsim_2link_planar_B.j;
    dynsim_2link_planar_X.Velocity_CSTATE[3] = dynsim_2link_planar_B.K23;
    dynsim_2link_planar_X.Velocity_CSTATE[4] = dynsim_2link_planar_B.K24;
    dynsim_2link_planar_X.Velocity_CSTATE[5] = dynsim_2link_planar_B.K34;
  }

  for (dynsim_2link_planar_B.i = 0; dynsim_2link_planar_B.i < 6;
       dynsim_2link_planar_B.i++) {
    dynsim_2link_planar_B.Velocity[dynsim_2link_planar_B.i] =
      dynsim_2link_planar_X.Velocity_CSTATE[dynsim_2link_planar_B.i];
  }

  // End of Integrator: '<S1>/Velocity'

  // Product: '<S4>/MatrixMultiply' incorporates:
  //   MATLABSystem: '<S15>/MATLAB System'

  for (dynsim_2link_planar_B.b_kstr = 0; dynsim_2link_planar_B.b_kstr < 6;
       dynsim_2link_planar_B.b_kstr++) {
    dynsim_2link_planar_B.MatrixMultiply[dynsim_2link_planar_B.b_kstr] = 0.0;
    for (dynsim_2link_planar_B.n_c = 0; dynsim_2link_planar_B.n_c < 6;
         dynsim_2link_planar_B.n_c++) {
      dynsim_2link_planar_B.vNum = b->data[6 * dynsim_2link_planar_B.n_c +
        dynsim_2link_planar_B.b_kstr] *
        dynsim_2link_planar_B.Velocity[dynsim_2link_planar_B.n_c] +
        dynsim_2link_planar_B.MatrixMultiply[dynsim_2link_planar_B.b_kstr];
      dynsim_2link_planar_B.MatrixMultiply[dynsim_2link_planar_B.b_kstr] =
        dynsim_2link_planar_B.vNum;
    }
  }

  // End of Product: '<S4>/MatrixMultiply'
  dynsim_2link_pla_emxFree_real_T(&b);
  if (rtmIsMajorTimeStep(dynsim_2link_planar_M)) {
    // Outputs for Atomic SubSystem: '<Root>/Subscribe'
    // MATLABSystem: '<S11>/SourceBlock' incorporates:
    //   Inport: '<S18>/In1'

    dynsim_2link_pl_SystemCore_step(&dynsim_2link_planar_B.b_bool,
      dynsim_2link_planar_B.b_varargout_2_Data,
      &dynsim_2link_planar_B.b_varargout_2_Data_SL_Info_Curr,
      &dynsim_2link_planar_B.b_varargout_2_Data_SL_Info_Rece,
      &dynsim_2link_planar_B.b_varargout_2_Layout_DataOffset,
      dynsim_2link_planar_B.b_varargout_2_Layout_Dim,
      &dynsim_2link_planar_B.b_varargout_2_Layout_Dim_SL_Inf,
      &dynsim_2link_planar_B.b_varargout_2_Layout_Dim_SL_I_c);

    // Outputs for Enabled SubSystem: '<S11>/Enabled Subsystem' incorporates:
    //   EnablePort: '<S18>/Enable'

    if (dynsim_2link_planar_B.b_bool) {
      memcpy(&dynsim_2link_planar_B.In1.Data[0],
             &dynsim_2link_planar_B.b_varargout_2_Data[0], sizeof(real_T) << 7U);
      dynsim_2link_planar_B.In1.Data_SL_Info.CurrentLength =
        dynsim_2link_planar_B.b_varargout_2_Data_SL_Info_Curr;
      dynsim_2link_planar_B.In1.Data_SL_Info.ReceivedLength =
        dynsim_2link_planar_B.b_varargout_2_Data_SL_Info_Rece;
      dynsim_2link_planar_B.In1.Layout.DataOffset =
        dynsim_2link_planar_B.b_varargout_2_Layout_DataOffset;
      memcpy(&dynsim_2link_planar_B.In1.Layout.Dim[0],
             &dynsim_2link_planar_B.b_varargout_2_Layout_Dim[0], sizeof
             (SL_Bus_dynsim_2link_planar_std_msgs_MultiArrayDimension) << 4U);
      dynsim_2link_planar_B.In1.Layout.Dim_SL_Info.CurrentLength =
        dynsim_2link_planar_B.b_varargout_2_Layout_Dim_SL_Inf;
      dynsim_2link_planar_B.In1.Layout.Dim_SL_Info.ReceivedLength =
        dynsim_2link_planar_B.b_varargout_2_Layout_Dim_SL_I_c;
    }

    // End of MATLABSystem: '<S11>/SourceBlock'
    // End of Outputs for SubSystem: '<S11>/Enabled Subsystem'
    // End of Outputs for SubSystem: '<Root>/Subscribe'
  }

  dynsim_2link_pla_emxInit_real_T(&L, 2);
  dynsim_2link_pla_emxInit_real_T(&lambda, 2);
  dynsim_2link_pla_emxInit_real_T(&tmp, 1);

  // MATLABSystem: '<S13>/MATLAB System' incorporates:
  //   Constant: '<S1>/Constant'
  //   Integrator: '<S1>/Position'

  obj_2 = &dynsim_2link_planar_DW.obj;
  RigidBodyTreeDynamics_massMatri(&dynsim_2link_planar_DW.obj.TreeInternal,
    dynsim_2link_planar_X.Position_CSTATE, L, lambda);
  dynsim_2link_planar_B.vNum = obj_2->TreeInternal.VelocityNumber;
  dynsim_2link_planar_B.rtb_MATLABSystem_tmp = static_cast<int32_T>
    (dynsim_2link_planar_B.vNum);
  dynsim_2link_planar_B.b_kstr = tmp->size[0];
  tmp->size[0] = dynsim_2link_planar_B.rtb_MATLABSystem_tmp;
  dynsim_emxEnsureCapacity_real_T(tmp, dynsim_2link_planar_B.b_kstr);
  for (dynsim_2link_planar_B.b_kstr = 0; dynsim_2link_planar_B.b_kstr <
       dynsim_2link_planar_B.rtb_MATLABSystem_tmp; dynsim_2link_planar_B.b_kstr
       ++) {
    tmp->data[dynsim_2link_planar_B.b_kstr] = 0.0;
  }

  RigidBodyTreeDynamics_inverseDy(&obj_2->TreeInternal,
    dynsim_2link_planar_X.Position_CSTATE, dynsim_2link_planar_B.Velocity,
    dynsim_2link_planar_P.Constant_Value_ij, dynsim_2link_planar_B.MATLABSystem);
  dynsim_2link_pla_emxFree_real_T(&tmp);

  // MATLABSystem: '<S13>/MATLAB System'
  for (dynsim_2link_planar_B.b_kstr = 0; dynsim_2link_planar_B.b_kstr < 6;
       dynsim_2link_planar_B.b_kstr++) {
    dynsim_2link_planar_B.MATLABSystem[dynsim_2link_planar_B.b_kstr] =
      dynsim_2link_planar_B.In1.Data[dynsim_2link_planar_B.b_kstr + 1] -
      dynsim_2link_planar_B.MATLABSystem[dynsim_2link_planar_B.b_kstr];
  }

  if ((L->size[0] == 0) || (L->size[1] == 0)) {
    dynsim_2link_planar_B.u1 = 0;
  } else {
    dynsim_2link_planar_B.i = L->size[0];
    dynsim_2link_planar_B.u1 = L->size[1];
    if (dynsim_2link_planar_B.i > dynsim_2link_planar_B.u1) {
      dynsim_2link_planar_B.u1 = dynsim_2link_planar_B.i;
    }
  }

  dynsim_2link_pla_emxInit_real_T(&H, 2);

  // MATLABSystem: '<S13>/MATLAB System'
  dynsim_2link_planar_B.b_kstr = H->size[0] * H->size[1];
  H->size[0] = L->size[0];
  H->size[1] = L->size[1];
  dynsim_emxEnsureCapacity_real_T(H, dynsim_2link_planar_B.b_kstr);
  dynsim_2link_planar_B.n_c = L->size[0] * L->size[1] - 1;
  for (dynsim_2link_planar_B.b_kstr = 0; dynsim_2link_planar_B.b_kstr <=
       dynsim_2link_planar_B.n_c; dynsim_2link_planar_B.b_kstr++) {
    H->data[dynsim_2link_planar_B.b_kstr] = L->data[dynsim_2link_planar_B.b_kstr];
  }

  dynsim_2link_planar_B.iend = static_cast<int32_T>(((-1.0 - static_cast<real_T>
    (dynsim_2link_planar_B.u1)) + 1.0) / -1.0) - 1;
  for (dynsim_2link_planar_B.i = 0; dynsim_2link_planar_B.i <=
       dynsim_2link_planar_B.iend; dynsim_2link_planar_B.i++) {
    dynsim_2link_planar_B.j = static_cast<real_T>(dynsim_2link_planar_B.u1) + -
      static_cast<real_T>(dynsim_2link_planar_B.i);
    dynsim_2link_planar_B.b_kstr = static_cast<int32_T>(dynsim_2link_planar_B.j);
    dynsim_2link_planar_B.n_c = dynsim_2link_planar_B.b_kstr - 1;
    H->data[(static_cast<int32_T>(dynsim_2link_planar_B.j) + H->size[0] * (
              static_cast<int32_T>(dynsim_2link_planar_B.j) - 1)) - 1] = sqrt
      (H->data[(dynsim_2link_planar_B.n_c * H->size[0] +
                dynsim_2link_planar_B.b_kstr) - 1]);
    dynsim_2link_planar_B.bid1 = lambda->data[dynsim_2link_planar_B.n_c];
    while (dynsim_2link_planar_B.bid1 > 0.0) {
      dynsim_2link_planar_B.i_a = static_cast<int32_T>
        (dynsim_2link_planar_B.bid1) - 1;
      H->data[(static_cast<int32_T>(dynsim_2link_planar_B.j) + H->size[0] * (
                static_cast<int32_T>(dynsim_2link_planar_B.bid1) - 1)) - 1] =
        H->data[(dynsim_2link_planar_B.i_a * H->size[0] +
                 dynsim_2link_planar_B.b_kstr) - 1] / H->data
        [((static_cast<int32_T>(dynsim_2link_planar_B.j) - 1) * H->size[0] +
          static_cast<int32_T>(dynsim_2link_planar_B.j)) - 1];
      dynsim_2link_planar_B.bid1 = lambda->data[dynsim_2link_planar_B.i_a];
    }

    dynsim_2link_planar_B.bid1 = lambda->data[dynsim_2link_planar_B.n_c];
    while (dynsim_2link_planar_B.bid1 > 0.0) {
      dynsim_2link_planar_B.j = dynsim_2link_planar_B.bid1;
      while (dynsim_2link_planar_B.j > 0.0) {
        dynsim_2link_planar_B.n_c = static_cast<int32_T>(dynsim_2link_planar_B.j)
          - 1;
        H->data[(static_cast<int32_T>(dynsim_2link_planar_B.bid1) + H->size[0] *
                 (static_cast<int32_T>(dynsim_2link_planar_B.j) - 1)) - 1] =
          H->data[(dynsim_2link_planar_B.n_c * H->size[0] + static_cast<int32_T>
                   (dynsim_2link_planar_B.bid1)) - 1] - H->data
          [((static_cast<int32_T>(dynsim_2link_planar_B.bid1) - 1) * H->size[0]
            + dynsim_2link_planar_B.b_kstr) - 1] * H->data[((static_cast<int32_T>
          (dynsim_2link_planar_B.j) - 1) * H->size[0] +
          dynsim_2link_planar_B.b_kstr) - 1];
        dynsim_2link_planar_B.j = lambda->data[dynsim_2link_planar_B.n_c];
      }

      dynsim_2link_planar_B.bid1 = lambda->data[static_cast<int32_T>
        (dynsim_2link_planar_B.bid1) - 1];
    }
  }

  dynsim_2link_planar_B.b_kstr = L->size[0] * L->size[1];
  L->size[0] = H->size[0];
  L->size[1] = H->size[1];
  dynsim_emxEnsureCapacity_real_T(L, dynsim_2link_planar_B.b_kstr);
  dynsim_2link_planar_B.n_c = H->size[0] * H->size[1] - 1;
  for (dynsim_2link_planar_B.b_kstr = 0; dynsim_2link_planar_B.b_kstr <=
       dynsim_2link_planar_B.n_c; dynsim_2link_planar_B.b_kstr++) {
    L->data[dynsim_2link_planar_B.b_kstr] = H->data[dynsim_2link_planar_B.b_kstr];
  }

  dynsim_2link_planar_B.n_c = H->size[1];
  if ((H->size[0] == 0) || (H->size[1] == 0) || (1 >= H->size[1])) {
  } else {
    dynsim_2link_planar_B.iend = 0;
    for (dynsim_2link_planar_B.b_kstr = 2; dynsim_2link_planar_B.b_kstr <=
         dynsim_2link_planar_B.n_c; dynsim_2link_planar_B.b_kstr++) {
      for (dynsim_2link_planar_B.i = 0; dynsim_2link_planar_B.i <=
           dynsim_2link_planar_B.iend; dynsim_2link_planar_B.i++) {
        L->data[dynsim_2link_planar_B.i + L->size[0] *
          (dynsim_2link_planar_B.b_kstr - 1)] = 0.0;
      }

      if (dynsim_2link_planar_B.iend + 1 < H->size[0]) {
        dynsim_2link_planar_B.iend++;
      }
    }
  }

  dynsim_2link_pla_emxFree_real_T(&H);

  // MATLABSystem: '<S13>/MATLAB System'
  dynsim_2link_planar_B.iend = static_cast<int32_T>(((-1.0 -
    dynsim_2link_planar_B.vNum) + 1.0) / -1.0) - 1;
  for (dynsim_2link_planar_B.i = 0; dynsim_2link_planar_B.i <=
       dynsim_2link_planar_B.iend; dynsim_2link_planar_B.i++) {
    dynsim_2link_planar_B.n_c = static_cast<int32_T>(dynsim_2link_planar_B.vNum
      + -static_cast<real_T>(dynsim_2link_planar_B.i));
    dynsim_2link_planar_B.b_kstr = dynsim_2link_planar_B.n_c - 1;
    dynsim_2link_planar_B.MATLABSystem[dynsim_2link_planar_B.b_kstr] /= L->data
      [(dynsim_2link_planar_B.b_kstr * L->size[0] + dynsim_2link_planar_B.n_c) -
      1];
    dynsim_2link_planar_B.j = lambda->data[dynsim_2link_planar_B.b_kstr];
    while (dynsim_2link_planar_B.j > 0.0) {
      dynsim_2link_planar_B.u1 = static_cast<int32_T>(dynsim_2link_planar_B.j) -
        1;
      dynsim_2link_planar_B.MATLABSystem[dynsim_2link_planar_B.u1] -= L->data
        [(dynsim_2link_planar_B.u1 * L->size[0] + dynsim_2link_planar_B.n_c) - 1]
        * dynsim_2link_planar_B.MATLABSystem[dynsim_2link_planar_B.b_kstr];
      dynsim_2link_planar_B.j = lambda->data[dynsim_2link_planar_B.u1];
    }
  }

  dynsim_2link_planar_B.n_c = dynsim_2link_planar_B.rtb_MATLABSystem_tmp - 1;
  for (dynsim_2link_planar_B.i = 0; dynsim_2link_planar_B.i <=
       dynsim_2link_planar_B.n_c; dynsim_2link_planar_B.i++) {
    dynsim_2link_planar_B.j = lambda->data[dynsim_2link_planar_B.i];
    while (dynsim_2link_planar_B.j > 0.0) {
      dynsim_2link_planar_B.b_kstr = static_cast<int32_T>
        (dynsim_2link_planar_B.j) - 1;
      dynsim_2link_planar_B.MATLABSystem[dynsim_2link_planar_B.i] -= L->
        data[dynsim_2link_planar_B.b_kstr * L->size[0] + dynsim_2link_planar_B.i]
        * dynsim_2link_planar_B.MATLABSystem[dynsim_2link_planar_B.b_kstr];
      dynsim_2link_planar_B.j = lambda->data[dynsim_2link_planar_B.b_kstr];
    }

    dynsim_2link_planar_B.MATLABSystem[dynsim_2link_planar_B.i] /= L->data
      [L->size[0] * dynsim_2link_planar_B.i + dynsim_2link_planar_B.i];
  }

  dynsim_2link_pla_emxFree_real_T(&lambda);
  dynsim_2link_pla_emxFree_real_T(&L);

  // MATLAB Function: '<S4>/MATLAB Function'
  dynsim_2link_planar_B.vNum = dynsim_2link_planar_B.T2[4] +
    dynsim_2link_planar_B.T2[1];
  dynsim_2link_planar_B.bid1 = dynsim_2link_planar_B.T2[8] +
    dynsim_2link_planar_B.T2[2];
  dynsim_2link_planar_B.j = dynsim_2link_planar_B.T2[6] -
    dynsim_2link_planar_B.T2[9];
  dynsim_2link_planar_B.K23 = dynsim_2link_planar_B.T2[9] +
    dynsim_2link_planar_B.T2[6];
  dynsim_2link_planar_B.K24 = dynsim_2link_planar_B.T2[8] -
    dynsim_2link_planar_B.T2[2];
  dynsim_2link_planar_B.K34 = dynsim_2link_planar_B.T2[1] -
    dynsim_2link_planar_B.T2[4];
  dynsim_2link_planar_B.T1[0] = ((dynsim_2link_planar_B.T2[0] -
    dynsim_2link_planar_B.T2[5]) - dynsim_2link_planar_B.T2[10]) / 3.0;
  dynsim_2link_planar_B.T1[4] = dynsim_2link_planar_B.vNum / 3.0;
  dynsim_2link_planar_B.T1[8] = dynsim_2link_planar_B.bid1 / 3.0;
  dynsim_2link_planar_B.T1[12] = dynsim_2link_planar_B.j / 3.0;
  dynsim_2link_planar_B.T1[1] = dynsim_2link_planar_B.vNum / 3.0;
  dynsim_2link_planar_B.T1[5] = ((dynsim_2link_planar_B.T2[5] -
    dynsim_2link_planar_B.T2[0]) - dynsim_2link_planar_B.T2[10]) / 3.0;
  dynsim_2link_planar_B.T1[9] = dynsim_2link_planar_B.K23 / 3.0;
  dynsim_2link_planar_B.T1[13] = dynsim_2link_planar_B.K24 / 3.0;
  dynsim_2link_planar_B.T1[2] = dynsim_2link_planar_B.bid1 / 3.0;
  dynsim_2link_planar_B.T1[6] = dynsim_2link_planar_B.K23 / 3.0;
  dynsim_2link_planar_B.T1[10] = ((dynsim_2link_planar_B.T2[10] -
    dynsim_2link_planar_B.T2[0]) - dynsim_2link_planar_B.T2[5]) / 3.0;
  dynsim_2link_planar_B.T1[14] = dynsim_2link_planar_B.K34 / 3.0;
  dynsim_2link_planar_B.T1[3] = dynsim_2link_planar_B.j / 3.0;
  dynsim_2link_planar_B.T1[7] = dynsim_2link_planar_B.K24 / 3.0;
  dynsim_2link_planar_B.T1[11] = dynsim_2link_planar_B.K34 / 3.0;
  dynsim_2link_planar_B.T1[15] = ((dynsim_2link_planar_B.T2[0] +
    dynsim_2link_planar_B.T2[5]) + dynsim_2link_planar_B.T2[10]) / 3.0;
  dynsim_2link_planar_eig(dynsim_2link_planar_B.T1, dynsim_2link_planar_B.eigVec,
    dynsim_2link_planar_B.eigVal);
  dynsim_2link_planar_B.cartOrn[0] = dynsim_2link_planar_B.eigVal[0].re;
  dynsim_2link_planar_B.cartOrn[1] = dynsim_2link_planar_B.eigVal[1].re;
  dynsim_2link_planar_B.cartOrn[2] = dynsim_2link_planar_B.eigVal[2].re;
  dynsim_2link_planar_B.cartOrn[3] = dynsim_2link_planar_B.eigVal[3].re;
  if (!rtIsNaN(dynsim_2link_planar_B.eigVal[0].re)) {
    dynsim_2link_planar_B.i = 1;
  } else {
    dynsim_2link_planar_B.i = 0;
    dynsim_2link_planar_B.b_kstr = 2;
    exitg2 = false;
    while ((!exitg2) && (dynsim_2link_planar_B.b_kstr < 5)) {
      if (!rtIsNaN(dynsim_2link_planar_B.cartOrn[dynsim_2link_planar_B.b_kstr -
                   1])) {
        dynsim_2link_planar_B.i = dynsim_2link_planar_B.b_kstr;
        exitg2 = true;
      } else {
        dynsim_2link_planar_B.b_kstr++;
      }
    }
  }

  if (dynsim_2link_planar_B.i != 0) {
    dynsim_2link_planar_B.vNum =
      dynsim_2link_planar_B.cartOrn[dynsim_2link_planar_B.i - 1];
    dynsim_2link_planar_B.b_kstr = dynsim_2link_planar_B.i - 1;
    while (dynsim_2link_planar_B.i + 1 < 5) {
      if (dynsim_2link_planar_B.vNum <
          dynsim_2link_planar_B.cartOrn[dynsim_2link_planar_B.i]) {
        dynsim_2link_planar_B.vNum =
          dynsim_2link_planar_B.cartOrn[dynsim_2link_planar_B.i];
        dynsim_2link_planar_B.b_kstr = dynsim_2link_planar_B.i;
      }

      dynsim_2link_planar_B.i++;
    }

    dynsim_2link_planar_B.i = dynsim_2link_planar_B.b_kstr;
  }

  dynsim_2link_planar_B.i <<= 2;
  dynsim_2link_planar_B.b_kstr = dynsim_2link_planar_B.i + 3;
  dynsim_2link_planar_B.cartOrn[0] =
    dynsim_2link_planar_B.eigVec[dynsim_2link_planar_B.b_kstr].re;
  dynsim_2link_planar_B.cartOrn[1] =
    dynsim_2link_planar_B.eigVec[dynsim_2link_planar_B.i].re;
  dynsim_2link_planar_B.n_c = dynsim_2link_planar_B.i + 1;
  dynsim_2link_planar_B.cartOrn[2] =
    dynsim_2link_planar_B.eigVec[dynsim_2link_planar_B.n_c].re;
  dynsim_2link_planar_B.rtb_MATLABSystem_tmp = dynsim_2link_planar_B.i + 2;
  dynsim_2link_planar_B.cartOrn[3] =
    dynsim_2link_planar_B.eigVec[dynsim_2link_planar_B.rtb_MATLABSystem_tmp].re;
  if (dynsim_2link_planar_B.eigVec[dynsim_2link_planar_B.b_kstr].re < 0.0) {
    dynsim_2link_planar_B.cartOrn[0] =
      -dynsim_2link_planar_B.eigVec[dynsim_2link_planar_B.b_kstr].re;
    dynsim_2link_planar_B.cartOrn[1] =
      -dynsim_2link_planar_B.eigVec[dynsim_2link_planar_B.i].re;
    dynsim_2link_planar_B.cartOrn[2] =
      -dynsim_2link_planar_B.eigVec[dynsim_2link_planar_B.n_c].re;
    dynsim_2link_planar_B.cartOrn[3] =
      -dynsim_2link_planar_B.eigVec[dynsim_2link_planar_B.rtb_MATLABSystem_tmp].
      re;
  }

  // Clock: '<Root>/Clock1' incorporates:
  //   Clock: '<Root>/Clock'

  dynsim_2link_planar_B.vNum = dynsim_2link_planar_M->Timing.t[0];

  // MATLAB Function: '<S12>/MATLAB Function' incorporates:
  //   Clock: '<Root>/Clock1'

  if (dynsim_2link_planar_B.vNum < 0.0) {
    // MATLAB Function: '<Root>/Assign to JointState msg'
    dynsim_2link_planar_B.bid1 = ceil(dynsim_2link_planar_B.vNum);
  } else {
    // MATLAB Function: '<Root>/Assign to JointState msg'
    dynsim_2link_planar_B.bid1 = floor(dynsim_2link_planar_B.vNum);
  }

  dynsim_2link_planar_B.j = (dynsim_2link_planar_B.vNum -
    dynsim_2link_planar_B.bid1) * 1.0E+9;
  if (dynsim_2link_planar_B.j < 0.0) {
    // MATLAB Function: '<Root>/Assign to JointState msg'
    dynsim_2link_planar_B.j = ceil(dynsim_2link_planar_B.j);
  } else {
    // MATLAB Function: '<Root>/Assign to JointState msg'
    dynsim_2link_planar_B.j = floor(dynsim_2link_planar_B.j);
  }

  // BusAssignment: '<S12>/Bus Assignment' incorporates:
  //   Constant: '<S19>/Constant'
  //   MATLAB Function: '<S12>/MATLAB Function'
  //   MATLAB Function: '<S4>/MATLAB Function'

  dynsim_2link_planar_B.BusAssignment = dynsim_2link_planar_P.Constant_Value_l;
  dynsim_2link_planar_B.BusAssignment.Header.Stamp.Sec =
    dynsim_2link_planar_B.bid1;
  dynsim_2link_planar_B.BusAssignment.Header.Stamp.Nsec =
    dynsim_2link_planar_B.j;
  dynsim_2link_planar_B.BusAssignment.Pose.Position.X =
    dynsim_2link_planar_B.T2[12];
  dynsim_2link_planar_B.BusAssignment.Pose.Position.Y =
    dynsim_2link_planar_B.T2[13];
  dynsim_2link_planar_B.BusAssignment.Pose.Position.Z =
    dynsim_2link_planar_B.T2[14];
  dynsim_2link_planar_B.BusAssignment.Pose.Orientation.X =
    dynsim_2link_planar_B.cartOrn[0];
  dynsim_2link_planar_B.BusAssignment.Pose.Orientation.Y =
    dynsim_2link_planar_B.cartOrn[1];
  dynsim_2link_planar_B.BusAssignment.Pose.Orientation.Z =
    dynsim_2link_planar_B.cartOrn[2];
  dynsim_2link_planar_B.BusAssignment.Pose.Orientation.W =
    dynsim_2link_planar_B.cartOrn[3];

  // Outputs for Atomic SubSystem: '<Root>/Publish2'
  // MATLABSystem: '<S9>/SinkBlock'
  Pub_dynsim_2link_planar_178.publish(&dynsim_2link_planar_B.BusAssignment);

  // End of Outputs for SubSystem: '<Root>/Publish2'

  // BusAssignment: '<S12>/Bus Assignment1' incorporates:
  //   Constant: '<S20>/Constant'
  //   MATLAB Function: '<S12>/MATLAB Function'
  //   MATLAB Function: '<S4>/MATLAB Function'

  dynsim_2link_planar_B.BusAssignment1 = dynsim_2link_planar_P.Constant_Value_a;
  dynsim_2link_planar_B.BusAssignment1.Header.Stamp.Sec =
    dynsim_2link_planar_B.bid1;
  dynsim_2link_planar_B.BusAssignment1.Header.Stamp.Nsec =
    dynsim_2link_planar_B.j;
  dynsim_2link_planar_B.BusAssignment1.Twist.Linear.X =
    dynsim_2link_planar_B.MatrixMultiply[3];
  dynsim_2link_planar_B.BusAssignment1.Twist.Linear.Y =
    dynsim_2link_planar_B.MatrixMultiply[4];
  dynsim_2link_planar_B.BusAssignment1.Twist.Linear.Z =
    dynsim_2link_planar_B.MatrixMultiply[5];
  dynsim_2link_planar_B.BusAssignment1.Twist.Angular.X =
    dynsim_2link_planar_B.MatrixMultiply[0];
  dynsim_2link_planar_B.BusAssignment1.Twist.Angular.Y =
    dynsim_2link_planar_B.MatrixMultiply[1];
  dynsim_2link_planar_B.BusAssignment1.Twist.Angular.Z =
    dynsim_2link_planar_B.MatrixMultiply[2];

  // Outputs for Atomic SubSystem: '<Root>/Publish3'
  // MATLABSystem: '<S10>/SinkBlock'
  Pub_dynsim_2link_planar_179.publish(&dynsim_2link_planar_B.BusAssignment1);

  // End of Outputs for SubSystem: '<Root>/Publish3'

  // MATLAB Function: '<Root>/Assign to JointState msg' incorporates:
  //   Constant: '<S5>/Constant'
  //   Integrator: '<S1>/Position'

  dynsim_2link_planar_B.msg = dynsim_2link_planar_P.Constant_Value;
  dynsim_2link_planar_B.msg.Header.Stamp.Sec = dynsim_2link_planar_B.bid1;
  dynsim_2link_planar_B.msg.Header.Stamp.Nsec = dynsim_2link_planar_B.j;
  dynsim_2link_planar_B.msg.Name_SL_Info.CurrentLength = 6U;
  dynsim_2link_planar_B.msg.Position_SL_Info.CurrentLength = 6U;
  dynsim_2link_planar_B.msg.Velocity_SL_Info.CurrentLength = 6U;
  dynsim_2link_planar_B.msg.Name[0].Data_SL_Info.CurrentLength = 11U;
  dynsim_2link_planar_B.msg.Position[0] = dynsim_2link_planar_X.Position_CSTATE
    [0];
  dynsim_2link_planar_B.msg.Velocity[0] = dynsim_2link_planar_B.Velocity[0];
  dynsim_2link_planar_B.msg.Name[1].Data_SL_Info.CurrentLength = 11U;
  dynsim_2link_planar_B.msg.Position[1] = dynsim_2link_planar_X.Position_CSTATE
    [1];
  dynsim_2link_planar_B.msg.Velocity[1] = dynsim_2link_planar_B.Velocity[1];
  dynsim_2link_planar_B.msg.Name[2].Data_SL_Info.CurrentLength = 11U;
  dynsim_2link_planar_B.msg.Position[2] = dynsim_2link_planar_X.Position_CSTATE
    [2];
  dynsim_2link_planar_B.msg.Velocity[2] = dynsim_2link_planar_B.Velocity[2];
  dynsim_2link_planar_B.msg.Name[3].Data_SL_Info.CurrentLength = 11U;
  dynsim_2link_planar_B.msg.Position[3] = dynsim_2link_planar_X.Position_CSTATE
    [3];
  dynsim_2link_planar_B.msg.Velocity[3] = dynsim_2link_planar_B.Velocity[3];
  dynsim_2link_planar_B.msg.Name[4].Data_SL_Info.CurrentLength = 11U;
  dynsim_2link_planar_B.msg.Position[4] = dynsim_2link_planar_X.Position_CSTATE
    [4];
  dynsim_2link_planar_B.msg.Velocity[4] = dynsim_2link_planar_B.Velocity[4];
  for (dynsim_2link_planar_B.b_kstr = 0; dynsim_2link_planar_B.b_kstr < 11;
       dynsim_2link_planar_B.b_kstr++) {
    dynsim_2link_planar_B.b_c.f1[dynsim_2link_planar_B.b_kstr] =
      h[dynsim_2link_planar_B.b_kstr];
    dynsim_2link_planar_B.c.f1[dynsim_2link_planar_B.b_kstr] =
      i[dynsim_2link_planar_B.b_kstr];
    dynsim_2link_planar_B.d.f1[dynsim_2link_planar_B.b_kstr] =
      j[dynsim_2link_planar_B.b_kstr];
    dynsim_2link_planar_B.e.f1[dynsim_2link_planar_B.b_kstr] =
      k[dynsim_2link_planar_B.b_kstr];
    dynsim_2link_planar_B.f.f1[dynsim_2link_planar_B.b_kstr] =
      l[dynsim_2link_planar_B.b_kstr];
    dynsim_2link_planar_B.g.f1[dynsim_2link_planar_B.b_kstr] =
      m[dynsim_2link_planar_B.b_kstr];
    dynsim_2link_planar_B.msg.Name[0].Data[dynsim_2link_planar_B.b_kstr] =
      static_cast<uint8_T>
      (dynsim_2link_planar_B.b_c.f1[dynsim_2link_planar_B.b_kstr]);
    dynsim_2link_planar_B.msg.Name[1].Data[dynsim_2link_planar_B.b_kstr] =
      static_cast<uint8_T>
      (dynsim_2link_planar_B.c.f1[dynsim_2link_planar_B.b_kstr]);
    dynsim_2link_planar_B.msg.Name[2].Data[dynsim_2link_planar_B.b_kstr] =
      static_cast<uint8_T>
      (dynsim_2link_planar_B.d.f1[dynsim_2link_planar_B.b_kstr]);
    dynsim_2link_planar_B.msg.Name[3].Data[dynsim_2link_planar_B.b_kstr] =
      static_cast<uint8_T>
      (dynsim_2link_planar_B.e.f1[dynsim_2link_planar_B.b_kstr]);
    dynsim_2link_planar_B.msg.Name[4].Data[dynsim_2link_planar_B.b_kstr] =
      static_cast<uint8_T>
      (dynsim_2link_planar_B.f.f1[dynsim_2link_planar_B.b_kstr]);
    dynsim_2link_planar_B.msg.Name[5].Data[dynsim_2link_planar_B.b_kstr] =
      static_cast<uint8_T>
      (dynsim_2link_planar_B.g.f1[dynsim_2link_planar_B.b_kstr]);
  }

  dynsim_2link_planar_B.msg.Name[5].Data_SL_Info.CurrentLength = 11U;
  dynsim_2link_planar_B.msg.Position[5] = dynsim_2link_planar_X.Position_CSTATE
    [5];
  dynsim_2link_planar_B.msg.Velocity[5] = dynsim_2link_planar_B.Velocity[5];

  // Outputs for Atomic SubSystem: '<Root>/Publish'
  // MATLABSystem: '<S7>/SinkBlock'
  Pub_dynsim_2link_planar_157.publish(&dynsim_2link_planar_B.msg);

  // End of Outputs for SubSystem: '<Root>/Publish'

  // MATLAB Function: '<Root>/Assign to Time msg'
  if (dynsim_2link_planar_B.vNum < 0.0) {
    dynsim_2link_planar_B.bid1 = ceil(dynsim_2link_planar_B.vNum);
  } else {
    dynsim_2link_planar_B.bid1 = floor(dynsim_2link_planar_B.vNum);
  }

  dynsim_2link_planar_B.msg_k.Clock_.Sec = dynsim_2link_planar_B.bid1;
  dynsim_2link_planar_B.j = (dynsim_2link_planar_B.vNum -
    dynsim_2link_planar_B.bid1) * 1.0E+9;
  if (dynsim_2link_planar_B.j < 0.0) {
    dynsim_2link_planar_B.msg_k.Clock_.Nsec = ceil(dynsim_2link_planar_B.j);
  } else {
    dynsim_2link_planar_B.msg_k.Clock_.Nsec = floor(dynsim_2link_planar_B.j);
  }

  // End of MATLAB Function: '<Root>/Assign to Time msg'

  // Outputs for Atomic SubSystem: '<Root>/Publish1'
  // MATLABSystem: '<S8>/SinkBlock'
  Pub_dynsim_2link_planar_158.publish(&dynsim_2link_planar_B.msg_k);

  // End of Outputs for SubSystem: '<Root>/Publish1'
  if (rtmIsMajorTimeStep(dynsim_2link_planar_M)) {
    // Update for Integrator: '<S1>/Position'
    dynsim_2link_planar_DW.Position_IWORK = 0;

    // Update for Integrator: '<S1>/Velocity'
    dynsim_2link_planar_DW.Velocity_IWORK = 0;
  }                                    // end MajorTimeStep

  if (rtmIsMajorTimeStep(dynsim_2link_planar_M)) {
    rt_ertODEUpdateContinuousStates(&dynsim_2link_planar_M->solverInfo);

    // Update absolute time for base rate
    // The "clockTick0" counts the number of times the code of this task has
    //  been executed. The absolute time is the multiplication of "clockTick0"
    //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
    //  overflow during the application lifespan selected.

    ++dynsim_2link_planar_M->Timing.clockTick0;
    dynsim_2link_planar_M->Timing.t[0] = rtsiGetSolverStopTime
      (&dynsim_2link_planar_M->solverInfo);

    {
      // Update absolute timer for sample time: [0.05s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.05, which is the step size
      //  of the task. Size of "clockTick1" ensures timer will not overflow during the
      //  application lifespan selected.

      dynsim_2link_planar_M->Timing.clockTick1++;
    }
  }                                    // end MajorTimeStep
}

// Derivatives for root system: '<Root>'
void dynsim_2link_planar_derivatives(void)
{
  int32_T i;
  XDot_dynsim_2link_planar_T *_rtXdot;
  _rtXdot = ((XDot_dynsim_2link_planar_T *) dynsim_2link_planar_M->derivs);
  for (i = 0; i < 6; i++) {
    // Derivatives for Integrator: '<S1>/Position'
    _rtXdot->Position_CSTATE[i] = dynsim_2link_planar_B.Velocity[i];

    // Derivatives for Integrator: '<S1>/Velocity'
    _rtXdot->Velocity_CSTATE[i] = dynsim_2link_planar_B.MATLABSystem[i];
  }
}

// Model initialize function
void dynsim_2link_planar_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&dynsim_2link_planar_M->solverInfo,
                          &dynsim_2link_planar_M->Timing.simTimeStep);
    rtsiSetTPtr(&dynsim_2link_planar_M->solverInfo, &rtmGetTPtr
                (dynsim_2link_planar_M));
    rtsiSetStepSizePtr(&dynsim_2link_planar_M->solverInfo,
                       &dynsim_2link_planar_M->Timing.stepSize0);
    rtsiSetdXPtr(&dynsim_2link_planar_M->solverInfo,
                 &dynsim_2link_planar_M->derivs);
    rtsiSetContStatesPtr(&dynsim_2link_planar_M->solverInfo, (real_T **)
                         &dynsim_2link_planar_M->contStates);
    rtsiSetNumContStatesPtr(&dynsim_2link_planar_M->solverInfo,
      &dynsim_2link_planar_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&dynsim_2link_planar_M->solverInfo,
      &dynsim_2link_planar_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&dynsim_2link_planar_M->solverInfo,
      &dynsim_2link_planar_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&dynsim_2link_planar_M->solverInfo,
      &dynsim_2link_planar_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&dynsim_2link_planar_M->solverInfo,
                          (&rtmGetErrorStatus(dynsim_2link_planar_M)));
    rtsiSetRTModelPtr(&dynsim_2link_planar_M->solverInfo, dynsim_2link_planar_M);
  }

  rtsiSetSimTimeStep(&dynsim_2link_planar_M->solverInfo, MAJOR_TIME_STEP);
  dynsim_2link_planar_M->intgData.y = dynsim_2link_planar_M->odeY;
  dynsim_2link_planar_M->intgData.f[0] = dynsim_2link_planar_M->odeF[0];
  dynsim_2link_planar_M->intgData.f[1] = dynsim_2link_planar_M->odeF[1];
  dynsim_2link_planar_M->intgData.f[2] = dynsim_2link_planar_M->odeF[2];
  dynsim_2link_planar_M->contStates = ((X_dynsim_2link_planar_T *)
    &dynsim_2link_planar_X);
  rtsiSetSolverData(&dynsim_2link_planar_M->solverInfo, static_cast<void *>
                    (&dynsim_2link_planar_M->intgData));
  rtsiSetSolverName(&dynsim_2link_planar_M->solverInfo,"ode3");
  rtmSetTPtr(dynsim_2link_planar_M, &dynsim_2link_planar_M->Timing.tArray[0]);
  dynsim_2link_planar_M->Timing.stepSize0 = 0.05;
  rtmSetFirstInitCond(dynsim_2link_planar_M, 1);

  {
    char_T tmp[14];
    char_T tmp_0[15];
    char_T tmp_1[7];
    char_T tmp_2[12];
    char_T tmp_3[13];
    int32_T i;
    static const char_T tmp_4[13] = { '/', 'j', 'o', 'i', 'n', 't', '_', 't',
      'o', 'r', 'q', 'u', 'e' };

    static const char_T tmp_5[14] = { '/', 'c', 'a', 'r', 't', 'e', 's', 'i',
      'a', 'n', '_', 'p', 'o', 's' };

    static const char_T tmp_6[14] = { '/', 'c', 'a', 'r', 't', 'e', 's', 'i',
      'a', 'n', '_', 'v', 'e', 'l' };

    static const char_T tmp_7[13] = { '/', 'j', 'o', 'i', 'n', 't', '_', 's',
      't', 'a', 't', 'e', 's' };

    static const char_T tmp_8[6] = { '/', 'c', 'l', 'o', 'c', 'k' };

    static const char_T tmp_9[11] = { '/', 'q', '1', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_a[11] = { '/', 'q', '2', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_b[11] = { '/', 'q', '3', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_c[11] = { '/', 'q', '4', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_d[11] = { '/', 'q', '5', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_e[11] = { '/', 'q', '6', '_', 'i', 'n', 'i', 't',
      'i', 'a', 'l' };

    static const char_T tmp_f[12] = { '/', 'q', 'v', '1', '_', 'i', 'n', 'i',
      't', 'i', 'a', 'l' };

    static const char_T tmp_g[12] = { '/', 'q', 'v', '2', '_', 'i', 'n', 'i',
      't', 'i', 'a', 'l' };

    static const char_T tmp_h[12] = { '/', 'q', 'v', '3', '_', 'i', 'n', 'i',
      't', 'i', 'a', 'l' };

    static const char_T tmp_i[12] = { '/', 'q', 'v', '4', '_', 'i', 'n', 'i',
      't', 'i', 'a', 'l' };

    static const char_T tmp_j[12] = { '/', 'q', 'v', '5', '_', 'i', 'n', 'i',
      't', 'i', 'a', 'l' };

    static const char_T tmp_k[12] = { '/', 'q', 'v', '6', '_', 'i', 'n', 'i',
      't', 'i', 'a', 'l' };

    // InitializeConditions for Integrator: '<S1>/Position' incorporates:
    //   Integrator: '<S1>/Velocity'

    if (rtmIsFirstInitCond(dynsim_2link_planar_M)) {
      dynsim_2link_planar_X.Position_CSTATE[0] = 0.0;
      dynsim_2link_planar_X.Position_CSTATE[1] = 0.0;
      dynsim_2link_planar_X.Position_CSTATE[2] = 0.0;
      dynsim_2link_planar_X.Position_CSTATE[3] = 0.0;
      dynsim_2link_planar_X.Position_CSTATE[4] = 0.0;
      dynsim_2link_planar_X.Position_CSTATE[5] = 0.0;
      dynsim_2link_planar_X.Velocity_CSTATE[0] = 0.0;
      dynsim_2link_planar_X.Velocity_CSTATE[1] = 0.0;
      dynsim_2link_planar_X.Velocity_CSTATE[2] = 0.0;
      dynsim_2link_planar_X.Velocity_CSTATE[3] = 0.0;
      dynsim_2link_planar_X.Velocity_CSTATE[4] = 0.0;
      dynsim_2link_planar_X.Velocity_CSTATE[5] = 0.0;
    }

    dynsim_2link_planar_DW.Position_IWORK = 1;

    // End of InitializeConditions for Integrator: '<S1>/Position'

    // InitializeConditions for Integrator: '<S1>/Velocity'
    dynsim_2link_planar_DW.Velocity_IWORK = 1;

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S11>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S18>/Out1'
    dynsim_2link_planar_B.In1 = dynsim_2link_planar_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S11>/Enabled Subsystem'

    // Start for MATLABSystem: '<S11>/SourceBlock'
    dynsim_2link_planar_DW.obj_fk.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_fk.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      tmp[i] = tmp_4[i];
    }

    tmp[13] = '\x00';
    Sub_dynsim_2link_planar_160.createSubscriber(tmp, 1);
    dynsim_2link_planar_DW.obj_fk.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish2'
    // Start for MATLABSystem: '<S9>/SinkBlock'
    dynsim_2link_planar_DW.obj_g.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_g.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_0[i] = tmp_5[i];
    }

    tmp_0[14] = '\x00';
    Pub_dynsim_2link_planar_178.createPublisher(tmp_0, 1);
    dynsim_2link_planar_DW.obj_g.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S9>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish2'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish3'
    // Start for MATLABSystem: '<S10>/SinkBlock'
    dynsim_2link_planar_DW.obj_dj.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_dj.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      tmp_0[i] = tmp_6[i];
    }

    tmp_0[14] = '\x00';
    Pub_dynsim_2link_planar_179.createPublisher(tmp_0, 1);
    dynsim_2link_planar_DW.obj_dj.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish3'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish'
    // Start for MATLABSystem: '<S7>/SinkBlock'
    dynsim_2link_planar_DW.obj_c.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_c.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      tmp[i] = tmp_7[i];
    }

    tmp[13] = '\x00';
    Pub_dynsim_2link_planar_157.createPublisher(tmp, 1);
    dynsim_2link_planar_DW.obj_c.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S7>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish1'
    // Start for MATLABSystem: '<S8>/SinkBlock'
    dynsim_2link_planar_DW.obj_iv.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_iv.isInitialized = 1;
    for (i = 0; i < 6; i++) {
      tmp_1[i] = tmp_8[i];
    }

    tmp_1[6] = '\x00';
    Pub_dynsim_2link_planar_158.createPublisher(tmp_1, 1);
    dynsim_2link_planar_DW.obj_iv.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S8>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish1'

    // Start for MATLABSystem: '<S14>/Get Parameter'
    dynsim_2link_planar_DW.obj_d.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_d.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      tmp_2[i] = tmp_9[i];
    }

    tmp_2[11] = '\x00';
    ParamGet_dynsim_2link_planar_131.initialize(tmp_2);
    ParamGet_dynsim_2link_planar_131.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_2link_planar_131.set_initial_value(0.0);
    dynsim_2link_planar_DW.obj_d.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S14>/Get Parameter'

    // Start for MATLABSystem: '<S14>/Get Parameter1'
    dynsim_2link_planar_DW.obj_l.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_l.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      tmp_2[i] = tmp_a[i];
    }

    tmp_2[11] = '\x00';
    ParamGet_dynsim_2link_planar_132.initialize(tmp_2);
    ParamGet_dynsim_2link_planar_132.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_2link_planar_132.set_initial_value(0.0);
    dynsim_2link_planar_DW.obj_l.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S14>/Get Parameter1'

    // Start for MATLABSystem: '<S14>/Get Parameter4'
    dynsim_2link_planar_DW.obj_a.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_a.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      tmp_2[i] = tmp_b[i];
    }

    tmp_2[11] = '\x00';
    ParamGet_dynsim_2link_planar_137.initialize(tmp_2);
    ParamGet_dynsim_2link_planar_137.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_2link_planar_137.set_initial_value(0.0);
    dynsim_2link_planar_DW.obj_a.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S14>/Get Parameter4'

    // Start for MATLABSystem: '<S14>/Get Parameter5'
    dynsim_2link_planar_DW.obj_dp.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_dp.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      tmp_2[i] = tmp_c[i];
    }

    tmp_2[11] = '\x00';
    ParamGet_dynsim_2link_planar_138.initialize(tmp_2);
    ParamGet_dynsim_2link_planar_138.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_2link_planar_138.set_initial_value(0.0);
    dynsim_2link_planar_DW.obj_dp.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S14>/Get Parameter5'

    // Start for MATLABSystem: '<S14>/Get Parameter6'
    dynsim_2link_planar_DW.obj_h.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_h.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      tmp_2[i] = tmp_d[i];
    }

    tmp_2[11] = '\x00';
    ParamGet_dynsim_2link_planar_139.initialize(tmp_2);
    ParamGet_dynsim_2link_planar_139.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_2link_planar_139.set_initial_value(0.0);
    dynsim_2link_planar_DW.obj_h.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S14>/Get Parameter6'

    // Start for MATLABSystem: '<S14>/Get Parameter7'
    dynsim_2link_planar_DW.obj_f0.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_f0.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      tmp_2[i] = tmp_e[i];
    }

    tmp_2[11] = '\x00';
    ParamGet_dynsim_2link_planar_140.initialize(tmp_2);
    ParamGet_dynsim_2link_planar_140.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_2link_planar_140.set_initial_value(0.0);
    dynsim_2link_planar_DW.obj_f0.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S14>/Get Parameter7'
    emxInitStruct_robotics_slmanip_(&dynsim_2link_planar_DW.obj_f);
    emxInitStruct_n_robotics_manip_(&dynsim_2link_planar_DW.gobj_1_m);
    emxInitStruct_n_robotics_manip_(&dynsim_2link_planar_DW.gobj_16_a);
    emxInitStruct_n_robotics_manip_(&dynsim_2link_planar_DW.gobj_15_c);
    emxInitStruct_n_robotics_manip_(&dynsim_2link_planar_DW.gobj_14_o);
    emxInitStruct_n_robotics_manip_(&dynsim_2link_planar_DW.gobj_13_i);
    emxInitStruct_n_robotics_manip_(&dynsim_2link_planar_DW.gobj_12_d);
    emxInitStruct_n_robotics_manip_(&dynsim_2link_planar_DW.gobj_11_c);
    emxInitStruct_n_robotics_manip_(&dynsim_2link_planar_DW.gobj_10_l);
    emxInitStruct_n_robotics_manip_(&dynsim_2link_planar_DW.gobj_9_p);
    emxInitStruct_n_robotics_manip_(&dynsim_2link_planar_DW.gobj_8_i);
    emxInitStruct_n_robotics_manip_(&dynsim_2link_planar_DW.gobj_7_o);
    emxInitStruct_n_robotics_manip_(&dynsim_2link_planar_DW.gobj_6_a);
    emxInitStruct_n_robotics_manip_(&dynsim_2link_planar_DW.gobj_5_a);
    emxInitStruct_n_robotics_manip_(&dynsim_2link_planar_DW.gobj_4_g);
    emxInitStruct_n_robotics_manip_(&dynsim_2link_planar_DW.gobj_3_g);
    emxInitStruct_n_robotics_manip_(&dynsim_2link_planar_DW.gobj_2_o);

    // Start for MATLABSystem: '<S15>/MATLAB System'
    dynsim_2link_planar_DW.obj_f.isInitialized = 0;
    dynsim_2link_planar_DW.obj_f.isInitialized = 1;
    d_RigidBodyTree_RigidBodyTree_a(&dynsim_2link_planar_DW.obj_f.TreeInternal,
      &dynsim_2link_planar_DW.gobj_2_o, &dynsim_2link_planar_DW.gobj_4_g,
      &dynsim_2link_planar_DW.gobj_5_a, &dynsim_2link_planar_DW.gobj_6_a,
      &dynsim_2link_planar_DW.gobj_7_o, &dynsim_2link_planar_DW.gobj_8_i,
      &dynsim_2link_planar_DW.gobj_9_p, &dynsim_2link_planar_DW.gobj_3_g);
    emxInitStruct_robotics_slmani_a(&dynsim_2link_planar_DW.obj_i);
    emxInitStruct_n_robotics_mani_a(&dynsim_2link_planar_DW.gobj_1_p);
    emxInitStruct_n_robotics_mani_a(&dynsim_2link_planar_DW.gobj_16_n);
    emxInitStruct_n_robotics_mani_a(&dynsim_2link_planar_DW.gobj_15_e);
    emxInitStruct_n_robotics_mani_a(&dynsim_2link_planar_DW.gobj_14_om);
    emxInitStruct_n_robotics_mani_a(&dynsim_2link_planar_DW.gobj_13_f);
    emxInitStruct_n_robotics_mani_a(&dynsim_2link_planar_DW.gobj_12_n);
    emxInitStruct_n_robotics_mani_a(&dynsim_2link_planar_DW.gobj_11_d);
    emxInitStruct_n_robotics_mani_a(&dynsim_2link_planar_DW.gobj_10_g);
    emxInitStruct_n_robotics_mani_a(&dynsim_2link_planar_DW.gobj_9_c);
    emxInitStruct_n_robotics_mani_a(&dynsim_2link_planar_DW.gobj_8_p);
    emxInitStruct_n_robotics_mani_a(&dynsim_2link_planar_DW.gobj_7_n);
    emxInitStruct_n_robotics_mani_a(&dynsim_2link_planar_DW.gobj_6_j);
    emxInitStruct_n_robotics_mani_a(&dynsim_2link_planar_DW.gobj_5_at);
    emxInitStruct_n_robotics_mani_a(&dynsim_2link_planar_DW.gobj_4_h);
    emxInitStruct_n_robotics_mani_a(&dynsim_2link_planar_DW.gobj_3_g0);
    emxInitStruct_n_robotics_mani_a(&dynsim_2link_planar_DW.gobj_2_on);

    // Start for MATLABSystem: '<S16>/MATLAB System'
    dynsim_2link_planar_DW.obj_i.isInitialized = 0;
    dynsim_2link_planar_DW.obj_i.isInitialized = 1;
    RigidBodyTree_RigidBodyTree_aw(&dynsim_2link_planar_DW.obj_i.TreeInternal,
      &dynsim_2link_planar_DW.gobj_2_on, &dynsim_2link_planar_DW.gobj_4_h,
      &dynsim_2link_planar_DW.gobj_5_at, &dynsim_2link_planar_DW.gobj_6_j,
      &dynsim_2link_planar_DW.gobj_7_n, &dynsim_2link_planar_DW.gobj_8_p,
      &dynsim_2link_planar_DW.gobj_9_c, &dynsim_2link_planar_DW.gobj_3_g0);

    // Start for MATLABSystem: '<S14>/Get Parameter2'
    dynsim_2link_planar_DW.obj_e.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_e.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      tmp_3[i] = tmp_f[i];
    }

    tmp_3[12] = '\x00';
    ParamGet_dynsim_2link_planar_135.initialize(tmp_3);
    ParamGet_dynsim_2link_planar_135.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_2link_planar_135.set_initial_value(0.0);
    dynsim_2link_planar_DW.obj_e.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S14>/Get Parameter2'

    // Start for MATLABSystem: '<S14>/Get Parameter3'
    dynsim_2link_planar_DW.obj_ez.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_ez.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      tmp_3[i] = tmp_g[i];
    }

    tmp_3[12] = '\x00';
    ParamGet_dynsim_2link_planar_136.initialize(tmp_3);
    ParamGet_dynsim_2link_planar_136.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_2link_planar_136.set_initial_value(0.0);
    dynsim_2link_planar_DW.obj_ez.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S14>/Get Parameter3'

    // Start for MATLABSystem: '<S14>/Get Parameter8'
    dynsim_2link_planar_DW.obj_io.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_io.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      tmp_3[i] = tmp_h[i];
    }

    tmp_3[12] = '\x00';
    ParamGet_dynsim_2link_planar_141.initialize(tmp_3);
    ParamGet_dynsim_2link_planar_141.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_2link_planar_141.set_initial_value(0.0);
    dynsim_2link_planar_DW.obj_io.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S14>/Get Parameter8'

    // Start for MATLABSystem: '<S14>/Get Parameter9'
    dynsim_2link_planar_DW.obj_ezd.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_ezd.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      tmp_3[i] = tmp_i[i];
    }

    tmp_3[12] = '\x00';
    ParamGet_dynsim_2link_planar_142.initialize(tmp_3);
    ParamGet_dynsim_2link_planar_142.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_2link_planar_142.set_initial_value(0.0);
    dynsim_2link_planar_DW.obj_ezd.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S14>/Get Parameter9'

    // Start for MATLABSystem: '<S14>/Get Parameter10'
    dynsim_2link_planar_DW.obj_n.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_n.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      tmp_3[i] = tmp_j[i];
    }

    tmp_3[12] = '\x00';
    ParamGet_dynsim_2link_planar_133.initialize(tmp_3);
    ParamGet_dynsim_2link_planar_133.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_2link_planar_133.set_initial_value(0.0);
    dynsim_2link_planar_DW.obj_n.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S14>/Get Parameter10'

    // Start for MATLABSystem: '<S14>/Get Parameter11'
    dynsim_2link_planar_DW.obj_p.matlabCodegenIsDeleted = false;
    dynsim_2link_planar_DW.obj_p.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      tmp_3[i] = tmp_k[i];
    }

    tmp_3[12] = '\x00';
    ParamGet_dynsim_2link_planar_134.initialize(tmp_3);
    ParamGet_dynsim_2link_planar_134.initialize_error_codes(0, 1, 2, 3);
    ParamGet_dynsim_2link_planar_134.set_initial_value(0.0);
    dynsim_2link_planar_DW.obj_p.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S14>/Get Parameter11'
    emxInitStruct_robotics_slman_aw(&dynsim_2link_planar_DW.obj);
    emxInitStruct_n_robotics_man_aw(&dynsim_2link_planar_DW.gobj_1);
    emxInitStruct_n_robotics_man_aw(&dynsim_2link_planar_DW.gobj_16);
    emxInitStruct_n_robotics_man_aw(&dynsim_2link_planar_DW.gobj_15);
    emxInitStruct_n_robotics_man_aw(&dynsim_2link_planar_DW.gobj_14);
    emxInitStruct_n_robotics_man_aw(&dynsim_2link_planar_DW.gobj_13);
    emxInitStruct_n_robotics_man_aw(&dynsim_2link_planar_DW.gobj_12);
    emxInitStruct_n_robotics_man_aw(&dynsim_2link_planar_DW.gobj_11);
    emxInitStruct_n_robotics_man_aw(&dynsim_2link_planar_DW.gobj_10);
    emxInitStruct_n_robotics_man_aw(&dynsim_2link_planar_DW.gobj_9);
    emxInitStruct_n_robotics_man_aw(&dynsim_2link_planar_DW.gobj_8);
    emxInitStruct_n_robotics_man_aw(&dynsim_2link_planar_DW.gobj_7);
    emxInitStruct_n_robotics_man_aw(&dynsim_2link_planar_DW.gobj_6);
    emxInitStruct_n_robotics_man_aw(&dynsim_2link_planar_DW.gobj_5);
    emxInitStruct_n_robotics_man_aw(&dynsim_2link_planar_DW.gobj_4);
    emxInitStruct_n_robotics_man_aw(&dynsim_2link_planar_DW.gobj_3);
    emxInitStruct_n_robotics_man_aw(&dynsim_2link_planar_DW.gobj_2);

    // Start for MATLABSystem: '<S13>/MATLAB System'
    dynsim_2link_planar_DW.obj.isInitialized = 0;
    dynsim_2link_planar_DW.obj.isInitialized = 1;
    dyn_RigidBodyTree_RigidBodyTree(&dynsim_2link_planar_DW.obj.TreeInternal,
      &dynsim_2link_planar_DW.gobj_2, &dynsim_2link_planar_DW.gobj_4,
      &dynsim_2link_planar_DW.gobj_5, &dynsim_2link_planar_DW.gobj_6,
      &dynsim_2link_planar_DW.gobj_7, &dynsim_2link_planar_DW.gobj_8,
      &dynsim_2link_planar_DW.gobj_9, &dynsim_2link_planar_DW.gobj_3);
  }

  // set "at time zero" to false
  if (rtmIsFirstInitCond(dynsim_2link_planar_M)) {
    rtmSetFirstInitCond(dynsim_2link_planar_M, 0);
  }
}

// Model terminate function
void dynsim_2link_planar_terminate(void)
{
  // Terminate for MATLABSystem: '<S14>/Get Parameter'
  matlabCodegenHandle_matlabCodeg(&dynsim_2link_planar_DW.obj_d);

  // Terminate for MATLABSystem: '<S14>/Get Parameter1'
  matlabCodegenHandle_matlabCodeg(&dynsim_2link_planar_DW.obj_l);

  // Terminate for MATLABSystem: '<S14>/Get Parameter4'
  matlabCodegenHandle_matlabCodeg(&dynsim_2link_planar_DW.obj_a);

  // Terminate for MATLABSystem: '<S14>/Get Parameter5'
  matlabCodegenHandle_matlabCodeg(&dynsim_2link_planar_DW.obj_dp);

  // Terminate for MATLABSystem: '<S14>/Get Parameter6'
  matlabCodegenHandle_matlabCodeg(&dynsim_2link_planar_DW.obj_h);

  // Terminate for MATLABSystem: '<S14>/Get Parameter7'
  matlabCodegenHandle_matlabCodeg(&dynsim_2link_planar_DW.obj_f0);
  emxFreeStruct_robotics_slmanip_(&dynsim_2link_planar_DW.obj_f);
  emxFreeStruct_n_robotics_manip_(&dynsim_2link_planar_DW.gobj_1_m);
  emxFreeStruct_n_robotics_manip_(&dynsim_2link_planar_DW.gobj_16_a);
  emxFreeStruct_n_robotics_manip_(&dynsim_2link_planar_DW.gobj_15_c);
  emxFreeStruct_n_robotics_manip_(&dynsim_2link_planar_DW.gobj_14_o);
  emxFreeStruct_n_robotics_manip_(&dynsim_2link_planar_DW.gobj_13_i);
  emxFreeStruct_n_robotics_manip_(&dynsim_2link_planar_DW.gobj_12_d);
  emxFreeStruct_n_robotics_manip_(&dynsim_2link_planar_DW.gobj_11_c);
  emxFreeStruct_n_robotics_manip_(&dynsim_2link_planar_DW.gobj_10_l);
  emxFreeStruct_n_robotics_manip_(&dynsim_2link_planar_DW.gobj_9_p);
  emxFreeStruct_n_robotics_manip_(&dynsim_2link_planar_DW.gobj_8_i);
  emxFreeStruct_n_robotics_manip_(&dynsim_2link_planar_DW.gobj_7_o);
  emxFreeStruct_n_robotics_manip_(&dynsim_2link_planar_DW.gobj_6_a);
  emxFreeStruct_n_robotics_manip_(&dynsim_2link_planar_DW.gobj_5_a);
  emxFreeStruct_n_robotics_manip_(&dynsim_2link_planar_DW.gobj_4_g);
  emxFreeStruct_n_robotics_manip_(&dynsim_2link_planar_DW.gobj_3_g);
  emxFreeStruct_n_robotics_manip_(&dynsim_2link_planar_DW.gobj_2_o);
  emxFreeStruct_robotics_slmani_a(&dynsim_2link_planar_DW.obj_i);
  emxFreeStruct_n_robotics_mani_a(&dynsim_2link_planar_DW.gobj_1_p);
  emxFreeStruct_n_robotics_mani_a(&dynsim_2link_planar_DW.gobj_16_n);
  emxFreeStruct_n_robotics_mani_a(&dynsim_2link_planar_DW.gobj_15_e);
  emxFreeStruct_n_robotics_mani_a(&dynsim_2link_planar_DW.gobj_14_om);
  emxFreeStruct_n_robotics_mani_a(&dynsim_2link_planar_DW.gobj_13_f);
  emxFreeStruct_n_robotics_mani_a(&dynsim_2link_planar_DW.gobj_12_n);
  emxFreeStruct_n_robotics_mani_a(&dynsim_2link_planar_DW.gobj_11_d);
  emxFreeStruct_n_robotics_mani_a(&dynsim_2link_planar_DW.gobj_10_g);
  emxFreeStruct_n_robotics_mani_a(&dynsim_2link_planar_DW.gobj_9_c);
  emxFreeStruct_n_robotics_mani_a(&dynsim_2link_planar_DW.gobj_8_p);
  emxFreeStruct_n_robotics_mani_a(&dynsim_2link_planar_DW.gobj_7_n);
  emxFreeStruct_n_robotics_mani_a(&dynsim_2link_planar_DW.gobj_6_j);
  emxFreeStruct_n_robotics_mani_a(&dynsim_2link_planar_DW.gobj_5_at);
  emxFreeStruct_n_robotics_mani_a(&dynsim_2link_planar_DW.gobj_4_h);
  emxFreeStruct_n_robotics_mani_a(&dynsim_2link_planar_DW.gobj_3_g0);
  emxFreeStruct_n_robotics_mani_a(&dynsim_2link_planar_DW.gobj_2_on);

  // Terminate for MATLABSystem: '<S14>/Get Parameter2'
  matlabCodegenHandle_matlabCodeg(&dynsim_2link_planar_DW.obj_e);

  // Terminate for MATLABSystem: '<S14>/Get Parameter3'
  matlabCodegenHandle_matlabCodeg(&dynsim_2link_planar_DW.obj_ez);

  // Terminate for MATLABSystem: '<S14>/Get Parameter8'
  matlabCodegenHandle_matlabCodeg(&dynsim_2link_planar_DW.obj_io);

  // Terminate for MATLABSystem: '<S14>/Get Parameter9'
  matlabCodegenHandle_matlabCodeg(&dynsim_2link_planar_DW.obj_ezd);

  // Terminate for MATLABSystem: '<S14>/Get Parameter10'
  matlabCodegenHandle_matlabCodeg(&dynsim_2link_planar_DW.obj_n);

  // Terminate for MATLABSystem: '<S14>/Get Parameter11'
  matlabCodegenHandle_matlabCodeg(&dynsim_2link_planar_DW.obj_p);

  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S11>/SourceBlock'
  dynsim_2link__matlabCodegenHa_p(&dynsim_2link_planar_DW.obj_fk);

  // End of Terminate for SubSystem: '<Root>/Subscribe'
  emxFreeStruct_robotics_slman_aw(&dynsim_2link_planar_DW.obj);
  emxFreeStruct_n_robotics_man_aw(&dynsim_2link_planar_DW.gobj_1);
  emxFreeStruct_n_robotics_man_aw(&dynsim_2link_planar_DW.gobj_16);
  emxFreeStruct_n_robotics_man_aw(&dynsim_2link_planar_DW.gobj_15);
  emxFreeStruct_n_robotics_man_aw(&dynsim_2link_planar_DW.gobj_14);
  emxFreeStruct_n_robotics_man_aw(&dynsim_2link_planar_DW.gobj_13);
  emxFreeStruct_n_robotics_man_aw(&dynsim_2link_planar_DW.gobj_12);
  emxFreeStruct_n_robotics_man_aw(&dynsim_2link_planar_DW.gobj_11);
  emxFreeStruct_n_robotics_man_aw(&dynsim_2link_planar_DW.gobj_10);
  emxFreeStruct_n_robotics_man_aw(&dynsim_2link_planar_DW.gobj_9);
  emxFreeStruct_n_robotics_man_aw(&dynsim_2link_planar_DW.gobj_8);
  emxFreeStruct_n_robotics_man_aw(&dynsim_2link_planar_DW.gobj_7);
  emxFreeStruct_n_robotics_man_aw(&dynsim_2link_planar_DW.gobj_6);
  emxFreeStruct_n_robotics_man_aw(&dynsim_2link_planar_DW.gobj_5);
  emxFreeStruct_n_robotics_man_aw(&dynsim_2link_planar_DW.gobj_4);
  emxFreeStruct_n_robotics_man_aw(&dynsim_2link_planar_DW.gobj_3);
  emxFreeStruct_n_robotics_man_aw(&dynsim_2link_planar_DW.gobj_2);

  // Terminate for Atomic SubSystem: '<Root>/Publish2'
  // Terminate for MATLABSystem: '<S9>/SinkBlock'
  matlabCodegenHandl_awtnueqcp3y5(&dynsim_2link_planar_DW.obj_g);

  // End of Terminate for SubSystem: '<Root>/Publish2'

  // Terminate for Atomic SubSystem: '<Root>/Publish3'
  // Terminate for MATLABSystem: '<S10>/SinkBlock'
  matlabCodegenHandl_awtnueqcp3y5(&dynsim_2link_planar_DW.obj_dj);

  // End of Terminate for SubSystem: '<Root>/Publish3'

  // Terminate for Atomic SubSystem: '<Root>/Publish'
  // Terminate for MATLABSystem: '<S7>/SinkBlock'
  matlabCodegenHandl_awtnueqcp3y5(&dynsim_2link_planar_DW.obj_c);

  // End of Terminate for SubSystem: '<Root>/Publish'

  // Terminate for Atomic SubSystem: '<Root>/Publish1'
  // Terminate for MATLABSystem: '<S8>/SinkBlock'
  matlabCodegenHandl_awtnueqcp3y5(&dynsim_2link_planar_DW.obj_iv);

  // End of Terminate for SubSystem: '<Root>/Publish1'
}

//
// File trailer for generated code.
//
// [EOF]
//
