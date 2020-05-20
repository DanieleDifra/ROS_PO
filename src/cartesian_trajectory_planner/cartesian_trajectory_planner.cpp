//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: cartesian_trajectory_planner.cpp
//
// Code generated for Simulink model 'cartesian_trajectory_planner'.
//
// Model version                  : 1.129
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Wed May 20 16:41:34 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "cartesian_trajectory_planner.h"
#include "cartesian_trajectory_planner_private.h"

// Block signals (default storage)
B_cartesian_trajectory_planne_T cartesian_trajectory_planner_B;

// Block states (default storage)
DW_cartesian_trajectory_plann_T cartesian_trajectory_planner_DW;

// Real-time model
RT_MODEL_cartesian_trajectory_T cartesian_trajectory_planner_M_ =
  RT_MODEL_cartesian_trajectory_T();
RT_MODEL_cartesian_trajectory_T *const cartesian_trajectory_planner_M =
  &cartesian_trajectory_planner_M_;

// Forward declaration for local functions
static real_T cartesian_trajectory_pla_norm_l(const real_T x[3]);
static void cartesian_trajec_emxInit_real_T(emxArray_real_T_cartesian_tra_T
  **pEmxArray, int32_T numDimensions);
static void cartes_emxEnsureCapacity_real_T(emxArray_real_T_cartesian_tra_T
  *emxArray, int32_T oldNumel);
static void cartesian_trajec_emxInit_char_T(emxArray_char_T_cartesian_tra_T
  **pEmxArray, int32_T numDimensions);
static void cartes_emxEnsureCapacity_char_T(emxArray_char_T_cartesian_tra_T
  *emxArray, int32_T oldNumel);
static void cartesian_trajec_emxFree_real_T(emxArray_real_T_cartesian_tra_T
  **pEmxArray);
static void cartesian_trajec_emxFree_char_T(emxArray_char_T_cartesian_tra_T
  **pEmxArray);
static void car_inverseKinematics_setupImpl(b_inverseKinematics_cartesian_T *obj,
  f_robotics_manip_internal_IKE_T *iobj_0);
static void cartesian_t_emxInit_f_cell_wrap(emxArray_f_cell_wrap_cartesia_T
  **pEmxArray, int32_T numDimensions);
static void c_inverseKinematics_setPoseGoal(b_inverseKinematics_cartesian_T *obj,
  const real_T tform[16], const real_T weights[6]);
static void RigidBodyTree_validateConfigu_a(x_robotics_manip_internal_Rig_T *obj,
  real_T Q[6]);
static boolean_T cartesian_trajectory_pla_strcmp(const
  emxArray_char_T_cartesian_tra_T *a, const emxArray_char_T_cartesian_tra_T *b);
static void rigidBodyJoint_get_JointAxis_as(const
  c_rigidBodyJoint_cartesian_as_T *obj, real_T ax[3]);
static void cartesian_trajectory_planne_cat(real_T varargin_1, real_T varargin_2,
  real_T varargin_3, real_T varargin_4, real_T varargin_5, real_T varargin_6,
  real_T varargin_7, real_T varargin_8, real_T varargin_9, real_T y[9]);
static void rigidBodyJoint_transformBodyT_a(const
  c_rigidBodyJoint_cartesian_as_T *obj, const real_T q_data[], const int32_T
  *q_size, real_T T[16]);
static void rigidBodyJoint_transformBodyToP(const
  c_rigidBodyJoint_cartesian_as_T *obj, real_T T[16]);
static void RigidBodyTree_efficientFKAndJac(x_robotics_manip_internal_Rig_T *obj,
  const real_T qv[6], const emxArray_char_T_cartesian_tra_T *body1Name, real_T
  T_data[], int32_T T_size[2], emxArray_real_T_cartesian_tra_T *Jac);
static creal_T cartesian_trajectory_plann_sqrt(const creal_T x);
static real_T cartesian_trajectory_plan_xnrm2(int32_T n, const real_T x[9],
  int32_T ix0);
static real_T cartesian_trajectory_plan_xdotc(int32_T n, const real_T x[9],
  int32_T ix0, const real_T y[9], int32_T iy0);
static void cartesian_trajectory_plan_xaxpy(int32_T n, real_T a, int32_T ix0,
  const real_T y[9], int32_T iy0, real_T b_y[9]);
static real_T cartesian_trajectory_pl_xnrm2_a(const real_T x[3], int32_T ix0);
static void cartesian_trajectory__xaxpy_ast(int32_T n, real_T a, const real_T x
  [9], int32_T ix0, real_T y[3], int32_T iy0);
static void cartesian_trajectory_p_xaxpy_as(int32_T n, real_T a, const real_T x
  [3], int32_T ix0, const real_T y[9], int32_T iy0, real_T b_y[9]);
static void cartesian_trajectory_plan_xswap(const real_T x[9], int32_T ix0,
  int32_T iy0, real_T b_x[9]);
static void cartesian_trajectory_plan_xrotg(real_T a, real_T b, real_T *b_a,
  real_T *b_b, real_T *c, real_T *s);
static void cartesian_trajectory_plann_xrot(const real_T x[9], int32_T ix0,
  int32_T iy0, real_T c, real_T s, real_T b_x[9]);
static void cartesian_trajectory_planne_svd(const real_T A[9], real_T U[9],
  real_T s[3], real_T V[9]);
static void cartesian_trajectory_rotm2axang(const real_T R[9], real_T axang[4]);
static void cartesian_IKHelpers_computeCost(const real_T x[6],
  f_robotics_manip_internal_IKE_T *args, real_T *cost, real_T W[36],
  emxArray_real_T_cartesian_tra_T *Jac, f_robotics_manip_internal_IKE_T **b_args);
static void cartesian_trajectory_planne_eye(real_T b_I[36]);
static void cartesian_tra_emxInit_boolean_T(emxArray_boolean_T_cartesian__T
  **pEmxArray, int32_T numDimensions);
static void cartesian_traje_emxInit_int32_T(emxArray_int32_T_cartesian_tr_T
  **pEmxArray, int32_T numDimensions);
static void car_emxEnsureCapacity_boolean_T(emxArray_boolean_T_cartesian__T
  *emxArray, int32_T oldNumel);
static void carte_emxEnsureCapacity_int32_T(emxArray_int32_T_cartesian_tr_T
  *emxArray, int32_T oldNumel);
static real_T cartesian_trajectory_pla_norm_a(const real_T x[6]);
static real_T SystemTimeProvider_getElapsedTi(const
  f_robotics_core_internal_Syst_T *obj);
static real_T cartesian_trajectory_p_xnrm2_as(int32_T n, const
  emxArray_real_T_cartesian_tra_T *x, int32_T ix0);
static void cartesian_trajectory_pla_qrpf_a(const
  emxArray_real_T_cartesian_tra_T *A, int32_T m, int32_T n,
  emxArray_real_T_cartesian_tra_T *tau, const emxArray_int32_T_cartesian_tr_T
  *jpvt, emxArray_real_T_cartesian_tra_T *b_A, emxArray_int32_T_cartesian_tr_T
  *b_jpvt);
static void cartesian_trajectory_pl_xzgetrf(int32_T m, int32_T n, const
  emxArray_real_T_cartesian_tra_T *A, int32_T lda,
  emxArray_real_T_cartesian_tra_T *b_A, emxArray_int32_T_cartesian_tr_T *ipiv,
  int32_T *info);
static void cartesian_trajectory_plan_xtrsm(int32_T m, int32_T n, const
  emxArray_real_T_cartesian_tra_T *A, int32_T lda, const
  emxArray_real_T_cartesian_tra_T *B, int32_T ldb,
  emxArray_real_T_cartesian_tra_T *b_B);
static void cartesian_traje_emxFree_int32_T(emxArray_int32_T_cartesian_tr_T
  **pEmxArray);
static void cartesian_trajectory_p_mldivide(const
  emxArray_real_T_cartesian_tra_T *A, const emxArray_real_T_cartesian_tra_T *B,
  emxArray_real_T_cartesian_tra_T *Y);
static void cartesian_tra_emxFree_boolean_T(emxArray_boolean_T_cartesian__T
  **pEmxArray);
static boolean_T DampedBFGSwGradientProjection_a(const
  h_robotics_core_internal_Damp_T *obj, const real_T Hg[6], const
  emxArray_real_T_cartesian_tra_T *alpha);
static void cartesian_trajectory_planne_inv(const
  emxArray_real_T_cartesian_tra_T *x, emxArray_real_T_cartesian_tra_T *y);
static void cartesian_trajectory_plann_diag(const
  emxArray_real_T_cartesian_tra_T *v, emxArray_real_T_cartesian_tra_T *d);
static void cartesian_trajectory_pl_sqrt_as(emxArray_real_T_cartesian_tra_T *x);
static boolean_T cartesian_trajectory_planne_any(const
  emxArray_boolean_T_cartesian__T *x);
static boolean_T cartesian_tr_isPositiveDefinite(const real_T B[36]);
static boolean_T DampedBFGSwGradientProjectio_as(const
  h_robotics_core_internal_Damp_T *obj, const real_T xNew[6]);
static void DampedBFGSwGradientProjection_s(h_robotics_core_internal_Damp_T *obj,
  real_T xSol[6], c_robotics_core_internal_NLPS_T *exitFlag, real_T *err, real_T
  *iter);
static void cartesian_trajectory_p_isfinite(const
  emxArray_real_T_cartesian_tra_T *x, emxArray_boolean_T_cartesian__T *b);
static void cartesi_genrand_uint32_vector_a(uint32_T mt[625], uint32_T u[2]);
static boolean_T cartesian_trajec_is_valid_state(const uint32_T mt[625]);
static real_T cartesian_trajectory_genrandu_a(uint32_T mt[625]);
static real_T cartesia_eml_rand_mt19937ar_ast(uint32_T state[625]);
static void cartesian_trajectory_plan_randn(const real_T varargin_1[2],
  emxArray_real_T_cartesian_tra_T *r);
static void cartesian__eml_rand_mt19937ar_a(const uint32_T state[625], uint32_T
  b_state[625], real_T *r);
static void cartesian_trajectory_pla_rand_a(real_T varargin_1,
  emxArray_real_T_cartesian_tra_T *r);
static void cartes_NLPSolverInterface_solve(h_robotics_core_internal_Damp_T *obj,
  const real_T seed[6], real_T xSol[6], real_T *solutionInfo_Iterations, real_T *
  solutionInfo_RRAttempts, real_T *solutionInfo_Error, real_T
  *solutionInfo_ExitFlag, char_T solutionInfo_Status_data[], int32_T
  solutionInfo_Status_size[2]);
static void cart_inverseKinematics_stepImpl(b_inverseKinematics_cartesian_T *obj,
  const real_T tform[16], const real_T weights[6], const real_T initialGuess[6],
  real_T QSol[6]);
static void c_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_cartesia_T
  *emxArray, int32_T oldNumel);
static void ca_rigidBodyJoint_get_JointAxis(const
  c_rigidBodyJoint_cartesian_tr_T *obj, real_T ax[3]);
static void RigidBodyTree_forwardKinematics(p_robotics_manip_internal_Rig_T *obj,
  const real_T qvec[6], emxArray_f_cell_wrap_cartesia_T *Ttree);
static void cartesian_t_emxFree_f_cell_wrap(emxArray_f_cell_wrap_cartesia_T
  **pEmxArray);
static void RigidBodyTree_geometricJacobian(p_robotics_manip_internal_Rig_T *obj,
  const real_T Q[6], emxArray_real_T_cartesian_tra_T *Jac);
static void rigidBodyJoint_get_JointAxis_a(const c_rigidBodyJoint_cartesian__a_T
  *obj, real_T ax[3]);
static void RigidBodyTree_forwardKinemati_a(p_robotics_manip_internal_R_a_T *obj,
  const real_T qvec[6], emxArray_f_cell_wrap_cartesia_T *Ttree);
static void matlabCodegenHandle_matla_astwh(ros_slros_internal_block_Subs_T *obj);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_GetP_T *obj);
static void matlabCodegenHandle_matlabC_ast(robotics_slmanip_internal__as_T *obj);
static void cartesian_tr_SystemCore_release(b_inverseKinematics_cartesian_T *obj);
static void cartesian__SystemCore_delete_as(b_inverseKinematics_cartesian_T *obj);
static void matlabCodegenHandle_matlabCo_as(b_inverseKinematics_cartesian_T *obj);
static void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_cartesian_as_T
  *pStruct);
static void emxFreeStruct_v_robotics_manip_(v_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_y_robotics_manip_(y_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_b_inverseKinemati(b_inverseKinematics_cartesian_T
  *pStruct);
static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal__as_T
  *pStruct);
static void emxFreeStruct_w_robotics_manip_(w_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_x_robotics_manip_(x_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_f_robotics_manip_(f_robotics_manip_internal_IKE_T
  *pStruct);
static void emxFreeStruct_h_robotics_core_i(h_robotics_core_internal_Damp_T
  *pStruct);
static void emxFreeStruct_c_rigidBodyJoint1(c_rigidBodyJoint_cartesian_tr_T
  *pStruct);
static void emxFreeStruct_o_robotics_manip_(o_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_p_robotics_manip_(p_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_robotics_slmani_a(robotics_slmanip_internal_blo_T
  *pStruct);
static void emxFreeStruct_n_robotics_manip_(n_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_c_rigidBodyJoint2(c_rigidBodyJoint_cartesian__a_T
  *pStruct);
static void emxFreeStruct_o_robotics_mani_a(o_robotics_manip_internal_R_a_T
  *pStruct);
static void emxFreeStruct_p_robotics_mani_a(p_robotics_manip_internal_R_a_T
  *pStruct);
static void emxFreeStruct_robotics_slman_as(robotics_slmanip_internal_b_a_T
  *pStruct);
static void emxFreeStruct_n_robotics_mani_a(n_robotics_manip_internal_R_a_T
  *pStruct);
static void matlabCodegenHandle_matlab_astw(ros_slros_internal_block_Publ_T *obj);
static void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_cartesian_as_T
  *pStruct);
static void emxInitStruct_v_robotics_manip_(v_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_y_robotics_manip_(y_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_b_inverseKinemati(b_inverseKinematics_cartesian_T
  *pStruct);
static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal__as_T
  *pStruct);
static void emxInitStruct_w_robotics_manip_(w_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_x_robotics_manip_(x_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_f_robotics_manip_(f_robotics_manip_internal_IKE_T
  *pStruct);
static void emxInitStruct_h_robotics_core_i(h_robotics_core_internal_Damp_T
  *pStruct);
static void cartesia_twister_state_vector_a(uint32_T mt[625]);
static void cartesian_tr_eml_rand_mt19937ar(uint32_T state[625]);
static v_robotics_manip_internal_Rig_T *RigidBody_RigidB_astwhqf2azttxa
  (v_robotics_manip_internal_Rig_T *obj);
static v_robotics_manip_internal_Rig_T *RigidBody_Rigid_astwhqf2azttxab
  (v_robotics_manip_internal_Rig_T *obj);
static v_robotics_manip_internal_Rig_T *c_RigidBody_Rigid_b
  (v_robotics_manip_internal_Rig_T *obj);
static v_robotics_manip_internal_Rig_T *c_RigidBody_Rigid_i
  (v_robotics_manip_internal_Rig_T *obj);
static v_robotics_manip_internal_Rig_T *c_RigidBody_Rigid_l
  (v_robotics_manip_internal_Rig_T *obj);
static v_robotics_manip_internal_Rig_T *c_RigidBody_Rigid_lh
  (v_robotics_manip_internal_Rig_T *obj);
static v_robotics_manip_internal_Rig_T *c_RigidBody_Rigid_k
  (v_robotics_manip_internal_Rig_T *obj);
static y_robotics_manip_internal_Rig_T *RigidBodyTree_RigidBodyTree_as
  (y_robotics_manip_internal_Rig_T *obj, v_robotics_manip_internal_Rig_T *iobj_0,
   v_robotics_manip_internal_Rig_T *iobj_1, v_robotics_manip_internal_Rig_T
   *iobj_2, v_robotics_manip_internal_Rig_T *iobj_3,
   v_robotics_manip_internal_Rig_T *iobj_4, v_robotics_manip_internal_Rig_T
   *iobj_5, v_robotics_manip_internal_Rig_T *iobj_6,
   v_robotics_manip_internal_Rig_T *iobj_7);
static void cartesian_trajectory_plann_rand(real_T r[5]);
static w_robotics_manip_internal_Rig_T *c_RigidBody_Rigid_h
  (w_robotics_manip_internal_Rig_T *obj, c_rigidBodyJoint_cartesian_as_T *iobj_0);
static w_robotics_manip_internal_Rig_T *c_RigidBody_Rigid_h2
  (w_robotics_manip_internal_Rig_T *obj, c_rigidBodyJoint_cartesian_as_T *iobj_0);
static w_robotics_manip_internal_Rig_T *c_RigidBody_Rigid_m
  (w_robotics_manip_internal_Rig_T *obj, c_rigidBodyJoint_cartesian_as_T *iobj_0);
static w_robotics_manip_internal_Rig_T *c_RigidBody_Rigid_o
  (w_robotics_manip_internal_Rig_T *obj, c_rigidBodyJoint_cartesian_as_T *iobj_0);
static w_robotics_manip_internal_Rig_T *c_RigidBody_Rigid_d
  (w_robotics_manip_internal_Rig_T *obj, c_rigidBodyJoint_cartesian_as_T *iobj_0);
static void ca_RigidBodyTree_clearAllBodies(x_robotics_manip_internal_Rig_T *obj,
  w_robotics_manip_internal_Rig_T *iobj_0, w_robotics_manip_internal_Rig_T
  *iobj_1, w_robotics_manip_internal_Rig_T *iobj_2,
  w_robotics_manip_internal_Rig_T *iobj_3, w_robotics_manip_internal_Rig_T
  *iobj_4, w_robotics_manip_internal_Rig_T *iobj_5,
  w_robotics_manip_internal_Rig_T *iobj_6, c_rigidBodyJoint_cartesian_as_T
  *iobj_7, c_rigidBodyJoint_cartesian_as_T *iobj_8,
  c_rigidBodyJoint_cartesian_as_T *iobj_9, c_rigidBodyJoint_cartesian_as_T
  *iobj_10, c_rigidBodyJoint_cartesian_as_T *iobj_11,
  c_rigidBodyJoint_cartesian_as_T *iobj_12, c_rigidBodyJoint_cartesian_as_T
  *iobj_13, c_rigidBodyJoint_cartesian_as_T *iobj_14,
  w_robotics_manip_internal_Rig_T *iobj_15);
static x_robotics_manip_internal_Rig_T *RigidBodyTree_RigidBodyTree_ast
  (x_robotics_manip_internal_Rig_T *obj, w_robotics_manip_internal_Rig_T *iobj_0,
   w_robotics_manip_internal_Rig_T *iobj_1, w_robotics_manip_internal_Rig_T
   *iobj_2, w_robotics_manip_internal_Rig_T *iobj_3,
   w_robotics_manip_internal_Rig_T *iobj_4, w_robotics_manip_internal_Rig_T
   *iobj_5, w_robotics_manip_internal_Rig_T *iobj_6,
   c_rigidBodyJoint_cartesian_as_T *iobj_7, c_rigidBodyJoint_cartesian_as_T
   *iobj_8, c_rigidBodyJoint_cartesian_as_T *iobj_9,
   c_rigidBodyJoint_cartesian_as_T *iobj_10, c_rigidBodyJoint_cartesian_as_T
   *iobj_11, c_rigidBodyJoint_cartesian_as_T *iobj_12,
   c_rigidBodyJoint_cartesian_as_T *iobj_13, c_rigidBodyJoint_cartesian_as_T
   *iobj_14, c_rigidBodyJoint_cartesian_as_T *iobj_15,
   w_robotics_manip_internal_Rig_T *iobj_16);
static c_rigidBodyJoint_cartesian_as_T *c_rigidBodyJoint_rigidBodyJoint
  (c_rigidBodyJoint_cartesian_as_T *obj, const emxArray_char_T_cartesian_tra_T
   *jname, const emxArray_char_T_cartesian_tra_T *jtype);
static w_robotics_manip_internal_Rig_T *cartesian_trajec_RigidBody_copy(const
  v_robotics_manip_internal_Rig_T *obj, c_rigidBodyJoint_cartesian_as_T *iobj_0,
  c_rigidBodyJoint_cartesian_as_T *iobj_1, w_robotics_manip_internal_Rig_T
  *iobj_2);
static void cartesian_RigidBodyTree_addBody(x_robotics_manip_internal_Rig_T *obj,
  v_robotics_manip_internal_Rig_T *bodyin, const emxArray_char_T_cartesian_tra_T
  *parentName, c_rigidBodyJoint_cartesian_as_T *iobj_0,
  c_rigidBodyJoint_cartesian_as_T *iobj_1, w_robotics_manip_internal_Rig_T
  *iobj_2);
static void inverseKinematics_set_RigidBody(b_inverseKinematics_cartesian_T *obj,
  y_robotics_manip_internal_Rig_T *rigidbodytree,
  w_robotics_manip_internal_Rig_T *iobj_0, w_robotics_manip_internal_Rig_T
  *iobj_1, w_robotics_manip_internal_Rig_T *iobj_2,
  w_robotics_manip_internal_Rig_T *iobj_3, w_robotics_manip_internal_Rig_T
  *iobj_4, w_robotics_manip_internal_Rig_T *iobj_5,
  w_robotics_manip_internal_Rig_T *iobj_6, w_robotics_manip_internal_Rig_T
  *iobj_7, w_robotics_manip_internal_Rig_T *iobj_8,
  w_robotics_manip_internal_Rig_T *iobj_9, w_robotics_manip_internal_Rig_T
  *iobj_10, w_robotics_manip_internal_Rig_T *iobj_11,
  w_robotics_manip_internal_Rig_T *iobj_12, w_robotics_manip_internal_Rig_T
  *iobj_13, w_robotics_manip_internal_Rig_T *iobj_14,
  c_rigidBodyJoint_cartesian_as_T *iobj_15, c_rigidBodyJoint_cartesian_as_T
  *iobj_16, c_rigidBodyJoint_cartesian_as_T *iobj_17,
  c_rigidBodyJoint_cartesian_as_T *iobj_18, c_rigidBodyJoint_cartesian_as_T
  *iobj_19, c_rigidBodyJoint_cartesian_as_T *iobj_20,
  c_rigidBodyJoint_cartesian_as_T *iobj_21, c_rigidBodyJoint_cartesian_as_T
  *iobj_22, c_rigidBodyJoint_cartesian_as_T *iobj_23,
  c_rigidBodyJoint_cartesian_as_T *iobj_24, c_rigidBodyJoint_cartesian_as_T
  *iobj_25, c_rigidBodyJoint_cartesian_as_T *iobj_26,
  c_rigidBodyJoint_cartesian_as_T *iobj_27, c_rigidBodyJoint_cartesian_as_T
  *iobj_28, c_rigidBodyJoint_cartesian_as_T *iobj_29,
  c_rigidBodyJoint_cartesian_as_T *iobj_30, c_rigidBodyJoint_cartesian_as_T
  *iobj_31, c_rigidBodyJoint_cartesian_as_T *iobj_32,
  c_rigidBodyJoint_cartesian_as_T *iobj_33, c_rigidBodyJoint_cartesian_as_T
  *iobj_34, c_rigidBodyJoint_cartesian_as_T *iobj_35,
  c_rigidBodyJoint_cartesian_as_T *iobj_36, c_rigidBodyJoint_cartesian_as_T
  *iobj_37, c_rigidBodyJoint_cartesian_as_T *iobj_38,
  c_rigidBodyJoint_cartesian_as_T *iobj_39, w_robotics_manip_internal_Rig_T
  *iobj_40, x_robotics_manip_internal_Rig_T *iobj_41);
static h_robotics_core_internal_Damp_T *DampedBFGSwGradientProjection_D
  (h_robotics_core_internal_Damp_T *obj);
static void emxInitStruct_c_rigidBodyJoint1(c_rigidBodyJoint_cartesian_tr_T
  *pStruct);
static void emxInitStruct_o_robotics_manip_(o_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_p_robotics_manip_(p_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_robotics_slmani_a(robotics_slmanip_internal_blo_T
  *pStruct);
static void emxInitStruct_n_robotics_manip_(n_robotics_manip_internal_Rig_T
  *pStruct);
static n_robotics_manip_internal_Rig_T *cartesian_t_RigidBody_RigidBody
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *cartesian_RigidBody_RigidBody_a
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *cartesia_RigidBody_RigidBody_as
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *cartesi_RigidBody_RigidBody_ast
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *cartes_RigidBody_RigidBody_astw
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *carte_RigidBody_RigidBody_astwh
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *cart_RigidBody_RigidBody_astwhq
  (n_robotics_manip_internal_Rig_T *obj);
static n_robotics_manip_internal_Rig_T *car_RigidBody_RigidBody_astwhqf
  (n_robotics_manip_internal_Rig_T *obj);
static o_robotics_manip_internal_Rig_T *ca_RigidBody_RigidBody_astwhqf2
  (o_robotics_manip_internal_Rig_T *obj);
static p_robotics_manip_internal_Rig_T *car_RigidBodyTree_RigidBodyTree
  (p_robotics_manip_internal_Rig_T *obj, n_robotics_manip_internal_Rig_T *iobj_0,
   n_robotics_manip_internal_Rig_T *iobj_1, n_robotics_manip_internal_Rig_T
   *iobj_2, n_robotics_manip_internal_Rig_T *iobj_3,
   n_robotics_manip_internal_Rig_T *iobj_4, n_robotics_manip_internal_Rig_T
   *iobj_5, n_robotics_manip_internal_Rig_T *iobj_6,
   n_robotics_manip_internal_Rig_T *iobj_7);
static void emxInitStruct_c_rigidBodyJoint2(c_rigidBodyJoint_cartesian__a_T
  *pStruct);
static void emxInitStruct_o_robotics_mani_a(o_robotics_manip_internal_R_a_T
  *pStruct);
static void emxInitStruct_p_robotics_mani_a(p_robotics_manip_internal_R_a_T
  *pStruct);
static void emxInitStruct_robotics_slman_as(robotics_slmanip_internal_b_a_T
  *pStruct);
static void emxInitStruct_n_robotics_mani_a(n_robotics_manip_internal_R_a_T
  *pStruct);
static n_robotics_manip_internal_R_a_T *c_RigidBody_RigidBody_astwhqf2a
  (n_robotics_manip_internal_R_a_T *obj);
static n_robotics_manip_internal_R_a_T *RigidBody_RigidBody_astwhqf2az
  (n_robotics_manip_internal_R_a_T *obj);
static n_robotics_manip_internal_R_a_T *RigidBody_RigidBody_astwhqf2azt
  (n_robotics_manip_internal_R_a_T *obj);
static n_robotics_manip_internal_R_a_T *RigidBody_RigidBod_astwhqf2aztt
  (n_robotics_manip_internal_R_a_T *obj);
static n_robotics_manip_internal_R_a_T *RigidBody_RigidBo_astwhqf2azttx
  (n_robotics_manip_internal_R_a_T *obj);
static p_robotics_manip_internal_R_a_T *c_RigidBodyTree_RigidBodyTree_a
  (p_robotics_manip_internal_R_a_T *obj, n_robotics_manip_internal_R_a_T *iobj_0,
   n_robotics_manip_internal_R_a_T *iobj_1, n_robotics_manip_internal_R_a_T
   *iobj_2, n_robotics_manip_internal_R_a_T *iobj_3,
   n_robotics_manip_internal_R_a_T *iobj_4, n_robotics_manip_internal_R_a_T
   *iobj_5, n_robotics_manip_internal_R_a_T *iobj_6,
   n_robotics_manip_internal_R_a_T *iobj_7);
int32_T div_s32_floor(int32_T numerator, int32_T denominator)
{
  int32_T quotient;
  uint32_T absNumerator;
  uint32_T absDenominator;
  uint32_T tempAbsQuotient;
  boolean_T quotientNeedsNegation;
  if (denominator == 0) {
    quotient = numerator >= 0 ? MAX_int32_T : MIN_int32_T;

    // Divide by zero handler
  } else {
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

    quotient = quotientNeedsNegation ? -static_cast<int32_T>(tempAbsQuotient) :
      static_cast<int32_T>(tempAbsQuotient);
  }

  return quotient;
}

// Function for MATLAB Function: '<Root>/Traslazione '
static real_T cartesian_trajectory_pla_norm_l(const real_T x[3])
{
  real_T y;
  real_T scale;
  real_T absxk;
  real_T t;
  scale = 3.3121686421112381E-170;
  absxk = fabs(x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
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

  return scale * sqrt(y);
}

static void cartesian_trajec_emxInit_real_T(emxArray_real_T_cartesian_tra_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_real_T_cartesian_tra_T *emxArray;
  *pEmxArray = (emxArray_real_T_cartesian_tra_T *)malloc(sizeof
    (emxArray_real_T_cartesian_tra_T));
  emxArray = *pEmxArray;
  emxArray->data = (real_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (cartesian_trajectory_planner_B.i_k = 0;
       cartesian_trajectory_planner_B.i_k < numDimensions;
       cartesian_trajectory_planner_B.i_k++) {
    emxArray->size[cartesian_trajectory_planner_B.i_k] = 0;
  }
}

static void cartes_emxEnsureCapacity_real_T(emxArray_real_T_cartesian_tra_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  cartesian_trajectory_planner_B.newNumel_i = 1;
  for (cartesian_trajectory_planner_B.i_e = 0;
       cartesian_trajectory_planner_B.i_e < emxArray->numDimensions;
       cartesian_trajectory_planner_B.i_e++) {
    cartesian_trajectory_planner_B.newNumel_i *= emxArray->
      size[cartesian_trajectory_planner_B.i_e];
  }

  if (cartesian_trajectory_planner_B.newNumel_i > emxArray->allocatedSize) {
    cartesian_trajectory_planner_B.i_e = emxArray->allocatedSize;
    if (cartesian_trajectory_planner_B.i_e < 16) {
      cartesian_trajectory_planner_B.i_e = 16;
    }

    while (cartesian_trajectory_planner_B.i_e <
           cartesian_trajectory_planner_B.newNumel_i) {
      if (cartesian_trajectory_planner_B.i_e > 1073741823) {
        cartesian_trajectory_planner_B.i_e = MAX_int32_T;
      } else {
        cartesian_trajectory_planner_B.i_e <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(cartesian_trajectory_planner_B.i_e),
                     sizeof(real_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(real_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (real_T *)newData;
    emxArray->allocatedSize = cartesian_trajectory_planner_B.i_e;
    emxArray->canFreeData = true;
  }
}

static void cartesian_trajec_emxInit_char_T(emxArray_char_T_cartesian_tra_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_char_T_cartesian_tra_T *emxArray;
  *pEmxArray = (emxArray_char_T_cartesian_tra_T *)malloc(sizeof
    (emxArray_char_T_cartesian_tra_T));
  emxArray = *pEmxArray;
  emxArray->data = (char_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (cartesian_trajectory_planner_B.i_j = 0;
       cartesian_trajectory_planner_B.i_j < numDimensions;
       cartesian_trajectory_planner_B.i_j++) {
    emxArray->size[cartesian_trajectory_planner_B.i_j] = 0;
  }
}

static void cartes_emxEnsureCapacity_char_T(emxArray_char_T_cartesian_tra_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  cartesian_trajectory_planner_B.newNumel = 1;
  for (cartesian_trajectory_planner_B.i_l = 0;
       cartesian_trajectory_planner_B.i_l < emxArray->numDimensions;
       cartesian_trajectory_planner_B.i_l++) {
    cartesian_trajectory_planner_B.newNumel *= emxArray->
      size[cartesian_trajectory_planner_B.i_l];
  }

  if (cartesian_trajectory_planner_B.newNumel > emxArray->allocatedSize) {
    cartesian_trajectory_planner_B.i_l = emxArray->allocatedSize;
    if (cartesian_trajectory_planner_B.i_l < 16) {
      cartesian_trajectory_planner_B.i_l = 16;
    }

    while (cartesian_trajectory_planner_B.i_l <
           cartesian_trajectory_planner_B.newNumel) {
      if (cartesian_trajectory_planner_B.i_l > 1073741823) {
        cartesian_trajectory_planner_B.i_l = MAX_int32_T;
      } else {
        cartesian_trajectory_planner_B.i_l <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(cartesian_trajectory_planner_B.i_l),
                     sizeof(char_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(char_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (char_T *)newData;
    emxArray->allocatedSize = cartesian_trajectory_planner_B.i_l;
    emxArray->canFreeData = true;
  }
}

static void cartesian_trajec_emxFree_real_T(emxArray_real_T_cartesian_tra_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T_cartesian_tra_T *)NULL) {
    if (((*pEmxArray)->data != (real_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_real_T_cartesian_tra_T *)NULL;
  }
}

static void cartesian_trajec_emxFree_char_T(emxArray_char_T_cartesian_tra_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_char_T_cartesian_tra_T *)NULL) {
    if (((*pEmxArray)->data != (char_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_char_T_cartesian_tra_T *)NULL;
  }
}

static void car_inverseKinematics_setupImpl(b_inverseKinematics_cartesian_T *obj,
  f_robotics_manip_internal_IKE_T *iobj_0)
{
  real_T n;
  emxArray_real_T_cartesian_tra_T *A;
  emxArray_real_T_cartesian_tra_T *b;
  real_T m;
  c_rigidBodyJoint_cartesian_as_T *joint;
  real_T pnum;
  int32_T d;
  int32_T b_i;
  emxArray_real_T_cartesian_tra_T *e;
  int32_T j;
  int32_T p;
  emxArray_real_T_cartesian_tra_T *s;
  x_robotics_manip_internal_Rig_T *obj_0;
  w_robotics_manip_internal_Rig_T *body;
  int32_T c;
  boolean_T b_bool;
  emxArray_char_T_cartesian_tra_T *a;
  int32_T b_kstr;
  char_T b_0[5];
  int8_T b_I[16];
  int32_T m_0;
  real_T t;
  real_T pnum_0;
  int32_T loop_ub;
  real_T tmp;
  real_T w;
  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  int32_T exitg1;
  cartesian_trajec_emxInit_real_T(&A, 2);
  n = obj->RigidBodyTreeInternal->PositionNumber;
  b_kstr = A->size[0] * A->size[1];
  c = static_cast<int32_T>(n);
  A->size[0] = c;
  loop_ub = static_cast<int32_T>(2.0 * n);
  A->size[1] = loop_ub;
  cartes_emxEnsureCapacity_real_T(A, b_kstr);
  m_0 = loop_ub * c - 1;
  for (b_kstr = 0; b_kstr <= m_0; b_kstr++) {
    A->data[b_kstr] = 0.0;
  }

  cartesian_trajec_emxInit_real_T(&b, 1);
  b_kstr = b->size[0];
  b->size[0] = loop_ub;
  cartes_emxEnsureCapacity_real_T(b, b_kstr);
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    b->data[b_kstr] = 0.0;
  }

  n = 1.0;
  m = 1.0;
  pnum = obj->RigidBodyTreeInternal->NumBodies;
  d = static_cast<int32_T>(pnum) - 1;
  cartesian_trajec_emxInit_real_T(&e, 2);
  cartesian_trajec_emxInit_real_T(&s, 2);
  cartesian_trajec_emxInit_char_T(&a, 2);
  if (0 <= d) {
    for (b_kstr = 0; b_kstr < 5; b_kstr++) {
      b_0[b_kstr] = tmp_0[b_kstr];
    }
  }

  for (b_i = 0; b_i <= d; b_i++) {
    body = obj->RigidBodyTreeInternal->Bodies[b_i];
    joint = body->JointInternal;
    pnum = joint->PositionNumber;
    b_kstr = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = joint->Type->size[1];
    cartes_emxEnsureCapacity_char_T(a, b_kstr);
    loop_ub = joint->Type->size[0] * joint->Type->size[1] - 1;
    for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
      a->data[b_kstr] = joint->Type->data[b_kstr];
    }

    b_bool = false;
    if (a->size[1] == 5) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 5) {
          loop_ub = b_kstr - 1;
          if (a->data[loop_ub] != b_0[loop_ub]) {
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

    if (!b_bool) {
      tmp = (n + pnum) - 1.0;
      if (n > tmp) {
        j = 0;
      } else {
        j = static_cast<int32_T>(n) - 1;
      }

      w = m + pnum;
      if (m > w - 1.0) {
        p = 0;
      } else {
        p = static_cast<int32_T>(m) - 1;
      }

      if (pnum < 0.0) {
        t = 0.0;
        pnum_0 = 0.0;
      } else {
        t = pnum;
        pnum_0 = pnum;
      }

      m_0 = static_cast<int32_T>(pnum_0) - 1;
      b_kstr = s->size[0] * s->size[1];
      c = static_cast<int32_T>(t);
      s->size[0] = c;
      s->size[1] = c;
      cartes_emxEnsureCapacity_real_T(s, b_kstr);
      loop_ub = c * c - 1;
      for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
        s->data[b_kstr] = 0.0;
      }

      if (c > 0) {
        for (b_kstr = 0; b_kstr <= m_0; b_kstr++) {
          s->data[b_kstr + s->size[0] * b_kstr] = 1.0;
        }
      }

      loop_ub = s->size[1];
      for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
        m_0 = s->size[0];
        for (c = 0; c < m_0; c++) {
          A->data[(j + c) + A->size[0] * (p + b_kstr)] = s->data[s->size[0] *
            b_kstr + c];
        }
      }

      if (n > tmp) {
        j = 0;
      } else {
        j = static_cast<int32_T>(n) - 1;
      }

      tmp = 2.0 * pnum + m;
      if (w > tmp - 1.0) {
        p = 0;
      } else {
        p = static_cast<int32_T>(w) - 1;
      }

      if (pnum < 0.0) {
        t = 0.0;
        pnum_0 = 0.0;
      } else {
        t = pnum;
        pnum_0 = pnum;
      }

      m_0 = static_cast<int32_T>(pnum_0) - 1;
      b_kstr = s->size[0] * s->size[1];
      c = static_cast<int32_T>(t);
      s->size[0] = c;
      s->size[1] = c;
      cartes_emxEnsureCapacity_real_T(s, b_kstr);
      loop_ub = c * c - 1;
      for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
        s->data[b_kstr] = 0.0;
      }

      if (c > 0) {
        for (b_kstr = 0; b_kstr <= m_0; b_kstr++) {
          s->data[b_kstr + s->size[0] * b_kstr] = 1.0;
        }
      }

      loop_ub = s->size[1];
      for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
        m_0 = s->size[0];
        for (c = 0; c < m_0; c++) {
          A->data[(j + c) + A->size[0] * (p + b_kstr)] = -s->data[s->size[0] *
            b_kstr + c];
        }
      }

      b_kstr = e->size[0] * e->size[1];
      e->size[0] = joint->PositionLimitsInternal->size[0];
      e->size[1] = 2;
      cartes_emxEnsureCapacity_real_T(e, b_kstr);
      loop_ub = joint->PositionLimitsInternal->size[0] *
        joint->PositionLimitsInternal->size[1] - 1;
      for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
        e->data[b_kstr] = joint->PositionLimitsInternal->data[b_kstr];
      }

      b->data[static_cast<int32_T>(m) - 1] = e->data[1];
      b_kstr = e->size[0] * e->size[1];
      e->size[0] = joint->PositionLimitsInternal->size[0];
      e->size[1] = 2;
      cartes_emxEnsureCapacity_real_T(e, b_kstr);
      loop_ub = joint->PositionLimitsInternal->size[0] *
        joint->PositionLimitsInternal->size[1] - 1;
      for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
        e->data[b_kstr] = joint->PositionLimitsInternal->data[b_kstr];
      }

      b->data[static_cast<int32_T>(m + 1.0) - 1] = -e->data[0];
      m = tmp;
    }

    n += pnum;
  }

  cartesian_trajec_emxFree_real_T(&s);
  b_kstr = A->size[0] * A->size[1];
  c = obj->Solver->ConstraintMatrix->size[0] * obj->Solver->
    ConstraintMatrix->size[1];
  obj->Solver->ConstraintMatrix->size[0] = A->size[0];
  obj->Solver->ConstraintMatrix->size[1] = A->size[1];
  cartes_emxEnsureCapacity_real_T(obj->Solver->ConstraintMatrix, c);
  loop_ub = b_kstr - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    obj->Solver->ConstraintMatrix->data[b_kstr] = A->data[b_kstr];
  }

  cartesian_trajec_emxFree_real_T(&A);
  b_kstr = obj->Solver->ConstraintBound->size[0];
  obj->Solver->ConstraintBound->size[0] = b->size[0];
  cartes_emxEnsureCapacity_real_T(obj->Solver->ConstraintBound, b_kstr);
  loop_ub = b->size[0];
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    obj->Solver->ConstraintBound->data[b_kstr] = b->data[b_kstr];
  }

  obj_0 = obj->RigidBodyTreeInternal;
  b_kstr = e->size[0] * e->size[1];
  e->size[0] = static_cast<int32_T>(obj_0->PositionNumber);
  e->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(e, b_kstr);
  loop_ub = (static_cast<int32_T>(obj_0->PositionNumber) << 1) - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    e->data[b_kstr] = 0.0;
  }

  n = 1.0;
  m = obj_0->NumBodies;
  c = static_cast<int32_T>(m) - 1;
  if (0 <= c) {
    for (b_kstr = 0; b_kstr < 5; b_kstr++) {
      b_0[b_kstr] = tmp_0[b_kstr];
    }
  }

  for (b_i = 0; b_i <= c; b_i++) {
    body = obj_0->Bodies[b_i];
    b_kstr = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = body->JointInternal->Type->size[1];
    cartes_emxEnsureCapacity_char_T(a, b_kstr);
    loop_ub = body->JointInternal->Type->size[0] * body->JointInternal->
      Type->size[1] - 1;
    for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
      a->data[b_kstr] = body->JointInternal->Type->data[b_kstr];
    }

    b_bool = false;
    if (a->size[1] == 5) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 5) {
          loop_ub = b_kstr - 1;
          if (a->data[loop_ub] != b_0[loop_ub]) {
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

    if (!b_bool) {
      pnum = body->JointInternal->PositionNumber;
      tmp = n + pnum;
      if (n > tmp - 1.0) {
        d = 0;
      } else {
        d = static_cast<int32_T>(n) - 1;
      }

      joint = body->JointInternal;
      loop_ub = joint->PositionLimitsInternal->size[0];
      for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
        e->data[d + b_kstr] = joint->PositionLimitsInternal->data[b_kstr];
      }

      loop_ub = joint->PositionLimitsInternal->size[0];
      for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
        e->data[(d + b_kstr) + e->size[0]] = joint->PositionLimitsInternal->
          data[b_kstr + joint->PositionLimitsInternal->size[0]];
      }

      n = tmp;
    }
  }

  cartesian_trajec_emxFree_char_T(&a);
  b_kstr = obj->Limits->size[0] * obj->Limits->size[1];
  obj->Limits->size[0] = e->size[0];
  obj->Limits->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->Limits, b_kstr);
  loop_ub = e->size[0] * e->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    obj->Limits->data[b_kstr] = e->data[b_kstr];
  }

  cartesian_trajec_emxFree_real_T(&e);
  obj->Solver->ExtraArgs = iobj_0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->Solver->ExtraArgs->WeightMatrix[b_kstr] = 0.0;
  }

  obj->Solver->ExtraArgs->Robot = obj->RigidBodyTreeInternal;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    b_I[b_kstr] = 0;
  }

  b_I[0] = 1;
  b_I[5] = 1;
  b_I[10] = 1;
  b_I[15] = 1;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->Solver->ExtraArgs->Tform[b_kstr] = b_I[b_kstr];
  }

  obj->Solver->ExtraArgs->BodyName->size[0] = 1;
  obj->Solver->ExtraArgs->BodyName->size[1] = 0;
  b_kstr = obj->Solver->ExtraArgs->ErrTemp->size[0];
  obj->Solver->ExtraArgs->ErrTemp->size[0] = 6;
  cartes_emxEnsureCapacity_real_T(obj->Solver->ExtraArgs->ErrTemp, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->Solver->ExtraArgs->ErrTemp->data[b_kstr] = 0.0;
  }

  obj->Solver->ExtraArgs->CostTemp = 0.0;
  b_kstr = b->size[0];
  b->size[0] = static_cast<int32_T>(obj->RigidBodyTreeInternal->PositionNumber);
  cartes_emxEnsureCapacity_real_T(b, b_kstr);
  loop_ub = static_cast<int32_T>(obj->RigidBodyTreeInternal->PositionNumber);
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    b->data[b_kstr] = 0.0;
  }

  b_kstr = obj->Solver->ExtraArgs->GradTemp->size[0];
  obj->Solver->ExtraArgs->GradTemp->size[0] = b->size[0];
  cartes_emxEnsureCapacity_real_T(obj->Solver->ExtraArgs->GradTemp, b_kstr);
  loop_ub = b->size[0];
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    obj->Solver->ExtraArgs->GradTemp->data[b_kstr] = b->data[b_kstr];
  }

  cartesian_trajec_emxFree_real_T(&b);
}

static void cartesian_t_emxInit_f_cell_wrap(emxArray_f_cell_wrap_cartesia_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_f_cell_wrap_cartesia_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_f_cell_wrap_cartesia_T *)malloc(sizeof
    (emxArray_f_cell_wrap_cartesia_T));
  emxArray = *pEmxArray;
  emxArray->data = (f_cell_wrap_cartesian_traject_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void c_inverseKinematics_setPoseGoal(b_inverseKinematics_cartesian_T *obj,
  const real_T tform[16], const real_T weights[6])
{
  real_T weightMatrix[36];
  f_robotics_manip_internal_IKE_T *args;
  int32_T b_j;
  static const char_T tmp[11] = { 'e', 'd', 'o', '_', 'l', 'i', 'n', 'k', '_',
    'e', 'e' };

  memset(&weightMatrix[0], 0, 36U * sizeof(real_T));
  for (b_j = 0; b_j < 6; b_j++) {
    weightMatrix[b_j + 6 * b_j] = weights[b_j];
  }

  args = obj->Solver->ExtraArgs;
  for (b_j = 0; b_j < 36; b_j++) {
    args->WeightMatrix[b_j] = weightMatrix[b_j];
  }

  b_j = args->BodyName->size[0] * args->BodyName->size[1];
  args->BodyName->size[0] = 1;
  args->BodyName->size[1] = 11;
  cartes_emxEnsureCapacity_char_T(args->BodyName, b_j);
  for (b_j = 0; b_j < 11; b_j++) {
    args->BodyName->data[b_j] = tmp[b_j];
  }

  for (b_j = 0; b_j < 16; b_j++) {
    args->Tform[b_j] = tform[b_j];
  }
}

static void RigidBodyTree_validateConfigu_a(x_robotics_manip_internal_Rig_T *obj,
  real_T Q[6])
{
  emxArray_real_T_cartesian_tra_T *limits;
  boolean_T ubOK[6];
  boolean_T lbOK[6];
  real_T k;
  w_robotics_manip_internal_Rig_T *body;
  real_T pnum;
  int32_T c;
  int32_T f;
  int32_T ii_data[6];
  boolean_T b_bool;
  emxArray_char_T_cartesian_tra_T *a;
  int32_T b_kstr;
  char_T b[5];
  c_rigidBodyJoint_cartesian_as_T *obj_0;
  int32_T idx;
  int32_T loop_ub;
  emxArray_real_T_cartesian_tra_T *limits_0;
  emxArray_real_T_cartesian_tra_T *limits_1;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  boolean_T guard1 = false;
  int32_T exitg1;
  boolean_T exitg2;
  cartesian_trajec_emxInit_real_T(&limits, 2);
  b_kstr = limits->size[0] * limits->size[1];
  limits->size[0] = static_cast<int32_T>(obj->PositionNumber);
  limits->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(limits, b_kstr);
  loop_ub = (static_cast<int32_T>(obj->PositionNumber) << 1) - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    limits->data[b_kstr] = 0.0;
  }

  k = 1.0;
  pnum = obj->NumBodies;
  c = static_cast<int32_T>(pnum) - 1;
  cartesian_trajec_emxInit_char_T(&a, 2);
  if (0 <= c) {
    for (b_kstr = 0; b_kstr < 5; b_kstr++) {
      b[b_kstr] = tmp[b_kstr];
    }
  }

  for (idx = 0; idx <= c; idx++) {
    body = obj->Bodies[idx];
    b_kstr = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = body->JointInternal->Type->size[1];
    cartes_emxEnsureCapacity_char_T(a, b_kstr);
    loop_ub = body->JointInternal->Type->size[0] * body->JointInternal->
      Type->size[1] - 1;
    for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
      a->data[b_kstr] = body->JointInternal->Type->data[b_kstr];
    }

    b_bool = false;
    if (a->size[1] == 5) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 5) {
          loop_ub = b_kstr - 1;
          if (a->data[loop_ub] != b[loop_ub]) {
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

    if (!b_bool) {
      pnum = body->JointInternal->PositionNumber;
      pnum += k;
      if (k > pnum - 1.0) {
        f = 0;
      } else {
        f = static_cast<int32_T>(k) - 1;
      }

      obj_0 = body->JointInternal;
      loop_ub = obj_0->PositionLimitsInternal->size[0];
      for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
        limits->data[f + b_kstr] = obj_0->PositionLimitsInternal->data[b_kstr];
      }

      loop_ub = obj_0->PositionLimitsInternal->size[0];
      for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
        limits->data[(f + b_kstr) + limits->size[0]] =
          obj_0->PositionLimitsInternal->data[b_kstr +
          obj_0->PositionLimitsInternal->size[0]];
      }

      k = pnum;
    }
  }

  cartesian_trajec_emxFree_char_T(&a);
  cartesian_trajec_emxInit_real_T(&limits_0, 1);
  loop_ub = limits->size[0];
  b_kstr = limits_0->size[0];
  limits_0->size[0] = loop_ub;
  cartes_emxEnsureCapacity_real_T(limits_0, b_kstr);
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    limits_0->data[b_kstr] = limits->data[b_kstr + limits->size[0]] +
      4.4408920985006262E-16;
  }

  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    ubOK[b_kstr] = (Q[b_kstr] <= limits_0->data[b_kstr]);
  }

  cartesian_trajec_emxFree_real_T(&limits_0);
  cartesian_trajec_emxInit_real_T(&limits_1, 1);
  loop_ub = limits->size[0];
  b_kstr = limits_1->size[0];
  limits_1->size[0] = loop_ub;
  cartes_emxEnsureCapacity_real_T(limits_1, b_kstr);
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    limits_1->data[b_kstr] = limits->data[b_kstr] - 4.4408920985006262E-16;
  }

  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    lbOK[b_kstr] = (Q[b_kstr] >= limits_1->data[b_kstr]);
  }

  cartesian_trajec_emxFree_real_T(&limits_1);
  b_bool = true;
  idx = 0;
  exitg2 = false;
  while ((!exitg2) && (idx < 6)) {
    if (!ubOK[idx]) {
      b_bool = false;
      exitg2 = true;
    } else {
      idx++;
    }
  }

  guard1 = false;
  if (b_bool) {
    b_bool = true;
    idx = 0;
    exitg2 = false;
    while ((!exitg2) && (idx < 6)) {
      if (!lbOK[idx]) {
        b_bool = false;
        exitg2 = true;
      } else {
        idx++;
      }
    }

    if (b_bool) {
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    idx = 0;
    b_kstr = 1;
    exitg2 = false;
    while ((!exitg2) && (b_kstr - 1 < 6)) {
      if (!ubOK[b_kstr - 1]) {
        idx++;
        ii_data[idx - 1] = b_kstr;
        if (idx >= 6) {
          exitg2 = true;
        } else {
          b_kstr++;
        }
      } else {
        b_kstr++;
      }
    }

    if (1 > idx) {
      idx = 0;
    }

    for (b_kstr = 0; b_kstr < idx; b_kstr++) {
      Q[ii_data[b_kstr] - 1] = limits->data[(ii_data[b_kstr] + limits->size[0])
        - 1];
    }

    idx = 0;
    b_kstr = 1;
    exitg2 = false;
    while ((!exitg2) && (b_kstr - 1 < 6)) {
      if (!lbOK[b_kstr - 1]) {
        idx++;
        ii_data[idx - 1] = b_kstr;
        if (idx >= 6) {
          exitg2 = true;
        } else {
          b_kstr++;
        }
      } else {
        b_kstr++;
      }
    }

    if (1 > idx) {
      idx = 0;
    }

    for (b_kstr = 0; b_kstr < idx; b_kstr++) {
      Q[ii_data[b_kstr] - 1] = limits->data[ii_data[b_kstr] - 1];
    }
  }

  cartesian_trajec_emxFree_real_T(&limits);
}

static boolean_T cartesian_trajectory_pla_strcmp(const
  emxArray_char_T_cartesian_tra_T *a, const emxArray_char_T_cartesian_tra_T *b)
{
  boolean_T b_bool;
  int32_T kstr;
  int32_T b_kstr;
  boolean_T d;
  int32_T exitg1;
  b_bool = false;
  d = (a->size[1] == 0);
  if (d && (b->size[1] == 0)) {
    b_bool = true;
  } else if (a->size[1] != b->size[1]) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 <= b->size[1] - 1) {
        kstr = b_kstr - 1;
        if (a->data[kstr] != b->data[kstr]) {
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

  return b_bool;
}

static void rigidBodyJoint_get_JointAxis_as(const
  c_rigidBodyJoint_cartesian_as_T *obj, real_T ax[3])
{
  emxArray_char_T_cartesian_tra_T *a;
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  boolean_T guard1 = false;
  int32_T exitg1;
  cartesian_trajec_emxInit_char_T(&a, 2);
  cartesian_trajectory_planner_B.b_kstr_m = a->size[0] * a->size[1];
  a->size[0] = 1;
  a->size[1] = obj->Type->size[1];
  cartes_emxEnsureCapacity_char_T(a, cartesian_trajectory_planner_B.b_kstr_m);
  cartesian_trajectory_planner_B.loop_ub_g = obj->Type->size[0] * obj->
    Type->size[1] - 1;
  for (cartesian_trajectory_planner_B.b_kstr_m = 0;
       cartesian_trajectory_planner_B.b_kstr_m <=
       cartesian_trajectory_planner_B.loop_ub_g;
       cartesian_trajectory_planner_B.b_kstr_m++) {
    a->data[cartesian_trajectory_planner_B.b_kstr_m] = obj->Type->
      data[cartesian_trajectory_planner_B.b_kstr_m];
  }

  for (cartesian_trajectory_planner_B.b_kstr_m = 0;
       cartesian_trajectory_planner_B.b_kstr_m < 8;
       cartesian_trajectory_planner_B.b_kstr_m++) {
    cartesian_trajectory_planner_B.b_j[cartesian_trajectory_planner_B.b_kstr_m] =
      tmp[cartesian_trajectory_planner_B.b_kstr_m];
  }

  cartesian_trajectory_planner_B.b_bool_c = false;
  if (a->size[1] == 8) {
    cartesian_trajectory_planner_B.b_kstr_m = 1;
    do {
      exitg1 = 0;
      if (cartesian_trajectory_planner_B.b_kstr_m - 1 < 8) {
        cartesian_trajectory_planner_B.loop_ub_g =
          cartesian_trajectory_planner_B.b_kstr_m - 1;
        if (a->data[cartesian_trajectory_planner_B.loop_ub_g] !=
            cartesian_trajectory_planner_B.b_j[cartesian_trajectory_planner_B.loop_ub_g])
        {
          exitg1 = 1;
        } else {
          cartesian_trajectory_planner_B.b_kstr_m++;
        }
      } else {
        cartesian_trajectory_planner_B.b_bool_c = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (cartesian_trajectory_planner_B.b_bool_c) {
    guard1 = true;
  } else {
    cartesian_trajectory_planner_B.b_kstr_m = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = obj->Type->size[1];
    cartes_emxEnsureCapacity_char_T(a, cartesian_trajectory_planner_B.b_kstr_m);
    cartesian_trajectory_planner_B.loop_ub_g = obj->Type->size[0] * obj->
      Type->size[1] - 1;
    for (cartesian_trajectory_planner_B.b_kstr_m = 0;
         cartesian_trajectory_planner_B.b_kstr_m <=
         cartesian_trajectory_planner_B.loop_ub_g;
         cartesian_trajectory_planner_B.b_kstr_m++) {
      a->data[cartesian_trajectory_planner_B.b_kstr_m] = obj->Type->
        data[cartesian_trajectory_planner_B.b_kstr_m];
    }

    for (cartesian_trajectory_planner_B.b_kstr_m = 0;
         cartesian_trajectory_planner_B.b_kstr_m < 9;
         cartesian_trajectory_planner_B.b_kstr_m++) {
      cartesian_trajectory_planner_B.b_n[cartesian_trajectory_planner_B.b_kstr_m]
        = tmp_0[cartesian_trajectory_planner_B.b_kstr_m];
    }

    cartesian_trajectory_planner_B.b_bool_c = false;
    if (a->size[1] == 9) {
      cartesian_trajectory_planner_B.b_kstr_m = 1;
      do {
        exitg1 = 0;
        if (cartesian_trajectory_planner_B.b_kstr_m - 1 < 9) {
          cartesian_trajectory_planner_B.loop_ub_g =
            cartesian_trajectory_planner_B.b_kstr_m - 1;
          if (a->data[cartesian_trajectory_planner_B.loop_ub_g] !=
              cartesian_trajectory_planner_B.b_n[cartesian_trajectory_planner_B.loop_ub_g])
          {
            exitg1 = 1;
          } else {
            cartesian_trajectory_planner_B.b_kstr_m++;
          }
        } else {
          cartesian_trajectory_planner_B.b_bool_c = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (cartesian_trajectory_planner_B.b_bool_c) {
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

  cartesian_trajec_emxFree_char_T(&a);
}

static void cartesian_trajectory_planne_cat(real_T varargin_1, real_T varargin_2,
  real_T varargin_3, real_T varargin_4, real_T varargin_5, real_T varargin_6,
  real_T varargin_7, real_T varargin_8, real_T varargin_9, real_T y[9])
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
  c_rigidBodyJoint_cartesian_as_T *obj, const real_T q_data[], const int32_T
  *q_size, real_T T[16])
{
  emxArray_char_T_cartesian_tra_T *switch_expression;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  cartesian_trajectory_planner_B.b_kstr_i = switch_expression->size[0] *
    switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression,
    cartesian_trajectory_planner_B.b_kstr_i);
  cartesian_trajectory_planner_B.loop_ub_h5 = obj->Type->size[0] * obj->
    Type->size[1] - 1;
  for (cartesian_trajectory_planner_B.b_kstr_i = 0;
       cartesian_trajectory_planner_B.b_kstr_i <=
       cartesian_trajectory_planner_B.loop_ub_h5;
       cartesian_trajectory_planner_B.b_kstr_i++) {
    switch_expression->data[cartesian_trajectory_planner_B.b_kstr_i] = obj->
      Type->data[cartesian_trajectory_planner_B.b_kstr_i];
  }

  for (cartesian_trajectory_planner_B.b_kstr_i = 0;
       cartesian_trajectory_planner_B.b_kstr_i < 5;
       cartesian_trajectory_planner_B.b_kstr_i++) {
    cartesian_trajectory_planner_B.b_e[cartesian_trajectory_planner_B.b_kstr_i] =
      tmp[cartesian_trajectory_planner_B.b_kstr_i];
  }

  cartesian_trajectory_planner_B.b_bool_k = false;
  if (switch_expression->size[1] == 5) {
    cartesian_trajectory_planner_B.b_kstr_i = 1;
    do {
      exitg1 = 0;
      if (cartesian_trajectory_planner_B.b_kstr_i - 1 < 5) {
        cartesian_trajectory_planner_B.loop_ub_h5 =
          cartesian_trajectory_planner_B.b_kstr_i - 1;
        if (switch_expression->data[cartesian_trajectory_planner_B.loop_ub_h5]
            !=
            cartesian_trajectory_planner_B.b_e[cartesian_trajectory_planner_B.loop_ub_h5])
        {
          exitg1 = 1;
        } else {
          cartesian_trajectory_planner_B.b_kstr_i++;
        }
      } else {
        cartesian_trajectory_planner_B.b_bool_k = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (cartesian_trajectory_planner_B.b_bool_k) {
    cartesian_trajectory_planner_B.b_kstr_i = 0;
  } else {
    for (cartesian_trajectory_planner_B.b_kstr_i = 0;
         cartesian_trajectory_planner_B.b_kstr_i < 8;
         cartesian_trajectory_planner_B.b_kstr_i++) {
      cartesian_trajectory_planner_B.b_m[cartesian_trajectory_planner_B.b_kstr_i]
        = tmp_0[cartesian_trajectory_planner_B.b_kstr_i];
    }

    cartesian_trajectory_planner_B.b_bool_k = false;
    if (switch_expression->size[1] == 8) {
      cartesian_trajectory_planner_B.b_kstr_i = 1;
      do {
        exitg1 = 0;
        if (cartesian_trajectory_planner_B.b_kstr_i - 1 < 8) {
          cartesian_trajectory_planner_B.loop_ub_h5 =
            cartesian_trajectory_planner_B.b_kstr_i - 1;
          if (switch_expression->data[cartesian_trajectory_planner_B.loop_ub_h5]
              !=
              cartesian_trajectory_planner_B.b_m[cartesian_trajectory_planner_B.loop_ub_h5])
          {
            exitg1 = 1;
          } else {
            cartesian_trajectory_planner_B.b_kstr_i++;
          }
        } else {
          cartesian_trajectory_planner_B.b_bool_k = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (cartesian_trajectory_planner_B.b_bool_k) {
      cartesian_trajectory_planner_B.b_kstr_i = 1;
    } else {
      cartesian_trajectory_planner_B.b_kstr_i = -1;
    }
  }

  cartesian_trajec_emxFree_char_T(&switch_expression);
  switch (cartesian_trajectory_planner_B.b_kstr_i) {
   case 0:
    memset(&cartesian_trajectory_planner_B.TJ[0], 0, sizeof(real_T) << 4U);
    cartesian_trajectory_planner_B.TJ[0] = 1.0;
    cartesian_trajectory_planner_B.TJ[5] = 1.0;
    cartesian_trajectory_planner_B.TJ[10] = 1.0;
    cartesian_trajectory_planner_B.TJ[15] = 1.0;
    break;

   case 1:
    rigidBodyJoint_get_JointAxis_as(obj, cartesian_trajectory_planner_B.v_jg);
    cartesian_trajectory_planner_B.loop_ub_h5 = (*q_size != 0) - 1;
    cartesian_trajectory_planner_B.result_data[0] =
      cartesian_trajectory_planner_B.v_jg[0];
    cartesian_trajectory_planner_B.result_data[1] =
      cartesian_trajectory_planner_B.v_jg[1];
    cartesian_trajectory_planner_B.result_data[2] =
      cartesian_trajectory_planner_B.v_jg[2];
    for (cartesian_trajectory_planner_B.b_kstr_i = 0;
         cartesian_trajectory_planner_B.b_kstr_i <=
         cartesian_trajectory_planner_B.loop_ub_h5;
         cartesian_trajectory_planner_B.b_kstr_i++) {
      cartesian_trajectory_planner_B.result_data[3] = q_data[0];
    }

    cartesian_trajectory_planner_B.cth = 1.0 / sqrt
      ((cartesian_trajectory_planner_B.result_data[0] *
        cartesian_trajectory_planner_B.result_data[0] +
        cartesian_trajectory_planner_B.result_data[1] *
        cartesian_trajectory_planner_B.result_data[1]) +
       cartesian_trajectory_planner_B.result_data[2] *
       cartesian_trajectory_planner_B.result_data[2]);
    cartesian_trajectory_planner_B.v_jg[0] =
      cartesian_trajectory_planner_B.result_data[0] *
      cartesian_trajectory_planner_B.cth;
    cartesian_trajectory_planner_B.v_jg[1] =
      cartesian_trajectory_planner_B.result_data[1] *
      cartesian_trajectory_planner_B.cth;
    cartesian_trajectory_planner_B.v_jg[2] =
      cartesian_trajectory_planner_B.result_data[2] *
      cartesian_trajectory_planner_B.cth;
    cartesian_trajectory_planner_B.cth = cos
      (cartesian_trajectory_planner_B.result_data[3]);
    cartesian_trajectory_planner_B.sth = sin
      (cartesian_trajectory_planner_B.result_data[3]);
    cartesian_trajectory_planner_B.tempR_tmp =
      cartesian_trajectory_planner_B.v_jg[1] *
      cartesian_trajectory_planner_B.v_jg[0] * (1.0 -
      cartesian_trajectory_planner_B.cth);
    cartesian_trajectory_planner_B.tempR_tmp_n =
      cartesian_trajectory_planner_B.v_jg[2] *
      cartesian_trajectory_planner_B.sth;
    cartesian_trajectory_planner_B.tempR_tmp_l =
      cartesian_trajectory_planner_B.v_jg[2] *
      cartesian_trajectory_planner_B.v_jg[0] * (1.0 -
      cartesian_trajectory_planner_B.cth);
    cartesian_trajectory_planner_B.tempR_tmp_c =
      cartesian_trajectory_planner_B.v_jg[1] *
      cartesian_trajectory_planner_B.sth;
    cartesian_trajectory_planner_B.tempR_tmp_nc =
      cartesian_trajectory_planner_B.v_jg[2] *
      cartesian_trajectory_planner_B.v_jg[1] * (1.0 -
      cartesian_trajectory_planner_B.cth);
    cartesian_trajectory_planner_B.sth *= cartesian_trajectory_planner_B.v_jg[0];
    cartesian_trajectory_planne_cat(cartesian_trajectory_planner_B.v_jg[0] *
      cartesian_trajectory_planner_B.v_jg[0] * (1.0 -
      cartesian_trajectory_planner_B.cth) + cartesian_trajectory_planner_B.cth,
      cartesian_trajectory_planner_B.tempR_tmp -
      cartesian_trajectory_planner_B.tempR_tmp_n,
      cartesian_trajectory_planner_B.tempR_tmp_l +
      cartesian_trajectory_planner_B.tempR_tmp_c,
      cartesian_trajectory_planner_B.tempR_tmp +
      cartesian_trajectory_planner_B.tempR_tmp_n,
      cartesian_trajectory_planner_B.v_jg[1] *
      cartesian_trajectory_planner_B.v_jg[1] * (1.0 -
      cartesian_trajectory_planner_B.cth) + cartesian_trajectory_planner_B.cth,
      cartesian_trajectory_planner_B.tempR_tmp_nc -
      cartesian_trajectory_planner_B.sth,
      cartesian_trajectory_planner_B.tempR_tmp_l -
      cartesian_trajectory_planner_B.tempR_tmp_c,
      cartesian_trajectory_planner_B.tempR_tmp_nc +
      cartesian_trajectory_planner_B.sth, cartesian_trajectory_planner_B.v_jg[2]
      * cartesian_trajectory_planner_B.v_jg[2] * (1.0 -
      cartesian_trajectory_planner_B.cth) + cartesian_trajectory_planner_B.cth,
      cartesian_trajectory_planner_B.tempR_l);
    for (cartesian_trajectory_planner_B.b_kstr_i = 0;
         cartesian_trajectory_planner_B.b_kstr_i < 3;
         cartesian_trajectory_planner_B.b_kstr_i++) {
      cartesian_trajectory_planner_B.loop_ub_h5 =
        cartesian_trajectory_planner_B.b_kstr_i + 1;
      cartesian_trajectory_planner_B.R_o[cartesian_trajectory_planner_B.loop_ub_h5
        - 1] = cartesian_trajectory_planner_B.tempR_l
        [(cartesian_trajectory_planner_B.loop_ub_h5 - 1) * 3];
      cartesian_trajectory_planner_B.loop_ub_h5 =
        cartesian_trajectory_planner_B.b_kstr_i + 1;
      cartesian_trajectory_planner_B.R_o[cartesian_trajectory_planner_B.loop_ub_h5
        + 2] = cartesian_trajectory_planner_B.tempR_l
        [(cartesian_trajectory_planner_B.loop_ub_h5 - 1) * 3 + 1];
      cartesian_trajectory_planner_B.loop_ub_h5 =
        cartesian_trajectory_planner_B.b_kstr_i + 1;
      cartesian_trajectory_planner_B.R_o[cartesian_trajectory_planner_B.loop_ub_h5
        + 5] = cartesian_trajectory_planner_B.tempR_l
        [(cartesian_trajectory_planner_B.loop_ub_h5 - 1) * 3 + 2];
    }

    memset(&cartesian_trajectory_planner_B.TJ[0], 0, sizeof(real_T) << 4U);
    for (cartesian_trajectory_planner_B.b_kstr_i = 0;
         cartesian_trajectory_planner_B.b_kstr_i < 3;
         cartesian_trajectory_planner_B.b_kstr_i++) {
      cartesian_trajectory_planner_B.loop_ub_h5 =
        cartesian_trajectory_planner_B.b_kstr_i << 2;
      cartesian_trajectory_planner_B.TJ[cartesian_trajectory_planner_B.loop_ub_h5]
        = cartesian_trajectory_planner_B.R_o[3 *
        cartesian_trajectory_planner_B.b_kstr_i];
      cartesian_trajectory_planner_B.TJ[cartesian_trajectory_planner_B.loop_ub_h5
        + 1] = cartesian_trajectory_planner_B.R_o[3 *
        cartesian_trajectory_planner_B.b_kstr_i + 1];
      cartesian_trajectory_planner_B.TJ[cartesian_trajectory_planner_B.loop_ub_h5
        + 2] = cartesian_trajectory_planner_B.R_o[3 *
        cartesian_trajectory_planner_B.b_kstr_i + 2];
    }

    cartesian_trajectory_planner_B.TJ[15] = 1.0;
    break;

   default:
    rigidBodyJoint_get_JointAxis_as(obj, cartesian_trajectory_planner_B.v_jg);
    memset(&cartesian_trajectory_planner_B.tempR_l[0], 0, 9U * sizeof(real_T));
    cartesian_trajectory_planner_B.tempR_l[0] = 1.0;
    cartesian_trajectory_planner_B.tempR_l[4] = 1.0;
    cartesian_trajectory_planner_B.tempR_l[8] = 1.0;
    for (cartesian_trajectory_planner_B.b_kstr_i = 0;
         cartesian_trajectory_planner_B.b_kstr_i < 3;
         cartesian_trajectory_planner_B.b_kstr_i++) {
      cartesian_trajectory_planner_B.loop_ub_h5 =
        cartesian_trajectory_planner_B.b_kstr_i << 2;
      cartesian_trajectory_planner_B.TJ[cartesian_trajectory_planner_B.loop_ub_h5]
        = cartesian_trajectory_planner_B.tempR_l[3 *
        cartesian_trajectory_planner_B.b_kstr_i];
      cartesian_trajectory_planner_B.TJ[cartesian_trajectory_planner_B.loop_ub_h5
        + 1] = cartesian_trajectory_planner_B.tempR_l[3 *
        cartesian_trajectory_planner_B.b_kstr_i + 1];
      cartesian_trajectory_planner_B.TJ[cartesian_trajectory_planner_B.loop_ub_h5
        + 2] = cartesian_trajectory_planner_B.tempR_l[3 *
        cartesian_trajectory_planner_B.b_kstr_i + 2];
      cartesian_trajectory_planner_B.TJ[cartesian_trajectory_planner_B.b_kstr_i
        + 12] =
        cartesian_trajectory_planner_B.v_jg[cartesian_trajectory_planner_B.b_kstr_i]
        * q_data[0];
    }

    cartesian_trajectory_planner_B.TJ[3] = 0.0;
    cartesian_trajectory_planner_B.TJ[7] = 0.0;
    cartesian_trajectory_planner_B.TJ[11] = 0.0;
    cartesian_trajectory_planner_B.TJ[15] = 1.0;
    break;
  }

  for (cartesian_trajectory_planner_B.b_kstr_i = 0;
       cartesian_trajectory_planner_B.b_kstr_i < 16;
       cartesian_trajectory_planner_B.b_kstr_i++) {
    cartesian_trajectory_planner_B.a[cartesian_trajectory_planner_B.b_kstr_i] =
      obj->JointToParentTransform[cartesian_trajectory_planner_B.b_kstr_i];
  }

  for (cartesian_trajectory_planner_B.b_kstr_i = 0;
       cartesian_trajectory_planner_B.b_kstr_i < 16;
       cartesian_trajectory_planner_B.b_kstr_i++) {
    cartesian_trajectory_planner_B.b[cartesian_trajectory_planner_B.b_kstr_i] =
      obj->ChildToJointTransform[cartesian_trajectory_planner_B.b_kstr_i];
  }

  for (cartesian_trajectory_planner_B.b_kstr_i = 0;
       cartesian_trajectory_planner_B.b_kstr_i < 4;
       cartesian_trajectory_planner_B.b_kstr_i++) {
    for (cartesian_trajectory_planner_B.loop_ub_h5 = 0;
         cartesian_trajectory_planner_B.loop_ub_h5 < 4;
         cartesian_trajectory_planner_B.loop_ub_h5++) {
      cartesian_trajectory_planner_B.a_tmp_tmp =
        cartesian_trajectory_planner_B.loop_ub_h5 << 2;
      cartesian_trajectory_planner_B.a_tmp =
        cartesian_trajectory_planner_B.b_kstr_i +
        cartesian_trajectory_planner_B.a_tmp_tmp;
      cartesian_trajectory_planner_B.a_d[cartesian_trajectory_planner_B.a_tmp] =
        0.0;
      cartesian_trajectory_planner_B.a_d[cartesian_trajectory_planner_B.a_tmp] +=
        cartesian_trajectory_planner_B.TJ[cartesian_trajectory_planner_B.a_tmp_tmp]
        * cartesian_trajectory_planner_B.a[cartesian_trajectory_planner_B.b_kstr_i];
      cartesian_trajectory_planner_B.a_d[cartesian_trajectory_planner_B.a_tmp] +=
        cartesian_trajectory_planner_B.TJ[cartesian_trajectory_planner_B.a_tmp_tmp
        + 1] *
        cartesian_trajectory_planner_B.a[cartesian_trajectory_planner_B.b_kstr_i
        + 4];
      cartesian_trajectory_planner_B.a_d[cartesian_trajectory_planner_B.a_tmp] +=
        cartesian_trajectory_planner_B.TJ[cartesian_trajectory_planner_B.a_tmp_tmp
        + 2] *
        cartesian_trajectory_planner_B.a[cartesian_trajectory_planner_B.b_kstr_i
        + 8];
      cartesian_trajectory_planner_B.a_d[cartesian_trajectory_planner_B.a_tmp] +=
        cartesian_trajectory_planner_B.TJ[cartesian_trajectory_planner_B.a_tmp_tmp
        + 3] *
        cartesian_trajectory_planner_B.a[cartesian_trajectory_planner_B.b_kstr_i
        + 12];
    }

    for (cartesian_trajectory_planner_B.loop_ub_h5 = 0;
         cartesian_trajectory_planner_B.loop_ub_h5 < 4;
         cartesian_trajectory_planner_B.loop_ub_h5++) {
      cartesian_trajectory_planner_B.a_tmp_tmp =
        cartesian_trajectory_planner_B.loop_ub_h5 << 2;
      cartesian_trajectory_planner_B.a_tmp =
        cartesian_trajectory_planner_B.b_kstr_i +
        cartesian_trajectory_planner_B.a_tmp_tmp;
      T[cartesian_trajectory_planner_B.a_tmp] = 0.0;
      T[cartesian_trajectory_planner_B.a_tmp] +=
        cartesian_trajectory_planner_B.b[cartesian_trajectory_planner_B.a_tmp_tmp]
        * cartesian_trajectory_planner_B.a_d[cartesian_trajectory_planner_B.b_kstr_i];
      T[cartesian_trajectory_planner_B.a_tmp] +=
        cartesian_trajectory_planner_B.b[cartesian_trajectory_planner_B.a_tmp_tmp
        + 1] *
        cartesian_trajectory_planner_B.a_d[cartesian_trajectory_planner_B.b_kstr_i
        + 4];
      T[cartesian_trajectory_planner_B.a_tmp] +=
        cartesian_trajectory_planner_B.b[cartesian_trajectory_planner_B.a_tmp_tmp
        + 2] *
        cartesian_trajectory_planner_B.a_d[cartesian_trajectory_planner_B.b_kstr_i
        + 8];
      T[cartesian_trajectory_planner_B.a_tmp] +=
        cartesian_trajectory_planner_B.b[cartesian_trajectory_planner_B.a_tmp_tmp
        + 3] *
        cartesian_trajectory_planner_B.a_d[cartesian_trajectory_planner_B.b_kstr_i
        + 12];
    }
  }
}

static void rigidBodyJoint_transformBodyToP(const
  c_rigidBodyJoint_cartesian_as_T *obj, real_T T[16])
{
  emxArray_char_T_cartesian_tra_T *switch_expression;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  cartesian_trajectory_planner_B.b_kstr_j = switch_expression->size[0] *
    switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression,
    cartesian_trajectory_planner_B.b_kstr_j);
  cartesian_trajectory_planner_B.loop_ub_ga = obj->Type->size[0] * obj->
    Type->size[1] - 1;
  for (cartesian_trajectory_planner_B.b_kstr_j = 0;
       cartesian_trajectory_planner_B.b_kstr_j <=
       cartesian_trajectory_planner_B.loop_ub_ga;
       cartesian_trajectory_planner_B.b_kstr_j++) {
    switch_expression->data[cartesian_trajectory_planner_B.b_kstr_j] = obj->
      Type->data[cartesian_trajectory_planner_B.b_kstr_j];
  }

  for (cartesian_trajectory_planner_B.b_kstr_j = 0;
       cartesian_trajectory_planner_B.b_kstr_j < 5;
       cartesian_trajectory_planner_B.b_kstr_j++) {
    cartesian_trajectory_planner_B.b_hn[cartesian_trajectory_planner_B.b_kstr_j]
      = tmp[cartesian_trajectory_planner_B.b_kstr_j];
  }

  cartesian_trajectory_planner_B.b_bool_l = false;
  if (switch_expression->size[1] == 5) {
    cartesian_trajectory_planner_B.b_kstr_j = 1;
    do {
      exitg1 = 0;
      if (cartesian_trajectory_planner_B.b_kstr_j - 1 < 5) {
        cartesian_trajectory_planner_B.loop_ub_ga =
          cartesian_trajectory_planner_B.b_kstr_j - 1;
        if (switch_expression->data[cartesian_trajectory_planner_B.loop_ub_ga]
            !=
            cartesian_trajectory_planner_B.b_hn[cartesian_trajectory_planner_B.loop_ub_ga])
        {
          exitg1 = 1;
        } else {
          cartesian_trajectory_planner_B.b_kstr_j++;
        }
      } else {
        cartesian_trajectory_planner_B.b_bool_l = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (cartesian_trajectory_planner_B.b_bool_l) {
    cartesian_trajectory_planner_B.b_kstr_j = 0;
  } else {
    for (cartesian_trajectory_planner_B.b_kstr_j = 0;
         cartesian_trajectory_planner_B.b_kstr_j < 8;
         cartesian_trajectory_planner_B.b_kstr_j++) {
      cartesian_trajectory_planner_B.b_kn[cartesian_trajectory_planner_B.b_kstr_j]
        = tmp_0[cartesian_trajectory_planner_B.b_kstr_j];
    }

    cartesian_trajectory_planner_B.b_bool_l = false;
    if (switch_expression->size[1] == 8) {
      cartesian_trajectory_planner_B.b_kstr_j = 1;
      do {
        exitg1 = 0;
        if (cartesian_trajectory_planner_B.b_kstr_j - 1 < 8) {
          cartesian_trajectory_planner_B.loop_ub_ga =
            cartesian_trajectory_planner_B.b_kstr_j - 1;
          if (switch_expression->data[cartesian_trajectory_planner_B.loop_ub_ga]
              !=
              cartesian_trajectory_planner_B.b_kn[cartesian_trajectory_planner_B.loop_ub_ga])
          {
            exitg1 = 1;
          } else {
            cartesian_trajectory_planner_B.b_kstr_j++;
          }
        } else {
          cartesian_trajectory_planner_B.b_bool_l = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (cartesian_trajectory_planner_B.b_bool_l) {
      cartesian_trajectory_planner_B.b_kstr_j = 1;
    } else {
      cartesian_trajectory_planner_B.b_kstr_j = -1;
    }
  }

  cartesian_trajec_emxFree_char_T(&switch_expression);
  switch (cartesian_trajectory_planner_B.b_kstr_j) {
   case 0:
    memset(&cartesian_trajectory_planner_B.TJ_g[0], 0, sizeof(real_T) << 4U);
    cartesian_trajectory_planner_B.TJ_g[0] = 1.0;
    cartesian_trajectory_planner_B.TJ_g[5] = 1.0;
    cartesian_trajectory_planner_B.TJ_g[10] = 1.0;
    cartesian_trajectory_planner_B.TJ_g[15] = 1.0;
    break;

   case 1:
    rigidBodyJoint_get_JointAxis_as(obj, cartesian_trajectory_planner_B.v_f);
    cartesian_trajectory_planner_B.axang_idx_0 =
      cartesian_trajectory_planner_B.v_f[0];
    cartesian_trajectory_planner_B.axang_idx_1 =
      cartesian_trajectory_planner_B.v_f[1];
    cartesian_trajectory_planner_B.axang_idx_2 =
      cartesian_trajectory_planner_B.v_f[2];
    cartesian_trajectory_planner_B.b_pi = 1.0 / sqrt
      ((cartesian_trajectory_planner_B.axang_idx_0 *
        cartesian_trajectory_planner_B.axang_idx_0 +
        cartesian_trajectory_planner_B.axang_idx_1 *
        cartesian_trajectory_planner_B.axang_idx_1) +
       cartesian_trajectory_planner_B.axang_idx_2 *
       cartesian_trajectory_planner_B.axang_idx_2);
    cartesian_trajectory_planner_B.v_f[0] =
      cartesian_trajectory_planner_B.axang_idx_0 *
      cartesian_trajectory_planner_B.b_pi;
    cartesian_trajectory_planner_B.v_f[1] =
      cartesian_trajectory_planner_B.axang_idx_1 *
      cartesian_trajectory_planner_B.b_pi;
    cartesian_trajectory_planner_B.v_f[2] =
      cartesian_trajectory_planner_B.axang_idx_2 *
      cartesian_trajectory_planner_B.b_pi;
    cartesian_trajectory_planner_B.axang_idx_0 =
      cartesian_trajectory_planner_B.v_f[1] *
      cartesian_trajectory_planner_B.v_f[0] * 0.0;
    cartesian_trajectory_planner_B.axang_idx_1 =
      cartesian_trajectory_planner_B.v_f[2] *
      cartesian_trajectory_planner_B.v_f[0] * 0.0;
    cartesian_trajectory_planner_B.axang_idx_2 =
      cartesian_trajectory_planner_B.v_f[2] *
      cartesian_trajectory_planner_B.v_f[1] * 0.0;
    cartesian_trajectory_planne_cat(cartesian_trajectory_planner_B.v_f[0] *
      cartesian_trajectory_planner_B.v_f[0] * 0.0 + 1.0,
      cartesian_trajectory_planner_B.axang_idx_0 -
      cartesian_trajectory_planner_B.v_f[2] * 0.0,
      cartesian_trajectory_planner_B.axang_idx_1 +
      cartesian_trajectory_planner_B.v_f[1] * 0.0,
      cartesian_trajectory_planner_B.axang_idx_0 +
      cartesian_trajectory_planner_B.v_f[2] * 0.0,
      cartesian_trajectory_planner_B.v_f[1] *
      cartesian_trajectory_planner_B.v_f[1] * 0.0 + 1.0,
      cartesian_trajectory_planner_B.axang_idx_2 -
      cartesian_trajectory_planner_B.v_f[0] * 0.0,
      cartesian_trajectory_planner_B.axang_idx_1 -
      cartesian_trajectory_planner_B.v_f[1] * 0.0,
      cartesian_trajectory_planner_B.axang_idx_2 +
      cartesian_trajectory_planner_B.v_f[0] * 0.0,
      cartesian_trajectory_planner_B.v_f[2] *
      cartesian_trajectory_planner_B.v_f[2] * 0.0 + 1.0,
      cartesian_trajectory_planner_B.tempR_m);
    for (cartesian_trajectory_planner_B.b_kstr_j = 0;
         cartesian_trajectory_planner_B.b_kstr_j < 3;
         cartesian_trajectory_planner_B.b_kstr_j++) {
      cartesian_trajectory_planner_B.loop_ub_ga =
        cartesian_trajectory_planner_B.b_kstr_j + 1;
      cartesian_trajectory_planner_B.R_m[cartesian_trajectory_planner_B.loop_ub_ga
        - 1] = cartesian_trajectory_planner_B.tempR_m
        [(cartesian_trajectory_planner_B.loop_ub_ga - 1) * 3];
      cartesian_trajectory_planner_B.loop_ub_ga =
        cartesian_trajectory_planner_B.b_kstr_j + 1;
      cartesian_trajectory_planner_B.R_m[cartesian_trajectory_planner_B.loop_ub_ga
        + 2] = cartesian_trajectory_planner_B.tempR_m
        [(cartesian_trajectory_planner_B.loop_ub_ga - 1) * 3 + 1];
      cartesian_trajectory_planner_B.loop_ub_ga =
        cartesian_trajectory_planner_B.b_kstr_j + 1;
      cartesian_trajectory_planner_B.R_m[cartesian_trajectory_planner_B.loop_ub_ga
        + 5] = cartesian_trajectory_planner_B.tempR_m
        [(cartesian_trajectory_planner_B.loop_ub_ga - 1) * 3 + 2];
    }

    memset(&cartesian_trajectory_planner_B.TJ_g[0], 0, sizeof(real_T) << 4U);
    for (cartesian_trajectory_planner_B.b_kstr_j = 0;
         cartesian_trajectory_planner_B.b_kstr_j < 3;
         cartesian_trajectory_planner_B.b_kstr_j++) {
      cartesian_trajectory_planner_B.loop_ub_ga =
        cartesian_trajectory_planner_B.b_kstr_j << 2;
      cartesian_trajectory_planner_B.TJ_g[cartesian_trajectory_planner_B.loop_ub_ga]
        = cartesian_trajectory_planner_B.R_m[3 *
        cartesian_trajectory_planner_B.b_kstr_j];
      cartesian_trajectory_planner_B.TJ_g[cartesian_trajectory_planner_B.loop_ub_ga
        + 1] = cartesian_trajectory_planner_B.R_m[3 *
        cartesian_trajectory_planner_B.b_kstr_j + 1];
      cartesian_trajectory_planner_B.TJ_g[cartesian_trajectory_planner_B.loop_ub_ga
        + 2] = cartesian_trajectory_planner_B.R_m[3 *
        cartesian_trajectory_planner_B.b_kstr_j + 2];
    }

    cartesian_trajectory_planner_B.TJ_g[15] = 1.0;
    break;

   default:
    rigidBodyJoint_get_JointAxis_as(obj, cartesian_trajectory_planner_B.v_f);
    memset(&cartesian_trajectory_planner_B.tempR_m[0], 0, 9U * sizeof(real_T));
    cartesian_trajectory_planner_B.tempR_m[0] = 1.0;
    cartesian_trajectory_planner_B.tempR_m[4] = 1.0;
    cartesian_trajectory_planner_B.tempR_m[8] = 1.0;
    for (cartesian_trajectory_planner_B.b_kstr_j = 0;
         cartesian_trajectory_planner_B.b_kstr_j < 3;
         cartesian_trajectory_planner_B.b_kstr_j++) {
      cartesian_trajectory_planner_B.loop_ub_ga =
        cartesian_trajectory_planner_B.b_kstr_j << 2;
      cartesian_trajectory_planner_B.TJ_g[cartesian_trajectory_planner_B.loop_ub_ga]
        = cartesian_trajectory_planner_B.tempR_m[3 *
        cartesian_trajectory_planner_B.b_kstr_j];
      cartesian_trajectory_planner_B.TJ_g[cartesian_trajectory_planner_B.loop_ub_ga
        + 1] = cartesian_trajectory_planner_B.tempR_m[3 *
        cartesian_trajectory_planner_B.b_kstr_j + 1];
      cartesian_trajectory_planner_B.TJ_g[cartesian_trajectory_planner_B.loop_ub_ga
        + 2] = cartesian_trajectory_planner_B.tempR_m[3 *
        cartesian_trajectory_planner_B.b_kstr_j + 2];
      cartesian_trajectory_planner_B.TJ_g[cartesian_trajectory_planner_B.b_kstr_j
        + 12] =
        cartesian_trajectory_planner_B.v_f[cartesian_trajectory_planner_B.b_kstr_j]
        * 0.0;
    }

    cartesian_trajectory_planner_B.TJ_g[3] = 0.0;
    cartesian_trajectory_planner_B.TJ_g[7] = 0.0;
    cartesian_trajectory_planner_B.TJ_g[11] = 0.0;
    cartesian_trajectory_planner_B.TJ_g[15] = 1.0;
    break;
  }

  for (cartesian_trajectory_planner_B.b_kstr_j = 0;
       cartesian_trajectory_planner_B.b_kstr_j < 16;
       cartesian_trajectory_planner_B.b_kstr_j++) {
    cartesian_trajectory_planner_B.a_l[cartesian_trajectory_planner_B.b_kstr_j] =
      obj->JointToParentTransform[cartesian_trajectory_planner_B.b_kstr_j];
  }

  for (cartesian_trajectory_planner_B.b_kstr_j = 0;
       cartesian_trajectory_planner_B.b_kstr_j < 16;
       cartesian_trajectory_planner_B.b_kstr_j++) {
    cartesian_trajectory_planner_B.b_d[cartesian_trajectory_planner_B.b_kstr_j] =
      obj->ChildToJointTransform[cartesian_trajectory_planner_B.b_kstr_j];
  }

  for (cartesian_trajectory_planner_B.b_kstr_j = 0;
       cartesian_trajectory_planner_B.b_kstr_j < 4;
       cartesian_trajectory_planner_B.b_kstr_j++) {
    for (cartesian_trajectory_planner_B.loop_ub_ga = 0;
         cartesian_trajectory_planner_B.loop_ub_ga < 4;
         cartesian_trajectory_planner_B.loop_ub_ga++) {
      cartesian_trajectory_planner_B.a_tmp_tmp_e =
        cartesian_trajectory_planner_B.loop_ub_ga << 2;
      cartesian_trajectory_planner_B.a_tmp_j =
        cartesian_trajectory_planner_B.b_kstr_j +
        cartesian_trajectory_planner_B.a_tmp_tmp_e;
      cartesian_trajectory_planner_B.a_dy[cartesian_trajectory_planner_B.a_tmp_j]
        = 0.0;
      cartesian_trajectory_planner_B.a_dy[cartesian_trajectory_planner_B.a_tmp_j]
        +=
        cartesian_trajectory_planner_B.TJ_g[cartesian_trajectory_planner_B.a_tmp_tmp_e]
        * cartesian_trajectory_planner_B.a_l[cartesian_trajectory_planner_B.b_kstr_j];
      cartesian_trajectory_planner_B.a_dy[cartesian_trajectory_planner_B.a_tmp_j]
        +=
        cartesian_trajectory_planner_B.TJ_g[cartesian_trajectory_planner_B.a_tmp_tmp_e
        + 1] *
        cartesian_trajectory_planner_B.a_l[cartesian_trajectory_planner_B.b_kstr_j
        + 4];
      cartesian_trajectory_planner_B.a_dy[cartesian_trajectory_planner_B.a_tmp_j]
        +=
        cartesian_trajectory_planner_B.TJ_g[cartesian_trajectory_planner_B.a_tmp_tmp_e
        + 2] *
        cartesian_trajectory_planner_B.a_l[cartesian_trajectory_planner_B.b_kstr_j
        + 8];
      cartesian_trajectory_planner_B.a_dy[cartesian_trajectory_planner_B.a_tmp_j]
        +=
        cartesian_trajectory_planner_B.TJ_g[cartesian_trajectory_planner_B.a_tmp_tmp_e
        + 3] *
        cartesian_trajectory_planner_B.a_l[cartesian_trajectory_planner_B.b_kstr_j
        + 12];
    }

    for (cartesian_trajectory_planner_B.loop_ub_ga = 0;
         cartesian_trajectory_planner_B.loop_ub_ga < 4;
         cartesian_trajectory_planner_B.loop_ub_ga++) {
      cartesian_trajectory_planner_B.a_tmp_tmp_e =
        cartesian_trajectory_planner_B.loop_ub_ga << 2;
      cartesian_trajectory_planner_B.a_tmp_j =
        cartesian_trajectory_planner_B.b_kstr_j +
        cartesian_trajectory_planner_B.a_tmp_tmp_e;
      T[cartesian_trajectory_planner_B.a_tmp_j] = 0.0;
      T[cartesian_trajectory_planner_B.a_tmp_j] +=
        cartesian_trajectory_planner_B.b_d[cartesian_trajectory_planner_B.a_tmp_tmp_e]
        * cartesian_trajectory_planner_B.a_dy[cartesian_trajectory_planner_B.b_kstr_j];
      T[cartesian_trajectory_planner_B.a_tmp_j] +=
        cartesian_trajectory_planner_B.b_d[cartesian_trajectory_planner_B.a_tmp_tmp_e
        + 1] *
        cartesian_trajectory_planner_B.a_dy[cartesian_trajectory_planner_B.b_kstr_j
        + 4];
      T[cartesian_trajectory_planner_B.a_tmp_j] +=
        cartesian_trajectory_planner_B.b_d[cartesian_trajectory_planner_B.a_tmp_tmp_e
        + 2] *
        cartesian_trajectory_planner_B.a_dy[cartesian_trajectory_planner_B.b_kstr_j
        + 8];
      T[cartesian_trajectory_planner_B.a_tmp_j] +=
        cartesian_trajectory_planner_B.b_d[cartesian_trajectory_planner_B.a_tmp_tmp_e
        + 3] *
        cartesian_trajectory_planner_B.a_dy[cartesian_trajectory_planner_B.b_kstr_j
        + 12];
    }
  }
}

static void RigidBodyTree_efficientFKAndJac(x_robotics_manip_internal_Rig_T *obj,
  const real_T qv[6], const emxArray_char_T_cartesian_tra_T *body1Name, real_T
  T_data[], int32_T T_size[2], emxArray_real_T_cartesian_tra_T *Jac)
{
  w_robotics_manip_internal_Rig_T *body1;
  w_robotics_manip_internal_Rig_T *body2;
  emxArray_real_T_cartesian_tra_T *kinematicPathIndices;
  c_rigidBodyJoint_cartesian_as_T *joint;
  emxArray_char_T_cartesian_tra_T *body2Name;
  emxArray_real_T_cartesian_tra_T *ancestorIndices1;
  emxArray_real_T_cartesian_tra_T *ancestorIndices2;
  emxArray_real_T_cartesian_tra_T *y;
  emxArray_real_T_cartesian_tra_T *b;
  w_robotics_manip_internal_Rig_T *body;
  emxArray_char_T_cartesian_tra_T *bname;
  emxArray_real_T_cartesian_tra_T *ancestorIndices1_0;
  emxArray_real_T_cartesian_tra_T *ancestorIndices2_0;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  boolean_T exitg1;
  int32_T exitg2;
  cartesian_trajec_emxInit_char_T(&body2Name, 2);
  cartesian_trajectory_planner_B.b_kstr = body2Name->size[0] * body2Name->size[1];
  body2Name->size[0] = 1;
  body2Name->size[1] = obj->Base.NameInternal->size[1];
  cartes_emxEnsureCapacity_char_T(body2Name,
    cartesian_trajectory_planner_B.b_kstr);
  cartesian_trajectory_planner_B.loop_ub_k = obj->Base.NameInternal->size[0] *
    obj->Base.NameInternal->size[1] - 1;
  for (cartesian_trajectory_planner_B.b_kstr = 0;
       cartesian_trajectory_planner_B.b_kstr <=
       cartesian_trajectory_planner_B.loop_ub_k;
       cartesian_trajectory_planner_B.b_kstr++) {
    body2Name->data[cartesian_trajectory_planner_B.b_kstr] =
      obj->Base.NameInternal->data[cartesian_trajectory_planner_B.b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&bname, 2);
  cartesian_trajectory_planner_B.bid1 = -1.0;
  cartesian_trajectory_planner_B.b_kstr = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj->Base.NameInternal->size[1];
  cartes_emxEnsureCapacity_char_T(bname, cartesian_trajectory_planner_B.b_kstr);
  cartesian_trajectory_planner_B.loop_ub_k = obj->Base.NameInternal->size[0] *
    obj->Base.NameInternal->size[1] - 1;
  for (cartesian_trajectory_planner_B.b_kstr = 0;
       cartesian_trajectory_planner_B.b_kstr <=
       cartesian_trajectory_planner_B.loop_ub_k;
       cartesian_trajectory_planner_B.b_kstr++) {
    bname->data[cartesian_trajectory_planner_B.b_kstr] = obj->
      Base.NameInternal->data[cartesian_trajectory_planner_B.b_kstr];
  }

  if (cartesian_trajectory_pla_strcmp(bname, body1Name)) {
    cartesian_trajectory_planner_B.bid1 = 0.0;
  } else {
    cartesian_trajectory_planner_B.qidx_idx_1 = obj->NumBodies;
    cartesian_trajectory_planner_B.b_i_d = 0;
    exitg1 = false;
    while ((!exitg1) && (cartesian_trajectory_planner_B.b_i_d <=
                         static_cast<int32_T>
                         (cartesian_trajectory_planner_B.qidx_idx_1) - 1)) {
      body1 = obj->Bodies[cartesian_trajectory_planner_B.b_i_d];
      cartesian_trajectory_planner_B.b_kstr = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = body1->NameInternal->size[1];
      cartes_emxEnsureCapacity_char_T(bname,
        cartesian_trajectory_planner_B.b_kstr);
      cartesian_trajectory_planner_B.loop_ub_k = body1->NameInternal->size[0] *
        body1->NameInternal->size[1] - 1;
      for (cartesian_trajectory_planner_B.b_kstr = 0;
           cartesian_trajectory_planner_B.b_kstr <=
           cartesian_trajectory_planner_B.loop_ub_k;
           cartesian_trajectory_planner_B.b_kstr++) {
        bname->data[cartesian_trajectory_planner_B.b_kstr] = body1->
          NameInternal->data[cartesian_trajectory_planner_B.b_kstr];
      }

      if (cartesian_trajectory_pla_strcmp(bname, body1Name)) {
        cartesian_trajectory_planner_B.bid1 = static_cast<real_T>
          (cartesian_trajectory_planner_B.b_i_d) + 1.0;
        exitg1 = true;
      } else {
        cartesian_trajectory_planner_B.b_i_d++;
      }
    }
  }

  cartesian_trajectory_planner_B.bid2 = -1.0;
  cartesian_trajectory_planner_B.b_kstr = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj->Base.NameInternal->size[1];
  cartes_emxEnsureCapacity_char_T(bname, cartesian_trajectory_planner_B.b_kstr);
  cartesian_trajectory_planner_B.loop_ub_k = obj->Base.NameInternal->size[0] *
    obj->Base.NameInternal->size[1] - 1;
  for (cartesian_trajectory_planner_B.b_kstr = 0;
       cartesian_trajectory_planner_B.b_kstr <=
       cartesian_trajectory_planner_B.loop_ub_k;
       cartesian_trajectory_planner_B.b_kstr++) {
    bname->data[cartesian_trajectory_planner_B.b_kstr] = obj->
      Base.NameInternal->data[cartesian_trajectory_planner_B.b_kstr];
  }

  if (cartesian_trajectory_pla_strcmp(bname, body2Name)) {
    cartesian_trajectory_planner_B.bid2 = 0.0;
  } else {
    cartesian_trajectory_planner_B.qidx_idx_1 = obj->NumBodies;
    cartesian_trajectory_planner_B.b_i_d = 0;
    exitg1 = false;
    while ((!exitg1) && (cartesian_trajectory_planner_B.b_i_d <=
                         static_cast<int32_T>
                         (cartesian_trajectory_planner_B.qidx_idx_1) - 1)) {
      body1 = obj->Bodies[cartesian_trajectory_planner_B.b_i_d];
      cartesian_trajectory_planner_B.b_kstr = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = body1->NameInternal->size[1];
      cartes_emxEnsureCapacity_char_T(bname,
        cartesian_trajectory_planner_B.b_kstr);
      cartesian_trajectory_planner_B.loop_ub_k = body1->NameInternal->size[0] *
        body1->NameInternal->size[1] - 1;
      for (cartesian_trajectory_planner_B.b_kstr = 0;
           cartesian_trajectory_planner_B.b_kstr <=
           cartesian_trajectory_planner_B.loop_ub_k;
           cartesian_trajectory_planner_B.b_kstr++) {
        bname->data[cartesian_trajectory_planner_B.b_kstr] = body1->
          NameInternal->data[cartesian_trajectory_planner_B.b_kstr];
      }

      if (cartesian_trajectory_pla_strcmp(bname, body2Name)) {
        cartesian_trajectory_planner_B.bid2 = static_cast<real_T>
          (cartesian_trajectory_planner_B.b_i_d) + 1.0;
        exitg1 = true;
      } else {
        cartesian_trajectory_planner_B.b_i_d++;
      }
    }
  }

  cartesian_trajec_emxFree_char_T(&bname);
  if (cartesian_trajectory_planner_B.bid1 == 0.0) {
    body1 = &obj->Base;
  } else {
    body1 = obj->Bodies[static_cast<int32_T>(cartesian_trajectory_planner_B.bid1)
      - 1];
  }

  if (cartesian_trajectory_planner_B.bid2 == 0.0) {
    body2 = &obj->Base;
  } else {
    body2 = obj->Bodies[static_cast<int32_T>(cartesian_trajectory_planner_B.bid2)
      - 1];
  }

  cartesian_trajec_emxInit_real_T(&ancestorIndices1, 2);
  body = body1;
  cartesian_trajectory_planner_B.b_kstr = ancestorIndices1->size[0] *
    ancestorIndices1->size[1];
  ancestorIndices1->size[0] = 1;
  ancestorIndices1->size[1] = static_cast<int32_T>(obj->NumBodies + 1.0);
  cartes_emxEnsureCapacity_real_T(ancestorIndices1,
    cartesian_trajectory_planner_B.b_kstr);
  cartesian_trajectory_planner_B.loop_ub_k = static_cast<int32_T>(obj->NumBodies
    + 1.0) - 1;
  for (cartesian_trajectory_planner_B.b_kstr = 0;
       cartesian_trajectory_planner_B.b_kstr <=
       cartesian_trajectory_planner_B.loop_ub_k;
       cartesian_trajectory_planner_B.b_kstr++) {
    ancestorIndices1->data[cartesian_trajectory_planner_B.b_kstr] = 0.0;
  }

  cartesian_trajectory_planner_B.bid1 = 2.0;
  ancestorIndices1->data[0] = body1->Index;
  while (body->ParentIndex > 0.0) {
    body = obj->Bodies[static_cast<int32_T>(body->ParentIndex) - 1];
    ancestorIndices1->data[static_cast<int32_T>
      (cartesian_trajectory_planner_B.bid1) - 1] = body->Index;
    cartesian_trajectory_planner_B.bid1++;
  }

  if (body->Index > 0.0) {
    ancestorIndices1->data[static_cast<int32_T>
      (cartesian_trajectory_planner_B.bid1) - 1] = body->ParentIndex;
    cartesian_trajectory_planner_B.bid1++;
  }

  cartesian_trajec_emxInit_real_T(&ancestorIndices1_0, 2);
  cartesian_trajectory_planner_B.loop_ub_k = static_cast<int32_T>
    (cartesian_trajectory_planner_B.bid1 - 1.0);
  cartesian_trajectory_planner_B.b_kstr = ancestorIndices1_0->size[0] *
    ancestorIndices1_0->size[1];
  ancestorIndices1_0->size[0] = 1;
  ancestorIndices1_0->size[1] = cartesian_trajectory_planner_B.loop_ub_k;
  cartes_emxEnsureCapacity_real_T(ancestorIndices1_0,
    cartesian_trajectory_planner_B.b_kstr);
  for (cartesian_trajectory_planner_B.b_kstr = 0;
       cartesian_trajectory_planner_B.b_kstr <
       cartesian_trajectory_planner_B.loop_ub_k;
       cartesian_trajectory_planner_B.b_kstr++) {
    ancestorIndices1_0->data[cartesian_trajectory_planner_B.b_kstr] =
      ancestorIndices1->data[cartesian_trajectory_planner_B.b_kstr];
  }

  cartesian_trajectory_planner_B.b_kstr = ancestorIndices1->size[0] *
    ancestorIndices1->size[1];
  ancestorIndices1->size[0] = 1;
  ancestorIndices1->size[1] = ancestorIndices1_0->size[1];
  cartes_emxEnsureCapacity_real_T(ancestorIndices1,
    cartesian_trajectory_planner_B.b_kstr);
  cartesian_trajectory_planner_B.loop_ub_k = ancestorIndices1_0->size[0] *
    ancestorIndices1_0->size[1];
  for (cartesian_trajectory_planner_B.b_kstr = 0;
       cartesian_trajectory_planner_B.b_kstr <
       cartesian_trajectory_planner_B.loop_ub_k;
       cartesian_trajectory_planner_B.b_kstr++) {
    ancestorIndices1->data[cartesian_trajectory_planner_B.b_kstr] =
      ancestorIndices1_0->data[cartesian_trajectory_planner_B.b_kstr];
  }

  cartesian_trajec_emxFree_real_T(&ancestorIndices1_0);
  cartesian_trajec_emxInit_real_T(&ancestorIndices2, 2);
  body = body2;
  cartesian_trajectory_planner_B.b_kstr = ancestorIndices2->size[0] *
    ancestorIndices2->size[1];
  ancestorIndices2->size[0] = 1;
  ancestorIndices2->size[1] = static_cast<int32_T>(obj->NumBodies + 1.0);
  cartes_emxEnsureCapacity_real_T(ancestorIndices2,
    cartesian_trajectory_planner_B.b_kstr);
  cartesian_trajectory_planner_B.loop_ub_k = static_cast<int32_T>(obj->NumBodies
    + 1.0) - 1;
  for (cartesian_trajectory_planner_B.b_kstr = 0;
       cartesian_trajectory_planner_B.b_kstr <=
       cartesian_trajectory_planner_B.loop_ub_k;
       cartesian_trajectory_planner_B.b_kstr++) {
    ancestorIndices2->data[cartesian_trajectory_planner_B.b_kstr] = 0.0;
  }

  cartesian_trajectory_planner_B.bid1 = 2.0;
  ancestorIndices2->data[0] = body2->Index;
  while (body->ParentIndex > 0.0) {
    body = obj->Bodies[static_cast<int32_T>(body->ParentIndex) - 1];
    ancestorIndices2->data[static_cast<int32_T>
      (cartesian_trajectory_planner_B.bid1) - 1] = body->Index;
    cartesian_trajectory_planner_B.bid1++;
  }

  if (body->Index > 0.0) {
    ancestorIndices2->data[static_cast<int32_T>
      (cartesian_trajectory_planner_B.bid1) - 1] = body->ParentIndex;
    cartesian_trajectory_planner_B.bid1++;
  }

  cartesian_trajec_emxInit_real_T(&ancestorIndices2_0, 2);
  cartesian_trajectory_planner_B.loop_ub_k = static_cast<int32_T>
    (cartesian_trajectory_planner_B.bid1 - 1.0);
  cartesian_trajectory_planner_B.b_kstr = ancestorIndices2_0->size[0] *
    ancestorIndices2_0->size[1];
  ancestorIndices2_0->size[0] = 1;
  ancestorIndices2_0->size[1] = cartesian_trajectory_planner_B.loop_ub_k;
  cartes_emxEnsureCapacity_real_T(ancestorIndices2_0,
    cartesian_trajectory_planner_B.b_kstr);
  for (cartesian_trajectory_planner_B.b_kstr = 0;
       cartesian_trajectory_planner_B.b_kstr <
       cartesian_trajectory_planner_B.loop_ub_k;
       cartesian_trajectory_planner_B.b_kstr++) {
    ancestorIndices2_0->data[cartesian_trajectory_planner_B.b_kstr] =
      ancestorIndices2->data[cartesian_trajectory_planner_B.b_kstr];
  }

  cartesian_trajectory_planner_B.b_kstr = ancestorIndices2->size[0] *
    ancestorIndices2->size[1];
  ancestorIndices2->size[0] = 1;
  ancestorIndices2->size[1] = ancestorIndices2_0->size[1];
  cartes_emxEnsureCapacity_real_T(ancestorIndices2,
    cartesian_trajectory_planner_B.b_kstr);
  cartesian_trajectory_planner_B.loop_ub_k = ancestorIndices2_0->size[0] *
    ancestorIndices2_0->size[1];
  for (cartesian_trajectory_planner_B.b_kstr = 0;
       cartesian_trajectory_planner_B.b_kstr <
       cartesian_trajectory_planner_B.loop_ub_k;
       cartesian_trajectory_planner_B.b_kstr++) {
    ancestorIndices2->data[cartesian_trajectory_planner_B.b_kstr] =
      ancestorIndices2_0->data[cartesian_trajectory_planner_B.b_kstr];
  }

  cartesian_trajec_emxFree_real_T(&ancestorIndices2_0);
  cartesian_trajectory_planner_B.bid1 = ancestorIndices1->size[1];
  cartesian_trajectory_planner_B.qidx_idx_1 = ancestorIndices2->size[1];
  if (cartesian_trajectory_planner_B.bid1 <
      cartesian_trajectory_planner_B.qidx_idx_1) {
    cartesian_trajectory_planner_B.qidx_idx_1 =
      cartesian_trajectory_planner_B.bid1;
  }

  cartesian_trajectory_planner_B.bid1 = static_cast<int32_T>
    (cartesian_trajectory_planner_B.qidx_idx_1);
  cartesian_trajectory_planner_B.b_i_d = 0;
  exitg1 = false;
  while ((!exitg1) && (cartesian_trajectory_planner_B.b_i_d <=
                       static_cast<int32_T>
                       (cartesian_trajectory_planner_B.qidx_idx_1) - 2)) {
    if (ancestorIndices1->data[static_cast<int32_T>(static_cast<real_T>
         (ancestorIndices1->size[1]) - (static_cast<real_T>
          (cartesian_trajectory_planner_B.b_i_d) + 1.0)) - 1] !=
        ancestorIndices2->data[static_cast<int32_T>(static_cast<real_T>
         (ancestorIndices2->size[1]) - (static_cast<real_T>
          (cartesian_trajectory_planner_B.b_i_d) + 1.0)) - 1]) {
      cartesian_trajectory_planner_B.bid1 = static_cast<real_T>
        (cartesian_trajectory_planner_B.b_i_d) + 1.0;
      exitg1 = true;
    } else {
      cartesian_trajectory_planner_B.b_i_d++;
    }
  }

  cartesian_trajectory_planner_B.qidx_idx_1 = static_cast<real_T>
    (ancestorIndices1->size[1]) - cartesian_trajectory_planner_B.bid1;
  if (1.0 > cartesian_trajectory_planner_B.qidx_idx_1) {
    cartesian_trajectory_planner_B.b_i_d = -1;
  } else {
    cartesian_trajectory_planner_B.b_i_d = static_cast<int32_T>
      (cartesian_trajectory_planner_B.qidx_idx_1) - 1;
  }

  cartesian_trajectory_planner_B.qidx_idx_1 = static_cast<real_T>
    (ancestorIndices2->size[1]) - cartesian_trajectory_planner_B.bid1;
  if (1.0 > cartesian_trajectory_planner_B.qidx_idx_1) {
    cartesian_trajectory_planner_B.j_f = 0;
    cartesian_trajectory_planner_B.jointSign = 1;
    cartesian_trajectory_planner_B.g = -1;
  } else {
    cartesian_trajectory_planner_B.j_f = static_cast<int32_T>
      (cartesian_trajectory_planner_B.qidx_idx_1) - 1;
    cartesian_trajectory_planner_B.jointSign = -1;
    cartesian_trajectory_planner_B.g = 0;
  }

  cartesian_trajec_emxInit_real_T(&kinematicPathIndices, 2);
  cartesian_trajectory_planner_B.b_kstr = kinematicPathIndices->size[0] *
    kinematicPathIndices->size[1];
  kinematicPathIndices->size[0] = 1;
  cartesian_trajectory_planner_B.loop_ub_k = div_s32_floor
    (cartesian_trajectory_planner_B.g - cartesian_trajectory_planner_B.j_f,
     cartesian_trajectory_planner_B.jointSign);
  kinematicPathIndices->size[1] = (cartesian_trajectory_planner_B.loop_ub_k +
    cartesian_trajectory_planner_B.b_i_d) + 3;
  cartes_emxEnsureCapacity_real_T(kinematicPathIndices,
    cartesian_trajectory_planner_B.b_kstr);
  for (cartesian_trajectory_planner_B.b_kstr = 0;
       cartesian_trajectory_planner_B.b_kstr <=
       cartesian_trajectory_planner_B.b_i_d;
       cartesian_trajectory_planner_B.b_kstr++) {
    kinematicPathIndices->data[cartesian_trajectory_planner_B.b_kstr] =
      ancestorIndices1->data[cartesian_trajectory_planner_B.b_kstr];
  }

  kinematicPathIndices->data[cartesian_trajectory_planner_B.b_i_d + 1] =
    ancestorIndices1->data[static_cast<int32_T>((static_cast<real_T>
    (ancestorIndices1->size[1]) - cartesian_trajectory_planner_B.bid1) + 1.0) -
    1];
  cartesian_trajec_emxFree_real_T(&ancestorIndices1);
  for (cartesian_trajectory_planner_B.b_kstr = 0;
       cartesian_trajectory_planner_B.b_kstr <=
       cartesian_trajectory_planner_B.loop_ub_k;
       cartesian_trajectory_planner_B.b_kstr++) {
    kinematicPathIndices->data[(cartesian_trajectory_planner_B.b_kstr +
      cartesian_trajectory_planner_B.b_i_d) + 2] = ancestorIndices2->
      data[cartesian_trajectory_planner_B.jointSign *
      cartesian_trajectory_planner_B.b_kstr + cartesian_trajectory_planner_B.j_f];
  }

  cartesian_trajec_emxFree_real_T(&ancestorIndices2);
  memset(&cartesian_trajectory_planner_B.T1[0], 0, sizeof(real_T) << 4U);
  cartesian_trajectory_planner_B.T1[0] = 1.0;
  cartesian_trajectory_planner_B.T1[5] = 1.0;
  cartesian_trajectory_planner_B.T1[10] = 1.0;
  cartesian_trajectory_planner_B.T1[15] = 1.0;
  cartesian_trajectory_planner_B.b_kstr = Jac->size[0] * Jac->size[1];
  Jac->size[0] = 6;
  Jac->size[1] = static_cast<int32_T>(obj->PositionNumber);
  cartes_emxEnsureCapacity_real_T(Jac, cartesian_trajectory_planner_B.b_kstr);
  cartesian_trajectory_planner_B.loop_ub_k = 6 * static_cast<int32_T>
    (obj->PositionNumber) - 1;
  for (cartesian_trajectory_planner_B.b_kstr = 0;
       cartesian_trajectory_planner_B.b_kstr <=
       cartesian_trajectory_planner_B.loop_ub_k;
       cartesian_trajectory_planner_B.b_kstr++) {
    Jac->data[cartesian_trajectory_planner_B.b_kstr] = 0.0;
  }

  cartesian_trajectory_planner_B.j_f = kinematicPathIndices->size[1] - 2;
  cartesian_trajec_emxInit_real_T(&y, 2);
  cartesian_trajec_emxInit_real_T(&b, 2);
  if (0 <= cartesian_trajectory_planner_B.j_f) {
    for (cartesian_trajectory_planner_B.b_kstr = 0;
         cartesian_trajectory_planner_B.b_kstr < 5;
         cartesian_trajectory_planner_B.b_kstr++) {
      cartesian_trajectory_planner_B.b_h[cartesian_trajectory_planner_B.b_kstr] =
        tmp[cartesian_trajectory_planner_B.b_kstr];
    }
  }

  for (cartesian_trajectory_planner_B.b_i_d = 0;
       cartesian_trajectory_planner_B.b_i_d <=
       cartesian_trajectory_planner_B.j_f; cartesian_trajectory_planner_B.b_i_d
       ++) {
    if (kinematicPathIndices->data[cartesian_trajectory_planner_B.b_i_d] != 0.0)
    {
      body1 = obj->Bodies[static_cast<int32_T>(kinematicPathIndices->
        data[cartesian_trajectory_planner_B.b_i_d]) - 1];
    } else {
      body1 = &obj->Base;
    }

    cartesian_trajectory_planner_B.b_kstr = static_cast<int32_T>
      ((static_cast<real_T>(cartesian_trajectory_planner_B.b_i_d) + 1.0) + 1.0)
      - 1;
    if (kinematicPathIndices->data[cartesian_trajectory_planner_B.b_kstr] != 0.0)
    {
      body2 = obj->Bodies[static_cast<int32_T>(kinematicPathIndices->
        data[cartesian_trajectory_planner_B.b_kstr]) - 1];
    } else {
      body2 = &obj->Base;
    }

    cartesian_trajectory_planner_B.nextBodyIsParent = (body2->Index ==
      body1->ParentIndex);
    if (cartesian_trajectory_planner_B.nextBodyIsParent) {
      body2 = body1;
      cartesian_trajectory_planner_B.jointSign = 1;
    } else {
      cartesian_trajectory_planner_B.jointSign = -1;
    }

    joint = body2->JointInternal;
    cartesian_trajectory_planner_B.b_kstr = body2Name->size[0] * body2Name->
      size[1];
    body2Name->size[0] = 1;
    body2Name->size[1] = joint->Type->size[1];
    cartes_emxEnsureCapacity_char_T(body2Name,
      cartesian_trajectory_planner_B.b_kstr);
    cartesian_trajectory_planner_B.loop_ub_k = joint->Type->size[0] *
      joint->Type->size[1] - 1;
    for (cartesian_trajectory_planner_B.b_kstr = 0;
         cartesian_trajectory_planner_B.b_kstr <=
         cartesian_trajectory_planner_B.loop_ub_k;
         cartesian_trajectory_planner_B.b_kstr++) {
      body2Name->data[cartesian_trajectory_planner_B.b_kstr] = joint->Type->
        data[cartesian_trajectory_planner_B.b_kstr];
    }

    cartesian_trajectory_planner_B.b_bool = false;
    if (body2Name->size[1] == 5) {
      cartesian_trajectory_planner_B.b_kstr = 1;
      do {
        exitg2 = 0;
        if (cartesian_trajectory_planner_B.b_kstr - 1 < 5) {
          cartesian_trajectory_planner_B.g =
            cartesian_trajectory_planner_B.b_kstr - 1;
          if (body2Name->data[cartesian_trajectory_planner_B.g] !=
              cartesian_trajectory_planner_B.b_h[cartesian_trajectory_planner_B.g])
          {
            exitg2 = 1;
          } else {
            cartesian_trajectory_planner_B.b_kstr++;
          }
        } else {
          cartesian_trajectory_planner_B.b_bool = true;
          exitg2 = 1;
        }
      } while (exitg2 == 0);
    }

    if (cartesian_trajectory_planner_B.b_bool) {
      rigidBodyJoint_transformBodyToP(joint, cartesian_trajectory_planner_B.Tc2p);
    } else {
      cartesian_trajectory_planner_B.b_kstr = static_cast<int32_T>(body2->Index);
      cartesian_trajectory_planner_B.bid1 = obj->
        PositionDoFMap[cartesian_trajectory_planner_B.b_kstr - 1];
      cartesian_trajectory_planner_B.qidx_idx_1 = obj->
        PositionDoFMap[cartesian_trajectory_planner_B.b_kstr + 7];
      if (cartesian_trajectory_planner_B.bid1 >
          cartesian_trajectory_planner_B.qidx_idx_1) {
        cartesian_trajectory_planner_B.g = 0;
        cartesian_trajectory_planner_B.b_kstr = -1;
      } else {
        cartesian_trajectory_planner_B.g = static_cast<int32_T>
          (cartesian_trajectory_planner_B.bid1) - 1;
        cartesian_trajectory_planner_B.b_kstr = static_cast<int32_T>
          (cartesian_trajectory_planner_B.qidx_idx_1) - 1;
      }

      cartesian_trajectory_planner_B.loop_ub_k =
        cartesian_trajectory_planner_B.b_kstr - cartesian_trajectory_planner_B.g;
      cartesian_trajectory_planner_B.qv_size =
        cartesian_trajectory_planner_B.loop_ub_k + 1;
      for (cartesian_trajectory_planner_B.b_kstr = 0;
           cartesian_trajectory_planner_B.b_kstr <=
           cartesian_trajectory_planner_B.loop_ub_k;
           cartesian_trajectory_planner_B.b_kstr++) {
        cartesian_trajectory_planner_B.qv_data[cartesian_trajectory_planner_B.b_kstr]
          = qv[cartesian_trajectory_planner_B.g +
          cartesian_trajectory_planner_B.b_kstr];
      }

      rigidBodyJoint_transformBodyT_a(joint,
        cartesian_trajectory_planner_B.qv_data,
        &cartesian_trajectory_planner_B.qv_size,
        cartesian_trajectory_planner_B.Tc2p);
      cartesian_trajectory_planner_B.b_kstr = static_cast<int32_T>(body2->Index);
      cartesian_trajectory_planner_B.bid1 = obj->
        VelocityDoFMap[cartesian_trajectory_planner_B.b_kstr - 1];
      cartesian_trajectory_planner_B.qidx_idx_1 = obj->
        VelocityDoFMap[cartesian_trajectory_planner_B.b_kstr + 7];
      if (cartesian_trajectory_planner_B.nextBodyIsParent) {
        for (cartesian_trajectory_planner_B.b_kstr = 0;
             cartesian_trajectory_planner_B.b_kstr < 16;
             cartesian_trajectory_planner_B.b_kstr++) {
          cartesian_trajectory_planner_B.Tj[cartesian_trajectory_planner_B.b_kstr]
            = joint->ChildToJointTransform[cartesian_trajectory_planner_B.b_kstr];
        }
      } else {
        for (cartesian_trajectory_planner_B.b_kstr = 0;
             cartesian_trajectory_planner_B.b_kstr < 16;
             cartesian_trajectory_planner_B.b_kstr++) {
          cartesian_trajectory_planner_B.T1j[cartesian_trajectory_planner_B.b_kstr]
            = joint->
            JointToParentTransform[cartesian_trajectory_planner_B.b_kstr];
        }

        for (cartesian_trajectory_planner_B.b_kstr = 0;
             cartesian_trajectory_planner_B.b_kstr < 3;
             cartesian_trajectory_planner_B.b_kstr++) {
          cartesian_trajectory_planner_B.R_g[3 *
            cartesian_trajectory_planner_B.b_kstr] =
            cartesian_trajectory_planner_B.T1j[cartesian_trajectory_planner_B.b_kstr];
          cartesian_trajectory_planner_B.R_g[3 *
            cartesian_trajectory_planner_B.b_kstr + 1] =
            cartesian_trajectory_planner_B.T1j[cartesian_trajectory_planner_B.b_kstr
            + 4];
          cartesian_trajectory_planner_B.R_g[3 *
            cartesian_trajectory_planner_B.b_kstr + 2] =
            cartesian_trajectory_planner_B.T1j[cartesian_trajectory_planner_B.b_kstr
            + 8];
        }

        for (cartesian_trajectory_planner_B.b_kstr = 0;
             cartesian_trajectory_planner_B.b_kstr < 9;
             cartesian_trajectory_planner_B.b_kstr++) {
          cartesian_trajectory_planner_B.R_c[cartesian_trajectory_planner_B.b_kstr]
            =
            -cartesian_trajectory_planner_B.R_g[cartesian_trajectory_planner_B.b_kstr];
        }

        for (cartesian_trajectory_planner_B.b_kstr = 0;
             cartesian_trajectory_planner_B.b_kstr < 3;
             cartesian_trajectory_planner_B.b_kstr++) {
          cartesian_trajectory_planner_B.g =
            cartesian_trajectory_planner_B.b_kstr << 2;
          cartesian_trajectory_planner_B.Tj[cartesian_trajectory_planner_B.g] =
            cartesian_trajectory_planner_B.R_g[3 *
            cartesian_trajectory_planner_B.b_kstr];
          cartesian_trajectory_planner_B.Tj[cartesian_trajectory_planner_B.g + 1]
            = cartesian_trajectory_planner_B.R_g[3 *
            cartesian_trajectory_planner_B.b_kstr + 1];
          cartesian_trajectory_planner_B.Tj[cartesian_trajectory_planner_B.g + 2]
            = cartesian_trajectory_planner_B.R_g[3 *
            cartesian_trajectory_planner_B.b_kstr + 2];
          cartesian_trajectory_planner_B.Tj[cartesian_trajectory_planner_B.b_kstr
            + 12] =
            cartesian_trajectory_planner_B.R_c[cartesian_trajectory_planner_B.b_kstr
            + 6] * cartesian_trajectory_planner_B.T1j[14] +
            (cartesian_trajectory_planner_B.R_c[cartesian_trajectory_planner_B.b_kstr
             + 3] * cartesian_trajectory_planner_B.T1j[13] +
             cartesian_trajectory_planner_B.R_c[cartesian_trajectory_planner_B.b_kstr]
             * cartesian_trajectory_planner_B.T1j[12]);
        }

        cartesian_trajectory_planner_B.Tj[3] = 0.0;
        cartesian_trajectory_planner_B.Tj[7] = 0.0;
        cartesian_trajectory_planner_B.Tj[11] = 0.0;
        cartesian_trajectory_planner_B.Tj[15] = 1.0;
      }

      for (cartesian_trajectory_planner_B.b_kstr = 0;
           cartesian_trajectory_planner_B.b_kstr < 4;
           cartesian_trajectory_planner_B.b_kstr++) {
        for (cartesian_trajectory_planner_B.g = 0;
             cartesian_trajectory_planner_B.g < 4;
             cartesian_trajectory_planner_B.g++) {
          cartesian_trajectory_planner_B.loop_ub_k =
            cartesian_trajectory_planner_B.g << 2;
          cartesian_trajectory_planner_B.n_l =
            cartesian_trajectory_planner_B.b_kstr +
            cartesian_trajectory_planner_B.loop_ub_k;
          cartesian_trajectory_planner_B.T1j[cartesian_trajectory_planner_B.n_l]
            = 0.0;
          cartesian_trajectory_planner_B.T1j[cartesian_trajectory_planner_B.n_l]
            +=
            cartesian_trajectory_planner_B.T1[cartesian_trajectory_planner_B.loop_ub_k]
            * cartesian_trajectory_planner_B.Tj[cartesian_trajectory_planner_B.b_kstr];
          cartesian_trajectory_planner_B.T1j[cartesian_trajectory_planner_B.n_l]
            +=
            cartesian_trajectory_planner_B.T1[cartesian_trajectory_planner_B.loop_ub_k
            + 1] *
            cartesian_trajectory_planner_B.Tj[cartesian_trajectory_planner_B.b_kstr
            + 4];
          cartesian_trajectory_planner_B.T1j[cartesian_trajectory_planner_B.n_l]
            +=
            cartesian_trajectory_planner_B.T1[cartesian_trajectory_planner_B.loop_ub_k
            + 2] *
            cartesian_trajectory_planner_B.Tj[cartesian_trajectory_planner_B.b_kstr
            + 8];
          cartesian_trajectory_planner_B.T1j[cartesian_trajectory_planner_B.n_l]
            +=
            cartesian_trajectory_planner_B.T1[cartesian_trajectory_planner_B.loop_ub_k
            + 3] *
            cartesian_trajectory_planner_B.Tj[cartesian_trajectory_planner_B.b_kstr
            + 12];
        }
      }

      for (cartesian_trajectory_planner_B.b_kstr = 0;
           cartesian_trajectory_planner_B.b_kstr < 3;
           cartesian_trajectory_planner_B.b_kstr++) {
        cartesian_trajectory_planner_B.R_g[3 *
          cartesian_trajectory_planner_B.b_kstr] =
          cartesian_trajectory_planner_B.T1j[cartesian_trajectory_planner_B.b_kstr];
        cartesian_trajectory_planner_B.R_g[3 *
          cartesian_trajectory_planner_B.b_kstr + 1] =
          cartesian_trajectory_planner_B.T1j[cartesian_trajectory_planner_B.b_kstr
          + 4];
        cartesian_trajectory_planner_B.R_g[3 *
          cartesian_trajectory_planner_B.b_kstr + 2] =
          cartesian_trajectory_planner_B.T1j[cartesian_trajectory_planner_B.b_kstr
          + 8];
      }

      for (cartesian_trajectory_planner_B.b_kstr = 0;
           cartesian_trajectory_planner_B.b_kstr < 9;
           cartesian_trajectory_planner_B.b_kstr++) {
        cartesian_trajectory_planner_B.R_c[cartesian_trajectory_planner_B.b_kstr]
          =
          -cartesian_trajectory_planner_B.R_g[cartesian_trajectory_planner_B.b_kstr];
      }

      for (cartesian_trajectory_planner_B.b_kstr = 0;
           cartesian_trajectory_planner_B.b_kstr < 3;
           cartesian_trajectory_planner_B.b_kstr++) {
        cartesian_trajectory_planner_B.g = cartesian_trajectory_planner_B.b_kstr
          << 2;
        cartesian_trajectory_planner_B.Tj[cartesian_trajectory_planner_B.g] =
          cartesian_trajectory_planner_B.R_g[3 *
          cartesian_trajectory_planner_B.b_kstr];
        cartesian_trajectory_planner_B.Tj[cartesian_trajectory_planner_B.g + 1] =
          cartesian_trajectory_planner_B.R_g[3 *
          cartesian_trajectory_planner_B.b_kstr + 1];
        cartesian_trajectory_planner_B.Tj[cartesian_trajectory_planner_B.g + 2] =
          cartesian_trajectory_planner_B.R_g[3 *
          cartesian_trajectory_planner_B.b_kstr + 2];
        cartesian_trajectory_planner_B.Tj[cartesian_trajectory_planner_B.b_kstr
          + 12] =
          cartesian_trajectory_planner_B.R_c[cartesian_trajectory_planner_B.b_kstr
          + 6] * cartesian_trajectory_planner_B.T1j[14] +
          (cartesian_trajectory_planner_B.R_c[cartesian_trajectory_planner_B.b_kstr
           + 3] * cartesian_trajectory_planner_B.T1j[13] +
           cartesian_trajectory_planner_B.R_c[cartesian_trajectory_planner_B.b_kstr]
           * cartesian_trajectory_planner_B.T1j[12]);
      }

      cartesian_trajectory_planner_B.Tj[3] = 0.0;
      cartesian_trajectory_planner_B.Tj[7] = 0.0;
      cartesian_trajectory_planner_B.Tj[11] = 0.0;
      cartesian_trajectory_planner_B.Tj[15] = 1.0;
      cartesian_trajectory_planner_B.R_g[0] = 0.0;
      cartesian_trajectory_planner_B.R_g[3] =
        -cartesian_trajectory_planner_B.Tj[14];
      cartesian_trajectory_planner_B.R_g[6] = cartesian_trajectory_planner_B.Tj
        [13];
      cartesian_trajectory_planner_B.R_g[1] = cartesian_trajectory_planner_B.Tj
        [14];
      cartesian_trajectory_planner_B.R_g[4] = 0.0;
      cartesian_trajectory_planner_B.R_g[7] =
        -cartesian_trajectory_planner_B.Tj[12];
      cartesian_trajectory_planner_B.R_g[2] =
        -cartesian_trajectory_planner_B.Tj[13];
      cartesian_trajectory_planner_B.R_g[5] = cartesian_trajectory_planner_B.Tj
        [12];
      cartesian_trajectory_planner_B.R_g[8] = 0.0;
      for (cartesian_trajectory_planner_B.b_kstr = 0;
           cartesian_trajectory_planner_B.b_kstr < 3;
           cartesian_trajectory_planner_B.b_kstr++) {
        for (cartesian_trajectory_planner_B.g = 0;
             cartesian_trajectory_planner_B.g < 3;
             cartesian_trajectory_planner_B.g++) {
          cartesian_trajectory_planner_B.loop_ub_k =
            cartesian_trajectory_planner_B.b_kstr + 3 *
            cartesian_trajectory_planner_B.g;
          cartesian_trajectory_planner_B.R_c[cartesian_trajectory_planner_B.loop_ub_k]
            = 0.0;
          cartesian_trajectory_planner_B.n_l = cartesian_trajectory_planner_B.g <<
            2;
          cartesian_trajectory_planner_B.R_c[cartesian_trajectory_planner_B.loop_ub_k]
            +=
            cartesian_trajectory_planner_B.Tj[cartesian_trajectory_planner_B.n_l]
            * cartesian_trajectory_planner_B.R_g[cartesian_trajectory_planner_B.b_kstr];
          cartesian_trajectory_planner_B.R_c[cartesian_trajectory_planner_B.loop_ub_k]
            +=
            cartesian_trajectory_planner_B.Tj[cartesian_trajectory_planner_B.n_l
            + 1] *
            cartesian_trajectory_planner_B.R_g[cartesian_trajectory_planner_B.b_kstr
            + 3];
          cartesian_trajectory_planner_B.R_c[cartesian_trajectory_planner_B.loop_ub_k]
            +=
            cartesian_trajectory_planner_B.Tj[cartesian_trajectory_planner_B.n_l
            + 2] *
            cartesian_trajectory_planner_B.R_g[cartesian_trajectory_planner_B.b_kstr
            + 6];
          cartesian_trajectory_planner_B.X[cartesian_trajectory_planner_B.g + 6 *
            cartesian_trajectory_planner_B.b_kstr] =
            cartesian_trajectory_planner_B.Tj
            [(cartesian_trajectory_planner_B.b_kstr << 2) +
            cartesian_trajectory_planner_B.g];
          cartesian_trajectory_planner_B.X[cartesian_trajectory_planner_B.g + 6 *
            (cartesian_trajectory_planner_B.b_kstr + 3)] = 0.0;
        }
      }

      for (cartesian_trajectory_planner_B.b_kstr = 0;
           cartesian_trajectory_planner_B.b_kstr < 3;
           cartesian_trajectory_planner_B.b_kstr++) {
        cartesian_trajectory_planner_B.X[6 *
          cartesian_trajectory_planner_B.b_kstr + 3] =
          cartesian_trajectory_planner_B.R_c[3 *
          cartesian_trajectory_planner_B.b_kstr];
        cartesian_trajectory_planner_B.g = cartesian_trajectory_planner_B.b_kstr
          << 2;
        cartesian_trajectory_planner_B.loop_ub_k = 6 *
          (cartesian_trajectory_planner_B.b_kstr + 3);
        cartesian_trajectory_planner_B.X[cartesian_trajectory_planner_B.loop_ub_k
          + 3] =
          cartesian_trajectory_planner_B.Tj[cartesian_trajectory_planner_B.g];
        cartesian_trajectory_planner_B.X[6 *
          cartesian_trajectory_planner_B.b_kstr + 4] =
          cartesian_trajectory_planner_B.R_c[3 *
          cartesian_trajectory_planner_B.b_kstr + 1];
        cartesian_trajectory_planner_B.X[cartesian_trajectory_planner_B.loop_ub_k
          + 4] =
          cartesian_trajectory_planner_B.Tj[cartesian_trajectory_planner_B.g + 1];
        cartesian_trajectory_planner_B.X[6 *
          cartesian_trajectory_planner_B.b_kstr + 5] =
          cartesian_trajectory_planner_B.R_c[3 *
          cartesian_trajectory_planner_B.b_kstr + 2];
        cartesian_trajectory_planner_B.X[cartesian_trajectory_planner_B.loop_ub_k
          + 5] =
          cartesian_trajectory_planner_B.Tj[cartesian_trajectory_planner_B.g + 2];
      }

      cartesian_trajectory_planner_B.b_kstr = b->size[0] * b->size[1];
      b->size[0] = 6;
      b->size[1] = joint->MotionSubspace->size[1];
      cartes_emxEnsureCapacity_real_T(b, cartesian_trajectory_planner_B.b_kstr);
      cartesian_trajectory_planner_B.loop_ub_k = joint->MotionSubspace->size[0] *
        joint->MotionSubspace->size[1] - 1;
      for (cartesian_trajectory_planner_B.b_kstr = 0;
           cartesian_trajectory_planner_B.b_kstr <=
           cartesian_trajectory_planner_B.loop_ub_k;
           cartesian_trajectory_planner_B.b_kstr++) {
        b->data[cartesian_trajectory_planner_B.b_kstr] = joint->
          MotionSubspace->data[cartesian_trajectory_planner_B.b_kstr];
      }

      cartesian_trajectory_planner_B.n_l = b->size[1] - 1;
      cartesian_trajectory_planner_B.b_kstr = y->size[0] * y->size[1];
      y->size[0] = 6;
      y->size[1] = b->size[1];
      cartes_emxEnsureCapacity_real_T(y, cartesian_trajectory_planner_B.b_kstr);
      for (cartesian_trajectory_planner_B.b_kstr = 0;
           cartesian_trajectory_planner_B.b_kstr <=
           cartesian_trajectory_planner_B.n_l;
           cartesian_trajectory_planner_B.b_kstr++) {
        cartesian_trajectory_planner_B.coffset_tmp =
          cartesian_trajectory_planner_B.b_kstr * 6 - 1;
        for (cartesian_trajectory_planner_B.g = 0;
             cartesian_trajectory_planner_B.g < 6;
             cartesian_trajectory_planner_B.g++) {
          cartesian_trajectory_planner_B.bid2 = 0.0;
          for (cartesian_trajectory_planner_B.loop_ub_k = 0;
               cartesian_trajectory_planner_B.loop_ub_k < 6;
               cartesian_trajectory_planner_B.loop_ub_k++) {
            cartesian_trajectory_planner_B.bid2 +=
              cartesian_trajectory_planner_B.X[cartesian_trajectory_planner_B.loop_ub_k
              * 6 + cartesian_trajectory_planner_B.g] * b->data
              [(cartesian_trajectory_planner_B.coffset_tmp +
                cartesian_trajectory_planner_B.loop_ub_k) + 1];
          }

          y->data[(cartesian_trajectory_planner_B.coffset_tmp +
                   cartesian_trajectory_planner_B.g) + 1] =
            cartesian_trajectory_planner_B.bid2;
        }
      }

      if (cartesian_trajectory_planner_B.bid1 >
          cartesian_trajectory_planner_B.qidx_idx_1) {
        cartesian_trajectory_planner_B.n_l = 0;
      } else {
        cartesian_trajectory_planner_B.n_l = static_cast<int32_T>
          (cartesian_trajectory_planner_B.bid1) - 1;
      }

      cartesian_trajectory_planner_B.loop_ub_k = y->size[1];
      for (cartesian_trajectory_planner_B.b_kstr = 0;
           cartesian_trajectory_planner_B.b_kstr <
           cartesian_trajectory_planner_B.loop_ub_k;
           cartesian_trajectory_planner_B.b_kstr++) {
        for (cartesian_trajectory_planner_B.g = 0;
             cartesian_trajectory_planner_B.g < 6;
             cartesian_trajectory_planner_B.g++) {
          Jac->data[cartesian_trajectory_planner_B.g + 6 *
            (cartesian_trajectory_planner_B.n_l +
             cartesian_trajectory_planner_B.b_kstr)] = y->data[6 *
            cartesian_trajectory_planner_B.b_kstr +
            cartesian_trajectory_planner_B.g] * static_cast<real_T>
            (cartesian_trajectory_planner_B.jointSign);
        }
      }
    }

    if (cartesian_trajectory_planner_B.nextBodyIsParent) {
      for (cartesian_trajectory_planner_B.b_kstr = 0;
           cartesian_trajectory_planner_B.b_kstr < 4;
           cartesian_trajectory_planner_B.b_kstr++) {
        for (cartesian_trajectory_planner_B.g = 0;
             cartesian_trajectory_planner_B.g < 4;
             cartesian_trajectory_planner_B.g++) {
          cartesian_trajectory_planner_B.loop_ub_k =
            cartesian_trajectory_planner_B.g << 2;
          cartesian_trajectory_planner_B.jointSign =
            cartesian_trajectory_planner_B.b_kstr +
            cartesian_trajectory_planner_B.loop_ub_k;
          cartesian_trajectory_planner_B.T1j[cartesian_trajectory_planner_B.jointSign]
            = 0.0;
          cartesian_trajectory_planner_B.T1j[cartesian_trajectory_planner_B.jointSign]
            +=
            cartesian_trajectory_planner_B.T1[cartesian_trajectory_planner_B.loop_ub_k]
            * cartesian_trajectory_planner_B.Tc2p[cartesian_trajectory_planner_B.b_kstr];
          cartesian_trajectory_planner_B.T1j[cartesian_trajectory_planner_B.jointSign]
            +=
            cartesian_trajectory_planner_B.T1[cartesian_trajectory_planner_B.loop_ub_k
            + 1] *
            cartesian_trajectory_planner_B.Tc2p[cartesian_trajectory_planner_B.b_kstr
            + 4];
          cartesian_trajectory_planner_B.T1j[cartesian_trajectory_planner_B.jointSign]
            +=
            cartesian_trajectory_planner_B.T1[cartesian_trajectory_planner_B.loop_ub_k
            + 2] *
            cartesian_trajectory_planner_B.Tc2p[cartesian_trajectory_planner_B.b_kstr
            + 8];
          cartesian_trajectory_planner_B.T1j[cartesian_trajectory_planner_B.jointSign]
            +=
            cartesian_trajectory_planner_B.T1[cartesian_trajectory_planner_B.loop_ub_k
            + 3] *
            cartesian_trajectory_planner_B.Tc2p[cartesian_trajectory_planner_B.b_kstr
            + 12];
        }
      }

      memcpy(&cartesian_trajectory_planner_B.T1[0],
             &cartesian_trajectory_planner_B.T1j[0], sizeof(real_T) << 4U);
    } else {
      for (cartesian_trajectory_planner_B.b_kstr = 0;
           cartesian_trajectory_planner_B.b_kstr < 3;
           cartesian_trajectory_planner_B.b_kstr++) {
        cartesian_trajectory_planner_B.R_g[3 *
          cartesian_trajectory_planner_B.b_kstr] =
          cartesian_trajectory_planner_B.Tc2p[cartesian_trajectory_planner_B.b_kstr];
        cartesian_trajectory_planner_B.R_g[3 *
          cartesian_trajectory_planner_B.b_kstr + 1] =
          cartesian_trajectory_planner_B.Tc2p[cartesian_trajectory_planner_B.b_kstr
          + 4];
        cartesian_trajectory_planner_B.R_g[3 *
          cartesian_trajectory_planner_B.b_kstr + 2] =
          cartesian_trajectory_planner_B.Tc2p[cartesian_trajectory_planner_B.b_kstr
          + 8];
      }

      for (cartesian_trajectory_planner_B.b_kstr = 0;
           cartesian_trajectory_planner_B.b_kstr < 9;
           cartesian_trajectory_planner_B.b_kstr++) {
        cartesian_trajectory_planner_B.R_c[cartesian_trajectory_planner_B.b_kstr]
          =
          -cartesian_trajectory_planner_B.R_g[cartesian_trajectory_planner_B.b_kstr];
      }

      for (cartesian_trajectory_planner_B.b_kstr = 0;
           cartesian_trajectory_planner_B.b_kstr < 3;
           cartesian_trajectory_planner_B.b_kstr++) {
        cartesian_trajectory_planner_B.loop_ub_k =
          cartesian_trajectory_planner_B.b_kstr << 2;
        cartesian_trajectory_planner_B.T1j[cartesian_trajectory_planner_B.loop_ub_k]
          = cartesian_trajectory_planner_B.R_g[3 *
          cartesian_trajectory_planner_B.b_kstr];
        cartesian_trajectory_planner_B.T1j[cartesian_trajectory_planner_B.loop_ub_k
          + 1] = cartesian_trajectory_planner_B.R_g[3 *
          cartesian_trajectory_planner_B.b_kstr + 1];
        cartesian_trajectory_planner_B.T1j[cartesian_trajectory_planner_B.loop_ub_k
          + 2] = cartesian_trajectory_planner_B.R_g[3 *
          cartesian_trajectory_planner_B.b_kstr + 2];
        cartesian_trajectory_planner_B.T1j[cartesian_trajectory_planner_B.b_kstr
          + 12] =
          cartesian_trajectory_planner_B.R_c[cartesian_trajectory_planner_B.b_kstr
          + 6] * cartesian_trajectory_planner_B.Tc2p[14] +
          (cartesian_trajectory_planner_B.R_c[cartesian_trajectory_planner_B.b_kstr
           + 3] * cartesian_trajectory_planner_B.Tc2p[13] +
           cartesian_trajectory_planner_B.R_c[cartesian_trajectory_planner_B.b_kstr]
           * cartesian_trajectory_planner_B.Tc2p[12]);
      }

      cartesian_trajectory_planner_B.T1j[3] = 0.0;
      cartesian_trajectory_planner_B.T1j[7] = 0.0;
      cartesian_trajectory_planner_B.T1j[11] = 0.0;
      cartesian_trajectory_planner_B.T1j[15] = 1.0;
      for (cartesian_trajectory_planner_B.b_kstr = 0;
           cartesian_trajectory_planner_B.b_kstr < 4;
           cartesian_trajectory_planner_B.b_kstr++) {
        for (cartesian_trajectory_planner_B.g = 0;
             cartesian_trajectory_planner_B.g < 4;
             cartesian_trajectory_planner_B.g++) {
          cartesian_trajectory_planner_B.jointSign =
            cartesian_trajectory_planner_B.g << 2;
          cartesian_trajectory_planner_B.loop_ub_k =
            cartesian_trajectory_planner_B.b_kstr +
            cartesian_trajectory_planner_B.jointSign;
          cartesian_trajectory_planner_B.Tc2p[cartesian_trajectory_planner_B.loop_ub_k]
            = 0.0;
          cartesian_trajectory_planner_B.Tc2p[cartesian_trajectory_planner_B.loop_ub_k]
            +=
            cartesian_trajectory_planner_B.T1[cartesian_trajectory_planner_B.jointSign]
            * cartesian_trajectory_planner_B.T1j[cartesian_trajectory_planner_B.b_kstr];
          cartesian_trajectory_planner_B.Tc2p[cartesian_trajectory_planner_B.loop_ub_k]
            +=
            cartesian_trajectory_planner_B.T1[cartesian_trajectory_planner_B.jointSign
            + 1] *
            cartesian_trajectory_planner_B.T1j[cartesian_trajectory_planner_B.b_kstr
            + 4];
          cartesian_trajectory_planner_B.Tc2p[cartesian_trajectory_planner_B.loop_ub_k]
            +=
            cartesian_trajectory_planner_B.T1[cartesian_trajectory_planner_B.jointSign
            + 2] *
            cartesian_trajectory_planner_B.T1j[cartesian_trajectory_planner_B.b_kstr
            + 8];
          cartesian_trajectory_planner_B.Tc2p[cartesian_trajectory_planner_B.loop_ub_k]
            +=
            cartesian_trajectory_planner_B.T1[cartesian_trajectory_planner_B.jointSign
            + 3] *
            cartesian_trajectory_planner_B.T1j[cartesian_trajectory_planner_B.b_kstr
            + 12];
        }
      }

      memcpy(&cartesian_trajectory_planner_B.T1[0],
             &cartesian_trajectory_planner_B.Tc2p[0], sizeof(real_T) << 4U);
    }
  }

  cartesian_trajec_emxFree_real_T(&b);
  cartesian_trajec_emxFree_char_T(&body2Name);
  cartesian_trajec_emxFree_real_T(&kinematicPathIndices);
  for (cartesian_trajectory_planner_B.b_kstr = 0;
       cartesian_trajectory_planner_B.b_kstr < 3;
       cartesian_trajectory_planner_B.b_kstr++) {
    cartesian_trajectory_planner_B.b_i_d = cartesian_trajectory_planner_B.b_kstr
      << 2;
    cartesian_trajectory_planner_B.X[6 * cartesian_trajectory_planner_B.b_kstr] =
      cartesian_trajectory_planner_B.T1[cartesian_trajectory_planner_B.b_i_d];
    cartesian_trajectory_planner_B.g = 6 *
      (cartesian_trajectory_planner_B.b_kstr + 3);
    cartesian_trajectory_planner_B.X[cartesian_trajectory_planner_B.g] = 0.0;
    cartesian_trajectory_planner_B.X[6 * cartesian_trajectory_planner_B.b_kstr +
      3] = 0.0;
    cartesian_trajectory_planner_B.X[cartesian_trajectory_planner_B.g + 3] =
      cartesian_trajectory_planner_B.T1[cartesian_trajectory_planner_B.b_i_d];
    cartesian_trajectory_planner_B.bid1 =
      cartesian_trajectory_planner_B.T1[cartesian_trajectory_planner_B.b_i_d + 1];
    cartesian_trajectory_planner_B.X[6 * cartesian_trajectory_planner_B.b_kstr +
      1] = cartesian_trajectory_planner_B.bid1;
    cartesian_trajectory_planner_B.X[cartesian_trajectory_planner_B.g + 1] = 0.0;
    cartesian_trajectory_planner_B.X[6 * cartesian_trajectory_planner_B.b_kstr +
      4] = 0.0;
    cartesian_trajectory_planner_B.X[cartesian_trajectory_planner_B.g + 4] =
      cartesian_trajectory_planner_B.bid1;
    cartesian_trajectory_planner_B.bid1 =
      cartesian_trajectory_planner_B.T1[cartesian_trajectory_planner_B.b_i_d + 2];
    cartesian_trajectory_planner_B.X[6 * cartesian_trajectory_planner_B.b_kstr +
      2] = cartesian_trajectory_planner_B.bid1;
    cartesian_trajectory_planner_B.X[cartesian_trajectory_planner_B.g + 2] = 0.0;
    cartesian_trajectory_planner_B.X[6 * cartesian_trajectory_planner_B.b_kstr +
      5] = 0.0;
    cartesian_trajectory_planner_B.X[cartesian_trajectory_planner_B.g + 5] =
      cartesian_trajectory_planner_B.bid1;
  }

  cartesian_trajectory_planner_B.n_l = Jac->size[1];
  cartesian_trajectory_planner_B.b_kstr = y->size[0] * y->size[1];
  y->size[0] = 6;
  y->size[1] = Jac->size[1];
  cartes_emxEnsureCapacity_real_T(y, cartesian_trajectory_planner_B.b_kstr);
  cartesian_trajectory_planner_B.loop_ub_k = Jac->size[0] * Jac->size[1] - 1;
  for (cartesian_trajectory_planner_B.b_kstr = 0;
       cartesian_trajectory_planner_B.b_kstr <=
       cartesian_trajectory_planner_B.loop_ub_k;
       cartesian_trajectory_planner_B.b_kstr++) {
    y->data[cartesian_trajectory_planner_B.b_kstr] = Jac->
      data[cartesian_trajectory_planner_B.b_kstr];
  }

  cartesian_trajectory_planner_B.b_kstr = Jac->size[0] * Jac->size[1];
  Jac->size[0] = 6;
  Jac->size[1] = cartesian_trajectory_planner_B.n_l;
  cartes_emxEnsureCapacity_real_T(Jac, cartesian_trajectory_planner_B.b_kstr);
  for (cartesian_trajectory_planner_B.b_kstr = 0;
       cartesian_trajectory_planner_B.b_kstr <
       cartesian_trajectory_planner_B.n_l; cartesian_trajectory_planner_B.b_kstr
       ++) {
    cartesian_trajectory_planner_B.coffset_tmp =
      cartesian_trajectory_planner_B.b_kstr * 6 - 1;
    for (cartesian_trajectory_planner_B.b_i_d = 0;
         cartesian_trajectory_planner_B.b_i_d < 6;
         cartesian_trajectory_planner_B.b_i_d++) {
      cartesian_trajectory_planner_B.bid2 = 0.0;
      for (cartesian_trajectory_planner_B.loop_ub_k = 0;
           cartesian_trajectory_planner_B.loop_ub_k < 6;
           cartesian_trajectory_planner_B.loop_ub_k++) {
        cartesian_trajectory_planner_B.bid2 +=
          cartesian_trajectory_planner_B.X[cartesian_trajectory_planner_B.loop_ub_k
          * 6 + cartesian_trajectory_planner_B.b_i_d] * y->data
          [(cartesian_trajectory_planner_B.coffset_tmp +
            cartesian_trajectory_planner_B.loop_ub_k) + 1];
      }

      Jac->data[(cartesian_trajectory_planner_B.coffset_tmp +
                 cartesian_trajectory_planner_B.b_i_d) + 1] =
        cartesian_trajectory_planner_B.bid2;
    }
  }

  cartesian_trajec_emxFree_real_T(&y);
  T_size[0] = 4;
  T_size[1] = 4;
  memcpy(&T_data[0], &cartesian_trajectory_planner_B.T1[0], sizeof(real_T) << 4U);
}

real_T rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T y;
  real_T a;
  a = fabs(u0);
  y = fabs(u1);
  if (a < y) {
    a /= y;
    y *= sqrt(a * a + 1.0);
  } else if (a > y) {
    y /= a;
    y = sqrt(y * y + 1.0) * a;
  } else {
    if (!rtIsNaN(y)) {
      y = a * 1.4142135623730951;
    }
  }

  return y;
}

static creal_T cartesian_trajectory_plann_sqrt(const creal_T x)
{
  creal_T b_x;
  real_T absxr;
  real_T absxi;
  if (x.im == 0.0) {
    if (x.re < 0.0) {
      absxr = 0.0;
      absxi = sqrt(-x.re);
    } else {
      absxr = sqrt(x.re);
      absxi = 0.0;
    }
  } else if (x.re == 0.0) {
    if (x.im < 0.0) {
      absxr = sqrt(-x.im / 2.0);
      absxi = -absxr;
    } else {
      absxr = sqrt(x.im / 2.0);
      absxi = absxr;
    }
  } else if (rtIsNaN(x.re)) {
    absxr = x.re;
    absxi = x.re;
  } else if (rtIsNaN(x.im)) {
    absxr = x.im;
    absxi = x.im;
  } else if (rtIsInf(x.im)) {
    absxr = fabs(x.im);
    absxi = x.im;
  } else if (rtIsInf(x.re)) {
    if (x.re < 0.0) {
      absxr = 0.0;
      absxi = x.im * -x.re;
    } else {
      absxr = x.re;
      absxi = 0.0;
    }
  } else {
    absxr = fabs(x.re);
    absxi = fabs(x.im);
    if ((absxr > 4.4942328371557893E+307) || (absxi > 4.4942328371557893E+307))
    {
      absxr *= 0.5;
      absxi *= 0.5;
      absxi = rt_hypotd_snf(absxr, absxi);
      if (absxi > absxr) {
        absxr = sqrt(absxr / absxi + 1.0) * sqrt(absxi);
      } else {
        absxr = sqrt(absxi) * 1.4142135623730951;
      }
    } else {
      absxr = sqrt((rt_hypotd_snf(absxr, absxi) + absxr) * 0.5);
    }

    if (x.re > 0.0) {
      absxi = x.im / absxr * 0.5;
    } else {
      if (x.im < 0.0) {
        absxi = -absxr;
      } else {
        absxi = absxr;
      }

      absxr = x.im / absxi * 0.5;
    }
  }

  b_x.re = absxr;
  b_x.im = absxi;
  return b_x;
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

static real_T cartesian_trajectory_plan_xnrm2(int32_T n, const real_T x[9],
  int32_T ix0)
{
  real_T y;
  real_T scale;
  int32_T kend;
  real_T absxk;
  real_T t;
  int32_T k;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  kend = ix0 + n;
  for (k = ix0; k < kend; k++) {
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

  return scale * sqrt(y);
}

static real_T cartesian_trajectory_plan_xdotc(int32_T n, const real_T x[9],
  int32_T ix0, const real_T y[9], int32_T iy0)
{
  real_T d;
  int32_T ix;
  int32_T iy;
  int32_T k;
  d = 0.0;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < n; k++) {
    d += x[ix] * y[iy];
    ix++;
    iy++;
  }

  return d;
}

static void cartesian_trajectory_plan_xaxpy(int32_T n, real_T a, int32_T ix0,
  const real_T y[9], int32_T iy0, real_T b_y[9])
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  memcpy(&b_y[0], &y[0], 9U * sizeof(real_T));
  if (!(a == 0.0)) {
    ix = ix0;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      b_y[iy] += b_y[ix - 1] * a;
      ix++;
      iy++;
    }
  }
}

static real_T cartesian_trajectory_pl_xnrm2_a(const real_T x[3], int32_T ix0)
{
  real_T y;
  real_T scale;
  real_T absxk;
  real_T t;
  int32_T k;
  y = 0.0;
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

  return scale * sqrt(y);
}

static void cartesian_trajectory__xaxpy_ast(int32_T n, real_T a, const real_T x
  [9], int32_T ix0, real_T y[3], int32_T iy0)
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  if (!(a == 0.0)) {
    ix = ix0;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += x[ix - 1] * a;
      ix++;
      iy++;
    }
  }
}

static void cartesian_trajectory_p_xaxpy_as(int32_T n, real_T a, const real_T x
  [3], int32_T ix0, const real_T y[9], int32_T iy0, real_T b_y[9])
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  memcpy(&b_y[0], &y[0], 9U * sizeof(real_T));
  if (!(a == 0.0)) {
    ix = ix0;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      b_y[iy] += x[ix - 1] * a;
      ix++;
      iy++;
    }
  }
}

static void cartesian_trajectory_plan_xswap(const real_T x[9], int32_T ix0,
  int32_T iy0, real_T b_x[9])
{
  int32_T ix;
  int32_T iy;
  real_T temp;
  memcpy(&b_x[0], &x[0], 9U * sizeof(real_T));
  ix = ix0 - 1;
  iy = iy0 - 1;
  temp = b_x[ix];
  b_x[ix] = b_x[iy];
  b_x[iy] = temp;
  ix++;
  iy++;
  temp = b_x[ix];
  b_x[ix] = b_x[iy];
  b_x[iy] = temp;
  ix++;
  iy++;
  temp = b_x[ix];
  b_x[ix] = b_x[iy];
  b_x[iy] = temp;
}

static void cartesian_trajectory_plan_xrotg(real_T a, real_T b, real_T *b_a,
  real_T *b_b, real_T *c, real_T *s)
{
  cartesian_trajectory_planner_B.roe = b;
  cartesian_trajectory_planner_B.absa = fabs(a);
  cartesian_trajectory_planner_B.absb = fabs(b);
  if (cartesian_trajectory_planner_B.absa > cartesian_trajectory_planner_B.absb)
  {
    cartesian_trajectory_planner_B.roe = a;
  }

  cartesian_trajectory_planner_B.scale = cartesian_trajectory_planner_B.absa +
    cartesian_trajectory_planner_B.absb;
  if (cartesian_trajectory_planner_B.scale == 0.0) {
    *s = 0.0;
    *c = 1.0;
    *b_a = 0.0;
    *b_b = 0.0;
  } else {
    cartesian_trajectory_planner_B.ads = cartesian_trajectory_planner_B.absa /
      cartesian_trajectory_planner_B.scale;
    cartesian_trajectory_planner_B.bds = cartesian_trajectory_planner_B.absb /
      cartesian_trajectory_planner_B.scale;
    *b_a = sqrt(cartesian_trajectory_planner_B.ads *
                cartesian_trajectory_planner_B.ads +
                cartesian_trajectory_planner_B.bds *
                cartesian_trajectory_planner_B.bds) *
      cartesian_trajectory_planner_B.scale;
    if (cartesian_trajectory_planner_B.roe < 0.0) {
      *b_a = -*b_a;
    }

    *c = a / *b_a;
    *s = b / *b_a;
    if (cartesian_trajectory_planner_B.absa >
        cartesian_trajectory_planner_B.absb) {
      *b_b = *s;
    } else if (*c != 0.0) {
      *b_b = 1.0 / *c;
    } else {
      *b_b = 1.0;
    }
  }
}

static void cartesian_trajectory_plann_xrot(const real_T x[9], int32_T ix0,
  int32_T iy0, real_T c, real_T s, real_T b_x[9])
{
  int32_T ix;
  int32_T iy;
  real_T temp;
  memcpy(&b_x[0], &x[0], 9U * sizeof(real_T));
  ix = ix0 - 1;
  iy = iy0 - 1;
  temp = c * b_x[ix] + s * b_x[iy];
  b_x[iy] = c * b_x[iy] - s * b_x[ix];
  b_x[ix] = temp;
  iy++;
  ix++;
  temp = c * b_x[ix] + s * b_x[iy];
  b_x[iy] = c * b_x[iy] - s * b_x[ix];
  b_x[ix] = temp;
  iy++;
  ix++;
  temp = c * b_x[ix] + s * b_x[iy];
  b_x[iy] = c * b_x[iy] - s * b_x[ix];
  b_x[ix] = temp;
}

static void cartesian_trajectory_planne_svd(const real_T A[9], real_T U[9],
  real_T s[3], real_T V[9])
{
  int32_T qq;
  boolean_T apply_transform;
  int32_T qjj;
  int32_T m;
  int32_T kase;
  int32_T c_q;
  int32_T d_k;
  boolean_T exitg1;
  cartesian_trajectory_planner_B.e_d[0] = 0.0;
  cartesian_trajectory_planner_B.work[0] = 0.0;
  cartesian_trajectory_planner_B.e_d[1] = 0.0;
  cartesian_trajectory_planner_B.work[1] = 0.0;
  cartesian_trajectory_planner_B.e_d[2] = 0.0;
  cartesian_trajectory_planner_B.work[2] = 0.0;
  for (m = 0; m < 9; m++) {
    cartesian_trajectory_planner_B.A[m] = A[m];
    U[m] = 0.0;
    V[m] = 0.0;
  }

  apply_transform = false;
  cartesian_trajectory_planner_B.nrm = cartesian_trajectory_plan_xnrm2(3,
    cartesian_trajectory_planner_B.A, 1);
  if (cartesian_trajectory_planner_B.nrm > 0.0) {
    apply_transform = true;
    if (cartesian_trajectory_planner_B.A[0] < 0.0) {
      cartesian_trajectory_planner_B.s[0] = -cartesian_trajectory_planner_B.nrm;
    } else {
      cartesian_trajectory_planner_B.s[0] = cartesian_trajectory_planner_B.nrm;
    }

    if (fabs(cartesian_trajectory_planner_B.s[0]) >= 1.0020841800044864E-292) {
      cartesian_trajectory_planner_B.nrm = 1.0 /
        cartesian_trajectory_planner_B.s[0];
      for (qq = 1; qq < 4; qq++) {
        cartesian_trajectory_planner_B.A[qq - 1] *=
          cartesian_trajectory_planner_B.nrm;
      }
    } else {
      for (qq = 1; qq < 4; qq++) {
        cartesian_trajectory_planner_B.A[qq - 1] /=
          cartesian_trajectory_planner_B.s[0];
      }
    }

    cartesian_trajectory_planner_B.A[0]++;
    cartesian_trajectory_planner_B.s[0] = -cartesian_trajectory_planner_B.s[0];
  } else {
    cartesian_trajectory_planner_B.s[0] = 0.0;
  }

  for (m = 2; m < 4; m++) {
    qjj = (m - 1) * 3 + 1;
    if (apply_transform) {
      memcpy(&cartesian_trajectory_planner_B.A_h[0],
             &cartesian_trajectory_planner_B.A[0], 9U * sizeof(real_T));
      cartesian_trajectory_plan_xaxpy(3, -(cartesian_trajectory_plan_xdotc(3,
        cartesian_trajectory_planner_B.A, 1, cartesian_trajectory_planner_B.A,
        qjj) / cartesian_trajectory_planner_B.A[0]), 1,
        cartesian_trajectory_planner_B.A_h, qjj,
        cartesian_trajectory_planner_B.A);
    }

    cartesian_trajectory_planner_B.e_d[m - 1] =
      cartesian_trajectory_planner_B.A[qjj - 1];
  }

  for (m = 1; m < 4; m++) {
    U[m - 1] = cartesian_trajectory_planner_B.A[m - 1];
  }

  cartesian_trajectory_planner_B.nrm = cartesian_trajectory_pl_xnrm2_a
    (cartesian_trajectory_planner_B.e_d, 2);
  if (cartesian_trajectory_planner_B.nrm == 0.0) {
    cartesian_trajectory_planner_B.e_d[0] = 0.0;
  } else {
    if (cartesian_trajectory_planner_B.e_d[1] < 0.0) {
      cartesian_trajectory_planner_B.rt = -cartesian_trajectory_planner_B.nrm;
      cartesian_trajectory_planner_B.e_d[0] =
        -cartesian_trajectory_planner_B.nrm;
    } else {
      cartesian_trajectory_planner_B.rt = cartesian_trajectory_planner_B.nrm;
      cartesian_trajectory_planner_B.e_d[0] = cartesian_trajectory_planner_B.nrm;
    }

    if (fabs(cartesian_trajectory_planner_B.rt) >= 1.0020841800044864E-292) {
      cartesian_trajectory_planner_B.nrm = 1.0 /
        cartesian_trajectory_planner_B.rt;
      for (qq = 2; qq < 4; qq++) {
        cartesian_trajectory_planner_B.e_d[qq - 1] *=
          cartesian_trajectory_planner_B.nrm;
      }
    } else {
      for (qq = 2; qq < 4; qq++) {
        cartesian_trajectory_planner_B.e_d[qq - 1] /=
          cartesian_trajectory_planner_B.rt;
      }
    }

    cartesian_trajectory_planner_B.e_d[1]++;
    cartesian_trajectory_planner_B.e_d[0] = -cartesian_trajectory_planner_B.e_d
      [0];
    for (m = 2; m < 4; m++) {
      cartesian_trajectory_planner_B.work[m - 1] = 0.0;
    }

    for (m = 2; m < 4; m++) {
      cartesian_trajectory__xaxpy_ast(2, cartesian_trajectory_planner_B.e_d[m -
        1], cartesian_trajectory_planner_B.A, 3 * (m - 1) + 2,
        cartesian_trajectory_planner_B.work, 2);
    }

    for (m = 2; m < 4; m++) {
      memcpy(&cartesian_trajectory_planner_B.A_h[0],
             &cartesian_trajectory_planner_B.A[0], 9U * sizeof(real_T));
      cartesian_trajectory_p_xaxpy_as(2, -cartesian_trajectory_planner_B.e_d[m -
        1] / cartesian_trajectory_planner_B.e_d[1],
        cartesian_trajectory_planner_B.work, 2,
        cartesian_trajectory_planner_B.A_h, (m - 1) * 3 + 2,
        cartesian_trajectory_planner_B.A);
    }
  }

  for (m = 2; m < 4; m++) {
    V[m - 1] = cartesian_trajectory_planner_B.e_d[m - 1];
  }

  apply_transform = false;
  cartesian_trajectory_planner_B.nrm = cartesian_trajectory_plan_xnrm2(2,
    cartesian_trajectory_planner_B.A, 5);
  if (cartesian_trajectory_planner_B.nrm > 0.0) {
    apply_transform = true;
    if (cartesian_trajectory_planner_B.A[4] < 0.0) {
      cartesian_trajectory_planner_B.s[1] = -cartesian_trajectory_planner_B.nrm;
    } else {
      cartesian_trajectory_planner_B.s[1] = cartesian_trajectory_planner_B.nrm;
    }

    if (fabs(cartesian_trajectory_planner_B.s[1]) >= 1.0020841800044864E-292) {
      cartesian_trajectory_planner_B.nrm = 1.0 /
        cartesian_trajectory_planner_B.s[1];
      for (qq = 5; qq < 7; qq++) {
        cartesian_trajectory_planner_B.A[qq - 1] *=
          cartesian_trajectory_planner_B.nrm;
      }
    } else {
      for (qq = 5; qq < 7; qq++) {
        cartesian_trajectory_planner_B.A[qq - 1] /=
          cartesian_trajectory_planner_B.s[1];
      }
    }

    cartesian_trajectory_planner_B.A[4]++;
    cartesian_trajectory_planner_B.s[1] = -cartesian_trajectory_planner_B.s[1];
  } else {
    cartesian_trajectory_planner_B.s[1] = 0.0;
  }

  if (apply_transform) {
    for (m = 3; m < 4; m++) {
      memcpy(&cartesian_trajectory_planner_B.A_h[0],
             &cartesian_trajectory_planner_B.A[0], 9U * sizeof(real_T));
      cartesian_trajectory_plan_xaxpy(2, -(cartesian_trajectory_plan_xdotc(2,
        cartesian_trajectory_planner_B.A, 5, cartesian_trajectory_planner_B.A, 8)
        / cartesian_trajectory_planner_B.A[4]), 5,
        cartesian_trajectory_planner_B.A_h, 8, cartesian_trajectory_planner_B.A);
    }
  }

  for (m = 2; m < 4; m++) {
    U[m + 2] = cartesian_trajectory_planner_B.A[m + 2];
  }

  m = 2;
  cartesian_trajectory_planner_B.s[2] = cartesian_trajectory_planner_B.A[8];
  cartesian_trajectory_planner_B.e_d[1] = cartesian_trajectory_planner_B.A[7];
  cartesian_trajectory_planner_B.e_d[2] = 0.0;
  U[6] = 0.0;
  U[7] = 0.0;
  U[8] = 1.0;
  for (c_q = 1; c_q >= 0; c_q--) {
    qq = 3 * c_q + c_q;
    if (cartesian_trajectory_planner_B.s[c_q] != 0.0) {
      for (kase = c_q + 2; kase < 4; kase++) {
        qjj = ((kase - 1) * 3 + c_q) + 1;
        memcpy(&cartesian_trajectory_planner_B.A[0], &U[0], 9U * sizeof(real_T));
        cartesian_trajectory_plan_xaxpy(3 - c_q,
          -(cartesian_trajectory_plan_xdotc(3 - c_q, U, qq + 1, U, qjj) / U[qq]),
          qq + 1, cartesian_trajectory_planner_B.A, qjj, U);
      }

      for (qjj = c_q + 1; qjj < 4; qjj++) {
        kase = (3 * c_q + qjj) - 1;
        U[kase] = -U[kase];
      }

      U[qq]++;
      if (0 <= c_q - 1) {
        U[3 * c_q] = 0.0;
      }
    } else {
      U[3 * c_q] = 0.0;
      U[3 * c_q + 1] = 0.0;
      U[3 * c_q + 2] = 0.0;
      U[qq] = 1.0;
    }
  }

  for (c_q = 2; c_q >= 0; c_q--) {
    if ((c_q + 1 <= 1) && (cartesian_trajectory_planner_B.e_d[0] != 0.0)) {
      memcpy(&cartesian_trajectory_planner_B.A[0], &V[0], 9U * sizeof(real_T));
      cartesian_trajectory_plan_xaxpy(2, -(cartesian_trajectory_plan_xdotc(2, V,
        2, V, 5) / V[1]), 2, cartesian_trajectory_planner_B.A, 5, V);
      memcpy(&cartesian_trajectory_planner_B.A[0], &V[0], 9U * sizeof(real_T));
      cartesian_trajectory_plan_xaxpy(2, -(cartesian_trajectory_plan_xdotc(2, V,
        2, V, 8) / V[1]), 2, cartesian_trajectory_planner_B.A, 8, V);
    }

    V[3 * c_q] = 0.0;
    V[3 * c_q + 1] = 0.0;
    V[3 * c_q + 2] = 0.0;
    V[c_q + 3 * c_q] = 1.0;
  }

  for (c_q = 0; c_q < 3; c_q++) {
    cartesian_trajectory_planner_B.ztest =
      cartesian_trajectory_planner_B.e_d[c_q];
    if (cartesian_trajectory_planner_B.s[c_q] != 0.0) {
      cartesian_trajectory_planner_B.rt = fabs
        (cartesian_trajectory_planner_B.s[c_q]);
      cartesian_trajectory_planner_B.nrm = cartesian_trajectory_planner_B.s[c_q]
        / cartesian_trajectory_planner_B.rt;
      cartesian_trajectory_planner_B.s[c_q] = cartesian_trajectory_planner_B.rt;
      if (c_q + 1 < 3) {
        cartesian_trajectory_planner_B.ztest =
          cartesian_trajectory_planner_B.e_d[c_q] /
          cartesian_trajectory_planner_B.nrm;
      }

      qjj = 3 * c_q;
      for (qq = qjj + 1; qq <= qjj + 3; qq++) {
        U[qq - 1] *= cartesian_trajectory_planner_B.nrm;
      }
    }

    if ((c_q + 1 < 3) && (cartesian_trajectory_planner_B.ztest != 0.0)) {
      cartesian_trajectory_planner_B.rt = fabs
        (cartesian_trajectory_planner_B.ztest);
      cartesian_trajectory_planner_B.nrm = cartesian_trajectory_planner_B.rt /
        cartesian_trajectory_planner_B.ztest;
      cartesian_trajectory_planner_B.ztest = cartesian_trajectory_planner_B.rt;
      cartesian_trajectory_planner_B.s[c_q + 1] *=
        cartesian_trajectory_planner_B.nrm;
      qjj = (c_q + 1) * 3;
      for (qq = qjj + 1; qq <= qjj + 3; qq++) {
        V[qq - 1] *= cartesian_trajectory_planner_B.nrm;
      }
    }

    cartesian_trajectory_planner_B.e_d[c_q] =
      cartesian_trajectory_planner_B.ztest;
  }

  qq = 0;
  cartesian_trajectory_planner_B.nrm = 0.0;
  cartesian_trajectory_planner_B.ztest = fabs(cartesian_trajectory_planner_B.s[0]);
  cartesian_trajectory_planner_B.rt = fabs(cartesian_trajectory_planner_B.e_d[0]);
  if ((cartesian_trajectory_planner_B.ztest > cartesian_trajectory_planner_B.rt)
      || rtIsNaN(cartesian_trajectory_planner_B.rt)) {
    cartesian_trajectory_planner_B.rt = cartesian_trajectory_planner_B.ztest;
  }

  if (!rtIsNaN(cartesian_trajectory_planner_B.rt)) {
    cartesian_trajectory_planner_B.nrm = cartesian_trajectory_planner_B.rt;
  }

  cartesian_trajectory_planner_B.ztest = fabs(cartesian_trajectory_planner_B.s[1]);
  cartesian_trajectory_planner_B.rt = fabs(cartesian_trajectory_planner_B.e_d[1]);
  if ((cartesian_trajectory_planner_B.ztest > cartesian_trajectory_planner_B.rt)
      || rtIsNaN(cartesian_trajectory_planner_B.rt)) {
    cartesian_trajectory_planner_B.rt = cartesian_trajectory_planner_B.ztest;
  }

  if ((!(cartesian_trajectory_planner_B.nrm > cartesian_trajectory_planner_B.rt))
      && (!rtIsNaN(cartesian_trajectory_planner_B.rt))) {
    cartesian_trajectory_planner_B.nrm = cartesian_trajectory_planner_B.rt;
  }

  cartesian_trajectory_planner_B.ztest = fabs(cartesian_trajectory_planner_B.s[2]);
  cartesian_trajectory_planner_B.rt = fabs(cartesian_trajectory_planner_B.e_d[2]);
  if ((cartesian_trajectory_planner_B.ztest > cartesian_trajectory_planner_B.rt)
      || rtIsNaN(cartesian_trajectory_planner_B.rt)) {
    cartesian_trajectory_planner_B.rt = cartesian_trajectory_planner_B.ztest;
  }

  if ((!(cartesian_trajectory_planner_B.nrm > cartesian_trajectory_planner_B.rt))
      && (!rtIsNaN(cartesian_trajectory_planner_B.rt))) {
    cartesian_trajectory_planner_B.nrm = cartesian_trajectory_planner_B.rt;
  }

  while ((m + 1 > 0) && (!(qq >= 75))) {
    c_q = m;
    qjj = m;
    exitg1 = false;
    while ((!exitg1) && (qjj > -1)) {
      c_q = qjj;
      if (qjj == 0) {
        exitg1 = true;
      } else {
        cartesian_trajectory_planner_B.rt = fabs
          (cartesian_trajectory_planner_B.e_d[qjj - 1]);
        if ((cartesian_trajectory_planner_B.rt <= (fabs
              (cartesian_trajectory_planner_B.s[qjj - 1]) + fabs
              (cartesian_trajectory_planner_B.s[qjj])) * 2.2204460492503131E-16)
            || (cartesian_trajectory_planner_B.rt <= 1.0020841800044864E-292) ||
            ((qq > 20) && (cartesian_trajectory_planner_B.rt <=
                           2.2204460492503131E-16 *
                           cartesian_trajectory_planner_B.nrm))) {
          cartesian_trajectory_planner_B.e_d[qjj - 1] = 0.0;
          exitg1 = true;
        } else {
          qjj--;
        }
      }
    }

    if (c_q == m) {
      kase = 4;
    } else {
      qjj = m + 1;
      kase = m + 1;
      exitg1 = false;
      while ((!exitg1) && (kase >= c_q)) {
        qjj = kase;
        if (kase == c_q) {
          exitg1 = true;
        } else {
          cartesian_trajectory_planner_B.rt = 0.0;
          if (kase < m + 1) {
            cartesian_trajectory_planner_B.rt = fabs
              (cartesian_trajectory_planner_B.e_d[kase - 1]);
          }

          if (kase > c_q + 1) {
            cartesian_trajectory_planner_B.rt += fabs
              (cartesian_trajectory_planner_B.e_d[kase - 2]);
          }

          cartesian_trajectory_planner_B.ztest = fabs
            (cartesian_trajectory_planner_B.s[kase - 1]);
          if ((cartesian_trajectory_planner_B.ztest <= 2.2204460492503131E-16 *
               cartesian_trajectory_planner_B.rt) ||
              (cartesian_trajectory_planner_B.ztest <= 1.0020841800044864E-292))
          {
            cartesian_trajectory_planner_B.s[kase - 1] = 0.0;
            exitg1 = true;
          } else {
            kase--;
          }
        }
      }

      if (qjj == c_q) {
        kase = 3;
      } else if (m + 1 == qjj) {
        kase = 1;
      } else {
        kase = 2;
        c_q = qjj;
      }
    }

    switch (kase) {
     case 1:
      cartesian_trajectory_planner_B.rt = cartesian_trajectory_planner_B.e_d[m -
        1];
      cartesian_trajectory_planner_B.e_d[m - 1] = 0.0;
      for (qjj = m; qjj >= c_q + 1; qjj--) {
        cartesian_trajectory_planner_B.ztest =
          cartesian_trajectory_planner_B.e_d[0];
        cartesian_trajectory_plan_xrotg(cartesian_trajectory_planner_B.s[qjj - 1],
          cartesian_trajectory_planner_B.rt,
          &cartesian_trajectory_planner_B.s[qjj - 1],
          &cartesian_trajectory_planner_B.rt,
          &cartesian_trajectory_planner_B.sqds,
          &cartesian_trajectory_planner_B.b_f);
        if (qjj > c_q + 1) {
          cartesian_trajectory_planner_B.rt =
            -cartesian_trajectory_planner_B.b_f *
            cartesian_trajectory_planner_B.e_d[0];
          cartesian_trajectory_planner_B.ztest =
            cartesian_trajectory_planner_B.e_d[0] *
            cartesian_trajectory_planner_B.sqds;
        }

        memcpy(&cartesian_trajectory_planner_B.A[0], &V[0], 9U * sizeof(real_T));
        cartesian_trajectory_plann_xrot(cartesian_trajectory_planner_B.A, (qjj -
          1) * 3 + 1, 3 * m + 1, cartesian_trajectory_planner_B.sqds,
          cartesian_trajectory_planner_B.b_f, V);
        cartesian_trajectory_planner_B.e_d[0] =
          cartesian_trajectory_planner_B.ztest;
      }
      break;

     case 2:
      cartesian_trajectory_planner_B.rt = cartesian_trajectory_planner_B.e_d[c_q
        - 1];
      cartesian_trajectory_planner_B.e_d[c_q - 1] = 0.0;
      for (qjj = c_q + 1; qjj <= m + 1; qjj++) {
        cartesian_trajectory_plan_xrotg(cartesian_trajectory_planner_B.s[qjj - 1],
          cartesian_trajectory_planner_B.rt,
          &cartesian_trajectory_planner_B.s[qjj - 1],
          &cartesian_trajectory_planner_B.ztest,
          &cartesian_trajectory_planner_B.sqds,
          &cartesian_trajectory_planner_B.b_f);
        cartesian_trajectory_planner_B.ztest =
          cartesian_trajectory_planner_B.e_d[qjj - 1];
        cartesian_trajectory_planner_B.rt = cartesian_trajectory_planner_B.ztest
          * -cartesian_trajectory_planner_B.b_f;
        cartesian_trajectory_planner_B.e_d[qjj - 1] =
          cartesian_trajectory_planner_B.ztest *
          cartesian_trajectory_planner_B.sqds;
        memcpy(&cartesian_trajectory_planner_B.A[0], &U[0], 9U * sizeof(real_T));
        cartesian_trajectory_plann_xrot(cartesian_trajectory_planner_B.A, (qjj -
          1) * 3 + 1, (c_q - 1) * 3 + 1, cartesian_trajectory_planner_B.sqds,
          cartesian_trajectory_planner_B.b_f, U);
      }
      break;

     case 3:
      cartesian_trajectory_planner_B.ztest = fabs
        (cartesian_trajectory_planner_B.s[m]);
      cartesian_trajectory_planner_B.sqds = cartesian_trajectory_planner_B.s[m -
        1];
      cartesian_trajectory_planner_B.rt = fabs
        (cartesian_trajectory_planner_B.sqds);
      if ((cartesian_trajectory_planner_B.ztest >
           cartesian_trajectory_planner_B.rt) || rtIsNaN
          (cartesian_trajectory_planner_B.rt)) {
        cartesian_trajectory_planner_B.rt = cartesian_trajectory_planner_B.ztest;
      }

      cartesian_trajectory_planner_B.b_f = cartesian_trajectory_planner_B.e_d[m
        - 1];
      cartesian_trajectory_planner_B.ztest = fabs
        (cartesian_trajectory_planner_B.b_f);
      if ((cartesian_trajectory_planner_B.rt >
           cartesian_trajectory_planner_B.ztest) || rtIsNaN
          (cartesian_trajectory_planner_B.ztest)) {
        cartesian_trajectory_planner_B.ztest = cartesian_trajectory_planner_B.rt;
      }

      cartesian_trajectory_planner_B.rt = fabs
        (cartesian_trajectory_planner_B.s[c_q]);
      if ((cartesian_trajectory_planner_B.ztest >
           cartesian_trajectory_planner_B.rt) || rtIsNaN
          (cartesian_trajectory_planner_B.rt)) {
        cartesian_trajectory_planner_B.rt = cartesian_trajectory_planner_B.ztest;
      }

      cartesian_trajectory_planner_B.ztest = fabs
        (cartesian_trajectory_planner_B.e_d[c_q]);
      if ((cartesian_trajectory_planner_B.rt >
           cartesian_trajectory_planner_B.ztest) || rtIsNaN
          (cartesian_trajectory_planner_B.ztest)) {
        cartesian_trajectory_planner_B.ztest = cartesian_trajectory_planner_B.rt;
      }

      cartesian_trajectory_planner_B.rt = cartesian_trajectory_planner_B.s[m] /
        cartesian_trajectory_planner_B.ztest;
      cartesian_trajectory_planner_B.smm1 = cartesian_trajectory_planner_B.sqds /
        cartesian_trajectory_planner_B.ztest;
      cartesian_trajectory_planner_B.emm1 = cartesian_trajectory_planner_B.b_f /
        cartesian_trajectory_planner_B.ztest;
      cartesian_trajectory_planner_B.sqds = cartesian_trajectory_planner_B.s[c_q]
        / cartesian_trajectory_planner_B.ztest;
      cartesian_trajectory_planner_B.b_f = ((cartesian_trajectory_planner_B.smm1
        + cartesian_trajectory_planner_B.rt) *
        (cartesian_trajectory_planner_B.smm1 - cartesian_trajectory_planner_B.rt)
        + cartesian_trajectory_planner_B.emm1 *
        cartesian_trajectory_planner_B.emm1) / 2.0;
      cartesian_trajectory_planner_B.smm1 = cartesian_trajectory_planner_B.rt *
        cartesian_trajectory_planner_B.emm1;
      cartesian_trajectory_planner_B.smm1 *= cartesian_trajectory_planner_B.smm1;
      if ((cartesian_trajectory_planner_B.b_f != 0.0) ||
          (cartesian_trajectory_planner_B.smm1 != 0.0)) {
        cartesian_trajectory_planner_B.emm1 = sqrt
          (cartesian_trajectory_planner_B.b_f *
           cartesian_trajectory_planner_B.b_f +
           cartesian_trajectory_planner_B.smm1);
        if (cartesian_trajectory_planner_B.b_f < 0.0) {
          cartesian_trajectory_planner_B.emm1 =
            -cartesian_trajectory_planner_B.emm1;
        }

        cartesian_trajectory_planner_B.emm1 =
          cartesian_trajectory_planner_B.smm1 /
          (cartesian_trajectory_planner_B.b_f +
           cartesian_trajectory_planner_B.emm1);
      } else {
        cartesian_trajectory_planner_B.emm1 = 0.0;
      }

      cartesian_trajectory_planner_B.rt = (cartesian_trajectory_planner_B.sqds +
        cartesian_trajectory_planner_B.rt) *
        (cartesian_trajectory_planner_B.sqds - cartesian_trajectory_planner_B.rt)
        + cartesian_trajectory_planner_B.emm1;
      cartesian_trajectory_planner_B.sqds *=
        cartesian_trajectory_planner_B.e_d[c_q] /
        cartesian_trajectory_planner_B.ztest;
      for (d_k = c_q + 1; d_k <= m; d_k++) {
        cartesian_trajectory_plan_xrotg(cartesian_trajectory_planner_B.rt,
          cartesian_trajectory_planner_B.sqds,
          &cartesian_trajectory_planner_B.ztest,
          &cartesian_trajectory_planner_B.emm1,
          &cartesian_trajectory_planner_B.b_f,
          &cartesian_trajectory_planner_B.smm1);
        if (d_k > c_q + 1) {
          cartesian_trajectory_planner_B.e_d[0] =
            cartesian_trajectory_planner_B.ztest;
        }

        cartesian_trajectory_planner_B.ztest =
          cartesian_trajectory_planner_B.e_d[d_k - 1];
        cartesian_trajectory_planner_B.rt = cartesian_trajectory_planner_B.s[d_k
          - 1];
        cartesian_trajectory_planner_B.e_d[d_k - 1] =
          cartesian_trajectory_planner_B.ztest *
          cartesian_trajectory_planner_B.b_f - cartesian_trajectory_planner_B.rt
          * cartesian_trajectory_planner_B.smm1;
        cartesian_trajectory_planner_B.sqds =
          cartesian_trajectory_planner_B.smm1 *
          cartesian_trajectory_planner_B.s[d_k];
        cartesian_trajectory_planner_B.s[d_k] *=
          cartesian_trajectory_planner_B.b_f;
        qjj = (d_k - 1) * 3 + 1;
        kase = 3 * d_k + 1;
        memcpy(&cartesian_trajectory_planner_B.A[0], &V[0], 9U * sizeof(real_T));
        cartesian_trajectory_plann_xrot(cartesian_trajectory_planner_B.A, qjj,
          kase, cartesian_trajectory_planner_B.b_f,
          cartesian_trajectory_planner_B.smm1, V);
        cartesian_trajectory_plan_xrotg(cartesian_trajectory_planner_B.rt *
          cartesian_trajectory_planner_B.b_f +
          cartesian_trajectory_planner_B.ztest *
          cartesian_trajectory_planner_B.smm1,
          cartesian_trajectory_planner_B.sqds,
          &cartesian_trajectory_planner_B.s[d_k - 1],
          &cartesian_trajectory_planner_B.unusedU2,
          &cartesian_trajectory_planner_B.emm1,
          &cartesian_trajectory_planner_B.d_sn);
        cartesian_trajectory_planner_B.rt =
          cartesian_trajectory_planner_B.e_d[d_k - 1] *
          cartesian_trajectory_planner_B.emm1 +
          cartesian_trajectory_planner_B.d_sn *
          cartesian_trajectory_planner_B.s[d_k];
        cartesian_trajectory_planner_B.s[d_k] =
          cartesian_trajectory_planner_B.e_d[d_k - 1] *
          -cartesian_trajectory_planner_B.d_sn +
          cartesian_trajectory_planner_B.emm1 *
          cartesian_trajectory_planner_B.s[d_k];
        cartesian_trajectory_planner_B.sqds =
          cartesian_trajectory_planner_B.d_sn *
          cartesian_trajectory_planner_B.e_d[d_k];
        cartesian_trajectory_planner_B.e_d[d_k] *=
          cartesian_trajectory_planner_B.emm1;
        memcpy(&cartesian_trajectory_planner_B.A[0], &U[0], 9U * sizeof(real_T));
        cartesian_trajectory_plann_xrot(cartesian_trajectory_planner_B.A, qjj,
          kase, cartesian_trajectory_planner_B.emm1,
          cartesian_trajectory_planner_B.d_sn, U);
      }

      cartesian_trajectory_planner_B.e_d[m - 1] =
        cartesian_trajectory_planner_B.rt;
      qq++;
      break;

     default:
      if (cartesian_trajectory_planner_B.s[c_q] < 0.0) {
        cartesian_trajectory_planner_B.s[c_q] =
          -cartesian_trajectory_planner_B.s[c_q];
        qjj = 3 * c_q;
        for (qq = qjj + 1; qq <= qjj + 3; qq++) {
          V[qq - 1] = -V[qq - 1];
        }
      }

      qq = c_q + 1;
      while ((c_q + 1 < 3) && (cartesian_trajectory_planner_B.s[c_q] <
              cartesian_trajectory_planner_B.s[qq])) {
        cartesian_trajectory_planner_B.rt = cartesian_trajectory_planner_B.s[c_q];
        cartesian_trajectory_planner_B.s[c_q] =
          cartesian_trajectory_planner_B.s[qq];
        cartesian_trajectory_planner_B.s[qq] = cartesian_trajectory_planner_B.rt;
        qjj = 3 * c_q + 1;
        kase = (c_q + 1) * 3 + 1;
        memcpy(&cartesian_trajectory_planner_B.A[0], &V[0], 9U * sizeof(real_T));
        cartesian_trajectory_plan_xswap(cartesian_trajectory_planner_B.A, qjj,
          kase, V);
        memcpy(&cartesian_trajectory_planner_B.A[0], &U[0], 9U * sizeof(real_T));
        cartesian_trajectory_plan_xswap(cartesian_trajectory_planner_B.A, qjj,
          kase, U);
        c_q = qq;
        qq++;
      }

      qq = 0;
      m--;
      break;
    }
  }

  s[0] = cartesian_trajectory_planner_B.s[0];
  s[1] = cartesian_trajectory_planner_B.s[1];
  s[2] = cartesian_trajectory_planner_B.s[2];
}

static void cartesian_trajectory_rotm2axang(const real_T R[9], real_T axang[4])
{
  boolean_T e;
  boolean_T p;
  boolean_T rEQ0;
  int32_T loop_ub_tmp;
  boolean_T exitg1;
  cartesian_trajectory_planner_B.u.re = (((R[0] + R[4]) + R[8]) - 1.0) * 0.5;
  if (!(fabs(cartesian_trajectory_planner_B.u.re) > 1.0)) {
    cartesian_trajectory_planner_B.v_fx.re = acos
      (cartesian_trajectory_planner_B.u.re);
  } else {
    cartesian_trajectory_planner_B.u_p.re = cartesian_trajectory_planner_B.u.re
      + 1.0;
    cartesian_trajectory_planner_B.u_p.im = 0.0;
    cartesian_trajectory_planner_B.dc.re = 1.0 -
      cartesian_trajectory_planner_B.u.re;
    cartesian_trajectory_planner_B.dc.im = 0.0;
    cartesian_trajectory_planner_B.v_fx.re = 2.0 * rt_atan2d_snf
      ((cartesian_trajectory_plann_sqrt(cartesian_trajectory_planner_B.dc)).re,
       (cartesian_trajectory_plann_sqrt(cartesian_trajectory_planner_B.u_p)).re);
  }

  cartesian_trajectory_planner_B.a_cm = 2.0 * sin
    (cartesian_trajectory_planner_B.v_fx.re);
  cartesian_trajectory_planner_B.v_nr[0] = (R[5] - R[7]) /
    cartesian_trajectory_planner_B.a_cm;
  cartesian_trajectory_planner_B.v_nr[1] = (R[6] - R[2]) /
    cartesian_trajectory_planner_B.a_cm;
  cartesian_trajectory_planner_B.v_nr[2] = (R[1] - R[3]) /
    cartesian_trajectory_planner_B.a_cm;
  if (rtIsNaN(cartesian_trajectory_planner_B.v_fx.re) || rtIsInf
      (cartesian_trajectory_planner_B.v_fx.re)) {
    cartesian_trajectory_planner_B.a_cm = (rtNaN);
  } else if (cartesian_trajectory_planner_B.v_fx.re == 0.0) {
    cartesian_trajectory_planner_B.a_cm = 0.0;
  } else {
    cartesian_trajectory_planner_B.a_cm = fmod
      (cartesian_trajectory_planner_B.v_fx.re, 3.1415926535897931);
    rEQ0 = (cartesian_trajectory_planner_B.a_cm == 0.0);
    if (!rEQ0) {
      cartesian_trajectory_planner_B.q = fabs
        (cartesian_trajectory_planner_B.v_fx.re / 3.1415926535897931);
      rEQ0 = !(fabs(cartesian_trajectory_planner_B.q - floor
                    (cartesian_trajectory_planner_B.q + 0.5)) >
               2.2204460492503131E-16 * cartesian_trajectory_planner_B.q);
    }

    if (rEQ0) {
      cartesian_trajectory_planner_B.a_cm = 0.0;
    } else {
      if (cartesian_trajectory_planner_B.v_fx.re < 0.0) {
        cartesian_trajectory_planner_B.a_cm += 3.1415926535897931;
      }
    }
  }

  rEQ0 = (cartesian_trajectory_planner_B.a_cm == 0.0);
  e = true;
  cartesian_trajectory_planner_B.b_k_p = 0;
  exitg1 = false;
  while ((!exitg1) && (cartesian_trajectory_planner_B.b_k_p < 3)) {
    if (!(cartesian_trajectory_planner_B.v_nr[cartesian_trajectory_planner_B.b_k_p]
          == 0.0)) {
      e = false;
      exitg1 = true;
    } else {
      cartesian_trajectory_planner_B.b_k_p++;
    }
  }

  if (rEQ0 || e) {
    loop_ub_tmp = (rEQ0 || e);
    cartesian_trajectory_planner_B.loop_ub_nk = loop_ub_tmp * 3 - 1;
    if (0 <= cartesian_trajectory_planner_B.loop_ub_nk) {
      memset(&cartesian_trajectory_planner_B.vspecial_data[0], 0,
             (cartesian_trajectory_planner_B.loop_ub_nk + 1) * sizeof(real_T));
    }

    loop_ub_tmp--;
    for (cartesian_trajectory_planner_B.loop_ub_nk = 0;
         cartesian_trajectory_planner_B.loop_ub_nk <= loop_ub_tmp;
         cartesian_trajectory_planner_B.loop_ub_nk++) {
      memset(&cartesian_trajectory_planner_B.b_I[0], 0, 9U * sizeof(real_T));
      cartesian_trajectory_planner_B.b_I[0] = 1.0;
      cartesian_trajectory_planner_B.b_I[4] = 1.0;
      cartesian_trajectory_planner_B.b_I[8] = 1.0;
      p = true;
      for (cartesian_trajectory_planner_B.b_k_p = 0;
           cartesian_trajectory_planner_B.b_k_p < 9;
           cartesian_trajectory_planner_B.b_k_p++) {
        cartesian_trajectory_planner_B.a_cm =
          cartesian_trajectory_planner_B.b_I[cartesian_trajectory_planner_B.b_k_p]
          - R[cartesian_trajectory_planner_B.b_k_p];
        if (p && ((!rtIsInf(cartesian_trajectory_planner_B.a_cm)) && (!rtIsNaN
              (cartesian_trajectory_planner_B.a_cm)))) {
        } else {
          p = false;
        }

        cartesian_trajectory_planner_B.b_I[cartesian_trajectory_planner_B.b_k_p]
          = cartesian_trajectory_planner_B.a_cm;
      }

      if (p) {
        cartesian_trajectory_planne_svd(cartesian_trajectory_planner_B.b_I,
          cartesian_trajectory_planner_B.b_U,
          cartesian_trajectory_planner_B.vspecial_data,
          cartesian_trajectory_planner_B.V_l);
      } else {
        for (cartesian_trajectory_planner_B.b_k_p = 0;
             cartesian_trajectory_planner_B.b_k_p < 9;
             cartesian_trajectory_planner_B.b_k_p++) {
          cartesian_trajectory_planner_B.V_l[cartesian_trajectory_planner_B.b_k_p]
            = (rtNaN);
        }
      }

      cartesian_trajectory_planner_B.vspecial_data[0] =
        cartesian_trajectory_planner_B.V_l[6];
      cartesian_trajectory_planner_B.vspecial_data[1] =
        cartesian_trajectory_planner_B.V_l[7];
      cartesian_trajectory_planner_B.vspecial_data[2] =
        cartesian_trajectory_planner_B.V_l[8];
    }

    loop_ub_tmp = 0;
    if (rEQ0 || e) {
      for (cartesian_trajectory_planner_B.loop_ub_nk = 0;
           cartesian_trajectory_planner_B.loop_ub_nk < 1;
           cartesian_trajectory_planner_B.loop_ub_nk++) {
        loop_ub_tmp++;
      }
    }

    for (cartesian_trajectory_planner_B.b_k_p = 0;
         cartesian_trajectory_planner_B.b_k_p < loop_ub_tmp;
         cartesian_trajectory_planner_B.b_k_p++) {
      cartesian_trajectory_planner_B.v_nr[0] =
        cartesian_trajectory_planner_B.vspecial_data[3 *
        cartesian_trajectory_planner_B.b_k_p];
      cartesian_trajectory_planner_B.v_nr[1] =
        cartesian_trajectory_planner_B.vspecial_data[3 *
        cartesian_trajectory_planner_B.b_k_p + 1];
      cartesian_trajectory_planner_B.v_nr[2] =
        cartesian_trajectory_planner_B.vspecial_data[3 *
        cartesian_trajectory_planner_B.b_k_p + 2];
    }
  }

  cartesian_trajectory_planner_B.a_cm = 1.0 / sqrt
    ((cartesian_trajectory_planner_B.v_nr[0] *
      cartesian_trajectory_planner_B.v_nr[0] +
      cartesian_trajectory_planner_B.v_nr[1] *
      cartesian_trajectory_planner_B.v_nr[1]) +
     cartesian_trajectory_planner_B.v_nr[2] *
     cartesian_trajectory_planner_B.v_nr[2]);
  cartesian_trajectory_planner_B.v_nr[0] *= cartesian_trajectory_planner_B.a_cm;
  cartesian_trajectory_planner_B.v_nr[1] *= cartesian_trajectory_planner_B.a_cm;
  axang[0] = cartesian_trajectory_planner_B.v_nr[0];
  axang[1] = cartesian_trajectory_planner_B.v_nr[1];
  axang[2] = cartesian_trajectory_planner_B.v_nr[2] *
    cartesian_trajectory_planner_B.a_cm;
  axang[3] = cartesian_trajectory_planner_B.v_fx.re;
}

static void cartesian_IKHelpers_computeCost(const real_T x[6],
  f_robotics_manip_internal_IKE_T *args, real_T *cost, real_T W[36],
  emxArray_real_T_cartesian_tra_T *Jac, f_robotics_manip_internal_IKE_T **b_args)
{
  x_robotics_manip_internal_Rig_T *treeInternal;
  emxArray_char_T_cartesian_tra_T *bodyName;
  emxArray_real_T_cartesian_tra_T *J;
  emxArray_real_T_cartesian_tra_T *y;
  cartesian_trajec_emxInit_char_T(&bodyName, 2);
  *b_args = args;
  treeInternal = args->Robot;
  cartesian_trajectory_planner_B.b_j_j = bodyName->size[0] * bodyName->size[1];
  bodyName->size[0] = 1;
  bodyName->size[1] = args->BodyName->size[1];
  cartes_emxEnsureCapacity_char_T(bodyName, cartesian_trajectory_planner_B.b_j_j);
  cartesian_trajectory_planner_B.loop_ub_h = args->BodyName->size[0] *
    args->BodyName->size[1] - 1;
  for (cartesian_trajectory_planner_B.b_j_j = 0;
       cartesian_trajectory_planner_B.b_j_j <=
       cartesian_trajectory_planner_B.loop_ub_h;
       cartesian_trajectory_planner_B.b_j_j++) {
    bodyName->data[cartesian_trajectory_planner_B.b_j_j] = args->BodyName->
      data[cartesian_trajectory_planner_B.b_j_j];
  }

  for (cartesian_trajectory_planner_B.b_j_j = 0;
       cartesian_trajectory_planner_B.b_j_j < 16;
       cartesian_trajectory_planner_B.b_j_j++) {
    cartesian_trajectory_planner_B.Td[cartesian_trajectory_planner_B.b_j_j] =
      args->Tform[cartesian_trajectory_planner_B.b_j_j];
  }

  for (cartesian_trajectory_planner_B.b_j_j = 0;
       cartesian_trajectory_planner_B.b_j_j < 36;
       cartesian_trajectory_planner_B.b_j_j++) {
    W[cartesian_trajectory_planner_B.b_j_j] = args->
      WeightMatrix[cartesian_trajectory_planner_B.b_j_j];
  }

  cartesian_trajec_emxInit_real_T(&J, 2);
  RigidBodyTree_efficientFKAndJac(treeInternal, x, bodyName,
    cartesian_trajectory_planner_B.T_data, cartesian_trajectory_planner_B.T_size,
    J);
  cartesian_trajectory_planner_B.b_j_j = Jac->size[0] * Jac->size[1];
  Jac->size[0] = 6;
  Jac->size[1] = J->size[1];
  cartes_emxEnsureCapacity_real_T(Jac, cartesian_trajectory_planner_B.b_j_j);
  cartesian_trajectory_planner_B.loop_ub_h = J->size[0] * J->size[1] - 1;
  cartesian_trajec_emxFree_char_T(&bodyName);
  for (cartesian_trajectory_planner_B.b_j_j = 0;
       cartesian_trajectory_planner_B.b_j_j <=
       cartesian_trajectory_planner_B.loop_ub_h;
       cartesian_trajectory_planner_B.b_j_j++) {
    Jac->data[cartesian_trajectory_planner_B.b_j_j] = -J->
      data[cartesian_trajectory_planner_B.b_j_j];
  }

  cartesian_trajec_emxFree_real_T(&J);
  for (cartesian_trajectory_planner_B.b_j_j = 0;
       cartesian_trajectory_planner_B.b_j_j < 3;
       cartesian_trajectory_planner_B.b_j_j++) {
    cartesian_trajectory_planner_B.T_i[3 * cartesian_trajectory_planner_B.b_j_j]
      =
      cartesian_trajectory_planner_B.T_data[cartesian_trajectory_planner_B.b_j_j];
    cartesian_trajectory_planner_B.n_m = 3 *
      cartesian_trajectory_planner_B.b_j_j + 1;
    cartesian_trajectory_planner_B.T_i[cartesian_trajectory_planner_B.n_m] =
      cartesian_trajectory_planner_B.T_data
      [((cartesian_trajectory_planner_B.b_j_j + 1) +
        cartesian_trajectory_planner_B.T_size[0]) - 1];
    cartesian_trajectory_planner_B.boffset_k = 3 *
      cartesian_trajectory_planner_B.b_j_j + 2;
    cartesian_trajectory_planner_B.T_i[cartesian_trajectory_planner_B.boffset_k]
      = cartesian_trajectory_planner_B.T_data
      [((cartesian_trajectory_planner_B.b_j_j + 1) +
        (cartesian_trajectory_planner_B.T_size[0] << 1)) - 1];
    for (cartesian_trajectory_planner_B.loop_ub_h = 0;
         cartesian_trajectory_planner_B.loop_ub_h < 3;
         cartesian_trajectory_planner_B.loop_ub_h++) {
      cartesian_trajectory_planner_B.Td_tmp =
        cartesian_trajectory_planner_B.loop_ub_h + 3 *
        cartesian_trajectory_planner_B.b_j_j;
      cartesian_trajectory_planner_B.Td_f[cartesian_trajectory_planner_B.Td_tmp]
        = 0.0;
      cartesian_trajectory_planner_B.Td_f[cartesian_trajectory_planner_B.Td_tmp]
        += cartesian_trajectory_planner_B.T_i[3 *
        cartesian_trajectory_planner_B.b_j_j] *
        cartesian_trajectory_planner_B.Td[cartesian_trajectory_planner_B.loop_ub_h];
      cartesian_trajectory_planner_B.Td_f[cartesian_trajectory_planner_B.Td_tmp]
        += cartesian_trajectory_planner_B.T_i[cartesian_trajectory_planner_B.n_m]
        * cartesian_trajectory_planner_B.Td[cartesian_trajectory_planner_B.loop_ub_h
        + 4];
      cartesian_trajectory_planner_B.Td_f[cartesian_trajectory_planner_B.Td_tmp]
        +=
        cartesian_trajectory_planner_B.T_i[cartesian_trajectory_planner_B.boffset_k]
        * cartesian_trajectory_planner_B.Td[cartesian_trajectory_planner_B.loop_ub_h
        + 8];
    }
  }

  cartesian_trajectory_rotm2axang(cartesian_trajectory_planner_B.Td_f,
    cartesian_trajectory_planner_B.v);
  cartesian_trajectory_planner_B.e[0] = cartesian_trajectory_planner_B.v[3] *
    cartesian_trajectory_planner_B.v[0];
  cartesian_trajectory_planner_B.e[3] = cartesian_trajectory_planner_B.Td[12] -
    cartesian_trajectory_planner_B.T_data[cartesian_trajectory_planner_B.T_size
    [0] * 3];
  cartesian_trajectory_planner_B.e[1] = cartesian_trajectory_planner_B.v[3] *
    cartesian_trajectory_planner_B.v[1];
  cartesian_trajectory_planner_B.e[4] = cartesian_trajectory_planner_B.Td[13] -
    cartesian_trajectory_planner_B.T_data[cartesian_trajectory_planner_B.T_size
    [0] * 3 + 1];
  cartesian_trajectory_planner_B.e[2] = cartesian_trajectory_planner_B.v[3] *
    cartesian_trajectory_planner_B.v[2];
  cartesian_trajectory_planner_B.e[5] = cartesian_trajectory_planner_B.Td[14] -
    cartesian_trajectory_planner_B.T_data[cartesian_trajectory_planner_B.T_size
    [0] * 3 + 2];
  cartesian_trajectory_planner_B.b_j_j = args->ErrTemp->size[0];
  args->ErrTemp->size[0] = 6;
  cartes_emxEnsureCapacity_real_T(args->ErrTemp,
    cartesian_trajectory_planner_B.b_j_j);
  for (cartesian_trajectory_planner_B.b_j_j = 0;
       cartesian_trajectory_planner_B.b_j_j < 6;
       cartesian_trajectory_planner_B.b_j_j++) {
    args->ErrTemp->data[cartesian_trajectory_planner_B.b_j_j] =
      cartesian_trajectory_planner_B.e[cartesian_trajectory_planner_B.b_j_j];
  }

  for (cartesian_trajectory_planner_B.b_j_j = 0;
       cartesian_trajectory_planner_B.b_j_j < 6;
       cartesian_trajectory_planner_B.b_j_j++) {
    cartesian_trajectory_planner_B.y[cartesian_trajectory_planner_B.b_j_j] = 0.0;
    for (cartesian_trajectory_planner_B.loop_ub_h = 0;
         cartesian_trajectory_planner_B.loop_ub_h < 6;
         cartesian_trajectory_planner_B.loop_ub_h++) {
      cartesian_trajectory_planner_B.s_g = W[6 *
        cartesian_trajectory_planner_B.b_j_j +
        cartesian_trajectory_planner_B.loop_ub_h] * (0.5 *
        cartesian_trajectory_planner_B.e[cartesian_trajectory_planner_B.loop_ub_h])
        + cartesian_trajectory_planner_B.y[cartesian_trajectory_planner_B.b_j_j];
      cartesian_trajectory_planner_B.y[cartesian_trajectory_planner_B.b_j_j] =
        cartesian_trajectory_planner_B.s_g;
    }
  }

  cartesian_trajectory_planner_B.s_g = 0.0;
  for (cartesian_trajectory_planner_B.b_j_j = 0;
       cartesian_trajectory_planner_B.b_j_j < 6;
       cartesian_trajectory_planner_B.b_j_j++) {
    cartesian_trajectory_planner_B.s_g +=
      cartesian_trajectory_planner_B.y[cartesian_trajectory_planner_B.b_j_j] *
      cartesian_trajectory_planner_B.e[cartesian_trajectory_planner_B.b_j_j];
  }

  args->CostTemp = cartesian_trajectory_planner_B.s_g;
  for (cartesian_trajectory_planner_B.b_j_j = 0;
       cartesian_trajectory_planner_B.b_j_j < 6;
       cartesian_trajectory_planner_B.b_j_j++) {
    cartesian_trajectory_planner_B.y[cartesian_trajectory_planner_B.b_j_j] = 0.0;
    for (cartesian_trajectory_planner_B.loop_ub_h = 0;
         cartesian_trajectory_planner_B.loop_ub_h < 6;
         cartesian_trajectory_planner_B.loop_ub_h++) {
      cartesian_trajectory_planner_B.s_g = W[6 *
        cartesian_trajectory_planner_B.b_j_j +
        cartesian_trajectory_planner_B.loop_ub_h] *
        cartesian_trajectory_planner_B.e[cartesian_trajectory_planner_B.loop_ub_h]
        + cartesian_trajectory_planner_B.y[cartesian_trajectory_planner_B.b_j_j];
      cartesian_trajectory_planner_B.y[cartesian_trajectory_planner_B.b_j_j] =
        cartesian_trajectory_planner_B.s_g;
    }
  }

  cartesian_trajec_emxInit_real_T(&y, 2);
  cartesian_trajectory_planner_B.n_m = Jac->size[1] - 1;
  cartesian_trajectory_planner_B.b_j_j = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = Jac->size[1];
  cartes_emxEnsureCapacity_real_T(y, cartesian_trajectory_planner_B.b_j_j);
  for (cartesian_trajectory_planner_B.b_j_j = 0;
       cartesian_trajectory_planner_B.b_j_j <=
       cartesian_trajectory_planner_B.n_m; cartesian_trajectory_planner_B.b_j_j
       ++) {
    cartesian_trajectory_planner_B.boffset_k =
      cartesian_trajectory_planner_B.b_j_j * 6 - 1;
    cartesian_trajectory_planner_B.s_g = 0.0;
    for (cartesian_trajectory_planner_B.loop_ub_h = 0;
         cartesian_trajectory_planner_B.loop_ub_h < 6;
         cartesian_trajectory_planner_B.loop_ub_h++) {
      cartesian_trajectory_planner_B.s_g += Jac->data
        [(cartesian_trajectory_planner_B.boffset_k +
          cartesian_trajectory_planner_B.loop_ub_h) + 1] *
        cartesian_trajectory_planner_B.y[cartesian_trajectory_planner_B.loop_ub_h];
    }

    y->data[cartesian_trajectory_planner_B.b_j_j] =
      cartesian_trajectory_planner_B.s_g;
  }

  cartesian_trajectory_planner_B.b_j_j = args->GradTemp->size[0];
  args->GradTemp->size[0] = y->size[1];
  cartes_emxEnsureCapacity_real_T(args->GradTemp,
    cartesian_trajectory_planner_B.b_j_j);
  cartesian_trajectory_planner_B.loop_ub_h = y->size[1];
  for (cartesian_trajectory_planner_B.b_j_j = 0;
       cartesian_trajectory_planner_B.b_j_j <
       cartesian_trajectory_planner_B.loop_ub_h;
       cartesian_trajectory_planner_B.b_j_j++) {
    args->GradTemp->data[cartesian_trajectory_planner_B.b_j_j] = y->
      data[cartesian_trajectory_planner_B.b_j_j];
  }

  cartesian_trajec_emxFree_real_T(&y);
  cartesian_trajectory_planner_B.s_g = args->CostTemp;
  *cost = cartesian_trajectory_planner_B.s_g;
}

static void cartesian_trajectory_planne_eye(real_T b_I[36])
{
  int32_T b_k;
  memset(&b_I[0], 0, 36U * sizeof(real_T));
  for (b_k = 0; b_k < 6; b_k++) {
    b_I[b_k + 6 * b_k] = 1.0;
  }
}

static void cartesian_tra_emxInit_boolean_T(emxArray_boolean_T_cartesian__T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_boolean_T_cartesian__T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_boolean_T_cartesian__T *)malloc(sizeof
    (emxArray_boolean_T_cartesian__T));
  emxArray = *pEmxArray;
  emxArray->data = (boolean_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void cartesian_traje_emxInit_int32_T(emxArray_int32_T_cartesian_tr_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_int32_T_cartesian_tr_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_int32_T_cartesian_tr_T *)malloc(sizeof
    (emxArray_int32_T_cartesian_tr_T));
  emxArray = *pEmxArray;
  emxArray->data = (int32_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void car_emxEnsureCapacity_boolean_T(emxArray_boolean_T_cartesian__T
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

    newData = calloc(static_cast<uint32_T>(i), sizeof(boolean_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(boolean_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (boolean_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void carte_emxEnsureCapacity_int32_T(emxArray_int32_T_cartesian_tr_T
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

    newData = calloc(static_cast<uint32_T>(i), sizeof(int32_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(int32_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (int32_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static real_T cartesian_trajectory_pla_norm_a(const real_T x[6])
{
  real_T y;
  real_T scale;
  real_T absxk;
  real_T t;
  int32_T b_k;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  for (b_k = 0; b_k < 6; b_k++) {
    absxk = fabs(x[b_k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

static real_T SystemTimeProvider_getElapsedTi(const
  f_robotics_core_internal_Syst_T *obj)
{
  real_T systemTime;
  systemTime = ctimefun();
  return systemTime - obj->StartTime;
}

static real_T cartesian_trajectory_p_xnrm2_as(int32_T n, const
  emxArray_real_T_cartesian_tra_T *x, int32_T ix0)
{
  real_T y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x->data[ix0 - 1]);
    } else {
      cartesian_trajectory_planner_B.scale_c = 3.3121686421112381E-170;
      cartesian_trajectory_planner_B.kend_e = ix0 + n;
      for (cartesian_trajectory_planner_B.k_in = ix0;
           cartesian_trajectory_planner_B.k_in <
           cartesian_trajectory_planner_B.kend_e;
           cartesian_trajectory_planner_B.k_in++) {
        cartesian_trajectory_planner_B.absxk_n = fabs(x->
          data[cartesian_trajectory_planner_B.k_in - 1]);
        if (cartesian_trajectory_planner_B.absxk_n >
            cartesian_trajectory_planner_B.scale_c) {
          cartesian_trajectory_planner_B.t_k =
            cartesian_trajectory_planner_B.scale_c /
            cartesian_trajectory_planner_B.absxk_n;
          y = y * cartesian_trajectory_planner_B.t_k *
            cartesian_trajectory_planner_B.t_k + 1.0;
          cartesian_trajectory_planner_B.scale_c =
            cartesian_trajectory_planner_B.absxk_n;
        } else {
          cartesian_trajectory_planner_B.t_k =
            cartesian_trajectory_planner_B.absxk_n /
            cartesian_trajectory_planner_B.scale_c;
          y += cartesian_trajectory_planner_B.t_k *
            cartesian_trajectory_planner_B.t_k;
        }
      }

      y = cartesian_trajectory_planner_B.scale_c * sqrt(y);
    }
  }

  return y;
}

static void cartesian_trajectory_pla_qrpf_a(const
  emxArray_real_T_cartesian_tra_T *A, int32_T m, int32_T n,
  emxArray_real_T_cartesian_tra_T *tau, const emxArray_int32_T_cartesian_tr_T
  *jpvt, emxArray_real_T_cartesian_tra_T *b_A, emxArray_int32_T_cartesian_tr_T
  *b_jpvt)
{
  emxArray_real_T_cartesian_tra_T *work;
  emxArray_real_T_cartesian_tra_T *vn1;
  emxArray_real_T_cartesian_tra_T *vn2;
  emxArray_real_T_cartesian_tra_T *c_x;
  int32_T exitg1;
  boolean_T exitg2;
  cartesian_trajectory_planner_B.kend = b_jpvt->size[0] * b_jpvt->size[1];
  b_jpvt->size[0] = 1;
  b_jpvt->size[1] = jpvt->size[1];
  carte_emxEnsureCapacity_int32_T(b_jpvt, cartesian_trajectory_planner_B.kend);
  cartesian_trajectory_planner_B.ix_m = jpvt->size[0] * jpvt->size[1] - 1;
  for (cartesian_trajectory_planner_B.kend = 0;
       cartesian_trajectory_planner_B.kend <=
       cartesian_trajectory_planner_B.ix_m; cartesian_trajectory_planner_B.kend
       ++) {
    b_jpvt->data[cartesian_trajectory_planner_B.kend] = jpvt->
      data[cartesian_trajectory_planner_B.kend];
  }

  cartesian_trajectory_planner_B.kend = b_A->size[0] * b_A->size[1];
  b_A->size[0] = A->size[0];
  b_A->size[1] = A->size[1];
  cartes_emxEnsureCapacity_real_T(b_A, cartesian_trajectory_planner_B.kend);
  cartesian_trajectory_planner_B.ix_m = A->size[0] * A->size[1] - 1;
  for (cartesian_trajectory_planner_B.kend = 0;
       cartesian_trajectory_planner_B.kend <=
       cartesian_trajectory_planner_B.ix_m; cartesian_trajectory_planner_B.kend
       ++) {
    b_A->data[cartesian_trajectory_planner_B.kend] = A->
      data[cartesian_trajectory_planner_B.kend];
  }

  cartesian_trajec_emxInit_real_T(&work, 1);
  cartesian_trajectory_planner_B.ma = A->size[0];
  if (m < n) {
    cartesian_trajectory_planner_B.m_g = m;
  } else {
    cartesian_trajectory_planner_B.m_g = n;
  }

  cartesian_trajectory_planner_B.minmn_j = cartesian_trajectory_planner_B.m_g -
    1;
  cartesian_trajectory_planner_B.kend = work->size[0];
  work->size[0] = A->size[1];
  cartes_emxEnsureCapacity_real_T(work, cartesian_trajectory_planner_B.kend);
  cartesian_trajectory_planner_B.ix_m = A->size[1];
  for (cartesian_trajectory_planner_B.kend = 0;
       cartesian_trajectory_planner_B.kend < cartesian_trajectory_planner_B.ix_m;
       cartesian_trajectory_planner_B.kend++) {
    work->data[cartesian_trajectory_planner_B.kend] = 0.0;
  }

  cartesian_trajec_emxInit_real_T(&vn1, 1);
  cartesian_trajectory_planner_B.kend = vn1->size[0];
  vn1->size[0] = A->size[1];
  cartes_emxEnsureCapacity_real_T(vn1, cartesian_trajectory_planner_B.kend);
  cartesian_trajectory_planner_B.ix_m = A->size[1];
  for (cartesian_trajectory_planner_B.kend = 0;
       cartesian_trajectory_planner_B.kend < cartesian_trajectory_planner_B.ix_m;
       cartesian_trajectory_planner_B.kend++) {
    vn1->data[cartesian_trajectory_planner_B.kend] = 0.0;
  }

  cartesian_trajec_emxInit_real_T(&vn2, 1);
  cartesian_trajectory_planner_B.kend = vn2->size[0];
  vn2->size[0] = A->size[1];
  cartes_emxEnsureCapacity_real_T(vn2, cartesian_trajectory_planner_B.kend);
  cartesian_trajectory_planner_B.ix_m = A->size[1];
  for (cartesian_trajectory_planner_B.kend = 0;
       cartesian_trajectory_planner_B.kend < cartesian_trajectory_planner_B.ix_m;
       cartesian_trajectory_planner_B.kend++) {
    vn2->data[cartesian_trajectory_planner_B.kend] = 0.0;
  }

  for (cartesian_trajectory_planner_B.m_g = 0;
       cartesian_trajectory_planner_B.m_g < n;
       cartesian_trajectory_planner_B.m_g++) {
    cartesian_trajectory_planner_B.pvt = cartesian_trajectory_planner_B.m_g *
      cartesian_trajectory_planner_B.ma;
    cartesian_trajectory_planner_B.smax = 0.0;
    if (m >= 1) {
      if (m == 1) {
        cartesian_trajectory_planner_B.smax = fabs(A->
          data[cartesian_trajectory_planner_B.pvt]);
      } else {
        cartesian_trajectory_planner_B.scale_h = 3.3121686421112381E-170;
        cartesian_trajectory_planner_B.kend = cartesian_trajectory_planner_B.pvt
          + m;
        for (cartesian_trajectory_planner_B.itemp =
             cartesian_trajectory_planner_B.pvt + 1;
             cartesian_trajectory_planner_B.itemp <=
             cartesian_trajectory_planner_B.kend;
             cartesian_trajectory_planner_B.itemp++) {
          cartesian_trajectory_planner_B.absxk = fabs(A->
            data[cartesian_trajectory_planner_B.itemp - 1]);
          if (cartesian_trajectory_planner_B.absxk >
              cartesian_trajectory_planner_B.scale_h) {
            cartesian_trajectory_planner_B.t =
              cartesian_trajectory_planner_B.scale_h /
              cartesian_trajectory_planner_B.absxk;
            cartesian_trajectory_planner_B.smax =
              cartesian_trajectory_planner_B.smax *
              cartesian_trajectory_planner_B.t *
              cartesian_trajectory_planner_B.t + 1.0;
            cartesian_trajectory_planner_B.scale_h =
              cartesian_trajectory_planner_B.absxk;
          } else {
            cartesian_trajectory_planner_B.t =
              cartesian_trajectory_planner_B.absxk /
              cartesian_trajectory_planner_B.scale_h;
            cartesian_trajectory_planner_B.smax +=
              cartesian_trajectory_planner_B.t *
              cartesian_trajectory_planner_B.t;
          }
        }

        cartesian_trajectory_planner_B.smax =
          cartesian_trajectory_planner_B.scale_h * sqrt
          (cartesian_trajectory_planner_B.smax);
      }
    }

    vn1->data[cartesian_trajectory_planner_B.m_g] =
      cartesian_trajectory_planner_B.smax;
    vn2->data[cartesian_trajectory_planner_B.m_g] = vn1->
      data[cartesian_trajectory_planner_B.m_g];
  }

  cartesian_trajec_emxInit_real_T(&c_x, 2);
  for (cartesian_trajectory_planner_B.m_g = 0;
       cartesian_trajectory_planner_B.m_g <=
       cartesian_trajectory_planner_B.minmn_j;
       cartesian_trajectory_planner_B.m_g++) {
    cartesian_trajectory_planner_B.iy_i = cartesian_trajectory_planner_B.m_g *
      cartesian_trajectory_planner_B.ma;
    cartesian_trajectory_planner_B.ii = cartesian_trajectory_planner_B.iy_i +
      cartesian_trajectory_planner_B.m_g;
    cartesian_trajectory_planner_B.nmi = n - cartesian_trajectory_planner_B.m_g;
    cartesian_trajectory_planner_B.mmi = (m - cartesian_trajectory_planner_B.m_g)
      - 1;
    if (cartesian_trajectory_planner_B.nmi < 1) {
      cartesian_trajectory_planner_B.kend = 0;
    } else {
      cartesian_trajectory_planner_B.kend = 1;
      if (cartesian_trajectory_planner_B.nmi > 1) {
        cartesian_trajectory_planner_B.ix_m = cartesian_trajectory_planner_B.m_g;
        cartesian_trajectory_planner_B.smax = fabs(vn1->
          data[cartesian_trajectory_planner_B.m_g]);
        for (cartesian_trajectory_planner_B.itemp = 2;
             cartesian_trajectory_planner_B.itemp <=
             cartesian_trajectory_planner_B.nmi;
             cartesian_trajectory_planner_B.itemp++) {
          cartesian_trajectory_planner_B.ix_m++;
          cartesian_trajectory_planner_B.scale_h = fabs(vn1->
            data[cartesian_trajectory_planner_B.ix_m]);
          if (cartesian_trajectory_planner_B.scale_h >
              cartesian_trajectory_planner_B.smax) {
            cartesian_trajectory_planner_B.kend =
              cartesian_trajectory_planner_B.itemp;
            cartesian_trajectory_planner_B.smax =
              cartesian_trajectory_planner_B.scale_h;
          }
        }
      }
    }

    cartesian_trajectory_planner_B.pvt = (cartesian_trajectory_planner_B.m_g +
      cartesian_trajectory_planner_B.kend) - 1;
    if (cartesian_trajectory_planner_B.pvt + 1 !=
        cartesian_trajectory_planner_B.m_g + 1) {
      cartesian_trajectory_planner_B.kend = c_x->size[0] * c_x->size[1];
      c_x->size[0] = b_A->size[0];
      c_x->size[1] = b_A->size[1];
      cartes_emxEnsureCapacity_real_T(c_x, cartesian_trajectory_planner_B.kend);
      cartesian_trajectory_planner_B.ix_m = b_A->size[0] * b_A->size[1] - 1;
      for (cartesian_trajectory_planner_B.kend = 0;
           cartesian_trajectory_planner_B.kend <=
           cartesian_trajectory_planner_B.ix_m;
           cartesian_trajectory_planner_B.kend++) {
        c_x->data[cartesian_trajectory_planner_B.kend] = b_A->
          data[cartesian_trajectory_planner_B.kend];
      }

      cartesian_trajectory_planner_B.ix_m = cartesian_trajectory_planner_B.pvt *
        cartesian_trajectory_planner_B.ma;
      for (cartesian_trajectory_planner_B.itemp = 0;
           cartesian_trajectory_planner_B.itemp < m;
           cartesian_trajectory_planner_B.itemp++) {
        cartesian_trajectory_planner_B.scale_h = c_x->
          data[cartesian_trajectory_planner_B.ix_m];
        c_x->data[cartesian_trajectory_planner_B.ix_m] = c_x->
          data[cartesian_trajectory_planner_B.iy_i];
        c_x->data[cartesian_trajectory_planner_B.iy_i] =
          cartesian_trajectory_planner_B.scale_h;
        cartesian_trajectory_planner_B.ix_m++;
        cartesian_trajectory_planner_B.iy_i++;
      }

      cartesian_trajectory_planner_B.kend = b_A->size[0] * b_A->size[1];
      b_A->size[0] = c_x->size[0];
      b_A->size[1] = c_x->size[1];
      cartes_emxEnsureCapacity_real_T(b_A, cartesian_trajectory_planner_B.kend);
      cartesian_trajectory_planner_B.ix_m = c_x->size[0] * c_x->size[1] - 1;
      for (cartesian_trajectory_planner_B.kend = 0;
           cartesian_trajectory_planner_B.kend <=
           cartesian_trajectory_planner_B.ix_m;
           cartesian_trajectory_planner_B.kend++) {
        b_A->data[cartesian_trajectory_planner_B.kend] = c_x->
          data[cartesian_trajectory_planner_B.kend];
      }

      cartesian_trajectory_planner_B.itemp = b_jpvt->
        data[cartesian_trajectory_planner_B.pvt];
      b_jpvt->data[cartesian_trajectory_planner_B.pvt] = b_jpvt->
        data[cartesian_trajectory_planner_B.m_g];
      b_jpvt->data[cartesian_trajectory_planner_B.m_g] =
        cartesian_trajectory_planner_B.itemp;
      vn1->data[cartesian_trajectory_planner_B.pvt] = vn1->
        data[cartesian_trajectory_planner_B.m_g];
      vn2->data[cartesian_trajectory_planner_B.pvt] = vn2->
        data[cartesian_trajectory_planner_B.m_g];
    }

    if (cartesian_trajectory_planner_B.m_g + 1 < m) {
      cartesian_trajectory_planner_B.pvt = cartesian_trajectory_planner_B.ii + 2;
      cartesian_trajectory_planner_B.kend = c_x->size[0] * c_x->size[1];
      c_x->size[0] = b_A->size[0];
      c_x->size[1] = b_A->size[1];
      cartes_emxEnsureCapacity_real_T(c_x, cartesian_trajectory_planner_B.kend);
      cartesian_trajectory_planner_B.ix_m = b_A->size[0] * b_A->size[1] - 1;
      for (cartesian_trajectory_planner_B.kend = 0;
           cartesian_trajectory_planner_B.kend <=
           cartesian_trajectory_planner_B.ix_m;
           cartesian_trajectory_planner_B.kend++) {
        c_x->data[cartesian_trajectory_planner_B.kend] = b_A->
          data[cartesian_trajectory_planner_B.kend];
      }

      cartesian_trajectory_planner_B.smax = b_A->
        data[cartesian_trajectory_planner_B.ii];
      tau->data[cartesian_trajectory_planner_B.m_g] = 0.0;
      if (cartesian_trajectory_planner_B.mmi + 1 > 0) {
        cartesian_trajectory_planner_B.scale_h = cartesian_trajectory_p_xnrm2_as
          (cartesian_trajectory_planner_B.mmi, b_A,
           cartesian_trajectory_planner_B.ii + 2);
        if (cartesian_trajectory_planner_B.scale_h != 0.0) {
          cartesian_trajectory_planner_B.scale_h = rt_hypotd_snf(b_A->
            data[cartesian_trajectory_planner_B.ii],
            cartesian_trajectory_planner_B.scale_h);
          if (b_A->data[cartesian_trajectory_planner_B.ii] >= 0.0) {
            cartesian_trajectory_planner_B.scale_h =
              -cartesian_trajectory_planner_B.scale_h;
          }

          if (fabs(cartesian_trajectory_planner_B.scale_h) <
              1.0020841800044864E-292) {
            cartesian_trajectory_planner_B.kend = -1;
            cartesian_trajectory_planner_B.ix_m =
              (cartesian_trajectory_planner_B.ii +
               cartesian_trajectory_planner_B.mmi) + 1;
            do {
              cartesian_trajectory_planner_B.kend++;
              for (cartesian_trajectory_planner_B.itemp =
                   cartesian_trajectory_planner_B.pvt;
                   cartesian_trajectory_planner_B.itemp <=
                   cartesian_trajectory_planner_B.ix_m;
                   cartesian_trajectory_planner_B.itemp++) {
                c_x->data[cartesian_trajectory_planner_B.itemp - 1] *=
                  9.9792015476736E+291;
              }

              cartesian_trajectory_planner_B.scale_h *= 9.9792015476736E+291;
              cartesian_trajectory_planner_B.smax *= 9.9792015476736E+291;
            } while (!(fabs(cartesian_trajectory_planner_B.scale_h) >=
                       1.0020841800044864E-292));

            cartesian_trajectory_planner_B.scale_h = rt_hypotd_snf
              (cartesian_trajectory_planner_B.smax,
               cartesian_trajectory_p_xnrm2_as
               (cartesian_trajectory_planner_B.mmi, c_x,
                cartesian_trajectory_planner_B.ii + 2));
            if (cartesian_trajectory_planner_B.smax >= 0.0) {
              cartesian_trajectory_planner_B.scale_h =
                -cartesian_trajectory_planner_B.scale_h;
            }

            tau->data[cartesian_trajectory_planner_B.m_g] =
              (cartesian_trajectory_planner_B.scale_h -
               cartesian_trajectory_planner_B.smax) /
              cartesian_trajectory_planner_B.scale_h;
            cartesian_trajectory_planner_B.smax = 1.0 /
              (cartesian_trajectory_planner_B.smax -
               cartesian_trajectory_planner_B.scale_h);
            for (cartesian_trajectory_planner_B.itemp =
                 cartesian_trajectory_planner_B.pvt;
                 cartesian_trajectory_planner_B.itemp <=
                 cartesian_trajectory_planner_B.ix_m;
                 cartesian_trajectory_planner_B.itemp++) {
              c_x->data[cartesian_trajectory_planner_B.itemp - 1] *=
                cartesian_trajectory_planner_B.smax;
            }

            for (cartesian_trajectory_planner_B.itemp = 0;
                 cartesian_trajectory_planner_B.itemp <=
                 cartesian_trajectory_planner_B.kend;
                 cartesian_trajectory_planner_B.itemp++) {
              cartesian_trajectory_planner_B.scale_h *= 1.0020841800044864E-292;
            }

            cartesian_trajectory_planner_B.smax =
              cartesian_trajectory_planner_B.scale_h;
          } else {
            tau->data[cartesian_trajectory_planner_B.m_g] =
              (cartesian_trajectory_planner_B.scale_h - b_A->
               data[cartesian_trajectory_planner_B.ii]) /
              cartesian_trajectory_planner_B.scale_h;
            cartesian_trajectory_planner_B.smax = 1.0 / (b_A->
              data[cartesian_trajectory_planner_B.ii] -
              cartesian_trajectory_planner_B.scale_h);
            cartesian_trajectory_planner_B.kend = c_x->size[0] * c_x->size[1];
            c_x->size[0] = b_A->size[0];
            c_x->size[1] = b_A->size[1];
            cartes_emxEnsureCapacity_real_T(c_x,
              cartesian_trajectory_planner_B.kend);
            cartesian_trajectory_planner_B.ix_m = b_A->size[0] * b_A->size[1] -
              1;
            for (cartesian_trajectory_planner_B.kend = 0;
                 cartesian_trajectory_planner_B.kend <=
                 cartesian_trajectory_planner_B.ix_m;
                 cartesian_trajectory_planner_B.kend++) {
              c_x->data[cartesian_trajectory_planner_B.kend] = b_A->
                data[cartesian_trajectory_planner_B.kend];
            }

            cartesian_trajectory_planner_B.b_oq =
              (cartesian_trajectory_planner_B.ii +
               cartesian_trajectory_planner_B.mmi) + 1;
            for (cartesian_trajectory_planner_B.itemp =
                 cartesian_trajectory_planner_B.pvt;
                 cartesian_trajectory_planner_B.itemp <=
                 cartesian_trajectory_planner_B.b_oq;
                 cartesian_trajectory_planner_B.itemp++) {
              c_x->data[cartesian_trajectory_planner_B.itemp - 1] *=
                cartesian_trajectory_planner_B.smax;
            }

            cartesian_trajectory_planner_B.smax =
              cartesian_trajectory_planner_B.scale_h;
          }
        }
      }

      cartesian_trajectory_planner_B.kend = b_A->size[0] * b_A->size[1];
      b_A->size[0] = c_x->size[0];
      b_A->size[1] = c_x->size[1];
      cartes_emxEnsureCapacity_real_T(b_A, cartesian_trajectory_planner_B.kend);
      cartesian_trajectory_planner_B.ix_m = c_x->size[0] * c_x->size[1] - 1;
      for (cartesian_trajectory_planner_B.kend = 0;
           cartesian_trajectory_planner_B.kend <=
           cartesian_trajectory_planner_B.ix_m;
           cartesian_trajectory_planner_B.kend++) {
        b_A->data[cartesian_trajectory_planner_B.kend] = c_x->
          data[cartesian_trajectory_planner_B.kend];
      }

      b_A->data[cartesian_trajectory_planner_B.ii] =
        cartesian_trajectory_planner_B.smax;
    } else {
      tau->data[cartesian_trajectory_planner_B.m_g] = 0.0;
    }

    if (cartesian_trajectory_planner_B.m_g + 1 < n) {
      cartesian_trajectory_planner_B.smax = b_A->
        data[cartesian_trajectory_planner_B.ii];
      b_A->data[cartesian_trajectory_planner_B.ii] = 1.0;
      cartesian_trajectory_planner_B.pvt = (cartesian_trajectory_planner_B.ii +
        cartesian_trajectory_planner_B.ma) + 1;
      cartesian_trajectory_planner_B.kend = c_x->size[0] * c_x->size[1];
      c_x->size[0] = b_A->size[0];
      c_x->size[1] = b_A->size[1];
      cartes_emxEnsureCapacity_real_T(c_x, cartesian_trajectory_planner_B.kend);
      cartesian_trajectory_planner_B.ix_m = b_A->size[0] * b_A->size[1] - 1;
      for (cartesian_trajectory_planner_B.kend = 0;
           cartesian_trajectory_planner_B.kend <=
           cartesian_trajectory_planner_B.ix_m;
           cartesian_trajectory_planner_B.kend++) {
        c_x->data[cartesian_trajectory_planner_B.kend] = b_A->
          data[cartesian_trajectory_planner_B.kend];
      }

      if (tau->data[cartesian_trajectory_planner_B.m_g] != 0.0) {
        cartesian_trajectory_planner_B.itemp =
          cartesian_trajectory_planner_B.mmi;
        cartesian_trajectory_planner_B.kend = cartesian_trajectory_planner_B.ii
          + cartesian_trajectory_planner_B.mmi;
        while ((cartesian_trajectory_planner_B.itemp + 1 > 0) && (b_A->
                data[cartesian_trajectory_planner_B.kend] == 0.0)) {
          cartesian_trajectory_planner_B.itemp--;
          cartesian_trajectory_planner_B.kend--;
        }

        cartesian_trajectory_planner_B.nmi--;
        exitg2 = false;
        while ((!exitg2) && (cartesian_trajectory_planner_B.nmi > 0)) {
          cartesian_trajectory_planner_B.ix_m =
            (cartesian_trajectory_planner_B.nmi - 1) *
            cartesian_trajectory_planner_B.ma +
            cartesian_trajectory_planner_B.pvt;
          cartesian_trajectory_planner_B.kend =
            cartesian_trajectory_planner_B.ix_m;
          do {
            exitg1 = 0;
            if (cartesian_trajectory_planner_B.kend <=
                cartesian_trajectory_planner_B.ix_m +
                cartesian_trajectory_planner_B.itemp) {
              if (b_A->data[cartesian_trajectory_planner_B.kend - 1] != 0.0) {
                exitg1 = 1;
              } else {
                cartesian_trajectory_planner_B.kend++;
              }
            } else {
              cartesian_trajectory_planner_B.nmi--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }

        cartesian_trajectory_planner_B.lastc =
          cartesian_trajectory_planner_B.nmi - 1;
        cartesian_trajectory_planner_B.kend = c_x->size[0] * c_x->size[1];
        c_x->size[0] = b_A->size[0];
        c_x->size[1] = b_A->size[1];
        cartes_emxEnsureCapacity_real_T(c_x, cartesian_trajectory_planner_B.kend);
        cartesian_trajectory_planner_B.ix_m = b_A->size[0] * b_A->size[1] - 1;
        for (cartesian_trajectory_planner_B.kend = 0;
             cartesian_trajectory_planner_B.kend <=
             cartesian_trajectory_planner_B.ix_m;
             cartesian_trajectory_planner_B.kend++) {
          c_x->data[cartesian_trajectory_planner_B.kend] = b_A->
            data[cartesian_trajectory_planner_B.kend];
        }
      } else {
        cartesian_trajectory_planner_B.itemp = -1;
        cartesian_trajectory_planner_B.lastc = -1;
      }

      if (cartesian_trajectory_planner_B.itemp + 1 > 0) {
        if (cartesian_trajectory_planner_B.lastc + 1 != 0) {
          for (cartesian_trajectory_planner_B.kend = 0;
               cartesian_trajectory_planner_B.kend <=
               cartesian_trajectory_planner_B.lastc;
               cartesian_trajectory_planner_B.kend++) {
            work->data[cartesian_trajectory_planner_B.kend] = 0.0;
          }

          cartesian_trajectory_planner_B.iy_i = 0;
          cartesian_trajectory_planner_B.b_oq =
            cartesian_trajectory_planner_B.ma *
            cartesian_trajectory_planner_B.lastc +
            cartesian_trajectory_planner_B.pvt;
          for (cartesian_trajectory_planner_B.nmi =
               cartesian_trajectory_planner_B.pvt;
               cartesian_trajectory_planner_B.ma < 0 ?
               cartesian_trajectory_planner_B.nmi >=
               cartesian_trajectory_planner_B.b_oq :
               cartesian_trajectory_planner_B.nmi <=
               cartesian_trajectory_planner_B.b_oq;
               cartesian_trajectory_planner_B.nmi +=
               cartesian_trajectory_planner_B.ma) {
            cartesian_trajectory_planner_B.ix_m =
              cartesian_trajectory_planner_B.ii;
            cartesian_trajectory_planner_B.scale_h = 0.0;
            cartesian_trajectory_planner_B.d_b =
              cartesian_trajectory_planner_B.nmi +
              cartesian_trajectory_planner_B.itemp;
            for (cartesian_trajectory_planner_B.kend =
                 cartesian_trajectory_planner_B.nmi;
                 cartesian_trajectory_planner_B.kend <=
                 cartesian_trajectory_planner_B.d_b;
                 cartesian_trajectory_planner_B.kend++) {
              cartesian_trajectory_planner_B.scale_h += c_x->
                data[cartesian_trajectory_planner_B.kend - 1] * c_x->
                data[cartesian_trajectory_planner_B.ix_m];
              cartesian_trajectory_planner_B.ix_m++;
            }

            work->data[cartesian_trajectory_planner_B.iy_i] +=
              cartesian_trajectory_planner_B.scale_h;
            cartesian_trajectory_planner_B.iy_i++;
          }
        }

        if (!(-tau->data[cartesian_trajectory_planner_B.m_g] == 0.0)) {
          cartesian_trajectory_planner_B.iy_i = 0;
          for (cartesian_trajectory_planner_B.kend = 0;
               cartesian_trajectory_planner_B.kend <=
               cartesian_trajectory_planner_B.lastc;
               cartesian_trajectory_planner_B.kend++) {
            if (work->data[cartesian_trajectory_planner_B.iy_i] != 0.0) {
              cartesian_trajectory_planner_B.scale_h = work->
                data[cartesian_trajectory_planner_B.iy_i] * -tau->
                data[cartesian_trajectory_planner_B.m_g];
              cartesian_trajectory_planner_B.ix_m =
                cartesian_trajectory_planner_B.ii;
              cartesian_trajectory_planner_B.b_oq =
                cartesian_trajectory_planner_B.itemp +
                cartesian_trajectory_planner_B.pvt;
              for (cartesian_trajectory_planner_B.nmi =
                   cartesian_trajectory_planner_B.pvt;
                   cartesian_trajectory_planner_B.nmi <=
                   cartesian_trajectory_planner_B.b_oq;
                   cartesian_trajectory_planner_B.nmi++) {
                c_x->data[cartesian_trajectory_planner_B.nmi - 1] += c_x->
                  data[cartesian_trajectory_planner_B.ix_m] *
                  cartesian_trajectory_planner_B.scale_h;
                cartesian_trajectory_planner_B.ix_m++;
              }
            }

            cartesian_trajectory_planner_B.iy_i++;
            cartesian_trajectory_planner_B.pvt +=
              cartesian_trajectory_planner_B.ma;
          }
        }
      }

      cartesian_trajectory_planner_B.kend = b_A->size[0] * b_A->size[1];
      b_A->size[0] = c_x->size[0];
      b_A->size[1] = c_x->size[1];
      cartes_emxEnsureCapacity_real_T(b_A, cartesian_trajectory_planner_B.kend);
      cartesian_trajectory_planner_B.ix_m = c_x->size[0] * c_x->size[1] - 1;
      for (cartesian_trajectory_planner_B.kend = 0;
           cartesian_trajectory_planner_B.kend <=
           cartesian_trajectory_planner_B.ix_m;
           cartesian_trajectory_planner_B.kend++) {
        b_A->data[cartesian_trajectory_planner_B.kend] = c_x->
          data[cartesian_trajectory_planner_B.kend];
      }

      b_A->data[cartesian_trajectory_planner_B.ii] =
        cartesian_trajectory_planner_B.smax;
    }

    for (cartesian_trajectory_planner_B.ii = cartesian_trajectory_planner_B.m_g
         + 2; cartesian_trajectory_planner_B.ii <= n;
         cartesian_trajectory_planner_B.ii++) {
      cartesian_trajectory_planner_B.pvt = ((cartesian_trajectory_planner_B.ii -
        1) * cartesian_trajectory_planner_B.ma +
        cartesian_trajectory_planner_B.m_g) + 1;
      if (vn1->data[cartesian_trajectory_planner_B.ii - 1] != 0.0) {
        cartesian_trajectory_planner_B.smax = fabs(b_A->
          data[cartesian_trajectory_planner_B.pvt - 1]) / vn1->
          data[cartesian_trajectory_planner_B.ii - 1];
        cartesian_trajectory_planner_B.smax = 1.0 -
          cartesian_trajectory_planner_B.smax *
          cartesian_trajectory_planner_B.smax;
        if (cartesian_trajectory_planner_B.smax < 0.0) {
          cartesian_trajectory_planner_B.smax = 0.0;
        }

        cartesian_trajectory_planner_B.scale_h = vn1->
          data[cartesian_trajectory_planner_B.ii - 1] / vn2->
          data[cartesian_trajectory_planner_B.ii - 1];
        cartesian_trajectory_planner_B.scale_h =
          cartesian_trajectory_planner_B.scale_h *
          cartesian_trajectory_planner_B.scale_h *
          cartesian_trajectory_planner_B.smax;
        if (cartesian_trajectory_planner_B.scale_h <= 1.4901161193847656E-8) {
          if (cartesian_trajectory_planner_B.m_g + 1 < m) {
            cartesian_trajectory_planner_B.smax = 0.0;
            if (cartesian_trajectory_planner_B.mmi >= 1) {
              if (cartesian_trajectory_planner_B.mmi == 1) {
                cartesian_trajectory_planner_B.smax = fabs(b_A->
                  data[cartesian_trajectory_planner_B.pvt]);
              } else {
                cartesian_trajectory_planner_B.scale_h = 3.3121686421112381E-170;
                cartesian_trajectory_planner_B.kend =
                  cartesian_trajectory_planner_B.pvt +
                  cartesian_trajectory_planner_B.mmi;
                for (cartesian_trajectory_planner_B.itemp =
                     cartesian_trajectory_planner_B.pvt + 1;
                     cartesian_trajectory_planner_B.itemp <=
                     cartesian_trajectory_planner_B.kend;
                     cartesian_trajectory_planner_B.itemp++) {
                  cartesian_trajectory_planner_B.absxk = fabs(b_A->
                    data[cartesian_trajectory_planner_B.itemp - 1]);
                  if (cartesian_trajectory_planner_B.absxk >
                      cartesian_trajectory_planner_B.scale_h) {
                    cartesian_trajectory_planner_B.t =
                      cartesian_trajectory_planner_B.scale_h /
                      cartesian_trajectory_planner_B.absxk;
                    cartesian_trajectory_planner_B.smax =
                      cartesian_trajectory_planner_B.smax *
                      cartesian_trajectory_planner_B.t *
                      cartesian_trajectory_planner_B.t + 1.0;
                    cartesian_trajectory_planner_B.scale_h =
                      cartesian_trajectory_planner_B.absxk;
                  } else {
                    cartesian_trajectory_planner_B.t =
                      cartesian_trajectory_planner_B.absxk /
                      cartesian_trajectory_planner_B.scale_h;
                    cartesian_trajectory_planner_B.smax +=
                      cartesian_trajectory_planner_B.t *
                      cartesian_trajectory_planner_B.t;
                  }
                }

                cartesian_trajectory_planner_B.smax =
                  cartesian_trajectory_planner_B.scale_h * sqrt
                  (cartesian_trajectory_planner_B.smax);
              }
            }

            vn1->data[cartesian_trajectory_planner_B.ii - 1] =
              cartesian_trajectory_planner_B.smax;
            vn2->data[cartesian_trajectory_planner_B.ii - 1] = vn1->
              data[cartesian_trajectory_planner_B.ii - 1];
          } else {
            vn1->data[cartesian_trajectory_planner_B.ii - 1] = 0.0;
            vn2->data[cartesian_trajectory_planner_B.ii - 1] = 0.0;
          }
        } else {
          vn1->data[cartesian_trajectory_planner_B.ii - 1] *= sqrt
            (cartesian_trajectory_planner_B.smax);
        }
      }
    }
  }

  cartesian_trajec_emxFree_real_T(&c_x);
  cartesian_trajec_emxFree_real_T(&vn2);
  cartesian_trajec_emxFree_real_T(&vn1);
  cartesian_trajec_emxFree_real_T(&work);
}

static void cartesian_trajectory_pl_xzgetrf(int32_T m, int32_T n, const
  emxArray_real_T_cartesian_tra_T *A, int32_T lda,
  emxArray_real_T_cartesian_tra_T *b_A, emxArray_int32_T_cartesian_tr_T *ipiv,
  int32_T *info)
{
  int32_T mmj;
  int32_T b;
  int32_T c;
  emxArray_real_T_cartesian_tra_T *c_x;
  int32_T ix;
  real_T smax;
  real_T s;
  int32_T iy;
  int32_T n_0;
  int32_T yk;
  int32_T k;
  int32_T jA;
  int32_T jy;
  int32_T c_0;
  int32_T c_tmp;
  k = b_A->size[0] * b_A->size[1];
  b_A->size[0] = A->size[0];
  b_A->size[1] = A->size[1];
  cartes_emxEnsureCapacity_real_T(b_A, k);
  iy = A->size[0] * A->size[1] - 1;
  for (k = 0; k <= iy; k++) {
    b_A->data[k] = A->data[k];
  }

  if (m < n) {
    n_0 = m;
  } else {
    n_0 = n;
  }

  if (n_0 < 1) {
    n_0 = 0;
  }

  k = ipiv->size[0] * ipiv->size[1];
  ipiv->size[0] = 1;
  ipiv->size[1] = n_0;
  carte_emxEnsureCapacity_int32_T(ipiv, k);
  if (n_0 > 0) {
    ipiv->data[0] = 1;
    yk = 1;
    for (k = 2; k <= n_0; k++) {
      yk++;
      ipiv->data[k - 1] = yk;
    }
  }

  yk = 0;
  cartesian_trajec_emxInit_real_T(&c_x, 2);
  if ((m < 1) || (n < 1)) {
  } else {
    n_0 = m - 1;
    if (n_0 >= n) {
      n_0 = n;
    }

    b = n_0 - 1;
    for (n_0 = 0; n_0 <= b; n_0++) {
      mmj = m - n_0;
      c_tmp = (lda + 1) * n_0;
      c = c_tmp + 2;
      if (mmj < 1) {
        iy = 0;
      } else {
        iy = 1;
        if (mmj > 1) {
          ix = c - 2;
          smax = fabs(b_A->data[c_tmp]);
          for (k = 2; k <= mmj; k++) {
            ix++;
            s = fabs(b_A->data[ix]);
            if (s > smax) {
              iy = k;
              smax = s;
            }
          }
        }
      }

      if (b_A->data[(c + iy) - 3] != 0.0) {
        if (iy - 1 != 0) {
          k = n_0 + iy;
          ipiv->data[n_0] = k;
          ix = c_x->size[0] * c_x->size[1];
          c_x->size[0] = b_A->size[0];
          c_x->size[1] = b_A->size[1];
          cartes_emxEnsureCapacity_real_T(c_x, ix);
          iy = b_A->size[0] * b_A->size[1] - 1;
          for (ix = 0; ix <= iy; ix++) {
            c_x->data[ix] = b_A->data[ix];
          }

          ix = n_0;
          iy = k - 1;
          for (k = 0; k < n; k++) {
            smax = c_x->data[ix];
            c_x->data[ix] = c_x->data[iy];
            c_x->data[iy] = smax;
            ix += lda;
            iy += lda;
          }

          k = b_A->size[0] * b_A->size[1];
          b_A->size[0] = c_x->size[0];
          b_A->size[1] = c_x->size[1];
          cartes_emxEnsureCapacity_real_T(b_A, k);
          iy = c_x->size[0] * c_x->size[1] - 1;
          for (k = 0; k <= iy; k++) {
            b_A->data[k] = c_x->data[k];
          }
        }

        iy = c + mmj;
        for (k = c; k <= iy - 2; k++) {
          b_A->data[k - 1] /= b_A->data[c_tmp];
        }
      } else {
        yk = n_0 + 1;
      }

      iy = (n - n_0) - 2;
      jy = c_tmp + lda;
      jA = jy + 1;
      for (k = 0; k <= iy; k++) {
        smax = b_A->data[jy];
        if (b_A->data[jy] != 0.0) {
          ix = c - 1;
          c_0 = (mmj + jA) - 1;
          for (c_tmp = jA + 1; c_tmp <= c_0; c_tmp++) {
            b_A->data[c_tmp - 1] += b_A->data[ix] * -smax;
            ix++;
          }
        }

        jy += lda;
        jA += lda;
      }
    }

    if ((yk == 0) && (m <= n) && (!(b_A->data[((m - 1) * b_A->size[0] + m) - 1]
          != 0.0))) {
      yk = m;
    }
  }

  cartesian_trajec_emxFree_real_T(&c_x);
  *info = yk;
}

static void cartesian_trajectory_plan_xtrsm(int32_T m, int32_T n, const
  emxArray_real_T_cartesian_tra_T *A, int32_T lda, const
  emxArray_real_T_cartesian_tra_T *B, int32_T ldb,
  emxArray_real_T_cartesian_tra_T *b_B)
{
  int32_T jBcol;
  int32_T kAcol;
  int32_T i;
  int32_T k;
  int32_T b;
  int32_T b_i;
  int32_T loop_ub;
  int32_T i_0;
  int32_T tmp;
  i_0 = b_B->size[0] * b_B->size[1];
  b_B->size[0] = B->size[0];
  b_B->size[1] = B->size[1];
  cartes_emxEnsureCapacity_real_T(b_B, i_0);
  loop_ub = B->size[0] * B->size[1] - 1;
  for (i_0 = 0; i_0 <= loop_ub; i_0++) {
    b_B->data[i_0] = B->data[i_0];
  }

  if ((n == 0) || ((B->size[0] == 0) || (B->size[1] == 0))) {
  } else {
    for (loop_ub = 0; loop_ub < n; loop_ub++) {
      jBcol = ldb * loop_ub - 1;
      for (k = m; k >= 1; k--) {
        kAcol = (k - 1) * lda - 1;
        i_0 = k + jBcol;
        if (b_B->data[i_0] != 0.0) {
          b_B->data[i_0] /= A->data[k + kAcol];
          b = k - 2;
          for (b_i = 0; b_i <= b; b_i++) {
            i = b_i + 1;
            tmp = i + jBcol;
            b_B->data[tmp] -= b_B->data[i_0] * A->data[i + kAcol];
          }
        }
      }
    }
  }
}

static void cartesian_traje_emxFree_int32_T(emxArray_int32_T_cartesian_tr_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_int32_T_cartesian_tr_T *)NULL) {
    if (((*pEmxArray)->data != (int32_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_int32_T_cartesian_tr_T *)NULL;
  }
}

static void cartesian_trajectory_p_mldivide(const
  emxArray_real_T_cartesian_tra_T *A, const emxArray_real_T_cartesian_tra_T *B,
  emxArray_real_T_cartesian_tra_T *Y)
{
  emxArray_real_T_cartesian_tra_T *c_A;
  emxArray_real_T_cartesian_tra_T *b_tau;
  emxArray_int32_T_cartesian_tr_T *b_jpvt;
  emxArray_real_T_cartesian_tra_T *B_0;
  emxArray_int32_T_cartesian_tr_T *b_jpvt_0;
  boolean_T guard1 = false;
  cartesian_trajec_emxInit_real_T(&c_A, 2);
  cartesian_trajec_emxInit_real_T(&b_tau, 1);
  cartesian_traje_emxInit_int32_T(&b_jpvt, 2);
  cartesian_trajec_emxInit_real_T(&B_0, 2);
  cartesian_traje_emxInit_int32_T(&b_jpvt_0, 2);
  if ((A->size[0] == 0) || (A->size[1] == 0) || ((B->size[0] == 0) || (B->size[1]
        == 0))) {
    cartesian_trajectory_planner_B.minmn = A->size[1];
    cartesian_trajectory_planner_B.minmana = B->size[1];
    cartesian_trajectory_planner_B.b_i_c = Y->size[0] * Y->size[1];
    Y->size[0] = cartesian_trajectory_planner_B.minmn;
    Y->size[1] = cartesian_trajectory_planner_B.minmana;
    cartes_emxEnsureCapacity_real_T(Y, cartesian_trajectory_planner_B.b_i_c);
    cartesian_trajectory_planner_B.minmn = cartesian_trajectory_planner_B.minmn *
      cartesian_trajectory_planner_B.minmana - 1;
    for (cartesian_trajectory_planner_B.b_i_c = 0;
         cartesian_trajectory_planner_B.b_i_c <=
         cartesian_trajectory_planner_B.minmn;
         cartesian_trajectory_planner_B.b_i_c++) {
      Y->data[cartesian_trajectory_planner_B.b_i_c] = 0.0;
    }
  } else if (A->size[0] == A->size[1]) {
    cartesian_trajectory_planner_B.minmn = A->size[0];
    cartesian_trajectory_planner_B.rankR = A->size[1];
    if (cartesian_trajectory_planner_B.minmn <
        cartesian_trajectory_planner_B.rankR) {
      cartesian_trajectory_planner_B.rankR =
        cartesian_trajectory_planner_B.minmn;
    }

    cartesian_trajectory_planner_B.minmn = B->size[0];
    if (cartesian_trajectory_planner_B.minmn <
        cartesian_trajectory_planner_B.rankR) {
      cartesian_trajectory_planner_B.rankR =
        cartesian_trajectory_planner_B.minmn;
    }

    cartesian_trajectory_planner_B.nb = B->size[1] - 1;
    cartesian_trajectory_pl_xzgetrf(cartesian_trajectory_planner_B.rankR,
      cartesian_trajectory_planner_B.rankR, A, A->size[0], c_A, b_jpvt,
      &cartesian_trajectory_planner_B.minmn);
    cartesian_trajectory_planner_B.b_i_c = B_0->size[0] * B_0->size[1];
    B_0->size[0] = B->size[0];
    B_0->size[1] = B->size[1];
    cartes_emxEnsureCapacity_real_T(B_0, cartesian_trajectory_planner_B.b_i_c);
    cartesian_trajectory_planner_B.minmn = B->size[0] * B->size[1] - 1;
    for (cartesian_trajectory_planner_B.b_i_c = 0;
         cartesian_trajectory_planner_B.b_i_c <=
         cartesian_trajectory_planner_B.minmn;
         cartesian_trajectory_planner_B.b_i_c++) {
      B_0->data[cartesian_trajectory_planner_B.b_i_c] = B->
        data[cartesian_trajectory_planner_B.b_i_c];
    }

    cartesian_trajectory_planner_B.minmn = cartesian_trajectory_planner_B.rankR
      - 2;
    for (cartesian_trajectory_planner_B.b_i_c = 0;
         cartesian_trajectory_planner_B.b_i_c <=
         cartesian_trajectory_planner_B.minmn;
         cartesian_trajectory_planner_B.b_i_c++) {
      if (cartesian_trajectory_planner_B.b_i_c + 1 != b_jpvt->
          data[cartesian_trajectory_planner_B.b_i_c]) {
        cartesian_trajectory_planner_B.na = b_jpvt->
          data[cartesian_trajectory_planner_B.b_i_c] - 1;
        for (cartesian_trajectory_planner_B.minmana = 0;
             cartesian_trajectory_planner_B.minmana <=
             cartesian_trajectory_planner_B.nb;
             cartesian_trajectory_planner_B.minmana++) {
          cartesian_trajectory_planner_B.tol_j = B_0->data[B_0->size[0] *
            cartesian_trajectory_planner_B.minmana +
            cartesian_trajectory_planner_B.b_i_c];
          B_0->data[cartesian_trajectory_planner_B.b_i_c + B_0->size[0] *
            cartesian_trajectory_planner_B.minmana] = B_0->data[B_0->size[0] *
            cartesian_trajectory_planner_B.minmana +
            cartesian_trajectory_planner_B.na];
          B_0->data[cartesian_trajectory_planner_B.na + B_0->size[0] *
            cartesian_trajectory_planner_B.minmana] =
            cartesian_trajectory_planner_B.tol_j;
        }
      }
    }

    if ((B->size[1] == 0) || ((B_0->size[0] == 0) || (B_0->size[1] == 0))) {
    } else {
      for (cartesian_trajectory_planner_B.minmana = 0;
           cartesian_trajectory_planner_B.minmana <=
           cartesian_trajectory_planner_B.nb;
           cartesian_trajectory_planner_B.minmana++) {
        cartesian_trajectory_planner_B.m_d4 = B->size[0] *
          cartesian_trajectory_planner_B.minmana - 1;
        for (cartesian_trajectory_planner_B.minmn = 0;
             cartesian_trajectory_planner_B.minmn <
             cartesian_trajectory_planner_B.rankR;
             cartesian_trajectory_planner_B.minmn++) {
          cartesian_trajectory_planner_B.nb_k = c_A->size[0] *
            cartesian_trajectory_planner_B.minmn - 1;
          cartesian_trajectory_planner_B.b_i_c =
            (cartesian_trajectory_planner_B.minmn +
             cartesian_trajectory_planner_B.m_d4) + 1;
          if (B_0->data[cartesian_trajectory_planner_B.b_i_c] != 0.0) {
            for (cartesian_trajectory_planner_B.na =
                 cartesian_trajectory_planner_B.minmn + 2;
                 cartesian_trajectory_planner_B.na <=
                 cartesian_trajectory_planner_B.rankR;
                 cartesian_trajectory_planner_B.na++) {
              cartesian_trajectory_planner_B.mn =
                cartesian_trajectory_planner_B.na +
                cartesian_trajectory_planner_B.m_d4;
              B_0->data[cartesian_trajectory_planner_B.mn] -= B_0->
                data[cartesian_trajectory_planner_B.b_i_c] * c_A->
                data[cartesian_trajectory_planner_B.na +
                cartesian_trajectory_planner_B.nb_k];
            }
          }
        }
      }
    }

    cartesian_trajectory_plan_xtrsm(cartesian_trajectory_planner_B.rankR,
      B->size[1], c_A, c_A->size[0], B_0, B->size[0], Y);
  } else {
    cartesian_trajectory_planner_B.na = A->size[1] - 1;
    cartesian_trajectory_planner_B.b_i_c = c_A->size[0] * c_A->size[1];
    c_A->size[0] = A->size[0];
    c_A->size[1] = A->size[1];
    cartes_emxEnsureCapacity_real_T(c_A, cartesian_trajectory_planner_B.b_i_c);
    cartesian_trajectory_planner_B.minmn = A->size[0] * A->size[1] - 1;
    for (cartesian_trajectory_planner_B.b_i_c = 0;
         cartesian_trajectory_planner_B.b_i_c <=
         cartesian_trajectory_planner_B.minmn;
         cartesian_trajectory_planner_B.b_i_c++) {
      c_A->data[cartesian_trajectory_planner_B.b_i_c] = A->
        data[cartesian_trajectory_planner_B.b_i_c];
    }

    cartesian_trajectory_planner_B.minmn = A->size[0];
    cartesian_trajectory_planner_B.minmana = A->size[1];
    if (cartesian_trajectory_planner_B.minmn <
        cartesian_trajectory_planner_B.minmana) {
      cartesian_trajectory_planner_B.minmana =
        cartesian_trajectory_planner_B.minmn;
    }

    cartesian_trajectory_planner_B.b_i_c = b_tau->size[0];
    b_tau->size[0] = cartesian_trajectory_planner_B.minmana;
    cartes_emxEnsureCapacity_real_T(b_tau, cartesian_trajectory_planner_B.b_i_c);
    for (cartesian_trajectory_planner_B.b_i_c = 0;
         cartesian_trajectory_planner_B.b_i_c <
         cartesian_trajectory_planner_B.minmana;
         cartesian_trajectory_planner_B.b_i_c++) {
      b_tau->data[cartesian_trajectory_planner_B.b_i_c] = 0.0;
    }

    guard1 = false;
    if ((A->size[0] == 0) || (A->size[1] == 0)) {
      guard1 = true;
    } else {
      cartesian_trajectory_planner_B.minmn = A->size[0];
      cartesian_trajectory_planner_B.minmana = A->size[1];
      if (cartesian_trajectory_planner_B.minmn <
          cartesian_trajectory_planner_B.minmana) {
        cartesian_trajectory_planner_B.minmana =
          cartesian_trajectory_planner_B.minmn;
      }

      if (cartesian_trajectory_planner_B.minmana < 1) {
        guard1 = true;
      } else {
        cartesian_trajectory_planner_B.b_i_c = b_jpvt->size[0] * b_jpvt->size[1];
        b_jpvt->size[0] = 1;
        b_jpvt->size[1] = A->size[1];
        carte_emxEnsureCapacity_int32_T(b_jpvt,
          cartesian_trajectory_planner_B.b_i_c);
        cartesian_trajectory_planner_B.minmn = A->size[1] - 1;
        for (cartesian_trajectory_planner_B.b_i_c = 0;
             cartesian_trajectory_planner_B.b_i_c <=
             cartesian_trajectory_planner_B.minmn;
             cartesian_trajectory_planner_B.b_i_c++) {
          b_jpvt->data[cartesian_trajectory_planner_B.b_i_c] = 0;
        }

        for (cartesian_trajectory_planner_B.minmn = 0;
             cartesian_trajectory_planner_B.minmn <=
             cartesian_trajectory_planner_B.na;
             cartesian_trajectory_planner_B.minmn++) {
          b_jpvt->data[cartesian_trajectory_planner_B.minmn] =
            cartesian_trajectory_planner_B.minmn + 1;
        }

        cartesian_trajectory_planner_B.b_i_c = b_jpvt_0->size[0] *
          b_jpvt_0->size[1];
        b_jpvt_0->size[0] = 1;
        b_jpvt_0->size[1] = b_jpvt->size[1];
        carte_emxEnsureCapacity_int32_T(b_jpvt_0,
          cartesian_trajectory_planner_B.b_i_c);
        cartesian_trajectory_planner_B.minmn = b_jpvt->size[0] * b_jpvt->size[1];
        for (cartesian_trajectory_planner_B.b_i_c = 0;
             cartesian_trajectory_planner_B.b_i_c <
             cartesian_trajectory_planner_B.minmn;
             cartesian_trajectory_planner_B.b_i_c++) {
          b_jpvt_0->data[cartesian_trajectory_planner_B.b_i_c] = b_jpvt->
            data[cartesian_trajectory_planner_B.b_i_c];
        }

        cartesian_trajectory_pla_qrpf_a(A, A->size[0], A->size[1], b_tau,
          b_jpvt_0, c_A, b_jpvt);
      }
    }

    if (guard1) {
      cartesian_trajectory_planner_B.b_i_c = b_jpvt->size[0] * b_jpvt->size[1];
      b_jpvt->size[0] = 1;
      b_jpvt->size[1] = A->size[1];
      carte_emxEnsureCapacity_int32_T(b_jpvt,
        cartesian_trajectory_planner_B.b_i_c);
      cartesian_trajectory_planner_B.minmn = A->size[1] - 1;
      for (cartesian_trajectory_planner_B.b_i_c = 0;
           cartesian_trajectory_planner_B.b_i_c <=
           cartesian_trajectory_planner_B.minmn;
           cartesian_trajectory_planner_B.b_i_c++) {
        b_jpvt->data[cartesian_trajectory_planner_B.b_i_c] = 0;
      }

      for (cartesian_trajectory_planner_B.minmana = 0;
           cartesian_trajectory_planner_B.minmana <=
           cartesian_trajectory_planner_B.na;
           cartesian_trajectory_planner_B.minmana++) {
        b_jpvt->data[cartesian_trajectory_planner_B.minmana] =
          cartesian_trajectory_planner_B.minmana + 1;
      }
    }

    cartesian_trajectory_planner_B.rankR = 0;
    if (c_A->size[0] < c_A->size[1]) {
      cartesian_trajectory_planner_B.minmn = c_A->size[0];
      cartesian_trajectory_planner_B.minmana = c_A->size[1];
    } else {
      cartesian_trajectory_planner_B.minmn = c_A->size[1];
      cartesian_trajectory_planner_B.minmana = c_A->size[0];
    }

    if (cartesian_trajectory_planner_B.minmn > 0) {
      cartesian_trajectory_planner_B.tol_j = 2.2204460492503131E-15 *
        static_cast<real_T>(cartesian_trajectory_planner_B.minmana);
      if (1.4901161193847656E-8 < cartesian_trajectory_planner_B.tol_j) {
        cartesian_trajectory_planner_B.tol_j = 1.4901161193847656E-8;
      }

      cartesian_trajectory_planner_B.tol_j *= fabs(c_A->data[0]);
      while ((cartesian_trajectory_planner_B.rankR <
              cartesian_trajectory_planner_B.minmn) && (!(fabs(c_A->data
                [c_A->size[0] * cartesian_trajectory_planner_B.rankR +
                cartesian_trajectory_planner_B.rankR]) <=
               cartesian_trajectory_planner_B.tol_j))) {
        cartesian_trajectory_planner_B.rankR++;
      }
    }

    cartesian_trajectory_planner_B.nb = B->size[1] - 1;
    cartesian_trajectory_planner_B.minmn = c_A->size[1];
    cartesian_trajectory_planner_B.minmana = B->size[1];
    cartesian_trajectory_planner_B.b_i_c = Y->size[0] * Y->size[1];
    Y->size[0] = cartesian_trajectory_planner_B.minmn;
    Y->size[1] = cartesian_trajectory_planner_B.minmana;
    cartes_emxEnsureCapacity_real_T(Y, cartesian_trajectory_planner_B.b_i_c);
    cartesian_trajectory_planner_B.minmn = cartesian_trajectory_planner_B.minmn *
      cartesian_trajectory_planner_B.minmana - 1;
    for (cartesian_trajectory_planner_B.b_i_c = 0;
         cartesian_trajectory_planner_B.b_i_c <=
         cartesian_trajectory_planner_B.minmn;
         cartesian_trajectory_planner_B.b_i_c++) {
      Y->data[cartesian_trajectory_planner_B.b_i_c] = 0.0;
    }

    cartesian_trajectory_planner_B.b_i_c = B_0->size[0] * B_0->size[1];
    B_0->size[0] = B->size[0];
    B_0->size[1] = B->size[1];
    cartes_emxEnsureCapacity_real_T(B_0, cartesian_trajectory_planner_B.b_i_c);
    cartesian_trajectory_planner_B.minmn = B->size[0] * B->size[1] - 1;
    for (cartesian_trajectory_planner_B.b_i_c = 0;
         cartesian_trajectory_planner_B.b_i_c <=
         cartesian_trajectory_planner_B.minmn;
         cartesian_trajectory_planner_B.b_i_c++) {
      B_0->data[cartesian_trajectory_planner_B.b_i_c] = B->
        data[cartesian_trajectory_planner_B.b_i_c];
    }

    cartesian_trajectory_planner_B.m_d4 = c_A->size[0];
    cartesian_trajectory_planner_B.nb_k = B->size[1] - 1;
    cartesian_trajectory_planner_B.minmn = c_A->size[0];
    cartesian_trajectory_planner_B.minmana = c_A->size[1];
    if (cartesian_trajectory_planner_B.minmn <
        cartesian_trajectory_planner_B.minmana) {
      cartesian_trajectory_planner_B.minmana =
        cartesian_trajectory_planner_B.minmn;
    }

    cartesian_trajectory_planner_B.mn = cartesian_trajectory_planner_B.minmana -
      1;
    for (cartesian_trajectory_planner_B.minmana = 0;
         cartesian_trajectory_planner_B.minmana <=
         cartesian_trajectory_planner_B.mn;
         cartesian_trajectory_planner_B.minmana++) {
      if (b_tau->data[cartesian_trajectory_planner_B.minmana] != 0.0) {
        for (cartesian_trajectory_planner_B.minmn = 0;
             cartesian_trajectory_planner_B.minmn <=
             cartesian_trajectory_planner_B.nb_k;
             cartesian_trajectory_planner_B.minmn++) {
          cartesian_trajectory_planner_B.tol_j = B_0->data[B_0->size[0] *
            cartesian_trajectory_planner_B.minmn +
            cartesian_trajectory_planner_B.minmana];
          for (cartesian_trajectory_planner_B.na =
               cartesian_trajectory_planner_B.minmana + 2;
               cartesian_trajectory_planner_B.na <=
               cartesian_trajectory_planner_B.m_d4;
               cartesian_trajectory_planner_B.na++) {
            cartesian_trajectory_planner_B.tol_j += c_A->data[(c_A->size[0] *
              cartesian_trajectory_planner_B.minmana +
              cartesian_trajectory_planner_B.na) - 1] * B_0->data[(B_0->size[0] *
              cartesian_trajectory_planner_B.minmn +
              cartesian_trajectory_planner_B.na) - 1];
          }

          cartesian_trajectory_planner_B.tol_j *= b_tau->
            data[cartesian_trajectory_planner_B.minmana];
          if (cartesian_trajectory_planner_B.tol_j != 0.0) {
            B_0->data[cartesian_trajectory_planner_B.minmana + B_0->size[0] *
              cartesian_trajectory_planner_B.minmn] -=
              cartesian_trajectory_planner_B.tol_j;
            for (cartesian_trajectory_planner_B.b_i_c =
                 cartesian_trajectory_planner_B.minmana + 2;
                 cartesian_trajectory_planner_B.b_i_c <=
                 cartesian_trajectory_planner_B.m_d4;
                 cartesian_trajectory_planner_B.b_i_c++) {
              B_0->data[(cartesian_trajectory_planner_B.b_i_c + B_0->size[0] *
                         cartesian_trajectory_planner_B.minmn) - 1] -= c_A->
                data[(c_A->size[0] * cartesian_trajectory_planner_B.minmana +
                      cartesian_trajectory_planner_B.b_i_c) - 1] *
                cartesian_trajectory_planner_B.tol_j;
            }
          }
        }
      }
    }

    for (cartesian_trajectory_planner_B.minmn = 0;
         cartesian_trajectory_planner_B.minmn <=
         cartesian_trajectory_planner_B.nb; cartesian_trajectory_planner_B.minmn
         ++) {
      for (cartesian_trajectory_planner_B.b_i_c = 0;
           cartesian_trajectory_planner_B.b_i_c <
           cartesian_trajectory_planner_B.rankR;
           cartesian_trajectory_planner_B.b_i_c++) {
        Y->data[(b_jpvt->data[cartesian_trajectory_planner_B.b_i_c] + Y->size[0]
                 * cartesian_trajectory_planner_B.minmn) - 1] = B_0->data
          [B_0->size[0] * cartesian_trajectory_planner_B.minmn +
          cartesian_trajectory_planner_B.b_i_c];
      }

      for (cartesian_trajectory_planner_B.minmana =
           cartesian_trajectory_planner_B.rankR;
           cartesian_trajectory_planner_B.minmana >= 1;
           cartesian_trajectory_planner_B.minmana--) {
        Y->data[(b_jpvt->data[cartesian_trajectory_planner_B.minmana - 1] +
                 Y->size[0] * cartesian_trajectory_planner_B.minmn) - 1] /=
          c_A->data[((cartesian_trajectory_planner_B.minmana - 1) * c_A->size[0]
                     + cartesian_trajectory_planner_B.minmana) - 1];
        cartesian_trajectory_planner_B.na =
          cartesian_trajectory_planner_B.minmana - 2;
        for (cartesian_trajectory_planner_B.b_i_c = 0;
             cartesian_trajectory_planner_B.b_i_c <=
             cartesian_trajectory_planner_B.na;
             cartesian_trajectory_planner_B.b_i_c++) {
          Y->data[(b_jpvt->data[cartesian_trajectory_planner_B.b_i_c] + Y->size
                   [0] * cartesian_trajectory_planner_B.minmn) - 1] -= Y->data
            [(b_jpvt->data[cartesian_trajectory_planner_B.minmana - 1] + Y->
              size[0] * cartesian_trajectory_planner_B.minmn) - 1] * c_A->data
            [(cartesian_trajectory_planner_B.minmana - 1) * c_A->size[0] +
            cartesian_trajectory_planner_B.b_i_c];
        }
      }
    }
  }

  cartesian_traje_emxFree_int32_T(&b_jpvt_0);
  cartesian_trajec_emxFree_real_T(&B_0);
  cartesian_traje_emxFree_int32_T(&b_jpvt);
  cartesian_trajec_emxFree_real_T(&b_tau);
  cartesian_trajec_emxFree_real_T(&c_A);
}

static void cartesian_tra_emxFree_boolean_T(emxArray_boolean_T_cartesian__T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_boolean_T_cartesian__T *)NULL) {
    if (((*pEmxArray)->data != (boolean_T *)NULL) && (*pEmxArray)->canFreeData)
    {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_boolean_T_cartesian__T *)NULL;
  }
}

static boolean_T DampedBFGSwGradientProjection_a(const
  h_robotics_core_internal_Damp_T *obj, const real_T Hg[6], const
  emxArray_real_T_cartesian_tra_T *alpha)
{
  boolean_T flag;
  boolean_T y;
  emxArray_boolean_T_cartesian__T *x;
  int32_T ix;
  int32_T loop_ub;
  boolean_T exitg1;
  cartesian_tra_emxInit_boolean_T(&x, 1);
  if (cartesian_trajectory_pla_norm_a(Hg) < obj->GradientTolerance) {
    ix = x->size[0];
    x->size[0] = alpha->size[0];
    car_emxEnsureCapacity_boolean_T(x, ix);
    loop_ub = alpha->size[0];
    for (ix = 0; ix < loop_ub; ix++) {
      x->data[ix] = (alpha->data[ix] <= 0.0);
    }

    y = true;
    ix = 0;
    exitg1 = false;
    while ((!exitg1) && (ix + 1 <= x->size[0])) {
      if (!x->data[ix]) {
        y = false;
        exitg1 = true;
      } else {
        ix++;
      }
    }

    if (y) {
      flag = true;
    } else {
      flag = false;
    }
  } else {
    flag = false;
  }

  cartesian_tra_emxFree_boolean_T(&x);
  return flag;
}

static void cartesian_trajectory_planne_inv(const
  emxArray_real_T_cartesian_tra_T *x, emxArray_real_T_cartesian_tra_T *y)
{
  int32_T n;
  emxArray_int32_T_cartesian_tr_T *p;
  int32_T c;
  emxArray_real_T_cartesian_tra_T *c_A;
  emxArray_int32_T_cartesian_tr_T *b_ipiv;
  int32_T info;
  int32_T n_0;
  int32_T yk;
  emxArray_real_T_cartesian_tra_T *y_0;
  if ((x->size[0] == 0) || (x->size[1] == 0)) {
    info = y->size[0] * y->size[1];
    y->size[0] = x->size[0];
    y->size[1] = x->size[1];
    cartes_emxEnsureCapacity_real_T(y, info);
    n_0 = x->size[0] * x->size[1] - 1;
    for (info = 0; info <= n_0; info++) {
      y->data[info] = x->data[info];
    }
  } else {
    n = x->size[0];
    info = y->size[0] * y->size[1];
    y->size[0] = x->size[0];
    y->size[1] = x->size[1];
    cartes_emxEnsureCapacity_real_T(y, info);
    n_0 = x->size[0] * x->size[1] - 1;
    for (info = 0; info <= n_0; info++) {
      y->data[info] = 0.0;
    }

    cartesian_traje_emxInit_int32_T(&p, 2);
    cartesian_trajec_emxInit_real_T(&c_A, 2);
    cartesian_traje_emxInit_int32_T(&b_ipiv, 2);
    cartesian_trajectory_pl_xzgetrf(x->size[0], x->size[0], x, x->size[0], c_A,
      b_ipiv, &info);
    if (x->size[0] < 1) {
      n_0 = 0;
    } else {
      n_0 = x->size[0];
    }

    info = p->size[0] * p->size[1];
    p->size[0] = 1;
    p->size[1] = n_0;
    carte_emxEnsureCapacity_int32_T(p, info);
    if (n_0 > 0) {
      p->data[0] = 1;
      yk = 1;
      for (info = 2; info <= n_0; info++) {
        yk++;
        p->data[info - 1] = yk;
      }
    }

    n_0 = b_ipiv->size[1] - 1;
    for (info = 0; info <= n_0; info++) {
      if (b_ipiv->data[info] > static_cast<real_T>(info) + 1.0) {
        yk = p->data[b_ipiv->data[info] - 1];
        p->data[b_ipiv->data[info] - 1] = p->data[info];
        p->data[info] = yk;
      }
    }

    cartesian_traje_emxFree_int32_T(&b_ipiv);
    for (info = 0; info < n; info++) {
      c = p->data[info] - 1;
      y->data[info + y->size[0] * (p->data[info] - 1)] = 1.0;
      for (n_0 = info + 1; n_0 <= n; n_0++) {
        if (y->data[(y->size[0] * c + n_0) - 1] != 0.0) {
          for (yk = n_0 + 1; yk <= n; yk++) {
            y->data[(yk + y->size[0] * c) - 1] -= c_A->data[((n_0 - 1) *
              c_A->size[0] + yk) - 1] * y->data[(y->size[0] * c + n_0) - 1];
          }
        }
      }
    }

    cartesian_traje_emxFree_int32_T(&p);
    cartesian_trajec_emxInit_real_T(&y_0, 2);
    info = y_0->size[0] * y_0->size[1];
    y_0->size[0] = y->size[0];
    y_0->size[1] = y->size[1];
    cartes_emxEnsureCapacity_real_T(y_0, info);
    n_0 = y->size[0] * y->size[1];
    for (info = 0; info < n_0; info++) {
      y_0->data[info] = y->data[info];
    }

    cartesian_trajectory_plan_xtrsm(x->size[0], x->size[0], c_A, x->size[0], y_0,
      x->size[0], y);
    cartesian_trajec_emxFree_real_T(&y_0);
    cartesian_trajec_emxFree_real_T(&c_A);
  }
}

static void cartesian_trajectory_plann_diag(const
  emxArray_real_T_cartesian_tra_T *v, emxArray_real_T_cartesian_tra_T *d)
{
  int32_T u0;
  int32_T u1;
  if ((v->size[0] == 1) && (v->size[1] == 1)) {
    u0 = d->size[0];
    d->size[0] = 1;
    cartes_emxEnsureCapacity_real_T(d, u0);
    d->data[0] = v->data[0];
  } else {
    if (0 < v->size[1]) {
      u0 = v->size[0];
      u1 = v->size[1];
      if (u0 < u1) {
        u1 = u0;
      }
    } else {
      u1 = 0;
    }

    u0 = d->size[0];
    d->size[0] = u1;
    cartes_emxEnsureCapacity_real_T(d, u0);
    for (u0 = 0; u0 < u1; u0++) {
      d->data[u0] = v->data[v->size[0] * u0 + u0];
    }
  }
}

static void cartesian_trajectory_pl_sqrt_as(emxArray_real_T_cartesian_tra_T *x)
{
  int32_T nx;
  int32_T b_k;
  nx = x->size[0] - 1;
  for (b_k = 0; b_k <= nx; b_k++) {
    x->data[b_k] = sqrt(x->data[b_k]);
  }
}

static boolean_T cartesian_trajectory_planne_any(const
  emxArray_boolean_T_cartesian__T *x)
{
  boolean_T y;
  int32_T ix;
  boolean_T exitg1;
  y = false;
  ix = 0;
  exitg1 = false;
  while ((!exitg1) && (ix + 1 <= x->size[0])) {
    if (!x->data[ix]) {
      ix++;
    } else {
      y = true;
      exitg1 = true;
    }
  }

  return y;
}

static boolean_T cartesian_tr_isPositiveDefinite(const real_T B[36])
{
  emxArray_real_T_cartesian_tra_T *b_x;
  boolean_T exitg1;
  cartesian_trajectory_planner_B.c_A_size_idx_0 = 6;
  cartesian_trajectory_planner_B.c_A_size_idx_1 = 6;
  memcpy(&cartesian_trajectory_planner_B.c_A_data[0], &B[0], 36U * sizeof(real_T));
  cartesian_trajectory_planner_B.b_info = 0;
  cartesian_trajectory_planner_B.b_j_i = 1;
  cartesian_trajec_emxInit_real_T(&b_x, 2);
  exitg1 = false;
  while ((!exitg1) && (cartesian_trajectory_planner_B.b_j_i - 1 < 6)) {
    cartesian_trajectory_planner_B.jm1 = cartesian_trajectory_planner_B.b_j_i -
      2;
    cartesian_trajectory_planner_B.idxAjj =
      ((cartesian_trajectory_planner_B.b_j_i - 1) * 6 +
       cartesian_trajectory_planner_B.b_j_i) - 1;
    cartesian_trajectory_planner_B.ssq = 0.0;
    if (cartesian_trajectory_planner_B.b_j_i - 1 >= 1) {
      cartesian_trajectory_planner_B.ix_f = cartesian_trajectory_planner_B.b_j_i
        - 1;
      cartesian_trajectory_planner_B.iy = cartesian_trajectory_planner_B.b_j_i -
        1;
      for (cartesian_trajectory_planner_B.k_i = 0;
           cartesian_trajectory_planner_B.k_i <=
           cartesian_trajectory_planner_B.jm1;
           cartesian_trajectory_planner_B.k_i++) {
        cartesian_trajectory_planner_B.ssq +=
          cartesian_trajectory_planner_B.c_A_data[cartesian_trajectory_planner_B.ix_f]
          * cartesian_trajectory_planner_B.c_A_data[cartesian_trajectory_planner_B.iy];
        cartesian_trajectory_planner_B.ix_f += 6;
        cartesian_trajectory_planner_B.iy += 6;
      }
    }

    cartesian_trajectory_planner_B.ssq =
      cartesian_trajectory_planner_B.c_A_data[cartesian_trajectory_planner_B.idxAjj]
      - cartesian_trajectory_planner_B.ssq;
    if (cartesian_trajectory_planner_B.ssq > 0.0) {
      cartesian_trajectory_planner_B.ssq = sqrt
        (cartesian_trajectory_planner_B.ssq);
      cartesian_trajectory_planner_B.c_A_data[cartesian_trajectory_planner_B.idxAjj]
        = cartesian_trajectory_planner_B.ssq;
      if (cartesian_trajectory_planner_B.b_j_i < 6) {
        if (cartesian_trajectory_planner_B.b_j_i - 1 != 0) {
          cartesian_trajectory_planner_B.ix_f =
            cartesian_trajectory_planner_B.b_j_i - 1;
          cartesian_trajectory_planner_B.jm1 =
            (cartesian_trajectory_planner_B.b_j_i - 2) * 6 +
            cartesian_trajectory_planner_B.b_j_i;
          for (cartesian_trajectory_planner_B.k_i =
               cartesian_trajectory_planner_B.b_j_i + 1;
               cartesian_trajectory_planner_B.k_i <=
               cartesian_trajectory_planner_B.jm1 + 1;
               cartesian_trajectory_planner_B.k_i += 6) {
            cartesian_trajectory_planner_B.c =
              -cartesian_trajectory_planner_B.c_A_data[cartesian_trajectory_planner_B.ix_f];
            cartesian_trajectory_planner_B.iy =
              cartesian_trajectory_planner_B.idxAjj + 1;
            cartesian_trajectory_planner_B.d_n =
              cartesian_trajectory_planner_B.k_i -
              cartesian_trajectory_planner_B.b_j_i;
            for (cartesian_trajectory_planner_B.ia =
                 cartesian_trajectory_planner_B.k_i;
                 cartesian_trajectory_planner_B.ia <=
                 cartesian_trajectory_planner_B.d_n + 5;
                 cartesian_trajectory_planner_B.ia++) {
              cartesian_trajectory_planner_B.c_A_data[cartesian_trajectory_planner_B.iy]
                +=
                cartesian_trajectory_planner_B.c_A_data[cartesian_trajectory_planner_B.ia
                - 1] * cartesian_trajectory_planner_B.c;
              cartesian_trajectory_planner_B.iy++;
            }

            cartesian_trajectory_planner_B.ix_f += 6;
          }
        }

        cartesian_trajectory_planner_B.ssq = 1.0 /
          cartesian_trajectory_planner_B.ssq;
        cartesian_trajectory_planner_B.ix_f = b_x->size[0] * b_x->size[1];
        b_x->size[0] = 6;
        b_x->size[1] = 6;
        cartes_emxEnsureCapacity_real_T(b_x, cartesian_trajectory_planner_B.ix_f);
        cartesian_trajectory_planner_B.c_A_size_idx_0 =
          cartesian_trajectory_planner_B.c_A_size_idx_0 *
          cartesian_trajectory_planner_B.c_A_size_idx_1 - 1;
        for (cartesian_trajectory_planner_B.ix_f = 0;
             cartesian_trajectory_planner_B.ix_f <=
             cartesian_trajectory_planner_B.c_A_size_idx_0;
             cartesian_trajectory_planner_B.ix_f++) {
          b_x->data[cartesian_trajectory_planner_B.ix_f] =
            cartesian_trajectory_planner_B.c_A_data[cartesian_trajectory_planner_B.ix_f];
        }

        cartesian_trajectory_planner_B.jm1 =
          cartesian_trajectory_planner_B.idxAjj -
          cartesian_trajectory_planner_B.b_j_i;
        for (cartesian_trajectory_planner_B.k_i =
             cartesian_trajectory_planner_B.idxAjj + 2;
             cartesian_trajectory_planner_B.k_i <=
             cartesian_trajectory_planner_B.jm1 + 7;
             cartesian_trajectory_planner_B.k_i++) {
          b_x->data[cartesian_trajectory_planner_B.k_i - 1] *=
            cartesian_trajectory_planner_B.ssq;
        }

        cartesian_trajectory_planner_B.c_A_size_idx_0 = b_x->size[0];
        cartesian_trajectory_planner_B.c_A_size_idx_1 = b_x->size[1];
        for (cartesian_trajectory_planner_B.ix_f = 0;
             cartesian_trajectory_planner_B.ix_f < 36;
             cartesian_trajectory_planner_B.ix_f++) {
          cartesian_trajectory_planner_B.c_A_data[cartesian_trajectory_planner_B.ix_f]
            = b_x->data[cartesian_trajectory_planner_B.ix_f];
        }
      }

      cartesian_trajectory_planner_B.b_j_i++;
    } else {
      cartesian_trajectory_planner_B.b_info =
        cartesian_trajectory_planner_B.b_j_i;
      exitg1 = true;
    }
  }

  cartesian_trajec_emxFree_real_T(&b_x);
  return cartesian_trajectory_planner_B.b_info == 0;
}

static boolean_T DampedBFGSwGradientProjectio_as(const
  h_robotics_core_internal_Damp_T *obj, const real_T xNew[6])
{
  boolean_T flag;
  emxArray_real_T_cartesian_tra_T *b;
  emxArray_real_T_cartesian_tra_T *c;
  int32_T m;
  int32_T inner;
  int32_T b_i;
  int32_T loop_ub;
  emxArray_boolean_T_cartesian__T *c_0;
  cartesian_trajec_emxInit_real_T(&b, 2);
  cartesian_trajec_emxInit_real_T(&c, 1);
  cartesian_tra_emxInit_boolean_T(&c_0, 1);
  if (obj->ConstraintsOn) {
    b_i = b->size[0] * b->size[1];
    b->size[0] = obj->ConstraintMatrix->size[0];
    b->size[1] = obj->ConstraintMatrix->size[1];
    cartes_emxEnsureCapacity_real_T(b, b_i);
    loop_ub = obj->ConstraintMatrix->size[0] * obj->ConstraintMatrix->size[1] -
      1;
    for (b_i = 0; b_i <= loop_ub; b_i++) {
      b->data[b_i] = obj->ConstraintMatrix->data[b_i];
    }

    m = b->size[1] - 1;
    inner = b->size[0] - 1;
    b_i = c->size[0];
    c->size[0] = b->size[1];
    cartes_emxEnsureCapacity_real_T(c, b_i);
    for (b_i = 0; b_i <= m; b_i++) {
      c->data[b_i] = 0.0;
    }

    for (b_i = 0; b_i <= inner; b_i++) {
      for (loop_ub = 0; loop_ub <= m; loop_ub++) {
        c->data[loop_ub] += b->data[loop_ub * b->size[0] + b_i] * xNew[b_i];
      }
    }

    b_i = c_0->size[0];
    c_0->size[0] = c->size[0];
    car_emxEnsureCapacity_boolean_T(c_0, b_i);
    loop_ub = c->size[0];
    for (b_i = 0; b_i < loop_ub; b_i++) {
      c_0->data[b_i] = (c->data[b_i] - obj->ConstraintBound->data[b_i] >
                        1.4901161193847656E-8);
    }

    if (cartesian_trajectory_planne_any(c_0)) {
      flag = true;
    } else {
      flag = false;
    }
  } else {
    flag = false;
  }

  cartesian_tra_emxFree_boolean_T(&c_0);
  cartesian_trajec_emxFree_real_T(&c);
  cartesian_trajec_emxFree_real_T(&b);
  return flag;
}

static void DampedBFGSwGradientProjection_s(h_robotics_core_internal_Damp_T *obj,
  real_T xSol[6], c_robotics_core_internal_NLPS_T *exitFlag, real_T *err, real_T
  *iter)
{
  emxArray_real_T_cartesian_tra_T *unusedU1;
  emxArray_real_T_cartesian_tra_T *grad;
  emxArray_boolean_T_cartesian__T *activeSet;
  emxArray_real_T_cartesian_tra_T *A;
  emxArray_real_T_cartesian_tra_T *alpha;
  emxArray_real_T_cartesian_tra_T *AIn;
  emxArray_real_T_cartesian_tra_T *L;
  f_robotics_manip_internal_IKE_T *b;
  f_robotics_manip_internal_IKE_T *c;
  f_robotics_manip_internal_IKE_T *d;
  emxArray_int32_T_cartesian_tr_T *cb;
  emxArray_int32_T_cartesian_tr_T *db;
  emxArray_int32_T_cartesian_tr_T *eb;
  emxArray_int32_T_cartesian_tr_T *fb;
  emxArray_int32_T_cartesian_tr_T *gb;
  f_robotics_manip_internal_IKE_T *args;
  emxArray_real_T_cartesian_tra_T *a;
  emxArray_int32_T_cartesian_tr_T *ii;
  emxArray_real_T_cartesian_tra_T *y;
  emxArray_int32_T_cartesian_tr_T *ii_0;
  emxArray_real_T_cartesian_tra_T *y_0;
  emxArray_boolean_T_cartesian__T *x;
  emxArray_real_T_cartesian_tra_T *A_0;
  emxArray_real_T_cartesian_tra_T *A_1;
  emxArray_real_T_cartesian_tra_T *A_2;
  emxArray_real_T_cartesian_tra_T *sigma;
  emxArray_real_T_cartesian_tra_T *tmp;
  emxArray_real_T_cartesian_tra_T *tmp_0;
  emxArray_real_T_cartesian_tra_T *grad_0;
  emxArray_real_T_cartesian_tra_T *sNew;
  emxArray_int32_T_cartesian_tr_T *ii_1;
  emxArray_int32_T_cartesian_tr_T *ii_2;
  emxArray_real_T_cartesian_tra_T *grad_1;
  emxArray_real_T_cartesian_tra_T *A_3;
  emxArray_real_T_cartesian_tra_T *alpha_0;
  emxArray_int32_T_cartesian_tr_T *ii_3;
  static const int8_T tmp_1[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  int32_T exitg1;
  int32_T exitg2;
  boolean_T exitg3;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  for (cartesian_trajectory_planner_B.i_n = 0;
       cartesian_trajectory_planner_B.i_n < 6;
       cartesian_trajectory_planner_B.i_n++) {
    cartesian_trajectory_planner_B.x[cartesian_trajectory_planner_B.i_n] =
      obj->SeedInternal[cartesian_trajectory_planner_B.i_n];
  }

  cartesian_trajec_emxInit_real_T(&unusedU1, 2);
  cartesian_trajec_emxInit_real_T(&grad, 1);
  obj->TimeObjInternal.StartTime = ctimefun();
  cartesian_IKHelpers_computeCost(cartesian_trajectory_planner_B.x,
    obj->ExtraArgs, &cartesian_trajectory_planner_B.cost,
    cartesian_trajectory_planner_B.unusedU0, unusedU1, &b);
  obj->ExtraArgs = b;
  args = obj->ExtraArgs;
  cartesian_trajectory_planner_B.b_i_o = grad->size[0];
  grad->size[0] = args->GradTemp->size[0];
  cartes_emxEnsureCapacity_real_T(grad, cartesian_trajectory_planner_B.b_i_o);
  cartesian_trajectory_planner_B.i_n = args->GradTemp->size[0];
  for (cartesian_trajectory_planner_B.b_i_o = 0;
       cartesian_trajectory_planner_B.b_i_o < cartesian_trajectory_planner_B.i_n;
       cartesian_trajectory_planner_B.b_i_o++) {
    grad->data[cartesian_trajectory_planner_B.b_i_o] = args->GradTemp->
      data[cartesian_trajectory_planner_B.b_i_o];
  }

  cartesian_trajectory_planne_eye(cartesian_trajectory_planner_B.unusedU0);
  memcpy(&cartesian_trajectory_planner_B.H[0],
         &cartesian_trajectory_planner_B.unusedU0[0], 36U * sizeof(real_T));
  cartesian_tra_emxInit_boolean_T(&activeSet, 1);
  cartesian_trajec_emxInit_real_T(&A, 2);
  cartesian_trajec_emxInit_real_T(&alpha, 1);
  cartesian_traje_emxInit_int32_T(&ii, 1);
  if (obj->ConstraintsOn) {
    cartesian_trajectory_planner_B.b_i_o = A->size[0] * A->size[1];
    A->size[0] = obj->ConstraintMatrix->size[0];
    A->size[1] = obj->ConstraintMatrix->size[1];
    cartes_emxEnsureCapacity_real_T(A, cartesian_trajectory_planner_B.b_i_o);
    cartesian_trajectory_planner_B.i_n = obj->ConstraintMatrix->size[0] *
      obj->ConstraintMatrix->size[1] - 1;
    for (cartesian_trajectory_planner_B.b_i_o = 0;
         cartesian_trajectory_planner_B.b_i_o <=
         cartesian_trajectory_planner_B.i_n;
         cartesian_trajectory_planner_B.b_i_o++) {
      A->data[cartesian_trajectory_planner_B.b_i_o] = obj->
        ConstraintMatrix->data[cartesian_trajectory_planner_B.b_i_o];
    }

    cartesian_trajectory_planner_B.m_hz = A->size[1] - 1;
    cartesian_trajectory_planner_B.inner = A->size[0] - 1;
    cartesian_trajectory_planner_B.b_i_o = alpha->size[0];
    alpha->size[0] = A->size[1];
    cartes_emxEnsureCapacity_real_T(alpha, cartesian_trajectory_planner_B.b_i_o);
    for (cartesian_trajectory_planner_B.b_i_o = 0;
         cartesian_trajectory_planner_B.b_i_o <=
         cartesian_trajectory_planner_B.m_hz;
         cartesian_trajectory_planner_B.b_i_o++) {
      alpha->data[cartesian_trajectory_planner_B.b_i_o] = 0.0;
    }

    for (cartesian_trajectory_planner_B.nx_k = 0;
         cartesian_trajectory_planner_B.nx_k <=
         cartesian_trajectory_planner_B.inner;
         cartesian_trajectory_planner_B.nx_k++) {
      for (cartesian_trajectory_planner_B.g_idx_0 = 0;
           cartesian_trajectory_planner_B.g_idx_0 <=
           cartesian_trajectory_planner_B.m_hz;
           cartesian_trajectory_planner_B.g_idx_0++) {
        alpha->data[cartesian_trajectory_planner_B.g_idx_0] += A->
          data[cartesian_trajectory_planner_B.g_idx_0 * A->size[0] +
          cartesian_trajectory_planner_B.nx_k] *
          cartesian_trajectory_planner_B.x[cartesian_trajectory_planner_B.nx_k];
      }
    }

    cartesian_trajectory_planner_B.b_i_o = activeSet->size[0];
    activeSet->size[0] = alpha->size[0];
    car_emxEnsureCapacity_boolean_T(activeSet,
      cartesian_trajectory_planner_B.b_i_o);
    cartesian_trajectory_planner_B.i_n = alpha->size[0];
    for (cartesian_trajectory_planner_B.b_i_o = 0;
         cartesian_trajectory_planner_B.b_i_o <
         cartesian_trajectory_planner_B.i_n;
         cartesian_trajectory_planner_B.b_i_o++) {
      activeSet->data[cartesian_trajectory_planner_B.b_i_o] = (alpha->
        data[cartesian_trajectory_planner_B.b_i_o] >= obj->ConstraintBound->
        data[cartesian_trajectory_planner_B.b_i_o]);
    }

    cartesian_trajectory_planner_B.nx_k = activeSet->size[0] - 1;
    cartesian_trajectory_planner_B.idx = 0;
    for (cartesian_trajectory_planner_B.g_idx_0 = 0;
         cartesian_trajectory_planner_B.g_idx_0 <=
         cartesian_trajectory_planner_B.nx_k;
         cartesian_trajectory_planner_B.g_idx_0++) {
      if (activeSet->data[cartesian_trajectory_planner_B.g_idx_0]) {
        cartesian_trajectory_planner_B.idx++;
      }
    }

    cartesian_trajectory_planner_B.b_i_o = ii->size[0];
    ii->size[0] = cartesian_trajectory_planner_B.idx;
    carte_emxEnsureCapacity_int32_T(ii, cartesian_trajectory_planner_B.b_i_o);
    cartesian_trajectory_planner_B.b_i_o = 0;
    for (cartesian_trajectory_planner_B.g_idx_0 = 0;
         cartesian_trajectory_planner_B.g_idx_0 <=
         cartesian_trajectory_planner_B.nx_k;
         cartesian_trajectory_planner_B.g_idx_0++) {
      if (activeSet->data[cartesian_trajectory_planner_B.g_idx_0]) {
        ii->data[cartesian_trajectory_planner_B.b_i_o] =
          cartesian_trajectory_planner_B.g_idx_0 + 1;
        cartesian_trajectory_planner_B.b_i_o++;
      }
    }

    cartesian_trajectory_planner_B.i_n = obj->ConstraintMatrix->size[0];
    cartesian_trajectory_planner_B.b_i_o = A->size[0] * A->size[1];
    A->size[0] = cartesian_trajectory_planner_B.i_n;
    A->size[1] = ii->size[0];
    cartes_emxEnsureCapacity_real_T(A, cartesian_trajectory_planner_B.b_i_o);
    cartesian_trajectory_planner_B.n_b = ii->size[0];
    for (cartesian_trajectory_planner_B.b_i_o = 0;
         cartesian_trajectory_planner_B.b_i_o <
         cartesian_trajectory_planner_B.n_b;
         cartesian_trajectory_planner_B.b_i_o++) {
      for (cartesian_trajectory_planner_B.idx = 0;
           cartesian_trajectory_planner_B.idx <
           cartesian_trajectory_planner_B.i_n;
           cartesian_trajectory_planner_B.idx++) {
        A->data[cartesian_trajectory_planner_B.idx + A->size[0] *
          cartesian_trajectory_planner_B.b_i_o] = obj->ConstraintMatrix->data
          [(ii->data[cartesian_trajectory_planner_B.b_i_o] - 1) *
          obj->ConstraintMatrix->size[0] + cartesian_trajectory_planner_B.idx];
      }
    }
  } else {
    cartesian_trajectory_planner_B.g_idx_0 = obj->ConstraintBound->size[0];
    cartesian_trajectory_planner_B.b_i_o = activeSet->size[0];
    activeSet->size[0] = cartesian_trajectory_planner_B.g_idx_0;
    car_emxEnsureCapacity_boolean_T(activeSet,
      cartesian_trajectory_planner_B.b_i_o);
    for (cartesian_trajectory_planner_B.b_i_o = 0;
         cartesian_trajectory_planner_B.b_i_o <
         cartesian_trajectory_planner_B.g_idx_0;
         cartesian_trajectory_planner_B.b_i_o++) {
      activeSet->data[cartesian_trajectory_planner_B.b_i_o] = false;
    }

    A->size[0] = 6;
    A->size[1] = 0;
  }

  cartesian_trajectory_planner_B.j = A->size[1] - 1;
  cartesian_trajec_emxInit_real_T(&AIn, 2);
  cartesian_trajec_emxInit_real_T(&A_0, 2);
  cartesian_trajec_emxInit_real_T(&A_1, 1);
  for (cartesian_trajectory_planner_B.nx_k = 0;
       cartesian_trajectory_planner_B.nx_k <= cartesian_trajectory_planner_B.j;
       cartesian_trajectory_planner_B.nx_k++) {
    cartesian_trajectory_planner_B.i_n = A->size[0];
    cartesian_trajectory_planner_B.b_i_o = A_0->size[0] * A_0->size[1];
    A_0->size[0] = 1;
    A_0->size[1] = cartesian_trajectory_planner_B.i_n;
    cartes_emxEnsureCapacity_real_T(A_0, cartesian_trajectory_planner_B.b_i_o);
    for (cartesian_trajectory_planner_B.b_i_o = 0;
         cartesian_trajectory_planner_B.b_i_o <
         cartesian_trajectory_planner_B.i_n;
         cartesian_trajectory_planner_B.b_i_o++) {
      A_0->data[cartesian_trajectory_planner_B.b_i_o] = A->data[A->size[0] *
        cartesian_trajectory_planner_B.nx_k +
        cartesian_trajectory_planner_B.b_i_o];
    }

    cartesian_trajectory_planner_B.i_n = A->size[0];
    cartesian_trajectory_planner_B.b_i_o = A_1->size[0];
    A_1->size[0] = cartesian_trajectory_planner_B.i_n;
    cartes_emxEnsureCapacity_real_T(A_1, cartesian_trajectory_planner_B.b_i_o);
    for (cartesian_trajectory_planner_B.b_i_o = 0;
         cartesian_trajectory_planner_B.b_i_o <
         cartesian_trajectory_planner_B.i_n;
         cartesian_trajectory_planner_B.b_i_o++) {
      A_1->data[cartesian_trajectory_planner_B.b_i_o] = A->data[A->size[0] *
        cartesian_trajectory_planner_B.nx_k +
        cartesian_trajectory_planner_B.b_i_o];
    }

    cartesian_trajectory_planner_B.A_d = 0.0;
    for (cartesian_trajectory_planner_B.b_i_o = 0;
         cartesian_trajectory_planner_B.b_i_o < 6;
         cartesian_trajectory_planner_B.b_i_o++) {
      cartesian_trajectory_planner_B.Hg[cartesian_trajectory_planner_B.b_i_o] =
        0.0;
      for (cartesian_trajectory_planner_B.idx = 0;
           cartesian_trajectory_planner_B.idx < 6;
           cartesian_trajectory_planner_B.idx++) {
        cartesian_trajectory_planner_B.s_e = cartesian_trajectory_planner_B.H[6 *
          cartesian_trajectory_planner_B.b_i_o +
          cartesian_trajectory_planner_B.idx] * A_0->
          data[cartesian_trajectory_planner_B.idx] +
          cartesian_trajectory_planner_B.Hg[cartesian_trajectory_planner_B.b_i_o];
        cartesian_trajectory_planner_B.Hg[cartesian_trajectory_planner_B.b_i_o] =
          cartesian_trajectory_planner_B.s_e;
      }

      cartesian_trajectory_planner_B.A_d +=
        cartesian_trajectory_planner_B.Hg[cartesian_trajectory_planner_B.b_i_o] *
        A_1->data[cartesian_trajectory_planner_B.b_i_o];
    }

    cartesian_trajectory_planner_B.s_e = 1.0 /
      cartesian_trajectory_planner_B.A_d;
    for (cartesian_trajectory_planner_B.b_i_o = 0;
         cartesian_trajectory_planner_B.b_i_o < 36;
         cartesian_trajectory_planner_B.b_i_o++) {
      cartesian_trajectory_planner_B.V[cartesian_trajectory_planner_B.b_i_o] =
        cartesian_trajectory_planner_B.s_e *
        cartesian_trajectory_planner_B.H[cartesian_trajectory_planner_B.b_i_o];
    }

    cartesian_trajectory_planner_B.i_n = A->size[0];
    cartesian_trajectory_planner_B.n_b = A->size[0];
    cartesian_trajectory_planner_B.b_i_o = AIn->size[0] * AIn->size[1];
    AIn->size[0] = cartesian_trajectory_planner_B.i_n;
    AIn->size[1] = cartesian_trajectory_planner_B.n_b;
    cartes_emxEnsureCapacity_real_T(AIn, cartesian_trajectory_planner_B.b_i_o);
    for (cartesian_trajectory_planner_B.b_i_o = 0;
         cartesian_trajectory_planner_B.b_i_o <
         cartesian_trajectory_planner_B.n_b;
         cartesian_trajectory_planner_B.b_i_o++) {
      for (cartesian_trajectory_planner_B.idx = 0;
           cartesian_trajectory_planner_B.idx <
           cartesian_trajectory_planner_B.i_n;
           cartesian_trajectory_planner_B.idx++) {
        AIn->data[cartesian_trajectory_planner_B.idx + AIn->size[0] *
          cartesian_trajectory_planner_B.b_i_o] = A->data[A->size[0] *
          cartesian_trajectory_planner_B.nx_k +
          cartesian_trajectory_planner_B.idx] * A->data[A->size[0] *
          cartesian_trajectory_planner_B.nx_k +
          cartesian_trajectory_planner_B.b_i_o];
      }
    }

    cartesian_trajectory_planner_B.n_b = AIn->size[1] - 1;
    cartesian_trajectory_planner_B.b_i_o = unusedU1->size[0] * unusedU1->size[1];
    unusedU1->size[0] = 6;
    unusedU1->size[1] = AIn->size[1];
    cartes_emxEnsureCapacity_real_T(unusedU1,
      cartesian_trajectory_planner_B.b_i_o);
    for (cartesian_trajectory_planner_B.idx = 0;
         cartesian_trajectory_planner_B.idx <=
         cartesian_trajectory_planner_B.n_b; cartesian_trajectory_planner_B.idx
         ++) {
      cartesian_trajectory_planner_B.coffset =
        cartesian_trajectory_planner_B.idx * 6 - 1;
      cartesian_trajectory_planner_B.boffset =
        cartesian_trajectory_planner_B.idx * AIn->size[0] - 1;
      for (cartesian_trajectory_planner_B.b_i_o = 0;
           cartesian_trajectory_planner_B.b_i_o < 6;
           cartesian_trajectory_planner_B.b_i_o++) {
        cartesian_trajectory_planner_B.s_e = 0.0;
        for (cartesian_trajectory_planner_B.g_idx_0 = 0;
             cartesian_trajectory_planner_B.g_idx_0 < 6;
             cartesian_trajectory_planner_B.g_idx_0++) {
          cartesian_trajectory_planner_B.s_e +=
            cartesian_trajectory_planner_B.V[cartesian_trajectory_planner_B.g_idx_0
            * 6 + cartesian_trajectory_planner_B.b_i_o] * AIn->data
            [(cartesian_trajectory_planner_B.boffset +
              cartesian_trajectory_planner_B.g_idx_0) + 1];
        }

        unusedU1->data[(cartesian_trajectory_planner_B.coffset +
                        cartesian_trajectory_planner_B.b_i_o) + 1] =
          cartesian_trajectory_planner_B.s_e;
      }
    }

    for (cartesian_trajectory_planner_B.b_i_o = 0;
         cartesian_trajectory_planner_B.b_i_o < 6;
         cartesian_trajectory_planner_B.b_i_o++) {
      for (cartesian_trajectory_planner_B.idx = 0;
           cartesian_trajectory_planner_B.idx < 6;
           cartesian_trajectory_planner_B.idx++) {
        cartesian_trajectory_planner_B.s_e = 0.0;
        for (cartesian_trajectory_planner_B.i_n = 0;
             cartesian_trajectory_planner_B.i_n < 6;
             cartesian_trajectory_planner_B.i_n++) {
          cartesian_trajectory_planner_B.s_e += unusedU1->data[6 *
            cartesian_trajectory_planner_B.i_n +
            cartesian_trajectory_planner_B.b_i_o] *
            cartesian_trajectory_planner_B.H[6 *
            cartesian_trajectory_planner_B.idx +
            cartesian_trajectory_planner_B.i_n];
        }

        cartesian_trajectory_planner_B.idxl = 6 *
          cartesian_trajectory_planner_B.idx +
          cartesian_trajectory_planner_B.b_i_o;
        cartesian_trajectory_planner_B.H_m[cartesian_trajectory_planner_B.idxl] =
          cartesian_trajectory_planner_B.H[cartesian_trajectory_planner_B.idxl]
          - cartesian_trajectory_planner_B.s_e;
      }
    }

    memcpy(&cartesian_trajectory_planner_B.H[0],
           &cartesian_trajectory_planner_B.H_m[0], 36U * sizeof(real_T));
  }

  cartesian_trajec_emxFree_real_T(&A_1);
  cartesian_trajec_emxFree_real_T(&A_0);
  for (cartesian_trajectory_planner_B.i_n = 0;
       cartesian_trajectory_planner_B.i_n < 6;
       cartesian_trajectory_planner_B.i_n++) {
    xSol[cartesian_trajectory_planner_B.i_n] =
      cartesian_trajectory_planner_B.x[cartesian_trajectory_planner_B.i_n];
  }

  cartesian_trajectory_planner_B.A_d = obj->MaxNumIterationInternal;
  cartesian_trajectory_planner_B.g_idx_0 = 0;
  cartesian_trajec_emxInit_real_T(&L, 1);
  cartesian_traje_emxInit_int32_T(&cb, 1);
  cartesian_traje_emxInit_int32_T(&db, 1);
  cartesian_traje_emxInit_int32_T(&eb, 1);
  cartesian_traje_emxInit_int32_T(&fb, 1);
  cartesian_traje_emxInit_int32_T(&gb, 1);
  cartesian_trajec_emxInit_real_T(&a, 2);
  cartesian_trajec_emxInit_real_T(&y, 1);
  cartesian_traje_emxInit_int32_T(&ii_0, 1);
  cartesian_trajec_emxInit_real_T(&y_0, 2);
  cartesian_tra_emxInit_boolean_T(&x, 1);
  cartesian_trajec_emxInit_real_T(&A_2, 2);
  cartesian_trajec_emxInit_real_T(&sigma, 2);
  cartesian_trajec_emxInit_real_T(&tmp, 2);
  cartesian_trajec_emxInit_real_T(&tmp_0, 2);
  cartesian_trajec_emxInit_real_T(&grad_0, 2);
  cartesian_trajec_emxInit_real_T(&sNew, 2);
  cartesian_traje_emxInit_int32_T(&ii_1, 1);
  cartesian_traje_emxInit_int32_T(&ii_2, 1);
  cartesian_trajec_emxInit_real_T(&grad_1, 2);
  cartesian_trajec_emxInit_real_T(&A_3, 2);
  cartesian_trajec_emxInit_real_T(&alpha_0, 2);
  cartesian_traje_emxInit_int32_T(&ii_3, 1);
  do {
    exitg2 = 0;
    if (cartesian_trajectory_planner_B.g_idx_0 <= static_cast<int32_T>
        (cartesian_trajectory_planner_B.A_d) - 1) {
      cartesian_trajectory_planner_B.s_e = SystemTimeProvider_getElapsedTi
        (&obj->TimeObjInternal);
      cartesian_trajectory_planner_B.flag = (cartesian_trajectory_planner_B.s_e >
        obj->MaxTimeInternal);
      if (cartesian_trajectory_planner_B.flag) {
        *exitFlag = TimeLimitExceeded;
        args = obj->ExtraArgs;
        for (cartesian_trajectory_planner_B.b_i_o = 0;
             cartesian_trajectory_planner_B.b_i_o < 36;
             cartesian_trajectory_planner_B.b_i_o++) {
          cartesian_trajectory_planner_B.unusedU0[cartesian_trajectory_planner_B.b_i_o]
            = args->WeightMatrix[cartesian_trajectory_planner_B.b_i_o];
        }

        cartesian_trajectory_planner_B.b_i_o = grad->size[0];
        grad->size[0] = args->ErrTemp->size[0];
        cartes_emxEnsureCapacity_real_T(grad,
          cartesian_trajectory_planner_B.b_i_o);
        cartesian_trajectory_planner_B.i_n = args->ErrTemp->size[0];
        for (cartesian_trajectory_planner_B.b_i_o = 0;
             cartesian_trajectory_planner_B.b_i_o <
             cartesian_trajectory_planner_B.i_n;
             cartesian_trajectory_planner_B.b_i_o++) {
          grad->data[cartesian_trajectory_planner_B.b_i_o] = args->ErrTemp->
            data[cartesian_trajectory_planner_B.b_i_o];
        }

        for (cartesian_trajectory_planner_B.b_i_o = 0;
             cartesian_trajectory_planner_B.b_i_o < 6;
             cartesian_trajectory_planner_B.b_i_o++) {
          cartesian_trajectory_planner_B.x[cartesian_trajectory_planner_B.b_i_o]
            = 0.0;
          for (cartesian_trajectory_planner_B.idx = 0;
               cartesian_trajectory_planner_B.idx < 6;
               cartesian_trajectory_planner_B.idx++) {
            cartesian_trajectory_planner_B.A_d =
              cartesian_trajectory_planner_B.unusedU0[6 *
              cartesian_trajectory_planner_B.idx +
              cartesian_trajectory_planner_B.b_i_o] * grad->
              data[cartesian_trajectory_planner_B.idx] +
              cartesian_trajectory_planner_B.x[cartesian_trajectory_planner_B.b_i_o];
            cartesian_trajectory_planner_B.x[cartesian_trajectory_planner_B.b_i_o]
              = cartesian_trajectory_planner_B.A_d;
          }
        }

        *err = cartesian_trajectory_pla_norm_a(cartesian_trajectory_planner_B.x);
        *iter = static_cast<real_T>(cartesian_trajectory_planner_B.g_idx_0) +
          1.0;
        exitg2 = 1;
      } else {
        if ((A->size[0] == 0) || (A->size[1] == 0)) {
          cartesian_trajectory_planner_B.b_i_o = alpha->size[0];
          alpha->size[0] = 1;
          cartes_emxEnsureCapacity_real_T(alpha,
            cartesian_trajectory_planner_B.b_i_o);
          alpha->data[0] = 0.0;
        } else {
          cartesian_trajectory_planner_B.m_hz = A->size[1] - 1;
          cartesian_trajectory_planner_B.inner = A->size[0] - 1;
          cartesian_trajectory_planner_B.n_b = A->size[1] - 1;
          cartesian_trajectory_planner_B.b_i_o = AIn->size[0] * AIn->size[1];
          AIn->size[0] = A->size[1];
          AIn->size[1] = A->size[1];
          cartes_emxEnsureCapacity_real_T(AIn,
            cartesian_trajectory_planner_B.b_i_o);
          for (cartesian_trajectory_planner_B.idx = 0;
               cartesian_trajectory_planner_B.idx <=
               cartesian_trajectory_planner_B.n_b;
               cartesian_trajectory_planner_B.idx++) {
            cartesian_trajectory_planner_B.coffset =
              (cartesian_trajectory_planner_B.m_hz + 1) *
              cartesian_trajectory_planner_B.idx - 1;
            cartesian_trajectory_planner_B.boffset =
              cartesian_trajectory_planner_B.idx * A->size[0] - 1;
            for (cartesian_trajectory_planner_B.b_i_o = 0;
                 cartesian_trajectory_planner_B.b_i_o <=
                 cartesian_trajectory_planner_B.m_hz;
                 cartesian_trajectory_planner_B.b_i_o++) {
              AIn->data[(cartesian_trajectory_planner_B.coffset +
                         cartesian_trajectory_planner_B.b_i_o) + 1] = 0.0;
            }

            for (cartesian_trajectory_planner_B.nx_k = 0;
                 cartesian_trajectory_planner_B.nx_k <=
                 cartesian_trajectory_planner_B.inner;
                 cartesian_trajectory_planner_B.nx_k++) {
              cartesian_trajectory_planner_B.s_e = A->data
                [(cartesian_trajectory_planner_B.boffset +
                  cartesian_trajectory_planner_B.nx_k) + 1];
              for (cartesian_trajectory_planner_B.j = 0;
                   cartesian_trajectory_planner_B.j <=
                   cartesian_trajectory_planner_B.m_hz;
                   cartesian_trajectory_planner_B.j++) {
                cartesian_trajectory_planner_B.b_i_o =
                  (cartesian_trajectory_planner_B.coffset +
                   cartesian_trajectory_planner_B.j) + 1;
                AIn->data[cartesian_trajectory_planner_B.b_i_o] += A->
                  data[cartesian_trajectory_planner_B.j * A->size[0] +
                  cartesian_trajectory_planner_B.nx_k] *
                  cartesian_trajectory_planner_B.s_e;
              }
            }
          }

          cartesian_trajectory_planner_B.b_i_o = A_2->size[0] * A_2->size[1];
          A_2->size[0] = A->size[1];
          A_2->size[1] = A->size[0];
          cartes_emxEnsureCapacity_real_T(A_2,
            cartesian_trajectory_planner_B.b_i_o);
          cartesian_trajectory_planner_B.i_n = A->size[0];
          for (cartesian_trajectory_planner_B.b_i_o = 0;
               cartesian_trajectory_planner_B.b_i_o <
               cartesian_trajectory_planner_B.i_n;
               cartesian_trajectory_planner_B.b_i_o++) {
            cartesian_trajectory_planner_B.n_b = A->size[1];
            for (cartesian_trajectory_planner_B.idx = 0;
                 cartesian_trajectory_planner_B.idx <
                 cartesian_trajectory_planner_B.n_b;
                 cartesian_trajectory_planner_B.idx++) {
              A_2->data[cartesian_trajectory_planner_B.idx + A_2->size[0] *
                cartesian_trajectory_planner_B.b_i_o] = A->data[A->size[0] *
                cartesian_trajectory_planner_B.idx +
                cartesian_trajectory_planner_B.b_i_o];
            }
          }

          cartesian_trajectory_p_mldivide(AIn, A_2, a);
          cartesian_trajectory_planner_B.m_hz = a->size[0] - 1;
          cartesian_trajectory_planner_B.inner = a->size[1] - 1;
          cartesian_trajectory_planner_B.b_i_o = alpha->size[0];
          alpha->size[0] = a->size[0];
          cartes_emxEnsureCapacity_real_T(alpha,
            cartesian_trajectory_planner_B.b_i_o);
          for (cartesian_trajectory_planner_B.b_i_o = 0;
               cartesian_trajectory_planner_B.b_i_o <=
               cartesian_trajectory_planner_B.m_hz;
               cartesian_trajectory_planner_B.b_i_o++) {
            alpha->data[cartesian_trajectory_planner_B.b_i_o] = 0.0;
          }

          for (cartesian_trajectory_planner_B.nx_k = 0;
               cartesian_trajectory_planner_B.nx_k <=
               cartesian_trajectory_planner_B.inner;
               cartesian_trajectory_planner_B.nx_k++) {
            cartesian_trajectory_planner_B.aoffset =
              cartesian_trajectory_planner_B.nx_k * a->size[0] - 1;
            for (cartesian_trajectory_planner_B.j = 0;
                 cartesian_trajectory_planner_B.j <=
                 cartesian_trajectory_planner_B.m_hz;
                 cartesian_trajectory_planner_B.j++) {
              alpha->data[cartesian_trajectory_planner_B.j] += a->data
                [(cartesian_trajectory_planner_B.aoffset +
                  cartesian_trajectory_planner_B.j) + 1] * grad->
                data[cartesian_trajectory_planner_B.nx_k];
            }
          }
        }

        for (cartesian_trajectory_planner_B.b_i_o = 0;
             cartesian_trajectory_planner_B.b_i_o < 6;
             cartesian_trajectory_planner_B.b_i_o++) {
          cartesian_trajectory_planner_B.Hg[cartesian_trajectory_planner_B.b_i_o]
            = 0.0;
          for (cartesian_trajectory_planner_B.idx = 0;
               cartesian_trajectory_planner_B.idx < 6;
               cartesian_trajectory_planner_B.idx++) {
            cartesian_trajectory_planner_B.b_gamma =
              cartesian_trajectory_planner_B.H[6 *
              cartesian_trajectory_planner_B.idx +
              cartesian_trajectory_planner_B.b_i_o] * grad->
              data[cartesian_trajectory_planner_B.idx] +
              cartesian_trajectory_planner_B.Hg[cartesian_trajectory_planner_B.b_i_o];
            cartesian_trajectory_planner_B.Hg[cartesian_trajectory_planner_B.b_i_o]
              = cartesian_trajectory_planner_B.b_gamma;
          }
        }

        if (DampedBFGSwGradientProjection_a(obj,
             cartesian_trajectory_planner_B.Hg, alpha)) {
          *exitFlag = LocalMinimumFound;
          args = obj->ExtraArgs;
          for (cartesian_trajectory_planner_B.b_i_o = 0;
               cartesian_trajectory_planner_B.b_i_o < 36;
               cartesian_trajectory_planner_B.b_i_o++) {
            cartesian_trajectory_planner_B.unusedU0[cartesian_trajectory_planner_B.b_i_o]
              = args->WeightMatrix[cartesian_trajectory_planner_B.b_i_o];
          }

          cartesian_trajectory_planner_B.b_i_o = grad->size[0];
          grad->size[0] = args->ErrTemp->size[0];
          cartes_emxEnsureCapacity_real_T(grad,
            cartesian_trajectory_planner_B.b_i_o);
          cartesian_trajectory_planner_B.i_n = args->ErrTemp->size[0];
          for (cartesian_trajectory_planner_B.b_i_o = 0;
               cartesian_trajectory_planner_B.b_i_o <
               cartesian_trajectory_planner_B.i_n;
               cartesian_trajectory_planner_B.b_i_o++) {
            grad->data[cartesian_trajectory_planner_B.b_i_o] = args->
              ErrTemp->data[cartesian_trajectory_planner_B.b_i_o];
          }

          for (cartesian_trajectory_planner_B.b_i_o = 0;
               cartesian_trajectory_planner_B.b_i_o < 6;
               cartesian_trajectory_planner_B.b_i_o++) {
            cartesian_trajectory_planner_B.x[cartesian_trajectory_planner_B.b_i_o]
              = 0.0;
            for (cartesian_trajectory_planner_B.idx = 0;
                 cartesian_trajectory_planner_B.idx < 6;
                 cartesian_trajectory_planner_B.idx++) {
              cartesian_trajectory_planner_B.A_d =
                cartesian_trajectory_planner_B.unusedU0[6 *
                cartesian_trajectory_planner_B.idx +
                cartesian_trajectory_planner_B.b_i_o] * grad->
                data[cartesian_trajectory_planner_B.idx] +
                cartesian_trajectory_planner_B.x[cartesian_trajectory_planner_B.b_i_o];
              cartesian_trajectory_planner_B.x[cartesian_trajectory_planner_B.b_i_o]
                = cartesian_trajectory_planner_B.A_d;
            }
          }

          *err = cartesian_trajectory_pla_norm_a
            (cartesian_trajectory_planner_B.x);
          *iter = static_cast<real_T>(cartesian_trajectory_planner_B.g_idx_0) +
            1.0;
          exitg2 = 1;
        } else {
          guard1 = false;
          guard2 = false;
          if (obj->ConstraintsOn && ((A->size[0] != 0) && (A->size[1] != 0))) {
            cartesian_trajectory_planner_B.m_hz = A->size[1] - 1;
            cartesian_trajectory_planner_B.inner = A->size[0] - 1;
            cartesian_trajectory_planner_B.n_b = A->size[1] - 1;
            cartesian_trajectory_planner_B.b_i_o = AIn->size[0] * AIn->size[1];
            AIn->size[0] = A->size[1];
            AIn->size[1] = A->size[1];
            cartes_emxEnsureCapacity_real_T(AIn,
              cartesian_trajectory_planner_B.b_i_o);
            for (cartesian_trajectory_planner_B.idx = 0;
                 cartesian_trajectory_planner_B.idx <=
                 cartesian_trajectory_planner_B.n_b;
                 cartesian_trajectory_planner_B.idx++) {
              cartesian_trajectory_planner_B.coffset =
                (cartesian_trajectory_planner_B.m_hz + 1) *
                cartesian_trajectory_planner_B.idx - 1;
              cartesian_trajectory_planner_B.boffset =
                cartesian_trajectory_planner_B.idx * A->size[0] - 1;
              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o <=
                   cartesian_trajectory_planner_B.m_hz;
                   cartesian_trajectory_planner_B.b_i_o++) {
                AIn->data[(cartesian_trajectory_planner_B.coffset +
                           cartesian_trajectory_planner_B.b_i_o) + 1] = 0.0;
              }

              for (cartesian_trajectory_planner_B.nx_k = 0;
                   cartesian_trajectory_planner_B.nx_k <=
                   cartesian_trajectory_planner_B.inner;
                   cartesian_trajectory_planner_B.nx_k++) {
                cartesian_trajectory_planner_B.s_e = A->data
                  [(cartesian_trajectory_planner_B.boffset +
                    cartesian_trajectory_planner_B.nx_k) + 1];
                for (cartesian_trajectory_planner_B.j = 0;
                     cartesian_trajectory_planner_B.j <=
                     cartesian_trajectory_planner_B.m_hz;
                     cartesian_trajectory_planner_B.j++) {
                  cartesian_trajectory_planner_B.b_i_o =
                    (cartesian_trajectory_planner_B.coffset +
                     cartesian_trajectory_planner_B.j) + 1;
                  AIn->data[cartesian_trajectory_planner_B.b_i_o] += A->
                    data[cartesian_trajectory_planner_B.j * A->size[0] +
                    cartesian_trajectory_planner_B.nx_k] *
                    cartesian_trajectory_planner_B.s_e;
                }
              }
            }

            cartesian_trajectory_planne_inv(AIn, a);
            cartesian_trajectory_plann_diag(a, L);
            cartesian_trajectory_pl_sqrt_as(L);
            cartesian_trajectory_planner_B.b_i_o = alpha->size[0];
            cartes_emxEnsureCapacity_real_T(alpha,
              cartesian_trajectory_planner_B.b_i_o);
            cartesian_trajectory_planner_B.i_n = alpha->size[0];
            for (cartesian_trajectory_planner_B.b_i_o = 0;
                 cartesian_trajectory_planner_B.b_i_o <
                 cartesian_trajectory_planner_B.i_n;
                 cartesian_trajectory_planner_B.b_i_o++) {
              alpha->data[cartesian_trajectory_planner_B.b_i_o] /= L->
                data[cartesian_trajectory_planner_B.b_i_o];
            }

            cartesian_trajectory_planner_B.n_b = alpha->size[0];
            if (alpha->size[0] <= 2) {
              if (alpha->size[0] == 1) {
                cartesian_trajectory_planner_B.s_e = alpha->data[0];
                cartesian_trajectory_planner_B.idxl = 0;
              } else if ((alpha->data[0] < alpha->data[1]) || (rtIsNaN
                          (alpha->data[0]) && (!rtIsNaN(alpha->data[1])))) {
                cartesian_trajectory_planner_B.s_e = alpha->data[1];
                cartesian_trajectory_planner_B.idxl = 1;
              } else {
                cartesian_trajectory_planner_B.s_e = alpha->data[0];
                cartesian_trajectory_planner_B.idxl = 0;
              }
            } else {
              if (!rtIsNaN(alpha->data[0])) {
                cartesian_trajectory_planner_B.idxl = 1;
              } else {
                cartesian_trajectory_planner_B.idxl = 0;
                cartesian_trajectory_planner_B.b_i_o = 2;
                exitg3 = false;
                while ((!exitg3) && (cartesian_trajectory_planner_B.b_i_o <=
                                     alpha->size[0])) {
                  if (!rtIsNaN(alpha->data[cartesian_trajectory_planner_B.b_i_o
                               - 1])) {
                    cartesian_trajectory_planner_B.idxl =
                      cartesian_trajectory_planner_B.b_i_o;
                    exitg3 = true;
                  } else {
                    cartesian_trajectory_planner_B.b_i_o++;
                  }
                }
              }

              if (cartesian_trajectory_planner_B.idxl == 0) {
                cartesian_trajectory_planner_B.s_e = alpha->data[0];
              } else {
                cartesian_trajectory_planner_B.s_e = alpha->
                  data[cartesian_trajectory_planner_B.idxl - 1];
                cartesian_trajectory_planner_B.nx_k =
                  cartesian_trajectory_planner_B.idxl;
                for (cartesian_trajectory_planner_B.b_i_o =
                     cartesian_trajectory_planner_B.idxl + 1;
                     cartesian_trajectory_planner_B.b_i_o <=
                     cartesian_trajectory_planner_B.n_b;
                     cartesian_trajectory_planner_B.b_i_o++) {
                  if (cartesian_trajectory_planner_B.s_e < alpha->
                      data[cartesian_trajectory_planner_B.b_i_o - 1]) {
                    cartesian_trajectory_planner_B.s_e = alpha->
                      data[cartesian_trajectory_planner_B.b_i_o - 1];
                    cartesian_trajectory_planner_B.nx_k =
                      cartesian_trajectory_planner_B.b_i_o;
                  }
                }

                cartesian_trajectory_planner_B.idxl =
                  cartesian_trajectory_planner_B.nx_k - 1;
              }
            }

            if (cartesian_trajectory_pla_norm_a
                (cartesian_trajectory_planner_B.Hg) < 0.5 *
                cartesian_trajectory_planner_B.s_e) {
              cartesian_trajectory_planner_B.nx_k = activeSet->size[0];
              cartesian_trajectory_planner_B.idx = 0;
              cartesian_trajectory_planner_B.b_i_o = ii->size[0];
              ii->size[0] = activeSet->size[0];
              carte_emxEnsureCapacity_int32_T(ii,
                cartesian_trajectory_planner_B.b_i_o);
              cartesian_trajectory_planner_B.b_i_o = 1;
              exitg3 = false;
              while ((!exitg3) && (cartesian_trajectory_planner_B.b_i_o - 1 <=
                                   cartesian_trajectory_planner_B.nx_k - 1)) {
                if (activeSet->data[cartesian_trajectory_planner_B.b_i_o - 1]) {
                  cartesian_trajectory_planner_B.idx++;
                  ii->data[cartesian_trajectory_planner_B.idx - 1] =
                    cartesian_trajectory_planner_B.b_i_o;
                  if (cartesian_trajectory_planner_B.idx >=
                      cartesian_trajectory_planner_B.nx_k) {
                    exitg3 = true;
                  } else {
                    cartesian_trajectory_planner_B.b_i_o++;
                  }
                } else {
                  cartesian_trajectory_planner_B.b_i_o++;
                }
              }

              if (activeSet->size[0] == 1) {
                if (cartesian_trajectory_planner_B.idx == 0) {
                  ii->size[0] = 0;
                }
              } else {
                if (1 > cartesian_trajectory_planner_B.idx) {
                  cartesian_trajectory_planner_B.idx = 0;
                }

                cartesian_trajectory_planner_B.b_i_o = ii_1->size[0];
                ii_1->size[0] = cartesian_trajectory_planner_B.idx;
                carte_emxEnsureCapacity_int32_T(ii_1,
                  cartesian_trajectory_planner_B.b_i_o);
                for (cartesian_trajectory_planner_B.b_i_o = 0;
                     cartesian_trajectory_planner_B.b_i_o <
                     cartesian_trajectory_planner_B.idx;
                     cartesian_trajectory_planner_B.b_i_o++) {
                  ii_1->data[cartesian_trajectory_planner_B.b_i_o] = ii->
                    data[cartesian_trajectory_planner_B.b_i_o];
                }

                cartesian_trajectory_planner_B.b_i_o = ii->size[0];
                ii->size[0] = ii_1->size[0];
                carte_emxEnsureCapacity_int32_T(ii,
                  cartesian_trajectory_planner_B.b_i_o);
                cartesian_trajectory_planner_B.i_n = ii_1->size[0];
                for (cartesian_trajectory_planner_B.b_i_o = 0;
                     cartesian_trajectory_planner_B.b_i_o <
                     cartesian_trajectory_planner_B.i_n;
                     cartesian_trajectory_planner_B.b_i_o++) {
                  ii->data[cartesian_trajectory_planner_B.b_i_o] = ii_1->
                    data[cartesian_trajectory_planner_B.b_i_o];
                }
              }

              cartesian_trajectory_planner_B.b_i_o = alpha->size[0];
              alpha->size[0] = ii->size[0];
              cartes_emxEnsureCapacity_real_T(alpha,
                cartesian_trajectory_planner_B.b_i_o);
              cartesian_trajectory_planner_B.i_n = ii->size[0];
              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o <
                   cartesian_trajectory_planner_B.i_n;
                   cartesian_trajectory_planner_B.b_i_o++) {
                alpha->data[cartesian_trajectory_planner_B.b_i_o] = ii->
                  data[cartesian_trajectory_planner_B.b_i_o];
              }

              activeSet->data[static_cast<int32_T>(alpha->
                data[cartesian_trajectory_planner_B.idxl]) - 1] = false;
              cartesian_trajectory_planner_B.nx_k = activeSet->size[0] - 1;
              cartesian_trajectory_planner_B.idx = 0;
              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o <=
                   cartesian_trajectory_planner_B.nx_k;
                   cartesian_trajectory_planner_B.b_i_o++) {
                if (activeSet->data[cartesian_trajectory_planner_B.b_i_o]) {
                  cartesian_trajectory_planner_B.idx++;
                }
              }

              cartesian_trajectory_planner_B.b_i_o = eb->size[0];
              eb->size[0] = cartesian_trajectory_planner_B.idx;
              carte_emxEnsureCapacity_int32_T(eb,
                cartesian_trajectory_planner_B.b_i_o);
              cartesian_trajectory_planner_B.idx = 0;
              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o <=
                   cartesian_trajectory_planner_B.nx_k;
                   cartesian_trajectory_planner_B.b_i_o++) {
                if (activeSet->data[cartesian_trajectory_planner_B.b_i_o]) {
                  eb->data[cartesian_trajectory_planner_B.idx] =
                    cartesian_trajectory_planner_B.b_i_o + 1;
                  cartesian_trajectory_planner_B.idx++;
                }
              }

              cartesian_trajectory_planner_B.i_n = obj->ConstraintMatrix->size[0];
              cartesian_trajectory_planner_B.b_i_o = A->size[0] * A->size[1];
              A->size[0] = cartesian_trajectory_planner_B.i_n;
              A->size[1] = eb->size[0];
              cartes_emxEnsureCapacity_real_T(A,
                cartesian_trajectory_planner_B.b_i_o);
              cartesian_trajectory_planner_B.n_b = eb->size[0];
              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o <
                   cartesian_trajectory_planner_B.n_b;
                   cartesian_trajectory_planner_B.b_i_o++) {
                for (cartesian_trajectory_planner_B.idx = 0;
                     cartesian_trajectory_planner_B.idx <
                     cartesian_trajectory_planner_B.i_n;
                     cartesian_trajectory_planner_B.idx++) {
                  A->data[cartesian_trajectory_planner_B.idx + A->size[0] *
                    cartesian_trajectory_planner_B.b_i_o] =
                    obj->ConstraintMatrix->data[(eb->
                    data[cartesian_trajectory_planner_B.b_i_o] - 1) *
                    obj->ConstraintMatrix->size[0] +
                    cartesian_trajectory_planner_B.idx];
                }
              }

              cartesian_trajectory_planner_B.m_hz = A->size[1] - 1;
              cartesian_trajectory_planner_B.inner = A->size[0] - 1;
              cartesian_trajectory_planner_B.n_b = A->size[1] - 1;
              cartesian_trajectory_planner_B.b_i_o = AIn->size[0] * AIn->size[1];
              AIn->size[0] = A->size[1];
              AIn->size[1] = A->size[1];
              cartes_emxEnsureCapacity_real_T(AIn,
                cartesian_trajectory_planner_B.b_i_o);
              for (cartesian_trajectory_planner_B.idx = 0;
                   cartesian_trajectory_planner_B.idx <=
                   cartesian_trajectory_planner_B.n_b;
                   cartesian_trajectory_planner_B.idx++) {
                cartesian_trajectory_planner_B.coffset =
                  (cartesian_trajectory_planner_B.m_hz + 1) *
                  cartesian_trajectory_planner_B.idx - 1;
                cartesian_trajectory_planner_B.boffset =
                  cartesian_trajectory_planner_B.idx * A->size[0] - 1;
                for (cartesian_trajectory_planner_B.b_i_o = 0;
                     cartesian_trajectory_planner_B.b_i_o <=
                     cartesian_trajectory_planner_B.m_hz;
                     cartesian_trajectory_planner_B.b_i_o++) {
                  AIn->data[(cartesian_trajectory_planner_B.coffset +
                             cartesian_trajectory_planner_B.b_i_o) + 1] = 0.0;
                }

                for (cartesian_trajectory_planner_B.nx_k = 0;
                     cartesian_trajectory_planner_B.nx_k <=
                     cartesian_trajectory_planner_B.inner;
                     cartesian_trajectory_planner_B.nx_k++) {
                  cartesian_trajectory_planner_B.s_e = A->data
                    [(cartesian_trajectory_planner_B.boffset +
                      cartesian_trajectory_planner_B.nx_k) + 1];
                  for (cartesian_trajectory_planner_B.j = 0;
                       cartesian_trajectory_planner_B.j <=
                       cartesian_trajectory_planner_B.m_hz;
                       cartesian_trajectory_planner_B.j++) {
                    cartesian_trajectory_planner_B.b_i_o =
                      (cartesian_trajectory_planner_B.coffset +
                       cartesian_trajectory_planner_B.j) + 1;
                    AIn->data[cartesian_trajectory_planner_B.b_i_o] += A->
                      data[cartesian_trajectory_planner_B.j * A->size[0] +
                      cartesian_trajectory_planner_B.nx_k] *
                      cartesian_trajectory_planner_B.s_e;
                  }
                }
              }

              cartesian_trajectory_planner_B.b_i_o = A_3->size[0] * A_3->size[1];
              A_3->size[0] = A->size[1];
              A_3->size[1] = A->size[0];
              cartes_emxEnsureCapacity_real_T(A_3,
                cartesian_trajectory_planner_B.b_i_o);
              cartesian_trajectory_planner_B.i_n = A->size[0];
              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o <
                   cartesian_trajectory_planner_B.i_n;
                   cartesian_trajectory_planner_B.b_i_o++) {
                cartesian_trajectory_planner_B.n_b = A->size[1];
                for (cartesian_trajectory_planner_B.idx = 0;
                     cartesian_trajectory_planner_B.idx <
                     cartesian_trajectory_planner_B.n_b;
                     cartesian_trajectory_planner_B.idx++) {
                  A_3->data[cartesian_trajectory_planner_B.idx + A_3->size[0] *
                    cartesian_trajectory_planner_B.b_i_o] = A->data[A->size[0] *
                    cartesian_trajectory_planner_B.idx +
                    cartesian_trajectory_planner_B.b_i_o];
                }
              }

              cartesian_trajectory_p_mldivide(AIn, A_3, a);
              cartesian_trajectory_planner_B.m_hz = A->size[0] - 1;
              cartesian_trajectory_planner_B.inner = A->size[1] - 1;
              cartesian_trajectory_planner_B.n_b = a->size[1] - 1;
              cartesian_trajectory_planner_B.b_i_o = AIn->size[0] * AIn->size[1];
              AIn->size[0] = A->size[0];
              AIn->size[1] = a->size[1];
              cartes_emxEnsureCapacity_real_T(AIn,
                cartesian_trajectory_planner_B.b_i_o);
              for (cartesian_trajectory_planner_B.idx = 0;
                   cartesian_trajectory_planner_B.idx <=
                   cartesian_trajectory_planner_B.n_b;
                   cartesian_trajectory_planner_B.idx++) {
                cartesian_trajectory_planner_B.coffset =
                  (cartesian_trajectory_planner_B.m_hz + 1) *
                  cartesian_trajectory_planner_B.idx - 1;
                cartesian_trajectory_planner_B.boffset =
                  cartesian_trajectory_planner_B.idx * a->size[0] - 1;
                for (cartesian_trajectory_planner_B.b_i_o = 0;
                     cartesian_trajectory_planner_B.b_i_o <=
                     cartesian_trajectory_planner_B.m_hz;
                     cartesian_trajectory_planner_B.b_i_o++) {
                  AIn->data[(cartesian_trajectory_planner_B.coffset +
                             cartesian_trajectory_planner_B.b_i_o) + 1] = 0.0;
                }

                for (cartesian_trajectory_planner_B.nx_k = 0;
                     cartesian_trajectory_planner_B.nx_k <=
                     cartesian_trajectory_planner_B.inner;
                     cartesian_trajectory_planner_B.nx_k++) {
                  cartesian_trajectory_planner_B.aoffset =
                    cartesian_trajectory_planner_B.nx_k * A->size[0] - 1;
                  cartesian_trajectory_planner_B.s_e = a->data
                    [(cartesian_trajectory_planner_B.boffset +
                      cartesian_trajectory_planner_B.nx_k) + 1];
                  for (cartesian_trajectory_planner_B.j = 0;
                       cartesian_trajectory_planner_B.j <=
                       cartesian_trajectory_planner_B.m_hz;
                       cartesian_trajectory_planner_B.j++) {
                    cartesian_trajectory_planner_B.i_n =
                      cartesian_trajectory_planner_B.j + 1;
                    cartesian_trajectory_planner_B.b_i_o =
                      cartesian_trajectory_planner_B.coffset +
                      cartesian_trajectory_planner_B.i_n;
                    AIn->data[cartesian_trajectory_planner_B.b_i_o] += A->
                      data[cartesian_trajectory_planner_B.aoffset +
                      cartesian_trajectory_planner_B.i_n] *
                      cartesian_trajectory_planner_B.s_e;
                  }
                }
              }

              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o < 36;
                   cartesian_trajectory_planner_B.b_i_o++) {
                cartesian_trajectory_planner_B.P[cartesian_trajectory_planner_B.b_i_o]
                  =
                  cartesian_trajectory_planner_B.unusedU0[cartesian_trajectory_planner_B.b_i_o]
                  - AIn->data[cartesian_trajectory_planner_B.b_i_o];
              }

              cartesian_trajectory_planner_B.s_e = alpha->
                data[cartesian_trajectory_planner_B.idxl];
              cartesian_trajectory_planner_B.b_i_o = static_cast<int32_T>
                (cartesian_trajectory_planner_B.s_e);
              cartesian_trajectory_planner_B.i_n = obj->ConstraintMatrix->size[0];
              cartesian_trajectory_planner_B.idx = alpha->size[0];
              alpha->size[0] = cartesian_trajectory_planner_B.i_n;
              cartes_emxEnsureCapacity_real_T(alpha,
                cartesian_trajectory_planner_B.idx);
              for (cartesian_trajectory_planner_B.idx = 0;
                   cartesian_trajectory_planner_B.idx <
                   cartesian_trajectory_planner_B.i_n;
                   cartesian_trajectory_planner_B.idx++) {
                alpha->data[cartesian_trajectory_planner_B.idx] =
                  obj->ConstraintMatrix->data
                  [(cartesian_trajectory_planner_B.b_i_o - 1) *
                  obj->ConstraintMatrix->size[0] +
                  cartesian_trajectory_planner_B.idx];
              }

              cartesian_trajectory_planner_B.b_i_o = alpha_0->size[0] *
                alpha_0->size[1];
              alpha_0->size[0] = 1;
              alpha_0->size[1] = alpha->size[0];
              cartes_emxEnsureCapacity_real_T(alpha_0,
                cartesian_trajectory_planner_B.b_i_o);
              cartesian_trajectory_planner_B.i_n = alpha->size[0];
              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o <
                   cartesian_trajectory_planner_B.i_n;
                   cartesian_trajectory_planner_B.b_i_o++) {
                alpha_0->data[cartesian_trajectory_planner_B.b_i_o] =
                  alpha->data[cartesian_trajectory_planner_B.b_i_o];
              }

              cartesian_trajectory_planner_B.s_e = 0.0;
              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o < 6;
                   cartesian_trajectory_planner_B.b_i_o++) {
                cartesian_trajectory_planner_B.Hg[cartesian_trajectory_planner_B.b_i_o]
                  = 0.0;
                for (cartesian_trajectory_planner_B.idx = 0;
                     cartesian_trajectory_planner_B.idx < 6;
                     cartesian_trajectory_planner_B.idx++) {
                  cartesian_trajectory_planner_B.b_gamma =
                    cartesian_trajectory_planner_B.P[6 *
                    cartesian_trajectory_planner_B.b_i_o +
                    cartesian_trajectory_planner_B.idx] * alpha_0->
                    data[cartesian_trajectory_planner_B.idx] +
                    cartesian_trajectory_planner_B.Hg[cartesian_trajectory_planner_B.b_i_o];
                  cartesian_trajectory_planner_B.Hg[cartesian_trajectory_planner_B.b_i_o]
                    = cartesian_trajectory_planner_B.b_gamma;
                }

                cartesian_trajectory_planner_B.s_e +=
                  cartesian_trajectory_planner_B.Hg[cartesian_trajectory_planner_B.b_i_o]
                  * alpha->data[cartesian_trajectory_planner_B.b_i_o];
              }

              cartesian_trajectory_planner_B.s_e = 1.0 /
                cartesian_trajectory_planner_B.s_e;
              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o < 36;
                   cartesian_trajectory_planner_B.b_i_o++) {
                cartesian_trajectory_planner_B.V[cartesian_trajectory_planner_B.b_i_o]
                  = cartesian_trajectory_planner_B.s_e *
                  cartesian_trajectory_planner_B.P[cartesian_trajectory_planner_B.b_i_o];
              }

              cartesian_trajectory_planner_B.b_i_o = AIn->size[0] * AIn->size[1];
              AIn->size[0] = alpha->size[0];
              AIn->size[1] = alpha->size[0];
              cartes_emxEnsureCapacity_real_T(AIn,
                cartesian_trajectory_planner_B.b_i_o);
              cartesian_trajectory_planner_B.i_n = alpha->size[0];
              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o <
                   cartesian_trajectory_planner_B.i_n;
                   cartesian_trajectory_planner_B.b_i_o++) {
                cartesian_trajectory_planner_B.n_b = alpha->size[0];
                for (cartesian_trajectory_planner_B.idx = 0;
                     cartesian_trajectory_planner_B.idx <
                     cartesian_trajectory_planner_B.n_b;
                     cartesian_trajectory_planner_B.idx++) {
                  AIn->data[cartesian_trajectory_planner_B.idx + AIn->size[0] *
                    cartesian_trajectory_planner_B.b_i_o] = alpha->
                    data[cartesian_trajectory_planner_B.idx] * alpha->
                    data[cartesian_trajectory_planner_B.b_i_o];
                }
              }

              cartesian_trajectory_planner_B.n_b = AIn->size[1] - 1;
              cartesian_trajectory_planner_B.b_i_o = unusedU1->size[0] *
                unusedU1->size[1];
              unusedU1->size[0] = 6;
              unusedU1->size[1] = AIn->size[1];
              cartes_emxEnsureCapacity_real_T(unusedU1,
                cartesian_trajectory_planner_B.b_i_o);
              for (cartesian_trajectory_planner_B.idx = 0;
                   cartesian_trajectory_planner_B.idx <=
                   cartesian_trajectory_planner_B.n_b;
                   cartesian_trajectory_planner_B.idx++) {
                cartesian_trajectory_planner_B.coffset =
                  cartesian_trajectory_planner_B.idx * 6 - 1;
                cartesian_trajectory_planner_B.boffset =
                  cartesian_trajectory_planner_B.idx * AIn->size[0] - 1;
                for (cartesian_trajectory_planner_B.b_i_o = 0;
                     cartesian_trajectory_planner_B.b_i_o < 6;
                     cartesian_trajectory_planner_B.b_i_o++) {
                  cartesian_trajectory_planner_B.s_e = 0.0;
                  for (cartesian_trajectory_planner_B.nx_k = 0;
                       cartesian_trajectory_planner_B.nx_k < 6;
                       cartesian_trajectory_planner_B.nx_k++) {
                    cartesian_trajectory_planner_B.s_e +=
                      cartesian_trajectory_planner_B.V[cartesian_trajectory_planner_B.nx_k
                      * 6 + cartesian_trajectory_planner_B.b_i_o] * AIn->data
                      [(cartesian_trajectory_planner_B.boffset +
                        cartesian_trajectory_planner_B.nx_k) + 1];
                  }

                  unusedU1->data[(cartesian_trajectory_planner_B.coffset +
                                  cartesian_trajectory_planner_B.b_i_o) + 1] =
                    cartesian_trajectory_planner_B.s_e;
                }
              }

              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o < 6;
                   cartesian_trajectory_planner_B.b_i_o++) {
                for (cartesian_trajectory_planner_B.idx = 0;
                     cartesian_trajectory_planner_B.idx < 6;
                     cartesian_trajectory_planner_B.idx++) {
                  cartesian_trajectory_planner_B.s_e = 0.0;
                  for (cartesian_trajectory_planner_B.i_n = 0;
                       cartesian_trajectory_planner_B.i_n < 6;
                       cartesian_trajectory_planner_B.i_n++) {
                    cartesian_trajectory_planner_B.s_e += unusedU1->data[6 *
                      cartesian_trajectory_planner_B.i_n +
                      cartesian_trajectory_planner_B.b_i_o] *
                      cartesian_trajectory_planner_B.P[6 *
                      cartesian_trajectory_planner_B.idx +
                      cartesian_trajectory_planner_B.i_n];
                  }

                  cartesian_trajectory_planner_B.idxl = 6 *
                    cartesian_trajectory_planner_B.idx +
                    cartesian_trajectory_planner_B.b_i_o;
                  cartesian_trajectory_planner_B.H[cartesian_trajectory_planner_B.idxl]
                    += cartesian_trajectory_planner_B.s_e;
                }
              }

              cartesian_trajectory_planner_B.g_idx_0++;
            } else {
              guard2 = true;
            }
          } else {
            guard2 = true;
          }

          if (guard2) {
            for (cartesian_trajectory_planner_B.b_i_o = 0;
                 cartesian_trajectory_planner_B.b_i_o < 6;
                 cartesian_trajectory_planner_B.b_i_o++) {
              cartesian_trajectory_planner_B.Hg[cartesian_trajectory_planner_B.b_i_o]
                =
                -cartesian_trajectory_planner_B.Hg[cartesian_trajectory_planner_B.b_i_o];
            }

            cartesian_trajectory_planner_B.idxl = -1;
            if (obj->ConstraintsOn) {
              cartesian_trajectory_planner_B.b_i_o = x->size[0];
              x->size[0] = activeSet->size[0];
              car_emxEnsureCapacity_boolean_T(x,
                cartesian_trajectory_planner_B.b_i_o);
              cartesian_trajectory_planner_B.i_n = activeSet->size[0];
              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o <
                   cartesian_trajectory_planner_B.i_n;
                   cartesian_trajectory_planner_B.b_i_o++) {
                x->data[cartesian_trajectory_planner_B.b_i_o] = !activeSet->
                  data[cartesian_trajectory_planner_B.b_i_o];
              }

              if (cartesian_trajectory_planne_any(x)) {
                cartesian_trajectory_planner_B.nx_k = activeSet->size[0] - 1;
                cartesian_trajectory_planner_B.idx = 0;
                for (cartesian_trajectory_planner_B.b_i_o = 0;
                     cartesian_trajectory_planner_B.b_i_o <=
                     cartesian_trajectory_planner_B.nx_k;
                     cartesian_trajectory_planner_B.b_i_o++) {
                  if (!activeSet->data[cartesian_trajectory_planner_B.b_i_o]) {
                    cartesian_trajectory_planner_B.idx++;
                  }
                }

                cartesian_trajectory_planner_B.b_i_o = cb->size[0];
                cb->size[0] = cartesian_trajectory_planner_B.idx;
                carte_emxEnsureCapacity_int32_T(cb,
                  cartesian_trajectory_planner_B.b_i_o);
                cartesian_trajectory_planner_B.idx = 0;
                for (cartesian_trajectory_planner_B.b_i_o = 0;
                     cartesian_trajectory_planner_B.b_i_o <=
                     cartesian_trajectory_planner_B.nx_k;
                     cartesian_trajectory_planner_B.b_i_o++) {
                  if (!activeSet->data[cartesian_trajectory_planner_B.b_i_o]) {
                    cb->data[cartesian_trajectory_planner_B.idx] =
                      cartesian_trajectory_planner_B.b_i_o + 1;
                    cartesian_trajectory_planner_B.idx++;
                  }
                }

                cartesian_trajectory_planner_B.b_i_o = alpha->size[0];
                alpha->size[0] = cb->size[0];
                cartes_emxEnsureCapacity_real_T(alpha,
                  cartesian_trajectory_planner_B.b_i_o);
                cartesian_trajectory_planner_B.i_n = cb->size[0];
                for (cartesian_trajectory_planner_B.b_i_o = 0;
                     cartesian_trajectory_planner_B.b_i_o <
                     cartesian_trajectory_planner_B.i_n;
                     cartesian_trajectory_planner_B.b_i_o++) {
                  alpha->data[cartesian_trajectory_planner_B.b_i_o] =
                    obj->ConstraintBound->data[cb->
                    data[cartesian_trajectory_planner_B.b_i_o] - 1];
                }

                cartesian_trajectory_planner_B.nx_k = activeSet->size[0] - 1;
                cartesian_trajectory_planner_B.idx = 0;
                for (cartesian_trajectory_planner_B.b_i_o = 0;
                     cartesian_trajectory_planner_B.b_i_o <=
                     cartesian_trajectory_planner_B.nx_k;
                     cartesian_trajectory_planner_B.b_i_o++) {
                  if (!activeSet->data[cartesian_trajectory_planner_B.b_i_o]) {
                    cartesian_trajectory_planner_B.idx++;
                  }
                }

                cartesian_trajectory_planner_B.b_i_o = db->size[0];
                db->size[0] = cartesian_trajectory_planner_B.idx;
                carte_emxEnsureCapacity_int32_T(db,
                  cartesian_trajectory_planner_B.b_i_o);
                cartesian_trajectory_planner_B.idx = 0;
                for (cartesian_trajectory_planner_B.b_i_o = 0;
                     cartesian_trajectory_planner_B.b_i_o <=
                     cartesian_trajectory_planner_B.nx_k;
                     cartesian_trajectory_planner_B.b_i_o++) {
                  if (!activeSet->data[cartesian_trajectory_planner_B.b_i_o]) {
                    db->data[cartesian_trajectory_planner_B.idx] =
                      cartesian_trajectory_planner_B.b_i_o + 1;
                    cartesian_trajectory_planner_B.idx++;
                  }
                }

                cartesian_trajectory_planner_B.i_n = obj->ConstraintMatrix->
                  size[0];
                cartesian_trajectory_planner_B.b_i_o = AIn->size[0] * AIn->size
                  [1];
                AIn->size[0] = cartesian_trajectory_planner_B.i_n;
                AIn->size[1] = db->size[0];
                cartes_emxEnsureCapacity_real_T(AIn,
                  cartesian_trajectory_planner_B.b_i_o);
                cartesian_trajectory_planner_B.n_b = db->size[0];
                for (cartesian_trajectory_planner_B.b_i_o = 0;
                     cartesian_trajectory_planner_B.b_i_o <
                     cartesian_trajectory_planner_B.n_b;
                     cartesian_trajectory_planner_B.b_i_o++) {
                  for (cartesian_trajectory_planner_B.idx = 0;
                       cartesian_trajectory_planner_B.idx <
                       cartesian_trajectory_planner_B.i_n;
                       cartesian_trajectory_planner_B.idx++) {
                    AIn->data[cartesian_trajectory_planner_B.idx + AIn->size[0] *
                      cartesian_trajectory_planner_B.b_i_o] =
                      obj->ConstraintMatrix->data[(db->
                      data[cartesian_trajectory_planner_B.b_i_o] - 1) *
                      obj->ConstraintMatrix->size[0] +
                      cartesian_trajectory_planner_B.idx];
                  }
                }

                cartesian_trajectory_planner_B.nx_k = x->size[0];
                cartesian_trajectory_planner_B.idx = 0;
                cartesian_trajectory_planner_B.b_i_o = ii->size[0];
                ii->size[0] = x->size[0];
                carte_emxEnsureCapacity_int32_T(ii,
                  cartesian_trajectory_planner_B.b_i_o);
                cartesian_trajectory_planner_B.b_i_o = 1;
                exitg3 = false;
                while ((!exitg3) && (cartesian_trajectory_planner_B.b_i_o - 1 <=
                                     cartesian_trajectory_planner_B.nx_k - 1)) {
                  if (x->data[cartesian_trajectory_planner_B.b_i_o - 1]) {
                    cartesian_trajectory_planner_B.idx++;
                    ii->data[cartesian_trajectory_planner_B.idx - 1] =
                      cartesian_trajectory_planner_B.b_i_o;
                    if (cartesian_trajectory_planner_B.idx >=
                        cartesian_trajectory_planner_B.nx_k) {
                      exitg3 = true;
                    } else {
                      cartesian_trajectory_planner_B.b_i_o++;
                    }
                  } else {
                    cartesian_trajectory_planner_B.b_i_o++;
                  }
                }

                if (x->size[0] == 1) {
                  if (cartesian_trajectory_planner_B.idx == 0) {
                    ii->size[0] = 0;
                  }
                } else {
                  if (1 > cartesian_trajectory_planner_B.idx) {
                    cartesian_trajectory_planner_B.idx = 0;
                  }

                  cartesian_trajectory_planner_B.b_i_o = ii_2->size[0];
                  ii_2->size[0] = cartesian_trajectory_planner_B.idx;
                  carte_emxEnsureCapacity_int32_T(ii_2,
                    cartesian_trajectory_planner_B.b_i_o);
                  for (cartesian_trajectory_planner_B.b_i_o = 0;
                       cartesian_trajectory_planner_B.b_i_o <
                       cartesian_trajectory_planner_B.idx;
                       cartesian_trajectory_planner_B.b_i_o++) {
                    ii_2->data[cartesian_trajectory_planner_B.b_i_o] = ii->
                      data[cartesian_trajectory_planner_B.b_i_o];
                  }

                  cartesian_trajectory_planner_B.b_i_o = ii->size[0];
                  ii->size[0] = ii_2->size[0];
                  carte_emxEnsureCapacity_int32_T(ii,
                    cartesian_trajectory_planner_B.b_i_o);
                  cartesian_trajectory_planner_B.i_n = ii_2->size[0];
                  for (cartesian_trajectory_planner_B.b_i_o = 0;
                       cartesian_trajectory_planner_B.b_i_o <
                       cartesian_trajectory_planner_B.i_n;
                       cartesian_trajectory_planner_B.b_i_o++) {
                    ii->data[cartesian_trajectory_planner_B.b_i_o] = ii_2->
                      data[cartesian_trajectory_planner_B.b_i_o];
                  }
                }

                cartesian_trajectory_planner_B.m_hz = AIn->size[1] - 1;
                cartesian_trajectory_planner_B.inner = AIn->size[0] - 1;
                cartesian_trajectory_planner_B.b_i_o = L->size[0];
                L->size[0] = AIn->size[1];
                cartes_emxEnsureCapacity_real_T(L,
                  cartesian_trajectory_planner_B.b_i_o);
                for (cartesian_trajectory_planner_B.b_i_o = 0;
                     cartesian_trajectory_planner_B.b_i_o <=
                     cartesian_trajectory_planner_B.m_hz;
                     cartesian_trajectory_planner_B.b_i_o++) {
                  L->data[cartesian_trajectory_planner_B.b_i_o] = 0.0;
                }

                for (cartesian_trajectory_planner_B.nx_k = 0;
                     cartesian_trajectory_planner_B.nx_k <=
                     cartesian_trajectory_planner_B.inner;
                     cartesian_trajectory_planner_B.nx_k++) {
                  for (cartesian_trajectory_planner_B.j = 0;
                       cartesian_trajectory_planner_B.j <=
                       cartesian_trajectory_planner_B.m_hz;
                       cartesian_trajectory_planner_B.j++) {
                    L->data[cartesian_trajectory_planner_B.j] += AIn->
                      data[cartesian_trajectory_planner_B.j * AIn->size[0] +
                      cartesian_trajectory_planner_B.nx_k] *
                      cartesian_trajectory_planner_B.x[cartesian_trajectory_planner_B.nx_k];
                  }
                }

                cartesian_trajectory_planner_B.m_hz = AIn->size[1] - 1;
                cartesian_trajectory_planner_B.inner = AIn->size[0] - 1;
                cartesian_trajectory_planner_B.b_i_o = y->size[0];
                y->size[0] = AIn->size[1];
                cartes_emxEnsureCapacity_real_T(y,
                  cartesian_trajectory_planner_B.b_i_o);
                for (cartesian_trajectory_planner_B.b_i_o = 0;
                     cartesian_trajectory_planner_B.b_i_o <=
                     cartesian_trajectory_planner_B.m_hz;
                     cartesian_trajectory_planner_B.b_i_o++) {
                  y->data[cartesian_trajectory_planner_B.b_i_o] = 0.0;
                }

                for (cartesian_trajectory_planner_B.nx_k = 0;
                     cartesian_trajectory_planner_B.nx_k <=
                     cartesian_trajectory_planner_B.inner;
                     cartesian_trajectory_planner_B.nx_k++) {
                  for (cartesian_trajectory_planner_B.j = 0;
                       cartesian_trajectory_planner_B.j <=
                       cartesian_trajectory_planner_B.m_hz;
                       cartesian_trajectory_planner_B.j++) {
                    y->data[cartesian_trajectory_planner_B.j] += AIn->
                      data[cartesian_trajectory_planner_B.j * AIn->size[0] +
                      cartesian_trajectory_planner_B.nx_k] *
                      cartesian_trajectory_planner_B.Hg[cartesian_trajectory_planner_B.nx_k];
                  }
                }

                cartesian_trajectory_planner_B.b_i_o = alpha->size[0];
                cartes_emxEnsureCapacity_real_T(alpha,
                  cartesian_trajectory_planner_B.b_i_o);
                cartesian_trajectory_planner_B.i_n = alpha->size[0];
                for (cartesian_trajectory_planner_B.b_i_o = 0;
                     cartesian_trajectory_planner_B.b_i_o <
                     cartesian_trajectory_planner_B.i_n;
                     cartesian_trajectory_planner_B.b_i_o++) {
                  alpha->data[cartesian_trajectory_planner_B.b_i_o] =
                    (alpha->data[cartesian_trajectory_planner_B.b_i_o] - L->
                     data[cartesian_trajectory_planner_B.b_i_o]) / y->
                    data[cartesian_trajectory_planner_B.b_i_o];
                }

                cartesian_trajectory_planner_B.b_i_o = x->size[0];
                x->size[0] = alpha->size[0];
                car_emxEnsureCapacity_boolean_T(x,
                  cartesian_trajectory_planner_B.b_i_o);
                cartesian_trajectory_planner_B.i_n = alpha->size[0];
                for (cartesian_trajectory_planner_B.b_i_o = 0;
                     cartesian_trajectory_planner_B.b_i_o <
                     cartesian_trajectory_planner_B.i_n;
                     cartesian_trajectory_planner_B.b_i_o++) {
                  x->data[cartesian_trajectory_planner_B.b_i_o] = (alpha->
                    data[cartesian_trajectory_planner_B.b_i_o] > 0.0);
                }

                cartesian_trajectory_planner_B.nx_k = x->size[0];
                cartesian_trajectory_planner_B.idx = 0;
                cartesian_trajectory_planner_B.b_i_o = ii_0->size[0];
                ii_0->size[0] = x->size[0];
                carte_emxEnsureCapacity_int32_T(ii_0,
                  cartesian_trajectory_planner_B.b_i_o);
                cartesian_trajectory_planner_B.b_i_o = 1;
                exitg3 = false;
                while ((!exitg3) && (cartesian_trajectory_planner_B.b_i_o - 1 <=
                                     cartesian_trajectory_planner_B.nx_k - 1)) {
                  if (x->data[cartesian_trajectory_planner_B.b_i_o - 1]) {
                    cartesian_trajectory_planner_B.idx++;
                    ii_0->data[cartesian_trajectory_planner_B.idx - 1] =
                      cartesian_trajectory_planner_B.b_i_o;
                    if (cartesian_trajectory_planner_B.idx >=
                        cartesian_trajectory_planner_B.nx_k) {
                      exitg3 = true;
                    } else {
                      cartesian_trajectory_planner_B.b_i_o++;
                    }
                  } else {
                    cartesian_trajectory_planner_B.b_i_o++;
                  }
                }

                if (x->size[0] == 1) {
                  if (cartesian_trajectory_planner_B.idx == 0) {
                    ii_0->size[0] = 0;
                  }
                } else {
                  if (1 > cartesian_trajectory_planner_B.idx) {
                    cartesian_trajectory_planner_B.idx = 0;
                  }

                  cartesian_trajectory_planner_B.b_i_o = ii_3->size[0];
                  ii_3->size[0] = cartesian_trajectory_planner_B.idx;
                  carte_emxEnsureCapacity_int32_T(ii_3,
                    cartesian_trajectory_planner_B.b_i_o);
                  for (cartesian_trajectory_planner_B.b_i_o = 0;
                       cartesian_trajectory_planner_B.b_i_o <
                       cartesian_trajectory_planner_B.idx;
                       cartesian_trajectory_planner_B.b_i_o++) {
                    ii_3->data[cartesian_trajectory_planner_B.b_i_o] =
                      ii_0->data[cartesian_trajectory_planner_B.b_i_o];
                  }

                  cartesian_trajectory_planner_B.b_i_o = ii_0->size[0];
                  ii_0->size[0] = ii_3->size[0];
                  carte_emxEnsureCapacity_int32_T(ii_0,
                    cartesian_trajectory_planner_B.b_i_o);
                  cartesian_trajectory_planner_B.i_n = ii_3->size[0];
                  for (cartesian_trajectory_planner_B.b_i_o = 0;
                       cartesian_trajectory_planner_B.b_i_o <
                       cartesian_trajectory_planner_B.i_n;
                       cartesian_trajectory_planner_B.b_i_o++) {
                    ii_0->data[cartesian_trajectory_planner_B.b_i_o] =
                      ii_3->data[cartesian_trajectory_planner_B.b_i_o];
                  }
                }

                cartesian_trajectory_planner_B.b_i_o = L->size[0];
                L->size[0] = ii_0->size[0];
                cartes_emxEnsureCapacity_real_T(L,
                  cartesian_trajectory_planner_B.b_i_o);
                cartesian_trajectory_planner_B.i_n = ii_0->size[0];
                for (cartesian_trajectory_planner_B.b_i_o = 0;
                     cartesian_trajectory_planner_B.b_i_o <
                     cartesian_trajectory_planner_B.i_n;
                     cartesian_trajectory_planner_B.b_i_o++) {
                  L->data[cartesian_trajectory_planner_B.b_i_o] = ii_0->
                    data[cartesian_trajectory_planner_B.b_i_o];
                }

                if (L->size[0] != 0) {
                  cartesian_trajectory_planner_B.nx_k = alpha->size[0] - 1;
                  cartesian_trajectory_planner_B.idx = 0;
                  for (cartesian_trajectory_planner_B.b_i_o = 0;
                       cartesian_trajectory_planner_B.b_i_o <=
                       cartesian_trajectory_planner_B.nx_k;
                       cartesian_trajectory_planner_B.b_i_o++) {
                    if (alpha->data[cartesian_trajectory_planner_B.b_i_o] > 0.0)
                    {
                      cartesian_trajectory_planner_B.idx++;
                    }
                  }

                  cartesian_trajectory_planner_B.b_i_o = fb->size[0];
                  fb->size[0] = cartesian_trajectory_planner_B.idx;
                  carte_emxEnsureCapacity_int32_T(fb,
                    cartesian_trajectory_planner_B.b_i_o);
                  cartesian_trajectory_planner_B.idx = 0;
                  for (cartesian_trajectory_planner_B.b_i_o = 0;
                       cartesian_trajectory_planner_B.b_i_o <=
                       cartesian_trajectory_planner_B.nx_k;
                       cartesian_trajectory_planner_B.b_i_o++) {
                    if (alpha->data[cartesian_trajectory_planner_B.b_i_o] > 0.0)
                    {
                      fb->data[cartesian_trajectory_planner_B.idx] =
                        cartesian_trajectory_planner_B.b_i_o + 1;
                      cartesian_trajectory_planner_B.idx++;
                    }
                  }

                  cartesian_trajectory_planner_B.n_b = fb->size[0];
                  if (fb->size[0] <= 2) {
                    if (fb->size[0] == 1) {
                      cartesian_trajectory_planner_B.s_e = alpha->data[fb->data
                        [0] - 1];
                      cartesian_trajectory_planner_B.idxl = 0;
                    } else if ((alpha->data[fb->data[0] - 1] > alpha->data
                                [fb->data[1] - 1]) || (rtIsNaN(alpha->data
                                 [fb->data[0] - 1]) && (!rtIsNaN(alpha->data
                                  [fb->data[1] - 1])))) {
                      cartesian_trajectory_planner_B.s_e = alpha->data[fb->data
                        [1] - 1];
                      cartesian_trajectory_planner_B.idxl = 1;
                    } else {
                      cartesian_trajectory_planner_B.s_e = alpha->data[fb->data
                        [0] - 1];
                      cartesian_trajectory_planner_B.idxl = 0;
                    }
                  } else {
                    if (!rtIsNaN(alpha->data[fb->data[0] - 1])) {
                      cartesian_trajectory_planner_B.idxl = 1;
                    } else {
                      cartesian_trajectory_planner_B.idxl = 0;
                      cartesian_trajectory_planner_B.b_i_o = 2;
                      exitg3 = false;
                      while ((!exitg3) && (cartesian_trajectory_planner_B.b_i_o <=
                                           fb->size[0])) {
                        if (!rtIsNaN(alpha->data[fb->
                                     data[cartesian_trajectory_planner_B.b_i_o -
                                     1] - 1])) {
                          cartesian_trajectory_planner_B.idxl =
                            cartesian_trajectory_planner_B.b_i_o;
                          exitg3 = true;
                        } else {
                          cartesian_trajectory_planner_B.b_i_o++;
                        }
                      }
                    }

                    if (cartesian_trajectory_planner_B.idxl == 0) {
                      cartesian_trajectory_planner_B.s_e = alpha->data[fb->data
                        [0] - 1];
                    } else {
                      cartesian_trajectory_planner_B.s_e = alpha->data[fb->
                        data[cartesian_trajectory_planner_B.idxl - 1] - 1];
                      cartesian_trajectory_planner_B.nx_k =
                        cartesian_trajectory_planner_B.idxl;
                      for (cartesian_trajectory_planner_B.b_i_o =
                           cartesian_trajectory_planner_B.idxl + 1;
                           cartesian_trajectory_planner_B.b_i_o <=
                           cartesian_trajectory_planner_B.n_b;
                           cartesian_trajectory_planner_B.b_i_o++) {
                        if (cartesian_trajectory_planner_B.s_e > alpha->data
                            [fb->data[cartesian_trajectory_planner_B.b_i_o - 1]
                            - 1]) {
                          cartesian_trajectory_planner_B.s_e = alpha->data
                            [fb->data[cartesian_trajectory_planner_B.b_i_o - 1]
                            - 1];
                          cartesian_trajectory_planner_B.nx_k =
                            cartesian_trajectory_planner_B.b_i_o;
                        }
                      }

                      cartesian_trajectory_planner_B.idxl =
                        cartesian_trajectory_planner_B.nx_k - 1;
                    }
                  }

                  cartesian_trajectory_planner_B.idxl = ii->data
                    [static_cast<int32_T>(L->
                    data[cartesian_trajectory_planner_B.idxl]) - 1];
                } else {
                  cartesian_trajectory_planner_B.s_e = 0.0;
                }
              } else {
                cartesian_trajectory_planner_B.s_e = 0.0;
              }
            } else {
              cartesian_trajectory_planner_B.s_e = 0.0;
            }

            if (cartesian_trajectory_planner_B.s_e > 0.0) {
              if (1.0 < cartesian_trajectory_planner_B.s_e) {
                cartesian_trajectory_planner_B.b_gamma = 1.0;
              } else {
                cartesian_trajectory_planner_B.b_gamma =
                  cartesian_trajectory_planner_B.s_e;
              }
            } else {
              cartesian_trajectory_planner_B.b_gamma = 1.0;
            }

            cartesian_trajectory_planner_B.beta = obj->ArmijoRuleBeta;
            cartesian_trajectory_planner_B.sigma = obj->ArmijoRuleSigma;
            for (cartesian_trajectory_planner_B.b_i_o = 0;
                 cartesian_trajectory_planner_B.b_i_o < 6;
                 cartesian_trajectory_planner_B.b_i_o++) {
              cartesian_trajectory_planner_B.sNew_a[cartesian_trajectory_planner_B.b_i_o]
                = cartesian_trajectory_planner_B.b_gamma *
                cartesian_trajectory_planner_B.Hg[cartesian_trajectory_planner_B.b_i_o]
                + cartesian_trajectory_planner_B.x[cartesian_trajectory_planner_B.b_i_o];
            }

            cartesian_IKHelpers_computeCost
              (cartesian_trajectory_planner_B.sNew_a, obj->ExtraArgs,
               &cartesian_trajectory_planner_B.costNew,
               cartesian_trajectory_planner_B.V, unusedU1, &c);
            obj->ExtraArgs = c;
            cartesian_trajectory_planner_B.m = 0.0;
            do {
              exitg1 = 0;
              for (cartesian_trajectory_planner_B.i_n = 0;
                   cartesian_trajectory_planner_B.i_n < 6;
                   cartesian_trajectory_planner_B.i_n++) {
                xSol[cartesian_trajectory_planner_B.i_n] =
                  cartesian_trajectory_planner_B.b_gamma *
                  cartesian_trajectory_planner_B.Hg[cartesian_trajectory_planner_B.i_n];
              }

              cartesian_trajectory_planner_B.b_i_o = sigma->size[0] *
                sigma->size[1];
              sigma->size[0] = 1;
              sigma->size[1] = grad->size[0];
              cartes_emxEnsureCapacity_real_T(sigma,
                cartesian_trajectory_planner_B.b_i_o);
              cartesian_trajectory_planner_B.i_n = grad->size[0];
              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o <
                   cartesian_trajectory_planner_B.i_n;
                   cartesian_trajectory_planner_B.b_i_o++) {
                sigma->data[cartesian_trajectory_planner_B.b_i_o] =
                  -cartesian_trajectory_planner_B.sigma * grad->
                  data[cartesian_trajectory_planner_B.b_i_o];
              }

              cartesian_trajectory_planner_B.sigma_i = 0.0;
              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o < 6;
                   cartesian_trajectory_planner_B.b_i_o++) {
                cartesian_trajectory_planner_B.sigma_i += sigma->
                  data[cartesian_trajectory_planner_B.b_i_o] *
                  xSol[cartesian_trajectory_planner_B.b_i_o];
              }

              if (cartesian_trajectory_planner_B.cost -
                  cartesian_trajectory_planner_B.costNew <
                  cartesian_trajectory_planner_B.sigma_i) {
                cartesian_trajectory_planner_B.flag =
                  (cartesian_trajectory_planner_B.b_gamma < obj->StepTolerance);
                if (cartesian_trajectory_planner_B.flag) {
                  for (cartesian_trajectory_planner_B.i_n = 0;
                       cartesian_trajectory_planner_B.i_n < 6;
                       cartesian_trajectory_planner_B.i_n++) {
                    xSol[cartesian_trajectory_planner_B.i_n] =
                      cartesian_trajectory_planner_B.x[cartesian_trajectory_planner_B.i_n];
                  }

                  *exitFlag = StepSizeBelowMinimum;
                  args = obj->ExtraArgs;
                  for (cartesian_trajectory_planner_B.b_i_o = 0;
                       cartesian_trajectory_planner_B.b_i_o < 36;
                       cartesian_trajectory_planner_B.b_i_o++) {
                    cartesian_trajectory_planner_B.unusedU0[cartesian_trajectory_planner_B.b_i_o]
                      = args->WeightMatrix[cartesian_trajectory_planner_B.b_i_o];
                  }

                  cartesian_trajectory_planner_B.b_i_o = grad->size[0];
                  grad->size[0] = args->ErrTemp->size[0];
                  cartes_emxEnsureCapacity_real_T(grad,
                    cartesian_trajectory_planner_B.b_i_o);
                  cartesian_trajectory_planner_B.i_n = args->ErrTemp->size[0];
                  for (cartesian_trajectory_planner_B.b_i_o = 0;
                       cartesian_trajectory_planner_B.b_i_o <
                       cartesian_trajectory_planner_B.i_n;
                       cartesian_trajectory_planner_B.b_i_o++) {
                    grad->data[cartesian_trajectory_planner_B.b_i_o] =
                      args->ErrTemp->data[cartesian_trajectory_planner_B.b_i_o];
                  }

                  for (cartesian_trajectory_planner_B.b_i_o = 0;
                       cartesian_trajectory_planner_B.b_i_o < 6;
                       cartesian_trajectory_planner_B.b_i_o++) {
                    cartesian_trajectory_planner_B.x[cartesian_trajectory_planner_B.b_i_o]
                      = 0.0;
                    for (cartesian_trajectory_planner_B.idx = 0;
                         cartesian_trajectory_planner_B.idx < 6;
                         cartesian_trajectory_planner_B.idx++) {
                      cartesian_trajectory_planner_B.A_d =
                        cartesian_trajectory_planner_B.unusedU0[6 *
                        cartesian_trajectory_planner_B.idx +
                        cartesian_trajectory_planner_B.b_i_o] * grad->
                        data[cartesian_trajectory_planner_B.idx] +
                        cartesian_trajectory_planner_B.x[cartesian_trajectory_planner_B.b_i_o];
                      cartesian_trajectory_planner_B.x[cartesian_trajectory_planner_B.b_i_o]
                        = cartesian_trajectory_planner_B.A_d;
                    }
                  }

                  *err = cartesian_trajectory_pla_norm_a
                    (cartesian_trajectory_planner_B.x);
                  *iter = static_cast<real_T>
                    (cartesian_trajectory_planner_B.g_idx_0) + 1.0;
                  exitg1 = 1;
                } else {
                  cartesian_trajectory_planner_B.b_gamma *=
                    cartesian_trajectory_planner_B.beta;
                  cartesian_trajectory_planner_B.m++;
                  for (cartesian_trajectory_planner_B.b_i_o = 0;
                       cartesian_trajectory_planner_B.b_i_o < 6;
                       cartesian_trajectory_planner_B.b_i_o++) {
                    cartesian_trajectory_planner_B.sNew_a[cartesian_trajectory_planner_B.b_i_o]
                      = cartesian_trajectory_planner_B.b_gamma *
                      cartesian_trajectory_planner_B.Hg[cartesian_trajectory_planner_B.b_i_o]
                      + cartesian_trajectory_planner_B.x[cartesian_trajectory_planner_B.b_i_o];
                  }

                  cartesian_IKHelpers_computeCost
                    (cartesian_trajectory_planner_B.sNew_a, obj->ExtraArgs,
                     &cartesian_trajectory_planner_B.costNew,
                     cartesian_trajectory_planner_B.V, unusedU1, &d);
                  obj->ExtraArgs = d;
                }
              } else {
                for (cartesian_trajectory_planner_B.b_i_o = 0;
                     cartesian_trajectory_planner_B.b_i_o < 6;
                     cartesian_trajectory_planner_B.b_i_o++) {
                  xSol[cartesian_trajectory_planner_B.b_i_o] +=
                    cartesian_trajectory_planner_B.x[cartesian_trajectory_planner_B.b_i_o];
                }

                args = obj->ExtraArgs;
                cartesian_trajectory_planner_B.b_i_o = alpha->size[0];
                alpha->size[0] = args->GradTemp->size[0];
                cartes_emxEnsureCapacity_real_T(alpha,
                  cartesian_trajectory_planner_B.b_i_o);
                cartesian_trajectory_planner_B.i_n = args->GradTemp->size[0];
                for (cartesian_trajectory_planner_B.b_i_o = 0;
                     cartesian_trajectory_planner_B.b_i_o <
                     cartesian_trajectory_planner_B.i_n;
                     cartesian_trajectory_planner_B.b_i_o++) {
                  alpha->data[cartesian_trajectory_planner_B.b_i_o] =
                    args->GradTemp->data[cartesian_trajectory_planner_B.b_i_o];
                }

                exitg1 = 2;
              }
            } while (exitg1 == 0);

            if (exitg1 == 1) {
              exitg2 = 1;
            } else if ((cartesian_trajectory_planner_B.m == 0.0) && (fabs
                        (cartesian_trajectory_planner_B.b_gamma -
                         cartesian_trajectory_planner_B.s_e) <
                        1.4901161193847656E-8)) {
              cartesian_trajectory_planner_B.i_n = obj->ConstraintMatrix->size[0];
              cartesian_trajectory_planner_B.b_i_o = grad->size[0];
              grad->size[0] = cartesian_trajectory_planner_B.i_n;
              cartes_emxEnsureCapacity_real_T(grad,
                cartesian_trajectory_planner_B.b_i_o);
              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o <
                   cartesian_trajectory_planner_B.i_n;
                   cartesian_trajectory_planner_B.b_i_o++) {
                grad->data[cartesian_trajectory_planner_B.b_i_o] =
                  obj->ConstraintMatrix->data
                  [(cartesian_trajectory_planner_B.idxl - 1) *
                  obj->ConstraintMatrix->size[0] +
                  cartesian_trajectory_planner_B.b_i_o];
              }

              activeSet->data[cartesian_trajectory_planner_B.idxl - 1] = true;
              cartesian_trajectory_planner_B.nx_k = activeSet->size[0] - 1;
              cartesian_trajectory_planner_B.idx = 0;
              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o <=
                   cartesian_trajectory_planner_B.nx_k;
                   cartesian_trajectory_planner_B.b_i_o++) {
                if (activeSet->data[cartesian_trajectory_planner_B.b_i_o]) {
                  cartesian_trajectory_planner_B.idx++;
                }
              }

              cartesian_trajectory_planner_B.b_i_o = gb->size[0];
              gb->size[0] = cartesian_trajectory_planner_B.idx;
              carte_emxEnsureCapacity_int32_T(gb,
                cartesian_trajectory_planner_B.b_i_o);
              cartesian_trajectory_planner_B.idx = 0;
              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o <=
                   cartesian_trajectory_planner_B.nx_k;
                   cartesian_trajectory_planner_B.b_i_o++) {
                if (activeSet->data[cartesian_trajectory_planner_B.b_i_o]) {
                  gb->data[cartesian_trajectory_planner_B.idx] =
                    cartesian_trajectory_planner_B.b_i_o + 1;
                  cartesian_trajectory_planner_B.idx++;
                }
              }

              cartesian_trajectory_planner_B.i_n = obj->ConstraintMatrix->size[0];
              cartesian_trajectory_planner_B.b_i_o = A->size[0] * A->size[1];
              A->size[0] = cartesian_trajectory_planner_B.i_n;
              A->size[1] = gb->size[0];
              cartes_emxEnsureCapacity_real_T(A,
                cartesian_trajectory_planner_B.b_i_o);
              cartesian_trajectory_planner_B.n_b = gb->size[0];
              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o <
                   cartesian_trajectory_planner_B.n_b;
                   cartesian_trajectory_planner_B.b_i_o++) {
                for (cartesian_trajectory_planner_B.idx = 0;
                     cartesian_trajectory_planner_B.idx <
                     cartesian_trajectory_planner_B.i_n;
                     cartesian_trajectory_planner_B.idx++) {
                  A->data[cartesian_trajectory_planner_B.idx + A->size[0] *
                    cartesian_trajectory_planner_B.b_i_o] =
                    obj->ConstraintMatrix->data[(gb->
                    data[cartesian_trajectory_planner_B.b_i_o] - 1) *
                    obj->ConstraintMatrix->size[0] +
                    cartesian_trajectory_planner_B.idx];
                }
              }

              cartesian_trajectory_planner_B.b_i_o = AIn->size[0] * AIn->size[1];
              AIn->size[0] = grad->size[0];
              AIn->size[1] = grad->size[0];
              cartes_emxEnsureCapacity_real_T(AIn,
                cartesian_trajectory_planner_B.b_i_o);
              cartesian_trajectory_planner_B.i_n = grad->size[0];
              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o <
                   cartesian_trajectory_planner_B.i_n;
                   cartesian_trajectory_planner_B.b_i_o++) {
                cartesian_trajectory_planner_B.n_b = grad->size[0];
                for (cartesian_trajectory_planner_B.idx = 0;
                     cartesian_trajectory_planner_B.idx <
                     cartesian_trajectory_planner_B.n_b;
                     cartesian_trajectory_planner_B.idx++) {
                  AIn->data[cartesian_trajectory_planner_B.idx + AIn->size[0] *
                    cartesian_trajectory_planner_B.b_i_o] = grad->
                    data[cartesian_trajectory_planner_B.idx] * grad->
                    data[cartesian_trajectory_planner_B.b_i_o];
                }
              }

              cartesian_trajectory_planner_B.m_hz = AIn->size[0] - 1;
              cartesian_trajectory_planner_B.inner = AIn->size[1] - 1;
              cartesian_trajectory_planner_B.b_i_o = y_0->size[0] * y_0->size[1];
              y_0->size[0] = AIn->size[0];
              y_0->size[1] = 6;
              cartes_emxEnsureCapacity_real_T(y_0,
                cartesian_trajectory_planner_B.b_i_o);
              for (cartesian_trajectory_planner_B.idx = 0;
                   cartesian_trajectory_planner_B.idx < 6;
                   cartesian_trajectory_planner_B.idx++) {
                cartesian_trajectory_planner_B.coffset =
                  (cartesian_trajectory_planner_B.m_hz + 1) *
                  cartesian_trajectory_planner_B.idx - 1;
                cartesian_trajectory_planner_B.boffset =
                  cartesian_trajectory_planner_B.idx * 6 - 1;
                for (cartesian_trajectory_planner_B.b_i_o = 0;
                     cartesian_trajectory_planner_B.b_i_o <=
                     cartesian_trajectory_planner_B.m_hz;
                     cartesian_trajectory_planner_B.b_i_o++) {
                  y_0->data[(cartesian_trajectory_planner_B.coffset +
                             cartesian_trajectory_planner_B.b_i_o) + 1] = 0.0;
                }

                for (cartesian_trajectory_planner_B.nx_k = 0;
                     cartesian_trajectory_planner_B.nx_k <=
                     cartesian_trajectory_planner_B.inner;
                     cartesian_trajectory_planner_B.nx_k++) {
                  cartesian_trajectory_planner_B.aoffset =
                    cartesian_trajectory_planner_B.nx_k * AIn->size[0] - 1;
                  cartesian_trajectory_planner_B.s_e =
                    cartesian_trajectory_planner_B.H
                    [(cartesian_trajectory_planner_B.boffset +
                      cartesian_trajectory_planner_B.nx_k) + 1];
                  for (cartesian_trajectory_planner_B.j = 0;
                       cartesian_trajectory_planner_B.j <=
                       cartesian_trajectory_planner_B.m_hz;
                       cartesian_trajectory_planner_B.j++) {
                    cartesian_trajectory_planner_B.i_n =
                      cartesian_trajectory_planner_B.j + 1;
                    cartesian_trajectory_planner_B.b_i_o =
                      cartesian_trajectory_planner_B.coffset +
                      cartesian_trajectory_planner_B.i_n;
                    y_0->data[cartesian_trajectory_planner_B.b_i_o] += AIn->
                      data[cartesian_trajectory_planner_B.aoffset +
                      cartesian_trajectory_planner_B.i_n] *
                      cartesian_trajectory_planner_B.s_e;
                  }
                }
              }

              cartesian_trajectory_planner_B.b_i_o = grad_1->size[0] *
                grad_1->size[1];
              grad_1->size[0] = 1;
              grad_1->size[1] = grad->size[0];
              cartes_emxEnsureCapacity_real_T(grad_1,
                cartesian_trajectory_planner_B.b_i_o);
              cartesian_trajectory_planner_B.i_n = grad->size[0];
              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o <
                   cartesian_trajectory_planner_B.i_n;
                   cartesian_trajectory_planner_B.b_i_o++) {
                grad_1->data[cartesian_trajectory_planner_B.b_i_o] = grad->
                  data[cartesian_trajectory_planner_B.b_i_o];
              }

              cartesian_trajectory_planner_B.beta = 0.0;
              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o < 6;
                   cartesian_trajectory_planner_B.b_i_o++) {
                cartesian_trajectory_planner_B.sNew_a[cartesian_trajectory_planner_B.b_i_o]
                  = 0.0;
                for (cartesian_trajectory_planner_B.idx = 0;
                     cartesian_trajectory_planner_B.idx < 6;
                     cartesian_trajectory_planner_B.idx++) {
                  cartesian_trajectory_planner_B.sigma =
                    cartesian_trajectory_planner_B.H[6 *
                    cartesian_trajectory_planner_B.b_i_o +
                    cartesian_trajectory_planner_B.idx] * grad_1->
                    data[cartesian_trajectory_planner_B.idx] +
                    cartesian_trajectory_planner_B.sNew_a[cartesian_trajectory_planner_B.b_i_o];
                  cartesian_trajectory_planner_B.sNew_a[cartesian_trajectory_planner_B.b_i_o]
                    = cartesian_trajectory_planner_B.sigma;
                }

                cartesian_trajectory_planner_B.beta +=
                  cartesian_trajectory_planner_B.sNew_a[cartesian_trajectory_planner_B.b_i_o]
                  * grad->data[cartesian_trajectory_planner_B.b_i_o];
              }

              cartesian_trajectory_planner_B.s_e = 1.0 /
                cartesian_trajectory_planner_B.beta;
              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o < 6;
                   cartesian_trajectory_planner_B.b_i_o++) {
                for (cartesian_trajectory_planner_B.idx = 0;
                     cartesian_trajectory_planner_B.idx < 6;
                     cartesian_trajectory_planner_B.idx++) {
                  cartesian_trajectory_planner_B.idxl =
                    cartesian_trajectory_planner_B.b_i_o + 6 *
                    cartesian_trajectory_planner_B.idx;
                  cartesian_trajectory_planner_B.H_m[cartesian_trajectory_planner_B.idxl]
                    = 0.0;
                  for (cartesian_trajectory_planner_B.i_n = 0;
                       cartesian_trajectory_planner_B.i_n < 6;
                       cartesian_trajectory_planner_B.i_n++) {
                    cartesian_trajectory_planner_B.H_m[cartesian_trajectory_planner_B.idxl]
                      += cartesian_trajectory_planner_B.H[6 *
                      cartesian_trajectory_planner_B.i_n +
                      cartesian_trajectory_planner_B.b_i_o] * y_0->data[6 *
                      cartesian_trajectory_planner_B.idx +
                      cartesian_trajectory_planner_B.i_n];
                  }
                }
              }

              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o < 36;
                   cartesian_trajectory_planner_B.b_i_o++) {
                cartesian_trajectory_planner_B.H[cartesian_trajectory_planner_B.b_i_o]
                  -= cartesian_trajectory_planner_B.s_e *
                  cartesian_trajectory_planner_B.H_m[cartesian_trajectory_planner_B.b_i_o];
              }

              guard1 = true;
            } else {
              cartesian_trajectory_planner_B.b_i_o = grad->size[0];
              grad->size[0] = alpha->size[0];
              cartes_emxEnsureCapacity_real_T(grad,
                cartesian_trajectory_planner_B.b_i_o);
              cartesian_trajectory_planner_B.i_n = alpha->size[0];
              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o <
                   cartesian_trajectory_planner_B.i_n;
                   cartesian_trajectory_planner_B.b_i_o++) {
                grad->data[cartesian_trajectory_planner_B.b_i_o] = alpha->
                  data[cartesian_trajectory_planner_B.b_i_o] - grad->
                  data[cartesian_trajectory_planner_B.b_i_o];
              }

              cartesian_trajectory_planner_B.b_gamma = 0.0;
              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o < 6;
                   cartesian_trajectory_planner_B.b_i_o++) {
                cartesian_trajectory_planner_B.b_gamma +=
                  cartesian_trajectory_planner_B.Hg[cartesian_trajectory_planner_B.b_i_o]
                  * grad->data[cartesian_trajectory_planner_B.b_i_o];
              }

              cartesian_trajectory_planner_B.b_i_o = tmp->size[0] * tmp->size[1];
              tmp->size[0] = 1;
              tmp->size[1] = grad->size[0];
              cartes_emxEnsureCapacity_real_T(tmp,
                cartesian_trajectory_planner_B.b_i_o);
              cartesian_trajectory_planner_B.i_n = grad->size[0];
              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o <
                   cartesian_trajectory_planner_B.i_n;
                   cartesian_trajectory_planner_B.b_i_o++) {
                tmp->data[cartesian_trajectory_planner_B.b_i_o] = 0.2 *
                  grad->data[cartesian_trajectory_planner_B.b_i_o];
              }

              cartesian_trajectory_planner_B.s_e = 0.0;
              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o < 6;
                   cartesian_trajectory_planner_B.b_i_o++) {
                cartesian_trajectory_planner_B.sNew_a[cartesian_trajectory_planner_B.b_i_o]
                  = 0.0;
                for (cartesian_trajectory_planner_B.idx = 0;
                     cartesian_trajectory_planner_B.idx < 6;
                     cartesian_trajectory_planner_B.idx++) {
                  cartesian_trajectory_planner_B.beta =
                    cartesian_trajectory_planner_B.H[6 *
                    cartesian_trajectory_planner_B.b_i_o +
                    cartesian_trajectory_planner_B.idx] * tmp->
                    data[cartesian_trajectory_planner_B.idx] +
                    cartesian_trajectory_planner_B.sNew_a[cartesian_trajectory_planner_B.b_i_o];
                  cartesian_trajectory_planner_B.sNew_a[cartesian_trajectory_planner_B.b_i_o]
                    = cartesian_trajectory_planner_B.beta;
                }

                cartesian_trajectory_planner_B.s_e +=
                  cartesian_trajectory_planner_B.sNew_a[cartesian_trajectory_planner_B.b_i_o]
                  * grad->data[cartesian_trajectory_planner_B.b_i_o];
              }

              if (cartesian_trajectory_planner_B.b_gamma <
                  cartesian_trajectory_planner_B.s_e) {
                cartesian_trajectory_planner_B.b_i_o = tmp_0->size[0] *
                  tmp_0->size[1];
                tmp_0->size[0] = 1;
                tmp_0->size[1] = grad->size[0];
                cartes_emxEnsureCapacity_real_T(tmp_0,
                  cartesian_trajectory_planner_B.b_i_o);
                cartesian_trajectory_planner_B.i_n = grad->size[0];
                for (cartesian_trajectory_planner_B.b_i_o = 0;
                     cartesian_trajectory_planner_B.b_i_o <
                     cartesian_trajectory_planner_B.i_n;
                     cartesian_trajectory_planner_B.b_i_o++) {
                  tmp_0->data[cartesian_trajectory_planner_B.b_i_o] = 0.8 *
                    grad->data[cartesian_trajectory_planner_B.b_i_o];
                }

                cartesian_trajectory_planner_B.s_e = 0.0;
                for (cartesian_trajectory_planner_B.b_i_o = 0;
                     cartesian_trajectory_planner_B.b_i_o < 6;
                     cartesian_trajectory_planner_B.b_i_o++) {
                  cartesian_trajectory_planner_B.sNew_a[cartesian_trajectory_planner_B.b_i_o]
                    = 0.0;
                  for (cartesian_trajectory_planner_B.idx = 0;
                       cartesian_trajectory_planner_B.idx < 6;
                       cartesian_trajectory_planner_B.idx++) {
                    cartesian_trajectory_planner_B.beta =
                      cartesian_trajectory_planner_B.H[6 *
                      cartesian_trajectory_planner_B.b_i_o +
                      cartesian_trajectory_planner_B.idx] * tmp_0->
                      data[cartesian_trajectory_planner_B.idx] +
                      cartesian_trajectory_planner_B.sNew_a[cartesian_trajectory_planner_B.b_i_o];
                    cartesian_trajectory_planner_B.sNew_a[cartesian_trajectory_planner_B.b_i_o]
                      = cartesian_trajectory_planner_B.beta;
                  }

                  cartesian_trajectory_planner_B.s_e +=
                    cartesian_trajectory_planner_B.sNew_a[cartesian_trajectory_planner_B.b_i_o]
                    * grad->data[cartesian_trajectory_planner_B.b_i_o];
                }

                cartesian_trajectory_planner_B.b_i_o = grad_0->size[0] *
                  grad_0->size[1];
                grad_0->size[0] = 1;
                grad_0->size[1] = grad->size[0];
                cartes_emxEnsureCapacity_real_T(grad_0,
                  cartesian_trajectory_planner_B.b_i_o);
                cartesian_trajectory_planner_B.i_n = grad->size[0];
                for (cartesian_trajectory_planner_B.b_i_o = 0;
                     cartesian_trajectory_planner_B.b_i_o <
                     cartesian_trajectory_planner_B.i_n;
                     cartesian_trajectory_planner_B.b_i_o++) {
                  grad_0->data[cartesian_trajectory_planner_B.b_i_o] =
                    grad->data[cartesian_trajectory_planner_B.b_i_o];
                }

                cartesian_trajectory_planner_B.beta = 0.0;
                cartesian_trajectory_planner_B.b_gamma = 0.0;
                for (cartesian_trajectory_planner_B.b_i_o = 0;
                     cartesian_trajectory_planner_B.b_i_o < 6;
                     cartesian_trajectory_planner_B.b_i_o++) {
                  cartesian_trajectory_planner_B.sNew_a[cartesian_trajectory_planner_B.b_i_o]
                    = 0.0;
                  for (cartesian_trajectory_planner_B.idx = 0;
                       cartesian_trajectory_planner_B.idx < 6;
                       cartesian_trajectory_planner_B.idx++) {
                    cartesian_trajectory_planner_B.sigma =
                      cartesian_trajectory_planner_B.H[6 *
                      cartesian_trajectory_planner_B.b_i_o +
                      cartesian_trajectory_planner_B.idx] * grad_0->
                      data[cartesian_trajectory_planner_B.idx] +
                      cartesian_trajectory_planner_B.sNew_a[cartesian_trajectory_planner_B.b_i_o];
                    cartesian_trajectory_planner_B.sNew_a[cartesian_trajectory_planner_B.b_i_o]
                      = cartesian_trajectory_planner_B.sigma;
                  }

                  cartesian_trajectory_planner_B.beta +=
                    cartesian_trajectory_planner_B.sNew_a[cartesian_trajectory_planner_B.b_i_o]
                    * grad->data[cartesian_trajectory_planner_B.b_i_o];
                  cartesian_trajectory_planner_B.b_gamma +=
                    cartesian_trajectory_planner_B.Hg[cartesian_trajectory_planner_B.b_i_o]
                    * grad->data[cartesian_trajectory_planner_B.b_i_o];
                }

                cartesian_trajectory_planner_B.b_gamma =
                  cartesian_trajectory_planner_B.s_e /
                  (cartesian_trajectory_planner_B.beta -
                   cartesian_trajectory_planner_B.b_gamma);
              } else {
                cartesian_trajectory_planner_B.b_gamma = 1.0;
              }

              cartesian_trajectory_planner_B.beta = 0.0;
              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o < 6;
                   cartesian_trajectory_planner_B.b_i_o++) {
                cartesian_trajectory_planner_B.s_e = 0.0;
                for (cartesian_trajectory_planner_B.idx = 0;
                     cartesian_trajectory_planner_B.idx < 6;
                     cartesian_trajectory_planner_B.idx++) {
                  cartesian_trajectory_planner_B.s_e +=
                    cartesian_trajectory_planner_B.H[6 *
                    cartesian_trajectory_planner_B.idx +
                    cartesian_trajectory_planner_B.b_i_o] * (1.0 -
                    cartesian_trajectory_planner_B.b_gamma) * grad->
                    data[cartesian_trajectory_planner_B.idx];
                }

                cartesian_trajectory_planner_B.s_e +=
                  cartesian_trajectory_planner_B.b_gamma *
                  cartesian_trajectory_planner_B.Hg[cartesian_trajectory_planner_B.b_i_o];
                cartesian_trajectory_planner_B.beta +=
                  cartesian_trajectory_planner_B.s_e * grad->
                  data[cartesian_trajectory_planner_B.b_i_o];
                cartesian_trajectory_planner_B.sNew_a[cartesian_trajectory_planner_B.b_i_o]
                  = cartesian_trajectory_planner_B.s_e;
              }

              cartesian_trajectory_planner_B.b_i_o = sNew->size[0] * sNew->size
                [1];
              sNew->size[0] = 6;
              sNew->size[1] = grad->size[0];
              cartes_emxEnsureCapacity_real_T(sNew,
                cartesian_trajectory_planner_B.b_i_o);
              cartesian_trajectory_planner_B.i_n = grad->size[0];
              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o <
                   cartesian_trajectory_planner_B.i_n;
                   cartesian_trajectory_planner_B.b_i_o++) {
                for (cartesian_trajectory_planner_B.idx = 0;
                     cartesian_trajectory_planner_B.idx < 6;
                     cartesian_trajectory_planner_B.idx++) {
                  cartesian_trajectory_planner_B.s_e =
                    cartesian_trajectory_planner_B.sNew_a[cartesian_trajectory_planner_B.idx]
                    * grad->data[cartesian_trajectory_planner_B.b_i_o];
                  sNew->data[cartesian_trajectory_planner_B.idx + 6 *
                    cartesian_trajectory_planner_B.b_i_o] =
                    cartesian_trajectory_planner_B.s_e /
                    cartesian_trajectory_planner_B.beta;
                }
              }

              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o < 36;
                   cartesian_trajectory_planner_B.b_i_o++) {
                cartesian_trajectory_planner_B.V[cartesian_trajectory_planner_B.b_i_o]
                  =
                  cartesian_trajectory_planner_B.unusedU0[cartesian_trajectory_planner_B.b_i_o]
                  - sNew->data[cartesian_trajectory_planner_B.b_i_o];
              }

              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o < 6;
                   cartesian_trajectory_planner_B.b_i_o++) {
                for (cartesian_trajectory_planner_B.idx = 0;
                     cartesian_trajectory_planner_B.idx < 6;
                     cartesian_trajectory_planner_B.idx++) {
                  cartesian_trajectory_planner_B.nx_k =
                    cartesian_trajectory_planner_B.b_i_o + 6 *
                    cartesian_trajectory_planner_B.idx;
                  cartesian_trajectory_planner_B.H_m[cartesian_trajectory_planner_B.nx_k]
                    = 0.0;
                  for (cartesian_trajectory_planner_B.i_n = 0;
                       cartesian_trajectory_planner_B.i_n < 6;
                       cartesian_trajectory_planner_B.i_n++) {
                    cartesian_trajectory_planner_B.H_m[cartesian_trajectory_planner_B.nx_k]
                      += cartesian_trajectory_planner_B.V[6 *
                      cartesian_trajectory_planner_B.i_n +
                      cartesian_trajectory_planner_B.b_i_o] *
                      cartesian_trajectory_planner_B.H[6 *
                      cartesian_trajectory_planner_B.idx +
                      cartesian_trajectory_planner_B.i_n];
                  }
                }
              }

              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o < 6;
                   cartesian_trajectory_planner_B.b_i_o++) {
                for (cartesian_trajectory_planner_B.idx = 0;
                     cartesian_trajectory_planner_B.idx < 6;
                     cartesian_trajectory_planner_B.idx++) {
                  cartesian_trajectory_planner_B.nx_k =
                    cartesian_trajectory_planner_B.b_i_o + 6 *
                    cartesian_trajectory_planner_B.idx;
                  cartesian_trajectory_planner_B.P[cartesian_trajectory_planner_B.nx_k]
                    = 0.0;
                  for (cartesian_trajectory_planner_B.i_n = 0;
                       cartesian_trajectory_planner_B.i_n < 6;
                       cartesian_trajectory_planner_B.i_n++) {
                    cartesian_trajectory_planner_B.P[cartesian_trajectory_planner_B.nx_k]
                      += cartesian_trajectory_planner_B.H_m[6 *
                      cartesian_trajectory_planner_B.i_n +
                      cartesian_trajectory_planner_B.b_i_o] *
                      cartesian_trajectory_planner_B.V[6 *
                      cartesian_trajectory_planner_B.i_n +
                      cartesian_trajectory_planner_B.idx];
                  }
                }
              }

              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o < 6;
                   cartesian_trajectory_planner_B.b_i_o++) {
                for (cartesian_trajectory_planner_B.idx = 0;
                     cartesian_trajectory_planner_B.idx < 6;
                     cartesian_trajectory_planner_B.idx++) {
                  cartesian_trajectory_planner_B.sNew[cartesian_trajectory_planner_B.idx
                    + 6 * cartesian_trajectory_planner_B.b_i_o] =
                    cartesian_trajectory_planner_B.sNew_a[cartesian_trajectory_planner_B.idx]
                    * cartesian_trajectory_planner_B.sNew_a[cartesian_trajectory_planner_B.b_i_o]
                    / cartesian_trajectory_planner_B.beta;
                }
              }

              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o < 36;
                   cartesian_trajectory_planner_B.b_i_o++) {
                cartesian_trajectory_planner_B.s_e =
                  cartesian_trajectory_planner_B.P[cartesian_trajectory_planner_B.b_i_o]
                  + cartesian_trajectory_planner_B.sNew[cartesian_trajectory_planner_B.b_i_o];
                cartesian_trajectory_planner_B.H_m[cartesian_trajectory_planner_B.b_i_o]
                  = 1.4901161193847656E-8 * static_cast<real_T>
                  (tmp_1[cartesian_trajectory_planner_B.b_i_o]) +
                  cartesian_trajectory_planner_B.s_e;
                cartesian_trajectory_planner_B.H[cartesian_trajectory_planner_B.b_i_o]
                  = cartesian_trajectory_planner_B.s_e;
              }

              if (!cartesian_tr_isPositiveDefinite
                  (cartesian_trajectory_planner_B.H_m)) {
                *exitFlag = HessianNotPositiveSemidefinite;
                args = obj->ExtraArgs;
                for (cartesian_trajectory_planner_B.b_i_o = 0;
                     cartesian_trajectory_planner_B.b_i_o < 36;
                     cartesian_trajectory_planner_B.b_i_o++) {
                  cartesian_trajectory_planner_B.unusedU0[cartesian_trajectory_planner_B.b_i_o]
                    = args->WeightMatrix[cartesian_trajectory_planner_B.b_i_o];
                }

                cartesian_trajectory_planner_B.b_i_o = grad->size[0];
                grad->size[0] = args->ErrTemp->size[0];
                cartes_emxEnsureCapacity_real_T(grad,
                  cartesian_trajectory_planner_B.b_i_o);
                cartesian_trajectory_planner_B.i_n = args->ErrTemp->size[0];
                for (cartesian_trajectory_planner_B.b_i_o = 0;
                     cartesian_trajectory_planner_B.b_i_o <
                     cartesian_trajectory_planner_B.i_n;
                     cartesian_trajectory_planner_B.b_i_o++) {
                  grad->data[cartesian_trajectory_planner_B.b_i_o] =
                    args->ErrTemp->data[cartesian_trajectory_planner_B.b_i_o];
                }

                for (cartesian_trajectory_planner_B.b_i_o = 0;
                     cartesian_trajectory_planner_B.b_i_o < 6;
                     cartesian_trajectory_planner_B.b_i_o++) {
                  cartesian_trajectory_planner_B.x[cartesian_trajectory_planner_B.b_i_o]
                    = 0.0;
                  for (cartesian_trajectory_planner_B.idx = 0;
                       cartesian_trajectory_planner_B.idx < 6;
                       cartesian_trajectory_planner_B.idx++) {
                    cartesian_trajectory_planner_B.A_d =
                      cartesian_trajectory_planner_B.unusedU0[6 *
                      cartesian_trajectory_planner_B.idx +
                      cartesian_trajectory_planner_B.b_i_o] * grad->
                      data[cartesian_trajectory_planner_B.idx] +
                      cartesian_trajectory_planner_B.x[cartesian_trajectory_planner_B.b_i_o];
                    cartesian_trajectory_planner_B.x[cartesian_trajectory_planner_B.b_i_o]
                      = cartesian_trajectory_planner_B.A_d;
                  }
                }

                *err = cartesian_trajectory_pla_norm_a
                  (cartesian_trajectory_planner_B.x);
                *iter = static_cast<real_T>
                  (cartesian_trajectory_planner_B.g_idx_0) + 1.0;
                exitg2 = 1;
              } else {
                guard1 = true;
              }
            }
          }

          if (guard1) {
            if (DampedBFGSwGradientProjectio_as(obj, xSol)) {
              for (cartesian_trajectory_planner_B.i_n = 0;
                   cartesian_trajectory_planner_B.i_n < 6;
                   cartesian_trajectory_planner_B.i_n++) {
                xSol[cartesian_trajectory_planner_B.i_n] =
                  cartesian_trajectory_planner_B.x[cartesian_trajectory_planner_B.i_n];
              }

              *exitFlag = SearchDirectionInvalid;
              args = obj->ExtraArgs;
              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o < 36;
                   cartesian_trajectory_planner_B.b_i_o++) {
                cartesian_trajectory_planner_B.unusedU0[cartesian_trajectory_planner_B.b_i_o]
                  = args->WeightMatrix[cartesian_trajectory_planner_B.b_i_o];
              }

              cartesian_trajectory_planner_B.b_i_o = grad->size[0];
              grad->size[0] = args->ErrTemp->size[0];
              cartes_emxEnsureCapacity_real_T(grad,
                cartesian_trajectory_planner_B.b_i_o);
              cartesian_trajectory_planner_B.i_n = args->ErrTemp->size[0];
              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o <
                   cartesian_trajectory_planner_B.i_n;
                   cartesian_trajectory_planner_B.b_i_o++) {
                grad->data[cartesian_trajectory_planner_B.b_i_o] = args->
                  ErrTemp->data[cartesian_trajectory_planner_B.b_i_o];
              }

              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o < 6;
                   cartesian_trajectory_planner_B.b_i_o++) {
                cartesian_trajectory_planner_B.x[cartesian_trajectory_planner_B.b_i_o]
                  = 0.0;
                for (cartesian_trajectory_planner_B.idx = 0;
                     cartesian_trajectory_planner_B.idx < 6;
                     cartesian_trajectory_planner_B.idx++) {
                  cartesian_trajectory_planner_B.A_d =
                    cartesian_trajectory_planner_B.unusedU0[6 *
                    cartesian_trajectory_planner_B.idx +
                    cartesian_trajectory_planner_B.b_i_o] * grad->
                    data[cartesian_trajectory_planner_B.idx] +
                    cartesian_trajectory_planner_B.x[cartesian_trajectory_planner_B.b_i_o];
                  cartesian_trajectory_planner_B.x[cartesian_trajectory_planner_B.b_i_o]
                    = cartesian_trajectory_planner_B.A_d;
                }
              }

              *err = cartesian_trajectory_pla_norm_a
                (cartesian_trajectory_planner_B.x);
              *iter = static_cast<real_T>(cartesian_trajectory_planner_B.g_idx_0)
                + 1.0;
              exitg2 = 1;
            } else {
              for (cartesian_trajectory_planner_B.i_n = 0;
                   cartesian_trajectory_planner_B.i_n < 6;
                   cartesian_trajectory_planner_B.i_n++) {
                cartesian_trajectory_planner_B.x[cartesian_trajectory_planner_B.i_n]
                  = xSol[cartesian_trajectory_planner_B.i_n];
              }

              cartesian_trajectory_planner_B.b_i_o = grad->size[0];
              grad->size[0] = alpha->size[0];
              cartes_emxEnsureCapacity_real_T(grad,
                cartesian_trajectory_planner_B.b_i_o);
              cartesian_trajectory_planner_B.i_n = alpha->size[0];
              for (cartesian_trajectory_planner_B.b_i_o = 0;
                   cartesian_trajectory_planner_B.b_i_o <
                   cartesian_trajectory_planner_B.i_n;
                   cartesian_trajectory_planner_B.b_i_o++) {
                grad->data[cartesian_trajectory_planner_B.b_i_o] = alpha->
                  data[cartesian_trajectory_planner_B.b_i_o];
              }

              cartesian_trajectory_planner_B.cost =
                cartesian_trajectory_planner_B.costNew;
              cartesian_trajectory_planner_B.g_idx_0++;
            }
          }
        }
      }
    } else {
      *exitFlag = IterationLimitExceeded;
      args = obj->ExtraArgs;
      for (cartesian_trajectory_planner_B.b_i_o = 0;
           cartesian_trajectory_planner_B.b_i_o < 36;
           cartesian_trajectory_planner_B.b_i_o++) {
        cartesian_trajectory_planner_B.unusedU0[cartesian_trajectory_planner_B.b_i_o]
          = args->WeightMatrix[cartesian_trajectory_planner_B.b_i_o];
      }

      cartesian_trajectory_planner_B.b_i_o = grad->size[0];
      grad->size[0] = args->ErrTemp->size[0];
      cartes_emxEnsureCapacity_real_T(grad, cartesian_trajectory_planner_B.b_i_o);
      cartesian_trajectory_planner_B.i_n = args->ErrTemp->size[0];
      for (cartesian_trajectory_planner_B.b_i_o = 0;
           cartesian_trajectory_planner_B.b_i_o <
           cartesian_trajectory_planner_B.i_n;
           cartesian_trajectory_planner_B.b_i_o++) {
        grad->data[cartesian_trajectory_planner_B.b_i_o] = args->ErrTemp->
          data[cartesian_trajectory_planner_B.b_i_o];
      }

      for (cartesian_trajectory_planner_B.b_i_o = 0;
           cartesian_trajectory_planner_B.b_i_o < 6;
           cartesian_trajectory_planner_B.b_i_o++) {
        cartesian_trajectory_planner_B.x[cartesian_trajectory_planner_B.b_i_o] =
          0.0;
        for (cartesian_trajectory_planner_B.idx = 0;
             cartesian_trajectory_planner_B.idx < 6;
             cartesian_trajectory_planner_B.idx++) {
          cartesian_trajectory_planner_B.A_d =
            cartesian_trajectory_planner_B.unusedU0[6 *
            cartesian_trajectory_planner_B.idx +
            cartesian_trajectory_planner_B.b_i_o] * grad->
            data[cartesian_trajectory_planner_B.idx] +
            cartesian_trajectory_planner_B.x[cartesian_trajectory_planner_B.b_i_o];
          cartesian_trajectory_planner_B.x[cartesian_trajectory_planner_B.b_i_o]
            = cartesian_trajectory_planner_B.A_d;
        }
      }

      *err = cartesian_trajectory_pla_norm_a(cartesian_trajectory_planner_B.x);
      *iter = obj->MaxNumIterationInternal;
      exitg2 = 1;
    }
  } while (exitg2 == 0);

  cartesian_traje_emxFree_int32_T(&ii_3);
  cartesian_trajec_emxFree_real_T(&alpha_0);
  cartesian_trajec_emxFree_real_T(&A_3);
  cartesian_trajec_emxFree_real_T(&grad_1);
  cartesian_traje_emxFree_int32_T(&ii_2);
  cartesian_traje_emxFree_int32_T(&ii_1);
  cartesian_trajec_emxFree_real_T(&sNew);
  cartesian_trajec_emxFree_real_T(&grad_0);
  cartesian_trajec_emxFree_real_T(&tmp_0);
  cartesian_trajec_emxFree_real_T(&tmp);
  cartesian_trajec_emxFree_real_T(&sigma);
  cartesian_trajec_emxFree_real_T(&A_2);
  cartesian_tra_emxFree_boolean_T(&x);
  cartesian_trajec_emxFree_real_T(&y_0);
  cartesian_traje_emxFree_int32_T(&ii_0);
  cartesian_trajec_emxFree_real_T(&y);
  cartesian_traje_emxFree_int32_T(&ii);
  cartesian_trajec_emxFree_real_T(&a);
  cartesian_traje_emxFree_int32_T(&gb);
  cartesian_traje_emxFree_int32_T(&fb);
  cartesian_traje_emxFree_int32_T(&eb);
  cartesian_traje_emxFree_int32_T(&db);
  cartesian_traje_emxFree_int32_T(&cb);
  cartesian_trajec_emxFree_real_T(&L);
  cartesian_trajec_emxFree_real_T(&AIn);
  cartesian_trajec_emxFree_real_T(&alpha);
  cartesian_trajec_emxFree_real_T(&A);
  cartesian_tra_emxFree_boolean_T(&activeSet);
  cartesian_trajec_emxFree_real_T(&grad);
  cartesian_trajec_emxFree_real_T(&unusedU1);
}

static void cartesian_trajectory_p_isfinite(const
  emxArray_real_T_cartesian_tra_T *x, emxArray_boolean_T_cartesian__T *b)
{
  emxArray_boolean_T_cartesian__T *tmp;
  int32_T loop_ub;
  int32_T i;
  i = b->size[0];
  b->size[0] = x->size[0];
  car_emxEnsureCapacity_boolean_T(b, i);
  loop_ub = x->size[0];
  for (i = 0; i < loop_ub; i++) {
    b->data[i] = rtIsInf(x->data[i]);
  }

  cartesian_tra_emxInit_boolean_T(&tmp, 1);
  i = tmp->size[0];
  tmp->size[0] = x->size[0];
  car_emxEnsureCapacity_boolean_T(tmp, i);
  loop_ub = x->size[0];
  for (i = 0; i < loop_ub; i++) {
    tmp->data[i] = rtIsNaN(x->data[i]);
  }

  i = b->size[0];
  car_emxEnsureCapacity_boolean_T(b, i);
  loop_ub = b->size[0];
  for (i = 0; i < loop_ub; i++) {
    b->data[i] = ((!b->data[i]) && (!tmp->data[i]));
  }

  cartesian_tra_emxFree_boolean_T(&tmp);
}

static void cartesi_genrand_uint32_vector_a(uint32_T mt[625], uint32_T u[2])
{
  for (cartesian_trajectory_planner_B.b_j_m = 0;
       cartesian_trajectory_planner_B.b_j_m < 2;
       cartesian_trajectory_planner_B.b_j_m++) {
    cartesian_trajectory_planner_B.mti = mt[624] + 1U;
    if (cartesian_trajectory_planner_B.mti >= 625U) {
      for (cartesian_trajectory_planner_B.b_kk = 0;
           cartesian_trajectory_planner_B.b_kk < 227;
           cartesian_trajectory_planner_B.b_kk++) {
        cartesian_trajectory_planner_B.y_in =
          (mt[cartesian_trajectory_planner_B.b_kk + 1] & 2147483647U) |
          (mt[cartesian_trajectory_planner_B.b_kk] & 2147483648U);
        if ((cartesian_trajectory_planner_B.y_in & 1U) == 0U) {
          cartesian_trajectory_planner_B.y_in >>= 1U;
        } else {
          cartesian_trajectory_planner_B.y_in =
            cartesian_trajectory_planner_B.y_in >> 1U ^ 2567483615U;
        }

        mt[cartesian_trajectory_planner_B.b_kk] =
          mt[cartesian_trajectory_planner_B.b_kk + 397] ^
          cartesian_trajectory_planner_B.y_in;
      }

      for (cartesian_trajectory_planner_B.b_kk = 0;
           cartesian_trajectory_planner_B.b_kk < 396;
           cartesian_trajectory_planner_B.b_kk++) {
        cartesian_trajectory_planner_B.y_in =
          (mt[cartesian_trajectory_planner_B.b_kk + 227] & 2147483648U) |
          (mt[cartesian_trajectory_planner_B.b_kk + 228] & 2147483647U);
        if ((cartesian_trajectory_planner_B.y_in & 1U) == 0U) {
          cartesian_trajectory_planner_B.y_in >>= 1U;
        } else {
          cartesian_trajectory_planner_B.y_in =
            cartesian_trajectory_planner_B.y_in >> 1U ^ 2567483615U;
        }

        mt[cartesian_trajectory_planner_B.b_kk + 227] =
          mt[cartesian_trajectory_planner_B.b_kk] ^
          cartesian_trajectory_planner_B.y_in;
      }

      cartesian_trajectory_planner_B.y_in = (mt[623] & 2147483648U) | (mt[0] &
        2147483647U);
      if ((cartesian_trajectory_planner_B.y_in & 1U) == 0U) {
        cartesian_trajectory_planner_B.y_in >>= 1U;
      } else {
        cartesian_trajectory_planner_B.y_in =
          cartesian_trajectory_planner_B.y_in >> 1U ^ 2567483615U;
      }

      mt[623] = mt[396] ^ cartesian_trajectory_planner_B.y_in;
      cartesian_trajectory_planner_B.mti = 1U;
    }

    cartesian_trajectory_planner_B.y_in = mt[static_cast<int32_T>
      (cartesian_trajectory_planner_B.mti) - 1];
    mt[624] = cartesian_trajectory_planner_B.mti;
    cartesian_trajectory_planner_B.y_in ^= cartesian_trajectory_planner_B.y_in >>
      11U;
    cartesian_trajectory_planner_B.y_in ^= cartesian_trajectory_planner_B.y_in <<
      7U & 2636928640U;
    cartesian_trajectory_planner_B.y_in ^= cartesian_trajectory_planner_B.y_in <<
      15U & 4022730752U;
    u[cartesian_trajectory_planner_B.b_j_m] =
      cartesian_trajectory_planner_B.y_in >> 18U ^
      cartesian_trajectory_planner_B.y_in;
  }
}

static boolean_T cartesian_trajec_is_valid_state(const uint32_T mt[625])
{
  boolean_T isvalid;
  boolean_T exitg1;
  if ((mt[624] >= 1U) && (mt[624] < 625U)) {
    isvalid = true;
  } else {
    isvalid = false;
  }

  if (isvalid) {
    isvalid = false;
    cartesian_trajectory_planner_B.k_a = 0;
    exitg1 = false;
    while ((!exitg1) && (cartesian_trajectory_planner_B.k_a + 1 < 625)) {
      if (mt[cartesian_trajectory_planner_B.k_a] == 0U) {
        cartesian_trajectory_planner_B.k_a++;
      } else {
        isvalid = true;
        exitg1 = true;
      }
    }
  }

  return isvalid;
}

static real_T cartesian_trajectory_genrandu_a(uint32_T mt[625])
{
  real_T r;
  int32_T exitg1;

  // ========================= COPYRIGHT NOTICE ============================
  //  This is a uniform (0,1) pseudorandom number generator based on:
  //
  //  A C-program for MT19937, with initialization improved 2002/1/26.
  //  Coded by Takuji Nishimura and Makoto Matsumoto.
  //
  //  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,
  //  All rights reserved.
  //
  //  Redistribution and use in source and binary forms, with or without
  //  modification, are permitted provided that the following conditions
  //  are met:
  //
  //    1. Redistributions of source code must retain the above copyright
  //       notice, this list of conditions and the following disclaimer.
  //
  //    2. Redistributions in binary form must reproduce the above copyright
  //       notice, this list of conditions and the following disclaimer
  //       in the documentation and/or other materials provided with the
  //       distribution.
  //
  //    3. The names of its contributors may not be used to endorse or
  //       promote products derived from this software without specific
  //       prior written permission.
  //
  //  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  //  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  //  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  //  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
  //  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  //  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  //  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  //  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  //  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  //  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  //  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  //
  // =============================   END   =================================
  do {
    exitg1 = 0;
    cartesi_genrand_uint32_vector_a(mt, cartesian_trajectory_planner_B.b_u_b);
    r = (static_cast<real_T>(cartesian_trajectory_planner_B.b_u_b[0] >> 5U) *
         6.7108864E+7 + static_cast<real_T>
         (cartesian_trajectory_planner_B.b_u_b[1] >> 6U)) *
      1.1102230246251565E-16;
    if (r == 0.0) {
      if (!cartesian_trajec_is_valid_state(mt)) {
        cartesian_trajectory_planner_B.r_a = 5489U;
        mt[0] = 5489U;
        for (cartesian_trajectory_planner_B.b_mti_m = 0;
             cartesian_trajectory_planner_B.b_mti_m < 623;
             cartesian_trajectory_planner_B.b_mti_m++) {
          cartesian_trajectory_planner_B.r_a =
            ((cartesian_trajectory_planner_B.r_a >> 30U ^
              cartesian_trajectory_planner_B.r_a) * 1812433253U +
             cartesian_trajectory_planner_B.b_mti_m) + 1U;
          mt[cartesian_trajectory_planner_B.b_mti_m + 1] =
            cartesian_trajectory_planner_B.r_a;
        }

        mt[624] = 624U;
      }
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return r;
}

static real_T cartesia_eml_rand_mt19937ar_ast(uint32_T state[625])
{
  real_T r;
  static const real_T tmp[257] = { 1.0, 0.977101701267673, 0.959879091800108,
    0.9451989534423, 0.932060075959231, 0.919991505039348, 0.908726440052131,
    0.898095921898344, 0.887984660755834, 0.878309655808918, 0.869008688036857,
    0.860033621196332, 0.851346258458678, 0.842915653112205, 0.834716292986884,
    0.826726833946222, 0.818929191603703, 0.811307874312656, 0.803849483170964,
    0.796542330422959, 0.789376143566025, 0.782341832654803, 0.775431304981187,
    0.768637315798486, 0.761953346836795, 0.755373506507096, 0.748892447219157,
    0.742505296340151, 0.736207598126863, 0.729995264561476, 0.72386453346863,
    0.717811932630722, 0.711834248878248, 0.705928501332754, 0.700091918136512,
    0.694321916126117, 0.688616083004672, 0.682972161644995, 0.677388036218774,
    0.671861719897082, 0.66639134390875, 0.660975147776663, 0.655611470579697,
    0.650298743110817, 0.645035480820822, 0.639820277453057, 0.634651799287624,
    0.629528779924837, 0.624450015547027, 0.619414360605834, 0.614420723888914,
    0.609468064925773, 0.604555390697468, 0.599681752619125, 0.594846243767987,
    0.590047996332826, 0.585286179263371, 0.580559996100791, 0.575868682972354,
    0.571211506735253, 0.566587763256165, 0.561996775814525, 0.557437893618766,
    0.552910490425833, 0.548413963255266, 0.543947731190026, 0.539511234256952,
    0.535103932380458, 0.530725304403662, 0.526374847171684, 0.522052074672322,
    0.517756517229756, 0.513487720747327, 0.509245245995748, 0.505028667943468,
    0.500837575126149, 0.49667156905249, 0.492530263643869, 0.488413284705458,
    0.484320269426683, 0.480250865909047, 0.476204732719506, 0.47218153846773,
    0.468180961405694, 0.464202689048174, 0.460246417812843, 0.456311852678716,
    0.452398706861849, 0.448506701507203, 0.444635565395739, 0.440785034665804,
    0.436954852547985, 0.433144769112652, 0.429354541029442, 0.425583931338022,
    0.421832709229496, 0.418100649837848, 0.414387534040891, 0.410693148270188,
    0.407017284329473, 0.403359739221114, 0.399720314980197, 0.396098818515832,
    0.392495061459315, 0.388908860018789, 0.385340034840077, 0.381788410873393,
    0.378253817245619, 0.374736087137891, 0.371235057668239, 0.367750569779032,
    0.364282468129004, 0.360830600989648, 0.357394820145781, 0.353974980800077,
    0.350570941481406, 0.347182563956794, 0.343809713146851, 0.340452257044522,
    0.337110066637006, 0.333783015830718, 0.330470981379163, 0.327173842813601,
    0.323891482376391, 0.320623784956905, 0.317370638029914, 0.314131931596337,
    0.310907558126286, 0.307697412504292, 0.30450139197665, 0.301319396100803,
    0.298151326696685, 0.294997087799962, 0.291856585617095, 0.288729728482183,
    0.285616426815502, 0.282516593083708, 0.279430141761638, 0.276356989295668,
    0.273297054068577, 0.270250256365875, 0.267216518343561, 0.264195763997261,
    0.261187919132721, 0.258192911337619, 0.255210669954662, 0.252241126055942,
    0.249284212418529, 0.246339863501264, 0.24340801542275, 0.240488605940501,
    0.237581574431238, 0.23468686187233, 0.231804410824339, 0.228934165414681,
    0.226076071322381, 0.223230075763918, 0.220396127480152, 0.217574176724331,
    0.214764175251174, 0.211966076307031, 0.209179834621125, 0.206405406397881,
    0.203642749310335, 0.200891822494657, 0.198152586545776, 0.195425003514135,
    0.192709036903589, 0.190004651670465, 0.187311814223801, 0.1846304924268,
    0.181960655599523, 0.179302274522848, 0.176655321443735, 0.174019770081839,
    0.171395595637506, 0.168782774801212, 0.166181285764482, 0.163591108232366,
    0.161012223437511, 0.158444614155925, 0.15588826472448, 0.153343161060263,
    0.150809290681846, 0.148286642732575, 0.145775208005994, 0.143274978973514,
    0.140785949814445, 0.138308116448551, 0.135841476571254, 0.133386029691669,
    0.130941777173644, 0.12850872228, 0.126086870220186, 0.123676228201597,
    0.12127680548479, 0.11888861344291, 0.116511665625611, 0.114145977827839,
    0.111791568163838, 0.109448457146812, 0.107116667774684, 0.104796225622487,
    0.102487158941935, 0.10018949876881, 0.0979032790388625, 0.095628536713009,
    0.093365311912691, 0.0911136480663738, 0.0888735920682759,
    0.0866451944505581, 0.0844285095703535, 0.082223595813203,
    0.0800305158146631, 0.0778493367020961, 0.0756801303589272,
    0.0735229737139814, 0.0713779490588905, 0.0692451443970068,
    0.0671246538277886, 0.065016577971243, 0.0629210244377582, 0.06083810834954,
    0.0587679529209339, 0.0567106901062031, 0.0546664613248891,
    0.0526354182767924, 0.0506177238609479, 0.0486135532158687,
    0.0466230949019305, 0.0446465522512946, 0.0426841449164746,
    0.0407361106559411, 0.0388027074045262, 0.0368842156885674,
    0.0349809414617162, 0.0330932194585786, 0.0312214171919203,
    0.0293659397581334, 0.0275272356696031, 0.0257058040085489,
    0.0239022033057959, 0.0221170627073089, 0.0203510962300445,
    0.0186051212757247, 0.0168800831525432, 0.0151770883079353,
    0.0134974506017399, 0.0118427578579079, 0.0102149714397015,
    0.00861658276939875, 0.00705087547137324, 0.00552240329925101,
    0.00403797259336304, 0.00260907274610216, 0.0012602859304986,
    0.000477467764609386 };

  const real_T *fitab;
  int32_T exitg1;
  cartesian_trajectory_planner_B.xi[0] = 0.0;
  cartesian_trajectory_planner_B.xi[1] = 0.215241895984875;
  cartesian_trajectory_planner_B.xi[2] = 0.286174591792068;
  cartesian_trajectory_planner_B.xi[3] = 0.335737519214422;
  cartesian_trajectory_planner_B.xi[4] = 0.375121332878378;
  cartesian_trajectory_planner_B.xi[5] = 0.408389134611989;
  cartesian_trajectory_planner_B.xi[6] = 0.43751840220787;
  cartesian_trajectory_planner_B.xi[7] = 0.46363433679088;
  cartesian_trajectory_planner_B.xi[8] = 0.487443966139235;
  cartesian_trajectory_planner_B.xi[9] = 0.50942332960209;
  cartesian_trajectory_planner_B.xi[10] = 0.529909720661557;
  cartesian_trajectory_planner_B.xi[11] = 0.549151702327164;
  cartesian_trajectory_planner_B.xi[12] = 0.567338257053817;
  cartesian_trajectory_planner_B.xi[13] = 0.584616766106378;
  cartesian_trajectory_planner_B.xi[14] = 0.601104617755991;
  cartesian_trajectory_planner_B.xi[15] = 0.61689699000775;
  cartesian_trajectory_planner_B.xi[16] = 0.63207223638606;
  cartesian_trajectory_planner_B.xi[17] = 0.646695714894993;
  cartesian_trajectory_planner_B.xi[18] = 0.660822574244419;
  cartesian_trajectory_planner_B.xi[19] = 0.674499822837293;
  cartesian_trajectory_planner_B.xi[20] = 0.687767892795788;
  cartesian_trajectory_planner_B.xi[21] = 0.700661841106814;
  cartesian_trajectory_planner_B.xi[22] = 0.713212285190975;
  cartesian_trajectory_planner_B.xi[23] = 0.725446140909999;
  cartesian_trajectory_planner_B.xi[24] = 0.737387211434295;
  cartesian_trajectory_planner_B.xi[25] = 0.749056662017815;
  cartesian_trajectory_planner_B.xi[26] = 0.760473406430107;
  cartesian_trajectory_planner_B.xi[27] = 0.771654424224568;
  cartesian_trajectory_planner_B.xi[28] = 0.782615023307232;
  cartesian_trajectory_planner_B.xi[29] = 0.793369058840623;
  cartesian_trajectory_planner_B.xi[30] = 0.80392911698997;
  cartesian_trajectory_planner_B.xi[31] = 0.814306670135215;
  cartesian_trajectory_planner_B.xi[32] = 0.824512208752291;
  cartesian_trajectory_planner_B.xi[33] = 0.834555354086381;
  cartesian_trajectory_planner_B.xi[34] = 0.844444954909153;
  cartesian_trajectory_planner_B.xi[35] = 0.854189171008163;
  cartesian_trajectory_planner_B.xi[36] = 0.863795545553308;
  cartesian_trajectory_planner_B.xi[37] = 0.87327106808886;
  cartesian_trajectory_planner_B.xi[38] = 0.882622229585165;
  cartesian_trajectory_planner_B.xi[39] = 0.891855070732941;
  cartesian_trajectory_planner_B.xi[40] = 0.900975224461221;
  cartesian_trajectory_planner_B.xi[41] = 0.909987953496718;
  cartesian_trajectory_planner_B.xi[42] = 0.91889818364959;
  cartesian_trajectory_planner_B.xi[43] = 0.927710533401999;
  cartesian_trajectory_planner_B.xi[44] = 0.936429340286575;
  cartesian_trajectory_planner_B.xi[45] = 0.945058684468165;
  cartesian_trajectory_planner_B.xi[46] = 0.953602409881086;
  cartesian_trajectory_planner_B.xi[47] = 0.96206414322304;
  cartesian_trajectory_planner_B.xi[48] = 0.970447311064224;
  cartesian_trajectory_planner_B.xi[49] = 0.978755155294224;
  cartesian_trajectory_planner_B.xi[50] = 0.986990747099062;
  cartesian_trajectory_planner_B.xi[51] = 0.99515699963509;
  cartesian_trajectory_planner_B.xi[52] = 1.00325667954467;
  cartesian_trajectory_planner_B.xi[53] = 1.01129241744;
  cartesian_trajectory_planner_B.xi[54] = 1.01926671746548;
  cartesian_trajectory_planner_B.xi[55] = 1.02718196603564;
  cartesian_trajectory_planner_B.xi[56] = 1.03504043983344;
  cartesian_trajectory_planner_B.xi[57] = 1.04284431314415;
  cartesian_trajectory_planner_B.xi[58] = 1.05059566459093;
  cartesian_trajectory_planner_B.xi[59] = 1.05829648333067;
  cartesian_trajectory_planner_B.xi[60] = 1.06594867476212;
  cartesian_trajectory_planner_B.xi[61] = 1.07355406579244;
  cartesian_trajectory_planner_B.xi[62] = 1.0811144097034;
  cartesian_trajectory_planner_B.xi[63] = 1.08863139065398;
  cartesian_trajectory_planner_B.xi[64] = 1.09610662785202;
  cartesian_trajectory_planner_B.xi[65] = 1.10354167942464;
  cartesian_trajectory_planner_B.xi[66] = 1.11093804601357;
  cartesian_trajectory_planner_B.xi[67] = 1.11829717411934;
  cartesian_trajectory_planner_B.xi[68] = 1.12562045921553;
  cartesian_trajectory_planner_B.xi[69] = 1.13290924865253;
  cartesian_trajectory_planner_B.xi[70] = 1.14016484436815;
  cartesian_trajectory_planner_B.xi[71] = 1.14738850542085;
  cartesian_trajectory_planner_B.xi[72] = 1.15458145035993;
  cartesian_trajectory_planner_B.xi[73] = 1.16174485944561;
  cartesian_trajectory_planner_B.xi[74] = 1.16887987673083;
  cartesian_trajectory_planner_B.xi[75] = 1.17598761201545;
  cartesian_trajectory_planner_B.xi[76] = 1.18306914268269;
  cartesian_trajectory_planner_B.xi[77] = 1.19012551542669;
  cartesian_trajectory_planner_B.xi[78] = 1.19715774787944;
  cartesian_trajectory_planner_B.xi[79] = 1.20416683014438;
  cartesian_trajectory_planner_B.xi[80] = 1.2111537262437;
  cartesian_trajectory_planner_B.xi[81] = 1.21811937548548;
  cartesian_trajectory_planner_B.xi[82] = 1.22506469375653;
  cartesian_trajectory_planner_B.xi[83] = 1.23199057474614;
  cartesian_trajectory_planner_B.xi[84] = 1.23889789110569;
  cartesian_trajectory_planner_B.xi[85] = 1.24578749554863;
  cartesian_trajectory_planner_B.xi[86] = 1.2526602218949;
  cartesian_trajectory_planner_B.xi[87] = 1.25951688606371;
  cartesian_trajectory_planner_B.xi[88] = 1.26635828701823;
  cartesian_trajectory_planner_B.xi[89] = 1.27318520766536;
  cartesian_trajectory_planner_B.xi[90] = 1.27999841571382;
  cartesian_trajectory_planner_B.xi[91] = 1.28679866449324;
  cartesian_trajectory_planner_B.xi[92] = 1.29358669373695;
  cartesian_trajectory_planner_B.xi[93] = 1.30036323033084;
  cartesian_trajectory_planner_B.xi[94] = 1.30712898903073;
  cartesian_trajectory_planner_B.xi[95] = 1.31388467315022;
  cartesian_trajectory_planner_B.xi[96] = 1.32063097522106;
  cartesian_trajectory_planner_B.xi[97] = 1.32736857762793;
  cartesian_trajectory_planner_B.xi[98] = 1.33409815321936;
  cartesian_trajectory_planner_B.xi[99] = 1.3408203658964;
  cartesian_trajectory_planner_B.xi[100] = 1.34753587118059;
  cartesian_trajectory_planner_B.xi[101] = 1.35424531676263;
  cartesian_trajectory_planner_B.xi[102] = 1.36094934303328;
  cartesian_trajectory_planner_B.xi[103] = 1.36764858359748;
  cartesian_trajectory_planner_B.xi[104] = 1.37434366577317;
  cartesian_trajectory_planner_B.xi[105] = 1.38103521107586;
  cartesian_trajectory_planner_B.xi[106] = 1.38772383568998;
  cartesian_trajectory_planner_B.xi[107] = 1.39441015092814;
  cartesian_trajectory_planner_B.xi[108] = 1.40109476367925;
  cartesian_trajectory_planner_B.xi[109] = 1.4077782768464;
  cartesian_trajectory_planner_B.xi[110] = 1.41446128977547;
  cartesian_trajectory_planner_B.xi[111] = 1.42114439867531;
  cartesian_trajectory_planner_B.xi[112] = 1.42782819703026;
  cartesian_trajectory_planner_B.xi[113] = 1.43451327600589;
  cartesian_trajectory_planner_B.xi[114] = 1.44120022484872;
  cartesian_trajectory_planner_B.xi[115] = 1.44788963128058;
  cartesian_trajectory_planner_B.xi[116] = 1.45458208188841;
  cartesian_trajectory_planner_B.xi[117] = 1.46127816251028;
  cartesian_trajectory_planner_B.xi[118] = 1.46797845861808;
  cartesian_trajectory_planner_B.xi[119] = 1.47468355569786;
  cartesian_trajectory_planner_B.xi[120] = 1.48139403962819;
  cartesian_trajectory_planner_B.xi[121] = 1.48811049705745;
  cartesian_trajectory_planner_B.xi[122] = 1.49483351578049;
  cartesian_trajectory_planner_B.xi[123] = 1.50156368511546;
  cartesian_trajectory_planner_B.xi[124] = 1.50830159628131;
  cartesian_trajectory_planner_B.xi[125] = 1.51504784277671;
  cartesian_trajectory_planner_B.xi[126] = 1.521803020761;
  cartesian_trajectory_planner_B.xi[127] = 1.52856772943771;
  cartesian_trajectory_planner_B.xi[128] = 1.53534257144151;
  cartesian_trajectory_planner_B.xi[129] = 1.542128153229;
  cartesian_trajectory_planner_B.xi[130] = 1.54892508547417;
  cartesian_trajectory_planner_B.xi[131] = 1.55573398346918;
  cartesian_trajectory_planner_B.xi[132] = 1.56255546753104;
  cartesian_trajectory_planner_B.xi[133] = 1.56939016341512;
  cartesian_trajectory_planner_B.xi[134] = 1.57623870273591;
  cartesian_trajectory_planner_B.xi[135] = 1.58310172339603;
  cartesian_trajectory_planner_B.xi[136] = 1.58997987002419;
  cartesian_trajectory_planner_B.xi[137] = 1.59687379442279;
  cartesian_trajectory_planner_B.xi[138] = 1.60378415602609;
  cartesian_trajectory_planner_B.xi[139] = 1.61071162236983;
  cartesian_trajectory_planner_B.xi[140] = 1.61765686957301;
  cartesian_trajectory_planner_B.xi[141] = 1.62462058283303;
  cartesian_trajectory_planner_B.xi[142] = 1.63160345693487;
  cartesian_trajectory_planner_B.xi[143] = 1.63860619677555;
  cartesian_trajectory_planner_B.xi[144] = 1.64562951790478;
  cartesian_trajectory_planner_B.xi[145] = 1.65267414708306;
  cartesian_trajectory_planner_B.xi[146] = 1.65974082285818;
  cartesian_trajectory_planner_B.xi[147] = 1.66683029616166;
  cartesian_trajectory_planner_B.xi[148] = 1.67394333092612;
  cartesian_trajectory_planner_B.xi[149] = 1.68108070472517;
  cartesian_trajectory_planner_B.xi[150] = 1.68824320943719;
  cartesian_trajectory_planner_B.xi[151] = 1.69543165193456;
  cartesian_trajectory_planner_B.xi[152] = 1.70264685479992;
  cartesian_trajectory_planner_B.xi[153] = 1.7098896570713;
  cartesian_trajectory_planner_B.xi[154] = 1.71716091501782;
  cartesian_trajectory_planner_B.xi[155] = 1.72446150294804;
  cartesian_trajectory_planner_B.xi[156] = 1.73179231405296;
  cartesian_trajectory_planner_B.xi[157] = 1.73915426128591;
  cartesian_trajectory_planner_B.xi[158] = 1.74654827828172;
  cartesian_trajectory_planner_B.xi[159] = 1.75397532031767;
  cartesian_trajectory_planner_B.xi[160] = 1.76143636531891;
  cartesian_trajectory_planner_B.xi[161] = 1.76893241491127;
  cartesian_trajectory_planner_B.xi[162] = 1.77646449552452;
  cartesian_trajectory_planner_B.xi[163] = 1.78403365954944;
  cartesian_trajectory_planner_B.xi[164] = 1.79164098655216;
  cartesian_trajectory_planner_B.xi[165] = 1.79928758454972;
  cartesian_trajectory_planner_B.xi[166] = 1.80697459135082;
  cartesian_trajectory_planner_B.xi[167] = 1.81470317596628;
  cartesian_trajectory_planner_B.xi[168] = 1.82247454009388;
  cartesian_trajectory_planner_B.xi[169] = 1.83028991968276;
  cartesian_trajectory_planner_B.xi[170] = 1.83815058658281;
  cartesian_trajectory_planner_B.xi[171] = 1.84605785028518;
  cartesian_trajectory_planner_B.xi[172] = 1.8540130597602;
  cartesian_trajectory_planner_B.xi[173] = 1.86201760539967;
  cartesian_trajectory_planner_B.xi[174] = 1.87007292107127;
  cartesian_trajectory_planner_B.xi[175] = 1.878180486293;
  cartesian_trajectory_planner_B.xi[176] = 1.88634182853678;
  cartesian_trajectory_planner_B.xi[177] = 1.8945585256707;
  cartesian_trajectory_planner_B.xi[178] = 1.90283220855043;
  cartesian_trajectory_planner_B.xi[179] = 1.91116456377125;
  cartesian_trajectory_planner_B.xi[180] = 1.91955733659319;
  cartesian_trajectory_planner_B.xi[181] = 1.92801233405266;
  cartesian_trajectory_planner_B.xi[182] = 1.93653142827569;
  cartesian_trajectory_planner_B.xi[183] = 1.94511656000868;
  cartesian_trajectory_planner_B.xi[184] = 1.95376974238465;
  cartesian_trajectory_planner_B.xi[185] = 1.96249306494436;
  cartesian_trajectory_planner_B.xi[186] = 1.97128869793366;
  cartesian_trajectory_planner_B.xi[187] = 1.98015889690048;
  cartesian_trajectory_planner_B.xi[188] = 1.98910600761744;
  cartesian_trajectory_planner_B.xi[189] = 1.99813247135842;
  cartesian_trajectory_planner_B.xi[190] = 2.00724083056053;
  cartesian_trajectory_planner_B.xi[191] = 2.0164337349062;
  cartesian_trajectory_planner_B.xi[192] = 2.02571394786385;
  cartesian_trajectory_planner_B.xi[193] = 2.03508435372962;
  cartesian_trajectory_planner_B.xi[194] = 2.04454796521753;
  cartesian_trajectory_planner_B.xi[195] = 2.05410793165065;
  cartesian_trajectory_planner_B.xi[196] = 2.06376754781173;
  cartesian_trajectory_planner_B.xi[197] = 2.07353026351874;
  cartesian_trajectory_planner_B.xi[198] = 2.0833996939983;
  cartesian_trajectory_planner_B.xi[199] = 2.09337963113879;
  cartesian_trajectory_planner_B.xi[200] = 2.10347405571488;
  cartesian_trajectory_planner_B.xi[201] = 2.11368715068665;
  cartesian_trajectory_planner_B.xi[202] = 2.12402331568952;
  cartesian_trajectory_planner_B.xi[203] = 2.13448718284602;
  cartesian_trajectory_planner_B.xi[204] = 2.14508363404789;
  cartesian_trajectory_planner_B.xi[205] = 2.15581781987674;
  cartesian_trajectory_planner_B.xi[206] = 2.16669518035431;
  cartesian_trajectory_planner_B.xi[207] = 2.17772146774029;
  cartesian_trajectory_planner_B.xi[208] = 2.18890277162636;
  cartesian_trajectory_planner_B.xi[209] = 2.20024554661128;
  cartesian_trajectory_planner_B.xi[210] = 2.21175664288416;
  cartesian_trajectory_planner_B.xi[211] = 2.22344334009251;
  cartesian_trajectory_planner_B.xi[212] = 2.23531338492992;
  cartesian_trajectory_planner_B.xi[213] = 2.24737503294739;
  cartesian_trajectory_planner_B.xi[214] = 2.25963709517379;
  cartesian_trajectory_planner_B.xi[215] = 2.27210899022838;
  cartesian_trajectory_planner_B.xi[216] = 2.28480080272449;
  cartesian_trajectory_planner_B.xi[217] = 2.29772334890286;
  cartesian_trajectory_planner_B.xi[218] = 2.31088825060137;
  cartesian_trajectory_planner_B.xi[219] = 2.32430801887113;
  cartesian_trajectory_planner_B.xi[220] = 2.33799614879653;
  cartesian_trajectory_planner_B.xi[221] = 2.35196722737914;
  cartesian_trajectory_planner_B.xi[222] = 2.36623705671729;
  cartesian_trajectory_planner_B.xi[223] = 2.38082279517208;
  cartesian_trajectory_planner_B.xi[224] = 2.39574311978193;
  cartesian_trajectory_planner_B.xi[225] = 2.41101841390112;
  cartesian_trajectory_planner_B.xi[226] = 2.42667098493715;
  cartesian_trajectory_planner_B.xi[227] = 2.44272531820036;
  cartesian_trajectory_planner_B.xi[228] = 2.4592083743347;
  cartesian_trajectory_planner_B.xi[229] = 2.47614993967052;
  cartesian_trajectory_planner_B.xi[230] = 2.49358304127105;
  cartesian_trajectory_planner_B.xi[231] = 2.51154444162669;
  cartesian_trajectory_planner_B.xi[232] = 2.53007523215985;
  cartesian_trajectory_planner_B.xi[233] = 2.54922155032478;
  cartesian_trajectory_planner_B.xi[234] = 2.56903545268184;
  cartesian_trajectory_planner_B.xi[235] = 2.58957598670829;
  cartesian_trajectory_planner_B.xi[236] = 2.61091051848882;
  cartesian_trajectory_planner_B.xi[237] = 2.63311639363158;
  cartesian_trajectory_planner_B.xi[238] = 2.65628303757674;
  cartesian_trajectory_planner_B.xi[239] = 2.68051464328574;
  cartesian_trajectory_planner_B.xi[240] = 2.70593365612306;
  cartesian_trajectory_planner_B.xi[241] = 2.73268535904401;
  cartesian_trajectory_planner_B.xi[242] = 2.76094400527999;
  cartesian_trajectory_planner_B.xi[243] = 2.79092117400193;
  cartesian_trajectory_planner_B.xi[244] = 2.82287739682644;
  cartesian_trajectory_planner_B.xi[245] = 2.85713873087322;
  cartesian_trajectory_planner_B.xi[246] = 2.89412105361341;
  cartesian_trajectory_planner_B.xi[247] = 2.93436686720889;
  cartesian_trajectory_planner_B.xi[248] = 2.97860327988184;
  cartesian_trajectory_planner_B.xi[249] = 3.02783779176959;
  cartesian_trajectory_planner_B.xi[250] = 3.08352613200214;
  cartesian_trajectory_planner_B.xi[251] = 3.147889289518;
  cartesian_trajectory_planner_B.xi[252] = 3.2245750520478;
  cartesian_trajectory_planner_B.xi[253] = 3.32024473383983;
  cartesian_trajectory_planner_B.xi[254] = 3.44927829856143;
  cartesian_trajectory_planner_B.xi[255] = 3.65415288536101;
  cartesian_trajectory_planner_B.xi[256] = 3.91075795952492;
  fitab = &tmp[0];
  do {
    exitg1 = 0;
    cartesi_genrand_uint32_vector_a(state, cartesian_trajectory_planner_B.u32);
    cartesian_trajectory_planner_B.i_f = static_cast<int32_T>
      ((cartesian_trajectory_planner_B.u32[1] >> 24U) + 1U);
    r = ((static_cast<real_T>(cartesian_trajectory_planner_B.u32[0] >> 3U) *
          1.6777216E+7 + static_cast<real_T>(static_cast<int32_T>
           (cartesian_trajectory_planner_B.u32[1]) & 16777215)) *
         2.2204460492503131E-16 - 1.0) *
      cartesian_trajectory_planner_B.xi[cartesian_trajectory_planner_B.i_f];
    if (fabs(r) <=
        cartesian_trajectory_planner_B.xi[cartesian_trajectory_planner_B.i_f - 1])
    {
      exitg1 = 1;
    } else if (cartesian_trajectory_planner_B.i_f < 256) {
      cartesian_trajectory_planner_B.x_o = cartesian_trajectory_genrandu_a(state);
      if ((fitab[cartesian_trajectory_planner_B.i_f - 1] -
           fitab[cartesian_trajectory_planner_B.i_f]) *
          cartesian_trajectory_planner_B.x_o +
          fitab[cartesian_trajectory_planner_B.i_f] < exp(-0.5 * r * r)) {
        exitg1 = 1;
      }
    } else {
      do {
        cartesian_trajectory_planner_B.x_o = cartesian_trajectory_genrandu_a
          (state);
        cartesian_trajectory_planner_B.x_o = log
          (cartesian_trajectory_planner_B.x_o) * 0.273661237329758;
        cartesian_trajectory_planner_B.d_u = cartesian_trajectory_genrandu_a
          (state);
      } while (!(-2.0 * log(cartesian_trajectory_planner_B.d_u) >
                 cartesian_trajectory_planner_B.x_o *
                 cartesian_trajectory_planner_B.x_o));

      if (r < 0.0) {
        r = cartesian_trajectory_planner_B.x_o - 3.65415288536101;
      } else {
        r = 3.65415288536101 - cartesian_trajectory_planner_B.x_o;
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return r;
}

static void cartesian_trajectory_plan_randn(const real_T varargin_1[2],
  emxArray_real_T_cartesian_tra_T *r)
{
  cartesian_trajectory_planner_B.b_k_o4 = r->size[0] * r->size[1];
  r->size[0] = static_cast<int32_T>(varargin_1[0]);
  r->size[1] = 1;
  cartes_emxEnsureCapacity_real_T(r, cartesian_trajectory_planner_B.b_k_o4);
  cartesian_trajectory_planner_B.d_j = r->size[0] - 1;
  for (cartesian_trajectory_planner_B.b_k_o4 = 0;
       cartesian_trajectory_planner_B.b_k_o4 <=
       cartesian_trajectory_planner_B.d_j; cartesian_trajectory_planner_B.b_k_o4
       ++) {
    r->data[cartesian_trajectory_planner_B.b_k_o4] =
      cartesia_eml_rand_mt19937ar_ast(cartesian_trajectory_planner_DW.state_m);
  }
}

static void cartesian__eml_rand_mt19937ar_a(const uint32_T state[625], uint32_T
  b_state[625], real_T *r)
{
  int32_T exitg1;
  memcpy(&b_state[0], &state[0], 625U * sizeof(uint32_T));

  // ========================= COPYRIGHT NOTICE ============================
  //  This is a uniform (0,1) pseudorandom number generator based on:
  //
  //  A C-program for MT19937, with initialization improved 2002/1/26.
  //  Coded by Takuji Nishimura and Makoto Matsumoto.
  //
  //  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,
  //  All rights reserved.
  //
  //  Redistribution and use in source and binary forms, with or without
  //  modification, are permitted provided that the following conditions
  //  are met:
  //
  //    1. Redistributions of source code must retain the above copyright
  //       notice, this list of conditions and the following disclaimer.
  //
  //    2. Redistributions in binary form must reproduce the above copyright
  //       notice, this list of conditions and the following disclaimer
  //       in the documentation and/or other materials provided with the
  //       distribution.
  //
  //    3. The names of its contributors may not be used to endorse or
  //       promote products derived from this software without specific
  //       prior written permission.
  //
  //  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  //  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  //  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  //  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
  //  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  //  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  //  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  //  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  //  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  //  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  //  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  //
  // =============================   END   =================================
  do {
    exitg1 = 0;
    cartesi_genrand_uint32_vector_a(b_state, cartesian_trajectory_planner_B.b_u);
    *r = (static_cast<real_T>(cartesian_trajectory_planner_B.b_u[0] >> 5U) *
          6.7108864E+7 + static_cast<real_T>(cartesian_trajectory_planner_B.b_u
           [1] >> 6U)) * 1.1102230246251565E-16;
    if (*r == 0.0) {
      if (!cartesian_trajec_is_valid_state(b_state)) {
        cartesian_trajectory_planner_B.r = 5489U;
        b_state[0] = 5489U;
        for (cartesian_trajectory_planner_B.b_mti = 0;
             cartesian_trajectory_planner_B.b_mti < 623;
             cartesian_trajectory_planner_B.b_mti++) {
          cartesian_trajectory_planner_B.r = ((cartesian_trajectory_planner_B.r >>
            30U ^ cartesian_trajectory_planner_B.r) * 1812433253U +
            cartesian_trajectory_planner_B.b_mti) + 1U;
          b_state[cartesian_trajectory_planner_B.b_mti + 1] =
            cartesian_trajectory_planner_B.r;
        }

        b_state[624] = 624U;
      }
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

static void cartesian_trajectory_pla_rand_a(real_T varargin_1,
  emxArray_real_T_cartesian_tra_T *r)
{
  cartesian_trajectory_planner_B.b_k_o = r->size[0];
  cartesian_trajectory_planner_B.d_f = static_cast<int32_T>(varargin_1);
  r->size[0] = cartesian_trajectory_planner_B.d_f;
  cartes_emxEnsureCapacity_real_T(r, cartesian_trajectory_planner_B.b_k_o);
  cartesian_trajectory_planner_B.d_f--;
  for (cartesian_trajectory_planner_B.b_k_o = 0;
       cartesian_trajectory_planner_B.b_k_o <=
       cartesian_trajectory_planner_B.d_f; cartesian_trajectory_planner_B.b_k_o
       ++) {
    memcpy(&cartesian_trajectory_planner_B.uv1[0],
           &cartesian_trajectory_planner_DW.state_m[0], 625U * sizeof(uint32_T));
    cartesian__eml_rand_mt19937ar_a(cartesian_trajectory_planner_B.uv1,
      cartesian_trajectory_planner_DW.state_m, &r->
      data[cartesian_trajectory_planner_B.b_k_o]);
  }
}

static void cartes_NLPSolverInterface_solve(h_robotics_core_internal_Damp_T *obj,
  const real_T seed[6], real_T xSol[6], real_T *solutionInfo_Iterations, real_T *
  solutionInfo_RRAttempts, real_T *solutionInfo_Error, real_T
  *solutionInfo_ExitFlag, char_T solutionInfo_Status_data[], int32_T
  solutionInfo_Status_size[2])
{
  emxArray_real_T_cartesian_tra_T *newseed;
  f_robotics_manip_internal_IKE_T *args;
  x_robotics_manip_internal_Rig_T *obj_0;
  emxArray_real_T_cartesian_tra_T *qi;
  c_rigidBodyJoint_cartesian_as_T *obj_1;
  emxArray_real_T_cartesian_tra_T *ub;
  emxArray_real_T_cartesian_tra_T *lb;
  emxArray_real_T_cartesian_tra_T *rn;
  emxArray_real_T_cartesian_tra_T *e;
  emxArray_boolean_T_cartesian__T *x;
  emxArray_boolean_T_cartesian__T *x_tmp;
  emxArray_boolean_T_cartesian__T *x_tmp_0;
  emxArray_boolean_T_cartesian__T *x_0;
  static const char_T tmp[14] = { 'b', 'e', 's', 't', ' ', 'a', 'v', 'a', 'i',
    'l', 'a', 'b', 'l', 'e' };

  static const char_T tmp_0[7] = { 's', 'u', 'c', 'c', 'e', 's', 's' };

  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  boolean_T exitg1;
  boolean_T exitg2;
  obj->MaxNumIterationInternal = obj->MaxNumIteration;
  obj->MaxTimeInternal = obj->MaxTime;
  for (cartesian_trajectory_planner_B.i_h = 0;
       cartesian_trajectory_planner_B.i_h < 6;
       cartesian_trajectory_planner_B.i_h++) {
    obj->SeedInternal[cartesian_trajectory_planner_B.i_h] =
      seed[cartesian_trajectory_planner_B.i_h];
  }

  cartesian_trajectory_planner_B.tol = obj->SolutionTolerance;
  obj->TimeObj.StartTime = ctimefun();
  DampedBFGSwGradientProjection_s(obj, xSol,
    &cartesian_trajectory_planner_B.exitFlag,
    &cartesian_trajectory_planner_B.err, &cartesian_trajectory_planner_B.iter);
  *solutionInfo_RRAttempts = 0.0;
  *solutionInfo_Iterations = cartesian_trajectory_planner_B.iter;
  *solutionInfo_Error = cartesian_trajectory_planner_B.err;
  cartesian_trajectory_planner_B.exitFlagPrev =
    cartesian_trajectory_planner_B.exitFlag;
  cartesian_trajec_emxInit_real_T(&newseed, 1);
  cartesian_trajec_emxInit_real_T(&qi, 2);
  cartesian_trajec_emxInit_real_T(&ub, 1);
  cartesian_trajec_emxInit_real_T(&lb, 1);
  cartesian_trajec_emxInit_real_T(&rn, 1);
  cartesian_trajec_emxInit_real_T(&e, 2);
  cartesian_tra_emxInit_boolean_T(&x, 1);
  cartesian_tra_emxInit_boolean_T(&x_tmp, 1);
  cartesian_tra_emxInit_boolean_T(&x_tmp_0, 1);
  cartesian_tra_emxInit_boolean_T(&x_0, 1);
  exitg1 = false;
  while ((!exitg1) && (obj->RandomRestart && (cartesian_trajectory_planner_B.err
           > cartesian_trajectory_planner_B.tol))) {
    obj->MaxNumIterationInternal -= cartesian_trajectory_planner_B.iter;
    cartesian_trajectory_planner_B.err = ctimefun();
    cartesian_trajectory_planner_B.err -= obj->TimeObj.StartTime;
    obj->MaxTimeInternal = obj->MaxTime - cartesian_trajectory_planner_B.err;
    if (obj->MaxNumIterationInternal <= 0.0) {
      cartesian_trajectory_planner_B.exitFlag = IterationLimitExceeded;
    }

    if ((cartesian_trajectory_planner_B.exitFlag == IterationLimitExceeded) ||
        (cartesian_trajectory_planner_B.exitFlag == TimeLimitExceeded)) {
      cartesian_trajectory_planner_B.exitFlagPrev =
        cartesian_trajectory_planner_B.exitFlag;
      exitg1 = true;
    } else {
      args = obj->ExtraArgs;
      obj_0 = args->Robot;
      cartesian_trajectory_planner_B.ix = newseed->size[0];
      newseed->size[0] = static_cast<int32_T>(obj_0->PositionNumber);
      cartes_emxEnsureCapacity_real_T(newseed, cartesian_trajectory_planner_B.ix);
      cartesian_trajectory_planner_B.nx = static_cast<int32_T>
        (obj_0->PositionNumber);
      for (cartesian_trajectory_planner_B.ix = 0;
           cartesian_trajectory_planner_B.ix < cartesian_trajectory_planner_B.nx;
           cartesian_trajectory_planner_B.ix++) {
        newseed->data[cartesian_trajectory_planner_B.ix] = 0.0;
      }

      cartesian_trajectory_planner_B.err = obj_0->NumBodies;
      cartesian_trajectory_planner_B.c_n = static_cast<int32_T>
        (cartesian_trajectory_planner_B.err) - 1;
      for (cartesian_trajectory_planner_B.i_h = 0;
           cartesian_trajectory_planner_B.i_h <=
           cartesian_trajectory_planner_B.c_n;
           cartesian_trajectory_planner_B.i_h++) {
        cartesian_trajectory_planner_B.err = obj_0->
          PositionDoFMap[cartesian_trajectory_planner_B.i_h];
        cartesian_trajectory_planner_B.iter = obj_0->
          PositionDoFMap[cartesian_trajectory_planner_B.i_h + 8];
        if (cartesian_trajectory_planner_B.err <=
            cartesian_trajectory_planner_B.iter) {
          obj_1 = obj_0->Bodies[cartesian_trajectory_planner_B.i_h]
            ->JointInternal;
          if (static_cast<int32_T>(obj_1->PositionNumber) == 0) {
            cartesian_trajectory_planner_B.ix = qi->size[0] * qi->size[1];
            qi->size[0] = 1;
            qi->size[1] = 1;
            cartes_emxEnsureCapacity_real_T(qi,
              cartesian_trajectory_planner_B.ix);
            qi->data[0] = (rtNaN);
          } else {
            cartesian_trajectory_planner_B.nx = obj_1->
              PositionLimitsInternal->size[0];
            cartesian_trajectory_planner_B.ix = ub->size[0];
            ub->size[0] = cartesian_trajectory_planner_B.nx;
            cartes_emxEnsureCapacity_real_T(ub,
              cartesian_trajectory_planner_B.ix);
            for (cartesian_trajectory_planner_B.ix = 0;
                 cartesian_trajectory_planner_B.ix <
                 cartesian_trajectory_planner_B.nx;
                 cartesian_trajectory_planner_B.ix++) {
              ub->data[cartesian_trajectory_planner_B.ix] =
                obj_1->PositionLimitsInternal->
                data[cartesian_trajectory_planner_B.ix +
                obj_1->PositionLimitsInternal->size[0]];
            }

            cartesian_trajectory_planner_B.nx = obj_1->
              PositionLimitsInternal->size[0];
            cartesian_trajectory_planner_B.ix = lb->size[0];
            lb->size[0] = cartesian_trajectory_planner_B.nx;
            cartes_emxEnsureCapacity_real_T(lb,
              cartesian_trajectory_planner_B.ix);
            for (cartesian_trajectory_planner_B.ix = 0;
                 cartesian_trajectory_planner_B.ix <
                 cartesian_trajectory_planner_B.nx;
                 cartesian_trajectory_planner_B.ix++) {
              lb->data[cartesian_trajectory_planner_B.ix] =
                obj_1->PositionLimitsInternal->
                data[cartesian_trajectory_planner_B.ix];
            }

            cartesian_trajectory_p_isfinite(lb, x_tmp);
            cartesian_trajectory_planner_B.y_fd = true;
            cartesian_trajectory_planner_B.ix = 0;
            exitg2 = false;
            while ((!exitg2) && (cartesian_trajectory_planner_B.ix + 1 <=
                                 x_tmp->size[0])) {
              if (!x_tmp->data[cartesian_trajectory_planner_B.ix]) {
                cartesian_trajectory_planner_B.y_fd = false;
                exitg2 = true;
              } else {
                cartesian_trajectory_planner_B.ix++;
              }
            }

            guard1 = false;
            guard2 = false;
            guard3 = false;
            if (cartesian_trajectory_planner_B.y_fd) {
              cartesian_trajectory_p_isfinite(ub, x);
              cartesian_trajectory_planner_B.y_fd = true;
              cartesian_trajectory_planner_B.ix = 0;
              exitg2 = false;
              while ((!exitg2) && (cartesian_trajectory_planner_B.ix + 1 <=
                                   x->size[0])) {
                if (!x->data[cartesian_trajectory_planner_B.ix]) {
                  cartesian_trajectory_planner_B.y_fd = false;
                  exitg2 = true;
                } else {
                  cartesian_trajectory_planner_B.ix++;
                }
              }

              if (cartesian_trajectory_planner_B.y_fd) {
                cartesian_trajectory_pla_rand_a(obj_1->PositionNumber, rn);
                cartesian_trajectory_planner_B.ix = qi->size[0] * qi->size[1];
                qi->size[0] = lb->size[0];
                qi->size[1] = 1;
                cartes_emxEnsureCapacity_real_T(qi,
                  cartesian_trajectory_planner_B.ix);
                cartesian_trajectory_planner_B.nx = lb->size[0] - 1;
                for (cartesian_trajectory_planner_B.ix = 0;
                     cartesian_trajectory_planner_B.ix <=
                     cartesian_trajectory_planner_B.nx;
                     cartesian_trajectory_planner_B.ix++) {
                  qi->data[cartesian_trajectory_planner_B.ix] = (ub->
                    data[cartesian_trajectory_planner_B.ix] - lb->
                    data[cartesian_trajectory_planner_B.ix]) * rn->
                    data[cartesian_trajectory_planner_B.ix] + lb->
                    data[cartesian_trajectory_planner_B.ix];
                }
              } else {
                guard3 = true;
              }
            } else {
              guard3 = true;
            }

            if (guard3) {
              cartesian_trajectory_planner_B.y_fd = true;
              cartesian_trajectory_planner_B.ix = 0;
              exitg2 = false;
              while ((!exitg2) && (cartesian_trajectory_planner_B.ix + 1 <=
                                   x_tmp->size[0])) {
                if (!x_tmp->data[cartesian_trajectory_planner_B.ix]) {
                  cartesian_trajectory_planner_B.y_fd = false;
                  exitg2 = true;
                } else {
                  cartesian_trajectory_planner_B.ix++;
                }
              }

              if (cartesian_trajectory_planner_B.y_fd) {
                cartesian_trajectory_p_isfinite(ub, x);
                cartesian_trajectory_planner_B.ix = x_0->size[0];
                x_0->size[0] = x->size[0];
                car_emxEnsureCapacity_boolean_T(x_0,
                  cartesian_trajectory_planner_B.ix);
                cartesian_trajectory_planner_B.nx = x->size[0];
                for (cartesian_trajectory_planner_B.ix = 0;
                     cartesian_trajectory_planner_B.ix <
                     cartesian_trajectory_planner_B.nx;
                     cartesian_trajectory_planner_B.ix++) {
                  x_0->data[cartesian_trajectory_planner_B.ix] = !x->
                    data[cartesian_trajectory_planner_B.ix];
                }

                if (cartesian_trajectory_planne_any(x_0)) {
                  cartesian_trajectory_planner_B.ub[0] = lb->size[0];
                  cartesian_trajectory_planner_B.ub[1] = 1.0;
                  cartesian_trajectory_plan_randn
                    (cartesian_trajectory_planner_B.ub, qi);
                  cartesian_trajectory_planner_B.nx = qi->size[0] - 1;
                  cartesian_trajectory_planner_B.ix = e->size[0] * e->size[1];
                  e->size[0] = qi->size[0];
                  e->size[1] = 1;
                  cartes_emxEnsureCapacity_real_T(e,
                    cartesian_trajectory_planner_B.ix);
                  for (cartesian_trajectory_planner_B.ix = 0;
                       cartesian_trajectory_planner_B.ix <=
                       cartesian_trajectory_planner_B.nx;
                       cartesian_trajectory_planner_B.ix++) {
                    e->data[cartesian_trajectory_planner_B.ix] = fabs(qi->
                      data[cartesian_trajectory_planner_B.ix]);
                  }

                  cartesian_trajectory_planner_B.ix = qi->size[0] * qi->size[1];
                  qi->size[0] = lb->size[0];
                  qi->size[1] = 1;
                  cartes_emxEnsureCapacity_real_T(qi,
                    cartesian_trajectory_planner_B.ix);
                  cartesian_trajectory_planner_B.nx = lb->size[0] - 1;
                  for (cartesian_trajectory_planner_B.ix = 0;
                       cartesian_trajectory_planner_B.ix <=
                       cartesian_trajectory_planner_B.nx;
                       cartesian_trajectory_planner_B.ix++) {
                    qi->data[cartesian_trajectory_planner_B.ix] = lb->
                      data[cartesian_trajectory_planner_B.ix] + e->
                      data[cartesian_trajectory_planner_B.ix];
                  }
                } else {
                  guard2 = true;
                }
              } else {
                guard2 = true;
              }
            }

            if (guard2) {
              cartesian_trajectory_planner_B.ix = x_tmp_0->size[0];
              x_tmp_0->size[0] = x_tmp->size[0];
              car_emxEnsureCapacity_boolean_T(x_tmp_0,
                cartesian_trajectory_planner_B.ix);
              cartesian_trajectory_planner_B.nx = x_tmp->size[0];
              for (cartesian_trajectory_planner_B.ix = 0;
                   cartesian_trajectory_planner_B.ix <
                   cartesian_trajectory_planner_B.nx;
                   cartesian_trajectory_planner_B.ix++) {
                x_tmp_0->data[cartesian_trajectory_planner_B.ix] = !x_tmp->
                  data[cartesian_trajectory_planner_B.ix];
              }

              if (cartesian_trajectory_planne_any(x_tmp_0)) {
                cartesian_trajectory_p_isfinite(ub, x);
                cartesian_trajectory_planner_B.y_fd = true;
                cartesian_trajectory_planner_B.ix = 0;
                exitg2 = false;
                while ((!exitg2) && (cartesian_trajectory_planner_B.ix + 1 <=
                                     x->size[0])) {
                  if (!x->data[cartesian_trajectory_planner_B.ix]) {
                    cartesian_trajectory_planner_B.y_fd = false;
                    exitg2 = true;
                  } else {
                    cartesian_trajectory_planner_B.ix++;
                  }
                }

                if (cartesian_trajectory_planner_B.y_fd) {
                  cartesian_trajectory_planner_B.ub[0] = ub->size[0];
                  cartesian_trajectory_planner_B.ub[1] = 1.0;
                  cartesian_trajectory_plan_randn
                    (cartesian_trajectory_planner_B.ub, qi);
                  cartesian_trajectory_planner_B.nx = qi->size[0] - 1;
                  cartesian_trajectory_planner_B.ix = e->size[0] * e->size[1];
                  e->size[0] = qi->size[0];
                  e->size[1] = 1;
                  cartes_emxEnsureCapacity_real_T(e,
                    cartesian_trajectory_planner_B.ix);
                  for (cartesian_trajectory_planner_B.ix = 0;
                       cartesian_trajectory_planner_B.ix <=
                       cartesian_trajectory_planner_B.nx;
                       cartesian_trajectory_planner_B.ix++) {
                    e->data[cartesian_trajectory_planner_B.ix] = fabs(qi->
                      data[cartesian_trajectory_planner_B.ix]);
                  }

                  cartesian_trajectory_planner_B.ix = qi->size[0] * qi->size[1];
                  qi->size[0] = ub->size[0];
                  qi->size[1] = 1;
                  cartes_emxEnsureCapacity_real_T(qi,
                    cartesian_trajectory_planner_B.ix);
                  cartesian_trajectory_planner_B.nx = ub->size[0] - 1;
                  for (cartesian_trajectory_planner_B.ix = 0;
                       cartesian_trajectory_planner_B.ix <=
                       cartesian_trajectory_planner_B.nx;
                       cartesian_trajectory_planner_B.ix++) {
                    qi->data[cartesian_trajectory_planner_B.ix] = ub->
                      data[cartesian_trajectory_planner_B.ix] - e->
                      data[cartesian_trajectory_planner_B.ix];
                  }
                } else {
                  guard1 = true;
                }
              } else {
                guard1 = true;
              }
            }

            if (guard1) {
              cartesian_trajectory_planner_B.ub[0] = ub->size[0];
              cartesian_trajectory_planner_B.ub[1] = 1.0;
              cartesian_trajectory_plan_randn(cartesian_trajectory_planner_B.ub,
                qi);
            }
          }

          if (cartesian_trajectory_planner_B.err >
              cartesian_trajectory_planner_B.iter) {
            cartesian_trajectory_planner_B.nx = 0;
            cartesian_trajectory_planner_B.ix = 0;
          } else {
            cartesian_trajectory_planner_B.nx = static_cast<int32_T>
              (cartesian_trajectory_planner_B.err) - 1;
            cartesian_trajectory_planner_B.ix = static_cast<int32_T>
              (cartesian_trajectory_planner_B.iter);
          }

          cartesian_trajectory_planner_B.unnamed_idx_1 =
            cartesian_trajectory_planner_B.ix -
            cartesian_trajectory_planner_B.nx;
          for (cartesian_trajectory_planner_B.ix = 0;
               cartesian_trajectory_planner_B.ix <
               cartesian_trajectory_planner_B.unnamed_idx_1;
               cartesian_trajectory_planner_B.ix++) {
            newseed->data[cartesian_trajectory_planner_B.nx +
              cartesian_trajectory_planner_B.ix] = qi->
              data[cartesian_trajectory_planner_B.ix];
          }
        }
      }

      for (cartesian_trajectory_planner_B.ix = 0;
           cartesian_trajectory_planner_B.ix < 6;
           cartesian_trajectory_planner_B.ix++) {
        obj->SeedInternal[cartesian_trajectory_planner_B.ix] = newseed->
          data[cartesian_trajectory_planner_B.ix];
      }

      DampedBFGSwGradientProjection_s(obj, cartesian_trajectory_planner_B.c_xSol,
        &cartesian_trajectory_planner_B.exitFlag,
        &cartesian_trajectory_planner_B.err,
        &cartesian_trajectory_planner_B.iter);
      if (cartesian_trajectory_planner_B.err < *solutionInfo_Error) {
        for (cartesian_trajectory_planner_B.i_h = 0;
             cartesian_trajectory_planner_B.i_h < 6;
             cartesian_trajectory_planner_B.i_h++) {
          xSol[cartesian_trajectory_planner_B.i_h] =
            cartesian_trajectory_planner_B.c_xSol[cartesian_trajectory_planner_B.i_h];
        }

        *solutionInfo_Error = cartesian_trajectory_planner_B.err;
        cartesian_trajectory_planner_B.exitFlagPrev =
          cartesian_trajectory_planner_B.exitFlag;
      }

      (*solutionInfo_RRAttempts)++;
      *solutionInfo_Iterations += cartesian_trajectory_planner_B.iter;
    }
  }

  cartesian_tra_emxFree_boolean_T(&x_0);
  cartesian_tra_emxFree_boolean_T(&x_tmp_0);
  cartesian_tra_emxFree_boolean_T(&x_tmp);
  cartesian_tra_emxFree_boolean_T(&x);
  cartesian_trajec_emxFree_real_T(&e);
  cartesian_trajec_emxFree_real_T(&rn);
  cartesian_trajec_emxFree_real_T(&lb);
  cartesian_trajec_emxFree_real_T(&ub);
  cartesian_trajec_emxFree_real_T(&qi);
  cartesian_trajec_emxFree_real_T(&newseed);
  *solutionInfo_ExitFlag = cartesian_trajectory_planner_B.exitFlagPrev;
  if (*solutionInfo_Error < cartesian_trajectory_planner_B.tol) {
    solutionInfo_Status_size[0] = 1;
    solutionInfo_Status_size[1] = 7;
    for (cartesian_trajectory_planner_B.ix = 0;
         cartesian_trajectory_planner_B.ix < 7;
         cartesian_trajectory_planner_B.ix++) {
      solutionInfo_Status_data[cartesian_trajectory_planner_B.ix] =
        tmp_0[cartesian_trajectory_planner_B.ix];
    }
  } else {
    solutionInfo_Status_size[0] = 1;
    solutionInfo_Status_size[1] = 14;
    for (cartesian_trajectory_planner_B.ix = 0;
         cartesian_trajectory_planner_B.ix < 14;
         cartesian_trajectory_planner_B.ix++) {
      solutionInfo_Status_data[cartesian_trajectory_planner_B.ix] =
        tmp[cartesian_trajectory_planner_B.ix];
    }
  }
}

static void cart_inverseKinematics_stepImpl(b_inverseKinematics_cartesian_T *obj,
  const real_T tform[16], const real_T weights[6], const real_T initialGuess[6],
  real_T QSol[6])
{
  emxArray_real_T_cartesian_tra_T *bodyIndices;
  emxArray_real_T_cartesian_tra_T *positionIndices;
  x_robotics_manip_internal_Rig_T *obj_0;
  emxArray_char_T_cartesian_tra_T *endEffectorName;
  w_robotics_manip_internal_Rig_T *body;
  emxArray_real_T_cartesian_tra_T *positionMap;
  emxArray_real_T_cartesian_tra_T *e;
  emxArray_int32_T_cartesian_tr_T *h;
  emxArray_real_T_cartesian_tra_T *y;
  emxArray_char_T_cartesian_tra_T *bname;
  emxArray_real_T_cartesian_tra_T *bodyIndices_0;
  boolean_T exitg1;
  c_inverseKinematics_setPoseGoal(obj, tform, weights);
  for (cartesian_trajectory_planner_B.i = 0; cartesian_trajectory_planner_B.i <
       6; cartesian_trajectory_planner_B.i++) {
    QSol[cartesian_trajectory_planner_B.i] =
      initialGuess[cartesian_trajectory_planner_B.i];
  }

  cartesian_trajec_emxInit_char_T(&endEffectorName, 2);
  RigidBodyTree_validateConfigu_a(obj->RigidBodyTreeInternal, QSol);
  cartes_NLPSolverInterface_solve(obj->Solver, QSol,
    cartesian_trajectory_planner_B.qvSolRaw, &cartesian_trajectory_planner_B.bid,
    &cartesian_trajectory_planner_B.numPositions,
    &cartesian_trajectory_planner_B.ndbl, &cartesian_trajectory_planner_B.apnd,
    cartesian_trajectory_planner_B.sol_Status_data,
    cartesian_trajectory_planner_B.sol_Status_size);
  obj_0 = obj->RigidBodyTreeInternal;
  cartesian_trajectory_planner_B.partialTrueCount = endEffectorName->size[0] *
    endEffectorName->size[1];
  endEffectorName->size[0] = 1;
  endEffectorName->size[1] = obj->Solver->ExtraArgs->BodyName->size[1];
  cartes_emxEnsureCapacity_char_T(endEffectorName,
    cartesian_trajectory_planner_B.partialTrueCount);
  cartesian_trajectory_planner_B.loop_ub = obj->Solver->ExtraArgs->
    BodyName->size[0] * obj->Solver->ExtraArgs->BodyName->size[1] - 1;
  for (cartesian_trajectory_planner_B.partialTrueCount = 0;
       cartesian_trajectory_planner_B.partialTrueCount <=
       cartesian_trajectory_planner_B.loop_ub;
       cartesian_trajectory_planner_B.partialTrueCount++) {
    endEffectorName->data[cartesian_trajectory_planner_B.partialTrueCount] =
      obj->Solver->ExtraArgs->BodyName->
      data[cartesian_trajectory_planner_B.partialTrueCount];
  }

  cartesian_trajec_emxInit_real_T(&bodyIndices, 1);
  cartesian_trajectory_planner_B.partialTrueCount = bodyIndices->size[0];
  bodyIndices->size[0] = static_cast<int32_T>(obj_0->NumBodies);
  cartes_emxEnsureCapacity_real_T(bodyIndices,
    cartesian_trajectory_planner_B.partialTrueCount);
  cartesian_trajectory_planner_B.loop_ub = static_cast<int32_T>(obj_0->NumBodies);
  for (cartesian_trajectory_planner_B.partialTrueCount = 0;
       cartesian_trajectory_planner_B.partialTrueCount <
       cartesian_trajectory_planner_B.loop_ub;
       cartesian_trajectory_planner_B.partialTrueCount++) {
    bodyIndices->data[cartesian_trajectory_planner_B.partialTrueCount] = 0.0;
  }

  cartesian_trajec_emxInit_char_T(&bname, 2);
  cartesian_trajectory_planner_B.bid = -1.0;
  cartesian_trajectory_planner_B.partialTrueCount = bname->size[0] * bname->
    size[1];
  bname->size[0] = 1;
  bname->size[1] = obj_0->Base.NameInternal->size[1];
  cartes_emxEnsureCapacity_char_T(bname,
    cartesian_trajectory_planner_B.partialTrueCount);
  cartesian_trajectory_planner_B.loop_ub = obj_0->Base.NameInternal->size[0] *
    obj_0->Base.NameInternal->size[1] - 1;
  for (cartesian_trajectory_planner_B.partialTrueCount = 0;
       cartesian_trajectory_planner_B.partialTrueCount <=
       cartesian_trajectory_planner_B.loop_ub;
       cartesian_trajectory_planner_B.partialTrueCount++) {
    bname->data[cartesian_trajectory_planner_B.partialTrueCount] =
      obj_0->Base.NameInternal->
      data[cartesian_trajectory_planner_B.partialTrueCount];
  }

  if (cartesian_trajectory_pla_strcmp(bname, endEffectorName)) {
    cartesian_trajectory_planner_B.bid = 0.0;
  } else {
    cartesian_trajectory_planner_B.numPositions = obj_0->NumBodies;
    cartesian_trajectory_planner_B.i = 0;
    exitg1 = false;
    while ((!exitg1) && (cartesian_trajectory_planner_B.i <= static_cast<int32_T>
                         (cartesian_trajectory_planner_B.numPositions) - 1)) {
      body = obj_0->Bodies[cartesian_trajectory_planner_B.i];
      cartesian_trajectory_planner_B.partialTrueCount = bname->size[0] *
        bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = body->NameInternal->size[1];
      cartes_emxEnsureCapacity_char_T(bname,
        cartesian_trajectory_planner_B.partialTrueCount);
      cartesian_trajectory_planner_B.loop_ub = body->NameInternal->size[0] *
        body->NameInternal->size[1] - 1;
      for (cartesian_trajectory_planner_B.partialTrueCount = 0;
           cartesian_trajectory_planner_B.partialTrueCount <=
           cartesian_trajectory_planner_B.loop_ub;
           cartesian_trajectory_planner_B.partialTrueCount++) {
        bname->data[cartesian_trajectory_planner_B.partialTrueCount] =
          body->NameInternal->
          data[cartesian_trajectory_planner_B.partialTrueCount];
      }

      if (cartesian_trajectory_pla_strcmp(bname, endEffectorName)) {
        cartesian_trajectory_planner_B.bid = static_cast<real_T>
          (cartesian_trajectory_planner_B.i) + 1.0;
        exitg1 = true;
      } else {
        cartesian_trajectory_planner_B.i++;
      }
    }
  }

  cartesian_trajec_emxFree_char_T(&bname);
  cartesian_trajec_emxFree_char_T(&endEffectorName);
  if (cartesian_trajectory_planner_B.bid == 0.0) {
    cartesian_trajectory_planner_B.partialTrueCount = bodyIndices->size[0];
    bodyIndices->size[0] = 1;
    cartes_emxEnsureCapacity_real_T(bodyIndices,
      cartesian_trajectory_planner_B.partialTrueCount);
    bodyIndices->data[0] = 0.0;
  } else {
    body = obj_0->Bodies[static_cast<int32_T>(cartesian_trajectory_planner_B.bid)
      - 1];
    cartesian_trajectory_planner_B.bid = 1.0;
    while (body->ParentIndex != 0.0) {
      bodyIndices->data[static_cast<int32_T>(cartesian_trajectory_planner_B.bid)
        - 1] = body->Index;
      body = obj_0->Bodies[static_cast<int32_T>(body->ParentIndex) - 1];
      cartesian_trajectory_planner_B.bid++;
    }

    if (1.0 > cartesian_trajectory_planner_B.bid - 1.0) {
      cartesian_trajectory_planner_B.c_i = -1;
    } else {
      cartesian_trajectory_planner_B.c_i = static_cast<int32_T>
        (cartesian_trajectory_planner_B.bid - 1.0) - 1;
    }

    cartesian_trajec_emxInit_real_T(&bodyIndices_0, 1);
    cartesian_trajectory_planner_B.partialTrueCount = bodyIndices_0->size[0];
    bodyIndices_0->size[0] = cartesian_trajectory_planner_B.c_i + 3;
    cartes_emxEnsureCapacity_real_T(bodyIndices_0,
      cartesian_trajectory_planner_B.partialTrueCount);
    for (cartesian_trajectory_planner_B.partialTrueCount = 0;
         cartesian_trajectory_planner_B.partialTrueCount <=
         cartesian_trajectory_planner_B.c_i;
         cartesian_trajectory_planner_B.partialTrueCount++) {
      bodyIndices_0->data[cartesian_trajectory_planner_B.partialTrueCount] =
        bodyIndices->data[cartesian_trajectory_planner_B.partialTrueCount];
    }

    bodyIndices_0->data[cartesian_trajectory_planner_B.c_i + 1] = body->Index;
    bodyIndices_0->data[cartesian_trajectory_planner_B.c_i + 2] = 0.0;
    cartesian_trajectory_planner_B.partialTrueCount = bodyIndices->size[0];
    bodyIndices->size[0] = bodyIndices_0->size[0];
    cartes_emxEnsureCapacity_real_T(bodyIndices,
      cartesian_trajectory_planner_B.partialTrueCount);
    cartesian_trajectory_planner_B.loop_ub = bodyIndices_0->size[0];
    for (cartesian_trajectory_planner_B.partialTrueCount = 0;
         cartesian_trajectory_planner_B.partialTrueCount <
         cartesian_trajectory_planner_B.loop_ub;
         cartesian_trajectory_planner_B.partialTrueCount++) {
      bodyIndices->data[cartesian_trajectory_planner_B.partialTrueCount] =
        bodyIndices_0->data[cartesian_trajectory_planner_B.partialTrueCount];
    }

    cartesian_trajec_emxFree_real_T(&bodyIndices_0);
  }

  obj_0 = obj->RigidBodyTreeInternal;
  cartesian_trajectory_planner_B.c_i = bodyIndices->size[0] - 1;
  cartesian_trajectory_planner_B.loop_ub = 0;
  for (cartesian_trajectory_planner_B.i = 0; cartesian_trajectory_planner_B.i <=
       cartesian_trajectory_planner_B.c_i; cartesian_trajectory_planner_B.i++) {
    if (bodyIndices->data[cartesian_trajectory_planner_B.i] != 0.0) {
      cartesian_trajectory_planner_B.loop_ub++;
    }
  }

  cartesian_traje_emxInit_int32_T(&h, 1);
  cartesian_trajectory_planner_B.partialTrueCount = h->size[0];
  h->size[0] = cartesian_trajectory_planner_B.loop_ub;
  carte_emxEnsureCapacity_int32_T(h,
    cartesian_trajectory_planner_B.partialTrueCount);
  cartesian_trajectory_planner_B.partialTrueCount = 0;
  for (cartesian_trajectory_planner_B.i = 0; cartesian_trajectory_planner_B.i <=
       cartesian_trajectory_planner_B.c_i; cartesian_trajectory_planner_B.i++) {
    if (bodyIndices->data[cartesian_trajectory_planner_B.i] != 0.0) {
      h->data[cartesian_trajectory_planner_B.partialTrueCount] =
        cartesian_trajectory_planner_B.i + 1;
      cartesian_trajectory_planner_B.partialTrueCount++;
    }
  }

  cartesian_trajec_emxInit_real_T(&positionMap, 2);
  cartesian_trajectory_planner_B.partialTrueCount = positionMap->size[0] *
    positionMap->size[1];
  positionMap->size[0] = h->size[0];
  positionMap->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(positionMap,
    cartesian_trajectory_planner_B.partialTrueCount);
  cartesian_trajectory_planner_B.loop_ub = h->size[0];
  for (cartesian_trajectory_planner_B.partialTrueCount = 0;
       cartesian_trajectory_planner_B.partialTrueCount <
       cartesian_trajectory_planner_B.loop_ub;
       cartesian_trajectory_planner_B.partialTrueCount++) {
    positionMap->data[cartesian_trajectory_planner_B.partialTrueCount] =
      obj_0->PositionDoFMap[static_cast<int32_T>(bodyIndices->data[h->
      data[cartesian_trajectory_planner_B.partialTrueCount] - 1]) - 1];
  }

  cartesian_trajectory_planner_B.loop_ub = h->size[0];
  for (cartesian_trajectory_planner_B.partialTrueCount = 0;
       cartesian_trajectory_planner_B.partialTrueCount <
       cartesian_trajectory_planner_B.loop_ub;
       cartesian_trajectory_planner_B.partialTrueCount++) {
    positionMap->data[cartesian_trajectory_planner_B.partialTrueCount +
      positionMap->size[0]] = obj_0->PositionDoFMap[static_cast<int32_T>
      (bodyIndices->data[h->data[cartesian_trajectory_planner_B.partialTrueCount]
       - 1]) + 7];
  }

  cartesian_traje_emxFree_int32_T(&h);
  cartesian_trajec_emxFree_real_T(&bodyIndices);
  cartesian_trajec_emxInit_real_T(&positionIndices, 2);
  cartesian_trajectory_planner_B.partialTrueCount = positionIndices->size[0] *
    positionIndices->size[1];
  positionIndices->size[0] = 1;
  positionIndices->size[1] = static_cast<int32_T>(obj_0->PositionNumber);
  cartes_emxEnsureCapacity_real_T(positionIndices,
    cartesian_trajectory_planner_B.partialTrueCount);
  cartesian_trajectory_planner_B.loop_ub = static_cast<int32_T>
    (obj_0->PositionNumber) - 1;
  for (cartesian_trajectory_planner_B.partialTrueCount = 0;
       cartesian_trajectory_planner_B.partialTrueCount <=
       cartesian_trajectory_planner_B.loop_ub;
       cartesian_trajectory_planner_B.partialTrueCount++) {
    positionIndices->data[cartesian_trajectory_planner_B.partialTrueCount] = 0.0;
  }

  cartesian_trajectory_planner_B.bid = 0.0;
  cartesian_trajectory_planner_B.c_i = positionMap->size[0] - 1;
  cartesian_trajec_emxInit_real_T(&e, 2);
  cartesian_trajec_emxInit_real_T(&y, 2);
  for (cartesian_trajectory_planner_B.i = 0; cartesian_trajectory_planner_B.i <=
       cartesian_trajectory_planner_B.c_i; cartesian_trajectory_planner_B.i++) {
    cartesian_trajectory_planner_B.numPositions = (positionMap->
      data[cartesian_trajectory_planner_B.i + positionMap->size[0]] -
      positionMap->data[cartesian_trajectory_planner_B.i]) + 1.0;
    if (cartesian_trajectory_planner_B.numPositions > 0.0) {
      if (cartesian_trajectory_planner_B.numPositions < 1.0) {
        y->size[0] = 1;
        y->size[1] = 0;
      } else if (rtIsInf(cartesian_trajectory_planner_B.numPositions) && (1.0 ==
                  cartesian_trajectory_planner_B.numPositions)) {
        cartesian_trajectory_planner_B.partialTrueCount = y->size[0] * y->size[1];
        y->size[0] = 1;
        y->size[1] = 1;
        cartes_emxEnsureCapacity_real_T(y,
          cartesian_trajectory_planner_B.partialTrueCount);
        y->data[0] = (rtNaN);
      } else {
        cartesian_trajectory_planner_B.partialTrueCount = y->size[0] * y->size[1];
        y->size[0] = 1;
        cartesian_trajectory_planner_B.loop_ub = static_cast<int32_T>(floor
          (cartesian_trajectory_planner_B.numPositions - 1.0));
        y->size[1] = cartesian_trajectory_planner_B.loop_ub + 1;
        cartes_emxEnsureCapacity_real_T(y,
          cartesian_trajectory_planner_B.partialTrueCount);
        for (cartesian_trajectory_planner_B.partialTrueCount = 0;
             cartesian_trajectory_planner_B.partialTrueCount <=
             cartesian_trajectory_planner_B.loop_ub;
             cartesian_trajectory_planner_B.partialTrueCount++) {
          y->data[cartesian_trajectory_planner_B.partialTrueCount] =
            static_cast<real_T>(cartesian_trajectory_planner_B.partialTrueCount)
            + 1.0;
        }
      }

      if (rtIsNaN(positionMap->data[cartesian_trajectory_planner_B.i]) ||
          rtIsNaN(positionMap->data[cartesian_trajectory_planner_B.i +
                  positionMap->size[0]])) {
        cartesian_trajectory_planner_B.partialTrueCount = e->size[0] * e->size[1];
        e->size[0] = 1;
        e->size[1] = 1;
        cartes_emxEnsureCapacity_real_T(e,
          cartesian_trajectory_planner_B.partialTrueCount);
        e->data[0] = (rtNaN);
      } else if (positionMap->data[cartesian_trajectory_planner_B.i +
                 positionMap->size[0]] < positionMap->
                 data[cartesian_trajectory_planner_B.i]) {
        e->size[0] = 1;
        e->size[1] = 0;
      } else if ((rtIsInf(positionMap->data[cartesian_trajectory_planner_B.i]) ||
                  rtIsInf(positionMap->data[cartesian_trajectory_planner_B.i +
                          positionMap->size[0]])) && (positionMap->
                  data[cartesian_trajectory_planner_B.i + positionMap->size[0]] ==
                  positionMap->data[cartesian_trajectory_planner_B.i])) {
        cartesian_trajectory_planner_B.partialTrueCount = e->size[0] * e->size[1];
        e->size[0] = 1;
        e->size[1] = 1;
        cartes_emxEnsureCapacity_real_T(e,
          cartesian_trajectory_planner_B.partialTrueCount);
        e->data[0] = (rtNaN);
      } else if (floor(positionMap->data[cartesian_trajectory_planner_B.i]) ==
                 positionMap->data[cartesian_trajectory_planner_B.i]) {
        cartesian_trajectory_planner_B.partialTrueCount = e->size[0] * e->size[1];
        e->size[0] = 1;
        e->size[1] = static_cast<int32_T>(floor(positionMap->
          data[cartesian_trajectory_planner_B.i + positionMap->size[0]] -
          positionMap->data[cartesian_trajectory_planner_B.i])) + 1;
        cartes_emxEnsureCapacity_real_T(e,
          cartesian_trajectory_planner_B.partialTrueCount);
        cartesian_trajectory_planner_B.loop_ub = static_cast<int32_T>(floor
          (positionMap->data[cartesian_trajectory_planner_B.i +
           positionMap->size[0]] - positionMap->
           data[cartesian_trajectory_planner_B.i]));
        for (cartesian_trajectory_planner_B.partialTrueCount = 0;
             cartesian_trajectory_planner_B.partialTrueCount <=
             cartesian_trajectory_planner_B.loop_ub;
             cartesian_trajectory_planner_B.partialTrueCount++) {
          e->data[cartesian_trajectory_planner_B.partialTrueCount] =
            positionMap->data[cartesian_trajectory_planner_B.i] +
            static_cast<real_T>(cartesian_trajectory_planner_B.partialTrueCount);
        }
      } else {
        cartesian_trajectory_planner_B.ndbl = floor((positionMap->
          data[cartesian_trajectory_planner_B.i + positionMap->size[0]] -
          positionMap->data[cartesian_trajectory_planner_B.i]) + 0.5);
        cartesian_trajectory_planner_B.apnd = positionMap->
          data[cartesian_trajectory_planner_B.i] +
          cartesian_trajectory_planner_B.ndbl;
        cartesian_trajectory_planner_B.cdiff =
          cartesian_trajectory_planner_B.apnd - positionMap->
          data[cartesian_trajectory_planner_B.i + positionMap->size[0]];
        cartesian_trajectory_planner_B.u0 = fabs(positionMap->
          data[cartesian_trajectory_planner_B.i]);
        cartesian_trajectory_planner_B.u1 = fabs(positionMap->
          data[cartesian_trajectory_planner_B.i + positionMap->size[0]]);
        if ((cartesian_trajectory_planner_B.u0 >
             cartesian_trajectory_planner_B.u1) || rtIsNaN
            (cartesian_trajectory_planner_B.u1)) {
          cartesian_trajectory_planner_B.u1 = cartesian_trajectory_planner_B.u0;
        }

        if (fabs(cartesian_trajectory_planner_B.cdiff) < 4.4408920985006262E-16 *
            cartesian_trajectory_planner_B.u1) {
          cartesian_trajectory_planner_B.ndbl++;
          cartesian_trajectory_planner_B.apnd = positionMap->
            data[cartesian_trajectory_planner_B.i + positionMap->size[0]];
        } else if (cartesian_trajectory_planner_B.cdiff > 0.0) {
          cartesian_trajectory_planner_B.apnd =
            (cartesian_trajectory_planner_B.ndbl - 1.0) + positionMap->
            data[cartesian_trajectory_planner_B.i];
        } else {
          cartesian_trajectory_planner_B.ndbl++;
        }

        if (cartesian_trajectory_planner_B.ndbl >= 0.0) {
          cartesian_trajectory_planner_B.partialTrueCount = static_cast<int32_T>
            (cartesian_trajectory_planner_B.ndbl);
        } else {
          cartesian_trajectory_planner_B.partialTrueCount = 0;
        }

        cartesian_trajectory_planner_B.loop_ub =
          cartesian_trajectory_planner_B.partialTrueCount - 1;
        cartesian_trajectory_planner_B.partialTrueCount = e->size[0] * e->size[1];
        e->size[0] = 1;
        e->size[1] = cartesian_trajectory_planner_B.loop_ub + 1;
        cartes_emxEnsureCapacity_real_T(e,
          cartesian_trajectory_planner_B.partialTrueCount);
        if (cartesian_trajectory_planner_B.loop_ub + 1 > 0) {
          e->data[0] = positionMap->data[cartesian_trajectory_planner_B.i];
          if (cartesian_trajectory_planner_B.loop_ub + 1 > 1) {
            e->data[cartesian_trajectory_planner_B.loop_ub] =
              cartesian_trajectory_planner_B.apnd;
            cartesian_trajectory_planner_B.nm1d2 =
              ((cartesian_trajectory_planner_B.loop_ub < 0) +
               cartesian_trajectory_planner_B.loop_ub) >> 1;
            cartesian_trajectory_planner_B.c_c =
              cartesian_trajectory_planner_B.nm1d2 - 2;
            for (cartesian_trajectory_planner_B.partialTrueCount = 0;
                 cartesian_trajectory_planner_B.partialTrueCount <=
                 cartesian_trajectory_planner_B.c_c;
                 cartesian_trajectory_planner_B.partialTrueCount++) {
              cartesian_trajectory_planner_B.k_f4 =
                cartesian_trajectory_planner_B.partialTrueCount + 1;
              e->data[cartesian_trajectory_planner_B.k_f4] = positionMap->
                data[cartesian_trajectory_planner_B.i] + static_cast<real_T>
                (cartesian_trajectory_planner_B.k_f4);
              e->data[cartesian_trajectory_planner_B.loop_ub -
                cartesian_trajectory_planner_B.k_f4] =
                cartesian_trajectory_planner_B.apnd - static_cast<real_T>
                (cartesian_trajectory_planner_B.k_f4);
            }

            if (cartesian_trajectory_planner_B.nm1d2 << 1 ==
                cartesian_trajectory_planner_B.loop_ub) {
              e->data[cartesian_trajectory_planner_B.nm1d2] = (positionMap->
                data[cartesian_trajectory_planner_B.i] +
                cartesian_trajectory_planner_B.apnd) / 2.0;
            } else {
              e->data[cartesian_trajectory_planner_B.nm1d2] = positionMap->
                data[cartesian_trajectory_planner_B.i] + static_cast<real_T>
                (cartesian_trajectory_planner_B.nm1d2);
              e->data[cartesian_trajectory_planner_B.nm1d2 + 1] =
                cartesian_trajectory_planner_B.apnd - static_cast<real_T>
                (cartesian_trajectory_planner_B.nm1d2);
            }
          }
        }
      }

      cartesian_trajectory_planner_B.partialTrueCount = e->size[0] * e->size[1];
      cartesian_trajectory_planner_B.loop_ub =
        cartesian_trajectory_planner_B.partialTrueCount - 1;
      for (cartesian_trajectory_planner_B.partialTrueCount = 0;
           cartesian_trajectory_planner_B.partialTrueCount <=
           cartesian_trajectory_planner_B.loop_ub;
           cartesian_trajectory_planner_B.partialTrueCount++) {
        positionIndices->data[static_cast<int32_T>
          (cartesian_trajectory_planner_B.bid + y->
           data[cartesian_trajectory_planner_B.partialTrueCount]) - 1] = e->
          data[cartesian_trajectory_planner_B.partialTrueCount];
      }

      cartesian_trajectory_planner_B.bid +=
        cartesian_trajectory_planner_B.numPositions;
    }
  }

  cartesian_trajec_emxFree_real_T(&y);
  cartesian_trajec_emxFree_real_T(&e);
  cartesian_trajec_emxFree_real_T(&positionMap);
  if (1.0 > cartesian_trajectory_planner_B.bid) {
    positionIndices->size[1] = 0;
  } else {
    cartesian_trajectory_planner_B.partialTrueCount = positionIndices->size[0] *
      positionIndices->size[1];
    positionIndices->size[1] = static_cast<int32_T>
      (cartesian_trajectory_planner_B.bid);
    cartes_emxEnsureCapacity_real_T(positionIndices,
      cartesian_trajectory_planner_B.partialTrueCount);
  }

  cartesian_trajectory_planner_B.loop_ub = positionIndices->size[0] *
    positionIndices->size[1];
  for (cartesian_trajectory_planner_B.partialTrueCount = 0;
       cartesian_trajectory_planner_B.partialTrueCount <
       cartesian_trajectory_planner_B.loop_ub;
       cartesian_trajectory_planner_B.partialTrueCount++) {
    QSol[static_cast<int32_T>(positionIndices->
      data[cartesian_trajectory_planner_B.partialTrueCount]) - 1] =
      cartesian_trajectory_planner_B.qvSolRaw[static_cast<int32_T>
      (positionIndices->data[cartesian_trajectory_planner_B.partialTrueCount]) -
      1];
  }

  cartesian_trajec_emxFree_real_T(&positionIndices);
}

static void c_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_cartesia_T
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
                     (f_cell_wrap_cartesian_traject_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(f_cell_wrap_cartesian_traject_T)
             * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (f_cell_wrap_cartesian_traject_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void ca_rigidBodyJoint_get_JointAxis(const
  c_rigidBodyJoint_cartesian_tr_T *obj, real_T ax[3])
{
  emxArray_char_T_cartesian_tra_T *a;
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  boolean_T guard1 = false;
  int32_T exitg1;
  cartesian_trajec_emxInit_char_T(&a, 2);
  cartesian_trajectory_planner_B.b_kstr_j5 = a->size[0] * a->size[1];
  a->size[0] = 1;
  a->size[1] = obj->Type->size[1];
  cartes_emxEnsureCapacity_char_T(a, cartesian_trajectory_planner_B.b_kstr_j5);
  cartesian_trajectory_planner_B.loop_ub_a3 = obj->Type->size[0] * obj->
    Type->size[1] - 1;
  for (cartesian_trajectory_planner_B.b_kstr_j5 = 0;
       cartesian_trajectory_planner_B.b_kstr_j5 <=
       cartesian_trajectory_planner_B.loop_ub_a3;
       cartesian_trajectory_planner_B.b_kstr_j5++) {
    a->data[cartesian_trajectory_planner_B.b_kstr_j5] = obj->Type->
      data[cartesian_trajectory_planner_B.b_kstr_j5];
  }

  for (cartesian_trajectory_planner_B.b_kstr_j5 = 0;
       cartesian_trajectory_planner_B.b_kstr_j5 < 8;
       cartesian_trajectory_planner_B.b_kstr_j5++) {
    cartesian_trajectory_planner_B.b_g4[cartesian_trajectory_planner_B.b_kstr_j5]
      = tmp[cartesian_trajectory_planner_B.b_kstr_j5];
  }

  cartesian_trajectory_planner_B.b_bool_aq = false;
  if (a->size[1] == 8) {
    cartesian_trajectory_planner_B.b_kstr_j5 = 1;
    do {
      exitg1 = 0;
      if (cartesian_trajectory_planner_B.b_kstr_j5 - 1 < 8) {
        cartesian_trajectory_planner_B.loop_ub_a3 =
          cartesian_trajectory_planner_B.b_kstr_j5 - 1;
        if (a->data[cartesian_trajectory_planner_B.loop_ub_a3] !=
            cartesian_trajectory_planner_B.b_g4[cartesian_trajectory_planner_B.loop_ub_a3])
        {
          exitg1 = 1;
        } else {
          cartesian_trajectory_planner_B.b_kstr_j5++;
        }
      } else {
        cartesian_trajectory_planner_B.b_bool_aq = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (cartesian_trajectory_planner_B.b_bool_aq) {
    guard1 = true;
  } else {
    cartesian_trajectory_planner_B.b_kstr_j5 = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = obj->Type->size[1];
    cartes_emxEnsureCapacity_char_T(a, cartesian_trajectory_planner_B.b_kstr_j5);
    cartesian_trajectory_planner_B.loop_ub_a3 = obj->Type->size[0] * obj->
      Type->size[1] - 1;
    for (cartesian_trajectory_planner_B.b_kstr_j5 = 0;
         cartesian_trajectory_planner_B.b_kstr_j5 <=
         cartesian_trajectory_planner_B.loop_ub_a3;
         cartesian_trajectory_planner_B.b_kstr_j5++) {
      a->data[cartesian_trajectory_planner_B.b_kstr_j5] = obj->Type->
        data[cartesian_trajectory_planner_B.b_kstr_j5];
    }

    for (cartesian_trajectory_planner_B.b_kstr_j5 = 0;
         cartesian_trajectory_planner_B.b_kstr_j5 < 9;
         cartesian_trajectory_planner_B.b_kstr_j5++) {
      cartesian_trajectory_planner_B.b_o[cartesian_trajectory_planner_B.b_kstr_j5]
        = tmp_0[cartesian_trajectory_planner_B.b_kstr_j5];
    }

    cartesian_trajectory_planner_B.b_bool_aq = false;
    if (a->size[1] == 9) {
      cartesian_trajectory_planner_B.b_kstr_j5 = 1;
      do {
        exitg1 = 0;
        if (cartesian_trajectory_planner_B.b_kstr_j5 - 1 < 9) {
          cartesian_trajectory_planner_B.loop_ub_a3 =
            cartesian_trajectory_planner_B.b_kstr_j5 - 1;
          if (a->data[cartesian_trajectory_planner_B.loop_ub_a3] !=
              cartesian_trajectory_planner_B.b_o[cartesian_trajectory_planner_B.loop_ub_a3])
          {
            exitg1 = 1;
          } else {
            cartesian_trajectory_planner_B.b_kstr_j5++;
          }
        } else {
          cartesian_trajectory_planner_B.b_bool_aq = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (cartesian_trajectory_planner_B.b_bool_aq) {
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

  cartesian_trajec_emxFree_char_T(&a);
}

static void RigidBodyTree_forwardKinematics(p_robotics_manip_internal_Rig_T *obj,
  const real_T qvec[6], emxArray_f_cell_wrap_cartesia_T *Ttree)
{
  n_robotics_manip_internal_Rig_T *body;
  emxArray_char_T_cartesian_tra_T *switch_expression;
  static const int8_T tmp[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  cartesian_trajectory_planner_B.n = obj->NumBodies;
  for (cartesian_trajectory_planner_B.b_kstr_d = 0;
       cartesian_trajectory_planner_B.b_kstr_d < 16;
       cartesian_trajectory_planner_B.b_kstr_d++) {
    cartesian_trajectory_planner_B.c_f1[cartesian_trajectory_planner_B.b_kstr_d]
      = tmp[cartesian_trajectory_planner_B.b_kstr_d];
  }

  cartesian_trajectory_planner_B.b_kstr_d = Ttree->size[0] * Ttree->size[1];
  Ttree->size[0] = 1;
  cartesian_trajectory_planner_B.e_kb = static_cast<int32_T>
    (cartesian_trajectory_planner_B.n);
  Ttree->size[1] = cartesian_trajectory_planner_B.e_kb;
  c_emxEnsureCapacity_f_cell_wrap(Ttree, cartesian_trajectory_planner_B.b_kstr_d);
  if (cartesian_trajectory_planner_B.e_kb != 0) {
    cartesian_trajectory_planner_B.ntilecols =
      cartesian_trajectory_planner_B.e_kb - 1;
    if (0 <= cartesian_trajectory_planner_B.ntilecols) {
      memcpy(&cartesian_trajectory_planner_B.expl_temp.f1[0],
             &cartesian_trajectory_planner_B.c_f1[0], sizeof(real_T) << 4U);
    }

    for (cartesian_trajectory_planner_B.b_jtilecol = 0;
         cartesian_trajectory_planner_B.b_jtilecol <=
         cartesian_trajectory_planner_B.ntilecols;
         cartesian_trajectory_planner_B.b_jtilecol++) {
      Ttree->data[cartesian_trajectory_planner_B.b_jtilecol] =
        cartesian_trajectory_planner_B.expl_temp;
    }
  }

  cartesian_trajectory_planner_B.k = 1.0;
  cartesian_trajectory_planner_B.ntilecols = static_cast<int32_T>
    (cartesian_trajectory_planner_B.n) - 1;
  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  if (0 <= cartesian_trajectory_planner_B.ntilecols) {
    for (cartesian_trajectory_planner_B.b_kstr_d = 0;
         cartesian_trajectory_planner_B.b_kstr_d < 5;
         cartesian_trajectory_planner_B.b_kstr_d++) {
      cartesian_trajectory_planner_B.b_oo[cartesian_trajectory_planner_B.b_kstr_d]
        = tmp_0[cartesian_trajectory_planner_B.b_kstr_d];
    }
  }

  for (cartesian_trajectory_planner_B.b_jtilecol = 0;
       cartesian_trajectory_planner_B.b_jtilecol <=
       cartesian_trajectory_planner_B.ntilecols;
       cartesian_trajectory_planner_B.b_jtilecol++) {
    body = obj->Bodies[cartesian_trajectory_planner_B.b_jtilecol];
    cartesian_trajectory_planner_B.n = body->JointInternal.PositionNumber;
    cartesian_trajectory_planner_B.n += cartesian_trajectory_planner_B.k;
    if (cartesian_trajectory_planner_B.k > cartesian_trajectory_planner_B.n -
        1.0) {
      cartesian_trajectory_planner_B.e_kb = 0;
      cartesian_trajectory_planner_B.d_l = 0;
    } else {
      cartesian_trajectory_planner_B.e_kb = static_cast<int32_T>
        (cartesian_trajectory_planner_B.k) - 1;
      cartesian_trajectory_planner_B.d_l = static_cast<int32_T>
        (cartesian_trajectory_planner_B.n - 1.0);
    }

    cartesian_trajectory_planner_B.b_kstr_d = switch_expression->size[0] *
      switch_expression->size[1];
    switch_expression->size[0] = 1;
    switch_expression->size[1] = body->JointInternal.Type->size[1];
    cartes_emxEnsureCapacity_char_T(switch_expression,
      cartesian_trajectory_planner_B.b_kstr_d);
    cartesian_trajectory_planner_B.loop_ub_n = body->JointInternal.Type->size[0]
      * body->JointInternal.Type->size[1] - 1;
    for (cartesian_trajectory_planner_B.b_kstr_d = 0;
         cartesian_trajectory_planner_B.b_kstr_d <=
         cartesian_trajectory_planner_B.loop_ub_n;
         cartesian_trajectory_planner_B.b_kstr_d++) {
      switch_expression->data[cartesian_trajectory_planner_B.b_kstr_d] =
        body->JointInternal.Type->data[cartesian_trajectory_planner_B.b_kstr_d];
    }

    cartesian_trajectory_planner_B.b_bool_an = false;
    if (switch_expression->size[1] == 5) {
      cartesian_trajectory_planner_B.b_kstr_d = 1;
      do {
        exitg1 = 0;
        if (cartesian_trajectory_planner_B.b_kstr_d - 1 < 5) {
          cartesian_trajectory_planner_B.loop_ub_n =
            cartesian_trajectory_planner_B.b_kstr_d - 1;
          if (switch_expression->data[cartesian_trajectory_planner_B.loop_ub_n]
              !=
              cartesian_trajectory_planner_B.b_oo[cartesian_trajectory_planner_B.loop_ub_n])
          {
            exitg1 = 1;
          } else {
            cartesian_trajectory_planner_B.b_kstr_d++;
          }
        } else {
          cartesian_trajectory_planner_B.b_bool_an = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (cartesian_trajectory_planner_B.b_bool_an) {
      cartesian_trajectory_planner_B.b_kstr_d = 0;
    } else {
      for (cartesian_trajectory_planner_B.b_kstr_d = 0;
           cartesian_trajectory_planner_B.b_kstr_d < 8;
           cartesian_trajectory_planner_B.b_kstr_d++) {
        cartesian_trajectory_planner_B.b_dz[cartesian_trajectory_planner_B.b_kstr_d]
          = tmp_1[cartesian_trajectory_planner_B.b_kstr_d];
      }

      cartesian_trajectory_planner_B.b_bool_an = false;
      if (switch_expression->size[1] == 8) {
        cartesian_trajectory_planner_B.b_kstr_d = 1;
        do {
          exitg1 = 0;
          if (cartesian_trajectory_planner_B.b_kstr_d - 1 < 8) {
            cartesian_trajectory_planner_B.loop_ub_n =
              cartesian_trajectory_planner_B.b_kstr_d - 1;
            if (switch_expression->data[cartesian_trajectory_planner_B.loop_ub_n]
                !=
                cartesian_trajectory_planner_B.b_dz[cartesian_trajectory_planner_B.loop_ub_n])
            {
              exitg1 = 1;
            } else {
              cartesian_trajectory_planner_B.b_kstr_d++;
            }
          } else {
            cartesian_trajectory_planner_B.b_bool_an = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (cartesian_trajectory_planner_B.b_bool_an) {
        cartesian_trajectory_planner_B.b_kstr_d = 1;
      } else {
        cartesian_trajectory_planner_B.b_kstr_d = -1;
      }
    }

    switch (cartesian_trajectory_planner_B.b_kstr_d) {
     case 0:
      memset(&cartesian_trajectory_planner_B.c_f1[0], 0, sizeof(real_T) << 4U);
      cartesian_trajectory_planner_B.c_f1[0] = 1.0;
      cartesian_trajectory_planner_B.c_f1[5] = 1.0;
      cartesian_trajectory_planner_B.c_f1[10] = 1.0;
      cartesian_trajectory_planner_B.c_f1[15] = 1.0;
      break;

     case 1:
      ca_rigidBodyJoint_get_JointAxis(&body->JointInternal,
        cartesian_trajectory_planner_B.v_g);
      cartesian_trajectory_planner_B.d_l -= cartesian_trajectory_planner_B.e_kb;
      for (cartesian_trajectory_planner_B.b_kstr_d = 0;
           cartesian_trajectory_planner_B.b_kstr_d <
           cartesian_trajectory_planner_B.d_l;
           cartesian_trajectory_planner_B.b_kstr_d++) {
        cartesian_trajectory_planner_B.e_data[cartesian_trajectory_planner_B.b_kstr_d]
          = cartesian_trajectory_planner_B.e_kb +
          cartesian_trajectory_planner_B.b_kstr_d;
      }

      cartesian_trajectory_planner_B.result_data_p[0] =
        cartesian_trajectory_planner_B.v_g[0];
      cartesian_trajectory_planner_B.result_data_p[1] =
        cartesian_trajectory_planner_B.v_g[1];
      cartesian_trajectory_planner_B.result_data_p[2] =
        cartesian_trajectory_planner_B.v_g[2];
      if (0 <= (cartesian_trajectory_planner_B.d_l != 0) - 1) {
        cartesian_trajectory_planner_B.result_data_p[3] =
          qvec[cartesian_trajectory_planner_B.e_data[0]];
      }

      cartesian_trajectory_planner_B.k = 1.0 / sqrt
        ((cartesian_trajectory_planner_B.result_data_p[0] *
          cartesian_trajectory_planner_B.result_data_p[0] +
          cartesian_trajectory_planner_B.result_data_p[1] *
          cartesian_trajectory_planner_B.result_data_p[1]) +
         cartesian_trajectory_planner_B.result_data_p[2] *
         cartesian_trajectory_planner_B.result_data_p[2]);
      cartesian_trajectory_planner_B.v_g[0] =
        cartesian_trajectory_planner_B.result_data_p[0] *
        cartesian_trajectory_planner_B.k;
      cartesian_trajectory_planner_B.v_g[1] =
        cartesian_trajectory_planner_B.result_data_p[1] *
        cartesian_trajectory_planner_B.k;
      cartesian_trajectory_planner_B.v_g[2] =
        cartesian_trajectory_planner_B.result_data_p[2] *
        cartesian_trajectory_planner_B.k;
      cartesian_trajectory_planner_B.k = cos
        (cartesian_trajectory_planner_B.result_data_p[3]);
      cartesian_trajectory_planner_B.sth_p = sin
        (cartesian_trajectory_planner_B.result_data_p[3]);
      cartesian_trajectory_planner_B.tempR_e[0] =
        cartesian_trajectory_planner_B.v_g[0] *
        cartesian_trajectory_planner_B.v_g[0] * (1.0 -
        cartesian_trajectory_planner_B.k) + cartesian_trajectory_planner_B.k;
      cartesian_trajectory_planner_B.tempR_tmp_o =
        cartesian_trajectory_planner_B.v_g[1] *
        cartesian_trajectory_planner_B.v_g[0] * (1.0 -
        cartesian_trajectory_planner_B.k);
      cartesian_trajectory_planner_B.tempR_tmp_lm =
        cartesian_trajectory_planner_B.v_g[2] *
        cartesian_trajectory_planner_B.sth_p;
      cartesian_trajectory_planner_B.tempR_e[1] =
        cartesian_trajectory_planner_B.tempR_tmp_o -
        cartesian_trajectory_planner_B.tempR_tmp_lm;
      cartesian_trajectory_planner_B.tempR_tmp_k =
        cartesian_trajectory_planner_B.v_g[2] *
        cartesian_trajectory_planner_B.v_g[0] * (1.0 -
        cartesian_trajectory_planner_B.k);
      cartesian_trajectory_planner_B.tempR_tmp_j =
        cartesian_trajectory_planner_B.v_g[1] *
        cartesian_trajectory_planner_B.sth_p;
      cartesian_trajectory_planner_B.tempR_e[2] =
        cartesian_trajectory_planner_B.tempR_tmp_k +
        cartesian_trajectory_planner_B.tempR_tmp_j;
      cartesian_trajectory_planner_B.tempR_e[3] =
        cartesian_trajectory_planner_B.tempR_tmp_o +
        cartesian_trajectory_planner_B.tempR_tmp_lm;
      cartesian_trajectory_planner_B.tempR_e[4] =
        cartesian_trajectory_planner_B.v_g[1] *
        cartesian_trajectory_planner_B.v_g[1] * (1.0 -
        cartesian_trajectory_planner_B.k) + cartesian_trajectory_planner_B.k;
      cartesian_trajectory_planner_B.tempR_tmp_o =
        cartesian_trajectory_planner_B.v_g[2] *
        cartesian_trajectory_planner_B.v_g[1] * (1.0 -
        cartesian_trajectory_planner_B.k);
      cartesian_trajectory_planner_B.tempR_tmp_lm =
        cartesian_trajectory_planner_B.v_g[0] *
        cartesian_trajectory_planner_B.sth_p;
      cartesian_trajectory_planner_B.tempR_e[5] =
        cartesian_trajectory_planner_B.tempR_tmp_o -
        cartesian_trajectory_planner_B.tempR_tmp_lm;
      cartesian_trajectory_planner_B.tempR_e[6] =
        cartesian_trajectory_planner_B.tempR_tmp_k -
        cartesian_trajectory_planner_B.tempR_tmp_j;
      cartesian_trajectory_planner_B.tempR_e[7] =
        cartesian_trajectory_planner_B.tempR_tmp_o +
        cartesian_trajectory_planner_B.tempR_tmp_lm;
      cartesian_trajectory_planner_B.tempR_e[8] =
        cartesian_trajectory_planner_B.v_g[2] *
        cartesian_trajectory_planner_B.v_g[2] * (1.0 -
        cartesian_trajectory_planner_B.k) + cartesian_trajectory_planner_B.k;
      for (cartesian_trajectory_planner_B.b_kstr_d = 0;
           cartesian_trajectory_planner_B.b_kstr_d < 3;
           cartesian_trajectory_planner_B.b_kstr_d++) {
        cartesian_trajectory_planner_B.e_kb =
          cartesian_trajectory_planner_B.b_kstr_d + 1;
        cartesian_trajectory_planner_B.R_p[cartesian_trajectory_planner_B.e_kb -
          1] = cartesian_trajectory_planner_B.tempR_e
          [(cartesian_trajectory_planner_B.e_kb - 1) * 3];
        cartesian_trajectory_planner_B.e_kb =
          cartesian_trajectory_planner_B.b_kstr_d + 1;
        cartesian_trajectory_planner_B.R_p[cartesian_trajectory_planner_B.e_kb +
          2] = cartesian_trajectory_planner_B.tempR_e
          [(cartesian_trajectory_planner_B.e_kb - 1) * 3 + 1];
        cartesian_trajectory_planner_B.e_kb =
          cartesian_trajectory_planner_B.b_kstr_d + 1;
        cartesian_trajectory_planner_B.R_p[cartesian_trajectory_planner_B.e_kb +
          5] = cartesian_trajectory_planner_B.tempR_e
          [(cartesian_trajectory_planner_B.e_kb - 1) * 3 + 2];
      }

      memset(&cartesian_trajectory_planner_B.c_f1[0], 0, sizeof(real_T) << 4U);
      for (cartesian_trajectory_planner_B.b_kstr_d = 0;
           cartesian_trajectory_planner_B.b_kstr_d < 3;
           cartesian_trajectory_planner_B.b_kstr_d++) {
        cartesian_trajectory_planner_B.d_l =
          cartesian_trajectory_planner_B.b_kstr_d << 2;
        cartesian_trajectory_planner_B.c_f1[cartesian_trajectory_planner_B.d_l] =
          cartesian_trajectory_planner_B.R_p[3 *
          cartesian_trajectory_planner_B.b_kstr_d];
        cartesian_trajectory_planner_B.c_f1[cartesian_trajectory_planner_B.d_l +
          1] = cartesian_trajectory_planner_B.R_p[3 *
          cartesian_trajectory_planner_B.b_kstr_d + 1];
        cartesian_trajectory_planner_B.c_f1[cartesian_trajectory_planner_B.d_l +
          2] = cartesian_trajectory_planner_B.R_p[3 *
          cartesian_trajectory_planner_B.b_kstr_d + 2];
      }

      cartesian_trajectory_planner_B.c_f1[15] = 1.0;
      break;

     default:
      ca_rigidBodyJoint_get_JointAxis(&body->JointInternal,
        cartesian_trajectory_planner_B.v_g);
      memset(&cartesian_trajectory_planner_B.tempR_e[0], 0, 9U * sizeof(real_T));
      cartesian_trajectory_planner_B.tempR_e[0] = 1.0;
      cartesian_trajectory_planner_B.tempR_e[4] = 1.0;
      cartesian_trajectory_planner_B.tempR_e[8] = 1.0;
      for (cartesian_trajectory_planner_B.b_kstr_d = 0;
           cartesian_trajectory_planner_B.b_kstr_d < 3;
           cartesian_trajectory_planner_B.b_kstr_d++) {
        cartesian_trajectory_planner_B.d_l =
          cartesian_trajectory_planner_B.b_kstr_d << 2;
        cartesian_trajectory_planner_B.c_f1[cartesian_trajectory_planner_B.d_l] =
          cartesian_trajectory_planner_B.tempR_e[3 *
          cartesian_trajectory_planner_B.b_kstr_d];
        cartesian_trajectory_planner_B.c_f1[cartesian_trajectory_planner_B.d_l +
          1] = cartesian_trajectory_planner_B.tempR_e[3 *
          cartesian_trajectory_planner_B.b_kstr_d + 1];
        cartesian_trajectory_planner_B.c_f1[cartesian_trajectory_planner_B.d_l +
          2] = cartesian_trajectory_planner_B.tempR_e[3 *
          cartesian_trajectory_planner_B.b_kstr_d + 2];
        cartesian_trajectory_planner_B.c_f1[cartesian_trajectory_planner_B.b_kstr_d
          + 12] =
          cartesian_trajectory_planner_B.v_g[cartesian_trajectory_planner_B.b_kstr_d]
          * qvec[cartesian_trajectory_planner_B.e_kb];
      }

      cartesian_trajectory_planner_B.c_f1[3] = 0.0;
      cartesian_trajectory_planner_B.c_f1[7] = 0.0;
      cartesian_trajectory_planner_B.c_f1[11] = 0.0;
      cartesian_trajectory_planner_B.c_f1[15] = 1.0;
      break;
    }

    for (cartesian_trajectory_planner_B.b_kstr_d = 0;
         cartesian_trajectory_planner_B.b_kstr_d < 16;
         cartesian_trajectory_planner_B.b_kstr_d++) {
      cartesian_trajectory_planner_B.a_o[cartesian_trajectory_planner_B.b_kstr_d]
        = body->
        JointInternal.JointToParentTransform[cartesian_trajectory_planner_B.b_kstr_d];
    }

    for (cartesian_trajectory_planner_B.b_kstr_d = 0;
         cartesian_trajectory_planner_B.b_kstr_d < 16;
         cartesian_trajectory_planner_B.b_kstr_d++) {
      cartesian_trajectory_planner_B.b_b[cartesian_trajectory_planner_B.b_kstr_d]
        = body->
        JointInternal.ChildToJointTransform[cartesian_trajectory_planner_B.b_kstr_d];
    }

    for (cartesian_trajectory_planner_B.b_kstr_d = 0;
         cartesian_trajectory_planner_B.b_kstr_d < 4;
         cartesian_trajectory_planner_B.b_kstr_d++) {
      for (cartesian_trajectory_planner_B.e_kb = 0;
           cartesian_trajectory_planner_B.e_kb < 4;
           cartesian_trajectory_planner_B.e_kb++) {
        cartesian_trajectory_planner_B.d_l = cartesian_trajectory_planner_B.e_kb
          << 2;
        cartesian_trajectory_planner_B.loop_ub_n =
          cartesian_trajectory_planner_B.b_kstr_d +
          cartesian_trajectory_planner_B.d_l;
        cartesian_trajectory_planner_B.a_n[cartesian_trajectory_planner_B.loop_ub_n]
          = 0.0;
        cartesian_trajectory_planner_B.a_n[cartesian_trajectory_planner_B.loop_ub_n]
          +=
          cartesian_trajectory_planner_B.c_f1[cartesian_trajectory_planner_B.d_l]
          * cartesian_trajectory_planner_B.a_o[cartesian_trajectory_planner_B.b_kstr_d];
        cartesian_trajectory_planner_B.a_n[cartesian_trajectory_planner_B.loop_ub_n]
          +=
          cartesian_trajectory_planner_B.c_f1[cartesian_trajectory_planner_B.d_l
          + 1] *
          cartesian_trajectory_planner_B.a_o[cartesian_trajectory_planner_B.b_kstr_d
          + 4];
        cartesian_trajectory_planner_B.a_n[cartesian_trajectory_planner_B.loop_ub_n]
          +=
          cartesian_trajectory_planner_B.c_f1[cartesian_trajectory_planner_B.d_l
          + 2] *
          cartesian_trajectory_planner_B.a_o[cartesian_trajectory_planner_B.b_kstr_d
          + 8];
        cartesian_trajectory_planner_B.a_n[cartesian_trajectory_planner_B.loop_ub_n]
          +=
          cartesian_trajectory_planner_B.c_f1[cartesian_trajectory_planner_B.d_l
          + 3] *
          cartesian_trajectory_planner_B.a_o[cartesian_trajectory_planner_B.b_kstr_d
          + 12];
      }

      for (cartesian_trajectory_planner_B.e_kb = 0;
           cartesian_trajectory_planner_B.e_kb < 4;
           cartesian_trajectory_planner_B.e_kb++) {
        cartesian_trajectory_planner_B.d_l = cartesian_trajectory_planner_B.e_kb
          << 2;
        cartesian_trajectory_planner_B.loop_ub_n =
          cartesian_trajectory_planner_B.b_kstr_d +
          cartesian_trajectory_planner_B.d_l;
        Ttree->data[cartesian_trajectory_planner_B.b_jtilecol]
          .f1[cartesian_trajectory_planner_B.loop_ub_n] = 0.0;
        Ttree->data[cartesian_trajectory_planner_B.b_jtilecol]
          .f1[cartesian_trajectory_planner_B.loop_ub_n] +=
          cartesian_trajectory_planner_B.b_b[cartesian_trajectory_planner_B.d_l]
          * cartesian_trajectory_planner_B.a_n[cartesian_trajectory_planner_B.b_kstr_d];
        Ttree->data[cartesian_trajectory_planner_B.b_jtilecol]
          .f1[cartesian_trajectory_planner_B.loop_ub_n] +=
          cartesian_trajectory_planner_B.b_b[cartesian_trajectory_planner_B.d_l
          + 1] *
          cartesian_trajectory_planner_B.a_n[cartesian_trajectory_planner_B.b_kstr_d
          + 4];
        Ttree->data[cartesian_trajectory_planner_B.b_jtilecol]
          .f1[cartesian_trajectory_planner_B.loop_ub_n] +=
          cartesian_trajectory_planner_B.b_b[cartesian_trajectory_planner_B.d_l
          + 2] *
          cartesian_trajectory_planner_B.a_n[cartesian_trajectory_planner_B.b_kstr_d
          + 8];
        Ttree->data[cartesian_trajectory_planner_B.b_jtilecol]
          .f1[cartesian_trajectory_planner_B.loop_ub_n] +=
          cartesian_trajectory_planner_B.b_b[cartesian_trajectory_planner_B.d_l
          + 3] *
          cartesian_trajectory_planner_B.a_n[cartesian_trajectory_planner_B.b_kstr_d
          + 12];
      }
    }

    cartesian_trajectory_planner_B.k = cartesian_trajectory_planner_B.n;
    if (body->ParentIndex > 0.0) {
      for (cartesian_trajectory_planner_B.b_kstr_d = 0;
           cartesian_trajectory_planner_B.b_kstr_d < 16;
           cartesian_trajectory_planner_B.b_kstr_d++) {
        cartesian_trajectory_planner_B.a_o[cartesian_trajectory_planner_B.b_kstr_d]
          = Ttree->data[static_cast<int32_T>(body->ParentIndex) - 1]
          .f1[cartesian_trajectory_planner_B.b_kstr_d];
      }

      for (cartesian_trajectory_planner_B.b_kstr_d = 0;
           cartesian_trajectory_planner_B.b_kstr_d < 4;
           cartesian_trajectory_planner_B.b_kstr_d++) {
        for (cartesian_trajectory_planner_B.e_kb = 0;
             cartesian_trajectory_planner_B.e_kb < 4;
             cartesian_trajectory_planner_B.e_kb++) {
          cartesian_trajectory_planner_B.d_l =
            cartesian_trajectory_planner_B.e_kb << 2;
          cartesian_trajectory_planner_B.loop_ub_n =
            cartesian_trajectory_planner_B.b_kstr_d +
            cartesian_trajectory_planner_B.d_l;
          cartesian_trajectory_planner_B.a_n[cartesian_trajectory_planner_B.loop_ub_n]
            = 0.0;
          cartesian_trajectory_planner_B.a_n[cartesian_trajectory_planner_B.loop_ub_n]
            += Ttree->data[cartesian_trajectory_planner_B.b_jtilecol]
            .f1[cartesian_trajectory_planner_B.d_l] *
            cartesian_trajectory_planner_B.a_o[cartesian_trajectory_planner_B.b_kstr_d];
          cartesian_trajectory_planner_B.a_n[cartesian_trajectory_planner_B.loop_ub_n]
            += Ttree->data[cartesian_trajectory_planner_B.b_jtilecol]
            .f1[cartesian_trajectory_planner_B.d_l + 1] *
            cartesian_trajectory_planner_B.a_o[cartesian_trajectory_planner_B.b_kstr_d
            + 4];
          cartesian_trajectory_planner_B.a_n[cartesian_trajectory_planner_B.loop_ub_n]
            += Ttree->data[cartesian_trajectory_planner_B.b_jtilecol]
            .f1[cartesian_trajectory_planner_B.d_l + 2] *
            cartesian_trajectory_planner_B.a_o[cartesian_trajectory_planner_B.b_kstr_d
            + 8];
          cartesian_trajectory_planner_B.a_n[cartesian_trajectory_planner_B.loop_ub_n]
            += Ttree->data[cartesian_trajectory_planner_B.b_jtilecol]
            .f1[cartesian_trajectory_planner_B.d_l + 3] *
            cartesian_trajectory_planner_B.a_o[cartesian_trajectory_planner_B.b_kstr_d
            + 12];
        }
      }

      for (cartesian_trajectory_planner_B.b_kstr_d = 0;
           cartesian_trajectory_planner_B.b_kstr_d < 16;
           cartesian_trajectory_planner_B.b_kstr_d++) {
        Ttree->data[cartesian_trajectory_planner_B.b_jtilecol]
          .f1[cartesian_trajectory_planner_B.b_kstr_d] =
          cartesian_trajectory_planner_B.a_n[cartesian_trajectory_planner_B.b_kstr_d];
      }
    }
  }

  cartesian_trajec_emxFree_char_T(&switch_expression);
}

static void cartesian_t_emxFree_f_cell_wrap(emxArray_f_cell_wrap_cartesia_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_f_cell_wrap_cartesia_T *)NULL) {
    if (((*pEmxArray)->data != (f_cell_wrap_cartesian_traject_T *)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_f_cell_wrap_cartesia_T *)NULL;
  }
}

static void RigidBodyTree_geometricJacobian(p_robotics_manip_internal_Rig_T *obj,
  const real_T Q[6], emxArray_real_T_cartesian_tra_T *Jac)
{
  emxArray_f_cell_wrap_cartesia_T *Ttree;
  n_robotics_manip_internal_Rig_T *body;
  emxArray_real_T_cartesian_tra_T *JacSlice;
  emxArray_char_T_cartesian_tra_T *bname;
  emxArray_real_T_cartesian_tra_T *b;
  static const char_T tmp[11] = { 'e', 'd', 'o', '_', 'l', 'i', 'n', 'k', '_',
    'e', 'e' };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  int32_T exitg1;
  boolean_T exitg2;
  cartesian_t_emxInit_f_cell_wrap(&Ttree, 2);
  RigidBodyTree_forwardKinematics(obj, Q, Ttree);
  cartesian_trajectory_planner_B.b_kstr_h = Jac->size[0] * Jac->size[1];
  Jac->size[0] = 6;
  Jac->size[1] = static_cast<int32_T>(obj->VelocityNumber);
  cartes_emxEnsureCapacity_real_T(Jac, cartesian_trajectory_planner_B.b_kstr_h);
  cartesian_trajectory_planner_B.loop_ub_g3 = 6 * static_cast<int32_T>
    (obj->VelocityNumber) - 1;
  for (cartesian_trajectory_planner_B.b_kstr_h = 0;
       cartesian_trajectory_planner_B.b_kstr_h <=
       cartesian_trajectory_planner_B.loop_ub_g3;
       cartesian_trajectory_planner_B.b_kstr_h++) {
    Jac->data[cartesian_trajectory_planner_B.b_kstr_h] = 0.0;
  }

  for (cartesian_trajectory_planner_B.b_kstr_h = 0;
       cartesian_trajectory_planner_B.b_kstr_h < 8;
       cartesian_trajectory_planner_B.b_kstr_h++) {
    cartesian_trajectory_planner_B.chainmask[cartesian_trajectory_planner_B.b_kstr_h]
      = 0;
  }

  cartesian_trajec_emxInit_char_T(&bname, 2);
  cartesian_trajectory_planner_B.b_kstr_h = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj->Base.NameInternal->size[1];
  cartes_emxEnsureCapacity_char_T(bname, cartesian_trajectory_planner_B.b_kstr_h);
  cartesian_trajectory_planner_B.loop_ub_g3 = obj->Base.NameInternal->size[0] *
    obj->Base.NameInternal->size[1] - 1;
  for (cartesian_trajectory_planner_B.b_kstr_h = 0;
       cartesian_trajectory_planner_B.b_kstr_h <=
       cartesian_trajectory_planner_B.loop_ub_g3;
       cartesian_trajectory_planner_B.b_kstr_h++) {
    bname->data[cartesian_trajectory_planner_B.b_kstr_h] =
      obj->Base.NameInternal->data[cartesian_trajectory_planner_B.b_kstr_h];
  }

  for (cartesian_trajectory_planner_B.b_kstr_h = 0;
       cartesian_trajectory_planner_B.b_kstr_h < 11;
       cartesian_trajectory_planner_B.b_kstr_h++) {
    cartesian_trajectory_planner_B.a_p2[cartesian_trajectory_planner_B.b_kstr_h]
      = tmp[cartesian_trajectory_planner_B.b_kstr_h];
  }

  cartesian_trajectory_planner_B.b_bool_b = false;
  if (11 == bname->size[1]) {
    cartesian_trajectory_planner_B.b_kstr_h = 1;
    do {
      exitg1 = 0;
      if (cartesian_trajectory_planner_B.b_kstr_h - 1 < 11) {
        cartesian_trajectory_planner_B.kstr =
          cartesian_trajectory_planner_B.b_kstr_h - 1;
        if (cartesian_trajectory_planner_B.a_p2[cartesian_trajectory_planner_B.kstr]
            != bname->data[cartesian_trajectory_planner_B.kstr]) {
          exitg1 = 1;
        } else {
          cartesian_trajectory_planner_B.b_kstr_h++;
        }
      } else {
        cartesian_trajectory_planner_B.b_bool_b = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (cartesian_trajectory_planner_B.b_bool_b) {
    memset(&cartesian_trajectory_planner_B.T2inv[0], 0, sizeof(real_T) << 4U);
    cartesian_trajectory_planner_B.T2inv[0] = 1.0;
    cartesian_trajectory_planner_B.T2inv[5] = 1.0;
    cartesian_trajectory_planner_B.T2inv[10] = 1.0;
    cartesian_trajectory_planner_B.T2inv[15] = 1.0;
    memset(&cartesian_trajectory_planner_B.T2[0], 0, sizeof(real_T) << 4U);
    cartesian_trajectory_planner_B.T2[0] = 1.0;
    cartesian_trajectory_planner_B.T2[5] = 1.0;
    cartesian_trajectory_planner_B.T2[10] = 1.0;
    cartesian_trajectory_planner_B.T2[15] = 1.0;
  } else {
    cartesian_trajectory_planner_B.endeffectorIndex = -1.0;
    cartesian_trajectory_planner_B.b_kstr_h = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = obj->Base.NameInternal->size[1];
    cartes_emxEnsureCapacity_char_T(bname,
      cartesian_trajectory_planner_B.b_kstr_h);
    cartesian_trajectory_planner_B.loop_ub_g3 = obj->Base.NameInternal->size[0] *
      obj->Base.NameInternal->size[1] - 1;
    for (cartesian_trajectory_planner_B.b_kstr_h = 0;
         cartesian_trajectory_planner_B.b_kstr_h <=
         cartesian_trajectory_planner_B.loop_ub_g3;
         cartesian_trajectory_planner_B.b_kstr_h++) {
      bname->data[cartesian_trajectory_planner_B.b_kstr_h] =
        obj->Base.NameInternal->data[cartesian_trajectory_planner_B.b_kstr_h];
    }

    cartesian_trajectory_planner_B.b_bool_b = false;
    if (bname->size[1] == 11) {
      cartesian_trajectory_planner_B.b_kstr_h = 1;
      do {
        exitg1 = 0;
        if (cartesian_trajectory_planner_B.b_kstr_h - 1 < 11) {
          cartesian_trajectory_planner_B.kstr =
            cartesian_trajectory_planner_B.b_kstr_h - 1;
          if (bname->data[cartesian_trajectory_planner_B.kstr] !=
              cartesian_trajectory_planner_B.a_p2[cartesian_trajectory_planner_B.kstr])
          {
            exitg1 = 1;
          } else {
            cartesian_trajectory_planner_B.b_kstr_h++;
          }
        } else {
          cartesian_trajectory_planner_B.b_bool_b = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (cartesian_trajectory_planner_B.b_bool_b) {
      cartesian_trajectory_planner_B.endeffectorIndex = 0.0;
    } else {
      cartesian_trajectory_planner_B.idx_idx_1 = obj->NumBodies;
      cartesian_trajectory_planner_B.b_i_oc = 0;
      exitg2 = false;
      while ((!exitg2) && (cartesian_trajectory_planner_B.b_i_oc <= static_cast<
                           int32_T>(cartesian_trajectory_planner_B.idx_idx_1) -
                           1)) {
        body = obj->Bodies[cartesian_trajectory_planner_B.b_i_oc];
        cartesian_trajectory_planner_B.b_kstr_h = bname->size[0] * bname->size[1];
        bname->size[0] = 1;
        bname->size[1] = body->NameInternal->size[1];
        cartes_emxEnsureCapacity_char_T(bname,
          cartesian_trajectory_planner_B.b_kstr_h);
        cartesian_trajectory_planner_B.loop_ub_g3 = body->NameInternal->size[0] *
          body->NameInternal->size[1] - 1;
        for (cartesian_trajectory_planner_B.b_kstr_h = 0;
             cartesian_trajectory_planner_B.b_kstr_h <=
             cartesian_trajectory_planner_B.loop_ub_g3;
             cartesian_trajectory_planner_B.b_kstr_h++) {
          bname->data[cartesian_trajectory_planner_B.b_kstr_h] =
            body->NameInternal->data[cartesian_trajectory_planner_B.b_kstr_h];
        }

        for (cartesian_trajectory_planner_B.b_kstr_h = 0;
             cartesian_trajectory_planner_B.b_kstr_h < 11;
             cartesian_trajectory_planner_B.b_kstr_h++) {
          cartesian_trajectory_planner_B.a_p2[cartesian_trajectory_planner_B.b_kstr_h]
            = tmp[cartesian_trajectory_planner_B.b_kstr_h];
        }

        cartesian_trajectory_planner_B.b_bool_b = false;
        if (bname->size[1] == 11) {
          cartesian_trajectory_planner_B.b_kstr_h = 1;
          do {
            exitg1 = 0;
            if (cartesian_trajectory_planner_B.b_kstr_h - 1 < 11) {
              cartesian_trajectory_planner_B.kstr =
                cartesian_trajectory_planner_B.b_kstr_h - 1;
              if (bname->data[cartesian_trajectory_planner_B.kstr] !=
                  cartesian_trajectory_planner_B.a_p2[cartesian_trajectory_planner_B.kstr])
              {
                exitg1 = 1;
              } else {
                cartesian_trajectory_planner_B.b_kstr_h++;
              }
            } else {
              cartesian_trajectory_planner_B.b_bool_b = true;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }

        if (cartesian_trajectory_planner_B.b_bool_b) {
          cartesian_trajectory_planner_B.endeffectorIndex = static_cast<real_T>
            (cartesian_trajectory_planner_B.b_i_oc) + 1.0;
          exitg2 = true;
        } else {
          cartesian_trajectory_planner_B.b_i_oc++;
        }
      }
    }

    cartesian_trajectory_planner_B.b_i_oc = static_cast<int32_T>
      (cartesian_trajectory_planner_B.endeffectorIndex) - 1;
    body = obj->Bodies[cartesian_trajectory_planner_B.b_i_oc];
    for (cartesian_trajectory_planner_B.b_kstr_h = 0;
         cartesian_trajectory_planner_B.b_kstr_h < 16;
         cartesian_trajectory_planner_B.b_kstr_h++) {
      cartesian_trajectory_planner_B.T2[cartesian_trajectory_planner_B.b_kstr_h]
        = Ttree->data[cartesian_trajectory_planner_B.b_i_oc]
        .f1[cartesian_trajectory_planner_B.b_kstr_h];
    }

    for (cartesian_trajectory_planner_B.b_kstr_h = 0;
         cartesian_trajectory_planner_B.b_kstr_h < 3;
         cartesian_trajectory_planner_B.b_kstr_h++) {
      cartesian_trajectory_planner_B.R_cn[3 *
        cartesian_trajectory_planner_B.b_kstr_h] = Ttree->
        data[cartesian_trajectory_planner_B.b_i_oc]
        .f1[cartesian_trajectory_planner_B.b_kstr_h];
      cartesian_trajectory_planner_B.R_cn[3 *
        cartesian_trajectory_planner_B.b_kstr_h + 1] = Ttree->
        data[cartesian_trajectory_planner_B.b_i_oc]
        .f1[cartesian_trajectory_planner_B.b_kstr_h + 4];
      cartesian_trajectory_planner_B.R_cn[3 *
        cartesian_trajectory_planner_B.b_kstr_h + 2] = Ttree->
        data[cartesian_trajectory_planner_B.b_i_oc]
        .f1[cartesian_trajectory_planner_B.b_kstr_h + 8];
    }

    for (cartesian_trajectory_planner_B.b_kstr_h = 0;
         cartesian_trajectory_planner_B.b_kstr_h < 9;
         cartesian_trajectory_planner_B.b_kstr_h++) {
      cartesian_trajectory_planner_B.R_f[cartesian_trajectory_planner_B.b_kstr_h]
        =
        -cartesian_trajectory_planner_B.R_cn[cartesian_trajectory_planner_B.b_kstr_h];
    }

    for (cartesian_trajectory_planner_B.b_kstr_h = 0;
         cartesian_trajectory_planner_B.b_kstr_h < 3;
         cartesian_trajectory_planner_B.b_kstr_h++) {
      cartesian_trajectory_planner_B.endeffectorIndex = Ttree->
        data[cartesian_trajectory_planner_B.b_i_oc].f1[12] *
        cartesian_trajectory_planner_B.R_f[cartesian_trajectory_planner_B.b_kstr_h];
      cartesian_trajectory_planner_B.loop_ub_g3 =
        cartesian_trajectory_planner_B.b_kstr_h << 2;
      cartesian_trajectory_planner_B.T2inv[cartesian_trajectory_planner_B.loop_ub_g3]
        = cartesian_trajectory_planner_B.R_cn[3 *
        cartesian_trajectory_planner_B.b_kstr_h];
      cartesian_trajectory_planner_B.endeffectorIndex +=
        cartesian_trajectory_planner_B.R_f[cartesian_trajectory_planner_B.b_kstr_h
        + 3] * Ttree->data[cartesian_trajectory_planner_B.b_i_oc].f1[13];
      cartesian_trajectory_planner_B.T2inv[cartesian_trajectory_planner_B.loop_ub_g3
        + 1] = cartesian_trajectory_planner_B.R_cn[3 *
        cartesian_trajectory_planner_B.b_kstr_h + 1];
      cartesian_trajectory_planner_B.endeffectorIndex +=
        cartesian_trajectory_planner_B.R_f[cartesian_trajectory_planner_B.b_kstr_h
        + 6] * Ttree->data[cartesian_trajectory_planner_B.b_i_oc].f1[14];
      cartesian_trajectory_planner_B.T2inv[cartesian_trajectory_planner_B.loop_ub_g3
        + 2] = cartesian_trajectory_planner_B.R_cn[3 *
        cartesian_trajectory_planner_B.b_kstr_h + 2];
      cartesian_trajectory_planner_B.T2inv[cartesian_trajectory_planner_B.b_kstr_h
        + 12] = cartesian_trajectory_planner_B.endeffectorIndex;
    }

    cartesian_trajectory_planner_B.T2inv[3] = 0.0;
    cartesian_trajectory_planner_B.T2inv[7] = 0.0;
    cartesian_trajectory_planner_B.T2inv[11] = 0.0;
    cartesian_trajectory_planner_B.T2inv[15] = 1.0;
    cartesian_trajectory_planner_B.chainmask[cartesian_trajectory_planner_B.b_i_oc]
      = 1;
    while (body->ParentIndex > 0.0) {
      body = obj->Bodies[static_cast<int32_T>(body->ParentIndex) - 1];
      cartesian_trajectory_planner_B.chainmask[static_cast<int32_T>(body->Index)
        - 1] = 1;
    }
  }

  cartesian_trajectory_planner_B.idx_idx_1 = obj->NumBodies;
  cartesian_trajectory_planner_B.c_h = static_cast<int32_T>
    (cartesian_trajectory_planner_B.idx_idx_1) - 1;
  cartesian_trajec_emxInit_real_T(&JacSlice, 2);
  cartesian_trajec_emxInit_real_T(&b, 2);
  if (0 <= cartesian_trajectory_planner_B.c_h) {
    for (cartesian_trajectory_planner_B.b_kstr_h = 0;
         cartesian_trajectory_planner_B.b_kstr_h < 5;
         cartesian_trajectory_planner_B.b_kstr_h++) {
      cartesian_trajectory_planner_B.b_jw[cartesian_trajectory_planner_B.b_kstr_h]
        = tmp_0[cartesian_trajectory_planner_B.b_kstr_h];
    }
  }

  for (cartesian_trajectory_planner_B.b_i_oc = 0;
       cartesian_trajectory_planner_B.b_i_oc <=
       cartesian_trajectory_planner_B.c_h; cartesian_trajectory_planner_B.b_i_oc
       ++) {
    body = obj->Bodies[cartesian_trajectory_planner_B.b_i_oc];
    cartesian_trajectory_planner_B.b_kstr_h = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = body->JointInternal.Type->size[1];
    cartes_emxEnsureCapacity_char_T(bname,
      cartesian_trajectory_planner_B.b_kstr_h);
    cartesian_trajectory_planner_B.loop_ub_g3 = body->JointInternal.Type->size[0]
      * body->JointInternal.Type->size[1] - 1;
    for (cartesian_trajectory_planner_B.b_kstr_h = 0;
         cartesian_trajectory_planner_B.b_kstr_h <=
         cartesian_trajectory_planner_B.loop_ub_g3;
         cartesian_trajectory_planner_B.b_kstr_h++) {
      bname->data[cartesian_trajectory_planner_B.b_kstr_h] =
        body->JointInternal.Type->data[cartesian_trajectory_planner_B.b_kstr_h];
    }

    cartesian_trajectory_planner_B.b_bool_b = false;
    if (bname->size[1] == 5) {
      cartesian_trajectory_planner_B.b_kstr_h = 1;
      do {
        exitg1 = 0;
        if (cartesian_trajectory_planner_B.b_kstr_h - 1 < 5) {
          cartesian_trajectory_planner_B.kstr =
            cartesian_trajectory_planner_B.b_kstr_h - 1;
          if (bname->data[cartesian_trajectory_planner_B.kstr] !=
              cartesian_trajectory_planner_B.b_jw[cartesian_trajectory_planner_B.kstr])
          {
            exitg1 = 1;
          } else {
            cartesian_trajectory_planner_B.b_kstr_h++;
          }
        } else {
          cartesian_trajectory_planner_B.b_bool_b = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if ((!cartesian_trajectory_planner_B.b_bool_b) &&
        (cartesian_trajectory_planner_B.chainmask[cartesian_trajectory_planner_B.b_i_oc]
         != 0)) {
      for (cartesian_trajectory_planner_B.b_kstr_h = 0;
           cartesian_trajectory_planner_B.b_kstr_h < 16;
           cartesian_trajectory_planner_B.b_kstr_h++) {
        cartesian_trajectory_planner_B.T1_l[cartesian_trajectory_planner_B.b_kstr_h]
          = Ttree->data[static_cast<int32_T>(body->Index) - 1]
          .f1[cartesian_trajectory_planner_B.b_kstr_h];
      }

      for (cartesian_trajectory_planner_B.b_kstr_h = 0;
           cartesian_trajectory_planner_B.b_kstr_h < 16;
           cartesian_trajectory_planner_B.b_kstr_h++) {
        cartesian_trajectory_planner_B.Tdh[cartesian_trajectory_planner_B.b_kstr_h]
          = body->
          JointInternal.ChildToJointTransform[cartesian_trajectory_planner_B.b_kstr_h];
      }

      for (cartesian_trajectory_planner_B.b_kstr_h = 0;
           cartesian_trajectory_planner_B.b_kstr_h < 3;
           cartesian_trajectory_planner_B.b_kstr_h++) {
        cartesian_trajectory_planner_B.R_cn[3 *
          cartesian_trajectory_planner_B.b_kstr_h] =
          cartesian_trajectory_planner_B.Tdh[cartesian_trajectory_planner_B.b_kstr_h];
        cartesian_trajectory_planner_B.R_cn[3 *
          cartesian_trajectory_planner_B.b_kstr_h + 1] =
          cartesian_trajectory_planner_B.Tdh[cartesian_trajectory_planner_B.b_kstr_h
          + 4];
        cartesian_trajectory_planner_B.R_cn[3 *
          cartesian_trajectory_planner_B.b_kstr_h + 2] =
          cartesian_trajectory_planner_B.Tdh[cartesian_trajectory_planner_B.b_kstr_h
          + 8];
      }

      for (cartesian_trajectory_planner_B.b_kstr_h = 0;
           cartesian_trajectory_planner_B.b_kstr_h < 9;
           cartesian_trajectory_planner_B.b_kstr_h++) {
        cartesian_trajectory_planner_B.R_f[cartesian_trajectory_planner_B.b_kstr_h]
          =
          -cartesian_trajectory_planner_B.R_cn[cartesian_trajectory_planner_B.b_kstr_h];
      }

      for (cartesian_trajectory_planner_B.b_kstr_h = 0;
           cartesian_trajectory_planner_B.b_kstr_h < 3;
           cartesian_trajectory_planner_B.b_kstr_h++) {
        cartesian_trajectory_planner_B.R_a5[cartesian_trajectory_planner_B.b_kstr_h]
          =
          cartesian_trajectory_planner_B.R_f[cartesian_trajectory_planner_B.b_kstr_h
          + 6] * cartesian_trajectory_planner_B.Tdh[14] +
          (cartesian_trajectory_planner_B.R_f[cartesian_trajectory_planner_B.b_kstr_h
           + 3] * cartesian_trajectory_planner_B.Tdh[13] +
           cartesian_trajectory_planner_B.R_f[cartesian_trajectory_planner_B.b_kstr_h]
           * cartesian_trajectory_planner_B.Tdh[12]);
      }

      for (cartesian_trajectory_planner_B.b_kstr_h = 0;
           cartesian_trajectory_planner_B.b_kstr_h < 4;
           cartesian_trajectory_planner_B.b_kstr_h++) {
        for (cartesian_trajectory_planner_B.kstr = 0;
             cartesian_trajectory_planner_B.kstr < 4;
             cartesian_trajectory_planner_B.kstr++) {
          cartesian_trajectory_planner_B.n_j =
            cartesian_trajectory_planner_B.kstr << 2;
          cartesian_trajectory_planner_B.loop_ub_g3 =
            cartesian_trajectory_planner_B.b_kstr_h +
            cartesian_trajectory_planner_B.n_j;
          cartesian_trajectory_planner_B.Tdh[cartesian_trajectory_planner_B.loop_ub_g3]
            = 0.0;
          cartesian_trajectory_planner_B.Tdh[cartesian_trajectory_planner_B.loop_ub_g3]
            +=
            cartesian_trajectory_planner_B.T1_l[cartesian_trajectory_planner_B.n_j]
            * cartesian_trajectory_planner_B.T2inv[cartesian_trajectory_planner_B.b_kstr_h];
          cartesian_trajectory_planner_B.Tdh[cartesian_trajectory_planner_B.loop_ub_g3]
            +=
            cartesian_trajectory_planner_B.T1_l[cartesian_trajectory_planner_B.n_j
            + 1] *
            cartesian_trajectory_planner_B.T2inv[cartesian_trajectory_planner_B.b_kstr_h
            + 4];
          cartesian_trajectory_planner_B.Tdh[cartesian_trajectory_planner_B.loop_ub_g3]
            +=
            cartesian_trajectory_planner_B.T1_l[cartesian_trajectory_planner_B.n_j
            + 2] *
            cartesian_trajectory_planner_B.T2inv[cartesian_trajectory_planner_B.b_kstr_h
            + 8];
          cartesian_trajectory_planner_B.Tdh[cartesian_trajectory_planner_B.loop_ub_g3]
            +=
            cartesian_trajectory_planner_B.T1_l[cartesian_trajectory_planner_B.n_j
            + 3] *
            cartesian_trajectory_planner_B.T2inv[cartesian_trajectory_planner_B.b_kstr_h
            + 12];
        }
      }

      for (cartesian_trajectory_planner_B.b_kstr_h = 0;
           cartesian_trajectory_planner_B.b_kstr_h < 3;
           cartesian_trajectory_planner_B.b_kstr_h++) {
        cartesian_trajectory_planner_B.kstr =
          cartesian_trajectory_planner_B.b_kstr_h << 2;
        cartesian_trajectory_planner_B.T1_l[cartesian_trajectory_planner_B.kstr]
          = cartesian_trajectory_planner_B.R_cn[3 *
          cartesian_trajectory_planner_B.b_kstr_h];
        cartesian_trajectory_planner_B.T1_l[cartesian_trajectory_planner_B.kstr
          + 1] = cartesian_trajectory_planner_B.R_cn[3 *
          cartesian_trajectory_planner_B.b_kstr_h + 1];
        cartesian_trajectory_planner_B.T1_l[cartesian_trajectory_planner_B.kstr
          + 2] = cartesian_trajectory_planner_B.R_cn[3 *
          cartesian_trajectory_planner_B.b_kstr_h + 2];
        cartesian_trajectory_planner_B.T1_l[cartesian_trajectory_planner_B.b_kstr_h
          + 12] =
          cartesian_trajectory_planner_B.R_a5[cartesian_trajectory_planner_B.b_kstr_h];
      }

      cartesian_trajectory_planner_B.T1_l[3] = 0.0;
      cartesian_trajectory_planner_B.T1_l[7] = 0.0;
      cartesian_trajectory_planner_B.T1_l[11] = 0.0;
      cartesian_trajectory_planner_B.T1_l[15] = 1.0;
      for (cartesian_trajectory_planner_B.b_kstr_h = 0;
           cartesian_trajectory_planner_B.b_kstr_h < 4;
           cartesian_trajectory_planner_B.b_kstr_h++) {
        for (cartesian_trajectory_planner_B.kstr = 0;
             cartesian_trajectory_planner_B.kstr < 4;
             cartesian_trajectory_planner_B.kstr++) {
          cartesian_trajectory_planner_B.loop_ub_g3 =
            cartesian_trajectory_planner_B.kstr << 2;
          cartesian_trajectory_planner_B.n_j =
            cartesian_trajectory_planner_B.b_kstr_h +
            cartesian_trajectory_planner_B.loop_ub_g3;
          cartesian_trajectory_planner_B.T[cartesian_trajectory_planner_B.n_j] =
            0.0;
          cartesian_trajectory_planner_B.T[cartesian_trajectory_planner_B.n_j] +=
            cartesian_trajectory_planner_B.T1_l[cartesian_trajectory_planner_B.loop_ub_g3]
            * cartesian_trajectory_planner_B.Tdh[cartesian_trajectory_planner_B.b_kstr_h];
          cartesian_trajectory_planner_B.T[cartesian_trajectory_planner_B.n_j] +=
            cartesian_trajectory_planner_B.T1_l[cartesian_trajectory_planner_B.loop_ub_g3
            + 1] *
            cartesian_trajectory_planner_B.Tdh[cartesian_trajectory_planner_B.b_kstr_h
            + 4];
          cartesian_trajectory_planner_B.T[cartesian_trajectory_planner_B.n_j] +=
            cartesian_trajectory_planner_B.T1_l[cartesian_trajectory_planner_B.loop_ub_g3
            + 2] *
            cartesian_trajectory_planner_B.Tdh[cartesian_trajectory_planner_B.b_kstr_h
            + 8];
          cartesian_trajectory_planner_B.T[cartesian_trajectory_planner_B.n_j] +=
            cartesian_trajectory_planner_B.T1_l[cartesian_trajectory_planner_B.loop_ub_g3
            + 3] *
            cartesian_trajectory_planner_B.Tdh[cartesian_trajectory_planner_B.b_kstr_h
            + 12];
        }
      }

      cartesian_trajectory_planner_B.endeffectorIndex = obj->
        PositionDoFMap[cartesian_trajectory_planner_B.b_i_oc];
      cartesian_trajectory_planner_B.idx_idx_1 = obj->
        PositionDoFMap[cartesian_trajectory_planner_B.b_i_oc + 8];
      cartesian_trajectory_planner_B.R_cn[0] = 0.0;
      cartesian_trajectory_planner_B.R_cn[3] =
        -cartesian_trajectory_planner_B.T[14];
      cartesian_trajectory_planner_B.R_cn[6] = cartesian_trajectory_planner_B.T
        [13];
      cartesian_trajectory_planner_B.R_cn[1] = cartesian_trajectory_planner_B.T
        [14];
      cartesian_trajectory_planner_B.R_cn[4] = 0.0;
      cartesian_trajectory_planner_B.R_cn[7] =
        -cartesian_trajectory_planner_B.T[12];
      cartesian_trajectory_planner_B.R_cn[2] =
        -cartesian_trajectory_planner_B.T[13];
      cartesian_trajectory_planner_B.R_cn[5] = cartesian_trajectory_planner_B.T
        [12];
      cartesian_trajectory_planner_B.R_cn[8] = 0.0;
      for (cartesian_trajectory_planner_B.b_kstr_h = 0;
           cartesian_trajectory_planner_B.b_kstr_h < 3;
           cartesian_trajectory_planner_B.b_kstr_h++) {
        for (cartesian_trajectory_planner_B.kstr = 0;
             cartesian_trajectory_planner_B.kstr < 3;
             cartesian_trajectory_planner_B.kstr++) {
          cartesian_trajectory_planner_B.loop_ub_g3 =
            cartesian_trajectory_planner_B.b_kstr_h + 3 *
            cartesian_trajectory_planner_B.kstr;
          cartesian_trajectory_planner_B.R_f[cartesian_trajectory_planner_B.loop_ub_g3]
            = 0.0;
          cartesian_trajectory_planner_B.n_j =
            cartesian_trajectory_planner_B.kstr << 2;
          cartesian_trajectory_planner_B.R_f[cartesian_trajectory_planner_B.loop_ub_g3]
            +=
            cartesian_trajectory_planner_B.T[cartesian_trajectory_planner_B.n_j]
            * cartesian_trajectory_planner_B.R_cn[cartesian_trajectory_planner_B.b_kstr_h];
          cartesian_trajectory_planner_B.R_f[cartesian_trajectory_planner_B.loop_ub_g3]
            +=
            cartesian_trajectory_planner_B.T[cartesian_trajectory_planner_B.n_j
            + 1] *
            cartesian_trajectory_planner_B.R_cn[cartesian_trajectory_planner_B.b_kstr_h
            + 3];
          cartesian_trajectory_planner_B.R_f[cartesian_trajectory_planner_B.loop_ub_g3]
            +=
            cartesian_trajectory_planner_B.T[cartesian_trajectory_planner_B.n_j
            + 2] *
            cartesian_trajectory_planner_B.R_cn[cartesian_trajectory_planner_B.b_kstr_h
            + 6];
          cartesian_trajectory_planner_B.X_f[cartesian_trajectory_planner_B.kstr
            + 6 * cartesian_trajectory_planner_B.b_kstr_h] =
            cartesian_trajectory_planner_B.T
            [(cartesian_trajectory_planner_B.b_kstr_h << 2) +
            cartesian_trajectory_planner_B.kstr];
          cartesian_trajectory_planner_B.X_f[cartesian_trajectory_planner_B.kstr
            + 6 * (cartesian_trajectory_planner_B.b_kstr_h + 3)] = 0.0;
        }
      }

      for (cartesian_trajectory_planner_B.b_kstr_h = 0;
           cartesian_trajectory_planner_B.b_kstr_h < 3;
           cartesian_trajectory_planner_B.b_kstr_h++) {
        cartesian_trajectory_planner_B.X_f[6 *
          cartesian_trajectory_planner_B.b_kstr_h + 3] =
          cartesian_trajectory_planner_B.R_f[3 *
          cartesian_trajectory_planner_B.b_kstr_h];
        cartesian_trajectory_planner_B.kstr =
          cartesian_trajectory_planner_B.b_kstr_h << 2;
        cartesian_trajectory_planner_B.loop_ub_g3 = 6 *
          (cartesian_trajectory_planner_B.b_kstr_h + 3);
        cartesian_trajectory_planner_B.X_f[cartesian_trajectory_planner_B.loop_ub_g3
          + 3] =
          cartesian_trajectory_planner_B.T[cartesian_trajectory_planner_B.kstr];
        cartesian_trajectory_planner_B.X_f[6 *
          cartesian_trajectory_planner_B.b_kstr_h + 4] =
          cartesian_trajectory_planner_B.R_f[3 *
          cartesian_trajectory_planner_B.b_kstr_h + 1];
        cartesian_trajectory_planner_B.X_f[cartesian_trajectory_planner_B.loop_ub_g3
          + 4] =
          cartesian_trajectory_planner_B.T[cartesian_trajectory_planner_B.kstr +
          1];
        cartesian_trajectory_planner_B.X_f[6 *
          cartesian_trajectory_planner_B.b_kstr_h + 5] =
          cartesian_trajectory_planner_B.R_f[3 *
          cartesian_trajectory_planner_B.b_kstr_h + 2];
        cartesian_trajectory_planner_B.X_f[cartesian_trajectory_planner_B.loop_ub_g3
          + 5] =
          cartesian_trajectory_planner_B.T[cartesian_trajectory_planner_B.kstr +
          2];
      }

      cartesian_trajectory_planner_B.b_kstr_h = b->size[0] * b->size[1];
      b->size[0] = 6;
      b->size[1] = body->JointInternal.MotionSubspace->size[1];
      cartes_emxEnsureCapacity_real_T(b, cartesian_trajectory_planner_B.b_kstr_h);
      cartesian_trajectory_planner_B.loop_ub_g3 =
        body->JointInternal.MotionSubspace->size[0] *
        body->JointInternal.MotionSubspace->size[1] - 1;
      for (cartesian_trajectory_planner_B.b_kstr_h = 0;
           cartesian_trajectory_planner_B.b_kstr_h <=
           cartesian_trajectory_planner_B.loop_ub_g3;
           cartesian_trajectory_planner_B.b_kstr_h++) {
        b->data[cartesian_trajectory_planner_B.b_kstr_h] =
          body->JointInternal.MotionSubspace->
          data[cartesian_trajectory_planner_B.b_kstr_h];
      }

      cartesian_trajectory_planner_B.n_j = b->size[1] - 1;
      cartesian_trajectory_planner_B.b_kstr_h = JacSlice->size[0] *
        JacSlice->size[1];
      JacSlice->size[0] = 6;
      JacSlice->size[1] = b->size[1];
      cartes_emxEnsureCapacity_real_T(JacSlice,
        cartesian_trajectory_planner_B.b_kstr_h);
      for (cartesian_trajectory_planner_B.b_kstr_h = 0;
           cartesian_trajectory_planner_B.b_kstr_h <=
           cartesian_trajectory_planner_B.n_j;
           cartesian_trajectory_planner_B.b_kstr_h++) {
        cartesian_trajectory_planner_B.coffset_tmp_j =
          cartesian_trajectory_planner_B.b_kstr_h * 6 - 1;
        for (cartesian_trajectory_planner_B.kstr = 0;
             cartesian_trajectory_planner_B.kstr < 6;
             cartesian_trajectory_planner_B.kstr++) {
          cartesian_trajectory_planner_B.s_j = 0.0;
          for (cartesian_trajectory_planner_B.loop_ub_g3 = 0;
               cartesian_trajectory_planner_B.loop_ub_g3 < 6;
               cartesian_trajectory_planner_B.loop_ub_g3++) {
            cartesian_trajectory_planner_B.s_j +=
              cartesian_trajectory_planner_B.X_f[cartesian_trajectory_planner_B.loop_ub_g3
              * 6 + cartesian_trajectory_planner_B.kstr] * b->data
              [(cartesian_trajectory_planner_B.coffset_tmp_j +
                cartesian_trajectory_planner_B.loop_ub_g3) + 1];
          }

          JacSlice->data[(cartesian_trajectory_planner_B.coffset_tmp_j +
                          cartesian_trajectory_planner_B.kstr) + 1] =
            cartesian_trajectory_planner_B.s_j;
        }
      }

      if (cartesian_trajectory_planner_B.endeffectorIndex >
          cartesian_trajectory_planner_B.idx_idx_1) {
        cartesian_trajectory_planner_B.n_j = 0;
      } else {
        cartesian_trajectory_planner_B.n_j = static_cast<int32_T>
          (cartesian_trajectory_planner_B.endeffectorIndex) - 1;
      }

      cartesian_trajectory_planner_B.loop_ub_g3 = JacSlice->size[1];
      for (cartesian_trajectory_planner_B.b_kstr_h = 0;
           cartesian_trajectory_planner_B.b_kstr_h <
           cartesian_trajectory_planner_B.loop_ub_g3;
           cartesian_trajectory_planner_B.b_kstr_h++) {
        for (cartesian_trajectory_planner_B.kstr = 0;
             cartesian_trajectory_planner_B.kstr < 6;
             cartesian_trajectory_planner_B.kstr++) {
          Jac->data[cartesian_trajectory_planner_B.kstr + 6 *
            (cartesian_trajectory_planner_B.n_j +
             cartesian_trajectory_planner_B.b_kstr_h)] = JacSlice->data[6 *
            cartesian_trajectory_planner_B.b_kstr_h +
            cartesian_trajectory_planner_B.kstr];
        }
      }
    }
  }

  cartesian_trajec_emxFree_char_T(&bname);
  cartesian_trajec_emxFree_real_T(&JacSlice);
  cartesian_t_emxFree_f_cell_wrap(&Ttree);
  for (cartesian_trajectory_planner_B.b_kstr_h = 0;
       cartesian_trajectory_planner_B.b_kstr_h < 3;
       cartesian_trajectory_planner_B.b_kstr_h++) {
    cartesian_trajectory_planner_B.b_i_oc =
      cartesian_trajectory_planner_B.b_kstr_h << 2;
    cartesian_trajectory_planner_B.X_f[6 *
      cartesian_trajectory_planner_B.b_kstr_h] =
      cartesian_trajectory_planner_B.T2[cartesian_trajectory_planner_B.b_i_oc];
    cartesian_trajectory_planner_B.kstr = 6 *
      (cartesian_trajectory_planner_B.b_kstr_h + 3);
    cartesian_trajectory_planner_B.X_f[cartesian_trajectory_planner_B.kstr] =
      0.0;
    cartesian_trajectory_planner_B.X_f[6 *
      cartesian_trajectory_planner_B.b_kstr_h + 3] = 0.0;
    cartesian_trajectory_planner_B.X_f[cartesian_trajectory_planner_B.kstr + 3] =
      cartesian_trajectory_planner_B.T2[cartesian_trajectory_planner_B.b_i_oc];
    cartesian_trajectory_planner_B.endeffectorIndex =
      cartesian_trajectory_planner_B.T2[cartesian_trajectory_planner_B.b_i_oc +
      1];
    cartesian_trajectory_planner_B.X_f[6 *
      cartesian_trajectory_planner_B.b_kstr_h + 1] =
      cartesian_trajectory_planner_B.endeffectorIndex;
    cartesian_trajectory_planner_B.X_f[cartesian_trajectory_planner_B.kstr + 1] =
      0.0;
    cartesian_trajectory_planner_B.X_f[6 *
      cartesian_trajectory_planner_B.b_kstr_h + 4] = 0.0;
    cartesian_trajectory_planner_B.X_f[cartesian_trajectory_planner_B.kstr + 4] =
      cartesian_trajectory_planner_B.endeffectorIndex;
    cartesian_trajectory_planner_B.endeffectorIndex =
      cartesian_trajectory_planner_B.T2[cartesian_trajectory_planner_B.b_i_oc +
      2];
    cartesian_trajectory_planner_B.X_f[6 *
      cartesian_trajectory_planner_B.b_kstr_h + 2] =
      cartesian_trajectory_planner_B.endeffectorIndex;
    cartesian_trajectory_planner_B.X_f[cartesian_trajectory_planner_B.kstr + 2] =
      0.0;
    cartesian_trajectory_planner_B.X_f[6 *
      cartesian_trajectory_planner_B.b_kstr_h + 5] = 0.0;
    cartesian_trajectory_planner_B.X_f[cartesian_trajectory_planner_B.kstr + 5] =
      cartesian_trajectory_planner_B.endeffectorIndex;
  }

  cartesian_trajectory_planner_B.n_j = Jac->size[1];
  cartesian_trajectory_planner_B.b_kstr_h = b->size[0] * b->size[1];
  b->size[0] = 6;
  b->size[1] = Jac->size[1];
  cartes_emxEnsureCapacity_real_T(b, cartesian_trajectory_planner_B.b_kstr_h);
  cartesian_trajectory_planner_B.loop_ub_g3 = Jac->size[0] * Jac->size[1] - 1;
  for (cartesian_trajectory_planner_B.b_kstr_h = 0;
       cartesian_trajectory_planner_B.b_kstr_h <=
       cartesian_trajectory_planner_B.loop_ub_g3;
       cartesian_trajectory_planner_B.b_kstr_h++) {
    b->data[cartesian_trajectory_planner_B.b_kstr_h] = Jac->
      data[cartesian_trajectory_planner_B.b_kstr_h];
  }

  cartesian_trajectory_planner_B.b_kstr_h = Jac->size[0] * Jac->size[1];
  Jac->size[0] = 6;
  Jac->size[1] = cartesian_trajectory_planner_B.n_j;
  cartes_emxEnsureCapacity_real_T(Jac, cartesian_trajectory_planner_B.b_kstr_h);
  for (cartesian_trajectory_planner_B.b_kstr_h = 0;
       cartesian_trajectory_planner_B.b_kstr_h <
       cartesian_trajectory_planner_B.n_j;
       cartesian_trajectory_planner_B.b_kstr_h++) {
    cartesian_trajectory_planner_B.coffset_tmp_j =
      cartesian_trajectory_planner_B.b_kstr_h * 6 - 1;
    for (cartesian_trajectory_planner_B.b_i_oc = 0;
         cartesian_trajectory_planner_B.b_i_oc < 6;
         cartesian_trajectory_planner_B.b_i_oc++) {
      cartesian_trajectory_planner_B.s_j = 0.0;
      for (cartesian_trajectory_planner_B.loop_ub_g3 = 0;
           cartesian_trajectory_planner_B.loop_ub_g3 < 6;
           cartesian_trajectory_planner_B.loop_ub_g3++) {
        cartesian_trajectory_planner_B.s_j +=
          cartesian_trajectory_planner_B.X_f[cartesian_trajectory_planner_B.loop_ub_g3
          * 6 + cartesian_trajectory_planner_B.b_i_oc] * b->data
          [(cartesian_trajectory_planner_B.coffset_tmp_j +
            cartesian_trajectory_planner_B.loop_ub_g3) + 1];
      }

      Jac->data[(cartesian_trajectory_planner_B.coffset_tmp_j +
                 cartesian_trajectory_planner_B.b_i_oc) + 1] =
        cartesian_trajectory_planner_B.s_j;
    }
  }

  cartesian_trajec_emxFree_real_T(&b);
}

static void rigidBodyJoint_get_JointAxis_a(const c_rigidBodyJoint_cartesian__a_T
  *obj, real_T ax[3])
{
  boolean_T b_bool;
  emxArray_char_T_cartesian_tra_T *a;
  int32_T loop_ub;
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  boolean_T guard1 = false;
  int32_T exitg1;
  cartesian_trajec_emxInit_char_T(&a, 2);
  cartesian_trajectory_planner_B.b_kstr_a = a->size[0] * a->size[1];
  a->size[0] = 1;
  a->size[1] = obj->Type->size[1];
  cartes_emxEnsureCapacity_char_T(a, cartesian_trajectory_planner_B.b_kstr_a);
  loop_ub = obj->Type->size[0] * obj->Type->size[1] - 1;
  for (cartesian_trajectory_planner_B.b_kstr_a = 0;
       cartesian_trajectory_planner_B.b_kstr_a <= loop_ub;
       cartesian_trajectory_planner_B.b_kstr_a++) {
    a->data[cartesian_trajectory_planner_B.b_kstr_a] = obj->Type->
      data[cartesian_trajectory_planner_B.b_kstr_a];
  }

  for (cartesian_trajectory_planner_B.b_kstr_a = 0;
       cartesian_trajectory_planner_B.b_kstr_a < 8;
       cartesian_trajectory_planner_B.b_kstr_a++) {
    cartesian_trajectory_planner_B.b_i[cartesian_trajectory_planner_B.b_kstr_a] =
      tmp[cartesian_trajectory_planner_B.b_kstr_a];
  }

  b_bool = false;
  if (a->size[1] == 8) {
    cartesian_trajectory_planner_B.b_kstr_a = 1;
    do {
      exitg1 = 0;
      if (cartesian_trajectory_planner_B.b_kstr_a - 1 < 8) {
        loop_ub = cartesian_trajectory_planner_B.b_kstr_a - 1;
        if (a->data[loop_ub] != cartesian_trajectory_planner_B.b_i[loop_ub]) {
          exitg1 = 1;
        } else {
          cartesian_trajectory_planner_B.b_kstr_a++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (b_bool) {
    guard1 = true;
  } else {
    cartesian_trajectory_planner_B.b_kstr_a = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = obj->Type->size[1];
    cartes_emxEnsureCapacity_char_T(a, cartesian_trajectory_planner_B.b_kstr_a);
    loop_ub = obj->Type->size[0] * obj->Type->size[1] - 1;
    for (cartesian_trajectory_planner_B.b_kstr_a = 0;
         cartesian_trajectory_planner_B.b_kstr_a <= loop_ub;
         cartesian_trajectory_planner_B.b_kstr_a++) {
      a->data[cartesian_trajectory_planner_B.b_kstr_a] = obj->Type->
        data[cartesian_trajectory_planner_B.b_kstr_a];
    }

    for (cartesian_trajectory_planner_B.b_kstr_a = 0;
         cartesian_trajectory_planner_B.b_kstr_a < 9;
         cartesian_trajectory_planner_B.b_kstr_a++) {
      cartesian_trajectory_planner_B.b_g[cartesian_trajectory_planner_B.b_kstr_a]
        = tmp_0[cartesian_trajectory_planner_B.b_kstr_a];
    }

    b_bool = false;
    if (a->size[1] == 9) {
      cartesian_trajectory_planner_B.b_kstr_a = 1;
      do {
        exitg1 = 0;
        if (cartesian_trajectory_planner_B.b_kstr_a - 1 < 9) {
          loop_ub = cartesian_trajectory_planner_B.b_kstr_a - 1;
          if (a->data[loop_ub] != cartesian_trajectory_planner_B.b_g[loop_ub]) {
            exitg1 = 1;
          } else {
            cartesian_trajectory_planner_B.b_kstr_a++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
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

  cartesian_trajec_emxFree_char_T(&a);
}

static void RigidBodyTree_forwardKinemati_a(p_robotics_manip_internal_R_a_T *obj,
  const real_T qvec[6], emxArray_f_cell_wrap_cartesia_T *Ttree)
{
  n_robotics_manip_internal_R_a_T *body;
  emxArray_char_T_cartesian_tra_T *switch_expression;
  boolean_T b_bool;
  static const int8_T tmp[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  cartesian_trajectory_planner_B.n_o = obj->NumBodies;
  for (cartesian_trajectory_planner_B.b_kstr_e = 0;
       cartesian_trajectory_planner_B.b_kstr_e < 16;
       cartesian_trajectory_planner_B.b_kstr_e++) {
    cartesian_trajectory_planner_B.c_f1_h[cartesian_trajectory_planner_B.b_kstr_e]
      = tmp[cartesian_trajectory_planner_B.b_kstr_e];
  }

  cartesian_trajectory_planner_B.b_kstr_e = Ttree->size[0] * Ttree->size[1];
  Ttree->size[0] = 1;
  cartesian_trajectory_planner_B.e_n = static_cast<int32_T>
    (cartesian_trajectory_planner_B.n_o);
  Ttree->size[1] = cartesian_trajectory_planner_B.e_n;
  c_emxEnsureCapacity_f_cell_wrap(Ttree, cartesian_trajectory_planner_B.b_kstr_e);
  if (cartesian_trajectory_planner_B.e_n != 0) {
    cartesian_trajectory_planner_B.ntilecols_f =
      cartesian_trajectory_planner_B.e_n - 1;
    if (0 <= cartesian_trajectory_planner_B.ntilecols_f) {
      memcpy(&cartesian_trajectory_planner_B.expl_temp_l.f1[0],
             &cartesian_trajectory_planner_B.c_f1_h[0], sizeof(real_T) << 4U);
    }

    for (cartesian_trajectory_planner_B.b_jtilecol_n = 0;
         cartesian_trajectory_planner_B.b_jtilecol_n <=
         cartesian_trajectory_planner_B.ntilecols_f;
         cartesian_trajectory_planner_B.b_jtilecol_n++) {
      Ttree->data[cartesian_trajectory_planner_B.b_jtilecol_n] =
        cartesian_trajectory_planner_B.expl_temp_l;
    }
  }

  cartesian_trajectory_planner_B.k_f = 1.0;
  cartesian_trajectory_planner_B.ntilecols_f = static_cast<int32_T>
    (cartesian_trajectory_planner_B.n_o) - 1;
  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  if (0 <= cartesian_trajectory_planner_B.ntilecols_f) {
    for (cartesian_trajectory_planner_B.b_kstr_e = 0;
         cartesian_trajectory_planner_B.b_kstr_e < 5;
         cartesian_trajectory_planner_B.b_kstr_e++) {
      cartesian_trajectory_planner_B.b_ew[cartesian_trajectory_planner_B.b_kstr_e]
        = tmp_0[cartesian_trajectory_planner_B.b_kstr_e];
    }
  }

  for (cartesian_trajectory_planner_B.b_jtilecol_n = 0;
       cartesian_trajectory_planner_B.b_jtilecol_n <=
       cartesian_trajectory_planner_B.ntilecols_f;
       cartesian_trajectory_planner_B.b_jtilecol_n++) {
    body = obj->Bodies[cartesian_trajectory_planner_B.b_jtilecol_n];
    cartesian_trajectory_planner_B.n_o = body->JointInternal.PositionNumber;
    cartesian_trajectory_planner_B.n_o += cartesian_trajectory_planner_B.k_f;
    if (cartesian_trajectory_planner_B.k_f > cartesian_trajectory_planner_B.n_o
        - 1.0) {
      cartesian_trajectory_planner_B.e_n = 0;
      cartesian_trajectory_planner_B.d_e = 0;
    } else {
      cartesian_trajectory_planner_B.e_n = static_cast<int32_T>
        (cartesian_trajectory_planner_B.k_f) - 1;
      cartesian_trajectory_planner_B.d_e = static_cast<int32_T>
        (cartesian_trajectory_planner_B.n_o - 1.0);
    }

    cartesian_trajectory_planner_B.b_kstr_e = switch_expression->size[0] *
      switch_expression->size[1];
    switch_expression->size[0] = 1;
    switch_expression->size[1] = body->JointInternal.Type->size[1];
    cartes_emxEnsureCapacity_char_T(switch_expression,
      cartesian_trajectory_planner_B.b_kstr_e);
    cartesian_trajectory_planner_B.loop_ub_by = body->JointInternal.Type->size[0]
      * body->JointInternal.Type->size[1] - 1;
    for (cartesian_trajectory_planner_B.b_kstr_e = 0;
         cartesian_trajectory_planner_B.b_kstr_e <=
         cartesian_trajectory_planner_B.loop_ub_by;
         cartesian_trajectory_planner_B.b_kstr_e++) {
      switch_expression->data[cartesian_trajectory_planner_B.b_kstr_e] =
        body->JointInternal.Type->data[cartesian_trajectory_planner_B.b_kstr_e];
    }

    b_bool = false;
    if (switch_expression->size[1] == 5) {
      cartesian_trajectory_planner_B.b_kstr_e = 1;
      do {
        exitg1 = 0;
        if (cartesian_trajectory_planner_B.b_kstr_e - 1 < 5) {
          cartesian_trajectory_planner_B.loop_ub_by =
            cartesian_trajectory_planner_B.b_kstr_e - 1;
          if (switch_expression->data[cartesian_trajectory_planner_B.loop_ub_by]
              !=
              cartesian_trajectory_planner_B.b_ew[cartesian_trajectory_planner_B.loop_ub_by])
          {
            exitg1 = 1;
          } else {
            cartesian_trajectory_planner_B.b_kstr_e++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      cartesian_trajectory_planner_B.b_kstr_e = 0;
    } else {
      for (cartesian_trajectory_planner_B.b_kstr_e = 0;
           cartesian_trajectory_planner_B.b_kstr_e < 8;
           cartesian_trajectory_planner_B.b_kstr_e++) {
        cartesian_trajectory_planner_B.b_cx[cartesian_trajectory_planner_B.b_kstr_e]
          = tmp_1[cartesian_trajectory_planner_B.b_kstr_e];
      }

      b_bool = false;
      if (switch_expression->size[1] == 8) {
        cartesian_trajectory_planner_B.b_kstr_e = 1;
        do {
          exitg1 = 0;
          if (cartesian_trajectory_planner_B.b_kstr_e - 1 < 8) {
            cartesian_trajectory_planner_B.loop_ub_by =
              cartesian_trajectory_planner_B.b_kstr_e - 1;
            if (switch_expression->
                data[cartesian_trajectory_planner_B.loop_ub_by] !=
                cartesian_trajectory_planner_B.b_cx[cartesian_trajectory_planner_B.loop_ub_by])
            {
              exitg1 = 1;
            } else {
              cartesian_trajectory_planner_B.b_kstr_e++;
            }
          } else {
            b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (b_bool) {
        cartesian_trajectory_planner_B.b_kstr_e = 1;
      } else {
        cartesian_trajectory_planner_B.b_kstr_e = -1;
      }
    }

    switch (cartesian_trajectory_planner_B.b_kstr_e) {
     case 0:
      memset(&cartesian_trajectory_planner_B.c_f1_h[0], 0, sizeof(real_T) << 4U);
      cartesian_trajectory_planner_B.c_f1_h[0] = 1.0;
      cartesian_trajectory_planner_B.c_f1_h[5] = 1.0;
      cartesian_trajectory_planner_B.c_f1_h[10] = 1.0;
      cartesian_trajectory_planner_B.c_f1_h[15] = 1.0;
      break;

     case 1:
      rigidBodyJoint_get_JointAxis_a(&body->JointInternal,
        cartesian_trajectory_planner_B.v_na);
      cartesian_trajectory_planner_B.d_e -= cartesian_trajectory_planner_B.e_n;
      for (cartesian_trajectory_planner_B.b_kstr_e = 0;
           cartesian_trajectory_planner_B.b_kstr_e <
           cartesian_trajectory_planner_B.d_e;
           cartesian_trajectory_planner_B.b_kstr_e++) {
        cartesian_trajectory_planner_B.e_data_c[cartesian_trajectory_planner_B.b_kstr_e]
          = cartesian_trajectory_planner_B.e_n +
          cartesian_trajectory_planner_B.b_kstr_e;
      }

      cartesian_trajectory_planner_B.result_data_c[0] =
        cartesian_trajectory_planner_B.v_na[0];
      cartesian_trajectory_planner_B.result_data_c[1] =
        cartesian_trajectory_planner_B.v_na[1];
      cartesian_trajectory_planner_B.result_data_c[2] =
        cartesian_trajectory_planner_B.v_na[2];
      if (0 <= (cartesian_trajectory_planner_B.d_e != 0) - 1) {
        cartesian_trajectory_planner_B.result_data_c[3] =
          qvec[cartesian_trajectory_planner_B.e_data_c[0]];
      }

      cartesian_trajectory_planner_B.k_f = 1.0 / sqrt
        ((cartesian_trajectory_planner_B.result_data_c[0] *
          cartesian_trajectory_planner_B.result_data_c[0] +
          cartesian_trajectory_planner_B.result_data_c[1] *
          cartesian_trajectory_planner_B.result_data_c[1]) +
         cartesian_trajectory_planner_B.result_data_c[2] *
         cartesian_trajectory_planner_B.result_data_c[2]);
      cartesian_trajectory_planner_B.v_na[0] =
        cartesian_trajectory_planner_B.result_data_c[0] *
        cartesian_trajectory_planner_B.k_f;
      cartesian_trajectory_planner_B.v_na[1] =
        cartesian_trajectory_planner_B.result_data_c[1] *
        cartesian_trajectory_planner_B.k_f;
      cartesian_trajectory_planner_B.v_na[2] =
        cartesian_trajectory_planner_B.result_data_c[2] *
        cartesian_trajectory_planner_B.k_f;
      cartesian_trajectory_planner_B.k_f = cos
        (cartesian_trajectory_planner_B.result_data_c[3]);
      cartesian_trajectory_planner_B.sth_o = sin
        (cartesian_trajectory_planner_B.result_data_c[3]);
      cartesian_trajectory_planner_B.tempR_o[0] =
        cartesian_trajectory_planner_B.v_na[0] *
        cartesian_trajectory_planner_B.v_na[0] * (1.0 -
        cartesian_trajectory_planner_B.k_f) + cartesian_trajectory_planner_B.k_f;
      cartesian_trajectory_planner_B.tempR_tmp_ln =
        cartesian_trajectory_planner_B.v_na[1] *
        cartesian_trajectory_planner_B.v_na[0] * (1.0 -
        cartesian_trajectory_planner_B.k_f);
      cartesian_trajectory_planner_B.tempR_tmp_lu =
        cartesian_trajectory_planner_B.v_na[2] *
        cartesian_trajectory_planner_B.sth_o;
      cartesian_trajectory_planner_B.tempR_o[1] =
        cartesian_trajectory_planner_B.tempR_tmp_ln -
        cartesian_trajectory_planner_B.tempR_tmp_lu;
      cartesian_trajectory_planner_B.tempR_tmp_g =
        cartesian_trajectory_planner_B.v_na[2] *
        cartesian_trajectory_planner_B.v_na[0] * (1.0 -
        cartesian_trajectory_planner_B.k_f);
      cartesian_trajectory_planner_B.tempR_tmp_d =
        cartesian_trajectory_planner_B.v_na[1] *
        cartesian_trajectory_planner_B.sth_o;
      cartesian_trajectory_planner_B.tempR_o[2] =
        cartesian_trajectory_planner_B.tempR_tmp_g +
        cartesian_trajectory_planner_B.tempR_tmp_d;
      cartesian_trajectory_planner_B.tempR_o[3] =
        cartesian_trajectory_planner_B.tempR_tmp_ln +
        cartesian_trajectory_planner_B.tempR_tmp_lu;
      cartesian_trajectory_planner_B.tempR_o[4] =
        cartesian_trajectory_planner_B.v_na[1] *
        cartesian_trajectory_planner_B.v_na[1] * (1.0 -
        cartesian_trajectory_planner_B.k_f) + cartesian_trajectory_planner_B.k_f;
      cartesian_trajectory_planner_B.tempR_tmp_ln =
        cartesian_trajectory_planner_B.v_na[2] *
        cartesian_trajectory_planner_B.v_na[1] * (1.0 -
        cartesian_trajectory_planner_B.k_f);
      cartesian_trajectory_planner_B.tempR_tmp_lu =
        cartesian_trajectory_planner_B.v_na[0] *
        cartesian_trajectory_planner_B.sth_o;
      cartesian_trajectory_planner_B.tempR_o[5] =
        cartesian_trajectory_planner_B.tempR_tmp_ln -
        cartesian_trajectory_planner_B.tempR_tmp_lu;
      cartesian_trajectory_planner_B.tempR_o[6] =
        cartesian_trajectory_planner_B.tempR_tmp_g -
        cartesian_trajectory_planner_B.tempR_tmp_d;
      cartesian_trajectory_planner_B.tempR_o[7] =
        cartesian_trajectory_planner_B.tempR_tmp_ln +
        cartesian_trajectory_planner_B.tempR_tmp_lu;
      cartesian_trajectory_planner_B.tempR_o[8] =
        cartesian_trajectory_planner_B.v_na[2] *
        cartesian_trajectory_planner_B.v_na[2] * (1.0 -
        cartesian_trajectory_planner_B.k_f) + cartesian_trajectory_planner_B.k_f;
      for (cartesian_trajectory_planner_B.b_kstr_e = 0;
           cartesian_trajectory_planner_B.b_kstr_e < 3;
           cartesian_trajectory_planner_B.b_kstr_e++) {
        cartesian_trajectory_planner_B.e_n =
          cartesian_trajectory_planner_B.b_kstr_e + 1;
        cartesian_trajectory_planner_B.R_e[cartesian_trajectory_planner_B.e_n -
          1] = cartesian_trajectory_planner_B.tempR_o
          [(cartesian_trajectory_planner_B.e_n - 1) * 3];
        cartesian_trajectory_planner_B.e_n =
          cartesian_trajectory_planner_B.b_kstr_e + 1;
        cartesian_trajectory_planner_B.R_e[cartesian_trajectory_planner_B.e_n +
          2] = cartesian_trajectory_planner_B.tempR_o
          [(cartesian_trajectory_planner_B.e_n - 1) * 3 + 1];
        cartesian_trajectory_planner_B.e_n =
          cartesian_trajectory_planner_B.b_kstr_e + 1;
        cartesian_trajectory_planner_B.R_e[cartesian_trajectory_planner_B.e_n +
          5] = cartesian_trajectory_planner_B.tempR_o
          [(cartesian_trajectory_planner_B.e_n - 1) * 3 + 2];
      }

      memset(&cartesian_trajectory_planner_B.c_f1_h[0], 0, sizeof(real_T) << 4U);
      for (cartesian_trajectory_planner_B.b_kstr_e = 0;
           cartesian_trajectory_planner_B.b_kstr_e < 3;
           cartesian_trajectory_planner_B.b_kstr_e++) {
        cartesian_trajectory_planner_B.d_e =
          cartesian_trajectory_planner_B.b_kstr_e << 2;
        cartesian_trajectory_planner_B.c_f1_h[cartesian_trajectory_planner_B.d_e]
          = cartesian_trajectory_planner_B.R_e[3 *
          cartesian_trajectory_planner_B.b_kstr_e];
        cartesian_trajectory_planner_B.c_f1_h[cartesian_trajectory_planner_B.d_e
          + 1] = cartesian_trajectory_planner_B.R_e[3 *
          cartesian_trajectory_planner_B.b_kstr_e + 1];
        cartesian_trajectory_planner_B.c_f1_h[cartesian_trajectory_planner_B.d_e
          + 2] = cartesian_trajectory_planner_B.R_e[3 *
          cartesian_trajectory_planner_B.b_kstr_e + 2];
      }

      cartesian_trajectory_planner_B.c_f1_h[15] = 1.0;
      break;

     default:
      rigidBodyJoint_get_JointAxis_a(&body->JointInternal,
        cartesian_trajectory_planner_B.v_na);
      memset(&cartesian_trajectory_planner_B.tempR_o[0], 0, 9U * sizeof(real_T));
      cartesian_trajectory_planner_B.tempR_o[0] = 1.0;
      cartesian_trajectory_planner_B.tempR_o[4] = 1.0;
      cartesian_trajectory_planner_B.tempR_o[8] = 1.0;
      for (cartesian_trajectory_planner_B.b_kstr_e = 0;
           cartesian_trajectory_planner_B.b_kstr_e < 3;
           cartesian_trajectory_planner_B.b_kstr_e++) {
        cartesian_trajectory_planner_B.d_e =
          cartesian_trajectory_planner_B.b_kstr_e << 2;
        cartesian_trajectory_planner_B.c_f1_h[cartesian_trajectory_planner_B.d_e]
          = cartesian_trajectory_planner_B.tempR_o[3 *
          cartesian_trajectory_planner_B.b_kstr_e];
        cartesian_trajectory_planner_B.c_f1_h[cartesian_trajectory_planner_B.d_e
          + 1] = cartesian_trajectory_planner_B.tempR_o[3 *
          cartesian_trajectory_planner_B.b_kstr_e + 1];
        cartesian_trajectory_planner_B.c_f1_h[cartesian_trajectory_planner_B.d_e
          + 2] = cartesian_trajectory_planner_B.tempR_o[3 *
          cartesian_trajectory_planner_B.b_kstr_e + 2];
        cartesian_trajectory_planner_B.c_f1_h[cartesian_trajectory_planner_B.b_kstr_e
          + 12] =
          cartesian_trajectory_planner_B.v_na[cartesian_trajectory_planner_B.b_kstr_e]
          * qvec[cartesian_trajectory_planner_B.e_n];
      }

      cartesian_trajectory_planner_B.c_f1_h[3] = 0.0;
      cartesian_trajectory_planner_B.c_f1_h[7] = 0.0;
      cartesian_trajectory_planner_B.c_f1_h[11] = 0.0;
      cartesian_trajectory_planner_B.c_f1_h[15] = 1.0;
      break;
    }

    for (cartesian_trajectory_planner_B.b_kstr_e = 0;
         cartesian_trajectory_planner_B.b_kstr_e < 16;
         cartesian_trajectory_planner_B.b_kstr_e++) {
      cartesian_trajectory_planner_B.a_c[cartesian_trajectory_planner_B.b_kstr_e]
        = body->
        JointInternal.JointToParentTransform[cartesian_trajectory_planner_B.b_kstr_e];
    }

    for (cartesian_trajectory_planner_B.b_kstr_e = 0;
         cartesian_trajectory_planner_B.b_kstr_e < 16;
         cartesian_trajectory_planner_B.b_kstr_e++) {
      cartesian_trajectory_planner_B.b_c[cartesian_trajectory_planner_B.b_kstr_e]
        = body->
        JointInternal.ChildToJointTransform[cartesian_trajectory_planner_B.b_kstr_e];
    }

    for (cartesian_trajectory_planner_B.b_kstr_e = 0;
         cartesian_trajectory_planner_B.b_kstr_e < 4;
         cartesian_trajectory_planner_B.b_kstr_e++) {
      for (cartesian_trajectory_planner_B.e_n = 0;
           cartesian_trajectory_planner_B.e_n < 4;
           cartesian_trajectory_planner_B.e_n++) {
        cartesian_trajectory_planner_B.d_e = cartesian_trajectory_planner_B.e_n <<
          2;
        cartesian_trajectory_planner_B.loop_ub_by =
          cartesian_trajectory_planner_B.b_kstr_e +
          cartesian_trajectory_planner_B.d_e;
        cartesian_trajectory_planner_B.a_p[cartesian_trajectory_planner_B.loop_ub_by]
          = 0.0;
        cartesian_trajectory_planner_B.a_p[cartesian_trajectory_planner_B.loop_ub_by]
          +=
          cartesian_trajectory_planner_B.c_f1_h[cartesian_trajectory_planner_B.d_e]
          * cartesian_trajectory_planner_B.a_c[cartesian_trajectory_planner_B.b_kstr_e];
        cartesian_trajectory_planner_B.a_p[cartesian_trajectory_planner_B.loop_ub_by]
          +=
          cartesian_trajectory_planner_B.c_f1_h[cartesian_trajectory_planner_B.d_e
          + 1] *
          cartesian_trajectory_planner_B.a_c[cartesian_trajectory_planner_B.b_kstr_e
          + 4];
        cartesian_trajectory_planner_B.a_p[cartesian_trajectory_planner_B.loop_ub_by]
          +=
          cartesian_trajectory_planner_B.c_f1_h[cartesian_trajectory_planner_B.d_e
          + 2] *
          cartesian_trajectory_planner_B.a_c[cartesian_trajectory_planner_B.b_kstr_e
          + 8];
        cartesian_trajectory_planner_B.a_p[cartesian_trajectory_planner_B.loop_ub_by]
          +=
          cartesian_trajectory_planner_B.c_f1_h[cartesian_trajectory_planner_B.d_e
          + 3] *
          cartesian_trajectory_planner_B.a_c[cartesian_trajectory_planner_B.b_kstr_e
          + 12];
      }

      for (cartesian_trajectory_planner_B.e_n = 0;
           cartesian_trajectory_planner_B.e_n < 4;
           cartesian_trajectory_planner_B.e_n++) {
        cartesian_trajectory_planner_B.d_e = cartesian_trajectory_planner_B.e_n <<
          2;
        cartesian_trajectory_planner_B.loop_ub_by =
          cartesian_trajectory_planner_B.b_kstr_e +
          cartesian_trajectory_planner_B.d_e;
        Ttree->data[cartesian_trajectory_planner_B.b_jtilecol_n]
          .f1[cartesian_trajectory_planner_B.loop_ub_by] = 0.0;
        Ttree->data[cartesian_trajectory_planner_B.b_jtilecol_n]
          .f1[cartesian_trajectory_planner_B.loop_ub_by] +=
          cartesian_trajectory_planner_B.b_c[cartesian_trajectory_planner_B.d_e]
          * cartesian_trajectory_planner_B.a_p[cartesian_trajectory_planner_B.b_kstr_e];
        Ttree->data[cartesian_trajectory_planner_B.b_jtilecol_n]
          .f1[cartesian_trajectory_planner_B.loop_ub_by] +=
          cartesian_trajectory_planner_B.b_c[cartesian_trajectory_planner_B.d_e
          + 1] *
          cartesian_trajectory_planner_B.a_p[cartesian_trajectory_planner_B.b_kstr_e
          + 4];
        Ttree->data[cartesian_trajectory_planner_B.b_jtilecol_n]
          .f1[cartesian_trajectory_planner_B.loop_ub_by] +=
          cartesian_trajectory_planner_B.b_c[cartesian_trajectory_planner_B.d_e
          + 2] *
          cartesian_trajectory_planner_B.a_p[cartesian_trajectory_planner_B.b_kstr_e
          + 8];
        Ttree->data[cartesian_trajectory_planner_B.b_jtilecol_n]
          .f1[cartesian_trajectory_planner_B.loop_ub_by] +=
          cartesian_trajectory_planner_B.b_c[cartesian_trajectory_planner_B.d_e
          + 3] *
          cartesian_trajectory_planner_B.a_p[cartesian_trajectory_planner_B.b_kstr_e
          + 12];
      }
    }

    cartesian_trajectory_planner_B.k_f = cartesian_trajectory_planner_B.n_o;
    if (body->ParentIndex > 0.0) {
      for (cartesian_trajectory_planner_B.b_kstr_e = 0;
           cartesian_trajectory_planner_B.b_kstr_e < 16;
           cartesian_trajectory_planner_B.b_kstr_e++) {
        cartesian_trajectory_planner_B.a_c[cartesian_trajectory_planner_B.b_kstr_e]
          = Ttree->data[static_cast<int32_T>(body->ParentIndex) - 1]
          .f1[cartesian_trajectory_planner_B.b_kstr_e];
      }

      for (cartesian_trajectory_planner_B.b_kstr_e = 0;
           cartesian_trajectory_planner_B.b_kstr_e < 4;
           cartesian_trajectory_planner_B.b_kstr_e++) {
        for (cartesian_trajectory_planner_B.e_n = 0;
             cartesian_trajectory_planner_B.e_n < 4;
             cartesian_trajectory_planner_B.e_n++) {
          cartesian_trajectory_planner_B.d_e =
            cartesian_trajectory_planner_B.e_n << 2;
          cartesian_trajectory_planner_B.loop_ub_by =
            cartesian_trajectory_planner_B.b_kstr_e +
            cartesian_trajectory_planner_B.d_e;
          cartesian_trajectory_planner_B.a_p[cartesian_trajectory_planner_B.loop_ub_by]
            = 0.0;
          cartesian_trajectory_planner_B.a_p[cartesian_trajectory_planner_B.loop_ub_by]
            += Ttree->data[cartesian_trajectory_planner_B.b_jtilecol_n]
            .f1[cartesian_trajectory_planner_B.d_e] *
            cartesian_trajectory_planner_B.a_c[cartesian_trajectory_planner_B.b_kstr_e];
          cartesian_trajectory_planner_B.a_p[cartesian_trajectory_planner_B.loop_ub_by]
            += Ttree->data[cartesian_trajectory_planner_B.b_jtilecol_n]
            .f1[cartesian_trajectory_planner_B.d_e + 1] *
            cartesian_trajectory_planner_B.a_c[cartesian_trajectory_planner_B.b_kstr_e
            + 4];
          cartesian_trajectory_planner_B.a_p[cartesian_trajectory_planner_B.loop_ub_by]
            += Ttree->data[cartesian_trajectory_planner_B.b_jtilecol_n]
            .f1[cartesian_trajectory_planner_B.d_e + 2] *
            cartesian_trajectory_planner_B.a_c[cartesian_trajectory_planner_B.b_kstr_e
            + 8];
          cartesian_trajectory_planner_B.a_p[cartesian_trajectory_planner_B.loop_ub_by]
            += Ttree->data[cartesian_trajectory_planner_B.b_jtilecol_n]
            .f1[cartesian_trajectory_planner_B.d_e + 3] *
            cartesian_trajectory_planner_B.a_c[cartesian_trajectory_planner_B.b_kstr_e
            + 12];
        }
      }

      for (cartesian_trajectory_planner_B.b_kstr_e = 0;
           cartesian_trajectory_planner_B.b_kstr_e < 16;
           cartesian_trajectory_planner_B.b_kstr_e++) {
        Ttree->data[cartesian_trajectory_planner_B.b_jtilecol_n]
          .f1[cartesian_trajectory_planner_B.b_kstr_e] =
          cartesian_trajectory_planner_B.a_p[cartesian_trajectory_planner_B.b_kstr_e];
      }
    }
  }

  cartesian_trajec_emxFree_char_T(&switch_expression);
}

void rt_invd6x6_snf(const real_T u[36], real_T y[36])
{
  int8_T p[6];
  real_T A[36];
  int8_T ipiv[6];
  int32_T jj;
  int32_T j;
  int32_T kAcol;
  int32_T c;
  int32_T ix;
  real_T smax;
  real_T s;
  int32_T iy;
  int32_T jy;
  int32_T j_0;
  int32_T ijA;
  for (iy = 0; iy < 36; iy++) {
    y[iy] = 0.0;
    A[iy] = u[iy];
  }

  for (iy = 0; iy < 6; iy++) {
    ipiv[iy] = static_cast<int8_T>(iy + 1);
  }

  for (j = 0; j < 5; j++) {
    c = j * 7 + 2;
    jj = j * 7;
    kAcol = 6 - j;
    iy = 1;
    ix = c - 2;
    smax = fabs(A[jj]);
    for (jy = 2; jy <= kAcol; jy++) {
      ix++;
      s = fabs(A[ix]);
      if (s > smax) {
        iy = jy;
        smax = s;
      }
    }

    if (A[(c + iy) - 3] != 0.0) {
      if (iy - 1 != 0) {
        jy = j + iy;
        ipiv[j] = static_cast<int8_T>(jy);
        ix = j;
        iy = jy - 1;
        for (jy = 0; jy < 6; jy++) {
          smax = A[ix];
          A[ix] = A[iy];
          A[iy] = smax;
          ix += 6;
          iy += 6;
        }
      }

      iy = c - j;
      for (ix = c; ix <= iy + 4; ix++) {
        A[ix - 1] /= A[jj];
      }
    }

    kAcol = 4 - j;
    jy = jj + 6;
    for (j_0 = 0; j_0 <= kAcol; j_0++) {
      if (A[jy] != 0.0) {
        smax = -A[jy];
        ix = c - 1;
        iy = jj - j;
        for (ijA = jj + 8; ijA <= iy + 12; ijA++) {
          A[ijA - 1] += A[ix] * smax;
          ix++;
        }
      }

      jy += 6;
      jj += 6;
    }
  }

  for (iy = 0; iy < 6; iy++) {
    p[iy] = static_cast<int8_T>(iy + 1);
  }

  for (jy = 0; jy < 5; jy++) {
    if (ipiv[jy] > jy + 1) {
      j = ipiv[jy] - 1;
      iy = p[j];
      p[j] = p[jy];
      p[jy] = static_cast<int8_T>(iy);
    }
  }

  for (jy = 0; jy < 6; jy++) {
    jj = p[jy] - 1;
    y[jy + 6 * jj] = 1.0;
    for (j = jy + 1; j < 7; j++) {
      iy = (6 * jj + j) - 1;
      if (y[iy] != 0.0) {
        for (ix = j + 1; ix < 7; ix++) {
          c = (6 * jj + ix) - 1;
          y[c] -= A[((j - 1) * 6 + ix) - 1] * y[iy];
        }
      }
    }
  }

  for (j = 0; j < 6; j++) {
    jj = 6 * j;
    for (jy = 5; jy >= 0; jy--) {
      kAcol = 6 * jy;
      iy = jy + jj;
      if (y[iy] != 0.0) {
        y[iy] /= A[jy + kAcol];
        j_0 = jy - 1;
        for (ix = 0; ix <= j_0; ix++) {
          c = ix + jj;
          y[c] -= y[iy] * A[ix + kAcol];
        }
      }
    }
  }
}

static void matlabCodegenHandle_matla_astwh(ros_slros_internal_block_Subs_T *obj)
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

static void matlabCodegenHandle_matlabC_ast(robotics_slmanip_internal__as_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void cartesian_tr_SystemCore_release(b_inverseKinematics_cartesian_T *obj)
{
  if (obj->isInitialized == 1) {
    obj->isInitialized = 2;
  }
}

static void cartesian__SystemCore_delete_as(b_inverseKinematics_cartesian_T *obj)
{
  cartesian_tr_SystemCore_release(obj);
}

static void matlabCodegenHandle_matlabCo_as(b_inverseKinematics_cartesian_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
    cartesian__SystemCore_delete_as(obj);
  }
}

static void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_cartesian_as_T
  *pStruct)
{
  cartesian_trajec_emxFree_char_T(&pStruct->Type);
  cartesian_trajec_emxFree_real_T(&pStruct->MotionSubspace);
  cartesian_trajec_emxFree_char_T(&pStruct->NameInternal);
  cartesian_trajec_emxFree_real_T(&pStruct->PositionLimitsInternal);
  cartesian_trajec_emxFree_real_T(&pStruct->HomePositionInternal);
}

static void emxFreeStruct_v_robotics_manip_(v_robotics_manip_internal_Rig_T
  *pStruct)
{
  cartesian_trajec_emxFree_char_T(&pStruct->NameInternal);
  emxFreeStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static void emxFreeStruct_y_robotics_manip_(y_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_v_robotics_manip_(&pStruct->Base);
}

static void emxFreeStruct_b_inverseKinemati(b_inverseKinematics_cartesian_T
  *pStruct)
{
  cartesian_trajec_emxFree_real_T(&pStruct->Limits);
}

static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal__as_T
  *pStruct)
{
  emxFreeStruct_y_robotics_manip_(&pStruct->TreeInternal);
  emxFreeStruct_b_inverseKinemati(&pStruct->IKInternal);
}

static void emxFreeStruct_w_robotics_manip_(w_robotics_manip_internal_Rig_T
  *pStruct)
{
  cartesian_trajec_emxFree_char_T(&pStruct->NameInternal);
}

static void emxFreeStruct_x_robotics_manip_(x_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_w_robotics_manip_(&pStruct->Base);
}

static void emxFreeStruct_f_robotics_manip_(f_robotics_manip_internal_IKE_T
  *pStruct)
{
  cartesian_trajec_emxFree_char_T(&pStruct->BodyName);
  cartesian_trajec_emxFree_real_T(&pStruct->ErrTemp);
  cartesian_trajec_emxFree_real_T(&pStruct->GradTemp);
}

static void emxFreeStruct_h_robotics_core_i(h_robotics_core_internal_Damp_T
  *pStruct)
{
  cartesian_trajec_emxFree_real_T(&pStruct->ConstraintMatrix);
  cartesian_trajec_emxFree_real_T(&pStruct->ConstraintBound);
}

static void emxFreeStruct_c_rigidBodyJoint1(c_rigidBodyJoint_cartesian_tr_T
  *pStruct)
{
  cartesian_trajec_emxFree_char_T(&pStruct->Type);
  cartesian_trajec_emxFree_real_T(&pStruct->MotionSubspace);
}

static void emxFreeStruct_o_robotics_manip_(o_robotics_manip_internal_Rig_T
  *pStruct)
{
  cartesian_trajec_emxFree_char_T(&pStruct->NameInternal);
  emxFreeStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static void emxFreeStruct_p_robotics_manip_(p_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_o_robotics_manip_(&pStruct->Base);
}

static void emxFreeStruct_robotics_slmani_a(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxFreeStruct_p_robotics_manip_(&pStruct->TreeInternal);
}

static void emxFreeStruct_n_robotics_manip_(n_robotics_manip_internal_Rig_T
  *pStruct)
{
  cartesian_trajec_emxFree_char_T(&pStruct->NameInternal);
  emxFreeStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static void emxFreeStruct_c_rigidBodyJoint2(c_rigidBodyJoint_cartesian__a_T
  *pStruct)
{
  cartesian_trajec_emxFree_char_T(&pStruct->Type);
}

static void emxFreeStruct_o_robotics_mani_a(o_robotics_manip_internal_R_a_T
  *pStruct)
{
  cartesian_trajec_emxFree_char_T(&pStruct->NameInternal);
  emxFreeStruct_c_rigidBodyJoint2(&pStruct->JointInternal);
}

static void emxFreeStruct_p_robotics_mani_a(p_robotics_manip_internal_R_a_T
  *pStruct)
{
  emxFreeStruct_o_robotics_mani_a(&pStruct->Base);
}

static void emxFreeStruct_robotics_slman_as(robotics_slmanip_internal_b_a_T
  *pStruct)
{
  emxFreeStruct_p_robotics_mani_a(&pStruct->TreeInternal);
}

static void emxFreeStruct_n_robotics_mani_a(n_robotics_manip_internal_R_a_T
  *pStruct)
{
  cartesian_trajec_emxFree_char_T(&pStruct->NameInternal);
  emxFreeStruct_c_rigidBodyJoint2(&pStruct->JointInternal);
}

static void matlabCodegenHandle_matlab_astw(ros_slros_internal_block_Publ_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_cartesian_as_T
  *pStruct)
{
  cartesian_trajec_emxInit_char_T(&pStruct->Type, 2);
  cartesian_trajec_emxInit_real_T(&pStruct->MotionSubspace, 2);
  cartesian_trajec_emxInit_char_T(&pStruct->NameInternal, 2);
  cartesian_trajec_emxInit_real_T(&pStruct->PositionLimitsInternal, 2);
  cartesian_trajec_emxInit_real_T(&pStruct->HomePositionInternal, 1);
}

static void emxInitStruct_v_robotics_manip_(v_robotics_manip_internal_Rig_T
  *pStruct)
{
  cartesian_trajec_emxInit_char_T(&pStruct->NameInternal, 2);
  emxInitStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static void emxInitStruct_y_robotics_manip_(y_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_v_robotics_manip_(&pStruct->Base);
}

static void emxInitStruct_b_inverseKinemati(b_inverseKinematics_cartesian_T
  *pStruct)
{
  cartesian_trajec_emxInit_real_T(&pStruct->Limits, 2);
}

static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal__as_T
  *pStruct)
{
  emxInitStruct_y_robotics_manip_(&pStruct->TreeInternal);
  emxInitStruct_b_inverseKinemati(&pStruct->IKInternal);
}

static void emxInitStruct_w_robotics_manip_(w_robotics_manip_internal_Rig_T
  *pStruct)
{
  cartesian_trajec_emxInit_char_T(&pStruct->NameInternal, 2);
}

static void emxInitStruct_x_robotics_manip_(x_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_w_robotics_manip_(&pStruct->Base);
}

static void emxInitStruct_f_robotics_manip_(f_robotics_manip_internal_IKE_T
  *pStruct)
{
  cartesian_trajec_emxInit_char_T(&pStruct->BodyName, 2);
  cartesian_trajec_emxInit_real_T(&pStruct->ErrTemp, 1);
  cartesian_trajec_emxInit_real_T(&pStruct->GradTemp, 1);
}

static void emxInitStruct_h_robotics_core_i(h_robotics_core_internal_Damp_T
  *pStruct)
{
  cartesian_trajec_emxInit_real_T(&pStruct->ConstraintMatrix, 2);
  cartesian_trajec_emxInit_real_T(&pStruct->ConstraintBound, 1);
}

static void cartesia_twister_state_vector_a(uint32_T mt[625])
{
  uint32_T r;
  int32_T b_mti;
  r = 5489U;
  mt[0] = 5489U;
  for (b_mti = 0; b_mti < 623; b_mti++) {
    r = ((r >> 30U ^ r) * 1812433253U + b_mti) + 1U;
    mt[b_mti + 1] = r;
  }

  mt[624] = 624U;
}

static void cartesian_tr_eml_rand_mt19937ar(uint32_T state[625])
{
  memset(&state[0], 0, 625U * sizeof(uint32_T));
  cartesia_twister_state_vector_a(state);
}

static v_robotics_manip_internal_Rig_T *RigidBody_RigidB_astwhqf2azttxa
  (v_robotics_manip_internal_Rig_T *obj)
{
  v_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  real_T poslim_data[12];
  emxArray_char_T_cartesian_tra_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[13] = { 'e', 'd', 'o', '_', 'b', 'a', 's', 'e', '_',
    'l', 'i', 'n', 'k' };

  static const real_T tmp_1[9] = { 0.012583419040406959, -0.00021487638648447484,
    -0.00022605919127205462, -0.00021487638648447484, 0.00052369449451288713,
    -0.00011525315957400814, -0.00022605919127205462, -0.00011525315957400814,
    0.012646079447789898 };

  static const real_T tmp_2[36] = { 0.012583419040406959,
    -0.00021487638648447484, -0.00022605919127205462, 0.0, -0.00392971169381184,
    0.00047022930128152475, -0.00021487638648447484, 0.00052369449451288713,
    -0.00011525315957400814, 0.00392971169381184, 0.0, -0.00449464704691423,
    -0.00022605919127205462, -0.00011525315957400814, 0.012646079447789898,
    -0.00047022930128152475, 0.00449464704691423, 0.0, 0.0, 0.00392971169381184,
    -0.00047022930128152475, 0.0785942338762368, 0.0, 0.0, -0.00392971169381184,
    0.0, 0.00449464704691423, 0.0, 0.0785942338762368, 0.0,
    0.00047022930128152475, -0.00449464704691423, 0.0, 0.0, 0.0,
    0.0785942338762368 };

  static const int8_T tmp_3[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_4[15] = { 'w', 'o', 'r', 'l', 'd', '_', 'e', 'd', 'o',
    '_', 'j', 'o', 'i', 'n', 't' };

  static const char_T tmp_5[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_6[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_7[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_8[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_9[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 13;
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 13; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 0.0;
  obj->MassInternal = 0.0785942338762368;
  obj->CenterOfMassInternal[0] = 0.057188;
  obj->CenterOfMassInternal[1] = 0.005983;
  obj->CenterOfMassInternal[2] = 0.05;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_2[b_kstr];
  }

  obj->JointInternal.InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  b_kstr = obj->JointInternal.NameInternal->size[0] *
    obj->JointInternal.NameInternal->size[1];
  obj->JointInternal.NameInternal->size[0] = 1;
  obj->JointInternal.NameInternal->size[1] = 15;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 15; b_kstr++) {
    obj->JointInternal.NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_5[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_6[b_kstr];
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
      b_0[b_kstr] = tmp_7[b_kstr];
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

  cartesian_trajec_emxFree_char_T(&switch_expression);
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

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    obj->JointInternal.VelocityNumber = 1.0;
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

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    obj->JointInternal.VelocityNumber = 0.0;
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    obj->JointInternal.PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 1; b_kstr++) {
    obj->JointInternal.HomePositionInternal->data[0] = 0.0;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_8[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_9[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.InTree = true;
  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr);
  obj->JointInternal.PositionLimitsInternal->data[0] = 0.0;
  obj->JointInternal.PositionLimitsInternal->data
    [obj->JointInternal.PositionLimitsInternal->size[0]] = 0.0;
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr);
  obj->JointInternal.HomePositionInternal->data[0] = 0.0;
  return b_obj;
}

static v_robotics_manip_internal_Rig_T *RigidBody_Rigid_astwhqf2azttxab
  (v_robotics_manip_internal_Rig_T *obj)
{
  v_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  real_T poslim_data[12];
  emxArray_char_T_cartesian_tra_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[10] = { 'e', 'd', 'o', '_', 'l', 'i', 'n', 'k', '_',
    '1' };

  static const real_T tmp_1[9] = { 0.012559660892485551, 0.00032713982710414,
    -1.32683892634271E-6, 0.00032713982710414, 0.00018048007331848145,
    9.17416945099368E-5, -1.32683892634271E-6, 9.17416945099368E-5,
    0.012672078048055372 };

  static const real_T tmp_2[36] = { 0.012559660892485551, 0.00032713982710414,
    -1.32683892634271E-6, 0.0, -0.0, 0.0037143634929909515, 0.00032713982710414,
    0.00018048007331848145, 9.17416945099368E-5, 0.0, 0.0, 0.0029444543779393356,
    -1.32683892634271E-6, 9.17416945099368E-5, 0.012672078048055372,
    -0.0037143634929909515, -0.0029444543779393356, 0.0, 0.0, 0.0,
    -0.0037143634929909515, 0.0785942338762368, 0.0, 0.0, -0.0, 0.0,
    -0.0029444543779393356, 0.0, 0.0785942338762368, 0.0, 0.0037143634929909515,
    0.0029444543779393356, 0.0, 0.0, 0.0, 0.0785942338762368 };

  static const int8_T tmp_3[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_4[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n', 't',
    '_', '1' };

  static const char_T tmp_5[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_6[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_7[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.337, 1.0 };

  static const real_T tmp_8[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_9[36] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 1.0;
  obj->MassInternal = 0.0785942338762368;
  obj->CenterOfMassInternal[0] = -0.037464;
  obj->CenterOfMassInternal[1] = 0.04726;
  obj->CenterOfMassInternal[2] = 0.0;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_2[b_kstr];
  }

  obj->JointInternal.InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  b_kstr = obj->JointInternal.NameInternal->size[0] *
    obj->JointInternal.NameInternal->size[1];
  obj->JointInternal.NameInternal->size[0] = 1;
  obj->JointInternal.NameInternal->size[1] = 11;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 11; b_kstr++) {
    obj->JointInternal.NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_5[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_5[b_kstr];
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
      b_0[b_kstr] = tmp_6[b_kstr];
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

  cartesian_trajec_emxFree_char_T(&switch_expression);
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

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    obj->JointInternal.VelocityNumber = 1.0;
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

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    obj->JointInternal.VelocityNumber = 0.0;
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    obj->JointInternal.PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 1; b_kstr++) {
    obj->JointInternal.HomePositionInternal->data[0] = 0.0;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_7[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_8[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_9[b_kstr];
  }

  obj->JointInternal.InTree = true;
  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr);
  obj->JointInternal.PositionLimitsInternal->data[0] = -3.10668606855;
  obj->JointInternal.PositionLimitsInternal->data
    [obj->JointInternal.PositionLimitsInternal->size[0]] = 3.10668606855;
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr);
  obj->JointInternal.HomePositionInternal->data[0] = 0.0;
  return b_obj;
}

static v_robotics_manip_internal_Rig_T *c_RigidBody_Rigid_b
  (v_robotics_manip_internal_Rig_T *obj)
{
  v_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  real_T poslim_data[12];
  emxArray_char_T_cartesian_tra_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[10] = { 'e', 'd', 'o', '_', 'l', 'i', 'n', 'k', '_',
    '2' };

  static const real_T tmp_1[9] = { 0.012452446533447339, 0.00097164648860403489,
    -0.0012079400901817208, 0.00097164648860403489, 0.0077631932790618,
    0.0060529024261622953, -0.0012079400901817208, 0.0060529024261622953,
    0.0051581160550583753 };

  static const real_T tmp_2[36] = { 0.012452446533447339, 0.00097164648860403489,
    -0.0012079400901817208, 0.0, 0.0036112478581453288, 0.0024995324199659588,
    0.00097164648860403489, 0.0077631932790618, 0.0060529024261622953,
    -0.0036112478581453288, 0.0, 0.0012907531029494371, -0.0012079400901817208,
    0.0060529024261622953, 0.0051581160550583753, -0.0024995324199659588,
    -0.0012907531029494371, 0.0, 0.0, -0.0036112478581453288,
    -0.0024995324199659588, 0.0785942338762368, 0.0, 0.0, 0.0036112478581453288,
    0.0, -0.0012907531029494371, 0.0, 0.0785942338762368, 0.0,
    0.0024995324199659588, 0.0012907531029494371, 0.0, 0.0, 0.0,
    0.0785942338762368 };

  static const int8_T tmp_3[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_4[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n', 't',
    '_', '2' };

  static const char_T tmp_5[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_6[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_7[16] = { 1.0, 0.0, -0.0, 0.0, 0.0,
    4.8965888601467475E-12, 1.0, 0.0, 0.0, -1.0, 4.8965888601467475E-12, 0.0,
    0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_8[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_9[36] = { 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 2.0;
  obj->MassInternal = 0.0785942338762368;
  obj->CenterOfMassInternal[0] = -0.016423;
  obj->CenterOfMassInternal[1] = 0.031803;
  obj->CenterOfMassInternal[2] = -0.045948;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_2[b_kstr];
  }

  obj->JointInternal.InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  b_kstr = obj->JointInternal.NameInternal->size[0] *
    obj->JointInternal.NameInternal->size[1];
  obj->JointInternal.NameInternal->size[0] = 1;
  obj->JointInternal.NameInternal->size[1] = 11;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 11; b_kstr++) {
    obj->JointInternal.NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_5[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_5[b_kstr];
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
      b_0[b_kstr] = tmp_6[b_kstr];
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

  cartesian_trajec_emxFree_char_T(&switch_expression);
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

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    obj->JointInternal.VelocityNumber = 1.0;
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

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    obj->JointInternal.VelocityNumber = 0.0;
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    obj->JointInternal.PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 1; b_kstr++) {
    obj->JointInternal.HomePositionInternal->data[0] = 0.0;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_7[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_8[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_9[b_kstr];
  }

  obj->JointInternal.InTree = true;
  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr);
  obj->JointInternal.PositionLimitsInternal->data[0] = -1.71042266695;
  obj->JointInternal.PositionLimitsInternal->data
    [obj->JointInternal.PositionLimitsInternal->size[0]] = 1.71042266695;
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = -1.0;
  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr);
  obj->JointInternal.HomePositionInternal->data[0] = 0.0;
  return b_obj;
}

static v_robotics_manip_internal_Rig_T *c_RigidBody_Rigid_i
  (v_robotics_manip_internal_Rig_T *obj)
{
  v_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  real_T poslim_data[12];
  emxArray_char_T_cartesian_tra_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[10] = { 'e', 'd', 'o', '_', 'l', 'i', 'n', 'k', '_',
    '3' };

  static const real_T tmp_1[9] = { 0.0123976159829631, 0.00022039108420015264,
    2.1332825710116445E-6, 0.00022039108420015264, 0.00014803877895089843,
    -9.2077339045129963E-5, 2.1332825710116445E-6, -9.2077339045129963E-5,
    0.012477575138803739 };

  static const real_T tmp_2[36] = { 0.0123976159829631, 0.00022039108420015264,
    2.1332825710116445E-6, 0.0, -2.56217202436532E-5, 0.0010295844637787021,
    0.00022039108420015264, 0.00014803877895089843, -9.2077339045129963E-5,
    2.56217202436532E-5, 0.0, 0.0024737535112545539, 2.1332825710116445E-6,
    -9.2077339045129963E-5, 0.012477575138803739, -0.0010295844637787021,
    -0.0024737535112545539, 0.0, 0.0, 2.56217202436532E-5,
    -0.0010295844637787021, 0.0785942338762368, 0.0, 0.0, -2.56217202436532E-5,
    0.0, -0.0024737535112545539, 0.0, 0.0785942338762368, 0.0,
    0.0010295844637787021, 0.0024737535112545539, 0.0, 0.0, 0.0,
    0.0785942338762368 };

  static const int8_T tmp_3[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_4[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n', 't',
    '_', '3' };

  static const char_T tmp_5[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_6[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_7[16] = { 1.0, 2.0682310711021444E-13,
    2.0682310711021444E-13, 0.0, 2.0682310711021444E-13, -1.0, -0.0, 0.0,
    2.0682310711021444E-13, 4.2775797634723234E-26, -1.0, 0.0, 0.0, 0.2105, 0.0,
    1.0 };

  static const real_T tmp_8[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_9[36] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 3.0;
  obj->MassInternal = 0.0785942338762368;
  obj->CenterOfMassInternal[0] = -0.031475;
  obj->CenterOfMassInternal[1] = 0.0131;
  obj->CenterOfMassInternal[2] = 0.000326;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_2[b_kstr];
  }

  obj->JointInternal.InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  b_kstr = obj->JointInternal.NameInternal->size[0] *
    obj->JointInternal.NameInternal->size[1];
  obj->JointInternal.NameInternal->size[0] = 1;
  obj->JointInternal.NameInternal->size[1] = 11;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 11; b_kstr++) {
    obj->JointInternal.NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_5[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_5[b_kstr];
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
      b_0[b_kstr] = tmp_6[b_kstr];
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

  cartesian_trajec_emxFree_char_T(&switch_expression);
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

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    obj->JointInternal.VelocityNumber = 1.0;
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

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    obj->JointInternal.VelocityNumber = 0.0;
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    obj->JointInternal.PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 1; b_kstr++) {
    obj->JointInternal.HomePositionInternal->data[0] = 0.0;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_7[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_8[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_9[b_kstr];
  }

  obj->JointInternal.InTree = true;
  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr);
  obj->JointInternal.PositionLimitsInternal->data[0] = -1.71042266695;
  obj->JointInternal.PositionLimitsInternal->data
    [obj->JointInternal.PositionLimitsInternal->size[0]] = 1.71042266695;
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr);
  obj->JointInternal.HomePositionInternal->data[0] = 0.0;
  return b_obj;
}

static v_robotics_manip_internal_Rig_T *c_RigidBody_Rigid_l
  (v_robotics_manip_internal_Rig_T *obj)
{
  v_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  real_T poslim_data[12];
  emxArray_char_T_cartesian_tra_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[10] = { 'e', 'd', 'o', '_', 'l', 'i', 'n', 'k', '_',
    '4' };

  static const real_T tmp_1[9] = { 0.0123990349928174, -2.7766271471639167E-6,
    0.00022466935228286869, -2.7766271471639167E-6, 0.012491487094789458,
    9.2330220708293281E-5, 0.00022466935228286869, 9.2330220708293281E-5,
    0.00016056153744711284 };

  static const real_T tmp_2[36] = { 0.0123990349928174, -2.7766271471639167E-6,
    0.00022466935228286869, 0.0, 0.0010818496293063995, 4.275526322867282E-5,
    -2.7766271471639167E-6, 0.012491487094789458, 9.2330220708293281E-5,
    -0.0010818496293063995, 0.0, -0.0026650518765093138, 0.00022466935228286869,
    9.2330220708293281E-5, 0.00016056153744711284, -4.275526322867282E-5,
    0.0026650518765093138, 0.0, 0.0, -0.0010818496293063995,
    -4.275526322867282E-5, 0.0785942338762368, 0.0, 0.0, 0.0010818496293063995,
    0.0, 0.0026650518765093138, 0.0, 0.0785942338762368, 0.0,
    4.275526322867282E-5, -0.0026650518765093138, 0.0, 0.0, 0.0,
    0.0785942338762368 };

  static const int8_T tmp_3[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_4[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n', 't',
    '_', '4' };

  static const char_T tmp_5[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_6[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_7[16] = { 1.0, 0.0, -0.0, 0.0, 0.0,
    4.8965888601467475E-12, 1.0, 0.0, 0.0, -1.0, 4.8965888601467475E-12, 0.0,
    0.0, -0.268, 0.0, 1.0 };

  static const real_T tmp_8[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_9[36] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 4.0;
  obj->MassInternal = 0.0785942338762368;
  obj->CenterOfMassInternal[0] = 0.033909;
  obj->CenterOfMassInternal[1] = 0.000544;
  obj->CenterOfMassInternal[2] = -0.013765;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_2[b_kstr];
  }

  obj->JointInternal.InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  b_kstr = obj->JointInternal.NameInternal->size[0] *
    obj->JointInternal.NameInternal->size[1];
  obj->JointInternal.NameInternal->size[0] = 1;
  obj->JointInternal.NameInternal->size[1] = 11;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 11; b_kstr++) {
    obj->JointInternal.NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_5[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_5[b_kstr];
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
      b_0[b_kstr] = tmp_6[b_kstr];
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

  cartesian_trajec_emxFree_char_T(&switch_expression);
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

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    obj->JointInternal.VelocityNumber = 1.0;
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

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    obj->JointInternal.VelocityNumber = 0.0;
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    obj->JointInternal.PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 1; b_kstr++) {
    obj->JointInternal.HomePositionInternal->data[0] = 0.0;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_7[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_8[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_9[b_kstr];
  }

  obj->JointInternal.InTree = true;
  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr);
  obj->JointInternal.PositionLimitsInternal->data[0] = -3.10668606855;
  obj->JointInternal.PositionLimitsInternal->data
    [obj->JointInternal.PositionLimitsInternal->size[0]] = 3.10668606855;
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr);
  obj->JointInternal.HomePositionInternal->data[0] = 0.0;
  return b_obj;
}

static v_robotics_manip_internal_Rig_T *c_RigidBody_Rigid_lh
  (v_robotics_manip_internal_Rig_T *obj)
{
  v_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  real_T poslim_data[12];
  emxArray_char_T_cartesian_tra_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[10] = { 'e', 'd', 'o', '_', 'l', 'i', 'n', 'k', '_',
    '5' };

  static const real_T tmp_1[9] = { 0.012440329403329006, 2.388185677857016E-6,
    0.00012602126519218373, 2.388185677857016E-6, 0.012510746127660349,
    -9.077919321137075E-5, 0.00012602126519218373, -9.077919321137075E-5,
    0.00013851261456175015 };

  static const real_T tmp_2[36] = { 0.012440329403329006, 2.388185677857016E-6,
    0.00012602126519218373, 0.0, -0.0021015312196166957, -3.5996159115316455E-5,
    2.388185677857016E-6, 0.012510746127660349, -9.077919321137075E-5,
    0.0021015312196166957, 0.0, -0.0023173509858408423, 0.00012602126519218373,
    -9.077919321137075E-5, 0.00013851261456175015, 3.5996159115316455E-5,
    0.0023173509858408423, 0.0, 0.0, 0.0021015312196166957,
    3.5996159115316455E-5, 0.0785942338762368, 0.0, 0.0, -0.0021015312196166957,
    0.0, 0.0023173509858408423, 0.0, 0.0785942338762368, 0.0,
    -3.5996159115316455E-5, -0.0023173509858408423, 0.0, 0.0, 0.0,
    0.0785942338762368 };

  static const int8_T tmp_3[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_4[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n', 't',
    '_', '5' };

  static const char_T tmp_5[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_6[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_7[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_8[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_9[36] = { 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 5.0;
  obj->MassInternal = 0.0785942338762368;
  obj->CenterOfMassInternal[0] = 0.029485;
  obj->CenterOfMassInternal[1] = -0.000458;
  obj->CenterOfMassInternal[2] = 0.026739;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_2[b_kstr];
  }

  obj->JointInternal.InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  b_kstr = obj->JointInternal.NameInternal->size[0] *
    obj->JointInternal.NameInternal->size[1];
  obj->JointInternal.NameInternal->size[0] = 1;
  obj->JointInternal.NameInternal->size[1] = 11;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 11; b_kstr++) {
    obj->JointInternal.NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_5[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_5[b_kstr];
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
      b_0[b_kstr] = tmp_6[b_kstr];
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

  cartesian_trajec_emxFree_char_T(&switch_expression);
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

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    obj->JointInternal.VelocityNumber = 1.0;
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

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    obj->JointInternal.VelocityNumber = 0.0;
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    obj->JointInternal.PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 1; b_kstr++) {
    obj->JointInternal.HomePositionInternal->data[0] = 0.0;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_7[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_8[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_9[b_kstr];
  }

  obj->JointInternal.InTree = true;
  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr);
  obj->JointInternal.PositionLimitsInternal->data[0] = -1.79768912955;
  obj->JointInternal.PositionLimitsInternal->data
    [obj->JointInternal.PositionLimitsInternal->size[0]] = 1.79768912955;
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 1.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr);
  obj->JointInternal.HomePositionInternal->data[0] = 0.0;
  return b_obj;
}

static v_robotics_manip_internal_Rig_T *c_RigidBody_Rigid_k
  (v_robotics_manip_internal_Rig_T *obj)
{
  v_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  real_T poslim_data[12];
  emxArray_char_T_cartesian_tra_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[10] = { 'e', 'd', 'o', '_', 'l', 'i', 'n', 'k', '_',
    '6' };

  static const real_T tmp_1[9] = { 1.0067862401982823E-5, 2.0153545938371486E-9,
    5.3285284099072352E-9, 2.0153545938371486E-9, 1.4574493611914028E-5,
    1.6742291194075022E-9, 5.3285284099072352E-9, 1.6742291194075022E-9,
    1.006193323459457E-5 };

  static const real_T tmp_2[36] = { 1.0067862401982823E-5, 2.0153545938371486E-9,
    5.3285284099072352E-9, 0.0, -1.6782149839359722E-7, -0.00026087851925284686,
    2.0153545938371486E-9, 1.4574493611914028E-5, 1.6742291194075022E-9,
    1.6782149839359722E-7, 0.0, -1.957917481258634E-7, 5.3285284099072352E-9,
    1.6742291194075022E-9, 1.006193323459457E-5, 0.00026087851925284686,
    1.957917481258634E-7, 0.0, 0.0, 1.6782149839359722E-7,
    0.00026087851925284686, 0.0279702497322662, 0.0, 0.0, -1.6782149839359722E-7,
    0.0, 1.957917481258634E-7, 0.0, 0.0279702497322662, 0.0,
    -0.00026087851925284686, -1.957917481258634E-7, 0.0, 0.0, 0.0,
    0.0279702497322662 };

  static const int8_T tmp_3[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_4[11] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n', 't',
    '_', '6' };

  static const char_T tmp_5[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_6[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_7[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.1745, 1.0 };

  static const real_T tmp_8[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_9[36] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 6.0;
  obj->MassInternal = 0.0279702497322662;
  obj->CenterOfMassInternal[0] = 7.0E-6;
  obj->CenterOfMassInternal[1] = -0.009327;
  obj->CenterOfMassInternal[2] = 6.0E-6;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_2[b_kstr];
  }

  obj->JointInternal.InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  b_kstr = obj->JointInternal.NameInternal->size[0] *
    obj->JointInternal.NameInternal->size[1];
  obj->JointInternal.NameInternal->size[0] = 1;
  obj->JointInternal.NameInternal->size[1] = 11;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 11; b_kstr++) {
    obj->JointInternal.NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_5[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_5[b_kstr];
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
      b_0[b_kstr] = tmp_6[b_kstr];
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

  cartesian_trajec_emxFree_char_T(&switch_expression);
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

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    obj->JointInternal.VelocityNumber = 1.0;
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

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    obj->JointInternal.VelocityNumber = 1.0;
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    obj->JointInternal.VelocityNumber = 0.0;
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    obj->JointInternal.PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 1; b_kstr++) {
    obj->JointInternal.HomePositionInternal->data[0] = 0.0;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_7[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_8[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_9[b_kstr];
  }

  obj->JointInternal.InTree = true;
  b_kstr = obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  obj->JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.PositionLimitsInternal,
    b_kstr);
  obj->JointInternal.PositionLimitsInternal->data[0] = -4.71238898038;
  obj->JointInternal.PositionLimitsInternal->data
    [obj->JointInternal.PositionLimitsInternal->size[0]] = 4.71238898038;
  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  b_kstr = obj->JointInternal.HomePositionInternal->size[0];
  obj->JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.HomePositionInternal,
    b_kstr);
  obj->JointInternal.HomePositionInternal->data[0] = 0.0;
  return b_obj;
}

static y_robotics_manip_internal_Rig_T *RigidBodyTree_RigidBodyTree_as
  (y_robotics_manip_internal_Rig_T *obj, v_robotics_manip_internal_Rig_T *iobj_0,
   v_robotics_manip_internal_Rig_T *iobj_1, v_robotics_manip_internal_Rig_T
   *iobj_2, v_robotics_manip_internal_Rig_T *iobj_3,
   v_robotics_manip_internal_Rig_T *iobj_4, v_robotics_manip_internal_Rig_T
   *iobj_5, v_robotics_manip_internal_Rig_T *iobj_6,
   v_robotics_manip_internal_Rig_T *iobj_7)
{
  y_robotics_manip_internal_Rig_T *b_obj;
  v_robotics_manip_internal_Rig_T *obj_0;
  int8_T msubspace_data[36];
  real_T poslim_data[12];
  int8_T b_I[9];
  emxArray_char_T_cartesian_tra_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[11] = { 'e', 'd', 'o', '_', 'l', 'i', 'n', 'k', '_',
    'e', 'e' };

  static const real_T tmp_1[36] = { 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0,
    -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0 };

  static const int8_T tmp_2[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_3[12] = { 'e', 'd', 'o', '_', 'j', 'o', 'i', 'n', 't',
    '_', 'e', 'e' };

  static const char_T tmp_4[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_5[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_6[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_7[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_8[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const char_T tmp_9[5] = { 'w', 'o', 'r', 'l', 'd' };

  static const char_T tmp_a[9] = { 'w', 'o', 'r', 'l', 'd', '_', 'j', 'n', 't' };

  int32_T exitg1;
  b_obj = obj;
  obj->Bodies[0] = RigidBody_RigidB_astwhqf2azttxa(iobj_7);
  obj->Bodies[0]->Index = 1.0;
  obj->Bodies[1] = RigidBody_Rigid_astwhqf2azttxab(iobj_0);
  obj->Bodies[1]->Index = 2.0;
  obj->Bodies[2] = c_RigidBody_Rigid_b(iobj_1);
  obj->Bodies[2]->Index = 3.0;
  obj->Bodies[3] = c_RigidBody_Rigid_i(iobj_2);
  obj->Bodies[3]->Index = 4.0;
  obj->Bodies[4] = c_RigidBody_Rigid_l(iobj_3);
  obj->Bodies[4]->Index = 5.0;
  obj->Bodies[5] = c_RigidBody_Rigid_lh(iobj_4);
  obj->Bodies[5]->Index = 6.0;
  obj->Bodies[6] = c_RigidBody_Rigid_k(iobj_5);
  obj->Bodies[6]->Index = 7.0;
  b_kstr = iobj_6->NameInternal->size[0] * iobj_6->NameInternal->size[1];
  iobj_6->NameInternal->size[0] = 1;
  iobj_6->NameInternal->size[1] = 11;
  cartes_emxEnsureCapacity_char_T(iobj_6->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 11; b_kstr++) {
    iobj_6->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  iobj_6->ParentIndex = 7.0;
  iobj_6->MassInternal = 0.0;
  iobj_6->CenterOfMassInternal[0] = 0.0;
  iobj_6->CenterOfMassInternal[1] = 0.0;
  iobj_6->CenterOfMassInternal[2] = 0.0;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    iobj_6->InertiaInternal[b_kstr] = 0.0;
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    iobj_6->SpatialInertia[b_kstr] = tmp_1[b_kstr];
  }

  iobj_6->JointInternal.InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_6->JointInternal.JointToParentTransform[b_kstr] = tmp_2[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_6->JointInternal.ChildToJointTransform[b_kstr] = tmp_2[b_kstr];
  }

  b_kstr = iobj_6->JointInternal.NameInternal->size[0] *
    iobj_6->JointInternal.NameInternal->size[1];
  iobj_6->JointInternal.NameInternal->size[0] = 1;
  iobj_6->JointInternal.NameInternal->size[1] = 12;
  cartes_emxEnsureCapacity_char_T(iobj_6->JointInternal.NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 12; b_kstr++) {
    iobj_6->JointInternal.NameInternal->data[b_kstr] = tmp_3[b_kstr];
  }

  b_kstr = iobj_6->JointInternal.Type->size[0] * iobj_6->
    JointInternal.Type->size[1];
  iobj_6->JointInternal.Type->size[0] = 1;
  iobj_6->JointInternal.Type->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(iobj_6->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_6->JointInternal.Type->data[b_kstr] = tmp_4[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_6->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_6->JointInternal.Type->size[0] * iobj_6->
    JointInternal.Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_6->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_5[b_kstr];
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
      b_0[b_kstr] = tmp_6[b_kstr];
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
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_6->JointInternal.VelocityNumber = 1.0;
    iobj_6->JointInternal.PositionNumber = 1.0;
    iobj_6->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_6->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_6->JointInternal.JointAxisInternal[2] = 1.0;
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

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_6->JointInternal.VelocityNumber = 1.0;
    iobj_6->JointInternal.PositionNumber = 1.0;
    iobj_6->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_6->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_6->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_6->JointInternal.VelocityNumber = 0.0;
    iobj_6->JointInternal.PositionNumber = 0.0;
    iobj_6->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_6->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_6->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_6->JointInternal.MotionSubspace->size[0] *
    iobj_6->JointInternal.MotionSubspace->size[1];
  iobj_6->JointInternal.MotionSubspace->size[0] = 6;
  iobj_6->JointInternal.MotionSubspace->size[1] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_6->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_6->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_6->JointInternal.PositionLimitsInternal->size[0] *
    iobj_6->JointInternal.PositionLimitsInternal->size[1];
  iobj_6->JointInternal.PositionLimitsInternal->size[0] = 1;
  iobj_6->JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(iobj_6->JointInternal.PositionLimitsInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_6->JointInternal.PositionLimitsInternal->data[b_kstr] =
      poslim_data[b_kstr];
  }

  b_kstr = iobj_6->JointInternal.HomePositionInternal->size[0];
  iobj_6->JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_6->JointInternal.HomePositionInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 1; b_kstr++) {
    iobj_6->JointInternal.HomePositionInternal->data[0] = 0.0;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_6->JointInternal.JointToParentTransform[b_kstr] = tmp_7[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_6->JointInternal.ChildToJointTransform[b_kstr] = tmp_8[b_kstr];
  }

  b_kstr = iobj_6->JointInternal.MotionSubspace->size[0] *
    iobj_6->JointInternal.MotionSubspace->size[1];
  iobj_6->JointInternal.MotionSubspace->size[0] = 6;
  iobj_6->JointInternal.MotionSubspace->size[1] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_6->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_6->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  iobj_6->JointInternal.InTree = true;
  b_kstr = iobj_6->JointInternal.PositionLimitsInternal->size[0] *
    iobj_6->JointInternal.PositionLimitsInternal->size[1];
  iobj_6->JointInternal.PositionLimitsInternal->size[0] = 1;
  iobj_6->JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(iobj_6->JointInternal.PositionLimitsInternal,
    b_kstr);
  iobj_6->JointInternal.PositionLimitsInternal->data[0] = 0.0;
  iobj_6->JointInternal.PositionLimitsInternal->data
    [iobj_6->JointInternal.PositionLimitsInternal->size[0]] = 0.0;
  iobj_6->JointInternal.JointAxisInternal[0] = 0.0;
  iobj_6->JointInternal.JointAxisInternal[1] = 0.0;
  iobj_6->JointInternal.JointAxisInternal[2] = 0.0;
  b_kstr = iobj_6->JointInternal.HomePositionInternal->size[0];
  iobj_6->JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_6->JointInternal.HomePositionInternal,
    b_kstr);
  iobj_6->JointInternal.HomePositionInternal->data[0] = 0.0;
  obj->Bodies[7] = iobj_6;
  obj->Bodies[7]->Index = 8.0;
  obj->NumBodies = 8.0;
  obj->Gravity[0] = 0.0;
  obj->Gravity[1] = 0.0;
  obj->Gravity[2] = 0.0;
  obj_0 = &obj->Base;
  b_kstr = obj->Base.NameInternal->size[0] * obj->Base.NameInternal->size[1];
  obj->Base.NameInternal->size[0] = 1;
  obj->Base.NameInternal->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(obj->Base.NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->Base.NameInternal->data[b_kstr] = tmp_9[b_kstr];
  }

  obj->Base.JointInternal.InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj_0->JointInternal.JointToParentTransform[b_kstr] = tmp_2[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj_0->JointInternal.ChildToJointTransform[b_kstr] = tmp_2[b_kstr];
  }

  b_kstr = obj->Base.JointInternal.NameInternal->size[0] *
    obj->Base.JointInternal.NameInternal->size[1];
  obj->Base.JointInternal.NameInternal->size[0] = 1;
  obj->Base.JointInternal.NameInternal->size[1] = 9;
  cartes_emxEnsureCapacity_char_T(obj->Base.JointInternal.NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj_0->JointInternal.NameInternal->data[b_kstr] = tmp_a[b_kstr];
  }

  b_kstr = obj->Base.JointInternal.Type->size[0] * obj->
    Base.JointInternal.Type->size[1];
  obj->Base.JointInternal.Type->size[0] = 1;
  obj->Base.JointInternal.Type->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(obj->Base.JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj_0->JointInternal.Type->data[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->Base.JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->Base.JointInternal.Type->size[0] * obj->
    Base.JointInternal.Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj_0->JointInternal.Type->data[b_kstr];
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
      b_0[b_kstr] = tmp_6[b_kstr];
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

  cartesian_trajec_emxFree_char_T(&switch_expression);
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

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    obj->Base.JointInternal.VelocityNumber = 1.0;
    obj->Base.JointInternal.PositionNumber = 1.0;
    obj->Base.JointInternal.JointAxisInternal[0] = 0.0;
    obj->Base.JointInternal.JointAxisInternal[1] = 0.0;
    obj->Base.JointInternal.JointAxisInternal[2] = 1.0;
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

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    obj->Base.JointInternal.VelocityNumber = 1.0;
    obj->Base.JointInternal.PositionNumber = 1.0;
    obj->Base.JointInternal.JointAxisInternal[0] = 0.0;
    obj->Base.JointInternal.JointAxisInternal[1] = 0.0;
    obj->Base.JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    obj->Base.JointInternal.VelocityNumber = 0.0;
    obj->Base.JointInternal.PositionNumber = 0.0;
    obj->Base.JointInternal.JointAxisInternal[0] = 0.0;
    obj->Base.JointInternal.JointAxisInternal[1] = 0.0;
    obj->Base.JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->Base.JointInternal.MotionSubspace->size[0] *
    obj->Base.JointInternal.MotionSubspace->size[1];
  obj->Base.JointInternal.MotionSubspace->size[0] = 6;
  obj->Base.JointInternal.MotionSubspace->size[1] = 1;
  cartes_emxEnsureCapacity_real_T(obj->Base.JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj_0->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = obj->Base.JointInternal.PositionLimitsInternal->size[0] *
    obj->Base.JointInternal.PositionLimitsInternal->size[1];
  obj->Base.JointInternal.PositionLimitsInternal->size[0] = 1;
  obj->Base.JointInternal.PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->Base.JointInternal.PositionLimitsInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    obj_0->JointInternal.PositionLimitsInternal->data[b_kstr] =
      poslim_data[b_kstr];
  }

  b_kstr = obj->Base.JointInternal.HomePositionInternal->size[0];
  obj->Base.JointInternal.HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(obj->Base.JointInternal.HomePositionInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 1; b_kstr++) {
    obj_0->JointInternal.HomePositionInternal->data[0] = 0.0;
  }

  obj->Base.Index = -1.0;
  obj->Base.ParentIndex = -1.0;
  obj->Base.MassInternal = 1.0;
  obj->Base.CenterOfMassInternal[0] = 0.0;
  obj->Base.CenterOfMassInternal[1] = 0.0;
  obj->Base.CenterOfMassInternal[2] = 0.0;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    b_I[b_kstr] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->Base.InertiaInternal[b_kstr] = b_I[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    msubspace_data[b_kstr] = 0;
  }

  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    msubspace_data[b_kstr + 6 * b_kstr] = 1;
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->Base.SpatialInertia[b_kstr] = msubspace_data[b_kstr];
  }

  return b_obj;
}

static void cartesian_trajectory_plann_rand(real_T r[5])
{
  for (cartesian_trajectory_planner_B.b_k_i = 0;
       cartesian_trajectory_planner_B.b_k_i < 5;
       cartesian_trajectory_planner_B.b_k_i++) {
    memcpy(&cartesian_trajectory_planner_B.uv[0],
           &cartesian_trajectory_planner_DW.state_m[0], 625U * sizeof(uint32_T));
    cartesian__eml_rand_mt19937ar_a(cartesian_trajectory_planner_B.uv,
      cartesian_trajectory_planner_DW.state_m,
      &r[cartesian_trajectory_planner_B.b_k_i]);
  }
}

static w_robotics_manip_internal_Rig_T *c_RigidBody_Rigid_h
  (w_robotics_manip_internal_Rig_T *obj, c_rigidBodyJoint_cartesian_as_T *iobj_0)
{
  w_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  real_T poslim_data[12];
  emxArray_char_T_cartesian_tra_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x01' };

  static const int8_T tmp_1[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_2[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x01', '_', 'j', 'n', 't' };

  static const char_T tmp_3[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_4[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_5[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  iobj_0->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_0->JointToParentTransform[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_0->ChildToJointTransform[b_kstr] = tmp_1[b_kstr];
  }

  b_kstr = iobj_0->NameInternal->size[0] * iobj_0->NameInternal->size[1];
  iobj_0->NameInternal->size[0] = 1;
  iobj_0->NameInternal->size[1] = 14;
  cartes_emxEnsureCapacity_char_T(iobj_0->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    iobj_0->NameInternal->data[b_kstr] = tmp_2[b_kstr];
  }

  b_kstr = iobj_0->Type->size[0] * iobj_0->Type->size[1];
  iobj_0->Type->size[0] = 1;
  iobj_0->Type->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(iobj_0->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_0->Type->data[b_kstr] = tmp_3[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_0->Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_0->Type->size[0] * iobj_0->Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_0->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_4[b_kstr];
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
      b_0[b_kstr] = tmp_5[b_kstr];
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

  cartesian_trajec_emxFree_char_T(&switch_expression);
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

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_0->VelocityNumber = 1.0;
    iobj_0->PositionNumber = 1.0;
    iobj_0->JointAxisInternal[0] = 0.0;
    iobj_0->JointAxisInternal[1] = 0.0;
    iobj_0->JointAxisInternal[2] = 1.0;
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

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_0->VelocityNumber = 1.0;
    iobj_0->PositionNumber = 1.0;
    iobj_0->JointAxisInternal[0] = 0.0;
    iobj_0->JointAxisInternal[1] = 0.0;
    iobj_0->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_0->VelocityNumber = 0.0;
    iobj_0->PositionNumber = 0.0;
    iobj_0->JointAxisInternal[0] = 0.0;
    iobj_0->JointAxisInternal[1] = 0.0;
    iobj_0->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_0->MotionSubspace->size[0] * iobj_0->MotionSubspace->size[1];
  iobj_0->MotionSubspace->size[0] = 6;
  iobj_0->MotionSubspace->size[1] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_0->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_0->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_0->PositionLimitsInternal->size[0] *
    iobj_0->PositionLimitsInternal->size[1];
  iobj_0->PositionLimitsInternal->size[0] = 1;
  iobj_0->PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(iobj_0->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_0->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_0->HomePositionInternal->size[0];
  iobj_0->HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_0->HomePositionInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 1; b_kstr++) {
    iobj_0->HomePositionInternal->data[0] = 0.0;
  }

  obj->JointInternal = iobj_0;
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  return b_obj;
}

static w_robotics_manip_internal_Rig_T *c_RigidBody_Rigid_h2
  (w_robotics_manip_internal_Rig_T *obj, c_rigidBodyJoint_cartesian_as_T *iobj_0)
{
  w_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  real_T poslim_data[12];
  emxArray_char_T_cartesian_tra_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x02' };

  static const int8_T tmp_1[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_2[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x02', '_', 'j', 'n', 't' };

  static const char_T tmp_3[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_4[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_5[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  iobj_0->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_0->JointToParentTransform[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_0->ChildToJointTransform[b_kstr] = tmp_1[b_kstr];
  }

  b_kstr = iobj_0->NameInternal->size[0] * iobj_0->NameInternal->size[1];
  iobj_0->NameInternal->size[0] = 1;
  iobj_0->NameInternal->size[1] = 14;
  cartes_emxEnsureCapacity_char_T(iobj_0->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    iobj_0->NameInternal->data[b_kstr] = tmp_2[b_kstr];
  }

  b_kstr = iobj_0->Type->size[0] * iobj_0->Type->size[1];
  iobj_0->Type->size[0] = 1;
  iobj_0->Type->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(iobj_0->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_0->Type->data[b_kstr] = tmp_3[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_0->Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_0->Type->size[0] * iobj_0->Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_0->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_4[b_kstr];
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
      b_0[b_kstr] = tmp_5[b_kstr];
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

  cartesian_trajec_emxFree_char_T(&switch_expression);
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

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_0->VelocityNumber = 1.0;
    iobj_0->PositionNumber = 1.0;
    iobj_0->JointAxisInternal[0] = 0.0;
    iobj_0->JointAxisInternal[1] = 0.0;
    iobj_0->JointAxisInternal[2] = 1.0;
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

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_0->VelocityNumber = 1.0;
    iobj_0->PositionNumber = 1.0;
    iobj_0->JointAxisInternal[0] = 0.0;
    iobj_0->JointAxisInternal[1] = 0.0;
    iobj_0->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_0->VelocityNumber = 0.0;
    iobj_0->PositionNumber = 0.0;
    iobj_0->JointAxisInternal[0] = 0.0;
    iobj_0->JointAxisInternal[1] = 0.0;
    iobj_0->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_0->MotionSubspace->size[0] * iobj_0->MotionSubspace->size[1];
  iobj_0->MotionSubspace->size[0] = 6;
  iobj_0->MotionSubspace->size[1] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_0->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_0->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_0->PositionLimitsInternal->size[0] *
    iobj_0->PositionLimitsInternal->size[1];
  iobj_0->PositionLimitsInternal->size[0] = 1;
  iobj_0->PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(iobj_0->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_0->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_0->HomePositionInternal->size[0];
  iobj_0->HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_0->HomePositionInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 1; b_kstr++) {
    iobj_0->HomePositionInternal->data[0] = 0.0;
  }

  obj->JointInternal = iobj_0;
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  return b_obj;
}

static w_robotics_manip_internal_Rig_T *c_RigidBody_Rigid_m
  (w_robotics_manip_internal_Rig_T *obj, c_rigidBodyJoint_cartesian_as_T *iobj_0)
{
  w_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  real_T poslim_data[12];
  emxArray_char_T_cartesian_tra_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x03' };

  static const int8_T tmp_1[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_2[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x03', '_', 'j', 'n', 't' };

  static const char_T tmp_3[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_4[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_5[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  iobj_0->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_0->JointToParentTransform[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_0->ChildToJointTransform[b_kstr] = tmp_1[b_kstr];
  }

  b_kstr = iobj_0->NameInternal->size[0] * iobj_0->NameInternal->size[1];
  iobj_0->NameInternal->size[0] = 1;
  iobj_0->NameInternal->size[1] = 14;
  cartes_emxEnsureCapacity_char_T(iobj_0->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    iobj_0->NameInternal->data[b_kstr] = tmp_2[b_kstr];
  }

  b_kstr = iobj_0->Type->size[0] * iobj_0->Type->size[1];
  iobj_0->Type->size[0] = 1;
  iobj_0->Type->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(iobj_0->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_0->Type->data[b_kstr] = tmp_3[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_0->Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_0->Type->size[0] * iobj_0->Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_0->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_4[b_kstr];
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
      b_0[b_kstr] = tmp_5[b_kstr];
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

  cartesian_trajec_emxFree_char_T(&switch_expression);
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

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_0->VelocityNumber = 1.0;
    iobj_0->PositionNumber = 1.0;
    iobj_0->JointAxisInternal[0] = 0.0;
    iobj_0->JointAxisInternal[1] = 0.0;
    iobj_0->JointAxisInternal[2] = 1.0;
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

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_0->VelocityNumber = 1.0;
    iobj_0->PositionNumber = 1.0;
    iobj_0->JointAxisInternal[0] = 0.0;
    iobj_0->JointAxisInternal[1] = 0.0;
    iobj_0->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_0->VelocityNumber = 0.0;
    iobj_0->PositionNumber = 0.0;
    iobj_0->JointAxisInternal[0] = 0.0;
    iobj_0->JointAxisInternal[1] = 0.0;
    iobj_0->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_0->MotionSubspace->size[0] * iobj_0->MotionSubspace->size[1];
  iobj_0->MotionSubspace->size[0] = 6;
  iobj_0->MotionSubspace->size[1] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_0->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_0->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_0->PositionLimitsInternal->size[0] *
    iobj_0->PositionLimitsInternal->size[1];
  iobj_0->PositionLimitsInternal->size[0] = 1;
  iobj_0->PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(iobj_0->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_0->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_0->HomePositionInternal->size[0];
  iobj_0->HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_0->HomePositionInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 1; b_kstr++) {
    iobj_0->HomePositionInternal->data[0] = 0.0;
  }

  obj->JointInternal = iobj_0;
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  return b_obj;
}

static w_robotics_manip_internal_Rig_T *c_RigidBody_Rigid_o
  (w_robotics_manip_internal_Rig_T *obj, c_rigidBodyJoint_cartesian_as_T *iobj_0)
{
  w_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  real_T poslim_data[12];
  emxArray_char_T_cartesian_tra_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x04' };

  static const int8_T tmp_1[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_2[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x04', '_', 'j', 'n', 't' };

  static const char_T tmp_3[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_4[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_5[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  iobj_0->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_0->JointToParentTransform[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_0->ChildToJointTransform[b_kstr] = tmp_1[b_kstr];
  }

  b_kstr = iobj_0->NameInternal->size[0] * iobj_0->NameInternal->size[1];
  iobj_0->NameInternal->size[0] = 1;
  iobj_0->NameInternal->size[1] = 14;
  cartes_emxEnsureCapacity_char_T(iobj_0->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    iobj_0->NameInternal->data[b_kstr] = tmp_2[b_kstr];
  }

  b_kstr = iobj_0->Type->size[0] * iobj_0->Type->size[1];
  iobj_0->Type->size[0] = 1;
  iobj_0->Type->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(iobj_0->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_0->Type->data[b_kstr] = tmp_3[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_0->Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_0->Type->size[0] * iobj_0->Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_0->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_4[b_kstr];
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
      b_0[b_kstr] = tmp_5[b_kstr];
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

  cartesian_trajec_emxFree_char_T(&switch_expression);
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

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_0->VelocityNumber = 1.0;
    iobj_0->PositionNumber = 1.0;
    iobj_0->JointAxisInternal[0] = 0.0;
    iobj_0->JointAxisInternal[1] = 0.0;
    iobj_0->JointAxisInternal[2] = 1.0;
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

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_0->VelocityNumber = 1.0;
    iobj_0->PositionNumber = 1.0;
    iobj_0->JointAxisInternal[0] = 0.0;
    iobj_0->JointAxisInternal[1] = 0.0;
    iobj_0->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_0->VelocityNumber = 0.0;
    iobj_0->PositionNumber = 0.0;
    iobj_0->JointAxisInternal[0] = 0.0;
    iobj_0->JointAxisInternal[1] = 0.0;
    iobj_0->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_0->MotionSubspace->size[0] * iobj_0->MotionSubspace->size[1];
  iobj_0->MotionSubspace->size[0] = 6;
  iobj_0->MotionSubspace->size[1] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_0->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_0->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_0->PositionLimitsInternal->size[0] *
    iobj_0->PositionLimitsInternal->size[1];
  iobj_0->PositionLimitsInternal->size[0] = 1;
  iobj_0->PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(iobj_0->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_0->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_0->HomePositionInternal->size[0];
  iobj_0->HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_0->HomePositionInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 1; b_kstr++) {
    iobj_0->HomePositionInternal->data[0] = 0.0;
  }

  obj->JointInternal = iobj_0;
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  return b_obj;
}

static w_robotics_manip_internal_Rig_T *c_RigidBody_Rigid_d
  (w_robotics_manip_internal_Rig_T *obj, c_rigidBodyJoint_cartesian_as_T *iobj_0)
{
  w_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  real_T poslim_data[12];
  emxArray_char_T_cartesian_tra_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x05' };

  static const int8_T tmp_1[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_2[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x05', '_', 'j', 'n', 't' };

  static const char_T tmp_3[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_4[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_5[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  iobj_0->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_0->JointToParentTransform[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_0->ChildToJointTransform[b_kstr] = tmp_1[b_kstr];
  }

  b_kstr = iobj_0->NameInternal->size[0] * iobj_0->NameInternal->size[1];
  iobj_0->NameInternal->size[0] = 1;
  iobj_0->NameInternal->size[1] = 14;
  cartes_emxEnsureCapacity_char_T(iobj_0->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    iobj_0->NameInternal->data[b_kstr] = tmp_2[b_kstr];
  }

  b_kstr = iobj_0->Type->size[0] * iobj_0->Type->size[1];
  iobj_0->Type->size[0] = 1;
  iobj_0->Type->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(iobj_0->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_0->Type->data[b_kstr] = tmp_3[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_0->Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_0->Type->size[0] * iobj_0->Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_0->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_4[b_kstr];
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
      b_0[b_kstr] = tmp_5[b_kstr];
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

  cartesian_trajec_emxFree_char_T(&switch_expression);
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

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_0->VelocityNumber = 1.0;
    iobj_0->PositionNumber = 1.0;
    iobj_0->JointAxisInternal[0] = 0.0;
    iobj_0->JointAxisInternal[1] = 0.0;
    iobj_0->JointAxisInternal[2] = 1.0;
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

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_0->VelocityNumber = 1.0;
    iobj_0->PositionNumber = 1.0;
    iobj_0->JointAxisInternal[0] = 0.0;
    iobj_0->JointAxisInternal[1] = 0.0;
    iobj_0->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_0->VelocityNumber = 0.0;
    iobj_0->PositionNumber = 0.0;
    iobj_0->JointAxisInternal[0] = 0.0;
    iobj_0->JointAxisInternal[1] = 0.0;
    iobj_0->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_0->MotionSubspace->size[0] * iobj_0->MotionSubspace->size[1];
  iobj_0->MotionSubspace->size[0] = 6;
  iobj_0->MotionSubspace->size[1] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_0->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_0->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_0->PositionLimitsInternal->size[0] *
    iobj_0->PositionLimitsInternal->size[1];
  iobj_0->PositionLimitsInternal->size[0] = 1;
  iobj_0->PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(iobj_0->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_0->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_0->HomePositionInternal->size[0];
  iobj_0->HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_0->HomePositionInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 1; b_kstr++) {
    iobj_0->HomePositionInternal->data[0] = 0.0;
  }

  obj->JointInternal = iobj_0;
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  return b_obj;
}

static void ca_RigidBodyTree_clearAllBodies(x_robotics_manip_internal_Rig_T *obj,
  w_robotics_manip_internal_Rig_T *iobj_0, w_robotics_manip_internal_Rig_T
  *iobj_1, w_robotics_manip_internal_Rig_T *iobj_2,
  w_robotics_manip_internal_Rig_T *iobj_3, w_robotics_manip_internal_Rig_T
  *iobj_4, w_robotics_manip_internal_Rig_T *iobj_5,
  w_robotics_manip_internal_Rig_T *iobj_6, c_rigidBodyJoint_cartesian_as_T
  *iobj_7, c_rigidBodyJoint_cartesian_as_T *iobj_8,
  c_rigidBodyJoint_cartesian_as_T *iobj_9, c_rigidBodyJoint_cartesian_as_T
  *iobj_10, c_rigidBodyJoint_cartesian_as_T *iobj_11,
  c_rigidBodyJoint_cartesian_as_T *iobj_12, c_rigidBodyJoint_cartesian_as_T
  *iobj_13, c_rigidBodyJoint_cartesian_as_T *iobj_14,
  w_robotics_manip_internal_Rig_T *iobj_15)
{
  emxArray_char_T_cartesian_tra_T *switch_expression;
  static const char_T tmp[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x06' };

  static const int8_T tmp_0[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_1[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x06', '_', 'j', 'n', 't' };

  static const char_T tmp_2[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_3[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_4[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_5[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x07' };

  static const char_T tmp_6[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x07', '_', 'j', 'n', 't' };

  static const char_T tmp_7[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x08' };

  static const char_T tmp_8[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x08', '_', 'j', 'n', 't' };

  int32_T exitg1;
  obj->Bodies[0] = c_RigidBody_Rigid_h(iobj_15, iobj_14);
  obj->Bodies[1] = c_RigidBody_Rigid_h2(iobj_0, iobj_7);
  obj->Bodies[2] = c_RigidBody_Rigid_m(iobj_1, iobj_8);
  obj->Bodies[3] = c_RigidBody_Rigid_o(iobj_2, iobj_9);
  obj->Bodies[4] = c_RigidBody_Rigid_d(iobj_3, iobj_10);
  cartesian_trajectory_planner_B.b_kstr_l = iobj_4->NameInternal->size[0] *
    iobj_4->NameInternal->size[1];
  iobj_4->NameInternal->size[0] = 1;
  iobj_4->NameInternal->size[1] = 10;
  cartes_emxEnsureCapacity_char_T(iobj_4->NameInternal,
    cartesian_trajectory_planner_B.b_kstr_l);
  for (cartesian_trajectory_planner_B.b_kstr_l = 0;
       cartesian_trajectory_planner_B.b_kstr_l < 10;
       cartesian_trajectory_planner_B.b_kstr_l++) {
    iobj_4->NameInternal->data[cartesian_trajectory_planner_B.b_kstr_l] =
      tmp[cartesian_trajectory_planner_B.b_kstr_l];
  }

  iobj_11->InTree = false;
  for (cartesian_trajectory_planner_B.b_kstr_l = 0;
       cartesian_trajectory_planner_B.b_kstr_l < 16;
       cartesian_trajectory_planner_B.b_kstr_l++) {
    iobj_11->JointToParentTransform[cartesian_trajectory_planner_B.b_kstr_l] =
      tmp_0[cartesian_trajectory_planner_B.b_kstr_l];
  }

  for (cartesian_trajectory_planner_B.b_kstr_l = 0;
       cartesian_trajectory_planner_B.b_kstr_l < 16;
       cartesian_trajectory_planner_B.b_kstr_l++) {
    iobj_11->ChildToJointTransform[cartesian_trajectory_planner_B.b_kstr_l] =
      tmp_0[cartesian_trajectory_planner_B.b_kstr_l];
  }

  cartesian_trajectory_planner_B.b_kstr_l = iobj_11->NameInternal->size[0] *
    iobj_11->NameInternal->size[1];
  iobj_11->NameInternal->size[0] = 1;
  iobj_11->NameInternal->size[1] = 14;
  cartes_emxEnsureCapacity_char_T(iobj_11->NameInternal,
    cartesian_trajectory_planner_B.b_kstr_l);
  for (cartesian_trajectory_planner_B.b_kstr_l = 0;
       cartesian_trajectory_planner_B.b_kstr_l < 14;
       cartesian_trajectory_planner_B.b_kstr_l++) {
    iobj_11->NameInternal->data[cartesian_trajectory_planner_B.b_kstr_l] =
      tmp_1[cartesian_trajectory_planner_B.b_kstr_l];
  }

  cartesian_trajectory_planner_B.b_kstr_l = iobj_11->Type->size[0] *
    iobj_11->Type->size[1];
  iobj_11->Type->size[0] = 1;
  iobj_11->Type->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(iobj_11->Type,
    cartesian_trajectory_planner_B.b_kstr_l);
  for (cartesian_trajectory_planner_B.b_kstr_l = 0;
       cartesian_trajectory_planner_B.b_kstr_l < 5;
       cartesian_trajectory_planner_B.b_kstr_l++) {
    iobj_11->Type->data[cartesian_trajectory_planner_B.b_kstr_l] =
      tmp_2[cartesian_trajectory_planner_B.b_kstr_l];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  cartesian_trajectory_planner_B.b_kstr_l = switch_expression->size[0] *
    switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_11->Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression,
    cartesian_trajectory_planner_B.b_kstr_l);
  cartesian_trajectory_planner_B.loop_ub_j = iobj_11->Type->size[0] *
    iobj_11->Type->size[1] - 1;
  for (cartesian_trajectory_planner_B.b_kstr_l = 0;
       cartesian_trajectory_planner_B.b_kstr_l <=
       cartesian_trajectory_planner_B.loop_ub_j;
       cartesian_trajectory_planner_B.b_kstr_l++) {
    switch_expression->data[cartesian_trajectory_planner_B.b_kstr_l] =
      iobj_11->Type->data[cartesian_trajectory_planner_B.b_kstr_l];
  }

  for (cartesian_trajectory_planner_B.b_kstr_l = 0;
       cartesian_trajectory_planner_B.b_kstr_l < 8;
       cartesian_trajectory_planner_B.b_kstr_l++) {
    cartesian_trajectory_planner_B.b_p[cartesian_trajectory_planner_B.b_kstr_l] =
      tmp_3[cartesian_trajectory_planner_B.b_kstr_l];
  }

  cartesian_trajectory_planner_B.b_bool_o = false;
  if (switch_expression->size[1] == 8) {
    cartesian_trajectory_planner_B.b_kstr_l = 1;
    do {
      exitg1 = 0;
      if (cartesian_trajectory_planner_B.b_kstr_l - 1 < 8) {
        cartesian_trajectory_planner_B.loop_ub_j =
          cartesian_trajectory_planner_B.b_kstr_l - 1;
        if (switch_expression->data[cartesian_trajectory_planner_B.loop_ub_j] !=
            cartesian_trajectory_planner_B.b_p[cartesian_trajectory_planner_B.loop_ub_j])
        {
          exitg1 = 1;
        } else {
          cartesian_trajectory_planner_B.b_kstr_l++;
        }
      } else {
        cartesian_trajectory_planner_B.b_bool_o = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (cartesian_trajectory_planner_B.b_bool_o) {
    cartesian_trajectory_planner_B.b_kstr_l = 0;
  } else {
    for (cartesian_trajectory_planner_B.b_kstr_l = 0;
         cartesian_trajectory_planner_B.b_kstr_l < 9;
         cartesian_trajectory_planner_B.b_kstr_l++) {
      cartesian_trajectory_planner_B.b_n3[cartesian_trajectory_planner_B.b_kstr_l]
        = tmp_4[cartesian_trajectory_planner_B.b_kstr_l];
    }

    cartesian_trajectory_planner_B.b_bool_o = false;
    if (switch_expression->size[1] == 9) {
      cartesian_trajectory_planner_B.b_kstr_l = 1;
      do {
        exitg1 = 0;
        if (cartesian_trajectory_planner_B.b_kstr_l - 1 < 9) {
          cartesian_trajectory_planner_B.loop_ub_j =
            cartesian_trajectory_planner_B.b_kstr_l - 1;
          if (switch_expression->data[cartesian_trajectory_planner_B.loop_ub_j]
              !=
              cartesian_trajectory_planner_B.b_n3[cartesian_trajectory_planner_B.loop_ub_j])
          {
            exitg1 = 1;
          } else {
            cartesian_trajectory_planner_B.b_kstr_l++;
          }
        } else {
          cartesian_trajectory_planner_B.b_bool_o = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (cartesian_trajectory_planner_B.b_bool_o) {
      cartesian_trajectory_planner_B.b_kstr_l = 1;
    } else {
      cartesian_trajectory_planner_B.b_kstr_l = -1;
    }
  }

  switch (cartesian_trajectory_planner_B.b_kstr_l) {
   case 0:
    cartesian_trajectory_planner_B.iv1[0] = 0;
    cartesian_trajectory_planner_B.iv1[1] = 0;
    cartesian_trajectory_planner_B.iv1[2] = 1;
    cartesian_trajectory_planner_B.iv1[3] = 0;
    cartesian_trajectory_planner_B.iv1[4] = 0;
    cartesian_trajectory_planner_B.iv1[5] = 0;
    for (cartesian_trajectory_planner_B.b_kstr_l = 0;
         cartesian_trajectory_planner_B.b_kstr_l < 6;
         cartesian_trajectory_planner_B.b_kstr_l++) {
      cartesian_trajectory_planner_B.msubspace_data_m[cartesian_trajectory_planner_B.b_kstr_l]
        =
        cartesian_trajectory_planner_B.iv1[cartesian_trajectory_planner_B.b_kstr_l];
    }

    cartesian_trajectory_planner_B.poslim_data_o[0] = -3.1415926535897931;
    cartesian_trajectory_planner_B.poslim_data_o[1] = 3.1415926535897931;
    iobj_11->VelocityNumber = 1.0;
    iobj_11->PositionNumber = 1.0;
    iobj_11->JointAxisInternal[0] = 0.0;
    iobj_11->JointAxisInternal[1] = 0.0;
    iobj_11->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    cartesian_trajectory_planner_B.iv1[0] = 0;
    cartesian_trajectory_planner_B.iv1[1] = 0;
    cartesian_trajectory_planner_B.iv1[2] = 0;
    cartesian_trajectory_planner_B.iv1[3] = 0;
    cartesian_trajectory_planner_B.iv1[4] = 0;
    cartesian_trajectory_planner_B.iv1[5] = 1;
    for (cartesian_trajectory_planner_B.b_kstr_l = 0;
         cartesian_trajectory_planner_B.b_kstr_l < 6;
         cartesian_trajectory_planner_B.b_kstr_l++) {
      cartesian_trajectory_planner_B.msubspace_data_m[cartesian_trajectory_planner_B.b_kstr_l]
        =
        cartesian_trajectory_planner_B.iv1[cartesian_trajectory_planner_B.b_kstr_l];
    }

    cartesian_trajectory_planner_B.poslim_data_o[0] = -0.5;
    cartesian_trajectory_planner_B.poslim_data_o[1] = 0.5;
    iobj_11->VelocityNumber = 1.0;
    iobj_11->PositionNumber = 1.0;
    iobj_11->JointAxisInternal[0] = 0.0;
    iobj_11->JointAxisInternal[1] = 0.0;
    iobj_11->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (cartesian_trajectory_planner_B.b_kstr_l = 0;
         cartesian_trajectory_planner_B.b_kstr_l < 6;
         cartesian_trajectory_planner_B.b_kstr_l++) {
      cartesian_trajectory_planner_B.msubspace_data_m[cartesian_trajectory_planner_B.b_kstr_l]
        = 0;
    }

    cartesian_trajectory_planner_B.poslim_data_o[0] = 0.0;
    cartesian_trajectory_planner_B.poslim_data_o[1] = 0.0;
    iobj_11->VelocityNumber = 0.0;
    iobj_11->PositionNumber = 0.0;
    iobj_11->JointAxisInternal[0] = 0.0;
    iobj_11->JointAxisInternal[1] = 0.0;
    iobj_11->JointAxisInternal[2] = 0.0;
    break;
  }

  cartesian_trajectory_planner_B.b_kstr_l = iobj_11->MotionSubspace->size[0] *
    iobj_11->MotionSubspace->size[1];
  iobj_11->MotionSubspace->size[0] = 6;
  iobj_11->MotionSubspace->size[1] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_11->MotionSubspace,
    cartesian_trajectory_planner_B.b_kstr_l);
  for (cartesian_trajectory_planner_B.b_kstr_l = 0;
       cartesian_trajectory_planner_B.b_kstr_l < 6;
       cartesian_trajectory_planner_B.b_kstr_l++) {
    iobj_11->MotionSubspace->data[cartesian_trajectory_planner_B.b_kstr_l] =
      cartesian_trajectory_planner_B.msubspace_data_m[cartesian_trajectory_planner_B.b_kstr_l];
  }

  cartesian_trajectory_planner_B.b_kstr_l = iobj_11->
    PositionLimitsInternal->size[0] * iobj_11->PositionLimitsInternal->size[1];
  iobj_11->PositionLimitsInternal->size[0] = 1;
  iobj_11->PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(iobj_11->PositionLimitsInternal,
    cartesian_trajectory_planner_B.b_kstr_l);
  for (cartesian_trajectory_planner_B.b_kstr_l = 0;
       cartesian_trajectory_planner_B.b_kstr_l < 2;
       cartesian_trajectory_planner_B.b_kstr_l++) {
    iobj_11->PositionLimitsInternal->
      data[cartesian_trajectory_planner_B.b_kstr_l] =
      cartesian_trajectory_planner_B.poslim_data_o[cartesian_trajectory_planner_B.b_kstr_l];
  }

  cartesian_trajectory_planner_B.b_kstr_l = iobj_11->HomePositionInternal->size
    [0];
  iobj_11->HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_11->HomePositionInternal,
    cartesian_trajectory_planner_B.b_kstr_l);
  for (cartesian_trajectory_planner_B.b_kstr_l = 0;
       cartesian_trajectory_planner_B.b_kstr_l < 1;
       cartesian_trajectory_planner_B.b_kstr_l++) {
    iobj_11->HomePositionInternal->data[0] = 0.0;
  }

  iobj_4->JointInternal = iobj_11;
  iobj_4->Index = -1.0;
  iobj_4->ParentIndex = -1.0;
  obj->Bodies[5] = iobj_4;
  cartesian_trajectory_planner_B.b_kstr_l = iobj_5->NameInternal->size[0] *
    iobj_5->NameInternal->size[1];
  iobj_5->NameInternal->size[0] = 1;
  iobj_5->NameInternal->size[1] = 10;
  cartes_emxEnsureCapacity_char_T(iobj_5->NameInternal,
    cartesian_trajectory_planner_B.b_kstr_l);
  for (cartesian_trajectory_planner_B.b_kstr_l = 0;
       cartesian_trajectory_planner_B.b_kstr_l < 10;
       cartesian_trajectory_planner_B.b_kstr_l++) {
    iobj_5->NameInternal->data[cartesian_trajectory_planner_B.b_kstr_l] =
      tmp_5[cartesian_trajectory_planner_B.b_kstr_l];
  }

  iobj_12->InTree = false;
  for (cartesian_trajectory_planner_B.b_kstr_l = 0;
       cartesian_trajectory_planner_B.b_kstr_l < 16;
       cartesian_trajectory_planner_B.b_kstr_l++) {
    iobj_12->JointToParentTransform[cartesian_trajectory_planner_B.b_kstr_l] =
      tmp_0[cartesian_trajectory_planner_B.b_kstr_l];
  }

  for (cartesian_trajectory_planner_B.b_kstr_l = 0;
       cartesian_trajectory_planner_B.b_kstr_l < 16;
       cartesian_trajectory_planner_B.b_kstr_l++) {
    iobj_12->ChildToJointTransform[cartesian_trajectory_planner_B.b_kstr_l] =
      tmp_0[cartesian_trajectory_planner_B.b_kstr_l];
  }

  cartesian_trajectory_planner_B.b_kstr_l = iobj_12->NameInternal->size[0] *
    iobj_12->NameInternal->size[1];
  iobj_12->NameInternal->size[0] = 1;
  iobj_12->NameInternal->size[1] = 14;
  cartes_emxEnsureCapacity_char_T(iobj_12->NameInternal,
    cartesian_trajectory_planner_B.b_kstr_l);
  for (cartesian_trajectory_planner_B.b_kstr_l = 0;
       cartesian_trajectory_planner_B.b_kstr_l < 14;
       cartesian_trajectory_planner_B.b_kstr_l++) {
    iobj_12->NameInternal->data[cartesian_trajectory_planner_B.b_kstr_l] =
      tmp_6[cartesian_trajectory_planner_B.b_kstr_l];
  }

  cartesian_trajectory_planner_B.b_kstr_l = iobj_12->Type->size[0] *
    iobj_12->Type->size[1];
  iobj_12->Type->size[0] = 1;
  iobj_12->Type->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(iobj_12->Type,
    cartesian_trajectory_planner_B.b_kstr_l);
  for (cartesian_trajectory_planner_B.b_kstr_l = 0;
       cartesian_trajectory_planner_B.b_kstr_l < 5;
       cartesian_trajectory_planner_B.b_kstr_l++) {
    iobj_12->Type->data[cartesian_trajectory_planner_B.b_kstr_l] =
      tmp_2[cartesian_trajectory_planner_B.b_kstr_l];
  }

  cartesian_trajectory_planner_B.b_kstr_l = switch_expression->size[0] *
    switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_12->Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression,
    cartesian_trajectory_planner_B.b_kstr_l);
  cartesian_trajectory_planner_B.loop_ub_j = iobj_12->Type->size[0] *
    iobj_12->Type->size[1] - 1;
  for (cartesian_trajectory_planner_B.b_kstr_l = 0;
       cartesian_trajectory_planner_B.b_kstr_l <=
       cartesian_trajectory_planner_B.loop_ub_j;
       cartesian_trajectory_planner_B.b_kstr_l++) {
    switch_expression->data[cartesian_trajectory_planner_B.b_kstr_l] =
      iobj_12->Type->data[cartesian_trajectory_planner_B.b_kstr_l];
  }

  cartesian_trajectory_planner_B.b_bool_o = false;
  if (switch_expression->size[1] == 8) {
    cartesian_trajectory_planner_B.b_kstr_l = 1;
    do {
      exitg1 = 0;
      if (cartesian_trajectory_planner_B.b_kstr_l - 1 < 8) {
        cartesian_trajectory_planner_B.loop_ub_j =
          cartesian_trajectory_planner_B.b_kstr_l - 1;
        if (switch_expression->data[cartesian_trajectory_planner_B.loop_ub_j] !=
            cartesian_trajectory_planner_B.b_p[cartesian_trajectory_planner_B.loop_ub_j])
        {
          exitg1 = 1;
        } else {
          cartesian_trajectory_planner_B.b_kstr_l++;
        }
      } else {
        cartesian_trajectory_planner_B.b_bool_o = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (cartesian_trajectory_planner_B.b_bool_o) {
    cartesian_trajectory_planner_B.b_kstr_l = 0;
  } else {
    for (cartesian_trajectory_planner_B.b_kstr_l = 0;
         cartesian_trajectory_planner_B.b_kstr_l < 9;
         cartesian_trajectory_planner_B.b_kstr_l++) {
      cartesian_trajectory_planner_B.b_n3[cartesian_trajectory_planner_B.b_kstr_l]
        = tmp_4[cartesian_trajectory_planner_B.b_kstr_l];
    }

    cartesian_trajectory_planner_B.b_bool_o = false;
    if (switch_expression->size[1] == 9) {
      cartesian_trajectory_planner_B.b_kstr_l = 1;
      do {
        exitg1 = 0;
        if (cartesian_trajectory_planner_B.b_kstr_l - 1 < 9) {
          cartesian_trajectory_planner_B.loop_ub_j =
            cartesian_trajectory_planner_B.b_kstr_l - 1;
          if (switch_expression->data[cartesian_trajectory_planner_B.loop_ub_j]
              !=
              cartesian_trajectory_planner_B.b_n3[cartesian_trajectory_planner_B.loop_ub_j])
          {
            exitg1 = 1;
          } else {
            cartesian_trajectory_planner_B.b_kstr_l++;
          }
        } else {
          cartesian_trajectory_planner_B.b_bool_o = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (cartesian_trajectory_planner_B.b_bool_o) {
      cartesian_trajectory_planner_B.b_kstr_l = 1;
    } else {
      cartesian_trajectory_planner_B.b_kstr_l = -1;
    }
  }

  switch (cartesian_trajectory_planner_B.b_kstr_l) {
   case 0:
    cartesian_trajectory_planner_B.iv1[0] = 0;
    cartesian_trajectory_planner_B.iv1[1] = 0;
    cartesian_trajectory_planner_B.iv1[2] = 1;
    cartesian_trajectory_planner_B.iv1[3] = 0;
    cartesian_trajectory_planner_B.iv1[4] = 0;
    cartesian_trajectory_planner_B.iv1[5] = 0;
    for (cartesian_trajectory_planner_B.b_kstr_l = 0;
         cartesian_trajectory_planner_B.b_kstr_l < 6;
         cartesian_trajectory_planner_B.b_kstr_l++) {
      cartesian_trajectory_planner_B.msubspace_data_m[cartesian_trajectory_planner_B.b_kstr_l]
        =
        cartesian_trajectory_planner_B.iv1[cartesian_trajectory_planner_B.b_kstr_l];
    }

    cartesian_trajectory_planner_B.poslim_data_o[0] = -3.1415926535897931;
    cartesian_trajectory_planner_B.poslim_data_o[1] = 3.1415926535897931;
    iobj_12->VelocityNumber = 1.0;
    iobj_12->PositionNumber = 1.0;
    iobj_12->JointAxisInternal[0] = 0.0;
    iobj_12->JointAxisInternal[1] = 0.0;
    iobj_12->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    cartesian_trajectory_planner_B.iv1[0] = 0;
    cartesian_trajectory_planner_B.iv1[1] = 0;
    cartesian_trajectory_planner_B.iv1[2] = 0;
    cartesian_trajectory_planner_B.iv1[3] = 0;
    cartesian_trajectory_planner_B.iv1[4] = 0;
    cartesian_trajectory_planner_B.iv1[5] = 1;
    for (cartesian_trajectory_planner_B.b_kstr_l = 0;
         cartesian_trajectory_planner_B.b_kstr_l < 6;
         cartesian_trajectory_planner_B.b_kstr_l++) {
      cartesian_trajectory_planner_B.msubspace_data_m[cartesian_trajectory_planner_B.b_kstr_l]
        =
        cartesian_trajectory_planner_B.iv1[cartesian_trajectory_planner_B.b_kstr_l];
    }

    cartesian_trajectory_planner_B.poslim_data_o[0] = -0.5;
    cartesian_trajectory_planner_B.poslim_data_o[1] = 0.5;
    iobj_12->VelocityNumber = 1.0;
    iobj_12->PositionNumber = 1.0;
    iobj_12->JointAxisInternal[0] = 0.0;
    iobj_12->JointAxisInternal[1] = 0.0;
    iobj_12->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (cartesian_trajectory_planner_B.b_kstr_l = 0;
         cartesian_trajectory_planner_B.b_kstr_l < 6;
         cartesian_trajectory_planner_B.b_kstr_l++) {
      cartesian_trajectory_planner_B.msubspace_data_m[cartesian_trajectory_planner_B.b_kstr_l]
        = 0;
    }

    cartesian_trajectory_planner_B.poslim_data_o[0] = 0.0;
    cartesian_trajectory_planner_B.poslim_data_o[1] = 0.0;
    iobj_12->VelocityNumber = 0.0;
    iobj_12->PositionNumber = 0.0;
    iobj_12->JointAxisInternal[0] = 0.0;
    iobj_12->JointAxisInternal[1] = 0.0;
    iobj_12->JointAxisInternal[2] = 0.0;
    break;
  }

  cartesian_trajectory_planner_B.b_kstr_l = iobj_12->MotionSubspace->size[0] *
    iobj_12->MotionSubspace->size[1];
  iobj_12->MotionSubspace->size[0] = 6;
  iobj_12->MotionSubspace->size[1] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_12->MotionSubspace,
    cartesian_trajectory_planner_B.b_kstr_l);
  for (cartesian_trajectory_planner_B.b_kstr_l = 0;
       cartesian_trajectory_planner_B.b_kstr_l < 6;
       cartesian_trajectory_planner_B.b_kstr_l++) {
    iobj_12->MotionSubspace->data[cartesian_trajectory_planner_B.b_kstr_l] =
      cartesian_trajectory_planner_B.msubspace_data_m[cartesian_trajectory_planner_B.b_kstr_l];
  }

  cartesian_trajectory_planner_B.b_kstr_l = iobj_12->
    PositionLimitsInternal->size[0] * iobj_12->PositionLimitsInternal->size[1];
  iobj_12->PositionLimitsInternal->size[0] = 1;
  iobj_12->PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(iobj_12->PositionLimitsInternal,
    cartesian_trajectory_planner_B.b_kstr_l);
  for (cartesian_trajectory_planner_B.b_kstr_l = 0;
       cartesian_trajectory_planner_B.b_kstr_l < 2;
       cartesian_trajectory_planner_B.b_kstr_l++) {
    iobj_12->PositionLimitsInternal->
      data[cartesian_trajectory_planner_B.b_kstr_l] =
      cartesian_trajectory_planner_B.poslim_data_o[cartesian_trajectory_planner_B.b_kstr_l];
  }

  cartesian_trajectory_planner_B.b_kstr_l = iobj_12->HomePositionInternal->size
    [0];
  iobj_12->HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_12->HomePositionInternal,
    cartesian_trajectory_planner_B.b_kstr_l);
  for (cartesian_trajectory_planner_B.b_kstr_l = 0;
       cartesian_trajectory_planner_B.b_kstr_l < 1;
       cartesian_trajectory_planner_B.b_kstr_l++) {
    iobj_12->HomePositionInternal->data[0] = 0.0;
  }

  iobj_5->JointInternal = iobj_12;
  iobj_5->Index = -1.0;
  iobj_5->ParentIndex = -1.0;
  obj->Bodies[6] = iobj_5;
  cartesian_trajectory_planner_B.b_kstr_l = iobj_6->NameInternal->size[0] *
    iobj_6->NameInternal->size[1];
  iobj_6->NameInternal->size[0] = 1;
  iobj_6->NameInternal->size[1] = 10;
  cartes_emxEnsureCapacity_char_T(iobj_6->NameInternal,
    cartesian_trajectory_planner_B.b_kstr_l);
  for (cartesian_trajectory_planner_B.b_kstr_l = 0;
       cartesian_trajectory_planner_B.b_kstr_l < 10;
       cartesian_trajectory_planner_B.b_kstr_l++) {
    iobj_6->NameInternal->data[cartesian_trajectory_planner_B.b_kstr_l] =
      tmp_7[cartesian_trajectory_planner_B.b_kstr_l];
  }

  iobj_13->InTree = false;
  for (cartesian_trajectory_planner_B.b_kstr_l = 0;
       cartesian_trajectory_planner_B.b_kstr_l < 16;
       cartesian_trajectory_planner_B.b_kstr_l++) {
    iobj_13->JointToParentTransform[cartesian_trajectory_planner_B.b_kstr_l] =
      tmp_0[cartesian_trajectory_planner_B.b_kstr_l];
  }

  for (cartesian_trajectory_planner_B.b_kstr_l = 0;
       cartesian_trajectory_planner_B.b_kstr_l < 16;
       cartesian_trajectory_planner_B.b_kstr_l++) {
    iobj_13->ChildToJointTransform[cartesian_trajectory_planner_B.b_kstr_l] =
      tmp_0[cartesian_trajectory_planner_B.b_kstr_l];
  }

  cartesian_trajectory_planner_B.b_kstr_l = iobj_13->NameInternal->size[0] *
    iobj_13->NameInternal->size[1];
  iobj_13->NameInternal->size[0] = 1;
  iobj_13->NameInternal->size[1] = 14;
  cartes_emxEnsureCapacity_char_T(iobj_13->NameInternal,
    cartesian_trajectory_planner_B.b_kstr_l);
  for (cartesian_trajectory_planner_B.b_kstr_l = 0;
       cartesian_trajectory_planner_B.b_kstr_l < 14;
       cartesian_trajectory_planner_B.b_kstr_l++) {
    iobj_13->NameInternal->data[cartesian_trajectory_planner_B.b_kstr_l] =
      tmp_8[cartesian_trajectory_planner_B.b_kstr_l];
  }

  cartesian_trajectory_planner_B.b_kstr_l = iobj_13->Type->size[0] *
    iobj_13->Type->size[1];
  iobj_13->Type->size[0] = 1;
  iobj_13->Type->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(iobj_13->Type,
    cartesian_trajectory_planner_B.b_kstr_l);
  for (cartesian_trajectory_planner_B.b_kstr_l = 0;
       cartesian_trajectory_planner_B.b_kstr_l < 5;
       cartesian_trajectory_planner_B.b_kstr_l++) {
    iobj_13->Type->data[cartesian_trajectory_planner_B.b_kstr_l] =
      tmp_2[cartesian_trajectory_planner_B.b_kstr_l];
  }

  cartesian_trajectory_planner_B.b_kstr_l = switch_expression->size[0] *
    switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_13->Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression,
    cartesian_trajectory_planner_B.b_kstr_l);
  cartesian_trajectory_planner_B.loop_ub_j = iobj_13->Type->size[0] *
    iobj_13->Type->size[1] - 1;
  for (cartesian_trajectory_planner_B.b_kstr_l = 0;
       cartesian_trajectory_planner_B.b_kstr_l <=
       cartesian_trajectory_planner_B.loop_ub_j;
       cartesian_trajectory_planner_B.b_kstr_l++) {
    switch_expression->data[cartesian_trajectory_planner_B.b_kstr_l] =
      iobj_13->Type->data[cartesian_trajectory_planner_B.b_kstr_l];
  }

  cartesian_trajectory_planner_B.b_bool_o = false;
  if (switch_expression->size[1] == 8) {
    cartesian_trajectory_planner_B.b_kstr_l = 1;
    do {
      exitg1 = 0;
      if (cartesian_trajectory_planner_B.b_kstr_l - 1 < 8) {
        cartesian_trajectory_planner_B.loop_ub_j =
          cartesian_trajectory_planner_B.b_kstr_l - 1;
        if (switch_expression->data[cartesian_trajectory_planner_B.loop_ub_j] !=
            cartesian_trajectory_planner_B.b_p[cartesian_trajectory_planner_B.loop_ub_j])
        {
          exitg1 = 1;
        } else {
          cartesian_trajectory_planner_B.b_kstr_l++;
        }
      } else {
        cartesian_trajectory_planner_B.b_bool_o = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (cartesian_trajectory_planner_B.b_bool_o) {
    cartesian_trajectory_planner_B.b_kstr_l = 0;
  } else {
    for (cartesian_trajectory_planner_B.b_kstr_l = 0;
         cartesian_trajectory_planner_B.b_kstr_l < 9;
         cartesian_trajectory_planner_B.b_kstr_l++) {
      cartesian_trajectory_planner_B.b_n3[cartesian_trajectory_planner_B.b_kstr_l]
        = tmp_4[cartesian_trajectory_planner_B.b_kstr_l];
    }

    cartesian_trajectory_planner_B.b_bool_o = false;
    if (switch_expression->size[1] == 9) {
      cartesian_trajectory_planner_B.b_kstr_l = 1;
      do {
        exitg1 = 0;
        if (cartesian_trajectory_planner_B.b_kstr_l - 1 < 9) {
          cartesian_trajectory_planner_B.loop_ub_j =
            cartesian_trajectory_planner_B.b_kstr_l - 1;
          if (switch_expression->data[cartesian_trajectory_planner_B.loop_ub_j]
              !=
              cartesian_trajectory_planner_B.b_n3[cartesian_trajectory_planner_B.loop_ub_j])
          {
            exitg1 = 1;
          } else {
            cartesian_trajectory_planner_B.b_kstr_l++;
          }
        } else {
          cartesian_trajectory_planner_B.b_bool_o = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (cartesian_trajectory_planner_B.b_bool_o) {
      cartesian_trajectory_planner_B.b_kstr_l = 1;
    } else {
      cartesian_trajectory_planner_B.b_kstr_l = -1;
    }
  }

  cartesian_trajec_emxFree_char_T(&switch_expression);
  switch (cartesian_trajectory_planner_B.b_kstr_l) {
   case 0:
    cartesian_trajectory_planner_B.iv1[0] = 0;
    cartesian_trajectory_planner_B.iv1[1] = 0;
    cartesian_trajectory_planner_B.iv1[2] = 1;
    cartesian_trajectory_planner_B.iv1[3] = 0;
    cartesian_trajectory_planner_B.iv1[4] = 0;
    cartesian_trajectory_planner_B.iv1[5] = 0;
    for (cartesian_trajectory_planner_B.b_kstr_l = 0;
         cartesian_trajectory_planner_B.b_kstr_l < 6;
         cartesian_trajectory_planner_B.b_kstr_l++) {
      cartesian_trajectory_planner_B.msubspace_data_m[cartesian_trajectory_planner_B.b_kstr_l]
        =
        cartesian_trajectory_planner_B.iv1[cartesian_trajectory_planner_B.b_kstr_l];
    }

    cartesian_trajectory_planner_B.poslim_data_o[0] = -3.1415926535897931;
    cartesian_trajectory_planner_B.poslim_data_o[1] = 3.1415926535897931;
    iobj_13->VelocityNumber = 1.0;
    iobj_13->PositionNumber = 1.0;
    iobj_13->JointAxisInternal[0] = 0.0;
    iobj_13->JointAxisInternal[1] = 0.0;
    iobj_13->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    cartesian_trajectory_planner_B.iv1[0] = 0;
    cartesian_trajectory_planner_B.iv1[1] = 0;
    cartesian_trajectory_planner_B.iv1[2] = 0;
    cartesian_trajectory_planner_B.iv1[3] = 0;
    cartesian_trajectory_planner_B.iv1[4] = 0;
    cartesian_trajectory_planner_B.iv1[5] = 1;
    for (cartesian_trajectory_planner_B.b_kstr_l = 0;
         cartesian_trajectory_planner_B.b_kstr_l < 6;
         cartesian_trajectory_planner_B.b_kstr_l++) {
      cartesian_trajectory_planner_B.msubspace_data_m[cartesian_trajectory_planner_B.b_kstr_l]
        =
        cartesian_trajectory_planner_B.iv1[cartesian_trajectory_planner_B.b_kstr_l];
    }

    cartesian_trajectory_planner_B.poslim_data_o[0] = -0.5;
    cartesian_trajectory_planner_B.poslim_data_o[1] = 0.5;
    iobj_13->VelocityNumber = 1.0;
    iobj_13->PositionNumber = 1.0;
    iobj_13->JointAxisInternal[0] = 0.0;
    iobj_13->JointAxisInternal[1] = 0.0;
    iobj_13->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (cartesian_trajectory_planner_B.b_kstr_l = 0;
         cartesian_trajectory_planner_B.b_kstr_l < 6;
         cartesian_trajectory_planner_B.b_kstr_l++) {
      cartesian_trajectory_planner_B.msubspace_data_m[cartesian_trajectory_planner_B.b_kstr_l]
        = 0;
    }

    cartesian_trajectory_planner_B.poslim_data_o[0] = 0.0;
    cartesian_trajectory_planner_B.poslim_data_o[1] = 0.0;
    iobj_13->VelocityNumber = 0.0;
    iobj_13->PositionNumber = 0.0;
    iobj_13->JointAxisInternal[0] = 0.0;
    iobj_13->JointAxisInternal[1] = 0.0;
    iobj_13->JointAxisInternal[2] = 0.0;
    break;
  }

  cartesian_trajectory_planner_B.b_kstr_l = iobj_13->MotionSubspace->size[0] *
    iobj_13->MotionSubspace->size[1];
  iobj_13->MotionSubspace->size[0] = 6;
  iobj_13->MotionSubspace->size[1] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_13->MotionSubspace,
    cartesian_trajectory_planner_B.b_kstr_l);
  for (cartesian_trajectory_planner_B.b_kstr_l = 0;
       cartesian_trajectory_planner_B.b_kstr_l < 6;
       cartesian_trajectory_planner_B.b_kstr_l++) {
    iobj_13->MotionSubspace->data[cartesian_trajectory_planner_B.b_kstr_l] =
      cartesian_trajectory_planner_B.msubspace_data_m[cartesian_trajectory_planner_B.b_kstr_l];
  }

  cartesian_trajectory_planner_B.b_kstr_l = iobj_13->
    PositionLimitsInternal->size[0] * iobj_13->PositionLimitsInternal->size[1];
  iobj_13->PositionLimitsInternal->size[0] = 1;
  iobj_13->PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(iobj_13->PositionLimitsInternal,
    cartesian_trajectory_planner_B.b_kstr_l);
  for (cartesian_trajectory_planner_B.b_kstr_l = 0;
       cartesian_trajectory_planner_B.b_kstr_l < 2;
       cartesian_trajectory_planner_B.b_kstr_l++) {
    iobj_13->PositionLimitsInternal->
      data[cartesian_trajectory_planner_B.b_kstr_l] =
      cartesian_trajectory_planner_B.poslim_data_o[cartesian_trajectory_planner_B.b_kstr_l];
  }

  cartesian_trajectory_planner_B.b_kstr_l = iobj_13->HomePositionInternal->size
    [0];
  iobj_13->HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_13->HomePositionInternal,
    cartesian_trajectory_planner_B.b_kstr_l);
  for (cartesian_trajectory_planner_B.b_kstr_l = 0;
       cartesian_trajectory_planner_B.b_kstr_l < 1;
       cartesian_trajectory_planner_B.b_kstr_l++) {
    iobj_13->HomePositionInternal->data[0] = 0.0;
  }

  iobj_6->JointInternal = iobj_13;
  iobj_6->Index = -1.0;
  iobj_6->ParentIndex = -1.0;
  obj->Bodies[7] = iobj_6;
  obj->NumBodies = 0.0;
  obj->NumNonFixedBodies = 0.0;
  obj->PositionNumber = 0.0;
  obj->VelocityNumber = 0.0;
  cartesian_trajectory_plann_rand(cartesian_trajectory_planner_B.unusedExpr_h);
  for (cartesian_trajectory_planner_B.b_kstr_l = 0;
       cartesian_trajectory_planner_B.b_kstr_l < 8;
       cartesian_trajectory_planner_B.b_kstr_l++) {
    obj->PositionDoFMap[cartesian_trajectory_planner_B.b_kstr_l] = 0.0;
  }

  for (cartesian_trajectory_planner_B.b_kstr_l = 0;
       cartesian_trajectory_planner_B.b_kstr_l < 8;
       cartesian_trajectory_planner_B.b_kstr_l++) {
    obj->PositionDoFMap[cartesian_trajectory_planner_B.b_kstr_l + 8] = -1.0;
  }

  for (cartesian_trajectory_planner_B.b_kstr_l = 0;
       cartesian_trajectory_planner_B.b_kstr_l < 8;
       cartesian_trajectory_planner_B.b_kstr_l++) {
    obj->VelocityDoFMap[cartesian_trajectory_planner_B.b_kstr_l] = 0.0;
  }

  for (cartesian_trajectory_planner_B.b_kstr_l = 0;
       cartesian_trajectory_planner_B.b_kstr_l < 8;
       cartesian_trajectory_planner_B.b_kstr_l++) {
    obj->VelocityDoFMap[cartesian_trajectory_planner_B.b_kstr_l + 8] = -1.0;
  }
}

static x_robotics_manip_internal_Rig_T *RigidBodyTree_RigidBodyTree_ast
  (x_robotics_manip_internal_Rig_T *obj, w_robotics_manip_internal_Rig_T *iobj_0,
   w_robotics_manip_internal_Rig_T *iobj_1, w_robotics_manip_internal_Rig_T
   *iobj_2, w_robotics_manip_internal_Rig_T *iobj_3,
   w_robotics_manip_internal_Rig_T *iobj_4, w_robotics_manip_internal_Rig_T
   *iobj_5, w_robotics_manip_internal_Rig_T *iobj_6,
   c_rigidBodyJoint_cartesian_as_T *iobj_7, c_rigidBodyJoint_cartesian_as_T
   *iobj_8, c_rigidBodyJoint_cartesian_as_T *iobj_9,
   c_rigidBodyJoint_cartesian_as_T *iobj_10, c_rigidBodyJoint_cartesian_as_T
   *iobj_11, c_rigidBodyJoint_cartesian_as_T *iobj_12,
   c_rigidBodyJoint_cartesian_as_T *iobj_13, c_rigidBodyJoint_cartesian_as_T
   *iobj_14, c_rigidBodyJoint_cartesian_as_T *iobj_15,
   w_robotics_manip_internal_Rig_T *iobj_16)
{
  x_robotics_manip_internal_Rig_T *b_obj;
  emxArray_char_T_cartesian_tra_T *switch_expression;
  static const int8_T tmp[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  static const char_T tmp_0[8] = { 'b', 'a', 's', 'e', '_', 'j', 'n', 't' };

  static const char_T tmp_1[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_2[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_3[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  cartesian_trajectory_planner_B.b_kstr_c = obj->Base.NameInternal->size[0] *
    obj->Base.NameInternal->size[1];
  obj->Base.NameInternal->size[0] = 1;
  obj->Base.NameInternal->size[1] = 4;
  cartes_emxEnsureCapacity_char_T(obj->Base.NameInternal,
    cartesian_trajectory_planner_B.b_kstr_c);
  obj->Base.NameInternal->data[0] = 'b';
  obj->Base.NameInternal->data[1] = 'a';
  obj->Base.NameInternal->data[2] = 's';
  obj->Base.NameInternal->data[3] = 'e';
  iobj_15->InTree = false;
  for (cartesian_trajectory_planner_B.b_kstr_c = 0;
       cartesian_trajectory_planner_B.b_kstr_c < 16;
       cartesian_trajectory_planner_B.b_kstr_c++) {
    iobj_15->JointToParentTransform[cartesian_trajectory_planner_B.b_kstr_c] =
      tmp[cartesian_trajectory_planner_B.b_kstr_c];
  }

  for (cartesian_trajectory_planner_B.b_kstr_c = 0;
       cartesian_trajectory_planner_B.b_kstr_c < 16;
       cartesian_trajectory_planner_B.b_kstr_c++) {
    iobj_15->ChildToJointTransform[cartesian_trajectory_planner_B.b_kstr_c] =
      tmp[cartesian_trajectory_planner_B.b_kstr_c];
  }

  cartesian_trajectory_planner_B.b_kstr_c = iobj_15->NameInternal->size[0] *
    iobj_15->NameInternal->size[1];
  iobj_15->NameInternal->size[0] = 1;
  iobj_15->NameInternal->size[1] = 8;
  cartes_emxEnsureCapacity_char_T(iobj_15->NameInternal,
    cartesian_trajectory_planner_B.b_kstr_c);
  for (cartesian_trajectory_planner_B.b_kstr_c = 0;
       cartesian_trajectory_planner_B.b_kstr_c < 8;
       cartesian_trajectory_planner_B.b_kstr_c++) {
    iobj_15->NameInternal->data[cartesian_trajectory_planner_B.b_kstr_c] =
      tmp_0[cartesian_trajectory_planner_B.b_kstr_c];
  }

  cartesian_trajectory_planner_B.b_kstr_c = iobj_15->Type->size[0] *
    iobj_15->Type->size[1];
  iobj_15->Type->size[0] = 1;
  iobj_15->Type->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(iobj_15->Type,
    cartesian_trajectory_planner_B.b_kstr_c);
  for (cartesian_trajectory_planner_B.b_kstr_c = 0;
       cartesian_trajectory_planner_B.b_kstr_c < 5;
       cartesian_trajectory_planner_B.b_kstr_c++) {
    iobj_15->Type->data[cartesian_trajectory_planner_B.b_kstr_c] =
      tmp_1[cartesian_trajectory_planner_B.b_kstr_c];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  cartesian_trajectory_planner_B.b_kstr_c = switch_expression->size[0] *
    switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_15->Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression,
    cartesian_trajectory_planner_B.b_kstr_c);
  cartesian_trajectory_planner_B.loop_ub_a = iobj_15->Type->size[0] *
    iobj_15->Type->size[1] - 1;
  for (cartesian_trajectory_planner_B.b_kstr_c = 0;
       cartesian_trajectory_planner_B.b_kstr_c <=
       cartesian_trajectory_planner_B.loop_ub_a;
       cartesian_trajectory_planner_B.b_kstr_c++) {
    switch_expression->data[cartesian_trajectory_planner_B.b_kstr_c] =
      iobj_15->Type->data[cartesian_trajectory_planner_B.b_kstr_c];
  }

  for (cartesian_trajectory_planner_B.b_kstr_c = 0;
       cartesian_trajectory_planner_B.b_kstr_c < 8;
       cartesian_trajectory_planner_B.b_kstr_c++) {
    cartesian_trajectory_planner_B.b_mx[cartesian_trajectory_planner_B.b_kstr_c]
      = tmp_2[cartesian_trajectory_planner_B.b_kstr_c];
  }

  cartesian_trajectory_planner_B.b_bool_i = false;
  if (switch_expression->size[1] == 8) {
    cartesian_trajectory_planner_B.b_kstr_c = 1;
    do {
      exitg1 = 0;
      if (cartesian_trajectory_planner_B.b_kstr_c - 1 < 8) {
        cartesian_trajectory_planner_B.loop_ub_a =
          cartesian_trajectory_planner_B.b_kstr_c - 1;
        if (switch_expression->data[cartesian_trajectory_planner_B.loop_ub_a] !=
            cartesian_trajectory_planner_B.b_mx[cartesian_trajectory_planner_B.loop_ub_a])
        {
          exitg1 = 1;
        } else {
          cartesian_trajectory_planner_B.b_kstr_c++;
        }
      } else {
        cartesian_trajectory_planner_B.b_bool_i = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (cartesian_trajectory_planner_B.b_bool_i) {
    cartesian_trajectory_planner_B.b_kstr_c = 0;
  } else {
    for (cartesian_trajectory_planner_B.b_kstr_c = 0;
         cartesian_trajectory_planner_B.b_kstr_c < 9;
         cartesian_trajectory_planner_B.b_kstr_c++) {
      cartesian_trajectory_planner_B.b_k[cartesian_trajectory_planner_B.b_kstr_c]
        = tmp_3[cartesian_trajectory_planner_B.b_kstr_c];
    }

    cartesian_trajectory_planner_B.b_bool_i = false;
    if (switch_expression->size[1] == 9) {
      cartesian_trajectory_planner_B.b_kstr_c = 1;
      do {
        exitg1 = 0;
        if (cartesian_trajectory_planner_B.b_kstr_c - 1 < 9) {
          cartesian_trajectory_planner_B.loop_ub_a =
            cartesian_trajectory_planner_B.b_kstr_c - 1;
          if (switch_expression->data[cartesian_trajectory_planner_B.loop_ub_a]
              !=
              cartesian_trajectory_planner_B.b_k[cartesian_trajectory_planner_B.loop_ub_a])
          {
            exitg1 = 1;
          } else {
            cartesian_trajectory_planner_B.b_kstr_c++;
          }
        } else {
          cartesian_trajectory_planner_B.b_bool_i = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (cartesian_trajectory_planner_B.b_bool_i) {
      cartesian_trajectory_planner_B.b_kstr_c = 1;
    } else {
      cartesian_trajectory_planner_B.b_kstr_c = -1;
    }
  }

  cartesian_trajec_emxFree_char_T(&switch_expression);
  switch (cartesian_trajectory_planner_B.b_kstr_c) {
   case 0:
    cartesian_trajectory_planner_B.iv[0] = 0;
    cartesian_trajectory_planner_B.iv[1] = 0;
    cartesian_trajectory_planner_B.iv[2] = 1;
    cartesian_trajectory_planner_B.iv[3] = 0;
    cartesian_trajectory_planner_B.iv[4] = 0;
    cartesian_trajectory_planner_B.iv[5] = 0;
    for (cartesian_trajectory_planner_B.b_kstr_c = 0;
         cartesian_trajectory_planner_B.b_kstr_c < 6;
         cartesian_trajectory_planner_B.b_kstr_c++) {
      cartesian_trajectory_planner_B.msubspace_data[cartesian_trajectory_planner_B.b_kstr_c]
        =
        cartesian_trajectory_planner_B.iv[cartesian_trajectory_planner_B.b_kstr_c];
    }

    cartesian_trajectory_planner_B.poslim_data[0] = -3.1415926535897931;
    cartesian_trajectory_planner_B.poslim_data[1] = 3.1415926535897931;
    iobj_15->VelocityNumber = 1.0;
    iobj_15->PositionNumber = 1.0;
    iobj_15->JointAxisInternal[0] = 0.0;
    iobj_15->JointAxisInternal[1] = 0.0;
    iobj_15->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    cartesian_trajectory_planner_B.iv[0] = 0;
    cartesian_trajectory_planner_B.iv[1] = 0;
    cartesian_trajectory_planner_B.iv[2] = 0;
    cartesian_trajectory_planner_B.iv[3] = 0;
    cartesian_trajectory_planner_B.iv[4] = 0;
    cartesian_trajectory_planner_B.iv[5] = 1;
    for (cartesian_trajectory_planner_B.b_kstr_c = 0;
         cartesian_trajectory_planner_B.b_kstr_c < 6;
         cartesian_trajectory_planner_B.b_kstr_c++) {
      cartesian_trajectory_planner_B.msubspace_data[cartesian_trajectory_planner_B.b_kstr_c]
        =
        cartesian_trajectory_planner_B.iv[cartesian_trajectory_planner_B.b_kstr_c];
    }

    cartesian_trajectory_planner_B.poslim_data[0] = -0.5;
    cartesian_trajectory_planner_B.poslim_data[1] = 0.5;
    iobj_15->VelocityNumber = 1.0;
    iobj_15->PositionNumber = 1.0;
    iobj_15->JointAxisInternal[0] = 0.0;
    iobj_15->JointAxisInternal[1] = 0.0;
    iobj_15->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (cartesian_trajectory_planner_B.b_kstr_c = 0;
         cartesian_trajectory_planner_B.b_kstr_c < 6;
         cartesian_trajectory_planner_B.b_kstr_c++) {
      cartesian_trajectory_planner_B.msubspace_data[cartesian_trajectory_planner_B.b_kstr_c]
        = 0;
    }

    cartesian_trajectory_planner_B.poslim_data[0] = 0.0;
    cartesian_trajectory_planner_B.poslim_data[1] = 0.0;
    iobj_15->VelocityNumber = 0.0;
    iobj_15->PositionNumber = 0.0;
    iobj_15->JointAxisInternal[0] = 0.0;
    iobj_15->JointAxisInternal[1] = 0.0;
    iobj_15->JointAxisInternal[2] = 0.0;
    break;
  }

  cartesian_trajectory_planner_B.b_kstr_c = iobj_15->MotionSubspace->size[0] *
    iobj_15->MotionSubspace->size[1];
  iobj_15->MotionSubspace->size[0] = 6;
  iobj_15->MotionSubspace->size[1] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_15->MotionSubspace,
    cartesian_trajectory_planner_B.b_kstr_c);
  for (cartesian_trajectory_planner_B.b_kstr_c = 0;
       cartesian_trajectory_planner_B.b_kstr_c < 6;
       cartesian_trajectory_planner_B.b_kstr_c++) {
    iobj_15->MotionSubspace->data[cartesian_trajectory_planner_B.b_kstr_c] =
      cartesian_trajectory_planner_B.msubspace_data[cartesian_trajectory_planner_B.b_kstr_c];
  }

  cartesian_trajectory_planner_B.b_kstr_c = iobj_15->
    PositionLimitsInternal->size[0] * iobj_15->PositionLimitsInternal->size[1];
  iobj_15->PositionLimitsInternal->size[0] = 1;
  iobj_15->PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(iobj_15->PositionLimitsInternal,
    cartesian_trajectory_planner_B.b_kstr_c);
  for (cartesian_trajectory_planner_B.b_kstr_c = 0;
       cartesian_trajectory_planner_B.b_kstr_c < 2;
       cartesian_trajectory_planner_B.b_kstr_c++) {
    iobj_15->PositionLimitsInternal->
      data[cartesian_trajectory_planner_B.b_kstr_c] =
      cartesian_trajectory_planner_B.poslim_data[cartesian_trajectory_planner_B.b_kstr_c];
  }

  cartesian_trajectory_planner_B.b_kstr_c = iobj_15->HomePositionInternal->size
    [0];
  iobj_15->HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_15->HomePositionInternal,
    cartesian_trajectory_planner_B.b_kstr_c);
  for (cartesian_trajectory_planner_B.b_kstr_c = 0;
       cartesian_trajectory_planner_B.b_kstr_c < 1;
       cartesian_trajectory_planner_B.b_kstr_c++) {
    iobj_15->HomePositionInternal->data[0] = 0.0;
  }

  obj->Base.JointInternal = iobj_15;
  obj->Base.Index = -1.0;
  obj->Base.ParentIndex = -1.0;
  obj->Base.Index = 0.0;
  cartesian_trajectory_plann_rand(cartesian_trajectory_planner_B.unusedExpr);
  ca_RigidBodyTree_clearAllBodies(obj, iobj_0, iobj_1, iobj_2, iobj_3, iobj_4,
    iobj_5, iobj_6, iobj_8, iobj_9, iobj_10, iobj_11, iobj_12, iobj_13, iobj_14,
    iobj_7, iobj_16);
  return b_obj;
}

static c_rigidBodyJoint_cartesian_as_T *c_rigidBodyJoint_rigidBodyJoint
  (c_rigidBodyJoint_cartesian_as_T *obj, const emxArray_char_T_cartesian_tra_T
   *jname, const emxArray_char_T_cartesian_tra_T *jtype)
{
  c_rigidBodyJoint_cartesian_as_T *b_obj;
  emxArray_char_T_cartesian_tra_T *switch_expression;
  static const int8_T tmp[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_2[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_3[128] = { '\x00', '\x01', '\x02', '\x03', '\x04',
    '\x05', '\x06', '\x07', '\x08', '\x09', '\x0a', '\x0b', '\x0c', '\x0d',
    '\x0e', '\x0f', '\x10', '\x11', '\x12', '\x13', '\x14', '\x15', '\x16',
    '\x17', '\x18', '\x19', '\x1a', '\x1b', '\x1c', '\x1d', '\x1e', '\x1f', ' ',
    '!', '\"', '#', '$', '%', '&', '\'', '(', ')', '*', '+', ',', '-', '.', '/',
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', ':', ';', '<', '=', '>',
    '?', '@', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm',
    'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '[', '\\',
    ']', '^', '_', '`', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k',
    'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z',
    '{', '|', '}', '~', '\x7f' };

  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  boolean_T guard4 = false;
  int32_T exitg1;
  boolean_T guard11 = false;
  obj->InTree = false;
  for (cartesian_trajectory_planner_B.minnanb = 0;
       cartesian_trajectory_planner_B.minnanb < 16;
       cartesian_trajectory_planner_B.minnanb++) {
    obj->JointToParentTransform[cartesian_trajectory_planner_B.minnanb] =
      tmp[cartesian_trajectory_planner_B.minnanb];
  }

  for (cartesian_trajectory_planner_B.minnanb = 0;
       cartesian_trajectory_planner_B.minnanb < 16;
       cartesian_trajectory_planner_B.minnanb++) {
    obj->ChildToJointTransform[cartesian_trajectory_planner_B.minnanb] =
      tmp[cartesian_trajectory_planner_B.minnanb];
  }

  b_obj = obj;
  cartesian_trajectory_planner_B.minnanb = obj->NameInternal->size[0] *
    obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = jname->size[1];
  cartes_emxEnsureCapacity_char_T(obj->NameInternal,
    cartesian_trajectory_planner_B.minnanb);
  cartesian_trajectory_planner_B.loop_ub_c = jname->size[0] * jname->size[1] - 1;
  for (cartesian_trajectory_planner_B.minnanb = 0;
       cartesian_trajectory_planner_B.minnanb <=
       cartesian_trajectory_planner_B.loop_ub_c;
       cartesian_trajectory_planner_B.minnanb++) {
    obj->NameInternal->data[cartesian_trajectory_planner_B.minnanb] =
      jname->data[cartesian_trajectory_planner_B.minnanb];
  }

  cartesian_trajectory_planner_B.partial_match_size_idx_1 = 0;
  for (cartesian_trajectory_planner_B.minnanb = 0;
       cartesian_trajectory_planner_B.minnanb < 8;
       cartesian_trajectory_planner_B.minnanb++) {
    cartesian_trajectory_planner_B.vstr[cartesian_trajectory_planner_B.minnanb] =
      tmp_0[cartesian_trajectory_planner_B.minnanb];
  }

  guard1 = false;
  guard2 = false;
  guard3 = false;
  guard4 = false;
  if (jtype->size[1] <= 8) {
    cartesian_trajectory_planner_B.loop_ub_c = jtype->size[1];
    for (cartesian_trajectory_planner_B.minnanb = 0;
         cartesian_trajectory_planner_B.minnanb < 8;
         cartesian_trajectory_planner_B.minnanb++) {
      cartesian_trajectory_planner_B.b_g4k[cartesian_trajectory_planner_B.minnanb]
        = tmp_0[cartesian_trajectory_planner_B.minnanb];
    }

    cartesian_trajectory_planner_B.b_bool_ok = false;
    cartesian_trajectory_planner_B.minnanb = jtype->size[1];
    if (cartesian_trajectory_planner_B.minnanb >= 8) {
      cartesian_trajectory_planner_B.minnanb = 8;
    }

    guard11 = false;
    if (cartesian_trajectory_planner_B.loop_ub_c <=
        cartesian_trajectory_planner_B.minnanb) {
      if (cartesian_trajectory_planner_B.minnanb <
          cartesian_trajectory_planner_B.loop_ub_c) {
        cartesian_trajectory_planner_B.loop_ub_c =
          cartesian_trajectory_planner_B.minnanb;
      }

      cartesian_trajectory_planner_B.minnanb =
        cartesian_trajectory_planner_B.loop_ub_c - 1;
      guard11 = true;
    } else {
      if (jtype->size[1] == 8) {
        cartesian_trajectory_planner_B.minnanb = 7;
        guard11 = true;
      }
    }

    if (guard11) {
      cartesian_trajectory_planner_B.loop_ub_c = 1;
      do {
        exitg1 = 0;
        if (cartesian_trajectory_planner_B.loop_ub_c - 1 <=
            cartesian_trajectory_planner_B.minnanb) {
          cartesian_trajectory_planner_B.kstr_n =
            cartesian_trajectory_planner_B.loop_ub_c - 1;
          if (tmp_3[static_cast<uint8_T>(jtype->
               data[cartesian_trajectory_planner_B.kstr_n]) & 127] != tmp_3[
              static_cast<int32_T>
              (cartesian_trajectory_planner_B.b_g4k[cartesian_trajectory_planner_B.kstr_n])])
          {
            exitg1 = 1;
          } else {
            cartesian_trajectory_planner_B.loop_ub_c++;
          }
        } else {
          cartesian_trajectory_planner_B.b_bool_ok = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (cartesian_trajectory_planner_B.b_bool_ok) {
      if (jtype->size[1] == 8) {
        cartesian_trajectory_planner_B.nmatched = 1;
        cartesian_trajectory_planner_B.partial_match_size_idx_1 = 8;
        for (cartesian_trajectory_planner_B.minnanb = 0;
             cartesian_trajectory_planner_B.minnanb < 8;
             cartesian_trajectory_planner_B.minnanb++) {
          cartesian_trajectory_planner_B.b_cj[cartesian_trajectory_planner_B.minnanb]
            =
            cartesian_trajectory_planner_B.vstr[cartesian_trajectory_planner_B.minnanb];
        }
      } else {
        cartesian_trajectory_planner_B.partial_match_size_idx_1 = 8;
        for (cartesian_trajectory_planner_B.minnanb = 0;
             cartesian_trajectory_planner_B.minnanb < 8;
             cartesian_trajectory_planner_B.minnanb++) {
          cartesian_trajectory_planner_B.partial_match_data[cartesian_trajectory_planner_B.minnanb]
            =
            cartesian_trajectory_planner_B.vstr[cartesian_trajectory_planner_B.minnanb];
        }

        cartesian_trajectory_planner_B.matched = true;
        cartesian_trajectory_planner_B.nmatched = 1;
        guard3 = true;
      }
    } else {
      guard4 = true;
    }
  } else {
    guard4 = true;
  }

  if (guard4) {
    cartesian_trajectory_planner_B.matched = false;
    cartesian_trajectory_planner_B.nmatched = 0;
    guard3 = true;
  }

  if (guard3) {
    for (cartesian_trajectory_planner_B.minnanb = 0;
         cartesian_trajectory_planner_B.minnanb < 9;
         cartesian_trajectory_planner_B.minnanb++) {
      cartesian_trajectory_planner_B.b_vstr[cartesian_trajectory_planner_B.minnanb]
        = tmp_1[cartesian_trajectory_planner_B.minnanb];
    }

    if (jtype->size[1] <= 9) {
      cartesian_trajectory_planner_B.loop_ub_c = jtype->size[1];
      for (cartesian_trajectory_planner_B.minnanb = 0;
           cartesian_trajectory_planner_B.minnanb < 9;
           cartesian_trajectory_planner_B.minnanb++) {
        cartesian_trajectory_planner_B.b_cj[cartesian_trajectory_planner_B.minnanb]
          = tmp_1[cartesian_trajectory_planner_B.minnanb];
      }

      cartesian_trajectory_planner_B.b_bool_ok = false;
      cartesian_trajectory_planner_B.minnanb = jtype->size[1];
      if (cartesian_trajectory_planner_B.minnanb >= 9) {
        cartesian_trajectory_planner_B.minnanb = 9;
      }

      guard11 = false;
      if (cartesian_trajectory_planner_B.loop_ub_c <=
          cartesian_trajectory_planner_B.minnanb) {
        if (cartesian_trajectory_planner_B.minnanb <
            cartesian_trajectory_planner_B.loop_ub_c) {
          cartesian_trajectory_planner_B.loop_ub_c =
            cartesian_trajectory_planner_B.minnanb;
        }

        cartesian_trajectory_planner_B.minnanb =
          cartesian_trajectory_planner_B.loop_ub_c - 1;
        guard11 = true;
      } else {
        if (jtype->size[1] == 9) {
          cartesian_trajectory_planner_B.minnanb = 8;
          guard11 = true;
        }
      }

      if (guard11) {
        cartesian_trajectory_planner_B.loop_ub_c = 1;
        do {
          exitg1 = 0;
          if (cartesian_trajectory_planner_B.loop_ub_c - 1 <=
              cartesian_trajectory_planner_B.minnanb) {
            cartesian_trajectory_planner_B.kstr_n =
              cartesian_trajectory_planner_B.loop_ub_c - 1;
            if (tmp_3[static_cast<uint8_T>(jtype->
                 data[cartesian_trajectory_planner_B.kstr_n]) & 127] != tmp_3[
                static_cast<int32_T>
                (cartesian_trajectory_planner_B.b_cj[cartesian_trajectory_planner_B.kstr_n])])
            {
              exitg1 = 1;
            } else {
              cartesian_trajectory_planner_B.loop_ub_c++;
            }
          } else {
            cartesian_trajectory_planner_B.b_bool_ok = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (cartesian_trajectory_planner_B.b_bool_ok) {
        if (jtype->size[1] == 9) {
          cartesian_trajectory_planner_B.nmatched = 1;
          cartesian_trajectory_planner_B.partial_match_size_idx_1 = 9;
          for (cartesian_trajectory_planner_B.minnanb = 0;
               cartesian_trajectory_planner_B.minnanb < 9;
               cartesian_trajectory_planner_B.minnanb++) {
            cartesian_trajectory_planner_B.b_cj[cartesian_trajectory_planner_B.minnanb]
              =
              cartesian_trajectory_planner_B.b_vstr[cartesian_trajectory_planner_B.minnanb];
          }
        } else {
          if (!cartesian_trajectory_planner_B.matched) {
            cartesian_trajectory_planner_B.partial_match_size_idx_1 = 9;
            for (cartesian_trajectory_planner_B.minnanb = 0;
                 cartesian_trajectory_planner_B.minnanb < 9;
                 cartesian_trajectory_planner_B.minnanb++) {
              cartesian_trajectory_planner_B.partial_match_data[cartesian_trajectory_planner_B.minnanb]
                =
                cartesian_trajectory_planner_B.b_vstr[cartesian_trajectory_planner_B.minnanb];
            }
          }

          cartesian_trajectory_planner_B.matched = true;
          cartesian_trajectory_planner_B.nmatched++;
          guard2 = true;
        }
      } else {
        guard2 = true;
      }
    } else {
      guard2 = true;
    }
  }

  if (guard2) {
    for (cartesian_trajectory_planner_B.minnanb = 0;
         cartesian_trajectory_planner_B.minnanb < 5;
         cartesian_trajectory_planner_B.minnanb++) {
      cartesian_trajectory_planner_B.c_vstr[cartesian_trajectory_planner_B.minnanb]
        = tmp_2[cartesian_trajectory_planner_B.minnanb];
    }

    if (jtype->size[1] <= 5) {
      cartesian_trajectory_planner_B.loop_ub_c = jtype->size[1];
      for (cartesian_trajectory_planner_B.minnanb = 0;
           cartesian_trajectory_planner_B.minnanb < 5;
           cartesian_trajectory_planner_B.minnanb++) {
        cartesian_trajectory_planner_B.b_hp[cartesian_trajectory_planner_B.minnanb]
          = tmp_2[cartesian_trajectory_planner_B.minnanb];
      }

      cartesian_trajectory_planner_B.b_bool_ok = false;
      cartesian_trajectory_planner_B.minnanb = jtype->size[1];
      if (cartesian_trajectory_planner_B.minnanb >= 5) {
        cartesian_trajectory_planner_B.minnanb = 5;
      }

      guard11 = false;
      if (cartesian_trajectory_planner_B.loop_ub_c <=
          cartesian_trajectory_planner_B.minnanb) {
        if (cartesian_trajectory_planner_B.minnanb <
            cartesian_trajectory_planner_B.loop_ub_c) {
          cartesian_trajectory_planner_B.loop_ub_c =
            cartesian_trajectory_planner_B.minnanb;
        }

        cartesian_trajectory_planner_B.minnanb =
          cartesian_trajectory_planner_B.loop_ub_c - 1;
        guard11 = true;
      } else {
        if (jtype->size[1] == 5) {
          cartesian_trajectory_planner_B.minnanb = 4;
          guard11 = true;
        }
      }

      if (guard11) {
        cartesian_trajectory_planner_B.loop_ub_c = 1;
        do {
          exitg1 = 0;
          if (cartesian_trajectory_planner_B.loop_ub_c - 1 <=
              cartesian_trajectory_planner_B.minnanb) {
            cartesian_trajectory_planner_B.kstr_n =
              cartesian_trajectory_planner_B.loop_ub_c - 1;
            if (tmp_3[static_cast<uint8_T>(jtype->
                 data[cartesian_trajectory_planner_B.kstr_n]) & 127] != tmp_3[
                static_cast<int32_T>
                (cartesian_trajectory_planner_B.b_hp[cartesian_trajectory_planner_B.kstr_n])])
            {
              exitg1 = 1;
            } else {
              cartesian_trajectory_planner_B.loop_ub_c++;
            }
          } else {
            cartesian_trajectory_planner_B.b_bool_ok = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (cartesian_trajectory_planner_B.b_bool_ok) {
        if (jtype->size[1] == 5) {
          cartesian_trajectory_planner_B.nmatched = 1;
          cartesian_trajectory_planner_B.partial_match_size_idx_1 = 5;
          for (cartesian_trajectory_planner_B.minnanb = 0;
               cartesian_trajectory_planner_B.minnanb < 5;
               cartesian_trajectory_planner_B.minnanb++) {
            cartesian_trajectory_planner_B.b_cj[cartesian_trajectory_planner_B.minnanb]
              =
              cartesian_trajectory_planner_B.c_vstr[cartesian_trajectory_planner_B.minnanb];
          }
        } else {
          if (!cartesian_trajectory_planner_B.matched) {
            cartesian_trajectory_planner_B.partial_match_size_idx_1 = 5;
            for (cartesian_trajectory_planner_B.minnanb = 0;
                 cartesian_trajectory_planner_B.minnanb < 5;
                 cartesian_trajectory_planner_B.minnanb++) {
              cartesian_trajectory_planner_B.partial_match_data[cartesian_trajectory_planner_B.minnanb]
                =
                cartesian_trajectory_planner_B.c_vstr[cartesian_trajectory_planner_B.minnanb];
            }
          }

          cartesian_trajectory_planner_B.nmatched++;
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
  }

  if (guard1) {
    if (cartesian_trajectory_planner_B.nmatched == 0) {
      cartesian_trajectory_planner_B.partial_match_size_idx_1 = 0;
    } else {
      cartesian_trajectory_planner_B.loop_ub_c =
        cartesian_trajectory_planner_B.partial_match_size_idx_1 - 1;
      if (0 <= cartesian_trajectory_planner_B.loop_ub_c) {
        memcpy(&cartesian_trajectory_planner_B.b_cj[0],
               &cartesian_trajectory_planner_B.partial_match_data[0],
               (cartesian_trajectory_planner_B.loop_ub_c + 1) * sizeof(char_T));
      }
    }
  }

  if ((cartesian_trajectory_planner_B.nmatched == 0) || ((jtype->size[1] == 0)
       != (cartesian_trajectory_planner_B.partial_match_size_idx_1 == 0))) {
    cartesian_trajectory_planner_B.partial_match_size_idx_1 = 0;
  } else {
    cartesian_trajectory_planner_B.loop_ub_c =
      cartesian_trajectory_planner_B.partial_match_size_idx_1 - 1;
    if (0 <= cartesian_trajectory_planner_B.loop_ub_c) {
      memcpy(&cartesian_trajectory_planner_B.partial_match_data[0],
             &cartesian_trajectory_planner_B.b_cj[0],
             (cartesian_trajectory_planner_B.loop_ub_c + 1) * sizeof(char_T));
    }
  }

  cartesian_trajectory_planner_B.minnanb = obj->Type->size[0] * obj->Type->size
    [1];
  obj->Type->size[0] = 1;
  obj->Type->size[1] = cartesian_trajectory_planner_B.partial_match_size_idx_1;
  cartes_emxEnsureCapacity_char_T(obj->Type,
    cartesian_trajectory_planner_B.minnanb);
  cartesian_trajectory_planner_B.loop_ub_c =
    cartesian_trajectory_planner_B.partial_match_size_idx_1 - 1;
  for (cartesian_trajectory_planner_B.minnanb = 0;
       cartesian_trajectory_planner_B.minnanb <=
       cartesian_trajectory_planner_B.loop_ub_c;
       cartesian_trajectory_planner_B.minnanb++) {
    obj->Type->data[cartesian_trajectory_planner_B.minnanb] =
      cartesian_trajectory_planner_B.partial_match_data[cartesian_trajectory_planner_B.minnanb];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  cartesian_trajectory_planner_B.minnanb = switch_expression->size[0] *
    switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression,
    cartesian_trajectory_planner_B.minnanb);
  cartesian_trajectory_planner_B.loop_ub_c = obj->Type->size[0] * obj->
    Type->size[1] - 1;
  for (cartesian_trajectory_planner_B.minnanb = 0;
       cartesian_trajectory_planner_B.minnanb <=
       cartesian_trajectory_planner_B.loop_ub_c;
       cartesian_trajectory_planner_B.minnanb++) {
    switch_expression->data[cartesian_trajectory_planner_B.minnanb] = obj->
      Type->data[cartesian_trajectory_planner_B.minnanb];
  }

  for (cartesian_trajectory_planner_B.minnanb = 0;
       cartesian_trajectory_planner_B.minnanb < 8;
       cartesian_trajectory_planner_B.minnanb++) {
    cartesian_trajectory_planner_B.b_g4k[cartesian_trajectory_planner_B.minnanb]
      = tmp_0[cartesian_trajectory_planner_B.minnanb];
  }

  cartesian_trajectory_planner_B.b_bool_ok = false;
  if (switch_expression->size[1] == 8) {
    cartesian_trajectory_planner_B.loop_ub_c = 1;
    do {
      exitg1 = 0;
      if (cartesian_trajectory_planner_B.loop_ub_c - 1 < 8) {
        cartesian_trajectory_planner_B.kstr_n =
          cartesian_trajectory_planner_B.loop_ub_c - 1;
        if (switch_expression->data[cartesian_trajectory_planner_B.kstr_n] !=
            cartesian_trajectory_planner_B.b_g4k[cartesian_trajectory_planner_B.kstr_n])
        {
          exitg1 = 1;
        } else {
          cartesian_trajectory_planner_B.loop_ub_c++;
        }
      } else {
        cartesian_trajectory_planner_B.b_bool_ok = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (cartesian_trajectory_planner_B.b_bool_ok) {
    cartesian_trajectory_planner_B.minnanb = 0;
  } else {
    for (cartesian_trajectory_planner_B.minnanb = 0;
         cartesian_trajectory_planner_B.minnanb < 9;
         cartesian_trajectory_planner_B.minnanb++) {
      cartesian_trajectory_planner_B.b_cj[cartesian_trajectory_planner_B.minnanb]
        = tmp_1[cartesian_trajectory_planner_B.minnanb];
    }

    cartesian_trajectory_planner_B.b_bool_ok = false;
    if (switch_expression->size[1] == 9) {
      cartesian_trajectory_planner_B.loop_ub_c = 1;
      do {
        exitg1 = 0;
        if (cartesian_trajectory_planner_B.loop_ub_c - 1 < 9) {
          cartesian_trajectory_planner_B.kstr_n =
            cartesian_trajectory_planner_B.loop_ub_c - 1;
          if (switch_expression->data[cartesian_trajectory_planner_B.kstr_n] !=
              cartesian_trajectory_planner_B.b_cj[cartesian_trajectory_planner_B.kstr_n])
          {
            exitg1 = 1;
          } else {
            cartesian_trajectory_planner_B.loop_ub_c++;
          }
        } else {
          cartesian_trajectory_planner_B.b_bool_ok = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (cartesian_trajectory_planner_B.b_bool_ok) {
      cartesian_trajectory_planner_B.minnanb = 1;
    } else {
      cartesian_trajectory_planner_B.minnanb = -1;
    }
  }

  cartesian_trajec_emxFree_char_T(&switch_expression);
  switch (cartesian_trajectory_planner_B.minnanb) {
   case 0:
    cartesian_trajectory_planner_B.iv3[0] = 0;
    cartesian_trajectory_planner_B.iv3[1] = 0;
    cartesian_trajectory_planner_B.iv3[2] = 1;
    cartesian_trajectory_planner_B.iv3[3] = 0;
    cartesian_trajectory_planner_B.iv3[4] = 0;
    cartesian_trajectory_planner_B.iv3[5] = 0;
    for (cartesian_trajectory_planner_B.minnanb = 0;
         cartesian_trajectory_planner_B.minnanb < 6;
         cartesian_trajectory_planner_B.minnanb++) {
      cartesian_trajectory_planner_B.msubspace_data_k[cartesian_trajectory_planner_B.minnanb]
        =
        cartesian_trajectory_planner_B.iv3[cartesian_trajectory_planner_B.minnanb];
    }

    cartesian_trajectory_planner_B.poslim_data_f[0] = -3.1415926535897931;
    cartesian_trajectory_planner_B.poslim_data_f[1] = 3.1415926535897931;
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    cartesian_trajectory_planner_B.iv3[0] = 0;
    cartesian_trajectory_planner_B.iv3[1] = 0;
    cartesian_trajectory_planner_B.iv3[2] = 0;
    cartesian_trajectory_planner_B.iv3[3] = 0;
    cartesian_trajectory_planner_B.iv3[4] = 0;
    cartesian_trajectory_planner_B.iv3[5] = 1;
    for (cartesian_trajectory_planner_B.minnanb = 0;
         cartesian_trajectory_planner_B.minnanb < 6;
         cartesian_trajectory_planner_B.minnanb++) {
      cartesian_trajectory_planner_B.msubspace_data_k[cartesian_trajectory_planner_B.minnanb]
        =
        cartesian_trajectory_planner_B.iv3[cartesian_trajectory_planner_B.minnanb];
    }

    cartesian_trajectory_planner_B.poslim_data_f[0] = -0.5;
    cartesian_trajectory_planner_B.poslim_data_f[1] = 0.5;
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (cartesian_trajectory_planner_B.minnanb = 0;
         cartesian_trajectory_planner_B.minnanb < 6;
         cartesian_trajectory_planner_B.minnanb++) {
      cartesian_trajectory_planner_B.msubspace_data_k[cartesian_trajectory_planner_B.minnanb]
        = 0;
    }

    cartesian_trajectory_planner_B.poslim_data_f[0] = 0.0;
    cartesian_trajectory_planner_B.poslim_data_f[1] = 0.0;
    obj->VelocityNumber = 0.0;
    obj->PositionNumber = 0.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 0.0;
    break;
  }

  cartesian_trajectory_planner_B.minnanb = obj->MotionSubspace->size[0] *
    obj->MotionSubspace->size[1];
  obj->MotionSubspace->size[0] = 6;
  obj->MotionSubspace->size[1] = 1;
  cartes_emxEnsureCapacity_real_T(obj->MotionSubspace,
    cartesian_trajectory_planner_B.minnanb);
  for (cartesian_trajectory_planner_B.minnanb = 0;
       cartesian_trajectory_planner_B.minnanb < 6;
       cartesian_trajectory_planner_B.minnanb++) {
    obj->MotionSubspace->data[cartesian_trajectory_planner_B.minnanb] =
      cartesian_trajectory_planner_B.msubspace_data_k[cartesian_trajectory_planner_B.minnanb];
  }

  cartesian_trajectory_planner_B.minnanb = obj->PositionLimitsInternal->size[0] *
    obj->PositionLimitsInternal->size[1];
  obj->PositionLimitsInternal->size[0] = 1;
  obj->PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(obj->PositionLimitsInternal,
    cartesian_trajectory_planner_B.minnanb);
  for (cartesian_trajectory_planner_B.minnanb = 0;
       cartesian_trajectory_planner_B.minnanb < 2;
       cartesian_trajectory_planner_B.minnanb++) {
    obj->PositionLimitsInternal->data[cartesian_trajectory_planner_B.minnanb] =
      cartesian_trajectory_planner_B.poslim_data_f[cartesian_trajectory_planner_B.minnanb];
  }

  cartesian_trajectory_planner_B.minnanb = obj->HomePositionInternal->size[0];
  obj->HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(obj->HomePositionInternal,
    cartesian_trajectory_planner_B.minnanb);
  for (cartesian_trajectory_planner_B.minnanb = 0;
       cartesian_trajectory_planner_B.minnanb < 1;
       cartesian_trajectory_planner_B.minnanb++) {
    obj->HomePositionInternal->data[0] = 0.0;
  }

  return b_obj;
}

static w_robotics_manip_internal_Rig_T *cartesian_trajec_RigidBody_copy(const
  v_robotics_manip_internal_Rig_T *obj, c_rigidBodyJoint_cartesian_as_T *iobj_0,
  c_rigidBodyJoint_cartesian_as_T *iobj_1, w_robotics_manip_internal_Rig_T
  *iobj_2)
{
  w_robotics_manip_internal_Rig_T *newbody;
  c_rigidBodyJoint_cartesian_as_T *newjoint;
  emxArray_char_T_cartesian_tra_T *jtype;
  emxArray_char_T_cartesian_tra_T *jname;
  emxArray_real_T_cartesian_tra_T *obj_0;
  emxArray_real_T_cartesian_tra_T *obj_1;
  emxArray_real_T_cartesian_tra_T *obj_2;
  static const int8_T tmp[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  cartesian_trajec_emxInit_char_T(&jtype, 2);
  cartesian_trajectory_planner_B.b_kstr_mu = jtype->size[0] * jtype->size[1];
  jtype->size[0] = 1;
  jtype->size[1] = obj->NameInternal->size[1];
  cartes_emxEnsureCapacity_char_T(jtype,
    cartesian_trajectory_planner_B.b_kstr_mu);
  cartesian_trajectory_planner_B.loop_ub_gu = obj->NameInternal->size[0] *
    obj->NameInternal->size[1] - 1;
  for (cartesian_trajectory_planner_B.b_kstr_mu = 0;
       cartesian_trajectory_planner_B.b_kstr_mu <=
       cartesian_trajectory_planner_B.loop_ub_gu;
       cartesian_trajectory_planner_B.b_kstr_mu++) {
    jtype->data[cartesian_trajectory_planner_B.b_kstr_mu] = obj->
      NameInternal->data[cartesian_trajectory_planner_B.b_kstr_mu];
  }

  newbody = iobj_2;
  cartesian_trajectory_planner_B.b_kstr_mu = iobj_2->NameInternal->size[0] *
    iobj_2->NameInternal->size[1];
  iobj_2->NameInternal->size[0] = 1;
  iobj_2->NameInternal->size[1] = jtype->size[1];
  cartes_emxEnsureCapacity_char_T(iobj_2->NameInternal,
    cartesian_trajectory_planner_B.b_kstr_mu);
  cartesian_trajectory_planner_B.loop_ub_gu = jtype->size[0] * jtype->size[1] -
    1;
  for (cartesian_trajectory_planner_B.b_kstr_mu = 0;
       cartesian_trajectory_planner_B.b_kstr_mu <=
       cartesian_trajectory_planner_B.loop_ub_gu;
       cartesian_trajectory_planner_B.b_kstr_mu++) {
    iobj_2->NameInternal->data[cartesian_trajectory_planner_B.b_kstr_mu] =
      jtype->data[cartesian_trajectory_planner_B.b_kstr_mu];
  }

  iobj_0->InTree = false;
  for (cartesian_trajectory_planner_B.b_kstr_mu = 0;
       cartesian_trajectory_planner_B.b_kstr_mu < 16;
       cartesian_trajectory_planner_B.b_kstr_mu++) {
    iobj_0->JointToParentTransform[cartesian_trajectory_planner_B.b_kstr_mu] =
      tmp[cartesian_trajectory_planner_B.b_kstr_mu];
  }

  for (cartesian_trajectory_planner_B.b_kstr_mu = 0;
       cartesian_trajectory_planner_B.b_kstr_mu < 16;
       cartesian_trajectory_planner_B.b_kstr_mu++) {
    iobj_0->ChildToJointTransform[cartesian_trajectory_planner_B.b_kstr_mu] =
      tmp[cartesian_trajectory_planner_B.b_kstr_mu];
  }

  cartesian_trajectory_planner_B.b_kstr_mu = iobj_0->NameInternal->size[0] *
    iobj_0->NameInternal->size[1];
  iobj_0->NameInternal->size[0] = 1;
  iobj_0->NameInternal->size[1] = jtype->size[1] + 4;
  cartes_emxEnsureCapacity_char_T(iobj_0->NameInternal,
    cartesian_trajectory_planner_B.b_kstr_mu);
  cartesian_trajectory_planner_B.loop_ub_gu = jtype->size[0] * jtype->size[1];
  for (cartesian_trajectory_planner_B.b_kstr_mu = 0;
       cartesian_trajectory_planner_B.b_kstr_mu <
       cartesian_trajectory_planner_B.loop_ub_gu;
       cartesian_trajectory_planner_B.b_kstr_mu++) {
    iobj_0->NameInternal->data[cartesian_trajectory_planner_B.b_kstr_mu] =
      jtype->data[cartesian_trajectory_planner_B.b_kstr_mu];
  }

  iobj_0->NameInternal->data[cartesian_trajectory_planner_B.loop_ub_gu] = '_';
  iobj_0->NameInternal->data[cartesian_trajectory_planner_B.loop_ub_gu + 1] =
    'j';
  iobj_0->NameInternal->data[cartesian_trajectory_planner_B.loop_ub_gu + 2] =
    'n';
  iobj_0->NameInternal->data[cartesian_trajectory_planner_B.loop_ub_gu + 3] =
    't';
  cartesian_trajectory_planner_B.b_kstr_mu = iobj_0->Type->size[0] *
    iobj_0->Type->size[1];
  iobj_0->Type->size[0] = 1;
  iobj_0->Type->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(iobj_0->Type,
    cartesian_trajectory_planner_B.b_kstr_mu);
  for (cartesian_trajectory_planner_B.b_kstr_mu = 0;
       cartesian_trajectory_planner_B.b_kstr_mu < 5;
       cartesian_trajectory_planner_B.b_kstr_mu++) {
    iobj_0->Type->data[cartesian_trajectory_planner_B.b_kstr_mu] =
      tmp_0[cartesian_trajectory_planner_B.b_kstr_mu];
  }

  cartesian_trajectory_planner_B.b_kstr_mu = jtype->size[0] * jtype->size[1];
  jtype->size[0] = 1;
  jtype->size[1] = iobj_0->Type->size[1];
  cartes_emxEnsureCapacity_char_T(jtype,
    cartesian_trajectory_planner_B.b_kstr_mu);
  cartesian_trajectory_planner_B.loop_ub_gu = iobj_0->Type->size[0] *
    iobj_0->Type->size[1] - 1;
  for (cartesian_trajectory_planner_B.b_kstr_mu = 0;
       cartesian_trajectory_planner_B.b_kstr_mu <=
       cartesian_trajectory_planner_B.loop_ub_gu;
       cartesian_trajectory_planner_B.b_kstr_mu++) {
    jtype->data[cartesian_trajectory_planner_B.b_kstr_mu] = iobj_0->Type->
      data[cartesian_trajectory_planner_B.b_kstr_mu];
  }

  for (cartesian_trajectory_planner_B.b_kstr_mu = 0;
       cartesian_trajectory_planner_B.b_kstr_mu < 8;
       cartesian_trajectory_planner_B.b_kstr_mu++) {
    cartesian_trajectory_planner_B.b_dx[cartesian_trajectory_planner_B.b_kstr_mu]
      = tmp_1[cartesian_trajectory_planner_B.b_kstr_mu];
  }

  cartesian_trajectory_planner_B.b_bool_c1 = false;
  if (jtype->size[1] == 8) {
    cartesian_trajectory_planner_B.b_kstr_mu = 1;
    do {
      exitg1 = 0;
      if (cartesian_trajectory_planner_B.b_kstr_mu - 1 < 8) {
        cartesian_trajectory_planner_B.loop_ub_gu =
          cartesian_trajectory_planner_B.b_kstr_mu - 1;
        if (jtype->data[cartesian_trajectory_planner_B.loop_ub_gu] !=
            cartesian_trajectory_planner_B.b_dx[cartesian_trajectory_planner_B.loop_ub_gu])
        {
          exitg1 = 1;
        } else {
          cartesian_trajectory_planner_B.b_kstr_mu++;
        }
      } else {
        cartesian_trajectory_planner_B.b_bool_c1 = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (cartesian_trajectory_planner_B.b_bool_c1) {
    cartesian_trajectory_planner_B.b_kstr_mu = 0;
  } else {
    for (cartesian_trajectory_planner_B.b_kstr_mu = 0;
         cartesian_trajectory_planner_B.b_kstr_mu < 9;
         cartesian_trajectory_planner_B.b_kstr_mu++) {
      cartesian_trajectory_planner_B.b_cq[cartesian_trajectory_planner_B.b_kstr_mu]
        = tmp_2[cartesian_trajectory_planner_B.b_kstr_mu];
    }

    cartesian_trajectory_planner_B.b_bool_c1 = false;
    if (jtype->size[1] == 9) {
      cartesian_trajectory_planner_B.b_kstr_mu = 1;
      do {
        exitg1 = 0;
        if (cartesian_trajectory_planner_B.b_kstr_mu - 1 < 9) {
          cartesian_trajectory_planner_B.loop_ub_gu =
            cartesian_trajectory_planner_B.b_kstr_mu - 1;
          if (jtype->data[cartesian_trajectory_planner_B.loop_ub_gu] !=
              cartesian_trajectory_planner_B.b_cq[cartesian_trajectory_planner_B.loop_ub_gu])
          {
            exitg1 = 1;
          } else {
            cartesian_trajectory_planner_B.b_kstr_mu++;
          }
        } else {
          cartesian_trajectory_planner_B.b_bool_c1 = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (cartesian_trajectory_planner_B.b_bool_c1) {
      cartesian_trajectory_planner_B.b_kstr_mu = 1;
    } else {
      cartesian_trajectory_planner_B.b_kstr_mu = -1;
    }
  }

  switch (cartesian_trajectory_planner_B.b_kstr_mu) {
   case 0:
    cartesian_trajectory_planner_B.iv2[0] = 0;
    cartesian_trajectory_planner_B.iv2[1] = 0;
    cartesian_trajectory_planner_B.iv2[2] = 1;
    cartesian_trajectory_planner_B.iv2[3] = 0;
    cartesian_trajectory_planner_B.iv2[4] = 0;
    cartesian_trajectory_planner_B.iv2[5] = 0;
    for (cartesian_trajectory_planner_B.b_kstr_mu = 0;
         cartesian_trajectory_planner_B.b_kstr_mu < 6;
         cartesian_trajectory_planner_B.b_kstr_mu++) {
      cartesian_trajectory_planner_B.msubspace_data_a[cartesian_trajectory_planner_B.b_kstr_mu]
        =
        cartesian_trajectory_planner_B.iv2[cartesian_trajectory_planner_B.b_kstr_mu];
    }

    cartesian_trajectory_planner_B.poslim_data_i[0] = -3.1415926535897931;
    cartesian_trajectory_planner_B.poslim_data_i[1] = 3.1415926535897931;
    iobj_0->VelocityNumber = 1.0;
    iobj_0->PositionNumber = 1.0;
    iobj_0->JointAxisInternal[0] = 0.0;
    iobj_0->JointAxisInternal[1] = 0.0;
    iobj_0->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    cartesian_trajectory_planner_B.iv2[0] = 0;
    cartesian_trajectory_planner_B.iv2[1] = 0;
    cartesian_trajectory_planner_B.iv2[2] = 0;
    cartesian_trajectory_planner_B.iv2[3] = 0;
    cartesian_trajectory_planner_B.iv2[4] = 0;
    cartesian_trajectory_planner_B.iv2[5] = 1;
    for (cartesian_trajectory_planner_B.b_kstr_mu = 0;
         cartesian_trajectory_planner_B.b_kstr_mu < 6;
         cartesian_trajectory_planner_B.b_kstr_mu++) {
      cartesian_trajectory_planner_B.msubspace_data_a[cartesian_trajectory_planner_B.b_kstr_mu]
        =
        cartesian_trajectory_planner_B.iv2[cartesian_trajectory_planner_B.b_kstr_mu];
    }

    cartesian_trajectory_planner_B.poslim_data_i[0] = -0.5;
    cartesian_trajectory_planner_B.poslim_data_i[1] = 0.5;
    iobj_0->VelocityNumber = 1.0;
    iobj_0->PositionNumber = 1.0;
    iobj_0->JointAxisInternal[0] = 0.0;
    iobj_0->JointAxisInternal[1] = 0.0;
    iobj_0->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (cartesian_trajectory_planner_B.b_kstr_mu = 0;
         cartesian_trajectory_planner_B.b_kstr_mu < 6;
         cartesian_trajectory_planner_B.b_kstr_mu++) {
      cartesian_trajectory_planner_B.msubspace_data_a[cartesian_trajectory_planner_B.b_kstr_mu]
        = 0;
    }

    cartesian_trajectory_planner_B.poslim_data_i[0] = 0.0;
    cartesian_trajectory_planner_B.poslim_data_i[1] = 0.0;
    iobj_0->VelocityNumber = 0.0;
    iobj_0->PositionNumber = 0.0;
    iobj_0->JointAxisInternal[0] = 0.0;
    iobj_0->JointAxisInternal[1] = 0.0;
    iobj_0->JointAxisInternal[2] = 0.0;
    break;
  }

  cartesian_trajectory_planner_B.b_kstr_mu = iobj_0->MotionSubspace->size[0] *
    iobj_0->MotionSubspace->size[1];
  iobj_0->MotionSubspace->size[0] = 6;
  iobj_0->MotionSubspace->size[1] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_0->MotionSubspace,
    cartesian_trajectory_planner_B.b_kstr_mu);
  for (cartesian_trajectory_planner_B.b_kstr_mu = 0;
       cartesian_trajectory_planner_B.b_kstr_mu < 6;
       cartesian_trajectory_planner_B.b_kstr_mu++) {
    iobj_0->MotionSubspace->data[cartesian_trajectory_planner_B.b_kstr_mu] =
      cartesian_trajectory_planner_B.msubspace_data_a[cartesian_trajectory_planner_B.b_kstr_mu];
  }

  cartesian_trajectory_planner_B.b_kstr_mu = iobj_0->
    PositionLimitsInternal->size[0] * iobj_0->PositionLimitsInternal->size[1];
  iobj_0->PositionLimitsInternal->size[0] = 1;
  iobj_0->PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(iobj_0->PositionLimitsInternal,
    cartesian_trajectory_planner_B.b_kstr_mu);
  for (cartesian_trajectory_planner_B.b_kstr_mu = 0;
       cartesian_trajectory_planner_B.b_kstr_mu < 2;
       cartesian_trajectory_planner_B.b_kstr_mu++) {
    iobj_0->PositionLimitsInternal->
      data[cartesian_trajectory_planner_B.b_kstr_mu] =
      cartesian_trajectory_planner_B.poslim_data_i[cartesian_trajectory_planner_B.b_kstr_mu];
  }

  cartesian_trajectory_planner_B.b_kstr_mu = iobj_0->HomePositionInternal->size
    [0];
  iobj_0->HomePositionInternal->size[0] = 1;
  cartes_emxEnsureCapacity_real_T(iobj_0->HomePositionInternal,
    cartesian_trajectory_planner_B.b_kstr_mu);
  for (cartesian_trajectory_planner_B.b_kstr_mu = 0;
       cartesian_trajectory_planner_B.b_kstr_mu < 1;
       cartesian_trajectory_planner_B.b_kstr_mu++) {
    iobj_0->HomePositionInternal->data[0] = 0.0;
  }

  iobj_2->JointInternal = iobj_0;
  iobj_2->Index = -1.0;
  iobj_2->ParentIndex = -1.0;
  cartesian_trajectory_planner_B.b_kstr_mu = jtype->size[0] * jtype->size[1];
  jtype->size[0] = 1;
  jtype->size[1] = obj->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(jtype,
    cartesian_trajectory_planner_B.b_kstr_mu);
  cartesian_trajectory_planner_B.loop_ub_gu = obj->JointInternal.Type->size[0] *
    obj->JointInternal.Type->size[1] - 1;
  for (cartesian_trajectory_planner_B.b_kstr_mu = 0;
       cartesian_trajectory_planner_B.b_kstr_mu <=
       cartesian_trajectory_planner_B.loop_ub_gu;
       cartesian_trajectory_planner_B.b_kstr_mu++) {
    jtype->data[cartesian_trajectory_planner_B.b_kstr_mu] =
      obj->JointInternal.Type->data[cartesian_trajectory_planner_B.b_kstr_mu];
  }

  cartesian_trajec_emxInit_char_T(&jname, 2);
  cartesian_trajectory_planner_B.b_kstr_mu = jname->size[0] * jname->size[1];
  jname->size[0] = 1;
  jname->size[1] = obj->JointInternal.NameInternal->size[1];
  cartes_emxEnsureCapacity_char_T(jname,
    cartesian_trajectory_planner_B.b_kstr_mu);
  cartesian_trajectory_planner_B.loop_ub_gu = obj->
    JointInternal.NameInternal->size[0] * obj->JointInternal.NameInternal->size
    [1] - 1;
  for (cartesian_trajectory_planner_B.b_kstr_mu = 0;
       cartesian_trajectory_planner_B.b_kstr_mu <=
       cartesian_trajectory_planner_B.loop_ub_gu;
       cartesian_trajectory_planner_B.b_kstr_mu++) {
    jname->data[cartesian_trajectory_planner_B.b_kstr_mu] =
      obj->JointInternal.NameInternal->
      data[cartesian_trajectory_planner_B.b_kstr_mu];
  }

  newjoint = c_rigidBodyJoint_rigidBodyJoint(iobj_1, jname, jtype);
  cartesian_trajectory_planner_B.b_kstr_mu = jtype->size[0] * jtype->size[1];
  jtype->size[0] = 1;
  jtype->size[1] = obj->JointInternal.NameInternal->size[1];
  cartes_emxEnsureCapacity_char_T(jtype,
    cartesian_trajectory_planner_B.b_kstr_mu);
  cartesian_trajectory_planner_B.loop_ub_gu = obj->
    JointInternal.NameInternal->size[0] * obj->JointInternal.NameInternal->size
    [1] - 1;
  cartesian_trajec_emxFree_char_T(&jname);
  for (cartesian_trajectory_planner_B.b_kstr_mu = 0;
       cartesian_trajectory_planner_B.b_kstr_mu <=
       cartesian_trajectory_planner_B.loop_ub_gu;
       cartesian_trajectory_planner_B.b_kstr_mu++) {
    jtype->data[cartesian_trajectory_planner_B.b_kstr_mu] =
      obj->JointInternal.NameInternal->
      data[cartesian_trajectory_planner_B.b_kstr_mu];
  }

  if (jtype->size[1] != 0) {
    cartesian_trajectory_planner_B.b_kstr_mu = jtype->size[0] * jtype->size[1];
    jtype->size[0] = 1;
    jtype->size[1] = obj->JointInternal.NameInternal->size[1];
    cartes_emxEnsureCapacity_char_T(jtype,
      cartesian_trajectory_planner_B.b_kstr_mu);
    cartesian_trajectory_planner_B.loop_ub_gu = obj->
      JointInternal.NameInternal->size[0] * obj->
      JointInternal.NameInternal->size[1] - 1;
    for (cartesian_trajectory_planner_B.b_kstr_mu = 0;
         cartesian_trajectory_planner_B.b_kstr_mu <=
         cartesian_trajectory_planner_B.loop_ub_gu;
         cartesian_trajectory_planner_B.b_kstr_mu++) {
      jtype->data[cartesian_trajectory_planner_B.b_kstr_mu] =
        obj->JointInternal.NameInternal->
        data[cartesian_trajectory_planner_B.b_kstr_mu];
    }

    if (!newjoint->InTree) {
      cartesian_trajectory_planner_B.b_kstr_mu = newjoint->NameInternal->size[0]
        * newjoint->NameInternal->size[1];
      newjoint->NameInternal->size[0] = 1;
      newjoint->NameInternal->size[1] = jtype->size[1];
      cartes_emxEnsureCapacity_char_T(newjoint->NameInternal,
        cartesian_trajectory_planner_B.b_kstr_mu);
      cartesian_trajectory_planner_B.loop_ub_gu = jtype->size[0] * jtype->size[1]
        - 1;
      for (cartesian_trajectory_planner_B.b_kstr_mu = 0;
           cartesian_trajectory_planner_B.b_kstr_mu <=
           cartesian_trajectory_planner_B.loop_ub_gu;
           cartesian_trajectory_planner_B.b_kstr_mu++) {
        newjoint->NameInternal->data[cartesian_trajectory_planner_B.b_kstr_mu] =
          jtype->data[cartesian_trajectory_planner_B.b_kstr_mu];
      }
    }
  }

  cartesian_trajec_emxFree_char_T(&jtype);
  cartesian_trajec_emxInit_real_T(&obj_0, 1);
  cartesian_trajectory_planner_B.loop_ub_gu =
    obj->JointInternal.PositionLimitsInternal->size[0] *
    obj->JointInternal.PositionLimitsInternal->size[1];
  cartesian_trajectory_planner_B.b_kstr_mu = newjoint->
    PositionLimitsInternal->size[0] * newjoint->PositionLimitsInternal->size[1];
  newjoint->PositionLimitsInternal->size[0] =
    obj->JointInternal.PositionLimitsInternal->size[0];
  newjoint->PositionLimitsInternal->size[1] = 2;
  cartes_emxEnsureCapacity_real_T(newjoint->PositionLimitsInternal,
    cartesian_trajectory_planner_B.b_kstr_mu);
  cartesian_trajectory_planner_B.b_kstr_mu = obj_0->size[0];
  obj_0->size[0] = cartesian_trajectory_planner_B.loop_ub_gu;
  cartes_emxEnsureCapacity_real_T(obj_0,
    cartesian_trajectory_planner_B.b_kstr_mu);
  for (cartesian_trajectory_planner_B.b_kstr_mu = 0;
       cartesian_trajectory_planner_B.b_kstr_mu <
       cartesian_trajectory_planner_B.loop_ub_gu;
       cartesian_trajectory_planner_B.b_kstr_mu++) {
    obj_0->data[cartesian_trajectory_planner_B.b_kstr_mu] =
      obj->JointInternal.PositionLimitsInternal->
      data[cartesian_trajectory_planner_B.b_kstr_mu];
  }

  cartesian_trajectory_planner_B.loop_ub_gu = obj_0->size[0];
  for (cartesian_trajectory_planner_B.b_kstr_mu = 0;
       cartesian_trajectory_planner_B.b_kstr_mu <
       cartesian_trajectory_planner_B.loop_ub_gu;
       cartesian_trajectory_planner_B.b_kstr_mu++) {
    newjoint->PositionLimitsInternal->
      data[cartesian_trajectory_planner_B.b_kstr_mu] = obj_0->
      data[cartesian_trajectory_planner_B.b_kstr_mu];
  }

  cartesian_trajec_emxFree_real_T(&obj_0);
  cartesian_trajec_emxInit_real_T(&obj_1, 1);
  cartesian_trajectory_planner_B.b_kstr_mu = obj_1->size[0];
  obj_1->size[0] = obj->JointInternal.HomePositionInternal->size[0];
  cartes_emxEnsureCapacity_real_T(obj_1,
    cartesian_trajectory_planner_B.b_kstr_mu);
  cartesian_trajectory_planner_B.loop_ub_gu =
    obj->JointInternal.HomePositionInternal->size[0];
  for (cartesian_trajectory_planner_B.b_kstr_mu = 0;
       cartesian_trajectory_planner_B.b_kstr_mu <
       cartesian_trajectory_planner_B.loop_ub_gu;
       cartesian_trajectory_planner_B.b_kstr_mu++) {
    obj_1->data[cartesian_trajectory_planner_B.b_kstr_mu] =
      obj->JointInternal.HomePositionInternal->
      data[cartesian_trajectory_planner_B.b_kstr_mu];
  }

  cartesian_trajectory_planner_B.b_kstr_mu = newjoint->
    HomePositionInternal->size[0];
  newjoint->HomePositionInternal->size[0] = obj_1->size[0];
  cartes_emxEnsureCapacity_real_T(newjoint->HomePositionInternal,
    cartesian_trajectory_planner_B.b_kstr_mu);
  cartesian_trajectory_planner_B.loop_ub_gu = obj_1->size[0];
  for (cartesian_trajectory_planner_B.b_kstr_mu = 0;
       cartesian_trajectory_planner_B.b_kstr_mu <
       cartesian_trajectory_planner_B.loop_ub_gu;
       cartesian_trajectory_planner_B.b_kstr_mu++) {
    newjoint->HomePositionInternal->
      data[cartesian_trajectory_planner_B.b_kstr_mu] = obj_1->
      data[cartesian_trajectory_planner_B.b_kstr_mu];
  }

  cartesian_trajec_emxFree_real_T(&obj_1);
  cartesian_trajectory_planner_B.obj_idx_0 =
    obj->JointInternal.JointAxisInternal[0];
  cartesian_trajectory_planner_B.obj_idx_1 =
    obj->JointInternal.JointAxisInternal[1];
  cartesian_trajectory_planner_B.obj_idx_2 =
    obj->JointInternal.JointAxisInternal[2];
  newjoint->JointAxisInternal[0] = cartesian_trajectory_planner_B.obj_idx_0;
  newjoint->JointAxisInternal[1] = cartesian_trajectory_planner_B.obj_idx_1;
  newjoint->JointAxisInternal[2] = cartesian_trajectory_planner_B.obj_idx_2;
  cartesian_trajec_emxInit_real_T(&obj_2, 1);
  cartesian_trajectory_planner_B.loop_ub_gu = obj->
    JointInternal.MotionSubspace->size[0] * obj->
    JointInternal.MotionSubspace->size[1];
  cartesian_trajectory_planner_B.b_kstr_mu = newjoint->MotionSubspace->size[0] *
    newjoint->MotionSubspace->size[1];
  newjoint->MotionSubspace->size[0] = 6;
  newjoint->MotionSubspace->size[1] = obj->JointInternal.MotionSubspace->size[1];
  cartes_emxEnsureCapacity_real_T(newjoint->MotionSubspace,
    cartesian_trajectory_planner_B.b_kstr_mu);
  cartesian_trajectory_planner_B.b_kstr_mu = obj_2->size[0];
  obj_2->size[0] = cartesian_trajectory_planner_B.loop_ub_gu;
  cartes_emxEnsureCapacity_real_T(obj_2,
    cartesian_trajectory_planner_B.b_kstr_mu);
  for (cartesian_trajectory_planner_B.b_kstr_mu = 0;
       cartesian_trajectory_planner_B.b_kstr_mu <
       cartesian_trajectory_planner_B.loop_ub_gu;
       cartesian_trajectory_planner_B.b_kstr_mu++) {
    obj_2->data[cartesian_trajectory_planner_B.b_kstr_mu] =
      obj->JointInternal.MotionSubspace->
      data[cartesian_trajectory_planner_B.b_kstr_mu];
  }

  cartesian_trajectory_planner_B.loop_ub_gu = obj_2->size[0];
  for (cartesian_trajectory_planner_B.b_kstr_mu = 0;
       cartesian_trajectory_planner_B.b_kstr_mu <
       cartesian_trajectory_planner_B.loop_ub_gu;
       cartesian_trajectory_planner_B.b_kstr_mu++) {
    newjoint->MotionSubspace->data[cartesian_trajectory_planner_B.b_kstr_mu] =
      obj_2->data[cartesian_trajectory_planner_B.b_kstr_mu];
  }

  cartesian_trajec_emxFree_real_T(&obj_2);
  for (cartesian_trajectory_planner_B.b_kstr_mu = 0;
       cartesian_trajectory_planner_B.b_kstr_mu < 16;
       cartesian_trajectory_planner_B.b_kstr_mu++) {
    cartesian_trajectory_planner_B.obj[cartesian_trajectory_planner_B.b_kstr_mu]
      = obj->
      JointInternal.JointToParentTransform[cartesian_trajectory_planner_B.b_kstr_mu];
  }

  for (cartesian_trajectory_planner_B.b_kstr_mu = 0;
       cartesian_trajectory_planner_B.b_kstr_mu < 16;
       cartesian_trajectory_planner_B.b_kstr_mu++) {
    newjoint->JointToParentTransform[cartesian_trajectory_planner_B.b_kstr_mu] =
      cartesian_trajectory_planner_B.obj[cartesian_trajectory_planner_B.b_kstr_mu];
  }

  for (cartesian_trajectory_planner_B.b_kstr_mu = 0;
       cartesian_trajectory_planner_B.b_kstr_mu < 16;
       cartesian_trajectory_planner_B.b_kstr_mu++) {
    cartesian_trajectory_planner_B.obj[cartesian_trajectory_planner_B.b_kstr_mu]
      = obj->
      JointInternal.ChildToJointTransform[cartesian_trajectory_planner_B.b_kstr_mu];
  }

  for (cartesian_trajectory_planner_B.b_kstr_mu = 0;
       cartesian_trajectory_planner_B.b_kstr_mu < 16;
       cartesian_trajectory_planner_B.b_kstr_mu++) {
    newjoint->ChildToJointTransform[cartesian_trajectory_planner_B.b_kstr_mu] =
      cartesian_trajectory_planner_B.obj[cartesian_trajectory_planner_B.b_kstr_mu];
  }

  iobj_2->JointInternal = newjoint;
  return newbody;
}

static void cartesian_RigidBodyTree_addBody(x_robotics_manip_internal_Rig_T *obj,
  v_robotics_manip_internal_Rig_T *bodyin, const emxArray_char_T_cartesian_tra_T
  *parentName, c_rigidBodyJoint_cartesian_as_T *iobj_0,
  c_rigidBodyJoint_cartesian_as_T *iobj_1, w_robotics_manip_internal_Rig_T
  *iobj_2)
{
  w_robotics_manip_internal_Rig_T *body;
  c_rigidBodyJoint_cartesian_as_T *jnt;
  emxArray_char_T_cartesian_tra_T *bname;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  boolean_T exitg1;
  int32_T exitg2;
  cartesian_trajec_emxInit_char_T(&bname, 2);
  cartesian_trajectory_planner_B.pid = -1.0;
  cartesian_trajectory_planner_B.b_kstr_f = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj->Base.NameInternal->size[1];
  cartes_emxEnsureCapacity_char_T(bname, cartesian_trajectory_planner_B.b_kstr_f);
  cartesian_trajectory_planner_B.loop_ub_ay = obj->Base.NameInternal->size[0] *
    obj->Base.NameInternal->size[1] - 1;
  for (cartesian_trajectory_planner_B.b_kstr_f = 0;
       cartesian_trajectory_planner_B.b_kstr_f <=
       cartesian_trajectory_planner_B.loop_ub_ay;
       cartesian_trajectory_planner_B.b_kstr_f++) {
    bname->data[cartesian_trajectory_planner_B.b_kstr_f] =
      obj->Base.NameInternal->data[cartesian_trajectory_planner_B.b_kstr_f];
  }

  if (cartesian_trajectory_pla_strcmp(bname, parentName)) {
    cartesian_trajectory_planner_B.pid = 0.0;
  } else {
    cartesian_trajectory_planner_B.b_index_f = obj->NumBodies;
    cartesian_trajectory_planner_B.b_i_n = 0;
    exitg1 = false;
    while ((!exitg1) && (cartesian_trajectory_planner_B.b_i_n <= static_cast<
                         int32_T>(cartesian_trajectory_planner_B.b_index_f) - 1))
    {
      body = obj->Bodies[cartesian_trajectory_planner_B.b_i_n];
      cartesian_trajectory_planner_B.b_kstr_f = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = body->NameInternal->size[1];
      cartes_emxEnsureCapacity_char_T(bname,
        cartesian_trajectory_planner_B.b_kstr_f);
      cartesian_trajectory_planner_B.loop_ub_ay = body->NameInternal->size[0] *
        body->NameInternal->size[1] - 1;
      for (cartesian_trajectory_planner_B.b_kstr_f = 0;
           cartesian_trajectory_planner_B.b_kstr_f <=
           cartesian_trajectory_planner_B.loop_ub_ay;
           cartesian_trajectory_planner_B.b_kstr_f++) {
        bname->data[cartesian_trajectory_planner_B.b_kstr_f] =
          body->NameInternal->data[cartesian_trajectory_planner_B.b_kstr_f];
      }

      if (cartesian_trajectory_pla_strcmp(bname, parentName)) {
        cartesian_trajectory_planner_B.pid = static_cast<real_T>
          (cartesian_trajectory_planner_B.b_i_n) + 1.0;
        exitg1 = true;
      } else {
        cartesian_trajectory_planner_B.b_i_n++;
      }
    }
  }

  cartesian_trajectory_planner_B.b_index_f = obj->NumBodies + 1.0;
  body = cartesian_trajec_RigidBody_copy(bodyin, iobj_1, iobj_0, iobj_2);
  obj->Bodies[static_cast<int32_T>(cartesian_trajectory_planner_B.b_index_f) - 1]
    = body;
  body->Index = cartesian_trajectory_planner_B.b_index_f;
  body->ParentIndex = cartesian_trajectory_planner_B.pid;
  body->JointInternal->InTree = true;
  obj->NumBodies++;
  jnt = body->JointInternal;
  cartesian_trajectory_planner_B.b_kstr_f = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = jnt->Type->size[1];
  cartes_emxEnsureCapacity_char_T(bname, cartesian_trajectory_planner_B.b_kstr_f);
  cartesian_trajectory_planner_B.loop_ub_ay = jnt->Type->size[0] * jnt->
    Type->size[1] - 1;
  for (cartesian_trajectory_planner_B.b_kstr_f = 0;
       cartesian_trajectory_planner_B.b_kstr_f <=
       cartesian_trajectory_planner_B.loop_ub_ay;
       cartesian_trajectory_planner_B.b_kstr_f++) {
    bname->data[cartesian_trajectory_planner_B.b_kstr_f] = jnt->Type->
      data[cartesian_trajectory_planner_B.b_kstr_f];
  }

  for (cartesian_trajectory_planner_B.b_kstr_f = 0;
       cartesian_trajectory_planner_B.b_kstr_f < 5;
       cartesian_trajectory_planner_B.b_kstr_f++) {
    cartesian_trajectory_planner_B.b_ho[cartesian_trajectory_planner_B.b_kstr_f]
      = tmp[cartesian_trajectory_planner_B.b_kstr_f];
  }

  cartesian_trajectory_planner_B.b_bool_g = false;
  if (bname->size[1] == 5) {
    cartesian_trajectory_planner_B.b_kstr_f = 1;
    do {
      exitg2 = 0;
      if (cartesian_trajectory_planner_B.b_kstr_f - 1 < 5) {
        cartesian_trajectory_planner_B.loop_ub_ay =
          cartesian_trajectory_planner_B.b_kstr_f - 1;
        if (bname->data[cartesian_trajectory_planner_B.loop_ub_ay] !=
            cartesian_trajectory_planner_B.b_ho[cartesian_trajectory_planner_B.loop_ub_ay])
        {
          exitg2 = 1;
        } else {
          cartesian_trajectory_planner_B.b_kstr_f++;
        }
      } else {
        cartesian_trajectory_planner_B.b_bool_g = true;
        exitg2 = 1;
      }
    } while (exitg2 == 0);
  }

  cartesian_trajec_emxFree_char_T(&bname);
  if (!cartesian_trajectory_planner_B.b_bool_g) {
    obj->NumNonFixedBodies++;
    jnt = body->JointInternal;
    cartesian_trajectory_planner_B.b_kstr_f = static_cast<int32_T>(body->Index)
      - 1;
    obj->PositionDoFMap[cartesian_trajectory_planner_B.b_kstr_f] =
      obj->PositionNumber + 1.0;
    obj->PositionDoFMap[cartesian_trajectory_planner_B.b_kstr_f + 8] =
      obj->PositionNumber + jnt->PositionNumber;
    jnt = body->JointInternal;
    cartesian_trajectory_planner_B.b_kstr_f = static_cast<int32_T>(body->Index)
      - 1;
    obj->VelocityDoFMap[cartesian_trajectory_planner_B.b_kstr_f] =
      obj->VelocityNumber + 1.0;
    obj->VelocityDoFMap[cartesian_trajectory_planner_B.b_kstr_f + 8] =
      obj->VelocityNumber + jnt->VelocityNumber;
  } else {
    cartesian_trajectory_planner_B.b_kstr_f = static_cast<int32_T>(body->Index);
    obj->PositionDoFMap[cartesian_trajectory_planner_B.b_kstr_f - 1] = 0.0;
    obj->PositionDoFMap[cartesian_trajectory_planner_B.b_kstr_f + 7] = -1.0;
    cartesian_trajectory_planner_B.b_kstr_f = static_cast<int32_T>(body->Index);
    obj->VelocityDoFMap[cartesian_trajectory_planner_B.b_kstr_f - 1] = 0.0;
    obj->VelocityDoFMap[cartesian_trajectory_planner_B.b_kstr_f + 7] = -1.0;
  }

  jnt = body->JointInternal;
  obj->PositionNumber += jnt->PositionNumber;
  jnt = body->JointInternal;
  obj->VelocityNumber += jnt->VelocityNumber;
}

static void inverseKinematics_set_RigidBody(b_inverseKinematics_cartesian_T *obj,
  y_robotics_manip_internal_Rig_T *rigidbodytree,
  w_robotics_manip_internal_Rig_T *iobj_0, w_robotics_manip_internal_Rig_T
  *iobj_1, w_robotics_manip_internal_Rig_T *iobj_2,
  w_robotics_manip_internal_Rig_T *iobj_3, w_robotics_manip_internal_Rig_T
  *iobj_4, w_robotics_manip_internal_Rig_T *iobj_5,
  w_robotics_manip_internal_Rig_T *iobj_6, w_robotics_manip_internal_Rig_T
  *iobj_7, w_robotics_manip_internal_Rig_T *iobj_8,
  w_robotics_manip_internal_Rig_T *iobj_9, w_robotics_manip_internal_Rig_T
  *iobj_10, w_robotics_manip_internal_Rig_T *iobj_11,
  w_robotics_manip_internal_Rig_T *iobj_12, w_robotics_manip_internal_Rig_T
  *iobj_13, w_robotics_manip_internal_Rig_T *iobj_14,
  c_rigidBodyJoint_cartesian_as_T *iobj_15, c_rigidBodyJoint_cartesian_as_T
  *iobj_16, c_rigidBodyJoint_cartesian_as_T *iobj_17,
  c_rigidBodyJoint_cartesian_as_T *iobj_18, c_rigidBodyJoint_cartesian_as_T
  *iobj_19, c_rigidBodyJoint_cartesian_as_T *iobj_20,
  c_rigidBodyJoint_cartesian_as_T *iobj_21, c_rigidBodyJoint_cartesian_as_T
  *iobj_22, c_rigidBodyJoint_cartesian_as_T *iobj_23,
  c_rigidBodyJoint_cartesian_as_T *iobj_24, c_rigidBodyJoint_cartesian_as_T
  *iobj_25, c_rigidBodyJoint_cartesian_as_T *iobj_26,
  c_rigidBodyJoint_cartesian_as_T *iobj_27, c_rigidBodyJoint_cartesian_as_T
  *iobj_28, c_rigidBodyJoint_cartesian_as_T *iobj_29,
  c_rigidBodyJoint_cartesian_as_T *iobj_30, c_rigidBodyJoint_cartesian_as_T
  *iobj_31, c_rigidBodyJoint_cartesian_as_T *iobj_32,
  c_rigidBodyJoint_cartesian_as_T *iobj_33, c_rigidBodyJoint_cartesian_as_T
  *iobj_34, c_rigidBodyJoint_cartesian_as_T *iobj_35,
  c_rigidBodyJoint_cartesian_as_T *iobj_36, c_rigidBodyJoint_cartesian_as_T
  *iobj_37, c_rigidBodyJoint_cartesian_as_T *iobj_38,
  c_rigidBodyJoint_cartesian_as_T *iobj_39, w_robotics_manip_internal_Rig_T
  *iobj_40, x_robotics_manip_internal_Rig_T *iobj_41)
{
  x_robotics_manip_internal_Rig_T *newrobot;
  v_robotics_manip_internal_Rig_T *body;
  v_robotics_manip_internal_Rig_T *parent;
  emxArray_char_T_cartesian_tra_T *b_basename;
  w_robotics_manip_internal_Rig_T *body_0;
  c_rigidBodyJoint_cartesian_as_T *jnt;
  emxArray_char_T_cartesian_tra_T *bname;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  boolean_T exitg1;
  int32_T exitg2;
  cartesian_trajec_emxInit_char_T(&b_basename, 2);
  newrobot = RigidBodyTree_RigidBodyTree_ast(iobj_41, iobj_0, iobj_1, iobj_2,
    iobj_3, iobj_4, iobj_5, iobj_6, iobj_15, iobj_16, iobj_17, iobj_18, iobj_19,
    iobj_20, iobj_21, iobj_22, iobj_39, iobj_40);
  cartesian_trajectory_planner_B.b_kstr_o = b_basename->size[0] *
    b_basename->size[1];
  b_basename->size[0] = 1;
  b_basename->size[1] = rigidbodytree->Base.NameInternal->size[1];
  cartes_emxEnsureCapacity_char_T(b_basename,
    cartesian_trajectory_planner_B.b_kstr_o);
  cartesian_trajectory_planner_B.loop_ub_hm = rigidbodytree->
    Base.NameInternal->size[0] * rigidbodytree->Base.NameInternal->size[1] - 1;
  for (cartesian_trajectory_planner_B.b_kstr_o = 0;
       cartesian_trajectory_planner_B.b_kstr_o <=
       cartesian_trajectory_planner_B.loop_ub_hm;
       cartesian_trajectory_planner_B.b_kstr_o++) {
    b_basename->data[cartesian_trajectory_planner_B.b_kstr_o] =
      rigidbodytree->Base.NameInternal->
      data[cartesian_trajectory_planner_B.b_kstr_o];
  }

  cartesian_trajec_emxInit_char_T(&bname, 2);
  cartesian_trajectory_planner_B.bid_a = -1.0;
  cartesian_trajectory_planner_B.b_kstr_o = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = newrobot->Base.NameInternal->size[1];
  cartes_emxEnsureCapacity_char_T(bname, cartesian_trajectory_planner_B.b_kstr_o);
  cartesian_trajectory_planner_B.loop_ub_hm = newrobot->Base.NameInternal->size
    [0] * newrobot->Base.NameInternal->size[1] - 1;
  for (cartesian_trajectory_planner_B.b_kstr_o = 0;
       cartesian_trajectory_planner_B.b_kstr_o <=
       cartesian_trajectory_planner_B.loop_ub_hm;
       cartesian_trajectory_planner_B.b_kstr_o++) {
    bname->data[cartesian_trajectory_planner_B.b_kstr_o] =
      newrobot->Base.NameInternal->data[cartesian_trajectory_planner_B.b_kstr_o];
  }

  if (cartesian_trajectory_pla_strcmp(bname, b_basename)) {
    cartesian_trajectory_planner_B.bid_a = 0.0;
  } else {
    cartesian_trajectory_planner_B.b_index = newrobot->NumBodies;
    cartesian_trajectory_planner_B.b_i_g = 0;
    exitg1 = false;
    while ((!exitg1) && (cartesian_trajectory_planner_B.b_i_g <=
                         static_cast<int32_T>
                         (cartesian_trajectory_planner_B.b_index) - 1)) {
      body_0 = newrobot->Bodies[cartesian_trajectory_planner_B.b_i_g];
      cartesian_trajectory_planner_B.b_kstr_o = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = body_0->NameInternal->size[1];
      cartes_emxEnsureCapacity_char_T(bname,
        cartesian_trajectory_planner_B.b_kstr_o);
      cartesian_trajectory_planner_B.loop_ub_hm = body_0->NameInternal->size[0] *
        body_0->NameInternal->size[1] - 1;
      for (cartesian_trajectory_planner_B.b_kstr_o = 0;
           cartesian_trajectory_planner_B.b_kstr_o <=
           cartesian_trajectory_planner_B.loop_ub_hm;
           cartesian_trajectory_planner_B.b_kstr_o++) {
        bname->data[cartesian_trajectory_planner_B.b_kstr_o] =
          body_0->NameInternal->data[cartesian_trajectory_planner_B.b_kstr_o];
      }

      if (cartesian_trajectory_pla_strcmp(bname, b_basename)) {
        cartesian_trajectory_planner_B.bid_a = static_cast<real_T>
          (cartesian_trajectory_planner_B.b_i_g) + 1.0;
        exitg1 = true;
      } else {
        cartesian_trajectory_planner_B.b_i_g++;
      }
    }
  }

  if ((!(cartesian_trajectory_planner_B.bid_a == 0.0)) &&
      (cartesian_trajectory_planner_B.bid_a < 0.0)) {
    cartesian_trajectory_planner_B.b_kstr_o = newrobot->Base.NameInternal->size
      [0] * newrobot->Base.NameInternal->size[1];
    newrobot->Base.NameInternal->size[0] = 1;
    newrobot->Base.NameInternal->size[1] = b_basename->size[1];
    cartes_emxEnsureCapacity_char_T(newrobot->Base.NameInternal,
      cartesian_trajectory_planner_B.b_kstr_o);
    cartesian_trajectory_planner_B.loop_ub_hm = b_basename->size[0] *
      b_basename->size[1] - 1;
    for (cartesian_trajectory_planner_B.b_kstr_o = 0;
         cartesian_trajectory_planner_B.b_kstr_o <=
         cartesian_trajectory_planner_B.loop_ub_hm;
         cartesian_trajectory_planner_B.b_kstr_o++) {
      newrobot->Base.NameInternal->data[cartesian_trajectory_planner_B.b_kstr_o]
        = b_basename->data[cartesian_trajectory_planner_B.b_kstr_o];
    }
  }

  if (1.0 <= rigidbodytree->NumBodies) {
    body = rigidbodytree->Bodies[0];
    cartesian_trajectory_planner_B.bid_a = body->ParentIndex;
    if (cartesian_trajectory_planner_B.bid_a > 0.0) {
      parent = rigidbodytree->Bodies[static_cast<int32_T>
        (cartesian_trajectory_planner_B.bid_a) - 1];
    } else {
      parent = &rigidbodytree->Base;
    }

    cartesian_trajectory_planner_B.b_kstr_o = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    cartes_emxEnsureCapacity_char_T(bname,
      cartesian_trajectory_planner_B.b_kstr_o);
    cartesian_trajectory_planner_B.loop_ub_hm = parent->NameInternal->size[0] *
      parent->NameInternal->size[1] - 1;
    for (cartesian_trajectory_planner_B.b_kstr_o = 0;
         cartesian_trajectory_planner_B.b_kstr_o <=
         cartesian_trajectory_planner_B.loop_ub_hm;
         cartesian_trajectory_planner_B.b_kstr_o++) {
      bname->data[cartesian_trajectory_planner_B.b_kstr_o] =
        parent->NameInternal->data[cartesian_trajectory_planner_B.b_kstr_o];
    }

    cartesian_RigidBodyTree_addBody(newrobot, body, bname, iobj_24, iobj_23,
      iobj_7);
  }

  if (2.0 <= rigidbodytree->NumBodies) {
    body = rigidbodytree->Bodies[1];
    cartesian_trajectory_planner_B.bid_a = body->ParentIndex;
    if (cartesian_trajectory_planner_B.bid_a > 0.0) {
      parent = rigidbodytree->Bodies[static_cast<int32_T>
        (cartesian_trajectory_planner_B.bid_a) - 1];
    } else {
      parent = &rigidbodytree->Base;
    }

    cartesian_trajectory_planner_B.b_kstr_o = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    cartes_emxEnsureCapacity_char_T(bname,
      cartesian_trajectory_planner_B.b_kstr_o);
    cartesian_trajectory_planner_B.loop_ub_hm = parent->NameInternal->size[0] *
      parent->NameInternal->size[1] - 1;
    for (cartesian_trajectory_planner_B.b_kstr_o = 0;
         cartesian_trajectory_planner_B.b_kstr_o <=
         cartesian_trajectory_planner_B.loop_ub_hm;
         cartesian_trajectory_planner_B.b_kstr_o++) {
      bname->data[cartesian_trajectory_planner_B.b_kstr_o] =
        parent->NameInternal->data[cartesian_trajectory_planner_B.b_kstr_o];
    }

    cartesian_RigidBodyTree_addBody(newrobot, body, bname, iobj_26, iobj_25,
      iobj_8);
  }

  if (3.0 <= rigidbodytree->NumBodies) {
    body = rigidbodytree->Bodies[2];
    cartesian_trajectory_planner_B.bid_a = body->ParentIndex;
    if (cartesian_trajectory_planner_B.bid_a > 0.0) {
      parent = rigidbodytree->Bodies[static_cast<int32_T>
        (cartesian_trajectory_planner_B.bid_a) - 1];
    } else {
      parent = &rigidbodytree->Base;
    }

    cartesian_trajectory_planner_B.b_kstr_o = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    cartes_emxEnsureCapacity_char_T(bname,
      cartesian_trajectory_planner_B.b_kstr_o);
    cartesian_trajectory_planner_B.loop_ub_hm = parent->NameInternal->size[0] *
      parent->NameInternal->size[1] - 1;
    for (cartesian_trajectory_planner_B.b_kstr_o = 0;
         cartesian_trajectory_planner_B.b_kstr_o <=
         cartesian_trajectory_planner_B.loop_ub_hm;
         cartesian_trajectory_planner_B.b_kstr_o++) {
      bname->data[cartesian_trajectory_planner_B.b_kstr_o] =
        parent->NameInternal->data[cartesian_trajectory_planner_B.b_kstr_o];
    }

    cartesian_RigidBodyTree_addBody(newrobot, body, bname, iobj_28, iobj_27,
      iobj_9);
  }

  if (4.0 <= rigidbodytree->NumBodies) {
    body = rigidbodytree->Bodies[3];
    cartesian_trajectory_planner_B.bid_a = body->ParentIndex;
    if (cartesian_trajectory_planner_B.bid_a > 0.0) {
      parent = rigidbodytree->Bodies[static_cast<int32_T>
        (cartesian_trajectory_planner_B.bid_a) - 1];
    } else {
      parent = &rigidbodytree->Base;
    }

    cartesian_trajectory_planner_B.b_kstr_o = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    cartes_emxEnsureCapacity_char_T(bname,
      cartesian_trajectory_planner_B.b_kstr_o);
    cartesian_trajectory_planner_B.loop_ub_hm = parent->NameInternal->size[0] *
      parent->NameInternal->size[1] - 1;
    for (cartesian_trajectory_planner_B.b_kstr_o = 0;
         cartesian_trajectory_planner_B.b_kstr_o <=
         cartesian_trajectory_planner_B.loop_ub_hm;
         cartesian_trajectory_planner_B.b_kstr_o++) {
      bname->data[cartesian_trajectory_planner_B.b_kstr_o] =
        parent->NameInternal->data[cartesian_trajectory_planner_B.b_kstr_o];
    }

    cartesian_RigidBodyTree_addBody(newrobot, body, bname, iobj_30, iobj_29,
      iobj_10);
  }

  if (5.0 <= rigidbodytree->NumBodies) {
    body = rigidbodytree->Bodies[4];
    cartesian_trajectory_planner_B.bid_a = body->ParentIndex;
    if (cartesian_trajectory_planner_B.bid_a > 0.0) {
      parent = rigidbodytree->Bodies[static_cast<int32_T>
        (cartesian_trajectory_planner_B.bid_a) - 1];
    } else {
      parent = &rigidbodytree->Base;
    }

    cartesian_trajectory_planner_B.b_kstr_o = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    cartes_emxEnsureCapacity_char_T(bname,
      cartesian_trajectory_planner_B.b_kstr_o);
    cartesian_trajectory_planner_B.loop_ub_hm = parent->NameInternal->size[0] *
      parent->NameInternal->size[1] - 1;
    for (cartesian_trajectory_planner_B.b_kstr_o = 0;
         cartesian_trajectory_planner_B.b_kstr_o <=
         cartesian_trajectory_planner_B.loop_ub_hm;
         cartesian_trajectory_planner_B.b_kstr_o++) {
      bname->data[cartesian_trajectory_planner_B.b_kstr_o] =
        parent->NameInternal->data[cartesian_trajectory_planner_B.b_kstr_o];
    }

    cartesian_RigidBodyTree_addBody(newrobot, body, bname, iobj_32, iobj_31,
      iobj_11);
  }

  if (6.0 <= rigidbodytree->NumBodies) {
    body = rigidbodytree->Bodies[5];
    cartesian_trajectory_planner_B.bid_a = body->ParentIndex;
    if (cartesian_trajectory_planner_B.bid_a > 0.0) {
      parent = rigidbodytree->Bodies[static_cast<int32_T>
        (cartesian_trajectory_planner_B.bid_a) - 1];
    } else {
      parent = &rigidbodytree->Base;
    }

    cartesian_trajectory_planner_B.b_kstr_o = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    cartes_emxEnsureCapacity_char_T(bname,
      cartesian_trajectory_planner_B.b_kstr_o);
    cartesian_trajectory_planner_B.loop_ub_hm = parent->NameInternal->size[0] *
      parent->NameInternal->size[1] - 1;
    for (cartesian_trajectory_planner_B.b_kstr_o = 0;
         cartesian_trajectory_planner_B.b_kstr_o <=
         cartesian_trajectory_planner_B.loop_ub_hm;
         cartesian_trajectory_planner_B.b_kstr_o++) {
      bname->data[cartesian_trajectory_planner_B.b_kstr_o] =
        parent->NameInternal->data[cartesian_trajectory_planner_B.b_kstr_o];
    }

    cartesian_RigidBodyTree_addBody(newrobot, body, bname, iobj_34, iobj_33,
      iobj_12);
  }

  if (7.0 <= rigidbodytree->NumBodies) {
    body = rigidbodytree->Bodies[6];
    cartesian_trajectory_planner_B.bid_a = body->ParentIndex;
    if (cartesian_trajectory_planner_B.bid_a > 0.0) {
      parent = rigidbodytree->Bodies[static_cast<int32_T>
        (cartesian_trajectory_planner_B.bid_a) - 1];
    } else {
      parent = &rigidbodytree->Base;
    }

    cartesian_trajectory_planner_B.b_kstr_o = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    cartes_emxEnsureCapacity_char_T(bname,
      cartesian_trajectory_planner_B.b_kstr_o);
    cartesian_trajectory_planner_B.loop_ub_hm = parent->NameInternal->size[0] *
      parent->NameInternal->size[1] - 1;
    for (cartesian_trajectory_planner_B.b_kstr_o = 0;
         cartesian_trajectory_planner_B.b_kstr_o <=
         cartesian_trajectory_planner_B.loop_ub_hm;
         cartesian_trajectory_planner_B.b_kstr_o++) {
      bname->data[cartesian_trajectory_planner_B.b_kstr_o] =
        parent->NameInternal->data[cartesian_trajectory_planner_B.b_kstr_o];
    }

    cartesian_trajectory_planner_B.bid_a = -1.0;
    cartesian_trajectory_planner_B.b_kstr_o = b_basename->size[0] *
      b_basename->size[1];
    b_basename->size[0] = 1;
    b_basename->size[1] = newrobot->Base.NameInternal->size[1];
    cartes_emxEnsureCapacity_char_T(b_basename,
      cartesian_trajectory_planner_B.b_kstr_o);
    cartesian_trajectory_planner_B.loop_ub_hm = newrobot->
      Base.NameInternal->size[0] * newrobot->Base.NameInternal->size[1] - 1;
    for (cartesian_trajectory_planner_B.b_kstr_o = 0;
         cartesian_trajectory_planner_B.b_kstr_o <=
         cartesian_trajectory_planner_B.loop_ub_hm;
         cartesian_trajectory_planner_B.b_kstr_o++) {
      b_basename->data[cartesian_trajectory_planner_B.b_kstr_o] =
        newrobot->Base.NameInternal->
        data[cartesian_trajectory_planner_B.b_kstr_o];
    }

    if (cartesian_trajectory_pla_strcmp(b_basename, bname)) {
      cartesian_trajectory_planner_B.bid_a = 0.0;
    } else {
      cartesian_trajectory_planner_B.b_index = newrobot->NumBodies;
      cartesian_trajectory_planner_B.b_i_g = 0;
      exitg1 = false;
      while ((!exitg1) && (cartesian_trajectory_planner_B.b_i_g <=
                           static_cast<int32_T>
                           (cartesian_trajectory_planner_B.b_index) - 1)) {
        body_0 = newrobot->Bodies[cartesian_trajectory_planner_B.b_i_g];
        cartesian_trajectory_planner_B.b_kstr_o = b_basename->size[0] *
          b_basename->size[1];
        b_basename->size[0] = 1;
        b_basename->size[1] = body_0->NameInternal->size[1];
        cartes_emxEnsureCapacity_char_T(b_basename,
          cartesian_trajectory_planner_B.b_kstr_o);
        cartesian_trajectory_planner_B.loop_ub_hm = body_0->NameInternal->size[0]
          * body_0->NameInternal->size[1] - 1;
        for (cartesian_trajectory_planner_B.b_kstr_o = 0;
             cartesian_trajectory_planner_B.b_kstr_o <=
             cartesian_trajectory_planner_B.loop_ub_hm;
             cartesian_trajectory_planner_B.b_kstr_o++) {
          b_basename->data[cartesian_trajectory_planner_B.b_kstr_o] =
            body_0->NameInternal->data[cartesian_trajectory_planner_B.b_kstr_o];
        }

        if (cartesian_trajectory_pla_strcmp(b_basename, bname)) {
          cartesian_trajectory_planner_B.bid_a = static_cast<real_T>
            (cartesian_trajectory_planner_B.b_i_g) + 1.0;
          exitg1 = true;
        } else {
          cartesian_trajectory_planner_B.b_i_g++;
        }
      }
    }

    cartesian_trajectory_planner_B.b_index = newrobot->NumBodies + 1.0;
    body_0 = cartesian_trajec_RigidBody_copy(body, iobj_35, iobj_36, iobj_13);
    newrobot->Bodies[static_cast<int32_T>(cartesian_trajectory_planner_B.b_index)
      - 1] = body_0;
    body_0->Index = cartesian_trajectory_planner_B.b_index;
    body_0->ParentIndex = cartesian_trajectory_planner_B.bid_a;
    body_0->JointInternal->InTree = true;
    newrobot->NumBodies++;
    jnt = body_0->JointInternal;
    cartesian_trajectory_planner_B.b_kstr_o = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = jnt->Type->size[1];
    cartes_emxEnsureCapacity_char_T(bname,
      cartesian_trajectory_planner_B.b_kstr_o);
    cartesian_trajectory_planner_B.loop_ub_hm = jnt->Type->size[0] * jnt->
      Type->size[1] - 1;
    for (cartesian_trajectory_planner_B.b_kstr_o = 0;
         cartesian_trajectory_planner_B.b_kstr_o <=
         cartesian_trajectory_planner_B.loop_ub_hm;
         cartesian_trajectory_planner_B.b_kstr_o++) {
      bname->data[cartesian_trajectory_planner_B.b_kstr_o] = jnt->Type->
        data[cartesian_trajectory_planner_B.b_kstr_o];
    }

    for (cartesian_trajectory_planner_B.b_kstr_o = 0;
         cartesian_trajectory_planner_B.b_kstr_o < 5;
         cartesian_trajectory_planner_B.b_kstr_o++) {
      cartesian_trajectory_planner_B.b_ku[cartesian_trajectory_planner_B.b_kstr_o]
        = tmp[cartesian_trajectory_planner_B.b_kstr_o];
    }

    cartesian_trajectory_planner_B.b_bool_a = false;
    if (bname->size[1] == 5) {
      cartesian_trajectory_planner_B.b_kstr_o = 1;
      do {
        exitg2 = 0;
        if (cartesian_trajectory_planner_B.b_kstr_o - 1 < 5) {
          cartesian_trajectory_planner_B.loop_ub_hm =
            cartesian_trajectory_planner_B.b_kstr_o - 1;
          if (bname->data[cartesian_trajectory_planner_B.loop_ub_hm] !=
              cartesian_trajectory_planner_B.b_ku[cartesian_trajectory_planner_B.loop_ub_hm])
          {
            exitg2 = 1;
          } else {
            cartesian_trajectory_planner_B.b_kstr_o++;
          }
        } else {
          cartesian_trajectory_planner_B.b_bool_a = true;
          exitg2 = 1;
        }
      } while (exitg2 == 0);
    }

    if (!cartesian_trajectory_planner_B.b_bool_a) {
      newrobot->NumNonFixedBodies++;
      jnt = body_0->JointInternal;
      cartesian_trajectory_planner_B.b_kstr_o = static_cast<int32_T>
        (body_0->Index) - 1;
      newrobot->PositionDoFMap[cartesian_trajectory_planner_B.b_kstr_o] =
        newrobot->PositionNumber + 1.0;
      newrobot->PositionDoFMap[cartesian_trajectory_planner_B.b_kstr_o + 8] =
        newrobot->PositionNumber + jnt->PositionNumber;
      jnt = body_0->JointInternal;
      cartesian_trajectory_planner_B.b_kstr_o = static_cast<int32_T>
        (body_0->Index) - 1;
      newrobot->VelocityDoFMap[cartesian_trajectory_planner_B.b_kstr_o] =
        newrobot->VelocityNumber + 1.0;
      newrobot->VelocityDoFMap[cartesian_trajectory_planner_B.b_kstr_o + 8] =
        newrobot->VelocityNumber + jnt->VelocityNumber;
    } else {
      cartesian_trajectory_planner_B.b_kstr_o = static_cast<int32_T>
        (body_0->Index);
      newrobot->PositionDoFMap[cartesian_trajectory_planner_B.b_kstr_o - 1] =
        0.0;
      newrobot->PositionDoFMap[cartesian_trajectory_planner_B.b_kstr_o + 7] =
        -1.0;
      cartesian_trajectory_planner_B.b_kstr_o = static_cast<int32_T>
        (body_0->Index);
      newrobot->VelocityDoFMap[cartesian_trajectory_planner_B.b_kstr_o - 1] =
        0.0;
      newrobot->VelocityDoFMap[cartesian_trajectory_planner_B.b_kstr_o + 7] =
        -1.0;
    }

    jnt = body_0->JointInternal;
    newrobot->PositionNumber += jnt->PositionNumber;
    jnt = body_0->JointInternal;
    newrobot->VelocityNumber += jnt->VelocityNumber;
  }

  if (8.0 <= rigidbodytree->NumBodies) {
    body = rigidbodytree->Bodies[7];
    cartesian_trajectory_planner_B.bid_a = body->ParentIndex;
    if (cartesian_trajectory_planner_B.bid_a > 0.0) {
      parent = rigidbodytree->Bodies[static_cast<int32_T>
        (cartesian_trajectory_planner_B.bid_a) - 1];
    } else {
      parent = &rigidbodytree->Base;
    }

    cartesian_trajectory_planner_B.b_kstr_o = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    cartes_emxEnsureCapacity_char_T(bname,
      cartesian_trajectory_planner_B.b_kstr_o);
    cartesian_trajectory_planner_B.loop_ub_hm = parent->NameInternal->size[0] *
      parent->NameInternal->size[1] - 1;
    for (cartesian_trajectory_planner_B.b_kstr_o = 0;
         cartesian_trajectory_planner_B.b_kstr_o <=
         cartesian_trajectory_planner_B.loop_ub_hm;
         cartesian_trajectory_planner_B.b_kstr_o++) {
      bname->data[cartesian_trajectory_planner_B.b_kstr_o] =
        parent->NameInternal->data[cartesian_trajectory_planner_B.b_kstr_o];
    }

    cartesian_trajectory_planner_B.bid_a = -1.0;
    cartesian_trajectory_planner_B.b_kstr_o = b_basename->size[0] *
      b_basename->size[1];
    b_basename->size[0] = 1;
    b_basename->size[1] = newrobot->Base.NameInternal->size[1];
    cartes_emxEnsureCapacity_char_T(b_basename,
      cartesian_trajectory_planner_B.b_kstr_o);
    cartesian_trajectory_planner_B.loop_ub_hm = newrobot->
      Base.NameInternal->size[0] * newrobot->Base.NameInternal->size[1] - 1;
    for (cartesian_trajectory_planner_B.b_kstr_o = 0;
         cartesian_trajectory_planner_B.b_kstr_o <=
         cartesian_trajectory_planner_B.loop_ub_hm;
         cartesian_trajectory_planner_B.b_kstr_o++) {
      b_basename->data[cartesian_trajectory_planner_B.b_kstr_o] =
        newrobot->Base.NameInternal->
        data[cartesian_trajectory_planner_B.b_kstr_o];
    }

    if (cartesian_trajectory_pla_strcmp(b_basename, bname)) {
      cartesian_trajectory_planner_B.bid_a = 0.0;
    } else {
      cartesian_trajectory_planner_B.b_index = newrobot->NumBodies;
      cartesian_trajectory_planner_B.b_i_g = 0;
      exitg1 = false;
      while ((!exitg1) && (cartesian_trajectory_planner_B.b_i_g <=
                           static_cast<int32_T>
                           (cartesian_trajectory_planner_B.b_index) - 1)) {
        body_0 = newrobot->Bodies[cartesian_trajectory_planner_B.b_i_g];
        cartesian_trajectory_planner_B.b_kstr_o = b_basename->size[0] *
          b_basename->size[1];
        b_basename->size[0] = 1;
        b_basename->size[1] = body_0->NameInternal->size[1];
        cartes_emxEnsureCapacity_char_T(b_basename,
          cartesian_trajectory_planner_B.b_kstr_o);
        cartesian_trajectory_planner_B.loop_ub_hm = body_0->NameInternal->size[0]
          * body_0->NameInternal->size[1] - 1;
        for (cartesian_trajectory_planner_B.b_kstr_o = 0;
             cartesian_trajectory_planner_B.b_kstr_o <=
             cartesian_trajectory_planner_B.loop_ub_hm;
             cartesian_trajectory_planner_B.b_kstr_o++) {
          b_basename->data[cartesian_trajectory_planner_B.b_kstr_o] =
            body_0->NameInternal->data[cartesian_trajectory_planner_B.b_kstr_o];
        }

        if (cartesian_trajectory_pla_strcmp(b_basename, bname)) {
          cartesian_trajectory_planner_B.bid_a = static_cast<real_T>
            (cartesian_trajectory_planner_B.b_i_g) + 1.0;
          exitg1 = true;
        } else {
          cartesian_trajectory_planner_B.b_i_g++;
        }
      }
    }

    cartesian_trajectory_planner_B.b_index = newrobot->NumBodies + 1.0;
    body_0 = cartesian_trajec_RigidBody_copy(body, iobj_37, iobj_38, iobj_14);
    newrobot->Bodies[static_cast<int32_T>(cartesian_trajectory_planner_B.b_index)
      - 1] = body_0;
    body_0->Index = cartesian_trajectory_planner_B.b_index;
    body_0->ParentIndex = cartesian_trajectory_planner_B.bid_a;
    body_0->JointInternal->InTree = true;
    newrobot->NumBodies++;
    jnt = body_0->JointInternal;
    cartesian_trajectory_planner_B.b_kstr_o = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = jnt->Type->size[1];
    cartes_emxEnsureCapacity_char_T(bname,
      cartesian_trajectory_planner_B.b_kstr_o);
    cartesian_trajectory_planner_B.loop_ub_hm = jnt->Type->size[0] * jnt->
      Type->size[1] - 1;
    for (cartesian_trajectory_planner_B.b_kstr_o = 0;
         cartesian_trajectory_planner_B.b_kstr_o <=
         cartesian_trajectory_planner_B.loop_ub_hm;
         cartesian_trajectory_planner_B.b_kstr_o++) {
      bname->data[cartesian_trajectory_planner_B.b_kstr_o] = jnt->Type->
        data[cartesian_trajectory_planner_B.b_kstr_o];
    }

    for (cartesian_trajectory_planner_B.b_kstr_o = 0;
         cartesian_trajectory_planner_B.b_kstr_o < 5;
         cartesian_trajectory_planner_B.b_kstr_o++) {
      cartesian_trajectory_planner_B.b_ku[cartesian_trajectory_planner_B.b_kstr_o]
        = tmp[cartesian_trajectory_planner_B.b_kstr_o];
    }

    cartesian_trajectory_planner_B.b_bool_a = false;
    if (bname->size[1] == 5) {
      cartesian_trajectory_planner_B.b_kstr_o = 1;
      do {
        exitg2 = 0;
        if (cartesian_trajectory_planner_B.b_kstr_o - 1 < 5) {
          cartesian_trajectory_planner_B.loop_ub_hm =
            cartesian_trajectory_planner_B.b_kstr_o - 1;
          if (bname->data[cartesian_trajectory_planner_B.loop_ub_hm] !=
              cartesian_trajectory_planner_B.b_ku[cartesian_trajectory_planner_B.loop_ub_hm])
          {
            exitg2 = 1;
          } else {
            cartesian_trajectory_planner_B.b_kstr_o++;
          }
        } else {
          cartesian_trajectory_planner_B.b_bool_a = true;
          exitg2 = 1;
        }
      } while (exitg2 == 0);
    }

    if (!cartesian_trajectory_planner_B.b_bool_a) {
      newrobot->NumNonFixedBodies++;
      jnt = body_0->JointInternal;
      cartesian_trajectory_planner_B.b_kstr_o = static_cast<int32_T>
        (body_0->Index) - 1;
      newrobot->PositionDoFMap[cartesian_trajectory_planner_B.b_kstr_o] =
        newrobot->PositionNumber + 1.0;
      newrobot->PositionDoFMap[cartesian_trajectory_planner_B.b_kstr_o + 8] =
        newrobot->PositionNumber + jnt->PositionNumber;
      jnt = body_0->JointInternal;
      cartesian_trajectory_planner_B.b_kstr_o = static_cast<int32_T>
        (body_0->Index) - 1;
      newrobot->VelocityDoFMap[cartesian_trajectory_planner_B.b_kstr_o] =
        newrobot->VelocityNumber + 1.0;
      newrobot->VelocityDoFMap[cartesian_trajectory_planner_B.b_kstr_o + 8] =
        newrobot->VelocityNumber + jnt->VelocityNumber;
    } else {
      cartesian_trajectory_planner_B.b_kstr_o = static_cast<int32_T>
        (body_0->Index);
      newrobot->PositionDoFMap[cartesian_trajectory_planner_B.b_kstr_o - 1] =
        0.0;
      newrobot->PositionDoFMap[cartesian_trajectory_planner_B.b_kstr_o + 7] =
        -1.0;
      cartesian_trajectory_planner_B.b_kstr_o = static_cast<int32_T>
        (body_0->Index);
      newrobot->VelocityDoFMap[cartesian_trajectory_planner_B.b_kstr_o - 1] =
        0.0;
      newrobot->VelocityDoFMap[cartesian_trajectory_planner_B.b_kstr_o + 7] =
        -1.0;
    }

    jnt = body_0->JointInternal;
    newrobot->PositionNumber += jnt->PositionNumber;
    jnt = body_0->JointInternal;
    newrobot->VelocityNumber += jnt->VelocityNumber;
  }

  cartesian_trajec_emxFree_char_T(&bname);
  cartesian_trajec_emxFree_char_T(&b_basename);
  obj->RigidBodyTreeInternal = newrobot;
}

static h_robotics_core_internal_Damp_T *DampedBFGSwGradientProjection_D
  (h_robotics_core_internal_Damp_T *obj)
{
  h_robotics_core_internal_Damp_T *b_obj;
  int32_T i;
  static const char_T tmp[22] = { 'B', 'F', 'G', 'S', 'G', 'r', 'a', 'd', 'i',
    'e', 'n', 't', 'P', 'r', 'o', 'j', 'e', 'c', 't', 'i', 'o', 'n' };

  b_obj = obj;
  obj->MaxNumIteration = 1500.0;
  obj->MaxTime = 10.0;
  obj->GradientTolerance = 1.0E-7;
  obj->SolutionTolerance = 1.0E-6;
  obj->ArmijoRuleBeta = 0.4;
  obj->ArmijoRuleSigma = 1.0E-5;
  obj->ConstraintsOn = true;
  obj->RandomRestart = true;
  obj->StepTolerance = 1.0E-14;
  for (i = 0; i < 22; i++) {
    obj->Name[i] = tmp[i];
  }

  obj->ConstraintMatrix->size[0] = 0;
  obj->ConstraintMatrix->size[1] = 0;
  obj->ConstraintBound->size[0] = 0;
  obj->TimeObj.StartTime = -1.0;
  obj->TimeObjInternal.StartTime = -1.0;
  return b_obj;
}

static void emxInitStruct_c_rigidBodyJoint1(c_rigidBodyJoint_cartesian_tr_T
  *pStruct)
{
  cartesian_trajec_emxInit_char_T(&pStruct->Type, 2);
  cartesian_trajec_emxInit_real_T(&pStruct->MotionSubspace, 2);
}

static void emxInitStruct_o_robotics_manip_(o_robotics_manip_internal_Rig_T
  *pStruct)
{
  cartesian_trajec_emxInit_char_T(&pStruct->NameInternal, 2);
  emxInitStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static void emxInitStruct_p_robotics_manip_(p_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_o_robotics_manip_(&pStruct->Base);
}

static void emxInitStruct_robotics_slmani_a(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxInitStruct_p_robotics_manip_(&pStruct->TreeInternal);
}

static void emxInitStruct_n_robotics_manip_(n_robotics_manip_internal_Rig_T
  *pStruct)
{
  cartesian_trajec_emxInit_char_T(&pStruct->NameInternal, 2);
  emxInitStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static n_robotics_manip_internal_Rig_T *cartesian_t_RigidBody_RigidBody
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_cartesian_tra_T *switch_expression;
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
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 13; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 0.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  cartesian_trajec_emxFree_char_T(&switch_expression);
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *cartesian_RigidBody_RigidBody_a
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_cartesian_tra_T *switch_expression;
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
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 1.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  cartesian_trajec_emxFree_char_T(&switch_expression);
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *cartesia_RigidBody_RigidBody_as
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_cartesian_tra_T *switch_expression;
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
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 2.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  cartesian_trajec_emxFree_char_T(&switch_expression);
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = -1.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *cartesi_RigidBody_RigidBody_ast
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_cartesian_tra_T *switch_expression;
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
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 3.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  cartesian_trajec_emxFree_char_T(&switch_expression);
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *cartes_RigidBody_RigidBody_astw
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_cartesian_tra_T *switch_expression;
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
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 4.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  cartesian_trajec_emxFree_char_T(&switch_expression);
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *carte_RigidBody_RigidBody_astwh
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_cartesian_tra_T *switch_expression;
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
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 5.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  cartesian_trajec_emxFree_char_T(&switch_expression);
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 1.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *cart_RigidBody_RigidBody_astwhq
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_cartesian_tra_T *switch_expression;
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
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 6.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  cartesian_trajec_emxFree_char_T(&switch_expression);
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static n_robotics_manip_internal_Rig_T *car_RigidBody_RigidBody_astwhqf
  (n_robotics_manip_internal_Rig_T *obj)
{
  n_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_cartesian_tra_T *switch_expression;
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
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 11; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->ParentIndex = 7.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  cartesian_trajec_emxFree_char_T(&switch_expression);
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static o_robotics_manip_internal_Rig_T *ca_RigidBody_RigidBody_astwhqf2
  (o_robotics_manip_internal_Rig_T *obj)
{
  o_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_cartesian_tra_T *switch_expression;
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
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  cartesian_trajec_emxFree_char_T(&switch_expression);
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
  cartes_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  return b_obj;
}

static p_robotics_manip_internal_Rig_T *car_RigidBodyTree_RigidBodyTree
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
  obj->Bodies[0] = cartesian_t_RigidBody_RigidBody(iobj_0);
  obj->Bodies[0]->Index = 1.0;
  obj->Bodies[1] = cartesian_RigidBody_RigidBody_a(iobj_7);
  obj->Bodies[1]->Index = 2.0;
  obj->Bodies[2] = cartesia_RigidBody_RigidBody_as(iobj_1);
  obj->Bodies[2]->Index = 3.0;
  obj->Bodies[3] = cartesi_RigidBody_RigidBody_ast(iobj_2);
  obj->Bodies[3]->Index = 4.0;
  obj->Bodies[4] = cartes_RigidBody_RigidBody_astw(iobj_3);
  obj->Bodies[4]->Index = 5.0;
  obj->Bodies[5] = carte_RigidBody_RigidBody_astwh(iobj_4);
  obj->Bodies[5]->Index = 6.0;
  obj->Bodies[6] = cart_RigidBody_RigidBody_astwhq(iobj_5);
  obj->Bodies[6]->Index = 7.0;
  obj->Bodies[7] = car_RigidBody_RigidBody_astwhqf(iobj_6);
  obj->Bodies[7]->Index = 8.0;
  obj->NumBodies = 8.0;
  obj->PositionNumber = 6.0;
  obj->VelocityNumber = 6.0;
  for (i = 0; i < 16; i++) {
    obj->PositionDoFMap[i] = tmp[i];
  }

  ca_RigidBody_RigidBody_astwhqf2(&obj->Base);
  return b_obj;
}

static void emxInitStruct_c_rigidBodyJoint2(c_rigidBodyJoint_cartesian__a_T
  *pStruct)
{
  cartesian_trajec_emxInit_char_T(&pStruct->Type, 2);
}

static void emxInitStruct_o_robotics_mani_a(o_robotics_manip_internal_R_a_T
  *pStruct)
{
  cartesian_trajec_emxInit_char_T(&pStruct->NameInternal, 2);
  emxInitStruct_c_rigidBodyJoint2(&pStruct->JointInternal);
}

static void emxInitStruct_p_robotics_mani_a(p_robotics_manip_internal_R_a_T
  *pStruct)
{
  emxInitStruct_o_robotics_mani_a(&pStruct->Base);
}

static void emxInitStruct_robotics_slman_as(robotics_slmanip_internal_b_a_T
  *pStruct)
{
  emxInitStruct_p_robotics_mani_a(&pStruct->TreeInternal);
}

static void emxInitStruct_n_robotics_mani_a(n_robotics_manip_internal_R_a_T
  *pStruct)
{
  cartesian_trajec_emxInit_char_T(&pStruct->NameInternal, 2);
  emxInitStruct_c_rigidBodyJoint2(&pStruct->JointInternal);
}

static n_robotics_manip_internal_R_a_T *c_RigidBody_RigidBody_astwhqf2a
  (n_robotics_manip_internal_R_a_T *obj)
{
  n_robotics_manip_internal_R_a_T *b_obj;
  emxArray_char_T_cartesian_tra_T *switch_expression;
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
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 13; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 0.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  cartesian_trajec_emxFree_char_T(&switch_expression);
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

static n_robotics_manip_internal_R_a_T *RigidBody_RigidBody_astwhqf2az
  (n_robotics_manip_internal_R_a_T *obj)
{
  n_robotics_manip_internal_R_a_T *b_obj;
  emxArray_char_T_cartesian_tra_T *switch_expression;
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
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 1.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  cartesian_trajec_emxFree_char_T(&switch_expression);
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

static n_robotics_manip_internal_R_a_T *RigidBody_RigidBody_astwhqf2azt
  (n_robotics_manip_internal_R_a_T *obj)
{
  n_robotics_manip_internal_R_a_T *b_obj;
  emxArray_char_T_cartesian_tra_T *switch_expression;
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
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 2.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  cartesian_trajec_emxFree_char_T(&switch_expression);
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

static n_robotics_manip_internal_R_a_T *RigidBody_RigidBod_astwhqf2aztt
  (n_robotics_manip_internal_R_a_T *obj)
{
  n_robotics_manip_internal_R_a_T *b_obj;
  emxArray_char_T_cartesian_tra_T *switch_expression;
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
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 3.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  cartesian_trajec_emxFree_char_T(&switch_expression);
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

static n_robotics_manip_internal_R_a_T *RigidBody_RigidBo_astwhqf2azttx
  (n_robotics_manip_internal_R_a_T *obj)
{
  n_robotics_manip_internal_R_a_T *b_obj;
  emxArray_char_T_cartesian_tra_T *switch_expression;
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
  cartes_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 4.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  cartes_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  cartesian_trajec_emxFree_char_T(&switch_expression);
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

static p_robotics_manip_internal_R_a_T *c_RigidBodyTree_RigidBodyTree_a
  (p_robotics_manip_internal_R_a_T *obj, n_robotics_manip_internal_R_a_T *iobj_0,
   n_robotics_manip_internal_R_a_T *iobj_1, n_robotics_manip_internal_R_a_T
   *iobj_2, n_robotics_manip_internal_R_a_T *iobj_3,
   n_robotics_manip_internal_R_a_T *iobj_4, n_robotics_manip_internal_R_a_T
   *iobj_5, n_robotics_manip_internal_R_a_T *iobj_6,
   n_robotics_manip_internal_R_a_T *iobj_7)
{
  p_robotics_manip_internal_R_a_T *b_obj;
  o_robotics_manip_internal_R_a_T *obj_0;
  emxArray_char_T_cartesian_tra_T *switch_expression;
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
  obj->Bodies[0] = c_RigidBody_RigidBody_astwhqf2a(iobj_0);
  obj->Bodies[1] = RigidBody_RigidBody_astwhqf2az(iobj_7);
  obj->Bodies[2] = RigidBody_RigidBody_astwhqf2azt(iobj_1);
  obj->Bodies[3] = RigidBody_RigidBod_astwhqf2aztt(iobj_2);
  obj->Bodies[4] = RigidBody_RigidBo_astwhqf2azttx(iobj_3);
  b_kstr = iobj_4->NameInternal->size[0] * iobj_4->NameInternal->size[1];
  iobj_4->NameInternal->size[0] = 1;
  iobj_4->NameInternal->size[1] = 10;
  cartes_emxEnsureCapacity_char_T(iobj_4->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    iobj_4->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  iobj_4->ParentIndex = 5.0;
  b_kstr = iobj_4->JointInternal.Type->size[0] * iobj_4->
    JointInternal.Type->size[1];
  iobj_4->JointInternal.Type->size[0] = 1;
  iobj_4->JointInternal.Type->size[1] = 8;
  cartes_emxEnsureCapacity_char_T(iobj_4->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    iobj_4->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  cartesian_trajec_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_4->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
  cartes_emxEnsureCapacity_char_T(iobj_5->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    iobj_5->NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  iobj_5->ParentIndex = 6.0;
  b_kstr = iobj_5->JointInternal.Type->size[0] * iobj_5->
    JointInternal.Type->size[1];
  iobj_5->JointInternal.Type->size[0] = 1;
  iobj_5->JointInternal.Type->size[1] = 8;
  cartes_emxEnsureCapacity_char_T(iobj_5->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    iobj_5->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_5->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_5->JointInternal.Type->size[0] * iobj_5->
    JointInternal.Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_5->JointInternal.Type->data[b_kstr];
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
  cartes_emxEnsureCapacity_char_T(iobj_6->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 11; b_kstr++) {
    iobj_6->NameInternal->data[b_kstr] = tmp_6[b_kstr];
  }

  iobj_6->ParentIndex = 7.0;
  b_kstr = iobj_6->JointInternal.Type->size[0] * iobj_6->
    JointInternal.Type->size[1];
  iobj_6->JointInternal.Type->size[0] = 1;
  iobj_6->JointInternal.Type->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(iobj_6->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_6->JointInternal.Type->data[b_kstr] = tmp_7[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_6->JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_6->JointInternal.Type->size[0] * iobj_6->
    JointInternal.Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_6->JointInternal.Type->data[b_kstr];
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
  cartes_emxEnsureCapacity_char_T(obj->Base.NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->Base.NameInternal->data[b_kstr] = tmp_8[b_kstr];
  }

  b_kstr = obj->Base.JointInternal.Type->size[0] * obj->
    Base.JointInternal.Type->size[1];
  obj->Base.JointInternal.Type->size[0] = 1;
  obj->Base.JointInternal.Type->size[1] = 5;
  cartes_emxEnsureCapacity_char_T(obj->Base.JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj_0->JointInternal.Type->data[b_kstr] = tmp_7[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->Base.JointInternal.Type->size[1];
  cartes_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->Base.JointInternal.Type->size[0] * obj->
    Base.JointInternal.Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj_0->JointInternal.Type->data[b_kstr];
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

  cartesian_trajec_emxFree_char_T(&switch_expression);
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
void cartesian_trajectory_planner_step(void)
{
  robotics_slmanip_internal__as_T *obj;
  emxArray_real_T_cartesian_tra_T *b;
  emxArray_f_cell_wrap_cartesia_T *obj_f;

  // Outputs for Atomic SubSystem: '<Root>/Subscribe'
  // MATLABSystem: '<S9>/SourceBlock' incorporates:
  //   Inport: '<S13>/In1'

  cartesian_trajectory_planner_B.b_varargout_1 =
    Sub_cartesian_trajectory_planner_280.getLatestMessage
    (&cartesian_trajectory_planner_B.b_varargout_2);

  // Outputs for Enabled SubSystem: '<S9>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S13>/Enable'

  if (cartesian_trajectory_planner_B.b_varargout_1) {
    cartesian_trajectory_planner_B.In1 =
      cartesian_trajectory_planner_B.b_varargout_2;
  }

  // End of MATLABSystem: '<S9>/SourceBlock'
  // End of Outputs for SubSystem: '<S9>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe'

  // MATLAB Function: '<Root>/MATLAB Function1'
  cartesian_trajectory_planner_B.time =
    cartesian_trajectory_planner_B.In1.Clock_.Nsec / 1.0E+9 +
    cartesian_trajectory_planner_B.In1.Clock_.Sec;

  // MATLABSystem: '<Root>/Get Parameter'
  ParamGet_cartesian_trajectory_planner_271.get_parameter
    (&cartesian_trajectory_planner_B.rtb_signal2_idx_1);

  // MATLABSystem: '<Root>/Get Parameter1'
  ParamGet_cartesian_trajectory_planner_272.get_parameter
    (&cartesian_trajectory_planner_B.rtb_signal2_idx_2);

  // MATLABSystem: '<S14>/Get Parameter3'
  ParamGet_cartesian_trajectory_planner_291.get_parameter
    (&cartesian_trajectory_planner_B.tc);

  // MATLABSystem: '<S14>/Get Parameter4'
  ParamGet_cartesian_trajectory_planner_292.get_parameter
    (&cartesian_trajectory_planner_B.SpMax);

  // MATLABSystem: '<S14>/Get Parameter5'
  ParamGet_cartesian_trajectory_planner_293.get_parameter
    (&cartesian_trajectory_planner_B.maxval);

  // MATLABSystem: '<S14>/Get Parameter6'
  ParamGet_cartesian_trajectory_planner_294.get_parameter
    (&cartesian_trajectory_planner_B.s_i);

  // MATLABSystem: '<S14>/Get Parameter'
  ParamGet_cartesian_trajectory_planner_288.get_parameter
    (&cartesian_trajectory_planner_B.value);

  // MATLABSystem: '<S14>/Get Parameter1'
  ParamGet_cartesian_trajectory_planner_289.get_parameter
    (&cartesian_trajectory_planner_B.value_h);

  // MATLABSystem: '<S14>/Get Parameter2'
  ParamGet_cartesian_trajectory_planner_290.get_parameter
    (&cartesian_trajectory_planner_B.value_n);

  // MATLABSystem: '<Root>/Coordinate Transformation Conversion' incorporates:
  //   MATLABSystem: '<S14>/Get Parameter'
  //   MATLABSystem: '<S14>/Get Parameter1'
  //   MATLABSystem: '<S14>/Get Parameter2'
  //   MATLABSystem: '<S14>/Get Parameter3'
  //   MATLABSystem: '<S14>/Get Parameter4'
  //   MATLABSystem: '<S14>/Get Parameter5'
  //   MATLABSystem: '<S14>/Get Parameter6'

  cartesian_trajectory_planner_B.s_vel = 1.0 / sqrt
    (((cartesian_trajectory_planner_B.tc * cartesian_trajectory_planner_B.tc +
       cartesian_trajectory_planner_B.SpMax *
       cartesian_trajectory_planner_B.SpMax) +
      cartesian_trajectory_planner_B.maxval *
      cartesian_trajectory_planner_B.maxval) +
     cartesian_trajectory_planner_B.s_i * cartesian_trajectory_planner_B.s_i);
  cartesian_trajectory_planner_B.tc *= cartesian_trajectory_planner_B.s_vel;
  cartesian_trajectory_planner_B.SpMax *= cartesian_trajectory_planner_B.s_vel;
  cartesian_trajectory_planner_B.maxval *= cartesian_trajectory_planner_B.s_vel;
  cartesian_trajectory_planner_B.tft = cartesian_trajectory_planner_B.s_i *
    cartesian_trajectory_planner_B.s_vel;
  cartesian_trajectory_planner_B.s_vel = cartesian_trajectory_planner_B.tft *
    cartesian_trajectory_planner_B.tft;
  cartesian_trajectory_planner_B.s_i = cartesian_trajectory_planner_B.maxval *
    cartesian_trajectory_planner_B.maxval;
  cartesian_trajectory_planner_B.tempR[0] = 1.0 -
    (cartesian_trajectory_planner_B.s_i + cartesian_trajectory_planner_B.s_vel) *
    2.0;
  cartesian_trajectory_planner_B.tcstar = cartesian_trajectory_planner_B.SpMax *
    cartesian_trajectory_planner_B.maxval;
  cartesian_trajectory_planner_B.tetaFIN = cartesian_trajectory_planner_B.tc *
    cartesian_trajectory_planner_B.tft;
  cartesian_trajectory_planner_B.tempR[1] =
    (cartesian_trajectory_planner_B.tcstar -
     cartesian_trajectory_planner_B.tetaFIN) * 2.0;
  cartesian_trajectory_planner_B.teta_vel = cartesian_trajectory_planner_B.SpMax
    * cartesian_trajectory_planner_B.tft;
  cartesian_trajectory_planner_B.tf = cartesian_trajectory_planner_B.tc *
    cartesian_trajectory_planner_B.maxval;
  cartesian_trajectory_planner_B.tempR[2] =
    (cartesian_trajectory_planner_B.teta_vel + cartesian_trajectory_planner_B.tf)
    * 2.0;
  cartesian_trajectory_planner_B.tempR[3] =
    (cartesian_trajectory_planner_B.tcstar +
     cartesian_trajectory_planner_B.tetaFIN) * 2.0;
  cartesian_trajectory_planner_B.tcstar = cartesian_trajectory_planner_B.SpMax *
    cartesian_trajectory_planner_B.SpMax;
  cartesian_trajectory_planner_B.tempR[4] = 1.0 -
    (cartesian_trajectory_planner_B.tcstar +
     cartesian_trajectory_planner_B.s_vel) * 2.0;
  cartesian_trajectory_planner_B.s_vel = cartesian_trajectory_planner_B.maxval *
    cartesian_trajectory_planner_B.tft;
  cartesian_trajectory_planner_B.tetaFIN = cartesian_trajectory_planner_B.tc *
    cartesian_trajectory_planner_B.SpMax;
  cartesian_trajectory_planner_B.tempR[5] =
    (cartesian_trajectory_planner_B.s_vel -
     cartesian_trajectory_planner_B.tetaFIN) * 2.0;
  cartesian_trajectory_planner_B.tempR[6] =
    (cartesian_trajectory_planner_B.teta_vel - cartesian_trajectory_planner_B.tf)
    * 2.0;
  cartesian_trajectory_planner_B.tempR[7] =
    (cartesian_trajectory_planner_B.s_vel +
     cartesian_trajectory_planner_B.tetaFIN) * 2.0;
  cartesian_trajectory_planner_B.tempR[8] = 1.0 -
    (cartesian_trajectory_planner_B.tcstar + cartesian_trajectory_planner_B.s_i)
    * 2.0;
  for (cartesian_trajectory_planner_B.r1 = 0; cartesian_trajectory_planner_B.r1 <
       3; cartesian_trajectory_planner_B.r1++) {
    cartesian_trajectory_planner_B.r2 = cartesian_trajectory_planner_B.r1 + 1;
    cartesian_trajectory_planner_B.R[cartesian_trajectory_planner_B.r2 - 1] =
      cartesian_trajectory_planner_B.tempR[(cartesian_trajectory_planner_B.r2 -
      1) * 3];
    cartesian_trajectory_planner_B.r2 = cartesian_trajectory_planner_B.r1 + 1;
    cartesian_trajectory_planner_B.R[cartesian_trajectory_planner_B.r2 + 2] =
      cartesian_trajectory_planner_B.tempR[(cartesian_trajectory_planner_B.r2 -
      1) * 3 + 1];
    cartesian_trajectory_planner_B.r2 = cartesian_trajectory_planner_B.r1 + 1;
    cartesian_trajectory_planner_B.R[cartesian_trajectory_planner_B.r2 + 5] =
      cartesian_trajectory_planner_B.tempR[(cartesian_trajectory_planner_B.r2 -
      1) * 3 + 2];
  }

  memset(&cartesian_trajectory_planner_B.out[0], 0, sizeof(real_T) << 4U);
  for (cartesian_trajectory_planner_B.r1 = 0; cartesian_trajectory_planner_B.r1 <
       3; cartesian_trajectory_planner_B.r1++) {
    cartesian_trajectory_planner_B.r2 = cartesian_trajectory_planner_B.r1 << 2;
    cartesian_trajectory_planner_B.out[cartesian_trajectory_planner_B.r2] =
      cartesian_trajectory_planner_B.R[3 * cartesian_trajectory_planner_B.r1];
    cartesian_trajectory_planner_B.out[cartesian_trajectory_planner_B.r2 + 1] =
      cartesian_trajectory_planner_B.R[3 * cartesian_trajectory_planner_B.r1 + 1];
    cartesian_trajectory_planner_B.out[cartesian_trajectory_planner_B.r2 + 2] =
      cartesian_trajectory_planner_B.R[3 * cartesian_trajectory_planner_B.r1 + 2];
  }

  cartesian_trajectory_planner_B.out[15] = 1.0;
  cartesian_trajectory_planner_B.out[12] = cartesian_trajectory_planner_B.value;
  cartesian_trajectory_planner_B.out[13] =
    cartesian_trajectory_planner_B.value_h;
  cartesian_trajectory_planner_B.out[14] =
    cartesian_trajectory_planner_B.value_n;

  // MATLABSystem: '<S15>/Get Parameter3'
  ParamGet_cartesian_trajectory_planner_303.get_parameter
    (&cartesian_trajectory_planner_B.tc);

  // MATLABSystem: '<S15>/Get Parameter4'
  ParamGet_cartesian_trajectory_planner_304.get_parameter
    (&cartesian_trajectory_planner_B.SpMax);

  // MATLABSystem: '<S15>/Get Parameter5'
  ParamGet_cartesian_trajectory_planner_305.get_parameter
    (&cartesian_trajectory_planner_B.maxval);

  // MATLABSystem: '<S15>/Get Parameter6'
  ParamGet_cartesian_trajectory_planner_306.get_parameter
    (&cartesian_trajectory_planner_B.s_i);

  // MATLABSystem: '<S15>/Get Parameter'
  ParamGet_cartesian_trajectory_planner_300.get_parameter
    (&cartesian_trajectory_planner_B.value_o);

  // MATLABSystem: '<S15>/Get Parameter1'
  ParamGet_cartesian_trajectory_planner_301.get_parameter
    (&cartesian_trajectory_planner_B.value_c);

  // MATLABSystem: '<S15>/Get Parameter2'
  ParamGet_cartesian_trajectory_planner_302.get_parameter
    (&cartesian_trajectory_planner_B.value_b);

  // MATLABSystem: '<Root>/Coordinate Transformation Conversion1' incorporates:
  //   MATLABSystem: '<S15>/Get Parameter'
  //   MATLABSystem: '<S15>/Get Parameter1'
  //   MATLABSystem: '<S15>/Get Parameter2'
  //   MATLABSystem: '<S15>/Get Parameter3'
  //   MATLABSystem: '<S15>/Get Parameter4'
  //   MATLABSystem: '<S15>/Get Parameter5'
  //   MATLABSystem: '<S15>/Get Parameter6'

  cartesian_trajectory_planner_B.s_vel = 1.0 / sqrt
    (((cartesian_trajectory_planner_B.tc * cartesian_trajectory_planner_B.tc +
       cartesian_trajectory_planner_B.SpMax *
       cartesian_trajectory_planner_B.SpMax) +
      cartesian_trajectory_planner_B.maxval *
      cartesian_trajectory_planner_B.maxval) +
     cartesian_trajectory_planner_B.s_i * cartesian_trajectory_planner_B.s_i);
  cartesian_trajectory_planner_B.tc *= cartesian_trajectory_planner_B.s_vel;
  cartesian_trajectory_planner_B.SpMax *= cartesian_trajectory_planner_B.s_vel;
  cartesian_trajectory_planner_B.maxval *= cartesian_trajectory_planner_B.s_vel;
  cartesian_trajectory_planner_B.tft = cartesian_trajectory_planner_B.s_i *
    cartesian_trajectory_planner_B.s_vel;
  cartesian_trajectory_planner_B.s_vel = cartesian_trajectory_planner_B.tft *
    cartesian_trajectory_planner_B.tft;
  cartesian_trajectory_planner_B.s_i = cartesian_trajectory_planner_B.maxval *
    cartesian_trajectory_planner_B.maxval;
  cartesian_trajectory_planner_B.tempR[0] = 1.0 -
    (cartesian_trajectory_planner_B.s_i + cartesian_trajectory_planner_B.s_vel) *
    2.0;
  cartesian_trajectory_planner_B.tcstar = cartesian_trajectory_planner_B.SpMax *
    cartesian_trajectory_planner_B.maxval;
  cartesian_trajectory_planner_B.tetaFIN = cartesian_trajectory_planner_B.tc *
    cartesian_trajectory_planner_B.tft;
  cartesian_trajectory_planner_B.tempR[1] =
    (cartesian_trajectory_planner_B.tcstar -
     cartesian_trajectory_planner_B.tetaFIN) * 2.0;
  cartesian_trajectory_planner_B.teta_vel = cartesian_trajectory_planner_B.SpMax
    * cartesian_trajectory_planner_B.tft;
  cartesian_trajectory_planner_B.tf = cartesian_trajectory_planner_B.tc *
    cartesian_trajectory_planner_B.maxval;
  cartesian_trajectory_planner_B.tempR[2] =
    (cartesian_trajectory_planner_B.teta_vel + cartesian_trajectory_planner_B.tf)
    * 2.0;
  cartesian_trajectory_planner_B.tempR[3] =
    (cartesian_trajectory_planner_B.tcstar +
     cartesian_trajectory_planner_B.tetaFIN) * 2.0;
  cartesian_trajectory_planner_B.tcstar = cartesian_trajectory_planner_B.SpMax *
    cartesian_trajectory_planner_B.SpMax;
  cartesian_trajectory_planner_B.tempR[4] = 1.0 -
    (cartesian_trajectory_planner_B.tcstar +
     cartesian_trajectory_planner_B.s_vel) * 2.0;
  cartesian_trajectory_planner_B.s_vel = cartesian_trajectory_planner_B.maxval *
    cartesian_trajectory_planner_B.tft;
  cartesian_trajectory_planner_B.tetaFIN = cartesian_trajectory_planner_B.tc *
    cartesian_trajectory_planner_B.SpMax;
  cartesian_trajectory_planner_B.tempR[5] =
    (cartesian_trajectory_planner_B.s_vel -
     cartesian_trajectory_planner_B.tetaFIN) * 2.0;
  cartesian_trajectory_planner_B.tempR[6] =
    (cartesian_trajectory_planner_B.teta_vel - cartesian_trajectory_planner_B.tf)
    * 2.0;
  cartesian_trajectory_planner_B.tempR[7] =
    (cartesian_trajectory_planner_B.s_vel +
     cartesian_trajectory_planner_B.tetaFIN) * 2.0;
  cartesian_trajectory_planner_B.tempR[8] = 1.0 -
    (cartesian_trajectory_planner_B.tcstar + cartesian_trajectory_planner_B.s_i)
    * 2.0;
  for (cartesian_trajectory_planner_B.r1 = 0; cartesian_trajectory_planner_B.r1 <
       3; cartesian_trajectory_planner_B.r1++) {
    cartesian_trajectory_planner_B.r2 = cartesian_trajectory_planner_B.r1 + 1;
    cartesian_trajectory_planner_B.R[cartesian_trajectory_planner_B.r2 - 1] =
      cartesian_trajectory_planner_B.tempR[(cartesian_trajectory_planner_B.r2 -
      1) * 3];
    cartesian_trajectory_planner_B.r2 = cartesian_trajectory_planner_B.r1 + 1;
    cartesian_trajectory_planner_B.R[cartesian_trajectory_planner_B.r2 + 2] =
      cartesian_trajectory_planner_B.tempR[(cartesian_trajectory_planner_B.r2 -
      1) * 3 + 1];
    cartesian_trajectory_planner_B.r2 = cartesian_trajectory_planner_B.r1 + 1;
    cartesian_trajectory_planner_B.R[cartesian_trajectory_planner_B.r2 + 5] =
      cartesian_trajectory_planner_B.tempR[(cartesian_trajectory_planner_B.r2 -
      1) * 3 + 2];
  }

  memset(&cartesian_trajectory_planner_B.out_j[0], 0, sizeof(real_T) << 4U);
  for (cartesian_trajectory_planner_B.r1 = 0; cartesian_trajectory_planner_B.r1 <
       3; cartesian_trajectory_planner_B.r1++) {
    cartesian_trajectory_planner_B.r2 = cartesian_trajectory_planner_B.r1 << 2;
    cartesian_trajectory_planner_B.out_j[cartesian_trajectory_planner_B.r2] =
      cartesian_trajectory_planner_B.R[3 * cartesian_trajectory_planner_B.r1];
    cartesian_trajectory_planner_B.out_j[cartesian_trajectory_planner_B.r2 + 1] =
      cartesian_trajectory_planner_B.R[3 * cartesian_trajectory_planner_B.r1 + 1];
    cartesian_trajectory_planner_B.out_j[cartesian_trajectory_planner_B.r2 + 2] =
      cartesian_trajectory_planner_B.R[3 * cartesian_trajectory_planner_B.r1 + 2];
  }

  cartesian_trajectory_planner_B.out_j[15] = 1.0;
  cartesian_trajectory_planner_B.out_j[12] =
    cartesian_trajectory_planner_B.value_o;
  cartesian_trajectory_planner_B.out_j[13] =
    cartesian_trajectory_planner_B.value_c;
  cartesian_trajectory_planner_B.out_j[14] =
    cartesian_trajectory_planner_B.value_b;

  // MATLAB Function: '<Root>/Traslazione ' incorporates:
  //   MATLABSystem: '<Root>/Get Parameter'
  //   MATLABSystem: '<Root>/Get Parameter1'

  cartesian_trajectory_planner_B.tc =
    cartesian_trajectory_planner_B.rtb_signal2_idx_1 /
    cartesian_trajectory_planner_B.rtb_signal2_idx_2;
  cartesian_trajectory_planner_B.SpMax = cartesian_trajectory_planner_B.tc *
    cartesian_trajectory_planner_B.rtb_signal2_idx_1;

  // MATLAB Function: '<Root>/Completa' incorporates:
  //   MATLAB Function: '<Root>/Traslazione '

  memset(&cartesian_trajectory_planner_B.POSA[0], 0, sizeof(real_T) << 4U);

  // MATLAB Function: '<Root>/Traslazione ' incorporates:
  //   MATLAB Function: '<Root>/Completa'
  //   MATLABSystem: '<Root>/Coordinate Transformation Conversion'
  //   MATLABSystem: '<Root>/Coordinate Transformation Conversion1'
  //   MATLABSystem: '<Root>/Get Parameter'
  //   MATLABSystem: '<Root>/Get Parameter1'
  //   MATLABSystem: '<S14>/Get Parameter'
  //   MATLABSystem: '<S14>/Get Parameter1'
  //   MATLABSystem: '<S14>/Get Parameter2'
  //   MATLABSystem: '<S15>/Get Parameter'
  //   MATLABSystem: '<S15>/Get Parameter1'
  //   MATLABSystem: '<S15>/Get Parameter2'

  cartesian_trajectory_planner_B.s_i = 0.0;
  cartesian_trajectory_planner_B.s_vel = 0.0;
  cartesian_trajectory_planner_B.tetaFIN =
    cartesian_trajectory_planner_B.value_o -
    cartesian_trajectory_planner_B.value;
  cartesian_trajectory_planner_B.out_m[0] =
    cartesian_trajectory_planner_B.tetaFIN;
  cartesian_trajectory_planner_B.tft = cartesian_trajectory_planner_B.value_c -
    cartesian_trajectory_planner_B.value_h;
  cartesian_trajectory_planner_B.out_m[1] = cartesian_trajectory_planner_B.tft;
  cartesian_trajectory_planner_B.teta_vel =
    cartesian_trajectory_planner_B.value_b -
    cartesian_trajectory_planner_B.value_n;
  cartesian_trajectory_planner_B.out_m[2] =
    cartesian_trajectory_planner_B.teta_vel;
  cartesian_trajectory_planner_B.maxval = cartesian_trajectory_pla_norm_l
    (cartesian_trajectory_planner_B.out_m);
  if (cartesian_trajectory_planner_B.SpMax >=
      cartesian_trajectory_planner_B.maxval) {
    cartesian_trajectory_planner_B.tcstar = sqrt
      (cartesian_trajectory_planner_B.maxval /
       cartesian_trajectory_planner_B.rtb_signal2_idx_2);
    cartesian_trajectory_planner_B.tf = 2.0 *
      cartesian_trajectory_planner_B.tcstar;
    if ((cartesian_trajectory_planner_B.time >= 0.0) &&
        (cartesian_trajectory_planner_B.time <
         cartesian_trajectory_planner_B.tcstar)) {
      cartesian_trajectory_planner_B.s_i = 0.5 *
        cartesian_trajectory_planner_B.rtb_signal2_idx_2 *
        cartesian_trajectory_planner_B.time *
        cartesian_trajectory_planner_B.time;
      cartesian_trajectory_planner_B.s_vel =
        cartesian_trajectory_planner_B.rtb_signal2_idx_2 *
        cartesian_trajectory_planner_B.time;
    }

    if ((cartesian_trajectory_planner_B.time >=
         cartesian_trajectory_planner_B.tcstar) &&
        (cartesian_trajectory_planner_B.time <=
         cartesian_trajectory_planner_B.tf)) {
      cartesian_trajectory_planner_B.s_vel = cartesian_trajectory_planner_B.tf -
        cartesian_trajectory_planner_B.time;
      cartesian_trajectory_planner_B.s_i = cartesian_trajectory_planner_B.maxval
        - 0.5 * cartesian_trajectory_planner_B.rtb_signal2_idx_2 *
        cartesian_trajectory_planner_B.s_vel *
        cartesian_trajectory_planner_B.s_vel;
      cartesian_trajectory_planner_B.s_vel *=
        cartesian_trajectory_planner_B.rtb_signal2_idx_2;
    }

    if (cartesian_trajectory_planner_B.time > cartesian_trajectory_planner_B.tf)
    {
      cartesian_trajectory_planner_B.s_i = cartesian_trajectory_planner_B.maxval;
      cartesian_trajectory_planner_B.s_vel = 0.0;
    }
  }

  if (cartesian_trajectory_planner_B.SpMax <
      cartesian_trajectory_planner_B.maxval) {
    cartesian_trajectory_planner_B.tcstar =
      cartesian_trajectory_planner_B.rtb_signal2_idx_2 *
      cartesian_trajectory_planner_B.tc;
    cartesian_trajectory_planner_B.tf = (cartesian_trajectory_planner_B.maxval -
      cartesian_trajectory_planner_B.tcstar * cartesian_trajectory_planner_B.tc)
      * (1.0 / cartesian_trajectory_planner_B.rtb_signal2_idx_1) + 2.0 *
      cartesian_trajectory_planner_B.tc;
    if ((cartesian_trajectory_planner_B.time >= 0.0) &&
        (cartesian_trajectory_planner_B.time <=
         cartesian_trajectory_planner_B.tc)) {
      cartesian_trajectory_planner_B.s_i = 0.5 *
        cartesian_trajectory_planner_B.rtb_signal2_idx_2 *
        cartesian_trajectory_planner_B.time *
        cartesian_trajectory_planner_B.time;
      cartesian_trajectory_planner_B.s_vel =
        cartesian_trajectory_planner_B.rtb_signal2_idx_2 *
        cartesian_trajectory_planner_B.time;
    }

    cartesian_trajectory_planner_B.d = cartesian_trajectory_planner_B.tf -
      cartesian_trajectory_planner_B.tc;
    if ((cartesian_trajectory_planner_B.time > cartesian_trajectory_planner_B.tc)
        && (cartesian_trajectory_planner_B.time <=
            cartesian_trajectory_planner_B.d)) {
      cartesian_trajectory_planner_B.s_i = (cartesian_trajectory_planner_B.time
        - cartesian_trajectory_planner_B.tc / 2.0) *
        cartesian_trajectory_planner_B.tcstar;
      cartesian_trajectory_planner_B.s_vel =
        cartesian_trajectory_planner_B.rtb_signal2_idx_1;
    }

    if ((cartesian_trajectory_planner_B.time > cartesian_trajectory_planner_B.d)
        && (cartesian_trajectory_planner_B.time <=
            cartesian_trajectory_planner_B.tf)) {
      cartesian_trajectory_planner_B.out_m[0] =
        cartesian_trajectory_planner_B.tetaFIN;
      cartesian_trajectory_planner_B.out_m[1] =
        cartesian_trajectory_planner_B.tft;
      cartesian_trajectory_planner_B.out_m[2] =
        cartesian_trajectory_planner_B.teta_vel;
      cartesian_trajectory_planner_B.s_i = cartesian_trajectory_pla_norm_l
        (cartesian_trajectory_planner_B.out_m) - 0.5 *
        cartesian_trajectory_planner_B.rtb_signal2_idx_2 *
        (cartesian_trajectory_planner_B.tf - cartesian_trajectory_planner_B.time)
        * (cartesian_trajectory_planner_B.tf -
           cartesian_trajectory_planner_B.time);
      cartesian_trajectory_planner_B.s_vel = (cartesian_trajectory_planner_B.tf
        - cartesian_trajectory_planner_B.time) *
        cartesian_trajectory_planner_B.rtb_signal2_idx_2;
    }

    if (cartesian_trajectory_planner_B.time > cartesian_trajectory_planner_B.tf)
    {
      cartesian_trajectory_planner_B.s_i = cartesian_trajectory_planner_B.maxval;
      cartesian_trajectory_planner_B.s_vel = 0.0;
    }
  }

  cartesian_trajectory_planner_B.P_e[0] = cartesian_trajectory_planner_B.s_i /
    cartesian_trajectory_planner_B.maxval *
    cartesian_trajectory_planner_B.tetaFIN +
    cartesian_trajectory_planner_B.value;
  cartesian_trajectory_planner_B.out_m[0] =
    cartesian_trajectory_planner_B.tetaFIN;
  cartesian_trajectory_planner_B.out_m0[0] =
    cartesian_trajectory_planner_B.tetaFIN;
  cartesian_trajectory_planner_B.out_m[1] = cartesian_trajectory_planner_B.tft;
  cartesian_trajectory_planner_B.out_m0[1] = cartesian_trajectory_planner_B.tft;
  cartesian_trajectory_planner_B.out_m[2] =
    cartesian_trajectory_planner_B.teta_vel;
  cartesian_trajectory_planner_B.out_m0[2] =
    cartesian_trajectory_planner_B.teta_vel;
  cartesian_trajectory_planner_B.P_e[1] = cartesian_trajectory_planner_B.s_i /
    cartesian_trajectory_pla_norm_l(cartesian_trajectory_planner_B.out_m) *
    cartesian_trajectory_planner_B.tft + cartesian_trajectory_planner_B.value_h;
  cartesian_trajectory_planner_B.P_e[2] = cartesian_trajectory_planner_B.s_i /
    cartesian_trajectory_pla_norm_l(cartesian_trajectory_planner_B.out_m0) *
    cartesian_trajectory_planner_B.teta_vel +
    cartesian_trajectory_planner_B.value_n;
  cartesian_trajectory_planner_B.POSA[15] = 1.0;
  for (cartesian_trajectory_planner_B.r1 = 0; cartesian_trajectory_planner_B.r1 <
       3; cartesian_trajectory_planner_B.r1++) {
    cartesian_trajectory_planner_B.POSA[cartesian_trajectory_planner_B.r1 + 12] =
      cartesian_trajectory_planner_B.P_e[cartesian_trajectory_planner_B.r1];
    cartesian_trajectory_planner_B.r2 = cartesian_trajectory_planner_B.r1 << 2;
    cartesian_trajectory_planner_B.tempR[3 * cartesian_trajectory_planner_B.r1] =
      cartesian_trajectory_planner_B.out_j[cartesian_trajectory_planner_B.r2];
    cartesian_trajectory_planner_B.tempR[3 * cartesian_trajectory_planner_B.r1 +
      1] =
      cartesian_trajectory_planner_B.out_j[cartesian_trajectory_planner_B.r2 + 1];
    cartesian_trajectory_planner_B.tempR[3 * cartesian_trajectory_planner_B.r1 +
      2] =
      cartesian_trajectory_planner_B.out_j[cartesian_trajectory_planner_B.r2 + 2];
  }

  cartesian_trajectory_planner_B.r1 = 0;
  cartesian_trajectory_planner_B.r2 = 1;
  cartesian_trajectory_planner_B.r3 = 2;
  cartesian_trajectory_planner_B.maxval = fabs
    (cartesian_trajectory_planner_B.out_j[0]);
  cartesian_trajectory_planner_B.s_i = fabs
    (cartesian_trajectory_planner_B.out_j[1]);
  if (cartesian_trajectory_planner_B.s_i > cartesian_trajectory_planner_B.maxval)
  {
    cartesian_trajectory_planner_B.maxval = cartesian_trajectory_planner_B.s_i;
    cartesian_trajectory_planner_B.r1 = 1;
    cartesian_trajectory_planner_B.r2 = 0;
  }

  if (fabs(cartesian_trajectory_planner_B.out_j[2]) >
      cartesian_trajectory_planner_B.maxval) {
    cartesian_trajectory_planner_B.r1 = 2;
    cartesian_trajectory_planner_B.r2 = 1;
    cartesian_trajectory_planner_B.r3 = 0;
  }

  cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r2] =
    cartesian_trajectory_planner_B.out_j[cartesian_trajectory_planner_B.r2] /
    cartesian_trajectory_planner_B.out_j[cartesian_trajectory_planner_B.r1];
  cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r3] /=
    cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r1];
  cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r2 + 3] -=
    cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r1 + 3] *
    cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r2];
  cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r3 + 3] -=
    cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r1 + 3] *
    cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r3];
  cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r2 + 6] -=
    cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r1 + 6] *
    cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r2];
  cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r3 + 6] -=
    cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r1 + 6] *
    cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r3];
  if (fabs
      (cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r3 +
       3]) > fabs
      (cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r2 +
       3])) {
    cartesian_trajectory_planner_B.rtemp = cartesian_trajectory_planner_B.r2;
    cartesian_trajectory_planner_B.r2 = cartesian_trajectory_planner_B.r3;
    cartesian_trajectory_planner_B.r3 = cartesian_trajectory_planner_B.rtemp;
  }

  cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r3 + 3] /=
    cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r2 + 3];
  cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r3 + 6] -=
    cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r3 + 3] *
    cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r2 + 6];
  cartesian_trajectory_planner_B.Matrot[3 * cartesian_trajectory_planner_B.r1] =
    cartesian_trajectory_planner_B.out[0] /
    cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r1];
  cartesian_trajectory_planner_B.maxval =
    cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r1 + 3];
  cartesian_trajectory_planner_B.Matrot[3 * cartesian_trajectory_planner_B.r2] =
    cartesian_trajectory_planner_B.out[4] -
    cartesian_trajectory_planner_B.Matrot[3 * cartesian_trajectory_planner_B.r1]
    * cartesian_trajectory_planner_B.maxval;
  cartesian_trajectory_planner_B.s_i =
    cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r1 + 6];
  cartesian_trajectory_planner_B.Matrot[3 * cartesian_trajectory_planner_B.r3] =
    cartesian_trajectory_planner_B.out[8] -
    cartesian_trajectory_planner_B.Matrot[3 * cartesian_trajectory_planner_B.r1]
    * cartesian_trajectory_planner_B.s_i;
  cartesian_trajectory_planner_B.tcstar =
    cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r2 + 3];
  cartesian_trajectory_planner_B.Matrot[3 * cartesian_trajectory_planner_B.r2] /=
    cartesian_trajectory_planner_B.tcstar;
  cartesian_trajectory_planner_B.tetaFIN =
    cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r2 + 6];
  cartesian_trajectory_planner_B.Matrot[3 * cartesian_trajectory_planner_B.r3] -=
    cartesian_trajectory_planner_B.Matrot[3 * cartesian_trajectory_planner_B.r2]
    * cartesian_trajectory_planner_B.tetaFIN;
  cartesian_trajectory_planner_B.tft =
    cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r3 + 6];
  cartesian_trajectory_planner_B.Matrot[3 * cartesian_trajectory_planner_B.r3] /=
    cartesian_trajectory_planner_B.tft;
  cartesian_trajectory_planner_B.teta_vel =
    cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r3 + 3];
  cartesian_trajectory_planner_B.Matrot[3 * cartesian_trajectory_planner_B.r2] -=
    cartesian_trajectory_planner_B.Matrot[3 * cartesian_trajectory_planner_B.r3]
    * cartesian_trajectory_planner_B.teta_vel;
  cartesian_trajectory_planner_B.Matrot[3 * cartesian_trajectory_planner_B.r1] -=
    cartesian_trajectory_planner_B.Matrot[3 * cartesian_trajectory_planner_B.r3]
    * cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r3];
  cartesian_trajectory_planner_B.Matrot[3 * cartesian_trajectory_planner_B.r1] -=
    cartesian_trajectory_planner_B.Matrot[3 * cartesian_trajectory_planner_B.r2]
    * cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r2];
  cartesian_trajectory_planner_B.rtemp = 3 * cartesian_trajectory_planner_B.r1 +
    1;
  cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.rtemp] =
    cartesian_trajectory_planner_B.out[1] /
    cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r1];
  cartesian_trajectory_planner_B.Matrot_tmp = 3 *
    cartesian_trajectory_planner_B.r2 + 1;
  cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.Matrot_tmp]
    = cartesian_trajectory_planner_B.out[5] -
    cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.rtemp] *
    cartesian_trajectory_planner_B.maxval;
  cartesian_trajectory_planner_B.Matrot_tmp_f = 3 *
    cartesian_trajectory_planner_B.r3 + 1;
  cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.Matrot_tmp_f]
    = cartesian_trajectory_planner_B.out[9] -
    cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.rtemp] *
    cartesian_trajectory_planner_B.s_i;
  cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.Matrot_tmp]
    /= cartesian_trajectory_planner_B.tcstar;
  cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.Matrot_tmp_f]
    -=
    cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.Matrot_tmp]
    * cartesian_trajectory_planner_B.tetaFIN;
  cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.Matrot_tmp_f]
    /= cartesian_trajectory_planner_B.tft;
  cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.Matrot_tmp]
    -=
    cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.Matrot_tmp_f]
    * cartesian_trajectory_planner_B.teta_vel;
  cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.rtemp] -=
    cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.Matrot_tmp_f]
    * cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r3];
  cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.rtemp] -=
    cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.Matrot_tmp]
    * cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r2];
  cartesian_trajectory_planner_B.rtemp = 3 * cartesian_trajectory_planner_B.r1 +
    2;
  cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.rtemp] =
    cartesian_trajectory_planner_B.out[2] /
    cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r1];
  cartesian_trajectory_planner_B.Matrot_tmp = 3 *
    cartesian_trajectory_planner_B.r2 + 2;
  cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.Matrot_tmp]
    = cartesian_trajectory_planner_B.out[6] -
    cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.rtemp] *
    cartesian_trajectory_planner_B.maxval;
  cartesian_trajectory_planner_B.Matrot_tmp_f = 3 *
    cartesian_trajectory_planner_B.r3 + 2;
  cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.Matrot_tmp_f]
    = cartesian_trajectory_planner_B.out[10] -
    cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.rtemp] *
    cartesian_trajectory_planner_B.s_i;
  cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.Matrot_tmp]
    /= cartesian_trajectory_planner_B.tcstar;
  cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.Matrot_tmp_f]
    -=
    cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.Matrot_tmp]
    * cartesian_trajectory_planner_B.tetaFIN;
  cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.Matrot_tmp_f]
    /= cartesian_trajectory_planner_B.tft;
  cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.Matrot_tmp]
    -=
    cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.Matrot_tmp_f]
    * cartesian_trajectory_planner_B.teta_vel;
  cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.rtemp] -=
    cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.Matrot_tmp_f]
    * cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r3];
  cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.rtemp] -=
    cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.Matrot_tmp]
    * cartesian_trajectory_planner_B.tempR[cartesian_trajectory_planner_B.r2];
  cartesian_trajectory_planner_B.tetaFIN = acos
    ((((cartesian_trajectory_planner_B.Matrot[0] +
        cartesian_trajectory_planner_B.Matrot[4]) +
       cartesian_trajectory_planner_B.Matrot[8]) - 1.0) / 2.0);
  cartesian_trajectory_planner_B.maxval = 1.0 / (2.0 * sin
    (cartesian_trajectory_planner_B.tetaFIN));
  cartesian_trajectory_planner_B.P_e[0] =
    (cartesian_trajectory_planner_B.Matrot[5] -
     cartesian_trajectory_planner_B.Matrot[7]) *
    cartesian_trajectory_planner_B.maxval;
  cartesian_trajectory_planner_B.P_e[1] =
    (cartesian_trajectory_planner_B.Matrot[6] -
     cartesian_trajectory_planner_B.Matrot[2]) *
    cartesian_trajectory_planner_B.maxval;
  cartesian_trajectory_planner_B.P_e[2] =
    (cartesian_trajectory_planner_B.Matrot[1] -
     cartesian_trajectory_planner_B.Matrot[3]) *
    cartesian_trajectory_planner_B.maxval;
  cartesian_trajectory_planner_B.s_i = 0.0;
  cartesian_trajectory_planner_B.teta_vel = 0.0;
  if (cartesian_trajectory_planner_B.SpMax >=
      cartesian_trajectory_planner_B.tetaFIN) {
    cartesian_trajectory_planner_B.tcstar = sqrt
      (cartesian_trajectory_planner_B.tetaFIN /
       cartesian_trajectory_planner_B.rtb_signal2_idx_2);
    cartesian_trajectory_planner_B.tft = 2.0 *
      cartesian_trajectory_planner_B.tcstar;
    if ((cartesian_trajectory_planner_B.time >= 0.0) &&
        (cartesian_trajectory_planner_B.time <
         cartesian_trajectory_planner_B.tcstar)) {
      cartesian_trajectory_planner_B.s_i = 0.5 *
        cartesian_trajectory_planner_B.rtb_signal2_idx_2 *
        cartesian_trajectory_planner_B.time *
        cartesian_trajectory_planner_B.time;
      cartesian_trajectory_planner_B.teta_vel =
        cartesian_trajectory_planner_B.rtb_signal2_idx_2 *
        cartesian_trajectory_planner_B.time;
    }

    if ((cartesian_trajectory_planner_B.time >=
         cartesian_trajectory_planner_B.tcstar) &&
        (cartesian_trajectory_planner_B.time <=
         cartesian_trajectory_planner_B.tft)) {
      cartesian_trajectory_planner_B.maxval = cartesian_trajectory_planner_B.tft
        - cartesian_trajectory_planner_B.time;
      cartesian_trajectory_planner_B.s_i =
        cartesian_trajectory_planner_B.tetaFIN - 0.5 *
        cartesian_trajectory_planner_B.rtb_signal2_idx_2 *
        cartesian_trajectory_planner_B.maxval *
        cartesian_trajectory_planner_B.maxval;
      cartesian_trajectory_planner_B.teta_vel =
        cartesian_trajectory_planner_B.maxval *
        cartesian_trajectory_planner_B.rtb_signal2_idx_2;
    }

    if (cartesian_trajectory_planner_B.time > cartesian_trajectory_planner_B.tft)
    {
      cartesian_trajectory_planner_B.s_i =
        cartesian_trajectory_planner_B.tetaFIN;
      cartesian_trajectory_planner_B.teta_vel = 0.0;
    }
  }

  if (cartesian_trajectory_planner_B.SpMax <
      cartesian_trajectory_planner_B.tetaFIN) {
    cartesian_trajectory_planner_B.tft = (cartesian_trajectory_planner_B.tetaFIN
      - cartesian_trajectory_planner_B.rtb_signal2_idx_2 *
      cartesian_trajectory_planner_B.tc * cartesian_trajectory_planner_B.tc) *
      (1.0 / cartesian_trajectory_planner_B.rtb_signal2_idx_1) + 2.0 *
      cartesian_trajectory_planner_B.tc;
    if ((cartesian_trajectory_planner_B.time >= 0.0) &&
        (cartesian_trajectory_planner_B.time <=
         cartesian_trajectory_planner_B.tc)) {
      cartesian_trajectory_planner_B.s_i = 0.5 *
        cartesian_trajectory_planner_B.rtb_signal2_idx_2 *
        cartesian_trajectory_planner_B.time *
        cartesian_trajectory_planner_B.time;
      cartesian_trajectory_planner_B.teta_vel =
        cartesian_trajectory_planner_B.rtb_signal2_idx_2 *
        cartesian_trajectory_planner_B.time;
    }

    cartesian_trajectory_planner_B.maxval = cartesian_trajectory_planner_B.tft -
      cartesian_trajectory_planner_B.tc;
    if ((cartesian_trajectory_planner_B.time > cartesian_trajectory_planner_B.tc)
        && (cartesian_trajectory_planner_B.time <=
            cartesian_trajectory_planner_B.maxval)) {
      cartesian_trajectory_planner_B.s_i = (cartesian_trajectory_planner_B.time
        - cartesian_trajectory_planner_B.tc / 2.0) *
        (cartesian_trajectory_planner_B.rtb_signal2_idx_2 *
         cartesian_trajectory_planner_B.tc);
      cartesian_trajectory_planner_B.teta_vel =
        cartesian_trajectory_planner_B.rtb_signal2_idx_1;
    }

    if ((cartesian_trajectory_planner_B.time >
         cartesian_trajectory_planner_B.maxval) &&
        (cartesian_trajectory_planner_B.time <=
         cartesian_trajectory_planner_B.tft)) {
      cartesian_trajectory_planner_B.s_i =
        cartesian_trajectory_planner_B.tetaFIN - 0.5 *
        cartesian_trajectory_planner_B.rtb_signal2_idx_2 *
        (cartesian_trajectory_planner_B.tft -
         cartesian_trajectory_planner_B.time) *
        (cartesian_trajectory_planner_B.tft -
         cartesian_trajectory_planner_B.time);
      cartesian_trajectory_planner_B.teta_vel =
        (cartesian_trajectory_planner_B.tft -
         cartesian_trajectory_planner_B.time) *
        cartesian_trajectory_planner_B.rtb_signal2_idx_1;
    }

    if (cartesian_trajectory_planner_B.time > cartesian_trajectory_planner_B.tft)
    {
      cartesian_trajectory_planner_B.s_i =
        cartesian_trajectory_planner_B.tetaFIN;
      cartesian_trajectory_planner_B.teta_vel = 0.0;
    }
  }

  cartesian_trajectory_planner_B.Vel[0] = (cartesian_trajectory_planner_B.value
    - cartesian_trajectory_planner_B.value_o) *
    cartesian_trajectory_planner_B.s_vel;
  cartesian_trajectory_planner_B.Vel[3] =
    cartesian_trajectory_planner_B.teta_vel *
    cartesian_trajectory_planner_B.P_e[0];
  cartesian_trajectory_planner_B.Vel[1] =
    (cartesian_trajectory_planner_B.value_h -
     cartesian_trajectory_planner_B.value_c) *
    cartesian_trajectory_planner_B.s_vel;
  cartesian_trajectory_planner_B.Vel[4] =
    cartesian_trajectory_planner_B.teta_vel *
    cartesian_trajectory_planner_B.P_e[1];
  cartesian_trajectory_planner_B.Vel[2] =
    (cartesian_trajectory_planner_B.value_n -
     cartesian_trajectory_planner_B.value_b) *
    cartesian_trajectory_planner_B.s_vel;
  cartesian_trajectory_planner_B.Vel[5] =
    cartesian_trajectory_planner_B.teta_vel *
    cartesian_trajectory_planner_B.P_e[2];
  cartesian_trajectory_planner_B.time = cartesian_trajectory_planner_B.P_e[0];
  cartesian_trajectory_planner_B.rtb_signal2_idx_1 =
    cartesian_trajectory_planner_B.P_e[1];
  cartesian_trajectory_planner_B.rtb_signal2_idx_2 =
    cartesian_trajectory_planner_B.P_e[2];

  // MATLABSystem: '<Root>/Coordinate Transformation Conversion2' incorporates:
  //   MATLAB Function: '<Root>/Traslazione '

  cartesian_trajectory_planner_B.s_vel = 1.0 / sqrt
    ((cartesian_trajectory_planner_B.time * cartesian_trajectory_planner_B.time
      + cartesian_trajectory_planner_B.rtb_signal2_idx_1 *
      cartesian_trajectory_planner_B.rtb_signal2_idx_1) +
     cartesian_trajectory_planner_B.rtb_signal2_idx_2 *
     cartesian_trajectory_planner_B.rtb_signal2_idx_2);
  cartesian_trajectory_planner_B.P_e[0] = cartesian_trajectory_planner_B.time *
    cartesian_trajectory_planner_B.s_vel;
  cartesian_trajectory_planner_B.P_e[1] =
    cartesian_trajectory_planner_B.rtb_signal2_idx_1 *
    cartesian_trajectory_planner_B.s_vel;
  cartesian_trajectory_planner_B.P_e[2] =
    cartesian_trajectory_planner_B.rtb_signal2_idx_2 *
    cartesian_trajectory_planner_B.s_vel;
  cartesian_trajectory_planner_B.time = cos(cartesian_trajectory_planner_B.s_i);
  cartesian_trajectory_planner_B.rtb_signal2_idx_1 = sin
    (cartesian_trajectory_planner_B.s_i);
  cartesian_trajectory_planner_B.tempR[0] = cartesian_trajectory_planner_B.P_e[0]
    * cartesian_trajectory_planner_B.P_e[0] * (1.0 -
    cartesian_trajectory_planner_B.time) + cartesian_trajectory_planner_B.time;
  cartesian_trajectory_planner_B.s_vel = cartesian_trajectory_planner_B.P_e[1] *
    cartesian_trajectory_planner_B.P_e[0] * (1.0 -
    cartesian_trajectory_planner_B.time);
  cartesian_trajectory_planner_B.s_i = cartesian_trajectory_planner_B.P_e[2] *
    cartesian_trajectory_planner_B.rtb_signal2_idx_1;
  cartesian_trajectory_planner_B.tempR[1] = cartesian_trajectory_planner_B.s_vel
    - cartesian_trajectory_planner_B.s_i;
  cartesian_trajectory_planner_B.tcstar = cartesian_trajectory_planner_B.P_e[2] *
    cartesian_trajectory_planner_B.P_e[0] * (1.0 -
    cartesian_trajectory_planner_B.time);
  cartesian_trajectory_planner_B.tetaFIN = cartesian_trajectory_planner_B.P_e[1]
    * cartesian_trajectory_planner_B.rtb_signal2_idx_1;
  cartesian_trajectory_planner_B.tempR[2] =
    cartesian_trajectory_planner_B.tcstar +
    cartesian_trajectory_planner_B.tetaFIN;
  cartesian_trajectory_planner_B.tempR[3] = cartesian_trajectory_planner_B.s_vel
    + cartesian_trajectory_planner_B.s_i;
  cartesian_trajectory_planner_B.tempR[4] = cartesian_trajectory_planner_B.P_e[1]
    * cartesian_trajectory_planner_B.P_e[1] * (1.0 -
    cartesian_trajectory_planner_B.time) + cartesian_trajectory_planner_B.time;
  cartesian_trajectory_planner_B.s_vel = cartesian_trajectory_planner_B.P_e[2] *
    cartesian_trajectory_planner_B.P_e[1] * (1.0 -
    cartesian_trajectory_planner_B.time);
  cartesian_trajectory_planner_B.s_i = cartesian_trajectory_planner_B.P_e[0] *
    cartesian_trajectory_planner_B.rtb_signal2_idx_1;
  cartesian_trajectory_planner_B.tempR[5] = cartesian_trajectory_planner_B.s_vel
    - cartesian_trajectory_planner_B.s_i;
  cartesian_trajectory_planner_B.tempR[6] =
    cartesian_trajectory_planner_B.tcstar -
    cartesian_trajectory_planner_B.tetaFIN;
  cartesian_trajectory_planner_B.tempR[7] = cartesian_trajectory_planner_B.s_vel
    + cartesian_trajectory_planner_B.s_i;
  cartesian_trajectory_planner_B.tempR[8] = cartesian_trajectory_planner_B.P_e[2]
    * cartesian_trajectory_planner_B.P_e[2] * (1.0 -
    cartesian_trajectory_planner_B.time) + cartesian_trajectory_planner_B.time;
  for (cartesian_trajectory_planner_B.r1 = 0; cartesian_trajectory_planner_B.r1 <
       3; cartesian_trajectory_planner_B.r1++) {
    cartesian_trajectory_planner_B.r2 = cartesian_trajectory_planner_B.r1 + 1;
    cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.r2 - 1]
      = cartesian_trajectory_planner_B.tempR[(cartesian_trajectory_planner_B.r2
      - 1) * 3];
    cartesian_trajectory_planner_B.r2 = cartesian_trajectory_planner_B.r1 + 1;
    cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.r2 + 2]
      = cartesian_trajectory_planner_B.tempR[(cartesian_trajectory_planner_B.r2
      - 1) * 3 + 1];
    cartesian_trajectory_planner_B.r2 = cartesian_trajectory_planner_B.r1 + 1;
    cartesian_trajectory_planner_B.Matrot[cartesian_trajectory_planner_B.r2 + 5]
      = cartesian_trajectory_planner_B.tempR[(cartesian_trajectory_planner_B.r2
      - 1) * 3 + 2];
  }

  // MATLAB Function: '<Root>/Completa' incorporates:
  //   MATLABSystem: '<Root>/Coordinate Transformation Conversion'
  //   MATLABSystem: '<Root>/Coordinate Transformation Conversion2'

  for (cartesian_trajectory_planner_B.r1 = 0; cartesian_trajectory_planner_B.r1 <
       3; cartesian_trajectory_planner_B.r1++) {
    for (cartesian_trajectory_planner_B.r2 = 0;
         cartesian_trajectory_planner_B.r2 < 3;
         cartesian_trajectory_planner_B.r2++) {
      cartesian_trajectory_planner_B.r3 = cartesian_trajectory_planner_B.r2 +
        (cartesian_trajectory_planner_B.r1 << 2);
      cartesian_trajectory_planner_B.POSA[cartesian_trajectory_planner_B.r3] =
        0.0;
      cartesian_trajectory_planner_B.POSA[cartesian_trajectory_planner_B.r3] +=
        cartesian_trajectory_planner_B.Matrot[3 *
        cartesian_trajectory_planner_B.r1] *
        cartesian_trajectory_planner_B.out[cartesian_trajectory_planner_B.r2];
      cartesian_trajectory_planner_B.POSA[cartesian_trajectory_planner_B.r3] +=
        cartesian_trajectory_planner_B.Matrot[3 *
        cartesian_trajectory_planner_B.r1 + 1] *
        cartesian_trajectory_planner_B.out[cartesian_trajectory_planner_B.r2 + 4];
      cartesian_trajectory_planner_B.POSA[cartesian_trajectory_planner_B.r3] +=
        cartesian_trajectory_planner_B.Matrot[3 *
        cartesian_trajectory_planner_B.r1 + 2] *
        cartesian_trajectory_planner_B.out[cartesian_trajectory_planner_B.r2 + 8];
    }
  }

  // MATLABSystem: '<S5>/MATLAB System' incorporates:
  //   MATLABSystem: '<S11>/Get Parameter'
  //   MATLABSystem: '<S11>/Get Parameter1'
  //   MATLABSystem: '<S11>/Get Parameter2'
  //   MATLABSystem: '<S11>/Get Parameter3'
  //   MATLABSystem: '<S11>/Get Parameter4'
  //   MATLABSystem: '<S11>/Get Parameter5'

  ParamGet_cartesian_trajectory_planner_312.get_parameter
    (&cartesian_trajectory_planner_B.MatrixMultiply1[0]);
  ParamGet_cartesian_trajectory_planner_313.get_parameter
    (&cartesian_trajectory_planner_B.MatrixMultiply1[1]);
  ParamGet_cartesian_trajectory_planner_314.get_parameter
    (&cartesian_trajectory_planner_B.MatrixMultiply1[2]);
  ParamGet_cartesian_trajectory_planner_315.get_parameter
    (&cartesian_trajectory_planner_B.MatrixMultiply1[3]);
  ParamGet_cartesian_trajectory_planner_316.get_parameter
    (&cartesian_trajectory_planner_B.MatrixMultiply1[4]);
  ParamGet_cartesian_trajectory_planner_317.get_parameter
    (&cartesian_trajectory_planner_B.MatrixMultiply1[5]);
  obj = &cartesian_trajectory_planner_DW.obj;
  if (cartesian_trajectory_planner_DW.obj.IKInternal.isInitialized != 1) {
    cartesian_trajectory_planner_DW.obj.IKInternal.isSetupComplete = false;
    cartesian_trajectory_planner_DW.obj.IKInternal.isInitialized = 1;
    car_inverseKinematics_setupImpl
      (&cartesian_trajectory_planner_DW.obj.IKInternal,
       &cartesian_trajectory_planner_DW.gobj_85);
    obj->IKInternal.isSetupComplete = true;
  }

  cartesian_trajec_emxInit_real_T(&b, 2);
  cartesian_t_emxInit_f_cell_wrap(&obj_f, 2);

  // Delay: '<Root>/Delay'
  for (cartesian_trajectory_planner_B.r1 = 0; cartesian_trajectory_planner_B.r1 <
       6; cartesian_trajectory_planner_B.r1++) {
    cartesian_trajectory_planner_B.dv1[cartesian_trajectory_planner_B.r1] =
      cartesian_trajectory_planner_DW.Delay_DSTATE[cartesian_trajectory_planner_B.r1];
  }

  // MATLABSystem: '<S5>/MATLAB System' incorporates:
  //   Delay: '<Root>/Delay'

  cart_inverseKinematics_stepImpl(&obj->IKInternal,
    cartesian_trajectory_planner_B.POSA,
    cartesian_trajectory_planner_B.MatrixMultiply1,
    cartesian_trajectory_planner_B.dv1,
    cartesian_trajectory_planner_DW.Delay_DSTATE);

  // MATLABSystem: '<S3>/MATLAB System' incorporates:
  //   Delay: '<Root>/Delay'

  RigidBodyTree_geometricJacobian
    (&cartesian_trajectory_planner_DW.obj_g.TreeInternal,
     cartesian_trajectory_planner_DW.Delay_DSTATE, b);

  // MATLABSystem: '<S4>/MATLAB System' incorporates:
  //   Delay: '<Root>/Delay'

  RigidBodyTree_forwardKinemati_a
    (&cartesian_trajectory_planner_DW.obj_f.TreeInternal,
     cartesian_trajectory_planner_DW.Delay_DSTATE, obj_f);

  // Product: '<Root>/Reciprocal' incorporates:
  //   MATLABSystem: '<S3>/MATLAB System'

  rt_invd6x6_snf(b->data, cartesian_trajectory_planner_B.dv);
  cartesian_t_emxFree_f_cell_wrap(&obj_f);
  cartesian_trajec_emxFree_real_T(&b);

  // MATLAB Function: '<Root>/MATLAB Function' incorporates:
  //   Constant: '<S1>/Constant'

  cartesian_trajectory_planner_B.msg =
    cartesian_trajectory_planner_P.Constant_Value;
  cartesian_trajectory_planner_B.msg.Velocities_SL_Info.CurrentLength = 6U;
  cartesian_trajectory_planner_B.msg.Positions_SL_Info.CurrentLength = 6U;
  for (cartesian_trajectory_planner_B.r2 = 0; cartesian_trajectory_planner_B.r2 <
       6; cartesian_trajectory_planner_B.r2++) {
    // Product: '<Root>/MatrixMultiply1' incorporates:
    //   Product: '<Root>/Reciprocal'

    cartesian_trajectory_planner_B.MatrixMultiply1[cartesian_trajectory_planner_B.r2]
      = 0.0;
    for (cartesian_trajectory_planner_B.r1 = 0;
         cartesian_trajectory_planner_B.r1 < 6;
         cartesian_trajectory_planner_B.r1++) {
      cartesian_trajectory_planner_B.MatrixMultiply1[cartesian_trajectory_planner_B.r2]
        += cartesian_trajectory_planner_B.dv[6 *
        cartesian_trajectory_planner_B.r1 + cartesian_trajectory_planner_B.r2] *
        cartesian_trajectory_planner_B.Vel[cartesian_trajectory_planner_B.r1];
    }

    // End of Product: '<Root>/MatrixMultiply1'

    // MATLAB Function: '<Root>/MATLAB Function' incorporates:
    //   Delay: '<Root>/Delay'

    cartesian_trajectory_planner_B.msg.Velocities[cartesian_trajectory_planner_B.r2]
      =
      cartesian_trajectory_planner_B.MatrixMultiply1[cartesian_trajectory_planner_B.r2];
    cartesian_trajectory_planner_B.msg.Positions[cartesian_trajectory_planner_B.r2]
      =
      cartesian_trajectory_planner_DW.Delay_DSTATE[cartesian_trajectory_planner_B.r2];
  }

  // Outputs for Atomic SubSystem: '<Root>/Publish'
  // MATLABSystem: '<S8>/SinkBlock'
  Pub_cartesian_trajectory_planner_278.publish
    (&cartesian_trajectory_planner_B.msg);

  // End of Outputs for SubSystem: '<Root>/Publish'

  // MATLABSystem: '<S10>/Get Parameter'
  ParamGet_cartesian_trajectory_planner_283.get_parameter
    (&cartesian_trajectory_planner_B.rtb_signal2_idx_1);

  // MATLABSystem: '<S10>/Get Parameter1'
  ParamGet_cartesian_trajectory_planner_284.get_parameter
    (&cartesian_trajectory_planner_B.rtb_signal2_idx_1);
}

// Model initialize function
void cartesian_trajectory_planner_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    robotics_slmanip_internal__as_T *obj;
    b_inverseKinematics_cartesian_T *obj_0;
    h_robotics_core_internal_Damp_T *obj_1;
    static const char_T tmp[6] = { '/', 'c', 'l', 'o', 'c', 'k' };

    static const char_T tmp_0[17] = { '/', 'j', 'o', 'i', 'n', 't', '_', 't',
      'r', 'a', 'j', 'e', 'c', 't', 'o', 'r', 'y' };

    static const char_T tmp_1[5] = { '/', 'o', 'i', '_', 'x' };

    static const char_T tmp_2[5] = { '/', 'o', 'i', '_', 'y' };

    static const char_T tmp_3[5] = { '/', 'o', 'i', '_', 'z' };

    static const char_T tmp_4[5] = { '/', 'o', 'i', '_', 'w' };

    static const char_T tmp_5[5] = { '/', 'p', 'i', '_', 'x' };

    static const char_T tmp_6[5] = { '/', 'p', 'i', '_', 'y' };

    static const char_T tmp_7[5] = { '/', 'p', 'i', '_', 'z' };

    static const char_T tmp_8[5] = { '/', 'o', 'f', '_', 'x' };

    static const char_T tmp_9[5] = { '/', 'o', 'f', '_', 'y' };

    static const char_T tmp_a[5] = { '/', 'o', 'f', '_', 'z' };

    static const char_T tmp_b[5] = { '/', 'o', 'f', '_', 'w' };

    static const char_T tmp_c[5] = { '/', 'p', 'f', '_', 'x' };

    static const char_T tmp_d[5] = { '/', 'p', 'f', '_', 'y' };

    static const char_T tmp_e[5] = { '/', 'p', 'f', '_', 'z' };

    static const char_T tmp_f[10] = { '/', 'w', 'e', 'i', 'g', 'h', 't', '_',
      'o', 'x' };

    static const char_T tmp_g[10] = { '/', 'w', 'e', 'i', 'g', 'h', 't', '_',
      'o', 'y' };

    static const char_T tmp_h[10] = { '/', 'w', 'e', 'i', 'g', 'h', 't', '_',
      'o', 'z' };

    static const char_T tmp_i[10] = { '/', 'w', 'e', 'i', 'g', 'h', 't', '_',
      'p', 'x' };

    static const char_T tmp_j[10] = { '/', 'w', 'e', 'i', 'g', 'h', 't', '_',
      'p', 'y' };

    static const char_T tmp_k[10] = { '/', 'w', 'e', 'i', 'g', 'h', 't', '_',
      'p', 'z' };

    static const char_T tmp_l[12] = { '/', 's', 't', 'a', 'r', 't', '_', 'd',
      'e', 'l', 'a', 'y' };

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S9>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S13>/Out1'
    cartesian_trajectory_planner_B.In1 = cartesian_trajectory_planner_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S9>/Enabled Subsystem'

    // Start for MATLABSystem: '<S9>/SourceBlock'
    cartesian_trajectory_planner_DW.obj_gl.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_gl.isInitialized = 1;
    for (cartesian_trajectory_planner_B.i_jb = 0;
         cartesian_trajectory_planner_B.i_jb < 6;
         cartesian_trajectory_planner_B.i_jb++) {
      // InitializeConditions for Delay: '<Root>/Delay'
      cartesian_trajectory_planner_DW.Delay_DSTATE[cartesian_trajectory_planner_B.i_jb]
        =
        cartesian_trajectory_planner_P.Delay_InitialCondition[cartesian_trajectory_planner_B.i_jb];

      // Start for MATLABSystem: '<S9>/SourceBlock'
      cartesian_trajectory_planner_B.cv3[cartesian_trajectory_planner_B.i_jb] =
        tmp[cartesian_trajectory_planner_B.i_jb];
    }

    // Start for MATLABSystem: '<S9>/SourceBlock'
    cartesian_trajectory_planner_B.cv3[6] = '\x00';
    Sub_cartesian_trajectory_planner_280.createSubscriber
      (cartesian_trajectory_planner_B.cv3, 1);
    cartesian_trajectory_planner_DW.obj_gl.isSetupComplete = true;

    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish'
    // Start for MATLABSystem: '<S8>/SinkBlock'
    cartesian_trajectory_planner_DW.obj_n.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_n.isInitialized = 1;
    for (cartesian_trajectory_planner_B.i_jb = 0;
         cartesian_trajectory_planner_B.i_jb < 17;
         cartesian_trajectory_planner_B.i_jb++) {
      cartesian_trajectory_planner_B.cv[cartesian_trajectory_planner_B.i_jb] =
        tmp_0[cartesian_trajectory_planner_B.i_jb];
    }

    cartesian_trajectory_planner_B.cv[17] = '\x00';
    Pub_cartesian_trajectory_planner_278.createPublisher
      (cartesian_trajectory_planner_B.cv, 1);
    cartesian_trajectory_planner_DW.obj_n.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S8>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish'

    // Start for MATLABSystem: '<Root>/Get Parameter'
    cartesian_trajectory_planner_DW.obj_cu.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_cu.isInitialized = 1;
    cartesian_trajectory_planner_B.cv5[0] = '/';
    cartesian_trajectory_planner_B.cv5[1] = 'V';
    cartesian_trajectory_planner_B.cv5[2] = 'e';
    cartesian_trajectory_planner_B.cv5[3] = 'e';
    cartesian_trajectory_planner_B.cv5[4] = '\x00';
    ParamGet_cartesian_trajectory_planner_271.initialize
      (cartesian_trajectory_planner_B.cv5);
    ParamGet_cartesian_trajectory_planner_271.initialize_error_codes(0, 1, 2, 3);
    ParamGet_cartesian_trajectory_planner_271.set_initial_value(1.0);
    cartesian_trajectory_planner_DW.obj_cu.isSetupComplete = true;

    // Start for MATLABSystem: '<Root>/Get Parameter1'
    cartesian_trajectory_planner_DW.obj_k.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_k.isInitialized = 1;
    cartesian_trajectory_planner_B.cv5[0] = '/';
    cartesian_trajectory_planner_B.cv5[1] = 'A';
    cartesian_trajectory_planner_B.cv5[2] = 'e';
    cartesian_trajectory_planner_B.cv5[3] = 'e';
    cartesian_trajectory_planner_B.cv5[4] = '\x00';
    ParamGet_cartesian_trajectory_planner_272.initialize
      (cartesian_trajectory_planner_B.cv5);
    ParamGet_cartesian_trajectory_planner_272.initialize_error_codes(0, 1, 2, 3);
    ParamGet_cartesian_trajectory_planner_272.set_initial_value(0.1);
    cartesian_trajectory_planner_DW.obj_k.isSetupComplete = true;

    // Start for MATLABSystem: '<S14>/Get Parameter3'
    cartesian_trajectory_planner_DW.obj_es.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_es.isInitialized = 1;
    for (cartesian_trajectory_planner_B.i_jb = 0;
         cartesian_trajectory_planner_B.i_jb < 5;
         cartesian_trajectory_planner_B.i_jb++) {
      cartesian_trajectory_planner_B.cv4[cartesian_trajectory_planner_B.i_jb] =
        tmp_1[cartesian_trajectory_planner_B.i_jb];
    }

    cartesian_trajectory_planner_B.cv4[5] = '\x00';
    ParamGet_cartesian_trajectory_planner_291.initialize
      (cartesian_trajectory_planner_B.cv4);
    ParamGet_cartesian_trajectory_planner_291.initialize_error_codes(0, 1, 2, 3);
    ParamGet_cartesian_trajectory_planner_291.set_initial_value(0.0);
    cartesian_trajectory_planner_DW.obj_es.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S14>/Get Parameter3'

    // Start for MATLABSystem: '<S14>/Get Parameter4'
    cartesian_trajectory_planner_DW.obj_f4.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_f4.isInitialized = 1;
    for (cartesian_trajectory_planner_B.i_jb = 0;
         cartesian_trajectory_planner_B.i_jb < 5;
         cartesian_trajectory_planner_B.i_jb++) {
      cartesian_trajectory_planner_B.cv4[cartesian_trajectory_planner_B.i_jb] =
        tmp_2[cartesian_trajectory_planner_B.i_jb];
    }

    cartesian_trajectory_planner_B.cv4[5] = '\x00';
    ParamGet_cartesian_trajectory_planner_292.initialize
      (cartesian_trajectory_planner_B.cv4);
    ParamGet_cartesian_trajectory_planner_292.initialize_error_codes(0, 1, 2, 3);
    ParamGet_cartesian_trajectory_planner_292.set_initial_value(0.0);
    cartesian_trajectory_planner_DW.obj_f4.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S14>/Get Parameter4'

    // Start for MATLABSystem: '<S14>/Get Parameter5'
    cartesian_trajectory_planner_DW.obj_en.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_en.isInitialized = 1;
    for (cartesian_trajectory_planner_B.i_jb = 0;
         cartesian_trajectory_planner_B.i_jb < 5;
         cartesian_trajectory_planner_B.i_jb++) {
      cartesian_trajectory_planner_B.cv4[cartesian_trajectory_planner_B.i_jb] =
        tmp_3[cartesian_trajectory_planner_B.i_jb];
    }

    cartesian_trajectory_planner_B.cv4[5] = '\x00';
    ParamGet_cartesian_trajectory_planner_293.initialize
      (cartesian_trajectory_planner_B.cv4);
    ParamGet_cartesian_trajectory_planner_293.initialize_error_codes(0, 1, 2, 3);
    ParamGet_cartesian_trajectory_planner_293.set_initial_value(0.0);
    cartesian_trajectory_planner_DW.obj_en.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S14>/Get Parameter5'

    // Start for MATLABSystem: '<S14>/Get Parameter6'
    cartesian_trajectory_planner_DW.obj_l0.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_l0.isInitialized = 1;
    for (cartesian_trajectory_planner_B.i_jb = 0;
         cartesian_trajectory_planner_B.i_jb < 5;
         cartesian_trajectory_planner_B.i_jb++) {
      cartesian_trajectory_planner_B.cv4[cartesian_trajectory_planner_B.i_jb] =
        tmp_4[cartesian_trajectory_planner_B.i_jb];
    }

    cartesian_trajectory_planner_B.cv4[5] = '\x00';
    ParamGet_cartesian_trajectory_planner_294.initialize
      (cartesian_trajectory_planner_B.cv4);
    ParamGet_cartesian_trajectory_planner_294.initialize_error_codes(0, 1, 2, 3);
    ParamGet_cartesian_trajectory_planner_294.set_initial_value(1.0);
    cartesian_trajectory_planner_DW.obj_l0.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S14>/Get Parameter6'

    // Start for MATLABSystem: '<S14>/Get Parameter'
    cartesian_trajectory_planner_DW.obj_i.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_i.isInitialized = 1;
    for (cartesian_trajectory_planner_B.i_jb = 0;
         cartesian_trajectory_planner_B.i_jb < 5;
         cartesian_trajectory_planner_B.i_jb++) {
      cartesian_trajectory_planner_B.cv4[cartesian_trajectory_planner_B.i_jb] =
        tmp_5[cartesian_trajectory_planner_B.i_jb];
    }

    cartesian_trajectory_planner_B.cv4[5] = '\x00';
    ParamGet_cartesian_trajectory_planner_288.initialize
      (cartesian_trajectory_planner_B.cv4);
    ParamGet_cartesian_trajectory_planner_288.initialize_error_codes(0, 1, 2, 3);
    ParamGet_cartesian_trajectory_planner_288.set_initial_value(0.0);
    cartesian_trajectory_planner_DW.obj_i.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S14>/Get Parameter'

    // Start for MATLABSystem: '<S14>/Get Parameter1'
    cartesian_trajectory_planner_DW.obj_db.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_db.isInitialized = 1;
    for (cartesian_trajectory_planner_B.i_jb = 0;
         cartesian_trajectory_planner_B.i_jb < 5;
         cartesian_trajectory_planner_B.i_jb++) {
      cartesian_trajectory_planner_B.cv4[cartesian_trajectory_planner_B.i_jb] =
        tmp_6[cartesian_trajectory_planner_B.i_jb];
    }

    cartesian_trajectory_planner_B.cv4[5] = '\x00';
    ParamGet_cartesian_trajectory_planner_289.initialize
      (cartesian_trajectory_planner_B.cv4);
    ParamGet_cartesian_trajectory_planner_289.initialize_error_codes(0, 1, 2, 3);
    ParamGet_cartesian_trajectory_planner_289.set_initial_value(0.0);
    cartesian_trajectory_planner_DW.obj_db.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S14>/Get Parameter1'

    // Start for MATLABSystem: '<S14>/Get Parameter2'
    cartesian_trajectory_planner_DW.obj_oq.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_oq.isInitialized = 1;
    for (cartesian_trajectory_planner_B.i_jb = 0;
         cartesian_trajectory_planner_B.i_jb < 5;
         cartesian_trajectory_planner_B.i_jb++) {
      cartesian_trajectory_planner_B.cv4[cartesian_trajectory_planner_B.i_jb] =
        tmp_7[cartesian_trajectory_planner_B.i_jb];
    }

    cartesian_trajectory_planner_B.cv4[5] = '\x00';
    ParamGet_cartesian_trajectory_planner_290.initialize
      (cartesian_trajectory_planner_B.cv4);
    ParamGet_cartesian_trajectory_planner_290.initialize_error_codes(0, 1, 2, 3);
    ParamGet_cartesian_trajectory_planner_290.set_initial_value(0.99);
    cartesian_trajectory_planner_DW.obj_oq.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S14>/Get Parameter2'

    // Start for MATLABSystem: '<S15>/Get Parameter3'
    cartesian_trajectory_planner_DW.obj_e0.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_e0.isInitialized = 1;
    for (cartesian_trajectory_planner_B.i_jb = 0;
         cartesian_trajectory_planner_B.i_jb < 5;
         cartesian_trajectory_planner_B.i_jb++) {
      cartesian_trajectory_planner_B.cv4[cartesian_trajectory_planner_B.i_jb] =
        tmp_8[cartesian_trajectory_planner_B.i_jb];
    }

    cartesian_trajectory_planner_B.cv4[5] = '\x00';
    ParamGet_cartesian_trajectory_planner_303.initialize
      (cartesian_trajectory_planner_B.cv4);
    ParamGet_cartesian_trajectory_planner_303.initialize_error_codes(0, 1, 2, 3);
    ParamGet_cartesian_trajectory_planner_303.set_initial_value(0.39);
    cartesian_trajectory_planner_DW.obj_e0.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S15>/Get Parameter3'

    // Start for MATLABSystem: '<S15>/Get Parameter4'
    cartesian_trajectory_planner_DW.obj_a.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_a.isInitialized = 1;
    for (cartesian_trajectory_planner_B.i_jb = 0;
         cartesian_trajectory_planner_B.i_jb < 5;
         cartesian_trajectory_planner_B.i_jb++) {
      cartesian_trajectory_planner_B.cv4[cartesian_trajectory_planner_B.i_jb] =
        tmp_9[cartesian_trajectory_planner_B.i_jb];
    }

    cartesian_trajectory_planner_B.cv4[5] = '\x00';
    ParamGet_cartesian_trajectory_planner_304.initialize
      (cartesian_trajectory_planner_B.cv4);
    ParamGet_cartesian_trajectory_planner_304.initialize_error_codes(0, 1, 2, 3);
    ParamGet_cartesian_trajectory_planner_304.set_initial_value(0.89);
    cartesian_trajectory_planner_DW.obj_a.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S15>/Get Parameter4'

    // Start for MATLABSystem: '<S15>/Get Parameter5'
    cartesian_trajectory_planner_DW.obj_p.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_p.isInitialized = 1;
    for (cartesian_trajectory_planner_B.i_jb = 0;
         cartesian_trajectory_planner_B.i_jb < 5;
         cartesian_trajectory_planner_B.i_jb++) {
      cartesian_trajectory_planner_B.cv4[cartesian_trajectory_planner_B.i_jb] =
        tmp_a[cartesian_trajectory_planner_B.i_jb];
    }

    cartesian_trajectory_planner_B.cv4[5] = '\x00';
    ParamGet_cartesian_trajectory_planner_305.initialize
      (cartesian_trajectory_planner_B.cv4);
    ParamGet_cartesian_trajectory_planner_305.initialize_error_codes(0, 1, 2, 3);
    ParamGet_cartesian_trajectory_planner_305.set_initial_value(0.2);
    cartesian_trajectory_planner_DW.obj_p.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S15>/Get Parameter5'

    // Start for MATLABSystem: '<S15>/Get Parameter6'
    cartesian_trajectory_planner_DW.obj_l.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_l.isInitialized = 1;
    for (cartesian_trajectory_planner_B.i_jb = 0;
         cartesian_trajectory_planner_B.i_jb < 5;
         cartesian_trajectory_planner_B.i_jb++) {
      cartesian_trajectory_planner_B.cv4[cartesian_trajectory_planner_B.i_jb] =
        tmp_b[cartesian_trajectory_planner_B.i_jb];
    }

    cartesian_trajectory_planner_B.cv4[5] = '\x00';
    ParamGet_cartesian_trajectory_planner_306.initialize
      (cartesian_trajectory_planner_B.cv4);
    ParamGet_cartesian_trajectory_planner_306.initialize_error_codes(0, 1, 2, 3);
    ParamGet_cartesian_trajectory_planner_306.set_initial_value(-0.12);
    cartesian_trajectory_planner_DW.obj_l.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S15>/Get Parameter6'

    // Start for MATLABSystem: '<S15>/Get Parameter'
    cartesian_trajectory_planner_DW.obj_mb.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_mb.isInitialized = 1;
    for (cartesian_trajectory_planner_B.i_jb = 0;
         cartesian_trajectory_planner_B.i_jb < 5;
         cartesian_trajectory_planner_B.i_jb++) {
      cartesian_trajectory_planner_B.cv4[cartesian_trajectory_planner_B.i_jb] =
        tmp_c[cartesian_trajectory_planner_B.i_jb];
    }

    cartesian_trajectory_planner_B.cv4[5] = '\x00';
    ParamGet_cartesian_trajectory_planner_300.initialize
      (cartesian_trajectory_planner_B.cv4);
    ParamGet_cartesian_trajectory_planner_300.initialize_error_codes(0, 1, 2, 3);
    ParamGet_cartesian_trajectory_planner_300.set_initial_value(0.3);
    cartesian_trajectory_planner_DW.obj_mb.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S15>/Get Parameter'

    // Start for MATLABSystem: '<S15>/Get Parameter1'
    cartesian_trajectory_planner_DW.obj_c.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_c.isInitialized = 1;
    for (cartesian_trajectory_planner_B.i_jb = 0;
         cartesian_trajectory_planner_B.i_jb < 5;
         cartesian_trajectory_planner_B.i_jb++) {
      cartesian_trajectory_planner_B.cv4[cartesian_trajectory_planner_B.i_jb] =
        tmp_d[cartesian_trajectory_planner_B.i_jb];
    }

    cartesian_trajectory_planner_B.cv4[5] = '\x00';
    ParamGet_cartesian_trajectory_planner_301.initialize
      (cartesian_trajectory_planner_B.cv4);
    ParamGet_cartesian_trajectory_planner_301.initialize_error_codes(0, 1, 2, 3);
    ParamGet_cartesian_trajectory_planner_301.set_initial_value(0.3);
    cartesian_trajectory_planner_DW.obj_c.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S15>/Get Parameter1'

    // Start for MATLABSystem: '<S15>/Get Parameter2'
    cartesian_trajectory_planner_DW.obj_f2.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_f2.isInitialized = 1;
    for (cartesian_trajectory_planner_B.i_jb = 0;
         cartesian_trajectory_planner_B.i_jb < 5;
         cartesian_trajectory_planner_B.i_jb++) {
      cartesian_trajectory_planner_B.cv4[cartesian_trajectory_planner_B.i_jb] =
        tmp_e[cartesian_trajectory_planner_B.i_jb];
    }

    cartesian_trajectory_planner_B.cv4[5] = '\x00';
    ParamGet_cartesian_trajectory_planner_302.initialize
      (cartesian_trajectory_planner_B.cv4);
    ParamGet_cartesian_trajectory_planner_302.initialize_error_codes(0, 1, 2, 3);
    ParamGet_cartesian_trajectory_planner_302.set_initial_value(0.3);
    cartesian_trajectory_planner_DW.obj_f2.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S15>/Get Parameter2'

    // Start for MATLABSystem: '<S11>/Get Parameter'
    cartesian_trajectory_planner_DW.obj_e.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_e.isInitialized = 1;
    for (cartesian_trajectory_planner_B.i_jb = 0;
         cartesian_trajectory_planner_B.i_jb < 10;
         cartesian_trajectory_planner_B.i_jb++) {
      cartesian_trajectory_planner_B.cv2[cartesian_trajectory_planner_B.i_jb] =
        tmp_f[cartesian_trajectory_planner_B.i_jb];
    }

    cartesian_trajectory_planner_B.cv2[10] = '\x00';
    ParamGet_cartesian_trajectory_planner_312.initialize
      (cartesian_trajectory_planner_B.cv2);
    ParamGet_cartesian_trajectory_planner_312.initialize_error_codes(0, 1, 2, 3);
    ParamGet_cartesian_trajectory_planner_312.set_initial_value(0.0);
    cartesian_trajectory_planner_DW.obj_e.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter'

    // Start for MATLABSystem: '<S11>/Get Parameter1'
    cartesian_trajectory_planner_DW.obj_d.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_d.isInitialized = 1;
    for (cartesian_trajectory_planner_B.i_jb = 0;
         cartesian_trajectory_planner_B.i_jb < 10;
         cartesian_trajectory_planner_B.i_jb++) {
      cartesian_trajectory_planner_B.cv2[cartesian_trajectory_planner_B.i_jb] =
        tmp_g[cartesian_trajectory_planner_B.i_jb];
    }

    cartesian_trajectory_planner_B.cv2[10] = '\x00';
    ParamGet_cartesian_trajectory_planner_313.initialize
      (cartesian_trajectory_planner_B.cv2);
    ParamGet_cartesian_trajectory_planner_313.initialize_error_codes(0, 1, 2, 3);
    ParamGet_cartesian_trajectory_planner_313.set_initial_value(0.0);
    cartesian_trajectory_planner_DW.obj_d.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter1'

    // Start for MATLABSystem: '<S11>/Get Parameter2'
    cartesian_trajectory_planner_DW.obj_m.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_m.isInitialized = 1;
    for (cartesian_trajectory_planner_B.i_jb = 0;
         cartesian_trajectory_planner_B.i_jb < 10;
         cartesian_trajectory_planner_B.i_jb++) {
      cartesian_trajectory_planner_B.cv2[cartesian_trajectory_planner_B.i_jb] =
        tmp_h[cartesian_trajectory_planner_B.i_jb];
    }

    cartesian_trajectory_planner_B.cv2[10] = '\x00';
    ParamGet_cartesian_trajectory_planner_314.initialize
      (cartesian_trajectory_planner_B.cv2);
    ParamGet_cartesian_trajectory_planner_314.initialize_error_codes(0, 1, 2, 3);
    ParamGet_cartesian_trajectory_planner_314.set_initial_value(0.0);
    cartesian_trajectory_planner_DW.obj_m.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter2'

    // Start for MATLABSystem: '<S11>/Get Parameter3'
    cartesian_trajectory_planner_DW.obj_h.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_h.isInitialized = 1;
    for (cartesian_trajectory_planner_B.i_jb = 0;
         cartesian_trajectory_planner_B.i_jb < 10;
         cartesian_trajectory_planner_B.i_jb++) {
      cartesian_trajectory_planner_B.cv2[cartesian_trajectory_planner_B.i_jb] =
        tmp_i[cartesian_trajectory_planner_B.i_jb];
    }

    cartesian_trajectory_planner_B.cv2[10] = '\x00';
    ParamGet_cartesian_trajectory_planner_315.initialize
      (cartesian_trajectory_planner_B.cv2);
    ParamGet_cartesian_trajectory_planner_315.initialize_error_codes(0, 1, 2, 3);
    ParamGet_cartesian_trajectory_planner_315.set_initial_value(1.0);
    cartesian_trajectory_planner_DW.obj_h.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter3'

    // Start for MATLABSystem: '<S11>/Get Parameter4'
    cartesian_trajectory_planner_DW.obj_mf.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_mf.isInitialized = 1;
    for (cartesian_trajectory_planner_B.i_jb = 0;
         cartesian_trajectory_planner_B.i_jb < 10;
         cartesian_trajectory_planner_B.i_jb++) {
      cartesian_trajectory_planner_B.cv2[cartesian_trajectory_planner_B.i_jb] =
        tmp_j[cartesian_trajectory_planner_B.i_jb];
    }

    cartesian_trajectory_planner_B.cv2[10] = '\x00';
    ParamGet_cartesian_trajectory_planner_316.initialize
      (cartesian_trajectory_planner_B.cv2);
    ParamGet_cartesian_trajectory_planner_316.initialize_error_codes(0, 1, 2, 3);
    ParamGet_cartesian_trajectory_planner_316.set_initial_value(1.0);
    cartesian_trajectory_planner_DW.obj_mf.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter4'

    // Start for MATLABSystem: '<S11>/Get Parameter5'
    cartesian_trajectory_planner_DW.obj_o.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_o.isInitialized = 1;
    for (cartesian_trajectory_planner_B.i_jb = 0;
         cartesian_trajectory_planner_B.i_jb < 10;
         cartesian_trajectory_planner_B.i_jb++) {
      cartesian_trajectory_planner_B.cv2[cartesian_trajectory_planner_B.i_jb] =
        tmp_k[cartesian_trajectory_planner_B.i_jb];
    }

    cartesian_trajectory_planner_B.cv2[10] = '\x00';
    ParamGet_cartesian_trajectory_planner_317.initialize
      (cartesian_trajectory_planner_B.cv2);
    ParamGet_cartesian_trajectory_planner_317.initialize_error_codes(0, 1, 2, 3);
    ParamGet_cartesian_trajectory_planner_317.set_initial_value(1.0);
    cartesian_trajectory_planner_DW.obj_o.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/Get Parameter5'
    emxInitStruct_robotics_slmanip_(&cartesian_trajectory_planner_DW.obj);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_1);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_50);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_49);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_48);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_47);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_46);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_45);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_44);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_43);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_42);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_41);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_40);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_39);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_38);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_37);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_36);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_35);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_34);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_33);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_32);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_31);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_30);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_29);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_28);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_27);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_26);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_25);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_24);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_23);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_22);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_21);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_20);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_19);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_18);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_17);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_16);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_15);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_14);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_13);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_12);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_11);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_10);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_9);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_8);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_7);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_6);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_5);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_4);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_3);
    emxInitStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_2);
    emxInitStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_51);
    emxInitStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_82);
    emxInitStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_81);
    emxInitStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_80);
    emxInitStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_79);
    emxInitStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_78);
    emxInitStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_77);
    emxInitStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_76);
    emxInitStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_75);
    emxInitStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_74);
    emxInitStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_73);
    emxInitStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_72);
    emxInitStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_71);
    emxInitStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_70);
    emxInitStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_69);
    emxInitStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_68);
    emxInitStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_67);
    emxInitStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_66);
    emxInitStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_65);
    emxInitStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_64);
    emxInitStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_63);
    emxInitStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_62);
    emxInitStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_61);
    emxInitStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_60);
    emxInitStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_59);
    emxInitStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_58);
    emxInitStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_57);
    emxInitStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_56);
    emxInitStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_55);
    emxInitStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_54);
    emxInitStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_53);
    emxInitStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_52);
    emxInitStruct_x_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_83);
    emxInitStruct_x_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_84);
    emxInitStruct_f_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_85);
    emxInitStruct_h_robotics_core_i(&cartesian_trajectory_planner_DW.gobj_86);
    emxInitStruct_h_robotics_core_i(&cartesian_trajectory_planner_DW.gobj_87);
    emxInitStruct_v_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_88);
    emxInitStruct_v_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_103);
    emxInitStruct_v_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_102);
    emxInitStruct_v_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_101);
    emxInitStruct_v_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_100);
    emxInitStruct_v_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_99);
    emxInitStruct_v_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_98);
    emxInitStruct_v_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_97);
    emxInitStruct_v_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_96);
    emxInitStruct_v_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_95);
    emxInitStruct_v_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_94);
    emxInitStruct_v_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_93);
    emxInitStruct_v_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_92);
    emxInitStruct_v_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_91);
    emxInitStruct_v_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_90);
    emxInitStruct_v_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_89);

    // Start for MATLABSystem: '<S5>/MATLAB System'
    cartesian_trajectory_planner_DW.obj.IKInternal.matlabCodegenIsDeleted = true;
    cartesian_trajectory_planner_DW.obj.matlabCodegenIsDeleted = true;
    cartesian_tr_eml_rand_mt19937ar(cartesian_trajectory_planner_DW.state_m);
    cartesian_trajectory_planner_DW.obj.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj.matlabCodegenIsDeleted = false;
    obj = &cartesian_trajectory_planner_DW.obj;
    cartesian_trajectory_planner_DW.obj.isInitialized = 1;
    RigidBodyTree_RigidBodyTree_as
      (&cartesian_trajectory_planner_DW.obj.TreeInternal,
       &cartesian_trajectory_planner_DW.gobj_90,
       &cartesian_trajectory_planner_DW.gobj_91,
       &cartesian_trajectory_planner_DW.gobj_92,
       &cartesian_trajectory_planner_DW.gobj_93,
       &cartesian_trajectory_planner_DW.gobj_94,
       &cartesian_trajectory_planner_DW.gobj_95,
       &cartesian_trajectory_planner_DW.gobj_96,
       &cartesian_trajectory_planner_DW.gobj_89);
    obj_0 = &cartesian_trajectory_planner_DW.obj.IKInternal;
    obj_0->isInitialized = 0;
    inverseKinematics_set_RigidBody(obj_0, &obj->TreeInternal,
      &cartesian_trajectory_planner_DW.gobj_69,
      &cartesian_trajectory_planner_DW.gobj_70,
      &cartesian_trajectory_planner_DW.gobj_71,
      &cartesian_trajectory_planner_DW.gobj_72,
      &cartesian_trajectory_planner_DW.gobj_73,
      &cartesian_trajectory_planner_DW.gobj_74,
      &cartesian_trajectory_planner_DW.gobj_75,
      &cartesian_trajectory_planner_DW.gobj_76,
      &cartesian_trajectory_planner_DW.gobj_77,
      &cartesian_trajectory_planner_DW.gobj_78,
      &cartesian_trajectory_planner_DW.gobj_79,
      &cartesian_trajectory_planner_DW.gobj_80,
      &cartesian_trajectory_planner_DW.gobj_81,
      &cartesian_trajectory_planner_DW.gobj_82,
      &cartesian_trajectory_planner_DW.gobj_51,
      &cartesian_trajectory_planner_DW.gobj_28,
      &cartesian_trajectory_planner_DW.gobj_29,
      &cartesian_trajectory_planner_DW.gobj_30,
      &cartesian_trajectory_planner_DW.gobj_31,
      &cartesian_trajectory_planner_DW.gobj_32,
      &cartesian_trajectory_planner_DW.gobj_33,
      &cartesian_trajectory_planner_DW.gobj_34,
      &cartesian_trajectory_planner_DW.gobj_35,
      &cartesian_trajectory_planner_DW.gobj_36,
      &cartesian_trajectory_planner_DW.gobj_37,
      &cartesian_trajectory_planner_DW.gobj_38,
      &cartesian_trajectory_planner_DW.gobj_39,
      &cartesian_trajectory_planner_DW.gobj_40,
      &cartesian_trajectory_planner_DW.gobj_41,
      &cartesian_trajectory_planner_DW.gobj_42,
      &cartesian_trajectory_planner_DW.gobj_43,
      &cartesian_trajectory_planner_DW.gobj_44,
      &cartesian_trajectory_planner_DW.gobj_45,
      &cartesian_trajectory_planner_DW.gobj_46,
      &cartesian_trajectory_planner_DW.gobj_47,
      &cartesian_trajectory_planner_DW.gobj_48,
      &cartesian_trajectory_planner_DW.gobj_49,
      &cartesian_trajectory_planner_DW.gobj_50,
      &cartesian_trajectory_planner_DW.gobj_1,
      &cartesian_trajectory_planner_DW.gobj_27,
      &cartesian_trajectory_planner_DW.gobj_68,
      &cartesian_trajectory_planner_DW.gobj_83);
    obj_0->Solver = DampedBFGSwGradientProjection_D
      (&cartesian_trajectory_planner_DW.gobj_87);
    obj_1 = obj_0->Solver;
    obj_1->MaxNumIteration = 1500.0;
    obj_1->MaxTime = 10.0;
    obj_1->GradientTolerance = 1.0E-7;
    obj_1->SolutionTolerance = 1.0E-6;
    obj_1->ConstraintsOn = true;
    obj_1->RandomRestart = false;
    obj_1->StepTolerance = 1.0E-14;
    obj_0->matlabCodegenIsDeleted = false;
    emxInitStruct_robotics_slmani_a(&cartesian_trajectory_planner_DW.obj_g);
    emxInitStruct_n_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_1_h);
    emxInitStruct_n_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_16_c);
    emxInitStruct_n_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_15_g);
    emxInitStruct_n_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_14_h);
    emxInitStruct_n_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_13_a);
    emxInitStruct_n_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_12_b);
    emxInitStruct_n_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_11_i);
    emxInitStruct_n_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_10_f);
    emxInitStruct_n_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_9_g);
    emxInitStruct_n_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_8_d);
    emxInitStruct_n_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_7_b);
    emxInitStruct_n_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_6_a);
    emxInitStruct_n_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_5_g);
    emxInitStruct_n_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_4_j);
    emxInitStruct_n_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_3_l);
    emxInitStruct_n_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_2_m);

    // Start for MATLABSystem: '<S3>/MATLAB System'
    cartesian_trajectory_planner_DW.obj_g.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj_g.isInitialized = 1;
    car_RigidBodyTree_RigidBodyTree
      (&cartesian_trajectory_planner_DW.obj_g.TreeInternal,
       &cartesian_trajectory_planner_DW.gobj_2_m,
       &cartesian_trajectory_planner_DW.gobj_4_j,
       &cartesian_trajectory_planner_DW.gobj_5_g,
       &cartesian_trajectory_planner_DW.gobj_6_a,
       &cartesian_trajectory_planner_DW.gobj_7_b,
       &cartesian_trajectory_planner_DW.gobj_8_d,
       &cartesian_trajectory_planner_DW.gobj_9_g,
       &cartesian_trajectory_planner_DW.gobj_3_l);
    emxInitStruct_robotics_slman_as(&cartesian_trajectory_planner_DW.obj_f);
    emxInitStruct_n_robotics_mani_a(&cartesian_trajectory_planner_DW.gobj_1_f);
    emxInitStruct_n_robotics_mani_a(&cartesian_trajectory_planner_DW.gobj_16_k);
    emxInitStruct_n_robotics_mani_a(&cartesian_trajectory_planner_DW.gobj_15_l);
    emxInitStruct_n_robotics_mani_a(&cartesian_trajectory_planner_DW.gobj_14_hu);
    emxInitStruct_n_robotics_mani_a(&cartesian_trajectory_planner_DW.gobj_13_p);
    emxInitStruct_n_robotics_mani_a(&cartesian_trajectory_planner_DW.gobj_12_e);
    emxInitStruct_n_robotics_mani_a(&cartesian_trajectory_planner_DW.gobj_11_c);
    emxInitStruct_n_robotics_mani_a(&cartesian_trajectory_planner_DW.gobj_10_n);
    emxInitStruct_n_robotics_mani_a(&cartesian_trajectory_planner_DW.gobj_9_l);
    emxInitStruct_n_robotics_mani_a(&cartesian_trajectory_planner_DW.gobj_8_l);
    emxInitStruct_n_robotics_mani_a(&cartesian_trajectory_planner_DW.gobj_7_n);
    emxInitStruct_n_robotics_mani_a(&cartesian_trajectory_planner_DW.gobj_6_j);
    emxInitStruct_n_robotics_mani_a(&cartesian_trajectory_planner_DW.gobj_5_n);
    emxInitStruct_n_robotics_mani_a(&cartesian_trajectory_planner_DW.gobj_4_f);
    emxInitStruct_n_robotics_mani_a(&cartesian_trajectory_planner_DW.gobj_3_i);
    emxInitStruct_n_robotics_mani_a(&cartesian_trajectory_planner_DW.gobj_2_a);

    // Start for MATLABSystem: '<S4>/MATLAB System'
    cartesian_trajectory_planner_DW.obj_f.isInitialized = 0;
    cartesian_trajectory_planner_DW.obj_f.isInitialized = 1;
    c_RigidBodyTree_RigidBodyTree_a
      (&cartesian_trajectory_planner_DW.obj_f.TreeInternal,
       &cartesian_trajectory_planner_DW.gobj_2_a,
       &cartesian_trajectory_planner_DW.gobj_4_f,
       &cartesian_trajectory_planner_DW.gobj_5_n,
       &cartesian_trajectory_planner_DW.gobj_6_j,
       &cartesian_trajectory_planner_DW.gobj_7_n,
       &cartesian_trajectory_planner_DW.gobj_8_l,
       &cartesian_trajectory_planner_DW.gobj_9_l,
       &cartesian_trajectory_planner_DW.gobj_3_i);

    // Start for MATLABSystem: '<S10>/Get Parameter'
    cartesian_trajectory_planner_DW.obj_di.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_di.isInitialized = 1;
    for (cartesian_trajectory_planner_B.i_jb = 0;
         cartesian_trajectory_planner_B.i_jb < 12;
         cartesian_trajectory_planner_B.i_jb++) {
      cartesian_trajectory_planner_B.cv1[cartesian_trajectory_planner_B.i_jb] =
        tmp_l[cartesian_trajectory_planner_B.i_jb];
    }

    cartesian_trajectory_planner_B.cv1[12] = '\x00';
    ParamGet_cartesian_trajectory_planner_283.initialize
      (cartesian_trajectory_planner_B.cv1);
    ParamGet_cartesian_trajectory_planner_283.initialize_error_codes(0, 1, 2, 3);
    ParamGet_cartesian_trajectory_planner_283.set_initial_value(40.0);
    cartesian_trajectory_planner_DW.obj_di.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/Get Parameter'

    // Start for MATLABSystem: '<S10>/Get Parameter1'
    cartesian_trajectory_planner_DW.obj_cj.matlabCodegenIsDeleted = false;
    cartesian_trajectory_planner_DW.obj_cj.isInitialized = 1;
    cartesian_trajectory_planner_B.cv6[0] = '/';
    cartesian_trajectory_planner_B.cv6[1] = 't';
    cartesian_trajectory_planner_B.cv6[2] = 'f';
    cartesian_trajectory_planner_B.cv6[3] = '\x00';
    ParamGet_cartesian_trajectory_planner_284.initialize
      (cartesian_trajectory_planner_B.cv6);
    ParamGet_cartesian_trajectory_planner_284.initialize_error_codes(0, 1, 2, 3);
    ParamGet_cartesian_trajectory_planner_284.set_initial_value(70.0);
    cartesian_trajectory_planner_DW.obj_cj.isSetupComplete = true;
  }
}

// Model terminate function
void cartesian_trajectory_planner_terminate(void)
{
  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S9>/SourceBlock'
  matlabCodegenHandle_matla_astwh(&cartesian_trajectory_planner_DW.obj_gl);

  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for MATLABSystem: '<Root>/Get Parameter'
  matlabCodegenHandle_matlabCodeg(&cartesian_trajectory_planner_DW.obj_cu);

  // Terminate for MATLABSystem: '<Root>/Get Parameter1'
  matlabCodegenHandle_matlabCodeg(&cartesian_trajectory_planner_DW.obj_k);

  // Terminate for MATLABSystem: '<S14>/Get Parameter3'
  matlabCodegenHandle_matlabCodeg(&cartesian_trajectory_planner_DW.obj_es);

  // Terminate for MATLABSystem: '<S14>/Get Parameter4'
  matlabCodegenHandle_matlabCodeg(&cartesian_trajectory_planner_DW.obj_f4);

  // Terminate for MATLABSystem: '<S14>/Get Parameter5'
  matlabCodegenHandle_matlabCodeg(&cartesian_trajectory_planner_DW.obj_en);

  // Terminate for MATLABSystem: '<S14>/Get Parameter6'
  matlabCodegenHandle_matlabCodeg(&cartesian_trajectory_planner_DW.obj_l0);

  // Terminate for MATLABSystem: '<S14>/Get Parameter'
  matlabCodegenHandle_matlabCodeg(&cartesian_trajectory_planner_DW.obj_i);

  // Terminate for MATLABSystem: '<S14>/Get Parameter1'
  matlabCodegenHandle_matlabCodeg(&cartesian_trajectory_planner_DW.obj_db);

  // Terminate for MATLABSystem: '<S14>/Get Parameter2'
  matlabCodegenHandle_matlabCodeg(&cartesian_trajectory_planner_DW.obj_oq);

  // Terminate for MATLABSystem: '<S15>/Get Parameter3'
  matlabCodegenHandle_matlabCodeg(&cartesian_trajectory_planner_DW.obj_e0);

  // Terminate for MATLABSystem: '<S15>/Get Parameter4'
  matlabCodegenHandle_matlabCodeg(&cartesian_trajectory_planner_DW.obj_a);

  // Terminate for MATLABSystem: '<S15>/Get Parameter5'
  matlabCodegenHandle_matlabCodeg(&cartesian_trajectory_planner_DW.obj_p);

  // Terminate for MATLABSystem: '<S15>/Get Parameter6'
  matlabCodegenHandle_matlabCodeg(&cartesian_trajectory_planner_DW.obj_l);

  // Terminate for MATLABSystem: '<S15>/Get Parameter'
  matlabCodegenHandle_matlabCodeg(&cartesian_trajectory_planner_DW.obj_mb);

  // Terminate for MATLABSystem: '<S15>/Get Parameter1'
  matlabCodegenHandle_matlabCodeg(&cartesian_trajectory_planner_DW.obj_c);

  // Terminate for MATLABSystem: '<S15>/Get Parameter2'
  matlabCodegenHandle_matlabCodeg(&cartesian_trajectory_planner_DW.obj_f2);

  // Terminate for MATLABSystem: '<S11>/Get Parameter'
  matlabCodegenHandle_matlabCodeg(&cartesian_trajectory_planner_DW.obj_e);

  // Terminate for MATLABSystem: '<S11>/Get Parameter1'
  matlabCodegenHandle_matlabCodeg(&cartesian_trajectory_planner_DW.obj_d);

  // Terminate for MATLABSystem: '<S11>/Get Parameter2'
  matlabCodegenHandle_matlabCodeg(&cartesian_trajectory_planner_DW.obj_m);

  // Terminate for MATLABSystem: '<S11>/Get Parameter3'
  matlabCodegenHandle_matlabCodeg(&cartesian_trajectory_planner_DW.obj_h);

  // Terminate for MATLABSystem: '<S11>/Get Parameter4'
  matlabCodegenHandle_matlabCodeg(&cartesian_trajectory_planner_DW.obj_mf);

  // Terminate for MATLABSystem: '<S11>/Get Parameter5'
  matlabCodegenHandle_matlabCodeg(&cartesian_trajectory_planner_DW.obj_o);

  // Terminate for MATLABSystem: '<S5>/MATLAB System'
  matlabCodegenHandle_matlabC_ast(&cartesian_trajectory_planner_DW.obj);
  matlabCodegenHandle_matlabCo_as
    (&cartesian_trajectory_planner_DW.obj.IKInternal);
  emxFreeStruct_robotics_slmanip_(&cartesian_trajectory_planner_DW.obj);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_1);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_50);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_49);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_48);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_47);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_46);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_45);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_44);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_43);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_42);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_41);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_40);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_39);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_38);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_37);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_36);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_35);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_34);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_33);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_32);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_31);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_30);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_29);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_28);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_27);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_26);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_25);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_24);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_23);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_22);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_21);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_20);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_19);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_18);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_17);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_16);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_15);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_14);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_13);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_12);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_11);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_10);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_9);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_8);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_7);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_6);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_5);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_4);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_3);
  emxFreeStruct_c_rigidBodyJoint(&cartesian_trajectory_planner_DW.gobj_2);
  emxFreeStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_51);
  emxFreeStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_82);
  emxFreeStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_81);
  emxFreeStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_80);
  emxFreeStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_79);
  emxFreeStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_78);
  emxFreeStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_77);
  emxFreeStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_76);
  emxFreeStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_75);
  emxFreeStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_74);
  emxFreeStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_73);
  emxFreeStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_72);
  emxFreeStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_71);
  emxFreeStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_70);
  emxFreeStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_69);
  emxFreeStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_68);
  emxFreeStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_67);
  emxFreeStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_66);
  emxFreeStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_65);
  emxFreeStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_64);
  emxFreeStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_63);
  emxFreeStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_62);
  emxFreeStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_61);
  emxFreeStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_60);
  emxFreeStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_59);
  emxFreeStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_58);
  emxFreeStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_57);
  emxFreeStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_56);
  emxFreeStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_55);
  emxFreeStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_54);
  emxFreeStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_53);
  emxFreeStruct_w_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_52);
  emxFreeStruct_x_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_83);
  emxFreeStruct_x_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_84);
  emxFreeStruct_f_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_85);
  emxFreeStruct_h_robotics_core_i(&cartesian_trajectory_planner_DW.gobj_86);
  emxFreeStruct_h_robotics_core_i(&cartesian_trajectory_planner_DW.gobj_87);
  emxFreeStruct_v_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_88);
  emxFreeStruct_v_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_103);
  emxFreeStruct_v_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_102);
  emxFreeStruct_v_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_101);
  emxFreeStruct_v_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_100);
  emxFreeStruct_v_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_99);
  emxFreeStruct_v_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_98);
  emxFreeStruct_v_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_97);
  emxFreeStruct_v_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_96);
  emxFreeStruct_v_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_95);
  emxFreeStruct_v_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_94);
  emxFreeStruct_v_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_93);
  emxFreeStruct_v_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_92);
  emxFreeStruct_v_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_91);
  emxFreeStruct_v_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_90);
  emxFreeStruct_v_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_89);
  emxFreeStruct_robotics_slmani_a(&cartesian_trajectory_planner_DW.obj_g);
  emxFreeStruct_n_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_1_h);
  emxFreeStruct_n_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_16_c);
  emxFreeStruct_n_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_15_g);
  emxFreeStruct_n_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_14_h);
  emxFreeStruct_n_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_13_a);
  emxFreeStruct_n_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_12_b);
  emxFreeStruct_n_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_11_i);
  emxFreeStruct_n_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_10_f);
  emxFreeStruct_n_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_9_g);
  emxFreeStruct_n_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_8_d);
  emxFreeStruct_n_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_7_b);
  emxFreeStruct_n_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_6_a);
  emxFreeStruct_n_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_5_g);
  emxFreeStruct_n_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_4_j);
  emxFreeStruct_n_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_3_l);
  emxFreeStruct_n_robotics_manip_(&cartesian_trajectory_planner_DW.gobj_2_m);
  emxFreeStruct_robotics_slman_as(&cartesian_trajectory_planner_DW.obj_f);
  emxFreeStruct_n_robotics_mani_a(&cartesian_trajectory_planner_DW.gobj_1_f);
  emxFreeStruct_n_robotics_mani_a(&cartesian_trajectory_planner_DW.gobj_16_k);
  emxFreeStruct_n_robotics_mani_a(&cartesian_trajectory_planner_DW.gobj_15_l);
  emxFreeStruct_n_robotics_mani_a(&cartesian_trajectory_planner_DW.gobj_14_hu);
  emxFreeStruct_n_robotics_mani_a(&cartesian_trajectory_planner_DW.gobj_13_p);
  emxFreeStruct_n_robotics_mani_a(&cartesian_trajectory_planner_DW.gobj_12_e);
  emxFreeStruct_n_robotics_mani_a(&cartesian_trajectory_planner_DW.gobj_11_c);
  emxFreeStruct_n_robotics_mani_a(&cartesian_trajectory_planner_DW.gobj_10_n);
  emxFreeStruct_n_robotics_mani_a(&cartesian_trajectory_planner_DW.gobj_9_l);
  emxFreeStruct_n_robotics_mani_a(&cartesian_trajectory_planner_DW.gobj_8_l);
  emxFreeStruct_n_robotics_mani_a(&cartesian_trajectory_planner_DW.gobj_7_n);
  emxFreeStruct_n_robotics_mani_a(&cartesian_trajectory_planner_DW.gobj_6_j);
  emxFreeStruct_n_robotics_mani_a(&cartesian_trajectory_planner_DW.gobj_5_n);
  emxFreeStruct_n_robotics_mani_a(&cartesian_trajectory_planner_DW.gobj_4_f);
  emxFreeStruct_n_robotics_mani_a(&cartesian_trajectory_planner_DW.gobj_3_i);
  emxFreeStruct_n_robotics_mani_a(&cartesian_trajectory_planner_DW.gobj_2_a);

  // Terminate for Atomic SubSystem: '<Root>/Publish'
  // Terminate for MATLABSystem: '<S8>/SinkBlock'
  matlabCodegenHandle_matlab_astw(&cartesian_trajectory_planner_DW.obj_n);

  // End of Terminate for SubSystem: '<Root>/Publish'

  // Terminate for MATLABSystem: '<S10>/Get Parameter'
  matlabCodegenHandle_matlabCodeg(&cartesian_trajectory_planner_DW.obj_di);

  // Terminate for MATLABSystem: '<S10>/Get Parameter1'
  matlabCodegenHandle_matlabCodeg(&cartesian_trajectory_planner_DW.obj_cj);
}

//
// File trailer for generated code.
//
// [EOF]
//
